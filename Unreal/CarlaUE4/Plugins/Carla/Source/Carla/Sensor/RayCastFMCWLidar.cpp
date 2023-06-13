// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Sensor/RayCastFMCWLidar.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"

#include <unordered_map>
#include <vector>

namespace crp = carla::rpc;

FActorDefinition ARayCastFMCWLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast_fmcw"));
}

ARayCastFMCWLidar::ARayCastFMCWLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
}

void ARayCastFMCWLidar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription);
  Set(LidarDescription);
}

void ARayCastFMCWLidar::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  FMCWLidarData = FFMCWLidarData(Description.Channels);
  CreateLasers();
  PointsPerChannel.resize(Description.Channels);
}

void ARayCastFMCWLidar::CreateLasers()
{
  const auto NumberOfLasers = Description.Channels;
  check(NumberOfLasers > 0u);
  const float DeltaAngle = NumberOfLasers == 1u ? 0.f :
    (Description.UpperFovLimit - Description.LowerFovLimit) /
    static_cast<float>(NumberOfLasers - 1);
  LaserAngles.Empty(NumberOfLasers);
  for(auto i = 0u; i < NumberOfLasers; ++i)
  {
    const float VerticalAngle =
        Description.UpperFovLimit - static_cast<float>(i) * DeltaAngle;
    LaserAngles.Emplace(VerticalAngle);
  }
}

void ARayCastFMCWLidar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastFMCWLidar::PostPhysTick);
  SimulateLidar(DeltaTime);

  CalculateCurrentVelocity(DeltaTime);

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    auto DataStream = GetDataStream(*this);
    DataStream.Send(*this, FMCWLidarData, DataStream.PopBufferFromPool());
  }
}

void ARayCastFMCWLidar::SimulateLidar(const float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastFMCWLidar::SimulateLidar);
  const uint32 ChannelCount = Description.Channels;
  // How many Laser scaned in a Tick/DeltaTime.
  const uint32 PointsToScanWithOneLaser =
    FMath::RoundHalfFromZero(
        Description.PointsPerSecond * DeltaTime / float(ChannelCount));

  if (PointsToScanWithOneLaser <= 0)
  {
    UE_LOG(
        LogCarla,
        Warning,
        TEXT("%s: no points requested this frame, try increasing the number of points per second."),
        *GetName());
    return;
  }

  check(ChannelCount == LaserAngles.Num());
  // The scan region of FMCW Lidar.(Horizental Angle)
  const float ScanAngleRange = Description.RightFovLimit - Description.LeftFovLimit;

  // const float CurrentHorizontalAngle = carla::geom::Math::ToDegrees(FMCWLidarData.GetHorizontalAngle());
  // const float AngleDistanceOfTick = Description.RotationFrequency * Description.HorizontalFov* DeltaTime;
  // const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;

  // Current Horizontal Angle
  const float CurrentHorizontalAngle = Description.LeftFovLimit;
  // The total angle scaned in a Tick/DeltaTime.
  // 1 / RotationFrequency means a scan form left side to right side.
  const float AngleDistanceOfTick = Description.RotationFrequency * ScanAngleRange * DeltaTime;
  // The angle between two adjacent Point in the channel
  const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / (PointsToScanWithOneLaser - 1);
  
  ResetRecordedHits(ChannelCount, PointsToScanWithOneLaser);
  PreprocessRays(ChannelCount, PointsToScanWithOneLaser);

  GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
  {
    TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
    ParallelFor(ChannelCount, [&](int32 idxChannel) {
      TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);

      FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
      TraceParams.bTraceComplex = true;
      TraceParams.bReturnPhysicalMaterial = false;

      for (auto idxPtsOneLaser = 0u; idxPtsOneLaser < PointsToScanWithOneLaser; idxPtsOneLaser++) {
        FHitResult HitResult;
        const float VertAngle = LaserAngles[idxChannel];
        const float HorizAngle = Description.LeftFovLimit + AngleDistanceOfLaserMeasure * idxPtsOneLaser;
        // const float HorizAngle = std::fmod(CurrentHorizontalAngle + AngleDistanceOfLaserMeasure * idxPtsOneLaser, Description.HorizontalFov) - Description.HorizontalFov / 2;
        const bool PreprocessResult = RayPreprocessCondition[idxChannel][idxPtsOneLaser];

        // If hit something, write it to RecordedHits(global variable).
        if (PreprocessResult && ShootLaser(VertAngle, HorizAngle, HitResult, TraceParams)) {
          WritePointAsync(idxChannel, HitResult);
        }
      };
    });
  }
  GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

  // Compute and save detection.
  FTransform ActorTransf = GetTransform();
  ComputeAndSaveDetections(ActorTransf, DeltaTime, AngleDistanceOfLaserMeasure);

  // // Set new current horizontal angle after a tick(Modulus Operation by ScanAngleRange).
  // // const float HorizontalAngle = carla::geom::Math::ToRadians(std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, Description.HorizontalFov));
  // const float HorizontalAngle = carla::geom::Math::ToRadians(Description.LeftFovLimit + AngleDistanceOfTick );
  // // const float HorizontalAngle = carla::geom::Math::ToRadians(Description.LeftFovLimit);
  // FMCWLidarData.SetHorizontalAngle(HorizontalAngle);
}

// Clear the recorded data structure.
void ARayCastFMCWLidar::ResetRecordedHits(uint32_t Channels, uint32_t MaxPointsPerChannel) {
  RecordedHits.resize(Channels);

  for (auto& hits : RecordedHits) {
    hits.clear();
    hits.reserve(MaxPointsPerChannel);
  }
}

void ARayCastFMCWLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
  RayPreprocessCondition.resize(Channels);

  for (auto& conds : RayPreprocessCondition) {
    conds.clear();
    conds.resize(MaxPointsPerChannel);
    std::fill(conds.begin(), conds.end(), true);
  }
}

// Saving the hits the raycast returns per channel
// Write the detection of the specific channel to RecordedHits(global variable).
void ARayCastFMCWLidar::WritePointAsync(uint32_t channel, FHitResult &detection) {
	TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
  DEBUG_ASSERT(GetChannelCount() > channel);
  RecordedHits[channel].emplace_back(detection);
}

// This method uses all the saved FHitResults, compute the
// RawDetections and then send it to the LidarData structure.
void ARayCastFMCWLidar::ComputeAndSaveDetections(const FTransform& SensorTransform, const float DeltaTime, const float DeltaAngleOfOneLaser) {
	TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
    PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();
  FMCWLidarData.ResetMemory(PointsPerChannel);  // Clear the space waiting for next writing

  //=================================================================================================
  // const FVector TargetVelocity = actor->GetVelocity();
  // Get Lidar location and rpy
  FTransform ActorTransf = GetTransform();  // Lidar Actor transform
  // FVector LidarBodyLoc = ActorTransf.GetLocation(); // Lidar location
  FRotator LidarBodyRot = ActorTransf.Rotator();  // Lidar rotation

  //=================================================================================================
  // Transform axis from world to lidar-----------------------------------------------------------------
  Eigen::Matrix3f LidarAxisRot; 
  LidarAxisRot = Eigen::AngleAxisf(carla::geom::Math::ToRadians(LidarBodyRot.Yaw), Eigen::Vector3f::UnitZ()) * 
            Eigen::AngleAxisf(carla::geom::Math::ToRadians(LidarBodyRot.Pitch), Eigen::Vector3f::UnitY()) * 
            Eigen::AngleAxisf(carla::geom::Math::ToRadians(LidarBodyRot.Roll), Eigen::Vector3f::UnitX());

  //=================================================================================================
  // // Convert to radians if required
  // float r = carla::geom::Math::ToRadians(LidarBodyRot.Roll);
  // float p = carla::geom::Math::ToRadians(LidarBodyRot.Pitch);
  // float y = carla::geom::Math::ToRadians(LidarBodyRot.Yaw);

  // // Define the rotation matrix
  // float c1 = cosf(r), s1 = sinf(r);
  // float c2 = cosf(p), s2 = sinf(p);
  // float c3 = cosf(y), s3 = sinf(y);
  // std::vector<std::vector<float>> LidarAxisRot = {
  //           {c1*c3+s1*s2*s3 , -c1*s3+s1*s2*c3, s1*c2},
  //           {c2*s3          , c2*c3          , -s2  },
  //           {-s1*c3+c1*s2*s3, s1*s3+c1*s2*c3 , c1*c2}};
  //=================================================================================================

  std::unordered_map<int, std::vector<float>> ActorInfoMap;
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
    for (auto& hit : RecordedHits[idxChannel]) {
      FFMCWDetection detection;
      // Compute Raw Detection.
      // Note: The order of idxChannel is from upper to lower while ring is from lower to upper.
      ComputeRawDetection(hit, SensorTransform, detection, Description.Channels-idxChannel-1, LidarAxisRot, &ActorInfoMap);
      int PointNum = static_cast<int>((Description.RightFovLimit - carla::geom::Math::ToDegrees(atan2(detection.point.y, detection.point.x))) / DeltaAngleOfOneLaser);
      detection.column = PointNum;
      // Add a time fluctuation
      // detection.time = -DeltaTime + PointToPointScanDeltaTime * (idxChannel + PointNum * Description.Channels);
      // detection.time = -DeltaTime + PointToPointScanDeltaTime * (idxChannel + PointNum * Description.Channels) + (rand()%52-26)*1e-7;
      detection.time = 0.0;

      // if(PostprocessDetection(detection))  //??
      FMCWLidarData.WritePointSync(detection);
    }
  }

  FMCWLidarData.WriteChannelCount(PointsPerChannel);
}

void ARayCastFMCWLidar::ComputeRawDetection(const FHitResult& HitInfo, const FTransform& SensorTransf, FFMCWDetection& Detection, uint32_t Channel, Eigen::Matrix3f LidarAxisRot, std::unordered_map<int, std::vector<float>>* ActorInfoMap) const
{
  const FVector HitPoint = HitInfo.ImpactPoint;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

  // const FVector VecInc = - (HitPoint - SensorTransf.GetLocation()).GetSafeNormal();
  // Detection.cos_inc_angle = FVector::DotProduct(VecInc, HitInfo.ImpactNormal);

  const FActorRegistry &Registry = GetEpisode().GetActorRegistry();

  const AActor* actor = HitInfo.Actor.Get();
  Detection.object_idx = 0;
  Detection.object_tag = static_cast<uint32_t>(HitInfo.Component->CustomDepthStencilValue);
  
  Detection.bbox_x = 0;
  Detection.bbox_y = 0;
  Detection.bbox_z = 0;
  Detection.bbox_w = 0;
  Detection.bbox_l = 0;
  Detection.bbox_h = 0;
  Detection.rotation_z = 0;

  //=================================================================================================
  // const FVector TargetVelocity = actor->GetVelocity();
  // Get Lidar location and rpy
  FTransform ActorTransf = GetTransform();  // Lidar Actor transform
  FVector LidarBodyLoc = ActorTransf.GetLocation(); // Lidar location
  FRotator LidarBodyRot = ActorTransf.Rotator();  // Lidar rotation
  
  if (actor != nullptr) {

    const FCarlaActor* view = Registry.FindCarlaActor(actor);
    if(view)
      Detection.object_idx = view->GetActorId();

    //===================================================================================================
    if(ActorInfoMap->count(Detection.object_idx) == 0){
      // Get object's Location and Rotation
      FTransform HitActorTransf = actor->GetTransform();
      FVector HitActorLoc = HitActorTransf.GetLocation();
      FRotator HitActorRot = HitActorTransf.Rotator();

      //----------------------------------------------------------------------------------------------------
      // Caculate the relative location between Lidar and object
      Eigen::Vector3f ActorRelativeLocation;
      ActorRelativeLocation(0) = HitActorLoc.X - LidarBodyLoc.X;
      ActorRelativeLocation(1) = HitActorLoc.Y - LidarBodyLoc.Y;
      ActorRelativeLocation(2) = HitActorLoc.Z - LidarBodyLoc.Z;

      // 因为carla的坐标是左手坐标系
      Eigen::Matrix3f I;
      I << 1, 0, 0, 0, -1, 0, 0, 0, 1;
      // 转换到雷达坐标系，加个inverse
      Eigen::Matrix3f LidarAxisRot_inv = LidarAxisRot.inverse();
      ActorRelativeLocation = I * LidarAxisRot_inv * ActorRelativeLocation; 
      // 地图内的激光雷达的rpy可能误差影响有点大，所以这里直接用z轴相减的数据，但是实际中应该使用rpy旋转矩阵的数据
      ActorRelativeLocation(2) = HitActorLoc.Z - LidarBodyLoc.Z;
      //----------------------------------------------------------------------------------------------------
      // float AbsoluteLoc_X = HitActorLoc.X - LidarBodyLoc.X;
      // float AbsoluteLoc_Y = LidarBodyLoc.Y - HitActorLoc.Y;
      // float AbsoluteLoc_Z = HitActorLoc.Z - LidarBodyLoc.Z;
      // float ActorRelativeLocation_X = LidarAxisRot[0][0]*(AbsoluteLoc_X) + LidarAxisRot[0][1]*(AbsoluteLoc_Y) + LidarAxisRot[0][2]*(AbsoluteLoc_Z);
      // float ActorRelativeLocation_Y = LidarAxisRot[1][0]*(AbsoluteLoc_X) + LidarAxisRot[1][1]*(AbsoluteLoc_Y) + LidarAxisRot[1][2]*(AbsoluteLoc_Z);
      // float ActorRelativeLocation_Z = LidarAxisRot[2][0]*(AbsoluteLoc_X) + LidarAxisRot[2][1]*(AbsoluteLoc_Y) + LidarAxisRot[2][2]*(AbsoluteLoc_Z);

      //----------------------------------------------------------------------------------------------------

      constexpr float TO_METERS = 1e-2;
      // Detection.bbox_x = ActorRelativeLocation_X * TO_METERS;
      // Detection.bbox_y = ActorRelativeLocation_Y * TO_METERS;
      // Detection.bbox_z = ActorRelativeLocation_Z * TO_METERS;
      Detection.bbox_x = ActorRelativeLocation(0) * TO_METERS;
      Detection.bbox_y = ActorRelativeLocation(1) * TO_METERS;
      Detection.bbox_z = ActorRelativeLocation(2) * TO_METERS;
      FBoundingBox BoundingBox = UBoundingBoxCalculator::GetActorBoundingBox(actor);
      Detection.bbox_w = BoundingBox.Extent.Y*2;
      Detection.bbox_l = BoundingBox.Extent.X*2;
      Detection.bbox_h = BoundingBox.Extent.Z*2;

      // rotarion_z
      float actor_r_y = HitActorRot.Yaw;
      if(actor_r_y < 0) actor_r_y += 360.0;
      float self_r_y = LidarBodyRot.Yaw;
      if(self_r_y < 0) self_r_y += 360.0; 
      float ActorRelativeRPY = self_r_y - actor_r_y + 90.0;
      if(ActorRelativeRPY < 0) ActorRelativeRPY += 360.0;
      Detection.rotation_z = ActorRelativeRPY;
      // Detection.rotation_z = ActorInfoMap->size();
      
      std::vector<float> ActorInfo;
      ActorInfo.push_back(Detection.bbox_x);
      ActorInfo.push_back(Detection.bbox_y);
      ActorInfo.push_back(Detection.bbox_z);
      ActorInfo.push_back(Detection.bbox_w);
      ActorInfo.push_back(Detection.bbox_l);
      ActorInfo.push_back(Detection.bbox_h);
      ActorInfo.push_back(Detection.rotation_z);
      // ActorInfoMap[Detection.object_idx] = ActorInfo;
      ActorInfoMap->insert(std::unordered_map<int, std::vector<float>>::value_type(Detection.object_idx, ActorInfo));

    }else{
      // std::vector<float>& ActorInfo = ActorInfoMap[Detection.object_idx];

      std::vector<float>& ActorInfo = ActorInfoMap->at(Detection.object_idx);
      Detection.bbox_x = ActorInfo[0];
      Detection.bbox_y = ActorInfo[1];
      Detection.bbox_z = ActorInfo[2];
      Detection.bbox_w = ActorInfo[3];
      Detection.bbox_l = ActorInfo[4];
      Detection.bbox_h = ActorInfo[5];
      Detection.rotation_z = ActorInfo[6];
      // Detection.rotation_z = ActorInfoMap->size();

    }
    //===================================================================================================
  }
  else {
    UE_LOG(LogCarla, Warning, TEXT("Actor not valid %p!!!!"), actor);
  }

  // Get the radial/relative velocity.
  const float RelativeVelocity = CalculateRelativeVelocity(HitInfo, LidarBodyLoc);
  Detection.velocity = RelativeVelocity;

  // Get the intensity.
  // const float Distance = Detection.point.Length();
  // const float AttenAtm = Description.AtmospAttenRate;
  // const float AbsAtm = exp(-AttenAtm * Distance);
  // const float IntRec = AbsAtm;
  // Detection.intensity = IntRec;

  // ring and time
  Detection.ring = Channel; // Note: The order of idxChannel is from upper to lower while ring is from lower to upper.
  // Detection.time = FPlatformTime::Seconds();

}


bool ARayCastFMCWLidar::ShootLaser(const float VerticalAngle, const float HorizontalAngle, FHitResult& HitResult, FCollisionQueryParams& TraceParams) const
{
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

  FHitResult HitInfo(ForceInit);

  FTransform ActorTransf = GetTransform();
  FVector LidarBodyLoc = ActorTransf.GetLocation();
  FRotator LidarBodyRot = ActorTransf.Rotator();

  FRotator LaserRot (VerticalAngle, HorizontalAngle, 0);  // float InPitch, float InYaw, float InRoll
  FRotator ResultRot = UKismetMathLibrary::ComposeRotators(
    LaserRot,
    LidarBodyRot
  );

  const auto Range = Description.Range;
  FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;

  GetWorld()->ParallelLineTraceSingleByChannel(
    HitInfo,
    LidarBodyLoc,
    EndTrace,
    ECC_GameTraceChannel2,
    TraceParams,
    FCollisionResponseParams::DefaultResponseParam
  );

  if (HitInfo.bBlockingHit) {
    HitResult = HitInfo;
    return true;
  } else {
    return false;
  }
}

void ARayCastFMCWLidar::CalculateCurrentVelocity(const float DeltaTime)
{
  const FVector LidarLocation = GetActorLocation();
  CurrentVelocity = (LidarLocation - PrevLocation) / DeltaTime;
  PrevLocation = LidarLocation;
}

float ARayCastFMCWLidar::CalculateRelativeVelocity(const FHitResult& OutHit, const FVector& LidarLocation) const
{
  constexpr float TO_METERS = 1e-2;

  const TWeakObjectPtr<AActor> HittedActor = OutHit.Actor;
  const FVector TargetVelocity = HittedActor->GetVelocity();
  const FVector TargetLocation = OutHit.ImpactPoint;
  const FVector Direction = (TargetLocation - LidarLocation).GetSafeNormal();
  const FVector DeltaVelocity = (TargetVelocity - CurrentVelocity);
  const float V = TO_METERS * FVector::DotProduct(DeltaVelocity, Direction);

  return V;
}