// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.



#pragma once

#include "Carla/Sensor/Sensor.h"

#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/LidarDescription.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/FMCWLidarData.h>
#include <compiler/enable-ue4-macros.h>

// for DeepLearning
#include "../../../../../../../Build/eigen-3.3.7-install/include/Eigen/Core"
#include "../../../../../../../Build/eigen-3.3.7-install/include/Eigen/Geometry"
#include "Carla/Util/BoundingBoxCalculator.h"

#include "RayCastFMCWLidar.generated.h"

/// A ray-cast based Lidar sensor.
UCLASS()
class CARLA_API ARayCastFMCWLidar : public ASensor
{
  GENERATED_BODY()

protected:

  using FFMCWLidarData = carla::sensor::data::FMCWLidarData;
  using FFMCWDetection = carla::sensor::data::FMCWLidarDetection;

public:
  static FActorDefinition GetSensorDefinition();

  ARayCastFMCWLidar(const FObjectInitializer &ObjectInitializer);

  virtual void Set(const FActorDescription &Description) override;
  virtual void Set(const FLidarDescription &LidarDescription);

protected:
  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime) override;

  /// Creates a Laser for each channel.
  void CreateLasers();

  /// Updates LidarMeasurement with the points read in DeltaTime.
  void SimulateLidar(const float DeltaTime);

  /// Shoot a laser ray-trace, return whether the laser hit something.
  bool ShootLaser(const float VerticalAngle, float HorizontalAngle, FHitResult &HitResult, FCollisionQueryParams& TraceParams) const;

  /// Method that allow to preprocess if the rays will be traced.
  virtual void PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel);

  /// Compute all raw detection information
  void ComputeRawDetection(const FHitResult& HitInfo, const FTransform& SensorTransf, FFMCWDetection& Detection, uint32_t Channel, Eigen::Matrix3f LidarAxisRot, std::unordered_map<int, std::vector<float>> *ActorInfoMap) const;

  /// Saving the hits the raycast returns per channel
  void WritePointAsync(uint32_t Channel, FHitResult &Detection);

  /// Clear the recorded data structure
  void ResetRecordedHits(uint32_t Channels, uint32_t MaxPointsPerChannel);

  /// This method uses all the saved FHitResults, compute the
  /// RawDetections and then send it to the LidarData structure.
  virtual void ComputeAndSaveDetections(const FTransform &SensorTransform, const float DeltaTime, const float DeltaAngleOfOneLaser);

  UPROPERTY(EditAnywhere)
  FLidarDescription Description;

  TArray<float> LaserAngles;

  std::vector<std::vector<FHitResult>> RecordedHits;
  std::vector<std::vector<bool>> RayPreprocessCondition;
  std::vector<uint32_t> PointsPerChannel;

private:
  FFMCWLidarData FMCWLidarData;

  // Caculate current velocity
  void CalculateCurrentVelocity(const float DeltaTime);

  // Caculate relative velocity
  float CalculateRelativeVelocity(const FHitResult& OutHit, const FVector& LidarLocation) const;

  FVector CurrentVelocity;
  FVector PrevLocation;

};
