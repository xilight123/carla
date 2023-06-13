// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/sensor/s11n/FMCWLidarSerializer.h"
#include "carla/sensor/data/FMCWLidarMeasurement.h"

namespace carla {
namespace sensor {
namespace s11n {

  SharedPtr<SensorData> FMCWLidarSerializer::Deserialize(RawData &&data) {
    return SharedPtr<data::FMCWLidarMeasurement>(
        new data::FMCWLidarMeasurement{std::move(data)});
  }

} // namespace s11n
} // namespace sensor
} // namespace carla