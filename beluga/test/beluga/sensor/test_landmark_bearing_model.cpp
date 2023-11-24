// Copyright 2023 Ekumen, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// external
#include <gmock/gmock.h>
#include "ciabatta/ciabatta.hpp"

// project
#include "beluga/sensor.hpp"
#include "beluga/sensor/data/landmark_map.hpp"
#include "beluga/sensor/landmark_bearing_model.hpp"

namespace beluga {

using UUT = ciabatta::mixin<
    ciabatta::curry<beluga::LandmarkBearingSensorModel, LandmarkMap>::mixin,
    ciabatta::provides<beluga::LandmarkBearingSensorModelInterface<LandmarkMap>>::mixin>;

struct LandmarkBearingModelTest : public ::testing::Test {
  // this places sensor at the origin
  Sophus::SE3d robot_in_world{Sophus::SO3d{}, Eigen::Vector3d{0.0, -1.0, 0.0}};
  Sophus::SE3d sensor_in_robot{Sophus::SO3d{}, Eigen::Vector3d{0.0, 1.0, 0.0}};

  Sophus::SE2d robot_in_world_2d{Sophus::SO2d{}, Eigen::Vector2d{0.0, -1.0}};

  double cubed(double v) { return v * v * v; }

  std::unique_ptr<UUT> build_uut() {
    LandmarkBearingModelParam model_params;
    model_params.sigma_bearing = 0.7845;  // 45 degrees
    model_params.sensor_in_robot = sensor_in_robot;

    beluga::LandmarkMap test_map{
        {0., 0., 10., 10.},
        {
            // category 0
            {9.0, 0.0, 0.0, 0},
            // category 1
            {9.0, 9.0, 0.0, 1},
        }};

    return std::make_unique<UUT>(model_params, test_map);
  }
};

TEST_F(LandmarkBearingModelTest, SmokeTest) {
  auto uut = build_uut();
}

TEST_F(LandmarkBearingModelTest, PerfectBearingMeasurement) {
  // perfect bearing measurement
  auto uut = build_uut();
  ASSERT_NO_THROW(uut->update_sensor({{1.0, 0.0, 0.0, 0}}));
  EXPECT_NEAR(1.0, uut->importance_weight(robot_in_world_2d), 1e-02);
}

TEST_F(LandmarkBearingModelTest, OneStdDevBearingError) {
  // perfect bearing measurement
  auto uut = build_uut();
  ASSERT_NO_THROW(uut->update_sensor({{1.0, 0.0, 0.0, 1}}));
  EXPECT_NEAR(cubed(0.6), uut->importance_weight(robot_in_world_2d), 1e-02);
}

TEST_F(LandmarkBearingModelTest, NoSuchLandmark) {
  // perfect bearing measurement
  auto uut = build_uut();
  ASSERT_NO_THROW(uut->update_sensor({{1.0, 0.0, 0.0, 99}}));
  EXPECT_NEAR(cubed(0.0), uut->importance_weight(robot_in_world_2d), 1e-02);
}

}  // namespace beluga
