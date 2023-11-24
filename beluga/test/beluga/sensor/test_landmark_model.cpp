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

#include <gmock/gmock.h>

#include "beluga/sensor.hpp"
#include "beluga/sensor/data/landmark_map.hpp"
#include "beluga/sensor/landmark_model.hpp"
#include "ciabatta/ciabatta.hpp"

namespace beluga {

namespace {
LandmarkModelParam GetParams() {
  LandmarkModelParam ret;
  ret.sigma_range = 1.0;
  ret.sigma_bearing = 1.0;
  return ret;
}
}  // namespace

using UUT = ciabatta::mixin<
    ciabatta::curry<beluga::LandmarkSensorModel, LandmarkMap>::mixin,
    ciabatta::provides<beluga::LandmarkSensorModelInterface2d<LandmarkMap>>::mixin>;

double cubed(double v) {
  return v * v * v;
}

TEST(LandmarkSensorModel, SmokeTest) {
  // test case where the landmark is exactly where we expected it
  UUT uut(GetParams(), LandmarkMap({0., 0., 10., 10.}, {{1.0, 2.0, 0.0, 0}}));

  ASSERT_NO_THROW(uut.update_sensor({{1.0, 2.0, 0}}));

  // robot in the map origin
  EXPECT_NEAR(1.0, uut.importance_weight(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{0.0, 0.0}}), 1e-02);
}

TEST(LandmarkSensorModel, OneStdInX) {
  // test case where the landmark is 1 std offset from the expected position
  UUT uut(GetParams(), LandmarkMap({0., 0., 10., 10.}, {{1.0, 0.0, 0.0, 0}}));
  ASSERT_NO_THROW(uut.update_sensor({{2.0, 0.0, 0}}));
  // robot in the map origin
  EXPECT_NEAR(cubed(0.6), uut.importance_weight(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{0.0, 0.0}}), 1e-02);
}

TEST(LandmarkSensorModel, OneStdInY) {
  // test case where the landmark is 1 std offset from the expected position
  UUT uut(GetParams(), LandmarkMap({0., 0., 10., 10.}, {{0.0, 1.0, 0.0, 0}}));
  ASSERT_NO_THROW(uut.update_sensor({{0.0, 2.0, 0}}));
  // robot in the map origin
  EXPECT_NEAR(cubed(0.6), uut.importance_weight(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{0.0, 0.0}}), 1e-02);
}

TEST(LandmarkSensorModel, NoSuchLandmark) {
  // test case where there is not landmark in the map of the type of the
  // observed ones
  UUT uut(GetParams(), LandmarkMap({0., 0., 10., 10.}, {{0.0, 1.0, 0.0, 99}}));
  ASSERT_NO_THROW(uut.update_sensor({{0.0, 2.0, 88}}));
  // robot in the map origin
  EXPECT_NEAR(cubed(0.0), uut.importance_weight(Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{0.0, 0.0}}), 1e-02);
}

}  // namespace beluga
