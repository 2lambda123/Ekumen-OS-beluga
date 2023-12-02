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

#include <gtest/gtest.h>

#include <ciabatta/ciabatta.hpp>

#include <beluga/sensor.hpp>
#include <beluga/sensor/data/landmark_map.hpp>

#include <algorithm>
#include <random>
#include <vector>

namespace beluga {

using Sensor2D = ciabatta::mixin<
    ciabatta::curry<beluga::BearingSensorModel2d, LandmarkMap>::mixin,
    ciabatta::provides<beluga::BearingSensorModelInterface<LandmarkMap>>::mixin>;

using Sensor3D = ciabatta::mixin<
    ciabatta::curry<beluga::BearingSensorModel3d, LandmarkMap>::mixin,
    ciabatta::provides<beluga::BearingSensorModelInterface<LandmarkMap>>::mixin>;

LandmarkMap::map_boundaries default_map_boundaries{-10.0, -10.0, 10.0, 10.0, 0.0, 0.0};

double expectedAggregatedProbability(std::vector<double> landmark_probs) {
  // nav2_amcl formula, $1.0 + \sum_{i=1}^n p_i^3$
  return std::transform_reduce(
      landmark_probs.cbegin(), landmark_probs.cend(), 1.0, std::plus{}, [](const double v) { return v * v * v; });
}

BearingModelParam getDefaultModelParams() {
  BearingModelParam ret;
  ret.sigma_bearing = Sophus::Constants<double>::pi() / 4.0;  // 45 degrees
  ret.sensor_in_robot_transform = Sophus::SE3d{Sophus::SO3d{}, Eigen::Vector3d{0.0, 0.0, 1.0}};
  return ret;
}

template <typename T>
T getRobotInWorldTransform();

template <>
Sophus::SE2d getRobotInWorldTransform<Sophus::SE2d>() {
  return Sophus::SE2d{Sophus::SO2d{-Sophus::Constants<double>::pi() / 2.0}, Eigen::Vector2d{1.0, -1.0}};
}

template <>
Sophus::SE3d getRobotInWorldTransform<Sophus::SE3d>() {
  return Sophus::SE3d{Sophus::SO3d::rotZ(-Sophus::Constants<double>::pi() / 2.0), Eigen::Vector3d{1.0, -1.0, 0.0}};
}

template <typename T>
struct BearingSensorModelTests : public ::testing::Test {};

using BearingSensorModelTestsTypes = ::testing::Types<Sensor2D, Sensor3D>;

TYPED_TEST_SUITE(BearingSensorModelTests, BearingSensorModelTestsTypes);

TYPED_TEST(BearingSensorModelTests, SmokeTest) {
  TypeParam uut(getDefaultModelParams(), LandmarkMap(default_map_boundaries, {{1.0, -1.0, 1.0, 0}}));
}

TYPED_TEST(BearingSensorModelTests, BullsEyeDetection) {
  // test case where the landmark is exactly where we expected it
  TypeParam uut(getDefaultModelParams(), LandmarkMap(default_map_boundaries, {{1.0, -2.0, 1.0, 0}}));
  ASSERT_NO_THROW(uut.update_sensor({{1.0, 0.0, 0.0, 0}}));
  EXPECT_NEAR(
      expectedAggregatedProbability({1.0}),
      uut.importance_weight(getRobotInWorldTransform<typename TypeParam::state_type>()), 1e-02);
}

TYPED_TEST(BearingSensorModelTests, MultipleBullsEyeDetections) {
  // Test multiple detections of with different ids, all perfectly matching
  TypeParam uut(
      getDefaultModelParams(),     //
      LandmarkMap(                 //
          default_map_boundaries,  //
          {
              {1.0, -2.0, 0.0, 0},  // landmark 0
              {1.0, -2.0, 1.0, 1},  // landmark 1
              {1.0, -2.0, 2.0, 2},  // landmark 2
          }));

  ASSERT_NO_THROW(uut.update_sensor({
      {+1.0, +0.0, -1.0, 0},  // landmark 0 detection
      {+1.0, +0.0, +0.0, 1},  // landmark 1 detection
      {+1.0, +0.0, +1.0, 2},  // landmark 2 detection
  }));

  EXPECT_NEAR(
      expectedAggregatedProbability({1.0, 1.0, 1.0}),
      uut.importance_weight(getRobotInWorldTransform<typename TypeParam::state_type>()), 1e-02);
}

TYPED_TEST(BearingSensorModelTests, OneStdInBearing) {
  // test case where the landmark is 1 std offset from the expected bearing
  TypeParam uut(getDefaultModelParams(), LandmarkMap(default_map_boundaries, {{1.0, -2.0, 1.0, 0}}));
  // baseline
  ASSERT_NO_THROW(uut.update_sensor({{1.0, 0.0, 0.0, 0}}));
  EXPECT_NEAR(
      expectedAggregatedProbability({1.0}),
      uut.importance_weight(getRobotInWorldTransform<typename TypeParam::state_type>()), 1e-02);
  // 1 std left
  ASSERT_NO_THROW(uut.update_sensor({{1.0, 1.0, 0.0, 0}}));
  EXPECT_NEAR(
      expectedAggregatedProbability({0.6}),
      uut.importance_weight(getRobotInWorldTransform<typename TypeParam::state_type>()), 1e-02);
  // 1 std right
  ASSERT_NO_THROW(uut.update_sensor({{1.0, -1.0, 0.0, 0}}));
  EXPECT_NEAR(
      expectedAggregatedProbability({0.6}),
      uut.importance_weight(getRobotInWorldTransform<typename TypeParam::state_type>()), 1e-02);
  // 1 std up
  ASSERT_NO_THROW(uut.update_sensor({{1.0, 0.0, 1.0, 0}}));
  EXPECT_NEAR(
      expectedAggregatedProbability({0.6}),
      uut.importance_weight(getRobotInWorldTransform<typename TypeParam::state_type>()), 1e-02);
  // 1 std down
  ASSERT_NO_THROW(uut.update_sensor({{1.0, 0.0, -1.0, 0}}));
  EXPECT_NEAR(
      expectedAggregatedProbability({0.6}),
      uut.importance_weight(getRobotInWorldTransform<typename TypeParam::state_type>()), 1e-02);
}

TYPED_TEST(BearingSensorModelTests, NoSuchLandmark) {
  // perfect bearing measurement
  TypeParam uut(getDefaultModelParams(), LandmarkMap(default_map_boundaries, {{1.0, -1.0, 1.0, 0}}));
  ASSERT_NO_THROW(uut.update_sensor({{1.0, 0.0, 0.0, 99}}));
  EXPECT_NEAR(
      expectedAggregatedProbability({0.0}),
      uut.importance_weight(getRobotInWorldTransform<typename TypeParam::state_type>()), 1e-02);
}

TYPED_TEST(BearingSensorModelTests, SampleGeneration) {
  // check that if I draw 1000 samples, I get a distribution that matches the expected one
  TypeParam uut(
      getDefaultModelParams(),     //
      LandmarkMap(                 //
          default_map_boundaries,  //
          {}));

  // create a random generator engine
  std::default_random_engine rd{42};

  // get the limits of the map
  const auto [xmin, xmax, ymin, ymax, zmin, zmax] = default_map_boundaries;

  // generate 1000 samples, checking each of them to be within the map limits
  std::vector<typename TypeParam::state_type> samples;
  for (int i = 0; i < 1000; ++i) {
    const auto sample = uut.make_random_state(rd);

    EXPECT_GE(sample.translation().x(), xmin);
    EXPECT_LE(sample.translation().x(), xmax);
    EXPECT_GE(sample.translation().y(), ymin);
    EXPECT_LE(sample.translation().y(), ymax);

    if constexpr (std::is_same_v<typename TypeParam::state_type, Sophus::SE3d>) {
      EXPECT_GE(sample.translation().z(), zmin);
      EXPECT_LE(sample.translation().z(), zmax);
    }
  }
}

}  // namespace beluga
