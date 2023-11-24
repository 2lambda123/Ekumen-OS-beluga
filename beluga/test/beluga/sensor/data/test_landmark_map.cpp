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

#include <beluga/sensor/data/landmark_map.hpp>

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace {

struct LandmarkMapCartesianTest : public ::testing::Test {};

TEST_F(LandmarkMapCartesianTest, SmokeTest) {
  ASSERT_NO_THROW(beluga::LandmarkMap({0., 0., 10., 10.}, beluga::LandmarkMap::landmark_vector{}));
}

TEST_F(LandmarkMapCartesianTest, SimpleMapLoading) {
  auto uut = beluga::LandmarkMap({0., 0., 10., 10.}, {{1.0, 2.0, 3.0, 0}, {5.0, 6.0, 7.0, 1}});

  ASSERT_EQ(uut.map_limits(), std::make_tuple(0., 0., 10., 10.));

  {
    const auto nearest = uut.find_nearest_landmark({1.0, 2.0, 3.0, 0});
    ASSERT_TRUE(nearest.has_value());
    EXPECT_EQ(nearest.value(), std::make_tuple(1.0, 2.0, 3.0, 0));
  }

  {
    const auto nearest = uut.find_nearest_landmark({1.0, 2.0, 3.0, 1});
    ASSERT_TRUE(nearest.has_value());
    EXPECT_EQ(nearest.value(), std::make_tuple(5.0, 6.0, 7.0, 1));
  }

  {
    const auto nearest = uut.find_nearest_landmark({1.0, 2.0, 3.0, 99});
    ASSERT_FALSE(nearest.has_value());
  }
}

TEST_F(LandmarkMapCartesianTest, EmptyMap) {
  auto uut = beluga::LandmarkMap({0., 0., 10., 10.}, beluga::LandmarkMap::landmark_vector{});
  ;
  ASSERT_EQ(uut.map_limits(), std::make_tuple(0., 0., 10., 10.));

  const auto nearest = uut.find_nearest_landmark({1.0, 2.0, 3.0, 0});
  ASSERT_FALSE(nearest.has_value());
}

struct LandmarkMapBearingTest : public ::testing::Test {
  beluga::LandmarkMap uut{
      {0., 0., 10., 10.},
      {
          // category 0
          {9.0, 0.0, 0.0, 0},
          {0.0, 9.0, 0.0, 0},
          {0.0, 0.0, 9.0, 0},
          // category 1
          {-9.0, 0.0, 0.0, 1},
          {0.0, -9.0, 0.0, 1},
          {0.0, 0.0, -9.0, 1},
          // category 2
          {0.0, 0.0, -9.0, 2},
      }};

  Sophus::SE3d sensor_in_world{};
};

TEST_F(LandmarkMapBearingTest, MapLimits) {
  ASSERT_EQ(uut.map_limits(), std::make_tuple(0., 0., 10., 10.));
}

TEST_F(LandmarkMapBearingTest, TrivialQuery1) {
  const auto nearest = uut.find_closest_bearing_landmark({1.0, 0.0, 0.0, 0}, sensor_in_world);
  ASSERT_TRUE(nearest.has_value());
  EXPECT_EQ(nearest.value(), std::make_tuple(1.0, 0.0, 0.0, 0));
}

TEST_F(LandmarkMapBearingTest, TrivialQuery2) {
  const auto nearest = uut.find_closest_bearing_landmark({-1.0, 0.0, 0.0, 1}, sensor_in_world);
  ASSERT_TRUE(nearest.has_value());
  EXPECT_EQ(nearest.value(), std::make_tuple(-1.0, 0.0, 0.0, 1));
}

TEST_F(LandmarkMapBearingTest, FeatureInTotallyDifferentDirection) {
  const auto nearest = uut.find_closest_bearing_landmark({1.0, 0.0, 0.0, 2}, sensor_in_world);
  ASSERT_TRUE(nearest.has_value());
  EXPECT_EQ(nearest.value(), std::make_tuple(0.0, 0.0, -1.0, 2));
}

TEST_F(LandmarkMapBearingTest, NoSuchFeature) {
  const auto nearest = uut.find_closest_bearing_landmark({1.0, 0.0, 0.0, 99}, sensor_in_world);
  ASSERT_FALSE(nearest.has_value());
}

}  // namespace
