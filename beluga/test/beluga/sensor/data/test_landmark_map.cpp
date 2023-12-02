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

struct LandmarkMapCartesianTest : public ::testing::Test {
  beluga::LandmarkMap::map_boundaries default_map_boundaries{0.0, 10.0, 1.0, 11.0, 2.0, 12.0};
};

TEST_F(LandmarkMapCartesianTest, SmokeTest) {
  ASSERT_NO_THROW(beluga::LandmarkMap(default_map_boundaries, beluga::LandmarkMap::landmark_vector{}));
}

TEST_F(LandmarkMapCartesianTest, SimpleMapLoading) {
  auto uut = beluga::LandmarkMap(default_map_boundaries, {{1.0, 2.0, 3.0, 0}, {5.0, 6.0, 7.0, 1}});

  ASSERT_EQ(uut.map_limits(), std::make_tuple(0., 10., 1., 11., 2., 12.));

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
  auto uut = beluga::LandmarkMap(default_map_boundaries, beluga::LandmarkMap::landmark_vector{});
  ASSERT_EQ(uut.map_limits(), std::make_tuple(0., 10., 1., 11., 2., 12.));

  const auto nearest = uut.find_nearest_landmark({1.0, 2.0, 3.0, 0});
  ASSERT_FALSE(nearest.has_value());
}

struct LandmarkMapBearingTest : public ::testing::Test {
  beluga::LandmarkMap::map_boundaries default_map_boundaries{0.0, 10.0, 1.0, 11.0, 2.0, 12.0};

  beluga::LandmarkMap uut{
      default_map_boundaries,
      {
          // category 0
          {+9.0, +0.0, +1.0, 0},
          {+0.0, +9.0, +1.0, 0},
          {+0.0, +0.0, +9.0, 0},
          // category 1
          {-9.0, +0.0, +1.0, 1},
          {+0.0, -9.0, +1.0, 1},
          {+0.0, +0.0, -9.0, 1},
          // category 2
          {+0.0, +0.0, -9.0, 2},
      }};

  Sophus::SE3d sensor_in_world_transform{Sophus::SO3d{}, Eigen::Vector3d{0.0, 0.0, 1.0}};
};

TEST_F(LandmarkMapBearingTest, MapLimits) {
  ASSERT_EQ(uut.map_limits(), std::make_tuple(0., 10., 1., 11., 2., 12.));
}

TEST_F(LandmarkMapBearingTest, TrivialQuery1) {
  const auto nearest = uut.find_closest_bearing_landmark({1.0, 0.0, 0.0, 0}, sensor_in_world_transform);
  ASSERT_TRUE(nearest.has_value());
  EXPECT_EQ(nearest.value(), std::make_tuple(1.0, 0.0, 0.0, 0));
}

TEST_F(LandmarkMapBearingTest, TrivialQuery2) {
  const auto nearest = uut.find_closest_bearing_landmark({-1.0, 0.0, 0.0, 1}, sensor_in_world_transform);
  ASSERT_TRUE(nearest.has_value());
  EXPECT_EQ(nearest.value(), std::make_tuple(-1.0, 0.0, 0.0, 1));
}

TEST_F(LandmarkMapBearingTest, FeatureInTotallyDifferentDirection) {
  const auto nearest = uut.find_closest_bearing_landmark({1.0, 0.0, 0.0, 2}, sensor_in_world_transform);
  ASSERT_TRUE(nearest.has_value());
  EXPECT_EQ(nearest.value(), std::make_tuple(0.0, 0.0, -1.0, 2));
}

TEST_F(LandmarkMapBearingTest, NoSuchFeature) {
  const auto nearest = uut.find_closest_bearing_landmark({1.0, 0.0, 0.0, 99}, sensor_in_world_transform);
  ASSERT_FALSE(nearest.has_value());
}

}  // namespace
