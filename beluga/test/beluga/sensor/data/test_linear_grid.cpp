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

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "beluga/sensor/data/value_grid.hpp"

#include <Eigen/Core>

namespace {

TEST(LinearGrid2, Indices) {
  constexpr std::size_t kWidth = 4;
  const auto grid =
      beluga::ValueGrid2<bool>(std::vector<bool>{true, false, true, false, false, true, false, true}, kWidth, 1.);

  EXPECT_EQ(grid.index_at(0, 0), 0);
  EXPECT_EQ(grid.index_at(3, 1), 7);

  EXPECT_EQ(grid.index_at(Eigen::Vector2i(0, 0)), 0);
  EXPECT_EQ(grid.index_at(Eigen::Vector2i(3, 1)), 7);
}

TEST(LinearGrid2, CoordinatesAtIndex) {
  constexpr std::size_t kWidth = 4;
  constexpr double kResolution = 1.;
  const auto grid = beluga::ValueGrid2<bool>(
      std::vector<bool>{true, false, true, false, false, true, false, true}, kWidth, kResolution);

  EXPECT_EQ(grid.coordinates_at(0), Eigen::Vector2d(0.5, 0.5));
  EXPECT_EQ(grid.coordinates_at(3), Eigen::Vector2d(3.5, 0.5));
  EXPECT_EQ(grid.coordinates_at(5), Eigen::Vector2d(1.5, 1.5));
}

}  // namespace
