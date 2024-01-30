// Copyright 2024 Ekumen, Inc.
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

#ifndef BELUGA_RANDOM_UNIFORM_OCCUPANCY_GRID_DISTRIBUTION_HPP
#define BELUGA_RANDOM_UNIFORM_OCCUPANCY_GRID_DISTRIBUTION_HPP

#include <random>

#include <range/v3/utility/random.hpp>
#include <sophus/se2.hpp>

#include <beluga/sensor/data/occupancy_grid.hpp>

namespace beluga {

template <class T>
class UniformOccupancyGridDistribution;

template <>
class UniformOccupancyGridDistribution<Sophus::SE2d> {
 public:
  template <class Derived>
  constexpr explicit UniformOccupancyGridDistribution(const BaseOccupancyGrid2<Derived>& grid)
      : free_states_{compute_free_states(static_cast<const Derived&>(grid))},
        distribution_{0, free_states_.size() - 1} {}

  template <class URNG = typename ranges::detail::default_random_engine>
  [[nodiscard]] Sophus::SE2d operator()(URNG& engine = ranges::detail::get_random_engine()) {
    return {Sophus::SO2d::sampleUniform(engine), free_states_[distribution_(engine)]};
  }

 private:
  std::vector<Eigen::Vector2d> free_states_;
  std::uniform_int_distribution<std::size_t> distribution_;

  template <class Grid>
  static std::vector<Eigen::Vector2d> compute_free_states(const Grid& grid) {
    constexpr auto kFrame = Grid::Frame::kGlobal;
    return grid.coordinates_for(grid.free_cells(), kFrame) | ranges::to<std::vector>;
  }
};

template <class Derived>
UniformOccupancyGridDistribution(const BaseOccupancyGrid2<Derived>&) -> UniformOccupancyGridDistribution<Sophus::SE2d>;

}  // namespace beluga

#endif
