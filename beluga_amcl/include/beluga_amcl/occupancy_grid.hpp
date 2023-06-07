// Copyright 2022-2023 Ekumen, Inc.
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

#ifndef BELUGA_AMCL__OCCUPANCY_GRID_HPP_
#define BELUGA_AMCL__OCCUPANCY_GRID_HPP_

#include <tf2/utils.h>

#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <beluga/sensor/data/occupancy_grid.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>

namespace beluga_amcl
{

class OccupancyGrid : public beluga::BaseOccupancyGrid2<OccupancyGrid>
{
public:
  struct ValueTraits
  {
    // https://wiki.ros.org/map_server#Value_Interpretation
    static constexpr std::int8_t free_value = 0;
    static constexpr std::int8_t unknown_value = -1;
    static constexpr std::int8_t occupied_value = 100;

    bool is_free(std::int8_t value) const
    {
      return value == free_value;
    }

    bool is_unknown(std::int8_t value) const
    {
      return value == unknown_value;
    }

    bool is_occupied(std::int8_t value) const
    {
      return value == occupied_value;
    }
  };

  explicit OccupancyGrid(nav_msgs::msg::OccupancyGrid::SharedPtr grid)
  : storage_size_{grid->data.size()},
    map_width_{grid->info.width},
    map_height_{grid->info.height},
    map_resolution_{grid->info.resolution},
    origin_(make_origin_transform(grid->info.origin)),
    storage_data_{std::move(grid->data)}
  {}

  [[nodiscard]] const Sophus::SE2d & origin() const
  {
    return origin_;
  }

  std::size_t size() const
  {
    return storage_size_;
  }


  [[nodiscard]] const auto & data() const
  {
    return storage_data_;
  }

  std::size_t width() const
  {
    return map_width_;
  }

  std::size_t height() const
  {
    return map_height_;
  }

  [[nodiscard]] double resolution() const
  {
    return map_resolution_;
  }

  [[nodiscard]] auto value_traits() const
  {
    return ValueTraits{};
  }

private:
  std::size_t storage_size_;
  std::size_t map_width_;
  std::size_t map_height_;
  double map_resolution_;
  Sophus::SE2d origin_;
  std::vector<std::int8_t> storage_data_;


  static Sophus::SE2d make_origin_transform(const geometry_msgs::msg::Pose & origin)
  {
    const auto rotation = Sophus::SO2d{tf2::getYaw(origin.orientation)};
    const auto translation = Eigen::Vector2d{origin.position.x, origin.position.y};
    return Sophus::SE2d{rotation, translation};
  }
};

}  // namespace beluga_amcl

#endif  // BELUGA_AMCL__OCCUPANCY_GRID_HPP_
