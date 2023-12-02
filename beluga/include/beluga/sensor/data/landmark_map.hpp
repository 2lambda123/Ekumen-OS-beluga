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

#ifndef BELUGA_SENSOR_DATA_LANDMARK_MAP_HPP
#define BELUGA_SENSOR_DATA_LANDMARK_MAP_HPP

// external
#include <range/v3/view/filter.hpp>
#include <sophus/se3.hpp>

// standard library
#include <algorithm>
#include <cstdint>
#include <tuple>
#include <utility>
#include <vector>

/**
 * \file
 * \brief Landmark map datatype.
 */

namespace beluga {

/// Basic 3D landmark map datatype
class LandmarkMap {
 public:
  /// Tuple with the map boundaries in the form (x_min, x_max, y_min, y_max, z_min, z_max)
  using map_boundaries = std::tuple<double, double, double, double, double, double>;
  /// Landmark data in the form (dx, dy, dz, category)
  using landmark_data = std::tuple<double, double, double, uint32_t>;
  /// Vector of landmarks
  using landmark_vector = std::vector<landmark_data>;
  /// Tuple with information about the relative bearing of a landmark in the form (dx, dy, dz, category)
  using landmark_bearing_data = std::tuple<double, double, double, uint32_t>;
  /// Type used to represent poses in the world frame
  using world_pose_type = Sophus::SE3d;

  /// @brief Constructor.
  /// @param boundaries Limits of the map.
  /// @param landmarks List of landmarks that can be expected to be detected.
  LandmarkMap(const map_boundaries& boundaries, landmark_vector landmarks)
      : landmarks_(std::move(landmarks)),
        x_min_(std::get<0>(boundaries)),
        x_max_(std::get<1>(boundaries)),
        y_min_(std::get<2>(boundaries)),
        y_max_(std::get<3>(boundaries)),
        z_min_(std::get<4>(boundaries)),
        z_max_(std::get<5>(boundaries)) {}

  /// @brief Returns the map boundaries.
  /// @return The map boundaries.
  [[nodiscard]] map_boundaries map_limits() const {
    return std::make_tuple(x_min_, x_max_, y_min_, y_max_, z_min_, z_max_);
  }

  /// @brief Finds the nearest landmark to a given detection and returns it's data.
  /// @param detection_in_world_position The detection data.
  /// @return The landmark data. nullopt if no landmark was found.
  [[nodiscard]] std::optional<landmark_data> find_nearest_landmark(
      const landmark_data& detection_in_world_position) const {
    const auto [dx, dy, dz, dcat] = detection_in_world_position;

    // only consider those that have the same id
    auto same_category_landmarks_view =
        landmarks_ | ranges::views::filter([dcat = dcat](const auto& l) { return dcat == std::get<3>(l); });

    // find the landmark that minimizes the distance to the detection position
    // This is O(n). A spatial data structure should be used instead.
    auto min = std::min_element(
        same_category_landmarks_view.begin(), same_category_landmarks_view.end(),
        [dx = dx, dy = dy, dz = dz](const landmark_data& a, const landmark_data& b) {
          const auto& [xa, ya, za, ca] = a;
          const auto& [xb, yb, zb, cb] = b;
          return ((dx - xa) * (dx - xa) + (dy - ya) * (dy - ya) + (dz - za) * (dz - za)) <
                 ((dx - xb) * (dx - xb) + (dy - yb) * (dy - yb) + (dz - zb) * (dz - zb));
        });

    if (min == same_category_landmarks_view.end()) {
      return std::nullopt;
    }

    return *min;
  }

  /// @brief Finds the landmark that minimizes the bearing error to a given detection and returns it's data.
  /// @param detection_in_sensor_position The detection data.
  /// @param sensor_in_world_transform The pose of the sensor in the world frame.
  /// @return The landmark data. nullopt if no landmark was found.
  [[nodiscard]] std::optional<landmark_bearing_data> find_closest_bearing_landmark(
      const landmark_bearing_data& detection_in_sensor_position,
      const world_pose_type& sensor_in_world_transform) const {
    const auto [dx, dy, dz, dcat] = detection_in_sensor_position;

    // only consider those that have the same detection id (category)
    auto same_category_landmarks_view =
        landmarks_ | ranges::views::filter([dcat = dcat](const auto& l) { return dcat == std::get<3>(l); });

    // find the landmark that minimizes the bearing error
    const auto detection_in_sensor_bearing = Eigen::Vector3d{dx, dy, dz}.normalized();
    const auto world_in_sensor_transform = sensor_in_world_transform.inverse();

    // This whole search thing is very expensive, with objects getting created, normalized and
    // destroyed multiple times and the same transformations being calculated over and over again.
    // This will only work as a proof of concept, but it needs to be optimized for large numbers of
    // landmarks.
    const auto minimization_function = [&detection_in_sensor_bearing, &world_in_sensor_transform](
                                           const landmark_data& a, const landmark_data& b) {
      const auto [ax, ay, az, acat] = a;
      const auto [bx, by, bz, bcat] = b;

      // convert the landmark locations relative to the sensor frame
      const auto landmark_a_in_sensor_bearing = (world_in_sensor_transform * Eigen::Vector3d{ax, ay, az}).normalized();
      const auto landmark_b_in_sensor_bearing = (world_in_sensor_transform * Eigen::Vector3d{bx, by, bz}).normalized();

      // find the landmark that minimizes the bearing error by maximizing the dot product against the
      // detection bearing vector
      const auto dot_product_a = landmark_a_in_sensor_bearing.dot(detection_in_sensor_bearing);
      const auto dot_product_b = landmark_b_in_sensor_bearing.dot(detection_in_sensor_bearing);

      return dot_product_a > dot_product_b;
    };

    auto min = std::min_element(
        same_category_landmarks_view.begin(), same_category_landmarks_view.end(), minimization_function);

    if (min == same_category_landmarks_view.end()) {
      return std::nullopt;
    }

    // find the normalized bearing vector to the landmark, relative to the sensor frame
    const auto [mlx, mly, mlz, mlcat] = *min;
    const auto landmark_in_sensor_bearing = (world_in_sensor_transform * Eigen::Vector3d{mlx, mly, mlz}).normalized();

    return std::make_tuple(
        landmark_in_sensor_bearing.x(), landmark_in_sensor_bearing.y(), landmark_in_sensor_bearing.z(), mlcat);
  }

 private:
  landmark_vector landmarks_;

  double x_min_{0.0};
  double x_max_{0.0};
  double y_min_{0.0};
  double y_max_{0.0};
  double z_min_{0.0};
  double z_max_{0.0};
};

}  // namespace beluga

#endif
