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
#include <sophus/se3.hpp>

// standard library
#include <algorithm>
#include <cstdint>
#include <range/v3/view/filter.hpp>
#include <tuple>
#include <vector>

/**
 * \file
 * \brief Landmark map datatype.
 */

namespace beluga {

/// Basic 3D landmark map datatype
class LandmarkMap {
 public:
  using map_boundaries = std::tuple<double, double, double, double>;

  using landmark_data = std::tuple<double, double, double, uint32_t>;
  using landmark_vector = std::vector<landmark_data>;
  using landmark_bearing_data = std::tuple<double, double, double, uint32_t>;

  using world_pose_type = Sophus::SE3d;

  LandmarkMap(const map_boundaries& boundaries, const landmark_vector& landmarks)
      : boundaries_{boundaries}, landmarks_(landmarks) {}

  [[nodiscard]] map_boundaries map_limits() const { return boundaries_; }

  [[nodiscard]] std::optional<landmark_data> find_nearest_landmark(const landmark_data& detection) const {
    const auto& [x, y, z, c] = detection;

    // only consider those that have the same dcat
    auto same_category_landmarks_view =
        landmarks_ | ranges::views::filter([c](const auto& l) { return c == std::get<3>(l); });

    // find the landmark that minimizes the distance to the detection position
    auto min = std::min_element(
        same_category_landmarks_view.begin(), same_category_landmarks_view.end(),
        [x, y, z](const landmark_data& a, const landmark_data& b) {
          const auto& [xa, ya, za, ca] = a;
          const auto& [xb, yb, zb, cb] = b;
          return ((x - xa) * (x - xa) + (y - ya) * (y - ya) + (z - za) * (z - za)) <
                 ((x - xb) * (x - xb) + (y - yb) * (y - yb) + (z - zb) * (z - zb));
        });

    if (min == same_category_landmarks_view.end()) {
      return std::nullopt;
    }

    return *min;
  }

  [[nodiscard]] std::optional<landmark_bearing_data> find_closest_bearing_landmark(
      const landmark_bearing_data& detection,
      const world_pose_type& sensor_in_world) const {
    // disassemble the detection parts
    const auto& [dx, dy, dz, dcat] = detection;

    // only consider those that have the same dcat
    auto same_category_landmarks_view =
        landmarks_ | ranges::views::filter([dcat](const auto& l) { return dcat == std::get<3>(l); });

    // find the landmark that minimizes the bearing error
    const auto& sensor_in_world_position = sensor_in_world.translation();
    const auto detection_bearing_vector = Eigen::Vector3d{dx, dy, dz}.normalized();

    const auto minimization_function = [detection_bearing_vector, &sensor_in_world_position](
                                           const landmark_data& a, const landmark_data& b) {
      const auto [ax, ay, az, acat] = a;
      const auto [bx, by, bz, bcat] = b;

      const auto landmark_to_a_vector = (Eigen::Vector3d{ax, ay, az} - sensor_in_world_position).normalized();
      const auto landmark_to_b_vector = (Eigen::Vector3d{bx, by, bz} - sensor_in_world_position).normalized();

      // find the landmark that minimizes the bearing error by maximizing the dot product against the
      // detection bearing vector
      const auto dot_product_a = landmark_to_a_vector.dot(detection_bearing_vector);
      const auto dot_product_b = landmark_to_b_vector.dot(detection_bearing_vector);

      return dot_product_a > dot_product_b;
    };

    auto min = std::min_element(
        same_category_landmarks_view.begin(), same_category_landmarks_view.end(), minimization_function);

    if (min == same_category_landmarks_view.end()) {
      return std::nullopt;
    }

    // find the normalized bearing vector to the landmark
    const auto [mlx, mly, mlz, mlcat] = *min;
    const auto landmark_to_sensor_vector = (Eigen::Vector3d{mlx, mly, mlz} - sensor_in_world_position).normalized();

    return std::make_tuple(
        landmark_to_sensor_vector.x(), landmark_to_sensor_vector.y(), landmark_to_sensor_vector.z(), mlcat);
  }

 private:
  map_boundaries boundaries_;
  landmark_vector landmarks_;
};

}  // namespace beluga

#endif
