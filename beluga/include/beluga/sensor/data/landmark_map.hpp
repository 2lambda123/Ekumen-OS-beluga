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
  using landmark_data = std::tuple<double, double, uint32_t>;
  using landmark_vector = std::vector<landmark_data>;

  LandmarkMap(const map_boundaries& boundaries, const landmark_vector& landmarks)
      : boundaries_{boundaries}, landmarks_(landmarks) {}

  [[nodiscard]] map_boundaries map_limits() const { return boundaries_; }

  [[nodiscard]] std::optional<landmark_data> find_nearest_landmark(const landmark_data& p) const {
    // find the landmark that minimizes the distance to the point p while having
    // the same category while having the same category using ranges
    const auto& [x, y, c] = p;

    auto same_category_landmarks_view = landmarks_ |  //
                                        ranges::views::filter([c](const auto& l) { return c == std::get<2>(l); });

    auto min = std::min_element(
        same_category_landmarks_view.begin(), same_category_landmarks_view.end(),
        [x, y](const landmark_data& a, const landmark_data& b) {
          const auto& [xa, ya, ca] = a;
          const auto& [xb, yb, cb] = b;
          return ((x - xa) * (x - xa) + (y - ya) * (y - ya)) < ((x - xb) * (x - xb) + (y - yb) * (y - yb));
        });

    if (min == same_category_landmarks_view.end()) {
      return std::nullopt;
    }
    return *min;
  }

 private:
  map_boundaries boundaries_;
  landmark_vector landmarks_;
};

}  // namespace beluga

#endif
