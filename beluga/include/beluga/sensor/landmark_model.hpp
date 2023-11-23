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

#ifndef BELUGA_SENSOR_LANDMARK_MODEL_HPP
#define BELUGA_SENSOR_LANDMARK_MODEL_HPP

#include <cmath>
#include <complex>
#include <random>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>
#include <tuple>
#include <vector>

/**
 * \file
 * \brief Implementation of a discrete landmark sensor model.
 */

namespace beluga {

/// Parameters used to construct a LandmarkSensorModel instance.
/**
 * See Probabilistic Robotics \cite thrun2005probabilistic section 6.6
 */
struct LandmarkModelParam {
  double sigma_range{1.0};
  double sigma_bearing{1.0};
};

/// Landmark model for discrete detection sensors.
/**
 * This class implements the LandmarkSensorModelInterface2d interface
 * and satisfies \ref SensorModelPage.
 *
 * See Probabilistic Robotics \cite thrun2005probabilistic Chapter 6.6.
 *
 * \tparam Mixin The mixed-in type with no particular requirements.
 * \tparam LandmarkMap class managing the list of known landmarks.
 *  It must satisfy \ref OccupancyGrid2Page.
 */
template <class Mixin, class LandmarkMap>
class LandmarkSensorModel : public Mixin {
 public:
  /// State type of a particle.
  using state_type = Sophus::SE2d;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: a point cloud for the range finder.
  using measurement_type = std::vector<std::tuple<double, double, uint32_t>>;

  /// Parameter type that the constructor uses to configure the beam sensor
  /// model.
  using param_type = LandmarkModelParam;

  /// Constructs a LandmarkSensorModel instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param params Parameters to configure this instance.
   *  See beluga::BeamModelParams for details.
   * \param grid Occupancy grid representing the static map.
   * \param ...rest Arguments that are not used by this part of the mixin, but
   * by others.
   */
  template <class... Args>
  explicit LandmarkSensorModel(const param_type& params, LandmarkMap landmark_map, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), params_{params}, landmark_map_{std::move(landmark_map)} {}

  // TODO(ivanpauno): is sensor model the best place for this?
  // Maybe the map could be provided by a different part of the mixin,
  // and that part could be used to generate the random state.
  /// Generates a random particle state.
  /**
   * The generated state is an unoccupied cell of the grid, any free cell is
   * sampled uniformly. The rotation is as well sampled uniformly.
   *
   * \tparam Generator  A random number generator that must satisfy the
   *  [UniformRandomBitGenerator](https://en.cppreference.com/w/cpp/named_req/UniformRandomBitGenerator)
   *  requirements.
   * \param gen An uniform random bit generator object.
   * \return The generated random state.
   */
  template <class Generator>
  [[nodiscard]] state_type make_random_state(Generator& gen) const {
    const auto [xmin, xmax, ymin, ymax] = landmark_map_.map_limits();
    auto x_distribution = std::uniform_real_distribution<double>{xmin, xmax};
    auto y_distribution = std::uniform_real_distribution<double>{ymin, ymax};
    return Sophus::SE2d{Sophus::SO2d::sampleUniform(gen), Eigen::Vector2d{x_distribution(gen), y_distribution(gen)}};
  }

  /// Gets the importance weight for a particle with the provided state.
  /**
   * \param state State of the particle to calculate its importance weight.
   * \return Calculated importance weight.
   */
  [[nodiscard]] weight_type importance_weight(const state_type& state) const {
    using bearing_rep = std::complex<double>;
    const auto transform_r_in_w = state;
    const auto x_offset = transform_r_in_w.translation().x();
    const auto y_offset = transform_r_in_w.translation().y();
    const auto cos_theta = transform_r_in_w.so2().unit_complex().x();
    const auto sin_theta = transform_r_in_w.so2().unit_complex().y();

    const auto detection_weight = [this, &state, &transform_r_in_w, x_offset, y_offset, cos_theta,
                                   sin_theta](const auto& point) {
      // calculate range and bearing to the sample from the robot
      // the sample is already in robot frame
      const auto [px, py, id] = point;
      const auto range = std::sqrt(px * px + py * py);
      const auto bearing = bearing_rep{px, py};

      // convert the sample to the world frame to query the map
      const auto w_px = x_offset + cos_theta * px - sin_theta * py;
      const auto w_py = y_offset + sin_theta * px + cos_theta * py;

      // find the closest matching landmark in the world map
      const auto landmark_opt = landmark_map_.find_nearest_landmark(std::make_tuple(w_px, w_py, id));

      // if we did not find a matching landmark, return 0.0
      if (!landmark_opt) {
        return 0.0;
      }

      // convert landmark pose to world frame
      const auto [w_lx, w_ly, w_lid] = *landmark_opt;

      const auto landmark_in_w = Sophus::SE2d{Sophus::SO2d{}, Eigen::Vector2d{w_lx, w_ly}};
      const auto landmark_in_r = transform_r_in_w.inverse() * landmark_in_w;

      const auto landmark_range = landmark_in_r.translation().norm();
      const auto landmark_bearing = bearing_rep{landmark_in_r.translation().x(), landmark_in_r.translation().y()};

      const auto relative_bearing = bearing * std::conj(landmark_bearing);

      const auto range_error = range - landmark_range;
      const auto bearing_error = std::arg(relative_bearing);

      const auto range_error_prob =
          std::exp(-range_error * range_error / (2. * params_.sigma_range * params_.sigma_range));
      const auto bearing_error_prob =
          std::exp(-bearing_error * bearing_error / (2. * params_.sigma_bearing * params_.sigma_bearing));
      constexpr auto signature_error_prob = 1.0;  // We'll assume the identification error zero

      const auto prob = range_error_prob * bearing_error_prob * signature_error_prob;

      // TODO: We continue to use the sum-of-cubes formula that nav2 uses
      // See https://github.com/Ekumen-OS/beluga/issues/153
      return prob * prob * prob;
    };

    return std::transform_reduce(points_.cbegin(), points_.cend(), 0.0, std::plus{}, detection_weight);
  }

  /// \copydoc LandmarkSensorModelInterface2d::update_sensor(measurement_type&&
  /// points)
  void update_sensor(measurement_type&& points) final { points_ = std::move(points); }

  /// \copydoc LandmarkSensorModelInterface2d::update_map(Map&& map)
  void update_map(LandmarkMap&& map) final { landmark_map_ = std::move(map); }

 private:
  param_type params_;
  LandmarkMap landmark_map_;
  measurement_type points_;
};

}  // namespace beluga

#endif
