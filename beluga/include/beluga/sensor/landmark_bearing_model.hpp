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

#ifndef BELUGA_SENSOR_LANDMARK_BEARING_MODEL_HPP
#define BELUGA_SENSOR_LANDMARK_BEARING_MODEL_HPP

// external
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

// project
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/transform.hpp>

// standard library
#include <cmath>
#include <complex>
#include <random>
#include <tuple>
#include <vector>

/**
 * \file
 * \brief Implementation of a discrete landmark sensor model.
 */

namespace beluga {

/// Parameters used to construct a LandmarkBearingSensorModel instance.
/**
 * See Probabilistic Robotics \cite thrun2005probabilistic section 6.6
 */
struct LandmarkBearingModelParam {
  double sigma_bearing{0.2};
  Sophus::SE3d sensor_in_robot{};
};

/// Landmark model for discrete detection sensors.
/**
 * This class implements the LandmarkBearingSensorModelInterface2d interface
 * and satisfies \ref SensorModelPage.
 *
 * See Probabilistic Robotics \cite thrun2005probabilistic Chapter 6.6.
 *
 * \tparam Mixin The mixed-in type with no particular requirements.
 * \tparam LandmarkMap class managing the list of known landmarks.
 *  It must satisfy \ref OccupancyGrid2Page.
 */
template <class Mixin, class LandmarkMap>
class LandmarkBearingSensorModel : public Mixin {
 public:
  /// State type of a particle.
  using state_type = Sophus::SE2d;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: (nx, ny, nz) director vector of the bearing + category.
  using measurement_type = std::vector<std::tuple<double, double, double, uint32_t>>;

  /// Parameter type that the constructor uses to configure the beam sensor
  /// model.
  using param_type = LandmarkBearingModelParam;

  /// Constructs a LandmarkBearingSensorModel instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param params Parameters to configure this instance.
   *  See beluga::BeamModelParams for details.
   * \param grid Occupancy grid representing the static map.
   * \param ...rest Arguments that are not used by this part of the mixin, but
   * by others.
   */
  template <class... Args>
  explicit LandmarkBearingSensorModel(const param_type& params, LandmarkMap landmark_map, Args&&... rest)
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
    // this 2d to 3d conversion is fishy
    const auto robot_in_world_transform = Sophus::SE3d{
        Sophus::SO3d::rotZ(state.so2().log()), Eigen::Vector3d{state.translation().x(), state.translation().y(), 0.0}};

    // precalculate the sensor pose in the world frame
    const auto sensor_in_world_transform = robot_in_world_transform * params_.sensor_in_robot;

    const auto detection_weight = [this, &robot_in_world_transform, &sensor_in_world_transform](const auto& sample) {
      // find the landmark the most closely matches the sample bearing vector
      const auto landmark_opt = landmark_map_.find_closest_bearing_landmark(sample, sensor_in_world_transform);

      // if we did not find a matching landmark, return 0.0
      if (!landmark_opt) {
        return 0.0;
      }

      // recover sample bearing vector
      const auto [s_nx, s_ny, s_nz, s_category] = sample;

      // recover landmark bearing vector
      const auto [l_nx, l_ny, l_nz, l_category] = landmark_opt.value();

      // calculate the aperture vector between the sample and the landmark
      const auto cos_aperture = s_nx * l_nx + s_ny * l_ny + s_nz * l_nz;
      const auto sin_aperture = std::sqrt(
          (s_ny * l_nz - s_nz * l_ny) * (s_ny * l_nz - s_nz * l_ny) +
          (s_nz * l_nx - s_nx * l_nz) * (s_nz * l_nx - s_nx * l_nz) +
          (s_nx * l_ny - s_ny * l_nx) * (s_nx * l_ny - s_ny * l_nx));

      // calculate the angle between the sample and the landmark
      const auto bearing_error = std::atan2(sin_aperture, cos_aperture);

      // model the probability of the landmark being detected as depending on the bearing error
      const auto bearing_error_prob =
          std::exp(-bearing_error * bearing_error / (2. * params_.sigma_bearing * params_.sigma_bearing));

      constexpr auto signature_error_prob = 1.0;  // We'll assume the identification error zero

      const auto prob = bearing_error_prob * signature_error_prob;

      // TODO: We continue to use the sum-of-cubes formula that nav2 uses
      // See https://github.com/Ekumen-OS/beluga/issues/153
      return prob * prob * prob;
    };

    return std::transform_reduce(points_.cbegin(), points_.cend(), 0.0, std::plus{}, detection_weight);
  }

  /// \copydoc LandmarkBearingSensorModelInterface2d::update_sensor(measurement_type&&
  /// points)
  void update_sensor(measurement_type&& points) final { points_ = std::move(points); }

  /// \copydoc LandmarkBearingSensorModelInterface2d::update_map(Map&& map)
  void update_map(LandmarkMap&& map) final { landmark_map_ = std::move(map); }

 private:
  param_type params_;
  LandmarkMap landmark_map_;
  measurement_type points_;
};

}  // namespace beluga

#endif
