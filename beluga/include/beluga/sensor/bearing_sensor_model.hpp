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

#ifndef BELUGA_SENSOR_BEARING_SENSOR_MODEL_HPP
#define BELUGA_SENSOR_BEARING_SENSOR_MODEL_HPP

// external
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/all.hpp>
#include <range/v3/view/transform.hpp>
#include <sophus/se2.hpp>
#include <sophus/se3.hpp>

// standard library
#include <cmath>
#include <complex>
#include <random>
#include <tuple>
#include <utility>
#include <vector>

/**
 * \file
 * \brief Implementation of a discrete landmark bearing sensor model.
 */

namespace beluga {

/// Parameters used to construct a BearingSensorModel instance.
struct BearingModelParam {
  double sigma_bearing{0.2};                 ///< Standard deviation of the bearing error.
  Sophus::SE3d sensor_in_robot_transform{};  ///< Pose of the sensor in the robot reference frame.
};

/// Generic bearing sensor model, for both 2D and 3D state types.
/**
 * This class implements the BearingSensorModelInterface interface
 * and satisfies \ref SensorModelPage.
 * \tparam Mixin The mixed-in type with no particular requirements.
 * \tparam LandmarkMap class managing the list of known landmarks.
 * \tparam StateType type of the state of the particle.
 */
template <class Mixin, class LandmarkMap, class StateType>
class BearingSensorModel : public Mixin {
 public:
  /// State type of a particle.
  using state_type = StateType;
  /// Weight type of the particle.
  using weight_type = double;
  /// Measurement type of the sensor: (nx, ny, nz) director vector of the detection (may or may not be normalized)
  /// and the landmark id.
  using measurement_type = std::vector<std::tuple<double, double, double, uint32_t>>;
  /// Parameter type that the constructor uses to configure the bearing sensor model.
  using param_type = BearingModelParam;

  /// Constructs a BearingSensorModel instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param params Parameters to configure this instance. See beluga::BearingModelParam for details.
   * \param landmark_map Map of landmarks to be used by this sensor model.
   * \param ...rest Arguments that are not used by this part of the mixin, but
   * by others.
   */
  template <class... Args>
  explicit BearingSensorModel(param_type params, LandmarkMap landmark_map, Args&&... rest)
      : Mixin(std::forward<Args>(rest)...), params_{std::move(params)}, landmark_map_{std::move(landmark_map)} {}

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
    const auto [xmin, xmax, ymin, ymax, zmin, zmax] = landmark_map_.map_limits();

    if constexpr (std::is_same_v<state_type, Sophus::SE3d>) {
      auto x_distribution = std::uniform_real_distribution<double>{xmin, xmax};
      auto y_distribution = std::uniform_real_distribution<double>{ymin, ymax};
      auto z_distribution = std::uniform_real_distribution<double>{zmin, zmax};
      return Sophus::SE3d{
          Sophus::SO3d::sampleUniform(gen),
          Eigen::Vector3d{x_distribution(gen), y_distribution(gen), z_distribution(gen)}};
    } else {
      auto x_distribution = std::uniform_real_distribution<double>{xmin, xmax};
      auto y_distribution = std::uniform_real_distribution<double>{ymin, ymax};
      return Sophus::SE2d{Sophus::SO2d::sampleUniform(gen), Eigen::Vector2d{x_distribution(gen), y_distribution(gen)}};
    }
  }

  /// Gets the importance weight for a particle with the provided state.
  /**
   * \param state State of the particle to calculate its importance weight.
   * \return Calculated importance weight.
   */
  [[nodiscard]] weight_type importance_weight(const state_type& state) const {
    Sophus::SE3d robot_in_world_transform;

    if constexpr (std::is_same_v<state_type, Sophus::SE3d>) {
      // The robot pose state is already given in 3D,
      robot_in_world_transform = state;
    } else {
      // The robot pose state is given in 2D. Notice that in this case
      // the 2D pose of the robot is assumed to be that of the robot footprint (projection of the robot
      // on the z=0 plane of the 3D world frame). This is so that we can tie the sensor reference frame
      // to the world frame where the landmarks are given without additional structural information.
      robot_in_world_transform = Sophus::SE3d{
          Sophus::SO3d::rotZ(state.so2().log()),
          Eigen::Vector3d{state.translation().x(), state.translation().y(), 0.0}};
    }

    // precalculate the sensor pose in the world frame
    const auto sensor_in_world_transform = robot_in_world_transform * params_.sensor_in_robot_transform;

    const auto detection_weight = [this, &sensor_in_world_transform](const auto& sample) {
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

      // We'll assume the probability of identification error to be zero
      const auto prob = bearing_error_prob;

      // TODO(unknown): We continue to use the sum-of-cubes formula that nav2 uses
      // See https://github.com/Ekumen-OS/beluga/issues/153
      return prob * prob * prob;
    };

    return std::transform_reduce(points_.cbegin(), points_.cend(), 1.0, std::plus{}, detection_weight);
  }

  /// \copydoc BearingSensorModelInterface::update_sensor(measurement_type&&points)
  void update_sensor(measurement_type&& points) final { points_ = std::move(points); }

  /// \copydoc BearingSensorModelInterface::update_map(Map&& map)
  void update_map(LandmarkMap&& map) final { landmark_map_ = std::move(map); }

 private:
  param_type params_;
  LandmarkMap landmark_map_;
  measurement_type points_;
};

/// Sensor model based on discrete landmarks bearing detection for 2D state types.
/**
 * This class implements the BearingSensorModelInterface interface
 * and satisfies \ref SensorModelPage.
 * \tparam Mixin The mixed-in type with no particular requirements.
 * \tparam LandmarkMap class managing the list of known landmarks.
 */
template <class Mixin, class LandmarkMap>
using BearingSensorModel2d = BearingSensorModel<Mixin, LandmarkMap, Sophus::SE2d>;

/// Sensor model based on discrete landmarks bearing detection for 3D state types.
/**
 * This class implements the BearingSensorModelInterface interface
 * and satisfies \ref SensorModelPage.
 * \tparam Mixin The mixed-in type with no particular requirements.
 * \tparam LandmarkMap class managing the list of known landmarks.
 */
template <class Mixin, class LandmarkMap>
using BearingSensorModel3d = BearingSensorModel<Mixin, LandmarkMap, Sophus::SE3d>;

}  // namespace beluga

#endif
