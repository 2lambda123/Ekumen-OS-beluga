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

#ifndef BELUGA_ESTIMATION_CLUSTER_BASED_ESTIMATOR_HPP
#define BELUGA_ESTIMATION_CLUSTER_BASED_ESTIMATOR_HPP

// standard library
#include <algorithm>
#include <initializer_list>
#include <numeric>
#include <optional>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// external
#include <range/v3/action/sort.hpp>
#include <range/v3/algorithm/max_element.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <range/v3/range/conversion.hpp>
#include <range/v3/view/cache1.hpp>
#include <range/v3/view/filter.hpp>
#include <range/v3/view/zip.hpp>
#include <sophus/se2.hpp>
#include <sophus/types.hpp>

// project
#include <beluga/algorithm/spatial_hash.hpp>
#include <beluga/estimation/utils/range_statistics.hpp>
#include <beluga/views.hpp>

/**
 * \file
 * \brief Implementation of a cluster-base estimator mixin.
 */

namespace beluga {

namespace cse_detail {

/// @brief A struct that holds the data of a single cell in the grid.
struct GridCellData {
  double weight{0.0};                         ///< average weight of the cell
  std::size_t num_particles{0};               ///< number of particles in the cell
  Sophus::SE2d representative_pose_in_world;  ///< state of a particle that is within the cell
  std::optional<std::size_t> cluster_id;      ///< cluster id of the cell
};

/// @brief A map that holds the sparse data about the particles grouped in cells. Used by the clusterization algorithm.
using GridCellDataMap2D = std::unordered_map<std::size_t, GridCellData>;

/// @brief Function that creates an vector containing the hashes of each of the states in the input range.
/// @tparam Range Type of the states range.
/// @tparam Hasher Hash function type to convert states into hashes.
/// @param states The range of states.
/// @param spatial_hash_function_ The hash object instance.
/// @return A vector containing the hashes of each of the states in the input range.
template <class Range, class Hasher>
[[nodiscard]] auto precalculate_particle_hashes(Range&& states, const Hasher& spatial_hash_function_) {
  const auto state_to_range = [&](const auto& state) { return spatial_hash_function_(state); };
  return states | ranges::views::transform(state_to_range) | ranges::to<std::vector<std::size_t>>();
}

/// @brief Populate the grid cell data map with the data from the particles and their weights.
/// @tparam GridCellDataMapType Type of the grid cell data map.
/// @tparam Range Type of the states range.
/// @tparam Weights Type of the weights range.
/// @tparam Hashes Type of the hashes range.
/// @param states Range containing the states of the particles.
/// @param weights Range containing the weights of the particles.
/// @param hashes Range containing the hashes of the particles.
/// @return New instance of the grid cell data map populated with the information from the states.
template <class GridCellDataMapType, class Range, class Weights, class Hashes>
[[nodiscard]] auto populate_grid_cell_data_from_particles(Range&& states, Weights&& weights, const Hashes& hashes) {
  GridCellDataMapType grid_cell_data;

  // preallocate memory with a very rough estimation of the number of grid_cells we might end up with
  grid_cell_data.reserve(states.size() / 5);

  // calculate the accumulated cell weight and save a single representative_pose_in_world for each cell
  for (const auto& [state, weight, hash] : ranges::views::zip(states, weights, hashes)) {
    auto [it, inserted] = grid_cell_data.try_emplace(hash, GridCellData{});
    it->second.weight += weight;
    it->second.num_particles++;
    if (inserted) {
      it->second.representative_pose_in_world = state;
    }
  }

  // normalize the accumulated weight by the number of particles in each cell
  // to avoid biasing the clustering algorithm towards cells that randomly end up
  // with more particles than others.
  for (auto& [hash, entry] : grid_cell_data) {
    entry.weight /= static_cast<double>(entry.num_particles);  // num_particles is guaranteed to be > 0
  }

  return grid_cell_data;
}

/// @brief Calculate the weight threshold that corresponds to a given percentile of the weights.
/// @tparam GridCellDataMapType Type of the grid cell data map.
/// @param grid_cell_data The grid cell data map.
/// @param threshold The percentile of the weights to calculate the threshold for (range: 0.0 to 1.0)
/// @return Threshold value that corresponds to the given percentile of the weights.
template <class GridCellDataMapType>
[[nodiscard]] auto calculate_percentile_weight_threshold(GridCellDataMapType&& grid_cell_data, double threshold) {
  const auto extract_weight_f = [](const auto& grid_cell) { return grid_cell.second.weight; };
  auto weights = grid_cell_data | ranges::views::transform(extract_weight_f) | ranges::to<std::vector<double>>() |
                 ranges::actions::sort;
  return weights[static_cast<std::size_t>(static_cast<double>(weights.size()) * threshold)];
}

/// @brief Cap the weight of each cell in the grid cell data map to a given value.
/// @tparam GridCellDataMapType Type of the grid cell data map.
/// @param grid_cell_data The grid cell data map.
/// @param weight_cap The maximum weight value to be assigned to each cell.
template <class GridCellDataMapType>
void cap_grid_cell_data_weights(GridCellDataMapType&& grid_cell_data, double weight_cap) {
  for (auto& [hash, entry] : grid_cell_data) {
    const auto capped_weight = std::min(entry.weight, weight_cap);
    entry.weight = capped_weight;
  }
}

/// @brief Creates the priority queue used by the clustering information from the grid cell data map.
/// @tparam GridCellDataMapType Type of the grid cell data map.
/// @param grid_cell_data The grid cell data map.
/// @return A priority queue containing the information from the grid cell data map.
template <class GridCellDataMapType>
[[nodiscard]] auto populate_priority_queue(GridCellDataMapType&& grid_cell_data) {
  struct PriorityQueueItem {
    double priority;   // priority value used to order the queue (higher value first).
    std::size_t hash;  // hash of the cell in the grid cell data map.
  };

  struct PriorityQueueItemCompare {
    constexpr bool operator()(const PriorityQueueItem& lhs, const PriorityQueueItem& rhs) const {
      return lhs.priority < rhs.priority;  // sort in decreasing priority order
    }
  };

  const auto cell_data_to_queue_item = [](const auto& grid_cell) {
    return PriorityQueueItem{grid_cell.second.weight, grid_cell.first};
  };

  auto queue_container = grid_cell_data |                                     //
                         ranges::views::transform(cell_data_to_queue_item) |  //
                         ranges::to<std::vector<PriorityQueueItem>>();        //
  return std::priority_queue<PriorityQueueItem, std::vector<PriorityQueueItem>, PriorityQueueItemCompare>(
      PriorityQueueItemCompare{}, std::move(queue_container));
}

/// @brief Function that runs the clustering algorithm and assigns a cluster id to each cell in the grid cell data map.
/// @tparam GridCellDataMapType Type of the grid cell data map.
/// @tparam Hasher Type of the hash function used to convert states into hashes.
/// @tparam Neighbors Type of the range containing the neighbors of a cell.
/// @param grid_cell_data The grid cell data map.
/// @param spatial_hash_function_ The hash object instance.
/// @param neighbors Range containing the neighbors of a cell.
/// @param weight_cap The maximum weight value to be assigned to each cell.
template <class GridCellDataMapType, class Hasher, class Neighbors>
void map_cells_to_clusters(
    GridCellDataMapType&& grid_cell_data,
    Hasher&& spatial_hash_function_,
    Neighbors&& neighbors,
    double weight_cap) {
  auto grid_cell_queue = populate_priority_queue(grid_cell_data);

  std::size_t next_cluster_id = 0;

  while (!grid_cell_queue.empty()) {
    const auto grid_cell_hash = grid_cell_queue.top().hash;
    grid_cell_queue.pop();

    // any hash that comes out of the queue is known to exist in the cell data map
    auto& grid_cell = grid_cell_data[grid_cell_hash];
    const auto& grid_cell_weight = grid_cell.weight;
    const auto& representative_pose_in_world = grid_cell.representative_pose_in_world;

    // if there's no cluster id assigned to the cell, assign it a new one
    if (!grid_cell.cluster_id.has_value()) {
      grid_cell.cluster_id = next_cluster_id++;
    }

    // process the neighbors of the cell; if they don't have a cluster id already assigned
    // then assign them one and add them to the queue with an inflated priority
    // to ensure they get processed ASAP before moving on to other local peaks.
    // Notice that with this algorithm each cell will go through the priority queue at most twice.

    const auto get_neighbor_hash = [&](const auto& neighbor_pose_in_representative) {
      return spatial_hash_function_(representative_pose_in_world * neighbor_pose_in_representative);
    };

    const auto filter_invalid_neighbors = [&](const auto& neighbor_hash) {
      auto it = grid_cell_data.find(neighbor_hash);
      return (
          (it != grid_cell_data.end()) &&            // is within the map
          (!it->second.cluster_id.has_value()) &&    // AND not yet mapped to a cluster
          (it->second.weight <= grid_cell_weight));  // AND has lower weight than the current cell
    };

    auto valid_neighbor_hashes_view =                     //
        neighbors |                                       //
        ranges::views::transform(get_neighbor_hash) |     //
        ranges::views::filter(filter_invalid_neighbors);  //

    for (const auto& neighbor_hash : valid_neighbor_hashes_view) {
      auto& neighbor = grid_cell_data[neighbor_hash];
      neighbor.cluster_id = grid_cell.cluster_id;
      const auto inflated_priority =
          weight_cap + neighbor.weight;  // since weights are capped at weight_cap, this gives us a value that is
                                         // guaranteed to be higher than any other weight from a local maximum.
      grid_cell_queue.push({inflated_priority, neighbor_hash});  // reintroduce with inflated priority
    }
  }
}

/// @brief For each cluster, estimate the mean and covariance of the states that belong to it.
/// @tparam StateType Type used for the pose estimation.
/// @tparam CovarianceType Type used for the covariance estimation.
/// @tparam GridCellDataMapType Type of the grid cell data map.
/// @tparam Range Range type of the states.
/// @tparam Weights Range type of the weights.
/// @tparam Hashes Range type of the hashes.
/// @param grid_cell_data Grid cell data map that has already been processed by the clustering algorithm.
/// @param states Range containing the states of the particles.
/// @param weights Range containing the weights of the particles.
/// @param hashes Range containing the hashes of the particles.
/// @return A vector of tuples, containing the weight, mean and covariance of each cluster, in no particular order.
template <class StateType, class CovarianceType, class GridCellDataMapType, class Range, class Weights, class Hashes>
[[nodiscard]] auto
estimate_clusters(GridCellDataMapType&& grid_cell_data, Range&& states, Weights&& weights, Hashes&& hashes) {
  static constexpr auto weight = [](const auto& t) { return std::get<1>(t); };
  static constexpr auto cluster = [](const auto& t) { return std::get<2>(t); };

  const auto cluster_from_hash = [&grid_cell_data](const std::size_t hash) {
    const auto& grid_cell = grid_cell_data[hash];
    return grid_cell.cluster_id;
  };

  const auto has_cluster = [](const auto& t) { return cluster(t).has_value(); };

  auto particles = ranges::views::zip(states, weights, hashes | ranges::views::transform(cluster_from_hash)) |
                   ranges::views::cache1 | ranges::views::filter(has_cluster);

  const auto cluster_ids = particles | ranges::views::transform(cluster) | ranges::to<std::unordered_set>;

  // for each cluster found, estimate the mean and covariance of the states that belong to it
  std::vector<std::tuple<double, StateType, CovarianceType>> cluster_estimates;

  for (const auto id : cluster_ids) {
    auto filtered_particles =
        particles | ranges::views::cache1 | ranges::views::filter([id](const auto& p) { return cluster(p) == id; });

    const auto particle_count = ranges::distance(filtered_particles);
    if (particle_count < 2) {
      // if there's only one sample in the cluster we can't estimate the covariance
      continue;
    }

    const auto total_weight = ranges::accumulate(filtered_particles, 0.0, std::plus{}, weight);
    auto filtered_states = filtered_particles | beluga::views::elements<0>;
    auto filtered_weights = filtered_particles | beluga::views::elements<1>;
    const auto [mean, covariance] = beluga::estimate(filtered_states, filtered_weights);
    cluster_estimates.emplace_back(total_weight, std::move(mean), std::move(covariance));
  }

  return cluster_estimates;
}

}  // namespace cse_detail

/// Primary template for a cluster-based estimation algorithm.
/**
 * Particles are groups into clusters around local maximums and the mean and covariance of the cluster with the highest
 * weight is returned.
 *
 * This class implements the EstimationInterface interface
 * and satisfies \ref StateEstimatorPage.
 * *
 * \tparam Mixin The mixin that implements the particle filter interface.
 * \tparam StateType The type of the state to be estimated.
 * \tparam CovarianceType The type of the covariance matrix of the state to be estimated.
 */
template <class Mixin, class StateType, class CovarianceType>
class ClusterBasedStateEstimator : public Mixin {
 public:
  /// Constructs a ClusterBasedStateEstimator instance.
  /**
   * \tparam ...Args Arguments types for the remaining mixin constructors.
   * \param ...args Arguments that are not used by this part of the mixin, but by others.
   */
  template <class... Args>
  explicit ClusterBasedStateEstimator(Args&&... args) : Mixin(std::forward<Args>(args)...) {}

  /// \copydoc EstimationInterface2d::estimate()
  [[nodiscard]] std::pair<StateType, CovarianceType> estimate() const final;

 private:
  static constexpr double kSpatialHashResolution = 0.20;   ///< clustering algorithm spatial resolution
  static constexpr double kAngularHashResolution = 0.524;  ///< clustering algorithm angular resolution

  /// @brief spatial hash function used to group particles in cells
  const beluga::spatial_hash<Sophus::SE2d> spatial_hash_function_{
      kSpatialHashResolution,  // x
      kSpatialHashResolution,  // y
      kAngularHashResolution   // theta
  };

  /// @brief Adjacent grid cells used by the clustering algorithm.
  const std::vector<Sophus::SE2d> adjacent_grid_cells_ = {
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{+kSpatialHashResolution, 0.0}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{-kSpatialHashResolution, 0.0}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{0.0, +kSpatialHashResolution}},
      Sophus::SE2d{Sophus::SO2d{0.0}, Sophus::Vector2d{0.0, -kSpatialHashResolution}},
      Sophus::SE2d{Sophus::SO2d{+kAngularHashResolution}, Sophus::Vector2d{0.0, 0.0}},
      Sophus::SE2d{Sophus::SO2d{-kAngularHashResolution}, Sophus::Vector2d{0.0, 0.0}},
  };
};

template <class Mixin, class StateType, class CovarianceType>
std::pair<StateType, CovarianceType> ClusterBasedStateEstimator<Mixin, StateType, CovarianceType>::estimate() const {
  auto hashes = cse_detail::precalculate_particle_hashes(this->self().states(), spatial_hash_function_);
  auto grid_cell_data = cse_detail::populate_grid_cell_data_from_particles<cse_detail::GridCellDataMap2D>(
      this->self().states(), this->self().weights(), hashes);
  const auto weight_cap = cse_detail::calculate_percentile_weight_threshold(grid_cell_data, 0.9);
  cse_detail::cap_grid_cell_data_weights(grid_cell_data, weight_cap);
  cse_detail::map_cells_to_clusters(grid_cell_data, spatial_hash_function_, adjacent_grid_cells_, weight_cap);

  auto per_cluster_estimates = cse_detail::estimate_clusters<StateType, CovarianceType>(
      grid_cell_data, this->self().states(), this->self().weights(), hashes);

  if (per_cluster_estimates.empty()) {
    // hmmm... maybe the particles are too fragmented? calculate the overall mean and covariance
    return beluga::estimate(this->self().states(), this->self().weights());
  }

  // return the element with the greater weight
  const auto [_, mean, covariance] =
      *ranges::max_element(per_cluster_estimates, std::less{}, [](const auto& t) { return std::get<0>(t); });
  return {mean, covariance};
}

/// An alias template for the simple state estimator in 2D.
template <class Mixin>
using ClusterBasedStateEstimator2d = ClusterBasedStateEstimator<Mixin, Sophus::SE2d, Eigen::Matrix3d>;

}  // namespace beluga

#endif
