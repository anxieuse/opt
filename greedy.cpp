#include "greedy.hpp"

#include <iostream>
#include <numeric>

nn_sol::nn_sol(const std::vector<nd> &nodes, const std::vector<veh> &vehicles,
               const std::vector<std::vector<double>> &distanceMatrix)
    : sol(nodes, vehicles, distanceMatrix) {}

nn_sol::nn_sol(const prob &p) : sol(p.nodes_, p.vehicles_, p.dist_mtx_) {}

void nn_sol::solve() {
  for (auto &v : vehicles_) {
    while (true) {
      const auto [found, closest_node] = find_closest(v);
      if (found && v.load_ - closest_node.demand_ >= 0) {
        v.load_ -= closest_node.demand_;
        v.cost_ += dist_mtx_[v.nodes_.back()][closest_node.id_];
        v.nodes_.push_back(closest_node.id_);
        nodes_[closest_node.id_].is_routed_ = true;
      } else {
        v.cost_ += dist_mtx_[v.nodes_.back()][depot_.id_];
        v.nodes_.push_back(depot_.id_);
        break;
      }
    }
  }

  double cost = std::accumulate(
      std::begin(vehicles_), std::end(vehicles_), 0.0,
      [](const double sum, const veh &v) { return sum + v.cost_; });
  std::cout << "Cost: " << cost << '\n';
  for (const auto &i : nodes_) {
    if (!i.is_routed_) {
      std::cout << "Unreached node: " << '\n';
      std::cout << i << '\n';
    }
  }

  auto sol_valid = check_sol_val();
  std::cout << "Solution valid: " << sol_valid << '\n';
}
