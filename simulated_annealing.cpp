#include "simulated_annealing.hpp"

#include <cmath>
#include <iostream>
#include <numeric>

constexpr double mae = 0.0000000001;

sa_sol::sa_sol(const std::vector<nd> &nodes, const std::vector<veh> &vehicles,
               const std::vector<std::vector<double>> &distanceMatrix,
               const int stag_limit, const double init_temp,
               const double cooling_rate, const int n_reheats)
    : sol(nodes, vehicles, distanceMatrix), stag_limit_(stag_limit),
      init_temp_(init_temp), cooling_rate_(cooling_rate),
      n_reheats_(n_reheats) {
  create_init_sol();
}

sa_sol::sa_sol(const prob &p, const int stag_limit, const double init_temp,
               const double cooling_rate, const int n_reheats)
    : sol(p.nodes_, p.vehicles_, p.dist_mtx_), stag_limit_(stag_limit),
      init_temp_(init_temp), cooling_rate_(cooling_rate),
      n_reheats_(n_reheats) {
  create_init_sol();
}

sa_sol::sa_sol(const sol &s, const int stag_limit, const double init_temp,
               const double cooling_rate, const int n_reheats)
    : sol(s), stag_limit_(stag_limit), init_temp_(init_temp),
      cooling_rate_(cooling_rate), n_reheats_(n_reheats) {
  if (!s.check_sol_val()) {
    std::cout << "The input solution is invalid. Exiting." << '\n';
    exit(0);
  }
}

inline bool sa_sol::allow_move(const double delta, const double temp) {
  return (delta < -mae) ||
         ((static_cast<double>(rand()) / RAND_MAX) < std::exp(-delta / temp));
}

void sa_sol::solve() {
  double cost = std::accumulate(
      std::begin(vehicles_), std::end(vehicles_), 0.0,
      [](const double sum, const veh &v) { return sum + v.cost_; });
  auto best_vehicles = vehicles_;
  double best_cost = cost;
  double current_cost = cost;
  for (int r = 0; r < n_reheats_; r++) {
    // std::cout << "Reheat number: " << r << '\n';
    int stag = stag_limit_;
    double temp = init_temp_;
    while (--stag >= 0) {
      temp *= cooling_rate_;
      const int n_vehicles = vehicles_.size();
      veh &v1 = vehicles_[rand() % n_vehicles];
      veh &v2 = vehicles_[rand() % n_vehicles];
      size_t cur = 0;
      if (v1.nodes_.size() > 2) {
        // do not select trailing zero or starting zero
        cur = rand() % (v1.nodes_.size() - 2) + 1;
      } else {
        continue;
      }
      const size_t rep =
          rand() % (v2.nodes_.size() - 1); // do not select trailing zero
      if (v1.id_ == v2.id_ && (cur == rep + 1 || cur == rep)) {
        continue;
      }
      const size_t prev = cur - 1;
      const size_t next_c = cur + 1;
      const size_t next_r = rep + 1;
      const double cost_reduction =
          dist_mtx_[v1.nodes_[prev]][v1.nodes_[next_c]] -
          dist_mtx_[v1.nodes_[prev]][v1.nodes_[cur]] -
          dist_mtx_[v1.nodes_[cur]][v1.nodes_[next_c]];
      const double cost_increase =
          dist_mtx_[v2.nodes_[rep]][v1.nodes_[cur]] +
          dist_mtx_[v1.nodes_[cur]][v2.nodes_[next_r]] -
          dist_mtx_[v2.nodes_[rep]][v2.nodes_[next_r]];
      const double delta = cost_increase + cost_reduction;
      if ((v2.load_ - nodes_[v1.nodes_[cur]].demand_ >= 0 ||
           v1.id_ == v2.id_) &&
          allow_move(delta, temp)) {
        v1.load_ += nodes_[v1.nodes_[cur]].demand_;
        v2.load_ -= nodes_[v1.nodes_[cur]].demand_;
        v1.cost_ += cost_reduction;
        v2.cost_ += cost_increase;
        const int val = v1.nodes_[cur];
        v1.nodes_.erase(v1.nodes_.begin() + cur);
        if (v1.id_ == v2.id_ && cur < rep) {
          v2.nodes_.insert(v2.nodes_.begin() + rep, val);
        } else {
          v2.nodes_.insert(v2.nodes_.begin() + rep + 1, val);
        }
        current_cost += delta;
      }
      if (current_cost < best_cost) {
        stag = stag_limit_;
        best_vehicles = vehicles_;
        best_cost = current_cost;
      }
    }
  }
  vehicles_ = best_vehicles;
  cost = std::accumulate(
      std::begin(vehicles_), std::end(vehicles_), 0.0,
      [](const double sum, const veh &v) { return sum + v.cost_; });
  std::cout << "Cost: " << cost << '\n';
  for (const auto &i : nodes_) {
    if (!i.is_routed_) {
      std::cout << "Unreached node: " << '\n';
      std::cout << i << '\n';
    }
  }
  std::cout << "Solution valid: " << check_sol_val() << '\n';
}
