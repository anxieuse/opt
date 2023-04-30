#ifndef GREEDY_HPP
#define GREEDY_HPP

#include "utils.hpp"

class nn_sol : public sol {
public:
  nn_sol(const std::vector<nd> &nodes, const std::vector<veh> &vehicles,
         const std::vector<std::vector<double>> &distanceMatrix);

  explicit nn_sol(const prob &p);

  void solve() override;
};
#endif // GREEDY_HPP
