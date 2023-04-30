#ifndef SA_HPP
#define SA_HPP

#include "utils.hpp"

class sa_sol : public sol {
public:
  sa_sol(const std::vector<nd> &nodes, const std::vector<veh> &vehicles,
         const std::vector<std::vector<double>> &distanceMatrix,
         const int stag_limit = 500000, const double init_temp = 5000,
         const double cooling_rate = 0.9999, const int n_reheats = 20);

  explicit sa_sol(const prob &p, const int stag_limit = 500000,
                  const double init_temp = 5000,
                  const double cooling_rate = 0.9999, const int n_reheats = 20);

  explicit sa_sol(const sol &s, int stag_limit = 500000,
                  double init_temp = 5000, double cooling_rate = 0.9999,
                  const int n_reheats = 20);

  void solve() override;

private:
  const int stag_limit_;
  const double init_temp_;
  const double cooling_rate_;
  const int n_reheats_;
  inline static bool allow_move(const double delta, const double temp);
};

#endif // SA_HPP
