#include <bits/stdc++.h>

#include "greedy.hpp"
#include "simulated_annealing.hpp"
#include "utils.hpp"

int main(int argc, char **argv) {
  std::string input_path;
  int novargs = 4;
  if (argc >= 2) {
    input_path = argv[1];
    if (argc >= 3) {
      novargs = std::stoi(argv[2]);
    }
  }

  prob p('#');
  if (!input_path.empty()) {
    std::cout << "Reading from file: " << input_path << '\n';
    p = prob(input_path, novargs);
  } else {
    std::cout << "Usage: ./cvrp input.vrp veh_num" << '\n';
    return 1;
  }

  std::vector<std::pair<double, double>> results;

  {
    std::cout << "NN: " << '\n';
    nn_sol nn(p.nodes_, p.vehicles_, p.dist_mtx_);

    auto start = std::chrono::high_resolution_clock::now();
    nn.solve();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() * 1e9 << " ns" << '\n';
    std::cout << '\n';

    double cost = std::accumulate(
        std::begin(nn.vehicles_), std::end(nn.vehicles_), 0.0,
        [](const double sum, const veh &v) { return sum + v.cost_; });
    results.push_back(std::make_pair(cost, elapsed.count()));
  }

  {
    std::cout << "SA: " << '\n';
    sa_sol sa(p, 500000, 50000, 0.9899, 20);
    auto start = std::chrono::high_resolution_clock::now();
    sa.solve();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() * 1e9 << " ns" << '\n';
    std::cout << '\n';

    double cost = std::accumulate(
        std::begin(sa.vehicles_), std::end(sa.vehicles_), 0.0,
        [](const double sum, const veh &v) { return sum + v.cost_; });
    results.push_back(std::make_pair(cost, elapsed.count()));
  }

  {
    std::cout << "Hybrid: " << '\n';
    nn_sol hyb(p);
    hyb.solve();
    auto s3 = hyb;
    sa_sol sa4hyb(s3, 500000, 50, 0.9899, 20);
    auto start = std::chrono::high_resolution_clock::now();
    sa4hyb.solve();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Elapsed time: " << elapsed.count() * 1e9 << " ns" << '\n';
    std::cout << '\n';

    double cost = std::accumulate(
        std::begin(sa4hyb.vehicles_), std::end(sa4hyb.vehicles_), 0.0,
        [](const double sum, const veh &v) { return sum + v.cost_; });
    results.push_back(std::make_pair(cost, elapsed.count()));
  }

  double prec = 2;
  std::cout << std::fixed << std::setprecision(prec);
  for (auto r : results) {
    std::cout << ceil(r.first) << "\t0\t" << r.second * 1e6 << '\t';
  }

  return 0;
}
