#ifndef UTILS_HPP
#define UTILS_HPP

#include <string>
#include <tuple>
#include <vector>
struct nd {
public:
  int x_, y_, id_, demand_;
  bool is_routed_;

  nd(const int x = 0, const int y = 0, const int id = 0, const int demand = 0,
     const bool is_routed = true)
      : x_(x), y_(y), id_(id), demand_(demand), is_routed_(is_routed) {}

  friend std::ostream &operator<<(std::ostream &os, const nd &node);
};

std::ostream &operator<<(std::ostream &os, const nd &node);

struct veh {
public:
  int id_, load_, capacity_;
  double cost_ = 0;
  std::vector<int> nodes_;

  veh(const int id = 0, const int load = 0, const int capacity = 0)
      : id_(id), load_(load), capacity_(capacity) {}

  friend std::ostream &operator<<(std::ostream &os, const veh &v);

  void calc_cost(const std::vector<std::vector<double>> &distanceMatrix);
};

std::ostream &operator<<(std::ostream &os, const veh &v);

void print_veh_route(const veh &v);

struct prob {
public:
  prob(char x);

  prob(const int noc = 1000, const int demand_range = 40, const int nov = 50,
       const int capacity = 800, const int grid_range = 1000,
       std::string distribution = "uniform", const int n_clusters = 5,
       const int cluster_range = 10);

  prob(const std::string &input_path, const int nov = 4);

  std::vector<nd> nodes_;
  std::vector<veh> vehicles_;
  std::vector<std::vector<double>> dist_mtx_;
  nd depot_;
  int capacity_;
};

class sol {
public:
  sol(std::vector<nd> nodes, const std::vector<veh> &vehicles,
      std::vector<std::vector<double>> distanceMatrix);

  explicit sol(const prob &p);

  sol(const sol &s) = default;

  sol &operator=(const sol &s) = default;

  sol(sol &&s) = default;

  sol &operator=(sol &&s) = default;

  virtual ~sol() = default;

  void create_init_sol();

  bool check_sol_val() const;

  virtual void solve() = 0;

  std::tuple<bool, nd> find_closest(const veh &v) const;

  void print_sol(const std::string &option = "") const;

  std::vector<nd> get_nodes() const { return nodes_; }

  std::vector<veh> get_vehicles() const { return vehicles_; }

  std::vector<nd> nodes_;
  std::vector<veh> vehicles_;
  std::vector<std::vector<double>> dist_mtx_;
  nd depot_;
  int capacity_;
};

#endif // UTILS_HPP
