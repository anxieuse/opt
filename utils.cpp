#include "utils.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <tuple>
#include <utility>

std::ostream &operator<<(std::ostream &os, const nd &node) {
  os << "Node Status" << '\n';
  os << "ID    : " << node.id_ << '\n';
  os << "X     : " << node.x_ << '\n';
  os << "Y     : " << node.y_ << '\n';
  os << "Demand: " << node.demand_ << '\n';
  os << '\n';
  return os;
}

void veh::calc_cost(const std::vector<std::vector<double>> &distanceMatrix) {
  cost_ = 0;
  for (size_t i = 0; i < nodes_.size() - 1; i++) {
    cost_ += distanceMatrix[nodes_[i]][nodes_[i + 1]];
  }
}

std::ostream &operator<<(std::ostream &os, const veh &v) {
  os << "Vehicle Status" << '\n';
  os << "Cost    : " << v.cost_ << '\n';
  os << "ID      : " << v.id_ << '\n';
  os << "Load    : " << v.load_ << '\n';
  os << "Capacity: " << v.capacity_ << '\n';
  os << "Path    : ";
  for (size_t i = 0; i < v.nodes_.size() - 1; ++i) {
    os << v.nodes_[i] << " ---> ";
  }
  os << "0";
  os << '\n' << '\n';
  return os;
}

void print_veh_route(const veh &v) {
  for (size_t i = 0; i < v.nodes_.size() - 1; ++i) {
    std::cout << v.nodes_[i] << " ---> ";
  }
  std::cout << "0";
  std::cout << '\n' << '\n';
}

sol::sol(std::vector<nd> nodes, const std::vector<veh> &vehicles,
         std::vector<std::vector<double>> distanceMatrix)
    : nodes_(std::move(nodes)), vehicles_(vehicles),
      dist_mtx_(std::move(distanceMatrix)) {
  depot_ = nodes_[0];
  capacity_ = vehicles[0].load_;
}

sol::sol(const prob &p)
    : nodes_(p.nodes_), vehicles_(p.vehicles_), dist_mtx_(p.dist_mtx_),
      capacity_(p.capacity_) {
  depot_ = nodes_[0];
}

void sol::create_init_sol() {
  for (auto &v : vehicles_) {
    while (true) {
      const auto [found, closest_node] = find_closest(v);
      if (found && v.load_ - closest_node.demand_ >= 0) { // }.2*capacity){
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
}

std::tuple<bool, nd> sol::find_closest(const veh &v) const {
  double cost = std::numeric_limits<double>::max();
  size_t id = 0;
  bool found = false;
  for (size_t j = 0; j < dist_mtx_[0].size(); j++) {
    if (!nodes_[j].is_routed_ && nodes_[j].demand_ <= v.load_ &&
        dist_mtx_[v.nodes_.back()][j] < cost) {
      cost = dist_mtx_[v.nodes_.back()][j];
      id = j;
      found = true;
    }
  }
  if (found) {
    return {true, nodes_[id]};
  }
  return {false, nd()};
}

bool sol::check_sol_val() const {
  std::vector<bool> check_nodes(nodes_.size(), false);
  check_nodes[0] = true;
  for (const auto &v : vehicles_) {
    int load = capacity_;
    for (const auto &n : v.nodes_) {
      load -= nodes_[n].demand_;
      check_nodes[n] = true;
    }
    if (load < 0) {
      return false;
    }
  }
  return std::all_of(std::begin(check_nodes), std::end(check_nodes),
                     [](const bool b) { return b; });
}

prob::prob(const std::string &input_path, const int nov) {
  /*
  Function to read the input file and initialize the problem.

  Input format:
NAME : sample 1
COMMENT : sample instance 1
TYPE : CVRP
DIMENSION : 6
EDGE_WEIGHT_TYPE : EUC_2D
CAPACITY : 30
NODE_COORD_SECTION
1 38 46
2 59 46
3 96 42
4 47 61
5 26 15
6 66 6
DEMAND_SECTION
1 0
2 16
3 18
4 1
5 13
6 8
DEPOT_SECTION
1
-1
EOF

  Depot is located in node 1.
  Dimension is the number of nodes.
  */

  std::ifstream ifile(input_path);
  std::string line;
  std::string word;
  std::vector<std::string> words;
  std::vector<std::vector<double>> coordinates;
  std::vector<int> demands;
  int dimension = 0;
  int capacity = 0;
  int depot = 0;
  while (std::getline(ifile, line)) {
    std::istringstream iss(line);
    while (iss >> word) {
      words.push_back(word);
    }
    if (words[0] == "DIMENSION") {
      dimension = std::stoi(words[2]);
    } else if (words[0] == "CAPACITY") {
      capacity = std::stoi(words[2]);
    } else if (words[0] == "NODE_COORD_SECTION") {
      for (int i = 0; i < dimension; i++) {
        std::getline(ifile, line);
        std::istringstream iss(line);
        int node_id = 0;
        iss >> node_id;
        std::vector<double> coord;
        while (iss >> word) {
          coord.push_back(std::stod(word));
        }
        coordinates.push_back(coord);
      }
    } else if (words[0] == "DEMAND_SECTION") {
      for (int i = 0; i < dimension; i++) {
        std::getline(ifile, line);
        std::istringstream iss(line);
        int demand_id = 0;
        iss >> demand_id;
        while (iss >> word) {
          demands.push_back(std::stoi(word));
        }
      }
    }
    words.clear();
  }
  ifile.close();

  // Create nodes
  nd depot_node(coordinates[0][0], coordinates[0][1], 0, 0, true);
  nodes_.push_back(depot_node);
  for (int i = 1; i < dimension; i++) {
    nd node(coordinates[i][0], coordinates[i][1], i, demands[i], false);
    nodes_.push_back(node);
  }

  // Create vehicles
  for (int i = 0; i < nov; i++) {
    veh vehicle(i, capacity, capacity);
    vehicles_.push_back(vehicle);
    vehicles_[i].nodes_.push_back(0);
  }

  // Create distance matrix
  for (const auto &n1 : nodes_) {
    std::vector<double> row;
    for (const auto &n2 : nodes_) {
      double distance =
          std::sqrt(std::pow(n1.x_ - n2.x_, 2) + std::pow(n1.y_ - n2.y_, 2));
      row.push_back(distance);
    }
    dist_mtx_.push_back(row);
  }

  capacity_ = capacity;

  return;
  // print the problem
  // nodes
  // std::cout << "Nodes: " << std::endl;
  // for (const auto &n : nodes_) {
  //   std::cout << n.id_ << " " << n.x_ << " " << n.y_ << " " << n.demand_
  //             << std::endl;
  // }

  // // distance matrix
  // std::cout << "Distance matrix: " << std::endl;
  // for (const auto &row : distanceMatrix_) {
  //   for (const auto &col : row) {
  //     std::cout << col << " ";
  //   }
  //   std::cout << std::endl;
  // }

  // // vehicle
  // std::cout << "Vehicle: " << std::endl;
  // for (const auto &v : vehicles_) {
  //   std::cout << v.id_ << " " << v.capacity_ << " " << v.load_ << std::endl;
  // }
}

// empty constructor
prob::prob(char x) {}

prob::prob(const int noc, const int demand_range, const int nov,
           const int capacity, const int grid_range, std::string distribution,
           const int n_clusters, const int cluster_range) {
  std::random_device rd;  // obtain a random number from hardware
  std::mt19937 eng(rd()); // seed the generator
  std::uniform_int_distribution<int> ran(-grid_range,
                                         grid_range); // define the range
  std::uniform_int_distribution<int> ran_d(0,
                                           demand_range); // define the range
  std::uniform_int_distribution<int> ran_c(-cluster_range, cluster_range);
  nd depot(0, 0, 0, 0, true);
  this->capacity_ = capacity;

  nodes_.push_back(depot);

  if (distribution != "uniform" && distribution != "cluster") {
    distribution = "uniform";
  }
  if (distribution == "uniform") {
    for (int i = 1; i <= noc; ++i) {
      nodes_.emplace_back(ran(eng), ran(eng), i, ran_d(eng), false);
    }
  } else if (distribution == "cluster") {
    int id = 1;
    int n_p_c = noc / n_clusters;
    int remain = noc % n_clusters;
    for (int i = 0; i < n_clusters; i++) {
      int x = ran(eng);
      int y = ran(eng);
      for (int j = 0; j < n_p_c; j++) {
        nodes_.emplace_back(x + ran_c(eng), y + ran_c(eng), id, ran_d(eng),
                            false);
        id++;
      }
    }
    int x = ran(eng);
    int y = ran(eng);
    for (int j = 0; j < remain; j++) {
      nodes_.emplace_back(x + ran_c(eng), y + ran_c(eng), id, ran_d(eng),
                          false);
      id++;
    }
  }

  std::vector<double> tmp(nodes_.size());
  for (size_t i = 0; i < nodes_.size(); ++i) {
    dist_mtx_.push_back(tmp);
  }
  for (size_t i = 0; i < nodes_.size(); ++i) {
    for (size_t j = i; j < nodes_.size(); ++j) {
      dist_mtx_[i][j] = sqrt(pow((nodes_[i].x_ - nodes_[j].x_), 2) +
                             pow((nodes_[i].y_ - nodes_[j].y_), 2));
      dist_mtx_[j][i] = dist_mtx_[i][j];
    }
  }

  int load = capacity_;
  for (int i = 0; i < nov; ++i) {
    vehicles_.emplace_back(i, load, capacity_);
    vehicles_[i].nodes_.push_back(0);
  }

  return;

  // // nodes
  // std::cout << "Nodes: " << std::endl;
  // for (const auto &n : nodes_) {
  //   std::cout << n.id_ << " " << n.x_ << " " << n.y_ << " " << n.demand_
  //             << std::endl;
  // }

  // // distance matrix
  // std::cout << "Distance matrix: " << std::endl;
  // for (const auto &row : distanceMatrix_) {
  //   for (const auto &col : row) {
  //     std::cout << col << " ";
  //   }
  //   std::cout << std::endl;
  // }

  // // vehicle
  // std::cout << "Vehicle: " << std::endl;
  // for (const auto &v : vehicles_) {
  //   std::cout << v.id_ << " " << v.capacity_ << " " << v.load_ << std::endl;
  // }
}

void sol::print_sol(const std::string &option) const {
  double total_cost = 0;
  for (const auto &v : vehicles_) {
    total_cost += v.cost_;
    if (option == "status") {
      print_veh_route(v);
    } else if (option == "route") {
      std::cout << "Vehicle ID: " << v.id_ << " | ";
      print_veh_route(v);
    }
  }
  bool valid = check_sol_val();
  std::cout << "Total solution cost: " << total_cost << '\n';
  std::cout << "Solution validity  : " << valid << '\n';
  if (!valid) {
    for (const auto &i : nodes_) {
      if (!i.is_routed_) {
        std::cout << "Unreached node: " << '\n';
        std::cout << i << '\n';
      }
    }
  }
}
