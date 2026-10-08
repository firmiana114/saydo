// Bounded layered search; every retained node has a complete valid prefix.
#ifndef SAYDO_ROBOT_IK_IK_BEAM_SEARCH_H_
#define SAYDO_ROBOT_IK_IK_BEAM_SEARCH_H_

#include <array>
#include <vector>
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace saydo_ik {
using Joints = std::array<float, 7>;

struct Node {
  Joints joints;
  std::vector<Joints> path;
  double cost = 0;
};

struct Result {
  bool complete = false;
  int failed_index = -1;
  std::vector<Node> frontier;
  std::vector<int> widths;
};

inline double distance(const Joints& a, const Joints& b) {
  double value = 0;
  for (int j = 0; j < 7; ++j) {
    value = std::max(value, double(std::abs(a[j] - b[j])));
  }
  return value;
}

inline Result search(
    const Joints& start,
    const std::vector<float>& limits,
    const std::function<std::vector<Joints>(int, const Joints&)>& candidates,
    const std::function<void()>& check_budget,
    int width = 8) {
  if (width < 1 || width > 16) {
    throw std::invalid_argument("beam width");
  }
  Result result;
  result.frontier.push_back({start, {}, 0});
  for (size_t i = 0; i < limits.size(); ++i) {
    if (!std::isfinite(limits[i]) || limits[i] <= 0) {
      throw std::invalid_argument("step limit");
    }
    std::vector<Node> next;
    for (const auto& parent : result.frontier) {
      check_budget();
      for (const auto& q : candidates(int(i), parent.joints)) {
        check_budget();
        bool finite = true;
        double cost = parent.cost;
        for (int j = 0; j < 7; ++j) {
          finite = finite && std::isfinite(q[j]);
          cost += std::abs(q[j] - parent.joints[j]);
        }
        if (!finite || distance(q, parent.joints) > limits[i]) {
          continue;
        }
        auto path = parent.path;
        path.push_back(q);
        Node node{q, std::move(path), cost};
        auto same = std::find_if(
            next.begin(), next.end(),
            [&](const Node& n) { return distance(q, n.joints) < .05; });
        if (same == next.end()) {
          next.push_back(std::move(node));
        } else if (cost < same->cost) {
          *same = std::move(node);
        }
      }
    }
    if (next.empty()) {
      result.failed_index = int(i);
      return result;
    }
    std::stable_sort(
        next.begin(), next.end(),
        [](const Node& a, const Node& b) { return a.cost < b.cost; });
    // Reserve some slots for distinct configurations, not only
    // near-identical cheapest solutions.
    std::vector<Node> selected;
    selected.push_back(next.front());
    for (const auto& n : next) {
      if (int(selected.size()) >= width) {
        break;
      }
      if (std::all_of(
              selected.begin(), selected.end(),
              [&](const Node& s) {
                return distance(n.joints, s.joints) >= 1.;
              })) {
        selected.push_back(n);
      }
    }
    for (const auto& n : next) {
      if (int(selected.size()) >= width) {
        break;
      }
      if (std::all_of(
              selected.begin(), selected.end(),
              [&](const Node& s) {
                return distance(n.joints, s.joints) >= .05;
              })) {
        selected.push_back(n);
      }
    }
    result.widths.push_back(int(selected.size()));
    result.frontier = std::move(selected);
  }
  result.complete = true;
  return result;
}
}  // namespace saydo_ik

#endif  // SAYDO_ROBOT_IK_IK_BEAM_SEARCH_H_
