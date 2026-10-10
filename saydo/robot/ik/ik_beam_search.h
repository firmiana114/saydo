// 沿一串路点逐层挑选关节解的搜索（束搜索）：每一层只保留有限个解，
// 每个保留下来的解都有一条从起点连到它的完整路径。
//
// 为什么要“保留多个”：7 关节机械臂在同一个夹爪位姿下有很多种摆法。
// 只保留最省力的那一种时，它可能在后面某个路点走进死路（超限位、碰到
// 立杆），而另一种摆法本来走得通。所以每层除了最省力的解，还特意留下
// 与它差别大的解。
//
// 本文件不依赖厂家算法库：每个路点上有哪些关节解、某一步是否可行，都由
// 调用方通过回调给出。
#ifndef SAYDO_ROBOT_IK_IK_BEAM_SEARCH_H_
#define SAYDO_ROBOT_IK_IK_BEAM_SEARCH_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <unordered_set>
#include <vector>

#include "saydo/robot/ik/joints.h"

namespace saydo_ik {

// 每层最多保留的解的个数的上限。
constexpr int kMaxBeamWidth = 32;
// 两组关节角各关节都相差不到这么多度时视为同一个解。
constexpr double kSameSolutionDeg = .05;
// 挑选每层保留的解时依次使用的“与已选的解至少相差多少度”的门槛。
// 先用大门槛挑出差别大的摆法，再逐步放宽，最后（0）按代价补满。
constexpr std::array<double, 4> kDiversitySteps = {45., 15., 1., 0.};

// 搜索结果。
struct SearchResult {
  // 是否每个路点都找到了解。
  bool complete = false;
  // 没有解的那个路点的序号（从 0 起）；全部找到时为 -1。
  int failed_index = -1;
  // 总转动量最小的那条路径：每个路点一组关节角，单位度。
  std::vector<Joints> path;
  // 这条路径的代价：各关节转动量之和，单位度。
  double cost = 0;
  // 每一层保留了多少个解。
  std::vector<int> widths;
};

// 给出一个路点上从某个解出发可以考虑的全部关节解。
// 参数：路点序号、上一个路点（或起点）的关节角。
using CandidateSource =
    std::function<std::vector<Joints>(int, const Joints&)>;
// 判断从上一个解走到这个解的这一步是否可行（例如沿途不碰撞）。只对即将
// 被保留的解调用，所以可以放比较费时的检查。
// 参数：路点序号、上一个解、这个解。
using StepCheck = std::function<bool(int, const Joints&, const Joints&)>;

namespace internal {

// 把一组关节角按 kSameSolutionDeg 取整后合成一个数，用来快速判断重复。
inline uint64_t SolutionKey(const Joints& joints) {
  uint64_t key = 1469598103934665603ull;
  for (float value : joints) {
    const int64_t cell = std::llround(double(value) / kSameSolutionDeg);
    key = (key ^ uint64_t(cell)) * 1099511628211ull;
  }
  return key;
}

// 一层里保留的一个解：关节角、它接在上一层的第几个解后面、累计代价。
struct Node {
  Joints joints{};
  int parent = -1;
  double cost = 0;
};

// 一条候选的连接：上一层第 parent 个解 → joints。
struct Edge {
  Joints joints{};
  int parent = 0;
  double cost = 0;
  uint64_t key = 0;
  bool rejected = false;
};

}  // namespace internal

// 沿路点逐层搜索一条关节角路径。
//
// 参数：
//   start：起点（机械臂当前）的关节角，单位度。
//   limits_deg：每个路点允许的相邻两组关节角的最大变化，单位度；它的
//     长度就是路点个数。
//   candidates：见 CandidateSource。可以抛出异常来中止搜索。
//   step_ok：见 StepCheck。可以抛出异常来中止搜索。
//   width：每层最多保留多少个解，1～kMaxBeamWidth。
// 返回：SearchResult。代价是各关节转动量之和，返回的是代价最小的路径。
// 抛出：width 或 limits_deg 无效时抛 std::invalid_argument。
inline SearchResult Search(
    const Joints& start,
    const std::vector<float>& limits_deg,
    const CandidateSource& candidates,
    const StepCheck& step_ok,
    int width = 16) {
  if (width < 1 || width > kMaxBeamWidth) {
    throw std::invalid_argument("beam width");
  }
  SearchResult result;
  std::vector<std::vector<internal::Node>> layers;
  std::vector<internal::Node> frontier{{start, -1, 0}};
  for (size_t i = 0; i < limits_deg.size(); ++i) {
    if (!std::isfinite(limits_deg[i]) || limits_deg[i] <= 0) {
      throw std::invalid_argument("step limit");
    }
    std::vector<internal::Edge> edges;
    for (size_t p = 0; p < frontier.size(); ++p) {
      const internal::Node& parent = frontier[p];
      for (const Joints& q : candidates(int(i), parent.joints)) {
        bool finite = true;
        double cost = parent.cost;
        for (int j = 0; j < 7; ++j) {
          finite = finite && std::isfinite(q[j]);
          cost += std::abs(q[j] - parent.joints[j]);
        }
        if (finite && LargestJointStep(q, parent.joints) <= limits_deg[i]) {
          edges.push_back({q, int(p), cost, internal::SolutionKey(q), false});
        }
      }
    }
    std::stable_sort(
        edges.begin(), edges.end(),
        [](const internal::Edge& a, const internal::Edge& b) {
          return a.cost < b.cost;
        });
    std::vector<internal::Node> selected;
    std::unordered_set<uint64_t> taken;
    for (double apart_deg : kDiversitySteps) {
      for (internal::Edge& edge : edges) {
        if (int(selected.size()) >= width) {
          break;
        }
        if (edge.rejected || taken.count(edge.key)) {
          continue;
        }
        if (apart_deg > 0 &&
            std::any_of(
                selected.begin(), selected.end(),
                [&](const internal::Node& node) {
                  return LargestJointStep(edge.joints, node.joints) <
                         apart_deg;
                })) {
          continue;
        }
        // 同一个解经由更省力的上一个解走不通时，后面还会轮到它经由
        // 别的上一个解的连接。
        if (!step_ok(int(i), frontier[edge.parent].joints, edge.joints)) {
          edge.rejected = true;
          continue;
        }
        taken.insert(edge.key);
        selected.push_back({edge.joints, edge.parent, edge.cost});
      }
    }
    if (selected.empty()) {
      result.failed_index = int(i);
      return result;
    }
    result.widths.push_back(int(selected.size()));
    layers.push_back(selected);
    frontier = std::move(selected);
  }
  result.complete = true;
  if (layers.empty()) {
    return result;
  }
  int best = 0;
  for (size_t k = 1; k < layers.back().size(); ++k) {
    if (layers.back()[k].cost < layers.back()[best].cost) {
      best = int(k);
    }
  }
  result.cost = layers.back()[best].cost;
  result.path.resize(layers.size());
  for (int i = int(layers.size()) - 1; i >= 0; --i) {
    result.path[i] = layers[i][best].joints;
    best = layers[i][best].parent;
  }
  return result;
}

}  // namespace saydo_ik

#endif  // SAYDO_ROBOT_IK_IK_BEAM_SEARCH_H_
