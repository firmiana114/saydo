// 只调用厂家算法库的计算接口，不连接机械臂，不发送任何运动。
#include "saydo/robot/ik/arm_angle_path.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "rm_interface.h"
#include "saydo/robot/ik/arm_collision.h"
#include "saydo/robot/ik/ik_beam_search.h"
#include "saydo/robot/ik/joints.h"

namespace saydo_ik {
namespace {

// 把所有手肘摆法都列出来时，臂角每隔多少度取一个。
constexpr float kArmAngleStepDeg = 5;
// 沿着上一个解继续走时，臂角相对上一个解可以偏开多少度。0 是保持不变，
// 其余几个让手肘能一点点转开，绕过限位或障碍物。
constexpr std::array<float, 5> kNearbyArmAnglesDeg = {0, 2, -2, 5, -5};
// 相邻关节角变化上限不超过这个值的路点是直线段上的取样点：只沿着上一个
// 解继续走，不重新列出所有摆法。单位度。
constexpr float kLinearStepLimitDeg = 10;
// 逆解结果换算回夹爪位姿后允许的偏差。
constexpr double kPositionToleranceM = .002;
constexpr double kOrientationToleranceDeg = 2;
// 厂家求解函数的调用次数上限。
constexpr int kMaxSolverCalls = 4000000;
// 每层保留多少个解。
constexpr int kBeamWidth = 16;

// 一个路点上各种原因各排除了多少个解。
struct RejectCounts {
  int solver = 0;
  int limits = 0;
  int self_collision = 0;
  int pose = 0;
  int continuity = 0;
  int obstacle = 0;
};

// 一个路点把所有摆法列出来之后的统计。
struct Enumeration {
  bool done = false;
  // 通过限位和障碍物检查的全部关节解。
  std::vector<Joints> solutions;
  // 够得到的个数：换算回夹爪位姿与目标一致、不超限位（不论是否碰到
  // 障碍物）。
  int reachable = 0;
};

// 把角度换到 (-180, 180] 之内，单位度。
float WrapDeg(float value_deg) {
  while (value_deg > 180) {
    value_deg -= 360;
  }
  while (value_deg <= -180) {
    value_deg += 360;
  }
  return value_deg;
}

// 读取一个位姿：位置 3 个数（米）加欧拉角 3 个数（弧度）。
bool ReadPose(rm_pose_t* pose) {
  if (!(std::cin >> pose->position.x >> pose->position.y >>
        pose->position.z >> pose->euler.rx >> pose->euler.ry >>
        pose->euler.rz)) {
    return false;
  }
  for (float value :
       {pose->position.x, pose->position.y, pose->position.z, pose->euler.rx,
        pose->euler.ry, pose->euler.rz}) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  pose->quaternion = rm_algo_euler2quaternion(pose->euler);
  return true;
}

// 两个位姿朝向之间的夹角，单位度；算不出时返回 NaN。
double OrientationErrorDeg(const rm_pose_t& a, const rm_pose_t& b) {
  const rm_quat_t qa = rm_algo_euler2quaternion(a.euler);
  const rm_quat_t qb = rm_algo_euler2quaternion(b.euler);
  const double dot = double(qa.w) * qb.w + double(qa.x) * qb.x +
                     double(qa.y) * qb.y + double(qa.z) * qb.z;
  const double norm_a = std::sqrt(
      double(qa.w) * qa.w + double(qa.x) * qa.x + double(qa.y) * qa.y +
      double(qa.z) * qa.z);
  const double norm_b = std::sqrt(
      double(qb.w) * qb.w + double(qb.x) * qb.x + double(qb.y) * qb.y +
      double(qb.z) * qb.z);
  if (!(norm_a > 0 && norm_b > 0)) {
    return NAN;
  }
  return 2 * std::acos(std::min(1., std::abs(dot / (norm_a * norm_b)))) *
         180 / std::acos(-1.);
}

// 在每个路点上给出候选的关节解，并检查某一步是否可行。
class ArmAngleSolver {
 public:
  using Clock = std::chrono::steady_clock;

  // poses 是各路点的夹爪位姿，limits_deg 是各路点相邻关节角变化上限，
  // low_deg / high_deg 是真实的关节限位，deadline 是搜索的截止时刻。
  ArmAngleSolver(
      std::vector<rm_pose_t> poses,
      std::vector<float> limits_deg,
      ArmModel model,
      const float* low_deg,
      const float* high_deg,
      Clock::time_point deadline)
      : poses_(std::move(poses)),
        limits_deg_(std::move(limits_deg)),
        model_(std::move(model)),
        deadline_(deadline),
        rejected_(poses_.size()),
        enumerations_(poses_.size()) {
    std::copy(low_deg, low_deg + 7, low_deg_.begin());
    std::copy(high_deg, high_deg + 7, high_deg_.begin());
  }

  // 第 index 个路点上，从 parent 出发可以考虑的关节解。
  // 超出时间或调用次数预算时抛 std::runtime_error。
  std::vector<Joints> Candidates(int index, const Joints& parent) {
    CheckBudget();
    active_index_ = index;
    std::vector<Joints> out;
    float reference[ARM_DOF]{};
    std::copy(parent.begin(), parent.end(), reference);
    float parent_arm_angle_deg = 0;
    // 先沿着上一个解继续走：臂角不变或只偏开一点，手肘弯向不变。
    if (rm_algo_calculate_arm_angle_from_config_rm75(
            reference, &parent_arm_angle_deg) == 0 &&
        std::isfinite(parent_arm_angle_deg)) {
      for (float offset_deg : kNearbyArmAnglesDeg) {
        Joints raw{};
        if (!SolveRaw(
                index, WrapDeg(parent_arm_angle_deg + offset_deg),
                parent[3] > 0, &raw)) {
          continue;
        }
        std::vector<Joints> variants;
        int reachable = 0;
        Expand(index, raw, &variants, &reachable);
        // 同一臂角下的几种变体里，只有离上一个解最近的那个算“继续走”。
        const Joints* nearest = nullptr;
        for (const Joints& q : variants) {
          if (nearest == nullptr ||
              LargestJointStep(q, parent) <
                  LargestJointStep(*nearest, parent)) {
            nearest = &q;
          }
        }
        if (nearest == nullptr) {
          continue;
        }
        if (LargestJointStep(*nearest, parent) > limits_deg_[index]) {
          ++rejected_[index].continuity;
          continue;
        }
        out.push_back(*nearest);
      }
    }
    // 允许大幅改变摆法的路点：把所有臂角、所有分支的解都列出来。
    if (limits_deg_[index] > kLinearStepLimitDeg) {
      const std::vector<Joints>& all = Enumerate(index).solutions;
      out.insert(out.end(), all.begin(), all.end());
    }
    return out;
  }

  // 从 parent 走到第 index 个路点上的解 q 这一步是否可行：q 不自碰撞、
  // 沿关节角直线走过去途中不碰障碍物。（q 换算回夹爪位姿与目标一致，
  // 已在 SolveRaw 里核对过。）超出预算时抛 std::runtime_error。
  bool StepOk(int index, const Joints& parent, const Joints& q) {
    CheckBudget();
    float joints[ARM_DOF]{};
    std::copy(q.begin(), q.end(), joints);
    if (rm_algo_safety_robot_self_collision_detection(joints) != 0) {
      ++rejected_[index].self_collision;
      return false;
    }
    if (!MotionClear(model_, parent, q)) {
      ++rejected_[index].obstacle;
      return false;
    }
    return true;
  }

  // 把第 index 个路点的所有摆法列出来（结果会记住，重复调用不重算）。
  const Enumeration& Enumerate(int index) {
    Enumeration& enumeration = enumerations_[index];
    if (enumeration.done) {
      return enumeration;
    }
    for (float arm_angle_deg = -180; arm_angle_deg < 180;
         arm_angle_deg += kArmAngleStepDeg) {
      for (bool elbow_positive : {false, true}) {
        Joints raw{};
        if (SolveRaw(index, arm_angle_deg, elbow_positive, &raw)) {
          Expand(
              index, raw, &enumeration.solutions,
              &enumeration.reachable);
        }
      }
    }
    enumeration.done = true;
    return enumeration;
  }

  // 一组关节角对应的臂角，单位度；算不出时返回 NaN。
  static float ArmAngleDeg(const Joints& q) {
    float joints[ARM_DOF]{};
    std::copy(q.begin(), q.end(), joints);
    float arm_angle_deg = NAN;
    if (rm_algo_calculate_arm_angle_from_config_rm75(
            joints, &arm_angle_deg) != 0) {
      return NAN;
    }
    return arm_angle_deg;
  }

  int calls() const { return calls_; }
  int active_index() const { return active_index_; }
  const RejectCounts& rejected(int index) const { return rejected_[index]; }

 private:
  // 超出时间或调用次数预算时抛 std::runtime_error。
  void CheckBudget() const {
    if (calls_ >= kMaxSolverCalls || Clock::now() >= deadline_) {
      throw std::runtime_error("budget");
    }
  }

  // 第 index 个路点在指定臂角（度）、指定手肘弯向下的一个解。厂家函数
  // 返回的手肘弯向与参考值一致，肩和腕的分支由 Expand 补全。
  bool SolveRaw(
      int index, float arm_angle_deg, bool elbow_positive, Joints* out) {
    rm_inverse_kinematics_params_t input{};
    input.q_pose = poses_[index];
    input.flag = 1;
    // 参考值避开第 2、4、6 关节为 0 的奇异位形，否则厂家函数求解失败。
    const float reference[7] = {
        0, -45, 0, elbow_positive ? 90.f : -90.f, 0, -45, 0};
    std::copy(reference, reference + 7, input.q_in);
    float solution[ARM_DOF]{};
    ++calls_;
    if (rm_algo_inverse_kinematics_rm75_for_arm_angle(
            input, arm_angle_deg, solution) != 0) {
      ++rejected_[index].solver;
      return false;
    }
    for (int j = 0; j < 7; ++j) {
      if (!std::isfinite(solution[j])) {
        ++rejected_[index].solver;
        return false;
      }
      (*out)[j] = solution[j];
    }
    if (((*out)[3] > 0) != elbow_positive) {
      ++rejected_[index].solver;
      return false;
    }
    // 目标够不到时（例如太远），厂家函数在限位放宽后仍然报成功，给出的
    // 却不是这个位姿的解。所以每个解都换算回夹爪位姿核对一遍。Expand
    // 补出的其余分支与它的夹爪位姿完全相同，不必重复核对。
    const rm_pose_t achieved = rm_algo_forward_kinematics(nullptr, solution);
    const rm_pose_t& target = poses_[index];
    const double distance_m = std::sqrt(
        std::pow(double(achieved.position.x) - target.position.x, 2) +
        std::pow(double(achieved.position.y) - target.position.y, 2) +
        std::pow(double(achieved.position.z) - target.position.z, 2));
    const double angle_deg = OrientationErrorDeg(achieved, target);
    if (!std::isfinite(distance_m) || !std::isfinite(angle_deg) ||
        distance_m > kPositionToleranceM ||
        angle_deg > kOrientationToleranceDeg) {
      ++rejected_[index].pose;
      return false;
    }
    return true;
  }

  // 由一个解补全同一臂角下的其余分支，把通过限位和障碍物检查的加入 out。
  //
  // 肩部三个关节的转轴交于一点，把第 1、3 关节各加 180 度、第 2 关节
  // 取反，夹爪位姿不变；腕部的第 5、6、7 关节同理。第 7 关节可以转
  // 正负一整圈，所以每个分支还有相差 360 度的两种。reachable 累加
  // 通过限位检查的个数（raw 已核对过确实到达目标位姿）。
  void Expand(
      int index,
      const Joints& raw,
      std::vector<Joints>* out,
      int* reachable) {
    for (bool flip_shoulder : {false, true}) {
      for (bool flip_wrist : {false, true}) {
        Joints q = raw;
        if (flip_shoulder) {
          q[0] += 180;
          q[1] = -q[1];
          q[2] += 180;
        }
        if (flip_wrist) {
          q[4] += 180;
          q[5] = -q[5];
          q[6] += 180;
        }
        for (int j = 0; j < 7; ++j) {
          q[j] = WrapDeg(q[j]);
        }
        for (float turn_deg : {0.f, 360.f, -360.f}) {
          Joints candidate = q;
          candidate[6] += turn_deg;
          bool inside = true;
          for (int j = 0; j < 7; ++j) {
            inside = inside && candidate[j] >= low_deg_[j] &&
                     candidate[j] <= high_deg_[j];
          }
          if (!inside) {
            ++rejected_[index].limits;
            continue;
          }
          ++*reachable;
          if (!ConfigurationClear(model_, candidate)) {
            ++rejected_[index].obstacle;
            continue;
          }
          out->push_back(candidate);
        }
      }
    }
  }

  std::vector<rm_pose_t> poses_;
  std::vector<float> limits_deg_;
  ArmModel model_;
  Clock::time_point deadline_;
  std::array<float, 7> low_deg_{};
  std::array<float, 7> high_deg_{};
  std::vector<RejectCounts> rejected_;
  std::vector<Enumeration> enumerations_;
  int calls_ = 0;
  int active_index_ = 0;
};

}  // namespace

int RunArmAnglePath(const float* start_joints_deg) {
  int count = 0, budget_ms = 0;
  if (!(std::cin >> count >> budget_ms) || count < 1 || count > 500 ||
      budget_ms < 1 || budget_ms > 3000) {
    return 2;
  }
  std::vector<rm_pose_t> poses(count);
  std::vector<float> limits_deg(count);
  for (int i = 0; i < count; ++i) {
    if (!ReadPose(&poses[i]) || !(std::cin >> limits_deg[i]) ||
        !std::isfinite(limits_deg[i]) || limits_deg[i] <= 0 ||
        limits_deg[i] > 360) {
      return 2;
    }
  }
  ArmModel model;
  if (!ReadArmModel(std::cin, &model)) {
    return 2;
  }
  float low_deg[ARM_DOF]{}, high_deg[ARM_DOF]{};
  rm_algo_get_joint_min_limit(low_deg);
  rm_algo_get_joint_max_limit(high_deg);
  // 厂家函数挑中的分支超出限位时不会改试别的分支，只报失败。这里把
  // 算法库里的限位放宽，让它总能给出一个解；真实限位由 Expand 逐个核对。
  float wide_low_deg[ARM_DOF]{}, wide_high_deg[ARM_DOF]{};
  for (int j = 0; j < ARM_DOF; ++j) {
    wide_high_deg[j] = j == 6 ? 720 : 360;
    wide_low_deg[j] = -wide_high_deg[j];
  }
  rm_algo_set_joint_min_limit(wide_low_deg);
  rm_algo_set_joint_max_limit(wide_high_deg);
  ArmAngleSolver solver(
      poses, limits_deg, model, low_deg, high_deg,
      ArmAngleSolver::Clock::now() + std::chrono::milliseconds(budget_ms));
  Joints start{};
  std::copy(start_joints_deg, start_joints_deg + 7, start.begin());
  SearchResult result;
  try {
    result = Search(
        start, limits_deg,
        [&solver](int index, const Joints& parent) {
          return solver.Candidates(index, parent);
        },
        [&solver](int index, const Joints& parent, const Joints& q) {
          return solver.StepOk(index, parent, q);
        },
        kBeamWidth);
  } catch (const std::runtime_error&) {
    std::cout << "SEARCH_FAIL " << solver.active_index()
              << " budget_exhausted " << solver.calls() << '\n';
    return 4;
  }
  for (size_t i = 0; i < result.widths.size(); ++i) {
    std::cout << "BEAM " << i << ' ' << result.widths[i] << '\n';
  }
  if (!result.complete) {
    // 把卡住的那个路点的所有摆法列出来，分清三种情况：任何摆法都够不到
    // 或超限位；够得到但都碰到障碍物（桌面、立杆、另一台臂）；有可行的
    // 摆法但与前面的路接不上。
    const int failed = result.failed_index;
    const Enumeration& all = solver.Enumerate(failed);
    const RejectCounts& rejected = solver.rejected(failed);
    const char* kind =
        all.reachable == 0
            ? "out_of_reach"
            : all.solutions.empty() ? "blocked_by_obstacle"
                                    : "no_continuous_solution";
    std::cout << "SEEDS " << failed << ' ' << all.reachable << ' '
              << all.solutions.size() << '\n';
    std::cout << "REJECT " << failed << ' ' << rejected.solver << ' '
              << rejected.limits << ' ' << rejected.self_collision << ' '
              << rejected.pose << ' ' << rejected.continuity << ' '
              << rejected.obstacle << '\n';
    std::cout << "SEARCH_FAIL " << failed << ' ' << kind << ' '
              << solver.calls() << '\n';
    return 1;
  }
  std::cout << "CALLS " << solver.calls() << '\n';
  for (int i = 0; i < count; ++i) {
    float joints[ARM_DOF]{};
    std::copy(result.path[i].begin(), result.path[i].end(), joints);
    const rm_pose_t achieved = rm_algo_forward_kinematics(nullptr, joints);
    std::cout << "IK 0 0";
    for (int j = 0; j < 7; ++j) {
      std::cout << ' ' << joints[j];
    }
    std::cout << '\n';
    std::cout << "VERIFY " << i << ' ' << achieved.position.x << ' '
              << achieved.position.y << ' ' << achieved.position.z << ' '
              << achieved.euler.rx << ' ' << achieved.euler.ry << ' '
              << achieved.euler.rz << '\n';
    std::cout << "ARM_ANGLE " << i << ' '
              << ArmAngleSolver::ArmAngleDeg(result.path[i]) << '\n';
  }
  return 0;
}

}  // namespace saydo_ik
