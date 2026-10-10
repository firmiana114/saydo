// 机械臂和夹爪离桌面、立杆有多近：与 saydo/robot/collision_model.py 同一套
// 算法的 C++ 版本，供逆解搜索在挑选手肘摆法时就地排除会碰撞的姿态。
//
// 外形数据（运动学参数、胶囊体、障碍物位置）全部由 Python 一侧传入，这里
// 不保存任何尺寸，所以两边不会各有一份数据。最终是否放行仍由 Python 的
// collision_model.check_path 决定；这里的判断比它严 0.2 毫米，避免两边
// 因浮点精度不同而在边界上结论相反。
//
// 只做几何计算，不依赖厂家算法库。
#ifndef SAYDO_ROBOT_IK_ARM_COLLISION_H_
#define SAYDO_ROBOT_IK_ARM_COLLISION_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <istream>
#include <limits>
#include <vector>

#include "saydo/robot/ik/joints.h"

namespace saydo_ik {

// 位姿的前三行（3×4，按行排列），长度单位米。
using Transform = std::array<double, 12>;

// 相邻两个姿态之间按关节角直线取样的间隔，单位度。与
// collision_model.PATH_STEP_DEG 相同。
constexpr double kPathStepDeg = 2.;
// 比 Python 的判断多留的余量，单位米。
constexpr double kStricterM = .0002;
// 桌面检查的计算容差，单位米。与 collision_model.check_configuration 相同。
constexpr double kTableToleranceM = .0005;
// 胶囊体挂在夹爪参考点坐标系上时用的坐标系序号；0～7 是底座和 7 个连杆。
constexpr int kToolFrame = 8;
// 另一台臂占着的空间最多用多少个胶囊体表示。
constexpr int kMaxOtherArmCapsules = 512;
// 给另一台臂的胶囊体算包围盒时，在余量之外再多留的距离，单位米。
constexpr double kOtherArmBoxSlackM = .002;

// 一个胶囊体：一条线段加一个半径，坐标在它所挂的坐标系里，单位米。
struct Capsule {
  int frame = 0;
  std::array<double, 3> start{};
  std::array<double, 3> end{};
  double radius_m = 0;
  bool table_checked = false;
};

// 另一台臂占着的一块空间：一个胶囊体，坐标在本臂的基座坐标系里，单位米。
// low / high 是它的包围盒，已按自身半径和两臂之间的余量向外扩过，用来
// 快速跳过离得远的胶囊体。
struct ObstacleCapsule {
  std::array<double, 3> start{};
  std::array<double, 3> end{};
  double radius_m = 0;
  std::array<double, 3> low{};
  std::array<double, 3> high{};
};

// 一台臂的外形和它周围要避开的东西，坐标都在这台臂的基座坐标系里。
struct ArmModel {
  // 改进型 DH 参数，每行是 (alpha 度, a 米, d 米)。
  std::vector<std::array<double, 3>> dh;
  // 夹爪参考点在法兰坐标系里的位姿。
  Transform flange_T_tool{};
  bool has_table = false;
  double table_z_m = 0;
  bool has_pole = false;
  double pole_x_m = 0;
  double pole_y_m = 0;
  // 立杆截面的外接圆半径，以及手臂与立杆表面之间至少要留的距离。
  double pole_radius_m = 0;
  double pole_margin_m = 0;
  std::vector<Capsule> capsules;
  // 另一台臂占着的空间，以及本臂与它之间至少要留的距离。
  std::vector<ObstacleCapsule> other_arm;
  double other_arm_margin_m = 0;

  // 有没有任何需要检查的障碍物。
  bool HasObstacles() const {
    return has_table || has_pole || !other_arm.empty();
  }
};

// 一个姿态下离障碍物的间距，单位米；不检查的一项为正无穷。离另一台臂
// 的间距只在小于“余量 + 2 毫米”时才是准确值，更远时可能报为正无穷。
struct Clearance {
  double table_m = std::numeric_limits<double>::infinity();
  double pole_m = std::numeric_limits<double>::infinity();
  double other_arm_m = std::numeric_limits<double>::infinity();
};

// 两条线段之间的最短距离，单位米。长度为零的线段按一个点处理。
inline double SegmentDistance(
    const std::array<double, 3>& a0, const std::array<double, 3>& a1,
    const std::array<double, 3>& b0, const std::array<double, 3>& b1) {
  const double tiny = 1e-12;
  double d1[3], d2[3], r[3];
  double a = 0, e = 0, b = 0, c = 0, f = 0;
  for (int k = 0; k < 3; ++k) {
    d1[k] = a1[k] - a0[k];
    d2[k] = b1[k] - b0[k];
    r[k] = a0[k] - b0[k];
    a += d1[k] * d1[k];
    e += d2[k] * d2[k];
    b += d1[k] * d2[k];
    c += d1[k] * r[k];
    f += d2[k] * r[k];
  }
  // 先求两条直线上最近的一对点，再把落在线段外的那一端夹回端点。
  const double denominator = a * e - b * b;
  double s = denominator > tiny && a > tiny
                 ? std::clamp((b * f - c * e) / denominator, 0., 1.)
                 : 0.;
  double t = e > tiny ? (b * s + f) / e : 0.;
  if (t < 0) {
    s = a > tiny ? std::clamp(-c / a, 0., 1.) : 0.;
  } else if (t > 1) {
    s = a > tiny ? std::clamp((b - c) / a, 0., 1.) : 0.;
  }
  t = std::clamp(t, 0., 1.);
  double squared = 0;
  for (int k = 0; k < 3; ++k) {
    const double gap = r[k] + s * d1[k] - t * d2[k];
    squared += gap * gap;
  }
  return std::sqrt(squared);
}

// 两个 3×4 位姿相乘：返回 a·b。
inline Transform Multiply(const Transform& a, const Transform& b) {
  Transform out{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 4; ++column) {
      double value = column == 3 ? a[row * 4 + 3] : 0;
      for (int k = 0; k < 3; ++k) {
        value += a[row * 4 + k] * b[k * 4 + column];
      }
      out[row * 4 + column] = value;
    }
  }
  return out;
}

// 把坐标系里的一个点换到基座坐标系，单位米。
inline std::array<double, 3> Apply(
    const Transform& frame, const std::array<double, 3>& point) {
  std::array<double, 3> out{};
  for (int row = 0; row < 3; ++row) {
    out[row] = frame[row * 4 + 3];
    for (int k = 0; k < 3; ++k) {
      out[row] += frame[row * 4 + k] * point[k];
    }
  }
  return out;
}

// 计算底座、7 个连杆和夹爪参考点共 9 个坐标系在基座坐标系里的位姿。
// 输入关节角单位度；第 0 个是底座，第 7 个是法兰，第 8 个是夹爪参考点。
inline std::array<Transform, 9> LinkFrames(
    const ArmModel& model, const Joints& joints_deg) {
  const double to_rad = std::acos(-1.) / 180;
  std::array<Transform, 9> frames{};
  Transform current{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
  frames[0] = current;
  for (size_t j = 0; j < 7 && j < model.dh.size(); ++j) {
    const double alpha = model.dh[j][0] * to_rad;
    const double a_m = model.dh[j][1], d_m = model.dh[j][2];
    const double theta = joints_deg[j] * to_rad;
    const double cos_a = std::cos(alpha), sin_a = std::sin(alpha);
    const double cos_t = std::cos(theta), sin_t = std::sin(theta);
    const Transform step{
        cos_t, -sin_t, 0, a_m,
        sin_t * cos_a, cos_t * cos_a, -sin_a, -sin_a * d_m,
        sin_t * sin_a, cos_t * sin_a, cos_a, cos_a * d_m};
    current = Multiply(current, step);
    frames[j + 1] = current;
  }
  frames[kToolFrame] = Multiply(frames[7], model.flange_T_tool);
  return frames;
}

// 一个姿态下机械臂和夹爪离桌面、立杆表面的最小间距。输入关节角单位度。
inline Clearance MeasureClearance(
    const ArmModel& model, const Joints& joints_deg) {
  Clearance result;
  if (!model.HasObstacles()) {
    return result;
  }
  const auto frames = LinkFrames(model, joints_deg);
  for (const Capsule& capsule : model.capsules) {
    const auto start = Apply(frames[capsule.frame], capsule.start);
    const auto end = Apply(frames[capsule.frame], capsule.end);
    if (model.has_pole) {
      // 立杆是一条竖直线，只需算线段的水平投影到它的距离。
      const double dx = end[0] - start[0], dy = end[1] - start[1];
      const double ox = model.pole_x_m - start[0];
      const double oy = model.pole_y_m - start[1];
      const double length_squared = dx * dx + dy * dy;
      const double fraction = std::clamp(
          (ox * dx + oy * dy) / (length_squared > 0 ? length_squared : 1.),
          0., 1.);
      const double distance =
          std::hypot(ox - fraction * dx, oy - fraction * dy);
      result.pole_m = std::min(
          result.pole_m, distance - capsule.radius_m - model.pole_radius_m);
    }
    if (model.has_table && capsule.table_checked) {
      result.table_m = std::min(
          result.table_m,
          std::min(start[2], end[2]) - capsule.radius_m - model.table_z_m);
    }
    for (const ObstacleCapsule& other : model.other_arm) {
      // 包围盒不相交时，两者的间距一定大于“余量 + 2 毫米”，不必细算。
      bool apart = false;
      for (int k = 0; k < 3; ++k) {
        apart = apart ||
                std::min(start[k], end[k]) - capsule.radius_m > other.high[k] ||
                std::max(start[k], end[k]) + capsule.radius_m < other.low[k];
      }
      if (!apart) {
        result.other_arm_m = std::min(
            result.other_arm_m,
            SegmentDistance(start, end, other.start, other.end) -
                capsule.radius_m - other.radius_m);
      }
    }
  }
  return result;
}

// 一个姿态是否离桌面和立杆都足够远。输入关节角单位度。
inline bool ConfigurationClear(
    const ArmModel& model, const Joints& joints_deg) {
  if (!model.HasObstacles()) {
    return true;
  }
  const Clearance clearance = MeasureClearance(model, joints_deg);
  return clearance.pole_m >= model.pole_margin_m + kStricterM &&
         clearance.table_m >= -kTableToleranceM + kStricterM &&
         clearance.other_arm_m >= model.other_arm_margin_m + kStricterM;
}

// 从一个姿态按关节角直线走到另一个姿态，途中（不含两端）是否都足够远。
// 关节运动各关节同步转动，走的就是这条直线。输入关节角单位度。
inline bool MotionClear(
    const ArmModel& model, const Joints& from_deg, const Joints& to_deg) {
  if (!model.HasObstacles()) {
    return true;
  }
  const int steps = std::max(
      1, int(std::ceil(LargestJointStep(from_deg, to_deg) / kPathStepDeg)));
  for (int step = 1; step < steps; ++step) {
    Joints sample{};
    for (int j = 0; j < 7; ++j) {
      sample[j] = from_deg[j] + (to_deg[j] - from_deg[j]) * step / steps;
    }
    if (!ConfigurationClear(model, sample)) {
      return false;
    }
  }
  return true;
}

// 从输入流读取外形与障碍物。格式（数字以空白分隔，长度单位米）：
//   DH 行数 n，随后 n 行 (alpha 度, a, d)；
//   flange_T_tool 的前三行共 12 个数；
//   有无桌面(0/1) 桌面高度 有无立杆(0/1) 立杆 x 立杆 y 立杆半径 立杆余量；
//   胶囊体个数 m，随后 m 行
//     (坐标系序号 起点 xyz 终点 xyz 半径 是否参与桌面检查 0/1)；
//   与另一台臂之间的余量、它占着的胶囊体个数 k，随后 k 行
//     (起点 xyz 终点 xyz 半径)。
// 数值缺失、不是有限数或序号越界时返回 false。
inline bool ReadArmModel(std::istream& input, ArmModel* model) {
  auto finite = [&input](double* value) {
    return bool(input >> *value) && std::isfinite(*value);
  };
  int dh_count = 0;
  if (!(input >> dh_count) || (dh_count != 0 && dh_count != 7)) {
    return false;
  }
  model->dh.assign(dh_count, {});
  for (auto& row : model->dh) {
    for (double& value : row) {
      if (!finite(&value)) {
        return false;
      }
    }
  }
  for (double& value : model->flange_T_tool) {
    if (!finite(&value)) {
      return false;
    }
  }
  int has_table = 0, has_pole = 0, capsule_count = 0;
  if (!(input >> has_table) || !finite(&model->table_z_m) ||
      !(input >> has_pole) || !finite(&model->pole_x_m) ||
      !finite(&model->pole_y_m) || !finite(&model->pole_radius_m) ||
      !finite(&model->pole_margin_m) || !(input >> capsule_count) ||
      capsule_count < 0 || capsule_count > 64 || model->pole_radius_m < 0 ||
      model->pole_margin_m < 0) {
    return false;
  }
  model->has_table = has_table != 0;
  model->has_pole = has_pole != 0;
  model->capsules.assign(capsule_count, {});
  for (Capsule& capsule : model->capsules) {
    int table_checked = 0;
    if (!(input >> capsule.frame) || capsule.frame < 0 ||
        capsule.frame > kToolFrame) {
      return false;
    }
    for (double& value : capsule.start) {
      if (!finite(&value)) {
        return false;
      }
    }
    for (double& value : capsule.end) {
      if (!finite(&value)) {
        return false;
      }
    }
    if (!finite(&capsule.radius_m) || capsule.radius_m < 0 ||
        !(input >> table_checked)) {
      return false;
    }
    capsule.table_checked = table_checked != 0;
  }
  int other_count = 0;
  if (!finite(&model->other_arm_margin_m) || model->other_arm_margin_m < 0 ||
      !(input >> other_count) || other_count < 0 ||
      other_count > kMaxOtherArmCapsules) {
    return false;
  }
  model->other_arm.assign(other_count, {});
  for (ObstacleCapsule& other : model->other_arm) {
    for (double& value : other.start) {
      if (!finite(&value)) {
        return false;
      }
    }
    for (double& value : other.end) {
      if (!finite(&value)) {
        return false;
      }
    }
    if (!finite(&other.radius_m) || other.radius_m < 0) {
      return false;
    }
    const double reach =
        other.radius_m + model->other_arm_margin_m + kOtherArmBoxSlackM;
    for (int k = 0; k < 3; ++k) {
      other.low[k] = std::min(other.start[k], other.end[k]) - reach;
      other.high[k] = std::max(other.start[k], other.end[k]) + reach;
    }
  }
  // 有障碍物要查，却没有外形数据，属于调用方出错。
  return !model->HasObstacles() ||
         (dh_count == 7 && !model->capsules.empty());
}

}  // namespace saydo_ik

#endif  // SAYDO_ROBOT_IK_ARM_COLLISION_H_
