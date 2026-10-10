// 关节角的公共类型，逆解搜索和碰撞检查共用。
#ifndef SAYDO_ROBOT_IK_JOINTS_H_
#define SAYDO_ROBOT_IK_JOINTS_H_

#include <algorithm>
#include <array>
#include <cmath>

namespace saydo_ik {

// 七个关节角，单位度。
using Joints = std::array<float, 7>;

// 两组关节角之间变化最大的那个关节相差多少，单位度。
inline double LargestJointStep(const Joints& a, const Joints& b) {
  double value = 0;
  for (int j = 0; j < 7; ++j) {
    value = std::max(value, double(std::abs(a[j] - b[j])));
  }
  return value;
}

}  // namespace saydo_ik

#endif  // SAYDO_ROBOT_IK_JOINTS_H_
