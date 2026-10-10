// Offline SDK geometry checks only. No connection or motion API is called.
#include "rm_interface.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <array>
#include <limits>
#include <chrono>
#include "saydo/robot/ik/arm_angle_path.h"

static bool read_pose(rm_pose_t& pose) {
  if (!(std::cin >> pose.position.x >> pose.position.y >> pose.position.z
                 >> pose.euler.rx >> pose.euler.ry >> pose.euler.rz)) {
    return false;
  }
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z) || !std::isfinite(pose.euler.rx) ||
      !std::isfinite(pose.euler.ry) || !std::isfinite(pose.euler.rz)) {
    return false;
  }
  pose.quaternion = rm_algo_euler2quaternion(pose.euler);
  return true;
}

static double orientation_error(const rm_pose_t& a, const rm_pose_t& b) {
  auto qa = rm_algo_euler2quaternion(a.euler);
  auto qb = rm_algo_euler2quaternion(b.euler);
  double dot = (double)qa.w * qb.w + (double)qa.x * qb.x +
               (double)qa.y * qb.y + (double)qa.z * qb.z;
  double na = std::sqrt((double)qa.w * qa.w + (double)qa.x * qa.x +
                        (double)qa.y * qa.y + (double)qa.z * qa.z);
  double nb = std::sqrt((double)qb.w * qb.w + (double)qb.x * qb.x +
                        (double)qb.y * qb.y + (double)qb.z * qb.z);
  if (!(na > 0 && nb > 0)) {
    return NAN;
  }
  return 2 * std::acos(std::min(1.0, std::abs(dot / (na * nb))));
}

static int joint_path(float* start) {
  float target[7], low[3], high[3], table, clearance;
  for (float& v : target) {
    if (!(std::cin >> v) || !std::isfinite(v)) {
      return 2;
    }
  }
  for (float& v : low) {
    if (!(std::cin >> v) || !std::isfinite(v)) {
      return 2;
    }
  }
  for (float& v : high) {
    if (!(std::cin >> v) || !std::isfinite(v)) {
      return 2;
    }
  }
  if (!(std::cin >> table >> clearance) || !std::isfinite(table) ||
      !std::isfinite(clearance) || clearance < 0) {
    return 2;
  }
  int steps = 1;
  for (int j = 0; j < 7; ++j) {
    steps = std::max(
        steps, (int)std::ceil(std::abs(target[j] - start[j]) / 2.0));
  }
  if (steps > 500) {
    return 2;
  }
  for (int n = 0; n <= steps; ++n) {
    float joints[ARM_DOF]{};
    for (int j = 0; j < 7; ++j) {
      joints[j] = start[j] + (target[j] - start[j]) * n / steps;
    }
    int collision = rm_algo_safety_robot_self_collision_detection(joints);
    auto pose = rm_algo_forward_kinematics(nullptr, joints);
    float xyz[] = {pose.position.x, pose.position.y, pose.position.z};
    bool valid = collision == 0 && xyz[2] >= table + clearance;
    for (int k = 0; k < 3; ++k) {
      valid = valid && std::isfinite(xyz[k]) && xyz[k] >= low[k] &&
              xyz[k] <= high[k];
    }
    if (!valid) {
      std::cout << "RETURN_FAIL " << n << ' ' << collision << ' ' << xyz[0]
                << ' ' << xyz[1] << ' ' << xyz[2] << '\n';
      return 1;
    }
  }
  std::cout << "RETURN_OK " << steps + 1 << '\n';
  return 0;
}

int main(int argc, char** argv) {
  rm_algo_init_sys_data(RM_MODEL_RM_75_E, RM_MODEL_RM_B_E);
  rm_frame_t tool{};
  if (!read_pose(tool.pose)) {
    return 2;
  }
  rm_algo_set_toolframe(&tool);
  float joints[ARM_DOF]{};
  for (int i = 0; i < 7; ++i) {
    if (!(std::cin >> joints[i]) || !std::isfinite(joints[i])) {
      return 2;
    }
  }
  if (argc == 2 && std::string(argv[1]) == "--joint-path-v2") {
    return joint_path(joints);
  }
  // 按臂角枚举逆解并搜索连续、不碰撞的关节角路径。
  const bool arm_angle =
      argc == 2 && std::string(argv[1]) == "--arm-angle-v2";
  const bool multi = argc == 2 && std::string(argv[1]) == "--multi-seed-v1";
  auto fk = rm_algo_forward_kinematics(nullptr, joints);
  std::cout.precision(12);
  std::cout << "PROTOCOL IK_FK_V1\n";
  if (multi) {
    std::cout << "CAPABILITY MULTI_SEED_V1\n";
  }
  if (arm_angle) {
    std::cout << "CAPABILITY ARM_ANGLE_V2\n";
  }
  std::cout << "FK " << fk.position.x << ' ' << fk.position.y << ' '
            << fk.position.z << ' ' << fk.euler.rx << ' ' << fk.euler.ry
            << ' ' << fk.euler.rz << '\n';
  if (arm_angle) {
    return saydo_ik::RunArmAnglePath(joints);
  }
  int count = 0;
  if (!(std::cin >> count) || count < 0 || count > 500) {
    return 2;
  }
  for (int i = 0; i < count; ++i) {
    rm_inverse_kinematics_params_t input{};
    for (int j = 0; j < 7; ++j) {
      input.q_in[j] = joints[j];
    }
    input.flag = 1;
    if (!read_pose(input.q_pose)) {
      return 2;
    }
    float solution[ARM_DOF]{};
    float step_limit = 360.f;
    if (multi && (!(std::cin >> step_limit) || !std::isfinite(step_limit) ||
                  step_limit <= 0)) {
      return 2;
    }
    int code = 1, collision = -1;
    int tries = 0, valid_count = 0;
    double best = std::numeric_limits<double>::infinity();
    float low[ARM_DOF]{}, high[ARM_DOF]{};
    rm_algo_get_joint_min_limit(low);
    rm_algo_get_joint_max_limit(high);
    std::vector<std::array<float, 7>> offsets(1);
    if (multi) {
      for (float delta : {10.f, -10.f, 25.f, -25.f, 45.f, -45.f}) {
        for (int joint : {1, 2, 3, 5}) {
          std::array<float, 7> offset{};
          offset[joint] = delta;
          offsets.push_back(offset);
        }
        std::array<float, 7> offset{};
        offset[1] = delta;
        offset[3] = -delta;
        offsets.push_back(offset);
      }
    }
    for (const auto& offset : offsets) {
      for (int j = 0; j < 7; ++j) {
        input.q_in[j] = std::clamp(joints[j] + offset[j], low[j], high[j]);
      }
      float candidate[ARM_DOF]{};
      ++tries;
      int rc = rm_algo_inverse_kinematics(nullptr, input, candidate);
      int hit = rc == 0 ?
          rm_algo_safety_robot_self_collision_detection(candidate) : -1;
      // Keep the first result for the existing detailed failure protocol.
      if (tries == 1) {
        code = rc;
        collision = hit;
        std::copy(candidate, candidate + 7, solution);
      }
      if (rc != 0 || hit != 0) {
        continue;
      }
      bool finite = true;
      double jump = 0, travel = 0;
      for (int j = 0; j < 7; ++j) {
        finite = finite && std::isfinite(candidate[j]) &&
                 candidate[j] >= low[j] && candidate[j] <= high[j];
        double delta = std::abs(candidate[j] - joints[j]);
        jump = std::max(jump, delta);
        travel += delta;
      }
      if (!finite) {
        continue;
      }
      auto check = rm_algo_forward_kinematics(nullptr, candidate);
      double dx = check.position.x - input.q_pose.position.x,
             dy = check.position.y - input.q_pose.position.y,
             dz = check.position.z - input.q_pose.position.z;
      double distance = std::sqrt(dx * dx + dy * dy + dz * dz),
             angle = orientation_error(check, input.q_pose);
      if (!std::isfinite(distance) || !std::isfinite(angle) ||
          distance > .002 || angle > 2 * std::acos(-1.0) / 180) {
        continue;
      }
      if (jump > step_limit) {
        continue;
      }
      ++valid_count;
      if (travel < best) {
        best = travel;
        code = rc;
        collision = hit;
        std::copy(candidate, candidate + 7, solution);
      }
      // Preserve an already-valid warm start; expand only when it fails.
      if (tries == 1) {
        break;
      }
    }
    for (int j = 0; j < 7; ++j) {
      if (!std::isfinite(solution[j]) || solution[j] < low[j] ||
          solution[j] > high[j]) {
        code = -2;
      }
    }
    if (multi) {
      std::cout << "SEEDS " << i << ' ' << tries << ' ' << valid_count
                << '\n';
    }
    std::cout << "IK " << code << ' ' << collision;
    for (int j = 0; j < 7; ++j) {
      std::cout << ' ' << solution[j];
    }
    std::cout << '\n';
    if (code != 0 || collision != 0) {
      return 1;
    }
    for (int j = 0; j < 7; ++j) {
      if (!std::isfinite(solution[j])) {
        return 3;
      }
    }
    auto achieved = rm_algo_forward_kinematics(nullptr, solution);
    double dx = achieved.position.x - input.q_pose.position.x;
    double dy = achieved.position.y - input.q_pose.position.y;
    double dz = achieved.position.z - input.q_pose.position.z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    double angle = orientation_error(achieved, input.q_pose);
    std::cout << "VERIFY " << i << ' ' << achieved.position.x << ' '
              << achieved.position.y << ' ' << achieved.position.z << ' '
              << achieved.euler.rx << ' ' << achieved.euler.ry << ' '
              << achieved.euler.rz << '\n';
    std::cout << "RESIDUAL " << i << ' ' << distance << ' ' << angle << '\n';
    // An IK success flag alone is insufficient near degenerate or
    // unreachable targets.
    if (!std::isfinite(distance) || !std::isfinite(angle) ||
        distance > 0.002 || angle > 2 * std::acos(-1.0) / 180) {
      return 3;
    }
    for (int j = 0; j < 7; ++j) {
      joints[j] = solution[j];
    }
  }
  return 0;
}
