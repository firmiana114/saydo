// Offline SDK geometry checks only. No connection or motion API is called.
#include "rm_interface.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <string>

static bool read_pose(rm_pose_t& pose) {
  if (!(std::cin >> pose.position.x >> pose.position.y >> pose.position.z
                >> pose.euler.rx >> pose.euler.ry >> pose.euler.rz)) return false;
  pose.quaternion = rm_algo_euler2quaternion(pose.euler);
  return true;
}

static int joint_path(float* start) {
  float target[7], low[3], high[3], table;
  for (float& v : target) if (!(std::cin >> v) || !std::isfinite(v)) return 2;
  for (float& v : low) if (!(std::cin >> v) || !std::isfinite(v)) return 2;
  for (float& v : high) if (!(std::cin >> v) || !std::isfinite(v)) return 2;
  if (!(std::cin >> table) || !std::isfinite(table)) return 2;
  int steps = 1;
  for (int j = 0; j < 7; ++j) steps = std::max(steps, (int)std::ceil(std::abs(target[j]-start[j])/2.0));
  if (steps > 500) return 2;
  for (int n = 0; n <= steps; ++n) {
    float joints[ARM_DOF]{};
    for (int j = 0; j < 7; ++j) joints[j] = start[j] + (target[j]-start[j])*n/steps;
    int collision = rm_algo_safety_robot_self_collision_detection(joints);
    auto pose = rm_algo_forward_kinematics(nullptr, joints);
    float xyz[] = {pose.position.x, pose.position.y, pose.position.z};
    bool valid = collision == 0 && xyz[2] >= table + .01f;
    for (int k = 0; k < 3; ++k) valid = valid && std::isfinite(xyz[k]) && xyz[k] >= low[k] && xyz[k] <= high[k];
    if (!valid) { std::cout << "RETURN_FAIL " << n << ' ' << collision << ' ' << xyz[0] << ' ' << xyz[1] << ' ' << xyz[2] << '\n'; return 1; }
  }
  std::cout << "RETURN_OK " << steps+1 << '\n';
  return 0;
}

int main(int argc, char** argv) {
  rm_algo_init_sys_data(RM_MODEL_RM_75_E, RM_MODEL_RM_B_E);
  rm_frame_t tool{};
  if (!read_pose(tool.pose)) return 2;
  rm_algo_set_toolframe(&tool);
  float joints[ARM_DOF]{};
  for (int i = 0; i < 7; ++i) if (!(std::cin >> joints[i])) return 2;
  if (argc == 2 && std::string(argv[1]) == "--joint-path") return joint_path(joints);
  auto fk = rm_algo_forward_kinematics(nullptr, joints);
  std::cout.precision(9);
  std::cout << "FK " << fk.position.x << ' ' << fk.position.y << ' ' << fk.position.z
            << ' ' << fk.euler.rx << ' ' << fk.euler.ry << ' ' << fk.euler.rz << '\n';
  int count = 0;
  if (!(std::cin >> count) || count < 0 || count > 500) return 2;
  for (int i = 0; i < count; ++i) {
    rm_inverse_kinematics_params_t input{};
    for (int j = 0; j < 7; ++j) input.q_in[j] = joints[j];
    input.flag = 1;
    if (!read_pose(input.q_pose)) return 2;
    float solution[ARM_DOF]{};
    int code = rm_algo_inverse_kinematics(nullptr, input, solution);
    int collision = code == 0 ? rm_algo_safety_robot_self_collision_detection(solution) : -1;
    std::cout << "IK " << code << ' ' << collision;
    for (int j = 0; j < 7; ++j) std::cout << ' ' << solution[j];
    std::cout << '\n';
    if (code != 0 || collision != 0) return 1;
    for (int j = 0; j < 7; ++j) joints[j] = solution[j];
  }
  return 0;
}
