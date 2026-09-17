// Offline SDK geometry checks only. No connection or motion API is called.
#include "rm_interface.h"
#include <cmath>
#include <iostream>

static bool read_pose(rm_pose_t& pose) {
  if (!(std::cin >> pose.position.x >> pose.position.y >> pose.position.z
                >> pose.euler.rx >> pose.euler.ry >> pose.euler.rz)) return false;
  pose.quaternion = rm_algo_euler2quaternion(pose.euler);
  return true;
}

int main() {
  rm_algo_init_sys_data(RM_MODEL_RM_75_E, RM_MODEL_RM_B_E);
  rm_frame_t tool{};
  if (!read_pose(tool.pose)) return 2;
  rm_algo_set_toolframe(&tool);
  float joints[ARM_DOF]{};
  for (int i = 0; i < 7; ++i) if (!(std::cin >> joints[i])) return 2;
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
