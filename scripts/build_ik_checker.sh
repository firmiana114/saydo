#!/usr/bin/env bash
# Build a versioned, offline-only verifier. Does not launch or command a robot.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
output="${1:-work/grasp/rm75_ik_check}"
mkdir -p "$(dirname -- "$output")"
temporary="${output}.build-$$"
trap 'rm -f -- "$temporary"' EXIT
sdk="$(pwd)/ros2_rm_ws/src/ros2_rm_robot/rm_driver"
# -I . 让源码按 Google 规范从项目根开始写头文件路径。
g++ -std=c++17 -O2 saydo/robot/ik/grasp_ik.cc \
  saydo/robot/ik/arm_angle_path.cc -I . \
  -I "$sdk/include/rm_driver" -L "$sdk/lib/linux_x86_c++_v1.1.5" \
  -Wl,-rpath,"$sdk/lib/linux_x86_c++_v1.1.5" -lapi_cpp -o "$temporary"
# The binary replacement is atomic. Existing processes keep their original
# executable.
mv -- "$temporary" "$output"
trap - EXIT
