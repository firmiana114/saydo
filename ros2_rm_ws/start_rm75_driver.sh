#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
source install_jazzy/local_setup.bash
export ROS_DOMAIN_ID=75
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec ros2 launch rm_driver rm_75_driver.launch.py
