#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
cd $WORKSPACE_ROOT
make clean
make build_all
make run_sim_multi_robot
