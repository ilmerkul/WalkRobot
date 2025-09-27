#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
cd $WORKSPACE_ROOT
rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO
exec "$@"
