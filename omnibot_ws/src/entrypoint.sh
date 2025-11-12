#!/bin/bash
set -e

# Source ROS and your workspace
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/root/omnibot_ws/install/setup.bash"

# Run given command (default: bash)
exec "$@"
