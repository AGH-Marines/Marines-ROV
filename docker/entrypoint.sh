#!/bin/bash
set -e

# Load the base ROS2 environment
source /opt/ros/humble/setup.bash

# Load the workspace environment, if it exists
if [ -f "/home/dev/ros2_ws/install/setup.bash" ]; then
    source /home/dev/ros2_ws/install/setup.bash
fi

# Hand off execution to the CMD from the Dockerfile (i.e. "bash")
exec "$@"