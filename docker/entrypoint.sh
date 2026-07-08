#!/bin/bash
set -e

# Załadowanie bazowego środowiska ROS2
source /opt/ros/humble/setup.bash

# Załadowanie środowiska przestrzeni roboczej, jeśli istnieje
if [ -f "/home/dev/ros2_ws/install/setup.bash" ]; then
    source /home/dev/ros2_ws/install/setup.bash
fi

# Przekazanie wykonania do CMD z Dockerfile (czyli "bash")
exec "$@"