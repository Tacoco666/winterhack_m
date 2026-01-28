#!/bin/bash
source /opt/ros/humble/setup.bash
source "/home/lonewolf/Desktop/ros2_ws /install/setup.bash"

# 性能优化
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=0

ros2 launch winterhack winterhack.launch.py
