#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/lonewolf/Desktop/ros2_ws/install/setup.bash

# 性能优化环境变量
export __GL_SYNC_TO_VBLANK=0  # 禁用垂直同步以提高帧率
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"  # 简化日志输出
export RCUTILS_COLORIZED_OUTPUT=0  # 禁用彩色输出以提高性能

echo "🚀 启动Gazebo（使用ogre2 GPU渲染引擎）..."

ros2 launch robot_gazebo worlds.launch.py world_name:=winterhack_maze
