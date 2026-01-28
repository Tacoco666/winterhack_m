#!/bin/bash

echo "=========================================="
echo "  清理所有 ROS2 和 Gazebo 进程"
echo "=========================================="

echo "停止 Gazebo..."
pkill -9 -f "gz sim"
pkill -9 -f "ign gazebo"
pkill -9 -f "ruby"

echo "停止 ROS2 launch..."
pkill -9 -f "ros2 launch"

echo "停止 ROS2 节点..."
pkill -9 -f "mission_runner"
pkill -9 -f "mission_challenge1"
pkill -9 -f "mission_challenge2"
pkill -9 -f "slam_toolbox"
pkill -9 -f "bt_navigator"
pkill -9 -f "controller_server"
pkill -9 -f "planner_server"

echo "停止桥接..."
pkill -9 -f "parameter_bridge"
pkill -9 -f "ros_gz_bridge"
pkill -9 -f "ros_ign_bridge"

echo "停止控制器..."
pkill -9 -f "spawner"
pkill -9 -f "controller_manager"

sleep 2

echo ""
echo "=========================================="
echo "  清理完成！"
echo "=========================================="
echo ""
echo "现在可以重新启动："
echo "  终端1: ./1_start_gazebo.sh"
echo "  终端2: ./2_start_winterhack.sh (等15秒)"
echo "  终端3: ./3_start_challenge2.sh (等30秒) - 颜色顺序救援"
echo "  或者:   ./4_start_challenge1.sh (等30秒) - 固定位置抓取"
echo ""
