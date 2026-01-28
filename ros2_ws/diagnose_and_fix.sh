#!/bin/bash

echo "=========================================="
echo "  WinterHack 导航系统诊断"
echo "=========================================="

source /opt/ros/humble/setup.bash
source /home/lonewolf/Desktop/ros2_ws/install/setup.bash

echo ""
echo "1. 检查关键节点..."
ros2 node list | grep -E "bt_navigator|controller_server|planner_server|slam_toolbox"

echo ""
echo "2. 检查 Action 服务..."
ros2 action list | grep -E "navigate|follow"

echo ""
echo "3. 检查话题..."
ros2 topic list | grep -E "cmd_vel|odom|scan|map"

echo ""
echo "4. 检查 TF 树..."
timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -n 5

echo ""
echo "5. 检查控制器..."
ros2 control list_controllers

echo ""
echo "6. 测试手动导航（5秒）..."
timeout 5 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" 2>&1 | tail -n 3

echo ""
echo "=========================================="
echo "  诊断完成"
echo "=========================================="
echo ""
echo "如果看到 'ABORTED' 或 'rejected'，请检查："
echo "  1. Nav2 参数配置是否正确"
echo "  2. Costmap 是否正常更新"
echo "  3. 机器人是否被困在障碍物中"
echo ""
