#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/lonewolf/Desktop/ros2_ws/install/setup.bash

echo "=========================================="
echo "  启动挑战2: 颜色顺序巡逻救援"
echo "  RED → BLUE → YELLOW"
echo "=========================================="
echo ""
echo "等待导航服务就绪..."
echo "检查 /follow_waypoints 服务..."

# 等待 follow_waypoints 服务可用（最多60秒）
timeout=60
elapsed=0
while [ $elapsed -lt $timeout ]; do
    if ros2 action list 2>/dev/null | grep -q "/follow_waypoints"; then
        echo "✓ /follow_waypoints 服务已就绪"
        break
    fi
    echo -n "."
    sleep 1
    elapsed=$((elapsed + 1))
done

if [ $elapsed -ge $timeout ]; then
    echo ""
    echo "错误: /follow_waypoints 服务启动超时！"
    echo "请确保已启动 WinterHack 系统："
    echo "  ./2_start_winterhack.sh"
    exit 1
fi

echo ""
echo "检查 /navigate_to_pose 服务..."
if ros2 action list 2>/dev/null | grep -q "/navigate_to_pose"; then
    echo "✓ /navigate_to_pose 服务已就绪"
else
    echo "警告: /navigate_to_pose 服务未就绪"
fi

echo ""
echo "=========================================="
echo "启动任务执行器 (Challenge 2)..."
echo "=========================================="
echo ""
ros2 run winterhack mission_challenge2
