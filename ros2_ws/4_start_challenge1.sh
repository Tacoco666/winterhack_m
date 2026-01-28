#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/lonewolf/Desktop/ros2_ws/install/setup.bash

echo "=========================================="
echo "  启动挑战1: 固定位置顺序抓取"
echo "=========================================="
echo ""
echo "挑战1说明："
echo "  - 按固定顺序去3个已知位置"
echo "  - 位置1 → 位置2 → 位置3"
echo "  - 在接近目标时检测到方块就立即抓取"
echo "  - 不需要精确到达目标点"
echo ""
echo "等待导航服务就绪..."
echo "检查 /navigate_to_pose 服务..."

# 等待 navigate_to_pose 服务可用（最多60秒）
timeout=60
elapsed=0
while [ $elapsed -lt $timeout ]; do
    if ros2 action list 2>/dev/null | grep -q "/navigate_to_pose"; then
        echo "✓ /navigate_to_pose 服务已就绪"
        break
    fi
    echo -n "."
    sleep 1
    elapsed=$((elapsed + 1))
done

if [ $elapsed -ge $timeout ]; then
    echo ""
    echo "错误: /navigate_to_pose 服务启动超时！"
    echo "请确保已启动 WinterHack 系统："
    echo "  ./2_start_winterhack.sh"
    exit 1
fi

echo ""
echo "启动任务执行器 (Challenge 1: 固定位置顺序抓取)..."
echo ""
ros2 run winterhack mission_challenge1
