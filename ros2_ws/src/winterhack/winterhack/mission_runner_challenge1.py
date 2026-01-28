#!/usr/bin/env python3
"""
Challenge 1: Known-coordinate Rescue (Efficiency-Driven)
直接导航到已知坐标，无需搜索
"""

import math
from enum import Enum
from typing import List, Tuple, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from winterhack_interfaces.action import Drop, Locate, Pick
from std_msgs.msg import String
import json


class MissionPhase(Enum):
    INIT = "INIT"
    NAVIGATE_TO_REGION = "NAVIGATE_TO_REGION"  # 导航到目标区域
    LOCATE = "LOCATE"  # 在区域内定位方块
    PICK = "PICK"
    NAVIGATE_HOME = "NAVIGATE_HOME"
    DROP = "DROP"
    IDLE = "IDLE"


def yaw_to_quat(yaw: float) -> Quaternion:
    half = yaw * 0.5
    return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))


def make_pose_stamped(x: float, y: float, yaw: float, frame_id: str, stamp=None) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp if stamp is not None else rclpy.time.Time().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    pose.pose.orientation = yaw_to_quat(yaw)
    return pose


def calculate_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """计算两点之间的欧几里得距离"""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def optimize_path(start: Tuple[float, float], targets: List[Tuple[float, float, str]]) -> List[Tuple[float, float, str]]:
    """
    简单的贪心算法优化路径（最近邻）
    targets: [(x, y, color), ...]
    返回优化后的顺序
    """
    if not targets:
        return []
    
    remaining = targets.copy()
    ordered = []
    current_pos = start
    
    while remaining:
        # 找到距离当前位置最近的目标
        nearest = min(remaining, key=lambda t: calculate_distance(current_pos, (t[0], t[1])))
        ordered.append(nearest)
        remaining.remove(nearest)
        current_pos = (nearest[0], nearest[1])
    
    return ordered


class MissionRunnerChallenge1(Node):
    """Challenge 1: 已知坐标高效救援"""

    def __init__(self):
        super().__init__("mission_runner_challenge1")

        self._cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()

        # 参数配置
        self.declare_parameter("home_xy", [0.0, 0.0])
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("detection_topic", "/color_detection/detection_info")
        self.declare_parameter("enable_locate", True)  # 是否使用视觉定位
        self.declare_parameter("enable_pick", True)
        self.declare_parameter("enable_drop", True)
        self.declare_parameter("early_detection", True)  # 导航时提前检测
        
        # 已知目标坐标 (x, y, color) - 来自 winterhack_maze.sdf
        # 这些是方块的大概位置，机器人会导航到附近然后用视觉搜索
        self.declare_parameter("target_red", [1.4, -1.4])      # RED 方块区域
        self.declare_parameter("target_blue", [-1.4, 1.4])     # BLUE 方块区域
        self.declare_parameter("target_yellow", [1.4, 1.4])    # YELLOW 方块区域

        self._home_xy = tuple(self.get_parameter("home_xy").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._detection_topic = str(self.get_parameter("detection_topic").value)
        self._enable_locate = bool(self.get_parameter("enable_locate").value)
        self._enable_pick = bool(self.get_parameter("enable_pick").value)
        self._enable_drop = bool(self.get_parameter("enable_drop").value)
        self._early_detection = bool(self.get_parameter("early_detection").value)

        # 构建目标列表 - 3个方块
        red_xy = tuple(self.get_parameter("target_red").value)
        blue_xy = tuple(self.get_parameter("target_blue").value)
        yellow_xy = tuple(self.get_parameter("target_yellow").value)

        # 优化路径顺序（从home出发）
        targets = [
            (red_xy[0], red_xy[1], "RED"),
            (blue_xy[0], blue_xy[1], "BLUE"),
            (yellow_xy[0], yellow_xy[1], "YELLOW")
        ]
        self._targets = optimize_path(self._home_xy, targets)
        self._current_target_idx = 0
        self._total_targets = 3  # 总共3个方块

        self.get_logger().info(f"优化后的救援顺序: {[t[2] for t in self._targets]}")

        # 状态
        self._state = MissionPhase.INIT
        self._picked_count = 0
        self._target_detected = False  # 当前目标是否被检测到

        # Action Clients
        self._nav_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose", callback_group=self._cb_group
        )
        self._locate_client = ActionClient(
            self, Locate, "/locate", callback_group=self._cb_group
        )
        self._pick_client = ActionClient(
            self, Pick, "/pick", callback_group=self._cb_group
        )
        self._drop_client = ActionClient(
            self, Drop, "/drop", callback_group=self._cb_group
        )

        # Goal handles
        self._nav_goal_future = None
        self._nav_goal_handle = None
        self._nav_result_future = None

        self._locate_goal_future = None
        self._locate_goal_handle = None
        self._locate_result_future = None

        self._pick_goal_future = None
        self._pick_goal_handle = None
        self._pick_result_future = None

        self._drop_goal_future = None
        self._drop_goal_handle = None
        self._drop_result_future = None

        # 订阅检测话题（导航时提前发现方块）
        if self._early_detection:
            self._detection_sub = self.create_subscription(
                String,
                self._detection_topic,
                self._on_detection,
                10,
                callback_group=self._cb_group
            )

        # 定时器
        self._timer = self.create_timer(0.1, self._tick, callback_group=self._timer_cb_group)

    def _set_state(self, new_state: MissionPhase):
        if self._state != new_state:
            self.get_logger().info(f"状态切换: {self._state.value} -> {new_state.value}")
            self._state = new_state

    def _on_detection(self, msg: String):
        """检测回调：在导航时如果看到目标方块就标记"""
        if self._state != MissionPhase.NAVIGATE_TO_REGION:
            return
        
        detected_color = self._extract_colour(msg)
        if detected_color is None:
            return
        
        # 检查是否是当前目标的颜色
        if self._current_target_idx < len(self._targets):
            target_color = self._targets[self._current_target_idx][2]
            if detected_color == target_color:
                if not self._target_detected:
                    self._target_detected = True
                    self.get_logger().info(f"👁️ 导航中检测到目标 {target_color}！")

    def _extract_colour(self, msg: String) -> Optional[str]:
        """从消息中提取颜色"""
        data = (msg.data or "").strip()
        if not data:
            return None
        try:
            payload = json.loads(data)
            colour = payload.get("colour") or payload.get("color")
            if colour:
                return str(colour).strip().upper()
        except json.JSONDecodeError:
            pass
        return None

    def _tick(self):
        if self._state == MissionPhase.INIT:
            self._tick_init()
        elif self._state == MissionPhase.NAVIGATE_TO_REGION:
            self._tick_navigate_to_region()
        elif self._state == MissionPhase.LOCATE:
            self._tick_locate()
        elif self._state == MissionPhase.PICK:
            self._tick_pick()
        elif self._state == MissionPhase.NAVIGATE_HOME:
            self._tick_navigate_home()
        elif self._state == MissionPhase.DROP:
            self._tick_drop()
        elif self._state == MissionPhase.IDLE:
            pass

    def _tick_init(self):
        if self._current_target_idx == 0:
            self.get_logger().info("任务开始！")
        if self._current_target_idx < len(self._targets):
            self._set_state(MissionPhase.NAVIGATE_TO_REGION)
        else:
            self.get_logger().info("🎉 所有3个目标已完成！")
            self._set_state(MissionPhase.IDLE)

    def _tick_navigate_to_region(self):
        """导航到目标区域（方块附近）"""
        # 如果在导航过程中检测到目标，立即取消导航
        if self._target_detected and self._nav_goal_handle is not None:
            target_name = self._targets[self._current_target_idx][2]
            self.get_logger().info(f"✓ 已看到 {target_name}，取消导航，直接定位！")
            self._nav_goal_handle.cancel_goal_async()
            self._clear_nav_goal()
            self._target_detected = False
            if self._enable_locate:
                self._set_state(MissionPhase.LOCATE)
            elif self._enable_pick:
                self._set_state(MissionPhase.PICK)
            else:
                self._current_target_idx += 1
                self._set_state(MissionPhase.INIT)
            return

        self._update_nav_goal()
        if self._nav_goal_handle is None and self._nav_goal_future is None:
            target = self._targets[self._current_target_idx]
            self.get_logger().info(f"🚗 导航到 {target[2]} 区域 ({target[0]}, {target[1]})")
            self._target_detected = False  # 重置检测标志
            self._send_nav_goal(target[0], target[1])

        if self._nav_result_future is not None and self._nav_result_future.done():
            result = self._nav_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            self._clear_nav_goal()
            if success:
                target_name = self._targets[self._current_target_idx][2]
                self.get_logger().info(f"✓ 到达 {target_name} 区域，开始视觉搜索...")
                if self._enable_locate:
                    self._set_state(MissionPhase.LOCATE)
                elif self._enable_pick:
                    self._set_state(MissionPhase.PICK)
                else:
                    self.get_logger().warning("Locate和Pick都被禁用！跳过")
                    self._current_target_idx += 1
                    self._set_state(MissionPhase.INIT)
            else:
                self.get_logger().error("❌ 导航失败！跳过此目标")
                self._current_target_idx += 1
                self._set_state(MissionPhase.INIT)

    def _tick_locate(self):
        """使用视觉定位方块精确位置"""
        self._update_locate_goal()
        if self._locate_goal_handle is None and self._locate_goal_future is None:
            target_name = self._targets[self._current_target_idx][2]
            self.get_logger().info(f"👁️ 定位 {target_name} 方块...")
            self._send_locate_goal()

        if self._locate_result_future is not None and self._locate_result_future.done():
            result = self._locate_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            self._clear_locate_goal()
            if success:
                target_name = self._targets[self._current_target_idx][2]
                self.get_logger().info(f"✓ {target_name} 定位成功！")
                if self._enable_pick:
                    self._set_state(MissionPhase.PICK)
                else:
                    self.get_logger().warning("Pick被禁用，跳过抓取")
                    self._current_target_idx += 1
                    self._set_state(MissionPhase.INIT)
            else:
                target_name = self._targets[self._current_target_idx][2]
                self.get_logger().error(f"❌ {target_name} 定位失败！跳过此目标")
                self._current_target_idx += 1
                self._set_state(MissionPhase.INIT)

    def _tick_pick(self):
        """抓取目标"""
        self._update_pick_goal()
        if self._pick_goal_handle is None and self._pick_goal_future is None:
            self.get_logger().info("开始抓取...")
            self._send_pick_goal()

        if self._pick_result_future is not None and self._pick_result_future.done():
            result = self._pick_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            self._clear_pick_goal()
            if success:
                self._picked_count += 1
                target_name = self._targets[self._current_target_idx][2]
                self.get_logger().info(f"✓ {target_name} 抓取成功！进度: {self._picked_count}/{self._total_targets}")
                self._set_state(MissionPhase.NAVIGATE_HOME)
            else:
                self.get_logger().error("抓取失败！跳过此目标")
                self._current_target_idx += 1
                self._set_state(MissionPhase.INIT)

    def _tick_navigate_home(self):
        """导航回家"""
        self._update_nav_goal()
        if self._nav_goal_handle is None and self._nav_goal_future is None:
            self.get_logger().info(f"返回Home ({self._home_xy[0]}, {self._home_xy[1]})")
            self._send_nav_goal(self._home_xy[0], self._home_xy[1])

        if self._nav_result_future is not None and self._nav_result_future.done():
            result = self._nav_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            self._clear_nav_goal()
            if success:
                self.get_logger().info("到达Home！")
                self._set_state(MissionPhase.DROP)
            else:
                self.get_logger().error("返回Home失败！")
                self._set_state(MissionPhase.DROP)

    def _tick_drop(self):
        """放置目标"""
        self._update_drop_goal()
        if self._drop_goal_handle is None and self._drop_goal_future is None:
            self.get_logger().info("开始放置...")
            self._send_drop_goal()

        if self._drop_result_future is not None and self._drop_result_future.done():
            result = self._drop_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            self._clear_drop_goal()
            if success:
                self.get_logger().info("放置成功！")
            else:
                self.get_logger().warning("放置失败！")
            
            # 继续下一个目标
            self._current_target_idx += 1
            self._set_state(MissionPhase.INIT)

    # === Action Client 辅助函数 ===
    def _send_nav_goal(self, x: float, y: float):
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("导航服务未就绪")
            return
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(x, y, 0.0, self._map_frame, self.get_clock().now().to_msg())
        self._nav_goal_future = self._nav_client.send_goal_async(goal)

    def _send_locate_goal(self):
        if not self._locate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("定位服务未就绪")
            return
        goal = Locate.Goal()
        self._locate_goal_future = self._locate_client.send_goal_async(goal)

    def _send_pick_goal(self):
        if not self._pick_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("抓取服务未就绪")
            return
        goal = Pick.Goal()
        self._pick_goal_future = self._pick_client.send_goal_async(goal)

    def _send_drop_goal(self):
        if not self._drop_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warning("放置服务未就绪")
            return
        goal = Drop.Goal()
        self._drop_goal_future = self._drop_client.send_goal_async(goal)

    def _update_nav_goal(self):
        if self._nav_goal_future is not None and self._nav_goal_future.done():
            self._nav_goal_handle = self._nav_goal_future.result()
            self._nav_goal_future = None
            if self._nav_goal_handle.accepted:
                self._nav_result_future = self._nav_goal_handle.get_result_async()

    def _update_locate_goal(self):
        if self._locate_goal_future is not None and self._locate_goal_future.done():
            self._locate_goal_handle = self._locate_goal_future.result()
            self._locate_goal_future = None
            if self._locate_goal_handle.accepted:
                self._locate_result_future = self._locate_goal_handle.get_result_async()

    def _update_pick_goal(self):
        if self._pick_goal_future is not None and self._pick_goal_future.done():
            self._pick_goal_handle = self._pick_goal_future.result()
            self._pick_goal_future = None
            if self._pick_goal_handle.accepted:
                self._pick_result_future = self._pick_goal_handle.get_result_async()

    def _update_drop_goal(self):
        if self._drop_goal_future is not None and self._drop_goal_future.done():
            self._drop_goal_handle = self._drop_goal_future.result()
            self._drop_goal_future = None
            if self._drop_goal_handle.accepted:
                self._drop_result_future = self._drop_goal_handle.get_result_async()

    def _clear_nav_goal(self):
        self._nav_goal_future = None
        self._nav_goal_handle = None
        self._nav_result_future = None

    def _clear_locate_goal(self):
        self._locate_goal_future = None
        self._locate_goal_handle = None
        self._locate_result_future = None

    def _clear_pick_goal(self):
        self._pick_goal_future = None
        self._pick_goal_handle = None
        self._pick_result_future = None

    def _clear_drop_goal(self):
        self._drop_goal_future = None
        self._drop_goal_handle = None
        self._drop_result_future = None


def main(args=None):
    rclpy.init(args=args)
    node = MissionRunnerChallenge1()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
