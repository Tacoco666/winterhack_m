#!/usr/bin/env python3
"""
Challenge 2: Priority-Order Rescue (Decision-Driven)
严格按照 RED → BLUE → YELLOW 顺序救援
"""

import json
import math
from enum import Enum
from typing import Optional, Set, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String

from winterhack_interfaces.action import Drop, Locate, Pick


# 严格的救援顺序
RESCUE_ORDER = ["RED", "BLUE", "YELLOW"]

DEFAULT_PATROL_TARGETS = [(-1.4, -1.4), (1.4, 1.4), (-1.4, 1.4), (1.4, -1.4)]
DEFAULT_HOME = (0.0, 0.0)


class MissionPhase(Enum):
    INIT = "INIT"
    PATROL_SEARCH = "PATROL_SEARCH"
    STOP_SEARCH = "STOP_SEARCH"
    LOCATE = "LOCATE"
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


class MissionRunnerChallenge2(Node):
    """Challenge 2: 优先级顺序救援"""

    def __init__(self):
        super().__init__("mission_runner_challenge2")

        self._cb_group = ReentrantCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()

        # 参数配置
        self.declare_parameter("home_xy", [0.0, 0.0])
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("detection_topic", "/color_detection/detection_info")
        self.declare_parameter("enable_locate", True)
        self.declare_parameter("enable_pick", True)
        self.declare_parameter("enable_drop", True)

        self._home_xy = tuple(self.get_parameter("home_xy").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._detection_topic = str(self.get_parameter("detection_topic").value)
        self._enable_locate = bool(self.get_parameter("enable_locate").value)
        self._enable_pick = bool(self.get_parameter("enable_pick").value)
        self._enable_drop = bool(self.get_parameter("enable_drop").value)

        # 巡逻目标
        self._patrol_targets = list(DEFAULT_PATROL_TARGETS)

        # 任务状态
        self._state = MissionPhase.INIT
        self._current_priority_idx = 0  # 当前需要的颜色索引 (0=RED, 1=BLUE, 2=YELLOW)
        self._detected_colour: Optional[str] = None
        self._rescued_count = 0

        # Action Clients
        self._follow_client = ActionClient(
            self, FollowWaypoints, "/follow_waypoints", callback_group=self._cb_group
        )
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
        self._follow_goal_future = None
        self._follow_goal_handle = None
        self._follow_result_future = None
        self._follow_cancel_future = None

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

        # 订阅检测话题
        self._detection_sub = self.create_subscription(
            String, self._detection_topic, self._on_detection, 10, callback_group=self._cb_group
        )

        # 定时器
        self._timer = self.create_timer(0.1, self._tick, callback_group=self._timer_cb_group)

        self.get_logger().info(f"任务开始！救援顺序: {' → '.join(RESCUE_ORDER)}")

    def _get_current_target_colour(self) -> Optional[str]:
        """获取当前需要救援的颜色"""
        if self._current_priority_idx < len(RESCUE_ORDER):
            return RESCUE_ORDER[self._current_priority_idx]
        return None

    def _set_state(self, new_state: MissionPhase):
        if self._state != new_state:
            self.get_logger().info(f"状态切换: {self._state.value} -> {new_state.value}")
            self._state = new_state

    def _on_detection(self, msg: String):
        """检测回调：只接受当前需要的颜色"""
        if self._state != MissionPhase.PATROL_SEARCH:
            return

        detected = self._extract_colour(msg)
        if detected is None:
            return

        target_colour = self._get_current_target_colour()
        if target_colour is None:
            return

        # 关键：只接受当前优先级的颜色
        if detected == target_colour:
            self._detected_colour = detected
            self.get_logger().info(f"✓ 检测到目标颜色: {detected} (当前需要: {target_colour})")
            self._set_state(MissionPhase.STOP_SEARCH)
        else:
            self.get_logger().info(f"✗ 检测到 {detected}，但当前需要 {target_colour}，忽略")

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
        elif self._state == MissionPhase.PATROL_SEARCH:
            self._tick_patrol()
        elif self._state == MissionPhase.STOP_SEARCH:
            self._tick_stop_search()
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
        target = self._get_current_target_colour()
        if target is None:
            self.get_logger().info("🎉 所有3个目标已救援完成！任务成功！")
            self._set_state(MissionPhase.IDLE)
        else:
            self.get_logger().info(f"🔍 开始搜索: {target} (进度: {self._rescued_count + 1}/3)")
            self._set_state(MissionPhase.PATROL_SEARCH)

    def _tick_patrol(self):
        """巡逻搜索"""
        self._update_follow_goal()
        if self._follow_goal_handle is None and self._follow_goal_future is None:
            self._send_patrol_goal()

        if self._follow_result_future is not None and self._follow_result_future.done():
            result = self._follow_result_future.result()
            self._clear_follow_goal()
            # 巡逻完一圈，继续巡逻
            self._rotate_patrol_targets()
            self.get_logger().info("巡逻一圈完成，继续搜索...")

    def _tick_stop_search(self):
        """停止巡逻"""
        if self._follow_goal_handle is not None:
            self.get_logger().info("取消巡逻...")
            self._follow_cancel_future = self._follow_goal_handle.cancel_goal_async()
        self._clear_follow_goal()
        
        if self._enable_locate:
            self._set_state(MissionPhase.LOCATE)
        elif self._enable_pick:
            self._set_state(MissionPhase.PICK)
        else:
            self.get_logger().warning("Locate和Pick都被禁用！")
            self._set_state(MissionPhase.PATROL_SEARCH)

    def _tick_locate(self):
        """定位目标"""
        self._update_locate_goal()
        if self._locate_goal_handle is None and self._locate_goal_future is None:
            self._send_locate_goal()

        if self._locate_result_future is not None and self._locate_result_future.done():
            result = self._locate_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            self._clear_locate_goal()
            if success:
                self._set_state(MissionPhase.PICK)
            else:
                self.get_logger().warning("定位失败，继续搜索")
                self._detected_colour = None
                self._set_state(MissionPhase.PATROL_SEARCH)

    def _tick_pick(self):
        """抓取目标"""
        self._update_pick_goal()
        if self._pick_goal_handle is None and self._pick_goal_future is None:
            self._send_pick_goal()

        if self._pick_result_future is not None and self._pick_result_future.done():
            result = self._pick_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            self._clear_pick_goal()
            if success:
                self._rescued_count += 1
                self.get_logger().info(f"✓ 抓取成功！已救援 {self._rescued_count}/3")
                self._set_state(MissionPhase.NAVIGATE_HOME)
            else:
                self.get_logger().warning("抓取失败，继续搜索")
                self._detected_colour = None
                self._set_state(MissionPhase.PATROL_SEARCH)

    def _tick_navigate_home(self):
        """返回Home"""
        self._update_nav_goal()
        if self._nav_goal_handle is None and self._nav_goal_future is None:
            self._send_home_goal()

        if self._nav_result_future is not None and self._nav_result_future.done():
            result = self._nav_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            self._clear_nav_goal()
            if success:
                self._set_state(MissionPhase.DROP)
            else:
                self.get_logger().warning("返回Home失败，仍尝试放置")
                self._set_state(MissionPhase.DROP)

    def _tick_drop(self):
        """放置目标"""
        self._update_drop_goal()
        if self._drop_goal_handle is None and self._drop_goal_future is None:
            self._send_drop_goal()

        if self._drop_result_future is not None and self._drop_result_future.done():
            result = self._drop_result_future.result()
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            self._clear_drop_goal()
            if success:
                self.get_logger().info(f"✓ 放置成功！{self._detected_colour} 已救援")
                # 进入下一个优先级
                self._current_priority_idx += 1
                self._detected_colour = None
                self._set_state(MissionPhase.INIT)
            else:
                self.get_logger().warning("放置失败，继续下一个目标")
                self._current_priority_idx += 1
                self._detected_colour = None
                self._set_state(MissionPhase.INIT)

    # === Action发送函数 ===
    def _send_patrol_goal(self):
        if not self._follow_client.wait_for_server(timeout_sec=1.0):
            return
        stamp = self.get_clock().now().to_msg()
        target = self._get_current_target_colour()
        self.get_logger().info(f"巡逻搜索 {target}，路径: {self._patrol_targets}")
        poses = [
            make_pose_stamped(x, y, 0.0, self._map_frame, stamp)
            for x, y in self._patrol_targets
        ]
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        self._follow_goal_future = self._follow_client.send_goal_async(goal)

    def _send_home_goal(self):
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            return
        goal = NavigateToPose.Goal()
        goal.pose = make_pose_stamped(
            self._home_xy[0], self._home_xy[1], 0.0, self._map_frame, self.get_clock().now().to_msg()
        )
        self._nav_goal_future = self._nav_client.send_goal_async(goal)

    def _send_locate_goal(self):
        if not self._locate_client.wait_for_server(timeout_sec=1.0):
            return
        goal = Locate.Goal()
        self._locate_goal_future = self._locate_client.send_goal_async(goal)

    def _send_pick_goal(self):
        if not self._pick_client.wait_for_server(timeout_sec=1.0):
            return
        goal = Pick.Goal()
        self._pick_goal_future = self._pick_client.send_goal_async(goal)

    def _send_drop_goal(self):
        if not self._drop_client.wait_for_server(timeout_sec=1.0):
            return
        goal = Drop.Goal()
        self._drop_goal_future = self._drop_client.send_goal_async(goal)

    def _rotate_patrol_targets(self):
        if len(self._patrol_targets) > 1:
            self._patrol_targets = self._patrol_targets[1:] + self._patrol_targets[:1]

    # === Goal更新和清理函数 ===
    def _update_follow_goal(self):
        if self._follow_goal_future is not None and self._follow_goal_future.done():
            self._follow_goal_handle = self._follow_goal_future.result()
            self._follow_goal_future = None
            if self._follow_goal_handle.accepted:
                self._follow_result_future = self._follow_goal_handle.get_result_async()

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

    def _clear_follow_goal(self):
        self._follow_goal_future = None
        self._follow_goal_handle = None
        self._follow_result_future = None

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
    node = MissionRunnerChallenge2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
