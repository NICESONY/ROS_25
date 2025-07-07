#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
터틀봇을 지정된 WAYPOINT 들로 순찰시키는 노드
 - Nav2 NavigateToPose 액션 사용
 - AMCL 위치 수신으로 초기 pose 확인
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# ──────────────────────────────────────────────
# ★ 순찰 좌표 (map 기준). 필요하면 원하는 만큼 추가
#    yaw 값은 라디안(π=180°) 단위.
WAYPOINTS = [
    {'x':  1.55, 'y': -0.07, 'yaw': 0.0},
    {'x': -2.73, 'y': -0.13, 'yaw': 0.0},
    {'x': -2.87, 'y': -1.70, 'yaw': 0.0},
    {'x': -2.71, 'y': -3.02, 'yaw': 0.0},
]
# ──────────────────────────────────────────────

class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')
        self.get_logger().info('=== PatrolManager 시작 ===')

        # Nav2 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Nav2 action 서버 대기 중…')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action 서버 연결 완료')

        # AMCL pose 구독
        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose',
            self.pose_callback, 10
        )

        # 상태
        self.state = 'IDLE'
        self.wp_index = 0

        # 1 Hz 상태 머신
        self.create_timer(1.0, self.state_machine)

    # ───────── 콜백들 ─────────
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def state_machine(self):
        if self.state == 'IDLE' and self.current_pose:
            wp = WAYPOINTS[self.wp_index]
            self.get_logger().info(f"▶ Waypoint {self.wp_index} 전송: {wp}")
            goal = self.make_goal(wp['x'], wp['y'], wp['yaw'])
            self.send_goal(goal)
            self.state = 'NAVIGATING'

    # ───────── Goal 관련 ─────────
    def make_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = ps
        return goal

    def send_goal(self, goal_msg: NavigateToPose.Goal):
        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal 거부됨")
            self.state = 'IDLE'
            return

        self.get_logger().info("Goal 수락 — 진행 중")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        pass  # 필요하면 진행률 출력

    def result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"✓ Waypoint {self.wp_index} 도착")
        else:
            self.get_logger().warn(f"✗ Waypoint {self.wp_index} 실패 (status={status})")

        # 다음 포인트
        self.wp_index = (self.wp_index + 1) % len(WAYPOINTS)
        self.state = 'IDLE'

# ───────── main ─────────
def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
