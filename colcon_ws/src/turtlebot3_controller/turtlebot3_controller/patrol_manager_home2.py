#!/usr/bin/env python3
# rclpy 기반 — ROS 2 Humble 이상

import math
import rclpy
from rclpy.node       import Node
from rclpy.action     import ActionClient
from geometry_msgs.msg    import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action     import NavigateToPose
from action_msgs.msg      import GoalStatus
from tf_transformations   import quaternion_from_euler   # pip install tf-transformations

WAYPOINTS = [
    {'x':  1.55, 'y': -0.07, 'yaw': 0.0},
    {'x': -2.73, 'y': -0.13, 'yaw': 0.0},
    {'x': -2.87, 'y': -1.70, 'yaw': 0.0},
    {'x': -2.71, 'y': -3.02, 'yaw': 0.0},
]

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # Nav2 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Nav2 action 서버 대기 중…')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action 서버 연결 완료')

        # AMCL 위치 수신
        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',      # 전역 토픽 이름
            self.pose_cb,
            10
        )

        self.wp_idx = 0
        self.create_timer(0.5, self.main_loop)

    def pose_cb(self, msg):
        self.current_pose = msg.pose.pose

    def main_loop(self):
        if self.current_pose is None:
            self.get_logger().debug('AMCL 위치 대기 중…')
            return
        self.send_wp_goal()

    def send_wp_goal(self):
        x, y, yaw_deg = WAYPOINTS[self.wp_idx]
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        yaw = math.radians(yaw_deg)
        q = quaternion_from_euler(0, 0, yaw)
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = ps
        self.get_logger().info(f'▶ Goal #{self.wp_idx} ({x:.2f},{y:.2f},{yaw_deg}°)')
        self._goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=None
        )
        self._goal_future.add_done_callback(self.goal_resp_cb)

    def goal_resp_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal 거부')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'✓ Waypoint {self.wp_idx} 도착')
        else:
            self.get_logger().warn(f'✗ Waypoint {self.wp_idx} 실패 (status={status})')
        self.wp_idx = (self.wp_idx + 1) % len(WAYPOINTS)

def main():
    rclpy.init()
    node = PatrolNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
