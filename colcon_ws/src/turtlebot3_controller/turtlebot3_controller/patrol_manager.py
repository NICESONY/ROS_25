#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from turtlebot3_controller.patrol_waypoints import WAYPOINTS

class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')
        self.get_logger().info('PatrolManager 시작')

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 현재 위치 구독
        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose',
            self.pose_callback, 10
        )

        # 상태 & 웨이포인트 인덱스
        self.state = 'IDLE'
        self.wp_index = 0

        # 타이머로 상태 머신 구동
        self.create_timer(1.0, self.check_and_act)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def check_and_act(self):
        if self.state == 'IDLE' and self.current_pose:
            # 다음 웨이포인트로 네비게이트
            wp = WAYPOINTS[self.wp_index]
            goal = self.make_goal(wp['x'], wp['y'], wp['yaw'])
            self.send_goal(goal)
            self.state = 'NAVIGATING'
        elif self.state == 'NAVIGATING':
            # 액션 결과 확인
            if self.goal_handle is not None and self.goal_handle.result():
                status = self.goal_handle.get_status()
                if status == NavigateToPose.Result().status.SUCCEEDED:
                    self.get_logger().info(f"Waypoint {self.wp_index} 도착")
                    # 다음 인덱스
                    self.wp_index = (self.wp_index + 1) % len(WAYPOINTS)
                    self.state = 'IDLE'

    def make_goal(self, x, y, yaw):
        from geometry_msgs.msg import PoseStamped
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = math.sin(yaw/2.0)
        ps.pose.orientation.w = math.cos(yaw/2.0)
        goal.pose = ps
        return goal

    def send_goal(self, goal_msg):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action 서버 미발견')
            return
        future = self.nav_client.send_goal_async(goal_msg)
        self.goal_handle = future.result().accepted_goal_handle
        self.get_logger().info(f"Waypoint {self.wp_index} 전송: ({goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y})")

def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
