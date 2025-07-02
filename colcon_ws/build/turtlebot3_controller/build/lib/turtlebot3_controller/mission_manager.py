#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        self.get_logger().info('MissionManager 시작')

        # 속도 직접 제어용 퍼블리셔
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 현재 위치 수신 (AMCL)
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose',
            self.pose_callback, 10)

        # Nav2 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.current_pose = None
        self.state = 'IDLE'
        self.create_timer(1.0, self.check_and_act)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def check_and_act(self):
        if self.state == 'IDLE' and self.current_pose:
            # 예시 목표: (1.0, 2.0)
            goal_pose = PoseWithCovarianceStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.pose.position.x = -0.7
            goal_pose.pose.pose.position.y = 2.7
            goal_pose.pose.pose.orientation.w = 0.5

            self.send_goal(goal_pose)
            self.state = 'NAVIGATING'

    def send_goal(self, goal_msg):
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action 서버 미발견')
            return
        goal = NavigateToPose.Goal()
        goal.pose = goal_msg
        self.get_logger().info(f"Goal 전송: x={goal_msg.pose.pose.position.x}, "
                               f"y={goal_msg.pose.pose.position.y}")
        self.nav_client.send_goal_async(goal)

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
