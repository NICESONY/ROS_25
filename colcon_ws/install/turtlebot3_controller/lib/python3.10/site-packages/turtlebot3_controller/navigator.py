#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.send_goal(1.0, 0.5, 0.0)  # 예시 목표(1.0, 0.5, 0°)

    def send_goal(self, x, y, yaw):
        # Quaternion (z,w) 생성 (단순화)
        from tf_transformations import quaternion_from_euler
        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)



    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected 😞')
            return
        self.get_logger().info('Goal accepted! 👍')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)


    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Remaining distance: {fb.distance_remaining:.2f} m')


    def result_callback(self, future):
        # future.result() 는 ActionClient.get_result_async() 의 반환 타입
        action_result = future.result()
        result_msg = action_result.result        # 실제 Result 메시지
        status_code = action_result.status       # GoalStatus 코드
        
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().error(f'Navigation failed: status {status_code}')

def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator)

if __name__ == '__main__':
    main()
