#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose', 10
        )

        # 한 번만 퍼블리시하기 위해 타이머 설정
        self.timer = self.create_timer(0.5, self.publish_and_shutdown)

    def publish_and_shutdown(self):
        msg = PoseWithCovarianceStamped()
        # Header
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        # Position (Gazebo 스폰 좌표와 동일하게)
        msg.pose.pose.position.x = float(self.declare_parameter('x', 2.1).value)
        msg.pose.pose.position.y = float(self.declare_parameter('y', 0.25).value)
        msg.pose.pose.position.z = 0.0
        # Orientation (yaw → quaternion)
        yaw = float(self.declare_parameter('a', 0.0).value)
        msg.pose.pose.orientation.w = float(__import__('math').cos(yaw/2.0))
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = float(__import__('math').sin(yaw/2.0))
        # Covariance (대각 0.25)
        cov = [0.25 if i % 7 == 0 else 0.0 for i in range(36)]
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.get_logger().info('📍 Published initial pose to /initialpose')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
