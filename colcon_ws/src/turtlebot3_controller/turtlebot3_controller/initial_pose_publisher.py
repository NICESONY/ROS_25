#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # 초기 포즈 한 번만 퍼블리시
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 2.1    # Gazebo 스폰 x_pose
        msg.pose.pose.position.y = 0.25   # Gazebo 스폰 y_pose
        msg.pose.pose.orientation.w = 1.0 # Gazebo 스폰 yaw = 0
        # covariance는 대각만 0.25로 간단히
        cov = [0.25 if i%7==0 else 0.0 for i in range(36)]
        msg.pose.covariance = cov
        # 약간 딜레이 후에 퍼블리시
        self.create_timer(0.5, lambda: self.publish_and_shutdown(pub, msg))

    def publish_and_shutdown(self, pub, msg):
        pub.publish(msg)
        self.get_logger().info("Published initial pose to /initialpose")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
