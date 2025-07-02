#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class SensorListener(Node):
    def __init__(self):
        super().__init__('sensor_listener')
        # LaserScan 구독
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        # Odometry 구독
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

    def scan_callback(self, msg: LaserScan):
        # 예시: 앞쪽 0도 방향 거리 출력
        front_index = len(msg.ranges) // 2
        front_dist = msg.ranges[front_index]
        self.get_logger().info(f'Front distance: {front_dist:.2f} m')

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'Odom position: x={x:.2f}, y={y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SensorListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
