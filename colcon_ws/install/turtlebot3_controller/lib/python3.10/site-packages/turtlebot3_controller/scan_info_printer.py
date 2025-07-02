#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanInfoPrinter(Node):
    def __init__(self):
        super().__init__('scan_info_printer')
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.cb, 10)

    def cb(self, msg: LaserScan):
        stamp = msg.header.stamp
        self.get_logger().info(f"stamp: {stamp.sec}.{stamp.nanosec:09d}")
        self.get_logger().info(f"frame_id: {msg.header.frame_id}")
        self.get_logger().info(f"angle_min: {msg.angle_min}")
        self.get_logger().info(f"angle_max: {msg.angle_max}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ScanInfoPrinter()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
