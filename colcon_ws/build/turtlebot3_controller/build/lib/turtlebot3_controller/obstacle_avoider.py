#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


"""
Obstacle Avoider Node for TurtleBot3

구독: /scan (sensor_msgs/LaserScan)

퍼블리시: /cmd_vel (geometry_msgs/Twist)

로직:

스캔 데이터 중 전방 [-15° ~ +15°] 구간의 최소 거리(min_dist)를 구한다.

min_dist < 안전거리(safety_distance) 이면

linear.x = 0

angular.z = 회전속도 (예: +0.5 rad/s)

그렇지 않으면

linear.x = 전진속도 (예: 0.2 m/s)

angular.z = 0


"""

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.declare_parameter('safety_distance', 0.5)   # [m]
        self.declare_parameter('forward_speed', 0.05)     # [m/s]
        self.declare_parameter('turn_speed', 0.05)        # [rad/s]
        self.safety_distance = self.get_parameter('safety_distance').value
        self.forward_speed   = self.get_parameter('forward_speed').value
        self.turn_speed      = self.get_parameter('turn_speed').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

    def scan_callback(self, msg: LaserScan):
        # 전방 ±15도 범위 인덱스 계산
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        # 인덱스 범위
        left_idx  = int((0.261799 - angle_min) / angle_increment)   # +15° ≈ 0.2618 rad
        right_idx = int((-0.261799 - angle_min) / angle_increment)  # -15°
        front_ranges = ranges[right_idx:left_idx+1]

        # 최소 거리 계산
        valid_ranges = [r for r in front_ranges if not math.isinf(r)]
        min_dist = min(valid_ranges) if valid_ranges else float('inf')
        self.get_logger().info(f'Front min dist: {min_dist:.2f} m')

        msg_out = Twist()
        if min_dist < self.safety_distance:
            # 장애물 가까이: 회전
            msg_out.linear.x = 0.0
            msg_out.angular.z = self.turn_speed
        else:
            # 전진
            msg_out.linear.x = self.forward_speed
            msg_out.angular.z = 0.0

        self.pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
