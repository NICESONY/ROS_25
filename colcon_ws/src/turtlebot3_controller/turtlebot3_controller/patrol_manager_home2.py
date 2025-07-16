#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PatrolNode for ROS 2 Humble
- Nav2 NavigateToPose action
- 자체 quat_from_yaw()
- AMCL 초기 위치를 코드 내부에서 /initialpose 로 1회 발행
"""

"""
termianl commad

ros2 launch turtlebot3_gazebo turtlebot3_home2.launch.py use_sim_time:=True

ros2 launch turtlebot3_navigation2 navigation2_home2.launch.py

ros2 run turtlebot3_controller patrol_manager_home2


debugging code

ros2 topic echo /amcl_pose


ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "header: {frame_id: 'map'}
pose:
  pose:
    position:    {x: 0.1, y: 0.0, z: 0.0}
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}"

"""



import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import (
    PoseStamped, PoseWithCovarianceStamped, Quaternion)
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

#  원하는 Waypoint 리스트 
WAYPOINTS = [
    {'x':  1.55, 'y': -0.07, 'yaw_deg': 0}, # 1
    {'x': -0.87, 'y': -1.55, 'yaw_deg': 0}, # 2
    {'x': -2.87, 'y': -1.70, 'yaw_deg': 0}, # 4
    {'x': -0.87, 'y': -1.55, 'yaw_deg': 0}, # 2
    {'x': -1.18, 'y': -0.38, 'yaw_deg': 0}, # 7
    {'x': -2.66, 'y': -0.16, 'yaw_deg': 0}, # 8
    {'x': -1.18, 'y': -0.38, 'yaw_deg': 0}, # 7
    {'x': -0.87, 'y': -1.55, 'yaw_deg': 0}, # 2
    {'x':  1.66, 'y': -1.60, 'yaw_deg': 0}, # 9
    {'x':  1.67, 'y': -3.04, 'yaw_deg': 0}, # 10
    {'x': -2.84, 'y': -3.00, 'yaw_deg': 0}, # 11
    {'x':  1.67, 'y': -3.04, 'yaw_deg': 0}, # 10
    {'x':  1.66, 'y': -1.60, 'yaw_deg': 0}, # 9
    {'x':  0.26, 'y': -1.53, 'yaw_deg': 0}, # 9
]
# 
#  AMCL 초기 위치 (하드코딩) 
INIT_X   = 0.10     # [m]
INIT_Y   = 0.00     # [m]
INIT_YAW = 0.0      # [rad]
# 

def quat_from_yaw(yaw_rad: float) -> Quaternion:
    """yaw(rad) → Quaternion(z, w 값만 사용)"""
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(yaw_rad / 2.0),
        w=math.cos(yaw_rad / 2.0)
    )

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        # ① 초기 pose 퍼블리셔 (0.5 s 뒤 한 번만 호출)
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.once_timer = self.create_timer(
            0.5, self.publish_initial_pose)

        # ② Nav2 액션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose,
                                       'navigate_to_pose')
        self.get_logger().info('Nav2 action 서버 대기 중…')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action 서버 연결 완료')

        # ③ AMCL pose 구독
        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_cb,
            10)

        # 순찰 상태 변수
        self.wp_idx = 0
        self.goal_active = False

        # 0.5 s 주기 메인 루프
        self.create_timer(0.5, self.main_loop)

    #  초기 pose 발행 후 타이머 취소 
    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = INIT_X
        msg.pose.pose.position.y = INIT_Y
        msg.pose.pose.orientation = quat_from_yaw(INIT_YAW)

        # 공분산: ±0.25 m, ±10°
        msg.pose.covariance[0]  = msg.pose.covariance[7] = 0.25**2
        msg.pose.covariance[35] = math.radians(10)**2

        # 두 번 발행(유실 대비)
        for _ in range(2):
            self.init_pub.publish(msg)

        self.get_logger().info('📍 초기 pose 발행 완료')
        self.once_timer.cancel()          # 더 이상 필요 없는 타이머 중지

    #  AMCL pose 콜백 
    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    #  메인 루프 
    def main_loop(self):
        if self.goal_active or self.current_pose is None:
            return
        self.send_wp_goal()

    #  목표 전송 
    def send_wp_goal(self):
        wp = WAYPOINTS[self.wp_idx]
        yaw_rad = math.radians(wp['yaw_deg'])

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.pose.position.x = wp['x']
        ps.pose.position.y = wp['y']
        ps.pose.orientation = quat_from_yaw(yaw_rad)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = ps

        self.get_logger().info(
            f"▶ Goal #{self.wp_idx}  "
            f"(x={wp['x']:.2f}, y={wp['y']:.2f}, yaw={wp['yaw_deg']}°)")
        self.goal_active = True

        self.nav_client.send_goal_async(goal_msg).add_done_callback(self.goal_resp_cb)


    #  액션 서버 응답 
    def goal_resp_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal 거부')
            self.goal_active = False
            return
        goal_handle.get_result_async().add_done_callback(self.result_cb)

    #  결과 
    def result_cb(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED :
            self.get_logger().info(f"✓ Waypoint {self.wp_idx} 도착")
        else:
            self.get_logger().warn(
                f"✗ Waypoint {self.wp_idx} 실패 (status={status})")

        self.wp_idx = (self.wp_idx + 1) % len(WAYPOINTS)  # 다음 포인트
        self.goal_active = False


def main():
    rclpy.init()
    rclpy.spin(PatrolNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
