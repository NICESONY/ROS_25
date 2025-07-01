#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from std_msgs.msg import Bool

class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        # 액션 클라이언트
        self.nav_to_pose_ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav_through_poses_ac = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        # 서비스 클라이언트
        self.clear_costmap_client = self.create_client(Empty, '/local_costmap/clear_entirely_local_costmap')
        # 이벤트 구독 (LiDAR 트리거)
        self.too_close_sub = self.create_subscription(Bool, '/event/too_close', self.too_close_cb, 10)
        # 미션 시작
        self.get_logger().info('Mission Manager started')
        self.run_mission()

    def run_mission(self):
        # 1단계: 통로 끝으로 이동
        self.send_goal_pose( x=5.0, y=0.0 )
        self.wait_for_result()

        # 2단계: Waypoint 순회 (예시 3개)
        waypoints = [
            self.make_pose(2.0, 1.0, 0.0),
            self.make_pose(4.0, 2.0, 1.57),
            self.make_pose(1.0, 3.0, 3.14),
        ]
        self.send_through_poses(waypoints)
        self.wait_for_result_through()

        self.get_logger().info('All tasks completed!')

    def send_goal_pose(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose(x, y, yaw)
        self.nav_to_pose_ac.wait_for_server()
        self._goal_handle = self.nav_to_pose_ac.send_goal_async(goal_msg)
        self.get_logger().info(f'Sent navigate_to_pose: ({x}, {y})')

    def wait_for_result(self):
        self._goal_handle.add_done_callback(lambda f: self.get_logger().info('Reached goal.'))

    def send_through_poses(self, poses):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = [PoseStamped(p.header, p.pose) for p in poses]
        self.nav_through_poses_ac.wait_for_server()
        self._tp_goal_handle = self.nav_through_poses_ac.send_goal_async(goal_msg)
        self.get_logger().info('Sent navigate_through_poses')

    def wait_for_result_through(self):
        self._tp_goal_handle.add_done_callback(lambda f: self.get_logger().info('Completed waypoints.'))

    def too_close_cb(self, msg):
        if msg.data:
            self.get_logger().warn('Too close! clearing costmap and retrying...')
            req = Empty.Request()
            self.clear_costmap_client.call_async(req)

    def make_pose(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = q
        return ps

def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
