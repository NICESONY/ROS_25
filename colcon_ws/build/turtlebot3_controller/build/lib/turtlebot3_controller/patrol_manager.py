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
        self.get_logger().info('=== PatrolManager 시작 ===')

        # Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().debug('ActionClient 생성 완료')

        # 현재 위치 (AMCL) 수신
        self.current_pose = None
        self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10
        )
        self.get_logger().debug('amcl_pose Subscription 생성 완료')

        # 상태 머신 변수
        self.state = 'IDLE'
        self.wp_index = 0
        self.goal_handle = None

        # 타이머 콜백
        self.get_logger().info('타이머 1.0s 주기로 등록')
        self.create_timer(1.0, self.check_and_act)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose
        # orientation으로부터 yaw 계산
        w = self.current_pose.orientation.w
        z = self.current_pose.orientation.z
        yaw = math.atan2(2.0 * (w * z), 1.0 - 2.0 * (z ** 2))
        # 로그 분리 출력
        self.get_logger().info(f"[pose_cb] AMCL 수신 → "
                               f"x={self.current_pose.position.x:.2f}, "
                               f"y={self.current_pose.position.y:.2f}, "
                               f"yaw={yaw:.2f}")

    def check_and_act(self):
        # 상태 머신 진입 로그
        self.get_logger().debug(f"[check_and_act] state={self.state}, "
                                f"current_pose={'set' if self.current_pose else 'None'}")
        if self.state == 'IDLE' and self.current_pose:
            wp = WAYPOINTS[self.wp_index]
            self.get_logger().info(f"[IDLE] → NAVIGATING to wp[{self.wp_index}]: {wp}")
            goal = self.make_goal(wp['x'], wp['y'], wp['yaw'])
            self.send_goal(goal)
            self.state = 'NAVIGATING'

        elif self.state == 'NAVIGATING':
            self.get_logger().debug("[NAVIGATING] 상태, goal_handle 조사 중…")
            if self.goal_handle is not None and self.goal_handle.result():
                status = self.goal_handle.get_status()
                self.get_logger().info(f"[Result] 상태 코드: {status}")
                if status == NavigateToPose.Result().status.SUCCEEDED:
                    self.get_logger().info(f"→ Waypoint {self.wp_index} 도착 성공")
                else:
                    self.get_logger().warn(f"→ Waypoint {self.wp_index} 실패 (status={status})")
                # 다음으로
                self.wp_index = (self.wp_index + 1) % len(WAYPOINTS)
                self.state = 'IDLE'
                self.get_logger().info(f"다음 웨이포인트 인덱스: {self.wp_index}")

    def make_goal(self, x: float, y: float, yaw: float) -> NavigateToPose.Goal:
        from geometry_msgs.msg import PoseStamped
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
        goal.pose = ps
        self.get_logger().debug(f"[make_goal] x={x}, y={y}, yaw={yaw}")
        return goal

    def send_goal(self, goal_msg: NavigateToPose.Goal):
        self.get_logger().info("[send_goal] Action 서버 연결 대기...")
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("[send_goal] Action 서버 미발견")
            return
        self.get_logger().info("[send_goal] Action 서버 연결 성공")
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        resp = future.result()
        if not resp.accepted:
            self.get_logger().error("[goal_response] Goal 거부됨")
            return
        self.goal_handle = resp
        self.get_logger().info("[goal_response] Goal 수용됨")

def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
