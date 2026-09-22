#!/usr/bin/env python3
import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String


class NavAgent(Node):
    """导航监控节点。

    该节点只读取 pose 和 goal 并发布监控结果：
    - 不调用 goToPose
    - 不取消任务
    - 不管理 waypoint 队列

    外部定位系统必须保证 pose 所在坐标系已经与 waypoint 坐标系对齐，
    否则这里只会做错误告警，不做 TF 变换补偿。
    """

    def __init__(self) -> None:
        super().__init__('nav_agent')

        self.declare_parameter('pose_topic', '/Odometry')
        self.declare_parameter('pose_topic_type', 'odometry')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('arrival_status_topic', '/agent/arrival_status')
        self.declare_parameter('feedback_topic', '/agent/feedback_log')
        self.declare_parameter('state_topic', '/agent/nav_state')
        self.declare_parameter('arrival_tolerance', 0.08)
        self.declare_parameter('yaw_tolerance_deg', 20.0)
        self.declare_parameter('use_yaw_tolerance', False)
        self.declare_parameter('goal_timeout_sec', 45.0)
        self.declare_parameter('pose_stale_timeout_sec', 1.0)
        self.declare_parameter('allow_frame_mismatch', False)
        self.declare_parameter('monitor_rate_hz', 10.0)

        pose_topic = self.get_parameter('pose_topic').value
        self.pose_topic_type = str(self.get_parameter('pose_topic_type').value).strip().lower()
        goal_topic = self.get_parameter('goal_topic').value
        self.arrival_topic = self.get_parameter('arrival_status_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.arrival_tolerance = float(self.get_parameter('arrival_tolerance').value)
        self.yaw_tolerance_deg = float(self.get_parameter('yaw_tolerance_deg').value)
        self.use_yaw_tolerance = bool(self.get_parameter('use_yaw_tolerance').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.pose_stale_timeout_sec = float(self.get_parameter('pose_stale_timeout_sec').value)
        self.allow_frame_mismatch = bool(self.get_parameter('allow_frame_mismatch').value)
        monitor_rate_hz = float(self.get_parameter('monitor_rate_hz').value)

        self.pose_sub = self._create_pose_subscription(pose_topic)
        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self._goal_callback, 20)
        self.arrival_pub = self.create_publisher(Bool, self.arrival_topic, 10)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.pose_frame_id = ''
        self.last_pose_time = None
        self.goal_msg: Optional[PoseStamped] = None
        self.goal_start_time = None
        self.arrival_sent = False
        self.failure_sent = False
        self.have_pose = False
        self.frame_mismatch_warned = False
        self.pose_stale_warned = False
        self.last_state: Optional[str] = None

        self.timer = self.create_timer(1.0 / max(monitor_rate_hz, 1.0), self._monitor_loop)
        self._publish_state('IDLE')
        self.get_logger().info(
            'NavAgent 已就绪，只做监控，不主动控制导航。'
            f' pose_topic={pose_topic}, pose_topic_type={self.pose_topic_type}'
        )

    def _create_pose_subscription(self, pose_topic: str):
        if self.pose_topic_type == 'odometry':
            return self.create_subscription(Odometry, pose_topic, self._odometry_callback, 20)
        if self.pose_topic_type == 'pose_stamped':
            return self.create_subscription(PoseStamped, pose_topic, self._pose_stamped_callback, 20)
        if self.pose_topic_type == 'pose_with_covariance_stamped':
            return self.create_subscription(
                PoseWithCovarianceStamped,
                pose_topic,
                self._pose_with_covariance_callback,
                20,
            )
        raise RuntimeError(
            'Unsupported pose_topic_type. Use one of: odometry, pose_stamped, '
            'pose_with_covariance_stamped.'
        )

    def _update_pose(self, x: float, y: float, yaw: float, frame_id: str) -> None:
        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw
        self.pose_frame_id = frame_id
        self.last_pose_time = self.get_clock().now()
        self.have_pose = True
        self.pose_stale_warned = False

    def _odometry_callback(self, msg: Odometry) -> None:
        self._update_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._yaw_from_quaternion(msg.pose.pose.orientation),
            msg.header.frame_id,
        )

    def _pose_stamped_callback(self, msg: PoseStamped) -> None:
        self._update_pose(
            msg.pose.position.x,
            msg.pose.position.y,
            self._yaw_from_quaternion(msg.pose.orientation),
            msg.header.frame_id,
        )

    def _pose_with_covariance_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._update_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._yaw_from_quaternion(msg.pose.pose.orientation),
            msg.header.frame_id,
        )

    def _goal_callback(self, msg: PoseStamped) -> None:
        self.goal_msg = msg
        self.goal_start_time = self.get_clock().now()
        self.arrival_sent = False
        self.failure_sent = False
        self.frame_mismatch_warned = False
        self.pose_stale_warned = False
        self._publish_state('TRACKING')
        self._publish_feedback(
            f'开始监控目标点: x={msg.pose.position.x:.3f}, '
            f'y={msg.pose.position.y:.3f}, frame={msg.header.frame_id}',
            log_to_console=True,
        )

    def _monitor_loop(self) -> None:
        if self.goal_msg is None:
            if not self.have_pose:
                self._publish_state('WAITING_POSE')
            return

        if self.arrival_sent:
            return

        if not self.have_pose:
            self._publish_state('WAITING_POSE')
            self._check_goal_timeout('尚未收到定位数据')
            return

        now = self.get_clock().now()
        pose_age_sec = None
        if self.last_pose_time is not None:
            pose_age_sec = (now - self.last_pose_time).nanoseconds / 1e9

        if pose_age_sec is not None and self.pose_stale_timeout_sec > 0.0:
            if pose_age_sec > self.pose_stale_timeout_sec:
                self._publish_state('POSE_STALE')
                if not self.pose_stale_warned:
                    self.pose_stale_warned = True
                    self._publish_feedback(
                        f'定位数据超时: pose_age={pose_age_sec:.2f}s, '
                        'nav_agent 暂不判定到点。',
                        level='warn',
                        log_to_console=True,
                    )
                self._check_goal_timeout('定位数据超时')
                return

        goal_frame = self.goal_msg.header.frame_id
        if (
            self.pose_frame_id
            and goal_frame
            and self.pose_frame_id != goal_frame
            and not self.allow_frame_mismatch
        ):
            self._publish_state('FRAME_MISMATCH')
            if not self.frame_mismatch_warned:
                self.frame_mismatch_warned = True
                self._publish_feedback(
                    f'坐标系不一致: pose_frame={self.pose_frame_id}, goal_frame={goal_frame}。'
                    ' 当前未启用 TF 变换，请统一外部定位与 waypoint 坐标系。',
                    level='warn',
                    log_to_console=True,
                )
            self._check_goal_timeout('坐标系不一致')
            return

        goal_x = self.goal_msg.pose.position.x
        goal_y = self.goal_msg.pose.position.y
        goal_yaw = self._yaw_from_quaternion(self.goal_msg.pose.orientation)

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.hypot(dx, dy)
        yaw_error_deg = abs(math.degrees(self._normalize_angle(goal_yaw - self.current_yaw)))
        pose_ok = distance <= self.arrival_tolerance
        yaw_ok = (yaw_error_deg <= self.yaw_tolerance_deg) if self.use_yaw_tolerance else True

        self._publish_feedback(
            f'dist={distance:.3f}m, yaw_err={yaw_error_deg:.1f}deg, '
            f'pose=({self.current_x:.3f},{self.current_y:.3f}), '
            f'pose_frame={self.pose_frame_id}, goal_frame={goal_frame}',
            log_to_console=True,
        )

        if pose_ok and yaw_ok:
            msg = Bool()
            msg.data = True
            self.arrival_pub.publish(msg)
            self.arrival_sent = True
            self.failure_sent = False
            self._publish_state('ARRIVED')
            self._publish_feedback(
                f'到点成功: dist={distance:.3f}, yaw_err={yaw_error_deg:.1f}deg',
                log_to_console=True,
            )
            return

        self._publish_state('TRACKING')
        self._check_goal_timeout(
            f'目标点超时前监控中: dist={distance:.3f}, yaw_err={yaw_error_deg:.1f}deg'
        )

    def _check_goal_timeout(self, reason: str) -> None:
        if self.goal_start_time is None or self.failure_sent or self.arrival_sent:
            return

        age = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        if age <= self.goal_timeout_sec:
            return

        msg = Bool()
        msg.data = False
        self.arrival_pub.publish(msg)
        self.failure_sent = True
        self.arrival_sent = False
        self._publish_state('TIMEOUT')
        self._publish_feedback(
            f'目标点超时: age={age:.1f}s, reason={reason}',
            level='warn',
            log_to_console=True,
        )

    def _publish_feedback(
        self,
        text: str,
        level: str = 'info',
        log_to_console: bool = False,
    ) -> None:
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)

        if not log_to_console:
            return

        if level == 'warn':
            self.get_logger().warn(text)
        elif level == 'error':
            self.get_logger().error(text)
        else:
            self.get_logger().info(text)

    def _publish_state(self, state: str) -> None:
        if state == self.last_state:
            return
        self.last_state = state
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = NavAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
