#!/usr/bin/env python3
import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from auto_nav_pkg.log_style import colorize_log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, String
from std_msgs.msg import Float32MultiArray

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except Exception as exc:  # pragma: no cover - depends on runtime environment
    BasicNavigator = None
    TaskResult = None
    NAV2_IMPORT_ERROR = exc
else:
    NAV2_IMPORT_ERROR = None


class NavExecutor(Node):
    """导航执行节点。

    支持三种后端：
    - pose_controller: 当前工程默认模式，直接把 goal + pose 转成 /cmd_vel
    - external: 只保留 /goal_pose 接口，不主动控制
    - nav2: 调用 nav2_simple_commander
    """

    def __init__(self) -> None:
        super().__init__('nav_executor')

        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('arrival_status_topic', '/agent/arrival_status')
        self.declare_parameter('executor_state_topic', '/nav_executor/state')
        self.declare_parameter('feedback_topic', '/nav_executor/feedback_log')
        self.declare_parameter('backend', 'pose_controller')
        self.declare_parameter('pose_topic', '/Odometry')
        self.declare_parameter('pose_topic_type', 'odometry')
        self.declare_parameter('pose_stale_timeout_sec', 1.0)
        self.declare_parameter('control_rate_hz', 15.0)
        self.declare_parameter('max_linear_speed_mps', 0.35)
        self.declare_parameter('min_linear_speed_mps', 0.05)
        self.declare_parameter('max_angular_speed_rps', 0.80)
        self.declare_parameter('min_angular_speed_rps', 0.12)
        self.declare_parameter('linear_kp', 0.60)
        self.declare_parameter('angular_kp', 1.50)
        self.declare_parameter('final_yaw_kp', 1.00)
        self.declare_parameter('pose_yaw_offset_deg', 0.0)
        self.declare_parameter('heading_align_threshold_deg', 20.0)
        self.declare_parameter('close_goal_relaxed_heading_enabled', True)
        self.declare_parameter('close_goal_relaxed_heading_distance_m', 0.45)
        self.declare_parameter('close_goal_relaxed_heading_max_error_deg', 110.0)
        self.declare_parameter('close_goal_relaxed_heading_min_heading_scale', 0.20)
        self.declare_parameter('close_goal_relaxed_heading_linear_speed_cap_mps', 0.18)
        self.declare_parameter('slow_down_distance_m', 0.60)
        self.declare_parameter('use_final_yaw_control', False)
        self.declare_parameter('final_yaw_distance_m', 0.12)
        self.declare_parameter('final_yaw_tolerance_deg', 8.0)
        self.declare_parameter('localizer_name', 'bt_navigator')
        self.declare_parameter('navigator_name', 'bt_navigator')
        self.declare_parameter('wait_for_nav2_active', True)
        self.declare_parameter('cancel_previous_goal', True)
        self.declare_parameter('nav2_goal_orientation_mode', 'path_heading')
        self.declare_parameter('nav2_goal_orientation_switch_distance_m', 0.30)
        self.declare_parameter('nav2_action_server_timeout_sec', 2.0)
        self.declare_parameter('nav2_cmd_vel_bridge_enabled', True)
        self.declare_parameter('nav2_cmd_vel_source_topic', '/cmd_vel')
        self.declare_parameter('nav2_local_spin_enabled', True)
        self.declare_parameter('nav2_local_spin_distance_m', 0.18)
        self.declare_parameter('nav2_local_spin_min_yaw_error_deg', 15.0)
        self.declare_parameter('nav2_local_spin_stop_yaw_tolerance_deg', 2.5)
        self.declare_parameter('nav2_local_spin_angular_kp', 2.2)
        self.declare_parameter('nav2_local_spin_max_angular_speed_rps', 1.20)
        self.declare_parameter('nav2_local_spin_min_angular_speed_rps', 0.35)
        self.declare_parameter('step_override_topic', '/serial_step_override')
        self.declare_parameter('nav2_local_spin_use_step_override', True)
        self.declare_parameter('nav2_local_spin_override_mode', 0)
        self.declare_parameter('nav2_local_spin_override_left_norm', -0.2)
        self.declare_parameter('nav2_local_spin_override_right_norm', 0.65)
        # 2026-05-02: Guard against Nav2 reporting success while Point-LIO pose
        # is still far from the requested goal. Without this, /cmd_vel stops and
        # obstacle_manager waits forever at the current waypoint.
        self.declare_parameter('nav2_success_pose_check_enabled', True)
        self.declare_parameter('nav2_success_pose_tolerance_m', 0.45)
        self.declare_parameter('status_check_period_sec', 0.25)
        self.declare_parameter('feedback_log_interval_sec', 2.0)
        self.declare_parameter('nav2_success_pose_mismatch_retry_enabled', True)
        self.declare_parameter('nav2_success_pose_mismatch_retry_limit', 2)
        self.declare_parameter('nav2_success_pose_mismatch_retry_delay_sec', 0.05)

        self.goal_topic = self.get_parameter('goal_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.arrival_status_topic = self.get_parameter('arrival_status_topic').value
        self.state_topic = self.get_parameter('executor_state_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.backend = str(self.get_parameter('backend').value).strip().lower()
        self.pose_topic = self.get_parameter('pose_topic').value
        self.pose_topic_type = str(self.get_parameter('pose_topic_type').value).strip().lower()
        self.pose_stale_timeout_sec = float(self.get_parameter('pose_stale_timeout_sec').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.max_linear_speed_mps = float(self.get_parameter('max_linear_speed_mps').value)
        self.min_linear_speed_mps = float(self.get_parameter('min_linear_speed_mps').value)
        self.max_angular_speed_rps = float(self.get_parameter('max_angular_speed_rps').value)
        self.min_angular_speed_rps = float(self.get_parameter('min_angular_speed_rps').value)
        self.linear_kp = float(self.get_parameter('linear_kp').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.final_yaw_kp = float(self.get_parameter('final_yaw_kp').value)
        self.pose_yaw_offset_rad = math.radians(
            float(self.get_parameter('pose_yaw_offset_deg').value)
        )
        self.heading_align_threshold_rad = math.radians(
            float(self.get_parameter('heading_align_threshold_deg').value)
        )
        self.close_goal_relaxed_heading_enabled = bool(
            self.get_parameter('close_goal_relaxed_heading_enabled').value
        )
        self.close_goal_relaxed_heading_distance_m = max(
            0.0,
            float(self.get_parameter('close_goal_relaxed_heading_distance_m').value),
        )
        self.close_goal_relaxed_heading_max_error_rad = math.radians(
            float(self.get_parameter('close_goal_relaxed_heading_max_error_deg').value)
        )
        self.close_goal_relaxed_heading_min_heading_scale = min(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'close_goal_relaxed_heading_min_heading_scale'
                    ).value
                ),
            ),
            1.0,
        )
        self.close_goal_relaxed_heading_linear_speed_cap_mps = max(
            0.0,
            float(self.get_parameter('close_goal_relaxed_heading_linear_speed_cap_mps').value),
        )
        self.slow_down_distance_m = float(self.get_parameter('slow_down_distance_m').value)
        self.use_final_yaw_control = bool(self.get_parameter('use_final_yaw_control').value)
        self.final_yaw_distance_m = float(self.get_parameter('final_yaw_distance_m').value)
        self.final_yaw_tolerance_rad = math.radians(
            float(self.get_parameter('final_yaw_tolerance_deg').value)
        )
        self.localizer_name = str(self.get_parameter('localizer_name').value).strip()
        self.navigator_name = str(self.get_parameter('navigator_name').value).strip() or 'bt_navigator'
        self.wait_for_nav2_active = bool(self.get_parameter('wait_for_nav2_active').value)
        self.cancel_previous_goal = bool(self.get_parameter('cancel_previous_goal').value)
        self.nav2_goal_orientation_mode = str(
            self.get_parameter('nav2_goal_orientation_mode').value
        ).strip().lower()
        self.nav2_goal_orientation_switch_distance_m = max(
            0.0,
            float(self.get_parameter('nav2_goal_orientation_switch_distance_m').value),
        )
        self.nav2_action_server_timeout_sec = max(
            0.0,
            float(self.get_parameter('nav2_action_server_timeout_sec').value),
        )
        self.nav2_cmd_vel_bridge_enabled = bool(
            self.get_parameter('nav2_cmd_vel_bridge_enabled').value
        )
        self.nav2_cmd_vel_source_topic = str(
            self.get_parameter('nav2_cmd_vel_source_topic').value
        ).strip()
        self.nav2_local_spin_enabled = bool(self.get_parameter('nav2_local_spin_enabled').value)
        self.nav2_local_spin_distance_m = max(
            0.0,
            float(self.get_parameter('nav2_local_spin_distance_m').value),
        )
        self.nav2_local_spin_min_yaw_error_rad = math.radians(
            float(self.get_parameter('nav2_local_spin_min_yaw_error_deg').value)
        )
        self.nav2_local_spin_stop_yaw_tolerance_rad = math.radians(
            float(self.get_parameter('nav2_local_spin_stop_yaw_tolerance_deg').value)
        )
        self.nav2_local_spin_angular_kp = float(
            self.get_parameter('nav2_local_spin_angular_kp').value
        )
        self.nav2_local_spin_max_angular_speed_rps = float(
            self.get_parameter('nav2_local_spin_max_angular_speed_rps').value
        )
        self.nav2_local_spin_min_angular_speed_rps = float(
            self.get_parameter('nav2_local_spin_min_angular_speed_rps').value
        )
        self.step_override_topic = str(self.get_parameter('step_override_topic').value).strip()
        self.nav2_local_spin_use_step_override = bool(
            self.get_parameter('nav2_local_spin_use_step_override').value
        )
        self.nav2_local_spin_override_mode = int(
            self.get_parameter('nav2_local_spin_override_mode').value
        )
        self.nav2_local_spin_override_left_norm = float(
            self.get_parameter('nav2_local_spin_override_left_norm').value
        )
        self.nav2_local_spin_override_right_norm = float(
            self.get_parameter('nav2_local_spin_override_right_norm').value
        )
        self.nav2_success_pose_check_enabled = bool(
            self.get_parameter('nav2_success_pose_check_enabled').value
        )
        self.nav2_success_pose_tolerance_m = max(
            0.0,
            float(self.get_parameter('nav2_success_pose_tolerance_m').value),
        )
        self.status_check_period_sec = max(
            0.05,
            float(self.get_parameter('status_check_period_sec').value),
        )
        self.feedback_log_interval_sec = max(
            0.0,
            float(self.get_parameter('feedback_log_interval_sec').value),
        )
        self.nav2_success_pose_mismatch_retry_enabled = bool(
            self.get_parameter('nav2_success_pose_mismatch_retry_enabled').value
        )
        self.nav2_success_pose_mismatch_retry_limit = max(
            0,
            int(self.get_parameter('nav2_success_pose_mismatch_retry_limit').value),
        )
        self.nav2_success_pose_mismatch_retry_delay_sec = max(
            0.0,
            float(self.get_parameter('nav2_success_pose_mismatch_retry_delay_sec').value),
        )
        if self.nav2_goal_orientation_mode not in ('goal', 'path_heading', 'current_heading'):
            self.get_logger().warn(
                f'Unsupported nav2_goal_orientation_mode={self.nav2_goal_orientation_mode}, '
                'fallback to path_heading.'
            )
            self.nav2_goal_orientation_mode = 'path_heading'

        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.step_override_pub = self.create_publisher(
            Float32MultiArray,
            self.step_override_topic,
            10,
        )
        goal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            # 2026-05-03 00:12 CST: accept both RViz/Nav2 volatile publishers
            # and local task-manager publishers. Revert to TRANSIENT_LOCAL only
            # if late-joining goal replay becomes required again.
            durability=DurabilityPolicy.VOLATILE,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped, self.goal_topic, self._goal_callback, goal_qos
        )
        self.arrival_sub = self.create_subscription(
            Bool,
            self.arrival_status_topic,
            self._arrival_callback,
            10,
        )

        self.cmd_vel_pub = None
        self.nav2_cmd_vel_sub = None
        self.pose_sub = None
        self.control_timer = None
        self.navigator = None

        self._last_goal: Optional[PoseStamped] = None
        self.goal_active = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.last_pose_time = None
        self.have_pose = False
        self.pose_stale_warned = False
        self.last_state: Optional[str] = None
        self.last_stop_reason: Optional[str] = None
        self.zero_cmd_sent = False
        self.nav2_completion_reported = False
        self.local_spin_active = False
        self.last_local_spin_log_time = 0.0
        self.last_feedback_log_time = 0.0
        self.nav2_pose_mismatch_retry_count = 0
        self.nav2_pose_mismatch_retry_timer = None

        self.feedback_timer = self.create_timer(self.status_check_period_sec, self._feedback_loop)
        self._setup_backend()

    def _setup_backend(self) -> None:
        if self.backend == 'pose_controller':
            self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
            self.pose_sub = self._create_pose_subscription(self.pose_topic)
            self.control_timer = self.create_timer(
                1.0 / max(self.control_rate_hz, 1.0),
                self._pose_controller_loop,
            )
            self._publish_state('READY')
            self._publish_feedback(
                'NavExecutor 运行在 pose_controller 模式：'
                f' goal={self.goal_topic}, pose={self.pose_topic}, cmd_vel={self.cmd_vel_topic}'
            )
            return

        if self.backend == 'nav2':
            if BasicNavigator is None:
                self._publish_feedback(
                    'nav2_simple_commander 不可用，自动降级为 pose_controller 模式。'
                    f' import_error={NAV2_IMPORT_ERROR}',
                    level='warn',
                )
                self.backend = 'pose_controller'
                self._setup_backend()
                return

            self.pose_sub = self._create_pose_subscription(self.pose_topic)
            if self.nav2_local_spin_enabled or self.nav2_cmd_vel_bridge_enabled:
                self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
            if self.nav2_cmd_vel_bridge_enabled:
                self.nav2_cmd_vel_sub = self.create_subscription(
                    Twist,
                    self.nav2_cmd_vel_source_topic,
                    self._nav2_cmd_vel_callback,
                    10,
                )
                self._publish_feedback(
                    f'Nav2 cmd_vel 桥接启用: {self.nav2_cmd_vel_source_topic} -> {self.cmd_vel_topic}'
                )
            if self.nav2_local_spin_enabled:
                self.control_timer = self.create_timer(
                    1.0 / max(self.control_rate_hz, 1.0),
                    self._nav2_local_spin_loop,
                )
            self.navigator = BasicNavigator()
            if (
                self.nav2_action_server_timeout_sec > 0.0 and
                not self.navigator.nav_to_pose_client.wait_for_server(
                    timeout_sec=self.nav2_action_server_timeout_sec
                )
            ):
                self._publish_feedback(
                    'NavigateToPose action server 不可用，自动降级为 pose_controller 模式。',
                    level='warn',
                )
                if self.nav2_cmd_vel_sub is not None:
                    self.destroy_subscription(self.nav2_cmd_vel_sub)
                    self.nav2_cmd_vel_sub = None
                if self.control_timer is not None:
                    self.destroy_timer(self.control_timer)
                    self.control_timer = None
                if self.cmd_vel_pub is not None:
                    self.destroy_publisher(self.cmd_vel_pub)
                    self.cmd_vel_pub = None
                if self.pose_sub is not None:
                    self.destroy_subscription(self.pose_sub)
                    self.pose_sub = None
                self.navigator.destroy_node()
                self.navigator = None
                self.backend = 'pose_controller'
                self._setup_backend()
                return
            if self.wait_for_nav2_active:
                self._wait_until_nav2_active()
            self._publish_state('READY')
            self._publish_feedback('NavExecutor 已连接 Nav2。')
            return

        self._publish_state('RELAY_ONLY')
        self._publish_feedback(
            'NavExecutor 运行在 external 模式，只保留 /goal_pose 接口，不主动发布 /cmd_vel。'
        )

    def _create_pose_subscription(self, pose_topic: str):
        if self.pose_topic_type == 'odometry':
            return self.create_subscription(
                Odometry,
                pose_topic,
                self._odometry_callback,
                qos_profile_sensor_data,
            )
        if self.pose_topic_type == 'pose_stamped':
            return self.create_subscription(
                PoseStamped,
                pose_topic,
                self._pose_stamped_callback,
                qos_profile_sensor_data,
            )
        if self.pose_topic_type == 'pose_with_covariance_stamped':
            return self.create_subscription(
                PoseWithCovarianceStamped,
                pose_topic,
                self._pose_with_covariance_callback,
                qos_profile_sensor_data,
            )
        raise RuntimeError(
            'Unsupported pose_topic_type. Use one of: odometry, pose_stamped, '
            'pose_with_covariance_stamped.'
        )

    def _update_pose(self, x: float, y: float, yaw: float) -> None:
        self.current_x = x
        self.current_y = y
        self.current_yaw = yaw
        self.last_pose_time = self.get_clock().now()
        self.have_pose = True
        self.pose_stale_warned = False

    def _odometry_callback(self, msg: Odometry) -> None:
        self._update_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def _pose_stamped_callback(self, msg: PoseStamped) -> None:
        self._update_pose(
            msg.pose.position.x,
            msg.pose.position.y,
            self._yaw_from_quaternion(msg.pose.orientation),
        )

    def _pose_with_covariance_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._update_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._yaw_from_quaternion(msg.pose.pose.orientation),
        )

    def _goal_callback(self, msg: PoseStamped) -> None:
        self._clear_nav2_pose_mismatch_retry_timer()
        self._last_goal = msg
        self.goal_active = True
        self.last_stop_reason = None
        self.zero_cmd_sent = False
        self.nav2_completion_reported = False
        self.local_spin_active = False
        self.nav2_pose_mismatch_retry_count = 0
        self._clear_step_override()
        self._publish_feedback(
            f'收到导航目标: x={msg.pose.position.x:.3f}, '
            f'y={msg.pose.position.y:.3f}, frame={msg.header.frame_id}, backend={self.backend}'
        )

        if self.backend == 'nav2' and self.navigator is not None:
            if self._should_use_local_spin(msg):
                self._activate_local_spin(msg, 'goal_nearby_with_large_yaw_error')
                self._publish_state('LOCAL_SPIN')
                return
            try:
                if self.cancel_previous_goal and not self.navigator.isTaskComplete():
                    self._publish_feedback('取消当前导航任务，切换到新目标。', level='warn')
                    self.navigator.cancelTask()
            except Exception:
                pass
            self.navigator.goToPose(self._prepare_nav2_goal(msg))

        self._publish_state('NAVIGATING')

    def _arrival_callback(self, msg: Bool) -> None:
        if not self.goal_active:
            return

        if msg.data:
            self._stop_motion('arrival_confirmed')
            self._publish_state('ARRIVED')
            return

        self._stop_motion('arrival_failed')
        self._publish_state('FAILED')

    def _nav2_cmd_vel_callback(self, msg: Twist) -> None:
        if self.backend != 'nav2' or self.cmd_vel_pub is None:
            return
        if self.local_spin_active:
            return
        self.cmd_vel_pub.publish(msg)

    def _pose_controller_loop(self) -> None:
        if self.backend != 'pose_controller':
            return

        if not self.goal_active or self._last_goal is None:
            self._publish_zero_cmd()
            if self.last_state not in ('READY', 'ARRIVED', 'FAILED', 'RELAY_ONLY'):
                self._publish_state('READY')
            return

        if not self.have_pose:
            self._publish_zero_cmd()
            self._publish_state('WAITING_POSE')
            return

        if self._pose_is_stale():
            self._publish_zero_cmd()
            self._publish_state('POSE_STALE')
            return

        cmd = self._compute_cmd_vel(self._last_goal)
        self.cmd_vel_pub.publish(cmd)
        self.zero_cmd_sent = False

    def _nav2_local_spin_loop(self) -> None:
        if self.backend != 'nav2':
            return

        if not self.local_spin_active or not self.goal_active:
            return

        if self._last_goal is None:
            self._clear_step_override()
            self._publish_zero_cmd()
            return

        if not self.have_pose:
            self._clear_step_override()
            self._publish_zero_cmd()
            self._publish_state('WAITING_POSE')
            return

        if self._pose_is_stale():
            self._clear_step_override()
            self._publish_zero_cmd()
            self._publish_state('POSE_STALE')
            return

        goal_yaw = self._yaw_from_quaternion(self._last_goal.pose.orientation)
        controller_yaw = self._controller_yaw()
        # Old: yaw_error = self._normalize_angle(goal_yaw - self.current_yaw)
        yaw_error = self._normalize_angle(goal_yaw - controller_yaw)
        distance = math.hypot(
            self._last_goal.pose.position.x - self.current_x,
            self._last_goal.pose.position.y - self.current_y,
        )

        if abs(yaw_error) <= self.nav2_local_spin_stop_yaw_tolerance_rad:
            self._clear_step_override()
            self._publish_zero_cmd()
            return

        if self.nav2_local_spin_use_step_override:
            self._publish_step_override(
                self.nav2_local_spin_override_left_norm,
                self.nav2_local_spin_override_right_norm,
            )
            self._publish_zero_cmd()
        else:
            cmd = Twist()
            cmd.angular.z = self._clamp_with_min_abs(
                self.nav2_local_spin_angular_kp * yaw_error,
                self.nav2_local_spin_max_angular_speed_rps,
                self.nav2_local_spin_min_angular_speed_rps,
            )
            if self.cmd_vel_pub is not None:
                self.cmd_vel_pub.publish(cmd)
        self.zero_cmd_sent = False
        self._publish_state('LOCAL_SPIN')

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if (now_sec - self.last_local_spin_log_time) >= 1.0:
            self.last_local_spin_log_time = now_sec
            if self.nav2_local_spin_use_step_override:
                self._publish_feedback(
                    f'Local spin override: dist={distance:.3f}m, '
                    f'yaw_err={math.degrees(yaw_error):.1f}deg, '
                    f'mode={self.nav2_local_spin_override_mode}, '
                    f'left={self.nav2_local_spin_override_left_norm:.3f}, '
                    f'right={self.nav2_local_spin_override_right_norm:.3f}',
                    log_to_console=True,
                )
            else:
                self._publish_feedback(
                    f'Local spin: dist={distance:.3f}m, yaw_err={math.degrees(yaw_error):.1f}deg, '
                    f'cmd_wz={cmd.angular.z:.3f}rad/s',
                    log_to_console=True,
                )

    def _compute_cmd_vel(self, goal: PoseStamped) -> Twist:
        cmd = Twist()

        dx = goal.pose.position.x - self.current_x
        dy = goal.pose.position.y - self.current_y
        distance = math.hypot(dx, dy)
        controller_yaw = self._controller_yaw()
        # Old: target_heading = math.atan2(dy, dx) if distance > 1e-6 else self.current_yaw
        target_heading = math.atan2(dy, dx) if distance > 1e-6 else controller_yaw
        # Old: heading_error = self._normalize_angle(target_heading - self.current_yaw)
        heading_error = self._normalize_angle(target_heading - controller_yaw)
        goal_yaw = self._yaw_from_quaternion(goal.pose.orientation)
        # Old: final_yaw_error = self._normalize_angle(goal_yaw - self.current_yaw)
        final_yaw_error = self._normalize_angle(goal_yaw - controller_yaw)

        if self.use_final_yaw_control and distance <= self.final_yaw_distance_m:
            if abs(final_yaw_error) > self.final_yaw_tolerance_rad:
                cmd.angular.z = self._clamp_with_min_abs(
                    self.final_yaw_kp * final_yaw_error,
                    self.max_angular_speed_rps,
                    self.min_angular_speed_rps,
                )
            return cmd

        # 2026-05-04 10:42 CST: short pole-route waypoints can be very close
        # to the robot while still needing a 40-100deg heading correction. A
        # pure spin here stalls route progress and fights the direct-turn
        # controller. Keep the old pure-spin behavior for far goals and for
        # extremely large close-range heading errors, but allow close goals to
        # advance with a slow arc turn instead of zeroing linear motion.
        # Rollback: set close_goal_relaxed_heading_enabled=false.
        close_goal_relaxed_heading = (
            self.close_goal_relaxed_heading_enabled
            and distance <= self.close_goal_relaxed_heading_distance_m
            and abs(heading_error) <= self.close_goal_relaxed_heading_max_error_rad
        )

        if abs(heading_error) > self.heading_align_threshold_rad and not close_goal_relaxed_heading:
            cmd.angular.z = self._clamp_with_min_abs(
                self.angular_kp * heading_error,
                self.max_angular_speed_rps,
                self.min_angular_speed_rps,
            )
            return cmd

        linear_scale = 1.0
        if self.slow_down_distance_m > 1e-6:
            linear_scale = min(distance / self.slow_down_distance_m, 1.0)

        heading_scale = max(math.cos(heading_error), 0.0)
        if close_goal_relaxed_heading:
            heading_scale = max(
                heading_scale,
                self.close_goal_relaxed_heading_min_heading_scale,
            )
        raw_linear = self.linear_kp * distance * linear_scale * heading_scale
        raw_angular = self.angular_kp * heading_error

        cmd.linear.x = self._clamp_with_min_abs(
            raw_linear,
            self.max_linear_speed_mps,
            self.min_linear_speed_mps,
        )
        if close_goal_relaxed_heading and self.close_goal_relaxed_heading_linear_speed_cap_mps > 0.0:
            cmd.linear.x = math.copysign(
                min(abs(cmd.linear.x), self.close_goal_relaxed_heading_linear_speed_cap_mps),
                cmd.linear.x,
            )
        cmd.angular.z = self._clamp_with_min_abs(
            raw_angular,
            self.max_angular_speed_rps,
            self.min_angular_speed_rps,
        )
        return cmd

    def _pose_is_stale(self) -> bool:
        if self.last_pose_time is None or self.pose_stale_timeout_sec <= 0.0:
            return False

        age = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if age <= self.pose_stale_timeout_sec:
            return False

        if not self.pose_stale_warned:
            self.pose_stale_warned = True
            self._publish_feedback(
                f'定位数据超时: pose_age={age:.2f}s，nav_executor 暂停输出 /cmd_vel。',
                level='warn',
            )
        return True

    def _publish_zero_cmd(self) -> None:
        if self.cmd_vel_pub is None or self.zero_cmd_sent:
            return

        self.cmd_vel_pub.publish(Twist())
        self.zero_cmd_sent = True

    def _stop_motion(self, reason: str) -> None:
        self._clear_nav2_pose_mismatch_retry_timer()
        self.goal_active = False
        self.local_spin_active = False
        self.last_stop_reason = reason
        self._clear_step_override()
        self._publish_zero_cmd()

        if self.backend == 'nav2' and self.navigator is not None:
            try:
                self.navigator.cancelTask()
            except Exception:
                pass

        self._publish_feedback(f'导航执行结束，reason={reason}')

    def _prepare_nav2_goal(self, goal: PoseStamped) -> PoseStamped:
        if self.nav2_goal_orientation_mode == 'goal':
            return goal

        if not self.have_pose:
            self._publish_feedback(
                'Nav2 目标朝向改写被跳过：当前还没有 pose，继续使用 YAML 里的 yaw。',
                level='warn',
            )
            return goal

        # Old: yaw = self.current_yaw
        yaw = self._controller_yaw()
        dx = goal.pose.position.x - self.current_x
        dy = goal.pose.position.y - self.current_y
        distance = math.hypot(dx, dy)
        if self.nav2_goal_orientation_mode == 'path_heading':
            if distance <= self.nav2_goal_orientation_switch_distance_m:
                yaw = self._yaw_from_quaternion(goal.pose.orientation)
            elif abs(dx) > 1e-6 or abs(dy) > 1e-6:
                yaw = math.atan2(dy, dx)

        adjusted = PoseStamped()
        adjusted.header = goal.header
        adjusted.pose.position.x = goal.pose.position.x
        adjusted.pose.position.y = goal.pose.position.y
        adjusted.pose.position.z = goal.pose.position.z
        adjusted.pose.orientation = self._quaternion_from_yaw(yaw)

        self._publish_feedback(
            f'Nav2 发送目标时使用朝向模式={self.nav2_goal_orientation_mode}, '
            f'adjusted_yaw={math.degrees(yaw):.1f}deg, '
            f'goal_distance={distance:.3f}m',
            log_to_console=True,
        )
        return adjusted

    def _wait_until_nav2_active(self) -> None:
        if self.navigator is None:
            return

        localizer = self.localizer_name.strip()
        localizer_disabled = localizer.lower() in ('', 'none', 'disabled', 'off', 'false')
        if localizer_disabled:
            self._publish_feedback(
                f'等待 Nav2 激活: navigator={self.navigator_name}, localizer=disabled'
            )
            self.navigator._waitForNodeToActivate(self.navigator_name)
            return

        if localizer == self.navigator_name:
            self._publish_feedback(
                f'等待 Nav2 激活: navigator={self.navigator_name}, '
                f'localizer={localizer} 与 navigator 相同，按单节点等待处理。'
            )
            self.navigator._waitForNodeToActivate(self.navigator_name)
            return

        self._publish_feedback(
            f'等待 Nav2 激活: navigator={self.navigator_name}, localizer={localizer}'
        )
        self.navigator.waitUntilNav2Active(
            navigator=self.navigator_name,
            localizer=localizer,
        )

    def _nav2_result_to_text(self, result) -> str:
        if TaskResult is None:
            return str(result)
        if result == TaskResult.SUCCEEDED:
            return 'SUCCEEDED'
        if result == TaskResult.FAILED:
            return 'FAILED'
        if result == TaskResult.CANCELED:
            return 'CANCELED'
        return 'UNKNOWN'

    def _nav2_success_pose_consistent(self, goal: PoseStamped) -> tuple:
        if not self.nav2_success_pose_check_enabled:
            return True, 0.0
        if self.nav2_success_pose_tolerance_m <= 0.0:
            return True, 0.0
        if not self.have_pose or self._pose_is_stale():
            return False, math.inf

        distance = math.hypot(
            goal.pose.position.x - self.current_x,
            goal.pose.position.y - self.current_y,
        )
        return distance <= self.nav2_success_pose_tolerance_m, distance

    def _clear_nav2_pose_mismatch_retry_timer(self) -> None:
        if self.nav2_pose_mismatch_retry_timer is None:
            return
        self.destroy_timer(self.nav2_pose_mismatch_retry_timer)
        self.nav2_pose_mismatch_retry_timer = None

    def _schedule_nav2_pose_mismatch_retry(self, pose_distance: float) -> bool:
        if not self.nav2_success_pose_mismatch_retry_enabled:
            return False
        if self.navigator is None or self._last_goal is None:
            return False
        if self.nav2_pose_mismatch_retry_count >= self.nav2_success_pose_mismatch_retry_limit:
            return False

        self.nav2_pose_mismatch_retry_count += 1
        self.nav2_completion_reported = False
        self.zero_cmd_sent = False
        self._publish_state('NAVIGATING')
        self._publish_feedback(
            'Nav2 返回 SUCCEEDED 但 pose 未到目标，节点内部快速重发同一目标，'
            '不交给 obstacle_manager 走 1 秒重试流程: '
            f'retry={self.nav2_pose_mismatch_retry_count}/'
            f'{self.nav2_success_pose_mismatch_retry_limit}, '
            f'pose_dist={pose_distance:.3f}m, '
            f'tolerance={self.nav2_success_pose_tolerance_m:.3f}m',
            level='warn',
            log_to_console=True,
        )

        if self.nav2_success_pose_mismatch_retry_delay_sec <= 1e-6:
            self._resend_last_nav2_goal_once()
            return True

        self._clear_nav2_pose_mismatch_retry_timer()
        self.nav2_pose_mismatch_retry_timer = self.create_timer(
            self.nav2_success_pose_mismatch_retry_delay_sec,
            self._resend_last_nav2_goal_once,
        )
        return True

    def _resend_last_nav2_goal_once(self) -> None:
        self._clear_nav2_pose_mismatch_retry_timer()
        if not self.goal_active or self._last_goal is None or self.navigator is None:
            return

        try:
            if not self.navigator.isTaskComplete():
                self.navigator.cancelTask()
        except Exception:
            pass

        try:
            self.navigator.goToPose(self._prepare_nav2_goal(self._last_goal))
            self._publish_state('NAVIGATING')
        except Exception as exc:
            self._publish_feedback(
                f'Nav2 内部重发目标失败: {exc}',
                level='error',
                log_to_console=True,
            )
            self._stop_motion('nav2_pose_mismatch_retry_failed')
            self._publish_state('FAILED')

    def _feedback_loop(self) -> None:
        if self._last_goal is None:
            return

        if self.backend == 'nav2' and self.navigator is not None:
            if self.local_spin_active:
                return
            try:
                if self.navigator.isTaskComplete():
                    if self.goal_active and not self.nav2_completion_reported:
                        result = self.navigator.getResult()
                        result_text = self._nav2_result_to_text(result)
                        if result_text == 'SUCCEEDED' and self._should_use_local_spin(self._last_goal):
                            self.nav2_completion_reported = True
                            self._activate_local_spin(
                                self._last_goal,
                                'nav2_finished_but_yaw_not_converged',
                            )
                            self._publish_state('LOCAL_SPIN')
                            return
                        if result_text == 'SUCCEEDED':
                            pose_ok, pose_distance = self._nav2_success_pose_consistent(
                                self._last_goal
                            )
                            if not pose_ok:
                                if self._schedule_nav2_pose_mismatch_retry(pose_distance):
                                    return
                                self.nav2_completion_reported = True
                                self._publish_feedback(
                                    'Nav2 返回 SUCCEEDED，但当前定位仍未接近目标，'
                                    '内部重发次数已用尽，按导航失败处理以触发 obstacle_manager 重试: '
                                    f'pose_dist={pose_distance:.3f}m, '
                                    f'tolerance={self.nav2_success_pose_tolerance_m:.3f}m, '
                                    f'pose=({self.current_x:.3f},{self.current_y:.3f}), '
                                    f'goal=({self._last_goal.pose.position.x:.3f},'
                                    f'{self._last_goal.pose.position.y:.3f})',
                                    level='error',
                                    log_to_console=True,
                                )
                                self._stop_motion('nav2_success_pose_mismatch')
                                self._publish_state('FAILED')
                                return
                        self.nav2_completion_reported = True
                        self._publish_feedback(
                            f'Nav2 任务结束: result={result_text}。'
                            ' 若此时 obstacle_manager 还未判定到点，/cmd_vel 会停止并由串口桥超时输出 0。',
                            level='warn' if result_text != 'SUCCEEDED' else 'info',
                            log_to_console=True,
                        )
                        if result_text == 'FAILED':
                            self._publish_state('FAILED')
                        elif result_text == 'CANCELED':
                            self._publish_state('CANCELED')
                        else:
                            self._publish_state('NAV2_DONE')
                    return
                feedback = self.navigator.getFeedback()
                if feedback is None:
                    return
                if not self._should_log_feedback():
                    return
                self._publish_feedback(
                    f'Nav2 反馈: remaining={feedback.distance_remaining:.3f} m, '
                    f'nav_time={feedback.navigation_time.sec} s',
                    log_to_console=True,
                )
            except Exception:
                return
            return

        if self.backend != 'pose_controller' or not self.have_pose:
            return
        if not self._should_log_feedback():
            return

        dx = self._last_goal.pose.position.x - self.current_x
        dy = self._last_goal.pose.position.y - self.current_y
        distance = math.hypot(dx, dy)
        self._publish_feedback(
            f'pose_controller: dist={distance:.3f}m, '
            f'pose=({self.current_x:.3f},{self.current_y:.3f}), active={self.goal_active}',
            log_to_console=True,
        )

    def _should_log_feedback(self) -> bool:
        if self.feedback_log_interval_sec <= 1e-6:
            return True
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if (now_sec - self.last_feedback_log_time) < self.feedback_log_interval_sec:
            return False
        self.last_feedback_log_time = now_sec
        return True

    def _should_use_local_spin(self, goal: PoseStamped) -> bool:
        if not self.nav2_local_spin_enabled or not self.have_pose:
            return False

        dx = goal.pose.position.x - self.current_x
        dy = goal.pose.position.y - self.current_y
        distance = math.hypot(dx, dy)
        if distance > self.nav2_local_spin_distance_m:
            return False

        goal_yaw = self._yaw_from_quaternion(goal.pose.orientation)
        # Old: yaw_error = abs(self._normalize_angle(goal_yaw - self.current_yaw))
        yaw_error = abs(self._normalize_angle(goal_yaw - self._controller_yaw()))
        return yaw_error >= self.nav2_local_spin_min_yaw_error_rad

    def _activate_local_spin(self, goal: PoseStamped, reason: str) -> None:
        self.local_spin_active = True
        self.zero_cmd_sent = False
        self._last_goal = goal
        goal_yaw = self._yaw_from_quaternion(goal.pose.orientation)
        controller_yaw = self._controller_yaw()
        # Old: yaw_error_deg = math.degrees(self._normalize_angle(goal_yaw - self.current_yaw))
        yaw_error_deg = math.degrees(self._normalize_angle(goal_yaw - controller_yaw))
        distance = math.hypot(
            goal.pose.position.x - self.current_x,
            goal.pose.position.y - self.current_y,
        )

        if self.navigator is not None:
            try:
                if not self.navigator.isTaskComplete():
                    self.navigator.cancelTask()
            except Exception:
                pass

        if self.nav2_local_spin_use_step_override:
            self._publish_step_override(
                self.nav2_local_spin_override_left_norm,
                self.nav2_local_spin_override_right_norm,
            )
        else:
            cmd = Twist()
            # Old: yaw_error = self._normalize_angle(goal_yaw - self.current_yaw)
            yaw_error = self._normalize_angle(goal_yaw - controller_yaw)
            cmd.angular.z = self._clamp_with_min_abs(
                self.nav2_local_spin_angular_kp * yaw_error,
                self.nav2_local_spin_max_angular_speed_rps,
                self.nav2_local_spin_min_angular_speed_rps,
            )
            if self.cmd_vel_pub is not None:
                self.cmd_vel_pub.publish(cmd)

        self._publish_feedback(
            f'切换到本地自旋控制: reason={reason}, dist={distance:.3f}m, yaw_err={yaw_error_deg:.1f}deg',
            level='warn',
            log_to_console=True,
        )

    def _publish_step_override(self, left_norm: float, right_norm: float) -> None:
        msg = Float32MultiArray()
        msg.data = [
            float(self.nav2_local_spin_override_mode),
            float(left_norm),
            float(right_norm),
        ]
        self.step_override_pub.publish(msg)

    def _clear_step_override(self) -> None:
        msg = Float32MultiArray()
        msg.data = [-1.0, 0.0, 0.0]
        self.step_override_pub.publish(msg)

    def _publish_state(self, state: str) -> None:
        if state == self.last_state:
            return
        self.last_state = state
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def _publish_feedback(
        self,
        text: str,
        level: str = 'info',
        log_to_console: bool = True,
    ) -> None:
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)

        if not log_to_console:
            return

        styled_text = colorize_log(text, level=level)
        if level == 'warn':
            self.get_logger().warn(styled_text)
        elif level == 'error':
            self.get_logger().error(styled_text)
        else:
            self.get_logger().info(styled_text)

    @staticmethod
    def _clamp_with_min_abs(value: float, max_abs: float, min_abs: float) -> float:
        if abs(value) < 1e-9:
            return 0.0

        sign = 1.0 if value >= 0.0 else -1.0
        clamped = min(abs(value), max_abs)
        if clamped < min_abs:
            clamped = min_abs
        return sign * clamped

    def _controller_yaw(self) -> float:
        return self._normalize_angle(self.current_yaw + self.pose_yaw_offset_rad)

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _quaternion_from_yaw(yaw: float):
        half_yaw = 0.5 * yaw
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(half_yaw)
        q.w = math.cos(half_yaw)
        return q

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def destroy_node(self) -> bool:
        self._clear_nav2_pose_mismatch_retry_timer()
        self._clear_step_override()
        self._publish_zero_cmd()
        if self.navigator is not None:
            try:
                self.navigator.cancelTask()
            except Exception:
                pass
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = NavExecutor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
