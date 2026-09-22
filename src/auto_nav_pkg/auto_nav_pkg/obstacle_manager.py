#!/usr/bin/env python3
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from auto_nav_pkg.log_style import colorize_log
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, Float32MultiArray, Int32, String
from tf2_msgs.msg import TFMessage
import yaml

MODE_DESCRIPTIONS = {
    0: '正常行走',
    1: '下坡',
    2: '趴下',
    3: '小跳',
    4: '大跳',
    5: '上坡',
    6: '自定义模式6',
}


class ObstacleManager(Node):
    """障碍赛上层状态机。

    当前版本直接基于 pose_topic(/Odometry 等) 做到点判定，不再依赖独立 nav_agent。
    控制时序为：
      1. 发布当前 waypoint 对应的 /goal_pose
      2. obstacle_manager 自己根据 pose_topic 判断是否到点
      3. 到点后切换到当前 waypoint 的 mode_on_arrival
      4. 该 mode 持续锁存到下一个 waypoint 到达后再切下一模式

    这样可减少节点间事件依赖，更适合当前障碍赛关键点 + 模式切换架构。
    """

    def __init__(self) -> None:
        super().__init__('obstacle_manager')

        # 文件与接口
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('mode_topic', '/dog_mode_current')
        self.declare_parameter('arrival_topic', '/agent/arrival_status')
        self.declare_parameter('state_topic', '/race_manager/state')
        self.declare_parameter('feedback_topic', '/race_manager/feedback_log')
        self.declare_parameter('current_waypoint_topic', '/race_manager/current_waypoint')
        self.declare_parameter('monitor_state_topic', '/agent/nav_state')
        self.declare_parameter('monitor_feedback_topic', '/agent/feedback_log')
        self.declare_parameter('executor_state_topic', '/nav_executor/state')
        self.declare_parameter('active', True)
        self.declare_parameter('active_topic', '/race_manager/active')
        self.declare_parameter('restart_from_waypoint_topic', '')
        self.declare_parameter('start_waypoint_index', 0)
        self.declare_parameter('stop_after_waypoint_index', -1)
        self.declare_parameter('inherit_previous_waypoint_segment_state', False)
        self.declare_parameter('inherited_segment_mode_override', -1)
        self.declare_parameter('step_override_topic', '/serial_step_override')
        self.declare_parameter('path_reference_start_topic', '/serial_path_reference_start')
        self.declare_parameter('path_reference_topic', '/serial_path_reference')
        self.declare_parameter('startup_reference_topic', '/slope_align_reference')
        self.declare_parameter('startup_reference_timeout_sec', 15.0)
        self.declare_parameter('runtime_route_yaw_reference_enabled', False)
        self.declare_parameter('route_yaw_reference_topic', '/slope_route_yaw_reference')
        self.declare_parameter('route_yaw_reference_timeout_sec', 0.0)
        self.declare_parameter(
            'bridge_retry_yaw_reference_topic',
            '/slope_bridge_retry_yaw_reference',
        )
        self.declare_parameter('bridge_retry_yaw_reference_timeout_sec', 0.0)
        self.declare_parameter('step_once_topic', '/serial_step_once')
        self.declare_parameter('uphill_step_floor_state_topic', '/race_manager/uphill_step_floor_enabled')
        self.declare_parameter('path_correction_profile_topic', '/race_manager/path_correction_profile')
        self.declare_parameter('apriltag_tf_topic', '/tf')
        self.declare_parameter('apriltag_tf_queue_depth', 10)
        self.declare_parameter('apriltag_tf_best_effort', False)
        self.declare_parameter('apriltag_base_tf_topic', '')
        self.declare_parameter('apriltag_base_tf_queue_depth', 1)
        self.declare_parameter('apriltag_base_tf_best_effort', True)
        self.declare_parameter('apriltag_base_tf_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('apriltag_base_tf_child_frame_id', 'base')

        # 位姿输入
        self.declare_parameter('pose_topic', '/Odometry')
        self.declare_parameter('pose_topic_type', 'odometry')
        self.declare_parameter('allow_frame_mismatch', False)
        self.declare_parameter('default_frame_id', 'camera_init')

        # 到点判定
        self.declare_parameter('arrival_tolerance', 0.20)
        self.declare_parameter('use_yaw_tolerance', False)
        self.declare_parameter('yaw_tolerance_deg', 20.0)
        self.declare_parameter('allow_goal_pass_through', True)
        self.declare_parameter('pass_through_lateral_tolerance', 0.25)
        self.declare_parameter('goal_timeout_sec', 45.0)
        self.declare_parameter('pose_stale_timeout_sec', 1.2)
        self.declare_parameter('monitor_rate_hz', 10.0)
        self.declare_parameter('progress_log_interval_sec', 1.0)
        self.declare_parameter('arrival_check_on_pose_update', True)
        self.declare_parameter('use_min_dist_for_yaw_waypoint', True)
        self.declare_parameter('step_override_refresh_interval_sec', 0.10)

        # 状态机
        self.declare_parameter('idle_mode', 0)
        self.declare_parameter('finish_mode', 0)
        self.declare_parameter('startup_delay_sec', 1.0)
        self.declare_parameter('settle_after_arrival_sec', 0.20)
        self.declare_parameter('retry_delay_sec', 1.0)
        self.declare_parameter('max_goal_retries', 2)
        self.declare_parameter('skip_waypoint_on_failure', False)
        self.declare_parameter('latch_mode_until_next_waypoint', True)
        self.declare_parameter('mode_transition_delay_sec', 0.15)
        self.declare_parameter('executor_failure_ignore_after_goal_send_sec', 0.25)

        # 读取参数
        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.mode_topic = self.get_parameter('mode_topic').value
        self.arrival_topic = self.get_parameter('arrival_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.current_waypoint_topic = self.get_parameter('current_waypoint_topic').value
        self.monitor_state_topic = self.get_parameter('monitor_state_topic').value
        self.monitor_feedback_topic = self.get_parameter('monitor_feedback_topic').value
        self.executor_state_topic = self.get_parameter('executor_state_topic').value
        self.manager_active = bool(self.get_parameter('active').value)
        self.active_topic = self.get_parameter('active_topic').value
        self.restart_from_waypoint_topic = str(
            self.get_parameter('restart_from_waypoint_topic').value
        )
        self.start_waypoint_index = max(0, int(self.get_parameter('start_waypoint_index').value))
        self.default_start_waypoint_index = self.start_waypoint_index
        self.stop_after_waypoint_index = int(self.get_parameter('stop_after_waypoint_index').value)
        self.inherit_previous_waypoint_segment_state = bool(
            self.get_parameter('inherit_previous_waypoint_segment_state').value
        )
        self.inherited_segment_mode_override = int(
            self.get_parameter('inherited_segment_mode_override').value
        )
        self.step_override_topic = self.get_parameter('step_override_topic').value
        self.path_reference_start_topic = self.get_parameter('path_reference_start_topic').value
        self.path_reference_topic = self.get_parameter('path_reference_topic').value
        self.startup_reference_topic = self.get_parameter('startup_reference_topic').value
        self.startup_reference_timeout_sec = max(
            0.0,
            float(self.get_parameter('startup_reference_timeout_sec').value),
        )
        self.runtime_route_yaw_reference_enabled = bool(
            self.get_parameter('runtime_route_yaw_reference_enabled').value
        )
        self.route_yaw_reference_topic = str(
            self.get_parameter('route_yaw_reference_topic').value
        )
        self.route_yaw_reference_timeout_sec = max(
            0.0,
            float(self.get_parameter('route_yaw_reference_timeout_sec').value),
        )
        self.bridge_retry_yaw_reference_topic = str(
            self.get_parameter('bridge_retry_yaw_reference_topic').value
        )
        self.bridge_retry_yaw_reference_timeout_sec = max(
            0.0,
            float(self.get_parameter('bridge_retry_yaw_reference_timeout_sec').value),
        )
        self.step_once_topic = self.get_parameter('step_once_topic').value
        self.uphill_step_floor_state_topic = self.get_parameter('uphill_step_floor_state_topic').value
        self.path_correction_profile_topic = self.get_parameter('path_correction_profile_topic').value
        self.apriltag_tf_topic = self.get_parameter('apriltag_tf_topic').value
        self.apriltag_tf_queue_depth = max(
            1,
            int(self.get_parameter('apriltag_tf_queue_depth').value),
        )
        self.apriltag_tf_best_effort = bool(
            self.get_parameter('apriltag_tf_best_effort').value
        )
        self.apriltag_base_tf_topic = str(
            self.get_parameter('apriltag_base_tf_topic').value
        ).strip()
        self.apriltag_base_tf_queue_depth = max(
            1,
            int(self.get_parameter('apriltag_base_tf_queue_depth').value),
        )
        self.apriltag_base_tf_best_effort = bool(
            self.get_parameter('apriltag_base_tf_best_effort').value
        )
        self.apriltag_base_tf_frame_id = str(
            self.get_parameter('apriltag_base_tf_frame_id').value
        )
        self.apriltag_base_tf_child_frame_id = str(
            self.get_parameter('apriltag_base_tf_child_frame_id').value
        )

        self.pose_topic = self.get_parameter('pose_topic').value
        self.pose_topic_type = str(self.get_parameter('pose_topic_type').value).strip().lower()
        self.allow_frame_mismatch = bool(self.get_parameter('allow_frame_mismatch').value)
        self.default_frame_id = self.get_parameter('default_frame_id').value

        self.default_arrival_tolerance = float(self.get_parameter('arrival_tolerance').value)
        self.default_use_yaw_tolerance = bool(self.get_parameter('use_yaw_tolerance').value)
        self.default_yaw_tolerance_deg = float(self.get_parameter('yaw_tolerance_deg').value)
        self.default_allow_goal_pass_through = bool(self.get_parameter('allow_goal_pass_through').value)
        self.default_pass_through_lateral_tolerance = float(
            self.get_parameter('pass_through_lateral_tolerance').value
        )
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.pose_stale_timeout_sec = float(self.get_parameter('pose_stale_timeout_sec').value)
        self.monitor_rate_hz = float(self.get_parameter('monitor_rate_hz').value)
        self.progress_log_interval_sec = float(self.get_parameter('progress_log_interval_sec').value)
        self.arrival_check_on_pose_update = bool(self.get_parameter('arrival_check_on_pose_update').value)
        self.use_min_dist_for_yaw_waypoint = bool(
            self.get_parameter('use_min_dist_for_yaw_waypoint').value
        )
        self.step_override_refresh_interval_sec = max(
            0.02,
            float(self.get_parameter('step_override_refresh_interval_sec').value),
        )

        self.idle_mode = int(self.get_parameter('idle_mode').value)
        self.finish_mode = int(self.get_parameter('finish_mode').value)
        self.startup_delay_sec = float(self.get_parameter('startup_delay_sec').value)
        self.settle_after_arrival_sec = float(self.get_parameter('settle_after_arrival_sec').value)
        self.retry_delay_sec = float(self.get_parameter('retry_delay_sec').value)
        self.max_goal_retries = int(self.get_parameter('max_goal_retries').value)
        self.skip_waypoint_on_failure = bool(self.get_parameter('skip_waypoint_on_failure').value)
        self.latch_mode_until_next_waypoint = bool(self.get_parameter('latch_mode_until_next_waypoint').value)
        self.mode_transition_delay_sec = float(self.get_parameter('mode_transition_delay_sec').value)
        self.executor_failure_ignore_after_goal_send_sec = max(
            0.0,
            float(self.get_parameter('executor_failure_ignore_after_goal_send_sec').value),
        )

        if not self.waypoints_file:
            raise RuntimeError('waypoints_file parameter is empty.')

        self.waypoints = self._load_waypoints(Path(self.waypoints_file))
        if not self.waypoints:
            raise RuntimeError(f'No waypoints loaded from {self.waypoints_file}')
        self.apriltag_compensation_tf_keys = {
            (
                str(
                    waypoint.get(
                        'body_forward_m_apriltag_z_compensation_frame_id',
                        waypoint.get('apriltag_arrival_frame_id', 'camera_color_optical_frame'),
                    )
                ),
                str(
                    waypoint.get(
                        'body_forward_m_apriltag_z_compensation_child_frame_id',
                        waypoint.get('apriltag_arrival_child_frame_id', 'bridge'),
                    )
                ),
            )
            for waypoint in self.waypoints
            if bool(waypoint.get('body_forward_m_apriltag_z_compensation_enabled', False))
        }
        if self.start_waypoint_index >= len(self.waypoints):
            raise RuntimeError(
                f'start_waypoint_index={self.start_waypoint_index} out of range for '
                f'{len(self.waypoints)} waypoints.'
            )
        if self.stop_after_waypoint_index >= len(self.waypoints):
            raise RuntimeError(
                f'stop_after_waypoint_index={self.stop_after_waypoint_index} out of range for '
                f'{len(self.waypoints)} waypoints.'
            )
        if (
            self.stop_after_waypoint_index >= 0
            and self.stop_after_waypoint_index < self.start_waypoint_index
        ):
            raise RuntimeError(
                'stop_after_waypoint_index must be -1 or >= start_waypoint_index.'
            )

        goal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 发布/订阅
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, goal_qos)
        self.mode_pub = self.create_publisher(Int32, self.mode_topic, 10)
        self.arrival_pub = self.create_publisher(Bool, self.arrival_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.current_waypoint_pub = self.create_publisher(String, self.current_waypoint_topic, 10)
        self.monitor_state_pub = self.create_publisher(String, self.monitor_state_topic, 10)
        self.monitor_feedback_pub = self.create_publisher(String, self.monitor_feedback_topic, 10)
        self.step_override_pub = self.create_publisher(Float32MultiArray, self.step_override_topic, 10)
        self.path_reference_start_pub = self.create_publisher(
            PoseStamped, self.path_reference_start_topic, goal_qos
        )
        self.path_reference_pub = self.create_publisher(PoseStamped, self.path_reference_topic, goal_qos)
        self.step_once_pub = self.create_publisher(Float32MultiArray, self.step_once_topic, 10)
        uphill_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.uphill_step_floor_state_pub = self.create_publisher(
            Bool,
            self.uphill_step_floor_state_topic,
            uphill_state_qos,
        )
        self.path_correction_profile_pub = self.create_publisher(
            Int32,
            self.path_correction_profile_topic,
            uphill_state_qos,
        )

        self.pose_sub = self._create_pose_subscription(self.pose_topic)
        self.executor_state_sub = self.create_subscription(
            String,
            self.executor_state_topic,
            self._executor_state_callback,
            10,
        )
        self.active_sub = self.create_subscription(
            Bool,
            self.active_topic,
            self._active_callback,
            10,
        )
        self.restart_from_waypoint_sub = None
        if self.restart_from_waypoint_topic:
            self.restart_from_waypoint_sub = self.create_subscription(
                String,
                self.restart_from_waypoint_topic,
                self._restart_from_waypoint_callback,
                10,
            )
        self.startup_reference_sub = self.create_subscription(
            PoseStamped,
            self.startup_reference_topic,
            self._startup_reference_callback,
            goal_qos,
        )
        self.route_yaw_reference_sub = self.create_subscription(
            PoseStamped,
            self.route_yaw_reference_topic,
            self._route_yaw_reference_callback,
            goal_qos,
        )
        self.bridge_retry_yaw_reference_sub = self.create_subscription(
            PoseStamped,
            self.bridge_retry_yaw_reference_topic,
            self._bridge_retry_yaw_reference_callback,
            goal_qos,
        )
        apriltag_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.apriltag_tf_queue_depth,
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
                if self.apriltag_tf_best_effort
                else ReliabilityPolicy.RELIABLE
            ),
            durability=DurabilityPolicy.VOLATILE,
        )
        self.apriltag_tf_sub = self.create_subscription(
            TFMessage,
            self.apriltag_tf_topic,
            self._apriltag_tf_callback,
            apriltag_tf_qos,
        )
        self.apriltag_base_tf_sub = None
        if self.apriltag_base_tf_topic:
            apriltag_base_tf_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=self.apriltag_base_tf_queue_depth,
                reliability=(
                    ReliabilityPolicy.BEST_EFFORT
                    if self.apriltag_base_tf_best_effort
                    else ReliabilityPolicy.RELIABLE
                ),
                durability=DurabilityPolicy.VOLATILE,
            )
            self.apriltag_base_tf_sub = self.create_subscription(
                TFMessage,
                self.apriltag_base_tf_topic,
                self._apriltag_base_tf_callback,
                apriltag_base_tf_qos,
            )

        # 运行时状态
        self.current_idx = self.start_waypoint_index
        self.current_goal_retry_count = 0
        self.goal_active = False
        self.route_finished = False
        self.route_failed = False
        self.awaiting_retry = False
        self.have_pose = False
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.pose_frame = ''
        self.last_pose_time = None
        self.last_progress_log_time = 0.0
        self.last_pose_stale_warn_time = 0.0
        self.active_goal_stamp = 0.0
        self.active_goal_msg: Optional[PoseStamped] = None
        self.active_goal_start_x = 0.0
        self.active_goal_start_y = 0.0
        self.active_goal_start_yaw = 0.0
        self.active_goal_start_captured = False
        self.active_goal_min_dist = math.inf
        self.last_state: Optional[str] = None
        self.last_monitor_state: Optional[str] = None
        self.last_feedback_line: Optional[str] = None
        self.current_mode = self.idle_mode
        self.current_uphill_step_floor_enabled = False
        self.current_path_correction_profile = 1
        self.transition_timer = None
        self.step_override_refresh_timer = None
        self.active_step_override_values: Optional[List[float]] = None
        self.active_step_override_debug_text = ''
        self.direct_step_yaw_recovery_active = False
        self.direct_step_yaw_recovery_waypoint_idx: Optional[int] = None
        self.direct_step_yaw_recovery_enter_candidate_start_sec = 0.0
        self.direct_step_yaw_recovery_exit_candidate_start_sec = 0.0
        self.direct_step_yaw_recovery_active_start_sec = 0.0
        self.direct_step_yaw_recovery_cooldown_until_sec = 0.0
        self.pending_post_arrival_mode_sequence: List[Dict[str, Any]] = []
        self.pending_post_arrival_waypoint_name: Optional[str] = None
        self.apriltag_pre_stand_backoff_active = False
        self.apriltag_pre_stand_backoff_waypoint_idx: Optional[int] = None
        self.apriltag_pre_stand_backoff_start_sec = 0.0
        self.apriltag_pre_stand_backoff_last_log_sec = 0.0
        self.pending_pose_stable_waypoint_idx: Optional[int] = None
        self.pose_stable_wait_start_time = 0.0
        self.pose_stable_anchor_time = 0.0
        self.pose_stable_anchor_x = 0.0
        self.pose_stable_anchor_y = 0.0
        self.pose_stable_anchor_yaw = 0.0
        self.last_pose_stable_log_time = 0.0
        self.pose_stable_override_active = False
        self.pose_recovery_offset_active = False
        self.pose_recovery_nominal_anchor_x = 0.0
        self.pose_recovery_nominal_anchor_y = 0.0
        self.pose_recovery_nominal_anchor_yaw = 0.0
        self.pose_recovery_raw_anchor_x = 0.0
        self.pose_recovery_raw_anchor_y = 0.0
        self.pose_recovery_raw_anchor_yaw = 0.0
        self.pose_recovery_apply_yaw_offset = False
        self.startup_deadline = self._now_sec() + self.startup_delay_sec
        self.last_executor_state: Optional[str] = None
        self.last_pose_x_switch_side: Optional[str] = None
        self.last_pose_x_switch_waypoint_name: Optional[str] = None
        self.last_pose_x_switch_waypoint_idx: Optional[int] = None
        self.startup_reference_yaw: Optional[float] = None
        self.startup_reference_frame: str = ''
        self.startup_reference_stamp_sec: float = 0.0
        self.startup_reference_align_duration_sec: float = 0.0
        self.route_yaw_reference: Optional[float] = None
        self.route_yaw_reference_frame: str = ''
        self.route_yaw_reference_stamp_sec: float = 0.0
        self.bridge_retry_yaw_reference: Optional[float] = None
        self.bridge_retry_yaw_reference_frame: str = ''
        self.bridge_retry_yaw_reference_stamp_sec: float = 0.0
        self.apriltag_z_m: Optional[float] = None
        self.apriltag_x_m: Optional[float] = None
        self.apriltag_y_m: Optional[float] = None
        self.apriltag_roll_rad: Optional[float] = None
        self.apriltag_pitch_rad: Optional[float] = None
        self.apriltag_yaw_rad: Optional[float] = None
        self.apriltag_frame_id: str = ''
        self.apriltag_child_frame_id: str = ''
        self.apriltag_stamp_sec: float = 0.0
        self.apriltag_received_sec: float = 0.0
        self.apriltag_stamp_lag_at_rx_sec: float = 0.0
        self.apriltag_compensation_tf_cache: Dict[
            Tuple[str, str],
            Tuple[float, float, float],
        ] = {}
        self.apriltag_last_seen_x_m: Optional[float] = None
        self.apriltag_last_seen_y_m: Optional[float] = None
        self.apriltag_last_seen_z_m: Optional[float] = None
        self.apriltag_last_seen_roll_rad: Optional[float] = None
        self.apriltag_last_seen_pitch_rad: Optional[float] = None
        self.apriltag_last_seen_yaw_rad: Optional[float] = None
        self.apriltag_last_seen_frame_id: str = ''
        self.apriltag_last_seen_child_frame_id: str = ''
        self.apriltag_last_seen_stamp_sec: float = 0.0
        self.apriltag_last_seen_received_sec: float = 0.0
        self.apriltag_last_seen_stamp_lag_at_rx_sec: float = 0.0
        self.apriltag_last_seen_waypoint_idx: Optional[int] = None
        self.apriltag_sample_waypoint_idx: Optional[int] = None
        self.apriltag_z_samples: List[Tuple[float, float]] = []
        self.apriltag_filtered_z_m: Optional[float] = None
        self.apriltag_pose_sample_waypoint_idx: Optional[int] = None
        self.apriltag_pose_samples: List[Dict[str, Any]] = []
        self.apriltag_filter_below_count = 0
        self.apriltag_filter_sample_count = 0
        self.apriltag_filter_window_size = 0
        self.apriltag_filter_required_count = 0
        self.apriltag_arrival_candidate_idx: Optional[int] = None
        self.apriltag_arrival_candidate_start_sec = 0.0
        self.last_apriltag_progress_log_time = 0.0
        self.last_apriltag_tf_debug_log_time = 0.0

        self.monitor_timer = self.create_timer(1.0 / max(self.monitor_rate_hz, 1.0), self._monitor_loop)

        if self.manager_active:
            self._activate_manager('startup', log_loaded=True)
        else:
            self._publish_state('INACTIVE')
            self._publish_monitor_state('INACTIVE')
            self._publish_feedback(
                f'obstacle_manager inactive on startup: '
                f'start_waypoint_index={self.start_waypoint_index}, '
                f'stop_after_waypoint_index={self.stop_after_waypoint_index}'
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
        raise RuntimeError('Unsupported pose_topic_type. Use odometry, pose_stamped or pose_with_covariance_stamped.')

    def _startup_reference_callback(self, msg: PoseStamped) -> None:
        self.startup_reference_yaw = self._yaw_from_quaternion(msg.pose.orientation)
        self.startup_reference_frame = str(msg.header.frame_id)
        self.startup_reference_stamp_sec = self._now_sec()
        duration_sec = float(msg.pose.position.z)
        self.startup_reference_align_duration_sec = (
            duration_sec if math.isfinite(duration_sec) and duration_sec > 0.0 else 0.0
        )

    def _route_yaw_reference_callback(self, msg: PoseStamped) -> None:
        self.route_yaw_reference = self._yaw_from_quaternion(msg.pose.orientation)
        self.route_yaw_reference_frame = str(msg.header.frame_id)
        self.route_yaw_reference_stamp_sec = self._now_sec()

    def _bridge_retry_yaw_reference_callback(self, msg: PoseStamped) -> None:
        self.bridge_retry_yaw_reference = self._yaw_from_quaternion(msg.pose.orientation)
        self.bridge_retry_yaw_reference_frame = str(msg.header.frame_id)
        self.bridge_retry_yaw_reference_stamp_sec = self._now_sec()

    def _apriltag_tf_callback(self, msg: TFMessage) -> None:
        self._process_apriltag_tf_message(msg)

    def _apriltag_base_tf_callback(self, msg: TFMessage) -> None:
        self._process_apriltag_tf_message(msg, base_only=True)

    def _process_apriltag_tf_message(
        self,
        msg: TFMessage,
        *,
        base_only: bool = False,
    ) -> None:
        for transform in msg.transforms:
            frame_id = str(transform.header.frame_id)
            child_frame_id = str(transform.child_frame_id)
            is_dedicated_base = (
                frame_id == self.apriltag_base_tf_frame_id
                and child_frame_id == self.apriltag_base_tf_child_frame_id
            )
            if base_only and not is_dedicated_base:
                continue
            stamp_sec = self._stamp_to_sec(transform.header.stamp)
            if (
                self.apriltag_base_tf_topic
                and is_dedicated_base
                and self.apriltag_frame_id == frame_id
                and self.apriltag_child_frame_id == child_frame_id
                and stamp_sec <= self.apriltag_stamp_sec
            ):
                continue
            self._record_apriltag_compensation_transform(transform)
            matches_active_goal = self._apriltag_transform_matches_active_goal(frame_id, child_frame_id)
            if matches_active_goal or self._should_record_apriltag_last_seen_transform(
                frame_id,
                child_frame_id,
            ):
                self._record_apriltag_last_seen_transform(transform)
            if not matches_active_goal:
                continue
            self.apriltag_x_m = float(transform.transform.translation.x)
            self.apriltag_y_m = float(transform.transform.translation.y)
            self.apriltag_z_m = float(transform.transform.translation.z)
            roll_rad, pitch_rad, yaw_rad = self._rpy_from_quaternion(transform.transform.rotation)
            self.apriltag_roll_rad = roll_rad
            self.apriltag_pitch_rad = pitch_rad
            self.apriltag_yaw_rad = yaw_rad
            self.apriltag_frame_id = frame_id
            self.apriltag_child_frame_id = child_frame_id
            now_sec = self._now_sec()
            self.apriltag_stamp_sec = stamp_sec
            self.apriltag_received_sec = now_sec
            self.apriltag_stamp_lag_at_rx_sec = now_sec - self.apriltag_stamp_sec
            self._record_apriltag_z_sample(self.apriltag_z_m, self.apriltag_stamp_sec)
            self._record_apriltag_pose_sample(
                frame_id,
                child_frame_id,
                self.apriltag_x_m,
                self.apriltag_y_m,
                self.apriltag_z_m,
                roll_rad,
                pitch_rad,
                yaw_rad,
                self.apriltag_stamp_sec,
                now_sec,
            )
            self._log_apriltag_tf_update(now_sec)
            break

    def _record_apriltag_compensation_transform(self, transform: Any) -> None:
        frame_id = str(transform.header.frame_id)
        child_frame_id = str(transform.child_frame_id)
        key = (frame_id, child_frame_id)
        if key not in getattr(self, 'apriltag_compensation_tf_keys', set()):
            return

        z_m = float(transform.transform.translation.z)
        stamp_sec = self._stamp_to_sec(transform.header.stamp)
        if not math.isfinite(z_m) or not math.isfinite(stamp_sec):
            return

        cache = self.apriltag_compensation_tf_cache
        previous = cache.get(key)
        if previous is not None and stamp_sec <= previous[1]:
            return
        cache[key] = (z_m, stamp_sec, self._now_sec())

    def _apriltag_transform_matches_active_goal(self, frame_id: str, child_frame_id: str) -> bool:
        if self.current_idx >= len(self.waypoints):
            return False
        waypoint = self.waypoints[self.current_idx]
        if not bool(waypoint.get('apriltag_arrival_enabled', False)):
            return False
        return (
            frame_id == str(waypoint.get('apriltag_arrival_frame_id', ''))
            and child_frame_id == str(waypoint.get('apriltag_arrival_child_frame_id', ''))
        )

    def _should_record_apriltag_last_seen_transform(
        self,
        frame_id: str,
        child_frame_id: str,
    ) -> bool:
        if self.current_idx >= len(self.waypoints):
            return False
        waypoint = self.waypoints[self.current_idx]
        if not bool(waypoint.get('apriltag_arrival_enabled', False)):
            return False
        expected_frame_id = str(waypoint.get('apriltag_arrival_frame_id', ''))
        expected_child_frame_id = str(waypoint.get('apriltag_arrival_child_frame_id', ''))
        return frame_id == expected_frame_id or child_frame_id == expected_child_frame_id

    def _record_apriltag_last_seen_transform(self, transform: Any) -> None:
        self.apriltag_last_seen_x_m = float(transform.transform.translation.x)
        self.apriltag_last_seen_y_m = float(transform.transform.translation.y)
        self.apriltag_last_seen_z_m = float(transform.transform.translation.z)
        roll_rad, pitch_rad, yaw_rad = self._rpy_from_quaternion(transform.transform.rotation)
        self.apriltag_last_seen_roll_rad = roll_rad
        self.apriltag_last_seen_pitch_rad = pitch_rad
        self.apriltag_last_seen_yaw_rad = yaw_rad
        self.apriltag_last_seen_frame_id = str(transform.header.frame_id)
        self.apriltag_last_seen_child_frame_id = str(transform.child_frame_id)
        now_sec = self._now_sec()
        self.apriltag_last_seen_stamp_sec = self._stamp_to_sec(transform.header.stamp)
        self.apriltag_last_seen_received_sec = now_sec
        self.apriltag_last_seen_stamp_lag_at_rx_sec = now_sec - self.apriltag_last_seen_stamp_sec
        self.apriltag_last_seen_waypoint_idx = self.current_idx

    def _record_apriltag_z_sample(self, z_m: float, stamp_sec: float) -> None:
        if not math.isfinite(float(z_m)):
            return
        if self.apriltag_sample_waypoint_idx != self.current_idx:
            self._clear_apriltag_arrival_filter()
            self.apriltag_sample_waypoint_idx = self.current_idx
        self.apriltag_z_samples.append((float(stamp_sec), float(z_m)))
        # Keep enough history for the largest practical N-of-M window without
        # carrying old waypoint samples forever.
        if len(self.apriltag_z_samples) > 30:
            self.apriltag_z_samples = self.apriltag_z_samples[-30:]
        self.apriltag_filtered_z_m = None

    def _record_apriltag_pose_sample(
        self,
        frame_id: str,
        child_frame_id: str,
        x_m: float,
        y_m: float,
        z_m: float,
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
        stamp_sec: float,
        received_sec: float,
    ) -> None:
        values = (x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad, stamp_sec, received_sec)
        if not all(math.isfinite(float(value)) for value in values):
            return
        if self.apriltag_pose_sample_waypoint_idx != self.current_idx:
            self.apriltag_pose_samples = []
            self.apriltag_pose_sample_waypoint_idx = self.current_idx
        self.apriltag_pose_samples.append({
            'frame_id': str(frame_id),
            'child_frame_id': str(child_frame_id),
            'x_m': float(x_m),
            'y_m': float(y_m),
            'z_m': float(z_m),
            'roll_rad': float(roll_rad),
            'pitch_rad': float(pitch_rad),
            'yaw_rad': float(yaw_rad),
            'stamp_sec': float(stamp_sec),
            'received_sec': float(received_sec),
        })
        if len(self.apriltag_pose_samples) > 20:
            self.apriltag_pose_samples = self.apriltag_pose_samples[-20:]

    def _log_apriltag_tf_update(self, now_sec: float) -> None:
        if (now_sec - self.last_apriltag_tf_debug_log_time) < 1.0:
            return
        self.last_apriltag_tf_debug_log_time = now_sec
        waypoint_name = 'none'
        stale_timeout_sec = 0.0
        if self.current_idx < len(self.waypoints):
            waypoint = self.waypoints[self.current_idx]
            waypoint_name = str(waypoint.get('name', 'unknown'))
            stale_timeout_sec = max(
                0.0,
                float(waypoint.get('apriltag_arrival_stale_timeout_sec', 0.30)),
            )
        level = 'warn' if (
            stale_timeout_sec > 0.0 and self.apriltag_stamp_lag_at_rx_sec > stale_timeout_sec
        ) else 'info'
        self._publish_feedback(
            f'AprilTag TF update: waypoint[{self.current_idx}]={waypoint_name}, '
            f'tf={self.apriltag_frame_id}->{self.apriltag_child_frame_id}, '
            f'xyz=({self.apriltag_x_m:.3f},{self.apriltag_y_m:.3f},{self.apriltag_z_m:.3f}), '
            f'rpy=({math.degrees(float(self.apriltag_roll_rad or 0.0)):.1f},'
            f'{math.degrees(float(self.apriltag_pitch_rad or 0.0)):.1f},'
            f'{math.degrees(float(self.apriltag_yaw_rad or 0.0)):.1f})deg, '
            f'stamp_lag_at_rx={self.apriltag_stamp_lag_at_rx_sec:.2f}s, '
            f'stale_limit={stale_timeout_sec:.2f}s',
            level=level,
        )

    def _normalize_path_correction_profile(self, value: Any) -> int:
        # 2026-05-01: profile is intentionally small and YAML-friendly:
        # 0/off disables serial-side path correction, 1/normal keeps old behavior,
        # 2/uphill_strong uses the stronger uphill lateral/yaw correction parameters,
        # 3/sign_test uses the same strong profile while validating lateral sign.
        if isinstance(value, str):
            normalized = value.strip().lower()
            aliases = {
                'off': 0,
                'disabled': 0,
                'disable': 0,
                'none': 0,
                '0': 0,
                'normal': 1,
                'default': 1,
                'on': 1,
                '1': 1,
                'uphill': 2,
                'strong': 2,
                'uphill_strong': 2,
                '2': 2,
                'mirror': 3,
                'uphill_mirror': 3,
                'mirror_uphill': 3,
                'sign_test': 3,
                'uphill_sign_test': 3,
                '3': 3,
            }
            if normalized not in aliases:
                raise RuntimeError(
                    f'Invalid path correction profile "{value}". Use off/normal/uphill_strong/sign_test or 0/1/2/3.'
                )
            return aliases[normalized]
        return max(0, min(3, int(value)))

    @staticmethod
    def _optional_float(item: Dict[str, Any], key: str) -> Optional[float]:
        value = item.get(key)
        if value is None:
            return None
        return float(value)

    @staticmethod
    def _optional_int(item: Dict[str, Any], key: str) -> Optional[int]:
        value = item.get(key)
        if value is None:
            return None
        return int(value)

    @staticmethod
    def _first_present_float(item: Dict[str, Any], keys: List[str], default: float) -> float:
        for key in keys:
            value = item.get(key)
            if value is not None:
                return float(value)
        return float(default)

    @staticmethod
    def _normalize_float_map(raw: Any) -> Dict[str, float]:
        if raw is None:
            return {}
        if not isinstance(raw, dict):
            raise RuntimeError('Expected a dict for float map configuration.')
        return {str(key).strip(): float(value) for key, value in raw.items()}

    @staticmethod
    def _normalize_pose_x_ranges(raw: Any, value_key: str) -> List[Dict[str, Any]]:
        if raw is None:
            return []
        if not isinstance(raw, list):
            raise RuntimeError(f'{value_key}_by_pose_x_ranges must be a list.')

        ranges: List[Dict[str, Any]] = []
        for idx, entry in enumerate(raw):
            if not isinstance(entry, dict):
                raise RuntimeError(f'{value_key}_by_pose_x_ranges[{idx}] must be a dict.')
            if value_key not in entry:
                raise RuntimeError(
                    f'{value_key}_by_pose_x_ranges[{idx}] is missing {value_key}.'
                )
            label = str(entry.get('label', f'range_{idx}')).strip()
            ranges.append({
                'label': label,
                'min_exclusive': ObstacleManager._optional_float(entry, 'min_exclusive'),
                'min_inclusive': ObstacleManager._optional_float(entry, 'min_inclusive'),
                'max_exclusive': ObstacleManager._optional_float(entry, 'max_exclusive'),
                'max_inclusive': ObstacleManager._optional_float(entry, 'max_inclusive'),
                value_key: float(entry[value_key]),
            })
        return ranges

    @staticmethod
    def _normalize_target_type(value: Any) -> str:
        target_type = str(value).strip().lower()
        aliases = {
            'absolute': 'absolute',
            'global': 'absolute',
            'map': 'absolute',
            'camera_init': 'absolute',
            'body': 'body_relative',
            'body_relative': 'body_relative',
            'relative': 'body_relative',
            'body_relative_delta': 'body_relative',
            'current_pose': 'body_relative',
        }
        if target_type not in aliases:
            raise RuntimeError(
                f'Invalid waypoint target_type "{value}". Use absolute or body_relative.'
            )
        return aliases[target_type]

    @staticmethod
    def _normalize_body_yaw_mode(value: Any) -> str:
        mode = str(value).strip().lower()
        aliases = {
            'delta': 'delta',
            'relative': 'delta',
            'body_relative_delta': 'delta',
            'yaw_delta': 'delta',
            'current': 'current',
            'current_yaw': 'current',
            'hold': 'current',
            'absolute': 'absolute',
            'global': 'absolute',
            'waypoint': 'absolute',
            'absolute_delta': 'absolute_delta',
            'global_delta': 'absolute_delta',
            'fixed_reference_delta': 'absolute_delta',
            'route_absolute': 'route_absolute',
            'route_absolute_delta': 'route_absolute_delta',
            'bridge_reference_delta': 'bridge_reference_delta',
            'previous': 'previous_target',
            'previous_target': 'previous_target',
            'previous_waypoint': 'previous_target',
            'last_target': 'previous_target',
            'previous_delta': 'previous_target_delta',
            'previous_target_delta': 'previous_target_delta',
            'previous_waypoint_delta': 'previous_target_delta',
            'last_target_delta': 'previous_target_delta',
            'waypoint_arrival': 'waypoint_arrival',
            'named_arrival': 'waypoint_arrival',
            'arrival_waypoint': 'waypoint_arrival',
            'waypoint_arrival_pose': 'waypoint_arrival',
            'apriltag_pose': 'apriltag_pose',
            'apriltag_pose_pitch': 'apriltag_pose',
            'tag_pose': 'apriltag_pose',
            'tag_pose_pitch': 'apriltag_pose',
            'bridge_tag_pose_pitch': 'apriltag_pose',
        }
        if mode not in aliases:
            raise RuntimeError(
                f'Invalid body_yaw_mode "{value}". Use delta, current, absolute, absolute_delta, '
                'route_absolute, route_absolute_delta, bridge_reference_delta, previous_target, '
                'previous_target_delta, waypoint_arrival, or apriltag_pose_pitch.'
            )
        return aliases[mode]

    @staticmethod
    def _normalize_body_reference_yaw_source(value: Any) -> str:
        source = str(value).strip().lower()
        aliases = {
            'current': 'current',
            'current_yaw': 'current',
            'pose': 'current',
            'odom': 'current',
            'source': 'current',
            'target': 'target_yaw',
            'target_yaw': 'target_yaw',
            'goal': 'target_yaw',
            'goal_yaw': 'target_yaw',
            'previous': 'previous_target',
            'previous_target': 'previous_target',
            'previous_waypoint': 'previous_target',
            'last_target': 'previous_target',
        }
        if source not in aliases:
            raise RuntimeError(
                f'Invalid body_reference_yaw_source "{value}". Use current, '
                'target_yaw, or previous_target.'
            )
        return aliases[source]

    @staticmethod
    def _normalize_body_position_anchor(value: Any) -> str:
        anchor = str(value).strip().lower()
        aliases = {
            'current': 'current',
            'current_pose': 'current',
            'pose': 'current',
            'odom': 'current',
            'source': 'current',
            'previous': 'previous_target',
            'previous_target': 'previous_target',
            'previous_waypoint': 'previous_target',
            'last_target': 'previous_target',
            'previous_arrival': 'previous_arrival',
            'previous_arrival_pose': 'previous_arrival',
            'last_arrival': 'previous_arrival',
            'last_arrival_pose': 'previous_arrival',
            'waypoint': 'waypoint_target',
            'waypoint_target': 'waypoint_target',
            'named': 'waypoint_target',
            'named_target': 'waypoint_target',
        }
        if anchor not in aliases:
            raise RuntimeError(
                f'Invalid body_position_anchor "{value}". Use current, previous_target, '
                'previous_arrival, or waypoint_target.'
            )
        return aliases[anchor]

    def _load_waypoints(self, yaml_path: Path) -> List[Dict[str, Any]]:
        with yaml_path.open('r', encoding='utf-8') as f:
            raw = yaml.safe_load(f) or {}

        waypoints = raw.get('waypoints', [])
        if not isinstance(waypoints, list):
            raise RuntimeError('waypoints YAML format error: top-level key "waypoints" must be a list.')

        normalized: List[Dict[str, Any]] = []
        for idx, item in enumerate(waypoints):
            if not isinstance(item, dict):
                raise RuntimeError(f'Waypoint #{idx} must be a dict.')
            if not bool(item.get('enabled', True)):
                continue
            inferred_relative = any(
                key in item
                for key in (
                    'body_forward_m',
                    'forward_m',
                    'relative_forward_m',
                    'body_left_m',
                    'left_m',
                    'relative_left_m',
                    'body_yaw_delta_deg',
                    'yaw_delta_deg',
                    'relative_yaw_delta_deg',
                )
            )
            target_type = self._normalize_target_type(
                item.get('target_type', 'body_relative' if inferred_relative else 'absolute')
            )
            if target_type == 'absolute' and ('x' not in item or 'y' not in item):
                raise RuntimeError(f'Waypoint #{idx} is missing x or y.')
            body_yaw_mode = self._normalize_body_yaw_mode(
                item.get('body_yaw_mode', item.get('yaw_mode', 'delta'))
            )
            body_reference_yaw_source = self._normalize_body_reference_yaw_source(
                item.get(
                    'body_reference_yaw_source',
                    item.get('body_offset_yaw_source', 'current'),
                )
            )
            body_position_anchor = self._normalize_body_position_anchor(
                item.get(
                    'body_position_anchor',
                    item.get('body_position_anchor_source', 'current'),
                )
            )
            raw_mode_sequence = item.get('post_arrival_mode_sequence', [])
            if raw_mode_sequence is None:
                raw_mode_sequence = []
            if not isinstance(raw_mode_sequence, list):
                raise RuntimeError(
                    f'Waypoint #{idx} post_arrival_mode_sequence must be a list.'
                )
            post_arrival_mode_sequence: List[Dict[str, Any]] = []
            for step_idx, step in enumerate(raw_mode_sequence):
                if not isinstance(step, dict):
                    raise RuntimeError(
                        f'Waypoint #{idx} post_arrival_mode_sequence[{step_idx}] must be a dict.'
                    )
                if 'mode' not in step:
                    raise RuntimeError(
                        f'Waypoint #{idx} post_arrival_mode_sequence[{step_idx}] is missing mode.'
                    )
                post_arrival_mode_sequence.append({
                    'mode': int(step['mode']),
                    'hold_sec': float(step.get('hold_sec', 0.0)),
                    'note': str(step.get('note', '')),
                    'publish_mode': bool(step.get('publish_mode', not bool(step.get('use_step_once', False)))),
                    'use_direct_step_override': bool(step.get('use_direct_step_override', False)),
                    'direct_step_override_mode': int(
                        step.get('direct_step_override_mode', step['mode'])
                    ),
                    'direct_step_override_left_norm': float(
                        step.get('direct_step_override_left_norm', 0.0)
                    ),
                    'direct_step_override_right_norm': float(
                        step.get('direct_step_override_right_norm', 0.0)
                    ),
                    'use_step_once': bool(step.get('use_step_once', False)),
                    'step_once_mode': int(
                        step.get(
                            'step_once_mode',
                            step.get('direct_step_override_mode', step['mode'])
                        )
                    ),
                    'step_once_left_norm': float(
                        step.get(
                            'step_once_left_norm',
                            step.get('direct_step_override_left_norm', 0.0)
                        )
                    ),
                    'step_once_right_norm': float(
                        step.get(
                            'step_once_right_norm',
                            step.get('direct_step_override_right_norm', 0.0)
                        )
                    ),
                })
            normalized.append({
                'name': str(item.get('name', f'wp_{idx:02d}')),
                'target_type': target_type,
                'x': float(item.get('x', 0.0)),
                'y': float(item.get('y', 0.0)),
                'yaw_deg': float(item.get('yaw_deg', 0.0)),
                'body_forward_m': self._first_present_float(
                    item,
                    ['body_forward_m', 'forward_m', 'relative_forward_m'],
                    0.0,
                ),
                'body_forward_m_bridge_retry_override_enabled': bool(
                    item.get('body_forward_m_bridge_retry_override_enabled', False)
                ),
                'body_forward_m_bridge_retry_override_m': float(
                    item.get('body_forward_m_bridge_retry_override_m', 0.0)
                ),
                'body_forward_m_by_pose_x_enabled': bool(
                    item.get('body_forward_m_by_pose_x_enabled', False)
                ),
                'body_forward_m_from_previous_pose_x_switch_enabled': bool(
                    item.get('body_forward_m_from_previous_pose_x_switch_enabled', False)
                ),
                'body_forward_m_slope_align_duration_bonus_enabled': bool(
                    item.get('body_forward_m_slope_align_duration_bonus_enabled', False)
                ),
                'body_forward_m_slope_align_duration_threshold_sec': float(
                    item.get('body_forward_m_slope_align_duration_threshold_sec', 1.5)
                ),
                'body_forward_m_slope_align_duration_bonus_m': float(
                    item.get('body_forward_m_slope_align_duration_bonus_m', 0.05)
                ),
                'body_forward_m_slope_align_duration_second_threshold_sec': float(
                    item.get(
                        'body_forward_m_slope_align_duration_second_threshold_sec',
                        0.0,
                    )
                ),
                'body_forward_m_slope_align_duration_second_bonus_m': float(
                    item.get(
                        'body_forward_m_slope_align_duration_second_bonus_m',
                        item.get('body_forward_m_slope_align_duration_bonus_m', 0.05),
                    )
                ),
                'body_forward_m_apriltag_z_compensation_enabled': bool(
                    item.get('body_forward_m_apriltag_z_compensation_enabled', False)
                ),
                'body_forward_m_apriltag_z_compensation_frame_id': str(
                    item.get(
                        'body_forward_m_apriltag_z_compensation_frame_id',
                        item.get('apriltag_arrival_frame_id', 'camera_color_optical_frame'),
                    )
                ),
                'body_forward_m_apriltag_z_compensation_child_frame_id': str(
                    item.get(
                        'body_forward_m_apriltag_z_compensation_child_frame_id',
                        item.get('apriltag_arrival_child_frame_id', 'bridge'),
                    )
                ),
                'body_forward_m_apriltag_z_compensation_stale_timeout_sec': float(
                    item.get(
                        'body_forward_m_apriltag_z_compensation_stale_timeout_sec',
                        item.get('apriltag_arrival_stale_timeout_sec', 1.20),
                    )
                ),
                'body_forward_m_apriltag_z_compensation_neutral_z_m': float(
                    item.get('body_forward_m_apriltag_z_compensation_neutral_z_m', 1.24)
                ),
                'body_forward_m_apriltag_z_compensation_near_z_m': float(
                    item.get('body_forward_m_apriltag_z_compensation_near_z_m', 1.10)
                ),
                'body_forward_m_apriltag_z_compensation_near_delta_m': float(
                    item.get('body_forward_m_apriltag_z_compensation_near_delta_m', -0.03)
                ),
                'body_forward_m_apriltag_z_compensation_far_z_m': float(
                    item.get('body_forward_m_apriltag_z_compensation_far_z_m', 1.50)
                ),
                'body_forward_m_apriltag_z_compensation_far_delta_m': float(
                    item.get('body_forward_m_apriltag_z_compensation_far_delta_m', 0.05)
                ),
                'body_forward_previous_pose_x_switch_source': str(
                    item.get('body_forward_previous_pose_x_switch_source', '')
                ),
                'body_forward_pose_x_threshold': float(
                    item.get('body_forward_pose_x_threshold', 0.0)
                ),
                'body_forward_m_when_pose_x_positive': self._optional_float(
                    item,
                    'body_forward_m_when_pose_x_positive',
                ),
                'body_forward_m_when_pose_x_negative': self._optional_float(
                    item,
                    'body_forward_m_when_pose_x_negative',
                ),
                'body_forward_m_when_previous_pose_x_right': self._optional_float(
                    item,
                    'body_forward_m_when_previous_pose_x_right',
                ),
                'body_forward_m_when_previous_pose_x_left': self._optional_float(
                    item,
                    'body_forward_m_when_previous_pose_x_left',
                ),
                'body_forward_m_when_previous_pose_x_center': self._optional_float(
                    item,
                    'body_forward_m_when_previous_pose_x_center',
                ),
                'body_forward_m_by_previous_pose_x_switch': self._normalize_float_map(
                    item.get('body_forward_m_by_previous_pose_x_switch', {})
                ),
                'body_left_m': self._first_present_float(
                    item,
                    ['body_left_m', 'left_m', 'relative_left_m'],
                    0.0,
                ),
                'body_left_m_by_pose_x_enabled': bool(
                    item.get('body_left_m_by_pose_x_enabled', False)
                ),
                'body_left_m_from_previous_pose_x_switch_enabled': bool(
                    item.get('body_left_m_from_previous_pose_x_switch_enabled', False)
                ),
                'body_left_previous_pose_x_switch_source': str(
                    item.get('body_left_previous_pose_x_switch_source', '')
                ),
                'body_left_pose_x_threshold': float(
                    item.get('body_left_pose_x_threshold', 0.0)
                ),
                'body_left_m_when_pose_x_positive': self._optional_float(
                    item,
                    'body_left_m_when_pose_x_positive',
                ),
                'body_left_m_when_pose_x_negative': self._optional_float(
                    item,
                    'body_left_m_when_pose_x_negative',
                ),
                'body_left_m_when_previous_pose_x_right': self._optional_float(
                    item,
                    'body_left_m_when_previous_pose_x_right',
                ),
                'body_left_m_when_previous_pose_x_left': self._optional_float(
                    item,
                    'body_left_m_when_previous_pose_x_left',
                ),
                'body_left_m_when_previous_pose_x_center': self._optional_float(
                    item,
                    'body_left_m_when_previous_pose_x_center',
                ),
                'body_left_m_by_previous_pose_x_switch': self._normalize_float_map(
                    item.get('body_left_m_by_previous_pose_x_switch', {})
                ),
                'body_left_m_by_pose_x_ranges': self._normalize_pose_x_ranges(
                    item.get('body_left_m_by_pose_x_ranges', []),
                    'body_left_m',
                ),
                'body_yaw_delta_deg': self._first_present_float(
                    item,
                    ['body_yaw_delta_deg', 'yaw_delta_deg', 'relative_yaw_delta_deg'],
                    0.0,
                ),
                'body_yaw_mode': body_yaw_mode,
                'route_yaw_fallback_mode': str(
                    item.get('route_yaw_fallback_mode', 'absolute')
                ).strip().lower(),
                'body_reference_yaw_source': body_reference_yaw_source,
                'body_position_anchor': body_position_anchor,
                'body_position_anchor_waypoint': str(
                    item.get('body_position_anchor_waypoint', '')
                ),
                'body_yaw_waypoint': str(
                    item.get(
                        'body_yaw_waypoint',
                        item.get(
                            'body_yaw_reference_waypoint',
                            item.get('body_yaw_anchor_waypoint', ''),
                        ),
                    )
                ),
                'body_yaw_waypoint_fallback_to_bridge_retry_reference': bool(
                    item.get(
                        'body_yaw_waypoint_fallback_to_bridge_retry_reference',
                        False,
                    )
                ),
                'body_yaw_apriltag_frame_id': str(
                    item.get(
                        'body_yaw_apriltag_frame_id',
                        item.get('apriltag_arrival_frame_id', 'camera_color_optical_frame'),
                    )
                ),
                'body_yaw_apriltag_child_frame_id': str(
                    item.get(
                        'body_yaw_apriltag_child_frame_id',
                        item.get('apriltag_arrival_child_frame_id', 'bridge'),
                    )
                ),
                'body_yaw_apriltag_metric': str(
                    item.get('body_yaw_apriltag_metric', 'pose_pitch')
                ).strip().lower(),
                'body_yaw_apriltag_target_deg': float(
                    item.get('body_yaw_apriltag_target_deg', 0.0)
                ),
                'body_yaw_apriltag_control_sign': float(
                    item.get('body_yaw_apriltag_control_sign', -1.0)
                ),
                'body_yaw_apriltag_stale_timeout_sec': float(
                    item.get(
                        'body_yaw_apriltag_stale_timeout_sec',
                        item.get('apriltag_arrival_stale_timeout_sec', 0.90),
                    )
                ),
                'body_yaw_apriltag_required_samples': int(
                    item.get('body_yaw_apriltag_required_samples', 1)
                ),
                'body_yaw_apriltag_sample_reducer': str(
                    item.get('body_yaw_apriltag_sample_reducer', 'median')
                ).strip().lower(),
                'body_yaw_apriltag_max_correction_deg': self._optional_float(
                    item,
                    'body_yaw_apriltag_max_correction_deg',
                ),
                'body_yaw_apriltag_fallback_mode': str(
                    item.get('body_yaw_apriltag_fallback_mode', 'absolute')
                ).strip().lower(),
                'body_yaw_apriltag_fallback_after_sec': float(
                    item.get('body_yaw_apriltag_fallback_after_sec', 0.0)
                ),
                'body_source_yaw_guard_enabled': bool(
                    item.get('body_source_yaw_guard_enabled', False)
                ),
                'body_source_yaw_reference_deg': self._optional_float(
                    item,
                    'body_source_yaw_reference_deg',
                ),
                'body_source_yaw_reference_mode': str(
                    item.get('body_source_yaw_reference_mode', 'absolute')
                ).strip().lower(),
                'body_source_yaw_max_error_deg': float(
                    item.get('body_source_yaw_max_error_deg', 30.0)
                ),
                'body_source_yaw_guard_hold_mode': int(
                    item.get('body_source_yaw_guard_hold_mode', 0)
                ),
                'body_source_yaw_guard_hold_left_norm': float(
                    item.get('body_source_yaw_guard_hold_left_norm', 0.0)
                ),
                'body_source_yaw_guard_hold_right_norm': float(
                    item.get('body_source_yaw_guard_hold_right_norm', 0.0)
                ),
                'frame_id': str(item.get('frame_id', self.default_frame_id)),
                'mode_on_arrival': int(item.get('mode_on_arrival', self.idle_mode)),
                'uphill_step_floor_on_arrival': bool(item.get('uphill_step_floor_on_arrival', False)),
                'path_correction_profile_on_arrival': self._normalize_path_correction_profile(
                    item.get('path_correction_profile_on_arrival', 1)
                ),
                'note': str(item.get('note', '')),
                # Backward-compatible alias:
                # `mode_hold_sec` is now treated as a post-arrival settle pause
                # before advancing to the next waypoint.
                'post_arrival_pause_sec': float(
                    item.get(
                        'post_arrival_pause_sec',
                        item.get('mode_hold_sec', self.settle_after_arrival_sec),
                    )
                ),
                'post_arrival_zero_step_hold': bool(
                    item.get('post_arrival_zero_step_hold', False)
                ),
                'post_arrival_zero_step_mode': int(
                    item.get(
                        'post_arrival_zero_step_mode',
                        item.get('mode_on_arrival', self.idle_mode),
                    )
                ),
                'distance_arrival_enabled': bool(item.get('distance_arrival_enabled', True)),
                'arrival_tolerance': float(item.get('arrival_tolerance', self.default_arrival_tolerance)),
                'use_yaw_tolerance': bool(item.get('use_yaw_tolerance', self.default_use_yaw_tolerance)),
                'yaw_tolerance_deg': float(item.get('yaw_tolerance_deg', self.default_yaw_tolerance_deg)),
                'travel_distance_arrival_enabled': bool(
                    item.get('travel_distance_arrival_enabled', False)
                ),
                'travel_distance_arrival_m': self._optional_float(
                    item,
                    'travel_distance_arrival_m',
                ),
                'travel_yaw_arrival_enabled': bool(
                    item.get('travel_yaw_arrival_enabled', False)
                ),
                'travel_yaw_arrival_deg': self._optional_float(
                    item,
                    'travel_yaw_arrival_deg',
                ),
                'travel_yaw_arrival_directional': bool(
                    item.get('travel_yaw_arrival_directional', True)
                ),
                'allow_goal_pass_through': bool(
                    item.get('allow_goal_pass_through', self.default_allow_goal_pass_through)
                ),
                'pass_through_lateral_tolerance': float(
                    item.get(
                        'pass_through_lateral_tolerance',
                        self.default_pass_through_lateral_tolerance,
                    )
                ),
                'pass_through_force_progress_ratio': self._optional_float(
                    item,
                    'pass_through_force_progress_ratio',
                ),
                'apriltag_arrival_enabled': bool(item.get('apriltag_arrival_enabled', False)),
                'apriltag_arrival_z_threshold_m': float(
                    item.get('apriltag_arrival_z_threshold_m', 0.45)
                ),
                'apriltag_arrival_frame_id': str(
                    item.get('apriltag_arrival_frame_id', 'camera_color_optical_frame')
                ),
                'apriltag_arrival_child_frame_id': str(
                    item.get('apriltag_arrival_child_frame_id', 'base')
                ),
                'apriltag_arrival_stale_timeout_sec': float(
                    item.get('apriltag_arrival_stale_timeout_sec', 0.30)
                ),
                'apriltag_arrival_hold_sec': float(item.get('apriltag_arrival_hold_sec', 0.0)),
                'apriltag_arrival_filter_window': max(
                    1,
                    int(item.get('apriltag_arrival_filter_window', 5)),
                ),
                'apriltag_arrival_required_count': max(
                    1,
                    int(item.get('apriltag_arrival_required_count', 4)),
                ),
                'apriltag_arrival_min_samples': max(
                    1,
                    int(
                        item.get(
                            'apriltag_arrival_min_samples',
                            item.get('apriltag_arrival_required_count', 4),
                        )
                    ),
                ),
                'apriltag_arrival_max_line_lateral_m': self._optional_float(
                    item,
                    'apriltag_arrival_max_line_lateral_m',
                ),
                'apriltag_arrival_max_yaw_error_deg': self._optional_float(
                    item,
                    'apriltag_arrival_max_yaw_error_deg',
                ),
                'apriltag_pre_stand_backoff_enabled': bool(
                    item.get('apriltag_pre_stand_backoff_enabled', False)
                ),
                'apriltag_pre_stand_backoff_trigger_z_m': float(
                    item.get('apriltag_pre_stand_backoff_trigger_z_m', 0.64)
                ),
                'apriltag_pre_stand_backoff_target_z_m': float(
                    item.get('apriltag_pre_stand_backoff_target_z_m', 0.67)
                ),
                'apriltag_pre_stand_backoff_stale_timeout_sec': float(
                    item.get(
                        'apriltag_pre_stand_backoff_stale_timeout_sec',
                        item.get('apriltag_arrival_stale_timeout_sec', 0.30),
                    )
                ),
                'apriltag_pre_stand_backoff_timeout_sec': float(
                    item.get('apriltag_pre_stand_backoff_timeout_sec', 1.20)
                ),
                'apriltag_pre_stand_backoff_check_period_sec': float(
                    item.get('apriltag_pre_stand_backoff_check_period_sec', 0.08)
                ),
                'apriltag_pre_stand_backoff_mode': int(
                    item.get('apriltag_pre_stand_backoff_mode', item.get('mode_on_arrival', 2))
                ),
                'apriltag_pre_stand_backoff_left_norm': float(
                    item.get('apriltag_pre_stand_backoff_left_norm', -0.20)
                ),
                'apriltag_pre_stand_backoff_right_norm': float(
                    item.get('apriltag_pre_stand_backoff_right_norm', -0.20)
                ),
                'post_arrival_mode_sequence': post_arrival_mode_sequence,
                'use_direct_step_override': bool(item.get('use_direct_step_override', False)),
                'publish_path_reference_for_direct_step_override': bool(
                    item.get('publish_path_reference_for_direct_step_override', False)
                ),
                'publish_path_reference_start_for_direct_step_override': bool(
                    item.get('publish_path_reference_start_for_direct_step_override', False)
                ),
                'path_reference_start_source': str(
                    item.get('path_reference_start_source', 'anchor')
                ).strip().lower(),
                'direct_step_override_mode': int(item.get('direct_step_override_mode', 0)),
                'direct_step_override_left_norm': float(item.get('direct_step_override_left_norm', 0.0)),
                'direct_step_override_right_norm': float(item.get('direct_step_override_right_norm', 0.0)),
                'direct_step_ramp_enabled': bool(item.get('direct_step_ramp_enabled', False)),
                'direct_step_ramp_duration_sec': float(item.get('direct_step_ramp_duration_sec', 0.0)),
                'direct_step_ramp_start_left_norm': self._optional_float(
                    item,
                    'direct_step_ramp_start_left_norm',
                ),
                'direct_step_ramp_start_right_norm': self._optional_float(
                    item,
                    'direct_step_ramp_start_right_norm',
                ),
                'direct_step_max_lr_delta_norm': self._optional_float(
                    item,
                    'direct_step_max_lr_delta_norm',
                ),
                'direct_step_yaw_correction_enabled': bool(
                    item.get('direct_step_yaw_correction_enabled', False)
                ),
                'direct_step_yaw_correction_tolerance_deg': float(
                    item.get('direct_step_yaw_correction_tolerance_deg', 3.0)
                ),
                'direct_step_yaw_correction_gain_per_rad': float(
                    item.get('direct_step_yaw_correction_gain_per_rad', 0.25)
                ),
                'direct_step_yaw_correction_max_delta_norm': float(
                    item.get('direct_step_yaw_correction_max_delta_norm', 0.15)
                ),
                'direct_step_yaw_correction_sign': float(
                    item.get('direct_step_yaw_correction_sign', 1.0)
                ),
                'direct_step_yaw_direction_override_enabled': bool(
                    item.get('direct_step_yaw_direction_override_enabled', False)
                ),
                'direct_step_yaw_direction_tolerance_deg': float(
                    item.get(
                        'direct_step_yaw_direction_tolerance_deg',
                        item.get('yaw_tolerance_deg', self.default_yaw_tolerance_deg),
                    )
                ),
                'direct_step_yaw_direction_cw_left_norm': float(
                    item.get('direct_step_yaw_direction_cw_left_norm', 0.0)
                ),
                'direct_step_yaw_direction_cw_right_norm': float(
                    item.get('direct_step_yaw_direction_cw_right_norm', 0.0)
                ),
                'direct_step_yaw_direction_ccw_left_norm': float(
                    item.get('direct_step_yaw_direction_ccw_left_norm', 0.0)
                ),
                'direct_step_yaw_direction_ccw_right_norm': float(
                    item.get('direct_step_yaw_direction_ccw_right_norm', 0.0)
                ),
                'direct_step_yaw_recovery_enabled': bool(
                    item.get('direct_step_yaw_recovery_enabled', False)
                ),
                'direct_step_yaw_recovery_enter_yaw_error_deg': float(
                    item.get('direct_step_yaw_recovery_enter_yaw_error_deg', 6.0)
                ),
                'direct_step_yaw_recovery_exit_yaw_error_deg': float(
                    item.get('direct_step_yaw_recovery_exit_yaw_error_deg', 3.0)
                ),
                'direct_step_yaw_recovery_enter_hold_sec': float(
                    item.get('direct_step_yaw_recovery_enter_hold_sec', 0.20)
                ),
                'direct_step_yaw_recovery_exit_hold_sec': float(
                    item.get('direct_step_yaw_recovery_exit_hold_sec', 0.20)
                ),
                'direct_step_yaw_recovery_min_progress_ratio': float(
                    item.get('direct_step_yaw_recovery_min_progress_ratio', 0.55)
                ),
                'direct_step_yaw_recovery_max_progress_ratio': float(
                    item.get('direct_step_yaw_recovery_max_progress_ratio', 0.90)
                ),
                'direct_step_yaw_recovery_max_line_lateral_m': self._optional_float(
                    item,
                    'direct_step_yaw_recovery_max_line_lateral_m',
                ),
                'direct_step_yaw_recovery_base_left_norm': float(
                    item.get('direct_step_yaw_recovery_base_left_norm', 0.0)
                ),
                'direct_step_yaw_recovery_base_right_norm': float(
                    item.get('direct_step_yaw_recovery_base_right_norm', 0.0)
                ),
                'direct_step_yaw_recovery_gain_per_rad': float(
                    item.get('direct_step_yaw_recovery_gain_per_rad', 3.0)
                ),
                'direct_step_yaw_recovery_max_delta_norm': float(
                    item.get('direct_step_yaw_recovery_max_delta_norm', 0.25)
                ),
                'direct_step_yaw_recovery_sign': float(
                    item.get(
                        'direct_step_yaw_recovery_sign',
                        item.get('direct_step_yaw_correction_sign', 1.0),
                    )
                ),
                'direct_step_yaw_recovery_timeout_sec': float(
                    item.get('direct_step_yaw_recovery_timeout_sec', 1.20)
                ),
                'direct_step_yaw_recovery_cooldown_sec': float(
                    item.get('direct_step_yaw_recovery_cooldown_sec', 0.35)
                ),
                'direct_step_line_lateral_correction_enabled': bool(
                    item.get('direct_step_line_lateral_correction_enabled', False)
                ),
                'direct_step_line_lateral_deadband_m': float(
                    item.get('direct_step_line_lateral_deadband_m', 0.10)
                ),
                'direct_step_line_lateral_gain_norm_per_m': float(
                    item.get('direct_step_line_lateral_gain_norm_per_m', 0.35)
                ),
                'direct_step_line_lateral_max_delta_norm': float(
                    item.get('direct_step_line_lateral_max_delta_norm', 0.04)
                ),
                'direct_step_line_lateral_sign': float(
                    item.get('direct_step_line_lateral_sign', -1.0)
                ),
                'direct_step_line_lateral_min_progress_ratio': float(
                    item.get('direct_step_line_lateral_min_progress_ratio', 0.15)
                ),
                'direct_step_line_lateral_max_progress_ratio': float(
                    item.get('direct_step_line_lateral_max_progress_ratio', 0.95)
                ),
                'direct_step_apriltag_lateral_correction_enabled': bool(
                    item.get('direct_step_apriltag_lateral_correction_enabled', False)
                ),
                'direct_step_apriltag_lateral_axis': str(
                    item.get('direct_step_apriltag_lateral_axis', 'x')
                ).strip().lower(),
                'direct_step_apriltag_lateral_target_m': float(
                    item.get('direct_step_apriltag_lateral_target_m', 0.0)
                ),
                'direct_step_apriltag_lateral_deadband_m': float(
                    item.get('direct_step_apriltag_lateral_deadband_m', 0.03)
                ),
                'direct_step_apriltag_lateral_gain_norm_per_m': float(
                    item.get('direct_step_apriltag_lateral_gain_norm_per_m', 1.0)
                ),
                'direct_step_apriltag_lateral_max_delta_norm': float(
                    item.get('direct_step_apriltag_lateral_max_delta_norm', 0.12)
                ),
                'direct_step_apriltag_lateral_sign': float(
                    item.get('direct_step_apriltag_lateral_sign', -1.0)
                ),
                'direct_step_apriltag_lateral_stale_timeout_sec': float(
                    item.get(
                        'direct_step_apriltag_lateral_stale_timeout_sec',
                        item.get('apriltag_arrival_stale_timeout_sec', 0.30),
                    )
                ),
                'smooth_transition': bool(item.get('smooth_transition', True)),
                # 2026-05-01: local localization guard, enabled only on selected critical waypoints.
                # It holds before advancing to the next waypoint until pose/cloud recovery is stable.
                'require_pose_stable_before_next': bool(item.get('require_pose_stable_before_next', False)),
                'pose_stable_expected_pose_source': str(
                    item.get('pose_stable_expected_pose_source', 'next')
                ).strip().lower(),
                'pose_stable_expected_x': self._optional_float(item, 'pose_stable_expected_x'),
                'pose_stable_expected_y': self._optional_float(item, 'pose_stable_expected_y'),
                'pose_stable_expected_yaw_deg': self._optional_float(item, 'pose_stable_expected_yaw_deg'),
                'pose_stable_duration_sec': float(item.get('pose_stable_duration_sec', 0.80)),
                'pose_stable_sample_tolerance_m': float(item.get('pose_stable_sample_tolerance_m', 0.08)),
                'pose_stable_sample_yaw_tolerance_deg': float(
                    item.get('pose_stable_sample_yaw_tolerance_deg', 5.0)
                ),
                'pose_stable_max_position_error_m': float(
                    item.get('pose_stable_max_position_error_m', 0.35)
                ),
                'pose_stable_max_yaw_error_deg': self._optional_float(
                    item,
                    'pose_stable_max_yaw_error_deg',
                ),
                'pose_stable_min_x': self._optional_float(item, 'pose_stable_min_x'),
                'pose_stable_max_x': self._optional_float(item, 'pose_stable_max_x'),
                'pose_stable_min_y': self._optional_float(item, 'pose_stable_min_y'),
                'pose_stable_max_y': self._optional_float(item, 'pose_stable_max_y'),
                'pose_stable_timeout_sec': float(item.get('pose_stable_timeout_sec', 6.0)),
                'pose_stable_timeout_action': str(
                    item.get('pose_stable_timeout_action', 'hold')
                ).strip().lower(),
                'pose_stable_hold_mode': self._optional_int(item, 'pose_stable_hold_mode'),
                'pose_stable_use_direct_step_override': bool(
                    item.get('pose_stable_use_direct_step_override', False)
                ),
                'pose_stable_direct_step_override_mode': self._optional_int(
                    item,
                    'pose_stable_direct_step_override_mode',
                ),
                'pose_stable_direct_step_override_left_norm': float(
                    item.get('pose_stable_direct_step_override_left_norm', 0.0)
                ),
                'pose_stable_direct_step_override_right_norm': float(
                    item.get('pose_stable_direct_step_override_right_norm', 0.0)
                ),
                # 2026-05-01: local pose recovery for jump/spin-only drift.
                # Old behavior only held on localization drift. The new optional modes can
                # remap future waypoint targets into the drifted Point-LIO frame.
                'pose_recovery_mode': str(item.get('pose_recovery_mode', 'none')).strip().lower(),
                'pose_recovery_max_offset_m': float(item.get('pose_recovery_max_offset_m', 0.0)),
                'pose_recovery_apply_yaw_offset': bool(item.get('pose_recovery_apply_yaw_offset', False)),
                'pose_recovery_clear_offset_on_arrival': bool(
                    item.get('pose_recovery_clear_offset_on_arrival', False)
                ),
            })
        return normalized

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

    def _update_pose(self, x: float, y: float, yaw: float, frame_id: str) -> None:
        self.pose_x = x
        self.pose_y = y
        self.pose_yaw = yaw
        self.pose_frame = frame_id
        self.have_pose = True
        self.last_pose_time = self.get_clock().now()
        if self.arrival_check_on_pose_update:
            self._check_arrival_from_pose_update()

    def _executor_state_callback(self, msg: String) -> None:
        if not self.manager_active:
            return
        state = str(msg.data).strip().upper()
        if not state or state == self.last_executor_state:
            return

        self.last_executor_state = state

        if state not in ('FAILED', 'CANCELED'):
            return
        if not self.goal_active or self.route_finished or self.route_failed:
            return
        if self.transition_timer is not None:
            return
        goal_age_sec = self._now_sec() - self.active_goal_stamp
        if (
            self.executor_failure_ignore_after_goal_send_sec > 0.0
            and self.active_goal_stamp > 0.0
            and goal_age_sec < self.executor_failure_ignore_after_goal_send_sec
        ):
            self._publish_feedback(
                f'忽略 nav_executor 过期状态={state}: 当前 waypoint 刚发送 '
                f'{goal_age_sec:.2f}s，短于保护窗口 '
                f'{self.executor_failure_ignore_after_goal_send_sec:.2f}s。',
            )
            return

        self._publish_feedback(
            f'收到 nav_executor 状态={state}，当前 waypoint 按失败处理并进入重试流程。',
            level='warn',
        )
        self._handle_goal_failure(f'nav_executor_{state.lower()}')

    def _monitor_loop(self) -> None:
        if not self.manager_active:
            return
        now = self._now_sec()

        if self.route_finished or self.route_failed:
            return

        if self.transition_timer is not None:
            return

        if now < self.startup_deadline:
            return

        if not self.goal_active:
            self._send_current_goal()
            return

        if not self.have_pose:
            self._publish_state('WAITING_POSE')
            self._publish_monitor_state('WAITING_POSE')
            return

        if self._pose_is_stale(now):
            self._publish_state('POSE_STALE')
            self._publish_monitor_state('POSE_STALE')
            if self.goal_timeout_sec > 0.0 and (now - self.active_goal_stamp) > self.goal_timeout_sec:
                self._handle_goal_failure('定位数据超时')
            return

        if self.active_goal_msg is None:
            return

        goal = self.waypoints[self.current_idx]
        goal_frame = self._target_frame_for_waypoint(goal)
        if not self.allow_frame_mismatch and self.pose_frame and goal_frame and self.pose_frame != goal_frame:
            text = (
                f'frame 不一致: pose_frame={self.pose_frame}, goal_frame={goal_frame}，'
                '当前不判定到点。'
            )
            self._publish_state('FRAME_MISMATCH')
            self._publish_monitor_state('FRAME_MISMATCH')
            self._publish_progress(text, force_console_every=None)
            if self.goal_timeout_sec > 0.0 and (now - self.active_goal_stamp) > self.goal_timeout_sec:
                self._handle_goal_failure('坐标系不一致')
            return

        self._refresh_direct_step_override(goal)

        if self._evaluate_goal_arrival(goal, publish_progress=True):
            return

        self._publish_monitor_state('TRACKING')
        self._publish_state('NAVIGATING')

        if self.goal_timeout_sec > 0.0 and (now - self.active_goal_stamp) > self.goal_timeout_sec:
            self._handle_goal_failure('导航超时')

    def _check_arrival_from_pose_update(self) -> None:
        if not self.manager_active:
            return
        if self.route_finished or self.route_failed:
            return
        if self.transition_timer is not None:
            return
        if not self.goal_active or self.active_goal_msg is None:
            return
        if not self.have_pose:
            return

        now = self._now_sec()
        if now < self.startup_deadline:
            return

        goal = self.waypoints[self.current_idx]
        goal_frame = self._target_frame_for_waypoint(goal)
        if not self.allow_frame_mismatch and self.pose_frame and goal_frame and self.pose_frame != goal_frame:
            return

        self._evaluate_goal_arrival(goal, publish_progress=False)

    def _evaluate_goal_arrival(self, goal: Dict[str, Any], publish_progress: bool) -> bool:
        self._capture_goal_start_pose()

        goal_x, goal_y, goal_yaw = self._target_pose_for_waypoint(goal)
        dist = math.hypot(goal_x - self.pose_x, goal_y - self.pose_y)
        self.active_goal_min_dist = min(self.active_goal_min_dist, dist)
        yaw_err = math.degrees(abs(self._normalize_angle(goal_yaw - self.pose_yaw)))
        yaw_ok = True
        if goal['use_yaw_tolerance']:
            yaw_ok = yaw_err <= float(goal['yaw_tolerance_deg'])

        if publish_progress:
            apriltag_text = self._apriltag_progress_text(goal)
            step_override_text = self._active_step_override_progress_text()
            # 2026-05-22
            # Purpose: diagnose direct-step segments where yaw correction spends
            # step length without producing enough along-track progress.
            # Behavior: progress logging only; arrival decisions still use the
            # existing distance/pass-through/AprilTag checks.
            # Rollback: remove this text append to restore the old progress log.
            line_progress_text = self._goal_line_progress_text(goal)
            travel_progress_text = self._travel_arrival_progress_text(goal)
            self._publish_progress(
                f'waypoint={self.current_idx + 1}/{len(self.waypoints)} {goal["name"]} '
                f'dist={dist:.3f}m, min_dist={self.active_goal_min_dist:.3f}m, yaw_err={yaw_err:.1f}deg, '
                f'pose=({self.pose_x:.3f},{self.pose_y:.3f}), '
                f'goal=({goal_x:.3f},{goal_y:.3f}), '
                f'pose_frame={self.pose_frame}, goal_frame={self._target_frame_for_waypoint(goal)}'
                f'{line_progress_text}'
                f'{travel_progress_text}'
                f'{apriltag_text}'
                f'{step_override_text}'
            )

        arrival_tolerance = float(goal['arrival_tolerance'])
        if self._apriltag_arrival_detected(goal):
            self._handle_goal_arrived(dist, yaw_err, self._apriltag_arrival_reason(goal))
            return True

        travel_arrival_reason = self._travel_arrival_reason(goal)
        if travel_arrival_reason is not None:
            self._handle_goal_arrived(dist, yaw_err, travel_arrival_reason)
            return True

        if bool(goal.get('distance_arrival_enabled', True)) and dist <= arrival_tolerance and yaw_ok:
            self._handle_goal_arrived(dist, yaw_err, 'distance_tolerance')
            return True

        pass_through_reason = self._goal_pass_through_reason(goal)
        if pass_through_reason is not None and yaw_ok:
            self._handle_goal_arrived(dist, yaw_err, pass_through_reason)
            return True

        if goal['use_yaw_tolerance'] and self.use_min_dist_for_yaw_waypoint:
            if self.active_goal_min_dist <= arrival_tolerance and yaw_ok:
                self._handle_goal_arrived(dist, yaw_err, 'distance_memory_with_yaw')
                return True

        return False

    def _apriltag_arrival_detected(self, waypoint: Dict[str, Any]) -> bool:
        arrived, _ = self._apriltag_arrival_status(waypoint, update_candidate=True)
        return arrived

    def _apriltag_filter_state(
        self,
        waypoint: Dict[str, Any],
    ) -> Optional[Tuple[float, int, int, int, int, int]]:
        if self.apriltag_sample_waypoint_idx != self.current_idx:
            return None
        now = self._now_sec()
        stale_timeout_sec = max(0.0, float(waypoint.get('apriltag_arrival_stale_timeout_sec', 0.30)))
        kept_samples: List[Tuple[float, float]] = []
        for stamp_sec, z_m in self.apriltag_z_samples:
            if not math.isfinite(float(z_m)):
                continue
            if stale_timeout_sec > 0.0 and (now - float(stamp_sec)) > stale_timeout_sec:
                continue
            kept_samples.append((float(stamp_sec), float(z_m)))
        if len(kept_samples) != len(self.apriltag_z_samples):
            self.apriltag_z_samples = kept_samples

        window_size = max(1, int(waypoint.get('apriltag_arrival_filter_window', 5)))
        required_count = max(1, int(waypoint.get('apriltag_arrival_required_count', 4)))
        required_count = min(required_count, window_size)
        min_samples = max(1, int(waypoint.get('apriltag_arrival_min_samples', required_count)))
        min_samples = min(max(min_samples, required_count), window_size)

        window_samples = kept_samples[-window_size:]
        sample_count = len(window_samples)
        self.apriltag_filter_sample_count = sample_count
        self.apriltag_filter_window_size = window_size
        self.apriltag_filter_required_count = required_count
        if sample_count <= 0:
            self.apriltag_filtered_z_m = None
            self.apriltag_filter_below_count = 0
            return None

        values = [z_m for _, z_m in window_samples]
        threshold_m = float(waypoint.get('apriltag_arrival_z_threshold_m', 0.45))
        filtered_z = self._median(values)
        below_count = sum(1 for value in values if value <= threshold_m)
        self.apriltag_filtered_z_m = filtered_z
        self.apriltag_filter_below_count = below_count

        return filtered_z, below_count, sample_count, window_size, required_count, min_samples

    @staticmethod
    def _median(values: List[float]) -> float:
        ordered = sorted(float(value) for value in values)
        count = len(ordered)
        if count <= 0:
            return math.nan
        mid = count // 2
        if count % 2:
            return ordered[mid]
        return 0.5 * (ordered[mid - 1] + ordered[mid])

    def _fresh_apriltag_z_for_waypoint(
        self,
        waypoint: Dict[str, Any],
        stale_timeout_sec: float,
    ) -> Optional[float]:
        if self.apriltag_z_m is None or not math.isfinite(float(self.apriltag_z_m)):
            return None
        if not self._apriltag_latest_transform_matches_waypoint(waypoint):
            return None
        now_sec = self._now_sec()
        if stale_timeout_sec > 0.0 and (now_sec - self.apriltag_stamp_sec) > stale_timeout_sec:
            return None
        return float(self.apriltag_z_m)

    def _apriltag_latest_transform_matches_waypoint(self, waypoint: Dict[str, Any]) -> bool:
        return (
            self.apriltag_frame_id == str(waypoint.get('apriltag_arrival_frame_id', ''))
            and self.apriltag_child_frame_id == str(waypoint.get('apriltag_arrival_child_frame_id', ''))
        )

    def _apriltag_arrival_reason(self, waypoint: Dict[str, Any]) -> str:
        now_sec = self._now_sec()
        z_text = 'unknown' if self.apriltag_z_m is None else f'{self.apriltag_z_m:.3f}'
        filtered_text = (
            'unknown'
            if self.apriltag_filtered_z_m is None
            else f'{self.apriltag_filtered_z_m:.3f}'
        )
        stamp_age_text = (
            'unknown'
            if self.apriltag_stamp_sec <= 0.0
            else f'{now_sec - self.apriltag_stamp_sec:.2f}s'
        )
        rx_age_text = (
            'unknown'
            if self.apriltag_received_sec <= 0.0
            else f'{now_sec - self.apriltag_received_sec:.2f}s'
        )
        hold_sec = float(waypoint.get('apriltag_arrival_hold_sec', 0.0))
        return (
            'apriltag_z_threshold'
            f'(raw_z={z_text}m, filtered_z={filtered_text}m'
            f'<={float(waypoint.get("apriltag_arrival_z_threshold_m", 0.45)):.3f}m, '
            f'below={self.apriltag_filter_below_count}/{self.apriltag_filter_sample_count}'
            f' required={self.apriltag_filter_required_count}, hold={hold_sec:.2f}s, '
            f'stamp_age={stamp_age_text}, rx_age={rx_age_text}, '
            f'stamp_lag_at_rx={self.apriltag_stamp_lag_at_rx_sec:.2f}s, '
            f'tf={self.apriltag_frame_id}->{self.apriltag_child_frame_id})'
        )

    def _apriltag_arrival_status(
        self,
        waypoint: Dict[str, Any],
        update_candidate: bool,
    ) -> Tuple[bool, str]:
        def blocked(reason: str) -> Tuple[bool, str]:
            if update_candidate:
                self._clear_apriltag_arrival_candidate()
            return False, f'blocked:{reason}'

        if not bool(waypoint.get('apriltag_arrival_enabled', False)):
            if update_candidate:
                self._clear_apriltag_arrival_candidate()
            return False, 'disabled'

        expected_frame_id = str(waypoint.get('apriltag_arrival_frame_id', ''))
        expected_child_frame_id = str(waypoint.get('apriltag_arrival_child_frame_id', ''))
        expected_tf_text = f'{expected_frame_id}->{expected_child_frame_id}'
        threshold_m = float(waypoint.get('apriltag_arrival_z_threshold_m', 0.45))
        raw_z_text = 'none' if self.apriltag_z_m is None else f'{self.apriltag_z_m:.3f}m'

        last_seen_text = 'last_seen=none'
        if self.apriltag_last_seen_waypoint_idx == self.current_idx:
            last_seen_age_text = (
                'unknown'
                if self.apriltag_last_seen_stamp_sec <= 0.0
                else f'{self._now_sec() - self.apriltag_last_seen_stamp_sec:.2f}s'
            )
            last_seen_xyz_text = (
                'xyz=unknown'
                if (
                    self.apriltag_last_seen_x_m is None
                    or self.apriltag_last_seen_y_m is None
                    or self.apriltag_last_seen_z_m is None
                )
                else (
                    f'xyz=({self.apriltag_last_seen_x_m:.3f},'
                    f'{self.apriltag_last_seen_y_m:.3f},{self.apriltag_last_seen_z_m:.3f})'
                )
            )
            last_seen_text = (
                f'last_seen_tf={self.apriltag_last_seen_frame_id}'
                f'->{self.apriltag_last_seen_child_frame_id}, '
                f'{last_seen_xyz_text}, '
                f'stamp_age={last_seen_age_text}, '
                f'stamp_lag_at_rx={self.apriltag_last_seen_stamp_lag_at_rx_sec:.2f}s'
            )

        if self.apriltag_z_m is None or not math.isfinite(float(self.apriltag_z_m)):
            return blocked(f'missing_matching_tf(expected_tf={expected_tf_text}, {last_seen_text})')

        if not self._apriltag_latest_transform_matches_waypoint(waypoint):
            return blocked(
                f'tf_mismatch(expected_tf={expected_tf_text}, '
                f'cached_tf={self.apriltag_frame_id}->{self.apriltag_child_frame_id}, '
                f'{last_seen_text})'
            )

        now_sec = self._now_sec()
        stamp_age_sec = now_sec - self.apriltag_stamp_sec
        stale_timeout_sec = max(0.0, float(waypoint.get('apriltag_arrival_stale_timeout_sec', 0.30)))
        filtered_text = (
            'none'
            if self.apriltag_filtered_z_m is None
            else f'{self.apriltag_filtered_z_m:.3f}m'
        )
        common_debug = (
            f'raw_z={raw_z_text}, filtered_z={filtered_text}, '
            f'threshold={threshold_m:.3f}m, tf={self.apriltag_frame_id}->{self.apriltag_child_frame_id}, '
            f'stamp_age={stamp_age_sec:.2f}s, stamp_lag_at_rx={self.apriltag_stamp_lag_at_rx_sec:.2f}s'
        )
        if stale_timeout_sec > 0.0 and stamp_age_sec > stale_timeout_sec:
            return blocked(
                f'stale_transform(age={stamp_age_sec:.2f}s>{stale_timeout_sec:.2f}s, {common_debug})'
            )

        filter_state = self._apriltag_filter_state(waypoint)
        if filter_state is None:
            return blocked(f'no_valid_samples(expected_tf={expected_tf_text}, {common_debug})')
        filtered_z, below_count, sample_count, window_size, required_count, min_samples = filter_state
        common_debug = (
            f'raw_z={raw_z_text}, filtered_z={filtered_z:.3f}m, '
            f'threshold={threshold_m:.3f}m, below={below_count}/{sample_count}, '
            f'required={required_count}/{window_size}, tf={self.apriltag_frame_id}'
            f'->{self.apriltag_child_frame_id}, stamp_age={stamp_age_sec:.2f}s, '
            f'stamp_lag_at_rx={self.apriltag_stamp_lag_at_rx_sec:.2f}s'
        )
        if sample_count < min_samples:
            return blocked(f'waiting_samples(samples={sample_count}/{min_samples}, {common_debug})')

        z_reasons: List[str] = []
        if filtered_z > threshold_m:
            z_reasons.append(f'filtered_z={filtered_z:.3f}m>{threshold_m:.3f}m')
        if below_count < required_count:
            z_reasons.append(f'below={below_count}/{sample_count}<{required_count}')
        if z_reasons:
            return blocked(f'z_gate({", ".join(z_reasons)}, {common_debug})')

        max_lateral_m = waypoint.get('apriltag_arrival_max_line_lateral_m')
        if max_lateral_m is not None:
            metrics = self._goal_line_metrics(waypoint)
            if metrics is None:
                return blocked(
                    f'line_metrics_unavailable(limit={float(max_lateral_m):.3f}m, {common_debug})'
                )
            _, _, lateral_error_m, _, _ = metrics
            if abs(lateral_error_m) > max(0.0, float(max_lateral_m)):
                return blocked(
                    f'line_lateral(abs={abs(lateral_error_m):.3f}m>'
                    f'{float(max_lateral_m):.3f}m, line_lateral={lateral_error_m:.3f}m, '
                    f'{common_debug})'
                )

        max_yaw_error_deg = waypoint.get('apriltag_arrival_max_yaw_error_deg')
        if max_yaw_error_deg is not None:
            target = waypoint.get('_resolved_target')
            if target is None or not self.have_pose:
                return blocked(
                    f'yaw_gate_pose_missing(limit={float(max_yaw_error_deg):.1f}deg, {common_debug})'
                )
            yaw_error_deg = abs(math.degrees(self._normalize_angle(float(target[2]) - self.pose_yaw)))
            if yaw_error_deg > max(0.0, float(max_yaw_error_deg)):
                return blocked(
                    f'yaw_error({yaw_error_deg:.1f}deg>{float(max_yaw_error_deg):.1f}deg, '
                    f'{common_debug})'
                )

        hold_sec = max(0.0, float(waypoint.get('apriltag_arrival_hold_sec', 0.0)))
        if hold_sec <= 0.0:
            return True, f'ready({common_debug})'

        if self.apriltag_arrival_candidate_idx != self.current_idx:
            if update_candidate:
                self.apriltag_arrival_candidate_idx = self.current_idx
                self.apriltag_arrival_candidate_start_sec = now_sec
            return blocked(f'hold(0.00s/{hold_sec:.2f}s, {common_debug})')

        held_sec = max(0.0, now_sec - self.apriltag_arrival_candidate_start_sec)
        if held_sec < hold_sec:
            return blocked(f'hold({held_sec:.2f}s/{hold_sec:.2f}s, {common_debug})')
        return True, f'ready(hold={held_sec:.2f}s/{hold_sec:.2f}s, {common_debug})'

    def _apriltag_progress_text(self, waypoint: Dict[str, Any]) -> str:
        if not bool(waypoint.get('apriltag_arrival_enabled', False)):
            return ''
        _, status_text = self._apriltag_arrival_status(waypoint, update_candidate=False)
        return f', apriltag_gate={status_text}'

    def _handle_goal_arrived(self, dist: float, yaw_err: float, reason: str) -> None:
        waypoint = self.waypoints[self.current_idx]
        pause_sec = max(0.0, float(waypoint.get('post_arrival_pause_sec', self.settle_after_arrival_sec)))
        zero_step_hold_active = (
            pause_sec > 1e-6 and bool(waypoint.get('post_arrival_zero_step_hold', False))
        )
        zero_step_hold_mode = int(
            waypoint.get('post_arrival_zero_step_mode', waypoint['mode_on_arrival'])
        )
        self.goal_active = False
        if zero_step_hold_active:
            self.active_step_override_debug_text = ''
            self._publish_step_override(zero_step_hold_mode, 0.0, 0.0)
        else:
            self._clear_step_override()
        self._clear_direct_step_yaw_recovery_state()
        self._clear_apriltag_pre_stand_backoff_state()
        self._clear_apriltag_arrival_candidate()
        self._clear_apriltag_arrival_filter()
        if self.have_pose:
            waypoint['_arrival_pose'] = (self.pose_x, self.pose_y, self.pose_yaw)
        self._publish_state('ARRIVED')
        self._publish_monitor_state('ARRIVED')
        self._publish_feedback(
            f'到点成功: waypoint[{self.current_idx}]={waypoint["name"]}, '
            f'dist={dist:.3f}, min_dist={self.active_goal_min_dist:.3f}, '
            f'yaw_err={yaw_err:.1f}deg, reason={reason}'
        )
        if bool(waypoint.get('pose_recovery_clear_offset_on_arrival', False)):
            self._clear_pose_recovery_offset('waypoint 配置到点清除')
        self._apply_mode_after_arrival(waypoint)
        if self._should_use_smooth_transition(waypoint):
            self._publish_feedback(
                f'waypoint[{self.current_idx}]={waypoint["name"]} 使用平滑切点：'
                '不主动发 arrival/0,0,0，直接推进到下一个目标。'
            )
            self._advance_once()
            return
        self._publish_arrival(True)
        if self._start_post_arrival_mode_sequence_if_needed(waypoint):
            return
        if pause_sec > 1e-6:
            if zero_step_hold_active:
                self._publish_feedback(
                    f'waypoint[{self.current_idx}]={waypoint["name"]} 到点后零步长保持: '
                    f'override=[{zero_step_hold_mode},0.000,0.000]，持续 {pause_sec:.2f}s。'
                )
            self._publish_feedback(
                f'waypoint[{self.current_idx}]={waypoint["name"]} 到点后等待 {pause_sec:.2f}s，再切换到下一个目标。'
            )
        self._schedule_transition(pause_sec, self._advance_after_optional_pose_stability)

    def _apply_mode_after_arrival(self, waypoint: Dict[str, Any]) -> None:
        mode = int(waypoint['mode_on_arrival'])
        description = MODE_DESCRIPTIONS.get(mode, '未知模式')
        uphill_floor_enabled = bool(waypoint.get('uphill_step_floor_on_arrival', False))
        path_correction_profile = int(waypoint.get('path_correction_profile_on_arrival', 1))
        # 2026-05-01: mirror mode_on_arrival semantics for the uphill speed floor.
        # This lets YAML define exact waypoint intervals where roll-based filtering is allowed.
        if self.latch_mode_until_next_waypoint:
            self._publish_mode(mode)
            self._publish_uphill_step_floor_state(uphill_floor_enabled)
            self._publish_path_correction_profile(path_correction_profile)
            self._publish_feedback(
                f'已到达 waypoint[{self.current_idx}]={waypoint["name"]}，切换模式为 {mode} ({description})，'
                f'上坡滤波={self.current_uphill_step_floor_enabled}，'
                f'路径修正profile={self.current_path_correction_profile}，并锁存到下一个 waypoint。'
            )
        else:
            self._publish_mode(mode)
            self._publish_uphill_step_floor_state(uphill_floor_enabled)
            self._publish_path_correction_profile(path_correction_profile)
            self._publish_feedback(
                f'已到达 waypoint[{self.current_idx}]={waypoint["name"]}，切换模式为 {mode} ({description})，'
                f'上坡滤波={self.current_uphill_step_floor_enabled}，'
                f'路径修正profile={self.current_path_correction_profile}。'
            )

    def _handle_goal_failure(self, reason: str) -> None:
        waypoint = self.waypoints[self.current_idx]
        self.goal_active = False
        self._clear_step_override()
        self._clear_post_arrival_mode_sequence()
        self._clear_pose_stability_state()
        self._clear_direct_step_yaw_recovery_state()
        self._clear_apriltag_pre_stand_backoff_state()
        self._clear_apriltag_arrival_candidate()
        self._clear_apriltag_arrival_filter()
        self._publish_arrival(False)
        self._publish_state('FAILED')
        self._publish_monitor_state('FAILED')

        if self.max_goal_retries < 0 or self.current_goal_retry_count < self.max_goal_retries:
            self.current_goal_retry_count += 1
            self._publish_feedback(
                f'导航失败或超时: waypoint[{self.current_idx}]={waypoint["name"]}, '
                f'reason={reason}，{self.retry_delay_sec:.2f}s 后进行第 {self.current_goal_retry_count + 1} 次尝试。',
                level='warn',
            )
            self._schedule_transition(self.retry_delay_sec, self._retry_current_goal_once)
            return

        if self.skip_waypoint_on_failure:
            self._publish_feedback(
                f'waypoint[{self.current_idx}]={waypoint["name"]} 达到最大重试次数，按配置跳过。',
                level='warn',
            )
            self._schedule_transition(self.mode_transition_delay_sec, self._advance_once)
            return

        self.route_failed = True
        self._publish_feedback(
            f'waypoint[{self.current_idx}]={waypoint["name"]} 重试耗尽，流程停在当前点。',
            level='error',
        )

    def _send_current_goal(self) -> None:
        if self._route_end_reached():
            self._finish_route()
            return

        self._clear_post_arrival_mode_sequence()
        self._clear_pose_stability_state()
        self._clear_step_override()
        self._clear_direct_step_yaw_recovery_state()
        self._clear_apriltag_arrival_candidate()
        self._clear_apriltag_arrival_filter()
        waypoint = self.waypoints[self.current_idx]
        if not self._resolve_body_relative_waypoint_target(
            waypoint,
            force_new=(self.current_goal_retry_count <= 0),
        ):
            return
        goal = self._build_goal_msg(waypoint)
        target_x, target_y, target_yaw = self._target_pose_for_waypoint(waypoint)
        target_yaw_deg = math.degrees(target_yaw)
        target_text = f'x={target_x:.3f}, y={target_y:.3f}, yaw={target_yaw_deg:.1f}°'
        if self._waypoint_uses_body_relative_target(waypoint):
            source = waypoint.get('_resolved_source_pose')
            source_text = ''
            if source is not None:
                source_text = (
                    f', source=({float(source[0]):.3f},{float(source[1]):.3f},'
                    f'{math.degrees(float(source[2])):.1f}°)'
                )
            anchor = waypoint.get('_resolved_body_position_anchor')
            anchor_text = ''
            if anchor is not None:
                anchor_text = (
                    f', anchor=({float(anchor[0]):.3f},{float(anchor[1]):.3f},'
                    f'{str(anchor[2])})'
                )
            reference_yaw = waypoint.get('_resolved_body_reference_yaw')
            reference_text = ''
            if reference_yaw is not None:
                reference_text = (
                    f', ref_yaw_source={waypoint.get("body_reference_yaw_source", "current")}, '
                    f'ref_yaw={math.degrees(float(reference_yaw)):.1f}°'
                )
            resolved_forward_m = float(
                waypoint.get('_resolved_body_forward_m', waypoint['body_forward_m'])
            )
            resolved_left_m = float(
                waypoint.get('_resolved_body_left_m', waypoint['body_left_m'])
            )
            target_text = (
                f'{target_text} '
                f'(body_relative forward={resolved_forward_m:.3f}, '
                f'left={resolved_left_m:.3f}, '
                f'yaw_mode={waypoint.get("body_yaw_mode", "delta")}, '
                f'yaw_delta={float(waypoint["body_yaw_delta_deg"]):.1f}°'
                f'{source_text}{anchor_text}{reference_text})'
            )
        elif self.pose_recovery_offset_active:
            target_text = (
                f'{target_text} '
                f'(nominal=({waypoint["x"]:.3f},{waypoint["y"]:.3f},{waypoint["yaw_deg"]:.1f}°), '
                'pose_recovery=active)'
            )
        self.active_goal_msg = goal
        self.goal_active = True
        self.active_goal_stamp = self._now_sec()
        self.active_goal_start_captured = False
        self.active_goal_start_yaw = 0.0
        self.active_goal_min_dist = math.inf
        self.current_goal_retry_count = max(self.current_goal_retry_count, 0)
        self._capture_goal_start_pose()

        if bool(waypoint.get('use_direct_step_override', False)):
            publish_path_reference = bool(
                waypoint.get('publish_path_reference_for_direct_step_override', False)
            )
            if publish_path_reference:
                if bool(waypoint.get('publish_path_reference_start_for_direct_step_override', False)):
                    start_msg = self._build_path_reference_start_msg(waypoint, goal)
                    if start_msg is not None:
                        self.path_reference_start_pub.publish(start_msg)
                self.path_reference_pub.publish(goal)
                self._publish_path_correction_profile(
                    int(waypoint.get('path_correction_profile_on_arrival', 1))
                )
            override_mode, override_left, override_right, yaw_correction_text = (
                self._direct_step_override_values(waypoint)
            )
            self.active_step_override_debug_text = yaw_correction_text
            self._publish_step_override(
                override_mode,
                override_left,
                override_right,
            )
            path_reference_text = (
                f'path_reference={self.path_reference_topic}, '
                if publish_path_reference
                else ''
            )
            self._publish_feedback(
                f'发送直驱覆盖点[{self.current_idx + 1}/{len(self.waypoints)}](attempt={self.current_goal_retry_count + 1}): '
                f'{waypoint["name"]} -> {target_text}, frame={self._target_frame_for_waypoint(waypoint)}, '
                f'{path_reference_text}'
                f'override=[{override_mode},{override_left:.3f},{override_right:.3f}]'
                f'{yaw_correction_text}, '
                f'到点后锁存模式={waypoint["mode_on_arrival"]}, note={waypoint["note"]}'
            )
        else:
            self.goal_pub.publish(goal)
            self._publish_feedback(
                f'发送目标点[{self.current_idx + 1}/{len(self.waypoints)}](attempt={self.current_goal_retry_count + 1}): '
                f'{waypoint["name"]} -> {target_text}, frame={self._target_frame_for_waypoint(waypoint)}, '
                f'到点后锁存模式={waypoint["mode_on_arrival"]}, note={waypoint["note"]}'
            )

        self._publish_state('NAVIGATING')
        self._publish_monitor_state('TRACKING')
        self._publish_current_waypoint(waypoint)

    def _advance_once(self) -> None:
        self._clear_transition_timer()
        self._clear_post_arrival_mode_sequence()
        self._clear_pose_stability_state()
        self.current_idx += 1
        self.current_goal_retry_count = 0
        if self._route_end_reached():
            self._finish_route()
            return
        self._publish_feedback(f'切换到下一个 waypoint: {self.current_idx + 1} / {len(self.waypoints)}')
        self._send_current_goal()

    def _retry_current_goal_once(self) -> None:
        self._clear_transition_timer()
        self._clear_post_arrival_mode_sequence()
        self._clear_pose_stability_state()
        if self.route_finished or self.route_failed:
            return
        self._send_current_goal()

    def _finish_route(self) -> None:
        self.route_finished = True
        self.goal_active = False
        self._clear_post_arrival_mode_sequence()
        self._clear_pose_stability_state()
        self._clear_pose_recovery_offset('流程结束')
        self._clear_step_override()
        self._clear_direct_step_yaw_recovery_state()
        self._publish_arrival(True)
        self._publish_mode(self.finish_mode)
        self._publish_uphill_step_floor_state(False)
        self._publish_path_correction_profile(0)
        self._publish_state('FINISHED')
        self._publish_monitor_state('FINISHED')
        self._publish_feedback(f'全部 waypoint 已完成，恢复为 mode={self.finish_mode}，流程结束。')

    def _start_post_arrival_mode_sequence_if_needed(self, waypoint: Dict[str, Any]) -> bool:
        sequence = list(waypoint.get('post_arrival_mode_sequence', []))
        if not sequence:
            return False

        self.pending_post_arrival_mode_sequence = sequence
        self.pending_post_arrival_waypoint_name = str(waypoint['name'])
        initial_delay_sec = max(
            0.0,
            float(waypoint.get('post_arrival_pause_sec', self.settle_after_arrival_sec)),
        )
        if initial_delay_sec > 1e-6:
            self._publish_feedback(
                f'waypoint[{self.current_idx}]={waypoint["name"]} 到点后等待 {initial_delay_sec:.2f}s，再执行额外动作序列。'
            )
            self._schedule_transition(initial_delay_sec, self._run_next_post_arrival_mode_sequence_step)
        else:
            self._publish_feedback(
                f'waypoint[{self.current_idx}]={waypoint["name"]} 到点后立即执行额外动作序列。'
            )
            if not self._start_apriltag_pre_stand_backoff_if_needed(waypoint):
                self._run_next_post_arrival_mode_sequence_step()
        return True

    def _run_next_post_arrival_mode_sequence_step(self) -> None:
        self._clear_transition_timer()
        if self._start_apriltag_pre_stand_backoff_if_needed(self.waypoints[self.current_idx]):
            return
        if not self.pending_post_arrival_mode_sequence:
            self._advance_once()
            return

        step = self.pending_post_arrival_mode_sequence.pop(0)
        mode = int(step['mode'])
        hold_sec = max(0.0, float(step.get('hold_sec', 0.0)))
        note = str(step.get('note', ''))
        description = MODE_DESCRIPTIONS.get(mode, '未知模式')
        if bool(step.get('publish_mode', True)):
            self._publish_mode(mode)
        if bool(step.get('use_direct_step_override', False)):
            self._publish_step_override(
                int(step['direct_step_override_mode']),
                float(step['direct_step_override_left_norm']),
                float(step['direct_step_override_right_norm']),
            )
            note_suffix = (
                f', override=[{int(step["direct_step_override_mode"])},'
                f'{float(step["direct_step_override_left_norm"]):.3f},'
                f'{float(step["direct_step_override_right_norm"]):.3f}]'
            )
        elif bool(step.get('use_step_once', False)):
            self._clear_step_override()
            self._publish_step_once(
                int(step['step_once_mode']),
                float(step['step_once_left_norm']),
                float(step['step_once_right_norm']),
            )
            note_suffix = (
                f', step_once=[{int(step["step_once_mode"])},'
                f'{float(step["step_once_left_norm"]):.3f},'
                f'{float(step["step_once_right_norm"]):.3f}]'
            )
        else:
            self._clear_step_override()
            note_suffix = ''
        if note:
            note_suffix += f', note={note}'
        self._publish_feedback(
            f'执行到点后额外动作: waypoint={self.pending_post_arrival_waypoint_name}, '
            f'mode={mode} ({description}), hold={hold_sec:.2f}s{note_suffix}'
        )

        next_callback = (
            self._run_next_post_arrival_mode_sequence_step
            if self.pending_post_arrival_mode_sequence
            else self._advance_after_optional_pose_stability
        )
        self._schedule_transition(hold_sec, next_callback)

    def _start_apriltag_pre_stand_backoff_if_needed(self, waypoint: Dict[str, Any]) -> bool:
        if self.apriltag_pre_stand_backoff_active:
            return True
        if not bool(waypoint.get('apriltag_pre_stand_backoff_enabled', False)):
            return False
        if self.apriltag_pre_stand_backoff_waypoint_idx == self.current_idx:
            return False
        z_m = self._fresh_apriltag_z_for_waypoint(
            waypoint,
            float(waypoint.get('apriltag_pre_stand_backoff_stale_timeout_sec', 0.30)),
        )
        trigger_z_m = float(waypoint.get('apriltag_pre_stand_backoff_trigger_z_m', 0.64))
        if z_m is None or z_m >= trigger_z_m:
            return False

        self.apriltag_pre_stand_backoff_active = True
        self.apriltag_pre_stand_backoff_waypoint_idx = self.current_idx
        self.apriltag_pre_stand_backoff_start_sec = self._now_sec()
        self.apriltag_pre_stand_backoff_last_log_sec = 0.0
        mode = int(waypoint.get('apriltag_pre_stand_backoff_mode', waypoint['mode_on_arrival']))
        left = float(waypoint.get('apriltag_pre_stand_backoff_left_norm', -0.20))
        right = float(waypoint.get('apriltag_pre_stand_backoff_right_norm', -0.20))
        target_z_m = float(waypoint.get('apriltag_pre_stand_backoff_target_z_m', trigger_z_m))
        self._publish_mode(mode)
        self._publish_step_override(mode, left, right)
        self._publish_feedback(
            f'AprilTag 站立前距离过近，先后退保护: waypoint[{self.current_idx}]={waypoint["name"]}, '
            f'z={z_m:.3f}m < trigger={trigger_z_m:.3f}m, target={target_z_m:.3f}m, '
            f'override=[{mode},{left:.3f},{right:.3f}]。'
        )
        self._schedule_transition(
            max(0.02, float(waypoint.get('apriltag_pre_stand_backoff_check_period_sec', 0.08))),
            self._run_apriltag_pre_stand_backoff,
        )
        return True

    def _run_apriltag_pre_stand_backoff(self) -> None:
        self._clear_transition_timer()
        if not self.apriltag_pre_stand_backoff_active:
            self._run_next_post_arrival_mode_sequence_step()
            return
        waypoint = self.waypoints[self.current_idx]
        z_m = self._fresh_apriltag_z_for_waypoint(
            waypoint,
            float(waypoint.get('apriltag_pre_stand_backoff_stale_timeout_sec', 0.30)),
        )
        now = self._now_sec()
        target_z_m = float(waypoint.get('apriltag_pre_stand_backoff_target_z_m', 0.67))
        timeout_sec = max(0.0, float(waypoint.get('apriltag_pre_stand_backoff_timeout_sec', 1.20)))
        elapsed_sec = now - self.apriltag_pre_stand_backoff_start_sec
        if z_m is not None and z_m >= target_z_m:
            self._publish_feedback(
                f'AprilTag 站立前后退保护完成: z={z_m:.3f}m >= target={target_z_m:.3f}m, '
                '继续执行站立序列。'
            )
            self._finish_apriltag_pre_stand_backoff_state()
            self._run_next_post_arrival_mode_sequence_step()
            return
        if timeout_sec > 0.0 and elapsed_sec >= timeout_sec:
            z_text = 'unknown' if z_m is None else f'{z_m:.3f}m'
            self._publish_feedback(
                f'AprilTag 站立前后退保护超时: z={z_text}, target={target_z_m:.3f}m, '
                f'timeout={timeout_sec:.2f}s，继续执行站立序列。',
                level='warn',
            )
            self._finish_apriltag_pre_stand_backoff_state()
            self._run_next_post_arrival_mode_sequence_step()
            return

        mode = int(waypoint.get('apriltag_pre_stand_backoff_mode', waypoint['mode_on_arrival']))
        left = float(waypoint.get('apriltag_pre_stand_backoff_left_norm', -0.20))
        right = float(waypoint.get('apriltag_pre_stand_backoff_right_norm', -0.20))
        self._publish_step_override(mode, left, right)
        if (now - self.apriltag_pre_stand_backoff_last_log_sec) >= 0.30:
            self.apriltag_pre_stand_backoff_last_log_sec = now
            z_text = 'unknown' if z_m is None else f'{z_m:.3f}m'
            self._publish_feedback(
                f'AprilTag 站立前后退中: z={z_text}, target={target_z_m:.3f}m, '
                f'elapsed={elapsed_sec:.2f}/{timeout_sec:.2f}s, override=[{mode},{left:.3f},{right:.3f}]。'
            )
        self._schedule_transition(
            max(0.02, float(waypoint.get('apriltag_pre_stand_backoff_check_period_sec', 0.08))),
            self._run_apriltag_pre_stand_backoff,
        )

    def _should_use_smooth_transition(self, waypoint: Dict[str, Any]) -> bool:
        if not bool(waypoint.get('smooth_transition', True)):
            return False
        if bool(waypoint.get('require_pose_stable_before_next', False)):
            return False
        if self.current_idx >= (len(self.waypoints) - 1):
            return False
        if waypoint.get('post_arrival_mode_sequence'):
            return False
        if bool(waypoint.get('use_direct_step_override', False)):
            return False
        return True

    def _advance_after_optional_pose_stability(self) -> None:
        self._clear_transition_timer()
        if self._route_end_reached():
            self._finish_route()
            return

        waypoint = self.waypoints[self.current_idx]
        if not bool(waypoint.get('require_pose_stable_before_next', False)):
            self._advance_once()
            return

        self._apply_pose_stable_hold_control(waypoint)

        self.pending_pose_stable_waypoint_idx = self.current_idx
        now = self._now_sec()
        self.pose_stable_wait_start_time = now
        self.pose_stable_anchor_time = now
        self.pose_stable_anchor_x = self.pose_x
        self.pose_stable_anchor_y = self.pose_y
        self.pose_stable_anchor_yaw = self.pose_yaw
        self.last_pose_stable_log_time = 0.0

        nominal_expected = self._pose_stable_expected_pose(waypoint)
        expected = self._apply_pose_recovery_transform(*nominal_expected)
        recovery_suffix = ''
        if self.pose_recovery_offset_active:
            recovery_suffix = (
                f' nominal=({nominal_expected[0]:.3f},{nominal_expected[1]:.3f}),'
                ' pose_recovery=active'
            )
        self._publish_feedback(
            f'waypoint[{self.current_idx}]={waypoint["name"]} 启用局部定位稳定门控：'
            f'expected=({expected[0]:.3f},{expected[1]:.3f}), '
            f'需要连续稳定 {float(waypoint["pose_stable_duration_sec"]):.2f}s 后再切换到下一个目标。'
            f'{recovery_suffix}'
        )
        self._schedule_transition(0.05, self._pose_stability_check_loop)

    def _apply_pose_stable_hold_control(self, waypoint: Dict[str, Any]) -> None:
        hold_mode = waypoint.get('pose_stable_hold_mode')
        gate_override_mode = waypoint.get('pose_stable_direct_step_override_mode')
        self._clear_step_override()
        self.pose_stable_override_active = False

        if hold_mode is not None:
            self._publish_mode(int(hold_mode))
        elif gate_override_mode is not None:
            self._publish_mode(int(gate_override_mode))

        if not bool(waypoint.get('pose_stable_use_direct_step_override', False)):
            return

        mode = int(
            gate_override_mode
            if gate_override_mode is not None
            else (hold_mode if hold_mode is not None else self.idle_mode)
        )
        left_norm = float(waypoint.get('pose_stable_direct_step_override_left_norm', 0.0))
        right_norm = float(waypoint.get('pose_stable_direct_step_override_right_norm', 0.0))
        self._publish_step_override(mode, left_norm, right_norm)
        self.pose_stable_override_active = True

    def _pose_stable_expected_pose(self, waypoint: Dict[str, Any]) -> tuple:
        source = str(waypoint.get('pose_stable_expected_pose_source', 'next')).strip().lower()
        expected_x = waypoint.get('pose_stable_expected_x')
        expected_y = waypoint.get('pose_stable_expected_y')
        expected_yaw_deg = waypoint.get('pose_stable_expected_yaw_deg')

        source_waypoint = waypoint
        if source == 'next' and self.current_idx < (len(self.waypoints) - 1):
            source_waypoint = self.waypoints[self.current_idx + 1]
        elif source == 'current':
            source_waypoint = waypoint
        elif source == 'custom':
            source_waypoint = waypoint
        elif source not in ('next', 'current', 'custom'):
            self._publish_feedback(
                f'未知 pose_stable_expected_pose_source={source}，回退使用 next。',
                level='warn',
            )
            if self.current_idx < (len(self.waypoints) - 1):
                source_waypoint = self.waypoints[self.current_idx + 1]

        target_x, target_y, target_yaw = self._target_pose_for_waypoint(source_waypoint)
        if expected_x is None:
            expected_x = target_x
        if expected_y is None:
            expected_y = target_y
        if expected_yaw_deg is None:
            return float(expected_x), float(expected_y), float(target_yaw)
        return float(expected_x), float(expected_y), math.radians(float(expected_yaw_deg))

    def _pose_inside_stable_bounds(self, waypoint: Dict[str, Any]) -> bool:
        checks = (
            ('pose_stable_min_x', lambda value: self.pose_x >= value),
            ('pose_stable_max_x', lambda value: self.pose_x <= value),
            ('pose_stable_min_y', lambda value: self.pose_y >= value),
            ('pose_stable_max_y', lambda value: self.pose_y <= value),
        )
        for key, predicate in checks:
            value = waypoint.get(key)
            if value is not None and not predicate(float(value)):
                return False
        return True

    def _pose_stability_check_loop(self) -> None:
        self._clear_transition_timer()
        idx = self.pending_pose_stable_waypoint_idx
        if idx is None or idx != self.current_idx or idx >= len(self.waypoints):
            self._clear_pose_stability_state()
            return

        waypoint = self.waypoints[idx]
        now = self._now_sec()
        timeout_sec = float(waypoint.get('pose_stable_timeout_sec', 6.0))
        timeout_action = str(waypoint.get('pose_stable_timeout_action', 'hold')).strip().lower()

        if not self.have_pose or self._pose_is_stale(now):
            self._publish_pose_stable_progress(waypoint, '等待有效 pose')
            if self._handle_pose_stable_timeout_if_needed(waypoint, now, timeout_sec, timeout_action):
                return
            self._schedule_transition(0.10, self._pose_stability_check_loop)
            return

        nominal_expected_x, nominal_expected_y, nominal_expected_yaw = self._pose_stable_expected_pose(waypoint)
        expected_x, expected_y, expected_yaw = self._apply_pose_recovery_transform(
            nominal_expected_x,
            nominal_expected_y,
            nominal_expected_yaw,
        )
        position_error = math.hypot(self.pose_x - expected_x, self.pose_y - expected_y)
        max_position_error = float(waypoint.get('pose_stable_max_position_error_m', 0.35))
        yaw_error = abs(self._normalize_angle(expected_yaw - self.pose_yaw))
        max_yaw_error_deg = waypoint.get('pose_stable_max_yaw_error_deg')
        yaw_ok = (
            True
            if max_yaw_error_deg is None
            else yaw_error <= math.radians(float(max_yaw_error_deg))
        )
        bounds_ok = self._pose_inside_stable_bounds(waypoint)
        expected_ok = position_error <= max_position_error and yaw_ok and bounds_ok

        sample_move = math.hypot(self.pose_x - self.pose_stable_anchor_x, self.pose_y - self.pose_stable_anchor_y)
        sample_yaw = abs(self._normalize_angle(self.pose_yaw - self.pose_stable_anchor_yaw))
        sample_move_tol = float(waypoint.get('pose_stable_sample_tolerance_m', 0.08))
        sample_yaw_tol = math.radians(float(waypoint.get('pose_stable_sample_yaw_tolerance_deg', 5.0)))
        if sample_move > sample_move_tol or sample_yaw > sample_yaw_tol:
            self.pose_stable_anchor_time = now
            self.pose_stable_anchor_x = self.pose_x
            self.pose_stable_anchor_y = self.pose_y
            self.pose_stable_anchor_yaw = self.pose_yaw
            sample_move = 0.0
            sample_yaw = 0.0

        stable_elapsed = now - self.pose_stable_anchor_time
        required_stable_sec = float(waypoint.get('pose_stable_duration_sec', 0.80))
        if expected_ok and stable_elapsed >= required_stable_sec:
            self._publish_feedback(
                f'局部定位稳定门控通过: waypoint[{idx}]={waypoint["name"]}, '
                f'pose=({self.pose_x:.3f},{self.pose_y:.3f}), '
                f'expected=({expected_x:.3f},{expected_y:.3f}), '
                f'err={position_error:.3f}m, stable={stable_elapsed:.2f}s。'
            )
            self._clear_pose_stability_state()
            self._advance_once()
            return

        if stable_elapsed >= required_stable_sec:
            if self._try_apply_pose_recovery_offset(
                waypoint,
                nominal_expected_x,
                nominal_expected_y,
                nominal_expected_yaw,
                position_error,
                reason='stable_offset',
            ):
                self._clear_pose_stability_state()
                self._advance_once()
                return

        reason = (
            f'pose=({self.pose_x:.3f},{self.pose_y:.3f}), '
            f'expected=({expected_x:.3f},{expected_y:.3f}), err={position_error:.3f}/{max_position_error:.3f}m, '
            f'bounds_ok={bounds_ok}, yaw_ok={yaw_ok}, stable={stable_elapsed:.2f}/{required_stable_sec:.2f}s'
        )
        self._publish_pose_stable_progress(waypoint, reason)
        if self._handle_pose_stable_timeout_if_needed(waypoint, now, timeout_sec, timeout_action):
            return
        self._schedule_transition(0.10, self._pose_stability_check_loop)

    def _publish_pose_stable_progress(self, waypoint: Dict[str, Any], reason: str) -> None:
        now = self._now_sec()
        if (now - self.last_pose_stable_log_time) < 0.80:
            return
        self.last_pose_stable_log_time = now
        self._publish_feedback(
            f'等待局部定位稳定: waypoint[{self.current_idx}]={waypoint["name"]}, {reason}',
            log_to_console=True,
        )

    def _handle_pose_stable_timeout_if_needed(
        self,
        waypoint: Dict[str, Any],
        now: float,
        timeout_sec: float,
        timeout_action: str,
    ) -> bool:
        if timeout_sec <= 0.0 or (now - self.pose_stable_wait_start_time) <= timeout_sec:
            return False

        self._publish_feedback(
            f'局部定位稳定门控超时: waypoint[{self.current_idx}]={waypoint["name"]}, '
            f'timeout={timeout_sec:.2f}s, action={timeout_action}',
            level='warn',
        )
        nominal_expected_x, nominal_expected_y, nominal_expected_yaw = self._pose_stable_expected_pose(waypoint)
        expected_x, expected_y, _ = self._apply_pose_recovery_transform(
            nominal_expected_x,
            nominal_expected_y,
            nominal_expected_yaw,
        )
        position_error = math.hypot(self.pose_x - expected_x, self.pose_y - expected_y)
        if timeout_action == 'force_offset' or self._pose_recovery_mode_allows(waypoint, 'force_offset'):
            if self._try_apply_pose_recovery_offset(
                waypoint,
                nominal_expected_x,
                nominal_expected_y,
                nominal_expected_yaw,
                position_error,
                reason='force_offset_timeout',
                force=True,
                allow_without_mode=(timeout_action == 'force_offset'),
            ):
                self._clear_pose_stability_state()
                self._advance_once()
                return True
        if timeout_action == 'advance':
            self._publish_localization_failure(
                waypoint,
                '局部定位门控已失效，但配置 timeout_action=advance，将跳过门控继续推进。',
                nominal_expected_x,
                nominal_expected_y,
                nominal_expected_yaw,
                position_error,
            )
            self._clear_pose_stability_state()
            self._advance_once()
            return True
        if timeout_action == 'fail':
            self._publish_localization_failure(
                waypoint,
                '局部定位门控已失效，配置 timeout_action=fail，流程将失败停止。',
                nominal_expected_x,
                nominal_expected_y,
                nominal_expected_yaw,
                position_error,
            )
            self._clear_pose_stability_state()
            self._handle_goal_failure('局部定位稳定门控超时')
            return True

        # Default hold: keep robot stopped and keep checking. This is safer near the wall.
        self._publish_localization_failure(
            waypoint,
            '局部定位门控已失效，当前按 hold 策略保持停止等待恢复。',
            nominal_expected_x,
            nominal_expected_y,
            nominal_expected_yaw,
            position_error,
        )
        self.pose_stable_wait_start_time = now
        return False

    def _pose_recovery_mode_allows(self, waypoint: Dict[str, Any], recovery_kind: str) -> bool:
        mode = str(waypoint.get('pose_recovery_mode', 'none')).strip().lower()
        if mode in ('', 'none', 'off', 'false'):
            return False
        if recovery_kind == 'stable_offset':
            return mode in ('stable_offset', 'stable_then_force')
        if recovery_kind == 'force_offset':
            return mode in ('force_offset', 'stable_then_force')
        return False

    def _try_apply_pose_recovery_offset(
        self,
        waypoint: Dict[str, Any],
        nominal_expected_x: float,
        nominal_expected_y: float,
        nominal_expected_yaw: float,
        position_error: float,
        reason: str,
        force: bool = False,
        allow_without_mode: bool = False,
    ) -> bool:
        recovery_kind = 'force_offset' if force else 'stable_offset'
        if not allow_without_mode and not self._pose_recovery_mode_allows(waypoint, recovery_kind):
            return False
        if not self.have_pose:
            self._publish_localization_failure(
                waypoint,
                f'局部定位恢复失败: 无有效 pose，无法执行 {recovery_kind}。',
                nominal_expected_x,
                nominal_expected_y,
                nominal_expected_yaw,
                position_error,
            )
            return False

        offset_m = math.hypot(self.pose_x - nominal_expected_x, self.pose_y - nominal_expected_y)
        max_offset_m = float(waypoint.get('pose_recovery_max_offset_m', 0.0))
        if max_offset_m <= 0.0:
            self._publish_localization_failure(
                waypoint,
                f'局部定位恢复失败: pose_recovery_max_offset_m={max_offset_m:.3f} 无效，无法执行 {recovery_kind}。',
                nominal_expected_x,
                nominal_expected_y,
                nominal_expected_yaw,
                position_error,
            )
            return False
        if offset_m > max_offset_m:
            self._publish_localization_failure(
                waypoint,
                f'局部定位恢复拒绝: offset={offset_m:.3f}m > max={max_offset_m:.3f}m, reason={reason}。',
                nominal_expected_x,
                nominal_expected_y,
                nominal_expected_yaw,
                position_error,
            )
            return False

        self.pose_recovery_offset_active = True
        self.pose_recovery_nominal_anchor_x = float(nominal_expected_x)
        self.pose_recovery_nominal_anchor_y = float(nominal_expected_y)
        self.pose_recovery_nominal_anchor_yaw = float(nominal_expected_yaw)
        self.pose_recovery_raw_anchor_x = self.pose_x
        self.pose_recovery_raw_anchor_y = self.pose_y
        self.pose_recovery_raw_anchor_yaw = self.pose_yaw
        self.pose_recovery_apply_yaw_offset = bool(waypoint.get('pose_recovery_apply_yaw_offset', False))
        yaw_offset_deg = math.degrees(
            self._normalize_angle(self.pose_recovery_raw_anchor_yaw - self.pose_recovery_nominal_anchor_yaw)
        )
        diagnostic = self._format_localization_diagnostic(
            waypoint,
            nominal_expected_x,
            nominal_expected_y,
            nominal_expected_yaw,
            position_error,
        )
        if force:
            self._publish_feedback(
                f'强制局部定位恢复offset已应用: waypoint[{self.current_idx}]={waypoint["name"]}, '
                f'mode={waypoint.get("pose_recovery_mode", "none")}, reason={reason}, '
                f'offset={offset_m:.3f}m, gate_err={position_error:.3f}m, '
                f'apply_yaw_offset={self.pose_recovery_apply_yaw_offset}, yaw_offset={yaw_offset_deg:.1f}deg。'
                f' {diagnostic}',
                level='warn',
            )
        else:
            self._publish_feedback(
                f'局部定位恢复offset已应用: waypoint[{self.current_idx}]={waypoint["name"]}, '
                f'mode={waypoint.get("pose_recovery_mode", "none")}, reason={reason}, '
                f'offset={offset_m:.3f}m, gate_err={position_error:.3f}m, '
                f'apply_yaw_offset={self.pose_recovery_apply_yaw_offset}, yaw_offset={yaw_offset_deg:.1f}deg。'
                f' {diagnostic}'
            )
        return True

    def _publish_localization_failure(
        self,
        waypoint: Dict[str, Any],
        reason: str,
        nominal_expected_x: float,
        nominal_expected_y: float,
        nominal_expected_yaw: float,
        position_error: float,
    ) -> None:
        self._publish_feedback(
            f'局部定位判断失效: waypoint[{self.current_idx}]={waypoint["name"]}, {reason} '
            f'{self._format_localization_diagnostic(waypoint, nominal_expected_x, nominal_expected_y, nominal_expected_yaw, position_error)}',
            level='error',
        )

    def _format_localization_diagnostic(
        self,
        waypoint: Dict[str, Any],
        nominal_expected_x: float,
        nominal_expected_y: float,
        nominal_expected_yaw: float,
        position_error: float,
    ) -> str:
        expected_x, expected_y, expected_yaw = self._apply_pose_recovery_transform(
            nominal_expected_x,
            nominal_expected_y,
            nominal_expected_yaw,
        )
        expected_yaw_error = math.degrees(abs(self._normalize_angle(expected_yaw - self.pose_yaw)))
        next_goal_text = 'next_goal=None'
        if self.current_idx < (len(self.waypoints) - 1):
            next_waypoint = self.waypoints[self.current_idx + 1]
            next_x, next_y, next_yaw = self._target_pose_for_waypoint(next_waypoint)
            next_goal_text = (
                f'next_goal[{self.current_idx + 1}]={next_waypoint["name"]}'
                f'({next_x:.3f},{next_y:.3f},{math.degrees(next_yaw):.1f}deg)'
            )
        bounds_text = (
            f'bounds_x=[{waypoint.get("pose_stable_min_x")},{waypoint.get("pose_stable_max_x")}], '
            f'bounds_y=[{waypoint.get("pose_stable_min_y")},{waypoint.get("pose_stable_max_y")}]'
        )
        return (
            f'have_pose={self.have_pose}, pose=({self.pose_x:.3f},{self.pose_y:.3f},'
            f'{math.degrees(self.pose_yaw):.1f}deg, frame={self.pose_frame}), '
            f'nominal_expected=({nominal_expected_x:.3f},{nominal_expected_y:.3f},'
            f'{math.degrees(nominal_expected_yaw):.1f}deg), '
            f'adjusted_expected=({expected_x:.3f},{expected_y:.3f},{math.degrees(expected_yaw):.1f}deg), '
            f'position_error={position_error:.3f}m, yaw_error={expected_yaw_error:.1f}deg, '
            f'{next_goal_text}, {bounds_text}, pose_recovery_active={self.pose_recovery_offset_active}'
        )

    def _apply_pose_recovery_transform(self, x: float, y: float, yaw: float) -> tuple:
        if not self.pose_recovery_offset_active:
            return float(x), float(y), float(yaw)

        yaw_delta = 0.0
        if self.pose_recovery_apply_yaw_offset:
            yaw_delta = self._normalize_angle(
                self.pose_recovery_raw_anchor_yaw - self.pose_recovery_nominal_anchor_yaw
            )

        dx = float(x) - self.pose_recovery_nominal_anchor_x
        dy = float(y) - self.pose_recovery_nominal_anchor_y
        cos_yaw = math.cos(yaw_delta)
        sin_yaw = math.sin(yaw_delta)
        transformed_x = self.pose_recovery_raw_anchor_x + cos_yaw * dx - sin_yaw * dy
        transformed_y = self.pose_recovery_raw_anchor_y + sin_yaw * dx + cos_yaw * dy
        transformed_yaw = self._normalize_angle(float(yaw) + yaw_delta)
        return transformed_x, transformed_y, transformed_yaw

    def _waypoint_uses_body_relative_target(self, waypoint: Dict[str, Any]) -> bool:
        return str(waypoint.get('target_type', 'absolute')).strip().lower() == 'body_relative'

    def _body_relative_offset_to_world(self, forward_m: float, left_m: float, yaw_rad: float) -> tuple:
        # Keep the same convention as orange_pole_relative_nav: Odometry yaw=0
        # means body-forward points to global +Y.
        forward_heading = float(yaw_rad) + math.pi / 2.0
        left_heading = forward_heading + math.pi / 2.0
        dx = (
            float(forward_m) * math.cos(forward_heading)
            + float(left_m) * math.cos(left_heading)
        )
        dy = (
            float(forward_m) * math.sin(forward_heading)
            + float(left_m) * math.sin(left_heading)
        )
        return dx, dy

    @staticmethod
    def _pose_x_matches_range(pose_x: float, pose_range: Dict[str, Any]) -> bool:
        min_exclusive = pose_range.get('min_exclusive')
        if min_exclusive is not None and not pose_x > float(min_exclusive):
            return False
        min_inclusive = pose_range.get('min_inclusive')
        if min_inclusive is not None and not pose_x >= float(min_inclusive):
            return False
        max_exclusive = pose_range.get('max_exclusive')
        if max_exclusive is not None and not pose_x < float(max_exclusive):
            return False
        max_inclusive = pose_range.get('max_inclusive')
        if max_inclusive is not None and not pose_x <= float(max_inclusive):
            return False
        return True

    @staticmethod
    def _format_pose_x_range(pose_range: Dict[str, Any]) -> str:
        parts = []
        min_exclusive = pose_range.get('min_exclusive')
        if min_exclusive is not None:
            parts.append(f'x>{float(min_exclusive):.3f}')
        min_inclusive = pose_range.get('min_inclusive')
        if min_inclusive is not None:
            parts.append(f'x>={float(min_inclusive):.3f}')
        max_exclusive = pose_range.get('max_exclusive')
        if max_exclusive is not None:
            parts.append(f'x<{float(max_exclusive):.3f}')
        max_inclusive = pose_range.get('max_inclusive')
        if max_inclusive is not None:
            parts.append(f'x<={float(max_inclusive):.3f}')
        return ' and '.join(parts) if parts else 'all'

    def _previous_pose_x_switch_side_for_waypoint(
        self,
        waypoint: Dict[str, Any],
        source_key: str = 'body_forward_previous_pose_x_switch_source',
    ) -> tuple:
        source_name = str(waypoint.get(source_key, '')).strip()
        if self.last_pose_x_switch_side:
            if not source_name or source_name == self.last_pose_x_switch_waypoint_name:
                return (
                    self.last_pose_x_switch_side,
                    (
                        f' source={self.last_pose_x_switch_waypoint_name}'
                        f'[{self.last_pose_x_switch_waypoint_idx}]'
                    ),
                )

        if source_name:
            return None, f' source={source_name}(unresolved)'
        return None, ' source=history(unresolved)'

    def _body_forward_m_apriltag_z_compensation(self, waypoint: Dict[str, Any]) -> tuple:
        if not bool(waypoint.get('body_forward_m_apriltag_z_compensation_enabled', False)):
            return 0.0, ''

        expected_frame_id = str(
            waypoint.get(
                'body_forward_m_apriltag_z_compensation_frame_id',
                waypoint.get('apriltag_arrival_frame_id', 'camera_color_optical_frame'),
            )
        )
        expected_child_frame_id = str(
            waypoint.get(
                'body_forward_m_apriltag_z_compensation_child_frame_id',
                waypoint.get('apriltag_arrival_child_frame_id', 'bridge'),
            )
        )
        cache_key = (expected_frame_id, expected_child_frame_id)
        cached_sample = getattr(self, 'apriltag_compensation_tf_cache', {}).get(cache_key)
        cache_source = 'compensation_cache'
        if cached_sample is not None:
            tag_z_m = float(cached_sample[0])
            tag_stamp_sec = float(cached_sample[1])
        elif (
            self.apriltag_z_m is not None
            and math.isfinite(float(self.apriltag_z_m))
            and self.apriltag_frame_id == expected_frame_id
            and self.apriltag_child_frame_id == expected_child_frame_id
        ):
            tag_z_m = float(self.apriltag_z_m)
            tag_stamp_sec = float(self.apriltag_stamp_sec)
            cache_source = 'active_goal_cache'
        else:
            return (
                0.0,
                (
                    ', apriltag_z_comp=0.000m'
                    f' reason=no_matching_tf expected={expected_frame_id}->{expected_child_frame_id}'
                    f' cached={self.apriltag_frame_id}->{self.apriltag_child_frame_id}'
                ),
            )

        now_sec = self._now_sec()
        stale_timeout_sec = max(
            0.0,
            float(waypoint.get('body_forward_m_apriltag_z_compensation_stale_timeout_sec', 1.20)),
        )
        stamp_age_sec = now_sec - tag_stamp_sec
        if stale_timeout_sec > 0.0 and stamp_age_sec > stale_timeout_sec:
            return (
                0.0,
                (
                    ', apriltag_z_comp=0.000m'
                    f' reason=stale_tf age={stamp_age_sec:.2f}s>{stale_timeout_sec:.2f}s'
                    f' tf={expected_frame_id}->{expected_child_frame_id}'
                    f' source={cache_source}'
                ),
            )

        neutral_z_m = float(waypoint.get('body_forward_m_apriltag_z_compensation_neutral_z_m', 1.24))
        near_z_m = float(waypoint.get('body_forward_m_apriltag_z_compensation_near_z_m', 1.10))
        near_delta_m = float(waypoint.get('body_forward_m_apriltag_z_compensation_near_delta_m', -0.03))
        far_z_m = float(waypoint.get('body_forward_m_apriltag_z_compensation_far_z_m', 1.50))
        far_delta_m = float(waypoint.get('body_forward_m_apriltag_z_compensation_far_delta_m', 0.05))

        if tag_z_m <= neutral_z_m:
            if abs(neutral_z_m - near_z_m) < 1e-6:
                delta_m = near_delta_m
            else:
                ratio = (tag_z_m - neutral_z_m) / (near_z_m - neutral_z_m)
                ratio = min(max(ratio, 0.0), 1.0)
                delta_m = ratio * near_delta_m
        else:
            if abs(far_z_m - neutral_z_m) < 1e-6:
                delta_m = far_delta_m
            else:
                ratio = (tag_z_m - neutral_z_m) / (far_z_m - neutral_z_m)
                ratio = min(max(ratio, 0.0), 1.0)
                delta_m = ratio * far_delta_m

        return (
            delta_m,
            (
                f', apriltag_z_comp={delta_m:+.3f}m'
                f' tag_z={tag_z_m:.3f}m neutral={neutral_z_m:.3f}m'
                f' near=({near_z_m:.3f}m,{near_delta_m:+.3f}m)'
                f' far=({far_z_m:.3f}m,{far_delta_m:+.3f}m)'
                f' tf_age={stamp_age_sec:.2f}s'
                f' source={cache_source}'
            ),
        )

    def _body_forward_m_for_current_pose(self, waypoint: Dict[str, Any]) -> tuple:
        if (
            bool(waypoint.get('body_forward_m_bridge_retry_override_enabled', False))
            and self._bridge_retry_yaw_reference_is_fresh()
        ):
            override_m = float(
                waypoint.get('body_forward_m_bridge_retry_override_m', 0.0)
            )
            return (
                override_m,
                f', bridge_retry_forward_override={override_m:.3f}m, '
                'apriltag_z_comp=skipped',
            )

        default_forward_m = float(waypoint.get('body_forward_m', 0.0))
        bonus_note = ''
        if bool(waypoint.get('body_forward_m_slope_align_duration_bonus_enabled', False)):
            threshold_sec = max(
                0.0,
                float(waypoint.get('body_forward_m_slope_align_duration_threshold_sec', 1.5)),
            )
            bonus_m = float(waypoint.get('body_forward_m_slope_align_duration_bonus_m', 0.05))
            second_threshold_sec = max(
                0.0,
                float(
                    waypoint.get(
                        'body_forward_m_slope_align_duration_second_threshold_sec',
                        0.0,
                    )
                ),
            )
            second_bonus_m = float(
                waypoint.get(
                    'body_forward_m_slope_align_duration_second_bonus_m',
                    bonus_m,
                )
            )
            duration_sec = float(self.startup_reference_align_duration_sec)
            selected_bonus_m = 0.0
            selected_threshold_sec = threshold_sec
            if (
                second_threshold_sec > threshold_sec
                and duration_sec > second_threshold_sec
                and abs(second_bonus_m) > 1e-6
            ):
                selected_bonus_m = second_bonus_m
                selected_threshold_sec = second_threshold_sec
            elif duration_sec > threshold_sec and abs(bonus_m) > 1e-6:
                selected_bonus_m = bonus_m
                selected_threshold_sec = threshold_sec
            if abs(selected_bonus_m) > 1e-6:
                default_forward_m += selected_bonus_m
                bonus_note = (
                    f', slope_align_duration_bonus=+{selected_bonus_m:.3f}m'
                    f' duration={duration_sec:.2f}s>{selected_threshold_sec:.2f}s'
                )
            else:
                threshold_note = (
                    f'{threshold_sec:.2f}s'
                    if second_threshold_sec <= threshold_sec
                    else f'{threshold_sec:.2f}s/{second_threshold_sec:.2f}s'
                )
                bonus_note = (
                    f', slope_align_duration_bonus=0.000m'
                    f' duration={duration_sec:.2f}s<={threshold_note}'
                )
        apriltag_z_delta_m, apriltag_z_note = self._body_forward_m_apriltag_z_compensation(waypoint)
        default_forward_m += apriltag_z_delta_m
        bonus_note = f'{bonus_note}{apriltag_z_note}'
        if bool(waypoint.get('body_forward_m_from_previous_pose_x_switch_enabled', False)):
            side, source_note = self._previous_pose_x_switch_side_for_waypoint(waypoint)
            switch_map = waypoint.get('body_forward_m_by_previous_pose_x_switch', {})
            if side in switch_map:
                return (
                    float(switch_map[side]) + (default_forward_m - float(waypoint.get('body_forward_m', 0.0))),
                    f'{bonus_note}, previous_pose_x_switch={side}{source_note}',
                )
            if side == 'right':
                selected = waypoint.get('body_forward_m_when_previous_pose_x_right')
                if selected is None:
                    selected = waypoint.get('body_forward_m_when_pose_x_positive')
                if selected is not None:
                    return (
                        float(selected) + (default_forward_m - float(waypoint.get('body_forward_m', 0.0))),
                        f'{bonus_note}, previous_pose_x_switch=right{source_note}',
                    )
            if side == 'left':
                selected = waypoint.get('body_forward_m_when_previous_pose_x_left')
                if selected is None:
                    selected = waypoint.get('body_forward_m_when_pose_x_negative')
                if selected is not None:
                    return (
                        float(selected) + (default_forward_m - float(waypoint.get('body_forward_m', 0.0))),
                        f'{bonus_note}, previous_pose_x_switch=left{source_note}',
                    )
            if side == 'middle':
                selected = waypoint.get('body_forward_m_when_previous_pose_x_center')
                if selected is not None:
                    return (
                        float(selected) + (default_forward_m - float(waypoint.get('body_forward_m', 0.0))),
                        f'{bonus_note}, previous_pose_x_switch=middle{source_note}',
                    )
            return (
                default_forward_m,
                f'{bonus_note}, previous_pose_x_switch=default{source_note}',
            )

        if not bool(waypoint.get('body_forward_m_by_pose_x_enabled', False)):
            return default_forward_m, bonus_note
        if not self.have_pose:
            return default_forward_m, f'{bonus_note}, pose_x_forward_switch=disabled(no_pose)'

        threshold = float(waypoint.get('body_forward_pose_x_threshold', 0.0))
        if self.pose_x > threshold:
            selected = waypoint.get('body_forward_m_when_pose_x_positive')
            if selected is not None:
                    return (
                        float(selected) + (default_forward_m - float(waypoint.get('body_forward_m', 0.0))),
                        f'{bonus_note}, pose_x_forward_switch=right pose_x={self.pose_x:.3f}>{threshold:.3f}',
                    )
        if self.pose_x < threshold:
            selected = waypoint.get('body_forward_m_when_pose_x_negative')
            if selected is not None:
                    return (
                        float(selected) + (default_forward_m - float(waypoint.get('body_forward_m', 0.0))),
                        f'{bonus_note}, pose_x_forward_switch=left pose_x={self.pose_x:.3f}<{threshold:.3f}',
                    )
        return (
            default_forward_m,
            f'{bonus_note}, pose_x_forward_switch=default pose_x={self.pose_x:.3f} threshold={threshold:.3f}',
        )

    def _body_left_m_for_current_pose(self, waypoint: Dict[str, Any]) -> tuple:
        default_left_m = float(waypoint.get('body_left_m', 0.0))
        if bool(waypoint.get('body_left_m_from_previous_pose_x_switch_enabled', False)):
            side, source_note = self._previous_pose_x_switch_side_for_waypoint(
                waypoint,
                'body_left_previous_pose_x_switch_source',
            )
            switch_map = waypoint.get('body_left_m_by_previous_pose_x_switch', {})
            if side in switch_map:
                return (
                    float(switch_map[side]),
                    f', left_previous_pose_x_switch={side}{source_note}',
                    None,
                )
            if side == 'right':
                selected = waypoint.get('body_left_m_when_previous_pose_x_right')
                if selected is None:
                    selected = waypoint.get('body_left_m_when_pose_x_positive')
                if selected is not None:
                    return (
                        float(selected),
                        f', left_previous_pose_x_switch=right{source_note}',
                        None,
                    )
            if side == 'left':
                selected = waypoint.get('body_left_m_when_previous_pose_x_left')
                if selected is None:
                    selected = waypoint.get('body_left_m_when_pose_x_negative')
                if selected is not None:
                    return (
                        float(selected),
                        f', left_previous_pose_x_switch=left{source_note}',
                        None,
                    )
            if side == 'middle':
                selected = waypoint.get('body_left_m_when_previous_pose_x_center')
                if selected is not None:
                    return (
                        float(selected),
                        f', left_previous_pose_x_switch=middle{source_note}',
                        None,
                    )
            return (
                default_left_m,
                f', left_previous_pose_x_switch=default{source_note}',
                None,
            )

        if not bool(waypoint.get('body_left_m_by_pose_x_enabled', False)):
            return default_left_m, '', None
        if not self.have_pose:
            return default_left_m, ', pose_x_switch=disabled(no_pose)', None

        for pose_range in waypoint.get('body_left_m_by_pose_x_ranges', []):
            if self._pose_x_matches_range(self.pose_x, pose_range):
                label = str(pose_range.get('label', '')).strip()
                return (
                    float(pose_range['body_left_m']),
                    (
                        f', pose_x_switch={label} pose_x={self.pose_x:.3f} '
                        f'matched({self._format_pose_x_range(pose_range)})'
                    ),
                    label,
                )

        threshold = float(waypoint.get('body_left_pose_x_threshold', 0.0))
        if self.pose_x > threshold:
            selected = waypoint.get('body_left_m_when_pose_x_positive')
            if selected is not None:
                return (
                    float(selected),
                    f', pose_x_switch=right pose_x={self.pose_x:.3f}>{threshold:.3f}',
                    'right',
                )
        if self.pose_x < threshold:
            selected = waypoint.get('body_left_m_when_pose_x_negative')
            if selected is not None:
                return (
                    float(selected),
                    f', pose_x_switch=left pose_x={self.pose_x:.3f}<{threshold:.3f}',
                    'left',
                )
        return (
            default_left_m,
            f', pose_x_switch=default pose_x={self.pose_x:.3f} threshold={threshold:.3f}',
            None,
        )

    def _previous_target_yaw(self) -> Optional[float]:
        if self.current_idx <= 0:
            return None
        previous_waypoint = self.waypoints[self.current_idx - 1]
        resolved = previous_waypoint.get('_resolved_target')
        if resolved is not None:
            return float(resolved[2])
        if not self._waypoint_uses_body_relative_target(previous_waypoint):
            return math.radians(float(previous_waypoint.get('yaw_deg', 0.0)))
        startup_fallback = self._previous_target_startup_fallback()
        if startup_fallback is not None:
            return float(startup_fallback[2])
        return None

    def _named_waypoint_arrival_yaw(self, waypoint: Dict[str, Any]) -> Optional[float]:
        waypoint.pop('_body_yaw_waypoint_reference_source', None)
        waypoint_name = str(waypoint.get('body_yaw_waypoint', '')).strip()
        if waypoint_name:
            for previous_idx in range(self.current_idx - 1, -1, -1):
                candidate = self.waypoints[previous_idx]
                if str(candidate.get('name', '')) != waypoint_name:
                    continue
                arrival_pose = candidate.get('_arrival_pose')
                if arrival_pose is not None:
                    waypoint['_body_yaw_waypoint_reference_source'] = (
                        f'waypoint_arrival:{waypoint_name}'
                    )
                    return float(arrival_pose[2])
                break

        if (
            bool(
                waypoint.get(
                    'body_yaw_waypoint_fallback_to_bridge_retry_reference',
                    False,
                )
            )
            and self._bridge_retry_yaw_reference_is_fresh()
        ):
            waypoint['_body_yaw_waypoint_reference_source'] = (
                'bridge_retry_yaw_reference'
            )
            return float(self.bridge_retry_yaw_reference)
        return None

    def _previous_target_startup_fallback(self) -> Optional[tuple]:
        if not self.have_pose:
            return None
        if not self.inherit_previous_waypoint_segment_state:
            return None
        if self.start_waypoint_index <= 0 or self.current_idx != self.start_waypoint_index:
            return None
        previous_waypoint = self.waypoints[self.current_idx - 1]
        fallback_yaw = float(self.pose_yaw)
        fallback_note = f'startup_fallback:{previous_waypoint["name"]}'
        if self._startup_reference_is_fresh():
            fallback_yaw = float(self.startup_reference_yaw)
            frame_note = (
                f', ref_frame={self.startup_reference_frame}'
                if self.startup_reference_frame
                else ''
            )
            fallback_note = (
                f'startup_fallback_locked_yaw:{previous_waypoint["name"]}'
                f'{frame_note}'
            )
        return (
            float(self.pose_x),
            float(self.pose_y),
            fallback_yaw,
            fallback_note,
        )

    def _startup_reference_is_fresh(self) -> bool:
        if self.startup_reference_yaw is None:
            return False
        if self.startup_reference_timeout_sec <= 0.0:
            return True
        return (self._now_sec() - self.startup_reference_stamp_sec) <= self.startup_reference_timeout_sec

    def _route_yaw_reference_is_fresh(self) -> bool:
        if not getattr(self, 'runtime_route_yaw_reference_enabled', False):
            return False
        if getattr(self, 'route_yaw_reference', None) is None:
            return False
        timeout_sec = float(getattr(self, 'route_yaw_reference_timeout_sec', 0.0))
        if timeout_sec <= 0.0:
            return True
        stamp_sec = float(getattr(self, 'route_yaw_reference_stamp_sec', 0.0))
        return (self._now_sec() - stamp_sec) <= timeout_sec

    def _bridge_retry_yaw_reference_is_fresh(self) -> bool:
        if not getattr(self, 'runtime_route_yaw_reference_enabled', False):
            return False
        if getattr(self, 'bridge_retry_yaw_reference', None) is None:
            return False
        timeout_sec = float(
            getattr(self, 'bridge_retry_yaw_reference_timeout_sec', 0.0)
        )
        if timeout_sec <= 0.0:
            return True
        stamp_sec = float(
            getattr(self, 'bridge_retry_yaw_reference_stamp_sec', 0.0)
        )
        return (self._now_sec() - stamp_sec) <= timeout_sec

    def _fallback_body_relative_target_yaw(
        self,
        waypoint: Dict[str, Any],
        source_yaw: float,
    ) -> Optional[float]:
        fallback_mode = str(
            waypoint.get('route_yaw_fallback_mode', 'absolute')
        ).strip().lower()
        if fallback_mode in (
            'route_absolute',
            'route_absolute_delta',
            'bridge_reference_delta',
        ):
            fallback_mode = 'absolute'
        fallback_waypoint = dict(waypoint)
        fallback_waypoint['body_yaw_mode'] = fallback_mode
        return self._body_relative_target_yaw(fallback_waypoint, source_yaw)

    def _body_relative_target_yaw(
        self,
        waypoint: Dict[str, Any],
        source_yaw: float,
    ) -> Optional[float]:
        mode = str(waypoint.get('body_yaw_mode', 'delta')).strip().lower()
        if mode == 'current':
            return float(source_yaw)
        if mode in ('route_absolute', 'route_absolute_delta'):
            if not self._route_yaw_reference_is_fresh():
                return self._fallback_body_relative_target_yaw(waypoint, source_yaw)
            target_yaw = (
                float(self.route_yaw_reference)
                + math.radians(float(waypoint.get('yaw_deg', 0.0)))
            )
            if mode == 'route_absolute_delta':
                target_yaw += math.radians(
                    float(waypoint.get('body_yaw_delta_deg', 0.0))
                )
            return self._normalize_angle(target_yaw)
        if mode == 'bridge_reference_delta':
            delta_yaw = math.radians(
                float(waypoint.get('body_yaw_delta_deg', 0.0))
            )
            if self._bridge_retry_yaw_reference_is_fresh():
                return self._normalize_angle(
                    float(self.bridge_retry_yaw_reference) + delta_yaw
                )
            if self._route_yaw_reference_is_fresh():
                return self._normalize_angle(
                    float(self.route_yaw_reference)
                    + math.radians(float(waypoint.get('yaw_deg', 0.0)))
                    + delta_yaw
                )
            return self._fallback_body_relative_target_yaw(waypoint, source_yaw)
        if mode == 'absolute':
            return math.radians(float(waypoint.get('yaw_deg', 0.0)))
        if mode == 'absolute_delta':
            return self._normalize_angle(
                math.radians(float(waypoint.get('yaw_deg', 0.0)))
                + math.radians(float(waypoint.get('body_yaw_delta_deg', 0.0)))
            )
        if mode == 'apriltag_pose':
            return self._apriltag_pose_target_yaw(waypoint, source_yaw)
        if mode in ('previous_target', 'previous_target_delta'):
            previous_yaw = self._previous_target_yaw()
            if previous_yaw is None:
                return None
            if mode == 'previous_target':
                return float(previous_yaw)
            return self._normalize_angle(
                float(previous_yaw) + math.radians(float(waypoint.get('body_yaw_delta_deg', 0.0)))
            )
        if mode == 'waypoint_arrival':
            return self._named_waypoint_arrival_yaw(waypoint)
        return self._normalize_angle(
            float(source_yaw) + math.radians(float(waypoint.get('body_yaw_delta_deg', 0.0)))
        )

    def _apriltag_pose_target_yaw(
        self,
        waypoint: Dict[str, Any],
        source_yaw: float,
    ) -> Optional[float]:
        metric, measured_rad, status = self._apriltag_pose_metric_for_body_yaw(waypoint)
        if measured_rad is None:
            wait_start_sec = waypoint.get('_body_yaw_apriltag_wait_start_sec')
            now_sec = self._now_sec()
            if wait_start_sec is None:
                waypoint['_body_yaw_apriltag_wait_start_sec'] = now_sec
                wait_start_sec = now_sec
            fallback_after_sec = max(
                0.0,
                float(waypoint.get('body_yaw_apriltag_fallback_after_sec', 0.0)),
            )
            if fallback_after_sec > 0.0 and (now_sec - float(wait_start_sec)) < fallback_after_sec:
                self._publish_progress(
                    f'等待 AprilTag 姿态 yaw 锁定: waypoint[{self.current_idx}]={waypoint["name"]}, '
                    f'wait={now_sec - float(wait_start_sec):.2f}/{fallback_after_sec:.2f}s, '
                    f'{self._apriltag_pose_yaw_debug_text(waypoint, status)}',
                    force_console_every=0.5,
                )
                return None
            return self._apriltag_pose_yaw_fallback(waypoint, source_yaw, status)

        waypoint.pop('_body_yaw_apriltag_wait_start_sec', None)
        target_rad = math.radians(float(waypoint.get('body_yaw_apriltag_target_deg', 0.0)))
        raw_error = self._normalize_angle(float(measured_rad) - target_rad)
        control_sign = float(waypoint.get('body_yaw_apriltag_control_sign', -1.0))
        raw_control_error = control_sign * raw_error
        max_correction_deg = waypoint.get('body_yaw_apriltag_max_correction_deg')
        if max_correction_deg is None:
            control_error = raw_control_error
            max_correction_rad = None
            limited = False
        else:
            max_correction_rad = math.radians(max(0.0, float(max_correction_deg)))
            control_error = max(-max_correction_rad, min(max_correction_rad, raw_control_error))
            limited = abs(control_error - raw_control_error) > 1e-9
        target_yaw = self._normalize_angle(float(source_yaw) + control_error)
        sample_count = int(waypoint.get('_body_yaw_apriltag_sample_count', 1))
        sample_values_text = str(waypoint.get('_body_yaw_apriltag_sample_values_text', ''))
        waypoint['_body_yaw_apriltag_resolved'] = (
            str(metric),
            float(measured_rad),
            float(target_rad),
            float(raw_error),
            float(raw_control_error),
            float(control_error),
            None if max_correction_rad is None else float(max_correction_rad),
            float(target_yaw),
            sample_count,
            sample_values_text,
        )
        self._publish_feedback(
            f'AprilTag 姿态 yaw 锁定: waypoint[{self.current_idx}]={waypoint["name"]}, '
            f'tf={waypoint.get("body_yaw_apriltag_frame_id", "")}'
            f'->{waypoint.get("body_yaw_apriltag_child_frame_id", "")}, '
            f'metric={metric}, measured={math.degrees(measured_rad):.2f}deg, '
            f'target_metric={math.degrees(target_rad):.2f}deg, '
            f'raw_err={math.degrees(raw_error):.2f}deg, '
            f'sign={control_sign:.1f}, raw_control={math.degrees(raw_control_error):.2f}deg, '
            f'limited_control={math.degrees(control_error):.2f}deg, '
            f'limit={("none" if max_correction_rad is None else f"{math.degrees(max_correction_rad):.2f}deg")}, '
            f'limited={limited}, samples={sample_count}, sample_values=[{sample_values_text}], '
            f'source_yaw={math.degrees(source_yaw):.2f}deg, '
            f'target_yaw={math.degrees(target_yaw):.2f}deg, '
            f'{self._apriltag_pose_yaw_debug_text(waypoint, "ok")}'
        )
        return target_yaw

    def _apriltag_pose_yaw_debug_text(self, waypoint: Dict[str, Any], status: str) -> str:
        expected_frame_id = str(waypoint.get('body_yaw_apriltag_frame_id', ''))
        expected_child_frame_id = str(waypoint.get('body_yaw_apriltag_child_frame_id', ''))
        metric = str(waypoint.get('body_yaw_apriltag_metric', 'pose_pitch')).strip().lower()
        target_deg = float(waypoint.get('body_yaw_apriltag_target_deg', 0.0))
        sign = float(waypoint.get('body_yaw_apriltag_control_sign', -1.0))
        required_samples = max(1, int(waypoint.get('body_yaw_apriltag_required_samples', 1)))
        sample_reducer = str(
            waypoint.get('body_yaw_apriltag_sample_reducer', 'median')
        ).strip().lower()
        max_correction_deg = waypoint.get('body_yaw_apriltag_max_correction_deg')
        max_correction_text = (
            'none'
            if max_correction_deg is None
            else f'{float(max_correction_deg):.2f}deg'
        )
        stale_timeout_sec = max(
            0.0,
            float(waypoint.get('body_yaw_apriltag_stale_timeout_sec', 0.90)),
        )
        stamp_age_text = (
            'unknown'
            if self.apriltag_stamp_sec <= 0.0
            else f'{self._now_sec() - self.apriltag_stamp_sec:.2f}s'
        )
        rx_age_text = (
            'unknown'
            if self.apriltag_received_sec <= 0.0
            else f'{self._now_sec() - self.apriltag_received_sec:.2f}s'
        )
        xyz_text = (
            'xyz=unknown'
            if self.apriltag_x_m is None or self.apriltag_y_m is None or self.apriltag_z_m is None
            else f'xyz=({self.apriltag_x_m:.3f},{self.apriltag_y_m:.3f},{self.apriltag_z_m:.3f})'
        )
        rpy_text = (
            'rpy=unknown'
            if self.apriltag_roll_rad is None or self.apriltag_pitch_rad is None or self.apriltag_yaw_rad is None
            else (
                f'rpy=({math.degrees(self.apriltag_roll_rad):.2f},'
                f'{math.degrees(self.apriltag_pitch_rad):.2f},'
                f'{math.degrees(self.apriltag_yaw_rad):.2f})deg'
            )
        )
        return (
            f'status={status}, expected_tf={expected_frame_id}->{expected_child_frame_id}, '
            f'cached_tf={self.apriltag_frame_id}->{self.apriltag_child_frame_id}, '
            f'metric={metric}, metric_target={target_deg:.2f}deg, sign={sign:.1f}, '
            f'required_samples={required_samples}, sample_reducer={sample_reducer}, '
            f'max_correction={max_correction_text}, '
            f'{xyz_text}, {rpy_text}, stamp_age={stamp_age_text}, rx_age={rx_age_text}, '
            f'stamp_lag_at_rx={self.apriltag_stamp_lag_at_rx_sec:.2f}s, '
            f'stale_limit={stale_timeout_sec:.2f}s'
        )

    def _apriltag_pose_metric_for_body_yaw(
        self,
        waypoint: Dict[str, Any],
    ) -> Tuple[str, Optional[float], str]:
        if self.apriltag_pose_sample_waypoint_idx != self.current_idx:
            self.apriltag_pose_samples = []
            self.apriltag_pose_sample_waypoint_idx = self.current_idx

        expected_frame_id = str(waypoint.get('body_yaw_apriltag_frame_id', ''))
        expected_child_frame_id = str(waypoint.get('body_yaw_apriltag_child_frame_id', ''))
        metric = str(waypoint.get('body_yaw_apriltag_metric', 'pose_pitch')).strip().lower()
        metric_aliases = {
            'roll': 'pose_roll',
            'pose_roll': 'pose_roll',
            'pitch': 'pose_pitch',
            'pose_pitch': 'pose_pitch',
            'yaw': 'pose_yaw',
            'pose_yaw': 'pose_yaw',
        }
        if metric not in metric_aliases:
            waypoint.pop('_body_yaw_apriltag_sample_count', None)
            waypoint.pop('_body_yaw_apriltag_sample_values_text', None)
            waypoint.pop('_body_yaw_apriltag_sample_reducer', None)
            return metric, None, f'invalid_metric:{metric}'
        metric = metric_aliases[metric]
        reducer = str(waypoint.get('body_yaw_apriltag_sample_reducer', 'median')).strip().lower()
        reducer_aliases = {
            'median': 'median',
            'middle': 'median',
            'mid': 'median',
            'min': 'min',
            'minimum': 'min',
            'lowest': 'min',
            'smallest': 'min',
            'max': 'max',
            'maximum': 'max',
            'highest': 'max',
            'largest': 'max',
            'mean': 'mean',
            'avg': 'mean',
            'average': 'mean',
        }
        if reducer not in reducer_aliases:
            waypoint.pop('_body_yaw_apriltag_sample_count', None)
            waypoint.pop('_body_yaw_apriltag_sample_values_text', None)
            waypoint.pop('_body_yaw_apriltag_sample_reducer', None)
            return metric, None, f'invalid_sample_reducer:{reducer}'
        reducer = reducer_aliases[reducer]
        stale_timeout_sec = max(
            0.0,
            float(waypoint.get('body_yaw_apriltag_stale_timeout_sec', 0.90)),
        )
        required_samples = max(1, int(waypoint.get('body_yaw_apriltag_required_samples', 1)))
        now_sec = self._now_sec()
        metric_key = {
            'pose_roll': 'roll_rad',
            'pose_pitch': 'pitch_rad',
            'pose_yaw': 'yaw_rad',
        }[metric]
        valid_samples: List[Dict[str, Any]] = []
        for sample in self.apriltag_pose_samples:
            if sample.get('frame_id') != expected_frame_id:
                continue
            if sample.get('child_frame_id') != expected_child_frame_id:
                continue
            stamp_sec = float(sample.get('stamp_sec', 0.0))
            if stale_timeout_sec > 0.0 and (stamp_sec <= 0.0 or (now_sec - stamp_sec) > stale_timeout_sec):
                continue
            measured = sample.get(metric_key)
            if measured is None or not math.isfinite(float(measured)):
                continue
            valid_samples.append(sample)

        if len(valid_samples) < required_samples:
            waypoint.pop('_body_yaw_apriltag_sample_count', None)
            waypoint.pop('_body_yaw_apriltag_sample_values_text', None)
            waypoint.pop('_body_yaw_apriltag_sample_reducer', None)
            latest_status = (
                f'cached={self.apriltag_frame_id}->{self.apriltag_child_frame_id}, '
                f'latest_stamp_age={now_sec - self.apriltag_stamp_sec:.2f}s'
                if self.apriltag_stamp_sec > 0.0
                else f'cached={self.apriltag_frame_id}->{self.apriltag_child_frame_id}, latest_stamp_age=unknown'
            )
            return metric, None, (
                f'waiting_samples({len(valid_samples)}/{required_samples}, '
                f'expected={expected_frame_id}->{expected_child_frame_id}, {latest_status})'
            )

        selected_samples = valid_samples[-required_samples:]
        values = sorted(float(sample[metric_key]) for sample in selected_samples)
        if reducer == 'min':
            measured_rad = values[0]
        elif reducer == 'max':
            measured_rad = values[-1]
        elif reducer == 'mean':
            measured_rad = sum(values) / len(values)
        else:
            mid = len(values) // 2
            if len(values) % 2 == 1:
                measured_rad = values[mid]
            else:
                measured_rad = 0.5 * (values[mid - 1] + values[mid])
        sample_values_text = ','.join(f'{math.degrees(value):.2f}' for value in values)
        waypoint['_body_yaw_apriltag_sample_count'] = len(selected_samples)
        waypoint['_body_yaw_apriltag_sample_values_text'] = sample_values_text
        waypoint['_body_yaw_apriltag_sample_reducer'] = reducer
        return metric, float(measured_rad), (
            f'ok_samples({len(selected_samples)}/{required_samples}, reducer={reducer})'
        )

    def _apriltag_pose_yaw_fallback(
        self,
        waypoint: Dict[str, Any],
        source_yaw: float,
        status: str,
    ) -> Optional[float]:
        fallback_mode = str(waypoint.get('body_yaw_apriltag_fallback_mode', 'absolute')).strip().lower()
        if fallback_mode in ('wait', 'none', 'disabled'):
            self._publish_progress(
                f'等待 AprilTag 姿态 yaw 锁定: waypoint[{self.current_idx}]={waypoint["name"]}, '
                f'fallback_mode={fallback_mode}, '
                f'{self._apriltag_pose_yaw_debug_text(waypoint, status)}',
                force_console_every=1.0,
            )
            return None
        if fallback_mode == 'current':
            fallback_yaw = float(source_yaw)
        elif fallback_mode == 'previous_target':
            previous_yaw = self._previous_target_yaw()
            if previous_yaw is None:
                return None
            fallback_yaw = float(previous_yaw)
        else:
            fallback_yaw = math.radians(float(waypoint.get('yaw_deg', 0.0)))
        self._publish_feedback(
            f'AprilTag 姿态 yaw 锁定回退: waypoint[{self.current_idx}]={waypoint["name"]}, '
            f'fallback_mode={fallback_mode}, fallback_yaw={math.degrees(fallback_yaw):.2f}deg, '
            f'{self._apriltag_pose_yaw_debug_text(waypoint, status)}',
            level='warn',
        )
        return self._normalize_angle(fallback_yaw)

    def _body_relative_reference_yaw(
        self,
        waypoint: Dict[str, Any],
        source_yaw: float,
        target_yaw: float,
    ) -> Optional[float]:
        reference_source = str(waypoint.get('body_reference_yaw_source', 'current')).strip().lower()
        if reference_source == 'current':
            return float(source_yaw)
        if reference_source == 'target_yaw':
            return float(target_yaw)
        if reference_source == 'previous_target':
            return self._previous_target_yaw()
        return float(source_yaw)

    def _body_relative_position_anchor(
        self,
        waypoint: Dict[str, Any],
    ) -> Optional[tuple]:
        anchor = str(waypoint.get('body_position_anchor', 'current')).strip().lower()
        if anchor == 'current':
            return self.pose_x, self.pose_y, 'current_pose'
        if anchor == 'previous_target':
            if self.current_idx <= 0:
                return None
            previous_waypoint = self.waypoints[self.current_idx - 1]
            resolved = previous_waypoint.get('_resolved_target')
            if resolved is not None:
                return float(resolved[0]), float(resolved[1]), f'previous_target:{previous_waypoint["name"]}'
            if not self._waypoint_uses_body_relative_target(previous_waypoint):
                x, y, _ = self._target_pose_for_waypoint(previous_waypoint)
                return float(x), float(y), f'previous_target:{previous_waypoint["name"]}'
            startup_fallback = self._previous_target_startup_fallback()
            if startup_fallback is not None:
                return (
                    float(startup_fallback[0]),
                    float(startup_fallback[1]),
                    str(startup_fallback[3]),
                )
            return None
        if anchor == 'previous_arrival':
            if self.current_idx <= 0:
                return None
            previous_waypoint = self.waypoints[self.current_idx - 1]
            arrival_pose = previous_waypoint.get('_arrival_pose')
            if arrival_pose is None:
                return None
            return (
                float(arrival_pose[0]),
                float(arrival_pose[1]),
                f'previous_arrival:{previous_waypoint["name"]}',
            )
        if anchor == 'waypoint_target':
            anchor_name = str(waypoint.get('body_position_anchor_waypoint', '')).strip()
            if not anchor_name:
                return None
            for previous_idx in range(self.current_idx - 1, -1, -1):
                candidate = self.waypoints[previous_idx]
                if str(candidate.get('name', '')) != anchor_name:
                    continue
                resolved = candidate.get('_resolved_target')
                if resolved is not None:
                    return (
                        float(resolved[0]),
                        float(resolved[1]),
                        f'waypoint_target:{anchor_name}',
                    )
                if not self._waypoint_uses_body_relative_target(candidate):
                    x, y, _ = self._target_pose_for_waypoint(candidate)
                    return float(x), float(y), f'waypoint_target:{anchor_name}'
                return None
            return None
        return self.pose_x, self.pose_y, 'current_pose'

    def _wait_for_body_reference(self, waypoint: Dict[str, Any], reason: str) -> bool:
        self._publish_state('WAITING_REFERENCE')
        self._publish_monitor_state('WAITING_REFERENCE')
        self._publish_progress(
            f'等待上一目标参考后解析机身相对 waypoint[{self.current_idx}]={waypoint["name"]}: '
            f'reason={reason}, body_yaw_mode={waypoint.get("body_yaw_mode", "delta")}, '
            f'body_yaw_waypoint={waypoint.get("body_yaw_waypoint", "")}, '
            f'body_reference_yaw_source={waypoint.get("body_reference_yaw_source", "current")}, '
            f'body_position_anchor={waypoint.get("body_position_anchor", "current")}',
            force_console_every=2.0,
        )
        return False

    def _target_frame_for_waypoint(self, waypoint: Dict[str, Any]) -> str:
        return str(waypoint.get('_resolved_frame_id') or waypoint.get('frame_id', self.default_frame_id))

    def _resolve_body_relative_waypoint_target(
        self,
        waypoint: Dict[str, Any],
        *,
        force_new: bool = False,
        log_resolution: bool = True,
    ) -> bool:
        if not self._waypoint_uses_body_relative_target(waypoint):
            return True
        if not force_new and waypoint.get('_resolved_target') is not None:
            return True
        if not self.have_pose:
            self._publish_state('WAITING_POSE')
            self._publish_monitor_state('WAITING_POSE')
            self._publish_progress(
                f'等待 pose 后解析机身相对 waypoint[{self.current_idx}]={waypoint["name"]}',
                force_console_every=2.0,
            )
            return False
        if self._pose_is_stale(self._now_sec()):
            self._publish_state('POSE_STALE')
            self._publish_monitor_state('POSE_STALE')
            self._publish_progress(
                f'pose 超时，暂不解析机身相对 waypoint[{self.current_idx}]={waypoint["name"]}',
                force_console_every=2.0,
            )
            return False

        if not self._body_source_yaw_guard_ready(waypoint):
            return False

        target_yaw = self._body_relative_target_yaw(waypoint, self.pose_yaw)
        if target_yaw is None:
            return self._wait_for_body_reference(waypoint, 'target_yaw_unavailable')
        reference_yaw = self._body_relative_reference_yaw(
            waypoint,
            self.pose_yaw,
            target_yaw,
        )
        if reference_yaw is None:
            return self._wait_for_body_reference(waypoint, 'reference_yaw_unavailable')
        anchor = self._body_relative_position_anchor(waypoint)
        if anchor is None:
            return self._wait_for_body_reference(waypoint, 'position_anchor_unavailable')
        anchor_x = float(anchor[0])
        anchor_y = float(anchor[1])
        anchor_note = str(anchor[2])

        forward_m, forward_note = self._body_forward_m_for_current_pose(waypoint)
        left_m, left_note, pose_x_switch_side = self._body_left_m_for_current_pose(waypoint)
        dx, dy = self._body_relative_offset_to_world(forward_m, left_m, reference_yaw)
        target_x = anchor_x + dx
        target_y = anchor_y + dy
        frame_id = self.pose_frame or str(waypoint.get('frame_id', self.default_frame_id))
        waypoint['_resolved_target'] = (target_x, target_y, target_yaw)
        waypoint['_resolved_frame_id'] = frame_id
        waypoint['_resolved_source_pose'] = (self.pose_x, self.pose_y, self.pose_yaw)
        waypoint['_resolved_body_position_anchor'] = (anchor_x, anchor_y, anchor_note)
        waypoint['_resolved_body_forward_m'] = forward_m
        waypoint['_resolved_body_left_m'] = left_m
        waypoint['_resolved_body_reference_yaw'] = reference_yaw
        waypoint['_resolved_pose_x_switch_side'] = pose_x_switch_side
        if pose_x_switch_side:
            self.last_pose_x_switch_side = pose_x_switch_side
            self.last_pose_x_switch_waypoint_name = str(waypoint.get('name', ''))
            self.last_pose_x_switch_waypoint_idx = self.current_idx
        if log_resolution:
            apriltag_yaw_lock_text = ''
            waypoint_yaw_reference_text = ''
            waypoint_yaw_reference_source = waypoint.get(
                '_body_yaw_waypoint_reference_source'
            )
            if waypoint_yaw_reference_source:
                waypoint_yaw_reference_text = (
                    f', waypoint_yaw_reference={waypoint_yaw_reference_source}'
                )
            resolved_apriltag_yaw = waypoint.get('_body_yaw_apriltag_resolved')
            if resolved_apriltag_yaw is not None:
                try:
                    if len(resolved_apriltag_yaw) >= 10:
                        (
                            tag_metric,
                            tag_measured_rad,
                            tag_target_rad,
                            tag_raw_error,
                            tag_raw_control_error,
                            tag_control_error,
                            tag_max_correction_rad,
                            tag_target_yaw,
                            tag_sample_count,
                            tag_sample_values_text,
                        ) = resolved_apriltag_yaw[:10]
                        tag_max_correction_text = (
                            'none'
                            if tag_max_correction_rad is None
                            else f'{math.degrees(float(tag_max_correction_rad)):.2f}deg'
                        )
                        apriltag_yaw_lock_text = (
                            f', apriltag_yaw_lock=(metric={tag_metric}, '
                            f'measured={math.degrees(float(tag_measured_rad)):.2f}deg, '
                            f'target_metric={math.degrees(float(tag_target_rad)):.2f}deg, '
                            f'raw_err={math.degrees(float(tag_raw_error)):.2f}deg, '
                            f'raw_control={math.degrees(float(tag_raw_control_error)):.2f}deg, '
                            f'limited_control={math.degrees(float(tag_control_error)):.2f}deg, '
                            f'limit={tag_max_correction_text}, '
                            f'samples={int(tag_sample_count)}, '
                            f'sample_values=[{tag_sample_values_text}], '
                            f'locked_yaw={math.degrees(float(tag_target_yaw)):.2f}deg)'
                        )
                    else:
                        (
                            tag_metric,
                            tag_measured_rad,
                            tag_target_rad,
                            tag_raw_error,
                            tag_control_error,
                            tag_target_yaw,
                        ) = resolved_apriltag_yaw
                        apriltag_yaw_lock_text = (
                            f', apriltag_yaw_lock=(metric={tag_metric}, '
                            f'measured={math.degrees(float(tag_measured_rad)):.2f}deg, '
                            f'target_metric={math.degrees(float(tag_target_rad)):.2f}deg, '
                            f'raw_err={math.degrees(float(tag_raw_error)):.2f}deg, '
                            f'control_err={math.degrees(float(tag_control_error)):.2f}deg, '
                            f'locked_yaw={math.degrees(float(tag_target_yaw)):.2f}deg)'
                        )
                except (TypeError, ValueError):
                    apriltag_yaw_lock_text = ', apriltag_yaw_lock=(malformed)'
            self._publish_feedback(
                f'解析机身相对 waypoint[{self.current_idx}]={waypoint["name"]}: '
                f'source=({self.pose_x:.3f},{self.pose_y:.3f},{math.degrees(self.pose_yaw):.1f}deg), '
                f'anchor=({anchor_x:.3f},{anchor_y:.3f},{anchor_note}), '
                f'body_offset=(forward={forward_m:.3f}{forward_note}, left={left_m:.3f}{left_note}), '
                f'yaw_mode={waypoint.get("body_yaw_mode", "delta")}, '
                f'yaw_delta={float(waypoint.get("body_yaw_delta_deg", 0.0)):.1f}deg, '
                f'ref_yaw_source={waypoint.get("body_reference_yaw_source", "current")}, '
                f'ref_yaw={math.degrees(reference_yaw):.1f}deg, '
                f'target=({target_x:.3f},{target_y:.3f},{math.degrees(target_yaw):.1f}deg), '
                f'frame={frame_id}{waypoint_yaw_reference_text}{apriltag_yaw_lock_text}'
            )
        return True

    def _body_source_yaw_guard_ready(self, waypoint: Dict[str, Any]) -> bool:
        if not bool(waypoint.get('body_source_yaw_guard_enabled', False)):
            return True

        reference_mode = str(
            waypoint.get('body_source_yaw_reference_mode', 'absolute')
        ).strip().lower()
        reference_deg = waypoint.get('body_source_yaw_reference_deg')
        if reference_deg is None:
            reference_deg = 0.0
        reference_yaw = math.radians(float(reference_deg))
        reference_text = f'{float(reference_deg):.1f}deg'
        if reference_mode == 'route_zero' and self._route_yaw_reference_is_fresh():
            reference_yaw = float(self.route_yaw_reference)
            reference_text = f'route_zero={math.degrees(reference_yaw):.1f}deg'
        max_error_rad = math.radians(
            max(0.0, float(waypoint.get('body_source_yaw_max_error_deg', 30.0)))
        )
        yaw_error = self._normalize_angle(self.pose_yaw - reference_yaw)
        if abs(yaw_error) <= max_error_rad:
            if bool(waypoint.get('_body_source_yaw_guard_waiting', False)):
                waypoint['_body_source_yaw_guard_waiting'] = False
                self._clear_step_override()
                self._publish_feedback(
                    f'body_relative yaw 守门恢复: waypoint[{self.current_idx}]={waypoint["name"]}, '
                    f'yaw={math.degrees(self.pose_yaw):.1f}deg, '
                    f'ref={reference_text}, '
                    f'err={math.degrees(yaw_error):.1f}deg，继续解析目标。'
                )
            return True

        waypoint['_body_source_yaw_guard_waiting'] = True
        hold_mode = int(waypoint.get('body_source_yaw_guard_hold_mode', 0))
        hold_left = float(waypoint.get('body_source_yaw_guard_hold_left_norm', 0.0))
        hold_right = float(waypoint.get('body_source_yaw_guard_hold_right_norm', 0.0))
        self._publish_step_override(hold_mode, hold_left, hold_right)
        self._publish_state('WAITING_BODY_YAW')
        self._publish_monitor_state('WAITING_BODY_YAW')
        self._publish_progress(
            f'body_relative yaw 守门等待: waypoint[{self.current_idx}]={waypoint["name"]}, '
            f'yaw={math.degrees(self.pose_yaw):.1f}deg, ref={reference_text}, '
            f'err={math.degrees(yaw_error):.1f}deg > max={math.degrees(max_error_rad):.1f}deg，'
            f'保持 [{hold_mode},{hold_left:.3f},{hold_right:.3f}] 等待 Odometry yaw 恢复。',
            force_console_every=1.0,
        )
        return False

    def _target_pose_for_waypoint(self, waypoint: Dict[str, Any]) -> tuple:
        if self._waypoint_uses_body_relative_target(waypoint):
            resolved = waypoint.get('_resolved_target')
            if resolved is None:
                if not self.have_pose:
                    return (
                        float(waypoint.get('x', 0.0)),
                        float(waypoint.get('y', 0.0)),
                        math.radians(float(waypoint.get('yaw_deg', 0.0))),
                )
                forward_m, _ = self._body_forward_m_for_current_pose(waypoint)
                left_m, _, _ = self._body_left_m_for_current_pose(waypoint)
                target_yaw = self._body_relative_target_yaw(waypoint, self.pose_yaw)
                if target_yaw is None:
                    target_yaw = self.pose_yaw
                reference_yaw = self._body_relative_reference_yaw(
                    waypoint,
                    self.pose_yaw,
                    target_yaw,
                )
                if reference_yaw is None:
                    reference_yaw = self.pose_yaw
                anchor = self._body_relative_position_anchor(waypoint)
                if anchor is None:
                    anchor_x = self.pose_x
                    anchor_y = self.pose_y
                else:
                    anchor_x = float(anchor[0])
                    anchor_y = float(anchor[1])
                dx, dy = self._body_relative_offset_to_world(forward_m, left_m, reference_yaw)
                return (
                    anchor_x + dx,
                    anchor_y + dy,
                    target_yaw,
                )
            return (
                float(resolved[0]),
                float(resolved[1]),
                float(resolved[2]),
            )
        return self._apply_pose_recovery_transform(
            float(waypoint['x']),
            float(waypoint['y']),
            math.radians(float(waypoint['yaw_deg'])),
        )

    def _clear_resolved_waypoint_targets(self) -> None:
        for waypoint in self.waypoints:
            waypoint.pop('_resolved_target', None)
            waypoint.pop('_resolved_frame_id', None)
            waypoint.pop('_resolved_source_pose', None)
            waypoint.pop('_resolved_body_position_anchor', None)
            waypoint.pop('_resolved_body_forward_m', None)
            waypoint.pop('_resolved_body_left_m', None)
            waypoint.pop('_resolved_body_reference_yaw', None)
            waypoint.pop('_resolved_pose_x_switch_side', None)
            waypoint.pop('_body_yaw_apriltag_wait_start_sec', None)
            waypoint.pop('_body_yaw_apriltag_resolved', None)
            waypoint.pop('_body_yaw_apriltag_sample_count', None)
            waypoint.pop('_body_yaw_apriltag_sample_values_text', None)
            waypoint.pop('_body_yaw_apriltag_sample_reducer', None)
            waypoint.pop('_body_yaw_waypoint_reference_source', None)
            waypoint.pop('_arrival_pose', None)

    def _clear_pose_recovery_offset(self, reason: str) -> None:
        if self.pose_recovery_offset_active:
            self._publish_feedback(f'清除局部定位恢复offset: reason={reason}')
        self.pose_recovery_offset_active = False
        self.pose_recovery_nominal_anchor_x = 0.0
        self.pose_recovery_nominal_anchor_y = 0.0
        self.pose_recovery_nominal_anchor_yaw = 0.0
        self.pose_recovery_raw_anchor_x = 0.0
        self.pose_recovery_raw_anchor_y = 0.0
        self.pose_recovery_raw_anchor_yaw = 0.0
        self.pose_recovery_apply_yaw_offset = False

    def _schedule_transition(self, delay_sec: float, callback) -> None:
        self._clear_transition_timer()
        self.transition_timer = self.create_timer(max(delay_sec, 0.0), callback)

    def _clear_transition_timer(self) -> None:
        if self.transition_timer is not None:
            self.transition_timer.cancel()
            self.transition_timer = None

    def _clear_post_arrival_mode_sequence(self) -> None:
        self.pending_post_arrival_mode_sequence = []
        self.pending_post_arrival_waypoint_name = None

    def _clear_apriltag_pre_stand_backoff_state(self) -> None:
        self.apriltag_pre_stand_backoff_active = False
        self.apriltag_pre_stand_backoff_waypoint_idx = None
        self.apriltag_pre_stand_backoff_start_sec = 0.0
        self.apriltag_pre_stand_backoff_last_log_sec = 0.0

    def _finish_apriltag_pre_stand_backoff_state(self) -> None:
        self.apriltag_pre_stand_backoff_active = False
        self.apriltag_pre_stand_backoff_waypoint_idx = self.current_idx
        self.apriltag_pre_stand_backoff_start_sec = 0.0
        self.apriltag_pre_stand_backoff_last_log_sec = 0.0

    def _clear_apriltag_arrival_candidate(self) -> None:
        self.apriltag_arrival_candidate_idx = None
        self.apriltag_arrival_candidate_start_sec = 0.0

    def _clear_apriltag_arrival_filter(self) -> None:
        self.apriltag_sample_waypoint_idx = None
        self.apriltag_z_samples = []
        self.apriltag_filtered_z_m = None
        self.apriltag_filter_below_count = 0
        self.apriltag_filter_sample_count = 0
        self.apriltag_filter_window_size = 0
        self.apriltag_filter_required_count = 0

    def _clear_direct_step_yaw_recovery_state(self) -> None:
        self.direct_step_yaw_recovery_active = False
        self.direct_step_yaw_recovery_waypoint_idx = None
        self.direct_step_yaw_recovery_enter_candidate_start_sec = 0.0
        self.direct_step_yaw_recovery_exit_candidate_start_sec = 0.0
        self.direct_step_yaw_recovery_active_start_sec = 0.0
        self.direct_step_yaw_recovery_cooldown_until_sec = 0.0

    def _clear_pose_stability_state(self) -> None:
        if self.pose_stable_override_active:
            self._clear_step_override()
        self.pose_stable_override_active = False
        self.pending_pose_stable_waypoint_idx = None
        self.pose_stable_wait_start_time = 0.0
        self.pose_stable_anchor_time = 0.0
        self.pose_stable_anchor_x = 0.0
        self.pose_stable_anchor_y = 0.0
        self.pose_stable_anchor_yaw = 0.0
        self.last_pose_stable_log_time = 0.0

    def _ensure_step_override_refresh_timer(self) -> None:
        if self.step_override_refresh_timer is not None:
            return
        self.step_override_refresh_timer = self.create_timer(
            self.step_override_refresh_interval_sec,
            self._refresh_active_step_override,
        )

    def _clear_step_override_refresh_timer(self) -> None:
        if self.step_override_refresh_timer is not None:
            self.step_override_refresh_timer.cancel()
            self.step_override_refresh_timer = None

    def _refresh_active_step_override(self) -> None:
        if self.active_step_override_values is None:
            return
        msg = Float32MultiArray()
        msg.data = list(self.active_step_override_values)
        self.step_override_pub.publish(msg)

    def _publish_arrival(self, value: bool) -> None:
        msg = Bool()
        msg.data = value
        self.arrival_pub.publish(msg)

    def _refresh_direct_step_override(self, waypoint: Dict[str, Any]) -> None:
        if not bool(waypoint.get('use_direct_step_override', False)):
            return
        mode, left_norm, right_norm, debug_text = self._direct_step_override_values(waypoint)
        self.active_step_override_debug_text = debug_text
        self._publish_step_override(
            mode,
            left_norm,
            right_norm,
        )

    def _build_path_reference_start_msg(
        self,
        waypoint: Dict[str, Any],
        goal: PoseStamped,
    ) -> Optional[PoseStamped]:
        start_source = str(waypoint.get('path_reference_start_source', 'anchor')).strip().lower()
        if start_source in ('current', 'current_odom', 'odom'):
            if not self.have_pose:
                return None
            start_x = self.pose_x
            start_y = self.pose_y
            start_yaw = self.pose_yaw
        elif start_source == 'source_pose':
            start_pose = waypoint.get('_resolved_source_pose')
            if start_pose is None:
                return None
            start_x = float(start_pose[0])
            start_y = float(start_pose[1])
            start_yaw = float(start_pose[2])
        else:
            anchor = waypoint.get('_resolved_body_position_anchor')
            if anchor is None:
                return None
            start_x = float(anchor[0])
            start_y = float(anchor[1])
            target = waypoint.get('_resolved_target')
            start_yaw = float(target[2]) if target is not None else self.pose_yaw
        if start_yaw is None:
            return None
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = goal.header.frame_id
        msg.pose.position.x = start_x
        msg.pose.position.y = start_y
        msg.pose.position.z = 0.0
        msg.pose.orientation = self._quaternion_from_yaw(start_yaw)
        return msg

    def _direct_step_yaw_recovery_values(self, waypoint: Dict[str, Any]) -> Optional[tuple]:
        if not bool(waypoint.get('direct_step_yaw_recovery_enabled', False)):
            if self.direct_step_yaw_recovery_waypoint_idx == self.current_idx:
                self._clear_direct_step_yaw_recovery_state()
            return None
        if self.direct_step_yaw_recovery_waypoint_idx not in (None, self.current_idx):
            self._clear_direct_step_yaw_recovery_state()

        target = waypoint.get('_resolved_target')
        if target is None or not self.have_pose:
            return None

        now = self._now_sec()
        target_yaw = float(target[2])
        yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
        yaw_error_deg = math.degrees(yaw_error)
        abs_yaw_error_deg = abs(yaw_error_deg)
        exit_yaw_error_deg = max(
            0.0,
            float(waypoint.get('direct_step_yaw_recovery_exit_yaw_error_deg', 3.0)),
        )
        enter_yaw_error_deg = max(
            exit_yaw_error_deg,
            float(waypoint.get('direct_step_yaw_recovery_enter_yaw_error_deg', 6.0)),
        )
        enter_hold_sec = max(
            0.0,
            float(waypoint.get('direct_step_yaw_recovery_enter_hold_sec', 0.20)),
        )
        exit_hold_sec = max(
            0.0,
            float(waypoint.get('direct_step_yaw_recovery_exit_hold_sec', 0.20)),
        )
        timeout_sec = max(
            0.0,
            float(waypoint.get('direct_step_yaw_recovery_timeout_sec', 1.20)),
        )
        cooldown_sec = max(
            0.0,
            float(waypoint.get('direct_step_yaw_recovery_cooldown_sec', 0.35)),
        )

        metrics = self._goal_line_metrics(waypoint)
        lateral_error_m: Optional[float] = None
        progress_ratio: Optional[float] = None
        if metrics is not None:
            _, _, lateral_error_m, progress_ratio, _ = metrics

        if self.direct_step_yaw_recovery_active:
            if timeout_sec > 0.0 and (
                now - self.direct_step_yaw_recovery_active_start_sec
            ) >= timeout_sec:
                self._publish_feedback(
                    f'direct_step yaw recovery 超时退出: waypoint[{self.current_idx}]={waypoint["name"]}, '
                    f'yaw_err={abs_yaw_error_deg:.1f}deg, timeout={timeout_sec:.2f}s，恢复基准线趴走。',
                    level='warn',
                )
                self._clear_direct_step_yaw_recovery_state()
                self.direct_step_yaw_recovery_cooldown_until_sec = now + cooldown_sec
                return None

            if abs_yaw_error_deg <= exit_yaw_error_deg:
                if self.direct_step_yaw_recovery_exit_candidate_start_sec <= 0.0:
                    self.direct_step_yaw_recovery_exit_candidate_start_sec = now
                if (now - self.direct_step_yaw_recovery_exit_candidate_start_sec) >= exit_hold_sec:
                    self._publish_feedback(
                        f'direct_step yaw recovery 完成: waypoint[{self.current_idx}]={waypoint["name"]}, '
                        f'yaw_err={abs_yaw_error_deg:.1f}deg <= {exit_yaw_error_deg:.1f}deg，恢复基准线修正。'
                    )
                    self._clear_direct_step_yaw_recovery_state()
                    return None
            else:
                self.direct_step_yaw_recovery_exit_candidate_start_sec = 0.0

            return self._direct_step_yaw_recovery_turn_values(
                waypoint,
                yaw_error,
                yaw_error_deg,
                lateral_error_m,
                progress_ratio,
                exit_yaw_error_deg,
            )

        if now < self.direct_step_yaw_recovery_cooldown_until_sec:
            return None

        progress_ok = True
        if progress_ratio is not None:
            min_progress = max(
                0.0,
                min(1.0, float(waypoint.get('direct_step_yaw_recovery_min_progress_ratio', 0.55))),
            )
            max_progress = max(
                min_progress,
                min(1.0, float(waypoint.get('direct_step_yaw_recovery_max_progress_ratio', 0.90))),
            )
            progress_ok = min_progress <= progress_ratio <= max_progress

        lateral_ok = True
        max_lateral_m = waypoint.get('direct_step_yaw_recovery_max_line_lateral_m')
        if max_lateral_m is not None and lateral_error_m is not None:
            lateral_ok = abs(lateral_error_m) <= max(0.0, float(max_lateral_m))

        trigger_ok = abs_yaw_error_deg >= enter_yaw_error_deg and progress_ok and lateral_ok
        if not trigger_ok:
            self.direct_step_yaw_recovery_enter_candidate_start_sec = 0.0
            return None

        if self.direct_step_yaw_recovery_enter_candidate_start_sec <= 0.0:
            self.direct_step_yaw_recovery_enter_candidate_start_sec = now
            return None
        if (now - self.direct_step_yaw_recovery_enter_candidate_start_sec) < enter_hold_sec:
            return None

        self.direct_step_yaw_recovery_active = True
        self.direct_step_yaw_recovery_waypoint_idx = self.current_idx
        self.direct_step_yaw_recovery_active_start_sec = now
        self.direct_step_yaw_recovery_enter_candidate_start_sec = 0.0
        self.direct_step_yaw_recovery_exit_candidate_start_sec = 0.0
        lat_text = 'none' if lateral_error_m is None else f'{lateral_error_m:.3f}m'
        progress_text = 'none' if progress_ratio is None else f'{progress_ratio:.2f}'
        self._publish_feedback(
            f'direct_step yaw recovery 触发: waypoint[{self.current_idx}]={waypoint["name"]}, '
            f'yaw_err={abs_yaw_error_deg:.1f}deg >= {enter_yaw_error_deg:.1f}deg, '
            f'lat={lat_text}, progress={progress_text}。暂停基准线修正，只做趴下姿态回正。'
        )
        return self._direct_step_yaw_recovery_turn_values(
            waypoint,
            yaw_error,
            yaw_error_deg,
            lateral_error_m,
            progress_ratio,
            exit_yaw_error_deg,
        )

    def _direct_step_yaw_recovery_turn_values(
        self,
        waypoint: Dict[str, Any],
        yaw_error: float,
        yaw_error_deg: float,
        lateral_error_m: Optional[float],
        progress_ratio: Optional[float],
        exit_yaw_error_deg: float,
    ) -> tuple:
        mode = int(waypoint.get('direct_step_yaw_recovery_mode', waypoint['direct_step_override_mode']))
        left_norm = float(waypoint.get('direct_step_yaw_recovery_base_left_norm', 0.0))
        right_norm = float(waypoint.get('direct_step_yaw_recovery_base_right_norm', 0.0))
        tolerance_rad = math.radians(max(0.0, exit_yaw_error_deg))
        gain = max(0.0, float(waypoint.get('direct_step_yaw_recovery_gain_per_rad', 3.0)))
        max_delta = max(0.0, float(waypoint.get('direct_step_yaw_recovery_max_delta_norm', 0.25)))
        delta_norm = min(max_delta, gain * max(0.0, abs(yaw_error) - tolerance_rad))
        signed_yaw_error = float(waypoint.get('direct_step_yaw_recovery_sign', 1.0)) * yaw_error
        if signed_yaw_error > 0.0:
            left_norm = self._clamp_norm(left_norm - delta_norm)
            right_norm = self._clamp_norm(right_norm + delta_norm)
        else:
            left_norm = self._clamp_norm(left_norm + delta_norm)
            right_norm = self._clamp_norm(right_norm - delta_norm)
        lat_text = 'none' if lateral_error_m is None else f'{lateral_error_m:.3f}'
        progress_text = 'none' if progress_ratio is None else f'{progress_ratio:.2f}'
        return (
            mode,
            left_norm,
            right_norm,
            (
                f', yaw_recovery=active,yaw_err={yaw_error_deg:.1f}deg,'
                f'turn={delta_norm:.3f},lat={lat_text},progress={progress_text}'
            ),
        )

    def _direct_step_yaw_direction_values(self, waypoint: Dict[str, Any]) -> Optional[tuple]:
        if not bool(waypoint.get('direct_step_yaw_direction_override_enabled', False)):
            return None

        mode = int(waypoint['direct_step_override_mode'])
        if not self.have_pose:
            return mode, 0.0, 0.0, ', yaw_dir=waiting_pose'

        target = waypoint.get('_resolved_target')
        if target is None:
            return mode, 0.0, 0.0, ', yaw_dir=no_target'

        target_yaw = float(target[2])
        yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
        yaw_error_deg = math.degrees(yaw_error)
        tolerance_rad = math.radians(
            max(0.0, float(waypoint.get('direct_step_yaw_direction_tolerance_deg', 3.0)))
        )
        if abs(yaw_error) <= tolerance_rad:
            return mode, 0.0, 0.0, f', yaw_dir=hold,yaw_err={yaw_error_deg:.1f}deg'

        if yaw_error < 0.0:
            left_norm = float(waypoint.get('direct_step_yaw_direction_cw_left_norm', 0.0))
            right_norm = float(waypoint.get('direct_step_yaw_direction_cw_right_norm', 0.0))
            direction = 'cw'
        else:
            left_norm = float(waypoint.get('direct_step_yaw_direction_ccw_left_norm', 0.0))
            right_norm = float(waypoint.get('direct_step_yaw_direction_ccw_right_norm', 0.0))
            direction = 'ccw'

        left_norm = self._clamp_norm(left_norm)
        right_norm = self._clamp_norm(right_norm)
        return (
            mode,
            left_norm,
            right_norm,
            f', yaw_dir={direction},yaw_err={yaw_error_deg:.1f}deg',
        )

    def _direct_step_override_values(self, waypoint: Dict[str, Any]) -> tuple:
        mode = int(waypoint['direct_step_override_mode'])
        left_norm = float(waypoint['direct_step_override_left_norm'])
        right_norm = float(waypoint['direct_step_override_right_norm'])
        debug_parts: List[str] = []

        yaw_direction_values = self._direct_step_yaw_direction_values(waypoint)
        if yaw_direction_values is not None:
            return yaw_direction_values

        yaw_recovery_values = self._direct_step_yaw_recovery_values(waypoint)
        if yaw_recovery_values is not None:
            return yaw_recovery_values

        if bool(waypoint.get('direct_step_yaw_correction_enabled', False)):
            if not self.have_pose:
                debug_parts.append('yaw_corr=waiting_pose')
            else:
                target = waypoint.get('_resolved_target')
                if target is None:
                    debug_parts.append('yaw_corr=no_target')
                else:
                    target_yaw = float(target[2])
                    yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
                    yaw_error_deg = math.degrees(yaw_error)
                    tolerance_rad = math.radians(
                        max(0.0, float(waypoint.get('direct_step_yaw_correction_tolerance_deg', 3.0)))
                    )
                    if abs(yaw_error) <= tolerance_rad:
                        debug_parts.append(f'yaw_corr=0.000,yaw_err={yaw_error_deg:.1f}deg')
                    else:
                        gain = max(0.0, float(waypoint.get('direct_step_yaw_correction_gain_per_rad', 0.25)))
                        max_delta = max(0.0, float(waypoint.get('direct_step_yaw_correction_max_delta_norm', 0.15)))
                        delta_norm = min(max_delta, gain * (abs(yaw_error) - tolerance_rad))
                        signed_yaw_error = float(
                            waypoint.get('direct_step_yaw_correction_sign', 1.0)
                        ) * yaw_error
                        if signed_yaw_error > 0.0:
                            left_norm = self._clamp_norm(left_norm - delta_norm)
                            right_norm = self._clamp_norm(right_norm + delta_norm)
                        else:
                            left_norm = self._clamp_norm(left_norm + delta_norm)
                            right_norm = self._clamp_norm(right_norm - delta_norm)
                        debug_parts.append(f'yaw_corr={delta_norm:.3f},yaw_err={yaw_error_deg:.1f}deg')

        left_norm, right_norm, line_text = self._direct_step_line_lateral_values(
            waypoint,
            left_norm,
            right_norm,
        )
        if line_text:
            debug_parts.append(line_text)

        left_norm, right_norm, tag_text = self._direct_step_apriltag_lateral_values(
            waypoint,
            left_norm,
            right_norm,
        )
        if tag_text:
            debug_parts.append(tag_text)

        max_lr_delta = waypoint.get('direct_step_max_lr_delta_norm')
        if max_lr_delta is not None and float(max_lr_delta) > 0.0:
            clamped_left, clamped_right, clamp_text = self._clamp_step_lr_delta(
                left_norm,
                right_norm,
                float(max_lr_delta),
            )
            left_norm, right_norm = clamped_left, clamped_right
            if clamp_text:
                debug_parts.append(clamp_text)

        left_norm, right_norm, ramp_text = self._direct_step_ramp_values(
            waypoint,
            left_norm,
            right_norm,
        )
        if ramp_text:
            debug_parts.append(ramp_text)

        return (
            mode,
            left_norm,
            right_norm,
            (', ' + ','.join(debug_parts)) if debug_parts else '',
        )

    def _direct_step_ramp_values(
        self,
        waypoint: Dict[str, Any],
        target_left_norm: float,
        target_right_norm: float,
    ) -> Tuple[float, float, str]:
        if not bool(waypoint.get('direct_step_ramp_enabled', False)):
            return target_left_norm, target_right_norm, ''

        duration_sec = max(0.0, float(waypoint.get('direct_step_ramp_duration_sec', 0.0)))
        if duration_sec <= 1e-6 or self.active_goal_stamp <= 0.0:
            return target_left_norm, target_right_norm, 'ramp=done'

        elapsed_sec = max(0.0, self._now_sec() - self.active_goal_stamp)
        alpha = max(0.0, min(1.0, elapsed_sec / duration_sec))
        # Smoothstep curve: slow start, smooth finish, no sudden jump at either end.
        smooth_alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        start_left = waypoint.get('direct_step_ramp_start_left_norm')
        start_right = waypoint.get('direct_step_ramp_start_right_norm')
        if start_left is None:
            start_left = target_left_norm
        if start_right is None:
            start_right = target_right_norm

        left_norm = self._clamp_norm(
            float(start_left) + (float(target_left_norm) - float(start_left)) * smooth_alpha
        )
        right_norm = self._clamp_norm(
            float(start_right) + (float(target_right_norm) - float(start_right)) * smooth_alpha
        )
        return (
            left_norm,
            right_norm,
            f'ramp={smooth_alpha:.2f},elapsed={elapsed_sec:.2f}/{duration_sec:.2f}s',
        )

    def _clamp_step_lr_delta(
        self,
        left_norm: float,
        right_norm: float,
        max_lr_delta_norm: float,
    ) -> Tuple[float, float, str]:
        max_delta = max(0.0, float(max_lr_delta_norm))
        lr_delta = float(right_norm) - float(left_norm)
        if abs(lr_delta) <= max_delta:
            return left_norm, right_norm, ''

        avg = 0.5 * (float(left_norm) + float(right_norm))
        clamped_delta = math.copysign(max_delta, lr_delta)
        clamped_left = self._clamp_norm(avg - 0.5 * clamped_delta)
        clamped_right = self._clamp_norm(avg + 0.5 * clamped_delta)
        return (
            clamped_left,
            clamped_right,
            f'lr_delta_clamp={lr_delta:.3f}->{clamped_delta:.3f}',
        )

    def _direct_step_line_lateral_values(
        self,
        waypoint: Dict[str, Any],
        left_norm: float,
        right_norm: float,
    ) -> Tuple[float, float, str]:
        if not bool(waypoint.get('direct_step_line_lateral_correction_enabled', False)):
            return left_norm, right_norm, ''

        metrics = self._goal_line_metrics(waypoint)
        if metrics is None:
            return left_norm, right_norm, 'line_corr=waiting_line'

        _, _, lateral_error_m, progress_ratio, _ = metrics
        min_progress = max(
            0.0,
            min(1.0, float(waypoint.get('direct_step_line_lateral_min_progress_ratio', 0.15))),
        )
        max_progress = max(
            min_progress,
            min(1.0, float(waypoint.get('direct_step_line_lateral_max_progress_ratio', 0.95))),
        )
        if progress_ratio < min_progress or progress_ratio > max_progress:
            return (
                left_norm,
                right_norm,
                f'line_corr=progress_skip,lat={lateral_error_m:.3f},progress={progress_ratio:.2f}',
            )

        deadband = max(0.0, float(waypoint.get('direct_step_line_lateral_deadband_m', 0.10)))
        abs_error = abs(lateral_error_m)
        if abs_error <= deadband:
            return left_norm, right_norm, f'line_corr=0.000,lat={lateral_error_m:.3f}'

        gain = max(0.0, float(waypoint.get('direct_step_line_lateral_gain_norm_per_m', 0.35)))
        max_delta = max(0.0, float(waypoint.get('direct_step_line_lateral_max_delta_norm', 0.04)))
        sign = float(waypoint.get('direct_step_line_lateral_sign', -1.0))
        delta_norm = min(max_delta, gain * (abs_error - deadband))
        signed_delta = sign * math.copysign(delta_norm, lateral_error_m)

        left_norm = self._clamp_norm(left_norm + signed_delta)
        right_norm = self._clamp_norm(right_norm - signed_delta)
        return (
            left_norm,
            right_norm,
            f'line_corr={signed_delta:.3f},lat={lateral_error_m:.3f},progress={progress_ratio:.2f}',
        )

    def _direct_step_apriltag_lateral_values(
        self,
        waypoint: Dict[str, Any],
        left_norm: float,
        right_norm: float,
    ) -> Tuple[float, float, str]:
        if not bool(waypoint.get('direct_step_apriltag_lateral_correction_enabled', False)):
            return left_norm, right_norm, ''
        if (
            self.apriltag_x_m is None
            or self.apriltag_y_m is None
            or self.apriltag_z_m is None
            or not self._apriltag_latest_transform_matches_waypoint(waypoint)
        ):
            return left_norm, right_norm, 'tag_corr=waiting_tag'

        stale_timeout_sec = max(
            0.0,
            float(waypoint.get('direct_step_apriltag_lateral_stale_timeout_sec', 0.30)),
        )
        age_sec = self._now_sec() - self.apriltag_stamp_sec
        if stale_timeout_sec > 0.0 and age_sec > stale_timeout_sec:
            return left_norm, right_norm, f'tag_corr=stale({age_sec:.2f}s)'

        axis = str(waypoint.get('direct_step_apriltag_lateral_axis', 'x')).strip().lower()
        if axis == 'y':
            measured_m = float(self.apriltag_y_m)
        elif axis == 'z':
            measured_m = float(self.apriltag_z_m)
        else:
            axis = 'x'
            measured_m = float(self.apriltag_x_m)

        target_m = float(waypoint.get('direct_step_apriltag_lateral_target_m', 0.0))
        error_m = measured_m - target_m
        deadband_m = max(
            0.0,
            float(waypoint.get('direct_step_apriltag_lateral_deadband_m', 0.03)),
        )
        if abs(error_m) <= deadband_m:
            return (
                left_norm,
                right_norm,
                f'tag_corr=0.000,tag_{axis}={measured_m:.3f}m',
            )

        gain = max(
            0.0,
            float(waypoint.get('direct_step_apriltag_lateral_gain_norm_per_m', 1.0)),
        )
        max_delta = max(
            0.0,
            float(waypoint.get('direct_step_apriltag_lateral_max_delta_norm', 0.12)),
        )
        delta_norm = min(max_delta, gain * (abs(error_m) - deadband_m))
        signed_error = float(waypoint.get('direct_step_apriltag_lateral_sign', -1.0)) * error_m
        if signed_error > 0.0:
            left_norm = self._clamp_norm(left_norm - delta_norm)
            right_norm = self._clamp_norm(right_norm + delta_norm)
        else:
            left_norm = self._clamp_norm(left_norm + delta_norm)
            right_norm = self._clamp_norm(right_norm - delta_norm)

        signed_delta = delta_norm if signed_error > 0.0 else -delta_norm
        return (
            left_norm,
            right_norm,
            f'tag_corr={signed_delta:.3f},tag_{axis}={measured_m:.3f}m,age={age_sec:.2f}s',
        )

    def _publish_step_override(self, mode: int, left_norm: float, right_norm: float) -> None:
        self.active_step_override_values = [float(mode), float(left_norm), float(right_norm)]
        self._ensure_step_override_refresh_timer()
        msg = Float32MultiArray()
        msg.data = list(self.active_step_override_values)
        self.step_override_pub.publish(msg)
        if self.step_override_pub.get_subscription_count() <= 0:
            self._publish_feedback(
                'serial_step_override 已发布，但当前没有任何订阅者；'
                'cmd_vel_to_serial 可能未启动或已退出。',
                level='error',
            )

    def _publish_step_once(self, mode: int, left_norm: float, right_norm: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(mode), float(left_norm), float(right_norm)]
        self.step_once_pub.publish(msg)

    def _clear_step_override(self) -> None:
        self.active_step_override_values = None
        self.active_step_override_debug_text = ''
        self._clear_step_override_refresh_timer()
        if not rclpy.ok():
            return
        msg = Float32MultiArray()
        msg.data = [-1.0, 0.0, 0.0]
        try:
            self.step_override_pub.publish(msg)
        except Exception:
            pass

    def _active_step_override_progress_text(self) -> str:
        if self.active_step_override_values is None:
            return ''
        mode, left_norm, right_norm = self.active_step_override_values
        return (
            f', override=[{int(mode)},{left_norm:.3f},{right_norm:.3f}]'
            f'{self.active_step_override_debug_text}'
        )

    def _publish_mode(self, mode: int) -> None:
        self.current_mode = int(mode)
        msg = Int32()
        msg.data = int(mode)
        self.mode_pub.publish(msg)
        description = MODE_DESCRIPTIONS.get(mode, '未知模式')
        self.get_logger().info(
            colorize_log(f'发布模式: {mode} ({description})', category='state')
        )

    def _publish_uphill_step_floor_state(self, enabled: bool) -> None:
        self.current_uphill_step_floor_enabled = bool(enabled)
        msg = Bool()
        msg.data = self.current_uphill_step_floor_enabled
        self.uphill_step_floor_state_pub.publish(msg)
        self.get_logger().info(
            colorize_log(
                f'发布上坡步长滤波: {self.current_uphill_step_floor_enabled}',
                category='state',
            )
        )

    def _publish_path_correction_profile(self, profile: int) -> None:
        self.current_path_correction_profile = max(0, min(3, int(profile)))
        msg = Int32()
        msg.data = self.current_path_correction_profile
        self.path_correction_profile_pub.publish(msg)
        self.get_logger().info(
            colorize_log(
                f'发布路径修正profile: {self.current_path_correction_profile} '
                '(0=关闭, 1=普通, 2=上坡强修正, 3=强修正符号验证)',
                category='state',
            )
        )

    def _publish_current_waypoint(self, waypoint: Dict[str, Any]) -> None:
        msg = String()
        msg.data = (
            f'{self.current_idx + 1}/{len(self.waypoints)} {waypoint["name"]} '
            f'target_type={waypoint.get("target_type", "absolute")} '
            f'frame={self._target_frame_for_waypoint(waypoint)} '
            f'next_segment_mode={waypoint["mode_on_arrival"]} '
            f'uphill_step_floor={self.current_uphill_step_floor_enabled} '
            f'path_correction_profile={self.current_path_correction_profile} '
            f'pose_recovery_active={self.pose_recovery_offset_active}'
        )
        self.current_waypoint_pub.publish(msg)

    def _route_end_reached(self) -> bool:
        if self.current_idx >= len(self.waypoints):
            return True
        if self.stop_after_waypoint_index >= 0 and self.current_idx > self.stop_after_waypoint_index:
            return True
        return False

    def _reset_route_runtime(self) -> None:
        self._clear_transition_timer()
        self._clear_post_arrival_mode_sequence()
        self._clear_pose_stability_state()
        self._clear_apriltag_arrival_candidate()
        self._clear_apriltag_arrival_filter()
        self._clear_pose_recovery_offset('流程重置')
        self._clear_resolved_waypoint_targets()
        self._clear_step_override()
        self.current_idx = self.start_waypoint_index
        self.current_goal_retry_count = 0
        self.goal_active = False
        self.route_finished = False
        self.route_failed = False
        self.awaiting_retry = False
        self.active_goal_stamp = 0.0
        self.active_goal_msg = None
        self.active_goal_start_captured = False
        self.active_goal_start_yaw = 0.0
        self.active_goal_min_dist = math.inf
        self.last_executor_state = None
        self.startup_deadline = self._now_sec() + self.startup_delay_sec

    def _activate_manager(self, reason: str, *, log_loaded: bool = False) -> None:
        self.manager_active = True
        self._reset_route_runtime()
        self.last_state = None
        self.last_monitor_state = None
        self._apply_initial_segment_state()
        self._publish_state('READY')
        self._publish_monitor_state('READY')
        if log_loaded:
            self._publish_feedback(
                f'Loaded {len(self.waypoints)} waypoints from {Path(self.waypoints_file).resolve()}'
            )
            self._publish_feedback('当前版本已去除独立 nav_agent，改为 obstacle_manager 直接基于 pose_topic 判定到点。')
            self._publish_feedback('编辑目标点和模式时，优先修改 YAML 文件，不需要改节点代码。')
            self._publish_feedback(
                f'当前模式语义: 到达 waypoint 后切换为该点 mode_on_arrival，并持续到下一个 waypoint。 '
                f'latch_mode_until_next_waypoint={self.latch_mode_until_next_waypoint}'
            )
        self._publish_feedback(
            f'obstacle_manager activated: reason={reason}, '
            f'start_waypoint_index={self.start_waypoint_index}, '
            f'stop_after_waypoint_index={self.stop_after_waypoint_index}, '
            f'inherit_previous_waypoint_segment_state={self.inherit_previous_waypoint_segment_state}'
        )

    def _apply_initial_segment_state(self) -> None:
        if not self.inherit_previous_waypoint_segment_state or self.start_waypoint_index <= 0:
            self._publish_mode(self.idle_mode)
            self._publish_uphill_step_floor_state(False)
            self._publish_path_correction_profile(1)
            return

        previous_waypoint = self.waypoints[self.start_waypoint_index - 1]
        inherited_mode = int(previous_waypoint['mode_on_arrival'])
        inherited_uphill = bool(previous_waypoint.get('uphill_step_floor_on_arrival', False))
        inherited_profile = int(previous_waypoint.get('path_correction_profile_on_arrival', 1))
        mode_source = f'previous:{previous_waypoint["name"]}'
        if self.inherited_segment_mode_override >= 0:
            inherited_mode = self.inherited_segment_mode_override
            mode_source = 'inherited_segment_mode_override'

        self._publish_mode(inherited_mode)
        self._publish_uphill_step_floor_state(inherited_uphill)
        self._publish_path_correction_profile(inherited_profile)
        self._publish_feedback(
            f'启动时继承上一 waypoint 段状态: source={previous_waypoint["name"]}, '
            f'mode={inherited_mode}, uphill_step_floor={inherited_uphill}, '
            f'path_correction_profile={inherited_profile}, mode_source={mode_source}'
        )

    def _deactivate_manager(self, reason: str) -> None:
        self.manager_active = False
        self.goal_active = False
        self._clear_transition_timer()
        self._clear_post_arrival_mode_sequence()
        self._clear_pose_stability_state()
        self._clear_pose_recovery_offset('流程停用')
        self._clear_step_override()
        self.last_state = None
        self.last_monitor_state = None
        self._publish_state('INACTIVE')
        self._publish_monitor_state('INACTIVE')
        self._publish_feedback(f'obstacle_manager deactivated: reason={reason}')

    def _active_callback(self, msg: Bool) -> None:
        requested = bool(msg.data)
        if requested == self.manager_active:
            return
        if requested:
            self.start_waypoint_index = self.default_start_waypoint_index
            self._activate_manager('external_topic')
            return
        self._deactivate_manager('external_topic')

    def _restart_from_waypoint_callback(self, msg: String) -> None:
        name = str(msg.data).strip()
        if not name:
            self._publish_feedback('收到空 waypoint 重启请求，已忽略。', level='warn')
            return
        index = self._waypoint_index_by_name(name)
        if index is None:
            self._publish_feedback(
                f'收到 waypoint 重启请求但找不到目标: name={name}',
                level='warn',
            )
            return
        if self.stop_after_waypoint_index >= 0 and index > self.stop_after_waypoint_index:
            self._publish_feedback(
                f'收到 waypoint 重启请求但目标超过 stop_after_waypoint_index: '
                f'name={name}, index={index}, stop={self.stop_after_waypoint_index}',
                level='warn',
            )
            return

        self.start_waypoint_index = index
        self.manager_active = True
        self._reset_route_runtime()
        self.last_state = None
        self.last_monitor_state = None
        self._apply_initial_segment_state()
        self._publish_state('READY')
        self._publish_monitor_state('READY')
        self._publish_feedback(
            f'按外部请求从 waypoint[{index}]={name} 重新启动流程。'
        )

    def _waypoint_index_by_name(self, name: str) -> Optional[int]:
        for index, waypoint in enumerate(self.waypoints):
            if str(waypoint.get('name', '')).strip() == name:
                return index
        return None

    def _publish_state(self, state: str) -> None:
        if state == self.last_state:
            return
        self.last_state = state
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def _publish_monitor_state(self, state: str) -> None:
        if state == self.last_monitor_state:
            return
        self.last_monitor_state = state
        msg = String()
        msg.data = state
        self.monitor_state_pub.publish(msg)

    def _publish_feedback(self, text: str, level: str = 'info', log_to_console: bool = True) -> None:
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)
        monitor_msg = String()
        monitor_msg.data = text
        self.monitor_feedback_pub.publish(monitor_msg)

        if not log_to_console:
            return
        styled_text = colorize_log(text, level=level)
        if level == 'warn':
            self.get_logger().warn(styled_text)
        elif level == 'error':
            self.get_logger().error(styled_text)
        else:
            self.get_logger().info(styled_text)

    def _publish_progress(self, text: str, force_console_every: Optional[float] = 1.0) -> None:
        msg = String()
        msg.data = text
        self.monitor_feedback_pub.publish(msg)
        self.feedback_pub.publish(msg)

        now = self._now_sec()
        if force_console_every is None:
            return
        if (now - self.last_progress_log_time) >= max(force_console_every, self.progress_log_interval_sec):
            self.last_progress_log_time = now
            self.get_logger().info(colorize_log(text, category='debug'))

    def _pose_is_stale(self, now_sec: float) -> bool:
        if self.last_pose_time is None or self.pose_stale_timeout_sec <= 0.0:
            return False
        age = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if age <= self.pose_stale_timeout_sec:
            return False
        if (now_sec - self.last_pose_stale_warn_time) >= 2.0:
            self.last_pose_stale_warn_time = now_sec
            self._publish_feedback(
                f'定位数据超时: pose_age={age:.2f}s，当前暂停到点判定。',
                level='warn',
            )
        return True

    def _capture_goal_start_pose(self) -> None:
        if self.active_goal_start_captured or not self.have_pose:
            return
        self.active_goal_start_x = self.pose_x
        self.active_goal_start_y = self.pose_y
        self.active_goal_start_yaw = self.pose_yaw
        self.active_goal_start_captured = True

    def _goal_line_metrics(self, waypoint: Dict[str, Any]) -> Optional[Tuple[float, float, float, float, float]]:
        if not self.active_goal_start_captured:
            return None
        target_x, target_y, _ = self._target_pose_for_waypoint(waypoint)
        seg_x = target_x - self.active_goal_start_x
        seg_y = target_y - self.active_goal_start_y
        seg_len = math.hypot(seg_x, seg_y)
        if seg_len <= 1e-6:
            return None

        rel_x = self.pose_x - self.active_goal_start_x
        rel_y = self.pose_y - self.active_goal_start_y
        progress_m = (rel_x * seg_x + rel_y * seg_y) / seg_len
        remaining_m = seg_len - progress_m
        lateral_error_m = (rel_x * seg_y - rel_y * seg_x) / seg_len
        progress_ratio = progress_m / seg_len
        return progress_m, remaining_m, lateral_error_m, progress_ratio, seg_len

    def _goal_line_progress_text(self, waypoint: Dict[str, Any]) -> str:
        metrics = self._goal_line_metrics(waypoint)
        if metrics is None:
            return ''
        progress_m, remaining_m, lateral_error_m, progress_ratio, seg_len = metrics
        return (
            f', line_progress={progress_m:.3f}/{seg_len:.3f}m '
            f'({progress_ratio * 100.0:.0f}%), '
            f'line_remaining={remaining_m:.3f}m, '
            f'line_lateral={lateral_error_m:.3f}m'
        )

    def _travel_arrival_distance_threshold(self, waypoint: Dict[str, Any]) -> Optional[float]:
        configured = waypoint.get('travel_distance_arrival_m')
        if configured is not None:
            return max(0.0, float(configured))
        if self._waypoint_uses_body_relative_target(waypoint):
            return math.hypot(
                float(waypoint.get('_resolved_body_forward_m', waypoint.get('body_forward_m', 0.0))),
                float(waypoint.get('_resolved_body_left_m', waypoint.get('body_left_m', 0.0))),
            )
        return None

    def _travel_arrival_yaw_threshold(self, waypoint: Dict[str, Any]) -> Optional[float]:
        configured = waypoint.get('travel_yaw_arrival_deg')
        if configured is not None:
            return math.radians(max(0.0, abs(float(configured))))
        if self._waypoint_uses_body_relative_target(waypoint):
            return math.radians(abs(float(waypoint.get('body_yaw_delta_deg', 0.0))))
        return None

    def _travel_arrival_progress(self, waypoint: Dict[str, Any]) -> Tuple[float, float]:
        if not self.active_goal_start_captured:
            return 0.0, 0.0
        travel_m = math.hypot(
            self.pose_x - self.active_goal_start_x,
            self.pose_y - self.active_goal_start_y,
        )
        yaw_travel_rad = self._normalize_angle(self.pose_yaw - self.active_goal_start_yaw)
        return travel_m, yaw_travel_rad

    def _travel_arrival_progress_text(self, waypoint: Dict[str, Any]) -> str:
        if not (
            bool(waypoint.get('travel_distance_arrival_enabled', False))
            or bool(waypoint.get('travel_yaw_arrival_enabled', False))
        ):
            return ''
        travel_m, yaw_travel_rad = self._travel_arrival_progress(waypoint)
        parts: List[str] = []
        distance_threshold = self._travel_arrival_distance_threshold(waypoint)
        if bool(waypoint.get('travel_distance_arrival_enabled', False)) and distance_threshold is not None:
            parts.append(f'travel={travel_m:.3f}/{distance_threshold:.3f}m')
        yaw_threshold = self._travel_arrival_yaw_threshold(waypoint)
        if bool(waypoint.get('travel_yaw_arrival_enabled', False)) and yaw_threshold is not None:
            parts.append(
                f'yaw_travel={math.degrees(yaw_travel_rad):.1f}/'
                f'{math.degrees(yaw_threshold):.1f}deg'
            )
        if not parts:
            return ''
        return ', ' + ', '.join(parts)

    def _travel_arrival_reason(self, waypoint: Dict[str, Any]) -> Optional[str]:
        if not self.active_goal_start_captured:
            return None
        travel_m, yaw_travel_rad = self._travel_arrival_progress(waypoint)

        if bool(waypoint.get('travel_distance_arrival_enabled', False)):
            threshold_m = self._travel_arrival_distance_threshold(waypoint)
            if threshold_m is not None and travel_m >= threshold_m:
                return f'travel_distance({travel_m:.3f}>={threshold_m:.3f}m)'

        if bool(waypoint.get('travel_yaw_arrival_enabled', False)):
            threshold_rad = self._travel_arrival_yaw_threshold(waypoint)
            if threshold_rad is None:
                return None
            directional = bool(waypoint.get('travel_yaw_arrival_directional', True))
            if directional:
                target_delta_rad = math.radians(float(waypoint.get('body_yaw_delta_deg', 0.0)))
                if target_delta_rad < 0.0 and yaw_travel_rad <= -threshold_rad:
                    return (
                        f'travel_yaw({math.degrees(yaw_travel_rad):.1f}<='
                        f'-{math.degrees(threshold_rad):.1f}deg)'
                    )
                if target_delta_rad >= 0.0 and yaw_travel_rad >= threshold_rad:
                    return (
                        f'travel_yaw({math.degrees(yaw_travel_rad):.1f}>='
                        f'{math.degrees(threshold_rad):.1f}deg)'
                    )
            elif abs(yaw_travel_rad) >= threshold_rad:
                return (
                    f'travel_yaw_abs({abs(math.degrees(yaw_travel_rad)):.1f}>='
                    f'{math.degrees(threshold_rad):.1f}deg)'
                )

        return None

    def _goal_pass_through_detected(self, waypoint: Dict[str, Any]) -> bool:
        return self._goal_pass_through_reason(waypoint) is not None

    def _goal_pass_through_reason(self, waypoint: Dict[str, Any]) -> Optional[str]:
        if not bool(waypoint['allow_goal_pass_through']):
            return None
        metrics = self._goal_line_metrics(waypoint)
        if metrics is None:
            return None

        _, _, lateral_error_m, progress_ratio, _ = metrics
        if progress_ratio < 1.0:
            return None

        force_progress_ratio = waypoint.get('pass_through_force_progress_ratio')
        if force_progress_ratio is not None:
            force_progress_ratio = max(1.0, float(force_progress_ratio))
            if progress_ratio >= force_progress_ratio:
                return (
                    f'pass_through_force_progress(progress={progress_ratio:.2f}>='
                    f'{force_progress_ratio:.2f}, lateral={lateral_error_m:.3f}m)'
                )

        lateral_tolerance = max(
            float(waypoint['arrival_tolerance']),
            float(waypoint['pass_through_lateral_tolerance']),
        )
        if abs(lateral_error_m) <= lateral_tolerance:
            return 'pass_through'
        return None

    def _build_goal_msg(self, waypoint: Dict[str, Any]) -> PoseStamped:
        target_x, target_y, target_yaw = self._target_pose_for_waypoint(waypoint)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self._target_frame_for_waypoint(waypoint)
        goal.pose.position.x = target_x
        goal.pose.position.y = target_y
        goal.pose.position.z = 0.0
        goal.pose.orientation = self._quaternion_from_yaw(target_yaw)
        return goal

    @staticmethod
    def _quaternion_from_yaw_deg(yaw_deg: float) -> Quaternion:
        return ObstacleManager._quaternion_from_yaw(math.radians(yaw_deg))

    @staticmethod
    def _quaternion_from_yaw(yaw_rad: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw_rad / 2.0)
        q.w = math.cos(yaw_rad / 2.0)
        return q

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def _rpy_from_quaternion(q) -> Tuple[float, float, float]:
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        yaw = ObstacleManager._yaw_from_quaternion(q)
        return roll, pitch, yaw

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp_norm(value: float) -> float:
        return max(-1.0, min(1.0, float(value)))

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def destroy_node(self) -> bool:
        self._clear_transition_timer()
        self._clear_step_override()
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        if rclpy.ok() and 'Unable to convert call argument' not in str(exc):
            raise
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
