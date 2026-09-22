#!/usr/bin/env python3
"""Relative waypoint route triggered by the lightweight HSV orange-pole detector.

2026-05-03 01:40 CST:
Reason: connect the CPU-light D435 HSV pole detector to the existing six-goal
relative navigation flow without changing the YOLO route node.
Rollback: launch yolo_relative_nav_node instead, or disable this node in the
orange_pole_relative_nav_bringup launch.
"""

import math
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

from apriltag_msgs.msg import AprilTagDetectionArray
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32MultiArray, Int32, String
from tf2_msgs.msg import TFMessage

from auto_nav_pkg.yaw_rate_estimator import WindowedYawRateEstimator


@dataclass
class PoleSnapshot:
    distance_m: float
    lateral_m: float
    vertical_m: float
    pixel_x: float
    pixel_y: float
    width_px: float
    height_px: float
    area_px: float
    valid_depth_count: int


@dataclass
class AprilTagSnapshot:
    frame_id: str
    child_frame_id: str
    stamp_sec: float
    received_sec: float
    x_m: float
    y_m: float
    z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float


@dataclass
class AprilTagDetectionInfo:
    stamp_sec: float = 0.0
    received_sec: float = 0.0
    raw_count: int = 0
    matched: bool = False
    selected_family: str = ''
    selected_id: int = -1
    decision_margin: float = math.nan
    hamming: int = -1
    centre_x: float = math.nan
    centre_y: float = math.nan
    decoded_tags: str = ''


class OrangePoleRelativeNav(Node):
    """Start direct walking, then run the measured route when HSV sees a pole."""

    def __init__(self) -> None:
        super().__init__('orange_pole_relative_nav')

        self.declare_parameter('nearest_topic', '/orange_pole_detector/nearest')
        self.declare_parameter('trigger_topic', '/orange_pole_detector/trigger')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('arrival_status_topic', '/agent/arrival_status')
        self.declare_parameter('step_override_topic', '/serial_step_override')
        self.declare_parameter('mode_topic', '/dog_mode_current')
        self.declare_parameter(
            'fixed_step_override_state_topic',
            '/motion_bridge/fixed_step_override_enabled',
        )
        self.declare_parameter('state_topic', '/orange_pole_relative_nav/state')
        self.declare_parameter('feedback_topic', '/orange_pole_relative_nav/feedback_log')
        self.declare_parameter('active', True)
        self.declare_parameter('active_topic', '/orange_pole_relative_nav/active')

        self.declare_parameter('pose_topic', '/Odometry')
        self.declare_parameter('pose_topic_type', 'odometry')
        self.declare_parameter('default_frame_id', 'camera_init')
        self.declare_parameter('pose_stale_timeout_sec', 1.2)
        self.declare_parameter('runtime_route_yaw_reference_enabled', False)
        self.declare_parameter(
            'route_yaw_reference_topic',
            '/slope_route_yaw_reference',
        )
        self.declare_parameter('route_yaw_reference_timeout_sec', 0.0)

        self.declare_parameter('use_trigger_topic', True)
        self.declare_parameter('trigger_from_nearest_topic', True)
        self.declare_parameter('trigger_source', 'hsv')
        self.declare_parameter('trigger_distance_m', 1.25)
        self.declare_parameter('nearest_stale_timeout_sec', 1.0)
        self.declare_parameter('max_lateral_abs_m', 0.0)
        self.declare_parameter('min_valid_depth_pixels', 1)

        self.declare_parameter('apriltag_tf_topic', '/tf')
        self.declare_parameter('apriltag_detections_topic', '/apriltag/detections')
        self.declare_parameter('apriltag_target_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('apriltag_target_child_frame_id', 'object')
        self.declare_parameter('apriltag_target_tag_id', 14)
        self.declare_parameter('apriltag_stale_timeout_sec', 0.30)
        self.declare_parameter('apriltag_trigger_require_tf', True)

        self.declare_parameter('pole_pre_align_enabled', False)
        self.declare_parameter('pole_pre_align_mode', 6)
        self.declare_parameter('pole_pre_align_angle_metric', 'pose_pitch')
        self.declare_parameter('pole_pre_align_target_pitch_deg', 0.0)
        self.declare_parameter('pole_pre_align_fixed_yaw_deg', 0.0)
        self.declare_parameter('pole_pre_align_angle_tolerance_deg', 1.0)
        self.declare_parameter('pole_pre_align_angle_exit_tolerance_deg', 0.0)
        self.declare_parameter('pole_pre_align_angle_hold_sec', 0.25)
        self.declare_parameter('pole_pre_align_pose_filter_window_size', 1)
        self.declare_parameter('pole_pre_align_pose_filter_min_samples', 1)
        self.declare_parameter('pole_pre_align_reverse_cooldown_sec', 0.0)
        self.declare_parameter('pole_pre_align_pose_control_sign', -1.0)
        self.declare_parameter('pole_pre_align_timeout_sec', 8.0)
        self.declare_parameter('pole_pre_align_total_timeout_sec', 24.0)
        self.declare_parameter('pole_pre_align_lateral_tolerance_m', 0.025)
        self.declare_parameter('pole_pre_align_camera_left_offset_m', 0.0)
        self.declare_parameter('pole_pre_align_lateral_control_sign', 1.0)
        self.declare_parameter('pole_pre_align_shift_yaw_deg', 90.0)
        self.declare_parameter('pole_pre_align_shift_distance_scale', 1.0)
        self.declare_parameter('pole_pre_align_shift_max_m', 0.30)
        self.declare_parameter('pole_pre_align_shift_deadband_m', 0.02)
        self.declare_parameter('pole_pre_align_shift_rebase_use_turn_drift', True)
        self.declare_parameter('pole_pre_align_reuse_existing_turn_steps', True)
        self.declare_parameter('pole_pre_align_reuse_existing_small_move_steps', True)
        self.declare_parameter('pole_pre_align_turn_mode', 6)
        self.declare_parameter('pole_pre_align_pose_turn_scale', 1.0)
        self.declare_parameter('pole_pre_align_ccw_left_norm', -0.10)
        self.declare_parameter('pole_pre_align_ccw_right_norm', 0.18)
        self.declare_parameter('pole_pre_align_cw_left_norm', 0.18)
        self.declare_parameter('pole_pre_align_cw_right_norm', -0.10)
        self.declare_parameter('pole_pre_align_turn_scale_enabled', True)
        self.declare_parameter('pole_pre_align_turn_min_scale', 0.35)
        self.declare_parameter('pole_pre_align_turn_yaw_gain_per_rad', 0.65)
        self.declare_parameter('pole_pre_align_turn_min_abs_angular_z', 0.035)
        self.declare_parameter('pole_pre_align_turn_max_abs_angular_z', 0.18)
        self.declare_parameter('pole_pre_align_precision_turn_enabled', False)
        self.declare_parameter('pole_pre_align_turn_far_error_deg', 35.0)
        self.declare_parameter('pole_pre_align_turn_far_scale', 1.35)
        self.declare_parameter('pole_pre_align_turn_yaw_damping_sec', 0.18)
        self.declare_parameter('pole_pre_align_turn_brake_yaw_rate_deg_s', 12.0)
        self.declare_parameter('pole_pre_align_turn_complete_yaw_rate_deg_s', 6.0)
        self.declare_parameter('pole_pre_align_turn_yaw_rate_filter_alpha', 1.0)
        self.declare_parameter('pole_pre_align_turn_yaw_rate_window_sec', 0.30)
        self.declare_parameter('pole_pre_align_turn_yaw_rate_min_span_sec', 0.18)
        self.declare_parameter('pole_pre_align_turn_yaw_rate_max_abs_deg_s', 90.0)
        self.declare_parameter('pole_pre_align_fine_angle_threshold_deg', 3.0)
        self.declare_parameter('pole_pre_align_fine_ccw_left_norm', -0.04)
        self.declare_parameter('pole_pre_align_fine_ccw_right_norm', 0.08)
        self.declare_parameter('pole_pre_align_fine_cw_left_norm', 0.08)
        self.declare_parameter('pole_pre_align_fine_cw_right_norm', -0.04)
        self.declare_parameter('pole_pre_align_walk_mode', 0)
        self.declare_parameter('pole_pre_align_walk_left_norm', 0.30)
        self.declare_parameter('pole_pre_align_walk_right_norm', 0.30)
        self.declare_parameter('pole_pre_align_walk_yaw_correction_enabled', True)
        self.declare_parameter('pole_pre_align_walk_yaw_deadband_deg', 1.0)
        self.declare_parameter('pole_pre_align_walk_yaw_gain_norm_per_rad', 0.90)
        self.declare_parameter('pole_pre_align_walk_yaw_max_delta_norm', 0.15)
        self.declare_parameter('pole_pre_align_stop_hold_sec', 0.10)
        self.declare_parameter('pole_pre_align_backup_enabled', True)
        self.declare_parameter('pole_pre_align_backup_mode', 0)
        self.declare_parameter('pole_pre_align_backup_left_norm', -0.25)
        self.declare_parameter('pole_pre_align_backup_right_norm', -0.25)
        self.declare_parameter('pole_pre_align_backup_distance_m', 0.08)
        self.declare_parameter('pole_pre_align_backup_max_retries', 1)
        self.declare_parameter('pole_pre_align_min_tag_z_m', 0.25)
        self.declare_parameter('pole_pre_align_entry_tag_z_m', 0.0)
        self.declare_parameter('pole_pre_align_entry_tag_z_tolerance_m', 0.03)
        self.declare_parameter('pole_pre_align_entry_forward_enabled', False)
        self.declare_parameter('pole_pre_align_entry_forward_mode', 0)
        self.declare_parameter('pole_pre_align_entry_forward_left_norm', 0.30)
        self.declare_parameter('pole_pre_align_entry_forward_right_norm', 0.30)
        self.declare_parameter('pole_pre_align_entry_forward_max_m', 0.40)
        self.declare_parameter('pole_pre_align_verify_require_tag', True)
        self.declare_parameter('pole_pre_align_verify_timeout_sec', 1.5)

        self.declare_parameter('startup_step_override_enabled', True)
        self.declare_parameter('startup_step_override_mode', 0)
        self.declare_parameter('startup_left_norm', 1.0)
        self.declare_parameter('startup_right_norm', 1.0)
        self.declare_parameter('startup_hold_when_near_enabled', False)
        self.declare_parameter('startup_hold_when_near_distance_m', 0.0)
        self.declare_parameter('startup_hold_when_near_lateral_abs_m', 0.0)
        self.declare_parameter('startup_ramp_duration_sec', 0.0)
        self.declare_parameter('startup_override_rate_hz', 10.0)
        self.declare_parameter('startup_route_handoff_enabled', False)
        self.declare_parameter('startup_route_handoff_cmd_vel_topic', '/race_cmd_vel')
        self.declare_parameter('startup_route_handoff_max_hold_sec', 0.25)
        self.declare_parameter('startup_route_handoff_min_linear_speed_mps', 0.02)
        self.declare_parameter('startup_route_handoff_min_angular_speed_rps', 0.05)
        self.declare_parameter('startup_route_handoff_republish_goal_on_timeout', True)
        self.declare_parameter('startup_route_handoff_max_republish_attempts', 2)
        self.declare_parameter('publish_mode_at_start', True)
        self.declare_parameter('route_mode', 6)
        self.declare_parameter('forward_route_mode', -1)
        self.declare_parameter('first_forward_route_mode', -1)
        self.declare_parameter('turn_route_mode', -1)
        self.declare_parameter('route_mode_refresh_rate_hz', 5.0)
        self.declare_parameter('disable_fixed_step_override_during_route', True)
        self.declare_parameter('finish_mode', 0)

        self.declare_parameter(
            'waypoint_offsets_xy',
            [0.7, 1.0, -1.4, 1.0, 0.0, 1.0, 0.7, 1.0, -0.7, 1.0, -0.7, 1.0],
        )
        # 2026-05-03 02:14 CST: declare empty array parameters with explicit
        # ROS types. In Humble, a bare [] is inferred as BYTE_ARRAY and rejects
        # YAML DOUBLE_ARRAY/BOOL_ARRAY overrides. Rollback: replace these with
        # concrete default arrays only if all route lengths become fixed again.
        self.declare_parameter('waypoint_yaws_deg', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('waypoint_use_yaw_tolerance', Parameter.Type.BOOL_ARRAY)
        self.declare_parameter('yaw_tolerance_deg', 6.0)
        self.declare_parameter('waypoint_use_direct_step_override', Parameter.Type.BOOL_ARRAY)
        self.declare_parameter('waypoint_direct_step_left_norms', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('waypoint_direct_step_right_norms', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('waypoint_use_forward_step_override', Parameter.Type.BOOL_ARRAY)
        self.declare_parameter('waypoint_forward_step_left_norms', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('waypoint_forward_step_right_norms', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('turn_waypoint_use_motion_primitive', True)
        self.declare_parameter('turn_waypoint_arc_inner_norm', 0.20)
        self.declare_parameter('turn_waypoint_arc_outer_norm', 0.42)
        self.declare_parameter('turn_waypoint_min_duration_sec', 0.35)
        self.declare_parameter('turn_waypoint_completion_yaw_tolerance_deg', 6.0)
        self.declare_parameter('turn_waypoint_completion_progress_tolerance_deg', 3.0)
        # 2026-06-28 11:18 CST
        # Reason: field logs showed in-place pole turns can miss a narrow yaw
        # completion window, then keep spinning in the same direction after the
        # accumulated progress already passed the target.
        # Purpose: expose optional pole-route-only overrun guards and a slow
        # final turn band; defaults stay disabled to preserve existing behavior.
        # Effect: YAML route files can enable these guards locally. Rollback:
        # leave enabled=false or remove the YAML overrides.
        self.declare_parameter('turn_waypoint_overshoot_guard_enabled', False)
        self.declare_parameter('turn_waypoint_overshoot_tolerance_deg', 5.0)
        self.declare_parameter('turn_waypoint_overshoot_accept_yaw_tolerance_deg', 12.0)
        self.declare_parameter('turn_waypoint_slowdown_enabled', False)
        self.declare_parameter('turn_waypoint_slowdown_remaining_deg', 15.0)
        self.declare_parameter('turn_waypoint_slowdown_cw_left_norm', 0.35)
        self.declare_parameter('turn_waypoint_slowdown_cw_right_norm', -0.10)
        self.declare_parameter('turn_waypoint_slowdown_ccw_left_norm', -0.10)
        self.declare_parameter('turn_waypoint_slowdown_ccw_right_norm', 0.35)
        self.declare_parameter('turn_waypoint_longitudinal_tolerance_m', 0.08)
        self.declare_parameter('turn_waypoint_lateral_tolerance_m', 0.18)
        self.declare_parameter('turn_waypoint_exit_forward_norm', 0.24)
        self.declare_parameter('turn_waypoint_exit_forward_min_duration_sec', 0.12)
        self.declare_parameter('turn_waypoint_exit_forward_timeout_sec', 1.5)
        self.declare_parameter('turn_waypoint_timeout_sec', 6.0)
        self.declare_parameter('turn_waypoint_simple_in_place_enabled', False)
        self.declare_parameter('direct_step_start_distance_m', 0.35)
        self.declare_parameter('direct_step_recover_distance_m', 0.50)
        self.declare_parameter('body_relative_waypoint_indices', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('body_relative_waypoint_offsets_fl', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('turn_waypoint_yaw_mode', 'global_waypoint')
        self.declare_parameter('body_relative_turn_waypoint_indices', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('body_relative_turn_deltas_deg', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('turn_waypoint_pole_distance_gate_indices', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('turn_waypoint_pole_distance_gate_thresholds_m', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('route_reference_mode', 'route_anchor')
        self.declare_parameter('forward_tracking_enabled', False)
        self.declare_parameter('forward_tracking_base_left_norm', 0.50)
        self.declare_parameter('forward_tracking_base_right_norm', 0.50)
        self.declare_parameter('forward_tracking_lateral_gain', 1.2)
        self.declare_parameter('forward_tracking_lateral_gain_waypoint_indices', Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('forward_tracking_lateral_gain_waypoint_values', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('forward_tracking_yaw_gain', 0.45)
        self.declare_parameter('forward_tracking_deadband_m', 0.02)
        self.declare_parameter('forward_tracking_max_delta_norm', 0.30)
        self.declare_parameter('forward_tracking_lateral_fail_m', 0.30)
        self.declare_parameter(
            'forward_tracking_arrival_lateral_tolerance_waypoint_indices',
            Parameter.Type.INTEGER_ARRAY,
        )
        self.declare_parameter(
            'forward_tracking_arrival_lateral_tolerance_waypoint_values',
            Parameter.Type.DOUBLE_ARRAY,
        )
        self.declare_parameter('forward_tracking_yaw_fail_deg', 18.0)
        self.declare_parameter('forward_tracking_timeout_sec_per_m', 8.0)
        self.declare_parameter('forward_tracking_timeout_extra_sec', 3.0)
        self.declare_parameter('goal_yaw_mode', 'current')
        self.declare_parameter('first_forward_goal_yaw_mode', 'inherit')
        self.declare_parameter('arrival_tolerance', 0.25)
        self.declare_parameter('waypoint_arrival_tolerances', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('forward_waypoint_pass_through_enabled', True)
        self.declare_parameter('forward_waypoint_pass_through_lateral_tolerance_m', 0.12)
        self.declare_parameter('forward_waypoint_pass_through_longitudinal_tolerance_m', 0.24)
        self.declare_parameter('require_fresh_pole_for_route_start', True)
        self.declare_parameter('goal_timeout_sec', 45.0)
        self.declare_parameter('next_goal_delay_sec', 0.20)
        self.declare_parameter('turn_settle_sec', 0.0)
        self.declare_parameter('turn_settle_yaw_tolerance_deg', 0.0)
        self.declare_parameter('turn_settle_yaw_unstable_fail_enabled', True)
        self.declare_parameter('monitor_rate_hz', 10.0)
        self.declare_parameter('progress_log_interval_sec', 1.0)
        self.declare_parameter('status_heartbeat_sec', 2.0)

        self.nearest_topic = str(self.get_parameter('nearest_topic').value)
        self.trigger_topic = str(self.get_parameter('trigger_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.arrival_status_topic = str(self.get_parameter('arrival_status_topic').value)
        self.step_override_topic = str(self.get_parameter('step_override_topic').value)
        self.mode_topic = str(self.get_parameter('mode_topic').value)
        self.fixed_step_override_state_topic = str(
            self.get_parameter('fixed_step_override_state_topic').value
        )
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.feedback_topic = str(self.get_parameter('feedback_topic').value)
        self.route_controller_active = bool(self.get_parameter('active').value)
        self.active_topic = str(self.get_parameter('active_topic').value)

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.pose_topic_type = str(self.get_parameter('pose_topic_type').value).strip().lower()
        self.default_frame_id = str(self.get_parameter('default_frame_id').value)
        self.pose_stale_timeout_sec = float(self.get_parameter('pose_stale_timeout_sec').value)
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

        self.use_trigger_topic = bool(self.get_parameter('use_trigger_topic').value)
        self.trigger_from_nearest_topic = bool(
            self.get_parameter('trigger_from_nearest_topic').value
        )
        self.trigger_source = str(self.get_parameter('trigger_source').value).strip().lower()
        if self.trigger_source not in ('hsv', 'apriltag', 'both'):
            raise RuntimeError('trigger_source must be one of: hsv, apriltag, both.')
        self.trigger_distance_m = float(self.get_parameter('trigger_distance_m').value)
        self.nearest_stale_timeout_sec = float(
            self.get_parameter('nearest_stale_timeout_sec').value
        )
        self.max_lateral_abs_m = max(0.0, float(self.get_parameter('max_lateral_abs_m').value))
        self.min_valid_depth_pixels = max(1, int(self.get_parameter('min_valid_depth_pixels').value))

        self.apriltag_tf_topic = str(self.get_parameter('apriltag_tf_topic').value)
        self.apriltag_detections_topic = str(
            self.get_parameter('apriltag_detections_topic').value
        )
        self.apriltag_target_frame_id = str(
            self.get_parameter('apriltag_target_frame_id').value
        )
        self.apriltag_target_child_frame_id = str(
            self.get_parameter('apriltag_target_child_frame_id').value
        )
        self.apriltag_target_tag_id = int(self.get_parameter('apriltag_target_tag_id').value)
        self.apriltag_stale_timeout_sec = max(
            0.0,
            float(self.get_parameter('apriltag_stale_timeout_sec').value),
        )
        self.apriltag_trigger_require_tf = bool(
            self.get_parameter('apriltag_trigger_require_tf').value
        )
        self.pole_pre_align_enabled = bool(
            self.get_parameter('pole_pre_align_enabled').value
        )
        self.pole_pre_align_mode = int(self.get_parameter('pole_pre_align_mode').value)
        self.pole_pre_align_angle_metric = str(
            self.get_parameter('pole_pre_align_angle_metric').value
        ).strip().lower()
        if self.pole_pre_align_angle_metric not in (
            'pose_pitch',
            'fixed_absolute_yaw',
        ):
            raise RuntimeError(
                'pole_pre_align_angle_metric must be pose_pitch or fixed_absolute_yaw.'
            )
        self.pole_pre_align_target_pitch_rad = math.radians(
            float(self.get_parameter('pole_pre_align_target_pitch_deg').value)
        )
        self.pole_pre_align_fixed_yaw_rad = self._normalize_angle(
            math.radians(
                float(self.get_parameter('pole_pre_align_fixed_yaw_deg').value)
            )
        )
        self.pole_pre_align_angle_tolerance_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('pole_pre_align_angle_tolerance_deg').value),
            )
        )
        angle_exit_tolerance_deg = float(
            self.get_parameter('pole_pre_align_angle_exit_tolerance_deg').value
        )
        if angle_exit_tolerance_deg <= 0.0:
            self.pole_pre_align_angle_exit_tolerance_rad = (
                self.pole_pre_align_angle_tolerance_rad
            )
        else:
            self.pole_pre_align_angle_exit_tolerance_rad = max(
                self.pole_pre_align_angle_tolerance_rad,
                math.radians(angle_exit_tolerance_deg),
            )
        self.pole_pre_align_angle_hold_sec = max(
            0.0,
            float(self.get_parameter('pole_pre_align_angle_hold_sec').value),
        )
        self.pole_pre_align_pose_filter_window_size = max(
            1,
            int(self.get_parameter('pole_pre_align_pose_filter_window_size').value),
        )
        self.pole_pre_align_pose_filter_min_samples = max(
            1,
            min(
                self.pole_pre_align_pose_filter_window_size,
                int(self.get_parameter('pole_pre_align_pose_filter_min_samples').value),
            ),
        )
        self.pole_pre_align_reverse_cooldown_sec = max(
            0.0,
            float(self.get_parameter('pole_pre_align_reverse_cooldown_sec').value),
        )
        self.pole_pre_align_pose_control_sign = float(
            self.get_parameter('pole_pre_align_pose_control_sign').value
        )
        if abs(self.pole_pre_align_pose_control_sign) <= 1e-6:
            self.pole_pre_align_pose_control_sign = -1.0
        self.pole_pre_align_timeout_sec = max(
            0.0,
            float(self.get_parameter('pole_pre_align_timeout_sec').value),
        )
        self.pole_pre_align_total_timeout_sec = max(
            0.0,
            float(self.get_parameter('pole_pre_align_total_timeout_sec').value),
        )
        self.pole_pre_align_lateral_tolerance_m = max(
            0.0,
            float(self.get_parameter('pole_pre_align_lateral_tolerance_m').value),
        )
        self.pole_pre_align_camera_left_offset_m = float(
            self.get_parameter('pole_pre_align_camera_left_offset_m').value
        )
        raw_lateral_control_sign = float(
            self.get_parameter('pole_pre_align_lateral_control_sign').value
        )
        self.pole_pre_align_lateral_control_sign = (
            -1.0 if raw_lateral_control_sign < 0.0 else 1.0
        )
        self.pole_pre_align_shift_yaw_rad = math.radians(
            max(0.0, float(self.get_parameter('pole_pre_align_shift_yaw_deg').value))
        )
        self.pole_pre_align_shift_distance_scale = max(
            0.0,
            float(self.get_parameter('pole_pre_align_shift_distance_scale').value),
        )
        self.pole_pre_align_shift_max_m = max(
            0.0,
            float(self.get_parameter('pole_pre_align_shift_max_m').value),
        )
        self.pole_pre_align_shift_deadband_m = max(
            0.0,
            float(self.get_parameter('pole_pre_align_shift_deadband_m').value),
        )
        self.pole_pre_align_shift_rebase_use_turn_drift = bool(
            self.get_parameter('pole_pre_align_shift_rebase_use_turn_drift').value
        )
        self.pole_pre_align_reuse_existing_turn_steps = bool(
            self.get_parameter('pole_pre_align_reuse_existing_turn_steps').value
        )
        self.pole_pre_align_reuse_existing_small_move_steps = bool(
            self.get_parameter('pole_pre_align_reuse_existing_small_move_steps').value
        )
        self.pole_pre_align_turn_mode = int(
            self.get_parameter('pole_pre_align_turn_mode').value
        )
        self.pole_pre_align_pose_turn_scale = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter('pole_pre_align_pose_turn_scale').value),
            ),
        )
        self.pole_pre_align_ccw_left_norm = float(
            self.get_parameter('pole_pre_align_ccw_left_norm').value
        )
        self.pole_pre_align_ccw_right_norm = float(
            self.get_parameter('pole_pre_align_ccw_right_norm').value
        )
        self.pole_pre_align_cw_left_norm = float(
            self.get_parameter('pole_pre_align_cw_left_norm').value
        )
        self.pole_pre_align_cw_right_norm = float(
            self.get_parameter('pole_pre_align_cw_right_norm').value
        )
        self.pole_pre_align_turn_scale_enabled = bool(
            self.get_parameter('pole_pre_align_turn_scale_enabled').value
        )
        self.pole_pre_align_turn_min_scale = max(
            0.0,
            min(1.0, float(self.get_parameter('pole_pre_align_turn_min_scale').value)),
        )
        self.pole_pre_align_turn_yaw_gain_per_rad = max(
            0.0,
            float(self.get_parameter('pole_pre_align_turn_yaw_gain_per_rad').value),
        )
        self.pole_pre_align_turn_min_abs_angular_z = max(
            0.0,
            float(self.get_parameter('pole_pre_align_turn_min_abs_angular_z').value),
        )
        self.pole_pre_align_turn_max_abs_angular_z = max(
            self.pole_pre_align_turn_min_abs_angular_z,
            float(self.get_parameter('pole_pre_align_turn_max_abs_angular_z').value),
        )
        self.pole_pre_align_precision_turn_enabled = bool(
            self.get_parameter('pole_pre_align_precision_turn_enabled').value
        )
        self.pole_pre_align_turn_far_error_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('pole_pre_align_turn_far_error_deg').value),
            )
        )
        self.pole_pre_align_turn_far_scale = max(
            1.0,
            float(self.get_parameter('pole_pre_align_turn_far_scale').value),
        )
        self.pole_pre_align_turn_yaw_damping_sec = max(
            0.0,
            float(self.get_parameter('pole_pre_align_turn_yaw_damping_sec').value),
        )
        self.pole_pre_align_turn_brake_yaw_rate_rad_s = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'pole_pre_align_turn_brake_yaw_rate_deg_s'
                    ).value
                ),
            )
        )
        self.pole_pre_align_turn_complete_yaw_rate_rad_s = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'pole_pre_align_turn_complete_yaw_rate_deg_s'
                    ).value
                ),
            )
        )
        self.pole_pre_align_turn_yaw_rate_filter_alpha = max(
            0.0,
            min(
                1.0,
                float(
                    self.get_parameter(
                        'pole_pre_align_turn_yaw_rate_filter_alpha'
                    ).value
                ),
            ),
        )
        self.pole_pre_align_turn_yaw_rate_window_sec = max(
            1e-3,
            float(
                self.get_parameter(
                    'pole_pre_align_turn_yaw_rate_window_sec'
                ).value
            ),
        )
        self.pole_pre_align_turn_yaw_rate_min_span_sec = max(
            1e-3,
            min(
                self.pole_pre_align_turn_yaw_rate_window_sec,
                float(
                    self.get_parameter(
                        'pole_pre_align_turn_yaw_rate_min_span_sec'
                    ).value
                ),
            ),
        )
        self.pole_pre_align_turn_yaw_rate_max_abs_rad_s = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'pole_pre_align_turn_yaw_rate_max_abs_deg_s'
                    ).value
                ),
            )
        )
        self.pole_pre_align_fine_angle_threshold_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('pole_pre_align_fine_angle_threshold_deg').value),
            )
        )
        self.pole_pre_align_fine_ccw_left_norm = float(
            self.get_parameter('pole_pre_align_fine_ccw_left_norm').value
        )
        self.pole_pre_align_fine_ccw_right_norm = float(
            self.get_parameter('pole_pre_align_fine_ccw_right_norm').value
        )
        self.pole_pre_align_fine_cw_left_norm = float(
            self.get_parameter('pole_pre_align_fine_cw_left_norm').value
        )
        self.pole_pre_align_fine_cw_right_norm = float(
            self.get_parameter('pole_pre_align_fine_cw_right_norm').value
        )
        self.pole_pre_align_walk_mode = int(
            self.get_parameter('pole_pre_align_walk_mode').value
        )
        self.pole_pre_align_walk_left_norm = float(
            self.get_parameter('pole_pre_align_walk_left_norm').value
        )
        self.pole_pre_align_walk_right_norm = float(
            self.get_parameter('pole_pre_align_walk_right_norm').value
        )
        self.pole_pre_align_walk_yaw_correction_enabled = bool(
            self.get_parameter('pole_pre_align_walk_yaw_correction_enabled').value
        )
        self.pole_pre_align_walk_yaw_deadband_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('pole_pre_align_walk_yaw_deadband_deg').value),
            )
        )
        self.pole_pre_align_walk_yaw_gain_norm_per_rad = max(
            0.0,
            float(self.get_parameter('pole_pre_align_walk_yaw_gain_norm_per_rad').value),
        )
        self.pole_pre_align_walk_yaw_max_delta_norm = max(
            0.0,
            float(self.get_parameter('pole_pre_align_walk_yaw_max_delta_norm').value),
        )
        self.pole_pre_align_stop_hold_sec = max(
            0.0,
            float(self.get_parameter('pole_pre_align_stop_hold_sec').value),
        )
        self.pole_pre_align_backup_enabled = bool(
            self.get_parameter('pole_pre_align_backup_enabled').value
        )
        self.pole_pre_align_backup_mode = int(
            self.get_parameter('pole_pre_align_backup_mode').value
        )
        self.pole_pre_align_backup_left_norm = float(
            self.get_parameter('pole_pre_align_backup_left_norm').value
        )
        self.pole_pre_align_backup_right_norm = float(
            self.get_parameter('pole_pre_align_backup_right_norm').value
        )
        self.pole_pre_align_backup_distance_m = max(
            0.0,
            float(self.get_parameter('pole_pre_align_backup_distance_m').value),
        )
        self.pole_pre_align_backup_max_retries = max(
            0,
            int(self.get_parameter('pole_pre_align_backup_max_retries').value),
        )
        self.pole_pre_align_min_tag_z_m = max(
            0.0,
            float(self.get_parameter('pole_pre_align_min_tag_z_m').value),
        )
        self.pole_pre_align_entry_tag_z_m = max(
            0.0,
            float(self.get_parameter('pole_pre_align_entry_tag_z_m').value),
        )
        self.pole_pre_align_entry_tag_z_tolerance_m = max(
            0.0,
            float(self.get_parameter('pole_pre_align_entry_tag_z_tolerance_m').value),
        )
        self.pole_pre_align_entry_forward_enabled = bool(
            self.get_parameter('pole_pre_align_entry_forward_enabled').value
        )
        self.pole_pre_align_entry_forward_mode = int(
            self.get_parameter('pole_pre_align_entry_forward_mode').value
        )
        self.pole_pre_align_entry_forward_left_norm = float(
            self.get_parameter('pole_pre_align_entry_forward_left_norm').value
        )
        self.pole_pre_align_entry_forward_right_norm = float(
            self.get_parameter('pole_pre_align_entry_forward_right_norm').value
        )
        self.pole_pre_align_entry_forward_max_m = max(
            0.0,
            float(self.get_parameter('pole_pre_align_entry_forward_max_m').value),
        )
        self.pole_pre_align_verify_require_tag = bool(
            self.get_parameter('pole_pre_align_verify_require_tag').value
        )
        self.pole_pre_align_verify_timeout_sec = max(
            0.0,
            float(self.get_parameter('pole_pre_align_verify_timeout_sec').value),
        )

        self.startup_step_override_enabled = bool(
            self.get_parameter('startup_step_override_enabled').value
        )
        self.startup_step_override_mode = int(self.get_parameter('startup_step_override_mode').value)
        self.startup_left_norm = float(self.get_parameter('startup_left_norm').value)
        self.startup_right_norm = float(self.get_parameter('startup_right_norm').value)
        self.startup_hold_when_near_enabled = bool(
            self.get_parameter('startup_hold_when_near_enabled').value
        )
        self.startup_hold_when_near_distance_m = max(
            0.0,
            float(self.get_parameter('startup_hold_when_near_distance_m').value),
        )
        self.startup_hold_when_near_lateral_abs_m = max(
            0.0,
            float(self.get_parameter('startup_hold_when_near_lateral_abs_m').value),
        )
        self.startup_ramp_duration_sec = max(
            0.0,
            float(self.get_parameter('startup_ramp_duration_sec').value),
        )
        self.startup_override_rate_hz = max(
            1.0,
            float(self.get_parameter('startup_override_rate_hz').value),
        )
        self.startup_route_handoff_enabled = bool(
            self.get_parameter('startup_route_handoff_enabled').value
        )
        self.startup_route_handoff_cmd_vel_topic = str(
            self.get_parameter('startup_route_handoff_cmd_vel_topic').value
        )
        self.startup_route_handoff_max_hold_sec = max(
            0.0,
            float(self.get_parameter('startup_route_handoff_max_hold_sec').value),
        )
        self.startup_route_handoff_min_linear_speed_mps = max(
            0.0,
            float(self.get_parameter('startup_route_handoff_min_linear_speed_mps').value),
        )
        self.startup_route_handoff_min_angular_speed_rps = max(
            0.0,
            float(self.get_parameter('startup_route_handoff_min_angular_speed_rps').value),
        )
        self.startup_route_handoff_republish_goal_on_timeout = bool(
            self.get_parameter('startup_route_handoff_republish_goal_on_timeout').value
        )
        self.startup_route_handoff_max_republish_attempts = max(
            0,
            int(self.get_parameter('startup_route_handoff_max_republish_attempts').value),
        )
        self.publish_mode_at_start = bool(self.get_parameter('publish_mode_at_start').value)
        self.route_mode = int(self.get_parameter('route_mode').value)
        raw_forward_route_mode = int(self.get_parameter('forward_route_mode').value)
        raw_first_forward_route_mode = int(
            self.get_parameter('first_forward_route_mode').value
        )
        raw_turn_route_mode = int(self.get_parameter('turn_route_mode').value)
        self.forward_route_mode = self.route_mode if raw_forward_route_mode < 0 else raw_forward_route_mode
        self.first_forward_route_mode = (
            self.forward_route_mode
            if raw_first_forward_route_mode < 0
            else raw_first_forward_route_mode
        )
        self.turn_route_mode = self.route_mode if raw_turn_route_mode < 0 else raw_turn_route_mode
        self.route_mode_refresh_rate_hz = max(
            1.0,
            float(self.get_parameter('route_mode_refresh_rate_hz').value),
        )
        self.disable_fixed_step_override_during_route = bool(
            self.get_parameter('disable_fixed_step_override_during_route').value
        )
        self.finish_mode = int(self.get_parameter('finish_mode').value)

        self.waypoint_offsets = self._parse_offsets(
            self.get_parameter('waypoint_offsets_xy').value
        )
        # 2026-05-03 03:24 CST: the measured route YAML stores adjacent
        # point-to-point deltas from src/点位.txt. Runtime targets now use the
        # trigger-time route anchor plus these cumulative deltas, so arrival
        # error at one waypoint cannot shift every later waypoint. Rollback:
        # set route_reference_mode=current_pose to restore the old chaining.
        self.waypoint_cumulative_offsets = self._cumulative_offsets(self.waypoint_offsets)
        self.route_reference_mode = str(
            self.get_parameter('route_reference_mode').value
        ).strip().lower()
        if self.route_reference_mode not in ('route_anchor', 'current_pose', 'locked_route_frame'):
            raise RuntimeError(
                'route_reference_mode must be route_anchor, current_pose, or locked_route_frame.'
            )
        self.forward_tracking_enabled = bool(
            self.get_parameter('forward_tracking_enabled').value
        )
        self.forward_tracking_base_left_norm = self._clamp_norm(
            float(self.get_parameter('forward_tracking_base_left_norm').value)
        )
        self.forward_tracking_base_right_norm = self._clamp_norm(
            float(self.get_parameter('forward_tracking_base_right_norm').value)
        )
        self.forward_tracking_lateral_gain = max(
            0.0,
            float(self.get_parameter('forward_tracking_lateral_gain').value),
        )
        self.forward_tracking_lateral_gain_map = self._parse_waypoint_scalar_map(
            self._get_optional_array_parameter_value(
                'forward_tracking_lateral_gain_waypoint_indices'
            ),
            self._get_optional_array_parameter_value(
                'forward_tracking_lateral_gain_waypoint_values'
            ),
            len(self.waypoint_offsets),
            'forward_tracking_lateral_gain_waypoint_indices',
            'forward_tracking_lateral_gain_waypoint_values',
        )
        self.forward_tracking_lateral_gain_map = {
            idx: max(0.0, value)
            for idx, value in self.forward_tracking_lateral_gain_map.items()
        }
        self.forward_tracking_yaw_gain = max(
            0.0,
            float(self.get_parameter('forward_tracking_yaw_gain').value),
        )
        self.forward_tracking_deadband_m = max(
            0.0,
            float(self.get_parameter('forward_tracking_deadband_m').value),
        )
        self.forward_tracking_max_delta_norm = max(
            0.0,
            float(self.get_parameter('forward_tracking_max_delta_norm').value),
        )
        self.forward_tracking_lateral_fail_m = max(
            0.0,
            float(self.get_parameter('forward_tracking_lateral_fail_m').value),
        )
        self.forward_tracking_arrival_lateral_tolerance_map = self._parse_waypoint_scalar_map(
            self._get_optional_array_parameter_value(
                'forward_tracking_arrival_lateral_tolerance_waypoint_indices'
            ),
            self._get_optional_array_parameter_value(
                'forward_tracking_arrival_lateral_tolerance_waypoint_values'
            ),
            len(self.waypoint_offsets),
            'forward_tracking_arrival_lateral_tolerance_waypoint_indices',
            'forward_tracking_arrival_lateral_tolerance_waypoint_values',
        )
        self.forward_tracking_arrival_lateral_tolerance_map = {
            idx: max(0.0, value)
            for idx, value in self.forward_tracking_arrival_lateral_tolerance_map.items()
        }
        self.forward_tracking_yaw_fail_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('forward_tracking_yaw_fail_deg').value),
            )
        )
        self.forward_tracking_timeout_sec_per_m = max(
            0.0,
            float(self.get_parameter('forward_tracking_timeout_sec_per_m').value),
        )
        self.forward_tracking_timeout_extra_sec = max(
            0.0,
            float(self.get_parameter('forward_tracking_timeout_extra_sec').value),
        )
        self.goal_yaw_mode = str(self.get_parameter('goal_yaw_mode').value).strip().lower()
        self.first_forward_goal_yaw_mode = str(
            self.get_parameter('first_forward_goal_yaw_mode').value
        ).strip().lower()
        self.arrival_tolerance = float(self.get_parameter('arrival_tolerance').value)
        self.waypoint_arrival_tolerances = self._parse_optional_float_list(
            self._get_optional_array_parameter_value('waypoint_arrival_tolerances'),
            len(self.waypoint_offsets),
            'waypoint_arrival_tolerances',
            self.arrival_tolerance,
        )
        self.forward_waypoint_pass_through_enabled = bool(
            self.get_parameter('forward_waypoint_pass_through_enabled').value
        )
        self.forward_waypoint_pass_through_lateral_tolerance_m = max(
            0.0,
            float(self.get_parameter('forward_waypoint_pass_through_lateral_tolerance_m').value),
        )
        self.forward_waypoint_pass_through_longitudinal_tolerance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'forward_waypoint_pass_through_longitudinal_tolerance_m'
                ).value
            ),
        )
        self.require_fresh_pole_for_route_start = bool(
            self.get_parameter('require_fresh_pole_for_route_start').value
        )
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.next_goal_delay_sec = max(0.0, float(self.get_parameter('next_goal_delay_sec').value))
        self.turn_settle_sec = max(0.0, float(self.get_parameter('turn_settle_sec').value))
        self.turn_settle_yaw_tolerance_deg = max(
            0.0,
            float(self.get_parameter('turn_settle_yaw_tolerance_deg').value),
        )
        self.turn_settle_yaw_unstable_fail_enabled = bool(
            self.get_parameter('turn_settle_yaw_unstable_fail_enabled').value
        )
        self.monitor_rate_hz = float(self.get_parameter('monitor_rate_hz').value)
        self.progress_log_interval_sec = float(
            self.get_parameter('progress_log_interval_sec').value
        )
        self.status_heartbeat_sec = max(0.0, float(self.get_parameter('status_heartbeat_sec').value))

        raw_waypoint_yaws = self._get_optional_array_parameter_value('waypoint_yaws_deg')
        self.waypoint_yaws_configured = len(raw_waypoint_yaws) > 0
        self.waypoint_yaws_deg = self._parse_optional_float_list(
            raw_waypoint_yaws,
            len(self.waypoint_offsets),
            'waypoint_yaws_deg',
            0.0,
        )
        self.waypoint_use_yaw_tolerance = self._parse_optional_bool_list(
            self._get_optional_array_parameter_value('waypoint_use_yaw_tolerance'),
            len(self.waypoint_offsets),
            'waypoint_use_yaw_tolerance',
            False,
        )
        self.yaw_tolerance_rad = math.radians(
            float(self.get_parameter('yaw_tolerance_deg').value)
        )
        self.waypoint_use_direct_step_override = self._parse_optional_bool_list(
            self._get_optional_array_parameter_value('waypoint_use_direct_step_override'),
            len(self.waypoint_offsets),
            'waypoint_use_direct_step_override',
            False,
        )
        self.waypoint_direct_step_left_norms = self._parse_optional_float_list(
            self._get_optional_array_parameter_value('waypoint_direct_step_left_norms'),
            len(self.waypoint_offsets),
            'waypoint_direct_step_left_norms',
            0.0,
        )
        self.waypoint_direct_step_right_norms = self._parse_optional_float_list(
            self._get_optional_array_parameter_value('waypoint_direct_step_right_norms'),
            len(self.waypoint_offsets),
            'waypoint_direct_step_right_norms',
            0.0,
        )
        # 2026-06-28 09:06 CST
        # Reason: field log showed pole waypoint 1/14 entered pose_controller
        # and saturated wz=0.8, arcing away from the 1.05m pre-run goal.
        # Purpose: allow selected forward waypoints to keep the existing
        # distance/yaw arrival monitor but drive with a fixed forward gait.
        # Effect: waypoint 1 can go straight through /serial_step_override
        # instead of letting pose_controller turn toward the point. Rollback:
        # set waypoint_use_forward_step_override to all false in YAML.
        self.waypoint_use_forward_step_override = self._parse_optional_bool_list(
            self._get_optional_array_parameter_value('waypoint_use_forward_step_override'),
            len(self.waypoint_offsets),
            'waypoint_use_forward_step_override',
            False,
        )
        self.waypoint_forward_step_left_norms = self._parse_optional_float_list(
            self._get_optional_array_parameter_value('waypoint_forward_step_left_norms'),
            len(self.waypoint_offsets),
            'waypoint_forward_step_left_norms',
            0.0,
        )
        self.waypoint_forward_step_right_norms = self._parse_optional_float_list(
            self._get_optional_array_parameter_value('waypoint_forward_step_right_norms'),
            len(self.waypoint_offsets),
            'waypoint_forward_step_right_norms',
            0.0,
        )
        self.turn_waypoint_use_motion_primitive = bool(
            self.get_parameter('turn_waypoint_use_motion_primitive').value
        )
        self.turn_waypoint_arc_inner_norm = max(
            0.0,
            float(self.get_parameter('turn_waypoint_arc_inner_norm').value),
        )
        self.turn_waypoint_arc_outer_norm = max(
            self.turn_waypoint_arc_inner_norm,
            float(self.get_parameter('turn_waypoint_arc_outer_norm').value),
        )
        self.turn_waypoint_min_duration_sec = max(
            0.0,
            float(self.get_parameter('turn_waypoint_min_duration_sec').value),
        )
        self.turn_waypoint_completion_yaw_tolerance_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('turn_waypoint_completion_yaw_tolerance_deg').value),
            )
        )
        self.turn_settle_yaw_tolerance_rad = (
            math.radians(self.turn_settle_yaw_tolerance_deg)
            if self.turn_settle_yaw_tolerance_deg > 1e-6
            else self.turn_waypoint_completion_yaw_tolerance_rad
        )
        self.turn_waypoint_completion_progress_tolerance_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter('turn_waypoint_completion_progress_tolerance_deg').value
                ),
            )
        )
        self.turn_waypoint_overshoot_guard_enabled = bool(
            self.get_parameter('turn_waypoint_overshoot_guard_enabled').value
        )
        self.turn_waypoint_overshoot_tolerance_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('turn_waypoint_overshoot_tolerance_deg').value),
            )
        )
        self.turn_waypoint_overshoot_accept_yaw_tolerance_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'turn_waypoint_overshoot_accept_yaw_tolerance_deg'
                    ).value
                ),
            )
        )
        self.turn_waypoint_slowdown_enabled = bool(
            self.get_parameter('turn_waypoint_slowdown_enabled').value
        )
        self.turn_waypoint_slowdown_remaining_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('turn_waypoint_slowdown_remaining_deg').value),
            )
        )
        self.turn_waypoint_slowdown_cw_left_norm = float(
            self.get_parameter('turn_waypoint_slowdown_cw_left_norm').value
        )
        self.turn_waypoint_slowdown_cw_right_norm = float(
            self.get_parameter('turn_waypoint_slowdown_cw_right_norm').value
        )
        self.turn_waypoint_slowdown_ccw_left_norm = float(
            self.get_parameter('turn_waypoint_slowdown_ccw_left_norm').value
        )
        self.turn_waypoint_slowdown_ccw_right_norm = float(
            self.get_parameter('turn_waypoint_slowdown_ccw_right_norm').value
        )
        self.turn_waypoint_longitudinal_tolerance_m = max(
            0.0,
            float(self.get_parameter('turn_waypoint_longitudinal_tolerance_m').value),
        )
        self.turn_waypoint_lateral_tolerance_m = max(
            0.0,
            float(self.get_parameter('turn_waypoint_lateral_tolerance_m').value),
        )
        self.turn_waypoint_exit_forward_norm = max(
            0.0,
            float(self.get_parameter('turn_waypoint_exit_forward_norm').value),
        )
        self.turn_waypoint_exit_forward_min_duration_sec = max(
            0.0,
            float(self.get_parameter('turn_waypoint_exit_forward_min_duration_sec').value),
        )
        self.turn_waypoint_exit_forward_timeout_sec = max(
            0.0,
            float(self.get_parameter('turn_waypoint_exit_forward_timeout_sec').value),
        )
        self.turn_waypoint_timeout_sec = max(
            0.0,
            float(self.get_parameter('turn_waypoint_timeout_sec').value),
        )
        self.turn_waypoint_simple_in_place_enabled = bool(
            self.get_parameter('turn_waypoint_simple_in_place_enabled').value
        )
        self.turn_waypoint_yaw_mode = str(
            self.get_parameter('turn_waypoint_yaw_mode').value
        ).strip().lower()
        self.direct_step_start_distance_m = max(
            0.0,
            float(self.get_parameter('direct_step_start_distance_m').value),
        )
        self.direct_step_recover_distance_m = max(
            self.direct_step_start_distance_m,
            float(self.get_parameter('direct_step_recover_distance_m').value),
        )
        body_relative_waypoint_indices = self._get_optional_array_parameter_value(
            'body_relative_waypoint_indices'
        )
        body_relative_waypoint_offsets = self._get_optional_array_parameter_value(
            'body_relative_waypoint_offsets_fl'
        )
        self.body_relative_waypoint_offset_map = self._parse_body_relative_waypoint_map(
            body_relative_waypoint_indices,
            body_relative_waypoint_offsets,
            len(self.waypoint_offsets),
        )
        body_relative_turn_indices = self._get_optional_array_parameter_value(
            'body_relative_turn_waypoint_indices'
        )
        body_relative_turn_deltas = self._get_optional_array_parameter_value(
            'body_relative_turn_deltas_deg'
        )
        self.body_relative_turn_delta_map = self._parse_waypoint_scalar_map(
            body_relative_turn_indices,
            body_relative_turn_deltas,
            len(self.waypoint_offsets),
            'body_relative_turn_waypoint_indices',
            'body_relative_turn_deltas_deg',
        )
        turn_gate_indices = self._get_optional_array_parameter_value(
            'turn_waypoint_pole_distance_gate_indices'
        )
        turn_gate_thresholds = self._get_optional_array_parameter_value(
            'turn_waypoint_pole_distance_gate_thresholds_m'
        )
        self.turn_waypoint_pole_distance_gate_map = self._parse_waypoint_threshold_map(
            turn_gate_indices,
            turn_gate_thresholds,
            len(self.waypoint_offsets),
            'turn_waypoint_pole_distance_gate_indices',
            'turn_waypoint_pole_distance_gate_thresholds_m',
        )

        if self.goal_yaw_mode not in ('current', 'path_heading', 'zero', 'waypoint'):
            raise RuntimeError(
                'goal_yaw_mode must be one of: current, path_heading, zero, waypoint.'
            )
        if self.first_forward_goal_yaw_mode not in (
            'inherit',
            'current',
            'path_heading',
            'zero',
            'waypoint',
        ):
            raise RuntimeError(
                'first_forward_goal_yaw_mode must be one of: '
                'inherit, current, path_heading, zero, waypoint.'
            )
        if self.goal_yaw_mode == 'waypoint' and not self.waypoint_yaws_configured:
            raise RuntimeError('goal_yaw_mode=waypoint requires waypoint_yaws_deg.')
        if (
            self.first_forward_goal_yaw_mode == 'waypoint'
            and not self.waypoint_yaws_configured
        ):
            raise RuntimeError(
                'first_forward_goal_yaw_mode=waypoint requires waypoint_yaws_deg.'
            )
        if self.turn_waypoint_yaw_mode not in ('global_waypoint', 'body_relative_delta'):
            raise RuntimeError(
                'turn_waypoint_yaw_mode must be global_waypoint or body_relative_delta.'
            )

        goal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            # 2026-05-03 01:40 CST: keep behavior aligned with
            # yolo_relative_nav so a restarted route consumer can still see the
            # latest goal; nav_executor subscriptions are already VOLATILE and
            # compatible. Rollback: switch this publisher to VOLATILE if a new
            # consumer rejects TRANSIENT_LOCAL publishers.
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        fixed_step_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, goal_qos)
        self.arrival_pub = self.create_publisher(Bool, self.arrival_status_topic, 10)
        self.step_override_pub = self.create_publisher(
            Float32MultiArray,
            self.step_override_topic,
            10,
        )
        self.mode_pub = self.create_publisher(Int32, self.mode_topic, 10)
        self.fixed_step_override_state_pub = self.create_publisher(
            Bool,
            self.fixed_step_override_state_topic,
            fixed_step_qos,
        )
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.startup_route_handoff_active = False
        self.startup_route_handoff_deadline = 0.0
        self.startup_route_handoff_republish_attempts = 0

        self.latest_apriltag: Optional[AprilTagSnapshot] = None
        self.latest_apriltag_detection = AprilTagDetectionInfo()
        self.route_yaw_reference: Optional[float] = None
        self.route_yaw_reference_frame = ''
        self.route_yaw_reference_stamp_sec = 0.0
        self.pose_sub = self._create_pose_subscription()
        self.route_yaw_reference_sub = None
        if self.runtime_route_yaw_reference_enabled:
            self.route_yaw_reference_sub = self.create_subscription(
                PoseStamped,
                self.route_yaw_reference_topic,
                self._route_yaw_reference_callback,
                10,
            )
        self.route_cmd_vel_sub = None
        if self.startup_route_handoff_enabled:
            self.route_cmd_vel_sub = self.create_subscription(
                Twist,
                self.startup_route_handoff_cmd_vel_topic,
                self._route_cmd_vel_callback,
                10,
            )
        self.apriltag_tf_sub = None
        self.apriltag_detection_sub = None
        if self._apriltag_runtime_enabled():
            self.apriltag_tf_sub = self.create_subscription(
                TFMessage,
                self.apriltag_tf_topic,
                self._apriltag_tf_callback,
                qos_profile_sensor_data,
            )
            self.apriltag_detection_sub = self.create_subscription(
                AprilTagDetectionArray,
                self.apriltag_detections_topic,
                self._apriltag_detections_callback,
                qos_profile_sensor_data,
            )
        self.nearest_sub = self.create_subscription(
            Float32MultiArray,
            self.nearest_topic,
            self._nearest_callback,
            10,
        )
        self.trigger_sub = self.create_subscription(
            Bool,
            self.trigger_topic,
            self._trigger_callback,
            10,
        )
        self.active_sub = self.create_subscription(
            Bool,
            self.active_topic,
            self._active_callback,
            10,
        )

        self.have_pose = False
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.pose_frame = ''
        self.last_pose_time = None
        self.pose_yaw_rate_rad_s = 0.0
        self.pose_yaw_rate_raw_rad_s = 0.0
        self.pose_yaw_rate_estimator = WindowedYawRateEstimator(
            window_sec=self.pole_pre_align_turn_yaw_rate_window_sec,
            min_span_sec=self.pole_pre_align_turn_yaw_rate_min_span_sec,
            max_abs_rate_rad_s=self.pole_pre_align_turn_yaw_rate_max_abs_rad_s,
            smoothing_alpha=self.pole_pre_align_turn_yaw_rate_filter_alpha,
        )

        self.triggered = False
        self.route_finished = False
        self.route_failed = False
        self.current_idx = 0
        self.active_goal = False
        self.route_start_pending = False
        self.pending_route_reason = ''
        self.pending_route_source = ''
        self.route_source = ''
        self.awaiting_next_goal = False
        self.next_goal_time = 0.0
        self.active_goal_x = 0.0
        self.active_goal_y = 0.0
        self.active_goal_yaw = 0.0
        self.active_goal_nav_yaw = 0.0
        self.active_goal_frame_id = ''
        self.active_goal_arrival_tolerance = self.arrival_tolerance
        self.active_goal_use_yaw_tolerance = False
        self.active_goal_allow_pass_through = False
        self.active_goal_has_direct_override = False
        self.active_goal_direct_override = False
        self.active_goal_has_forward_override = False
        self.active_goal_has_forward_tracking = False
        self.active_goal_is_turn_primitive = False
        self.active_turn_phase = ''
        self.active_turn_settle_reason = ''
        self.active_turn_settle_dist = 0.0
        self.active_goal_stamp = 0.0
        self.active_segment_start_x = 0.0
        self.active_segment_start_y = 0.0
        self.active_turn_direction_sign = 0
        self.active_turn_start_yaw = 0.0
        self.active_turn_last_yaw = 0.0
        self.active_turn_progress = 0.0
        self.active_turn_target_delta = 0.0
        self.active_turn_uses_body_relative_yaw = False
        self.active_turn_uses_locked_yaw = False
        self.active_turn_body_relative_delta_rad = 0.0
        self.active_turn_segment_length = 0.0
        self.active_turn_start_time = 0.0
        self.active_turn_phase_start_time = 0.0
        self.active_turn_pole_distance_gate_m = 0.0
        self.route_anchor_ready = False
        self.route_anchor_x = 0.0
        self.route_anchor_y = 0.0
        self.route_anchor_yaw = 0.0
        self.route_anchor_frame = ''
        self.pending_route_source = ''
        self.route_source = ''
        self.last_state: Optional[str] = None
        self.last_feedback: Optional[str] = None
        self.last_progress_log_time = 0.0
        self.last_wait_log_time = 0.0
        self.last_status_heartbeat_time = 0.0
        self.startup_override_start_time: Optional[float] = None
        self.last_pole_msg_time: Optional[float] = None
        self.last_trigger_msg_time: Optional[float] = None
        self.last_detector_trigger = False
        self.last_pole: Optional[PoleSnapshot] = None
        self.last_pole_text = 'none'
        self._reset_pole_pre_align_runtime()

        self.monitor_timer = self.create_timer(
            1.0 / max(self.monitor_rate_hz, 1.0),
            self._monitor_loop,
        )
        self.startup_override_timer = None
        if self.startup_step_override_enabled:
            self._ensure_startup_override_timer()
        self.route_mode_timer = self.create_timer(
            1.0 / self.route_mode_refresh_rate_hz,
            self._refresh_route_mode,
        )

        if self.route_controller_active:
            self._activate_controller('startup', log_wait=True)
        else:
            self._publish_state('INACTIVE')
            self._publish_feedback('orange_pole_relative_nav inactive on startup.')

    def _create_pose_subscription(self):
        if self.pose_topic_type == 'odometry':
            return self.create_subscription(
                Odometry,
                self.pose_topic,
                self._odometry_callback,
                qos_profile_sensor_data,
            )
        if self.pose_topic_type == 'pose_stamped':
            return self.create_subscription(
                PoseStamped,
                self.pose_topic,
                self._pose_stamped_callback,
                qos_profile_sensor_data,
            )
        if self.pose_topic_type == 'pose_with_covariance_stamped':
            return self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self._pose_with_covariance_callback,
                qos_profile_sensor_data,
            )
        raise RuntimeError(
            'Unsupported pose_topic_type. Use odometry, pose_stamped, '
            'or pose_with_covariance_stamped.'
        )

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

    def _route_yaw_reference_callback(self, msg: PoseStamped) -> None:
        self.route_yaw_reference = self._normalize_angle(
            self._yaw_from_quaternion(msg.pose.orientation)
        )
        self.route_yaw_reference_frame = str(msg.header.frame_id)
        self.route_yaw_reference_stamp_sec = self._now_sec()

    def _pose_with_covariance_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._update_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._yaw_from_quaternion(msg.pose.pose.orientation),
            msg.header.frame_id,
        )

    def _update_pose(self, x: float, y: float, yaw: float, frame_id: str) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9
        (
            self.pose_yaw_rate_rad_s,
            self.pose_yaw_rate_raw_rad_s,
        ) = self.pose_yaw_rate_estimator.update(yaw, now_sec)
        self.pose_x = float(x)
        self.pose_y = float(y)
        self.pose_yaw = float(yaw)
        self.pose_frame = frame_id or self.default_frame_id
        self.last_pose_time = now
        self.have_pose = True

    def _apriltag_tf_callback(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            frame_id = str(transform.header.frame_id)
            child_frame_id = str(transform.child_frame_id)
            if (
                frame_id != self.apriltag_target_frame_id
                or child_frame_id != self.apriltag_target_child_frame_id
            ):
                continue

            roll, pitch, yaw = self._rpy_from_quaternion(transform.transform.rotation)
            self.latest_apriltag = AprilTagSnapshot(
                frame_id=frame_id,
                child_frame_id=child_frame_id,
                stamp_sec=self._stamp_to_sec(transform.header.stamp),
                received_sec=self._now_sec(),
                x_m=float(transform.transform.translation.x),
                y_m=float(transform.transform.translation.y),
                z_m=float(transform.transform.translation.z),
                roll_rad=roll,
                pitch_rad=pitch,
                yaw_rad=yaw,
            )
            if self.pole_pre_align_active:
                self._record_pole_pre_align_pitch_sample(self.latest_apriltag)
            if (
                self._apriltag_trigger_enabled()
                and self.latest_apriltag_detection.matched
                and not self.triggered
                and self.route_start_pending
                and self.pending_route_source == 'apriltag'
            ):
                self._start_route(
                    f'AprilTag ID {self.latest_apriltag_detection.selected_id} TF ready',
                    source='apriltag',
                )
            return

    def _apriltag_detections_callback(self, msg: AprilTagDetectionArray) -> None:
        info = AprilTagDetectionInfo(
            stamp_sec=self._stamp_to_sec(msg.header.stamp),
            received_sec=self._now_sec(),
            raw_count=len(msg.detections),
        )

        decoded_parts = []
        for detection in msg.detections:
            family = str(detection.family)
            tag_id = int(detection.id)
            decoded_parts.append(
                f'{family}:{tag_id}(margin={float(detection.decision_margin):.1f},'
                f'ham={int(detection.hamming)},'
                f'center={float(detection.centre.x):.0f}/{float(detection.centre.y):.0f})'
            )
            if self.apriltag_target_tag_id >= 0 and tag_id != self.apriltag_target_tag_id:
                continue
            info.matched = True
            info.selected_family = family
            info.selected_id = tag_id
            info.decision_margin = float(detection.decision_margin)
            info.hamming = int(detection.hamming)
            info.centre_x = float(detection.centre.x)
            info.centre_y = float(detection.centre.y)

        info.decoded_tags = ';'.join(decoded_parts)
        self.latest_apriltag_detection = info

        if not self._apriltag_trigger_enabled():
            return
        if not self.route_controller_active:
            return
        if self.triggered or self.route_finished or self.route_failed:
            return
        if not info.matched:
            if (info.received_sec - self.last_wait_log_time) >= self.progress_log_interval_sec:
                self.last_wait_log_time = info.received_sec
                self._publish_feedback(
                    f'AprilTag 已检测但未看到目标 ID {self.apriltag_target_tag_id}: '
                    f'raw={info.raw_count}, decoded={info.decoded_tags or "empty"}'
                )
            return

        if self.apriltag_trigger_require_tf and not self._apriltag_ready_for_anchor():
            self._start_route(
                f'AprilTag ID {info.selected_id} detection trigger, waiting fresh TF',
                source='apriltag',
            )
            return

        self._start_route(
            f'AprilTag ID {info.selected_id} detection trigger: {self._apriltag_detection_text()}',
            source='apriltag',
        )

    def _nearest_callback(self, msg: Float32MultiArray) -> None:
        now = self._now_sec()
        self.last_pole_msg_time = now
        pole = self._parse_pole_snapshot(msg)
        if pole is None:
            self.last_pole_text = 'invalid'
            return

        self.last_pole = pole
        self.last_pole_text = self._pole_text(pole)
        if not self._hsv_trigger_enabled():
            return
        if not self.route_controller_active:
            return
        if self.triggered or self.route_finished or self.route_failed:
            return

        if self.max_lateral_abs_m > 0.0 and abs(pole.lateral_m) > self.max_lateral_abs_m:
            if (now - self.last_wait_log_time) >= self.progress_log_interval_sec:
                self.last_wait_log_time = now
                self._publish_feedback(
                    f'HSV 已看到杆但 lateral 超限: {self.last_pole_text}, '
                    f'max_lateral_abs={self.max_lateral_abs_m:.2f}m'
                )
            return

        if pole.distance_m > self.trigger_distance_m:
            if (now - self.last_wait_log_time) >= self.progress_log_interval_sec:
                self.last_wait_log_time = now
                self._publish_feedback(
                    f'HSV 已看到杆但未触发: {self.last_pole_text}, '
                    f'threshold={self.trigger_distance_m:.2f}m'
                )
            return

        if self.route_start_pending:
            self._start_route(f'HSV 触发后收到最近杆: {self.last_pole_text}', source='hsv')
            if self.triggered:
                return

        if self.trigger_from_nearest_topic:
            self._start_route(f'HSV 最近杆距离触发: {self.last_pole_text}', source='hsv')

    def _trigger_callback(self, msg: Bool) -> None:
        self.last_detector_trigger = bool(msg.data)
        self.last_trigger_msg_time = self._now_sec()
        if not self._hsv_trigger_enabled():
            return
        if not self.route_controller_active:
            return
        if self.triggered or self.route_finished or self.route_failed:
            return
        if not self.use_trigger_topic or not msg.data:
            return

        if self.last_pole is None:
            self._start_route('HSV trigger topic=True', source='hsv')
            return
        self._start_route(f'HSV trigger topic=True: {self._pole_text(self.last_pole)}', source='hsv')

    def _cached_trigger_ready_for_route_start(self) -> bool:
        if not self._hsv_trigger_enabled():
            return False
        if not self.last_detector_trigger:
            return False
        if self.last_trigger_msg_time is None:
            return False
        if self.nearest_stale_timeout_sec > 0.0:
            if (self._now_sec() - self.last_trigger_msg_time) > self.nearest_stale_timeout_sec:
                return False
        if self.last_pole is None or self._nearest_is_stale():
            return False
        if self.last_pole.distance_m > self.trigger_distance_m:
            return False
        if self.max_lateral_abs_m > 0.0 and abs(self.last_pole.lateral_m) > self.max_lateral_abs_m:
            return False
        return True

    def _cached_apriltag_ready_for_route_start(self) -> bool:
        if not self._apriltag_trigger_enabled():
            return False
        if not self.latest_apriltag_detection.matched:
            return False
        return not self.apriltag_trigger_require_tf or self._apriltag_ready_for_anchor()

    def _clear_cached_pole_detection(self) -> None:
        self.last_detector_trigger = False
        self.last_trigger_msg_time = None
        self.last_pole_msg_time = None
        self.last_pole = None
        self.last_pole_text = 'none'

    def _parse_pole_snapshot(self, msg: Float32MultiArray) -> Optional[PoleSnapshot]:
        if len(msg.data) < 9:
            self._publish_feedback(
                f'orange nearest 数据长度不足: len={len(msg.data)}, expected>=9'
            )
            return None

        values = [float(item) for item in msg.data[:9]]
        if not all(math.isfinite(value) for value in values[:8]):
            self._publish_feedback('orange nearest 包含非有限距离/像素数据。')
            return None

        valid_depth_count = int(values[8])
        if valid_depth_count < self.min_valid_depth_pixels:
            return None

        distance_m = values[0]
        if distance_m <= 0.0:
            return None

        return PoleSnapshot(
            distance_m=distance_m,
            lateral_m=values[1],
            vertical_m=values[2],
            pixel_x=values[3],
            pixel_y=values[4],
            width_px=values[5],
            height_px=values[6],
            area_px=values[7],
            valid_depth_count=valid_depth_count,
        )

    def _start_route(self, reason: str, source: str = 'hsv') -> None:
        if not self.route_controller_active:
            return
        if self.triggered or self.route_finished or self.route_failed:
            return
        if self.pole_pre_align_active:
            if (self._now_sec() - self.last_wait_log_time) >= self.progress_log_interval_sec:
                self.last_wait_log_time = self._now_sec()
                self._publish_feedback(
                    '绕杆前 AprilTag 预对齐进行中，忽略重复 route start: '
                    f'reason={reason}, source={source}, '
                    f'{self._pole_pre_align_status_text()}'
                )
            return

        source = source if source in ('hsv', 'apriltag') else self.trigger_source
        self.route_start_pending = True
        self.pending_route_reason = reason
        self.pending_route_source = source
        if not self._route_start_prerequisites_ready():
            self._publish_route_start_wait_state()
            return
        if self._should_run_pole_pre_align(source):
            self._start_pole_pre_align(reason, source)
            return
        self._activate_route(reason, source)

    def _activate_route(self, reason: str, source: str = '') -> None:
        if self.triggered or self.route_finished or self.route_failed:
            return

        route_source = source or self.pending_route_source or self.trigger_source
        if route_source not in ('hsv', 'apriltag'):
            route_source = self.trigger_source if self.trigger_source != 'both' else 'hsv'
        if (
            self.pole_pre_align_enabled
            and route_source == 'apriltag'
            and self.trigger_source == 'apriltag'
            and not self.pole_pre_align_done
        ):
            if self.pole_pre_align_active:
                self._publish_feedback(
                    '绕杆前 AprilTag 预对齐未完成，阻止原 14 点路线提前启动: '
                    f'reason={reason}, {self._pole_pre_align_status_text()}'
                )
                return
            self._start_pole_pre_align(reason, route_source)
            return
        self.route_start_pending = False
        self.pending_route_reason = ''
        self.pending_route_source = ''
        self.route_source = route_source
        self.triggered = True
        self.active_goal = False
        self.awaiting_next_goal = False
        self.current_idx = 0
        self.route_anchor_ready = False
        force_anchor = self.pole_pre_align_force_anchor_once
        self.pole_pre_align_force_anchor_once = False
        if self.pole_pre_align_done and route_source == 'apriltag':
            self._release_startup_route_handoff(
                'pole_pre_align_done_route_start',
                log_release=False,
            )
        else:
            self._arm_startup_route_handoff()
        self._set_route_mode_active(True)
        self._publish_feedback(
            f'{reason}, trigger_source={route_source}, '
            f'first_forward_mode={self.first_forward_route_mode}, '
            f'forward_mode={self.forward_route_mode}, turn_mode={self.turn_route_mode}'
        )
        self._ensure_route_anchor(force=force_anchor)
        self._send_current_goal()

    def _monitor_loop(self) -> None:
        if not self.route_controller_active:
            return
        self._publish_status_heartbeat()
        self._maybe_release_startup_route_handoff()

        if self.route_finished or self.route_failed:
            return

        if self.pole_pre_align_active:
            self._run_pole_pre_align()
            return

        if self.route_start_pending and not self.triggered:
            if self._route_start_prerequisites_ready():
                if self._should_run_pole_pre_align(self.pending_route_source):
                    self._start_pole_pre_align(
                        self.pending_route_reason,
                        self.pending_route_source,
                    )
                    return
                self._activate_route(self.pending_route_reason, self.pending_route_source)
            else:
                self._publish_route_start_wait_state()
            return

        self._refresh_route_mode()

        if self.triggered and not self.active_goal and not self.awaiting_next_goal:
            if self.current_idx < len(self.waypoint_offsets):
                if self._route_start_prerequisites_ready():
                    self._ensure_route_anchor()
                    self._send_current_goal()
                else:
                    self._publish_route_start_wait_state()
            return

        if self.awaiting_next_goal:
            if self._now_sec() < self.next_goal_time:
                return
            self.awaiting_next_goal = False
            if self.current_idx >= len(self.waypoint_offsets):
                self._finish_route()
                return
            if self._route_start_prerequisites_ready():
                self._ensure_route_anchor()
                self._send_current_goal()
            else:
                self._publish_route_start_wait_state()
            return

        if not self.active_goal:
            return

        if not self.have_pose:
            self._publish_state('WAITING_POSE')
            return

        if self._pose_is_stale():
            if self.active_goal_has_forward_tracking:
                self._handle_goal_failed('forward_tracking_pose_stale')
                return
            self._publish_state('POSE_STALE')
            if self.goal_timeout_sec > 0.0 and (
                self._now_sec() - self.active_goal_stamp
            ) > self.goal_timeout_sec:
                self._handle_goal_failed('goal_timeout pose_stale')
            return

        dist = math.hypot(self.active_goal_x - self.pose_x, self.active_goal_y - self.pose_y)
        yaw_err = abs(self._normalize_angle(self.active_goal_yaw - self.pose_yaw))

        if self.active_goal_is_turn_primitive:
            self._monitor_turn_primitive(dist, yaw_err)
            return

        if self.active_goal_has_direct_override:
            body_relative_turn_delta_rad = self._turn_waypoint_body_relative_delta_rad(
                self.current_idx
            )
            requested_turn_direction_sign = None
            if body_relative_turn_delta_rad is not None and abs(body_relative_turn_delta_rad) > 1e-6:
                requested_turn_direction_sign = 1 if body_relative_turn_delta_rad > 0.0 else -1
            turn_left_norm, turn_right_norm, _ = self._resolved_turn_override(
                self.current_idx,
                requested_turn_direction_sign,
            )
            # 2026-05-03 03:43 CST: direct step override is for the final short
            # turn, not for travelling half a meter to the turn point. Approach
            # the turn target through /goal_pose first, then arm mode=6 direct
            # override only near the target. Rollback: set
            # direct_step_start_distance_m to a very large value to get the old
            # immediate direct-override behavior.
            if self.active_goal_direct_override:
                if dist > self.direct_step_recover_distance_m:
                    self._clear_step_override()
                    self.active_goal_direct_override = False
                    self._set_route_mode_active(True)
                    self._publish_active_goal_pose()
                    self._publish_state('NAVIGATING_TO_DIRECT_STEP')
                    self._publish_feedback(
                        f'转向点偏离过大，退出 direct override 重新接近: '
                        f'waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)}, '
                        f'dist={dist:.3f}m, recover>{self.direct_step_recover_distance_m:.2f}m'
                    )
                else:
                    self._publish_step_override(
                        self.turn_route_mode,
                        turn_left_norm,
                        turn_right_norm,
                    )
            elif dist <= self.direct_step_start_distance_m:
                self.active_goal_direct_override = True
                self._set_route_mode_active(True)
                self._publish_step_override(
                    self.turn_route_mode,
                    turn_left_norm,
                    turn_right_norm,
                )
                self._publish_state('DIRECT_STEP_NAVIGATING')
                self._publish_feedback(
                    f'进入 HSV 绕杆 direct override: '
                    f'waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)}, '
                    f'dist={dist:.3f}m<=start={self.direct_step_start_distance_m:.2f}m, '
                    f'yaw_err={math.degrees(yaw_err):.1f}deg, '
                    f'override=[{self.turn_route_mode},'
                    f'{turn_left_norm:.3f},{turn_right_norm:.3f}]'
                )

        if self.active_goal_has_forward_override:
            left_norm, right_norm = self._forward_override_values(self.current_idx)
            self._publish_step_override(
                self._forward_mode_for_waypoint(self.current_idx),
                left_norm,
                right_norm,
            )

        if self.active_goal_has_forward_tracking:
            if self._monitor_forward_tracking(dist):
                return

        yaw_ok = (
            True
            if not self.active_goal_use_yaw_tolerance
            else yaw_err <= self.yaw_tolerance_rad
        )
        yaw_text = ''
        if self.active_goal_use_yaw_tolerance:
            yaw_text = (
                f' yaw_err={math.degrees(yaw_err):.1f}/'
                f'{math.degrees(self.yaw_tolerance_rad):.1f}deg'
            )
        self._publish_progress(
            f'waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)} '
            f'dist={dist:.3f}m pose=({self.pose_x:.3f},{self.pose_y:.3f}) '
            f'goal=({self.active_goal_x:.3f},{self.active_goal_y:.3f})'
            f'{yaw_text}'
        )

        arrival_reason = self._arrival_reason(dist, yaw_ok)
        if arrival_reason is not None:
            self._handle_goal_arrived(dist, arrival_reason)
            return

        if self.goal_timeout_sec > 0.0 and (
            self._now_sec() - self.active_goal_stamp
        ) > self.goal_timeout_sec:
            self._handle_goal_failed(f'goal_timeout dist={dist:.3f}m')

    def _send_current_goal(self) -> None:
        if self.current_idx >= len(self.waypoint_offsets):
            self._finish_route()
            return

        waypoint_number = self.current_idx + 1
        body_relative_offset = self._body_relative_waypoint_offset(self.current_idx)
        use_direct_override = self.waypoint_use_direct_step_override[self.current_idx]
        use_forward_override = self.waypoint_use_forward_step_override[self.current_idx]
        locked_yaw: Optional[float] = None
        use_locked_route_goal = self.route_reference_mode == 'locked_route_frame'
        if use_locked_route_goal:
            if not self._ensure_route_anchor():
                self._publish_route_start_wait_state()
                return
            if body_relative_offset is not None:
                body_forward_m, body_left_m = body_relative_offset
            else:
                body_forward_m = 0.0
                body_left_m = 0.0
            (
                segment_start_x,
                segment_start_y,
                goal_x,
                goal_y,
                segment_dx,
                segment_dy,
                locked_yaw,
            ) = self._locked_route_segment(self.current_idx)
            cumulative_dx = goal_x - self.route_anchor_x
            cumulative_dy = goal_y - self.route_anchor_y
            frame_id = self.route_anchor_frame or self.default_frame_id
        elif body_relative_offset is not None:
            body_forward_m, body_left_m = body_relative_offset
            cumulative_dx = 0.0
            cumulative_dy = 0.0
            segment_dx, segment_dy = self._body_relative_offset_to_world(
                body_forward_m,
                body_left_m,
            )
            goal_x = self.pose_x + segment_dx
            goal_y = self.pose_y + segment_dy
            frame_id = self.pose_frame or self.default_frame_id
            segment_start_x = self.pose_x
            segment_start_y = self.pose_y
        else:
            segment_dx, segment_dy = self.waypoint_offsets[self.current_idx]
            cumulative_dx, cumulative_dy = self._target_offset_for_current_waypoint()
            if self.route_reference_mode == 'route_anchor':
                if not self._ensure_route_anchor():
                    self._publish_route_start_wait_state()
                    return
                goal_x = self.route_anchor_x + cumulative_dx
                goal_y = self.route_anchor_y + cumulative_dy
                frame_id = self.route_anchor_frame or self.default_frame_id
            else:
                goal_x = self.pose_x + segment_dx
                goal_y = self.pose_y + segment_dy
                frame_id = self.pose_frame or self.default_frame_id
            segment_start_x, segment_start_y = self._segment_start_for_current_waypoint()

        arrival_yaw = self._goal_yaw(segment_dx, segment_dy, self.current_idx, locked_yaw)
        use_forward_tracking = self._forward_tracking_enabled_for_current_goal(
            body_relative_offset,
            use_direct_override,
            segment_dx,
            segment_dy,
        )
        nav_goal_yaw = self._goal_publish_yaw(
            segment_dx,
            segment_dy,
            arrival_yaw,
            use_direct_override,
        )
        # 2026-05-03 03:24 CST: normal forward waypoints are position-only;
        # short turn waypoints use mode=6 direct override and must satisfy yaw.
        # Rollback: set waypoint_use_yaw_tolerance true for every waypoint in
        # YAML if exact yaw is required at all measured points again.
        use_yaw_tolerance = (
            self.waypoint_use_yaw_tolerance[self.current_idx] or use_direct_override
        )
        allow_pass_through = (
            self.forward_waypoint_pass_through_enabled
            and not use_direct_override
            and not use_forward_override
            and not use_forward_tracking
        )
        arrival_tolerance = self.waypoint_arrival_tolerances[self.current_idx]

        if (
            body_relative_offset is not None
            and not use_direct_override
            and math.hypot(segment_dx, segment_dy) <= 1e-6
        ):
            # Zero-distance route placeholders should not be handed to the
            # downstream mode-0 walker, which can otherwise emit its default
            # full-speed gait before the next waypoint is armed.
            self.active_goal = False
            self.awaiting_next_goal = False
            self._clear_step_override()
            self._publish_state('NAVIGATING')
            self._publish_feedback(
                f'跳过零位移机身相对目标点[{waypoint_number}/{len(self.waypoint_offsets)}]: '
                f'body_offset=(forward={body_forward_m:.2f}, left={body_left_m:.2f}), '
                f'goal=({goal_x:.3f},{goal_y:.3f}), '
                f'直接进入下一点，避免 mode={self._forward_mode_for_waypoint(self.current_idx)} '
                f'默认步态脉冲, forward_override_configured={use_forward_override}'
            )
            self._handle_goal_arrived(0.0, 'zero_body_relative_waypoint_skipped')
            return

        self._set_route_mode_active(True)

        if use_direct_override and self.turn_waypoint_use_motion_primitive:
            self._start_turn_primitive(
                goal_x=goal_x,
                goal_y=goal_y,
                frame_id=frame_id,
                arrival_yaw=arrival_yaw,
                arrival_tolerance=arrival_tolerance,
                use_yaw_tolerance=use_yaw_tolerance,
                planned_segment_start_x=segment_start_x,
                planned_segment_start_y=segment_start_y,
                segment_dx=segment_dx,
                segment_dy=segment_dy,
                cumulative_dx=cumulative_dx,
                cumulative_dy=cumulative_dy,
            )
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation = self._quaternion_from_yaw(nav_goal_yaw)

        self.active_goal_x = goal_x
        self.active_goal_y = goal_y
        self.active_goal_yaw = arrival_yaw
        self.active_goal_nav_yaw = nav_goal_yaw
        self.active_goal_frame_id = frame_id
        self.active_goal_arrival_tolerance = arrival_tolerance
        self.active_goal_use_yaw_tolerance = use_yaw_tolerance
        self.active_goal_allow_pass_through = allow_pass_through
        self.active_goal_has_direct_override = use_direct_override
        self.active_goal_direct_override = False
        self.active_goal_has_forward_override = use_forward_override
        self.active_goal_has_forward_tracking = use_forward_tracking
        self.active_goal_stamp = self._now_sec()
        self.active_segment_start_x = segment_start_x
        self.active_segment_start_y = segment_start_y
        self.active_goal = True
        self.awaiting_next_goal = False

        self._clear_step_override()
        self._publish_active_goal_pose()
        if use_forward_tracking:
            forward_mode = self._forward_mode_for_waypoint(self.current_idx)
            self._release_startup_route_handoff('forward_tracking_started', log_release=False)
            left_norm, right_norm, delta_norm, along, lateral_error, yaw_error, remaining = (
                self._forward_tracking_command()
            )
            self._publish_step_override(forward_mode, left_norm, right_norm)
            self._publish_state('FORWARD_TRACKING_NAVIGATING')
            self._publish_feedback(
                f'启动 locked route 前进纠偏[{waypoint_number}/{len(self.waypoint_offsets)}]: '
                f'route_origin=({self.route_anchor_x:.3f},{self.route_anchor_y:.3f}), '
                f'route_yaw={math.degrees(self.route_anchor_yaw):.1f}deg, '
                f'frame={self.route_anchor_frame or self.default_frame_id}, '
                f'start=({segment_start_x:.3f},{segment_start_y:.3f}), '
                f'goal=({goal_x:.3f},{goal_y:.3f}), '
                f'segment_delta=({segment_dx:.3f},{segment_dy:.3f}), '
                f'base=[{self.forward_tracking_base_left_norm:.3f},'
                f'{self.forward_tracking_base_right_norm:.3f}], '
                f'gain_lat={self._forward_tracking_lateral_gain_for_current_waypoint():.2f}, '
                f'gain_yaw={self.forward_tracking_yaw_gain:.2f}, '
                f'arrival_lat_tol='
                f'{self._forward_tracking_arrival_lateral_tolerance_for_current_waypoint():.2f}m, '
                f'along={along:.3f}m remaining={remaining:.3f}m '
                f'lateral={lateral_error:+.3f}m yaw_err={math.degrees(yaw_error):+.1f}deg '
                f'corr={delta_norm:+.3f} override=[{forward_mode},{left_norm:.3f},{right_norm:.3f}]'
            )
            return
        if use_forward_override:
            forward_mode = self._forward_mode_for_waypoint(self.current_idx)
            left_norm, right_norm = self._forward_override_values(self.current_idx)
            self._release_startup_route_handoff('forward_override_started', log_release=False)
            self._publish_step_override(forward_mode, left_norm, right_norm)
            self._publish_state('FORWARD_STEP_NAVIGATING')
            # 2026-06-28 09:06 CST
            # Reason: pole pre-run waypoint should preserve body-heading travel;
            # pose_controller heading correction was steering it into an arc.
            # Purpose: keep /goal_pose for progress/arrival logging while the
            # serial bridge receives a fixed forward gait override.
            # Effect: field logs should show forward_override and step_debug
            # override=true with balanced left/right norms. Rollback: disable
            # waypoint_use_forward_step_override for this waypoint.
            if body_relative_offset is not None:
                target_text = (
                    f'body_offset=(forward={body_forward_m:.2f}, left={body_left_m:.2f}), '
                    f'world_delta=({segment_dx:.2f},{segment_dy:.2f})'
                )
            else:
                target_text = (
                    f'segment_offset=({segment_dx:.2f},{segment_dy:.2f}), '
                    f'cumulative_offset=({cumulative_dx:.2f},{cumulative_dy:.2f})'
                )
            self._publish_feedback(
                f'发送 HSV 绕杆前进直驱点[{waypoint_number}/{len(self.waypoint_offsets)}]: '
                f'{target_text}, goal=({goal_x:.3f},{goal_y:.3f}), '
                f'frame={msg.header.frame_id}, yaw={math.degrees(nav_goal_yaw):.1f}deg, '
                f'arrival_tolerance={arrival_tolerance:.2f}m, '
                f'forward_override=[{forward_mode},{left_norm:.3f},{right_norm:.3f}]'
            )
            return
        if use_direct_override:
            # 2026-05-03 03:43 CST: measured turn points may still be several
            # decimeters away if the previous waypoint was accepted with loose
            # tolerance. Publish the /goal_pose first, and arm direct override
            # only once Odometry is within direct_step_start_distance_m.
            # Rollback: set direct_step_start_distance_m high, or set
            # waypoint_use_direct_step_override to all false in YAML.
            self._publish_state('NAVIGATING_TO_DIRECT_STEP')
            if body_relative_offset is not None:
                self._publish_feedback(
                    f'发送机身相对转向接近点[{waypoint_number}/{len(self.waypoint_offsets)}]: '
                    f'body_offset=(forward={body_forward_m:.2f}, left={body_left_m:.2f}), '
                    f'world_delta=({segment_dx:.2f},{segment_dy:.2f}), '
                    f'target=({goal_x:.3f},{goal_y:.3f}), '
                    f'approach_yaw={math.degrees(nav_goal_yaw):.1f}deg, '
                    f'arrival_yaw={math.degrees(arrival_yaw):.1f}deg, '
                    f'arrival_tolerance={arrival_tolerance:.2f}m, '
                    f'yaw_tolerance={math.degrees(self.yaw_tolerance_rad):.1f}deg, '
                    f'direct_start_distance={self.direct_step_start_distance_m:.2f}m'
                )
            else:
                self._publish_feedback(
                    f'发送 HSV 绕杆转向接近点[{waypoint_number}/{len(self.waypoint_offsets)}]: '
                    f'segment_offset=({segment_dx:.2f},{segment_dy:.2f}), '
                    f'cumulative_offset=({cumulative_dx:.2f},{cumulative_dy:.2f}), '
                    f'target=({goal_x:.3f},{goal_y:.3f}), '
                    f'approach_yaw={math.degrees(nav_goal_yaw):.1f}deg, '
                    f'arrival_yaw={math.degrees(arrival_yaw):.1f}deg, '
                    f'arrival_tolerance={arrival_tolerance:.2f}m, '
                    f'yaw_tolerance={math.degrees(self.yaw_tolerance_rad):.1f}deg, '
                    f'direct_start_distance={self.direct_step_start_distance_m:.2f}m'
                )
            return

        self._publish_state('NAVIGATING')
        forward_mode = self._forward_mode_for_waypoint(self.current_idx)
        if body_relative_offset is not None:
            self._publish_feedback(
                f'发送机身相对目标点[{waypoint_number}/{len(self.waypoint_offsets)}]: '
                f'body_offset=(forward={body_forward_m:.2f}, left={body_left_m:.2f}), '
                f'world_delta=({segment_dx:.2f},{segment_dy:.2f}), '
                f'goal=({goal_x:.3f},{goal_y:.3f}), '
                f'frame={msg.header.frame_id}, yaw={math.degrees(nav_goal_yaw):.1f}deg, '
                f'mode={forward_mode}, arrival_tolerance={arrival_tolerance:.2f}m'
            )
        else:
            self._publish_feedback(
                f'发送 HSV 绕杆目标点[{waypoint_number}/{len(self.waypoint_offsets)}]: '
                f'segment_offset=({segment_dx:.2f},{segment_dy:.2f}), '
                f'cumulative_offset=({cumulative_dx:.2f},{cumulative_dy:.2f}), '
                f'goal=({goal_x:.3f},{goal_y:.3f}), '
                f'frame={msg.header.frame_id}, yaw={math.degrees(nav_goal_yaw):.1f}deg, '
                f'mode={forward_mode}, arrival_tolerance={arrival_tolerance:.2f}m'
            )

    def _publish_active_goal_pose(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.active_goal_frame_id or self.default_frame_id
        msg.pose.position.x = self.active_goal_x
        msg.pose.position.y = self.active_goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation = self._quaternion_from_yaw(self.active_goal_nav_yaw)
        self.goal_pub.publish(msg)

    def _handle_goal_arrived(self, dist: float, reason: str) -> None:
        arrived_idx = self.current_idx
        self.active_goal = False
        self.active_goal_allow_pass_through = False
        self.active_goal_has_direct_override = False
        self.active_goal_direct_override = False
        self.active_goal_has_forward_override = False
        self.active_goal_has_forward_tracking = False
        self.active_goal_is_turn_primitive = False
        self.active_turn_phase = ''
        self.active_turn_settle_reason = ''
        self.active_turn_settle_dist = 0.0
        self.active_turn_direction_sign = 0
        self.active_turn_last_yaw = 0.0
        self.active_turn_progress = 0.0
        self.active_turn_target_delta = 0.0
        self.active_turn_uses_body_relative_yaw = False
        self.active_turn_uses_locked_yaw = False
        self.active_turn_body_relative_delta_rad = 0.0
        self.active_turn_pole_distance_gate_m = 0.0
        self._clear_step_override()
        self._publish_arrival(True)
        self._publish_feedback(
            f'到达 {self._route_source_label()} 绕杆目标点[{arrived_idx + 1}/{len(self.waypoint_offsets)}]: '
            f'dist={dist:.3f}m, reason={reason}'
        )
        self.current_idx += 1
        if self.current_idx >= len(self.waypoint_offsets):
            self._finish_route()
            return

        if self._current_waypoint_uses_turn_primitive(self.current_idx):
            self.awaiting_next_goal = False
            if self._route_start_prerequisites_ready():
                self._ensure_route_anchor()
                self._send_current_goal()
            else:
                self._publish_route_start_wait_state()
            return

        self.awaiting_next_goal = True
        self.next_goal_time = self._now_sec() + self.next_goal_delay_sec
        self._publish_state('ARRIVED_WAIT_NEXT')

    def _handle_goal_failed(self, reason: str) -> None:
        self.active_goal = False
        self.active_goal_allow_pass_through = False
        self.active_goal_has_direct_override = False
        self.active_goal_direct_override = False
        self.active_goal_has_forward_override = False
        self.active_goal_has_forward_tracking = False
        self.active_goal_is_turn_primitive = False
        self.active_turn_phase = ''
        self.active_turn_settle_reason = ''
        self.active_turn_settle_dist = 0.0
        self.active_turn_direction_sign = 0
        self.active_turn_last_yaw = 0.0
        self.active_turn_progress = 0.0
        self.active_turn_target_delta = 0.0
        self.active_turn_uses_body_relative_yaw = False
        self.active_turn_uses_locked_yaw = False
        self.active_turn_body_relative_delta_rad = 0.0
        self.active_turn_pole_distance_gate_m = 0.0
        self.route_failed = True
        self._release_startup_route_handoff('route_failed', log_release=False)
        self._publish_arrival(False)
        self._clear_step_override()
        self._set_route_mode_active(False)
        self._publish_mode(self.finish_mode)
        self._publish_state('FAILED')
        self._publish_feedback(
            f'{self._route_source_label()} 绕杆导航失败: waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)}, '
            f'reason={reason}'
        )

    def _should_run_pole_pre_align(self, source: str) -> bool:
        if not self.pole_pre_align_enabled:
            return False
        if source != 'apriltag':
            return False
        if self.trigger_source != 'apriltag':
            return False
        if self.pole_pre_align_active or self.pole_pre_align_done:
            return False
        return True

    def _reset_pole_pre_align_runtime(self) -> None:
        self.pole_pre_align_active = False
        self.pole_pre_align_done = False
        self.pole_pre_align_phase = ''
        self.pole_pre_align_start_sec = 0.0
        self.pole_pre_align_phase_start_sec = 0.0
        self.pole_pre_align_stop_until_sec = 0.0
        self.pole_pre_align_pending_reason = ''
        self.pole_pre_align_pending_source = ''
        self.pole_pre_align_locked_tag: Optional[AprilTagSnapshot] = None
        self.pole_pre_align_locked_raw_tag_x_m = 0.0
        self.pole_pre_align_locked_tag_x_m = 0.0
        self.pole_pre_align_locked_tag_z_m = 0.0
        self.pole_pre_align_locked_pitch_err_rad = 0.0
        self.pole_pre_align_pitch_samples: Deque[Tuple[float, float]] = deque(
            maxlen=self.pole_pre_align_pose_filter_window_size
        )
        self.pole_pre_align_last_pitch_sample_stamp_sec = 0.0
        self.pole_pre_align_last_raw_pitch_err_rad = math.nan
        self.pole_pre_align_last_filtered_pitch_err_rad = math.nan
        self.pole_pre_align_base_yaw: Optional[float] = None
        self.pole_pre_align_side_yaw: Optional[float] = None
        self.pole_pre_align_shift_motion_yaw: Optional[float] = None
        self.pole_pre_align_shift_walk_sign = 1.0
        self.pole_pre_align_shift_direction_sign = 0.0
        self.pole_pre_align_shift_m = 0.0
        self.pole_pre_align_lateral_shift_done = False
        self.pole_pre_align_lateral_estimate_frozen = False
        self.pole_pre_align_shift_plan_start_x: Optional[float] = None
        self.pole_pre_align_shift_plan_start_y: Optional[float] = None
        self.pole_pre_align_move_start_x: Optional[float] = None
        self.pole_pre_align_move_start_y: Optional[float] = None
        self.pole_pre_align_entry_start_x: Optional[float] = None
        self.pole_pre_align_entry_start_y: Optional[float] = None
        self.pole_pre_align_entry_start_tag_z_m = 0.0
        self.pole_pre_align_backup_start_x: Optional[float] = None
        self.pole_pre_align_backup_start_y: Optional[float] = None
        self.pole_pre_align_stable_since: Optional[float] = None
        self.pole_pre_align_last_turn_direction = 0
        self.pole_pre_align_pending_turn_direction = 0
        self.pole_pre_align_reverse_hold_until_sec = 0.0
        self.pole_pre_align_backup_retries = 0
        self.pole_pre_align_last_log_sec = 0.0
        self.pole_pre_align_force_anchor_once = False

    def _start_pole_pre_align(self, reason: str, source: str) -> None:
        if not self._pose_ready_for_goal():
            self._publish_route_start_wait_state()
            return
        if not self._apriltag_ready_for_anchor():
            self._publish_state('WAITING_APRILTAG_SNAPSHOT')
            self._publish_feedback('绕杆前预对齐等待新鲜 AprilTag TF。')
            return
        tag = self.latest_apriltag
        self._reset_pole_pre_align_runtime()
        if tag is not None:
            filtered_pitch_err = self._pole_pre_align_filtered_pitch_err(tag)
            self._lock_pole_pre_align_tag(
                tag,
                (
                    self._pole_pre_align_pitch_err(tag)
                    if filtered_pitch_err is None
                    else filtered_pitch_err
                ),
            )
        self.route_anchor_ready = False
        self.pole_pre_align_active = True
        self.pole_pre_align_pending_reason = reason
        self.pole_pre_align_pending_source = source
        self.pole_pre_align_start_sec = self._now_sec()
        initial_phase = (
            'PRE_ALIGN_FIXED_ABSOLUTE_YAW'
            if self._pole_pre_align_uses_fixed_absolute_yaw()
            else 'PRE_ALIGN_YAW_POSE_PITCH'
        )
        self._enter_pole_pre_align_phase(initial_phase, 'apriltag_route_start')

    def _enter_pole_pre_align_phase(self, phase: str, reason: str) -> None:
        now = self._now_sec()
        self.pole_pre_align_phase = phase
        self.pole_pre_align_phase_start_sec = now
        self.pole_pre_align_stop_until_sec = now + self.pole_pre_align_stop_hold_sec
        self.pole_pre_align_stable_since = None
        self._reset_pole_pre_align_turn_guard()
        self.pole_pre_align_last_log_sec = 0.0
        self._publish_state(phase)
        self._publish_feedback(
            f'绕杆前 AprilTag 预对齐进入 {phase}: reason={reason}, '
            f'{self._pole_pre_align_status_text()}'
        )

    def _run_pole_pre_align(self) -> None:
        now = self._now_sec()
        if self.pole_pre_align_total_timeout_sec > 0.0 and (
            now - self.pole_pre_align_start_sec
        ) > self.pole_pre_align_total_timeout_sec:
            self._fail_pole_pre_align('total_timeout')
            return
        if self.pole_pre_align_timeout_sec > 0.0 and (
            now - self.pole_pre_align_phase_start_sec
        ) > self.pole_pre_align_timeout_sec:
            self._pole_pre_align_recover_or_fail(f'{self.pole_pre_align_phase}_timeout')
            return
        if not self._pose_ready_for_goal():
            pose_age_text = 'nan'
            if self.last_pose_time is not None:
                pose_age = (
                    self.get_clock().now() - self.last_pose_time
                ).nanoseconds / 1e9
                pose_age_text = f'{pose_age:.2f}'
            wait_reason = 'pose_missing' if not self.have_pose else 'pose_stale'
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_pole_pre_align_progress(
                now,
                f'wait_odom reason={wait_reason} '
                f'pose_age={pose_age_text}s '
                f'timeout={self.pose_stale_timeout_sec:.2f}s '
                'action=hold_until_odom_or_phase_timeout '
                'override=[0,0.000,0.000]',
                force=True,
            )
            return

        if now < self.pole_pre_align_stop_until_sec:
            self._publish_step_override(0, 0.0, 0.0)
            return

        phase = self.pole_pre_align_phase
        if phase in ('PRE_ALIGN_YAW_POSE_PITCH', 'PRE_ALIGN_FIXED_ABSOLUTE_YAW'):
            self._run_pole_pre_align_yaw(now)
        elif phase == 'PRE_FINAL_YAW_POSE_PITCH':
            self._run_pole_pre_align_final_yaw(now)
        elif phase == 'PRE_SHIFT_TURN_OUT':
            self._run_pole_pre_align_turn_to_yaw(now, self.pole_pre_align_side_yaw, 'side_yaw')
        elif phase == 'PRE_SHIFT_MOVE':
            self._run_pole_pre_align_shift_move(now)
        elif phase == 'PRE_SHIFT_TURN_BACK':
            self._run_pole_pre_align_turn_to_yaw(now, self.pole_pre_align_base_yaw, 'base_yaw')
        elif phase == 'PRE_ENTRY_FORWARD':
            self._run_pole_pre_align_entry_forward(now)
        elif phase == 'PRE_VERIFY_TAG':
            self._run_pole_pre_align_verify(now)
        elif phase == 'PRE_BACKUP':
            self._run_pole_pre_align_backup(now)
        else:
            self._fail_pole_pre_align(f'unknown_phase={phase}')

    def _run_pole_pre_align_yaw(self, now: float) -> None:
        tag = self._fresh_pole_pre_align_tag()
        pitch_filter_ready = True
        if tag is None:
            if self.pole_pre_align_locked_tag is None:
                self._publish_step_override(0, 0.0, 0.0)
                self._publish_pole_pre_align_progress(
                    now,
                    '绕杆前角度对齐中: status=wait_tag reason=no_live_or_locked_tag '
                    f'metric={self.pole_pre_align_angle_metric} '
                    'override=[0,0.000,0.000]',
                )
                return
            if self._pole_pre_align_uses_fixed_absolute_yaw():
                # Fixed-yaw mode can keep converging from Odometry while the
                # Tag briefly drops out. A fresh Tag is still required below
                # before locking the lateral correction plan.
                pitch_err = self._pole_pre_align_pitch_err(
                    self.pole_pre_align_locked_tag
                )
                tag_raw_x = self.pole_pre_align_locked_raw_tag_x_m
                tag_x = self.pole_pre_align_locked_tag_x_m
                tag_z = self.pole_pre_align_locked_tag_z_m
                tag_source = 'locked_tag_heading_only'
            elif abs(
                self.pole_pre_align_locked_pitch_err_rad
            ) > self.pole_pre_align_angle_tolerance_rad:
                self._publish_step_override(0, 0.0, 0.0)
                self._publish_pole_pre_align_progress(
                    now,
                    '绕杆前角度对齐中: status=wait_live_tag '
                    'reason=locked_angle_not_aligned '
                    f'locked_angle_err={math.degrees(self.pole_pre_align_locked_pitch_err_rad):+.2f}/'
                    f'{math.degrees(self.pole_pre_align_angle_tolerance_rad):.2f}deg '
                    'override=[0,0.000,0.000]',
                )
                return
            else:
                pitch_err = self.pole_pre_align_locked_pitch_err_rad
                tag_raw_x = self.pole_pre_align_locked_raw_tag_x_m
                tag_x = self.pole_pre_align_locked_tag_x_m
                tag_z = self.pole_pre_align_locked_tag_z_m
                tag_source = 'locked_tag_after_loss'
        else:
            if self.pole_pre_align_lateral_estimate_frozen:
                self.pole_pre_align_locked_tag = tag
                self.pole_pre_align_locked_tag_z_m = tag.z_m
                pitch_err = self.pole_pre_align_locked_pitch_err_rad
                tag_raw_x = self._pole_pre_align_estimated_raw_tag_x()
                tag_x = self._pole_pre_align_estimated_tag_x()
                tag_source = 'live_tag_lateral_frozen'
            else:
                raw_pitch_err = self._pole_pre_align_pitch_err(tag)
                filtered_pitch_err = self._pole_pre_align_filtered_pitch_err(tag)
                pitch_filter_ready = filtered_pitch_err is not None
                pitch_err = (
                    raw_pitch_err
                    if filtered_pitch_err is None
                    else filtered_pitch_err
                )
                self._lock_pole_pre_align_tag(tag, pitch_err)
                tag_raw_x = tag.x_m
                tag_x = self._pole_pre_align_center_x_from_raw(tag.x_m)
                tag_source = 'live_tag_filtered'
            tag_z = tag.z_m
        if tag_z <= self.pole_pre_align_min_tag_z_m:
            self._pole_pre_align_recover_or_fail(
                f'yaw_tag_too_close tag_z={tag_z:.3f}m source={tag_source}'
            )
            return
        if not pitch_filter_ready:
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_pole_pre_align_progress(
                now,
                '绕杆前角度对齐中: status=wait_angle_filter '
                f'samples={len(self.pole_pre_align_pitch_samples)}/'
                f'{self.pole_pre_align_pose_filter_min_samples} '
                f'metric={self.pole_pre_align_angle_metric} '
                f'raw_angle_err={math.degrees(self.pole_pre_align_last_raw_pitch_err_rad):+.2f}deg '
                'override=[0,0.000,0.000]',
            )
            return
        pitch_within_tolerance, active_tolerance = (
            self._pole_pre_align_error_within_hold(pitch_err)
        )
        if pitch_within_tolerance:
            if self._pole_pre_align_uses_fixed_absolute_yaw() and tag is None:
                self.pole_pre_align_stable_since = None
                self._cancel_pole_pre_align_pending_reverse()
                self._publish_step_override(0, 0.0, 0.0)
                self._publish_pole_pre_align_progress(
                    now,
                    '绕杆前全局角已对齐，等待新鲜 Tag 锁定横向/距离: '
                    f'yaw_err={math.degrees(pitch_err):+.2f}deg '
                    'override=[0,0.000,0.000]',
                )
                return
            if self.pole_pre_align_stable_since is None:
                self.pole_pre_align_stable_since = now
            held = now - self.pole_pre_align_stable_since
            self._cancel_pole_pre_align_pending_reverse()
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_pole_pre_align_progress(
                now,
                '绕杆前角度对齐保持: '
                f'source={tag_source} metric={self.pole_pre_align_angle_metric} '
                f'angle_err={math.degrees(pitch_err):+.2f}/'
                f'{math.degrees(active_tolerance):.2f}deg '
                f'hold={held:.2f}/{self.pole_pre_align_angle_hold_sec:.2f}s '
                f'raw_tag_x={tag_raw_x:+.3f}m center_x={tag_x:+.3f}m '
                f'camera_left_offset={self.pole_pre_align_camera_left_offset_m:+.3f}m '
                f'tag_z={tag_z:.3f}m '
                'next=lateral_check override=[0,0.000,0.000]'
            )
            if held >= self.pole_pre_align_angle_hold_sec:
                self.pole_pre_align_base_yaw = self._pole_pre_align_reference_yaw()
                self.pole_pre_align_shift_plan_start_x = self.pose_x
                self.pole_pre_align_shift_plan_start_y = self.pose_y
                shift_m = min(
                    self.pole_pre_align_shift_max_m,
                    abs(tag_x) * self.pole_pre_align_shift_distance_scale,
                )
                self.pole_pre_align_shift_m = shift_m
                if (
                    abs(tag_x) <= self.pole_pre_align_lateral_tolerance_m
                    or shift_m <= self.pole_pre_align_shift_deadband_m
                ):
                    self.pole_pre_align_lateral_shift_done = True
                    frozen_raw_x, frozen_center_x = self._freeze_pole_pre_align_lateral_estimate()
                    self._enter_pole_pre_align_phase(
                        'PRE_VERIFY_TAG',
                        f'lateral_already_ok center_x={tag_x:+.3f}m '
                        f'raw_tag_x={tag_raw_x:+.3f}m '
                        f'frozen_center_x={frozen_center_x:+.3f}m '
                        f'frozen_raw_tag_x={frozen_raw_x:+.3f}m '
                        f'source={tag_source}',
                    )
                    return
                self.pole_pre_align_shift_direction_sign = 1.0 if tag_x > 0.0 else -1.0
                physical_shift_sign = (
                    self.pole_pre_align_shift_direction_sign
                    * self.pole_pre_align_lateral_control_sign
                )
                self.pole_pre_align_side_yaw = self._normalize_angle(
                    self.pole_pre_align_base_yaw
                    - physical_shift_sign * self.pole_pre_align_shift_yaw_rad
                )
                self.pole_pre_align_shift_motion_yaw = self.pole_pre_align_side_yaw
                self.pole_pre_align_shift_walk_sign = 1.0
                self.pole_pre_align_lateral_shift_done = False
                self._enter_pole_pre_align_phase(
                    'PRE_SHIFT_TURN_OUT',
                    f'横向补偿锁定 tag_side={self._pole_pre_align_side_text(self.pole_pre_align_shift_direction_sign)} '
                    f'move_side={self._pole_pre_align_side_text(physical_shift_sign)} '
                    f'center_x={tag_x:+.3f}m raw_tag_x={tag_raw_x:+.3f}m '
                    f'camera_left_offset={self.pole_pre_align_camera_left_offset_m:+.3f}m '
                    f'shift={shift_m:.3f}m '
                    f'lateral_control_sign={self.pole_pre_align_lateral_control_sign:+.0f} '
                    f'source={tag_source}',
                )
            return

        self.pole_pre_align_stable_since = None
        control_error = self._pole_pre_align_angle_control_error(pitch_err)
        left_norm, right_norm, fine_turn, cooldown_remaining = (
            self._pole_pre_align_guarded_pose_turn_norms(now, control_error)
        )
        if cooldown_remaining > 0.0:
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_pole_pre_align_progress(
                now,
                '绕杆前角度对齐反向冷却: '
                f'metric={self.pole_pre_align_angle_metric} '
                f'angle_err={math.degrees(pitch_err):+.2f}deg '
                f'control_err={math.degrees(control_error):+.2f}deg '
                f'remaining={cooldown_remaining:.2f}s '
                'override=[0,0.000,0.000]',
            )
            return
        self._publish_step_override(self.pole_pre_align_turn_mode, left_norm, right_norm)
        self._publish_pole_pre_align_progress(
            now,
            '绕杆前角度对齐中: '
            f'source={tag_source} metric={self.pole_pre_align_angle_metric} '
            f'raw_tag_x={tag_raw_x:+.3f}m center_x={tag_x:+.3f}m tag_z={tag_z:.3f}m '
            f'angle_err={math.degrees(pitch_err):+.2f}/'
            f'{math.degrees(self.pole_pre_align_angle_tolerance_rad):.2f}deg '
            f'target={self._pole_pre_align_angle_target_text()} '
            f'control_err={math.degrees(control_error):+.2f}deg '
            f'pose_turn_scale={self.pole_pre_align_pose_turn_scale:.2f} '
            f'fine_turn={fine_turn} override=[{self.pole_pre_align_turn_mode},'
            f'{left_norm:+.3f},{right_norm:+.3f}]'
        )

    def _run_pole_pre_align_turn_to_yaw(
        self,
        now: float,
        target_yaw: Optional[float],
        label: str,
    ) -> None:
        if target_yaw is None:
            self._fail_pole_pre_align(f'{label}_missing')
            return
        yaw_err = self._normalize_angle(target_yaw - self.pose_yaw)
        stage_text = self._pole_pre_align_turn_stage_text(label)
        yaw_within_tolerance, active_tolerance = (
            self._pole_pre_align_error_within_hold(yaw_err)
        )
        if yaw_within_tolerance:
            if self.pole_pre_align_stable_since is None:
                self.pole_pre_align_stable_since = now
            held = now - self.pole_pre_align_stable_since
            self._cancel_pole_pre_align_pending_reverse()
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_pole_pre_align_progress(
                now,
                f'{stage_text}: status=hold label={label} '
                f'yaw_err={math.degrees(yaw_err):+.2f}/'
                f'{math.degrees(active_tolerance):.2f}deg '
                f'yaw_rate={math.degrees(self.pose_yaw_rate_rad_s):+.1f}deg/s '
                f'yaw_rate_raw={math.degrees(self.pose_yaw_rate_raw_rad_s):+.1f}deg/s '
                f'current_yaw={math.degrees(self.pose_yaw):+.1f}deg '
                f'target_yaw={math.degrees(target_yaw):+.1f}deg '
                f'hold={held:.2f}/{self.pole_pre_align_angle_hold_sec:.2f}s '
                f'next={self._pole_pre_align_turn_next_phase_text(label)} '
                f'override=[0,0.000,0.000]'
            )
            if held >= self.pole_pre_align_angle_hold_sec:
                if self.pole_pre_align_phase == 'PRE_SHIFT_TURN_OUT':
                    self._prepare_pole_pre_align_shift_move_after_turn(label)
                else:
                    self._enter_pole_pre_align_phase(
                        'PRE_FINAL_YAW_POSE_PITCH',
                        f'{label}_reached; final_heading_check_before_verify',
                    )
            return

        self.pole_pre_align_stable_since = None
        left_norm, right_norm, fine_turn, cooldown_remaining = (
            self._pole_pre_align_guarded_turn_norms(now, yaw_err)
        )
        if cooldown_remaining > 0.0:
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_pole_pre_align_progress(
                now,
                f'{stage_text}: status=reverse_cooldown label={label} '
                f'yaw_err={math.degrees(yaw_err):+.2f}deg '
                f'remaining={cooldown_remaining:.2f}s '
                'override=[0,0.000,0.000]',
            )
            return
        self._publish_step_override(self.pole_pre_align_turn_mode, left_norm, right_norm)
        base_yaw_text = (
            'nan'
            if self.pole_pre_align_base_yaw is None
            else f'{math.degrees(self.pole_pre_align_base_yaw):+.1f}deg'
        )
        self._publish_pole_pre_align_progress(
            now,
            f'{stage_text}: status=turning label={label} '
            f'yaw_err={math.degrees(yaw_err):+.2f}/'
            f'{math.degrees(self.pole_pre_align_angle_tolerance_rad):.2f}deg '
            f'yaw_rate={math.degrees(self.pose_yaw_rate_rad_s):+.1f}deg/s '
            f'yaw_rate_raw={math.degrees(self.pose_yaw_rate_raw_rad_s):+.1f}deg/s '
            f'current_yaw={math.degrees(self.pose_yaw):+.1f}deg '
            f'target_yaw={math.degrees(target_yaw):+.1f}deg '
            f'base_yaw={base_yaw_text} '
            f'fine_turn={fine_turn} override=[{self.pole_pre_align_turn_mode},'
            f'{left_norm:+.3f},{right_norm:+.3f}]'
        )

    def _run_pole_pre_align_final_yaw(self, now: float) -> None:
        tag = self._fresh_pole_pre_align_tag()
        pitch_filter_ready = True
        if tag is None:
            if self.pole_pre_align_locked_tag is None:
                self._publish_step_override(0, 0.0, 0.0)
                self._publish_pole_pre_align_progress(
                    now,
                    '绕杆前最终角度修正中: status=wait_tag '
                    f'reason=no_live_or_locked_tag metric={self.pole_pre_align_angle_metric} '
                    'override=[0,0.000,0.000]',
                    force=True,
                )
                return
            pitch_err = self.pole_pre_align_locked_pitch_err_rad
            tag_raw_x = self._pole_pre_align_estimated_raw_tag_x()
            tag_x = self._pole_pre_align_estimated_tag_x()
            tag_z = self._pole_pre_align_estimated_tag_z()
            tag_source = 'locked_estimate'
        else:
            if self.pole_pre_align_lateral_estimate_frozen:
                self.pole_pre_align_locked_tag = tag
                self.pole_pre_align_locked_tag_z_m = tag.z_m
                pitch_err = self.pole_pre_align_locked_pitch_err_rad
                tag_raw_x = self._pole_pre_align_estimated_raw_tag_x()
                tag_x = self._pole_pre_align_estimated_tag_x()
                tag_source = 'live_tag_lateral_frozen'
            else:
                raw_pitch_err = self._pole_pre_align_pitch_err(tag)
                filtered_pitch_err = self._pole_pre_align_filtered_pitch_err(tag)
                pitch_filter_ready = filtered_pitch_err is not None
                pitch_err = (
                    raw_pitch_err
                    if filtered_pitch_err is None
                    else filtered_pitch_err
                )
                self._lock_pole_pre_align_tag(tag, pitch_err)
                tag_raw_x = tag.x_m
                tag_x = self._pole_pre_align_center_x_from_raw(tag.x_m)
                tag_source = 'live_tag_filtered'
            tag_z = tag.z_m

        if tag_z <= self.pole_pre_align_min_tag_z_m:
            self._pole_pre_align_recover_or_fail(
                f'final_yaw_tag_too_close tag_z={tag_z:.3f}m source={tag_source}'
            )
            return
        if not pitch_filter_ready:
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_pole_pre_align_progress(
                now,
                '绕杆前最终角度修正中: status=wait_angle_filter '
                f'samples={len(self.pole_pre_align_pitch_samples)}/'
                f'{self.pole_pre_align_pose_filter_min_samples} '
                f'metric={self.pole_pre_align_angle_metric} '
                'override=[0,0.000,0.000]',
                force=True,
            )
            return

        yaw_err = (
            0.0
            if self.pole_pre_align_base_yaw is None
            else self._normalize_angle(self.pole_pre_align_base_yaw - self.pose_yaw)
        )
        active_tolerance = self._pole_pre_align_active_hold_tolerance()
        pitch_ok = (
            True
            if self._pole_pre_align_uses_fixed_absolute_yaw()
            else abs(pitch_err) <= active_tolerance
        )
        yaw_ok = abs(yaw_err) <= active_tolerance

        if pitch_ok and yaw_ok:
            if self.pole_pre_align_stable_since is None:
                self.pole_pre_align_stable_since = now
            held = now - self.pole_pre_align_stable_since
            self._cancel_pole_pre_align_pending_reverse()
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_pole_pre_align_progress(
                now,
                '绕杆前最终角度修正保持: '
                f'source={tag_source} metric={self.pole_pre_align_angle_metric} '
                f'angle_err={math.degrees(pitch_err):+.2f}/'
                f'{math.degrees(active_tolerance):.2f}deg '
                f'yaw_err={math.degrees(yaw_err):+.2f}/'
                f'{math.degrees(active_tolerance):.2f}deg '
                f'raw_tag_x={tag_raw_x:+.3f}m center_x={tag_x:+.3f}m '
                f'tag_z={tag_z:.3f}m '
                f'hold={held:.2f}/{self.pole_pre_align_angle_hold_sec:.2f}s '
                'next=PRE_VERIFY_TAG override=[0,0.000,0.000]',
                force=True,
            )
            if held >= self.pole_pre_align_angle_hold_sec:
                self._enter_pole_pre_align_phase(
                    'PRE_VERIFY_TAG',
                    'final_heading_aligned',
                )
            return

        self.pole_pre_align_stable_since = None
        if not yaw_ok and self.pole_pre_align_base_yaw is not None:
            control_error = yaw_err
            metric = 'base_yaw'
        else:
            control_error = self._pole_pre_align_angle_control_error(pitch_err)
            metric = self.pole_pre_align_angle_metric
        left_norm, right_norm, fine_turn, cooldown_remaining = (
            self._pole_pre_align_guarded_pose_turn_norms(now, control_error)
        )
        if cooldown_remaining > 0.0:
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_pole_pre_align_progress(
                now,
                '绕杆前最终角度修正反向冷却: '
                f'metric={metric} control_err={math.degrees(control_error):+.2f}deg '
                f'remaining={cooldown_remaining:.2f}s '
                'override=[0,0.000,0.000]',
                force=True,
            )
            return
        self._publish_step_override(self.pole_pre_align_turn_mode, left_norm, right_norm)
        self._publish_pole_pre_align_progress(
            now,
            '绕杆前最终角度修正中: '
            f'source={tag_source} metric={metric} '
            f'raw_tag_x={tag_raw_x:+.3f}m center_x={tag_x:+.3f}m tag_z={tag_z:.3f}m '
            f'angle_err={math.degrees(pitch_err):+.2f}/'
            f'{math.degrees(self.pole_pre_align_angle_tolerance_rad):.2f}deg '
            f'yaw_err={math.degrees(yaw_err):+.2f}/'
            f'{math.degrees(self.pole_pre_align_angle_tolerance_rad):.2f}deg '
            f'control_err={math.degrees(control_error):+.2f}deg '
            f'pose_turn_scale={self.pole_pre_align_pose_turn_scale:.2f} '
            f'fine_turn={fine_turn} override=[{self.pole_pre_align_turn_mode},'
            f'{left_norm:+.3f},{right_norm:+.3f}]',
            force=True,
        )

    def _run_pole_pre_align_shift_move(self, now: float) -> None:
        if (
            self.pole_pre_align_move_start_x is None
            or self.pole_pre_align_move_start_y is None
            or self.pole_pre_align_side_yaw is None
            or self.pole_pre_align_shift_motion_yaw is None
            or self.pole_pre_align_base_yaw is None
        ):
            self._fail_pole_pre_align('shift_not_locked')
            return
        along_m = self._pole_pre_align_signed_progress_along(
            self.pole_pre_align_move_start_x,
            self.pole_pre_align_move_start_y,
            self.pole_pre_align_shift_motion_yaw,
        )
        remaining_m = max(self.pole_pre_align_shift_m - along_m, 0.0)
        if along_m >= self.pole_pre_align_shift_m:
            frozen_raw_x, frozen_center_x = self._freeze_pole_pre_align_lateral_estimate()
            self._publish_step_override(0, 0.0, 0.0)
            self._enter_pole_pre_align_phase(
                'PRE_SHIFT_TURN_BACK',
                f'横向补偿完成 along={along_m:.3f}m '
                f'remaining={remaining_m:.3f}m '
                f'frozen_raw_tag_x={frozen_raw_x:+.3f}m '
                f'frozen_center_x={frozen_center_x:+.3f}m '
                'next=turn_back_base_yaw',
            )
            return
        left_norm, right_norm, yaw_corr = self._pole_pre_align_walk_norms(
            self.pole_pre_align_side_yaw,
            self.pole_pre_align_walk_left_norm * self.pole_pre_align_shift_walk_sign,
            self.pole_pre_align_walk_right_norm * self.pole_pre_align_shift_walk_sign,
        )
        yaw_err = self._normalize_angle(self.pole_pre_align_side_yaw - self.pose_yaw)
        physical_shift_sign = (
            self.pole_pre_align_shift_direction_sign
            * self.pole_pre_align_lateral_control_sign
        )
        self._publish_step_override(self.pole_pre_align_walk_mode, left_norm, right_norm)
        self._publish_pole_pre_align_progress(
            now,
            '绕杆前横向补偿中: status=walking '
            f'tag_side={self._pole_pre_align_side_text(self.pole_pre_align_shift_direction_sign)} '
            f'move_side={self._pole_pre_align_side_text(physical_shift_sign)} '
            f'locked_raw_tag_x={self.pole_pre_align_locked_raw_tag_x_m:+.3f}m '
            f'locked_center_x={self.pole_pre_align_locked_tag_x_m:+.3f}m '
            f'est_center_x={self._pole_pre_align_estimated_tag_x():+.3f}/'
            f'{self.pole_pre_align_lateral_tolerance_m:.3f}m '
            f'target_shift={self.pole_pre_align_shift_m:.3f}m '
            f'along_m={along_m:.3f}m remaining_m={remaining_m:.3f}m '
            f'base_yaw={math.degrees(self.pole_pre_align_base_yaw):+.1f}deg '
            f'side_yaw={math.degrees(self.pole_pre_align_side_yaw):+.1f}deg '
            f'motion_yaw={math.degrees(self.pole_pre_align_shift_motion_yaw):+.1f}deg '
            f'walk_sign={self.pole_pre_align_shift_walk_sign:+.0f} '
            f'yaw_err={math.degrees(yaw_err):+.2f}deg '
            f'yaw_corr={yaw_corr:+.3f} '
            f'yaw_correction={self.pole_pre_align_walk_yaw_correction_enabled} '
            f'override=[{self.pole_pre_align_walk_mode},{left_norm:+.3f},{right_norm:+.3f}]'
        )

    def _run_pole_pre_align_verify(self, now: float) -> None:
        tag_x, tag_z, pitch_err, tag_source, tag_raw_x = (
            self._pole_pre_align_verify_measurement(now)
        )
        if tag_source == 'waiting_live_tag':
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_pole_pre_align_progress(
                now,
                'verify_wait_tag override=[0,0.000,0.000]',
            )
            return
        if tag_source == 'none':
            self._pole_pre_align_recover_or_fail('verify_no_tag')
            return
        if tag_source == 'locked_estimate_after_timeout':
            self._publish_pole_pre_align_progress(
                now,
                'verify_tag_timeout_using_locked_estimate',
                force=True,
            )
        yaw_err = (
            0.0
            if self.pole_pre_align_base_yaw is None
            else self._normalize_angle(self.pole_pre_align_base_yaw - self.pose_yaw)
        )
        lateral_ok = abs(tag_x) <= self.pole_pre_align_lateral_tolerance_m
        # Keep lateral in the diagnostic log, but do not block the route after
        # the one-shot pre-align shift. Field runs can leave a few centimeters
        # of residual tag_x while the chassis is already sufficiently aligned
        # to continue into entry-forward / the locked 14-point route.
        lateral_gate_ok = True
        pitch_ok = (
            True
            if self._pole_pre_align_uses_fixed_absolute_yaw()
            else abs(pitch_err) <= self.pole_pre_align_angle_tolerance_rad
        )
        yaw_ok = abs(yaw_err) <= self.pole_pre_align_angle_tolerance_rad
        distance_ok = tag_z > self.pole_pre_align_min_tag_z_m
        entry_z_ok, entry_z_reason = self._pole_pre_align_entry_tag_z_ok(tag_z)
        failed_gates = self._pole_pre_align_verify_failed_gates(
            pitch_ok,
            lateral_gate_ok,
            yaw_ok,
            distance_ok,
            entry_z_ok,
        )
        next_action = 'start_original_route'
        if failed_gates:
            next_action = self._pole_pre_align_verify_next_action_text(
                failed_gates,
                tag_z,
            )
        self._publish_step_override(0, 0.0, 0.0)
        self._publish_pole_pre_align_progress(
            now,
            f'verify source={tag_source} raw_tag_x={tag_raw_x:+.3f}m '
            f'center_x={tag_x:+.3f}m tag_z={tag_z:.3f}m '
            f'angle_metric={self.pole_pre_align_angle_metric} '
            f'angle_err={math.degrees(pitch_err):+.2f}deg/'
            f'{math.degrees(self.pole_pre_align_angle_tolerance_rad):.2f}deg '
            f'lateral_err(center_x)={tag_x:+.3f}m/'
            f'{self.pole_pre_align_lateral_tolerance_m:.3f}m '
            f'camera_left_offset={self.pole_pre_align_camera_left_offset_m:+.3f}m '
            f'yaw_err={math.degrees(yaw_err):+.2f}deg/'
            f'{math.degrees(self.pole_pre_align_angle_tolerance_rad):.2f}deg '
            f'min_tag_z={self.pole_pre_align_min_tag_z_m:.3f}m '
            f'entry_gate={self._pole_pre_align_entry_gate_text()} '
            f'angle_ok={pitch_ok} tag_pose_ignored='
            f'{self._pole_pre_align_uses_fixed_absolute_yaw()} '
            f'lateral_ok={lateral_ok} '
            f'lateral_gate_ok={lateral_gate_ok} '
            f'lateral_shift_done={self.pole_pre_align_lateral_shift_done} '
            f'yaw_ok={yaw_ok} '
            f'distance_ok={distance_ok} entry_z_ok={entry_z_ok} '
            f'failed={failed_gates or ["none"]} next={next_action} '
            f'entry_z_reason={entry_z_reason} override=[0,0.000,0.000]',
            force=True,
        )
        if pitch_ok and lateral_gate_ok and yaw_ok and distance_ok and entry_z_ok:
            self._complete_pole_pre_align()
            return
        if not pitch_ok or not yaw_ok:
            self._enter_pole_pre_align_phase(
                'PRE_FINAL_YAW_POSE_PITCH',
                f'verify_realign angle_ok={pitch_ok} yaw_ok={yaw_ok} '
                f'source={tag_source}',
            )
            return
        if not lateral_gate_ok:
            self._fail_pole_pre_align(
                f'verify_lateral_out_of_gate_after_single_shift '
                f'center_x={tag_x:+.3f}m raw_tag_x={tag_raw_x:+.3f}m '
                f'tol={self.pole_pre_align_lateral_tolerance_m:.3f}m '
                f'lateral_shift_done={self.pole_pre_align_lateral_shift_done} '
                f'source={tag_source}'
            )
            return
        if not distance_ok:
            self._pole_pre_align_recover_or_fail(
                f'verify_tag_too_close tag_z={tag_z:.3f}m '
                f'min={self.pole_pre_align_min_tag_z_m:.3f}m source={tag_source}'
            )
            return
        if not entry_z_ok:
            if self._maybe_start_pole_pre_align_entry_forward(tag_z, tag_source, entry_z_reason):
                return
            self._pole_pre_align_recover_or_fail(
                f'verify_entry_z_failed source={tag_source} {entry_z_reason}'
            )
            return
        reasons = []
        if not pitch_ok:
            reasons.append(f'angle_err={math.degrees(pitch_err):+.2f}deg')
        if not lateral_ok:
            reasons.append(f'center_x={tag_x:+.3f}m')
        if not yaw_ok:
            reasons.append(f'yaw_err={math.degrees(yaw_err):+.2f}deg')
        if not distance_ok:
            reasons.append(f'tag_z={tag_z:.3f}m')
        if not entry_z_ok:
            reasons.append(entry_z_reason)
        self._pole_pre_align_recover_or_fail(
            'verify_failed source=' + tag_source + ' ' + ','.join(reasons)
        )

    def _pole_pre_align_entry_tag_z_ok(self, tag_z: float) -> Tuple[bool, str]:
        if self.pole_pre_align_entry_tag_z_m <= 0.0:
            return True, 'entry_tag_z_disabled'
        if not math.isfinite(tag_z):
            return False, 'entry_tag_z_invalid'
        tolerance = self.pole_pre_align_entry_tag_z_tolerance_m
        target = self.pole_pre_align_entry_tag_z_m
        max_allowed = target + tolerance
        if tag_z > max_allowed:
            return False, f'entry_tag_z_too_far {tag_z:.3f}>{max_allowed:.3f}'
        return True, f'entry_tag_z_reached tag_z={tag_z:.3f}<={max_allowed:.3f} target={target:.3f}+tol={tolerance:.3f}'

    def _pole_pre_align_verify_next_action_text(
        self,
        failed_gates: List[str],
        tag_z: float,
    ) -> str:
        if 'pitch' in failed_gates or 'yaw' in failed_gates:
            return 'final_heading_realign'
        if 'min_tag_z' in failed_gates:
            return self._pole_pre_align_recovery_action_text()
        if failed_gates == ['entry_z'] and self._pole_pre_align_entry_too_far(tag_z):
            return 'entry_forward'
        return self._pole_pre_align_recovery_action_text()

    def _pole_pre_align_entry_too_far(self, tag_z: float) -> bool:
        if self.pole_pre_align_entry_tag_z_m <= 0.0 or not math.isfinite(tag_z):
            return False
        return tag_z > (
            self.pole_pre_align_entry_tag_z_m
            + self.pole_pre_align_entry_tag_z_tolerance_m
        )

    def _pole_pre_align_side_text(self, sign: float) -> str:
        if sign > 1e-6:
            return 'right'
        if sign < -1e-6:
            return 'left'
        return 'none'

    def _pole_pre_align_turn_stage_text(self, label: str) -> str:
        if label == 'side_yaw':
            return '绕杆前横向补偿转出中'
        if label == 'base_yaw':
            return '绕杆前横向补偿回正中'
        return f'绕杆前转向中({label})'

    def _pole_pre_align_turn_next_phase_text(self, label: str) -> str:
        if label == 'side_yaw':
            return 'PRE_SHIFT_MOVE'
        if label == 'base_yaw':
            return 'PRE_FINAL_YAW_POSE_PITCH'
        return 'unknown'

    def _prepare_pole_pre_align_shift_move_after_turn(self, label: str) -> None:
        if self.pole_pre_align_side_yaw is None or self.pole_pre_align_base_yaw is None:
            self._fail_pole_pre_align('shift_turn_reached_but_yaw_missing')
            return
        if (
            self.pole_pre_align_shift_plan_start_x is None
            or self.pole_pre_align_shift_plan_start_y is None
        ):
            self.pole_pre_align_shift_plan_start_x = self.pose_x
            self.pole_pre_align_shift_plan_start_y = self.pose_y

        raw_turn_drift_m = self._pole_pre_align_signed_progress_along(
            self.pole_pre_align_shift_plan_start_x,
            self.pole_pre_align_shift_plan_start_y,
            self.pole_pre_align_side_yaw,
        )
        turn_drift_m = raw_turn_drift_m if self.pole_pre_align_shift_rebase_use_turn_drift else 0.0
        if self.pole_pre_align_shift_rebase_use_turn_drift:
            rebased_raw_tag_x = self._pole_pre_align_estimated_raw_tag_x()
            rebase_source = 'locked_tag_estimate_with_turn_drift'
        else:
            rebased_raw_tag_x = self.pole_pre_align_locked_raw_tag_x_m
            rebase_source = 'locked_tag_no_turn_drift'
        rebased_center_x = self._pole_pre_align_center_x_from_raw(rebased_raw_tag_x)
        initial_shift_direction_sign = self.pole_pre_align_shift_direction_sign
        initial_physical_shift_sign = (
            initial_shift_direction_sign
            * self.pole_pre_align_lateral_control_sign
        )
        planned_shift_m = self.pole_pre_align_shift_m
        turn_drift_toward_initial_m = (
            initial_shift_direction_sign
            * initial_physical_shift_sign
            * turn_drift_m
        )
        residual_shift_m = max(
            0.0,
            planned_shift_m - max(0.0, turn_drift_toward_initial_m),
        )
        rebase_note = ''
        if (
            abs(initial_shift_direction_sign) > 1e-6
            and abs(rebased_center_x) > 1e-6
            and (rebased_center_x * initial_shift_direction_sign) < 0.0
        ):
            center_before_clamp = rebased_center_x
            raw_before_clamp = rebased_raw_tag_x
            # Do not allow the side-turn odom estimate to flip the original
            # tag side. Treat a sign flip as "crossed center", then turn back.
            rebased_raw_tag_x = rebased_raw_tag_x - rebased_center_x
            rebased_center_x = 0.0
            rebase_note = (
                'sign_flip_clamped '
                f'initial_tag_side={self._pole_pre_align_side_text(initial_shift_direction_sign)} '
                f'raw_before={raw_before_clamp:+.3f}m '
                f'center_before={center_before_clamp:+.3f}m '
            )

        self.pole_pre_align_locked_raw_tag_x_m = rebased_raw_tag_x
        self.pole_pre_align_locked_tag_x_m = rebased_center_x
        self.pole_pre_align_move_start_x = self.pose_x
        self.pole_pre_align_move_start_y = self.pose_y
        self.pole_pre_align_shift_plan_start_x = self.pose_x
        self.pole_pre_align_shift_plan_start_y = self.pose_y

        if (
            abs(rebased_center_x) <= self.pole_pre_align_lateral_tolerance_m
            and residual_shift_m <= self.pole_pre_align_shift_deadband_m
        ):
            self.pole_pre_align_shift_m = 0.0
            self.pole_pre_align_lateral_shift_done = True
            frozen_raw_x, frozen_center_x = self._freeze_pole_pre_align_lateral_estimate()
            self._enter_pole_pre_align_phase(
                'PRE_SHIFT_TURN_BACK',
                f'{label}_reached; turn_drift_already_aligned '
                f'rebase_source={rebase_source} '
                f'{rebase_note}'
                f'planned_shift={planned_shift_m:.3f}m '
                f'residual_shift={residual_shift_m:.3f}m '
                f'turn_drift={turn_drift_m:+.3f}m '
                f'raw_turn_drift={raw_turn_drift_m:+.3f}m '
                f'turn_toward_initial={turn_drift_toward_initial_m:+.3f}m '
                f'center_x={rebased_center_x:+.3f}m '
                f'raw_tag_x={rebased_raw_tag_x:+.3f}m '
                f'frozen_center_x={frozen_center_x:+.3f}m '
                f'frozen_raw_tag_x={frozen_raw_x:+.3f}m',
            )
            return

        use_residual_initial_side = (
            residual_shift_m > self.pole_pre_align_shift_deadband_m
            and abs(initial_shift_direction_sign) > 1e-6
            and abs(rebased_center_x) <= self.pole_pre_align_lateral_tolerance_m
        )
        if use_residual_initial_side:
            self.pole_pre_align_shift_direction_sign = initial_shift_direction_sign
            rebase_note += (
                'residual_keeps_initial_side '
                f'initial_tag_side={self._pole_pre_align_side_text(initial_shift_direction_sign)} '
            )
        else:
            self.pole_pre_align_shift_direction_sign = (
                1.0 if rebased_center_x > 0.0 else -1.0
            )
        physical_shift_sign = (
            self.pole_pre_align_shift_direction_sign
            * self.pole_pre_align_lateral_control_sign
        )
        desired_side_yaw = self._normalize_angle(
            self.pole_pre_align_base_yaw
            - physical_shift_sign * self.pole_pre_align_shift_yaw_rad
        )
        desired_yaw_err = self._normalize_angle(desired_side_yaw - self.pose_yaw)
        if abs(desired_yaw_err) > (math.pi / 2.0):
            self.pole_pre_align_side_yaw = self.pose_yaw
            self.pole_pre_align_shift_motion_yaw = desired_side_yaw
            self.pole_pre_align_shift_walk_sign = -1.0
            motion_text = 'reverse_walk_after_sign_flip'
        else:
            self.pole_pre_align_side_yaw = desired_side_yaw
            self.pole_pre_align_shift_motion_yaw = desired_side_yaw
            self.pole_pre_align_shift_walk_sign = 1.0
            motion_text = 'forward_walk'
        self.pole_pre_align_shift_m = min(
            self.pole_pre_align_shift_max_m,
            max(
                abs(rebased_center_x) * self.pole_pre_align_shift_distance_scale,
                residual_shift_m,
            ),
        )

        if self.pole_pre_align_shift_m <= 1e-6:
            self.pole_pre_align_lateral_shift_done = True
            frozen_raw_x, frozen_center_x = self._freeze_pole_pre_align_lateral_estimate()
            self._enter_pole_pre_align_phase(
                'PRE_SHIFT_TURN_BACK',
                f'{label}_reached; shift_rebased_zero '
                f'rebase_source={rebase_source} '
                f'{rebase_note}'
                f'turn_drift={turn_drift_m:+.3f}m '
                f'center_x={rebased_center_x:+.3f}m '
                f'frozen_center_x={frozen_center_x:+.3f}m '
                f'frozen_raw_tag_x={frozen_raw_x:+.3f}m',
            )
            return

        yaw_err = self._normalize_angle(self.pole_pre_align_side_yaw - self.pose_yaw)
        reason = (
            f'{label}_reached; shift_rebased_after_turn_drift '
            f'rebase_source={rebase_source} '
            f'{rebase_note}'
            f'turn_drift={turn_drift_m:+.3f}m '
            f'raw_turn_drift={raw_turn_drift_m:+.3f}m '
            f'turn_toward_initial={turn_drift_toward_initial_m:+.3f}m '
            f'planned_shift={planned_shift_m:.3f}m '
            f'residual_shift={residual_shift_m:.3f}m '
            f'center_x={rebased_center_x:+.3f}m '
            f'raw_tag_x={rebased_raw_tag_x:+.3f}m '
            f'shift={self.pole_pre_align_shift_m:.3f}m '
            f'tag_side={self._pole_pre_align_side_text(self.pole_pre_align_shift_direction_sign)} '
            f'move_side={self._pole_pre_align_side_text(physical_shift_sign)} '
            f'motion={motion_text} '
            f'yaw_err={math.degrees(yaw_err):+.2f}deg'
        )
        if abs(yaw_err) > self.pole_pre_align_angle_tolerance_rad:
            self._enter_pole_pre_align_phase('PRE_SHIFT_TURN_OUT', reason)
            return
        self._enter_pole_pre_align_phase('PRE_SHIFT_MOVE', reason)

    def _pole_pre_align_entry_gate_text(self) -> str:
        if self.pole_pre_align_entry_tag_z_m <= 0.0:
            return 'disabled'
        target = self.pole_pre_align_entry_tag_z_m
        tolerance = self.pole_pre_align_entry_tag_z_tolerance_m
        max_allowed = target + tolerance
        return (
            f'tag_z<={max_allowed:.3f}m '
            f'(target={target:.3f}+tol={tolerance:.3f}; min_safe={self.pole_pre_align_min_tag_z_m:.3f})'
        )

    def _pole_pre_align_verify_failed_gates(
        self,
        pitch_ok: bool,
        lateral_ok: bool,
        yaw_ok: bool,
        distance_ok: bool,
        entry_z_ok: bool,
    ) -> List[str]:
        failed = []
        if not pitch_ok:
            failed.append('pitch')
        if not lateral_ok:
            failed.append('lateral')
        if not yaw_ok:
            failed.append('yaw')
        if not distance_ok:
            failed.append('min_tag_z')
        if not entry_z_ok:
            failed.append('entry_z')
        return failed

    def _pole_pre_align_recovery_action_text(self) -> str:
        if (
            self.pole_pre_align_backup_enabled
            and self.pole_pre_align_backup_retries < self.pole_pre_align_backup_max_retries
            and self.pole_pre_align_backup_distance_m > 0.0
        ):
            return (
                f'backup retry={self.pole_pre_align_backup_retries + 1}/'
                f'{self.pole_pre_align_backup_max_retries}'
            )
        return 'fail_stop'

    def _maybe_start_pole_pre_align_entry_forward(
        self,
        tag_z: float,
        tag_source: str,
        entry_z_reason: str,
    ) -> bool:
        if self.pole_pre_align_entry_tag_z_m <= 0.0:
            return False
        if not self.pole_pre_align_entry_forward_enabled:
            return False
        if not math.isfinite(tag_z):
            return False
        max_allowed = (
            self.pole_pre_align_entry_tag_z_m
            + self.pole_pre_align_entry_tag_z_tolerance_m
        )
        if tag_z <= max_allowed:
            return False
        if self.pole_pre_align_base_yaw is None:
            self.pole_pre_align_base_yaw = self._pole_pre_align_reference_yaw()
        self.pole_pre_align_entry_start_x = self.pose_x
        self.pole_pre_align_entry_start_y = self.pose_y
        self.pole_pre_align_entry_start_tag_z_m = tag_z
        self._enter_pole_pre_align_phase(
            'PRE_ENTRY_FORWARD',
            f'{entry_z_reason}; source={tag_source}; '
            f'entry_gate={self._pole_pre_align_entry_gate_text()}',
        )
        return True

    def _run_pole_pre_align_entry_forward(self, now: float) -> None:
        if (
            self.pole_pre_align_entry_start_x is None
            or self.pole_pre_align_entry_start_y is None
            or self.pole_pre_align_base_yaw is None
        ):
            self._fail_pole_pre_align('entry_forward_not_locked')
            return
        along_m = self._pole_pre_align_progress_along(
            self.pole_pre_align_entry_start_x,
            self.pole_pre_align_entry_start_y,
            self.pole_pre_align_base_yaw,
        )
        estimated_tag_z = max(1e-3, self.pole_pre_align_entry_start_tag_z_m - along_m)
        live_tag = self._fresh_pole_pre_align_tag()
        if live_tag is not None:
            live_entry_z_ok, live_entry_z_reason = self._pole_pre_align_entry_tag_z_ok(
                live_tag.z_m
            )
            if live_entry_z_ok:
                if self.pole_pre_align_lateral_estimate_frozen:
                    self.pole_pre_align_locked_tag = live_tag
                    self.pole_pre_align_locked_tag_z_m = live_tag.z_m
                else:
                    self._lock_pole_pre_align_tag(live_tag)
                self._publish_step_override(0, 0.0, 0.0)
                self._enter_pole_pre_align_phase(
                    'PRE_FINAL_YAW_POSE_PITCH',
                    f'entry_forward_live_tag_done tag_z={live_tag.z_m:.3f}m '
                    f'along={along_m:.3f}m entry_reason={live_entry_z_reason}; '
                    'final_heading_check_before_verify',
                )
                return
        entry_z_ok, entry_z_reason = self._pole_pre_align_entry_tag_z_ok(estimated_tag_z)
        if entry_z_ok:
            self.pole_pre_align_locked_tag_z_m = estimated_tag_z
            self._publish_step_override(0, 0.0, 0.0)
            self._enter_pole_pre_align_phase(
                'PRE_FINAL_YAW_POSE_PITCH',
                f'entry_forward_done est_tag_z={estimated_tag_z:.3f}m '
                f'along={along_m:.3f}m entry_reason={entry_z_reason}; '
                'final_heading_check_before_verify',
            )
            return
        if (
            self.pole_pre_align_entry_forward_max_m > 0.0
            and along_m > self.pole_pre_align_entry_forward_max_m
        ):
            self._pole_pre_align_recover_or_fail(
                f'entry_forward_space_exhausted along={along_m:.3f}m>'
                f'{self.pole_pre_align_entry_forward_max_m:.3f}m '
                f'est_tag_z={estimated_tag_z:.3f}m reason={entry_z_reason}'
            )
            return
        if estimated_tag_z <= self.pole_pre_align_min_tag_z_m:
            self._pole_pre_align_recover_or_fail(
                f'entry_forward_too_close est_tag_z={estimated_tag_z:.3f}m'
            )
            return
        left_norm, right_norm, yaw_corr = self._pole_pre_align_walk_norms(
            self.pole_pre_align_base_yaw,
            self.pole_pre_align_entry_forward_left_norm,
            self.pole_pre_align_entry_forward_right_norm,
        )
        remaining_z = max(
            estimated_tag_z
            - (
                self.pole_pre_align_entry_tag_z_m
                + self.pole_pre_align_entry_tag_z_tolerance_m
            ),
            0.0,
        )
        self._publish_step_override(
            self.pole_pre_align_entry_forward_mode,
            left_norm,
            right_norm,
        )
        self._publish_pole_pre_align_progress(
            now,
            f'entry_forward target_z={self.pole_pre_align_entry_tag_z_m:.3f}m '
            f'entry_gate={self._pole_pre_align_entry_gate_text()} '
            f'est_tag_z={estimated_tag_z:.3f}m remaining_z={remaining_z:.3f}m '
            f'along_m={along_m:.3f}/{self.pole_pre_align_entry_forward_max_m:.3f}m '
            f'yaw_corr={yaw_corr:+.3f} override=['
            f'{self.pole_pre_align_entry_forward_mode},'
            f'{left_norm:+.3f},{right_norm:+.3f}]'
        )

    def _pole_pre_align_verify_measurement(
        self,
        now: float,
    ) -> Tuple[float, float, float, str, float]:
        tag = self._fresh_pole_pre_align_tag()
        if tag is not None:
            if self.pole_pre_align_lateral_estimate_frozen:
                self.pole_pre_align_locked_tag = tag
                self.pole_pre_align_locked_tag_z_m = tag.z_m
                return (
                    self._pole_pre_align_estimated_tag_x(),
                    tag.z_m,
                    self.pole_pre_align_locked_pitch_err_rad,
                    'live_tag_lateral_frozen',
                    self._pole_pre_align_estimated_raw_tag_x(),
                )
            filtered_pitch_err = self._pole_pre_align_filtered_pitch_err(tag)
            pitch_err = (
                self._pole_pre_align_pitch_err(tag)
                if filtered_pitch_err is None
                else filtered_pitch_err
            )
            self._lock_pole_pre_align_tag(tag, pitch_err)
            return (
                self._pole_pre_align_center_x_from_raw(tag.x_m),
                tag.z_m,
                pitch_err,
                'live_tag',
                tag.x_m,
            )

        if self.pole_pre_align_locked_tag is not None:
            return (
                self._pole_pre_align_estimated_tag_x(),
                self._pole_pre_align_estimated_tag_z(),
                self.pole_pre_align_locked_pitch_err_rad,
                'locked_estimate',
                self._pole_pre_align_estimated_raw_tag_x(),
            )

        if self.pole_pre_align_verify_require_tag:
            if (
                self.pole_pre_align_verify_timeout_sec <= 0.0
                or (now - self.pole_pre_align_phase_start_sec)
                < self.pole_pre_align_verify_timeout_sec
            ):
                return 0.0, 0.0, 0.0, 'waiting_live_tag', 0.0
            tag_source = 'locked_estimate_after_timeout'
        else:
            tag_source = 'locked_estimate'

        if self.pole_pre_align_locked_tag is None:
            return 0.0, 0.0, 0.0, 'none', 0.0
        return (
            self._pole_pre_align_estimated_tag_x(),
            self._pole_pre_align_estimated_tag_z(),
            self.pole_pre_align_locked_pitch_err_rad,
            tag_source,
            self._pole_pre_align_estimated_raw_tag_x(),
        )

    def _run_pole_pre_align_backup(self, now: float) -> None:
        if self.pole_pre_align_backup_start_x is None or self.pole_pre_align_backup_start_y is None:
            self._fail_pole_pre_align('backup_start_missing')
            return
        if self.pole_pre_align_base_yaw is None:
            self.pole_pre_align_base_yaw = self._pole_pre_align_reference_yaw()
        backup_yaw = self._normalize_angle(self.pole_pre_align_base_yaw + math.pi)
        along_m = self._pole_pre_align_progress_along(
            self.pole_pre_align_backup_start_x,
            self.pole_pre_align_backup_start_y,
            backup_yaw,
        )
        remaining_m = max(self.pole_pre_align_backup_distance_m - along_m, 0.0)
        if along_m >= self.pole_pre_align_backup_distance_m:
            self._publish_step_override(0, 0.0, 0.0)
            self._clear_pole_pre_align_locked_tag(reset_yaw_plan=True)
            self._enter_pole_pre_align_phase(
                'PRE_ALIGN_YAW_POSE_PITCH',
                f'backup_done retry={self.pole_pre_align_backup_retries}',
            )
            return
        left_norm, right_norm = self._pole_pre_align_small_move_norms(
            self.pole_pre_align_backup_left_norm,
            self.pole_pre_align_backup_right_norm,
        )
        yaw_corr = 0.0
        self._publish_step_override(self.pole_pre_align_backup_mode, left_norm, right_norm)
        self._publish_pole_pre_align_progress(
            now,
            f'backup retry={self.pole_pre_align_backup_retries}/'
            f'{self.pole_pre_align_backup_max_retries} along_m={along_m:.3f} '
            f'remaining_m={remaining_m:.3f} yaw_corr={yaw_corr:+.3f} '
            f'override=[{self.pole_pre_align_backup_mode},{left_norm:+.3f},{right_norm:+.3f}]'
        )

    def _complete_pole_pre_align(self) -> None:
        reason = self.pole_pre_align_pending_reason or 'pole_pre_align_done'
        source = self.pole_pre_align_pending_source or 'apriltag'
        self.pole_pre_align_active = False
        self.pole_pre_align_done = True
        self.pole_pre_align_phase = ''
        self.route_anchor_ready = False
        self.pole_pre_align_force_anchor_once = True
        self._clear_step_override()
        self._publish_feedback(
            '绕杆前 AprilTag 预对齐完成，准备从修正后的当前位置重新锁定 route anchor: '
            f'{self._pole_pre_align_status_text()}'
        )
        self._activate_route(f'{reason} + pole_pre_align_done', source)

    def _pole_pre_align_recover_or_fail(self, reason: str) -> None:
        if (
            self.pole_pre_align_backup_enabled
            and self.pole_pre_align_backup_retries < self.pole_pre_align_backup_max_retries
            and self.pole_pre_align_backup_distance_m > 0.0
            and self._pose_ready_for_goal()
        ):
            self.pole_pre_align_backup_retries += 1
            self.pole_pre_align_backup_start_x = self.pose_x
            self.pole_pre_align_backup_start_y = self.pose_y
            if self.pole_pre_align_base_yaw is None:
                self.pole_pre_align_base_yaw = self._pole_pre_align_reference_yaw()
            self._enter_pole_pre_align_phase(
                'PRE_BACKUP',
                f'{reason}; retry={self.pole_pre_align_backup_retries}',
            )
            return
        self._fail_pole_pre_align(reason)

    def _fail_pole_pre_align(self, reason: str) -> None:
        self.pole_pre_align_active = False
        self.pole_pre_align_done = False
        self.route_start_pending = False
        self.pending_route_reason = ''
        self.pending_route_source = ''
        self.route_failed = True
        self._clear_step_override()
        self._set_route_mode_active(False)
        self._publish_mode(self.finish_mode)
        self._publish_arrival(False)
        self._publish_state('FAILED')
        self._publish_feedback(
            f'绕杆前 AprilTag 预对齐失败，禁止进入原 14 点绕杆路线: '
            f'reason={reason}, {self._pole_pre_align_status_text()}'
        )

    def _fresh_pole_pre_align_tag(self) -> Optional[AprilTagSnapshot]:
        if not self._apriltag_ready_for_anchor():
            return None
        return self.latest_apriltag

    def _pole_pre_align_uses_fixed_absolute_yaw(self) -> bool:
        return self.pole_pre_align_angle_metric == 'fixed_absolute_yaw'

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

    def _pole_pre_align_fixed_target_yaw(self) -> float:
        nominal_yaw = float(self.pole_pre_align_fixed_yaw_rad)
        if self._route_yaw_reference_is_fresh():
            return self._normalize_angle(
                float(self.route_yaw_reference) + nominal_yaw
            )
        return self._normalize_angle(nominal_yaw)

    def _pole_pre_align_reference_yaw(self) -> float:
        if self._pole_pre_align_uses_fixed_absolute_yaw():
            return self._pole_pre_align_fixed_target_yaw()
        return self.pose_yaw

    def _pole_pre_align_angle_control_error(self, angle_error_rad: float) -> float:
        if self._pole_pre_align_uses_fixed_absolute_yaw():
            return angle_error_rad
        return self.pole_pre_align_pose_control_sign * angle_error_rad

    def _pole_pre_align_angle_target_text(self) -> str:
        if self._pole_pre_align_uses_fixed_absolute_yaw():
            target_yaw = self._pole_pre_align_fixed_target_yaw()
            if self._route_yaw_reference_is_fresh():
                return (
                    f'route_yaw={math.degrees(target_yaw):+.2f}deg '
                    f'(route_zero={math.degrees(self.route_yaw_reference):+.2f}deg, '
                    f'nominal={math.degrees(self.pole_pre_align_fixed_yaw_rad):+.2f}deg)'
                )
            return f'global_yaw={math.degrees(target_yaw):+.2f}deg'
        return f'pose_pitch={math.degrees(self.pole_pre_align_target_pitch_rad):+.2f}deg'

    def _pole_pre_align_pitch_err(self, tag: AprilTagSnapshot) -> float:
        if self._pole_pre_align_uses_fixed_absolute_yaw():
            return self._normalize_angle(
                self._pole_pre_align_fixed_target_yaw() - self.pose_yaw
            )
        return self._normalize_angle(tag.pitch_rad - self.pole_pre_align_target_pitch_rad)

    def _record_pole_pre_align_pitch_sample(self, tag: AprilTagSnapshot) -> None:
        stamp_sec = tag.stamp_sec if tag.stamp_sec > 0.0 else tag.received_sec
        if (
            self.pole_pre_align_pitch_samples
            and abs(stamp_sec - self.pole_pre_align_last_pitch_sample_stamp_sec) <= 1e-6
        ):
            return
        raw_pitch_err = self._pole_pre_align_pitch_err(tag)
        self.pole_pre_align_pitch_samples.append((stamp_sec, raw_pitch_err))
        self.pole_pre_align_last_pitch_sample_stamp_sec = stamp_sec
        self.pole_pre_align_last_raw_pitch_err_rad = raw_pitch_err
        if len(self.pole_pre_align_pitch_samples) >= self.pole_pre_align_pose_filter_min_samples:
            values = sorted(value for _, value in self.pole_pre_align_pitch_samples)
            mid = len(values) // 2
            if len(values) % 2:
                filtered = values[mid]
            else:
                filtered = 0.5 * (values[mid - 1] + values[mid])
            self.pole_pre_align_last_filtered_pitch_err_rad = filtered

    def _pole_pre_align_filtered_pitch_err(
        self,
        tag: AprilTagSnapshot,
    ) -> Optional[float]:
        self._record_pole_pre_align_pitch_sample(tag)
        if len(self.pole_pre_align_pitch_samples) < self.pole_pre_align_pose_filter_min_samples:
            return None
        return self.pole_pre_align_last_filtered_pitch_err_rad

    def _pole_pre_align_active_hold_tolerance(self) -> float:
        if self.pole_pre_align_stable_since is None:
            return self.pole_pre_align_angle_tolerance_rad
        return self.pole_pre_align_angle_exit_tolerance_rad

    def _pole_pre_align_error_within_hold(
        self,
        error_rad: float,
    ) -> Tuple[bool, float]:
        tolerance = self._pole_pre_align_active_hold_tolerance()
        angle_ready = abs(error_rad) <= tolerance
        yaw_rate_ready = (
            not getattr(self, 'pole_pre_align_precision_turn_enabled', False)
            or abs(getattr(self, 'pose_yaw_rate_rad_s', 0.0))
            <= self.pole_pre_align_turn_complete_yaw_rate_rad_s
        )
        return angle_ready and yaw_rate_ready, tolerance

    def _reset_pole_pre_align_turn_guard(self) -> None:
        self.pole_pre_align_last_turn_direction = 0
        self.pole_pre_align_pending_turn_direction = 0
        self.pole_pre_align_reverse_hold_until_sec = 0.0

    def _cancel_pole_pre_align_pending_reverse(self) -> None:
        self.pole_pre_align_pending_turn_direction = 0
        self.pole_pre_align_reverse_hold_until_sec = 0.0

    def _pole_pre_align_guarded_turn_norms(
        self,
        now: float,
        error_rad: float,
    ) -> Tuple[float, float, bool, float]:
        control_error = error_rad
        if getattr(self, 'pole_pre_align_precision_turn_enabled', False):
            yaw_rate = getattr(self, 'pose_yaw_rate_rad_s', 0.0)
            if error_rad * yaw_rate > 0.0:
                predicted_error = (
                    error_rad
                    - self.pole_pre_align_turn_yaw_damping_sec * yaw_rate
                )
                should_brake = (
                    abs(yaw_rate)
                    >= self.pole_pre_align_turn_brake_yaw_rate_rad_s
                    and (
                        predicted_error * error_rad <= 0.0
                        or abs(predicted_error)
                        <= self.pole_pre_align_angle_tolerance_rad
                    )
                )
                if should_brake:
                    return 0.0, 0.0, False, 0.0
                if predicted_error * error_rad > 0.0:
                    control_error = predicted_error

        requested_direction = 1 if control_error > 0.0 else -1
        if self.pole_pre_align_reverse_cooldown_sec > 0.0:
            if self.pole_pre_align_pending_turn_direction != 0:
                if requested_direction == self.pole_pre_align_last_turn_direction:
                    self._cancel_pole_pre_align_pending_reverse()
                elif requested_direction != self.pole_pre_align_pending_turn_direction:
                    self.pole_pre_align_pending_turn_direction = requested_direction
                    self.pole_pre_align_reverse_hold_until_sec = (
                        now + self.pole_pre_align_reverse_cooldown_sec
                    )
                    return 0.0, 0.0, False, self.pole_pre_align_reverse_cooldown_sec
                elif now < self.pole_pre_align_reverse_hold_until_sec:
                    return (
                        0.0,
                        0.0,
                        False,
                        self.pole_pre_align_reverse_hold_until_sec - now,
                    )
                else:
                    self.pole_pre_align_last_turn_direction = requested_direction
                    self._cancel_pole_pre_align_pending_reverse()
            elif (
                self.pole_pre_align_last_turn_direction != 0
                and requested_direction != self.pole_pre_align_last_turn_direction
            ):
                self.pole_pre_align_pending_turn_direction = requested_direction
                self.pole_pre_align_reverse_hold_until_sec = (
                    now + self.pole_pre_align_reverse_cooldown_sec
                )
                return 0.0, 0.0, False, self.pole_pre_align_reverse_cooldown_sec

        self.pole_pre_align_last_turn_direction = requested_direction
        left_norm, right_norm, fine_turn = self._pole_pre_align_turn_norms(
            control_error
        )
        return left_norm, right_norm, fine_turn, 0.0

    def _pole_pre_align_guarded_pose_turn_norms(
        self,
        now: float,
        error_rad: float,
    ) -> Tuple[float, float, bool, float]:
        left_norm, right_norm, fine_turn, cooldown_remaining = (
            self._pole_pre_align_guarded_turn_norms(now, error_rad)
        )
        if cooldown_remaining > 0.0:
            return left_norm, right_norm, fine_turn, cooldown_remaining
        return (
            self._clamp_norm(left_norm * self.pole_pre_align_pose_turn_scale),
            self._clamp_norm(right_norm * self.pole_pre_align_pose_turn_scale),
            fine_turn,
            0.0,
        )

    def _pole_pre_align_center_x_from_raw(self, raw_tag_x_m: float) -> float:
        # Field tests show this rig's camera/body lateral correction has the
        # opposite sign from the earlier assumption: a positive configured
        # offset should increase the body-relative tag_x used for lateral shift.
        return raw_tag_x_m + self.pole_pre_align_camera_left_offset_m

    def _lock_pole_pre_align_tag(
        self,
        tag: AprilTagSnapshot,
        pitch_err: Optional[float] = None,
    ) -> None:
        if pitch_err is None:
            pitch_err = self._pole_pre_align_pitch_err(tag)
        self.pole_pre_align_locked_tag = tag
        if self.pole_pre_align_lateral_estimate_frozen:
            self.pole_pre_align_locked_tag_z_m = tag.z_m
            return
        self.pole_pre_align_locked_raw_tag_x_m = tag.x_m
        self.pole_pre_align_locked_tag_x_m = self._pole_pre_align_center_x_from_raw(tag.x_m)
        self.pole_pre_align_locked_tag_z_m = tag.z_m
        self.pole_pre_align_locked_pitch_err_rad = pitch_err

    def _freeze_pole_pre_align_lateral_estimate(self) -> Tuple[float, float]:
        frozen_raw_x = self._pole_pre_align_estimated_raw_tag_x()
        frozen_center_x = self._pole_pre_align_center_x_from_raw(frozen_raw_x)
        self.pole_pre_align_locked_raw_tag_x_m = frozen_raw_x
        self.pole_pre_align_locked_tag_x_m = frozen_center_x
        self.pole_pre_align_lateral_estimate_frozen = True
        self.pole_pre_align_shift_plan_start_x = None
        self.pole_pre_align_shift_plan_start_y = None
        return frozen_raw_x, frozen_center_x

    def _clear_pole_pre_align_locked_tag(self, *, reset_yaw_plan: bool = False) -> None:
        self.pole_pre_align_locked_tag = None
        self.pole_pre_align_locked_raw_tag_x_m = 0.0
        self.pole_pre_align_locked_tag_x_m = 0.0
        self.pole_pre_align_locked_tag_z_m = 0.0
        self.pole_pre_align_locked_pitch_err_rad = 0.0
        if reset_yaw_plan:
            self.pole_pre_align_base_yaw = None
            self.pole_pre_align_side_yaw = None
            self.pole_pre_align_shift_motion_yaw = None
            self.pole_pre_align_shift_walk_sign = 1.0
            self.pole_pre_align_shift_direction_sign = 0.0
            self.pole_pre_align_shift_m = 0.0
            self.pole_pre_align_lateral_estimate_frozen = False
            self.pole_pre_align_shift_plan_start_x = None
            self.pole_pre_align_shift_plan_start_y = None
            self.pole_pre_align_move_start_x = None
            self.pole_pre_align_move_start_y = None
            self.pole_pre_align_entry_start_x = None
            self.pole_pre_align_entry_start_y = None
            self.pole_pre_align_entry_start_tag_z_m = 0.0

    def _pole_pre_align_turn_norms(self, error_rad: float) -> Tuple[float, float, bool]:
        fine_turn = abs(error_rad) <= self.pole_pre_align_fine_angle_threshold_rad
        ccw = error_rad > 0.0
        if fine_turn:
            if ccw:
                left_norm = self.pole_pre_align_fine_ccw_left_norm
                right_norm = self.pole_pre_align_fine_ccw_right_norm
            else:
                left_norm = self.pole_pre_align_fine_cw_left_norm
                right_norm = self.pole_pre_align_fine_cw_right_norm
        elif self.pole_pre_align_reuse_existing_turn_steps and self.turn_waypoint_slowdown_enabled:
            left_norm, right_norm = self._turn_slowdown_override(1 if ccw else -1)
        elif ccw:
            left_norm = self.pole_pre_align_ccw_left_norm
            right_norm = self.pole_pre_align_ccw_right_norm
        else:
            left_norm = self.pole_pre_align_cw_left_norm
            right_norm = self.pole_pre_align_cw_right_norm

        left_norm = self._clamp_norm(left_norm)
        right_norm = self._clamp_norm(right_norm)
        if not fine_turn and self.pole_pre_align_turn_scale_enabled:
            requested = min(
                self.pole_pre_align_turn_max_abs_angular_z,
                max(
                    self.pole_pre_align_turn_min_abs_angular_z,
                    abs(error_rad) * self.pole_pre_align_turn_yaw_gain_per_rad,
                ),
            )
            base = max(abs(left_norm), abs(right_norm), 1e-6)
            scale = max(self.pole_pre_align_turn_min_scale, requested / base)
            scale = min(scale, 1.0)
            left_norm = self._clamp_norm(left_norm * scale)
            right_norm = self._clamp_norm(right_norm * scale)
        if (
            getattr(self, 'pole_pre_align_precision_turn_enabled', False)
            and abs(error_rad) >= self.pole_pre_align_turn_far_error_rad
        ):
            left_norm = self._clamp_norm(
                left_norm * self.pole_pre_align_turn_far_scale
            )
            right_norm = self._clamp_norm(
                right_norm * self.pole_pre_align_turn_far_scale
            )
        return left_norm, right_norm, fine_turn

    def _pole_pre_align_walk_norms(
        self,
        target_yaw: Optional[float],
        base_left_norm: float,
        base_right_norm: float,
    ) -> Tuple[float, float, float]:
        left_norm, right_norm = self._pole_pre_align_small_move_norms(
            base_left_norm,
            base_right_norm,
        )
        if (
            not self.pole_pre_align_walk_yaw_correction_enabled
            or target_yaw is None
            or not self.have_pose
        ):
            return left_norm, right_norm, 0.0
        yaw_err = self._normalize_angle(target_yaw - self.pose_yaw)
        abs_err = abs(yaw_err)
        if abs_err <= self.pole_pre_align_walk_yaw_deadband_rad:
            return left_norm, right_norm, 0.0
        yaw_corr = math.copysign(
            min(
                self.pole_pre_align_walk_yaw_max_delta_norm,
                (abs_err - self.pole_pre_align_walk_yaw_deadband_rad)
                * self.pole_pre_align_walk_yaw_gain_norm_per_rad,
            ),
            yaw_err,
        )
        left_norm = self._clamp_norm(left_norm - yaw_corr)
        right_norm = self._clamp_norm(right_norm + yaw_corr)
        return left_norm, right_norm, yaw_corr

    def _pole_pre_align_small_move_norms(
        self,
        base_left_norm: float,
        base_right_norm: float,
    ) -> Tuple[float, float]:
        left_norm = self._clamp_norm(base_left_norm)
        right_norm = self._clamp_norm(base_right_norm)
        if not self.pole_pre_align_reuse_existing_small_move_steps:
            return left_norm, right_norm
        cap = abs(self.turn_waypoint_exit_forward_norm)
        if cap <= 1e-6:
            return left_norm, right_norm

        def capped(value: float) -> float:
            if abs(value) <= 1e-6:
                return 0.0
            return math.copysign(min(abs(value), cap), value)

        return self._clamp_norm(capped(left_norm)), self._clamp_norm(capped(right_norm))

    def _pole_pre_align_signed_progress_along(
        self,
        start_x: float,
        start_y: float,
        target_yaw: float,
    ) -> float:
        unit_x, unit_y = self._body_relative_offset_to_world_from_yaw(target_yaw, 1.0, 0.0)
        length = math.hypot(unit_x, unit_y)
        if length <= 1e-6:
            return 0.0
        unit_x /= length
        unit_y /= length
        rel_x = self.pose_x - start_x
        rel_y = self.pose_y - start_y
        return rel_x * unit_x + rel_y * unit_y

    def _pole_pre_align_progress_along(
        self,
        start_x: float,
        start_y: float,
        target_yaw: float,
    ) -> float:
        return max(
            0.0,
            self._pole_pre_align_signed_progress_along(start_x, start_y, target_yaw),
        )

    def _pole_pre_align_estimated_raw_tag_x(self) -> float:
        if self.pole_pre_align_lateral_estimate_frozen:
            return self.pole_pre_align_locked_raw_tag_x_m
        if (
            self.pole_pre_align_shift_plan_start_x is None
            or self.pole_pre_align_shift_plan_start_y is None
            or (
                self.pole_pre_align_shift_motion_yaw is None
                and self.pole_pre_align_side_yaw is None
            )
            or abs(self.pole_pre_align_shift_direction_sign) <= 1e-6
        ):
            return self.pole_pre_align_locked_raw_tag_x_m
        along_m = self._pole_pre_align_signed_progress_along(
            self.pole_pre_align_shift_plan_start_x,
            self.pole_pre_align_shift_plan_start_y,
            self.pole_pre_align_shift_motion_yaw
            if self.pole_pre_align_shift_motion_yaw is not None
            else self.pole_pre_align_side_yaw,
        )
        physical_shift_sign = (
            self.pole_pre_align_shift_direction_sign
            * self.pole_pre_align_lateral_control_sign
        )
        return (
            self.pole_pre_align_locked_raw_tag_x_m
            - physical_shift_sign * along_m
        )

    def _pole_pre_align_estimated_tag_x(self) -> float:
        return self._pole_pre_align_center_x_from_raw(
            self._pole_pre_align_estimated_raw_tag_x()
        )

    def _pole_pre_align_estimated_tag_z(self) -> float:
        if (
            self.pole_pre_align_entry_start_x is None
            or self.pole_pre_align_entry_start_y is None
            or self.pole_pre_align_base_yaw is None
            or self.pole_pre_align_entry_start_tag_z_m <= 0.0
        ):
            return self.pole_pre_align_locked_tag_z_m
        along_m = self._pole_pre_align_progress_along(
            self.pole_pre_align_entry_start_x,
            self.pole_pre_align_entry_start_y,
            self.pole_pre_align_base_yaw,
        )
        return max(1e-3, self.pole_pre_align_entry_start_tag_z_m - along_m)

    def _publish_pole_pre_align_progress(
        self,
        now: float,
        detail: str,
        *,
        force: bool = False,
    ) -> None:
        if (
            not force
            and self.progress_log_interval_sec > 0.0
            and (now - self.pole_pre_align_last_log_sec) < self.progress_log_interval_sec
        ):
            return
        self.pole_pre_align_last_log_sec = now
        self._publish_feedback(
            f'绕杆前预对齐: phase={self.pole_pre_align_phase}, {detail}, '
            f'{self._pole_pre_align_status_text()}'
        )

    def _pole_pre_align_status_text(self) -> str:
        tag = self._fresh_pole_pre_align_tag()
        if tag is not None and not self.pole_pre_align_lateral_estimate_frozen:
            raw_tag_x = tag.x_m
            tag_x = self._pole_pre_align_center_x_from_raw(tag.x_m)
            tag_z = tag.z_m
            raw_pitch_err = self._pole_pre_align_pitch_err(tag)
            pitch_err = (
                self.pole_pre_align_last_filtered_pitch_err_rad
                if math.isfinite(self.pole_pre_align_last_filtered_pitch_err_rad)
                else raw_pitch_err
            )
            tag_source = 'live'
        elif tag is not None and self.pole_pre_align_lateral_estimate_frozen:
            raw_tag_x = self._pole_pre_align_estimated_raw_tag_x()
            tag_x = self._pole_pre_align_estimated_tag_x()
            tag_z = tag.z_m
            pitch_err = self.pole_pre_align_locked_pitch_err_rad
            raw_pitch_err = pitch_err
            tag_source = 'live_lateral_frozen'
        elif self.pole_pre_align_locked_tag is not None:
            raw_tag_x = self._pole_pre_align_estimated_raw_tag_x()
            tag_x = self._pole_pre_align_estimated_tag_x()
            tag_z = self._pole_pre_align_estimated_tag_z()
            pitch_err = self.pole_pre_align_locked_pitch_err_rad
            raw_pitch_err = pitch_err
            tag_source = 'locked'
        else:
            raw_tag_x = math.nan
            tag_x = math.nan
            tag_z = math.nan
            pitch_err = math.nan
            raw_pitch_err = math.nan
            tag_source = 'none'

        yaw_err = (
            math.nan
            if self.pole_pre_align_base_yaw is None
            else self._normalize_angle(self.pole_pre_align_base_yaw - self.pose_yaw)
        )
        along_m = 0.0
        remaining_m = self.pole_pre_align_shift_m
        if (
            self.pole_pre_align_move_start_x is not None
            and self.pole_pre_align_move_start_y is not None
            and self.pole_pre_align_shift_motion_yaw is not None
        ):
            along_m = self._pole_pre_align_signed_progress_along(
                self.pole_pre_align_move_start_x,
                self.pole_pre_align_move_start_y,
                self.pole_pre_align_shift_motion_yaw,
            )
            remaining_m = max(self.pole_pre_align_shift_m - along_m, 0.0)
        base_yaw_deg = (
            math.nan
            if self.pole_pre_align_base_yaw is None
            else math.degrees(self.pole_pre_align_base_yaw)
        )
        side_yaw_deg = (
            math.nan
            if self.pole_pre_align_side_yaw is None
            else math.degrees(self.pole_pre_align_side_yaw)
        )
        allow_route = (
            (not self.pole_pre_align_active)
            and (
                not self.pole_pre_align_enabled
                or self.pole_pre_align_done
                or self.trigger_source != 'apriltag'
            )
        )
        return (
            f'pre_align_active={self.pole_pre_align_active}, '
            f'pre_align_phase={self.pole_pre_align_phase or "none"}, '
            f'angle_metric={self.pole_pre_align_angle_metric}, '
            f'tag_source={tag_source}, raw_tag_x={raw_tag_x:+.3f}m, '
            f'center_x={tag_x:+.3f}m, tag_z={tag_z:.3f}m, '
            f'raw_angle_err_deg={math.degrees(raw_pitch_err):+.2f}, '
            f'angle_err_deg={math.degrees(pitch_err):+.2f}, '
            f'angle_filter={len(self.pole_pre_align_pitch_samples)}/'
            f'{self.pole_pre_align_pose_filter_window_size}, '
            f'yaw_err_deg={math.degrees(yaw_err):+.2f}, '
            f'base_yaw={base_yaw_deg:+.1f}deg, side_yaw={side_yaw_deg:+.1f}deg, '
            f'shift_m={self.pole_pre_align_shift_m:.3f}, along_m={along_m:.3f}, '
            f'remaining_m={remaining_m:.3f}, '
            f'backup_retry={self.pole_pre_align_backup_retries}/'
            f'{self.pole_pre_align_backup_max_retries}, '
            f'allow_original_route={allow_route}'
        )

    def _finish_route(self) -> None:
        if self.route_finished:
            return
        self.active_goal = False
        self.route_start_pending = False
        self.pending_route_reason = ''
        self.pending_route_source = ''
        self.active_goal_allow_pass_through = False
        self.active_goal_has_direct_override = False
        self.active_goal_direct_override = False
        self.active_goal_has_forward_override = False
        self.active_goal_has_forward_tracking = False
        self.active_goal_is_turn_primitive = False
        self.active_turn_phase = ''
        self.active_turn_settle_reason = ''
        self.active_turn_settle_dist = 0.0
        self.active_turn_direction_sign = 0
        self.active_turn_last_yaw = 0.0
        self.active_turn_progress = 0.0
        self.active_turn_target_delta = 0.0
        self.active_turn_uses_body_relative_yaw = False
        self.active_turn_uses_locked_yaw = False
        self.active_turn_body_relative_delta_rad = 0.0
        self.active_turn_pole_distance_gate_m = 0.0
        self.awaiting_next_goal = False
        self.route_finished = True
        self._release_startup_route_handoff('route_finished', log_release=False)
        self._clear_step_override()
        self._set_route_mode_active(False)
        self._publish_mode(self.finish_mode)
        self._publish_state('FINISHED')
        self._publish_feedback(f'{self._route_source_label()} 触发的相对目标点已全部完成。')

    def _ensure_route_anchor(self, *, force: bool = False) -> bool:
        if not self._uses_route_anchor_reference() or self.route_anchor_ready:
            return True
        if not self._pose_ready_for_goal():
            return False
        if self.require_fresh_pole_for_route_start and not force:
            if self._current_route_start_source() == 'apriltag':
                if not self._apriltag_ready_for_anchor():
                    self._publish_state('WAITING_APRILTAG_SNAPSHOT')
                    return False
            elif not self._nearest_ready_for_anchor():
                self._publish_state('WAITING_ORANGE_POLE_SNAPSHOT')
                return False

        self.route_anchor_x = self.pose_x
        self.route_anchor_y = self.pose_y
        use_fixed_route_yaw = (
            self.pole_pre_align_done
            and self._pole_pre_align_uses_fixed_absolute_yaw()
            and self.pole_pre_align_base_yaw is not None
        )
        self.route_anchor_yaw = (
            self.pole_pre_align_base_yaw if use_fixed_route_yaw else self.pose_yaw
        )
        self.route_anchor_frame = self.pose_frame or self.default_frame_id
        self.route_anchor_ready = True
        source_text = 'AprilTag' if self._current_route_start_source() == 'apriltag' else 'HSV'
        reference_text = 'route frame' if self.route_reference_mode == 'locked_route_frame' else 'anchor'
        self._publish_feedback(
            f'锁定 {source_text} 绕杆 {reference_text}: pose=({self.route_anchor_x:.3f},'
            f'{self.route_anchor_y:.3f}), yaw={math.degrees(self.route_anchor_yaw):.1f}deg, '
            f'yaw_source={"fixed_pre_align" if use_fixed_route_yaw else "pose"}, '
            f'frame={self.route_anchor_frame}, reference_mode={self.route_reference_mode}'
        )
        return True

    def _target_offset_for_current_waypoint(self) -> Tuple[float, float]:
        if self._uses_route_anchor_reference():
            return self.waypoint_cumulative_offsets[self.current_idx]
        return self.waypoint_offsets[self.current_idx]

    def _pose_ready_for_goal(self) -> bool:
        if not self.have_pose:
            return False
        return not self._pose_is_stale()

    def _route_start_prerequisites_ready(self) -> bool:
        if not self._pose_ready_for_goal():
            return False
        if not self._uses_route_anchor_reference() or self.route_anchor_ready:
            return True
        if not self.require_fresh_pole_for_route_start:
            return True
        if self._current_route_start_source() == 'apriltag':
            return self._apriltag_ready_for_anchor()
        return self._nearest_ready_for_anchor()

    def _publish_route_start_wait_state(self) -> None:
        if not self.have_pose or self._pose_is_stale():
            self._publish_state('WAITING_POSE')
            self._publish_feedback(
                f'{self._route_source_label()} 已触发，等待可用 Odometry 后再锁定 anchor。'
            )
            return
        if self.require_fresh_pole_for_route_start:
            if self._current_route_start_source() == 'apriltag':
                if not self._apriltag_ready_for_anchor():
                    self._publish_state('WAITING_APRILTAG_SNAPSHOT')
                    self._publish_feedback(
                        'AprilTag 已触发，等待新鲜 tag TF 后再锁定 anchor。'
                    )
                return
            if not self._nearest_ready_for_anchor():
                self._publish_state('WAITING_ORANGE_POLE_SNAPSHOT')
                self._publish_feedback('HSV 已触发，等待新鲜最近杆数据后再锁定 anchor。')
            return

    def _pose_is_stale(self) -> bool:
        if self.last_pose_time is None or self.pose_stale_timeout_sec <= 0.0:
            return False
        age = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        return age > self.pose_stale_timeout_sec

    def _nearest_is_stale(self) -> bool:
        if self.last_pole_msg_time is None or self.nearest_stale_timeout_sec <= 0.0:
            return False
        return (self._now_sec() - self.last_pole_msg_time) > self.nearest_stale_timeout_sec

    def _nearest_ready_for_anchor(self) -> bool:
        if self.last_pole is None:
            return False
        return not self._nearest_is_stale()

    def _hsv_trigger_enabled(self) -> bool:
        return self.trigger_source in ('hsv', 'both')

    def _apriltag_trigger_enabled(self) -> bool:
        return self.trigger_source in ('apriltag', 'both')

    def _apriltag_runtime_enabled(self) -> bool:
        return self._apriltag_trigger_enabled()

    def _current_route_start_source(self) -> str:
        if self.route_start_pending and self.pending_route_source:
            return self.pending_route_source
        if self.route_source:
            return self.route_source
        if self.trigger_source == 'apriltag':
            return 'apriltag'
        return 'hsv'

    def _route_source_label(self) -> str:
        return 'AprilTag' if self._current_route_start_source() == 'apriltag' else 'HSV'

    def _apriltag_pose_is_stale(self) -> bool:
        if self.latest_apriltag is None:
            return True
        if self.apriltag_stale_timeout_sec <= 0.0:
            return False
        stamp_sec = (
            self.latest_apriltag.stamp_sec
            if self.latest_apriltag.stamp_sec > 0.0
            else self.latest_apriltag.received_sec
        )
        return (self._now_sec() - stamp_sec) > self.apriltag_stale_timeout_sec

    def _apriltag_detection_is_stale(self) -> bool:
        if self.latest_apriltag_detection.received_sec <= 0.0:
            return True
        if self.apriltag_stale_timeout_sec <= 0.0:
            return False
        stamp_sec = (
            self.latest_apriltag_detection.stamp_sec
            if self.latest_apriltag_detection.stamp_sec > 0.0
            else self.latest_apriltag_detection.received_sec
        )
        return (
            self._now_sec() - stamp_sec
        ) > self.apriltag_stale_timeout_sec

    def _apriltag_ready_for_anchor(self) -> bool:
        if self.latest_apriltag is None or self._apriltag_pose_is_stale():
            return False
        if not self.latest_apriltag_detection.matched:
            return False
        if self._apriltag_detection_is_stale():
            return False
        return True

    def _apriltag_age_sec(self, tag: AprilTagSnapshot) -> float:
        stamp_sec = tag.stamp_sec if tag.stamp_sec > 0.0 else tag.received_sec
        if stamp_sec <= 0.0:
            return 0.0
        return max(0.0, self._now_sec() - stamp_sec)

    def _segment_start_for_current_waypoint(self) -> Tuple[float, float]:
        if self._uses_route_anchor_reference():
            if self.current_idx == 0:
                return self.route_anchor_x, self.route_anchor_y
            prev_dx, prev_dy = self.waypoint_cumulative_offsets[self.current_idx - 1]
            return self.route_anchor_x + prev_dx, self.route_anchor_y + prev_dy
        return self.pose_x, self.pose_y

    def _uses_route_anchor_reference(self) -> bool:
        return self.route_reference_mode in ('route_anchor', 'locked_route_frame')

    def _body_relative_waypoint_offset(self, idx: int) -> Optional[Tuple[float, float]]:
        return self.body_relative_waypoint_offset_map.get(idx)

    def _turn_waypoint_pole_distance_gate(self, idx: int) -> Optional[float]:
        return self.turn_waypoint_pole_distance_gate_map.get(idx)

    def _turn_waypoint_body_relative_delta_rad(self, idx: int) -> Optional[float]:
        if self.turn_waypoint_yaw_mode != 'body_relative_delta':
            return None
        delta_deg = self.body_relative_turn_delta_map.get(idx)
        if delta_deg is None:
            return None
        return math.radians(delta_deg)

    def _forward_mode_for_waypoint(self, idx: int) -> int:
        if idx == 0:
            return self.first_forward_route_mode
        return self.forward_route_mode

    def _forward_override_values(self, idx: int) -> Tuple[float, float]:
        return (
            self.waypoint_forward_step_left_norms[idx],
            self.waypoint_forward_step_right_norms[idx],
        )

    def _active_route_mode(self) -> int:
        if self.active_goal_is_turn_primitive:
            return self.turn_route_mode
        if self.active_goal_has_direct_override and self.active_goal_direct_override:
            return self.turn_route_mode
        if self.active_goal_has_forward_tracking:
            return self._forward_mode_for_waypoint(self.current_idx)
        return self._forward_mode_for_waypoint(self.current_idx)

    def _forward_tracking_enabled_for_current_goal(
        self,
        body_relative_offset: Optional[Tuple[float, float]],
        use_direct_override: bool,
        segment_dx: float,
        segment_dy: float,
    ) -> bool:
        if not self.forward_tracking_enabled:
            return False
        if self.route_reference_mode != 'locked_route_frame':
            return False
        if body_relative_offset is None:
            return False
        if use_direct_override:
            return False
        return math.hypot(segment_dx, segment_dy) > 1e-6

    def _monitor_forward_tracking(self, dist: float) -> bool:
        if self._pose_is_stale():
            self._handle_goal_failed('forward_tracking_pose_stale')
            return True

        (
            left_norm,
            right_norm,
            delta_norm,
            along,
            lateral_error,
            yaw_error,
            remaining,
        ) = self._forward_tracking_command()
        abs_lateral = abs(lateral_error)
        abs_yaw_error = abs(yaw_error)
        if (
            self.forward_tracking_lateral_fail_m > 0.0
            and abs_lateral > self.forward_tracking_lateral_fail_m
        ):
            self._handle_goal_failed(
                f'forward_tracking_lateral_fail lateral={lateral_error:+.3f}m>'
                f'{self.forward_tracking_lateral_fail_m:.3f}m '
                f'along={along:.3f}m remaining={remaining:.3f}m'
            )
            return True
        if (
            self.forward_tracking_yaw_fail_rad > 0.0
            and abs_yaw_error > self.forward_tracking_yaw_fail_rad
        ):
            self._handle_goal_failed(
                f'forward_tracking_yaw_fail yaw_err={math.degrees(yaw_error):+.1f}deg>'
                f'{math.degrees(self.forward_tracking_yaw_fail_rad):.1f}deg '
                f'along={along:.3f}m lateral={lateral_error:+.3f}m'
            )
            return True

        segment_length = self._active_segment_length()
        tracking_timeout = (
            self.forward_tracking_timeout_extra_sec
            + self.forward_tracking_timeout_sec_per_m * segment_length
        )
        if tracking_timeout > 0.0 and (self._now_sec() - self.active_goal_stamp) > tracking_timeout:
            self._handle_goal_failed(
                f'forward_tracking_timeout timeout={tracking_timeout:.2f}s '
                f'along={along:.3f}/{segment_length:.3f}m remaining={remaining:.3f}m '
                f'lateral={lateral_error:+.3f}m yaw_err={math.degrees(yaw_error):+.1f}deg'
            )
            return True
        lateral_tolerance = (
            self._forward_tracking_arrival_lateral_tolerance_for_current_waypoint()
        )
        overrun = max(0.0, along - segment_length)
        if (
            lateral_tolerance > 0.0
            and abs_lateral > lateral_tolerance
            and overrun > self.active_goal_arrival_tolerance
        ):
            self._handle_goal_failed(
                f'forward_tracking_overrun_lateral_fail '
                f'along={along:.3f}/{segment_length:.3f}m '
                f'overrun={overrun:.3f}m>{self.active_goal_arrival_tolerance:.2f}m '
                f'lateral={lateral_error:+.3f}m>{lateral_tolerance:.3f}m '
                f'remaining={remaining:.3f}m yaw_err={math.degrees(yaw_error):+.1f}deg'
            )
            return True

        self._publish_step_override(
            self._forward_mode_for_waypoint(self.current_idx),
            left_norm,
            right_norm,
        )
        self._publish_progress(
            f'forward_tracking waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)} '
            f'route_origin=({self.route_anchor_x:.3f},{self.route_anchor_y:.3f}) '
            f'route_yaw={math.degrees(self.route_anchor_yaw):.1f}deg '
            f'start=({self.active_segment_start_x:.3f},{self.active_segment_start_y:.3f}) '
            f'goal=({self.active_goal_x:.3f},{self.active_goal_y:.3f}) '
            f'pose=({self.pose_x:.3f},{self.pose_y:.3f}) dist={dist:.3f}m '
            f'along={along:.3f}m remaining={remaining:.3f}m '
            f'lateral={lateral_error:+.3f}m yaw_err={math.degrees(yaw_error):+.1f}deg '
            f'gain_lat={self._forward_tracking_lateral_gain_for_current_waypoint():.2f} '
            f'arrival_lat_tol='
            f'{self._forward_tracking_arrival_lateral_tolerance_for_current_waypoint():.2f}m '
            f'corr={delta_norm:+.3f} '
            f'override=[{self._forward_mode_for_waypoint(self.current_idx)},'
            f'{left_norm:+.3f},{right_norm:+.3f}]'
        )

        arrival_reason = self._forward_tracking_arrival_reason(along, remaining, abs_lateral)
        if arrival_reason is not None:
            self._handle_goal_arrived(dist, arrival_reason)
            return True
        return True

    def _forward_tracking_command(
        self,
    ) -> Tuple[float, float, float, float, float, float, float]:
        along, lateral_error, remaining = self._segment_progress(signed_lateral=True)
        yaw_error = self._normalize_angle(self.active_goal_yaw - self.pose_yaw)
        lateral_gain = self._forward_tracking_lateral_gain_for_current_waypoint()
        lateral_for_control = 0.0
        if abs(lateral_error) > self.forward_tracking_deadband_m:
            lateral_for_control = (
                math.copysign(
                    abs(lateral_error) - self.forward_tracking_deadband_m,
                    lateral_error,
                )
            )
        delta_norm = (
            lateral_gain * lateral_for_control
            + self.forward_tracking_yaw_gain * yaw_error
        )
        if self.forward_tracking_max_delta_norm > 0.0:
            delta_norm = max(
                -self.forward_tracking_max_delta_norm,
                min(self.forward_tracking_max_delta_norm, delta_norm),
            )
        else:
            delta_norm = 0.0

        left_norm = self._clamp_norm(self.forward_tracking_base_left_norm - delta_norm)
        right_norm = self._clamp_norm(self.forward_tracking_base_right_norm + delta_norm)
        if self.forward_tracking_max_delta_norm > 0.0:
            left_norm, right_norm = self._clamp_lr_delta(
                left_norm,
                right_norm,
                2.0 * self.forward_tracking_max_delta_norm,
            )
        return left_norm, right_norm, delta_norm, along, lateral_error, yaw_error, remaining

    def _forward_tracking_lateral_gain_for_current_waypoint(self) -> float:
        return self.forward_tracking_lateral_gain_map.get(
            self.current_idx,
            self.forward_tracking_lateral_gain,
        )

    def _forward_tracking_arrival_lateral_tolerance_for_current_waypoint(self) -> float:
        return self.forward_tracking_arrival_lateral_tolerance_map.get(
            self.current_idx,
            0.0,
        )

    def _forward_tracking_arrival_reason(
        self,
        along: float,
        remaining: float,
        abs_lateral: float,
    ) -> Optional[str]:
        segment_length = self._active_segment_length()
        if segment_length <= 1e-6:
            return 'forward_tracking_zero_segment'
        lateral_tolerance = (
            self._forward_tracking_arrival_lateral_tolerance_for_current_waypoint()
        )
        if lateral_tolerance > 0.0 and abs_lateral > lateral_tolerance:
            return None
        if remaining <= self.active_goal_arrival_tolerance:
            return (
                f'forward_tracking_remaining={remaining:.3f}m<'
                f'{self.active_goal_arrival_tolerance:.2f}m '
                f'along={along:.3f}/{segment_length:.3f}m lateral={abs_lateral:.3f}m'
                f' lat_tol={lateral_tolerance:.3f}m'
            )
        if along >= segment_length:
            return (
                f'forward_tracking_passed_goal along={along:.3f}/{segment_length:.3f}m '
                f'lateral={abs_lateral:.3f}m lat_tol={lateral_tolerance:.3f}m'
            )
        return None

    def _body_relative_offset_to_world(
        self,
        forward_m: float,
        left_m: float,
    ) -> Tuple[float, float]:
        return self._body_relative_offset_to_world_from_yaw(
            self.pose_yaw,
            forward_m,
            left_m,
        )

    @staticmethod
    def _body_relative_offset_to_world_from_yaw(
        yaw: float,
        forward_m: float,
        left_m: float,
    ) -> Tuple[float, float]:
        forward_heading = yaw + math.pi / 2.0
        left_heading = forward_heading + math.pi / 2.0
        dx = (
            forward_m * math.cos(forward_heading)
            + left_m * math.cos(left_heading)
        )
        dy = (
            forward_m * math.sin(forward_heading)
            + left_m * math.sin(left_heading)
        )
        return dx, dy

    def _locked_route_segment(
        self,
        idx: int,
    ) -> Tuple[float, float, float, float, float, float, float]:
        x = self.route_anchor_x
        y = self.route_anchor_y
        yaw = self.route_anchor_yaw
        start_x = x
        start_y = y
        goal_x = x
        goal_y = y
        target_yaw = yaw
        for route_idx in range(0, idx + 1):
            start_x = x
            start_y = y
            start_yaw = yaw
            offset = self._body_relative_waypoint_offset(route_idx)
            if offset is not None:
                forward_m, left_m = offset
                dx, dy = self._body_relative_offset_to_world_from_yaw(
                    yaw,
                    forward_m,
                    left_m,
                )
                x += dx
                y += dy
            elif not self.waypoint_use_direct_step_override[route_idx]:
                dx, dy = self.waypoint_offsets[route_idx]
                x += dx
                y += dy

            goal_x = x
            goal_y = y
            target_yaw = start_yaw
            turn_delta_rad = self._turn_waypoint_body_relative_delta_rad(route_idx)
            if self.waypoint_use_direct_step_override[route_idx] and turn_delta_rad is not None:
                target_yaw = self._normalize_angle(start_yaw + turn_delta_rad)
                yaw = target_yaw

        segment_dx = goal_x - start_x
        segment_dy = goal_y - start_y
        return start_x, start_y, goal_x, goal_y, segment_dx, segment_dy, target_yaw

    def _goal_publish_yaw(
        self,
        dx: float,
        dy: float,
        arrival_yaw: float,
        use_direct_override: bool,
    ) -> float:
        # 2026-05-04 10:42 CST: short turn waypoints should approach along the
        # measured segment direction, then let direct override finish the body
        # rotation. Publishing the final turn yaw too early makes the close
        # approach look like a pose-alignment task instead of a short arc turn.
        # Rollback: return arrival_yaw for every waypoint.
        if use_direct_override and math.hypot(dx, dy) > 1e-6:
            return math.atan2(dy, dx)
        return arrival_yaw

    def _current_waypoint_uses_turn_primitive(self, idx: int) -> bool:
        if idx < 0 or idx >= len(self.waypoint_offsets):
            return False
        return (
            self.turn_waypoint_use_motion_primitive
            and self.waypoint_use_direct_step_override[idx]
        )

    def _start_turn_primitive(
        self,
        *,
        goal_x: float,
        goal_y: float,
        frame_id: str,
        arrival_yaw: float,
        arrival_tolerance: float,
        use_yaw_tolerance: bool,
        planned_segment_start_x: float,
        planned_segment_start_y: float,
        segment_dx: float,
        segment_dy: float,
        cumulative_dx: float,
        cumulative_dy: float,
    ) -> None:
        body_relative_turn_delta_rad = self._turn_waypoint_body_relative_delta_rad(self.current_idx)
        if body_relative_turn_delta_rad is not None:
            if abs(body_relative_turn_delta_rad) <= 1e-6:
                self._handle_goal_failed(
                    f'turn_primitive_zero_body_relative_delta waypoint={self.current_idx + 1}'
                )
                return
            left_norm, right_norm, direction_sign = self._resolved_turn_override(
                self.current_idx,
                1 if body_relative_turn_delta_rad > 0.0 else -1,
            )
            if direction_sign == 0:
                self._handle_goal_failed(
                    f'turn_primitive_invalid_body_relative_override waypoint={self.current_idx + 1} '
                    f'delta_deg={math.degrees(body_relative_turn_delta_rad):.1f}'
                )
                return
        else:
            left_norm, right_norm, direction_sign = self._resolved_turn_override(
                self.current_idx
            )
            if direction_sign == 0:
                self._handle_goal_failed(
                    f'turn_primitive_invalid_override waypoint={self.current_idx + 1}'
                )
                return

        self.active_goal_x = goal_x
        self.active_goal_y = goal_y
        self.active_goal_yaw = arrival_yaw
        self.active_goal_nav_yaw = arrival_yaw
        self.active_goal_frame_id = frame_id
        self.active_goal_arrival_tolerance = arrival_tolerance
        self.active_goal_use_yaw_tolerance = use_yaw_tolerance
        self.active_goal_allow_pass_through = False
        self.active_goal_has_direct_override = True
        self.active_goal_direct_override = True
        self.active_goal_is_turn_primitive = True
        self.active_turn_phase = (
            'IN_PLACE_TURN'
            if self.turn_waypoint_simple_in_place_enabled
            else 'ARC_TURN'
        )
        self.active_goal_stamp = self._now_sec()
        self.active_goal = True
        self.awaiting_next_goal = False
        self.active_segment_start_x = self.pose_x
        self.active_segment_start_y = self.pose_y
        self.active_turn_direction_sign = direction_sign
        self.active_turn_start_yaw = self.pose_yaw
        self.active_turn_last_yaw = self.pose_yaw
        self.active_turn_progress = 0.0
        self.active_turn_target_delta = self._directional_angle_delta(
            self.active_turn_start_yaw,
            arrival_yaw,
            direction_sign,
        )
        self.active_turn_segment_length = math.hypot(
            goal_x - self.active_segment_start_x,
            goal_y - self.active_segment_start_y,
        )
        self.active_turn_start_time = self.active_goal_stamp
        self.active_turn_phase_start_time = self.active_goal_stamp
        self.active_turn_uses_body_relative_yaw = body_relative_turn_delta_rad is not None
        self.active_turn_uses_locked_yaw = self.route_reference_mode == 'locked_route_frame'
        self.active_turn_body_relative_delta_rad = body_relative_turn_delta_rad or 0.0
        self.active_turn_pole_distance_gate_m = (
            self._turn_waypoint_pole_distance_gate(self.current_idx) or 0.0
        )

        self._clear_step_override()
        direction_text = 'ccw' if direction_sign > 0 else 'cw'
        if self.active_turn_pole_distance_gate_m > 0.0:
            self.active_turn_phase = 'WAIT_POLE_DISTANCE'
            self._publish_state('TURN_PRIMITIVE_WAIT_POLE_DISTANCE')
            self._publish_feedback(
                f'发送 HSV 绕杆转向等待点[{self.current_idx + 1}/{len(self.waypoint_offsets)}]: '
                f'segment_offset=({segment_dx:.2f},{segment_dy:.2f}), '
                f'cumulative_offset=({cumulative_dx:.2f},{cumulative_dy:.2f}), '
                f'planned_start=({planned_segment_start_x:.3f},{planned_segment_start_y:.3f}), '
                f'actual_start=({self.active_segment_start_x:.3f},{self.active_segment_start_y:.3f}), '
                f'target=({goal_x:.3f},{goal_y:.3f}), target_yaw={math.degrees(arrival_yaw):.1f}deg, '
                f'direction={direction_text}, yaw_mode={self.turn_waypoint_yaw_mode}, '
                f'wait_pole_distance<={self.active_turn_pole_distance_gate_m:.2f}m'
            )
            return

        self._begin_turn_motion_phase()

    def _begin_turn_motion_phase(self) -> None:
        self.active_turn_start_yaw = self.pose_yaw
        self.active_turn_last_yaw = self.pose_yaw
        self.active_turn_progress = 0.0
        if self.active_turn_uses_body_relative_yaw and not self.active_turn_uses_locked_yaw:
            self.active_goal_yaw = self._normalize_angle(
                self.active_turn_start_yaw + self.active_turn_body_relative_delta_rad
            )
            self.active_turn_target_delta = abs(self.active_turn_body_relative_delta_rad)
        else:
            self.active_turn_target_delta = self._directional_angle_delta(
                self.active_turn_start_yaw,
                self.active_goal_yaw,
                self.active_turn_direction_sign,
            )
        self.active_turn_phase_start_time = self._now_sec()
        direction_text = 'ccw' if self.active_turn_direction_sign > 0 else 'cw'
        yaw_mode_text = (
            f'locked_route_frame delta={math.degrees(self.active_turn_body_relative_delta_rad):.1f}deg'
            if self.active_turn_uses_locked_yaw and self.active_turn_uses_body_relative_yaw
            else f'body_relative_delta={math.degrees(self.active_turn_body_relative_delta_rad):.1f}deg'
            if self.active_turn_uses_body_relative_yaw
            else 'global_waypoint'
        )
        if self.turn_waypoint_simple_in_place_enabled:
            left_norm, right_norm, _ = self._resolved_turn_override(
                self.current_idx,
                self.active_turn_direction_sign,
            )
            self.active_turn_phase = 'IN_PLACE_TURN'
            self._set_route_mode_active(True)
            self._publish_step_override(self.turn_route_mode, left_norm, right_norm)
            self._publish_state('TURN_PRIMITIVE_IN_PLACE')
            self._publish_feedback(
                f'开始 HSV 绕杆原地转向[{self.current_idx + 1}/{len(self.waypoint_offsets)}]: '
                f'target=({self.active_goal_x:.3f},{self.active_goal_y:.3f}), '
                f'target_yaw={math.degrees(self.active_goal_yaw):.1f}deg, '
                f'direction={direction_text}, yaw_mode={yaw_mode_text}, '
                f'target_turn={math.degrees(self.active_turn_target_delta):.1f}deg, '
                f'completion_yaw_tol={math.degrees(self.turn_waypoint_completion_yaw_tolerance_rad):.1f}deg, '
                f'completion_progress_tol={math.degrees(self.turn_waypoint_completion_progress_tolerance_rad):.1f}deg, '
                f'in_place_override=[{self.turn_route_mode},{left_norm:.3f},{right_norm:.3f}]'
            )
            return

        arc_left_norm, arc_right_norm = self._turn_arc_override(self.active_turn_direction_sign)
        self.active_turn_phase = 'ARC_TURN'
        self._set_route_mode_active(True)
        self._publish_step_override(self.turn_route_mode, arc_left_norm, arc_right_norm)
        self._publish_state('TURN_PRIMITIVE_ARC')
        self._publish_feedback(
            f'开始 HSV 绕杆转向原语[{self.current_idx + 1}/{len(self.waypoint_offsets)}]: '
            f'target=({self.active_goal_x:.3f},{self.active_goal_y:.3f}), '
            f'target_yaw={math.degrees(self.active_goal_yaw):.1f}deg, '
            f'direction={direction_text}, yaw_mode={yaw_mode_text}, '
            f'target_turn={math.degrees(self.active_turn_target_delta):.1f}deg, '
            f'completion_yaw_tol={math.degrees(self.turn_waypoint_completion_yaw_tolerance_rad):.1f}deg, '
            f'completion_progress_tol={math.degrees(self.turn_waypoint_completion_progress_tolerance_rad):.1f}deg, '
            f'arc_override=[{self.turn_route_mode},{arc_left_norm:.3f},{arc_right_norm:.3f}]'
        )

    def _start_turn_settle(self, dist: float, reason: str) -> None:
        if self.turn_settle_sec <= 1e-6:
            self._handle_goal_arrived(dist, reason)
            return
        self.active_turn_phase = 'SETTLE'
        self.active_turn_phase_start_time = self._now_sec()
        self.active_turn_settle_reason = reason
        self.active_turn_settle_dist = dist
        self._clear_step_override()
        self._publish_state('TURN_PRIMITIVE_SETTLE')
        self._publish_feedback(
            f'转向完成，进入停稳确认: waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)}, '
            f'settle={self.turn_settle_sec:.2f}s, '
            f'yaw_tol={math.degrees(self.turn_settle_yaw_tolerance_rad):.1f}deg, '
            f'reason={reason}'
        )

    def _monitor_turn_primitive(self, dist: float, yaw_err: float) -> None:
        now = self._now_sec()
        if self.active_turn_phase == 'SETTLE':
            self._clear_step_override()
            elapsed = now - self.active_turn_phase_start_time
            yaw_done = (
                yaw_err <= self.turn_settle_yaw_tolerance_rad
                if self.active_goal_use_yaw_tolerance
                else True
            )
            self._publish_progress(
                f'turn_settle waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)} '
                f'elapsed={elapsed:.2f}/{self.turn_settle_sec:.2f}s '
                f'yaw_err={math.degrees(yaw_err):.1f}deg '
                f'yaw_tol={math.degrees(self.turn_settle_yaw_tolerance_rad):.1f}deg '
                f'yaw_done={yaw_done} '
                f'pose=({self.pose_x:.3f},{self.pose_y:.3f})'
            )
            if elapsed >= self.turn_settle_sec:
                if not yaw_done:
                    unstable_text = (
                        f' yaw_unstable yaw_err={math.degrees(yaw_err):.1f}deg>'
                        f'{math.degrees(self.turn_settle_yaw_tolerance_rad):.1f}deg'
                    )
                    if self.turn_settle_yaw_unstable_fail_enabled:
                        self._handle_goal_failed('turn_settle_yaw_unstable' + unstable_text)
                        return
                    self._publish_feedback(
                        f'转向停稳 yaw 复检超限但不中断: waypoint={self.current_idx + 1}/'
                        f'{len(self.waypoint_offsets)}, {unstable_text.strip()}, '
                        '已按进入 settle 前的完成判定继续下一点'
                    )
                else:
                    unstable_text = ''
                self._handle_goal_arrived(
                    self.active_turn_settle_dist,
                    f'{self.active_turn_settle_reason} settle={elapsed:.2f}s{unstable_text}',
                )
            return

        if self.active_turn_phase == 'WAIT_POLE_DISTANCE':
            pole_distance_m = self._fresh_pole_distance()
            pole_text = (
                f'{pole_distance_m:.3f}m'
                if pole_distance_m is not None
                else 'none_or_stale'
            )
            self._publish_progress(
                f'turn_wait_pole waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)} '
                f'pole_dist={pole_text} gate<={self.active_turn_pole_distance_gate_m:.2f}m '
                f'pose=({self.pose_x:.3f},{self.pose_y:.3f})'
            )
            if (
                pole_distance_m is not None
                and self.active_turn_pole_distance_gate_m > 0.0
                and pole_distance_m <= self.active_turn_pole_distance_gate_m
            ):
                self._publish_feedback(
                    f'最近杆距离满足转向门控: waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)}, '
                    f'pole_dist={pole_distance_m:.3f}m<=gate={self.active_turn_pole_distance_gate_m:.2f}m'
                )
                self._begin_turn_motion_phase()
                return
            turn_timeout = self.turn_waypoint_timeout_sec or self.goal_timeout_sec
            if turn_timeout > 0.0 and (now - self.active_turn_start_time) > turn_timeout:
                self._handle_goal_failed(
                    f'turn_wait_pole_timeout gate={self.active_turn_pole_distance_gate_m:.2f}m '
                    f'pole_dist={pole_text}'
                )
            return

        yaw_delta = self._normalize_angle(self.pose_yaw - self.active_turn_last_yaw)
        directed_increment = self.active_turn_direction_sign * yaw_delta
        if directed_increment > 0.0:
            self.active_turn_progress += directed_increment
        self.active_turn_last_yaw = self.pose_yaw

        turn_progress = self.active_turn_progress
        along, lateral, remaining = self._segment_progress()
        turn_remaining = max(self.active_turn_target_delta - turn_progress, 0.0)
        turn_overshoot = max(turn_progress - self.active_turn_target_delta, 0.0)
        progress_done = (
            turn_progress + self.turn_waypoint_completion_progress_tolerance_rad
            >= self.active_turn_target_delta
        )
        yaw_done = (
            yaw_err <= self.turn_waypoint_completion_yaw_tolerance_rad
            if self.active_goal_use_yaw_tolerance
            else True
        )
        turn_done = progress_done and yaw_done
        if self.active_turn_phase == 'IN_PLACE_TURN':
            left_norm, right_norm, _ = self._resolved_turn_override(
                self.current_idx,
                self.active_turn_direction_sign,
            )
            slow_turn_active = (
                self.turn_waypoint_slowdown_enabled
                and self.turn_waypoint_slowdown_remaining_rad > 0.0
                and 0.0 < turn_remaining <= self.turn_waypoint_slowdown_remaining_rad
            )
            if slow_turn_active:
                left_norm, right_norm = self._turn_slowdown_override(
                    self.active_turn_direction_sign
                )
            self._publish_step_override(self.turn_route_mode, left_norm, right_norm)
            min_duration_done = (
                now - self.active_turn_phase_start_time
            ) >= self.turn_waypoint_min_duration_sec
            overshoot_guard_done = (
                self.turn_waypoint_overshoot_guard_enabled
                and progress_done
                and turn_overshoot >= self.turn_waypoint_overshoot_tolerance_rad
                and (
                    not self.active_goal_use_yaw_tolerance
                    or yaw_err <= self.turn_waypoint_overshoot_accept_yaw_tolerance_rad
                )
            )
            self._publish_progress(
                f'turn_in_place waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)} '
                f'yaw_err={math.degrees(yaw_err):.1f}deg '
                f'turn_progress={math.degrees(turn_progress):.1f}/'
                f'{math.degrees(self.active_turn_target_delta):.1f}deg '
                f'remaining={math.degrees(turn_remaining):.1f}deg '
                f'overshoot={math.degrees(turn_overshoot):.1f}deg '
                f'progress_done={progress_done} yaw_done={yaw_done} '
                f'slow_turn={slow_turn_active} overshoot_guard={overshoot_guard_done} '
                f'pose=({self.pose_x:.3f},{self.pose_y:.3f})'
            )
            if min_duration_done and turn_done:
                self._start_turn_settle(
                    0.0,
                    f'turn_in_place turn_progress={math.degrees(turn_progress):.1f}/'
                    f'{math.degrees(self.active_turn_target_delta):.1f}deg '
                    f'yaw_err={math.degrees(yaw_err):.1f}deg',
                )
                return
            if min_duration_done and overshoot_guard_done:
                self._start_turn_settle(
                    0.0,
                    f'turn_in_place overshoot_guard turn_progress={math.degrees(turn_progress):.1f}/'
                    f'{math.degrees(self.active_turn_target_delta):.1f}deg '
                    f'overshoot={math.degrees(turn_overshoot):.1f}deg '
                    f'yaw_err={math.degrees(yaw_err):.1f}deg',
                )
                return

        elif self.active_turn_phase == 'ARC_TURN':
            arc_left_norm, arc_right_norm = self._turn_arc_override(
                self.active_turn_direction_sign
            )
            self._publish_step_override(self.turn_route_mode, arc_left_norm, arc_right_norm)
            min_duration_done = (
                now - self.active_turn_phase_start_time
            ) >= self.turn_waypoint_min_duration_sec
            self._publish_progress(
                f'turn_arc waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)} '
                f'dist={dist:.3f}m yaw_err={math.degrees(yaw_err):.1f}deg '
                f'turn_progress={math.degrees(turn_progress):.1f}/'
                f'{math.degrees(self.active_turn_target_delta):.1f}deg '
                f'progress_done={progress_done} yaw_done={yaw_done} '
                f'along={along:.3f}m remaining={remaining:.3f}m lateral={lateral:.3f}m'
            )
            if min_duration_done and turn_done:
                self.active_turn_phase = 'EXIT_FORWARD'
                self.active_turn_phase_start_time = now
                self._publish_step_override(
                    self.turn_route_mode,
                    self.turn_waypoint_exit_forward_norm,
                    self.turn_waypoint_exit_forward_norm,
                )
                self._publish_state('TURN_PRIMITIVE_EXIT_FORWARD')
                self._publish_feedback(
                    f'转向原语进入前送阶段: waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)}, '
                    f'turn_progress={math.degrees(turn_progress):.1f}/'
                    f'{math.degrees(self.active_turn_target_delta):.1f}deg, '
                    f'exit_override=[{self.turn_route_mode},{self.turn_waypoint_exit_forward_norm:.3f},'
                    f'{self.turn_waypoint_exit_forward_norm:.3f}]'
                )
                return

        elif self.active_turn_phase == 'EXIT_FORWARD':
            self._publish_step_override(
                self.turn_route_mode,
                self.turn_waypoint_exit_forward_norm,
                self.turn_waypoint_exit_forward_norm,
            )
            corridor_done = (
                remaining <= self.turn_waypoint_longitudinal_tolerance_m
                and lateral <= self.turn_waypoint_lateral_tolerance_m
            )
            dist_done = dist <= self.active_goal_arrival_tolerance
            min_duration_done = (
                now - self.active_turn_phase_start_time
            ) >= self.turn_waypoint_exit_forward_min_duration_sec
            self._publish_progress(
                f'turn_exit waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)} '
                f'dist={dist:.3f}m yaw_err={math.degrees(yaw_err):.1f}deg '
                f'along={along:.3f}m remaining={remaining:.3f}m lateral={lateral:.3f}m'
            )
            if min_duration_done and (dist_done or corridor_done):
                reason = (
                    f'turn_exit corridor={corridor_done} dist={dist:.3f}m '
                    f'remaining={remaining:.3f}m lateral={lateral:.3f}m'
                )
                self._start_turn_settle(dist, reason)
                return
            if (
                self.turn_waypoint_exit_forward_timeout_sec > 0.0
                and (now - self.active_turn_phase_start_time) > self.turn_waypoint_exit_forward_timeout_sec
            ):
                self._handle_goal_failed(
                    f'turn_exit_timeout dist={dist:.3f}m remaining={remaining:.3f}m '
                    f'lateral={lateral:.3f}m'
                )
                return

        turn_timeout = self.turn_waypoint_timeout_sec or self.goal_timeout_sec
        if turn_timeout > 0.0 and (now - self.active_turn_start_time) > turn_timeout:
            self._handle_goal_failed(
                f'turn_primitive_timeout phase={self.active_turn_phase} dist={dist:.3f}m '
                f'turn_progress={math.degrees(turn_progress):.1f}/'
                f'{math.degrees(self.active_turn_target_delta):.1f}deg'
            )

    def _segment_progress(self, *, signed_lateral: bool = False) -> Tuple[float, float, float]:
        segment_dx = self.active_goal_x - self.active_segment_start_x
        segment_dy = self.active_goal_y - self.active_segment_start_y
        segment_length = math.hypot(segment_dx, segment_dy)
        if segment_length <= 1e-6:
            return 0.0, 0.0, 0.0

        unit_x = segment_dx / segment_length
        unit_y = segment_dy / segment_length
        rel_x = self.pose_x - self.active_segment_start_x
        rel_y = self.pose_y - self.active_segment_start_y
        along = rel_x * unit_x + rel_y * unit_y
        lateral = rel_x * unit_y - rel_y * unit_x
        if not signed_lateral:
            lateral = abs(lateral)
        remaining = max(segment_length - along, 0.0)
        return along, lateral, remaining

    def _active_segment_length(self) -> float:
        return math.hypot(
            self.active_goal_x - self.active_segment_start_x,
            self.active_goal_y - self.active_segment_start_y,
        )

    def _turn_arc_override(self, direction_sign: int) -> Tuple[float, float]:
        if direction_sign >= 0:
            return (
                self.turn_waypoint_arc_inner_norm,
                self.turn_waypoint_arc_outer_norm,
            )
        return (
            self.turn_waypoint_arc_outer_norm,
            self.turn_waypoint_arc_inner_norm,
        )

    def _turn_slowdown_override(self, direction_sign: int) -> Tuple[float, float]:
        if direction_sign >= 0:
            return (
                self.turn_waypoint_slowdown_ccw_left_norm,
                self.turn_waypoint_slowdown_ccw_right_norm,
            )
        return (
            self.turn_waypoint_slowdown_cw_left_norm,
            self.turn_waypoint_slowdown_cw_right_norm,
        )

    def _arrival_reason(self, dist: float, yaw_ok: bool) -> Optional[str]:
        if dist <= self.active_goal_arrival_tolerance and yaw_ok:
            return f'distance<{self.active_goal_arrival_tolerance:.2f}m'
        if not self.active_goal_allow_pass_through or not yaw_ok:
            return None

        segment_dx = self.active_goal_x - self.active_segment_start_x
        segment_dy = self.active_goal_y - self.active_segment_start_y
        segment_length = math.hypot(segment_dx, segment_dy)
        if segment_length <= 1e-6:
            return None

        unit_x = segment_dx / segment_length
        unit_y = segment_dy / segment_length
        rel_x = self.pose_x - self.active_segment_start_x
        rel_y = self.pose_y - self.active_segment_start_y
        along = rel_x * unit_x + rel_y * unit_y
        lateral = abs(rel_x * unit_y - rel_y * unit_x)
        remaining = segment_length - along

        # 2026-05-04 10:42 CST: forward points near the pole route often stop
        # with a small remaining distance but very low lateral error. Accept a
        # narrow corridor near the segment end, or a clean pass-through beyond
        # the target line, instead of forcing a circular stop radius.
        # Rollback: set forward_waypoint_pass_through_enabled=false.
        if lateral > self.forward_waypoint_pass_through_lateral_tolerance_m:
            return None
        if along >= segment_length:
            return (
                f'pass_through along={along:.3f}m '
                f'lateral={lateral:.3f}m'
            )
        if remaining <= self.forward_waypoint_pass_through_longitudinal_tolerance_m:
            return (
                f'corridor_remaining={max(remaining, 0.0):.3f}m '
                f'lateral={lateral:.3f}m'
            )
        return None

    def _goal_yaw(
        self,
        dx: float,
        dy: float,
        idx: int,
        locked_yaw: Optional[float] = None,
    ) -> float:
        if locked_yaw is not None:
            return locked_yaw
        body_relative_turn_delta_rad = self._turn_waypoint_body_relative_delta_rad(idx)
        if self.waypoint_use_direct_step_override[idx] and body_relative_turn_delta_rad is not None:
            return self._normalize_angle(self.pose_yaw + body_relative_turn_delta_rad)
        mode = self.goal_yaw_mode
        if idx == 0 and self.first_forward_goal_yaw_mode != 'inherit':
            mode = self.first_forward_goal_yaw_mode
        if mode == 'waypoint':
            return math.radians(self.waypoint_yaws_deg[idx])
        if mode == 'zero':
            return 0.0
        if mode == 'path_heading' and math.hypot(dx, dy) > 1e-6:
            return math.atan2(dy, dx)
        return self.pose_yaw

    def _publish_startup_step_override(self) -> None:
        if not self.route_controller_active:
            return
        if self.route_finished or self.route_failed:
            return
        if self.pole_pre_align_active:
            return
        if self.triggered and not self.startup_route_handoff_active:
            return
        if self._startup_should_hold_for_near_pole():
            self._publish_step_override(self.startup_step_override_mode, 0.0, 0.0)
            return
        if self.startup_override_start_time is None:
            self.startup_override_start_time = self._now_sec()
        scale = 1.0
        if self.startup_ramp_duration_sec > 1e-6:
            elapsed = self._now_sec() - self.startup_override_start_time
            scale = min(max(elapsed / self.startup_ramp_duration_sec, 0.0), 1.0)
        self._publish_step_override(
            self.first_forward_route_mode if self.startup_route_handoff_active else self.startup_step_override_mode,
            self.startup_left_norm * scale,
            self.startup_right_norm * scale,
        )

    def _startup_should_hold_for_near_pole(self) -> bool:
        if not self.startup_hold_when_near_enabled:
            return False
        if self.triggered or self.route_start_pending or self.pole_pre_align_active:
            return False
        if self.last_pole is None or self._nearest_is_stale():
            return False
        if self.startup_hold_when_near_distance_m <= 0.0:
            return False
        if self.last_pole.distance_m > self.startup_hold_when_near_distance_m:
            return False
        if (
            self.startup_hold_when_near_lateral_abs_m > 0.0
            and abs(self.last_pole.lateral_m) > self.startup_hold_when_near_lateral_abs_m
        ):
            return False
        return True

    def _cancel_startup_override_timer(self) -> None:
        if self.startup_override_timer is not None:
            self.startup_override_timer.cancel()
            self.startup_override_timer = None

    def _ensure_startup_override_timer(self) -> None:
        if not self.startup_step_override_enabled:
            return
        if self.startup_override_timer is not None:
            return
        self.startup_override_timer = self.create_timer(
            1.0 / self.startup_override_rate_hz,
            self._publish_startup_step_override,
        )

    def _arm_startup_route_handoff(self) -> None:
        if (
            self.startup_step_override_enabled
            and self.startup_route_handoff_enabled
            and self.startup_route_handoff_max_hold_sec > 1e-6
        ):
            self.startup_route_handoff_active = True
            self.startup_route_handoff_deadline = (
                self._now_sec() + self.startup_route_handoff_max_hold_sec
            )
            self.startup_route_handoff_republish_attempts = 0
            return

        self._release_startup_route_handoff('disabled', log_release=False)

    def _release_startup_route_handoff(self, reason: str, *, log_release: bool = True) -> None:
        had_handoff = self.startup_route_handoff_active
        self.startup_route_handoff_active = False
        self.startup_route_handoff_deadline = 0.0
        self._cancel_startup_override_timer()
        self._clear_step_override()
        if had_handoff and log_release:
            self.get_logger().info(f'释放 startup handoff: reason={reason}')

    def _maybe_release_startup_route_handoff(self) -> None:
        if not self.startup_route_handoff_active:
            return
        if self.route_finished or self.route_failed:
            self._release_startup_route_handoff('route_done', log_release=False)
            return
        if self.startup_route_handoff_deadline > 0.0 and self._now_sec() >= self.startup_route_handoff_deadline:
            if (
                self.startup_route_handoff_republish_goal_on_timeout
                and self.active_goal
                and self.startup_route_handoff_republish_attempts <
                self.startup_route_handoff_max_republish_attempts
            ):
                self.startup_route_handoff_republish_attempts += 1
                self._publish_active_goal_pose()
                self.startup_route_handoff_deadline = (
                    self._now_sec() + self.startup_route_handoff_max_hold_sec
                )
                self.get_logger().info(
                    'startup handoff timeout: republish active goal '
                    f'attempt={self.startup_route_handoff_republish_attempts}/'
                    f'{self.startup_route_handoff_max_republish_attempts}'
                )
                return
            self._release_startup_route_handoff('timeout')

    def _route_cmd_vel_callback(self, msg: Twist) -> None:
        if not self.startup_route_handoff_active:
            return
        if (
            abs(msg.linear.x) < self.startup_route_handoff_min_linear_speed_mps
            and abs(msg.angular.z) < self.startup_route_handoff_min_angular_speed_rps
        ):
            return
        self._release_startup_route_handoff('cmd_vel_ready')

    def _refresh_route_mode(self) -> None:
        if not self.route_controller_active:
            return
        if not self.triggered or self.route_finished or self.route_failed:
            return
        self._set_route_mode_active(True)

    def _reset_route_runtime(self) -> None:
        self._release_startup_route_handoff('reset', log_release=False)
        self.triggered = False
        self.route_finished = False
        self.route_failed = False
        self.current_idx = 0
        self.active_goal = False
        self.route_start_pending = False
        self.pending_route_reason = ''
        self.awaiting_next_goal = False
        self.next_goal_time = 0.0
        self.active_goal_x = 0.0
        self.active_goal_y = 0.0
        self.active_goal_yaw = 0.0
        self.active_goal_nav_yaw = 0.0
        self.active_goal_frame_id = ''
        self.active_goal_arrival_tolerance = self.arrival_tolerance
        self.active_goal_use_yaw_tolerance = False
        self.active_goal_allow_pass_through = False
        self.active_goal_has_direct_override = False
        self.active_goal_direct_override = False
        self.active_goal_has_forward_override = False
        self.active_goal_has_forward_tracking = False
        self.active_goal_is_turn_primitive = False
        self.active_turn_phase = ''
        self.active_turn_settle_reason = ''
        self.active_turn_settle_dist = 0.0
        self.active_goal_stamp = 0.0
        self.active_segment_start_x = 0.0
        self.active_segment_start_y = 0.0
        self.active_turn_direction_sign = 0
        self.active_turn_start_yaw = 0.0
        self.active_turn_last_yaw = 0.0
        self.active_turn_progress = 0.0
        self.active_turn_target_delta = 0.0
        self.active_turn_uses_body_relative_yaw = False
        self.active_turn_uses_locked_yaw = False
        self.active_turn_body_relative_delta_rad = 0.0
        self.active_turn_segment_length = 0.0
        self.active_turn_start_time = 0.0
        self.active_turn_phase_start_time = 0.0
        self.active_turn_pole_distance_gate_m = 0.0
        self.route_anchor_ready = False
        self.route_anchor_x = 0.0
        self.route_anchor_y = 0.0
        self.route_anchor_yaw = 0.0
        self.route_anchor_frame = ''
        self.startup_override_start_time = None
        self.last_state = None
        self.last_feedback = None
        self._reset_pole_pre_align_runtime()

    def _activate_controller(self, reason: str, *, log_wait: bool = False) -> None:
        self.route_controller_active = True
        self._reset_route_runtime()
        self._ensure_startup_override_timer()
        self._publish_fixed_step_override_state(True)
        if self.publish_mode_at_start:
            self._publish_mode(self.startup_step_override_mode)
        if self.startup_step_override_enabled:
            self._publish_startup_step_override()
        if self.trigger_source == 'apriltag':
            self._publish_state('WAITING_APRILTAG_TRIGGER')
        elif self.trigger_source == 'both':
            self._publish_state('WAITING_POLE_OR_APRILTAG_TRIGGER')
        else:
            self._publish_state('WAITING_ORANGE_POLE_TRIGGER')
        if log_wait:
            if self._apriltag_trigger_enabled():
                tag_text = (
                    f'AprilTag id={self.apriltag_target_tag_id}, '
                    f'tf={self.apriltag_target_frame_id}->{self.apriltag_target_child_frame_id}, '
                    f'detections={self.apriltag_detections_topic}'
                )
            else:
                tag_text = 'AprilTag disabled'
            self._publish_feedback(
                f'等待绕杆触发: trigger_source={self.trigger_source}, '
                f'HSV nearest={self.nearest_topic}, trigger={self.trigger_topic}, '
                f'distance<={self.trigger_distance_m:.2f}m, {tag_text}, '
                f'first_forward_mode={self.first_forward_route_mode}, '
                f'forward_mode={self.forward_route_mode}, turn_mode={self.turn_route_mode}, '
                f'offsets={self.waypoint_offsets}, reference={self.route_reference_mode}, '
                f'goal_yaw_mode={self.goal_yaw_mode}'
            )
        if self._cached_trigger_ready_for_route_start():
            self._start_route(
                f'external activate: fresh cached HSV trigger=True {self._pole_text(self.last_pole)}',
                source='hsv',
            )
        elif self._cached_apriltag_ready_for_route_start():
            self._start_route(
                f'external activate: fresh cached AprilTag {self._apriltag_detection_text()}',
                source='apriltag',
            )

    def _deactivate_controller(self, reason: str) -> None:
        self.route_controller_active = False
        self._release_startup_route_handoff(reason, log_release=False)
        self._clear_step_override()
        self._set_route_mode_active(False)
        self.last_state = None
        self._publish_state('INACTIVE')
        self._publish_feedback(f'orange_pole_relative_nav deactivated: reason={reason}')

    def _active_callback(self, msg: Bool) -> None:
        requested = bool(msg.data)
        if requested == self.route_controller_active:
            return
        if requested:
            self._clear_cached_pole_detection()
            self._activate_controller('external_topic', log_wait=True)
            return
        self._deactivate_controller('external_topic')

    def _set_route_mode_active(self, active: bool) -> None:
        if active:
            if self.disable_fixed_step_override_during_route:
                # 2026-05-03 01:40 CST: route_mode=6 must be active for all
                # six HSV-triggered goals, but the serial bridge also has a
                # legacy fixed mode=6 step override. Disable only that fixed
                # mapping during route tracking. Rollback: set
                # disable_fixed_step_override_during_route=false.
                self._publish_fixed_step_override_state(False)
            self._publish_mode(self._active_route_mode())
            return

        if self.disable_fixed_step_override_during_route:
            self._publish_fixed_step_override_state(True)

    def _publish_step_override(self, mode: int, left_norm: float, right_norm: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(mode), float(left_norm), float(right_norm)]
        self.step_override_pub.publish(msg)

    def _clear_step_override(self) -> None:
        msg = Float32MultiArray()
        msg.data = [-1.0, 0.0, 0.0]
        self.step_override_pub.publish(msg)

    def _publish_arrival(self, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        self.arrival_pub.publish(msg)

    def _publish_mode(self, mode: int) -> None:
        msg = Int32()
        msg.data = int(mode)
        self.mode_pub.publish(msg)

    def _publish_fixed_step_override_state(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self.fixed_step_override_state_pub.publish(msg)

    def _publish_state(self, state: str) -> None:
        if state == self.last_state:
            return
        self.last_state = state
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f'state={state}')

    def _publish_feedback(self, text: str) -> None:
        if text == self.last_feedback:
            return
        self.last_feedback = text
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)
        self.get_logger().info(text)

    def _publish_progress(self, text: str) -> None:
        now = self._now_sec()
        if (now - self.last_progress_log_time) < self.progress_log_interval_sec:
            return
        self.last_progress_log_time = now
        self._publish_feedback(text)

    def _publish_status_heartbeat(self) -> None:
        if self.status_heartbeat_sec <= 0.0:
            return

        now = self._now_sec()
        if (now - self.last_status_heartbeat_time) < self.status_heartbeat_sec:
            return
        self.last_status_heartbeat_time = now

        # 2026-05-03 01:40 CST: publish a heartbeat while waiting so field
        # tests can tell whether the route node is alive, receiving HSV pole
        # data, and waiting on trigger/pose. Rollback:
        # status_heartbeat_sec=0.0.
        state_msg = String()
        state_msg.data = self.last_state or 'UNKNOWN'
        self.state_pub.publish(state_msg)

        if self.last_pole_msg_time is None:
            pole_age_text = 'none'
        else:
            pole_age_text = f'{now - self.last_pole_msg_time:.2f}s'

        if self.last_trigger_msg_time is None:
            trigger_age_text = 'none'
        else:
            trigger_age_text = f'{now - self.last_trigger_msg_time:.2f}s'

        nearest_stale = self._nearest_is_stale()
        feedback = (
            f'HEARTBEAT state={state_msg.data}, triggered={self.triggered}, '
            f'trigger_source={self.trigger_source}, route_source={self.route_source or "none"}, '
            f'reference={self.route_reference_mode}, route_frame_ready={self.route_anchor_ready}, '
            f'route_finished={self.route_finished}, route_failed={self.route_failed}, '
            f'active_goal={self.active_goal}, direct_override={self.active_goal_direct_override}, '
            f'forward_override={self.active_goal_has_forward_override}, '
            f'forward_tracking={self.active_goal_has_forward_tracking}, '
            f'waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)}, '
            f'pole_age={pole_age_text}, pole_stale={nearest_stale}, '
            f'trigger_age={trigger_age_text}, detector_trigger={self.last_detector_trigger}, '
            f'nearest=({self.last_pole_text}), '
            f'apriltag_det=({self._apriltag_detection_text()}), '
            f'apriltag_tf=({self._apriltag_tf_text()}), '
            f'pre_align=({self._pole_pre_align_status_text()})'
        )
        msg = String()
        msg.data = feedback
        self.feedback_pub.publish(msg)
        self.get_logger().info(feedback)

    def shutdown(self) -> None:
        self._cancel_startup_override_timer()
        if not self.context.ok():
            return
        try:
            self._clear_step_override()
            self._set_route_mode_active(False)
            self._publish_mode(self.finish_mode)
        except Exception:
            return

    @staticmethod
    def _pole_text(pole: PoleSnapshot) -> str:
        return (
            f'dist={pole.distance_m:.3f}m, lateral={pole.lateral_m:.3f}m, '
            f'vertical={pole.vertical_m:.3f}m, pixel=({pole.pixel_x:.1f},{pole.pixel_y:.1f}), '
            f'box={pole.width_px:.0f}x{pole.height_px:.0f}, depth_px={pole.valid_depth_count}'
        )

    def _apriltag_detection_text(self) -> str:
        info = self.latest_apriltag_detection
        if info.received_sec <= 0.0:
            return 'none'
        age_sec = self._now_sec() - (
            info.stamp_sec if info.stamp_sec > 0.0 else info.received_sec
        )
        if not info.matched:
            return (
                f'target_id={self.apriltag_target_tag_id},match=false,'
                f'raw={info.raw_count},age={age_sec:.2f}s'
            )
        return (
            f'{info.selected_family}:{info.selected_id},match=true,raw={info.raw_count},'
            f'margin={info.decision_margin:.1f},hamming={info.hamming},'
            f'center=({info.centre_x:.0f},{info.centre_y:.0f}),age={age_sec:.2f}s'
        )

    def _apriltag_tf_text(self) -> str:
        tag = self.latest_apriltag
        if tag is None:
            return 'none'
        return (
            f'{tag.frame_id}->{tag.child_frame_id}, '
            f'xyz=({tag.x_m:+.3f},{tag.y_m:+.3f},{tag.z_m:+.3f})m, '
            f'age={self._apriltag_age_sec(tag):.2f}s'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _fresh_pole_distance(self) -> Optional[float]:
        if self.last_pole is None or self._nearest_is_stale():
            return None
        return self.last_pole.distance_m

    def _get_optional_array_parameter_value(self, name: str):
        try:
            return self.get_parameter(name).value
        except ParameterUninitializedException:
            return []

    @staticmethod
    def _stamp_to_sec(stamp: object) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _sign(value: float) -> float:
        return 1.0 if value >= 0.0 else -1.0

    @staticmethod
    def _clamp_norm(value: float) -> float:
        return min(1.0, max(-1.0, float(value)))

    @classmethod
    def _clamp_lr_delta(
        cls,
        left_norm: float,
        right_norm: float,
        max_lr_delta_norm: float,
    ) -> Tuple[float, float]:
        max_delta = max(0.0, float(max_lr_delta_norm))
        lr_delta = float(right_norm) - float(left_norm)
        if abs(lr_delta) <= max_delta:
            return left_norm, right_norm

        avg = 0.5 * (float(left_norm) + float(right_norm))
        clamped_delta = math.copysign(max_delta, lr_delta)
        return (
            cls._clamp_norm(avg - 0.5 * clamped_delta),
            cls._clamp_norm(avg + 0.5 * clamped_delta),
        )

    @staticmethod
    def _parse_waypoint_scalar_map(
        raw_indices,
        raw_values,
        waypoint_count: int,
        indices_name: str,
        values_name: str,
    ):
        indices = [int(item) for item in raw_indices]
        values = [float(item) for item in raw_values]
        if not indices:
            if values:
                raise RuntimeError(f'{values_name} requires {indices_name}.')
            return {}
        if len(values) != len(indices):
            raise RuntimeError(
                f'{values_name} must contain exactly one value per {indices_name} entry.'
            )

        scalar_map = {}
        for map_idx, waypoint_number in enumerate(indices):
            if waypoint_number < 1 or waypoint_number > waypoint_count:
                raise RuntimeError(
                    f'{indices_name} values must be within 1..{waypoint_count}.'
                )
            zero_based_idx = waypoint_number - 1
            if zero_based_idx in scalar_map:
                raise RuntimeError(f'{indices_name} contains duplicates.')
            scalar_map[zero_based_idx] = values[map_idx]
        return scalar_map

    @staticmethod
    def _parse_offsets(raw_value) -> List[Tuple[float, float]]:
        values = [float(item) for item in raw_value]
        if len(values) < 2 or len(values) % 2 != 0:
            raise RuntimeError('waypoint_offsets_xy must contain x/y pairs.')
        return [(values[idx], values[idx + 1]) for idx in range(0, len(values), 2)]

    @staticmethod
    def _cumulative_offsets(offsets: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        cumulative: List[Tuple[float, float]] = []
        total_x = 0.0
        total_y = 0.0
        for dx, dy in offsets:
            total_x += dx
            total_y += dy
            cumulative.append((total_x, total_y))
        return cumulative

    @staticmethod
    def _parse_optional_float_list(raw_value, expected_len: int, name: str, default: float) -> List[float]:
        values = [float(item) for item in raw_value]
        if not values:
            return [float(default)] * expected_len
        if len(values) != expected_len:
            raise RuntimeError(f'{name} must be empty or contain {expected_len} values.')
        return values

    @staticmethod
    def _parse_optional_bool_list(raw_value, expected_len: int, name: str, default: bool) -> List[bool]:
        values = [bool(item) for item in raw_value]
        if not values:
            return [bool(default)] * expected_len
        if len(values) != expected_len:
            raise RuntimeError(f'{name} must be empty or contain {expected_len} values.')
        return values

    @staticmethod
    def _parse_body_relative_waypoint_map(raw_indices, raw_offsets, waypoint_count: int):
        indices = [int(item) for item in raw_indices]
        offsets = [float(item) for item in raw_offsets]
        if not indices:
            if offsets:
                raise RuntimeError(
                    'body_relative_waypoint_offsets_fl requires body_relative_waypoint_indices.'
                )
            return {}
        if len(offsets) != len(indices) * 2:
            raise RuntimeError(
                'body_relative_waypoint_offsets_fl must contain forward/left pairs '
                'for every body_relative_waypoint_indices entry.'
            )

        offset_map = {}
        for map_idx, waypoint_number in enumerate(indices):
            if waypoint_number < 1 or waypoint_number > waypoint_count:
                raise RuntimeError(
                    f'body_relative_waypoint_indices values must be within 1..{waypoint_count}.'
                )
            zero_based_idx = waypoint_number - 1
            if zero_based_idx in offset_map:
                raise RuntimeError('body_relative_waypoint_indices contains duplicates.')
            offset_map[zero_based_idx] = (
                offsets[map_idx * 2],
                offsets[map_idx * 2 + 1],
            )
        return offset_map

    @staticmethod
    def _parse_waypoint_threshold_map(
        raw_indices,
        raw_thresholds,
        waypoint_count: int,
        indices_name: str,
        thresholds_name: str,
    ):
        indices = [int(item) for item in raw_indices]
        thresholds = [float(item) for item in raw_thresholds]
        if not indices:
            if thresholds:
                raise RuntimeError(f'{thresholds_name} requires {indices_name}.')
            return {}
        if len(thresholds) != len(indices):
            raise RuntimeError(
                f'{thresholds_name} must contain exactly one value per {indices_name} entry.'
            )

        threshold_map = {}
        for map_idx, waypoint_number in enumerate(indices):
            if waypoint_number < 1 or waypoint_number > waypoint_count:
                raise RuntimeError(
                    f'{indices_name} values must be within 1..{waypoint_count}.'
                )
            zero_based_idx = waypoint_number - 1
            if zero_based_idx in threshold_map:
                raise RuntimeError(f'{indices_name} contains duplicates.')
            threshold_m = thresholds[map_idx]
            if threshold_m <= 0.0:
                raise RuntimeError(f'{thresholds_name} values must be > 0.')
            threshold_map[zero_based_idx] = threshold_m
        return threshold_map

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _turn_direction_from_override(left_norm: float, right_norm: float) -> int:
        if right_norm > left_norm:
            return 1
        if right_norm < left_norm:
            return -1
        return 0

    def _resolved_turn_override(
        self,
        idx: int,
        requested_direction_sign: Optional[int] = None,
    ) -> Tuple[float, float, int]:
        left_norm = self.waypoint_direct_step_left_norms[idx]
        right_norm = self.waypoint_direct_step_right_norms[idx]
        if requested_direction_sign is None:
            return left_norm, right_norm, self._turn_direction_from_override(left_norm, right_norm)

        if requested_direction_sign == 0:
            return 0.0, 0.0, 0

        forward_norm = max(abs(left_norm), abs(right_norm))
        reverse_norm = min(abs(left_norm), abs(right_norm))
        if forward_norm <= 1e-6 and reverse_norm <= 1e-6:
            return 0.0, 0.0, 0

        if requested_direction_sign > 0:
            return -reverse_norm, forward_norm, 1
        return forward_norm, -reverse_norm, -1

    @classmethod
    def _directional_angle_delta(cls, start_yaw: float, end_yaw: float, direction_sign: int) -> float:
        if direction_sign >= 0:
            delta = cls._normalize_angle(end_yaw - start_yaw)
        else:
            delta = cls._normalize_angle(start_yaw - end_yaw)
        if delta < 0.0:
            delta += 2.0 * math.pi
        return delta

    @staticmethod
    def _yaw_from_quaternion(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _rpy_from_quaternion(q: Quaternion) -> Tuple[float, float, float]:
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrangePoleRelativeNav()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
