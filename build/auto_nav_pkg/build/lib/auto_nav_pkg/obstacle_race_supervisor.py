#!/usr/bin/env python3
"""Integrated obstacle-race supervisor for randomized post-stair obstacles."""

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

from apriltag_msgs.msg import AprilTagDetectionArray
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_msgs.msg import Bool, Float32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener
from tf2_msgs.msg import TFMessage
import yaml

from auto_nav_pkg.lateral_s_curve import (
    LateralSCurvePlan,
    build_lateral_s_curve_plan,
)
from auto_nav_pkg.yaw_rate_estimator import WindowedYawRateEstimator


@dataclass
class DetectionState:
    triggered: bool = False
    trigger_stamp_sec: float = 0.0
    trigger_start_stamp_sec: float = 0.0
    distance_m: Optional[float] = None
    lateral_m: Optional[float] = None
    nearest_stamp_sec: float = 0.0

    def is_fresh(self, now_sec: float, timeout_sec: float) -> bool:
        if self.triggered and (now_sec - self.trigger_stamp_sec) <= timeout_sec:
            return True
        if self.distance_m is not None and (now_sec - self.nearest_stamp_sec) <= timeout_sec:
            return True
        return False

    def reset(self) -> None:
        self.triggered = False
        self.trigger_stamp_sec = 0.0
        self.trigger_start_stamp_sec = 0.0
        self.distance_m = None
        self.lateral_m = None
        self.nearest_stamp_sec = 0.0


@dataclass
class LimitBarTagSample:
    received_stamp_sec: float
    x_m: float
    y_m: float
    z_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    frame_id: str
    child_frame_id: str


@dataclass
class SandpitTagSample:
    received_stamp_sec: float
    x_m: float
    y_m: float
    z_m: float
    distance_m: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    frame_id: str
    child_frame_id: str


def _rotate_optical_xz_for_yaw_delta(
    x_m: float,
    z_m: float,
    yaw_delta_rad: float,
) -> tuple[float, float]:
    """Express a fixed target in the camera frame after a robot yaw change."""
    cos_yaw = math.cos(yaw_delta_rad)
    sin_yaw = math.sin(yaw_delta_rad)
    return (
        cos_yaw * x_m + sin_yaw * z_m,
        -sin_yaw * x_m + cos_yaw * z_m,
    )


class ObstacleRaceSupervisor(Node):
    STATE_WAIT_START = 'WAIT_START'
    STATE_STAIRS = 'STAIRS_ACTIVE'
    STATE_SEARCH = 'SEARCH'
    STATE_LIMIT_BAR_ALIGN = 'LIMIT_BAR_ALIGN'
    STATE_LIMIT_BAR_LATERAL_SHIFT = 'LIMIT_BAR_LATERAL_SHIFT'
    STATE_LIMIT_BAR_STOP_BUFFER = 'LIMIT_BAR_STOP_BUFFER'
    STATE_LIMIT_BAR_PREPARE = 'LIMIT_BAR_PREPARE'
    STATE_LIMIT_BAR = 'LIMIT_BAR_DUCK'
    STATE_LIMIT_BAR_WAIT = 'LIMIT_BAR_WAIT_RESUME'
    STATE_LIMIT_BAR_POST_RESUME_ALIGN = 'LIMIT_BAR_POST_RESUME_ALIGN'
    STATE_HURDLE_ALIGN = 'HURDLE_TAG_ALIGN'
    STATE_HURDLE_WAIT_JUMP = 'HURDLE_WAIT_JUMP'
    STATE_HURDLE_WAIT_RESUME = 'HURDLE_WAIT_RESUME'
    STATE_HURDLE_POST_ALIGN = 'HURDLE_POST_ALIGN'
    STATE_UPSTAIRS_ALIGN = 'UPSTAIRS_TAG_ALIGN'
    STATE_POLE = 'POLE_ACTIVE'
    STATE_SANDPIT_PRE_ALIGN = 'SANDPIT_PRE_ALIGN'
    STATE_SANDPIT_BYPASS = 'SANDPIT_BYPASS'
    STATE_SLOPE_ALIGN = 'SLOPE_ALIGN'
    STATE_POST_STAIRS_SLOPE_APPROACH = 'POST_STAIRS_SLOPE_APPROACH'
    STATE_POST_STAIRS_SLOPE_CONFIRM = 'POST_STAIRS_SLOPE_CONFIRM'
    STATE_SLOPE = 'SLOPE_ACTIVE'
    STATE_DONE = 'DONE'
    STATE_FAILED = 'FAILED'

    def __init__(self) -> None:
        super().__init__('obstacle_race_supervisor')

        self.declare_parameter('pose_topic', '/Odometry')
        self.declare_parameter('pose_stale_timeout_sec', 5.0)
        self.declare_parameter('step_override_topic', '/serial_step_override')
        self.declare_parameter('step_once_topic', '/serial_step_once')
        self.declare_parameter('arrival_status_topic', '/agent/arrival_status')
        self.declare_parameter('state_topic', '/obstacle_race_supervisor/state')
        self.declare_parameter('feedback_topic', '/obstacle_race_supervisor/feedback_log')
        self.declare_parameter('sandpit_bypass_file', '')
        self.declare_parameter('pole_bypass_file', '')
        self.declare_parameter('apriltag_tf_topic', '/tf')
        self.declare_parameter('apriltag_tf_queue_depth', 100)
        self.declare_parameter('apriltag_detections_topic', '/apriltag/detections')

        self.declare_parameter('limit_trigger_topic', '/limit_bar_duck_test/trigger')
        self.declare_parameter('limit_nearest_topic', '/limit_bar_duck_test/nearest')
        self.declare_parameter('hurdle_trigger_topic', '/orange_hurdle_jump_test/trigger')
        self.declare_parameter('hurdle_nearest_topic', '/orange_hurdle_jump_test/nearest')
        self.declare_parameter('pole_trigger_topic', '/orange_pole_detector/trigger')
        self.declare_parameter('pole_nearest_topic', '/orange_pole_detector/nearest')
        self.declare_parameter('slope_trigger_topic', '/slope_branch_detector/trigger')
        self.declare_parameter('slope_nearest_topic', '/slope_branch_detector/nearest')
        self.declare_parameter('upstairs_trigger_topic', '/upstairs_detector/trigger')
        self.declare_parameter('upstairs_nearest_topic', '/upstairs_detector/nearest')

        self.declare_parameter('stairs_active_topic', '/obstacle_supervisor/stairs_active')
        self.declare_parameter('stairs_state_topic', '/stairs_manager/state')
        self.declare_parameter('slope_active_topic', '/obstacle_supervisor/slope_active')
        self.declare_parameter('slope_state_topic', '/slope_manager/state')
        self.declare_parameter('slope_current_waypoint_topic', '/slope_manager/current_waypoint')
        self.declare_parameter(
            'slope_retry_restart_from_waypoint_topic',
            '/slope_manager/restart_from_waypoint',
        )
        self.declare_parameter('pole_active_topic', '/orange_pole_relative_nav/active')
        self.declare_parameter('pole_state_topic', '/orange_pole_relative_nav/state')
        self.declare_parameter('pole_feedback_topic', '/orange_pole_relative_nav/feedback_log')

        self.declare_parameter('start_with_stairs_manager', True)
        self.declare_parameter('wait_for_start_signal', False)
        self.declare_parameter('start_signal_topic', '/motion_bridge/start_signal')
        self.declare_parameter('start_retry_guard_sec', 5.0)
        self.declare_parameter('finish_signal_topic', '/motion_bridge/finish_signal')
        self.declare_parameter('action_ack_topic', '/motion_bridge/action_ack')
        self.declare_parameter('serial_connected_topic', '/motion_bridge/serial_connected')
        self.declare_parameter('start_wait_mode', 0)
        self.declare_parameter('start_wait_left_norm', 0.0)
        self.declare_parameter('start_wait_right_norm', 0.0)
        self.declare_parameter('search_publish_hz', 10.0)
        self.declare_parameter('search_mode', 0)
        self.declare_parameter('search_left_norm', 0.5)
        self.declare_parameter('search_right_norm', 0.5)
        self.declare_parameter('search_rearm_delay_sec', 0.8)
        self.declare_parameter('search_yaw_correction_enabled', False)
        self.declare_parameter('search_yaw_use_startup_yaw', True)
        self.declare_parameter('search_yaw_target_deg', 0.0)
        self.declare_parameter('search_yaw_tolerance_deg', 3.0)
        self.declare_parameter('search_yaw_gain_per_rad', 0.35)
        self.declare_parameter('search_yaw_max_delta_norm', 0.20)
        self.declare_parameter('search_yaw_log_interval_sec', 1.0)
        self.declare_parameter('detection_stale_timeout_sec', 0.75)
        self.declare_parameter('limit_trigger_min_duration_sec', 0.35)
        self.declare_parameter('hurdle_trigger_min_duration_sec', 0.20)
        self.declare_parameter('slope_trigger_min_duration_sec', 0.35)
        self.declare_parameter('pole_visual_trigger_enabled', True)
        self.declare_parameter('pole_trigger_min_duration_sec', 0.35)
        self.declare_parameter('upstairs_trigger_min_duration_sec', 0.0)
        self.declare_parameter('pole_retry_failures_before_bypass', 2)
        self.declare_parameter('pole_retry_search_rearm_delay_sec', 0.8)
        self.declare_parameter('pole_bypass_enabled', True)
        self.declare_parameter('pole_bypass_force_on_detection', False)
        self.declare_parameter('pole_bypass_pre_bypass_waypoints', 0)
        self.declare_parameter('pole_bypass_complete_once', True)
        self.declare_parameter('pole_bypass_search_rearm_delay_sec', 0.8)
        self.declare_parameter('upstairs_trigger_distance_m', 0.610)
        self.declare_parameter('upstairs_require_hurdle_completed', False)
        self.declare_parameter('upstairs_slope_guard_enabled', True)
        self.declare_parameter('upstairs_slope_guard_distance_m', 1.50)
        self.declare_parameter('upstairs_tag_trigger_enabled', False)
        self.declare_parameter('upstairs_tag_target_tag_id', 18)
        self.declare_parameter('upstairs_tag_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('upstairs_tag_child_frame_id', 'upstairs')
        self.declare_parameter('upstairs_tag_to_center_x_m', 0.0)
        self.declare_parameter('upstairs_tag_stale_timeout_sec', 0.60)
        self.declare_parameter('upstairs_tag_min_samples', 1)
        self.declare_parameter('upstairs_tag_max_sample_age_sec', 0.50)
        self.declare_parameter('upstairs_tag_trigger_distance_m', 1.70)
        self.declare_parameter('upstairs_tag_visual_fallback_enabled', False)
        self.declare_parameter(
            'upstairs_tag_visual_fallback_require_hurdle_completed',
            True,
        )
        self.declare_parameter(
            'upstairs_tag_visual_fallback_require_recent_tag_id',
            False,
        )
        self.declare_parameter(
            'upstairs_tag_visual_fallback_max_detection_age_sec',
            2.0,
        )
        self.declare_parameter('upstairs_tag_odom_handoff_enabled', True)
        self.declare_parameter('upstairs_tag_odom_handoff_timeout_sec', 2.0)
        self.declare_parameter('upstairs_tag_odom_handoff_max_start_distance_m', 1.85)
        self.declare_parameter('upstairs_tag_slope_guard_enabled', True)
        self.declare_parameter('upstairs_tag_slope_guard_distance_m', 1.85)
        self.declare_parameter('upstairs_tag_yaw_reference_mode', 'fixed_absolute')
        self.declare_parameter('upstairs_tag_fixed_reference_yaw_deg', 0.0)
        self.declare_parameter('upstairs_tag_yaw_align_from_lateral_enabled', False)
        self.declare_parameter('upstairs_lateral_pre_align_enabled', True)
        self.declare_parameter('upstairs_lateral_pre_align_mode', 'fixed_absolute_yaw')
        self.declare_parameter('upstairs_lateral_pre_align_use_visual_feedback', False)
        self.declare_parameter('upstairs_lateral_pre_align_timeout_sec', 8.0)
        self.declare_parameter('upstairs_lateral_pre_align_max_visual_error_deg', 0.0)
        self.declare_parameter('upstairs_lateral_pre_align_hold_sec', 0.10)
        self.declare_parameter('upstairs_lateral_pre_align_tolerance_deg', 1.0)
        self.declare_parameter('upstairs_lateral_pre_align_pose_control_sign', -1.0)
        self.declare_parameter('upstairs_lateral_pre_align_target_pose_pitch_deg', 0.0)
        self.declare_parameter('slope_wait_for_upstairs_nearest_sec', 0.50)
        self.declare_parameter('done_hold_step_override_enabled', True)
        self.declare_parameter('done_hold_mode', 0)
        self.declare_parameter('done_hold_left_norm', 0.0)
        self.declare_parameter('done_hold_right_norm', 0.0)
        self.declare_parameter('upstairs_done_retry_guard_enabled', True)
        self.declare_parameter('upstairs_done_retry_guard_duration_sec', 30.0)
        self.declare_parameter('upstairs_done_retry_guard_mode', 0)
        self.declare_parameter('upstairs_done_retry_guard_left_norm', 0.1)
        self.declare_parameter('upstairs_done_retry_guard_right_norm', 0.1)
        self.declare_parameter('failed_hold_step_override_enabled', True)
        self.declare_parameter('failed_stop_nav_executor_enabled', True)
        self.declare_parameter('failed_hold_mode', 0)
        self.declare_parameter('failed_hold_left_norm', 0.0)
        self.declare_parameter('failed_hold_right_norm', 0.0)
        self.declare_parameter('pole_max_lateral_abs_m', 0.0)
        self.declare_parameter('pole_after_slope_search_delay_sec', 1.50)
        self.declare_parameter('pole_near_preempts_limit_bar', True)
        self.declare_parameter('pole_near_preempt_limit_bar_distance_m', 0.65)
        self.declare_parameter('pole_apriltag_trigger_enabled', False)
        self.declare_parameter('pole_apriltag_target_tag_id', 14)
        self.declare_parameter('pole_apriltag_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('pole_apriltag_child_frame_id', 'object')
        self.declare_parameter('pole_apriltag_stale_timeout_sec', 0.60)
        self.declare_parameter('pole_apriltag_detection_latch_sec', 0.70)
        self.declare_parameter('pole_apriltag_require_tf', True)
        self.declare_parameter('pole_apriltag_min_duration_sec', 0.0)
        self.declare_parameter('pole_apriltag_distance_axis', 'z')
        self.declare_parameter('pole_apriltag_trigger_distance_m', 1.00)
        self.declare_parameter('pole_apriltag_odom_handoff_enabled', True)
        self.declare_parameter('pole_apriltag_odom_handoff_timeout_sec', 20.0)
        self.declare_parameter(
            'pole_apriltag_odom_handoff_max_start_distance_m',
            2.0,
        )
        self.declare_parameter('limit_bar_nearest_preempts_slope', True)
        self.declare_parameter('limit_bar_nearest_preempt_distance_m', 1.25)
        self.declare_parameter('limit_bar_visual_trigger_enabled', True)
        self.declare_parameter('limit_bar_tag_trigger_enabled', False)
        self.declare_parameter('limit_bar_tag_target_tag_id', 12)

        self.declare_parameter('duck_prepare_mode', 2)
        self.declare_parameter('duck_prepare_left_norm', 0.0)
        self.declare_parameter('duck_prepare_right_norm', 0.0)
        self.declare_parameter('duck_prepare_stop_buffer_sec', 0.0)
        self.declare_parameter('duck_prepare_delay_sec', 0.8)
        self.declare_parameter('duck_walk_mode', 2)
        self.declare_parameter('duck_walk_left_norm', 1.0)
        self.declare_parameter('duck_walk_right_norm', 1.0)
        self.declare_parameter('duck_resume_mode', 0)
        self.declare_parameter('duck_resume_left_norm', 0.0)
        self.declare_parameter('duck_resume_right_norm', 0.0)
        self.declare_parameter('duck_resume_delay_sec', 1.0)
        self.declare_parameter('duck_travel_distance_m', 0.78)
        self.declare_parameter('duck_travel_projection_enabled', False)
        self.declare_parameter('duck_travel_timeout_sec', 8.0)
        self.declare_parameter('duck_progress_log_interval_sec', 1.0)
        self.declare_parameter('duck_travel_delta_x_m', 0.0)
        self.declare_parameter('duck_travel_delta_y_m', 0.78)
        self.declare_parameter('duck_travel_max_x_error_m', 0.35)
        self.declare_parameter('limit_bar_post_resume_yaw_align_enabled', True)
        self.declare_parameter('limit_bar_post_resume_yaw_tolerance_deg', 1.0)
        self.declare_parameter('limit_bar_post_resume_yaw_hold_sec', 0.30)
        self.declare_parameter('limit_bar_post_resume_yaw_timeout_sec', 2.0)
        self.declare_parameter('duck_pre_align_enabled', False)
        self.declare_parameter('duck_visual_yaw_align_enabled', True)
        self.declare_parameter('duck_visual_yaw_lateral_sign', -1.0)
        self.declare_parameter('duck_visual_yaw_max_offset_deg', 20.0)
        self.declare_parameter('duck_visual_yaw_stale_timeout_sec', 0.75)
        self.declare_parameter('duck_visual_yaw_min_distance_m', 0.20)
        self.declare_parameter('duck_yaw_correction_enabled', False)
        self.declare_parameter('duck_yaw_correction_tolerance_deg', 2.0)
        self.declare_parameter('duck_yaw_correction_gain_per_rad', 0.8)
        self.declare_parameter('duck_yaw_correction_max_delta_norm', 0.12)
        self.declare_parameter('duck_yaw_correction_force_max_delta', False)
        self.declare_parameter('duck_yaw_correction_sign', 1.0)
        self.declare_parameter('limit_bar_tag_align_enabled', False)
        self.declare_parameter('limit_bar_tag_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('limit_bar_tag_child_frame_id', 'limit_bar_right')
        self.declare_parameter('limit_bar_tag_to_center_x_m', -0.705)
        self.declare_parameter('limit_bar_tag_stale_timeout_sec', 0.60)
        self.declare_parameter('limit_bar_tag_min_samples', 3)
        self.declare_parameter('limit_bar_tag_max_sample_age_sec', 0.50)
        self.declare_parameter('limit_bar_tag_geometry_lock_enabled', True)
        self.declare_parameter('limit_bar_tag_geometry_lock_sample_count', 3)
        self.declare_parameter('limit_bar_tag_trigger_distance_m', 0.0)
        self.declare_parameter('limit_bar_tag_odom_handoff_enabled', True)
        self.declare_parameter('limit_bar_tag_odom_handoff_timeout_sec', 2.0)
        self.declare_parameter('limit_bar_tag_odom_handoff_max_start_distance_m', 1.85)
        self.declare_parameter('limit_bar_tag_presearch_handoff_enabled', True)
        self.declare_parameter('limit_bar_tag_presearch_handoff_max_abs_roll_deg', 30.0)
        self.declare_parameter('limit_bar_tag_presearch_handoff_max_abs_pitch_deg', 30.0)
        self.declare_parameter('limit_bar_tag_slope_guard_enabled', True)
        self.declare_parameter('limit_bar_tag_slope_guard_distance_m', 1.70)
        self.declare_parameter('limit_bar_tag_align_tolerance_deg', 2.0)
        self.declare_parameter('limit_bar_tag_align_timeout_sec', 3.0)
        self.declare_parameter('limit_bar_tag_align_yaw_offset_sign', -1.0)
        self.declare_parameter('limit_bar_tag_yaw_reference_mode', 'tag_lateral')
        self.declare_parameter('limit_bar_tag_fixed_reference_yaw_deg', 180.0)
        self.declare_parameter('limit_bar_tag_align_mode', 6)
        self.declare_parameter('limit_bar_tag_align_ccw_left_norm', -0.12)
        self.declare_parameter('limit_bar_tag_align_ccw_right_norm', 0.22)
        self.declare_parameter('limit_bar_tag_align_cw_left_norm', 0.22)
        self.declare_parameter('limit_bar_tag_align_cw_right_norm', -0.12)
        self.declare_parameter('limit_bar_tag_align_turn_scale_enabled', True)
        self.declare_parameter('limit_bar_tag_align_turn_min_scale', 0.45)
        self.declare_parameter('limit_bar_tag_align_yaw_gain_per_rad', 0.90)
        self.declare_parameter('limit_bar_tag_align_min_abs_angular_z', 0.05)
        self.declare_parameter('limit_bar_tag_align_max_abs_angular_z', 0.25)
        self.declare_parameter('limit_bar_lateral_precision_turn_enabled', False)
        self.declare_parameter('limit_bar_lateral_turn_far_error_deg', 35.0)
        self.declare_parameter('limit_bar_lateral_turn_far_scale', 1.25)
        self.declare_parameter('limit_bar_lateral_turn_yaw_damping_sec', 0.18)
        self.declare_parameter('limit_bar_lateral_turn_brake_yaw_rate_deg_s', 12.0)
        self.declare_parameter('limit_bar_lateral_turn_complete_yaw_rate_deg_s', 6.0)
        self.declare_parameter('limit_bar_lateral_turn_yaw_rate_filter_alpha', 1.0)
        self.declare_parameter('limit_bar_lateral_turn_yaw_rate_window_sec', 0.30)
        self.declare_parameter('limit_bar_lateral_turn_yaw_rate_min_span_sec', 0.18)
        self.declare_parameter('limit_bar_lateral_turn_yaw_rate_max_abs_deg_s', 90.0)
        self.declare_parameter('limit_bar_lateral_turn_hold_sec', 0.12)
        self.declare_parameter('limit_bar_lateral_turn_reverse_cooldown_sec', 0.20)
        self.declare_parameter('limit_bar_lateral_turn_timeout_sec', 8.0)
        self.declare_parameter(
            'limit_bar_lateral_turn_timeout_settle_grace_sec',
            0.60,
        )
        self.declare_parameter('limit_bar_lateral_shift_timeout_sec', 4.0)
        self.declare_parameter('limit_bar_lateral_final_forward_timeout_sec', 12.0)
        self.declare_parameter('lateral_s_curve_shadow_enabled', False)
        self.declare_parameter('hurdle_lateral_s_curve_shadow_enabled', False)
        self.declare_parameter('limit_bar_lateral_s_curve_shadow_enabled', False)
        self.declare_parameter('upstairs_lateral_s_curve_shadow_enabled', False)
        self.declare_parameter('lateral_s_curve_shadow_min_forward_m', 0.35)
        self.declare_parameter('lateral_s_curve_shadow_max_abs_lateral_m', 0.45)
        self.declare_parameter('lateral_s_curve_shadow_max_heading_deg', 25.0)
        self.declare_parameter('limit_bar_lateral_s_curve_max_heading_deg', 30.0)
        self.declare_parameter('lateral_s_curve_shadow_max_curvature_m_inv', 2.5)
        self.declare_parameter(
            'limit_bar_lateral_s_curve_max_curvature_m_inv',
            2.5,
        )
        self.declare_parameter('lateral_s_curve_control_enabled', False)
        self.declare_parameter('hurdle_lateral_s_curve_control_enabled', False)
        self.declare_parameter('limit_bar_lateral_s_curve_control_enabled', False)
        self.declare_parameter('upstairs_lateral_s_curve_control_enabled', False)
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_cruise_left_norm',
            0.32,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_cruise_right_norm',
            0.32,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_slow_left_norm',
            0.20,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_slow_right_norm',
            0.20,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_slowdown_distance_m',
            0.18,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_timeout_sec',
            8.0,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_fail_open_enabled',
            False,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_max_forward_overshoot_m',
            0.08,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_max_abs_cross_track_m',
            0.12,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_forward_tolerance_m',
            0.03,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_lateral_tolerance_m',
            0.05,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_arrival_settle_sec',
            0.20,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_pre_stop_distance_m',
            0.0,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_pre_stop_hold_sec',
            0.0,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_final_abs_lateral_tolerance_m',
            0.08,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_final_max_lateral_drift_m',
            0.05,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_final_lateral_violation_hold_sec',
            0.20,
        )
        self.declare_parameter(
            'limit_bar_lateral_s_curve_control_final_align_norm',
            0.08,
        )
        self.declare_parameter(
            'limit_bar_stop_nav_executor_on_entry_enabled',
            True,
        )
        self.declare_parameter(
            'upstairs_lateral_s_curve_control_cruise_left_norm',
            0.40,
        )
        self.declare_parameter(
            'upstairs_lateral_s_curve_control_cruise_right_norm',
            0.40,
        )
        self.declare_parameter(
            'upstairs_lateral_s_curve_control_slow_left_norm',
            0.25,
        )
        self.declare_parameter(
            'upstairs_lateral_s_curve_control_slow_right_norm',
            0.25,
        )
        self.declare_parameter(
            'upstairs_lateral_s_curve_control_slowdown_distance_m',
            0.25,
        )
        self.declare_parameter(
            'upstairs_lateral_s_curve_control_timeout_sec',
            12.0,
        )
        self.declare_parameter(
            'upstairs_lateral_s_curve_control_max_forward_overshoot_m',
            0.20,
        )
        self.declare_parameter(
            'upstairs_lateral_s_curve_control_distance_max_remaining_m',
            0.25,
        )
        self.declare_parameter(
            'upstairs_lateral_s_curve_control_precreep_yaw_gate_deg',
            4.0,
        )
        self.declare_parameter(
            'upstairs_lateral_s_curve_control_final_align_norm',
            0.08,
        )
        self.declare_parameter(
            'hurdle_lateral_s_curve_adaptive_clamp_enabled',
            False,
        )
        self.declare_parameter(
            'hurdle_lateral_s_curve_adaptive_clamp_min_scale',
            0.90,
        )
        self.declare_parameter('lateral_s_curve_control_cruise_left_norm', 0.40)
        self.declare_parameter('lateral_s_curve_control_cruise_right_norm', 0.40)
        self.declare_parameter('lateral_s_curve_control_slow_left_norm', 0.25)
        self.declare_parameter('lateral_s_curve_control_slow_right_norm', 0.25)
        self.declare_parameter('lateral_s_curve_control_slowdown_distance_m', 0.25)
        self.declare_parameter('lateral_s_curve_control_lookahead_m', 0.12)
        self.declare_parameter('lateral_s_curve_control_heading_gain_norm_per_rad', 0.90)
        self.declare_parameter('lateral_s_curve_control_cross_track_gain_rad_per_m', 1.50)
        self.declare_parameter('lateral_s_curve_control_cross_track_max_heading_deg', 10.0)
        self.declare_parameter('lateral_s_curve_control_max_delta_norm', 0.18)
        self.declare_parameter('lateral_s_curve_control_walk_yaw_gate_deg', 12.0)
        self.declare_parameter('lateral_s_curve_control_forward_tolerance_m', 0.05)
        self.declare_parameter('lateral_s_curve_control_lateral_tolerance_m', 0.10)
        self.declare_parameter('lateral_s_curve_control_yaw_tolerance_deg', 2.0)
        self.declare_parameter('lateral_s_curve_control_arrival_settle_sec', 0.0)
        self.declare_parameter('lateral_s_curve_control_completion_hold_sec', 0.20)
        self.declare_parameter(
            'lateral_s_curve_control_completion_max_retreat_m',
            0.0,
        )
        self.declare_parameter(
            'lateral_s_curve_control_completion_max_lateral_drift_m',
            0.0,
        )
        self.declare_parameter(
            'lateral_s_curve_control_completion_recovery_enabled',
            False,
        )
        self.declare_parameter(
            'lateral_s_curve_control_completion_recovery_left_norm',
            0.15,
        )
        self.declare_parameter(
            'lateral_s_curve_control_completion_recovery_right_norm',
            0.15,
        )
        self.declare_parameter(
            'lateral_s_curve_control_completion_recovery_yaw_gain_norm_per_rad',
            0.45,
        )
        self.declare_parameter(
            'lateral_s_curve_control_completion_recovery_yaw_max_delta_norm',
            0.06,
        )
        self.declare_parameter(
            'lateral_s_curve_control_completion_recovery_forward_tolerance_m',
            0.015,
        )
        self.declare_parameter(
            'lateral_s_curve_control_completion_recovery_yaw_tolerance_deg',
            10.0,
        )
        self.declare_parameter(
            'lateral_s_curve_control_completion_recovery_max_extra_forward_m',
            0.03,
        )
        self.declare_parameter(
            'lateral_s_curve_control_completion_recovery_max_abs_lateral_m',
            0.15,
        )
        self.declare_parameter(
            'hurdle_lateral_s_curve_final_align_independent_turn_enabled',
            False,
        )
        self.declare_parameter(
            'hurdle_lateral_s_curve_visual_completion_ignore_plan_overshoot_enabled',
            False,
        )
        self.declare_parameter(
            'hurdle_lateral_s_curve_visual_completion_max_plan_overshoot_m',
            0.25,
        )
        self.declare_parameter('lateral_s_curve_control_timeout_sec', 8.0)
        self.declare_parameter('lateral_s_curve_control_max_forward_overshoot_m', 0.15)
        self.declare_parameter('lateral_s_curve_control_max_abs_cross_track_m', 0.20)
        self.declare_parameter('limit_bar_lateral_pre_align_enabled', False)
        self.declare_parameter('limit_bar_lateral_pre_align_mode', 'pose_pitch')
        self.declare_parameter('limit_bar_lateral_pre_align_use_visual_feedback', True)
        self.declare_parameter('limit_bar_lateral_pre_align_timeout_sec', 8.0)
        self.declare_parameter('limit_bar_lateral_pre_align_max_visual_error_deg', 3.0)
        self.declare_parameter('limit_bar_lateral_pre_align_hold_sec', 0.10)
        self.declare_parameter('limit_bar_lateral_pre_align_tolerance_deg', 2.0)
        self.declare_parameter(
            'limit_bar_lateral_pre_align_stop_during_hold_enabled',
            True,
        )
        self.declare_parameter('limit_bar_lateral_pre_align_pose_control_sign', -1.0)
        self.declare_parameter('limit_bar_lateral_pre_align_target_pose_pitch_deg', 0.0)
        self.declare_parameter('limit_bar_lateral_shift_enabled', False)
        self.declare_parameter('limit_bar_lateral_shift_yaw_deg', 90.0)
        self.declare_parameter('limit_bar_lateral_shift_yaw_sign', -1.0)
        self.declare_parameter('limit_bar_lateral_shift_deadband_m', 0.03)
        self.declare_parameter('limit_bar_lateral_shift_arrival_tolerance_m', 0.03)
        self.declare_parameter('limit_bar_lateral_shift_skip_below_m', 0.18)
        self.declare_parameter('limit_bar_lateral_shift_distance_scale', 0.90)
        self.declare_parameter('limit_bar_lateral_shift_max_distance_m', 0.95)
        self.declare_parameter('limit_bar_lateral_shift_target_distance_m', 0.45)
        self.declare_parameter('limit_bar_lateral_shift_walk_mode', 0)
        self.declare_parameter('limit_bar_lateral_shift_walk_left_norm', 0.50)
        self.declare_parameter('limit_bar_lateral_shift_walk_right_norm', 0.50)
        self.declare_parameter('limit_bar_lateral_shift_use_euclidean_progress', False)
        self.declare_parameter('limit_bar_lateral_shift_final_yaw_gate_deg', 6.0)
        self.declare_parameter('limit_bar_lateral_shift_duck_entry_yaw_gate_deg', 2.0)
        self.declare_parameter('limit_bar_lateral_shift_duck_entry_hold_sec', 0.30)
        self.declare_parameter('limit_bar_pre_duck_global_yaw_align_enabled', False)
        self.declare_parameter('limit_bar_pre_duck_yaw_reference_mode', 'base_yaw')
        self.declare_parameter('limit_bar_pre_duck_global_yaw_deg', 180.0)
        self.declare_parameter('limit_bar_pre_duck_global_yaw_tolerance_deg', 1.0)
        self.declare_parameter('limit_bar_pre_duck_global_yaw_exit_tolerance_deg', 0.0)
        self.declare_parameter('limit_bar_pre_duck_global_yaw_hold_sec', 0.30)
        self.declare_parameter('limit_bar_pre_duck_global_yaw_timeout_sec', 4.0)
        self.declare_parameter(
            'limit_bar_pre_duck_global_yaw_fail_open_enabled',
            False,
        )
        self.declare_parameter('limit_bar_pre_duck_stationary_turn_enabled', True)
        self.declare_parameter('limit_bar_pre_duck_stationary_turn_norm', 0.08)
        self.declare_parameter('limit_bar_lateral_shift_walk_yaw_correction_enabled', False)
        self.declare_parameter('limit_bar_lateral_shift_walk_yaw_deadband_deg', 2.0)
        self.declare_parameter('limit_bar_lateral_shift_walk_yaw_gain_norm_per_rad', 0.8)
        self.declare_parameter('limit_bar_lateral_shift_walk_yaw_max_delta_norm', 0.18)
        self.declare_parameter('limit_bar_lateral_shift_duck_align_ccw_left_norm', -0.035)
        self.declare_parameter('limit_bar_lateral_shift_duck_align_ccw_right_norm', 0.060)
        self.declare_parameter('limit_bar_lateral_shift_duck_align_cw_left_norm', 0.060)
        self.declare_parameter('limit_bar_lateral_shift_duck_align_cw_right_norm', -0.035)
        self.declare_parameter('limit_bar_duck_prepare_yaw_settle_enabled', False)
        self.declare_parameter('limit_bar_duck_prepare_yaw_tolerance_deg', 1.0)
        self.declare_parameter('limit_bar_duck_prepare_yaw_hold_sec', 0.30)
        self.declare_parameter('limit_bar_duck_prepare_yaw_timeout_sec', 2.0)
        self.declare_parameter('limit_bar_duck_prepare_yaw_mode', 2)
        self.declare_parameter('limit_bar_duck_prepare_yaw_cw_left_norm', 0.35)
        self.declare_parameter('limit_bar_duck_prepare_yaw_cw_right_norm', 0.10)
        self.declare_parameter('limit_bar_duck_prepare_yaw_ccw_left_norm', 0.10)
        self.declare_parameter('limit_bar_duck_prepare_yaw_ccw_right_norm', 0.35)
        self.declare_parameter('duck_centerline_correction_enabled', False)
        self.declare_parameter('duck_centerline_lateral_gain_norm_per_m', 0.8)
        self.declare_parameter('duck_centerline_yaw_gain_norm_per_rad', 0.6)
        self.declare_parameter('duck_centerline_deadband_m', 0.03)
        self.declare_parameter('duck_centerline_yaw_deadband_deg', 2.0)
        self.declare_parameter('duck_centerline_max_delta_norm', 0.20)
        self.declare_parameter('duck_centerline_lateral_fail_m', 0.35)
        self.declare_parameter('duck_centerline_yaw_fail_deg', 25.0)
        self.declare_parameter('duck_centerline_odom_forward_axis', 'x')

        self.declare_parameter('hurdle_visual_trigger_enabled', True)
        self.declare_parameter('hurdle_tag_trigger_enabled', False)
        self.declare_parameter('hurdle_tag_target_tag_id', 31)
        self.declare_parameter('hurdle_tag_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('hurdle_tag_child_frame_id', 'hurdle_right')
        self.declare_parameter('hurdle_tag_to_center_x_m', -0.420)
        self.declare_parameter('hurdle_lateral_shift_target_distance_m', 0.40)
        self.declare_parameter('hurdle_lateral_shift_yaw_sign', -1.0)
        self.declare_parameter('hurdle_lateral_shift_distance_scale', 1.0)
        self.declare_parameter('hurdle_lateral_shift_deadband_m', 0.06)
        self.declare_parameter('hurdle_lateral_shift_arrival_tolerance_m', 0.03)
        self.declare_parameter('hurdle_final_forward_tolerance_m', 0.02)
        self.declare_parameter(
            'hurdle_lateral_shift_legacy_relock_final_forward_enabled',
            False,
        )
        self.declare_parameter(
            'hurdle_lateral_shift_legacy_relock_visual_cap_enabled',
            False,
        )
        self.declare_parameter(
            'hurdle_lateral_shift_legacy_relock_min_tag_samples',
            2,
        )
        self.declare_parameter(
            'hurdle_lateral_shift_legacy_relock_max_reduction_m',
            0.30,
        )
        self.declare_parameter(
            'hurdle_lateral_shift_legacy_slowdown_distance_m',
            0.0,
        )
        self.declare_parameter(
            'hurdle_lateral_shift_legacy_slow_left_norm',
            0.25,
        )
        self.declare_parameter(
            'hurdle_lateral_shift_legacy_slow_right_norm',
            0.25,
        )
        self.declare_parameter(
            'hurdle_lateral_shift_legacy_max_forward_overshoot_m',
            0.0,
        )
        self.declare_parameter('hurdle_lateral_shift_visual_jump_enabled', True)
        self.declare_parameter('hurdle_lateral_shift_visual_jump_distance_m', 0.65)
        self.declare_parameter('hurdle_lateral_shift_visual_jump_max_age_sec', 0.60)
        self.declare_parameter('hurdle_lateral_shift_visual_jump_yaw_gate_deg', 6.0)
        self.declare_parameter(
            'hurdle_lateral_s_curve_visual_jump_confirm_samples',
            1,
        )
        self.declare_parameter(
            'hurdle_lateral_s_curve_visual_jump_confirm_duration_sec',
            0.0,
        )
        self.declare_parameter(
            'hurdle_lateral_s_curve_visual_jump_max_remaining_m',
            0.0,
        )
        self.declare_parameter(
            'hurdle_lateral_s_curve_visual_jump_tag_max_delta_m',
            0.0,
        )
        self.declare_parameter('hurdle_tag_stale_timeout_sec', 0.90)
        self.declare_parameter('hurdle_tag_min_samples', 1)
        self.declare_parameter('hurdle_tag_max_sample_age_sec', 0.80)
        self.declare_parameter('hurdle_tf_buffer_fallback_enabled', False)
        self.declare_parameter('hurdle_tag_trigger_distance_m', 1.70)
        self.declare_parameter('hurdle_tag_odom_handoff_enabled', True)
        self.declare_parameter('hurdle_tag_odom_handoff_timeout_sec', 2.0)
        self.declare_parameter('hurdle_tag_odom_handoff_max_start_distance_m', 1.85)
        self.declare_parameter('hurdle_tag_slope_guard_enabled', True)
        self.declare_parameter('hurdle_tag_slope_guard_distance_m', 1.85)
        self.declare_parameter('hurdle_tag_yaw_reference_mode', 'tag_lateral')
        self.declare_parameter('hurdle_tag_fixed_reference_yaw_deg', 0.0)
        self.declare_parameter('hurdle_lateral_pre_align_enabled', True)
        self.declare_parameter('hurdle_lateral_pre_align_mode', 'pose_pitch')
        self.declare_parameter('hurdle_lateral_pre_align_use_visual_feedback', False)
        self.declare_parameter('hurdle_lateral_pre_align_timeout_sec', 8.0)
        self.declare_parameter('hurdle_lateral_pre_align_max_visual_error_deg', 0.0)
        self.declare_parameter('hurdle_lateral_pre_align_hold_sec', 0.10)
        self.declare_parameter('hurdle_lateral_pre_align_tolerance_deg', 2.5)
        self.declare_parameter(
            'hurdle_lateral_pre_align_stop_during_hold_enabled',
            False,
        )
        self.declare_parameter(
            'hurdle_lateral_pre_align_independent_turn_enabled',
            False,
        )
        self.declare_parameter('hurdle_fine_yaw_turn_ccw_left_norm', -0.04)
        self.declare_parameter('hurdle_fine_yaw_turn_ccw_right_norm', 0.08)
        self.declare_parameter('hurdle_fine_yaw_turn_cw_left_norm', 0.08)
        self.declare_parameter('hurdle_fine_yaw_turn_cw_right_norm', -0.04)
        self.declare_parameter('hurdle_fine_yaw_turn_min_scale', 0.50)
        self.declare_parameter('hurdle_fine_yaw_turn_full_error_deg', 10.0)
        self.declare_parameter('hurdle_lateral_pre_align_pose_control_sign', -1.0)
        self.declare_parameter('hurdle_lateral_pre_align_target_pose_pitch_deg', 0.0)
        self.declare_parameter('hurdle_stand_mode', 0)
        self.declare_parameter('hurdle_stand_left_norm', 0.0)
        self.declare_parameter('hurdle_stand_right_norm', 0.0)
        self.declare_parameter('hurdle_jump_mode', 4)
        self.declare_parameter('hurdle_jump_left_norm', 0.0)
        self.declare_parameter('hurdle_jump_right_norm', 0.0)
        self.declare_parameter('hurdle_stand_to_jump_delay_sec', 2.0)
        self.declare_parameter('hurdle_jump_to_walk_delay_sec', 5.0)
        self.declare_parameter('hurdle_jump_step_once_repeat_count', 1)
        self.declare_parameter('hurdle_jump_step_once_repeat_interval_sec', 0.08)
        self.declare_parameter('hurdle_post_align_enabled', False)
        self.declare_parameter('hurdle_post_align_use_startup_yaw', True)
        self.declare_parameter('hurdle_post_align_target_yaw_deg', 0.0)
        self.declare_parameter('hurdle_post_align_tolerance_deg', 3.0)
        self.declare_parameter('hurdle_post_align_exit_hold_sec', 0.20)
        self.declare_parameter('hurdle_post_align_timeout_sec', 3.0)
        self.declare_parameter('hurdle_post_align_timeout_action', 'advance')
        self.declare_parameter('hurdle_post_align_mode', 6)
        self.declare_parameter('hurdle_post_align_ccw_left_norm', -0.12)
        self.declare_parameter('hurdle_post_align_ccw_right_norm', 0.28)
        self.declare_parameter('hurdle_post_align_cw_left_norm', 0.28)
        self.declare_parameter('hurdle_post_align_cw_right_norm', -0.12)
        self.declare_parameter('hurdle_post_align_fine_threshold_deg', 6.0)
        self.declare_parameter('hurdle_post_align_fine_ccw_left_norm', -0.08)
        self.declare_parameter('hurdle_post_align_fine_ccw_right_norm', 0.20)
        self.declare_parameter('hurdle_post_align_fine_cw_left_norm', 0.20)
        self.declare_parameter('hurdle_post_align_fine_cw_right_norm', -0.08)
        self.declare_parameter('hurdle_post_align_hold_mode', 0)
        self.declare_parameter('hurdle_post_align_hold_left_norm', 0.0)
        self.declare_parameter('hurdle_post_align_hold_right_norm', 0.0)
        self.declare_parameter('upstairs_step_once_mode', 5)
        self.declare_parameter('upstairs_step_once_left_norm', 0.0)
        self.declare_parameter('upstairs_step_once_right_norm', 0.0)
        self.declare_parameter('upstairs_step_once_resend_after_sec', 1.0)
        self.declare_parameter('upstairs_step_once_resend_max_count', 3)
        self.declare_parameter('upstairs_lateral_shift_yaw_deg', 90.0)
        self.declare_parameter('upstairs_lateral_shift_yaw_sign', -1.0)
        self.declare_parameter('upstairs_lateral_shift_distance_scale', 1.0)
        self.declare_parameter('upstairs_lateral_shift_target_distance_m', 0.510)
        self.declare_parameter('upstairs_final_forward_tolerance_m', 0.05)
        self.declare_parameter('upstairs_final_tag_distance_tolerance_m', 0.05)
        self.declare_parameter('upstairs_final_visual_distance_m', 0.560)
        self.declare_parameter('upstairs_final_slowdown_distance_m', 0.25)
        self.declare_parameter('upstairs_final_slow_walk_left_norm', 0.30)
        self.declare_parameter('upstairs_final_slow_walk_right_norm', 0.30)
        self.declare_parameter('upstairs_final_confirm_creep_max_extra_m', 0.18)
        self.declare_parameter('upstairs_final_confirm_creep_left_norm', 0.15)
        self.declare_parameter('upstairs_final_confirm_creep_right_norm', 0.15)
        self.declare_parameter('upstairs_lateral_shift_deadband_m', 0.12)
        self.declare_parameter('upstairs_lateral_shift_arrival_tolerance_m', 0.04)
        self.declare_parameter('upstairs_lateral_shift_max_distance_m', 0.30)
        self.declare_parameter('upstairs_lateral_shift_live_check_enabled', False)
        self.declare_parameter('upstairs_lateral_shift_live_check_center_gate_m', 0.10)
        self.declare_parameter('upstairs_lateral_shift_live_check_yaw_gate_deg', 5.0)
        self.declare_parameter('upstairs_lateral_shift_live_check_max_replans', 1)
        self.declare_parameter('upstairs_final_forward_tag_yaw_correction_enabled', False)
        self.declare_parameter('upstairs_final_forward_tag_yaw_deadband_deg', 1.5)
        self.declare_parameter('upstairs_final_forward_tag_yaw_gain_norm_per_rad', 0.45)
        self.declare_parameter('upstairs_final_forward_tag_yaw_max_delta_norm', 0.10)
        self.declare_parameter('upstairs_final_forward_realign_gate_deg', 15.0)
        self.declare_parameter('upstairs_final_yaw_complete_gate_deg', 1.0)

        self.declare_parameter('slope_visual_detection_enabled', True)
        self.declare_parameter('slope_roll_detection_enabled', False)
        self.declare_parameter('slope_roll_detection_deg', 8.0)
        self.declare_parameter('slope_roll_detection_use_abs', True)
        self.declare_parameter('slope_roll_detection_hold_sec', 0.50)
        self.declare_parameter('slope_roll_detection_post_stairs_only', True)
        self.declare_parameter('slope_fallback_enabled', True)
        self.declare_parameter('slope_fallback_timeout_sec', 2.5)
        self.declare_parameter('slope_fallback_min_distance_m', 0.60)
        self.declare_parameter('slope_align_enabled', True)
        self.declare_parameter('slope_align_on_detected_branch_enabled', True)
        self.declare_parameter('slope_align_use_startup_yaw', True)
        self.declare_parameter('slope_align_target_yaw_deg', 0.0)
        self.declare_parameter('slope_align_tolerance_deg', 4.0)
        self.declare_parameter('slope_align_exit_hold_sec', 0.0)
        self.declare_parameter('slope_align_timeout_sec', 3.0)
        self.declare_parameter('slope_align_timeout_action', 'advance')
        self.declare_parameter('slope_align_mode', 6)
        self.declare_parameter('slope_align_ccw_left_norm', -0.2)
        self.declare_parameter('slope_align_ccw_right_norm', 0.70)
        self.declare_parameter('slope_align_cw_left_norm', 0.70)
        self.declare_parameter('slope_align_cw_right_norm', -0.2)
        self.declare_parameter('slope_align_fine_threshold_deg', 0.0)
        self.declare_parameter('slope_align_fine_ccw_left_norm', -0.1)
        self.declare_parameter('slope_align_fine_ccw_right_norm', 0.2)
        self.declare_parameter('slope_align_fine_cw_left_norm', 0.2)
        self.declare_parameter('slope_align_fine_cw_right_norm', -0.1)
        self.declare_parameter('slope_align_hold_mode', 0)
        self.declare_parameter('slope_align_hold_left_norm', 0.0)
        self.declare_parameter('slope_align_hold_right_norm', 0.0)
        self.declare_parameter('slope_align_reference_topic', '/slope_align_reference')
        self.declare_parameter('slope_retry_runtime_yaw_reference_enabled', False)
        self.declare_parameter(
            'slope_retry_standard_waypoints_basename',
            'obstacle_waypoints.yaml',
        )
        self.declare_parameter(
            'slope_retry_mirror_waypoints_basename',
            'obstacle_waypoints_mirror.yaml',
        )
        self.declare_parameter('slope_retry_rezero_search_only_slope', True)
        self.declare_parameter('slope_retry_rezero_rearm_delay_sec', 0.30)
        self.declare_parameter('slope_retry_route_rezero_once', True)
        self.declare_parameter('slope_bridge_full_route_retry_count', 3)
        self.declare_parameter(
            'slope_retry_pre_bridge_waypoints',
            [
                'platform_center',
                'platform_center_spin',
                'crouch_yaw_settle',
                'wall_jump_prep',
            ],
        )
        self.declare_parameter(
            'slope_retry_bridge_waypoints',
            [
                'wall_post_stand_yaw_align',
                'turn_180_spin',
                'turn_180_yaw_settle',
                'bridge_tag_yaw_lock',
                'bridge_apriltag_lateral_precheck',
                'bridge_entry',
                'bridge_exit',
                'bridge_exit_ccw_45_spin',
                'bridge_exit_cw_45_spin_mirror',
                'final_descent_approach',
                'final_descent_mid_spin',
                'final_descent_mid_spin_mirror',
            ],
        )
        self.declare_parameter(
            'slope_retry_search_waypoints',
            [
                'final_descent_second_approach',
                'final_descent_second_spin',
                'final_descent_second_spin_mirror',
                'final_descent_second_spin_extra_approach',
                'final_descent_second_spin_extra_spin',
                'final_descent_third_approach',
                'final_heading_align',
            ],
        )
        self.declare_parameter(
            'slope_route_yaw_reference_topic',
            '/slope_route_yaw_reference',
        )
        self.declare_parameter(
            'slope_bridge_retry_yaw_reference_topic',
            '/slope_bridge_retry_yaw_reference',
        )
        self.declare_parameter('downstream_route_yaw_reference_enabled', False)
        self.declare_parameter('downstream_route_yaw_reference_timeout_sec', 0.0)
        self.declare_parameter('downstream_route_yaw_hurdle_enabled', True)
        self.declare_parameter('downstream_route_yaw_upstairs_enabled', True)
        self.declare_parameter('downstream_route_yaw_limit_bar_pre_duck_enabled', True)
        self.declare_parameter('post_stairs_slope_entry_enabled', False)
        self.declare_parameter('post_stairs_slope_entry_distance_m', 0.80)
        self.declare_parameter('post_stairs_slope_entry_timeout_sec', 8.0)
        self.declare_parameter('post_stairs_slope_entry_mode', 0)
        self.declare_parameter('post_stairs_slope_entry_left_norm', 0.5)
        self.declare_parameter('post_stairs_slope_entry_right_norm', 0.5)
        self.declare_parameter('post_stairs_slope_entry_initial_yaw_align_enabled', False)
        self.declare_parameter('post_stairs_slope_entry_yaw_correction_enabled', False)
        self.declare_parameter('post_stairs_slope_entry_yaw_tolerance_deg', 1.0)
        self.declare_parameter('post_stairs_slope_entry_yaw_gain_norm_per_rad', 0.8)
        self.declare_parameter('post_stairs_slope_entry_yaw_max_delta_norm', 0.12)
        self.declare_parameter('post_stairs_slope_entry_finish_yaw_realign_threshold_deg', 0.0)
        self.declare_parameter('post_stairs_slope_search_after_entry', False)
        self.declare_parameter('post_stairs_slope_search_rearm_delay_sec', 0.0)
        self.declare_parameter('post_stairs_slope_priority_enabled', False)
        self.declare_parameter('post_stairs_slope_roll_priority_enabled', False)
        self.declare_parameter('post_stairs_slope_roll_priority_deg', 8.0)
        self.declare_parameter('post_stairs_slope_roll_priority_use_abs', True)
        self.declare_parameter('post_stairs_slope_roll_priority_hold_sec', 0.10)
        self.declare_parameter('post_stairs_slope_confirm_required', False)
        self.declare_parameter('post_stairs_slope_confirm_timeout_sec', 1.0)
        self.declare_parameter('post_stairs_slope_confirm_timeout_action', 'advance')
        self.declare_parameter('post_stairs_slope_confirm_hold_mode', 0)
        self.declare_parameter('post_stairs_slope_confirm_hold_left_norm', 0.0)
        self.declare_parameter('post_stairs_slope_confirm_hold_right_norm', 0.0)

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.pose_stale_timeout_sec = max(
            0.0, float(self.get_parameter('pose_stale_timeout_sec').value)
        )
        self.step_override_topic = str(self.get_parameter('step_override_topic').value)
        self.step_once_topic = str(self.get_parameter('step_once_topic').value)
        self.arrival_status_topic = str(self.get_parameter('arrival_status_topic').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.feedback_topic = str(self.get_parameter('feedback_topic').value)
        self.sandpit_bypass_file = str(self.get_parameter('sandpit_bypass_file').value)
        self.pole_bypass_file = str(self.get_parameter('pole_bypass_file').value)
        self.apriltag_tf_topic = str(self.get_parameter('apriltag_tf_topic').value)
        self.apriltag_tf_queue_depth = max(
            10,
            int(self.get_parameter('apriltag_tf_queue_depth').value),
        )
        self.apriltag_detections_topic = str(
            self.get_parameter('apriltag_detections_topic').value
        )

        self.limit_trigger_topic = str(self.get_parameter('limit_trigger_topic').value)
        self.limit_nearest_topic = str(self.get_parameter('limit_nearest_topic').value)
        self.hurdle_trigger_topic = str(self.get_parameter('hurdle_trigger_topic').value)
        self.hurdle_nearest_topic = str(self.get_parameter('hurdle_nearest_topic').value)
        self.pole_trigger_topic = str(self.get_parameter('pole_trigger_topic').value)
        self.pole_nearest_topic = str(self.get_parameter('pole_nearest_topic').value)
        self.slope_trigger_topic = str(self.get_parameter('slope_trigger_topic').value)
        self.slope_nearest_topic = str(self.get_parameter('slope_nearest_topic').value)
        self.upstairs_trigger_topic = str(self.get_parameter('upstairs_trigger_topic').value)
        self.upstairs_nearest_topic = str(self.get_parameter('upstairs_nearest_topic').value)

        self.stairs_active_topic = str(self.get_parameter('stairs_active_topic').value)
        self.stairs_state_topic = str(self.get_parameter('stairs_state_topic').value)
        self.slope_active_topic = str(self.get_parameter('slope_active_topic').value)
        self.slope_state_topic = str(self.get_parameter('slope_state_topic').value)
        self.slope_current_waypoint_topic = str(
            self.get_parameter('slope_current_waypoint_topic').value
        )
        self.slope_retry_restart_from_waypoint_topic = str(
            self.get_parameter('slope_retry_restart_from_waypoint_topic').value
        )
        self.pole_active_topic = str(self.get_parameter('pole_active_topic').value)
        self.pole_state_topic = str(self.get_parameter('pole_state_topic').value)
        self.pole_feedback_topic = str(self.get_parameter('pole_feedback_topic').value)

        self.start_with_stairs_manager = bool(self.get_parameter('start_with_stairs_manager').value)
        self.wait_for_start_signal = bool(self.get_parameter('wait_for_start_signal').value)
        self.start_signal_topic = str(self.get_parameter('start_signal_topic').value)
        self.start_retry_guard_sec = max(
            0.0, float(self.get_parameter('start_retry_guard_sec').value)
        )
        self.finish_signal_topic = str(self.get_parameter('finish_signal_topic').value)
        self.action_ack_topic = str(self.get_parameter('action_ack_topic').value)
        self.serial_connected_topic = str(self.get_parameter('serial_connected_topic').value)
        self.start_wait_mode = int(self.get_parameter('start_wait_mode').value)
        self.start_wait_left_norm = float(self.get_parameter('start_wait_left_norm').value)
        self.start_wait_right_norm = float(self.get_parameter('start_wait_right_norm').value)
        self.search_publish_hz = max(2.0, float(self.get_parameter('search_publish_hz').value))
        self.search_mode = int(self.get_parameter('search_mode').value)
        self.search_left_norm = float(self.get_parameter('search_left_norm').value)
        self.search_right_norm = float(self.get_parameter('search_right_norm').value)
        self.search_rearm_delay_sec = max(
            0.0, float(self.get_parameter('search_rearm_delay_sec').value)
        )
        self.search_yaw_correction_enabled = bool(
            self.get_parameter('search_yaw_correction_enabled').value
        )
        self.search_yaw_use_startup_yaw = bool(
            self.get_parameter('search_yaw_use_startup_yaw').value
        )
        self.search_yaw_target = math.radians(
            float(self.get_parameter('search_yaw_target_deg').value)
        )
        self.search_yaw_tolerance_rad = math.radians(
            max(0.0, float(self.get_parameter('search_yaw_tolerance_deg').value))
        )
        self.search_yaw_gain_per_rad = max(
            0.0, float(self.get_parameter('search_yaw_gain_per_rad').value)
        )
        self.search_yaw_max_delta_norm = max(
            0.0, float(self.get_parameter('search_yaw_max_delta_norm').value)
        )
        self.search_yaw_log_interval_sec = max(
            0.0, float(self.get_parameter('search_yaw_log_interval_sec').value)
        )
        self.detection_stale_timeout_sec = max(
            0.05, float(self.get_parameter('detection_stale_timeout_sec').value)
        )
        self.limit_trigger_min_duration_sec = max(
            0.0, float(self.get_parameter('limit_trigger_min_duration_sec').value)
        )
        self.hurdle_trigger_min_duration_sec = max(
            0.0, float(self.get_parameter('hurdle_trigger_min_duration_sec').value)
        )
        self.slope_trigger_min_duration_sec = max(
            0.0, float(self.get_parameter('slope_trigger_min_duration_sec').value)
        )
        self.pole_visual_trigger_enabled = bool(
            self.get_parameter('pole_visual_trigger_enabled').value
        )
        self.pole_trigger_min_duration_sec = max(
            0.0, float(self.get_parameter('pole_trigger_min_duration_sec').value)
        )
        self.upstairs_trigger_min_duration_sec = max(
            0.0, float(self.get_parameter('upstairs_trigger_min_duration_sec').value)
        )
        self.pole_retry_failures_before_bypass = max(
            1, int(self.get_parameter('pole_retry_failures_before_bypass').value)
        )
        self.pole_retry_search_rearm_delay_sec = max(
            0.0, float(self.get_parameter('pole_retry_search_rearm_delay_sec').value)
        )
        self.pole_bypass_enabled = bool(self.get_parameter('pole_bypass_enabled').value)
        self.pole_bypass_force_on_detection = bool(
            self.get_parameter('pole_bypass_force_on_detection').value
        )
        self.pole_bypass_pre_bypass_waypoints = max(
            0, int(self.get_parameter('pole_bypass_pre_bypass_waypoints').value)
        )
        self.pole_bypass_complete_once = bool(
            self.get_parameter('pole_bypass_complete_once').value
        )
        self.pole_bypass_search_rearm_delay_sec = max(
            0.0, float(self.get_parameter('pole_bypass_search_rearm_delay_sec').value)
        )
        self.upstairs_trigger_distance_m = max(
            0.0, float(self.get_parameter('upstairs_trigger_distance_m').value)
        )
        self.upstairs_require_hurdle_completed = bool(
            self.get_parameter('upstairs_require_hurdle_completed').value
        )
        self.upstairs_slope_guard_enabled = bool(
            self.get_parameter('upstairs_slope_guard_enabled').value
        )
        self.upstairs_slope_guard_distance_m = max(
            0.0, float(self.get_parameter('upstairs_slope_guard_distance_m').value)
        )
        self.upstairs_tag_trigger_enabled = bool(
            self.get_parameter('upstairs_tag_trigger_enabled').value
        )
        self.upstairs_tag_target_tag_id = int(
            self.get_parameter('upstairs_tag_target_tag_id').value
        )
        self.upstairs_tag_frame_id = str(
            self.get_parameter('upstairs_tag_frame_id').value
        )
        self.upstairs_tag_child_frame_id = str(
            self.get_parameter('upstairs_tag_child_frame_id').value
        )
        self.upstairs_tag_to_center_x_m = float(
            self.get_parameter('upstairs_tag_to_center_x_m').value
        )
        self.upstairs_tag_stale_timeout_sec = max(
            0.0, float(self.get_parameter('upstairs_tag_stale_timeout_sec').value)
        )
        self.upstairs_tag_min_samples = max(
            1, int(self.get_parameter('upstairs_tag_min_samples').value)
        )
        self.upstairs_tag_max_sample_age_sec = max(
            0.0, float(self.get_parameter('upstairs_tag_max_sample_age_sec').value)
        )
        self.upstairs_tag_trigger_distance_m = max(
            0.0, float(self.get_parameter('upstairs_tag_trigger_distance_m').value)
        )
        self.upstairs_tag_visual_fallback_enabled = bool(
            self.get_parameter('upstairs_tag_visual_fallback_enabled').value
        )
        self.upstairs_tag_visual_fallback_require_hurdle_completed = bool(
            self.get_parameter(
                'upstairs_tag_visual_fallback_require_hurdle_completed'
            ).value
        )
        self.upstairs_tag_visual_fallback_require_recent_tag_id = bool(
            self.get_parameter(
                'upstairs_tag_visual_fallback_require_recent_tag_id'
            ).value
        )
        self.upstairs_tag_visual_fallback_max_detection_age_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'upstairs_tag_visual_fallback_max_detection_age_sec'
                ).value
            ),
        )
        self.upstairs_tag_odom_handoff_enabled = bool(
            self.get_parameter('upstairs_tag_odom_handoff_enabled').value
        )
        self.upstairs_tag_odom_handoff_timeout_sec = max(
            0.0,
            float(self.get_parameter('upstairs_tag_odom_handoff_timeout_sec').value),
        )
        self.upstairs_tag_odom_handoff_max_start_distance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'upstairs_tag_odom_handoff_max_start_distance_m'
                ).value
            ),
        )
        self.upstairs_tag_slope_guard_enabled = bool(
            self.get_parameter('upstairs_tag_slope_guard_enabled').value
        )
        self.upstairs_tag_slope_guard_distance_m = max(
            0.0,
            float(self.get_parameter('upstairs_tag_slope_guard_distance_m').value),
        )
        self.upstairs_tag_yaw_reference_mode = (
            str(self.get_parameter('upstairs_tag_yaw_reference_mode').value)
            .strip()
            .lower()
        )
        if self.upstairs_tag_yaw_reference_mode not in (
            'tag_lateral',
            'fixed_absolute',
        ):
            raise RuntimeError(
                'upstairs_tag_yaw_reference_mode must be tag_lateral or '
                'fixed_absolute.'
            )
        self.upstairs_tag_fixed_reference_yaw = math.radians(
            float(self.get_parameter('upstairs_tag_fixed_reference_yaw_deg').value)
        )
        self.upstairs_tag_yaw_align_from_lateral_enabled = bool(
            self.get_parameter('upstairs_tag_yaw_align_from_lateral_enabled').value
        )
        self.upstairs_lateral_pre_align_enabled = bool(
            self.get_parameter('upstairs_lateral_pre_align_enabled').value
        )
        self.upstairs_lateral_pre_align_mode = (
            str(self.get_parameter('upstairs_lateral_pre_align_mode').value)
            .strip()
            .lower()
        )
        if self.upstairs_lateral_pre_align_mode not in (
            'tag_center',
            'centerline',
            'pose_roll',
            'pose_pitch',
            'pose_yaw',
            'fixed_absolute_yaw',
        ):
            raise RuntimeError(
                'upstairs_lateral_pre_align_mode must be one of: tag_center, '
                'centerline, pose_roll, pose_pitch, pose_yaw, fixed_absolute_yaw.'
            )
        self.upstairs_lateral_pre_align_use_visual_feedback = bool(
            self.get_parameter('upstairs_lateral_pre_align_use_visual_feedback').value
        )
        self.upstairs_lateral_pre_align_timeout_sec = max(
            0.0,
            float(self.get_parameter('upstairs_lateral_pre_align_timeout_sec').value),
        )
        self.upstairs_lateral_pre_align_max_visual_error_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'upstairs_lateral_pre_align_max_visual_error_deg'
                    ).value
                ),
            )
        )
        self.upstairs_lateral_pre_align_hold_sec = max(
            0.0,
            float(self.get_parameter('upstairs_lateral_pre_align_hold_sec').value),
        )
        self.upstairs_lateral_pre_align_tolerance_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('upstairs_lateral_pre_align_tolerance_deg').value),
            )
        )
        self.upstairs_lateral_pre_align_pose_control_sign = float(
            self.get_parameter('upstairs_lateral_pre_align_pose_control_sign').value
        )
        self.upstairs_lateral_pre_align_target_pose_pitch_rad = math.radians(
            float(
                self.get_parameter(
                    'upstairs_lateral_pre_align_target_pose_pitch_deg'
                ).value
            )
        )
        self.slope_wait_for_upstairs_nearest_sec = max(
            0.0, float(self.get_parameter('slope_wait_for_upstairs_nearest_sec').value)
        )
        self.done_hold_step_override_enabled = bool(
            self.get_parameter('done_hold_step_override_enabled').value
        )
        self.done_hold_mode = int(self.get_parameter('done_hold_mode').value)
        self.done_hold_left_norm = float(self.get_parameter('done_hold_left_norm').value)
        self.done_hold_right_norm = float(self.get_parameter('done_hold_right_norm').value)
        self.upstairs_done_retry_guard_enabled = bool(
            self.get_parameter('upstairs_done_retry_guard_enabled').value
        )
        self.upstairs_done_retry_guard_duration_sec = max(
            0.0,
            float(self.get_parameter('upstairs_done_retry_guard_duration_sec').value),
        )
        self.upstairs_done_retry_guard_mode = int(
            self.get_parameter('upstairs_done_retry_guard_mode').value
        )
        self.upstairs_done_retry_guard_left_norm = float(
            self.get_parameter('upstairs_done_retry_guard_left_norm').value
        )
        self.upstairs_done_retry_guard_right_norm = float(
            self.get_parameter('upstairs_done_retry_guard_right_norm').value
        )
        self.failed_hold_step_override_enabled = bool(
            self.get_parameter('failed_hold_step_override_enabled').value
        )
        self.failed_stop_nav_executor_enabled = bool(
            self.get_parameter('failed_stop_nav_executor_enabled').value
        )
        self.failed_hold_mode = int(self.get_parameter('failed_hold_mode').value)
        self.failed_hold_left_norm = float(
            self.get_parameter('failed_hold_left_norm').value
        )
        self.failed_hold_right_norm = float(
            self.get_parameter('failed_hold_right_norm').value
        )
        self.pole_max_lateral_abs_m = max(
            0.0, float(self.get_parameter('pole_max_lateral_abs_m').value)
        )
        self.pole_after_slope_search_delay_sec = max(
            0.0, float(self.get_parameter('pole_after_slope_search_delay_sec').value)
        )
        self.pole_near_preempts_limit_bar = bool(
            self.get_parameter('pole_near_preempts_limit_bar').value
        )
        self.pole_near_preempt_limit_bar_distance_m = max(
            0.0,
            float(self.get_parameter('pole_near_preempt_limit_bar_distance_m').value),
        )
        self.pole_apriltag_trigger_enabled = bool(
            self.get_parameter('pole_apriltag_trigger_enabled').value
        )
        self.pole_apriltag_target_tag_id = int(
            self.get_parameter('pole_apriltag_target_tag_id').value
        )
        self.pole_apriltag_frame_id = str(
            self.get_parameter('pole_apriltag_frame_id').value
        )
        self.pole_apriltag_child_frame_id = str(
            self.get_parameter('pole_apriltag_child_frame_id').value
        )
        self.pole_apriltag_stale_timeout_sec = max(
            0.05,
            float(self.get_parameter('pole_apriltag_stale_timeout_sec').value),
        )
        self.pole_apriltag_detection_latch_sec = max(
            0.0,
            float(self.get_parameter('pole_apriltag_detection_latch_sec').value),
        )
        self.pole_apriltag_require_tf = bool(
            self.get_parameter('pole_apriltag_require_tf').value
        )
        self.pole_apriltag_min_duration_sec = max(
            0.0,
            float(self.get_parameter('pole_apriltag_min_duration_sec').value),
        )
        self.pole_apriltag_distance_axis = (
            str(self.get_parameter('pole_apriltag_distance_axis').value).strip().lower()
        )
        if self.pole_apriltag_distance_axis not in ('x', 'y', 'z', 'euclidean'):
            raise RuntimeError(
                'pole_apriltag_distance_axis must be x, y, z, or euclidean.'
            )
        self.pole_apriltag_trigger_distance_m = max(
            0.0,
            float(self.get_parameter('pole_apriltag_trigger_distance_m').value),
        )
        self.pole_apriltag_odom_handoff_enabled = bool(
            self.get_parameter('pole_apriltag_odom_handoff_enabled').value
        )
        self.pole_apriltag_odom_handoff_timeout_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'pole_apriltag_odom_handoff_timeout_sec'
                ).value
            ),
        )
        self.pole_apriltag_odom_handoff_max_start_distance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'pole_apriltag_odom_handoff_max_start_distance_m'
                ).value
            ),
        )
        self.limit_bar_nearest_preempts_slope = bool(
            self.get_parameter('limit_bar_nearest_preempts_slope').value
        )
        self.limit_bar_nearest_preempt_distance_m = max(
            0.0,
            float(self.get_parameter('limit_bar_nearest_preempt_distance_m').value),
        )
        self.limit_bar_visual_trigger_enabled = bool(
            self.get_parameter('limit_bar_visual_trigger_enabled').value
        )
        self.limit_bar_tag_trigger_enabled = bool(
            self.get_parameter('limit_bar_tag_trigger_enabled').value
        )
        self.limit_bar_tag_target_tag_id = int(
            self.get_parameter('limit_bar_tag_target_tag_id').value
        )

        self.duck_prepare_mode = int(self.get_parameter('duck_prepare_mode').value)
        self.duck_prepare_left_norm = float(self.get_parameter('duck_prepare_left_norm').value)
        self.duck_prepare_right_norm = float(self.get_parameter('duck_prepare_right_norm').value)
        self.duck_prepare_stop_buffer_sec = max(
            0.0, float(self.get_parameter('duck_prepare_stop_buffer_sec').value)
        )
        self.duck_prepare_delay_sec = max(
            0.0, float(self.get_parameter('duck_prepare_delay_sec').value)
        )
        self.duck_walk_mode = int(self.get_parameter('duck_walk_mode').value)
        self.duck_walk_left_norm = float(self.get_parameter('duck_walk_left_norm').value)
        self.duck_walk_right_norm = float(self.get_parameter('duck_walk_right_norm').value)
        self.duck_resume_mode = int(self.get_parameter('duck_resume_mode').value)
        self.duck_resume_left_norm = float(self.get_parameter('duck_resume_left_norm').value)
        self.duck_resume_right_norm = float(self.get_parameter('duck_resume_right_norm').value)
        self.duck_resume_delay_sec = max(
            0.0, float(self.get_parameter('duck_resume_delay_sec').value)
        )
        self.duck_travel_distance_m = max(
            0.0, float(self.get_parameter('duck_travel_distance_m').value)
        )
        self.duck_travel_projection_enabled = bool(
            self.get_parameter('duck_travel_projection_enabled').value
        )
        self.duck_travel_timeout_sec = max(
            0.0, float(self.get_parameter('duck_travel_timeout_sec').value)
        )
        self.duck_progress_log_interval_sec = max(
            0.0, float(self.get_parameter('duck_progress_log_interval_sec').value)
        )
        self.duck_travel_delta_x_m = float(self.get_parameter('duck_travel_delta_x_m').value)
        self.duck_travel_delta_y_m = float(self.get_parameter('duck_travel_delta_y_m').value)
        self.duck_travel_max_x_error_m = max(
            0.0, float(self.get_parameter('duck_travel_max_x_error_m').value)
        )
        if self.duck_travel_distance_m <= 0.0:
            self.duck_travel_distance_m = math.hypot(
                self.duck_travel_delta_x_m,
                self.duck_travel_delta_y_m,
            )
        self.limit_bar_post_resume_yaw_align_enabled = bool(
            self.get_parameter('limit_bar_post_resume_yaw_align_enabled').value
        )
        self.limit_bar_post_resume_yaw_tolerance_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('limit_bar_post_resume_yaw_tolerance_deg').value),
            )
        )
        self.limit_bar_post_resume_yaw_hold_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_post_resume_yaw_hold_sec').value),
        )
        self.limit_bar_post_resume_yaw_timeout_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_post_resume_yaw_timeout_sec').value),
        )
        self.duck_pre_align_enabled = bool(
            self.get_parameter('duck_pre_align_enabled').value
        )
        self.duck_visual_yaw_align_enabled = bool(
            self.get_parameter('duck_visual_yaw_align_enabled').value
        )
        self.duck_visual_yaw_lateral_sign = float(
            self.get_parameter('duck_visual_yaw_lateral_sign').value
        )
        self.duck_visual_yaw_max_offset_rad = math.radians(
            max(0.0, float(self.get_parameter('duck_visual_yaw_max_offset_deg').value))
        )
        self.duck_visual_yaw_stale_timeout_sec = max(
            0.0, float(self.get_parameter('duck_visual_yaw_stale_timeout_sec').value)
        )
        self.duck_visual_yaw_min_distance_m = max(
            1e-3, float(self.get_parameter('duck_visual_yaw_min_distance_m').value)
        )
        self.duck_yaw_correction_enabled = bool(
            self.get_parameter('duck_yaw_correction_enabled').value
        )
        self.duck_yaw_correction_tolerance_rad = math.radians(
            max(0.0, float(self.get_parameter('duck_yaw_correction_tolerance_deg').value))
        )
        self.duck_yaw_correction_gain_per_rad = max(
            0.0, float(self.get_parameter('duck_yaw_correction_gain_per_rad').value)
        )
        self.duck_yaw_correction_max_delta_norm = max(
            0.0, float(self.get_parameter('duck_yaw_correction_max_delta_norm').value)
        )
        self.duck_yaw_correction_force_max_delta = bool(
            self.get_parameter('duck_yaw_correction_force_max_delta').value
        )
        self.duck_yaw_correction_sign = float(
            self.get_parameter('duck_yaw_correction_sign').value
        )
        self.limit_bar_tag_align_enabled = bool(
            self.get_parameter('limit_bar_tag_align_enabled').value
        )
        self.limit_bar_tag_frame_id = str(
            self.get_parameter('limit_bar_tag_frame_id').value
        )
        self.limit_bar_tag_child_frame_id = str(
            self.get_parameter('limit_bar_tag_child_frame_id').value
        )
        self.limit_bar_tag_to_center_x_m = float(
            self.get_parameter('limit_bar_tag_to_center_x_m').value
        )
        self.limit_bar_tag_stale_timeout_sec = max(
            0.0, float(self.get_parameter('limit_bar_tag_stale_timeout_sec').value)
        )
        self.limit_bar_tag_min_samples = max(
            1, int(self.get_parameter('limit_bar_tag_min_samples').value)
        )
        self.limit_bar_tag_max_sample_age_sec = max(
            0.0, float(self.get_parameter('limit_bar_tag_max_sample_age_sec').value)
        )
        self.limit_bar_tag_geometry_lock_enabled = bool(
            self.get_parameter('limit_bar_tag_geometry_lock_enabled').value
        )
        self.limit_bar_tag_geometry_lock_sample_count = max(
            1,
            int(
                self.get_parameter(
                    'limit_bar_tag_geometry_lock_sample_count'
                ).value
            ),
        )
        self.limit_bar_tag_trigger_distance_m = max(
            0.0, float(self.get_parameter('limit_bar_tag_trigger_distance_m').value)
        )
        self.limit_bar_tag_odom_handoff_enabled = bool(
            self.get_parameter('limit_bar_tag_odom_handoff_enabled').value
        )
        self.limit_bar_tag_odom_handoff_timeout_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_tag_odom_handoff_timeout_sec').value),
        )
        self.limit_bar_tag_odom_handoff_max_start_distance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_tag_odom_handoff_max_start_distance_m'
                ).value
            ),
        )
        self.limit_bar_tag_presearch_handoff_enabled = bool(
            self.get_parameter('limit_bar_tag_presearch_handoff_enabled').value
        )
        self.limit_bar_tag_presearch_handoff_max_abs_roll_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'limit_bar_tag_presearch_handoff_max_abs_roll_deg'
                    ).value
                ),
            )
        )
        self.limit_bar_tag_presearch_handoff_max_abs_pitch_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'limit_bar_tag_presearch_handoff_max_abs_pitch_deg'
                    ).value
                ),
            )
        )
        self.limit_bar_tag_slope_guard_enabled = bool(
            self.get_parameter('limit_bar_tag_slope_guard_enabled').value
        )
        self.limit_bar_tag_slope_guard_distance_m = max(
            0.0,
            float(self.get_parameter('limit_bar_tag_slope_guard_distance_m').value),
        )
        self.limit_bar_tag_align_tolerance_rad = math.radians(
            max(0.0, float(self.get_parameter('limit_bar_tag_align_tolerance_deg').value))
        )
        self.limit_bar_tag_align_timeout_sec = max(
            0.0, float(self.get_parameter('limit_bar_tag_align_timeout_sec').value)
        )
        self.limit_bar_tag_align_yaw_offset_sign = float(
            self.get_parameter('limit_bar_tag_align_yaw_offset_sign').value
        )
        self.limit_bar_tag_yaw_reference_mode = (
            str(self.get_parameter('limit_bar_tag_yaw_reference_mode').value)
            .strip()
            .lower()
        )
        if self.limit_bar_tag_yaw_reference_mode not in (
            'tag_lateral',
            'fixed_absolute',
        ):
            raise RuntimeError(
                'limit_bar_tag_yaw_reference_mode must be '
                'tag_lateral or fixed_absolute.'
            )
        self.limit_bar_tag_fixed_reference_yaw = math.radians(
            float(
                self.get_parameter(
                    'limit_bar_tag_fixed_reference_yaw_deg'
                ).value
            )
        )
        self.limit_bar_tag_align_mode = int(
            self.get_parameter('limit_bar_tag_align_mode').value
        )
        self.limit_bar_tag_align_ccw_left_norm = float(
            self.get_parameter('limit_bar_tag_align_ccw_left_norm').value
        )
        self.limit_bar_tag_align_ccw_right_norm = float(
            self.get_parameter('limit_bar_tag_align_ccw_right_norm').value
        )
        self.limit_bar_tag_align_cw_left_norm = float(
            self.get_parameter('limit_bar_tag_align_cw_left_norm').value
        )
        self.limit_bar_tag_align_cw_right_norm = float(
            self.get_parameter('limit_bar_tag_align_cw_right_norm').value
        )
        self.limit_bar_tag_align_turn_scale_enabled = bool(
            self.get_parameter('limit_bar_tag_align_turn_scale_enabled').value
        )
        self.limit_bar_tag_align_turn_min_scale = max(
            0.0,
            min(1.0, float(self.get_parameter('limit_bar_tag_align_turn_min_scale').value)),
        )
        self.limit_bar_tag_align_yaw_gain_per_rad = max(
            0.0,
            float(self.get_parameter('limit_bar_tag_align_yaw_gain_per_rad').value),
        )
        self.limit_bar_tag_align_min_abs_angular_z = max(
            0.0,
            float(self.get_parameter('limit_bar_tag_align_min_abs_angular_z').value),
        )
        self.limit_bar_tag_align_max_abs_angular_z = max(
            1e-6,
            float(self.get_parameter('limit_bar_tag_align_max_abs_angular_z').value),
        )
        self.limit_bar_tag_align_min_abs_angular_z = min(
            self.limit_bar_tag_align_min_abs_angular_z,
            self.limit_bar_tag_align_max_abs_angular_z,
        )
        self.limit_bar_lateral_precision_turn_enabled = bool(
            self.get_parameter('limit_bar_lateral_precision_turn_enabled').value
        )
        self.limit_bar_lateral_turn_far_error_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('limit_bar_lateral_turn_far_error_deg').value),
            )
        )
        self.limit_bar_lateral_turn_far_scale = max(
            1.0,
            float(self.get_parameter('limit_bar_lateral_turn_far_scale').value),
        )
        self.limit_bar_lateral_turn_yaw_damping_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_lateral_turn_yaw_damping_sec').value),
        )
        self.limit_bar_lateral_turn_brake_yaw_rate_rad_s = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'limit_bar_lateral_turn_brake_yaw_rate_deg_s'
                    ).value
                ),
            )
        )
        self.limit_bar_lateral_turn_complete_yaw_rate_rad_s = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'limit_bar_lateral_turn_complete_yaw_rate_deg_s'
                    ).value
                ),
            )
        )
        self.limit_bar_lateral_turn_yaw_rate_filter_alpha = max(
            0.0,
            min(
                1.0,
                float(
                    self.get_parameter(
                        'limit_bar_lateral_turn_yaw_rate_filter_alpha'
                    ).value
                ),
            ),
        )
        self.limit_bar_lateral_turn_yaw_rate_window_sec = max(
            1e-3,
            float(
                self.get_parameter(
                    'limit_bar_lateral_turn_yaw_rate_window_sec'
                ).value
            ),
        )
        self.limit_bar_lateral_turn_yaw_rate_min_span_sec = max(
            1e-3,
            min(
                self.limit_bar_lateral_turn_yaw_rate_window_sec,
                float(
                    self.get_parameter(
                        'limit_bar_lateral_turn_yaw_rate_min_span_sec'
                    ).value
                ),
            ),
        )
        self.limit_bar_lateral_turn_yaw_rate_max_abs_rad_s = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'limit_bar_lateral_turn_yaw_rate_max_abs_deg_s'
                    ).value
                ),
            )
        )
        self.limit_bar_lateral_turn_hold_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_lateral_turn_hold_sec').value),
        )
        self.limit_bar_lateral_turn_reverse_cooldown_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_turn_reverse_cooldown_sec'
                ).value
            ),
        )
        self.limit_bar_lateral_turn_timeout_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_lateral_turn_timeout_sec').value),
        )
        self.limit_bar_lateral_turn_timeout_settle_grace_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_turn_timeout_settle_grace_sec'
                ).value
            ),
        )
        self.limit_bar_lateral_shift_timeout_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_lateral_shift_timeout_sec').value),
        )
        self.limit_bar_lateral_final_forward_timeout_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_final_forward_timeout_sec'
                ).value
            ),
        )
        self.lateral_s_curve_shadow_enabled = bool(
            self.get_parameter('lateral_s_curve_shadow_enabled').value
        )
        self.hurdle_lateral_s_curve_shadow_enabled = bool(
            self.get_parameter('hurdle_lateral_s_curve_shadow_enabled').value
        )
        self.limit_bar_lateral_s_curve_shadow_enabled = bool(
            self.get_parameter('limit_bar_lateral_s_curve_shadow_enabled').value
        )
        self.upstairs_lateral_s_curve_shadow_enabled = bool(
            self.get_parameter('upstairs_lateral_s_curve_shadow_enabled').value
        )
        self.lateral_s_curve_shadow_min_forward_m = max(
            0.0,
            float(self.get_parameter('lateral_s_curve_shadow_min_forward_m').value),
        )
        self.lateral_s_curve_shadow_max_abs_lateral_m = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_shadow_max_abs_lateral_m'
                ).value
            ),
        )
        self.lateral_s_curve_shadow_max_heading_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'lateral_s_curve_shadow_max_heading_deg'
                    ).value
                ),
            )
        )
        self.limit_bar_lateral_s_curve_max_heading_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'limit_bar_lateral_s_curve_max_heading_deg'
                    ).value
                ),
            )
        )
        self.lateral_s_curve_shadow_max_curvature_m_inv = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_shadow_max_curvature_m_inv'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_max_curvature_m_inv = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_max_curvature_m_inv'
                ).value
            ),
        )
        self.lateral_s_curve_control_enabled = bool(
            self.get_parameter('lateral_s_curve_control_enabled').value
        )
        self.hurdle_lateral_s_curve_control_enabled = bool(
            self.get_parameter('hurdle_lateral_s_curve_control_enabled').value
        )
        self.limit_bar_lateral_s_curve_control_enabled = bool(
            self.get_parameter('limit_bar_lateral_s_curve_control_enabled').value
        )
        self.upstairs_lateral_s_curve_control_enabled = bool(
            self.get_parameter('upstairs_lateral_s_curve_control_enabled').value
        )
        self.limit_bar_lateral_s_curve_control_cruise_left_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_cruise_left_norm'
                ).value
            )
        )
        self.limit_bar_lateral_s_curve_control_cruise_right_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_cruise_right_norm'
                ).value
            )
        )
        self.limit_bar_lateral_s_curve_control_slow_left_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_slow_left_norm'
                ).value
            )
        )
        self.limit_bar_lateral_s_curve_control_slow_right_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_slow_right_norm'
                ).value
            )
        )
        self.limit_bar_lateral_s_curve_control_slowdown_distance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_slowdown_distance_m'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_control_timeout_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_timeout_sec'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_fail_open_enabled = bool(
            self.get_parameter(
                'limit_bar_lateral_s_curve_fail_open_enabled'
            ).value
        )
        self.limit_bar_lateral_s_curve_control_max_forward_overshoot_m = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_max_forward_overshoot_m'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_control_max_abs_cross_track_m = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_max_abs_cross_track_m'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_control_forward_tolerance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_forward_tolerance_m'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_control_lateral_tolerance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_lateral_tolerance_m'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_control_arrival_settle_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_arrival_settle_sec'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_control_pre_stop_distance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_pre_stop_distance_m'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_control_pre_stop_hold_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_pre_stop_hold_sec'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_control_final_abs_lateral_tolerance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_final_abs_lateral_tolerance_m'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_control_final_max_lateral_drift_m = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_final_max_lateral_drift_m'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_control_final_lateral_violation_hold_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_s_curve_control_final_lateral_violation_hold_sec'
                ).value
            ),
        )
        self.limit_bar_lateral_s_curve_control_final_align_norm = min(
            1.0,
            max(
                0.0,
                float(
                    self.get_parameter(
                        'limit_bar_lateral_s_curve_control_final_align_norm'
                    ).value
                ),
            ),
        )
        self.limit_bar_stop_nav_executor_on_entry_enabled = bool(
            self.get_parameter(
                'limit_bar_stop_nav_executor_on_entry_enabled'
            ).value
        )
        self.upstairs_lateral_s_curve_control_cruise_left_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'upstairs_lateral_s_curve_control_cruise_left_norm'
                ).value
            )
        )
        self.upstairs_lateral_s_curve_control_cruise_right_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'upstairs_lateral_s_curve_control_cruise_right_norm'
                ).value
            )
        )
        self.upstairs_lateral_s_curve_control_slow_left_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'upstairs_lateral_s_curve_control_slow_left_norm'
                ).value
            )
        )
        self.upstairs_lateral_s_curve_control_slow_right_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'upstairs_lateral_s_curve_control_slow_right_norm'
                ).value
            )
        )
        self.upstairs_lateral_s_curve_control_slowdown_distance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'upstairs_lateral_s_curve_control_slowdown_distance_m'
                ).value
            ),
        )
        self.upstairs_lateral_s_curve_control_timeout_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'upstairs_lateral_s_curve_control_timeout_sec'
                ).value
            ),
        )
        self.upstairs_lateral_s_curve_control_max_forward_overshoot_m = max(
            0.0,
            float(
                self.get_parameter(
                    'upstairs_lateral_s_curve_control_max_forward_overshoot_m'
                ).value
            ),
        )
        self.upstairs_lateral_s_curve_control_distance_max_remaining_m = max(
            0.0,
            float(
                self.get_parameter(
                    'upstairs_lateral_s_curve_control_distance_max_remaining_m'
                ).value
            ),
        )
        self.upstairs_lateral_s_curve_control_precreep_yaw_gate_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'upstairs_lateral_s_curve_control_precreep_yaw_gate_deg'
                    ).value
                ),
            )
        )
        self.upstairs_lateral_s_curve_control_final_align_norm = min(
            1.0,
            max(
                0.0,
                float(
                    self.get_parameter(
                        'upstairs_lateral_s_curve_control_final_align_norm'
                    ).value
                ),
            ),
        )
        self.hurdle_lateral_s_curve_adaptive_clamp_enabled = bool(
            self.get_parameter(
                'hurdle_lateral_s_curve_adaptive_clamp_enabled'
            ).value
        )
        self.hurdle_lateral_s_curve_adaptive_clamp_min_scale = min(
            1.0,
            max(
                0.0,
                float(
                    self.get_parameter(
                        'hurdle_lateral_s_curve_adaptive_clamp_min_scale'
                    ).value
                ),
            ),
        )
        self.lateral_s_curve_control_cruise_left_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'lateral_s_curve_control_cruise_left_norm'
                ).value
            )
        )
        self.lateral_s_curve_control_cruise_right_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'lateral_s_curve_control_cruise_right_norm'
                ).value
            )
        )
        self.lateral_s_curve_control_slow_left_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'lateral_s_curve_control_slow_left_norm'
                ).value
            )
        )
        self.lateral_s_curve_control_slow_right_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'lateral_s_curve_control_slow_right_norm'
                ).value
            )
        )
        self.lateral_s_curve_control_slowdown_distance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_slowdown_distance_m'
                ).value
            ),
        )
        self.lateral_s_curve_control_lookahead_m = max(
            0.0,
            float(self.get_parameter('lateral_s_curve_control_lookahead_m').value),
        )
        self.lateral_s_curve_control_heading_gain_norm_per_rad = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_heading_gain_norm_per_rad'
                ).value
            ),
        )
        self.lateral_s_curve_control_cross_track_gain_rad_per_m = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_cross_track_gain_rad_per_m'
                ).value
            ),
        )
        self.lateral_s_curve_control_cross_track_max_heading_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'lateral_s_curve_control_cross_track_max_heading_deg'
                    ).value
                ),
            )
        )
        self.lateral_s_curve_control_max_delta_norm = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_max_delta_norm'
                ).value
            ),
        )
        self.lateral_s_curve_control_walk_yaw_gate_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'lateral_s_curve_control_walk_yaw_gate_deg'
                    ).value
                ),
            )
        )
        self.lateral_s_curve_control_forward_tolerance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_forward_tolerance_m'
                ).value
            ),
        )
        self.lateral_s_curve_control_lateral_tolerance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_lateral_tolerance_m'
                ).value
            ),
        )
        self.lateral_s_curve_control_yaw_tolerance_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'lateral_s_curve_control_yaw_tolerance_deg'
                    ).value
                ),
            )
        )
        self.lateral_s_curve_control_arrival_settle_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_arrival_settle_sec'
                ).value
            ),
        )
        self.lateral_s_curve_control_completion_hold_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_completion_hold_sec'
                ).value
            ),
        )
        self.lateral_s_curve_control_completion_max_retreat_m = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_completion_max_retreat_m'
                ).value
            ),
        )
        self.lateral_s_curve_control_completion_max_lateral_drift_m = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_completion_max_lateral_drift_m'
                ).value
            ),
        )
        self.lateral_s_curve_control_completion_recovery_enabled = bool(
            self.get_parameter(
                'lateral_s_curve_control_completion_recovery_enabled'
            ).value
        )
        self.lateral_s_curve_control_completion_recovery_left_norm = (
            self._clamp_norm(
                float(
                    self.get_parameter(
                        'lateral_s_curve_control_completion_recovery_left_norm'
                    ).value
                )
            )
        )
        self.lateral_s_curve_control_completion_recovery_right_norm = (
            self._clamp_norm(
                float(
                    self.get_parameter(
                        'lateral_s_curve_control_completion_recovery_right_norm'
                    ).value
                )
            )
        )
        self.lateral_s_curve_control_completion_recovery_yaw_gain_norm_per_rad = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_completion_recovery_yaw_gain_norm_per_rad'
                ).value
            ),
        )
        self.lateral_s_curve_control_completion_recovery_yaw_max_delta_norm = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_completion_recovery_yaw_max_delta_norm'
                ).value
            ),
        )
        self.lateral_s_curve_control_completion_recovery_forward_tolerance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_completion_recovery_forward_tolerance_m'
                ).value
            ),
        )
        self.lateral_s_curve_control_completion_recovery_yaw_tolerance_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'lateral_s_curve_control_completion_recovery_yaw_tolerance_deg'
                    ).value
                ),
            )
        )
        self.lateral_s_curve_control_completion_recovery_max_extra_forward_m = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_completion_recovery_max_extra_forward_m'
                ).value
            ),
        )
        self.lateral_s_curve_control_completion_recovery_max_abs_lateral_m = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_completion_recovery_max_abs_lateral_m'
                ).value
            ),
        )
        self.hurdle_lateral_s_curve_final_align_independent_turn_enabled = bool(
            self.get_parameter(
                'hurdle_lateral_s_curve_final_align_independent_turn_enabled'
            ).value
        )
        self.hurdle_lateral_s_curve_visual_completion_ignore_plan_overshoot_enabled = bool(
            self.get_parameter(
                'hurdle_lateral_s_curve_visual_completion_ignore_plan_overshoot_enabled'
            ).value
        )
        self.hurdle_lateral_s_curve_visual_completion_max_plan_overshoot_m = max(
            0.0,
            float(
                self.get_parameter(
                    'hurdle_lateral_s_curve_visual_completion_max_plan_overshoot_m'
                ).value
            ),
        )
        self.lateral_s_curve_control_timeout_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_timeout_sec'
                ).value
            ),
        )
        self.lateral_s_curve_control_max_forward_overshoot_m = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_max_forward_overshoot_m'
                ).value
            ),
        )
        self.lateral_s_curve_control_max_abs_cross_track_m = max(
            0.0,
            float(
                self.get_parameter(
                    'lateral_s_curve_control_max_abs_cross_track_m'
                ).value
            ),
        )
        self.limit_bar_lateral_pre_align_enabled = bool(
            self.get_parameter('limit_bar_lateral_pre_align_enabled').value
        )
        self.limit_bar_lateral_pre_align_mode = (
            str(self.get_parameter('limit_bar_lateral_pre_align_mode').value)
            .strip()
            .lower()
        )
        if self.limit_bar_lateral_pre_align_mode not in (
            'tag_center',
            'centerline',
            'pose_roll',
            'pose_pitch',
            'pose_yaw',
            'fixed_absolute_yaw',
        ):
            raise RuntimeError(
                'limit_bar_lateral_pre_align_mode must be one of: tag_center, '
                'centerline, pose_roll, pose_pitch, pose_yaw, fixed_absolute_yaw.'
            )
        self.limit_bar_lateral_pre_align_use_visual_feedback = bool(
            self.get_parameter('limit_bar_lateral_pre_align_use_visual_feedback').value
        )
        self.limit_bar_lateral_pre_align_timeout_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_lateral_pre_align_timeout_sec').value),
        )
        self.limit_bar_lateral_pre_align_max_visual_error_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'limit_bar_lateral_pre_align_max_visual_error_deg'
                    ).value
                ),
            )
        )
        self.limit_bar_lateral_pre_align_hold_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_lateral_pre_align_hold_sec').value),
        )
        self.limit_bar_lateral_pre_align_tolerance_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('limit_bar_lateral_pre_align_tolerance_deg').value),
            )
        )
        self.limit_bar_lateral_pre_align_stop_during_hold_enabled = bool(
            self.get_parameter(
                'limit_bar_lateral_pre_align_stop_during_hold_enabled'
            ).value
        )
        self.limit_bar_lateral_pre_align_pose_control_sign = float(
            self.get_parameter('limit_bar_lateral_pre_align_pose_control_sign').value
        )
        self.limit_bar_lateral_pre_align_target_pose_pitch_rad = math.radians(
            float(
                self.get_parameter(
                    'limit_bar_lateral_pre_align_target_pose_pitch_deg'
                ).value
            )
        )
        self.limit_bar_lateral_shift_enabled = bool(
            self.get_parameter('limit_bar_lateral_shift_enabled').value
        )
        self.limit_bar_lateral_shift_yaw_rad = math.radians(
            max(0.0, abs(float(self.get_parameter('limit_bar_lateral_shift_yaw_deg').value)))
        )
        self.limit_bar_lateral_shift_yaw_sign = (
            -1.0
            if float(self.get_parameter('limit_bar_lateral_shift_yaw_sign').value) < 0.0
            else 1.0
        )
        self.limit_bar_lateral_shift_deadband_m = max(
            0.0, float(self.get_parameter('limit_bar_lateral_shift_deadband_m').value)
        )
        self.limit_bar_lateral_shift_arrival_tolerance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_shift_arrival_tolerance_m'
                ).value
            ),
        )
        self.limit_bar_lateral_shift_skip_below_m = max(
            0.0, float(self.get_parameter('limit_bar_lateral_shift_skip_below_m').value)
        )
        self.limit_bar_lateral_shift_distance_scale = max(
            0.0,
            min(1.5, float(self.get_parameter('limit_bar_lateral_shift_distance_scale').value)),
        )
        self.limit_bar_lateral_shift_max_distance_m = max(
            0.0, float(self.get_parameter('limit_bar_lateral_shift_max_distance_m').value)
        )
        self.limit_bar_lateral_shift_target_distance_m = max(
            0.0,
            float(self.get_parameter('limit_bar_lateral_shift_target_distance_m').value),
        )
        self.limit_bar_lateral_shift_walk_mode = int(
            self.get_parameter('limit_bar_lateral_shift_walk_mode').value
        )
        self.limit_bar_lateral_shift_walk_left_norm = float(
            self.get_parameter('limit_bar_lateral_shift_walk_left_norm').value
        )
        self.limit_bar_lateral_shift_walk_right_norm = float(
            self.get_parameter('limit_bar_lateral_shift_walk_right_norm').value
        )
        self.limit_bar_lateral_shift_use_euclidean_progress = bool(
            self.get_parameter('limit_bar_lateral_shift_use_euclidean_progress').value
        )
        self.limit_bar_lateral_shift_final_yaw_gate_rad = math.radians(
            max(0.0, float(self.get_parameter('limit_bar_lateral_shift_final_yaw_gate_deg').value))
        )
        self.limit_bar_lateral_shift_duck_entry_yaw_gate_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('limit_bar_lateral_shift_duck_entry_yaw_gate_deg').value),
            )
        )
        self.limit_bar_lateral_shift_duck_entry_hold_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_lateral_shift_duck_entry_hold_sec').value),
        )
        self.limit_bar_pre_duck_global_yaw_align_enabled = bool(
            self.get_parameter('limit_bar_pre_duck_global_yaw_align_enabled').value
        )
        self.limit_bar_pre_duck_yaw_reference_mode = (
            str(
                self.get_parameter(
                    'limit_bar_pre_duck_yaw_reference_mode'
                ).value
            )
            .strip()
            .lower()
        )
        if self.limit_bar_pre_duck_yaw_reference_mode not in (
            'base_yaw',
            'route_zero_plus_offset',
        ):
            raise RuntimeError(
                'limit_bar_pre_duck_yaw_reference_mode must be '
                'base_yaw or route_zero_plus_offset.'
            )
        self.limit_bar_pre_duck_global_yaw_rad = self._normalize_angle(
            math.radians(
                float(self.get_parameter('limit_bar_pre_duck_global_yaw_deg').value)
            )
        )
        self.limit_bar_pre_duck_global_yaw_tolerance_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'limit_bar_pre_duck_global_yaw_tolerance_deg'
                    ).value
                ),
            )
        )
        pre_duck_yaw_exit_tolerance_deg = float(
            self.get_parameter(
                'limit_bar_pre_duck_global_yaw_exit_tolerance_deg'
            ).value
        )
        self.limit_bar_pre_duck_global_yaw_exit_tolerance_rad = max(
            self.limit_bar_pre_duck_global_yaw_tolerance_rad,
            math.radians(max(0.0, pre_duck_yaw_exit_tolerance_deg)),
        )
        self.limit_bar_pre_duck_global_yaw_hold_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_pre_duck_global_yaw_hold_sec').value),
        )
        self.limit_bar_pre_duck_global_yaw_timeout_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_pre_duck_global_yaw_timeout_sec').value),
        )
        self.limit_bar_pre_duck_global_yaw_fail_open_enabled = bool(
            self.get_parameter(
                'limit_bar_pre_duck_global_yaw_fail_open_enabled'
            ).value
        )
        self.limit_bar_pre_duck_stationary_turn_enabled = bool(
            self.get_parameter(
                'limit_bar_pre_duck_stationary_turn_enabled'
            ).value
        )
        self.limit_bar_pre_duck_stationary_turn_norm = min(
            1.0,
            max(
                0.0,
                float(
                    self.get_parameter(
                        'limit_bar_pre_duck_stationary_turn_norm'
                    ).value
                ),
            ),
        )
        self.limit_bar_lateral_shift_walk_yaw_correction_enabled = bool(
            self.get_parameter('limit_bar_lateral_shift_walk_yaw_correction_enabled').value
        )
        self.limit_bar_lateral_shift_walk_yaw_deadband_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'limit_bar_lateral_shift_walk_yaw_deadband_deg'
                    ).value
                ),
            )
        )
        self.limit_bar_lateral_shift_walk_yaw_gain_norm_per_rad = max(
            0.0,
            float(
                self.get_parameter(
                    'limit_bar_lateral_shift_walk_yaw_gain_norm_per_rad'
                ).value
            ),
        )
        self.limit_bar_lateral_shift_walk_yaw_max_delta_norm = max(
            0.0,
            float(
                self.get_parameter('limit_bar_lateral_shift_walk_yaw_max_delta_norm').value
            ),
        )
        self.limit_bar_lateral_shift_duck_align_ccw_left_norm = float(
            self.get_parameter('limit_bar_lateral_shift_duck_align_ccw_left_norm').value
        )
        self.limit_bar_lateral_shift_duck_align_ccw_right_norm = float(
            self.get_parameter('limit_bar_lateral_shift_duck_align_ccw_right_norm').value
        )
        self.limit_bar_lateral_shift_duck_align_cw_left_norm = float(
            self.get_parameter('limit_bar_lateral_shift_duck_align_cw_left_norm').value
        )
        self.limit_bar_lateral_shift_duck_align_cw_right_norm = float(
            self.get_parameter('limit_bar_lateral_shift_duck_align_cw_right_norm').value
        )
        self.limit_bar_duck_prepare_yaw_settle_enabled = bool(
            self.get_parameter('limit_bar_duck_prepare_yaw_settle_enabled').value
        )
        self.limit_bar_duck_prepare_yaw_tolerance_rad = math.radians(
            max(0.0, float(self.get_parameter('limit_bar_duck_prepare_yaw_tolerance_deg').value))
        )
        self.limit_bar_duck_prepare_yaw_hold_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_duck_prepare_yaw_hold_sec').value),
        )
        self.limit_bar_duck_prepare_yaw_timeout_sec = max(
            0.0,
            float(self.get_parameter('limit_bar_duck_prepare_yaw_timeout_sec').value),
        )
        self.limit_bar_duck_prepare_yaw_mode = int(
            self.get_parameter('limit_bar_duck_prepare_yaw_mode').value
        )
        self.limit_bar_duck_prepare_yaw_cw_left_norm = float(
            self.get_parameter('limit_bar_duck_prepare_yaw_cw_left_norm').value
        )
        self.limit_bar_duck_prepare_yaw_cw_right_norm = float(
            self.get_parameter('limit_bar_duck_prepare_yaw_cw_right_norm').value
        )
        self.limit_bar_duck_prepare_yaw_ccw_left_norm = float(
            self.get_parameter('limit_bar_duck_prepare_yaw_ccw_left_norm').value
        )
        self.limit_bar_duck_prepare_yaw_ccw_right_norm = float(
            self.get_parameter('limit_bar_duck_prepare_yaw_ccw_right_norm').value
        )
        self.duck_centerline_correction_enabled = bool(
            self.get_parameter('duck_centerline_correction_enabled').value
        )
        self.duck_centerline_lateral_gain_norm_per_m = max(
            0.0,
            float(self.get_parameter('duck_centerline_lateral_gain_norm_per_m').value),
        )
        self.duck_centerline_yaw_gain_norm_per_rad = max(
            0.0,
            float(self.get_parameter('duck_centerline_yaw_gain_norm_per_rad').value),
        )
        self.duck_centerline_deadband_m = max(
            0.0, float(self.get_parameter('duck_centerline_deadband_m').value)
        )
        self.duck_centerline_yaw_deadband_rad = math.radians(
            max(0.0, float(self.get_parameter('duck_centerline_yaw_deadband_deg').value))
        )
        self.duck_centerline_max_delta_norm = max(
            0.0,
            min(1.0, float(self.get_parameter('duck_centerline_max_delta_norm').value)),
        )
        self.duck_centerline_lateral_fail_m = max(
            0.0, float(self.get_parameter('duck_centerline_lateral_fail_m').value)
        )
        self.duck_centerline_yaw_fail_rad = math.radians(
            max(0.0, float(self.get_parameter('duck_centerline_yaw_fail_deg').value))
        )
        self.duck_centerline_odom_forward_axis = (
            str(self.get_parameter('duck_centerline_odom_forward_axis').value)
            .strip()
            .lower()
        )
        if self.duck_centerline_odom_forward_axis not in ('x', 'y'):
            raise RuntimeError(
                'duck_centerline_odom_forward_axis must be "x" or "y".'
            )

        self.hurdle_visual_trigger_enabled = bool(
            self.get_parameter('hurdle_visual_trigger_enabled').value
        )
        self.hurdle_tag_trigger_enabled = bool(
            self.get_parameter('hurdle_tag_trigger_enabled').value
        )
        self.hurdle_tag_target_tag_id = int(
            self.get_parameter('hurdle_tag_target_tag_id').value
        )
        self.hurdle_tag_frame_id = str(
            self.get_parameter('hurdle_tag_frame_id').value
        )
        self.hurdle_tag_child_frame_id = str(
            self.get_parameter('hurdle_tag_child_frame_id').value
        )
        self.hurdle_tag_to_center_x_m = float(
            self.get_parameter('hurdle_tag_to_center_x_m').value
        )
        self.hurdle_lateral_shift_target_distance_m = max(
            0.0,
            float(self.get_parameter('hurdle_lateral_shift_target_distance_m').value),
        )
        self.hurdle_lateral_shift_yaw_sign = (
            -1.0
            if float(self.get_parameter('hurdle_lateral_shift_yaw_sign').value) < 0.0
            else 1.0
        )
        self.hurdle_lateral_shift_distance_scale = max(
            0.0,
            min(
                1.5,
                float(self.get_parameter('hurdle_lateral_shift_distance_scale').value),
            ),
        )
        self.hurdle_lateral_shift_deadband_m = max(
            0.0,
            float(self.get_parameter('hurdle_lateral_shift_deadband_m').value),
        )
        self.hurdle_lateral_shift_arrival_tolerance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'hurdle_lateral_shift_arrival_tolerance_m'
                ).value
            ),
        )
        self.hurdle_final_forward_tolerance_m = max(
            0.0,
            float(self.get_parameter('hurdle_final_forward_tolerance_m').value),
        )
        self.hurdle_lateral_shift_legacy_relock_final_forward_enabled = bool(
            self.get_parameter(
                'hurdle_lateral_shift_legacy_relock_final_forward_enabled'
            ).value
        )
        self.hurdle_lateral_shift_legacy_relock_visual_cap_enabled = bool(
            self.get_parameter(
                'hurdle_lateral_shift_legacy_relock_visual_cap_enabled'
            ).value
        )
        self.hurdle_lateral_shift_legacy_relock_min_tag_samples = max(
            1,
            int(
                self.get_parameter(
                    'hurdle_lateral_shift_legacy_relock_min_tag_samples'
                ).value
            ),
        )
        self.hurdle_lateral_shift_legacy_relock_max_reduction_m = max(
            0.0,
            float(
                self.get_parameter(
                    'hurdle_lateral_shift_legacy_relock_max_reduction_m'
                ).value
            ),
        )
        self.hurdle_lateral_shift_legacy_slowdown_distance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'hurdle_lateral_shift_legacy_slowdown_distance_m'
                ).value
            ),
        )
        self.hurdle_lateral_shift_legacy_slow_left_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'hurdle_lateral_shift_legacy_slow_left_norm'
                ).value
            )
        )
        self.hurdle_lateral_shift_legacy_slow_right_norm = self._clamp_norm(
            float(
                self.get_parameter(
                    'hurdle_lateral_shift_legacy_slow_right_norm'
                ).value
            )
        )
        self.hurdle_lateral_shift_legacy_max_forward_overshoot_m = max(
            0.0,
            float(
                self.get_parameter(
                    'hurdle_lateral_shift_legacy_max_forward_overshoot_m'
                ).value
            ),
        )
        self.hurdle_lateral_shift_visual_jump_enabled = bool(
            self.get_parameter('hurdle_lateral_shift_visual_jump_enabled').value
        )
        self.hurdle_lateral_shift_visual_jump_distance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'hurdle_lateral_shift_visual_jump_distance_m'
                ).value
            ),
        )
        self.hurdle_lateral_shift_visual_jump_max_age_sec = max(
            0.0,
            float(self.get_parameter('hurdle_lateral_shift_visual_jump_max_age_sec').value),
        )
        self.hurdle_lateral_shift_visual_jump_yaw_gate_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('hurdle_lateral_shift_visual_jump_yaw_gate_deg').value),
            )
        )
        self.hurdle_lateral_s_curve_visual_jump_confirm_samples = max(
            1,
            int(
                self.get_parameter(
                    'hurdle_lateral_s_curve_visual_jump_confirm_samples'
                ).value
            ),
        )
        self.hurdle_lateral_s_curve_visual_jump_confirm_duration_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'hurdle_lateral_s_curve_visual_jump_confirm_duration_sec'
                ).value
            ),
        )
        self.hurdle_lateral_s_curve_visual_jump_max_remaining_m = max(
            0.0,
            float(
                self.get_parameter(
                    'hurdle_lateral_s_curve_visual_jump_max_remaining_m'
                ).value
            ),
        )
        self.hurdle_lateral_s_curve_visual_jump_tag_max_delta_m = max(
            0.0,
            float(
                self.get_parameter(
                    'hurdle_lateral_s_curve_visual_jump_tag_max_delta_m'
                ).value
            ),
        )
        self.hurdle_tag_stale_timeout_sec = max(
            0.0, float(self.get_parameter('hurdle_tag_stale_timeout_sec').value)
        )
        self.hurdle_tag_min_samples = max(
            1, int(self.get_parameter('hurdle_tag_min_samples').value)
        )
        self.hurdle_tag_max_sample_age_sec = max(
            0.0, float(self.get_parameter('hurdle_tag_max_sample_age_sec').value)
        )
        self.hurdle_tf_buffer_fallback_enabled = bool(
            self.get_parameter('hurdle_tf_buffer_fallback_enabled').value
        )
        self.hurdle_tag_trigger_distance_m = max(
            0.0, float(self.get_parameter('hurdle_tag_trigger_distance_m').value)
        )
        self.hurdle_tag_odom_handoff_enabled = bool(
            self.get_parameter('hurdle_tag_odom_handoff_enabled').value
        )
        self.hurdle_tag_odom_handoff_timeout_sec = max(
            0.0,
            float(self.get_parameter('hurdle_tag_odom_handoff_timeout_sec').value),
        )
        self.hurdle_tag_odom_handoff_max_start_distance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'hurdle_tag_odom_handoff_max_start_distance_m'
                ).value
            ),
        )
        self.hurdle_tag_slope_guard_enabled = bool(
            self.get_parameter('hurdle_tag_slope_guard_enabled').value
        )
        self.hurdle_tag_slope_guard_distance_m = max(
            0.0,
            float(self.get_parameter('hurdle_tag_slope_guard_distance_m').value),
        )
        self.hurdle_tag_yaw_reference_mode = (
            str(self.get_parameter('hurdle_tag_yaw_reference_mode').value)
            .strip()
            .lower()
        )
        if self.hurdle_tag_yaw_reference_mode not in ('tag_lateral', 'fixed_absolute'):
            raise RuntimeError(
                'hurdle_tag_yaw_reference_mode must be tag_lateral or fixed_absolute.'
            )
        self.hurdle_tag_fixed_reference_yaw = math.radians(
            float(self.get_parameter('hurdle_tag_fixed_reference_yaw_deg').value)
        )
        self.hurdle_lateral_pre_align_enabled = bool(
            self.get_parameter('hurdle_lateral_pre_align_enabled').value
        )
        self.hurdle_lateral_pre_align_mode = (
            str(self.get_parameter('hurdle_lateral_pre_align_mode').value)
            .strip()
            .lower()
        )
        if self.hurdle_lateral_pre_align_mode not in (
            'tag_center',
            'centerline',
            'pose_roll',
            'pose_pitch',
            'pose_yaw',
            'fixed_absolute_yaw',
        ):
            raise RuntimeError(
                'hurdle_lateral_pre_align_mode must be one of: tag_center, '
                'centerline, pose_roll, pose_pitch, pose_yaw, fixed_absolute_yaw'
            )
        self.hurdle_lateral_pre_align_use_visual_feedback = bool(
            self.get_parameter('hurdle_lateral_pre_align_use_visual_feedback').value
        )
        self.hurdle_lateral_pre_align_timeout_sec = max(
            0.0,
            float(self.get_parameter('hurdle_lateral_pre_align_timeout_sec').value),
        )
        self.hurdle_lateral_pre_align_max_visual_error_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'hurdle_lateral_pre_align_max_visual_error_deg'
                    ).value
                ),
            )
        )
        self.hurdle_lateral_pre_align_hold_sec = max(
            0.0,
            float(self.get_parameter('hurdle_lateral_pre_align_hold_sec').value),
        )
        self.hurdle_lateral_pre_align_tolerance_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('hurdle_lateral_pre_align_tolerance_deg').value),
            )
        )
        self.hurdle_lateral_pre_align_stop_during_hold_enabled = bool(
            self.get_parameter(
                'hurdle_lateral_pre_align_stop_during_hold_enabled'
            ).value
        )
        self.hurdle_lateral_pre_align_independent_turn_enabled = bool(
            self.get_parameter(
                'hurdle_lateral_pre_align_independent_turn_enabled'
            ).value
        )
        self.hurdle_fine_yaw_turn_ccw_left_norm = self._clamp_norm(
            float(self.get_parameter('hurdle_fine_yaw_turn_ccw_left_norm').value)
        )
        self.hurdle_fine_yaw_turn_ccw_right_norm = self._clamp_norm(
            float(self.get_parameter('hurdle_fine_yaw_turn_ccw_right_norm').value)
        )
        self.hurdle_fine_yaw_turn_cw_left_norm = self._clamp_norm(
            float(self.get_parameter('hurdle_fine_yaw_turn_cw_left_norm').value)
        )
        self.hurdle_fine_yaw_turn_cw_right_norm = self._clamp_norm(
            float(self.get_parameter('hurdle_fine_yaw_turn_cw_right_norm').value)
        )
        self.hurdle_fine_yaw_turn_min_scale = min(
            1.0,
            max(
                0.0,
                float(self.get_parameter('hurdle_fine_yaw_turn_min_scale').value),
            ),
        )
        self.hurdle_fine_yaw_turn_full_error_rad = math.radians(
            max(
                0.1,
                float(
                    self.get_parameter(
                        'hurdle_fine_yaw_turn_full_error_deg'
                    ).value
                ),
            )
        )
        self.hurdle_lateral_pre_align_pose_control_sign = float(
            self.get_parameter('hurdle_lateral_pre_align_pose_control_sign').value
        )
        self.hurdle_lateral_pre_align_target_pose_pitch_rad = math.radians(
            float(
                self.get_parameter(
                    'hurdle_lateral_pre_align_target_pose_pitch_deg'
                ).value
            )
        )
        self.hurdle_stand_mode = int(self.get_parameter('hurdle_stand_mode').value)
        self.hurdle_stand_left_norm = float(self.get_parameter('hurdle_stand_left_norm').value)
        self.hurdle_stand_right_norm = float(self.get_parameter('hurdle_stand_right_norm').value)
        self.hurdle_jump_mode = int(self.get_parameter('hurdle_jump_mode').value)
        self.hurdle_jump_left_norm = float(self.get_parameter('hurdle_jump_left_norm').value)
        self.hurdle_jump_right_norm = float(self.get_parameter('hurdle_jump_right_norm').value)
        self.hurdle_stand_to_jump_delay_sec = max(
            0.0, float(self.get_parameter('hurdle_stand_to_jump_delay_sec').value)
        )
        self.hurdle_jump_to_walk_delay_sec = max(
            0.0, float(self.get_parameter('hurdle_jump_to_walk_delay_sec').value)
        )
        self.hurdle_jump_step_once_repeat_count = max(
            1, int(self.get_parameter('hurdle_jump_step_once_repeat_count').value)
        )
        self.hurdle_jump_step_once_repeat_interval_sec = max(
            0.0,
            float(self.get_parameter('hurdle_jump_step_once_repeat_interval_sec').value),
        )
        self.hurdle_post_align_enabled = bool(
            self.get_parameter('hurdle_post_align_enabled').value
        )
        self.hurdle_post_align_use_startup_yaw = bool(
            self.get_parameter('hurdle_post_align_use_startup_yaw').value
        )
        self.hurdle_post_align_target_yaw = math.radians(
            float(self.get_parameter('hurdle_post_align_target_yaw_deg').value)
        )
        self.hurdle_post_align_tolerance_rad = math.radians(
            max(0.0, float(self.get_parameter('hurdle_post_align_tolerance_deg').value))
        )
        self.hurdle_post_align_exit_hold_sec = max(
            0.0, float(self.get_parameter('hurdle_post_align_exit_hold_sec').value)
        )
        self.hurdle_post_align_timeout_sec = max(
            0.0, float(self.get_parameter('hurdle_post_align_timeout_sec').value)
        )
        self.hurdle_post_align_timeout_action = str(
            self.get_parameter('hurdle_post_align_timeout_action').value
        ).strip().lower()
        self.hurdle_post_align_mode = int(
            self.get_parameter('hurdle_post_align_mode').value
        )
        self.hurdle_post_align_ccw_left_norm = float(
            self.get_parameter('hurdle_post_align_ccw_left_norm').value
        )
        self.hurdle_post_align_ccw_right_norm = float(
            self.get_parameter('hurdle_post_align_ccw_right_norm').value
        )
        self.hurdle_post_align_cw_left_norm = float(
            self.get_parameter('hurdle_post_align_cw_left_norm').value
        )
        self.hurdle_post_align_cw_right_norm = float(
            self.get_parameter('hurdle_post_align_cw_right_norm').value
        )
        self.hurdle_post_align_fine_threshold_rad = math.radians(
            max(0.0, float(self.get_parameter('hurdle_post_align_fine_threshold_deg').value))
        )
        self.hurdle_post_align_fine_ccw_left_norm = float(
            self.get_parameter('hurdle_post_align_fine_ccw_left_norm').value
        )
        self.hurdle_post_align_fine_ccw_right_norm = float(
            self.get_parameter('hurdle_post_align_fine_ccw_right_norm').value
        )
        self.hurdle_post_align_fine_cw_left_norm = float(
            self.get_parameter('hurdle_post_align_fine_cw_left_norm').value
        )
        self.hurdle_post_align_fine_cw_right_norm = float(
            self.get_parameter('hurdle_post_align_fine_cw_right_norm').value
        )
        self.hurdle_post_align_hold_mode = int(
            self.get_parameter('hurdle_post_align_hold_mode').value
        )
        self.hurdle_post_align_hold_left_norm = float(
            self.get_parameter('hurdle_post_align_hold_left_norm').value
        )
        self.hurdle_post_align_hold_right_norm = float(
            self.get_parameter('hurdle_post_align_hold_right_norm').value
        )
        self.upstairs_step_once_mode = int(self.get_parameter('upstairs_step_once_mode').value)
        self.upstairs_step_once_left_norm = float(
            self.get_parameter('upstairs_step_once_left_norm').value
        )
        self.upstairs_step_once_right_norm = float(
            self.get_parameter('upstairs_step_once_right_norm').value
        )
        self.upstairs_step_once_resend_after_sec = max(
            0.0,
            float(self.get_parameter('upstairs_step_once_resend_after_sec').value),
        )
        self.upstairs_step_once_resend_max_count = max(
            0,
            int(self.get_parameter('upstairs_step_once_resend_max_count').value),
        )
        self.upstairs_lateral_shift_yaw_rad = math.radians(
            max(0.0, abs(float(self.get_parameter('upstairs_lateral_shift_yaw_deg').value)))
        )
        self.upstairs_lateral_shift_yaw_sign = (
            -1.0
            if float(self.get_parameter('upstairs_lateral_shift_yaw_sign').value) < 0.0
            else 1.0
        )
        self.upstairs_lateral_shift_distance_scale = max(
            0.0,
            min(
                1.5,
                float(
                    self.get_parameter(
                        'upstairs_lateral_shift_distance_scale'
                    ).value
                ),
            ),
        )
        self.upstairs_lateral_shift_target_distance_m = max(
            0.0,
            float(self.get_parameter('upstairs_lateral_shift_target_distance_m').value),
        )
        self.upstairs_final_forward_tolerance_m = max(
            0.0,
            float(self.get_parameter('upstairs_final_forward_tolerance_m').value),
        )
        self.upstairs_final_tag_distance_tolerance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'upstairs_final_tag_distance_tolerance_m'
                ).value
            ),
        )
        self.upstairs_final_visual_distance_m = max(
            0.0,
            float(self.get_parameter('upstairs_final_visual_distance_m').value),
        )
        self.upstairs_final_slowdown_distance_m = max(
            0.0,
            float(self.get_parameter('upstairs_final_slowdown_distance_m').value),
        )
        self.upstairs_final_slow_walk_left_norm = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter('upstairs_final_slow_walk_left_norm').value),
            ),
        )
        self.upstairs_final_slow_walk_right_norm = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter('upstairs_final_slow_walk_right_norm').value),
            ),
        )
        self.upstairs_final_confirm_creep_max_extra_m = max(
            0.0,
            float(
                self.get_parameter(
                    'upstairs_final_confirm_creep_max_extra_m'
                ).value
            ),
        )
        self.upstairs_final_confirm_creep_left_norm = max(
            0.0,
            min(
                1.0,
                float(
                    self.get_parameter(
                        'upstairs_final_confirm_creep_left_norm'
                    ).value
                ),
            ),
        )
        self.upstairs_final_confirm_creep_right_norm = max(
            0.0,
            min(
                1.0,
                float(
                    self.get_parameter(
                        'upstairs_final_confirm_creep_right_norm'
                    ).value
                ),
            ),
        )
        self.upstairs_lateral_shift_deadband_m = max(
            0.0,
            float(self.get_parameter('upstairs_lateral_shift_deadband_m').value),
        )
        self.upstairs_lateral_shift_arrival_tolerance_m = max(
            0.0,
            float(
                self.get_parameter(
                    'upstairs_lateral_shift_arrival_tolerance_m'
                ).value
            ),
        )
        self.upstairs_lateral_shift_max_distance_m = max(
            0.0,
            float(self.get_parameter('upstairs_lateral_shift_max_distance_m').value),
        )
        self.upstairs_lateral_shift_live_check_enabled = bool(
            self.get_parameter('upstairs_lateral_shift_live_check_enabled').value
        )
        self.upstairs_lateral_shift_live_check_center_gate_m = max(
            0.0,
            float(self.get_parameter('upstairs_lateral_shift_live_check_center_gate_m').value),
        )
        self.upstairs_lateral_shift_live_check_yaw_gate_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('upstairs_lateral_shift_live_check_yaw_gate_deg').value),
            )
        )
        self.upstairs_lateral_shift_live_check_max_replans = max(
            0,
            int(self.get_parameter('upstairs_lateral_shift_live_check_max_replans').value),
        )
        self.upstairs_final_forward_tag_yaw_correction_enabled = bool(
            self.get_parameter('upstairs_final_forward_tag_yaw_correction_enabled').value
        )
        self.upstairs_final_forward_tag_yaw_deadband_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('upstairs_final_forward_tag_yaw_deadband_deg').value),
            )
        )
        self.upstairs_final_forward_tag_yaw_gain_norm_per_rad = max(
            0.0,
            float(self.get_parameter('upstairs_final_forward_tag_yaw_gain_norm_per_rad').value),
        )
        self.upstairs_final_forward_tag_yaw_max_delta_norm = max(
            0.0,
            float(self.get_parameter('upstairs_final_forward_tag_yaw_max_delta_norm').value),
        )
        self.upstairs_final_forward_realign_gate_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'upstairs_final_forward_realign_gate_deg'
                    ).value
                ),
            )
        )
        self.upstairs_final_yaw_complete_gate_rad = math.radians(
            max(0.0, float(self.get_parameter('upstairs_final_yaw_complete_gate_deg').value))
        )

        self.slope_visual_detection_enabled = bool(
            self.get_parameter('slope_visual_detection_enabled').value
        )
        self.slope_roll_detection_enabled = bool(
            self.get_parameter('slope_roll_detection_enabled').value
        )
        self.slope_roll_detection_rad = math.radians(
            max(0.0, float(self.get_parameter('slope_roll_detection_deg').value))
        )
        self.slope_roll_detection_use_abs = bool(
            self.get_parameter('slope_roll_detection_use_abs').value
        )
        self.slope_roll_detection_hold_sec = max(
            0.0, float(self.get_parameter('slope_roll_detection_hold_sec').value)
        )
        self.slope_roll_detection_post_stairs_only = bool(
            self.get_parameter('slope_roll_detection_post_stairs_only').value
        )
        self.slope_fallback_enabled = bool(self.get_parameter('slope_fallback_enabled').value)
        self.slope_fallback_timeout_sec = max(
            0.0, float(self.get_parameter('slope_fallback_timeout_sec').value)
        )
        self.slope_fallback_min_distance_m = max(
            0.0, float(self.get_parameter('slope_fallback_min_distance_m').value)
        )
        self.slope_align_enabled = bool(self.get_parameter('slope_align_enabled').value)
        self.slope_align_on_detected_branch_enabled = bool(
            self.get_parameter('slope_align_on_detected_branch_enabled').value
        )
        self.slope_align_use_startup_yaw = bool(
            self.get_parameter('slope_align_use_startup_yaw').value
        )
        self.slope_align_target_yaw = math.radians(
            float(self.get_parameter('slope_align_target_yaw_deg').value)
        )
        self.slope_align_tolerance_rad = math.radians(
            max(0.0, float(self.get_parameter('slope_align_tolerance_deg').value))
        )
        self.slope_align_exit_hold_sec = max(
            0.0, float(self.get_parameter('slope_align_exit_hold_sec').value)
        )
        self.slope_align_timeout_sec = max(
            0.0, float(self.get_parameter('slope_align_timeout_sec').value)
        )
        self.slope_align_timeout_action = str(
            self.get_parameter('slope_align_timeout_action').value
        ).strip().lower()
        self.slope_align_mode = int(self.get_parameter('slope_align_mode').value)
        self.slope_align_ccw_left_norm = float(
            self.get_parameter('slope_align_ccw_left_norm').value
        )
        self.slope_align_ccw_right_norm = float(
            self.get_parameter('slope_align_ccw_right_norm').value
        )
        self.slope_align_cw_left_norm = float(
            self.get_parameter('slope_align_cw_left_norm').value
        )
        self.slope_align_cw_right_norm = float(
            self.get_parameter('slope_align_cw_right_norm').value
        )
        self.slope_align_fine_threshold_rad = math.radians(
            max(0.0, float(self.get_parameter('slope_align_fine_threshold_deg').value))
        )
        self.slope_align_fine_ccw_left_norm = float(
            self.get_parameter('slope_align_fine_ccw_left_norm').value
        )
        self.slope_align_fine_ccw_right_norm = float(
            self.get_parameter('slope_align_fine_ccw_right_norm').value
        )
        self.slope_align_fine_cw_left_norm = float(
            self.get_parameter('slope_align_fine_cw_left_norm').value
        )
        self.slope_align_fine_cw_right_norm = float(
            self.get_parameter('slope_align_fine_cw_right_norm').value
        )
        self.slope_align_hold_mode = int(self.get_parameter('slope_align_hold_mode').value)
        self.slope_align_hold_left_norm = float(
            self.get_parameter('slope_align_hold_left_norm').value
        )
        self.slope_align_hold_right_norm = float(
            self.get_parameter('slope_align_hold_right_norm').value
        )
        self.slope_align_reference_topic = str(
            self.get_parameter('slope_align_reference_topic').value
        )
        self.slope_retry_runtime_yaw_reference_enabled = bool(
            self.get_parameter('slope_retry_runtime_yaw_reference_enabled').value
        )
        self.slope_retry_standard_waypoints_basename = str(
            self.get_parameter('slope_retry_standard_waypoints_basename').value
        ).strip()
        self.slope_retry_mirror_waypoints_basename = str(
            self.get_parameter('slope_retry_mirror_waypoints_basename').value
        ).strip()
        self.slope_retry_rezero_search_only_slope = bool(
            self.get_parameter('slope_retry_rezero_search_only_slope').value
        )
        self.slope_retry_rezero_rearm_delay_sec = max(
            0.0,
            float(self.get_parameter('slope_retry_rezero_rearm_delay_sec').value),
        )
        self.slope_retry_route_rezero_once = bool(
            self.get_parameter('slope_retry_route_rezero_once').value
        )
        self.slope_bridge_full_route_retry_count = max(
            1,
            int(self.get_parameter('slope_bridge_full_route_retry_count').value),
        )
        self.slope_retry_pre_bridge_waypoints = {
            str(name).strip()
            for name in self.get_parameter('slope_retry_pre_bridge_waypoints').value
            if str(name).strip()
        }
        self.slope_retry_bridge_waypoints = {
            str(name).strip()
            for name in self.get_parameter('slope_retry_bridge_waypoints').value
            if str(name).strip()
        }
        self.slope_retry_search_waypoints = {
            str(name).strip()
            for name in self.get_parameter('slope_retry_search_waypoints').value
            if str(name).strip()
        }
        self.slope_route_yaw_reference_topic = str(
            self.get_parameter('slope_route_yaw_reference_topic').value
        )
        self.slope_bridge_retry_yaw_reference_topic = str(
            self.get_parameter('slope_bridge_retry_yaw_reference_topic').value
        )
        self.downstream_route_yaw_reference_enabled = bool(
            self.get_parameter('downstream_route_yaw_reference_enabled').value
        )
        self.downstream_route_yaw_reference_timeout_sec = max(
            0.0,
            float(
                self.get_parameter(
                    'downstream_route_yaw_reference_timeout_sec'
                ).value
            ),
        )
        self.downstream_route_yaw_hurdle_enabled = bool(
            self.get_parameter('downstream_route_yaw_hurdle_enabled').value
        )
        self.downstream_route_yaw_upstairs_enabled = bool(
            self.get_parameter('downstream_route_yaw_upstairs_enabled').value
        )
        self.downstream_route_yaw_limit_bar_pre_duck_enabled = bool(
            self.get_parameter(
                'downstream_route_yaw_limit_bar_pre_duck_enabled'
            ).value
        )
        self.post_stairs_slope_entry_enabled = bool(
            self.get_parameter('post_stairs_slope_entry_enabled').value
        )
        self.post_stairs_slope_entry_distance_m = max(
            0.0, float(self.get_parameter('post_stairs_slope_entry_distance_m').value)
        )
        self.post_stairs_slope_entry_timeout_sec = max(
            0.0, float(self.get_parameter('post_stairs_slope_entry_timeout_sec').value)
        )
        self.post_stairs_slope_entry_mode = int(
            self.get_parameter('post_stairs_slope_entry_mode').value
        )
        self.post_stairs_slope_entry_left_norm = float(
            self.get_parameter('post_stairs_slope_entry_left_norm').value
        )
        self.post_stairs_slope_entry_right_norm = float(
            self.get_parameter('post_stairs_slope_entry_right_norm').value
        )
        self.post_stairs_slope_entry_initial_yaw_align_enabled = bool(
            self.get_parameter(
                'post_stairs_slope_entry_initial_yaw_align_enabled'
            ).value
        )
        self.post_stairs_slope_entry_yaw_correction_enabled = bool(
            self.get_parameter('post_stairs_slope_entry_yaw_correction_enabled').value
        )
        self.post_stairs_slope_entry_yaw_tolerance_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('post_stairs_slope_entry_yaw_tolerance_deg').value),
            )
        )
        self.post_stairs_slope_entry_yaw_gain_norm_per_rad = max(
            0.0,
            float(self.get_parameter('post_stairs_slope_entry_yaw_gain_norm_per_rad').value),
        )
        self.post_stairs_slope_entry_yaw_max_delta_norm = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter('post_stairs_slope_entry_yaw_max_delta_norm').value),
            ),
        )
        self.post_stairs_slope_entry_finish_yaw_realign_threshold_rad = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'post_stairs_slope_entry_finish_yaw_realign_threshold_deg'
                    ).value
                ),
            )
        )
        self.post_stairs_slope_search_after_entry = bool(
            self.get_parameter('post_stairs_slope_search_after_entry').value
        )
        self.post_stairs_slope_search_rearm_delay_sec = max(
            0.0,
            float(self.get_parameter('post_stairs_slope_search_rearm_delay_sec').value),
        )
        self.post_stairs_slope_priority_enabled = bool(
            self.get_parameter('post_stairs_slope_priority_enabled').value
        )
        self.post_stairs_slope_roll_priority_enabled = bool(
            self.get_parameter('post_stairs_slope_roll_priority_enabled').value
        )
        self.post_stairs_slope_roll_priority_rad = math.radians(
            max(
                0.0,
                float(self.get_parameter('post_stairs_slope_roll_priority_deg').value),
            )
        )
        self.post_stairs_slope_roll_priority_use_abs = bool(
            self.get_parameter('post_stairs_slope_roll_priority_use_abs').value
        )
        self.post_stairs_slope_roll_priority_hold_sec = max(
            0.0,
            float(self.get_parameter('post_stairs_slope_roll_priority_hold_sec').value),
        )
        self.post_stairs_slope_confirm_required = bool(
            self.get_parameter('post_stairs_slope_confirm_required').value
        )
        self.post_stairs_slope_confirm_timeout_sec = max(
            0.0, float(self.get_parameter('post_stairs_slope_confirm_timeout_sec').value)
        )
        self.post_stairs_slope_confirm_timeout_action = str(
            self.get_parameter('post_stairs_slope_confirm_timeout_action').value
        ).strip().lower()
        self.post_stairs_slope_confirm_hold_mode = int(
            self.get_parameter('post_stairs_slope_confirm_hold_mode').value
        )
        self.post_stairs_slope_confirm_hold_left_norm = float(
            self.get_parameter('post_stairs_slope_confirm_hold_left_norm').value
        )
        self.post_stairs_slope_confirm_hold_right_norm = float(
            self.get_parameter('post_stairs_slope_confirm_hold_right_norm').value
        )
        if self.slope_align_timeout_action not in ('advance', 'fail'):
            raise RuntimeError(
                'slope_align_timeout_action must be "advance" or "fail".'
            )
        if self.hurdle_post_align_timeout_action not in ('advance', 'fail'):
            raise RuntimeError(
                'hurdle_post_align_timeout_action must be "advance" or "fail".'
            )
        if self.post_stairs_slope_confirm_timeout_action not in ('advance', 'fail', 'search'):
            raise RuntimeError(
                'post_stairs_slope_confirm_timeout_action must be '
                '"advance", "fail", or "search".'
            )
        self._configure_sandpit_bypass()
        self._configure_pole_bypass()

        self.pose_x: Optional[float] = None
        self.pose_y: Optional[float] = None
        self.pose_yaw: Optional[float] = None
        self.pose_roll: Optional[float] = None
        self.pose_pitch: Optional[float] = None
        self.pose_frame: str = ''
        self.pose_stamp_sec: float = 0.0
        self.pose_yaw_rate_rad_s = 0.0
        self.pose_yaw_rate_raw_rad_s = 0.0
        self.pose_yaw_rate_estimator = WindowedYawRateEstimator(
            window_sec=self.limit_bar_lateral_turn_yaw_rate_window_sec,
            min_span_sec=self.limit_bar_lateral_turn_yaw_rate_min_span_sec,
            max_abs_rate_rad_s=(
                self.limit_bar_lateral_turn_yaw_rate_max_abs_rad_s
            ),
            smoothing_alpha=(
                self.limit_bar_lateral_turn_yaw_rate_filter_alpha
            ),
        )
        self.startup_pose_yaw: Optional[float] = None
        self.slope_align_reference_yaw: Optional[float] = None
        self.slope_align_reference_frame: str = ''
        self.slope_align_reference_stamp_sec: float = 0.0
        self.slope_route_yaw_reference: Optional[float] = None
        self.slope_route_yaw_reference_frame: str = ''
        self.slope_route_yaw_reference_stamp_sec: float = 0.0
        self.slope_bridge_retry_yaw_reference: Optional[float] = None
        self.slope_bridge_retry_yaw_reference_frame: str = ''
        self.slope_bridge_retry_yaw_reference_stamp_sec: float = 0.0
        self.slope_bridge_retry_count = 0
        self.slope_retry_route_rezero_used = False
        self.slope_retry_rezero_search_active = False
        self.limit_detection = DetectionState()
        self.hurdle_detection = DetectionState()
        self.hurdle_visual_detection = DetectionState()
        self.pole_detection = DetectionState()
        self.pole_apriltag_detection = DetectionState()
        self.pole_apriltag_last_seen_stamp_sec = 0.0
        self.pole_apriltag_handoff_active = False
        self.pole_apriltag_handoff_stamp_sec = 0.0
        self.pole_apriltag_handoff_start_x: Optional[float] = None
        self.pole_apriltag_handoff_start_y: Optional[float] = None
        self.pole_apriltag_handoff_start_yaw: Optional[float] = None
        self.pole_apriltag_handoff_tag_odom_x: Optional[float] = None
        self.pole_apriltag_handoff_tag_odom_y: Optional[float] = None
        self.pole_apriltag_handoff_distance_m = 0.0
        self.pole_apriltag_handoff_lateral_m = 0.0
        self.pole_apriltag_ready_source = ''
        self.last_pole_apriltag_handoff_log_sec = 0.0
        self.slope_detection = DetectionState()
        self.upstairs_detection = DetectionState()
        self.upstairs_visual_detection = DetectionState()
        self.sandpit_detection = DetectionState()
        self.stairs_state = 'INACTIVE'
        self.slope_state = 'INACTIVE'
        self.pole_state = 'INACTIVE'
        self.failed_from_state = ''
        self.failed_from_branch = ''
        self.last_failure_text = ''
        self.slope_current_waypoint_name = ''
        self.slope_branch_completed = False
        self.limit_bar_completed = False
        self.hurdle_completed = False
        self.hurdle_entry_yaw: Optional[float] = None
        self.pole_completed = False
        self.pole_retry_failure_count = 0
        self.pole_bypass_completed = False
        self.pole_completed_waypoints = 0
        self.pole_pre_bypass_pending = False
        self.pole_pre_bypass_reason = ''
        self.upstairs_completed = False
        self.upstairs_jumping_signal_received = False
        self.upstairs_jumping_signal_stamp_sec = 0.0
        self.upstairs_step_once_waiting_ack = False
        self.upstairs_step_once_next_resend_sec: Optional[float] = None
        self.upstairs_step_once_resend_count = 0
        self.upstairs_done_retry_guard_deadline_sec: Optional[float] = None
        self.sandpit_completed = False

        self.search_enter_time = 0.0
        self.search_arm_deadline = 0.0
        self.search_reason = ''
        self.search_start_x: Optional[float] = None
        self.search_start_y: Optional[float] = None
        self.search_reference_yaw: Optional[float] = None
        self.last_search_yaw_log_time = 0.0
        self.slope_roll_detection_start_sec = 0.0
        self.last_slope_roll_detection_log_sec = 0.0
        self.slope_align_enter_time = 0.0
        self.slope_align_reason = ''
        self.slope_align_feedback = ''
        self.slope_align_next_action = 'slope_branch'
        self.slope_align_custom_target_yaw: Optional[float] = None
        self.last_slope_align_log_time = 0.0
        self.slope_align_stable_since: Optional[float] = None
        self.post_stairs_slope_entry_start_x: Optional[float] = None
        self.post_stairs_slope_entry_start_y: Optional[float] = None
        self.post_stairs_slope_confirm_start_time = 0.0
        self.last_post_stairs_slope_log_time = 0.0
        self.post_stairs_slope_roll_priority_start_sec = 0.0
        self.last_post_stairs_slope_roll_priority_log_sec = 0.0

        self.duck_start_x: Optional[float] = None
        self.duck_start_y: Optional[float] = None
        self.duck_reference_yaw: Optional[float] = None
        self.duck_reference_yaw_source = ''
        self.duck_reference_yaw_offset_rad = 0.0
        self.limit_bar_tag_samples: List[LimitBarTagSample] = []
        self.hurdle_tag_samples: List[LimitBarTagSample] = []
        self.upstairs_tag_samples: List[LimitBarTagSample] = []
        self.sandpit_tag_samples: List[SandpitTagSample] = []
        self.limit_bar_tag_detection_stamp_sec = 0.0
        self.limit_bar_tag_detection_ids_text = ''
        self.last_limit_bar_tag_tf_log_sec = 0.0
        self.last_limit_bar_tag_wait_log_sec = 0.0
        self.last_limit_bar_tag_guard_log_sec = 0.0
        self.limit_bar_tag_handoff_active = False
        self.limit_bar_tag_handoff_stamp_sec = 0.0
        self.limit_bar_tag_handoff_start_x: Optional[float] = None
        self.limit_bar_tag_handoff_start_y: Optional[float] = None
        self.limit_bar_tag_handoff_start_yaw: Optional[float] = None
        self.limit_bar_tag_handoff_tag_x_m = 0.0
        self.limit_bar_tag_handoff_tag_z_m = 0.0
        self.limit_bar_tag_handoff_center_x_m = 0.0
        self.limit_bar_tag_handoff_source = ''
        self.limit_bar_tag_handoff_pose_roll_rad = 0.0
        self.limit_bar_tag_handoff_pose_pitch_rad = 0.0
        self.last_limit_bar_tag_presearch_handoff_log_sec = 0.0
        self.limit_bar_locked_tag_valid = False
        self.limit_bar_locked_tag_stamp_sec = 0.0
        self.limit_bar_locked_tag_x_m = 0.0
        self.limit_bar_locked_tag_z_m = 0.0
        self.limit_bar_locked_center_x_m = 0.0
        self.limit_bar_locked_tag_odom_yaw: Optional[float] = None
        self.limit_bar_locked_tag_sample_count = 0
        self.limit_bar_locked_tag_source = ''
        self.hurdle_tag_detection_stamp_sec = 0.0
        self.hurdle_tag_detection_ids_text = ''
        self.last_hurdle_tag_tf_log_sec = 0.0
        self.last_hurdle_tf_buffer_log_sec = 0.0
        self.hurdle_tf_buffer_last_source_stamp_sec = 0.0
        self.last_hurdle_tag_wait_log_sec = 0.0
        self.last_hurdle_tag_guard_log_sec = 0.0
        self.hurdle_tag_handoff_active = False
        self.hurdle_tag_handoff_stamp_sec = 0.0
        self.hurdle_tag_handoff_start_x: Optional[float] = None
        self.hurdle_tag_handoff_start_y: Optional[float] = None
        self.hurdle_tag_handoff_start_yaw: Optional[float] = None
        self.hurdle_tag_handoff_tag_x_m = 0.0
        self.hurdle_tag_handoff_tag_z_m = 0.0
        self.hurdle_tag_handoff_center_x_m = 0.0
        self.hurdle_locked_tag_valid = False
        self.hurdle_locked_tag_stamp_sec = 0.0
        self.hurdle_locked_tag_x_m = 0.0
        self.hurdle_locked_tag_z_m = 0.0
        self.hurdle_locked_center_x_m = 0.0
        self.upstairs_tag_detection_stamp_sec = 0.0
        self.upstairs_tag_detection_ids_text = ''
        self.last_upstairs_tag_tf_log_sec = 0.0
        self.last_upstairs_tag_wait_log_sec = 0.0
        self.last_upstairs_tag_guard_log_sec = 0.0
        self.last_upstairs_visual_fallback_log_sec = 0.0
        self.upstairs_tag_handoff_active = False
        self.upstairs_tag_handoff_stamp_sec = 0.0
        self.upstairs_tag_handoff_start_x: Optional[float] = None
        self.upstairs_tag_handoff_start_y: Optional[float] = None
        self.upstairs_tag_handoff_start_yaw: Optional[float] = None
        self.upstairs_tag_handoff_tag_x_m = 0.0
        self.upstairs_tag_handoff_tag_z_m = 0.0
        self.upstairs_tag_handoff_center_x_m = 0.0
        self.apriltag_tf_last_received_sec = 0.0
        self.apriltag_tf_last_children_text = 'none'
        self.sandpit_tag_detection_stamp_sec = 0.0
        self.sandpit_tag_detection_ids_text = ''
        self.sandpit_priority_guard_stamp_sec = 0.0
        self.last_sandpit_tag_log_sec = 0.0
        self.sandpit_pre_align_locked = False
        self.sandpit_pre_align_start_sec = 0.0
        self.sandpit_pre_align_locked_distance_m: Optional[float] = None
        self.sandpit_pre_align_locked_lateral_m: Optional[float] = None
        self.sandpit_pre_align_start_x: Optional[float] = None
        self.sandpit_pre_align_start_y: Optional[float] = None
        self.sandpit_pre_align_start_yaw: Optional[float] = None
        self.sandpit_pre_align_correction_kind = ''
        self.sandpit_pre_align_target_yaw: Optional[float] = None
        self.sandpit_pre_align_pose_error_rad = 0.0
        self.sandpit_pre_align_locked_lateral_error_m = 0.0
        self.sandpit_pre_align_locked_center_angle_rad = 0.0
        self.sandpit_pre_align_locked_pose_metric = ''
        self.sandpit_pre_align_locked_pose_measured_rad = 0.0
        self.sandpit_pre_align_locked_pose_raw_error_rad = 0.0
        self.sandpit_pre_align_locked_pose_control_error_rad = 0.0
        self.sandpit_pre_align_completed_yaw: Optional[float] = None
        self.sandpit_pre_align_fallback_active = False
        self.sandpit_pre_align_active_mode = self.sandpit_pre_align_mode
        self.sandpit_pre_align_active_hold_mode = self.sandpit_pre_align_hold_mode
        self.sandpit_pre_align_active_hold_left_norm = self.sandpit_pre_align_hold_left_norm
        self.sandpit_pre_align_active_hold_right_norm = self.sandpit_pre_align_hold_right_norm
        self.sandpit_pre_align_last_log_sec = 0.0
        self.sandpit_pre_align_overlay_active = False
        self.sandpit_pre_align_backup_active = False
        self.sandpit_pre_align_backup_start_distance_m: Optional[float] = None
        self.sandpit_pre_align_polyline_phase = ''
        self.sandpit_pre_align_polyline_base_yaw: Optional[float] = None
        self.sandpit_pre_align_polyline_side_yaw: Optional[float] = None
        self.sandpit_pre_align_polyline_distance_m = 0.0
        self.sandpit_pre_align_polyline_raw_distance_m = 0.0
        self.sandpit_pre_align_polyline_start_x: Optional[float] = None
        self.sandpit_pre_align_polyline_start_y: Optional[float] = None
        self.sandpit_pre_align_polyline_start_sec = 0.0
        self.sandpit_pre_align_polyline_completed_once = False
        self.limit_bar_tag_align_active = False
        self.limit_bar_lateral_pre_align_active = False
        self.limit_bar_lateral_pre_align_done = False
        self.limit_bar_lateral_pre_align_start_sec: Optional[float] = None
        self.limit_bar_lateral_pre_align_stable_since: Optional[float] = None
        self.limit_bar_lateral_pre_align_target_yaw: Optional[float] = None
        self.limit_bar_lateral_pre_align_source_metric = ''
        self.limit_bar_lateral_pre_align_source_target_rad = 0.0
        self.limit_bar_lateral_pre_align_source_measured_rad = 0.0
        self.limit_bar_lateral_pre_align_source_control_rad = 0.0
        self.limit_bar_lateral_pre_align_source_odom_yaw: Optional[float] = None
        self.limit_bar_lateral_pre_align_source_tag_x_m = 0.0
        self.limit_bar_lateral_pre_align_source_tag_z_m = 0.0
        self.limit_bar_lateral_pre_align_source_center_offset_m = 0.0
        self.limit_bar_lateral_shift_phase = ''
        self.limit_bar_lateral_shift_phase_start_sec = 0.0
        self.limit_bar_lateral_turn_stable_since_sec: Optional[float] = None
        self.limit_bar_lateral_turn_last_direction = 0
        self.limit_bar_lateral_turn_pending_direction = 0
        self.limit_bar_lateral_turn_reverse_hold_until_sec = 0.0
        self.limit_bar_lateral_shift_reason = ''
        self.limit_bar_lateral_shift_yaw_ref_text = 'unknown'
        self.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'
        self.limit_bar_lateral_shift_base_yaw: Optional[float] = None
        self.limit_bar_lateral_shift_side_yaw: Optional[float] = None
        self.limit_bar_lateral_shift_center_x_m = 0.0
        self.limit_bar_lateral_shift_center_z_m = 0.0
        self.limit_bar_lateral_shift_distance_m = 0.0
        self.limit_bar_lateral_shift_raw_distance_m = 0.0
        self.limit_bar_lateral_shift_requested_distance_m = 0.0
        self.limit_bar_lateral_shift_forward_component_m = 0.0
        self.limit_bar_lateral_shift_final_forward_m = 0.0
        self.lateral_s_curve_shadow_plan: Optional[LateralSCurvePlan] = None
        self.lateral_s_curve_shadow_branch = ''
        self.lateral_s_curve_shadow_start_sec = 0.0
        self.lateral_s_curve_shadow_summary_logged = False
        self.lateral_s_curve_control_active = False
        self.lateral_s_curve_control_plan: Optional[LateralSCurvePlan] = None
        self.lateral_s_curve_control_start_x: Optional[float] = None
        self.lateral_s_curve_control_start_y: Optional[float] = None
        self.lateral_s_curve_control_start_sec = 0.0
        self.lateral_s_curve_control_arrival_sec: Optional[float] = None
        self.lateral_s_curve_control_arrival_forward_m: Optional[float] = None
        self.lateral_s_curve_control_arrival_cross_error_m: Optional[float] = None
        self.lateral_s_curve_control_completion_max_forward_m: Optional[float] = None
        self.lateral_s_curve_control_completion_since_sec: Optional[float] = None
        self.limit_bar_lateral_s_curve_final_lateral_violation_since_sec: Optional[
            float
        ] = None
        self.limit_bar_lateral_s_curve_fail_open_cross_track_logged = False
        self.limit_bar_lateral_s_curve_fail_open_final_lateral_logged = False
        self.lateral_s_curve_control_completion_reason = ''
        self.lateral_s_curve_control_completion_recovery_active = False
        self.lateral_s_curve_control_completion_recovery_target_forward_m: Optional[
            float
        ] = None
        self.lateral_s_curve_control_completion_recovery_trigger = ''
        self.limit_bar_lateral_s_curve_control_completed = False
        self.hurdle_lateral_s_curve_visual_close_sample_count = 0
        self.hurdle_lateral_s_curve_visual_close_start_sec: Optional[float] = None
        self.last_hurdle_lateral_s_curve_visual_guard_log_sec = 0.0
        self.limit_bar_lateral_shift_segment_start_x: Optional[float] = None
        self.limit_bar_lateral_shift_segment_start_y: Optional[float] = None
        self.limit_bar_lateral_shift_duck_align_since_sec: Optional[float] = None
        self.upstairs_final_align_latched_along_m: Optional[float] = None
        self.upstairs_final_align_latched_raw_along_m: Optional[float] = None
        self.upstairs_final_align_latched_remaining_m: Optional[float] = None
        self.upstairs_final_align_latched_reason = ''
        self.upstairs_final_align_latched_live_text = ''
        self.limit_bar_pre_duck_global_yaw_align_start_sec: Optional[float] = None
        self.limit_bar_pre_duck_global_yaw_stable_since_sec: Optional[float] = None
        self.limit_bar_pre_duck_global_yaw_hold_active = False
        self.upstairs_lateral_shift_live_replan_count = 0
        self.last_limit_bar_lateral_shift_log_sec = 0.0
        self.limit_bar_duck_prepare_yaw_settle_start_sec: Optional[float] = None
        self.limit_bar_duck_prepare_yaw_stable_since_sec: Optional[float] = None
        self.last_limit_bar_duck_prepare_yaw_log_sec = 0.0
        self.limit_bar_post_resume_yaw_align_start_sec: Optional[float] = None
        self.limit_bar_post_resume_yaw_stable_since_sec: Optional[float] = None
        self.last_limit_bar_post_resume_yaw_log_sec = 0.0
        self.duck_centerline_locked = False
        self.duck_centerline_start_x: Optional[float] = None
        self.duck_centerline_start_y: Optional[float] = None
        self.duck_centerline_yaw: Optional[float] = None
        self.duck_centerline_goal_x: Optional[float] = None
        self.duck_centerline_goal_y: Optional[float] = None
        self.sandpit_phase_index: Optional[int] = None
        self.sandpit_phase_start_x: Optional[float] = None
        self.sandpit_phase_start_y: Optional[float] = None
        self.sandpit_phase_start_yaw: Optional[float] = None
        self.sandpit_phase_start_sec: float = 0.0
        self.sandpit_entry_yaw: Optional[float] = None
        self.sandpit_phase_target_yaw: Optional[float] = None
        self.sandpit_phase_target_distance_m: Optional[float] = None
        self.sandpit_phase_deadline_sec: Optional[float] = None
        self.sandpit_phase_stable_since: Optional[float] = None
        self.sandpit_phase_last_log_sec: float = 0.0
        self.sandpit_phase_turn_direction_sign: int = 0
        self.sandpit_phase_turn_last_yaw: Optional[float] = None
        self.sandpit_phase_turn_progress_rad: float = 0.0
        self.sandpit_phase_turn_target_delta_rad: Optional[float] = None
        self.sandpit_phase_walk_yaw_guard_active = False
        self.sandpit_phase_wait_for_finish = False
        self.sandpit_jumping_signal_received = False
        self.sandpit_jumping_signal_stamp_sec = 0.0
        self.sandpit_finish_signal_received = False
        self.sandpit_finish_signal_stamp_sec = 0.0
        self.sandpit_last_step_once_command: Optional[tuple[int, float, float]] = None
        self.sandpit_last_step_once_yaw: Optional[float] = None
        self.sandpit_wait_finish_resend_after_sec = 0.0
        self.sandpit_wait_finish_next_resend_sec: Optional[float] = None
        self.sandpit_wait_finish_resend_count = 0
        self.sandpit_wait_finish_resend_max_count = 0
        self.sandpit_wait_finish_serial_disconnect_active = False
        self.sandpit_wait_finish_serial_disconnect_stamp_sec = 0.0
        self.sandpit_wait_finish_serial_disconnect_had_jumping = False
        self.serial_connected = True
        self.serial_connected_stamp_sec = 0.0
        self.active_bypass_kind = 'sandpit'
        self.pending_limit_bar_reason = ''
        self.pending_limit_bar_yaw_ref_text = 'unknown'
        self.last_duck_progress_log_time = 0.0
        self.phase_deadline_sec: Optional[float] = None
        self.pending_hurdle_jump_once_repeats = 0
        self.next_hurdle_jump_once_repeat_sec = 0.0

        self.last_state = ''
        self.last_feedback = ''
        self.last_step_override = ''
        self.current_active_branch = ''
        self.start_signal_received = False
        self.start_retry_guard_deadline_sec = 0.0
        self.slope_upstairs_wait_start_sec = 0.0
        self.last_slope_upstairs_wait_log_sec = 0.0

        self.step_override_pub = self.create_publisher(
            Float32MultiArray, self.step_override_topic, 10
        )
        self.step_once_pub = self.create_publisher(
            Float32MultiArray, self.step_once_topic, 10
        )
        self.arrival_pub = self.create_publisher(Bool, self.arrival_status_topic, 10)
        apriltag_tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=self.apriltag_tf_queue_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.hurdle_tf_buffer: Optional[Buffer] = None
        self.hurdle_tf_listener: Optional[TransformListener] = None
        if self.hurdle_tf_buffer_fallback_enabled:
            self.hurdle_tf_buffer = Buffer()
            self.hurdle_tf_listener = TransformListener(
                self.hurdle_tf_buffer,
                self,
                spin_thread=False,
                qos=apriltag_tf_qos,
            )
        self.apriltag_tf_sub = self.create_subscription(
            TFMessage,
            self.apriltag_tf_topic,
            self._apriltag_tf_callback,
            apriltag_tf_qos,
        )
        self.apriltag_detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.apriltag_detections_topic,
            self._apriltag_detections_callback,
            qos_profile_sensor_data,
        )
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(String, self.state_topic, event_qos)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, event_qos)
        self.stairs_active_pub = self.create_publisher(Bool, self.stairs_active_topic, 10)
        self.slope_active_pub = self.create_publisher(Bool, self.slope_active_topic, 10)
        self.pole_active_pub = self.create_publisher(Bool, self.pole_active_topic, 10)
        reference_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.slope_align_reference_pub = self.create_publisher(
            PoseStamped,
            self.slope_align_reference_topic,
            reference_qos,
        )
        self.slope_route_yaw_reference_pub = self.create_publisher(
            PoseStamped,
            self.slope_route_yaw_reference_topic,
            reference_qos,
        )
        self.slope_bridge_retry_yaw_reference_pub = self.create_publisher(
            PoseStamped,
            self.slope_bridge_retry_yaw_reference_topic,
            reference_qos,
        )

        self.pose_sub = self.create_subscription(
            Odometry,
            self.pose_topic,
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self.limit_trigger_sub = self.create_subscription(
            Bool, self.limit_trigger_topic, self._limit_trigger_callback, 10
        )
        self.limit_nearest_sub = self.create_subscription(
            Float32MultiArray, self.limit_nearest_topic, self._limit_nearest_callback, 10
        )
        self.hurdle_trigger_sub = self.create_subscription(
            Bool, self.hurdle_trigger_topic, self._hurdle_trigger_callback, 10
        )
        self.hurdle_nearest_sub = self.create_subscription(
            Float32MultiArray, self.hurdle_nearest_topic, self._hurdle_nearest_callback, 10
        )
        self.pole_trigger_sub = self.create_subscription(
            Bool, self.pole_trigger_topic, self._pole_trigger_callback, 10
        )
        self.pole_nearest_sub = self.create_subscription(
            Float32MultiArray, self.pole_nearest_topic, self._pole_nearest_callback, 10
        )
        self.slope_trigger_sub = self.create_subscription(
            Bool, self.slope_trigger_topic, self._slope_trigger_callback, 10
        )
        self.slope_nearest_sub = self.create_subscription(
            Float32MultiArray, self.slope_nearest_topic, self._slope_nearest_callback, 10
        )
        self.upstairs_trigger_sub = self.create_subscription(
            Bool, self.upstairs_trigger_topic, self._upstairs_trigger_callback, 10
        )
        self.upstairs_nearest_sub = self.create_subscription(
            Float32MultiArray, self.upstairs_nearest_topic, self._upstairs_nearest_callback, 10
        )
        self.stairs_state_sub = self.create_subscription(
            String, self.stairs_state_topic, self._stairs_state_callback, 10
        )
        self.slope_state_sub = self.create_subscription(
            String, self.slope_state_topic, self._slope_state_callback, 10
        )
        self.slope_current_waypoint_sub = self.create_subscription(
            String,
            self.slope_current_waypoint_topic,
            self._slope_current_waypoint_callback,
            10,
        )
        self.pole_state_sub = self.create_subscription(
            String, self.pole_state_topic, self._pole_state_callback, 10
        )
        self.pole_feedback_sub = self.create_subscription(
            String, self.pole_feedback_topic, self._pole_feedback_callback, 10
        )
        start_signal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.start_signal_sub = self.create_subscription(
            Bool,
            self.start_signal_topic,
            self._start_signal_callback,
            start_signal_qos,
        )
        self.finish_signal_sub = self.create_subscription(
            Bool,
            self.finish_signal_topic,
            self._finish_signal_callback,
            10,
        )
        self.action_ack_sub = self.create_subscription(
            String,
            self.action_ack_topic,
            self._action_ack_callback,
            10,
        )
        self.serial_connected_sub = self.create_subscription(
            Bool,
            self.serial_connected_topic,
            self._serial_connected_callback,
            10,
        )
        self.slope_retry_restart_pub = self.create_publisher(
            String,
            self.slope_retry_restart_from_waypoint_topic,
            10,
        )

        now = self._now_sec()
        if self.wait_for_start_signal:
            self.state = self.STATE_WAIT_START
            self.current_active_branch = ''
            self._publish_state(self.state)
            self._publish_feedback('初始化完成，等待电控串口 [start] 信号后开始下楼梯流程。')
            self._publish_active_flags()
        else:
            self._enter_started_flow(now, reason='startup', clear_wait_override=False)

        self.control_timer = self.create_timer(1.0 / self.search_publish_hz, self._control_loop)

    def _configure_sandpit_bypass(self) -> None:
        config = self._load_sandpit_bypass_config(self.sandpit_bypass_file)
        apriltag_config = self._dict_value(config, 'apriltag', 'sandpit_bypass')
        sequence_config = self._dict_value(config, 'sequence', 'sandpit_bypass')
        turn_config = self._dict_value(config, 'turn', 'sandpit_bypass')
        walk_config = self._dict_value(config, 'walk', 'sandpit_bypass')
        traverse_config = self._dict_value(config, 'traverse', 'sandpit_bypass')
        pre_align_config = self._dict_value(config, 'pre_align', 'sandpit_bypass')

        self.sandpit_bypass_enabled = bool(config.get('enabled', False))
        self.sandpit_behavior = str(config.get('behavior', 'bypass')).strip().lower()
        if self.sandpit_behavior not in ('bypass', 'traverse'):
            raise RuntimeError('sandpit_bypass.behavior must be "bypass" or "traverse".')
        self.sandpit_bypass_complete_once = bool(config.get('complete_once', True))
        self.sandpit_search_rearm_delay_sec = max(
            0.0,
            float(config.get('search_rearm_delay_sec', self.search_rearm_delay_sec)),
        )
        self.sandpit_progress_log_interval_sec = max(
            0.0,
            float(config.get('progress_log_interval_sec', 0.5)),
        )

        self.sandpit_tag_frame_id = str(
            apriltag_config.get('frame_id', 'camera_color_optical_frame')
        )
        self.sandpit_tag_child_frame_id = str(apriltag_config.get('child_frame_id', 'object'))
        self.sandpit_tag_distance_axis = str(
            apriltag_config.get('distance_axis', 'z')
        ).strip().lower()
        if self.sandpit_tag_distance_axis not in ('x', 'y', 'z', 'euclidean'):
            raise RuntimeError(
                'sandpit_bypass.apriltag.distance_axis must be x, y, z, or euclidean.'
            )
        self.sandpit_trigger_distance_m = max(
            0.0,
            float(apriltag_config.get('trigger_distance_m', 0.45)),
        )
        self.sandpit_trigger_stale_timeout_sec = max(
            0.05,
            float(apriltag_config.get('stale_timeout_sec', 0.80)),
        )
        self.sandpit_trigger_min_duration_sec = max(
            0.0,
            float(apriltag_config.get('min_duration_sec', 0.0)),
        )
        self.sandpit_tag_min_samples = max(
            1,
            int(apriltag_config.get('min_samples', 2)),
        )
        self.sandpit_tag_max_sample_age_sec = max(
            0.0,
            float(apriltag_config.get('max_sample_age_sec', 0.80)),
        )
        self.sandpit_tag_target_id = int(apriltag_config.get('target_tag_id', 10))
        self.sandpit_tag_detection_guard_sec = max(
            0.0,
            float(
                apriltag_config.get(
                    'detection_guard_sec',
                    self.sandpit_trigger_stale_timeout_sec,
                )
            ),
        )
        self.sandpit_apriltag_priority_guard_enabled = bool(
            apriltag_config.get('priority_guard_enabled', True)
        )
        self.sandpit_apriltag_priority_distance_m = max(
            self.sandpit_trigger_distance_m,
            float(
                apriltag_config.get(
                    'priority_distance_m',
                    self.sandpit_trigger_distance_m,
                )
            ),
        )
        self.sandpit_apriltag_priority_guard_hold_sec = max(
            self.sandpit_trigger_stale_timeout_sec,
            float(
                apriltag_config.get(
                    'priority_guard_hold_sec',
                    max(2.5, self.sandpit_tag_detection_guard_sec),
                )
            ),
        )
        self.sandpit_pre_align_enabled = bool(pre_align_config.get('enabled', False))
        self.sandpit_pre_align_start_distance_m = max(
            self.sandpit_trigger_distance_m,
            float(pre_align_config.get('start_distance_m', 1.4)),
        )
        self.sandpit_pre_align_handoff_distance_m = max(
            0.0,
            min(
                self.sandpit_pre_align_start_distance_m,
                float(
                    pre_align_config.get(
                        'handoff_distance_m',
                        self.sandpit_trigger_distance_m,
                    )
                ),
            ),
        )
        self.sandpit_pre_align_min_samples = max(
            1,
            int(pre_align_config.get('min_samples', 3)),
        )
        self.sandpit_pre_align_target_tag_x_m = float(
            pre_align_config.get('target_tag_x_m', 0.0)
        )
        self.sandpit_pre_align_lateral_skip_below_m = max(
            0.0,
            float(pre_align_config.get('lateral_skip_below_m', 0.0)),
        )
        self.sandpit_pre_align_deadband_m = max(
            0.0,
            float(pre_align_config.get('deadband_m', 0.03)),
        )
        self.sandpit_pre_align_mode = int(pre_align_config.get('mode', self.search_mode))
        self.sandpit_pre_align_left_norm = self._clamp_norm(
            float(pre_align_config.get('left_norm', self.search_left_norm))
        )
        self.sandpit_pre_align_right_norm = self._clamp_norm(
            float(pre_align_config.get('right_norm', self.search_right_norm))
        )
        self.sandpit_pre_align_hold_mode = int(pre_align_config.get('hold_mode', 0))
        self.sandpit_pre_align_hold_left_norm = self._clamp_norm(
            float(pre_align_config.get('hold_left_norm', 0.0))
        )
        self.sandpit_pre_align_hold_right_norm = self._clamp_norm(
            float(pre_align_config.get('hold_right_norm', 0.0))
        )
        self.sandpit_pre_align_yaw_center_enabled = bool(
            pre_align_config.get('yaw_center_enabled', False)
        )
        self.sandpit_pre_align_yaw_pose_axis = str(
            pre_align_config.get('yaw_pose_axis', 'pose_pitch')
        ).strip().lower()
        if self.sandpit_pre_align_yaw_pose_axis not in (
            'pose_roll',
            'pose_pitch',
            'pose_yaw',
        ):
            raise RuntimeError(
                'sandpit_bypass.pre_align.yaw_pose_axis must be '
                'pose_roll, pose_pitch, or pose_yaw.'
            )
        self.sandpit_pre_align_yaw_pose_target_rad = math.radians(
            float(pre_align_config.get('yaw_pose_target_deg', 0.0))
        )
        pose_control_sign = float(pre_align_config.get('yaw_pose_control_sign', -1.0))
        self.sandpit_pre_align_yaw_pose_control_sign = (
            -1.0 if pose_control_sign < 0.0 else 1.0
        )
        self.sandpit_pre_align_yaw_tolerance_rad = math.radians(
            max(0.0, float(pre_align_config.get('yaw_tolerance_deg', 4.0)))
        )
        self.sandpit_pre_align_yaw_timeout_sec = max(
            0.0,
            float(pre_align_config.get('yaw_timeout_sec', 0.0)),
        )
        self.sandpit_pre_align_yaw_max_angle_rad = math.radians(
            max(0.0, float(pre_align_config.get('yaw_max_angle_deg', 35.0)))
        )
        self.sandpit_pre_align_yaw_turn_mode = int(
            pre_align_config.get('yaw_turn_mode', 6)
        )
        self.sandpit_pre_align_yaw_cw_left_norm = self._clamp_norm(
            float(pre_align_config.get('yaw_cw_left_norm', 0.20))
        )
        self.sandpit_pre_align_yaw_cw_right_norm = self._clamp_norm(
            float(pre_align_config.get('yaw_cw_right_norm', -0.08))
        )
        self.sandpit_pre_align_yaw_ccw_left_norm = self._clamp_norm(
            float(pre_align_config.get('yaw_ccw_left_norm', -0.08))
        )
        self.sandpit_pre_align_yaw_ccw_right_norm = self._clamp_norm(
            float(pre_align_config.get('yaw_ccw_right_norm', 0.20))
        )
        self.sandpit_pre_align_polyline_enabled = bool(
            pre_align_config.get('polyline_enabled', False)
        )
        self.sandpit_pre_align_polyline_angle_rad = math.radians(
            max(0.0, float(pre_align_config.get('polyline_angle_deg', 25.0)))
        )
        self.sandpit_pre_align_polyline_large_angle_rad = math.radians(
            max(
                0.0,
                float(
                    pre_align_config.get(
                        'polyline_large_angle_deg',
                        math.degrees(self.sandpit_pre_align_polyline_angle_rad),
                    )
                ),
            )
        )
        self.sandpit_pre_align_polyline_large_angle_error_m = max(
            0.0,
            float(pre_align_config.get('polyline_large_angle_error_m', 0.16)),
        )
        self.sandpit_pre_align_polyline_large_angle_center_rad = math.radians(
            max(0.0, float(pre_align_config.get('polyline_large_angle_center_deg', 10.0)))
        )
        self.sandpit_pre_align_polyline_distance_scale = max(
            0.0,
            float(pre_align_config.get('polyline_distance_scale', 1.20)),
        )
        self.sandpit_pre_align_polyline_min_distance_m = max(
            0.0,
            float(pre_align_config.get('polyline_min_distance_m', 0.08)),
        )
        self.sandpit_pre_align_polyline_max_distance_m = max(
            0.0,
            float(pre_align_config.get('polyline_max_distance_m', 0.35)),
        )
        self.sandpit_pre_align_polyline_turn_tolerance_rad = math.radians(
            max(0.0, float(pre_align_config.get('polyline_turn_tolerance_deg', 3.0)))
        )
        self.sandpit_pre_align_polyline_turn_timeout_sec = max(
            0.0,
            float(pre_align_config.get('polyline_turn_timeout_sec', 1.8)),
        )
        self.sandpit_pre_align_polyline_turn_fine_threshold_rad = math.radians(
            max(0.0, float(pre_align_config.get('polyline_turn_fine_threshold_deg', 25.0)))
        )
        self.sandpit_pre_align_polyline_turn_min_scale = max(
            0.05,
            min(1.0, float(pre_align_config.get('polyline_turn_min_scale', 0.35))),
        )
        self.sandpit_pre_align_polyline_walk_timeout_sec_per_m = max(
            0.0,
            float(pre_align_config.get('polyline_walk_timeout_sec_per_m', 5.0)),
        )
        self.sandpit_pre_align_polyline_walk_timeout_extra_sec = max(
            0.0,
            float(pre_align_config.get('polyline_walk_timeout_extra_sec', 1.0)),
        )
        self.sandpit_pre_align_polyline_walk_left_norm = self._clamp_norm(
            float(
                pre_align_config.get(
                    'polyline_walk_left_norm',
                    self.sandpit_pre_align_left_norm,
                )
            )
        )
        self.sandpit_pre_align_polyline_walk_right_norm = self._clamp_norm(
            float(
                pre_align_config.get(
                    'polyline_walk_right_norm',
                    self.sandpit_pre_align_right_norm,
                )
            )
        )
        self.sandpit_pre_align_polyline_shift_yaw_guard_rad = math.radians(
            max(0.0, float(pre_align_config.get('polyline_shift_yaw_guard_deg', 8.0)))
        )
        self.sandpit_pre_align_polyline_yaw_correction_enabled = bool(
            pre_align_config.get('polyline_yaw_correction_enabled', True)
        )
        self.sandpit_pre_align_polyline_yaw_gain_per_rad = max(
            0.0,
            float(pre_align_config.get('polyline_yaw_gain_per_rad', 0.35)),
        )
        self.sandpit_pre_align_polyline_yaw_max_delta_norm = max(
            0.0,
            min(1.0, float(pre_align_config.get('polyline_yaw_max_delta_norm', 0.16))),
        )
        self.sandpit_pre_align_backup_enabled = bool(
            pre_align_config.get('backup_enabled', False)
        )
        self.sandpit_pre_align_backup_trigger_distance_m = max(
            0.0,
            float(pre_align_config.get('backup_trigger_distance_m', 0.75)),
        )
        self.sandpit_pre_align_backup_release_distance_m = max(
            self.sandpit_pre_align_backup_trigger_distance_m,
            float(pre_align_config.get('backup_release_distance_m', 0.88)),
        )
        self.sandpit_pre_align_backup_max_distance_m = max(
            0.0,
            float(pre_align_config.get('backup_max_distance_m', 0.20)),
        )
        self.sandpit_pre_align_backup_force_distance_m = max(
            0.0,
            float(pre_align_config.get('backup_force_distance_m', 0.62)),
        )
        self.sandpit_pre_align_backup_skip_lateral_error_m = max(
            0.0,
            float(pre_align_config.get('backup_skip_lateral_error_m', 0.16)),
        )
        self.sandpit_pre_align_backup_skip_center_angle_rad = math.radians(
            max(0.0, float(pre_align_config.get('backup_skip_center_angle_deg', 10.0)))
        )
        self.sandpit_pre_align_backup_min_polyline_clearance_m = max(
            0.0,
            float(pre_align_config.get('backup_min_polyline_clearance_m', 0.18)),
        )
        self.sandpit_pre_align_backup_mode = int(
            pre_align_config.get('backup_mode', self.sandpit_pre_align_mode)
        )
        self.sandpit_pre_align_backup_left_norm = self._clamp_norm(
            float(pre_align_config.get('backup_left_norm', -0.25))
        )
        self.sandpit_pre_align_backup_right_norm = self._clamp_norm(
            float(pre_align_config.get('backup_right_norm', -0.25))
        )
        self.sandpit_pre_align_lateral_gain_norm_per_m = max(
            0.0,
            float(pre_align_config.get('lateral_gain_norm_per_m', 0.55)),
        )
        self.sandpit_pre_align_lateral_max_delta_norm = max(
            0.0,
            min(1.0, float(pre_align_config.get('lateral_max_delta_norm', 0.12))),
        )
        lateral_sign = float(pre_align_config.get('lateral_sign', -1.0))
        self.sandpit_pre_align_lateral_sign = -1.0 if lateral_sign < 0.0 else 1.0
        self.sandpit_pre_align_log_interval_sec = max(
            0.0,
            float(
                pre_align_config.get(
                    'log_interval_sec',
                    self.sandpit_progress_log_interval_sec,
                )
            ),
        )

        self.sandpit_turn_sequence_deg = self._float_sequence(
            sequence_config.get('turn_sequence_deg', [-90.0, 90.0, -90.0]),
            3,
            'sandpit_bypass.sequence.turn_sequence_deg',
        )
        self.sandpit_straight_distances_m = self._float_sequence(
            sequence_config.get('straight_distances_m', [1.0, 2.0]),
            2,
            'sandpit_bypass.sequence.straight_distances_m',
        )

        self.sandpit_turn_mode = int(turn_config.get('mode', 6))
        self.sandpit_turn_cw_left_norm = float(turn_config.get('cw_left_norm', 0.60))
        self.sandpit_turn_cw_right_norm = float(turn_config.get('cw_right_norm', -0.30))
        self.sandpit_turn_ccw_left_norm = float(turn_config.get('ccw_left_norm', -0.35))
        self.sandpit_turn_ccw_right_norm = float(turn_config.get('ccw_right_norm', 0.60))
        self.sandpit_turn_hold_mode = int(turn_config.get('hold_mode', 0))
        self.sandpit_turn_hold_left_norm = float(turn_config.get('hold_left_norm', 0.0))
        self.sandpit_turn_hold_right_norm = float(turn_config.get('hold_right_norm', 0.0))
        self.sandpit_turn_tolerance_rad = math.radians(
            max(0.0, float(turn_config.get('yaw_tolerance_deg', 3.0)))
        )
        self.sandpit_turn_progress_tolerance_rad = math.radians(
            max(0.0, float(turn_config.get('progress_tolerance_deg', 3.0)))
        )
        self.sandpit_turn_exit_hold_sec = max(
            0.0,
            float(turn_config.get('exit_hold_sec', 0.10)),
        )
        self.sandpit_turn_timeout_sec = max(
            0.0,
            float(turn_config.get('timeout_sec', 8.0)),
        )

        self.sandpit_walk_mode = int(walk_config.get('mode', self.search_mode))
        self.sandpit_walk_left_norm = float(walk_config.get('left_norm', self.search_left_norm))
        self.sandpit_walk_right_norm = float(walk_config.get('right_norm', self.search_right_norm))
        self.sandpit_walk_timeout_sec_per_m = max(
            0.0,
            float(walk_config.get('timeout_sec_per_m', 8.0)),
        )
        self.sandpit_walk_timeout_extra_sec = max(
            0.0,
            float(walk_config.get('timeout_extra_sec', 3.0)),
        )
        self.sandpit_walk_yaw_correction_enabled = bool(
            walk_config.get('yaw_correction_enabled', True)
        )
        self.sandpit_walk_yaw_tolerance_rad = math.radians(
            max(0.0, float(walk_config.get('yaw_tolerance_deg', 3.0)))
        )
        self.sandpit_walk_yaw_gain_per_rad = max(
            0.0,
            float(walk_config.get('yaw_gain_per_rad', 0.35)),
        )
        self.sandpit_walk_yaw_max_delta_norm = max(
            0.0,
            float(walk_config.get('yaw_max_delta_norm', 0.20)),
        )

        traverse_phases = traverse_config.get('phases', [])
        if traverse_phases is None:
            traverse_phases = []
        if not isinstance(traverse_phases, list):
            raise RuntimeError('sandpit_bypass.traverse.phases must be a YAML list.')
        self.sandpit_traverse_phases: List[Dict[str, Any]] = []
        for index, phase in enumerate(traverse_phases, start=1):
            if not isinstance(phase, dict):
                raise RuntimeError(
                    f'sandpit_bypass.traverse.phases[{index}] must be a YAML mapping.'
                )
            phase_copy = dict(phase)
            kind = str(phase_copy.get('kind', '')).strip().lower()
            if kind not in ('align', 'walk', 'turn', 'wait', 'step_once', 'wait_finish'):
                raise RuntimeError(
                    f'sandpit_bypass.traverse.phases[{index}].kind is unsupported: {kind}'
                )
            phase_copy['kind'] = kind
            self.sandpit_traverse_phases.append(phase_copy)
        if self.sandpit_behavior == 'traverse' and not self.sandpit_traverse_phases:
            raise RuntimeError(
                'sandpit_bypass.behavior is "traverse" but traverse.phases is empty.'
            )

    def _configure_pole_bypass(self) -> None:
        config = self._load_pole_bypass_config(self.pole_bypass_file)
        sequence_config = self._dict_value(config, 'sequence', 'pole_bypass')
        turn_config = self._dict_value(config, 'turn', 'pole_bypass')
        walk_config = self._dict_value(config, 'walk', 'pole_bypass')

        self.pole_bypass_enabled = bool(config.get('enabled', self.pole_bypass_enabled))
        self.pole_bypass_force_on_detection = bool(
            config.get('force_on_detection', self.pole_bypass_force_on_detection)
        )
        self.pole_bypass_pre_bypass_waypoints = max(
            0,
            int(
                config.get(
                    'pre_bypass_waypoints',
                    self.pole_bypass_pre_bypass_waypoints,
                )
            ),
        )
        self.pole_bypass_complete_once = bool(
            config.get('complete_once', self.pole_bypass_complete_once)
        )
        self.pole_bypass_search_rearm_delay_sec = max(
            0.0,
            float(
                config.get(
                    'search_rearm_delay_sec',
                    self.pole_bypass_search_rearm_delay_sec,
                )
            ),
        )
        self.pole_bypass_progress_log_interval_sec = max(
            0.0,
            float(config.get('progress_log_interval_sec', 0.5)),
        )

        self.pole_bypass_turn_sequence_deg = self._float_sequence(
            sequence_config.get('turn_sequence_deg', [90.0, -90.0, 90.0]),
            3,
            'pole_bypass.sequence.turn_sequence_deg',
        )
        self.pole_bypass_straight_distances_m = self._float_sequence(
            sequence_config.get('straight_distances_m', [0.8, 1.2]),
            2,
            'pole_bypass.sequence.straight_distances_m',
        )

        self.pole_bypass_turn_mode = int(turn_config.get('mode', self.sandpit_turn_mode))
        self.pole_bypass_turn_cw_left_norm = float(
            turn_config.get('cw_left_norm', self.sandpit_turn_cw_left_norm)
        )
        self.pole_bypass_turn_cw_right_norm = float(
            turn_config.get('cw_right_norm', self.sandpit_turn_cw_right_norm)
        )
        self.pole_bypass_turn_ccw_left_norm = float(
            turn_config.get('ccw_left_norm', self.sandpit_turn_ccw_left_norm)
        )
        self.pole_bypass_turn_ccw_right_norm = float(
            turn_config.get('ccw_right_norm', self.sandpit_turn_ccw_right_norm)
        )
        self.pole_bypass_turn_hold_mode = int(
            turn_config.get('hold_mode', self.sandpit_turn_hold_mode)
        )
        self.pole_bypass_turn_hold_left_norm = float(
            turn_config.get('hold_left_norm', self.sandpit_turn_hold_left_norm)
        )
        self.pole_bypass_turn_hold_right_norm = float(
            turn_config.get('hold_right_norm', self.sandpit_turn_hold_right_norm)
        )
        self.pole_bypass_turn_tolerance_rad = math.radians(
            max(0.0, float(turn_config.get('yaw_tolerance_deg', 3.0)))
        )
        self.pole_bypass_turn_progress_tolerance_rad = math.radians(
            max(0.0, float(turn_config.get('progress_tolerance_deg', 3.0)))
        )
        self.pole_bypass_turn_exit_hold_sec = max(
            0.0,
            float(turn_config.get('exit_hold_sec', 0.10)),
        )
        self.pole_bypass_turn_timeout_sec = max(
            0.0,
            float(turn_config.get('timeout_sec', 1.0)),
        )

        self.pole_bypass_walk_mode = int(walk_config.get('mode', self.search_mode))
        self.pole_bypass_walk_left_norm = float(
            walk_config.get('left_norm', self.search_left_norm)
        )
        self.pole_bypass_walk_right_norm = float(
            walk_config.get('right_norm', self.search_right_norm)
        )
        self.pole_bypass_walk_timeout_sec_per_m = max(
            0.0,
            float(walk_config.get('timeout_sec_per_m', 8.0)),
        )
        self.pole_bypass_walk_timeout_extra_sec = max(
            0.0,
            float(walk_config.get('timeout_extra_sec', 3.0)),
        )
        self.pole_bypass_walk_yaw_correction_enabled = bool(
            walk_config.get('yaw_correction_enabled', True)
        )
        self.pole_bypass_walk_yaw_tolerance_rad = math.radians(
            max(0.0, float(walk_config.get('yaw_tolerance_deg', 3.0)))
        )
        self.pole_bypass_walk_yaw_gain_per_rad = max(
            0.0,
            float(walk_config.get('yaw_gain_per_rad', 0.35)),
        )
        self.pole_bypass_walk_yaw_max_delta_norm = max(
            0.0,
            float(walk_config.get('yaw_max_delta_norm', 0.20)),
        )

    def _load_sandpit_bypass_config(self, path_text: str) -> Dict[str, Any]:
        path_text = str(path_text).strip()
        if not path_text:
            return {}
        path = Path(path_text).expanduser()
        if not path.exists():
            raise RuntimeError(f'sandpit_bypass_file does not exist: {path}')
        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f'sandpit_bypass_file must contain a YAML mapping: {path}')
        section = data.get('sandpit_bypass', {})
        if section is None:
            return {}
        if not isinstance(section, dict):
            raise RuntimeError('sandpit_bypass section must be a YAML mapping.')
        return section

    def _load_pole_bypass_config(self, path_text: str) -> Dict[str, Any]:
        path_text = str(path_text).strip()
        if not path_text:
            return {}
        path = Path(path_text).expanduser()
        if not path.exists():
            raise RuntimeError(f'pole_bypass_file does not exist: {path}')
        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f'pole_bypass_file must contain a YAML mapping: {path}')

        section = data.get('pole_bypass')
        if section is None:
            nav_section = data.get('orange_pole_relative_nav', {})
            if isinstance(nav_section, dict):
                ros_params = nav_section.get('ros__parameters', {})
                if isinstance(ros_params, dict):
                    section = ros_params.get('pole_bypass')
        if section is None:
            return {}
        if not isinstance(section, dict):
            raise RuntimeError('pole_bypass section must be a YAML mapping.')
        return section

    @staticmethod
    def _dict_value(parent: Dict[str, Any], key: str, prefix: str) -> Dict[str, Any]:
        value = parent.get(key, {})
        if value is None:
            return {}
        if not isinstance(value, dict):
            raise RuntimeError(f'{prefix}.{key} must be a YAML mapping.')
        return value

    @staticmethod
    def _float_sequence(value: Any, expected_len: int, label: str) -> List[float]:
        if not isinstance(value, list) or len(value) != expected_len:
            raise RuntimeError(f'{label} must be a list with {expected_len} numbers.')
        result = [float(item) for item in value]
        for item in result:
            if not math.isfinite(item):
                raise RuntimeError(f'{label} contains a non-finite number.')
        return result

    def _pose_callback(self, msg: Odometry) -> None:
        now = self._now_sec()
        self.pose_x = float(msg.pose.pose.position.x)
        self.pose_y = float(msg.pose.pose.position.y)
        roll_rad, pitch_rad, yaw_rad = self._rpy_from_quaternion(
            msg.pose.pose.orientation
        )
        self._update_pose_yaw_rate(yaw_rad, now)
        self.pose_yaw = yaw_rad
        self.pose_roll = roll_rad
        self.pose_pitch = pitch_rad
        self.pose_frame = str(msg.header.frame_id)
        self.pose_stamp_sec = now
        if self.startup_pose_yaw is None:
            self.startup_pose_yaw = self.pose_yaw
        if self.state == self.STATE_SEARCH and self.search_start_x is None:
            self.search_start_x = self.pose_x
            self.search_start_y = self.pose_y

    def _update_pose_yaw_rate(self, yaw_rad: float, stamp_sec: float) -> None:
        (
            self.pose_yaw_rate_rad_s,
            self.pose_yaw_rate_raw_rad_s,
        ) = self.pose_yaw_rate_estimator.update(yaw_rad, stamp_sec)

    def _limit_trigger_callback(self, msg: Bool) -> None:
        self._update_detection_trigger(self.limit_detection, bool(msg.data))

    def _limit_nearest_callback(self, msg: Float32MultiArray) -> None:
        self.limit_detection.distance_m = self._distance_from_array(msg)
        self.limit_detection.lateral_m = self._value_from_array(msg, 1)
        self.limit_detection.nearest_stamp_sec = self._now_sec()

    def _hurdle_trigger_callback(self, msg: Bool) -> None:
        triggered = bool(msg.data)
        self._update_detection_trigger(
            self.hurdle_visual_detection,
            triggered,
        )
        if not triggered:
            self._reset_hurdle_lateral_s_curve_visual_confirmation()

    def _hurdle_nearest_callback(self, msg: Float32MultiArray) -> None:
        now = self._now_sec()
        distance_m = self._distance_from_array(msg)
        self.hurdle_visual_detection.distance_m = distance_m
        self.hurdle_visual_detection.lateral_m = self._value_from_array(msg, 1)
        self.hurdle_visual_detection.nearest_stamp_sec = now
        if (
            distance_m is not None
            and distance_m
            <= self.hurdle_lateral_shift_visual_jump_distance_m
        ):
            if getattr(
                self,
                'hurdle_lateral_s_curve_visual_close_start_sec',
                None,
            ) is None:
                self.hurdle_lateral_s_curve_visual_close_start_sec = now
                self.hurdle_lateral_s_curve_visual_close_sample_count = 1
            else:
                self.hurdle_lateral_s_curve_visual_close_sample_count = (
                    getattr(
                        self,
                        'hurdle_lateral_s_curve_visual_close_sample_count',
                        0,
                    )
                    + 1
                )
        else:
            self._reset_hurdle_lateral_s_curve_visual_confirmation()

    def _pole_trigger_callback(self, msg: Bool) -> None:
        if not self.pole_visual_trigger_enabled:
            self._update_detection_trigger(self.pole_detection, False)
            return
        self._update_detection_trigger(self.pole_detection, bool(msg.data))

    def _pole_nearest_callback(self, msg: Float32MultiArray) -> None:
        self.pole_detection.distance_m = self._distance_from_array(msg)
        self.pole_detection.lateral_m = self._value_from_array(msg, 1)
        self.pole_detection.nearest_stamp_sec = self._now_sec()

    def _slope_trigger_callback(self, msg: Bool) -> None:
        self._update_detection_trigger(self.slope_detection, bool(msg.data))

    def _slope_nearest_callback(self, msg: Float32MultiArray) -> None:
        self.slope_detection.distance_m = self._distance_from_array(msg)
        self.slope_detection.nearest_stamp_sec = self._now_sec()

    def _upstairs_trigger_callback(self, msg: Bool) -> None:
        self._update_detection_trigger(self.upstairs_detection, bool(msg.data))
        self._update_detection_trigger(self.upstairs_visual_detection, bool(msg.data))

    def _upstairs_nearest_callback(self, msg: Float32MultiArray) -> None:
        distance_m = self._distance_from_array(msg)
        lateral_m = self._value_from_array(msg, 1)
        now = self._now_sec()
        self.upstairs_detection.distance_m = distance_m
        self.upstairs_detection.lateral_m = lateral_m
        self.upstairs_detection.nearest_stamp_sec = now
        self.upstairs_visual_detection.distance_m = distance_m
        self.upstairs_visual_detection.lateral_m = lateral_m
        self.upstairs_visual_detection.nearest_stamp_sec = now

    def _apriltag_detections_callback(self, msg: AprilTagDetectionArray) -> None:
        if not (
            self.pole_apriltag_trigger_enabled
            or self.limit_bar_tag_trigger_enabled
            or self.hurdle_tag_trigger_enabled
            or self.upstairs_tag_trigger_enabled
            or self.sandpit_bypass_enabled
        ):
            return
        pole_matched = False
        limit_bar_matched = False
        hurdle_matched = False
        upstairs_matched = False
        sandpit_matched = False
        seen_ids = []
        for detection in msg.detections:
            try:
                tag_id = int(detection.id)
            except (TypeError, ValueError):
                continue
            seen_ids.append(tag_id)
            if tag_id == self.pole_apriltag_target_tag_id:
                pole_matched = True
            if tag_id == self.limit_bar_tag_target_tag_id:
                limit_bar_matched = True
            if tag_id == self.hurdle_tag_target_tag_id:
                hurdle_matched = True
            if tag_id == self.upstairs_tag_target_tag_id:
                upstairs_matched = True
            if tag_id == self.sandpit_tag_target_id:
                sandpit_matched = True
        if self.sandpit_bypass_enabled:
            if sandpit_matched:
                self.sandpit_tag_detection_stamp_sec = self._now_sec()
            if seen_ids:
                self.sandpit_tag_detection_ids_text = ','.join(str(tag_id) for tag_id in seen_ids)
            elif not sandpit_matched:
                self.sandpit_tag_detection_ids_text = 'none'
        if self.pole_apriltag_trigger_enabled:
            now = self._now_sec()
            if pole_matched:
                self.pole_apriltag_last_seen_stamp_sec = now
                self._update_detection_trigger(self.pole_apriltag_detection, True)
            else:
                latch_active = (
                    self.pole_apriltag_detection_latch_sec > 0.0
                    and self.pole_apriltag_last_seen_stamp_sec > 0.0
                    and (now - self.pole_apriltag_last_seen_stamp_sec)
                    <= self.pole_apriltag_detection_latch_sec
                )
                if not latch_active:
                    self._update_detection_trigger(self.pole_apriltag_detection, False)
        if self.limit_bar_tag_trigger_enabled:
            if limit_bar_matched:
                self.limit_bar_tag_detection_stamp_sec = self._now_sec()
            if seen_ids:
                self.limit_bar_tag_detection_ids_text = ','.join(str(tag_id) for tag_id in seen_ids)
            elif not limit_bar_matched:
                self.limit_bar_tag_detection_ids_text = 'none'
        if self.hurdle_tag_trigger_enabled:
            if hurdle_matched:
                self.hurdle_tag_detection_stamp_sec = self._now_sec()
            if seen_ids:
                self.hurdle_tag_detection_ids_text = ','.join(str(tag_id) for tag_id in seen_ids)
            elif not hurdle_matched:
                self.hurdle_tag_detection_ids_text = 'none'
        if self.upstairs_tag_trigger_enabled:
            if upstairs_matched:
                self.upstairs_tag_detection_stamp_sec = self._now_sec()
            if seen_ids:
                self.upstairs_tag_detection_ids_text = ','.join(str(tag_id) for tag_id in seen_ids)
            elif not upstairs_matched:
                self.upstairs_tag_detection_ids_text = 'none'

    def _apriltag_tf_callback(self, msg: TFMessage) -> None:
        callback_now = self._now_sec()
        self.apriltag_tf_last_received_sec = callback_now
        tf_children = [
            f'{transform.header.frame_id}->{transform.child_frame_id}'
            for transform in msg.transforms
        ]
        self.apriltag_tf_last_children_text = (
            ','.join(tf_children) if tf_children else 'none'
        )
        for transform in msg.transforms:
            frame_id = str(transform.header.frame_id)
            child_frame_id = str(transform.child_frame_id)
            if (
                self.pole_apriltag_trigger_enabled
                and frame_id == self.pole_apriltag_frame_id
                and child_frame_id == self.pole_apriltag_child_frame_id
            ):
                distance_m = self._pole_apriltag_distance(transform.transform.translation)
                if distance_m is not None:
                    now = self._now_sec()
                    lateral_m = float(transform.transform.translation.x)
                    self.pole_apriltag_detection.distance_m = distance_m
                    self.pole_apriltag_detection.lateral_m = lateral_m
                    self.pole_apriltag_detection.nearest_stamp_sec = now
                    self._lock_pole_apriltag_handoff(
                        now,
                        distance_m,
                        lateral_m,
                    )
            if (
                (self.limit_bar_tag_align_enabled or self.limit_bar_tag_trigger_enabled)
                and frame_id == self.limit_bar_tag_frame_id
                and child_frame_id == self.limit_bar_tag_child_frame_id
            ):
                translation = transform.transform.translation
                roll_rad, pitch_rad, yaw_rad = self._rpy_from_quaternion(
                    transform.transform.rotation
                )
                now = self._now_sec()
                sample = LimitBarTagSample(
                    received_stamp_sec=now,
                    x_m=float(translation.x),
                    y_m=float(translation.y),
                    z_m=float(translation.z),
                    roll_rad=roll_rad,
                    pitch_rad=pitch_rad,
                    yaw_rad=yaw_rad,
                    frame_id=frame_id,
                    child_frame_id=child_frame_id,
                )
                if (
                    math.isfinite(sample.x_m)
                    and math.isfinite(sample.y_m)
                    and math.isfinite(sample.z_m)
                    and math.isfinite(sample.roll_rad)
                    and math.isfinite(sample.pitch_rad)
                    and math.isfinite(sample.yaw_rad)
                ):
                    self.limit_bar_tag_samples.append(sample)
                    self.limit_detection.distance_m = sample.z_m
                    self.limit_detection.lateral_m = sample.x_m
                    self.limit_detection.nearest_stamp_sec = now
                    self._prune_limit_bar_tag_samples(now)
                    if (
                        self.limit_bar_tag_presearch_handoff_enabled
                        and self.state in (self.STATE_WAIT_START, self.STATE_STAIRS)
                        and not self.limit_bar_completed
                    ):
                        self._lock_limit_bar_tag_handoff(
                            now,
                            sample.x_m,
                            sample.z_m,
                            sample.x_m + self.limit_bar_tag_to_center_x_m,
                            source='presearch',
                            require_flat_pose=True,
                        )
            if (
                self.hurdle_tag_trigger_enabled
                and frame_id == self.hurdle_tag_frame_id
                and child_frame_id == self.hurdle_tag_child_frame_id
            ):
                translation = transform.transform.translation
                roll_rad, pitch_rad, yaw_rad = self._rpy_from_quaternion(
                    transform.transform.rotation
                )
                now = self._now_sec()
                sample = LimitBarTagSample(
                    received_stamp_sec=now,
                    x_m=float(translation.x),
                    y_m=float(translation.y),
                    z_m=float(translation.z),
                    roll_rad=roll_rad,
                    pitch_rad=pitch_rad,
                    yaw_rad=yaw_rad,
                    frame_id=frame_id,
                    child_frame_id=child_frame_id,
                )
                if (
                    math.isfinite(sample.x_m)
                    and math.isfinite(sample.y_m)
                    and math.isfinite(sample.z_m)
                    and math.isfinite(sample.roll_rad)
                    and math.isfinite(sample.pitch_rad)
                    and math.isfinite(sample.yaw_rad)
                ):
                    self.hurdle_tag_samples.append(sample)
                    self.hurdle_detection.distance_m = sample.z_m
                    self.hurdle_detection.lateral_m = sample.x_m
                    self.hurdle_detection.nearest_stamp_sec = now
                    self._prune_hurdle_tag_samples(now)
                    if (now - self.last_hurdle_tag_tf_log_sec) >= 0.50:
                        self.last_hurdle_tag_tf_log_sec = now
                        self._publish_feedback(
                            f'高墙 AprilTag TF 已接收: '
                            f'frame={frame_id}->{child_frame_id}, '
                            f'tag_x={sample.x_m:.3f}m, tag_y={sample.y_m:.3f}m, '
                            f'tag_z={sample.z_m:.3f}m, '
                            f'rpy=({math.degrees(sample.roll_rad):.1f},'
                            f'{math.degrees(sample.pitch_rad):.1f},'
                            f'{math.degrees(sample.yaw_rad):.1f})deg, '
                            f'samples={len(self.hurdle_tag_samples)}, '
                            f'target_id={self.hurdle_tag_target_tag_id}。'
                        )
            if (
                self.upstairs_tag_trigger_enabled
                and frame_id == self.upstairs_tag_frame_id
                and child_frame_id == self.upstairs_tag_child_frame_id
            ):
                translation = transform.transform.translation
                roll_rad, pitch_rad, yaw_rad = self._rpy_from_quaternion(
                    transform.transform.rotation
                )
                now = self._now_sec()
                sample = LimitBarTagSample(
                    received_stamp_sec=now,
                    x_m=float(translation.x),
                    y_m=float(translation.y),
                    z_m=float(translation.z),
                    roll_rad=roll_rad,
                    pitch_rad=pitch_rad,
                    yaw_rad=yaw_rad,
                    frame_id=frame_id,
                    child_frame_id=child_frame_id,
                )
                if (
                    math.isfinite(sample.x_m)
                    and math.isfinite(sample.y_m)
                    and math.isfinite(sample.z_m)
                    and math.isfinite(sample.roll_rad)
                    and math.isfinite(sample.pitch_rad)
                    and math.isfinite(sample.yaw_rad)
                ):
                    self.upstairs_tag_samples.append(sample)
                    self.upstairs_detection.distance_m = sample.z_m
                    self.upstairs_detection.lateral_m = sample.x_m
                    self.upstairs_detection.nearest_stamp_sec = now
                    self._prune_upstairs_tag_samples(now)
                    if (now - self.last_upstairs_tag_tf_log_sec) >= 0.50:
                        self.last_upstairs_tag_tf_log_sec = now
                        self._publish_feedback(
                            f'上台阶 AprilTag TF 已接收: '
                            f'frame={frame_id}->{child_frame_id}, '
                            f'tag_x={sample.x_m:.3f}m, tag_y={sample.y_m:.3f}m, '
                            f'tag_z={sample.z_m:.3f}m, '
                            f'rpy=({math.degrees(sample.roll_rad):.1f},'
                            f'{math.degrees(sample.pitch_rad):.1f},'
                            f'{math.degrees(sample.yaw_rad):.1f})deg, '
                            f'samples={len(self.upstairs_tag_samples)}, '
                            f'target_id={self.upstairs_tag_target_tag_id}。'
                        )
            if (
                not self.sandpit_bypass_enabled
                or frame_id != self.sandpit_tag_frame_id
                or child_frame_id != self.sandpit_tag_child_frame_id
            ):
                continue
            translation = transform.transform.translation
            roll_rad, pitch_rad, yaw_rad = self._rpy_from_quaternion(
                transform.transform.rotation
            )
            distance_m = self._sandpit_tag_distance(translation)
            if distance_m is None:
                continue
            now = self._now_sec()
            sample = SandpitTagSample(
                received_stamp_sec=now,
                x_m=float(translation.x),
                y_m=float(translation.y),
                z_m=float(translation.z),
                distance_m=distance_m,
                roll_rad=roll_rad,
                pitch_rad=pitch_rad,
                yaw_rad=yaw_rad,
                frame_id=frame_id,
                child_frame_id=child_frame_id,
            )
            if not all(
                math.isfinite(value)
                for value in (
                    sample.x_m,
                    sample.y_m,
                    sample.z_m,
                    sample.distance_m,
                    sample.roll_rad,
                    sample.pitch_rad,
                    sample.yaw_rad,
                )
            ):
                continue
            self.sandpit_tag_samples.append(sample)
            self._prune_sandpit_tag_samples(now)
            recent_samples = self._recent_sandpit_tag_samples(now)
            stable_distance_m = self._median(
                [sample.distance_m for sample in recent_samples]
            )
            stable_lateral_m = self._median([sample.x_m for sample in recent_samples])
            self.sandpit_detection.distance_m = stable_distance_m
            self.sandpit_detection.lateral_m = stable_lateral_m
            self.sandpit_detection.nearest_stamp_sec = now
            if (
                self.sandpit_apriltag_priority_guard_enabled
                and stable_distance_m <= self.sandpit_apriltag_priority_distance_m
            ):
                self.sandpit_priority_guard_stamp_sec = now
            self._update_detection_trigger(
                self.sandpit_detection,
                stable_distance_m <= self.sandpit_trigger_distance_m
                and len(recent_samples) >= self.sandpit_tag_min_samples,
            )
            if (now - self.last_sandpit_tag_log_sec) >= 0.50:
                self.last_sandpit_tag_log_sec = now
                self._publish_feedback(
                    f'沙坑 AprilTag TF 已接收: frame={frame_id}->{child_frame_id}, '
                    f'samples={len(recent_samples)}/{self.sandpit_tag_min_samples}, '
                    f'raw_dist={distance_m:.3f}m, stable_dist={stable_distance_m:.3f}m, '
                    f'lateral={stable_lateral_m:.3f}m, '
                    f'rpy=({math.degrees(sample.roll_rad):.1f},'
                    f'{math.degrees(sample.pitch_rad):.1f},'
                    f'{math.degrees(sample.yaw_rad):.1f})deg, '
                    f'trigger={self.sandpit_trigger_distance_m:.3f}m, '
                    f'priority={self.sandpit_apriltag_priority_distance_m:.3f}m, '
                    f'target_id={self.sandpit_tag_target_id}。'
                )

    def _sandpit_tag_distance(self, translation) -> Optional[float]:
        x = float(translation.x)
        y = float(translation.y)
        z = float(translation.z)
        if self.sandpit_tag_distance_axis == 'euclidean':
            distance = math.sqrt(x * x + y * y + z * z)
        elif self.sandpit_tag_distance_axis == 'x':
            distance = x
        elif self.sandpit_tag_distance_axis == 'y':
            distance = y
        else:
            distance = z
        if not math.isfinite(distance) or distance < 0.0:
            return None
        return distance

    def _update_detection_trigger(self, detection: DetectionState, triggered: bool) -> None:
        now = self._now_sec()
        if triggered and not detection.triggered:
            detection.trigger_start_stamp_sec = now
        if not triggered:
            detection.trigger_start_stamp_sec = 0.0
        detection.triggered = triggered
        detection.trigger_stamp_sec = now

    def _stairs_state_callback(self, msg: String) -> None:
        self.stairs_state = str(msg.data).strip().upper()

    def _slope_state_callback(self, msg: String) -> None:
        self.slope_state = str(msg.data).strip().upper()

    def _slope_current_waypoint_callback(self, msg: String) -> None:
        self.slope_current_waypoint_name = self._parse_current_waypoint_name(str(msg.data))

    def _pole_state_callback(self, msg: String) -> None:
        self.pole_state = str(msg.data).strip().upper()

    def _pole_feedback_callback(self, msg: String) -> None:
        completed_index = self._parse_pole_completed_waypoint_index(str(msg.data))
        if completed_index is None:
            return
        self.pole_completed_waypoints = max(self.pole_completed_waypoints, completed_index)

    @staticmethod
    def _parse_pole_completed_waypoint_index(text: str) -> Optional[int]:
        marker = '到达 HSV 绕杆目标点['
        index = str(text).find(marker)
        if index < 0:
            return None
        start = index + len(marker)
        end = str(text).find('/', start)
        if end < 0:
            end = str(text).find(']', start)
        if end < 0:
            return None
        try:
            value = int(str(text)[start:end].strip())
        except ValueError:
            return None
        return value if value > 0 else None

    def _start_signal_callback(self, msg: Bool) -> None:
        if not bool(msg.data):
            return
        now = self._now_sec()
        self.start_signal_received = True
        if self.state == self.STATE_WAIT_START:
            self.start_retry_guard_deadline_sec = now + self.start_retry_guard_sec
            self._enter_started_flow(
                now,
                reason='serial_start_signal',
                clear_wait_override=True,
            )
            return
        # A real failure must remain recoverable even if it happens during the
        # initial duplicate-start guard window.
        if self.state != self.STATE_FAILED and now < self.start_retry_guard_deadline_sec:
            remaining_sec = self.start_retry_guard_deadline_sec - now
            self._publish_feedback(
                f'首次启动保护窗口内收到电控 [start]，已忽略；'
                f'剩余保护时间={remaining_sec:.2f}s。'
            )
            return
        self._handle_retry_signal(now)

    def _finish_signal_callback(self, msg: Bool) -> None:
        if not bool(msg.data):
            return
        self.sandpit_jumping_signal_received = True
        self.sandpit_jumping_signal_stamp_sec = self._now_sec()
        self.sandpit_finish_signal_received = True
        self.sandpit_finish_signal_stamp_sec = self._now_sec()
        if self.state == self.STATE_SANDPIT_BYPASS and self.active_bypass_kind == 'sandpit':
            self._publish_feedback('收到电控 [finish] 回包，继续执行砂坑穿越后半段。')

    def _serial_connected_callback(self, msg: Bool) -> None:
        connected = bool(msg.data)
        if connected == self.serial_connected:
            self.serial_connected_stamp_sec = self._now_sec()
            return
        self.serial_connected = connected
        self.serial_connected_stamp_sec = self._now_sec()
        if self.state == self.STATE_SANDPIT_BYPASS and self.sandpit_phase_wait_for_finish:
            state_text = '已重连' if connected else '已断开'
            self._publish_feedback(
                f'{self._active_bypass_label()}等待电控回包期间检测到串口{state_text}: '
                f'jumping={self.sandpit_jumping_signal_received}, '
                f'finish={self.sandpit_finish_signal_received}。'
            )

    def _action_ack_callback(self, msg: String) -> None:
        token = str(msg.data).strip().lower()
        now = self._now_sec()
        if token == 'jumping':
            self.sandpit_jumping_signal_received = True
            self.sandpit_jumping_signal_stamp_sec = now
            if self.upstairs_step_once_waiting_ack:
                self.upstairs_jumping_signal_received = True
                self.upstairs_jumping_signal_stamp_sec = now
                self.upstairs_step_once_waiting_ack = False
                self.upstairs_step_once_next_resend_sec = None
                self._publish_feedback('收到电控 [jumping] 回包，已确认上台阶动作，停止重发 step_once。')
            if self.state == self.STATE_SANDPIT_BYPASS and self.active_bypass_kind == 'sandpit':
                self._publish_feedback('收到电控 [jumping] 回包，已确认起跳，停止重发 step_once。')
            return
        if token == 'finish':
            self.sandpit_jumping_signal_received = True
            self.sandpit_jumping_signal_stamp_sec = now
            self.sandpit_finish_signal_received = True
            self.sandpit_finish_signal_stamp_sec = now
            if self.upstairs_step_once_waiting_ack:
                self.upstairs_jumping_signal_received = True
                self.upstairs_jumping_signal_stamp_sec = now
                self.upstairs_step_once_waiting_ack = False
                self.upstairs_step_once_next_resend_sec = None
                self._publish_feedback('收到电控 [finish] 回包，已确认上台阶动作，停止重发 step_once。')
            if self.state == self.STATE_SANDPIT_BYPASS and self.active_bypass_kind == 'sandpit':
                self._publish_feedback('收到电控 [finish] 回包，继续执行砂坑穿越后半段。')

    @staticmethod
    def _parse_current_waypoint_name(text: str) -> str:
        parts = str(text).strip().split()
        if len(parts) >= 2 and '/' in parts[0]:
            return parts[1]
        return parts[0] if parts else ''

    def _handle_retry_signal(self, now: float) -> None:
        if self.state == self.STATE_SEARCH:
            self._reset_detection_states()
            self.search_arm_deadline = now + self.search_rearm_delay_sec
            self._publish_feedback('搜索状态收到电控 [start] 重试信号，清除当前检测缓存并继续搜索。')
            return
        if self.state == self.STATE_DONE:
            if self._upstairs_done_retry_guard_active(now):
                self._handle_upstairs_done_retry(now)
                return
            self._publish_feedback(f'状态={self.state} 收到电控 [start]，流程已结束，已忽略。')
            return
        if self.state == self.STATE_FAILED:
            if self._failed_retry_uses_bridge_logic():
                self._handle_slope_bridge_retry(now)
                return
            self._handle_failed_retry(now)
            return
        if self.state == self.STATE_WAIT_START:
            return
        if self.state in (self.STATE_SLOPE, self.STATE_SLOPE_ALIGN):
            self._handle_slope_retry(now)
            return
        if self.state == self.STATE_POLE:
            self._handle_pole_retry(now)
            return

        self._handle_generic_retry(now)

    def _failed_retry_uses_bridge_logic(self) -> bool:
        return (
            str(getattr(self, 'failed_from_state', '')).strip() == self.STATE_SLOPE
            and self._slope_runtime_yaw_reference_enabled()
            and self.slope_current_waypoint_name in self.slope_retry_bridge_waypoints
        )

    def _handle_failed_retry(self, now: float) -> None:
        failed_from_state = str(getattr(self, 'failed_from_state', '')).strip()
        failed_from_branch = str(getattr(self, 'failed_from_branch', '')).strip()
        failure_text = str(getattr(self, 'last_failure_text', '')).strip()

        if failed_from_state in (self.STATE_SLOPE, self.STATE_SLOPE_ALIGN) or (
            failed_from_branch in ('slope', 'slope_align')
        ):
            self.slope_branch_completed = False

        self._clear_step_override()
        self._reset_detection_states()
        self._reset_transient_branch_state()
        reason_suffix = (failed_from_state or failed_from_branch or 'unknown').lower()
        self._enter_search(now, reason=f'retry_after_failed_{reason_suffix}')
        self.search_arm_deadline = now + self.search_rearm_delay_sec
        self._publish_active_flags()

        source_text = failed_from_state or failed_from_branch or 'unknown'
        failure_detail = f'，原失败原因={failure_text}' if failure_text else ''
        self._publish_feedback(
            f'状态=FAILED 收到电控 [start] 重试信号: '
            f'失败来源={source_text}{failure_detail}；已清理失败分支并返回搜索模式。'
        )

    def _handle_generic_retry(self, now: float) -> None:
        previous_state = self.state
        self._clear_step_override()
        self._reset_detection_states()
        self._reset_transient_branch_state()
        self._enter_search(now, reason=f'retry_from_{previous_state.lower()}')
        self.search_arm_deadline = now + self.search_rearm_delay_sec
        self._publish_active_flags()
        self._publish_feedback(
            f'收到电控 [start] 重试信号: 从 {previous_state} 返回搜索模式。'
        )

    def _handle_upstairs_done_retry(self, now: float) -> None:
        self._clear_step_override()
        self._reset_detection_states()
        self._reset_transient_branch_state()
        self.upstairs_completed = False
        self.upstairs_done_retry_guard_deadline_sec = None
        self.upstairs_jumping_signal_received = False
        self.upstairs_jumping_signal_stamp_sec = 0.0
        self.upstairs_step_once_waiting_ack = False
        self.upstairs_step_once_next_resend_sec = None
        self.upstairs_step_once_resend_count = 0
        self._enter_search(now, reason='retry_during_upstairs_done_guard')
        self.search_arm_deadline = now + self.search_rearm_delay_sec
        self._publish_active_flags()
        self._publish_feedback(
            '上台阶结束保护阶段收到电控 [start] 重试信号：'
            '已停止保护小前进、将上台阶分支重置为未完成，并返回搜索模式。'
        )

    def _handle_pole_retry(self, now: float) -> None:
        self.pole_retry_failure_count += 1
        self._clear_step_override()
        self._reset_detection_states()
        self._reset_transient_branch_state()
        self._enter_search(now, reason='pole_retry')
        self.search_arm_deadline = now + self.pole_retry_search_rearm_delay_sec
        self._publish_active_flags()
        self._publish_feedback(
            f'收到电控 [start] 绕杆重试信号: failure_count='
            f'{self.pole_retry_failure_count}/{self.pole_retry_failures_before_bypass}，'
            '返回搜索模式。'
        )

    def _handle_slope_retry(self, now: float) -> None:
        name = self.slope_current_waypoint_name
        if name in getattr(self, 'slope_retry_search_waypoints', set()):
            self._handle_slope_search_retry(now)
            return
        if self._slope_runtime_yaw_reference_enabled():
            if name in self.slope_retry_bridge_waypoints:
                self._handle_slope_bridge_retry(now)
                return
            if name in self.slope_retry_pre_bridge_waypoints:
                self._handle_slope_route_rezero_retry(now)
                return

        restart_name = self._slope_retry_restart_waypoint_name()
        self._restart_slope_manager_from(
            restart_name,
            (
                f'收到电控 [start] 斜坡重试信号: 当前 slope waypoint='
                f'{self.slope_current_waypoint_name or "unknown"}，'
                f'从 {restart_name} 重新启动斜坡 manager。'
            ),
        )

    def _handle_slope_search_retry(self, now: float) -> None:
        waypoint_name = self.slope_current_waypoint_name or 'unknown'
        self._clear_step_override()
        self._reset_detection_states()
        self._reset_transient_branch_state()
        self.slope_branch_completed = False
        self.slope_retry_rezero_search_active = False
        self._enter_search(now, reason='slope_retry_after_final_descent_mid_spin')
        self.search_arm_deadline = now + self.search_rearm_delay_sec
        self._publish_active_flags()
        self._publish_feedback(
            f'收到电控 [start] 斜坡后段重试信号: 当前 slope waypoint='
            f'{waypoint_name}；已停用 slope_manager 并返回搜索模式。'
        )

    def _slope_runtime_yaw_reference_enabled(self) -> bool:
        if not getattr(self, 'slope_retry_runtime_yaw_reference_enabled', False):
            return False
        standard_basename = str(
            getattr(self, 'slope_retry_standard_waypoints_basename', '')
        ).strip()
        mirror_basename = str(
            getattr(
                self,
                'slope_retry_mirror_waypoints_basename',
                'obstacle_waypoints_mirror.yaml',
            )
        ).strip()
        expected_basenames = {
            basename
            for basename in (standard_basename, mirror_basename)
            if basename
        }
        if not expected_basenames:
            return True
        active_file = str(getattr(self, 'sandpit_bypass_file', '')).strip()
        return bool(active_file) and Path(active_file).name in expected_basenames

    def _slope_mirror_waypoints_active(self) -> bool:
        mirror_basename = str(
            getattr(
                self,
                'slope_retry_mirror_waypoints_basename',
                'obstacle_waypoints_mirror.yaml',
            )
        ).strip()
        active_file = str(getattr(self, 'sandpit_bypass_file', '')).strip()
        return (
            bool(mirror_basename and active_file)
            and Path(active_file).name == mirror_basename
        )

    def _handle_slope_route_rezero_retry(self, now: float) -> None:
        if not self._pose_is_fresh(now):
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_feedback(
                '桥前斜坡重试未重新锁零：当前 Odometry 不新鲜，保持停止；请在 Odometry 恢复后再次发送重试。'
            )
            return

        reuse_reference = (
            self.slope_retry_route_rezero_once
            and self.slope_retry_route_rezero_used
            and self.slope_route_yaw_reference is not None
        )
        if reuse_reference:
            reference_yaw = float(self.slope_route_yaw_reference)
            reference_action = '复用已锁存'
        else:
            reference_yaw = float(self.pose_yaw)
            self.slope_retry_route_rezero_used = True
            reference_action = '重新锁存'
        self._publish_slope_route_yaw_reference(now, reference_yaw)

        self._clear_step_override()
        self._reset_detection_states()
        self._reset_transient_branch_state()
        self.slope_branch_completed = False
        self.slope_retry_rezero_search_active = True
        self._enter_search(now, reason='post_stairs_slope_retry_rezero')
        self.search_reference_yaw = reference_yaw
        self.search_arm_deadline = now + self.slope_retry_rezero_rearm_delay_sec
        self._publish_active_flags()
        self._publish_feedback(
            f'收到电控 [start] 桥前斜坡重试信号: 当前 slope waypoint='
            f'{self.slope_current_waypoint_name or "unknown"}，'
            f'{reference_action}赛道 0deg 基准='
            f'{math.degrees(reference_yaw):.1f}deg；已停用 slope_manager 并返回仅斜坡恢复搜索。'
        )

    def _handle_slope_bridge_retry(self, now: float) -> None:
        if not self._pose_is_fresh(now):
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_feedback(
                '入桥重试未锁存桥方向：当前 Odometry 不新鲜，保持停止；请在 Odometry 恢复后再次发送重试。'
            )
            return

        reference_yaw = float(self.pose_yaw)
        self._publish_slope_bridge_retry_yaw_reference(now, reference_yaw)
        self.slope_bridge_retry_count = (
            int(getattr(self, 'slope_bridge_retry_count', 0)) + 1
        )
        full_route_retry_count = max(
            1,
            int(getattr(self, 'slope_bridge_full_route_retry_count', 3)),
        )
        restart_name = (
            'turn_180_yaw_settle'
            if self.slope_bridge_retry_count <= full_route_retry_count
            else (
                'bridge_exit_cw_45_spin_mirror'
                if self._slope_mirror_waypoints_active()
                else 'bridge_exit_ccw_45_spin'
            )
        )
        self.slope_retry_rezero_search_active = False
        self._restart_slope_manager_from(
            restart_name,
            (
                f'收到电控 [start] 桥区第 {self.slope_bridge_retry_count} 次重试信号: '
                f'当前 slope waypoint='
                f'{self.slope_current_waypoint_name or "unknown"}，'
                f'锁存桥方向基准={math.degrees(reference_yaw):.1f}deg，'
                f'从 {restart_name} 重新启动。'
            ),
        )

    def _restart_slope_manager_from(self, restart_name: str, feedback: str) -> None:
        self._clear_step_override()
        self._reset_detection_states()
        self._reset_transient_branch_state()
        self.slope_branch_completed = False
        self.state = self.STATE_SLOPE
        self.current_active_branch = 'slope'
        self.phase_deadline_sec = None
        self.slope_state = 'RESTARTING'
        self._publish_state(self.state)
        self._publish_active_flags()
        msg = String()
        msg.data = restart_name
        self.slope_retry_restart_pub.publish(msg)
        self._publish_feedback(feedback)

    def _slope_retry_restart_waypoint_name(self) -> str:
        name = self.slope_current_waypoint_name
        if not name:
            return 'turn_180_spin'
        if name in ('bridge_entry', 'bridge_exit'):
            return 'turn_180_yaw_settle'
        if name.startswith('bridge_'):
            return 'turn_180_yaw_settle'
        return 'turn_180_spin'

    def _control_loop(self) -> None:
        now = self._now_sec()
        self._publish_active_flags()

        if self.state == self.STATE_WAIT_START:
            self._run_wait_start(now)
            return

        if self.state == self.STATE_DONE:
            self._run_done(now)
            return

        if self.state == self.STATE_FAILED:
            self._run_failed()
            return

        if self.state == self.STATE_STAIRS:
            if self.stairs_state == 'FINISHED':
                if self.post_stairs_slope_entry_enabled and not self.slope_branch_completed:
                    self._enter_post_stairs_slope_entry(now, reason='stairs_finished')
                else:
                    self._enter_search(now, reason='stairs_finished')
            elif self.stairs_state == 'FAILED':
                self._fail('固定下楼梯分支失败。')
            return

        if self.state == self.STATE_SEARCH:
            self._run_search(now)
            if now < self.search_arm_deadline and not self._pole_apriltag_ready(now):
                return
            if (
                self.slope_retry_rezero_search_active
                and self.slope_retry_rezero_search_only_slope
            ):
                branch = self._classify_slope_retry_rezero_branch(now)
            else:
                branch = self._classify_search_branch(now)
            if branch is not None and self._enter_detected_branch(
                branch,
                now,
                'slope_fallback' if branch == 'slope_fallback' else f'{branch}_detected',
            ):
                return
            return

        if self.state == self.STATE_LIMIT_BAR_ALIGN:
            self._run_slope_align(now)
            return

        if self.state == self.STATE_HURDLE_ALIGN:
            self._run_slope_align(now)
            return

        if self.state == self.STATE_UPSTAIRS_ALIGN:
            self._run_slope_align(now)
            return

        if self.state == self.STATE_LIMIT_BAR_LATERAL_SHIFT:
            self._run_limit_bar_lateral_shift(now)
            return

        if self.state == self.STATE_LIMIT_BAR_STOP_BUFFER:
            self._publish_step_override(0, 0.0, 0.0)
            if self.phase_deadline_sec is None or now < self.phase_deadline_sec:
                return
            self._enter_limit_bar_prepare(
                now,
                self.pending_limit_bar_reason,
                self.pending_limit_bar_yaw_ref_text,
                skip_stop_buffer=True,
            )
            return

        if self.state == self.STATE_LIMIT_BAR_PREPARE:
            self._publish_step_override(
                self.duck_prepare_mode,
                self.duck_prepare_left_norm,
                self.duck_prepare_right_norm,
            )
            if self.phase_deadline_sec is None or now < self.phase_deadline_sec:
                return
            if self.pose_x is None or self.pose_y is None:
                self._publish_feedback('限高杆趴下等待完成，但当前没有可用 Odometry，继续保持趴下停止。')
                return
            if self._run_limit_bar_duck_prepare_yaw_settle(now):
                return
            if self.duck_centerline_correction_enabled:
                if self.duck_reference_yaw is None:
                    self.duck_centerline_locked = False
                    self.duck_centerline_start_x = None
                    self.duck_centerline_start_y = None
                    self.duck_centerline_yaw = None
                    self.duck_centerline_goal_x = None
                    self.duck_centerline_goal_y = None
                    self.duck_start_x = self.pose_x
                    self.duck_start_y = self.pose_y
                    self._publish_feedback(
                        '限高杆趴下等待完成，但 yaw 基准无效，改用当前位置距离趴走。'
                    )
                elif not self._lock_duck_centerline(
                    now,
                    self.duck_reference_yaw,
                    'limit_bar_prepare_done',
                ):
                    self._fail('限高杆趴下等待完成后无法锁定趴走中心线。')
                    return
            else:
                self.duck_start_x = self.pose_x
                self.duck_start_y = self.pose_y
            self.phase_deadline_sec = (
                now + self.duck_travel_timeout_sec
                if self.duck_travel_timeout_sec > 0.0
                else None
            )
            self.last_duck_progress_log_time = 0.0
            self.state = self.STATE_LIMIT_BAR
            self._publish_state(self.state)
            duck_start_text = 'unknown'
            if self.duck_start_x is not None and self.duck_start_y is not None:
                duck_start_text = f'({self.duck_start_x:.3f},{self.duck_start_y:.3f})'
            if self.duck_centerline_correction_enabled and self.duck_centerline_locked:
                duck_start_text = (
                    f'centerline=({self.duck_centerline_start_x:.3f},'
                    f'{self.duck_centerline_start_y:.3f})'
                )
            self._publish_feedback(
                f'限高杆趴下等待完成，开始按'
                f'{"锁存 yaw 方向投影距离" if self.duck_travel_projection_enabled else "实际移动距离"} '
                f'{self.duck_travel_distance_m:.2f}m 通过，'
                f'起点={duck_start_text}, '
                f'yaw_ref={self._duck_reference_yaw_text()}。'
            )
            return

        if self.state == self.STATE_LIMIT_BAR:
            if self.duck_centerline_correction_enabled and self.duck_centerline_locked:
                (
                    walk_left_norm,
                    walk_right_norm,
                    along_m,
                    lateral_m,
                    yaw_error_rad,
                    centerline_delta_norm,
                ) = self._duck_centerline_corrected_norms(
                    self.duck_walk_left_norm,
                    self.duck_walk_right_norm,
                )
                yaw_error_deg = (
                    None if yaw_error_rad is None else math.degrees(yaw_error_rad)
                )
                if along_m is None or lateral_m is None or yaw_error_rad is None:
                    self._fail('限高杆中心线趴走无法计算 Odometry 误差，停止限高杆趴走。')
                    return
                failure_reason = self._duck_centerline_failure_reason(lateral_m, yaw_error_rad)
                if failure_reason is not None:
                    self._fail(
                        f'{failure_reason} along={self._distance_text(along_m)}, '
                        f'centerline_yaw={self._yaw_text(self.duck_centerline_yaw)}，'
                        '停止限高杆趴走。'
                    )
                    return
                walk_left_norm, walk_right_norm, sandpit_overlay_text = (
                    self._sandpit_pre_align_overlay_norms(
                        now,
                        walk_left_norm,
                        walk_right_norm,
                        source='limit_bar_duck',
                    )
                )
                self._publish_step_override(self.duck_walk_mode, walk_left_norm, walk_right_norm)
                if along_m is not None and along_m >= self.duck_travel_distance_m:
                    self._enter_limit_bar_wait(
                        now,
                        f'限高杆中心线通过距离满足: along={along_m:.3f}m >= '
                        f'{self.duck_travel_distance_m:.3f}m，持续站起停止并等待 '
                        f'{self.duck_resume_delay_sec:.1f}s。',
                    )
                    return
                if self.phase_deadline_sec is not None and now >= self.phase_deadline_sec:
                    travel_text = 'unknown' if along_m is None else f'{along_m:.3f}m'
                    self._enter_limit_bar_wait(
                        now,
                        f'限高杆中心线趴走超时: along={travel_text}, '
                        f'timeout={self.duck_travel_timeout_sec:.1f}s，先站起停止。',
                    )
                    return
                self._log_duck_progress(
                    now,
                    along_m,
                    yaw_error_deg,
                    centerline_delta_norm,
                    lateral_error_m=lateral_m,
                    left_norm=walk_left_norm,
                    right_norm=walk_right_norm,
                    correction_kind='centerline',
                    extra_text=sandpit_overlay_text,
                )
                return

            walk_left_norm, walk_right_norm, yaw_error_deg, yaw_correction_norm = (
                self._duck_corrected_norms(
                    self.duck_walk_left_norm,
                    self.duck_walk_right_norm,
                )
            )
            walk_left_norm, walk_right_norm, sandpit_overlay_text = (
                self._sandpit_pre_align_overlay_norms(
                    now,
                    walk_left_norm,
                    walk_right_norm,
                    source='limit_bar_duck',
                )
            )
            self._publish_step_override(self.duck_walk_mode, walk_left_norm, walk_right_norm)
            duck_travel_m, progress_label = self._duck_travel_progress()
            if duck_travel_m is not None and duck_travel_m >= self.duck_travel_distance_m:
                self._enter_limit_bar_wait(
                    now,
                    f'限高杆通过距离满足: '
                    f'{progress_label}={duck_travel_m:.3f}m >= '
                    f'{self.duck_travel_distance_m:.3f}m，持续站起停止并等待 '
                    f'{self.duck_resume_delay_sec:.1f}s。',
                )
                return
            if self.phase_deadline_sec is not None and now >= self.phase_deadline_sec:
                travel_text = 'unknown' if duck_travel_m is None else f'{duck_travel_m:.3f}m'
                self._enter_limit_bar_wait(
                    now,
                    f'限高杆趴走超时: {progress_label}={travel_text}, '
                    f'timeout={self.duck_travel_timeout_sec:.1f}s，先站起停止。',
                )
                return
            self._log_duck_progress(
                now,
                duck_travel_m,
                yaw_error_deg,
                yaw_correction_norm,
                left_norm=walk_left_norm,
                right_norm=walk_right_norm,
                extra_text=sandpit_overlay_text,
                progress_label=progress_label,
            )
            return

        if self.state == self.STATE_LIMIT_BAR_WAIT:
            self._publish_step_override(
                self.duck_resume_mode,
                self.duck_resume_left_norm,
                self.duck_resume_right_norm,
            )
            if self.phase_deadline_sec is None or now < self.phase_deadline_sec:
                return
            if (
                self.limit_bar_post_resume_yaw_align_enabled
                and self.duck_reference_yaw is not None
                and self.pose_yaw is not None
            ):
                self._enter_limit_bar_post_resume_align(now)
                return
            self._finish_limit_bar_done(now, reason='limit_bar_done')
            return

        if self.state == self.STATE_LIMIT_BAR_POST_RESUME_ALIGN:
            self._run_limit_bar_post_resume_align(now)
            return

        if self.state == self.STATE_HURDLE_WAIT_JUMP:
            self._publish_step_override(
                self.hurdle_stand_mode,
                self.hurdle_stand_left_norm,
                self.hurdle_stand_right_norm,
            )
            if self.phase_deadline_sec is None or now < self.phase_deadline_sec:
                return
            self._clear_step_override()
            self._publish_step_once(
                self.hurdle_jump_mode,
                self.hurdle_jump_left_norm,
                self.hurdle_jump_right_norm,
            )
            self.pending_hurdle_jump_once_repeats = (
                self.hurdle_jump_step_once_repeat_count - 1
            )
            self.next_hurdle_jump_once_repeat_sec = (
                now + self.hurdle_jump_step_once_repeat_interval_sec
            )
            self.phase_deadline_sec = now + self.hurdle_jump_to_walk_delay_sec
            self.state = self.STATE_HURDLE_WAIT_RESUME
            self._publish_state(self.state)
            self._publish_feedback(
                f'跳跃指令重复发送 {self.hurdle_jump_step_once_repeat_count} 次，'
                f'等待 {self.hurdle_jump_to_walk_delay_sec:.1f}s 后恢复搜索前行。'
            )
            return

        if self.state == self.STATE_HURDLE_WAIT_RESUME:
            self._publish_step_override(
                self.hurdle_stand_mode,
                self.hurdle_stand_left_norm,
                self.hurdle_stand_right_norm,
            )
            self._publish_pending_hurdle_jump_once_repeats(now)
            if self.phase_deadline_sec is None or now < self.phase_deadline_sec:
                return
            self._clear_step_override()
            self.phase_deadline_sec = None
            self.hurdle_completed = True
            if self._enter_hurdle_post_align(now):
                return
            self._enter_search(now, reason='hurdle_done')
            return

        if self.state == self.STATE_HURDLE_POST_ALIGN:
            self._run_hurdle_post_align(now)
            return

        if self.state == self.STATE_POLE:
            if (
                self.pole_pre_bypass_pending
                and self.pole_completed_waypoints >= self.pole_bypass_pre_bypass_waypoints
            ):
                if not self._pose_is_fresh(now):
                    self._publish_step_override(0, 0.0, 0.0)
                    self._publish_feedback(
                        '绕杆预跑 waypoint 已完成，但当前没有新鲜 Odometry，'
                        '先停止等待再进入绕杆绕行。'
                    )
                    return
                completed_waypoints = self.pole_completed_waypoints
                reason = self.pole_pre_bypass_reason or 'pole_pre_bypass_done'
                self.pole_pre_bypass_pending = False
                self.pole_pre_bypass_reason = ''
                self._enter_pole_bypass(
                    now,
                    f'{reason}_after_{completed_waypoints}_pole_waypoints',
                )
                self._publish_active_flags()
                return
            if self.pole_state == 'FINISHED':
                self.pole_completed = True
                self.pole_retry_failure_count = 0
                self.pole_pre_bypass_pending = False
                self.pole_pre_bypass_reason = ''
                self._enter_search(now, reason='pole_done')
            elif self.pole_state == 'FAILED':
                self._handle_pole_retry(now)
            return

        if self.state == self.STATE_SANDPIT_PRE_ALIGN:
            self._run_sandpit_pre_align(now)
            return

        if self.state == self.STATE_SANDPIT_BYPASS:
            self._run_sandpit_bypass(now)
            return

        if self.state == self.STATE_SLOPE_ALIGN:
            if self._upstairs_ready(now):
                self._enter_detected_branch('upstairs', now, 'upstairs_preempt_slope_align')
                return
            if self._upstairs_slope_guard_ready(now):
                self._publish_feedback(
                    '检测到疑似上台阶，暂不进入斜坡木桥分支，继续搜索等待上台阶触发距离。'
                )
                self._enter_search(now, reason='upstairs_guard_slope_align')
                return
            self._run_slope_align(now)
            return

        if self.state == self.STATE_POST_STAIRS_SLOPE_APPROACH:
            self._run_post_stairs_slope_approach(now)
            return

        if self.state == self.STATE_POST_STAIRS_SLOPE_CONFIRM:
            self._run_post_stairs_slope_confirm(now)
            return

        if self.state == self.STATE_SLOPE:
            if self.slope_state == 'FINISHED':
                self.slope_branch_completed = True
                self._enter_search(now, reason='slope_done')
            elif self.slope_state == 'FAILED':
                self._fail('斜坡木桥分支失败。')
            return

    def _publish_active_flags(self) -> None:
        stairs_active = self.state == self.STATE_STAIRS
        slope_active = self.state == self.STATE_SLOPE
        pole_active = self.state == self.STATE_POLE
        self._publish_bool(self.stairs_active_pub, stairs_active)
        self._publish_bool(self.slope_active_pub, slope_active)
        self._publish_bool(self.pole_active_pub, pole_active)

    def _run_wait_start(self, now: float) -> None:
        self._publish_step_override(
            self.start_wait_mode,
            self.start_wait_left_norm,
            self.start_wait_right_norm,
        )
        if self.start_signal_received:
            self._enter_started_flow(
                now,
                reason='serial_start_signal',
                clear_wait_override=True,
            )

    def _enter_started_flow(
        self,
        now: float,
        reason: str,
        clear_wait_override: bool,
    ) -> None:
        if clear_wait_override:
            self._clear_step_override()
        self._reset_detection_states()
        self.phase_deadline_sec = None
        self.slope_route_yaw_reference = None
        self.slope_route_yaw_reference_frame = ''
        self.slope_route_yaw_reference_stamp_sec = 0.0
        self.slope_bridge_retry_yaw_reference = None
        self.slope_bridge_retry_yaw_reference_frame = ''
        self.slope_bridge_retry_yaw_reference_stamp_sec = 0.0
        self.slope_bridge_retry_count = 0
        self.slope_retry_route_rezero_used = False
        self.slope_retry_rezero_search_active = False
        if self.start_with_stairs_manager:
            self.state = self.STATE_STAIRS
            self.current_active_branch = 'stairs'
            self._publish_state(self.state)
            if reason == 'startup':
                self._publish_feedback('启动后先执行固定下楼梯分支。')
            else:
                self._publish_feedback('收到电控串口 [start]，开始固定下楼梯分支。')
            self._publish_active_flags()
            return

        self._enter_search(now, reason=reason)
        self._publish_active_flags()

    def _enter_search(self, now: float, reason: str) -> None:
        self.state = self.STATE_SEARCH
        self.current_active_branch = ''
        self.phase_deadline_sec = None
        self.slope_align_next_action = 'slope_branch'
        self.slope_align_custom_target_yaw = None
        self.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'
        self._reset_lateral_s_curve_shadow()
        self._clear_hurdle_locked_tag()
        self._clear_limit_bar_locked_tag()
        self.post_stairs_slope_entry_start_x = None
        self.post_stairs_slope_entry_start_y = None
        self.duck_start_x = self.pose_x
        self.duck_start_y = self.pose_y
        self.duck_reference_yaw = None
        self.duck_reference_yaw_source = ''
        self.duck_reference_yaw_offset_rad = 0.0
        self._clear_duck_centerline()
        self.sandpit_phase_index = None
        self.sandpit_phase_start_x = None
        self.sandpit_phase_start_y = None
        self.sandpit_phase_start_yaw = None
        self.sandpit_phase_start_sec = 0.0
        self.sandpit_entry_yaw = None
        self.sandpit_phase_target_yaw = None
        self.sandpit_phase_target_distance_m = None
        self.sandpit_phase_deadline_sec = None
        self.sandpit_phase_stable_since = None
        self.sandpit_phase_turn_direction_sign = 0
        self.sandpit_phase_turn_last_yaw = None
        self.sandpit_phase_turn_progress_rad = 0.0
        self.sandpit_phase_turn_target_delta_rad = None
        self.sandpit_phase_walk_yaw_guard_active = False
        self.sandpit_phase_wait_for_finish = False
        self.sandpit_jumping_signal_received = False
        self.sandpit_jumping_signal_stamp_sec = 0.0
        self.sandpit_finish_signal_received = False
        self.sandpit_finish_signal_stamp_sec = 0.0
        self.sandpit_last_step_once_command = None
        self.sandpit_last_step_once_yaw = None
        self.sandpit_wait_finish_serial_disconnect_active = False
        self.sandpit_wait_finish_serial_disconnect_stamp_sec = 0.0
        self.sandpit_wait_finish_serial_disconnect_had_jumping = False
        self._reset_sandpit_pre_align_state()
        self.pole_pre_bypass_pending = False
        self.pole_pre_bypass_reason = ''
        self.pole_completed_waypoints = 0
        self.search_enter_time = now
        self.search_arm_deadline = now + self.search_rearm_delay_sec
        self.search_reason = reason
        if reason in ('sandpit_traverse_done', 'sandpit_bypass_done'):
            ok, approach_m, estimated_distance_m = (
                self._pole_apriltag_handoff_progress(now)
            )
            if ok:
                self._publish_feedback(
                    f'继承沙坑阶段 ID{self.pole_apriltag_target_tag_id} 锁存进入 SEARCH: '
                    f'latched_dist={self.pole_apriltag_handoff_distance_m:.3f}m, '
                    f'odom_approach={approach_m:.3f}m, '
                    f'est_dist={estimated_distance_m:.3f}m, '
                    f'age={now - self.pole_apriltag_handoff_stamp_sec:.2f}/'
                    f'{self.pole_apriltag_odom_handoff_timeout_sec:.2f}s。'
                )
        self.search_start_x = self.pose_x
        self.search_start_y = self.pose_y
        self.search_reference_yaw = self.pose_yaw
        self.last_search_yaw_log_time = 0.0
        self.slope_roll_detection_start_sec = 0.0
        self.last_slope_roll_detection_log_sec = 0.0
        self.slope_align_stable_since = None
        self.post_stairs_slope_roll_priority_start_sec = 0.0
        self.last_post_stairs_slope_roll_priority_log_sec = 0.0
        self._publish_state(self.state)
        self._publish_feedback(
            f'进入搜索模式: reason={reason}, '
            f'walk=[{self.search_mode},{self.search_left_norm:.3f},{self.search_right_norm:.3f}]。'
        )

    def _reset_transient_branch_state(self) -> None:
        self.phase_deadline_sec = None
        self.current_active_branch = ''
        self.pending_hurdle_jump_once_repeats = 0
        self.next_hurdle_jump_once_repeat_sec = 0.0
        self.pending_limit_bar_reason = ''
        self.pending_limit_bar_yaw_ref_text = 'unknown'
        self.duck_start_x = self.pose_x
        self.duck_start_y = self.pose_y
        self.duck_reference_yaw = None
        self.duck_reference_yaw_source = ''
        self.duck_reference_yaw_offset_rad = 0.0
        self._clear_duck_centerline()
        self.slope_align_next_action = 'slope_branch'
        self.slope_align_custom_target_yaw = None
        self.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'
        self._clear_hurdle_locked_tag()
        self._clear_limit_bar_locked_tag()
        self.slope_align_stable_since = None
        self.post_stairs_slope_entry_start_x = None
        self.post_stairs_slope_entry_start_y = None
        self.post_stairs_slope_confirm_start_time = 0.0
        self.slope_roll_detection_start_sec = 0.0
        self.post_stairs_slope_roll_priority_start_sec = 0.0
        self.slope_upstairs_wait_start_sec = 0.0
        self.sandpit_phase_index = None
        self.sandpit_phase_start_x = None
        self.sandpit_phase_start_y = None
        self.sandpit_phase_start_yaw = None
        self.sandpit_phase_start_sec = 0.0
        self.sandpit_entry_yaw = None
        self.sandpit_phase_target_yaw = None
        self.sandpit_phase_target_distance_m = None
        self.sandpit_phase_deadline_sec = None
        self.sandpit_phase_stable_since = None
        self.sandpit_phase_turn_direction_sign = 0
        self.sandpit_phase_turn_last_yaw = None
        self.sandpit_phase_turn_progress_rad = 0.0
        self.sandpit_phase_turn_target_delta_rad = None
        self.sandpit_phase_walk_yaw_guard_active = False
        self.sandpit_phase_wait_for_finish = False
        self.sandpit_jumping_signal_received = False
        self.sandpit_jumping_signal_stamp_sec = 0.0
        self.sandpit_finish_signal_received = False
        self.sandpit_finish_signal_stamp_sec = 0.0
        self.sandpit_last_step_once_command = None
        self.sandpit_last_step_once_yaw = None
        self.sandpit_wait_finish_serial_disconnect_active = False
        self.sandpit_wait_finish_serial_disconnect_stamp_sec = 0.0
        self.sandpit_wait_finish_serial_disconnect_had_jumping = False
        self._reset_sandpit_pre_align_state()
        self.pole_pre_bypass_pending = False
        self.pole_pre_bypass_reason = ''
        self.pole_completed_waypoints = 0

    def _run_done(self, now: float) -> None:
        if self._upstairs_done_retry_guard_active(now):
            self._publish_upstairs_done_retry_guard_override()
        else:
            if self.upstairs_done_retry_guard_deadline_sec is not None:
                self.upstairs_done_retry_guard_deadline_sec = None
                self._publish_feedback(
                    '上台阶结束保护小前进已维持 '
                    f'{self.upstairs_done_retry_guard_duration_sec:.1f}s，'
                    '保护窗口结束，恢复 DONE 保持。'
                )
            self._publish_done_hold_override()
        self._run_upstairs_step_once_ack_resend(now)

    def _run_failed(self) -> None:
        if not self.failed_hold_step_override_enabled:
            return
        self._publish_step_override(
            self.failed_hold_mode,
            self.failed_hold_left_norm,
            self.failed_hold_right_norm,
        )

    def _run_upstairs_step_once_ack_resend(self, now: float) -> None:
        if not self.upstairs_step_once_waiting_ack:
            return
        if self.upstairs_jumping_signal_received:
            self.upstairs_step_once_waiting_ack = False
            self.upstairs_step_once_next_resend_sec = None
            return
        if self.upstairs_step_once_next_resend_sec is None:
            return
        if now < self.upstairs_step_once_next_resend_sec:
            return
        if self.upstairs_step_once_resend_count >= self.upstairs_step_once_resend_max_count:
            self.upstairs_step_once_waiting_ack = False
            self.upstairs_step_once_next_resend_sec = None
            self._publish_feedback(
                '上台阶 step_once 等待 [jumping] 回包超时，'
                f'已达到最大重发次数 {self.upstairs_step_once_resend_max_count}，停止重发。'
            )
            return

        self._publish_step_once(
            self.upstairs_step_once_mode,
            self.upstairs_step_once_left_norm,
            self.upstairs_step_once_right_norm,
        )
        self.upstairs_step_once_resend_count += 1
        if self.upstairs_step_once_resend_count < self.upstairs_step_once_resend_max_count:
            self.upstairs_step_once_next_resend_sec = (
                now + self.upstairs_step_once_resend_after_sec
            )
        else:
            self.upstairs_step_once_next_resend_sec = None
        self._publish_feedback(
            '上台阶 step_once 等待 [jumping] 超过 '
            f'{self.upstairs_step_once_resend_after_sec:.1f}s，'
            f'重发 step_once=[{self.upstairs_step_once_mode},'
            f'{self.upstairs_step_once_left_norm:.3f},'
            f'{self.upstairs_step_once_right_norm:.3f}] '
            f'({self.upstairs_step_once_resend_count}/'
            f'{self.upstairs_step_once_resend_max_count})。'
        )

    def _publish_done_hold_override(self) -> None:
        if not self.done_hold_step_override_enabled:
            return
        self._publish_step_override(
            self.done_hold_mode,
            self.done_hold_left_norm,
            self.done_hold_right_norm,
        )

    def _upstairs_done_retry_guard_active(self, now: float) -> bool:
        return (
            self.upstairs_done_retry_guard_enabled
            and self.upstairs_done_retry_guard_deadline_sec is not None
            and now < self.upstairs_done_retry_guard_deadline_sec
        )

    def _publish_upstairs_done_retry_guard_override(self) -> None:
        self._publish_step_override(
            self.upstairs_done_retry_guard_mode,
            self.upstairs_done_retry_guard_left_norm,
            self.upstairs_done_retry_guard_right_norm,
        )

    def _run_search(self, now: float) -> None:
        left_norm = self.search_left_norm
        right_norm = self.search_right_norm
        yaw_error_deg = None
        if self.search_yaw_correction_enabled and self.pose_yaw is not None:
            target_yaw = self._search_target_yaw()
            if target_yaw is not None:
                yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
                yaw_error_deg = math.degrees(yaw_error)
                if abs(yaw_error) > self.search_yaw_tolerance_rad:
                    delta_norm = min(
                        self.search_yaw_max_delta_norm,
                        self.search_yaw_gain_per_rad * abs(yaw_error),
                    )
                    if yaw_error > 0.0:
                        left_norm = self._clamp_norm(left_norm - delta_norm)
                        right_norm = self._clamp_norm(right_norm + delta_norm)
                    else:
                        left_norm = self._clamp_norm(left_norm + delta_norm)
                        right_norm = self._clamp_norm(right_norm - delta_norm)
                else:
                    yaw_error_deg = 0.0
        self._publish_step_override(self.search_mode, left_norm, right_norm)

        if not self.search_yaw_correction_enabled or self.pose_yaw is None:
            return
        if self.search_yaw_log_interval_sec <= 0.0:
            return
        if (now - self.last_search_yaw_log_time) < self.search_yaw_log_interval_sec:
            return
        self.last_search_yaw_log_time = now
        if yaw_error_deg is None:
            self._publish_feedback(
                f'搜索中 yaw 修正等待 Odometry: override=[{self.search_mode},{left_norm:.3f},{right_norm:.3f}]。'
            )
        else:
            self._publish_feedback(
                f'搜索中 yaw_err={yaw_error_deg:.1f}deg, '
                f'override=[{self.search_mode},{left_norm:.3f},{right_norm:.3f}]。'
            )

    def _reset_sandpit_pre_align_state(self) -> None:
        self.sandpit_pre_align_locked = False
        self.sandpit_pre_align_start_sec = 0.0
        self.sandpit_pre_align_locked_distance_m = None
        self.sandpit_pre_align_locked_lateral_m = None
        self.sandpit_pre_align_start_x = None
        self.sandpit_pre_align_start_y = None
        self.sandpit_pre_align_start_yaw = None
        self.sandpit_pre_align_correction_kind = ''
        self.sandpit_pre_align_target_yaw = None
        self.sandpit_pre_align_pose_error_rad = 0.0
        self.sandpit_pre_align_locked_lateral_error_m = 0.0
        self.sandpit_pre_align_locked_center_angle_rad = 0.0
        self.sandpit_pre_align_locked_pose_metric = ''
        self.sandpit_pre_align_locked_pose_measured_rad = 0.0
        self.sandpit_pre_align_locked_pose_raw_error_rad = 0.0
        self.sandpit_pre_align_locked_pose_control_error_rad = 0.0
        self.sandpit_pre_align_completed_yaw = None
        self.sandpit_pre_align_fallback_active = False
        self.sandpit_pre_align_active_mode = self.sandpit_pre_align_mode
        self.sandpit_pre_align_active_hold_mode = self.sandpit_pre_align_hold_mode
        self.sandpit_pre_align_active_hold_left_norm = self.sandpit_pre_align_hold_left_norm
        self.sandpit_pre_align_active_hold_right_norm = self.sandpit_pre_align_hold_right_norm
        self.sandpit_pre_align_last_log_sec = 0.0
        self.sandpit_pre_align_overlay_active = False
        self.sandpit_pre_align_backup_active = False
        self.sandpit_pre_align_backup_start_distance_m = None
        self.sandpit_pre_align_polyline_phase = ''
        self.sandpit_pre_align_polyline_base_yaw = None
        self.sandpit_pre_align_polyline_side_yaw = None
        self.sandpit_pre_align_polyline_distance_m = 0.0
        self.sandpit_pre_align_polyline_raw_distance_m = 0.0
        self.sandpit_pre_align_polyline_start_x = None
        self.sandpit_pre_align_polyline_start_y = None
        self.sandpit_pre_align_polyline_start_sec = 0.0
        self.sandpit_pre_align_polyline_completed_once = False

    def _sandpit_pre_align_overlay_norms(
        self,
        now: float,
        left_norm: float,
        right_norm: float,
        *,
        source: str,
    ) -> tuple[float, float, str]:
        if not self._sandpit_pre_align_ready(now):
            self.sandpit_pre_align_overlay_active = False
            return left_norm, right_norm, ''
        lateral_m = self.sandpit_detection.lateral_m
        if lateral_m is None:
            self.sandpit_pre_align_overlay_active = False
            return left_norm, right_norm, ''
        tag_x_error_m = lateral_m - self.sandpit_pre_align_target_tag_x_m
        if not self.sandpit_pre_align_overlay_active:
            self.sandpit_pre_align_overlay_active = True
            self._publish_feedback(
                f'沙坑前预修正叠加已禁用: source={source}, '
                f'tag_dist={self._distance_text(self.sandpit_detection.distance_m)}, '
                f'tag_x={lateral_m:+.3f}m, '
                f'target={self.sandpit_pre_align_target_tag_x_m:+.3f}m, '
                f'x_err={tag_x_error_m:+.3f}m；横向修正统一交给独立折线预修正。'
            )
        overlay_text = (
            f', sandpit_overlay_disabled={source}: tag_dist='
            f'{self._distance_text(self.sandpit_detection.distance_m)}, '
            f'tag_x={lateral_m:+.3f}m'
        )
        return left_norm, right_norm, overlay_text

    def _enter_detected_branch(self, branch: str, now: float, reason: str) -> bool:
        if branch == 'upstairs':
            if self.upstairs_completed:
                self._publish_feedback('上台阶分支已经完成，本次触发被忽略，继续搜索后续障碍。')
                self._enter_search(now, reason='upstairs_already_completed')
                return True
            if self.upstairs_tag_trigger_enabled:
                if self._upstairs_tag_ready(now) and self._capture_upstairs_tag_reference_yaw(now):
                    self._enter_upstairs_align(now, reason, self._duck_reference_yaw_text())
                    return True
                fallback_ready, fallback_reason = (
                    self._upstairs_tag_visual_fallback_ready(now)
                )
                if fallback_ready:
                    self.upstairs_detection.distance_m = (
                        self.upstairs_visual_detection.distance_m
                    )
                    self.upstairs_detection.lateral_m = (
                        self.upstairs_visual_detection.lateral_m
                    )
                    self.upstairs_detection.nearest_stamp_sec = (
                        self.upstairs_visual_detection.nearest_stamp_sec
                    )
                    self._publish_feedback(
                        f'上台阶 AprilTag TF 缺失，启用近距离视觉兜底: '
                        f'{fallback_reason}；高墙已完成，直接发送上台阶 step_once，'
                        f'不再继续搜索前进。'
                    )
                    self._finish_upstairs_done(
                        now,
                        f'{reason}_tag_tf_missing_visual_fallback',
                    )
                    return True
                self._publish_feedback('上台阶 tag 触发但 TF/yaw 基准尚未锁定，继续搜索等待稳定样本。')
                self._enter_search(now, reason='upstairs_tag_waiting_tf')
                return True
            self._finish_upstairs_done(now, reason)
            return True

        if branch == 'limit_bar':
            if self.limit_bar_completed:
                self._publish_feedback('限高杆分支已经完成，本次触发被忽略，继续搜索后续障碍。')
                self._enter_search(now, reason='limit_bar_already_completed')
                return True
            if self.limit_bar_stop_nav_executor_on_entry_enabled:
                self._publish_nav_arrival(True)
            yaw_ref_text = 'unknown'
            if self._capture_duck_reference_yaw(now):
                yaw_ref_text = self._duck_reference_yaw_text()
            if (
                (self.duck_pre_align_enabled or self.limit_bar_tag_align_active)
                and self.duck_reference_yaw is not None
                and self.pose_yaw is not None
            ):
                self._enter_limit_bar_align(now, reason, yaw_ref_text)
            else:
                self._enter_limit_bar_prepare(now, reason, yaw_ref_text)
            return True

        if branch == 'hurdle':
            if self.hurdle_completed:
                self._publish_feedback('高墙跳跃分支已经完成，本次触发被忽略，继续搜索后续障碍。')
                self._enter_search(now, reason='hurdle_already_completed')
                return True
            if self._hurdle_tag_ready(now) and self._capture_hurdle_tag_reference_yaw(now):
                self._enter_hurdle_align(now, reason, self._duck_reference_yaw_text())
            else:
                self._enter_hurdle_wait_jump(now, reason)
            return True

        if branch == 'pole':
            if self.pole_completed:
                self._publish_feedback('绕杆分支已经完成，本次触发被忽略，继续搜索后续障碍。')
                self._enter_search(now, reason='pole_already_completed')
                return True
            if self._pole_apriltag_ready(now):
                reason = (
                    f'pole_apriltag_detected(id={self.pole_apriltag_target_tag_id},'
                    f'tf={self.pole_apriltag_frame_id}->{self.pole_apriltag_child_frame_id},'
                    f'source={self.pole_apriltag_ready_source or "unknown"})'
                )
            if self._should_pre_bypass_pole():
                self._enter_pole_branch(now, reason, pre_bypass=True)
                return True
            if self._should_use_pole_bypass():
                self._enter_pole_bypass(now, reason)
                return True
            self._enter_pole_branch(now, reason, pre_bypass=False)
            return True

        if branch == 'sandpit_pre_align':
            if self.sandpit_bypass_complete_once and self.sandpit_completed:
                self._publish_feedback('砂砾碎木坑绕行已经完成，本次 tag 预修正触发被忽略。')
                self._enter_search(now, reason='sandpit_pre_align_already_completed')
                return True
            self._enter_sandpit_pre_align(now, reason)
            return True

        if branch == 'sandpit':
            if self.sandpit_bypass_complete_once and self.sandpit_completed:
                self._publish_feedback('砂砾碎木坑绕行已经完成，本次 tag 触发被忽略，继续搜索后续障碍。')
                self._enter_search(now, reason='sandpit_already_completed')
                return True
            self._enter_sandpit_bypass(now, reason)
            return True

        if branch == 'slope':
            self._enter_slope_or_align(
                now,
                reason=reason,
                feedback=(
                    f'进入斜坡木桥分支: '
                    f'dist={self._distance_text(self.slope_detection.distance_m)}。'
                ),
            )
            return True

        if branch == 'slope_roll':
            roll_text = 'unknown'
            if self.pose_roll is not None:
                roll_text = f'{math.degrees(self.pose_roll):.1f}deg'
            self._enter_slope_or_align(
                now,
                reason=reason,
                feedback=(
                    f'按 Odometry roll 进入斜坡木桥分支: roll={roll_text}, '
                    f'threshold={math.degrees(self.slope_roll_detection_rad):.1f}deg。'
                ),
            )
            return True

        if branch == 'slope_fallback':
            self._enter_slope_or_align(
                now,
                reason=reason,
                feedback='斜坡视觉未稳定触发，按搜索超时优先回退到斜坡木桥分支。',
            )
            return True

        return False

    def _enter_sandpit_pre_align(
        self,
        now: float,
        reason: str,
        *,
        mode_override: Optional[int] = None,
    ) -> None:
        recent_samples = self._recent_sandpit_tag_samples(now)
        if len(recent_samples) < self.sandpit_pre_align_min_samples:
            self._publish_step_override(
                self.sandpit_pre_align_hold_mode,
                self.sandpit_pre_align_hold_left_norm,
                self.sandpit_pre_align_hold_right_norm,
            )
            self._publish_feedback(
                f'沙坑 ID{self.sandpit_tag_target_id} 预修正等待稳定样本: '
                f'{len(recent_samples)}/{self.sandpit_pre_align_min_samples}。'
            )
            return
        distance_m = self._median([sample.distance_m for sample in recent_samples])
        lateral_m = self._median([sample.x_m for sample in recent_samples])
        z_m = self._median([sample.z_m for sample in recent_samples])
        tag_x_error_m = lateral_m - self.sandpit_pre_align_target_tag_x_m
        center_distance_m = z_m if math.isfinite(z_m) and z_m > 1e-3 else distance_m
        center_angle_rad = math.atan2(tag_x_error_m, max(1e-3, center_distance_m))
        (
            pose_metric,
            pose_measured_rad,
            pose_raw_error_rad,
            pose_control_error_rad,
        ) = self._sandpit_pre_align_pose_error(recent_samples[-1])
        lateral_needed = self._sandpit_pre_align_lateral_needed(tag_x_error_m)
        yaw_needed = self._sandpit_pre_align_yaw_needed(pose_raw_error_rad)
        correction_kind = ''
        if lateral_needed:
            correction_kind = 'lateral'
        elif yaw_needed:
            correction_kind = 'yaw'
        else:
            self._publish_feedback(
                f'沙坑 ID{self.sandpit_tag_target_id} 预修正无需进入: '
                f'dist={distance_m:.3f}m, tag_x_err={tag_x_error_m:+.3f}m '
                f'< {self.sandpit_pre_align_lateral_skip_below_m:.3f}m, '
                f'{pose_metric}_err={math.degrees(pose_raw_error_rad):+.1f}deg '
                f'<= {math.degrees(self.sandpit_pre_align_yaw_tolerance_rad):.1f}deg。'
            )
            return
        self.sandpit_detection.distance_m = distance_m
        self.sandpit_detection.lateral_m = lateral_m
        self.sandpit_detection.nearest_stamp_sec = recent_samples[-1].received_stamp_sec
        self._clear_step_override()
        self.state = self.STATE_SANDPIT_PRE_ALIGN
        self.current_active_branch = 'sandpit_pre_align'
        self.active_bypass_kind = 'sandpit'
        self._reset_sandpit_pre_align_state()
        if mode_override is not None:
            self.sandpit_pre_align_active_mode = int(mode_override)
            self.sandpit_pre_align_active_hold_mode = int(mode_override)
            self.sandpit_pre_align_active_hold_left_norm = 0.0
            self.sandpit_pre_align_active_hold_right_norm = 0.0
        self.sandpit_pre_align_locked = True
        self.sandpit_pre_align_start_sec = now
        self.sandpit_pre_align_locked_distance_m = distance_m
        self.sandpit_pre_align_locked_lateral_m = lateral_m
        self.sandpit_pre_align_locked_lateral_error_m = tag_x_error_m
        self.sandpit_pre_align_locked_center_angle_rad = center_angle_rad
        self.sandpit_pre_align_locked_pose_metric = pose_metric
        self.sandpit_pre_align_locked_pose_measured_rad = pose_measured_rad
        self.sandpit_pre_align_locked_pose_raw_error_rad = pose_raw_error_rad
        self.sandpit_pre_align_locked_pose_control_error_rad = pose_control_error_rad
        self.sandpit_pre_align_pose_error_rad = pose_raw_error_rad
        self.sandpit_pre_align_correction_kind = correction_kind
        self.sandpit_pre_align_start_x = self.pose_x
        self.sandpit_pre_align_start_y = self.pose_y
        self.sandpit_pre_align_start_yaw = self.pose_yaw
        if correction_kind == 'yaw' and self.pose_yaw is not None:
            self.sandpit_pre_align_target_yaw = self._normalize_angle(
                self.pose_yaw + pose_control_error_rad
            )
        self._publish_state(self.state)
        self._publish_feedback(
            f'进入沙坑前 AprilTag 预修正: reason={reason}, '
            f'id={self.sandpit_tag_target_id}, samples={len(recent_samples)}, '
            f'dist={distance_m:.3f}m <= {self.sandpit_pre_align_start_distance_m:.3f}m, '
            f'handoff={self.sandpit_pre_align_handoff_distance_m:.3f}m, '
            f'tag_x={lateral_m:+.3f}m, target_x={self.sandpit_pre_align_target_tag_x_m:+.3f}m, '
            f'x_err={tag_x_error_m:+.3f}m, center={math.degrees(center_angle_rad):+.1f}deg, '
            f'{pose_metric}={math.degrees(pose_measured_rad):+.1f}deg, '
            f'pose_err={math.degrees(pose_raw_error_rad):+.1f}deg, '
            f'correction={correction_kind}, mode={self.sandpit_pre_align_active_mode}。'
        )

    def _enter_sandpit_bypass(self, now: float, reason: str) -> None:
        if not self._pose_is_fresh(now):
            if self.state == self.STATE_SANDPIT_PRE_ALIGN:
                self._publish_step_override(
                    self.sandpit_pre_align_active_hold_mode,
                    self.sandpit_pre_align_active_hold_left_norm,
                    self.sandpit_pre_align_active_hold_right_norm,
                )
            else:
                self._publish_step_override(0, 0.0, 0.0)
            self.search_arm_deadline = now + 0.20
            self._publish_feedback('砂砾碎木坑 tag 已触发，但当前没有新鲜 Odometry，先停止等待。')
            return
        pre_align_completed_yaw = self.sandpit_pre_align_completed_yaw
        self._clear_step_override()
        self._reset_sandpit_pre_align_state()
        self.state = self.STATE_SANDPIT_BYPASS
        self.current_active_branch = 'sandpit'
        self.active_bypass_kind = 'sandpit'
        self.phase_deadline_sec = None
        self.sandpit_phase_index = None
        self.sandpit_phase_start_x = None
        self.sandpit_phase_start_y = None
        self.sandpit_phase_start_yaw = None
        self.sandpit_phase_start_sec = 0.0
        self.sandpit_entry_yaw = (
            self._normalize_angle(pre_align_completed_yaw)
            if pre_align_completed_yaw is not None
            else self.pose_yaw
        )
        self.sandpit_phase_target_yaw = None
        self.sandpit_phase_target_distance_m = None
        self.sandpit_phase_deadline_sec = None
        self.sandpit_phase_stable_since = None
        self.sandpit_phase_last_log_sec = 0.0
        self.sandpit_phase_turn_direction_sign = 0
        self.sandpit_phase_turn_last_yaw = None
        self.sandpit_phase_turn_progress_rad = 0.0
        self.sandpit_phase_turn_target_delta_rad = None
        self.sandpit_phase_wait_for_finish = False
        self.sandpit_jumping_signal_received = False
        self.sandpit_jumping_signal_stamp_sec = 0.0
        self.sandpit_finish_signal_received = False
        self.sandpit_finish_signal_stamp_sec = 0.0
        self.sandpit_last_step_once_command = None
        self.sandpit_last_step_once_yaw = None
        self.sandpit_wait_finish_serial_disconnect_active = False
        self.sandpit_wait_finish_serial_disconnect_stamp_sec = 0.0
        self.sandpit_wait_finish_serial_disconnect_had_jumping = False
        self._publish_state(self.state)
        if self.sandpit_behavior == 'traverse':
            self._publish_feedback(
                f'进入砂坑穿越: reason={reason}, '
                f'tag_dist={self._distance_text(self.sandpit_detection.distance_m)} <= '
                f'{self.sandpit_trigger_distance_m:.3f}m，'
                f'entry_yaw={math.degrees(self.sandpit_entry_yaw):.1f}deg，'
                f'phases={len(self.sandpit_traverse_phases)}。'
            )
        else:
            self._publish_feedback(
                f'进入砂砾碎木坑绕行: reason={reason}, '
                f'tag_dist={self._distance_text(self.sandpit_detection.distance_m)} <= '
                f'{self.sandpit_trigger_distance_m:.3f}m，'
                f'turns={self.sandpit_turn_sequence_deg}, '
                f'straights={self.sandpit_straight_distances_m}m。'
            )
        self._start_sandpit_phase(0, now)

    def _should_use_pole_bypass(self) -> bool:
        if not self.pole_bypass_enabled:
            return False
        if self.pole_bypass_complete_once and self.pole_bypass_completed:
            return False
        if self.pole_bypass_force_on_detection:
            return True
        return self.pole_retry_failure_count >= self.pole_retry_failures_before_bypass

    def _should_pre_bypass_pole(self) -> bool:
        if not self.pole_bypass_enabled:
            return False
        if self.pole_bypass_complete_once and self.pole_bypass_completed:
            return False
        if self.pole_retry_failure_count >= self.pole_retry_failures_before_bypass:
            return False
        return self.pole_bypass_force_on_detection and self.pole_bypass_pre_bypass_waypoints > 0

    def _enter_pole_branch(self, now: float, reason: str, *, pre_bypass: bool) -> None:
        del now
        self._publish_nav_arrival(True)
        self._publish_step_override(0, 0.0, 0.0)
        self.state = self.STATE_POLE
        self.current_active_branch = 'pole'
        self.pole_state = 'STARTING'
        self.pole_completed_waypoints = 0
        self.pole_pre_bypass_pending = bool(pre_bypass)
        self.pole_pre_bypass_reason = reason if pre_bypass else ''
        self._publish_state(self.state)
        if pre_bypass:
            self._publish_feedback(
                f'进入绕杆预跑分支: reason={reason}, '
                f'dist={self._distance_text(self.pole_detection.distance_m)}, '
                f'pole_tag_dist={self._distance_text(self.pole_apriltag_detection.distance_m)}, '
                f'先完成前 {self.pole_bypass_pre_bypass_waypoints} 个 waypoint，'
                '再切入绕杆绕行。'
            )
            return
        self._publish_feedback(
            f'进入绕杆分支: reason={reason}, dist={self._distance_text(self.pole_detection.distance_m)}, '
            f'pole_tag_dist={self._distance_text(self.pole_apriltag_detection.distance_m)}, '
            f'retry_failures={self.pole_retry_failure_count}/'
            f'{self.pole_retry_failures_before_bypass}。'
        )

    def _enter_pole_bypass(self, now: float, reason: str) -> None:
        if not self._pose_is_fresh(now):
            self._publish_step_override(0, 0.0, 0.0)
            self.search_arm_deadline = now + 0.20
            self._publish_feedback('绕杆绕行已触发，但当前没有新鲜 Odometry，先停止等待。')
            return
        self._clear_step_override()
        self.state = self.STATE_SANDPIT_BYPASS
        self.current_active_branch = 'pole_bypass'
        self.active_bypass_kind = 'pole'
        self.phase_deadline_sec = None
        self.sandpit_phase_index = None
        self.sandpit_phase_start_x = None
        self.sandpit_phase_start_y = None
        self.sandpit_phase_start_yaw = None
        self.sandpit_phase_start_sec = 0.0
        self.sandpit_entry_yaw = None
        self.sandpit_phase_target_yaw = None
        self.sandpit_phase_target_distance_m = None
        self.sandpit_phase_deadline_sec = None
        self.sandpit_phase_stable_since = None
        self.sandpit_phase_last_log_sec = 0.0
        self.sandpit_phase_turn_direction_sign = 0
        self.sandpit_phase_turn_last_yaw = None
        self.sandpit_phase_turn_progress_rad = 0.0
        self.sandpit_phase_turn_target_delta_rad = None
        self.sandpit_phase_wait_for_finish = False
        self.sandpit_finish_signal_received = False
        self.sandpit_finish_signal_stamp_sec = 0.0
        self._publish_state(self.state)
        pre_bypass_done = '_pole_waypoints' in str(reason)
        if pre_bypass_done:
            prefix = f'绕杆预跑 {self.pole_bypass_pre_bypass_waypoints} 个 waypoint 后进入绕杆绕行'
        elif (
            self.pole_bypass_force_on_detection
            and self.pole_retry_failure_count < self.pole_retry_failures_before_bypass
        ):
            prefix = '识别到绕杆后强制进入绕杆绕行'
        else:
            prefix = f'绕杆连续重试失败 {self.pole_retry_failure_count} 次，进入绕杆绕行'
        self._publish_feedback(
            f'{prefix}: '
            f'reason={reason}, pole_dist={self._distance_text(self.pole_detection.distance_m)}, '
            f'turns={self.pole_bypass_turn_sequence_deg}, '
            f'straights={self.pole_bypass_straight_distances_m}m。'
        )
        self._start_sandpit_phase(0, now)

    def _sandpit_phase_specs(self) -> List[Dict[str, Any]]:
        if self.active_bypass_kind == 'sandpit' and self.sandpit_behavior == 'traverse':
            return self.sandpit_traverse_phases
        turn_sequence = self._active_bypass_turn_sequence_deg()
        straight_distances = self._active_bypass_straight_distances_m()
        return [
            {'kind': 'turn', 'yaw_delta_deg': turn_sequence[0]},
            {'kind': 'walk', 'distance_m': straight_distances[0]},
            {'kind': 'turn', 'yaw_delta_deg': turn_sequence[1]},
            {'kind': 'walk', 'distance_m': straight_distances[1]},
            {'kind': 'turn', 'yaw_delta_deg': turn_sequence[2]},
        ]

    def _sandpit_phase_total(self) -> int:
        return len(self._sandpit_phase_specs())

    def _sandpit_current_phase(self) -> Dict[str, Any]:
        phases = self._sandpit_phase_specs()
        index = 0 if self.sandpit_phase_index is None else self.sandpit_phase_index
        if index < 0 or index >= len(phases):
            return {}
        return phases[index]

    @staticmethod
    def _phase_float(phase: Dict[str, Any], key: str, default: float) -> float:
        return float(phase.get(key, default))

    @staticmethod
    def _phase_int(phase: Dict[str, Any], key: str, default: int) -> int:
        return int(phase.get(key, default))

    def _sandpit_phase_requires_pose(self, phase: Dict[str, Any]) -> bool:
        return str(phase.get('kind', '')).strip().lower() in ('align', 'turn', 'walk')

    def _sandpit_alignment_target_yaw(self, phase: Dict[str, Any]) -> float:
        source = str(phase.get('target_yaw_source', 'search_reference')).strip().lower()
        if source == 'absolute':
            base_yaw = math.radians(float(phase.get('target_yaw_deg', 0.0)))
        elif source in ('sandpit_entry', 'entry', 'sandpit_locked'):
            if self.sandpit_entry_yaw is not None:
                base_yaw = self.sandpit_entry_yaw
            elif 'target_yaw_deg' in phase:
                base_yaw = math.radians(float(phase.get('target_yaw_deg', 0.0)))
            else:
                base_yaw = self.pose_yaw
        elif source == 'startup':
            if self.startup_pose_yaw is not None:
                base_yaw = self.startup_pose_yaw
            elif 'target_yaw_deg' in phase:
                base_yaw = math.radians(float(phase.get('target_yaw_deg', 0.0)))
            else:
                base_yaw = self.pose_yaw
        elif source in ('last_step_once', 'pre_step_once', 'pre_jump', 'jump_start'):
            if self.sandpit_last_step_once_yaw is not None:
                base_yaw = self.sandpit_last_step_once_yaw
            else:
                base_yaw = self.pose_yaw
        elif source == 'current':
            base_yaw = self.pose_yaw
        else:
            if self.search_reference_yaw is not None:
                base_yaw = self.search_reference_yaw
            elif self.startup_pose_yaw is not None:
                base_yaw = self.startup_pose_yaw
            else:
                base_yaw = self.pose_yaw
        if base_yaw is None:
            base_yaw = 0.0
        delta_rad = math.radians(float(phase.get('target_yaw_delta_deg', 0.0)))
        return self._normalize_angle(base_yaw + delta_rad)

    def _start_sandpit_phase(self, phase_index: int, now: float) -> None:
        phases = self._sandpit_phase_specs()
        if phase_index >= len(phases):
            self._finish_sandpit_bypass(now)
            return
        phase = phases[phase_index]
        if self._sandpit_phase_requires_pose(phase) and not self._pose_is_fresh(now):
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_feedback(f'{self._active_bypass_label()}等待新鲜 Odometry。')
            return
        self.sandpit_phase_index = phase_index
        self.sandpit_phase_start_x = self.pose_x
        self.sandpit_phase_start_y = self.pose_y
        self.sandpit_phase_start_yaw = self.pose_yaw
        self.sandpit_phase_start_sec = now
        self.sandpit_phase_target_yaw = None
        self.sandpit_phase_target_distance_m = None
        self.sandpit_phase_deadline_sec = None
        self.sandpit_phase_stable_since = None
        self.sandpit_phase_last_log_sec = 0.0
        self.sandpit_phase_turn_direction_sign = 0
        self.sandpit_phase_turn_last_yaw = None
        self.sandpit_phase_turn_progress_rad = 0.0
        self.sandpit_phase_turn_target_delta_rad = None
        self.sandpit_phase_wait_for_finish = False
        self.sandpit_wait_finish_resend_after_sec = 0.0
        self.sandpit_wait_finish_next_resend_sec = None
        self.sandpit_wait_finish_resend_count = 0
        self.sandpit_wait_finish_resend_max_count = 0
        self.sandpit_wait_finish_serial_disconnect_active = False
        self.sandpit_wait_finish_serial_disconnect_stamp_sec = 0.0
        self.sandpit_wait_finish_serial_disconnect_had_jumping = False

        kind = str(phase['kind'])
        total = self._sandpit_phase_total()
        if kind == 'align':
            target_yaw = self._sandpit_alignment_target_yaw(phase)
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            self.sandpit_phase_target_yaw = target_yaw
            self.sandpit_phase_turn_direction_sign = 1 if yaw_error > 0.0 else -1 if yaw_error < 0.0 else 0
            self.sandpit_phase_turn_last_yaw = self.pose_yaw
            self.sandpit_phase_turn_target_delta_rad = abs(yaw_error)
            timeout_sec = max(0.0, self._phase_float(phase, 'timeout_sec', self._active_bypass_turn_timeout_sec()))
            self.sandpit_phase_deadline_sec = now + timeout_sec if timeout_sec > 0.0 else None
            self._publish_feedback(
                f'{self._active_bypass_label()} phase {phase_index + 1}/{total}: 角度对齐，'
                f'target_yaw={math.degrees(target_yaw):.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):.1f}deg。'
            )
            return
        if kind == 'turn':
            yaw_delta_deg = float(phase['yaw_delta_deg'])
            target_source = str(phase.get('target_yaw_source', 'current')).strip().lower()
            if target_source == 'current':
                self.sandpit_phase_target_yaw = self._normalize_angle(
                    self.pose_yaw + math.radians(yaw_delta_deg)
                )
            else:
                self.sandpit_phase_target_yaw = self._sandpit_alignment_target_yaw(
                    {
                        **phase,
                        'target_yaw_delta_deg': yaw_delta_deg,
                    }
                )
            if target_source == 'current':
                if yaw_delta_deg > 0.0:
                    self.sandpit_phase_turn_direction_sign = 1
                elif yaw_delta_deg < 0.0:
                    self.sandpit_phase_turn_direction_sign = -1
                else:
                    self.sandpit_phase_turn_direction_sign = 0
            else:
                yaw_error = self._normalize_angle(
                    self.sandpit_phase_target_yaw - self.pose_yaw
                )
                self.sandpit_phase_turn_direction_sign = 1 if yaw_error > 0.0 else -1 if yaw_error < 0.0 else 0
            self.sandpit_phase_turn_last_yaw = self.pose_yaw
            if target_source == 'current':
                self.sandpit_phase_turn_target_delta_rad = math.radians(abs(yaw_delta_deg))
            else:
                self.sandpit_phase_turn_target_delta_rad = abs(
                    self._normalize_angle(self.sandpit_phase_target_yaw - self.pose_yaw)
                )
            timeout_sec = max(0.0, self._phase_float(phase, 'timeout_sec', self._active_bypass_turn_timeout_sec()))
            self.sandpit_phase_deadline_sec = now + timeout_sec if timeout_sec > 0.0 else None
            direction = '逆时针' if yaw_delta_deg > 0.0 else '顺时针'
            self._publish_feedback(
                f'{self._active_bypass_label()} phase {phase_index + 1}/{total}: {direction}转 '
                f'{abs(yaw_delta_deg):.1f}deg, '
                f'target_yaw={math.degrees(self.sandpit_phase_target_yaw):.1f}deg。'
            )
            return
        if kind == 'walk':
            distance_m = max(0.0, float(phase['distance_m']))
            if 'target_yaw_source' in phase or 'target_yaw_delta_deg' in phase:
                self.sandpit_phase_target_yaw = self._sandpit_alignment_target_yaw(phase)
            else:
                self.sandpit_phase_target_yaw = self.pose_yaw
            self.sandpit_phase_target_distance_m = distance_m
            timeout_sec = (
                distance_m * self._phase_float(phase, 'timeout_sec_per_m', self._active_bypass_walk_timeout_sec_per_m())
                + self._phase_float(phase, 'timeout_extra_sec', self._active_bypass_walk_timeout_extra_sec())
            )
            self.sandpit_phase_deadline_sec = now + timeout_sec if timeout_sec > 0.0 else None
            self._publish_feedback(
                f'{self._active_bypass_label()} phase {phase_index + 1}/{total}: 沿当前机身前方直走 '
                f'{distance_m:.2f}m，mode={self._phase_int(phase, "mode", self._active_bypass_walk_mode())}, '
                f'起点=({self.pose_x:.3f},{self.pose_y:.3f}), '
                f'yaw_ref={math.degrees(self.sandpit_phase_target_yaw):.1f}deg。'
            )
            return
        if kind == 'wait':
            hold_sec = max(0.0, self._phase_float(phase, 'hold_sec', 0.0))
            self.sandpit_phase_deadline_sec = now + hold_sec
            self._publish_feedback(
                f'{self._active_bypass_label()} phase {phase_index + 1}/{total}: '
                f'稳定等待 {hold_sec:.2f}s。'
            )
            return
        if kind == 'step_once':
            if bool(phase.get('clear_finish_signal', False)):
                self.sandpit_jumping_signal_received = False
                self.sandpit_jumping_signal_stamp_sec = 0.0
                self.sandpit_finish_signal_received = False
                self.sandpit_finish_signal_stamp_sec = 0.0
            mode = self._phase_int(phase, 'mode', 0)
            left_norm = self._phase_float(phase, 'left_norm', 0.0)
            right_norm = self._phase_float(phase, 'right_norm', 0.0)
            self.sandpit_last_step_once_yaw = self.pose_yaw
            self._publish_step_once(mode, left_norm, right_norm)
            self.sandpit_last_step_once_command = (mode, left_norm, right_norm)
            hold_sec = max(0.0, self._phase_float(phase, 'hold_sec', 0.0))
            self.sandpit_phase_deadline_sec = now + hold_sec
            self._publish_feedback(
                f'{self._active_bypass_label()} phase {phase_index + 1}/{total}: '
                f'发送电控一次性命令 [{mode},{left_norm:.3f},{right_norm:.3f}]，'
                f'随后等待 {hold_sec:.2f}s。'
            )
            return
        if kind == 'wait_finish':
            self.sandpit_phase_wait_for_finish = True
            timeout_sec = max(0.0, self._phase_float(phase, 'timeout_sec', 0.0))
            self.sandpit_phase_deadline_sec = now + timeout_sec if timeout_sec > 0.0 else None
            self.sandpit_wait_finish_resend_after_sec = max(
                0.0,
                self._phase_float(phase, 'resend_after_sec', 3.0),
            )
            self.sandpit_wait_finish_resend_max_count = max(
                0,
                self._phase_int(phase, 'resend_max_count', 1),
            )
            self.sandpit_wait_finish_resend_count = 0
            self.sandpit_wait_finish_serial_disconnect_active = False
            self.sandpit_wait_finish_serial_disconnect_stamp_sec = 0.0
            self.sandpit_wait_finish_serial_disconnect_had_jumping = False
            if (
                self.sandpit_last_step_once_command is not None
                and self.sandpit_wait_finish_resend_after_sec > 0.0
                and self.sandpit_wait_finish_resend_max_count > 0
            ):
                self.sandpit_wait_finish_next_resend_sec = (
                    now + self.sandpit_wait_finish_resend_after_sec
                )
            else:
                self.sandpit_wait_finish_next_resend_sec = None
            require_jumping = bool(phase.get('require_jumping_before_finish', True))
            wait_target = 'jumping/finish' if require_jumping else 'finish'
            self._publish_feedback(
                f'{self._active_bypass_label()} phase {phase_index + 1}/{total}: 等待电控 [{wait_target}] 回包，'
                f'期间保持 [0,0,0]；若 {self.sandpit_wait_finish_resend_after_sec:.1f}s 未收到回包，'
                f'重发上一次 step_once 最多 {self.sandpit_wait_finish_resend_max_count} 次。'
            )
            return
        raise RuntimeError(f'unsupported sandpit phase kind: {kind}')

    def _advance_sandpit_phase(self, now: float, reason: str) -> None:
        index = 0 if self.sandpit_phase_index is None else self.sandpit_phase_index
        total = self._sandpit_phase_total()
        self._publish_feedback(f'{self._active_bypass_label()} phase {index + 1}/{total} 完成: {reason}')
        self._start_sandpit_phase(index + 1, now)

    def _sandpit_pre_align_polyline_ready(
        self,
        now: float,
        tag_x_error_m: float,
        center_angle_rad: float,
    ) -> bool:
        if self.sandpit_pre_align_polyline_phase:
            return True
        if self.sandpit_pre_align_polyline_completed_once:
            return False
        if not self._pose_is_fresh(now):
            self._publish_step_override(
                self.sandpit_pre_align_active_hold_mode,
                self.sandpit_pre_align_active_hold_left_norm,
                self.sandpit_pre_align_active_hold_right_norm,
            )
            if (
                self.sandpit_pre_align_log_interval_sec > 0.0
                and (now - self.sandpit_pre_align_last_log_sec)
                >= self.sandpit_pre_align_log_interval_sec
            ):
                self.sandpit_pre_align_last_log_sec = now
                self._publish_feedback('沙坑前折线预修正等待新鲜 Odometry，暂不锁定折线路径。')
            return True

        use_large_angle = (
            self.sandpit_pre_align_polyline_large_angle_rad
            > self.sandpit_pre_align_polyline_angle_rad
            and (
                abs(tag_x_error_m)
                >= self.sandpit_pre_align_polyline_large_angle_error_m
                or abs(center_angle_rad)
                >= self.sandpit_pre_align_polyline_large_angle_center_rad
            )
        )
        yaw_abs_rad = (
            self.sandpit_pre_align_polyline_large_angle_rad
            if use_large_angle
            else self.sandpit_pre_align_polyline_angle_rad
        )
        sin_yaw = math.sin(yaw_abs_rad)
        if yaw_abs_rad <= 1e-6 or sin_yaw <= 1e-6:
            return False
        abs_error_m = abs(tag_x_error_m)
        if abs_error_m <= max(
            self.sandpit_pre_align_deadband_m,
            self.sandpit_pre_align_lateral_skip_below_m,
        ):
            return False

        raw_distance_m = abs_error_m / sin_yaw
        requested_distance_m = (
            raw_distance_m * self.sandpit_pre_align_polyline_distance_scale
        )
        distance_m = max(
            self.sandpit_pre_align_polyline_min_distance_m,
            requested_distance_m,
        )
        if self.sandpit_pre_align_polyline_max_distance_m > 0.0:
            distance_m = min(
                distance_m,
                self.sandpit_pre_align_polyline_max_distance_m,
            )

        side_delta_rad = -math.copysign(yaw_abs_rad, center_angle_rad)
        base_yaw = self._normalize_angle(self.pose_yaw)
        side_yaw = self._normalize_angle(base_yaw + side_delta_rad)

        self.sandpit_pre_align_polyline_phase = 'align_out'
        self.sandpit_pre_align_polyline_base_yaw = base_yaw
        self.sandpit_pre_align_polyline_side_yaw = side_yaw
        self.sandpit_pre_align_polyline_distance_m = distance_m
        self.sandpit_pre_align_polyline_raw_distance_m = raw_distance_m
        self.sandpit_pre_align_polyline_start_x = None
        self.sandpit_pre_align_polyline_start_y = None
        self.sandpit_pre_align_polyline_start_sec = now
        clipped = distance_m + 1e-6 < requested_distance_m
        self._publish_feedback(
            f'沙坑前折线预修正计划锁定: tag_x_err={tag_x_error_m:+.3f}m, '
            f'center={math.degrees(center_angle_rad):+.1f}deg, '
            f'base_yaw={math.degrees(base_yaw):.1f}deg, '
            f'side_delta={math.degrees(side_delta_rad):+.1f}deg, '
            f'side_yaw={math.degrees(side_yaw):.1f}deg, '
            f'angle_mode={"large" if use_large_angle else "normal"}, '
            f'shift={distance_m:.3f}m(raw={raw_distance_m:.3f}m, '
            f'requested={requested_distance_m:.3f}m, clipped={clipped})。'
        )
        return True

    def _sandpit_pre_align_should_backup(
        self,
        distance: float,
        tag_x_error_m: float,
        center_angle_rad: float,
    ) -> tuple[bool, str]:
        abs_error_m = abs(tag_x_error_m)
        abs_center_rad = abs(center_angle_rad)
        direct_polyline_ok = (
            abs_error_m >= self.sandpit_pre_align_backup_skip_lateral_error_m
            or abs_center_rad >= self.sandpit_pre_align_backup_skip_center_angle_rad
        )
        clearance_m = distance - self.sandpit_pre_align_handoff_distance_m
        direct_clearance_ok = (
            direct_polyline_ok
            and clearance_m >= self.sandpit_pre_align_backup_min_polyline_clearance_m
        )
        if direct_clearance_ok:
            reason = (
                f'direct_polyline_clearance={clearance_m:.3f}m, '
                f'x_err={abs_error_m:.3f}m, center={math.degrees(abs_center_rad):.1f}deg'
            )
            return False, reason
        if self.sandpit_pre_align_backup_active:
            return True, 'active'
        if distance > self.sandpit_pre_align_backup_trigger_distance_m:
            return False, 'far_enough'
        if (
            self.sandpit_pre_align_backup_force_distance_m > 0.0
            and distance <= self.sandpit_pre_align_backup_force_distance_m
        ):
            return True, 'force_close'
        return True, 'near_and_no_clearance'

    def _sandpit_pre_align_polyline_progress(
        self,
        yaw_rad: float,
    ) -> tuple[Optional[float], Optional[float]]:
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.sandpit_pre_align_polyline_start_x is None
            or self.sandpit_pre_align_polyline_start_y is None
        ):
            return None, None
        dx = self.pose_x - self.sandpit_pre_align_polyline_start_x
        dy = self.pose_y - self.sandpit_pre_align_polyline_start_y
        forward_x, forward_y = self._odom_forward_unit(yaw_rad)
        raw_m = dx * forward_x + dy * forward_y
        return math.hypot(dx, dy), raw_m

    def _sandpit_pre_align_turn_override(
        self,
        yaw_error: float,
    ) -> tuple[int, float, float]:
        if yaw_error >= 0.0:
            left_norm = self.sandpit_pre_align_yaw_ccw_left_norm
            right_norm = self.sandpit_pre_align_yaw_ccw_right_norm
        else:
            left_norm = self.sandpit_pre_align_yaw_cw_left_norm
            right_norm = self.sandpit_pre_align_yaw_cw_right_norm
        yaw_abs = abs(yaw_error)
        if self.sandpit_pre_align_polyline_turn_fine_threshold_rad > 1e-6:
            scale = min(
                1.0,
                max(
                    self.sandpit_pre_align_polyline_turn_min_scale,
                    yaw_abs / self.sandpit_pre_align_polyline_turn_fine_threshold_rad,
                ),
            )
            left_norm = self._clamp_norm(left_norm * scale)
            right_norm = self._clamp_norm(right_norm * scale)
        return self.sandpit_pre_align_yaw_turn_mode, left_norm, right_norm

    def _sandpit_pre_align_walk_yaw_corrected_norms(
        self,
        yaw_error: float,
    ) -> tuple[float, float, float]:
        left_norm = self.sandpit_pre_align_polyline_walk_left_norm
        right_norm = self.sandpit_pre_align_polyline_walk_right_norm
        if not self.sandpit_pre_align_polyline_yaw_correction_enabled:
            return left_norm, right_norm, 0.0
        yaw_abs = abs(yaw_error)
        if yaw_abs <= self.sandpit_pre_align_polyline_turn_tolerance_rad:
            return left_norm, right_norm, 0.0
        delta_norm = min(
            self.sandpit_pre_align_polyline_yaw_max_delta_norm,
            self.sandpit_pre_align_polyline_yaw_gain_per_rad
            * (yaw_abs - self.sandpit_pre_align_polyline_turn_tolerance_rad),
        )
        if yaw_error > 0.0:
            left_norm = self._clamp_norm(left_norm - delta_norm)
            right_norm = self._clamp_norm(right_norm + delta_norm)
        else:
            left_norm = self._clamp_norm(left_norm + delta_norm)
            right_norm = self._clamp_norm(right_norm - delta_norm)
        return left_norm, right_norm, delta_norm

    def _sandpit_pre_align_fallback_progress(self) -> tuple[Optional[float], Optional[float]]:
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.sandpit_pre_align_start_x is None
            or self.sandpit_pre_align_start_y is None
            or self.sandpit_pre_align_start_yaw is None
            or self.sandpit_pre_align_locked_distance_m is None
        ):
            return None, None
        forward_x, forward_y = self._odom_forward_unit(self.sandpit_pre_align_start_yaw)
        dx = self.pose_x - self.sandpit_pre_align_start_x
        dy = self.pose_y - self.sandpit_pre_align_start_y
        along_m = dx * forward_x + dy * forward_y
        estimated_distance_m = self.sandpit_pre_align_locked_distance_m - max(0.0, along_m)
        return along_m, max(0.0, estimated_distance_m)

    def _sandpit_pre_align_pose_error(self, sample: SandpitTagSample) -> tuple[str, float, float, float]:
        if self.sandpit_pre_align_yaw_pose_axis == 'pose_roll':
            measured = sample.roll_rad
            metric = 'pose_roll'
        elif self.sandpit_pre_align_yaw_pose_axis == 'pose_yaw':
            measured = sample.yaw_rad
            metric = 'pose_yaw'
        else:
            measured = sample.pitch_rad
            metric = 'pose_pitch'
        raw_error = self._normalize_angle(
            measured - self.sandpit_pre_align_yaw_pose_target_rad
        )
        control_error = self.sandpit_pre_align_yaw_pose_control_sign * raw_error
        return metric, measured, raw_error, control_error

    def _sandpit_pre_align_lateral_needed(self, tag_x_error_m: float) -> bool:
        threshold_m = self.sandpit_pre_align_lateral_skip_below_m
        if threshold_m <= 0.0:
            threshold_m = self.sandpit_pre_align_deadband_m
        return abs(tag_x_error_m) >= threshold_m

    def _sandpit_pre_align_yaw_needed(self, pose_raw_error_rad: float) -> bool:
        if not self.sandpit_pre_align_yaw_center_enabled:
            return False
        abs_error_rad = abs(pose_raw_error_rad)
        if abs_error_rad <= self.sandpit_pre_align_yaw_tolerance_rad:
            return False
        return (
            self.sandpit_pre_align_yaw_max_angle_rad <= 0.0
            or abs_error_rad <= self.sandpit_pre_align_yaw_max_angle_rad
        )

    def _sandpit_pre_align_locked_yaw_done(self, now: float, distance: Optional[float]) -> bool:
        recent_samples = self._recent_sandpit_tag_samples(now)
        pose_metric = self.sandpit_pre_align_locked_pose_metric or self.sandpit_pre_align_yaw_pose_axis
        pose_measured_rad = self.sandpit_pre_align_locked_pose_measured_rad
        pose_raw_error_rad = self.sandpit_pre_align_locked_pose_raw_error_rad
        pose_control_error_rad = self.sandpit_pre_align_locked_pose_control_error_rad
        if recent_samples:
            (
                pose_metric,
                pose_measured_rad,
                pose_raw_error_rad,
                pose_control_error_rad,
            ) = self._sandpit_pre_align_pose_error(recent_samples[-1])
            self.sandpit_pre_align_locked_pose_metric = pose_metric
            self.sandpit_pre_align_locked_pose_measured_rad = pose_measured_rad
            self.sandpit_pre_align_locked_pose_raw_error_rad = pose_raw_error_rad
            self.sandpit_pre_align_locked_pose_control_error_rad = pose_control_error_rad

        if self.sandpit_pre_align_target_yaw is None:
            if (
                self.sandpit_pre_align_correction_kind == 'yaw'
                and self._pose_is_fresh(now)
                and self.pose_yaw is not None
            ):
                self.sandpit_pre_align_target_yaw = self._normalize_angle(
                    self.pose_yaw
                    + self.sandpit_pre_align_locked_pose_control_error_rad
                )
                self._publish_feedback(
                    f'沙坑前 yaw 预修正锁定目标 yaw: '
                    f'pose_yaw={math.degrees(self.pose_yaw):.1f}deg, '
                    f'control={math.degrees(self.sandpit_pre_align_locked_pose_control_error_rad):+.1f}deg, '
                    f'target_yaw={math.degrees(self.sandpit_pre_align_target_yaw):.1f}deg。'
                )
            else:
                self._publish_step_override(
                    self.sandpit_pre_align_active_hold_mode,
                    self.sandpit_pre_align_active_hold_left_norm,
                    self.sandpit_pre_align_active_hold_right_norm,
                )
                if (
                    self.sandpit_pre_align_log_interval_sec > 0.0
                    and (now - self.sandpit_pre_align_last_log_sec)
                    >= self.sandpit_pre_align_log_interval_sec
                ):
                    self.sandpit_pre_align_last_log_sec = now
                    self._publish_feedback('沙坑前 yaw 预修正等待新鲜 Odometry 后锁定目标 yaw。')
                return False
        if not self._pose_is_fresh(now):
            self._publish_step_override(
                self.sandpit_pre_align_active_hold_mode,
                self.sandpit_pre_align_active_hold_left_norm,
                self.sandpit_pre_align_active_hold_right_norm,
            )
            if (
                self.sandpit_pre_align_log_interval_sec > 0.0
                and (now - self.sandpit_pre_align_last_log_sec)
                >= self.sandpit_pre_align_log_interval_sec
            ):
                self.sandpit_pre_align_last_log_sec = now
                self._publish_feedback('沙坑前 yaw 预修正等待新鲜 Odometry，保持不切模式。')
            return False

        yaw_error = self._normalize_angle(self.sandpit_pre_align_target_yaw - self.pose_yaw)
        visual_feedback_active = bool(recent_samples)
        aligned = (
            abs(pose_raw_error_rad) <= self.sandpit_pre_align_yaw_tolerance_rad
            if visual_feedback_active
            else abs(yaw_error) <= self.sandpit_pre_align_yaw_tolerance_rad
        )
        if aligned:
            self.sandpit_pre_align_correction_kind = 'approach'
            self.sandpit_pre_align_completed_yaw = self._normalize_angle(
                self.pose_yaw if visual_feedback_active else self.sandpit_pre_align_target_yaw
            )
            if distance is not None and distance <= self.sandpit_pre_align_handoff_distance_m:
                self._update_detection_trigger(self.sandpit_detection, True)
                self._publish_feedback(
                    f'沙坑前 yaw 预修正完成且已到 handoff: metric={pose_metric}, '
                    f'visual_err={math.degrees(pose_raw_error_rad):+.1f}deg, '
                    f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                    f'dist={distance:.3f}m <= {self.sandpit_pre_align_handoff_distance_m:.3f}m，'
                    '直接切入原沙坑流程。'
                )
                self._enter_sandpit_bypass(now, reason='sandpit_pre_align_yaw_done_handoff')
                return True
            self._publish_step_override(
                self.sandpit_pre_align_active_mode,
                self.sandpit_pre_align_left_norm,
                self.sandpit_pre_align_right_norm,
            )
            self._publish_feedback(
                f'沙坑前 yaw 预修正完成: metric={pose_metric}, '
                f'visual_err={math.degrees(pose_raw_error_rad):+.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                f'target_yaw={math.degrees(self.sandpit_pre_align_target_yaw):.1f}deg, '
                f'entry_base_yaw={math.degrees(self.sandpit_pre_align_completed_yaw):.1f}deg, '
                f'dist={self._distance_text(distance)}，立即低速接近 handoff，'
                f'override=[{self.sandpit_pre_align_active_mode},'
                f'{self.sandpit_pre_align_left_norm:.3f},'
                f'{self.sandpit_pre_align_right_norm:.3f}]。'
            )
            return True

        elapsed_sec = now - self.sandpit_pre_align_start_sec
        if (
            self.sandpit_pre_align_yaw_timeout_sec > 0.0
            and elapsed_sec >= self.sandpit_pre_align_yaw_timeout_sec
        ):
            self.sandpit_pre_align_correction_kind = 'approach'
            self.sandpit_pre_align_completed_yaw = self._normalize_angle(self.pose_yaw)
            if distance is not None and distance <= self.sandpit_pre_align_handoff_distance_m:
                self._update_detection_trigger(self.sandpit_detection, True)
                self._publish_feedback(
                    f'沙坑前 yaw 预修正超时且已到 handoff: metric={pose_metric}, '
                    f'visual_err={math.degrees(pose_raw_error_rad):+.1f}deg, '
                    f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                    f'elapsed={elapsed_sec:.1f}/{self.sandpit_pre_align_yaw_timeout_sec:.1f}s, '
                    f'dist={distance:.3f}m <= {self.sandpit_pre_align_handoff_distance_m:.3f}m，'
                    '切入原沙坑流程。'
                )
                self._enter_sandpit_bypass(now, reason='sandpit_pre_align_yaw_timeout_handoff')
                return True
            self._publish_step_override(
                self.sandpit_pre_align_active_mode,
                self.sandpit_pre_align_left_norm,
                self.sandpit_pre_align_right_norm,
            )
            self._publish_feedback(
                f'沙坑前 yaw 预修正超时，转为低速接近 handoff: metric={pose_metric}, '
                f'visual_err={math.degrees(pose_raw_error_rad):+.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                f'elapsed={elapsed_sec:.1f}/{self.sandpit_pre_align_yaw_timeout_sec:.1f}s, '
                f'dist={self._distance_text(distance)}, '
                f'entry_base_yaw={math.degrees(self.sandpit_pre_align_completed_yaw):.1f}deg, '
                f'override=[{self.sandpit_pre_align_active_mode},'
                f'{self.sandpit_pre_align_left_norm:.3f},'
                f'{self.sandpit_pre_align_right_norm:.3f}]。'
            )
            return True

        command_error = pose_control_error_rad if visual_feedback_active else yaw_error
        mode, left_norm, right_norm = self._sandpit_pre_align_turn_override(command_error)
        self._publish_step_override(mode, left_norm, right_norm)
        if (
            self.sandpit_pre_align_log_interval_sec > 0.0
            and (now - self.sandpit_pre_align_last_log_sec)
            >= self.sandpit_pre_align_log_interval_sec
        ):
            self.sandpit_pre_align_last_log_sec = now
            self._publish_feedback(
                f'沙坑前 yaw 预修正锁存执行: metric={self.sandpit_pre_align_locked_pose_metric}, '
                f'measured={math.degrees(pose_measured_rad):+.1f}deg, '
                f'raw_err={math.degrees(pose_raw_error_rad):+.1f}deg, '
                f'control={math.degrees(pose_control_error_rad):+.1f}deg, '
                f'target_yaw={math.degrees(self.sandpit_pre_align_target_yaw):.1f}deg, '
                f'pose_yaw={math.degrees(self.pose_yaw):.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                f'feedback={"visual" if visual_feedback_active else "odom"}, '
                f'override=[{mode},{left_norm:.3f},{right_norm:.3f}]。'
            )
        return False

    def _run_sandpit_pre_align_polyline(
        self,
        now: float,
        sample_count: int,
        distance: Optional[float],
        lateral_m: float,
        center_angle_rad: float,
    ) -> None:
        if (
            not self._pose_is_fresh(now)
            or self.sandpit_pre_align_polyline_base_yaw is None
            or self.sandpit_pre_align_polyline_side_yaw is None
        ):
            self._publish_step_override(
                self.sandpit_pre_align_active_hold_mode,
                self.sandpit_pre_align_active_hold_left_norm,
                self.sandpit_pre_align_active_hold_right_norm,
            )
            self._publish_feedback('沙坑前折线预修正等待新鲜 Odometry/yaw 基准。')
            return

        phase = self.sandpit_pre_align_polyline_phase
        target_yaw = self.sandpit_pre_align_polyline_base_yaw
        yaw_error = 0.0
        along_m: Optional[float] = None
        raw_along_m: Optional[float] = None
        remaining_m = 0.0
        yaw_corr_norm = 0.0
        mode = self.sandpit_pre_align_active_mode
        left_norm = self.sandpit_pre_align_active_hold_left_norm
        right_norm = self.sandpit_pre_align_active_hold_right_norm
        label = phase

        if phase == 'align_out':
            target_yaw = self.sandpit_pre_align_polyline_side_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            elapsed_sec = now - self.sandpit_pre_align_polyline_start_sec
            if abs(yaw_error) <= self.sandpit_pre_align_polyline_turn_tolerance_rad:
                self.sandpit_pre_align_polyline_phase = 'shift'
                self.sandpit_pre_align_polyline_start_x = self.pose_x
                self.sandpit_pre_align_polyline_start_y = self.pose_y
                self.sandpit_pre_align_polyline_start_sec = now
                self._publish_step_override(0, 0.0, 0.0)
                self._publish_feedback(
                    f'沙坑前折线预修正: 已转到斜向 yaw={math.degrees(target_yaw):.1f}deg，'
                    f'yaw_err={math.degrees(yaw_error):.1f}deg，开始直走 '
                    f'{self.sandpit_pre_align_polyline_distance_m:.3f}m。'
                )
                return
            if (
                self.sandpit_pre_align_polyline_turn_timeout_sec > 0.0
                and elapsed_sec >= self.sandpit_pre_align_polyline_turn_timeout_sec
            ):
                self._publish_feedback(
                    f'沙坑前折线预修正转向未到位，继续转向不切横移: '
                    f'phase=align_out, yaw_err={math.degrees(yaw_error):.1f}deg, '
                    f'tolerance={math.degrees(self.sandpit_pre_align_polyline_turn_tolerance_rad):.1f}deg, '
                    f'elapsed={elapsed_sec:.1f}s。'
                )
            mode, left_norm, right_norm = self._sandpit_pre_align_turn_override(
                yaw_error
            )

        elif phase == 'shift':
            target_yaw = self.sandpit_pre_align_polyline_side_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            along_m, raw_along_m = self._sandpit_pre_align_polyline_progress(
                target_yaw
            )
            if along_m is None:
                self._publish_step_override(0, 0.0, 0.0)
                self._publish_feedback('沙坑前折线预修正无法计算斜向移动里程，停止等待 Odometry。')
                return
            remaining_m = max(
                0.0,
                self.sandpit_pre_align_polyline_distance_m - along_m,
            )
            walk_timeout_sec = (
                self.sandpit_pre_align_polyline_distance_m
                * self.sandpit_pre_align_polyline_walk_timeout_sec_per_m
                + self.sandpit_pre_align_polyline_walk_timeout_extra_sec
            )
            elapsed_sec = now - self.sandpit_pre_align_polyline_start_sec
            if remaining_m <= self.sandpit_pre_align_deadband_m or (
                walk_timeout_sec > 0.0 and elapsed_sec >= walk_timeout_sec
            ):
                self.sandpit_pre_align_polyline_phase = 'align_back'
                self.sandpit_pre_align_polyline_start_sec = now
                self._publish_step_override(0, 0.0, 0.0)
                self._publish_feedback(
                    f'沙坑前折线预修正: 斜向移动完成 along={along_m:.3f}m'
                    f'(raw={raw_along_m:.3f}m)，开始转回基准 yaw='
                    f'{math.degrees(self.sandpit_pre_align_polyline_base_yaw):.1f}deg。'
                )
                return
            if (
                self.sandpit_pre_align_polyline_shift_yaw_guard_rad > 0.0
                and abs(yaw_error)
                > self.sandpit_pre_align_polyline_shift_yaw_guard_rad
            ):
                label = 'shift_yaw_guard'
                mode, left_norm, right_norm = self._sandpit_pre_align_turn_override(
                    yaw_error
                )
            else:
                left_norm, right_norm, yaw_corr_norm = (
                    self._sandpit_pre_align_walk_yaw_corrected_norms(yaw_error)
                )
                mode = self.sandpit_pre_align_active_mode

        elif phase == 'align_back':
            target_yaw = self.sandpit_pre_align_polyline_base_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            elapsed_sec = now - self.sandpit_pre_align_polyline_start_sec
            if abs(yaw_error) <= self.sandpit_pre_align_polyline_turn_tolerance_rad:
                self.sandpit_pre_align_polyline_phase = ''
                self.sandpit_pre_align_polyline_base_yaw = None
                self.sandpit_pre_align_polyline_side_yaw = None
                self.sandpit_pre_align_polyline_start_x = None
                self.sandpit_pre_align_polyline_start_y = None
                self.sandpit_pre_align_polyline_start_sec = now
                self.sandpit_pre_align_polyline_completed_once = True
                self._publish_step_override(
                    self.sandpit_pre_align_active_hold_mode,
                    self.sandpit_pre_align_active_hold_left_norm,
                    self.sandpit_pre_align_active_hold_right_norm,
                )
                self._publish_feedback(
                    f'沙坑前折线预修正完成: yaw_err={math.degrees(yaw_error):.1f}deg, '
                    f'tag_x={lateral_m:+.3f}m, dist={self._distance_text(distance)}；'
                    '继续按 ID10 距离进入原沙坑流程。'
                )
                return
            if (
                self.sandpit_pre_align_polyline_turn_timeout_sec > 0.0
                and elapsed_sec >= self.sandpit_pre_align_polyline_turn_timeout_sec
            ):
                self._publish_feedback(
                    f'沙坑前折线预修正转回未到位，继续转向不结束: '
                    f'phase=align_back, yaw_err={math.degrees(yaw_error):.1f}deg, '
                    f'tolerance={math.degrees(self.sandpit_pre_align_polyline_turn_tolerance_rad):.1f}deg, '
                    f'elapsed={elapsed_sec:.1f}s。'
                )
            mode, left_norm, right_norm = self._sandpit_pre_align_turn_override(
                yaw_error
            )

        else:
            self.sandpit_pre_align_polyline_phase = ''
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_feedback(f'沙坑前折线预修正未知 phase={phase}，已重置。')
            return

        self._publish_step_override(mode, left_norm, right_norm)
        if (
            self.sandpit_pre_align_log_interval_sec > 0.0
            and (now - self.sandpit_pre_align_last_log_sec)
            >= self.sandpit_pre_align_log_interval_sec
        ):
            self.sandpit_pre_align_last_log_sec = now
            along_text = 'unknown' if along_m is None else f'{along_m:.3f}m'
            raw_text = 'unknown' if raw_along_m is None else f'{raw_along_m:.3f}m'
            self._publish_feedback(
                f'沙坑前折线预修正中: phase={phase}/{label}, samples={sample_count}, '
                f'dist={self._distance_text(distance)}, tag_x={lateral_m:+.3f}m, '
                f'center={math.degrees(center_angle_rad):+.1f}deg, '
                f'target_yaw={math.degrees(target_yaw):.1f}deg, '
                f'pose_yaw={math.degrees(self.pose_yaw):.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):.1f}deg, along={along_text}, '
                f'raw_along={raw_text}, remaining={remaining_m:.3f}m, '
                f'yaw_corr={yaw_corr_norm:.3f}, override=[{mode},{left_norm:.3f},{right_norm:.3f}]。'
            )

    def _run_sandpit_pre_align(self, now: float) -> None:
        recent_samples = self._recent_sandpit_tag_samples(now)
        if not recent_samples:
            if (
                self.sandpit_pre_align_polyline_enabled
                and self.sandpit_pre_align_polyline_phase
            ):
                self._run_sandpit_pre_align_polyline(
                    now,
                    0,
                    self._usable_distance(self.sandpit_detection.distance_m),
                    (
                        self.sandpit_detection.lateral_m
                        if self.sandpit_detection.lateral_m is not None
                        else self.sandpit_pre_align_target_tag_x_m
                    ),
                    0.0,
                )
                return
            along_m, estimated_distance_m = self._sandpit_pre_align_fallback_progress()
            if self.sandpit_pre_align_correction_kind == 'yaw':
                if self._sandpit_pre_align_locked_yaw_done(now, estimated_distance_m):
                    return
                return
            if (
                self.sandpit_pre_align_locked
                and self._pose_is_fresh(now)
                and estimated_distance_m is not None
            ):
                if estimated_distance_m <= self.sandpit_pre_align_handoff_distance_m:
                    self._update_detection_trigger(self.sandpit_detection, True)
                    self._publish_feedback(
                        f'沙坑前预修正 ID{self.sandpit_tag_target_id} TF 丢失，'
                        f'按 Odometry 估算到达 handoff: '
                        f'est_dist={estimated_distance_m:.3f}m <= '
                        f'{self.sandpit_pre_align_handoff_distance_m:.3f}m, '
                        f'along={along_m:.3f}m，切入原沙坑流程。'
                    )
                    self._enter_sandpit_bypass(now, reason='sandpit_pre_align_odom_handoff')
                    return
                self.sandpit_pre_align_fallback_active = True
                self._publish_step_override(
                    self.sandpit_pre_align_active_mode,
                    self.sandpit_pre_align_left_norm,
                    self.sandpit_pre_align_right_norm,
                )
                if (
                    self.sandpit_pre_align_log_interval_sec > 0.0
                    and (now - self.sandpit_pre_align_last_log_sec)
                    >= self.sandpit_pre_align_log_interval_sec
                ):
                    self.sandpit_pre_align_last_log_sec = now
                    self._publish_feedback(
                        f'沙坑前预修正 ID{self.sandpit_tag_target_id} TF 暂丢，'
                        f'按 Odometry 继续低速接近: '
                        f'est_dist={estimated_distance_m:.3f}m -> '
                        f'{self.sandpit_pre_align_handoff_distance_m:.3f}m, '
                        f'along={along_m:.3f}m, '
                        f'override=[{self.sandpit_pre_align_active_mode},'
                        f'{self.sandpit_pre_align_left_norm:.3f},'
                        f'{self.sandpit_pre_align_right_norm:.3f}]。'
                    )
                return
            self._publish_step_override(
                self.sandpit_pre_align_active_hold_mode,
                self.sandpit_pre_align_active_hold_left_norm,
                self.sandpit_pre_align_active_hold_right_norm,
            )
            if (
                self.sandpit_pre_align_log_interval_sec > 0.0
                and (now - self.sandpit_pre_align_last_log_sec)
                >= self.sandpit_pre_align_log_interval_sec
            ):
                self.sandpit_pre_align_last_log_sec = now
                self._publish_feedback(
                    f'沙坑前预修正等待 ID{self.sandpit_tag_target_id} 新鲜 TF，'
                    f'保持 [{self.sandpit_pre_align_active_hold_mode},'
                    f'{self.sandpit_pre_align_active_hold_left_norm:.3f},'
                    f'{self.sandpit_pre_align_active_hold_right_norm:.3f}]。'
                )
            return

        distance_m = self._median([sample.distance_m for sample in recent_samples])
        lateral_m = self._median([sample.x_m for sample in recent_samples])
        z_m = self._median([sample.z_m for sample in recent_samples])
        self.sandpit_detection.distance_m = distance_m
        self.sandpit_detection.lateral_m = lateral_m
        self.sandpit_detection.nearest_stamp_sec = recent_samples[-1].received_stamp_sec
        distance = self._usable_distance(distance_m)

        tag_x_error_m = lateral_m - self.sandpit_pre_align_target_tag_x_m
        center_distance_m = z_m if math.isfinite(z_m) and z_m > 1e-3 else distance_m
        center_angle_rad = math.atan2(tag_x_error_m, max(1e-3, center_distance_m))
        if (
            self.sandpit_pre_align_polyline_enabled
            and self.sandpit_pre_align_polyline_phase
        ):
            self._run_sandpit_pre_align_polyline(
                now,
                len(recent_samples),
                distance,
                lateral_m,
                center_angle_rad,
            )
            return

        if distance is not None and distance <= self.sandpit_pre_align_handoff_distance_m:
            self._update_detection_trigger(self.sandpit_detection, True)
            self._publish_feedback(
                f'沙坑前预修正到达原触发线: dist={distance:.3f}m <= '
                f'{self.sandpit_pre_align_handoff_distance_m:.3f}m，切入原沙坑流程。'
            )
            self._enter_sandpit_bypass(now, reason='sandpit_pre_align_handoff')
            return

        if self.sandpit_pre_align_correction_kind == 'lateral':
            if (
                self.sandpit_pre_align_polyline_enabled
                and not self.sandpit_pre_align_polyline_completed_once
            ):
                polyline_ready = self._sandpit_pre_align_polyline_ready(
                    now,
                    self.sandpit_pre_align_locked_lateral_error_m,
                    self.sandpit_pre_align_locked_center_angle_rad,
                )
                if self.sandpit_pre_align_polyline_phase:
                    self._run_sandpit_pre_align_polyline(
                        now,
                        len(recent_samples),
                        distance,
                        lateral_m,
                        self.sandpit_pre_align_locked_center_angle_rad,
                    )
                    return
                if polyline_ready:
                    return
            self.sandpit_pre_align_correction_kind = 'approach'

        if self.sandpit_pre_align_correction_kind == 'yaw':
            if self._sandpit_pre_align_locked_yaw_done(now, distance):
                return
            return

        self._publish_step_override(
            self.sandpit_pre_align_active_mode,
            self.sandpit_pre_align_left_norm,
            self.sandpit_pre_align_right_norm,
        )

        if (
            self.sandpit_pre_align_log_interval_sec > 0.0
            and (now - self.sandpit_pre_align_last_log_sec)
            >= self.sandpit_pre_align_log_interval_sec
        ):
            self.sandpit_pre_align_last_log_sec = now
            elapsed_sec = now - self.sandpit_pre_align_start_sec
            self._publish_feedback(
                f'沙坑前预修正中: samples={len(recent_samples)}, '
                f'dist={self._distance_text(distance_m)} -> '
                f'{self.sandpit_pre_align_handoff_distance_m:.3f}m, '
                f'tag_x={lateral_m:+.3f}m target={self.sandpit_pre_align_target_tag_x_m:+.3f}m, '
                f'x_err={tag_x_error_m:+.3f}m, center={math.degrees(center_angle_rad):+.1f}deg, '
                f'locked={self.sandpit_pre_align_correction_kind or "approach"}, '
                f'elapsed={elapsed_sec:.1f}s, '
                f'override=[{self.sandpit_pre_align_active_mode},'
                f'{self.sandpit_pre_align_left_norm:.3f},'
                f'{self.sandpit_pre_align_right_norm:.3f}]。'
            )

    def _run_sandpit_bypass(self, now: float) -> None:
        if self.sandpit_phase_index is None:
            self._start_sandpit_phase(0, now)
            return
        phases = self._sandpit_phase_specs()
        if self.sandpit_phase_index >= len(phases):
            self._finish_sandpit_bypass(now)
            return
        phase = phases[self.sandpit_phase_index]
        if self._sandpit_phase_requires_pose(phase) and not self._pose_is_fresh(now):
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_feedback(f'{self._active_bypass_label()}等待新鲜 Odometry。')
            return
        kind = str(phase['kind'])
        if kind in ('align', 'turn'):
            self._run_sandpit_turn_phase(now)
            return
        if kind == 'walk':
            self._run_sandpit_walk_phase(now)
            return
        if kind in ('wait', 'step_once'):
            self._run_sandpit_wait_phase(now)
            return
        if kind == 'wait_finish':
            self._run_sandpit_wait_finish_phase(now)
            return
        self._fail(f'未知 {self._active_bypass_label()} phase kind: {kind}')

    def _sandpit_turn_tolerance_rad_for_phase(self, phase: Dict[str, Any]) -> float:
        return math.radians(max(0.0, self._phase_float(phase, 'yaw_tolerance_deg', math.degrees(self._active_bypass_turn_tolerance_rad()))))

    def _sandpit_turn_progress_tolerance_rad_for_phase(self, phase: Dict[str, Any]) -> float:
        return math.radians(max(0.0, self._phase_float(phase, 'progress_tolerance_deg', math.degrees(self._active_bypass_turn_progress_tolerance_rad()))))

    def _sandpit_turn_override_for_phase(
        self,
        phase: Dict[str, Any],
        direction_sign: int,
        yaw_error_rad: Optional[float] = None,
    ) -> tuple[float, float, str]:
        use_fine = False
        if yaw_error_rad is not None:
            fine_threshold_rad = math.radians(
                max(0.0, self._phase_float(phase, 'fine_threshold_deg', 0.0))
            )
            use_fine = (
                fine_threshold_rad > self._sandpit_turn_tolerance_rad_for_phase(phase)
                and abs(yaw_error_rad) <= fine_threshold_rad
            )
        if direction_sign > 0:
            left_default = self._phase_float(
                phase,
                'ccw_left_norm',
                self._phase_float(phase, 'left_norm', self._active_bypass_turn_ccw_left_norm()),
            )
            right_default = self._phase_float(
                phase,
                'ccw_right_norm',
                self._phase_float(phase, 'right_norm', self._active_bypass_turn_ccw_right_norm()),
            )
            if use_fine:
                return (
                    self._phase_float(phase, 'fine_ccw_left_norm', left_default),
                    self._phase_float(phase, 'fine_ccw_right_norm', right_default),
                    'ccw_fine',
                )
            return (
                left_default,
                right_default,
                'ccw',
            )
        left_default = self._phase_float(
            phase,
            'cw_left_norm',
            self._phase_float(phase, 'left_norm', self._active_bypass_turn_cw_left_norm()),
        )
        right_default = self._phase_float(
            phase,
            'cw_right_norm',
            self._phase_float(phase, 'right_norm', self._active_bypass_turn_cw_right_norm()),
        )
        if use_fine:
            return (
                self._phase_float(phase, 'fine_cw_left_norm', left_default),
                self._phase_float(phase, 'fine_cw_right_norm', right_default),
                'cw_fine',
            )
        return (
            left_default,
            right_default,
            'cw',
        )

    def _run_sandpit_turn_phase(self, now: float) -> None:
        phase = self._sandpit_current_phase()
        is_align = str(phase.get('kind', '')).strip().lower() == 'align'
        is_fixed_target_turn = (
            str(phase.get('kind', '')).strip().lower() == 'turn'
            and str(phase.get('target_yaw_source', 'current')).strip().lower() != 'current'
        )
        target_yaw = self.sandpit_phase_target_yaw
        if target_yaw is None:
            self._start_sandpit_phase(self.sandpit_phase_index or 0, now)
            return
        if self.sandpit_phase_turn_last_yaw is None:
            self.sandpit_phase_turn_last_yaw = self.pose_yaw
        if self.sandpit_phase_turn_target_delta_rad is None:
            self.sandpit_phase_turn_target_delta_rad = abs(
                self._normalize_angle(target_yaw - self.pose_yaw)
            )
        yaw_delta = self._normalize_angle(self.pose_yaw - self.sandpit_phase_turn_last_yaw)
        directed_increment = self.sandpit_phase_turn_direction_sign * yaw_delta
        if directed_increment > 0.0:
            self.sandpit_phase_turn_progress_rad += directed_increment
        self.sandpit_phase_turn_last_yaw = self.pose_yaw
        yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
        yaw_error_deg = math.degrees(yaw_error)
        turn_progress_rad = self.sandpit_phase_turn_progress_rad
        target_delta_rad = self.sandpit_phase_turn_target_delta_rad or 0.0
        progress_done = (
            is_align
            or is_fixed_target_turn
            or turn_progress_rad + self._sandpit_turn_progress_tolerance_rad_for_phase(phase) >=
            target_delta_rad
        )
        yaw_done = abs(yaw_error) <= self._sandpit_turn_tolerance_rad_for_phase(phase)
        hold_mode = self._phase_int(phase, 'hold_mode', self._active_bypass_turn_hold_mode())
        hold_left = self._phase_float(phase, 'hold_left_norm', self._active_bypass_turn_hold_left_norm())
        hold_right = self._phase_float(phase, 'hold_right_norm', self._active_bypass_turn_hold_right_norm())
        exit_hold_sec = max(0.0, self._phase_float(phase, 'exit_hold_sec', self._active_bypass_turn_exit_hold_sec()))
        if progress_done and yaw_done:
            if self.sandpit_phase_stable_since is None:
                self.sandpit_phase_stable_since = now
            stable_sec = now - self.sandpit_phase_stable_since
            self._publish_step_override(hold_mode, hold_left, hold_right)
            if stable_sec >= exit_hold_sec:
                self._advance_sandpit_phase(
                    now,
                    f'turn_progress={math.degrees(turn_progress_rad):.1f}/'
                    f'{math.degrees(target_delta_rad):.1f}deg, '
                    f'yaw_err={yaw_error_deg:.1f}deg, stable={stable_sec:.2f}s',
                )
                return
        else:
            self.sandpit_phase_stable_since = None
            mode = self._phase_int(phase, 'mode', self._active_bypass_turn_mode())
            if progress_done and not is_align and not is_fixed_target_turn:
                left_norm = hold_left
                right_norm = hold_right
                direction = 'hold'
                mode = hold_mode
            else:
                direction_sign = self.sandpit_phase_turn_direction_sign
                if is_align or is_fixed_target_turn:
                    direction_sign = 1 if yaw_error > 0.0 else -1
                left_norm, right_norm, direction = self._sandpit_turn_override_for_phase(
                    phase,
                    direction_sign,
                    yaw_error,
                )
            self._publish_step_override(mode, left_norm, right_norm)
            if self._sandpit_should_log(now):
                self._publish_feedback(
                    f'{self._active_bypass_label()}转向中: phase='
                    f'{self.sandpit_phase_index + 1}/{self._sandpit_phase_total()}, '
                    f'turn_progress={math.degrees(turn_progress_rad):.1f}/'
                    f'{math.degrees(target_delta_rad):.1f}deg, '
                    f'progress_done={progress_done}, yaw_err={yaw_error_deg:.1f}deg, '
                    f'yaw_done={yaw_done}, direction={direction}, '
                    f'override=[{mode},{left_norm:.3f},{right_norm:.3f}]。'
                )
        if self.sandpit_phase_deadline_sec is not None and now >= self.sandpit_phase_deadline_sec:
            action = str(phase.get('timeout_action', 'advance')).strip().lower()
            if action == 'fail':
                self._fail(
                    f'{self._active_bypass_label()}转向超时: '
                    f'progress={math.degrees(turn_progress_rad):.1f}/'
                    f'{math.degrees(target_delta_rad):.1f}deg, yaw_err={yaw_error_deg:.1f}deg。'
                )
                return
            self._advance_sandpit_phase(
                now,
                f'turn_timeout progress={math.degrees(turn_progress_rad):.1f}/'
                f'{math.degrees(target_delta_rad):.1f}deg yaw_err={yaw_error_deg:.1f}deg',
            )

    def _run_sandpit_walk_phase(self, now: float) -> None:
        phase = self._sandpit_current_phase()
        distance_m = self.sandpit_phase_target_distance_m
        if distance_m is None:
            self._start_sandpit_phase(self.sandpit_phase_index or 0, now)
            return
        progress_m = self._sandpit_walk_progress_m()
        elapsed_sec = max(0.0, now - self.sandpit_phase_start_sec)
        min_elapsed_sec = max(0.0, self._phase_float(phase, 'min_elapsed_sec', 0.0))
        distance_reached = progress_m is not None and progress_m >= distance_m
        if distance_reached and elapsed_sec >= min_elapsed_sec:
            self._advance_sandpit_phase(
                now,
                f'walk_progress={progress_m:.3f}/{distance_m:.3f}m, '
                f'elapsed={elapsed_sec:.2f}s>=min={min_elapsed_sec:.2f}s',
            )
            return
        if self.sandpit_phase_deadline_sec is not None and now >= self.sandpit_phase_deadline_sec:
            progress_text = 'unknown' if progress_m is None else f'{progress_m:.3f}m'
            self._advance_sandpit_phase(
                now,
                f'walk_timeout progress={progress_text}/{distance_m:.3f}m',
            )
            return

        left_norm = self._phase_float(phase, 'left_norm', self._active_bypass_walk_left_norm())
        right_norm = self._phase_float(phase, 'right_norm', self._active_bypass_walk_right_norm())
        yaw_error_deg: Optional[float] = None
        yaw_correction_norm = 0.0
        yaw_correction_enabled = bool(
            phase.get('yaw_correction_enabled', self._active_bypass_walk_yaw_correction_enabled())
        )
        yaw_tolerance_rad = math.radians(
            max(0.0, self._phase_float(phase, 'yaw_tolerance_deg', math.degrees(self._active_bypass_walk_yaw_tolerance_rad())))
        )
        yaw_gain_per_rad = max(
            0.0,
            self._phase_float(phase, 'yaw_gain_per_rad', self._active_bypass_walk_yaw_gain_per_rad()),
        )
        yaw_max_delta_norm = max(
            0.0,
            self._phase_float(phase, 'yaw_max_delta_norm', self._active_bypass_walk_yaw_max_delta_norm()),
        )
        if yaw_correction_enabled and self.sandpit_phase_target_yaw is not None:
            yaw_error = self._normalize_angle(self.sandpit_phase_target_yaw - self.pose_yaw)
            yaw_error_deg = math.degrees(yaw_error)
            yaw_guard_enabled = bool(phase.get('yaw_guard_enabled', False))
            if yaw_guard_enabled:
                yaw_guard_enter_rad = math.radians(
                    max(0.0, self._phase_float(phase, 'yaw_guard_enter_deg', 5.0))
                )
                yaw_guard_exit_rad = math.radians(
                    max(0.0, self._phase_float(phase, 'yaw_guard_exit_deg', 3.0))
                )
                yaw_guard_exit_rad = min(yaw_guard_exit_rad, yaw_guard_enter_rad)
                if abs(yaw_error) > yaw_guard_enter_rad:
                    self.sandpit_phase_walk_yaw_guard_active = True
                elif (
                    self.sandpit_phase_walk_yaw_guard_active
                    and abs(yaw_error) <= yaw_guard_exit_rad
                ):
                    self.sandpit_phase_walk_yaw_guard_active = False
                if self.sandpit_phase_walk_yaw_guard_active:
                    mode = self._phase_int(
                        phase,
                        'yaw_guard_mode',
                        self._phase_int(phase, 'mode', self._active_bypass_turn_mode()),
                    )
                    left_norm, right_norm, direction = self._sandpit_turn_override_for_phase(
                        phase,
                        1 if yaw_error > 0.0 else -1,
                        yaw_error,
                    )
                    self._publish_step_override(mode, left_norm, right_norm)
                    if self._sandpit_should_log(now):
                        progress_text = 'unknown' if progress_m is None else f'{progress_m:.3f}'
                        self._publish_feedback(
                            f'{self._active_bypass_label()}直走 yaw 保护: phase='
                            f'{self.sandpit_phase_index + 1}/{self._sandpit_phase_total()}, '
                            f'progress={progress_text}/{distance_m:.3f}m, '
                            f'yaw_err={yaw_error_deg:.1f}deg, direction={direction}, '
                            f'override=[{mode},{left_norm:.3f},{right_norm:.3f}]；'
                            '对齐后继续直走。'
                        )
                    return
            yaw_excess = max(0.0, abs(yaw_error) - yaw_tolerance_rad)
            if yaw_excess > 0.0:
                yaw_correction_norm = min(yaw_max_delta_norm, yaw_excess * yaw_gain_per_rad)
                if yaw_error > 0.0:
                    left_norm = self._clamp_norm(left_norm - yaw_correction_norm)
                    right_norm = self._clamp_norm(right_norm + yaw_correction_norm)
                else:
                    left_norm = self._clamp_norm(left_norm + yaw_correction_norm)
                    right_norm = self._clamp_norm(right_norm - yaw_correction_norm)
            else:
                yaw_error_deg = 0.0
        mode = self._phase_int(phase, 'mode', self._active_bypass_walk_mode())
        self._publish_step_override(mode, left_norm, right_norm)

        if self._sandpit_should_log(now):
            progress_text = 'unknown' if progress_m is None else f'{progress_m:.3f}'
            yaw_text = 'off' if yaw_error_deg is None else f'{yaw_error_deg:.1f}deg'
            min_elapsed_text = ''
            if min_elapsed_sec > 0.0:
                min_elapsed_text = (
                    f', elapsed={elapsed_sec:.2f}/{min_elapsed_sec:.2f}s'
                    f', dist_gate={distance_reached}'
                )
            self._publish_feedback(
                f'{self._active_bypass_label()}直走中: phase='
                f'{self.sandpit_phase_index + 1}/{self._sandpit_phase_total()}, '
                f'progress={progress_text}/{distance_m:.3f}m, '
                f'yaw_err={yaw_text}, yaw_corr={yaw_correction_norm:.3f}, '
                f'override=[{mode},{left_norm:.3f},{right_norm:.3f}]'
                f'{min_elapsed_text}。'
            )

    def _run_sandpit_wait_phase(self, now: float) -> None:
        phase = self._sandpit_current_phase()
        mode = self._phase_int(phase, 'hold_mode', self._phase_int(phase, 'mode', 0))
        left_norm = self._phase_float(phase, 'hold_left_norm', self._phase_float(phase, 'left_norm', 0.0))
        right_norm = self._phase_float(phase, 'hold_right_norm', self._phase_float(phase, 'right_norm', 0.0))
        self._publish_step_override(mode, left_norm, right_norm)
        if self.sandpit_phase_deadline_sec is None or now >= self.sandpit_phase_deadline_sec:
            self._advance_sandpit_phase(now, 'wait_done')
            return
        if self._sandpit_should_log(now):
            remain_sec = self.sandpit_phase_deadline_sec - now
            self._publish_feedback(
                f'{self._active_bypass_label()}等待中: phase='
                f'{self.sandpit_phase_index + 1}/{self._sandpit_phase_total()}, '
                f'remain={remain_sec:.1f}s, override=[{mode},{left_norm:.3f},{right_norm:.3f}]。'
            )

    def _run_sandpit_wait_finish_phase(self, now: float) -> None:
        phase = self._sandpit_current_phase()
        mode = self._phase_int(phase, 'hold_mode', self._phase_int(phase, 'mode', 0))
        left_norm = self._phase_float(phase, 'hold_left_norm', self._phase_float(phase, 'left_norm', 0.0))
        right_norm = self._phase_float(phase, 'hold_right_norm', self._phase_float(phase, 'right_norm', 0.0))
        self._publish_step_override(mode, left_norm, right_norm)
        require_jumping = bool(phase.get('require_jumping_before_finish', True))
        if not self.serial_connected:
            if not self.sandpit_wait_finish_serial_disconnect_active:
                self.sandpit_wait_finish_serial_disconnect_active = True
                self.sandpit_wait_finish_serial_disconnect_stamp_sec = now
                self.sandpit_wait_finish_serial_disconnect_had_jumping = (
                    self.sandpit_jumping_signal_received
                )
                self.sandpit_wait_finish_next_resend_sec = None
                self._publish_feedback(
                    f'{self._active_bypass_label()}等待电控回包时串口断开，'
                    f'保护保持 [0,0,0] 等待重连；断开前 '
                    f'jumping={self.sandpit_wait_finish_serial_disconnect_had_jumping}, '
                    f'finish={self.sandpit_finish_signal_received}。'
                )
            if self._sandpit_should_log(now):
                disconnected_for = now - self.sandpit_wait_finish_serial_disconnect_stamp_sec
                self._publish_feedback(
                    f'{self._active_bypass_label()}串口断开保护中: '
                    f'disconnected_for={disconnected_for:.1f}s, '
                    f'override=[{mode},{left_norm:.3f},{right_norm:.3f}], '
                    f'jumping={self.sandpit_jumping_signal_received}, '
                    f'finish={self.sandpit_finish_signal_received}。'
                )
            return
        if self.sandpit_wait_finish_serial_disconnect_active:
            had_jumping = (
                self.sandpit_wait_finish_serial_disconnect_had_jumping
                or self.sandpit_jumping_signal_received
            )
            disconnected_for = now - self.sandpit_wait_finish_serial_disconnect_stamp_sec
            self.sandpit_wait_finish_serial_disconnect_active = False
            if self.sandpit_finish_signal_received:
                self._publish_feedback(
                    f'{self._active_bypass_label()}串口重连，断开期间已收到 finish，'
                    f'disconnected_for={disconnected_for:.1f}s，继续正常完成判定。'
                )
            elif had_jumping:
                self.sandpit_jumping_signal_received = True
                self.sandpit_jumping_signal_stamp_sec = now
                self.sandpit_finish_signal_received = True
                self.sandpit_finish_signal_stamp_sec = now
                self.sandpit_phase_wait_for_finish = False
                self.sandpit_wait_finish_next_resend_sec = None
                self._publish_feedback(
                    f'{self._active_bypass_label()}串口重连，断开前已收到 [jumping] '
                    f'但未收到 [finish]；按跳跃已完成处理并继续后续流程，'
                    f'disconnected_for={disconnected_for:.1f}s。'
                )
                self._advance_sandpit_phase(now, 'serial_reconnect_after_jumping_assume_finish')
                return
            elif self.sandpit_last_step_once_command is not None:
                resend_mode, resend_left, resend_right = self.sandpit_last_step_once_command
                self._publish_step_once(resend_mode, resend_left, resend_right)
                self.sandpit_wait_finish_resend_count += 1
                if (
                    self.sandpit_wait_finish_resend_after_sec > 0.0
                    and self.sandpit_wait_finish_resend_count
                    < self.sandpit_wait_finish_resend_max_count
                ):
                    self.sandpit_wait_finish_next_resend_sec = (
                        now + self.sandpit_wait_finish_resend_after_sec
                    )
                else:
                    self.sandpit_wait_finish_next_resend_sec = None
                self._publish_feedback(
                    f'{self._active_bypass_label()}串口重连，断开前未收到 [jumping]；'
                    f'重发一次 step_once=[{resend_mode},{resend_left:.3f},{resend_right:.3f}]，'
                    f'disconnected_for={disconnected_for:.1f}s, '
                    f'resend_count={self.sandpit_wait_finish_resend_count}/'
                    f'{self.sandpit_wait_finish_resend_max_count}。'
                )
            else:
                self._publish_feedback(
                    f'{self._active_bypass_label()}串口重连，但没有可重发的 step_once 记录；'
                    f'继续保持 [0,0,0] 等待回包。'
                )
        if self.sandpit_finish_signal_received:
            self.sandpit_phase_wait_for_finish = False
            self.sandpit_wait_finish_next_resend_sec = None
            self._advance_sandpit_phase(now, 'finish_signal_received')
            return
        if require_jumping and self.sandpit_jumping_signal_received:
            self.sandpit_wait_finish_next_resend_sec = None
        if (
            self.sandpit_wait_finish_next_resend_sec is not None
            and now >= self.sandpit_wait_finish_next_resend_sec
            and self.sandpit_last_step_once_command is not None
            and self.sandpit_wait_finish_resend_count < self.sandpit_wait_finish_resend_max_count
            and (not require_jumping or not self.sandpit_jumping_signal_received)
        ):
            resend_mode, resend_left, resend_right = self.sandpit_last_step_once_command
            self._publish_step_once(resend_mode, resend_left, resend_right)
            self.sandpit_wait_finish_resend_count += 1
            if self.sandpit_wait_finish_resend_count < self.sandpit_wait_finish_resend_max_count:
                self.sandpit_wait_finish_next_resend_sec = (
                    now + self.sandpit_wait_finish_resend_after_sec
                )
            else:
                self.sandpit_wait_finish_next_resend_sec = None
            self._publish_feedback(
                f'{self._active_bypass_label()}等待 jumping 超过 '
                f'{self.sandpit_wait_finish_resend_after_sec:.1f}s，'
                f'重发 step_once=[{resend_mode},{resend_left:.3f},{resend_right:.3f}] '
                f'({self.sandpit_wait_finish_resend_count}/'
                f'{self.sandpit_wait_finish_resend_max_count})；继续 [0,0,0] 等待回包。'
            )
        if self.sandpit_phase_deadline_sec is not None and now >= self.sandpit_phase_deadline_sec:
            action = str(phase.get('timeout_action', 'fail')).strip().lower()
            if action == 'advance':
                self._advance_sandpit_phase(now, 'finish_wait_timeout_advance')
                return
            self._fail(f'{self._active_bypass_label()}等待电控 [finish] 回包超时。')
            return
        if self._sandpit_should_log(now):
            timeout_text = 'none'
            if self.sandpit_phase_deadline_sec is not None:
                timeout_text = f'{self.sandpit_phase_deadline_sec - now:.1f}s'
            self._publish_feedback(
                f'{self._active_bypass_label()}等待回包: phase='
                f'{self.sandpit_phase_index + 1}/{self._sandpit_phase_total()}, '
                f'jumping={self.sandpit_jumping_signal_received}, '
                f'finish={self.sandpit_finish_signal_received}, '
                f'timeout_left={timeout_text}, override=[{mode},{left_norm:.3f},{right_norm:.3f}]。'
            )

    def _sandpit_walk_progress_m(self) -> Optional[float]:
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.sandpit_phase_start_x is None
            or self.sandpit_phase_start_y is None
            or self.sandpit_phase_start_yaw is None
        ):
            return None
        forward_heading = self.sandpit_phase_start_yaw + math.pi / 2.0
        dx = self.pose_x - self.sandpit_phase_start_x
        dy = self.pose_y - self.sandpit_phase_start_y
        progress_m = dx * math.cos(forward_heading) + dy * math.sin(forward_heading)
        if not math.isfinite(progress_m):
            return None
        return progress_m

    def _sandpit_should_log(self, now: float) -> bool:
        interval_sec = self._active_bypass_progress_log_interval_sec()
        if interval_sec <= 0.0:
            return False
        if (now - self.sandpit_phase_last_log_sec) < interval_sec:
            return False
        self.sandpit_phase_last_log_sec = now
        return True

    def _sandpit_turn_override(self, direction_sign: int) -> tuple[float, float, str]:
        if direction_sign > 0:
            return (
                self._active_bypass_turn_ccw_left_norm(),
                self._active_bypass_turn_ccw_right_norm(),
                'ccw',
            )
        return (
            self._active_bypass_turn_cw_left_norm(),
            self._active_bypass_turn_cw_right_norm(),
            'cw',
        )

    def _active_bypass_label(self) -> str:
        if self.active_bypass_kind == 'pole':
            return '绕杆绕行'
        if self.sandpit_behavior == 'traverse':
            return '砂坑穿越'
        return '砂坑绕行'

    def _active_bypass_turn_sequence_deg(self) -> List[float]:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_sequence_deg
        return self.sandpit_turn_sequence_deg

    def _active_bypass_straight_distances_m(self) -> List[float]:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_straight_distances_m
        return self.sandpit_straight_distances_m

    def _active_bypass_progress_log_interval_sec(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_progress_log_interval_sec
        return self.sandpit_progress_log_interval_sec

    def _active_bypass_turn_mode(self) -> int:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_mode
        return self.sandpit_turn_mode

    def _active_bypass_turn_cw_left_norm(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_cw_left_norm
        return self.sandpit_turn_cw_left_norm

    def _active_bypass_turn_cw_right_norm(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_cw_right_norm
        return self.sandpit_turn_cw_right_norm

    def _active_bypass_turn_ccw_left_norm(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_ccw_left_norm
        return self.sandpit_turn_ccw_left_norm

    def _active_bypass_turn_ccw_right_norm(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_ccw_right_norm
        return self.sandpit_turn_ccw_right_norm

    def _active_bypass_turn_hold_mode(self) -> int:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_hold_mode
        return self.sandpit_turn_hold_mode

    def _active_bypass_turn_hold_left_norm(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_hold_left_norm
        return self.sandpit_turn_hold_left_norm

    def _active_bypass_turn_hold_right_norm(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_hold_right_norm
        return self.sandpit_turn_hold_right_norm

    def _active_bypass_turn_tolerance_rad(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_tolerance_rad
        return self.sandpit_turn_tolerance_rad

    def _active_bypass_turn_progress_tolerance_rad(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_progress_tolerance_rad
        return self.sandpit_turn_progress_tolerance_rad

    def _active_bypass_turn_exit_hold_sec(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_exit_hold_sec
        return self.sandpit_turn_exit_hold_sec

    def _active_bypass_turn_timeout_sec(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_turn_timeout_sec
        return self.sandpit_turn_timeout_sec

    def _active_bypass_walk_mode(self) -> int:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_walk_mode
        return self.sandpit_walk_mode

    def _active_bypass_walk_left_norm(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_walk_left_norm
        return self.sandpit_walk_left_norm

    def _active_bypass_walk_right_norm(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_walk_right_norm
        return self.sandpit_walk_right_norm

    def _active_bypass_walk_timeout_sec_per_m(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_walk_timeout_sec_per_m
        return self.sandpit_walk_timeout_sec_per_m

    def _active_bypass_walk_timeout_extra_sec(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_walk_timeout_extra_sec
        return self.sandpit_walk_timeout_extra_sec

    def _active_bypass_walk_yaw_correction_enabled(self) -> bool:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_walk_yaw_correction_enabled
        return self.sandpit_walk_yaw_correction_enabled

    def _active_bypass_walk_yaw_tolerance_rad(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_walk_yaw_tolerance_rad
        return self.sandpit_walk_yaw_tolerance_rad

    def _active_bypass_walk_yaw_gain_per_rad(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_walk_yaw_gain_per_rad
        return self.sandpit_walk_yaw_gain_per_rad

    def _active_bypass_walk_yaw_max_delta_norm(self) -> float:
        if self.active_bypass_kind == 'pole':
            return self.pole_bypass_walk_yaw_max_delta_norm
        return self.sandpit_walk_yaw_max_delta_norm

    def _finish_sandpit_bypass(self, now: float) -> None:
        is_pole_bypass = self.active_bypass_kind == 'pole'
        if is_pole_bypass:
            self.pole_completed = True
            self.pole_bypass_completed = True
        else:
            self.sandpit_completed = True
        self.sandpit_phase_index = None
        self.sandpit_phase_deadline_sec = None
        self.sandpit_phase_stable_since = None
        self.sandpit_phase_turn_direction_sign = 0
        self.sandpit_phase_turn_last_yaw = None
        self.sandpit_phase_turn_progress_rad = 0.0
        self.sandpit_phase_turn_target_delta_rad = None
        self.sandpit_phase_wait_for_finish = False
        self.sandpit_finish_signal_received = False
        self.sandpit_finish_signal_stamp_sec = 0.0
        if is_pole_bypass:
            self.pole_detection.reset()
            self.pole_apriltag_detection.reset()
            self.pole_apriltag_last_seen_stamp_sec = 0.0
        else:
            self.sandpit_detection.reset()
        self._clear_step_override()
        self._publish_feedback(f'{self._active_bypass_label()}完成，返回搜索模式继续寻找后续障碍。')
        finish_reason = 'pole_bypass_done'
        if not is_pole_bypass:
            finish_reason = (
                'sandpit_traverse_done'
                if self.sandpit_behavior == 'traverse'
                else 'sandpit_bypass_done'
            )
        self._enter_search(now, reason=finish_reason)
        self.search_arm_deadline = now + (
            self.pole_bypass_search_rearm_delay_sec
            if is_pole_bypass
            else self.sandpit_search_rearm_delay_sec
        )
        self.active_bypass_kind = 'sandpit'

    def _enter_post_stairs_slope_entry(self, now: float, reason: str) -> None:
        if not self._pose_is_fresh(now):
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_feedback('下楼梯完成后等待新鲜 Odometry，再执行斜坡前 yaw 对齐。')
            return

        feedback = (
            '下楼梯完成后使用固定入口进入障碍识别分流：先 yaw 对齐，再按里程计固定前进 '
            f'{self.post_stairs_slope_entry_distance_m:.2f}m，到固定入口后再按识别结果切分支。'
        )
        if self.post_stairs_slope_entry_initial_yaw_align_enabled:
            self._enter_slope_or_align(
                now,
                reason=reason,
                feedback=feedback,
                align_next_action='post_stairs_slope_approach',
                allow_upstairs_preempt=False,
                force_align=True,
            )
            return
        self._enter_post_stairs_slope_approach(now)

    def _enter_limit_bar_align(self, now: float, reason: str, yaw_ref_text: str) -> None:
        self._clear_step_override()
        self.state = self.STATE_LIMIT_BAR_ALIGN
        self.current_active_branch = 'limit_bar_align'
        self.slope_align_enter_time = now
        self.slope_align_reason = reason
        self.slope_align_feedback = (
            f'限高杆前 yaw 对齐完成，保持趴下等待 {self.duck_prepare_delay_sec:.1f}s 后通过。'
        )
        self.slope_align_next_action = 'limit_bar_prepare'
        self.slope_align_custom_target_yaw = self.duck_reference_yaw
        self.last_slope_align_log_time = 0.0
        self.slope_align_stable_since = None
        self.limit_bar_lateral_pre_align_active = (
            self.limit_bar_tag_align_active
            and self.limit_bar_lateral_shift_enabled
            and self.limit_bar_lateral_pre_align_enabled
        )
        self.limit_bar_lateral_pre_align_done = False
        self.limit_bar_lateral_pre_align_start_sec = None
        self.limit_bar_lateral_pre_align_stable_since = None
        self.limit_bar_lateral_pre_align_target_yaw = None
        self.limit_bar_lateral_pre_align_source_odom_yaw = None
        if self.limit_bar_lateral_pre_align_active:
            self.slope_align_custom_target_yaw = None
        self.phase_deadline_sec = None
        self._publish_state(self.state)
        if self.limit_bar_lateral_pre_align_active:
            if self.limit_bar_lateral_pre_align_mode == 'fixed_absolute_yaw':
                align_text = '先按锁定的全局/赛道 yaw 预对齐，Tag 仅用于距离和横向中心线。'
            else:
                align_text = (
                    f'先按 {self.limit_bar_lateral_pre_align_mode} 视觉预对齐 tag。'
                )
        else:
            align_text = '先停下做 yaw 对齐。'
        self._publish_feedback(
            f'进入限高杆分支: reason={reason}, dist={self._distance_text(self.limit_detection.distance_m)}，'
            f'锁存趴走 yaw 基准={yaw_ref_text}，{align_text}'
        )

    def _enter_hurdle_align(self, now: float, reason: str, yaw_ref_text: str) -> None:
        self._clear_step_override()
        self.state = self.STATE_HURDLE_ALIGN
        self.current_active_branch = 'hurdle_tag_align'
        self.slope_align_enter_time = now
        self.slope_align_reason = reason
        self.slope_align_feedback = '高墙跳跃前 AprilTag 对中完成，进入起跳等待。'
        self.slope_align_next_action = 'hurdle_wait_jump'
        self.slope_align_custom_target_yaw = self.duck_reference_yaw
        self.limit_bar_lateral_shift_next_action = 'hurdle_wait_jump'
        self.last_slope_align_log_time = 0.0
        self.slope_align_stable_since = None
        self.limit_bar_lateral_pre_align_active = (
            self.limit_bar_tag_align_active
            and self.limit_bar_lateral_shift_enabled
            and self.hurdle_lateral_pre_align_enabled
        )
        self.limit_bar_lateral_pre_align_done = False
        self.limit_bar_lateral_pre_align_start_sec = None
        self.limit_bar_lateral_pre_align_stable_since = None
        self.limit_bar_lateral_pre_align_target_yaw = None
        self.limit_bar_lateral_pre_align_source_odom_yaw = None
        if self.limit_bar_lateral_pre_align_active:
            self.slope_align_custom_target_yaw = None
        self.phase_deadline_sec = None
        self._publish_state(self.state)
        align_text = (
            f'先按 {self.hurdle_lateral_pre_align_mode} 视觉预对齐 tag，'
            '再复用限高杆横向补偿流程。'
            if self.limit_bar_lateral_pre_align_active
            else '先复用限高杆横向补偿流程。'
        )
        self._publish_feedback(
            f'进入高墙 AprilTag 对中分支: reason={reason}, '
            f'dist={self._distance_text(self.hurdle_detection.distance_m)}，'
            f'锁存起跳 yaw 基准={yaw_ref_text}，{align_text}'
        )

    def _enter_upstairs_align(self, now: float, reason: str, yaw_ref_text: str) -> None:
        self._clear_step_override()
        self.state = self.STATE_UPSTAIRS_ALIGN
        self.current_active_branch = 'upstairs_tag_align'
        self.slope_align_enter_time = now
        self.slope_align_reason = reason
        self.slope_align_feedback = '上台阶 AprilTag 对中完成，进入上台阶动作。'
        self.slope_align_next_action = 'upstairs_step_once'
        self.slope_align_custom_target_yaw = self.duck_reference_yaw
        self.limit_bar_lateral_shift_next_action = 'upstairs_step_once'
        self.last_slope_align_log_time = 0.0
        self.slope_align_stable_since = None
        self.limit_bar_lateral_pre_align_active = (
            self.limit_bar_tag_align_active
            and self.limit_bar_lateral_shift_enabled
            and self.upstairs_lateral_pre_align_enabled
        )
        self.limit_bar_lateral_pre_align_done = False
        self.limit_bar_lateral_pre_align_start_sec = None
        self.limit_bar_lateral_pre_align_stable_since = None
        self.limit_bar_lateral_pre_align_target_yaw = None
        self.limit_bar_lateral_pre_align_source_odom_yaw = None
        self.limit_bar_lateral_pre_align_source_center_offset_m = self.upstairs_tag_to_center_x_m
        if self.limit_bar_lateral_pre_align_active:
            self.slope_align_custom_target_yaw = None
        self.phase_deadline_sec = None
        self._publish_state(self.state)
        if (
            self.limit_bar_lateral_pre_align_active
            and self.upstairs_lateral_pre_align_mode == 'fixed_absolute_yaw'
        ):
            upstairs_target_yaw = self._upstairs_fixed_reference_target_yaw()
            pre_align_text = (
                '先按 odom 绝对 '
                f'{math.degrees(upstairs_target_yaw):.1f}deg '
                '做航向预对齐，'
            )
        elif self.limit_bar_lateral_pre_align_active:
            pre_align_text = (
                f'先按 {self.upstairs_lateral_pre_align_mode} 视觉预对齐 tag，'
            )
        else:
            pre_align_text = ''
        self._publish_feedback(
            f'进入上台阶 AprilTag 对中分支: reason={reason}, '
            f'dist={self._distance_text(self.upstairs_detection.distance_m)}，'
            f'锁存上台阶 yaw 基准={yaw_ref_text}，'
            f'{pre_align_text}'
            f'再复用限高杆/高墙横向补偿流程。'
        )

    def _finish_upstairs_done(
        self,
        now: float,
        reason: str,
        final_along_m: Optional[float] = None,
        raw_final_along_m: Optional[float] = None,
        final_remaining_m: Optional[float] = None,
    ) -> None:
        self.upstairs_completed = True
        self.phase_deadline_sec = None
        self.current_active_branch = ''
        self.state = self.STATE_DONE
        self.upstairs_jumping_signal_received = False
        self.upstairs_jumping_signal_stamp_sec = 0.0
        self.upstairs_step_once_waiting_ack = self.upstairs_step_once_resend_max_count > 0
        self.upstairs_step_once_resend_count = 0
        if (
            self.upstairs_done_retry_guard_enabled
            and self.upstairs_done_retry_guard_duration_sec > 0.0
        ):
            self.upstairs_done_retry_guard_deadline_sec = (
                now + self.upstairs_done_retry_guard_duration_sec
            )
        else:
            self.upstairs_done_retry_guard_deadline_sec = None
        if (
            self.upstairs_step_once_waiting_ack
            and self.upstairs_step_once_resend_after_sec > 0.0
        ):
            self.upstairs_step_once_next_resend_sec = (
                now + self.upstairs_step_once_resend_after_sec
            )
        else:
            self.upstairs_step_once_next_resend_sec = None
        self._publish_state(self.state)
        self._publish_active_flags()
        self._clear_step_override()
        self._publish_step_once(
            self.upstairs_step_once_mode,
            self.upstairs_step_once_left_norm,
            self.upstairs_step_once_right_norm,
        )
        if self._upstairs_done_retry_guard_active(now):
            self._publish_upstairs_done_retry_guard_override()
        else:
            self._publish_done_hold_override()
        final_progress_text = ''
        if final_along_m is not None:
            raw_text = (
                'unknown'
                if raw_final_along_m is None
                else f'{raw_final_along_m:.3f}m'
            )
            remaining_text = (
                'unknown'
                if final_remaining_m is None
                else f'{max(0.0, final_remaining_m):.3f}m'
            )
            final_progress_text = (
                f'latest_tag_dist={self._distance_text(self.upstairs_detection.distance_m)}, '
                f'target_dist={self.upstairs_lateral_shift_target_distance_m:.3f}m, '
                f'planned_final_forward={self.limit_bar_lateral_shift_final_forward_m:.3f}m, '
                f'actual_final_along={final_along_m:.3f}m(raw={raw_text}), '
                f'remaining={remaining_text}，'
            )
        else:
            final_progress_text = (
                f'latest_dist={self._distance_text(self.upstairs_detection.distance_m)}, '
                f'trigger_threshold={self.upstairs_trigger_distance_m:.3f}m，'
            )
        self._publish_feedback(
            f'进入上台阶分支: reason={reason}, '
            f'{final_progress_text}'
            f'已向电控发送一次 step_once=[{self.upstairs_step_once_mode},'
            f'{self.upstairs_step_once_left_norm:.3f},'
            f'{self.upstairs_step_once_right_norm:.3f}]；若 '
            f'{self.upstairs_step_once_resend_after_sec:.1f}s 内未收到 [jumping]，'
            f'最多重发 {self.upstairs_step_once_resend_max_count} 次。总状态机结束；'
            f'随后持续发布保护小前进=['
            f'{self.upstairs_done_retry_guard_mode},'
            f'{self.upstairs_done_retry_guard_left_norm:.3f},'
            f'{self.upstairs_done_retry_guard_right_norm:.3f}] '
            f'{self.upstairs_done_retry_guard_duration_sec:.1f}s，'
            '保护期内收到 [start] 可重新搜索并再次进入上台阶分支。'
        )

    def _enter_hurdle_wait_jump(self, now: float, reason: str) -> None:
        self.hurdle_entry_yaw = self.pose_yaw
        self._publish_step_override(
            self.hurdle_stand_mode,
            self.hurdle_stand_left_norm,
            self.hurdle_stand_right_norm,
        )
        self.phase_deadline_sec = now + self.hurdle_stand_to_jump_delay_sec
        self.state = self.STATE_HURDLE_WAIT_JUMP
        self.current_active_branch = ''
        self.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'
        self._publish_state(self.state)
        yaw_text = 'unknown'
        if self.hurdle_entry_yaw is not None:
            yaw_text = f'{math.degrees(self.hurdle_entry_yaw):.1f}deg'
        self._publish_feedback(
            f'进入高墙跳跃分支: reason={reason}, '
            f'dist={self._distance_text(self.hurdle_detection.distance_m)}，'
            f'entry_yaw={yaw_text}，持续停止等待 '
            f'{self.hurdle_stand_to_jump_delay_sec:.1f}s 后起跳。'
        )

    def _enter_limit_bar_stop_buffer(self, now: float, reason: str, yaw_ref_text: str) -> None:
        self.pending_limit_bar_reason = reason
        self.pending_limit_bar_yaw_ref_text = yaw_ref_text
        self._publish_step_override(0, 0.0, 0.0)
        self.phase_deadline_sec = now + self.duck_prepare_stop_buffer_sec
        self.state = self.STATE_LIMIT_BAR_STOP_BUFFER
        self.current_active_branch = ''
        self._publish_state(self.state)
        self._publish_feedback(
            f'进入限高杆趴下前缓冲: reason={reason}, dist={self._distance_text(self.limit_detection.distance_m)}，'
            f'先持续 [0,0.000,0.000] 停稳 {self.duck_prepare_stop_buffer_sec:.1f}s，'
            f'再切到 [{self.duck_prepare_mode},{self.duck_prepare_left_norm:.3f},'
            f'{self.duck_prepare_right_norm:.3f}] 趴下等待。'
        )

    def _enter_limit_bar_prepare(
        self,
        now: float,
        reason: str,
        yaw_ref_text: str,
        *,
        skip_stop_buffer: bool = False,
    ) -> None:
        if self.duck_prepare_stop_buffer_sec > 0.0 and not skip_stop_buffer:
            self._enter_limit_bar_stop_buffer(now, reason, yaw_ref_text)
            return
        self._publish_step_override(
            self.duck_prepare_mode,
            self.duck_prepare_left_norm,
            self.duck_prepare_right_norm,
        )
        self.phase_deadline_sec = now + self.duck_prepare_delay_sec
        self.limit_bar_duck_prepare_yaw_settle_start_sec = None
        self.limit_bar_duck_prepare_yaw_stable_since_sec = None
        self.last_limit_bar_duck_prepare_yaw_log_sec = 0.0
        self.state = self.STATE_LIMIT_BAR_PREPARE
        self.current_active_branch = ''
        self.pending_limit_bar_reason = ''
        self.pending_limit_bar_yaw_ref_text = 'unknown'
        self._publish_state(self.state)
        self._publish_feedback(
            f'进入限高杆分支: reason={reason}, dist={self._distance_text(self.limit_detection.distance_m)}，'
            f'锁存趴走 yaw 基准={yaw_ref_text}，'
            f'持续 [{self.duck_prepare_mode},{self.duck_prepare_left_norm:.3f},'
            f'{self.duck_prepare_right_norm:.3f}] 趴下等待 '
            f'{self.duck_prepare_delay_sec:.1f}s，再持续 '
            f'[{self.duck_walk_mode},{self.duck_walk_left_norm:.3f},'
            f'{self.duck_walk_right_norm:.3f}] 通过。'
        )

    def _start_limit_bar_lateral_shift(
        self,
        now: float,
        reason: str,
        yaw_ref_text: str,
    ) -> bool:
        if not self.limit_bar_lateral_shift_enabled:
            return False
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.pose_yaw is None
            or self.duck_reference_yaw is None
        ):
            self._publish_feedback(
                f'{self._lateral_shift_label()}横向补偿等待 Odometry/yaw 基准，暂不进入补偿阶段。'
            )
            return False

        use_pre_align_locked_tag = (
            (
                self.upstairs_lateral_pre_align_enabled
                if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once'
                else self.limit_bar_lateral_pre_align_enabled
            )
            and self.limit_bar_lateral_pre_align_done
        )
        if not use_pre_align_locked_tag:
            self._refresh_limit_bar_lateral_shift_target(now)
        center_x_m = self.limit_bar_lateral_shift_center_x_m
        center_z_m = self.limit_bar_lateral_shift_center_z_m
        if not math.isfinite(center_x_m) or not math.isfinite(center_z_m):
            self._publish_feedback(
                f'{self._lateral_shift_label()}横向补偿未启用: tag 中心线数据无效。'
            )
            return False

        yaw_abs_rad = self._lateral_shift_yaw_abs_rad()
        sin_yaw = max(1e-6, math.sin(yaw_abs_rad))
        abs_center_x_m = abs(center_x_m)
        lateral_deadband_m = self._lateral_shift_deadband_m()
        arrival_tolerance_m = self._lateral_shift_arrival_tolerance_m()
        skip_lateral_shift = (
            self.limit_bar_lateral_shift_skip_below_m > 0.0
            and abs_center_x_m <= self.limit_bar_lateral_shift_skip_below_m
        )
        if skip_lateral_shift or abs_center_x_m <= lateral_deadband_m:
            side_delta_rad = 0.0
            raw_shift_m = 0.0
            requested_shift_m = 0.0
            shift_m = 0.0
        else:
            side_delta_rad = (
                self._lateral_shift_yaw_sign()
                * math.copysign(yaw_abs_rad, center_x_m)
            )
            raw_shift_m = abs_center_x_m / sin_yaw
            requested_shift_m = raw_shift_m * self._lateral_shift_distance_scale()
            max_shift_distance_m = self._lateral_shift_max_distance_m()
            if max_shift_distance_m > 0.0:
                shift_m = min(requested_shift_m, max_shift_distance_m)
            else:
                shift_m = requested_shift_m

        base_yaw = self._normalize_angle(self.duck_reference_yaw)
        side_yaw = self._normalize_angle(base_yaw + side_delta_rad)
        forward_component_m = shift_m * math.cos(abs(side_delta_rad))
        target_distance_m = (
            self.hurdle_lateral_shift_target_distance_m
            if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump'
            else self.upstairs_lateral_shift_target_distance_m
            if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once'
            else self.limit_bar_lateral_shift_target_distance_m
        )
        final_forward_m = max(
            0.0,
            center_z_m - forward_component_m - target_distance_m,
        )

        self._prepare_lateral_s_curve_shadow(
            now=now,
            shift_m=shift_m,
            side_delta_rad=side_delta_rad,
            forward_component_m=forward_component_m,
            final_forward_m=final_forward_m,
        )

        control_plan = self.lateral_s_curve_shadow_plan
        use_s_curve_control = bool(
            self._lateral_s_curve_control_requested()
            and control_plan is not None
            and control_plan.feasible
        )

        initial_phase = (
            's_curve'
            if use_s_curve_control
            else
            'align_out'
            if shift_m > arrival_tolerance_m
            else 'final_forward'
        )
        self.limit_bar_lateral_shift_reason = reason
        self.limit_bar_lateral_shift_yaw_ref_text = yaw_ref_text
        self.limit_bar_lateral_shift_base_yaw = base_yaw
        self.limit_bar_lateral_shift_side_yaw = side_yaw
        self.limit_bar_lateral_shift_distance_m = shift_m
        self.limit_bar_lateral_shift_raw_distance_m = raw_shift_m
        self.limit_bar_lateral_shift_requested_distance_m = requested_shift_m
        self.limit_bar_lateral_shift_forward_component_m = forward_component_m
        self.limit_bar_lateral_shift_final_forward_m = final_forward_m
        self.limit_bar_lateral_shift_segment_start_x = (
            self.pose_x if initial_phase in ('final_forward', 's_curve') else None
        )
        self.limit_bar_lateral_shift_segment_start_y = (
            self.pose_y if initial_phase in ('final_forward', 's_curve') else None
        )
        if use_s_curve_control:
            self.lateral_s_curve_control_active = True
            self.lateral_s_curve_control_plan = control_plan
            self.lateral_s_curve_control_start_x = self.pose_x
            self.lateral_s_curve_control_start_y = self.pose_y
            self.lateral_s_curve_control_start_sec = now
            self.lateral_s_curve_control_completion_since_sec = None
            self.lateral_s_curve_control_completion_reason = ''
        self._set_limit_bar_lateral_shift_phase(initial_phase, now)
        self.limit_bar_lateral_shift_duck_align_since_sec = None
        self.upstairs_final_align_latched_along_m = None
        self.upstairs_final_align_latched_raw_along_m = None
        self.upstairs_final_align_latched_remaining_m = None
        self.upstairs_final_align_latched_reason = ''
        self.upstairs_final_align_latched_live_text = ''
        self.limit_bar_pre_duck_global_yaw_align_start_sec = None
        self.limit_bar_pre_duck_global_yaw_stable_since_sec = None
        self.limit_bar_pre_duck_global_yaw_hold_active = False
        self.last_limit_bar_lateral_shift_log_sec = 0.0
        if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
            self.upstairs_lateral_shift_live_replan_count = 0
        self.state = self.STATE_LIMIT_BAR_LATERAL_SHIFT
        self.current_active_branch = (
            'hurdle_lateral_shift'
            if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump'
            else 'upstairs_lateral_shift'
            if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once'
            else 'limit_bar_lateral_shift'
        )
        self.phase_deadline_sec = None
        self._publish_state(self.state)
        clipped = shift_m + 1e-6 < requested_shift_m
        self._publish_feedback(
            f'{self._lateral_shift_label()}横向补偿计划锁定: center_x={center_x_m:+.3f}m, '
            f'center_z={center_z_m:.3f}m, base_yaw={math.degrees(base_yaw):.1f}deg, '
            f'side_delta={math.degrees(side_delta_rad):+.1f}deg, '
            f'side_yaw={math.degrees(side_yaw):.1f}deg, '
            f'shift={shift_m:.3f}m(raw={raw_shift_m:.3f}m, '
            f'requested={requested_shift_m:.3f}m, clipped={clipped}), '
            f'trigger_deadband={lateral_deadband_m:.3f}m, '
            f'arrival_tolerance={arrival_tolerance_m:.3f}m, '
            f'skip_lateral={skip_lateral_shift}, '
            f'forward_component={forward_component_m:.3f}m, '
            f'final_forward={final_forward_m:.3f}m, '
            f'target_dist={target_distance_m:.3f}m。'
        )
        if use_s_curve_control:
            if (
                self.limit_bar_lateral_shift_next_action
                == 'upstairs_step_once'
            ):
                cruise_left_norm = (
                    self.upstairs_lateral_s_curve_control_cruise_left_norm
                )
                cruise_right_norm = (
                    self.upstairs_lateral_s_curve_control_cruise_right_norm
                )
                slow_left_norm = (
                    self.upstairs_lateral_s_curve_control_slow_left_norm
                )
                slow_right_norm = (
                    self.upstairs_lateral_s_curve_control_slow_right_norm
                )
                slowdown_distance_m = (
                    self.upstairs_lateral_s_curve_control_slowdown_distance_m
                )
                rollback_parameter = (
                    'upstairs_lateral_s_curve_control_enabled'
                )
            elif (
                self.limit_bar_lateral_shift_next_action
                == 'limit_bar_prepare'
            ):
                cruise_left_norm = (
                    self.limit_bar_lateral_s_curve_control_cruise_left_norm
                )
                cruise_right_norm = (
                    self.limit_bar_lateral_s_curve_control_cruise_right_norm
                )
                slow_left_norm = (
                    self.limit_bar_lateral_s_curve_control_slow_left_norm
                )
                slow_right_norm = (
                    self.limit_bar_lateral_s_curve_control_slow_right_norm
                )
                slowdown_distance_m = (
                    self.limit_bar_lateral_s_curve_control_slowdown_distance_m
                )
                rollback_parameter = (
                    'limit_bar_lateral_s_curve_control_enabled'
                )
            else:
                cruise_left_norm = self.lateral_s_curve_control_cruise_left_norm
                cruise_right_norm = self.lateral_s_curve_control_cruise_right_norm
                slow_left_norm = self.lateral_s_curve_control_slow_left_norm
                slow_right_norm = self.lateral_s_curve_control_slow_right_norm
                slowdown_distance_m = (
                    self.lateral_s_curve_control_slowdown_distance_m
                )
                rollback_parameter = 'lateral_s_curve_control_enabled'
            self._publish_feedback(
                f'第二阶段 S 曲线实际控制启用: branch={self._lateral_s_curve_shadow_branch_name()}, '
                f'route={control_plan.forward_m:.3f}m/{control_plan.lateral_m:+.3f}m, '
                f'cruise=[{cruise_left_norm:.3f},'
                f'{cruise_right_norm:.3f}], '
                f'slow=[{slow_left_norm:.3f},'
                f'{slow_right_norm:.3f}], '
                f'slowdown={slowdown_distance_m:.3f}m；'
                f'关闭 {rollback_parameter} 可立即回退旧 90deg 流程。'
            )
        elif self._lateral_s_curve_control_requested():
            fallback_reason = (
                'missing_plan'
                if control_plan is None
                else control_plan.reason
            )
            self._publish_feedback(
                f'第二阶段 S 曲线实际控制未启用: reason={fallback_reason}，'
                '本次自动回退旧 90deg 横向补偿流程。'
            )
        return True

    def _lateral_s_curve_shadow_branch_name(self) -> str:
        if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump':
            return 'hurdle'
        if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
            return 'upstairs'
        return 'limit_bar'

    def _lateral_s_curve_shadow_branch_enabled(self) -> bool:
        branch = self._lateral_s_curve_shadow_branch_name()
        if branch == 'hurdle':
            return self.hurdle_lateral_s_curve_shadow_enabled
        if branch == 'upstairs':
            return self.upstairs_lateral_s_curve_shadow_enabled
        return self.limit_bar_lateral_s_curve_shadow_enabled

    def _lateral_s_curve_control_branch_enabled(self) -> bool:
        branch = self._lateral_s_curve_shadow_branch_name()
        if branch == 'hurdle':
            return self.hurdle_lateral_s_curve_control_enabled
        if branch == 'upstairs':
            return self.upstairs_lateral_s_curve_control_enabled
        return self.limit_bar_lateral_s_curve_control_enabled

    def _lateral_s_curve_control_requested(self) -> bool:
        return (
            getattr(self, 'lateral_s_curve_control_enabled', False)
            and self._lateral_s_curve_control_branch_enabled()
        )

    def _reset_lateral_s_curve_control(self) -> None:
        self.lateral_s_curve_control_active = False
        self.lateral_s_curve_control_plan = None
        self.lateral_s_curve_control_start_x = None
        self.lateral_s_curve_control_start_y = None
        self.lateral_s_curve_control_start_sec = 0.0
        self.lateral_s_curve_control_arrival_sec = None
        self.lateral_s_curve_control_arrival_forward_m = None
        self.lateral_s_curve_control_arrival_cross_error_m = None
        self.lateral_s_curve_control_completion_max_forward_m = None
        self.lateral_s_curve_control_completion_since_sec = None
        self.limit_bar_lateral_s_curve_final_lateral_violation_since_sec = None
        self.limit_bar_lateral_s_curve_pre_stop_start_sec = None
        self.limit_bar_lateral_s_curve_pre_stop_completed = False
        self.limit_bar_lateral_s_curve_fail_open_cross_track_logged = False
        self.limit_bar_lateral_s_curve_fail_open_final_lateral_logged = False
        self.lateral_s_curve_control_completion_reason = ''
        self.lateral_s_curve_control_completion_recovery_active = False
        self.lateral_s_curve_control_completion_recovery_target_forward_m = None
        self.lateral_s_curve_control_completion_recovery_trigger = ''
        self.limit_bar_lateral_s_curve_control_completed = False
        self._reset_hurdle_lateral_s_curve_visual_confirmation()
        self.last_hurdle_lateral_s_curve_visual_guard_log_sec = 0.0

    def _reset_lateral_s_curve_shadow(self) -> None:
        self.lateral_s_curve_shadow_plan = None
        self.lateral_s_curve_shadow_branch = ''
        self.lateral_s_curve_shadow_start_sec = 0.0
        self.lateral_s_curve_shadow_summary_logged = False
        self._reset_lateral_s_curve_control()

    def _prepare_lateral_s_curve_shadow(
        self,
        *,
        now: float,
        shift_m: float,
        side_delta_rad: float,
        forward_component_m: float,
        final_forward_m: float,
    ) -> None:
        self._reset_lateral_s_curve_shadow()
        shadow_requested = (
            self.lateral_s_curve_shadow_enabled
            and self._lateral_s_curve_shadow_branch_enabled()
        )
        control_requested = self._lateral_s_curve_control_requested()
        if not shadow_requested and not control_requested:
            return

        branch = self._lateral_s_curve_shadow_branch_name()
        max_heading_rad = (
            self.limit_bar_lateral_s_curve_max_heading_rad
            if branch == 'limit_bar'
            else self.lateral_s_curve_shadow_max_heading_rad
        )
        max_curvature_m_inv = (
            self.limit_bar_lateral_s_curve_max_curvature_m_inv
            if branch == 'limit_bar'
            else self.lateral_s_curve_shadow_max_curvature_m_inv
        )
        lateral_m = shift_m * math.sin(side_delta_rad)
        forward_m = forward_component_m + final_forward_m
        plan = build_lateral_s_curve_plan(
            forward_m=forward_m,
            lateral_m=lateral_m,
            min_forward_m=self.lateral_s_curve_shadow_min_forward_m,
            max_abs_lateral_m=self.lateral_s_curve_shadow_max_abs_lateral_m,
            max_abs_heading_rad=max_heading_rad,
            max_abs_curvature_m_inv=max_curvature_m_inv,
        )
        adaptive_text = ''
        if (
            branch == 'hurdle'
            and control_requested
            and getattr(
                self,
                'hurdle_lateral_s_curve_adaptive_clamp_enabled',
                False,
            )
            and plan.reason == 'heading_above_limit'
            and forward_m > 0.0
            and 0.0 < self.lateral_s_curve_shadow_max_heading_rad < math.pi / 2.0
            and abs(lateral_m) > 1e-9
        ):
            max_lateral_m = (
                forward_m
                * math.tan(self.lateral_s_curve_shadow_max_heading_rad)
                / 1.875
            )
            clamped_lateral_m = math.copysign(
                min(abs(lateral_m), max_lateral_m * (1.0 - 1e-6)),
                lateral_m,
            )
            clamp_scale = abs(clamped_lateral_m / lateral_m)
            min_scale = getattr(
                self,
                'hurdle_lateral_s_curve_adaptive_clamp_min_scale',
                0.90,
            )
            if min_scale <= clamp_scale < 1.0:
                clamped_plan = build_lateral_s_curve_plan(
                    forward_m=forward_m,
                    lateral_m=clamped_lateral_m,
                    min_forward_m=self.lateral_s_curve_shadow_min_forward_m,
                    max_abs_lateral_m=(
                        self.lateral_s_curve_shadow_max_abs_lateral_m
                    ),
                    max_abs_heading_rad=(
                        self.lateral_s_curve_shadow_max_heading_rad
                    ),
                    max_abs_curvature_m_inv=(
                        self.lateral_s_curve_shadow_max_curvature_m_inv
                    ),
                )
                if clamped_plan.feasible:
                    plan = clamped_plan
                    adaptive_text = (
                        f', adaptive_lateral={lateral_m:+.3f}->'
                        f'{clamped_lateral_m:+.3f}m(scale={clamp_scale:.3f})'
                    )
        self.lateral_s_curve_shadow_plan = plan
        self.lateral_s_curve_shadow_branch = branch
        self.lateral_s_curve_shadow_start_sec = now
        peak_heading = plan.sample(plan.forward_m * 0.5).heading_rad
        legacy_linear_m = shift_m + final_forward_m
        control_mode = (
            'actual_pending'
            if control_requested and plan.feasible
            else 'legacy_fallback'
            if control_requested
            else 'legacy_90deg'
        )
        self._publish_feedback(
            f'第二阶段 S 曲线候选计划: branch={branch}, '
            f'forward={plan.forward_m:.3f}m, lateral={plan.lateral_m:+.3f}m, '
            f'path_length={plan.path_length_m:.3f}m, '
            f'peak_heading={math.degrees(peak_heading):+.1f}deg, '
            f'max_curvature={plan.max_abs_curvature_m_inv:.2f}/m, '
            f'legacy_linear={legacy_linear_m:.3f}m, '
            f'feasible={plan.feasible}({plan.reason}), '
            f'control={control_mode}{adaptive_text}。'
        )

    def _complete_lateral_s_curve_shadow(
        self,
        now: float,
        *,
        completion_reason: str,
    ) -> None:
        plan = getattr(self, 'lateral_s_curve_shadow_plan', None)
        if plan is None or getattr(
            self,
            'lateral_s_curve_shadow_summary_logged',
            False,
        ):
            return
        self.lateral_s_curve_shadow_summary_logged = True
        elapsed_sec = max(
            0.0,
            now - getattr(self, 'lateral_s_curve_shadow_start_sec', now),
        )
        self._publish_feedback(
            f'第二阶段 S 曲线影子结果: '
            f'branch={self.lateral_s_curve_shadow_branch}, '
            f'feasible={plan.feasible}({plan.reason}), '
            f'route={plan.forward_m:.3f}m/{plan.lateral_m:+.3f}m, '
            f'path_length={plan.path_length_m:.3f}m, '
            f'legacy_elapsed={elapsed_sec:.2f}s, '
            f'completion={completion_reason}, '
            'control=legacy_90deg。'
        )

    def _refresh_limit_bar_lateral_shift_target(self, now: float) -> None:
        if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
            if self.upstairs_tag_yaw_reference_mode == 'fixed_absolute':
                # 上台阶绝对零度模式固定使用进入分支时的一次性 Tag 锁存，
                # 不允许横移开始前再被最新视觉样本覆盖。
                return
            recent_samples = self._recent_upstairs_tag_samples(now)
            if len(recent_samples) < self.upstairs_tag_min_samples:
                return
            tag_x = self._median([sample.x_m for sample in recent_samples])
            tag_z = self._median([sample.z_m for sample in recent_samples])
            if not math.isfinite(tag_x) or not math.isfinite(tag_z) or tag_z <= 1e-3:
                return
            self.limit_bar_lateral_shift_center_x_m = tag_x + self.upstairs_tag_to_center_x_m
            self.limit_bar_lateral_shift_center_z_m = tag_z
            self.upstairs_detection.distance_m = tag_z
            self.upstairs_detection.lateral_m = tag_x
            self.upstairs_detection.nearest_stamp_sec = now
            self._publish_feedback(
                f'上台阶横向补偿使用最新 tag 重新锁定: '
                f'tag_x={tag_x:.3f}m, tag_z={tag_z:.3f}m, '
                f'center_x={self.limit_bar_lateral_shift_center_x_m:.3f}m。'
            )
            return
        if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump':
            recent_samples = self._recent_hurdle_tag_samples(now)
            if len(recent_samples) >= self.hurdle_tag_min_samples:
                tag_x = self._median([sample.x_m for sample in recent_samples])
                tag_z = self._median([sample.z_m for sample in recent_samples])
                if math.isfinite(tag_x) and math.isfinite(tag_z) and tag_z > 1e-3:
                    self.limit_bar_lateral_shift_center_x_m = (
                        tag_x + self.hurdle_tag_to_center_x_m
                    )
                    self.limit_bar_lateral_shift_center_z_m = tag_z
                    self.hurdle_detection.distance_m = tag_z
                    self.hurdle_detection.lateral_m = tag_x
                    self.hurdle_detection.nearest_stamp_sec = now
                    self._publish_feedback(
                        f'高墙横向补偿使用最新 tag 重新锁定: '
                        f'tag_x={tag_x:.3f}m, tag_z={tag_z:.3f}m, '
                        f'center_x={self.limit_bar_lateral_shift_center_x_m:.3f}m。'
                    )
                    return
            if self.hurdle_locked_tag_valid:
                self.limit_bar_lateral_shift_center_x_m = self.hurdle_locked_center_x_m
                self.limit_bar_lateral_shift_center_z_m = self.hurdle_locked_tag_z_m
                self.hurdle_detection.distance_m = self.hurdle_locked_tag_z_m
                self.hurdle_detection.lateral_m = self.hurdle_locked_tag_x_m
                self.hurdle_detection.nearest_stamp_sec = self.hurdle_locked_tag_stamp_sec
                self._publish_feedback(
                    f'高墙横向补偿未取得新鲜 tag，回退初始锁定 tag: '
                    f'tag_x={self.hurdle_locked_tag_x_m:.3f}m, '
                    f'tag_z={self.hurdle_locked_tag_z_m:.3f}m, '
                    f'center_x={self.hurdle_locked_center_x_m:.3f}m。'
                )
                return
            return
        recent_samples = self._recent_limit_bar_tag_samples(now)
        if len(recent_samples) < self.limit_bar_tag_min_samples:
            return
        tag_x = self._median([sample.x_m for sample in recent_samples])
        tag_z = self._median([sample.z_m for sample in recent_samples])
        if not math.isfinite(tag_x) or not math.isfinite(tag_z) or tag_z <= 1e-3:
            return
        self.limit_bar_lateral_shift_center_x_m = tag_x + self.limit_bar_tag_to_center_x_m
        self.limit_bar_lateral_shift_center_z_m = tag_z
        self.limit_detection.distance_m = tag_z
        self.limit_detection.lateral_m = tag_x
        self.limit_detection.nearest_stamp_sec = now
        self._publish_feedback(
            f'限高杆横向补偿使用最新 tag 重新锁定: '
            f'tag_x={tag_x:.3f}m, tag_z={tag_z:.3f}m, '
            f'center_x={self.limit_bar_lateral_shift_center_x_m:.3f}m。'
        )

    def _replan_limit_bar_lateral_shift_from_latest_tag(self, now: float) -> bool:
        old_center_x_m = self.limit_bar_lateral_shift_center_x_m
        old_center_z_m = self.limit_bar_lateral_shift_center_z_m
        old_shift_m = self.limit_bar_lateral_shift_distance_m
        old_final_forward_m = self.limit_bar_lateral_shift_final_forward_m

        self._refresh_limit_bar_lateral_shift_target(now)
        center_x_m = self.limit_bar_lateral_shift_center_x_m
        center_z_m = self.limit_bar_lateral_shift_center_z_m
        if not math.isfinite(center_x_m) or not math.isfinite(center_z_m):
            self._publish_feedback(
                f'{self._lateral_shift_label()}横向补偿开始前重算失败: tag 中心线数据无效，'
                f'沿用旧计划 shift={old_shift_m:.3f}m。'
            )
            return False

        yaw_abs_rad = self._lateral_shift_yaw_abs_rad()
        sin_yaw = max(1e-6, math.sin(yaw_abs_rad))
        abs_center_x_m = abs(center_x_m)
        lateral_deadband_m = self._lateral_shift_deadband_m()
        arrival_tolerance_m = self._lateral_shift_arrival_tolerance_m()
        skip_lateral_shift = (
            self.limit_bar_lateral_shift_skip_below_m > 0.0
            and abs_center_x_m <= self.limit_bar_lateral_shift_skip_below_m
        )
        if skip_lateral_shift or abs_center_x_m <= lateral_deadband_m:
            side_delta_rad = 0.0
            raw_shift_m = 0.0
            requested_shift_m = 0.0
            shift_m = 0.0
        else:
            side_delta_rad = (
                self._lateral_shift_yaw_sign()
                * math.copysign(yaw_abs_rad, center_x_m)
            )
            raw_shift_m = abs_center_x_m / sin_yaw
            requested_shift_m = raw_shift_m * self._lateral_shift_distance_scale()
            max_shift_distance_m = self._lateral_shift_max_distance_m()
            if max_shift_distance_m > 0.0:
                shift_m = min(requested_shift_m, max_shift_distance_m)
            else:
                shift_m = requested_shift_m

        base_yaw = self._normalize_angle(self.limit_bar_lateral_shift_base_yaw)
        side_yaw = self._normalize_angle(base_yaw + side_delta_rad)
        forward_component_m = shift_m * math.cos(abs(side_delta_rad))
        target_distance_m = (
            self.hurdle_lateral_shift_target_distance_m
            if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump'
            else self.upstairs_lateral_shift_target_distance_m
            if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once'
            else self.limit_bar_lateral_shift_target_distance_m
        )
        final_forward_m = max(
            0.0,
            center_z_m - forward_component_m - target_distance_m,
        )

        self.limit_bar_lateral_shift_side_yaw = side_yaw
        self.limit_bar_lateral_shift_distance_m = shift_m
        self.limit_bar_lateral_shift_raw_distance_m = raw_shift_m
        self.limit_bar_lateral_shift_requested_distance_m = requested_shift_m
        self.limit_bar_lateral_shift_forward_component_m = forward_component_m
        self.limit_bar_lateral_shift_final_forward_m = final_forward_m
        replanned_phase = (
            'align_out'
            if shift_m > arrival_tolerance_m
            else 'final_forward'
        )
        self._set_limit_bar_lateral_shift_phase(replanned_phase, now)
        self.limit_bar_lateral_shift_segment_start_x = self.pose_x
        self.limit_bar_lateral_shift_segment_start_y = self.pose_y

        clipped = shift_m + 1e-6 < requested_shift_m
        self._publish_feedback(
            f'{self._lateral_shift_label()}横向补偿按最新 tag 重算: '
            f'center_x {old_center_x_m:+.3f}->{center_x_m:+.3f}m, '
            f'center_z {old_center_z_m:.3f}->{center_z_m:.3f}m, '
            f'shift {old_shift_m:.3f}->{shift_m:.3f}m, '
            f'final_forward {old_final_forward_m:.3f}->{final_forward_m:.3f}m, '
            f'trigger_deadband={lateral_deadband_m:.3f}m, '
            f'arrival_tolerance={arrival_tolerance_m:.3f}m, '
            f'phase={self.limit_bar_lateral_shift_phase}, '
            f'side_yaw={math.degrees(side_yaw):.1f}deg, '
            f'skip_lateral={skip_lateral_shift}, clipped={clipped}。'
        )
        return True

    def _upstairs_live_tag_alignment(
        self,
        now: float,
    ) -> Optional[tuple[float, float, float, float, float]]:
        if self.limit_bar_lateral_shift_next_action != 'upstairs_step_once':
            return None
        if self.limit_bar_lateral_shift_base_yaw is None or self.pose_yaw is None:
            return None
        recent_samples = self._recent_upstairs_tag_samples(now)
        if len(recent_samples) < self.upstairs_tag_min_samples:
            return None
        tag_x = self._median([sample.x_m for sample in recent_samples])
        tag_z = self._median([sample.z_m for sample in recent_samples])
        if not math.isfinite(tag_x) or not math.isfinite(tag_z) or tag_z <= 1e-3:
            return None
        live_center_x = tag_x + self.upstairs_tag_to_center_x_m
        if not math.isfinite(live_center_x):
            return None
        if self.upstairs_tag_yaw_reference_mode == 'fixed_absolute':
            vision_yaw_corr = 0.0
            target_yaw_if_live = self._upstairs_fixed_reference_target_yaw()
        else:
            vision_yaw_corr = -math.atan2(live_center_x, tag_z)
            target_yaw_if_live = self._normalize_angle(
                self.pose_yaw + vision_yaw_corr
            )
        self.upstairs_detection.distance_m = tag_z
        self.upstairs_detection.lateral_m = tag_x
        self.upstairs_detection.nearest_stamp_sec = now
        return tag_x, tag_z, live_center_x, vision_yaw_corr, target_yaw_if_live

    def _upstairs_live_tag_alignment_text(
        self,
        live_alignment: Optional[tuple[float, float, float, float, float]],
    ) -> str:
        if live_alignment is None:
            return 'live_center_x=none, vision_yaw_corr=none, target_yaw_if_live=none'
        tag_x, tag_z, live_center_x, vision_yaw_corr, target_yaw_if_live = live_alignment
        return (
            f'live_tag_x={tag_x:+.3f}m, live_tag_z={tag_z:.3f}m, '
            f'live_center_x={live_center_x:+.3f}m, '
            f'vision_yaw_corr={math.degrees(vision_yaw_corr):+.1f}deg, '
            f'target_yaw_if_live={math.degrees(target_yaw_if_live):.1f}deg'
        )

    def _upstairs_live_tag_needs_replan(
        self,
        live_alignment: Optional[tuple[float, float, float, float, float]],
    ) -> bool:
        if self.upstairs_tag_yaw_reference_mode == 'fixed_absolute':
            return False
        if not self.upstairs_lateral_shift_live_check_enabled:
            return False
        if live_alignment is None:
            return False
        _, _, live_center_x, vision_yaw_corr, _ = live_alignment
        return (
            abs(live_center_x) > self.upstairs_lateral_shift_live_check_center_gate_m
            or abs(vision_yaw_corr) > self.upstairs_lateral_shift_live_check_yaw_gate_rad
        )

    def _upstairs_final_forward_tag_yaw_corrected_norms(
        self,
        base_left_norm: float,
        base_right_norm: float,
        live_alignment: Optional[tuple[float, float, float, float, float]],
    ) -> tuple[float, float, float]:
        if (
            self.upstairs_tag_yaw_reference_mode == 'fixed_absolute'
            or
            not self.upstairs_final_forward_tag_yaw_correction_enabled
            or live_alignment is None
        ):
            return base_left_norm, base_right_norm, 0.0
        _, _, _, vision_yaw_corr, _ = live_alignment
        yaw_abs = abs(vision_yaw_corr)
        if yaw_abs <= self.upstairs_final_forward_tag_yaw_deadband_rad:
            return base_left_norm, base_right_norm, 0.0
        delta_norm = self.upstairs_final_forward_tag_yaw_gain_norm_per_rad * (
            yaw_abs - self.upstairs_final_forward_tag_yaw_deadband_rad
        )
        delta_norm = min(
            self.upstairs_final_forward_tag_yaw_max_delta_norm,
            max(0.0, delta_norm),
        )
        if vision_yaw_corr > 0.0:
            left_norm = self._clamp_norm(base_left_norm - delta_norm)
            right_norm = self._clamp_norm(base_right_norm + delta_norm)
        else:
            left_norm = self._clamp_norm(base_left_norm + delta_norm)
            right_norm = self._clamp_norm(base_right_norm - delta_norm)
        return left_norm, right_norm, delta_norm

    def _lateral_shift_label(self) -> str:
        if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump':
            return '高墙'
        if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
            return '上台阶'
        return '限高杆'

    def _lateral_shift_target_action_text(self) -> str:
        if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump':
            return '起跳'
        if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
            return '上台阶'
        return '趴下'

    def _lateral_shift_yaw_abs_rad(self) -> float:
        if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
            return self.upstairs_lateral_shift_yaw_rad
        return self.limit_bar_lateral_shift_yaw_rad

    def _lateral_shift_yaw_sign(self) -> float:
        if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump':
            return self.hurdle_lateral_shift_yaw_sign
        if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
            return self.upstairs_lateral_shift_yaw_sign
        return self.limit_bar_lateral_shift_yaw_sign

    def _lateral_shift_distance_scale(self) -> float:
        if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump':
            return self.hurdle_lateral_shift_distance_scale
        if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
            return self.upstairs_lateral_shift_distance_scale
        return self.limit_bar_lateral_shift_distance_scale

    def _lateral_shift_max_distance_m(self) -> float:
        if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
            return self.upstairs_lateral_shift_max_distance_m
        return self.limit_bar_lateral_shift_max_distance_m

    def _lateral_shift_deadband_m(self) -> float:
        if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump':
            return self.hurdle_lateral_shift_deadband_m
        if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
            return self.upstairs_lateral_shift_deadband_m
        return self.limit_bar_lateral_shift_deadband_m

    def _lateral_shift_arrival_tolerance_m(self) -> float:
        if not getattr(self, 'limit_bar_lateral_precision_turn_enabled', False):
            return self._lateral_shift_deadband_m()
        if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump':
            return self.hurdle_lateral_shift_arrival_tolerance_m
        if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
            return self.upstairs_lateral_shift_arrival_tolerance_m
        return self.limit_bar_lateral_shift_arrival_tolerance_m

    def _reset_limit_bar_lateral_turn_guard(self) -> None:
        self.limit_bar_lateral_turn_stable_since_sec = None
        self.limit_bar_lateral_turn_last_direction = 0
        self.limit_bar_lateral_turn_pending_direction = 0
        self.limit_bar_lateral_turn_reverse_hold_until_sec = 0.0

    def _set_limit_bar_lateral_shift_phase(self, phase: str, now: float) -> None:
        self.limit_bar_lateral_shift_phase = phase
        self.limit_bar_lateral_shift_phase_start_sec = now
        self.limit_bar_pre_duck_global_yaw_hold_active = False
        self._reset_limit_bar_lateral_turn_guard()

    def _latch_limit_bar_lateral_s_curve_completion(
        self,
        now: float,
        forward_m: float,
        endpoint_cross_error_m: float,
        reason: str,
    ) -> None:
        if self.lateral_s_curve_control_completion_reason:
            return
        self.lateral_s_curve_control_completion_reason = reason
        self.lateral_s_curve_control_arrival_sec = now
        self.lateral_s_curve_control_arrival_forward_m = forward_m
        self.lateral_s_curve_control_arrival_cross_error_m = (
            endpoint_cross_error_m
        )
        self.lateral_s_curve_control_completion_since_sec = None
        self._reset_limit_bar_lateral_turn_guard()

    def _limit_bar_lateral_phase_timeout_sec(self, phase: str) -> float:
        if phase == 's_curve':
            if (
                self.limit_bar_lateral_shift_next_action
                == 'upstairs_step_once'
            ):
                return getattr(
                    self,
                    'upstairs_lateral_s_curve_control_timeout_sec',
                    0.0,
                )
            if (
                self.limit_bar_lateral_shift_next_action
                == 'limit_bar_prepare'
            ):
                return getattr(
                    self,
                    'limit_bar_lateral_s_curve_control_timeout_sec',
                    0.0,
                )
            return getattr(
                self,
                'lateral_s_curve_control_timeout_sec',
                0.0,
            )
        if not getattr(self, 'limit_bar_lateral_precision_turn_enabled', False):
            return 0.0
        if phase in ('align_out', 'align_back', 'upstairs_final_align'):
            return getattr(self, 'limit_bar_lateral_turn_timeout_sec', 0.0)
        if phase == 'shift':
            return getattr(self, 'limit_bar_lateral_shift_timeout_sec', 0.0)
        if phase == 'final_forward':
            return getattr(
                self,
                'limit_bar_lateral_final_forward_timeout_sec',
                0.0,
            )
        return 0.0

    def _limit_bar_lateral_phase_timed_out(self, now: float, phase: str) -> bool:
        timeout_sec = self._limit_bar_lateral_phase_timeout_sec(phase)
        phase_start_sec = getattr(
            self,
            'limit_bar_lateral_shift_phase_start_sec',
            0.0,
        )
        return (
            timeout_sec > 0.0
            and phase_start_sec > 0.0
            and (now - phase_start_sec) >= timeout_sec
        )

    def _fail_limit_bar_lateral_phase_timeout(
        self,
        now: float,
        phase: str,
        shift_label: str,
        *,
        yaw_error: Optional[float] = None,
        tolerance_rad: Optional[float] = None,
    ) -> bool:
        """Stop a still-active phase after its completion checks have run."""
        if getattr(self, 'state', None) == getattr(self, 'STATE_FAILED', 'FAILED'):
            return False
        if not self._limit_bar_lateral_phase_timed_out(now, phase):
            return False
        timeout_sec = self._limit_bar_lateral_phase_timeout_sec(phase)
        settle_grace_sec = getattr(
            self,
            'limit_bar_lateral_turn_timeout_settle_grace_sec',
            0.0,
        )
        if (
            settle_grace_sec > 0.0
            and self.limit_bar_lateral_shift_next_action == 'limit_bar_prepare'
            and phase in ('align_out', 'align_back')
            and yaw_error is not None
            and tolerance_rad is not None
            and abs(yaw_error) <= tolerance_rad
            and (
                now - self.limit_bar_lateral_shift_phase_start_sec
            ) < timeout_sec + settle_grace_sec
        ):
            return False
        if (
            phase == 's_curve'
            and self.limit_bar_lateral_shift_next_action == 'limit_bar_prepare'
            and getattr(
                self,
                'limit_bar_lateral_s_curve_fail_open_enabled',
                False,
            )
        ):
            progress = self._lateral_s_curve_control_progress()
            plan = self.lateral_s_curve_control_plan
            if (
                progress[0] is not None
                and progress[1] is not None
                and plan is not None
            ):
                forward_m, lateral_m = progress
                endpoint_cross_error_m = plan.lateral_m - lateral_m
                self._publish_step_override(0, 0.0, 0.0)
                self._latch_limit_bar_lateral_s_curve_completion(
                    now,
                    forward_m,
                    endpoint_cross_error_m,
                    'fail_open_timeout',
                )
                self.limit_bar_lateral_shift_phase_start_sec = now
                self._publish_feedback(
                    f'{shift_label} S 曲线控制超时已旁路: '
                    f'timeout={timeout_sec:.1f}s, '
                    f'forward={forward_m:.3f}/{plan.forward_m:.3f}m, '
                    f'cross_err={endpoint_cross_error_m:+.3f}m；'
                    '已停止前进并锁存完成，继续进入趴下前收尾流程。'
                )
                return True
        self._fail(
            f'{shift_label}横向补偿阶段超时: phase={phase}, '
            f'timeout={timeout_sec:.1f}s, '
            f'pose_yaw={math.degrees(self.pose_yaw):.1f}deg。'
        )
        return True

    def _limit_bar_lateral_precision_turn_active(self) -> bool:
        return (
            getattr(self, 'limit_bar_lateral_precision_turn_enabled', False)
            and getattr(self, 'state', '') == self.STATE_LIMIT_BAR_LATERAL_SHIFT
        )

    def _limit_bar_lateral_yaw_rate_ready(self) -> bool:
        if not self._limit_bar_lateral_precision_turn_active():
            return True
        return (
            abs(getattr(self, 'pose_yaw_rate_rad_s', 0.0))
            <= self.limit_bar_lateral_turn_complete_yaw_rate_rad_s
        )

    def _limit_bar_lateral_turn_ready(
        self,
        now: float,
        yaw_error: float,
        tolerance_rad: float,
    ) -> bool:
        if not self._limit_bar_lateral_precision_turn_active():
            return abs(yaw_error) <= tolerance_rad
        if (
            abs(yaw_error) > tolerance_rad
            or not self._limit_bar_lateral_yaw_rate_ready()
        ):
            self.limit_bar_lateral_turn_stable_since_sec = None
            return False
        if self.limit_bar_lateral_turn_stable_since_sec is None:
            self.limit_bar_lateral_turn_stable_since_sec = now
        return (
            now - self.limit_bar_lateral_turn_stable_since_sec
            >= self.limit_bar_lateral_turn_hold_sec
        )

    def _limit_bar_lateral_shift_progress(self, yaw_rad: float) -> tuple[Optional[float], Optional[float]]:
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.limit_bar_lateral_shift_segment_start_x is None
            or self.limit_bar_lateral_shift_segment_start_y is None
        ):
            return None, None
        dx = self.pose_x - self.limit_bar_lateral_shift_segment_start_x
        dy = self.pose_y - self.limit_bar_lateral_shift_segment_start_y
        forward_x, forward_y = self._odom_forward_unit(yaw_rad)
        raw_m = dx * forward_x + dy * forward_y
        if self.limit_bar_lateral_shift_use_euclidean_progress:
            return math.hypot(dx, dy), raw_m
        return raw_m, raw_m

    def _fresh_hurdle_visual_distance(self, now: float) -> Optional[float]:
        distance_m = self._usable_distance(
            self.hurdle_visual_detection.distance_m
        )
        if distance_m is None:
            return None
        if self.hurdle_lateral_shift_visual_jump_max_age_sec > 0.0:
            age_sec = now - self.hurdle_visual_detection.nearest_stamp_sec
            if age_sec > self.hurdle_lateral_shift_visual_jump_max_age_sec:
                return None
        return distance_m

    def _hurdle_visual_jump_ready(self, now: float) -> bool:
        if not self.hurdle_lateral_shift_visual_jump_enabled:
            return False
        distance_m = self._fresh_hurdle_visual_distance(now)
        if distance_m is None:
            return False
        return distance_m <= self.hurdle_lateral_shift_visual_jump_distance_m

    def _reset_hurdle_lateral_s_curve_visual_confirmation(self) -> None:
        self.hurdle_lateral_s_curve_visual_close_sample_count = 0
        self.hurdle_lateral_s_curve_visual_close_start_sec = None

    def _hurdle_lateral_s_curve_visual_jump_status(
        self,
        now: float,
        remaining_m: float,
    ) -> tuple[bool, bool, str]:
        if not self._hurdle_visual_jump_ready(now):
            return False, False, 'visual_not_close'

        reasons = []
        max_remaining_m = getattr(
            self,
            'hurdle_lateral_s_curve_visual_jump_max_remaining_m',
            0.0,
        )
        if max_remaining_m > 0.0 and remaining_m > max_remaining_m:
            reasons.append(
                f'remaining={remaining_m:.3f}m>{max_remaining_m:.3f}m'
            )

        confirm_samples = getattr(
            self,
            'hurdle_lateral_s_curve_visual_jump_confirm_samples',
            1,
        )
        confirm_duration_sec = getattr(
            self,
            'hurdle_lateral_s_curve_visual_jump_confirm_duration_sec',
            0.0,
        )
        sample_count = getattr(
            self,
            'hurdle_lateral_s_curve_visual_close_sample_count',
            0,
        )
        close_start_sec = getattr(
            self,
            'hurdle_lateral_s_curve_visual_close_start_sec',
            None,
        )
        close_duration_sec = (
            max(0.0, now - close_start_sec)
            if close_start_sec is not None
            else 0.0
        )
        confirmation_checks = []
        if confirm_samples > 1:
            confirmation_checks.append(sample_count >= confirm_samples)
        if confirm_duration_sec > 0.0:
            confirmation_checks.append(close_duration_sec >= confirm_duration_sec)
        confirmation_ready = (
            any(confirmation_checks) if confirmation_checks else True
        )
        if not getattr(self.hurdle_visual_detection, 'triggered', False):
            confirmation_ready = False
        if not confirmation_ready:
            reasons.append(
                f'confirm={sample_count}/{confirm_samples},'
                f'{close_duration_sec:.2f}/{confirm_duration_sec:.2f}s'
            )

        tag_max_delta_m = getattr(
            self,
            'hurdle_lateral_s_curve_visual_jump_tag_max_delta_m',
            0.0,
        )
        if tag_max_delta_m > 0.0:
            tag_distances_m = [
                sample.z_m
                for sample in self._recent_hurdle_tag_samples(now)
                if math.isfinite(sample.z_m) and sample.z_m > 0.0
            ]
            visual_distance_m = self._fresh_hurdle_visual_distance(now)
            if tag_distances_m and visual_distance_m is not None:
                tag_distance_m = self._median(tag_distances_m)
                tag_delta_m = abs(tag_distance_m - visual_distance_m)
                if tag_delta_m > tag_max_delta_m:
                    reasons.append(
                        f'tag_delta={tag_delta_m:.3f}m>'
                        f'{tag_max_delta_m:.3f}m'
                    )

        return not reasons, True, ', '.join(reasons) if reasons else 'confirmed'

    def _relock_hurdle_legacy_final_forward(self, now: float) -> None:
        if (
            self.limit_bar_lateral_shift_next_action != 'hurdle_wait_jump'
            or not self.hurdle_lateral_shift_legacy_relock_final_forward_enabled
        ):
            return

        old_forward_m = self.limit_bar_lateral_shift_final_forward_m
        candidates: List[tuple[str, float]] = []
        tag_samples = self._recent_hurdle_tag_samples(now)
        valid_tag_z = [
            sample.z_m
            for sample in tag_samples
            if math.isfinite(sample.z_m) and sample.z_m > 0.0
        ]
        if (
            len(valid_tag_z)
            >= self.hurdle_lateral_shift_legacy_relock_min_tag_samples
        ):
            tag_z_m = self._median(valid_tag_z)
            tag_forward_m = max(
                0.0,
                tag_z_m - self.hurdle_lateral_shift_target_distance_m,
            )
            candidates.append((f'tag_z={tag_z_m:.3f}m', tag_forward_m))

        if self.hurdle_lateral_shift_legacy_relock_visual_cap_enabled:
            visual_distance_m = self._fresh_hurdle_visual_distance(now)
            if visual_distance_m is not None:
                visual_forward_m = max(
                    0.0,
                    visual_distance_m
                    - self.hurdle_lateral_shift_visual_jump_distance_m,
                )
                candidates.append(
                    (f'visual={visual_distance_m:.3f}m', visual_forward_m)
                )

        if not candidates:
            self._publish_feedback(
                '高墙旧横移流程末段距离重锁跳过: '
                f'fresh_tag={len(valid_tag_z)}/'
                f'{self.hurdle_lateral_shift_legacy_relock_min_tag_samples}，'
                '无可用新鲜视觉距离，保留原计划 '
                f'{old_forward_m:.3f}m。'
            )
            return

        requested_forward_m = min(value for _, value in candidates)
        relocked_forward_m = min(old_forward_m, requested_forward_m)
        max_reduction_m = (
            self.hurdle_lateral_shift_legacy_relock_max_reduction_m
        )
        if max_reduction_m > 0.0:
            relocked_forward_m = max(
                relocked_forward_m,
                max(0.0, old_forward_m - max_reduction_m),
            )
        self.limit_bar_lateral_shift_final_forward_m = relocked_forward_m
        source_text = ', '.join(
            f'{name}->remaining={value:.3f}m'
            for name, value in candidates
        )
        self._publish_feedback(
            f'高墙旧横移流程末段距离重锁: final_forward='
            f'{old_forward_m:.3f}->{relocked_forward_m:.3f}m, '
            f'{source_text}, max_reduction={max_reduction_m:.3f}m；'
            '仅缩短末段前进，不改变已锁定横移方向和距离。'
        )

    def _scale_limit_bar_tag_turn_norms(
        self,
        yaw_error: float,
        left_norm: float,
        right_norm: float,
    ) -> tuple[float, float]:
        if not self.limit_bar_tag_align_turn_scale_enabled:
            return left_norm, right_norm

        cmd_wz = self.limit_bar_tag_align_yaw_gain_per_rad * yaw_error
        if abs(cmd_wz) > 1e-6:
            cmd_wz = math.copysign(
                min(
                    self.limit_bar_tag_align_max_abs_angular_z,
                    max(self.limit_bar_tag_align_min_abs_angular_z, abs(cmd_wz)),
                ),
                cmd_wz,
            )
        scale = min(1.0, abs(cmd_wz) / self.limit_bar_tag_align_max_abs_angular_z)
        scale = max(self.limit_bar_tag_align_turn_min_scale, scale)
        return (
            self._clamp_norm(left_norm * scale),
            self._clamp_norm(right_norm * scale),
        )

    def _limit_bar_tag_turn_norms(
        self,
        yaw_error: float,
        *,
        duck_align: bool = False,
    ) -> tuple[float, float]:
        if yaw_error >= 0.0:
            if duck_align:
                return (
                    self.limit_bar_lateral_shift_duck_align_ccw_left_norm,
                    self.limit_bar_lateral_shift_duck_align_ccw_right_norm,
                )
            left_norm = self.limit_bar_tag_align_ccw_left_norm
            right_norm = self.limit_bar_tag_align_ccw_right_norm
            return self._scale_limit_bar_tag_turn_norms(yaw_error, left_norm, right_norm)
        if duck_align:
            return (
                self.limit_bar_lateral_shift_duck_align_cw_left_norm,
                self.limit_bar_lateral_shift_duck_align_cw_right_norm,
            )
        left_norm = self.limit_bar_tag_align_cw_left_norm
        right_norm = self.limit_bar_tag_align_cw_right_norm
        return self._scale_limit_bar_tag_turn_norms(yaw_error, left_norm, right_norm)

    def _hurdle_fine_yaw_turn_norms(
        self,
        yaw_error: float,
    ) -> tuple[float, float]:
        full_error_rad = max(
            1e-6,
            self.hurdle_fine_yaw_turn_full_error_rad,
        )
        scale = min(
            1.0,
            max(
                self.hurdle_fine_yaw_turn_min_scale,
                abs(yaw_error) / full_error_rad,
            ),
        )
        if yaw_error >= 0.0:
            left_norm = self.hurdle_fine_yaw_turn_ccw_left_norm
            right_norm = self.hurdle_fine_yaw_turn_ccw_right_norm
        else:
            left_norm = self.hurdle_fine_yaw_turn_cw_left_norm
            right_norm = self.hurdle_fine_yaw_turn_cw_right_norm
        return (
            self._clamp_norm(left_norm * scale),
            self._clamp_norm(right_norm * scale),
        )

    def _upstairs_lateral_s_curve_final_turn_override(
        self,
        yaw_error: float,
    ) -> tuple[int, float, float]:
        turn_norm = getattr(
            self,
            'upstairs_lateral_s_curve_control_final_align_norm',
            0.08,
        )
        if yaw_error >= 0.0:
            left_norm = -turn_norm
            right_norm = turn_norm
        else:
            left_norm = turn_norm
            right_norm = -turn_norm
        return (
            self.limit_bar_tag_align_mode,
            self._clamp_norm(left_norm),
            self._clamp_norm(right_norm),
        )

    def _limit_bar_lateral_s_curve_stationary_turn_override(
        self,
        yaw_error: float,
    ) -> tuple[int, float, float]:
        turn_norm = getattr(
            self,
            'limit_bar_lateral_s_curve_control_final_align_norm',
            0.08,
        )
        if (
            abs(yaw_error)
            >= getattr(self, 'limit_bar_lateral_turn_far_error_rad', math.inf)
        ):
            turn_norm *= getattr(
                self,
                'limit_bar_lateral_turn_far_scale',
                1.0,
            )
        turn_norm = min(1.0, max(0.0, turn_norm))
        if yaw_error >= 0.0:
            left_norm = -turn_norm
            right_norm = turn_norm
        else:
            left_norm = turn_norm
            right_norm = -turn_norm
        return (
            self.limit_bar_tag_align_mode,
            self._clamp_norm(left_norm),
            self._clamp_norm(right_norm),
        )

    def _limit_bar_pre_duck_stationary_turn_override(
        self,
        yaw_error: float,
    ) -> tuple[int, float, float]:
        if self._limit_bar_lateral_precision_turn_active():
            guarded_mode, guarded_left, guarded_right = (
                self._limit_bar_lateral_shift_turn_override(
                    yaw_error,
                    duck_align=True,
                )
            )
            if (
                guarded_mode == 0
                or (
                    abs(guarded_left) <= 1e-9
                    and abs(guarded_right) <= 1e-9
                )
            ):
                return 0, 0.0, 0.0
        turn_norm = getattr(
            self,
            'limit_bar_pre_duck_stationary_turn_norm',
            0.08,
        )
        if (
            abs(yaw_error)
            >= getattr(self, 'limit_bar_lateral_turn_far_error_rad', math.inf)
        ):
            turn_norm *= getattr(
                self,
                'limit_bar_lateral_turn_far_scale',
                1.0,
            )
        turn_norm = min(1.0, max(0.0, turn_norm))
        if yaw_error >= 0.0:
            left_norm = -turn_norm
            right_norm = turn_norm
        else:
            left_norm = turn_norm
            right_norm = -turn_norm
        return (
            self.limit_bar_tag_align_mode,
            self._clamp_norm(left_norm),
            self._clamp_norm(right_norm),
        )

    def _limit_bar_lateral_shift_turn_override(
        self,
        yaw_error: float,
        *,
        duck_align: bool = False,
        hurdle_s_curve: bool = False,
    ) -> tuple[int, float, float]:
        if not self._limit_bar_lateral_precision_turn_active():
            if hurdle_s_curve:
                left_norm, right_norm = self._hurdle_fine_yaw_turn_norms(
                    yaw_error
                )
            else:
                left_norm, right_norm = self._limit_bar_tag_turn_norms(
                    yaw_error,
                    duck_align=duck_align,
                )
            return self.limit_bar_tag_align_mode, left_norm, right_norm

        phase = self.limit_bar_lateral_shift_phase
        tolerance_rad = (
            self.limit_bar_pre_duck_global_yaw_tolerance_rad
            if phase == 'pre_duck_global_yaw_align'
            else self.upstairs_final_yaw_complete_gate_rad
            if phase == 'upstairs_final_align'
            else self.upstairs_final_yaw_complete_gate_rad
            if (
                duck_align
                and self.limit_bar_lateral_shift_next_action
                == 'upstairs_step_once'
            )
            else self.limit_bar_lateral_shift_duck_entry_yaw_gate_rad
            if duck_align
            else self.limit_bar_tag_align_tolerance_rad
            if phase in ('align_out', 'align_back')
            else self.limit_bar_lateral_shift_final_yaw_gate_rad
        )
        yaw_rate = self.pose_yaw_rate_rad_s
        if abs(yaw_error) <= tolerance_rad:
            return 0, 0.0, 0.0

        control_error = yaw_error
        moving_toward_target = yaw_error * yaw_rate > 0.0
        if moving_toward_target:
            predicted_error = (
                yaw_error
                - self.limit_bar_lateral_turn_yaw_damping_sec * yaw_rate
            )
            should_brake = (
                abs(yaw_rate) >= self.limit_bar_lateral_turn_brake_yaw_rate_rad_s
                and (
                    predicted_error * yaw_error <= 0.0
                    or abs(predicted_error) <= tolerance_rad
                )
            )
            if should_brake:
                return 0, 0.0, 0.0
            if predicted_error * yaw_error > 0.0:
                control_error = predicted_error

        now = self._now_sec()
        requested_direction = 1 if control_error > 0.0 else -1
        if self.limit_bar_lateral_turn_reverse_cooldown_sec > 0.0:
            if self.limit_bar_lateral_turn_pending_direction != 0:
                if requested_direction == self.limit_bar_lateral_turn_last_direction:
                    self.limit_bar_lateral_turn_pending_direction = 0
                    self.limit_bar_lateral_turn_reverse_hold_until_sec = 0.0
                elif requested_direction != self.limit_bar_lateral_turn_pending_direction:
                    self.limit_bar_lateral_turn_pending_direction = requested_direction
                    self.limit_bar_lateral_turn_reverse_hold_until_sec = (
                        now + self.limit_bar_lateral_turn_reverse_cooldown_sec
                    )
                    return 0, 0.0, 0.0
                elif now < self.limit_bar_lateral_turn_reverse_hold_until_sec:
                    return 0, 0.0, 0.0
                else:
                    self.limit_bar_lateral_turn_last_direction = requested_direction
                    self.limit_bar_lateral_turn_pending_direction = 0
                    self.limit_bar_lateral_turn_reverse_hold_until_sec = 0.0
            elif (
                self.limit_bar_lateral_turn_last_direction != 0
                and requested_direction != self.limit_bar_lateral_turn_last_direction
            ):
                self.limit_bar_lateral_turn_pending_direction = requested_direction
                self.limit_bar_lateral_turn_reverse_hold_until_sec = (
                    now + self.limit_bar_lateral_turn_reverse_cooldown_sec
                )
                return 0, 0.0, 0.0

        self.limit_bar_lateral_turn_last_direction = requested_direction
        if hurdle_s_curve:
            left_norm, right_norm = self._hurdle_fine_yaw_turn_norms(
                control_error
            )
        else:
            left_norm, right_norm = self._limit_bar_tag_turn_norms(
                control_error,
                duck_align=duck_align,
            )
        if abs(yaw_error) >= self.limit_bar_lateral_turn_far_error_rad:
            left_norm = self._clamp_norm(
                left_norm * self.limit_bar_lateral_turn_far_scale
            )
            right_norm = self._clamp_norm(
                right_norm * self.limit_bar_lateral_turn_far_scale
            )
        return self.limit_bar_tag_align_mode, left_norm, right_norm

    def _limit_bar_lateral_shift_walk_yaw_corrected_norms(
        self,
        base_left_norm: float,
        base_right_norm: float,
        yaw_error: float,
    ) -> tuple[float, float, float]:
        if not self.limit_bar_lateral_shift_walk_yaw_correction_enabled:
            return base_left_norm, base_right_norm, 0.0
        yaw_abs = abs(yaw_error)
        if yaw_abs <= self.limit_bar_lateral_shift_walk_yaw_deadband_rad:
            return base_left_norm, base_right_norm, 0.0
        delta_norm = self.limit_bar_lateral_shift_walk_yaw_gain_norm_per_rad * (
            yaw_abs - self.limit_bar_lateral_shift_walk_yaw_deadband_rad
        )
        delta_norm = min(
            self.limit_bar_lateral_shift_walk_yaw_max_delta_norm,
            max(0.0, delta_norm),
        )
        if yaw_error > 0.0:
            left_norm = self._clamp_norm(base_left_norm - delta_norm)
            right_norm = self._clamp_norm(base_right_norm + delta_norm)
        else:
            left_norm = self._clamp_norm(base_left_norm + delta_norm)
            right_norm = self._clamp_norm(base_right_norm - delta_norm)
        return left_norm, right_norm, delta_norm

    def _limit_bar_pre_duck_global_yaw_align_required(self) -> bool:
        return (
            self.limit_bar_pre_duck_global_yaw_align_enabled
            and self.limit_bar_lateral_shift_next_action == 'limit_bar_prepare'
        )

    def _finish_limit_bar_pre_duck_global_yaw_align(
        self,
        now: float,
        target_yaw: float,
        yaw_source: str,
        feedback: str,
    ) -> None:
        self._set_limit_bar_lateral_shift_phase('', now)
        self.limit_bar_lateral_shift_base_yaw = target_yaw
        self.duck_reference_yaw = target_yaw
        self.duck_reference_yaw_source = (
            f'limit_bar_pre_duck_{yaw_source}'
        )
        self.duck_reference_yaw_offset_rad = 0.0
        self.limit_bar_pre_duck_global_yaw_align_start_sec = None
        self.limit_bar_pre_duck_global_yaw_stable_since_sec = None
        self.limit_bar_pre_duck_global_yaw_hold_active = False
        if not self.duck_centerline_correction_enabled:
            self.duck_start_x = self.pose_x
            self.duck_start_y = self.pose_y
        self._publish_step_override(0, 0.0, 0.0)
        self._publish_feedback(feedback)
        self._enter_limit_bar_prepare(
            now,
            self.limit_bar_lateral_shift_reason,
            self._duck_reference_yaw_text(),
        )

    def _limit_bar_duck_prepare_yaw_override(
        self,
        yaw_error: float,
    ) -> tuple[int, float, float]:
        if yaw_error >= 0.0:
            return (
                self.limit_bar_duck_prepare_yaw_mode,
                self.limit_bar_duck_prepare_yaw_ccw_left_norm,
                self.limit_bar_duck_prepare_yaw_ccw_right_norm,
            )
        return (
            self.limit_bar_duck_prepare_yaw_mode,
            self.limit_bar_duck_prepare_yaw_cw_left_norm,
            self.limit_bar_duck_prepare_yaw_cw_right_norm,
        )

    def _run_limit_bar_duck_prepare_yaw_settle(self, now: float) -> bool:
        if not self.limit_bar_duck_prepare_yaw_settle_enabled:
            return False
        if self.duck_reference_yaw is None or self.pose_yaw is None:
            return False
        if self.limit_bar_duck_prepare_yaw_settle_start_sec is None:
            self.limit_bar_duck_prepare_yaw_settle_start_sec = now
            self.limit_bar_duck_prepare_yaw_stable_since_sec = None
            self.last_limit_bar_duck_prepare_yaw_log_sec = 0.0

        yaw_error = self._normalize_angle(self.duck_reference_yaw - self.pose_yaw)
        yaw_error_deg = math.degrees(yaw_error)
        if abs(yaw_error) <= self.limit_bar_duck_prepare_yaw_tolerance_rad:
            if self.limit_bar_duck_prepare_yaw_stable_since_sec is None:
                self.limit_bar_duck_prepare_yaw_stable_since_sec = now
            stable_sec = now - self.limit_bar_duck_prepare_yaw_stable_since_sec
            self._publish_step_override(
                self.duck_prepare_mode,
                self.duck_prepare_left_norm,
                self.duck_prepare_right_norm,
            )
            if stable_sec >= self.limit_bar_duck_prepare_yaw_hold_sec:
                self._publish_feedback(
                    f'限高杆趴下后 yaw 停稳完成: yaw_err={yaw_error_deg:.1f}deg, '
                    f'stable={stable_sec:.2f}s，开始锁定趴走中心线。'
                )
                self.limit_bar_duck_prepare_yaw_settle_start_sec = None
                self.limit_bar_duck_prepare_yaw_stable_since_sec = None
                return False
            if (now - self.last_limit_bar_duck_prepare_yaw_log_sec) >= 0.30:
                self.last_limit_bar_duck_prepare_yaw_log_sec = now
                self._publish_feedback(
                    f'限高杆趴下后 yaw 停稳保持中: yaw_err={yaw_error_deg:.1f}deg, '
                    f'stable={stable_sec:.2f}/{self.limit_bar_duck_prepare_yaw_hold_sec:.2f}s。'
                )
            return True

        self.limit_bar_duck_prepare_yaw_stable_since_sec = None
        elapsed_sec = now - self.limit_bar_duck_prepare_yaw_settle_start_sec
        if (
            self.limit_bar_duck_prepare_yaw_timeout_sec > 0.0
            and elapsed_sec >= self.limit_bar_duck_prepare_yaw_timeout_sec
        ):
            self._publish_step_override(
                self.duck_prepare_mode,
                self.duck_prepare_left_norm,
                self.duck_prepare_right_norm,
            )
            self._publish_feedback(
                f'限高杆趴下后 yaw 停稳超时: yaw_err={yaw_error_deg:.1f}deg, '
                f'timeout={self.limit_bar_duck_prepare_yaw_timeout_sec:.1f}s，'
                '继续锁定中心线并交给趴走修正。'
            )
            self.limit_bar_duck_prepare_yaw_settle_start_sec = None
            return False

        mode, left_norm, right_norm = self._limit_bar_duck_prepare_yaw_override(yaw_error)
        self._publish_step_override(mode, left_norm, right_norm)
        if (now - self.last_limit_bar_duck_prepare_yaw_log_sec) >= 0.30:
            self.last_limit_bar_duck_prepare_yaw_log_sec = now
            direction = 'ccw' if yaw_error > 0.0 else 'cw'
            self._publish_feedback(
                f'限高杆趴下后 yaw 停稳修正中: yaw_err={yaw_error_deg:.1f}deg, '
                f'direction={direction}, elapsed={elapsed_sec:.2f}/'
                f'{self.limit_bar_duck_prepare_yaw_timeout_sec:.2f}s, '
                f'override=[{mode},{left_norm:.3f},{right_norm:.3f}]。'
            )
        return True

    def _lateral_s_curve_control_progress(
        self,
    ) -> tuple[Optional[float], Optional[float]]:
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.limit_bar_lateral_shift_base_yaw is None
            or self.lateral_s_curve_control_start_x is None
            or self.lateral_s_curve_control_start_y is None
        ):
            return None, None
        dx = self.pose_x - self.lateral_s_curve_control_start_x
        dy = self.pose_y - self.lateral_s_curve_control_start_y
        forward_x, forward_y = self._odom_forward_unit(
            self.limit_bar_lateral_shift_base_yaw
        )
        right_x, right_y = self._odom_right_unit(
            self.limit_bar_lateral_shift_base_yaw
        )
        forward_m = dx * forward_x + dy * forward_y
        # LateralSCurvePlan uses left-positive route coordinates.
        lateral_m = -(dx * right_x + dy * right_y)
        return forward_m, lateral_m

    def _lateral_s_curve_control_walk_norms(
        self,
        base_left_norm: float,
        base_right_norm: float,
        yaw_error: float,
    ) -> tuple[float, float, float]:
        delta_norm = min(
            self.lateral_s_curve_control_max_delta_norm,
            self.lateral_s_curve_control_heading_gain_norm_per_rad
            * abs(yaw_error),
        )
        if yaw_error > 0.0:
            left_norm = self._clamp_norm(base_left_norm - delta_norm)
            right_norm = self._clamp_norm(base_right_norm + delta_norm)
        else:
            left_norm = self._clamp_norm(base_left_norm + delta_norm)
            right_norm = self._clamp_norm(base_right_norm - delta_norm)
        return left_norm, right_norm, delta_norm

    def _lateral_s_curve_completion_recovery_walk_norms(
        self,
        yaw_error: float,
    ) -> tuple[float, float, float]:
        delta_norm = min(
            self.lateral_s_curve_control_completion_recovery_yaw_max_delta_norm,
            self.lateral_s_curve_control_completion_recovery_yaw_gain_norm_per_rad
            * abs(yaw_error),
        )
        left_norm = self.lateral_s_curve_control_completion_recovery_left_norm
        right_norm = self.lateral_s_curve_control_completion_recovery_right_norm
        if yaw_error > 0.0:
            left_norm -= delta_norm
            right_norm += delta_norm
        else:
            left_norm += delta_norm
            right_norm -= delta_norm
        return (
            max(0.0, self._clamp_norm(left_norm)),
            max(0.0, self._clamp_norm(right_norm)),
            delta_norm,
        )

    def _complete_lateral_s_curve_control(
        self,
        now: float,
        *,
        completion_reason: str,
        forward_m: float,
        lateral_m: float,
        cross_track_error_m: float,
        yaw_error: float,
    ) -> None:
        plan = self.lateral_s_curve_control_plan
        elapsed_sec = max(0.0, now - self.lateral_s_curve_control_start_sec)
        self._publish_feedback(
            f'第二阶段 S 曲线实际结果: branch=hurdle, '
            f'route={plan.forward_m:.3f}m/{plan.lateral_m:+.3f}m, '
            f'actual={forward_m:.3f}m/{lateral_m:+.3f}m, '
            f'cross_err={cross_track_error_m:+.3f}m, '
            f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
            f'elapsed={elapsed_sec:.2f}s, completion={completion_reason}, '
            'control=actual_s_curve。'
        )

    def _run_hurdle_lateral_s_curve_control(self, now: float) -> None:
        plan = self.lateral_s_curve_control_plan
        if (
            not self.lateral_s_curve_control_active
            or plan is None
            or not plan.feasible
            or self.limit_bar_lateral_shift_next_action != 'hurdle_wait_jump'
        ):
            self._fail('高墙 S 曲线控制状态不完整，停止高墙流程。')
            return

        forward_m, lateral_m = self._lateral_s_curve_control_progress()
        if forward_m is None or lateral_m is None:
            self._fail('高墙 S 曲线控制无法计算路线坐标。')
            return

        clamped_forward_m = max(0.0, min(plan.forward_m, forward_m))
        current_sample = plan.sample(clamped_forward_m)
        endpoint_cross_error_m = plan.lateral_m - lateral_m
        cross_track_error_m = current_sample.lateral_m - lateral_m
        remaining_m = max(0.0, plan.forward_m - forward_m)
        forward_overshoot_m = max(0.0, forward_m - plan.forward_m)

        visual_completion_latched = (
            self.lateral_s_curve_control_completion_reason == 'visual_jump'
        )
        ignore_plan_overshoot_after_visual_completion = (
            self.hurdle_lateral_s_curve_visual_completion_ignore_plan_overshoot_enabled
            and visual_completion_latched
            and (
                self.hurdle_lateral_s_curve_visual_completion_max_plan_overshoot_m
                <= 0.0
                or forward_overshoot_m
                <= self.hurdle_lateral_s_curve_visual_completion_max_plan_overshoot_m
            )
        )
        if (
            not ignore_plan_overshoot_after_visual_completion
            and self.lateral_s_curve_control_max_forward_overshoot_m > 0.0
            and forward_overshoot_m
            > self.lateral_s_curve_control_max_forward_overshoot_m
        ):
            self._publish_step_override(0, 0.0, 0.0)
            self._fail(
                f'高墙 S 曲线前向过冲: overshoot={forward_overshoot_m:.3f}m > '
                f'{self.lateral_s_curve_control_max_forward_overshoot_m:.3f}m。'
            )
            return
        if (
            self.lateral_s_curve_control_max_abs_cross_track_m > 0.0
            and abs(cross_track_error_m)
            > self.lateral_s_curve_control_max_abs_cross_track_m
        ):
            self._publish_step_override(0, 0.0, 0.0)
            self._fail(
                f'高墙 S 曲线横向跟踪超限: cross_err={cross_track_error_m:+.3f}m > '
                f'{self.lateral_s_curve_control_max_abs_cross_track_m:.3f}m。'
            )
            return

        (
            visual_jump_ready,
            visual_jump_candidate,
            visual_jump_guard_reason,
        ) = self._hurdle_lateral_s_curve_visual_jump_status(
            now,
            remaining_m,
        )
        if (
            visual_jump_candidate
            and not visual_jump_ready
            and (
                now - self.last_hurdle_lateral_s_curve_visual_guard_log_sec
            )
            >= 0.50
        ):
            self.last_hurdle_lateral_s_curve_visual_guard_log_sec = now
            self._publish_feedback(
                f'高墙 S 曲线视觉近距离候选暂不采纳: '
                f'visual_dist='
                f'{self._distance_text(self.hurdle_visual_detection.distance_m)}, '
                f'forward={forward_m:.3f}/{plan.forward_m:.3f}m, '
                f'{visual_jump_guard_reason}，继续曲线控制。'
            )
        forward_ready = (
            remaining_m <= self.lateral_s_curve_control_forward_tolerance_m
        )
        lateral_ready = (
            abs(endpoint_cross_error_m)
            <= self.lateral_s_curve_control_lateral_tolerance_m
        )

        if visual_jump_ready and not lateral_ready:
            self._publish_step_override(0, 0.0, 0.0)
            self._fail(
                f'高墙 S 曲线已到视觉起跳线但横向误差超限: '
                f'endpoint_cross_err={endpoint_cross_error_m:+.3f}m > '
                f'{self.lateral_s_curve_control_lateral_tolerance_m:.3f}m。'
            )
            return

        if not self.lateral_s_curve_control_completion_reason:
            if visual_jump_ready and lateral_ready:
                self.lateral_s_curve_control_completion_reason = 'visual_jump'
            elif forward_ready and lateral_ready:
                self.lateral_s_curve_control_completion_reason = 'odometry'
            if self.lateral_s_curve_control_completion_reason:
                self.lateral_s_curve_control_arrival_sec = now
                self.lateral_s_curve_control_arrival_forward_m = forward_m
                self.lateral_s_curve_control_arrival_cross_error_m = (
                    endpoint_cross_error_m
                )
                self.lateral_s_curve_control_completion_max_forward_m = (
                    forward_m
                )

        if self.lateral_s_curve_control_completion_reason:
            target_yaw = self.limit_bar_lateral_shift_base_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            if self.lateral_s_curve_control_arrival_sec is None:
                self.lateral_s_curve_control_arrival_sec = now
            if self.lateral_s_curve_control_arrival_forward_m is None:
                self.lateral_s_curve_control_arrival_forward_m = forward_m
            if self.lateral_s_curve_control_arrival_cross_error_m is None:
                self.lateral_s_curve_control_arrival_cross_error_m = (
                    endpoint_cross_error_m
                )
            if self.lateral_s_curve_control_completion_max_forward_m is None:
                self.lateral_s_curve_control_completion_max_forward_m = forward_m
            else:
                self.lateral_s_curve_control_completion_max_forward_m = max(
                    self.lateral_s_curve_control_completion_max_forward_m,
                    forward_m,
                )

            arrival_settle_sec = max(
                0.0,
                now - self.lateral_s_curve_control_arrival_sec,
            )
            retreat_m = max(
                0.0,
                self.lateral_s_curve_control_completion_max_forward_m
                - forward_m,
            )
            lateral_drift_m = abs(
                endpoint_cross_error_m
                - self.lateral_s_curve_control_arrival_cross_error_m
            )
            arrival_settle_required_sec = getattr(
                self,
                'lateral_s_curve_control_arrival_settle_sec',
                0.0,
            )
            max_completion_retreat_m = getattr(
                self,
                'lateral_s_curve_control_completion_max_retreat_m',
                0.0,
            )
            max_completion_lateral_drift_m = getattr(
                self,
                'lateral_s_curve_control_completion_max_lateral_drift_m',
                0.0,
            )
            settled_after_arrival = (
                arrival_settle_sec
                >= arrival_settle_required_sec
            )
            recovery_enabled = getattr(
                self,
                'lateral_s_curve_control_completion_recovery_enabled',
                False,
            )
            recovery_active = getattr(
                self,
                'lateral_s_curve_control_completion_recovery_active',
                False,
            )
            if (
                settled_after_arrival
                and not recovery_active
                and max_completion_retreat_m > 0.0
                and retreat_m
                > max_completion_retreat_m
            ):
                recovery_max_abs_lateral_m = getattr(
                    self,
                    'lateral_s_curve_control_completion_recovery_max_abs_lateral_m',
                    0.0,
                )
                recovery_lateral_safe = (
                    recovery_max_abs_lateral_m <= 0.0
                    or abs(endpoint_cross_error_m)
                    <= recovery_max_abs_lateral_m
                )
                if recovery_enabled and recovery_lateral_safe:
                    recovery_forward_tolerance_m = getattr(
                        self,
                        'lateral_s_curve_control_completion_recovery_forward_tolerance_m',
                        0.0,
                    )
                    recovery_target_forward_m = max(
                        0.0,
                        self.lateral_s_curve_control_completion_max_forward_m
                        - recovery_forward_tolerance_m,
                    )
                    self.lateral_s_curve_control_completion_recovery_active = True
                    self.lateral_s_curve_control_completion_recovery_target_forward_m = (
                        recovery_target_forward_m
                    )
                    self.lateral_s_curve_control_completion_recovery_trigger = (
                        'retreat_limit'
                    )
                    self.lateral_s_curve_control_completion_since_sec = None
                    recovery_active = True
                    self._publish_feedback(
                        f'高墙 S 曲线收尾倒退超限，启动前向补偿: '
                        f'retreat={retreat_m:.3f}m > '
                        f'{max_completion_retreat_m:.3f}m, '
                        f'forward={forward_m:.3f}m, max_forward='
                        f'{self.lateral_s_curve_control_completion_max_forward_m:.3f}m, '
                        f'recovery_target={recovery_target_forward_m:.3f}m。'
                    )
                else:
                    self._publish_step_override(0, 0.0, 0.0)
                    unsafe_text = (
                        f', recovery_refused_cross_err='
                        f'{endpoint_cross_error_m:+.3f}m > '
                        f'{recovery_max_abs_lateral_m:.3f}m'
                        if recovery_enabled and not recovery_lateral_safe
                        else ''
                    )
                    self._fail(
                        f'高墙 S 曲线收尾倒退超限: retreat={retreat_m:.3f}m > '
                        f'{max_completion_retreat_m:.3f}m, '
                        f'forward={forward_m:.3f}m, max_forward='
                        f'{self.lateral_s_curve_control_completion_max_forward_m:.3f}m'
                        f'{unsafe_text}。'
                    )
                    return
            if (
                settled_after_arrival
                and max_completion_lateral_drift_m > 0.0
                and lateral_drift_m
                > max_completion_lateral_drift_m
            ):
                self._publish_step_override(0, 0.0, 0.0)
                self._fail(
                    f'高墙 S 曲线收尾横向漂移超限: '
                    f'drift={lateral_drift_m:.3f}m > '
                    f'{max_completion_lateral_drift_m:.3f}m, '
                    f'arrival_cross_err='
                    f'{self.lateral_s_curve_control_arrival_cross_error_m:+.3f}m, '
                    f'current_cross_err={endpoint_cross_error_m:+.3f}m。'
                )
                return

            recovery_target_forward_m = getattr(
                self,
                'lateral_s_curve_control_completion_recovery_target_forward_m',
                None,
            )
            recovery_yaw_tolerance_rad = getattr(
                self,
                'lateral_s_curve_control_completion_recovery_yaw_tolerance_rad',
                self.lateral_s_curve_control_yaw_tolerance_rad,
            )
            recovery_max_extra_forward_m = getattr(
                self,
                'lateral_s_curve_control_completion_recovery_max_extra_forward_m',
                0.0,
            )
            recovery_remaining_m = 0.0
            recovery_extra_forward_m = 0.0
            recovery_yaw_ready = False
            if recovery_active and recovery_target_forward_m is not None:
                recovery_remaining_m = max(
                    0.0,
                    recovery_target_forward_m - forward_m,
                )
                recovery_extra_forward_m = max(
                    0.0,
                    forward_m - recovery_target_forward_m,
                )
                recovery_yaw_ready = (
                    abs(yaw_error) <= recovery_yaw_tolerance_rad
                )

            if (
                recovery_active
                and recovery_target_forward_m is not None
                and (
                    recovery_remaining_m > 0.0
                    or (
                        not recovery_yaw_ready
                        and recovery_extra_forward_m
                        < recovery_max_extra_forward_m
                    )
                )
            ):
                self.lateral_s_curve_control_completion_since_sec = None
                left_norm, right_norm, _ = (
                    self._lateral_s_curve_completion_recovery_walk_norms(
                        yaw_error
                    )
                )
                self._publish_step_override(
                    self.limit_bar_lateral_shift_walk_mode,
                    left_norm,
                    right_norm,
                )
                label = 'recovery_forward'
            elif not settled_after_arrival:
                self.lateral_s_curve_control_completion_since_sec = None
                self._publish_step_override(0, 0.0, 0.0)
                label = 'arrival_settle'
            elif (
                recovery_active
                and recovery_yaw_ready
                and not self._limit_bar_lateral_yaw_rate_ready()
            ):
                self.lateral_s_curve_control_completion_since_sec = None
                self._publish_step_override(0, 0.0, 0.0)
                label = 'recovery_yaw_settle'
            elif (
                abs(yaw_error)
                > (
                    recovery_yaw_tolerance_rad
                    if recovery_active
                    else self.lateral_s_curve_control_yaw_tolerance_rad
                )
                or not self._limit_bar_lateral_yaw_rate_ready()
            ):
                self.lateral_s_curve_control_completion_since_sec = None
                mode, left_norm, right_norm = (
                    self._limit_bar_lateral_shift_turn_override(
                        yaw_error,
                        duck_align=True,
                        hurdle_s_curve=(
                            getattr(
                                self,
                                'hurdle_lateral_s_curve_final_align_independent_turn_enabled',
                                False,
                            )
                        ),
                    )
                )
                self._publish_step_override(mode, left_norm, right_norm)
                label = 'final_align'
            else:
                if self.lateral_s_curve_control_completion_since_sec is None:
                    self.lateral_s_curve_control_completion_since_sec = now
                stable_sec = (
                    now - self.lateral_s_curve_control_completion_since_sec
                )
                self._publish_step_override(0, 0.0, 0.0)
                label = 'completion_hold'
                if stable_sec >= self.lateral_s_curve_control_completion_hold_sec:
                    if not lateral_ready:
                        self._fail(
                            f'高墙 S 曲线收尾后横向误差超限: '
                            f'cross_err={endpoint_cross_error_m:+.3f}m > '
                            f'{self.lateral_s_curve_control_lateral_tolerance_m:.3f}m。'
                        )
                        return
                    completion_reason = (
                        self.lateral_s_curve_control_completion_reason
                    )
                    self._complete_lateral_s_curve_control(
                        now,
                        completion_reason=completion_reason,
                        forward_m=forward_m,
                        lateral_m=lateral_m,
                        cross_track_error_m=endpoint_cross_error_m,
                        yaw_error=yaw_error,
                    )
                    self._set_limit_bar_lateral_shift_phase('', now)
                    self.lateral_s_curve_control_active = False
                    self.lateral_s_curve_control_completion_recovery_active = False
                    self.duck_reference_yaw = self._normalize_angle(
                        self.limit_bar_lateral_shift_base_yaw
                    )
                    self.duck_reference_yaw_source = (
                        'hurdle_lateral_s_curve_base_yaw'
                    )
                    self.duck_reference_yaw_offset_rad = 0.0
                    visual_text = ''
                    if completion_reason == 'visual_jump':
                        visual_text = (
                            ', visual_dist='
                            f'{self._distance_text(self.hurdle_visual_detection.distance_m)}'
                            f' <= {self.hurdle_lateral_shift_visual_jump_distance_m:.3f}m'
                        )
                    self._publish_feedback(
                        f'高墙 S 曲线横向补偿完成: forward={forward_m:.3f}m, '
                        f'lateral={lateral_m:+.3f}m, '
                        f'cross_err={endpoint_cross_error_m:+.3f}m, '
                        f'yaw_err={math.degrees(yaw_error):+.1f}deg'
                        f'{visual_text}，进入起跳等待。'
                    )
                    self._enter_hurdle_wait_jump(
                        now,
                        self.limit_bar_lateral_shift_reason,
                    )
                    return

            if (now - self.last_limit_bar_lateral_shift_log_sec) >= 0.25:
                self.last_limit_bar_lateral_shift_log_sec = now
                self._publish_feedback(
                    f'高墙 S 曲线收尾中: phase={label}, '
                    f'forward={forward_m:.3f}/{plan.forward_m:.3f}m, '
                    f'lateral={lateral_m:+.3f}/{plan.lateral_m:+.3f}m, '
                    f'cross_err={endpoint_cross_error_m:+.3f}m, '
                    f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                    f'arrival_settle={arrival_settle_sec:.2f}/'
                    f'{arrival_settle_required_sec:.2f}s, '
                    f'retreat={retreat_m:.3f}m, lateral_drift={lateral_drift_m:.3f}m, '
                    f'recovery={recovery_active}, '
                    f'recovery_remaining={recovery_remaining_m:.3f}m, '
                    f'completion={self.lateral_s_curve_control_completion_reason}。'
                )
            return

        lookahead_forward_m = min(
            plan.forward_m,
            max(0.0, forward_m) + self.lateral_s_curve_control_lookahead_m,
        )
        lookahead_sample = plan.sample(lookahead_forward_m)
        heading_correction_rad = max(
            -self.lateral_s_curve_control_cross_track_max_heading_rad,
            min(
                self.lateral_s_curve_control_cross_track_max_heading_rad,
                self.lateral_s_curve_control_cross_track_gain_rad_per_m
                * cross_track_error_m,
            ),
        )
        target_relative_heading_rad = (
            lookahead_sample.heading_rad + heading_correction_rad
        )
        target_yaw = self._normalize_angle(
            self.limit_bar_lateral_shift_base_yaw
            + target_relative_heading_rad
        )
        yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)

        endpoint_correction = forward_ready and not lateral_ready
        if endpoint_correction:
            target_yaw = self._normalize_angle(
                self.limit_bar_lateral_shift_base_yaw
                + max(
                    -self.lateral_s_curve_control_cross_track_max_heading_rad,
                    min(
                        self.lateral_s_curve_control_cross_track_max_heading_rad,
                        self.lateral_s_curve_control_cross_track_gain_rad_per_m
                        * endpoint_cross_error_m,
                    ),
                )
            )
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)

        if abs(yaw_error) > self.lateral_s_curve_control_walk_yaw_gate_rad:
            mode, left_norm, right_norm = self._limit_bar_lateral_shift_turn_override(
                yaw_error
            )
            yaw_corr_norm = 0.0
            label = 'path_yaw_realign'
        else:
            slow = (
                endpoint_correction
                or remaining_m
                <= self.lateral_s_curve_control_slowdown_distance_m
            )
            base_left_norm = (
                self.lateral_s_curve_control_slow_left_norm
                if slow
                else self.lateral_s_curve_control_cruise_left_norm
            )
            base_right_norm = (
                self.lateral_s_curve_control_slow_right_norm
                if slow
                else self.lateral_s_curve_control_cruise_right_norm
            )
            left_norm, right_norm, yaw_corr_norm = (
                self._lateral_s_curve_control_walk_norms(
                    base_left_norm,
                    base_right_norm,
                    yaw_error,
                )
            )
            mode = self.limit_bar_lateral_shift_walk_mode
            label = 'endpoint_correction' if endpoint_correction else (
                'slow' if slow else 'cruise'
            )
        self._publish_step_override(mode, left_norm, right_norm)
        if (now - self.last_limit_bar_lateral_shift_log_sec) >= 0.25:
            self.last_limit_bar_lateral_shift_log_sec = now
            self._publish_feedback(
                f'高墙 S 曲线控制中: phase={label}, '
                f'forward={forward_m:.3f}/{plan.forward_m:.3f}m, '
                f'lateral={lateral_m:+.3f}/{current_sample.lateral_m:+.3f}m, '
                f'cross_err={cross_track_error_m:+.3f}m, '
                f'remaining={remaining_m:.3f}m, '
                f'target_heading={math.degrees(target_relative_heading_rad):+.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                f'yaw_corr={yaw_corr_norm:.3f}, '
                f'override=[{mode},{left_norm:.3f},{right_norm:.3f}]。'
            )

    def _run_upstairs_lateral_s_curve_control(self, now: float) -> None:
        plan = self.lateral_s_curve_control_plan
        if (
            not self.lateral_s_curve_control_active
            or plan is None
            or not plan.feasible
            or self.limit_bar_lateral_shift_next_action
            != 'upstairs_step_once'
        ):
            self._fail('上台阶 S 曲线控制状态不完整，停止上台阶流程。')
            return

        forward_m, lateral_m = self._lateral_s_curve_control_progress()
        if forward_m is None or lateral_m is None:
            self._fail('上台阶 S 曲线控制无法计算路线坐标。')
            return

        clamped_forward_m = max(0.0, min(plan.forward_m, forward_m))
        current_sample = plan.sample(clamped_forward_m)
        endpoint_cross_error_m = plan.lateral_m - lateral_m
        cross_track_error_m = current_sample.lateral_m - lateral_m
        remaining_m = max(0.0, plan.forward_m - forward_m)
        forward_overshoot_m = max(0.0, forward_m - plan.forward_m)
        completion_latched = bool(
            self.lateral_s_curve_control_completion_reason
        )

        max_forward_overshoot_m = getattr(
            self,
            'upstairs_lateral_s_curve_control_max_forward_overshoot_m',
            self.lateral_s_curve_control_max_forward_overshoot_m,
        )
        if (
            not completion_latched
            and max_forward_overshoot_m > 0.0
            and forward_overshoot_m > max_forward_overshoot_m
        ):
            self._publish_step_override(0, 0.0, 0.0)
            self._fail(
                f'上台阶 S 曲线前向过冲: '
                f'overshoot={forward_overshoot_m:.3f}m > '
                f'{max_forward_overshoot_m:.3f}m。'
            )
            return
        if (
            self.lateral_s_curve_control_max_abs_cross_track_m > 0.0
            and abs(cross_track_error_m)
            > self.lateral_s_curve_control_max_abs_cross_track_m
        ):
            self._publish_step_override(0, 0.0, 0.0)
            self._fail(
                f'上台阶 S 曲线横向跟踪超限: '
                f'cross_err={cross_track_error_m:+.3f}m > '
                f'{self.lateral_s_curve_control_max_abs_cross_track_m:.3f}m。'
            )
            return

        live_alignment = self._upstairs_live_tag_alignment(now)
        distance_candidate_ready, distance_reason = (
            self._upstairs_final_approach_ready(now, live_alignment)
        )
        distance_max_remaining_m = getattr(
            self,
            'upstairs_lateral_s_curve_control_distance_max_remaining_m',
            0.0,
        )
        distance_progress_ready = (
            distance_max_remaining_m <= 0.0
            or remaining_m <= distance_max_remaining_m
        )
        distance_ready = (
            distance_candidate_ready and distance_progress_ready
        )
        forward_ready = (
            remaining_m <= self.lateral_s_curve_control_forward_tolerance_m
        )
        lateral_ready = (
            abs(endpoint_cross_error_m)
            <= self.lateral_s_curve_control_lateral_tolerance_m
        )

        if distance_ready and not lateral_ready:
            self._publish_step_override(0, 0.0, 0.0)
            self._fail(
                f'上台阶 S 曲线已到动作线但横向误差超限: '
                f'endpoint_cross_err={endpoint_cross_error_m:+.3f}m > '
                f'{self.lateral_s_curve_control_lateral_tolerance_m:.3f}m, '
                f'distance=({distance_reason})。'
            )
            return

        if (distance_ready and lateral_ready) or completion_latched:
            if not self.lateral_s_curve_control_completion_reason:
                self.lateral_s_curve_control_completion_reason = (
                    'distance_confirmed'
                )
                self.lateral_s_curve_control_arrival_sec = now
                self.lateral_s_curve_control_arrival_forward_m = forward_m
                self.lateral_s_curve_control_arrival_cross_error_m = (
                    endpoint_cross_error_m
                )
                self.upstairs_final_align_latched_reason = distance_reason
            completion_distance_reason = (
                self.upstairs_final_align_latched_reason
                or distance_reason
            )
            target_yaw = self.limit_bar_lateral_shift_base_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            arrival_settle_sec = max(
                0.0,
                now - self.lateral_s_curve_control_arrival_sec,
            )
            if (
                arrival_settle_sec
                < self.lateral_s_curve_control_arrival_settle_sec
            ):
                self._reset_limit_bar_lateral_turn_guard()
                self._publish_step_override(0, 0.0, 0.0)
                label = 'arrival_settle'
            else:
                final_turn_ready = self._limit_bar_lateral_turn_ready(
                    now,
                    yaw_error,
                    self.upstairs_final_yaw_complete_gate_rad,
                )
                if not final_turn_ready:
                    if (
                        abs(yaw_error)
                        <= self.upstairs_final_yaw_complete_gate_rad
                    ):
                        self._publish_step_override(0, 0.0, 0.0)
                        label = 'final_yaw_hold'
                    else:
                        mode, left_norm, right_norm = (
                            self._upstairs_lateral_s_curve_final_turn_override(
                                yaw_error
                            )
                        )
                        self._publish_step_override(
                            mode,
                            left_norm,
                            right_norm,
                        )
                        label = 'final_align_zero_forward'
                    if (
                        now - self.last_limit_bar_lateral_shift_log_sec
                    ) >= 0.25:
                        self.last_limit_bar_lateral_shift_log_sec = now
                        self._publish_feedback(
                            f'上台阶 S 曲线收尾中: phase={label}, '
                            f'forward={forward_m:.3f}/{plan.forward_m:.3f}m, '
                            f'lateral={lateral_m:+.3f}/'
                            f'{plan.lateral_m:+.3f}m, '
                            f'cross_err={endpoint_cross_error_m:+.3f}m, '
                            f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                            f'arrival_settle={arrival_settle_sec:.2f}/'
                            f'{self.lateral_s_curve_control_arrival_settle_sec:.2f}s, '
                            f'distance=({completion_distance_reason})。'
                        )
                    return
                if not lateral_ready:
                    self._publish_step_override(0, 0.0, 0.0)
                    self._fail(
                        f'上台阶 S 曲线收尾后横向误差超限: '
                        f'cross_err={endpoint_cross_error_m:+.3f}m > '
                        f'{self.lateral_s_curve_control_lateral_tolerance_m:.3f}m。'
                    )
                    return
                elapsed_sec = max(
                    0.0,
                    now - self.lateral_s_curve_control_start_sec,
                )
                self._publish_step_override(0, 0.0, 0.0)
                self._publish_feedback(
                    f'第二阶段 S 曲线实际结果: branch=upstairs, '
                    f'route={plan.forward_m:.3f}m/'
                    f'{plan.lateral_m:+.3f}m, '
                    f'actual={forward_m:.3f}m/{lateral_m:+.3f}m, '
                    f'cross_err={endpoint_cross_error_m:+.3f}m, '
                    f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                    f'elapsed={elapsed_sec:.2f}s, '
                    'completion=distance_confirmed, '
                    'control=actual_s_curve。'
                )
                self._set_limit_bar_lateral_shift_phase('', now)
                self.lateral_s_curve_control_active = False
                self.duck_reference_yaw = self._normalize_angle(
                    self.limit_bar_lateral_shift_base_yaw
                )
                self.duck_reference_yaw_source = (
                    'upstairs_lateral_s_curve_base_yaw'
                )
                self.duck_reference_yaw_offset_rad = 0.0
                self._publish_feedback(
                    f'上台阶 S 曲线横向补偿完成: '
                    f'forward={forward_m:.3f}/{plan.forward_m:.3f}m, '
                    f'lateral={lateral_m:+.3f}/{plan.lateral_m:+.3f}m, '
                    f'cross_err={endpoint_cross_error_m:+.3f}m, '
                    f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                    f'distance=({completion_distance_reason})，'
                    '直接进入 step_once。'
                )
                self._finish_upstairs_done(
                    now,
                    self.limit_bar_lateral_shift_reason,
                    final_along_m=forward_m,
                    raw_final_along_m=forward_m,
                    final_remaining_m=remaining_m,
                )
                return

            if (now - self.last_limit_bar_lateral_shift_log_sec) >= 0.25:
                self.last_limit_bar_lateral_shift_log_sec = now
                self._publish_feedback(
                    f'上台阶 S 曲线收尾中: phase={label}, '
                    f'forward={forward_m:.3f}/{plan.forward_m:.3f}m, '
                    f'lateral={lateral_m:+.3f}/{plan.lateral_m:+.3f}m, '
                    f'cross_err={endpoint_cross_error_m:+.3f}m, '
                    f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                    f'arrival_settle={arrival_settle_sec:.2f}/'
                    f'{self.lateral_s_curve_control_arrival_settle_sec:.2f}s, '
                    f'distance=({completion_distance_reason})。'
                )
            return

        if forward_ready and lateral_ready:
            extra_forward_m = forward_overshoot_m
            if (
                self.upstairs_final_confirm_creep_max_extra_m <= 0.0
                or extra_forward_m
                >= self.upstairs_final_confirm_creep_max_extra_m
            ):
                self._publish_step_override(0, 0.0, 0.0)
                label = 'distance_wait'
            else:
                target_yaw = self.limit_bar_lateral_shift_base_yaw
                yaw_error = self._normalize_angle(
                    target_yaw - self.pose_yaw
                )
                if (
                    abs(yaw_error)
                    > self.upstairs_lateral_s_curve_control_precreep_yaw_gate_rad
                ):
                    mode, left_norm, right_norm = (
                        self._upstairs_lateral_s_curve_final_turn_override(
                            yaw_error
                        )
                    )
                    label = 'distance_confirm_yaw_align_zero_forward'
                elif not self._limit_bar_lateral_yaw_rate_ready():
                    mode = 0
                    left_norm = 0.0
                    right_norm = 0.0
                    label = 'distance_confirm_yaw_settle'
                else:
                    left_norm, right_norm, _ = (
                        self._lateral_s_curve_control_walk_norms(
                            self.upstairs_final_confirm_creep_left_norm,
                            self.upstairs_final_confirm_creep_right_norm,
                            yaw_error,
                        )
                    )
                    left_norm, right_norm, _ = (
                        self._upstairs_final_forward_tag_yaw_corrected_norms(
                            left_norm,
                            right_norm,
                            live_alignment,
                        )
                    )
                    mode = self.limit_bar_lateral_shift_walk_mode
                    label = 'distance_confirm_creep'
                self._publish_step_override(mode, left_norm, right_norm)
            if (now - self.last_limit_bar_lateral_shift_log_sec) >= 0.30:
                self.last_limit_bar_lateral_shift_log_sec = now
                self._publish_feedback(
                    f'上台阶 S 曲线里程已到但距离未确认: '
                    f'forward={forward_m:.3f}/{plan.forward_m:.3f}m, '
                    f'extra_forward={extra_forward_m:.3f}/'
                    f'{self.upstairs_final_confirm_creep_max_extra_m:.3f}m, '
                    f'cross_err={endpoint_cross_error_m:+.3f}m, '
                    f'distance=({distance_reason}), action={label}。'
                )
            return

        lookahead_forward_m = min(
            plan.forward_m,
            max(0.0, forward_m) + self.lateral_s_curve_control_lookahead_m,
        )
        lookahead_sample = plan.sample(lookahead_forward_m)
        heading_correction_rad = max(
            -self.lateral_s_curve_control_cross_track_max_heading_rad,
            min(
                self.lateral_s_curve_control_cross_track_max_heading_rad,
                self.lateral_s_curve_control_cross_track_gain_rad_per_m
                * cross_track_error_m,
            ),
        )
        target_relative_heading_rad = (
            lookahead_sample.heading_rad + heading_correction_rad
        )
        target_yaw = self._normalize_angle(
            self.limit_bar_lateral_shift_base_yaw
            + target_relative_heading_rad
        )
        yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)

        endpoint_correction = forward_ready and not lateral_ready
        if endpoint_correction:
            target_relative_heading_rad = max(
                -self.lateral_s_curve_control_cross_track_max_heading_rad,
                min(
                    self.lateral_s_curve_control_cross_track_max_heading_rad,
                    self.lateral_s_curve_control_cross_track_gain_rad_per_m
                    * endpoint_cross_error_m,
                ),
            )
            target_yaw = self._normalize_angle(
                self.limit_bar_lateral_shift_base_yaw
                + target_relative_heading_rad
            )
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)

        if abs(yaw_error) > self.lateral_s_curve_control_walk_yaw_gate_rad:
            mode, left_norm, right_norm = (
                self._limit_bar_lateral_shift_turn_override(yaw_error)
            )
            yaw_corr_norm = 0.0
            label = 'path_yaw_realign'
        else:
            slow = (
                endpoint_correction
                or remaining_m
                <= self.upstairs_lateral_s_curve_control_slowdown_distance_m
            )
            base_left_norm = (
                self.upstairs_lateral_s_curve_control_slow_left_norm
                if slow
                else self.upstairs_lateral_s_curve_control_cruise_left_norm
            )
            base_right_norm = (
                self.upstairs_lateral_s_curve_control_slow_right_norm
                if slow
                else self.upstairs_lateral_s_curve_control_cruise_right_norm
            )
            left_norm, right_norm, yaw_corr_norm = (
                self._lateral_s_curve_control_walk_norms(
                    base_left_norm,
                    base_right_norm,
                    yaw_error,
                )
            )
            mode = self.limit_bar_lateral_shift_walk_mode
            label = (
                'endpoint_correction'
                if endpoint_correction
                else 'slow'
                if slow
                else 'cruise'
            )
        self._publish_step_override(mode, left_norm, right_norm)
        if (now - self.last_limit_bar_lateral_shift_log_sec) >= 0.25:
            self.last_limit_bar_lateral_shift_log_sec = now
            distance_progress_text = (
                f', distance_candidate_ignored_remaining='
                f'{remaining_m:.3f}m>{distance_max_remaining_m:.3f}m'
                if distance_candidate_ready and not distance_progress_ready
                else ''
            )
            self._publish_feedback(
                f'上台阶 S 曲线控制中: phase={label}, '
                f'forward={forward_m:.3f}/{plan.forward_m:.3f}m, '
                f'lateral={lateral_m:+.3f}/'
                f'{current_sample.lateral_m:+.3f}m, '
                f'cross_err={cross_track_error_m:+.3f}m, '
                f'remaining={remaining_m:.3f}m, '
                f'target_heading='
                f'{math.degrees(target_relative_heading_rad):+.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                f'yaw_corr={yaw_corr_norm:.3f}'
                f'{distance_progress_text}, '
                f'override=[{mode},{left_norm:.3f},{right_norm:.3f}]。'
            )

    def _run_limit_bar_lateral_s_curve_control(self, now: float) -> None:
        plan = self.lateral_s_curve_control_plan
        if (
            not self.lateral_s_curve_control_active
            or plan is None
            or not plan.feasible
            or self.limit_bar_lateral_shift_next_action
            != 'limit_bar_prepare'
        ):
            self._fail('限高杆 S 曲线控制状态不完整，停止限高杆流程。')
            return

        forward_m, lateral_m = self._lateral_s_curve_control_progress()
        if forward_m is None or lateral_m is None:
            self._fail('限高杆 S 曲线控制无法计算路线坐标。')
            return

        clamped_forward_m = max(0.0, min(plan.forward_m, forward_m))
        current_sample = plan.sample(clamped_forward_m)
        endpoint_cross_error_m = plan.lateral_m - lateral_m
        cross_track_error_m = current_sample.lateral_m - lateral_m
        remaining_m = max(0.0, plan.forward_m - forward_m)
        forward_overshoot_m = max(0.0, forward_m - plan.forward_m)
        estimated_tag_distance_m = max(
            0.0,
            self.limit_bar_lateral_shift_center_z_m - forward_m,
        )
        fail_open_enabled = getattr(
            self,
            'limit_bar_lateral_s_curve_fail_open_enabled',
            False,
        )

        if (
            self.limit_bar_lateral_s_curve_control_max_forward_overshoot_m
            > 0.0
            and forward_overshoot_m
            > self.limit_bar_lateral_s_curve_control_max_forward_overshoot_m
        ):
            self._publish_step_override(0, 0.0, 0.0)
            if not fail_open_enabled:
                self._fail(
                    f'限高杆 S 曲线前向过冲: '
                    f'overshoot={forward_overshoot_m:.3f}m > '
                    f'{self.limit_bar_lateral_s_curve_control_max_forward_overshoot_m:.3f}m, '
                    f'est_tag_dist={estimated_tag_distance_m:.3f}m。'
                )
                return
            if not self.lateral_s_curve_control_completion_reason:
                self._latch_limit_bar_lateral_s_curve_completion(
                    now,
                    forward_m,
                    endpoint_cross_error_m,
                    'fail_open_forward_overshoot',
                )
                self._publish_feedback(
                    f'限高杆 S 曲线前向过冲保护已旁路: '
                    f'overshoot={forward_overshoot_m:.3f}m > '
                    f'{self.limit_bar_lateral_s_curve_control_max_forward_overshoot_m:.3f}m, '
                    f'est_tag_dist={estimated_tag_distance_m:.3f}m；'
                    '已停止前进并锁存完成，不进入 FAILED。'
                )
        if (
            self.limit_bar_lateral_s_curve_control_max_abs_cross_track_m
            > 0.0
            and abs(cross_track_error_m)
            > self.limit_bar_lateral_s_curve_control_max_abs_cross_track_m
        ):
            if not fail_open_enabled:
                self._publish_step_override(0, 0.0, 0.0)
                self._fail(
                    f'限高杆 S 曲线横向跟踪超限: '
                    f'cross_err={cross_track_error_m:+.3f}m > '
                    f'{self.limit_bar_lateral_s_curve_control_max_abs_cross_track_m:.3f}m。'
                )
                return
            if not self.limit_bar_lateral_s_curve_fail_open_cross_track_logged:
                self.limit_bar_lateral_s_curve_fail_open_cross_track_logged = True
                self._publish_feedback(
                    f'限高杆 S 曲线横向跟踪保护已旁路: '
                    f'cross_err={cross_track_error_m:+.3f}m > '
                    f'{self.limit_bar_lateral_s_curve_control_max_abs_cross_track_m:.3f}m；'
                    '继续控制，前向到达或超时时将锁存完成。'
                )

        forward_ready = (
            remaining_m
            <= self.limit_bar_lateral_s_curve_control_forward_tolerance_m
        )
        lateral_ready = (
            abs(endpoint_cross_error_m)
            <= self.limit_bar_lateral_s_curve_control_lateral_tolerance_m
        )
        completion_lateral_tolerance_m = max(
            self.limit_bar_lateral_s_curve_control_lateral_tolerance_m,
            self.limit_bar_lateral_s_curve_control_final_abs_lateral_tolerance_m,
        )
        completion_lateral_ready = (
            abs(endpoint_cross_error_m) <= completion_lateral_tolerance_m
        )
        if (
            not self.lateral_s_curve_control_completion_reason
            and forward_ready
            and completion_lateral_ready
        ):
            self._latch_limit_bar_lateral_s_curve_completion(
                now,
                forward_m,
                endpoint_cross_error_m,
                'odometry',
            )
            if not lateral_ready:
                self._publish_feedback(
                    f'限高杆 S 曲线前向已到，按收尾横向保护锁存完成: '
                    f'cross_err={endpoint_cross_error_m:+.3f}m <= '
                    f'{completion_lateral_tolerance_m:.3f}m；'
                    '停止前进并进入收尾保护，不再执行 endpoint_correction。'
                )

        if (
            not self.lateral_s_curve_control_completion_reason
            and not self.limit_bar_lateral_s_curve_pre_stop_completed
            and not forward_ready
            and self.limit_bar_lateral_s_curve_control_pre_stop_distance_m > 0.0
            and remaining_m
            <= self.limit_bar_lateral_s_curve_control_pre_stop_distance_m
        ):
            if self.limit_bar_lateral_s_curve_pre_stop_start_sec is None:
                self.limit_bar_lateral_s_curve_pre_stop_start_sec = now
                self._publish_feedback(
                    f'限高杆 S 曲线到达前预停开始: '
                    f'remaining={remaining_m:.3f}m <= '
                    f'{self.limit_bar_lateral_s_curve_control_pre_stop_distance_m:.3f}m, '
                    f'hold={self.limit_bar_lateral_s_curve_control_pre_stop_hold_sec:.2f}s；'
                    '先释放步态惯性，再继续判断到达。'
                )
            pre_stop_elapsed_sec = max(
                0.0,
                now - self.limit_bar_lateral_s_curve_pre_stop_start_sec,
            )
            if (
                pre_stop_elapsed_sec
                < self.limit_bar_lateral_s_curve_control_pre_stop_hold_sec
            ):
                self._publish_step_override(0, 0.0, 0.0)
                return
            self.limit_bar_lateral_s_curve_pre_stop_start_sec = None
            self.limit_bar_lateral_s_curve_pre_stop_completed = True
            self._publish_feedback(
                f'限高杆 S 曲线到达前预停完成: '
                f'remaining={remaining_m:.3f}m, '
                f'held={pre_stop_elapsed_sec:.2f}s；恢复低速末段控制。'
            )

        if self.lateral_s_curve_control_completion_reason:
            target_yaw = self.limit_bar_lateral_shift_base_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            arrival_sec = self.lateral_s_curve_control_arrival_sec
            if arrival_sec is None:
                arrival_sec = now
                self.lateral_s_curve_control_arrival_sec = now
            arrival_settle_sec = max(0.0, now - arrival_sec)
            arrival_cross_error_m = (
                endpoint_cross_error_m
                if self.lateral_s_curve_control_arrival_cross_error_m is None
                else self.lateral_s_curve_control_arrival_cross_error_m
            )
            final_lateral_drift_m = abs(
                endpoint_cross_error_m - arrival_cross_error_m
            )
            final_abs_lateral_ready = (
                abs(endpoint_cross_error_m)
                <= self.limit_bar_lateral_s_curve_control_final_abs_lateral_tolerance_m
            )
            final_lateral_drift_ready = (
                final_lateral_drift_m
                <= self.limit_bar_lateral_s_curve_control_final_max_lateral_drift_m
            )
            final_lateral_ready = (
                final_abs_lateral_ready and final_lateral_drift_ready
            )
            if (
                arrival_settle_sec
                < self.limit_bar_lateral_s_curve_control_arrival_settle_sec
            ):
                self._reset_limit_bar_lateral_turn_guard()
                self.lateral_s_curve_control_completion_since_sec = None
                self._publish_step_override(0, 0.0, 0.0)
                label = 'arrival_settle'
            else:
                final_turn_ready = (
                    fail_open_enabled
                    or self._limit_bar_lateral_turn_ready(
                        now,
                        yaw_error,
                        self.limit_bar_lateral_shift_duck_entry_yaw_gate_rad,
                    )
                )
                if not final_turn_ready:
                    self.limit_bar_lateral_s_curve_final_lateral_violation_since_sec = None
                    self.lateral_s_curve_control_completion_since_sec = None
                    if (
                        abs(yaw_error)
                        <= self.limit_bar_lateral_shift_duck_entry_yaw_gate_rad
                    ):
                        self._publish_step_override(0, 0.0, 0.0)
                        label = 'final_yaw_settle'
                    else:
                        mode, left_norm, right_norm = (
                            self._limit_bar_lateral_s_curve_stationary_turn_override(
                                yaw_error
                            )
                        )
                        self._publish_step_override(
                            mode,
                            left_norm,
                            right_norm,
                        )
                        label = 'final_align_zero_forward'
                else:
                    if not final_lateral_ready and not fail_open_enabled:
                        self.lateral_s_curve_control_completion_since_sec = None
                        self._publish_step_override(0, 0.0, 0.0)
                        if (
                            self.limit_bar_lateral_s_curve_final_lateral_violation_since_sec
                            is None
                        ):
                            self.limit_bar_lateral_s_curve_final_lateral_violation_since_sec = now
                        violation_sec = max(
                            0.0,
                            now
                            - self.limit_bar_lateral_s_curve_final_lateral_violation_since_sec,
                        )
                        label = 'final_lateral_guard'
                        if (
                            violation_sec
                            >= self.limit_bar_lateral_s_curve_control_final_lateral_violation_hold_sec
                        ):
                            self._fail(
                                f'限高杆 S 曲线收尾横向误差持续超限: '
                                f'cross_err={endpoint_cross_error_m:+.3f}m, '
                                f'arrival_cross_err={arrival_cross_error_m:+.3f}m, '
                                f'lateral_drift={final_lateral_drift_m:.3f}m, '
                                f'abs_limit='
                                f'{self.limit_bar_lateral_s_curve_control_final_abs_lateral_tolerance_m:.3f}m, '
                                f'drift_limit='
                                f'{self.limit_bar_lateral_s_curve_control_final_max_lateral_drift_m:.3f}m, '
                                f'held={violation_sec:.2f}s。'
                            )
                            return
                    else:
                        self.limit_bar_lateral_s_curve_final_lateral_violation_since_sec = None
                        if (
                            fail_open_enabled
                            and not final_lateral_ready
                            and not self.limit_bar_lateral_s_curve_fail_open_final_lateral_logged
                        ):
                            self.limit_bar_lateral_s_curve_fail_open_final_lateral_logged = True
                            self._publish_feedback(
                                f'限高杆 S 曲线收尾横向保护已旁路: '
                                f'cross_err={endpoint_cross_error_m:+.3f}m, '
                                f'lateral_drift={final_lateral_drift_m:.3f}m；'
                                '保持零速并继续完成流程，不进入 FAILED。'
                            )
                        if self.lateral_s_curve_control_completion_since_sec is None:
                            self.lateral_s_curve_control_completion_since_sec = now
                        stable_sec = (
                            now
                            - self.lateral_s_curve_control_completion_since_sec
                        )
                        self._publish_step_override(0, 0.0, 0.0)
                        label = (
                            'completion_hold_fail_open'
                            if fail_open_enabled
                            else 'completion_hold'
                        )
                        if (
                            stable_sec
                            >= self.limit_bar_lateral_shift_duck_entry_hold_sec
                        ):
                            elapsed_sec = max(
                                0.0,
                                now - self.lateral_s_curve_control_start_sec,
                            )
                            self.lateral_s_curve_control_active = False
                            self.limit_bar_lateral_s_curve_control_completed = True
                            self.duck_reference_yaw = self._normalize_angle(
                                self.limit_bar_lateral_shift_base_yaw
                            )
                            self.duck_reference_yaw_source = (
                                'limit_bar_lateral_s_curve_base_yaw'
                            )
                            self.duck_reference_yaw_offset_rad = 0.0
                            self._publish_feedback(
                                f'第二阶段 S 曲线实际结果: branch=limit_bar, '
                                f'route={plan.forward_m:.3f}m/'
                                f'{plan.lateral_m:+.3f}m, '
                                f'actual={forward_m:.3f}m/{lateral_m:+.3f}m, '
                                f'cross_err={endpoint_cross_error_m:+.3f}m, '
                                f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                                f'est_tag_dist={estimated_tag_distance_m:.3f}m, '
                                f'elapsed={elapsed_sec:.2f}s, '
                                f'completion={self.lateral_s_curve_control_completion_reason}, '
                                'control=actual_s_curve。'
                            )
                            if self._limit_bar_pre_duck_global_yaw_align_required():
                                pre_duck_target_yaw = (
                                    self._limit_bar_pre_duck_target_yaw()
                                )
                                pre_duck_yaw_source = (
                                    self._limit_bar_pre_duck_yaw_source()
                                )
                                self._set_limit_bar_lateral_shift_phase(
                                    'pre_duck_global_yaw_align',
                                    now,
                                )
                                self.limit_bar_pre_duck_global_yaw_align_start_sec = now
                                self.limit_bar_pre_duck_global_yaw_stable_since_sec = None
                                self._publish_feedback(
                                    f'限高杆 S 曲线横向补偿完成: '
                                    f'forward={forward_m:.3f}/{plan.forward_m:.3f}m, '
                                    f'lateral={lateral_m:+.3f}/'
                                    f'{plan.lateral_m:+.3f}m, '
                                    f'est_tag_dist={estimated_tag_distance_m:.3f}m，'
                                    f'保持站立并开始趴下前 yaw 对齐 '
                                    f'{math.degrees(pre_duck_target_yaw):.1f}deg, '
                                    f'source={pre_duck_yaw_source}。'
                                )
                                return
                            self._set_limit_bar_lateral_shift_phase('', now)
                            self._publish_feedback(
                                f'限高杆 S 曲线横向补偿完成: '
                                f'est_tag_dist={estimated_tag_distance_m:.3f}m，'
                                '进入趴下流程。'
                            )
                            self._enter_limit_bar_prepare(
                                now,
                                self.limit_bar_lateral_shift_reason,
                                self._duck_reference_yaw_text(),
                            )
                            return

            if (now - self.last_limit_bar_lateral_shift_log_sec) >= 0.25:
                self.last_limit_bar_lateral_shift_log_sec = now
                stable_sec = (
                    0.0
                    if self.lateral_s_curve_control_completion_since_sec is None
                    else now
                    - self.lateral_s_curve_control_completion_since_sec
                )
                self._publish_feedback(
                    f'限高杆 S 曲线收尾中: phase={label}, '
                    f'forward={forward_m:.3f}/{plan.forward_m:.3f}m, '
                    f'lateral={lateral_m:+.3f}/{plan.lateral_m:+.3f}m, '
                    f'cross_err={endpoint_cross_error_m:+.3f}m, '
                    f'arrival_cross_err={arrival_cross_error_m:+.3f}m, '
                    f'lateral_drift={final_lateral_drift_m:.3f}m, '
                    f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                    f'est_tag_dist={estimated_tag_distance_m:.3f}m, '
                    f'arrival_settle={arrival_settle_sec:.2f}/'
                    f'{self.limit_bar_lateral_s_curve_control_arrival_settle_sec:.2f}s, '
                    f'hold={stable_sec:.2f}/'
                    f'{self.limit_bar_lateral_shift_duck_entry_hold_sec:.2f}s。'
                )
            return

        lookahead_forward_m = min(
            plan.forward_m,
            max(0.0, forward_m) + self.lateral_s_curve_control_lookahead_m,
        )
        lookahead_sample = plan.sample(lookahead_forward_m)
        heading_correction_rad = max(
            -self.lateral_s_curve_control_cross_track_max_heading_rad,
            min(
                self.lateral_s_curve_control_cross_track_max_heading_rad,
                self.lateral_s_curve_control_cross_track_gain_rad_per_m
                * cross_track_error_m,
            ),
        )
        target_relative_heading_rad = (
            lookahead_sample.heading_rad + heading_correction_rad
        )
        target_yaw = self._normalize_angle(
            self.limit_bar_lateral_shift_base_yaw
            + target_relative_heading_rad
        )
        yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)

        endpoint_correction = forward_ready and not completion_lateral_ready
        if endpoint_correction:
            target_relative_heading_rad = max(
                -self.lateral_s_curve_control_cross_track_max_heading_rad,
                min(
                    self.lateral_s_curve_control_cross_track_max_heading_rad,
                    self.lateral_s_curve_control_cross_track_gain_rad_per_m
                    * endpoint_cross_error_m,
                ),
            )
            target_yaw = self._normalize_angle(
                self.limit_bar_lateral_shift_base_yaw
                + target_relative_heading_rad
            )
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)

        if abs(yaw_error) > self.lateral_s_curve_control_walk_yaw_gate_rad:
            mode, left_norm, right_norm = (
                self._limit_bar_lateral_s_curve_stationary_turn_override(
                    yaw_error
                )
            )
            yaw_corr_norm = 0.0
            label = 'path_yaw_realign_zero_forward'
        else:
            slow = (
                endpoint_correction
                or remaining_m
                <= self.limit_bar_lateral_s_curve_control_slowdown_distance_m
            )
            base_left_norm = (
                self.limit_bar_lateral_s_curve_control_slow_left_norm
                if slow
                else self.limit_bar_lateral_s_curve_control_cruise_left_norm
            )
            base_right_norm = (
                self.limit_bar_lateral_s_curve_control_slow_right_norm
                if slow
                else self.limit_bar_lateral_s_curve_control_cruise_right_norm
            )
            left_norm, right_norm, yaw_corr_norm = (
                self._lateral_s_curve_control_walk_norms(
                    base_left_norm,
                    base_right_norm,
                    yaw_error,
                )
            )
            mode = self.limit_bar_lateral_shift_walk_mode
            label = 'endpoint_correction' if endpoint_correction else (
                'slow' if slow else 'cruise'
            )
        self._publish_step_override(mode, left_norm, right_norm)
        if (now - self.last_limit_bar_lateral_shift_log_sec) >= 0.25:
            self.last_limit_bar_lateral_shift_log_sec = now
            self._publish_feedback(
                f'限高杆 S 曲线控制中: phase={label}, '
                f'forward={forward_m:.3f}/{plan.forward_m:.3f}m, '
                f'lateral={lateral_m:+.3f}/'
                f'{current_sample.lateral_m:+.3f}m, '
                f'cross_err={cross_track_error_m:+.3f}m, '
                f'remaining={remaining_m:.3f}m, '
                f'est_tag_dist={estimated_tag_distance_m:.3f}m, '
                f'target_heading='
                f'{math.degrees(target_relative_heading_rad):+.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                f'yaw_corr={yaw_corr_norm:.3f}, '
                f'override=[{mode},{left_norm:.3f},{right_norm:.3f}]。'
            )

    def _run_limit_bar_lateral_shift(self, now: float) -> None:
        shift_label = self._lateral_shift_label()
        if (
            self.limit_bar_lateral_shift_base_yaw is None
            or self.limit_bar_lateral_shift_side_yaw is None
        ):
            self._fail(f'{shift_label}横向补偿状态不完整，停止{shift_label}流程。')
            return
        if not self._pose_is_fresh(now):
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_feedback(f'{shift_label}横向补偿等待新鲜 Odometry。')
            return

        phase = self.limit_bar_lateral_shift_phase
        if phase == 's_curve':
            if (
                self.limit_bar_lateral_shift_next_action
                == 'upstairs_step_once'
            ):
                self._run_upstairs_lateral_s_curve_control(now)
            elif (
                self.limit_bar_lateral_shift_next_action
                == 'limit_bar_prepare'
            ):
                self._run_limit_bar_lateral_s_curve_control(now)
            else:
                self._run_hurdle_lateral_s_curve_control(now)
            if (
                self.limit_bar_lateral_shift_phase == phase
                and self._fail_limit_bar_lateral_phase_timeout(
                    now,
                    phase,
                    shift_label,
                )
            ):
                return
            return
        target_yaw = self.limit_bar_lateral_shift_base_yaw
        yaw_error = 0.0
        along_m: Optional[float] = None
        raw_along_m: Optional[float] = None
        remaining_m = 0.0
        mode = self.limit_bar_lateral_shift_walk_mode
        left_norm = 0.0
        right_norm = 0.0
        label = 'idle'
        yaw_corr_norm = 0.0
        live_alignment: Optional[tuple[float, float, float, float, float]] = None
        live_text = ''
        upstairs_distance_ready = False
        upstairs_distance_reason = 'not_checked'
        upstairs_distance_text = ''

        if phase == 'align_out':
            target_yaw = self.limit_bar_lateral_shift_side_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            if self._limit_bar_lateral_turn_ready(
                now,
                yaw_error,
                self.limit_bar_tag_align_tolerance_rad,
            ):
                self._set_limit_bar_lateral_shift_phase('shift', now)
                self.limit_bar_lateral_shift_segment_start_x = self.pose_x
                self.limit_bar_lateral_shift_segment_start_y = self.pose_y
                self._publish_step_override(0, 0.0, 0.0)
                self._publish_feedback(
                    f'{shift_label}横向补偿: 已转到斜向 yaw={math.degrees(target_yaw):.1f}deg，'
                    f'开始斜向移动 shift={self.limit_bar_lateral_shift_distance_m:.3f}m；'
                    '锁定初始 tag 计划，不再用旋转后的最新 tag 重算方向。'
                )
                return
            if self._fail_limit_bar_lateral_phase_timeout(
                now,
                phase,
                shift_label,
                yaw_error=yaw_error,
                tolerance_rad=self.limit_bar_tag_align_tolerance_rad,
            ):
                return
            mode, left_norm, right_norm = self._limit_bar_lateral_shift_turn_override(yaw_error)
            label = 'align_out'

        elif phase == 'shift':
            target_yaw = self.limit_bar_lateral_shift_side_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            along_m, raw_along_m = self._limit_bar_lateral_shift_progress(target_yaw)
            if along_m is None:
                self._fail(f'{shift_label}横向补偿无法计算斜向移动里程。')
                return
            remaining_m = max(0.0, self.limit_bar_lateral_shift_distance_m - along_m)
            arrival_tolerance_m = self._lateral_shift_arrival_tolerance_m()
            if remaining_m <= arrival_tolerance_m:
                self._set_limit_bar_lateral_shift_phase('align_back', now)
                self._publish_step_override(0, 0.0, 0.0)
                self._publish_feedback(
                    f'{shift_label}横向补偿: 斜向移动完成 along={along_m:.3f}m'
                    f'(raw={raw_along_m:.3f}m), remaining={remaining_m:.3f}m '
                    f'<= arrival_tolerance={arrival_tolerance_m:.3f}m，开始转回基准 yaw='
                    f'{math.degrees(self.limit_bar_lateral_shift_base_yaw):.1f}deg。'
                )
                return
            if self._fail_limit_bar_lateral_phase_timeout(
                now,
                phase,
                shift_label,
            ):
                return
            mode = self.limit_bar_lateral_shift_walk_mode
            left_norm, right_norm, yaw_corr_norm = (
                self._limit_bar_lateral_shift_walk_yaw_corrected_norms(
                    self.limit_bar_lateral_shift_walk_left_norm,
                    self.limit_bar_lateral_shift_walk_right_norm,
                    yaw_error,
                )
            )
            label = 'shift'

        elif phase == 'align_back':
            target_yaw = self.limit_bar_lateral_shift_base_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            if self._limit_bar_lateral_turn_ready(
                now,
                yaw_error,
                self.limit_bar_tag_align_tolerance_rad,
            ):
                live_alignment = self._upstairs_live_tag_alignment(now)
                if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
                    live_text = self._upstairs_live_tag_alignment_text(live_alignment)
                if self._upstairs_live_tag_needs_replan(live_alignment):
                    if (
                        self.upstairs_lateral_shift_live_replan_count
                        < self.upstairs_lateral_shift_live_check_max_replans
                    ):
                        _, _, live_center_x, vision_yaw_corr, target_yaw_if_live = live_alignment
                        old_base_yaw = self.limit_bar_lateral_shift_base_yaw
                        self.upstairs_lateral_shift_live_replan_count += 1
                        self.limit_bar_lateral_shift_base_yaw = target_yaw_if_live
                        self.duck_reference_yaw = target_yaw_if_live
                        self.duck_reference_yaw_offset_rad = vision_yaw_corr
                        self.limit_bar_lateral_shift_duck_align_since_sec = None
                        self._publish_feedback(
                            f'上台阶横向补偿 live tag 复核未通过，仅重锁 yaw，不追加横移: '
                            f'{self._upstairs_live_tag_alignment_text(live_alignment)}, '
                            f'base_yaw={math.degrees(old_base_yaw):.1f}->'
                            f'{math.degrees(target_yaw_if_live):.1f}deg, '
                            f'center_x={live_center_x:+.3f}m, '
                            f'yaw_relock={self.upstairs_lateral_shift_live_replan_count}/'
                            f'{self.upstairs_lateral_shift_live_check_max_replans}。'
                        )
                        return
                    self._publish_feedback(
                        f'上台阶横向补偿 live tag 复核仍超限但已达 yaw 重锁上限，继续 final_forward: '
                        f'{self._upstairs_live_tag_alignment_text(live_alignment)}, '
                        f'yaw_relock={self.upstairs_lateral_shift_live_replan_count}/'
                        f'{self.upstairs_lateral_shift_live_check_max_replans}。'
                    )
                self._relock_hurdle_legacy_final_forward(now)
                self._set_limit_bar_lateral_shift_phase('final_forward', now)
                self.limit_bar_lateral_shift_segment_start_x = self.pose_x
                self.limit_bar_lateral_shift_segment_start_y = self.pose_y
                self._publish_feedback(
                    f'{shift_label}横向补偿: 已转回基准 yaw={math.degrees(target_yaw):.1f}deg，'
                    f'{live_text + ", " if live_text else ""}'
                    f'开始前进到{self._lateral_shift_target_action_text()}阈值 final_forward='
                    f'{self.limit_bar_lateral_shift_final_forward_m:.3f}m。'
                )
                return
            if self._fail_limit_bar_lateral_phase_timeout(
                now,
                phase,
                shift_label,
                yaw_error=yaw_error,
                tolerance_rad=self.limit_bar_tag_align_tolerance_rad,
            ):
                return
            mode, left_norm, right_norm = self._limit_bar_lateral_shift_turn_override(yaw_error)
            label = 'align_back'

        elif phase == 'pre_duck_global_yaw_align':
            target_yaw = self._limit_bar_pre_duck_target_yaw()
            pre_duck_yaw_source = self._limit_bar_pre_duck_yaw_source()
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            if self.limit_bar_pre_duck_global_yaw_align_start_sec is None:
                self.limit_bar_pre_duck_global_yaw_align_start_sec = now

            yaw_abs = abs(yaw_error)
            if yaw_abs <= self.limit_bar_pre_duck_global_yaw_tolerance_rad:
                self.limit_bar_pre_duck_global_yaw_hold_active = True
            elif (
                self.limit_bar_pre_duck_global_yaw_hold_active
                and yaw_abs
                > self.limit_bar_pre_duck_global_yaw_exit_tolerance_rad
            ):
                self.limit_bar_pre_duck_global_yaw_hold_active = False

            holding_yaw = (
                self.limit_bar_pre_duck_global_yaw_hold_active
                and yaw_abs
                <= self.limit_bar_pre_duck_global_yaw_exit_tolerance_rad
            )
            yaw_rate_ready = self._limit_bar_lateral_yaw_rate_ready()
            if holding_yaw:
                mode = 0
                left_norm = 0.0
                right_norm = 0.0
                label = 'pre_duck_global_yaw_settle'
                if (
                    yaw_abs <= self.limit_bar_pre_duck_global_yaw_tolerance_rad
                    and yaw_rate_ready
                ):
                    if self.limit_bar_pre_duck_global_yaw_stable_since_sec is None:
                        self.limit_bar_pre_duck_global_yaw_stable_since_sec = now
                    stable_sec = (
                        now - self.limit_bar_pre_duck_global_yaw_stable_since_sec
                    )
                    label = 'pre_duck_global_yaw_hold'
                    if stable_sec >= self.limit_bar_pre_duck_global_yaw_hold_sec:
                        self._finish_limit_bar_pre_duck_global_yaw_align(
                            now,
                            target_yaw,
                            pre_duck_yaw_source,
                            f'限高杆最终前进后 yaw 对齐完成: '
                            f'target_yaw={math.degrees(target_yaw):.1f}deg, '
                            f'source={pre_duck_yaw_source}, '
                            f'yaw_err={math.degrees(yaw_error):.1f}deg, '
                            f'stable={stable_sec:.2f}s，进入趴下流程。',
                        )
                        return
                else:
                    self.limit_bar_pre_duck_global_yaw_stable_since_sec = None
            else:
                self.limit_bar_pre_duck_global_yaw_stable_since_sec = None

            elapsed_sec = now - self.limit_bar_pre_duck_global_yaw_align_start_sec
            if (
                self.limit_bar_pre_duck_global_yaw_timeout_sec > 0.0
                and elapsed_sec >= self.limit_bar_pre_duck_global_yaw_timeout_sec
            ):
                if getattr(
                    self,
                    'limit_bar_pre_duck_global_yaw_fail_open_enabled',
                    False,
                ):
                    self._finish_limit_bar_pre_duck_global_yaw_align(
                        now,
                        target_yaw,
                        pre_duck_yaw_source,
                        f'限高杆最终前进后 yaw 对齐超时已旁路: '
                        f'target_yaw={math.degrees(target_yaw):.1f}deg, '
                        f'source={pre_duck_yaw_source}, '
                        f'pose_yaw={math.degrees(self.pose_yaw):.1f}deg, '
                        f'yaw_err={math.degrees(yaw_error):.1f}deg, '
                        f'timeout={self.limit_bar_pre_duck_global_yaw_timeout_sec:.1f}s；'
                        '已停止转向并继续进入趴下流程，不进入 FAILED。',
                    )
                    return
                self._fail(
                    f'限高杆最终前进后 yaw 对齐超时: '
                    f'target_yaw={math.degrees(target_yaw):.1f}deg, '
                    f'source={pre_duck_yaw_source}, '
                    f'pose_yaw={math.degrees(self.pose_yaw):.1f}deg, '
                    f'yaw_err={math.degrees(yaw_error):.1f}deg, '
                    f'timeout={self.limit_bar_pre_duck_global_yaw_timeout_sec:.1f}s。'
                )
                return

            if not holding_yaw:
                if getattr(
                    self,
                    'limit_bar_pre_duck_stationary_turn_enabled',
                    False,
                ):
                    mode, left_norm, right_norm = (
                        self._limit_bar_pre_duck_stationary_turn_override(
                            yaw_error
                        )
                    )
                else:
                    mode, left_norm, right_norm = (
                        self._limit_bar_lateral_shift_turn_override(
                            yaw_error,
                            duck_align=True,
                        )
                    )
                label = 'pre_duck_global_yaw_align'

        elif phase == 'final_forward':
            target_yaw = self.limit_bar_lateral_shift_base_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            live_alignment = self._upstairs_live_tag_alignment(now)
            if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
                live_text = self._upstairs_live_tag_alignment_text(live_alignment)
                upstairs_distance_ready, upstairs_distance_reason = (
                    self._upstairs_final_approach_ready(
                        now,
                        live_alignment,
                    )
                )
                upstairs_distance_text = (
                    f'upstairs_final_ready={upstairs_distance_ready}'
                    f'({upstairs_distance_reason})'
                )
            along_m, raw_along_m = self._limit_bar_lateral_shift_progress(target_yaw)
            if along_m is None:
                self._fail(f'{shift_label}横向补偿无法计算最终前进里程。')
                return
            remaining_m = max(0.0, self.limit_bar_lateral_shift_final_forward_m - along_m)
            if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump':
                overshoot_m = max(
                    0.0,
                    along_m - self.limit_bar_lateral_shift_final_forward_m,
                )
                if (
                    self.hurdle_lateral_shift_legacy_max_forward_overshoot_m
                    > 0.0
                    and overshoot_m
                    > self.hurdle_lateral_shift_legacy_max_forward_overshoot_m
                ):
                    self._publish_step_override(0, 0.0, 0.0)
                    self._fail(
                        f'高墙旧横移流程末段前向过冲: '
                        f'overshoot={overshoot_m:.3f}m > '
                        f'{self.hurdle_lateral_shift_legacy_max_forward_overshoot_m:.3f}m。'
                    )
                    return
            visual_jump_ready = (
                self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump'
                and self._hurdle_visual_jump_ready(now)
            )
            visual_jump_yaw_ready = (
                visual_jump_ready
                and abs(yaw_error) <= self.hurdle_lateral_shift_visual_jump_yaw_gate_rad
                and self._limit_bar_lateral_yaw_rate_ready()
            )
            final_forward_tolerance_m = (
                self.hurdle_final_forward_tolerance_m
                if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump'
                else self.upstairs_final_forward_tolerance_m
                if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once'
                else self._lateral_shift_deadband_m()
            )
            odom_final_forward_ready = remaining_m <= final_forward_tolerance_m
            upstairs_waiting_for_distance_confirm = (
                self.limit_bar_lateral_shift_next_action == 'upstairs_step_once'
                and odom_final_forward_ready
                and not upstairs_distance_ready
            )
            if (
                odom_final_forward_ready
                or visual_jump_ready
                or upstairs_distance_ready
            ):
                if upstairs_waiting_for_distance_confirm:
                    self.limit_bar_lateral_shift_duck_align_since_sec = None
                    extra_forward_m = max(
                        0.0,
                        along_m - self.limit_bar_lateral_shift_final_forward_m,
                    )
                    creep_yaw_ready = (
                        abs(yaw_error)
                        <= self.upstairs_final_forward_realign_gate_rad
                    )
                    creep_distance_ready = (
                        self.upstairs_final_confirm_creep_max_extra_m > 0.0
                        and extra_forward_m
                        < self.upstairs_final_confirm_creep_max_extra_m
                    )
                    if creep_yaw_ready and creep_distance_ready:
                        mode = self.limit_bar_lateral_shift_walk_mode
                        left_norm, right_norm, yaw_corr_norm = (
                            self._limit_bar_lateral_shift_walk_yaw_corrected_norms(
                                self.upstairs_final_confirm_creep_left_norm,
                                self.upstairs_final_confirm_creep_right_norm,
                                yaw_error,
                            )
                        )
                        label = 'upstairs_final_confirm_creep'
                    elif abs(yaw_error) > self.upstairs_final_yaw_complete_gate_rad:
                        mode, left_norm, right_norm = (
                            self._limit_bar_lateral_shift_turn_override(
                                yaw_error,
                                duck_align=True,
                            )
                        )
                        label = 'upstairs_final_wait_distance_align'
                    else:
                        mode = 0
                        left_norm = 0.0
                        right_norm = 0.0
                        label = 'upstairs_final_wait_distance'
                    if (now - self.last_limit_bar_lateral_shift_log_sec) >= 0.30:
                        self.last_limit_bar_lateral_shift_log_sec = now
                        self._publish_feedback(
                            f'上台阶最终前进里程已到但距离未确认: '
                            f'final_along={along_m:.3f}m(raw={raw_along_m:.3f}m), '
                            f'remaining={remaining_m:.3f}m <= {final_forward_tolerance_m:.3f}m, '
                            f'extra_forward={extra_forward_m:.3f}/'
                            f'{self.upstairs_final_confirm_creep_max_extra_m:.3f}m, '
                            f'distance_ready={upstairs_distance_ready}'
                            f'({upstairs_distance_reason}), '
                            f'{live_text}，action={label}。'
                        )
                    self._publish_step_override(mode, left_norm, right_norm)
                    return
                if self._limit_bar_pre_duck_global_yaw_align_required():
                    pre_duck_target_yaw = self._limit_bar_pre_duck_target_yaw()
                    pre_duck_yaw_source = (
                        self._limit_bar_pre_duck_yaw_source()
                    )
                    self._set_limit_bar_lateral_shift_phase(
                        'pre_duck_global_yaw_align',
                        now,
                    )
                    self.limit_bar_pre_duck_global_yaw_align_start_sec = now
                    self.limit_bar_pre_duck_global_yaw_stable_since_sec = None
                    self._publish_step_override(0, 0.0, 0.0)
                    self._publish_feedback(
                        f'限高杆最终前进完成: final_along={along_m:.3f}m'
                        f'(raw={raw_along_m:.3f}m), remaining={remaining_m:.3f}m，'
                        f'趴下前开始对齐 yaw='
                        f'{math.degrees(pre_duck_target_yaw):.1f}deg, '
                        f'source={pre_duck_yaw_source}。'
                    )
                    return
                if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
                    if (
                        abs(yaw_error) <= self.upstairs_final_yaw_complete_gate_rad
                        and self._limit_bar_lateral_yaw_rate_ready()
                    ):
                        self._set_limit_bar_lateral_shift_phase('', now)
                        self.duck_reference_yaw = self._normalize_angle(
                            self.limit_bar_lateral_shift_base_yaw
                        )
                        self.duck_reference_yaw_source = 'upstairs_lateral_shift_base_yaw'
                        self.duck_reference_yaw_offset_rad = 0.0
                        self._publish_feedback(
                            f'上台阶横向补偿完成: final_along={along_m:.3f}m'
                            f'(raw={raw_along_m:.3f}m), yaw_err='
                            f'{math.degrees(yaw_error):.1f}deg <= '
                            f'{math.degrees(self.upstairs_final_yaw_complete_gate_rad):.1f}deg, '
                            f'remaining={remaining_m:.3f}m, '
                            f'distance_ready={upstairs_distance_ready}'
                            f'({upstairs_distance_reason}), '
                            f'{live_text}，直接进入上台阶 step_once。'
                        )
                        self._finish_upstairs_done(
                            now,
                            self.limit_bar_lateral_shift_reason,
                            final_along_m=along_m,
                            raw_final_along_m=raw_along_m,
                            final_remaining_m=remaining_m,
                        )
                        return
                    self.limit_bar_lateral_shift_duck_align_since_sec = None
                    self._set_limit_bar_lateral_shift_phase(
                        'upstairs_final_align',
                        now,
                    )
                    self.upstairs_final_align_latched_along_m = along_m
                    self.upstairs_final_align_latched_raw_along_m = raw_along_m
                    self.upstairs_final_align_latched_remaining_m = remaining_m
                    self.upstairs_final_align_latched_reason = upstairs_distance_reason
                    self.upstairs_final_align_latched_live_text = live_text
                    mode, left_norm, right_norm = self._limit_bar_lateral_shift_turn_override(
                        yaw_error,
                        duck_align=True,
                    )
                    label = 'upstairs_final_align'
                elif (
                    (
                        visual_jump_yaw_ready
                        or abs(yaw_error)
                        <= self.limit_bar_lateral_shift_duck_entry_yaw_gate_rad
                    )
                    and self._limit_bar_lateral_yaw_rate_ready()
                ):
                    if self.limit_bar_lateral_shift_duck_align_since_sec is None:
                        self.limit_bar_lateral_shift_duck_align_since_sec = now
                    hold_sec = (
                        self.limit_bar_lateral_shift_duck_entry_hold_sec
                        if visual_jump_yaw_ready
                        else now - self.limit_bar_lateral_shift_duck_align_since_sec
                    )
                    self._publish_step_override(0, 0.0, 0.0)
                    if hold_sec >= self.limit_bar_lateral_shift_duck_entry_hold_sec:
                        self._set_limit_bar_lateral_shift_phase('', now)
                        self.duck_reference_yaw = self._normalize_angle(
                            self.limit_bar_lateral_shift_base_yaw
                        )
                        self.duck_reference_yaw_source = 'limit_bar_lateral_shift_base_yaw'
                        self.duck_reference_yaw_offset_rad = 0.0
                        if not self.duck_centerline_correction_enabled:
                            self.duck_start_x = self.pose_x
                            self.duck_start_y = self.pose_y
                        if self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump':
                            self._complete_lateral_s_curve_shadow(
                                now,
                                completion_reason=(
                                    'visual_jump'
                                    if visual_jump_ready
                                    else 'odometry'
                                ),
                            )
                            visual_text = ''
                            if visual_jump_ready:
                                visual_text = (
                                    f', visual_dist={self._distance_text(self.hurdle_visual_detection.distance_m)}'
                                    f' <= {self.hurdle_lateral_shift_visual_jump_distance_m:.3f}m, '
                                    f'visual_yaw_gate={math.degrees(self.hurdle_lateral_shift_visual_jump_yaw_gate_rad):.1f}deg'
                                )
                            self._publish_feedback(
                                f'高墙横向补偿完成: final_along={along_m:.3f}m'
                                f'(raw={raw_along_m:.3f}m), yaw_err='
                                f'{math.degrees(yaw_error):.1f}deg{visual_text}，进入起跳等待。'
                            )
                            self._enter_hurdle_wait_jump(
                                now,
                                self.limit_bar_lateral_shift_reason,
                            )
                        elif self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
                            self._publish_feedback(
                                f'上台阶横向补偿完成: final_along={along_m:.3f}m'
                                f'(raw={raw_along_m:.3f}m), yaw_err='
                                f'{math.degrees(yaw_error):.1f}deg, '
                                f'{live_text}，进入上台阶 step_once。'
                            )
                            self._finish_upstairs_done(
                                now,
                                self.limit_bar_lateral_shift_reason,
                                final_along_m=along_m,
                                raw_final_along_m=raw_along_m,
                                final_remaining_m=remaining_m,
                            )
                        else:
                            self._publish_feedback(
                                f'限高杆横向补偿完成: final_along={along_m:.3f}m'
                                f'(raw={raw_along_m:.3f}m), yaw_err='
                                f'{math.degrees(yaw_error):.1f}deg，进入趴下流程；'
                                f'趴走 yaw 基准={self._duck_reference_yaw_text()}。'
                            )
                            self._enter_limit_bar_prepare(
                                now,
                                self.limit_bar_lateral_shift_reason,
                                self._duck_reference_yaw_text(),
                            )
                        return
                    label = 'duck_yaw_hold'
                else:
                    self.limit_bar_lateral_shift_duck_align_since_sec = None
                    mode, left_norm, right_norm = self._limit_bar_lateral_shift_turn_override(
                        yaw_error,
                        duck_align=True,
                    )
                    label = 'duck_align'
            elif abs(yaw_error) > (
                self.upstairs_final_forward_realign_gate_rad
                if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once'
                else self.limit_bar_lateral_shift_final_yaw_gate_rad
            ):
                self.limit_bar_lateral_shift_duck_align_since_sec = None
                mode, left_norm, right_norm = self._limit_bar_lateral_shift_turn_override(
                    yaw_error,
                    duck_align=(
                        self.limit_bar_lateral_shift_next_action
                        == 'upstairs_step_once'
                    ),
                )
                label = 'final_align'
            else:
                self.limit_bar_lateral_shift_duck_align_since_sec = None
                mode = self.limit_bar_lateral_shift_walk_mode
                upstairs_slow_approach = (
                    self.limit_bar_lateral_shift_next_action == 'upstairs_step_once'
                    and self.upstairs_final_slowdown_distance_m > 0.0
                    and remaining_m <= self.upstairs_final_slowdown_distance_m
                )
                hurdle_legacy_slow_approach = (
                    self.limit_bar_lateral_shift_next_action == 'hurdle_wait_jump'
                    and self.hurdle_lateral_shift_legacy_slowdown_distance_m > 0.0
                    and remaining_m
                    <= self.hurdle_lateral_shift_legacy_slowdown_distance_m
                )
                walk_left_norm = self.limit_bar_lateral_shift_walk_left_norm
                walk_right_norm = self.limit_bar_lateral_shift_walk_right_norm
                if upstairs_slow_approach:
                    walk_left_norm = self.upstairs_final_slow_walk_left_norm
                    walk_right_norm = self.upstairs_final_slow_walk_right_norm
                elif hurdle_legacy_slow_approach:
                    walk_left_norm = (
                        self.hurdle_lateral_shift_legacy_slow_left_norm
                    )
                    walk_right_norm = (
                        self.hurdle_lateral_shift_legacy_slow_right_norm
                    )
                left_norm, right_norm, yaw_corr_norm = (
                    self._limit_bar_lateral_shift_walk_yaw_corrected_norms(
                        walk_left_norm,
                        walk_right_norm,
                        yaw_error,
                    )
                )
                if self.limit_bar_lateral_shift_next_action == 'upstairs_step_once':
                    left_norm, right_norm, tag_yaw_corr_norm = (
                        self._upstairs_final_forward_tag_yaw_corrected_norms(
                            left_norm,
                            right_norm,
                            live_alignment,
                        )
                    )
                    yaw_corr_norm = max(yaw_corr_norm, tag_yaw_corr_norm)
                label = (
                    'upstairs_final_slow'
                    if upstairs_slow_approach
                    else 'hurdle_legacy_final_slow'
                    if hurdle_legacy_slow_approach
                    else 'final_forward'
                )
            if self._fail_limit_bar_lateral_phase_timeout(now, phase, shift_label):
                return

        elif phase == 'upstairs_final_align':
            if self.limit_bar_lateral_shift_next_action != 'upstairs_step_once':
                self._fail(f'{shift_label}横向补偿 phase=upstairs_final_align 仅允许上台阶动作。')
                return
            target_yaw = self.limit_bar_lateral_shift_base_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            along_m, raw_along_m = self._limit_bar_lateral_shift_progress(target_yaw)
            if along_m is None:
                along_m = self.upstairs_final_align_latched_along_m
                raw_along_m = self.upstairs_final_align_latched_raw_along_m
            remaining_m = (
                self.upstairs_final_align_latched_remaining_m
                if self.upstairs_final_align_latched_remaining_m is not None
                else 0.0
            )
            if self._limit_bar_lateral_turn_ready(
                now,
                yaw_error,
                self.upstairs_final_yaw_complete_gate_rad,
            ):
                self._set_limit_bar_lateral_shift_phase('', now)
                self.duck_reference_yaw = self._normalize_angle(
                    self.limit_bar_lateral_shift_base_yaw
                )
                self.duck_reference_yaw_source = 'upstairs_lateral_shift_base_yaw'
                self.duck_reference_yaw_offset_rad = 0.0
                finish_along_m = (
                    along_m
                    if along_m is not None
                    else self.upstairs_final_align_latched_along_m
                )
                finish_raw_along_m = (
                    raw_along_m
                    if raw_along_m is not None
                    else self.upstairs_final_align_latched_raw_along_m
                )
                finish_remaining_m = (
                    self.upstairs_final_align_latched_remaining_m
                    if self.upstairs_final_align_latched_remaining_m is not None
                    else remaining_m
                )
                self._publish_feedback(
                    f'上台阶最终起跳 yaw 对齐完成: yaw_err='
                    f'{math.degrees(yaw_error):.1f}deg <= '
                    f'{math.degrees(self.upstairs_final_yaw_complete_gate_rad):.1f}deg, '
                    f'latched_distance=({self.upstairs_final_align_latched_reason}), '
                    f'final_along={self._distance_text(finish_along_m)}, '
                    f'remaining={max(0.0, finish_remaining_m):.3f}m，'
                    f'直接进入上台阶 step_once。'
                )
                self._finish_upstairs_done(
                    now,
                    self.limit_bar_lateral_shift_reason,
                    final_along_m=finish_along_m,
                    raw_final_along_m=finish_raw_along_m,
                    final_remaining_m=finish_remaining_m,
                )
                return
            if self._fail_limit_bar_lateral_phase_timeout(now, phase, shift_label):
                return
            self.limit_bar_lateral_shift_duck_align_since_sec = None
            mode, left_norm, right_norm = self._limit_bar_lateral_shift_turn_override(
                yaw_error,
                duck_align=True,
            )
            label = 'upstairs_final_align_locked'
            upstairs_distance_text = (
                'upstairs_final_ready=True'
                f'(latched:{self.upstairs_final_align_latched_reason})'
            )
            live_text = self.upstairs_final_align_latched_live_text

        else:
            self._fail(f'{shift_label}横向补偿未知 phase={phase}。')
            return

        self._publish_step_override(mode, left_norm, right_norm)
        if (now - self.last_limit_bar_lateral_shift_log_sec) >= 0.50:
            self.last_limit_bar_lateral_shift_log_sec = now
            along_text = 'unknown' if along_m is None else f'{along_m:.3f}m'
            raw_text = 'unknown' if raw_along_m is None else f'{raw_along_m:.3f}m'
            self._publish_feedback(
                f'{shift_label}横向补偿中: phase={phase}/{label}, '
                f'target_yaw={math.degrees(target_yaw):.1f}deg, '
                f'pose_yaw={math.degrees(self.pose_yaw):.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):.1f}deg, '
                f'yaw_rate={math.degrees(getattr(self, "pose_yaw_rate_rad_s", 0.0)):+.1f}deg/s, '
                f'yaw_rate_raw={math.degrees(getattr(self, "pose_yaw_rate_raw_rad_s", 0.0)):+.1f}deg/s, '
                f'along={along_text}, raw_along={raw_text}, '
                f'remaining={remaining_m:.3f}m, '
                f'walk_yaw_corr={yaw_corr_norm:.3f}, '
                f'{upstairs_distance_text + ", " if upstairs_distance_text else ""}'
                f'{live_text + ", " if live_text else ""}'
                f'override=[{mode},{left_norm:.3f},{right_norm:.3f}]。'
            )

    def _enter_slope_or_align(
        self,
        now: float,
        reason: str,
        feedback: str,
        *,
        align_next_action: str = 'slope_branch',
        allow_upstairs_preempt: bool = True,
        force_align: bool = False,
    ) -> None:
        if (
            self.slope_retry_rezero_search_active
            and self.slope_route_yaw_reference is not None
        ):
            allow_upstairs_preempt = False
            self.slope_align_custom_target_yaw = float(
                self.slope_route_yaw_reference
            )
            self._publish_slope_route_yaw_reference(
                now,
                self.slope_route_yaw_reference,
            )
        if allow_upstairs_preempt:
            if self._upstairs_ready(now):
                self._enter_detected_branch('upstairs', now, f'{reason}_upstairs_preempt')
                return
            if self._upstairs_slope_guard_ready(now):
                self._publish_feedback(
                    '检测到疑似上台阶，忽略本次斜坡触发，继续搜索等待上台阶触发距离。'
                )
                if self.state != self.STATE_SEARCH:
                    self._enter_search(now, reason=f'{reason}_upstairs_guard')
                return
        if self.slope_branch_completed:
            self._publish_feedback('斜坡分支已经完成，本次斜坡触发被忽略，继续搜索后续障碍。')
            self._enter_search(now, reason='slope_already_completed')
            return
        if (not self.slope_align_enabled and not force_align) or (
            align_next_action == 'slope_branch'
            and not self.slope_align_on_detected_branch_enabled
        ):
            self._enter_slope_branch(feedback)
            return
        if self.pose_yaw is None:
            self._publish_feedback('准备进入斜坡分支，但当前没有可用 yaw，先继续搜索等待 Odometry。')
            if self.state != self.STATE_SEARCH:
                self._enter_search(now, reason=f'{reason}_waiting_odom_yaw')
            return
        self._clear_step_override()
        self.state = self.STATE_SLOPE_ALIGN
        self.current_active_branch = 'slope_align'
        self.slope_align_enter_time = now
        self.slope_align_reason = reason
        self.slope_align_feedback = feedback
        self.slope_align_next_action = align_next_action
        self.last_slope_align_log_time = 0.0
        self.slope_align_stable_since = None
        self._publish_state(self.state)
        self._publish_feedback(
            f'进入斜坡前 yaw 对齐: reason={reason}, '
            f'target={math.degrees(self._slope_align_target_yaw()):.1f}deg, '
            f'tolerance={math.degrees(self.slope_align_tolerance_rad):.1f}deg, '
            f'hold={self.slope_align_exit_hold_sec:.2f}s。'
        )

    def _enter_slope_branch(self, feedback: str) -> None:
        self._clear_step_override()
        if (
            self.slope_retry_rezero_search_active
            and self.slope_route_yaw_reference is not None
        ):
            self._publish_slope_route_yaw_reference(
                self._now_sec(),
                self.slope_route_yaw_reference,
            )
        self.slope_retry_rezero_search_active = False
        self.slope_branch_completed = True
        self.state = self.STATE_SLOPE
        self.current_active_branch = 'slope'
        self.slope_state = 'STARTING'
        self.slope_align_next_action = 'slope_branch'
        self.post_stairs_slope_entry_start_x = None
        self.post_stairs_slope_entry_start_y = None
        self.slope_roll_detection_start_sec = 0.0
        self.last_slope_roll_detection_log_sec = 0.0
        self.post_stairs_slope_roll_priority_start_sec = 0.0
        self.last_post_stairs_slope_roll_priority_log_sec = 0.0
        self.phase_deadline_sec = None
        self._publish_state(self.state)
        self._publish_feedback(feedback)

    def _enter_post_stairs_slope_approach(self, now: float) -> None:
        self.state = self.STATE_POST_STAIRS_SLOPE_APPROACH
        self.current_active_branch = 'post_stairs_slope_entry'
        self.post_stairs_slope_entry_start_x = None
        self.post_stairs_slope_entry_start_y = None
        self.last_post_stairs_slope_log_time = 0.0
        self.phase_deadline_sec = (
            now + self.post_stairs_slope_entry_timeout_sec
            if self.post_stairs_slope_entry_timeout_sec > 0.0
            else None
        )
        self._publish_state(self.state)
        self._publish_feedback(
            f'进入固定斜坡入口前进: distance={self.post_stairs_slope_entry_distance_m:.2f}m, '
            f'override=[{self.post_stairs_slope_entry_mode},'
            f'{self.post_stairs_slope_entry_left_norm:.3f},'
            f'{self.post_stairs_slope_entry_right_norm:.3f}], '
            f'yaw_target={math.degrees(self._post_stairs_slope_entry_yaw_target()):.1f}deg, '
            f'yaw_correction={self.post_stairs_slope_entry_yaw_correction_enabled}。'
        )

    def _run_post_stairs_slope_approach(self, now: float) -> None:
        if not self._pose_is_fresh(now):
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_feedback('固定斜坡入口前进等待新鲜 Odometry。')
            return

        if self.post_stairs_slope_entry_start_x is None or self.post_stairs_slope_entry_start_y is None:
            self.post_stairs_slope_entry_start_x = self.pose_x
            self.post_stairs_slope_entry_start_y = self.pose_y
            self.last_post_stairs_slope_log_time = 0.0
            self._publish_feedback(
                f'固定斜坡入口前进起点=({self.pose_x:.3f},{self.pose_y:.3f})。'
            )

        travel_m = math.hypot(
            self.pose_x - self.post_stairs_slope_entry_start_x,
            self.pose_y - self.post_stairs_slope_entry_start_y,
        )
        if travel_m >= self.post_stairs_slope_entry_distance_m:
            self._clear_step_override()
            yaw_target = self._post_stairs_slope_entry_yaw_target()
            yaw_error = self._normalize_angle(yaw_target - self.pose_yaw)
            yaw_error_deg = math.degrees(yaw_error)
            if (
                self.post_stairs_slope_entry_initial_yaw_align_enabled
                and self.post_stairs_slope_entry_finish_yaw_realign_threshold_rad > 0.0
                and abs(yaw_error) > self.post_stairs_slope_entry_finish_yaw_realign_threshold_rad
            ):
                self._publish_feedback(
                    f'固定斜坡入口前进完成但 yaw 偏差过大: travel={travel_m:.3f}m, '
                    f'yaw_err={yaw_error_deg:.1f}deg > '
                    f'{math.degrees(self.post_stairs_slope_entry_finish_yaw_realign_threshold_rad):.1f}deg，'
                    f'回到 yaw 对齐后再执行入口后续分流。'
                )
                self._enter_slope_or_align(
                    now,
                    reason='post_stairs_slope_entry_finish_yaw_realign',
                    feedback=self._post_stairs_slope_post_entry_feedback(),
                    align_next_action='post_stairs_slope_confirm',
                    allow_upstairs_preempt=False,
                    force_align=True,
                )
                return
            self._publish_feedback(
                f'固定斜坡入口前进完成: travel={travel_m:.3f}m >= '
                f'{self.post_stairs_slope_entry_distance_m:.3f}m, '
                f'yaw_err={yaw_error_deg:.1f}deg, target={math.degrees(yaw_target):.1f}deg。'
            )
            self._enter_post_stairs_slope_confirm_or_branch(now)
            return

        if self.phase_deadline_sec is not None and now >= self.phase_deadline_sec:
            self._clear_step_override()
            next_step_text = '进入搜索模式。'
            if not self.post_stairs_slope_search_after_entry:
                next_step_text = '进入固定入口识别确认。'
                if not self.post_stairs_slope_confirm_required:
                    next_step_text = '直接进入斜坡分支。'
            self._publish_feedback(
                f'固定斜坡入口前进超时: travel={travel_m:.3f}m, '
                f'timeout={self.post_stairs_slope_entry_timeout_sec:.1f}s，{next_step_text}'
            )
            self._enter_post_stairs_slope_confirm_or_branch(now)
            return

        yaw_target = self._post_stairs_slope_entry_yaw_target()
        yaw_error = self._normalize_angle(yaw_target - self.pose_yaw)
        yaw_correction_norm = 0.0
        left_norm = self.post_stairs_slope_entry_left_norm
        right_norm = self.post_stairs_slope_entry_right_norm
        if self.post_stairs_slope_entry_yaw_correction_enabled:
            yaw_excess = max(0.0, abs(yaw_error) - self.post_stairs_slope_entry_yaw_tolerance_rad)
            if yaw_excess > 0.0:
                yaw_correction_norm = math.copysign(
                    min(
                        self.post_stairs_slope_entry_yaw_max_delta_norm,
                        yaw_excess * self.post_stairs_slope_entry_yaw_gain_norm_per_rad,
                    ),
                    yaw_error,
                )
                left_norm = self._clamp_norm(left_norm - yaw_correction_norm)
                right_norm = self._clamp_norm(right_norm + yaw_correction_norm)

        self._publish_step_override(
            self.post_stairs_slope_entry_mode,
            left_norm,
            right_norm,
        )
        if (now - self.last_post_stairs_slope_log_time) >= 0.8:
            self.last_post_stairs_slope_log_time = now
            self._publish_feedback(
                f'固定斜坡入口前进中: travel={travel_m:.3f}/'
                f'{self.post_stairs_slope_entry_distance_m:.3f}m, '
                f'yaw_err={math.degrees(yaw_error):.1f}deg, '
                f'yaw_corr={yaw_correction_norm:.3f}, '
                f'override=[{self.post_stairs_slope_entry_mode},'
                f'{left_norm:.3f},{right_norm:.3f}]。'
            )

    def _enter_post_stairs_slope_confirm_or_branch(self, now: float) -> None:
        if self.post_stairs_slope_search_after_entry:
            self._clear_step_override()
            self._reset_detection_states()
            self._enter_search(now, reason='post_stairs_entry_done')
            self.search_arm_deadline = now + self.post_stairs_slope_search_rearm_delay_sec
            return
        if not self.post_stairs_slope_confirm_required:
            self._enter_slope_branch('固定斜坡入口已到达，开始斜坡分支。')
            return
        self.state = self.STATE_POST_STAIRS_SLOPE_CONFIRM
        self.current_active_branch = 'post_stairs_slope_confirm'
        self.post_stairs_slope_confirm_start_time = now
        self._reset_detection_states()
        self.phase_deadline_sec = (
            now + self.post_stairs_slope_confirm_timeout_sec
            if self.post_stairs_slope_confirm_timeout_sec > 0.0
            else None
        )
        self._publish_state(self.state)
        self._publish_feedback('固定斜坡入口已到达，原地等待障碍识别确认。')

    def _enter_hurdle_post_align(self, now: float) -> bool:
        if not self.hurdle_post_align_enabled:
            return False
        if self.pose_yaw is None:
            self._publish_feedback('高墙后 yaw 对齐已启用，但当前没有可用 yaw，直接进入搜索。')
            return False
        if self.hurdle_entry_yaw is None:
            self.hurdle_entry_yaw = self.pose_yaw
        self._clear_step_override()
        self.state = self.STATE_HURDLE_POST_ALIGN
        self.current_active_branch = 'hurdle_post_align'
        self.phase_deadline_sec = None
        self.slope_align_enter_time = now
        self.last_slope_align_log_time = 0.0
        self.slope_align_stable_since = None
        self._publish_state(self.state)
        self._publish_feedback(
            f'进入高墙后 yaw 对齐: target={math.degrees(self._hurdle_post_align_target_yaw()):.1f}deg, '
            f'tolerance={math.degrees(self.hurdle_post_align_tolerance_rad):.1f}deg, '
            f'hold={self.hurdle_post_align_exit_hold_sec:.2f}s。'
        )
        return True

    def _run_hurdle_post_align(self, now: float) -> None:
        if not self._pose_is_fresh(now):
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_feedback('高墙后 yaw 对齐等待新鲜 Odometry。')
            return

        target_yaw = self._hurdle_post_align_target_yaw()
        yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
        yaw_error_deg = math.degrees(yaw_error)
        if abs(yaw_error) <= self.hurdle_post_align_tolerance_rad:
            if self.slope_align_stable_since is None:
                self.slope_align_stable_since = now
            stable_sec = now - self.slope_align_stable_since
            self._publish_step_override(
                self.hurdle_post_align_hold_mode,
                self.hurdle_post_align_hold_left_norm,
                self.hurdle_post_align_hold_right_norm,
            )
            if stable_sec >= self.hurdle_post_align_exit_hold_sec:
                self._clear_step_override()
                self.slope_align_stable_since = None
                self._publish_feedback(
                    f'高墙后 yaw 对齐完成: yaw_err={yaw_error_deg:.1f}deg, '
                    f'stable={stable_sec:.2f}s，进入搜索。'
                )
                self._enter_search(now, reason='hurdle_done_aligned')
                return
            if (now - self.last_slope_align_log_time) >= 0.5:
                self.last_slope_align_log_time = now
                self._publish_feedback(
                    f'高墙后 yaw 对齐保持中: yaw_err={yaw_error_deg:.1f}deg, '
                    f'stable={stable_sec:.2f}/{self.hurdle_post_align_exit_hold_sec:.2f}s。'
                )
            return
        self.slope_align_stable_since = None

        if (
            self.hurdle_post_align_timeout_sec > 0.0
            and (now - self.slope_align_enter_time) > self.hurdle_post_align_timeout_sec
        ):
            if self.hurdle_post_align_timeout_action == 'advance':
                self._clear_step_override()
                self._publish_feedback(
                    f'高墙后 yaw 对齐超时，按配置进入搜索: yaw_err={yaw_error_deg:.1f}deg, '
                    f'timeout={self.hurdle_post_align_timeout_sec:.1f}s。'
                )
                self._enter_search(now, reason='hurdle_done_align_timeout')
                return
            if self.hurdle_post_align_timeout_action == 'fail':
                self._fail(
                    f'高墙后 yaw 对齐超时: yaw_err={yaw_error_deg:.1f}deg, '
                    f'timeout={self.hurdle_post_align_timeout_sec:.1f}s。'
                )
                return

        use_fine = (
            self.hurdle_post_align_fine_threshold_rad > self.hurdle_post_align_tolerance_rad
            and abs(yaw_error) <= self.hurdle_post_align_fine_threshold_rad
        )
        if yaw_error > 0.0:
            if use_fine:
                left_norm = self.hurdle_post_align_fine_ccw_left_norm
                right_norm = self.hurdle_post_align_fine_ccw_right_norm
            else:
                left_norm = self.hurdle_post_align_ccw_left_norm
                right_norm = self.hurdle_post_align_ccw_right_norm
            direction = 'ccw_fine' if use_fine else 'ccw'
        else:
            if use_fine:
                left_norm = self.hurdle_post_align_fine_cw_left_norm
                right_norm = self.hurdle_post_align_fine_cw_right_norm
            else:
                left_norm = self.hurdle_post_align_cw_left_norm
                right_norm = self.hurdle_post_align_cw_right_norm
            direction = 'cw_fine' if use_fine else 'cw'
        self._publish_step_override(
            self.hurdle_post_align_mode,
            left_norm,
            right_norm,
        )
        if (now - self.last_slope_align_log_time) >= 0.5:
            self.last_slope_align_log_time = now
            self._publish_feedback(
                f'高墙后 yaw 对齐中: yaw_err={yaw_error_deg:.1f}deg, '
                f'direction={direction}, '
                f'override=[{self.hurdle_post_align_mode},{left_norm:.3f},{right_norm:.3f}]。'
            )

    def _run_post_stairs_slope_confirm(self, now: float) -> None:
        self._publish_step_override(
            self.post_stairs_slope_confirm_hold_mode,
            self.post_stairs_slope_confirm_hold_left_norm,
            self.post_stairs_slope_confirm_hold_right_norm,
        )
        branch = self._classify_search_branch(now)
        if branch in ('slope', 'slope_roll'):
            self._clear_step_override()
            self._enter_detected_branch(
                branch,
                now,
                f'post_stairs_{branch}_detected',
            )
            return
        if branch in ('limit_bar', 'hurdle', 'pole', 'upstairs'):
            self._clear_step_override()
            self._publish_feedback(
                f'固定入口处障碍识别完成: branch={branch}，切换到对应分支。'
            )
            self._enter_detected_branch(
                branch,
                now,
                f'post_stairs_{branch}_detected',
            )
            return
        if self.phase_deadline_sec is not None and now >= self.phase_deadline_sec:
            if self.post_stairs_slope_confirm_timeout_action == 'advance':
                self._clear_step_override()
                self._enter_slope_branch(
                    f'固定入口处等待斜坡视觉确认超时 '
                    f'{self.post_stairs_slope_confirm_timeout_sec:.1f}s，按配置继续斜坡分支。'
                )
                return
            if self.post_stairs_slope_confirm_timeout_action == 'fail':
                self._fail(
                    f'固定入口处等待斜坡视觉确认超时 '
                    f'{self.post_stairs_slope_confirm_timeout_sec:.1f}s。'
                )
                return
            if self.post_stairs_slope_confirm_timeout_action == 'search':
                self._clear_step_override()
                self._publish_feedback(
                    f'固定入口处等待障碍识别超时 '
                    f'{self.post_stairs_slope_confirm_timeout_sec:.1f}s，返回搜索模式。'
                )
                self._enter_search(now, reason='post_stairs_confirm_timeout')
                return

    def _enter_limit_bar_wait(self, now: float, feedback: str) -> None:
        self._publish_step_override(
            self.duck_resume_mode,
            self.duck_resume_left_norm,
            self.duck_resume_right_norm,
        )
        self.phase_deadline_sec = now + self.duck_resume_delay_sec
        self.state = self.STATE_LIMIT_BAR_WAIT
        self._publish_state(self.state)
        self._publish_feedback(feedback)

    def _enter_limit_bar_post_resume_align(self, now: float) -> None:
        self._clear_step_override()
        self.limit_bar_post_resume_yaw_align_start_sec = now
        self.limit_bar_post_resume_yaw_stable_since_sec = None
        self.last_limit_bar_post_resume_yaw_log_sec = 0.0
        self.phase_deadline_sec = (
            now + self.limit_bar_post_resume_yaw_timeout_sec
            if self.limit_bar_post_resume_yaw_timeout_sec > 0.0
            else None
        )
        self.state = self.STATE_LIMIT_BAR_POST_RESUME_ALIGN
        self._publish_state(self.state)
        self._publish_feedback(
            '限高杆站起后 yaw 复对齐开始: '
            f'target={self._duck_reference_yaw_text()}, '
            f'pose_yaw={self._yaw_text(self.pose_yaw)}, '
            f'tolerance={math.degrees(self.limit_bar_post_resume_yaw_tolerance_rad):.1f}deg。'
        )

    def _run_limit_bar_post_resume_align(self, now: float) -> None:
        if self.duck_reference_yaw is None:
            self._finish_limit_bar_done(now, reason='limit_bar_done_no_post_resume_yaw_ref')
            return
        if not self._pose_is_fresh(now):
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_feedback('限高杆站起后 yaw 复对齐等待新鲜 Odometry。')
            return

        yaw_error = self._normalize_angle(self.duck_reference_yaw - self.pose_yaw)
        yaw_error_deg = math.degrees(yaw_error)
        if abs(yaw_error) <= self.limit_bar_post_resume_yaw_tolerance_rad:
            if self.limit_bar_post_resume_yaw_stable_since_sec is None:
                self.limit_bar_post_resume_yaw_stable_since_sec = now
            stable_sec = now - self.limit_bar_post_resume_yaw_stable_since_sec
            self._publish_step_override(0, 0.0, 0.0)
            if stable_sec >= self.limit_bar_post_resume_yaw_hold_sec:
                self._publish_feedback(
                    f'限高杆站起后 yaw 复对齐完成: yaw_err={yaw_error_deg:.1f}deg, '
                    f'stable={stable_sec:.2f}s, target={self._duck_reference_yaw_text()}。'
                )
                self._finish_limit_bar_done(now, reason='limit_bar_done')
                return
            if (now - self.last_limit_bar_post_resume_yaw_log_sec) >= 0.30:
                self.last_limit_bar_post_resume_yaw_log_sec = now
                self._publish_feedback(
                    f'限高杆站起后 yaw 复对齐保持中: yaw_err={yaw_error_deg:.1f}deg, '
                    f'stable={stable_sec:.2f}/{self.limit_bar_post_resume_yaw_hold_sec:.2f}s。'
                )
            return

        self.limit_bar_post_resume_yaw_stable_since_sec = None
        if self.phase_deadline_sec is not None and now >= self.phase_deadline_sec:
            self._publish_feedback(
                f'限高杆站起后 yaw 复对齐超时: yaw_err={yaw_error_deg:.1f}deg, '
                f'target={self._duck_reference_yaw_text()}，继续后续流程。'
            )
            self._finish_limit_bar_done(now, reason='limit_bar_done_post_resume_align_timeout')
            return

        mode, left_norm, right_norm = self._limit_bar_lateral_shift_turn_override(
            yaw_error,
            duck_align=True,
        )
        self._publish_step_override(mode, left_norm, right_norm)
        if (now - self.last_limit_bar_post_resume_yaw_log_sec) >= 0.30:
            self.last_limit_bar_post_resume_yaw_log_sec = now
            direction = 'ccw' if yaw_error > 0.0 else 'cw'
            timeout_left_text = 'none'
            if self.phase_deadline_sec is not None:
                timeout_left_text = f'{max(0.0, self.phase_deadline_sec - now):.2f}s'
            self._publish_feedback(
                f'限高杆站起后 yaw 复对齐中: yaw_err={yaw_error_deg:.1f}deg, '
                f'direction={direction}, target={self._duck_reference_yaw_text()}, '
                f'override=[{mode},{left_norm:.3f},{right_norm:.3f}], '
                f'timeout_left={timeout_left_text}。'
            )

    def _finish_limit_bar_done(self, now: float, *, reason: str) -> None:
        self._clear_step_override()
        self.phase_deadline_sec = None
        self.limit_bar_post_resume_yaw_align_start_sec = None
        self.limit_bar_post_resume_yaw_stable_since_sec = None
        self.limit_bar_completed = True
        if self._sandpit_pre_align_ready(now):
            self._publish_feedback(
                f'限高杆完成后直接进入沙坑前预修正: reason={reason}，'
                '避免 SEARCH 前进步抢先执行。'
            )
            self._enter_sandpit_pre_align(now, reason='limit_bar_done_sandpit_pre_align')
            return
        self._enter_search(now, reason=reason)

    def _run_slope_align(self, now: float) -> None:
        label = self._yaw_align_label()
        if not self._pose_is_fresh(now):
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_feedback(f'{label} yaw 对齐等待新鲜 Odometry。')
            return

        if (
            self.state in (
                self.STATE_LIMIT_BAR_ALIGN,
                self.STATE_HURDLE_ALIGN,
                self.STATE_UPSTAIRS_ALIGN,
            )
            and self.limit_bar_lateral_pre_align_active
        ):
            self._run_limit_bar_lateral_pre_align(now)
            return

        use_limit_tag_params = (
            self.state in (
                self.STATE_LIMIT_BAR_ALIGN,
                self.STATE_HURDLE_ALIGN,
                self.STATE_UPSTAIRS_ALIGN,
            )
            and self.limit_bar_tag_align_active
        )
        tolerance_rad = (
            self.limit_bar_tag_align_tolerance_rad
            if use_limit_tag_params
            else self.slope_align_tolerance_rad
        )
        timeout_sec = (
            self.limit_bar_tag_align_timeout_sec
            if use_limit_tag_params
            else self.slope_align_timeout_sec
        )
        target_yaw = self._slope_align_target_yaw()
        yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
        yaw_error_deg = math.degrees(yaw_error)
        if abs(yaw_error) <= tolerance_rad:
            if self.slope_align_stable_since is None:
                self.slope_align_stable_since = now
            stable_sec = now - self.slope_align_stable_since
            self._publish_step_override(
                self.slope_align_hold_mode,
                self.slope_align_hold_left_norm,
                self.slope_align_hold_right_norm,
            )
            if stable_sec >= self.slope_align_exit_hold_sec:
                self._clear_step_override()
                self._publish_feedback(
                    f'{label} yaw 对齐完成: yaw_err={yaw_error_deg:.1f}deg, '
                    f'stable={stable_sec:.2f}s。'
                )
                self._finish_slope_align(now, target_yaw)
                return
            if (now - self.last_slope_align_log_time) >= 0.5:
                self.last_slope_align_log_time = now
                self._publish_feedback(
                    f'{label} yaw 对齐保持中: yaw_err={yaw_error_deg:.1f}deg, '
                    f'stable={stable_sec:.2f}/{self.slope_align_exit_hold_sec:.2f}s。'
                )
            return
        self.slope_align_stable_since = None

        if (
            timeout_sec > 0.0
            and (now - self.slope_align_enter_time) > timeout_sec
        ):
            if use_limit_tag_params:
                self._fail(
                    f'{label} AprilTag yaw 对齐超时: yaw_err={yaw_error_deg:.1f}deg, '
                    f'target_yaw={math.degrees(target_yaw):.1f}deg, '
                    f'timeout={timeout_sec:.1f}s。'
                )
                return
            if self.slope_align_timeout_action == 'advance':
                self._clear_step_override()
                self._publish_feedback(
                    f'{label} yaw 对齐超时，按配置继续后续流程: '
                    f'yaw_err={yaw_error_deg:.1f}deg, '
                    f'timeout={timeout_sec:.1f}s。',
                )
                self._finish_slope_align(now, target_yaw)
                return
            if self.slope_align_timeout_action == 'fail':
                self._fail(
                    f'{label} yaw 对齐超时: yaw_err={yaw_error_deg:.1f}deg, '
                    f'timeout={timeout_sec:.1f}s。'
                )
                return

        use_fine = (
            not use_limit_tag_params
            and
            self.slope_align_fine_threshold_rad > self.slope_align_tolerance_rad
            and abs(yaw_error) <= self.slope_align_fine_threshold_rad
        )
        if use_limit_tag_params:
            left_norm, right_norm = self._limit_bar_tag_turn_norms(yaw_error)
            direction = 'ccw' if yaw_error > 0.0 else 'cw'
        elif yaw_error > 0.0:
            if use_fine:
                left_norm = self.slope_align_fine_ccw_left_norm
                right_norm = self.slope_align_fine_ccw_right_norm
            else:
                left_norm = self.slope_align_ccw_left_norm
                right_norm = self.slope_align_ccw_right_norm
            direction = 'ccw_fine' if use_fine else 'ccw'
        else:
            if use_fine:
                left_norm = self.slope_align_fine_cw_left_norm
                right_norm = self.slope_align_fine_cw_right_norm
            else:
                left_norm = self.slope_align_cw_left_norm
                right_norm = self.slope_align_cw_right_norm
            direction = 'cw_fine' if use_fine else 'cw'
        align_mode = (
            self.limit_bar_tag_align_mode
            if use_limit_tag_params
            else self.slope_align_mode
        )
        self._publish_step_override(align_mode, left_norm, right_norm)

        if (now - self.last_slope_align_log_time) >= 0.5:
            self.last_slope_align_log_time = now
            self._publish_feedback(
                f'{label} yaw 对齐中: yaw_err={yaw_error_deg:.1f}deg, '
                f'direction={direction}, '
                f'override=[{align_mode},{left_norm:.3f},{right_norm:.3f}]。'
            )

    def _finish_slope_align(self, now: float, reference_yaw: float) -> None:
        align_duration_sec = max(0.0, now - self.slope_align_enter_time)
        self.slope_align_stable_since = None
        self._publish_slope_align_reference(now, reference_yaw, align_duration_sec)
        if (
            self.state == self.STATE_SLOPE_ALIGN
            and self._slope_runtime_yaw_reference_enabled()
        ):
            self._publish_slope_route_yaw_reference(now, reference_yaw)
        self.slope_align_custom_target_yaw = None
        if self.slope_align_next_action == 'post_stairs_slope_approach':
            self._enter_post_stairs_slope_approach(now)
            return
        if self.slope_align_next_action == 'limit_bar_prepare':
            if self.limit_bar_tag_align_active:
                self.duck_reference_yaw = self._normalize_angle(reference_yaw)
                if self.limit_bar_lateral_shift_enabled:
                    self.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'
                    if self._start_limit_bar_lateral_shift(
                        now,
                        self.slope_align_reason,
                        self._duck_reference_yaw_text(),
                    ):
                        return
            self._enter_limit_bar_prepare(
                now,
                self.slope_align_reason,
                self._duck_reference_yaw_text(),
            )
            return
        if self.slope_align_next_action == 'hurdle_wait_jump':
            if self.limit_bar_tag_align_active:
                self.duck_reference_yaw = self._normalize_angle(reference_yaw)
                self.limit_bar_lateral_shift_next_action = 'hurdle_wait_jump'
                if self.limit_bar_lateral_shift_enabled:
                    if self._start_limit_bar_lateral_shift(
                        now,
                        self.slope_align_reason,
                        self._duck_reference_yaw_text(),
                    ):
                        return
            self._enter_hurdle_wait_jump(now, self.slope_align_reason)
            return
        if self.slope_align_next_action == 'upstairs_step_once':
            if self.limit_bar_tag_align_active:
                self.duck_reference_yaw = self._normalize_angle(reference_yaw)
                self.limit_bar_lateral_shift_next_action = 'upstairs_step_once'
                if self.limit_bar_lateral_shift_enabled:
                    if self._start_limit_bar_lateral_shift(
                        now,
                        self.slope_align_reason,
                        self._duck_reference_yaw_text(),
                    ):
                        return
            self._finish_upstairs_done(now, self.slope_align_reason)
            return
        if self.slope_align_next_action == 'post_stairs_slope_confirm':
            self._enter_post_stairs_slope_confirm_or_branch(now)
            return
        self._enter_slope_branch(self.slope_align_feedback)

    def _post_stairs_slope_post_entry_feedback(self) -> str:
        if self.post_stairs_slope_search_after_entry:
            return '固定斜坡入口前进后 yaw 已重新对齐，进入搜索模式等待障碍分流。'
        if self.post_stairs_slope_confirm_required:
            return '固定斜坡入口前进后 yaw 已重新对齐，进入固定入口识别确认。'
        return '固定斜坡入口前进后 yaw 已重新对齐，开始斜坡分支。'

    def _yaw_align_label(self) -> str:
        if self.state == self.STATE_LIMIT_BAR_ALIGN:
            return '限高杆前'
        if self.state == self.STATE_HURDLE_ALIGN:
            return '高墙前'
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return '上台阶前'
        return '斜坡前'

    def _slope_align_target_yaw(self) -> float:
        if self.slope_align_custom_target_yaw is not None:
            return self.slope_align_custom_target_yaw
        if self.slope_align_use_startup_yaw and self.startup_pose_yaw is not None:
            return self.startup_pose_yaw
        return self.slope_align_target_yaw

    def _downstream_route_yaw_reference_is_fresh(self) -> bool:
        if not getattr(self, 'downstream_route_yaw_reference_enabled', False):
            return False
        if getattr(self, 'slope_route_yaw_reference', None) is None:
            return False
        timeout_sec = float(
            getattr(self, 'downstream_route_yaw_reference_timeout_sec', 0.0)
        )
        if timeout_sec <= 0.0:
            return True
        stamp_sec = float(getattr(self, 'slope_route_yaw_reference_stamp_sec', 0.0))
        return (self._now_sec() - stamp_sec) <= timeout_sec

    def _downstream_route_adjusted_yaw(
        self,
        nominal_yaw: float,
        branch_enabled_attr: str,
    ) -> float:
        if (
            getattr(self, branch_enabled_attr, True)
            and self._downstream_route_yaw_reference_is_fresh()
        ):
            return self._normalize_angle(
                float(self.slope_route_yaw_reference) + float(nominal_yaw)
            )
        return self._normalize_angle(float(nominal_yaw))

    def _hurdle_fixed_reference_target_yaw(self) -> float:
        return self._downstream_route_adjusted_yaw(
            self.hurdle_tag_fixed_reference_yaw,
            'downstream_route_yaw_hurdle_enabled',
        )

    def _limit_bar_fixed_reference_target_yaw(self) -> float:
        return self._downstream_route_adjusted_yaw(
            self.limit_bar_tag_fixed_reference_yaw,
            'downstream_route_yaw_limit_bar_pre_duck_enabled',
        )

    def _upstairs_fixed_reference_target_yaw(self) -> float:
        return self._downstream_route_adjusted_yaw(
            self.upstairs_tag_fixed_reference_yaw,
            'downstream_route_yaw_upstairs_enabled',
        )

    def _limit_bar_pre_duck_target_yaw(self) -> float:
        if (
            getattr(
                self,
                'limit_bar_pre_duck_yaw_reference_mode',
                'route_zero_plus_offset',
            )
            == 'base_yaw'
        ):
            base_yaw = getattr(
                self,
                'limit_bar_lateral_shift_base_yaw',
                None,
            )
            if base_yaw is not None:
                return self._normalize_angle(float(base_yaw))
            duck_yaw = getattr(self, 'duck_reference_yaw', None)
            if duck_yaw is not None:
                return self._normalize_angle(float(duck_yaw))
        return self._downstream_route_adjusted_yaw(
            self.limit_bar_pre_duck_global_yaw_rad,
            'downstream_route_yaw_limit_bar_pre_duck_enabled',
        )

    def _limit_bar_pre_duck_yaw_source(self) -> str:
        if (
            getattr(
                self,
                'limit_bar_pre_duck_yaw_reference_mode',
                'route_zero_plus_offset',
            )
            == 'base_yaw'
            and (
                getattr(self, 'limit_bar_lateral_shift_base_yaw', None)
                is not None
                or getattr(self, 'duck_reference_yaw', None) is not None
            )
        ):
            return 'base_yaw'
        if self._downstream_route_yaw_reference_is_fresh():
            return 'route_zero_plus_offset'
        return 'absolute_offset_fallback'

    def _hurdle_post_align_target_yaw(self) -> float:
        if self.hurdle_post_align_use_startup_yaw and self.startup_pose_yaw is not None:
            return self.startup_pose_yaw
        return self._downstream_route_adjusted_yaw(
            self.hurdle_post_align_target_yaw,
            'downstream_route_yaw_hurdle_enabled',
        )

    def _post_stairs_slope_entry_yaw_target(self) -> float:
        if self.slope_align_reference_yaw is not None:
            return self.slope_align_reference_yaw
        return self._slope_align_target_yaw()

    def _search_target_yaw(self) -> Optional[float]:
        if self.search_reference_yaw is not None:
            return self.search_reference_yaw
        if self.search_yaw_use_startup_yaw:
            return self.startup_pose_yaw
        return self.search_yaw_target

    def _classify_slope_retry_rezero_branch(self, now: float) -> Optional[str]:
        if self._slope_ready(now):
            return 'slope'
        if self._slope_roll_ready(now) or self._post_stairs_slope_roll_priority_ready(now):
            return 'slope_roll'
        if self._slope_fallback_ready(now):
            return 'slope_fallback'
        return None

    def _classify_search_branch(self, now: float) -> Optional[str]:
        ready_branches = []
        if self._sandpit_ready(now):
            return 'sandpit'
        if self._sandpit_pre_align_ready(now):
            return 'sandpit_pre_align'
        sandpit_apriltag_priority_ready = self._sandpit_apriltag_priority_ready(now)
        slope_roll_ready = self._slope_roll_ready(now)
        post_stairs_slope_roll_ready = self._post_stairs_slope_roll_priority_ready(now)
        slope_ready = self._slope_ready(now)
        upstairs_ready = self._upstairs_ready(now)
        upstairs_slope_guard_ready = False
        if not upstairs_ready:
            upstairs_slope_guard_ready = (
                self._upstairs_slope_guard_ready(now)
                or self._upstairs_tag_detection_guard_ready(now)
            )
        if not slope_ready:
            self.slope_upstairs_wait_start_sec = 0.0
        if slope_ready and self._slope_waiting_for_upstairs_nearest(now):
            slope_ready = False
        if slope_ready and upstairs_slope_guard_ready:
            slope_ready = False
        limit_bar_visual_ready = not self.limit_bar_completed and self._limit_bar_ready(now)
        limit_bar_tag_ready = not self.limit_bar_completed and self._limit_bar_tag_ready(now)
        limit_bar_ready = limit_bar_visual_ready or limit_bar_tag_ready
        limit_bar_tag_slope_guard_ready = (
            not self.limit_bar_completed
            and not limit_bar_ready
            and self._limit_bar_tag_slope_guard_ready(now)
        )
        if slope_ready and limit_bar_tag_slope_guard_ready:
            slope_ready = False
        hurdle_tag_ready = not self.hurdle_completed and self._hurdle_tag_ready(now)
        hurdle_visual_ready = (
            not self.hurdle_completed
            and not hurdle_tag_ready
            and self._hurdle_ready(now)
        )
        hurdle_ready = hurdle_tag_ready or hurdle_visual_ready
        hurdle_tag_slope_guard_ready = (
            not self.hurdle_completed
            and not hurdle_ready
            and (
                self._hurdle_tag_slope_guard_ready(now)
                or self._hurdle_tag_detection_guard_ready(now)
            )
        )
        if slope_ready and hurdle_tag_slope_guard_ready:
            slope_ready = False
        if slope_ready and self._post_stairs_slope_priority_active() and not upstairs_ready:
            return 'slope'
        pole_ready = not self.pole_completed and self._pole_ready(now)
        pole_apriltag_ready = not self.pole_completed and self._pole_apriltag_ready(now)
        if pole_apriltag_ready:
            return 'pole'
        if upstairs_ready:
            return 'upstairs'
        if (
            slope_ready
            and not limit_bar_ready
            and self._limit_bar_nearest_preempts_slope_ready(now)
        ):
            limit_bar_ready = True
        if slope_ready:
            ready_branches.append(('slope', self.slope_detection.distance_m))
        sandpit_tag_guard_ready = (
            self._sandpit_tag_detection_guard_ready(now)
            or sandpit_apriltag_priority_ready
        )
        if sandpit_tag_guard_ready and not self.hurdle_completed:
            recent_sandpit_samples = self._recent_sandpit_tag_samples(now)
            sandpit_distance = self._usable_distance(self.sandpit_detection.distance_m)
            if len(recent_sandpit_samples) < self.sandpit_tag_min_samples:
                sandpit_wait_reason = (
                    f'样本不足 {len(recent_sandpit_samples)}/{self.sandpit_tag_min_samples}'
                )
            elif sandpit_distance is None:
                sandpit_wait_reason = '距离无效'
            elif sandpit_distance > self.sandpit_trigger_distance_m:
                sandpit_wait_reason = '未到触发线'
            else:
                sandpit_wait_reason = '等待触发稳定'
            self._publish_feedback(
                f'沙坑 AprilTag ID{self.sandpit_tag_target_id} 已检测但暂未进入分支'
                f'({sandpit_wait_reason})，暂时抑制高墙误触发: '
                f'ids={self.sandpit_tag_detection_ids_text or "none"}, '
                f'tf_samples={len(recent_sandpit_samples)}/{self.sandpit_tag_min_samples}, '
                f'tag_dist={self._distance_text(self.sandpit_detection.distance_m)}, '
                f'trigger={self.sandpit_trigger_distance_m:.3f}m, '
                f'priority={self.sandpit_apriltag_priority_distance_m:.3f}m, '
                f'hurdle_dist={self._distance_text(self.hurdle_detection.distance_m)}。'
            )
        if (
            not self.hurdle_completed
            and not sandpit_tag_guard_ready
            and hurdle_ready
        ):
            hurdle_ready_distance_m = (
                self.hurdle_detection.distance_m
                if hurdle_tag_ready
                else self.hurdle_visual_detection.distance_m
            )
            ready_branches.append(('hurdle', hurdle_ready_distance_m))
        if limit_bar_ready:
            ready_branches.append(('limit_bar', self.limit_detection.distance_m))
        if pole_ready:
            ready_branches.append(('pole', self.pole_detection.distance_m))
        if upstairs_ready:
            ready_branches.append(('upstairs', self.upstairs_detection.distance_m))

        # Prefer discrete post-stair obstacles over the slope background.
        # When the low bar / hurdle / pole is in front of the ramp, the green
        # ramp can still be visible behind it and should not win arbitration.
        non_slope_ready_branches = [
            (branch, distance_m)
            for branch, distance_m in ready_branches
            if branch != 'slope'
        ]
        # 2026-05-17: In the low-bar scene, floor yellow/orange guide lines can
        # occasionally satisfy the orange-pole detector and look closer than the
        # actual low bar. A confirmed low-bar trigger is more specific and must
        # not be stolen by the pole branch.
        #
        # 2026-06-26: When the pole itself is already close and stable, it is
        # more reliable than a wide low-bar component candidate in the background.
        if (
            limit_bar_ready
            and pole_ready
            and self._pole_near_preempts_limit_bar_ready(now)
        ):
            return 'pole'
        for branch, _ in non_slope_ready_branches:
            if branch == 'limit_bar':
                return 'limit_bar'

        nearest_branch = self._nearest_ready_branch(non_slope_ready_branches)
        if nearest_branch is None:
            nearest_branch = self._nearest_ready_branch(ready_branches)
        if nearest_branch is not None:
            return nearest_branch

        # If a trigger arrives before its nearest-distance message, keep a
        # deterministic branch choice for that control tick.
        if non_slope_ready_branches:
            return non_slope_ready_branches[0][0]
        if ready_branches:
            return ready_branches[0][0]

        if slope_roll_ready or post_stairs_slope_roll_ready:
            return 'slope_roll'

        if (
            not limit_bar_tag_slope_guard_ready
            and not hurdle_tag_slope_guard_ready
            and not self.slope_branch_completed
            and self._slope_fallback_ready(now)
        ):
            return 'slope_fallback'
        return None

    def _slope_roll_detection_context_active(self) -> bool:
        if self.slope_branch_completed:
            return False
        if self.state == self.STATE_POST_STAIRS_SLOPE_CONFIRM:
            return True
        if self.state != self.STATE_SEARCH:
            return False
        if not self.slope_roll_detection_post_stairs_only:
            return True
        return self.search_reason.startswith('post_stairs')

    def _slope_roll_ready(self, now: float) -> bool:
        if (
            not self.slope_roll_detection_enabled
            or not self._slope_roll_detection_context_active()
            or self.slope_roll_detection_rad <= 0.0
        ):
            self.slope_roll_detection_start_sec = 0.0
            return False
        if not self._pose_is_fresh(now) or self.pose_roll is None:
            self.slope_roll_detection_start_sec = 0.0
            return False

        roll_value = abs(self.pose_roll) if self.slope_roll_detection_use_abs else self.pose_roll
        if self.slope_roll_detection_start_sec <= 0.0:
            # Give the discrete obstacle guards priority when no slope
            # candidate has been latched yet. Once the first over-threshold
            # sample starts the timer, keep that timer running through brief
            # roll dips so one noisy Odometry sample cannot restart detection.
            if (
                roll_value < self.slope_roll_detection_rad
                or self._upstairs_ready(now)
                or self._upstairs_slope_guard_ready(now)
                or self._upstairs_tag_detection_guard_ready(now)
            ):
                self.slope_roll_detection_start_sec = 0.0
                return False
            self.slope_roll_detection_start_sec = now
            self.last_slope_roll_detection_log_sec = 0.0
        held_sec = now - self.slope_roll_detection_start_sec
        if held_sec >= self.slope_roll_detection_hold_sec:
            self._publish_feedback(
                f'斜坡 roll 触发: first_sample>='
                f'{math.degrees(self.slope_roll_detection_rad):.1f}deg, '
                f'current_roll={math.degrees(self.pose_roll):.1f}deg, '
                f'hold={held_sec:.2f}s。'
            )
            return True

        if (
            self.last_slope_roll_detection_log_sec <= 0.0
            or (now - self.last_slope_roll_detection_log_sec) >= 0.25
        ):
            self.last_slope_roll_detection_log_sec = now
            self._publish_feedback(
                f'斜坡 roll 候选(首帧已锁存): '
                f'current_roll={math.degrees(self.pose_roll):.1f}deg, '
                f'hold={held_sec:.2f}/{self.slope_roll_detection_hold_sec:.2f}s。'
            )
        return False

    def _post_stairs_slope_priority_active(self) -> bool:
        if not self.post_stairs_slope_priority_enabled:
            return False
        if self.slope_branch_completed:
            return False
        if self.state == self.STATE_POST_STAIRS_SLOPE_CONFIRM:
            return True
        if self.state != self.STATE_SEARCH:
            return False
        return self.search_reason.startswith('post_stairs')

    def _post_stairs_slope_roll_priority_ready(self, now: float) -> bool:
        if (
            not self.post_stairs_slope_roll_priority_enabled
            or not self._post_stairs_slope_priority_active()
            or self.post_stairs_slope_roll_priority_rad <= 0.0
        ):
            self.post_stairs_slope_roll_priority_start_sec = 0.0
            return False
        if not self._pose_is_fresh(now) or self.pose_roll is None:
            self.post_stairs_slope_roll_priority_start_sec = 0.0
            return False
        if (
            self._upstairs_ready(now)
            or self._upstairs_slope_guard_ready(now)
            or self._upstairs_tag_detection_guard_ready(now)
        ):
            self.post_stairs_slope_roll_priority_start_sec = 0.0
            return False

        roll_value = (
            abs(self.pose_roll)
            if self.post_stairs_slope_roll_priority_use_abs
            else self.pose_roll
        )
        if roll_value < self.post_stairs_slope_roll_priority_rad:
            self.post_stairs_slope_roll_priority_start_sec = 0.0
            return False

        if self.post_stairs_slope_roll_priority_start_sec <= 0.0:
            self.post_stairs_slope_roll_priority_start_sec = now
            self.last_post_stairs_slope_roll_priority_log_sec = 0.0
        held_sec = now - self.post_stairs_slope_roll_priority_start_sec
        if held_sec >= self.post_stairs_slope_roll_priority_hold_sec:
            self._publish_feedback(
                f'楼梯后搜索检测到上坡姿态: roll={math.degrees(self.pose_roll):.1f}deg '
                f'>= {math.degrees(self.post_stairs_slope_roll_priority_rad):.1f}deg, '
                f'hold={held_sec:.2f}s，优先进入斜坡分支。'
            )
            return True

        if (
            self.last_post_stairs_slope_roll_priority_log_sec <= 0.0
            or (now - self.last_post_stairs_slope_roll_priority_log_sec) >= 0.25
        ):
            self.last_post_stairs_slope_roll_priority_log_sec = now
            self._publish_feedback(
                f'楼梯后搜索检测到疑似上坡姿态: roll={math.degrees(self.pose_roll):.1f}deg, '
                f'hold={held_sec:.2f}/{self.post_stairs_slope_roll_priority_hold_sec:.2f}s。'
            )
        return False

    def _nearest_ready_branch(self, ready_branches) -> Optional[str]:
        nearest_branch = None
        nearest_distance = None
        for branch, distance_m in ready_branches:
            distance = self._usable_distance(distance_m)
            if distance is None:
                continue
            if nearest_distance is None or distance < nearest_distance:
                nearest_branch = branch
                nearest_distance = distance
        return nearest_branch

    def _sandpit_pre_align_ready(self, now: float) -> bool:
        if not self.sandpit_pre_align_enabled:
            return False
        if not self.sandpit_bypass_enabled:
            return False
        if self.sandpit_bypass_complete_once and self.sandpit_completed:
            return False
        recent_samples = self._recent_sandpit_tag_samples(now)
        if len(recent_samples) < self.sandpit_pre_align_min_samples:
            return False
        distance = self._median([sample.distance_m for sample in recent_samples])
        lateral = self._median([sample.x_m for sample in recent_samples])
        self.sandpit_detection.distance_m = distance
        self.sandpit_detection.lateral_m = lateral
        self.sandpit_detection.nearest_stamp_sec = recent_samples[-1].received_stamp_sec
        if not self.sandpit_detection.is_fresh(now, self.sandpit_trigger_stale_timeout_sec):
            return False
        usable_distance = self._usable_distance(distance)
        if usable_distance is None:
            return False
        if (
            usable_distance > self.sandpit_pre_align_start_distance_m
            or usable_distance <= self.sandpit_pre_align_handoff_distance_m
        ):
            return False

        tag_x_error_m = lateral - self.sandpit_pre_align_target_tag_x_m
        lateral_needed = self._sandpit_pre_align_lateral_needed(tag_x_error_m)
        pose_metric, _, pose_raw_error_rad, _ = self._sandpit_pre_align_pose_error(
            recent_samples[-1]
        )
        yaw_needed = self._sandpit_pre_align_yaw_needed(pose_raw_error_rad)
        if lateral_needed or yaw_needed:
            return True

        if (
            self.sandpit_pre_align_log_interval_sec > 0.0
            and (now - self.sandpit_pre_align_last_log_sec)
            >= self.sandpit_pre_align_log_interval_sec
        ):
            self.sandpit_pre_align_last_log_sec = now
            self._publish_feedback(
                f'沙坑 ID{self.sandpit_tag_target_id} 预修正跳过: '
                f'dist={usable_distance:.3f}m, tag_x_err={tag_x_error_m:+.3f}m '
                f'< {self.sandpit_pre_align_lateral_skip_below_m:.3f}m, '
                f'{pose_metric}_err={math.degrees(pose_raw_error_rad):+.1f}deg '
                f'<= {math.degrees(self.sandpit_pre_align_yaw_tolerance_rad):.1f}deg。'
            )
        return False

    def _sandpit_ready(self, now: float) -> bool:
        if not self.sandpit_bypass_enabled:
            return False
        if self.sandpit_bypass_complete_once and self.sandpit_completed:
            return False
        recent_samples = self._recent_sandpit_tag_samples(now)
        if len(recent_samples) < self.sandpit_tag_min_samples:
            self._update_detection_trigger(self.sandpit_detection, False)
            return False
        distance = self._median([sample.distance_m for sample in recent_samples])
        self.sandpit_detection.distance_m = distance
        self.sandpit_detection.lateral_m = self._median(
            [sample.x_m for sample in recent_samples]
        )
        self.sandpit_detection.nearest_stamp_sec = recent_samples[-1].received_stamp_sec
        if not self.sandpit_detection.is_fresh(now, self.sandpit_trigger_stale_timeout_sec):
            self._update_detection_trigger(self.sandpit_detection, False)
            return False
        distance = self._usable_distance(distance)
        if distance is None or distance > self.sandpit_trigger_distance_m:
            self._update_detection_trigger(self.sandpit_detection, False)
            return False
        self._update_detection_trigger(self.sandpit_detection, True)
        if self.sandpit_trigger_min_duration_sec <= 0.0:
            return True
        if self.sandpit_detection.trigger_start_stamp_sec <= 0.0:
            return False
        return (now - self.sandpit_detection.trigger_start_stamp_sec) >= (
            self.sandpit_trigger_min_duration_sec
        )

    def _sandpit_apriltag_priority_ready(self, now: float) -> bool:
        if not self.sandpit_apriltag_priority_guard_enabled:
            return False
        if not self.sandpit_bypass_enabled:
            return False
        if self.sandpit_bypass_complete_once and self.sandpit_completed:
            return False
        recent_samples = self._recent_sandpit_tag_samples(now)
        if not recent_samples:
            return False
        stable_distance_m = self._median(
            [sample.distance_m for sample in recent_samples]
        )
        self.sandpit_detection.distance_m = stable_distance_m
        self.sandpit_detection.lateral_m = self._median(
            [sample.x_m for sample in recent_samples]
        )
        self.sandpit_detection.nearest_stamp_sec = recent_samples[-1].received_stamp_sec
        distance = self._usable_distance(self.sandpit_detection.distance_m)
        if (
            distance is not None
            and self.sandpit_detection.is_fresh(now, self.sandpit_trigger_stale_timeout_sec)
            and distance <= self.sandpit_apriltag_priority_distance_m
        ):
            self.sandpit_priority_guard_stamp_sec = now
            return True
        if self.sandpit_priority_guard_stamp_sec <= 0.0:
            return False
        return (
            now - self.sandpit_priority_guard_stamp_sec
        ) <= self.sandpit_apriltag_priority_guard_hold_sec

    def _sandpit_tag_detection_guard_ready(self, now: float) -> bool:
        if not self.sandpit_apriltag_priority_guard_enabled:
            return False
        if not self.sandpit_bypass_enabled:
            return False
        if self.sandpit_bypass_complete_once and self.sandpit_completed:
            return False
        if self.sandpit_tag_detection_guard_sec <= 0.0:
            return False
        if self.sandpit_tag_detection_stamp_sec <= 0.0:
            return False
        return (now - self.sandpit_tag_detection_stamp_sec) <= self.sandpit_tag_detection_guard_sec

    def _clear_pole_apriltag_handoff(self) -> None:
        self.pole_apriltag_handoff_active = False
        self.pole_apriltag_handoff_stamp_sec = 0.0
        self.pole_apriltag_handoff_start_x = None
        self.pole_apriltag_handoff_start_y = None
        self.pole_apriltag_handoff_start_yaw = None
        self.pole_apriltag_handoff_tag_odom_x = None
        self.pole_apriltag_handoff_tag_odom_y = None
        self.pole_apriltag_handoff_distance_m = 0.0
        self.pole_apriltag_handoff_lateral_m = 0.0

    def _lock_pole_apriltag_handoff(
        self,
        now: float,
        distance_m: float,
        lateral_m: float,
    ) -> bool:
        if not self.pole_apriltag_odom_handoff_enabled:
            return False
        if self.pole_apriltag_trigger_distance_m <= 0.0:
            return False
        if (
            not self._pose_is_fresh(now)
            or self.pose_x is None
            or self.pose_y is None
            or self.pose_yaw is None
            or not math.isfinite(distance_m)
            or not math.isfinite(lateral_m)
            or distance_m <= 1e-3
        ):
            return False
        if (
            self.pole_apriltag_odom_handoff_max_start_distance_m > 0.0
            and distance_m
            > self.pole_apriltag_odom_handoff_max_start_distance_m
        ):
            return False
        self.pole_apriltag_handoff_active = True
        self.pole_apriltag_handoff_stamp_sec = now
        self.pole_apriltag_handoff_start_x = self.pose_x
        self.pole_apriltag_handoff_start_y = self.pose_y
        self.pole_apriltag_handoff_start_yaw = self.pose_yaw
        forward_x, forward_y = self._odom_forward_unit(self.pose_yaw)
        right_x, right_y = self._odom_right_unit(self.pose_yaw)
        self.pole_apriltag_handoff_tag_odom_x = (
            self.pose_x + distance_m * forward_x + lateral_m * right_x
        )
        self.pole_apriltag_handoff_tag_odom_y = (
            self.pose_y + distance_m * forward_y + lateral_m * right_y
        )
        self.pole_apriltag_handoff_distance_m = distance_m
        self.pole_apriltag_handoff_lateral_m = lateral_m
        return True

    def _pole_apriltag_handoff_progress(
        self,
        now: float,
    ) -> tuple[bool, float, float]:
        if not self.pole_apriltag_handoff_active:
            return False, 0.0, 0.0
        if not self.pole_apriltag_odom_handoff_enabled:
            self._clear_pole_apriltag_handoff()
            return False, 0.0, 0.0
        if (
            self.pole_apriltag_odom_handoff_timeout_sec > 0.0
            and (now - self.pole_apriltag_handoff_stamp_sec)
            > self.pole_apriltag_odom_handoff_timeout_sec
        ):
            self._clear_pole_apriltag_handoff()
            return False, 0.0, 0.0
        if (
            not self._pose_is_fresh(now)
            or self.pose_x is None
            or self.pose_y is None
            or self.pole_apriltag_handoff_start_x is None
            or self.pole_apriltag_handoff_start_y is None
            or self.pole_apriltag_handoff_start_yaw is None
            or self.pole_apriltag_handoff_tag_odom_x is None
            or self.pole_apriltag_handoff_tag_odom_y is None
        ):
            return False, 0.0, 0.0
        tag_dx = self.pole_apriltag_handoff_tag_odom_x - self.pose_x
        tag_dy = self.pole_apriltag_handoff_tag_odom_y - self.pose_y
        estimated_distance_m = math.hypot(tag_dx, tag_dy)
        initial_range_m = math.hypot(
            self.pole_apriltag_handoff_distance_m,
            self.pole_apriltag_handoff_lateral_m,
        )
        approach_m = initial_range_m - estimated_distance_m
        if not math.isfinite(approach_m):
            return False, 0.0, 0.0
        if not math.isfinite(estimated_distance_m):
            return False, 0.0, 0.0
        return True, max(0.0, approach_m), max(1e-3, estimated_distance_m)

    def _pole_apriltag_handoff_ready(self, now: float) -> bool:
        ok, approach_m, estimated_distance_m = (
            self._pole_apriltag_handoff_progress(now)
        )
        if not ok:
            return False
        if estimated_distance_m > self.pole_apriltag_trigger_distance_m:
            if (now - self.last_pole_apriltag_handoff_log_sec) >= 0.50:
                self.last_pole_apriltag_handoff_log_sec = now
                self._publish_feedback(
                    f'绕杆 AprilTag 远距锁存接力等待触发线: '
                    f'latched_dist={self.pole_apriltag_handoff_distance_m:.3f}m, '
                    f'odom_approach={approach_m:.3f}m, '
                    f'est_dist={estimated_distance_m:.3f}m > '
                    f'trigger={self.pole_apriltag_trigger_distance_m:.3f}m, '
                    f'latched_lateral={self.pole_apriltag_handoff_lateral_m:+.3f}m, '
                    f'age={now - self.pole_apriltag_handoff_stamp_sec:.2f}/'
                    f'{self.pole_apriltag_odom_handoff_timeout_sec:.2f}s。'
                )
            return False
        self.pole_apriltag_detection.distance_m = estimated_distance_m
        self.pole_apriltag_detection.lateral_m = (
            self.pole_apriltag_handoff_lateral_m
        )
        self.pole_apriltag_ready_source = 'odom_handoff'
        if (now - self.last_pole_apriltag_handoff_log_sec) >= 0.20:
            self.last_pole_apriltag_handoff_log_sec = now
            self._publish_feedback(
                f'绕杆 AprilTag 分支触发条件满足: '
                f'reason=odom_handoff_after_tag_lost, '
                f'latched_dist={self.pole_apriltag_handoff_distance_m:.3f}m, '
                f'odom_approach={approach_m:.3f}m, '
                f'est_dist={estimated_distance_m:.3f}m <= '
                f'trigger={self.pole_apriltag_trigger_distance_m:.3f}m, '
                f'latched_lateral={self.pole_apriltag_handoff_lateral_m:+.3f}m。'
            )
        return True

    def _pole_apriltag_ready(self, now: float) -> bool:
        self.pole_apriltag_ready_source = ''
        if not self.pole_apriltag_trigger_enabled:
            return False
        if (
            self.search_reason == 'slope_done'
            and (now - self.search_enter_time) < self.pole_after_slope_search_delay_sec
        ):
            return False
        fresh_detection = (
            self.pole_apriltag_detection.triggered
            and self.pole_apriltag_detection.is_fresh(
                now,
                self.pole_apriltag_stale_timeout_sec,
            )
        )
        if fresh_detection:
            measurement_ready = True
            if (
                self.pole_apriltag_require_tf
                or self.pole_apriltag_trigger_distance_m > 0.0
            ):
                distance = self._usable_distance(
                    self.pole_apriltag_detection.distance_m
                )
                measurement_ready = (
                    distance is not None
                    and (
                        now - self.pole_apriltag_detection.nearest_stamp_sec
                    ) <= self.pole_apriltag_stale_timeout_sec
                    and (
                        self.pole_apriltag_trigger_distance_m <= 0.0
                        or distance <= self.pole_apriltag_trigger_distance_m
                    )
                )
            duration_ready = self.pole_apriltag_min_duration_sec <= 0.0
            if not duration_ready:
                duration_ready = (
                    self.pole_apriltag_detection.trigger_start_stamp_sec > 0.0
                    and (
                        now
                        - self.pole_apriltag_detection.trigger_start_stamp_sec
                    ) >= self.pole_apriltag_min_duration_sec
                )
            if measurement_ready and duration_ready:
                self.pole_apriltag_ready_source = 'fresh_tf'
                return True
        return self._pole_apriltag_handoff_ready(now)

    def _pole_apriltag_distance(self, translation) -> Optional[float]:
        x = float(translation.x)
        y = float(translation.y)
        z = float(translation.z)
        if not all(math.isfinite(value) for value in (x, y, z)):
            return None
        if self.pole_apriltag_distance_axis == 'x':
            return abs(x)
        if self.pole_apriltag_distance_axis == 'y':
            return abs(y)
        if self.pole_apriltag_distance_axis == 'z':
            return abs(z)
        return math.sqrt(x * x + y * y + z * z)

    @staticmethod
    def _usable_distance(distance_m: Optional[float]) -> Optional[float]:
        if distance_m is None:
            return None
        distance = float(distance_m)
        if not math.isfinite(distance) or distance < 0.0:
            return None
        return distance

    def _slope_ready(self, now: float) -> bool:
        if not self.slope_visual_detection_enabled:
            return False
        if self.slope_branch_completed:
            return False
        if not self.slope_detection.triggered:
            return False
        if not self.slope_detection.is_fresh(now, self.detection_stale_timeout_sec):
            return False
        if self.slope_trigger_min_duration_sec <= 0.0:
            return True
        if self.slope_detection.trigger_start_stamp_sec <= 0.0:
            return False
        return (now - self.slope_detection.trigger_start_stamp_sec) >= (
            self.slope_trigger_min_duration_sec
        )

    def _slope_fallback_ready(self, now: float) -> bool:
        if not self.slope_fallback_enabled:
            return False
        if (now - self.search_enter_time) < self.slope_fallback_timeout_sec:
            return False
        if self.pose_x is None or self.pose_y is None:
            return False
        if self.search_start_x is None or self.search_start_y is None:
            return False
        travel_m = math.hypot(
            self.pose_x - self.search_start_x,
            self.pose_y - self.search_start_y,
        )
        if travel_m < self.slope_fallback_min_distance_m:
            return False
        return True

    def _hurdle_ready(self, now: float) -> bool:
        if not self.hurdle_visual_trigger_enabled:
            return False
        if not self.hurdle_visual_detection.triggered:
            return False
        if not self.hurdle_visual_detection.is_fresh(
            now,
            self.detection_stale_timeout_sec,
        ):
            return False
        if self.hurdle_trigger_min_duration_sec <= 0.0:
            return True
        if self.hurdle_visual_detection.trigger_start_stamp_sec <= 0.0:
            return False
        return (now - self.hurdle_visual_detection.trigger_start_stamp_sec) >= (
            self.hurdle_trigger_min_duration_sec
        )

    def _hurdle_tag_ready(self, now: float) -> bool:
        if not self.hurdle_tag_trigger_enabled:
            return False
        if self.hurdle_completed:
            return False
        recent_samples = self._recent_hurdle_tag_samples(now)
        if len(recent_samples) < self.hurdle_tag_min_samples:
            if self._hurdle_tag_handoff_ready(now):
                return True
            if (now - self.last_hurdle_tag_wait_log_sec) >= 0.50:
                self.last_hurdle_tag_wait_log_sec = now
                latest_age_text = 'none'
                if self.hurdle_tag_samples:
                    latest_age_text = f'{now - self.hurdle_tag_samples[-1].received_stamp_sec:.2f}s'
                det_age_text = 'none'
                if self.hurdle_tag_detection_stamp_sec > 0.0:
                    det_age_text = f'{now - self.hurdle_tag_detection_stamp_sec:.2f}s'
                self._publish_feedback(
                    f'高墙 AprilTag 已检测但暂未进入分支: '
                    f'tf_samples={len(recent_samples)}/{self.hurdle_tag_min_samples}, '
                    f'latest_tf_age={latest_age_text}, det_age={det_age_text}, '
                    f'ids={self.hurdle_tag_detection_ids_text or "none"}, '
                    f'expected_tf={self.hurdle_tag_frame_id}->{self.hurdle_tag_child_frame_id}。'
                )
            return False
        tag_x = self._median([sample.x_m for sample in recent_samples])
        tag_z = self._median([sample.z_m for sample in recent_samples])
        center_x = tag_x + self.hurdle_tag_to_center_x_m
        self._lock_hurdle_tag_handoff(now, tag_x, tag_z, center_x)
        if self.hurdle_tag_trigger_distance_m > 0.0 and tag_z > self.hurdle_tag_trigger_distance_m:
            if (now - self.last_hurdle_tag_wait_log_sec) >= 0.50:
                self.last_hurdle_tag_wait_log_sec = now
                self._publish_feedback(
                    f'高墙 AprilTag 已锁定但距离未到触发线: '
                    f'samples={len(recent_samples)}, tag_x={tag_x:.3f}m, '
                    f'tag_z={tag_z:.3f}m > trigger={self.hurdle_tag_trigger_distance_m:.3f}m, '
                    f'center_x={center_x:.3f}m, '
                    f'target_id={self.hurdle_tag_target_tag_id}。'
                )
            return False
        self._apply_hurdle_tag_measurement(now, tag_x, tag_z, center_x)
        self._lock_hurdle_initial_tag(
            now,
            tag_x,
            tag_z,
            center_x,
            source='trigger',
        )
        self._clear_hurdle_tag_handoff()
        if (now - self.last_hurdle_tag_wait_log_sec) >= 0.50:
            self.last_hurdle_tag_wait_log_sec = now
            self._publish_feedback(
                f'高墙 AprilTag 分支触发条件满足: samples={len(recent_samples)}, '
                f'tag_x={tag_x:.3f}m, tag_z={tag_z:.3f}m, '
                f'trigger={self.hurdle_tag_trigger_distance_m:.3f}m, '
                f'center_x={center_x:.3f}m, '
                f'target_id={self.hurdle_tag_target_tag_id}。'
            )
        return True

    def _hurdle_tag_slope_guard_ready(self, now: float) -> bool:
        if not self.hurdle_tag_slope_guard_enabled:
            return False
        if not self.hurdle_tag_trigger_enabled:
            return False
        if self.hurdle_completed:
            return False
        if self.hurdle_tag_slope_guard_distance_m <= 0.0:
            return False
        recent_samples = self._recent_hurdle_tag_samples(now)
        if not recent_samples:
            return False
        tag_x = self._median([sample.x_m for sample in recent_samples])
        tag_z = self._median([sample.z_m for sample in recent_samples])
        if tag_z > self.hurdle_tag_slope_guard_distance_m:
            return False
        if (now - self.last_hurdle_tag_guard_log_sec) >= 0.50:
            self.last_hurdle_tag_guard_log_sec = now
            self._publish_feedback(
                f'高墙 AprilTag 保护斜坡抢占: '
                f'samples={len(recent_samples)}, tag_x={tag_x:.3f}m, '
                f'tag_z={tag_z:.3f}m, trigger={self.hurdle_tag_trigger_distance_m:.3f}m, '
                f'guard={self.hurdle_tag_slope_guard_distance_m:.3f}m, '
                f'center_x={tag_x + self.hurdle_tag_to_center_x_m:.3f}m，'
                f'继续搜索等待触发线，不进入斜坡 yaw 对齐。'
            )
        return True

    def _hurdle_tag_detection_guard_ready(self, now: float) -> bool:
        if not self.hurdle_tag_slope_guard_enabled:
            return False
        if not self.hurdle_tag_trigger_enabled:
            return False
        if self.hurdle_completed:
            return False
        if self.hurdle_tag_detection_stamp_sec <= 0.0:
            return False
        age = now - self.hurdle_tag_detection_stamp_sec
        if self.hurdle_tag_stale_timeout_sec > 0.0 and age > self.hurdle_tag_stale_timeout_sec:
            return False
        if (now - self.last_hurdle_tag_guard_log_sec) >= 0.50:
            self.last_hurdle_tag_guard_log_sec = now
            self._publish_feedback(
                f'高墙 AprilTag ID{self.hurdle_tag_target_tag_id} 已检测，'
                f'等待 TF/距离确认期间抑制斜坡抢占: '
                f'ids={self.hurdle_tag_detection_ids_text or "none"}, '
                f'det_age={age:.2f}s, '
                f'expected_tf={self.hurdle_tag_frame_id}->{self.hurdle_tag_child_frame_id}。'
            )
        return True

    def _clear_hurdle_tag_handoff(self) -> None:
        self.hurdle_tag_handoff_active = False
        self.hurdle_tag_handoff_stamp_sec = 0.0
        self.hurdle_tag_handoff_start_x = None
        self.hurdle_tag_handoff_start_y = None
        self.hurdle_tag_handoff_start_yaw = None
        self.hurdle_tag_handoff_tag_x_m = 0.0
        self.hurdle_tag_handoff_tag_z_m = 0.0
        self.hurdle_tag_handoff_center_x_m = 0.0

    def _clear_hurdle_locked_tag(self) -> None:
        self.hurdle_locked_tag_valid = False
        self.hurdle_locked_tag_stamp_sec = 0.0
        self.hurdle_locked_tag_x_m = 0.0
        self.hurdle_locked_tag_z_m = 0.0
        self.hurdle_locked_center_x_m = 0.0

    def _clear_limit_bar_locked_tag(self) -> None:
        self.limit_bar_locked_tag_valid = False
        self.limit_bar_locked_tag_stamp_sec = 0.0
        self.limit_bar_locked_tag_x_m = 0.0
        self.limit_bar_locked_tag_z_m = 0.0
        self.limit_bar_locked_center_x_m = 0.0
        self.limit_bar_locked_tag_odom_yaw = None
        self.limit_bar_locked_tag_sample_count = 0
        self.limit_bar_locked_tag_source = ''

    def _lock_limit_bar_initial_tag(
        self,
        now: float,
        tag_x: float,
        tag_z: float,
        center_x: float,
        *,
        sample_count: int,
        source: str,
    ) -> bool:
        if (
            not math.isfinite(tag_x)
            or not math.isfinite(tag_z)
            or not math.isfinite(center_x)
            or tag_z <= 1e-3
            or self.pose_yaw is None
        ):
            return False
        self.limit_bar_locked_tag_valid = True
        self.limit_bar_locked_tag_stamp_sec = now
        self.limit_bar_locked_tag_x_m = tag_x
        self.limit_bar_locked_tag_z_m = tag_z
        self.limit_bar_locked_center_x_m = center_x
        self.limit_bar_locked_tag_odom_yaw = self.pose_yaw
        self.limit_bar_locked_tag_sample_count = max(1, int(sample_count))
        self.limit_bar_locked_tag_source = source
        self._publish_feedback(
            f'限高杆几何初始锁定: source={source}, '
            f'samples={self.limit_bar_locked_tag_sample_count}, '
            f'tag_x={tag_x:.3f}m, tag_z={tag_z:.3f}m, '
            f'center_x={center_x:.3f}m, '
            f'odom_yaw={math.degrees(self.pose_yaw):.1f}deg，'
            '后续仅按 yaw 坐标补偿，不再使用完成阶段最新单帧 Tag。'
        )
        return True

    def _select_limit_bar_geometry_lock_sample(
        self,
        recent_samples: List[LimitBarTagSample],
    ) -> tuple[LimitBarTagSample, int, str]:
        sample_count = min(
            len(recent_samples),
            self.limit_bar_tag_geometry_lock_sample_count,
        )
        window = recent_samples[-sample_count:]
        selected_sample = min(
            window,
            key=lambda sample: abs(
                sample.x_m + self.limit_bar_tag_to_center_x_m
            ),
        )
        return selected_sample, sample_count, 'min_abs_center_last_samples'

    def _limit_bar_locked_tag_for_yaw(
        self,
        target_yaw: float,
    ) -> Optional[tuple[float, float]]:
        if (
            not self.limit_bar_locked_tag_valid
            or self.limit_bar_locked_tag_odom_yaw is None
        ):
            return None
        yaw_delta = self._normalize_angle(
            target_yaw - self.limit_bar_locked_tag_odom_yaw
        )
        tag_x, tag_z = _rotate_optical_xz_for_yaw_delta(
            self.limit_bar_locked_tag_x_m,
            self.limit_bar_locked_tag_z_m,
            yaw_delta,
        )
        if not math.isfinite(tag_x) or not math.isfinite(tag_z) or tag_z <= 1e-3:
            return None
        return tag_x, tag_z

    def _lock_hurdle_initial_tag(
        self,
        now: float,
        tag_x: float,
        tag_z: float,
        center_x: float,
        *,
        source: str,
        force: bool = False,
    ) -> bool:
        if self.hurdle_locked_tag_valid and not force:
            return False
        if (
            not math.isfinite(tag_x)
            or not math.isfinite(tag_z)
            or not math.isfinite(center_x)
            or tag_z <= 1e-3
        ):
            return False
        self.hurdle_locked_tag_valid = True
        self.hurdle_locked_tag_stamp_sec = now
        self.hurdle_locked_tag_x_m = tag_x
        self.hurdle_locked_tag_z_m = tag_z
        self.hurdle_locked_center_x_m = center_x
        self._publish_feedback(
            f'高墙 AprilTag 初始锁定: source={source}, '
            f'tag_x={tag_x:.3f}m, tag_z={tag_z:.3f}m, '
            f'center_x={center_x:.3f}m，后续横向补偿不再重算 tag。'
        )
        return True

    def _hurdle_locked_or_latest_tag(
        self,
        now: float,
    ) -> Optional[tuple[float, float, float, int, str]]:
        if self.hurdle_locked_tag_valid:
            return (
                self.hurdle_locked_tag_x_m,
                self.hurdle_locked_tag_z_m,
                self.hurdle_locked_center_x_m,
                1,
                'initial_locked',
            )
        recent_samples = self._recent_hurdle_tag_samples(now)
        if len(recent_samples) < self.hurdle_tag_min_samples:
            return None
        tag_x = self._median([sample.x_m for sample in recent_samples])
        tag_z = self._median([sample.z_m for sample in recent_samples])
        center_x = tag_x + self.hurdle_tag_to_center_x_m
        return tag_x, tag_z, center_x, len(recent_samples), 'latest'

    def _lock_hurdle_tag_handoff(
        self,
        now: float,
        tag_x: float,
        tag_z: float,
        center_x: float,
    ) -> bool:
        if not self.hurdle_tag_odom_handoff_enabled:
            return False
        if self.hurdle_tag_trigger_distance_m <= 0.0:
            return False
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.pose_yaw is None
            or not math.isfinite(tag_x)
            or not math.isfinite(tag_z)
            or not math.isfinite(center_x)
            or tag_z <= 1e-3
        ):
            return False
        if (
            self.hurdle_tag_odom_handoff_max_start_distance_m > 0.0
            and tag_z > self.hurdle_tag_odom_handoff_max_start_distance_m
        ):
            return False
        self.hurdle_tag_handoff_active = True
        self.hurdle_tag_handoff_stamp_sec = now
        self.hurdle_tag_handoff_start_x = self.pose_x
        self.hurdle_tag_handoff_start_y = self.pose_y
        self.hurdle_tag_handoff_start_yaw = self.pose_yaw
        self.hurdle_tag_handoff_tag_x_m = tag_x
        self.hurdle_tag_handoff_tag_z_m = tag_z
        self.hurdle_tag_handoff_center_x_m = center_x
        return True

    def _hurdle_tag_handoff_progress(self, now: float) -> tuple[bool, float, float]:
        if not self.hurdle_tag_handoff_active:
            return False, 0.0, 0.0
        if not self.hurdle_tag_odom_handoff_enabled:
            self._clear_hurdle_tag_handoff()
            return False, 0.0, 0.0
        if (
            self.hurdle_tag_odom_handoff_timeout_sec > 0.0
            and (now - self.hurdle_tag_handoff_stamp_sec)
            > self.hurdle_tag_odom_handoff_timeout_sec
        ):
            self._clear_hurdle_tag_handoff()
            return False, 0.0, 0.0
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.hurdle_tag_handoff_start_x is None
            or self.hurdle_tag_handoff_start_y is None
            or self.hurdle_tag_handoff_start_yaw is None
        ):
            return False, 0.0, 0.0
        dx = self.pose_x - self.hurdle_tag_handoff_start_x
        dy = self.pose_y - self.hurdle_tag_handoff_start_y
        forward_x, forward_y = self._odom_forward_unit(
            self.hurdle_tag_handoff_start_yaw
        )
        along_m = dx * forward_x + dy * forward_y
        if not math.isfinite(along_m):
            return False, 0.0, 0.0
        along_m = max(0.0, along_m)
        estimated_tag_z = self.hurdle_tag_handoff_tag_z_m - along_m
        if not math.isfinite(estimated_tag_z):
            return False, 0.0, 0.0
        estimated_tag_z = max(1e-3, estimated_tag_z)
        return True, along_m, estimated_tag_z

    def _apply_hurdle_tag_measurement(
        self,
        now: float,
        tag_x: float,
        tag_z: float,
        center_x: float,
    ) -> None:
        self.hurdle_detection.distance_m = tag_z
        self.hurdle_detection.lateral_m = tag_x
        self.hurdle_detection.nearest_stamp_sec = now
        self.limit_bar_lateral_shift_center_x_m = center_x
        self.limit_bar_lateral_shift_center_z_m = tag_z

    def _hurdle_tag_handoff_ready(self, now: float) -> bool:
        ok, along_m, estimated_tag_z = self._hurdle_tag_handoff_progress(now)
        if not ok:
            return False
        if (
            self.hurdle_tag_trigger_distance_m > 0.0
            and estimated_tag_z > self.hurdle_tag_trigger_distance_m
        ):
            if (now - self.last_hurdle_tag_wait_log_sec) >= 0.50:
                self.last_hurdle_tag_wait_log_sec = now
                self._publish_feedback(
                    f'高墙 AprilTag 丢码接力等待触发线: '
                    f'reason=odom_handoff_distance_not_reached, '
                    f'latched_tag_x={self.hurdle_tag_handoff_tag_x_m:.3f}m, '
                    f'latched_tag_z={self.hurdle_tag_handoff_tag_z_m:.3f}m, '
                    f'odom_forward={along_m:.3f}m, '
                    f'est_tag_z={estimated_tag_z:.3f}m > '
                    f'trigger={self.hurdle_tag_trigger_distance_m:.3f}m, '
                    f'center_x={self.hurdle_tag_handoff_center_x_m:.3f}m。'
                )
            return False
        self._apply_hurdle_tag_measurement(
            now,
            self.hurdle_tag_handoff_tag_x_m,
            estimated_tag_z,
            self.hurdle_tag_handoff_center_x_m,
        )
        self._lock_hurdle_initial_tag(
            now,
            self.hurdle_tag_handoff_tag_x_m,
            estimated_tag_z,
            self.hurdle_tag_handoff_center_x_m,
            source='odom_handoff_trigger',
        )
        if (now - self.last_hurdle_tag_wait_log_sec) >= 0.20:
            self.last_hurdle_tag_wait_log_sec = now
            inside_by = (
                self.hurdle_tag_trigger_distance_m - estimated_tag_z
                if self.hurdle_tag_trigger_distance_m > 0.0
                else 0.0
            )
            self._publish_feedback(
                f'高墙 AprilTag 分支触发条件满足: '
                f'reason=odom_handoff_after_tag_lost, '
                f'latched_tag_x={self.hurdle_tag_handoff_tag_x_m:.3f}m, '
                f'latched_tag_z={self.hurdle_tag_handoff_tag_z_m:.3f}m, '
                f'odom_forward={along_m:.3f}m, '
                f'est_tag_z={estimated_tag_z:.3f}m, '
                f'trigger_distance={self.hurdle_tag_trigger_distance_m:.3f}m, '
                f'inside_by={inside_by:.3f}m, '
                f'center_x={self.hurdle_tag_handoff_center_x_m:.3f}m, '
                f'tag_to_center={self.hurdle_tag_to_center_x_m:+.3f}m。'
            )
        return True

    def _limit_bar_ready(self, now: float) -> bool:
        if not self.limit_bar_visual_trigger_enabled:
            return False
        if not self.limit_detection.triggered:
            return False
        if not self.limit_detection.is_fresh(now, self.detection_stale_timeout_sec):
            return False
        if self.limit_trigger_min_duration_sec <= 0.0:
            return True
        if self.limit_detection.trigger_start_stamp_sec <= 0.0:
            return False
        return (now - self.limit_detection.trigger_start_stamp_sec) >= (
            self.limit_trigger_min_duration_sec
        )

    def _clear_limit_bar_tag_handoff(self) -> None:
        self.limit_bar_tag_handoff_active = False
        self.limit_bar_tag_handoff_stamp_sec = 0.0
        self.limit_bar_tag_handoff_start_x = None
        self.limit_bar_tag_handoff_start_y = None
        self.limit_bar_tag_handoff_start_yaw = None
        self.limit_bar_tag_handoff_tag_x_m = 0.0
        self.limit_bar_tag_handoff_tag_z_m = 0.0
        self.limit_bar_tag_handoff_center_x_m = 0.0
        self.limit_bar_tag_handoff_source = ''
        self.limit_bar_tag_handoff_pose_roll_rad = 0.0
        self.limit_bar_tag_handoff_pose_pitch_rad = 0.0

    def _lock_limit_bar_tag_handoff(
        self,
        now: float,
        tag_x: float,
        tag_z: float,
        center_x: float,
        *,
        source: str = 'search_fresh',
        require_flat_pose: bool = False,
    ) -> bool:
        if not self.limit_bar_tag_odom_handoff_enabled:
            return False
        if self.limit_bar_tag_trigger_distance_m <= 0.0:
            return False
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.pose_yaw is None
            or self.pose_roll is None
            or self.pose_pitch is None
            or not math.isfinite(tag_x)
            or not math.isfinite(tag_z)
            or not math.isfinite(center_x)
            or tag_z <= 1e-3
        ):
            return False
        if require_flat_pose:
            abs_roll = abs(self.pose_roll)
            abs_pitch = abs(self.pose_pitch)
            roll_limit = self.limit_bar_tag_presearch_handoff_max_abs_roll_rad
            pitch_limit = self.limit_bar_tag_presearch_handoff_max_abs_pitch_rad
            if (
                (roll_limit > 0.0 and abs_roll > roll_limit)
                or (pitch_limit > 0.0 and abs_pitch > pitch_limit)
            ):
                if (now - self.last_limit_bar_tag_presearch_handoff_log_sec) >= 0.50:
                    self.last_limit_bar_tag_presearch_handoff_log_sec = now
                    self._publish_feedback(
                        f'限高杆 AprilTag SEARCH 前锁存跳过: '
                        f'reason=pose_not_flat, tag_x={tag_x:.3f}m, tag_z={tag_z:.3f}m, '
                        f'roll={math.degrees(self.pose_roll):.1f}deg/'
                        f'{math.degrees(roll_limit):.1f}deg, '
                        f'pitch={math.degrees(self.pose_pitch):.1f}deg/'
                        f'{math.degrees(pitch_limit):.1f}deg。'
                    )
                return False
        if (
            self.limit_bar_tag_odom_handoff_max_start_distance_m > 0.0
            and tag_z > self.limit_bar_tag_odom_handoff_max_start_distance_m
        ):
            return False
        self.limit_bar_tag_handoff_active = True
        self.limit_bar_tag_handoff_stamp_sec = now
        self.limit_bar_tag_handoff_start_x = self.pose_x
        self.limit_bar_tag_handoff_start_y = self.pose_y
        self.limit_bar_tag_handoff_start_yaw = self.pose_yaw
        self.limit_bar_tag_handoff_tag_x_m = tag_x
        self.limit_bar_tag_handoff_tag_z_m = tag_z
        self.limit_bar_tag_handoff_center_x_m = center_x
        self.limit_bar_tag_handoff_source = source
        self.limit_bar_tag_handoff_pose_roll_rad = self.pose_roll
        self.limit_bar_tag_handoff_pose_pitch_rad = self.pose_pitch
        return True

    def _limit_bar_tag_handoff_progress(self, now: float) -> tuple[bool, float, float]:
        if not self.limit_bar_tag_handoff_active:
            return False, 0.0, 0.0
        if not self.limit_bar_tag_odom_handoff_enabled:
            self._clear_limit_bar_tag_handoff()
            return False, 0.0, 0.0
        if (
            self.limit_bar_tag_odom_handoff_timeout_sec > 0.0
            and (now - self.limit_bar_tag_handoff_stamp_sec)
            > self.limit_bar_tag_odom_handoff_timeout_sec
        ):
            self._clear_limit_bar_tag_handoff()
            return False, 0.0, 0.0
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.limit_bar_tag_handoff_start_x is None
            or self.limit_bar_tag_handoff_start_y is None
            or self.limit_bar_tag_handoff_start_yaw is None
        ):
            return False, 0.0, 0.0
        dx = self.pose_x - self.limit_bar_tag_handoff_start_x
        dy = self.pose_y - self.limit_bar_tag_handoff_start_y
        forward_x, forward_y = self._odom_forward_unit(
            self.limit_bar_tag_handoff_start_yaw
        )
        along_m = dx * forward_x + dy * forward_y
        if not math.isfinite(along_m):
            return False, 0.0, 0.0
        along_m = max(0.0, along_m)
        estimated_tag_z = self.limit_bar_tag_handoff_tag_z_m - along_m
        if not math.isfinite(estimated_tag_z):
            return False, 0.0, 0.0
        estimated_tag_z = max(1e-3, estimated_tag_z)
        return True, along_m, estimated_tag_z

    def _apply_limit_bar_tag_measurement(
        self,
        now: float,
        tag_x: float,
        tag_z: float,
        center_x: float,
    ) -> None:
        self.limit_detection.distance_m = tag_z
        self.limit_detection.lateral_m = tag_x
        self.limit_detection.nearest_stamp_sec = now
        self.limit_bar_lateral_shift_center_x_m = center_x
        self.limit_bar_lateral_shift_center_z_m = tag_z

    def _limit_bar_tag_handoff_ready(self, now: float) -> bool:
        ok, along_m, estimated_tag_z = self._limit_bar_tag_handoff_progress(now)
        if not ok:
            return False
        if (
            self.limit_bar_tag_trigger_distance_m > 0.0
            and estimated_tag_z > self.limit_bar_tag_trigger_distance_m
        ):
            if (now - self.last_limit_bar_tag_wait_log_sec) >= 0.50:
                self.last_limit_bar_tag_wait_log_sec = now
                self._publish_feedback(
                    f'限高杆 AprilTag 丢码接力等待触发线: '
                    f'reason=odom_handoff_distance_not_reached, '
                    f'source={self.limit_bar_tag_handoff_source or "unknown"}, '
                    f'latched_tag_x={self.limit_bar_tag_handoff_tag_x_m:.3f}m, '
                    f'latched_tag_z={self.limit_bar_tag_handoff_tag_z_m:.3f}m, '
                    f'odom_forward={along_m:.3f}m, '
                    f'est_tag_z={estimated_tag_z:.3f}m > '
                    f'trigger={self.limit_bar_tag_trigger_distance_m:.3f}m, '
                    f'center_x={self.limit_bar_tag_handoff_center_x_m:.3f}m, '
                    f'lock_roll={math.degrees(self.limit_bar_tag_handoff_pose_roll_rad):.1f}deg, '
                    f'lock_pitch={math.degrees(self.limit_bar_tag_handoff_pose_pitch_rad):.1f}deg。'
                )
            return False
        self._apply_limit_bar_tag_measurement(
            now,
            self.limit_bar_tag_handoff_tag_x_m,
            estimated_tag_z,
            self.limit_bar_tag_handoff_center_x_m,
        )
        if (now - self.last_limit_bar_tag_wait_log_sec) >= 0.20:
            self.last_limit_bar_tag_wait_log_sec = now
            inside_by = (
                self.limit_bar_tag_trigger_distance_m - estimated_tag_z
                if self.limit_bar_tag_trigger_distance_m > 0.0
                else 0.0
            )
            self._publish_feedback(
                f'限高杆 AprilTag 分支触发条件满足: '
                f'reason=odom_handoff_after_tag_lost, '
                f'source={self.limit_bar_tag_handoff_source or "unknown"}, '
                f'latched_tag_x={self.limit_bar_tag_handoff_tag_x_m:.3f}m, '
                f'latched_tag_z={self.limit_bar_tag_handoff_tag_z_m:.3f}m, '
                f'odom_forward={along_m:.3f}m, '
                f'est_tag_z={estimated_tag_z:.3f}m, '
                f'trigger_distance={self.limit_bar_tag_trigger_distance_m:.3f}m, '
                f'inside_by={inside_by:.3f}m, '
                f'center_x={self.limit_bar_tag_handoff_center_x_m:.3f}m, '
                f'tag_to_center={self.limit_bar_tag_to_center_x_m:+.3f}m, '
                f'lock_roll={math.degrees(self.limit_bar_tag_handoff_pose_roll_rad):.1f}deg, '
                f'lock_pitch={math.degrees(self.limit_bar_tag_handoff_pose_pitch_rad):.1f}deg。'
            )
        return True

    def _clear_upstairs_tag_handoff(self) -> None:
        self.upstairs_tag_handoff_active = False
        self.upstairs_tag_handoff_stamp_sec = 0.0
        self.upstairs_tag_handoff_start_x = None
        self.upstairs_tag_handoff_start_y = None
        self.upstairs_tag_handoff_start_yaw = None
        self.upstairs_tag_handoff_tag_x_m = 0.0
        self.upstairs_tag_handoff_tag_z_m = 0.0
        self.upstairs_tag_handoff_center_x_m = 0.0

    def _upstairs_tag_trigger_distance(self) -> float:
        if self.upstairs_tag_trigger_distance_m > 0.0:
            return self.upstairs_tag_trigger_distance_m
        return self.upstairs_trigger_distance_m

    def _lock_upstairs_tag_handoff(
        self,
        now: float,
        tag_x: float,
        tag_z: float,
        center_x: float,
    ) -> bool:
        if not self.upstairs_tag_odom_handoff_enabled:
            return False
        if self._upstairs_tag_trigger_distance() <= 0.0:
            return False
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.pose_yaw is None
            or not math.isfinite(tag_x)
            or not math.isfinite(tag_z)
            or not math.isfinite(center_x)
            or tag_z <= 1e-3
        ):
            return False
        if (
            self.upstairs_tag_odom_handoff_max_start_distance_m > 0.0
            and tag_z > self.upstairs_tag_odom_handoff_max_start_distance_m
        ):
            return False
        self.upstairs_tag_handoff_active = True
        self.upstairs_tag_handoff_stamp_sec = now
        self.upstairs_tag_handoff_start_x = self.pose_x
        self.upstairs_tag_handoff_start_y = self.pose_y
        self.upstairs_tag_handoff_start_yaw = self.pose_yaw
        self.upstairs_tag_handoff_tag_x_m = tag_x
        self.upstairs_tag_handoff_tag_z_m = tag_z
        self.upstairs_tag_handoff_center_x_m = center_x
        return True

    def _upstairs_tag_handoff_progress(self, now: float) -> tuple[bool, float, float]:
        if not self.upstairs_tag_handoff_active:
            return False, 0.0, 0.0
        if not self.upstairs_tag_odom_handoff_enabled:
            self._clear_upstairs_tag_handoff()
            return False, 0.0, 0.0
        if (
            self.upstairs_tag_odom_handoff_timeout_sec > 0.0
            and (now - self.upstairs_tag_handoff_stamp_sec)
            > self.upstairs_tag_odom_handoff_timeout_sec
        ):
            self._clear_upstairs_tag_handoff()
            return False, 0.0, 0.0
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.upstairs_tag_handoff_start_x is None
            or self.upstairs_tag_handoff_start_y is None
            or self.upstairs_tag_handoff_start_yaw is None
        ):
            return False, 0.0, 0.0
        dx = self.pose_x - self.upstairs_tag_handoff_start_x
        dy = self.pose_y - self.upstairs_tag_handoff_start_y
        forward_x, forward_y = self._odom_forward_unit(
            self.upstairs_tag_handoff_start_yaw
        )
        along_m = dx * forward_x + dy * forward_y
        if not math.isfinite(along_m):
            return False, 0.0, 0.0
        along_m = max(0.0, along_m)
        estimated_tag_z = self.upstairs_tag_handoff_tag_z_m - along_m
        if not math.isfinite(estimated_tag_z):
            return False, 0.0, 0.0
        estimated_tag_z = max(1e-3, estimated_tag_z)
        return True, along_m, estimated_tag_z

    def _apply_upstairs_tag_measurement(
        self,
        now: float,
        tag_x: float,
        tag_z: float,
        center_x: float,
    ) -> None:
        self.upstairs_detection.distance_m = tag_z
        self.upstairs_detection.lateral_m = tag_x
        self.upstairs_detection.nearest_stamp_sec = now
        self.limit_bar_lateral_shift_center_x_m = center_x
        self.limit_bar_lateral_shift_center_z_m = tag_z

    def _upstairs_tag_handoff_ready(self, now: float) -> bool:
        ok, along_m, estimated_tag_z = self._upstairs_tag_handoff_progress(now)
        if not ok:
            return False
        trigger_distance_m = self._upstairs_tag_trigger_distance()
        if trigger_distance_m > 0.0 and estimated_tag_z > trigger_distance_m:
            if (now - self.last_upstairs_tag_wait_log_sec) >= 0.50:
                self.last_upstairs_tag_wait_log_sec = now
                self._publish_feedback(
                    f'上台阶 AprilTag 丢码接力等待触发线: '
                    f'reason=odom_handoff_distance_not_reached, '
                    f'latched_tag_x={self.upstairs_tag_handoff_tag_x_m:.3f}m, '
                    f'latched_tag_z={self.upstairs_tag_handoff_tag_z_m:.3f}m, '
                    f'odom_forward={along_m:.3f}m, '
                    f'est_tag_z={estimated_tag_z:.3f}m > '
                    f'trigger={trigger_distance_m:.3f}m, '
                    f'center_x={self.upstairs_tag_handoff_center_x_m:.3f}m。'
                )
            return False
        self._apply_upstairs_tag_measurement(
            now,
            self.upstairs_tag_handoff_tag_x_m,
            estimated_tag_z,
            self.upstairs_tag_handoff_center_x_m,
        )
        if (now - self.last_upstairs_tag_wait_log_sec) >= 0.20:
            self.last_upstairs_tag_wait_log_sec = now
            inside_by = (
                trigger_distance_m - estimated_tag_z
                if trigger_distance_m > 0.0
                else 0.0
            )
            self._publish_feedback(
                f'上台阶 AprilTag 分支触发条件满足: '
                f'reason=odom_handoff_after_tag_lost, '
                f'latched_tag_x={self.upstairs_tag_handoff_tag_x_m:.3f}m, '
                f'latched_tag_z={self.upstairs_tag_handoff_tag_z_m:.3f}m, '
                f'odom_forward={along_m:.3f}m, '
                f'est_tag_z={estimated_tag_z:.3f}m, '
                f'trigger_distance={trigger_distance_m:.3f}m, '
                f'inside_by={inside_by:.3f}m, '
                f'center_x={self.upstairs_tag_handoff_center_x_m:.3f}m, '
                f'tag_to_center={self.upstairs_tag_to_center_x_m:+.3f}m。'
            )
        return True

    def _limit_bar_tag_ready(self, now: float) -> bool:
        if not self.limit_bar_tag_trigger_enabled:
            return False
        if self.limit_bar_completed:
            return False
        recent_samples = self._recent_limit_bar_tag_samples(now)
        if len(recent_samples) < self.limit_bar_tag_min_samples:
            if self._limit_bar_tag_handoff_ready(now):
                return True
            if (now - self.last_limit_bar_tag_wait_log_sec) >= 0.50:
                self.last_limit_bar_tag_wait_log_sec = now
                latest_age_text = 'none'
                if self.limit_bar_tag_samples:
                    latest_age_text = f'{now - self.limit_bar_tag_samples[-1].received_stamp_sec:.2f}s'
                det_age_text = 'none'
                if self.limit_bar_tag_detection_stamp_sec > 0.0:
                    det_age_text = f'{now - self.limit_bar_tag_detection_stamp_sec:.2f}s'
                self._publish_feedback(
                    f'限高杆 AprilTag 已检测但暂未进入分支: '
                    f'tf_samples={len(recent_samples)}/{self.limit_bar_tag_min_samples}, '
                    f'latest_tf_age={latest_age_text}, det_age={det_age_text}, '
                    f'ids={self.limit_bar_tag_detection_ids_text or "none"}, '
                    f'expected_tf={self.limit_bar_tag_frame_id}->{self.limit_bar_tag_child_frame_id}。'
                )
            return False
        tag_x = self._median([sample.x_m for sample in recent_samples])
        tag_z = self._median([sample.z_m for sample in recent_samples])
        center_x = tag_x + self.limit_bar_tag_to_center_x_m
        self._lock_limit_bar_tag_handoff(
            now,
            tag_x,
            tag_z,
            center_x,
            source='search_fresh',
            require_flat_pose=False,
        )
        if (
            self.limit_bar_tag_trigger_distance_m > 0.0
            and tag_z > self.limit_bar_tag_trigger_distance_m
        ):
            if (now - self.last_limit_bar_tag_wait_log_sec) >= 0.50:
                self.last_limit_bar_tag_wait_log_sec = now
                trigger_margin_m = tag_z - self.limit_bar_tag_trigger_distance_m
                self._publish_feedback(
                    f'限高杆 AprilTag 已锁定但距离未到触发线: '
                    f'reason=distance_not_reached, '
                    f'samples={len(recent_samples)}, tag_x={tag_x:.3f}m, '
                    f'tag_z={tag_z:.3f}m > trigger={self.limit_bar_tag_trigger_distance_m:.3f}m, '
                    f'over_by={trigger_margin_m:.3f}m, '
                    f'center_x={center_x:.3f}m, '
                    f'tag_to_center={self.limit_bar_tag_to_center_x_m:+.3f}m, '
                    f'target_id={self.limit_bar_tag_target_tag_id}。'
                )
            return False
        self._apply_limit_bar_tag_measurement(now, tag_x, tag_z, center_x)
        self._clear_limit_bar_tag_handoff()
        if (now - self.last_limit_bar_tag_wait_log_sec) >= 0.50:
            self.last_limit_bar_tag_wait_log_sec = now
            trigger_margin_m = (
                self.limit_bar_tag_trigger_distance_m - tag_z
                if self.limit_bar_tag_trigger_distance_m > 0.0
                else 0.0
            )
            self._publish_feedback(
                f'限高杆 AprilTag 分支触发条件满足: '
                f'reason=tag_z_within_trigger, '
                f'samples={len(recent_samples)}/{self.limit_bar_tag_min_samples}, '
                f'tag_x={tag_x:.3f}m, tag_z={tag_z:.3f}m, '
                f'trigger_distance={self.limit_bar_tag_trigger_distance_m:.3f}m, '
                f'inside_by={trigger_margin_m:.3f}m, '
                f'center_x={center_x:.3f}m, '
                f'tag_to_center={self.limit_bar_tag_to_center_x_m:+.3f}m, '
                f'target_id={self.limit_bar_tag_target_tag_id}, '
                f'expected_tf={self.limit_bar_tag_frame_id}->{self.limit_bar_tag_child_frame_id}, '
                f'completed={self.limit_bar_completed}。'
            )
        return True

    def _limit_bar_tag_slope_guard_ready(self, now: float) -> bool:
        if not self.limit_bar_tag_slope_guard_enabled:
            return False
        if not self.limit_bar_tag_trigger_enabled:
            return False
        if self.limit_bar_completed:
            return False
        if self.limit_bar_tag_slope_guard_distance_m <= 0.0:
            return False
        recent_samples = self._recent_limit_bar_tag_samples(now)
        if not recent_samples:
            return False
        tag_x = self._median([sample.x_m for sample in recent_samples])
        tag_z = self._median([sample.z_m for sample in recent_samples])
        if tag_z > self.limit_bar_tag_slope_guard_distance_m:
            return False
        if (now - self.last_limit_bar_tag_guard_log_sec) >= 0.50:
            self.last_limit_bar_tag_guard_log_sec = now
            self._publish_feedback(
                f'限高杆 AprilTag 保护斜坡抢占: '
                f'samples={len(recent_samples)}, tag_x={tag_x:.3f}m, '
                f'tag_z={tag_z:.3f}m, trigger={self.limit_bar_tag_trigger_distance_m:.3f}m, '
                f'guard={self.limit_bar_tag_slope_guard_distance_m:.3f}m, '
                f'center_x={tag_x + self.limit_bar_tag_to_center_x_m:.3f}m，'
                f'继续搜索等待触发线，不进入斜坡 yaw 对齐。'
            )
        return True

    def _limit_bar_nearest_preempts_slope_ready(self, now: float) -> bool:
        if self.limit_bar_completed:
            return False
        if not self.limit_bar_nearest_preempts_slope:
            return False
        if self.limit_bar_nearest_preempt_distance_m <= 0.0:
            return False
        if self.limit_detection.distance_m is None:
            return False
        if (now - self.limit_detection.nearest_stamp_sec) > self.detection_stale_timeout_sec:
            return False
        distance = self._usable_distance(self.limit_detection.distance_m)
        if distance is None:
            return False
        return distance <= self.limit_bar_nearest_preempt_distance_m

    def _pole_near_preempts_limit_bar_ready(self, now: float) -> bool:
        if not self.pole_near_preempts_limit_bar:
            return False
        if self.pole_near_preempt_limit_bar_distance_m <= 0.0:
            return False
        if self.pole_detection.distance_m is None:
            return False
        if (now - self.pole_detection.nearest_stamp_sec) > self.detection_stale_timeout_sec:
            return False
        distance = self._usable_distance(self.pole_detection.distance_m)
        if distance is None:
            return False
        return distance <= self.pole_near_preempt_limit_bar_distance_m

    def _pole_ready(self, now: float) -> bool:
        if not self.pole_visual_trigger_enabled:
            return False
        if not self.pole_detection.triggered:
            return False
        if not self.pole_detection.is_fresh(now, self.detection_stale_timeout_sec):
            return False
        if (
            self.search_reason == 'slope_done'
            and (now - self.search_enter_time) < self.pole_after_slope_search_delay_sec
        ):
            return False
        if self.pole_max_lateral_abs_m > 0.0:
            if self.pole_detection.lateral_m is None:
                return False
            if (now - self.pole_detection.nearest_stamp_sec) > self.detection_stale_timeout_sec:
                return False
            if abs(self.pole_detection.lateral_m) > self.pole_max_lateral_abs_m:
                return False
        if self.pole_trigger_min_duration_sec <= 0.0:
            return True
        if self.pole_detection.trigger_start_stamp_sec <= 0.0:
            return False
        return (now - self.pole_detection.trigger_start_stamp_sec) >= (
            self.pole_trigger_min_duration_sec
        )

    def _upstairs_ready(self, now: float) -> bool:
        if self.upstairs_completed:
            return False
        if not self._upstairs_sequence_allowed():
            if self.upstairs_tag_trigger_enabled and (now - self.last_upstairs_tag_wait_log_sec) >= 0.50:
                self.last_upstairs_tag_wait_log_sec = now
                self._publish_feedback(
                    f'上台阶 AprilTag 已检测但顺序门槛未满足: '
                    f'require_hurdle_completed={self.upstairs_require_hurdle_completed}, '
                    f'hurdle_completed={self.hurdle_completed}, '
                    f'tag_dist={self._distance_text(self.upstairs_detection.distance_m)}, '
                    f'trigger={self._upstairs_tag_trigger_distance():.3f}m。'
                )
            return False
        if self.upstairs_tag_trigger_enabled:
            if self._upstairs_tag_ready(now):
                return True
            fallback_ready, fallback_reason = (
                self._upstairs_tag_visual_fallback_ready(now)
            )
            if fallback_ready:
                if (
                    now - self.last_upstairs_visual_fallback_log_sec
                ) >= 0.50:
                    self.last_upstairs_visual_fallback_log_sec = now
                    self._publish_feedback(
                        f'上台阶近距离视觉兜底条件满足: {fallback_reason}。'
                    )
                return True
            return False
        if not self.upstairs_detection.triggered:
            return False
        if not self.upstairs_detection.is_fresh(now, self.detection_stale_timeout_sec):
            return False
        if self.upstairs_trigger_distance_m > 0.0:
            if self.upstairs_detection.distance_m is None:
                return False
            if (
                now - self.upstairs_detection.nearest_stamp_sec
            ) > self.detection_stale_timeout_sec:
                return False
            distance = self._usable_distance(self.upstairs_detection.distance_m)
            if distance is None or distance > self.upstairs_trigger_distance_m:
                return False
        if self.upstairs_trigger_min_duration_sec <= 0.0:
            return True
        if self.upstairs_detection.trigger_start_stamp_sec <= 0.0:
            return False
        return (now - self.upstairs_detection.trigger_start_stamp_sec) >= (
            self.upstairs_trigger_min_duration_sec
        )

    def _upstairs_visual_step_ready(
        self,
        now: float,
        distance_threshold_m: Optional[float] = None,
    ) -> tuple[bool, str]:
        detection = self.upstairs_visual_detection
        threshold_m = (
            self.upstairs_trigger_distance_m
            if distance_threshold_m is None
            else max(0.0, float(distance_threshold_m))
        )
        if not detection.triggered:
            return False, 'trigger=false'
        if not detection.is_fresh(now, self.detection_stale_timeout_sec):
            return False, 'stale'
        if threshold_m > 0.0:
            if detection.distance_m is None:
                return False, 'distance=none'
            if (now - detection.nearest_stamp_sec) > self.detection_stale_timeout_sec:
                return False, 'nearest_stale'
            distance = self._usable_distance(detection.distance_m)
            if distance is None:
                return False, f'distance_invalid({detection.distance_m})'
            if distance > threshold_m:
                return (
                    False,
                    f'dist={distance:.3f}m>{threshold_m:.3f}m',
                )
        if self.upstairs_trigger_min_duration_sec > 0.0:
            if detection.trigger_start_stamp_sec <= 0.0:
                return False, 'trigger_start=none'
            held_sec = now - detection.trigger_start_stamp_sec
            if held_sec < self.upstairs_trigger_min_duration_sec:
                return (
                    False,
                    f'hold={held_sec:.2f}/{self.upstairs_trigger_min_duration_sec:.2f}s',
                )
        distance_text = self._distance_text(detection.distance_m)
        return True, (
            f'trigger=true, dist={distance_text}, '
            f'threshold={threshold_m:.3f}m'
        )

    def _upstairs_final_approach_ready(
        self,
        now: float,
        live_alignment: Optional[tuple[float, float, float, float, float]],
    ) -> tuple[bool, str]:
        if live_alignment is not None:
            tag_z_m = live_alignment[1]
            threshold_m = (
                self.upstairs_lateral_shift_target_distance_m
                + self.upstairs_final_tag_distance_tolerance_m
            )
            tag_ready = math.isfinite(tag_z_m) and tag_z_m <= threshold_m
            tag_comparator = '<=' if tag_ready else '>'
            tag_reason = (
                f'tag_z={tag_z_m:.3f}m{tag_comparator}{threshold_m:.3f}m'
            )
            if tag_ready:
                return True, f'source=live_tag, {tag_reason}'
            visual_ready, visual_reason = self._upstairs_visual_step_ready(
                now,
                self.upstairs_final_visual_distance_m,
            )
            if visual_ready:
                return (
                    True,
                    f'source=visual_with_live_tag, {visual_reason}, {tag_reason}',
                )
            return (
                False,
                f'source=live_tag, {tag_reason}, visual={visual_reason}',
            )

        visual_ready, visual_reason = self._upstairs_visual_step_ready(
            now,
            self.upstairs_final_visual_distance_m,
        )
        return visual_ready, f'source=visual_no_fresh_tag, {visual_reason}'

    def _upstairs_tag_visual_fallback_ready(
        self,
        now: float,
    ) -> tuple[bool, str]:
        if not self.upstairs_tag_visual_fallback_enabled:
            return False, 'disabled'
        if (
            self.upstairs_tag_visual_fallback_require_hurdle_completed
            and not self.hurdle_completed
        ):
            return False, 'hurdle_not_completed'
        if self._recent_upstairs_tag_samples(now):
            return False, 'fresh_tag_tf_available'
        tag_detection_text = 'tag_id_not_seen'
        if self.upstairs_tag_detection_stamp_sec > 0.0:
            tag_detection_age_sec = now - self.upstairs_tag_detection_stamp_sec
            tag_detection_text = f'tag_id_age={tag_detection_age_sec:.2f}s'
            tag_id_stale = (
                self.upstairs_tag_visual_fallback_max_detection_age_sec > 0.0
                and tag_detection_age_sec
                > self.upstairs_tag_visual_fallback_max_detection_age_sec
            )
            if (
                self.upstairs_tag_visual_fallback_require_recent_tag_id
                and tag_id_stale
            ):
                return (
                    False,
                    f'tag_detection_stale({tag_detection_age_sec:.2f}s)',
                )
        elif self.upstairs_tag_visual_fallback_require_recent_tag_id:
            return False, 'tag_id_not_seen'
        visual_ready, visual_reason = self._upstairs_visual_step_ready(now)
        if not visual_ready:
            return False, visual_reason
        return (
            True,
            f'{visual_reason}, {tag_detection_text}, '
            f'tf_queue_depth={self.apriltag_tf_queue_depth}',
        )

    def _upstairs_slope_guard_ready(self, now: float) -> bool:
        if self.upstairs_completed:
            return False
        if not self._upstairs_sequence_allowed():
            return False
        if self.upstairs_tag_trigger_enabled:
            return self._upstairs_tag_slope_guard_ready(now)
        if not self.upstairs_slope_guard_enabled:
            return False
        if self.upstairs_slope_guard_distance_m <= 0.0:
            return False
        if not self.upstairs_detection.triggered:
            return False
        if not self.upstairs_detection.is_fresh(now, self.detection_stale_timeout_sec):
            return False
        if self.upstairs_detection.distance_m is None:
            return False
        if (
            now - self.upstairs_detection.nearest_stamp_sec
        ) > self.detection_stale_timeout_sec:
            return False
        distance = self._usable_distance(self.upstairs_detection.distance_m)
        if distance is None:
            return False
        return distance <= self.upstairs_slope_guard_distance_m

    def _upstairs_tag_ready(self, now: float) -> bool:
        if not self.upstairs_tag_trigger_enabled:
            return False
        if self.upstairs_completed:
            return False
        if not self._upstairs_sequence_allowed():
            return False
        recent_samples = self._recent_upstairs_tag_samples(now)
        if len(recent_samples) < self.upstairs_tag_min_samples:
            if self._upstairs_tag_handoff_ready(now):
                return True
            if (now - self.last_upstairs_tag_wait_log_sec) >= 0.50:
                self.last_upstairs_tag_wait_log_sec = now
                latest_age_text = 'none'
                if self.upstairs_tag_samples:
                    latest_age_text = (
                        f'{now - self.upstairs_tag_samples[-1].received_stamp_sec:.2f}s'
                    )
                det_age_text = 'none'
                if self.upstairs_tag_detection_stamp_sec > 0.0:
                    det_age_text = f'{now - self.upstairs_tag_detection_stamp_sec:.2f}s'
                tf_topic_age_text = 'none'
                if self.apriltag_tf_last_received_sec > 0.0:
                    tf_topic_age_text = (
                        f'{now - self.apriltag_tf_last_received_sec:.2f}s'
                    )
                self._publish_feedback(
                    f'上台阶 AprilTag 已检测但暂未进入分支: '
                    f'tf_samples={len(recent_samples)}/{self.upstairs_tag_min_samples}, '
                    f'latest_tf_age={latest_age_text}, det_age={det_age_text}, '
                    f'ids={self.upstairs_tag_detection_ids_text or "none"}, '
                    f'tf_topic_age={tf_topic_age_text}, '
                    f'last_tf_children=[{self.apriltag_tf_last_children_text}], '
                    f'expected_tf={self.upstairs_tag_frame_id}->{self.upstairs_tag_child_frame_id}。'
                )
            return False

        tag_x = self._median([sample.x_m for sample in recent_samples])
        tag_z = self._median([sample.z_m for sample in recent_samples])
        if not math.isfinite(tag_x) or not math.isfinite(tag_z) or tag_z <= 1e-3:
            return False

        center_x = tag_x + self.upstairs_tag_to_center_x_m
        self._lock_upstairs_tag_handoff(now, tag_x, tag_z, center_x)
        trigger_distance_m = self._upstairs_tag_trigger_distance()
        if trigger_distance_m > 0.0 and tag_z > trigger_distance_m:
            if (now - self.last_upstairs_tag_wait_log_sec) >= 0.50:
                self.last_upstairs_tag_wait_log_sec = now
                trigger_margin_m = tag_z - trigger_distance_m
                self._publish_feedback(
                    f'上台阶 AprilTag 已锁定但距离未到触发线: '
                    f'reason=distance_not_reached, '
                    f'samples={len(recent_samples)}, tag_x={tag_x:.3f}m, '
                    f'tag_z={tag_z:.3f}m > trigger={trigger_distance_m:.3f}m, '
                    f'over_by={trigger_margin_m:.3f}m, '
                    f'center_x={center_x:.3f}m, '
                    f'tag_to_center={self.upstairs_tag_to_center_x_m:+.3f}m, '
                    f'target_id={self.upstairs_tag_target_tag_id}。'
                )
            return False

        self._apply_upstairs_tag_measurement(now, tag_x, tag_z, center_x)
        self._clear_upstairs_tag_handoff()
        if (now - self.last_upstairs_tag_wait_log_sec) >= 0.50:
            self.last_upstairs_tag_wait_log_sec = now
            trigger_margin_m = trigger_distance_m - tag_z if trigger_distance_m > 0.0 else 0.0
            self._publish_feedback(
                f'上台阶 AprilTag 分支触发条件满足: '
                f'reason=tag_z_within_trigger, '
                f'samples={len(recent_samples)}/{self.upstairs_tag_min_samples}, '
                f'tag_x={tag_x:.3f}m, tag_z={tag_z:.3f}m, '
                f'trigger_distance={trigger_distance_m:.3f}m, '
                f'inside_by={trigger_margin_m:.3f}m, '
                f'center_x={center_x:.3f}m, '
                f'tag_to_center={self.upstairs_tag_to_center_x_m:+.3f}m, '
                f'target_id={self.upstairs_tag_target_tag_id}, '
                f'expected_tf={self.upstairs_tag_frame_id}->{self.upstairs_tag_child_frame_id}, '
                f'completed={self.upstairs_completed}。'
            )
        return True

    def _upstairs_tag_slope_guard_ready(self, now: float) -> bool:
        if not self.upstairs_tag_slope_guard_enabled:
            return False
        if not self.upstairs_tag_trigger_enabled:
            return False
        if self.upstairs_completed:
            return False
        if not self._upstairs_sequence_allowed():
            return False
        if self.upstairs_tag_slope_guard_distance_m <= 0.0:
            return False
        recent_samples = self._recent_upstairs_tag_samples(now)
        if not recent_samples:
            return False
        tag_x = self._median([sample.x_m for sample in recent_samples])
        tag_z = self._median([sample.z_m for sample in recent_samples])
        if not math.isfinite(tag_x) or not math.isfinite(tag_z) or tag_z <= 1e-3:
            return False

        self.upstairs_detection.distance_m = tag_z
        self.upstairs_detection.lateral_m = tag_x
        self.upstairs_detection.nearest_stamp_sec = now
        if tag_z > self.upstairs_tag_slope_guard_distance_m:
            return False
        trigger_distance_m = (
            self.upstairs_tag_trigger_distance_m
            if self.upstairs_tag_trigger_distance_m > 0.0
            else self.upstairs_trigger_distance_m
        )
        if (now - self.last_upstairs_tag_guard_log_sec) >= 0.50:
            self.last_upstairs_tag_guard_log_sec = now
            self._publish_feedback(
                f'上台阶 AprilTag 保护斜坡抢占: '
                f'samples={len(recent_samples)}, tag_x={tag_x:.3f}m, '
                f'tag_z={tag_z:.3f}m, trigger={trigger_distance_m:.3f}m, '
                f'guard={self.upstairs_tag_slope_guard_distance_m:.3f}m, '
                f'center_x={tag_x + self.upstairs_tag_to_center_x_m:.3f}m，'
                f'继续搜索等待触发线，不进入斜坡 yaw 对齐。'
            )
        return True

    def _upstairs_tag_detection_guard_ready(self, now: float) -> bool:
        if not self.upstairs_tag_slope_guard_enabled:
            return False
        if not self.upstairs_tag_trigger_enabled:
            return False
        if self.upstairs_completed:
            return False
        if not self._upstairs_sequence_allowed():
            return False
        if self.upstairs_tag_detection_stamp_sec <= 0.0:
            return False
        age = now - self.upstairs_tag_detection_stamp_sec
        if self.upstairs_tag_stale_timeout_sec > 0.0 and age > self.upstairs_tag_stale_timeout_sec:
            return False
        if (now - self.last_upstairs_tag_guard_log_sec) >= 0.50:
            self.last_upstairs_tag_guard_log_sec = now
            self._publish_feedback(
                f'上台阶 AprilTag ID{self.upstairs_tag_target_tag_id} 已检测，'
                f'等待 TF/距离确认期间抑制斜坡抢占: '
                f'ids={self.upstairs_tag_detection_ids_text or "none"}, '
                f'det_age={age:.2f}s, '
                f'expected_tf={self.upstairs_tag_frame_id}->{self.upstairs_tag_child_frame_id}。'
            )
        return True

    def _slope_waiting_for_upstairs_nearest(self, now: float) -> bool:
        if self.upstairs_completed:
            return False
        if not self._upstairs_sequence_allowed():
            return False
        if not self.upstairs_slope_guard_enabled:
            return False
        if self.slope_wait_for_upstairs_nearest_sec <= 0.0:
            return False
        if self.upstairs_detection.distance_m is not None:
            nearest_age_sec = now - self.upstairs_detection.nearest_stamp_sec
            if nearest_age_sec <= self.detection_stale_timeout_sec:
                self.slope_upstairs_wait_start_sec = 0.0
                return False
        if self.slope_upstairs_wait_start_sec <= 0.0:
            self.slope_upstairs_wait_start_sec = now
            self.last_slope_upstairs_wait_log_sec = 0.0
        waited_sec = now - self.slope_upstairs_wait_start_sec
        if waited_sec >= self.slope_wait_for_upstairs_nearest_sec:
            return False
        if (
            self.last_slope_upstairs_wait_log_sec <= 0.0
            or (now - self.last_slope_upstairs_wait_log_sec) >= 0.25
        ):
            self.last_slope_upstairs_wait_log_sec = now
            self._publish_feedback(
                f'斜坡已触发，但上台阶 detector 还没有新鲜 nearest，先等待 '
                f'{waited_sec:.2f}/{self.slope_wait_for_upstairs_nearest_sec:.2f}s '
                f'再决定是否进入斜坡。'
            )
        return True

    def _upstairs_sequence_allowed(self) -> bool:
        if self.upstairs_tag_trigger_enabled:
            return True
        if not self.upstairs_require_hurdle_completed:
            return True
        return self.hurdle_completed

    def _duck_travel_distance(self) -> Optional[float]:
        if (
            self.pose_x is None or self.pose_y is None
            or self.duck_start_x is None or self.duck_start_y is None
        ):
            return None
        delta_x = self.pose_x - self.duck_start_x
        delta_y = self.pose_y - self.duck_start_y
        if not math.isfinite(delta_x) or not math.isfinite(delta_y):
            return None
        travel_m = math.hypot(delta_x, delta_y)
        if not math.isfinite(travel_m):
            return None
        return travel_m

    def _duck_projected_travel_distance(self) -> Optional[float]:
        if (
            self.pose_x is None or self.pose_y is None
            or self.duck_start_x is None or self.duck_start_y is None
            or self.duck_reference_yaw is None
        ):
            return None
        delta_x = self.pose_x - self.duck_start_x
        delta_y = self.pose_y - self.duck_start_y
        if not math.isfinite(delta_x) or not math.isfinite(delta_y):
            return None
        forward_x, forward_y = self._odom_forward_unit(self.duck_reference_yaw)
        along_m = delta_x * forward_x + delta_y * forward_y
        if not math.isfinite(along_m):
            return None
        return along_m

    def _duck_travel_progress(self) -> tuple[Optional[float], str]:
        if self.duck_travel_projection_enabled:
            return self._duck_projected_travel_distance(), 'along'
        return self._duck_travel_distance(), 'travel'

    def _prune_limit_bar_tag_samples(self, now: float) -> None:
        max_age = max(
            self.limit_bar_tag_max_sample_age_sec,
            self.limit_bar_tag_stale_timeout_sec,
        )
        if max_age <= 0.0:
            self.limit_bar_tag_samples = self.limit_bar_tag_samples[-20:]
            return
        self.limit_bar_tag_samples = [
            sample
            for sample in self.limit_bar_tag_samples
            if (now - sample.received_stamp_sec) <= max_age
        ][-20:]

    def _prune_hurdle_tag_samples(self, now: float) -> None:
        max_age = max(
            self.hurdle_tag_max_sample_age_sec,
            self.hurdle_tag_stale_timeout_sec,
        )
        if max_age <= 0.0:
            self.hurdle_tag_samples = self.hurdle_tag_samples[-20:]
            return
        self.hurdle_tag_samples = [
            sample
            for sample in self.hurdle_tag_samples
            if (now - sample.received_stamp_sec) <= max_age
        ][-20:]

    def _prune_upstairs_tag_samples(self, now: float) -> None:
        max_age = max(
            self.upstairs_tag_max_sample_age_sec,
            self.upstairs_tag_stale_timeout_sec,
        )
        if max_age <= 0.0:
            self.upstairs_tag_samples = self.upstairs_tag_samples[-20:]
            return
        self.upstairs_tag_samples = [
            sample
            for sample in self.upstairs_tag_samples
            if (now - sample.received_stamp_sec) <= max_age
        ][-20:]

    def _prune_sandpit_tag_samples(self, now: float) -> None:
        max_age = max(
            self.sandpit_tag_max_sample_age_sec,
            self.sandpit_trigger_stale_timeout_sec,
        )
        if max_age <= 0.0:
            self.sandpit_tag_samples = self.sandpit_tag_samples[-20:]
            return
        self.sandpit_tag_samples = [
            sample
            for sample in self.sandpit_tag_samples
            if (now - sample.received_stamp_sec) <= max_age
        ][-20:]

    @staticmethod
    def _median(values: List[float]) -> float:
        sorted_values = sorted(values)
        count = len(sorted_values)
        mid = count // 2
        if count % 2:
            return sorted_values[mid]
        return 0.5 * (sorted_values[mid - 1] + sorted_values[mid])

    def _recent_limit_bar_tag_samples(self, now: float) -> List[LimitBarTagSample]:
        if not (self.limit_bar_tag_align_enabled or self.limit_bar_tag_trigger_enabled):
            return []
        self._prune_limit_bar_tag_samples(now)
        recent_samples = []
        for sample in self.limit_bar_tag_samples:
            age = now - sample.received_stamp_sec
            if (
                self.limit_bar_tag_stale_timeout_sec > 0.0
                and age > self.limit_bar_tag_stale_timeout_sec
            ):
                continue
            if (
                self.limit_bar_tag_max_sample_age_sec > 0.0
                and age > self.limit_bar_tag_max_sample_age_sec
            ):
                continue
            recent_samples.append(sample)
        return recent_samples

    def _recent_hurdle_tag_samples(self, now: float) -> List[LimitBarTagSample]:
        if not self.hurdle_tag_trigger_enabled:
            return []
        self._prune_hurdle_tag_samples(now)
        recent_samples = []
        for sample in self.hurdle_tag_samples:
            age = now - sample.received_stamp_sec
            if self.hurdle_tag_stale_timeout_sec > 0.0 and age > self.hurdle_tag_stale_timeout_sec:
                continue
            if self.hurdle_tag_max_sample_age_sec > 0.0 and age > self.hurdle_tag_max_sample_age_sec:
                continue
            recent_samples.append(sample)
        if not recent_samples and self._refresh_hurdle_tag_from_tf_buffer(now):
            recent_samples.append(self.hurdle_tag_samples[-1])
        return recent_samples

    def _refresh_hurdle_tag_from_tf_buffer(self, now: float) -> bool:
        if (
            not self.hurdle_tf_buffer_fallback_enabled
            or self.hurdle_tf_buffer is None
        ):
            return False
        try:
            transform = self.hurdle_tf_buffer.lookup_transform(
                self.hurdle_tag_frame_id,
                self.hurdle_tag_child_frame_id,
                Time(),
            )
        except TransformException:
            return False

        source_stamp_sec = (
            float(transform.header.stamp.sec)
            + float(transform.header.stamp.nanosec) / 1e9
        )
        if (
            source_stamp_sec <= 0.0
            or source_stamp_sec
            <= self.hurdle_tf_buffer_last_source_stamp_sec + 1e-6
        ):
            return False
        source_age_sec = now - source_stamp_sec
        max_ages = [
            value
            for value in (
                self.hurdle_tag_stale_timeout_sec,
                self.hurdle_tag_max_sample_age_sec,
            )
            if value > 0.0
        ]
        max_age_sec = min(max_ages) if max_ages else 0.0
        if source_age_sec < -0.10:
            return False
        if max_age_sec > 0.0 and source_age_sec > max_age_sec:
            return False

        translation = transform.transform.translation
        roll_rad, pitch_rad, yaw_rad = self._rpy_from_quaternion(
            transform.transform.rotation
        )
        sample = LimitBarTagSample(
            received_stamp_sec=now,
            x_m=float(translation.x),
            y_m=float(translation.y),
            z_m=float(translation.z),
            roll_rad=roll_rad,
            pitch_rad=pitch_rad,
            yaw_rad=yaw_rad,
            frame_id=str(transform.header.frame_id),
            child_frame_id=str(transform.child_frame_id),
        )
        if (
            sample.frame_id != self.hurdle_tag_frame_id
            or sample.child_frame_id != self.hurdle_tag_child_frame_id
            or not math.isfinite(sample.x_m)
            or not math.isfinite(sample.y_m)
            or not math.isfinite(sample.z_m)
            or not math.isfinite(sample.roll_rad)
            or not math.isfinite(sample.pitch_rad)
            or not math.isfinite(sample.yaw_rad)
            or sample.z_m <= 1e-3
        ):
            return False

        self.hurdle_tf_buffer_last_source_stamp_sec = source_stamp_sec
        self.hurdle_tag_samples.append(sample)
        self.hurdle_detection.distance_m = sample.z_m
        self.hurdle_detection.lateral_m = sample.x_m
        self.hurdle_detection.nearest_stamp_sec = now
        self._prune_hurdle_tag_samples(now)
        if (now - self.last_hurdle_tf_buffer_log_sec) >= 0.50:
            self.last_hurdle_tf_buffer_log_sec = now
            self._publish_feedback(
                f'高墙 AprilTag TF Buffer 兜底已接收: '
                f'frame={sample.frame_id}->{sample.child_frame_id}, '
                f'tag_x={sample.x_m:.3f}m, tag_z={sample.z_m:.3f}m, '
                f'source_age={source_age_sec:.3f}s。'
            )
        return True

    def _recent_upstairs_tag_samples(self, now: float) -> List[LimitBarTagSample]:
        if not self.upstairs_tag_trigger_enabled:
            return []
        self._prune_upstairs_tag_samples(now)
        recent_samples = []
        for sample in self.upstairs_tag_samples:
            age = now - sample.received_stamp_sec
            if self.upstairs_tag_stale_timeout_sec > 0.0 and age > self.upstairs_tag_stale_timeout_sec:
                continue
            if self.upstairs_tag_max_sample_age_sec > 0.0 and age > self.upstairs_tag_max_sample_age_sec:
                continue
            recent_samples.append(sample)
        return recent_samples

    def _recent_sandpit_tag_samples(self, now: float) -> List[SandpitTagSample]:
        if not self.sandpit_bypass_enabled:
            return []
        self._prune_sandpit_tag_samples(now)
        recent_samples = []
        for sample in self.sandpit_tag_samples:
            age = now - sample.received_stamp_sec
            if (
                self.sandpit_trigger_stale_timeout_sec > 0.0
                and age > self.sandpit_trigger_stale_timeout_sec
            ):
                continue
            if (
                self.sandpit_tag_max_sample_age_sec > 0.0
                and age > self.sandpit_tag_max_sample_age_sec
            ):
                continue
            recent_samples.append(sample)
        return recent_samples

    def _limit_bar_lateral_pre_align_error(
        self,
        sample: LimitBarTagSample,
    ) -> tuple[str, float, float, float, float]:
        mode = self._active_lateral_pre_align_mode()
        center_offset_m = self._active_lateral_pre_align_center_offset_m()
        if mode == 'tag_center':
            measured = math.atan2(sample.x_m, sample.z_m)
            target = 0.0
            raw_error = self._normalize_angle(measured - target)
            control_error = self.limit_bar_tag_align_yaw_offset_sign * raw_error
            return 'tag_center', measured, target, raw_error, control_error
        if mode == 'centerline':
            measured = math.atan2(sample.x_m + center_offset_m, sample.z_m)
            target = 0.0
            raw_error = self._normalize_angle(measured - target)
            control_error = self.limit_bar_tag_align_yaw_offset_sign * raw_error
            return 'centerline', measured, target, raw_error, control_error
        if mode == 'fixed_absolute_yaw':
            if self.pose_yaw is None:
                return 'fixed_absolute_yaw', 0.0, 0.0, 0.0, 0.0
            measured = self.pose_yaw
            target = self._active_lateral_pre_align_fixed_yaw_rad()
            raw_error = self._normalize_angle(measured - target)
            control_error = self._normalize_angle(target - measured)
            return 'fixed_absolute_yaw', measured, target, raw_error, control_error
        if mode == 'pose_roll':
            measured = sample.roll_rad
            target = 0.0
            metric = 'pose_roll'
        elif mode == 'pose_pitch':
            measured = sample.pitch_rad
            target = self._active_lateral_pre_align_target_pose_pitch_rad()
            metric = 'pose_pitch'
        else:
            measured = sample.yaw_rad
            target = 0.0
            metric = 'pose_yaw'
        raw_error = self._normalize_angle(measured - target)
        control_error = self._active_lateral_pre_align_pose_control_sign() * raw_error
        return metric, measured, target, raw_error, control_error

    def _active_lateral_pre_align_samples(self, now: float) -> List[LimitBarTagSample]:
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return self._recent_upstairs_tag_samples(now)
        if self.state == self.STATE_HURDLE_ALIGN:
            return self._recent_hurdle_tag_samples(now)
        return self._recent_limit_bar_tag_samples(now)

    def _hurdle_fixed_yaw_locked_sample(
        self,
        now: float,
    ) -> Optional[LimitBarTagSample]:
        if (
            self.state != self.STATE_HURDLE_ALIGN
            or self.hurdle_lateral_pre_align_mode != 'fixed_absolute_yaw'
            or self.hurdle_lateral_pre_align_use_visual_feedback
            or not self.hurdle_locked_tag_valid
            or not math.isfinite(self.hurdle_locked_tag_x_m)
            or not math.isfinite(self.hurdle_locked_tag_z_m)
            or self.hurdle_locked_tag_z_m <= 1e-3
        ):
            return None
        return LimitBarTagSample(
            received_stamp_sec=now,
            x_m=self.hurdle_locked_tag_x_m,
            y_m=0.0,
            z_m=self.hurdle_locked_tag_z_m,
            roll_rad=0.0,
            pitch_rad=0.0,
            yaw_rad=0.0,
            frame_id=self.hurdle_tag_frame_id,
            child_frame_id=self.hurdle_tag_child_frame_id,
        )

    def _active_lateral_pre_align_label(self) -> str:
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return '上台阶'
        if self.state == self.STATE_HURDLE_ALIGN:
            return '高墙'
        return '限高杆'

    def _active_lateral_pre_align_mode(self) -> str:
        if self.state == self.STATE_HURDLE_ALIGN:
            return self.hurdle_lateral_pre_align_mode
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return self.upstairs_lateral_pre_align_mode
        return self.limit_bar_lateral_pre_align_mode

    def _active_lateral_pre_align_use_visual_feedback(self) -> bool:
        if self.state == self.STATE_HURDLE_ALIGN:
            return self.hurdle_lateral_pre_align_use_visual_feedback
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return self.upstairs_lateral_pre_align_use_visual_feedback
        return self.limit_bar_lateral_pre_align_use_visual_feedback

    def _active_lateral_pre_align_timeout_sec(self) -> float:
        if self.state == self.STATE_HURDLE_ALIGN:
            return self.hurdle_lateral_pre_align_timeout_sec
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return self.upstairs_lateral_pre_align_timeout_sec
        return self.limit_bar_lateral_pre_align_timeout_sec

    def _active_lateral_pre_align_max_visual_error_rad(self) -> float:
        if self.state == self.STATE_HURDLE_ALIGN:
            return self.hurdle_lateral_pre_align_max_visual_error_rad
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return self.upstairs_lateral_pre_align_max_visual_error_rad
        return self.limit_bar_lateral_pre_align_max_visual_error_rad

    def _active_lateral_pre_align_hold_sec(self) -> float:
        if self.state == self.STATE_HURDLE_ALIGN:
            return self.hurdle_lateral_pre_align_hold_sec
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return self.upstairs_lateral_pre_align_hold_sec
        return self.limit_bar_lateral_pre_align_hold_sec

    def _active_lateral_pre_align_stop_during_hold_enabled(self) -> bool:
        if self.state == self.STATE_HURDLE_ALIGN:
            return getattr(
                self,
                'hurdle_lateral_pre_align_stop_during_hold_enabled',
                False,
            )
        if self.state == self.STATE_LIMIT_BAR_ALIGN:
            return getattr(
                self,
                'limit_bar_lateral_pre_align_stop_during_hold_enabled',
                False,
            )
        return False

    def _active_lateral_pre_align_tolerance_rad(self) -> float:
        if self.state == self.STATE_HURDLE_ALIGN:
            return self.hurdle_lateral_pre_align_tolerance_rad
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return self.upstairs_lateral_pre_align_tolerance_rad
        return self.limit_bar_lateral_pre_align_tolerance_rad

    def _active_lateral_pre_align_pose_control_sign(self) -> float:
        if self.state == self.STATE_HURDLE_ALIGN:
            return self.hurdle_lateral_pre_align_pose_control_sign
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return self.upstairs_lateral_pre_align_pose_control_sign
        return self.limit_bar_lateral_pre_align_pose_control_sign

    def _active_lateral_pre_align_target_pose_pitch_rad(self) -> float:
        if self.state == self.STATE_HURDLE_ALIGN:
            return self.hurdle_lateral_pre_align_target_pose_pitch_rad
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return self.upstairs_lateral_pre_align_target_pose_pitch_rad
        return self.limit_bar_lateral_pre_align_target_pose_pitch_rad

    def _active_lateral_pre_align_fixed_yaw_rad(self) -> float:
        if self.state == self.STATE_HURDLE_ALIGN:
            return self._hurdle_fixed_reference_target_yaw()
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return self._upstairs_fixed_reference_target_yaw()
        if self.duck_reference_yaw is not None:
            return self.duck_reference_yaw
        if self.pose_yaw is not None:
            return self.pose_yaw
        return 0.0

    def _active_lateral_pre_align_center_offset_m(self) -> float:
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return self.upstairs_tag_to_center_x_m
        if self.state == self.STATE_HURDLE_ALIGN:
            return self.hurdle_tag_to_center_x_m
        return self.limit_bar_tag_to_center_x_m

    def _active_lateral_pre_align_expected_frame_text(self) -> str:
        if self.state == self.STATE_UPSTAIRS_ALIGN:
            return f'{self.upstairs_tag_frame_id}->{self.upstairs_tag_child_frame_id}'
        if self.state == self.STATE_HURDLE_ALIGN:
            return f'{self.hurdle_tag_frame_id}->{self.hurdle_tag_child_frame_id}'
        return f'{self.limit_bar_tag_frame_id}->{self.limit_bar_tag_child_frame_id}'

    def _select_lateral_pre_align_lock_sample(
        self,
        recent_samples: List[LimitBarTagSample],
    ) -> tuple[LimitBarTagSample, str, float]:
        latest_sample = recent_samples[-1]
        if (
            self.state != self.STATE_UPSTAIRS_ALIGN
            or self._active_lateral_pre_align_mode() != 'pose_pitch'
            or len(recent_samples) < 3
        ):
            return latest_sample, 'latest', latest_sample.pitch_rad

        median_pitch = self._median(
            [sample.pitch_rad for sample in recent_samples]
        )
        selected_sample = min(
            recent_samples,
            key=lambda sample: abs(sample.pitch_rad - median_pitch),
        )
        return selected_sample, 'median_pose_pitch', median_pitch

    def _run_limit_bar_lateral_pre_align(self, now: float) -> None:
        label = self._active_lateral_pre_align_label()
        tolerance_rad = self._active_lateral_pre_align_tolerance_rad()
        timeout_sec = self._active_lateral_pre_align_timeout_sec()
        max_visual_error_rad = self._active_lateral_pre_align_max_visual_error_rad()
        hold_sec = self._active_lateral_pre_align_hold_sec()
        use_visual_feedback = self._active_lateral_pre_align_use_visual_feedback()
        center_offset_m = self._active_lateral_pre_align_center_offset_m()
        recent_samples = self._active_lateral_pre_align_samples(now)
        locked_hurdle_sample = False
        if not recent_samples:
            locked_sample = self._hurdle_fixed_yaw_locked_sample(now)
            if locked_sample is not None:
                if not self._pose_is_fresh(now):
                    self._publish_step_override(0, 0.0, 0.0)
                    self._publish_feedback(
                        '高墙固定绝对 yaw 预对齐等待新鲜 Odometry，'
                        '无需等待新的 AprilTag TF。'
                    )
                    return
                recent_samples = [locked_sample]
                locked_hurdle_sample = True
        if not recent_samples:
            if self.limit_bar_lateral_pre_align_target_yaw is not None:
                yaw_error = self._normalize_angle(
                    self.limit_bar_lateral_pre_align_target_yaw - self.pose_yaw
                )
                if abs(yaw_error) <= tolerance_rad:
                    self.limit_bar_lateral_pre_align_active = False
                    self.limit_bar_lateral_pre_align_done = True
                    self.limit_bar_lateral_pre_align_stable_since = None
                    completion_yaw = self.limit_bar_lateral_pre_align_target_yaw
                    yaw_source = 'pre_align_target_yaw'
                    self.duck_reference_yaw = self._normalize_angle(completion_yaw)
                    self.duck_reference_yaw_offset_rad = self._normalize_angle(
                        self.duck_reference_yaw - self.pose_yaw
                    )
                    self.duck_reference_yaw_source = (
                        f'limit_tag_pre_align metric='
                        f'{self.limit_bar_lateral_pre_align_source_metric},'
                        'tag_lost_after_target_lock'
                    )
                    self.slope_align_custom_target_yaw = self.duck_reference_yaw
                    self.slope_align_enter_time = now
                    self.slope_align_stable_since = None
                    self.last_slope_align_log_time = 0.0
                    shift_source = 'pre_align_target_lock'
                    source_tag_x_m = self.limit_bar_lateral_pre_align_source_tag_x_m
                    source_tag_z_m = self.limit_bar_lateral_pre_align_source_tag_z_m
                    shift_transform_text = ''
                    if (
                        self.state in (
                            self.STATE_HURDLE_ALIGN,
                            self.STATE_UPSTAIRS_ALIGN,
                        )
                        and self.limit_bar_lateral_pre_align_source_odom_yaw is not None
                        and source_tag_x_m is not None
                        and source_tag_z_m is not None
                        and math.isfinite(source_tag_x_m)
                        and math.isfinite(source_tag_z_m)
                    ):
                        source_tag_x_before_m = source_tag_x_m
                        source_tag_z_before_m = source_tag_z_m
                        yaw_delta = self._normalize_angle(
                            completion_yaw
                            - self.limit_bar_lateral_pre_align_source_odom_yaw
                        )
                        rotated_tag_x_m, rotated_tag_z_m = (
                            _rotate_optical_xz_for_yaw_delta(
                                source_tag_x_m,
                                source_tag_z_m,
                                yaw_delta,
                            )
                        )
                        if (
                            math.isfinite(rotated_tag_x_m)
                            and math.isfinite(rotated_tag_z_m)
                            and rotated_tag_z_m > 1e-3
                        ):
                            shift_source = (
                                'hurdle_initial_lock_yaw_compensated'
                                if self.state == self.STATE_HURDLE_ALIGN
                                else 'upstairs_initial_lock_yaw_compensated'
                            )
                            source_tag_x_m = rotated_tag_x_m
                            source_tag_z_m = rotated_tag_z_m
                            shift_transform_text = (
                                f', yaw_delta={math.degrees(yaw_delta):.1f}deg, '
                                f'tag_xz_before=({source_tag_x_before_m:.3f},'
                                f'{source_tag_z_before_m:.3f})m, '
                                f'tag_xz_after=({source_tag_x_m:.3f},'
                                f'{source_tag_z_m:.3f})m'
                            )
                    elif (
                        self.state == self.STATE_HURDLE_ALIGN
                        and self.hurdle_locked_tag_valid
                        and math.isfinite(self.hurdle_locked_tag_x_m)
                        and math.isfinite(self.hurdle_locked_tag_z_m)
                        and self.hurdle_locked_tag_z_m > 1e-3
                    ):
                        shift_source = 'hurdle_initial_lock_uncompensated_fallback'
                        source_tag_x_m = self.hurdle_locked_tag_x_m
                        source_tag_z_m = self.hurdle_locked_tag_z_m
                    if (
                        source_tag_x_m is not None
                        and source_tag_z_m is not None
                        and math.isfinite(source_tag_x_m)
                        and math.isfinite(source_tag_z_m)
                    ):
                        self.limit_bar_lateral_shift_center_x_m = (
                            source_tag_x_m
                            + self.limit_bar_lateral_pre_align_source_center_offset_m
                        )
                        self.limit_bar_lateral_shift_center_z_m = source_tag_z_m
                    self._publish_feedback(
                        f'{label} AprilTag 姿态预对齐完成: '
                        f'reason=tag_lost_after_target_lock, '
                        f'target_yaw={math.degrees(self.duck_reference_yaw):.1f}deg, '
                        f'yaw_err={math.degrees(yaw_error):.1f}deg, '
                        f'yaw_source={yaw_source}, shift_source={shift_source}, '
                        f'center_x={self.limit_bar_lateral_shift_center_x_m:.3f}m'
                        f'{shift_transform_text}。'
                    )
                    if self._start_limit_bar_lateral_shift(
                        now,
                        self.slope_align_reason,
                        self._duck_reference_yaw_text(),
                    ):
                        return
                    return
                direction = 'ccw' if yaw_error > 0.0 else 'cw'
                left_norm, right_norm = self._limit_bar_tag_turn_norms(yaw_error)
                self._publish_step_override(
                    self.limit_bar_tag_align_mode,
                    left_norm,
                    right_norm,
                )
                if (now - self.last_slope_align_log_time) >= 0.5:
                    self.last_slope_align_log_time = now
                    self._publish_feedback(
                        f'{label} AprilTag 姿态预对齐继续 odom 对齐: '
                        f'reason=tag_lost_after_target_lock, '
                        f'target_yaw={math.degrees(self.limit_bar_lateral_pre_align_target_yaw):.1f}deg, '
                        f'yaw_err={math.degrees(yaw_error):.1f}deg, '
                        f'direction={direction}, '
                        f'override=[{self.limit_bar_tag_align_mode},{left_norm:.3f},{right_norm:.3f}]。'
                    )
                return
            self._publish_step_override(0, 0.0, 0.0)
            self._publish_feedback(
                f'{label} AprilTag 姿态预对齐等待新鲜 TF: '
                f'expected={self._active_lateral_pre_align_expected_frame_text()}。'
            )
            return

        latest_sample = recent_samples[-1]
        lock_source = 'latest'
        lock_metric_rad = latest_sample.pitch_rad
        sample = latest_sample
        if self.limit_bar_lateral_pre_align_target_yaw is None:
            if locked_hurdle_sample:
                lock_source = 'hurdle_initial_lock_fixed_yaw'
            else:
                sample, lock_source, lock_metric_rad = (
                    self._select_lateral_pre_align_lock_sample(recent_samples)
                )
        metric, measured, target, raw_error, control_error = (
            self._limit_bar_lateral_pre_align_error(sample)
        )
        if self.limit_bar_lateral_pre_align_target_yaw is None:
            self.limit_bar_lateral_pre_align_start_sec = now
            self.limit_bar_lateral_pre_align_target_yaw = self._normalize_angle(
                self.pose_yaw + control_error
            )
            self.limit_bar_lateral_pre_align_source_metric = metric
            self.limit_bar_lateral_pre_align_source_target_rad = target
            self.limit_bar_lateral_pre_align_source_measured_rad = measured
            self.limit_bar_lateral_pre_align_source_control_rad = control_error
            source_odom_yaw = self.pose_yaw
            source_tag_x_m = sample.x_m
            source_tag_z_m = sample.z_m
            if (
                self.state == self.STATE_LIMIT_BAR_ALIGN
                and self.limit_bar_locked_tag_valid
                and self.limit_bar_locked_tag_odom_yaw is not None
            ):
                source_odom_yaw = self.limit_bar_locked_tag_odom_yaw
                source_tag_x_m = self.limit_bar_locked_tag_x_m
                source_tag_z_m = self.limit_bar_locked_tag_z_m
                lock_source = (
                    f'limit_bar_trigger_lock:{self.limit_bar_locked_tag_source}'
                )
            self.limit_bar_lateral_pre_align_source_odom_yaw = source_odom_yaw
            self.limit_bar_lateral_pre_align_source_tag_x_m = source_tag_x_m
            self.limit_bar_lateral_pre_align_source_tag_z_m = source_tag_z_m
            self.limit_bar_lateral_pre_align_source_center_offset_m = center_offset_m
            self._publish_feedback(
                f'{label} AprilTag 姿态预对齐目标锁定: '
                f'metric={metric}, tag_x={source_tag_x_m:.3f}m, '
                f'tag_z={source_tag_z_m:.3f}m, '
                f'measured={math.degrees(measured):.1f}deg, '
                f'target={math.degrees(target):.1f}deg, '
                f'raw_err={math.degrees(raw_error):.1f}deg, '
                f'control_delta={math.degrees(control_error):.1f}deg, '
                f'rpy=({math.degrees(sample.roll_rad):.1f},'
                f'{math.degrees(sample.pitch_rad):.1f},'
                f'{math.degrees(sample.yaw_rad):.1f})deg, '
                f'lock_source={lock_source}, lock_samples={len(recent_samples)}, '
                f'lock_metric={math.degrees(lock_metric_rad):.1f}deg, '
                f'latest_pose_pitch={math.degrees(latest_sample.pitch_rad):.1f}deg, '
                f'odom_yaw0={math.degrees(self.pose_yaw):.1f}deg, '
                f'target_yaw={math.degrees(self.limit_bar_lateral_pre_align_target_yaw):.1f}deg。'
            )

        target_yaw = self.limit_bar_lateral_pre_align_target_yaw
        yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
        if use_visual_feedback:
            aligned = abs(raw_error) <= tolerance_rad
            visual_aligned = aligned
            command_error = control_error
        else:
            aligned = abs(yaw_error) <= tolerance_rad
            visual_aligned = (
                max_visual_error_rad <= 0.0
                or abs(raw_error) <= max_visual_error_rad
            )
            command_error = yaw_error

        if aligned and visual_aligned:
            if self.limit_bar_lateral_pre_align_stable_since is None:
                self.limit_bar_lateral_pre_align_stable_since = now
            stable_sec = now - self.limit_bar_lateral_pre_align_stable_since
        else:
            self.limit_bar_lateral_pre_align_stable_since = None
            stable_sec = 0.0

        timed_out = (
            timeout_sec > 0.0
            and self.limit_bar_lateral_pre_align_start_sec is not None
            and (now - self.limit_bar_lateral_pre_align_start_sec)
            >= timeout_sec
        )
        if timed_out and not (aligned and visual_aligned):
            self._fail(
                f'{label} AprilTag 姿态预对齐超时: '
                f'metric={metric}, visual_err={math.degrees(raw_error):.1f}deg, '
                f'odom_yaw_err={math.degrees(yaw_error):.1f}deg, '
                f'timeout={timeout_sec:.1f}s。'
            )
            return

        if (
            aligned
            and visual_aligned
            and stable_sec < hold_sec
            and self._active_lateral_pre_align_stop_during_hold_enabled()
        ):
            self._publish_step_override(0, 0.0, 0.0)
            if (now - self.last_slope_align_log_time) >= 0.5:
                self.last_slope_align_log_time = now
                self._publish_feedback(
                    f'{label} AprilTag 姿态预对齐保持中: '
                    f'yaw_err={math.degrees(yaw_error):.1f}deg, '
                    f'hold={stable_sec:.2f}/{hold_sec:.2f}s, '
                    'override=[0,0.000,0.000]。'
                )
            return

        if (
            aligned
            and visual_aligned
            and stable_sec >= hold_sec
        ):
            self.limit_bar_lateral_pre_align_active = False
            self.limit_bar_lateral_pre_align_done = True
            self.limit_bar_lateral_pre_align_stable_since = None
            aligned_forward_yaw = (
                self.pose_yaw if use_visual_feedback else target_yaw
            )
            initial_reference_yaw = self.duck_reference_yaw
            initial_reference_text = self._duck_reference_yaw_text()
            self.duck_reference_yaw = self._normalize_angle(aligned_forward_yaw)
            self.duck_reference_yaw_offset_rad = self._normalize_angle(
                self.duck_reference_yaw - self.pose_yaw
            )
            self.duck_reference_yaw_source = (
                f'limit_tag_pre_align_forward metric={metric},'
                f'tag_x={self.limit_bar_lateral_pre_align_source_tag_x_m:.3f}m,'
                f'tag_z={self.limit_bar_lateral_pre_align_source_tag_z_m:.3f}m,'
                f'visual_err={math.degrees(raw_error):.1f}deg,'
                f'initial_ref={initial_reference_text}'
            )
            self.slope_align_custom_target_yaw = self.duck_reference_yaw
            self.slope_align_enter_time = now
            self.slope_align_stable_since = None
            self.last_slope_align_log_time = 0.0
            shift_source = 'current_aligned_tag'
            shift_tag_x_m = sample.x_m
            shift_tag_z_m = sample.z_m
            shift_transform_text = ''
            locked_limit_bar_geometry = (
                self.state == self.STATE_LIMIT_BAR_ALIGN
                and self.limit_bar_locked_tag_valid
                and self.limit_bar_lateral_pre_align_source_odom_yaw is not None
                and self.limit_bar_lateral_pre_align_source_tag_x_m is not None
                and self.limit_bar_lateral_pre_align_source_tag_z_m is not None
                and math.isfinite(self.limit_bar_lateral_pre_align_source_tag_x_m)
                and math.isfinite(self.limit_bar_lateral_pre_align_source_tag_z_m)
            )
            if (
                (
                    locked_limit_bar_geometry
                    or (
                        locked_hurdle_sample
                        and self.state == self.STATE_HURDLE_ALIGN
                    )
                )
                and self.limit_bar_lateral_pre_align_source_odom_yaw is not None
                and self.limit_bar_lateral_pre_align_source_tag_x_m is not None
                and self.limit_bar_lateral_pre_align_source_tag_z_m is not None
                and math.isfinite(self.limit_bar_lateral_pre_align_source_tag_x_m)
                and math.isfinite(self.limit_bar_lateral_pre_align_source_tag_z_m)
            ):
                source_tag_x_m = self.limit_bar_lateral_pre_align_source_tag_x_m
                source_tag_z_m = self.limit_bar_lateral_pre_align_source_tag_z_m
                yaw_delta = self._normalize_angle(
                    aligned_forward_yaw
                    - self.limit_bar_lateral_pre_align_source_odom_yaw
                )
                rotated_tag_x_m, rotated_tag_z_m = _rotate_optical_xz_for_yaw_delta(
                    source_tag_x_m,
                    source_tag_z_m,
                    yaw_delta,
                )
                if (
                    math.isfinite(rotated_tag_x_m)
                    and math.isfinite(rotated_tag_z_m)
                    and rotated_tag_z_m > 1e-3
                ):
                    shift_source = (
                        'limit_bar_trigger_lock_yaw_compensated'
                        if locked_limit_bar_geometry
                        else 'hurdle_initial_lock_yaw_compensated'
                    )
                    shift_tag_x_m = rotated_tag_x_m
                    shift_tag_z_m = rotated_tag_z_m
                    shift_transform_text = (
                        f', yaw_delta={math.degrees(yaw_delta):.1f}deg, '
                        f'locked_tag_xz=({source_tag_x_m:.3f},'
                        f'{source_tag_z_m:.3f})m'
                    )
            self.limit_bar_lateral_shift_center_x_m = shift_tag_x_m + center_offset_m
            self.limit_bar_lateral_shift_center_z_m = shift_tag_z_m
            self._publish_feedback(
                f'{label} AprilTag 姿态预对齐完成: '
                f'metric={metric}, current_measured={math.degrees(measured):.1f}deg, '
                f'current_raw_err={math.degrees(raw_error):.1f}deg, '
                f'forward_yaw={math.degrees(aligned_forward_yaw):.1f}deg, '
                f'initial_ref_yaw={self._yaw_text(initial_reference_yaw)}, '
                f'base_yaw={math.degrees(self.duck_reference_yaw):.1f}deg, '
                f'tag_x={shift_tag_x_m:.3f}m, tag_z={shift_tag_z_m:.3f}m, '
                f'shift_source={shift_source}, shift_tag_x={shift_tag_x_m:.3f}m, '
                f'shift_tag_z={shift_tag_z_m:.3f}m, '
                f'center_x={self.limit_bar_lateral_shift_center_x_m:.3f}m'
                f'{shift_transform_text}。'
            )
            if self._start_limit_bar_lateral_shift(
                now,
                self.slope_align_reason,
                self._duck_reference_yaw_text(),
            ):
                return
            return

        direction = 'ccw' if command_error > 0.0 else 'cw'
        if (
            self.state == self.STATE_HURDLE_ALIGN
            and getattr(
                self,
                'hurdle_lateral_pre_align_independent_turn_enabled',
                False,
            )
        ):
            left_norm, right_norm = self._hurdle_fine_yaw_turn_norms(
                command_error
            )
        else:
            left_norm, right_norm = self._limit_bar_tag_turn_norms(command_error)
        self._publish_step_override(
            self.limit_bar_tag_align_mode,
            left_norm,
            right_norm,
        )
        if (now - self.last_slope_align_log_time) >= 0.5:
            self.last_slope_align_log_time = now
            self._publish_feedback(
                f'{label} AprilTag 姿态预对齐中: '
                f'metric={metric}, measured={math.degrees(measured):.1f}deg, '
                f'target={math.degrees(target):.1f}deg, '
                f'visual_err={math.degrees(raw_error):.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):.1f}deg, '
                f'feedback={"visual" if use_visual_feedback else "odom"}, '
                f'direction={direction}, hold={stable_sec:.2f}/'
                f'{hold_sec:.2f}s, '
                f'override=[{self.limit_bar_tag_align_mode},{left_norm:.3f},{right_norm:.3f}]。'
            )

    def _capture_limit_bar_tag_reference_yaw(self, now: float) -> bool:
        if self.pose_yaw is None:
            return False
        if not self.limit_bar_tag_align_enabled:
            return False
        recent_samples = self._recent_limit_bar_tag_samples(now)
        if len(recent_samples) < self.limit_bar_tag_min_samples:
            ok, along_m, estimated_tag_z = self._limit_bar_tag_handoff_progress(now)
            if ok:
                tag_x = self.limit_bar_tag_handoff_tag_x_m
                center_x = self.limit_bar_tag_handoff_center_x_m
                center_z = estimated_tag_z
                self._apply_limit_bar_tag_measurement(now, tag_x, center_z, center_x)
                self._lock_limit_bar_initial_tag(
                    now,
                    tag_x,
                    center_z,
                    center_x,
                    sample_count=1,
                    source='odom_handoff',
                )
                lateral_yaw_offset = (
                    self.limit_bar_tag_align_yaw_offset_sign
                    * math.atan2(center_x, center_z)
                )
                if self.limit_bar_tag_yaw_reference_mode == 'fixed_absolute':
                    target_yaw = self._limit_bar_fixed_reference_target_yaw()
                    yaw_offset = self._normalize_angle(target_yaw - self.pose_yaw)
                    yaw_reference_source = (
                        f'fixed_absolute={math.degrees(target_yaw):.1f}deg'
                    )
                else:
                    yaw_offset = lateral_yaw_offset
                    target_yaw = self._normalize_angle(self.pose_yaw + yaw_offset)
                    yaw_reference_source = 'tag_lateral'
                self.duck_reference_yaw = target_yaw
                self.duck_reference_yaw_offset_rad = yaw_offset
                self.duck_reference_yaw_source = (
                    f'limit_tag_odom_handoff '
                    f'source={self.limit_bar_tag_handoff_source or "unknown"},'
                    f'latched_tag_x={tag_x:.3f}m,'
                    f'latched_tag_z={self.limit_bar_tag_handoff_tag_z_m:.3f}m,'
                    f'odom_forward={along_m:.3f}m,'
                    f'est_tag_z={center_z:.3f}m,center_x={center_x:.3f}m,'
                    f'yaw_source={yaw_reference_source}'
                )
                self.limit_bar_tag_align_active = True
                self._publish_feedback(
                    f'限高杆 AprilTag 对中锁存: reason=odom_handoff_after_tag_lost, '
                    f'source={self.limit_bar_tag_handoff_source or "unknown"}, '
                    f'latched_tag_x={tag_x:.3f}m, '
                    f'latched_tag_z={self.limit_bar_tag_handoff_tag_z_m:.3f}m, '
                    f'odom_forward={along_m:.3f}m, est_tag_z={center_z:.3f}m, '
                    f'center_x={center_x:.3f}m, '
                    f'lateral_yaw_offset={math.degrees(lateral_yaw_offset):.1f}deg, '
                    f'yaw_offset={math.degrees(yaw_offset):.1f}deg, '
                    f'yaw_source={yaw_reference_source}, '
                    f'target_yaw={math.degrees(target_yaw):.1f}deg, '
                    f'lock_roll={math.degrees(self.limit_bar_tag_handoff_pose_roll_rad):.1f}deg, '
                    f'lock_pitch={math.degrees(self.limit_bar_tag_handoff_pose_pitch_rad):.1f}deg。'
                )
                return True
            latest_age_text = 'none'
            if self.limit_bar_tag_samples:
                latest_age_text = (
                    f'{now - self.limit_bar_tag_samples[-1].received_stamp_sec:.2f}s'
                )
            self._publish_feedback(
                f'限高杆 AprilTag 未用于对中: samples={len(recent_samples)}/'
                f'{self.limit_bar_tag_min_samples}, latest_age={latest_age_text}, '
                f'frame={self.limit_bar_tag_frame_id}->{self.limit_bar_tag_child_frame_id}。'
            )
            return False

        geometry_source = 'recent_median'
        geometry_sample_count = len(recent_samples)
        if self.limit_bar_tag_geometry_lock_enabled:
            selected_sample, geometry_sample_count, geometry_source = (
                self._select_limit_bar_geometry_lock_sample(recent_samples)
            )
            tag_x = selected_sample.x_m
            tag_z = selected_sample.z_m
        else:
            tag_x = self._median([sample.x_m for sample in recent_samples])
            tag_z = self._median([sample.z_m for sample in recent_samples])
        if not math.isfinite(tag_x) or not math.isfinite(tag_z) or tag_z <= 1e-3:
            self._publish_feedback(
                f'限高杆 AprilTag 未用于对中: tag_x={tag_x:.3f}, tag_z={tag_z:.3f} 无效。'
            )
            return False

        self.limit_detection.distance_m = tag_z
        self.limit_detection.lateral_m = tag_x
        self.limit_detection.nearest_stamp_sec = now

        center_x = tag_x + self.limit_bar_tag_to_center_x_m
        center_z = tag_z
        self._lock_limit_bar_initial_tag(
            now,
            tag_x,
            tag_z,
            center_x,
            sample_count=geometry_sample_count,
            source=geometry_source,
        )
        self.limit_bar_lateral_shift_center_x_m = center_x
        self.limit_bar_lateral_shift_center_z_m = center_z
        lateral_yaw_offset = (
            self.limit_bar_tag_align_yaw_offset_sign
            * math.atan2(center_x, center_z)
        )
        if self.limit_bar_tag_yaw_reference_mode == 'fixed_absolute':
            target_yaw = self._limit_bar_fixed_reference_target_yaw()
            yaw_offset = self._normalize_angle(target_yaw - self.pose_yaw)
            yaw_reference_source = (
                f'fixed_absolute={math.degrees(target_yaw):.1f}deg'
            )
        else:
            yaw_offset = lateral_yaw_offset
            target_yaw = self._normalize_angle(self.pose_yaw + yaw_offset)
            yaw_reference_source = 'tag_lateral'
        self._clear_duck_centerline()
        self.duck_reference_yaw = target_yaw
        self.duck_reference_yaw_offset_rad = yaw_offset
        self.duck_reference_yaw_source = (
            f'limit_tag tag_x={tag_x:.3f}m,tag_z={tag_z:.3f}m,'
            f'center_x={center_x:.3f}m,center_z={center_z:.3f}m,'
            f'yaw_source={yaw_reference_source}'
        )
        self.limit_bar_tag_align_active = True
        self._publish_feedback(
            f'限高杆 AprilTag 对中锁存: samples={len(recent_samples)}, '
            f'tag_x={tag_x:.3f}m, tag_z={tag_z:.3f}m, '
            f'center_x={center_x:.3f}m, '
            f'geometry_source={geometry_source}, '
            f'lateral_yaw_offset={math.degrees(lateral_yaw_offset):.1f}deg, '
            f'yaw_offset={math.degrees(yaw_offset):.1f}deg, '
            f'yaw_source={yaw_reference_source}, '
            f'target_yaw={math.degrees(target_yaw):.1f}deg。'
        )
        return True

    def _capture_hurdle_tag_reference_yaw(self, now: float) -> bool:
        if self.pose_yaw is None:
            return False
        if not self.hurdle_tag_trigger_enabled:
            return False
        locked_or_latest = self._hurdle_locked_or_latest_tag(now)
        if locked_or_latest is None:
            recent_samples = self._recent_hurdle_tag_samples(now)
            ok, along_m, estimated_tag_z = self._hurdle_tag_handoff_progress(now)
            if ok:
                tag_x = self.hurdle_tag_handoff_tag_x_m
                center_x = self.hurdle_tag_handoff_center_x_m
                center_z = estimated_tag_z
                self._apply_hurdle_tag_measurement(now, tag_x, center_z, center_x)
                self._lock_hurdle_initial_tag(
                    now,
                    tag_x,
                    center_z,
                    center_x,
                    source='odom_handoff_align',
                )
                lateral_yaw_offset = (
                    self.limit_bar_tag_align_yaw_offset_sign
                    * math.atan2(center_x, center_z)
                )
                if self.hurdle_tag_yaw_reference_mode == 'fixed_absolute':
                    target_yaw = self._hurdle_fixed_reference_target_yaw()
                    yaw_offset = self._normalize_angle(target_yaw - self.pose_yaw)
                    yaw_reference_source = (
                        f'fixed_absolute={math.degrees(target_yaw):.1f}deg'
                    )
                else:
                    yaw_offset = lateral_yaw_offset
                    target_yaw = self._normalize_angle(self.pose_yaw + yaw_offset)
                    yaw_reference_source = 'tag_lateral'
                self._clear_duck_centerline()
                self.duck_reference_yaw = target_yaw
                self.duck_reference_yaw_offset_rad = yaw_offset
                self.duck_reference_yaw_source = (
                    f'hurdle_tag_odom_handoff '
                    f'latched_tag_x={tag_x:.3f}m,'
                    f'latched_tag_z={self.hurdle_tag_handoff_tag_z_m:.3f}m,'
                    f'odom_forward={along_m:.3f}m,'
                    f'est_tag_z={center_z:.3f}m,center_x={center_x:.3f}m,'
                    f'yaw_source={yaw_reference_source}'
                )
                self.limit_bar_tag_align_active = True
                self._publish_feedback(
                    f'高墙 AprilTag 对中锁存: reason=odom_handoff_after_tag_lost, '
                    f'latched_tag_x={tag_x:.3f}m, '
                    f'latched_tag_z={self.hurdle_tag_handoff_tag_z_m:.3f}m, '
                    f'odom_forward={along_m:.3f}m, est_tag_z={center_z:.3f}m, '
                    f'center_x={center_x:.3f}m, '
                    f'lateral_yaw_offset={math.degrees(lateral_yaw_offset):.1f}deg, '
                    f'yaw_offset={math.degrees(yaw_offset):.1f}deg, '
                    f'target_yaw={math.degrees(target_yaw):.1f}deg。'
                )
                return True
            latest_age_text = 'none'
            if self.hurdle_tag_samples:
                latest_age_text = (
                    f'{now - self.hurdle_tag_samples[-1].received_stamp_sec:.2f}s'
                )
            self._publish_feedback(
                f'高墙 AprilTag 未用于对中: samples={len(recent_samples)}/'
                f'{self.hurdle_tag_min_samples}, latest_age={latest_age_text}, '
                f'frame={self.hurdle_tag_frame_id}->{self.hurdle_tag_child_frame_id}。'
            )
            return False

        tag_x, tag_z, center_x, sample_count, tag_source = locked_or_latest
        if not math.isfinite(tag_x) or not math.isfinite(tag_z) or tag_z <= 1e-3:
            self._publish_feedback(
                f'高墙 AprilTag 未用于对中: tag_x={tag_x:.3f}, tag_z={tag_z:.3f} 无效。'
            )
            return False

        center_z = tag_z
        self._apply_hurdle_tag_measurement(now, tag_x, tag_z, center_x)
        self._lock_hurdle_initial_tag(
            now,
            tag_x,
            tag_z,
            center_x,
            source=f'align_{tag_source}',
        )
        lateral_yaw_offset = self.limit_bar_tag_align_yaw_offset_sign * math.atan2(
            center_x,
            center_z,
        )
        if self.hurdle_tag_yaw_reference_mode == 'fixed_absolute':
            target_yaw = self._hurdle_fixed_reference_target_yaw()
            yaw_offset = self._normalize_angle(target_yaw - self.pose_yaw)
            yaw_reference_source = (
                f'fixed_absolute={math.degrees(target_yaw):.1f}deg'
            )
        else:
            yaw_offset = lateral_yaw_offset
            target_yaw = self._normalize_angle(self.pose_yaw + yaw_offset)
            yaw_reference_source = 'tag_lateral'
        self._clear_duck_centerline()
        self.duck_reference_yaw = target_yaw
        self.duck_reference_yaw_offset_rad = yaw_offset
        self.duck_reference_yaw_source = (
            f'hurdle_tag tag_x={tag_x:.3f}m,tag_z={tag_z:.3f}m,'
            f'center_x={center_x:.3f}m,center_z={center_z:.3f}m,'
            f'yaw_source={yaw_reference_source}'
        )
        self.limit_bar_tag_align_active = True
        self._publish_feedback(
            f'高墙 AprilTag 对中锁存: source={tag_source}, samples={sample_count}, '
            f'tag_x={tag_x:.3f}m, tag_z={tag_z:.3f}m, '
            f'center_x={center_x:.3f}m, '
            f'lateral_yaw_offset={math.degrees(lateral_yaw_offset):.1f}deg, '
            f'yaw_offset={math.degrees(yaw_offset):.1f}deg, '
            f'target_yaw={math.degrees(target_yaw):.1f}deg。'
        )
        return True

    def _capture_upstairs_tag_reference_yaw(self, now: float) -> bool:
        if self.pose_yaw is None:
            return False
        if not self.upstairs_tag_trigger_enabled:
            return False
        recent_samples = self._recent_upstairs_tag_samples(now)
        if len(recent_samples) < self.upstairs_tag_min_samples:
            ok, along_m, estimated_tag_z = self._upstairs_tag_handoff_progress(now)
            if ok:
                tag_x = self.upstairs_tag_handoff_tag_x_m
                center_x = self.upstairs_tag_handoff_center_x_m
                center_z = estimated_tag_z
                self._apply_upstairs_tag_measurement(now, tag_x, center_z, center_x)
                lateral_yaw_offset = (
                    self.limit_bar_tag_align_yaw_offset_sign
                    * math.atan2(center_x, center_z)
                )
                if self.upstairs_tag_yaw_reference_mode == 'fixed_absolute':
                    target_yaw = self._upstairs_fixed_reference_target_yaw()
                    yaw_offset = self._normalize_angle(target_yaw - self.pose_yaw)
                    yaw_reference_source = (
                        'fixed_absolute='
                        f'{math.degrees(target_yaw):.1f}deg'
                    )
                else:
                    yaw_offset = (
                        lateral_yaw_offset
                        if self.upstairs_tag_yaw_align_from_lateral_enabled
                        else 0.0
                    )
                    target_yaw = self._normalize_angle(self.pose_yaw + yaw_offset)
                    yaw_reference_source = (
                        'tag_lateral'
                        if self.upstairs_tag_yaw_align_from_lateral_enabled
                        else 'current_yaw'
                    )
                self._clear_duck_centerline()
                self.duck_reference_yaw = target_yaw
                self.duck_reference_yaw_offset_rad = yaw_offset
                self.duck_reference_yaw_source = (
                    f'upstairs_tag_odom_handoff '
                    f'latched_tag_x={tag_x:.3f}m,'
                    f'latched_tag_z={self.upstairs_tag_handoff_tag_z_m:.3f}m,'
                    f'odom_forward={along_m:.3f}m,'
                    f'est_tag_z={center_z:.3f}m,center_x={center_x:.3f}m,'
                    f'yaw_source={yaw_reference_source}'
                )
                self.limit_bar_tag_align_active = True
                self._publish_feedback(
                    f'上台阶 AprilTag 对中锁存: reason=odom_handoff_after_tag_lost, '
                    f'latched_tag_x={tag_x:.3f}m, '
                    f'latched_tag_z={self.upstairs_tag_handoff_tag_z_m:.3f}m, '
                    f'odom_forward={along_m:.3f}m, est_tag_z={center_z:.3f}m, '
                    f'center_x={center_x:.3f}m, '
                    f'lateral_yaw_offset={math.degrees(lateral_yaw_offset):.1f}deg, '
                    f'yaw_offset={math.degrees(yaw_offset):.1f}deg, '
                    f'target_yaw={math.degrees(target_yaw):.1f}deg, '
                    f'yaw_source={yaw_reference_source}。'
                )
                return True
            latest_age_text = 'none'
            if self.upstairs_tag_samples:
                latest_age_text = (
                    f'{now - self.upstairs_tag_samples[-1].received_stamp_sec:.2f}s'
                )
            self._publish_feedback(
                f'上台阶 AprilTag 未用于对中: samples={len(recent_samples)}/'
                f'{self.upstairs_tag_min_samples}, latest_age={latest_age_text}, '
                f'frame={self.upstairs_tag_frame_id}->{self.upstairs_tag_child_frame_id}。'
            )
            return False

        tag_x = self._median([sample.x_m for sample in recent_samples])
        tag_z = self._median([sample.z_m for sample in recent_samples])
        if not math.isfinite(tag_x) or not math.isfinite(tag_z) or tag_z <= 1e-3:
            self._publish_feedback(
                f'上台阶 AprilTag 未用于对中: tag_x={tag_x:.3f}, tag_z={tag_z:.3f} 无效。'
            )
            return False

        self.upstairs_detection.distance_m = tag_z
        self.upstairs_detection.lateral_m = tag_x
        self.upstairs_detection.nearest_stamp_sec = now

        center_x = tag_x + self.upstairs_tag_to_center_x_m
        center_z = tag_z
        self._apply_upstairs_tag_measurement(now, tag_x, center_z, center_x)
        lateral_yaw_offset = self.limit_bar_tag_align_yaw_offset_sign * math.atan2(
            center_x,
            center_z,
        )
        if self.upstairs_tag_yaw_reference_mode == 'fixed_absolute':
            target_yaw = self._upstairs_fixed_reference_target_yaw()
            yaw_offset = self._normalize_angle(target_yaw - self.pose_yaw)
            yaw_reference_source = (
                'fixed_absolute='
                f'{math.degrees(target_yaw):.1f}deg'
            )
        else:
            yaw_offset = (
                lateral_yaw_offset
                if self.upstairs_tag_yaw_align_from_lateral_enabled
                else 0.0
            )
            target_yaw = self._normalize_angle(self.pose_yaw + yaw_offset)
            yaw_reference_source = (
                'tag_lateral'
                if self.upstairs_tag_yaw_align_from_lateral_enabled
                else 'current_yaw'
            )
        self._clear_duck_centerline()
        self.duck_reference_yaw = target_yaw
        self.duck_reference_yaw_offset_rad = yaw_offset
        self.duck_reference_yaw_source = (
            f'upstairs_tag tag_x={tag_x:.3f}m,tag_z={tag_z:.3f}m,'
            f'center_x={center_x:.3f}m,center_z={center_z:.3f}m,'
            f'yaw_source={yaw_reference_source}'
        )
        self.limit_bar_tag_align_active = True
        self._publish_feedback(
            f'上台阶 AprilTag 对中锁存: samples={len(recent_samples)}, '
            f'tag_x={tag_x:.3f}m, tag_z={tag_z:.3f}m, '
            f'center_x={center_x:.3f}m, '
            f'lateral_yaw_offset={math.degrees(lateral_yaw_offset):.1f}deg, '
            f'yaw_offset={math.degrees(yaw_offset):.1f}deg, '
            f'target_yaw={math.degrees(target_yaw):.1f}deg, '
            f'yaw_source={yaw_reference_source}。'
        )
        return True

    def _clear_duck_centerline(self) -> None:
        self.limit_bar_tag_align_active = False
        self.limit_bar_lateral_pre_align_active = False
        self.limit_bar_lateral_pre_align_done = False
        self.limit_bar_lateral_pre_align_start_sec = None
        self.limit_bar_lateral_pre_align_stable_since = None
        self.limit_bar_lateral_pre_align_target_yaw = None
        self.limit_bar_lateral_pre_align_source_odom_yaw = None
        self.upstairs_final_align_latched_along_m = None
        self.upstairs_final_align_latched_raw_along_m = None
        self.upstairs_final_align_latched_remaining_m = None
        self.upstairs_final_align_latched_reason = ''
        self.upstairs_final_align_latched_live_text = ''
        self.duck_centerline_locked = False
        self.duck_centerline_start_x = None
        self.duck_centerline_start_y = None
        self.duck_centerline_yaw = None
        self.duck_centerline_goal_x = None
        self.duck_centerline_goal_y = None

    def _lock_duck_centerline(self, now: float, yaw: float, source: str) -> bool:
        if self.pose_x is None or self.pose_y is None:
            self._publish_feedback('限高杆中心线锁定失败: 当前没有可用 Odometry。')
            return False
        centerline_yaw = self._normalize_angle(yaw)
        forward_x, forward_y = self._odom_forward_unit(centerline_yaw)
        goal_x = self.pose_x + self.duck_travel_distance_m * forward_x
        goal_y = self.pose_y + self.duck_travel_distance_m * forward_y
        self.duck_centerline_start_x = self.pose_x
        self.duck_centerline_start_y = self.pose_y
        self.duck_centerline_yaw = centerline_yaw
        self.duck_centerline_goal_x = goal_x
        self.duck_centerline_goal_y = goal_y
        self.duck_centerline_locked = True
        self.duck_start_x = self.pose_x
        self.duck_start_y = self.pose_y
        self._publish_feedback(
            f'限高杆中心线已锁定({source}): '
            f'start=({self.duck_centerline_start_x:.3f},{self.duck_centerline_start_y:.3f}), '
            f'goal=({goal_x:.3f},{goal_y:.3f}), '
            f'centerline_yaw={math.degrees(centerline_yaw):.1f}deg, '
            f'odom_forward_axis={self.duck_centerline_odom_forward_axis}, '
            f'distance={self.duck_travel_distance_m:.3f}m, stamp={now:.3f}。'
        )
        return True

    def _odom_forward_unit(self, yaw_rad: float) -> tuple[float, float]:
        if self.duck_centerline_odom_forward_axis == 'y':
            return -math.sin(yaw_rad), math.cos(yaw_rad)
        return math.cos(yaw_rad), math.sin(yaw_rad)

    def _odom_right_unit(self, yaw_rad: float) -> tuple[float, float]:
        if self.duck_centerline_odom_forward_axis == 'y':
            return math.cos(yaw_rad), math.sin(yaw_rad)
        return math.sin(yaw_rad), -math.cos(yaw_rad)

    def _duck_centerline_errors(self) -> tuple[Optional[float], Optional[float], Optional[float]]:
        if (
            self.pose_x is None or self.pose_y is None or self.pose_yaw is None
            or self.duck_centerline_start_x is None
            or self.duck_centerline_start_y is None
            or self.duck_centerline_yaw is None
        ):
            return None, None, None
        dx = self.pose_x - self.duck_centerline_start_x
        dy = self.pose_y - self.duck_centerline_start_y
        forward_x, forward_y = self._odom_forward_unit(self.duck_centerline_yaw)
        right_x, right_y = self._odom_right_unit(self.duck_centerline_yaw)
        along_m = dx * forward_x + dy * forward_y
        lateral_m = dx * right_x + dy * right_y
        yaw_error = self._normalize_angle(self.duck_centerline_yaw - self.pose_yaw)
        if (
            not math.isfinite(along_m)
            or not math.isfinite(lateral_m)
            or not math.isfinite(yaw_error)
        ):
            return None, None, None
        return along_m, lateral_m, yaw_error

    def _duck_centerline_corrected_norms(
        self,
        base_left_norm: float,
        base_right_norm: float,
    ) -> tuple[float, float, Optional[float], Optional[float], Optional[float], float]:
        along_m, lateral_m, yaw_error = self._duck_centerline_errors()
        if along_m is None or lateral_m is None or yaw_error is None:
            return base_left_norm, base_right_norm, along_m, lateral_m, yaw_error, 0.0

        lateral_for_control = 0.0
        if abs(lateral_m) > self.duck_centerline_deadband_m:
            lateral_for_control = lateral_m
        yaw_for_control = 0.0
        if abs(yaw_error) > self.duck_centerline_yaw_deadband_rad:
            yaw_for_control = yaw_error
        delta_norm = (
            self.duck_centerline_lateral_gain_norm_per_m * lateral_for_control
            + self.duck_centerline_yaw_gain_norm_per_rad * yaw_for_control
        )
        delta_norm = max(
            -self.duck_centerline_max_delta_norm,
            min(self.duck_centerline_max_delta_norm, delta_norm),
        )
        left_norm = self._clamp_norm(base_left_norm - delta_norm)
        right_norm = self._clamp_norm(base_right_norm + delta_norm)
        return left_norm, right_norm, along_m, lateral_m, yaw_error, delta_norm

    def _duck_centerline_failure_reason(
        self,
        lateral_m: Optional[float],
        yaw_error: Optional[float],
    ) -> Optional[str]:
        if lateral_m is None or yaw_error is None:
            return None
        if (
            self.duck_centerline_lateral_fail_m > 0.0
            and abs(lateral_m) > self.duck_centerline_lateral_fail_m
        ):
            return (
                f'限高杆中心线偏差过大: lateral_error={lateral_m:.3f}m > '
                f'{self.duck_centerline_lateral_fail_m:.3f}m。'
            )
        if (
            self.duck_centerline_yaw_fail_rad > 0.0
            and abs(yaw_error) > self.duck_centerline_yaw_fail_rad
        ):
            return (
                f'限高杆中心线 yaw 偏差过大: yaw_error={math.degrees(yaw_error):.1f}deg > '
                f'{math.degrees(self.duck_centerline_yaw_fail_rad):.1f}deg。'
            )
        return None

    def _capture_duck_reference_yaw(self, now: Optional[float] = None) -> bool:
        if self.pose_yaw is None:
            return False
        if now is None:
            now = self._now_sec()
        self._clear_duck_centerline()
        if self._capture_limit_bar_tag_reference_yaw(now):
            return True
        self.duck_reference_yaw = self.pose_yaw
        self.duck_reference_yaw_source = 'current_yaw'
        self.duck_reference_yaw_offset_rad = 0.0
        self.limit_bar_tag_align_active = False

        if not self.duck_visual_yaw_align_enabled:
            return True
        if (
            self.limit_detection.nearest_stamp_sec <= 0.0
            or (now - self.limit_detection.nearest_stamp_sec)
            > self.duck_visual_yaw_stale_timeout_sec
        ):
            self.duck_reference_yaw_source = 'current_yaw(no_fresh_limit_nearest)'
            return True
        distance = self._usable_distance(self.limit_detection.distance_m)
        lateral = self.limit_detection.lateral_m
        if (
            distance is None
            or distance < self.duck_visual_yaw_min_distance_m
            or lateral is None
            or not math.isfinite(lateral)
        ):
            self.duck_reference_yaw_source = 'current_yaw(no_valid_limit_lateral)'
            return True

        offset = self.duck_visual_yaw_lateral_sign * math.atan2(lateral, distance)
        if self.duck_visual_yaw_max_offset_rad > 0.0:
            offset = max(
                -self.duck_visual_yaw_max_offset_rad,
                min(self.duck_visual_yaw_max_offset_rad, offset),
            )
        self.duck_reference_yaw = self._normalize_angle(self.pose_yaw + offset)
        self.duck_reference_yaw_offset_rad = offset
        self.duck_reference_yaw_source = (
            f'limit_lateral={lateral:.3f}m,dist={distance:.3f}m'
        )
        return True

    def _duck_reference_yaw_text(self) -> str:
        if self.duck_reference_yaw is None:
            return 'unknown'
        text = self._yaw_text(self.duck_reference_yaw)
        if self.duck_reference_yaw_source:
            text += f'({self.duck_reference_yaw_source}'
            if abs(self.duck_reference_yaw_offset_rad) > 1e-6:
                text += f',offset={math.degrees(self.duck_reference_yaw_offset_rad):.1f}deg'
            text += ')'
        return text

    def _duck_corrected_norms(
        self,
        base_left_norm: float,
        base_right_norm: float,
    ) -> tuple[float, float, Optional[float], float]:
        left_norm = base_left_norm
        right_norm = base_right_norm
        yaw_error_deg: Optional[float] = None
        yaw_correction_norm = 0.0
        if (
            not self.duck_yaw_correction_enabled
            or self.duck_reference_yaw is None
            or self.pose_yaw is None
        ):
            return left_norm, right_norm, yaw_error_deg, yaw_correction_norm

        yaw_error = self._normalize_angle(self.duck_reference_yaw - self.pose_yaw)
        yaw_error_deg = math.degrees(yaw_error)
        if abs(yaw_error) <= self.duck_yaw_correction_tolerance_rad:
            return left_norm, right_norm, 0.0, yaw_correction_norm

        if self.duck_yaw_correction_force_max_delta:
            yaw_correction_norm = self.duck_yaw_correction_max_delta_norm
        else:
            yaw_correction_norm = min(
                self.duck_yaw_correction_max_delta_norm,
                self.duck_yaw_correction_gain_per_rad * abs(yaw_error),
            )
        signed_yaw_error = self.duck_yaw_correction_sign * yaw_error
        if signed_yaw_error > 0.0:
            left_norm = self._clamp_norm(left_norm - yaw_correction_norm)
            right_norm = self._clamp_norm(right_norm + yaw_correction_norm)
        else:
            left_norm = self._clamp_norm(left_norm + yaw_correction_norm)
            right_norm = self._clamp_norm(right_norm - yaw_correction_norm)
        return left_norm, right_norm, yaw_error_deg, yaw_correction_norm

    def _reset_detection_states(self) -> None:
        self.limit_detection.reset()
        self.hurdle_detection.reset()
        self.hurdle_visual_detection.reset()
        self.pole_detection.reset()
        self.pole_apriltag_detection.reset()
        self.pole_apriltag_last_seen_stamp_sec = 0.0
        self._clear_pole_apriltag_handoff()
        self.pole_apriltag_ready_source = ''
        self.last_pole_apriltag_handoff_log_sec = 0.0
        self.slope_detection.reset()
        self.upstairs_detection.reset()
        self.upstairs_visual_detection.reset()
        self.sandpit_detection.reset()

    def _log_duck_progress(
        self,
        now: float,
        travel_m: Optional[float],
        yaw_error_deg: Optional[float],
        yaw_correction_norm: float,
        *,
        lateral_error_m: Optional[float] = None,
        left_norm: Optional[float] = None,
        right_norm: Optional[float] = None,
        correction_kind: str = 'yaw',
        extra_text: str = '',
        progress_label: str = 'travel',
    ) -> None:
        if self.duck_progress_log_interval_sec <= 0.0:
            return
        if (now - self.last_duck_progress_log_time) < self.duck_progress_log_interval_sec:
            return
        self.last_duck_progress_log_time = now
        if travel_m is None:
            self._publish_feedback('限高杆趴走中: 当前 Odometry/起点无效，继续保持趴走。')
            return
        timeout_left = 0.0
        if self.phase_deadline_sec is not None:
            timeout_left = max(0.0, self.phase_deadline_sec - now)
        yaw_text = 'unknown'
        if yaw_error_deg is not None:
            yaw_text = f'{yaw_error_deg:.1f}deg'
        left_text = 'unknown' if left_norm is None else f'{left_norm:.3f}'
        right_text = 'unknown' if right_norm is None else f'{right_norm:.3f}'
        if correction_kind == 'centerline':
            lateral_text = 'unknown'
            if lateral_error_m is not None:
                lateral_text = f'{lateral_error_m:.3f}m'
            goal_text = 'unknown'
            if (
                self.duck_centerline_goal_x is not None
                and self.duck_centerline_goal_y is not None
            ):
                goal_text = (
                    f'({self.duck_centerline_goal_x:.3f},'
                    f'{self.duck_centerline_goal_y:.3f})'
                )
            self._publish_feedback(
                f'限高杆中心线趴走中: along={travel_m:.3f}/{self.duck_travel_distance_m:.3f}m, '
                f'lateral_error={lateral_text}, yaw_error={yaw_text}, '
                f'delta={yaw_correction_norm:.3f}, '
                f'override=[{self.duck_walk_mode},{left_text},{right_text}], '
                f'start=({self.duck_centerline_start_x:.3f},{self.duck_centerline_start_y:.3f}), '
                f'goal={goal_text}, timeout_left={timeout_left:.1f}s{extra_text}。'
            )
            return
        self._publish_feedback(
            f'限高杆趴走中: '
            f'{progress_label}={travel_m:.3f}/{self.duck_travel_distance_m:.3f}m, '
            f'pose=({self.pose_x:.3f},{self.pose_y:.3f}), '
            f'start=({self.duck_start_x:.3f},{self.duck_start_y:.3f}), '
            f'yaw_ref={self._duck_reference_yaw_text()}, '
            f'yaw_err={yaw_text}, yaw_corr={yaw_correction_norm:.3f}, '
            f'override=[{self.duck_walk_mode},{left_text},{right_text}], '
            f'timeout_left={timeout_left:.1f}s{extra_text}。'
        )

    def _publish_step_override(self, mode: int, left_norm: float, right_norm: float) -> None:
        payload = f'{mode}:{left_norm:.3f}:{right_norm:.3f}'
        if payload == self.last_step_override:
            msg = Float32MultiArray()
            msg.data = [float(mode), float(left_norm), float(right_norm)]
            self.step_override_pub.publish(msg)
            return
        self.last_step_override = payload
        msg = Float32MultiArray()
        msg.data = [float(mode), float(left_norm), float(right_norm)]
        self.step_override_pub.publish(msg)

    def _clear_step_override(self) -> None:
        if self.last_step_override == 'CLEARED':
            return
        self.last_step_override = 'CLEARED'
        if not rclpy.ok():
            return
        msg = Float32MultiArray()
        msg.data = [-1.0, 0.0, 0.0]
        try:
            self.step_override_pub.publish(msg)
        except Exception:
            pass

    def _publish_nav_arrival(self, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        self.arrival_pub.publish(msg)

    def _publish_step_once(self, mode: int, left_norm: float, right_norm: float) -> None:
        msg = Float32MultiArray()
        msg.data = [float(mode), float(left_norm), float(right_norm)]
        self.step_once_pub.publish(msg)

    def _publish_pending_hurdle_jump_once_repeats(self, now: float) -> None:
        while (
            self.pending_hurdle_jump_once_repeats > 0
            and now >= self.next_hurdle_jump_once_repeat_sec
        ):
            self._publish_step_once(
                self.hurdle_jump_mode,
                self.hurdle_jump_left_norm,
                self.hurdle_jump_right_norm,
            )
            self.pending_hurdle_jump_once_repeats -= 1
            self.next_hurdle_jump_once_repeat_sec += (
                self.hurdle_jump_step_once_repeat_interval_sec
            )
            if self.hurdle_jump_step_once_repeat_interval_sec <= 0.0:
                continue
            break

    @staticmethod
    def _publish_bool(pub, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        pub.publish(msg)

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

    def _fail(self, text: str) -> None:
        self.failed_from_state = self.state
        self.failed_from_branch = self.current_active_branch
        self.last_failure_text = text
        self.state = self.STATE_FAILED
        self.current_active_branch = ''
        if self.failed_stop_nav_executor_enabled:
            self._publish_nav_arrival(True)
        if self.failed_hold_step_override_enabled:
            self._run_failed()
        else:
            self._clear_step_override()
        self._publish_state(self.state)
        self._publish_feedback(text)

    @staticmethod
    def _distance_from_array(msg: Float32MultiArray) -> Optional[float]:
        return ObstacleRaceSupervisor._value_from_array(msg, 0)

    @staticmethod
    def _value_from_array(msg: Float32MultiArray, index: int) -> Optional[float]:
        if index < 0 or len(msg.data) <= index:
            return None
        value = float(msg.data[index])
        if not math.isfinite(value):
            return None
        return value

    @staticmethod
    def _distance_text(distance_m: Optional[float]) -> str:
        if distance_m is None:
            return 'unknown'
        return f'{distance_m:.3f}m'

    @staticmethod
    def _yaw_text(yaw_rad: Optional[float]) -> str:
        if yaw_rad is None:
            return 'unknown'
        return f'{math.degrees(yaw_rad):.1f}deg'

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def _roll_from_quaternion(q) -> float:
        return math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        )

    @staticmethod
    def _rpy_from_quaternion(q) -> tuple[float, float, float]:
        roll = math.atan2(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        )
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
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
        return max(-1.0, min(1.0, value))

    def _pose_is_fresh(self, now: float) -> bool:
        if self.pose_x is None or self.pose_y is None or self.pose_yaw is None:
            return False
        if self.pose_stale_timeout_sec <= 0.0:
            return True
        return (now - self.pose_stamp_sec) <= self.pose_stale_timeout_sec

    def _publish_slope_align_reference(
        self,
        now: float,
        reference_yaw: float,
        align_duration_sec: float = 0.0,
    ) -> None:
        if not self._pose_is_fresh(now):
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.pose_frame or 'camera_init'
        msg.pose.position.x = float(self.pose_x)
        msg.pose.position.y = float(self.pose_y)
        msg.pose.position.z = max(0.0, float(align_duration_sec))
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(reference_yaw * 0.5)
        msg.pose.orientation.w = math.cos(reference_yaw * 0.5)
        self.slope_align_reference_yaw = reference_yaw
        self.slope_align_reference_frame = msg.header.frame_id
        self.slope_align_reference_stamp_sec = now
        self.slope_align_reference_pub.publish(msg)
        label = '限高杆对齐参考' if self.state == self.STATE_LIMIT_BAR_ALIGN else '斜坡对齐参考'
        self._publish_feedback(
            f'锁存{label}: target_yaw={math.degrees(reference_yaw):.1f}deg, '
            f'pose_yaw={math.degrees(self.pose_yaw):.1f}deg, '
            f'pose=({self.pose_x:.3f},{self.pose_y:.3f}), '
            f'align_duration={align_duration_sec:.2f}s, frame={msg.header.frame_id}。'
        )

    def _publish_slope_route_yaw_reference(
        self,
        now: float,
        reference_yaw: float,
    ) -> None:
        if not self._pose_is_fresh(now):
            return
        msg = self._yaw_reference_pose(reference_yaw)
        self.slope_route_yaw_reference = self._normalize_angle(reference_yaw)
        self.slope_route_yaw_reference_frame = msg.header.frame_id
        self.slope_route_yaw_reference_stamp_sec = now
        self.slope_route_yaw_reference_pub.publish(msg)
        self._publish_feedback(
            f'发布斜坡路线 0deg 基准: '
            f'yaw={math.degrees(self.slope_route_yaw_reference):.1f}deg, '
            f'pose=({self.pose_x:.3f},{self.pose_y:.3f}), '
            f'frame={msg.header.frame_id}。'
        )

    def _publish_slope_bridge_retry_yaw_reference(
        self,
        now: float,
        reference_yaw: float,
    ) -> None:
        if not self._pose_is_fresh(now):
            return
        msg = self._yaw_reference_pose(reference_yaw)
        self.slope_bridge_retry_yaw_reference = self._normalize_angle(reference_yaw)
        self.slope_bridge_retry_yaw_reference_frame = msg.header.frame_id
        self.slope_bridge_retry_yaw_reference_stamp_sec = now
        self.slope_bridge_retry_yaw_reference_pub.publish(msg)
        self._publish_feedback(
            f'发布入桥重试方向基准: '
            f'yaw={math.degrees(self.slope_bridge_retry_yaw_reference):.1f}deg, '
            f'pose=({self.pose_x:.3f},{self.pose_y:.3f}), '
            f'frame={msg.header.frame_id}。'
        )

    def _yaw_reference_pose(self, reference_yaw: float) -> PoseStamped:
        normalized_yaw = self._normalize_angle(reference_yaw)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.pose_frame or 'camera_init'
        msg.pose.position.x = float(self.pose_x)
        msg.pose.position.y = float(self.pose_y)
        msg.pose.orientation.z = math.sin(normalized_yaw * 0.5)
        msg.pose.orientation.w = math.cos(normalized_yaw * 0.5)
        return msg

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleRaceSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
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
