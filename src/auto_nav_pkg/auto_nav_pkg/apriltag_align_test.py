#!/usr/bin/env python3
"""Standalone AprilTag alignment and approach test node."""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from auto_nav_pkg.log_style import colorize_log
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, Int32, String
from tf2_msgs.msg import TFMessage


@dataclass
class TagPose:
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
class DetectionInfo:
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


class AprilTagAlignTest(Node):
    """Read AprilTag TF and optionally command yaw and forward corrections."""

    def __init__(self) -> None:
        super().__init__('apriltag_align_test')

        self.declare_parameter('tf_topic', '/tf')
        self.declare_parameter('detections_topic', '/apriltag/detections')
        self.declare_parameter('cmd_vel_topic', '/race_cmd_vel')
        self.declare_parameter('pose_topic', '/Odometry')
        self.declare_parameter('mode_topic', '/dog_mode_current')
        self.declare_parameter('step_override_topic', '/serial_step_override')
        self.declare_parameter('feedback_topic', '/apriltag_align_test/feedback_log')
        self.declare_parameter('state_topic', '/apriltag_align_test/state')
        self.declare_parameter('debug_topic', '/apriltag_align_test/debug')

        self.declare_parameter('target_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('target_child_frame_id', 'object')
        self.declare_parameter('target_tag_id', 14)
        self.declare_parameter('target_offset_x_m', 0.0)
        self.declare_parameter('target_tag_x_m', math.nan)
        self.declare_parameter('robot_center_offset_x_m', 0.0)
        self.declare_parameter('alignment_mode', 'center_x')

        self.declare_parameter('enable_control', False)
        self.declare_parameter('control_output_mode', 'hybrid')
        self.declare_parameter('control_mode', 0)
        self.declare_parameter('control_rate_hz', 15.0)
        self.declare_parameter('control_sign', 1.0)
        self.declare_parameter('center_control_sign', -1.0)
        self.declare_parameter('pose_control_sign', 1.0)
        self.declare_parameter('target_center_angle_deg', 0.0)
        self.declare_parameter('target_pose_roll_deg', 0.0)
        self.declare_parameter('target_pose_pitch_deg', 0.0)
        self.declare_parameter('target_pose_yaw_deg', 0.0)
        self.declare_parameter('yaw_deadband_deg', 1.5)
        self.declare_parameter('yaw_kp', 0.90)
        self.declare_parameter('min_abs_angular_z', 0.05)
        self.declare_parameter('max_abs_angular_z', 0.25)
        self.declare_parameter('enable_forward_approach', False)
        self.declare_parameter('target_distance_m', 0.45)
        self.declare_parameter('distance_deadband_m', 0.03)
        self.declare_parameter('forward_kp', 0.35)
        self.declare_parameter('min_forward_speed_mps', 0.03)
        self.declare_parameter('max_forward_speed_mps', 0.12)
        self.declare_parameter('forward_max_yaw_error_deg', 3.0)
        self.declare_parameter('approach_requires_aligned', False)
        self.declare_parameter('approach_distance_axis', 'z')
        self.declare_parameter('bridge_lateral_calibration_enabled', False)
        self.declare_parameter('bridge_lateral_deadband_m', 0.03)
        self.declare_parameter('bridge_lateral_hold_sec', 0.25)
        self.declare_parameter('bridge_lateral_forward_speed_mps', 0.05)
        self.declare_parameter('bridge_lateral_kp_radps_per_m', 1.5)
        self.declare_parameter('bridge_lateral_min_abs_angular_z', 0.02)
        self.declare_parameter('bridge_lateral_max_abs_angular_z', 0.18)
        self.declare_parameter('bridge_lateral_control_sign', -1.0)
        self.declare_parameter('bridge_lateral_use_step_override', True)
        self.declare_parameter('bridge_lateral_step_override_mode', 0)
        self.declare_parameter('bridge_lateral_left_error_left_norm', -0.10)
        self.declare_parameter('bridge_lateral_left_error_right_norm', 0.20)
        self.declare_parameter('bridge_lateral_right_error_left_norm', 0.20)
        self.declare_parameter('bridge_lateral_right_error_right_norm', -0.10)
        self.declare_parameter('bridge_lateral_max_abs_error_m', 0.45)
        self.declare_parameter('bridge_lateral_finish_after_hold', True)
        self.declare_parameter('bridge_lateral_lock_after_hold', True)
        self.declare_parameter('bridge_lateral_yaw_realign_enabled', False)
        self.declare_parameter('bridge_lateral_yaw_reference_source', 'auto')
        self.declare_parameter('bridge_lateral_yaw_deadband_deg', 2.0)
        self.declare_parameter('bridge_lateral_yaw_hold_sec', 0.20)
        self.declare_parameter('bridge_lateral_yaw_use_step_override', True)
        self.declare_parameter('bridge_lateral_yaw_turn_mode', 6)
        self.declare_parameter('bridge_lateral_yaw_cw_left_norm', 0.08)
        self.declare_parameter('bridge_lateral_yaw_cw_right_norm', -0.05)
        self.declare_parameter('bridge_lateral_yaw_ccw_left_norm', -0.05)
        self.declare_parameter('bridge_lateral_yaw_ccw_right_norm', 0.08)
        self.declare_parameter('bridge_lateral_control_strategy', 'arc')
        self.declare_parameter('bridge_lateral_twt_yaw_deg', 8.0)
        self.declare_parameter('bridge_lateral_twt_yaw_sign', -1.0)
        self.declare_parameter('bridge_lateral_twt_distance_scale', 1.0)
        self.declare_parameter('bridge_lateral_twt_min_distance_m', 0.03)
        self.declare_parameter('bridge_lateral_twt_max_distance_m', 0.15)
        self.declare_parameter('bridge_lateral_twt_distance_deadband_m', 0.01)
        self.declare_parameter('bridge_lateral_twt_live_stop_enabled', False)
        self.declare_parameter('bridge_lateral_twt_max_iterations', 1)
        self.declare_parameter('bridge_lateral_twt_forward_mode', 0)
        self.declare_parameter('bridge_lateral_twt_forward_left_norm', 0.12)
        self.declare_parameter('bridge_lateral_twt_forward_right_norm', 0.12)
        self.declare_parameter('max_abs_error_deg', 45.0)
        self.declare_parameter('min_distance_m', 0.15)
        self.declare_parameter('max_distance_m', 3.00)
        self.declare_parameter('stale_timeout_sec', 0.30)
        self.declare_parameter('pose_stale_timeout_sec', 0.80)
        self.declare_parameter('lock_reference_on_tag_loss', False)
        self.declare_parameter('locked_reference_along_sign', 1.0)
        self.declare_parameter('lock_reference_update_while_visible', True)
        self.declare_parameter('lateral_shift_sequence_enabled', False)
        self.declare_parameter('lateral_shift_pre_align_enabled', False)
        self.declare_parameter('lateral_shift_pre_align_mode', 'tag_center')
        self.declare_parameter('lateral_shift_pre_align_use_visual_feedback', False)
        self.declare_parameter('lateral_shift_continue_after_tag_loss', True)
        self.declare_parameter('lateral_shift_pre_align_timeout_sec', 2.5)
        self.declare_parameter('lateral_shift_pre_align_max_visual_error_deg', 6.0)
        self.declare_parameter('lateral_shift_pre_align_hold_sec', 0.20)
        self.declare_parameter('lateral_shift_pre_align_tolerance_deg', 6.0)
        self.declare_parameter('lateral_shift_yaw_deg', 25.0)
        self.declare_parameter('lateral_shift_yaw_sign', -1.0)
        self.declare_parameter('lateral_shift_deadband_m', 0.03)
        self.declare_parameter('lateral_shift_distance_scale', 1.0)
        self.declare_parameter('lateral_shift_max_distance_m', 0.80)
        self.declare_parameter('lateral_shift_use_euclidean_progress', True)
        self.declare_parameter('lateral_shift_final_yaw_gate_deg', 8.0)
        self.declare_parameter('lateral_shift_duck_entry_yaw_gate_deg', 2.0)
        self.declare_parameter('lateral_shift_duck_entry_hold_sec', 0.30)
        self.declare_parameter('limit_bar_duck_sequence_enabled', False)
        self.declare_parameter('duck_prepare_mode', 2)
        self.declare_parameter('duck_prepare_left_norm', 0.0)
        self.declare_parameter('duck_prepare_right_norm', 0.0)
        self.declare_parameter('duck_prepare_delay_sec', 2.0)
        self.declare_parameter('duck_walk_mode', 2)
        self.declare_parameter('duck_walk_left_norm', 0.90)
        self.declare_parameter('duck_walk_right_norm', 0.90)
        self.declare_parameter('duck_walk_yaw_correction_enabled', False)
        self.declare_parameter('duck_walk_yaw_gain_norm_per_rad', 0.45)
        self.declare_parameter('duck_walk_yaw_deadband_deg', 3.0)
        self.declare_parameter('duck_walk_yaw_max_delta_norm', 0.18)
        self.declare_parameter('duck_walk_distance_m', 0.85)
        self.declare_parameter('duck_walk_timeout_sec', 10.0)
        self.declare_parameter('duck_resume_mode', 0)
        self.declare_parameter('duck_resume_left_norm', 0.0)
        self.declare_parameter('duck_resume_right_norm', 0.0)
        self.declare_parameter('duck_resume_delay_sec', 1.0)
        self.declare_parameter('aligned_hold_sec', 0.30)
        self.declare_parameter('finish_after_aligned_hold', False)
        self.declare_parameter('publish_zero_when_idle', True)
        self.declare_parameter('publish_mode_when_control_enabled', True)
        self.declare_parameter('mode_publish_interval_sec', 1.0)
        self.declare_parameter('step_override_forward_mode', 0)
        self.declare_parameter('step_override_forward_left_norm', 0.70)
        self.declare_parameter('step_override_forward_right_norm', 0.70)
        self.declare_parameter('step_override_forward_scale_enabled', False)
        self.declare_parameter('step_override_forward_min_scale', 0.45)
        self.declare_parameter('step_override_forward_yaw_correction_enabled', False)
        self.declare_parameter('step_override_forward_yaw_gain_norm_per_radps', 0.70)
        self.declare_parameter('step_override_forward_yaw_max_delta_norm', 0.16)
        self.declare_parameter('near_approach_use_cmd_vel', True)
        self.declare_parameter('near_approach_distance_error_m', 0.18)
        self.declare_parameter('step_override_turn_mode', 6)
        self.declare_parameter('step_override_turn_cw_left_norm', 0.60)
        self.declare_parameter('step_override_turn_cw_right_norm', -0.30)
        self.declare_parameter('step_override_turn_ccw_left_norm', -0.35)
        self.declare_parameter('step_override_turn_ccw_right_norm', 0.60)
        self.declare_parameter('step_override_turn_scale_enabled', False)
        self.declare_parameter('step_override_turn_min_scale', 0.45)
        self.declare_parameter('step_override_duck_align_turn_enabled', False)
        self.declare_parameter('step_override_duck_align_cw_left_norm', 0.08)
        self.declare_parameter('step_override_duck_align_cw_right_norm', -0.045)
        self.declare_parameter('step_override_duck_align_ccw_left_norm', -0.045)
        self.declare_parameter('step_override_duck_align_ccw_right_norm', 0.08)
        self.declare_parameter('step_override_clear_mode', -1)
        self.declare_parameter('log_interval_sec', 0.25)
        self.declare_parameter('warn_interval_sec', 1.0)

        self.tf_topic = str(self.get_parameter('tf_topic').value)
        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.mode_topic = str(self.get_parameter('mode_topic').value)
        self.step_override_topic = str(self.get_parameter('step_override_topic').value)
        self.feedback_topic = str(self.get_parameter('feedback_topic').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.debug_topic = str(self.get_parameter('debug_topic').value)

        self.target_frame_id = str(self.get_parameter('target_frame_id').value)
        self.target_child_frame_id = str(self.get_parameter('target_child_frame_id').value)
        self.target_tag_id = int(self.get_parameter('target_tag_id').value)
        self.target_offset_x_m = float(self.get_parameter('target_offset_x_m').value)
        self.target_tag_x_m = float(self.get_parameter('target_tag_x_m').value)
        self.robot_center_offset_x_m = float(
            self.get_parameter('robot_center_offset_x_m').value
        )
        self.alignment_mode = str(self.get_parameter('alignment_mode').value).strip().lower()

        self.enable_control = bool(self.get_parameter('enable_control').value)
        self.control_output_mode = (
            str(self.get_parameter('control_output_mode').value).strip().lower()
        )
        self.control_mode = int(self.get_parameter('control_mode').value)
        self.control_rate_hz = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.control_sign = self._sign(float(self.get_parameter('control_sign').value))
        self.center_control_sign = self._sign(
            float(self.get_parameter('center_control_sign').value)
        )
        self.pose_control_sign = self._sign(float(self.get_parameter('pose_control_sign').value))
        self.target_center_angle_rad = math.radians(
            float(self.get_parameter('target_center_angle_deg').value)
        )
        self.target_pose_roll_rad = math.radians(
            float(self.get_parameter('target_pose_roll_deg').value)
        )
        self.target_pose_pitch_rad = math.radians(
            float(self.get_parameter('target_pose_pitch_deg').value)
        )
        self.target_pose_yaw_rad = math.radians(
            float(self.get_parameter('target_pose_yaw_deg').value)
        )
        self.yaw_deadband_rad = max(
            0.0,
            math.radians(float(self.get_parameter('yaw_deadband_deg').value)),
        )
        self.yaw_kp = max(0.0, float(self.get_parameter('yaw_kp').value))
        self.min_abs_angular_z = max(0.0, float(self.get_parameter('min_abs_angular_z').value))
        self.max_abs_angular_z = max(
            0.0,
            float(self.get_parameter('max_abs_angular_z').value),
        )
        self.enable_forward_approach = bool(
            self.get_parameter('enable_forward_approach').value
        )
        self.target_distance_m = max(0.0, float(self.get_parameter('target_distance_m').value))
        self.distance_deadband_m = max(
            0.0,
            float(self.get_parameter('distance_deadband_m').value),
        )
        self.forward_kp = max(0.0, float(self.get_parameter('forward_kp').value))
        self.min_forward_speed_mps = max(
            0.0,
            float(self.get_parameter('min_forward_speed_mps').value),
        )
        self.max_forward_speed_mps = max(
            0.0,
            float(self.get_parameter('max_forward_speed_mps').value),
        )
        self.forward_max_yaw_error_rad = max(
            self.yaw_deadband_rad,
            math.radians(float(self.get_parameter('forward_max_yaw_error_deg').value)),
        )
        self.approach_requires_aligned = bool(
            self.get_parameter('approach_requires_aligned').value
        )
        self.approach_distance_axis = (
            str(self.get_parameter('approach_distance_axis').value).strip().lower()
        )
        self.bridge_lateral_calibration_enabled = bool(
            self.get_parameter('bridge_lateral_calibration_enabled').value
        )
        self.bridge_lateral_deadband_m = max(
            0.0,
            float(self.get_parameter('bridge_lateral_deadband_m').value),
        )
        self.bridge_lateral_hold_sec = max(
            0.0,
            float(self.get_parameter('bridge_lateral_hold_sec').value),
        )
        self.bridge_lateral_forward_speed_mps = max(
            0.0,
            float(self.get_parameter('bridge_lateral_forward_speed_mps').value),
        )
        self.bridge_lateral_kp_radps_per_m = max(
            0.0,
            float(self.get_parameter('bridge_lateral_kp_radps_per_m').value),
        )
        self.bridge_lateral_min_abs_angular_z = max(
            0.0,
            float(self.get_parameter('bridge_lateral_min_abs_angular_z').value),
        )
        self.bridge_lateral_max_abs_angular_z = max(
            0.0,
            float(self.get_parameter('bridge_lateral_max_abs_angular_z').value),
        )
        self.bridge_lateral_control_sign = self._sign(
            float(self.get_parameter('bridge_lateral_control_sign').value)
        )
        self.bridge_lateral_use_step_override = bool(
            self.get_parameter('bridge_lateral_use_step_override').value
        )
        self.bridge_lateral_step_override_mode = int(
            self.get_parameter('bridge_lateral_step_override_mode').value
        )
        self.bridge_lateral_left_error_left_norm = self._clamp_norm(
            float(self.get_parameter('bridge_lateral_left_error_left_norm').value)
        )
        self.bridge_lateral_left_error_right_norm = self._clamp_norm(
            float(self.get_parameter('bridge_lateral_left_error_right_norm').value)
        )
        self.bridge_lateral_right_error_left_norm = self._clamp_norm(
            float(self.get_parameter('bridge_lateral_right_error_left_norm').value)
        )
        self.bridge_lateral_right_error_right_norm = self._clamp_norm(
            float(self.get_parameter('bridge_lateral_right_error_right_norm').value)
        )
        self.bridge_lateral_max_abs_error_m = max(
            0.0,
            float(self.get_parameter('bridge_lateral_max_abs_error_m').value),
        )
        self.bridge_lateral_finish_after_hold = bool(
            self.get_parameter('bridge_lateral_finish_after_hold').value
        )
        self.bridge_lateral_lock_after_hold = bool(
            self.get_parameter('bridge_lateral_lock_after_hold').value
        )
        self.bridge_lateral_yaw_realign_enabled = bool(
            self.get_parameter('bridge_lateral_yaw_realign_enabled').value
        )
        self.bridge_lateral_yaw_reference_source = (
            str(self.get_parameter('bridge_lateral_yaw_reference_source').value)
            .strip()
            .lower()
        )
        if self.bridge_lateral_yaw_reference_source not in (
            'auto',
            'odom',
            'tag',
            'tag_yaw',
            'tag_pose_yaw',
        ):
            self.bridge_lateral_yaw_reference_source = 'auto'
        self.bridge_lateral_yaw_deadband_rad = max(
            0.0,
            math.radians(float(self.get_parameter('bridge_lateral_yaw_deadband_deg').value)),
        )
        self.bridge_lateral_yaw_hold_sec = max(
            0.0,
            float(self.get_parameter('bridge_lateral_yaw_hold_sec').value),
        )
        self.bridge_lateral_yaw_use_step_override = bool(
            self.get_parameter('bridge_lateral_yaw_use_step_override').value
        )
        self.bridge_lateral_yaw_turn_mode = int(
            self.get_parameter('bridge_lateral_yaw_turn_mode').value
        )
        self.bridge_lateral_yaw_cw_left_norm = self._clamp_norm(
            float(self.get_parameter('bridge_lateral_yaw_cw_left_norm').value)
        )
        self.bridge_lateral_yaw_cw_right_norm = self._clamp_norm(
            float(self.get_parameter('bridge_lateral_yaw_cw_right_norm').value)
        )
        self.bridge_lateral_yaw_ccw_left_norm = self._clamp_norm(
            float(self.get_parameter('bridge_lateral_yaw_ccw_left_norm').value)
        )
        self.bridge_lateral_yaw_ccw_right_norm = self._clamp_norm(
            float(self.get_parameter('bridge_lateral_yaw_ccw_right_norm').value)
        )
        self.bridge_lateral_control_strategy = (
            str(self.get_parameter('bridge_lateral_control_strategy').value)
            .strip()
            .lower()
        )
        if self.bridge_lateral_control_strategy not in ('arc', 'turn_walk_turn'):
            self.bridge_lateral_control_strategy = 'arc'
        self.bridge_lateral_twt_yaw_rad = max(
            math.radians(1.0),
            min(
                math.radians(30.0),
                abs(math.radians(float(self.get_parameter('bridge_lateral_twt_yaw_deg').value))),
            ),
        )
        self.bridge_lateral_twt_yaw_sign = self._sign(
            float(self.get_parameter('bridge_lateral_twt_yaw_sign').value)
        )
        self.bridge_lateral_twt_distance_scale = max(
            0.0,
            min(1.5, float(self.get_parameter('bridge_lateral_twt_distance_scale').value)),
        )
        self.bridge_lateral_twt_min_distance_m = max(
            0.0,
            float(self.get_parameter('bridge_lateral_twt_min_distance_m').value),
        )
        self.bridge_lateral_twt_max_distance_m = max(
            self.bridge_lateral_twt_min_distance_m,
            float(self.get_parameter('bridge_lateral_twt_max_distance_m').value),
        )
        self.bridge_lateral_twt_distance_deadband_m = max(
            0.0,
            float(self.get_parameter('bridge_lateral_twt_distance_deadband_m').value),
        )
        self.bridge_lateral_twt_live_stop_enabled = bool(
            self.get_parameter('bridge_lateral_twt_live_stop_enabled').value
        )
        self.bridge_lateral_twt_max_iterations = max(
            0,
            int(self.get_parameter('bridge_lateral_twt_max_iterations').value),
        )
        self.bridge_lateral_twt_forward_mode = int(
            self.get_parameter('bridge_lateral_twt_forward_mode').value
        )
        self.bridge_lateral_twt_forward_left_norm = self._clamp_norm(
            float(self.get_parameter('bridge_lateral_twt_forward_left_norm').value)
        )
        self.bridge_lateral_twt_forward_right_norm = self._clamp_norm(
            float(self.get_parameter('bridge_lateral_twt_forward_right_norm').value)
        )
        self.max_abs_error_rad = max(
            0.0,
            math.radians(float(self.get_parameter('max_abs_error_deg').value)),
        )
        self.min_distance_m = max(0.0, float(self.get_parameter('min_distance_m').value))
        self.max_distance_m = max(
            self.min_distance_m,
            float(self.get_parameter('max_distance_m').value),
        )
        self.stale_timeout_sec = max(0.0, float(self.get_parameter('stale_timeout_sec').value))
        self.pose_stale_timeout_sec = max(
            0.0,
            float(self.get_parameter('pose_stale_timeout_sec').value),
        )
        self.lock_reference_on_tag_loss = bool(
            self.get_parameter('lock_reference_on_tag_loss').value
        )
        self.locked_reference_along_sign = self._sign(
            float(self.get_parameter('locked_reference_along_sign').value)
        )
        self.lock_reference_update_while_visible = bool(
            self.get_parameter('lock_reference_update_while_visible').value
        )
        self.lateral_shift_sequence_enabled = bool(
            self.get_parameter('lateral_shift_sequence_enabled').value
        )
        self.lateral_shift_pre_align_enabled = bool(
            self.get_parameter('lateral_shift_pre_align_enabled').value
        )
        self.lateral_shift_pre_align_mode = (
            str(self.get_parameter('lateral_shift_pre_align_mode').value)
            .strip()
            .lower()
        )
        self.lateral_shift_pre_align_use_visual_feedback = bool(
            self.get_parameter('lateral_shift_pre_align_use_visual_feedback').value
        )
        self.lateral_shift_continue_after_tag_loss = bool(
            self.get_parameter('lateral_shift_continue_after_tag_loss').value
        )
        self.lateral_shift_pre_align_timeout_sec = max(
            0.0,
            float(self.get_parameter('lateral_shift_pre_align_timeout_sec').value),
        )
        self.lateral_shift_pre_align_max_visual_error_rad = max(
            0.0,
            math.radians(
                float(self.get_parameter('lateral_shift_pre_align_max_visual_error_deg').value)
            ),
        )
        self.lateral_shift_pre_align_hold_sec = max(
            0.0,
            float(self.get_parameter('lateral_shift_pre_align_hold_sec').value),
        )
        self.lateral_shift_pre_align_tolerance_rad = max(
            self.yaw_deadband_rad,
            math.radians(float(self.get_parameter('lateral_shift_pre_align_tolerance_deg').value)),
        )
        self.lateral_shift_yaw_rad = max(
            math.radians(3.0),
            min(
                math.radians(60.0),
                abs(math.radians(float(self.get_parameter('lateral_shift_yaw_deg').value))),
            ),
        )
        self.lateral_shift_yaw_sign = self._sign(
            float(self.get_parameter('lateral_shift_yaw_sign').value)
        )
        self.lateral_shift_deadband_m = max(
            0.0,
            float(self.get_parameter('lateral_shift_deadband_m').value),
        )
        self.lateral_shift_distance_scale = max(
            0.0,
            min(1.5, float(self.get_parameter('lateral_shift_distance_scale').value)),
        )
        self.lateral_shift_max_distance_m = max(
            0.0,
            float(self.get_parameter('lateral_shift_max_distance_m').value),
        )
        self.lateral_shift_use_euclidean_progress = bool(
            self.get_parameter('lateral_shift_use_euclidean_progress').value
        )
        self.lateral_shift_final_yaw_gate_rad = max(
            self.yaw_deadband_rad,
            math.radians(float(self.get_parameter('lateral_shift_final_yaw_gate_deg').value)),
        )
        self.lateral_shift_duck_entry_yaw_gate_rad = max(
            0.0,
            math.radians(
                float(self.get_parameter('lateral_shift_duck_entry_yaw_gate_deg').value)
            ),
        )
        self.lateral_shift_duck_entry_hold_sec = max(
            0.0,
            float(self.get_parameter('lateral_shift_duck_entry_hold_sec').value),
        )
        self.limit_bar_duck_sequence_enabled = bool(
            self.get_parameter('limit_bar_duck_sequence_enabled').value
        )
        self.duck_prepare_mode = int(self.get_parameter('duck_prepare_mode').value)
        self.duck_prepare_left_norm = self._clamp_norm(
            float(self.get_parameter('duck_prepare_left_norm').value)
        )
        self.duck_prepare_right_norm = self._clamp_norm(
            float(self.get_parameter('duck_prepare_right_norm').value)
        )
        self.duck_prepare_delay_sec = max(
            0.0,
            float(self.get_parameter('duck_prepare_delay_sec').value),
        )
        self.duck_walk_mode = int(self.get_parameter('duck_walk_mode').value)
        self.duck_walk_left_norm = self._clamp_norm(
            float(self.get_parameter('duck_walk_left_norm').value)
        )
        self.duck_walk_right_norm = self._clamp_norm(
            float(self.get_parameter('duck_walk_right_norm').value)
        )
        self.duck_walk_yaw_correction_enabled = bool(
            self.get_parameter('duck_walk_yaw_correction_enabled').value
        )
        self.duck_walk_yaw_gain_norm_per_rad = max(
            0.0,
            float(self.get_parameter('duck_walk_yaw_gain_norm_per_rad').value),
        )
        self.duck_walk_yaw_deadband_rad = max(
            0.0,
            math.radians(float(self.get_parameter('duck_walk_yaw_deadband_deg').value)),
        )
        self.duck_walk_yaw_max_delta_norm = max(
            0.0,
            min(1.0, float(self.get_parameter('duck_walk_yaw_max_delta_norm').value)),
        )
        self.duck_walk_distance_m = max(
            0.0,
            float(self.get_parameter('duck_walk_distance_m').value),
        )
        self.duck_walk_timeout_sec = max(
            0.0,
            float(self.get_parameter('duck_walk_timeout_sec').value),
        )
        self.duck_resume_mode = int(self.get_parameter('duck_resume_mode').value)
        self.duck_resume_left_norm = self._clamp_norm(
            float(self.get_parameter('duck_resume_left_norm').value)
        )
        self.duck_resume_right_norm = self._clamp_norm(
            float(self.get_parameter('duck_resume_right_norm').value)
        )
        self.duck_resume_delay_sec = max(
            0.0,
            float(self.get_parameter('duck_resume_delay_sec').value),
        )
        self.aligned_hold_sec = max(0.0, float(self.get_parameter('aligned_hold_sec').value))
        self.finish_after_aligned_hold = bool(
            self.get_parameter('finish_after_aligned_hold').value
        )
        self.publish_zero_when_idle = bool(self.get_parameter('publish_zero_when_idle').value)
        self.publish_mode_when_control_enabled = bool(
            self.get_parameter('publish_mode_when_control_enabled').value
        )
        self.mode_publish_interval_sec = max(
            0.0,
            float(self.get_parameter('mode_publish_interval_sec').value),
        )
        self.step_override_forward_mode = int(
            self.get_parameter('step_override_forward_mode').value
        )
        self.step_override_forward_left_norm = self._clamp_norm(
            float(self.get_parameter('step_override_forward_left_norm').value)
        )
        self.step_override_forward_right_norm = self._clamp_norm(
            float(self.get_parameter('step_override_forward_right_norm').value)
        )
        self.step_override_forward_scale_enabled = bool(
            self.get_parameter('step_override_forward_scale_enabled').value
        )
        self.step_override_forward_min_scale = max(
            0.0,
            min(1.0, float(self.get_parameter('step_override_forward_min_scale').value)),
        )
        self.step_override_forward_yaw_correction_enabled = bool(
            self.get_parameter('step_override_forward_yaw_correction_enabled').value
        )
        self.step_override_forward_yaw_gain_norm_per_radps = max(
            0.0,
            float(self.get_parameter('step_override_forward_yaw_gain_norm_per_radps').value),
        )
        self.step_override_forward_yaw_max_delta_norm = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter('step_override_forward_yaw_max_delta_norm').value),
            ),
        )
        self.near_approach_use_cmd_vel = bool(
            self.get_parameter('near_approach_use_cmd_vel').value
        )
        self.near_approach_distance_error_m = max(
            0.0,
            float(self.get_parameter('near_approach_distance_error_m').value),
        )
        self.step_override_turn_mode = int(self.get_parameter('step_override_turn_mode').value)
        self.step_override_turn_cw_left_norm = self._clamp_norm(
            float(self.get_parameter('step_override_turn_cw_left_norm').value)
        )
        self.step_override_turn_cw_right_norm = self._clamp_norm(
            float(self.get_parameter('step_override_turn_cw_right_norm').value)
        )
        self.step_override_turn_ccw_left_norm = self._clamp_norm(
            float(self.get_parameter('step_override_turn_ccw_left_norm').value)
        )
        self.step_override_turn_ccw_right_norm = self._clamp_norm(
            float(self.get_parameter('step_override_turn_ccw_right_norm').value)
        )
        self.step_override_turn_scale_enabled = bool(
            self.get_parameter('step_override_turn_scale_enabled').value
        )
        self.step_override_turn_min_scale = max(
            0.0,
            min(1.0, float(self.get_parameter('step_override_turn_min_scale').value)),
        )
        self.step_override_duck_align_turn_enabled = bool(
            self.get_parameter('step_override_duck_align_turn_enabled').value
        )
        self.step_override_duck_align_cw_left_norm = self._clamp_norm(
            float(self.get_parameter('step_override_duck_align_cw_left_norm').value)
        )
        self.step_override_duck_align_cw_right_norm = self._clamp_norm(
            float(self.get_parameter('step_override_duck_align_cw_right_norm').value)
        )
        self.step_override_duck_align_ccw_left_norm = self._clamp_norm(
            float(self.get_parameter('step_override_duck_align_ccw_left_norm').value)
        )
        self.step_override_duck_align_ccw_right_norm = self._clamp_norm(
            float(self.get_parameter('step_override_duck_align_ccw_right_norm').value)
        )
        self.step_override_clear_mode = int(self.get_parameter('step_override_clear_mode').value)
        self.log_interval_sec = max(0.05, float(self.get_parameter('log_interval_sec').value))
        self.warn_interval_sec = max(0.20, float(self.get_parameter('warn_interval_sec').value))

        if self.alignment_mode not in (
            'center_x',
            'center_x_offset',
            'offset_center_x',
            'pose_roll',
            'pose_pitch',
            'pose_yaw',
        ):
            raise RuntimeError(
                'alignment_mode must be one of: center_x, center_x_offset, '
                'offset_center_x, pose_roll, pose_pitch, pose_yaw.'
            )
        if self.approach_distance_axis not in ('z', 'euclidean'):
            raise RuntimeError('approach_distance_axis must be one of: z, euclidean.')
        if self.control_output_mode not in ('cmd_vel', 'step_override', 'hybrid'):
            raise RuntimeError(
                'control_output_mode must be one of: cmd_vel, step_override, hybrid.'
            )
        if self.lateral_shift_pre_align_mode not in (
            'tag_center',
            'centerline',
            'pose_roll',
            'pose_pitch',
            'pose_yaw',
        ):
            raise RuntimeError(
                'lateral_shift_pre_align_mode must be one of: tag_center, centerline, '
                'pose_roll, pose_pitch, pose_yaw.'
            )

        self.latest_tag: Optional[TagPose] = None
        self.latest_detection = DetectionInfo()
        self.pose_x: Optional[float] = None
        self.pose_y: Optional[float] = None
        self.pose_yaw: Optional[float] = None
        self.pose_stamp_sec = 0.0
        self.pose_received_sec = 0.0
        self.locked_reference_active = False
        self.locked_reference_target_yaw: Optional[float] = None
        self.locked_reference_start_x: Optional[float] = None
        self.locked_reference_start_y: Optional[float] = None
        self.locked_reference_distance_m = 0.0
        self.locked_reference_source = ''
        self.locked_reference_stamp_sec = 0.0
        self.lateral_shift_pre_align_done = False
        self.lateral_shift_pre_align_since_sec: Optional[float] = None
        self.lateral_shift_pre_align_start_sec: Optional[float] = None
        self.lateral_shift_pre_align_target_yaw: Optional[float] = None
        self.lateral_shift_pre_align_source_metric = ''
        self.lateral_shift_pre_align_source_target_rad = 0.0
        self.lateral_shift_pre_align_source_angle_rad = 0.0
        self.lateral_shift_pre_align_source_control_rad = 0.0
        self.lateral_shift_pre_align_source_x_m = 0.0
        self.lateral_shift_pre_align_source_z_m = 0.0
        self.lateral_shift_pre_align_best_metric = ''
        self.lateral_shift_pre_align_best_error_rad: Optional[float] = None
        self.lateral_shift_pre_align_best_measured_rad = 0.0
        self.lateral_shift_pre_align_best_target_rad = 0.0
        self.lateral_shift_pre_align_best_pose_yaw: Optional[float] = None
        self.lateral_shift_pre_align_best_tag: Optional[TagPose] = None
        self.lateral_shift_pre_align_best_sec = 0.0
        self.lateral_shift_phase = ''
        self.lateral_shift_base_yaw: Optional[float] = None
        self.lateral_shift_side_yaw: Optional[float] = None
        self.lateral_shift_center_x_m = 0.0
        self.lateral_shift_center_z_m = 0.0
        self.lateral_shift_distance_m = 0.0
        self.lateral_shift_forward_component_m = 0.0
        self.lateral_shift_final_forward_m = 0.0
        self.lateral_shift_raw_distance_m = 0.0
        self.lateral_shift_requested_distance_m = 0.0
        self.lateral_shift_duck_align_since_sec: Optional[float] = None
        self.lateral_segment_start_x: Optional[float] = None
        self.lateral_segment_start_y: Optional[float] = None
        self.lateral_shift_locked_tag: Optional[TagPose] = None
        self.duck_phase = ''
        self.duck_phase_enter_sec = 0.0
        self.duck_phase_deadline_sec: Optional[float] = None
        self.duck_walk_start_x: Optional[float] = None
        self.duck_walk_start_y: Optional[float] = None
        self.duck_reference_yaw: Optional[float] = None
        self.last_log_time = 0.0
        self.last_warn_time = 0.0
        self.last_mode_publish_time = -math.inf
        self.last_mode_value: Optional[int] = None
        self.step_override_active = False
        self.aligned_since_sec: Optional[float] = None
        self.bridge_lateral_aligned_since_sec: Optional[float] = None
        self.bridge_lateral_yaw_aligned_since_sec: Optional[float] = None
        self.bridge_lateral_reference_yaw: Optional[float] = None
        self.bridge_lateral_reference_yaw_source = ''
        self.bridge_lateral_position_locked = False
        self.bridge_lateral_twt_phase = ''
        self.bridge_lateral_twt_base_yaw: Optional[float] = None
        self.bridge_lateral_twt_side_yaw: Optional[float] = None
        self.bridge_lateral_twt_start_x: Optional[float] = None
        self.bridge_lateral_twt_start_y: Optional[float] = None
        self.bridge_lateral_twt_distance_m = 0.0
        self.bridge_lateral_twt_error_x_m = 0.0
        self.bridge_lateral_twt_live_error_x_m = math.nan
        self.bridge_lateral_twt_stop_reason = ''
        self.bridge_lateral_twt_iteration = 0
        self.finished = False
        self.last_state = ''

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.mode_pub = self.create_publisher(Int32, self.mode_topic, 10)
        self.step_override_pub = self.create_publisher(
            Float32MultiArray,
            self.step_override_topic,
            10,
        )
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.debug_pub = self.create_publisher(Float32MultiArray, self.debug_topic, 10)

        self.tf_sub = self.create_subscription(
            TFMessage,
            self.tf_topic,
            self._tf_callback,
            qos_profile_sensor_data,
        )
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            self.detections_topic,
            self._detections_callback,
            qos_profile_sensor_data,
        )
        self.pose_sub = self.create_subscription(
            Odometry,
            self.pose_topic,
            self._pose_callback,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._timer_callback)

        self._publish_feedback(
            'apriltag_align_test started: '
            f'tf={self.tf_topic}, detections={self.detections_topic}, '
            f'target_tf={self.target_frame_id}->{self.target_child_frame_id}, '
            f'target_tag_id={self.target_tag_id}, mode={self.alignment_mode}, '
            f'offset_x={self.target_offset_x_m:+.3f}m, '
            f'target_tag_x={self._target_tag_x_text()}, '
            f'robot_center_offset_x={self.robot_center_offset_x_m:+.3f}m, '
            f'targets=center={math.degrees(self.target_center_angle_rad):+.1f}deg, '
            f'pose_rpy=({math.degrees(self.target_pose_roll_rad):+.1f},'
            f'{math.degrees(self.target_pose_pitch_rad):+.1f},'
            f'{math.degrees(self.target_pose_yaw_rad):+.1f})deg, '
            f'deadband={math.degrees(self.yaw_deadband_rad):.1f}deg, '
            f'enable_control={self.enable_control}, output={self.control_output_mode}, '
            f'cmd_vel={self.cmd_vel_topic}, pose={self.pose_topic}, '
            f'step_override={self.step_override_topic}, '
            f'control_mode={self.control_mode}, max_wz={self.max_abs_angular_z:.2f}rad/s, '
            f'forward_approach={self.enable_forward_approach}, '
            f'target_distance={self.target_distance_m:.2f}m, '
            f'bridge_lateral={self.bridge_lateral_calibration_enabled}, '
            f'bridge_target_tag_x={self._target_tag_x_text()}, '
            f'bridge_deadband={self.bridge_lateral_deadband_m:.3f}m, '
            f'bridge_hold={self.bridge_lateral_hold_sec:.2f}s, '
            f'bridge_vx={self.bridge_lateral_forward_speed_mps:.3f}m/s, '
            f'bridge_wz_gain={self.bridge_lateral_kp_radps_per_m:.2f}, '
            f'bridge_wz_limits=[{self.bridge_lateral_min_abs_angular_z:.2f},'
            f'{self.bridge_lateral_max_abs_angular_z:.2f}]rad/s, '
            f'bridge_sign={self.bridge_lateral_control_sign:+.0f}, '
            f'bridge_step_override={self.bridge_lateral_use_step_override}, '
            f'bridge_left_error=[{self.bridge_lateral_step_override_mode},'
            f'{self.bridge_lateral_left_error_left_norm:+.2f},'
            f'{self.bridge_lateral_left_error_right_norm:+.2f}], '
            f'bridge_right_error=[{self.bridge_lateral_step_override_mode},'
            f'{self.bridge_lateral_right_error_left_norm:+.2f},'
            f'{self.bridge_lateral_right_error_right_norm:+.2f}], '
            f'bridge_yaw_realign={self.bridge_lateral_yaw_realign_enabled}, '
            f'bridge_lock_after_hold={self.bridge_lateral_lock_after_hold}, '
            f'bridge_yaw_source={self.bridge_lateral_yaw_reference_source}, '
            f'bridge_yaw_deadband={math.degrees(self.bridge_lateral_yaw_deadband_rad):.1f}deg, '
            f'bridge_yaw_hold={self.bridge_lateral_yaw_hold_sec:.2f}s, '
            f'bridge_yaw_turn_cw=[{self.bridge_lateral_yaw_turn_mode},'
            f'{self.bridge_lateral_yaw_cw_left_norm:+.3f},'
            f'{self.bridge_lateral_yaw_cw_right_norm:+.3f}], '
            f'bridge_yaw_turn_ccw=[{self.bridge_lateral_yaw_turn_mode},'
            f'{self.bridge_lateral_yaw_ccw_left_norm:+.3f},'
            f'{self.bridge_lateral_yaw_ccw_right_norm:+.3f}], '
            f'bridge_strategy={self.bridge_lateral_control_strategy}, '
            f'bridge_twt_yaw={math.degrees(self.bridge_lateral_twt_yaw_rad):.1f}deg, '
            f'bridge_twt_sign={self.bridge_lateral_twt_yaw_sign:+.0f}, '
            f'bridge_twt_dist=[{self.bridge_lateral_twt_min_distance_m:.3f},'
            f'{self.bridge_lateral_twt_max_distance_m:.3f}]m, '
            f'bridge_twt_fixed_walk=True, '
            f'bridge_twt_max_iter={self.bridge_lateral_twt_max_iterations}, '
            f'bridge_twt_step=[{self.bridge_lateral_twt_forward_mode},'
            f'{self.bridge_lateral_twt_forward_left_norm:+.3f},'
            f'{self.bridge_lateral_twt_forward_right_norm:+.3f}], '
            f'lock_on_loss={self.lock_reference_on_tag_loss}, '
            f'lock_update_visible={self.lock_reference_update_while_visible}, '
            f'lateral_shift={self.lateral_shift_sequence_enabled}, '
            f'lateral_pre_align={self.lateral_shift_pre_align_enabled}, '
            f'lateral_pre_align_mode={self.lateral_shift_pre_align_mode}, '
            f'lateral_pre_align_visual_feedback='
            f'{self.lateral_shift_pre_align_use_visual_feedback}, '
            f'lateral_continue_after_loss={self.lateral_shift_continue_after_tag_loss}, '
            f'lateral_pre_align_timeout={self.lateral_shift_pre_align_timeout_sec:.2f}s, '
            f'lateral_pre_align_visual_gate='
            f'{math.degrees(self.lateral_shift_pre_align_max_visual_error_rad):.1f}deg, '
            f'lateral_pre_align_tol={math.degrees(self.lateral_shift_pre_align_tolerance_rad):.1f}deg, '
            f'lateral_yaw={math.degrees(self.lateral_shift_yaw_rad):.1f}deg, '
            f'lateral_sign={self.lateral_shift_yaw_sign:+.0f}, '
            f'lateral_scale={self.lateral_shift_distance_scale:.2f}, '
            f'lateral_progress_euclidean={self.lateral_shift_use_euclidean_progress}, '
            f'lateral_final_gate={math.degrees(self.lateral_shift_final_yaw_gate_rad):.1f}deg, '
            f'lateral_duck_gate={math.degrees(self.lateral_shift_duck_entry_yaw_gate_rad):.1f}deg, '
            f'lateral_duck_hold={self.lateral_shift_duck_entry_hold_sec:.2f}s, '
            f'duck_sequence={self.limit_bar_duck_sequence_enabled}, '
            f'duck_yaw_corr={self.duck_walk_yaw_correction_enabled}, '
            f'duck_yaw_gain={self.duck_walk_yaw_gain_norm_per_rad:.2f}, '
            f'duck_yaw_deadband={math.degrees(self.duck_walk_yaw_deadband_rad):.1f}deg, '
            f'duck_yaw_max_delta={self.duck_walk_yaw_max_delta_norm:.2f}, '
            f'max_vx={self.max_forward_speed_mps:.2f}m/s, '
            f'forward_step=[{self.step_override_forward_mode},'
            f'{self.step_override_forward_left_norm:.2f},'
            f'{self.step_override_forward_right_norm:.2f}], '
            f'forward_scale={self.step_override_forward_scale_enabled}'
            f'@min={self.step_override_forward_min_scale:.2f}, '
            f'forward_yaw_corr={self.step_override_forward_yaw_correction_enabled}, '
            f'forward_yaw_gain={self.step_override_forward_yaw_gain_norm_per_radps:.2f}, '
            f'forward_yaw_max_delta={self.step_override_forward_yaw_max_delta_norm:.2f}, '
            f'near_cmd_vel={self.near_approach_use_cmd_vel}'
            f'@dist_err<={self.near_approach_distance_error_m:.2f}m, '
            f'turn_cw=[{self.step_override_turn_mode},'
            f'{self.step_override_turn_cw_left_norm:.2f},'
            f'{self.step_override_turn_cw_right_norm:.2f}], '
            f'turn_ccw=[{self.step_override_turn_mode},'
            f'{self.step_override_turn_ccw_left_norm:.2f},'
            f'{self.step_override_turn_ccw_right_norm:.2f}], '
            f'duck_turn_slow={self.step_override_duck_align_turn_enabled}, '
            f'duck_turn_cw=[{self.step_override_turn_mode},'
            f'{self.step_override_duck_align_cw_left_norm:.3f},'
            f'{self.step_override_duck_align_cw_right_norm:.3f}], '
            f'duck_turn_ccw=[{self.step_override_turn_mode},'
            f'{self.step_override_duck_align_ccw_left_norm:.3f},'
            f'{self.step_override_duck_align_ccw_right_norm:.3f}]',
            category='state',
        )

    def _tf_callback(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            frame_id = str(transform.header.frame_id)
            child_frame_id = str(transform.child_frame_id)
            if frame_id != self.target_frame_id or child_frame_id != self.target_child_frame_id:
                continue

            roll, pitch, yaw = self._rpy_from_quaternion(transform.transform.rotation)
            self.latest_tag = TagPose(
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
            return

    def _pose_callback(self, msg: Odometry) -> None:
        self.pose_x = float(msg.pose.pose.position.x)
        self.pose_y = float(msg.pose.pose.position.y)
        self.pose_yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        self.pose_stamp_sec = self._stamp_to_sec(msg.header.stamp)
        self.pose_received_sec = self._now_sec()

    def _detections_callback(self, msg: AprilTagDetectionArray) -> None:
        info = DetectionInfo(
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
            if self.target_tag_id >= 0 and int(detection.id) != self.target_tag_id:
                continue
            info.matched = True
            info.selected_family = family
            info.selected_id = tag_id
            info.decision_margin = float(detection.decision_margin)
            info.hamming = int(detection.hamming)
            info.centre_x = float(detection.centre.x)
            info.centre_y = float(detection.centre.y)

        info.decoded_tags = ';'.join(decoded_parts)
        self.latest_detection = info

    def _timer_callback(self) -> None:
        now_sec = self._now_sec()

        if self.duck_phase:
            self._run_duck_sequence(now_sec)
            return

        if self.finished:
            self._publish_state('FINISHED')
            self._publish_zero_if_enabled()
            return

        if self.lateral_shift_phase:
            self._run_lateral_shift_sequence(now_sec)
            return

        if self.bridge_lateral_twt_phase:
            self._run_bridge_lateral_twt(now_sec)
            return

        tag = self.latest_tag
        if tag is None:
            self._handle_blocked(now_sec, 'WAITING_TAG', 'waiting for matching AprilTag TF')
            return

        age_sec = now_sec - tag.stamp_sec
        rx_age_sec = now_sec - tag.received_sec
        if self.stale_timeout_sec > 0.0 and age_sec > self.stale_timeout_sec:
            if (
                self.lateral_shift_sequence_enabled
                and self.lateral_shift_continue_after_tag_loss
                and self.lateral_shift_locked_tag is not None
                and not self.lateral_shift_phase
            ):
                if (
                    self.lateral_shift_pre_align_enabled
                    and not self.lateral_shift_pre_align_done
                ):
                    if self.lateral_shift_pre_align_use_visual_feedback:
                        if not self._complete_lateral_pre_align_from_best(now_sec, 'tag_lost'):
                            self._handle_blocked(
                                now_sec,
                                'LATERAL_PRE_ALIGN_TAG_LOST',
                                'visual pre-align needs live tag and no good visual sample is locked',
                            )
                            return
                    if (
                        not self.lateral_shift_pre_align_done
                        and self._run_lateral_shift_pre_align(
                            now_sec,
                            self.lateral_shift_locked_tag,
                        )
                    ):
                        return
                approach_distance_m = self._approach_distance(
                    self.lateral_shift_locked_tag,
                    math.sqrt(
                        self.lateral_shift_locked_tag.x_m * self.lateral_shift_locked_tag.x_m
                        + self.lateral_shift_locked_tag.y_m * self.lateral_shift_locked_tag.y_m
                        + self.lateral_shift_locked_tag.z_m * self.lateral_shift_locked_tag.z_m
                    ),
                )
                if self._start_lateral_shift_sequence(
                    now_sec,
                    self.lateral_shift_locked_tag,
                    approach_distance_m,
                    source='locked_tag_after_loss',
                ):
                    self._run_lateral_shift_sequence(now_sec)
                    return
            if self._run_locked_reference(now_sec, f'tag_stale age={age_sec:.2f}s'):
                return
            self._handle_blocked(
                now_sec,
                'STALE_TAG',
                f'stale tag TF age={age_sec:.2f}s>{self.stale_timeout_sec:.2f}s',
            )
            return

        distance_m = math.sqrt(tag.x_m * tag.x_m + tag.y_m * tag.y_m + tag.z_m * tag.z_m)
        if distance_m < self.min_distance_m or distance_m > self.max_distance_m:
            self._handle_blocked(
                now_sec,
                'DISTANCE_BLOCKED',
                f'tag distance {distance_m:.2f}m outside [{self.min_distance_m:.2f}, '
                f'{self.max_distance_m:.2f}]m',
            )
            return

        approach_distance_m = self._approach_distance(tag, distance_m)
        if self.bridge_lateral_calibration_enabled:
            if self._run_bridge_lateral_calibration(
                now_sec,
                tag,
                distance_m,
                approach_distance_m,
                age_sec,
                rx_age_sec,
            ):
                return

        (
            metric_name,
            measured_rad,
            target_rad,
            raw_error_rad,
            control_error_rad,
        ) = self._alignment_error(tag)
        if self.max_abs_error_rad > 0.0 and abs(raw_error_rad) > self.max_abs_error_rad:
            self._handle_blocked(
                now_sec,
                'ERROR_BLOCKED',
                f'{metric_name} error {math.degrees(raw_error_rad):.1f}deg exceeds '
                f'{math.degrees(self.max_abs_error_rad):.1f}deg',
            )
            return

        distance_error_m = approach_distance_m - self.target_distance_m
        reached_distance = distance_error_m <= self.distance_deadband_m
        too_close = distance_error_m < -self.distance_deadband_m
        aligned = abs(raw_error_rad) <= self.yaw_deadband_rad

        if self.lateral_shift_sequence_enabled:
            if (
                self.lateral_shift_pre_align_enabled
                and not self.lateral_shift_pre_align_done
            ):
                if self._run_lateral_shift_pre_align(now_sec, tag):
                    return
            plan_tag = tag
            plan_distance_m = approach_distance_m
            plan_source = 'live_tag'
            if self.lateral_shift_pre_align_enabled and self.lateral_shift_locked_tag is not None:
                plan_tag = self.lateral_shift_locked_tag
                plan_distance_m = self._approach_distance(
                    plan_tag,
                    math.sqrt(
                        plan_tag.x_m * plan_tag.x_m
                        + plan_tag.y_m * plan_tag.y_m
                        + plan_tag.z_m * plan_tag.z_m
                    ),
                )
                plan_source = 'pre_align_locked_tag'
            if self._start_lateral_shift_sequence(
                now_sec,
                plan_tag,
                plan_distance_m,
                source=plan_source,
            ):
                self._run_lateral_shift_sequence(now_sec)
                return

        if aligned:
            if self.aligned_since_sec is None:
                self.aligned_since_sec = now_sec
            hold_sec = now_sec - self.aligned_since_sec
        else:
            self.aligned_since_sec = None
            hold_sec = 0.0

        held = aligned and hold_sec >= self.aligned_hold_sec
        if (
            held
            and self.finish_after_aligned_hold
            and (not self.enable_forward_approach or reached_distance)
        ):
            self.finished = True

        state = 'ALIGNED' if aligned else 'ALIGNING'
        cmd_vx = 0.0
        cmd_wz = 0.0 if aligned else self._compute_wz(control_error_rad)
        step_override = self._step_override_for_state(state, cmd_vx, cmd_wz)

        if self.enable_forward_approach:
            yaw_ok_for_forward = (
                aligned
                if self.approach_requires_aligned
                else abs(raw_error_rad) <= self.forward_max_yaw_error_rad
            )
            if too_close:
                state = 'TOO_CLOSE' if aligned else 'ALIGNING'
                cmd_wz = 0.0 if aligned else cmd_wz
            elif reached_distance and aligned:
                state = 'AT_TARGET'
                cmd_wz = 0.0
            elif reached_distance:
                state = 'ALIGNING'
                cmd_wz = self._compute_wz(control_error_rad)
            elif yaw_ok_for_forward:
                state = 'APPROACHING'
                cmd_vx = self._compute_vx(distance_error_m)
                cmd_wz = self._compute_wz(control_error_rad)
            else:
                state = 'ALIGNING'
                cmd_wz = self._compute_wz(control_error_rad)
            step_override = self._step_override_for_state(
                state,
                cmd_vx,
                cmd_wz,
                distance_error_m,
            )

        if state == 'AT_TARGET' and aligned:
            if self._enter_duck_prepare(now_sec, 'visual_at_target'):
                return

        self._update_locked_reference(
            now_sec,
            tag,
            raw_error_rad,
            control_error_rad,
            approach_distance_m,
            distance_error_m,
        )

        if self.enable_control:
            self._publish_mode(now_sec)
            self._publish_control(cmd_vx, cmd_wz, step_override)

        self._publish_state(state)
        self._publish_debug(
            tag,
            measured_rad,
            target_rad,
            raw_error_rad,
            control_error_rad,
            cmd_vx,
            cmd_wz,
            distance_m,
            approach_distance_m,
            distance_error_m,
            step_override,
        )

        if (now_sec - self.last_log_time) >= self.log_interval_sec:
            self.last_log_time = now_sec
            self._publish_feedback(
                'AprilTag align: '
                f'state={state}, metric={metric_name}, '
                f'target={math.degrees(target_rad):+.1f}deg, '
                f'measured={math.degrees(measured_rad):+.1f}deg, '
                f'raw_err=measured-target={math.degrees(raw_error_rad):+.1f}deg, '
                f'ctrl_err={math.degrees(control_error_rad):+.1f}deg, '
                f'deadband={math.degrees(self.yaw_deadband_rad):.1f}deg, '
                f'forward_gate={math.degrees(self.forward_max_yaw_error_rad):.1f}deg, '
                f'approach_requires_aligned={self.approach_requires_aligned}, '
                f'near_cmd_vel={self.near_approach_use_cmd_vel}'
                f'@dist_err<={self.near_approach_distance_error_m:.3f}m, '
                f'cmd_vx={cmd_vx:+.3f}m/s, '
                f'cmd_wz={cmd_wz:+.3f}rad/s, '
                f'step_override={self._step_override_text(step_override)}, '
                f'active_output={self._active_output_text(cmd_vx, cmd_wz, step_override)}, '
                f'output={self.control_output_mode}, '
                f'wz_limits=[{self.min_abs_angular_z:.2f},{self.max_abs_angular_z:.2f}]rad/s, '
                f'vx_limits=[{self.min_forward_speed_mps:.2f},{self.max_forward_speed_mps:.2f}]m/s, '
                f'xyz=({tag.x_m:+.3f},{tag.y_m:+.3f},{tag.z_m:+.3f})m, '
                f'dist={distance_m:.3f}m, '
                f'approach_axis={self.approach_distance_axis}, '
                f'approach_dist={approach_distance_m:.3f}m, '
                f'target_dist={self.target_distance_m:.3f}m, '
                f'dist_err={distance_error_m:+.3f}m, '
                f'dist_deadband={self.distance_deadband_m:.3f}m, '
                f'center_angle={math.degrees(self._center_angle_rad(tag, include_offset=self._uses_center_offset())):+.1f}deg, '
                f'offset_x={self.target_offset_x_m:+.3f}m, '
                f'rpy=({math.degrees(tag.roll_rad):+.1f},'
                f'{math.degrees(tag.pitch_rad):+.1f},'
                f'{math.degrees(tag.yaw_rad):+.1f})deg, '
                f'tf_age={age_sec:.2f}s, rx_age={rx_age_sec:.2f}s, '
                f'decoded={self._decoded_tags_text()}, '
                f'selected={self._detection_text(now_sec)}, '
                f'target_tf={self.target_frame_id}->{self.target_child_frame_id}, '
                f'control={self.enable_control}, '
                f'forward_approach={self.enable_forward_approach}, '
                f'locked={self._locked_reference_text(now_sec)}, '
                f'hold={hold_sec:.2f}/{self.aligned_hold_sec:.2f}s',
                category='detection',
            )

    def _alignment_error(self, tag: TagPose) -> Tuple[str, float, float, float, float]:
        if self.alignment_mode == 'center_x':
            measured = self._center_angle_rad(tag, include_offset=False)
            target = self.target_center_angle_rad
            raw_error = self._normalize_angle(measured - target)
            control_error = self.control_sign * self.center_control_sign * raw_error
            return 'center_x', measured, target, raw_error, control_error

        if self._uses_center_offset():
            measured = self._center_angle_rad(tag, include_offset=True)
            target = self.target_center_angle_rad
            raw_error = self._normalize_angle(measured - target)
            control_error = self.control_sign * self.center_control_sign * raw_error
            return 'center_x_offset', measured, target, raw_error, control_error

        if self.alignment_mode == 'pose_roll':
            measured = tag.roll_rad
            target = self.target_pose_roll_rad
            name = 'pose_roll'
        elif self.alignment_mode == 'pose_pitch':
            measured = tag.pitch_rad
            target = self.target_pose_pitch_rad
            name = 'pose_pitch'
        else:
            measured = tag.yaw_rad
            target = self.target_pose_yaw_rad
            name = 'pose_yaw'

        raw_error = self._normalize_angle(measured - target)
        control_error = self.control_sign * self.pose_control_sign * raw_error
        return name, measured, target, raw_error, control_error

    def _lateral_pre_align_error(self, tag: TagPose) -> Tuple[str, float, float, float, float]:
        """Compute the visual metric used only to lock the Odometry yaw target."""
        if self.lateral_shift_pre_align_mode == 'tag_center':
            measured = self._center_angle_rad(tag, include_offset=False)
            target = self.target_center_angle_rad
            raw_error = self._normalize_angle(measured - target)
            control_error = self.control_sign * self.center_control_sign * raw_error
            return 'tag_center', measured, target, raw_error, control_error

        if self.lateral_shift_pre_align_mode == 'centerline':
            measured = self._center_angle_rad(tag, include_offset=True)
            target = self.target_center_angle_rad
            raw_error = self._normalize_angle(measured - target)
            control_error = self.control_sign * self.center_control_sign * raw_error
            return 'centerline', measured, target, raw_error, control_error

        if self.lateral_shift_pre_align_mode == 'pose_roll':
            measured = tag.roll_rad
            target = self.target_pose_roll_rad
            name = 'pose_roll'
        elif self.lateral_shift_pre_align_mode == 'pose_pitch':
            measured = tag.pitch_rad
            target = self.target_pose_pitch_rad
            name = 'pose_pitch'
        else:
            measured = tag.yaw_rad
            target = self.target_pose_yaw_rad
            name = 'pose_yaw'

        raw_error = self._normalize_angle(measured - target)
        control_error = self.control_sign * self.pose_control_sign * raw_error
        return name, measured, target, raw_error, control_error

    def _center_angle_rad(self, tag: TagPose, *, include_offset: bool = False) -> float:
        if include_offset:
            return math.atan2(self._lateral_error_x_m(tag), tag.z_m)
        return math.atan2(tag.x_m, tag.z_m)

    def _uses_target_tag_x(self) -> bool:
        return math.isfinite(self.target_tag_x_m)

    def _lateral_error_x_m(self, tag: TagPose) -> float:
        if self._uses_target_tag_x():
            return tag.x_m - self.target_tag_x_m - self.robot_center_offset_x_m
        return tag.x_m + self.target_offset_x_m - self.robot_center_offset_x_m

    def _uses_center_offset(self) -> bool:
        return self.alignment_mode in ('center_x_offset', 'offset_center_x')

    def _compute_wz(self, control_error_rad: float) -> float:
        return self._compute_wz_with_deadband(control_error_rad, self.yaw_deadband_rad)

    def _compute_wz_with_deadband(
        self,
        control_error_rad: float,
        deadband_rad: float,
    ) -> float:
        abs_error = abs(control_error_rad)
        if abs_error <= deadband_rad:
            return 0.0

        excess = max(0.0, abs_error - deadband_rad)
        magnitude = self.yaw_kp * excess
        if self.min_abs_angular_z > 0.0:
            magnitude = max(self.min_abs_angular_z, magnitude)
        magnitude = min(self.max_abs_angular_z, magnitude)
        return math.copysign(magnitude, control_error_rad)

    def _compute_vx(self, distance_error_m: float) -> float:
        if distance_error_m <= self.distance_deadband_m:
            return 0.0

        excess = distance_error_m - self.distance_deadband_m
        magnitude = self.forward_kp * excess
        if self.min_forward_speed_mps > 0.0:
            magnitude = max(self.min_forward_speed_mps, magnitude)
        magnitude = min(self.max_forward_speed_mps, magnitude)
        return magnitude

    def _compute_bridge_lateral_wz(self, error_x_m: float) -> float:
        abs_error = abs(error_x_m)
        if abs_error <= self.bridge_lateral_deadband_m:
            return 0.0

        excess = abs_error - self.bridge_lateral_deadband_m
        magnitude = self.bridge_lateral_kp_radps_per_m * excess
        if self.bridge_lateral_min_abs_angular_z > 0.0:
            magnitude = max(self.bridge_lateral_min_abs_angular_z, magnitude)
        if self.bridge_lateral_max_abs_angular_z > 0.0:
            magnitude = min(self.bridge_lateral_max_abs_angular_z, magnitude)
        return math.copysign(magnitude, self.bridge_lateral_control_sign * error_x_m)

    def _bridge_lateral_step_override(
        self,
        error_x_m: float,
    ) -> Optional[Tuple[int, float, float]]:
        if not self.bridge_lateral_use_step_override:
            return None
        if abs(error_x_m) <= self.bridge_lateral_deadband_m:
            return None
        if error_x_m < 0.0:
            return (
                self.bridge_lateral_step_override_mode,
                self.bridge_lateral_left_error_left_norm,
                self.bridge_lateral_left_error_right_norm,
            )
        return (
            self.bridge_lateral_step_override_mode,
            self.bridge_lateral_right_error_left_norm,
            self.bridge_lateral_right_error_right_norm,
        )

    def _bridge_lateral_yaw_step_override(
        self,
        cmd_wz: float,
    ) -> Optional[Tuple[int, float, float]]:
        if not self.bridge_lateral_yaw_use_step_override:
            return None
        if abs(cmd_wz) <= 1e-6:
            return None
        if cmd_wz > 0.0:
            return (
                self.bridge_lateral_yaw_turn_mode,
                self.bridge_lateral_yaw_ccw_left_norm,
                self.bridge_lateral_yaw_ccw_right_norm,
            )
        return (
            self.bridge_lateral_yaw_turn_mode,
            self.bridge_lateral_yaw_cw_left_norm,
            self.bridge_lateral_yaw_cw_right_norm,
        )

    def _bridge_lateral_yaw_measurement(
        self,
        tag: TagPose,
        now_sec: float,
    ) -> Tuple[Optional[float], str]:
        source = self.bridge_lateral_yaw_reference_source
        if source in ('auto', 'odom') and self._pose_is_fresh(now_sec):
            return self.pose_yaw, 'odom'
        if source in ('auto', 'tag', 'tag_yaw', 'tag_pose_yaw'):
            return tag.yaw_rad, 'tag_yaw'
        return None, source

    def _bridge_lateral_twt_forward_step_override(self) -> Tuple[int, float, float]:
        return (
            self.bridge_lateral_twt_forward_mode,
            self.bridge_lateral_twt_forward_left_norm,
            self.bridge_lateral_twt_forward_right_norm,
        )

    def _start_bridge_lateral_twt(
        self,
        now_sec: float,
        tag: TagPose,
        error_x_m: float,
    ) -> bool:
        if abs(error_x_m) <= self.bridge_lateral_deadband_m:
            return False
        if not self._pose_is_fresh(now_sec):
            self._handle_blocked(
                now_sec,
                'BRIDGE_LATERAL_TWT_WAITING_ODOM',
                'turn-walk-turn bridge correction needs fresh Odometry before the nudge',
            )
            return True

        sin_yaw = max(1e-6, math.sin(self.bridge_lateral_twt_yaw_rad))
        raw_distance_m = abs(error_x_m) / sin_yaw
        distance_m = raw_distance_m * self.bridge_lateral_twt_distance_scale
        distance_m = max(
            self.bridge_lateral_twt_min_distance_m,
            min(self.bridge_lateral_twt_max_distance_m, distance_m),
        )
        side_delta_rad = (
            self.bridge_lateral_twt_yaw_sign
            * math.copysign(self.bridge_lateral_twt_yaw_rad, error_x_m)
        )

        self.bridge_lateral_twt_iteration += 1
        self.bridge_lateral_twt_phase = 'turn_out'
        self.bridge_lateral_twt_base_yaw = self.pose_yaw
        self.bridge_lateral_twt_side_yaw = self._normalize_angle(self.pose_yaw + side_delta_rad)
        self.bridge_lateral_twt_start_x = None
        self.bridge_lateral_twt_start_y = None
        self.bridge_lateral_twt_distance_m = distance_m
        self.bridge_lateral_twt_error_x_m = error_x_m
        self.bridge_lateral_twt_live_error_x_m = math.nan
        self.bridge_lateral_twt_stop_reason = ''
        self.bridge_lateral_aligned_since_sec = None
        self.bridge_lateral_yaw_aligned_since_sec = None
        self.bridge_lateral_position_locked = False

        if (now_sec - self.last_log_time) >= self.log_interval_sec:
            self.last_log_time = now_sec
            self._publish_feedback(
                'Bridge AprilTag turn-walk-turn start: '
                f'iter={self.bridge_lateral_twt_iteration}, '
                f'error_x={error_x_m:+.3f}m, tag_x={tag.x_m:+.3f}m, '
                f'target_tag_x={self.target_tag_x_m:+.3f}m, '
                f'base_yaw={math.degrees(self.bridge_lateral_twt_base_yaw):+.1f}deg, '
                f'side_yaw={math.degrees(self.bridge_lateral_twt_side_yaw):+.1f}deg, '
                f'yaw_step={math.degrees(side_delta_rad):+.1f}deg, '
                f'walk={distance_m:.3f}m raw={raw_distance_m:.3f}m, '
                f'scale={self.bridge_lateral_twt_distance_scale:.2f}',
                category='detection',
            )
        return True

    def _bridge_lateral_twt_progress(self) -> float:
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.bridge_lateral_twt_start_x is None
            or self.bridge_lateral_twt_start_y is None
        ):
            return 0.0
        dx = self.pose_x - self.bridge_lateral_twt_start_x
        dy = self.pose_y - self.bridge_lateral_twt_start_y
        return math.sqrt(dx * dx + dy * dy)

    def _run_bridge_lateral_twt(self, now_sec: float) -> None:
        if not self._pose_is_fresh(now_sec):
            self.bridge_lateral_twt_phase = ''
            self._handle_blocked(
                now_sec,
                'BRIDGE_LATERAL_TWT_ODOM_STALE',
                'turn-walk-turn bridge correction stopped because Odometry is stale',
            )
            return

        cmd_vx = 0.0
        cmd_wz = 0.0
        step_override: Optional[Tuple[int, float, float]] = None
        state = f'BRIDGE_LATERAL_TWT_{self.bridge_lateral_twt_phase.upper()}'
        yaw_target = self.bridge_lateral_twt_side_yaw
        if self.bridge_lateral_twt_phase == 'turn_back':
            yaw_target = self.bridge_lateral_twt_base_yaw

        yaw_error_rad = 0.0
        walked_m = self._bridge_lateral_twt_progress()
        remaining_m = max(0.0, self.bridge_lateral_twt_distance_m - walked_m)

        if self.bridge_lateral_twt_phase in ('turn_out', 'turn_back'):
            if yaw_target is None:
                self.bridge_lateral_twt_phase = ''
                self._handle_blocked(
                    now_sec,
                    'BRIDGE_LATERAL_TWT_NO_YAW_TARGET',
                    'turn-walk-turn bridge correction has no yaw target',
                )
                return
            yaw_error_rad = self._normalize_angle(yaw_target - self.pose_yaw)
            if abs(yaw_error_rad) <= self.bridge_lateral_yaw_deadband_rad:
                if self.bridge_lateral_twt_phase == 'turn_out':
                    self.bridge_lateral_twt_phase = 'walk'
                    self.bridge_lateral_twt_start_x = self.pose_x
                    self.bridge_lateral_twt_start_y = self.pose_y
                    state = 'BRIDGE_LATERAL_TWT_WALK'
                    walked_m = 0.0
                    remaining_m = self.bridge_lateral_twt_distance_m
                    step_override = self._bridge_lateral_twt_forward_step_override()
                    cmd_vx = self.bridge_lateral_forward_speed_mps
                else:
                    self.bridge_lateral_twt_phase = ''
                    self._publish_zero_if_enabled(now_sec)
                    self._publish_state('BRIDGE_LATERAL_TWT_RECHECK_TAG')
                    return
            else:
                cmd_wz = self._compute_wz_with_deadband(
                    yaw_error_rad,
                    self.bridge_lateral_yaw_deadband_rad,
                )
                step_override = self._bridge_lateral_yaw_step_override(cmd_wz)
                if step_override is None:
                    step_override = self._step_override_for_state('ALIGNING', cmd_vx, cmd_wz)

        elif self.bridge_lateral_twt_phase == 'walk':
            walked_m = self._bridge_lateral_twt_progress()
            remaining_m = max(0.0, self.bridge_lateral_twt_distance_m - walked_m)
            if remaining_m <= self.bridge_lateral_twt_distance_deadband_m:
                self.bridge_lateral_twt_phase = 'turn_back'
                self.bridge_lateral_twt_stop_reason = 'planned_distance_reached'
                state = 'BRIDGE_LATERAL_TWT_TURN_BACK'
                if self.bridge_lateral_twt_base_yaw is None:
                    yaw_error_rad = 0.0
                else:
                    yaw_error_rad = self._normalize_angle(
                        self.bridge_lateral_twt_base_yaw - self.pose_yaw
                    )
                if abs(yaw_error_rad) > self.bridge_lateral_yaw_deadband_rad:
                    cmd_wz = self._compute_wz_with_deadband(
                        yaw_error_rad,
                        self.bridge_lateral_yaw_deadband_rad,
                    )
                    step_override = self._bridge_lateral_yaw_step_override(cmd_wz)
            else:
                cmd_vx = self.bridge_lateral_forward_speed_mps
                step_override = self._bridge_lateral_twt_forward_step_override()
        else:
            self.bridge_lateral_twt_phase = ''
            self._publish_zero_if_enabled(now_sec)
            return

        if self.enable_control:
            self._publish_mode(now_sec)
            self._publish_control(cmd_vx, cmd_wz, step_override)
        self._publish_state(state)

        if (now_sec - self.last_log_time) >= self.log_interval_sec:
            self.last_log_time = now_sec
            yaw_target_text = self._angle_text(yaw_target)
            self._publish_feedback(
                'Bridge AprilTag turn-walk-turn: '
                f'state={state}, iter={self.bridge_lateral_twt_iteration}, '
                f'phase={self.bridge_lateral_twt_phase or "recheck"}, '
                f'planned_error_x={self.bridge_lateral_twt_error_x_m:+.3f}m, '
                f'live_error_x={self.bridge_lateral_twt_live_error_x_m:+.3f}m, '
                f'stop_reason={self.bridge_lateral_twt_stop_reason or "none"}, '
                f'walk={walked_m:.3f}/{self.bridge_lateral_twt_distance_m:.3f}m, '
                f'remaining={remaining_m:.3f}m, '
                f'yaw_now={math.degrees(self.pose_yaw):+.1f}deg, '
                f'yaw_target={yaw_target_text}, '
                f'yaw_err={math.degrees(yaw_error_rad):+.1f}deg, '
                f'cmd_vx={cmd_vx:+.3f}m/s, cmd_wz={cmd_wz:+.3f}rad/s, '
                f'step_override={self._step_override_text(step_override)}, '
                f'active_output={self._active_output_text(cmd_vx, cmd_wz, step_override)}, '
                f'control={self.enable_control}',
                category='detection',
            )

    def _approach_distance(self, tag: TagPose, distance_m: float) -> float:
        if self.approach_distance_axis == 'euclidean':
            return distance_m
        return tag.z_m

    def _run_bridge_lateral_calibration(
        self,
        now_sec: float,
        tag: TagPose,
        distance_m: float,
        approach_distance_m: float,
        age_sec: float,
        rx_age_sec: float,
    ) -> bool:
        if not self._uses_target_tag_x():
            self._handle_blocked(
                now_sec,
                'BRIDGE_LATERAL_WAITING_TARGET_X',
                'bridge lateral calibration needs target_tag_x_m set to the measured '
                'tag_x at the bridge centerline',
            )
            return True

        error_x_m = self._lateral_error_x_m(tag)
        yaw_measurement, yaw_source = self._bridge_lateral_yaw_measurement(tag, now_sec)
        if (
            self.bridge_lateral_yaw_realign_enabled
            and self.bridge_lateral_reference_yaw is None
            and yaw_measurement is not None
        ):
            self.bridge_lateral_reference_yaw = yaw_measurement
            self.bridge_lateral_reference_yaw_source = yaw_source

        if (
            self.bridge_lateral_max_abs_error_m > 0.0
            and abs(error_x_m) > self.bridge_lateral_max_abs_error_m
        ):
            self.bridge_lateral_aligned_since_sec = None
            self._handle_blocked(
                now_sec,
                'BRIDGE_LATERAL_ERROR_BLOCKED',
                f'bridge tag_x error {error_x_m:+.3f}m exceeds '
                f'{self.bridge_lateral_max_abs_error_m:.3f}m safety gate',
            )
            return True

        live_lateral_aligned = abs(error_x_m) <= self.bridge_lateral_deadband_m
        lateral_aligned = live_lateral_aligned or self.bridge_lateral_position_locked
        if lateral_aligned:
            if self.bridge_lateral_aligned_since_sec is None:
                self.bridge_lateral_aligned_since_sec = now_sec
            hold_sec = now_sec - self.bridge_lateral_aligned_since_sec
        else:
            self.bridge_lateral_aligned_since_sec = None
            self.bridge_lateral_yaw_aligned_since_sec = None
            hold_sec = 0.0
            if self.bridge_lateral_control_strategy == 'turn_walk_turn':
                if (
                    self.bridge_lateral_twt_max_iterations > 0
                    and self.bridge_lateral_twt_iteration
                    >= self.bridge_lateral_twt_max_iterations
                ):
                    state = 'BRIDGE_LATERAL_TWT_MAX_ITERATIONS'
                    self._publish_zero_if_enabled(now_sec)
                    self._publish_state(state)
                    self._publish_debug(
                        tag,
                        tag.x_m,
                        self.target_tag_x_m,
                        error_x_m,
                        error_x_m,
                        0.0,
                        0.0,
                        distance_m,
                        approach_distance_m,
                        error_x_m,
                        None,
                    )
                    if (now_sec - self.last_log_time) >= self.log_interval_sec:
                        self.last_log_time = now_sec
                        self._publish_feedback(
                            'Bridge AprilTag turn-walk-turn stopped: '
                            f'state={state}, '
                            f'iterations={self.bridge_lateral_twt_iteration}/'
                            f'{self.bridge_lateral_twt_max_iterations}, '
                            f'tag_x={tag.x_m:+.3f}m, '
                            f'target_tag_x={self.target_tag_x_m:+.3f}m, '
                            f'error_x={error_x_m:+.3f}m, '
                            f'deadband={self.bridge_lateral_deadband_m:.3f}m, '
                            'increase bridge_lateral_twt_max_iterations or tune '
                            'bridge_lateral_twt_yaw_deg/max_distance after checking '
                            'single-step direction',
                            category='detection',
                        )
                    return True
                self._start_bridge_lateral_twt(now_sec, tag, error_x_m)
                return True

        lateral_held = self.bridge_lateral_position_locked or (
            live_lateral_aligned and hold_sec >= self.bridge_lateral_hold_sec
        )
        if lateral_held and self.bridge_lateral_lock_after_hold:
            self.bridge_lateral_position_locked = True
            lateral_aligned = True
        yaw_error_rad = 0.0
        yaw_hold_sec = 0.0
        yaw_held = not self.bridge_lateral_yaw_realign_enabled
        yaw_target_text = self._angle_text(self.bridge_lateral_reference_yaw)
        yaw_current_text = self._angle_text(yaw_measurement)

        state = (
            'BRIDGE_LATERAL_ALIGNED'
            if lateral_aligned
            else 'BRIDGE_LATERAL_CALIBRATING'
        )
        cmd_vx = 0.0 if lateral_aligned else self.bridge_lateral_forward_speed_mps
        cmd_wz = 0.0 if lateral_aligned else self._compute_bridge_lateral_wz(error_x_m)

        if not lateral_held:
            step_override = self._bridge_lateral_step_override(error_x_m)
            if step_override is None:
                control_state = 'APPROACHING' if cmd_vx > 1e-6 else 'ALIGNED'
                step_override = self._step_override_for_state(
                    control_state,
                    cmd_vx,
                    cmd_wz,
                    abs(error_x_m),
                )
        elif self.bridge_lateral_yaw_realign_enabled:
            yaw_measurement, yaw_source = self._bridge_lateral_yaw_measurement(tag, now_sec)
            if yaw_measurement is None or self.bridge_lateral_reference_yaw is None:
                self._handle_blocked(
                    now_sec,
                    'BRIDGE_LATERAL_WAITING_ODOM_YAW',
                    'bridge lateral yaw realign needs a fresh yaw measurement '
                    f'(source={self.bridge_lateral_yaw_reference_source}) to restore '
                    'the pre-calibration heading',
                )
                return True

            yaw_error_rad = self._normalize_angle(
                self.bridge_lateral_reference_yaw - yaw_measurement
            )
            yaw_current_text = self._angle_text(yaw_measurement)
            yaw_aligned = abs(yaw_error_rad) <= self.bridge_lateral_yaw_deadband_rad
            if yaw_aligned:
                if self.bridge_lateral_yaw_aligned_since_sec is None:
                    self.bridge_lateral_yaw_aligned_since_sec = now_sec
                yaw_hold_sec = now_sec - self.bridge_lateral_yaw_aligned_since_sec
            else:
                self.bridge_lateral_yaw_aligned_since_sec = None
                yaw_hold_sec = 0.0

            yaw_held = yaw_aligned and yaw_hold_sec >= self.bridge_lateral_yaw_hold_sec
            if yaw_held:
                state = 'BRIDGE_LATERAL_DONE'
                cmd_vx = 0.0
                cmd_wz = 0.0
                step_override = None
            elif yaw_aligned:
                state = 'BRIDGE_LATERAL_YAW_ALIGNED'
                cmd_vx = 0.0
                cmd_wz = 0.0
                step_override = None
            else:
                state = 'BRIDGE_LATERAL_YAW_ALIGNING'
                cmd_vx = 0.0
                cmd_wz = self._compute_wz_with_deadband(
                    yaw_error_rad,
                    self.bridge_lateral_yaw_deadband_rad,
                )
                step_override = self._bridge_lateral_yaw_step_override(cmd_wz)
                if step_override is None:
                    step_override = self._step_override_for_state('ALIGNING', cmd_vx, cmd_wz)
        else:
            state = 'BRIDGE_LATERAL_DONE'
            step_override = None

        done = lateral_held and yaw_held
        if done and self.bridge_lateral_finish_after_hold:
            self.finished = True

        if self.enable_control:
            self._publish_mode(now_sec)
            self._publish_control(cmd_vx, cmd_wz, step_override)
        self._publish_state(state)
        self._publish_debug(
            tag,
            tag.x_m,
            self.target_tag_x_m,
            error_x_m,
            error_x_m,
            cmd_vx,
            cmd_wz,
            distance_m,
            approach_distance_m,
            error_x_m,
            step_override,
        )

        if done and self.bridge_lateral_finish_after_hold:
            self._publish_zero_if_enabled(now_sec)

        if (now_sec - self.last_log_time) >= self.log_interval_sec:
            self.last_log_time = now_sec
            self._publish_feedback(
                'Bridge AprilTag lateral calibration: '
                f'state={state}, tag_x={tag.x_m:+.3f}m, '
                f'target_tag_x={self.target_tag_x_m:+.3f}m, '
                f'robot_center_offset_x={self.robot_center_offset_x_m:+.3f}m, '
                f'error_x={error_x_m:+.3f}m, '
                f'deadband={self.bridge_lateral_deadband_m:.3f}m, '
                f'hold={hold_sec:.2f}/{self.bridge_lateral_hold_sec:.2f}s, '
                f'lateral_live={live_lateral_aligned}, '
                f'lateral_locked={self.bridge_lateral_position_locked}, '
                f'cmd_vx={cmd_vx:+.3f}m/s, cmd_wz={cmd_wz:+.3f}rad/s, '
                f'strategy={self.bridge_lateral_control_strategy}, '
                f'yaw_realign={self.bridge_lateral_yaw_realign_enabled}, '
                f'yaw_source={self.bridge_lateral_reference_yaw_source or yaw_source}, '
                f'yaw_ref={yaw_target_text}, yaw_now={yaw_current_text}, '
                f'yaw_err={math.degrees(yaw_error_rad):+.1f}deg, '
                f'yaw_deadband={math.degrees(self.bridge_lateral_yaw_deadband_rad):.1f}deg, '
                f'yaw_hold={yaw_hold_sec:.2f}/{self.bridge_lateral_yaw_hold_sec:.2f}s, '
                f'wz_sign={self.bridge_lateral_control_sign:+.0f}, '
                f'wz_gain={self.bridge_lateral_kp_radps_per_m:.2f}, '
                f'wz_limits=[{self.bridge_lateral_min_abs_angular_z:.2f},'
                f'{self.bridge_lateral_max_abs_angular_z:.2f}]rad/s, '
                f'direct_step_override={self.bridge_lateral_use_step_override}, '
                f'approach_axis={self.approach_distance_axis}, '
                f'approach_dist={approach_distance_m:.3f}m, '
                f'xyz=({tag.x_m:+.3f},{tag.y_m:+.3f},{tag.z_m:+.3f})m, '
                f'dist={distance_m:.3f}m, tf_age={age_sec:.2f}s, '
                f'rx_age={rx_age_sec:.2f}s, decoded={self._decoded_tags_text()}, '
                f'selected={self._detection_text(now_sec)}, '
                f'step_override={self._step_override_text(step_override)}, '
                f'active_output={self._active_output_text(cmd_vx, cmd_wz, step_override)}, '
                f'control={self.enable_control}',
                category='detection',
            )
        return True

    def _step_override_for_state(
        self,
        state: str,
        cmd_vx: float,
        cmd_wz: float,
        distance_error_m: Optional[float] = None,
    ) -> Optional[Tuple[int, float, float]]:
        if self.control_output_mode not in ('step_override', 'hybrid'):
            return None

        if state == 'APPROACHING' and cmd_vx > 1e-6:
            if (
                self.control_output_mode == 'hybrid'
                and self.near_approach_use_cmd_vel
                and distance_error_m is not None
                and distance_error_m <= self.near_approach_distance_error_m
            ):
                return None
            left = self.step_override_forward_left_norm
            right = self.step_override_forward_right_norm
            if (
                self.step_override_forward_scale_enabled
                and self.max_forward_speed_mps > 1e-6
            ):
                scale = cmd_vx / self.max_forward_speed_mps
                scale = max(self.step_override_forward_min_scale, min(1.0, scale))
                left = self._clamp_norm(left * scale)
                right = self._clamp_norm(right * scale)
            if (
                self.step_override_forward_yaw_correction_enabled
                and abs(cmd_wz) > 1e-6
            ):
                delta = self.step_override_forward_yaw_gain_norm_per_radps * cmd_wz
                delta = max(
                    -self.step_override_forward_yaw_max_delta_norm,
                    min(self.step_override_forward_yaw_max_delta_norm, delta),
                )
                left = self._clamp_norm(left - delta)
                right = self._clamp_norm(right + delta)
            return (
                self.step_override_forward_mode,
                left,
                right,
            )

        if self.control_output_mode != 'step_override':
            return None

        if state in ('ALIGNING', 'DUCK_ALIGNING') and abs(cmd_wz) > 1e-6:
            if state == 'DUCK_ALIGNING' and self.step_override_duck_align_turn_enabled:
                if cmd_wz > 0.0:
                    return (
                        self.step_override_turn_mode,
                        self.step_override_duck_align_ccw_left_norm,
                        self.step_override_duck_align_ccw_right_norm,
                    )
                return (
                    self.step_override_turn_mode,
                    self.step_override_duck_align_cw_left_norm,
                    self.step_override_duck_align_cw_right_norm,
                )
            if cmd_wz > 0.0:
                left = self.step_override_turn_ccw_left_norm
                right = self.step_override_turn_ccw_right_norm
            else:
                left = self.step_override_turn_cw_left_norm
                right = self.step_override_turn_cw_right_norm
            if self.step_override_turn_scale_enabled and self.max_abs_angular_z > 1e-6:
                scale = abs(cmd_wz) / self.max_abs_angular_z
                scale = max(self.step_override_turn_min_scale, min(1.0, scale))
                left = self._clamp_norm(left * scale)
                right = self._clamp_norm(right * scale)
            return (
                self.step_override_turn_mode,
                left,
                right,
            )

        return None

    def _pose_is_fresh(self, now_sec: float) -> bool:
        if self.pose_x is None or self.pose_y is None or self.pose_yaw is None:
            return False
        if self.pose_stale_timeout_sec <= 0.0:
            return True
        return (now_sec - self.pose_received_sec) <= self.pose_stale_timeout_sec

    def _update_locked_reference(
        self,
        now_sec: float,
        tag: TagPose,
        raw_error_rad: float,
        control_error_rad: float,
        approach_distance_m: float,
        distance_error_m: float,
    ) -> None:
        if not self.lock_reference_on_tag_loss:
            return
        if not self._pose_is_fresh(now_sec):
            return
        if self.locked_reference_active and not self.lock_reference_update_while_visible:
            return

        target_yaw = self._normalize_angle(self.pose_yaw + control_error_rad)
        remaining_distance = max(0.0, distance_error_m)
        first_lock = not self.locked_reference_active
        self.locked_reference_active = True
        self.locked_reference_target_yaw = target_yaw
        self.locked_reference_start_x = self.pose_x
        self.locked_reference_start_y = self.pose_y
        self.locked_reference_distance_m = remaining_distance
        self.locked_reference_source = (
            f'tag_x={tag.x_m:+.3f}m, tag_z={tag.z_m:+.3f}m, '
            f'center_x={tag.x_m + self.target_offset_x_m:+.3f}m, '
            f'raw_yaw={math.degrees(raw_error_rad):+.1f}deg, '
            f'approach={approach_distance_m:.3f}m'
        )
        self.locked_reference_stamp_sec = now_sec

        if first_lock:
            self._publish_feedback(
                'AprilTag locked odom reference: '
                f'target_yaw={math.degrees(target_yaw):+.1f}deg, '
                f'start=({self.pose_x:.3f},{self.pose_y:.3f}), '
                f'remaining={remaining_distance:.3f}m, '
                f'{self.locked_reference_source}',
                category='state',
            )

    def _record_lateral_pre_align_best(
        self,
        now_sec: float,
        tag: TagPose,
        metric_name: str,
        measured_rad: float,
        target_rad: float,
        raw_error_rad: float,
    ) -> None:
        if not self.lateral_shift_pre_align_use_visual_feedback:
            return
        if (
            self.lateral_shift_pre_align_best_error_rad is not None
            and abs(raw_error_rad) >= abs(self.lateral_shift_pre_align_best_error_rad)
        ):
            return
        self.lateral_shift_pre_align_best_metric = metric_name
        self.lateral_shift_pre_align_best_error_rad = raw_error_rad
        self.lateral_shift_pre_align_best_measured_rad = measured_rad
        self.lateral_shift_pre_align_best_target_rad = target_rad
        self.lateral_shift_pre_align_best_pose_yaw = self.pose_yaw
        self.lateral_shift_pre_align_best_tag = tag
        self.lateral_shift_pre_align_best_sec = now_sec

    def _complete_lateral_pre_align_from_best(self, now_sec: float, reason: str) -> bool:
        if not self.lateral_shift_pre_align_use_visual_feedback:
            return False
        if (
            self.lateral_shift_pre_align_best_tag is None
            or self.lateral_shift_pre_align_best_pose_yaw is None
            or self.lateral_shift_pre_align_best_error_rad is None
        ):
            return False
        if (
            self.lateral_shift_pre_align_max_visual_error_rad > 0.0
            and abs(self.lateral_shift_pre_align_best_error_rad)
            > self.lateral_shift_pre_align_max_visual_error_rad
        ):
            return False

        self.lateral_shift_pre_align_done = True
        self.lateral_shift_pre_align_since_sec = None
        self.lateral_shift_pre_align_target_yaw = self.lateral_shift_pre_align_best_pose_yaw
        self.lateral_shift_locked_tag = self.lateral_shift_pre_align_best_tag
        self._publish_feedback(
            'AprilTag lateral pre-align complete: '
            f'reason=best_visual_{reason}, '
            f'metric={self.lateral_shift_pre_align_best_metric}, '
            f'best_measured={math.degrees(self.lateral_shift_pre_align_best_measured_rad):+.1f}deg, '
            f'best_target={math.degrees(self.lateral_shift_pre_align_best_target_rad):+.1f}deg, '
            f'best_raw_err={math.degrees(self.lateral_shift_pre_align_best_error_rad):+.1f}deg, '
            f'best_pose_yaw={math.degrees(self.lateral_shift_pre_align_best_pose_yaw):+.1f}deg, '
            f'best_age={max(0.0, now_sec - self.lateral_shift_pre_align_best_sec):.2f}s, '
            f'tag_x={self.lateral_shift_locked_tag.x_m:+.3f}m, '
            f'tag_z={self.lateral_shift_locked_tag.z_m:+.3f}m. '
            'Continuing from best visual sample instead of blocking.',
            category='state',
        )
        return True

    def _run_lateral_shift_pre_align(self, now_sec: float, tag: TagPose) -> bool:
        """Lock one visual yaw correction, then rotate by Odometry to avoid oscillation."""
        if not self._pose_is_fresh(now_sec):
            self._handle_blocked(
                now_sec,
                'LATERAL_PRE_ALIGN_WAITING_ODOM',
                'pre-align needs fresh Odometry before locking yaw target',
            )
            return True

        metric_name, measured_rad, target_rad, raw_error_rad, control_error_rad = (
            self._lateral_pre_align_error(tag)
        )
        self._record_lateral_pre_align_best(
            now_sec,
            tag,
            metric_name,
            measured_rad,
            target_rad,
            raw_error_rad,
        )

        if self.lateral_shift_pre_align_target_yaw is None:
            self.lateral_shift_pre_align_start_sec = now_sec
            self.lateral_shift_pre_align_target_yaw = self._normalize_angle(
                self.pose_yaw + control_error_rad
            )
            self.lateral_shift_pre_align_source_metric = metric_name
            self.lateral_shift_pre_align_source_target_rad = target_rad
            self.lateral_shift_pre_align_source_angle_rad = measured_rad
            self.lateral_shift_pre_align_source_control_rad = control_error_rad
            self.lateral_shift_pre_align_source_x_m = tag.x_m
            self.lateral_shift_pre_align_source_z_m = tag.z_m
            self.lateral_shift_locked_tag = tag
            self._publish_feedback(
                'AprilTag lateral pre-align target locked: '
                f'metric={metric_name}, '
                f'tag_x={tag.x_m:+.3f}m, tag_z={tag.z_m:+.3f}m, '
                f'measured={math.degrees(measured_rad):+.1f}deg, '
                f'target={math.degrees(target_rad):+.1f}deg, '
                f'raw_err={math.degrees(raw_error_rad):+.1f}deg, '
                f'control_delta={math.degrees(control_error_rad):+.1f}deg, '
                f'rpy=({math.degrees(tag.roll_rad):+.1f},'
                f'{math.degrees(tag.pitch_rad):+.1f},'
                f'{math.degrees(tag.yaw_rad):+.1f})deg, '
                f'odom_yaw0={math.degrees(self.pose_yaw):+.1f}deg, '
                f'target_yaw={math.degrees(self.lateral_shift_pre_align_target_yaw):+.1f}deg.',
                category='state',
            )

        target_yaw = self.lateral_shift_pre_align_target_yaw
        yaw_error_rad = self._normalize_angle(target_yaw - self.pose_yaw)
        if self.lateral_shift_pre_align_use_visual_feedback:
            aligned = abs(raw_error_rad) <= self.lateral_shift_pre_align_tolerance_rad
            visual_aligned = aligned
            command_error_rad = control_error_rad
        else:
            aligned = abs(yaw_error_rad) <= self.lateral_shift_pre_align_tolerance_rad
            visual_aligned = (
                self.lateral_shift_pre_align_max_visual_error_rad <= 0.0
                or abs(raw_error_rad) <= self.lateral_shift_pre_align_max_visual_error_rad
            )
            command_error_rad = yaw_error_rad

        if aligned and visual_aligned:
            if self.lateral_shift_pre_align_since_sec is None:
                self.lateral_shift_pre_align_since_sec = now_sec
            hold_sec = now_sec - self.lateral_shift_pre_align_since_sec
        else:
            self.lateral_shift_pre_align_since_sec = None
            hold_sec = 0.0

        timed_out = (
            self.lateral_shift_pre_align_timeout_sec > 0.0
            and self.lateral_shift_pre_align_start_sec is not None
            and (now_sec - self.lateral_shift_pre_align_start_sec)
            >= self.lateral_shift_pre_align_timeout_sec
        )

        if timed_out and not (aligned and visual_aligned):
            if self._complete_lateral_pre_align_from_best(now_sec, 'timeout'):
                return False
            self.lateral_shift_pre_align_since_sec = None
            self._handle_blocked(
                now_sec,
                'LATERAL_PRE_ALIGN_TIMEOUT',
                f'pre-align timeout: metric={metric_name}, '
                f'visual_err={math.degrees(raw_error_rad):+.1f}deg, '
                f'odom_yaw_err={math.degrees(yaw_error_rad):+.1f}deg; '
                'not locking a bad forward baseline',
            )
            return True

        if aligned and visual_aligned and hold_sec >= self.lateral_shift_pre_align_hold_sec:
            self.lateral_shift_pre_align_done = True
            self.lateral_shift_pre_align_since_sec = None
            if self.lateral_shift_pre_align_use_visual_feedback:
                target_yaw = self.pose_yaw
                self.lateral_shift_pre_align_target_yaw = target_yaw
                self.lateral_shift_locked_tag = tag
            complete_reason = 'visual_aligned' if self.lateral_shift_pre_align_use_visual_feedback else 'aligned'
            self._publish_feedback(
                'AprilTag lateral pre-align complete: '
                f'reason={complete_reason}, '
                f'metric={self.lateral_shift_pre_align_source_metric}, '
                f'source_tag_x={self.lateral_shift_pre_align_source_x_m:+.3f}m, '
                f'source_tag_z={self.lateral_shift_pre_align_source_z_m:+.3f}m, '
                f'source_measured={math.degrees(self.lateral_shift_pre_align_source_angle_rad):+.1f}deg, '
                f'source_target={math.degrees(self.lateral_shift_pre_align_source_target_rad):+.1f}deg, '
                f'current_measured={math.degrees(measured_rad):+.1f}deg, '
                f'current_raw_err={math.degrees(raw_error_rad):+.1f}deg, '
                f'current_rpy=({math.degrees(tag.roll_rad):+.1f},'
                f'{math.degrees(tag.pitch_rad):+.1f},'
                f'{math.degrees(tag.yaw_rad):+.1f})deg, '
                f'target_yaw={math.degrees(target_yaw):+.1f}deg, '
                f'odom_yaw={math.degrees(self.pose_yaw):+.1f}deg, '
                f'yaw_err={math.degrees(yaw_error_rad):+.1f}deg. '
                'Continuing with locked yaw as the forward baseline.',
                category='state',
            )
            return False

        cmd_vx = 0.0
        cmd_wz = self._compute_wz(command_error_rad)
        step_override = self._step_override_for_state('ALIGNING', cmd_vx, cmd_wz)
        if self.enable_control:
            self._publish_mode(now_sec)
            self._publish_control(cmd_vx, cmd_wz, step_override)
        self._publish_state('LATERAL_PRE_ALIGN_TAG')

        if (now_sec - self.last_log_time) >= self.log_interval_sec:
            self.last_log_time = now_sec
            self._publish_feedback(
                'AprilTag lateral pre-align: '
                f'metric={metric_name}, '
                f'tag_x={tag.x_m:+.3f}m, tag_z={tag.z_m:+.3f}m, '
                f'measured_now={math.degrees(measured_rad):+.1f}deg, '
                f'target={math.degrees(target_rad):+.1f}deg, '
                f'source_measured={math.degrees(self.lateral_shift_pre_align_source_angle_rad):+.1f}deg, '
                f'target_yaw={math.degrees(target_yaw):+.1f}deg, '
                f'odom_yaw={math.degrees(self.pose_yaw):+.1f}deg, '
                f'yaw_err={math.degrees(yaw_error_rad):+.1f}deg, '
                f'visual_err_now={math.degrees(raw_error_rad):+.1f}deg, '
                f'visual_ok={visual_aligned}, '
                f'feedback={"visual" if self.lateral_shift_pre_align_use_visual_feedback else "odom"}, '
                f'rpy=({math.degrees(tag.roll_rad):+.1f},'
                f'{math.degrees(tag.pitch_rad):+.1f},'
                f'{math.degrees(tag.yaw_rad):+.1f})deg, '
                f'hold={hold_sec:.2f}/{self.lateral_shift_pre_align_hold_sec:.2f}s, '
                f'timeout={self._elapsed_text(self.lateral_shift_pre_align_start_sec, now_sec)}'
                f'/{self.lateral_shift_pre_align_timeout_sec:.2f}s, '
                f'cmd_wz={cmd_wz:+.3f}rad/s, '
                f'step_override={self._step_override_text(step_override)}.',
                category='detection',
            )
        return True

    def _start_lateral_shift_sequence(
        self,
        now_sec: float,
        tag: TagPose,
        approach_distance_m: float,
        source: str = 'live_tag',
    ) -> bool:
        if not self.lateral_shift_sequence_enabled:
            return False
        if not self._pose_is_fresh(now_sec):
            self._handle_blocked(
                now_sec,
                'LATERAL_SHIFT_WAITING_ODOM',
                'lateral shift sequence needs fresh Odometry before locking plan',
            )
            return True

        center_x_m = self._lateral_error_x_m(tag)
        center_z_m = approach_distance_m
        abs_center_x_m = abs(center_x_m)
        yaw_abs_rad = self.lateral_shift_yaw_rad
        sin_yaw = max(1e-6, math.sin(yaw_abs_rad))

        if abs_center_x_m <= self.lateral_shift_deadband_m:
            side_delta_rad = 0.0
            raw_shift_distance_m = 0.0
            requested_shift_distance_m = 0.0
            shift_distance_m = 0.0
        else:
            side_delta_rad = (
                self.lateral_shift_yaw_sign
                * math.copysign(yaw_abs_rad, center_x_m)
            )
            raw_shift_distance_m = abs_center_x_m / sin_yaw
            requested_shift_distance_m = (
                raw_shift_distance_m * self.lateral_shift_distance_scale
            )
            if self.lateral_shift_max_distance_m > 0.0:
                shift_distance_m = min(
                    requested_shift_distance_m,
                    self.lateral_shift_max_distance_m,
                )
            else:
                shift_distance_m = requested_shift_distance_m

        base_yaw = self.pose_yaw
        base_yaw_source = 'current_odom'
        if (
            self.lateral_shift_pre_align_enabled
            and self.lateral_shift_pre_align_done
            and self.lateral_shift_pre_align_target_yaw is not None
        ):
            base_yaw = self.lateral_shift_pre_align_target_yaw
            base_yaw_source = 'pre_align_target'
        side_yaw = self._normalize_angle(base_yaw + side_delta_rad)
        forward_component_m = shift_distance_m * math.cos(abs(side_delta_rad))
        final_forward_m = max(
            0.0,
            center_z_m - forward_component_m - self.target_distance_m,
        )

        self.lateral_shift_phase = (
            'align_out'
            if shift_distance_m > self.lateral_shift_deadband_m
            else 'final_forward'
        )
        self.lateral_shift_base_yaw = base_yaw
        self.lateral_shift_side_yaw = side_yaw
        self.lateral_shift_center_x_m = center_x_m
        self.lateral_shift_center_z_m = center_z_m
        self.lateral_shift_distance_m = shift_distance_m
        self.lateral_shift_forward_component_m = forward_component_m
        self.lateral_shift_final_forward_m = final_forward_m
        self.lateral_shift_raw_distance_m = raw_shift_distance_m
        self.lateral_shift_requested_distance_m = requested_shift_distance_m
        self.lateral_shift_duck_align_since_sec = None
        self.lateral_segment_start_x = self.pose_x if self.lateral_shift_phase == 'final_forward' else None
        self.lateral_segment_start_y = self.pose_y if self.lateral_shift_phase == 'final_forward' else None

        clipped_text = (
            ', clipped=true'
            if shift_distance_m + 1e-6 < requested_shift_distance_m
            else ', clipped=false'
        )
        self._publish_feedback(
            'AprilTag lateral shift plan locked: '
            f'source={source}, '
            f'tag_x={tag.x_m:+.3f}m, tag_z={tag.z_m:+.3f}m, '
            f'target_tag_x={self._target_tag_x_text()}, '
            f'offset_x={self.target_offset_x_m:+.3f}m, '
            f'robot_center_offset_x={self.robot_center_offset_x_m:+.3f}m, '
            f'center_x={center_x_m:+.3f}m, center_z={center_z_m:.3f}m, '
            f'base_yaw={math.degrees(base_yaw):+.1f}deg, '
            f'base_yaw_source={base_yaw_source}, '
            f'side_delta={math.degrees(side_delta_rad):+.1f}deg, '
            f'side_yaw={math.degrees(side_yaw):+.1f}deg, '
            f'shift={shift_distance_m:.3f}m(raw={raw_shift_distance_m:.3f}m, '
            f'scale={self.lateral_shift_distance_scale:.2f}, '
            f'requested={requested_shift_distance_m:.3f}m'
            f'{clipped_text}), '
            f'forward_component={forward_component_m:.3f}m, '
            f'final_forward={final_forward_m:.3f}m, '
            f'target_dist={self.target_distance_m:.3f}m.',
            category='state',
        )
        return True

    def _lateral_segment_progress(
        self,
        yaw_rad: float,
    ) -> Tuple[Optional[float], Optional[float]]:
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.lateral_segment_start_x is None
            or self.lateral_segment_start_y is None
        ):
            return None, None
        dx = self.pose_x - self.lateral_segment_start_x
        dy = self.pose_y - self.lateral_segment_start_y
        raw_m = dx * math.cos(yaw_rad) + dy * math.sin(yaw_rad)
        if self.lateral_shift_use_euclidean_progress:
            return math.hypot(dx, dy), raw_m
        return self.locked_reference_along_sign * raw_m, raw_m

    def _run_lateral_shift_sequence(self, now_sec: float) -> None:
        if (
            self.lateral_shift_base_yaw is None
            or self.lateral_shift_side_yaw is None
        ):
            self.lateral_shift_phase = ''
            self._handle_blocked(now_sec, 'LATERAL_SHIFT_INVALID', 'missing lateral plan')
            return
        if not self._pose_is_fresh(now_sec):
            self._handle_blocked(
                now_sec,
                'LATERAL_SHIFT_WAITING_ODOM',
                'lateral shift sequence active but Odometry is stale/missing',
            )
            return

        phase = self.lateral_shift_phase
        target_yaw = self.lateral_shift_base_yaw
        yaw_error = 0.0
        along_m: Optional[float] = None
        raw_along_m: Optional[float] = None
        remaining_m = 0.0
        cmd_vx = 0.0
        cmd_wz = 0.0
        control_state = 'ALIGNED'
        state = 'LATERAL_SHIFT_IDLE'

        if phase == 'align_out':
            target_yaw = self.lateral_shift_side_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            if abs(yaw_error) <= self.yaw_deadband_rad:
                self.lateral_shift_phase = 'shift'
                self.lateral_segment_start_x = self.pose_x
                self.lateral_segment_start_y = self.pose_y
                self._publish_feedback(
                    'AprilTag lateral shift aligned out, starting diagonal walk: '
                    f'target_yaw={math.degrees(target_yaw):+.1f}deg, '
                    f'shift={self.lateral_shift_distance_m:.3f}m.',
                    category='state',
                )
                return
            state = 'LATERAL_ALIGN_OUT'
            control_state = 'ALIGNING'
            cmd_wz = self._compute_wz(yaw_error)

        elif phase == 'shift':
            target_yaw = self.lateral_shift_side_yaw
            along_m, raw_along_m = self._lateral_segment_progress(target_yaw)
            if along_m is None:
                self._handle_blocked(now_sec, 'LATERAL_SHIFT_WAITING_ODOM', 'missing shift start')
                return
            remaining_m = max(0.0, self.lateral_shift_distance_m - along_m)
            if remaining_m <= self.distance_deadband_m:
                self.lateral_shift_phase = 'align_back'
                self._publish_feedback(
                    'AprilTag lateral shift distance complete, aligning back: '
                    f'along={along_m:.3f}m(raw={raw_along_m:.3f}m), '
                    f'target_base_yaw={math.degrees(self.lateral_shift_base_yaw):+.1f}deg.',
                    category='state',
                )
                return
            state = 'LATERAL_SHIFT_WALK'
            control_state = 'APPROACHING'
            cmd_vx = self._compute_vx(remaining_m)
            cmd_wz = self._compute_wz(yaw_error)

        elif phase == 'align_back':
            target_yaw = self.lateral_shift_base_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            if abs(yaw_error) <= self.yaw_deadband_rad:
                self.lateral_shift_phase = 'final_forward'
                self.lateral_segment_start_x = self.pose_x
                self.lateral_segment_start_y = self.pose_y
                self._publish_feedback(
                    'AprilTag lateral shift aligned back, starting final approach: '
                    f'base_yaw={math.degrees(target_yaw):+.1f}deg, '
                    f'final_forward={self.lateral_shift_final_forward_m:.3f}m.',
                    category='state',
                )
                return
            state = 'LATERAL_ALIGN_BACK'
            control_state = 'ALIGNING'
            cmd_wz = self._compute_wz(yaw_error)

        elif phase == 'final_forward':
            target_yaw = self.lateral_shift_base_yaw
            yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
            along_m, raw_along_m = self._lateral_segment_progress(target_yaw)
            if along_m is None:
                self._handle_blocked(now_sec, 'LATERAL_SHIFT_WAITING_ODOM', 'missing final start')
                return
            remaining_m = max(0.0, self.lateral_shift_final_forward_m - along_m)
            forward_yaw_ok = abs(yaw_error) <= self.lateral_shift_final_yaw_gate_rad
            duck_yaw_ok = abs(yaw_error) <= self.lateral_shift_duck_entry_yaw_gate_rad
            if remaining_m <= self.distance_deadband_m:
                if duck_yaw_ok:
                    if self.lateral_shift_duck_align_since_sec is None:
                        self.lateral_shift_duck_align_since_sec = now_sec
                    duck_hold_sec = now_sec - self.lateral_shift_duck_align_since_sec
                    if duck_hold_sec >= self.lateral_shift_duck_entry_hold_sec:
                        self.lateral_shift_phase = ''
                        if self._enter_duck_prepare(now_sec, 'lateral_shift_at_target'):
                            return
                        self.finished = True
                        self._publish_state('LATERAL_AT_TARGET')
                        self._publish_zero_if_enabled()
                        return
                    state = 'LATERAL_DUCK_ALIGN_HOLD'
                    control_state = 'ALIGNING'
                    cmd_wz = 0.0
                else:
                    self.lateral_shift_duck_align_since_sec = None
                    state = 'LATERAL_DUCK_ALIGN'
                    control_state = 'DUCK_ALIGNING'
                    cmd_wz = self._compute_wz_with_deadband(
                        yaw_error,
                        self.lateral_shift_duck_entry_yaw_gate_rad,
                    )
            elif not forward_yaw_ok:
                self.lateral_shift_duck_align_since_sec = None
                state = 'LATERAL_FINAL_ALIGN'
                control_state = 'ALIGNING'
                cmd_wz = self._compute_wz(yaw_error)
            else:
                self.lateral_shift_duck_align_since_sec = None
                state = 'LATERAL_FINAL_APPROACH'
                control_state = 'APPROACHING'
                cmd_vx = self._compute_vx(remaining_m)
                cmd_wz = self._compute_wz(yaw_error)

        else:
            self.lateral_shift_phase = ''
            self._handle_blocked(now_sec, 'LATERAL_SHIFT_INVALID', f'unknown phase={phase}')
            return

        step_override = self._step_override_for_state(
            control_state,
            cmd_vx,
            cmd_wz,
            remaining_m,
        )
        if self.enable_control:
            self._publish_mode(now_sec)
            self._publish_control(cmd_vx, cmd_wz, step_override)
        self._publish_state(state)

        if (now_sec - self.last_log_time) >= self.log_interval_sec:
            self.last_log_time = now_sec
            along_text = 'n/a' if along_m is None else f'{along_m:.3f}m'
            raw_text = 'n/a' if raw_along_m is None else f'{raw_along_m:.3f}m'
            duck_hold_text = (
                'n/a'
                if self.lateral_shift_duck_align_since_sec is None
                else f'{now_sec - self.lateral_shift_duck_align_since_sec:.2f}s'
            )
            self._publish_feedback(
                'AprilTag lateral shift: '
                f'phase={phase}, state={state}, '
                f'center_x={self.lateral_shift_center_x_m:+.3f}m, '
                f'center_z={self.lateral_shift_center_z_m:.3f}m, '
                f'target_yaw={math.degrees(target_yaw):+.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                f'along={along_text}(raw={raw_text}), remaining={remaining_m:.3f}m, '
                f'shift={self.lateral_shift_distance_m:.3f}m'
                f'(raw={self.lateral_shift_raw_distance_m:.3f}m, '
                f'scale={self.lateral_shift_distance_scale:.2f}), '
                f'final_forward={self.lateral_shift_final_forward_m:.3f}m, '
                f'cmd_vx={cmd_vx:+.3f}m/s, cmd_wz={cmd_wz:+.3f}rad/s, '
                f'duck_hold={duck_hold_text}/{self.lateral_shift_duck_entry_hold_sec:.2f}s, '
                f'step_override={self._step_override_text(step_override)}, '
                f'active_output={self._active_output_text(cmd_vx, cmd_wz, step_override)}.',
                category='detection',
            )

    def _run_locked_reference(self, now_sec: float, reason: str) -> bool:
        if not self.lock_reference_on_tag_loss or not self.locked_reference_active:
            return False
        if (
            self.locked_reference_target_yaw is None
            or self.locked_reference_start_x is None
            or self.locked_reference_start_y is None
        ):
            return False
        if not self._pose_is_fresh(now_sec):
            self._handle_blocked(
                now_sec,
                'LOCKED_ODOM_WAITING',
                f'locked reference active but Odometry is stale/missing after {reason}',
            )
            return True

        target_yaw = self.locked_reference_target_yaw
        yaw_error = self._normalize_angle(target_yaw - self.pose_yaw)
        aligned = abs(yaw_error) <= self.yaw_deadband_rad
        dx = self.pose_x - self.locked_reference_start_x
        dy = self.pose_y - self.locked_reference_start_y
        along_raw_m = dx * math.cos(target_yaw) + dy * math.sin(target_yaw)
        along_m = self.locked_reference_along_sign * along_raw_m
        remaining_m = max(0.0, self.locked_reference_distance_m - along_m)
        reached_distance = remaining_m <= self.distance_deadband_m

        state = 'LOCKED_ALIGNED' if aligned else 'LOCKED_ALIGNING'
        control_state = 'ALIGNED' if aligned else 'ALIGNING'
        cmd_vx = 0.0
        cmd_wz = 0.0 if aligned else self._compute_wz(yaw_error)

        if self.enable_forward_approach:
            yaw_ok_for_forward = (
                aligned
                if self.approach_requires_aligned
                else abs(yaw_error) <= self.forward_max_yaw_error_rad
            )
            if reached_distance and aligned:
                state = 'LOCKED_AT_TARGET'
                control_state = 'AT_TARGET'
                cmd_wz = 0.0
            elif reached_distance:
                state = 'LOCKED_ALIGNING'
                control_state = 'ALIGNING'
                cmd_wz = self._compute_wz(yaw_error)
            elif yaw_ok_for_forward:
                state = 'LOCKED_APPROACHING'
                control_state = 'APPROACHING'
                cmd_vx = self._compute_vx(remaining_m)
                cmd_wz = self._compute_wz(yaw_error)
            else:
                state = 'LOCKED_ALIGNING'
                control_state = 'ALIGNING'
                cmd_wz = self._compute_wz(yaw_error)

        if state == 'LOCKED_AT_TARGET' and aligned:
            if self._enter_duck_prepare(now_sec, 'locked_odom_at_target'):
                return

        step_override = self._step_override_for_state(
            control_state,
            cmd_vx,
            cmd_wz,
            remaining_m,
        )
        if self.enable_control:
            self._publish_mode(now_sec)
            self._publish_control(cmd_vx, cmd_wz, step_override)
        self._publish_state(state)

        if (now_sec - self.last_log_time) >= self.log_interval_sec:
            self.last_log_time = now_sec
            self._publish_feedback(
                'AprilTag locked odom: '
                f'reason={reason}, state={state}, '
                f'target_yaw={math.degrees(target_yaw):+.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                f'along={along_m:.3f}m(raw={along_raw_m:.3f}m), '
                f'remaining={remaining_m:.3f}m, '
                f'cmd_vx={cmd_vx:+.3f}m/s, cmd_wz={cmd_wz:+.3f}rad/s, '
                f'step_override={self._step_override_text(step_override)}, '
                f'active_output={self._active_output_text(cmd_vx, cmd_wz, step_override)}, '
                f'source=({self.locked_reference_source})',
                category='detection',
            )
        return True

    def _enter_duck_prepare(self, now_sec: float, reason: str) -> bool:
        if not self.limit_bar_duck_sequence_enabled:
            return False
        self.duck_reference_yaw = self.pose_yaw
        if self.lateral_shift_base_yaw is not None:
            self.duck_reference_yaw = self.lateral_shift_base_yaw
        elif self.locked_reference_target_yaw is not None:
            self.duck_reference_yaw = self.locked_reference_target_yaw
        self.duck_phase = 'prepare'
        self.duck_phase_enter_sec = now_sec
        self.duck_phase_deadline_sec = now_sec + self.duck_prepare_delay_sec
        self._publish_state('DUCK_PREPARE')
        if self.enable_control:
            self._publish_mode(now_sec)
            self._publish_control(
                0.0,
                0.0,
                (
                    self.duck_prepare_mode,
                    self.duck_prepare_left_norm,
                    self.duck_prepare_right_norm,
                ),
            )
        self._publish_feedback(
            f'AprilTag target reached, entering duck prepare: reason={reason}, '
            f'override=[{self.duck_prepare_mode},{self.duck_prepare_left_norm:.3f},'
            f'{self.duck_prepare_right_norm:.3f}], '
            f'duck_ref_yaw={self._angle_text(self.duck_reference_yaw)}, '
            f'wait={self.duck_prepare_delay_sec:.2f}s.',
            category='state',
        )
        return True

    def _duck_walk_override(self) -> Tuple[int, float, float, float, float]:
        yaw_error = 0.0
        delta = 0.0
        left = self.duck_walk_left_norm
        right = self.duck_walk_right_norm
        if (
            self.duck_walk_yaw_correction_enabled
            and self.pose_yaw is not None
            and self.duck_reference_yaw is not None
        ):
            yaw_error = self._normalize_angle(self.duck_reference_yaw - self.pose_yaw)
            if abs(yaw_error) > self.duck_walk_yaw_deadband_rad:
                delta = self.duck_walk_yaw_gain_norm_per_rad * yaw_error
                delta = max(
                    -self.duck_walk_yaw_max_delta_norm,
                    min(self.duck_walk_yaw_max_delta_norm, delta),
                )
                left = self._clamp_norm(self.duck_walk_left_norm - delta)
                right = self._clamp_norm(self.duck_walk_right_norm + delta)
        return self.duck_walk_mode, left, right, yaw_error, delta

    def _angle_text(self, angle_rad: Optional[float]) -> str:
        if angle_rad is None:
            return 'n/a'
        return f'{math.degrees(angle_rad):+.1f}deg'

    def _elapsed_text(self, start_sec: Optional[float], now_sec: float) -> str:
        if start_sec is None:
            return 'n/a'
        return f'{max(0.0, now_sec - start_sec):.2f}'

    def _run_duck_sequence(self, now_sec: float) -> None:
        if self.duck_phase == 'prepare':
            if self.enable_control:
                self._publish_mode(now_sec)
                self._publish_control(
                    0.0,
                    0.0,
                    (
                        self.duck_prepare_mode,
                        self.duck_prepare_left_norm,
                        self.duck_prepare_right_norm,
                    ),
                )
            if self.duck_phase_deadline_sec is not None and now_sec >= self.duck_phase_deadline_sec:
                self.duck_phase = 'walk'
                self.duck_phase_enter_sec = now_sec
                self.duck_phase_deadline_sec = (
                    now_sec + self.duck_walk_timeout_sec
                    if self.duck_walk_timeout_sec > 0.0
                    else None
                )
                self.duck_walk_start_x = self.pose_x
                self.duck_walk_start_y = self.pose_y
                self._publish_state('DUCK_WALK')
                mode, left, right, yaw_error, delta = self._duck_walk_override()
                self._publish_feedback(
                    f'Duck prepare complete, walking through: '
                    f'override=[{mode},{left:.3f},{right:.3f}], '
                    f'distance={self.duck_walk_distance_m:.3f}m, '
                    f'duck_ref_yaw={self._angle_text(self.duck_reference_yaw)}, '
                    f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                    f'yaw_delta={delta:+.3f}.',
                    category='state',
                )
            return

        if self.duck_phase == 'walk':
            walked_m = self._duck_walk_distance()
            mode, left, right, yaw_error, delta = self._duck_walk_override()
            if self.enable_control:
                self._publish_mode(now_sec)
                self._publish_control(
                    0.0,
                    0.0,
                    (
                        mode,
                        left,
                        right,
                    ),
                )
            done_by_distance = (
                walked_m is not None
                and walked_m >= self.duck_walk_distance_m
            )
            done_by_timeout = (
                self.duck_phase_deadline_sec is not None
                and now_sec >= self.duck_phase_deadline_sec
            )
            if done_by_distance or done_by_timeout:
                self.duck_phase = 'resume'
                self.duck_phase_enter_sec = now_sec
                self.duck_phase_deadline_sec = now_sec + self.duck_resume_delay_sec
                self._publish_state('DUCK_RESUME')
                reason = (
                    f'walked={walked_m:.3f}m'
                    if walked_m is not None
                    else f'timeout={self.duck_walk_timeout_sec:.1f}s'
                )
                self._publish_feedback(
                    f'Duck walk complete, resuming: {reason}, '
                    f'override=[{self.duck_resume_mode},{self.duck_resume_left_norm:.3f},'
                    f'{self.duck_resume_right_norm:.3f}], '
                    f'wait={self.duck_resume_delay_sec:.2f}s.',
                    category='state',
                )
                return
            if (now_sec - self.last_log_time) >= self.log_interval_sec:
                self.last_log_time = now_sec
                walked_text = 'unknown' if walked_m is None else f'{walked_m:.3f}m'
                self._publish_feedback(
                    f'Duck walking: walked={walked_text}/{self.duck_walk_distance_m:.3f}m, '
                    f'duck_ref_yaw={self._angle_text(self.duck_reference_yaw)}, '
                    f'odom_yaw={self._angle_text(self.pose_yaw)}, '
                    f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                    f'yaw_delta={delta:+.3f}, '
                    f'override=[{mode},{left:.3f},{right:.3f}].',
                    category='detection',
                )
            return

        if self.duck_phase == 'resume':
            if self.enable_control:
                self._publish_mode(now_sec)
                self._publish_control(
                    0.0,
                    0.0,
                    (
                        self.duck_resume_mode,
                        self.duck_resume_left_norm,
                        self.duck_resume_right_norm,
                    ),
                )
            if self.duck_phase_deadline_sec is not None and now_sec >= self.duck_phase_deadline_sec:
                self.duck_phase = ''
                self.finished = True
                self._publish_state('FINISHED')
                self._publish_feedback('Duck sequence finished.', category='state')

    def _duck_walk_distance(self) -> Optional[float]:
        if (
            self.pose_x is None
            or self.pose_y is None
            or self.duck_walk_start_x is None
            or self.duck_walk_start_y is None
        ):
            return None
        return math.hypot(
            self.pose_x - self.duck_walk_start_x,
            self.pose_y - self.duck_walk_start_y,
        )

    def _handle_blocked(self, now_sec: float, state: str, reason: str) -> None:
        self.aligned_since_sec = None
        self.bridge_lateral_aligned_since_sec = None
        self._publish_state(state)
        self._publish_zero_if_enabled(now_sec)
        if (now_sec - self.last_warn_time) >= self.warn_interval_sec:
            self.last_warn_time = now_sec
            self._publish_feedback(
                f'AprilTag align blocked: {reason}, '
                f'expected_tf={self.target_frame_id}->{self.target_child_frame_id}, '
                f'latest_tf={self._latest_tf_text()}, det={self._detection_text(now_sec)}, '
                f'control={self.enable_control}',
                level='warn',
                category='warn',
            )

    def _publish_zero_if_enabled(self, now_sec: Optional[float] = None) -> None:
        if self.enable_control and self.publish_zero_when_idle:
            self._publish_mode(now_sec)
            self._publish_control(0.0, 0.0, None)

    def _publish_control(
        self,
        vx: float,
        wz: float,
        step_override: Optional[Tuple[int, float, float]],
    ) -> None:
        if self.control_output_mode == 'step_override':
            self._publish_twist(0.0, 0.0)
            if step_override is None:
                self._clear_step_override()
            else:
                self._publish_step_override(*step_override)
            return

        if step_override is not None:
            self._publish_twist(0.0, 0.0)
            self._publish_step_override(*step_override)
            return

        if self.step_override_active:
            self._clear_step_override()
        self._publish_twist(vx, wz)

    def _publish_twist(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.cmd_vel_pub.publish(msg)

    def _publish_step_override(self, mode: int, left_norm: float, right_norm: float) -> None:
        msg = Float32MultiArray()
        msg.data = [
            float(mode),
            self._clamp_norm(left_norm),
            self._clamp_norm(right_norm),
        ]
        self.step_override_pub.publish(msg)
        self.step_override_active = True

    def _clear_step_override(self) -> None:
        msg = Float32MultiArray()
        msg.data = [float(self.step_override_clear_mode), 0.0, 0.0]
        self.step_override_pub.publish(msg)
        self.step_override_active = False

    def _publish_mode(self, now_sec: Optional[float] = None) -> None:
        if not self.publish_mode_when_control_enabled:
            return
        now = self._now_sec() if now_sec is None else now_sec
        mode_value = int(self.control_mode)
        if (
            self.last_mode_value == mode_value
            and self.mode_publish_interval_sec > 0.0
            and (now - self.last_mode_publish_time) < self.mode_publish_interval_sec
        ):
            return
        msg = Int32()
        msg.data = mode_value
        self.mode_pub.publish(msg)
        self.last_mode_value = mode_value
        self.last_mode_publish_time = now

    def _publish_state(self, state: str) -> None:
        if state == self.last_state:
            return
        self.last_state = state
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

    def _publish_debug(
        self,
        tag: TagPose,
        measured_rad: float,
        target_rad: float,
        raw_error_rad: float,
        control_error_rad: float,
        cmd_vx: float,
        cmd_wz: float,
        distance_m: float,
        approach_distance_m: float,
        distance_error_m: float,
        step_override: Optional[Tuple[int, float, float]],
    ) -> None:
        msg = Float32MultiArray()
        msg.data = [
            float(measured_rad),
            float(target_rad),
            float(raw_error_rad),
            float(control_error_rad),
            float(cmd_vx),
            float(cmd_wz),
            float(tag.x_m),
            float(tag.y_m),
            float(tag.z_m),
            float(distance_m),
            float(approach_distance_m),
            float(distance_error_m),
            float(tag.roll_rad),
            float(tag.pitch_rad),
            float(tag.yaw_rad),
            float(step_override[0]) if step_override is not None else float(self.step_override_clear_mode),
            float(step_override[1]) if step_override is not None else 0.0,
            float(step_override[2]) if step_override is not None else 0.0,
        ]
        self.debug_pub.publish(msg)

    def _publish_feedback(
        self,
        text: str,
        *,
        level: str = 'info',
        category: Optional[str] = None,
    ) -> None:
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)
        logger = self.get_logger()
        colored = colorize_log(text, level=level, category=category)
        if level == 'warn':
            logger.warn(colored)
        elif level == 'error':
            logger.error(colored)
        else:
            logger.info(colored)

    def _detection_text(self, now_sec: float) -> str:
        info = self.latest_detection
        if info.received_sec <= 0.0:
            return 'none'
        age_sec = now_sec - info.stamp_sec
        if not info.matched:
            return f'target_id={self.target_tag_id},match=false,raw={info.raw_count},age={age_sec:.2f}s'
        return (
            f'{info.selected_family}:{info.selected_id},match=true,raw={info.raw_count},'
            f'margin={info.decision_margin:.1f},'
            f'hamming={info.hamming},center=({info.centre_x:.0f},{info.centre_y:.0f}),'
            f'age={age_sec:.2f}s'
        )

    def _decoded_tags_text(self) -> str:
        info = self.latest_detection
        if info.received_sec <= 0.0:
            return 'none'
        return info.decoded_tags if info.decoded_tags else 'empty'

    def _step_override_text(self, step_override: Optional[Tuple[int, float, float]]) -> str:
        if step_override is None:
            return 'clear'
        return f'[{step_override[0]},{step_override[1]:+.3f},{step_override[2]:+.3f}]'

    def _active_output_text(
        self,
        vx: float,
        wz: float,
        step_override: Optional[Tuple[int, float, float]],
    ) -> str:
        if not self.enable_control:
            return 'disabled'
        if step_override is not None:
            return 'step_override'
        if abs(vx) > 1e-6 or abs(wz) > 1e-6:
            return 'cmd_vel'
        return 'idle'

    def _latest_tf_text(self) -> str:
        if self.latest_tag is None:
            return 'none'
        return (
            f'{self.latest_tag.frame_id}->{self.latest_tag.child_frame_id}, '
            f'xyz=({self.latest_tag.x_m:.3f},{self.latest_tag.y_m:.3f},'
            f'{self.latest_tag.z_m:.3f})'
        )

    def _locked_reference_text(self, now_sec: float) -> str:
        if not self.lock_reference_on_tag_loss:
            return 'disabled'
        if not self.locked_reference_active:
            return 'none'
        age_sec = now_sec - self.locked_reference_stamp_sec
        yaw_text = 'none'
        if self.locked_reference_target_yaw is not None:
            yaw_text = f'{math.degrees(self.locked_reference_target_yaw):+.1f}deg'
        return (
            f'active,age={age_sec:.2f}s,target_yaw={yaw_text},'
            f'remaining0={self.locked_reference_distance_m:.3f}m'
        )

    def _target_tag_x_text(self) -> str:
        if not self._uses_target_tag_x():
            return 'unset'
        return f'{self.target_tag_x_m:+.3f}m'

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_to_sec(stamp: object) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _sign(value: float) -> float:
        return 1.0 if value >= 0.0 else -1.0

    @staticmethod
    def _clamp_norm(value: float) -> float:
        return min(1.0, max(-1.0, float(value)))

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    @staticmethod
    def _yaw_from_quaternion(q: Quaternion) -> float:
        return AprilTagAlignTest._rpy_from_quaternion(q)[2]

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AprilTagAlignTest()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
