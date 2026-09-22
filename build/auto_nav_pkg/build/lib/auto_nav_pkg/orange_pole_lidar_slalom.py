#!/usr/bin/env python3
"""LiDAR-gated continuous slalom controller for the right-angle pole array."""

import math
from collections import deque
from typing import Deque, Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from auto_nav_pkg.lidar_slalom_core import (
    DEFAULT_TEMPLATE_POINTS_RL,
    SlalomFrame,
    build_slalom_path,
    detect_entry_p4,
    detect_expected_pole,
    differential_tracking_command,
    final_turn_override,
    flat_pairs,
    normalize_angle,
    oriented_rectangle_pole_clearance,
    path_rectangle_pole_clearance,
    path_tracking_target,
    quaternion_rotation_matrix,
    relative_turn_target_yaw,
    replace_path_tail_with_pose_curve,
)
from auto_nav_pkg.orange_pole_relative_nav_stable import OrangePoleRelativeNavStable


class OrangePoleLidarSlalom(OrangePoleRelativeNavStable):
    """Build a run-local pole frame at P4 and follow a continuous LiDAR route."""

    def __init__(self) -> None:
        # Base construction dispatches to several virtual methods. Seed every
        # field used by those overrides before calling the parent constructor.
        self.lidar_slalom_enabled = False
        self.lidar_slalom_active = False
        self.lidar_slalom_phase = ''
        self.lidar_slalom_motion_started = False
        self.lidar_cloud_window_frames = 7
        self.lidar_cloud_frames: Deque[tuple[float, np.ndarray]] = deque(maxlen=7)
        self.lidar_cloud_sequence = 0
        self.lidar_last_cloud_receive_sec = 0.0
        self.lidar_have_full_odom = False
        self.lidar_pose_xyz = np.zeros(3, dtype=float)
        self.lidar_pose_rotation = np.eye(3, dtype=float)
        self.lidar_pose_stamp_sec = 0.0
        self.lidar_entry_pose_xyz: Optional[np.ndarray] = None
        self.lidar_entry_forward_xy: Optional[np.ndarray] = None
        self.lidar_lock_candidate_xy: Optional[np.ndarray] = None
        self.lidar_lock_candidate_count = 0
        self.lidar_lock_last_cloud_sequence = -1
        self.lidar_lock_start_sec = 0.0
        self.lidar_run_start_sec = 0.0
        self.lidar_route_start_xy: Optional[np.ndarray] = None
        self.lidar_frame: Optional[SlalomFrame] = None
        self.lidar_path_xy = np.empty((0, 2), dtype=float)
        self.lidar_path_index = 0
        self.lidar_final_curve_start_index = -1
        self.lidar_zone_indices: list[int] = []
        self.lidar_zone_cursor = 0
        self.lidar_locked_poles: dict[int, np.ndarray] = {}
        self.lidar_track_candidate_xy: Optional[np.ndarray] = None
        self.lidar_track_candidate_count = 0
        self.lidar_track_last_cloud_sequence = -1
        self.lidar_last_progress_log_sec = 0.0
        self.lidar_last_runtime_guard_warning_sec = 0.0
        self.lidar_final_turn_start_yaw = 0.0
        self.lidar_final_turn_last_yaw = 0.0
        self.lidar_final_turn_progress_rad = 0.0
        self.lidar_final_turn_target_yaw = 0.0
        self.lidar_final_turn_start_sec = 0.0
        self.lidar_final_turn_stable_since: Optional[float] = None

        super().__init__()

        self.declare_parameter('lidar_slalom_enabled', False)
        self.declare_parameter('lidar_slalom_topic', '/livox/lidar')
        self.declare_parameter('lidar_slalom_cloud_window_frames', 7)
        self.declare_parameter('lidar_slalom_max_planar_range_m', 3.2)
        self.declare_parameter(
            'lidar_slalom_lidar_to_body_translation_m',
            [-0.011, -0.02329, 0.04412],
        )
        self.declare_parameter('lidar_slalom_lock_timeout_sec', 1.50)
        self.declare_parameter('lidar_slalom_lock_samples', 3)
        self.declare_parameter('lidar_slalom_lock_consistency_m', 0.06)
        self.declare_parameter('lidar_slalom_fallback_to_legacy_before_motion', True)
        self.declare_parameter('lidar_slalom_pole_spacing_m', 1.0)
        self.declare_parameter('lidar_slalom_entry_forward_min_m', 0.45)
        self.declare_parameter('lidar_slalom_entry_forward_max_m', 1.30)
        self.declare_parameter('lidar_slalom_entry_lateral_max_m', 0.35)
        self.declare_parameter('lidar_slalom_vertical_z_min_m', -0.42)
        self.declare_parameter('lidar_slalom_vertical_z_max_m', 0.42)
        self.declare_parameter('lidar_slalom_vertical_cell_size_m', 0.04)
        self.declare_parameter('lidar_slalom_vertical_min_points', 5)
        self.declare_parameter('lidar_slalom_vertical_min_z_span_m', 0.18)
        self.declare_parameter('lidar_slalom_expected_search_radius_m', 0.20)
        self.declare_parameter('lidar_slalom_pose_cloud_max_skew_sec', 0.12)
        self.declare_parameter('lidar_slalom_cloud_stale_timeout_sec', 0.40)
        self.declare_parameter('lidar_slalom_runtime_cloud_stale_stop_enabled', True)
        self.declare_parameter('lidar_slalom_sample_spacing_m', 0.03)
        self.declare_parameter('lidar_slalom_lookahead_m', 0.20)
        self.declare_parameter('lidar_slalom_mode', 0)
        self.declare_parameter('lidar_slalom_cruise_norm', 0.42)
        self.declare_parameter('lidar_slalom_minimum_norm', 0.18)
        self.declare_parameter('lidar_slalom_heading_gain_norm_per_rad', 0.70)
        self.declare_parameter('lidar_slalom_max_delta_norm', 0.28)
        self.declare_parameter('lidar_slalom_slowdown_heading_deg', 35.0)
        self.declare_parameter('lidar_slalom_robot_length_m', 0.66)
        self.declare_parameter('lidar_slalom_robot_width_m', 0.46)
        self.declare_parameter('lidar_slalom_pole_radius_m', 0.025)
        self.declare_parameter('lidar_slalom_runtime_geometry_stop_enabled', True)
        self.declare_parameter('lidar_slalom_min_body_clearance_m', 0.04)
        self.declare_parameter('lidar_slalom_min_planned_body_clearance_m', 0.08)
        self.declare_parameter('lidar_slalom_max_cross_track_m', 0.28)
        self.declare_parameter('lidar_slalom_mandatory_zone_tolerance_m', 0.14)
        self.declare_parameter('lidar_slalom_finish_distance_m', 0.16)
        self.declare_parameter('lidar_slalom_route_timeout_sec', 50.0)
        self.declare_parameter('lidar_slalom_final_absolute_yaw_enabled', False)
        self.declare_parameter('lidar_slalom_final_absolute_yaw_deg', 0.0)
        self.declare_parameter(
            'lidar_slalom_final_absolute_yaw_tolerance_deg', 8.0
        )
        self.declare_parameter(
            'lidar_slalom_final_curve_anchor_rl', [2.10, -0.45]
        )
        self.declare_parameter('lidar_slalom_final_curve_start_handle_m', 0.60)
        self.declare_parameter('lidar_slalom_final_curve_end_handle_m', 0.10)
        self.declare_parameter('lidar_slalom_final_curve_sample_spacing_m', 0.005)
        self.declare_parameter('lidar_slalom_final_curve_min_lookahead_m', 0.04)
        self.declare_parameter('lidar_slalom_final_curve_minimum_norm', 0.10)
        self.declare_parameter('lidar_slalom_final_curve_max_delta_norm', 0.38)
        self.declare_parameter('lidar_slalom_final_curve_yaw_blend_start', 0.80)
        self.declare_parameter('lidar_slalom_final_turn_enabled', True)
        self.declare_parameter('lidar_slalom_final_turn_delta_deg', 90.0)
        self.declare_parameter('lidar_slalom_final_turn_mode', 6)
        self.declare_parameter('lidar_slalom_final_turn_ccw_left_norm', -0.20)
        self.declare_parameter('lidar_slalom_final_turn_ccw_right_norm', 0.65)
        self.declare_parameter('lidar_slalom_final_turn_cw_left_norm', 0.65)
        self.declare_parameter('lidar_slalom_final_turn_cw_right_norm', -0.20)
        self.declare_parameter('lidar_slalom_final_turn_fine_error_deg', 12.0)
        self.declare_parameter('lidar_slalom_final_turn_fine_left_norm', -0.10)
        self.declare_parameter('lidar_slalom_final_turn_fine_right_norm', 0.35)
        self.declare_parameter('lidar_slalom_final_turn_tolerance_deg', 3.0)
        self.declare_parameter('lidar_slalom_final_turn_finish_immediately', True)
        self.declare_parameter('lidar_slalom_final_turn_max_yaw_rate_deg_s', 6.0)
        self.declare_parameter('lidar_slalom_final_turn_hold_sec', 0.20)
        self.declare_parameter('lidar_slalom_final_turn_timeout_sec', 12.0)
        self.declare_parameter('lidar_slalom_progress_log_interval_sec', 1.0)
        self.declare_parameter(
            'lidar_slalom_template_points_rl',
            [value for pair in DEFAULT_TEMPLATE_POINTS_RL for value in pair],
        )

        self.lidar_slalom_enabled = bool(
            self.get_parameter('lidar_slalom_enabled').value
        )
        self.lidar_slalom_topic = str(
            self.get_parameter('lidar_slalom_topic').value
        )
        self.lidar_cloud_window_frames = max(
            3, int(self.get_parameter('lidar_slalom_cloud_window_frames').value)
        )
        self.lidar_max_planar_range_m = max(
            1.0,
            float(self.get_parameter('lidar_slalom_max_planar_range_m').value),
        )
        translation = list(
            self.get_parameter('lidar_slalom_lidar_to_body_translation_m').value
        )
        if len(translation) != 3:
            raise ValueError('lidar_slalom_lidar_to_body_translation_m must have 3 values')
        self.lidar_to_body_translation = np.asarray(translation, dtype=float)
        self.lidar_lock_timeout_sec = max(
            0.2, float(self.get_parameter('lidar_slalom_lock_timeout_sec').value)
        )
        self.lidar_lock_samples = max(
            1, int(self.get_parameter('lidar_slalom_lock_samples').value)
        )
        self.lidar_lock_consistency_m = max(
            0.02,
            float(self.get_parameter('lidar_slalom_lock_consistency_m').value),
        )
        self.lidar_fallback_to_legacy_before_motion = bool(
            self.get_parameter('lidar_slalom_fallback_to_legacy_before_motion').value
        )
        self.lidar_pole_spacing_m = max(
            0.80, float(self.get_parameter('lidar_slalom_pole_spacing_m').value)
        )
        self.lidar_entry_forward_min_m = max(
            0.20,
            float(self.get_parameter('lidar_slalom_entry_forward_min_m').value),
        )
        self.lidar_entry_forward_max_m = max(
            self.lidar_entry_forward_min_m + 0.10,
            float(self.get_parameter('lidar_slalom_entry_forward_max_m').value),
        )
        self.lidar_entry_lateral_max_m = max(
            0.10,
            float(self.get_parameter('lidar_slalom_entry_lateral_max_m').value),
        )
        self.lidar_vertical_z_min_m = float(
            self.get_parameter('lidar_slalom_vertical_z_min_m').value
        )
        self.lidar_vertical_z_max_m = float(
            self.get_parameter('lidar_slalom_vertical_z_max_m').value
        )
        self.lidar_vertical_cell_size_m = max(
            0.02,
            float(self.get_parameter('lidar_slalom_vertical_cell_size_m').value),
        )
        self.lidar_vertical_min_points = max(
            3, int(self.get_parameter('lidar_slalom_vertical_min_points').value)
        )
        self.lidar_vertical_min_z_span_m = max(
            0.10,
            float(self.get_parameter('lidar_slalom_vertical_min_z_span_m').value),
        )
        self.lidar_expected_search_radius_m = max(
            0.10,
            float(self.get_parameter('lidar_slalom_expected_search_radius_m').value),
        )
        self.lidar_pose_cloud_max_skew_sec = max(
            0.02,
            float(self.get_parameter('lidar_slalom_pose_cloud_max_skew_sec').value),
        )
        self.lidar_cloud_stale_timeout_sec = max(
            0.10,
            float(self.get_parameter('lidar_slalom_cloud_stale_timeout_sec').value),
        )
        self.lidar_runtime_cloud_stale_stop_enabled = bool(
            self.get_parameter(
                'lidar_slalom_runtime_cloud_stale_stop_enabled'
            ).value
        )
        self.lidar_sample_spacing_m = max(
            0.01, float(self.get_parameter('lidar_slalom_sample_spacing_m').value)
        )
        self.lidar_lookahead_m = max(
            0.08, float(self.get_parameter('lidar_slalom_lookahead_m').value)
        )
        self.lidar_slalom_mode = int(self.get_parameter('lidar_slalom_mode').value)
        self.lidar_cruise_norm = max(
            0.0, min(1.0, float(self.get_parameter('lidar_slalom_cruise_norm').value))
        )
        self.lidar_minimum_norm = max(
            0.0,
            min(
                self.lidar_cruise_norm,
                float(self.get_parameter('lidar_slalom_minimum_norm').value),
            ),
        )
        self.lidar_heading_gain_norm_per_rad = max(
            0.0,
            float(
                self.get_parameter('lidar_slalom_heading_gain_norm_per_rad').value
            ),
        )
        self.lidar_max_delta_norm = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter('lidar_slalom_max_delta_norm').value),
            ),
        )
        self.lidar_slowdown_heading_rad = math.radians(
            max(
                5.0,
                float(self.get_parameter('lidar_slalom_slowdown_heading_deg').value),
            )
        )
        self.lidar_robot_length_m = max(
            0.10, float(self.get_parameter('lidar_slalom_robot_length_m').value)
        )
        self.lidar_robot_width_m = max(
            0.10, float(self.get_parameter('lidar_slalom_robot_width_m').value)
        )
        self.lidar_pole_radius_m = max(
            0.0, float(self.get_parameter('lidar_slalom_pole_radius_m').value)
        )
        self.lidar_runtime_geometry_stop_enabled = bool(
            self.get_parameter(
                'lidar_slalom_runtime_geometry_stop_enabled'
            ).value
        )
        self.lidar_min_body_clearance_m = max(
            0.0,
            float(self.get_parameter('lidar_slalom_min_body_clearance_m').value),
        )
        self.lidar_min_planned_body_clearance_m = max(
            self.lidar_min_body_clearance_m,
            float(
                self.get_parameter(
                    'lidar_slalom_min_planned_body_clearance_m'
                ).value
            ),
        )
        self.lidar_max_cross_track_m = max(
            0.10,
            float(self.get_parameter('lidar_slalom_max_cross_track_m').value),
        )
        self.lidar_mandatory_zone_tolerance_m = max(
            0.05,
            float(
                self.get_parameter('lidar_slalom_mandatory_zone_tolerance_m').value
            ),
        )
        self.lidar_finish_distance_m = max(
            0.05,
            float(self.get_parameter('lidar_slalom_finish_distance_m').value),
        )
        self.lidar_route_timeout_sec = max(
            5.0, float(self.get_parameter('lidar_slalom_route_timeout_sec').value)
        )
        self.lidar_final_absolute_yaw_enabled = bool(
            self.get_parameter(
                'lidar_slalom_final_absolute_yaw_enabled'
            ).value
        )
        self.lidar_final_absolute_yaw_rad = math.radians(
            float(
                self.get_parameter('lidar_slalom_final_absolute_yaw_deg').value
            )
        )
        self.lidar_final_absolute_yaw_tolerance_rad = math.radians(
            max(
                1.0,
                float(
                    self.get_parameter(
                        'lidar_slalom_final_absolute_yaw_tolerance_deg'
                    ).value
                ),
            )
        )
        final_curve_anchor = list(
            self.get_parameter('lidar_slalom_final_curve_anchor_rl').value
        )
        if len(final_curve_anchor) != 2:
            raise ValueError('lidar_slalom_final_curve_anchor_rl must have 2 values')
        self.lidar_final_curve_anchor_rl = np.asarray(
            final_curve_anchor, dtype=float
        )
        self.lidar_final_curve_start_handle_m = max(
            0.05,
            float(
                self.get_parameter(
                    'lidar_slalom_final_curve_start_handle_m'
                ).value
            ),
        )
        self.lidar_final_curve_end_handle_m = max(
            0.05,
            float(
                self.get_parameter('lidar_slalom_final_curve_end_handle_m').value
            ),
        )
        self.lidar_final_curve_sample_spacing_m = max(
            0.005,
            float(
                self.get_parameter(
                    'lidar_slalom_final_curve_sample_spacing_m'
                ).value
            ),
        )
        self.lidar_final_curve_min_lookahead_m = max(
            0.02,
            float(
                self.get_parameter(
                    'lidar_slalom_final_curve_min_lookahead_m'
                ).value
            ),
        )
        self.lidar_final_curve_minimum_norm = max(
            0.0,
            min(
                self.lidar_cruise_norm,
                float(
                    self.get_parameter(
                        'lidar_slalom_final_curve_minimum_norm'
                    ).value
                ),
            ),
        )
        self.lidar_final_curve_max_delta_norm = max(
            0.0,
            min(
                1.0,
                float(
                    self.get_parameter(
                        'lidar_slalom_final_curve_max_delta_norm'
                    ).value
                ),
            ),
        )
        self.lidar_final_curve_yaw_blend_start = max(
            0.0,
            min(
                0.95,
                float(
                    self.get_parameter(
                        'lidar_slalom_final_curve_yaw_blend_start'
                    ).value
                ),
            ),
        )
        self.lidar_final_turn_enabled = bool(
            self.get_parameter('lidar_slalom_final_turn_enabled').value
        )
        self.lidar_final_turn_delta_rad = math.radians(
            float(self.get_parameter('lidar_slalom_final_turn_delta_deg').value)
        )
        self.lidar_final_turn_mode = int(
            self.get_parameter('lidar_slalom_final_turn_mode').value
        )
        self.lidar_final_turn_ccw_left_norm = float(
            self.get_parameter('lidar_slalom_final_turn_ccw_left_norm').value
        )
        self.lidar_final_turn_ccw_right_norm = float(
            self.get_parameter('lidar_slalom_final_turn_ccw_right_norm').value
        )
        self.lidar_final_turn_cw_left_norm = float(
            self.get_parameter('lidar_slalom_final_turn_cw_left_norm').value
        )
        self.lidar_final_turn_cw_right_norm = float(
            self.get_parameter('lidar_slalom_final_turn_cw_right_norm').value
        )
        self.lidar_final_turn_fine_error_rad = math.radians(
            max(
                1.0,
                float(
                    self.get_parameter(
                        'lidar_slalom_final_turn_fine_error_deg'
                    ).value
                ),
            )
        )
        self.lidar_final_turn_fine_left_norm = float(
            self.get_parameter('lidar_slalom_final_turn_fine_left_norm').value
        )
        self.lidar_final_turn_fine_right_norm = float(
            self.get_parameter('lidar_slalom_final_turn_fine_right_norm').value
        )
        self.lidar_final_turn_tolerance_rad = math.radians(
            max(
                0.5,
                float(
                    self.get_parameter(
                        'lidar_slalom_final_turn_tolerance_deg'
                    ).value
                ),
            )
        )
        self.lidar_final_turn_finish_immediately = bool(
            self.get_parameter(
                'lidar_slalom_final_turn_finish_immediately'
            ).value
        )
        self.lidar_final_turn_max_yaw_rate_rad_s = math.radians(
            max(
                0.0,
                float(
                    self.get_parameter(
                        'lidar_slalom_final_turn_max_yaw_rate_deg_s'
                    ).value
                ),
            )
        )
        self.lidar_final_turn_hold_sec = max(
            0.0,
            float(self.get_parameter('lidar_slalom_final_turn_hold_sec').value),
        )
        self.lidar_final_turn_timeout_sec = max(
            2.0,
            float(self.get_parameter('lidar_slalom_final_turn_timeout_sec').value),
        )
        self.lidar_progress_log_interval_sec = max(
            0.2,
            float(
                self.get_parameter('lidar_slalom_progress_log_interval_sec').value
            ),
        )
        self.lidar_template_points_rl = flat_pairs(
            self.get_parameter('lidar_slalom_template_points_rl').value
        )

        self.lidar_cloud_frames = deque(maxlen=self.lidar_cloud_window_frames)
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.lidar_slalom_topic,
            self._lidar_callback,
            qos_profile_sensor_data,
        )
        if self.lidar_slalom_enabled:
            self._publish_feedback(
                'LiDAR 连续绕杆已启用: '
                f'topic={self.lidar_slalom_topic}, window={self.lidar_cloud_window_frames}, '
                f'P4_lock={self.lidar_lock_samples} samples/{self.lidar_lock_timeout_sec:.2f}s, '
                f'cruise={self.lidar_cruise_norm:.2f}, lookahead={self.lidar_lookahead_m:.2f}m, '
                f'footprint={self.lidar_robot_length_m:.2f}x{self.lidar_robot_width_m:.2f}m, '
                'runtime_geometry_stop='
                f'{"on" if self.lidar_runtime_geometry_stop_enabled else "off"}, '
                'runtime_cloud_stale_stop='
                f'{"on" if self.lidar_runtime_cloud_stale_stop_enabled else "off"}, '
                f'body_clearance_limit={self.lidar_min_body_clearance_m:.2f}m, '
                'final_exit=' + (
                    f'curve_to_abs_{math.degrees(self.lidar_final_absolute_yaw_rad):+.0f}deg'
                    if self.lidar_final_absolute_yaw_enabled
                    else f'turn_{math.degrees(self.lidar_final_turn_delta_rad):+.0f}deg'
                )
            )

    def _odometry_callback(self, msg: Odometry) -> None:
        super()._odometry_callback(msg)
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.lidar_pose_xyz = np.array(
            (position.x, position.y, position.z), dtype=float
        )
        self.lidar_pose_rotation = quaternion_rotation_matrix(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        self.lidar_pose_stamp_sec = self._message_stamp_sec(msg)
        self.lidar_have_full_odom = True

    def _lidar_callback(self, msg: PointCloud2) -> None:
        if not self.lidar_slalom_enabled or not self.route_controller_active:
            return
        if not self.lidar_have_full_odom or self.pose_topic_type != 'odometry':
            return
        cloud_stamp = self._message_stamp_sec(msg)
        if (
            cloud_stamp > 0.0
            and self.lidar_pose_stamp_sec > 0.0
            and abs(cloud_stamp - self.lidar_pose_stamp_sec)
            > self.lidar_pose_cloud_max_skew_sec
        ):
            return
        try:
            structured = point_cloud2.read_points(
                msg,
                field_names=('x', 'y', 'z'),
                skip_nans=True,
            )
            xyz = np.column_stack(
                (structured['x'], structured['y'], structured['z'])
            ).astype(np.float64, copy=False)
        except (KeyError, ValueError, TypeError) as error:
            self.get_logger().warning(f'LiDAR 点云解析失败: {error}')
            return
        planar_range = np.linalg.norm(xyz[:, :2], axis=1)
        valid = (
            np.isfinite(xyz).all(axis=1)
            & (planar_range >= 0.15)
            & (planar_range <= self.lidar_max_planar_range_m)
        )
        body_points = xyz[valid] + self.lidar_to_body_translation[None, :]
        world_points = (
            body_points @ self.lidar_pose_rotation.T
            + self.lidar_pose_xyz[None, :]
        )
        broad_z_mask = (
            (world_points[:, 2] - self.lidar_pose_xyz[2])
            >= self.lidar_vertical_z_min_m - 0.10
        ) & (
            (world_points[:, 2] - self.lidar_pose_xyz[2])
            <= self.lidar_vertical_z_max_m + 0.10
        )
        receive_sec = self._now_sec()
        self.lidar_cloud_frames.append(
            (cloud_stamp if cloud_stamp > 0.0 else receive_sec, world_points[broad_z_mask])
        )
        self.lidar_cloud_sequence += 1
        self.lidar_last_cloud_receive_sec = receive_sec

    def _start_route(self, reason: str, source: str = 'hsv') -> None:
        if not self.lidar_slalom_enabled or source != 'apriltag':
            super()._start_route(reason, source=source)
            return
        if not self.route_controller_active:
            return
        if self.triggered or self.route_finished or self.route_failed:
            return

        self.route_start_pending = False
        self.pending_route_reason = ''
        self.pending_route_source = ''
        self.route_source = 'apriltag'
        self.triggered = True
        self.active_goal = False
        self.awaiting_next_goal = False
        self.lidar_slalom_active = True
        self.lidar_slalom_motion_started = False
        self.lidar_lock_start_sec = self._now_sec()
        self.lidar_lock_candidate_xy = None
        self.lidar_lock_candidate_count = 0
        self.lidar_lock_last_cloud_sequence = -1
        self.lidar_track_last_cloud_sequence = -1
        self._release_startup_route_handoff('lidar_slalom_start', log_release=False)
        self._set_route_mode_active(True)
        self._publish_step_override(self.lidar_slalom_mode, 0.0, 0.0)
        if self._capture_lidar_entry_pose():
            self.lidar_slalom_phase = 'LOCK_P4'
            self._publish_state('LIDAR_SLALOM_LOCK_P4')
        else:
            self.lidar_slalom_phase = 'WAIT_ODOMETRY'
            self._publish_state('LIDAR_SLALOM_WAIT_ODOMETRY')
        self._publish_feedback(
            f'AprilTag 触发 LiDAR 连续绕杆: reason={reason}; '
            '不使用固定 camera_init 坐标，停车锁定当前 P4。'
        )

    def _monitor_loop(self) -> None:
        if not self.route_controller_active:
            return
        if self.lidar_slalom_active:
            self._monitor_lidar_slalom()
            return
        super()._monitor_loop()

    def _monitor_lidar_slalom(self) -> None:
        self._publish_status_heartbeat()
        if self.route_finished or self.route_failed:
            return
        self._refresh_route_mode()
        now = self._now_sec()

        if self.lidar_slalom_phase == 'WAIT_ODOMETRY':
            self._publish_step_override(self.lidar_slalom_mode, 0.0, 0.0)
            if self._capture_lidar_entry_pose():
                self.lidar_slalom_phase = 'LOCK_P4'
                self.lidar_lock_start_sec = now
                self._publish_state('LIDAR_SLALOM_LOCK_P4')
                return
            if (now - self.lidar_lock_start_sec) > self.lidar_lock_timeout_sec:
                self._fallback_or_fail_lidar('no_fresh_odometry_before_p4_lock')
            return

        if self._pose_is_stale() or not self.lidar_have_full_odom:
            self._stop_and_fail_lidar('odometry_stale')
            return

        if self.lidar_slalom_phase == 'LOCK_P4':
            self._monitor_p4_lock(now)
            return

        if self.lidar_slalom_phase == 'FINAL_TURN':
            self._monitor_lidar_final_turn(now)
            return

        if self.lidar_slalom_phase != 'RUNNING':
            self._stop_and_fail_lidar(
                f'unknown_lidar_slalom_phase={self.lidar_slalom_phase}'
            )
            return

        cloud_stale = (
            self.lidar_last_cloud_receive_sec <= 0.0
            or (now - self.lidar_last_cloud_receive_sec)
            > self.lidar_cloud_stale_timeout_sec
        )
        if cloud_stale:
            cloud_age = (
                math.inf
                if self.lidar_last_cloud_receive_sec <= 0.0
                else now - self.lidar_last_cloud_receive_sec
            )
            stopped = self._handle_lidar_runtime_violation(
                now,
                stop_enabled=self.lidar_runtime_cloud_stale_stop_enabled,
                reason=(
                    f'raw_lidar_cloud_stale age={cloud_age:.3f}s>'
                    f'{self.lidar_cloud_stale_timeout_sec:.3f}s '
                    f'poles={len(self.lidar_locked_poles)}/4'
                ),
            )
            if stopped:
                return
        if (now - self.lidar_run_start_sec) > self.lidar_route_timeout_sec:
            self._stop_and_fail_lidar('lidar_slalom_route_timeout')
            return

        self._try_lock_next_pole()
        self._update_mandatory_zone_progress()
        if (
            self.lidar_final_turn_enabled
            and self.lidar_zone_indices
            and self.lidar_zone_cursor >= len(self.lidar_zone_indices)
        ):
            self._start_lidar_final_turn()
            return

        pose_xy = np.array((self.pose_x, self.pose_y), dtype=float)
        center_clearance = self._minimum_pole_clearance(pose_xy)
        body_clearance = self._minimum_body_clearance(pose_xy)
        if body_clearance < self.lidar_min_body_clearance_m:
            stopped = self._handle_lidar_runtime_violation(
                now,
                stop_enabled=self.lidar_runtime_geometry_stop_enabled,
                reason=(
                    f'body_to_pole_clearance={body_clearance:.3f}m<'
                    f'{self.lidar_min_body_clearance_m:.3f}m '
                    f'center_distance={center_clearance:.3f}m'
                ),
            )
            if stopped:
                return

        tracking = path_tracking_target(
            self.lidar_path_xy,
            pose_xy,
            self.lidar_path_index,
            lookahead_m=self.lidar_lookahead_m,
        )
        final_curve_active = (
            self.lidar_final_absolute_yaw_enabled
            and self.lidar_final_curve_start_index >= 0
            and tracking.progress_index >= self.lidar_final_curve_start_index
        )
        if final_curve_active:
            tapered_lookahead = max(
                self.lidar_final_curve_min_lookahead_m,
                min(self.lidar_lookahead_m, 0.35 * tracking.remaining_m),
            )
            tracking = path_tracking_target(
                self.lidar_path_xy,
                pose_xy,
                self.lidar_path_index,
                lookahead_m=tapered_lookahead,
            )
        if tracking.cross_track_m > self.lidar_max_cross_track_m:
            stopped = self._handle_lidar_runtime_violation(
                now,
                stop_enabled=self.lidar_runtime_geometry_stop_enabled,
                reason=(
                    f'cross_track={tracking.cross_track_m:.3f}m>'
                    f'{self.lidar_max_cross_track_m:.3f}m'
                ),
            )
            if stopped:
                return

        target_index = tracking.target_index
        if self.lidar_zone_cursor < len(self.lidar_zone_indices):
            zone_index = self.lidar_zone_indices[self.lidar_zone_cursor]
            target_index = min(target_index, zone_index)
            self.lidar_path_index = min(tracking.progress_index, zone_index)
        else:
            self.lidar_path_index = tracking.progress_index
        target_xy = self.lidar_path_xy[target_index]

        final_distance = float(np.linalg.norm(pose_xy - self.lidar_path_xy[-1]))
        final_yaw_error = normalize_angle(
            self.lidar_final_absolute_yaw_rad - self.pose_yaw
        )
        final_yaw_ready = (
            not self.lidar_final_absolute_yaw_enabled
            or abs(final_yaw_error)
            <= self.lidar_final_absolute_yaw_tolerance_rad
        )
        if (
            self.lidar_zone_cursor >= len(self.lidar_zone_indices)
            and tracking.remaining_m <= self.lidar_finish_distance_m
            and final_distance <= self.lidar_finish_distance_m
            and final_yaw_ready
        ):
            self._finish_lidar_slalom()
            return

        command_cruise_norm = self.lidar_cruise_norm
        command_minimum_norm = self.lidar_minimum_norm
        command_max_delta_norm = self.lidar_max_delta_norm
        command_target_yaw = None
        if final_curve_active:
            curve_length = max(
                1,
                len(self.lidar_path_xy) - 1 - self.lidar_final_curve_start_index,
            )
            curve_progress = max(
                0.0,
                min(
                    1.0,
                    (
                        tracking.progress_index
                        - self.lidar_final_curve_start_index
                    )
                    / curve_length,
                ),
            )
            slowdown_progress = max(
                0.0,
                min(1.0, tracking.remaining_m / 0.35),
            )
            command_cruise_norm = (
                self.lidar_final_curve_minimum_norm
                + slowdown_progress
                * (
                    self.lidar_cruise_norm
                    - self.lidar_final_curve_minimum_norm
                )
            )
            command_minimum_norm = self.lidar_final_curve_minimum_norm
            command_max_delta_norm = self.lidar_final_curve_max_delta_norm
            if curve_progress > self.lidar_final_curve_yaw_blend_start:
                target_delta = target_xy - pose_xy
                path_target_yaw = (
                    math.atan2(float(target_delta[1]), float(target_delta[0]))
                    - math.pi / 2.0
                )
                blend = (
                    curve_progress - self.lidar_final_curve_yaw_blend_start
                ) / (1.0 - self.lidar_final_curve_yaw_blend_start)
                blend = blend * blend * (3.0 - 2.0 * blend)
                command_target_yaw = normalize_angle(
                    path_target_yaw
                    + blend
                    * normalize_angle(
                        self.lidar_final_absolute_yaw_rad - path_target_yaw
                    )
                )
        left, right, yaw_error = differential_tracking_command(
            pose_xy,
            self.pose_yaw,
            target_xy,
            cruise_norm=command_cruise_norm,
            minimum_norm=command_minimum_norm,
            heading_gain_norm_per_rad=self.lidar_heading_gain_norm_per_rad,
            max_delta_norm=command_max_delta_norm,
            slowdown_heading_rad=self.lidar_slowdown_heading_rad,
            target_pose_yaw_rad=command_target_yaw,
        )
        self.lidar_slalom_motion_started = True
        self._publish_step_override(self.lidar_slalom_mode, left, right)
        self._publish_state('LIDAR_SLALOM_RUNNING')
        if (now - self.lidar_last_progress_log_sec) >= self.lidar_progress_log_interval_sec:
            self.lidar_last_progress_log_sec = now
            self._publish_feedback(
                'LiDAR 连续绕杆中: '
                f'path={self.lidar_path_index}/{len(self.lidar_path_xy) - 1}, '
                f'remaining={tracking.remaining_m:.2f}m, cross={tracking.cross_track_m:.2f}m, '
                f'center_clearance={center_clearance:.2f}m, '
                f'body_clearance={body_clearance:.2f}m, '
                f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                f'final_pose_curve={final_curve_active}, '
                f'zones={self.lidar_zone_cursor}/{len(self.lidar_zone_indices)}, '
                f'poles={len(self.lidar_locked_poles)}/4, '
                f'override=[{self.lidar_slalom_mode},{left:.2f},{right:.2f}]'
            )

    def _monitor_p4_lock(self, now: float) -> None:
        self._publish_step_override(self.lidar_slalom_mode, 0.0, 0.0)
        if self._raw_cloud_is_stale(now):
            if (now - self.lidar_lock_start_sec) > self.lidar_lock_timeout_sec:
                self._fallback_or_fail_lidar('raw_lidar_missing_before_p4_lock')
            return
        if self.lidar_cloud_sequence == self.lidar_lock_last_cloud_sequence:
            return
        self.lidar_lock_last_cloud_sequence = self.lidar_cloud_sequence
        points = self._aggregate_lidar_points()
        candidate = detect_entry_p4(
            points,
            self.lidar_entry_pose_xyz,
            self.lidar_entry_forward_xy,
            forward_min_m=self.lidar_entry_forward_min_m,
            forward_max_m=self.lidar_entry_forward_max_m,
            lateral_max_m=self.lidar_entry_lateral_max_m,
            z_min_m=self.lidar_vertical_z_min_m,
            z_max_m=self.lidar_vertical_z_max_m,
            cell_size_m=self.lidar_vertical_cell_size_m,
            min_points=self.lidar_vertical_min_points,
            min_z_span_m=self.lidar_vertical_min_z_span_m,
        )
        if candidate is not None:
            candidate_xy = candidate.xy
            if (
                self.lidar_lock_candidate_xy is not None
                and np.linalg.norm(candidate_xy - self.lidar_lock_candidate_xy)
                <= self.lidar_lock_consistency_m
            ):
                self.lidar_lock_candidate_count += 1
                weight = 1.0 / self.lidar_lock_candidate_count
                self.lidar_lock_candidate_xy = (
                    (1.0 - weight) * self.lidar_lock_candidate_xy
                    + weight * candidate_xy
                )
            else:
                self.lidar_lock_candidate_xy = candidate_xy
                self.lidar_lock_candidate_count = 1
            if self.lidar_lock_candidate_count >= self.lidar_lock_samples:
                self._complete_p4_lock()
                return
        if (now - self.lidar_lock_start_sec) > self.lidar_lock_timeout_sec:
            self._fallback_or_fail_lidar('p4_vertical_cluster_lock_timeout')

    def _complete_p4_lock(self) -> None:
        p4 = self.lidar_lock_candidate_xy.copy()
        self.lidar_frame = SlalomFrame.from_p4_and_entry_heading(
            p4,
            self.lidar_entry_forward_xy,
            self.lidar_pole_spacing_m,
        )
        self.lidar_locked_poles = {0: p4}
        self.lidar_route_start_xy = np.array((self.pose_x, self.pose_y), dtype=float)
        self._rebuild_lidar_path(reset_progress=True)
        path_center_clearance = self._minimum_path_clearance()
        path_body_clearance = self._minimum_path_body_clearance()
        if path_body_clearance < self.lidar_min_planned_body_clearance_m:
            self._fallback_or_fail_lidar(
                f'planned_body_clearance={path_body_clearance:.3f}m<'
                f'{self.lidar_min_planned_body_clearance_m:.3f}m'
            )
            return
        self.lidar_slalom_phase = 'RUNNING'
        self.lidar_run_start_sec = self._now_sec()
        self.lidar_last_progress_log_sec = 0.0
        self._publish_state('LIDAR_SLALOM_RUNNING')
        self._publish_feedback(
            'P4 局部锁定完成并立即进入运动: '
            f'p4=({p4[0]:.3f},{p4[1]:.3f}) only-in-current-run, '
            f'path={len(self.lidar_path_xy)} samples, '
            f'planned_center_clearance={path_center_clearance:.3f}m, '
            f'planned_body_clearance={path_body_clearance:.3f}m。'
        )

    def _try_lock_next_pole(self) -> None:
        if self.lidar_frame is None or len(self.lidar_locked_poles) >= 4:
            return
        if self.lidar_cloud_sequence == self.lidar_track_last_cloud_sequence:
            return
        self.lidar_track_last_cloud_sequence = self.lidar_cloud_sequence
        next_index = len(self.lidar_locked_poles)
        expected = self.lidar_frame.pole_centers_world()[next_index]
        candidate = detect_expected_pole(
            self._aggregate_lidar_points(),
            expected,
            self.lidar_entry_pose_xyz[2],
            search_radius_m=self.lidar_expected_search_radius_m,
            z_min_m=self.lidar_vertical_z_min_m,
            z_max_m=self.lidar_vertical_z_max_m,
            cell_size_m=self.lidar_vertical_cell_size_m,
            min_points=self.lidar_vertical_min_points,
            min_z_span_m=self.lidar_vertical_min_z_span_m,
        )
        if candidate is None:
            self.lidar_track_candidate_xy = None
            self.lidar_track_candidate_count = 0
            return
        candidate_xy = candidate.xy
        if (
            self.lidar_track_candidate_xy is not None
            and np.linalg.norm(candidate_xy - self.lidar_track_candidate_xy)
            <= self.lidar_lock_consistency_m
        ):
            self.lidar_track_candidate_count += 1
            weight = 1.0 / self.lidar_track_candidate_count
            self.lidar_track_candidate_xy = (
                (1.0 - weight) * self.lidar_track_candidate_xy
                + weight * candidate_xy
            )
        else:
            self.lidar_track_candidate_xy = candidate_xy
            self.lidar_track_candidate_count = 1
        if self.lidar_track_candidate_count < self.lidar_lock_samples:
            return
        self.lidar_locked_poles[next_index] = self.lidar_track_candidate_xy.copy()
        self.lidar_track_candidate_xy = None
        self.lidar_track_candidate_count = 0
        self._refit_lidar_frame()
        self._rebuild_lidar_path(reset_progress=False)
        pole_name = ('P4', 'P3', 'P2', 'P1')[next_index]
        center = self.lidar_locked_poles[next_index]
        self._publish_feedback(
            f'{pole_name} 顺序门控锁定: center=({center[0]:.3f},{center[1]:.3f}), '
            f'spacing={self.lidar_frame.pole_spacing_m:.3f}m; 已重投影后续路径。'
        )

    def _refit_lidar_frame(self) -> None:
        p4 = self.lidar_locked_poles[0]
        old_frame = self.lidar_frame
        if 1 not in self.lidar_locked_poles:
            return
        p3 = self.lidar_locked_poles[1]
        up = self._unit_xy(p3 - p4)
        leg = -up
        spacing_values = [float(np.linalg.norm(p3 - p4))]
        row = np.array((up[1], -up[0]), dtype=float)
        if 2 in self.lidar_locked_poles:
            p2 = self.lidar_locked_poles[2]
            measured_row = self._unit_xy(p2 - p3)
            if float(measured_row @ row) < 0.0:
                measured_row = -measured_row
            row = measured_row
            spacing_values.append(float(np.linalg.norm(p2 - p3)))
        if 3 in self.lidar_locked_poles:
            p1 = self.lidar_locked_poles[3]
            measured_row = self._unit_xy(p1 - p3)
            if float(measured_row @ row) < 0.0:
                measured_row = -measured_row
            row = measured_row
            spacing_values.append(0.5 * float(np.linalg.norm(p1 - p3)))
        measured_spacing = float(np.median(spacing_values))
        reference_spacing = old_frame.pole_spacing_m if old_frame is not None else 1.0
        spacing = max(0.90 * reference_spacing, min(1.10 * reference_spacing, measured_spacing))
        self.lidar_frame = SlalomFrame(
            p3_xy=p3.copy(),
            row_unit=row,
            leg_unit=leg,
            pole_spacing_m=spacing,
        )

    def _rebuild_lidar_path(self, *, reset_progress: bool) -> None:
        old_length = len(self.lidar_path_xy)
        old_index = self.lidar_path_index
        self.lidar_path_xy = build_slalom_path(
            self.lidar_frame,
            self.lidar_route_start_xy,
            self.lidar_template_points_rl,
            sample_spacing_m=self.lidar_sample_spacing_m,
        )
        self.lidar_final_curve_start_index = -1
        zones_local = np.asarray(
            ((0.40, 1.40), (-0.30, -0.30), (2.40, 0.40)), dtype=float
        ) * self.lidar_frame.pole_spacing_m
        zones_world = self.lidar_frame.local_to_world(zones_local)
        if self.lidar_final_absolute_yaw_enabled:
            anchor_world = self.lidar_frame.local_to_world(
                (
                    self.lidar_final_curve_anchor_rl
                    * self.lidar_frame.pole_spacing_m,
                )
            )[0]
            final_forward = np.array(
                (
                    -math.sin(self.lidar_final_absolute_yaw_rad),
                    math.cos(self.lidar_final_absolute_yaw_rad),
                ),
                dtype=float,
            )
            (
                self.lidar_path_xy,
                self.lidar_final_curve_start_index,
            ) = replace_path_tail_with_pose_curve(
                self.lidar_path_xy,
                anchor_world,
                zones_world[-1],
                final_forward,
                start_handle_m=self.lidar_final_curve_start_handle_m,
                final_handle_m=self.lidar_final_curve_end_handle_m,
                sample_spacing_m=self.lidar_final_curve_sample_spacing_m,
            )
        if reset_progress or old_length < 2:
            self.lidar_path_index = 0
        else:
            fraction = old_index / max(1, old_length - 1)
            self.lidar_path_index = min(
                len(self.lidar_path_xy) - 1,
                int(round(fraction * (len(self.lidar_path_xy) - 1))),
            )
        self.lidar_zone_indices = [
            int(np.argmin(np.linalg.norm(self.lidar_path_xy - zone[None, :], axis=1)))
            for zone in zones_world
        ]
        self.lidar_zone_cursor = min(self.lidar_zone_cursor, len(self.lidar_zone_indices))

    def _update_mandatory_zone_progress(self) -> None:
        if self.lidar_zone_cursor >= len(self.lidar_zone_indices):
            return
        pose = np.array((self.pose_x, self.pose_y), dtype=float)
        zone_index = self.lidar_zone_indices[self.lidar_zone_cursor]
        zone = self.lidar_path_xy[zone_index]
        distance = float(np.linalg.norm(pose - zone))
        if distance > self.lidar_mandatory_zone_tolerance_m:
            return
        self.lidar_zone_cursor += 1
        self._publish_feedback(
            f'圆形必达区 {self.lidar_zone_cursor}/3 已进入: center_error={distance:.3f}m '
            f'<= {self.lidar_mandatory_zone_tolerance_m:.3f}m。'
        )
        if (
            self.lidar_final_absolute_yaw_enabled
            and self.lidar_zone_cursor >= len(self.lidar_zone_indices)
        ):
            self._publish_feedback(
                '最后必达点位置已进入，当前位置就是绕杆终点；'
                '等待同一条末端曲线将航向收敛到绝对 '
                f'{math.degrees(self.lidar_final_absolute_yaw_rad):+.1f}deg。'
            )

    def _capture_lidar_entry_pose(self) -> bool:
        if (
            self.pose_topic_type != 'odometry'
            or not self._pose_ready_for_goal()
            or not self.lidar_have_full_odom
        ):
            return False
        self.lidar_entry_pose_xyz = self.lidar_pose_xyz.copy()
        self.lidar_entry_forward_xy = np.array(
            (-math.sin(self.pose_yaw), math.cos(self.pose_yaw)), dtype=float
        )
        return True

    def _aggregate_lidar_points(self) -> np.ndarray:
        if not self.lidar_cloud_frames:
            return np.empty((0, 3), dtype=float)
        return np.concatenate([frame[1] for frame in self.lidar_cloud_frames], axis=0)

    def _minimum_pole_clearance(self, pose_xy: np.ndarray) -> float:
        if self.lidar_frame is None:
            return math.inf
        centers = self._current_pole_centers()
        return float(np.min(np.linalg.norm(centers - pose_xy[None, :], axis=1)))

    def _minimum_body_clearance(self, pose_xy: np.ndarray) -> float:
        forward = np.array(
            (-math.sin(self.pose_yaw), math.cos(self.pose_yaw)), dtype=float
        )
        return oriented_rectangle_pole_clearance(
            pose_xy,
            forward,
            self._current_pole_centers(),
            robot_length_m=self.lidar_robot_length_m,
            robot_width_m=self.lidar_robot_width_m,
            pole_radius_m=self.lidar_pole_radius_m,
        )

    def _minimum_path_clearance(self) -> float:
        centers = self.lidar_frame.pole_centers_world()
        distances = np.linalg.norm(
            self.lidar_path_xy[:, None, :] - centers[None, :, :], axis=2
        )
        return float(np.min(distances))

    def _minimum_path_body_clearance(self) -> float:
        return path_rectangle_pole_clearance(
            self.lidar_path_xy,
            self.lidar_frame.pole_centers_world(),
            robot_length_m=self.lidar_robot_length_m,
            robot_width_m=self.lidar_robot_width_m,
            pole_radius_m=self.lidar_pole_radius_m,
        )

    def _current_pole_centers(self) -> np.ndarray:
        centers = self.lidar_frame.pole_centers_world()
        for index, locked in self.lidar_locked_poles.items():
            centers[index] = locked
        return centers

    def _raw_cloud_is_stale(self, now: float) -> bool:
        return (
            self.lidar_last_cloud_receive_sec <= 0.0
            or (now - self.lidar_last_cloud_receive_sec)
            > self.lidar_cloud_stale_timeout_sec
        )

    def _start_lidar_final_turn(self) -> None:
        self.lidar_final_turn_start_yaw = self.pose_yaw
        self.lidar_final_turn_last_yaw = self.pose_yaw
        self.lidar_final_turn_progress_rad = 0.0
        self.lidar_final_turn_target_yaw = relative_turn_target_yaw(
            self.lidar_final_turn_start_yaw,
            self.lidar_final_turn_delta_rad,
        )
        self.lidar_final_turn_start_sec = self._now_sec()
        self.lidar_final_turn_stable_since = None
        self.lidar_slalom_phase = 'FINAL_TURN'
        self._publish_step_override(self.lidar_final_turn_mode, 0.0, 0.0)
        self._publish_state('LIDAR_SLALOM_FINAL_TURN')
        self._publish_feedback(
            '最后必达区到达，开始相对当前航向收尾转向: '
            f'delta={math.degrees(self.lidar_final_turn_delta_rad):+.1f}deg, '
            f'target_yaw={math.degrees(self.lidar_final_turn_target_yaw):+.1f}deg, '
            f'start_yaw={math.degrees(self.lidar_final_turn_start_yaw):+.1f}deg。'
        )

    def _monitor_lidar_final_turn(self, now: float) -> None:
        if (now - self.lidar_final_turn_start_sec) > self.lidar_final_turn_timeout_sec:
            self._stop_and_fail_lidar('final_ccw_turn_timeout')
            return
        yaw_error = self._normalize_angle(
            self.lidar_final_turn_target_yaw - self.pose_yaw
        )
        yaw_step = self._normalize_angle(
            self.pose_yaw - self.lidar_final_turn_last_yaw
        )
        self.lidar_final_turn_last_yaw = self.pose_yaw
        turn_direction = 1.0 if self.lidar_final_turn_delta_rad >= 0.0 else -1.0
        self.lidar_final_turn_progress_rad = max(
            0.0,
            self.lidar_final_turn_progress_rad + turn_direction * yaw_step,
        )
        target_progress_rad = abs(self.lidar_final_turn_delta_rad)
        target_reached = (
            self.lidar_final_turn_progress_rad >= target_progress_rad
            or abs(yaw_error) <= self.lidar_final_turn_tolerance_rad
        )
        if self.lidar_final_turn_finish_immediately and target_reached:
            self._publish_step_override(self.lidar_final_turn_mode, 0.0, 0.0)
            self._publish_feedback(
                'LiDAR 绕杆收尾转角已达到，立即结束且不等待稳定: '
                f'progress={math.degrees(self.lidar_final_turn_progress_rad):.1f}/'
                f'{math.degrees(target_progress_rad):.1f}deg, '
                f'yaw_err={math.degrees(yaw_error):+.1f}deg。'
            )
            self._finish_lidar_slalom()
            return
        yaw_rate_ok = (
            self.lidar_final_turn_max_yaw_rate_rad_s <= 0.0
            or abs(self.pose_yaw_rate_rad_s)
            <= self.lidar_final_turn_max_yaw_rate_rad_s
        )
        if abs(yaw_error) <= self.lidar_final_turn_tolerance_rad:
            self._publish_step_override(self.lidar_final_turn_mode, 0.0, 0.0)
            if not yaw_rate_ok:
                self.lidar_final_turn_stable_since = None
                return
            if self.lidar_final_turn_stable_since is None:
                self.lidar_final_turn_stable_since = now
                return
            if (
                now - self.lidar_final_turn_stable_since
                >= self.lidar_final_turn_hold_sec
            ):
                self._finish_lidar_slalom()
            return

        self.lidar_final_turn_stable_since = None
        left, right = final_turn_override(
            yaw_error,
            fine_error_rad=self.lidar_final_turn_fine_error_rad,
            ccw_left_norm=self.lidar_final_turn_ccw_left_norm,
            ccw_right_norm=self.lidar_final_turn_ccw_right_norm,
            cw_left_norm=self.lidar_final_turn_cw_left_norm,
            cw_right_norm=self.lidar_final_turn_cw_right_norm,
            fine_left_norm=self.lidar_final_turn_fine_left_norm,
            fine_right_norm=self.lidar_final_turn_fine_right_norm,
        )
        self._publish_step_override(self.lidar_final_turn_mode, left, right)
        self._publish_state('LIDAR_SLALOM_FINAL_TURN')
        if (now - self.lidar_last_progress_log_sec) >= self.lidar_progress_log_interval_sec:
            self.lidar_last_progress_log_sec = now
            self._publish_feedback(
                'LiDAR 绕杆收尾转向中: '
                f'yaw_err={math.degrees(yaw_error):+.1f}deg, '
                f'yaw_rate={math.degrees(self.pose_yaw_rate_rad_s):+.1f}deg/s, '
                f'override=[{self.lidar_final_turn_mode},{left:.3f},{right:.3f}]'
            )

    def _handle_lidar_runtime_violation(
        self,
        now: float,
        *,
        stop_enabled: bool,
        reason: str,
    ) -> bool:
        if stop_enabled:
            self._stop_and_fail_lidar(reason)
            return True
        if (
            now - self.lidar_last_runtime_guard_warning_sec
            >= self.lidar_progress_log_interval_sec
        ):
            self.lidar_last_runtime_guard_warning_sec = now
            self._publish_feedback(
                'LiDAR 运行期停车已关闭，异常仅告警并继续: '
                f'{reason}'
            )
        return False

    def _fallback_or_fail_lidar(self, reason: str) -> None:
        if (
            not self.lidar_slalom_motion_started
            and self.lidar_fallback_to_legacy_before_motion
        ):
            self._publish_step_override(self.lidar_slalom_mode, 0.0, 0.0)
            self._publish_feedback(
                f'LiDAR 绕杆尚未运动且锁定失败，回退旧 14 点路线: {reason}'
            )
            self.lidar_slalom_active = False
            self.lidar_slalom_phase = ''
            self.triggered = False
            super()._start_route(
                f'lidar_pre_motion_fallback: {reason}',
                source='apriltag',
            )
            return
        self._stop_and_fail_lidar(reason)

    def _stop_and_fail_lidar(self, reason: str) -> None:
        self._publish_step_override(self.lidar_slalom_mode, 0.0, 0.0)
        self.lidar_slalom_active = False
        self.lidar_slalom_phase = ''
        self._handle_goal_failed(f'lidar_slalom: {reason}')

    def _finish_lidar_slalom(self) -> None:
        self._publish_step_override(self.lidar_slalom_mode, 0.0, 0.0)
        self._publish_arrival(True)
        self.lidar_slalom_active = False
        self.lidar_slalom_phase = ''
        self._finish_route()
        if self.lidar_final_absolute_yaw_enabled:
            exit_summary = (
                '出口曲线与绝对航向 '
                f'{math.degrees(self.lidar_final_absolute_yaw_rad):+.1f}deg 均完成。'
            )
        else:
            exit_summary = '出口相对收尾转向已完成。'
        self._publish_feedback(
            'LiDAR 连续绕杆完成：3 个圆形必达区均通过，P4->P3->P2->P1 '
            f'顺序正确，{exit_summary}'
        )

    def _active_route_mode(self) -> int:
        if self.lidar_slalom_active:
            if self.lidar_slalom_phase == 'FINAL_TURN':
                return self.lidar_final_turn_mode
            return self.lidar_slalom_mode
        return super()._active_route_mode()

    def _reset_route_runtime(self) -> None:
        super()._reset_route_runtime()
        self._reset_lidar_runtime(clear_clouds=True)

    def _deactivate_controller(self, reason: str) -> None:
        self._reset_lidar_runtime(clear_clouds=True)
        super()._deactivate_controller(reason)

    def _reset_lidar_runtime(self, *, clear_clouds: bool) -> None:
        self.lidar_slalom_active = False
        self.lidar_slalom_phase = ''
        self.lidar_slalom_motion_started = False
        self.lidar_entry_pose_xyz = None
        self.lidar_entry_forward_xy = None
        self.lidar_lock_candidate_xy = None
        self.lidar_lock_candidate_count = 0
        self.lidar_lock_last_cloud_sequence = -1
        self.lidar_lock_start_sec = 0.0
        self.lidar_run_start_sec = 0.0
        self.lidar_route_start_xy = None
        self.lidar_frame = None
        self.lidar_path_xy = np.empty((0, 2), dtype=float)
        self.lidar_path_index = 0
        self.lidar_final_curve_start_index = -1
        self.lidar_zone_indices = []
        self.lidar_zone_cursor = 0
        self.lidar_locked_poles = {}
        self.lidar_track_candidate_xy = None
        self.lidar_track_candidate_count = 0
        self.lidar_track_last_cloud_sequence = -1
        self.lidar_last_progress_log_sec = 0.0
        self.lidar_last_runtime_guard_warning_sec = 0.0
        self.lidar_final_turn_start_yaw = 0.0
        self.lidar_final_turn_last_yaw = 0.0
        self.lidar_final_turn_progress_rad = 0.0
        self.lidar_final_turn_target_yaw = 0.0
        self.lidar_final_turn_start_sec = 0.0
        self.lidar_final_turn_stable_since = None
        if clear_clouds:
            self.lidar_cloud_frames.clear()
            self.lidar_cloud_sequence = 0
            self.lidar_last_cloud_receive_sec = 0.0

    def _message_stamp_sec(self, msg) -> float:
        stamp = msg.header.stamp
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if value > 0.0 else self._now_sec()

    @staticmethod
    def _unit_xy(vector: np.ndarray) -> np.ndarray:
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-9:
            raise ValueError('cannot normalize a zero-length pole vector')
        return vector / norm


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrangePoleLidarSlalom()
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
