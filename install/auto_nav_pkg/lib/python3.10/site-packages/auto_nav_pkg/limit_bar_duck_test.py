#!/usr/bin/env python3
"""Blue-white limit-bar detector with direct-serial crouch traversal test.

2026-05-05:
Purpose: detect the obstacle-race limit bar without YOLO, then switch the dog
into crouch mode and keep walking under the bar for a fixed Odometry distance.
The jump-bar test remains in orange_hurdle_jump_test.py; this node is a
separate bringup for the limit-bar scenario.
"""

import math
from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from auto_nav_pkg.log_style import colorize_log
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32MultiArray, String

from .orange_hurdle_jump_test import RawSerialSender


@dataclass
class LimitBarCandidate:
    distance_m: float
    lateral_m: float
    vertical_m: float
    pixel_x: float
    pixel_y: float
    line_length_px: float
    angle_deg: float
    valid_depth_count: int
    blue_pixels: int
    white_pixels: int
    x1: int
    y1: int
    x2: int
    y2: int
    width_px: int
    height_px: int
    source: str


class LimitBarDuckTest(Node):
    """Detect a blue-white horizontal limit bar and duck under it."""

    STATE_WALKING = 'WALKING'
    STATE_DUCK_WALK = 'DUCK_WALK'
    STATE_WAIT_RESUME = 'WAIT_RESUME'
    STATE_WALKING_DONE = 'WALKING_DONE'

    def __init__(self) -> None:
        super().__init__('limit_bar_duck_test')

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('pose_topic', '/Odometry')

        self.declare_parameter('nearest_topic', '/limit_bar_duck_test/nearest')
        self.declare_parameter('trigger_topic', '/limit_bar_duck_test/trigger')
        self.declare_parameter('feedback_topic', '/limit_bar_duck_test/feedback_log')
        self.declare_parameter('state_topic', '/limit_bar_duck_test/state')
        self.declare_parameter('debug_image_topic', '/limit_bar_duck_test/debug_image')
        self.declare_parameter('detector_only', False)

        self.declare_parameter('serial_device', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('output_precision', 3)
        self.declare_parameter('message_prefix', '[')
        self.declare_parameter('field_separator', ',')
        self.declare_parameter('message_suffix', ']\n')

        self.declare_parameter('blue_hsv_lower', [95, 100, 60])
        self.declare_parameter('blue_hsv_upper', [125, 255, 255])
        self.declare_parameter('white_hsv_lower', [0, 0, 120])
        self.declare_parameter('white_hsv_upper', [180, 90, 255])
        self.declare_parameter('orange_suppress_enabled', False)
        self.declare_parameter('orange_suppress_hsv_lower', [0, 50, 50])
        self.declare_parameter('orange_suppress_hsv_upper', [45, 255, 255])
        self.declare_parameter('orange_suppress_min_area_ratio', 0.08)
        self.declare_parameter('orange_suppress_min_width_ratio', 0.35)
        self.declare_parameter('orange_suppress_min_height_ratio', 0.18)
        self.declare_parameter('orange_suppress_min_bottom_y_ratio', 0.75)
        self.declare_parameter('orange_suppress_vertical_enabled', True)
        self.declare_parameter('orange_suppress_vertical_min_area_ratio', 0.015)
        self.declare_parameter('orange_suppress_vertical_min_height_ratio', 0.45)
        self.declare_parameter('orange_suppress_vertical_max_width_ratio', 0.35)
        self.declare_parameter('orange_suppress_vertical_min_bottom_y_ratio', 0.65)
        self.declare_parameter('orange_suppress_vertical_min_aspect_ratio', 2.0)
        self.declare_parameter('morph_kernel_px', 5)
        self.declare_parameter('canny_low_threshold', 40)
        self.declare_parameter('canny_high_threshold', 120)
        self.declare_parameter('hough_threshold', 35)
        self.declare_parameter('min_line_length_px', 140)
        self.declare_parameter('max_line_gap_px', 24)
        self.declare_parameter('horizontal_angle_tolerance_deg', 12.0)
        self.declare_parameter('line_band_half_height_px', 12)
        self.declare_parameter('line_band_horizontal_pad_px', 12)
        self.declare_parameter('min_blue_pixels', 40)
        self.declare_parameter('min_white_pixels', 40)
        self.declare_parameter('component_close_kernel_width_px', 41)
        self.declare_parameter('component_close_kernel_height_px', 11)
        self.declare_parameter('component_search_y_min_px', 180)
        self.declare_parameter('component_search_y_max_px', 380)
        self.declare_parameter('component_expand_x_px', 80)
        self.declare_parameter('component_expand_y_px', 18)
        self.declare_parameter('min_component_area_px', 1800.0)
        self.declare_parameter('min_component_width_px', 180)
        self.declare_parameter('min_component_height_px', 18)
        self.declare_parameter('max_component_height_px', 90)
        self.declare_parameter('min_component_aspect_ratio', 4.0)
        self.declare_parameter('min_component_blue_pixels', 300)
        self.declare_parameter('min_component_white_pixels', 300)
        self.declare_parameter('min_valid_depth_pixels', 12)
        self.declare_parameter('depth_units_divisor', 1000.0)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 2.00)
        self.declare_parameter('depth_percentile', 25.0)
        self.declare_parameter('distance_bias_m', 0.0)
        self.declare_parameter('trigger_distance_m', 0.30)
        self.declare_parameter('trigger_min_valid_depth_pixels', 120)
        self.declare_parameter('trigger_line_min_valid_depth_pixels', 240)
        self.declare_parameter('trigger_min_blue_ratio', 0.10)
        self.declare_parameter('depth_stale_timeout_sec', 0.50)
        self.declare_parameter('camera_info_stale_timeout_sec', 5.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('log_interval_sec', 1.0)

        self.declare_parameter('walk_publish_hz', 10.0)
        self.declare_parameter('resume_delay_sec', 1.0)
        self.declare_parameter('retrigger_enabled', False)

        self.declare_parameter('travel_distance_m', 0.90)
        self.declare_parameter('travel_delta_x_m', 0.0)
        self.declare_parameter('travel_delta_y_m', 0.90)
        self.declare_parameter('travel_max_x_error_m', 0.35)

        self.declare_parameter('walk_mode', 0)
        self.declare_parameter('walk_left_norm', 1.0)
        self.declare_parameter('walk_right_norm', 1.0)
        self.declare_parameter('duck_prepare_mode', 2)
        self.declare_parameter('duck_prepare_left_norm', 0.0)
        self.declare_parameter('duck_prepare_right_norm', 0.0)
        self.declare_parameter('duck_walk_mode', 2)
        self.declare_parameter('duck_walk_left_norm', 1.0)
        self.declare_parameter('duck_walk_right_norm', 1.0)
        self.declare_parameter('stand_mode', 0)
        self.declare_parameter('stand_left_norm', 0.0)
        self.declare_parameter('stand_right_norm', 0.0)

        self.color_topic = str(self.get_parameter('color_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.pose_topic = str(self.get_parameter('pose_topic').value)

        self.nearest_topic = str(self.get_parameter('nearest_topic').value)
        self.trigger_topic = str(self.get_parameter('trigger_topic').value)
        self.feedback_topic = str(self.get_parameter('feedback_topic').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.detector_only = bool(self.get_parameter('detector_only').value)

        self.serial_device = str(self.get_parameter('serial_device').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.output_precision = max(0, int(self.get_parameter('output_precision').value))
        self.message_prefix = str(self.get_parameter('message_prefix').value)
        self.field_separator = str(self.get_parameter('field_separator').value)
        self.message_suffix = str(self.get_parameter('message_suffix').value)

        self.blue_hsv_lower = np.array(self._int_list_param('blue_hsv_lower', 3), dtype=np.uint8)
        self.blue_hsv_upper = np.array(self._int_list_param('blue_hsv_upper', 3), dtype=np.uint8)
        self.white_hsv_lower = np.array(self._int_list_param('white_hsv_lower', 3), dtype=np.uint8)
        self.white_hsv_upper = np.array(self._int_list_param('white_hsv_upper', 3), dtype=np.uint8)
        self.orange_suppress_enabled = bool(
            self.get_parameter('orange_suppress_enabled').value
        )
        self.orange_suppress_hsv_lower = np.array(
            self._int_list_param('orange_suppress_hsv_lower', 3),
            dtype=np.uint8,
        )
        self.orange_suppress_hsv_upper = np.array(
            self._int_list_param('orange_suppress_hsv_upper', 3),
            dtype=np.uint8,
        )
        self.orange_suppress_min_area_ratio = max(
            0.0, float(self.get_parameter('orange_suppress_min_area_ratio').value)
        )
        self.orange_suppress_min_width_ratio = max(
            0.0, float(self.get_parameter('orange_suppress_min_width_ratio').value)
        )
        self.orange_suppress_min_height_ratio = max(
            0.0, float(self.get_parameter('orange_suppress_min_height_ratio').value)
        )
        self.orange_suppress_min_bottom_y_ratio = max(
            0.0, float(self.get_parameter('orange_suppress_min_bottom_y_ratio').value)
        )
        self.orange_suppress_vertical_enabled = bool(
            self.get_parameter('orange_suppress_vertical_enabled').value
        )
        self.orange_suppress_vertical_min_area_ratio = max(
            0.0,
            float(self.get_parameter('orange_suppress_vertical_min_area_ratio').value),
        )
        self.orange_suppress_vertical_min_height_ratio = max(
            0.0,
            float(self.get_parameter('orange_suppress_vertical_min_height_ratio').value),
        )
        self.orange_suppress_vertical_max_width_ratio = max(
            0.0,
            float(self.get_parameter('orange_suppress_vertical_max_width_ratio').value),
        )
        self.orange_suppress_vertical_min_bottom_y_ratio = max(
            0.0,
            float(self.get_parameter('orange_suppress_vertical_min_bottom_y_ratio').value),
        )
        self.orange_suppress_vertical_min_aspect_ratio = max(
            1.0,
            float(self.get_parameter('orange_suppress_vertical_min_aspect_ratio').value),
        )
        self.morph_kernel_px = max(1, int(self.get_parameter('morph_kernel_px').value))
        self.canny_low_threshold = int(self.get_parameter('canny_low_threshold').value)
        self.canny_high_threshold = int(self.get_parameter('canny_high_threshold').value)
        self.hough_threshold = int(self.get_parameter('hough_threshold').value)
        self.min_line_length_px = max(1, int(self.get_parameter('min_line_length_px').value))
        self.max_line_gap_px = max(0, int(self.get_parameter('max_line_gap_px').value))
        self.horizontal_angle_tolerance_deg = max(
            0.0,
            float(self.get_parameter('horizontal_angle_tolerance_deg').value),
        )
        self.line_band_half_height_px = max(
            1, int(self.get_parameter('line_band_half_height_px').value)
        )
        self.line_band_horizontal_pad_px = max(
            0, int(self.get_parameter('line_band_horizontal_pad_px').value)
        )
        self.min_blue_pixels = max(0, int(self.get_parameter('min_blue_pixels').value))
        self.min_white_pixels = max(0, int(self.get_parameter('min_white_pixels').value))
        self.component_close_kernel_width_px = max(
            3, int(self.get_parameter('component_close_kernel_width_px').value)
        )
        self.component_close_kernel_height_px = max(
            3, int(self.get_parameter('component_close_kernel_height_px').value)
        )
        self.component_search_y_min_px = max(
            0, int(self.get_parameter('component_search_y_min_px').value)
        )
        self.component_search_y_max_px = max(
            0, int(self.get_parameter('component_search_y_max_px').value)
        )
        self.component_expand_x_px = max(
            0, int(self.get_parameter('component_expand_x_px').value)
        )
        self.component_expand_y_px = max(
            0, int(self.get_parameter('component_expand_y_px').value)
        )
        self.min_component_area_px = max(
            1.0, float(self.get_parameter('min_component_area_px').value)
        )
        self.min_component_width_px = max(
            1, int(self.get_parameter('min_component_width_px').value)
        )
        self.min_component_height_px = max(
            1, int(self.get_parameter('min_component_height_px').value)
        )
        self.max_component_height_px = max(
            self.min_component_height_px,
            int(self.get_parameter('max_component_height_px').value),
        )
        self.min_component_aspect_ratio = max(
            1.0, float(self.get_parameter('min_component_aspect_ratio').value)
        )
        self.min_component_blue_pixels = max(
            0, int(self.get_parameter('min_component_blue_pixels').value)
        )
        self.min_component_white_pixels = max(
            0, int(self.get_parameter('min_component_white_pixels').value)
        )
        self.min_valid_depth_pixels = max(
            1, int(self.get_parameter('min_valid_depth_pixels').value)
        )
        self.depth_units_divisor = float(self.get_parameter('depth_units_divisor').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.depth_percentile = float(
            np.clip(float(self.get_parameter('depth_percentile').value), 0.0, 100.0)
        )
        self.distance_bias_m = float(self.get_parameter('distance_bias_m').value)
        self.trigger_distance_m = float(self.get_parameter('trigger_distance_m').value)
        self.trigger_min_valid_depth_pixels = max(
            1, int(self.get_parameter('trigger_min_valid_depth_pixels').value)
        )
        self.trigger_line_min_valid_depth_pixels = max(
            1, int(self.get_parameter('trigger_line_min_valid_depth_pixels').value)
        )
        self.trigger_min_blue_ratio = float(
            np.clip(float(self.get_parameter('trigger_min_blue_ratio').value), 0.0, 1.0)
        )
        self.depth_stale_timeout_sec = float(self.get_parameter('depth_stale_timeout_sec').value)
        self.camera_info_stale_timeout_sec = float(
            self.get_parameter('camera_info_stale_timeout_sec').value
        )
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)

        self.walk_publish_hz = max(1.0, float(self.get_parameter('walk_publish_hz').value))
        self.resume_delay_sec = max(0.0, float(self.get_parameter('resume_delay_sec').value))
        self.retrigger_enabled = bool(self.get_parameter('retrigger_enabled').value)

        self.travel_distance_m = max(
            0.0, float(self.get_parameter('travel_distance_m').value)
        )
        self.travel_delta_x_m = float(self.get_parameter('travel_delta_x_m').value)
        self.travel_delta_y_m = float(self.get_parameter('travel_delta_y_m').value)
        self.travel_max_x_error_m = max(
            0.0, float(self.get_parameter('travel_max_x_error_m').value)
        )
        if self.travel_distance_m <= 0.0:
            self.travel_distance_m = math.hypot(
                self.travel_delta_x_m,
                self.travel_delta_y_m,
            )

        self.walk_mode = int(self.get_parameter('walk_mode').value)
        self.walk_left_norm = float(self.get_parameter('walk_left_norm').value)
        self.walk_right_norm = float(self.get_parameter('walk_right_norm').value)
        self.duck_prepare_mode = int(self.get_parameter('duck_prepare_mode').value)
        self.duck_prepare_left_norm = float(self.get_parameter('duck_prepare_left_norm').value)
        self.duck_prepare_right_norm = float(self.get_parameter('duck_prepare_right_norm').value)
        self.duck_walk_mode = int(self.get_parameter('duck_walk_mode').value)
        self.duck_walk_left_norm = float(self.get_parameter('duck_walk_left_norm').value)
        self.duck_walk_right_norm = float(self.get_parameter('duck_walk_right_norm').value)
        self.stand_mode = int(self.get_parameter('stand_mode').value)
        self.stand_left_norm = float(self.get_parameter('stand_left_norm').value)
        self.stand_right_norm = float(self.get_parameter('stand_right_norm').value)

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_stamp_sec: Optional[float] = None
        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_camera_info_stamp_sec: Optional[float] = None
        self.last_log_time = 0.0
        self.frame_count = 0
        self.last_feedback = ''
        self.last_state = ''
        self.last_trigger_state: Optional[bool] = None
        self.current_triggered = False
        self.latest_candidate: Optional[LimitBarCandidate] = None
        self.latest_candidate_stamp_sec: Optional[float] = None

        self.latest_odom_x: Optional[float] = None
        self.latest_odom_y: Optional[float] = None
        self.latest_odom_stamp_sec: Optional[float] = None
        self.crouch_start_x: Optional[float] = None
        self.crouch_start_y: Optional[float] = None
        self.target_x: Optional[float] = None
        self.target_y: Optional[float] = None

        self.sequence_started = False
        self.state = 'DETECTOR_ONLY' if self.detector_only else self.STATE_WALKING
        self.phase_deadline_sec: Optional[float] = None
        self.serial_sender: Optional[RawSerialSender] = None
        if not self.detector_only:
            self.serial_sender = RawSerialSender(self.serial_device, self.baud_rate)
        self.serial_opened = False
        self.last_serial_payload = ''

        self.nearest_pub = self.create_publisher(Float32MultiArray, self.nearest_topic, 10)
        self.trigger_pub = self.create_publisher(Bool, self.trigger_topic, 10)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self._depth_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.color_sub = self.create_subscription(
            Image,
            self.color_topic,
            self._color_callback,
            qos_profile_sensor_data,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.pose_topic,
            self._odometry_callback,
            20,
        )
        self.command_timer = None
        if not self.detector_only:
            self.command_timer = self.create_timer(1.0 / self.walk_publish_hz, self._command_timer)

        self._publish_state(self.state)
        self._publish_feedback(
            'limit_bar_duck_test started: '
            f'color={self.color_topic}, depth={self.depth_topic}, pose={self.pose_topic}, '
            f'detector_only={self.detector_only}, '
            f'serial={self.serial_device}@{self.baud_rate}, '
            f'trigger_distance={self.trigger_distance_m:.2f}m, '
            f'travel_distance={self.travel_distance_m:.2f}m, '
            f'walk=[{self.walk_mode},{self.walk_left_norm:.3f},{self.walk_right_norm:.3f}], '
            f'duck_prepare=[{self.duck_prepare_mode},{self.duck_prepare_left_norm:.3f},{self.duck_prepare_right_norm:.3f}], '
            f'duck_walk=[{self.duck_walk_mode},{self.duck_walk_left_norm:.3f},{self.duck_walk_right_norm:.3f}]'
        )

    def _depth_callback(self, msg: Image) -> None:
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self._publish_feedback(f'深度图转换失败: {exc}', level='warn')
            return
        self.latest_depth_stamp_sec = self._stamp_to_sec(msg.header.stamp)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg
        self.latest_camera_info_stamp_sec = self._stamp_to_sec(msg.header.stamp)

    def _odometry_callback(self, msg: Odometry) -> None:
        self.latest_odom_x = float(msg.pose.pose.position.x)
        self.latest_odom_y = float(msg.pose.pose.position.y)
        self.latest_odom_stamp_sec = self._stamp_to_sec(msg.header.stamp)

    def _color_callback(self, msg: Image) -> None:
        self.frame_count += 1
        now = self._now_sec()

        if self.latest_depth is None:
            self._log_periodic('等待对齐深度图。', now)
            self._publish_trigger(False)
            return

        if self.latest_camera_info is None:
            self._log_periodic('等待 color camera_info。', now)
            self._publish_trigger(False)
            return

        if self._data_is_stale(now):
            self._clear_detection()
            self._publish_trigger(False)
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self._publish_feedback(f'彩色图转换失败: {exc}', level='warn')
            self._publish_trigger(False)
            return

        if self.latest_depth.shape[:2] != bgr.shape[:2]:
            self._log_periodic(
                f'彩色图和深度图尺寸不一致: color={bgr.shape[:2]}, depth={self.latest_depth.shape[:2]}。'
                '请确认使用 aligned_depth_to_color。',
                now,
                level='warn',
            )
            self._clear_detection()
            self._publish_trigger(False)
            return

        blue_mask, white_mask, combined_mask = self._build_masks(bgr)
        orange_suppressed, orange_summary = self._orange_foreground_info(bgr)
        candidates = (
            []
            if orange_suppressed
            else self._extract_candidates(blue_mask, white_mask, combined_mask)
        )
        nearest = min(candidates, key=lambda item: item.distance_m) if candidates else None
        triggered = nearest is not None and self._candidate_can_trigger(nearest)

        self.latest_candidate = nearest
        self.latest_candidate_stamp_sec = now if nearest is not None else None
        self.current_triggered = triggered

        self._publish_trigger(triggered)
        if nearest is not None:
            self._publish_nearest(nearest)

        if self.publish_debug_image:
            self._publish_debug_image(bgr, combined_mask, candidates, nearest, msg.header)

        self._log_detection(
            now,
            blue_mask,
            white_mask,
            combined_mask,
            candidates,
            nearest,
            triggered,
            orange_suppressed,
            orange_summary,
        )

    def _command_timer(self) -> None:
        now = self._now_sec()
        self._advance_state_machine(now)
        if self.state in (self.STATE_WALKING, self.STATE_WALKING_DONE):
            self._send_serial_command(
                self.walk_mode,
                self.walk_left_norm,
                self.walk_right_norm,
                log_payload_change=False,
            )
        elif self.state == self.STATE_DUCK_WALK:
            self._send_serial_command(
                self.duck_walk_mode,
                self.duck_walk_left_norm,
                self.duck_walk_right_norm,
                log_payload_change=False,
            )

    def _advance_state_machine(self, now: float) -> None:
        if self.state in (self.STATE_WALKING, self.STATE_WALKING_DONE):
            if not self._should_start_duck(now):
                return

            self.sequence_started = True
            self.crouch_start_x = self.latest_odom_x
            self.crouch_start_y = self.latest_odom_y
            self.target_x = None
            self.target_y = None
            self._send_serial_command(
                self.duck_prepare_mode,
                self.duck_prepare_left_norm,
                self.duck_prepare_right_norm,
                log_payload_change=True,
            )
            self._set_state(self.STATE_DUCK_WALK)
            self._publish_feedback(
                f'触发限高杆: dist={self.latest_candidate.distance_m:.3f}m, '
                f'travel_distance={self.travel_distance_m:.3f}m; 单发趴下指令'
            )
            return

        if self.state == self.STATE_DUCK_WALK:
            if not self._duck_target_reached():
                return

            self._send_serial_command(
                self.stand_mode,
                self.stand_left_norm,
                self.stand_right_norm,
                log_payload_change=True,
            )
            self.phase_deadline_sec = now + self.resume_delay_sec
            self._set_state(self.STATE_WAIT_RESUME)
            delta_x = self.latest_odom_x - self.crouch_start_x
            delta_y = self.latest_odom_y - self.crouch_start_y
            travel_m = math.hypot(delta_x, delta_y)
            self._publish_feedback(
                f'限高杆通过距离已满足: travel={travel_m:.3f}m, '
                f'dx={delta_x:.3f}m, dy={delta_y:.3f}m; 单发站立指令'
            )
            return

        if self.state == self.STATE_WAIT_RESUME:
            if self.phase_deadline_sec is None or now < self.phase_deadline_sec:
                return
            self.phase_deadline_sec = None
            next_state = self.STATE_WALKING if self.retrigger_enabled else self.STATE_WALKING_DONE
            self._set_state(next_state)
            self._publish_feedback(
                f'站立等待结束，恢复正常前行: mode={self.walk_mode}, '
                f'left={self.walk_left_norm:.3f}, right={self.walk_right_norm:.3f}'
            )

    def _should_start_duck(self, now: float) -> bool:
        if not self.current_triggered:
            return False
        if self.latest_candidate is None or self.latest_candidate_stamp_sec is None:
            return False
        if (now - self.latest_candidate_stamp_sec) > self.depth_stale_timeout_sec:
            return False
        if self.sequence_started and not self.retrigger_enabled:
            return False
        if self.latest_odom_x is None or self.latest_odom_y is None:
            self._log_periodic('已识别限高杆，但还没有可用 Odometry。', now, level='warn')
            return False
        return True

    def _duck_target_reached(self) -> bool:
        if (
            self.latest_odom_x is None or self.latest_odom_y is None
            or self.crouch_start_x is None or self.crouch_start_y is None
        ):
            return False

        delta_x = self.latest_odom_x - self.crouch_start_x
        delta_y = self.latest_odom_y - self.crouch_start_y
        travel_m = math.hypot(delta_x, delta_y)
        return travel_m >= self.travel_distance_m

    def _candidate_can_trigger(self, candidate: LimitBarCandidate) -> bool:
        if candidate.distance_m > self.trigger_distance_m:
            return False
        min_depth_pixels = self.trigger_min_valid_depth_pixels
        if candidate.source == 'line':
            min_depth_pixels = max(min_depth_pixels, self.trigger_line_min_valid_depth_pixels)
        if candidate.valid_depth_count < min_depth_pixels:
            return False
        total_color_pixels = candidate.blue_pixels + candidate.white_pixels
        if total_color_pixels <= 0:
            return False
        blue_ratio = float(candidate.blue_pixels) / float(total_color_pixels)
        if blue_ratio < self.trigger_min_blue_ratio:
            return False
        return True

    def _build_masks(self, bgr: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, self.blue_hsv_lower, self.blue_hsv_upper)
        white_mask = cv2.inRange(hsv, self.white_hsv_lower, self.white_hsv_upper)
        combined_mask = cv2.bitwise_or(blue_mask, white_mask)
        kernel = np.ones((self.morph_kernel_px, self.morph_kernel_px), dtype=np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        return blue_mask, white_mask, combined_mask

    def _orange_foreground_info(self, bgr: np.ndarray) -> tuple[bool, str]:
        if not self.orange_suppress_enabled:
            return False, 'disabled'

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            self.orange_suppress_hsv_lower,
            self.orange_suppress_hsv_upper,
        )
        kernel = np.ones((self.morph_kernel_px, self.morph_kernel_px), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, 'contours=0'

        image_h, image_w = mask.shape[:2]
        image_area = max(float(image_h * image_w), 1.0)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            x, y, w, h = cv2.boundingRect(contour)
            width_ratio = float(w) / max(float(image_w), 1.0)
            height_ratio = float(h) / max(float(image_h), 1.0)
            bottom_ratio = float(y + h) / max(float(image_h), 1.0)
            aspect_ratio = float(h) / max(float(w), 1.0)
            if (
                self.orange_suppress_vertical_enabled
                and (area / image_area) >= self.orange_suppress_vertical_min_area_ratio
                and height_ratio >= self.orange_suppress_vertical_min_height_ratio
                and (
                    self.orange_suppress_vertical_max_width_ratio <= 0.0
                    or width_ratio <= self.orange_suppress_vertical_max_width_ratio
                )
                and bottom_ratio >= self.orange_suppress_vertical_min_bottom_y_ratio
                and aspect_ratio >= self.orange_suppress_vertical_min_aspect_ratio
            ):
                return (
                    True,
                    f'orange_vertical=area={area:.0f},box={w}x{h},bottom={bottom_ratio:.2f},'
                    f'ratios=({area / image_area:.2f},{width_ratio:.2f},{height_ratio:.2f}),'
                    f'aspect={aspect_ratio:.2f}',
                )
            if (area / image_area) < self.orange_suppress_min_area_ratio:
                continue
            if width_ratio < self.orange_suppress_min_width_ratio:
                continue
            if height_ratio < self.orange_suppress_min_height_ratio:
                continue
            if bottom_ratio < self.orange_suppress_min_bottom_y_ratio:
                continue
            return (
                True,
                f'area={area:.0f},box={w}x{h},bottom={bottom_ratio:.2f},'
                f'ratios=({area / image_area:.2f},{width_ratio:.2f},'
                f'{height_ratio:.2f})',
            )
        largest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest))
        x, y, w, h = cv2.boundingRect(largest)
        bottom_ratio = float(y + h) / max(float(image_h), 1.0)
        return (
            False,
            f'largest=area:{area:.0f},box:{w}x{h},bottom:{bottom_ratio:.2f},'
            f'contours={len(contours)}',
        )

    def _extract_candidates(
        self,
        blue_mask: np.ndarray,
        white_mask: np.ndarray,
        combined_mask: np.ndarray,
    ) -> List[LimitBarCandidate]:
        component_candidates = self._extract_component_candidates(
            blue_mask,
            white_mask,
            combined_mask,
        )
        if component_candidates:
            return component_candidates

        return self._extract_line_candidates(blue_mask, white_mask, combined_mask)

    def _extract_line_candidates(
        self,
        blue_mask: np.ndarray,
        white_mask: np.ndarray,
        combined_mask: np.ndarray,
    ) -> List[LimitBarCandidate]:
        # White walls/background can make the blue+white mask cover most of the
        # frame. Prefer blue-only edges for the horizontal line, then confirm
        # nearby white pixels in the same band.
        kernel = np.ones((self.morph_kernel_px, self.morph_kernel_px), dtype=np.uint8)
        blue_line_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
        blue_line_mask = cv2.morphologyEx(blue_line_mask, cv2.MORPH_OPEN, kernel)
        blue_line_candidates = self._extract_line_candidates_from_mask(
            blue_line_mask,
            blue_mask,
            white_mask,
        )
        if blue_line_candidates:
            return blue_line_candidates

        return self._extract_line_candidates_from_mask(
            combined_mask,
            blue_mask,
            white_mask,
        )

    def _extract_line_candidates_from_mask(
        self,
        line_mask: np.ndarray,
        blue_mask: np.ndarray,
        white_mask: np.ndarray,
    ) -> List[LimitBarCandidate]:
        edges = cv2.Canny(line_mask, self.canny_low_threshold, self.canny_high_threshold)
        lines = cv2.HoughLinesP(
            edges,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length_px,
            maxLineGap=self.max_line_gap_px,
        )
        if lines is None:
            return []

        height, width = line_mask.shape[:2]
        candidates: List[LimitBarCandidate] = []
        for line in lines[:, 0, :]:
            x1, y1, x2, y2 = [int(v) for v in line]
            dx = float(x2 - x1)
            dy = float(y2 - y1)
            length_px = math.hypot(dx, dy)
            if length_px < self.min_line_length_px:
                continue
            angle_deg = math.degrees(math.atan2(dy, dx))
            if abs(angle_deg) > self.horizontal_angle_tolerance_deg:
                continue

            x_min = max(0, min(x1, x2) - self.line_band_horizontal_pad_px)
            x_max = min(width, max(x1, x2) + self.line_band_horizontal_pad_px + 1)
            y_min = max(0, min(y1, y2) - self.line_band_half_height_px)
            y_max = min(height, max(y1, y2) + self.line_band_half_height_px + 1)
            if x_max <= x_min or y_max <= y_min:
                continue

            roi_blue = blue_mask[y_min:y_max, x_min:x_max] > 0
            roi_white = white_mask[y_min:y_max, x_min:x_max] > 0
            roi_combined = roi_blue | roi_white
            blue_pixels = int(np.count_nonzero(roi_blue))
            white_pixels = int(np.count_nonzero(roi_white))
            if blue_pixels < self.min_blue_pixels or white_pixels < self.min_white_pixels:
                continue

            candidate = self._candidate_from_roi(
                roi_combined=roi_combined,
                x_min=x_min,
                y_min=y_min,
                x_max=x_max,
                y_max=y_max,
                x1=x1,
                y1=y1,
                x2=x2,
                y2=y2,
                line_length_px=length_px,
                angle_deg=angle_deg,
                blue_pixels=blue_pixels,
                white_pixels=white_pixels,
                width_px=x_max - x_min,
                height_px=y_max - y_min,
                source='line',
            )
            if candidate is not None:
                candidates.append(candidate)

        return candidates

    def _extract_component_candidates(
        self,
        blue_mask: np.ndarray,
        white_mask: np.ndarray,
        combined_mask: np.ndarray,
    ) -> List[LimitBarCandidate]:
        height, width = blue_mask.shape[:2]
        y_min = int(np.clip(self.component_search_y_min_px, 0, height - 1))
        y_max = int(np.clip(self.component_search_y_max_px, y_min + 1, height))

        search_mask = np.zeros_like(blue_mask)
        search_mask[y_min:y_max, :] = blue_mask[y_min:y_max, :]

        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (self.component_close_kernel_width_px, self.component_close_kernel_height_px),
        )
        merged_mask = cv2.morphologyEx(search_mask, cv2.MORPH_CLOSE, close_kernel)
        contours, _ = cv2.findContours(
            merged_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        candidates: List[LimitBarCandidate] = []
        for contour in contours:
            area_px = float(cv2.contourArea(contour))
            if area_px < self.min_component_area_px:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w < self.min_component_width_px:
                continue
            if h < self.min_component_height_px or h > self.max_component_height_px:
                continue
            if (float(w) / max(float(h), 1.0)) < self.min_component_aspect_ratio:
                continue

            roi_x_min = max(0, x - self.component_expand_x_px)
            roi_x_max = min(width, x + w + self.component_expand_x_px)
            roi_y_min = max(0, y - self.component_expand_y_px)
            roi_y_max = min(height, y + h + self.component_expand_y_px)

            roi_blue = blue_mask[roi_y_min:roi_y_max, roi_x_min:roi_x_max] > 0
            roi_white = white_mask[roi_y_min:roi_y_max, roi_x_min:roi_x_max] > 0
            blue_pixels = int(np.count_nonzero(roi_blue))
            white_pixels = int(np.count_nonzero(roi_white))
            if blue_pixels < self.min_component_blue_pixels:
                continue
            if white_pixels < self.min_component_white_pixels:
                continue

            roi_combined = roi_blue | roi_white
            center_y = y + (h // 2)
            candidate = self._candidate_from_roi(
                roi_combined=roi_combined,
                x_min=roi_x_min,
                y_min=roi_y_min,
                x_max=roi_x_max,
                y_max=roi_y_max,
                x1=roi_x_min,
                y1=center_y,
                x2=roi_x_max - 1,
                y2=center_y,
                line_length_px=float(roi_x_max - roi_x_min),
                angle_deg=0.0,
                blue_pixels=blue_pixels,
                white_pixels=white_pixels,
                width_px=roi_x_max - roi_x_min,
                height_px=roi_y_max - roi_y_min,
                source='component',
            )
            if candidate is not None:
                candidates.append(candidate)

        return candidates

    def _candidate_from_roi(
        self,
        roi_combined: np.ndarray,
        x_min: int,
        y_min: int,
        x_max: int,
        y_max: int,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        line_length_px: float,
        angle_deg: float,
        blue_pixels: int,
        white_pixels: int,
        width_px: int,
        height_px: int,
        source: str,
    ) -> Optional[LimitBarCandidate]:
        roi_depth = self.latest_depth[y_min:y_max, x_min:x_max]
        if roi_depth.size == 0:
            return None

        depth_m = self._depth_to_meters(roi_depth)
        valid = (
            roi_combined &
            np.isfinite(depth_m) &
            (depth_m >= self.min_depth_m) &
            (depth_m <= self.max_depth_m)
        )
        valid_depth_count = int(np.count_nonzero(valid))
        if valid_depth_count < self.min_valid_depth_pixels:
            return None

        selected_depth = depth_m[valid]
        raw_distance_m = float(np.percentile(selected_depth, self.depth_percentile))
        distance_m = raw_distance_m + self.distance_bias_m
        if not math.isfinite(distance_m):
            return None

        pixel_x = 0.5 * float(x1 + x2)
        pixel_y = 0.5 * float(y1 + y2)
        lateral_m, vertical_m = self._project_pixel(pixel_x, pixel_y, distance_m)
        return LimitBarCandidate(
            distance_m=distance_m,
            lateral_m=lateral_m,
            vertical_m=vertical_m,
            pixel_x=pixel_x,
            pixel_y=pixel_y,
            line_length_px=line_length_px,
            angle_deg=angle_deg,
            valid_depth_count=valid_depth_count,
            blue_pixels=blue_pixels,
            white_pixels=white_pixels,
            x1=x1,
            y1=y1,
            x2=x2,
            y2=y2,
            width_px=width_px,
            height_px=height_px,
            source=source,
        )

    def _depth_to_meters(self, depth: np.ndarray) -> np.ndarray:
        if np.issubdtype(depth.dtype, np.integer):
            return depth.astype(np.float32) / max(self.depth_units_divisor, 1e-6)
        return depth.astype(np.float32)

    def _project_pixel(self, u: float, v: float, depth_m: float) -> tuple[float, float]:
        info = self.latest_camera_info
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        lateral_m = (u - cx) * depth_m / max(fx, 1e-6)
        vertical_m = (v - cy) * depth_m / max(fy, 1e-6)
        return lateral_m, vertical_m

    def _send_serial_command(
        self,
        mode: int,
        left_norm: float,
        right_norm: float,
        log_payload_change: bool,
    ) -> None:
        if self.detector_only or self.serial_sender is None:
            return
        payload = (
            f'{self.message_prefix}{int(mode)}{self.field_separator}'
            f'{float(np.clip(left_norm, -1.0, 1.0)):.{self.output_precision}f}{self.field_separator}'
            f'{float(np.clip(right_norm, -1.0, 1.0)):.{self.output_precision}f}{self.message_suffix}'
        )
        try:
            self.serial_sender.send(payload.encode('utf-8'))
        except Exception as exc:
            self.serial_sender.close()
            self.serial_opened = False
            self._publish_feedback(f'串口发送失败: {exc}', level='warn')
            return

        if not self.serial_opened:
            self.serial_opened = True
            self._publish_feedback(f'已打开串口 {self.serial_device} @ {self.baud_rate}')

        if log_payload_change and payload != self.last_serial_payload:
            self._publish_feedback(f'serial tx: {payload.rstrip()}')
        self.last_serial_payload = payload

    def _publish_nearest(self, candidate: LimitBarCandidate) -> None:
        msg = Float32MultiArray()
        msg.data = [
            float(candidate.distance_m),
            float(candidate.lateral_m),
            float(candidate.vertical_m),
            float(candidate.pixel_x),
            float(candidate.pixel_y),
            float(candidate.line_length_px),
            float(candidate.angle_deg),
            float(candidate.valid_depth_count),
            float(candidate.blue_pixels),
            float(candidate.white_pixels),
        ]
        self.nearest_pub.publish(msg)

    def _publish_trigger(self, triggered: bool) -> None:
        msg = Bool()
        msg.data = bool(triggered)
        self.trigger_pub.publish(msg)
        if self.last_trigger_state is not None and self.last_trigger_state == triggered:
            return
        self.last_trigger_state = triggered
        self._publish_feedback(f'触发状态更新: trigger={triggered}')

    def _publish_debug_image(
        self,
        bgr: np.ndarray,
        combined_mask: np.ndarray,
        candidates: List[LimitBarCandidate],
        nearest: Optional[LimitBarCandidate],
        header,
    ) -> None:
        debug = bgr.copy()
        for candidate in candidates:
            color = (0, 255, 0) if candidate is nearest else (255, 0, 0)
            if candidate.source == 'component':
                x = min(candidate.x1, candidate.x2)
                y = int(candidate.pixel_y - 0.5 * candidate.height_px)
                cv2.rectangle(
                    debug,
                    (x, y),
                    (x + candidate.width_px, y + candidate.height_px),
                    color,
                    2,
                )
            else:
                cv2.line(
                    debug,
                    (candidate.x1, candidate.y1),
                    (candidate.x2, candidate.y2),
                    color,
                    3,
                )
            label = (
                f'{candidate.distance_m:.2f}m '
                f'{candidate.width_px}x{candidate.height_px} {candidate.source}'
            )
            cv2.putText(
                debug,
                label,
                (max(min(candidate.x1, candidate.x2), 8), max(min(candidate.y1, candidate.y2) - 8, 16)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
        cv2.putText(
            debug,
            f'state={self.state}',
            (16, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        mask_bgr = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
        combined = cv2.addWeighted(debug, 0.75, mask_bgr, 0.25, 0.0)
        out = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
        out.header = header
        self.debug_image_pub.publish(out)

    def _log_detection(
        self,
        now: float,
        blue_mask: np.ndarray,
        white_mask: np.ndarray,
        combined_mask: np.ndarray,
        candidates: List[LimitBarCandidate],
        nearest: Optional[LimitBarCandidate],
        triggered: bool,
        orange_suppressed: bool,
        orange_summary: str,
    ) -> None:
        if (now - self.last_log_time) < self.log_interval_sec:
            return
        self.last_log_time = now

        if nearest is None:
            reason = 'orange_foreground_suppressed' if orange_suppressed else 'no_candidate'
            reject_summary = (
                f'orange={orange_summary}'
                if orange_suppressed
                else self._candidate_reject_summary(blue_mask, white_mask, combined_mask)
            )
            self._publish_feedback(
                f'未检测到有效限高杆: candidates=0, frame={self.frame_count}, '
                f'reason={reason}, '
                f'{reject_summary}, '
                f'blue_hsv={self.blue_hsv_lower.tolist()}-{self.blue_hsv_upper.tolist()}, '
                f'white_hsv={self.white_hsv_lower.tolist()}-{self.white_hsv_upper.tolist()}'
            )
            return

        self._publish_feedback(
            f'最近限高杆: dist={nearest.distance_m:.3f}m, lateral={nearest.lateral_m:.3f}m, '
            f'vertical={nearest.vertical_m:.3f}m, pixel=({nearest.pixel_x:.1f},{nearest.pixel_y:.1f}), '
            f'size={nearest.width_px}x{nearest.height_px}, source={nearest.source}, '
            f'line_len={nearest.line_length_px:.1f}px, angle={nearest.angle_deg:.1f}deg, '
            f'depth_px={nearest.valid_depth_count}, blue_px={nearest.blue_pixels}, '
            f'white_px={nearest.white_pixels}, '
            f'blue_ratio={nearest.blue_pixels / max(nearest.blue_pixels + nearest.white_pixels, 1):.2f}, '
            f'candidates={len(candidates)}, trigger={triggered}'
        )

    def _candidate_reject_summary(
        self,
        blue_mask: np.ndarray,
        white_mask: np.ndarray,
        combined_mask: np.ndarray,
    ) -> str:
        blue_px = int(np.count_nonzero(blue_mask))
        white_px = int(np.count_nonzero(white_mask))
        combined_px = int(np.count_nonzero(combined_mask))
        component_summary = self._component_reject_summary(blue_mask, white_mask)
        line_summary = self._line_reject_summary(blue_mask, white_mask, combined_mask)
        return (
            f'mask_px=blue:{blue_px},white:{white_px},combined:{combined_px}, '
            f'component={component_summary}, line={line_summary}'
        )

    def _component_reject_summary(self, blue_mask: np.ndarray, white_mask: np.ndarray) -> str:
        height, width = blue_mask.shape[:2]
        y_min = int(np.clip(self.component_search_y_min_px, 0, height - 1))
        y_max = int(np.clip(self.component_search_y_max_px, y_min + 1, height))
        search_mask = np.zeros_like(blue_mask)
        search_mask[y_min:y_max, :] = blue_mask[y_min:y_max, :]
        search_blue_px = int(np.count_nonzero(search_mask))
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (self.component_close_kernel_width_px, self.component_close_kernel_height_px),
        )
        merged_mask = cv2.morphologyEx(search_mask, cv2.MORPH_CLOSE, close_kernel)
        contours, _ = cv2.findContours(
            merged_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        if not contours:
            return (
                f'roi_y={y_min}:{y_max},search_blue_px={search_blue_px},'
                f'contours=0,reject=no_blue_component'
            )

        largest = max(contours, key=cv2.contourArea)
        area_px = float(cv2.contourArea(largest))
        x, y, w, h = cv2.boundingRect(largest)
        aspect_ratio = float(w) / max(float(h), 1.0)
        reject_reasons = []
        if area_px < self.min_component_area_px:
            reject_reasons.append('area')
        if w < self.min_component_width_px:
            reject_reasons.append('width')
        if h < self.min_component_height_px:
            reject_reasons.append('height')
        if h > self.max_component_height_px:
            reject_reasons.append('height_max')
        if aspect_ratio < self.min_component_aspect_ratio:
            reject_reasons.append('aspect')

        roi_x_min = max(0, x - self.component_expand_x_px)
        roi_x_max = min(width, x + w + self.component_expand_x_px)
        roi_y_min = max(0, y - self.component_expand_y_px)
        roi_y_max = min(height, y + h + self.component_expand_y_px)
        roi_blue = blue_mask[roi_y_min:roi_y_max, roi_x_min:roi_x_max] > 0
        roi_white = white_mask[roi_y_min:roi_y_max, roi_x_min:roi_x_max] > 0
        blue_pixels = int(np.count_nonzero(roi_blue))
        white_pixels = int(np.count_nonzero(roi_white))
        if blue_pixels < self.min_component_blue_pixels:
            reject_reasons.append('blue_px')
        if white_pixels < self.min_component_white_pixels:
            reject_reasons.append('white_px')
        if not reject_reasons:
            reject_reasons.append(
                self._depth_reject_summary(
                    roi_blue | roi_white,
                    roi_x_min,
                    roi_y_min,
                    roi_x_max,
                    roi_y_max,
                )
            )

        return (
            f'roi_y={y_min}:{y_max},search_blue_px={search_blue_px},contours={len(contours)},'
            f'largest=area:{area_px:.0f},box:{w}x{h},aspect:{aspect_ratio:.2f},'
            f'roi_blue:{blue_pixels},roi_white:{white_pixels},'
            f'reject:{"+".join(reject_reasons)}'
        )

    def _line_reject_summary(
        self,
        blue_mask: np.ndarray,
        white_mask: np.ndarray,
        combined_mask: np.ndarray,
    ) -> str:
        kernel = np.ones((self.morph_kernel_px, self.morph_kernel_px), dtype=np.uint8)
        blue_line_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)
        blue_line_mask = cv2.morphologyEx(blue_line_mask, cv2.MORPH_OPEN, kernel)
        blue_summary = self._line_mask_reject_summary('blue_only', blue_line_mask, blue_mask, white_mask)
        combined_summary = self._line_mask_reject_summary(
            'combined',
            combined_mask,
            blue_mask,
            white_mask,
        )
        return f'{blue_summary}; {combined_summary}'

    def _line_mask_reject_summary(
        self,
        label: str,
        line_mask: np.ndarray,
        blue_mask: np.ndarray,
        white_mask: np.ndarray,
    ) -> str:
        edges = cv2.Canny(line_mask, self.canny_low_threshold, self.canny_high_threshold)
        lines = cv2.HoughLinesP(
            edges,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length_px,
            maxLineGap=self.max_line_gap_px,
        )
        mask_px = int(np.count_nonzero(line_mask))
        if lines is None:
            return f'{label}(mask_px={mask_px},lines=0,reject=no_hough_line)'

        height, width = line_mask.shape[:2]
        representative = max(
            lines[:, 0, :],
            key=lambda line: math.hypot(float(line[2] - line[0]), float(line[3] - line[1])),
        )
        x1, y1, x2, y2 = [int(v) for v in representative]
        dx = float(x2 - x1)
        dy = float(y2 - y1)
        length_px = math.hypot(dx, dy)
        angle_deg = math.degrees(math.atan2(dy, dx))
        reject_reasons = []
        if length_px < self.min_line_length_px:
            reject_reasons.append('length')
        if abs(angle_deg) > self.horizontal_angle_tolerance_deg:
            reject_reasons.append('angle')

        x_min = max(0, min(x1, x2) - self.line_band_horizontal_pad_px)
        x_max = min(width, max(x1, x2) + self.line_band_horizontal_pad_px + 1)
        y_min = max(0, min(y1, y2) - self.line_band_half_height_px)
        y_max = min(height, max(y1, y2) + self.line_band_half_height_px + 1)
        roi_blue = blue_mask[y_min:y_max, x_min:x_max] > 0
        roi_white = white_mask[y_min:y_max, x_min:x_max] > 0
        blue_pixels = int(np.count_nonzero(roi_blue))
        white_pixels = int(np.count_nonzero(roi_white))
        if blue_pixels < self.min_blue_pixels:
            reject_reasons.append('blue_px')
        if white_pixels < self.min_white_pixels:
            reject_reasons.append('white_px')
        if not reject_reasons:
            reject_reasons.append(
                self._depth_reject_summary(
                    roi_blue | roi_white,
                    x_min,
                    y_min,
                    x_max,
                    y_max,
                )
            )

        return (
            f'{label}(mask_px={mask_px},lines={len(lines)},'
            f'best=len:{length_px:.0f},angle:{angle_deg:.1f},band:{x_max - x_min}x{y_max - y_min},'
            f'blue:{blue_pixels},white:{white_pixels},reject:{"+".join(reject_reasons)})'
        )

    def _depth_reject_summary(
        self,
        roi_combined: np.ndarray,
        x_min: int,
        y_min: int,
        x_max: int,
        y_max: int,
    ) -> str:
        if self.latest_depth is None:
            return 'depth_missing'
        roi_depth = self.latest_depth[y_min:y_max, x_min:x_max]
        if roi_depth.size == 0:
            return 'depth_empty_roi'

        depth_m = self._depth_to_meters(roi_depth)
        valid = (
            roi_combined &
            np.isfinite(depth_m) &
            (depth_m >= self.min_depth_m) &
            (depth_m <= self.max_depth_m)
        )
        valid_depth_count = int(np.count_nonzero(valid))
        if valid_depth_count < self.min_valid_depth_pixels:
            return f'depth_invalid(valid_px={valid_depth_count},min_px={self.min_valid_depth_pixels})'

        selected_depth = depth_m[valid]
        raw_distance_m = float(np.percentile(selected_depth, self.depth_percentile))
        distance_m = raw_distance_m + self.distance_bias_m
        if not math.isfinite(distance_m):
            return 'depth_nan'
        return f'depth_ok(valid_px={valid_depth_count},dist={distance_m:.3f})'

    def _data_is_stale(self, now: float) -> bool:
        depth_age = (
            now - self.latest_depth_stamp_sec
            if self.latest_depth_stamp_sec is not None
            else math.inf
        )
        if depth_age > self.depth_stale_timeout_sec:
            self._log_periodic(f'深度图超时: age={depth_age:.2f}s', now, level='warn')
            return True

        info_age = (
            now - self.latest_camera_info_stamp_sec
            if self.latest_camera_info_stamp_sec is not None
            else math.inf
        )
        if info_age > self.camera_info_stale_timeout_sec:
            self._log_periodic(f'camera_info 超时: age={info_age:.2f}s', now, level='warn')
            return True
        return False

    def _clear_detection(self) -> None:
        self.latest_candidate = None
        self.latest_candidate_stamp_sec = None
        self.current_triggered = False

    def _set_state(self, state: str) -> None:
        if state == self.state:
            return
        self.state = state
        self._publish_state(state)

    def _publish_state(self, state: str) -> None:
        if state == self.last_state:
            return
        self.last_state = state
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
        self.get_logger().info(f'state={state}')

    def _log_periodic(self, text: str, now: float, level: str = 'info') -> None:
        if (now - self.last_log_time) < self.log_interval_sec:
            return
        self.last_log_time = now
        self._publish_feedback(text, level=level)

    def _publish_feedback(self, text: str, level: str = 'info') -> None:
        if text == self.last_feedback:
            return
        self.last_feedback = text
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)
        if level == 'warn':
            self.get_logger().warn(colorize_log(text, level=level))
        else:
            self.get_logger().info(colorize_log(text, level=level))

    def _int_list_param(self, name: str, expected_len: int) -> List[int]:
        values = [int(item) for item in self.get_parameter(name).value]
        if len(values) != expected_len:
            raise RuntimeError(f'{name} must contain {expected_len} integer values.')
        return values

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def destroy_node(self) -> bool:
        if self.serial_sender is not None:
            self.serial_sender.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LimitBarDuckTest()
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
