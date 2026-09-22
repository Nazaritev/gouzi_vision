#!/usr/bin/env python3
"""HSV+depth orange hurdle detector with a timed jump command sequence.

2026-05-05:
Purpose: provide a standalone test node for the orange horizontal bar obstacle.
The node continuously sends a walk override, then switches to a timed
stand->jump->walk sequence when a horizontal orange bar is detected within the
configured depth threshold.
Rollback: remove this node from setup.py and use direct serial_step_tester.py
or a manual ros2 topic pub workflow instead.
"""

import math
import os
import termios
from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from auto_nav_pkg.log_style import colorize_log
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32MultiArray, String


@dataclass
class HurdleCandidate:
    distance_m: float
    lateral_m: float
    vertical_m: float
    pixel_x: float
    pixel_y: float
    width_px: int
    height_px: int
    area_px: float
    valid_depth_count: int


BAUD_MAP = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: termios.B230400,
}


class RawSerialSender:
    def __init__(self, device: str, baud_rate: int) -> None:
        self.device = device
        self.baud_rate = baud_rate
        self.fd: Optional[int] = None

    def ensure_open(self) -> None:
        if self.fd is not None:
            return
        if self.baud_rate not in BAUD_MAP:
            raise ValueError(f'Unsupported baud_rate={self.baud_rate}')

        self.fd = os.open(self.device, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] &= ~(
            termios.IGNBRK | termios.ICRNL | termios.INLCR | termios.IXON | termios.IXOFF | termios.IXANY
        )
        attrs[1] &= ~(termios.OPOST | termios.ONLCR | termios.OCRNL)
        attrs[2] &= ~termios.CSIZE
        attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[2] &= ~(termios.PARENB | termios.PARODD | termios.CSTOPB | termios.CRTSCTS)
        attrs[3] = 0
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 5
        speed = BAUD_MAP[self.baud_rate]
        if hasattr(termios, 'cfsetispeed') and hasattr(termios, 'cfsetospeed'):
            termios.cfsetispeed(attrs, speed)
            termios.cfsetospeed(attrs, speed)
        else:
            attrs[4] = speed
            attrs[5] = speed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def send(self, payload: bytes) -> None:
        self.ensure_open()
        total = 0
        while total < len(payload):
            written = os.write(self.fd, payload[total:])
            if written <= 0:
                raise RuntimeError('Serial write returned no data')
            total += written


class OrangeHurdleJumpTest(Node):
    """Detect an orange horizontal hurdle and run a fixed jump sequence."""

    STATE_WALKING = 'WALKING'
    STATE_WAIT_JUMP = 'WAIT_JUMP'
    STATE_WAIT_RESUME = 'WAIT_RESUME'
    STATE_WALKING_DONE = 'WALKING_DONE'

    def __init__(self) -> None:
        super().__init__('orange_hurdle_jump_test')

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('nearest_topic', '/orange_hurdle_jump_test/nearest')
        self.declare_parameter('trigger_topic', '/orange_hurdle_jump_test/trigger')
        self.declare_parameter('feedback_topic', '/orange_hurdle_jump_test/feedback_log')
        self.declare_parameter('state_topic', '/orange_hurdle_jump_test/state')
        self.declare_parameter('debug_image_topic', '/orange_hurdle_jump_test/debug_image')
        self.declare_parameter('detector_only', False)
        self.declare_parameter('serial_device', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('output_precision', 3)
        self.declare_parameter('message_prefix', '[')
        self.declare_parameter('field_separator', ',')
        self.declare_parameter('message_suffix', ']\n')

        self.declare_parameter('hsv_lower', [5, 70, 70])
        self.declare_parameter('hsv_upper', [35, 255, 255])
        self.declare_parameter('min_area_px', 180.0)
        self.declare_parameter('min_width_px', 40)
        self.declare_parameter('min_height_px', 8)
        self.declare_parameter('max_height_px', 220)
        self.declare_parameter('min_horizontal_aspect_ratio', 2.2)
        self.declare_parameter('large_foreground_enabled', False)
        self.declare_parameter('large_foreground_min_area_ratio', 0.08)
        self.declare_parameter('large_foreground_min_width_ratio', 0.35)
        self.declare_parameter('large_foreground_min_height_ratio', 0.18)
        self.declare_parameter('large_foreground_min_bottom_y_ratio', 0.75)
        self.declare_parameter('large_foreground_use_top_edge_filter', False)
        self.declare_parameter('morph_kernel_px', 5)
        self.declare_parameter('depth_units_divisor', 1000.0)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 2.00)
        self.declare_parameter('min_valid_depth_pixels', 12)
        self.declare_parameter('depth_mask_erode_px', 3)
        self.declare_parameter('depth_percentile', 25.0)
        self.declare_parameter('distance_bias_m', 0.0)
        self.declare_parameter('trigger_distance_m', 0.30)
        self.declare_parameter('depth_stale_timeout_sec', 0.50)
        self.declare_parameter('camera_info_stale_timeout_sec', 5.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('log_interval_sec', 1.0)
        self.declare_parameter('use_top_edge_filter', False)
        self.declare_parameter('expected_top_edge_vertical_m', 0.0)
        self.declare_parameter('top_edge_vertical_tolerance_m', 0.10)

        self.declare_parameter('walk_publish_hz', 10.0)
        self.declare_parameter('stand_to_jump_delay_sec', 2.0)
        self.declare_parameter('jump_to_walk_delay_sec', 5.0)
        self.declare_parameter('retrigger_enabled', False)

        self.declare_parameter('walk_mode', 0)
        self.declare_parameter('walk_left_norm', 1.0)
        self.declare_parameter('walk_right_norm', 1.0)
        self.declare_parameter('stand_mode', 0)
        self.declare_parameter('stand_left_norm', 0.0)
        self.declare_parameter('stand_right_norm', 0.0)
        self.declare_parameter('jump_mode', 4)
        self.declare_parameter('jump_left_norm', 0.0)
        self.declare_parameter('jump_right_norm', 0.0)

        self.color_topic = str(self.get_parameter('color_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
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

        self.hsv_lower = np.array(self._int_list_param('hsv_lower', 3), dtype=np.uint8)
        self.hsv_upper = np.array(self._int_list_param('hsv_upper', 3), dtype=np.uint8)
        self.min_area_px = float(self.get_parameter('min_area_px').value)
        self.min_width_px = int(self.get_parameter('min_width_px').value)
        self.min_height_px = int(self.get_parameter('min_height_px').value)
        self.max_height_px = int(self.get_parameter('max_height_px').value)
        self.min_horizontal_aspect_ratio = float(
            self.get_parameter('min_horizontal_aspect_ratio').value
        )
        self.large_foreground_enabled = bool(self.get_parameter('large_foreground_enabled').value)
        self.large_foreground_min_area_ratio = max(
            0.0, float(self.get_parameter('large_foreground_min_area_ratio').value)
        )
        self.large_foreground_min_width_ratio = max(
            0.0, float(self.get_parameter('large_foreground_min_width_ratio').value)
        )
        self.large_foreground_min_height_ratio = max(
            0.0, float(self.get_parameter('large_foreground_min_height_ratio').value)
        )
        self.large_foreground_min_bottom_y_ratio = max(
            0.0, float(self.get_parameter('large_foreground_min_bottom_y_ratio').value)
        )
        self.large_foreground_use_top_edge_filter = bool(
            self.get_parameter('large_foreground_use_top_edge_filter').value
        )
        self.morph_kernel_px = max(1, int(self.get_parameter('morph_kernel_px').value))
        self.depth_units_divisor = float(self.get_parameter('depth_units_divisor').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.min_valid_depth_pixels = int(self.get_parameter('min_valid_depth_pixels').value)
        self.depth_mask_erode_px = max(0, int(self.get_parameter('depth_mask_erode_px').value))
        self.depth_percentile = float(
            np.clip(float(self.get_parameter('depth_percentile').value), 0.0, 100.0)
        )
        self.distance_bias_m = float(self.get_parameter('distance_bias_m').value)
        self.trigger_distance_m = float(self.get_parameter('trigger_distance_m').value)
        self.depth_stale_timeout_sec = float(self.get_parameter('depth_stale_timeout_sec').value)
        self.camera_info_stale_timeout_sec = float(
            self.get_parameter('camera_info_stale_timeout_sec').value
        )
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)
        self.use_top_edge_filter = bool(self.get_parameter('use_top_edge_filter').value)
        self.expected_top_edge_vertical_m = float(
            self.get_parameter('expected_top_edge_vertical_m').value
        )
        self.top_edge_vertical_tolerance_m = max(
            0.0,
            float(self.get_parameter('top_edge_vertical_tolerance_m').value),
        )

        self.walk_publish_hz = max(1.0, float(self.get_parameter('walk_publish_hz').value))
        self.stand_to_jump_delay_sec = max(
            0.0, float(self.get_parameter('stand_to_jump_delay_sec').value)
        )
        self.jump_to_walk_delay_sec = max(
            0.0, float(self.get_parameter('jump_to_walk_delay_sec').value)
        )
        self.retrigger_enabled = bool(self.get_parameter('retrigger_enabled').value)

        self.walk_mode = int(self.get_parameter('walk_mode').value)
        self.walk_left_norm = float(self.get_parameter('walk_left_norm').value)
        self.walk_right_norm = float(self.get_parameter('walk_right_norm').value)
        self.stand_mode = int(self.get_parameter('stand_mode').value)
        self.stand_left_norm = float(self.get_parameter('stand_left_norm').value)
        self.stand_right_norm = float(self.get_parameter('stand_right_norm').value)
        self.jump_mode = int(self.get_parameter('jump_mode').value)
        self.jump_left_norm = float(self.get_parameter('jump_left_norm').value)
        self.jump_right_norm = float(self.get_parameter('jump_right_norm').value)

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

        self.latest_candidate: Optional[HurdleCandidate] = None
        self.latest_candidate_stamp_sec: Optional[float] = None
        self.current_triggered = False
        self.serial_sender: Optional[RawSerialSender] = None
        if not self.detector_only:
            self.serial_sender = RawSerialSender(self.serial_device, self.baud_rate)

        self.jump_started = False
        self.state = 'DETECTOR_ONLY' if self.detector_only else self.STATE_WALKING
        self.phase_deadline_sec: Optional[float] = None
        self.last_serial_payload = ''
        self.serial_opened = False

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
        self.command_timer = None
        if not self.detector_only:
            self.command_timer = self.create_timer(1.0 / self.walk_publish_hz, self._command_timer)

        self._publish_state(self.state)
        self._publish_feedback(
            'orange_hurdle_jump_test started: '
            f'color={self.color_topic}, depth={self.depth_topic}, '
            f'detector_only={self.detector_only}, '
            f'serial={self.serial_device}@{self.baud_rate}, '
            f'trigger_distance={self.trigger_distance_m:.2f}m, '
            f'walk=[{self.walk_mode},{self.walk_left_norm:.3f},{self.walk_right_norm:.3f}], '
            f'stand=[{self.stand_mode},{self.stand_left_norm:.3f},{self.stand_right_norm:.3f}], '
            f'jump=[{self.jump_mode},{self.jump_left_norm:.3f},{self.jump_right_norm:.3f}]'
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

        mask = self._build_mask(bgr)
        candidates = self._extract_candidates(mask)
        nearest = min(candidates, key=lambda item: item.distance_m) if candidates else None
        triggered = nearest is not None and nearest.distance_m <= self.trigger_distance_m

        self.latest_candidate = nearest
        self.latest_candidate_stamp_sec = now if nearest is not None else None
        self.current_triggered = triggered

        self._publish_trigger(triggered)
        if nearest is not None:
            self._publish_nearest(nearest)

        if self.publish_debug_image:
            self._publish_debug_image(bgr, mask, candidates, nearest, msg.header)

        self._log_detection(now, mask, candidates, nearest, triggered)

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

    def _advance_state_machine(self, now: float) -> None:
        if self.state in (self.STATE_WALKING, self.STATE_WALKING_DONE):
            if self._should_start_jump(now):
                self.jump_started = True
                self.phase_deadline_sec = now + self.stand_to_jump_delay_sec
                self._send_serial_command(
                    self.stand_mode,
                    self.stand_left_norm,
                    self.stand_right_norm,
                    log_payload_change=True,
                )
                self._set_state(self.STATE_WAIT_JUMP)
                candidate = self.latest_candidate
                if candidate is not None:
                    self._publish_feedback(
                        f'触发横栏跳跃: dist={candidate.distance_m:.3f}m, '
                        f'box={candidate.width_px}x{candidate.height_px}, '
                        f'depth_px={candidate.valid_depth_count}; 单发站立指令'
                    )
                else:
                    self._publish_feedback('触发横栏跳跃: 单发站立指令')
                return
            return

        if self.state == self.STATE_WAIT_JUMP:
            if self.phase_deadline_sec is None or now < self.phase_deadline_sec:
                return
            self.phase_deadline_sec = now + self.jump_to_walk_delay_sec
            self._send_serial_command(
                self.jump_mode,
                self.jump_left_norm,
                self.jump_right_norm,
                log_payload_change=True,
            )
            self._set_state(self.STATE_WAIT_RESUME)
            self._publish_feedback(
                f'等待结束，单发跳栏指令: mode={self.jump_mode}, '
                f'left={self.jump_left_norm:.3f}, right={self.jump_right_norm:.3f}'
            )
            return

        if self.state == self.STATE_WAIT_RESUME:
            if self.phase_deadline_sec is None or now < self.phase_deadline_sec:
                return
            self.phase_deadline_sec = None
            next_state = self.STATE_WALKING if self.retrigger_enabled else self.STATE_WALKING_DONE
            self._set_state(next_state)
            self._publish_feedback(
                f'跳栏阶段结束，恢复正常走: mode={self.walk_mode}, '
                f'left={self.walk_left_norm:.3f}, right={self.walk_right_norm:.3f}'
            )

    def _should_start_jump(self, now: float) -> bool:
        if self.current_triggered is False:
            return False
        if self.latest_candidate is None:
            return False
        if self.latest_candidate_stamp_sec is None:
            return False
        if (now - self.latest_candidate_stamp_sec) > self.depth_stale_timeout_sec:
            return False
        if self.jump_started and not self.retrigger_enabled:
            return False
        return True

    def _build_mask(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        kernel = np.ones((self.morph_kernel_px, self.morph_kernel_px), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _extract_candidates(self, mask: np.ndarray) -> List[HurdleCandidate]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates: List[HurdleCandidate] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area_px:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            large_foreground = self._is_large_foreground_candidate(mask, x, y, w, h, area)
            if w < self.min_width_px or h < self.min_height_px:
                continue
            if not large_foreground and h > self.max_height_px:
                continue

            aspect_ratio = w / max(float(h), 1.0)
            if not large_foreground and aspect_ratio < self.min_horizontal_aspect_ratio:
                continue

            check_top_edge = self.use_top_edge_filter
            if large_foreground:
                check_top_edge = self.large_foreground_use_top_edge_filter
            candidate = self._candidate_from_roi(mask, x, y, w, h, area, check_top_edge)
            if candidate is not None:
                candidates.append(candidate)
        return candidates

    def _is_large_foreground_candidate(
        self,
        mask: np.ndarray,
        x: int,
        y: int,
        w: int,
        h: int,
        area: float,
    ) -> bool:
        if not self.large_foreground_enabled:
            return False
        image_h, image_w = mask.shape[:2]
        image_area = max(float(image_h * image_w), 1.0)
        if (area / image_area) < self.large_foreground_min_area_ratio:
            return False
        if (float(w) / max(float(image_w), 1.0)) < self.large_foreground_min_width_ratio:
            return False
        if (float(h) / max(float(image_h), 1.0)) < self.large_foreground_min_height_ratio:
            return False
        bottom_ratio = float(y + h) / max(float(image_h), 1.0)
        if bottom_ratio < self.large_foreground_min_bottom_y_ratio:
            return False
        return True

    def _candidate_from_roi(
        self,
        mask: np.ndarray,
        x: int,
        y: int,
        w: int,
        h: int,
        area: float,
        check_top_edge: bool,
    ) -> Optional[HurdleCandidate]:
        roi_depth = self.latest_depth[y:y + h, x:x + w]
        roi_mask = mask[y:y + h, x:x + w] > 0
        if roi_depth.size == 0:
            return None

        depth_m = self._depth_to_meters(roi_depth)
        depth_roi_mask = roi_mask
        if self.depth_mask_erode_px > 0:
            erode_kernel = np.ones(
                (self.depth_mask_erode_px, self.depth_mask_erode_px),
                dtype=np.uint8,
            )
            eroded = cv2.erode(roi_mask.astype(np.uint8), erode_kernel, iterations=1) > 0
            if int(np.count_nonzero(eroded)) >= self.min_valid_depth_pixels:
                depth_roi_mask = eroded

        valid = (
            depth_roi_mask &
            np.isfinite(depth_m) &
            (depth_m >= self.min_depth_m) &
            (depth_m <= self.max_depth_m)
        )
        if int(np.count_nonzero(valid)) < self.min_valid_depth_pixels:
            return None

        selected_depth = depth_m[valid]
        raw_distance_m = float(np.percentile(selected_depth, self.depth_percentile))
        distance_m = raw_distance_m + self.distance_bias_m
        if not math.isfinite(distance_m):
            return None

        moments = cv2.moments(roi_mask.astype(np.uint8))
        if abs(moments['m00']) > 1e-6:
            pixel_x = float(x + moments['m10'] / moments['m00'])
            pixel_y = float(y + moments['m01'] / moments['m00'])
        else:
            pixel_x = float(x + w * 0.5)
            pixel_y = float(y + h * 0.5)

        lateral_m, vertical_m = self._project_pixel(pixel_x, pixel_y, distance_m)
        if check_top_edge:
            top_edge_vertical_m = self._project_pixel(float(x + 0.5 * w), float(y), distance_m)[1]
            if (
                abs(top_edge_vertical_m - self.expected_top_edge_vertical_m)
                > self.top_edge_vertical_tolerance_m
            ):
                return None
        return HurdleCandidate(
            distance_m=distance_m,
            lateral_m=lateral_m,
            vertical_m=vertical_m,
            pixel_x=pixel_x,
            pixel_y=pixel_y,
            width_px=w,
            height_px=h,
            area_px=area,
            valid_depth_count=int(np.count_nonzero(valid)),
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

    def _publish_nearest(self, candidate: HurdleCandidate) -> None:
        msg = Float32MultiArray()
        msg.data = [
            float(candidate.distance_m),
            float(candidate.lateral_m),
            float(candidate.vertical_m),
            float(candidate.pixel_x),
            float(candidate.pixel_y),
            float(candidate.width_px),
            float(candidate.height_px),
            float(candidate.area_px),
            float(candidate.valid_depth_count),
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
        mask: np.ndarray,
        candidates: List[HurdleCandidate],
        nearest: Optional[HurdleCandidate],
        header,
    ) -> None:
        debug = bgr.copy()
        for candidate in candidates:
            x = int(candidate.pixel_x - candidate.width_px * 0.5)
            y = int(candidate.pixel_y - candidate.height_px * 0.5)
            color = (0, 255, 0) if candidate is nearest else (255, 0, 0)
            cv2.rectangle(
                debug,
                (x, y),
                (x + candidate.width_px, y + candidate.height_px),
                color,
                2,
            )
            cv2.putText(
                debug,
                f'{candidate.distance_m:.2f}m',
                (x, max(y - 8, 16)),
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
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = cv2.addWeighted(debug, 0.75, mask_bgr, 0.25, 0.0)
        out = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
        out.header = header
        self.debug_image_pub.publish(out)

    def _log_detection(
        self,
        now: float,
        mask: np.ndarray,
        candidates: List[HurdleCandidate],
        nearest: Optional[HurdleCandidate],
        triggered: bool,
    ) -> None:
        if (now - self.last_log_time) < self.log_interval_sec:
            return
        self.last_log_time = now

        if nearest is None:
            self._publish_feedback(
                f'未检测到有效橘色横栏: candidates=0, frame={self.frame_count}, '
                f'{self._mask_reject_summary(mask)}, '
                f'hsv_lower={self.hsv_lower.tolist()}, hsv_upper={self.hsv_upper.tolist()}'
            )
            return

        self._publish_feedback(
            f'最近横栏: dist={nearest.distance_m:.3f}m, lateral={nearest.lateral_m:.3f}m, '
            f'vertical={nearest.vertical_m:.3f}m, pixel=({nearest.pixel_x:.1f},{nearest.pixel_y:.1f}), '
            f'box={nearest.width_px}x{nearest.height_px}, depth_px={nearest.valid_depth_count}, '
            f'candidates={len(candidates)}, trigger={triggered}'
        )

    def _mask_reject_summary(self, mask: np.ndarray) -> str:
        mask_px = int(np.count_nonzero(mask))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return f'mask_px={mask_px}, contours=0, reject=no_hsv_blob'

        largest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest))
        x, y, w, h = cv2.boundingRect(largest)
        aspect_ratio = w / max(float(h), 1.0)

        reject_reasons = []
        if area < self.min_area_px:
            reject_reasons.append('area')
        if w < self.min_width_px:
            reject_reasons.append('width')
        if h < self.min_height_px:
            reject_reasons.append('height')
        if h > self.max_height_px:
            reject_reasons.append('height_max')
        if aspect_ratio < self.min_horizontal_aspect_ratio:
            reject_reasons.append('aspect')
        if self._is_large_foreground_candidate(mask, x, y, w, h, area):
            reject_reasons = [
                reason
                for reason in reject_reasons
                if reason not in ('height_max', 'aspect')
            ]
            check_top_edge = self.large_foreground_use_top_edge_filter
        else:
            check_top_edge = self.use_top_edge_filter
        if not reject_reasons:
            reject_reasons.append(self._candidate_reject_summary(mask, x, y, w, h, check_top_edge))

        return (
            f'mask_px={mask_px}, contours={len(contours)}, '
            f'largest=area:{area:.0f},box:{w}x{h},aspect:{aspect_ratio:.2f},'
            f'reject:{"+".join(reject_reasons)}'
        )

    def _candidate_reject_summary(
        self,
        mask: np.ndarray,
        x: int,
        y: int,
        w: int,
        h: int,
        check_top_edge: bool,
    ) -> str:
        if self.latest_depth is None:
            return 'depth_missing'

        roi_depth = self.latest_depth[y:y + h, x:x + w]
        roi_mask = mask[y:y + h, x:x + w] > 0
        if roi_depth.size == 0:
            return 'depth_empty_roi'

        depth_m = self._depth_to_meters(roi_depth)
        depth_roi_mask = roi_mask
        if self.depth_mask_erode_px > 0:
            erode_kernel = np.ones(
                (self.depth_mask_erode_px, self.depth_mask_erode_px),
                dtype=np.uint8,
            )
            eroded = cv2.erode(roi_mask.astype(np.uint8), erode_kernel, iterations=1) > 0
            if int(np.count_nonzero(eroded)) >= self.min_valid_depth_pixels:
                depth_roi_mask = eroded

        valid = (
            depth_roi_mask &
            np.isfinite(depth_m) &
            (depth_m >= self.min_depth_m) &
            (depth_m <= self.max_depth_m)
        )
        valid_depth_count = int(np.count_nonzero(valid))
        if valid_depth_count < self.min_valid_depth_pixels:
            return f'depth_px:{valid_depth_count}'

        selected_depth = depth_m[valid]
        raw_distance_m = float(np.percentile(selected_depth, self.depth_percentile))
        distance_m = raw_distance_m + self.distance_bias_m
        if not math.isfinite(distance_m):
            return 'depth_nan'

        if check_top_edge:
            top_edge_vertical_m = self._project_pixel(float(x + 0.5 * w), float(y), distance_m)[1]
            top_edge_error_m = top_edge_vertical_m - self.expected_top_edge_vertical_m
            if abs(top_edge_error_m) > self.top_edge_vertical_tolerance_m:
                return (
                    f'top_edge:{top_edge_vertical_m:.3f}m'
                    f'(err={top_edge_error_m:.3f})'
                )

        return 'unknown'

    def _data_is_stale(self, now: float) -> bool:
        depth_age = now - self.latest_depth_stamp_sec if self.latest_depth_stamp_sec is not None else math.inf
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
    node = OrangeHurdleJumpTest()
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
