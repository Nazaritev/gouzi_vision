#!/usr/bin/env python3
"""HSV+depth green ramp detector for the randomized obstacle-race supervisor."""

import math
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
class SlopeCandidate:
    distance_m: float
    lateral_m: float
    vertical_m: float
    pixel_x: float
    pixel_y: float
    width_px: int
    height_px: int
    area_px: float
    fill_ratio: float
    valid_depth_count: int
    bottom_gap_px: int


class SlopeBranchDetector(Node):
    """Detect the green uphill ramp used by the slope/bridge branch."""

    def __init__(self) -> None:
        super().__init__('slope_branch_detector')

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        self.declare_parameter('nearest_topic', '/slope_branch_detector/nearest')
        self.declare_parameter('trigger_topic', '/slope_branch_detector/trigger')
        self.declare_parameter('feedback_topic', '/slope_branch_detector/feedback_log')
        self.declare_parameter('state_topic', '/slope_branch_detector/state')
        self.declare_parameter('debug_image_topic', '/slope_branch_detector/debug_image')

        self.declare_parameter('hsv_lower', [35, 40, 40])
        self.declare_parameter('hsv_upper', [95, 255, 255])
        self.declare_parameter('morph_kernel_px', 7)
        self.declare_parameter('morph_open_kernel_px', 0)
        self.declare_parameter('morph_close_kernel_px', 0)
        self.declare_parameter('morph_close_iterations', 1)
        self.declare_parameter('search_y_min_px', 150)
        self.declare_parameter('search_y_max_px', 470)
        self.declare_parameter('min_area_px', 16000.0)
        self.declare_parameter('min_width_px', 240)
        self.declare_parameter('min_height_px', 90)
        self.declare_parameter('min_aspect_ratio', 1.2)
        self.declare_parameter('min_fill_ratio', 0.35)
        self.declare_parameter('max_fill_ratio', 1.0)
        self.declare_parameter('max_bottom_gap_px', 70)
        self.declare_parameter('max_abs_lateral_m', 1.20)
        self.declare_parameter('depth_mask_erode_px', 3)
        self.declare_parameter('depth_fallback_enabled', True)
        self.declare_parameter('depth_fallback_y_min_ratio', 0.45)
        self.declare_parameter('depth_fallback_y_max_ratio', 0.95)
        self.declare_parameter('depth_fallback_x_margin_ratio', 0.15)
        self.declare_parameter('min_valid_depth_pixels', 24)
        self.declare_parameter('depth_units_divisor', 1000.0)
        self.declare_parameter('min_depth_m', 0.20)
        self.declare_parameter('max_depth_m', 3.50)
        self.declare_parameter('depth_percentile', 20.0)
        self.declare_parameter('distance_bias_m', 0.0)
        self.declare_parameter('trigger_distance_m', 1.20)
        self.declare_parameter('trigger_hold_sec', 0.0)
        self.declare_parameter('depth_stale_timeout_sec', 0.50)
        self.declare_parameter('camera_info_stale_timeout_sec', 5.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('log_interval_sec', 1.0)

        self.color_topic = str(self.get_parameter('color_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)

        self.nearest_topic = str(self.get_parameter('nearest_topic').value)
        self.trigger_topic = str(self.get_parameter('trigger_topic').value)
        self.feedback_topic = str(self.get_parameter('feedback_topic').value)
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)

        self.hsv_lower = np.array(self._int_list_param('hsv_lower', 3), dtype=np.uint8)
        self.hsv_upper = np.array(self._int_list_param('hsv_upper', 3), dtype=np.uint8)
        self.morph_kernel_px = max(1, int(self.get_parameter('morph_kernel_px').value))
        self.morph_open_kernel_px = max(0, int(self.get_parameter('morph_open_kernel_px').value))
        self.morph_close_kernel_px = max(0, int(self.get_parameter('morph_close_kernel_px').value))
        self.morph_close_iterations = max(1, int(self.get_parameter('morph_close_iterations').value))
        self.search_y_min_px = max(0, int(self.get_parameter('search_y_min_px').value))
        self.search_y_max_px = max(0, int(self.get_parameter('search_y_max_px').value))
        self.min_area_px = max(1.0, float(self.get_parameter('min_area_px').value))
        self.min_width_px = max(1, int(self.get_parameter('min_width_px').value))
        self.min_height_px = max(1, int(self.get_parameter('min_height_px').value))
        self.min_aspect_ratio = max(1.0, float(self.get_parameter('min_aspect_ratio').value))
        self.min_fill_ratio = float(np.clip(float(self.get_parameter('min_fill_ratio').value), 0.0, 1.0))
        self.max_fill_ratio = max(
            self.min_fill_ratio,
            float(np.clip(float(self.get_parameter('max_fill_ratio').value), 0.0, 1.0)),
        )
        self.max_bottom_gap_px = max(0, int(self.get_parameter('max_bottom_gap_px').value))
        self.max_abs_lateral_m = max(0.0, float(self.get_parameter('max_abs_lateral_m').value))
        self.depth_mask_erode_px = max(0, int(self.get_parameter('depth_mask_erode_px').value))
        self.depth_fallback_enabled = bool(self.get_parameter('depth_fallback_enabled').value)
        self.depth_fallback_y_min_ratio = float(
            np.clip(float(self.get_parameter('depth_fallback_y_min_ratio').value), 0.0, 1.0)
        )
        self.depth_fallback_y_max_ratio = float(
            np.clip(float(self.get_parameter('depth_fallback_y_max_ratio').value), 0.0, 1.0)
        )
        self.depth_fallback_x_margin_ratio = float(
            np.clip(float(self.get_parameter('depth_fallback_x_margin_ratio').value), 0.0, 0.45)
        )
        self.min_valid_depth_pixels = max(1, int(self.get_parameter('min_valid_depth_pixels').value))
        self.depth_units_divisor = float(self.get_parameter('depth_units_divisor').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.depth_percentile = float(
            np.clip(float(self.get_parameter('depth_percentile').value), 0.0, 100.0)
        )
        self.distance_bias_m = float(self.get_parameter('distance_bias_m').value)
        self.trigger_distance_m = float(self.get_parameter('trigger_distance_m').value)
        self.trigger_hold_sec = max(0.0, float(self.get_parameter('trigger_hold_sec').value))
        self.depth_stale_timeout_sec = float(self.get_parameter('depth_stale_timeout_sec').value)
        self.camera_info_stale_timeout_sec = float(
            self.get_parameter('camera_info_stale_timeout_sec').value
        )
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)

        self.bridge = CvBridge()
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_depth_stamp_sec: Optional[float] = None
        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_camera_info_stamp_sec: Optional[float] = None
        self.latest_candidate: Optional[SlopeCandidate] = None
        self.latest_candidate_stamp_sec: Optional[float] = None
        self.current_triggered = False
        self.last_raw_trigger_stamp_sec = 0.0
        self.last_trigger_state: Optional[bool] = None
        self.last_feedback = ''
        self.last_state = ''
        self.last_log_time = 0.0
        self.frame_count = 0

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

        self._publish_state('DETECTING')
        self._publish_feedback(
            f'slope_branch_detector started: color={self.color_topic}, depth={self.depth_topic}, '
            f'trigger_distance={self.trigger_distance_m:.2f}m, '
            f'hsv={self.hsv_lower.tolist()}-{self.hsv_upper.tolist()}, '
            f'fill_range=[{self.min_fill_ratio:.2f},{self.max_fill_ratio:.2f}]'
        )

    def _depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self._publish_feedback(f'深度图转换失败: {exc}', level='warn')
            return
        self.latest_depth = np.asarray(depth)
        self.latest_depth_stamp_sec = self._now_sec()

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg
        self.latest_camera_info_stamp_sec = self._now_sec()

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
        raw_triggered = nearest is not None and nearest.distance_m <= self.trigger_distance_m
        if raw_triggered:
            self.last_raw_trigger_stamp_sec = now
        triggered = raw_triggered or (
            self.trigger_hold_sec > 0.0
            and self.last_raw_trigger_stamp_sec > 0.0
            and (now - self.last_raw_trigger_stamp_sec) <= self.trigger_hold_sec
        )

        self.latest_candidate = nearest
        self.latest_candidate_stamp_sec = now if nearest is not None else None
        self.current_triggered = triggered

        self._publish_trigger(triggered)
        if nearest is not None:
            self._publish_nearest(nearest)

        if self.publish_debug_image:
            self._publish_debug_image(bgr, mask, candidates, nearest, msg.header)

        self._log_detection(now, mask, candidates, nearest, triggered)

    def _build_mask(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        open_kernel_px = self.morph_open_kernel_px or self.morph_kernel_px
        close_kernel_px = self.morph_close_kernel_px or self.morph_kernel_px
        if open_kernel_px > 1:
            open_kernel = np.ones((open_kernel_px, open_kernel_px), dtype=np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        if close_kernel_px > 1:
            close_kernel = np.ones((close_kernel_px, close_kernel_px), dtype=np.uint8)
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                close_kernel,
                iterations=self.morph_close_iterations,
            )
        height = mask.shape[0]
        y_min = int(np.clip(self.search_y_min_px, 0, max(height - 1, 0)))
        y_max = int(np.clip(self.search_y_max_px, y_min + 1, height))
        filtered = np.zeros_like(mask)
        filtered[y_min:y_max, :] = mask[y_min:y_max, :]
        return filtered

    def _extract_candidates(self, mask: np.ndarray) -> List[SlopeCandidate]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates: List[SlopeCandidate] = []
        image_height = mask.shape[0]
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area_px:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if w < self.min_width_px or h < self.min_height_px:
                continue

            aspect_ratio = w / max(float(h), 1.0)
            if aspect_ratio < self.min_aspect_ratio:
                continue

            fill_ratio = area / max(float(w * h), 1.0)
            if fill_ratio < self.min_fill_ratio:
                continue
            if fill_ratio > self.max_fill_ratio:
                continue

            bottom_gap_px = max(0, image_height - (y + h))
            if bottom_gap_px > self.max_bottom_gap_px:
                continue

            candidate = self._candidate_from_roi(mask, x, y, w, h, area, fill_ratio, bottom_gap_px)
            if candidate is not None:
                candidates.append(candidate)
        return candidates

    def _candidate_from_roi(
        self,
        mask: np.ndarray,
        x: int,
        y: int,
        w: int,
        h: int,
        area: float,
        fill_ratio: float,
        bottom_gap_px: int,
    ) -> Optional[SlopeCandidate]:
        roi_depth = self.latest_depth[y:y + h, x:x + w]
        roi_mask = mask[y:y + h, x:x + w] > 0
        if roi_depth.size == 0:
            return None

        depth_m = self._depth_to_meters(roi_depth)
        depth_roi_mask = self._primary_depth_mask(roi_mask)
        depth_result = self._depth_result_from_mask(depth_m, depth_roi_mask)
        if depth_result is None and self.depth_fallback_enabled:
            depth_result = self._depth_result_from_mask(
                depth_m,
                self._fallback_depth_mask(roi_mask.shape),
            )
        if depth_result is None:
            return None

        distance_m, valid_depth_count = depth_result

        moments = cv2.moments(roi_mask.astype(np.uint8))
        if abs(moments['m00']) > 1e-6:
            pixel_x = float(x + moments['m10'] / moments['m00'])
            pixel_y = float(y + moments['m01'] / moments['m00'])
        else:
            pixel_x = float(x + w * 0.5)
            pixel_y = float(y + h * 0.5)

        lateral_m, vertical_m = self._project_pixel(pixel_x, pixel_y, distance_m)
        if self.max_abs_lateral_m > 0.0 and abs(lateral_m) > self.max_abs_lateral_m:
            return None

        return SlopeCandidate(
            distance_m=distance_m,
            lateral_m=lateral_m,
            vertical_m=vertical_m,
            pixel_x=pixel_x,
            pixel_y=pixel_y,
            width_px=w,
            height_px=h,
            area_px=area,
            fill_ratio=fill_ratio,
            valid_depth_count=valid_depth_count,
            bottom_gap_px=bottom_gap_px,
        )

    def _primary_depth_mask(self, roi_mask: np.ndarray) -> np.ndarray:
        depth_roi_mask = roi_mask
        if self.depth_mask_erode_px <= 0:
            return depth_roi_mask

        erode_kernel = np.ones(
            (self.depth_mask_erode_px, self.depth_mask_erode_px),
            dtype=np.uint8,
        )
        eroded = cv2.erode(roi_mask.astype(np.uint8), erode_kernel, iterations=1) > 0
        if int(np.count_nonzero(eroded)) >= self.min_valid_depth_pixels:
            depth_roi_mask = eroded
        return depth_roi_mask

    def _fallback_depth_mask(self, shape: tuple) -> np.ndarray:
        height, width = shape[:2]
        fallback = np.zeros((height, width), dtype=bool)
        if height <= 0 or width <= 0:
            return fallback

        y0 = int(np.clip(round(height * self.depth_fallback_y_min_ratio), 0, height - 1))
        y1 = int(np.clip(round(height * self.depth_fallback_y_max_ratio), y0 + 1, height))
        x_margin = int(np.clip(round(width * self.depth_fallback_x_margin_ratio), 0, width // 2))
        x0 = x_margin
        x1 = max(x0 + 1, width - x_margin)
        fallback[y0:y1, x0:x1] = True
        return fallback

    def _depth_result_from_mask(self, depth_m: np.ndarray, mask: np.ndarray) -> Optional[tuple]:
        valid = (
            mask &
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
        return distance_m, valid_depth_count

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

    def _publish_nearest(self, candidate: SlopeCandidate) -> None:
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
            float(candidate.fill_ratio),
            float(candidate.valid_depth_count),
            float(candidate.bottom_gap_px),
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
        candidates: List[SlopeCandidate],
        nearest: Optional[SlopeCandidate],
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
                f'{candidate.distance_m:.2f}m fill={candidate.fill_ratio:.2f}',
                (x, max(y - 8, 16)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
        cv2.putText(
            debug,
            'state=DETECTING',
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
        candidates: List[SlopeCandidate],
        nearest: Optional[SlopeCandidate],
        triggered: bool,
    ) -> None:
        if (now - self.last_log_time) < self.log_interval_sec:
            return
        self.last_log_time = now

        if nearest is None:
            self._publish_feedback(
                f'未检测到有效斜坡: candidates=0, frame={self.frame_count}, '
                f'{self._mask_reject_summary(mask)}, '
                f'hsv_lower={self.hsv_lower.tolist()}, hsv_upper={self.hsv_upper.tolist()}'
            )
            return

        self._publish_feedback(
            f'最近斜坡: dist={nearest.distance_m:.3f}m, lateral={nearest.lateral_m:.3f}m, '
            f'vertical={nearest.vertical_m:.3f}m, pixel=({nearest.pixel_x:.1f},{nearest.pixel_y:.1f}), '
            f'box={nearest.width_px}x{nearest.height_px}, area={nearest.area_px:.0f}, '
            f'fill={nearest.fill_ratio:.2f}, bottom_gap={nearest.bottom_gap_px}px, '
            f'depth_px={nearest.valid_depth_count}, candidates={len(candidates)}, trigger={triggered}'
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
        fill_ratio = area / max(float(w * h), 1.0)
        bottom_gap_px = max(0, mask.shape[0] - (y + h))

        reject_reasons = []
        if area < self.min_area_px:
            reject_reasons.append('area')
        if w < self.min_width_px:
            reject_reasons.append('width')
        if h < self.min_height_px:
            reject_reasons.append('height')
        if aspect_ratio < self.min_aspect_ratio:
            reject_reasons.append('aspect')
        if fill_ratio < self.min_fill_ratio:
            reject_reasons.append('fill')
        if fill_ratio > self.max_fill_ratio:
            reject_reasons.append('fill_max')
        if bottom_gap_px > self.max_bottom_gap_px:
            reject_reasons.append('bottom_gap')
        if not reject_reasons:
            reject_reasons.append(self._candidate_reject_summary(mask, x, y, w, h))

        return (
            f'mask_px={mask_px}, contours={len(contours)}, '
            f'largest=area:{area:.0f},box:{w}x{h},fill:{fill_ratio:.2f},'
            f'bottom_gap:{bottom_gap_px},reject:{"+".join(reject_reasons)}'
        )

    def _candidate_reject_summary(self, mask: np.ndarray, x: int, y: int, w: int, h: int) -> str:
        if self.latest_depth is None:
            return 'depth_missing'
        roi_depth = self.latest_depth[y:y + h, x:x + w]
        roi_mask = mask[y:y + h, x:x + w] > 0
        if roi_depth.size == 0:
            return 'depth_empty_roi'

        depth_m = self._depth_to_meters(roi_depth)
        primary_mask = self._primary_depth_mask(roi_mask)
        primary_count = self._valid_depth_count(depth_m, primary_mask)
        fallback_mask = self._fallback_depth_mask(roi_mask.shape)
        fallback_count = self._valid_depth_count(depth_m, fallback_mask)
        depth_result = self._depth_result_from_mask(depth_m, primary_mask)
        depth_source = 'primary'
        if depth_result is None and self.depth_fallback_enabled:
            depth_result = self._depth_result_from_mask(depth_m, fallback_mask)
            depth_source = 'fallback'
        if depth_result is None:
            return (
                f'depth_invalid(primary_px={primary_count},fallback_px={fallback_count},'
                f'min_px={self.min_valid_depth_pixels})'
            )

        distance_m, _ = depth_result
        moments = cv2.moments(roi_mask.astype(np.uint8))
        if abs(moments['m00']) > 1e-6:
            pixel_x = float(x + moments['m10'] / moments['m00'])
            pixel_y = float(y + moments['m01'] / moments['m00'])
        else:
            pixel_x = float(x + w * 0.5)
            pixel_y = float(y + h * 0.5)
        lateral_m, _ = self._project_pixel(pixel_x, pixel_y, distance_m)
        if self.max_abs_lateral_m > 0.0 and abs(lateral_m) > self.max_abs_lateral_m:
            return (
                f'lateral({lateral_m:.3f}>{self.max_abs_lateral_m:.3f},'
                f'depth_source={depth_source},dist={distance_m:.3f})'
            )
        return (
            f'depth_or_lateral_unknown(primary_px={primary_count},'
            f'fallback_px={fallback_count},depth_source={depth_source},'
            f'dist={distance_m:.3f},lateral={lateral_m:.3f})'
        )

    def _valid_depth_count(self, depth_m: np.ndarray, mask: np.ndarray) -> int:
        valid = (
            mask &
            np.isfinite(depth_m) &
            (depth_m >= self.min_depth_m) &
            (depth_m <= self.max_depth_m)
        )
        return int(np.count_nonzero(valid))

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
        self.last_raw_trigger_stamp_sec = 0.0

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
        if level == 'info' and text == self.last_feedback:
            return
        if level == 'info':
            self.last_feedback = text
        msg = String()
        msg.data = text
        self.feedback_pub.publish(msg)
        getattr(self.get_logger(), level)(colorize_log(text, level=level))

    def _int_list_param(self, name: str, expected_len: int) -> List[int]:
        values = list(self.get_parameter(name).value)
        if len(values) != expected_len:
            raise ValueError(f'Parameter {name} must contain {expected_len} elements')
        return [int(v) for v in values]

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlopeBranchDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
