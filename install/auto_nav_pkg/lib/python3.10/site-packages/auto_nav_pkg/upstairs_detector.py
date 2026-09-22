#!/usr/bin/env python3
"""HSV+depth detector for the final green upstairs obstacle."""

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
class UpstairsCandidate:
    distance_m: float
    lateral_m: float
    vertical_m: float
    pixel_x: float
    pixel_y: float
    width_px: int
    height_px: int
    area_px: int
    valid_depth_count: int
    band_index_from_bottom: int
    y_min_px: int
    y_max_px: int


class UpstairsDetector(Node):
    """Detect green stair bands and report the configured target stair depth."""

    def __init__(self) -> None:
        super().__init__('upstairs_detector')

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        self.declare_parameter('nearest_topic', '/upstairs_detector/nearest')
        self.declare_parameter('trigger_topic', '/upstairs_detector/trigger')
        self.declare_parameter('feedback_topic', '/upstairs_detector/feedback_log')
        self.declare_parameter('state_topic', '/upstairs_detector/state')
        self.declare_parameter('debug_image_topic', '/upstairs_detector/debug_image')

        self.declare_parameter('hsv_lower', [30, 40, 30])
        self.declare_parameter('hsv_upper', [100, 255, 255])
        self.declare_parameter('morph_open_kernel_px', 5)
        self.declare_parameter('morph_close_kernel_px', 0)
        self.declare_parameter('morph_close_iterations', 1)
        self.declare_parameter('search_y_min_px', 160)
        self.declare_parameter('search_y_max_px', 470)
        self.declare_parameter('row_min_green_ratio', 0.35)
        self.declare_parameter('row_min_valid_depth_pixels', 80)
        self.declare_parameter('depth_split_threshold_m', 0.16)
        self.declare_parameter('max_row_gap_px', 3)
        self.declare_parameter('target_band_from_bottom', 2)
        self.declare_parameter('min_band_height_px', 20)
        self.declare_parameter('min_band_width_px', 180)
        self.declare_parameter('min_band_area_px', 4000)
        self.declare_parameter('max_abs_lateral_m', 1.50)

        self.declare_parameter('depth_units_divisor', 1000.0)
        self.declare_parameter('min_depth_m', 0.20)
        self.declare_parameter('max_depth_m', 3.00)
        self.declare_parameter('depth_percentile', 50.0)
        self.declare_parameter('distance_bias_m', 0.0)
        self.declare_parameter('trigger_distance_m', 0.610)
        self.declare_parameter('trigger_hold_sec', 0.20)
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
        self.morph_open_kernel_px = max(0, int(self.get_parameter('morph_open_kernel_px').value))
        self.morph_close_kernel_px = max(0, int(self.get_parameter('morph_close_kernel_px').value))
        self.morph_close_iterations = max(
            1, int(self.get_parameter('morph_close_iterations').value)
        )
        self.search_y_min_px = max(0, int(self.get_parameter('search_y_min_px').value))
        self.search_y_max_px = max(0, int(self.get_parameter('search_y_max_px').value))
        self.row_min_green_ratio = float(
            np.clip(float(self.get_parameter('row_min_green_ratio').value), 0.0, 1.0)
        )
        self.row_min_valid_depth_pixels = max(
            1, int(self.get_parameter('row_min_valid_depth_pixels').value)
        )
        self.depth_split_threshold_m = max(
            0.01, float(self.get_parameter('depth_split_threshold_m').value)
        )
        self.max_row_gap_px = max(1, int(self.get_parameter('max_row_gap_px').value))
        self.target_band_from_bottom = max(1, int(self.get_parameter('target_band_from_bottom').value))
        self.min_band_height_px = max(1, int(self.get_parameter('min_band_height_px').value))
        self.min_band_width_px = max(1, int(self.get_parameter('min_band_width_px').value))
        self.min_band_area_px = max(1, int(self.get_parameter('min_band_area_px').value))
        self.max_abs_lateral_m = max(0.0, float(self.get_parameter('max_abs_lateral_m').value))

        self.depth_units_divisor = float(self.get_parameter('depth_units_divisor').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.depth_percentile = float(
            np.clip(float(self.get_parameter('depth_percentile').value), 0.0, 100.0)
        )
        self.distance_bias_m = float(self.get_parameter('distance_bias_m').value)
        self.trigger_distance_m = max(0.0, float(self.get_parameter('trigger_distance_m').value))
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
        self.latest_candidate: Optional[UpstairsCandidate] = None
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
            f'upstairs_detector started: color={self.color_topic}, depth={self.depth_topic}, '
            f'target_band_from_bottom={self.target_band_from_bottom}, '
            f'trigger_distance={self.trigger_distance_m:.2f}m, '
            f'hsv={self.hsv_lower.tolist()}-{self.hsv_upper.tolist()}'
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
        target = self._select_target_candidate(candidates)
        raw_triggered = target is not None and target.distance_m <= self.trigger_distance_m
        if raw_triggered:
            self.last_raw_trigger_stamp_sec = now
        triggered = raw_triggered or (
            self.trigger_hold_sec > 0.0
            and self.last_raw_trigger_stamp_sec > 0.0
            and (now - self.last_raw_trigger_stamp_sec) <= self.trigger_hold_sec
        )

        self.latest_candidate = target
        self.current_triggered = triggered

        self._publish_trigger(triggered)
        if target is not None:
            self._publish_nearest(target)
        if self.publish_debug_image:
            self._publish_debug_image(bgr, mask, candidates, target, msg.header)
        self._log_detection(now, mask, candidates, target, triggered)

    def _build_mask(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        height = mask.shape[0]
        y_min = int(np.clip(self.search_y_min_px, 0, max(height - 1, 0)))
        y_max = int(np.clip(self.search_y_max_px, y_min + 1, height))
        filtered = np.zeros_like(mask)
        filtered[y_min:y_max, :] = mask[y_min:y_max, :]

        if self.morph_open_kernel_px > 1:
            kernel = np.ones(
                (self.morph_open_kernel_px, self.morph_open_kernel_px),
                dtype=np.uint8,
            )
            filtered = cv2.morphologyEx(filtered, cv2.MORPH_OPEN, kernel)
        if self.morph_close_kernel_px > 1:
            kernel = np.ones(
                (self.morph_close_kernel_px, self.morph_close_kernel_px),
                dtype=np.uint8,
            )
            filtered = cv2.morphologyEx(
                filtered,
                cv2.MORPH_CLOSE,
                kernel,
                iterations=self.morph_close_iterations,
            )
        return filtered

    def _extract_candidates(self, mask: np.ndarray) -> List[UpstairsCandidate]:
        depth_m = self._depth_to_meters(self.latest_depth)
        height, width = mask.shape[:2]
        y_min = int(np.clip(self.search_y_min_px, 0, max(height - 1, 0)))
        y_max = int(np.clip(self.search_y_max_px, y_min + 1, height))

        row_profiles = []
        min_green_px = max(1, int(round(width * self.row_min_green_ratio)))
        for y in range(y_min, y_max):
            row_mask = mask[y, :] > 0
            green_count = int(np.count_nonzero(row_mask))
            if green_count < min_green_px:
                continue
            row_depth = depth_m[y, :]
            valid = (
                row_mask
                & np.isfinite(row_depth)
                & (row_depth >= self.min_depth_m)
                & (row_depth <= self.max_depth_m)
            )
            if int(np.count_nonzero(valid)) < self.row_min_valid_depth_pixels:
                continue
            row_profiles.append((y, float(np.percentile(row_depth[valid], 50.0))))

        segments = self._segment_rows_by_depth(row_profiles)
        candidates = []
        for segment in segments:
            candidate = self._candidate_from_segment(mask, depth_m, segment)
            if candidate is not None:
                candidates.append(candidate)

        candidates.sort(key=lambda item: item.pixel_y)
        total = len(candidates)
        for idx, candidate in enumerate(candidates):
            candidate.band_index_from_bottom = total - idx
        return candidates

    def _segment_rows_by_depth(self, rows: List[tuple[int, float]]) -> List[List[tuple[int, float]]]:
        segments: List[List[tuple[int, float]]] = []
        current: List[tuple[int, float]] = []
        for row in rows:
            y, distance = row
            if not current:
                current = [row]
                continue
            recent_distances = [item[1] for item in current[-10:]]
            reference_distance = float(np.median(recent_distances))
            if (
                y - current[-1][0] <= self.max_row_gap_px
                and abs(distance - reference_distance) <= self.depth_split_threshold_m
            ):
                current.append(row)
            else:
                segments.append(current)
                current = [row]
        if current:
            segments.append(current)
        return segments

    def _candidate_from_segment(
        self,
        mask: np.ndarray,
        depth_m: np.ndarray,
        segment: List[tuple[int, float]],
    ) -> Optional[UpstairsCandidate]:
        if len(segment) < self.min_band_height_px:
            return None
        y_values = [item[0] for item in segment]
        y_min = min(y_values)
        y_max = max(y_values) + 1
        band_mask = mask[y_min:y_max, :] > 0
        area_px = int(np.count_nonzero(band_mask))
        if area_px < self.min_band_area_px:
            return None

        active_columns = np.where(np.any(band_mask, axis=0))[0]
        if active_columns.size <= 0:
            return None
        x_min = int(active_columns.min())
        x_max = int(active_columns.max()) + 1
        width_px = x_max - x_min
        height_px = y_max - y_min
        if width_px < self.min_band_width_px:
            return None
        if height_px < self.min_band_height_px:
            return None

        roi_depth = depth_m[y_min:y_max, :]
        valid_depth = (
            band_mask
            & np.isfinite(roi_depth)
            & (roi_depth >= self.min_depth_m)
            & (roi_depth <= self.max_depth_m)
        )
        valid_depth_count = int(np.count_nonzero(valid_depth))
        if valid_depth_count < self.min_band_area_px:
            return None

        distance_m = float(np.percentile(roi_depth[valid_depth], self.depth_percentile))
        distance_m += self.distance_bias_m
        if not math.isfinite(distance_m):
            return None

        moments = cv2.moments(band_mask.astype(np.uint8))
        if abs(moments['m00']) > 1e-6:
            pixel_x = float(moments['m10'] / moments['m00'])
            pixel_y = float(y_min + moments['m01'] / moments['m00'])
        else:
            pixel_x = float((x_min + x_max) * 0.5)
            pixel_y = float((y_min + y_max) * 0.5)
        lateral_m, vertical_m = self._project_pixel(pixel_x, pixel_y, distance_m)
        if self.max_abs_lateral_m > 0.0 and abs(lateral_m) > self.max_abs_lateral_m:
            return None

        return UpstairsCandidate(
            distance_m=distance_m,
            lateral_m=lateral_m,
            vertical_m=vertical_m,
            pixel_x=pixel_x,
            pixel_y=pixel_y,
            width_px=width_px,
            height_px=height_px,
            area_px=area_px,
            valid_depth_count=valid_depth_count,
            band_index_from_bottom=0,
            y_min_px=y_min,
            y_max_px=y_max,
        )

    def _select_target_candidate(
        self,
        candidates: List[UpstairsCandidate],
    ) -> Optional[UpstairsCandidate]:
        if not candidates:
            return None
        if len(candidates) < self.target_band_from_bottom:
            return None
        return candidates[-self.target_band_from_bottom]

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

    def _publish_nearest(self, candidate: UpstairsCandidate) -> None:
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
            float(candidate.band_index_from_bottom),
            float(candidate.y_min_px),
            float(candidate.y_max_px),
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
        candidates: List[UpstairsCandidate],
        target: Optional[UpstairsCandidate],
        header,
    ) -> None:
        debug = bgr.copy()
        for candidate in candidates:
            color = (0, 255, 0) if candidate is target else (255, 0, 0)
            y_min = candidate.y_min_px
            y_max = candidate.y_max_px
            x_min = max(0, int(candidate.pixel_x - candidate.width_px * 0.5))
            x_max = min(mask.shape[1] - 1, int(candidate.pixel_x + candidate.width_px * 0.5))
            cv2.rectangle(debug, (x_min, y_min), (x_max, y_max), color, 2)
            cv2.putText(
                debug,
                f'#{candidate.band_index_from_bottom} {candidate.distance_m:.2f}m',
                (x_min, max(y_min - 8, 16)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
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
        candidates: List[UpstairsCandidate],
        target: Optional[UpstairsCandidate],
        triggered: bool,
    ) -> None:
        if (now - self.last_log_time) < self.log_interval_sec:
            return
        self.last_log_time = now
        if target is None:
            self._publish_feedback(
                f'未检测到有效上台阶: bands={len(candidates)}, '
                f'target_band_from_bottom={self.target_band_from_bottom}, '
                f'mask_px={int(np.count_nonzero(mask))}, '
                f'frame={self.frame_count}, hsv={self.hsv_lower.tolist()}-{self.hsv_upper.tolist()}'
            )
            return

        self._publish_feedback(
            f'最近上台阶: dist={target.distance_m:.3f}m, '
            f'band_from_bottom={target.band_index_from_bottom}, '
            f'lateral={target.lateral_m:.3f}m, vertical={target.vertical_m:.3f}m, '
            f'pixel=({target.pixel_x:.1f},{target.pixel_y:.1f}), '
            f'box={target.width_px}x{target.height_px}, area={target.area_px}, '
            f'depth_px={target.valid_depth_count}, bands={len(candidates)}, trigger={triggered}'
        )

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
    node = UpstairsDetector()
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
