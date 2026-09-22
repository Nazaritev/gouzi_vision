#!/usr/bin/env python3
"""Lightweight orange pole detector using RGB HSV segmentation and aligned depth.

2026-05-03 01:05 CST:
Reason: D435 + YOLO on the CPU-only mini PC saturates the CPU and causes
frame delay. The right-angle pole obstacle can be localized more cheaply from
its orange color and depth image.
Rollback: keep this as a separate node/launch. Existing yolo_relative_nav is
unchanged and can still be used by launching the old YOLO path.
"""

import math
from dataclasses import dataclass
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from auto_nav_pkg.log_style import colorize_log
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32MultiArray, String


@dataclass
class PoleCandidate:
    distance_m: float
    lateral_m: float
    vertical_m: float
    physical_height_m: Optional[float]
    pixel_x: float
    pixel_y: float
    width_px: int
    height_px: int
    area_px: float
    valid_depth_count: int


class OrangePoleDetector(Node):
    """Detect orange vertical poles and publish the nearest candidate."""

    def __init__(self) -> None:
        super().__init__('orange_pole_detector')

        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('nearest_topic', '/orange_pole_detector/nearest')
        self.declare_parameter('nearest_pose_topic', '/orange_pole_detector/nearest_pose')
        self.declare_parameter('trigger_topic', '/orange_pole_detector/trigger')
        self.declare_parameter('feedback_topic', '/orange_pole_detector/feedback_log')
        self.declare_parameter('debug_image_topic', '/orange_pole_detector/debug_image')

        self.declare_parameter('hsv_lower', [5, 70, 70])
        self.declare_parameter('hsv_upper', [35, 255, 255])
        self.declare_parameter('min_area_px', 120.0)
        self.declare_parameter('min_height_px', 45)
        self.declare_parameter('min_width_px', 4)
        self.declare_parameter('max_width_px', 260)
        self.declare_parameter('min_bottom_y_ratio', 0.0)
        self.declare_parameter('min_aspect_ratio', 1.8)
        self.declare_parameter('morph_kernel_px', 5)
        self.declare_parameter('depth_units_divisor', 1000.0)
        self.declare_parameter('min_depth_m', 0.20)
        self.declare_parameter('max_depth_m', 3.00)
        self.declare_parameter('min_valid_depth_pixels', 8)
        self.declare_parameter('depth_mask_erode_px', 3)
        self.declare_parameter('depth_percentile', 25.0)
        self.declare_parameter('distance_bias_m', 0.0)
        self.declare_parameter('depth_stale_timeout_sec', 0.50)
        self.declare_parameter('camera_info_stale_timeout_sec', 5.0)
        self.declare_parameter('depth_wait_warn_after_sec', 2.0)
        self.declare_parameter('trigger_distance_m', 1.25)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('log_interval_sec', 1.0)
        self.declare_parameter('camera_height_m', 0.23)
        self.declare_parameter('use_ground_height_filter', False)
        self.declare_parameter('ground_height_tolerance_m', 0.18)
        self.declare_parameter('use_physical_height_filter', False)
        self.declare_parameter('min_physical_height_m', 0.10)
        self.declare_parameter('physical_height_band_ratio', 0.25)
        self.declare_parameter('physical_height_min_valid_pixels', 3)

        self.color_topic = str(self.get_parameter('color_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.nearest_topic = str(self.get_parameter('nearest_topic').value)
        self.nearest_pose_topic = str(self.get_parameter('nearest_pose_topic').value)
        self.trigger_topic = str(self.get_parameter('trigger_topic').value)
        self.feedback_topic = str(self.get_parameter('feedback_topic').value)
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)

        self.hsv_lower = np.array(self._int_list_param('hsv_lower', 3), dtype=np.uint8)
        self.hsv_upper = np.array(self._int_list_param('hsv_upper', 3), dtype=np.uint8)
        self.min_area_px = float(self.get_parameter('min_area_px').value)
        self.min_height_px = int(self.get_parameter('min_height_px').value)
        self.min_width_px = int(self.get_parameter('min_width_px').value)
        self.max_width_px = int(self.get_parameter('max_width_px').value)
        self.min_bottom_y_ratio = max(
            0.0,
            min(1.0, float(self.get_parameter('min_bottom_y_ratio').value)),
        )
        self.min_aspect_ratio = float(self.get_parameter('min_aspect_ratio').value)
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
        self.depth_stale_timeout_sec = float(self.get_parameter('depth_stale_timeout_sec').value)
        self.camera_info_stale_timeout_sec = float(
            self.get_parameter('camera_info_stale_timeout_sec').value
        )
        self.depth_wait_warn_after_sec = max(
            0.0,
            float(self.get_parameter('depth_wait_warn_after_sec').value),
        )
        self.trigger_distance_m = float(self.get_parameter('trigger_distance_m').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.log_interval_sec = float(self.get_parameter('log_interval_sec').value)
        self.camera_height_m = float(self.get_parameter('camera_height_m').value)
        self.use_ground_height_filter = bool(self.get_parameter('use_ground_height_filter').value)
        self.ground_height_tolerance_m = float(
            self.get_parameter('ground_height_tolerance_m').value
        )
        self.use_physical_height_filter = bool(
            self.get_parameter('use_physical_height_filter').value
        )
        self.min_physical_height_m = max(
            0.0,
            float(self.get_parameter('min_physical_height_m').value),
        )
        self.physical_height_band_ratio = float(
            np.clip(float(self.get_parameter('physical_height_band_ratio').value), 0.05, 0.50)
        )
        self.physical_height_min_valid_pixels = max(
            1,
            int(self.get_parameter('physical_height_min_valid_pixels').value),
        )

        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_stamp_sec: Optional[float] = None
        self.latest_depth_frame = ''
        self.latest_camera_info: Optional[CameraInfo] = None
        self.latest_camera_info_stamp_sec: Optional[float] = None
        self.node_start_time_sec = self._now_sec()
        self.last_log_time = 0.0
        self.frame_count = 0
        self.depth_frame_count = 0
        self.last_trigger_state: Optional[bool] = None

        self.nearest_pub = self.create_publisher(Float32MultiArray, self.nearest_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.nearest_pose_topic, 10)
        self.trigger_pub = self.create_publisher(Bool, self.trigger_topic, 10)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)
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

        self._publish_feedback(
            f'orange_pole_detector started: color={self.color_topic}, depth={self.depth_topic}, '
            f'camera_info={self.camera_info_topic}, hsv_lower={self.hsv_lower.tolist()}, '
            f'hsv_upper={self.hsv_upper.tolist()}, trigger_distance={self.trigger_distance_m:.2f}m, '
            f'camera_height={self.camera_height_m:.2f}m, '
            f'depth_percentile={self.depth_percentile:.1f}, distance_bias={self.distance_bias_m:.3f}m'
        )

    def _depth_callback(self, msg: Image) -> None:
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self._publish_feedback(f'深度图转换失败: {exc}', level='warn')
            return
        self.depth_frame_count += 1
        self.latest_depth_stamp_sec = self._stamp_to_sec(msg.header.stamp)
        self.latest_depth_frame = msg.header.frame_id
        if self.depth_frame_count == 1:
            self._publish_feedback(
                f'已收到第一帧对齐深度图: topic={self.depth_topic}, frame={self.latest_depth_frame}, '
                f'shape={self.latest_depth.shape[:2]}'
            )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.latest_camera_info = msg
        self.latest_camera_info_stamp_sec = self._stamp_to_sec(msg.header.stamp)

    def _color_callback(self, msg: Image) -> None:
        self.frame_count += 1
        now = self._now_sec()

        if self.latest_depth is None:
            wait_time = max(0.0, now - self.node_start_time_sec)
            if wait_time >= self.depth_wait_warn_after_sec:
                self._log_periodic(
                    f'仍未收到对齐深度图: wait={wait_time:.2f}s, depth_topic={self.depth_topic}, '
                    f'camera_info_received={self.latest_camera_info is not None}',
                    now,
                    level='warn',
                )
            else:
                self._log_periodic(
                    f'等待对齐深度图: wait={wait_time:.2f}s, depth_topic={self.depth_topic}',
                    now,
                )
            self._publish_trigger(False)
            return

        if self.latest_camera_info is None:
            self._log_periodic('等待 color camera_info。', now)
            self._publish_trigger(False)
            return

        if self._data_is_stale(now):
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
            self._publish_trigger(False)
            return

        mask = self._build_mask(bgr)
        candidates = self._extract_candidates(mask)
        nearest = min(candidates, key=lambda item: item.distance_m) if candidates else None
        triggered = nearest is not None and nearest.distance_m <= self.trigger_distance_m

        self._publish_trigger(triggered)
        if nearest is not None:
            self._publish_nearest(nearest, msg.header.frame_id, msg.header.stamp)

        if self.publish_debug_image:
            self._publish_debug_image(bgr, mask, candidates, nearest, msg.header)

        self._log_detection(now, candidates, nearest, triggered)

    def _build_mask(self, bgr: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        kernel = np.ones((self.morph_kernel_px, self.morph_kernel_px), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _extract_candidates(self, mask: np.ndarray) -> List[PoleCandidate]:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates: List[PoleCandidate] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area_px:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if h < self.min_height_px or w < self.min_width_px or w > self.max_width_px:
                continue
            if self.min_bottom_y_ratio > 0.0:
                image_height = int(mask.shape[0])
                if image_height > 0 and float(y + h) < self.min_bottom_y_ratio * image_height:
                    continue
            aspect_ratio = h / max(float(w), 1.0)
            if aspect_ratio < self.min_aspect_ratio:
                continue

            candidate = self._candidate_from_roi(mask, x, y, w, h, area)
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
    ) -> Optional[PoleCandidate]:
        roi_depth = self.latest_depth[y:y + h, x:x + w]
        roi_mask = mask[y:y + h, x:x + w] > 0
        if roi_depth.size == 0:
            return None

        depth_m = self._depth_to_meters(roi_depth)
        # 2026-05-03 01:29 CST: thin orange poles can include background/floor
        # depth at mask edges. Erode the mask for distance estimation, then use
        # a near-side percentile instead of the median. Rollback: set
        # depth_mask_erode_px=0 and depth_percentile=50.0 to recover the old
        # median behavior.
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
        physical_height_m = self._estimate_physical_height_m(depth_m, roi_mask, x, y, w, h)

        # 2026-05-17: A yellow/orange floor guide line can become a tall, thin
        # contour under perspective and pass the 2D pole shape checks. Estimate
        # physical top-vs-bottom height with each band's own depth: a floor line
        # remains near ground height at both ends, while a real pole has visible
        # vertical extent. Rollback: set use_physical_height_filter=false.
        if self.use_physical_height_filter:
            if physical_height_m is None or physical_height_m < self.min_physical_height_m:
                return None

        if self.use_ground_height_filter:
            bottom_vertical_m = self._project_pixel(pixel_x, float(y + h), distance_m)[1]
            if abs(bottom_vertical_m - self.camera_height_m) > self.ground_height_tolerance_m:
                return None

        return PoleCandidate(
            distance_m=distance_m,
            lateral_m=lateral_m,
            vertical_m=vertical_m,
            physical_height_m=physical_height_m,
            pixel_x=pixel_x,
            pixel_y=pixel_y,
            width_px=w,
            height_px=h,
            area_px=area,
            valid_depth_count=int(np.count_nonzero(valid)),
        )

    def _estimate_physical_height_m(
        self,
        depth_m: np.ndarray,
        roi_mask: np.ndarray,
        roi_x: int,
        roi_y: int,
        width_px: int,
        height_px: int,
    ) -> Optional[float]:
        ys, _ = np.nonzero(roi_mask)
        if ys.size < self.physical_height_min_valid_pixels * 2:
            return None

        top_y = int(np.min(ys))
        bottom_y = int(np.max(ys))
        band_h = max(1, int(round(height_px * self.physical_height_band_ratio)))
        row_index = np.arange(height_px)[:, None]
        top_mask = roi_mask & (row_index <= top_y + band_h)
        bottom_mask = roi_mask & (row_index >= bottom_y - band_h)

        top_sample = self._height_band_sample(depth_m, top_mask, roi_x, roi_y)
        bottom_sample = self._height_band_sample(depth_m, bottom_mask, roi_x, roi_y)
        if top_sample is None or bottom_sample is None:
            return None

        top_x, top_y_px, top_depth = top_sample
        bottom_x, bottom_y_px, bottom_depth = bottom_sample
        top_vertical_m = self._project_pixel(top_x, top_y_px, top_depth)[1]
        bottom_vertical_m = self._project_pixel(bottom_x, bottom_y_px, bottom_depth)[1]
        physical_height_m = bottom_vertical_m - top_vertical_m
        if not math.isfinite(physical_height_m):
            return None
        return max(0.0, physical_height_m)

    def _height_band_sample(
        self,
        depth_m: np.ndarray,
        band_mask: np.ndarray,
        roi_x: int,
        roi_y: int,
    ) -> Optional[tuple]:
        valid = (
            band_mask &
            np.isfinite(depth_m) &
            (depth_m >= self.min_depth_m) &
            (depth_m <= self.max_depth_m)
        )
        if int(np.count_nonzero(valid)) < self.physical_height_min_valid_pixels:
            return None

        ys, xs = np.nonzero(valid)
        depth = float(np.percentile(depth_m[valid], 50.0))
        if not math.isfinite(depth):
            return None
        return (
            float(roi_x + np.mean(xs)),
            float(roi_y + np.mean(ys)),
            depth,
        )

    def _depth_to_meters(self, depth: np.ndarray) -> np.ndarray:
        if np.issubdtype(depth.dtype, np.integer):
            return depth.astype(np.float32) / max(self.depth_units_divisor, 1e-6)
        return depth.astype(np.float32)

    def _project_pixel(self, u: float, v: float, depth_m: float) -> tuple:
        info = self.latest_camera_info
        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])
        lateral_m = (u - cx) * depth_m / max(fx, 1e-6)
        vertical_m = (v - cy) * depth_m / max(fy, 1e-6)
        return lateral_m, vertical_m

    def _publish_nearest(self, candidate: PoleCandidate, frame_id: str, stamp) -> None:
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

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = frame_id or self.latest_depth_frame
        pose.pose.position.x = candidate.lateral_m
        pose.pose.position.y = candidate.vertical_m
        pose.pose.position.z = candidate.distance_m
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

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
        candidates: List[PoleCandidate],
        nearest: Optional[PoleCandidate],
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
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = cv2.addWeighted(debug, 0.75, mask_bgr, 0.25, 0.0)
        out = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
        out.header = header
        self.debug_image_pub.publish(out)

    def _log_detection(
        self,
        now: float,
        candidates: List[PoleCandidate],
        nearest: Optional[PoleCandidate],
        triggered: bool,
    ) -> None:
        if (now - self.last_log_time) < self.log_interval_sec:
            return
        self.last_log_time = now

        if nearest is None:
            self._publish_feedback(
                f'未检测到有效橘色竖杆: candidates=0, frame={self.frame_count}, '
                f'hsv_lower={self.hsv_lower.tolist()}, hsv_upper={self.hsv_upper.tolist()}'
            )
            return

        self._publish_feedback(
            f'最近杆: dist={nearest.distance_m:.3f}m, lateral={nearest.lateral_m:.3f}m, '
            f'vertical={nearest.vertical_m:.3f}m, pixel=({nearest.pixel_x:.1f},{nearest.pixel_y:.1f}), '
            f'box={nearest.width_px}x{nearest.height_px}, height3d='
            f'{self._optional_distance_text(nearest.physical_height_m)}, '
            f'depth_px={nearest.valid_depth_count}, '
            f'candidates={len(candidates)}, trigger={triggered}'
        )

    @staticmethod
    def _optional_distance_text(value: Optional[float]) -> str:
        if value is None or not math.isfinite(value):
            return 'unknown'
        return f'{value:.3f}m'

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

    def _log_periodic(self, text: str, now: float, level: str = 'info') -> None:
        if (now - self.last_log_time) < self.log_interval_sec:
            return
        self.last_log_time = now
        self._publish_feedback(text, level=level)

    def _publish_feedback(self, text: str, level: str = 'info') -> None:
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrangePoleDetector()
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
