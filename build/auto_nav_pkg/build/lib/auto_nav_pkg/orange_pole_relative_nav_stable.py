#!/usr/bin/env python3
"""Stable-trigger HSV pole route variant.

2026-05-05:
Reason: keep the current fast-trigger route untouched, but provide a second
route node that waits for a short stable nearest-pole window before activating
the route. This reduces trigger-time jitter without forcing a large delay.
Rollback: launch the original orange_pole_relative_nav_node instead.
"""

import math
from collections import deque
from typing import Deque, List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float32MultiArray

from auto_nav_pkg.orange_pole_relative_nav import OrangePoleRelativeNav, PoleSnapshot


class OrangePoleRelativeNavStable(OrangePoleRelativeNav):
    """Orange-pole route with a short stable-trigger gate before activation."""

    def __init__(self) -> None:
        # Base __init__ publishes startup override immediately, and that path
        # dispatches to this subclass override. Seed all stable-trigger fields
        # first so construction does not depend on subclass parameters being
        # declared already.
        self.stable_trigger_enabled = False
        self.stable_trigger_window_size = 5
        self.stable_trigger_min_samples = 3
        self.stable_trigger_max_sample_age_sec = 0.45
        self.stable_trigger_distance_spread_max_m = 0.08
        self.stable_trigger_lateral_spread_max_m = 0.08
        self.stable_trigger_median_distance_margin_m = 0.0
        self.stable_trigger_arm_timeout_sec = 0.60
        self.stable_trigger_timeout_fallback_to_immediate = True
        self.stable_trigger_hold_override_enabled = True
        self.stable_trigger_hold_mode = 0
        self.stable_trigger_hold_left_norm = 0.18
        self.stable_trigger_hold_right_norm = 0.18
        self.stable_trigger_compensate_pose_drift = True
        self.stable_trigger_use_armed_pose_as_anchor = True
        self.stable_trigger_history = deque(maxlen=self.stable_trigger_window_size)
        self.stable_trigger_armed = False
        self.stable_trigger_reason = ''
        self.stable_trigger_arm_time = 0.0
        self.stable_trigger_ref_pose = None
        self.stable_trigger_ref_pole = None

        super().__init__()

        self.declare_parameter('stable_trigger_enabled', True)
        self.declare_parameter('stable_trigger_window_size', 5)
        self.declare_parameter('stable_trigger_min_samples', 3)
        self.declare_parameter('stable_trigger_max_sample_age_sec', 0.45)
        self.declare_parameter('stable_trigger_distance_spread_max_m', 0.08)
        self.declare_parameter('stable_trigger_lateral_spread_max_m', 0.08)
        self.declare_parameter('stable_trigger_median_distance_margin_m', 0.0)
        self.declare_parameter('stable_trigger_arm_timeout_sec', 0.60)
        self.declare_parameter('stable_trigger_timeout_fallback_to_immediate', True)
        self.declare_parameter('stable_trigger_hold_override_enabled', True)
        self.declare_parameter('stable_trigger_hold_mode', 0)
        self.declare_parameter('stable_trigger_hold_left_norm', 0.18)
        self.declare_parameter('stable_trigger_hold_right_norm', 0.18)
        self.declare_parameter('stable_trigger_compensate_pose_drift', True)
        self.declare_parameter('stable_trigger_use_armed_pose_as_anchor', True)

        self.stable_trigger_enabled = bool(
            self.get_parameter('stable_trigger_enabled').value
        )
        self.stable_trigger_window_size = max(
            1, int(self.get_parameter('stable_trigger_window_size').value)
        )
        self.stable_trigger_min_samples = max(
            1, int(self.get_parameter('stable_trigger_min_samples').value)
        )
        self.stable_trigger_max_sample_age_sec = max(
            0.0,
            float(self.get_parameter('stable_trigger_max_sample_age_sec').value),
        )
        self.stable_trigger_distance_spread_max_m = max(
            0.0,
            float(self.get_parameter('stable_trigger_distance_spread_max_m').value),
        )
        self.stable_trigger_lateral_spread_max_m = max(
            0.0,
            float(self.get_parameter('stable_trigger_lateral_spread_max_m').value),
        )
        self.stable_trigger_median_distance_margin_m = max(
            0.0,
            float(self.get_parameter('stable_trigger_median_distance_margin_m').value),
        )
        self.stable_trigger_arm_timeout_sec = max(
            0.0,
            float(self.get_parameter('stable_trigger_arm_timeout_sec').value),
        )
        self.stable_trigger_timeout_fallback_to_immediate = bool(
            self.get_parameter('stable_trigger_timeout_fallback_to_immediate').value
        )
        self.stable_trigger_hold_override_enabled = bool(
            self.get_parameter('stable_trigger_hold_override_enabled').value
        )
        self.stable_trigger_hold_mode = int(
            self.get_parameter('stable_trigger_hold_mode').value
        )
        self.stable_trigger_hold_left_norm = max(
            0.0,
            min(1.0, float(self.get_parameter('stable_trigger_hold_left_norm').value)),
        )
        self.stable_trigger_hold_right_norm = max(
            0.0,
            min(1.0, float(self.get_parameter('stable_trigger_hold_right_norm').value)),
        )
        self.stable_trigger_compensate_pose_drift = bool(
            self.get_parameter('stable_trigger_compensate_pose_drift').value
        )
        self.stable_trigger_use_armed_pose_as_anchor = bool(
            self.get_parameter('stable_trigger_use_armed_pose_as_anchor').value
        )

        self.stable_trigger_history: Deque[
            Tuple[float, PoleSnapshot, Optional[Tuple[float, float, float, str]]]
        ] = deque(
            maxlen=self.stable_trigger_window_size
        )
        self.stable_trigger_armed = False
        self.stable_trigger_reason = ''
        self.stable_trigger_arm_time = 0.0
        self.stable_trigger_ref_pose: Optional[Tuple[float, float, float, str]] = None
        self.stable_trigger_ref_pole: Optional[PoleSnapshot] = None

        if self.stable_trigger_enabled:
            self._publish_feedback(
                '稳定触发版已启用: '
                f'samples>={self.stable_trigger_min_samples}, '
                f'window={self.stable_trigger_window_size}, '
                f'max_age={self.stable_trigger_max_sample_age_sec:.2f}s, '
                f'dist_spread<={self.stable_trigger_distance_spread_max_m:.3f}m, '
                f'lat_spread<={self.stable_trigger_lateral_spread_max_m:.3f}m, '
                f'timeout={self.stable_trigger_arm_timeout_sec:.2f}s, '
                f'hold_override={self.stable_trigger_hold_override_enabled} '
                f'[{self.stable_trigger_hold_mode},'
                f'{self.stable_trigger_hold_left_norm:.3f},'
                f'{self.stable_trigger_hold_right_norm:.3f}], '
                f'drift_comp={self.stable_trigger_compensate_pose_drift}'
            )

    def _nearest_callback(self, msg: Float32MultiArray) -> None:
        now = self._now_sec()
        self.last_pole_msg_time = now
        pole = self._parse_pole_snapshot(msg)
        if pole is None:
            self.last_pole_text = 'invalid'
            self._prune_stable_trigger_history(now)
            self._maybe_activate_stable_trigger(now)
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
            self._maybe_activate_stable_trigger(now)
            return

        if pole.distance_m > self.trigger_distance_m:
            if (now - self.last_wait_log_time) >= self.progress_log_interval_sec:
                self.last_wait_log_time = now
                self._publish_feedback(
                    f'HSV 已看到杆但未触发: {self.last_pole_text}, '
                    f'threshold={self.trigger_distance_m:.2f}m'
                )
            self._maybe_activate_stable_trigger(now)
            return

        if self.route_start_pending:
            super()._start_route(f'HSV 触发后收到最近杆: {self.last_pole_text}', source='hsv')
            if self.triggered:
                return

        self._record_stable_trigger_sample(now, pole)
        if self.trigger_from_nearest_topic:
            self._start_route(f'HSV 最近杆距离触发: {self.last_pole_text}', source='hsv')

        self._maybe_activate_stable_trigger(now)

    def _start_route(self, reason: str, source: str = 'hsv') -> None:
        if not self.route_controller_active:
            return
        if self.triggered or self.route_finished or self.route_failed:
            return
        if self.pole_pre_align_active:
            super()._start_route(reason, source=source)
            return

        if source == 'apriltag':
            super()._start_route(reason, source=source)
            return

        if not self.stable_trigger_enabled or self.route_start_pending:
            super()._start_route(reason, source=source)
            return

        if not self.stable_trigger_armed:
            self.stable_trigger_armed = True
            self.stable_trigger_reason = reason
            self.stable_trigger_arm_time = self._now_sec()
            self.stable_trigger_history.clear()
            self.stable_trigger_ref_pose = self._current_pose_snapshot()
            self.stable_trigger_ref_pole = None
        else:
            self.stable_trigger_reason = reason
        if (
            self.last_pole is not None
            and not self._nearest_is_stale()
            and self.last_pole.distance_m <= self.trigger_distance_m
            and (
                self.max_lateral_abs_m <= 0.0
                or abs(self.last_pole.lateral_m) <= self.max_lateral_abs_m
            )
        ):
            self._record_stable_trigger_sample(self._now_sec(), self.last_pole)

        self._publish_stable_trigger_wait()
        self._maybe_activate_stable_trigger()

    def _monitor_loop(self) -> None:
        if not self.route_controller_active:
            return
        self._maybe_activate_stable_trigger()
        super()._monitor_loop()

    def _publish_startup_step_override(self) -> None:
        if not self.route_controller_active:
            return
        if (
            self.stable_trigger_enabled
            and self.stable_trigger_armed
            and not self.triggered
            and self.stable_trigger_hold_override_enabled
        ):
            if self.route_finished or self.route_failed:
                return
            self._publish_step_override(
                self.stable_trigger_hold_mode,
                self.stable_trigger_hold_left_norm,
                self.stable_trigger_hold_right_norm,
            )
            return
        super()._publish_startup_step_override()

    def _route_start_prerequisites_ready(self) -> bool:
        if not self._pose_ready_for_goal():
            return False
        if not self._uses_route_anchor_reference() or self.route_anchor_ready:
            return True
        if (
            self._current_route_start_source() == 'hsv'
            and
            self.stable_trigger_enabled
            and self.stable_trigger_use_armed_pose_as_anchor
            and self.stable_trigger_ref_pose is not None
        ):
            return True
        return super()._route_start_prerequisites_ready()

    def _ensure_route_anchor(self, *, force: bool = False) -> bool:
        if force:
            return super()._ensure_route_anchor(force=force)
        if (
            self._uses_route_anchor_reference()
            and not self.route_anchor_ready
            and self._current_route_start_source() == 'hsv'
            and self.stable_trigger_enabled
            and self.stable_trigger_use_armed_pose_as_anchor
            and self.stable_trigger_ref_pose is not None
        ):
            anchor_x, anchor_y, anchor_yaw, anchor_frame = self.stable_trigger_ref_pose
            self.route_anchor_x = anchor_x
            self.route_anchor_y = anchor_y
            self.route_anchor_yaw = anchor_yaw
            self.route_anchor_frame = anchor_frame or self.default_frame_id
            self.route_anchor_ready = True
            self._publish_feedback(
                f'锁定稳定触发 anchor: pose=({self.route_anchor_x:.3f},'
                f'{self.route_anchor_y:.3f}), yaw={math.degrees(self.route_anchor_yaw):.1f}deg, '
                f'frame={self.route_anchor_frame}, 后续目标使用稳定触发参考位姿+累计偏移'
            )
            return True
        return super()._ensure_route_anchor(force=force)

    def _current_pose_snapshot(self) -> Optional[Tuple[float, float, float, str]]:
        if not self._pose_ready_for_goal():
            return None
        return (
            self.pose_x,
            self.pose_y,
            self.pose_yaw,
            self.pose_frame or self.default_frame_id,
        )

    def _record_stable_trigger_sample(self, now: float, pole: PoleSnapshot) -> None:
        pose_snapshot = self._current_pose_snapshot()
        if self.stable_trigger_armed:
            if self.stable_trigger_ref_pose is None and pose_snapshot is not None:
                self.stable_trigger_ref_pose = pose_snapshot
            if self.stable_trigger_ref_pole is None:
                self.stable_trigger_ref_pole = pole
        self.stable_trigger_history.append((now, pole, pose_snapshot))
        self._prune_stable_trigger_history(now)

    def _prune_stable_trigger_history(self, now: float) -> None:
        if self.stable_trigger_max_sample_age_sec <= 1e-6:
            while len(self.stable_trigger_history) > self.stable_trigger_window_size:
                self.stable_trigger_history.popleft()
            return

        while self.stable_trigger_history:
            sample_time, _, _ = self.stable_trigger_history[0]
            if (now - sample_time) <= self.stable_trigger_max_sample_age_sec:
                break
            self.stable_trigger_history.popleft()

    def _stable_trigger_samples(
        self,
        now: float,
    ) -> List[Tuple[PoleSnapshot, Optional[Tuple[float, float, float, str]]]]:
        self._prune_stable_trigger_history(now)
        return [(pole, pose_snapshot) for _, pole, pose_snapshot in self.stable_trigger_history]

    @staticmethod
    def _median(values: List[float]) -> float:
        if not values:
            return 0.0
        sorted_values = sorted(values)
        mid = len(sorted_values) // 2
        if len(sorted_values) % 2 == 1:
            return sorted_values[mid]
        return 0.5 * (sorted_values[mid - 1] + sorted_values[mid])

    def _stable_trigger_metrics(
        self, now: float
    ) -> Tuple[List[PoleSnapshot], float, float, float]:
        samples = self._stable_trigger_samples(now)
        if not samples:
            return [], 0.0, 0.0, 0.0

        compensated_samples: List[PoleSnapshot] = []
        distances: List[float] = []
        laterals: List[float] = []
        for sample, pose_snapshot in samples:
            distance_m, lateral_m = self._compensated_pole_components(sample, pose_snapshot)
            distances.append(distance_m)
            laterals.append(lateral_m)
            compensated_samples.append(
                PoleSnapshot(
                    distance_m=distance_m,
                    lateral_m=lateral_m,
                    vertical_m=sample.vertical_m,
                    pixel_x=sample.pixel_x,
                    pixel_y=sample.pixel_y,
                    width_px=sample.width_px,
                    height_px=sample.height_px,
                    area_px=sample.area_px,
                    valid_depth_count=sample.valid_depth_count,
                )
            )
        median_distance = self._median(distances)
        distance_spread = max(distances) - min(distances)
        lateral_spread = max(laterals) - min(laterals)
        return compensated_samples, median_distance, distance_spread, lateral_spread

    def _compensated_pole_components(
        self,
        sample: PoleSnapshot,
        pose_snapshot: Optional[Tuple[float, float, float, str]],
    ) -> Tuple[float, float]:
        if (
            not self.stable_trigger_compensate_pose_drift
            or pose_snapshot is None
            or self.stable_trigger_ref_pose is None
            or self.stable_trigger_ref_pole is None
        ):
            return sample.distance_m, sample.lateral_m

        arm_x, arm_y, arm_yaw, _ = self.stable_trigger_ref_pose
        sample_x, sample_y, _, _ = pose_snapshot
        pole_forward = math.sqrt(
            max(
                self.stable_trigger_ref_pole.distance_m ** 2
                - self.stable_trigger_ref_pole.lateral_m ** 2,
                0.0,
            )
        )
        pole_dx, pole_dy = self._body_relative_offset_to_world_from_yaw(
            arm_yaw,
            pole_forward,
            self.stable_trigger_ref_pole.lateral_m,
        )
        pole_norm = math.hypot(pole_dx, pole_dy)
        if pole_norm <= 1e-6:
            return sample.distance_m, sample.lateral_m

        pole_dir_x = pole_dx / pole_norm
        pole_dir_y = pole_dy / pole_norm
        pole_left_x = -pole_dir_y
        pole_left_y = pole_dir_x

        disp_x = sample_x - arm_x
        disp_y = sample_y - arm_y
        forward_progress = disp_x * pole_dir_x + disp_y * pole_dir_y
        lateral_shift = disp_x * pole_left_x + disp_y * pole_left_y
        return (
            sample.distance_m + forward_progress,
            sample.lateral_m + lateral_shift,
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

    def _stable_trigger_ready(self, now: float) -> Tuple[bool, str]:
        samples, median_distance, distance_spread, lateral_spread = (
            self._stable_trigger_metrics(now)
        )
        if len(samples) < self.stable_trigger_min_samples:
            return (
                False,
                f'samples={len(samples)}/{self.stable_trigger_min_samples}',
            )
        if median_distance > (self.trigger_distance_m + self.stable_trigger_median_distance_margin_m):
            return (
                False,
                f'median_dist={median_distance:.3f}>{self.trigger_distance_m + self.stable_trigger_median_distance_margin_m:.3f}',
            )
        if distance_spread > self.stable_trigger_distance_spread_max_m:
            return (
                False,
                f'dist_spread={distance_spread:.3f}>{self.stable_trigger_distance_spread_max_m:.3f}',
            )
        if lateral_spread > self.stable_trigger_lateral_spread_max_m:
            return (
                False,
                f'lat_spread={lateral_spread:.3f}>{self.stable_trigger_lateral_spread_max_m:.3f}',
            )
        return (
            True,
            f'stable median_dist={median_distance:.3f} dist_spread={distance_spread:.3f} '
            f'lat_spread={lateral_spread:.3f} samples={len(samples)}',
        )

    def _publish_stable_trigger_wait(self) -> None:
        if not self.stable_trigger_armed:
            return
        now = self._now_sec()
        if (now - self.last_wait_log_time) < self.progress_log_interval_sec:
            return
        self.last_wait_log_time = now
        samples, median_distance, distance_spread, lateral_spread = (
            self._stable_trigger_metrics(now)
        )
        self._publish_state('WAITING_TRIGGER_STABLE')
        self._publish_feedback(
            'HSV 已触发，等待稳定触发: '
            f'samples={len(samples)}/{self.stable_trigger_min_samples}, '
            f'median_dist={median_distance:.3f}m, '
            f'dist_spread={distance_spread:.3f}m, '
            f'lat_spread={lateral_spread:.3f}m'
        )

    def _disarm_stable_trigger(self, *, clear_reference: bool = True) -> None:
        self.stable_trigger_armed = False
        self.stable_trigger_reason = ''
        self.stable_trigger_arm_time = 0.0
        if clear_reference:
            self.stable_trigger_ref_pose = None
            self.stable_trigger_ref_pole = None
            self.stable_trigger_history.clear()

    def _reset_route_runtime(self) -> None:
        super()._reset_route_runtime()
        self._disarm_stable_trigger()

    def _maybe_activate_stable_trigger(self, now: Optional[float] = None) -> None:
        if not self.route_controller_active:
            return
        if not self.stable_trigger_enabled or not self.stable_trigger_armed:
            return
        if self.triggered or self.route_finished or self.route_failed:
            self._disarm_stable_trigger()
            return

        if now is None:
            now = self._now_sec()

        ready, reason = self._stable_trigger_ready(now)
        if ready:
            trigger_reason = f'{self.stable_trigger_reason} ({reason})'
            self._disarm_stable_trigger(clear_reference=False)
            super()._start_route(trigger_reason, source='hsv')
            return

        if (
            self.stable_trigger_arm_timeout_sec > 1e-6 and
            (now - self.stable_trigger_arm_time) > self.stable_trigger_arm_timeout_sec
        ):
            if (
                self.stable_trigger_timeout_fallback_to_immediate and
                self.last_pole is not None and
                not self._nearest_is_stale() and
                self.last_pole.distance_m <= self.trigger_distance_m
            ):
                trigger_reason = (
                    f'{self.stable_trigger_reason} '
                    f'(stable_timeout_fallback: {self.last_pole_text})'
                )
                self._publish_feedback(
                    f'稳定触发等待超时，回退到即时触发: {reason}'
                )
                self._disarm_stable_trigger(clear_reference=False)
                super()._start_route(trigger_reason, source='hsv')
                return

            self._publish_feedback(f'稳定触发等待超时，放弃本次触发: {reason}')
            self._disarm_stable_trigger()
            return

        self._publish_stable_trigger_wait()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrangePoleRelativeNavStable()
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
