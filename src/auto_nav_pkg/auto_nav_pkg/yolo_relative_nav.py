#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import Bool, Float32MultiArray, Int32, String
from yolo_msgs.msg import Detection, DetectionArray


@dataclass
class DetectionCandidate:
    class_key: str
    class_name: str
    score: float
    distance: float
    forward: float
    lateral: float


class YoloRelativeNav(Node):
    """Start with direct walking, then trigger relative waypoints from YOLO range."""

    def __init__(self) -> None:
        super().__init__('yolo_relative_nav')

        self.declare_parameter('yolo_topic', '/yolo/detections_3d')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('arrival_status_topic', '/agent/arrival_status')
        self.declare_parameter('step_override_topic', '/serial_step_override')
        self.declare_parameter('mode_topic', '/dog_mode_current')
        self.declare_parameter(
            'fixed_step_override_state_topic',
            '/motion_bridge/fixed_step_override_enabled',
        )
        self.declare_parameter('state_topic', '/yolo_relative_nav/state')
        self.declare_parameter('feedback_topic', '/yolo_relative_nav/feedback_log')

        self.declare_parameter('pose_topic', '/Odometry')
        self.declare_parameter('pose_topic_type', 'odometry')
        self.declare_parameter('default_frame_id', 'camera_init')
        self.declare_parameter('pose_stale_timeout_sec', 1.2)

        self.declare_parameter('target_class_name', '')
        self.declare_parameter('min_score', 0.25)
        self.declare_parameter('min_same_class_count', 1)
        self.declare_parameter('trigger_distance_m', 1.0)
        self.declare_parameter('trigger_forward_axis', 'x')
        self.declare_parameter('trigger_lateral_axis', 'y')
        self.declare_parameter('trigger_distance_mode', 'forward_axis')
        self.declare_parameter('max_lateral_abs_m', 0.0)

        self.declare_parameter('startup_step_override_enabled', True)
        self.declare_parameter('startup_step_override_mode', 0)
        self.declare_parameter('startup_left_norm', 1.0)
        self.declare_parameter('startup_right_norm', 1.0)
        self.declare_parameter('startup_override_rate_hz', 10.0)
        self.declare_parameter('publish_mode_at_start', True)
        self.declare_parameter('route_mode', 6)
        self.declare_parameter('route_mode_refresh_rate_hz', 5.0)
        self.declare_parameter('disable_fixed_step_override_during_route', True)
        self.declare_parameter('finish_mode', 0)

        self.declare_parameter(
            'waypoint_offsets_xy',
            [0.7, 1.0, -1.4, 1.0, 0.0, 1.0, 0.7, 1.0, -0.7, 1.0, -0.7, 1.0],
        )
        self.declare_parameter('goal_yaw_mode', 'current')
        self.declare_parameter('arrival_tolerance', 0.25)
        self.declare_parameter('goal_timeout_sec', 45.0)
        self.declare_parameter('next_goal_delay_sec', 0.20)
        self.declare_parameter('monitor_rate_hz', 10.0)
        self.declare_parameter('progress_log_interval_sec', 1.0)
        self.declare_parameter('status_heartbeat_sec', 2.0)

        self.yolo_topic = str(self.get_parameter('yolo_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.arrival_status_topic = str(self.get_parameter('arrival_status_topic').value)
        self.step_override_topic = str(self.get_parameter('step_override_topic').value)
        self.mode_topic = str(self.get_parameter('mode_topic').value)
        self.fixed_step_override_state_topic = str(
            self.get_parameter('fixed_step_override_state_topic').value
        )
        self.state_topic = str(self.get_parameter('state_topic').value)
        self.feedback_topic = str(self.get_parameter('feedback_topic').value)

        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.pose_topic_type = str(self.get_parameter('pose_topic_type').value).strip().lower()
        self.default_frame_id = str(self.get_parameter('default_frame_id').value)
        self.pose_stale_timeout_sec = float(self.get_parameter('pose_stale_timeout_sec').value)

        self.target_class_name = str(self.get_parameter('target_class_name').value).strip().lower()
        self.min_score = float(self.get_parameter('min_score').value)
        self.min_same_class_count = max(1, int(self.get_parameter('min_same_class_count').value))
        self.trigger_distance_m = float(self.get_parameter('trigger_distance_m').value)
        self.trigger_forward_axis = self._normalize_axis(
            str(self.get_parameter('trigger_forward_axis').value)
        )
        self.trigger_lateral_axis = self._normalize_axis(
            str(self.get_parameter('trigger_lateral_axis').value)
        )
        self.trigger_distance_mode = (
            str(self.get_parameter('trigger_distance_mode').value).strip().lower()
        )
        self.max_lateral_abs_m = max(0.0, float(self.get_parameter('max_lateral_abs_m').value))

        self.startup_step_override_enabled = bool(
            self.get_parameter('startup_step_override_enabled').value
        )
        self.startup_step_override_mode = int(self.get_parameter('startup_step_override_mode').value)
        self.startup_left_norm = float(self.get_parameter('startup_left_norm').value)
        self.startup_right_norm = float(self.get_parameter('startup_right_norm').value)
        self.startup_override_rate_hz = max(
            1.0,
            float(self.get_parameter('startup_override_rate_hz').value),
        )
        self.publish_mode_at_start = bool(self.get_parameter('publish_mode_at_start').value)
        self.route_mode = int(self.get_parameter('route_mode').value)
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
        self.goal_yaw_mode = str(self.get_parameter('goal_yaw_mode').value).strip().lower()
        self.arrival_tolerance = float(self.get_parameter('arrival_tolerance').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)
        self.next_goal_delay_sec = max(0.0, float(self.get_parameter('next_goal_delay_sec').value))
        self.monitor_rate_hz = float(self.get_parameter('monitor_rate_hz').value)
        self.progress_log_interval_sec = float(
            self.get_parameter('progress_log_interval_sec').value
        )
        self.status_heartbeat_sec = max(0.0, float(self.get_parameter('status_heartbeat_sec').value))

        if self.trigger_distance_mode not in ('forward_axis', 'euclidean'):
            raise RuntimeError('trigger_distance_mode must be "forward_axis" or "euclidean".')
        if self.goal_yaw_mode not in ('current', 'path_heading', 'zero'):
            raise RuntimeError('goal_yaw_mode must be one of: current, path_heading, zero.')

        goal_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            # 2026-05-03 00:23 CST: keep yolo_relative_nav goals latched so a
            # restarted local consumer can still see the latest requested goal.
            # nav_executor/cmd_vel_to_serial subscriptions were relaxed to
            # VOLATILE on the same date, so this publisher remains compatible.
            # Rollback: change to VOLATILE only if another subscriber rejects
            # TRANSIENT_LOCAL publishers.
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
        fixed_step_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.fixed_step_override_state_pub = self.create_publisher(
            Bool,
            self.fixed_step_override_state_topic,
            fixed_step_qos,
        )
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)

        self.pose_sub = self._create_pose_subscription()
        self.yolo_sub = self.create_subscription(
            DetectionArray,
            self.yolo_topic,
            self._yolo_callback,
            10,
        )

        self.have_pose = False
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.pose_frame = ''
        self.last_pose_time = None

        self.triggered = False
        self.route_finished = False
        self.route_failed = False
        self.current_idx = 0
        self.active_goal = False
        self.awaiting_next_goal = False
        self.next_goal_time = 0.0
        self.active_goal_x = 0.0
        self.active_goal_y = 0.0
        self.active_goal_stamp = 0.0
        self.last_state: Optional[str] = None
        self.last_feedback: Optional[str] = None
        self.last_progress_log_time = 0.0
        self.last_yolo_wait_log_time = 0.0
        self.last_status_heartbeat_time = 0.0
        self.last_yolo_msg_time: Optional[float] = None
        self.last_yolo_detection_count = 0
        self.last_yolo_candidate_count = 0
        self.last_yolo_nearest_text = 'none'

        self.monitor_timer = self.create_timer(
            1.0 / max(self.monitor_rate_hz, 1.0),
            self._monitor_loop,
        )
        self.startup_override_timer = None
        if self.startup_step_override_enabled:
            self.startup_override_timer = self.create_timer(
                1.0 / self.startup_override_rate_hz,
                self._publish_startup_step_override,
            )
        self.route_mode_timer = self.create_timer(
            1.0 / self.route_mode_refresh_rate_hz,
            self._refresh_route_mode,
        )

        self._publish_fixed_step_override_state(True)
        if self.publish_mode_at_start:
            self._publish_mode(self.startup_step_override_mode)
        if self.startup_step_override_enabled:
            self._publish_startup_step_override()

        self._publish_state('WAITING_YOLO_TRIGGER')
        self._publish_feedback(
            f'等待 YOLO 触发: topic={self.yolo_topic}, target_class={self.target_class_name or "ANY"}, '
            f'min_score={self.min_score:.2f}, distance<={self.trigger_distance_m:.2f}m, '
            f'axis={self.trigger_forward_axis}, route_mode={self.route_mode}, '
            f'offsets={self.waypoint_offsets}'
        )

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

    def _pose_with_covariance_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self._update_pose(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._yaw_from_quaternion(msg.pose.pose.orientation),
            msg.header.frame_id,
        )

    def _update_pose(self, x: float, y: float, yaw: float, frame_id: str) -> None:
        self.pose_x = float(x)
        self.pose_y = float(y)
        self.pose_yaw = float(yaw)
        self.pose_frame = frame_id or self.default_frame_id
        self.last_pose_time = self.get_clock().now()
        self.have_pose = True

    def _yolo_callback(self, msg: DetectionArray) -> None:
        if self.triggered or self.route_finished or self.route_failed:
            return

        now = self._now_sec()
        self.last_yolo_msg_time = now
        self.last_yolo_detection_count = len(msg.detections)
        # 2026-05-03 00:23 CST: filter by class/score before distance.
        # Field test showed the obstacle pole class is "montant" and the
        # 3D topic may contain multiple same-class rods in one frame.
        # Rollback: set target_class_name="" in YAML to accept any class again.
        candidates = self._collect_candidates(msg.detections)
        self.last_yolo_candidate_count = len(candidates)
        if not candidates:
            self.last_yolo_nearest_text = 'none'
            if msg.detections:
                if (now - self.last_yolo_wait_log_time) >= self.progress_log_interval_sec:
                    self.last_yolo_wait_log_time = now
                    self._publish_feedback(
                        '收到 YOLO 检测，但没有可用的 3D 距离候选；请确认 yolo_topic 使用 '
                        '/yolo/detections_3d，且 trigger_forward_axis/target_class_name/min_score '
                        '与检测结果一致。'
                    )
            return

        nearest = min(candidates, key=lambda item: item.distance)
        self.last_yolo_nearest_text = (
            f'class={nearest.class_name}, dist={nearest.distance:.2f}m, '
            f'forward={nearest.forward:.2f}m, lateral={nearest.lateral:.2f}m'
        )
        if nearest.distance > self.trigger_distance_m:
            if (now - self.last_yolo_wait_log_time) >= self.progress_log_interval_sec:
                self.last_yolo_wait_log_time = now
                self._publish_feedback(
                    f'YOLO 已看到目标但未触发: class={nearest.class_name}, '
                    f'dist={nearest.distance:.2f}m, forward={nearest.forward:.2f}m, '
                    f'threshold={self.trigger_distance_m:.2f}m'
                )
            return

        self.triggered = True
        self.active_goal = False
        self.awaiting_next_goal = False
        self.current_idx = 0
        self._cancel_startup_override_timer()
        self._clear_step_override()
        self._set_route_mode_active(True)
        self._publish_feedback(
            f'YOLO 触发导航: class={nearest.class_name}, score={nearest.score:.2f}, '
            f'dist={nearest.distance:.2f}m, forward={nearest.forward:.2f}m, '
            f'lateral={nearest.lateral:.2f}m, route_mode={self.route_mode}'
        )

        if self._pose_ready_for_goal():
            self._send_current_goal()
        else:
            self._publish_state('WAITING_POSE')
            self._publish_feedback('YOLO 已触发，但还没有可用 Odometry，等待定位后发布第一个目标。')

    def _collect_candidates(self, detections: Sequence[Detection]) -> List[DetectionCandidate]:
        raw_candidates: List[DetectionCandidate] = []
        class_counts = {}

        for detection in detections:
            if self.target_class_name and detection.class_name.strip().lower() != self.target_class_name:
                continue
            if detection.score < self.min_score:
                continue

            candidate = self._candidate_from_detection(detection)
            if candidate is None:
                continue

            raw_candidates.append(candidate)
            class_counts[candidate.class_key] = class_counts.get(candidate.class_key, 0) + 1

        if self.min_same_class_count <= 1:
            return raw_candidates

        return [
            candidate
            for candidate in raw_candidates
            if class_counts.get(candidate.class_key, 0) >= self.min_same_class_count
        ]

    def _candidate_from_detection(self, detection: Detection) -> Optional[DetectionCandidate]:
        bbox3d = detection.bbox3d
        position = bbox3d.center.position
        forward = self._axis_value(position, self.trigger_forward_axis)
        lateral = self._axis_value(position, self.trigger_lateral_axis)

        if not self._finite(forward) or not self._finite(lateral):
            return None
        if forward <= 0.0:
            return None
        if self.max_lateral_abs_m > 0.0 and abs(lateral) > self.max_lateral_abs_m:
            return None

        euclidean = math.sqrt(
            position.x * position.x + position.y * position.y + position.z * position.z
        )
        if not self._finite(euclidean) or euclidean <= 0.0:
            return None

        distance = forward if self.trigger_distance_mode == 'forward_axis' else euclidean
        class_name = detection.class_name or f'class_{detection.class_id}'
        class_key = class_name.strip().lower() or str(detection.class_id)
        return DetectionCandidate(
            class_key=class_key,
            class_name=class_name,
            score=float(detection.score),
            distance=float(distance),
            forward=float(forward),
            lateral=float(lateral),
        )

    def _monitor_loop(self) -> None:
        self._publish_status_heartbeat()

        if self.route_finished or self.route_failed:
            return

        self._refresh_route_mode()

        if self.triggered and not self.active_goal and not self.awaiting_next_goal:
            if self.current_idx < len(self.waypoint_offsets):
                if self._pose_ready_for_goal():
                    self._send_current_goal()
                else:
                    self._publish_state('WAITING_POSE')
            return

        if self.awaiting_next_goal:
            if self._now_sec() < self.next_goal_time:
                return
            self.awaiting_next_goal = False
            if self.current_idx >= len(self.waypoint_offsets):
                self._finish_route()
                return
            if self._pose_ready_for_goal():
                self._send_current_goal()
            else:
                self._publish_state('WAITING_POSE')
            return

        if not self.active_goal:
            return

        if not self.have_pose:
            self._publish_state('WAITING_POSE')
            return

        if self._pose_is_stale():
            self._publish_state('POSE_STALE')
            if self.goal_timeout_sec > 0.0 and (
                self._now_sec() - self.active_goal_stamp
            ) > self.goal_timeout_sec:
                self._handle_goal_failed('goal_timeout pose_stale')
            return

        dist = math.hypot(self.active_goal_x - self.pose_x, self.active_goal_y - self.pose_y)
        self._publish_progress(
            f'waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)} '
            f'dist={dist:.3f}m pose=({self.pose_x:.3f},{self.pose_y:.3f}) '
            f'goal=({self.active_goal_x:.3f},{self.active_goal_y:.3f})'
        )

        if dist <= self.arrival_tolerance:
            self._handle_goal_arrived(dist)
            return

        if self.goal_timeout_sec > 0.0 and (
            self._now_sec() - self.active_goal_stamp
        ) > self.goal_timeout_sec:
            self._handle_goal_failed(f'goal_timeout dist={dist:.3f}m')

    def _send_current_goal(self) -> None:
        if self.current_idx >= len(self.waypoint_offsets):
            self._finish_route()
            return

        dx, dy = self.waypoint_offsets[self.current_idx]
        goal_x = self.pose_x + dx
        goal_y = self.pose_y + dy
        goal_yaw = self._goal_yaw(dx, dy)
        self._set_route_mode_active(True)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.pose_frame or self.default_frame_id
        msg.pose.position.x = goal_x
        msg.pose.position.y = goal_y
        msg.pose.position.z = 0.0
        msg.pose.orientation = self._quaternion_from_yaw(goal_yaw)

        self.active_goal_x = goal_x
        self.active_goal_y = goal_y
        self.active_goal_stamp = self._now_sec()
        self.active_goal = True
        self.awaiting_next_goal = False
        self.goal_pub.publish(msg)
        self._publish_state('NAVIGATING')
        self._publish_feedback(
            f'发送目标点[{self.current_idx + 1}/{len(self.waypoint_offsets)}]: '
            f'offset=({dx:.2f},{dy:.2f}), goal=({goal_x:.3f},{goal_y:.3f}), '
            f'frame={msg.header.frame_id}, yaw={math.degrees(goal_yaw):.1f}deg, '
            f'mode={self.route_mode}'
        )

    def _handle_goal_arrived(self, dist: float) -> None:
        self.active_goal = False
        self._publish_arrival(True)
        self._publish_feedback(
            f'到达目标点[{self.current_idx + 1}/{len(self.waypoint_offsets)}]: dist={dist:.3f}m'
        )
        self.current_idx += 1
        if self.current_idx >= len(self.waypoint_offsets):
            self._finish_route()
            return

        self.awaiting_next_goal = True
        self.next_goal_time = self._now_sec() + self.next_goal_delay_sec
        self._publish_state('ARRIVED_WAIT_NEXT')

    def _handle_goal_failed(self, reason: str) -> None:
        self.active_goal = False
        self.route_failed = True
        self._publish_arrival(False)
        self._clear_step_override()
        self._set_route_mode_active(False)
        self._publish_mode(self.finish_mode)
        self._publish_state('FAILED')
        self._publish_feedback(
            f'导航失败: waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)}, reason={reason}'
        )

    def _finish_route(self) -> None:
        if self.route_finished:
            return
        self.active_goal = False
        self.awaiting_next_goal = False
        self.route_finished = True
        self._clear_step_override()
        self._set_route_mode_active(False)
        self._publish_mode(self.finish_mode)
        self._publish_state('FINISHED')
        self._publish_feedback('6 个相对目标点已全部完成。')

    def _pose_ready_for_goal(self) -> bool:
        if not self.have_pose:
            return False
        return not self._pose_is_stale()

    def _pose_is_stale(self) -> bool:
        if self.last_pose_time is None or self.pose_stale_timeout_sec <= 0.0:
            return False
        age = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        return age > self.pose_stale_timeout_sec

    def _goal_yaw(self, dx: float, dy: float) -> float:
        if self.goal_yaw_mode == 'zero':
            return 0.0
        if self.goal_yaw_mode == 'path_heading' and math.hypot(dx, dy) > 1e-6:
            return math.atan2(dy, dx)
        return self.pose_yaw

    def _publish_startup_step_override(self) -> None:
        if self.triggered or self.route_finished or self.route_failed:
            return
        self._publish_step_override(
            self.startup_step_override_mode,
            self.startup_left_norm,
            self.startup_right_norm,
        )

    def _cancel_startup_override_timer(self) -> None:
        if self.startup_override_timer is not None:
            self.startup_override_timer.cancel()
            self.startup_override_timer = None

    def _refresh_route_mode(self) -> None:
        if not self.triggered or self.route_finished or self.route_failed:
            return
        self._set_route_mode_active(True)

    def _set_route_mode_active(self, active: bool) -> None:
        if active:
            if self.disable_fixed_step_override_during_route:
                # 2026-05-03 00:23 CST: route_mode=6 is required during the
                # six relative goals, but cmd_vel_to_serial also has a legacy
                # fixed mode=6 spin mapping. Disable only that legacy fixed
                # mapping while preserving normal nav_executor /race_cmd_vel
                # control. Rollback: set disable_fixed_step_override_during_route=false.
                self._publish_fixed_step_override_state(False)
            self._publish_mode(self.route_mode)
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

        # 2026-05-03 00:31 CST: feedback/state are event topics, so late
        # `ros2 topic echo` subscribers can otherwise see nothing while the
        # node is alive and waiting. Publish a compact heartbeat for field
        # diagnostics. Rollback: set status_heartbeat_sec=0.0 in YAML to
        # disable periodic diagnostic logs without changing code.
        state_msg = String()
        state_msg.data = self.last_state or 'UNKNOWN'
        self.state_pub.publish(state_msg)

        if self.last_yolo_msg_time is None:
            yolo_age_text = 'none'
        else:
            yolo_age_text = f'{now - self.last_yolo_msg_time:.2f}s'

        feedback = (
            f'HEARTBEAT state={state_msg.data}, triggered={self.triggered}, '
            f'route_finished={self.route_finished}, route_failed={self.route_failed}, '
            f'active_goal={self.active_goal}, waypoint={self.current_idx + 1}/{len(self.waypoint_offsets)}, '
            f'yolo_age={yolo_age_text}, detections={self.last_yolo_detection_count}, '
            f'candidates={self.last_yolo_candidate_count}, nearest=({self.last_yolo_nearest_text})'
        )
        msg = String()
        msg.data = feedback
        self.feedback_pub.publish(msg)
        self.get_logger().info(feedback)

    def shutdown(self) -> None:
        self._cancel_startup_override_timer()
        self._clear_step_override()
        self._set_route_mode_active(False)
        self._publish_mode(self.finish_mode)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _parse_offsets(raw_value) -> List[Tuple[float, float]]:
        values = [float(item) for item in raw_value]
        if len(values) != 12 or len(values) % 2 != 0:
            raise RuntimeError('waypoint_offsets_xy must contain 12 numbers for 6 x/y offsets.')
        return [(values[idx], values[idx + 1]) for idx in range(0, len(values), 2)]

    @staticmethod
    def _normalize_axis(axis: str) -> str:
        normalized = axis.strip().lower()
        if normalized not in ('x', 'y', 'z'):
            raise RuntimeError('axis parameter must be one of: x, y, z.')
        return normalized

    @staticmethod
    def _axis_value(position, axis: str) -> float:
        if axis == 'x':
            return float(position.x)
        if axis == 'y':
            return float(position.y)
        return float(position.z)

    @staticmethod
    def _finite(value: float) -> bool:
        return math.isfinite(float(value))

    @staticmethod
    def _yaw_from_quaternion(q: Quaternion) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YoloRelativeNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
