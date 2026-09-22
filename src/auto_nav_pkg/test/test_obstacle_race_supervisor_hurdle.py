import math

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray

from auto_nav_pkg.obstacle_race_supervisor import (
    DetectionState,
    LimitBarTagSample,
    ObstacleRaceSupervisor,
    _rotate_optical_xz_for_yaw_delta,
)


class FakeBuffer:
    def __init__(self, transform):
        self.transform = transform

    def lookup_transform(self, target_frame, source_frame, time):
        assert target_frame == 'camera_color_optical_frame'
        assert source_frame == 'hurdle_right'
        return self.transform


def make_supervisor_stub():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.hurdle_tag_trigger_enabled = True
    node.hurdle_tag_stale_timeout_sec = 0.5
    node.hurdle_tag_max_sample_age_sec = 0.5
    node.hurdle_tag_samples = []
    node.hurdle_tag_frame_id = 'camera_color_optical_frame'
    node.hurdle_tag_child_frame_id = 'hurdle_right'
    node.hurdle_tf_buffer_fallback_enabled = False
    node.hurdle_tf_buffer = None
    node.hurdle_tf_buffer_last_source_stamp_sec = 0.0
    node.last_hurdle_tf_buffer_log_sec = 0.0
    node.hurdle_detection = type(
        'Detection',
        (),
        {'distance_m': None, 'lateral_m': None, 'nearest_stamp_sec': 0.0},
    )()
    node._publish_feedback = lambda message: None
    return node


def test_yaw_compensation_matches_aligned_hurdle_tag_direction():
    x_m, z_m = _rotate_optical_xz_for_yaw_delta(
        0.022,
        1.666,
        math.radians(13.7),
    )

    assert math.isclose(x_m, 0.416, abs_tol=0.01)
    assert math.isclose(z_m, 1.613, abs_tol=0.01)


def test_hurdle_visual_distance_is_not_overwritten_by_tag_distance():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.hurdle_detection = DetectionState()
    node.hurdle_visual_detection = DetectionState()
    node.hurdle_lateral_shift_visual_jump_enabled = True
    node.hurdle_lateral_shift_visual_jump_max_age_sec = 0.50
    node.hurdle_lateral_shift_target_distance_m = 0.50
    node.hurdle_lateral_shift_visual_jump_distance_m = 0.65
    node._now_sec = lambda: 10.0

    nearest = Float32MultiArray()
    nearest.data = [0.63, 0.02]
    node._hurdle_nearest_callback(nearest)
    node._apply_hurdle_tag_measurement(
        10.05,
        tag_x=0.20,
        tag_z=1.20,
        center_x=-0.16,
    )

    assert math.isclose(
        node.hurdle_visual_detection.distance_m,
        0.63,
        abs_tol=1e-6,
    )
    assert math.isclose(node.hurdle_detection.distance_m, 1.20)
    assert node._hurdle_visual_jump_ready(10.10)


def test_hurdle_pre_align_uses_current_aligned_tag_for_shift():
    node = make_supervisor_stub()
    node.state = node.STATE_HURDLE_ALIGN
    node.hurdle_lateral_pre_align_mode = 'fixed_absolute_yaw'
    node.hurdle_lateral_pre_align_use_visual_feedback = False
    node.hurdle_tag_to_center_x_m = -0.36
    node.pose_yaw = math.radians(0.3)
    node.duck_reference_yaw = 0.0
    node.duck_reference_yaw_offset_rad = 0.0
    node.limit_bar_lateral_pre_align_active = True
    node.limit_bar_lateral_pre_align_done = False
    node.limit_bar_lateral_pre_align_start_sec = 9.0
    node.limit_bar_lateral_pre_align_stable_since = 9.8
    node.limit_bar_lateral_pre_align_target_yaw = 0.0
    node.limit_bar_lateral_pre_align_source_odom_yaw = math.radians(-13.7)
    node.limit_bar_lateral_pre_align_source_tag_x_m = 0.022
    node.limit_bar_lateral_pre_align_source_tag_z_m = 1.666
    node.limit_bar_lateral_pre_align_source_center_offset_m = -0.36
    node.slope_align_reason = 'test'
    node.slope_align_enter_time = 0.0
    node.slope_align_stable_since = None
    node.last_slope_align_log_time = 0.0
    node._active_lateral_pre_align_label = lambda: '高墙'
    node._active_lateral_pre_align_tolerance_rad = lambda: math.radians(1.5)
    node._active_lateral_pre_align_timeout_sec = lambda: 8.0
    node._active_lateral_pre_align_max_visual_error_rad = lambda: 0.0
    node._active_lateral_pre_align_hold_sec = lambda: 0.1
    node._active_lateral_pre_align_use_visual_feedback = lambda: False
    node._active_lateral_pre_align_center_offset_m = lambda: -0.36
    node._active_lateral_pre_align_samples = lambda now: [
        LimitBarTagSample(
            received_stamp_sec=now,
            x_m=0.344,
            y_m=0.0,
            z_m=1.480,
            roll_rad=0.0,
            pitch_rad=0.0,
            yaw_rad=0.0,
            frame_id='camera_color_optical_frame',
            child_frame_id='hurdle_right',
        )
    ]
    node._limit_bar_lateral_pre_align_error = (
        lambda sample: ('fixed_absolute_yaw', 0.0, 0.0, 0.0, 0.0)
    )
    node._duck_reference_yaw_text = lambda: '0.0deg'
    node._yaw_text = lambda yaw: '0.0deg'
    feedback = []
    node._publish_feedback = feedback.append
    shift_starts = []
    node._start_limit_bar_lateral_shift = (
        lambda now, reason, yaw_text: shift_starts.append(
            (now, reason, yaw_text)
        ) or True
    )

    node._run_limit_bar_lateral_pre_align(10.0)

    assert math.isclose(node.limit_bar_lateral_shift_center_x_m, -0.016)
    assert math.isclose(node.limit_bar_lateral_shift_center_z_m, 1.480)
    assert shift_starts
    assert any('shift_source=current_aligned_tag' in message for message in feedback)


def test_hurdle_fixed_yaw_uses_locked_tag_without_fresh_tf():
    node = make_supervisor_stub()
    node.state = node.STATE_HURDLE_ALIGN
    node.hurdle_lateral_pre_align_mode = 'fixed_absolute_yaw'
    node.hurdle_lateral_pre_align_use_visual_feedback = False
    node.hurdle_locked_tag_valid = True
    node.hurdle_locked_tag_x_m = 0.357
    node.hurdle_locked_tag_z_m = 1.691

    sample = node._hurdle_fixed_yaw_locked_sample(10.0)

    assert sample is not None
    assert sample.x_m == 0.357
    assert sample.z_m == 1.691
    assert sample.frame_id == 'camera_color_optical_frame'
    assert sample.child_frame_id == 'hurdle_right'


def test_hurdle_fixed_yaw_starts_odom_alignment_without_fresh_tf():
    node = make_supervisor_stub()
    node.state = node.STATE_HURDLE_ALIGN
    node.hurdle_lateral_pre_align_mode = 'fixed_absolute_yaw'
    node.hurdle_lateral_pre_align_use_visual_feedback = False
    node.hurdle_lateral_pre_align_tolerance_rad = math.radians(1.5)
    node.hurdle_lateral_pre_align_timeout_sec = 8.0
    node.hurdle_lateral_pre_align_max_visual_error_rad = 0.0
    node.hurdle_lateral_pre_align_hold_sec = 0.1
    node.hurdle_tag_to_center_x_m = -0.36
    node.hurdle_tag_fixed_reference_yaw = 0.0
    node.hurdle_locked_tag_valid = True
    node.hurdle_locked_tag_x_m = 0.357
    node.hurdle_locked_tag_z_m = 1.691
    node.pose_x = 0.0
    node.pose_y = 0.0
    node.pose_yaw = math.radians(10.0)
    node.pose_stamp_sec = 10.0
    node.pose_stale_timeout_sec = 1.0
    node.limit_bar_lateral_pre_align_target_yaw = None
    node.limit_bar_lateral_pre_align_start_sec = None
    node.limit_bar_lateral_pre_align_stable_since = None
    node.last_slope_align_log_time = 0.0
    node.limit_bar_tag_align_mode = 6
    commands = []
    feedback = []
    node._limit_bar_tag_turn_norms = lambda error: (0.08, -0.04)
    node._publish_step_override = (
        lambda mode, left, right: commands.append((mode, left, right))
    )
    node._publish_feedback = feedback.append

    node._run_limit_bar_lateral_pre_align(10.1)

    assert math.isclose(node.limit_bar_lateral_pre_align_target_yaw, 0.0)
    assert math.isclose(node.limit_bar_lateral_pre_align_source_tag_x_m, 0.357)
    assert math.isclose(node.limit_bar_lateral_pre_align_source_tag_z_m, 1.691)
    assert commands == [(6, 0.08, -0.04)]
    assert not any('等待新鲜 TF' in message for message in feedback)


def test_hurdle_pre_align_stops_while_holding_inside_tolerance():
    node = make_supervisor_stub()
    node.state = node.STATE_HURDLE_ALIGN
    node.hurdle_lateral_pre_align_stop_during_hold_enabled = True
    node.pose_yaw = math.radians(2.0)
    node.limit_bar_lateral_pre_align_target_yaw = 0.0
    node.limit_bar_lateral_pre_align_start_sec = 9.0
    node.limit_bar_lateral_pre_align_stable_since = None
    node.last_slope_align_log_time = 0.0
    node._active_lateral_pre_align_label = lambda: '高墙'
    node._active_lateral_pre_align_tolerance_rad = lambda: math.radians(3.0)
    node._active_lateral_pre_align_timeout_sec = lambda: 8.0
    node._active_lateral_pre_align_max_visual_error_rad = lambda: 0.0
    node._active_lateral_pre_align_hold_sec = lambda: 0.10
    node._active_lateral_pre_align_use_visual_feedback = lambda: False
    node._active_lateral_pre_align_center_offset_m = lambda: -0.36
    sample = LimitBarTagSample(
        received_stamp_sec=10.0,
        x_m=0.10,
        y_m=0.0,
        z_m=1.60,
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=0.0,
        frame_id='camera_color_optical_frame',
        child_frame_id='hurdle_right',
    )
    node._active_lateral_pre_align_samples = lambda now: [sample]
    node._limit_bar_lateral_pre_align_error = (
        lambda current: ('fixed_absolute_yaw', 0.0, 0.0, 0.0, 0.0)
    )
    commands = []
    node._publish_step_override = (
        lambda mode, left, right: commands.append((mode, left, right))
    )

    node._run_limit_bar_lateral_pre_align(10.0)

    assert commands == [(0, 0.0, 0.0)]
    assert node.limit_bar_lateral_pre_align_stable_since == 10.0


def test_hurdle_fine_yaw_turn_does_not_use_overlay_step_size():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.hurdle_fine_yaw_turn_ccw_left_norm = -0.04
    node.hurdle_fine_yaw_turn_ccw_right_norm = 0.08
    node.hurdle_fine_yaw_turn_cw_left_norm = 0.08
    node.hurdle_fine_yaw_turn_cw_right_norm = -0.04
    node.hurdle_fine_yaw_turn_min_scale = 0.50
    node.hurdle_fine_yaw_turn_full_error_rad = math.radians(10.0)

    left_norm, right_norm = node._hurdle_fine_yaw_turn_norms(
        math.radians(5.0)
    )

    assert math.isclose(left_norm, -0.02)
    assert math.isclose(right_norm, 0.04)


def test_hurdle_tf_buffer_fallback_accepts_fresh_transform():
    node = make_supervisor_stub()
    transform = TransformStamped()
    transform.header.frame_id = 'camera_color_optical_frame'
    transform.child_frame_id = 'hurdle_right'
    transform.header.stamp.sec = 100
    transform.header.stamp.nanosec = 100_000_000
    transform.transform.translation.x = 0.25
    transform.transform.translation.y = 0.10
    transform.transform.translation.z = 1.65
    transform.transform.rotation.w = 1.0
    node.hurdle_tf_buffer_fallback_enabled = True
    node.hurdle_tf_buffer = FakeBuffer(transform)

    refreshed = node._refresh_hurdle_tag_from_tf_buffer(100.2)

    assert refreshed
    assert len(node.hurdle_tag_samples) == 1
    assert math.isclose(node.hurdle_detection.distance_m, 1.65)
    assert math.isclose(node.hurdle_detection.lateral_m, 0.25)


def test_hurdle_tf_buffer_fallback_rejects_stale_transform():
    node = make_supervisor_stub()
    transform = TransformStamped()
    transform.header.frame_id = 'camera_color_optical_frame'
    transform.child_frame_id = 'hurdle_right'
    transform.header.stamp.sec = 99
    transform.transform.translation.z = 1.65
    transform.transform.rotation.w = 1.0
    node.hurdle_tf_buffer_fallback_enabled = True
    node.hurdle_tf_buffer = FakeBuffer(transform)

    refreshed = node._refresh_hurdle_tag_from_tf_buffer(100.0)

    assert not refreshed
    assert node.hurdle_tag_samples == []
