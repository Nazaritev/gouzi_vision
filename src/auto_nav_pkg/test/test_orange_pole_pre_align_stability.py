import math

from auto_nav_pkg.orange_pole_relative_nav import (
    AprilTagSnapshot,
    OrangePoleRelativeNav,
)


def make_nav_stub():
    node = OrangePoleRelativeNav.__new__(OrangePoleRelativeNav)
    node.pole_pre_align_pose_filter_window_size = 5
    node.pole_pre_align_pose_filter_min_samples = 3
    node._reset_pole_pre_align_runtime()
    node.pole_pre_align_active = True
    node.pole_pre_align_angle_metric = 'pose_pitch'
    node.pole_pre_align_target_pitch_rad = 0.0
    node.pole_pre_align_fixed_yaw_rad = math.radians(-90.0)
    node.pole_pre_align_pose_control_sign = -1.0
    node.pose_yaw = 0.0
    node.pole_pre_align_angle_tolerance_rad = math.radians(2.0)
    node.pole_pre_align_angle_exit_tolerance_rad = math.radians(3.5)
    node.pole_pre_align_reverse_cooldown_sec = 0.25
    node.pole_pre_align_pose_turn_scale = 0.70
    node.pole_pre_align_fine_angle_threshold_rad = math.radians(8.0)
    node.pole_pre_align_fine_ccw_left_norm = -0.025
    node.pole_pre_align_fine_ccw_right_norm = 0.050
    node.pole_pre_align_fine_cw_left_norm = 0.050
    node.pole_pre_align_fine_cw_right_norm = -0.025
    node._clamp_norm = lambda value: max(-1.0, min(1.0, value))
    return node


def make_tag(stamp_sec, pitch_deg):
    return AprilTagSnapshot(
        frame_id='camera_color_optical_frame',
        child_frame_id='role',
        stamp_sec=stamp_sec,
        received_sec=stamp_sec,
        x_m=0.0,
        y_m=0.0,
        z_m=0.8,
        roll_rad=0.0,
        pitch_rad=math.radians(pitch_deg),
        yaw_rad=0.0,
    )


def test_pre_align_pitch_filter_waits_for_minimum_samples_then_uses_median():
    node = make_nav_stub()

    assert node._pole_pre_align_filtered_pitch_err(make_tag(1.0, 8.0)) is None
    assert node._pole_pre_align_filtered_pitch_err(make_tag(1.1, -1.0)) is None

    filtered = node._pole_pre_align_filtered_pitch_err(make_tag(1.2, 2.0))
    assert math.isclose(math.degrees(filtered), 2.0, abs_tol=1e-6)

    node._pole_pre_align_filtered_pitch_err(make_tag(1.3, 20.0))
    filtered = node._pole_pre_align_filtered_pitch_err(make_tag(1.4, 1.0))
    assert math.isclose(math.degrees(filtered), 2.0, abs_tol=1e-6)


def test_pre_align_hysteresis_keeps_hold_until_exit_tolerance():
    node = make_nav_stub()

    ok, tolerance = node._pole_pre_align_error_within_hold(math.radians(2.4))
    assert not ok
    assert math.isclose(tolerance, math.radians(2.0))

    node.pole_pre_align_stable_since = 10.0
    ok, tolerance = node._pole_pre_align_error_within_hold(math.radians(3.0))
    assert ok
    assert math.isclose(tolerance, math.radians(3.5))

    ok, _ = node._pole_pre_align_error_within_hold(math.radians(3.8))
    assert not ok


def test_pre_align_reverse_cooldown_inserts_zero_before_opposite_turn():
    node = make_nav_stub()

    left, right, fine_turn, cooldown = node._pole_pre_align_guarded_turn_norms(
        10.0,
        math.radians(5.0),
    )
    assert (left, right) == (-0.025, 0.050)
    assert fine_turn
    assert cooldown == 0.0

    left, right, fine_turn, cooldown = node._pole_pre_align_guarded_turn_norms(
        10.1,
        math.radians(-5.0),
    )
    assert (left, right) == (0.0, 0.0)
    assert not fine_turn
    assert math.isclose(cooldown, 0.25, abs_tol=1e-6)

    left, right, fine_turn, cooldown = node._pole_pre_align_guarded_turn_norms(
        10.2,
        math.radians(-5.0),
    )
    assert (left, right) == (0.0, 0.0)
    assert cooldown > 0.0

    left, right, fine_turn, cooldown = node._pole_pre_align_guarded_turn_norms(
        10.4,
        math.radians(-5.0),
    )
    assert (left, right) == (0.050, -0.025)
    assert fine_turn
    assert cooldown == 0.0


def test_pose_turn_scale_damps_tag_alignment_without_slowing_shift_turns():
    node = make_nav_stub()

    left, right, fine_turn, cooldown = node._pole_pre_align_guarded_pose_turn_norms(
        10.0,
        math.radians(5.0),
    )
    assert math.isclose(left, -0.0175, abs_tol=1e-9)
    assert math.isclose(right, 0.035, abs_tol=1e-9)
    assert fine_turn
    assert cooldown == 0.0

    node._reset_pole_pre_align_turn_guard()
    left, right, fine_turn, cooldown = node._pole_pre_align_guarded_turn_norms(
        10.0,
        math.radians(5.0),
    )
    assert (left, right) == (-0.025, 0.050)
    assert fine_turn
    assert cooldown == 0.0


def test_fixed_absolute_yaw_ignores_tag_pose_and_locks_global_reference():
    node = make_nav_stub()
    node.pole_pre_align_angle_metric = 'fixed_absolute_yaw'
    node.pose_yaw = math.radians(-84.0)
    tag = make_tag(1.0, 35.0)

    angle_error = node._pole_pre_align_pitch_err(tag)
    opposite_tag_pose_error = node._pole_pre_align_pitch_err(make_tag(2.0, -35.0))

    assert math.isclose(math.degrees(angle_error), -6.0, abs_tol=1e-6)
    assert math.isclose(opposite_tag_pose_error, angle_error, abs_tol=1e-9)
    assert math.isclose(
        node._pole_pre_align_angle_control_error(angle_error),
        angle_error,
        abs_tol=1e-9,
    )
    assert math.isclose(
        node._pole_pre_align_reference_yaw(),
        math.radians(-90.0),
        abs_tol=1e-9,
    )
    assert 'global_yaw=-90.00deg' in node._pole_pre_align_angle_target_text()


def test_fixed_absolute_yaw_is_used_for_locked_route_frame():
    node = make_nav_stub()
    node.pole_pre_align_angle_metric = 'fixed_absolute_yaw'
    node.pole_pre_align_done = True
    node.pole_pre_align_base_yaw = math.radians(-90.0)
    node.pose_x = 1.0
    node.pose_y = 2.0
    node.pose_yaw = math.radians(-88.4)
    node.pose_frame = 'camera_init'
    node.default_frame_id = 'map'
    node.route_anchor_ready = False
    node.route_reference_mode = 'locked_route_frame'
    node.require_fresh_pole_for_route_start = False
    node._uses_route_anchor_reference = lambda: True
    node._pose_ready_for_goal = lambda: True
    node._current_route_start_source = lambda: 'apriltag'
    feedback = []
    node._publish_feedback = feedback.append

    assert node._ensure_route_anchor()
    assert math.isclose(node.route_anchor_yaw, math.radians(-90.0), abs_tol=1e-9)
    assert 'yaw_source=fixed_pre_align' in feedback[-1]


def test_fixed_pole_yaw_adds_runtime_route_zero_when_enabled():
    node = make_nav_stub()
    node.pole_pre_align_angle_metric = 'fixed_absolute_yaw'
    node.runtime_route_yaw_reference_enabled = True
    node.route_yaw_reference = math.radians(11.3)
    node.route_yaw_reference_timeout_sec = 0.0
    node.route_yaw_reference_stamp_sec = 10.0
    node.pose_yaw = math.radians(-75.0)

    target_yaw = node._pole_pre_align_reference_yaw()
    angle_error = node._pole_pre_align_pitch_err(make_tag(1.0, 35.0))

    assert math.isclose(math.degrees(target_yaw), -78.7, abs_tol=1e-9)
    assert math.isclose(math.degrees(angle_error), -3.7, abs_tol=1e-9)
    assert 'route_zero=+11.30deg' in node._pole_pre_align_angle_target_text()


def test_fixed_pole_yaw_falls_back_when_runtime_reference_is_disabled():
    node = make_nav_stub()
    node.pole_pre_align_angle_metric = 'fixed_absolute_yaw'
    node.runtime_route_yaw_reference_enabled = False
    node.route_yaw_reference = math.radians(11.3)
    node.route_yaw_reference_timeout_sec = 0.0

    assert math.isclose(
        node._pole_pre_align_reference_yaw(),
        math.radians(-90.0),
        abs_tol=1e-9,
    )


def configure_precision_turn(node):
    node.pole_pre_align_precision_turn_enabled = True
    node.pose_yaw_rate_rad_s = 0.0
    node.pole_pre_align_turn_complete_yaw_rate_rad_s = math.radians(6.0)
    node.pole_pre_align_turn_yaw_damping_sec = 0.18
    node.pole_pre_align_turn_brake_yaw_rate_rad_s = math.radians(12.0)
    node.pole_pre_align_turn_far_error_rad = math.radians(35.0)
    node.pole_pre_align_turn_far_scale = 1.35
    node.pole_pre_align_reuse_existing_turn_steps = False
    node.pole_pre_align_ccw_left_norm = -0.10
    node.pole_pre_align_ccw_right_norm = 0.25
    node.pole_pre_align_cw_left_norm = 0.25
    node.pole_pre_align_cw_right_norm = -0.10
    node.pole_pre_align_turn_scale_enabled = True
    node.pole_pre_align_turn_min_scale = 0.35
    node.pole_pre_align_turn_yaw_gain_per_rad = 0.65
    node.pole_pre_align_turn_min_abs_angular_z = 0.035
    node.pole_pre_align_turn_max_abs_angular_z = 0.18


def test_precision_turn_hold_requires_low_yaw_rate():
    node = make_nav_stub()
    configure_precision_turn(node)
    node.pose_yaw_rate_rad_s = math.radians(8.0)

    ready, tolerance = node._pole_pre_align_error_within_hold(
        math.radians(1.0)
    )

    assert not ready
    assert math.isclose(tolerance, math.radians(2.0))


def test_precision_turn_brakes_before_reversing_near_target():
    node = make_nav_stub()
    configure_precision_turn(node)
    node.pose_yaw_rate_rad_s = math.radians(20.0)

    command = node._pole_pre_align_guarded_turn_norms(
        10.0,
        math.radians(3.0),
    )

    assert command == (0.0, 0.0, False, 0.0)


def test_precision_turn_scales_up_only_far_from_target():
    node = make_nav_stub()
    configure_precision_turn(node)

    left_norm, right_norm, fine_turn, cooldown = (
        node._pole_pre_align_guarded_turn_norms(
            10.0,
            math.radians(90.0),
        )
    )

    assert not fine_turn
    assert cooldown == 0.0
    assert math.isclose(left_norm, -0.0972, abs_tol=1e-9)
    assert math.isclose(right_norm, 0.243, abs_tol=1e-9)
