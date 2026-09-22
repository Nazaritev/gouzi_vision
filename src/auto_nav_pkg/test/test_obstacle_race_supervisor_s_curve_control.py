import math

from auto_nav_pkg.lateral_s_curve import build_lateral_s_curve_plan
from auto_nav_pkg.obstacle_race_supervisor import ObstacleRaceSupervisor


def make_control_stub():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.state = node.STATE_LIMIT_BAR_LATERAL_SHIFT
    node.limit_bar_lateral_shift_phase = 's_curve'
    node.limit_bar_lateral_shift_phase_start_sec = 1.0
    node.limit_bar_lateral_shift_base_yaw = 0.0
    node.limit_bar_lateral_shift_next_action = 'hurdle_wait_jump'
    node.limit_bar_lateral_shift_walk_mode = 0
    node.duck_centerline_odom_forward_axis = 'y'
    node.lateral_s_curve_control_active = True
    node.lateral_s_curve_control_plan = build_lateral_s_curve_plan(
        forward_m=1.0,
        lateral_m=-0.10,
        min_forward_m=0.35,
        max_abs_lateral_m=0.45,
        max_abs_heading_rad=math.radians(25.0),
        max_abs_curvature_m_inv=2.5,
    )
    node.lateral_s_curve_control_start_x = 0.0
    node.lateral_s_curve_control_start_y = 0.0
    node.lateral_s_curve_control_start_sec = 1.0
    node.lateral_s_curve_control_arrival_sec = None
    node.lateral_s_curve_control_arrival_forward_m = None
    node.lateral_s_curve_control_arrival_cross_error_m = None
    node.lateral_s_curve_control_completion_max_forward_m = None
    node.lateral_s_curve_control_completion_since_sec = None
    node.lateral_s_curve_control_completion_reason = ''
    node.lateral_s_curve_control_cruise_left_norm = 0.40
    node.lateral_s_curve_control_cruise_right_norm = 0.40
    node.lateral_s_curve_control_slow_left_norm = 0.25
    node.lateral_s_curve_control_slow_right_norm = 0.25
    node.lateral_s_curve_control_slowdown_distance_m = 0.25
    node.lateral_s_curve_control_lookahead_m = 0.12
    node.lateral_s_curve_control_heading_gain_norm_per_rad = 0.90
    node.lateral_s_curve_control_cross_track_gain_rad_per_m = 1.50
    node.lateral_s_curve_control_cross_track_max_heading_rad = math.radians(10.0)
    node.lateral_s_curve_control_max_delta_norm = 0.18
    node.lateral_s_curve_control_walk_yaw_gate_rad = math.radians(12.0)
    node.lateral_s_curve_control_forward_tolerance_m = 0.05
    node.lateral_s_curve_control_lateral_tolerance_m = 0.10
    node.lateral_s_curve_control_yaw_tolerance_rad = math.radians(2.0)
    node.lateral_s_curve_control_arrival_settle_sec = 0.0
    node.lateral_s_curve_control_completion_hold_sec = 0.20
    node.lateral_s_curve_control_completion_max_retreat_m = 0.0
    node.lateral_s_curve_control_completion_max_lateral_drift_m = 0.0
    node.lateral_s_curve_control_completion_recovery_enabled = False
    node.lateral_s_curve_control_completion_recovery_left_norm = 0.15
    node.lateral_s_curve_control_completion_recovery_right_norm = 0.15
    node.lateral_s_curve_control_completion_recovery_yaw_gain_norm_per_rad = 0.45
    node.lateral_s_curve_control_completion_recovery_yaw_max_delta_norm = 0.06
    node.lateral_s_curve_control_completion_recovery_forward_tolerance_m = 0.015
    node.lateral_s_curve_control_completion_recovery_yaw_tolerance_rad = (
        math.radians(10.0)
    )
    node.lateral_s_curve_control_completion_recovery_max_extra_forward_m = 0.03
    node.lateral_s_curve_control_completion_recovery_max_abs_lateral_m = 0.15
    node.lateral_s_curve_control_completion_recovery_active = False
    node.lateral_s_curve_control_completion_recovery_target_forward_m = None
    node.lateral_s_curve_control_completion_recovery_trigger = ''
    node.hurdle_lateral_s_curve_visual_jump_confirm_samples = 1
    node.hurdle_lateral_s_curve_visual_jump_confirm_duration_sec = 0.0
    node.hurdle_lateral_s_curve_visual_jump_max_remaining_m = 0.0
    node.hurdle_lateral_s_curve_visual_jump_tag_max_delta_m = 0.0
    node.hurdle_lateral_s_curve_visual_close_sample_count = 0
    node.hurdle_lateral_s_curve_visual_close_start_sec = None
    node.last_hurdle_lateral_s_curve_visual_guard_log_sec = 0.0
    node.hurdle_lateral_s_curve_final_align_independent_turn_enabled = False
    node.hurdle_lateral_s_curve_visual_completion_ignore_plan_overshoot_enabled = False
    node.hurdle_lateral_s_curve_visual_completion_max_plan_overshoot_m = 0.25
    node.lateral_s_curve_control_timeout_sec = 3.0
    node.lateral_s_curve_control_max_forward_overshoot_m = 0.15
    node.lateral_s_curve_control_max_abs_cross_track_m = 0.20
    node.limit_bar_lateral_shift_duck_entry_yaw_gate_rad = math.radians(2.0)
    node.limit_bar_lateral_shift_final_yaw_gate_rad = math.radians(6.0)
    node.limit_bar_lateral_precision_turn_enabled = False
    node.pose_x = 0.0
    node.pose_y = 0.20
    node.pose_yaw = 0.0
    node.pose_yaw_rate_rad_s = 0.0
    node.pose_yaw_rate_raw_rad_s = 0.0
    node.last_limit_bar_lateral_shift_log_sec = 0.0
    node._pose_is_fresh = lambda now: True
    node._hurdle_visual_jump_ready = lambda now: False
    node._publish_feedback_messages = []
    node._publish_feedback = node._publish_feedback_messages.append
    node._commands = []
    node._publish_step_override = (
        lambda mode, left, right: node._commands.append((mode, left, right))
    )
    node._fail_messages = []
    node._fail = node._fail_messages.append
    node._hurdle_entries = []
    node._enter_hurdle_wait_jump = (
        lambda now, reason: node._hurdle_entries.append((now, reason))
    )
    node.limit_bar_lateral_shift_reason = 'test'
    node.hurdle_visual_detection = type(
        'Detection',
        (),
        {
            'distance_m': 0.5,
            'triggered': False,
            'trigger_start_stamp_sec': 0.0,
            'nearest_stamp_sec': 0.0,
        },
    )()
    node.hurdle_lateral_shift_visual_jump_max_age_sec = 0.0
    node.hurdle_lateral_shift_visual_jump_distance_m = 0.65
    node._limit_bar_lateral_yaw_rate_ready = lambda: True
    node._distance_text = (
        lambda distance: 'unknown' if distance is None else f'{distance:.3f}m'
    )
    return node


def make_upstairs_control_stub():
    node = make_control_stub()
    node.limit_bar_lateral_shift_next_action = 'upstairs_step_once'
    node.upstairs_lateral_s_curve_control_cruise_left_norm = 0.40
    node.upstairs_lateral_s_curve_control_cruise_right_norm = 0.40
    node.upstairs_lateral_s_curve_control_slow_left_norm = 0.25
    node.upstairs_lateral_s_curve_control_slow_right_norm = 0.25
    node.upstairs_lateral_s_curve_control_slowdown_distance_m = 0.25
    node.upstairs_lateral_s_curve_control_timeout_sec = 12.0
    node.upstairs_lateral_s_curve_control_max_forward_overshoot_m = 0.20
    node.upstairs_lateral_s_curve_control_distance_max_remaining_m = 0.25
    node.upstairs_lateral_s_curve_control_precreep_yaw_gate_rad = math.radians(
        4.0
    )
    node.upstairs_lateral_s_curve_control_final_align_norm = 0.08
    node.upstairs_final_confirm_creep_max_extra_m = 0.18
    node.upstairs_final_confirm_creep_left_norm = 0.15
    node.upstairs_final_confirm_creep_right_norm = 0.15
    node.upstairs_final_forward_realign_gate_rad = math.radians(15.0)
    node.upstairs_final_yaw_complete_gate_rad = math.radians(1.0)
    node.limit_bar_tag_align_mode = 6
    node.upstairs_tag_yaw_reference_mode = 'fixed_absolute'
    node.upstairs_final_align_latched_reason = ''
    node._upstairs_live_tag_alignment = lambda now: None
    node._upstairs_final_approach_ready = (
        lambda now, live_alignment: (False, 'distance_not_ready')
    )
    node._upstairs_done = []
    node._finish_upstairs_done = (
        lambda now, reason, final_along_m=None, raw_final_along_m=None,
        final_remaining_m=None: node._upstairs_done.append(
            (
                now,
                reason,
                final_along_m,
                raw_final_along_m,
                final_remaining_m,
            )
        )
    )
    return node


def make_limit_bar_control_stub():
    node = make_control_stub()
    node.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'
    node.limit_bar_lateral_shift_side_yaw = -math.pi / 2.0
    node.lateral_s_curve_control_plan = build_lateral_s_curve_plan(
        forward_m=0.55,
        lateral_m=-0.10,
        min_forward_m=0.35,
        max_abs_lateral_m=0.45,
        max_abs_heading_rad=math.radians(30.0),
        max_abs_curvature_m_inv=2.5,
    )
    node.limit_bar_lateral_s_curve_control_cruise_left_norm = 0.32
    node.limit_bar_lateral_s_curve_control_cruise_right_norm = 0.32
    node.limit_bar_lateral_s_curve_control_slow_left_norm = 0.20
    node.limit_bar_lateral_s_curve_control_slow_right_norm = 0.20
    node.limit_bar_lateral_s_curve_control_slowdown_distance_m = 0.18
    node.limit_bar_lateral_s_curve_control_timeout_sec = 8.0
    node.limit_bar_lateral_s_curve_fail_open_enabled = False
    node.limit_bar_lateral_s_curve_control_max_forward_overshoot_m = 0.08
    node.limit_bar_lateral_s_curve_control_max_abs_cross_track_m = 0.12
    node.limit_bar_lateral_s_curve_control_forward_tolerance_m = 0.03
    node.limit_bar_lateral_s_curve_control_lateral_tolerance_m = 0.05
    node.limit_bar_lateral_s_curve_control_arrival_settle_sec = 0.20
    node.limit_bar_lateral_s_curve_control_pre_stop_distance_m = 0.12
    node.limit_bar_lateral_s_curve_control_pre_stop_hold_sec = 0.20
    node.limit_bar_lateral_s_curve_control_final_abs_lateral_tolerance_m = 0.08
    node.limit_bar_lateral_s_curve_control_final_max_lateral_drift_m = 0.05
    node.limit_bar_lateral_s_curve_control_final_lateral_violation_hold_sec = 0.20
    node.limit_bar_lateral_s_curve_control_final_align_norm = 0.08
    node.limit_bar_lateral_s_curve_final_lateral_violation_since_sec = None
    node.limit_bar_lateral_s_curve_pre_stop_start_sec = None
    node.limit_bar_lateral_s_curve_pre_stop_completed = False
    node.limit_bar_lateral_s_curve_fail_open_cross_track_logged = False
    node.limit_bar_lateral_s_curve_fail_open_final_lateral_logged = False
    node.limit_bar_lateral_s_curve_control_completed = False
    node.limit_bar_lateral_shift_center_z_m = 1.05
    node.limit_bar_lateral_shift_duck_entry_hold_sec = 0.30
    node.limit_bar_tag_align_mode = 6
    node.limit_bar_lateral_turn_far_error_rad = math.radians(35.0)
    node.limit_bar_lateral_turn_far_scale = 1.25
    node.limit_bar_pre_duck_global_yaw_align_enabled = True
    node.limit_bar_pre_duck_yaw_reference_mode = 'base_yaw'
    node.limit_bar_pre_duck_global_yaw_align_start_sec = None
    node.limit_bar_pre_duck_global_yaw_stable_since_sec = None
    node._prepare_entries = []
    node._enter_limit_bar_prepare = (
        lambda now, reason, yaw_text: node._prepare_entries.append(
            (now, reason, yaw_text)
        )
    )
    node._duck_reference_yaw_text = lambda: 'test_yaw'
    return node


def test_s_curve_progress_uses_base_yaw_route_coordinates():
    node = make_control_stub()

    forward_m, lateral_m = node._lateral_s_curve_control_progress()

    assert math.isclose(forward_m, 0.20)
    assert math.isclose(lateral_m, 0.0)

    node.pose_x = 0.10
    forward_m, lateral_m = node._lateral_s_curve_control_progress()
    assert math.isclose(forward_m, 0.20)
    assert math.isclose(lateral_m, -0.10)


def test_s_curve_rightward_route_commands_right_turn_correction():
    node = make_control_stub()

    node._run_hurdle_lateral_s_curve_control(1.2)

    mode, left_norm, right_norm = node._commands[-1]
    assert mode == 0
    assert left_norm > right_norm
    assert any('高墙 S 曲线控制中' in message for message in node._publish_feedback_messages)


def test_s_curve_slows_before_endpoint():
    node = make_control_stub()
    node.pose_y = 0.90
    node.pose_x = 0.10

    node._run_hurdle_lateral_s_curve_control(1.2)

    mode, left_norm, right_norm = node._commands[-1]
    assert mode == 0
    assert math.isclose((left_norm + right_norm) * 0.5, 0.25)
    assert max(left_norm, right_norm) < 0.26


def test_s_curve_ignores_single_frame_visual_close_candidate():
    node = make_control_stub()
    node.pose_y = 0.75
    node.pose_x = 0.10
    node.hurdle_lateral_s_curve_visual_jump_confirm_samples = 3
    node.hurdle_lateral_s_curve_visual_jump_confirm_duration_sec = 0.15
    node.hurdle_lateral_s_curve_visual_jump_max_remaining_m = 0.35
    node.hurdle_visual_detection.triggered = True
    node.hurdle_lateral_s_curve_visual_close_sample_count = 1
    node.hurdle_lateral_s_curve_visual_close_start_sec = 1.99
    node._hurdle_visual_jump_ready = lambda now: True

    node._run_hurdle_lateral_s_curve_control(2.0)

    assert not node._fail_messages
    assert node.lateral_s_curve_control_completion_reason == ''
    assert node._commands[-1] != (0, 0.0, 0.0)
    assert any(
        '视觉近距离候选暂不采纳' in message
        and 'confirm=1/3' in message
        for message in node._publish_feedback_messages
    )


def test_s_curve_ignores_early_visual_candidate_inconsistent_with_tag():
    node = make_control_stub()
    node.pose_y = 0.01
    node.pose_x = 0.10
    node.hurdle_visual_detection.distance_m = 0.192
    node.hurdle_visual_detection.triggered = True
    node.hurdle_lateral_s_curve_visual_jump_confirm_samples = 3
    node.hurdle_lateral_s_curve_visual_jump_confirm_duration_sec = 0.15
    node.hurdle_lateral_s_curve_visual_jump_max_remaining_m = 0.35
    node.hurdle_lateral_s_curve_visual_jump_tag_max_delta_m = 0.40
    node.hurdle_lateral_s_curve_visual_close_sample_count = 3
    node.hurdle_lateral_s_curve_visual_close_start_sec = 1.80
    node._hurdle_visual_jump_ready = lambda now: True
    node._recent_hurdle_tag_samples = lambda now: [
        type('TagSample', (), {'z_m': 1.55})()
    ]

    node._run_hurdle_lateral_s_curve_control(2.0)

    assert not node._fail_messages
    assert node.lateral_s_curve_control_completion_reason == ''
    assert node._commands[-1] != (0, 0.0, 0.0)
    assert any(
        'remaining=' in message and 'tag_delta=' in message
        for message in node._publish_feedback_messages
    )


def test_s_curve_accepts_confirmed_visual_candidate_near_endpoint():
    node = make_control_stub()
    node.pose_y = 0.75
    node.pose_x = 0.10
    node.hurdle_visual_detection.distance_m = 0.50
    node.hurdle_visual_detection.triggered = True
    node.hurdle_lateral_s_curve_visual_jump_confirm_samples = 3
    node.hurdle_lateral_s_curve_visual_jump_confirm_duration_sec = 0.15
    node.hurdle_lateral_s_curve_visual_jump_max_remaining_m = 0.35
    node.hurdle_lateral_s_curve_visual_jump_tag_max_delta_m = 0.40
    node.hurdle_lateral_s_curve_visual_close_sample_count = 3
    node.hurdle_lateral_s_curve_visual_close_start_sec = 1.80
    node._hurdle_visual_jump_ready = lambda now: True
    node._recent_hurdle_tag_samples = lambda now: [
        type('TagSample', (), {'z_m': 0.62})()
    ]

    node._run_hurdle_lateral_s_curve_control(2.0)

    assert not node._fail_messages
    assert node.lateral_s_curve_control_completion_reason == 'visual_jump'
    assert node._commands[-1] == (0, 0.0, 0.0)


def test_s_curve_completion_holds_then_enters_hurdle_wait():
    node = make_control_stub()
    node.pose_y = 1.0
    node.pose_x = 0.10

    node._run_hurdle_lateral_s_curve_control(2.0)
    assert node.lateral_s_curve_control_completion_reason == 'odometry'
    assert node._hurdle_entries == []
    assert node._commands[-1] == (0, 0.0, 0.0)

    node._run_hurdle_lateral_s_curve_control(2.21)

    assert node._hurdle_entries == [(2.21, 'test')]
    assert not node.lateral_s_curve_control_active
    assert any('第二阶段 S 曲线实际结果' in message for message in node._publish_feedback_messages)


def test_hurdle_visual_completion_continues_after_plan_endpoint_overshoot():
    node = make_control_stub()
    node.hurdle_lateral_s_curve_visual_completion_ignore_plan_overshoot_enabled = True
    node.lateral_s_curve_control_completion_reason = 'visual_jump'
    node.lateral_s_curve_control_arrival_sec = 1.5
    node.lateral_s_curve_control_arrival_forward_m = 0.90
    node.lateral_s_curve_control_arrival_cross_error_m = 0.0
    node.lateral_s_curve_control_completion_max_forward_m = 1.16
    node.pose_y = 1.16
    node.pose_x = 0.10

    # Reproduce the field log: visual jump distance is already latched, while
    # final yaw/recovery projects 16 cm beyond the nominal curve endpoint.
    node._run_hurdle_lateral_s_curve_control(2.0)
    node._run_hurdle_lateral_s_curve_control(2.21)

    assert not node._fail_messages
    assert node._hurdle_entries == [(2.21, 'test')]
    assert node.lateral_s_curve_control_active is False


def test_hurdle_visual_completion_keeps_secondary_overshoot_limit():
    node = make_control_stub()
    node.hurdle_lateral_s_curve_visual_completion_ignore_plan_overshoot_enabled = True
    node.lateral_s_curve_control_completion_reason = 'visual_jump'
    node.pose_y = 1.26
    node.pose_x = 0.10

    node._run_hurdle_lateral_s_curve_control(2.0)

    assert node._commands[-1] == (0, 0.0, 0.0)
    assert node._fail_messages
    assert '高墙 S 曲线前向过冲' in node._fail_messages[-1]


def test_s_curve_arrival_settle_stops_before_completion_hold():
    node = make_control_stub()
    node.pose_y = 1.0
    node.pose_x = 0.10
    node.lateral_s_curve_control_arrival_settle_sec = 0.30

    node._run_hurdle_lateral_s_curve_control(2.0)
    node._run_hurdle_lateral_s_curve_control(2.25)

    assert node._hurdle_entries == []
    assert node._commands[-1] == (0, 0.0, 0.0)
    assert node.lateral_s_curve_control_completion_since_sec is None
    assert any(
        'phase=arrival_settle' in message
        for message in node._publish_feedback_messages
    )

    node._run_hurdle_lateral_s_curve_control(2.31)
    assert node.lateral_s_curve_control_completion_since_sec == 2.31
    node._run_hurdle_lateral_s_curve_control(2.52)
    assert node._hurdle_entries == [(2.52, 'test')]


def test_s_curve_completion_stops_when_retreat_exceeds_limit():
    node = make_control_stub()
    node.pose_y = 1.0
    node.pose_x = 0.10
    node.lateral_s_curve_control_arrival_settle_sec = 0.10
    node.lateral_s_curve_control_completion_max_retreat_m = 0.03

    node._run_hurdle_lateral_s_curve_control(2.0)
    node.pose_y = 1.05
    node._run_hurdle_lateral_s_curve_control(2.05)
    node.pose_y = 1.01
    node._run_hurdle_lateral_s_curve_control(2.11)

    assert node._commands[-1] == (0, 0.0, 0.0)
    assert node._fail_messages
    assert '收尾倒退超限' in node._fail_messages[-1]


def test_s_curve_completion_recovers_retreat_and_continues_flow():
    node = make_control_stub()
    node.pose_y = 1.0
    node.pose_x = 0.10
    node.lateral_s_curve_control_arrival_settle_sec = 0.10
    node.lateral_s_curve_control_completion_max_retreat_m = 0.03
    node.lateral_s_curve_control_completion_recovery_enabled = True

    node._run_hurdle_lateral_s_curve_control(2.0)
    node.pose_y = 1.05
    node._run_hurdle_lateral_s_curve_control(2.05)
    node.pose_y = 1.01
    node.pose_yaw = math.radians(6.0)
    node._run_hurdle_lateral_s_curve_control(2.11)

    mode, left_norm, right_norm = node._commands[-1]
    assert mode == node.limit_bar_lateral_shift_walk_mode
    assert left_norm > 0.0
    assert right_norm > 0.0
    assert left_norm > right_norm
    assert not node._fail_messages
    assert node.lateral_s_curve_control_completion_recovery_active is True
    assert math.isclose(
        node.lateral_s_curve_control_completion_recovery_target_forward_m,
        1.035,
    )
    assert any(
        '启动前向补偿' in message
        for message in node._publish_feedback_messages
    )

    node.pose_y = 1.036
    node.pose_yaw = 0.0
    node._run_hurdle_lateral_s_curve_control(2.20)
    assert node._commands[-1] == (0, 0.0, 0.0)
    assert node.lateral_s_curve_control_completion_since_sec == 2.20

    node._run_hurdle_lateral_s_curve_control(2.41)
    assert node._hurdle_entries == [(2.41, 'test')]
    assert not node._fail_messages
    assert node.lateral_s_curve_control_active is False


def test_s_curve_completion_rechecks_lateral_error_after_arrival():
    node = make_control_stub()
    node.pose_y = 1.0
    node.pose_x = 0.10

    node._run_hurdle_lateral_s_curve_control(2.0)
    node.pose_x = 0.21
    node._run_hurdle_lateral_s_curve_control(2.21)

    assert node._commands[-1] == (0, 0.0, 0.0)
    assert node._fail_messages
    assert '收尾后横向误差超限' in node._fail_messages[-1]


def test_upstairs_s_curve_tracks_route_with_branch_specific_speed():
    node = make_upstairs_control_stub()

    assert node._limit_bar_lateral_phase_timeout_sec('s_curve') == 12.0
    node._run_upstairs_lateral_s_curve_control(1.2)

    mode, left_norm, right_norm = node._commands[-1]
    assert mode == 0
    assert left_norm > right_norm
    assert not node._fail_messages
    assert any(
        '上台阶 S 曲线控制中' in message
        for message in node._publish_feedback_messages
    )


def test_upstairs_s_curve_ignores_distance_candidate_too_early():
    node = make_upstairs_control_stub()
    node._upstairs_final_approach_ready = (
        lambda now, live_alignment: (True, 'source=test')
    )

    node._run_upstairs_lateral_s_curve_control(1.2)

    assert node.lateral_s_curve_control_completion_reason == ''
    assert node._commands[-1] != (0, 0.0, 0.0)
    assert not node._fail_messages
    assert any(
        'distance_candidate_ignored_remaining=' in message
        for message in node._publish_feedback_messages
    )


def test_upstairs_s_curve_creeps_when_odom_arrives_before_distance():
    node = make_upstairs_control_stub()
    node.pose_y = 1.0
    node.pose_x = 0.10

    node._run_upstairs_lateral_s_curve_control(2.0)

    assert node._commands[-1] == (0, 0.15, 0.15)
    assert not node._upstairs_done
    assert not node._fail_messages
    assert any(
        'action=distance_confirm_creep' in message
        for message in node._publish_feedback_messages
    )


def test_upstairs_s_curve_aligns_yaw_before_distance_creep():
    node = make_upstairs_control_stub()
    node.pose_y = 1.0
    node.pose_x = 0.10
    node.pose_yaw = math.radians(-8.0)

    node._run_upstairs_lateral_s_curve_control(2.0)

    assert node._commands[-1] == (6, -0.08, 0.08)
    assert math.isclose(sum(node._commands[-1][1:]), 0.0, abs_tol=1e-9)
    assert not node._upstairs_done
    assert not node._fail_messages
    assert any(
        'action=distance_confirm_yaw_align_zero_forward' in message
        for message in node._publish_feedback_messages
    )


def test_upstairs_s_curve_final_align_uses_zero_forward_turn():
    node = make_upstairs_control_stub()
    node.pose_y = 0.90
    node.pose_x = 0.10
    node.pose_yaw = math.radians(-8.0)
    node._upstairs_final_approach_ready = (
        lambda now, live_alignment: (True, 'source=test_confirmed')
    )

    node._run_upstairs_lateral_s_curve_control(2.0)

    assert node.lateral_s_curve_control_completion_reason == 'distance_confirmed'
    assert node._commands[-1] == (6, -0.08, 0.08)
    assert math.isclose(sum(node._commands[-1][1:]), 0.0, abs_tol=1e-9)
    assert not node._upstairs_done
    assert not node._fail_messages
    assert any(
        'phase=final_align_zero_forward' in message
        for message in node._publish_feedback_messages
    )


def test_upstairs_s_curve_completion_latch_ignores_route_overshoot():
    node = make_upstairs_control_stub()
    node.pose_y = 0.90
    node.pose_x = 0.10
    node.lateral_s_curve_control_arrival_settle_sec = 0.30
    node._upstairs_final_approach_ready = (
        lambda now, live_alignment: (True, 'source=test_confirmed')
    )

    node._run_upstairs_lateral_s_curve_control(2.0)
    assert node.lateral_s_curve_control_completion_reason == 'distance_confirmed'

    node.pose_y = 1.21
    node._upstairs_final_approach_ready = (
        lambda now, live_alignment: (False, 'source=test_lost')
    )
    node._run_upstairs_lateral_s_curve_control(2.10)

    assert node._commands[-1] == (0, 0.0, 0.0)
    assert not node._upstairs_done
    assert not node._fail_messages


def test_upstairs_s_curve_latches_distance_then_finishes_step_once():
    node = make_upstairs_control_stub()
    node.pose_y = 0.90
    node.pose_x = 0.10
    distance_ready = [True]
    node._upstairs_final_approach_ready = (
        lambda now, live_alignment: (
            distance_ready[0],
            'source=test_confirmed'
            if distance_ready[0]
            else 'source=test_lost',
        )
    )
    node.lateral_s_curve_control_arrival_settle_sec = 0.30
    node._limit_bar_lateral_turn_ready = (
        lambda now, yaw_error, tolerance: True
    )

    node._run_upstairs_lateral_s_curve_control(2.0)
    assert node.lateral_s_curve_control_completion_reason == 'distance_confirmed'
    assert node._commands[-1] == (0, 0.0, 0.0)
    assert not node._upstairs_done

    distance_ready[0] = False
    node._run_upstairs_lateral_s_curve_control(2.31)

    assert len(node._upstairs_done) == 1
    assert node._upstairs_done[0][1] == 'test'
    assert node.lateral_s_curve_control_active is False
    assert not node._fail_messages
    assert any(
        'distance=(source=test_confirmed)' in message
        for message in node._publish_feedback_messages
    )


def test_limit_bar_short_s_curve_tracks_with_conservative_speed():
    node = make_limit_bar_control_stub()
    node.pose_y = 0.10
    node.lateral_s_curve_control_walk_yaw_gate_rad = math.radians(90.0)

    assert node._limit_bar_lateral_phase_timeout_sec('s_curve') == 8.0
    node._run_limit_bar_lateral_s_curve_control(1.2)

    mode, left_norm, right_norm = node._commands[-1]
    assert mode == node.limit_bar_lateral_shift_walk_mode
    assert left_norm > right_norm
    assert math.isclose((left_norm + right_norm) * 0.5, 0.32)
    assert not node._fail_messages
    assert any(
        '限高杆 S 曲线控制中' in message
        for message in node._publish_feedback_messages
    )


def test_limit_bar_s_curve_phase_dispatches_to_limit_bar_controller():
    node = make_limit_bar_control_stub()
    calls = []
    node._run_limit_bar_lateral_s_curve_control = lambda now: calls.append(now)

    node._run_limit_bar_lateral_shift(1.2)

    assert calls == [1.2]
    assert not node._fail_messages


def test_limit_bar_s_curve_final_align_has_zero_average_command():
    node = make_limit_bar_control_stub()
    node.pose_y = 0.55
    node.pose_x = 0.10
    node.pose_yaw = math.radians(-6.0)
    node.limit_bar_lateral_s_curve_control_arrival_settle_sec = 0.0

    node._run_limit_bar_lateral_s_curve_control(2.0)

    assert node.lateral_s_curve_control_completion_reason == 'odometry'
    assert node._commands[-1] == (6, -0.08, 0.08)
    assert math.isclose(sum(node._commands[-1][1:]), 0.0, abs_tol=1e-9)
    assert not node._fail_messages
    assert any(
        'phase=final_align_zero_forward' in message
        for message in node._publish_feedback_messages
    )


def test_limit_bar_s_curve_completes_into_pre_duck_yaw_align():
    node = make_limit_bar_control_stub()
    node.pose_y = 0.55
    node.pose_x = 0.10

    node._run_limit_bar_lateral_s_curve_control(2.0)
    assert node._commands[-1] == (0, 0.0, 0.0)
    assert node.lateral_s_curve_control_completion_reason == 'odometry'

    node._run_limit_bar_lateral_s_curve_control(2.21)
    node._run_limit_bar_lateral_s_curve_control(2.52)

    assert node.limit_bar_lateral_shift_phase == 'pre_duck_global_yaw_align'
    assert node.limit_bar_pre_duck_global_yaw_align_start_sec == 2.52
    assert node.limit_bar_lateral_s_curve_control_completed is True
    assert node.lateral_s_curve_control_active is False
    assert node._prepare_entries == []
    assert not node._fail_messages
    assert any(
        '保持站立并开始趴下前 yaw 对齐' in message
        and 'source=base_yaw' in message
        for message in node._publish_feedback_messages
    )


def test_limit_bar_s_curve_tolerates_transient_final_lateral_pose_jump():
    node = make_limit_bar_control_stub()
    node.pose_y = 0.55
    node.pose_x = 0.075

    node._run_limit_bar_lateral_s_curve_control(2.0)
    assert math.isclose(
        node.lateral_s_curve_control_arrival_cross_error_m,
        -0.025,
        abs_tol=1e-9,
    )

    # Reproduce the field log: final turning changes the projected endpoint
    # error from 2.5 cm to 5 cm even though the route already arrived.
    node.pose_x = 0.05
    node._run_limit_bar_lateral_s_curve_control(2.21)
    node._run_limit_bar_lateral_s_curve_control(2.52)

    assert not node._fail_messages
    assert node.limit_bar_lateral_s_curve_control_completed is True
    assert node.limit_bar_lateral_shift_phase == 'pre_duck_global_yaw_align'


def test_limit_bar_s_curve_pre_stops_once_before_endpoint():
    node = make_limit_bar_control_stub()
    node.pose_y = 0.45
    route_sample = node.lateral_s_curve_control_plan.sample(node.pose_y)
    node.pose_x = -route_sample.lateral_m

    node._run_limit_bar_lateral_s_curve_control(2.0)

    assert node._commands[-1] == (0, 0.0, 0.0)
    assert node.limit_bar_lateral_s_curve_pre_stop_start_sec == 2.0
    assert node.limit_bar_lateral_s_curve_pre_stop_completed is False
    assert any(
        '到达前预停开始' in message
        for message in node._publish_feedback_messages
    )

    node._run_limit_bar_lateral_s_curve_control(2.21)

    assert node.limit_bar_lateral_s_curve_pre_stop_start_sec is None
    assert node.limit_bar_lateral_s_curve_pre_stop_completed is True
    assert node._commands[-1] != (0, 0.0, 0.0)

    node._run_limit_bar_lateral_s_curve_control(2.22)
    assert node._commands[-1] != (0, 0.0, 0.0)


def test_limit_bar_s_curve_stops_at_endpoint_within_final_eight_centimeters():
    node = make_limit_bar_control_stub()
    node.pose_y = 0.55
    node.pose_x = 0.03

    node._run_limit_bar_lateral_s_curve_control(2.0)

    assert node.lateral_s_curve_control_completion_reason == 'odometry'
    assert math.isclose(
        node.lateral_s_curve_control_arrival_cross_error_m,
        -0.07,
        abs_tol=1e-9,
    )
    assert node._commands[-1] == (0, 0.0, 0.0)
    assert not node._fail_messages
    assert any(
        '不再执行 endpoint_correction' in message
        for message in node._publish_feedback_messages
    )


def test_limit_bar_s_curve_final_lateral_violation_requires_hold():
    node = make_limit_bar_control_stub()
    node.pose_y = 0.55
    node.pose_x = 0.075

    node._run_limit_bar_lateral_s_curve_control(2.0)
    node.pose_x = 0.01
    node._run_limit_bar_lateral_s_curve_control(2.26)

    assert not node._fail_messages
    assert node._commands[-1] == (0, 0.0, 0.0)
    assert any(
        'phase=final_lateral_guard' in message
        for message in node._publish_feedback_messages
    )

    node._run_limit_bar_lateral_s_curve_control(2.47)

    assert node._fail_messages
    assert '收尾横向误差持续超限' in node._fail_messages[-1]


def test_limit_bar_s_curve_stops_on_eight_centimeter_overshoot():
    node = make_limit_bar_control_stub()
    node.pose_y = 0.64
    node.pose_x = 0.10

    node._run_limit_bar_lateral_s_curve_control(2.0)

    assert node._commands[-1] == (0, 0.0, 0.0)
    assert node._fail_messages
    assert '限高杆 S 曲线前向过冲' in node._fail_messages[-1]


def test_limit_bar_s_curve_fail_open_finishes_after_forward_overshoot():
    node = make_limit_bar_control_stub()
    node.limit_bar_lateral_s_curve_fail_open_enabled = True
    node.pose_y = 0.64
    node.pose_x = 0.10

    node._run_limit_bar_lateral_s_curve_control(2.0)

    assert node._commands[-1] == (0, 0.0, 0.0)
    assert not node._fail_messages
    assert (
        node.lateral_s_curve_control_completion_reason
        == 'fail_open_forward_overshoot'
    )

    node._run_limit_bar_lateral_s_curve_control(2.21)
    node._run_limit_bar_lateral_s_curve_control(2.52)

    assert node.limit_bar_lateral_s_curve_control_completed is True
    assert node.limit_bar_lateral_shift_phase == 'pre_duck_global_yaw_align'
    assert any(
        '前向过冲保护已旁路' in message
        for message in node._publish_feedback_messages
    )


def test_limit_bar_s_curve_fail_open_bypasses_final_lateral_failure():
    node = make_limit_bar_control_stub()
    node.limit_bar_lateral_s_curve_fail_open_enabled = True
    node.pose_y = 0.55
    node.pose_x = 0.075

    node._run_limit_bar_lateral_s_curve_control(2.0)
    node.pose_x = 0.01
    node._run_limit_bar_lateral_s_curve_control(2.21)
    node._run_limit_bar_lateral_s_curve_control(2.52)

    assert not node._fail_messages
    assert node.limit_bar_lateral_s_curve_control_completed is True
    assert node.limit_bar_lateral_shift_phase == 'pre_duck_global_yaw_align'
    assert any(
        '收尾横向保护已旁路' in message
        for message in node._publish_feedback_messages
    )


def test_limit_bar_s_curve_fail_open_bypasses_cross_track_failure():
    node = make_limit_bar_control_stub()
    node.limit_bar_lateral_s_curve_fail_open_enabled = True
    node.pose_y = 0.10
    node.pose_x = 0.20
    node.lateral_s_curve_control_walk_yaw_gate_rad = math.radians(90.0)

    node._run_limit_bar_lateral_s_curve_control(2.0)

    assert not node._fail_messages
    assert node.lateral_s_curve_control_completion_reason == ''
    assert node._commands[-1] != (0, 0.0, 0.0)
    assert any(
        '横向跟踪保护已旁路' in message
        for message in node._publish_feedback_messages
    )


def test_limit_bar_s_curve_fail_open_converts_timeout_to_completion():
    node = make_limit_bar_control_stub()
    node.limit_bar_lateral_s_curve_fail_open_enabled = True

    handled = node._fail_limit_bar_lateral_phase_timeout(
        9.1,
        's_curve',
        '限高杆',
    )

    assert handled is True
    assert node._commands[-1] == (0, 0.0, 0.0)
    assert not node._fail_messages
    assert (
        node.lateral_s_curve_control_completion_reason
        == 'fail_open_timeout'
    )
    assert any(
        '控制超时已旁路' in message
        for message in node._publish_feedback_messages
    )


def test_post_stairs_yaw_alignment_is_independent_from_global_slope_align():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.pose_x = 0.0
    node.pose_y = 0.0
    node.pose_yaw = math.radians(12.0)
    node.post_stairs_slope_entry_initial_yaw_align_enabled = True
    node.post_stairs_slope_entry_distance_m = 0.0
    node.post_stairs_slope_entry_mode = 0
    node.post_stairs_slope_entry_left_norm = 0.5
    node.post_stairs_slope_entry_right_norm = 0.5
    node.slope_align_enabled = False
    node._pose_is_fresh = lambda now: True
    node._publish_feedback = lambda message: None
    node._enter_post_stairs_slope_approach = lambda now: None
    calls = []
    node._enter_slope_or_align = lambda *args, **kwargs: calls.append((args, kwargs))

    node._enter_post_stairs_slope_entry(10.0, 'test')

    assert calls
    assert calls[0][1]['force_align'] is True
    assert calls[0][1]['align_next_action'] == 'post_stairs_slope_approach'
