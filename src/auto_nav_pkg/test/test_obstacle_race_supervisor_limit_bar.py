import math
from types import SimpleNamespace

from auto_nav_pkg.obstacle_race_supervisor import ObstacleRaceSupervisor


def make_duck_travel_stub():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.pose_x = 0.0
    node.pose_y = 0.0
    node.duck_start_x = 0.0
    node.duck_start_y = 0.0
    node.duck_reference_yaw = 0.0
    node.duck_centerline_odom_forward_axis = 'y'
    node.duck_travel_projection_enabled = True
    return node


def test_duck_travel_projection_only_counts_locked_yaw_direction():
    node = make_duck_travel_stub()
    node.pose_x = 0.50
    node.pose_y = 1.00

    progress_m, label = node._duck_travel_progress()

    assert label == 'along'
    assert math.isclose(progress_m, 1.00)
    assert math.isclose(node._duck_travel_distance(), math.hypot(0.50, 1.00))


def test_duck_travel_projection_uses_signed_progress():
    node = make_duck_travel_stub()
    node.pose_y = -0.25

    progress_m, label = node._duck_travel_progress()

    assert label == 'along'
    assert math.isclose(progress_m, -0.25)


def test_duck_travel_projection_switch_preserves_euclidean_mode():
    node = make_duck_travel_stub()
    node.duck_travel_projection_enabled = False
    node.pose_x = 0.50
    node.pose_y = 1.00

    progress_m, label = node._duck_travel_progress()

    assert label == 'travel'
    assert math.isclose(progress_m, math.hypot(0.50, 1.00))


def make_lateral_shift_stub():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.limit_bar_lateral_shift_base_yaw = 0.0
    node.limit_bar_lateral_shift_side_yaw = math.pi / 2.0
    node.limit_bar_lateral_shift_phase = 'final_forward'
    node.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'
    node.limit_bar_lateral_shift_final_forward_m = 1.0
    node.limit_bar_lateral_shift_deadband_m = 0.06
    node.limit_bar_lateral_shift_reason = 'test'
    node.limit_bar_lateral_shift_walk_mode = 0
    node.limit_bar_pre_duck_global_yaw_align_enabled = True
    node.limit_bar_pre_duck_yaw_reference_mode = 'base_yaw'
    node.limit_bar_pre_duck_global_yaw_rad = math.pi
    node.limit_bar_pre_duck_global_yaw_tolerance_rad = math.radians(1.0)
    node.limit_bar_pre_duck_global_yaw_exit_tolerance_rad = math.radians(1.0)
    node.limit_bar_pre_duck_global_yaw_hold_sec = 0.30
    node.limit_bar_pre_duck_global_yaw_timeout_sec = 4.0
    node.limit_bar_pre_duck_global_yaw_fail_open_enabled = False
    node.limit_bar_pre_duck_global_yaw_align_start_sec = None
    node.limit_bar_pre_duck_global_yaw_stable_since_sec = None
    node.limit_bar_pre_duck_global_yaw_hold_active = False
    node.limit_bar_pre_duck_stationary_turn_enabled = True
    node.limit_bar_pre_duck_stationary_turn_norm = 0.08
    node.limit_bar_tag_align_mode = 6
    node.limit_bar_tag_align_tolerance_rad = math.radians(2.0)
    node.limit_bar_lateral_turn_far_error_rad = math.radians(35.0)
    node.limit_bar_lateral_turn_far_scale = 1.25
    node.limit_bar_lateral_shift_duck_align_since_sec = None
    node.last_limit_bar_lateral_shift_log_sec = 0.0
    node.hurdle_lateral_shift_legacy_max_forward_overshoot_m = 0.0
    node.pose_x = 1.0
    node.pose_y = 2.0
    node.pose_yaw = 0.0
    node.duck_centerline_correction_enabled = False
    node.duck_reference_yaw = 0.0
    node.duck_reference_yaw_source = ''
    node.duck_reference_yaw_offset_rad = 0.0
    node.duck_start_x = None
    node.duck_start_y = None
    node._pose_is_fresh = lambda now: True
    node._lateral_shift_label = lambda: '限高杆'
    node._lateral_shift_target_action_text = lambda: '趴下'
    node._upstairs_live_tag_alignment = lambda now: None
    node._limit_bar_lateral_shift_progress = lambda yaw: (1.0, 1.0)
    node._limit_bar_lateral_shift_turn_override = (
        lambda error, duck_align=False: (6, -0.06, 0.11)
    )
    node._publish_feedback_messages = []
    node._publish_feedback = node._publish_feedback_messages.append
    node._commands = []
    node._publish_step_override = (
        lambda mode, left, right: node._commands.append((mode, left, right))
    )
    node._fail_messages = []
    node._fail = node._fail_messages.append
    node._prepare_entries = []
    node._enter_limit_bar_prepare = (
        lambda now, reason, yaw_text: node._prepare_entries.append(
            (now, reason, yaw_text)
        )
    )
    return node


def make_upstairs_final_forward_stub():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_next_action = 'upstairs_step_once'
    node.limit_bar_lateral_shift_final_forward_m = 1.165
    node.upstairs_lateral_shift_deadband_m = 0.12
    node.upstairs_lateral_shift_target_distance_m = 0.46
    node.upstairs_final_forward_tolerance_m = 0.05
    node.upstairs_final_tag_distance_tolerance_m = 0.05
    node.upstairs_final_visual_distance_m = 0.50
    node.upstairs_final_slowdown_distance_m = 0.25
    node.upstairs_final_slow_walk_left_norm = 0.30
    node.upstairs_final_slow_walk_right_norm = 0.30
    node.upstairs_final_confirm_creep_max_extra_m = 0.18
    node.upstairs_final_confirm_creep_left_norm = 0.15
    node.upstairs_final_confirm_creep_right_norm = 0.15
    node.upstairs_tag_yaw_reference_mode = 'fixed_absolute'
    node.upstairs_final_forward_realign_gate_rad = math.radians(15.0)
    node.upstairs_final_yaw_complete_gate_rad = math.radians(1.0)
    node.limit_bar_lateral_shift_final_yaw_gate_rad = math.radians(6.0)
    node.limit_bar_lateral_shift_duck_entry_yaw_gate_rad = math.radians(2.0)
    node.limit_bar_lateral_shift_walk_left_norm = 0.50
    node.limit_bar_lateral_shift_walk_right_norm = 0.50
    node.limit_bar_lateral_shift_walk_yaw_correction_enabled = True
    node.limit_bar_lateral_shift_walk_yaw_deadband_rad = math.radians(1.5)
    node.limit_bar_lateral_shift_walk_yaw_gain_norm_per_rad = 0.90
    node.limit_bar_lateral_shift_walk_yaw_max_delta_norm = 0.18
    node.upstairs_final_align_latched_along_m = None
    node.upstairs_final_align_latched_raw_along_m = None
    node.upstairs_final_align_latched_remaining_m = None
    node.upstairs_final_align_latched_reason = ''
    node.upstairs_final_align_latched_live_text = ''
    node._distance_text = (
        lambda distance: 'unknown' if distance is None else f'{distance:.3f}m'
    )
    node._upstairs_visual_step_ready = (
        lambda now, distance_threshold_m=None: (False, 'trigger=false')
    )
    node._limit_bar_lateral_shift_progress = lambda yaw: (1.010, 1.010)
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


def test_final_forward_enters_base_yaw_align_before_duck():
    node = make_lateral_shift_stub()

    node._run_limit_bar_lateral_shift(10.0)

    assert node.limit_bar_lateral_shift_phase == 'pre_duck_global_yaw_align'
    assert node.limit_bar_pre_duck_global_yaw_align_start_sec == 10.0
    assert node._commands == [(0, 0.0, 0.0)]
    assert node._prepare_entries == []
    assert any('趴下前开始对齐 yaw=0.0deg, source=base_yaw' in message
               for message in node._publish_feedback_messages)


def test_base_yaw_align_handles_wraparound_then_enters_duck():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_phase = 'pre_duck_global_yaw_align'
    node.limit_bar_pre_duck_global_yaw_align_start_sec = 10.0
    node.limit_bar_lateral_shift_base_yaw = math.pi
    node.pose_yaw = math.radians(-179.5)

    node._run_limit_bar_lateral_shift(10.1)
    assert node.limit_bar_pre_duck_global_yaw_stable_since_sec == 10.1
    assert node._prepare_entries == []

    node._run_limit_bar_lateral_shift(10.5)

    assert node.limit_bar_lateral_shift_phase == ''
    assert math.isclose(node.duck_reference_yaw, math.pi)
    assert node.duck_reference_yaw_source == 'limit_bar_pre_duck_base_yaw'
    assert node._prepare_entries
    assert node._fail_messages == []


def test_pre_duck_yaw_timeout_still_fails_when_fail_open_is_disabled():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_phase = 'pre_duck_global_yaw_align'
    node.limit_bar_pre_duck_global_yaw_align_start_sec = 10.0
    node.limit_bar_lateral_shift_base_yaw = math.pi
    node.pose_yaw = math.radians(-175.5)

    node._run_limit_bar_lateral_shift(14.1)

    assert node._fail_messages
    assert 'yaw 对齐超时' in node._fail_messages[-1]
    assert node._prepare_entries == []


def test_pre_duck_yaw_timeout_fail_open_continues_into_duck():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_phase = 'pre_duck_global_yaw_align'
    node.limit_bar_pre_duck_global_yaw_align_start_sec = 10.0
    node.limit_bar_lateral_shift_base_yaw = math.pi
    node.pose_yaw = math.radians(-175.5)
    node.limit_bar_pre_duck_global_yaw_fail_open_enabled = True

    node._run_limit_bar_lateral_shift(14.1)

    assert not node._fail_messages
    assert node.limit_bar_lateral_shift_phase == ''
    assert node._commands[-1] == (0, 0.0, 0.0)
    assert math.isclose(node.duck_reference_yaw, math.pi)
    assert node._prepare_entries
    assert any(
        'yaw 对齐超时已旁路' in message
        for message in node._publish_feedback_messages
    )


def test_pre_duck_yaw_stops_inside_tolerance_while_yaw_rate_settles():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_phase = 'pre_duck_global_yaw_align'
    node.limit_bar_pre_duck_global_yaw_align_start_sec = 10.0
    node.limit_bar_lateral_shift_base_yaw = math.pi
    node.pose_yaw = math.radians(175.0)
    node.limit_bar_pre_duck_global_yaw_tolerance_rad = math.radians(6.0)
    node.limit_bar_pre_duck_global_yaw_exit_tolerance_rad = math.radians(8.0)
    node._limit_bar_lateral_yaw_rate_ready = lambda: False

    node._run_limit_bar_lateral_shift(10.1)

    assert node.limit_bar_pre_duck_global_yaw_hold_active
    assert node._commands[-1] == (0, 0.0, 0.0)
    assert node._prepare_entries == []
    assert node._fail_messages == []


def test_pre_duck_yaw_hysteresis_only_resumes_after_exit_tolerance():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_phase = 'pre_duck_global_yaw_align'
    node.limit_bar_pre_duck_global_yaw_align_start_sec = 10.0
    node.limit_bar_lateral_shift_base_yaw = math.pi
    node.limit_bar_pre_duck_global_yaw_tolerance_rad = math.radians(6.0)
    node.limit_bar_pre_duck_global_yaw_exit_tolerance_rad = math.radians(8.0)
    node._limit_bar_lateral_yaw_rate_ready = lambda: False

    node.pose_yaw = math.radians(175.0)
    node._run_limit_bar_lateral_shift(10.1)
    node.pose_yaw = math.radians(173.0)
    node._run_limit_bar_lateral_shift(10.2)

    assert node.limit_bar_pre_duck_global_yaw_hold_active
    assert node._commands[-1] == (0, 0.0, 0.0)

    node.pose_yaw = math.radians(171.0)
    node._run_limit_bar_lateral_shift(10.3)

    assert not node.limit_bar_pre_duck_global_yaw_hold_active
    assert node._commands[-1] == (6, -0.08, 0.08)


def test_pre_duck_yaw_settle_timeout_fail_open_still_continues():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_phase = 'pre_duck_global_yaw_align'
    node.limit_bar_pre_duck_global_yaw_align_start_sec = 10.0
    node.limit_bar_lateral_shift_base_yaw = math.pi
    node.pose_yaw = math.radians(175.0)
    node.limit_bar_pre_duck_global_yaw_tolerance_rad = math.radians(6.0)
    node.limit_bar_pre_duck_global_yaw_exit_tolerance_rad = math.radians(8.0)
    node.limit_bar_pre_duck_global_yaw_fail_open_enabled = True
    node._limit_bar_lateral_yaw_rate_ready = lambda: False

    node._run_limit_bar_lateral_shift(14.1)

    assert node._fail_messages == []
    assert node._commands[-1] == (0, 0.0, 0.0)
    assert node._prepare_entries
    assert any(
        'yaw 对齐超时已旁路' in message
        for message in node._publish_feedback_messages
    )


def test_pre_duck_stationary_turn_uses_precision_brake_guard():
    node = make_lateral_shift_stub()
    node._limit_bar_lateral_precision_turn_active = lambda: True
    node._limit_bar_lateral_shift_turn_override = (
        lambda error, duck_align=False: (0, 0.0, 0.0)
    )

    assert node._limit_bar_pre_duck_stationary_turn_override(
        math.radians(9.0)
    ) == (0, 0.0, 0.0)


def test_global_pre_duck_align_only_applies_to_limit_bar():
    node = make_lateral_shift_stub()

    node.limit_bar_lateral_shift_next_action = 'hurdle_wait_jump'
    assert not node._limit_bar_pre_duck_global_yaw_align_required()

    node.limit_bar_lateral_shift_next_action = 'upstairs_step_once'
    assert not node._limit_bar_pre_duck_global_yaw_align_required()

    node.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'
    assert node._limit_bar_pre_duck_global_yaw_align_required()


def test_limit_bar_legacy_pre_duck_align_has_zero_average_command():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_phase = 'pre_duck_global_yaw_align'
    node.limit_bar_lateral_shift_base_yaw = math.pi

    node._run_limit_bar_lateral_shift(10.0)

    mode, left_norm, right_norm = node._commands[-1]
    assert mode == 6
    assert math.isclose(left_norm, -0.10)
    assert math.isclose(right_norm, 0.10)
    assert math.isclose(left_norm + right_norm, 0.0, abs_tol=1e-9)
    assert not node._fail_messages


def test_limit_bar_pre_duck_stationary_switch_restores_legacy_turn():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_phase = 'pre_duck_global_yaw_align'
    node.limit_bar_lateral_shift_base_yaw = math.radians(10.0)
    node.limit_bar_pre_duck_stationary_turn_enabled = False

    node._run_limit_bar_lateral_shift(10.0)

    assert node._commands[-1] == (6, -0.06, 0.11)
    assert not node._fail_messages


def test_upstairs_visual_ready_finishes_when_odom_progress_stalls():
    node = make_upstairs_final_forward_stub()
    node.pose_yaw = math.radians(0.3)
    node._upstairs_visual_step_ready = (
        lambda now, distance_threshold_m=None: (
            True,
            'trigger=true, dist=0.403m, threshold=0.500m',
        )
    )

    node._run_limit_bar_lateral_shift(10.0)

    assert len(node._upstairs_done) == 1
    assert math.isclose(node._upstairs_done[0][2], 1.010)
    assert math.isclose(node._upstairs_done[0][4], 0.155)


def test_upstairs_odom_complete_creeps_until_distance_is_confirmed():
    node = make_upstairs_final_forward_stub()
    node.pose_yaw = math.radians(0.3)
    node._limit_bar_lateral_shift_progress = lambda yaw: (1.130, 1.130)
    node._upstairs_live_tag_alignment = (
        lambda now: (0.0, 0.725, 0.0, 0.0, 0.0)
    )
    node._upstairs_visual_step_ready = (
        lambda now, distance_threshold_m=None: (
            False,
            'dist=0.637m>0.500m',
        )
    )

    node._run_limit_bar_lateral_shift(10.0)

    assert node._upstairs_done == []
    assert node._commands[-1] == (0, 0.15, 0.15)
    assert any(
        '最终前进里程已到但距离未确认' in message
        for message in node._publish_feedback_messages
    )
    assert any(
        'action=upstairs_final_confirm_creep' in message
        for message in node._publish_feedback_messages
    )


def test_upstairs_distance_confirm_creep_stops_at_extra_distance_limit():
    node = make_upstairs_final_forward_stub()
    node.pose_yaw = math.radians(0.3)
    node._limit_bar_lateral_shift_progress = lambda yaw: (1.350, 1.350)
    node._upstairs_live_tag_alignment = (
        lambda now: (0.0, 0.725, 0.0, 0.0, 0.0)
    )

    node._run_limit_bar_lateral_shift(10.0)

    assert node._upstairs_done == []
    assert node._commands[-1] == (0, 0.0, 0.0)
    assert any(
        'extra_forward=0.185/0.180m' in message
        for message in node._publish_feedback_messages
    )


def test_upstairs_visual_confirmation_can_override_live_tag_distance():
    node = make_upstairs_final_forward_stub()
    node.pose_yaw = math.radians(0.3)
    node._limit_bar_lateral_shift_progress = lambda yaw: (1.062, 1.062)
    node._upstairs_live_tag_alignment = (
        lambda now: (0.0, 0.725, 0.0, 0.0, 0.0)
    )
    node._upstairs_visual_step_ready = (
        lambda now, distance_threshold_m=None: (
            True,
            'trigger=true, dist=0.560m, threshold=0.580m',
        )
    )

    node._run_limit_bar_lateral_shift(10.0)

    assert len(node._upstairs_done) == 1
    assert any(
        'source=visual_with_live_tag' in message
        for message in node._publish_feedback_messages
    )


def test_upstairs_final_align_latches_distance_and_never_walks_forward_again():
    node = make_upstairs_final_forward_stub()
    node.pose_yaw = math.radians(2.0)
    visual_ready = [True]
    node._upstairs_visual_step_ready = (
        lambda now, distance_threshold_m=None: (
            visual_ready[0],
            'trigger=true, dist=0.550m, threshold=0.580m'
            if visual_ready[0]
            else 'dist=0.622m>0.580m',
        )
    )

    node._run_limit_bar_lateral_shift(10.0)

    assert node.limit_bar_lateral_shift_phase == 'upstairs_final_align'
    assert node._commands[-1][0] == 6
    assert node._upstairs_done == []

    visual_ready[0] = False
    node._run_limit_bar_lateral_shift(10.1)

    assert node.limit_bar_lateral_shift_phase == 'upstairs_final_align'
    assert node._commands[-1][0] == 6
    assert not any(
        mode == node.limit_bar_lateral_shift_walk_mode and left > 0.0 and right > 0.0
        for mode, left, right in node._commands
    )
    assert node._upstairs_done == []

    node.pose_yaw = math.radians(0.3)
    node._run_limit_bar_lateral_shift(10.2)

    assert node.limit_bar_lateral_shift_phase == ''
    assert len(node._upstairs_done) == 1


def make_upstairs_visual_fallback_stub():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.upstairs_completed = False
    node.upstairs_tag_visual_fallback_enabled = True
    node.upstairs_tag_visual_fallback_require_hurdle_completed = True
    node.upstairs_tag_visual_fallback_require_recent_tag_id = False
    node.upstairs_tag_visual_fallback_max_detection_age_sec = 2.0
    node.hurdle_completed = True
    node.upstairs_tag_samples = []
    node.upstairs_tag_trigger_enabled = True
    node.upstairs_tag_stale_timeout_sec = 0.60
    node.upstairs_tag_max_sample_age_sec = 0.50
    node.upstairs_tag_detection_stamp_sec = 0.0
    node.upstairs_trigger_distance_m = 0.66
    node.upstairs_trigger_min_duration_sec = 0.0
    node.detection_stale_timeout_sec = 0.75
    node.apriltag_tf_queue_depth = 30
    node.upstairs_visual_detection = type(
        'Detection',
        (),
        {
            'triggered': True,
            'distance_m': 0.393,
            'nearest_stamp_sec': 10.0,
            'trigger_start_stamp_sec': 9.9,
            'is_fresh': lambda self, now, timeout: True,
        },
    )()
    node._prune_upstairs_tag_samples = lambda now: None
    node._distance_text = lambda distance: f'{distance:.3f}m'
    return node


def test_upstairs_visual_fallback_allows_close_image_without_recent_tag_id():
    node = make_upstairs_visual_fallback_stub()

    ready, reason = node._upstairs_tag_visual_fallback_ready(10.1)

    assert ready
    assert 'trigger=true, dist=0.393m' in reason
    assert 'tag_id_not_seen' in reason


def test_upstairs_visual_fallback_can_require_recent_tag_id_when_configured():
    node = make_upstairs_visual_fallback_stub()
    node.upstairs_tag_visual_fallback_require_recent_tag_id = True

    ready, reason = node._upstairs_tag_visual_fallback_ready(10.1)

    assert not ready
    assert reason == 'tag_id_not_seen'


def test_upstairs_fresh_tag_blocks_early_visual_completion():
    node = make_upstairs_final_forward_stub()
    node.pose_yaw = math.radians(0.3)
    node._upstairs_live_tag_alignment = (
        lambda now: (0.0, 0.894, 0.0, 0.0, 0.0)
    )
    node._upstairs_visual_step_ready = (
        lambda now, distance_threshold_m=None: (
            False,
            'dist=0.637m>0.500m',
        )
    )

    node._run_limit_bar_lateral_shift(10.0)

    assert node._upstairs_done == []
    assert node._commands
    assert any(
        'source=live_tag, tag_z=0.894m>0.510m' in message
        for message in node._publish_feedback_messages
    )


def test_upstairs_fresh_tag_completes_at_final_distance():
    node = make_upstairs_final_forward_stub()
    node.pose_yaw = math.radians(0.3)
    node._upstairs_live_tag_alignment = (
        lambda now: (0.0, 0.500, 0.0, 0.0, 0.0)
    )

    node._run_limit_bar_lateral_shift(10.0)

    assert len(node._upstairs_done) == 1
    assert any(
        'source=live_tag, tag_z=0.500m<=0.510m' in message
        for message in node._publish_feedback_messages
    )


def test_upstairs_final_forward_does_not_reuse_lateral_deadband():
    node = make_upstairs_final_forward_stub()
    node.pose_yaw = math.radians(0.3)
    node._limit_bar_lateral_shift_progress = lambda yaw: (1.085, 1.085)

    node._run_limit_bar_lateral_shift(10.0)

    assert math.isclose(
        node.limit_bar_lateral_shift_final_forward_m - 1.085,
        0.080,
    )
    assert node._upstairs_done == []


def test_upstairs_final_forward_slows_down_near_stop_distance():
    node = make_upstairs_final_forward_stub()
    node.pose_yaw = 0.0
    node._limit_bar_lateral_shift_progress = lambda yaw: (0.965, 0.965)

    node._run_limit_bar_lateral_shift(10.0)

    assert node._upstairs_done == []
    assert node._commands[-1] == (0, 0.30, 0.30)
    assert any(
        'phase=final_forward/upstairs_final_slow' in message
        for message in node._publish_feedback_messages
    )


def test_limit_bar_final_forward_does_not_use_upstairs_slow_speed():
    node = make_lateral_shift_stub()
    node.limit_bar_pre_duck_global_yaw_align_enabled = False
    node.limit_bar_lateral_shift_walk_left_norm = 0.50
    node.limit_bar_lateral_shift_walk_right_norm = 0.50
    node.limit_bar_lateral_shift_walk_yaw_correction_enabled = False
    node.limit_bar_lateral_shift_final_yaw_gate_rad = math.radians(6.0)
    node._limit_bar_lateral_shift_progress = lambda yaw: (0.80, 0.80)

    node._run_limit_bar_lateral_shift(10.0)

    assert node._commands[-1] == (0, 0.50, 0.50)


def test_upstairs_small_yaw_error_keeps_walking_instead_of_point_turning():
    node = make_upstairs_final_forward_stub()
    node.pose_yaw = math.radians(10.0)

    node._run_limit_bar_lateral_shift(10.0)

    assert node._upstairs_done == []
    assert node._commands
    mode, left_norm, right_norm = node._commands[-1]
    assert mode == node.limit_bar_lateral_shift_walk_mode
    assert left_norm > right_norm


def test_limit_bar_still_uses_original_six_degree_point_turn_gate():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_final_yaw_gate_rad = math.radians(6.0)
    node.limit_bar_lateral_shift_walk_left_norm = 0.50
    node.limit_bar_lateral_shift_walk_right_norm = 0.50
    node.limit_bar_lateral_shift_walk_yaw_correction_enabled = True
    node.limit_bar_lateral_shift_walk_yaw_deadband_rad = math.radians(1.5)
    node.limit_bar_lateral_shift_walk_yaw_gain_norm_per_rad = 0.90
    node.limit_bar_lateral_shift_walk_yaw_max_delta_norm = 0.18
    node.pose_yaw = math.radians(10.0)
    node._limit_bar_lateral_shift_progress = lambda yaw: (0.50, 0.50)

    node._run_limit_bar_lateral_shift(10.0)

    assert node._commands
    mode, _, _ = node._commands[-1]
    assert mode == 6
    assert node.limit_bar_lateral_shift_phase == 'final_forward'


def test_hurdle_visual_jump_completion_behavior_is_unchanged():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_next_action = 'hurdle_wait_jump'
    node.hurdle_final_forward_tolerance_m = 0.02
    node.hurdle_lateral_shift_visual_jump_yaw_gate_rad = math.radians(6.0)
    node.hurdle_lateral_shift_target_distance_m = 0.50
    node.hurdle_lateral_shift_visual_jump_distance_m = 0.65
    node.limit_bar_lateral_shift_duck_entry_yaw_gate_rad = math.radians(2.0)
    node.limit_bar_lateral_shift_duck_entry_hold_sec = 0.30
    node.limit_bar_lateral_shift_final_yaw_gate_rad = math.radians(6.0)
    node.pose_yaw = math.radians(1.0)
    node.hurdle_visual_detection = type(
        'Detection',
        (),
        {'distance_m': 0.50},
    )()
    node._limit_bar_lateral_shift_progress = lambda yaw: (0.20, 0.20)
    node._hurdle_visual_jump_ready = lambda now: True
    node._hurdle_entries = []
    node._enter_hurdle_wait_jump = (
        lambda now, reason: node._hurdle_entries.append((now, reason))
    )

    node._run_limit_bar_lateral_shift(10.0)

    assert node._hurdle_entries == [(10.0, 'test')]
    assert node.limit_bar_lateral_shift_phase == ''


def test_hurdle_legacy_final_forward_relock_uses_fresh_shorter_distance():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_next_action = 'hurdle_wait_jump'
    node.limit_bar_lateral_shift_final_forward_m = 1.083
    node.hurdle_lateral_shift_legacy_relock_final_forward_enabled = True
    node.hurdle_lateral_shift_legacy_relock_visual_cap_enabled = True
    node.hurdle_lateral_shift_legacy_relock_min_tag_samples = 2
    node.hurdle_lateral_shift_legacy_relock_max_reduction_m = 0.30
    node.hurdle_lateral_shift_target_distance_m = 0.50
    node.hurdle_lateral_shift_visual_jump_distance_m = 0.55
    node.hurdle_lateral_shift_visual_jump_max_age_sec = 0.50
    node.hurdle_visual_detection = type(
        'Detection',
        (),
        {'distance_m': 1.485, 'nearest_stamp_sec': 9.90},
    )()
    node._recent_hurdle_tag_samples = lambda now: [
        type('TagSample', (), {'z_m': 1.450})(),
        type('TagSample', (), {'z_m': 1.470})(),
    ]

    node._relock_hurdle_legacy_final_forward(10.0)

    assert math.isclose(node.limit_bar_lateral_shift_final_forward_m, 0.935)
    assert any(
        'final_forward=1.083->0.935m' in message
        for message in node._publish_feedback_messages
    )


def test_hurdle_legacy_final_forward_relock_limits_single_reduction():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_next_action = 'hurdle_wait_jump'
    node.limit_bar_lateral_shift_final_forward_m = 1.083
    node.hurdle_lateral_shift_legacy_relock_final_forward_enabled = True
    node.hurdle_lateral_shift_legacy_relock_visual_cap_enabled = False
    node.hurdle_lateral_shift_legacy_relock_min_tag_samples = 2
    node.hurdle_lateral_shift_legacy_relock_max_reduction_m = 0.30
    node.hurdle_lateral_shift_target_distance_m = 0.50
    node._recent_hurdle_tag_samples = lambda now: [
        type('TagSample', (), {'z_m': 0.60})(),
        type('TagSample', (), {'z_m': 0.62})(),
    ]

    node._relock_hurdle_legacy_final_forward(10.0)

    assert math.isclose(node.limit_bar_lateral_shift_final_forward_m, 0.783)


def test_hurdle_legacy_final_forward_slows_near_stop_distance():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_next_action = 'hurdle_wait_jump'
    node.hurdle_final_forward_tolerance_m = 0.05
    node.hurdle_lateral_shift_visual_jump_yaw_gate_rad = math.radians(6.0)
    node.limit_bar_lateral_shift_final_yaw_gate_rad = math.radians(6.0)
    node.limit_bar_lateral_shift_walk_left_norm = 0.50
    node.limit_bar_lateral_shift_walk_right_norm = 0.50
    node.limit_bar_lateral_shift_walk_yaw_correction_enabled = False
    node.hurdle_lateral_shift_legacy_slowdown_distance_m = 0.25
    node.hurdle_lateral_shift_legacy_slow_left_norm = 0.25
    node.hurdle_lateral_shift_legacy_slow_right_norm = 0.25
    node._hurdle_visual_jump_ready = lambda now: False
    node._limit_bar_lateral_shift_progress = lambda yaw: (0.80, 0.80)

    node._run_limit_bar_lateral_shift(10.0)

    assert node._commands[-1] == (0, 0.25, 0.25)
    assert any(
        'phase=final_forward/hurdle_legacy_final_slow' in message
        for message in node._publish_feedback_messages
    )


def test_hurdle_legacy_final_forward_hard_stops_on_excess_overshoot():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_shift_next_action = 'hurdle_wait_jump'
    node.hurdle_lateral_shift_legacy_max_forward_overshoot_m = 0.10
    node._limit_bar_lateral_shift_progress = lambda yaw: (1.101, 1.101)

    node._run_limit_bar_lateral_shift(10.0)

    assert node._commands[-1] == (0, 0.0, 0.0)
    assert node._fail_messages
    assert '末段前向过冲' in node._fail_messages[-1]


def make_precision_turn_stub():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.state = node.STATE_LIMIT_BAR_LATERAL_SHIFT
    node.limit_bar_lateral_shift_phase = 'align_out'
    node.limit_bar_lateral_precision_turn_enabled = True
    node.limit_bar_tag_align_tolerance_rad = math.radians(2.0)
    node.limit_bar_lateral_shift_final_yaw_gate_rad = math.radians(6.0)
    node.limit_bar_lateral_shift_duck_entry_yaw_gate_rad = math.radians(2.0)
    node.limit_bar_pre_duck_global_yaw_tolerance_rad = math.radians(1.0)
    node.upstairs_final_yaw_complete_gate_rad = math.radians(1.0)
    node.limit_bar_tag_align_mode = 6
    node.limit_bar_tag_align_ccw_left_norm = -0.15
    node.limit_bar_tag_align_ccw_right_norm = 0.20
    node.limit_bar_tag_align_cw_left_norm = 0.20
    node.limit_bar_tag_align_cw_right_norm = -0.15
    node.limit_bar_lateral_shift_duck_align_ccw_left_norm = -0.04
    node.limit_bar_lateral_shift_duck_align_ccw_right_norm = 0.08
    node.limit_bar_lateral_shift_duck_align_cw_left_norm = 0.08
    node.limit_bar_lateral_shift_duck_align_cw_right_norm = -0.04
    node.limit_bar_tag_align_turn_scale_enabled = True
    node.limit_bar_tag_align_turn_min_scale = 0.35
    node.limit_bar_tag_align_yaw_gain_per_rad = 0.58
    node.limit_bar_tag_align_min_abs_angular_z = 0.035
    node.limit_bar_tag_align_max_abs_angular_z = 0.18
    node.limit_bar_lateral_turn_far_error_rad = math.radians(35.0)
    node.limit_bar_lateral_turn_far_scale = 1.25
    node.limit_bar_lateral_turn_yaw_damping_sec = 0.18
    node.limit_bar_lateral_turn_brake_yaw_rate_rad_s = math.radians(12.0)
    node.limit_bar_lateral_turn_complete_yaw_rate_rad_s = math.radians(6.0)
    node.limit_bar_lateral_turn_hold_sec = 0.12
    node.limit_bar_lateral_turn_reverse_cooldown_sec = 0.20
    node.limit_bar_lateral_turn_stable_since_sec = None
    node.limit_bar_lateral_turn_last_direction = 0
    node.limit_bar_lateral_turn_pending_direction = 0
    node.limit_bar_lateral_turn_reverse_hold_until_sec = 0.0
    node.pose_yaw_rate_rad_s = 0.0
    node._now_sec = lambda: 10.0
    return node


def test_precision_turn_uses_faster_far_command_but_preserves_direction():
    node = make_precision_turn_stub()

    mode, left_norm, right_norm = node._limit_bar_lateral_shift_turn_override(
        math.radians(90.0)
    )

    assert mode == 6
    assert math.isclose(left_norm, -0.1875, abs_tol=1e-9)
    assert math.isclose(right_norm, 0.25, abs_tol=1e-9)


def test_precision_turn_brakes_before_predicted_overshoot():
    node = make_precision_turn_stub()
    node.pose_yaw_rate_rad_s = math.radians(20.0)

    command = node._limit_bar_lateral_shift_turn_override(math.radians(3.0))

    assert command == (0, 0.0, 0.0)


def test_pre_duck_stationary_precision_turn_cools_down_before_reverse():
    node = make_precision_turn_stub()
    node.limit_bar_lateral_shift_phase = 'pre_duck_global_yaw_align'
    node.limit_bar_pre_duck_stationary_turn_norm = 0.08
    clock = [10.0]
    node._now_sec = lambda: clock[0]

    assert node._limit_bar_pre_duck_stationary_turn_override(
        math.radians(9.0)
    ) == (6, -0.08, 0.08)

    clock[0] = 10.10
    assert node._limit_bar_pre_duck_stationary_turn_override(
        math.radians(-9.0)
    ) == (0, 0.0, 0.0)

    clock[0] = 10.31
    assert node._limit_bar_pre_duck_stationary_turn_override(
        math.radians(-9.0)
    ) == (6, 0.08, -0.08)


def test_precision_duck_align_uses_action_entry_gate_not_six_degree_walk_gate():
    node = make_precision_turn_stub()
    node.limit_bar_lateral_shift_phase = 'final_forward'
    node.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'

    mode, left_norm, right_norm = node._limit_bar_lateral_shift_turn_override(
        math.radians(3.0),
        duck_align=True,
    )

    assert mode == 6
    assert left_norm < 0.0
    assert right_norm > 0.0


def test_precision_turn_requires_low_yaw_rate_and_hold_before_phase_change():
    node = make_precision_turn_stub()
    node.pose_yaw_rate_rad_s = math.radians(8.0)

    assert not node._limit_bar_lateral_turn_ready(
        10.0,
        math.radians(1.0),
        math.radians(2.0),
    )
    node.pose_yaw_rate_rad_s = 0.0
    assert not node._limit_bar_lateral_turn_ready(
        10.0,
        math.radians(1.0),
        math.radians(2.0),
    )
    assert node._limit_bar_lateral_turn_ready(
        10.13,
        math.radians(1.0),
        math.radians(2.0),
    )


def test_lateral_shift_arrival_tolerance_is_independent_from_trigger_deadband():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_precision_turn_enabled = True
    node.limit_bar_lateral_shift_phase = 'shift'
    node.limit_bar_lateral_shift_distance_m = 0.10
    node.limit_bar_lateral_shift_arrival_tolerance_m = 0.02
    node.limit_bar_lateral_shift_walk_left_norm = 0.50
    node.limit_bar_lateral_shift_walk_right_norm = 0.50
    node.limit_bar_lateral_shift_walk_yaw_correction_enabled = False
    node._limit_bar_lateral_shift_progress = lambda yaw: (0.075, 0.075)

    node._run_limit_bar_lateral_shift(10.0)

    assert node.limit_bar_lateral_shift_phase == 'shift'
    assert node._commands[-1] == (0, 0.50, 0.50)

    node._limit_bar_lateral_shift_progress = lambda yaw: (0.081, 0.081)
    node._run_limit_bar_lateral_shift(10.1)
    assert node.limit_bar_lateral_shift_phase == 'align_back'


def test_lateral_turn_phase_timeout_stops_instead_of_waiting_forever():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_precision_turn_enabled = True
    node.limit_bar_lateral_shift_phase = 'align_out'
    node.limit_bar_lateral_shift_phase_start_sec = 1.0
    node.limit_bar_lateral_turn_timeout_sec = 2.0

    node._run_limit_bar_lateral_shift(3.1)

    assert node._fail_messages
    assert 'phase=align_out' in node._fail_messages[-1]


def test_lateral_turn_completion_wins_when_watchdog_boundary_is_reached():
    node = make_lateral_shift_stub()
    node.state = node.STATE_LIMIT_BAR_LATERAL_SHIFT
    node.limit_bar_lateral_precision_turn_enabled = True
    node.limit_bar_lateral_shift_phase = 'align_back'
    node.limit_bar_lateral_shift_phase_start_sec = 1.0
    node.limit_bar_lateral_turn_timeout_sec = 8.0
    node.limit_bar_tag_align_tolerance_rad = math.radians(2.0)
    node.limit_bar_lateral_turn_complete_yaw_rate_rad_s = math.radians(6.0)
    node.limit_bar_lateral_turn_hold_sec = 0.12
    node.limit_bar_lateral_turn_stable_since_sec = 8.8
    node.limit_bar_lateral_turn_last_direction = 0
    node.limit_bar_lateral_turn_pending_direction = 0
    node.limit_bar_lateral_turn_reverse_hold_until_sec = 0.0
    node.pose_yaw_rate_rad_s = 0.0
    node._upstairs_live_tag_needs_replan = lambda alignment: False
    node._relock_hurdle_legacy_final_forward = lambda now: None

    node._run_limit_bar_lateral_shift(9.1)

    assert node.limit_bar_lateral_shift_phase == 'final_forward'
    assert node._fail_messages == []


def test_limit_bar_turn_timeout_grace_waits_for_yaw_rate_to_settle():
    node = make_lateral_shift_stub()
    node.state = node.STATE_LIMIT_BAR_LATERAL_SHIFT
    node.limit_bar_lateral_precision_turn_enabled = True
    node.limit_bar_lateral_shift_phase = 'align_back'
    node.limit_bar_lateral_shift_phase_start_sec = 1.0
    node.limit_bar_lateral_turn_timeout_sec = 8.0
    node.limit_bar_lateral_turn_timeout_settle_grace_sec = 0.60
    node.limit_bar_tag_align_tolerance_rad = math.radians(2.0)
    node.limit_bar_lateral_turn_complete_yaw_rate_rad_s = math.radians(6.0)
    node.limit_bar_lateral_turn_hold_sec = 0.12
    node.limit_bar_lateral_turn_stable_since_sec = None
    node.limit_bar_lateral_turn_last_direction = 0
    node.limit_bar_lateral_turn_pending_direction = 0
    node.limit_bar_lateral_turn_reverse_hold_until_sec = 0.0
    node.pose_yaw_rate_rad_s = math.radians(8.0)
    node._upstairs_live_tag_needs_replan = lambda alignment: False
    node._limit_bar_lateral_shift_turn_override = (
        lambda error, duck_align=False: (0, 0.0, 0.0)
    )

    node._run_limit_bar_lateral_shift(9.1)

    assert node.limit_bar_lateral_shift_phase == 'align_back'
    assert node._fail_messages == []
    assert node._commands[-1] == (0, 0.0, 0.0)

    node._run_limit_bar_lateral_shift(9.7)
    assert node._fail_messages
    assert 'phase=align_back' in node._fail_messages[-1]


def test_precision_master_switch_restores_legacy_arrival_and_timeout_behavior():
    node = make_lateral_shift_stub()
    node.limit_bar_lateral_precision_turn_enabled = False
    node.limit_bar_lateral_shift_arrival_tolerance_m = 0.02
    node.limit_bar_lateral_turn_timeout_sec = 2.0

    assert math.isclose(
        node._lateral_shift_arrival_tolerance_m(),
        node.limit_bar_lateral_shift_deadband_m,
    )
    assert node._limit_bar_lateral_phase_timeout_sec('align_out') == 0.0


def make_s_curve_shadow_stub():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.limit_bar_lateral_shift_next_action = 'hurdle_wait_jump'
    node.lateral_s_curve_shadow_enabled = True
    node.hurdle_lateral_s_curve_shadow_enabled = True
    node.limit_bar_lateral_s_curve_shadow_enabled = False
    node.upstairs_lateral_s_curve_shadow_enabled = False
    node.lateral_s_curve_shadow_min_forward_m = 0.35
    node.lateral_s_curve_shadow_max_abs_lateral_m = 0.45
    node.lateral_s_curve_shadow_max_heading_rad = math.radians(25.0)
    node.limit_bar_lateral_s_curve_max_heading_rad = math.radians(42.0)
    node.lateral_s_curve_shadow_max_curvature_m_inv = 2.5
    node.limit_bar_lateral_s_curve_max_curvature_m_inv = 3.5
    node.hurdle_lateral_s_curve_adaptive_clamp_enabled = False
    node.hurdle_lateral_s_curve_adaptive_clamp_min_scale = 0.90
    node._publish_feedback_messages = []
    node._publish_feedback = node._publish_feedback_messages.append
    node._reset_lateral_s_curve_shadow()
    return node


def test_limit_bar_uses_its_own_relaxed_s_curve_heading_limit():
    node = make_s_curve_shadow_stub()
    node.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'
    node.limit_bar_lateral_s_curve_shadow_enabled = True
    node.lateral_s_curve_control_enabled = True
    node.hurdle_lateral_s_curve_control_enabled = False
    node.limit_bar_lateral_s_curve_control_enabled = True
    node.upstairs_lateral_s_curve_control_enabled = False

    node._prepare_lateral_s_curve_shadow(
        now=10.0,
        shift_m=0.314,
        side_delta_rad=math.pi / 2.0,
        forward_component_m=0.0,
        final_forward_m=0.867,
    )

    plan = node.lateral_s_curve_shadow_plan
    assert plan is not None
    assert plan.feasible
    assert math.degrees(plan.max_abs_heading_rad) > 34.0
    assert math.degrees(plan.max_abs_heading_rad) < 42.0
    assert math.isclose(
        node.lateral_s_curve_shadow_max_heading_rad,
        math.radians(25.0),
    )


def test_limit_bar_short_field_route_uses_branch_curvature_limit():
    node = make_s_curve_shadow_stub()
    node.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'
    node.limit_bar_lateral_s_curve_shadow_enabled = True
    node.lateral_s_curve_control_enabled = True
    node.hurdle_lateral_s_curve_control_enabled = False
    node.limit_bar_lateral_s_curve_control_enabled = True
    node.upstairs_lateral_s_curve_control_enabled = False

    node._prepare_lateral_s_curve_shadow(
        now=10.0,
        shift_m=0.200,
        side_delta_rad=math.pi / 2.0,
        forward_component_m=0.0,
        final_forward_m=0.604,
    )

    plan = node.lateral_s_curve_shadow_plan
    assert plan is not None
    assert plan.feasible
    assert plan.max_abs_curvature_m_inv > 2.5
    assert plan.max_abs_curvature_m_inv < 3.5
    assert math.isclose(node.lateral_s_curve_shadow_max_curvature_m_inv, 2.5)
    assert any(
        'branch=limit_bar' in message and 'control=actual_pending' in message
        for message in node._publish_feedback_messages
    )


def test_limit_bar_30cm_shift_stays_on_s_curve_with_42deg_heading_limit():
    node = make_s_curve_shadow_stub()
    node.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'
    node.limit_bar_lateral_s_curve_shadow_enabled = True
    node.lateral_s_curve_control_enabled = True
    node.hurdle_lateral_s_curve_control_enabled = False
    node.limit_bar_lateral_s_curve_control_enabled = True
    node.upstairs_lateral_s_curve_control_enabled = False

    node._prepare_lateral_s_curve_shadow(
        now=10.0,
        shift_m=0.304,
        side_delta_rad=math.pi / 2.0,
        forward_component_m=0.0,
        final_forward_m=0.726,
    )

    plan = node.lateral_s_curve_shadow_plan
    assert plan is not None
    assert plan.feasible
    assert math.degrees(plan.max_abs_heading_rad) > 38.0
    assert math.degrees(plan.max_abs_heading_rad) < 42.0
    assert plan.max_abs_curvature_m_inv < 3.5
    assert any(
        'branch=limit_bar' in message and 'control=actual_pending' in message
        for message in node._publish_feedback_messages
    )


def test_limit_bar_fixed_yaw_keeps_tag_centerline_geometry():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.pose_yaw = math.radians(-179.7)
    node.limit_bar_tag_align_enabled = True
    node.limit_bar_tag_min_samples = 3
    node.limit_bar_tag_to_center_x_m = -0.460
    node.limit_bar_tag_align_yaw_offset_sign = -1.0
    node.limit_bar_tag_geometry_lock_enabled = True
    node.limit_bar_tag_geometry_lock_sample_count = 3
    node.limit_bar_tag_yaw_reference_mode = 'fixed_absolute'
    node.limit_bar_tag_fixed_reference_yaw = math.pi
    node.downstream_route_yaw_reference_enabled = False
    node.downstream_route_yaw_limit_bar_pre_duck_enabled = True
    node.limit_detection = SimpleNamespace(
        distance_m=None,
        lateral_m=None,
        nearest_stamp_sec=0.0,
    )
    node._recent_limit_bar_tag_samples = lambda now: [
        SimpleNamespace(x_m=0.205, z_m=1.231),
        SimpleNamespace(x_m=0.400, z_m=1.200),
        SimpleNamespace(x_m=0.450, z_m=1.300),
    ]
    node._clear_duck_centerline = lambda: None
    node._publish_feedback_messages = []
    node._publish_feedback = node._publish_feedback_messages.append
    node.limit_bar_locked_tag_valid = False
    node.limit_bar_locked_tag_stamp_sec = 0.0
    node.limit_bar_locked_tag_x_m = 0.0
    node.limit_bar_locked_tag_z_m = 0.0
    node.limit_bar_locked_center_x_m = 0.0
    node.limit_bar_locked_tag_odom_yaw = None
    node.limit_bar_locked_tag_sample_count = 0
    node.limit_bar_locked_tag_source = ''

    assert node._capture_limit_bar_tag_reference_yaw(10.0)
    assert math.isclose(node.duck_reference_yaw, math.pi)
    assert math.isclose(node.limit_bar_lateral_shift_center_x_m, -0.010)
    assert math.isclose(node.limit_bar_lateral_shift_center_z_m, 1.300)
    assert node.limit_bar_locked_tag_sample_count == 3
    assert 'yaw_source=fixed_absolute=180.0deg' in node.duck_reference_yaw_source


def test_limit_bar_geometry_lock_selects_minimum_absolute_center_with_paired_depth():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.limit_bar_tag_to_center_x_m = -0.460
    node.limit_bar_tag_geometry_lock_sample_count = 3
    samples = [
        SimpleNamespace(x_m=0.205, z_m=1.231),
        SimpleNamespace(x_m=0.400, z_m=1.200),
        SimpleNamespace(x_m=0.450, z_m=1.300),
    ]

    selected, count, source = node._select_limit_bar_geometry_lock_sample(samples)

    assert selected.x_m == 0.450
    assert selected.z_m == 1.300
    assert count == 3
    assert source == 'min_abs_center_last_samples'


def test_limit_bar_locked_geometry_is_rotated_by_odom_yaw_only():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.limit_bar_locked_tag_valid = True
    node.limit_bar_locked_tag_x_m = 0.388
    node.limit_bar_locked_tag_z_m = 1.171
    node.limit_bar_locked_tag_odom_yaw = math.radians(-174.7)

    tag_x, tag_z = node._limit_bar_locked_tag_for_yaw(math.radians(-180.0))

    assert math.isclose(tag_x, 0.278, abs_tol=0.002)
    assert math.isclose(tag_z, 1.202, abs_tol=0.002)


def test_hurdle_s_curve_shadow_maps_clockwise_shift_to_route_right():
    node = make_s_curve_shadow_stub()

    node._prepare_lateral_s_curve_shadow(
        now=10.0,
        shift_m=0.084,
        side_delta_rad=-math.pi / 2.0,
        forward_component_m=0.0,
        final_forward_m=1.029,
    )

    plan = node.lateral_s_curve_shadow_plan
    assert plan is not None
    assert plan.feasible
    assert math.isclose(plan.forward_m, 1.029)
    assert math.isclose(plan.lateral_m, -0.084)
    assert any('control=legacy_90deg' in message
               for message in node._publish_feedback_messages)


def test_s_curve_shadow_master_switch_is_a_complete_rollback():
    node = make_s_curve_shadow_stub()
    node.lateral_s_curve_shadow_enabled = False

    node._prepare_lateral_s_curve_shadow(
        now=10.0,
        shift_m=0.084,
        side_delta_rad=-math.pi / 2.0,
        forward_component_m=0.0,
        final_forward_m=1.029,
    )

    assert node.lateral_s_curve_shadow_plan is None
    assert node._publish_feedback_messages == []


def test_s_curve_shadow_completion_summary_is_logged_once():
    node = make_s_curve_shadow_stub()
    node._prepare_lateral_s_curve_shadow(
        now=10.0,
        shift_m=0.084,
        side_delta_rad=-math.pi / 2.0,
        forward_component_m=0.0,
        final_forward_m=1.029,
    )

    node._complete_lateral_s_curve_shadow(19.63, completion_reason='visual_jump')
    node._complete_lateral_s_curve_shadow(20.00, completion_reason='visual_jump')

    result_messages = [
        message for message in node._publish_feedback_messages
        if '第二阶段 S 曲线影子结果' in message
    ]
    assert len(result_messages) == 1
    assert 'legacy_elapsed=9.63s' in result_messages[0]
    assert 'completion=visual_jump' in result_messages[0]


def test_s_curve_actual_control_candidate_is_high_wall_gated():
    node = make_s_curve_shadow_stub()
    node.lateral_s_curve_control_enabled = True
    node.hurdle_lateral_s_curve_control_enabled = True
    node.limit_bar_lateral_s_curve_control_enabled = False
    node.upstairs_lateral_s_curve_control_enabled = False

    node._prepare_lateral_s_curve_shadow(
        now=10.0,
        shift_m=0.155,
        side_delta_rad=-math.pi / 2.0,
        forward_component_m=0.0,
        final_forward_m=0.884,
    )

    assert node.lateral_s_curve_shadow_plan is not None
    assert node.lateral_s_curve_shadow_plan.feasible
    assert any(
        'control=actual_pending' in message
        for message in node._publish_feedback_messages
    )

    node.limit_bar_lateral_shift_next_action = 'limit_bar_prepare'
    assert not node._lateral_s_curve_control_requested()


def test_hurdle_s_curve_marginal_heading_excess_clamps_lateral_target():
    node = make_s_curve_shadow_stub()
    node.lateral_s_curve_control_enabled = True
    node.hurdle_lateral_s_curve_control_enabled = True
    node.limit_bar_lateral_s_curve_control_enabled = False
    node.upstairs_lateral_s_curve_control_enabled = False
    node.hurdle_lateral_s_curve_adaptive_clamp_enabled = True

    node._prepare_lateral_s_curve_shadow(
        now=10.0,
        shift_m=0.278,
        side_delta_rad=math.pi / 2.0,
        forward_component_m=0.0,
        final_forward_m=1.083,
    )

    plan = node.lateral_s_curve_shadow_plan
    expected_lateral_m = (
        1.083 * math.tan(math.radians(25.0)) / 1.875 * (1.0 - 1e-6)
    )
    assert plan is not None
    assert plan.feasible
    assert math.isclose(plan.lateral_m, expected_lateral_m)
    assert math.isclose(plan.max_abs_heading_rad, math.radians(25.0), abs_tol=1e-6)
    assert any(
        'control=actual_pending, adaptive_lateral=+0.278->+0.269m' in message
        for message in node._publish_feedback_messages
    )


def test_s_curve_actual_master_switch_keeps_legacy_candidate():
    node = make_s_curve_shadow_stub()
    node.lateral_s_curve_control_enabled = False
    node.hurdle_lateral_s_curve_control_enabled = True
    node.limit_bar_lateral_s_curve_control_enabled = False
    node.upstairs_lateral_s_curve_control_enabled = False

    node._prepare_lateral_s_curve_shadow(
        now=10.0,
        shift_m=0.084,
        side_delta_rad=-math.pi / 2.0,
        forward_component_m=0.0,
        final_forward_m=1.029,
    )

    assert any(
        'control=legacy_90deg' in message
        for message in node._publish_feedback_messages
    )
