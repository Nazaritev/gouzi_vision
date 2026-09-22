import math
from pathlib import Path

import yaml

from auto_nav_pkg.obstacle_race_supervisor import ObstacleRaceSupervisor


def supervisor_params():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'obstacle_race_supervisor.yaml'
    )
    with config_path.open('r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    return config['obstacle_race_supervisor']['ros__parameters']


def supervisor_overlay_params():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'obstacle_race_supervisor_overlay.yaml'
    )
    with config_path.open('r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    return config['obstacle_race_supervisor']['ros__parameters']


def standard_waypoint_config():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'obstacle_waypoints.yaml'
    )
    with config_path.open('r', encoding='utf-8') as config_file:
        return yaml.safe_load(config_file)


def pole_v4_params():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'orange_pole_body_relative_test_v4.yaml'
    )
    with config_path.open('r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    return config['orange_pole_relative_nav']['ros__parameters']


def pole_v1_params():
    config_path = (
        Path(__file__).resolve().parents[1]
        / 'config'
        / 'orange_pole_body_relative_test_v1.yaml'
    )
    with config_path.open('r', encoding='utf-8') as config_file:
        config = yaml.safe_load(config_file)
    return config['orange_pole_relative_nav']['ros__parameters']


def test_slope_align_enabled_is_bool():
    params = supervisor_params()

    assert isinstance(params['slope_align_enabled'], bool)


def test_upstairs_done_retry_guard_matches_requested_forward_protection():
    params = supervisor_params()

    assert params['upstairs_done_retry_guard_enabled'] is True
    assert params['upstairs_done_retry_guard_duration_sec'] == 30.0
    assert params['upstairs_done_retry_guard_mode'] == 0
    assert params['upstairs_done_retry_guard_left_norm'] == 0.1
    assert params['upstairs_done_retry_guard_right_norm'] == 0.1


def test_slope_roll_detection_latches_first_over_threshold_sample():
    assert supervisor_params()['slope_roll_detection_hold_sec'] == 0.50

    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.slope_branch_completed = False
    node.state = node.STATE_SEARCH
    node.search_reason = 'post_stairs_entry_done'
    node.slope_roll_detection_enabled = True
    node.slope_roll_detection_post_stairs_only = True
    node.slope_roll_detection_rad = math.radians(8.0)
    node.slope_roll_detection_use_abs = True
    node.slope_roll_detection_hold_sec = 0.50
    node.slope_roll_detection_start_sec = 0.0
    node.last_slope_roll_detection_log_sec = 0.0
    node.pose_roll = math.radians(9.0)
    node._pose_is_fresh = lambda now: True
    node._upstairs_ready = lambda now: False
    node._upstairs_slope_guard_ready = lambda now: False
    node._upstairs_tag_detection_guard_ready = lambda now: False
    node.feedback = []
    node._publish_feedback = node.feedback.append

    assert not node._slope_roll_ready(10.0)
    assert node.slope_roll_detection_start_sec == 10.0

    node.pose_roll = math.radians(7.0)
    assert not node._slope_roll_ready(10.30)
    assert node.slope_roll_detection_start_sec == 10.0

    assert node._slope_roll_ready(10.50)
    assert 'first_sample' in node.feedback[-1]


def test_sandpit_pre_align_targets_id10_and_matches_pole_v4_turns():
    sandpit = standard_waypoint_config()['sandpit_bypass']
    apriltag = sandpit['apriltag']
    pre_align = sandpit['pre_align']
    pole = pole_v4_params()

    assert apriltag['target_tag_id'] == 10
    assert apriltag['frame_id'] == 'camera_color_optical_frame'
    assert apriltag['child_frame_id'] == 'object'
    assert pre_align['polyline_distance_scale'] == 0.65
    assert pre_align['yaw_turn_mode'] == pole['pole_pre_align_turn_mode']
    assert pre_align['yaw_cw_left_norm'] == pole['pole_pre_align_cw_left_norm']
    assert pre_align['yaw_cw_right_norm'] == pole['pole_pre_align_cw_right_norm']
    assert pre_align['yaw_ccw_left_norm'] == pole['pole_pre_align_ccw_left_norm']
    assert pre_align['yaw_ccw_right_norm'] == pole['pole_pre_align_ccw_right_norm']


def test_pole_v4_pre_align_scales_lateral_shift_before_absolute_cap():
    pole = pole_v4_params()

    assert pole['pole_pre_align_shift_distance_scale'] == 0.65
    assert pole['pole_pre_align_shift_max_m'] == 0.20


def test_pole_v1_shared_control_config_matches_v4_without_overwriting_route_tuning():
    v1 = pole_v1_params()
    v4 = pole_v4_params()
    route_specific_keys = {
        'forward_tracking_lateral_gain_waypoint_indices',
        'forward_tracking_lateral_gain_waypoint_values',
        'forward_tracking_arrival_lateral_tolerance_waypoint_indices',
        'forward_tracking_arrival_lateral_tolerance_waypoint_values',
        'body_relative_turn_deltas_deg',
        'waypoint_arrival_tolerances',
        'body_relative_waypoint_offsets_fl',
    }

    assert {
        key: value for key, value in v1.items() if key not in route_specific_keys
    } == {
        key: value for key, value in v4.items() if key not in route_specific_keys
    }
    assert v1['body_relative_turn_deltas_deg'] != v4['body_relative_turn_deltas_deg']
    assert v1['body_relative_waypoint_offsets_fl'] != v4['body_relative_waypoint_offsets_fl']


def test_pole_configs_enable_same_coordinate_free_lidar_route_with_legacy_rollback():
    v1 = pole_v1_params()
    v4 = pole_v4_params()

    assert v1['lidar_slalom_enabled'] is True
    assert v4['lidar_slalom_enabled'] is True
    assert v1['lidar_slalom_lock_samples'] == 3
    assert v1['lidar_slalom_lock_consistency_m'] == 0.06
    assert v1['lidar_slalom_template_points_rl'] == v4['lidar_slalom_template_points_rl']
    assert v1['lidar_slalom_fallback_to_legacy_before_motion'] is True
    assert v1['lidar_slalom_runtime_cloud_stale_stop_enabled'] is False
    assert v1['lidar_slalom_robot_length_m'] == 0.66
    assert v1['lidar_slalom_robot_width_m'] == 0.46
    assert v1['lidar_slalom_runtime_geometry_stop_enabled'] is False
    assert v1['lidar_slalom_min_body_clearance_m'] == 0.04
    assert v1['lidar_slalom_mandatory_zone_tolerance_m'] == 0.14
    assert v1['lidar_slalom_route_timeout_sec'] == 50.0
    assert v1['lidar_slalom_final_absolute_yaw_enabled'] is True
    assert v1['lidar_slalom_final_absolute_yaw_deg'] == 0.0
    assert v1['lidar_slalom_final_absolute_yaw_tolerance_deg'] == 8.0
    assert v1['lidar_slalom_final_curve_anchor_rl'] == [2.10, -0.45]
    assert v1['lidar_slalom_final_curve_yaw_blend_start'] == 0.80
    assert v1['lidar_slalom_final_turn_enabled'] is False
    assert v1['lidar_slalom_final_turn_delta_deg'] == 90.0
    assert v1['lidar_slalom_final_turn_mode'] == 6
    assert v1['lidar_slalom_final_turn_finish_immediately'] is True
    assert v1['lidar_slalom_final_turn_ccw_left_norm'] == -0.20
    assert v1['lidar_slalom_final_turn_ccw_right_norm'] == 0.65
    assert v1['lidar_slalom_final_turn_fine_left_norm'] == -0.10
    assert v1['lidar_slalom_final_turn_fine_right_norm'] == 0.35


def test_full_flow_launch_connects_lidar_pole_branch_to_supervisor():
    launch_path = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'obstacle_race_visual_supervisor.launch.py'
    )
    source = launch_path.read_text(encoding='utf-8')

    assert "executable='orange_pole_lidar_slalom_node'" in source
    assert "'active_topic': '/orange_pole_relative_nav/active'" in source
    assert "executable='obstacle_race_supervisor_node'" in source
    assert "'pole_bypass_file': body_relative_nav_config" in source


def test_phase_two_s_curve_enables_hurdle_and_upstairs_shadow_modes():
    params = supervisor_params()

    assert params['lateral_s_curve_shadow_enabled'] is True
    assert params['hurdle_lateral_s_curve_shadow_enabled'] is True
    assert params['limit_bar_lateral_s_curve_shadow_enabled'] is True
    assert params['upstairs_lateral_s_curve_shadow_enabled'] is True


def test_phase_two_actual_control_enables_hurdle_and_upstairs_with_rollback():
    params = supervisor_params()

    assert params['lateral_s_curve_control_enabled'] is True
    assert params['hurdle_lateral_s_curve_control_enabled'] is True
    assert params['limit_bar_lateral_s_curve_control_enabled'] is True
    assert params['upstairs_lateral_s_curve_control_enabled'] is True
    assert params['limit_bar_lateral_s_curve_control_cruise_left_norm'] == 0.32
    assert params['limit_bar_lateral_s_curve_control_cruise_right_norm'] == 0.32
    assert params['limit_bar_lateral_s_curve_control_slow_left_norm'] == 0.20
    assert params['limit_bar_lateral_s_curve_control_slow_right_norm'] == 0.20
    assert params['limit_bar_lateral_s_curve_control_slowdown_distance_m'] == 0.18
    assert params['limit_bar_lateral_s_curve_control_timeout_sec'] == 8.0
    assert params['limit_bar_lateral_s_curve_fail_open_enabled'] is True
    assert (
        params['limit_bar_lateral_s_curve_control_max_forward_overshoot_m']
        == 0.08
    )
    assert (
        params['limit_bar_lateral_s_curve_control_max_abs_cross_track_m']
        == 0.12
    )
    assert params['limit_bar_lateral_s_curve_control_forward_tolerance_m'] == 0.03
    assert params['limit_bar_lateral_s_curve_control_lateral_tolerance_m'] == 0.05
    assert params['limit_bar_lateral_s_curve_control_arrival_settle_sec'] == 0.20
    assert params['limit_bar_lateral_s_curve_control_pre_stop_distance_m'] == 0.12
    assert params['limit_bar_lateral_s_curve_control_pre_stop_hold_sec'] == 0.20
    assert (
        params[
            'limit_bar_lateral_s_curve_control_final_abs_lateral_tolerance_m'
        ]
        == 0.08
    )
    assert (
        params[
            'limit_bar_lateral_s_curve_control_final_max_lateral_drift_m'
        ]
        == 0.05
    )
    assert (
        params[
            'limit_bar_lateral_s_curve_control_final_lateral_violation_hold_sec'
        ]
        == 0.20
    )
    assert params['limit_bar_lateral_s_curve_control_final_align_norm'] == 0.08
    assert params['limit_bar_stop_nav_executor_on_entry_enabled'] is True
    assert params['limit_bar_pre_duck_yaw_reference_mode'] == 'base_yaw'
    assert params['limit_bar_pre_duck_global_yaw_tolerance_deg'] == 6.0
    assert params['limit_bar_pre_duck_global_yaw_exit_tolerance_deg'] == 8.0
    assert params['limit_bar_pre_duck_global_yaw_fail_open_enabled'] is True
    assert params['limit_bar_pre_duck_stationary_turn_enabled'] is True
    assert params['limit_bar_pre_duck_stationary_turn_norm'] == 0.08
    assert params['limit_bar_tag_yaw_reference_mode'] == 'fixed_absolute'
    assert params['limit_bar_tag_fixed_reference_yaw_deg'] == 180.0
    assert params['limit_bar_tag_min_samples'] == 1
    assert params['limit_bar_tag_geometry_lock_enabled'] is True
    assert params['limit_bar_tag_geometry_lock_sample_count'] == 3
    assert params['limit_bar_lateral_pre_align_mode'] == 'fixed_absolute_yaw'
    assert params['limit_bar_lateral_pre_align_use_visual_feedback'] is False
    assert params['limit_bar_lateral_pre_align_stop_during_hold_enabled'] is True
    assert params['limit_bar_lateral_s_curve_max_heading_deg'] == 42.0
    assert params['limit_bar_lateral_s_curve_max_curvature_m_inv'] == 3.5
    assert params['limit_bar_lateral_turn_timeout_settle_grace_sec'] == 0.60
    assert params['limit_bar_tag_trigger_distance_m'] == 1.40
    assert params['limit_bar_lateral_shift_target_distance_m'] == 0.42
    assert params['upstairs_lateral_s_curve_control_cruise_left_norm'] == 0.40
    assert params['upstairs_lateral_s_curve_control_cruise_right_norm'] == 0.40
    assert params['upstairs_lateral_s_curve_control_slow_left_norm'] == 0.25
    assert params['upstairs_lateral_s_curve_control_slow_right_norm'] == 0.25
    assert params['upstairs_lateral_s_curve_control_timeout_sec'] == 12.0
    assert (
        params['upstairs_lateral_s_curve_control_max_forward_overshoot_m']
        == 0.20
    )
    assert (
        params['upstairs_lateral_s_curve_control_distance_max_remaining_m']
        == 0.25
    )
    assert (
        params['upstairs_lateral_s_curve_control_precreep_yaw_gate_deg']
        == 4.0
    )
    assert params['upstairs_lateral_s_curve_control_final_align_norm'] == 0.08
    assert params['lateral_s_curve_control_timeout_sec'] > 0.0
    assert params['lateral_s_curve_control_max_forward_overshoot_m'] > 0.0
    assert params['lateral_s_curve_shadow_max_heading_deg'] == 30.0
    assert params['lateral_s_curve_control_walk_yaw_gate_deg'] == 18.0
    assert params['lateral_s_curve_control_lookahead_m'] == 0.20
    assert params['lateral_s_curve_control_cruise_left_norm'] == 0.43
    assert params['lateral_s_curve_control_cruise_right_norm'] == 0.43
    assert params['lateral_s_curve_control_yaw_tolerance_deg'] == 3.0
    assert params['lateral_s_curve_control_arrival_settle_sec'] == 0.30
    assert params['lateral_s_curve_control_completion_max_retreat_m'] == 0.05
    assert params['lateral_s_curve_control_completion_recovery_enabled'] is True
    assert (
        params[
            'lateral_s_curve_control_completion_recovery_forward_tolerance_m'
        ]
        == 0.015
    )
    assert (
        params['lateral_s_curve_control_completion_recovery_yaw_tolerance_deg']
        == 4.0
    )
    assert params['hurdle_lateral_s_curve_visual_jump_confirm_samples'] == 3
    assert (
        params['hurdle_lateral_s_curve_visual_jump_confirm_duration_sec']
        == 0.15
    )
    assert (
        params['hurdle_lateral_s_curve_visual_jump_max_remaining_m']
        == 0.35
    )
    assert (
        params['hurdle_lateral_s_curve_visual_jump_tag_max_delta_m']
        == 0.40
    )
    assert (
        params['hurdle_lateral_s_curve_final_align_independent_turn_enabled']
        is True
    )
    assert (
        params[
            'hurdle_lateral_s_curve_visual_completion_ignore_plan_overshoot_enabled'
        ]
        is True
    )
    assert (
        params[
            'hurdle_lateral_s_curve_visual_completion_max_plan_overshoot_m'
        ]
        == 0.25
    )
    assert params['hurdle_lateral_s_curve_adaptive_clamp_enabled'] is True
    assert params['hurdle_lateral_s_curve_adaptive_clamp_min_scale'] == 0.90
    assert params['hurdle_lateral_shift_visual_jump_distance_m'] == 0.55
    assert (
        params['hurdle_lateral_shift_legacy_relock_final_forward_enabled']
        is True
    )
    assert params['hurdle_lateral_shift_legacy_relock_visual_cap_enabled'] is True
    assert params['hurdle_lateral_shift_legacy_slowdown_distance_m'] == 0.25
    assert params['hurdle_lateral_shift_legacy_max_forward_overshoot_m'] > 0.0
    assert params['hurdle_lateral_pre_align_tolerance_deg'] == 3.0
    assert params['hurdle_lateral_pre_align_stop_during_hold_enabled'] is True
    assert params['hurdle_lateral_pre_align_independent_turn_enabled'] is True


def test_supervisor_overlay_preserves_pre_duck_yaw_fail_open():
    params = supervisor_overlay_params()

    assert params['limit_bar_pre_duck_global_yaw_tolerance_deg'] == 6.0
    assert params['limit_bar_pre_duck_global_yaw_exit_tolerance_deg'] == 8.0
    assert params['limit_bar_pre_duck_global_yaw_timeout_sec'] == 4.0
    assert params['limit_bar_pre_duck_global_yaw_fail_open_enabled'] is True


def test_post_stairs_initial_yaw_alignment_matches_enabled_configuration():
    params = supervisor_params()

    assert params['slope_align_enabled'] is True
    assert params['post_stairs_slope_entry_initial_yaw_align_enabled'] is True


def test_failed_state_holds_stop_and_cancels_stale_navigation():
    params = supervisor_params()

    assert params['failed_hold_step_override_enabled'] is True
    assert params['failed_stop_nav_executor_enabled'] is True
    assert params['failed_hold_mode'] == 0
    assert params['failed_hold_left_norm'] == 0.0
    assert params['failed_hold_right_norm'] == 0.0
