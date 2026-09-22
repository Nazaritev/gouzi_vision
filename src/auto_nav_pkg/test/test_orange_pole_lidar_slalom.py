import math

import numpy as np

from auto_nav_pkg.lidar_slalom_core import SlalomFrame
from auto_nav_pkg.orange_pole_lidar_slalom import OrangePoleLidarSlalom


def make_final_turn_node():
    node = OrangePoleLidarSlalom.__new__(OrangePoleLidarSlalom)
    node.lidar_frame = SlalomFrame.from_p4_and_entry_heading(
        p4_xy=(0.0, 1.0),
        entry_forward_xy=(0.0, 1.0),
        pole_spacing_m=1.0,
    )
    node.lidar_final_turn_delta_rad = math.radians(90.0)
    node.lidar_final_turn_mode = 6
    node.lidar_final_turn_timeout_sec = 12.0
    node.lidar_final_turn_tolerance_rad = math.radians(3.0)
    node.lidar_final_turn_finish_immediately = True
    node.lidar_final_turn_max_yaw_rate_rad_s = math.radians(6.0)
    node.lidar_final_turn_hold_sec = 0.20
    node.lidar_final_turn_fine_error_rad = math.radians(12.0)
    node.lidar_final_turn_ccw_left_norm = -0.20
    node.lidar_final_turn_ccw_right_norm = 0.65
    node.lidar_final_turn_cw_left_norm = 0.65
    node.lidar_final_turn_cw_right_norm = -0.20
    node.lidar_final_turn_fine_left_norm = -0.10
    node.lidar_final_turn_fine_right_norm = 0.35
    node.lidar_progress_log_interval_sec = 1.0
    node.lidar_last_progress_log_sec = 0.0
    node.pose_yaw = math.radians(-90.0)
    node.pose_yaw_rate_rad_s = 0.0
    node.commands = []
    node.states = []
    node.feedback = []
    node.finished = []
    node.failed = []
    node.now = 10.0
    node._now_sec = lambda: node.now
    node._publish_step_override = lambda *args: node.commands.append(args)
    node._publish_state = node.states.append
    node._publish_feedback = node.feedback.append
    node._finish_lidar_slalom = lambda: node.finished.append(True)
    node._stop_and_fail_lidar = node.failed.append
    return node


def test_final_turn_locks_current_yaw_and_commands_fast_mode_6_ccw():
    node = make_final_turn_node()

    node._start_lidar_final_turn()
    node._monitor_lidar_final_turn(node.now + 0.1)

    assert node.lidar_slalom_phase == 'FINAL_TURN'
    assert math.isclose(node.lidar_final_turn_start_yaw, math.radians(-90.0))
    assert math.isclose(node.lidar_final_turn_target_yaw, 0.0, abs_tol=1e-12)
    assert node.commands[-1] == (6, -0.20, 0.65)
    assert not node.finished
    assert not node.failed


def test_running_switches_to_final_turn_immediately_after_zone_three():
    node = OrangePoleLidarSlalom.__new__(OrangePoleLidarSlalom)
    node.route_finished = False
    node.route_failed = False
    node.lidar_slalom_phase = 'RUNNING'
    node.lidar_have_full_odom = True
    node.lidar_last_cloud_receive_sec = 10.0
    node.lidar_cloud_stale_timeout_sec = 0.40
    node.lidar_run_start_sec = 0.0
    node.lidar_route_timeout_sec = 50.0
    node.lidar_zone_indices = [10, 20, 30]
    node.lidar_zone_cursor = 2
    node.lidar_final_turn_enabled = True
    started = []
    node._publish_status_heartbeat = lambda: None
    node._refresh_route_mode = lambda: None
    node._now_sec = lambda: 10.0
    node._pose_is_stale = lambda: False
    node._try_lock_next_pole = lambda: None
    node._update_mandatory_zone_progress = lambda: setattr(
        node, 'lidar_zone_cursor', 3
    )
    node._start_lidar_final_turn = lambda: started.append(True)
    node._minimum_pole_clearance = lambda *_: (_ for _ in ()).throw(
        AssertionError('path tracking must not continue after zone 3')
    )

    node._monitor_lidar_slalom()

    assert started == [True]


def test_running_exit_curve_locks_absolute_yaw_before_finishing():
    node = OrangePoleLidarSlalom.__new__(OrangePoleLidarSlalom)
    node.route_finished = False
    node.route_failed = False
    node.lidar_slalom_phase = 'RUNNING'
    node.lidar_have_full_odom = True
    node.lidar_last_cloud_receive_sec = 10.0
    node.lidar_cloud_stale_timeout_sec = 0.40
    node.lidar_runtime_cloud_stale_stop_enabled = False
    node.lidar_run_start_sec = 0.0
    node.lidar_route_timeout_sec = 50.0
    node.lidar_zone_indices = [0, 0, 1]
    node.lidar_zone_cursor = 3
    node.lidar_final_turn_enabled = False
    node.lidar_final_absolute_yaw_enabled = True
    node.lidar_final_absolute_yaw_rad = 0.0
    node.lidar_final_absolute_yaw_tolerance_rad = math.radians(8.0)
    node.lidar_final_curve_start_index = 0
    node.lidar_final_curve_min_lookahead_m = 0.04
    node.lidar_final_curve_minimum_norm = 0.10
    node.lidar_final_curve_max_delta_norm = 0.38
    node.lidar_final_curve_yaw_blend_start = 0.80
    node.lidar_path_xy = np.asarray(((0.0, 0.0), (0.0, 0.10)))
    node.lidar_path_index = 0
    node.lidar_lookahead_m = 0.20
    node.lidar_finish_distance_m = 0.16
    node.pose_x = 0.0
    node.pose_y = 0.05
    node.pose_yaw = math.radians(-20.0)
    node.lidar_runtime_geometry_stop_enabled = False
    node.lidar_min_body_clearance_m = 0.04
    node.lidar_max_cross_track_m = 0.28
    node.lidar_slalom_mode = 0
    node.lidar_cruise_norm = 0.42
    node.lidar_minimum_norm = 0.18
    node.lidar_heading_gain_norm_per_rad = 0.70
    node.lidar_max_delta_norm = 0.28
    node.lidar_slowdown_heading_rad = math.radians(35.0)
    node.lidar_locked_poles = {}
    node.lidar_last_progress_log_sec = 10.0
    node.lidar_progress_log_interval_sec = 1.0
    commands = []
    finished = []
    node._publish_status_heartbeat = lambda: None
    node._refresh_route_mode = lambda: None
    node._now_sec = lambda: 10.0
    node._pose_is_stale = lambda: False
    node._try_lock_next_pole = lambda: None
    node._minimum_pole_clearance = lambda *_: 1.0
    node._minimum_body_clearance = lambda *_: 1.0
    node._publish_step_override = lambda *args: commands.append(args)
    node._publish_state = lambda *_: None
    node._publish_feedback = lambda *_: None
    node._finish_lidar_slalom = lambda: finished.append(True)
    node._start_lidar_final_turn = lambda: (_ for _ in ()).throw(
        AssertionError('absolute-yaw exit must not enter FINAL_TURN')
    )

    node._monitor_lidar_slalom()

    assert not finished
    assert commands[-1][1] < commands[-1][2]

    node.pose_yaw = 0.0
    node._monitor_lidar_slalom()

    assert finished == [True]


def test_p4_lock_does_not_recount_the_same_cloud_frame():
    node = OrangePoleLidarSlalom.__new__(OrangePoleLidarSlalom)
    node.lidar_slalom_mode = 0
    node.lidar_cloud_sequence = 7
    node.lidar_lock_last_cloud_sequence = 7
    node._publish_step_override = lambda *_: None
    node._raw_cloud_is_stale = lambda *_: False
    node._aggregate_lidar_points = lambda: (_ for _ in ()).throw(
        AssertionError('same cloud frame must not be evaluated twice')
    )

    node._monitor_p4_lock(10.0)


def test_sequential_pole_lock_does_not_recount_the_same_cloud_frame():
    node = OrangePoleLidarSlalom.__new__(OrangePoleLidarSlalom)
    node.lidar_frame = SlalomFrame.from_p4_and_entry_heading(
        p4_xy=(0.0, 1.0),
        entry_forward_xy=(0.0, 1.0),
        pole_spacing_m=1.0,
    )
    node.lidar_locked_poles = {0: node.lidar_frame.pole_centers_world()[0]}
    node.lidar_cloud_sequence = 8
    node.lidar_track_last_cloud_sequence = 8
    node._aggregate_lidar_points = lambda: (_ for _ in ()).throw(
        AssertionError('same cloud frame must not refine the next pole twice')
    )

    node._try_lock_next_pole()


def test_runtime_violation_warns_and_continues_when_stop_is_disabled():
    node = OrangePoleLidarSlalom.__new__(OrangePoleLidarSlalom)
    node.lidar_runtime_geometry_stop_enabled = False
    node.lidar_last_runtime_guard_warning_sec = 0.0
    node.lidar_progress_log_interval_sec = 1.0
    feedback = []
    failed = []
    node._publish_feedback = feedback.append
    node._stop_and_fail_lidar = failed.append

    stopped = node._handle_lidar_runtime_violation(
        10.0,
        stop_enabled=False,
        reason='body_to_pole_clearance=0.018m<0.040m',
    )

    assert stopped is False
    assert not failed
    assert '异常仅告警并继续' in feedback[-1]


def test_runtime_violation_still_stops_when_enabled():
    node = OrangePoleLidarSlalom.__new__(OrangePoleLidarSlalom)
    node.lidar_runtime_geometry_stop_enabled = True
    failed = []
    node._stop_and_fail_lidar = failed.append

    stopped = node._handle_lidar_runtime_violation(
        10.0,
        stop_enabled=True,
        reason='cross_track=0.300m>0.280m',
    )

    assert stopped is True
    assert failed == ['cross_track=0.300m>0.280m']


def test_runtime_raw_cloud_stale_warns_and_continues_after_path_started():
    node = OrangePoleLidarSlalom.__new__(OrangePoleLidarSlalom)
    node.lidar_last_runtime_guard_warning_sec = 0.0
    node.lidar_progress_log_interval_sec = 1.0
    feedback = []
    failed = []
    node._publish_feedback = feedback.append
    node._stop_and_fail_lidar = failed.append

    stopped = node._handle_lidar_runtime_violation(
        10.0,
        stop_enabled=False,
        reason='raw_lidar_cloud_stale age=0.500s>0.400s poles=4/4',
    )

    assert stopped is False
    assert not failed
    assert 'raw_lidar_cloud_stale' in feedback[-1]


def test_final_turn_finishes_immediately_inside_tolerance():
    node = make_final_turn_node()
    node._start_lidar_final_turn()
    node.pose_yaw = node.lidar_final_turn_target_yaw

    node._monitor_lidar_final_turn(11.0)

    assert node.finished == [True]
    assert not node.failed


def test_final_turn_finishes_on_first_target_crossing_outside_tolerance():
    node = make_final_turn_node()
    node._start_lidar_final_turn()
    node.pose_yaw = math.radians(4.0)

    node._monitor_lidar_final_turn(11.0)

    assert node.finished == [True]
    assert math.degrees(node.lidar_final_turn_progress_rad) == 94.0
    assert not node.failed


def test_final_turn_can_restore_legacy_stable_yaw_hold():
    node = make_final_turn_node()
    node.lidar_final_turn_finish_immediately = False
    node._start_lidar_final_turn()
    node.pose_yaw = node.lidar_final_turn_target_yaw

    node._monitor_lidar_final_turn(11.0)
    assert not node.finished
    node._monitor_lidar_final_turn(11.19)
    assert not node.finished
    node._monitor_lidar_final_turn(11.21)

    assert node.finished == [True]
    assert not node.failed
