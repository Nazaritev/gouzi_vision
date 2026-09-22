import math
from types import SimpleNamespace

from auto_nav_pkg.obstacle_race_supervisor import ObstacleRaceSupervisor


def make_retry_stub():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.state = node.STATE_SLOPE
    node.pose_x = 1.0
    node.pose_y = 2.0
    node.pose_yaw = math.radians(11.0)
    node.slope_current_waypoint_name = 'platform_center'
    node.slope_branch_completed = True
    node.slope_retry_route_rezero_once = True
    node.slope_retry_route_rezero_used = False
    node.slope_retry_rezero_search_active = False
    node.slope_bridge_retry_count = 0
    node.slope_retry_rezero_rearm_delay_sec = 0.3
    node.search_rearm_delay_sec = 0.8
    node.slope_route_yaw_reference = None
    node.search_reference_yaw = None
    node.search_arm_deadline = 0.0
    node._pose_is_fresh = lambda now: True
    node._clear_step_override = lambda: None
    node._reset_detection_states = lambda: None
    node._reset_transient_branch_state = lambda: None
    node._publish_active_flags = lambda: None
    node.feedback = []
    node._publish_feedback = node.feedback.append
    node._enter_search = (
        lambda now, reason: setattr(node, 'state', node.STATE_SEARCH)
    )

    def publish_route_reference(now, yaw):
        node.slope_route_yaw_reference = yaw

    node._publish_slope_route_yaw_reference = publish_route_reference
    return node


def test_pre_bridge_retry_latches_route_zero_and_returns_to_search():
    node = make_retry_stub()

    node._handle_slope_route_rezero_retry(10.0)

    assert node.state == node.STATE_SEARCH
    assert node.slope_retry_rezero_search_active
    assert node.slope_retry_route_rezero_used
    assert not node.slope_branch_completed
    assert math.isclose(node.slope_route_yaw_reference, math.radians(11.0))
    assert math.isclose(node.search_reference_yaw, math.radians(11.0))
    assert math.isclose(node.search_arm_deadline, 10.3)


def test_second_pre_bridge_retry_reuses_first_route_zero():
    node = make_retry_stub()
    node.slope_retry_route_rezero_used = True
    node.slope_route_yaw_reference = math.radians(7.0)

    node._handle_slope_route_rezero_retry(10.0)

    assert math.isclose(node.slope_route_yaw_reference, math.radians(7.0))
    assert math.isclose(node.search_reference_yaw, math.radians(7.0))


def test_bridge_retry_latches_heading_and_restarts_turn_180_settle():
    node = make_retry_stub()
    node.slope_current_waypoint_name = 'turn_180_yaw_settle'
    captured = {}

    def publish_bridge_reference(now, yaw):
        captured['yaw'] = yaw

    def restart(name, feedback):
        captured['restart'] = name
        captured['feedback'] = feedback

    node._publish_slope_bridge_retry_yaw_reference = publish_bridge_reference
    node._restart_slope_manager_from = restart

    node._handle_slope_bridge_retry(10.0)

    assert math.isclose(captured['yaw'], math.radians(11.0))
    assert captured['restart'] == 'turn_180_yaw_settle'
    assert node.slope_bridge_retry_count == 1
    assert '锁存桥方向基准' in captured['feedback']


def test_first_three_bridge_retries_cross_bridge_and_fourth_uses_exit_spin():
    node = make_retry_stub()
    node.slope_current_waypoint_name = 'bridge_exit'
    captured = {'references': [], 'restarts': []}

    node._publish_slope_bridge_retry_yaw_reference = (
        lambda now, yaw: captured['references'].append(yaw)
    )
    node._restart_slope_manager_from = (
        lambda name, feedback: captured['restarts'].append((name, feedback))
    )

    node._handle_slope_bridge_retry(10.0)
    node.pose_yaw = math.radians(13.0)
    node._handle_slope_bridge_retry(11.0)
    node.pose_yaw = math.radians(15.0)
    node._handle_slope_bridge_retry(12.0)
    node.pose_yaw = math.radians(17.0)
    node._handle_slope_bridge_retry(13.0)

    assert node.slope_bridge_retry_count == 4
    assert math.isclose(captured['references'][-1], math.radians(17.0))
    assert captured['restarts'][0][0] == 'turn_180_yaw_settle'
    assert captured['restarts'][1][0] == 'turn_180_yaw_settle'
    assert captured['restarts'][2][0] == 'turn_180_yaw_settle'
    assert captured['restarts'][3][0] == 'bridge_exit_ccw_45_spin'
    assert '第 4 次重试' in captured['restarts'][3][1]


def test_mirror_fourth_bridge_retry_uses_mirror_exit_spin():
    node = make_retry_stub()
    node.slope_current_waypoint_name = 'bridge_exit'
    node.slope_retry_mirror_waypoints_basename = 'obstacle_waypoints_mirror.yaml'
    node.sandpit_bypass_file = '/tmp/obstacle_waypoints_mirror.yaml'
    captured = []

    node._publish_slope_bridge_retry_yaw_reference = lambda now, yaw: None
    node._restart_slope_manager_from = (
        lambda name, feedback: captured.append((name, feedback))
    )

    for attempt in range(4):
        node._handle_slope_bridge_retry(10.0 + attempt)

    assert captured[3][0] == 'bridge_exit_cw_45_spin_mirror'
    assert '第 4 次重试' in captured[3][1]


def test_runtime_yaw_retry_feature_enables_for_mirror_file():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.slope_retry_runtime_yaw_reference_enabled = True
    node.slope_retry_standard_waypoints_basename = 'obstacle_waypoints.yaml'
    node.slope_retry_mirror_waypoints_basename = 'obstacle_waypoints_mirror.yaml'
    node.sandpit_bypass_file = '/tmp/obstacle_waypoints_mirror.yaml'

    assert node._slope_runtime_yaw_reference_enabled()


def test_runtime_yaw_retry_feature_stays_disabled_for_unknown_file():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.slope_retry_runtime_yaw_reference_enabled = True
    node.slope_retry_standard_waypoints_basename = 'obstacle_waypoints.yaml'
    node.slope_retry_mirror_waypoints_basename = 'obstacle_waypoints_mirror.yaml'
    node.sandpit_bypass_file = '/tmp/custom_waypoints.yaml'

    assert not node._slope_runtime_yaw_reference_enabled()


def test_runtime_yaw_retry_feature_enables_for_standard_file():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.slope_retry_runtime_yaw_reference_enabled = True
    node.slope_retry_standard_waypoints_basename = 'obstacle_waypoints.yaml'
    node.sandpit_bypass_file = '/tmp/obstacle_waypoints.yaml'

    assert node._slope_runtime_yaw_reference_enabled()


def test_retry_dispatch_uses_bridge_logic_from_turn_180_settle():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.slope_current_waypoint_name = 'turn_180_yaw_settle'
    node.slope_retry_runtime_yaw_reference_enabled = True
    node.slope_retry_standard_waypoints_basename = 'obstacle_waypoints.yaml'
    node.sandpit_bypass_file = '/tmp/obstacle_waypoints.yaml'
    node.slope_retry_bridge_waypoints = {'turn_180_yaw_settle', 'bridge_entry'}
    node.slope_retry_pre_bridge_waypoints = {'platform_center', 'turn_180_spin'}
    called = []
    node._handle_slope_bridge_retry = lambda now: called.append(('bridge', now))
    node._handle_slope_route_rezero_retry = lambda now: called.append(('route', now))

    node._handle_slope_retry(10.0)

    assert called == [('bridge', 10.0)]


def test_retry_dispatch_keeps_final_descent_mid_spin_in_bridge_logic():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.slope_current_waypoint_name = 'final_descent_mid_spin'
    node.slope_retry_runtime_yaw_reference_enabled = True
    node.slope_retry_standard_waypoints_basename = 'obstacle_waypoints.yaml'
    node.sandpit_bypass_file = '/tmp/obstacle_waypoints.yaml'
    node.slope_retry_bridge_waypoints = {'final_descent_mid_spin'}
    node.slope_retry_pre_bridge_waypoints = set()
    node.slope_retry_search_waypoints = {'final_descent_second_approach'}
    called = []
    node._handle_slope_bridge_retry = lambda now: called.append(('bridge', now))
    node._handle_slope_search_retry = lambda now: called.append(('search', now))

    node._handle_slope_retry(10.0)

    assert called == [('bridge', 10.0)]


def test_retry_dispatch_keeps_mirror_mid_spin_in_bridge_logic():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.slope_current_waypoint_name = 'final_descent_mid_spin_mirror'
    node.slope_retry_runtime_yaw_reference_enabled = True
    node.slope_retry_standard_waypoints_basename = 'obstacle_waypoints.yaml'
    node.slope_retry_mirror_waypoints_basename = 'obstacle_waypoints_mirror.yaml'
    node.sandpit_bypass_file = '/tmp/obstacle_waypoints_mirror.yaml'
    node.slope_retry_bridge_waypoints = {'final_descent_mid_spin_mirror'}
    node.slope_retry_pre_bridge_waypoints = set()
    node.slope_retry_search_waypoints = {'final_descent_second_approach'}
    called = []
    node._handle_slope_bridge_retry = lambda now: called.append(('bridge', now))
    node._handle_slope_search_retry = lambda now: called.append(('search', now))

    node._handle_slope_retry(10.0)

    assert called == [('bridge', 10.0)]


def test_retry_dispatch_returns_to_search_after_final_descent_mid_spin():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.slope_current_waypoint_name = 'final_descent_second_approach'
    node.slope_retry_search_waypoints = {'final_descent_second_approach'}
    called = []
    node._handle_slope_search_retry = lambda now: called.append(('search', now))

    node._handle_slope_retry(10.0)

    assert called == [('search', 10.0)]


def test_post_mid_slope_retry_clears_branch_and_enters_search():
    node = make_retry_stub()
    node.slope_current_waypoint_name = 'final_descent_third_approach'

    node._handle_slope_search_retry(10.0)

    assert node.state == node.STATE_SEARCH
    assert not node.slope_branch_completed
    assert not node.slope_retry_rezero_search_active
    assert math.isclose(node.search_arm_deadline, 10.8)
    assert '斜坡后段重试信号' in node.feedback[-1]
    assert '返回搜索模式' in node.feedback[-1]


def test_retry_dispatch_uses_rezero_logic_before_turn_180_settle():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.slope_current_waypoint_name = 'turn_180_spin'
    node.slope_retry_runtime_yaw_reference_enabled = True
    node.slope_retry_standard_waypoints_basename = 'obstacle_waypoints.yaml'
    node.sandpit_bypass_file = '/tmp/obstacle_waypoints.yaml'
    node.slope_retry_bridge_waypoints = {'turn_180_yaw_settle', 'bridge_entry'}
    node.slope_retry_pre_bridge_waypoints = {'platform_center', 'turn_180_spin'}
    called = []
    node._handle_slope_bridge_retry = lambda now: called.append(('bridge', now))
    node._handle_slope_route_rezero_retry = lambda now: called.append(('route', now))

    node._handle_slope_retry(10.0)

    assert called == [('route', 10.0)]


def test_mirror_retry_dispatch_uses_rezero_logic_before_turn_180_settle():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.slope_current_waypoint_name = 'platform_center'
    node.slope_retry_runtime_yaw_reference_enabled = True
    node.slope_retry_standard_waypoints_basename = 'obstacle_waypoints.yaml'
    node.slope_retry_mirror_waypoints_basename = 'obstacle_waypoints_mirror.yaml'
    node.sandpit_bypass_file = '/tmp/obstacle_waypoints_mirror.yaml'
    node.slope_retry_bridge_waypoints = {'turn_180_yaw_settle', 'bridge_entry'}
    node.slope_retry_pre_bridge_waypoints = {'platform_center', 'turn_180_spin'}
    node.slope_retry_search_waypoints = set()
    called = []
    node._handle_slope_bridge_retry = lambda now: called.append(('bridge', now))
    node._handle_slope_route_rezero_retry = lambda now: called.append(('route', now))

    node._handle_slope_retry(10.0)

    assert called == [('route', 10.0)]


def test_failed_bridge_retry_still_uses_bridge_restart_logic():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.state = node.STATE_FAILED
    node.failed_from_state = node.STATE_SLOPE
    node.slope_current_waypoint_name = 'bridge_exit'
    node.slope_retry_runtime_yaw_reference_enabled = True
    node.slope_retry_standard_waypoints_basename = 'obstacle_waypoints.yaml'
    node.sandpit_bypass_file = '/tmp/obstacle_waypoints.yaml'
    node.slope_retry_bridge_waypoints = {'bridge_entry', 'bridge_exit'}
    called = []
    node._handle_slope_bridge_retry = lambda now: called.append(('bridge', now))
    node._handle_failed_retry = lambda now: called.append(('failed_search', now))

    node._handle_retry_signal(10.0)

    assert called == [('bridge', 10.0)]


def make_failed_retry_stub(failed_from_state):
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.state = node.STATE_FAILED
    node.failed_from_state = failed_from_state
    node.failed_from_branch = ''
    node.last_failure_text = '测试失败'
    node.slope_branch_completed = True
    node.pose_x = 1.0
    node.pose_y = 2.0
    node.pose_yaw = math.radians(5.0)
    node.search_rearm_delay_sec = 0.4
    node.search_arm_deadline = 0.0
    node._clear_step_override = lambda: None
    node._reset_detection_states = lambda: None
    node._reset_transient_branch_state = lambda: None
    node._publish_active_flags = lambda: None
    node.feedback = []
    node._publish_feedback = node.feedback.append

    def enter_search(now, reason):
        node.state = node.STATE_SEARCH
        node.search_reason = reason

    node._enter_search = enter_search
    return node


def test_failed_limit_bar_retry_returns_to_search():
    node = make_failed_retry_stub(ObstacleRaceSupervisor.STATE_LIMIT_BAR_LATERAL_SHIFT)

    node._handle_retry_signal(10.0)

    assert node.state == node.STATE_SEARCH
    assert node.search_reason == 'retry_after_failed_limit_bar_lateral_shift'
    assert math.isclose(node.search_arm_deadline, 10.4)
    assert node.slope_branch_completed
    assert '状态=FAILED 收到电控 [start] 重试信号' in node.feedback[-1]


def test_failed_slope_retry_rearms_slope_detection():
    node = make_failed_retry_stub(ObstacleRaceSupervisor.STATE_SLOPE)

    node._handle_retry_signal(10.0)

    assert node.state == node.STATE_SEARCH
    assert not node.slope_branch_completed


def test_failed_retry_bypasses_initial_start_guard():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.state = node.STATE_FAILED
    node.start_signal_received = False
    node.start_retry_guard_deadline_sec = 20.0
    node._now_sec = lambda: 10.0
    calls = []
    node._handle_retry_signal = lambda now: calls.append(now)
    node._publish_feedback = lambda text: None

    node._start_signal_callback(SimpleNamespace(data=True))

    assert node.start_signal_received
    assert calls == [10.0]


def test_fail_records_recovery_source_before_entering_failed():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.state = node.STATE_LIMIT_BAR_LATERAL_SHIFT
    node.current_active_branch = 'limit_bar'
    node.failed_hold_step_override_enabled = True
    node.failed_stop_nav_executor_enabled = True
    node.failed_hold_mode = 0
    node.failed_hold_left_norm = 0.0
    node.failed_hold_right_norm = 0.0
    node._clear_step_override = lambda: None
    stop_commands = []
    node._publish_step_override = (
        lambda mode, left, right: stop_commands.append((mode, left, right))
    )
    nav_arrivals = []
    node._publish_nav_arrival = nav_arrivals.append
    published_states = []
    node._publish_state = published_states.append
    node._publish_feedback = lambda text: None

    node._fail('限高杆测试失败')

    assert node.state == node.STATE_FAILED
    assert node.failed_from_state == node.STATE_LIMIT_BAR_LATERAL_SHIFT
    assert node.failed_from_branch == 'limit_bar'
    assert node.last_failure_text == '限高杆测试失败'
    assert published_states == [node.STATE_FAILED]
    assert nav_arrivals == [True]
    assert stop_commands == [(0, 0.0, 0.0)]

    node._run_failed()
    assert stop_commands[-1] == (0, 0.0, 0.0)
