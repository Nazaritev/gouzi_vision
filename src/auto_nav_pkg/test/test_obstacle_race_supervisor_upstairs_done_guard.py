from auto_nav_pkg.obstacle_race_supervisor import ObstacleRaceSupervisor


def make_done_guard_stub():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.state = node.STATE_DONE
    node.upstairs_completed = True
    node.upstairs_done_retry_guard_enabled = True
    node.upstairs_done_retry_guard_duration_sec = 30.0
    node.upstairs_done_retry_guard_deadline_sec = 40.0
    node.upstairs_done_retry_guard_mode = 0
    node.upstairs_done_retry_guard_left_norm = 0.1
    node.upstairs_done_retry_guard_right_norm = 0.1
    node.done_hold_step_override_enabled = True
    node.done_hold_mode = 0
    node.done_hold_left_norm = 0.0
    node.done_hold_right_norm = 0.0
    node.upstairs_jumping_signal_received = False
    node.upstairs_jumping_signal_stamp_sec = 0.0
    node.upstairs_step_once_waiting_ack = False
    node.upstairs_step_once_next_resend_sec = None
    node.upstairs_step_once_resend_count = 0
    node.search_rearm_delay_sec = 0.8
    node.search_arm_deadline = 0.0
    node.published_overrides = []
    node.feedback = []
    node._publish_step_override = (
        lambda mode, left, right: node.published_overrides.append((mode, left, right))
    )
    node._publish_feedback = node.feedback.append
    node._clear_step_override = lambda: None
    node._reset_detection_states = lambda: None
    node._reset_transient_branch_state = lambda: None
    node._publish_active_flags = lambda: None
    node._enter_search = lambda now, reason: (
        setattr(node, 'state', node.STATE_SEARCH),
        setattr(node, 'search_reason', reason),
    )
    return node


def test_done_guard_publishes_small_forward_then_returns_to_done_hold():
    node = make_done_guard_stub()

    node._run_done(20.0)
    assert node.published_overrides[-1] == (0, 0.1, 0.1)

    node._run_done(40.0)
    assert node.published_overrides[-1] == (0, 0.0, 0.0)
    assert node.upstairs_done_retry_guard_deadline_sec is None
    assert '保护窗口结束' in node.feedback[-1]


def test_retry_during_done_guard_rearms_upstairs_and_returns_to_search():
    node = make_done_guard_stub()
    node.upstairs_step_once_waiting_ack = True
    node.upstairs_step_once_next_resend_sec = 21.0
    node.upstairs_step_once_resend_count = 1

    node._handle_retry_signal(20.0)

    assert node.state == node.STATE_SEARCH
    assert node.search_reason == 'retry_during_upstairs_done_guard'
    assert not node.upstairs_completed
    assert node.upstairs_done_retry_guard_deadline_sec is None
    assert not node.upstairs_step_once_waiting_ack
    assert node.upstairs_step_once_next_resend_sec is None
    assert node.upstairs_step_once_resend_count == 0
    assert node.search_arm_deadline == 20.8
    assert '重置为未完成' in node.feedback[-1]


def test_retry_after_done_guard_expired_is_still_ignored():
    node = make_done_guard_stub()

    node._handle_retry_signal(40.0)

    assert node.state == node.STATE_DONE
    assert node.upstairs_completed
    assert '已忽略' in node.feedback[-1]
