import math

from auto_nav_pkg.obstacle_race_supervisor import ObstacleRaceSupervisor


def make_supervisor_stub():
    node = ObstacleRaceSupervisor.__new__(ObstacleRaceSupervisor)
    node.downstream_route_yaw_reference_enabled = True
    node.downstream_route_yaw_reference_timeout_sec = 0.0
    node.downstream_route_yaw_hurdle_enabled = True
    node.downstream_route_yaw_upstairs_enabled = True
    node.downstream_route_yaw_limit_bar_pre_duck_enabled = True
    node.slope_route_yaw_reference = math.radians(11.3)
    node.slope_route_yaw_reference_stamp_sec = 10.0
    node.hurdle_tag_fixed_reference_yaw = 0.0
    node.upstairs_tag_fixed_reference_yaw = 0.0
    node.limit_bar_tag_fixed_reference_yaw = math.pi
    node.limit_bar_pre_duck_yaw_reference_mode = 'route_zero_plus_offset'
    node.limit_bar_pre_duck_global_yaw_rad = math.pi
    node.limit_bar_lateral_shift_base_yaw = math.radians(160.0)
    node.hurdle_post_align_use_startup_yaw = False
    node.startup_pose_yaw = None
    node.hurdle_post_align_target_yaw = 0.0
    return node


def test_runtime_route_zero_adjusts_only_requested_downstream_angles():
    node = make_supervisor_stub()

    assert math.isclose(
        math.degrees(node._hurdle_fixed_reference_target_yaw()),
        11.3,
        abs_tol=1e-9,
    )
    assert math.isclose(
        math.degrees(node._upstairs_fixed_reference_target_yaw()),
        11.3,
        abs_tol=1e-9,
    )
    assert math.isclose(
        math.degrees(node._limit_bar_fixed_reference_target_yaw()),
        -168.7,
        abs_tol=1e-9,
    )
    assert math.isclose(
        math.degrees(node._hurdle_post_align_target_yaw()),
        11.3,
        abs_tol=1e-9,
    )
    assert math.isclose(
        math.degrees(node._limit_bar_pre_duck_target_yaw()),
        -168.7,
        abs_tol=1e-9,
    )


def test_master_switch_or_missing_reference_preserves_nominal_angles():
    node = make_supervisor_stub()
    node.downstream_route_yaw_reference_enabled = False

    assert math.isclose(node._hurdle_fixed_reference_target_yaw(), 0.0)
    assert math.isclose(node._upstairs_fixed_reference_target_yaw(), 0.0)
    assert math.isclose(node._limit_bar_fixed_reference_target_yaw(), math.pi)
    assert math.isclose(node._limit_bar_pre_duck_target_yaw(), math.pi)

    node.downstream_route_yaw_reference_enabled = True
    node.slope_route_yaw_reference = None

    assert math.isclose(node._hurdle_fixed_reference_target_yaw(), 0.0)
    assert math.isclose(node._upstairs_fixed_reference_target_yaw(), 0.0)
    assert math.isclose(node._limit_bar_fixed_reference_target_yaw(), math.pi)
    assert math.isclose(node._limit_bar_pre_duck_target_yaw(), math.pi)


def test_per_branch_switches_preserve_that_branch_nominal_angle():
    node = make_supervisor_stub()
    node.downstream_route_yaw_hurdle_enabled = False
    node.downstream_route_yaw_upstairs_enabled = False
    node.downstream_route_yaw_limit_bar_pre_duck_enabled = False

    assert math.isclose(node._hurdle_fixed_reference_target_yaw(), 0.0)
    assert math.isclose(node._upstairs_fixed_reference_target_yaw(), 0.0)
    assert math.isclose(node._limit_bar_fixed_reference_target_yaw(), math.pi)
    assert math.isclose(node._hurdle_post_align_target_yaw(), 0.0)
    assert math.isclose(node._limit_bar_pre_duck_target_yaw(), math.pi)


def test_limit_bar_base_yaw_mode_ignores_global_route_offset():
    node = make_supervisor_stub()
    node.limit_bar_pre_duck_yaw_reference_mode = 'base_yaw'

    assert math.isclose(
        math.degrees(node._limit_bar_pre_duck_target_yaw()),
        160.0,
        abs_tol=1e-9,
    )
    assert node._limit_bar_pre_duck_yaw_source() == 'base_yaw'
