import math

from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

from auto_nav_pkg.obstacle_manager import ObstacleManager


def make_manager_stub():
    node = ObstacleManager.__new__(ObstacleManager)
    node.current_idx = 0
    node.waypoints = [{'name': 'bridge_entry'}]
    node.apriltag_base_tf_topic = '/apriltag/tag_tf'
    node.apriltag_base_tf_frame_id = 'camera_color_optical_frame'
    node.apriltag_base_tf_child_frame_id = 'base'
    node.apriltag_compensation_tf_keys = {
        ('camera_color_optical_frame', 'bridge'),
        ('camera_color_optical_frame', 'base'),
    }
    node.apriltag_compensation_tf_cache = {}
    node.apriltag_z_m = None
    node.apriltag_frame_id = ''
    node.apriltag_child_frame_id = ''
    node.apriltag_stamp_sec = 0.0
    node._now_sec = lambda: 10.0
    return node


def make_tf_message(child_frame_id, z_m, stamp_sec):
    transform = TransformStamped()
    transform.header.frame_id = 'camera_color_optical_frame'
    transform.child_frame_id = child_frame_id
    transform.header.stamp.sec = int(stamp_sec)
    transform.header.stamp.nanosec = round(
        (stamp_sec - int(stamp_sec)) * 1_000_000_000
    )
    transform.transform.translation.z = z_m
    transform.transform.rotation.w = 1.0
    message = TFMessage()
    message.transforms = [transform]
    return message


def test_compensation_cache_records_bridge_outside_active_arrival_goal():
    node = make_manager_stub()

    node._process_apriltag_tf_message(make_tf_message('bridge', 1.50, 9.6))
    node._process_apriltag_tf_message(
        make_tf_message('base', 0.72, 9.7),
        base_only=True,
    )

    bridge_sample = node.apriltag_compensation_tf_cache[
        ('camera_color_optical_frame', 'bridge')
    ]
    base_sample = node.apriltag_compensation_tf_cache[
        ('camera_color_optical_frame', 'base')
    ]
    assert node.apriltag_frame_id == ''
    assert node.apriltag_child_frame_id == ''
    assert math.isclose(bridge_sample[0], 1.50)
    assert math.isclose(bridge_sample[1], 9.6)
    assert math.isclose(base_sample[0], 0.72)
    assert math.isclose(base_sample[1], 9.7)


def test_bridge_compensation_uses_keyed_cache_when_active_cache_is_base():
    node = make_manager_stub()
    node.apriltag_z_m = 0.72
    node.apriltag_frame_id = 'camera_color_optical_frame'
    node.apriltag_child_frame_id = 'base'
    node.apriltag_stamp_sec = 9.8
    node.apriltag_compensation_tf_cache[
        ('camera_color_optical_frame', 'bridge')
    ] = (1.50, 9.6, 9.6)
    waypoint = {
        'body_forward_m_apriltag_z_compensation_enabled': True,
        'body_forward_m_apriltag_z_compensation_frame_id': 'camera_color_optical_frame',
        'body_forward_m_apriltag_z_compensation_child_frame_id': 'bridge',
        'body_forward_m_apriltag_z_compensation_stale_timeout_sec': 1.20,
        'body_forward_m_apriltag_z_compensation_neutral_z_m': 1.24,
        'body_forward_m_apriltag_z_compensation_near_z_m': 1.10,
        'body_forward_m_apriltag_z_compensation_near_delta_m': -0.05,
        'body_forward_m_apriltag_z_compensation_far_z_m': 1.50,
        'body_forward_m_apriltag_z_compensation_far_delta_m': 0.03,
    }

    delta_m, note = node._body_forward_m_apriltag_z_compensation(waypoint)

    assert math.isclose(delta_m, 0.03, abs_tol=1e-9)
    assert 'tag_z=1.500m' in note
    assert 'source=compensation_cache' in note


def bridge_exit_forward_waypoint():
    return {
        'body_forward_m': 2.30,
        'body_forward_m_bridge_retry_override_enabled': True,
        'body_forward_m_bridge_retry_override_m': 0.50,
        'body_forward_m_apriltag_z_compensation_enabled': True,
        'body_forward_m_apriltag_z_compensation_frame_id': 'camera_color_optical_frame',
        'body_forward_m_apriltag_z_compensation_child_frame_id': 'bridge',
        'body_forward_m_apriltag_z_compensation_stale_timeout_sec': 1.20,
        'body_forward_m_apriltag_z_compensation_neutral_z_m': 1.24,
        'body_forward_m_apriltag_z_compensation_near_z_m': 1.10,
        'body_forward_m_apriltag_z_compensation_near_delta_m': -0.05,
        'body_forward_m_apriltag_z_compensation_far_z_m': 1.50,
        'body_forward_m_apriltag_z_compensation_far_delta_m': 0.03,
    }


def test_bridge_exit_keeps_normal_distance_and_tag_compensation_without_retry():
    node = make_manager_stub()
    configure_runtime_yaw_references(node)
    node.apriltag_compensation_tf_cache[
        ('camera_color_optical_frame', 'bridge')
    ] = (1.50, 9.6, 9.6)

    forward_m, note = node._body_forward_m_for_current_pose(
        bridge_exit_forward_waypoint()
    )

    assert math.isclose(forward_m, 2.33, abs_tol=1e-9)
    assert 'apriltag_z_comp=+0.030m' in note
    assert 'bridge_retry_forward_override' not in note


def test_bridge_exit_uses_exact_half_meter_and_skips_tag_compensation_after_retry():
    node = make_manager_stub()
    configure_runtime_yaw_references(node, bridge_deg=-168.0)
    node.apriltag_compensation_tf_cache[
        ('camera_color_optical_frame', 'bridge')
    ] = (1.50, 9.6, 9.6)

    forward_m, note = node._body_forward_m_for_current_pose(
        bridge_exit_forward_waypoint()
    )

    assert math.isclose(forward_m, 0.50, abs_tol=1e-9)
    assert 'bridge_retry_forward_override=0.500m' in note
    assert 'apriltag_z_comp=skipped' in note


def test_absolute_delta_yaw_uses_global_reference_instead_of_current_pose():
    node = make_manager_stub()
    waypoint = {
        'body_yaw_mode': 'absolute_delta',
        'yaw_deg': 180.0,
        'body_yaw_delta_deg': 50.0,
    }

    target_yaw = node._body_relative_target_yaw(waypoint, math.radians(12.0))

    assert math.isclose(target_yaw, math.radians(-130.0), abs_tol=1e-9)


def test_absolute_delta_yaw_preserves_negative_increment():
    node = make_manager_stub()
    waypoint = {
        'body_yaw_mode': 'absolute_delta',
        'yaw_deg': 180.0,
        'body_yaw_delta_deg': -72.0,
    }

    target_yaw = node._body_relative_target_yaw(waypoint, math.radians(-35.0))

    assert math.isclose(target_yaw, math.radians(108.0), abs_tol=1e-9)


def configure_runtime_yaw_references(
    node,
    *,
    route_deg=None,
    bridge_deg=None,
    enabled=True,
):
    node.runtime_route_yaw_reference_enabled = enabled
    node.route_yaw_reference = (
        None if route_deg is None else math.radians(route_deg)
    )
    node.route_yaw_reference_stamp_sec = 9.0
    node.route_yaw_reference_timeout_sec = 300.0
    node.bridge_retry_yaw_reference = (
        None if bridge_deg is None else math.radians(bridge_deg)
    )
    node.bridge_retry_yaw_reference_stamp_sec = 9.0
    node.bridge_retry_yaw_reference_timeout_sec = 300.0


def test_route_absolute_yaw_adds_latched_route_zero():
    node = make_manager_stub()
    configure_runtime_yaw_references(node, route_deg=12.0)
    waypoint = {
        'body_yaw_mode': 'route_absolute',
        'route_yaw_fallback_mode': 'absolute',
        'yaw_deg': 92.0,
    }

    target_yaw = node._body_relative_target_yaw(waypoint, math.radians(-20.0))

    assert math.isclose(target_yaw, math.radians(104.0), abs_tol=1e-9)


def test_route_absolute_yaw_uses_legacy_fallback_when_feature_is_disabled():
    node = make_manager_stub()
    configure_runtime_yaw_references(node, route_deg=12.0, enabled=False)
    waypoint = {
        'body_yaw_mode': 'route_absolute',
        'route_yaw_fallback_mode': 'absolute',
        'yaw_deg': 92.0,
    }

    target_yaw = node._body_relative_target_yaw(waypoint, math.radians(-20.0))

    assert math.isclose(target_yaw, math.radians(92.0), abs_tol=1e-9)


def test_bridge_reference_delta_prefers_bridge_retry_heading():
    node = make_manager_stub()
    configure_runtime_yaw_references(node, route_deg=12.0, bridge_deg=-168.0)
    waypoint = {
        'body_yaw_mode': 'bridge_reference_delta',
        'route_yaw_fallback_mode': 'absolute_delta',
        'yaw_deg': 180.0,
        'body_yaw_delta_deg': 50.0,
    }

    target_yaw = node._body_relative_target_yaw(waypoint, math.radians(0.0))

    assert math.isclose(target_yaw, math.radians(-118.0), abs_tol=1e-9)


def test_bridge_reference_delta_uses_route_zero_without_bridge_retry():
    node = make_manager_stub()
    configure_runtime_yaw_references(node, route_deg=12.0)
    waypoint = {
        'body_yaw_mode': 'bridge_reference_delta',
        'route_yaw_fallback_mode': 'absolute_delta',
        'yaw_deg': 180.0,
        'body_yaw_delta_deg': -90.0,
    }

    target_yaw = node._body_relative_target_yaw(waypoint, math.radians(0.0))

    assert math.isclose(target_yaw, math.radians(102.0), abs_tol=1e-9)


def final_heading_waypoint():
    return {
        'name': 'final_heading_align',
        'body_yaw_mode': 'waypoint_arrival',
        'body_yaw_waypoint': 'bridge_entry',
        'body_yaw_waypoint_fallback_to_bridge_retry_reference': True,
    }


def test_waypoint_arrival_yaw_prefers_real_bridge_entry_arrival():
    node = make_manager_stub()
    configure_runtime_yaw_references(node, bridge_deg=-168.0)
    waypoint = final_heading_waypoint()
    node.current_idx = 1
    node.waypoints = [
        {
            'name': 'bridge_entry',
            '_arrival_pose': (1.0, 2.0, math.radians(177.0)),
        },
        waypoint,
    ]

    target_yaw = node._body_relative_target_yaw(waypoint, math.radians(0.0))

    assert math.isclose(target_yaw, math.radians(177.0), abs_tol=1e-9)
    assert waypoint['_body_yaw_waypoint_reference_source'] == (
        'waypoint_arrival:bridge_entry'
    )


def test_waypoint_arrival_yaw_falls_back_to_latest_bridge_retry_heading():
    node = make_manager_stub()
    configure_runtime_yaw_references(node, bridge_deg=-168.0)
    waypoint = final_heading_waypoint()
    node.current_idx = 1
    node.waypoints = [{'name': 'bridge_entry'}, waypoint]

    target_yaw = node._body_relative_target_yaw(waypoint, math.radians(0.0))

    assert math.isclose(target_yaw, math.radians(-168.0), abs_tol=1e-9)
    assert waypoint['_body_yaw_waypoint_reference_source'] == (
        'bridge_retry_yaw_reference'
    )


def test_waypoint_arrival_yaw_still_waits_without_any_reference():
    node = make_manager_stub()
    configure_runtime_yaw_references(node)
    waypoint = final_heading_waypoint()
    node.current_idx = 1
    node.waypoints = [{'name': 'bridge_entry'}, waypoint]

    target_yaw = node._body_relative_target_yaw(waypoint, math.radians(0.0))

    assert target_yaw is None
    assert '_body_yaw_waypoint_reference_source' not in waypoint
