import math

import numpy as np

from auto_nav_pkg.lidar_slalom_core import (
    SlalomFrame,
    build_slalom_path,
    detect_entry_p4,
    differential_tracking_command,
    final_turn_override,
    oriented_rectangle_pole_clearance,
    path_rectangle_pole_clearance,
    relative_turn_target_yaw,
    replace_path_tail_with_pose_curve,
)


def test_entry_p4_detector_prefers_centered_vertical_cluster():
    z_values = np.linspace(-0.30, 0.28, 18)
    pole = np.column_stack(
        (
            np.full_like(z_values, 0.02),
            np.full_like(z_values, 0.80),
            z_values,
        )
    )
    distractor = np.column_stack(
        (
            np.full_like(z_values, -0.26),
            np.full_like(z_values, 0.55),
            z_values,
        )
    )
    floor = np.column_stack(
        (
            np.linspace(-0.30, 0.30, 40),
            np.linspace(0.40, 1.10, 40),
            np.full(40, -0.31),
        )
    )

    candidate = detect_entry_p4(
        np.vstack((pole, distractor, floor)),
        (0.0, 0.0, 0.0),
        (0.0, 1.0),
        forward_min_m=0.45,
        forward_max_m=1.30,
        lateral_max_m=0.35,
        z_min_m=-0.42,
        z_max_m=0.42,
        cell_size_m=0.04,
        min_points=5,
        min_z_span_m=0.18,
    )

    assert candidate is not None
    assert math.isclose(candidate.x_m, 0.02, abs_tol=0.01)
    assert math.isclose(candidate.y_m, 0.80, abs_tol=0.01)


def test_transient_frame_matches_mirrored_p4_to_p1_topology():
    frame = SlalomFrame.from_p4_and_entry_heading(
        p4_xy=(10.0, -3.0),
        entry_forward_xy=(0.0, 1.0),
        pole_spacing_m=1.0,
    )
    p4, p3, p2, p1 = frame.pole_centers_world()

    assert np.allclose(p4, (10.0, -3.0))
    assert np.allclose(p3, (10.0, -2.0))
    assert np.allclose(p2, (11.0, -2.0))
    assert np.allclose(p1, (12.0, -2.0))


def test_continuous_path_hits_rule_zones_and_preserves_body_clearance():
    frame = SlalomFrame.from_p4_and_entry_heading(
        p4_xy=(0.0, 1.0),
        entry_forward_xy=(0.0, 1.0),
        pole_spacing_m=1.0,
    )
    path = build_slalom_path(frame, start_world_xy=(0.0, 0.0))
    poles = frame.pole_centers_world()
    zones = frame.local_to_world(((0.40, 1.40), (-0.30, -0.30), (2.40, 0.40)))

    path_clearance = np.min(
        np.linalg.norm(path[:, None, :] - poles[None, :, :], axis=2)
    )
    assert path_clearance >= 0.40
    for zone in zones:
        assert np.min(np.linalg.norm(path - zone[None, :], axis=1)) <= 0.03


def test_tracking_command_uses_expected_right_turn_sign():
    left, right, yaw_error = differential_tracking_command(
        pose_xy=(0.0, 0.0),
        pose_yaw_rad=0.0,
        target_xy=(0.20, 1.0),
        cruise_norm=0.42,
        minimum_norm=0.18,
        heading_gain_norm_per_rad=0.70,
        max_delta_norm=0.28,
        slowdown_heading_rad=math.radians(35.0),
    )

    assert yaw_error < 0.0
    assert left > right


def test_tracking_command_can_lock_absolute_yaw_while_translating():
    left, right, yaw_error = differential_tracking_command(
        pose_xy=(0.0, 0.0),
        pose_yaw_rad=math.radians(-90.0),
        target_xy=(1.0, 0.0),
        cruise_norm=0.42,
        minimum_norm=0.18,
        heading_gain_norm_per_rad=0.70,
        max_delta_norm=0.28,
        slowdown_heading_rad=math.radians(35.0),
        target_pose_yaw_rad=0.0,
    )

    assert math.isclose(yaw_error, math.radians(90.0), abs_tol=1e-12)
    assert left < right


def test_corrected_robot_footprint_uses_oriented_rectangle_clearance():
    clearance = oriented_rectangle_pole_clearance(
        pose_xy=(0.0, 0.0),
        forward_unit_xy=(0.0, 1.0),
        pole_centers_xy=((0.40, 0.0),),
        robot_length_m=0.66,
        robot_width_m=0.46,
        pole_radius_m=0.025,
    )

    assert math.isclose(clearance, 0.145, abs_tol=1e-12)


def test_slalom_path_has_margin_for_66_by_46_cm_footprint():
    frame = SlalomFrame.from_p4_and_entry_heading(
        p4_xy=(0.0, 1.0),
        entry_forward_xy=(0.0, 1.0),
        pole_spacing_m=1.0,
    )
    path = build_slalom_path(frame, start_world_xy=(0.0, 0.0))

    clearance = path_rectangle_pole_clearance(
        path,
        frame.pole_centers_world(),
        robot_length_m=0.66,
        robot_width_m=0.46,
        pole_radius_m=0.025,
    )

    assert clearance >= 0.10


def test_terminal_pose_curve_ends_at_zone_three_with_absolute_zero_yaw():
    frame = SlalomFrame.from_p4_and_entry_heading(
        p4_xy=(1.0, 0.0),
        entry_forward_xy=(-1.0, 0.0),
        pole_spacing_m=1.0,
    )
    base_path = build_slalom_path(frame, start_world_xy=(1.95, 0.0))
    anchor = frame.local_to_world(((2.10, -0.45),))[0]
    zone_three = frame.local_to_world(((2.40, 0.40),))[0]

    path, curve_start = replace_path_tail_with_pose_curve(
        base_path,
        anchor,
        zone_three,
        final_forward_world_xy=(0.0, 1.0),
        start_handle_m=0.60,
        final_handle_m=0.10,
        sample_spacing_m=0.005,
    )

    endpoint_forward = path[-1] - path[-2]
    endpoint_forward /= np.linalg.norm(endpoint_forward)
    clearance = path_rectangle_pole_clearance(
        path,
        frame.pole_centers_world(),
        robot_length_m=0.66,
        robot_width_m=0.46,
        pole_radius_m=0.025,
    )

    assert curve_start < len(path) - 1
    assert np.allclose(path[-1], zone_three)
    endpoint_error_deg = math.degrees(
        math.acos(np.clip(endpoint_forward @ (0.0, 1.0), -1.0, 1.0))
    )
    assert endpoint_error_deg < 8.0
    assert clearance >= 0.08


def test_p3_p2_gate_keeps_margin_with_large_heading_error():
    frame = SlalomFrame.from_p4_and_entry_heading(
        p4_xy=(0.0, 1.0),
        entry_forward_xy=(0.0, 1.0),
        pole_spacing_m=1.0,
    )
    path = build_slalom_path(frame, start_world_xy=(0.0, 0.0))
    poles = frame.pole_centers_world()
    local_path = frame.world_to_local(path)
    tangents = np.gradient(path, axis=0)
    tangents /= np.linalg.norm(tangents, axis=1)[:, None]
    gate_mask = (
        (local_path[:, 0] >= 0.20)
        & (local_path[:, 0] <= 0.80)
        & (local_path[:, 1] >= -0.55)
        & (local_path[:, 1] <= 0.55)
    )
    clearances = []
    for point, tangent in zip(path[gate_mask], tangents[gate_mask]):
        for heading_error_deg in (-40.0, 40.0):
            angle = math.radians(heading_error_deg)
            rotation = np.array(
                ((math.cos(angle), -math.sin(angle)),
                 (math.sin(angle), math.cos(angle)))
            )
            clearances.append(
                oriented_rectangle_pole_clearance(
                    point,
                    rotation @ tangent,
                    poles,
                    robot_length_m=0.66,
                    robot_width_m=0.46,
                    pole_radius_m=0.025,
                )
            )

    assert min(clearances) >= 0.055


def test_final_turn_is_ccw_90_degrees_from_turn_entry_yaw():
    target_pose_yaw = relative_turn_target_yaw(
        current_pose_yaw=math.radians(150.0),
        turn_delta_rad=math.radians(90.0),
    )
    left, right = final_turn_override(
        math.radians(90.0),
        fine_error_rad=math.radians(12.0),
        ccw_left_norm=-0.20,
        ccw_right_norm=0.65,
        cw_left_norm=0.65,
        cw_right_norm=-0.20,
        fine_left_norm=-0.10,
        fine_right_norm=0.35,
    )

    assert math.isclose(target_pose_yaw, math.radians(-120.0), abs_tol=1e-12)
    assert left < 0.0 < right
