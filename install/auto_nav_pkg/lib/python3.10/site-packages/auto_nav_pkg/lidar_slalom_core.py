#!/usr/bin/env python3
"""Coordinate-free pole-array geometry, detection, and path tracking helpers."""

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np


DEFAULT_TEMPLATE_POINTS_RL = (
    (0.40, 1.40),  # P4-side mandatory-zone center
    (0.45, 1.00),
    (0.34, 0.62),
    (-0.22, 0.38),
    (-0.42, 0.08),
    (-0.30, -0.30),  # outside-corner mandatory-zone center
    (0.10, -0.45),
    (0.42, -0.35),
    (0.49, -0.15),
    (0.50, 0.12),
    (0.58, 0.35),
    (0.80, 0.45),
    (1.12, 0.45),
    (1.45, 0.20),
    (1.50, 0.00),
    (1.75, -0.35),
    (2.10, -0.45),
    (2.42, -0.18),
    (2.43, 0.12),
    (2.40, 0.40),  # P1-side mandatory-zone center
    (2.70, 0.42),
)


@dataclass(frozen=True)
class PoleCandidate:
    x_m: float
    y_m: float
    point_count: int
    z_span_m: float

    @property
    def xy(self) -> np.ndarray:
        return np.array((self.x_m, self.y_m), dtype=float)


@dataclass(frozen=True)
class SlalomFrame:
    """Transient obstacle frame: +row is P3->P2 and +leg is P3->P4."""

    p3_xy: np.ndarray
    row_unit: np.ndarray
    leg_unit: np.ndarray
    pole_spacing_m: float

    @classmethod
    def from_p4_and_entry_heading(
        cls,
        p4_xy: Sequence[float],
        entry_forward_xy: Sequence[float],
        pole_spacing_m: float,
    ) -> 'SlalomFrame':
        forward = _unit(entry_forward_xy)
        spacing = max(0.5, float(pole_spacing_m))
        p4 = np.asarray(p4_xy, dtype=float)
        p3 = p4 + spacing * forward
        row = np.array((forward[1], -forward[0]), dtype=float)
        return cls(
            p3_xy=p3,
            row_unit=row,
            leg_unit=-forward,
            pole_spacing_m=spacing,
        )

    def local_to_world(self, points_rl: Sequence[Sequence[float]]) -> np.ndarray:
        points = np.asarray(points_rl, dtype=float).reshape((-1, 2))
        return (
            self.p3_xy[None, :]
            + points[:, 0:1] * self.row_unit[None, :]
            + points[:, 1:2] * self.leg_unit[None, :]
        )

    def world_to_local(self, points_xy: Sequence[Sequence[float]]) -> np.ndarray:
        points = np.asarray(points_xy, dtype=float).reshape((-1, 2))
        delta = points - self.p3_xy[None, :]
        return np.column_stack((delta @ self.row_unit, delta @ self.leg_unit))

    def pole_centers_world(self) -> np.ndarray:
        spacing = self.pole_spacing_m
        return self.local_to_world(
            ((0.0, spacing), (0.0, 0.0), (spacing, 0.0), (2.0 * spacing, 0.0))
        )


@dataclass(frozen=True)
class PathTrackingTarget:
    progress_index: int
    target_index: int
    target_xy: np.ndarray
    cross_track_m: float
    remaining_m: float


def flat_pairs(values: Iterable[float]) -> list[tuple[float, float]]:
    flat = [float(value) for value in values]
    if len(flat) < 4 or len(flat) % 2:
        raise ValueError('point list must contain at least two x/y pairs')
    if not all(math.isfinite(value) for value in flat):
        raise ValueError('point list contains a non-finite value')
    return list(zip(flat[0::2], flat[1::2]))


def quaternion_rotation_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm <= 1e-12:
        return np.eye(3, dtype=float)
    scale = 2.0 / norm
    return np.array(
        (
            (1.0 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)),
            (scale * (x * y + z * w), 1.0 - scale * (x * x + z * z), scale * (y * z - x * w)),
            (scale * (x * z - y * w), scale * (y * z + x * w), 1.0 - scale * (x * x + y * y)),
        ),
        dtype=float,
    )


def detect_entry_p4(
    points_xyz: np.ndarray,
    pose_xyz: Sequence[float],
    entry_forward_xy: Sequence[float],
    *,
    forward_min_m: float,
    forward_max_m: float,
    lateral_max_m: float,
    z_min_m: float,
    z_max_m: float,
    cell_size_m: float,
    min_points: int,
    min_z_span_m: float,
) -> PoleCandidate | None:
    points = _finite_xyz(points_xyz)
    if points.size == 0:
        return None
    pose = np.asarray(pose_xyz, dtype=float)
    forward = _unit(entry_forward_xy)
    left = np.array((-forward[1], forward[0]), dtype=float)
    delta = points[:, :2] - pose[None, :2]
    along = delta @ forward
    lateral = delta @ left
    relative_z = points[:, 2] - pose[2]
    mask = (
        (along >= forward_min_m)
        & (along <= forward_max_m)
        & (np.abs(lateral) <= lateral_max_m)
        & (relative_z >= z_min_m)
        & (relative_z <= z_max_m)
    )
    candidates = vertical_cell_candidates(
        points[mask],
        cell_size_m=cell_size_m,
        min_points=min_points,
        min_z_span_m=min_z_span_m,
    )
    if not candidates:
        return None

    def score(candidate: PoleCandidate) -> tuple[float, float, int]:
        offset = candidate.xy - pose[:2]
        candidate_along = float(offset @ forward)
        candidate_lateral = float(offset @ left)
        return (
            abs(candidate_lateral),
            abs(candidate_along - 0.80),
            -candidate.point_count,
        )

    return min(candidates, key=score)


def detect_expected_pole(
    points_xyz: np.ndarray,
    expected_xy: Sequence[float],
    pose_z_m: float,
    *,
    search_radius_m: float,
    z_min_m: float,
    z_max_m: float,
    cell_size_m: float,
    min_points: int,
    min_z_span_m: float,
) -> PoleCandidate | None:
    points = _finite_xyz(points_xyz)
    if points.size == 0:
        return None
    expected = np.asarray(expected_xy, dtype=float)
    distance = np.linalg.norm(points[:, :2] - expected[None, :], axis=1)
    relative_z = points[:, 2] - float(pose_z_m)
    mask = (
        (distance <= search_radius_m)
        & (relative_z >= z_min_m)
        & (relative_z <= z_max_m)
    )
    candidates = vertical_cell_candidates(
        points[mask],
        cell_size_m=cell_size_m,
        min_points=min_points,
        min_z_span_m=min_z_span_m,
    )
    if not candidates:
        return None
    return min(candidates, key=lambda candidate: float(np.linalg.norm(candidate.xy - expected)))


def vertical_cell_candidates(
    points_xyz: np.ndarray,
    *,
    cell_size_m: float,
    min_points: int,
    min_z_span_m: float,
) -> list[PoleCandidate]:
    points = _finite_xyz(points_xyz)
    if points.shape[0] < min_points:
        return []
    cell_size = max(0.01, float(cell_size_m))
    keys = np.floor(points[:, :2] / cell_size).astype(np.int64)
    order = np.lexsort((keys[:, 1], keys[:, 0]))
    sorted_keys = keys[order]
    changes = np.any(np.diff(sorted_keys, axis=0) != 0, axis=1)
    starts = np.r_[0, np.flatnonzero(changes) + 1]
    ends = np.r_[starts[1:], len(order)]
    candidates: list[PoleCandidate] = []
    for start, end in zip(starts, ends):
        count = int(end - start)
        if count < min_points:
            continue
        group = points[order[start:end]]
        z_span = float(np.ptp(group[:, 2]))
        if z_span < min_z_span_m:
            continue
        center = np.median(group[:, :2], axis=0)
        candidates.append(
            PoleCandidate(
                x_m=float(center[0]),
                y_m=float(center[1]),
                point_count=count,
                z_span_m=z_span,
            )
        )
    return candidates


def build_slalom_path(
    frame: SlalomFrame,
    start_world_xy: Sequence[float],
    template_points_rl: Sequence[Sequence[float]] = DEFAULT_TEMPLATE_POINTS_RL,
    *,
    sample_spacing_m: float = 0.03,
) -> np.ndarray:
    scale = frame.pole_spacing_m
    template = np.asarray(template_points_rl, dtype=float).reshape((-1, 2)) * scale
    start_local = frame.world_to_local((start_world_xy,))[0]
    controls = np.vstack((start_local, template))
    local_path = catmull_rom_path(controls, sample_spacing_m=sample_spacing_m)
    return frame.local_to_world(local_path)


def replace_path_tail_with_pose_curve(
    path_xy: np.ndarray,
    anchor_world_xy: Sequence[float],
    goal_world_xy: Sequence[float],
    final_forward_world_xy: Sequence[float],
    *,
    start_handle_m: float,
    final_handle_m: float,
    sample_spacing_m: float,
    tangent_back_points: int = 3,
) -> tuple[np.ndarray, int]:
    """Replace a path tail with a cubic curve ending at an exact body pose."""

    path = np.asarray(path_xy, dtype=float).reshape((-1, 2))
    if path.shape[0] < 4:
        raise ValueError('path must contain at least four points')
    anchor = np.asarray(anchor_world_xy, dtype=float).reshape(2)
    goal = np.asarray(goal_world_xy, dtype=float).reshape(2)
    final_forward = _unit(final_forward_world_xy)
    anchor_index = int(np.argmin(np.linalg.norm(path - anchor[None, :], axis=1)))
    if anchor_index <= 0 or anchor_index >= len(path) - 1:
        raise ValueError('terminal curve anchor must be inside the path')

    back_index = max(0, anchor_index - max(1, int(tangent_back_points)))
    start_forward = _unit(path[anchor_index] - path[back_index])
    p0 = path[anchor_index]
    p1 = p0 + start_forward * max(0.02, float(start_handle_m))
    p3 = goal
    p2 = p3 - final_forward * max(0.02, float(final_handle_m))

    spacing = max(0.005, float(sample_spacing_m))
    control_length = (
        np.linalg.norm(p1 - p0)
        + np.linalg.norm(p2 - p1)
        + np.linalg.norm(p3 - p2)
    )
    dense_count = max(80, int(math.ceil(control_length / spacing)) * 8)
    ratio = np.linspace(0.0, 1.0, dense_count)
    inverse = 1.0 - ratio
    dense = (
        inverse[:, None] ** 3 * p0
        + 3.0 * inverse[:, None] ** 2 * ratio[:, None] * p1
        + 3.0 * inverse[:, None] * ratio[:, None] ** 2 * p2
        + ratio[:, None] ** 3 * p3
    )
    arc = np.r_[
        0.0,
        np.cumsum(np.linalg.norm(np.diff(dense, axis=0), axis=1)),
    ]
    samples = np.r_[np.arange(0.0, arc[-1], spacing), arc[-1]]
    curve = np.column_stack(
        (
            np.interp(samples, arc, dense[:, 0]),
            np.interp(samples, arc, dense[:, 1]),
        )
    )
    curve[-1] = goal
    return np.vstack((path[:anchor_index], curve)), anchor_index


def catmull_rom_path(
    control_points: Sequence[Sequence[float]],
    *,
    sample_spacing_m: float,
) -> np.ndarray:
    controls = np.asarray(control_points, dtype=float).reshape((-1, 2))
    if controls.shape[0] < 2:
        raise ValueError('at least two control points are required')
    spacing = max(0.005, float(sample_spacing_m))
    extended = np.vstack((controls[0], controls, controls[-1]))
    dense: list[np.ndarray] = []
    for index in range(1, len(extended) - 2):
        p0, p1, p2, p3 = extended[index - 1:index + 3]
        sample_count = max(6, int(math.ceil(np.linalg.norm(p2 - p1) / spacing)) * 3)
        for ratio in np.linspace(0.0, 1.0, sample_count, endpoint=False):
            ratio2 = ratio * ratio
            ratio3 = ratio2 * ratio
            dense.append(
                0.5
                * (
                    2.0 * p1
                    + (-p0 + p2) * ratio
                    + (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * ratio2
                    + (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * ratio3
                )
            )
    dense.append(controls[-1])
    dense_points = np.asarray(dense, dtype=float)
    segment_lengths = np.linalg.norm(np.diff(dense_points, axis=0), axis=1)
    keep = np.r_[True, segment_lengths > 1e-8]
    dense_points = dense_points[keep]
    arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(dense_points, axis=0), axis=1))]
    samples = np.arange(0.0, arc[-1], spacing)
    samples = np.r_[samples, arc[-1]]
    return np.column_stack(
        (
            np.interp(samples, arc, dense_points[:, 0]),
            np.interp(samples, arc, dense_points[:, 1]),
        )
    )


def path_tracking_target(
    path_xy: np.ndarray,
    pose_xy: Sequence[float],
    current_index: int,
    *,
    lookahead_m: float,
    search_back_points: int = 10,
    search_forward_points: int = 100,
) -> PathTrackingTarget:
    path = np.asarray(path_xy, dtype=float).reshape((-1, 2))
    pose = np.asarray(pose_xy, dtype=float)
    if path.shape[0] < 2:
        raise ValueError('path must contain at least two points')
    current = max(0, min(int(current_index), len(path) - 1))
    start = max(0, current - max(0, int(search_back_points)))
    end = min(len(path), current + max(2, int(search_forward_points)))
    distances = np.linalg.norm(path[start:end] - pose[None, :], axis=1)
    nearest = start + int(np.argmin(distances))
    progress = max(current, nearest)
    target = progress
    distance_ahead = 0.0
    requested_lookahead = max(0.0, float(lookahead_m))
    while target + 1 < len(path) and distance_ahead < requested_lookahead:
        distance_ahead += float(np.linalg.norm(path[target + 1] - path[target]))
        target += 1
    remaining = float(
        np.sum(np.linalg.norm(np.diff(path[progress:], axis=0), axis=1))
    )
    return PathTrackingTarget(
        progress_index=progress,
        target_index=target,
        target_xy=path[target].copy(),
        cross_track_m=float(np.linalg.norm(path[nearest] - pose)),
        remaining_m=remaining,
    )


def differential_tracking_command(
    pose_xy: Sequence[float],
    pose_yaw_rad: float,
    target_xy: Sequence[float],
    *,
    cruise_norm: float,
    minimum_norm: float,
    heading_gain_norm_per_rad: float,
    max_delta_norm: float,
    slowdown_heading_rad: float,
    target_pose_yaw_rad: float | None = None,
) -> tuple[float, float, float]:
    pose = np.asarray(pose_xy, dtype=float)
    target = np.asarray(target_xy, dtype=float)
    delta_xy = target - pose
    if target_pose_yaw_rad is None:
        target_heading = math.atan2(float(delta_xy[1]), float(delta_xy[0]))
        target_pose_yaw = target_heading - math.pi / 2.0
    else:
        # Keep translating with the same differential command, but let the
        # caller own the terminal body attitude. This is used by the slalom
        # exit curve so reaching global yaw zero does not require a later
        # in-place turn.
        target_pose_yaw = float(target_pose_yaw_rad)
    yaw_error = normalize_angle(target_pose_yaw - float(pose_yaw_rad))
    cruise = max(0.0, min(1.0, float(cruise_norm)))
    minimum = max(0.0, min(cruise, float(minimum_norm)))
    slowdown = max(math.radians(5.0), float(slowdown_heading_rad))
    heading_scale = min(1.0, abs(yaw_error) / slowdown)
    base = cruise - (cruise - minimum) * 0.65 * heading_scale
    correction = max(
        -max_delta_norm,
        min(max_delta_norm, heading_gain_norm_per_rad * yaw_error),
    )
    left = max(-1.0, min(1.0, base - correction))
    right = max(-1.0, min(1.0, base + correction))
    return left, right, yaw_error


def oriented_rectangle_pole_clearance(
    pose_xy: Sequence[float],
    forward_unit_xy: Sequence[float],
    pole_centers_xy: Sequence[Sequence[float]],
    *,
    robot_length_m: float,
    robot_width_m: float,
    pole_radius_m: float,
) -> float:
    """Return minimum pole-surface distance from an oriented robot rectangle."""

    pose = np.asarray(pose_xy, dtype=float).reshape(2)
    forward = _unit(forward_unit_xy)
    left = np.array((-forward[1], forward[0]), dtype=float)
    centers = np.asarray(pole_centers_xy, dtype=float).reshape((-1, 2))
    delta = centers - pose[None, :]
    along = np.abs(delta @ forward)
    lateral = np.abs(delta @ left)
    outside_along = np.maximum(along - 0.5 * float(robot_length_m), 0.0)
    outside_lateral = np.maximum(lateral - 0.5 * float(robot_width_m), 0.0)
    rectangle_distance = np.hypot(outside_along, outside_lateral)
    return float(np.min(rectangle_distance) - max(0.0, float(pole_radius_m)))


def path_rectangle_pole_clearance(
    path_xy: np.ndarray,
    pole_centers_xy: Sequence[Sequence[float]],
    *,
    robot_length_m: float,
    robot_width_m: float,
    pole_radius_m: float,
) -> float:
    """Evaluate oriented-footprint clearance using the path tangent as heading."""

    path = np.asarray(path_xy, dtype=float).reshape((-1, 2))
    if path.shape[0] < 2:
        raise ValueError('path must contain at least two points')
    tangents = np.gradient(path, axis=0)
    tangent_norms = np.linalg.norm(tangents, axis=1)
    valid = tangent_norms > 1e-9
    tangents[valid] /= tangent_norms[valid, None]
    tangents[~valid] = (1.0, 0.0)
    centers = np.asarray(pole_centers_xy, dtype=float).reshape((-1, 2))
    left = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    delta = centers[None, :, :] - path[:, None, :]
    along = np.abs(np.sum(delta * tangents[:, None, :], axis=2))
    lateral = np.abs(np.sum(delta * left[:, None, :], axis=2))
    outside_along = np.maximum(along - 0.5 * float(robot_length_m), 0.0)
    outside_lateral = np.maximum(lateral - 0.5 * float(robot_width_m), 0.0)
    rectangle_distance = np.hypot(outside_along, outside_lateral)
    return float(np.min(rectangle_distance) - max(0.0, float(pole_radius_m)))


def relative_turn_target_yaw(
    current_pose_yaw: float,
    turn_delta_rad: float,
) -> float:
    """Resolve a body-relative turn target from the yaw at turn entry."""

    return normalize_angle(float(current_pose_yaw) + float(turn_delta_rad))


def final_turn_override(
    yaw_error_rad: float,
    *,
    fine_error_rad: float,
    ccw_left_norm: float,
    ccw_right_norm: float,
    cw_left_norm: float,
    cw_right_norm: float,
    fine_left_norm: float,
    fine_right_norm: float,
) -> tuple[float, float]:
    """Resolve coarse/fine differential commands for the final yaw closure."""

    yaw_error = float(yaw_error_rad)
    if abs(yaw_error) <= max(0.0, float(fine_error_rad)):
        if yaw_error >= 0.0:
            return float(fine_left_norm), float(fine_right_norm)
        return float(fine_right_norm), float(fine_left_norm)
    if yaw_error > 0.0:
        return float(ccw_left_norm), float(ccw_right_norm)
    return float(cw_left_norm), float(cw_right_norm)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _finite_xyz(points_xyz: np.ndarray) -> np.ndarray:
    points = np.asarray(points_xyz, dtype=float).reshape((-1, 3))
    return points[np.isfinite(points).all(axis=1)]


def _unit(vector_xy: Sequence[float]) -> np.ndarray:
    vector = np.asarray(vector_xy, dtype=float).reshape(2)
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError('direction vector must be finite and non-zero')
    return vector / norm
