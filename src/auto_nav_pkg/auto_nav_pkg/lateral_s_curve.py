#!/usr/bin/env python3
"""Quintic route-frame S-curve planning for lateral obstacle alignment."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class LateralSCurveSample:
    forward_m: float
    lateral_m: float
    heading_rad: float
    curvature_m_inv: float


@dataclass(frozen=True)
class LateralSCurvePlan:
    forward_m: float
    lateral_m: float
    path_length_m: float
    max_abs_heading_rad: float
    max_abs_curvature_m_inv: float
    feasible: bool
    reason: str

    def sample(self, forward_m: float) -> LateralSCurveSample:
        if self.forward_m <= 1e-9:
            return LateralSCurveSample(0.0, 0.0, 0.0, 0.0)

        clamped_forward_m = max(0.0, min(self.forward_m, float(forward_m)))
        progress = clamped_forward_m / self.forward_m
        shape, first, second = _quintic_shape(progress)
        slope = self.lateral_m * first / self.forward_m
        second_derivative = (
            self.lateral_m * second / (self.forward_m * self.forward_m)
        )
        curvature = second_derivative / ((1.0 + slope * slope) ** 1.5)
        return LateralSCurveSample(
            forward_m=clamped_forward_m,
            lateral_m=self.lateral_m * shape,
            heading_rad=math.atan(slope),
            curvature_m_inv=curvature,
        )


def build_lateral_s_curve_plan(
    *,
    forward_m: float,
    lateral_m: float,
    min_forward_m: float,
    max_abs_lateral_m: float,
    max_abs_heading_rad: float,
    max_abs_curvature_m_inv: float,
    integration_steps: int = 200,
) -> LateralSCurvePlan:
    """Build a zero-slope, zero-curvature endpoint route in the base-yaw frame."""

    forward_m = float(forward_m)
    lateral_m = float(lateral_m)
    if not math.isfinite(forward_m) or not math.isfinite(lateral_m):
        return _invalid_plan(forward_m, lateral_m, 'non_finite_input')
    if forward_m < max(0.0, float(min_forward_m)):
        return _invalid_plan(forward_m, lateral_m, 'forward_below_minimum')
    if abs(lateral_m) > max(0.0, float(max_abs_lateral_m)):
        return _invalid_plan(forward_m, lateral_m, 'lateral_above_limit')

    integration_steps = max(20, int(integration_steps))
    step_m = forward_m / integration_steps
    path_length_m = 0.0
    max_heading = 0.0
    max_curvature = 0.0
    previous_integrand = 1.0
    for index in range(integration_steps + 1):
        progress = index / integration_steps
        _, first, second = _quintic_shape(progress)
        slope = lateral_m * first / forward_m
        second_derivative = lateral_m * second / (forward_m * forward_m)
        integrand = math.sqrt(1.0 + slope * slope)
        if index > 0:
            path_length_m += 0.5 * (previous_integrand + integrand) * step_m
        previous_integrand = integrand
        max_heading = max(max_heading, abs(math.atan(slope)))
        curvature = abs(second_derivative) / ((1.0 + slope * slope) ** 1.5)
        max_curvature = max(max_curvature, curvature)

    reason = 'ok'
    feasible = True
    if max_abs_heading_rad > 0.0 and max_heading > max_abs_heading_rad:
        feasible = False
        reason = 'heading_above_limit'
    elif (
        max_abs_curvature_m_inv > 0.0
        and max_curvature > max_abs_curvature_m_inv
    ):
        feasible = False
        reason = 'curvature_above_limit'

    return LateralSCurvePlan(
        forward_m=forward_m,
        lateral_m=lateral_m,
        path_length_m=path_length_m,
        max_abs_heading_rad=max_heading,
        max_abs_curvature_m_inv=max_curvature,
        feasible=feasible,
        reason=reason,
    )


def _invalid_plan(
    forward_m: float,
    lateral_m: float,
    reason: str,
) -> LateralSCurvePlan:
    return LateralSCurvePlan(
        forward_m=forward_m,
        lateral_m=lateral_m,
        path_length_m=0.0,
        max_abs_heading_rad=0.0,
        max_abs_curvature_m_inv=0.0,
        feasible=False,
        reason=reason,
    )


def _quintic_shape(progress: float) -> tuple[float, float, float]:
    progress = max(0.0, min(1.0, float(progress)))
    progress2 = progress * progress
    progress3 = progress2 * progress
    progress4 = progress3 * progress
    progress5 = progress4 * progress
    shape = 10.0 * progress3 - 15.0 * progress4 + 6.0 * progress5
    first = 30.0 * progress2 - 60.0 * progress3 + 30.0 * progress4
    second = 60.0 * progress - 180.0 * progress2 + 120.0 * progress3
    return shape, first, second
