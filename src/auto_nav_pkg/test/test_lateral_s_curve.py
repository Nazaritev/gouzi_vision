import math

from auto_nav_pkg.lateral_s_curve import build_lateral_s_curve_plan


def make_plan(*, forward_m=1.029, lateral_m=-0.084):
    return build_lateral_s_curve_plan(
        forward_m=forward_m,
        lateral_m=lateral_m,
        min_forward_m=0.35,
        max_abs_lateral_m=0.45,
        max_abs_heading_rad=math.radians(25.0),
        max_abs_curvature_m_inv=2.5,
    )


def test_hurdle_shadow_plan_has_smooth_route_frame_endpoints():
    plan = make_plan()

    assert plan.feasible
    assert plan.reason == 'ok'
    assert plan.path_length_m > plan.forward_m
    assert plan.path_length_m < plan.forward_m + 0.02

    start = plan.sample(0.0)
    end = plan.sample(plan.forward_m)
    assert start.lateral_m == 0.0
    assert start.heading_rad == 0.0
    assert start.curvature_m_inv == 0.0
    assert math.isclose(end.lateral_m, plan.lateral_m, abs_tol=1e-12)
    assert math.isclose(end.heading_rad, 0.0, abs_tol=1e-12)
    assert math.isclose(end.curvature_m_inv, 0.0, abs_tol=1e-12)


def test_lateral_sign_changes_heading_direction_without_changing_limits():
    right_plan = make_plan(lateral_m=-0.084)
    left_plan = make_plan(lateral_m=0.084)

    right_midpoint = right_plan.sample(right_plan.forward_m * 0.5)
    left_midpoint = left_plan.sample(left_plan.forward_m * 0.5)
    assert right_midpoint.heading_rad < 0.0
    assert left_midpoint.heading_rad > 0.0
    assert math.isclose(
        right_plan.max_abs_heading_rad,
        left_plan.max_abs_heading_rad,
        abs_tol=1e-12,
    )


def test_plan_rejects_route_that_exceeds_heading_limit():
    plan = make_plan(forward_m=0.40, lateral_m=0.20)

    assert not plan.feasible
    assert plan.reason == 'heading_above_limit'
    assert plan.max_abs_heading_rad > math.radians(25.0)


def test_plan_rejects_insufficient_forward_distance_before_division():
    plan = make_plan(forward_m=0.10, lateral_m=0.02)

    assert not plan.feasible
    assert plan.reason == 'forward_below_minimum'
