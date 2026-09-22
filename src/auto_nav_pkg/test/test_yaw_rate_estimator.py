import math

from auto_nav_pkg.yaw_rate_estimator import WindowedYawRateEstimator


def make_estimator():
    return WindowedYawRateEstimator(
        window_sec=0.30,
        min_span_sec=0.18,
        max_abs_rate_rad_s=math.radians(90.0),
        smoothing_alpha=1.0,
    )


def wrapped_yaw(degrees):
    radians = math.radians(degrees)
    return math.atan2(math.sin(radians), math.cos(radians))


def test_windowed_rate_tracks_constant_turn_across_angle_wrap():
    estimator = make_estimator()

    for index in range(11):
        stamp_sec = index * 0.05
        yaw_deg = 170.0 + 30.0 * stamp_sec
        rate, _ = estimator.update(wrapped_yaw(yaw_deg), stamp_sec)

    assert math.isclose(math.degrees(rate), 30.0, abs_tol=0.5)


def test_short_interval_yaw_spike_does_not_become_control_rate_spike():
    estimator = make_estimator()
    for index in range(7):
        stamp_sec = index * 0.05
        estimator.update(math.radians(20.0 * stamp_sec), stamp_sec)

    filtered_rate, raw_rate = estimator.update(math.radians(11.0), 0.301)

    assert abs(math.degrees(raw_rate)) > 1000.0
    assert abs(math.degrees(filtered_rate)) < 50.0

    estimator.update(math.radians(7.0), 0.35)
    filtered_rate, _ = estimator.update(math.radians(8.0), 0.40)
    assert math.isclose(math.degrees(filtered_rate), 20.0, abs_tol=8.0)


def test_rate_waits_for_minimum_window_span():
    estimator = make_estimator()

    estimator.update(0.0, 0.0)
    rate, raw_rate = estimator.update(math.radians(1.0), 0.05)

    assert rate == 0.0
    assert math.isclose(math.degrees(raw_rate), 20.0, abs_tol=1e-9)


def test_long_pose_gap_resets_rate_history():
    estimator = make_estimator()
    for index in range(7):
        estimator.update(math.radians(index), index * 0.05)

    rate, raw_rate = estimator.update(math.radians(40.0), 1.0)

    assert rate == 0.0
    assert raw_rate == 0.0
