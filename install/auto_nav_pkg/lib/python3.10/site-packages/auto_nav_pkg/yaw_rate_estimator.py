#!/usr/bin/env python3
"""Windowed yaw-rate estimation for noisy odometry streams."""

import math
from collections import deque
from typing import Deque, Tuple


class WindowedYawRateEstimator:
    """Estimate yaw rate from an unwrapped, time-windowed linear fit."""

    def __init__(
        self,
        *,
        window_sec: float,
        min_span_sec: float,
        max_abs_rate_rad_s: float,
        smoothing_alpha: float = 1.0,
        max_gap_sec: float = 0.50,
    ) -> None:
        self.window_sec = max(1e-3, float(window_sec))
        self.min_span_sec = max(
            1e-3,
            min(float(min_span_sec), self.window_sec),
        )
        self.max_abs_rate_rad_s = max(0.0, float(max_abs_rate_rad_s))
        self.smoothing_alpha = max(0.0, min(1.0, float(smoothing_alpha)))
        self.max_gap_sec = max(self.window_sec, float(max_gap_sec))
        self.samples: Deque[Tuple[float, float]] = deque()
        self.last_wrapped_yaw: float | None = None
        self.rate_rad_s = 0.0
        self.raw_rate_rad_s = 0.0

    def reset(self) -> None:
        self.samples.clear()
        self.last_wrapped_yaw = None
        self.rate_rad_s = 0.0
        self.raw_rate_rad_s = 0.0

    def update(self, yaw_rad: float, stamp_sec: float) -> Tuple[float, float]:
        yaw_rad = float(yaw_rad)
        stamp_sec = float(stamp_sec)
        if not math.isfinite(yaw_rad) or not math.isfinite(stamp_sec):
            return self.rate_rad_s, self.raw_rate_rad_s

        if not self.samples:
            self.samples.append((stamp_sec, yaw_rad))
            self.last_wrapped_yaw = yaw_rad
            return self.rate_rad_s, self.raw_rate_rad_s

        last_stamp_sec, last_unwrapped_yaw = self.samples[-1]
        dt = stamp_sec - last_stamp_sec
        if dt <= 0.0:
            return self.rate_rad_s, self.raw_rate_rad_s
        if dt > self.max_gap_sec:
            self.reset()
            self.samples.append((stamp_sec, yaw_rad))
            self.last_wrapped_yaw = yaw_rad
            return self.rate_rad_s, self.raw_rate_rad_s

        last_wrapped_yaw = (
            yaw_rad if self.last_wrapped_yaw is None else self.last_wrapped_yaw
        )
        yaw_delta = self._normalize_angle(yaw_rad - last_wrapped_yaw)
        self.raw_rate_rad_s = yaw_delta / dt
        unwrapped_yaw = last_unwrapped_yaw + yaw_delta
        self.samples.append((stamp_sec, unwrapped_yaw))
        self.last_wrapped_yaw = yaw_rad

        cutoff_sec = stamp_sec - self.window_sec
        while len(self.samples) > 2 and self.samples[1][0] <= cutoff_sec:
            self.samples.popleft()

        span_sec = self.samples[-1][0] - self.samples[0][0]
        if span_sec < self.min_span_sec:
            self.rate_rad_s = 0.0
            return self.rate_rad_s, self.raw_rate_rad_s

        mean_stamp = sum(sample[0] for sample in self.samples) / len(self.samples)
        mean_yaw = sum(sample[1] for sample in self.samples) / len(self.samples)
        variance = sum(
            (sample[0] - mean_stamp) ** 2 for sample in self.samples
        )
        if variance <= 1e-12:
            self.rate_rad_s = 0.0
            return self.rate_rad_s, self.raw_rate_rad_s
        fitted_rate = sum(
            (sample[0] - mean_stamp) * (sample[1] - mean_yaw)
            for sample in self.samples
        ) / variance
        if self.max_abs_rate_rad_s > 0.0:
            fitted_rate = max(
                -self.max_abs_rate_rad_s,
                min(self.max_abs_rate_rad_s, fitted_rate),
            )
        alpha = self.smoothing_alpha
        self.rate_rad_s = (
            alpha * fitted_rate + (1.0 - alpha) * self.rate_rad_s
        )
        return self.rate_rad_s, self.raw_rate_rad_s

    @staticmethod
    def _normalize_angle(angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))
