#!/usr/bin/env python3
"""Pure target-profile math shared by Task C publisher and tests."""

import math


def has_reached_observation_time(current_time, target_time, tolerance=1.0e-9):
    if not all(math.isfinite(value) for value in (current_time, target_time, tolerance)):
        raise ValueError("observation times and tolerance must be finite")
    if target_time < 0.0 or tolerance < 0.0:
        raise ValueError("target time and tolerance must be nonnegative")
    return current_time + tolerance >= target_time


def quintic_state(tau, duration):
    if not math.isfinite(tau) or not 0.0 <= tau <= 1.0:
        raise ValueError("tau must be finite and within [0, 1]")
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration must be positive and finite")
    position = 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5
    derivative_tau = 30.0 * tau ** 2 - 60.0 * tau ** 3 + 30.0 * tau ** 4
    second_derivative_tau = 60.0 * tau - 180.0 * tau ** 2 + 120.0 * tau ** 3
    return (
        position,
        derivative_tau / duration,
        second_derivative_tau / (duration * duration),
    )


def sample_profile(profile, duration, sample_period):
    if profile not in ("linear", "quintic"):
        raise ValueError("profile must be linear or quintic")
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration must be positive and finite")
    if not math.isfinite(sample_period) or sample_period <= 0.0:
        raise ValueError("sample_period must be positive and finite")
    if profile == "linear":
        return [(0.0, 0.0), (duration, 1.0)]

    segment_count = max(2, int(math.ceil(duration / sample_period)))
    samples = []
    for index in range(segment_count + 1):
        tau = float(index) / segment_count
        samples.append((duration * tau, quintic_state(tau, duration)[0]))
    samples[0] = (0.0, 0.0)
    samples[-1] = (duration, 1.0)
    return samples
