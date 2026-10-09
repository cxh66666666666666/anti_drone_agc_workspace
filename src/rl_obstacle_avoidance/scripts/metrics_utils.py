#!/usr/bin/env python3
"""Pure episode-metric helpers used by training logs and offline checks."""

import math


def _finite(values):
    return [float(value) for value in values if math.isfinite(float(value))]


def path_length(points):
    """Return planar polyline length for ``(x, y[, z])`` samples."""
    total = 0.0
    previous = None
    for point in points:
        if point is None or len(point) < 2:
            continue
        current = (float(point[0]), float(point[1]))
        if previous is not None:
            total += math.hypot(current[0] - previous[0],
                                current[1] - previous[1])
        previous = current
    return total


def recovery_time(times, errors, tolerance, hold_duration,
                  start_time=None):
    """Return time until error remains within tolerance for hold_duration.

    ``None`` means no qualifying recovery window was observed. If no recovery
    event is applicable (``start_time is None``), return ``0.0``.
    """
    if start_time is None:
        return 0.0
    samples = [(float(t), abs(float(error)))
               for t, error in zip(times, errors)
               if math.isfinite(float(t)) and math.isfinite(float(error)) and
               float(t) >= float(start_time)]
    if not samples:
        return None
    samples.sort()
    for index, (candidate, error) in enumerate(samples):
        if error > tolerance:
            continue
        end = candidate + max(0.0, float(hold_duration))
        if all(t >= end or current_error <= tolerance
               for t, current_error in samples[index:]):
            # Require the observed samples to span the requested hold time.
            if samples[-1][0] + 1e-9 >= end:
                return max(0.0, candidate - float(start_time))
    return None


def summarize_episode(errors, times=None, positions=None, steady_window_s=1.0,
                      recovery_start_time=None, recovery_tolerance=1.0,
                      recovery_hold_s=0.5):
    """Compute the fixed Stage-1 task-5 metric dictionary."""
    finite_errors = _finite(errors)
    if not finite_errors:
        return {
            "rmse": float("nan"),
            "max_error": float("nan"),
            "steady_error": float("nan"),
            "path_length": path_length(positions or []),
            "recovery_time": (0.0 if recovery_start_time is None else None),
        }
    if times is None:
        times = list(range(len(errors)))
    paired = [(float(t), abs(float(error)))
              for t, error in zip(times, errors)
              if math.isfinite(float(t)) and math.isfinite(float(error))]
    paired.sort()
    window_start = paired[-1][0] - max(0.0, float(steady_window_s))
    steady = [error for t, error in paired if t >= window_start]
    return {
        "rmse": math.sqrt(sum(error * error for _, error in paired) /
                           len(paired)),
        "max_error": max(error for _, error in paired),
        "steady_error": (sum(steady) / len(steady) if steady else float("nan")),
        "path_length": path_length(positions or []),
        "recovery_time": recovery_time(
            [t for t, _ in paired], [error for _, error in paired],
            recovery_tolerance, recovery_hold_s, recovery_start_time),
    }
