#!/usr/bin/env python3
"""Vectorized helpers for reducing a point cloud to horizontal sectors."""

import math

import numpy as np


def points_to_sectors(points, num_sectors, min_range, max_range,
                      max_abs_height):
    """Return normalized minimum XY range in each body-frame sector."""
    if num_sectors <= 0 or not 0.0 <= min_range < max_range:
        raise ValueError("invalid sector or range configuration")
    if max_abs_height < 0.0:
        raise ValueError("max_abs_height must be non-negative")

    points = np.asarray(points, dtype=np.float32)
    if points.size == 0:
        return [1.0] * num_sectors
    points = points.reshape((-1, 3))
    finite = np.isfinite(points).all(axis=1)
    distances = np.hypot(points[:, 0], points[:, 1])
    valid = (finite & (distances >= min_range) &
             (distances <= max_range) &
             (np.abs(points[:, 2]) <= max_abs_height))
    if not np.any(valid):
        return [1.0] * num_sectors

    distances = distances[valid]
    angles = np.arctan2(points[valid, 1], points[valid, 0])
    sector_size = 2.0 * math.pi / num_sectors
    indices = np.floor((angles + math.pi) / sector_size).astype(np.int32)
    indices = np.clip(indices, 0, num_sectors - 1)
    sectors = np.full(num_sectors, max_range, dtype=np.float32)
    np.minimum.at(sectors, indices, distances)
    return (sectors / max_range).astype(float).tolist()
