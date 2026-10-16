"""Radius-based point selection."""

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:
    cKDTree = None


class _BruteTree:
    def __init__(self, xyz):
        self.xyz = xyz

    def query_ball_point(self, point, radius):
        distance_squared = np.sum(
            (self.xyz - np.asarray(point, dtype=np.float32)) ** 2,
            axis=1)
        return np.flatnonzero(distance_squared <= radius * radius)


class RadiusSelector:
    """Select point indices within a radius without exposing the tree."""

    def __init__(self):
        self._tree = None

    def set_points(self, points):
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError('points must have shape (N, 3)')
        if points.shape[0] == 0:
            self._tree = None
        elif cKDTree is not None:
            self._tree = cKDTree(points)
        else:
            self._tree = _BruteTree(points)

    def select(self, center, radius):
        if self._tree is None:
            return np.empty(0, dtype=np.int64)
        return np.asarray(
            self._tree.query_ball_point(center, radius), dtype=np.int64)
