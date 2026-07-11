import numpy as np
from scipy.spatial import cKDTree


class OccupancyIndex:
    def __init__(self, robot_radius: float):
        self.robot_radius = robot_radius
        self._centers = np.empty((0, 3))
        self._half_sizes = np.empty((0,))
        self._tree = None

    def update(self, marker_array):
        centers, half_sizes = [], []
        for marker in marker_array.markers:
            if marker.type != marker.CUBE_LIST or len(marker.points) == 0:
                continue
            half = marker.scale.x / 2.0
            for p in marker.points:
                centers.append((p.x, p.y, p.z))
                half_sizes.append(half)
        if not centers:
            self._tree = None
            return
        self._centers = np.array(centers)
        self._half_sizes = np.array(half_sizes)
        self._tree = cKDTree(self._centers)

    def nearby(self, point, radius):
        if self._tree is None:
            return np.empty((0, 3))
        idxs = self._tree.query_ball_point(point, r=radius)
        if not idxs:
            return np.empty((0, 3))
        return self._centers[idxs]

    def is_occupied(self, point) -> bool:
        if self._tree is None:
            return False  # no map yet: optimistically free, so as not to block planning
        max_half = self._half_sizes.max() if len(self._half_sizes) else 0.0
        margin = max_half + self.robot_radius
        idxs = self._tree.query_ball_point(point, r=margin * 1.7320508)  # sqrt(3), safe AABB bound
        for i in idxs:
            c = self._centers[i]
            h = self._half_sizes[i] + self.robot_radius
            if np.all(np.abs(np.asarray(point) - c) <= h):
                return True
        return False
