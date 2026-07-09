import numpy as np
from scipy.spatial import cKDTree


class OccupancyIndex:
    """Utrzymuje odpytywalną strukturę zajętości zbudowaną z tablicy MarkerArray
    /occupied_cells_vis_array publikowanej przez octomap_server (jeden marker
    typu CUBE_LIST na głębokość drzewa ósemkowego)."""

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

    def is_occupied(self, point) -> bool:
        if self._tree is None:
            return False  # brak mapy jeszcze: optymistycznie wolne, żeby nie blokować planowania
        max_half = self._half_sizes.max() if len(self._half_sizes) else 0.0
        margin = max_half + self.robot_radius
        idxs = self._tree.query_ball_point(point, r=margin * 1.7320508)  # sqrt(3), bezpieczny bound AABB
        for i in idxs:
            c = self._centers[i]
            h = self._half_sizes[i] + self.robot_radius
            if np.all(np.abs(np.asarray(point) - c) <= h):
                return True
        return False
