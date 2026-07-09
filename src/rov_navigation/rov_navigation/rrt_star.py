import math
import random
import numpy as np


class RRTNode:
    __slots__ = ('point', 'parent', 'cost')

    def __init__(self, point, parent=None, cost=0.0):
        self.point = np.asarray(point, dtype=float)
        self.parent = parent
        self.cost = cost


class RRTStarPlanner:
    def __init__(self, bounds_min, bounds_max, collision_fn, step_size=0.5,
                 goal_bias=0.1, search_radius_coeff=3.0, max_iterations=3000,
                 collision_check_resolution=0.1):
        self.bounds_min = np.array(bounds_min, dtype=float)
        self.bounds_max = np.array(bounds_max, dtype=float)
        self.collision_fn = collision_fn  # Callable[[np.ndarray]] -> bool (True == zajęte)
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.search_radius_coeff = search_radius_coeff
        self.max_iterations = max_iterations
        self.collision_check_resolution = collision_check_resolution

    def _sample(self, goal):
        if random.random() < self.goal_bias:
            return np.asarray(goal, dtype=float)
        return self.bounds_min + np.random.rand(3) * (self.bounds_max - self.bounds_min)

    def _nearest(self, tree, point):
        return min(tree, key=lambda n: np.linalg.norm(n.point - point))

    def _steer(self, from_node, to_point):
        direction = to_point - from_node.point
        dist = np.linalg.norm(direction)
        if dist <= self.step_size:
            return to_point
        return from_node.point + direction / dist * self.step_size

    def _collision_free(self, a, b):
        dist = np.linalg.norm(b - a)
        steps = max(1, int(dist / self.collision_check_resolution))
        for i in range(steps + 1):
            p = a + (b - a) * (i / steps)
            if self.collision_fn(p):
                return False
        return True

    def _near(self, tree, point, n):
        r = self.search_radius_coeff * (math.log(max(n, 2)) / max(n, 2)) ** (1.0 / 3.0)
        r = max(r, self.step_size)
        return [node for node in tree if np.linalg.norm(node.point - point) <= r]

    def plan(self, start, goal, goal_tolerance=0.3):
        start = np.asarray(start, dtype=float)
        goal = np.asarray(goal, dtype=float)
        if self.collision_fn(start) or self.collision_fn(goal):
            return None  # start/cel wewnątrz przeszkody

        tree = [RRTNode(start, parent=None, cost=0.0)]
        best_goal_node = None

        for _ in range(self.max_iterations):
            rnd = self._sample(goal)
            nearest = self._nearest(tree, rnd)
            new_point = self._steer(nearest, rnd)

            if not self._collision_free(nearest.point, new_point):
                continue

            near_nodes = self._near(tree, new_point, len(tree))
            best_parent = nearest
            best_cost = nearest.cost + np.linalg.norm(new_point - nearest.point)
            for cand in near_nodes:
                c = cand.cost + np.linalg.norm(new_point - cand.point)
                if c < best_cost and self._collision_free(cand.point, new_point):
                    best_parent, best_cost = cand, c

            new_node = RRTNode(new_point, parent=best_parent, cost=best_cost)
            tree.append(new_node)

            for cand in near_nodes:
                c = new_node.cost + np.linalg.norm(cand.point - new_point)
                if c < cand.cost and self._collision_free(new_point, cand.point):
                    cand.parent = new_node
                    cand.cost = c

            if np.linalg.norm(new_point - goal) < goal_tolerance:
                if best_goal_node is None or new_node.cost < best_goal_node.cost:
                    best_goal_node = new_node

        if best_goal_node is None:
            return None

        path = []
        node = best_goal_node
        while node is not None:
            path.append(node.point)
            node = node.parent
        path.reverse()
        path.append(goal)
        return self._simplify(path)

    def _simplify(self, path):
        """Wygładzanie skrótami: zachłannie łączy najwcześniejszy waypoint
        z najdalszym, do którego odcinek jest wolny od kolizji."""
        if len(path) <= 2:
            return path
        simplified = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._collision_free(path[i], path[j]):
                j -= 1
            simplified.append(path[j])
            i = j
        return simplified
