#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import quaternion_from_euler

from rov_navigation.occupancy_index import OccupancyIndex
from rov_navigation.rrt_star import RRTStarPlanner


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('robot_radius', 0.3)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('step_size', 0.5)
        self.declare_parameter('goal_bias', 0.1)
        self.declare_parameter('max_iterations', 3000)

        self.declare_parameter('refine_iterations', 250)

        self.declare_parameter('bounds_min', [-0.5, -4.7, -3.8])
        self.declare_parameter('bounds_max', [25.5, 4.7, -0.2])

        self.declare_parameter('replan_period', 0.5)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.occupancy = OccupancyIndex(robot_radius=self.get_parameter('robot_radius').value)
        self.create_subscription(MarkerArray, '/occupied_cells_vis_array', self.occupancy_callback, 10)
        self.create_subscription(PoseStamped, '/marines_rov/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/marines_rov/global_path', 10)

        self.current_goal = None
        self.current_waypoints = None
        self.replan_timer = self.create_timer(
            self.get_parameter('replan_period').value, self.replan_check)

        self.get_logger().info('Path Planning Node (RRT*) started.')

    def occupancy_callback(self, msg: MarkerArray):
        self.occupancy.update(msg)

    def get_current_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'TF unavailable for planning: {ex}', throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z)

    def make_planner(self):
        return RRTStarPlanner(
            bounds_min=self.get_parameter('bounds_min').value,
            bounds_max=self.get_parameter('bounds_max').value,
            collision_fn=self.occupancy.is_occupied,
            step_size=self.get_parameter('step_size').value,
            goal_bias=self.get_parameter('goal_bias').value,
            max_iterations=self.get_parameter('max_iterations').value,
        )

    def goal_callback(self, msg: PoseStamped):
        self.current_goal = msg
        # New goal = plan from scratch (don't seed with the route to the previous goal).
        self.current_waypoints = None
        self._plan_and_publish(msg)

    def _goal_reached(self, position):
        goal = self.current_goal.pose.position
        dist = math.dist(position, (goal.x, goal.y, goal.z))
        return dist < self.get_parameter('goal_tolerance').value

    def _segment_occupied(self, a, b, res=0.15):
        """Samples the straight segment a->b at `res` meter spacing against the
        current occupancy map. Shared by _path_still_clear (checks the stored
        route) and replan_check (checks the robot's direct line to where it's
        currently heading)."""
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        dist = float(np.linalg.norm(b - a))
        steps = max(1, int(dist / res))
        for i in range(steps + 1):
            p = a + (b - a) * (i / steps)
            if self.occupancy.is_occupied(p):
                return True
        return False

    def _path_still_clear(self, waypoints):
        pts = [np.asarray(w, dtype=float) for w in waypoints]
        for a, b in zip(pts[:-1], pts[1:]):
            if self._segment_occupied(a, b):
                return False
        return True

    def _nearest_segment_index(self, position, waypoints):
        """Index i such that the robot's current position projects closest onto
        segment (waypoints[i], waypoints[i+1]) - i.e. which leg of the stored
        route it's currently on."""
        pos = np.asarray(position, dtype=float)
        best_d = float('inf')
        best_i = 0
        for i in range(len(waypoints) - 1):
            a, b = np.asarray(waypoints[i], dtype=float), np.asarray(waypoints[i + 1], dtype=float)
            ab = b - a
            denom = float(np.dot(ab, ab))
            t = 0.0 if denom < 1e-9 else float(np.clip(np.dot(pos - a, ab) / denom, 0.0, 1.0))
            d = float(np.linalg.norm(pos - (a + t * ab)))
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _next_waypoint(self, position, waypoints):
        i = self._nearest_segment_index(position, waypoints)
        return np.asarray(waypoints[i + 1], dtype=float)

    def _trim_seed(self, position, waypoints):
        """Drops already-passed waypoints before handing the old route to RRT*
        as a seed. Without this, _seed_tree() walks the seed IN ORDER starting
        from waypoints[0] - which the robot has long since left behind - and
        happily builds robot -> (old, passed) waypoint -> ... -> goal, since
        that's geometrically collision-free (the robot just came from there).
        refine_iterations then only gets a limited budget to rewire that
        backtrack away, and often doesn't: the robot visibly reverses to a
        waypoint it already passed before continuing towards the goal."""
        if len(waypoints) < 2:
            return waypoints
        i = self._nearest_segment_index(position, waypoints)
        return waypoints[i + 1:]

    def replan_check(self):
        if self.current_goal is None:
            return
        position = self.get_current_position()
        if position is None:
            return
        if self._goal_reached(position):
            return
        if self.current_waypoints:
            if self._path_still_clear(self.current_waypoints):
                target = self._next_waypoint(position, self.current_waypoints)
                if not self._segment_occupied(position, target):
                    return
        self._plan_and_publish(self.current_goal)

    def _plan_and_publish(self, goal_msg: PoseStamped):
        start = self.get_current_position()
        if start is None:
            return
        goal = (goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z)

        planner = self.make_planner()
        seed_path = self._trim_seed(start, self.current_waypoints) if self.current_waypoints else None
        waypoints = planner.plan(
            start, goal,
            goal_tolerance=self.get_parameter('goal_tolerance').value,
            seed_path=seed_path,
            refine_iterations=self.get_parameter('refine_iterations').value,
        )

        if waypoints is None:
            self.get_logger().warn('RRT*: no route to goal (max_iterations exceeded or goal in collision).')
            return

        self.current_waypoints = waypoints
        published_waypoints = waypoints

        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for idx, wp in enumerate(published_waypoints):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = float(wp[0]), float(wp[1]), float(wp[2])
            if idx < len(published_waypoints) - 1:
                nxt = published_waypoints[idx + 1]
                yaw = math.atan2(nxt[1] - wp[1], nxt[0] - wp[0])
            else:
                yaw = 0.0
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
            pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
            pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published route: {len(published_waypoints)} points.')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
