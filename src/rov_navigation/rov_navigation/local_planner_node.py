#!/usr/bin/env python3
"""Reactive local layer (APF) for the hybrid navigation mode.

Hybrid architecture: the global planner (planner_node, RRT*) computes the
`/marines_rov/global_path` route, and this node continuously (every cycle)
corrects it locally against the live obstacle map and publishes a "carrot"
(target point to follow) on `/marines_rov/local_path`. The pid_controller_node
tracks this topic instead of the global one (switched via the path_topic
parameter), so the low-level control layer is identical in both modes - making
an A/B comparison fair.

Method: Pure Pursuit (attraction toward a point on the path at the lookahead
distance) + Artificial Potential Fields (repulsion from the nearest occupied
octomap cells). The global path guards against APF local minima - the
attraction leads around obstacles along the RRT* route.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import quaternion_from_euler

from rov_navigation.occupancy_index import OccupancyIndex


class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner_node')

        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('lookahead_dist', 1.5)     # how far ahead of the robot pure-pursuit aims [m]
        self.declare_parameter('repulsion_radius', 1)   # range of the repulsion effect [m]
        self.declare_parameter('repulsion_gain', 0.5)    # repulsion strength
        self.declare_parameter('max_lateral_offset', 2.0) # max carrot offset from the path [m]
        self.declare_parameter('goal_tolerance', 0.2)     # when to consider the global goal reached [m]
        self.declare_parameter('robot_radius', 0.3)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.occupancy = OccupancyIndex(robot_radius=self.get_parameter('robot_radius').value)
        self.path_pts = None  # list of np.ndarray(3,) - global path waypoints

        self.create_subscription(MarkerArray, '/occupied_cells_vis_array', self._occupancy_cb, 10)
        self.create_subscription(Path, '/marines_rov/global_path', self._path_cb, 10)
        self.carrot_pub = self.create_publisher(Path, '/marines_rov/local_path', 10)
        self.complete_pub = self.create_publisher(Bool, '/marines_rov/path_complete', 10)

        period = 1.0 / self.get_parameter('rate').value
        self.timer = self.create_timer(period, self._update)

        self.get_logger().info('Local Planner Node (hybrid: Pure-Pursuit + APF) started.')

    def _occupancy_cb(self, msg: MarkerArray):
        self.occupancy.update(msg)

    def _path_cb(self, msg: Path):
        if not msg.poses:
            self.path_pts = None
            return
        self.path_pts = [
            np.array([p.pose.position.x, p.pose.position.y, p.pose.position.z], dtype=float)
            for p in msg.poses
        ]

    def _current_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            self.get_logger().warn('TF unavailable in local_planner.', throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        return np.array([t.x, t.y, t.z], dtype=float)

    def _pure_pursuit_point(self, robot, pts, lookahead):
        """Point on the path at distance `lookahead` ahead of the robot's projection onto the path."""
        if len(pts) == 1:
            return pts[0]

        # 1) find the nearest point (projection) on the polyline
        best_d = float('inf')
        best_seg = 0
        best_proj = pts[0]
        for i in range(len(pts) - 1):
            a, b = pts[i], pts[i + 1]
            ab = b - a
            denom = float(np.dot(ab, ab))
            t = 0.0 if denom < 1e-9 else float(np.clip(np.dot(robot - a, ab) / denom, 0.0, 1.0))
            proj = a + t * ab
            d = float(np.linalg.norm(robot - proj))
            if d < best_d:
                best_d, best_seg, best_proj = d, i, proj

        # 2) walk `lookahead` meters forward along the path from the projection
        remaining = lookahead
        cur = best_proj
        i = best_seg
        while i < len(pts) - 1:
            seg_end = pts[i + 1]
            seg = seg_end - cur
            seg_len = float(np.linalg.norm(seg))
            if seg_len >= remaining:
                return cur + (seg / seg_len) * remaining if seg_len > 1e-9 else seg_end
            remaining -= seg_len
            cur = seg_end
            i += 1
        return pts[-1]  # reached the end of the path -> aim at the goal

    def _repulsion(self, robot, radius, gain):
        """Repulsion vector (offset in meters) from nearby occupied cells."""
        centers = self.occupancy.nearby(robot, radius)
        if len(centers) == 0:
            return np.zeros(3)
        rep = np.zeros(3)
        for c in centers:
            diff = robot - c
            d = float(np.linalg.norm(diff))
            if d < 1e-3:
                continue
            # classic APF: strength increases with proximity, drops to zero at `radius`
            strength = gain * (1.0 / d - 1.0 / radius) / (d * d)
            if strength > 0.0:
                rep += (diff / d) * strength
        return rep

    def _update(self):
        if self.path_pts is None:
            return
        robot = self._current_position()
        if robot is None:
            return

        goal = self.path_pts[-1]
        if float(np.linalg.norm(goal - robot)) < self.get_parameter('goal_tolerance').value:
            self.complete_pub.publish(Bool(data=True))
            self._publish_carrot(goal, robot)
            # self.get_logger().info("goal reached")
            return
        self.complete_pub.publish(Bool(data=False))

        lookahead = self._pure_pursuit_point(robot, self.path_pts,
                                              self.get_parameter('lookahead_dist').value)
        rep = self._repulsion(robot,
                              self.get_parameter('repulsion_radius').value,
                              self.get_parameter('repulsion_gain').value)

        max_off = self.get_parameter('max_lateral_offset').value
        rep_norm = float(np.linalg.norm(rep))
        if rep_norm > max_off:
            rep = rep / rep_norm * max_off

        carrot = lookahead + rep
        self._publish_carrot(carrot, robot)

    def _publish_carrot(self, carrot, robot):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = float(carrot[0])
        pose.pose.position.y = float(carrot[1])
        pose.pose.position.z = float(carrot[2])
        yaw = math.atan2(carrot[1] - robot[1], carrot[0] - robot[0])
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        msg.poses.append(pose)
        self.carrot_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
