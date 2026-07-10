#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
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
        # wnętrze basenu, ściany .scn @ +-5, dno @ z=4 w konwencji Stonefisha (Z w dół).
        # Rama "odom" ma Z odwrócone (odometry_converter_node, powierzchnia ~= 0, głębiej
        # = bardziej ujemne), więc dno (najgłębiej) to bounds_min, powierzchnia to bounds_max.
        self.declare_parameter('bounds_min', [-4.5, -4.5, -3.8])
        self.declare_parameter('bounds_max', [4.5, 4.5, -0.2])
        self.declare_parameter('replan_period', 2.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.occupancy = OccupancyIndex(robot_radius=self.get_parameter('robot_radius').value)
        self.create_subscription(MarkerArray, '/occupied_cells_vis_array', self.occupancy_callback, 10)
        self.create_subscription(PoseStamped, '/marines_rov/goal_pose', self.goal_callback, 10)
        self.create_subscription(Bool, '/marines_rov/path_complete', self.path_complete_callback, 10)
        self.path_pub = self.create_publisher(Path, '/marines_rov/global_path', 10)

        self.current_goal = None
        self.goal_reached = False
        self.current_waypoints = None
        self.replan_timer = self.create_timer(
            self.get_parameter('replan_period').value, self.replan_check)

        self.get_logger().info('Węzeł Planowania Ścieżki (RRT*) uruchomiony.')

    def occupancy_callback(self, msg: MarkerArray):
        self.occupancy.update(msg)

    def get_current_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'TF niedostępny do planowania: {ex}', throttle_duration_sec=2.0)
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
        self.goal_reached = False
        self._plan_and_publish(msg)

    def path_complete_callback(self, msg: Bool):
        # Gdy dotrze do celu, przestajemy replanować do tego samego punktu -
        # inaczej pid_controller_node co replan_period dostaje nową (trywialną)
        # trasę i cyklicznie wchodzi/wychodzi z trybu podążania po ścieżce.
        if msg.data:
            self.goal_reached = True

    def replan_check(self):
        """Wymóg Trybu 2: 'na bieżąco korygując trasę w przypadku wykrycia
        nowych obiektów' — replanuje TYLKO gdy aktualna trasa faktycznie
        przestała być bezpieczna. Bezwarunkowe przeliczanie co replan_period
        (nawet gdy trasa wciąż jest poprawna) powodowało, że robot dostawał
        świeżo policzoną trasę zaczynającą się w pozycji sprzed chwili (RRT*
        planuje z opóźnieniem), czyli punkt startowy nowej trasy był już "za
        robotem" - stąd cykliczne zawracanie."""
        if self.current_goal is None or self.goal_reached:
            return
        if self.current_waypoints and all(
            not self.occupancy.is_occupied(wp) for wp in self.current_waypoints
        ):
            return
        self._plan_and_publish(self.current_goal)

    def _plan_and_publish(self, goal_msg: PoseStamped):
        start = self.get_current_position()
        if start is None:
            return
        goal = (goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z)

        planner = self.make_planner()
        waypoints = planner.plan(start, goal, goal_tolerance=self.get_parameter('goal_tolerance').value)

        if waypoints is None:
            self.get_logger().warn('RRT*: brak trasy do celu (przekroczono max_iterations lub cel w kolizji).')
            return

        self.current_waypoints = waypoints

        # waypoints[0] to pozycja startowa użyta do planowania (czyli w praktyce
        # "tam gdzie robot już jest", z niewielkim opóźnieniem - planowanie chwilę
        # trwa) - to nie jest sensowny cel do dopłynięcia, tylko punkt odniesienia.
        # Pomijamy go, żeby kontroler od razu celował w pierwszy realny punkt trasy.
        published_waypoints = waypoints[1:] if len(waypoints) > 1 else waypoints

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
        self.get_logger().info(f'Opublikowano trasę: {len(published_waypoints)} punktów.')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
