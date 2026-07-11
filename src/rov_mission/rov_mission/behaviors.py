import math

import py_trees
import py_trees.common
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from std_srvs.srv import SetBool
from octomap_msgs.msg import Octomap
from tf2_ros import Buffer, TransformListener, TransformException
from tf_transformations import euler_from_quaternion, quaternion_from_euler


def _lookup_pose(node, tf_buffer, map_frame, base_frame):
    try:
        # Time() (zero stamp) asks for the latest available transform. Passing
        # node.get_clock().now() instead fails on every call with an
        # extrapolation-into-the-future error, since the TF buffer is always
        # slightly behind wall-clock time - which silently stalls this lookup
        # forever (the exception is swallowed below).
        tf = tf_buffer.lookup_transform(map_frame, base_frame, Time())
    except TransformException:
        return None
    t = tf.transform.translation
    q = tf.transform.rotation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return (t.x, t.y, t.z, yaw)


class CaptureHomePose(py_trees.behaviour.Behaviour):
    def __init__(self, name="CaptureHomePose"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="home_pose", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.map_frame = self.node.get_parameter('map_frame').value
        self.base_frame = self.node.get_parameter('base_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def update(self):
        pose = _lookup_pose(self.node, self.tf_buffer, self.map_frame, self.base_frame)
        if pose is None:
            return py_trees.common.Status.RUNNING
        self.blackboard.home_pose = pose
        return py_trees.common.Status.SUCCESS


class ArmThrusters(py_trees.behaviour.Behaviour):
    def __init__(self, name="ArmThrusters"):
        super().__init__(name)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.client = self.node.create_client(SetBool, '/rov_actuation/arm')
        self._future = None

    def initialise(self):
        self._future = None

    def update(self):
        if not self.client.service_is_ready():
            return py_trees.common.Status.RUNNING
        if self._future is None:
            req = SetBool.Request()
            req.data = True
            self._future = self.client.call_async(req)
            return py_trees.common.Status.RUNNING
        if not self._future.done():
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS if self._future.result().success else py_trees.common.Status.FAILURE


class DiveToDepth(py_trees.behaviour.Behaviour):
    """Publishes its dive target through /marines_rov/goal_pose (like PublishGoal),
    NOT directly onto /marines_rov/global_path: that topic is owned by planner_node
    (RRT*) and consumed by local_planner_node to decide whether "the goal" has been
    reached. A second writer publishing a trivial 1-point "stay here" path there
    raced with the real route to the mission goal and got misread as an instant,
    false goal-reached signal (see MonitorMission)."""

    def __init__(self, name="DiveToTargetDepth", target_z=-1.5, tolerance=0.15):
        super().__init__(name)
        self.target_z = target_z
        self.tolerance = tolerance

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.map_frame = self.node.get_parameter('map_frame').value
        self.base_frame = self.node.get_parameter('base_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.goal_pub = self.node.create_publisher(PoseStamped, '/marines_rov/goal_pose', 10)
        self._sent = False

    def initialise(self):
        self._sent = False

    def update(self):
        pose = _lookup_pose(self.node, self.tf_buffer, self.map_frame, self.base_frame)
        if pose is None:
            return py_trees.common.Status.RUNNING
        cx, cy, cz, cyaw = pose
        if not self._sent:
            msg = PoseStamped()
            msg.header.frame_id = self.map_frame
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = cx, cy, self.target_z
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, cyaw)
            msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
            msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
            self.goal_pub.publish(msg)
            self._sent = True
        if abs(cz - self.target_z) < self.tolerance:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class StartMapping(py_trees.behaviour.Behaviour):
    """Does not dynamically launch octomap_server (it already runs continuously in
    autonomy.launch.py); gates the mission on the mapping pipeline being alive."""

    def __init__(self, name="StartMapping"):
        super().__init__(name)
        self._ready = False

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.create_subscription(Octomap, '/octomap_binary', self._cb, 1)

    def _cb(self, _msg):
        self._ready = True

    def update(self):
        return py_trees.common.Status.SUCCESS if self._ready else py_trees.common.Status.RUNNING


class PublishGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name, key=None, static_goal=None):
        """key: blackboard key to read the (x,y,z,yaw) tuple from (e.g. 'home_pose').
        static_goal: (x,y,z,yaw) tuple used when key is None (mission parameters)."""
        super().__init__(name)
        self.key = key
        self.static_goal = static_goal
        if key is not None:
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.pub = self.node.create_publisher(PoseStamped, '/marines_rov/goal_pose', 10)
        self.map_frame = self.node.get_parameter('map_frame').value
        self._sent = False

    def initialise(self):
        self._sent = False

    def update(self):
        if self._sent:
            return py_trees.common.Status.SUCCESS
        goal = getattr(self.blackboard, self.key) if self.key else self.static_goal
        if goal is None:
            return py_trees.common.Status.RUNNING
        x, y, z, yaw = goal
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.orientation.x, msg.pose.orientation.y = qx, qy
        msg.pose.orientation.z, msg.pose.orientation.w = qz, qw
        self.pub.publish(msg)
        self._sent = True
        return py_trees.common.Status.SUCCESS


class MonitorMission(py_trees.behaviour.Behaviour):
    """Confirms arrival by checking the robot's own tf position against the target,
    rather than trusting /marines_rov/path_complete: that flag is shared with
    planner_node/local_planner_node's internal bookkeeping and can flip True for a
    stale or unrelated path (see DiveToDepth), which previously made the mission
    report success within one tick of publishing a new goal."""

    def __init__(self, name="MonitorPathExecution", key=None, static_goal=None,
                 timeout_sec=120.0, tolerance=0.3):
        super().__init__(name)
        self.key = key
        self.static_goal = static_goal
        self.timeout_sec = timeout_sec
        self.tolerance = tolerance
        self._start_time = None
        if key is not None:
            self.blackboard = self.attach_blackboard_client(name=self.name)
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.map_frame = self.node.get_parameter('map_frame').value
        self.base_frame = self.node.get_parameter('base_frame').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def initialise(self):
        self._start_time = self.node.get_clock().now()

    def update(self):
        goal = getattr(self.blackboard, self.key) if self.key else self.static_goal
        pose = _lookup_pose(self.node, self.tf_buffer, self.map_frame, self.base_frame)
        if goal is not None and pose is not None:
            gx, gy, gz, _ = goal
            cx, cy, cz, _ = pose
            if math.dist((cx, cy, cz), (gx, gy, gz)) < self.tolerance:
                return py_trees.common.Status.SUCCESS
        elapsed = (self.node.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed > self.timeout_sec:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING
