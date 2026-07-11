import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

from rov_actuation.robot_model import load_thrusters

VERTICAL_DIR_Z_THRESHOLD = 0.5


def build_horizontal_allocation_pinv(thrusters):
    """Builds matrix A (3xN, rows Fx, Fy, Mz) for the horizontal thrusters."""
    columns = []
    for t in thrusters:
        x, y = t.position[0], t.position[1]
        dir_x, dir_y = t.thrust_dir[0], t.thrust_dir[1]
        mz = x * dir_y - y * dir_x
        columns.append([dir_x, dir_y, mz])
    a = np.array(columns).T
    return np.linalg.pinv(a)


def build_vertical_allocation_pinv(thrusters):
    """Builds matrix A (3xN, rows Fz, Tx, Ty) for the vertical thrusters."""
    columns = []
    for t in thrusters:
        x, y = t.position[0], t.position[1]
        dir_z = t.thrust_dir[2]
        columns.append([dir_z, y * dir_z, -x * dir_z])
    a = np.array(columns).T
    return np.linalg.pinv(a)


class ThrusterAllocatorNode(Node):
    def __init__(self):
        super().__init__('thruster_allocator_node')

        self.armed = True
        self.max_thrust_per_motor = 10.0 # Newtons
        self.declare_parameter('robot_model_path', '')
        model_path = self.get_parameter('robot_model_path').value or None
        self.thrusters = load_thrusters(model_path)

        self.horizontal_idx = [
            i for i, t in enumerate(self.thrusters)
            if abs(t.thrust_dir[2]) < VERTICAL_DIR_Z_THRESHOLD
        ]
        self.vertical_idx = [
            i for i, t in enumerate(self.thrusters)
            if abs(t.thrust_dir[2]) >= VERTICAL_DIR_Z_THRESHOLD
        ]
        self.horizontal_pinv = build_horizontal_allocation_pinv(
            [self.thrusters[i] for i in self.horizontal_idx])
        self.vertical_pinv = build_vertical_allocation_pinv(
            [self.thrusters[i] for i in self.vertical_idx])

        self.wrench_sub = self.create_subscription(Wrench, '/marines_rov/cmd_wrench', self.wrench_callback, 10)
        self.thruster_pub = self.create_publisher(Float64MultiArray, '/thrusters/STONEFISH', 10)
        self.arm_srv = self.create_service(SetBool, '/rov_actuation/arm', self.arm_callback)

        self.get_logger().info(
            f'Thrust Allocation Node (Active Stabilization) started with '
            f'{len(self.thrusters)} thrusters ({len(self.horizontal_idx)} horizontal, '
            f'{len(self.vertical_idx)} vertical), loaded from {model_path or "rov_simulation"}.')

    def arm_callback(self, request, response):
        self.armed = request.data
        response.success = True
        response.message = 'armed' if self.armed else 'disarmed'
        return response

    def wrench_callback(self, msg):
        thrusters = [0.0] * len(self.thrusters)
        cmd_msg = Float64MultiArray()

        if not self.armed:
            cmd_msg.data = thrusters
            self.thruster_pub.publish(cmd_msg)
            return

        # 1. Horizontal thrusters (X, Y, Yaw)
        wrench_xy = np.array([msg.force.x, msg.force.y, msg.torque.z])
        horizontal_cmds = self.horizontal_pinv @ wrench_xy
        for local_i, global_i in enumerate(self.horizontal_idx):
            # Normalize the force to a [-1.0, 1.0] signal
            thrusters[global_i] = float(horizontal_cmds[local_i]) / self.max_thrust_per_motor

        # 2. Vertical thrusters (Z, Roll, Pitch)
        wrench_z_rp = np.array([msg.force.z, msg.torque.x, msg.torque.y])
        vertical_cmds = self.vertical_pinv @ wrench_z_rp
        for local_i, global_i in enumerate(self.vertical_idx):
            # Normalize the force to a [-1.0, 1.0] signal
            thrusters[global_i] = float(vertical_cmds[local_i]) / self.max_thrust_per_motor

        # 3. Signal saturation
        thrusters = [max(min(t, 1.0), -1.0) for t in thrusters]

        cmd_msg.data = thrusters
        self.thruster_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterAllocatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
