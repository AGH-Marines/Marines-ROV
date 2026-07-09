#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

# Geometria 4 silników poziomych (indeksy 0-3)
HORIZONTAL_THRUSTERS = [
    (-0.149471, -0.189471, -0.7853981634),  # FrontRight
    (-0.149471, 0.189471, 0.7853981634),    # FrontLeft
    (0.149471, -0.189471, -2.3561944902),   # BackRight
    (0.149471, 0.189471, 2.3561944902),     # BackLeft
]

# Geometria 4 silników pionowych (indeksy 4-7) [x, y]
VERTICAL_THRUSTERS = [
    (0.26325, 0.20925),   # DiveFrontRight
    (0.26325, -0.20925),  # DiveFrontLeft
    (-0.26325, 0.20925),  # DiveBackRight
    (-0.26325, -0.20925), # DiveBackLeft
]

def build_horizontal_allocation_pinv():
    columns = []
    for x, y, yaw in HORIZONTAL_THRUSTERS:
        dir_x, dir_y = math.cos(yaw), math.sin(yaw)
        mz = x * dir_y - y * dir_x
        columns.append([dir_x, dir_y, mz])
    a = np.array(columns).T
    return np.linalg.pinv(a)

def build_vertical_allocation_pinv():
    """Buduje macierz A (3x4, wiersze Fz, Tx, Ty) dla silników pionowych."""
    columns = []
    for x, y in VERTICAL_THRUSTERS:
        # Zakładamy kierunek ciągu Z.
        # Ramię siły do momentu: Torque_x = y * Fz, Torque_y = -x * Fz
        columns.append([1.0, y, -x])
    a = np.array(columns).T
    return np.linalg.pinv(a)


class ThrusterAllocatorNode(Node):
    def __init__(self):
        super().__init__('thruster_allocator_node')

        self.armed = True
        self.max_thrust_per_motor = 10.0  # Szacunkowy maksymalny ciąg jednego silnika (Newtony)

        self.horizontal_pinv = build_horizontal_allocation_pinv()
        self.vertical_pinv = build_vertical_allocation_pinv()

        self.wrench_sub = self.create_subscription(Wrench, '/marines_rov/cmd_wrench', self.wrench_callback, 10)
        self.thruster_pub = self.create_publisher(Float64MultiArray, '/thrusters/STONEFISH', 10)
        self.arm_srv = self.create_service(SetBool, '/rov_actuation/arm', self.arm_callback)

        self.get_logger().info('Węzeł Alokacji Ciągu (Aktywna Stabilizacja) uruchomiony.')

    def arm_callback(self, request, response):
        self.armed = request.data
        response.success = True
        response.message = 'armed' if self.armed else 'disarmed'
        return response

    def wrench_callback(self, msg):
        thrusters = [0.0] * 8
        cmd_msg = Float64MultiArray()

        if not self.armed:
            cmd_msg.data = thrusters
            self.thruster_pub.publish(cmd_msg)
            return

        # 1. Silniki poziome (X, Y, Yaw)
        wrench_xy = np.array([msg.force.x, msg.force.y, msg.torque.z])
        horizontal_cmds = self.horizontal_pinv @ wrench_xy
        for i in range(4):
            # Normalizacja siły do sygnału [-1.0, 1.0]
            thrusters[i] = float(horizontal_cmds[i]) / self.max_thrust_per_motor

        # 2. Silniki pionowe (Z, Roll, Pitch)
        wrench_z_rp = np.array([msg.force.z, msg.torque.x, msg.torque.y])
        vertical_cmds = self.vertical_pinv @ wrench_z_rp
        for i in range(4):
            # Normalizacja siły do sygnału [-1.0, 1.0]
            thrusters[i+4] = float(vertical_cmds[i]) / self.max_thrust_per_motor

        # 3. Saturacja sygnału
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