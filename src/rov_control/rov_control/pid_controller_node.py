#!/usr/bin/env python3
import math
import rclpy
from PyQt5.QtCore import flush
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
from geometry_msgs.msg import Wrench, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class PID:
    def __init__(self, kp, ki, kd, out_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.out_limit = out_limit

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):

        if dt <= 0.0:
            return 0.0
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        out = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.out_limit is not None:
            out = max(min(out, self.out_limit), -self.out_limit)
        return out


class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        self.declare_parameter('map_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('default_depth', -2.0)
        self.declare_parameter('default_yaw', 0.0)
        self.declare_parameter('xy_deadzone', 0.05)
        self.declare_parameter('xy_force_scale', 40.0)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.wrench_pub = self.create_publisher(Wrench, '/marines_rov/cmd_wrench', 10)
        self.path_complete_pub = self.create_publisher(Bool, '/marines_rov/path_complete', 10)

        self.create_subscription(Twist, '/marines_rov/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Path, '/marines_rov/global_path', self.path_callback, 10)

        # Regulatory dla wszystkich 6 stopni swobody
        self.pid_x = PID(6.0, 0.1, 1.0, out_limit=60.0)
        self.pid_y = PID(6.0, 0.1, 1.0, out_limit=60.0)
        self.pid_z = PID(5.0, 0.1, 0.5, out_limit=60.0)

        # Aktywna stabilizacja poziomu (Roll/Pitch docelowo zawsze 0.0)
        self.pid_roll = PID(15.0, 0.5, 2.0, out_limit=40.0)
        self.pid_pitch = PID(15.0, 0.5, 2.0, out_limit=40.0)
        self.pid_yaw = PID(1.8, 0.05, 0.8, out_limit=30.0)

        self.hold_x = None
        self.hold_y = None
        self.hold_z = self.get_parameter('default_depth').value
        self.hold_yaw = self.get_parameter('default_yaw').value

        self.last_cmd_vel = Twist()
        self.last_cmd_vel_time = self.get_clock().now()

        self.path = None
        self.path_index = 0

        self.last_time = self.get_clock().now()
        period = 1.0 / self.get_parameter('control_rate').value
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info('Węzeł Regulatora PID (z Aktywną Stabilizacją 6-DOF) uruchomiony.')

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.path_index = 0
        self.pid_x.reset()
        self.pid_y.reset()
        # Wyczyść zapamiętaną pozycję/kurs trzymania - inaczej po zakończeniu tej
        # trasy robot próbowałby wrócić do miejsca sprzed jej rozpoczęcia zamiast
        # zostać tam, gdzie faktycznie dopłynął.
        self.hold_x = None
        self.hold_y = None
        self.get_logger().info(f'Nowa ścieżka: {len(self.path)} punktów.')

    def get_current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'TF {self.map_frame}->{self.base_frame} niedostępny.', throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return t.x, t.y, t.z, roll, pitch, yaw

    def control_loop(self):
        pose = self.get_current_pose()
        if pose is None:
            return
        cx, cy, cz, croll, cpitch, cyaw = pose

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return

        if self.path and self.path_index < len(self.path):
            target = self.path[self.path_index].pose.position
            dist = math.sqrt((target.x - cx) ** 2 + (target.y - cy) ** 2 + (target.z - cz) ** 2)
            if dist < self.get_parameter('goal_tolerance').value:
                self.path_index += 1
                if self.path_index >= len(self.path):
                    self.path_complete_pub.publish(Bool(data=True))
                    print("GOAL", flush=True)

            if self.path_index < len(self.path):
                target = self.path[self.path_index].pose.position
                sp_x, sp_y, sp_z = target.x, target.y, target.z
                # Celowo liczymy namiar na bieżąco (do aktualnego celu z aktualnej
                # pozycji), a nie bierzemy orientacji zapisanej w trasie przy jej
                # generowaniu - dzięki temu dziób (i czujniki) cały czas są
                # skierowane w stronę, w którą robot faktycznie płynie.
                sp_yaw = math.atan2(sp_y - cy, sp_x - cx)
            else:
                sp_x, sp_y, sp_z, sp_yaw = cx, cy, self.hold_z, self.hold_yaw

            force_x = self.pid_x.update(sp_x - cx, dt)
            force_y = self.pid_y.update(sp_y - cy, dt)
            force_in_body_frame = False
        else:
            self.path_complete_pub.publish(Bool(data=False))
            twist = self.last_cmd_vel
            stale = (now - self.last_cmd_vel_time) > Duration(seconds=self.get_parameter('cmd_vel_timeout').value)
            deadzone = self.get_parameter('xy_deadzone').value

            if stale or (abs(twist.linear.x) < deadzone and abs(twist.linear.y) < deadzone):
                if self.hold_x is None:
                    # Zapamiętaj aktualną pozycję I kurs w momencie wejścia w tryb
                    # trzymania pozycji - inaczej hold_yaw zostaje na domyślnym
                    # default_yaw (0.0) i robot obraca się z powrotem do zera
                    # świata zamiast trzymać kierunek, w którym właśnie płynął.
                    self.hold_x, self.hold_y = cx, cy
                    self.hold_yaw = cyaw
                force_x = self.pid_x.update(self.hold_x - cx, dt)
                force_y = self.pid_y.update(self.hold_y - cy, dt)
                force_in_body_frame = False
            else:
                self.hold_x = None
                self.hold_y = None
                self.pid_x.reset()
                self.pid_y.reset()
                scale = self.get_parameter('xy_force_scale').value
                # cmd_vel jest już w ramie ciała (konwencja teleopu) - nie obracać.
                force_x = twist.linear.x * scale
                force_y = twist.linear.y * scale
                force_in_body_frame = True

            if not stale:
                if abs(twist.linear.z) > deadzone:
                    self.hold_z -= twist.linear.z * 1.5 * dt
                    self.hold_z = max(0.1, self.hold_z)

                if abs(twist.angular.z) > deadzone:
                    self.hold_yaw += twist.angular.z * 1.0 * dt
                    self.hold_yaw = normalize_angle(self.hold_yaw)

            sp_z, sp_yaw = self.hold_z, self.hold_yaw

        force_z = self.pid_z.update(sp_z - cz, dt)

        # Aktywna stabilizacja - celem jest zawsze kąt 0.0 (poziom)
        torque_x = self.pid_roll.update(croll, dt)
        torque_y = self.pid_pitch.update(cpitch, dt)

        yaw_error = normalize_angle(sp_yaw - cyaw)
        torque_z = self.pid_yaw.update(yaw_error, dt)

        # force_x/force_y z PID-ów pozycji są policzone w osiach świata (błąd
        # pozycji X/Y), a thruster_allocator_node oczekuje siły w ramie ciała
        # robota - trzeba obrócić o bieżący yaw, inaczej robot pcha się w
        # kierunku świata zamiast przed siebie, nawet gdy dziób jest już
        # skierowany na cel. cmd_vel z teleopu jest już w ramie ciała.
        if force_in_body_frame:
            body_force_x, body_force_y = force_x, force_y
        else:
            body_force_x = force_x * math.cos(cyaw) + force_y * math.sin(cyaw)
            body_force_y = -force_x * math.sin(cyaw) + force_y * math.cos(cyaw)

        wrench_msg = Wrench()
        wrench_msg.force.x = body_force_x
        wrench_msg.force.y = body_force_y
        wrench_msg.force.z = force_z
        wrench_msg.torque.x = torque_x
        wrench_msg.torque.y = torque_y
        wrench_msg.torque.z = torque_z
        self.wrench_pub.publish(wrench_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()