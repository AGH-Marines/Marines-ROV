import math
import rclpy
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
    def __init__(self, kp, ki, kd, out_limit=None, integral_tau: float | None =10.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.out_limit = out_limit
        self.integral_tau = integral_tau
        self.integral_limit = (out_limit / ki) if (out_limit is not None and ki) else None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):

        if dt <= 0.0:
            return 0.0
        decay = math.exp(-dt / self.integral_tau) if self.integral_tau else 1.0
        self.integral = self.integral * decay + error * dt
        if self.integral_limit is not None:
            self.integral = max(min(self.integral, self.integral_limit), -self.integral_limit)
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

        self.declare_parameter('path_topic', '/marines_rov/global_path')
        self.declare_parameter('publish_path_complete', True)
        self.declare_parameter('reset_pid_on_new_path', True)

        self.declare_parameter('forward_motion_only', True)
        self.declare_parameter('lateral_force_scale', 0.1)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_path_complete = self.get_parameter('publish_path_complete').value
        self.forward_motion_only = self.get_parameter('forward_motion_only').value
        self.lateral_force_scale = self.get_parameter('lateral_force_scale').value
        self.reset_pid_on_new_path = self.get_parameter('reset_pid_on_new_path').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.wrench_pub = self.create_publisher(Wrench, '/marines_rov/cmd_wrench', 10)
        self.path_complete_pub = self.create_publisher(Bool, '/marines_rov/path_complete', 10)

        self.create_subscription(Twist, '/marines_rov/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Path, self.get_parameter('path_topic').value, self.path_callback, 10)

        self.pid_x = PID(10.0, 0.3, 0.5, out_limit=60.0, integral_tau=3.0)
        self.pid_y = PID(7.0, 0.5, 1.0, out_limit=60.0, integral_tau=3.0)
        self.pid_z = PID(10.0, 0.4, 0.5, out_limit=60.0, integral_tau=None)

        self.pid_roll = PID(15.0, 0.5, 2.0, out_limit=40.0, integral_tau=None)
        self.pid_pitch = PID(15.0, 0.5, 2.0, out_limit=40.0, integral_tau=None)
        self.pid_yaw = PID(3.0, 0.05, 0.9, out_limit=30.0, integral_tau=None)

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

        self.get_logger().info('PID Controller Node (with Active 6-DOF Stabilization) started.')

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.path_index = 0

        if self.reset_pid_on_new_path:
            self.pid_x.reset()
            self.pid_y.reset()

            self.hold_x = None
            self.hold_y = None
            self.get_logger().info(f'New path: {len(self.path)} points.')

    def get_current_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'TF {self.map_frame}->{self.base_frame} unavailable.', throttle_duration_sec=2.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return t.x, t.y, t.z, roll, pitch, yaw

    def control_loop(self):
        # self.get_logger().info("--- START control_loop ---")

        pose = self.get_current_pose()
        if pose is None:
            # self.get_logger().info("BŁĄD/POMINIĘCIE: pose is None")
            return
        cx, cy, cz, croll, cpitch, cyaw = pose
        # self.get_logger().info(
         #   f"POZA ROBOTA: x={cx:.3f}, y={cy:.3f}, z={cz:.3f}, roll={croll:.3f}, pitch={cpitch:.3f}, yaw={cyaw:.3f}")

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # self.get_logger().info(f"CZAS: dt={dt:.5f}s")
        if dt <= 0.0:
            # self.get_logger().info("POMINIĘCIE: dt <= 0.0")
            return

        # Wskaźnik, w którym trybie aktualnie działa pętla
        mode = "UNKNOWN"

        if self.path:
            mode = "PATH_FOLLOWING"

            if self.path_index < len(self.path):
                target = self.path[self.path_index].pose.position
                dist = math.sqrt((target.x - cx) ** 2 + (target.y - cy) ** 2 + (target.z - cz) ** 2)
                goal_tol = self.get_parameter('goal_tolerance').value

                # self.get_logger().info(
                #    f"TRYB: ŚCIEŻKA (punkt {self.path_index + 1}/{len(self.path)}) | Cel: x={target.x:.2f}, y={target.y:.2f}, z={target.z:.2f}")
                # self.get_logger().info(f"ŚCIEŻKA: dystans={dist:.3f}, tolerancja={goal_tol:.3f}")

                if dist < goal_tol:
                    self.path_index += 1
                    # self.get_logger().info(f"ŚCIEŻKA: Osiągnięto punkt! Zwiększam path_index na {self.path_index}")
                    if self.path_index >= len(self.path):
                        self.hold_z = cz
                        self.hold_yaw = cyaw
                        if self.publish_path_complete:
                            self.get_logger().info("ŚCIEŻKA: Zakończono całą ścieżkę. Publikuję path_complete=True")
                            self.path_complete_pub.publish(Bool(data=True))

            # Zawsze celujemy w dokładną pozycję ostatniego znanego punktu ścieżki
            # (bieżącego albo, gdy już wszystkie osiągnięte, ostatniego) zamiast
            # podstawiać sp_x/sp_y=cx/cy w momencie "osiągnięcia" - to drugie
            # zerowało błąd (i całą siłę napędową) natychmiast po wejściu w
            # tolerancję, więc robot zatrzymywał się tuż PRZED właściwym celem
            # (np. gdy przez odpychanie APF karotka nie pokrywa się dokładnie
            # z celem) i nigdy go realnie nie osiągał - stąd "Osiągnięto punkt!"
            # w kółko bez końca, a local_planner/MonitorMission (własne, ciaśniejsze
            # tolerancje) nigdy nie widziały sukcesu.
            target = self.path[min(self.path_index, len(self.path) - 1)].pose.position
            sp_x, sp_y, sp_z = target.x, target.y, target.z
            horiz_dist = math.hypot(sp_x - cx, sp_y - cy)
            sp_yaw = math.atan2(sp_y - cy, sp_x - cx) if horiz_dist > 0.05 else cyaw

            # Obliczenie uchybu globalnego
            err_x_world = sp_x - cx
            err_y_world = sp_y - cy

            # 1. Transformacja UCHYBU do układu robota (Body Frame)
            err_x_body = err_x_world * math.cos(cyaw) + err_y_world * math.sin(cyaw)
            err_y_body = -err_x_world * math.sin(cyaw) + err_y_world * math.cos(cyaw)

            # 2. PIDy liczą na uchybie lokalnym - od razu wypluwają siły w Body Frame!
            force_x = self.pid_x.update(err_x_body, dt)
            force_y = self.pid_y.update(err_y_body, dt)
            force_in_body_frame = True

            # self.get_logger().info(
             #   f"PID (World Frame): sp_x={sp_x:.3f}, sp_y={sp_y:.3f} -> force_x={force_x:.3f}, force_y={force_y:.3f}")

        else:
            mode = "TELEOP_OR_HOLD"
            # self.get_logger().info(f"TRYB: TELEOP/HOLD | Opublikować zakończenie? {self.publish_path_complete}")
            if self.publish_path_complete:
                self.path_complete_pub.publish(Bool(data=False))

            twist = self.last_cmd_vel
            stale = (now - self.last_cmd_vel_time) > Duration(seconds=self.get_parameter('cmd_vel_timeout').value)
            deadzone = self.get_parameter('xy_deadzone').value

            # self.get_logger().info(f"TELEOP: stale={stale}, deadzone={deadzone:.3f}")
            # self.get_logger().info(
              #  f"TELEOP CMD_VEL: lin.x={twist.linear.x:.3f}, lin.y={twist.linear.y:.3f}, lin.z={twist.linear.z:.3f}, ang.z={twist.angular.z:.3f}")

            if stale or (abs(twist.linear.x) < deadzone and abs(twist.linear.y) < deadzone):
                if self.hold_x is None:
                    self.hold_x, self.hold_y = cx, cy
                    self.hold_yaw = cyaw
                    # self.get_logger().info(
                     #   f"TELEOP: Zapisuję nową pozycję HOLD: x={self.hold_x:.3f}, y={self.hold_y:.3f}, yaw={self.hold_yaw:.3f}")

                err_x_world = self.hold_x - cx
                err_y_world = self.hold_y - cy

                # Transformacja uchybu
                err_x_body = err_x_world * math.cos(cyaw) + err_y_world * math.sin(cyaw)
                err_y_body = -err_x_world * math.sin(cyaw) + err_y_world * math.cos(cyaw)

                force_x = self.pid_x.update(err_x_body, dt)
                force_y = self.pid_y.update(err_y_body, dt)
                force_in_body_frame = True
                # self.get_logger().info(
                 #   f"TELEOP (Hold Pos): błąd_x={(self.hold_x - cx):.3f}, błąd_y={(self.hold_y - cy):.3f} -> force_x={force_x:.3f}, force_y={force_y:.3f}")
            else:
                self.hold_x = None
                self.hold_y = None
                self.pid_x.reset()
                self.pid_y.reset()
                scale = self.get_parameter('xy_force_scale').value
                force_x = twist.linear.x * scale
                force_y = twist.linear.y * scale
                force_in_body_frame = True
                # self.get_logger().info(
                 #   f"TELEOP (Ruch): PIDy zresetowane. scale={scale:.3f} -> force_x={force_x:.3f}, force_y={force_y:.3f}")

            if not stale:
                if abs(twist.linear.z) > deadzone:
                    self.hold_z -= twist.linear.z * 1.5 * dt
                    self.hold_z = max(0.1, self.hold_z)
                    # self.get_logger().info(f"TELEOP: Aktualizacja hold_z={self.hold_z:.3f}")

                if abs(twist.angular.z) > deadzone:
                    self.hold_yaw += twist.angular.z * 1.0 * dt
                    self.hold_yaw = normalize_angle(self.hold_yaw)
                    # self.get_logger().info(f"TELEOP: Aktualizacja hold_yaw={self.hold_yaw:.3f}")

            sp_z, sp_yaw = self.hold_z, self.hold_yaw

        force_z = self.pid_z.update(sp_z - cz, dt)
        # self.get_logger().info(f"PID Z: sp_z={sp_z:.3f}, cz={cz:.3f}, błąd={(sp_z - cz):.3f} -> force_z={force_z:.3f}")

        torque_x = self.pid_roll.update(croll, dt)
        torque_y = self.pid_pitch.update(cpitch, dt)
        # self.get_logger().info(
          #  f"PID ROLL/PITCH: croll={croll:.3f} -> torque_x={torque_x:.3f} | cpitch={cpitch:.3f} -> torque_y={torque_y:.3f}")

        yaw_error = normalize_angle(sp_yaw - cyaw)
        torque_z = self.pid_yaw.update(yaw_error, dt)
        # self.get_logger().info(
           # f"PID YAW: sp_yaw={sp_yaw:.3f}, cyaw={cyaw:.3f}, błąd={yaw_error:.3f} -> torque_z={torque_z:.3f}")

        # Konwersja układów współrzędnych
        # self.get_logger().info(f"RAMA ODNIESIENIA: force_in_body_frame={force_in_body_frame}")
        if force_in_body_frame:
            body_force_x, body_force_y = force_x, force_y
        else:
            body_force_x = force_x * math.cos(cyaw) + force_y * math.sin(cyaw)
            body_force_y = -force_x * math.sin(cyaw) + force_y * math.cos(cyaw)
            # self.get_logger().info(f"TRANSFORMACJA: (World -> Body) x={body_force_x:.3f}, y={body_force_y:.3f}")

            if getattr(self, 'forward_motion_only', False):
                heading_gate = max(0.0, math.cos(yaw_error))
                body_force_x *= heading_gate
                body_force_y *= self.lateral_force_scale
                # self.get_logger().info(
                  #  f"FORWARD_MOTION_ONLY: gate={heading_gate:.3f}, lateral_scale={self.lateral_force_scale:.3f} -> body_x={body_force_x:.3f}, body_y={body_force_y:.3f}")

        wrench_msg = Wrench()
        wrench_msg.force.x = float(body_force_x)
        wrench_msg.force.y = float(body_force_y)
        wrench_msg.force.z = float(force_z)
        wrench_msg.torque.x = float(torque_x)
        wrench_msg.torque.y = float(torque_y)
        wrench_msg.torque.z = float(torque_z)

        # self.get_logger().info(
         #   f"PUBLISH WRENCH -> F: [{wrench_msg.force.x:.2f}, {wrench_msg.force.y:.2f}, {wrench_msg.force.z:.2f}], "
         #   f"T: [{wrench_msg.torque.x:.2f}, {wrench_msg.torque.y:.2f}, {wrench_msg.torque.z:.2f}]")
        # self.get_logger().info("--- KONIEC control_loop ---\n")

        self.wrench_pub.publish(wrench_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()