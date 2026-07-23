import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, Wrench


class ManualWrenchNode(Node):
    """Open-loop joystick -> thruster force converter.

    Fallback for when pid_controller_node's TF lookup (odom -> base_link,
    fed by the ZED/EKF localization stack) is unavailable or misbehaving:
    this node does no TF lookup, no odometry, and no depth/attitude
    stabilization - it just scales /marines_rov/cmd_vel straight into
    /marines_rov/cmd_wrench every cycle, same topics pid_controller_node
    uses so it's a drop-in swap in rov_actuation's thruster_allocator input.
    """

    def __init__(self):
        super().__init__('manual_wrench_node')

        self.declare_parameter('xy_force_scale', 40.0)
        self.declare_parameter('z_force_scale', 40.0)
        self.declare_parameter('yaw_torque_scale', 30.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('control_rate', 20.0)

        self.xy_scale = self.get_parameter('xy_force_scale').value
        self.z_scale = self.get_parameter('z_force_scale').value
        self.yaw_scale = self.get_parameter('yaw_torque_scale').value
        self.timeout = Duration(seconds=self.get_parameter('cmd_vel_timeout').value)

        self.last_cmd_vel = Twist()
        self.last_cmd_vel_time = self.get_clock().now()

        self.wrench_pub = self.create_publisher(Wrench, '/marines_rov/cmd_wrench', 10)
        self.create_subscription(Twist, '/marines_rov/cmd_vel', self.cmd_vel_callback, 10)

        period = 1.0 / self.get_parameter('control_rate').value
        self.create_timer(period, self.publish_wrench)

        self.get_logger().warn(
            'Manual Wrench Node started: RAW joystick control, NO odometry, '
            'NO stabilization (roll/pitch/depth/yaw hold all disabled). '
            'Use only as a fallback when the localization stack is down.')

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    def publish_wrench(self):
        stale = (self.get_clock().now() - self.last_cmd_vel_time) > self.timeout
        twist = Twist() if stale else self.last_cmd_vel

        wrench = Wrench()
        wrench.force.x = twist.linear.x * self.xy_scale
        wrench.force.y = twist.linear.y * self.xy_scale
        wrench.force.z = twist.linear.z * self.z_scale
        wrench.torque.z = twist.angular.z * self.yaw_scale
        self.wrench_pub.publish(wrench)


def main(args=None):
    rclpy.init(args=args)
    node = ManualWrenchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
