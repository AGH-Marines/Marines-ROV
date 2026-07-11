import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdometryConverterNode(Node):
    """
    Stonefish publishes /marines_rov/odometry in its native "down-positive"
    world convention (Z increases downward - the pool bottom in
    underwater_world.scn is at z=4.0, the ROV starts at z=1.0), while the rest
    of the stack (e.g. the default target depth of -2.0 in pid_controller_node)
    assumes Z-up (surface ~= 0, negative Z underwater). So we only convert the
    sign of the Z position.

    Orientation and twist are passed through unchanged: the orientation from
    odom0 isn't fused in ekf.yaml anyway (odom0_config has "false" for all of
    Roll/Pitch/Yaw), and child_frame_id is deliberately left different from
    "base_link", so the twist from this topic remains unused by the EKF - as
    before. Only the corrected position (X, Y, Z) is fused here.
    """

    def __init__(self):
        super().__init__('odometry_converter_node')
        self.sub = self.create_subscription(
            Odometry,
            '/marines_rov/odometry',
            self.odometry_callback,
            10
        )
        self.pub = self.create_publisher(
            Odometry,
            '/marines_rov/odometry_enu',
            10
        )
        self.get_logger().info('Odometry Converter Node (NED -> ENU, Z) started.')

    def odometry_callback(self, msg):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = 'odom'
        out.child_frame_id = msg.child_frame_id

        out.pose = msg.pose
        out.pose.pose.position.z = -msg.pose.pose.position.z

        out.twist = msg.twist

        # Stonefish never fills in the covariance (all zeros). The "odometry" sensor
        # in underwater_world.scn has an explicitly set position="0.05" noise (std.
        # dev. in meters, see Odometry::setNoise in Stonefish) - variance = 0.05^2.
        # NOTE: a variance of 0.0 does NOT mean "perfect measurement" - in a Kalman
        # filter that's R=0, i.e. "trust this measurement 100%, ignore the estimate".
        # With real noise on the input (0.05m) this causes violent, continuous
        # corrections (the EKF "jumps" on every noisy reading, 100x/s), and through
        # the cross-correlated state matrix it can throw off even the orientation,
        # even though we don't fuse it here.
        # pose_cov = list(out.pose.covariance)
        # for idx in (0, 7, 14):  # X, Y, Z
        #     pose_cov[idx] = 0
        out.pose.covariance = out.pose.covariance

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
