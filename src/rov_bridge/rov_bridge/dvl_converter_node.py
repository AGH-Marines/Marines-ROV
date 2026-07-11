import rclpy
from rclpy.node import Node
from stonefish_ros2.msg import DVL
from geometry_msgs.msg import TwistWithCovarianceStamped


class DvlConverterNode(Node):
    def __init__(self):
        super().__init__('dvl_converter_node')
        # Subscribe to the raw data from Stonefish
        self.sub = self.create_subscription(
            DVL,
            '/marines_rov/dvl_raw',
            self.dvl_callback,
            10
        )
        # Publish data understood by the EKF filter
        self.pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/marines_rov/dvl',
            10
        )
        self.get_logger().info('DVL Converter Node started.')

    def dvl_callback(self, msg):
        twist_msg = TwistWithCovarianceStamped()

        # Copy the header (very important for the EKF so the timing lines up)
        twist_msg.header = msg.header
        twist_msg.header.frame_id = 'base_link'  # DVL velocities are usually relative to base_link

        # Copy over the linear velocities
        twist_msg.twist.twist.linear.x = msg.velocity.x
        twist_msg.twist.twist.linear.y = msg.velocity.y
        twist_msg.twist.twist.linear.z = msg.velocity.z

        # Set a baseline velocity covariance (the smaller the value, the higher the confidence)
        # Indices: 0 (x), 7 (y), 14 (z)
        twist_msg.twist.covariance[0] = 0.005
        twist_msg.twist.covariance[7] = 0.005
        twist_msg.twist.covariance[14] = 0.005

        self.pub.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DvlConverterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()