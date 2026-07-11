import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from rov_actuation.robot_model import load_thrusters


class ThrusterTestNode(Node):
    def __init__(self):
        super().__init__('thruster_test_node')

        self.declare_parameter('robot_model_path', '')
        self.declare_parameter('topic', '/thrusters/STONEFISH')
        self.declare_parameter('thrust_value', 0.3)
        self.declare_parameter('duration', 3.0)

        model_path = self.get_parameter('robot_model_path').value or None
        self.thrusters = load_thrusters(model_path)
        self.thrust_value = self.get_parameter('thrust_value').value
        self.index = 0

        topic = self.get_parameter('topic').value
        self.pub = self.create_publisher(Float64MultiArray, topic, 10)

        self.get_logger().info(
            f'Thruster test: {len(self.thrusters)} thrusters found, publishing to '
            f'"{topic}", {self.get_parameter("duration").value}s each.')

        self._publish_current()
        self.timer = self.create_timer(self.get_parameter('duration').value, self._advance)

    def _zero_data(self):
        return [0.0] * len(self.thrusters)

    def _publish_current(self):
        data = self._zero_data()
        thruster = self.thrusters[self.index]
        data[self.index] = self.thrust_value
        self.get_logger().info(
            f'Testing thruster {self.index + 1}/{len(self.thrusters)}: {thruster.name}')
        self.pub.publish(Float64MultiArray(data=data))

    def _advance(self):
        self.index += 1
        if self.index >= len(self.thrusters):
            self.get_logger().info('Thruster test complete, all thrusters stopped.')
            self.pub.publish(Float64MultiArray(data=self._zero_data()))
            self.timer.cancel()
            return
        self._publish_current()

    def destroy_node(self):
        self.pub.publish(Float64MultiArray(data=self._zero_data()))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
