import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class RovTeleop(Node):
    def __init__(self):
        super().__init__('rov_teleop')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, '/marines_rov/cmd_vel', 10)

    def joy_callback(self, msg):
        twist = Twist()

        # Sterowanie przód/tył, lewo/prawo z gałek (np. osie 1 i 0)
        twist.linear.x = msg.axes[1]
        twist.linear.y = -msg.axes[0]
        twist.angular.z = -msg.axes[2]

        # --- ZANURZANIE PRZYCISKAMI (Trójkąt i X) ---
        # Zakładając, że Trójkąt to msg.buttons[2], a X to msg.buttons[0]
        if msg.buttons[2] == 1:
            twist.linear.z = 1.0  # Wynurzanie
        elif msg.buttons[0] == 1:
            twist.linear.z = -1.0  # Zanurzanie
        else:
            twist.linear.z = 0.0  # Brak ruchu

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = RovTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()