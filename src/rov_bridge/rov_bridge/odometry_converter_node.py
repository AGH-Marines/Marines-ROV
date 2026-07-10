import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdometryConverterNode(Node):
    """
    Stonefish publikuje /marines_rov/odometry w swojej natywnej, "zejściowej"
    konwencji świata (Z rośnie w dół - dno basenu w underwater_world.scn jest na
    z=4.0, ROV startuje na z=1.0), a reszta stosu (m.in. domyślna głębokość
    docelowa -2.0 w pid_controller_node) zakłada Z w górę (powierzchnia ~= 0,
    pod wodą Z ujemne). Konwertujemy więc tylko znak Z pozycji.

    Orientację i twist przepisujemy bez zmian: orientacja z odom0 i tak nie jest
    fuzowana w ekf.yaml (odom0_config ma same "false" dla Roll/Pitch/Yaw), a
    child_frame_id celowo zostawiamy inne niż "base_link", więc twist z tego
    tematu pozostaje nieużywany przez EKF - tak jak dotychczas. Fuzowana jest
    tu wyłącznie skorygowana pozycja (X, Y, Z).
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
        self.get_logger().info('Węzeł Konwertera Odometrii (NED -> ENU, Z) uruchomiony.')

    def odometry_callback(self, msg):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = 'odom'
        out.child_frame_id = msg.child_frame_id

        out.pose = msg.pose
        out.pose.pose.position.z = -msg.pose.pose.position.z

        out.twist = msg.twist

        # Stonefish nigdy nie wypełnia kowariancji (same zera). Sensor "odometry"
        # w underwater_world.scn ma jawnie ustawiony szum position="0.05" (odchylenie
        # std. w metrach, patrz Odometry::setNoise w Stonefish) - wariancja = 0.05^2.
        # UWAGA: wariancja 0.0 nie oznacza "idealny pomiar" - w Kalman filterze to
        # R=0, czyli "ufaj temu pomiarowi w 100%, olej estymatę". Przy realnym szumie
        # na wejściu (0.05m) to powoduje gwałtowne, ciągłe korekcje (EKF "skacze" co
        # każdy zaszumiony odczyt, 100x/s), a przez współ-skorelowaną macierz stanu
        # potrafi to rozjechać nawet orientację, mimo że jej tu nie fuzujemy.
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
