import rclpy

from rov_bridge.thrusterBridge import ThrusterBridge

def main():
    rclpy.init()

    thrusterBridge = ThrusterBridge()

    rclpy.spin(thrusterBridge)

    thrusterBridge.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()