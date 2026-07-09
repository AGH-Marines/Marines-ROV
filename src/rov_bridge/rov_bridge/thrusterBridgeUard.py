import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Float64MultiArray

from struct import *
import serial
from typing import Literal

InputType = Literal['PWM']


class ThrusterBridge(Node):
    def __init__(self):
        super().__init__("thruster_bridge")

        self.__package_data_type = '<BB8f'

        # 🔧 UART zamiast IP
        self.declare_parameter('serial_port', '/dev/ttyTHS1')
        self.declare_parameter('baudrate', 578000)

        self.declare_parameter('num_of_thrusters', 8)
        self.declare_parameter('queue',[5,1,3,7,6,2,4,8])
        self.declare_parameter('input_type', 'PWM')
        self.declare_parameter('input_topic', 'thrusters/PWM')

        # 🔧 init UART
        self.ser = serial.Serial(
            port=self.serial_port,
            baudrate=self.baudrate,
            timeout=1
        )

        self.create_subscription(
            Float64MultiArray,
            self.input_topic,
            self.cb_input,
            10
        )

    # ================= UART =================

    @property
    def serial_port(self):
        return self.get_parameter('serial_port').value

    @property
    def baudrate(self):
        return self.get_parameter('baudrate').value

    def send(self, data: bytes):
        self.ser.write(data)


    @property
    def num_of_thrusters(self):
        return self.get_parameter('num_of_thrusters').value

    @property
    def queue(self):
        return self.get_parameter('queue').value

    @property
    def input_type(self):
        return self.get_parameter('input_type').value

    @property
    def input_topic(self):
        return self.get_parameter('input_topic').value

    @property
    def package_data_type(self):
        return self.__package_data_type

    @package_data_type.setter
    def package_data_type(self, value):
        self.__package_data_type = ''.join(['i' for _ in range(value)])

    def rearrange(self, input_array: list):
        data = []
        for i in self.queue:

            v = (input_array[i-1] - 775) / 225 * 0.5
            if v < -0.4:
                v = -0.4
            if v > 0.4:
                v = 0.4
            data.append(v)
        return data

    def cb_input(self, msg: Float64MultiArray):
        data = self.rearrange(msg.data)

        # dalej masz swój protokół
        print(data)
        data = pack('<BB8f', 2, 32, *data)
        self.send(data)