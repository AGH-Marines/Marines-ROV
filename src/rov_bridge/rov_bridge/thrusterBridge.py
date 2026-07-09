import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from std_msgs.msg import Float64MultiArray

import re
from struct import *
from rov_bridge.bridge import Bridge
from typing import Literal

InputType = Literal['PWM']

class ThrusterBridge(Bridge, Node):
    def __init__(self):
        Bridge.__init__(self=self)
        Node.__init__(self=self, node_name="thruster_bridge")

        self.__package_data_type = '<BB8f'

        self.declare_parameter('ip', "10.0.0.2")
        self.declare_parameter('port', 5555)
        self.declare_parameter('num_of_thrusters', 8)
        self.declare_parameter('queue', [x for x in range(self.num_of_thrusters)])
        self.declare_parameter('input_type', 'PWM' )
        self.declare_parameter('input_topic', 'thrusters/PWM')
        
        self.create_subscription(Float64MultiArray, self.input_topic, self.cb_input, 10)

    @property
    def ip(self):
        return self.get_parameter('ip').value
    
    @ip.setter
    def ip(self, value: str):
        if not re.fullmatch('^((25[0-5]|(2[0-4]|1[0-9]|[1-9]|)[0-9])(\.(?!$)|$)){4}$', value):
            raise Exception(f"Wrong ip address")
        
        self.set_parameters([Parameter('ip', Parameter.Type.STRING, value)])
        self.__ip = value

    @property
    def port(self):
        return self.get_parameter('port').value
    
    @port.setter
    def port(self, value):
        self.set_parameters([Parameter('port', Parameter.Type.INTEGER, value)])
        self.__port = value

    @property
    def num_of_thrusters(self):
        return self.get_parameter('num_of_thrusters').value
    
    @num_of_thrusters.setter
    def num_of_thrusters(self, value: int):
        self.set_parameters([Parameter('num_of_thrusters', Parameter.Type.INTEGER, value)])
        self.package_data_type = value

    @property
    def queue(self):
        return self.get_parameter('queue').value

    @queue.setter
    def queue(self, value: list[int]):
        if len(value) > self.num_of_thrusters:
            raise Exception('Too many thrusters in queue')
        
        if max(value) > self.num_of_thrusters - 1 or min(value) < 0:
            raise Exception(f"List {value} has values not in range {0}-{self.num_of_thrusters - 1}")

        self.set_parameters([Parameter('queue', Parameter.Type.INTEGER_ARRAY, value)])

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
            data.append((input_array[i] - 1500) / 400)

        return data
    
    def cb_input(self, msg: Float64MultiArray):
        
        data = self.rearrange(msg.data)
        data = pack(self.package_data_type, 2, 34,*data)
        
        self.send(data=data)