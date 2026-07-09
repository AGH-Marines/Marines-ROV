import re
import socket
import random
import time
from struct import *

class Bridge:
    def __init__(self):
        self.__ip = None
        self.__port = None
        self.__socket = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    @property
    def ip(self):
        return self.__ip
    
    @ip.setter
    def ip(self, ip: str):
        if not re.fullmatch('^((25[0-5]|(2[0-4]|1[0-9]|[1-9]|)[0-9])(\.(?!$)|$)){4}$', ip):
            raise Exception(f"Wrong ip address")

        self.__ip = ip

    @property
    def port(self):
        return self.__port
    
    @port.setter
    def port(self, port):
        self.__port = port

    @property
    def sock(self) -> socket.socket:
        return self.__socket
    
    @sock.setter
    def sock(self, socket: socket.socket):
        self.__socket = socket

    def send(self, data):
        print(self.ip, self.port, self.sock)
        self.sock.sendto(data, (self.ip, self.port))

if __name__ == '__main__':

    # bridge = Bridge('192.168.69.100', 25565)
    bridge = Bridge()
    bridge.ip = '10.0.0.2'
    bridge.port = 5555

    while True:
        data = pack('ffffffff', 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,0.5,0.5)
        print(data)
        bridge.send(data=data)
        time.sleep(1)
        
