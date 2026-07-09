import time
from struct import pack
import serial


class Bridge:
    def __init__(self):
        self.__port = None
        self.__baudrate = 578000
        self.__serial = None

    @property
    def port(self):
        return self.__port

    @port.setter
    def port(self, port: str):
        self.__port = port
        self.__serial = serial.Serial(
            port=self.__port,
            baudrate=self.__baudrate,
            timeout=1
        )

    @property
    def ser(self) -> serial.Serial:
        return self.__serial

    def send(self, data: bytes):
        if self.__serial is None:
            raise Exception("Serial not initialized")
        self.__serial.write(data)


if __name__ == '__main__':

    bridge = Bridge()
    bridge.port = '/dev/ttyTHS1'

    time.sleep(2)

    while True:
        data = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        data = pack('<BB6f', 2, 32, *data)
        bridge.send(data)
        print("Wysłano:", data)
        time.sleep(1)