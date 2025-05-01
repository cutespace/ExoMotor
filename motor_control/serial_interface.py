import serial

import time
class SerialInterface:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    def connect(self):
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.3)
        self.ser.setDTR(True)
        self.ser.setRTS(False)

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def send(self, data: bytes) -> str:
        self.ser.reset_input_buffer()
        self.ser.write(data)
        time.sleep(0.05)  # 小等待
        deadline = time.time() + 1  # 最长等1秒
        response = bytearray()
        while time.time() < deadline:
            if self.ser.in_waiting:
                response += self.ser.read(self.ser.in_waiting)
                if len(response) >= 7:  # 最小完整 Modbus 响应
                    break
            time.sleep(0.01)
        return response.hex() if response else "无响应"

    def is_open(self):
        return self.ser and self.ser.is_open
