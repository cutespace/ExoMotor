def modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, 'little')

# def build_modbus_write_single(slave_id: int, register: int, value: int) -> bytes:
#     cmd = bytearray([slave_id, 0x06])
#     cmd += register.to_bytes(2, 'big')
#     cmd += value.to_bytes(2, 'big')
#     cmd += modbus_crc(cmd)
#     return cmd
def build_modbus_write_single(slave_id: int, register: int, value: int, length=2) -> bytes:
    """可支持16位或32位写入"""
    cmd = bytearray([slave_id, 0x10 if length == 4 else 0x06])
    cmd += register.to_bytes(2, 'big')
    if length == 2:
        cmd += value.to_bytes(2, 'big')
        cmd += modbus_crc(cmd)
    else:  # 32位寄存器
        cmd += (2).to_bytes(2, 'big')  # 写入2个寄存器
        cmd += (4).to_bytes(1, 'big')  # 字节数
        cmd += value.to_bytes(4, 'little', signed=True)
        cmd += modbus_crc(cmd)
    return cmd
def build_modbus_read_cmd(slave_id: int, register: int, length: int) -> bytes:
    cmd = bytearray([slave_id, 0x03])
    cmd += register.to_bytes(2, 'big')
    cmd += length.to_bytes(2, 'big')
    cmd += modbus_crc(cmd)
    return cmd


# File: motor_control/serial_interface.py

import serial

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
        return self.ser.read_all().hex() if self.ser.in_waiting else "无响应"

    def is_open(self):
        return self.ser and self.ser.is_open