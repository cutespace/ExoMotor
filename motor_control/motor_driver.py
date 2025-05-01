import time
from .modbus_util import build_modbus_write_single

class MotorDriver:
    def __init__(self, serial_interface, ctrl_addr=0x3100):
        self.si = serial_interface
        self.addr = ctrl_addr

    def enable_motor(self):
        # steps = [0x0080, 0x0006, 0x0007, 0x000F]
        steps = [0x000F]
        for val in steps:
            frame = build_modbus_write_single(1, self.addr, val)
            print(f"写入控制字: {val:04X} -> {self.si.send(frame)}")
            time.sleep(0.5)

    def disable_motor(self):
        frame = build_modbus_write_single(1, self.addr, 0x0006)
        print(f"取消使能 -> {self.si.send(frame)}")
    
    # def set_position_mode(self):
    #     """设置工作模式为位置模式（1）"""
    #     frame = build_modbus_write_single(1, 0x3500, 0x0001)
    #     response = self.si.send(frame)
    #     print(f"设置工作模式为位置模式 -> {response}")
    #     time.sleep(0.2)  # 等待电机内部模式切换
    #     self.check_active_mode()  # <--- 添加在这里       
    def set_position_mode(self):
        print("准备进入位置模式...")
        # 先清除故障并进入就绪状态
        for ctrl in (0x0080, 0x0006):
            frame = build_modbus_write_single(1, 0x3100, ctrl)
            print(f"写入控制字 {ctrl:04X} -> {self.si.send(frame)}")
            time.sleep(0.2)

        # 写入工作模式 1
        frame = build_modbus_write_single(1, 0x3500, 0x0001)
        print(f"设置工作模式为位置模式 -> {self.si.send(frame)}")
        time.sleep(0.5)

        # 验证实际模式
        self.check_active_mode()
         
    def run_position_mode(self, pos: int, speed: int, acc: int, dec: int):
        """设置位置模式运行参数并启动"""

        param_list = [
            (0x4000, pos, 4),     # 目标位置
            (0x4A00, speed, 4),   # 
            (0x4B00, acc, 4),     # 
            (0x4C00, dec, 4),     # 
        ]

        for addr, value, length in param_list:
            frame = build_modbus_write_single(1, addr, value, length)
            print(f"写入 {addr:#06x} <- {value} -> {self.si.send(frame)}")
            time.sleep(0.1)

        # 连续触发： 启动位置运动
        for ctrl_word in (0x0006, 0x000F,0x002F, 0x003F):
            frame = build_modbus_write_single(1, 0x3100, ctrl_word)
            print(f"启动控制字 {ctrl_word:04X} -> {self.si.send(frame)}")
            time.sleep(0.1)
        status = self.read_status_word()
        if status is not None:
            if not (status & 0x004F):
                print("⚠️ 状态字未进入期望状态，电机可能未使能或未准备好运动")
            else:
                print("✅ 状态正常，可进入运动")

    def read_status_word(self):
        from .modbus_util import build_modbus_read_cmd
        cmd = build_modbus_read_cmd(1, 0x3200, 1)
        response = self.si.send(cmd)
        print(f"读取状态字 -> {response}")
        if len(response) >= 18 and response != "无响应":
            try:
                value = int.from_bytes(bytes.fromhex(response[6:10]), byteorder='big')
                print(f"状态字值: 0x{value:04X}")
                return value
            except Exception as e:
                print(f"状态解析失败: {e}")
        return None
    def check_active_mode(self):
        """读取 0x3600 有效工作模式，返回当前模式编号（0: 停止, 1: 位置, 2: 速度, 3: 力矩）"""
        from .modbus_util import build_modbus_read_cmd
        cmd = build_modbus_read_cmd(1, 0x3600, 1)
        response = self.si.send(cmd)
        print(f"读取有效工作模式 -> {response}")
        if len(response) >= 18 and response != "无响应":
            try:
                data_bytes = bytes.fromhex(response[10:14])  # ← 从第6字节开始取 2 字节内容
                value = int.from_bytes(data_bytes, byteorder='big')
                print(f"当前有效工作模式: {value}（0=停止, 1=位置, 2=速度, 3=力矩）")
                return value
            except Exception as e:
                print(f"解析失败: {e}")
        return None

