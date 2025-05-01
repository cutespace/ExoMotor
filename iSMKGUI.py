#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time  # 如果文件顶部还没 import，就添加这一行

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import struct

SLAVE_ID = 0x01
BAUDRATE = 115200
TIMEOUT  = 0.5
POSITION_FACTOR = 65536  # 65536 pulses per revolution
ACCEL_FACTOR = POSITION_FACTOR**2 / 4_000_000  # for rps/s → raw
def calc_crc16(data: bytes) -> bytes:
    """计算 Modbus‑RTU CRC16，返回 low-byte + high-byte"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)  # little‑endian：low, high

def build_frame_single(register: int, value: int) -> bytes:
    """功能码 0x06：写单个寄存器"""
    payload = struct.pack('>B B H H',
                          SLAVE_ID,
                          0x06,
                          register,
                          value)
    return payload + calc_crc16(payload)

def build_frame_multiple(register: int, values: list[int]) -> bytes:
    """功能码 0x10：写多个寄存器。values 是 16bit 寄存器列表。"""
    n_regs = len(values)
    byte_count = n_regs * 2
    header = struct.pack('>B B H H B',
                         SLAVE_ID,
                         0x10,
                         register,
                         n_regs,
                         byte_count)
    data_bytes = b''.join(struct.pack('>H', v) for v in values)
    frame = header + data_bytes
    return frame + calc_crc16(frame)
def build_read_frame(register: int, count: int) -> bytes:
    """Function 0x03: read `count` registers starting at `register`."""
    payload = struct.pack('>B B H H',
                          SLAVE_ID,
                          0x03,
                          register,
                          count)
    return payload + calc_crc16(payload)
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Kinco Modbus 控制")
        self.serial_port: serial.Serial | None = None

        # --- UI 布局 ---
        frm = ttk.Frame(self, padding=10)
        frm.grid()

        # 串口选择
        ttk.Label(frm, text="串口:").grid(row=0, column=0, sticky='e')
        self.cb_ports = ttk.Combobox(frm, width=15, state='readonly')
        self.cb_ports['values'] = [p.device for p in serial.tools.list_ports.comports()]
        self.cb_ports.grid(row=0, column=1, padx=5)
        ttk.Button(frm, text="刷新", command=self.refresh_ports).grid(row=0, column=2)
        self.btn_conn = ttk.Button(frm, text="连接", command=self.toggle_connection)
        self.btn_conn.grid(row=0, column=3, padx=5)

        # 操作按钮
        ops = [
            ("使能", self.enable_motor),
            ("取消使能", self.disable_motor),
            ("位置模式", self.set_position_mode),
            ("启动运动", self.start_motion),
        ]
        for i, (txt, cmd) in enumerate(ops, start=1):
            ttk.Button(frm, text=txt, command=cmd, width=12)\
               .grid(row=i, column=0, columnspan=2, pady=2)

        # 目标位置
        ttk.Label(frm, text="目标位置:").grid(row=1, column=2, sticky='e')
        self.ent_pos = ttk.Entry(frm, width=12)
        self.ent_pos.insert(0, "50000")
        self.ent_pos.grid(row=1, column=3)

        ttk.Button(frm, text="写入位置", command=self.write_position)\
            .grid(row=2, column=2, columnspan=2, pady=2)

        # 梯形速度
        ttk.Label(frm, text="速度 RPM:").grid(row=3, column=2, sticky='e')
        self.ent_spd = ttk.Entry(frm, width=12)
        self.ent_spd.insert(0, "200")
        self.ent_spd.grid(row=3, column=3)
        ttk.Button(frm, text="写入速度", command=self.write_speed).grid(row=4, column=2, columnspan=2, pady=2)
        # 梯形加速度
        ttk.Label(frm, text="加速度(rps/s):").grid(row=5, column=2, sticky='e')
        self.ent_acc = ttk.Entry(frm, width=12)
        self.ent_acc.insert(0, "300")
        self.ent_acc.grid(row=5, column=3)
        ttk.Button(frm, text="写入加速度", command=self.write_accel).grid(row=6, column=2, columnspan=2, pady=2)

        # 梯形减速度
        ttk.Label(frm, text="减速度(rps/s):").grid(row=7, column=2, sticky='e')
        self.ent_dec = ttk.Entry(frm, width=12)
        self.ent_dec.insert(0, "300")
        self.ent_dec.grid(row=7, column=3)
        ttk.Button(frm, text="写入减速度", command=self.write_decel).grid(row=8, column=2, columnspan=2, pady=2)
        # 日志区
        self.txt_log = tk.Text(frm, width=50, height=8, state='disabled')
        self.txt_log.grid(row=5, column=0, columnspan=4, pady=10)
        # “位置测试”按钮改成启动非阻塞测试
        ttk.Button(frm, text="位置测试", command=self.position_test_start)\
            .grid(row=5, column=0, columnspan=2, pady=2)
        # 日志区移到 row=6
        self.txt_log.grid(row=6, column=0, columnspan=4, pady=10)
        # --- 实际位置显示 & 刷新 ---
        ttk.Label(frm, text="实际位置:").grid(row=9, column=2, sticky='e')
        self.lbl_actual = ttk.Label(frm, text="--")
        self.lbl_actual.grid(row=9, column=3)
        ttk.Button(frm, text="刷新位置", command=self.read_actual_position)\
            .grid(row=10, column=0, columnspan=2, pady=2)
        # 用于测试的内部状态
        self._test_seq = []     # 存放 (pulse_count) 的列表
        self._test_idx = 0      # 当前执行到哪一步
        self._test_running = False
        
        self.prev_raw_u = None       # 上一次读取的无符号 raw
        self.total_counts = 0        # 累计的增量（无符号计数增量，可能为负数）

    def log(self, msg: str):
        """在日志区追加一行"""
        self.txt_log['state'] = 'normal'
        self.txt_log.insert('end', msg + '\n')
        self.txt_log.see('end')
        self.txt_log['state'] = 'disabled'

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cb_ports['values'] = ports

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None
            self.btn_conn.config(text="连接")
            self.log("已断开串口")
        else:
            port = self.cb_ports.get()
            if not port:
                messagebox.showwarning("警告", "请选择串口")
                return
            try:
                self.serial_port = serial.Serial(port, BAUDRATE, timeout=TIMEOUT)
                self.btn_conn.config(text="断开")
                self.log(f"已连接 {port}")
            except Exception as e:
                messagebox.showerror("错误", f"无法打开串口: {e}")

    def send_frame(self, frame: bytes):
        if not (self.serial_port and self.serial_port.is_open):
            messagebox.showwarning("警告", "请先打开串口")
            return
        self.serial_port.write(frame)
        self.log(">> " + frame.hex(' ').upper())

    # ---- 各功能实现 ----
    def enable_motor(self):
        """写 0x3100 = 0x000F"""
        frm = build_frame_single(0x3100, 0x000F)
        self.send_frame(frm)

    def disable_motor(self):
        """写 0x3100 = 0x0006"""
        frm = build_frame_single(0x3100, 0x0006)
        self.send_frame(frm)

    def set_position_mode(self):
        """写 0x3500 = 0x0001"""
        frm = build_frame_single(0x3500, 0x0001)
        self.send_frame(frm)

    def write_position(self):
        """
        写入“目标位置”：
        - 界面输入带小数的转数（float），
        - 乘以 POSITION_FACTOR 得到整数脉冲，
        - 按 LSW→MSW 顺序打包成 Modbus 0x10 报文并发送。
        """
        text = self.ent_pos.get().strip()
        try:
            # 用户输入带小数的“转数”
            turns = float(text)
        except ValueError:
            messagebox.showerror("错误", f"“目标位置”请输入数值，例如 1.23")
            return

        # 换算成整数脉冲
        pulse_count = int(turns * POSITION_FACTOR)

        # 拆成两个 16-bit 寄存器：低字低位在前，高字在后
        lo = pulse_count & 0xFFFF
        hi = (pulse_count >> 16) & 0xFFFF

        # 0x4000: 目标位置对象地址，写 2 个寄存器
        frame = build_frame_multiple(0x4000, [lo, hi])
        self.send_frame(frame)
        self.log(f"写入位置: {turns} 转 → {pulse_count} 脉冲")

    def write_speed(self):
        """写 0x4A00 = 梯形速度 (RPM → 内部单位)，并按 LSW→MSW 顺序打包。"""
        try:
            rpm = float(self.ent_spd.get())
        except ValueError:
            messagebox.showerror("错误", "速度请输入数字")
            return

        # 1) 按手册示例的因子换算：
        SCALE = 546133 / 200.0     # 从手册示例反推，=2730.665
        raw = int(rpm * SCALE)

        # 2) 拆成两个 16-bit 寄存器：先低字(lo)，再高字(hi)
        lo = raw & 0xFFFF
        hi = (raw >> 16) & 0xFFFF

        # 3) 打包并发送
        frm = build_frame_multiple(0x4A00, [lo, hi])
        self.send_frame(frm)
        self.log(f"写入速度: {rpm} RPM → {raw} (0x{raw:08X})")
    def write_accel(self):
        try:
            accel = float(self.ent_acc.get())
        except ValueError:
            messagebox.showerror("错误", "加速度请输入数字")
            return
        raw = int(accel * ACCEL_FACTOR)
        lo = raw & 0xFFFF
        hi = (raw >> 16) & 0xFFFF
        frm = build_frame_multiple(0x4B00, [lo, hi])
        self.send_frame(frm)
        self.log(f"写入加速度: {accel} rps/s → {raw} (0x{raw:08X})")

    def write_decel(self):
        try:
            decel = float(self.ent_dec.get())
        except ValueError:
            messagebox.showerror("错误", "减速度请输入数字")
            return
        raw = int(decel * ACCEL_FACTOR)
        lo = raw & 0xFFFF
        hi = (raw >> 16) & 0xFFFF
        frm = build_frame_multiple(0x4C00, [lo, hi])
        self.send_frame(frm)
        self.log(f"写入减速度: {decel} rps/s → {raw} (0x{raw:08X})")

    def start_motion(self):
            """
            启动运动：先取消使能（写 0x3100 = 0x0006），
            再发送启动命令（写 0x3100 = 0x003F）。
            """
            # 第一步：取消使能
            frm_disable = build_frame_single(0x3100, 0x0006)
            self.send_frame(frm_disable)
            self.log(">> 取消使能")
            
            # 等待电机内部状态切换
            time.sleep(0.1)

            # 第二步：启动运动
            frm_start = build_frame_single(0x3100, 0x003F)
            self.send_frame(frm_start)
            self.log(">> 启动运动")
    def position_test_start(self):
        if not (self.serial_port and self.serial_port.is_open):
            messagebox.showwarning("警告", "请先连接串口")
            return
        # 构造完整序列：每个位置写一次，然后 off,on，总共 4 pos × 10 周
        # position_pulses = [-65536, 0, -127431, -191146]
        position_pulses = [0, -127431]
        cycles = 2
        seq = []
        for _ in range(cycles):
            for p in position_pulses:
                seq.append(('pos', p))
                seq.append(('off', None))
                seq.append(('on', None))
        self._test_seq = seq
        self._test_idx = 0
        self._test_running = True
        self.log("位置测试开始")
        self._run_position_step()

    def _run_position_step(self):
        if not self._test_running or self._test_idx >= len(self._test_seq):
            self.log("位置测试结束")
            self._test_running = False
            return

        cmd, arg = self._test_seq[self._test_idx]
        if cmd == 'pos':
            # 写目标位置
            pulse = arg
            lo = pulse & 0xFFFF
            hi = (pulse >> 16) & 0xFFFF
            frame = build_frame_multiple(0x4000, [lo, hi])
            self.send_frame(frame)
            self.log(f"[Test] 写脉冲 {pulse}")
        elif cmd == 'off':
            frm = build_frame_single(0x3100, 0x0006)
            self.send_frame(frm)
        elif cmd == 'on':
            frm = build_frame_single(0x3100, 0x003F)
            self.send_frame(frm)

        self._test_idx += 1
        # 1 秒后执行下一步
        self.after(2000, self._run_position_step)
    # ---- 读取实际位置 ----
    def read_actual_position(self):
        """读 0x3700 → 2 寄存器（Integer32），并更新多圈累计角度显示。"""
        if not (self.serial_port and self.serial_port.is_open):
            messagebox.showwarning("警告", "请先连接串口")
            return

        # 丢弃旧数据
        self.serial_port.reset_input_buffer()
        # 发送读命令
        frm = build_read_frame(0x3700, 2)
        self.serial_port.write(frm)
        self.log(">> " + frm.hex(' ').upper())

        resp = self.serial_port.read(9)
        self.log("<< " + (resp.hex(' ').upper() if resp else "<no data>"))
        if len(resp) != 9 or resp[0] != SLAVE_ID or resp[1] != 0x03:
            messagebox.showerror("错误", "读取实际位置失败")
            return

        # 5) 拆 4 字节数据成有符号 32 位
        raw = struct.unpack('>i', resp[3:7])[0]
        # 转成无符号 32 位
        raw_u = raw & 0xFFFFFFFF

        # 累计增量
        if self.prev_raw_u is not None:
            delta = raw_u - self.prev_raw_u
            # 处理环绕
            if   delta >  (1<<31): delta -= (1<<32)
            elif delta < -(1<<31): delta += (1<<32)
            self.total_counts += delta

        self.prev_raw_u = raw_u

        # 多圈累计度数
        cumulative_deg = self.total_counts * 360.0 / (1<<32)

        # 更新到界面（同时显示 raw_u 和累计度数）
        self.lbl_actual.config(text=f"{raw_u}  ({cumulative_deg:.2f}°)")



if __name__ == '__main__':
    App().mainloop()     
