#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import threading
import queue
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import struct

# ------- 常量配置 -------
SLAVE_ID = 0x01
BAUDRATE = 115200
TIMEOUT = 0.05       # 串口超时 (秒)
POSITION_FACTOR = 65536  # 脉冲/转
ACCEL_FACTOR = POSITION_FACTOR**2 / 4_000_000  # 加速因子
POLL_HZ = 20        # 串口轮询频率 (Hz)
UI_HZ = 10          # 界面更新频率 (Hz)

# ------- CRC16 查表法 -------
CRC16_TABLE = []
for i in range(256):
    crc = i
    for _ in range(8):
        if crc & 1:
            crc = (crc >> 1) ^ 0xA001
        else:
            crc >>= 1
    CRC16_TABLE.append(crc)

def calc_crc16(data: bytes) -> bytes:
    """计算 Modbus-RTU CRC16"""
    crc = 0xFFFF
    for b in data:
        crc = (crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]
    return struct.pack('<H', crc)

# ------- 帧构建 -------
def build_frame_single(register: int, value: int) -> bytes:
    payload = struct.pack('>B B H H', SLAVE_ID, 0x06, register, value)
    return payload + calc_crc16(payload)

def build_frame_multiple(register: int, values: list[int]) -> bytes:
    header = struct.pack('>B B H H B', SLAVE_ID, 0x10, register, len(values), len(values)*2)
    body = b''.join(struct.pack('>H', v) for v in values)
    frame = header + body
    return frame + calc_crc16(frame)

def build_read_frame(register: int, count: int) -> bytes:
    payload = struct.pack('>B B H H', SLAVE_ID, 0x03, register, count)
    return payload + calc_crc16(payload)

# ------- 串口管理线程 -------
class SerialWorker(threading.Thread):
    def __init__(self, ser: serial.Serial, cmd_q: queue.Queue, resp_q: queue.Queue, poll_hz=POLL_HZ):
        super().__init__(daemon=True)
        self.ser = ser
        self.cmd_q = cmd_q
        self.resp_q = resp_q
        self.poll_interval = 1.0 / poll_hz
        self.read_frame = build_read_frame(0x3700, 2)
        self.running = True

    def run(self):
        while self.running:
            start = time.time()
            # 处理所有发送命令
            try:
                while True:
                    frame = self.cmd_q.get_nowait()
                    self.ser.write(frame)
                    self.cmd_q.task_done()
            except queue.Empty:
                pass
            # 周期性读取实际位置
            try:
                self.ser.write(self.read_frame)
                resp = self.ser.read(9)
                if len(resp) == 9 and resp[0] == SLAVE_ID and resp[1] == 0x03:
                    raw_signed = struct.unpack('>i', resp[3:7])[0]
                    raw_u = raw_signed & 0xFFFFFFFF
                    self.resp_q.put(raw_u)
            except Exception:
                pass
            # 保持周期
            elapsed = time.time() - start
            if elapsed < self.poll_interval:
                time.sleep(self.poll_interval - elapsed)

    def stop(self):
        self.running = False
        self.join()

# ------- 主应用 -------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Kinco Modbus 虚拟限位版")
        # 串口和线程
        self.serial_port = None
        self.worker = None
        self.cmd_q = None
        self.resp_q = None
        # 多圈累积
        self.prev_raw_u = None
        self.total_counts = 0
        # 虚拟限位配置
        self.virtual_limit_enabled = False
        self.upper_limit_turns = 2.0   # +720°
        self.lower_limit_turns = -1.0  # -360°
        self.buffer_turns = 0.025      # ±0.025 转
        self.locked_upper = False
        self.locked_lower = False
        # 构建 UI
        self._build_ui()
        self.after(int(1000/UI_HZ), self._poll_feedback)

    def _build_ui(self):
        frm = ttk.Frame(self, padding=10)
        frm.grid()
        # 串口选择
        ttk.Label(frm, text="串口:").grid(row=0, column=0, sticky='e')
        self.cb_ports = ttk.Combobox(frm, width=15, state='readonly')
        self.cb_ports['values'] = [p.device for p in serial.tools.list_ports.comports()]
        self.cb_ports.grid(row=0, column=1)
        ttk.Button(frm, text="刷新", command=self._refresh_ports).grid(row=0, column=2)
        self.btn_conn = ttk.Button(frm, text="连接", command=self._toggle_connection)
        self.btn_conn.grid(row=0, column=3)
        # 基本操作
        ops = [
            ("使能", self._cmd_enable),
            ("取消使能", self._cmd_disable),
            ("位置模式", self._cmd_pos_mode),
            ("启动运动", self._cmd_start_motion)
        ]
        for i, (txt, cmd) in enumerate(ops, 1):
            ttk.Button(frm, text=txt, command=cmd, width=12).grid(row=i, column=0, columnspan=2, pady=2)
        # 虚拟限位切换
        self.btn_virtual = ttk.Button(frm, text="启用虚拟限位", command=self._toggle_virtual_limit, width=12)
        self.btn_virtual.grid(row=5, column=0, columnspan=2, pady=2)
        # 参数输入与写入
        ttk.Label(frm, text="目标位置(转):").grid(row=1, column=2, sticky='e')
        self.ent_pos = ttk.Entry(frm, width=12); self.ent_pos.insert(0, "1.0"); self.ent_pos.grid(row=1, column=3)
        ttk.Button(frm, text="写入位置", command=self._cmd_write_position).grid(row=2, column=2, columnspan=2, pady=2)
        ttk.Label(frm, text="速度(RPM):").grid(row=3, column=2, sticky='e')
        self.ent_spd = ttk.Entry(frm, width=12); self.ent_spd.insert(0, "200"); self.ent_spd.grid(row=3, column=3)
        ttk.Button(frm, text="写入速度", command=self._cmd_write_speed).grid(row=4, column=2, columnspan=2, pady=2)
        ttk.Label(frm, text="加速(rps/s):").grid(row=6, column=2, sticky='e')
        self.ent_acc = ttk.Entry(frm, width=12); self.ent_acc.insert(0, "300"); self.ent_acc.grid(row=6, column=3)
        ttk.Button(frm, text="写入加速", command=self._cmd_write_accel).grid(row=7, column=2, columnspan=2, pady=2)
        ttk.Label(frm, text="减速(rps/s):").grid(row=8, column=2, sticky='e')
        self.ent_dec = ttk.Entry(frm, width=12); self.ent_dec.insert(0, "300"); self.ent_dec.grid(row=8, column=3)
        ttk.Button(frm, text="写入减速", command=self._cmd_write_decel).grid(row=9, column=2, columnspan=2, pady=2)
        # 日志
        self.txt_log = tk.Text(frm, width=60, height=10, state='disabled')
        self.txt_log.grid(row=10, column=0, columnspan=4, pady=10)
        # 实际位置显示
        ttk.Label(frm, text="反馈位置:").grid(row=11, column=2, sticky='e')
        self.lbl_actual = ttk.Label(frm, text="--")
        self.lbl_actual.grid(row=11, column=3)

    def log(self, msg: str):
        self.txt_log['state'] = 'normal'
        self.txt_log.insert('end', msg + '\n')
        self.txt_log.see('end')
        self.txt_log['state'] = 'disabled'

    def _refresh_ports(self):
        self.cb_ports['values'] = [p.device for p in serial.tools.list_ports.comports()]

    def _toggle_connection(self):
        if self.worker and self.worker.is_alive():
            self.worker.stop()
            self.serial_port.close()
            self.worker = None
            self.serial_port = None
            self.log("已断开串口")
            self.btn_conn.config(text="连接")
        else:
            port = self.cb_ports.get()
            if not port:
                messagebox.showwarning("警告", "请选择串口")
                return
            try:
                self.serial_port = serial.Serial(port, BAUDRATE, timeout=TIMEOUT)
                self.cmd_q = queue.Queue()
                self.resp_q = queue.Queue()
                self.worker = SerialWorker(self.serial_port, self.cmd_q, self.resp_q, POLL_HZ)
                self.worker.start()
                self.log(f"已连接 {port}")
                self.btn_conn.config(text="断开")
            except Exception as e:
                messagebox.showerror("错误", f"无法连接串口: {e}")

    def _enqueue_frame(self, frame: bytes, note: str = None):
        if not (self.worker and self.worker.is_alive()):
            messagebox.showwarning("警告", "请先连接串口")
            return
        self.log((note + ' ') if note else '' + ">> " + frame.hex(' ').upper())
        self.cmd_q.put(frame)

    # 基础命令
    def _cmd_enable(self):   self._enqueue_frame(build_frame_single(0x3100, 0x000F), "使能:")
    def _cmd_disable(self):  self._enqueue_frame(build_frame_single(0x3100, 0x0006), "取消使能:")
    def _cmd_pos_mode(self): self._enqueue_frame(build_frame_single(0x3500, 0x0001), "位置模式:")
    def _cmd_start_motion(self):
        self._enqueue_frame(build_frame_single(0x3100, 0x0006), "运动前取消使能:")
        time.sleep(0.1)
        self._enqueue_frame(build_frame_single(0x3100, 0x003F), "启动运动:")

    # 写入参数
    def _cmd_write_position(self):
        try:
            turns = float(self.ent_pos.get())
        except ValueError:
            messagebox.showerror("错误", "位置请输入数字")
            return
        pulse = int(turns * POSITION_FACTOR)
        lo, hi = pulse & 0xFFFF, (pulse >> 16) & 0xFFFF
        self._enqueue_frame(build_frame_multiple(0x4000, [lo, hi]), f"写入位置 {turns} 转:")

    def _cmd_write_speed(self):
        try: rpm = float(self.ent_spd.get())
        except ValueError:
            messagebox.showerror("错误", "速度请输入数字")
            return
        raw = int(rpm * (546133/200.0))
        lo, hi = raw & 0xFFFF, (raw >> 16) & 0xFFFF
        self._enqueue_frame(build_frame_multiple(0x4A00, [lo, hi]), f"写入速度 {rpm} RPM:")

    def _cmd_write_accel(self):
        try: accel = float(self.ent_acc.get())
        except ValueError:
            messagebox.showerror("错误", "加速请输入数字")
            return
        raw = int(accel * ACCEL_FACTOR)
        lo, hi = raw & 0xFFFF, (raw >> 16) & 0xFFFF
        self._enqueue_frame(build_frame_multiple(0x4B00, [lo, hi]), f"写入加速 {accel} rps/s:")

    def _cmd_write_decel(self):
        try: decel = float(self.ent_dec.get())
        except ValueError:
            messagebox.showerror("错误", "减速请输入数字")
            return
        raw = int(decel * ACCEL_FACTOR)
        lo, hi = raw & 0xFFFF, (raw >> 16) & 0xFFFF
        self._enqueue_frame(build_frame_multiple(0x4C00, [lo, hi]), f"写入减速 {decel} rps/s:")

    # 虚拟限位开关
    def _toggle_virtual_limit(self):
        self.virtual_limit_enabled = not self.virtual_limit_enabled
        if self.virtual_limit_enabled:
            self.btn_virtual.config(text="关闭虚拟限位")
            self.log(f"虚拟限位启用: [{self.lower_limit_turns}..{self.upper_limit_turns}] 转, 缓冲±{self.buffer_turns}")
        else:
            self.btn_virtual.config(text="启用虚拟限位")
            self.log("虚拟限位已禁用")
            self.locked_upper = self.locked_lower = False
            self._enqueue_frame(build_frame_single(0x3100, 0x000F), "解除限位, 重新使能:")

    # 锁定位置并使能
    def _lock_at_turns(self, turns: float):
        # 1) 写目标位置
        pulse = int(turns * POSITION_FACTOR)
        lo, hi = pulse & 0xFFFF, (pulse >> 16) & 0xFFFF
        frame_pos = build_frame_multiple(0x4000, [lo, hi])
        # 2) 立即写入并延时
        self.worker.ser.write(frame_pos)
        time.sleep(0.01)
        # 3) 使能锁定扭矩
        frame_on = build_frame_single(0x3100, 0x000F)
        self.worker.ser.write(frame_on)
        self.log(f"[限位] 锁定于 {turns} 转，并使能")
        # 重置队列中可能的残留指令
        with self.cmd_q.mutex:
            self.cmd_q.queue.clear()

    # 界面轮询 & 虚拟限位逻辑
    def _poll_feedback(self):
        if self.resp_q:
            last = None
            try:
                while True:
                    last = self.resp_q.get_nowait()
            except queue.Empty:
                pass
            if last is not None:
                # 累计多圈度数
                if self.prev_raw_u is not None:
                    delta = last - self.prev_raw_u
                    if delta > (1<<31): delta -= (1<<32)
                    if delta < -(1<<31): delta += (1<<32)
                    self.total_counts += delta
                self.prev_raw_u = last
                deg = self.total_counts * 360.0 / (1<<32)
                # 虚拟限位逻辑
                if self.virtual_limit_enabled:
                    upper_thresh = (self.upper_limit_turns - self.buffer_turns) * 360
                    lower_thresh = (self.lower_limit_turns + self.buffer_turns) * 360
                    if not self.locked_upper and deg >= upper_thresh:
                        self.locked_upper = True; self.locked_lower = False
                        self._lock_at_turns(self.upper_limit_turns)
                    elif not self.locked_lower and deg <= lower_thresh:
                        self.locked_lower = True; self.locked_upper = False
                        self._lock_at_turns(self.lower_limit_turns)
                    else:
                        # 解锁条件：回到安全区
                        if self.locked_upper and deg < upper_thresh:
                            self.worker.ser.write(build_frame_single(0x3100, 0x0006))
                            self.log("[限位] 上限解锁，取消使能")
                            self.locked_upper = False
                        if self.locked_lower and deg > lower_thresh:
                            self.worker.ser.write(build_frame_single(0x3100, 0x0006))
                            self.log("[限位] 下限解锁，取消使能")
                            self.locked_lower = False
                # 更新显示
                self.lbl_actual.config(text=f"{last}  ({deg:.2f}°)")
        self.after(int(1000/UI_HZ), self._poll_feedback)

if __name__ == '__main__':
    App().mainloop()
