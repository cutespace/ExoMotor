#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import struct
import tkinter as tk
from tkinter import ttk, messagebox

try:
    import serial
    from serial.tools.list_ports import comports
except ImportError:
    messagebox.showerror("导入错误", "请先安装 pyserial：pip install pyserial")
    raise

SLAVE_ID = 0x01
BAUDRATE = 115200
TIMEOUT  = 0.2

def calc_crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return struct.pack('<H', crc)

def build_read(reg: int, cnt: int) -> bytes:
    p = struct.pack('>B B H H', SLAVE_ID, 0x03, reg, cnt)
    return p + calc_crc16(p)

def build_write(reg: int, val: int) -> bytes:
    p = struct.pack('>B B H H', SLAVE_ID, 0x06, reg, val & 0xFFFF)
    return p + calc_crc16(p)

def read_int16(ser, reg: int) -> int:
    ser.reset_input_buffer()
    ser.write(build_read(reg, 1))
    resp = ser.read(7)
    if len(resp)==7 and resp[1]==0x03:
        raw = (resp[3]<<8)|resp[4]
        return raw-0x10000 if raw&0x8000 else raw
    raise IOError(f"读0x{reg:04X}失败: {resp.hex().upper()}")

def read_raw32(ser, reg: int) -> (int):
    """读 2 寄存器，返回 (signed32, unsigned32)"""
    ser.reset_input_buffer()
    ser.write(build_read(reg,2))
    resp = ser.read(9)
    if len(resp)==9 and resp[1]==0x03:
        raw = struct.unpack('>i', resp[3:7])[0]
        raw_u = raw & 0xFFFFFFFF
        return raw, raw_u
    raise IOError(f"读0x{reg:04X}失败: {resp.hex().upper()}")

def write_int16(ser, reg: int, val: int):
    ser.write(build_write(reg,val))
    time.sleep(0.002)
    ser.read(8)

class ImpedanceApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("阻抗控制示例")
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.ser = None
        self.running = False

        # 多圈位置展开变量
        self.prev_raw_u   = None
        self.total_counts = 0
        self.prev_pos     = 0
        self.prev_t       = time.time()

        frm = ttk.Frame(self, padding=8); frm.grid()
        # 串口
        ttk.Label(frm, text="串口:").grid(row=0,column=0)
        self.cb = ttk.Combobox(frm,width=12,state='readonly'); self.cb.grid(row=0,column=1)
        ttk.Button(frm,text="刷新",command=self._refresh).grid(row=0,column=2)
        self.btn_conn = ttk.Button(frm,text="连接",command=self._toggle_conn); self.btn_conn.grid(row=0,column=3)
        self._refresh()

        # 参数区
        params = [("目标位置 (turns):","1.00"),("位置增益 Kp:","5"),("速度增益 Kd:","1")]
        self.vars = {}
        for i,(lbl,init) in enumerate(params, start=1):
            ttk.Label(frm,text=lbl).grid(row=i,column=0,sticky='e')
            v = tk.StringVar(value=init); self.vars[lbl]=v
            ttk.Entry(frm,textvariable=v,width=10).grid(row=i,column=1,columnspan=2)

        # 启停
        self.btn_start = ttk.Button(frm,text="▶ 启动阻抗",command=self._start)
        self.btn_start.grid(row=4,column=0,columnspan=2,sticky='ew',pady=6)
        self.btn_stop  = ttk.Button(frm,text="■ 停止",command=self._stop,state='disabled')
        self.btn_stop.grid(row=4,column=2,columnspan=2,sticky='ew')

        # 日志
        self.txt = tk.Text(frm,width=60,height=12,state='disabled')
        self.txt.grid(row=5,column=0,columnspan=4,pady=8)

    def log(self,msg):
        self.txt['state']='normal'
        self.txt.insert('end',msg+"\n")
        self.txt.see('end')
        self.txt['state']='disabled'

    def _refresh(self):
        self.cb['values'] = [p.device for p in comports()]

    def _toggle_conn(self):
        if self.ser:
            self.ser.close(); self.ser=None
            self.btn_conn.config(text="连接"); self.log("串口已断开")
        else:
            port=self.cb.get()
            if not port: return messagebox.showwarning("请选择串口")
            try:
                self.ser=serial.Serial(port,BAUDRATE,timeout=TIMEOUT)
                self.btn_conn.config(text="断开"); self.log(f"已连接 {port}")
            except Exception as e:
                messagebox.showerror("打开失败",str(e))

    def _start(self):
        if not(self.ser and self.ser.is_open):
            return messagebox.showwarning("请先连接串口")
        try:
            turns = float(self.vars["目标位置 (turns):"].get())
            self.Kp = float(self.vars["位置增益 Kp:"].get())
            self.Kd = float(self.vars["速度增益 Kd:"].get())
        except:
            return messagebox.showerror("参数错误","请输入合法数值")

        # 读首个 raw_u，清零累计
        _, raw_u = read_raw32(self.ser,0x3700)
        self.prev_raw_u   = raw_u
        self.total_counts = 0
        self.prev_pos     = 0
        self.prev_t       = time.time()

        # 目标位置转换为脉冲
        self.pos_target = int(turns * 65536)

        # 切扭矩模式 & 使能
        write_int16(self.ser,0x3500,4)
        write_int16(self.ser,0x3100,0x000F)
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.log("已切扭矩模式并使能")

        self.running=True
        self.btn_start.config(state='disabled')
        self.btn_stop.config(state='normal')
        self._loop()

    def _loop(self):
        if not self.running: return
        t0 = time.time()
        # 读单圈 raw + raw_u
        raw, raw_u = read_raw32(self.ser,0x3700)
        # 计算增量并展开
        delta = raw_u - self.prev_raw_u
        if   delta >  (1<<31): delta -= (1<<32)
        elif delta < -(1<<31): delta += (1<<32)
        self.total_counts += delta
        self.prev_raw_u = raw_u

        # 多圈位置 pos
        pos = self.total_counts
        dt  = t0 - self.prev_t
        vel = (pos - self.prev_pos) / dt if dt>1e-3 else 0
        self.prev_pos, self.prev_t = pos, t0

        # PD 计算扭矩
        tau = int(self.Kp*(self.pos_target-pos) + self.Kd*(-vel))
        tau = max(-100, min(100, tau))
        write_int16(self.ser,0x3C00,tau)

        # 读实际电流
        i_fb = read_int16(self.ser,0x3E00)

        self.log(f"raw_u={raw_u}, pos={pos}, vel={vel:.1f}, cmdτ={tau}, I_fb={i_fb}")
        self.after(10,self._loop)

    def _stop(self):
        self.running=False
        write_int16(self.ser,0x3100,0x0006)
        self.log("已停止 & 取消使能")
        self.btn_start.config(state='normal')
        self.btn_stop.config(state='disabled')

    def _on_close(self):
        self.running=False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.destroy()

if __name__=='__main__':
    ImpedanceApp().mainloop()
