#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time, struct, tkinter as tk
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
# 驱动器峰值电流 A，和之前一样
I_PEAK = 62.7
# 1 A 对应多少 dec
DEC_PER_AMP = 2048 / (I_PEAK / 1.414)
# 电机转矩常数 Nm/A（SMK60-020 取 0.124）
KT = 0.124

def calc_crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return struct.pack('<H', crc)

def build_read(reg:int, cnt:int) -> bytes:
    p = struct.pack('>B B H H', SLAVE_ID, 0x03, reg, cnt)
    return p + calc_crc16(p)

def build_write(reg:int, val:int) -> bytes:
    p = struct.pack('>B B H H', SLAVE_ID, 0x06, reg, val & 0xFFFF)
    return p + calc_crc16(p)

def build_write_multiple(reg:int, vals:list[int]) -> bytes:
    n = len(vals); bc = n*2
    hdr = struct.pack('>B B H H B', SLAVE_ID, 0x10, reg, n, bc)
    data = b''.join(struct.pack('>H', v & 0xFFFF) for v in vals)
    frame = hdr + data
    return frame + calc_crc16(frame)

def read_int16(ser, reg:int) -> int:
    ser.reset_input_buffer()
    ser.write(build_read(reg,1))
    r = ser.read(7)
    if len(r)==7 and r[1]==0x03:
        raw = (r[3]<<8)|r[4]
        return raw - 0x10000 if raw&0x8000 else raw
    raise IOError(f"读寄存器0x{reg:04X}失败: {r.hex().upper()}")

def read_raw32(ser, reg:int) -> (int):
    ser.reset_input_buffer()
    ser.write(build_read(reg,2))
    r = ser.read(9)
    if len(r)==9 and r[1]==0x03:
        s = struct.unpack('>i', r[3:7])[0]
        return s, s & 0xFFFFFFFF
    raise IOError(f"读寄存器0x{reg:04X}失败: {r.hex().upper()}")

def write_int16(ser, reg:int, val:int):
    ser.write(build_write(reg,val))
    time.sleep(0.002); ser.read(8)

def write_multiple(ser, reg:int, vals:list[int]):
    ser.write(build_write_multiple(reg,vals))
    time.sleep(0.005)
    ser.read(8)

class AdmittanceApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("导纳控制示例")
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.ser = None; self.running = False

        # 展开多圈位置 & 导纳累积
        self.prev_raw_u = None
        self.total_counts = 0
        self.prev_t = time.time()
        self.pos_ref_f = 0.0   # 浮点型目标位置

        frm = ttk.Frame(self,padding=8); frm.grid()
        ttk.Label(frm,text="串口:").grid(row=0,column=0)
        self.cb = ttk.Combobox(frm,width=12,state='readonly'); self.cb.grid(row=0,column=1)
        ttk.Button(frm,text="刷新",command=self._refresh).grid(row=0,column=2)
        self.btn_conn = ttk.Button(frm,text="连接",command=self._toggle_conn); self.btn_conn.grid(row=0,column=3)
        self._refresh()

        params = [
            ("质量 M (kg·m²):", "1.0"),
            ("阻尼 B (N·m·s/rad):", "1.0"),
        ]
        self.vars = {}
        for i,(lbl,init) in enumerate(params, start=1):
            ttk.Label(frm,text=lbl).grid(row=i,column=0,sticky='e')
            v = tk.StringVar(value=init); self.vars[lbl]=v
            ttk.Entry(frm,textvariable=v,width=10).grid(row=i,column=1,columnspan=2)

        self.btn_start = ttk.Button(frm,text="▶ 启动导纳",command=self._start)
        self.btn_start.grid(row=4,column=0,columnspan=2,sticky='ew',pady=6)
        self.btn_stop  = ttk.Button(frm,text="■ 停止",command=self._stop,state='disabled')
        self.btn_stop.grid(row=4,column=2,columnspan=2,sticky='ew')

        self.txt = tk.Text(frm,width=60,height=12,state='disabled')
        self.txt.grid(row=5,column=0,columnspan=4,pady=8)

    def log(self,msg):
        self.txt['state']='normal'
        self.txt.insert('end',msg+"\n"); self.txt.see('end')
        self.txt['state']='disabled'

    def _refresh(self):
        self.cb['values']=[p.device for p in comports()]

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
            self.M = float(self.vars["质量 M (kg·m²):"].get())
            self.B = float(self.vars["阻尼 B (N·m·s/rad):"].get())
        except:
            return messagebox.showerror("参数错误","请输入合法数值")

        # 初读 raw_u，清零多圈累计 & pos_ref_f
        _, raw_u = read_raw32(self.ser,0x3700)
        self.prev_raw_u = raw_u
        self.total_counts = 0
        self.prev_t = time.time()
        self.pos_ref_f = 0.0

        # 切位置模式 & 使能
        write_int16(self.ser,0x3500,3)
        write_int16(self.ser,0x3100,0x0006)
        time.sleep(0.05); self.ser.reset_input_buffer()
        self.log("已切位置模式并使能")

        self.running=True
        self.btn_start.config(state='disabled')
        self.btn_stop.config(state='normal')
        self._loop()
    def _loop(self):
        if not self.running: return
        t0 = time.time()

        # —— 多圈位置展开，仅作显示/速度计算用 —— 
        raw, raw_u = read_raw32(self.ser, 0x3700)
        delta = raw_u - self.prev_raw_u
        if delta >  (1<<31): delta -= (1<<32)
        elif delta < -(1<<31): delta += (1<<32)
        self.total_counts += delta
        self.prev_raw_u = raw_u
        pos_act = self.total_counts

        dt = max(1e-3, t0 - self.prev_t)
        self.prev_t = t0

        # —— 读反馈电流 dec —— 
        i_dec = read_int16(self.ser, 0x3E00)

        # —— 单位换算 & 导纳映射 —— 
        i_amp    = i_dec / DEC_PER_AMP            # dec → A
        torque   = i_amp * KT                     # A → Nm
        w_rad_s  = torque / self.B                # Nm → rad/s
        rpm      = w_rad_s * 60.0 / (2 * 3.1415926) 
        raw_spd  = int(rpm * (546133.0/200.0))     # RPM → internal units :contentReference[oaicite:2]{index=2}&#8203;:contentReference[oaicite:3]{index=3}

        # —— 下发速度指令 —— 
        lo = raw_spd & 0xFFFF
        hi = (raw_spd >> 16) & 0xFFFF
        write_multiple(self.ser, 0x4A00, [lo, hi])

        self.log(f"pos={pos_act}, I_dec={i_dec}, I={i_amp:.2f}A, τ={torque:.3f}Nm, ω={w_rad_s:.2f}rad/s, raw_spd={raw_spd}")
        self.after(10, self._loop)



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
    AdmittanceApp().mainloop()
