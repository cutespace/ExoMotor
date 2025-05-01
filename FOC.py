#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tkinter as tk
from tkinter import ttk, messagebox
import serial, serial.tools.list_ports
import struct

SLAVE_ID   = 0x01
BAUDRATE   = 115200
TIMEOUT    = 0.5

# 驱动器峰值电流 (A)，按实际填写
I_PEAK = 62.7  
DEC_PER_AMP = 2048 / (I_PEAK / 1.414)  # 1 A -> DEC

def calc_crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return struct.pack('<H', crc)

def build_frame_single(reg: int, val: int) -> bytes:
    p = struct.pack('>B B H H', SLAVE_ID, 0x06, reg, val)
    return p + calc_crc16(p)

def build_read_frame(reg: int, cnt: int) -> bytes:
    p = struct.pack('>B B H H', SLAVE_ID, 0x03, reg, cnt)
    return p + calc_crc16(p)

class TorqueTestApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("FOC 扭矩/电流测试")
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.ser = None
        self.monitoring = False

        frm = ttk.Frame(self, padding=10)
        frm.grid()

        # 串口选择
        ttk.Label(frm, text="串口:").grid(row=0, column=0)
        self.cb = ttk.Combobox(frm, width=12, state='readonly')
        self.cb['values'] = self._scan_ports()
        self.cb.grid(row=0, column=1)
        ttk.Button(frm, text="刷新", command=self._refresh).grid(row=0, column=2)
        self.btn_conn = ttk.Button(frm, text="连接", command=self._toggle_conn)
        self.btn_conn.grid(row=0, column=3, padx=(5,0))

        # 模式和使能控制
        ttk.Button(frm, text="切到扭矩模式", command=self._torque_mode).grid(row=1, column=0, columnspan=2, sticky='ew', pady=5)
        ttk.Button(frm, text="使能驱动", command=self._enable).grid(row=1, column=2, sticky='ew', padx=2)
        ttk.Button(frm, text="取消使能", command=self._disable).grid(row=1, column=3, sticky='ew')

        # 目标扭矩%
        ttk.Label(frm, text="目标扭矩 (%):").grid(row=2, column=0, sticky='e')
        self.ent_tau = ttk.Entry(frm, width=8)
        self.ent_tau.insert(0, "4.00")
        self.ent_tau.grid(row=2, column=1)
        ttk.Button(frm, text="下发扭矩%", command=self._set_tau).grid(row=2, column=2, columnspan=2, sticky='ew')

        # 目标电流
        ttk.Label(frm, text="目标电流 (A):").grid(row=3, column=0, sticky='e')
        self.ent_cur = ttk.Entry(frm, width=8)
        self.ent_cur.insert(0, "1.00")
        self.ent_cur.grid(row=3, column=1)
        ttk.Button(frm, text="下发电流", command=self._set_cur).grid(row=3, column=2, columnspan=2, sticky='ew')

        # 实际电流显示
        ttk.Label(frm, text="实际电流 (A):").grid(row=4, column=0, sticky='e')
        self.lbl = ttk.Label(frm, text="--")
        self.lbl.grid(row=4, column=1, columnspan=2, sticky='w')
        ttk.Button(frm, text="读取一次", command=self._read_once).grid(row=4, column=3)

        # 监测开关
        self.btn_mon = ttk.Button(frm, text="开始监测", command=self._toggle_mon)
        self.btn_mon.grid(row=5, column=0, columnspan=4, sticky='ew', pady=5)

        # 日志区
        self.txt = tk.Text(frm, width=60, height=10, state='disabled')
        self.txt.grid(row=6, column=0, columnspan=4, pady=10)

    def log(self, msg: str):
        self.txt['state'] = 'normal'
        self.txt.insert('end', msg + '\n')
        self.txt.see('end')
        self.txt['state'] = 'disabled'

    def _scan_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def _refresh(self):
        self.cb['values'] = self._scan_ports()

    def _toggle_conn(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.btn_conn.config(text="连接")
            self.log("串口已断开")
        else:
            port = self.cb.get()
            if not port:
                return messagebox.showwarning("提示", "请选择串口")
            try:
                self.ser = serial.Serial(port, BAUDRATE, timeout=TIMEOUT)
                self.btn_conn.config(text="断开")
                self.log(f"已连接 {port}")
            except Exception as e:
                messagebox.showerror("错误", f"打开失败: {e}")

    def _torque_mode(self):
        if not self._chk(): return
        frm = build_frame_single(0x3500, 4)
        self.ser.write(frm)
        self.log("切到扭矩模式 >> " + frm.hex(' ').upper())

    def _enable(self):
        if not self._chk(): return
        frm = build_frame_single(0x3100, 0x000F)
        self.ser.write(frm)
        self.log("使能驱动 >> " + frm.hex(' ').upper())

    def _disable(self):
        if not self._chk(): return
        frm = build_frame_single(0x3100, 0x0006)
        self.ser.write(frm)
        self.log("取消使能 >> " + frm.hex(' ').upper())

    def _set_tau(self):
        if not self._chk(): return
        try:
            pct = float(self.ent_tau.get())
        except ValueError:
            return messagebox.showerror("错误", "扭矩百分比输入有误")
        raw = int(pct * 10)        # 例如 pct = -10.00 → raw = -1000
        raw16 = raw & 0xFFFF        # now raw16 = 0xFC18（二进制补码）
        frm = build_frame_single(0x3C00, raw16)
        self.ser.write(frm)    
        self.log(f"下发扭矩% >> {pct:.2f}% (DEC={raw}) | " + frm.hex(' ').upper())

    def _set_cur(self):
        if not self._chk(): return
        try:
            a = float(self.ent_cur.get())
        except ValueError:
            return messagebox.showerror("错误", "电流输入有误")
        raw = int(a * DEC_PER_AMP)
        frm = build_frame_single(0x5880, raw)
        self.ser.write(frm)
        self.log(f"下发电流 >> {a:.2f}A (DEC={raw}) | " + frm.hex(' ').upper())

    def _read_once(self):
        if not self._chk(): return
        val = self._read_act(debug=True)
        self.lbl.config(text=f"{val:.2f}" if val is not None else "ERR")

    def _toggle_mon(self):
        if not self._chk(): return
        self.monitoring = not self.monitoring
        self.btn_mon.config(text="停止监测" if self.monitoring else "开始监测")
        if self.monitoring:
            self._poll()

    def _poll(self):
        val = self._read_act()
        self.lbl.config(text=f"{val:.2f}" if val is not None else "ERR")
        if self.monitoring:
            self.after(500, self._poll)

    def _read_act(self, debug=False):
        self.ser.reset_input_buffer()
        frm = build_read_frame(0x3E00, 1)
        self.ser.write(frm)
        if debug:
            self.log(">> 读取电流 >> " + frm.hex(' ').upper())
        resp = self.ser.read(7)
        if debug:
            self.log("<< " + (resp.hex(' ').upper() if resp else "<no data>"))
        if len(resp) != 7 or resp[1] != 0x03:
            return None
        hi, lo = resp[3], resp[4]
        raw = (hi << 8) | lo
        if raw & 0x8000:
            raw -= 0x10000
        return raw / DEC_PER_AMP

    def _chk(self):
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("提示", "请先连接串口")
            return False
        return True

    def _on_close(self):
        self.monitoring = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.destroy()

if __name__ == "__main__":
    TorqueTestApp().mainloop()
