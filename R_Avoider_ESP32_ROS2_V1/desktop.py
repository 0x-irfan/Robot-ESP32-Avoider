#!/usr/bin/env python3
import math
import queue
import sys
import threading
import time
import tkinter as tk
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray, String

try:                                    # supaya Ctrl+C di terminal menutup jendela
    from rclpy.signals import SignalHandlerOptions
except ImportError:                     # pragma: no cover
    SignalHandlerOptions = None

MAX_CM = 120            # jangkauan radar (cm)
TRAIL_S = 6.0           # titik deteksi memudar dalam (detik)
GLOW_S = 2.5            # jejak sapuan garis hijau memudar dalam (detik)
MIRROR_X = False        # True = 0 derajat di KANAN radar (pandangan dari atas robot)
FRAME_MS = 33           # ~30 fps
OFFLINE_AFTER_S = 1.5   # tanpa telemetry selama ini -> OFFLINE

BG = '#05080d'
PANEL = '#06101a'
CELL = '#04080d'
CYAN = '#00f0ff'
CYAN_M = '#008b99'
PINK = '#ff0055'
YELLOW = '#ffe600'
RED = '#ff1e56'
GREEN = '#00ff66'
GREY = '#7e93a5'
GRID = '#0c4a58'
FONT = 'DejaVu Sans Mono'

TEL_KEYS = ['a', 'rdy', 'd', 'rtt', 'rssi', 'ch', 'up', 'heap', 'ml', 'mr', 'fs']
TEL_DEFAULTS = [90.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
INFO_KEYS = ['chip', 'mac', 'ip', 'ssid', 'bssid', 'cpu', 'sdk']


def _rgb(h):
    return tuple(int(h[i:i + 2], 16) for i in (1, 3, 5))


def blend(fg, bg, t):
    """Campur warna fg ke bg (t=1 -> fg penuh). Tk tidak punya alpha, jadi memudar = campur ke latar."""
    t = max(0.0, min(1.0, t))
    a, b = _rgb(fg), _rgb(bg)
    return '#%02x%02x%02x' % tuple(int(b[i] + (a[i] - b[i]) * t) for i in range(3))

class RosBridge(Node):
    def __init__(self, q):
        super().__init__('avoider_desktop')
        self.q = q
        self.rx_times = deque(maxlen=20)
        self.create_subscription(Float32MultiArray, '/avoider/telemetry',
                                 self.on_tel, qos_profile_sensor_data)
        self.create_subscription(String, '/avoider/info', self.on_info, qos_profile_sensor_data)
        self.create_subscription(String, '/avoider/status', self.on_status, 10)

    def push(self, item):
        try:
            self.q.put_nowait(item)
        except queue.Full:
            pass

    def on_tel(self, msg):
        d = list(msg.data)
        if len(d) < 6:
            return
        d = d + TEL_DEFAULTS[len(d):]
        t = dict(zip(TEL_KEYS, d))
        t['a'] = int(round(t['a']))
        t['rdy'] = t['rdy'] > 0.5
        t['rssi'] = int(t['rssi'])
        t['ch'] = int(t['ch'])
        t['up'] = int(t['up'])
        t['heap'] = int(t['heap'])
        t['fs'] = t['fs'] > 0.5
        now = time.monotonic()
        self.rx_times.append(now)
        rate = 0.0
        if len(self.rx_times) >= 2 and self.rx_times[-1] > self.rx_times[0]:
            rate = (len(self.rx_times) - 1) / (self.rx_times[-1] - self.rx_times[0])
        self.push(('tel', now, t, rate))

    def on_info(self, msg):
        parts = msg.data.split('|')
        parts += ['--'] * (len(INFO_KEYS) - len(parts))
        self.push(('info', dict(zip(INFO_KEYS, parts[:len(INFO_KEYS)]))))

    def on_status(self, msg):
        self.push(('status', msg.data))

class App:
    def __init__(self, root, q):
        self.root = root
        self.q = q

        # ---- state data ----
        self.online = False
        self.last_rx = -1e9
        self.status = '--'
        self.last_status_logged = None
        self.info = {k: '--' for k in INFO_KEYS}
        self.tel = None
        self.rate = 0.0
        self.target_angle = 90
        self.shown_angle = 90.0
        self.ready = False
        self.dist = -1.0
        self.last_glow_angle = None
        self.glow = []                  # {'a','t'}
        self.trail = []                 # {'a','d','t'}
        self.last_trail_add = 0.0
        self.last_snr_log = 0.0
        self.last_fs = None
        self.lat_hist = []
        self.last_lat_push = 0.0
        self.paused = False
        self.t0 = time.monotonic()
        self.cache = {}                 # cache teks label agar tidak update berlebihan

        self.cw, self.ch = 800, 600
        self.cx = self.cy = self.R = self.pxcm = 0

        self.build_ui()
        self.log('cm', 'INIT AVOIDER_OS...')
        self.log('cm', 'WAITING /avoider/telemetry ...')
        self.refresh_badges()
        self.tick()

    def panel(self, parent):
        return tk.Frame(parent, bg=PANEL, highlightthickness=1,
                        highlightbackground=CYAN, highlightcolor=CYAN)

    def badge(self, parent):
        return tk.Label(parent, text='', font=(FONT, 9, 'bold'), bg=BG, padx=8, pady=2,
                        highlightthickness=1)

    def build_ui(self):
        root = self.root
        root.title('Robot Avoider')
        root.configure(bg=BG)
        root.geometry('1280x720')
        root.minsize(1000, 560)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(1, weight=1)

        hdr = tk.Frame(root, bg=BG)
        hdr.grid(row=0, column=0, sticky='ew', padx=10, pady=(8, 4))
        hdr.columnconfigure(1, weight=1)
        self.mode_lbl = self.badge(hdr)
        self.mode_lbl.grid(row=0, column=0, sticky='w')
        title = tk.Frame(hdr, bg=BG)
        title.grid(row=0, column=1)
        tk.Label(title, text='ROBOT', font=(FONT, 16, 'bold'), fg=CYAN, bg=BG).pack(side='left')
        tk.Label(title, text=' AVOIDER', font=(FONT, 16, 'bold'), fg=YELLOW, bg=BG).pack(side='left')
        self.conn_lbl = self.badge(hdr)
        self.conn_lbl.grid(row=0, column=2, sticky='e')

        main = tk.Frame(root, bg=BG)
        main.grid(row=1, column=0, sticky='nsew', padx=10, pady=(0, 10))
        main.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=0)
        main.columnconfigure(1, weight=1)

        left = tk.Frame(main, bg=BG)
        left.grid(row=0, column=0, sticky='ns', padx=(0, 8))
        left.rowconfigure(1, weight=1)

        self.build_info(left)
        self.build_terminal(left)
        self.build_radar(main)

    def build_info(self, parent):
        p = self.panel(parent)
        p.grid(row=0, column=0, sticky='ew', pady=(0, 8))

        hd = tk.Frame(p, bg=PANEL)
        hd.pack(fill='x', padx=8, pady=(6, 4))
        tk.Label(hd, text='ESP32 // LINK INFO', font=(FONT, 10, 'bold'), fg=CYAN, bg=PANEL).pack(side='left')
        self.sdk_lbl = tk.Label(hd, text='micro-ROS . UDP', font=(FONT, 7), fg='#64829a', bg=PANEL)
        self.sdk_lbl.pack(side='right')

        grid = tk.Frame(p, bg=PANEL)
        grid.pack(fill='x', padx=6, pady=(0, 6))
        for c in range(3):
            grid.columnconfigure(c, weight=1, uniform='cell')

        def cell(r, c, caption, size=9, width=15):
            f = tk.Frame(grid, bg=CELL, highlightthickness=1, highlightbackground=CYAN_M)
            f.grid(row=r, column=c, sticky='nsew', padx=2, pady=2)
            tk.Label(f, text=caption, font=(FONT, 7, 'bold'), fg=GREY, bg=CELL,
                     anchor='w').pack(fill='x', padx=5)
            row = tk.Frame(f, bg=CELL)
            row.pack(fill='x', padx=5, pady=(0, 3))
            val = tk.Label(row, text='--', font=(FONT, size, 'bold'), fg=CYAN, bg=CELL,
                           anchor='w', width=width)
            val.pack(side='left')
            return val, row

        self.v = {}
        self.v['chip'], _ = cell(0, 0, 'CHIP', 8, 17)
        self.v['cpu'], _ = cell(0, 1, 'CPU')
        self.v['up'], _ = cell(0, 2, 'UPTIME')

        self.v['ssid'], _ = cell(1, 0, 'SSID')
        self.v['ip'], _ = cell(1, 1, 'IP ADDR')
        self.v['bssid'], _ = cell(1, 2, 'BSSID', 8, 17)

        self.v['mac'], _ = cell(2, 0, 'MAC', 8, 17)
        self.v['ch'], _ = cell(2, 1, 'FREQ CH')
        self.v['rssi'], rrow = cell(2, 2, 'RSSI', 9, 9)
        self.bars = tk.Canvas(rrow, width=22, height=14, bg=CELL, highlightthickness=0)
        self.bars.pack(side='left')

        self.v['lat'], lrow = cell(3, 0, 'LATENCY', 9, 6)
        self.spark = tk.Canvas(lrow, width=44, height=16, bg=CELL, highlightthickness=0)
        self.spark.pack(side='left')
        self.v['rate'], _ = cell(3, 1, 'DATA RATE')
        self.v['heap'], _ = cell(3, 2, 'FREE HEAP')

        self.v['ml'], _ = cell(4, 0, 'MOTOR L')
        self.v['mr'], _ = cell(4, 1, 'MOTOR R')
        self.v['fs'], _ = cell(4, 2, 'FAILSAFE')

    def build_terminal(self, parent):
        p = self.panel(parent)
        p.grid(row=1, column=0, sticky='nsew')
        p.rowconfigure(1, weight=1)
        p.columnconfigure(0, weight=1)

        hd = tk.Frame(p, bg=PANEL)
        hd.grid(row=0, column=0, sticky='ew', padx=8, pady=(6, 4))
        tk.Label(hd, text='COM.TERMINAL', font=(FONT, 10, 'bold'), fg=CYAN, bg=PANEL).pack(side='left')

        def btn(text, cmd):
            return tk.Button(hd, text=text, command=cmd, font=(FONT, 8, 'bold'), fg=PINK, bg=PANEL,
                             activeforeground=PANEL, activebackground=PINK, relief='flat', bd=0,
                             highlightthickness=1, highlightbackground=PINK, padx=8, pady=0,
                             cursor='hand2')
        self.btn_clear = btn('CLEAR', self.clear_term)
        self.btn_clear.pack(side='right')
        self.btn_pause = btn('PAUSE', self.toggle_pause)
        self.btn_pause.pack(side='right', padx=(0, 6))

        body = tk.Frame(p, bg='#020508', highlightthickness=1, highlightbackground=CYAN_M)
        body.grid(row=1, column=0, sticky='nsew', padx=8, pady=(0, 8))
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=1)

        self.term = tk.Text(body, bg='#020508', fg=GREEN, font=(FONT, 9), height=8, width=44,
                            wrap='none', bd=0, highlightthickness=0, state='disabled',
                            cursor='arrow', padx=6, pady=4)
        sb = tk.Scrollbar(body, command=self.term.yview, bg=PANEL, troughcolor='#020508',
                          bd=0, highlightthickness=0, width=10)
        self.term.configure(yscrollcommand=sb.set)
        self.term.grid(row=0, column=0, sticky='nsew')
        sb.grid(row=0, column=1, sticky='ns')
        self.term.tag_configure('ts', foreground=CYAN_M)
        self.term.tag_configure('us', foreground=GREEN)
        self.term.tag_configure('cm', foreground=YELLOW)
        self.term.tag_configure('er', foreground=RED)

    def log(self, tag, msg):
        if self.paused and tag == 'us':
            return
        s = int(time.monotonic() - self.t0)
        ts = '%02d:%02d' % (s // 60, s % 60)
        t = self.term
        t.configure(state='normal')
        t.insert('end', '[%s] ' % ts, 'ts')
        t.insert('end', msg + '\n', tag)
        lines = int(t.index('end-1c').split('.')[0])
        if lines > 120:
            t.delete('1.0', '%d.0' % (lines - 100))
        t.see('end')
        t.configure(state='disabled')

    def clear_term(self):
        self.term.configure(state='normal')
        self.term.delete('1.0', 'end')
        self.term.configure(state='disabled')

    def toggle_pause(self):
        self.paused = not self.paused
        self.btn_pause.configure(text='RESUME' if self.paused else 'PAUSE')

    def build_radar(self, parent):
        p = self.panel(parent)
        p.grid(row=0, column=1, sticky='nsew')
        self.canvas = tk.Canvas(p, bg=PANEL, highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        self.canvas.bind('<Configure>', self.on_resize)

    def on_resize(self, e):
        self.cw, self.ch = max(e.width, 200), max(e.height, 200)
        top, bottom, side = 46, 100, 34
        self.cx = self.cw / 2
        self.cy = self.ch - bottom
        self.R = max(40, min(self.cw / 2 - side, self.cy - top - 14))
        self.pxcm = self.R / MAX_CM
        self.draw_grid()

    def pol(self, a, r):
        """0 derajat = kiri, 90 = atas, 180 = kanan (MIRROR_X membaliknya)."""
        if MIRROR_X:
            a = 180 - a
        rad = math.radians(a)
        return self.cx - r * math.cos(rad), self.cy - r * math.sin(rad)

    def draw_grid(self):
        c = self.canvas
        c.delete('grid')
        cx, cy, R = self.cx, self.cy, self.R
        c.create_text(14, 14, anchor='nw', text='SONAR HC-SR04', fill=CYAN,
                      font=(FONT, 11, 'bold'), tags='grid')
        for cm in (30, 60, 90, 120):
            r = cm * self.pxcm
            outer = (cm == MAX_CM)
            c.create_arc(cx - r, cy - r, cx + r, cy + r, start=0, extent=180, style='arc',
                         outline=CYAN_M if outer else GRID, width=2 if outer else 1, tags='grid')
            c.create_text(cx + 4, cy - r + 2, anchor='nw', fill='#4b6b7a', font=(FONT, 7),
                          text=('%dcm' % cm) if outer else str(cm), tags='grid')
        for a in range(0, 181, 30):
            x, y = self.pol(a, R)
            c.create_line(cx, cy, x, y, fill=GRID, tags='grid')
            tx, ty = self.pol(a, R + 14)
            c.create_text(tx, ty, text='%d\u00b0' % a, fill='#6f93a5', font=(FONT, 8), tags='grid')

    def draw_dyn(self, now):
        c = self.canvas
        c.delete('dyn')
        if self.R <= 0:
            return
        cx, cy, R = self.cx, self.cy, self.R

        c.create_text(self.cw - 14, 14, anchor='ne', fill='#cbd5e1', font=(FONT, 10, 'bold'),
                      text='SWEEP %d\u00b0' % self.target_angle, tags='dyn')

        if self.online:
            # jejak sapuan garis hijau
            for g in self.glow:
                op = (1 - (now - g['t']) / GLOW_S) * 0.35
                x, y = self.pol(g['a'], R - 1)
                c.create_line(cx, cy, x, y, fill=blend(GREEN, PANEL, op), width=3, tags='dyn')
            # jejak titik deteksi
            for p in self.trail:
                op = (1 - (now - p['t']) / TRAIL_S) * 0.9
                x, y = self.pol(p['a'], p['d'] * self.pxcm)
                c.create_oval(x - 4, y - 4, x + 4, y + 4, fill=blend(RED, PANEL, op),
                              outline='', tags='dyn')
            # kipas + garis sapuan
            a = self.shown_angle
            pts = [cx, cy]
            for da in (-12, -6, 0, 6, 12):
                x, y = self.pol(a + da, R * 0.98)
                pts += [x, y]
            c.create_polygon(*pts, fill=GREEN, stipple='gray12', outline='', tags='dyn')
            x, y = self.pol(a, R)
            c.create_line(cx, cy, x, y, fill=GREEN, width=2, tags='dyn')
            # blip objek saat ini
            if self.ready and 0 <= self.dist <= MAX_CM:
                x, y = self.pol(a, self.dist * self.pxcm)
                ph = (now * 1.4) % 1.0
                rr = 6 + 10 * ph
                c.create_oval(x - rr, y - rr, x + rr, y + rr, outline=blend(RED, PANEL, 1 - ph),
                              width=2, tags='dyn')
                c.create_oval(x - 5, y - 5, x + 5, y + 5, fill=RED, outline='', tags='dyn')

        # readout jarak besar
        if self.online:
            txt = ('%.1f' % self.dist) if self.dist >= 0 else 'ERR'
        else:
            txt = '--'
        size = max(20, min(40, int(self.ch / 14)))
        yy = self.ch - 46
        c.create_text(cx + 36, yy, anchor='e', text=txt, fill=RED, font=(FONT, size, 'bold'), tags='dyn')
        c.create_text(cx + 44, yy + size * 0.35, anchor='w', text='CM', fill=CYAN,
                      font=(FONT, 12, 'bold'), tags='dyn')

    def set_val(self, key, text, fg=None):
        lbl = self.v[key]
        k = (key, text, fg)
        if self.cache.get(key) == k:
            return
        self.cache[key] = k
        lbl.configure(text=text, fg=fg or CYAN)

    def set_badge(self, lbl, text, color):
        lbl.configure(text='\u25a0 ' + text, fg=color, highlightbackground=color)

    def refresh_badges(self):
        self.set_badge(self.conn_lbl, 'ONLINE' if self.online else 'OFFLINE',
                       GREEN if self.online else RED)
        if not self.online:
            self.set_badge(self.mode_lbl, 'NO LINK', RED)
        elif self.status in ('STANDBY', 'NO DATA', '--'):
            self.set_badge(self.mode_lbl, 'WAITING BRAIN' if self.status == '--' else self.status, YELLOW)
        else:
            self.set_badge(self.mode_lbl, 'AUTONOMOUS \u00b7 ' + self.status, GREEN)

    def draw_rssi(self, v):
        col = GREEN if v >= -60 else YELLOW if v >= -75 else RED
        lvl = 4 if v >= -55 else 3 if v >= -65 else 2 if v >= -75 else 1 if v >= -85 else 0
        self.set_val('rssi', '%d dBm' % v, col)
        self.draw_bars(lvl, col)

    def draw_bars(self, lvl, col):
        b = self.bars
        b.delete('all')
        for i in range(4):
            h = 4 + i * 3
            b.create_rectangle(i * 6, 14 - h, i * 6 + 4, 14,
                               fill=col if i < lvl else '#10303a', outline='')

    def draw_latency(self, ms):
        self.lat_hist.append(ms)
        self.lat_hist = self.lat_hist[-20:]
        col = GREEN if ms < 80 else YELLOW if ms < 250 else RED
        self.set_val('lat', '%d ms' % round(ms), col)
        s = self.spark
        s.delete('all')
        w, h = 44, 16
        top = max(120.0, max(self.lat_hist))
        n = 20
        step = w / (n - 1)
        off = n - len(self.lat_hist)
        pts = []
        for i, v in enumerate(self.lat_hist):
            pts += [(off + i) * step, h - min(h, v / top * h)]
        if len(pts) >= 4:
            s.create_line(*pts, fill=col, width=2)

    def clear_info(self):
        for k in self.v:
            self.set_val(k, '--')
        self.draw_bars(0, GREEN)
        self.spark.delete('all')
        self.lat_hist = []

    @staticmethod
    def fmt_up(s):
        return '%02d:%02d:%02d' % (s // 3600, s % 3600 // 60, s % 60)

    @staticmethod
    def fmt_motor(v):
        return ('+' if v > 0 else '') + '%.2f' % v

    def update_info(self, t, rate):
        i = self.info
        self.set_val('chip', i['chip'])
        self.set_val('cpu', i['cpu'] + ' MHz' if i['cpu'] != '--' else '--')
        self.set_val('ssid', i['ssid'])
        self.set_val('ip', i['ip'])
        self.set_val('bssid', i['bssid'])
        self.set_val('mac', i['mac'])
        self.sdk_lbl.configure(text='micro-ROS . UDP . %s' % i['sdk'] if i['sdk'] != '--' else 'micro-ROS . UDP')
        self.set_val('up', self.fmt_up(t['up']))
        self.set_val('ch', '%02d' % t['ch'])
        self.set_val('heap', '%d KB' % t['heap'])
        self.set_val('rate', '%.1f Hz' % rate)
        self.set_val('ml', self.fmt_motor(t['ml']), YELLOW if t['ml'] else None)
        self.set_val('mr', self.fmt_motor(t['mr']), YELLOW if t['mr'] else None)
        self.set_val('fs', 'CMD LOST' if t['fs'] else 'OK', RED if t['fs'] else GREEN)
        self.draw_rssi(t['rssi'])

    def on_tel(self, rx, t, rate):
        self.last_rx = rx
        self.tel = t
        self.rate = rate
        self.update_info(t, rate)
        if time.monotonic() - self.last_lat_push >= 0.5:      # RTT baru tiap 0.5 dtk dari ESP32
            self.last_lat_push = time.monotonic()
            self.draw_latency(t['rtt'])

        if self.last_fs is None or t['fs'] != self.last_fs:
            self.last_fs = t['fs']
            if t['fs']:
                self.log('er', 'FAILSAFE > no cmd, motors OFF')
            else:
                self.log('cm', 'FAILSAFE > cmd received')

        self.target_angle = max(0, min(180, t['a']))
        self.ready = t['rdy']
        self.dist = t['d']

        now = time.monotonic()
        valid = t['d'] >= 0
        if valid and t['rdy'] and t['d'] <= MAX_CM and now - self.last_trail_add >= 0.1:
            self.last_trail_add = now
            self.trail.append({'a': t['a'], 'd': t['d'], 't': now})
            self.trail = self.trail[-300:]
        if now - self.last_snr_log >= 0.3:
            self.last_snr_log = now
            self.log('us', ('SNR: %.1f cm' % t['d']) if valid else 'SNR: LOSS')

    def on_status(self, s):
        self.status = s
        if self.online and s != self.last_status_logged:
            self.last_status_logged = s
            self.log('cm', 'AUTO > ' + s)
        self.refresh_badges()

    def set_online(self, online):
        self.online = online
        self.log('cm' if online else 'er', 'LINK > ESP32 ONLINE' if online else 'LINK > ESP32 LOST')
        if not online:
            self.clear_info()
            self.ready = False
            self.dist = -1.0
            self.last_fs = None
            self.last_glow_angle = None
            self.last_lat_push = 0.0
            self.last_status_logged = None
        elif self.status != '--':
            self.last_status_logged = self.status
            self.log('cm', 'AUTO > ' + self.status)
        self.refresh_badges()

    def tick(self):
        try:
            self._tick()
        except Exception:                    # jangan sampai loop GUI mati karena 1 error
            import traceback
            traceback.print_exc()
        finally:
            self.root.after(FRAME_MS, self.tick)

    def _tick(self):
        now = time.monotonic()

        # 1) kuras event dari thread ROS
        for _ in range(200):
            try:
                ev = self.q.get_nowait()
            except queue.Empty:
                break
            kind = ev[0]
            if kind == 'info':
                self.info = ev[1]
            elif kind == 'status':
                self.on_status(ev[1])
            elif kind == 'tel':
                _, rx, t, rate = ev
                if not self.online:
                    self.set_online(True)
                self.on_tel(rx, t, rate)

        # 2) deteksi OFFLINE
        if self.online and (now - self.last_rx) > OFFLINE_AFTER_S:
            self.set_online(False)

        # 3) animasi sapuan + jejak
        diff = self.target_angle - self.shown_angle
        self.shown_angle = self.target_angle if abs(diff) < 0.05 else self.shown_angle + diff * 0.35
        if self.online and (self.last_glow_angle is None or
                            abs(self.shown_angle - self.last_glow_angle) >= 0.7):
            self.last_glow_angle = self.shown_angle
            self.glow.append({'a': self.shown_angle, 't': now})
            self.glow = self.glow[-400:]
        self.glow = [g for g in self.glow if now - g['t'] < GLOW_S]
        self.trail = [p for p in self.trail if now - p['t'] < TRAIL_S]

        # 4) gambar
        self.draw_dyn(now)


def main():
    q = queue.Queue(maxsize=1000)

    if SignalHandlerOptions is not None:
        rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    else:
        rclpy.init(args=sys.argv)
    node = RosBridge(q)

    def spin():
        try:
            rclpy.spin(node)
        except Exception:
            pass
    threading.Thread(target=spin, daemon=True).start()

    try:
        root = tk.Tk()
    except tk.TclError as e:
        print('Tidak bisa membuka jendela (DISPLAY?):', e)
        print('Pastikan dijalankan di desktop Ubuntu, dan paket python3-tk sudah terpasang.')
        rclpy.shutdown()
        return
    App(root, q)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
