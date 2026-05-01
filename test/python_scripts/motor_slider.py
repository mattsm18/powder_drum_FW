"""
Motor Control GUI
-----------------
Connects to Nano Every over serial and sets stepper motor angular velocity via slider.
Plots setpoint vs measured angular velocity in real time.
Requires: pip install pyserial matplotlib
"""

import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import math
import collections
import time

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation

# ── Constants ──────────────────────────────────────────────────────────────────
MAX_VEL    =  30
MIN_VEL    = -30
BAUD_RATE  = 115200
HISTORY    = 200
GRAPH_HZ   = 20

BG        = "#0d0d0d"
PANEL     = "#141414"
ACCENT    = "#00e5ff"
ACCENT2   = "#ff4f7b"
DIM       = "#222222"
GRID      = "#1a1a1a"
TEXT      = "#f0f0f0"
SUBTEXT   = "#555555"
FONT_MONO = ("Courier New", 11)
FONT_BIG  = ("Courier New", 32, "bold")
FONT_LBL  = ("Courier New", 9)


class MotorControlApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("POWDER DRUM — MOTOR CONTROL")
        self.configure(bg=BG)
        self.resizable(False, False)

        self.serial_conn = None
        self.connected   = False
        self.send_lock   = threading.Lock()

        self._times     = collections.deque(maxlen=HISTORY)
        self._setpoints = collections.deque(maxlen=HISTORY)
        self._measured  = collections.deque(maxlen=HISTORY)
        self._t0        = time.monotonic()
        self._setpoint  = 0.0

        self._build_ui()
        self._build_graph()
        self._refresh_ports()
        self._start_serial_reader()

    # ── UI ─────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # Header
        hdr = tk.Frame(self, bg=BG, pady=14)
        hdr.pack(fill="x", padx=32)
        tk.Label(hdr, text="POWDER DRUM", font=("Courier New", 13, "bold"),
                 bg=BG, fg=ACCENT).pack(side="left")
        tk.Label(hdr, text="motor control v1.1", font=FONT_LBL,
                 bg=BG, fg=SUBTEXT).pack(side="left", padx=12)
        self.status_dot = tk.Label(hdr, text="●", font=("Courier New", 14),
                                   bg=BG, fg=DIM)
        self.status_dot.pack(side="right")
        self.status_lbl = tk.Label(hdr, text="DISCONNECTED", font=FONT_LBL,
                                   bg=BG, fg=SUBTEXT)
        self.status_lbl.pack(side="right", padx=6)

        tk.Frame(self, bg=DIM, height=1).pack(fill="x", padx=32)

        # Connection panel
        conn = tk.Frame(self, bg=PANEL, padx=24, pady=14)
        conn.pack(fill="x", padx=32, pady=12)
        tk.Label(conn, text="PORT", font=FONT_LBL, bg=PANEL, fg=SUBTEXT).grid(row=0, column=0, sticky="w")
        tk.Label(conn, text="BAUD", font=FONT_LBL, bg=PANEL, fg=SUBTEXT).grid(row=0, column=1, sticky="w", padx=(16,0))

        self.port_var = tk.StringVar()
        self.port_cb  = ttk.Combobox(self, textvariable=self.port_var,
                                     width=18, state="readonly", font=FONT_MONO)
        self.port_cb.grid(in_=conn, row=1, column=0, sticky="w")

        self.baud_var = tk.StringVar(value=str(BAUD_RATE))
        tk.Entry(conn, textvariable=self.baud_var, width=10, font=FONT_MONO,
                 bg=BG, fg=TEXT, insertbackground=ACCENT, relief="flat",
                 highlightthickness=1, highlightbackground=DIM).grid(
                     row=1, column=1, sticky="w", padx=(16,0))

        self.connect_btn = tk.Button(
            conn, text="CONNECT", font=FONT_LBL, bg=ACCENT, fg=BG,
            relief="flat", padx=16, pady=6, cursor="hand2",
            activebackground="#00b8cc", activeforeground=BG,
            command=self._toggle_connection)
        self.connect_btn.grid(row=1, column=2, padx=(16,0))

        tk.Button(conn, text="↺", font=("Courier New", 13), bg=PANEL, fg=SUBTEXT,
                  relief="flat", cursor="hand2", activebackground=PANEL,
                  command=self._refresh_ports).grid(row=1, column=3, padx=(8,0))

        tk.Frame(self, bg=DIM, height=1).pack(fill="x", padx=32)

        # Velocity display — setpoint left, measured right
        disp = tk.Frame(self, bg=BG, pady=16)
        disp.pack(fill="x", padx=32)

        left = tk.Frame(disp, bg=BG)
        left.pack(side="left", expand=True)
        right = tk.Frame(disp, bg=BG)
        right.pack(side="right", expand=True)

        tk.Label(left, text="SETPOINT", font=FONT_LBL, bg=BG, fg=ACCENT).pack()
        sp_row = tk.Frame(left, bg=BG)
        sp_row.pack()
        self.vel_lbl = tk.Label(sp_row, text=" 0.00", font=FONT_BIG, bg=BG, fg=TEXT)
        self.vel_lbl.pack(side="left")
        tk.Label(sp_row, text=" rad/s", font=("Courier New", 12),
                 bg=BG, fg=SUBTEXT).pack(side="left", pady=(10,0))

        tk.Label(right, text="MEASURED", font=FONT_LBL, bg=BG, fg=ACCENT2).pack()
        meas_row = tk.Frame(right, bg=BG)
        meas_row.pack()
        self.meas_lbl = tk.Label(meas_row, text=" 0.00", font=FONT_BIG, bg=BG, fg=TEXT)
        self.meas_lbl.pack(side="left")
        tk.Label(meas_row, text=" rad/s", font=("Courier New", 12),
                 bg=BG, fg=SUBTEXT).pack(side="left", pady=(10,0))

        # Slider
        sf = tk.Frame(self, bg=BG, padx=32, pady=4)
        sf.pack(fill="x")
        tk.Label(sf, text=f"{MIN_VEL:.1f}", font=FONT_LBL, bg=BG, fg=SUBTEXT).pack(side="left")
        tk.Label(sf, text=f"{MAX_VEL:.1f}", font=FONT_LBL, bg=BG, fg=SUBTEXT).pack(side="right")

        self.vel_var = tk.DoubleVar(value=0.0)
        tk.Scale(self, from_=MIN_VEL, to=MAX_VEL, resolution=0.01,
                 orient="horizontal", variable=self.vel_var,
                 bg=BG, fg=TEXT, troughcolor=DIM, activebackground=ACCENT,
                 highlightthickness=0, showvalue=False, sliderlength=24,
                 command=self._on_slider).pack(fill="x", padx=32)

        # Quick buttons
        bf = tk.Frame(self, bg=BG, pady=12)
        bf.pack()
        for label, value in [("−2π", -2*math.pi), ("−π", -math.pi),
                              ("STOP", 0), ("+π", math.pi), ("+2π", 2*math.pi)]:
            is_stop = label == "STOP"
            tk.Button(bf, text=label, font=FONT_LBL,
                      bg=("#ff3b5c" if is_stop else PANEL),
                      fg=(BG if is_stop else TEXT),
                      relief="flat", padx=14, pady=8, cursor="hand2",
                      activebackground=(ACCENT if not is_stop else "#cc2f4a"),
                      activeforeground=BG,
                      command=lambda v=value: self._set_velocity(v)
                      ).pack(side="left", padx=4)

        tk.Frame(self, bg=DIM, height=1).pack(fill="x", padx=32, pady=(8,0))

    def _build_graph(self):
        gf = tk.Frame(self, bg=BG, padx=32, pady=8)
        gf.pack(fill="x")

        legend_row = tk.Frame(gf, bg=BG)
        legend_row.pack(anchor="w", pady=(0,4))
        tk.Label(legend_row, text="VELOCITY TRACE", font=FONT_LBL,
                 bg=BG, fg=SUBTEXT).pack(side="left")
        tk.Label(legend_row, text="  ── setpoint", font=FONT_LBL,
                 bg=BG, fg=ACCENT).pack(side="left", padx=(16,0))
        tk.Label(legend_row, text="  ╌╌ measured", font=FONT_LBL,
                 bg=BG, fg=ACCENT2).pack(side="left", padx=(8,0))

        fig = Figure(figsize=(6.8, 2.4), dpi=100, facecolor=BG)
        self._ax = fig.add_subplot(111)
        self._ax.set_facecolor(BG)
        fig.subplots_adjust(left=0.08, right=0.98, top=0.95, bottom=0.2)

        self._ax.tick_params(colors=SUBTEXT, labelsize=7)
        for spine in self._ax.spines.values():
            spine.set_edgecolor(DIM)
        self._ax.set_ylabel("rad/s", fontsize=7, color=SUBTEXT)
        self._ax.set_xlabel("time (s)", fontsize=7, color=SUBTEXT)
        self._ax.grid(True, color=GRID, linewidth=0.5)
        self._ax.set_ylim(MIN_VEL, MAX_VEL)
        self._ax.axhline(0, color=DIM, linewidth=0.8)

        self._line_sp,   = self._ax.plot([], [], color=ACCENT,  linewidth=1.4)
        self._line_meas, = self._ax.plot([], [], color=ACCENT2, linewidth=1.4,
                                          linestyle="--")

        canvas = FigureCanvasTkAgg(fig, master=gf)
        canvas.get_tk_widget().pack(fill="x")
        self._canvas = canvas

        self._ani = animation.FuncAnimation(
            fig, self._update_graph,
            interval=int(1000 / GRAPH_HZ), blit=True, cache_frame_data=False)

    def _update_graph(self, _):
        if not self._times:
            return self._line_sp, self._line_meas

        ts  = list(self._times)
        sps = list(self._setpoints)
        ms  = list(self._measured)

        self._line_sp.set_data(ts, sps)
        self._line_meas.set_data(ts, ms)

        if len(ts) > 1:
            self._ax.set_xlim(ts[0], max(ts[-1], ts[0] + 1.0))

        return self._line_sp, self._line_meas

    # ── Serial ─────────────────────────────────────────────────────────────────

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb["values"] = ports
        if ports:
            self.port_var.set(ports[0])

    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        baud = int(self.baud_var.get())
        try:
            self.serial_conn = serial.Serial(port, baud, timeout=1)
            self.connected   = True
            self.connect_btn.config(text="DISCONNECT", bg="#ff3b5c", fg=TEXT)
            self.status_dot.config(fg=ACCENT)
            self.status_lbl.config(text="CONNECTED", fg=ACCENT)
            self._t0 = time.monotonic()
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))

    def _disconnect(self):
        self._set_velocity(0.0)
        if self.serial_conn:
            self.serial_conn.close()
        self.connected = False
        self.connect_btn.config(text="CONNECT", bg=ACCENT, fg=BG)
        self.status_dot.config(fg=DIM)
        self.status_lbl.config(text="DISCONNECTED", fg=SUBTEXT)

    def _start_serial_reader(self):
        """Background thread — reads measured velocity lines from Arduino."""
        def _reader():
            while True:
                if self.connected and self.serial_conn:
                    try:
                        line = self.serial_conn.readline().decode("utf-8").strip()
                        if line:
                            val = float(line)
                            t   = time.monotonic() - self._t0
                            self._times.append(t)
                            self._setpoints.append(self._setpoint)
                            self._measured.append(val)
                            self.after(0, self.meas_lbl.config, {"text": f"{val:+.2f}"})
                    except (ValueError, UnicodeDecodeError):
                        pass
                    except Exception:
                        pass
                else:
                    time.sleep(0.05)

        threading.Thread(target=_reader, daemon=True).start()

    def _send(self, value: float):
        if not self.connected or not self.serial_conn:
            return
        with self.send_lock:
            try:
                self.serial_conn.write(f"{value:.4f}\n".encode())
            except Exception:
                pass

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _on_slider(self, _=None):
        v = self.vel_var.get()
        self._setpoint = v
        self.vel_lbl.config(text=f"{v:+.2f}")
        self._send(v)

    def _set_velocity(self, value: float):
        self.vel_var.set(value)
        self._setpoint = value
        self.vel_lbl.config(text=f"{value:+.2f}")
        self._send(value)

    def on_close(self):
        if self.connected:
            self._disconnect()
        self.destroy()


if __name__ == "__main__":
    app = MotorControlApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()