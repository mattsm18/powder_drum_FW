"""
Motor Control GUI
-----------------
Connects to Nano Every over serial and sets stepper motor angular velocity via slider.
Plots any comma-separated values from serial in real time (like Arduino Serial Plotter).
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
FONT_BIG  = ("Courier New", 22, "bold")
FONT_SML  = ("Courier New", 11)
FONT_LBL  = ("Courier New", 9)

# Fixed channel definitions — label, colour, linestyle
CHANNEL_DEFS = [
    ("Input setpoint",        "#00e5ff", "-"),   # ch0 — cyan solid
    ("Ramped setpoint",       "#ffb300", "-"),   # ch1 — amber solid
    ("Measured angular vel.", "#ff4f7b", "-"),   # ch2 — red solid
]

EXTRA_COLOURS = ["#a8ff3e", "#bf5af2", "#ff6d00", "#00e676", "#ff1744"]


class MotorControlApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("POWDER DRUM — MOTOR CONTROL")
        self.configure(bg=BG)
        self.resizable(False, False)

        self._conn_lock  = threading.Lock()
        self._serial     = None
        self.connected   = False
        self.send_lock   = threading.Lock()

        self._times      = collections.deque(maxlen=HISTORY)
        self._t0         = time.monotonic()
        self._setpoint   = 0.0

        self._data_lock        = threading.Lock()
        self._channel_data     = {}
        self._channel_labels   = {}
        self._pending_channels = []

        self._channel_lines  = {}
        self._legend_labels  = {}

        # Latest values for numerical readouts (rad/s), one per defined channel
        self._latest = [0.0] * len(CHANNEL_DEFS)

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
        tk.Label(hdr, text="motor control v1.4", font=FONT_LBL,
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

        # ── Setpoint control: entry + slider side by side ──
        ctrl = tk.Frame(self, bg=PANEL, padx=24, pady=14)
        ctrl.pack(fill="x", padx=32, pady=12)
        ctrl.columnconfigure(1, weight=1)

        # Entry column
        entry_col = tk.Frame(ctrl, bg=PANEL)
        entry_col.grid(row=0, column=0, sticky="w")

        tk.Label(entry_col, text="SETPOINT (rad/s)", font=FONT_LBL,
                 bg=PANEL, fg=SUBTEXT).pack(anchor="w")

        entry_row = tk.Frame(entry_col, bg=PANEL)
        entry_row.pack(anchor="w", pady=(4,0))

        self.entry_var = tk.StringVar(value="0.0")
        self.entry = tk.Entry(entry_row, textvariable=self.entry_var, width=10,
                              font=FONT_MONO, bg=BG, fg=TEXT,
                              insertbackground=ACCENT, relief="flat",
                              highlightthickness=1, highlightbackground=DIM)
        self.entry.pack(side="left")
        self.entry.bind("<Return>",   self._on_entry)
        self.entry.bind("<KP_Enter>", self._on_entry)

        tk.Button(entry_row, text="SET", font=FONT_LBL, bg=ACCENT, fg=BG,
                  relief="flat", padx=10, pady=4, cursor="hand2",
                  activebackground="#00b8cc", activeforeground=BG,
                  command=self._on_entry).pack(side="left", padx=(8,0))

        # Slider column
        slider_col = tk.Frame(ctrl, bg=PANEL)
        slider_col.grid(row=0, column=1, sticky="ew", padx=(32,0))

        tk.Label(slider_col, text="SLIDER", font=FONT_LBL,
                 bg=PANEL, fg=SUBTEXT).pack(anchor="w")

        slider_row = tk.Frame(slider_col, bg=PANEL)
        slider_row.pack(fill="x", pady=(4,0))

        tk.Label(slider_row, text=f"{MIN_VEL:.0f}", font=FONT_LBL,
                 bg=PANEL, fg=SUBTEXT).pack(side="left")
        self.vel_var = tk.DoubleVar(value=0.0)
        tk.Scale(slider_row, from_=MIN_VEL, to=MAX_VEL, resolution=0.01,
                 orient="horizontal", variable=self.vel_var,
                 bg=PANEL, fg=TEXT, troughcolor=DIM, activebackground=ACCENT,
                 highlightthickness=0, showvalue=False, sliderlength=24,
                 command=self._on_slider).pack(side="left", fill="x", expand=True)
        tk.Label(slider_row, text=f"{MAX_VEL:.0f}", font=FONT_LBL,
                 bg=PANEL, fg=SUBTEXT).pack(side="left")

        # Quick buttons
        bf = tk.Frame(self, bg=BG, pady=10)
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

        # ── Numerical readouts: one column per channel ──
        readout = tk.Frame(self, bg=BG, pady=12)
        readout.pack(fill="x", padx=32)

        self._readout_rads = []
        self._readout_rpms = []

        for i, (label, colour, _) in enumerate(CHANNEL_DEFS):
            col = tk.Frame(readout, bg=BG)
            col.pack(side="left", expand=True)

            tk.Label(col, text=label.upper(), font=FONT_LBL,
                     bg=BG, fg=colour).pack()

            rads_lbl = tk.Label(col, text=" 0.00 rad/s", font=FONT_BIG,
                                bg=BG, fg=TEXT)
            rads_lbl.pack()

            rpm_lbl = tk.Label(col, text=" 0.00 RPM", font=FONT_SML,
                               bg=BG, fg=SUBTEXT)
            rpm_lbl.pack()

            self._readout_rads.append(rads_lbl)
            self._readout_rpms.append(rpm_lbl)

        tk.Frame(self, bg=DIM, height=1).pack(fill="x", padx=32, pady=(8,0))

    def _build_graph(self):
        gf = tk.Frame(self, bg=BG, padx=32, pady=8)
        gf.pack(fill="x")

        legend_row = tk.Frame(gf, bg=BG)
        legend_row.pack(anchor="w", pady=(0,4))
        tk.Label(legend_row, text="ANGULAR VELOCITY OVER TIME", font=FONT_LBL,
                 bg=BG, fg=SUBTEXT).pack(side="left")
        for label, colour, _ in CHANNEL_DEFS:
            tk.Label(legend_row, text=f"  ── {label}",
                     font=FONT_LBL, bg=BG, fg=colour).pack(side="left", padx=(8,0))

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
        self._ax.axhline(0, color=DIM, linewidth=0.8)

        canvas = FigureCanvasTkAgg(fig, master=gf)
        canvas.get_tk_widget().pack(fill="x")
        self._canvas = canvas
        self._fig    = fig

        self._ani = animation.FuncAnimation(
            fig, self._update_graph,
            interval=int(1000 / GRAPH_HZ), blit=False, cache_frame_data=False)

    # ── Channel management (UI thread only) ────────────────────────────────────

    def _flush_pending_channels(self):
        with self._data_lock:
            pending = self._pending_channels[:]
            self._pending_channels.clear()

        for idx, label in pending:
            if idx in self._channel_lines:
                continue
            if idx < len(CHANNEL_DEFS):
                _, colour, linestyle = CHANNEL_DEFS[idx]
            else:
                ei = idx - len(CHANNEL_DEFS)
                colour    = EXTRA_COLOURS[ei % len(EXTRA_COLOURS)]
                linestyle = "-"

            line, = self._ax.plot([], [], color=colour, linewidth=1.4,
                                  linestyle=linestyle, label=label)
            self._channel_lines[idx] = line

    def _update_graph(self, _):
        self._flush_pending_channels()

        with self._data_lock:
            if not self._times or not self._channel_data:
                return

            ts = list(self._times)

            for idx, line in self._channel_lines.items():
                if idx not in self._channel_data:
                    continue
                data = list(self._channel_data[idx])
                if len(data) < len(ts):
                    data = [float('nan')] * (len(ts) - len(data)) + data
                line.set_data(ts, data)

            if len(ts) > 1:
                self._ax.set_xlim(ts[0], max(ts[-1], ts[0] + 1.0))

            all_vals = []
            for idx in self._channel_data:
                all_vals.extend(v for v in self._channel_data[idx]
                                if not math.isnan(v))
            if all_vals:
                margin = max(abs(max(all_vals) - min(all_vals)) * 0.1, 1.0)
                self._ax.set_ylim(min(all_vals) - margin,
                                  max(all_vals) + margin)

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
        if not port:
            messagebox.showwarning("No port", "Select a serial port first.")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showwarning("Bad baud", "Enter a valid baud rate.")
            return

        try:
            conn = serial.Serial(port, baud, timeout=1)
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            return

        with self._data_lock:
            self._channel_data.clear()
            self._channel_labels.clear()
            self._pending_channels.clear()
            self._times.clear()

        for line in self._channel_lines.values():
            line.remove()
        self._channel_lines.clear()
        for lbl in self._legend_labels.values():
            lbl.destroy()
        self._legend_labels.clear()

        self._t0 = time.monotonic()

        with self._conn_lock:
            self._serial   = conn
            self.connected = True

        self.connect_btn.config(text="DISCONNECT", bg="#ff3b5c", fg=TEXT)
        self.status_dot.config(fg=ACCENT)
        self.status_lbl.config(text="CONNECTED", fg=ACCENT)

    def _disconnect(self):
        self._set_velocity(0.0)

        with self._conn_lock:
            conn           = self._serial
            self._serial   = None
            self.connected = False

        if conn:
            try:
                conn.close()
            except Exception:
                pass

        self.connect_btn.config(text="CONNECT", bg=ACCENT, fg=BG)
        self.status_dot.config(fg=DIM)
        self.status_lbl.config(text="DISCONNECTED", fg=SUBTEXT)

    def _parse_line(self, line: str):
        parts   = [p.strip() for p in line.split(",")]
        results = []
        for part in parts:
            if ":" in part:
                label, _, val_str = part.partition(":")
                try:
                    results.append((label.strip(), float(val_str.strip())))
                except ValueError:
                    pass
            else:
                try:
                    results.append((None, float(part)))
                except ValueError:
                    pass
        return results

    def _start_serial_reader(self):
        def _reader():
            while True:
                with self._conn_lock:
                    conn      = self._serial
                    connected = self.connected

                if not connected or conn is None:
                    time.sleep(0.05)
                    continue

                try:
                    raw = conn.readline().decode("utf-8").strip()
                except Exception:
                    time.sleep(0.05)
                    continue

                if not raw:
                    continue

                parsed = self._parse_line(raw)
                if not parsed:
                    continue

                t = time.monotonic() - self._t0

                with self._data_lock:
                    self._times.append(t)

                    for idx, (label, value) in enumerate(parsed):
                        if label and idx not in self._channel_labels:
                            self._channel_labels[idx] = label

                        if idx not in self._channel_data:
                            self._channel_data[idx] = collections.deque(maxlen=HISTORY)
                            if idx < len(CHANNEL_DEFS):
                                resolved = CHANNEL_DEFS[idx][0]
                            else:
                                resolved = self._channel_labels.get(idx, f"ch{idx}")
                            self._pending_channels.append((idx, resolved))

                        self._channel_data[idx].append(value)

                        if idx < len(self._latest):
                            self._latest[idx] = value

                self.after(0, self._update_readouts)

        threading.Thread(target=_reader, daemon=True).start()

    def _update_readouts(self):
        for i, (rads_lbl, rpm_lbl) in enumerate(
                zip(self._readout_rads, self._readout_rpms)):
            v   = self._latest[i] if i < len(self._latest) else 0.0
            rpm = v * 60.0 / (2.0 * math.pi)
            rads_lbl.config(text=f"{v:+.2f} rad/s")
            rpm_lbl.config(text=f"{rpm:+.2f} Roller RPM | {(rpm / 4):+.2f} Drum RPM")

    def _send(self, value: float):
        with self._conn_lock:
            conn      = self._serial
            connected = self.connected
        if not connected or conn is None:
            return
        with self.send_lock:
            try:
                conn.write(f"{value:.4f}\n".encode())
            except Exception:
                pass

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _on_entry(self, _=None):
        try:
            v = float(self.entry_var.get())
        except ValueError:
            return
        v = max(MIN_VEL, min(MAX_VEL, v))
        self._set_velocity(v)

    def _on_slider(self, _=None):
        v = self.vel_var.get()
        self._setpoint = v
        self.entry_var.set(f"{v:.2f}")
        self._send(v)

    def _set_velocity(self, value: float):
        self.vel_var.set(value)
        self._setpoint = value
        self.entry_var.set(f"{value:.2f}")
        self._send(value)

    def on_close(self):
        if self.connected:
            self._disconnect()
        self.destroy()


if __name__ == "__main__":
    app = MotorControlApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()