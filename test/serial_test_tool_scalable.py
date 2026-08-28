#!/usr/bin/env python3
"""Standalone serial test tool for the PC <-> MCU comms protocol.

Re-implements framing/CRC locally (no dependency on src/). Loads protocol JSON
at runtime. Supports GET/SET, raw packet injection, TX/RX logging, an
automated edge-case suite, and a live multi-parameter poll/graph view.

Requires: pyserial  (pip install pyserial)
Usage:    python test/serial_test_tool.py [--protocol path/to/protocol.json]
"""

from __future__ import annotations

import argparse
import json
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk
from typing import Callable

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial is required: pip install pyserial")
    sys.exit(1)

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_PROTOCOL = REPO_ROOT / "config" / "pd_comms_protocol_v1.1.json"
BOARD_RESET_SETTLE_MS = 2000
DEFAULT_TEST_WAIT_MS = 400
LOG_FONT = ("TkFixedFont", 10)

# Official Raspberry Pi 7" touchscreen panel resolution (landscape).
PI_TOUCHSCREEN_W = 800
PI_TOUCHSCREEN_H = 480

# Palette cycled through for graphed parameters (extended if more are selected)
GRAPH_COLORS = [
    "#00AAFF", "#FF5555", "#55FF88", "#FFCC00",
    "#CC66FF", "#FF8800", "#00FFCC", "#FF66AA",
    "#AAAAAA", "#66CCFF",
]


# ---------------------------------------------------------------------------
# Protocol definition
# ---------------------------------------------------------------------------

@dataclass
class Param:
    id: int
    name: str
    label: str
    access: str
    type: str
    default: float
    min: float | int | None
    max: float | int | None
    unit: str
    group: str

    @property
    def readable(self) -> bool:
        return "r" in self.access

    @property
    def writable(self) -> bool:
        return "w" in self.access

    @property
    def bounded(self) -> bool:
        """True only if both min and max are present. Partial bounds (one
        set, one missing) are treated as unbounded rather than guessed at —
        a lone min/max without its counterpart isn't enough to validate
        against safely."""
        return self.min is not None and self.max is not None


@dataclass
class ProtocolDef:
    path: Path
    version: str
    sof: int
    proto_version: int
    header_size: int
    max_payload: int
    timeout_ms: int
    max_retries: int
    dir_by_name: dict[str, int]
    dir_by_value: dict[int, str]
    msgid_by_name: dict[str, int]
    msgid_by_value: dict[int, str]
    nack_by_name: dict[str, int]
    nack_by_value: dict[int, str]
    params_by_id: dict[int, Param]
    params_by_name: dict[str, Param]

    @staticmethod
    def load(path: Path) -> ProtocolDef:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)

        frame = data["frame"]
        dir_by_name = {k: int(v, 16) for k, v in data["direction"].items()}
        msgid_by_name = {k: int(v, 16) for k, v in data["msg_id"].items()}
        nack_by_name = {k: int(v, 16) for k, v in data["nack_error"].items()}

        params_by_id: dict[int, Param] = {}
        params_by_name: dict[str, Param] = {}
        for p in data["parameters"]:
            param = Param(
                id=int(p["id"], 16),
                name=p["name"],
                label=p["label"],
                access="".join(p["access"]),
                type=p["type"],
                default=p["default"],
                min=p.get("min"),
                max=p.get("max"),
                unit=p["unit"],
                group=p["group"],
            )
            params_by_id[param.id] = param
            params_by_name[param.name] = param

        return ProtocolDef(
            path=path,
            version=data.get("protocol_version", "?"),
            sof=int(frame["sof_byte"], 16),
            proto_version=int(frame["version"], 16),
            header_size=frame["header_size_bytes"],
            max_payload=frame["max_payload_bytes"],
            timeout_ms=frame.get("timeout_ms", 100),
            max_retries=frame.get("max_retries", 3),
            dir_by_name=dir_by_name,
            dir_by_value={v: k for k, v in dir_by_name.items()},
            msgid_by_name=msgid_by_name,
            msgid_by_value={v: k for k, v in msgid_by_name.items()},
            nack_by_name=nack_by_name,
            nack_by_value={v: k for k, v in nack_by_name.items()},
            params_by_id=params_by_id,
            params_by_name=params_by_name,
        )


# ---------------------------------------------------------------------------
# Framing / CRC (local copy — not imported from firmware library)
# ---------------------------------------------------------------------------

def compute_crc(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
    return crc


def build_packet(
    proto: ProtocolDef,
    msg_id: int,
    direction: int,
    payload: bytes,
    *,
    corrupt_crc: bool = False,
    version_override: int | None = None,
    length_override: int | None = None,
) -> bytes:
    version = proto.proto_version if version_override is None else version_override
    length = len(payload) if length_override is None else length_override
    header = bytes([proto.sof, version, msg_id, direction, length])
    crc = compute_crc(header + payload)
    if corrupt_crc:
        crc = (crc + 1) & 0xFF
    return header + payload + bytes([crc])


def build_get(proto: ProtocolDef, param_id: int) -> bytes:
    return build_packet(
        proto,
        proto.msgid_by_name["CMD_GET"],
        proto.dir_by_name["PC_TO_MCU"],
        bytes([param_id]),
    )


def build_set(proto: ProtocolDef, param_id: int, value: float) -> bytes:
    payload = bytes([param_id]) + struct.pack("<f", value)
    return build_packet(
        proto,
        proto.msgid_by_name["CMD_SET"],
        proto.dir_by_name["PC_TO_MCU"],
        payload,
    )


def extract_frames(buf: bytearray, proto: ProtocolDef) -> list[bytes]:
    frames: list[bytes] = []
    while True:
        idx = buf.find(bytes([proto.sof]))
        if idx == -1:
            buf.clear()
            break
        if idx > 0:
            del buf[:idx]
        if len(buf) < proto.header_size:
            break
        length = buf[proto.header_size - 1]
        total = proto.header_size + length + 1
        if len(buf) < total:
            break
        frames.append(bytes(buf[:total]))
        del buf[:total]
    return frames


# ---------------------------------------------------------------------------
# Frame parsing
# ---------------------------------------------------------------------------

@dataclass
class FrameInfo:
    raw: bytes
    version: int
    msg_id: int
    msg_name: str
    direction: int
    dir_name: str
    length: int
    payload: bytes
    crc_received: int | None
    crc_computed: int
    crc_ok: bool
    version_ok: bool
    detail: str = ""
    nack_error_name: str | None = None
    acked_msg_name: str | None = None


def _param_name(proto: ProtocolDef, param_id: int) -> str:
    p = proto.params_by_id.get(param_id)
    return p.name if p else f"0x{param_id:02X}(unknown)"


def _decode_payload(proto: ProtocolDef, msg_name: str, payload: bytes) -> str:
    try:
        if msg_name == "CMD_GET" and len(payload) >= 5:
            pid, val = payload[0], struct.unpack("<f", payload[1:5])[0]
            return f"param={_param_name(proto, pid)} value={val:.4f}"
        if msg_name == "CMD_GET" and len(payload) == 1:
            return f"GET request param={_param_name(proto, payload[0])}"
        if msg_name == "CMD_SET" and len(payload) >= 5:
            pid, val = payload[0], struct.unpack("<f", payload[1:5])[0]
            return f"param={_param_name(proto, pid)} value={val:.4f}"
        if msg_name == "STATUS" and len(payload) >= 1:
            return f"status_code=0x{payload[0]:02X}"
        if msg_name == "ACK" and len(payload) >= 1:
            acked = proto.msgid_by_value.get(payload[0], f"0x{payload[0]:02X}")
            return f"acked={acked}"
        if msg_name == "NACK" and len(payload) >= 2:
            nacked = proto.msgid_by_value.get(payload[0], f"0x{payload[0]:02X}")
            err = proto.nack_by_value.get(payload[1], f"0x{payload[1]:02X}")
            return f"nacked={nacked} error={err}"
        if msg_name == "HEARTBEAT":
            return ""
    except (struct.error, IndexError):
        return "(payload decode error)"

    if payload:
        return "payload=[" + " ".join(f"{b:02X}" for b in payload) + "]"
    return ""


def parse_frame(proto: ProtocolDef, frame: bytes) -> FrameInfo | None:
    if len(frame) < proto.header_size + 1:
        return None

    version, msg_id, direction, length = frame[1], frame[2], frame[3], frame[4]
    payload = frame[5 : 5 + length]
    crc_received = frame[5 + length] if len(frame) > 5 + length else None
    crc_computed = compute_crc(frame[: 5 + length])

    msg_name = proto.msgid_by_value.get(msg_id, f"UNKNOWN(0x{msg_id:02X})")
    dir_name = proto.dir_by_value.get(direction, f"0x{direction:02X}")

    info = FrameInfo(
        raw=frame,
        version=version,
        msg_id=msg_id,
        msg_name=msg_name,
        direction=direction,
        dir_name=dir_name,
        length=length,
        payload=payload,
        crc_received=crc_received,
        crc_computed=crc_computed,
        crc_ok=(crc_received == crc_computed),
        version_ok=(version == proto.proto_version),
    )
    info.detail = _decode_payload(proto, msg_name, payload)

    if msg_name == "NACK" and len(payload) >= 2:
        info.nack_error_name = proto.nack_by_value.get(payload[1], f"0x{payload[1]:02X}")
    if msg_name == "ACK" and len(payload) >= 1:
        info.acked_msg_name = proto.msgid_by_value.get(payload[0], f"0x{payload[0]:02X}")

    return info


def decode_frame(proto: ProtocolDef, frame: bytes) -> str:
    info = parse_frame(proto, frame)
    if info is None:
        return "malformed (shorter than header+crc)"

    parts = [
        f"ver=0x{info.version:02X}",
        f"msg={info.msg_name}",
        f"dir={info.dir_name}",
        f"len={info.length}",
    ]
    if not info.version_ok:
        parts.append("VERSION MISMATCH")
    if not info.crc_ok:
        recv = f"0x{info.crc_received:02X}" if info.crc_received is not None else "?"
        parts.append(f"CRC MISMATCH (recv={recv} calc=0x{info.crc_computed:02X})")
    if info.detail:
        parts.append(info.detail)
    return "  |  ".join(parts)


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def parse_hex_bytes(text: str) -> bytes:
    text = text.strip()
    if not text:
        return b""
    return bytes(int(b, 16) for b in text.split())


# ---------------------------------------------------------------------------
# Edge-case suite
# ---------------------------------------------------------------------------

@dataclass
class EdgeCase:
    name: str
    build: Callable[[], bytes]
    match: Callable[[FrameInfo], bool] | None
    wait_ms: int = DEFAULT_TEST_WAIT_MS


# ---------------------------------------------------------------------------
# Serial reader thread
# ---------------------------------------------------------------------------

class SerialReader(threading.Thread):
    def __init__(self, ser: serial.Serial, rx_queue: queue.Queue) -> None:
        super().__init__(daemon=True)
        self._ser = ser
        self._rx_queue = rx_queue
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        while not self._stop.is_set():
            try:
                data = self._ser.read(self._ser.in_waiting or 1)
            except (serial.SerialException, OSError) as exc:
                self._rx_queue.put(("error", str(exc)))
                return
            if data:
                self._rx_queue.put(("raw", bytes(data)))


# ---------------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------------

class SerialTestTool(tk.Tk):
    def __init__(self, initial_protocol: Path) -> None:
        super().__init__()
        self.title("PD Comms Serial Test Tool")
        # Fixed to the official Raspberry Pi 7" touchscreen panel (800x480).
        # Not resizable: on a dedicated touch panel there's no benefit to
        # resizing, and it removes the risk of an accidental drag leaving
        # the layout in a half-sized, half-usable state.
        self.geometry(f"{PI_TOUCHSCREEN_W}x{PI_TOUCHSCREEN_H}")
        self.minsize(PI_TOUCHSCREEN_W, PI_TOUCHSCREEN_H)
        self.resizable(False, False)
        self._fullscreen = False
        self.bind("<F11>", self._toggle_fullscreen)
        self.bind("<Escape>", self._exit_fullscreen)

        self.proto: ProtocolDef | None = None
        self.ser: serial.Serial | None = None
        self.reader: SerialReader | None = None
        self.rx_queue: queue.Queue = queue.Queue()
        self._rx_buf = bytearray()
        self._recent_frames: list[tuple[float, FrameInfo]] = []
        self._board_ready = False
        self._edge_cases: list[EdgeCase] = []
        self._edge_case_results: list[bool] = []

        # -- Live graph state --
        self._graph_checkbox_vars: dict[int, tk.BooleanVar] = {}  # param id -> ticked state
        self._graph_swatches: dict[int, tk.Canvas] = {}            # param id -> colour swatch widget
        self._graph_selected_ids: list[int] = []      # currently polled/plotted param ids (ticked)
        self._graph_data: dict[int, deque] = {}       # param id -> deque[(elapsed_s, value)]
        self._graph_colors: dict[int, str] = {}        # param id -> hex colour
        self._graph_running = False
        self._graph_start_time: float | None = None
        self._graph_after_id: str | None = None
        self._graph_interval_ms = 100
        self._graph_poll_index = 0

        self._configure_styles()

        # Fixed-size touch layout: a slim two-row status/control bar on top
        # (protocol + connection, rarely touched once set up) and a full-width
        # tabbed notebook underneath that gets all the remaining screen. Tabs
        # replace the old side-by-side panes — dragging a thin sash is hard
        # to hit with a finger, so each section now gets the whole width.
        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=0)
        self.rowconfigure(1, weight=1)

        self._build_top_bar(initial_protocol)
        self._build_notebook()

        self._load_protocol()
        self._refresh_ports()
        self.after(50, self._poll_rx_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _toggle_fullscreen(self, _event: tk.Event | None = None) -> None:
        """F11 toggles kiosk-style fullscreen for running standalone on the Pi."""
        self._fullscreen = not self._fullscreen
        self.attributes("-fullscreen", self._fullscreen)

    def _exit_fullscreen(self, _event: tk.Event | None = None) -> None:
        self._fullscreen = False
        self.attributes("-fullscreen", False)

    def _configure_styles(self) -> None:
        style = ttk.Style(self)
        if sys.platform == "win32":
            style.theme_use("vista")

        # Touch-friendly sizing: bigger fonts and bigger hit targets than a
        # mouse-driven desktop layout needs, since fingers are far less
        # precise than a pointer on a small fixed panel.
        base_font = ("TkDefaultFont", 11)
        self.option_add("*Font", base_font)
        style.configure(".", font=base_font)
        style.configure("TButton", padding=(12, 10))
        style.configure("TCheckbutton", padding=(4, 6))
        style.configure("TNotebook.Tab", padding=(16, 9), font=("TkDefaultFont", 10, "bold"))
        style.configure("Treeview", rowheight=30, font=("TkDefaultFont", 10))
        style.configure("Treeview.Heading", font=("TkDefaultFont", 10, "bold"))

        # Compact variants for the top status bar, where space is tight and
        # the controls on it are touched rarely (setup, not moment-to-moment use).
        style.configure("Small.TLabel", font=("TkDefaultFont", 9))
        style.configure("Small.TButton", padding=(6, 4), font=("TkDefaultFont", 9))
        style.configure("Small.TEntry", font=("TkDefaultFont", 9))
        style.configure("Small.TCombobox", font=("TkDefaultFont", 9))

    # -- Top status bar (protocol + connection) ------------------------------

    def _build_top_bar(self, initial_protocol: Path) -> None:
        bar = ttk.Frame(self, padding=(6, 4))
        bar.grid(row=0, column=0, sticky="ew")

        # -- Protocol row --
        proto_row = ttk.Frame(bar)
        proto_row.pack(fill="x", pady=(0, 3))
        proto_row.columnconfigure(0, weight=1)

        self.protocol_path_var = tk.StringVar(value=str(initial_protocol))
        ttk.Entry(proto_row, textvariable=self.protocol_path_var, style="Small.TEntry").grid(
            row=0, column=0, sticky="ew", padx=(0, 4)
        )
        ttk.Button(proto_row, text="Browse", style="Small.TButton", command=self._browse_protocol).grid(
            row=0, column=1, padx=2
        )
        ttk.Button(proto_row, text="Reload", style="Small.TButton", command=self._load_protocol).grid(
            row=0, column=2, padx=(2, 6)
        )
        self.protocol_status_var = tk.StringVar(value="Not loaded")
        ttk.Label(proto_row, textvariable=self.protocol_status_var, style="Small.TLabel").grid(
            row=0, column=3, sticky="w"
        )

        # -- Connection row --
        conn_row = ttk.Frame(bar)
        conn_row.pack(fill="x")

        ttk.Label(conn_row, text="Port", style="Small.TLabel").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            conn_row, textvariable=self.port_var, width=13, state="readonly", style="Small.TCombobox"
        )
        self.port_combo.pack(side="left", padx=(4, 4))
        ttk.Button(conn_row, text="\u21bb", width=3, style="Small.TButton", command=self._refresh_ports).pack(
            side="left", padx=(0, 10)
        )

        ttk.Label(conn_row, text="Baud", style="Small.TLabel").pack(side="left")
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(conn_row, textvariable=self.baud_var, width=7, style="Small.TEntry").pack(
            side="left", padx=(4, 10)
        )

        self.connect_btn = ttk.Button(
            conn_row, text="Connect", style="Small.TButton", command=self._toggle_connect
        )
        self.connect_btn.pack(side="left", padx=(0, 10))

        self.conn_status_var = tk.StringVar(value="Disconnected")
        self.conn_status_lbl = ttk.Label(
            conn_row, textvariable=self.conn_status_var, style="Small.TLabel", foreground="#b00020"
        )
        self.conn_status_lbl.pack(side="left")

    def _browse_protocol(self) -> None:
        path = filedialog.askopenfilename(
            title="Select protocol JSON",
            initialdir=str(REPO_ROOT / "config"),
            filetypes=[("Protocol JSON", "*.json"), ("All files", "*.*")],
        )
        if path:
            self.protocol_path_var.set(path)
            self._load_protocol()

    def _load_protocol(self) -> None:
        path = Path(self.protocol_path_var.get())
        try:
            self.proto = ProtocolDef.load(path)
        except OSError as exc:
            self.protocol_status_var.set("Load failed")
            messagebox.showerror("Protocol load failed", str(exc))
            return
        except (json.JSONDecodeError, KeyError, ValueError) as exc:
            self.protocol_status_var.set("Invalid protocol file")
            messagebox.showerror("Protocol load failed", str(exc))
            return

        n = len(self.proto.params_by_id)
        self.protocol_status_var.set(f"v{self.proto.version} · {n} params · {path.name}")
        self._populate_param_tree()
        self._populate_raw_combos()
        self._populate_graph_checkboxes()

    # -- Connection handling --------------------------------------------------

    def _refresh_ports(self) -> None:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self) -> None:
        if self.ser is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self) -> None:
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Select a serial port first.")
            return
        try:
            baud = int(self.baud_var.get())
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        except (ValueError, serial.SerialException) as exc:
            messagebox.showerror("Connection failed", str(exc))
            self.ser = None
            return

        self.reader = SerialReader(self.ser, self.rx_queue)
        self.reader.start()
        self.connect_btn.config(text="Disconnect")
        self._board_ready = False
        self.conn_status_var.set(f"{port} @ {baud} — waiting for reset…")
        self.conn_status_lbl.config(foreground="#c77700")
        self.after(BOARD_RESET_SETTLE_MS, self._mark_board_ready)

    def _mark_board_ready(self) -> None:
        if self.ser is None:
            return
        self._board_ready = True
        self.conn_status_var.set(f"{self.port_var.get()} @ {self.baud_var.get()}")
        self.conn_status_lbl.config(foreground="#007a00")

        # If parameters were left ticked from a previous connection, resume logging them
        if self._graph_selected_ids and not self._graph_running:
            self._start_graph_polling()

    def _disconnect(self) -> None:
        self._stop_graph_polling()
        if self.reader:
            self.reader.stop()
            self.reader = None
        if self.ser:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None
        self._board_ready = False
        self._recent_frames.clear()
        self.connect_btn.config(text="Connect")
        self.conn_status_var.set("Disconnected")
        self.conn_status_lbl.config(foreground="#b00020")

    # -- Parameters + raw packet --------------------------------------------

    def _build_params_tab(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent, padding=8)
        frame.pack(fill="both", expand=True)

        columns = ("id", "name", "access", "type", "unit", "group")
        self.param_tree = ttk.Treeview(frame, columns=columns, show="headings", height=8)

        # Relative column weights. The tree expands/contracts with its parent.
        column_weights = {"id": 0.7, "name": 2.6, "access": 0.8, "type": 0.9, "unit": 0.9, "group": 1.3}
        for col in columns:
            self.param_tree.heading(col, text=col.upper())
            self.param_tree.column(col, width=60, minwidth=40, anchor="w", stretch=True)

        self.param_tree.pack(fill="both", expand=True)
        self.param_tree.bind("<<TreeviewSelect>>", self._on_param_selected)
        self.param_frame = frame
        self._param_column_weights = column_weights
        self.param_frame.bind("<Configure>", self._resize_param_tree)

        actions = ttk.Frame(frame)
        actions.pack(fill="x", pady=(10, 0))
        ttk.Label(actions, text="Value").pack(side="left")
        self.param_value_var = tk.StringVar(value="0.0")
        ttk.Entry(actions, textvariable=self.param_value_var, width=12).pack(side="left", padx=(6, 12))
        ttk.Button(actions, text="GET", command=self._send_get).pack(side="left", padx=(0, 6))
        ttk.Button(actions, text="SET", command=self._send_set).pack(side="left")

    def _build_raw_tab(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent, padding=12)
        frame.pack(fill="both", expand=True)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(3, weight=1)

        ttk.Label(frame, text="Message").grid(row=0, column=0, sticky="w", pady=6, padx=(0, 8))
        self.raw_msgid_var = tk.StringVar()
        self.raw_msgid_combo = ttk.Combobox(frame, textvariable=self.raw_msgid_var, state="readonly")
        self.raw_msgid_combo.grid(row=0, column=1, sticky="ew", pady=6, padx=(0, 20))

        ttk.Label(frame, text="Direction").grid(row=0, column=2, sticky="w", pady=6, padx=(0, 8))
        self.raw_dir_var = tk.StringVar()
        self.raw_dir_combo = ttk.Combobox(frame, textvariable=self.raw_dir_var, state="readonly")
        self.raw_dir_combo.grid(row=0, column=3, sticky="ew", pady=6)

        ttk.Label(frame, text="Payload (hex)").grid(row=1, column=0, sticky="w", pady=6, padx=(0, 8))
        self.raw_payload_var = tk.StringVar(value="20 00 00 80 3F")
        ttk.Entry(frame, textvariable=self.raw_payload_var).grid(
            row=1, column=1, columnspan=3, sticky="ew", pady=6
        )

        ttk.Label(frame, text="Version").grid(row=2, column=0, sticky="w", pady=6, padx=(0, 8))
        self.raw_version_var = tk.StringVar()
        ttk.Entry(frame, textvariable=self.raw_version_var, width=8).grid(row=2, column=1, sticky="w", pady=6)

        ttk.Label(frame, text="Length").grid(row=2, column=2, sticky="w", pady=6, padx=(0, 8))
        self.raw_length_var = tk.StringVar()
        ttk.Entry(frame, textvariable=self.raw_length_var, width=8).grid(row=2, column=3, sticky="w", pady=6)

        self.corrupt_crc_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(frame, text="Corrupt CRC", variable=self.corrupt_crc_var).grid(
            row=3, column=0, columnspan=4, sticky="w", pady=(10, 6)
        )

        ttk.Button(frame, text="Send Raw Packet", command=self._send_raw).grid(
            row=4, column=0, columnspan=4, sticky="ew", pady=(12, 0), ipady=6
        )

    def _resize_param_tree(self, _event: tk.Event | None = None) -> None:
        """Resize parameter columns proportionally to the available width."""
        if not hasattr(self, "param_tree"):
            return
        available = max(260, self.param_tree.winfo_width() - 4)
        total = sum(self._param_column_weights.values())
        for col, weight in self._param_column_weights.items():
            width = max(42, int(available * weight / total))
            self.param_tree.column(col, width=width)

    def _populate_param_tree(self) -> None:
        self.param_tree.delete(*self.param_tree.get_children())
        for p in sorted(self.proto.params_by_id.values(), key=lambda x: x.id):
            self.param_tree.insert(
                "",
                "end",
                iid=str(p.id),
                values=(f"0x{p.id:02X}", p.name, p.access, p.type, p.unit, p.group),
            )

    def _populate_raw_combos(self) -> None:
        self.raw_msgid_combo["values"] = list(self.proto.msgid_by_name.keys())
        self.raw_dir_combo["values"] = list(self.proto.dir_by_name.keys())
        if self.proto.msgid_by_name:
            self.raw_msgid_var.set(next(iter(self.proto.msgid_by_name)))
        if self.proto.dir_by_name:
            self.raw_dir_var.set(next(iter(self.proto.dir_by_name)))

    def _selected_param(self) -> Param | None:
        sel = self.param_tree.selection()
        if not sel:
            return None
        return self.proto.params_by_id.get(int(sel[0]))

    def _on_param_selected(self, _event: tk.Event) -> None:
        p = self._selected_param()
        if p:
            self.param_value_var.set(str(p.default))

    # -- Edge-case suite ----------------------------------------------------

    def _build_tests_tab(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent, padding=20)
        frame.pack(fill="both", expand=True)

        ttk.Label(
            frame,
            text=(
                "Runs a scripted sequence of protocol edge cases\n"
                "(bad CRC, wrong version, unknown params, etc.)\n"
                "Results are written to the Traffic Log tab."
            ),
            justify="center",
        ).pack(pady=(0, 18))

        ttk.Button(
            frame, text="Run Edge-Case Suite", command=self._run_edge_case_suite
        ).pack(ipadx=24, ipady=12)

        self.suite_status_var = tk.StringVar(value="Not run yet")
        ttk.Label(
            frame, textvariable=self.suite_status_var, font=("TkDefaultFont", 12, "bold")
        ).pack(pady=(18, 0))

    def _pick_param(self, *, writable: bool | None) -> Param | None:
        for p in self.proto.params_by_id.values():
            if writable is None:
                return p
            if writable and p.writable:
                return p
            if writable is False and not p.writable:
                return p
        return None

    def _pick_unknown_id(self) -> int | None:
        for pid in range(256):
            if pid not in self.proto.params_by_id:
                return pid
        return None

    def _build_edge_case_defs(self) -> list[EdgeCase]:
        proto = self.proto
        writable = self._pick_param(writable=True)
        readonly = self._pick_param(writable=False)
        unknown_id = self._pick_unknown_id()
        test_value = 4.25
        bad_version = 0x01 if proto.proto_version != 0x01 else 0x02
        length_lie_wait_ms = proto.timeout_ms + 250
        cases: list[EdgeCase] = []

        if writable:
            wid = writable.id
            cases.extend([
                EdgeCase(
                    "SET writable parameter",
                    lambda: build_set(proto, wid, test_value),
                    lambda info: info.msg_name == "ACK" and info.acked_msg_name == "CMD_SET",
                ),
                EdgeCase(
                    "GET writable parameter back",
                    lambda: build_get(proto, wid),
                    lambda info: (
                        info.msg_name == "CMD_GET"
                        and len(info.payload) >= 5
                        and abs(struct.unpack("<f", info.payload[1:5])[0] - test_value) < 1e-3
                    ),
                ),
                EdgeCase(
                    "Wrong direction byte",
                    lambda: build_packet(
                        proto,
                        proto.msgid_by_name["CMD_GET"],
                        proto.dir_by_name["MCU_TO_PC"],
                        bytes([wid]),
                    ),
                    lambda info: info.msg_name == "NACK" and info.nack_error_name == "BAD_DIRECTION",
                ),
                EdgeCase(
                    "Corrupted CRC",
                    lambda: build_packet(
                        proto,
                        proto.msgid_by_name["CMD_GET"],
                        proto.dir_by_name["PC_TO_MCU"],
                        bytes([wid]),
                        corrupt_crc=True,
                    ),
                    lambda info: info.msg_name == "NACK" and info.nack_error_name == "BAD_CRC",
                ),
                EdgeCase(
                    "Recovery after bad CRC",
                    lambda: build_packet(
                        proto,
                        proto.msgid_by_name["HEARTBEAT"],
                        proto.dir_by_name["PC_TO_MCU"],
                        b"",
                    ),
                    lambda info: info.msg_name == "HEARTBEAT",
                ),
                EdgeCase(
                    "Wrong protocol version",
                    lambda: build_packet(
                        proto,
                        proto.msgid_by_name["CMD_GET"],
                        proto.dir_by_name["PC_TO_MCU"],
                        bytes([wid]),
                        version_override=bad_version,
                    ),
                    lambda info: info.msg_name == "NACK" and info.nack_error_name == "VERSION_MISMATCH",
                ),
                EdgeCase(
                    "Declared length longer than sent",
                    lambda: build_packet(
                        proto,
                        proto.msgid_by_name["CMD_GET"],
                        proto.dir_by_name["PC_TO_MCU"],
                        bytes([wid]),
                        length_override=50,
                    ),
                    None,
                    length_lie_wait_ms,
                ),
                EdgeCase(
                    "Recovery after length-lie timeout",
                    lambda: build_packet(
                        proto,
                        proto.msgid_by_name["HEARTBEAT"],
                        proto.dir_by_name["PC_TO_MCU"],
                        b"",
                    ),
                    lambda info: info.msg_name == "HEARTBEAT",
                ),
            ])

        if readonly:
            cases.append(EdgeCase(
                "SET read-only parameter",
                lambda: build_set(proto, readonly.id, 0.0),
                lambda info: info.msg_name == "NACK" and info.nack_error_name == "READ_ONLY",
            ))

        if unknown_id is not None:
            cases.append(EdgeCase(
                "GET unknown parameter id",
                lambda: build_get(proto, unknown_id),
                lambda info: info.msg_name == "NACK" and info.nack_error_name == "UNKNOWN_PARAM",
            ))

        cases.extend([
            EdgeCase(
                "Unknown message id",
                lambda: build_packet(
                    proto,
                    proto.msgid_by_name["ACK"],
                    proto.dir_by_name["PC_TO_MCU"],
                    b"\x00",
                ),
                lambda info: info.msg_name == "NACK" and info.nack_error_name == "UNKNOWN_MSG",
            ),
            EdgeCase(
                "Plain heartbeat",
                lambda: build_packet(
                    proto,
                    proto.msgid_by_name["HEARTBEAT"],
                    proto.dir_by_name["PC_TO_MCU"],
                    b"",
                ),
                lambda info: info.msg_name == "HEARTBEAT",
            ),
        ])
        return cases

    def _run_edge_case_suite(self) -> None:
        if not self._require_ready():
            return

        self._edge_cases = self._build_edge_case_defs()
        self._edge_case_results = []
        self.suite_status_var.set(f"Running 0/{len(self._edge_cases)}…")
        self._run_next_edge_case(0)

    def _run_next_edge_case(self, index: int) -> None:
        if index >= len(self._edge_cases):
            passed = sum(self._edge_case_results)
            total = len(self._edge_case_results)
            self.suite_status_var.set(f"Done: {passed}/{total} passed")
            self._log("suite", f"Suite complete — {passed}/{total} passed", "pass" if passed == total else "fail")
            return

        case = self._edge_cases[index]
        send_time = time.monotonic()
        self._transmit(case.build())
        self.suite_status_var.set(f"Running {index + 1}/{len(self._edge_cases)}…")
        self.after(case.wait_ms, lambda: self._check_edge_case(index, send_time))

    def _check_edge_case(self, index: int, send_time: float) -> None:
        case = self._edge_cases[index]
        relevant = [info for ts, info in self._recent_frames if ts >= send_time]

        if case.match is None:
            ok = len(relevant) == 0
        else:
            ok = any(case.match(info) for info in relevant)

        self._edge_case_results.append(ok)
        tag = "pass" if ok else "fail"
        self._log("suite", f"[{index + 1}/{len(self._edge_cases)}] {case.name} — {'PASS' if ok else 'FAIL'}", tag)
        self._run_next_edge_case(index + 1)

    # -- Main notebook (fills all remaining screen space below the top bar) --

    def _build_notebook(self) -> None:
        notebook = ttk.Notebook(self)
        notebook.grid(row=1, column=0, sticky="nsew", padx=6, pady=(2, 6))
        self.notebook = notebook

        params_tab = ttk.Frame(notebook)
        raw_tab = ttk.Frame(notebook)
        graph_tab = ttk.Frame(notebook)
        log_tab = ttk.Frame(notebook)
        tests_tab = ttk.Frame(notebook)

        notebook.add(params_tab, text="Parameters")
        notebook.add(raw_tab, text="Raw Packet")
        notebook.add(graph_tab, text="Live Graph")
        notebook.add(log_tab, text="Traffic Log")
        notebook.add(tests_tab, text="Tests")

        self._build_params_tab(params_tab)
        self._build_raw_tab(raw_tab)
        self._build_graph_panel(graph_tab)
        self._build_log_panel(log_tab)
        self._build_tests_tab(tests_tab)
        self.after_idle(lambda: self._resize_param_tree())

    # -- Live multi-parameter graph ------------------------------------------

    def _build_graph_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent, padding=(8, 6))
        frame.pack(fill="both", expand=True)

        controls = ttk.Frame(frame)
        controls.pack(side="left", fill="y", padx=(0, 6))
        controls.configure(width=190)

        ttk.Label(controls, text="Tick a parameter to log & graph it").pack(anchor="w")

        list_container = ttk.Frame(controls, relief="sunken", borderwidth=1)
        list_container.pack(fill="both", expand=True)

        list_scrollbar = ttk.Scrollbar(list_container, orient="vertical")
        self._graph_checkbox_canvas = tk.Canvas(
            list_container, width=190, highlightthickness=0, background="#1e1e1e",
            yscrollcommand=list_scrollbar.set,
        )
        list_scrollbar.config(command=self._graph_checkbox_canvas.yview)
        list_scrollbar.pack(side="right", fill="y")
        self._graph_checkbox_canvas.pack(side="left", fill="both", expand=True)

        self._graph_checkbox_frame = ttk.Frame(self._graph_checkbox_canvas)
        self._graph_checkbox_canvas.create_window((0, 0), window=self._graph_checkbox_frame, anchor="nw")
        self._graph_checkbox_frame.bind(
            "<Configure>",
            lambda _e: self._graph_checkbox_canvas.configure(
                scrollregion=self._graph_checkbox_canvas.bbox("all")
            ),
        )

        interval_row = ttk.Frame(controls)
        interval_row.pack(fill="x", pady=(6, 2))
        ttk.Label(interval_row, text="Poll (ms)").pack(side="left")
        self.graph_interval_var = tk.StringVar(value="100")
        ttk.Entry(interval_row, textvariable=self.graph_interval_var, width=6).pack(side="left", padx=(4, 0))

        window_row = ttk.Frame(controls)
        window_row.pack(fill="x", pady=(2, 6))
        ttk.Label(window_row, text="Window (s)").pack(side="left")
        self.graph_window_var = tk.StringVar(value="30")
        ttk.Entry(window_row, textvariable=self.graph_window_var, width=6).pack(side="left", padx=(4, 0))

        ttk.Button(controls, text="Clear data", command=self._clear_graph).pack(fill="x")

        ttk.Label(controls, text="Latest values").pack(anchor="w", pady=(8, 0))
        self.graph_legend = tk.Text(
            controls, height=6, width=22, state="disabled", font=("TkFixedFont", 9),
            background="#1e1e1e", foreground="#e0e0e0", relief="flat"
        )
        self.graph_legend.pack(fill="both", expand=True, pady=(2, 0))

        self.graph_canvas = tk.Canvas(frame, background="#101010", highlightthickness=0)
        self.graph_canvas.pack(side="left", fill="both", expand=True)
        self.graph_canvas.bind("<Configure>", lambda _e: self._redraw_graph())
        frame.bind("<Configure>", self._resize_graph_controls)

    def _resize_graph_controls(self, event: tk.Event) -> None:
        """Keep graph controls usable without stealing excessive plot width."""
        if not hasattr(self, "_graph_checkbox_canvas"):
            return
        width = max(150, min(240, int(event.width * 0.28)))
        self._graph_controls_width = width
        self._graph_checkbox_canvas.configure(width=width)
        if hasattr(self, "graph_legend"):
            self.graph_legend.configure(width=max(16, int(width / 8)))

    def _populate_graph_checkboxes(self) -> None:
        # New protocol/param set invalidates whatever was ticked before
        self._stop_graph_polling()
        self._graph_selected_ids = []
        self._graph_data = {}
        self._graph_colors = {}
        self._graph_swatches = {}

        for child in self._graph_checkbox_frame.winfo_children():
            child.destroy()
        self._graph_checkbox_vars = {}

        for p in sorted(self.proto.params_by_id.values(), key=lambda x: x.id):
            if not p.readable:
                continue

            var = tk.BooleanVar(value=False)
            self._graph_checkbox_vars[p.id] = var

            row = ttk.Frame(self._graph_checkbox_frame)
            row.pack(fill="x", anchor="w")

            swatch = tk.Canvas(row, width=12, height=12, highlightthickness=0, background="#1e1e1e")
            swatch.pack(side="left", padx=(3, 5), pady=6)
            self._graph_swatches[p.id] = swatch

            ttk.Checkbutton(
                row, text=f"0x{p.id:02X}  {p.name}", variable=var,
                command=lambda pid=p.id: self._on_param_checkbox_toggle(pid),
            ).pack(side="left", anchor="w", fill="x", expand=True)

        self._update_legend()

    def _on_param_checkbox_toggle(self, pid: int) -> None:
        var = self._graph_checkbox_vars.get(pid)
        if var is None:
            return

        if var.get():
            if not self._require_ready():
                var.set(False)
                return

            if pid not in self._graph_selected_ids:
                self._graph_selected_ids.append(pid)
            self._graph_data.setdefault(pid, deque())
            if pid not in self._graph_colors:
                self._graph_colors[pid] = GRAPH_COLORS[len(self._graph_colors) % len(GRAPH_COLORS)]

            swatch = self._graph_swatches.get(pid)
            if swatch is not None:
                swatch.configure(background=self._graph_colors[pid])

            if not self._graph_running:
                self._start_graph_polling()
        else:
            if pid in self._graph_selected_ids:
                self._graph_selected_ids.remove(pid)

            swatch = self._graph_swatches.get(pid)
            if swatch is not None:
                swatch.configure(background="#1e1e1e")

            if not self._graph_selected_ids:
                self._stop_graph_polling()

        self._update_legend()
        self._redraw_graph()

    def _start_graph_polling(self) -> None:
        try:
            interval_ms = max(20, int(self.graph_interval_var.get()))
        except ValueError:
            interval_ms = 100
            self.graph_interval_var.set("100")
        self._graph_interval_ms = interval_ms

        self._graph_running = True
        if self._graph_start_time is None:
            self._graph_start_time = time.monotonic()
        self._graph_poll_index = 0
        self._graph_poll_tick()

    def _stop_graph_polling(self) -> None:
        self._graph_running = False
        if self._graph_after_id is not None:
            try:
                self.after_cancel(self._graph_after_id)
            except ValueError:
                pass
            self._graph_after_id = None

    def _graph_poll_tick(self) -> None:
        if not self._graph_running:
            return
        if self.ser is None or not self._board_ready:
            self._stop_graph_polling()
            return

        if self._graph_selected_ids:
            # Round-robin one GET per tick so the MCU never has more than one
            # outstanding request — sending all params at once could exceed
            # the firmware's per-byte timeout budget on a busy link.
            pid = self._graph_selected_ids[self._graph_poll_index % len(self._graph_selected_ids)]
            self._graph_poll_index += 1
            try:
                self._transmit(build_get(self.proto, pid))
            except (serial.SerialException, OSError):
                self._stop_graph_polling()
                return

        # Re-read the interval each tick so a mid-run edit takes effect immediately
        try:
            interval_ms = max(20, int(self.graph_interval_var.get()))
        except ValueError:
            interval_ms = self._graph_interval_ms
        self._graph_interval_ms = interval_ms

        self._graph_after_id = self.after(self._graph_interval_ms, self._graph_poll_tick)

    def _clear_graph(self) -> None:
        self._graph_data = {pid: deque() for pid in self._graph_selected_ids}
        self._graph_start_time = time.monotonic() if self._graph_running else None
        self._redraw_graph()
        self._update_legend()

    def _on_frame_received(self, info: FrameInfo) -> None:
        if not self._graph_running:
            return
        if info.msg_name != "CMD_GET" or len(info.payload) < 5:
            return

        pid = info.payload[0]
        if pid not in self._graph_selected_ids:
            return

        try:
            value = struct.unpack("<f", info.payload[1:5])[0]
        except struct.error:
            return

        now = time.monotonic()
        if self._graph_start_time is None:
            self._graph_start_time = now
        t = now - self._graph_start_time

        data = self._graph_data.setdefault(pid, deque())
        data.append((t, value))

        try:
            window_s = max(1.0, float(self.graph_window_var.get()))
        except ValueError:
            window_s = 30.0
        cutoff = t - window_s
        while data and data[0][0] < cutoff:
            data.popleft()

        self._redraw_graph()
        self._update_legend()

    def _format_axis_value(self, value: float, span: float) -> str:
        """Pick a sensible number of decimals based on how large the visible range is."""
        if span < 1:
            return f"{value:.3f}"
        if span < 10:
            return f"{value:.2f}"
        if span < 100:
            return f"{value:.1f}"
        return f"{value:.0f}"

    def _redraw_graph(self) -> None:
        canvas = self.graph_canvas
        canvas.delete("all")
        width = canvas.winfo_width()
        height = canvas.winfo_height()
        if width < 20 or height < 20:
            return

        margin_l, margin_r, margin_t, margin_b = 52, 10, 10, 18
        plot_w = max(1, width - margin_l - margin_r)
        plot_h = max(1, height - margin_t - margin_b)

        canvas.create_rectangle(
            margin_l, margin_t, margin_l + plot_w, margin_t + plot_h, outline="#333"
        )

        try:
            window_s = max(1.0, float(self.graph_window_var.get()))
        except ValueError:
            window_s = 30.0

        if not self._graph_selected_ids:
            canvas.create_text(
                margin_l + plot_w / 2, margin_t + plot_h / 2,
                text="Tick a parameter on the left to start logging",
                fill="#666", font=("TkFixedFont", 10),
            )
            return

        now_t = 0.0
        have_data = False
        for pid in self._graph_selected_ids:
            data = self._graph_data.get(pid)
            if data:
                have_data = True
                now_t = max(now_t, data[-1][0])

        if not have_data:
            canvas.create_text(
                margin_l + plot_w / 2, margin_t + plot_h / 2,
                text="Waiting for data…", fill="#666", font=("TkFixedFont", 10),
            )
            return

        t_min = now_t - window_s

        # Shared axis range across every currently-plotted parameter — real values, not normalised
        global_min = float("inf")
        global_max = float("-inf")

        for pid in self._graph_selected_ids:
            data = self._graph_data.get(pid)
            if not data:
                continue
            for t, v in data:
                if t >= t_min:
                    global_min = min(global_min, v)
                    global_max = max(global_max, v)

        if global_min == float("inf"):
            return

        span = global_max - global_min
        if span < 1e-6:
            # Flat signal — give it a small fixed window around the value rather than
            # collapsing to a single line or blowing up noise to fill the plot.
            pad = 0.5 if abs(global_max) < 1.0 else abs(global_max) * 0.05
            global_min -= pad
            global_max += pad
        else:
            # Small headroom so traces don't hug the top/bottom edge exactly
            pad = span * 0.08
            global_min -= pad
            global_max += pad

        axis_span = global_max - global_min

        # Y-axis gridlines + real value labels (5 evenly spaced ticks)
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = margin_t + plot_h - frac * plot_h
            value = global_min + frac * axis_span
            canvas.create_line(margin_l, y, margin_l + plot_w, y, fill="#222")
            canvas.create_text(
                margin_l - 6, y,
                text=self._format_axis_value(value, axis_span),
                fill="#777", anchor="e", font=("TkFixedFont", 8),
            )

        for pid in self._graph_selected_ids:
            data = self._graph_data.get(pid)
            if not data or len(data) < 2:
                continue

            color = self._graph_colors.get(pid, "#00AAFF")
            flat: list[float] = []
            for t, v in data:
                if t < t_min:
                    continue
                x = margin_l + ((t - t_min) / window_s) * plot_w
                norm = (v - global_min) / axis_span
                norm = min(1.0, max(0.0, norm))
                y = margin_t + plot_h - norm * plot_h
                flat.extend((x, y))
            if len(flat) >= 4:
                canvas.create_line(*flat, fill=color, width=2)

        canvas.create_text(
            margin_l + plot_w, margin_t + plot_h + 10,
            text=f"last {window_s:.0f}s (auto-scaled, shared axis across ticked params)",
            fill="#666", anchor="e", font=("TkFixedFont", 8),
        )

    def _update_legend(self) -> None:
        if not hasattr(self, "graph_legend"):
            return
        self.graph_legend.config(state="normal")
        self.graph_legend.delete("1.0", "end")
        for pid in self._graph_selected_ids:
            param = self.proto.params_by_id.get(pid) if self.proto else None
            name = param.name if param else f"0x{pid:02X}"
            unit = f" {param.unit}" if param and param.unit else ""
            data = self._graph_data.get(pid)
            latest = f"{data[-1][1]:.3f}{unit}" if data else "—"
            color = self._graph_colors.get(pid, "#00AAFF")
            tag = f"legend_{pid}"
            self.graph_legend.tag_configure(tag, foreground=color)
            self.graph_legend.insert("end", f"\u25CF {name}: {latest}\n", tag)
        self.graph_legend.config(state="disabled")

    # -- Log panel ----------------------------------------------------------

    def _build_log_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent, padding=(8, 6))
        frame.pack(fill="both", expand=True)

        toolbar = ttk.Frame(frame)
        toolbar.pack(fill="x", pady=(0, 4))
        ttk.Button(toolbar, text="Clear", command=self._clear_log).pack(side="right")

        self.log_text = scrolledtext.ScrolledText(
            frame, height=8, font=LOG_FONT, state="disabled", wrap="none"
        )
        self.log_text.pack(fill="both", expand=True)
        for tag, color in (
            ("tx", "#005a9e"),
            ("rx", "#007a00"),
            ("nack", "#b00020"),
            ("pass", "#007a00"),
            ("fail", "#b00020"),
            ("error", "#b00020"),
        ):
            self.log_text.tag_configure(tag, foreground=color)

    def _clear_log(self) -> None:
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

    def _log(self, direction: str, text: str, tag: str | None = None) -> None:
        ts = datetime.now().strftime("%H:%M:%S")
        line = f"{ts}  {direction.upper():5}  {text}\n"
        self.log_text.config(state="normal")
        self.log_text.insert("end", line, tag or direction)
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    # -- Sending ------------------------------------------------------------

    def _require_ready(self) -> bool:
        if self.proto is None:
            messagebox.showwarning("No protocol", "Load a protocol JSON first.")
            return False
        if self.ser is None:
            messagebox.showwarning("Not connected", "Connect to a serial port first.")
            return False
        if not self._board_ready:
            messagebox.showinfo(
                "Board resetting",
                f"Wait {BOARD_RESET_SETTLE_MS} ms after connect before sending.",
            )
            return False
        return True

    def _transmit(self, frame: bytes) -> None:
        self.ser.write(frame)
        self._log("tx", f"{hexdump(frame):<36}  {decode_frame(self.proto, frame)}")

    def _send_get(self) -> None:
        if not self._require_ready():
            return
        p = self._selected_param()
        if not p:
            messagebox.showwarning("No parameter", "Select a parameter first.")
            return
        self._transmit(build_get(self.proto, p.id))

    def _send_set(self) -> None:
        if not self._require_ready():
            return
        p = self._selected_param()
        if not p:
            messagebox.showwarning("No parameter", "Select a parameter first.")
            return
        try:
            value = float(self.param_value_var.get())
        except ValueError:
            messagebox.showwarning("Bad value", "Value must be a number.")
            return
        self._transmit(build_set(self.proto, p.id, value))

    def _send_raw(self) -> None:
        if not self._require_ready():
            return
        try:
            msg_id = self.proto.msgid_by_name[self.raw_msgid_var.get()]
            direction = self.proto.dir_by_name[self.raw_dir_var.get()]
            payload = parse_hex_bytes(self.raw_payload_var.get())

            version_str = self.raw_version_var.get().strip()
            length_str = self.raw_length_var.get().strip()
            version_override = int(version_str, 0) if version_str else None
            length_override = int(length_str, 0) if length_str else None
        except (KeyError, ValueError) as exc:
            messagebox.showwarning("Invalid packet", str(exc))
            return

        frame = build_packet(
            self.proto,
            msg_id,
            direction,
            payload,
            corrupt_crc=self.corrupt_crc_var.get(),
            version_override=version_override,
            length_override=length_override,
        )
        self._transmit(frame)

    # -- RX handling --------------------------------------------------------

    def _poll_rx_queue(self) -> None:
        try:
            while True:
                kind, payload = self.rx_queue.get_nowait()
                if kind == "raw":
                    self._rx_buf.extend(payload)
                    if self.proto:
                        for frame in extract_frames(self._rx_buf, self.proto):
                            info = parse_frame(self.proto, frame)
                            if info is not None:
                                self._recent_frames.append((time.monotonic(), info))
                                self._on_frame_received(info)
                            tag = "nack" if info and info.msg_name == "NACK" else "rx"
                            self._log("rx", f"{hexdump(frame):<36}  {decode_frame(self.proto, frame)}", tag)
                elif kind == "error":
                    self._log("error", payload)
                    self._disconnect()
        except queue.Empty:
            pass

        cutoff = time.monotonic() - 10.0
        self._recent_frames = [(ts, info) for ts, info in self._recent_frames if ts > cutoff]
        self.after(50, self._poll_rx_queue)

    def _on_close(self) -> None:
        self._stop_graph_polling()
        self._disconnect()
        self.destroy()


def main() -> None:
    parser = argparse.ArgumentParser(description="PD Comms serial protocol test tool")
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    args = parser.parse_args()
    SerialTestTool(args.protocol).mainloop()


if __name__ == "__main__":
    main()