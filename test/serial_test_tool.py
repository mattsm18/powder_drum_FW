#!/usr/bin/env python3
#
# Title: test/serial_test_tool.py
# Purpose:
# - Standalone serial test tool for the PC <-> MCU comms protocol.
# - Deliberately has ZERO dependency on config/ or src/managers/serial/ —
#   it re-implements the (small) framing/CRC logic itself so it keeps
#   working unmodified regardless of what happens to the SW library.
# - Loads any pd_comms_protocol_*.json version at runtime (path is
#   editable in the GUI), so it tracks protocol changes without being
#   re-run through the firmware codegen.
# - Lets you:
#     * GET / SET any parameter defined in the loaded protocol file
#     * send a hand-crafted raw packet (arbitrary msg_id / direction /
#       payload bytes, optional bad version, lying declared length,
#       corrupted CRC) to test edge cases like SET-on-read-only,
#       unknown parameter ids, wrong direction, bad CRC, etc.
#     * see every TX/RX frame, decoded, in a scrolling log — including a
#       raw hexdump of anything that arrives even if it never resolves
#       into a complete parsed frame, so "board said nothing" and "board
#       said something the parser choked on" don't look identical
#     * run an automated edge-case suite that fires each of the above at
#       the board in sequence and reports PASS/FAIL against the expected
#       protocol behaviour
#
# Requires: pyserial   (pip install pyserial --break-system-packages)
#
# Usage:
#   python test/serial_test_tool.py
#   python test/serial_test_tool.py --protocol path/to/pd_comms_protocol_v1.1.json

import argparse
import json
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk
from typing import Callable, Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial is required: pip install pyserial --break-system-packages")
    sys.exit(1)


REPO_ROOT        = Path(__file__).resolve().parent.parent
DEFAULT_PROTOCOL = REPO_ROOT / "config" / "pd_comms_protocol_v1.1.json"

# How long to hold off sending after opening the port. Nano Every (and most
# Arduino-family boards) resets on DTR toggle when the serial port is opened,
# so bytes sent immediately after Connect can land while the board is still
# in setup() / mid-reset and simply get dropped — which looks identical to
# "the board never replies" from this tool's point of view.
BOARD_RESET_SETTLE_MS = 2000

# Default window to wait for a reply before declaring an edge-case test
# failed/timed-out. Kept generous relative to typical USB-serial latency.
DEFAULT_TEST_WAIT_MS = 400


# ══════════════════════════════════════════════════════════════
# Protocol definition — parsed fresh from whatever JSON is loaded.
# ══════════════════════════════════════════════════════════════

@dataclass
class Param:
    id: int
    name: str
    label: str
    access: str   # e.g. "rw", "r"
    type: str
    default: float
    min: float
    max: float
    unit: str
    group: str

    @property
    def readable(self) -> bool: return "r" in self.access

    @property
    def writable(self) -> bool: return "w" in self.access

    def display(self) -> str:
        unit = f" {self.unit}" if self.unit else ""
        return f"0x{self.id:02X}  {self.name}  [{self.access}]  {self.type}{unit}"


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
    dir_by_name: dict
    dir_by_value: dict
    msgid_by_name: dict
    msgid_by_value: dict
    nack_by_name: dict
    nack_by_value: dict
    params_by_id: dict
    params_by_name: dict

    @staticmethod
    def load(path: Path) -> "ProtocolDef":
        with open(path, "r") as f:
            data = json.load(f)

        frame     = data["frame"]
        direction = data["direction"]
        msg_id    = data["msg_id"]
        nack      = data["nack_error"]

        dir_by_name   = {k: int(v, 16) for k, v in direction.items()}
        msgid_by_name = {k: int(v, 16) for k, v in msg_id.items()}
        nack_by_name  = {k: int(v, 16) for k, v in nack.items()}

        params_by_id, params_by_name = {}, {}
        for p in data["parameters"]:
            param = Param(
                id=int(p["id"], 16), name=p["name"], label=p["label"],
                access="".join(p["access"]), type=p["type"], default=p["default"],
                min=p["min"], max=p["max"], unit=p["unit"], group=p["group"],
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
            dir_by_name=dir_by_name, dir_by_value={v: k for k, v in dir_by_name.items()},
            msgid_by_name=msgid_by_name, msgid_by_value={v: k for k, v in msgid_by_name.items()},
            nack_by_name=nack_by_name, nack_by_value={v: k for k, v in nack_by_name.items()},
            params_by_id=params_by_id, params_by_name=params_by_name,
        )


# ══════════════════════════════════════════════════════════════
# Framing / CRC — deliberately re-implemented here, not imported,
# so this tool has no dependency on src/managers/serial/protocol.py.
# ══════════════════════════════════════════════════════════════

def compute_crc(data: bytes) -> int:
    crc = 0x00
    for b in data: crc ^= b
    return crc


def build_packet(
    proto: ProtocolDef, msg_id: int, direction: int, payload: bytes,
    corrupt_crc: bool = False,
    version_override: Optional[int] = None,
    length_override: Optional[int] = None,
) -> bytes:
    """Build a frame. version_override/length_override let you deliberately
    lie in the header — that's the only way to exercise the firmware's
    version-mismatch and declared-length-vs-actual-bytes edge cases, since
    a well-formed packet by definition can't trigger them."""
    version = proto.proto_version if version_override is None else version_override
    length  = len(payload) if length_override is None else length_override
    header = bytes([proto.sof, version, msg_id, direction, length])
    crc = compute_crc(header + payload)
    if corrupt_crc: crc = (crc + 1) & 0xFF
    return header + payload + bytes([crc])


def build_get(proto: ProtocolDef, param_id: int) -> bytes:
    return build_packet(proto, proto.msgid_by_name["CMD_GET"], proto.dir_by_name["PC_TO_MCU"], bytes([param_id]))


def build_set(proto: ProtocolDef, param_id: int, value: float) -> bytes:
    payload = bytes([param_id]) + struct.pack("<f", value)
    return build_packet(proto, proto.msgid_by_name["CMD_SET"], proto.dir_by_name["PC_TO_MCU"], payload)


def extract_frames(buf: bytearray, proto: ProtocolDef) -> list:
    """Pull any complete frames out of a rolling RX buffer (SOF-hunting)."""
    frames = []
    while True:
        idx = buf.find(bytes([proto.sof]))
        if idx == -1:
            buf.clear()
            break
        if idx > 0:
            del buf[:idx]

        if len(buf) < proto.header_size:
            break  # wait for the rest of the header (incl. length byte)

        length = buf[proto.header_size - 1]
        total = proto.header_size + length + 1  # + CRC byte

        if len(buf) < total:
            break  # wait for the rest of the frame

        frames.append(bytes(buf[:total]))
        del buf[:total]
    return frames


# ══════════════════════════════════════════════════════════════
# Frame parsing — structured, so both the human-readable log line
# AND the automated test-suite assertions come from the same source
# of truth instead of the log text being re-parsed for assertions.
# ══════════════════════════════════════════════════════════════

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
    crc_received: Optional[int]
    crc_computed: int
    crc_ok: bool
    version_ok: bool
    detail: str = ""
    nack_error_name: Optional[str] = None
    acked_msg_name: Optional[str] = None


def _pname(proto: ProtocolDef, pid: int) -> str:
    p = proto.params_by_id.get(pid)
    return p.name if p else f"0x{pid:02X}(unknown)"


def _decode_payload(proto: ProtocolDef, msg_name: str, payload: bytes) -> str:
    try:
        if msg_name == "CMD_GET" and len(payload) >= 5:
            pid, val = payload[0], struct.unpack("<f", payload[1:5])[0]
            return f"param={_pname(proto, pid)} value={val:.4f}"
        if msg_name == "CMD_GET" and len(payload) == 1:
            return f"GET request param={_pname(proto, payload[0])}"
        if msg_name == "CMD_SET" and len(payload) >= 5:
            pid, val = payload[0], struct.unpack("<f", payload[1:5])[0]
            return f"param={_pname(proto, pid)} value={val:.4f}"
        if msg_name == "STATUS" and len(payload) >= 1:
            return f"status_code=0x{payload[0]:02X}"
        if msg_name == "ACK" and len(payload) >= 1:
            return f"acked={proto.msgid_by_value.get(payload[0], f'0x{payload[0]:02X}')}"
        if msg_name == "NACK" and len(payload) >= 2:
            nacked = proto.msgid_by_value.get(payload[0], f"0x{payload[0]:02X}")
            err = proto.nack_by_value.get(payload[1], f"0x{payload[1]:02X}")
            return f"nacked={nacked} error={err}"
        if msg_name == "HEARTBEAT":
            return ""
    except (struct.error, IndexError):
        return "(payload decode error)"

    return f"payload=[{' '.join(f'{b:02X}' for b in payload)}]" if payload else ""


def parse_frame(proto: ProtocolDef, frame: bytes) -> Optional[FrameInfo]:
    if len(frame) < proto.header_size + 1:
        return None

    version, msg_id, dir_, length = frame[1], frame[2], frame[3], frame[4]
    payload = frame[5:5 + length]
    crc_received = frame[5 + length] if len(frame) > 5 + length else None
    crc_computed = compute_crc(frame[:5 + length])

    msg_name = proto.msgid_by_value.get(msg_id, f"UNKNOWN(0x{msg_id:02X})")
    dir_name = proto.dir_by_value.get(dir_, f"0x{dir_:02X}")

    info = FrameInfo(
        raw=frame, version=version, msg_id=msg_id, msg_name=msg_name,
        direction=dir_, dir_name=dir_name, length=length, payload=payload,
        crc_received=crc_received, crc_computed=crc_computed,
        crc_ok=(crc_received == crc_computed), version_ok=(version == proto.proto_version),
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

    bits = [f"ver=0x{info.version:02X}", f"msg={info.msg_name}", f"dir={info.dir_name}", f"len={info.length}"]

    if not info.version_ok:
        bits.append("VERSION MISMATCH")
    if not info.crc_ok:
        recv = f"0x{info.crc_received:02X}" if info.crc_received is not None else "?"
        bits.append(f"CRC MISMATCH (recv={recv} calc=0x{info.crc_computed:02X})")
    if info.detail:
        bits.append(info.detail)

    return "  |  ".join(bits)


def hexdump(frame: bytes) -> str:
    return " ".join(f"{b:02X}" for b in frame)


# ══════════════════════════════════════════════════════════════
# Automated edge-case suite — fires known-bad/known-good packets at
# the board in sequence and checks the reply (or deliberate absence
# of one) against what the protocol says should happen.
# ══════════════════════════════════════════════════════════════

@dataclass
class EdgeCase:
    name: str
    note: str
    build: Callable[[], bytes]
    # None means "expect no reply at all within wait_ms" (used for the
    # declared-length-lie test, where the correct behaviour is silence
    # until the timeout guard resets the state machine).
    match: Optional[Callable[[FrameInfo], bool]]
    wait_ms: int = DEFAULT_TEST_WAIT_MS


# ══════════════════════════════════════════════════════════════
# Serial reader thread — just moves bytes off the wire into a queue.
# All decoding/GUI updates happen back on the main thread.
# ══════════════════════════════════════════════════════════════

class SerialReader(threading.Thread):
    def __init__(self, ser: serial.Serial, rx_queue: "queue.Queue"):
        super().__init__(daemon=True)
        self._ser = ser
        self._rx_queue = rx_queue
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def run(self):
        while not self._stop.is_set():
            try:
                data = self._ser.read(self._ser.in_waiting or 1)
            except (serial.SerialException, OSError) as e:
                self._rx_queue.put(("error", str(e)))
                return

            if not data:
                continue

            self._rx_queue.put(("raw", bytes(data)))


# ══════════════════════════════════════════════════════════════
# GUI
# ══════════════════════════════════════════════════════════════

class SerialTestTool(tk.Tk):
    def __init__(self, initial_protocol: Path):
        super().__init__()
        self.title("PD Comms Serial Test Tool")
        self.geometry("1040x760")

        self.proto: Optional[ProtocolDef] = None
        self.ser: Optional[serial.Serial] = None
        self.reader: Optional[SerialReader] = None
        self.rx_queue: "queue.Queue" = queue.Queue()
        self._rx_buf = bytearray()

        # Every parsed frame, timestamped, so the edge-case runner can ask
        # "did anything matching X arrive after I sent at time T" without
        # re-parsing the log text.
        self._recent_frames: list = []

        self._board_ready = False   # gated behind BOARD_RESET_SETTLE_MS after connect
        self._edge_cases: list = []
        self._edge_case_results: list = []

        self._build_protocol_frame(initial_protocol)
        self._build_connection_frame()
        self._build_main_panes()
        self._build_edge_case_frame()
        self._build_log_frame()

        self._load_protocol()
        self._refresh_ports()
        self.after(50, self._poll_rx_queue)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ── Protocol file panel ─────────────────────────────────────

    def _build_protocol_frame(self, initial_protocol: Path):
        frame = ttk.LabelFrame(self, text="Protocol definition")
        frame.pack(fill="x", padx=8, pady=(8, 4))

        self.protocol_path_var = tk.StringVar(value=str(initial_protocol))
        ttk.Entry(frame, textvariable=self.protocol_path_var, width=70).pack(side="left", padx=6, pady=6)
        ttk.Button(frame, text="Browse...", command=self._browse_protocol).pack(side="left", padx=4)
        ttk.Button(frame, text="Load / Reload", command=self._load_protocol).pack(side="left", padx=4)

        self.protocol_status_var = tk.StringVar(value="not loaded")
        ttk.Label(frame, textvariable=self.protocol_status_var).pack(side="left", padx=12)

    def _browse_protocol(self):
        path = filedialog.askopenfilename(
            title="Select protocol JSON",
            initialdir=str(REPO_ROOT / "config"),
            filetypes=[("Protocol JSON", "*.json"), ("All files", "*.*")],
        )
        if path:
            self.protocol_path_var.set(path)
            self._load_protocol()

    def _load_protocol(self):
        path = Path(self.protocol_path_var.get())
        try:
            self.proto = ProtocolDef.load(path)
        except Exception as e:
            self.protocol_status_var.set(f"FAILED TO LOAD: {e}")
            messagebox.showerror("Protocol load failed", str(e))
            return

        self.protocol_status_var.set(
            f"v{self.proto.version} — {len(self.proto.params_by_id)} parameters — "
            f"timeout={self.proto.timeout_ms}ms retries={self.proto.max_retries} — loaded from {path.name}"
        )
        self._populate_param_tree()
        self._populate_raw_combos()
        self._log("info", f"Loaded protocol {path} (v{self.proto.version})")

    # ── Connection panel ────────────────────────────────────────

    def _build_connection_frame(self):
        frame = ttk.LabelFrame(self, text="Connection")
        frame.pack(fill="x", padx=8, pady=4)

        ttk.Label(frame, text="Port:").pack(side="left", padx=(6, 2))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var, width=20, state="readonly")
        self.port_combo.pack(side="left", padx=2)
        ttk.Button(frame, text="Refresh", command=self._refresh_ports).pack(side="left", padx=4)

        ttk.Label(frame, text="Baud:").pack(side="left", padx=(12, 2))
        self.baud_var = tk.StringVar(value="115200")
        ttk.Entry(frame, textvariable=self.baud_var, width=10).pack(side="left", padx=2)

        self.connect_btn = ttk.Button(frame, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side="left", padx=12)

        self.conn_status_var = tk.StringVar(value="disconnected")
        self.conn_status_lbl = ttk.Label(frame, textvariable=self.conn_status_var, foreground="red")
        self.conn_status_lbl.pack(side="left", padx=6)

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        if self.ser is None:
            self._connect()
        else:
            self._disconnect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("No port", "Select a serial port first")
            return
        try:
            baud = int(self.baud_var.get())
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        except (ValueError, serial.SerialException) as e:
            messagebox.showerror("Connection failed", str(e))
            self.ser = None
            return

        self.reader = SerialReader(self.ser, self.rx_queue)
        self.reader.start()
        self.connect_btn.config(text="Disconnect")

        # Most Arduino-family boards (Nano Every included) reset when the
        # serial port is opened (DTR toggle). Sending immediately risks the
        # board still being mid-reset/in setup() and silently dropping
        # bytes — indistinguishable from "board never replies" otherwise.
        self._board_ready = False
        self.conn_status_var.set(f"connected: {port} @ {baud}  (waiting for board reset...)")
        self.conn_status_lbl.config(foreground="orange")
        self._log("info", f"Connected to {port} @ {baud} — board likely reset on DTR, "
                           f"holding sends for {BOARD_RESET_SETTLE_MS}ms")
        self.after(BOARD_RESET_SETTLE_MS, self._mark_board_ready)

    def _mark_board_ready(self):
        if self.ser is None:
            return  # disconnected during the settle window
        self._board_ready = True
        self.conn_status_var.set(f"connected: {self.port_var.get()} @ {self.baud_var.get()}")
        self.conn_status_lbl.config(foreground="green")
        self._log("info", "Board should be up now — ready to send")

    def _disconnect(self):
        if self.reader:
            self.reader.stop()
            self.reader = None
        if self.ser:
            try: self.ser.close()
            except (serial.SerialException, OSError): pass
            self.ser = None
        self._board_ready = False
        self._recent_frames.clear()
        self.connect_btn.config(text="Connect")
        self.conn_status_var.set("disconnected")
        self.conn_status_lbl.config(foreground="red")
        self._log("info", "Disconnected")

    # ── Parameters + raw packet panels ──────────────────────────

    def _build_main_panes(self):
        panes = ttk.PanedWindow(self, orient="horizontal")
        panes.pack(fill="both", expand=True, padx=8, pady=4)

        # --- Parameters ---
        param_frame = ttk.LabelFrame(panes, text="Parameters (GET / SET)")
        panes.add(param_frame, weight=2)

        columns = ("id", "name", "access", "type", "unit", "group")
        self.param_tree = ttk.Treeview(param_frame, columns=columns, show="headings", height=10)
        for col, width in zip(columns, (50, 160, 60, 60, 70, 80)):
            self.param_tree.heading(col, text=col.upper())
            self.param_tree.column(col, width=width, anchor="w")
        self.param_tree.pack(fill="both", expand=True, padx=6, pady=6)
        self.param_tree.bind("<<TreeviewSelect>>", self._on_param_selected)

        entry_row = ttk.Frame(param_frame)
        entry_row.pack(fill="x", padx=6, pady=(0, 6))
        ttk.Label(entry_row, text="Value:").pack(side="left")
        self.param_value_var = tk.StringVar(value="0.0")
        ttk.Entry(entry_row, textvariable=self.param_value_var, width=12).pack(side="left", padx=4)
        ttk.Button(entry_row, text="Send GET", command=self._send_get).pack(side="left", padx=4)
        ttk.Button(entry_row, text="Send SET", command=self._send_set).pack(side="left", padx=4)
        self.param_hint_var = tk.StringVar(value="")
        ttk.Label(entry_row, textvariable=self.param_hint_var, foreground="#666").pack(side="left", padx=8)

        # --- Raw packet ---
        raw_frame = ttk.LabelFrame(panes, text="Raw packet (edge-case testing)")
        panes.add(raw_frame, weight=1)

        row1 = ttk.Frame(raw_frame); row1.pack(fill="x", padx=6, pady=4)
        ttk.Label(row1, text="msg_id:").pack(side="left")
        self.raw_msgid_var = tk.StringVar()
        self.raw_msgid_combo = ttk.Combobox(row1, textvariable=self.raw_msgid_var, width=14, state="readonly")
        self.raw_msgid_combo.pack(side="left", padx=4)

        row2 = ttk.Frame(raw_frame); row2.pack(fill="x", padx=6, pady=4)
        ttk.Label(row2, text="direction:").pack(side="left")
        self.raw_dir_var = tk.StringVar()
        self.raw_dir_combo = ttk.Combobox(row2, textvariable=self.raw_dir_var, width=14, state="readonly")
        self.raw_dir_combo.pack(side="left", padx=4)

        row3 = ttk.Frame(raw_frame); row3.pack(fill="x", padx=6, pady=4)
        ttk.Label(row3, text="payload (hex bytes):").pack(anchor="w")
        self.raw_payload_var = tk.StringVar(value="20 00 00 80 3F")
        ttk.Entry(row3, textvariable=self.raw_payload_var).pack(fill="x", pady=2)
        ttk.Label(row3, text="e.g. param 0x20 + float 1.0 little-endian, space separated", foreground="#666").pack(anchor="w")

        row4 = ttk.Frame(raw_frame); row4.pack(fill="x", padx=6, pady=4)
        ttk.Label(row4, text="version override (blank = protocol default):").pack(anchor="w")
        self.raw_version_var = tk.StringVar(value="")
        ttk.Entry(row4, textvariable=self.raw_version_var, width=10).pack(anchor="w", pady=2)
        ttk.Label(row4, text="set to any value ≠ current version to test VERSION_MISMATCH", foreground="#666").pack(anchor="w")

        row5 = ttk.Frame(raw_frame); row5.pack(fill="x", padx=6, pady=4)
        ttk.Label(row5, text="declared length override (blank = actual payload length):").pack(anchor="w")
        self.raw_length_var = tk.StringVar(value="")
        ttk.Entry(row5, textvariable=self.raw_length_var, width=10).pack(anchor="w", pady=2)
        ttk.Label(row5, text="set higher than the real payload to test the mid-packet timeout guard", foreground="#666").pack(anchor="w")

        row6 = ttk.Frame(raw_frame); row6.pack(fill="x", padx=6, pady=4)
        self.corrupt_crc_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(row6, text="Corrupt CRC (send CRC+1)", variable=self.corrupt_crc_var).pack(anchor="w")

        ttk.Button(raw_frame, text="Send Raw Packet", command=self._send_raw).pack(padx=6, pady=8, anchor="w")

    def _populate_param_tree(self):
        self.param_tree.delete(*self.param_tree.get_children())
        for p in sorted(self.proto.params_by_id.values(), key=lambda p: p.id):
            self.param_tree.insert("", "end", iid=str(p.id),
                                    values=(f"0x{p.id:02X}", p.name, p.access, p.type, p.unit, p.group))

    def _populate_raw_combos(self):
        self.raw_msgid_combo["values"] = list(self.proto.msgid_by_name.keys())
        if self.proto.msgid_by_name:
            self.raw_msgid_var.set(next(iter(self.proto.msgid_by_name)))
        self.raw_dir_combo["values"] = list(self.proto.dir_by_name.keys())
        if self.proto.dir_by_name:
            self.raw_dir_var.set(next(iter(self.proto.dir_by_name)))

    def _selected_param(self) -> Optional[Param]:
        sel = self.param_tree.selection()
        if not sel: return None
        return self.proto.params_by_id.get(int(sel[0]))

    def _on_param_selected(self, _event):
        p = self._selected_param()
        if not p: return
        self.param_value_var.set(str(p.default))
        hint = f"range [{p.min}, {p.max}]  access={p.access}"
        if not p.writable:
            hint += "  — read-only: SET should come back NACK READ_ONLY"
        self.param_hint_var.set(hint)

    # ── Edge-case suite panel ───────────────────────────────────

    def _build_edge_case_frame(self):
        frame = ttk.LabelFrame(self, text="Automated edge-case suite")
        frame.pack(fill="x", padx=8, pady=4)

        ttk.Button(frame, text="Run Edge Case Suite", command=self._run_edge_case_suite).pack(side="left", padx=6, pady=6)
        ttk.Label(
            frame,
            text="Runs: happy-path round trip, read-only SET, unknown param, wrong direction, "
                 "bad CRC (+ recovery), wrong version, length-lie timeout (+ recovery), unknown msg_id, heartbeat.",
            foreground="#666",
        ).pack(side="left", padx=8)

    def _pick_writable_param(self) -> Optional[Param]:
        for p in self.proto.params_by_id.values():
            if p.writable: return p
        return None

    def _pick_readonly_param(self) -> Optional[Param]:
        for p in self.proto.params_by_id.values():
            if not p.writable: return p
        return None

    def _pick_unknown_id(self) -> Optional[int]:
        for pid in range(256):
            if pid not in self.proto.params_by_id: return pid
        return None

    def _build_edge_case_defs(self) -> list:
        proto = self.proto
        writable = self._pick_writable_param()
        readonly = self._pick_readonly_param()
        unknown_id = self._pick_unknown_id()
        test_value = 4.25
        bad_version = 0x01 if proto.proto_version != 0x01 else 0x02
        length_lie_wait_ms = proto.timeout_ms + 250

        cases: list = []

        if writable:
            cases.append(EdgeCase(
                "SET writable parameter",
                f"SET {writable.name}=0x{writable.id:02X} to {test_value} — expect ACK",
                lambda: build_set(proto, writable.id, test_value),
                lambda info: info.msg_name == "ACK" and info.acked_msg_name == "CMD_SET",
            ))
            cases.append(EdgeCase(
                "GET writable parameter back",
                f"GET {writable.name} — expect value ≈ {test_value}",
                lambda: build_get(proto, writable.id),
                lambda info: info.msg_name == "CMD_GET" and len(info.payload) >= 5
                             and abs(struct.unpack("<f", info.payload[1:5])[0] - test_value) < 1e-3,
            ))
            cases.append(EdgeCase(
                "Wrong direction byte",
                f"CMD_GET on {writable.name} with direction=MCU_TO_PC — expect NACK BAD_DIRECTION",
                lambda: build_packet(proto, proto.msgid_by_name["CMD_GET"], proto.dir_by_name["MCU_TO_PC"],
                                      bytes([writable.id])),
                lambda info: info.msg_name == "NACK" and info.nack_error_name == "BAD_DIRECTION",
            ))
            cases.append(EdgeCase(
                "Corrupted CRC",
                f"valid GET on {writable.name} with CRC+1 — expect NACK BAD_CRC",
                lambda: build_packet(proto, proto.msgid_by_name["CMD_GET"], proto.dir_by_name["PC_TO_MCU"],
                                      bytes([writable.id]), corrupt_crc=True),
                lambda info: info.msg_name == "NACK" and info.nack_error_name == "BAD_CRC",
            ))
            cases.append(EdgeCase(
                "Recovery after bad CRC",
                "heartbeat sent immediately after — state machine must not be stuck",
                lambda: build_packet(proto, proto.msgid_by_name["HEARTBEAT"], proto.dir_by_name["PC_TO_MCU"], b""),
                lambda info: info.msg_name == "HEARTBEAT",
            ))
            cases.append(EdgeCase(
                "Wrong protocol version",
                f"version byte=0x{bad_version:02X} (actual=0x{proto.proto_version:02X}) — expect NACK VERSION_MISMATCH",
                lambda: build_packet(proto, proto.msgid_by_name["CMD_GET"], proto.dir_by_name["PC_TO_MCU"],
                                      bytes([writable.id]), version_override=bad_version),
                lambda info: info.msg_name == "NACK" and info.nack_error_name == "VERSION_MISMATCH",
            ))
            cases.append(EdgeCase(
                "Declared length longer than sent",
                f"length byte claims 50 bytes — expect NO reply within {length_lie_wait_ms}ms (timeout guard, not a NACK)",
                lambda: build_packet(proto, proto.msgid_by_name["CMD_GET"], proto.dir_by_name["PC_TO_MCU"],
                                      bytes([writable.id]), length_override=50),
                None,
                length_lie_wait_ms,
            ))
            cases.append(EdgeCase(
                "Recovery after length-lie timeout",
                "heartbeat sent after the stall — state machine must have reset back to IDLE",
                lambda: build_packet(proto, proto.msgid_by_name["HEARTBEAT"], proto.dir_by_name["PC_TO_MCU"], b""),
                lambda info: info.msg_name == "HEARTBEAT",
            ))

        if readonly:
            cases.append(EdgeCase(
                "SET read-only parameter",
                f"SET {readonly.name} (read-only) — expect NACK READ_ONLY",
                lambda: build_set(proto, readonly.id, 0.0),
                lambda info: info.msg_name == "NACK" and info.nack_error_name == "READ_ONLY",
            ))

        if unknown_id is not None:
            cases.append(EdgeCase(
                "GET unknown parameter id",
                f"GET 0x{unknown_id:02X} (not in protocol) — expect NACK UNKNOWN_PARAM",
                lambda: build_get(proto, unknown_id),
                lambda info: info.msg_name == "NACK" and info.nack_error_name == "UNKNOWN_PARAM",
            ))

        cases.append(EdgeCase(
            "Unknown message id (ACK sent by PC)",
            "PC sends an ACK frame (nothing should ever send us one) — expect NACK UNKNOWN_MSG",
            lambda: build_packet(proto, proto.msgid_by_name["ACK"], proto.dir_by_name["PC_TO_MCU"], b"\x00"),
            lambda info: info.msg_name == "NACK" and info.nack_error_name == "UNKNOWN_MSG",
        ))

        cases.append(EdgeCase(
            "Plain heartbeat",
            "expect a heartbeat back",
            lambda: build_packet(proto, proto.msgid_by_name["HEARTBEAT"], proto.dir_by_name["PC_TO_MCU"], b""),
            lambda info: info.msg_name == "HEARTBEAT",
        ))

        return cases

    def _run_edge_case_suite(self):
        if not self._require_ready(): return

        self._edge_cases = self._build_edge_case_defs()
        self._edge_case_results = []
        self._log("test", f"===== Running edge-case suite ({len(self._edge_cases)} tests) =====")
        self._run_next_edge_case(0)

    def _run_next_edge_case(self, index: int):
        if index >= len(self._edge_cases):
            passed = sum(1 for r in self._edge_case_results if r)
            total = len(self._edge_case_results)
            tag = "test" if passed == total else "fail"
            self._log(tag, f"===== Suite complete: {passed}/{total} passed =====")
            return

        case = self._edge_cases[index]
        send_time = time.monotonic()
        self._transmit(case.build())
        self._log("test", f"[{index + 1}/{len(self._edge_cases)}] {case.name} — {case.note}")
        self.after(case.wait_ms, lambda: self._check_edge_case(index, send_time))

    def _check_edge_case(self, index: int, send_time: float):
        case = self._edge_cases[index]
        relevant = [info for (ts, info) in self._recent_frames if ts >= send_time]

        if case.match is None:
            ok = len(relevant) == 0
        else:
            ok = any(case.match(info) for info in relevant)

        self._edge_case_results.append(ok)
        self._log("test" if ok else "fail", f"    -> {'PASS' if ok else 'FAIL'}")
        self._run_next_edge_case(index + 1)

    # ── Log panel ────────────────────────────────────────────────

    def _build_log_frame(self):
        frame = ttk.LabelFrame(self, text="Log")
        frame.pack(fill="both", expand=True, padx=8, pady=(4, 8))

        self.log_text = scrolledtext.ScrolledText(frame, height=16, font=("Courier New", 10), state="disabled")
        self.log_text.pack(fill="both", expand=True, padx=6, pady=(6, 0))

        ttk.Button(frame, text="Clear log", command=self._clear_log).pack(anchor="e", padx=6, pady=4)

    def _clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

    def _log(self, tag: str, text: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"[{ts}] {tag.upper():7} {text}\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    # ── Sending ──────────────────────────────────────────────────

    def _require_ready(self) -> bool:
        if self.proto is None:
            messagebox.showwarning("No protocol", "Load a protocol JSON first")
            return False
        if self.ser is None:
            messagebox.showwarning("Not connected", "Connect to a serial port first")
            return False
        if not self._board_ready:
            messagebox.showinfo(
                "Board still resetting",
                f"Still within the {BOARD_RESET_SETTLE_MS}ms post-connect settle window — "
                "the board likely just reset on DTR. Try again in a moment.",
            )
            return False
        return True

    def _transmit(self, frame: bytes):
        self.ser.write(frame)
        self._log("tx", f"{hexdump(frame):<40}  {decode_frame(self.proto, frame)}")

    def _send_get(self):
        if not self._require_ready(): return
        p = self._selected_param()
        if not p:
            messagebox.showwarning("No parameter", "Select a parameter in the table first")
            return
        self._transmit(build_get(self.proto, p.id))

    def _send_set(self):
        if not self._require_ready(): return
        p = self._selected_param()
        if not p:
            messagebox.showwarning("No parameter", "Select a parameter in the table first")
            return
        try:
            value = float(self.param_value_var.get())
        except ValueError:
            messagebox.showwarning("Bad value", "Value must be a number")
            return
        if not p.writable:
            self._log("info", f"'{p.name}' is read-only per protocol — sending anyway to check FW enforces it")
        self._transmit(build_set(self.proto, p.id, value))

    def _send_raw(self):
        if not self._require_ready(): return
        try:
            msg_id = self.proto.msgid_by_name[self.raw_msgid_var.get()]
            direction = self.proto.dir_by_name[self.raw_dir_var.get()]
            payload_str = self.raw_payload_var.get().strip()
            payload = bytes(int(b, 16) for b in payload_str.split()) if payload_str else b""

            version_str = self.raw_version_var.get().strip()
            length_str = self.raw_length_var.get().strip()
            version_override = int(version_str, 0) if version_str else None
            length_override = int(length_str, 0) if length_str else None
        except (KeyError, ValueError) as e:
            messagebox.showwarning("Bad raw packet", f"Couldn't build packet: {e}")
            return

        frame = build_packet(
            self.proto, msg_id, direction, payload,
            corrupt_crc=self.corrupt_crc_var.get(),
            version_override=version_override,
            length_override=length_override,
        )
        self._transmit(frame)

    # ── RX handling ──────────────────────────────────────────────

    def _poll_rx_queue(self):
        try:
            while True:
                kind, payload = self.rx_queue.get_nowait()
                if kind == "raw":
                    #self._log("rx-raw", hexdump(payload))
                    self._rx_buf.extend(payload)
                    if self.proto:
                        for frame in extract_frames(self._rx_buf, self.proto):
                            info = parse_frame(self.proto, frame)
                            if info is not None:
                                self._recent_frames.append((time.monotonic(), info))
                            self._log("rx", f"{hexdump(frame):<40}  {decode_frame(self.proto, frame)}")
                elif kind == "error":
                    self._log("error", payload)
                    self._disconnect()
        except queue.Empty:
            pass

        # Trim old frames so this doesn't grow unbounded over a long session.
        cutoff = time.monotonic() - 10.0
        if self._recent_frames:
            self._recent_frames = [x for x in self._recent_frames if x[0] > cutoff]

        self.after(50, self._poll_rx_queue)

    def _on_close(self):
        self._disconnect()
        self.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    args = parser.parse_args()

    app = SerialTestTool(args.protocol)
    app.mainloop()


if __name__ == "__main__":
    main()