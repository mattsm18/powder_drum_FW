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
#       payload bytes, optional corrupted CRC) to test edge cases like
#       SET-on-read-only, unknown parameter ids, bad CRC, etc.
#     * see every TX/RX frame, decoded, in a scrolling log
#
# Requires: pyserial   (pip install pyserial --break-system-packages)
#
# Usage:
#   python test/serial_test_tool.py
#   python test/serial_test_tool.py --protocol path/to/pd_comms_protocol_v1.0.json

import argparse
import json
import queue
import struct
import sys
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, scrolledtext, ttk
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial is required: pip install pyserial --break-system-packages")
    sys.exit(1)


REPO_ROOT        = Path(__file__).resolve().parent.parent
DEFAULT_PROTOCOL = REPO_ROOT / "config" / "pd_comms_protocol_v1.0.json"


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


def build_packet(proto: ProtocolDef, msg_id: int, direction: int, payload: bytes, corrupt_crc: bool = False) -> bytes:
    header = bytes([proto.sof, proto.proto_version, msg_id, direction, len(payload)])
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


def decode_frame(proto: ProtocolDef, frame: bytes) -> str:
    if len(frame) < proto.header_size + 1:
        return "malformed (shorter than header+crc)"

    version, msg_id, dir_, length = frame[1], frame[2], frame[3], frame[4]
    payload = frame[5:5 + length]
    crc = frame[5 + length] if len(frame) > 5 + length else None
    computed = compute_crc(frame[:5 + length])

    msg_name = proto.msgid_by_value.get(msg_id, f"UNKNOWN(0x{msg_id:02X})")
    dir_name = proto.dir_by_value.get(dir_, f"0x{dir_:02X}")

    bits = [f"ver=0x{version:02X}", f"msg={msg_name}", f"dir={dir_name}", f"len={length}"]

    if version != proto.proto_version:
        bits.append("VERSION MISMATCH")
    if crc is None or crc != computed:
        bits.append(f"CRC MISMATCH (recv={crc if crc is None else f'0x{crc:02X}'} calc=0x{computed:02X})")

    detail = _decode_payload(proto, msg_name, payload)
    if detail: bits.append(detail)
    return "  |  ".join(bits)


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


def hexdump(frame: bytes) -> str:
    return " ".join(f"{b:02X}" for b in frame)


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
        buf = bytearray()
        while not self._stop.is_set():
            try:
                data = self._ser.read(self._ser.in_waiting or 1)
            except (serial.SerialException, OSError) as e:
                self._rx_queue.put(("error", str(e)))
                return

            if not data:
                continue

            buf.extend(data)
            self._rx_queue.put(("raw", bytes(data)))


# ══════════════════════════════════════════════════════════════
# GUI
# ══════════════════════════════════════════════════════════════

class SerialTestTool(tk.Tk):
    def __init__(self, initial_protocol: Path):
        super().__init__()
        self.title("PD Comms Serial Test Tool")
        self.geometry("980x680")

        self.proto: Optional[ProtocolDef] = None
        self.ser: Optional[serial.Serial] = None
        self.reader: Optional[SerialReader] = None
        self.rx_queue: "queue.Queue" = queue.Queue()
        self._rx_buf = bytearray()

        self._build_protocol_frame(initial_protocol)
        self._build_connection_frame()
        self._build_main_panes()
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
            f"v{self.proto.version} — {len(self.proto.params_by_id)} parameters loaded from {path.name}"
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
        self.conn_status_var.set(f"connected: {port} @ {baud}")
        self.conn_status_lbl.config(foreground="green")
        self._log("info", f"Connected to {port} @ {baud}")

    def _disconnect(self):
        if self.reader:
            self.reader.stop()
            self.reader = None
        if self.ser:
            try: self.ser.close()
            except (serial.SerialException, OSError): pass
            self.ser = None
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
        self.corrupt_crc_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(row4, text="Corrupt CRC (send CRC+1)", variable=self.corrupt_crc_var).pack(anchor="w")

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
            hint += "  — read-only: SET should come back NACK"
        self.param_hint_var.set(hint)

    # ── Log panel ────────────────────────────────────────────────

    def _build_log_frame(self):
        frame = ttk.LabelFrame(self, text="Log")
        frame.pack(fill="both", expand=True, padx=8, pady=(4, 8))

        self.log_text = scrolledtext.ScrolledText(frame, height=14, font=("Courier New", 10), state="disabled")
        self.log_text.pack(fill="both", expand=True, padx=6, pady=(6, 0))

        ttk.Button(frame, text="Clear log", command=self._clear_log).pack(anchor="e", padx=6, pady=4)

    def _clear_log(self):
        self.log_text.config(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.config(state="disabled")

    def _log(self, tag: str, text: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"[{ts}] {tag.upper():5} {text}\n")
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
        except (KeyError, ValueError) as e:
            messagebox.showwarning("Bad raw packet", f"Couldn't build packet: {e}")
            return

        frame = build_packet(self.proto, msg_id, direction, payload, corrupt_crc=self.corrupt_crc_var.get())
        self._transmit(frame)

    # ── RX handling ──────────────────────────────────────────────

    def _poll_rx_queue(self):
        try:
            while True:
                kind, payload = self.rx_queue.get_nowait()
                if kind == "raw":
                    self._rx_buf.extend(payload)
                    if self.proto:
                        for frame in extract_frames(self._rx_buf, self.proto):
                            self._log("rx", f"{hexdump(frame):<40}  {decode_frame(self.proto, frame)}")
                elif kind == "error":
                    self._log("error", payload)
                    self._disconnect()
        except queue.Empty:
            pass
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