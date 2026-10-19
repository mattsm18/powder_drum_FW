#!/usr/bin/env python3
#
# Title: scripts/generate_protocol.py
# Purpose:
# - Read the single-source-of-truth comms protocol definition
#   (config/pd_comms_protocol_v1.0.json) and generate the firmware-side
#   C++ header (include/protocol_generated.h).
# - The Python/software side does NOT need a generated file: it reads the
#   same JSON dynamically at runtime via config/protocol_config.py.
#
# Run this any time pd_comms_protocol_v1.0.json changes, before building
# firmware. Wire it up as a PlatformIO extra_script (pre-build) so it can
# never go stale: https://docs.platformio.org/en/latest/scripting/index.html
#
# Usage:
#   python scripts/generate_protocol.py
#   python scripts/generate_protocol.py --protocol path/to/other.json --out path/to/header.h

import argparse
import json
import sys
import re
from pathlib import Path

REPO_ROOT           = Path(__file__).resolve().parent.parent
DEFAULT_HEADER_OUT = REPO_ROOT / "include" / "protocol_generated.h"

# Regex to find file with latest version and run with it..
def find_latest_protocol() -> Path:
    config_dir = REPO_ROOT / "config"

    candidates = list(config_dir.glob("pd_comms_protocol_v*.json"))
    if not candidates:
        sys.exit(f"No protocol files found in {config_dir}")

    version_re = re.compile(r"pd_comms_protocol_v(\d+(?:\.\d+)*)\.json$")

    def version(path: Path):
        m = version_re.match(path.name)
        if not m: return ()

        return tuple(int(x) for x in m.group(1).split("."))

    latest = max(candidates, key=version)
    return latest

DEFAULT_PROTOCOL = find_latest_protocol()

TYPE_ENUM = {
    "float": "PARAM_TYPE_FLOAT",
    "int":   "PARAM_TYPE_INT",
    "bool":  "PARAM_TYPE_BOOL",
}


def load_protocol(path: Path) -> dict:
    with open(path, "r") as f:
        data = json.load(f)

    _validate(data)
    return data


def _validate(data: dict):
    ids, names = set(), set()
    for p in data["parameters"]:
        pid = int(p["id"], 16)
        if pid in ids:
            sys.exit(f"generate_protocol: duplicate parameter id {p['id']!r} ({p['name']})")
        if p["name"] in names:
            sys.exit(f"generate_protocol: duplicate parameter name {p['name']!r}")
        if p["type"] not in TYPE_ENUM:
            sys.exit(f"generate_protocol: unknown type {p['type']!r} for parameter {p['name']!r}")
        if not (0 <= pid <= 0xFF):
            sys.exit(f"generate_protocol: parameter id {p['id']} out of uint8_t range ({p['name']})")
        ids.add(pid)
        names.add(p["name"])


def _access_flags_expr(access: list[str]) -> str:
    flags = []
    if "r" in access: flags.append("PARAM_ACCESS_R")
    if "w" in access: flags.append("PARAM_ACCESS_W")
    return " | ".join(flags) if flags else "0"


# Convert e.g. "encoderAngularVelocity" -> "ENCODERANGULARVELOCITY" for the
# ParamID namespace. Kept as a straight uppercase of the JSON `name` so it's
# trivially traceable back to the source of truth.
def _param_const_name(name: str) -> str: return name.upper()

def render_header(data: dict, protocol_path: Path) -> str:
    frame  = data["frame"]
    direc  = data["direction"]
    msgid  = data["msg_id"]
    nack   = data["nack_error"]
    params = data["parameters"]

    lines = []
    lines.append("// ============================================================")
    lines.append("// AUTO-GENERATED FILE \u2014 DO NOT EDIT BY HAND")
    lines.append(
    f"// Source:    {protocol_path.relative_to(REPO_ROOT)} "
    f"(protocol v{data['protocol_version']})"
    )
    lines.append("// Generator: scripts/generate_protocol.py")
    lines.append("// Regenerate with: python scripts/generate_protocol.py")
    lines.append("// ============================================================")
    lines.append("")
    lines.append("#ifndef PROTOCOL_GENERATED_H")
    lines.append("#define PROTOCOL_GENERATED_H")
    lines.append("")
    lines.append("#include <stdint.h>")
    lines.append("#include <avr/pgmspace.h>")
    lines.append("")
    lines.append("// ---- Frame constants ----")
    lines.append(f"#define SOF_BYTE                    {frame['sof_byte']}")
    lines.append(f"#define SERIAL_PROTOCOL_VERSION     {frame['version']}")
    lines.append(f"#define HEADER_SIZE_BYTES           {frame['header_size_bytes']}")
    lines.append(f"#define MAX_PAYLOAD_BYTES           {frame['max_payload_bytes']}")
    lines.append(f"#define TIMEOUT_MS                  {frame['timeout_ms']}")
    lines.append(f"#define MAX_RETRIES                 {frame['max_retries']}")
    lines.append("")
    lines.append("// ---- Direction ----")
    for name, val in direc.items():
        lines.append(f"#define DIR_{name:<24} {val}")
    lines.append("")
    lines.append("// ---- Message IDs ----")
    lines.append("enum MsgID : uint8_t {")
    for name, val in msgid.items():
        lines.append(f"    MSG_{name:<12} = {val},")
    lines.append("};")
    lines.append("")
    lines.append("// ---- NACK error codes ----")
    lines.append("enum NackError : uint8_t {")
    for name, val in nack.items():
        lines.append(f"    ERR_{name:<18} = {val},")
    lines.append("};")
    lines.append("")
    lines.append("// ---- Parameter access flags (bitmask) ----")
    lines.append("#define PARAM_ACCESS_R 0x01")
    lines.append("#define PARAM_ACCESS_W 0x02")
    lines.append("")
    lines.append("// ---- Parameter logical types ----")
    lines.append("// NB: every parameter is still carried on the wire as a 4-byte IEEE-754")
    lines.append("// float (see SerialHandler::sendParameter / MSG_CMD_SET payload layout).")
    lines.append("// `type` here is metadata for validation/UI only, not the wire encoding.")
    lines.append("enum ParamType : uint8_t {")
    lines.append("    PARAM_TYPE_FLOAT = 0,")
    lines.append("    PARAM_TYPE_INT   = 1,")
    lines.append("    PARAM_TYPE_BOOL  = 2,")
    lines.append("};")
    lines.append("")
    lines.append("// ---- Parameter IDs ----")
    lines.append("// Use these instead of magic hex when binding parameters in main.cpp.")
    lines.append("namespace ParamID {")
    for p in params:
        const = _param_const_name(p["name"])
        ro = "" if "w" in p["access"] else "  (read-only)"
        unit = f" {p['unit']}" if p["unit"] else ""
        lines.append(f"    constexpr uint8_t {const:<26} = {p['id']}; // {p['label']}{unit} [{p['group']}]{ro}")
    lines.append("}")
    lines.append("")
    lines.append("// ---- Parameter metadata table ----")
    lines.append("// SerialHandler uses this to reject SET on read-only params and GET/SET on")
    lines.append("// unknown ids generically, without per-parameter code. Lives in flash (PROGMEM)")
    lines.append("// since the AVR target's SRAM is scarce.")
    lines.append("struct ParamMeta {")
    lines.append("    uint8_t id;")
    lines.append("    uint8_t type;")
    lines.append("    uint8_t access;")
    lines.append("};")
    lines.append("")
    lines.append("const ParamMeta PARAM_TABLE[] PROGMEM = {")
    for p in params:
        const = _param_const_name(p["name"])
        lines.append(f"    {{ ParamID::{const}, {TYPE_ENUM[p['type']]}, {_access_flags_expr(p['access'])} }},")
    lines.append("};")
    lines.append("")
    lines.append("#define PARAM_TABLE_LEN (sizeof(PARAM_TABLE) / sizeof(PARAM_TABLE[0]))")
    lines.append("")
    lines.append("#endif // PROTOCOL_GENERATED_H")
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--protocol", type=Path, default=DEFAULT_PROTOCOL)
    parser.add_argument("--out",      type=Path, default=DEFAULT_HEADER_OUT)
    args = parser.parse_args()

    data = load_protocol(args.protocol)
    header = render_header(data, args.protocol)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(header)

    print(f"generate_protocol: wrote {args.out} ({len(data['parameters'])} parameters, protocol v{data['protocol_version']})")


if __name__ == "__main__":
    main()