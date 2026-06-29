#!/usr/bin/env python3
"""
fastboot_tool.py  —  ZephyrWatch fastboot host tool
====================================================
Speaks the OKAY / FAIL / INFO text protocol over USB-Serial (CDC).
Mirrors the standard `fastboot` CLI interface as closely as possible.

Requirements:
    pip install pyserial

Usage:
    python fastboot_tool.py --port /dev/ttyACM0 devices
    python fastboot_tool.py --port /dev/ttyACM0 getvar version
    python fastboot_tool.py --port /dev/ttyACM0 getvar all
    python fastboot_tool.py --port /dev/ttyACM0 rtc
    python fastboot_tool.py --port /dev/ttyACM0 rtc set 2026-05-03 14:30:00
    python fastboot_tool.py --port /dev/ttyACM0 reboot
    python fastboot_tool.py --port /dev/ttyACM0 oem version
    python fastboot_tool.py --port /dev/ttyACM0 oem dump-rtc
    python fastboot_tool.py --port COM3 pins
    python fastboot_tool.py --port COM3 memory

Auto-detect port (Linux/Mac):
    python fastboot_tool.py devices          # scans for ACM/USB serial devices

Flags:
    --port  / -p   serial port  (default: auto-detect)
    --baud  / -b   baud rate    (default: 115200)
    --timeout / -t read timeout (default: 3.0 s)
    --verbose / -v print raw protocol lines
"""

from __future__ import annotations

import sys
import time
import argparse
import os
import glob
from typing import Optional

# ── Dependency check ──────────────────────────────────────────────────────────

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed.  Run:  pip install pyserial", file=sys.stderr)
    sys.exit(1)

# ── ANSI colors (disabled on Windows unless VT mode) ─────────────────────────

_USE_COLOR = sys.stdout.isatty() and os.name != "nt"

def _c(code: str, text: str) -> str:
    return f"\033[{code}m{text}\033[0m" if _USE_COLOR else text

def green(t: str)  -> str: return _c("32", t)
def red(t: str)    -> str: return _c("31", t)
def yellow(t: str) -> str: return _c("33", t)
def cyan(t: str)   -> str: return _c("36", t)
def bold(t: str)   -> str: return _c("1",  t)

# ── Port auto-detection ───────────────────────────────────────────────────────

def find_device_port() -> Optional[str]:
    """Scan for a ZephyrWatch / ESP32 USB-Serial port."""
    candidates: list[str] = []

    # pyserial list_ports
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        mfg  = (p.manufacturer or "").lower()
        if any(kw in desc or kw in mfg for kw in
               ("esp32", "cp210", "ch340", "ch341", "ftdi", "cdc", "serial")):
            candidates.append(p.device)

    # Fallback: glob
    if not candidates:
        for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/cu.usbmodem*"):
            candidates.extend(glob.glob(pattern))

    return candidates[0] if candidates else None

# ── Low-level protocol ────────────────────────────────────────────────────────

class FastbootProtocol:
    """Thin wrapper around a serial.Serial port for the text fastboot protocol."""

    def __init__(self, port: str, baud: int = 115_200, timeout: float = 3.0,
                 verbose: bool = False):
        self.port    = port
        self.verbose = verbose
        self._ser    = serial.Serial(port, baud, timeout=0.1)
        time.sleep(0.15)          # let device settle after DTR toggle
        self._ser.reset_input_buffer()

    def close(self):
        self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # ── Send a command and collect INFO + final OKAY/FAIL ─────────────────────

    def command(self, cmd: str, timeout: float = 3.0) -> tuple[list[str], bool, str]:
        """
        Send `cmd` and read response lines until OKAY or FAIL.

        Returns:
            (info_lines, ok, final_msg)
              info_lines  — list of INFO message strings
              ok          — True if OKAY, False if FAIL or timeout
              final_msg   — the message after OKAY/FAIL
        """
        self._ser.reset_input_buffer()
        wire = (cmd + "\n").encode()
        self._ser.write(wire)
        self._ser.flush()
        if self.verbose:
            print(f"  {cyan('>>>')} {cmd}")

        info_lines: list[str] = []
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            raw = self._ser.readline()
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="replace").strip()
            except Exception:
                continue

            if not line:
                continue

            if self.verbose:
                print(f"  {yellow('<<<')} {line}")

            if line.startswith("INFO"):
                info_lines.append(line[4:])
            elif line.startswith("OKAY"):
                return info_lines, True, line[4:]
            elif line.startswith("FAIL"):
                return info_lines, False, line[4:]
            # Ignore prompt lines ("fastboot> ") and echo

        return info_lines, False, "timeout — no response from device"

# ── Command implementations ───────────────────────────────────────────────────

def cmd_devices(fb: FastbootProtocol, args: list[str], opts) -> int:
    info, ok, msg = fb.command("devices", opts.timeout)
    if ok:
        # msg contains "ZW-ESP32S3-001\tfastboot"
        serial_no = msg.strip() if msg.strip() else "ZW-ESP32S3-001\tfastboot"
        print(serial_no)
    else:
        print(red(f"error: {msg}"), file=sys.stderr)
        return 1
    return 0


def cmd_getvar(fb: FastbootProtocol, args: list[str], opts) -> int:
    var  = args[0] if args else "all"
    # Protocol framing: "getvar:version" or "getvar:all"
    info, ok, msg = fb.command(f"getvar:{var}", opts.timeout)

    for line in info:
        # line is "version:0.4" style — pretty-print
        if ":" in line:
            k, _, v = line.partition(":")
            print(f"{bold(k)}: {v}")
        else:
            print(line)

    if ok:
        if msg:  # single-var response comes in OKAY
            if ":" in msg:
                k, _, v = msg.partition(":")
                print(f"{bold(k)}: {v}")
            else:
                print(msg)
    else:
        print(red(f"FAIL: {msg}"), file=sys.stderr)
        return 1
    return 0


def cmd_reboot(fb: FastbootProtocol, args: list[str], opts) -> int:
    sub   = args[0] if args else ""
    cmd   = "reboot bootloader" if sub == "bootloader" else "reboot"
    _, ok, msg = fb.command(cmd, opts.timeout)
    if ok:
        print(green(f"rebooting: {msg}"))
    else:
        print(red(f"FAIL: {msg}"), file=sys.stderr)
        return 1
    return 0


def cmd_rtc(fb: FastbootProtocol, args: list[str], opts) -> int:
    if args and args[0] == "set":
        # rtc set YYYY-MM-DD HH:MM:SS
        date_str = args[1] if len(args) > 1 else ""
        time_str = args[2] if len(args) > 2 else ""
        wire_cmd = f"rtc set {date_str} {time_str}".strip()
    else:
        wire_cmd = "rtc"

    info, ok, msg = fb.command(wire_cmd, opts.timeout)
    for line in info:
        print(line)
    if ok:
        print(green(msg if msg else "OK"))
    else:
        print(red(f"FAIL: {msg}"), file=sys.stderr)
        return 1
    return 0


def cmd_memory(fb: FastbootProtocol, args: list[str], opts) -> int:
    info, ok, msg = fb.command("memory", opts.timeout)
    for line in info:
        print(line)
    if ok:
        print(green("memory snapshot sent to device debug console"))
    else:
        print(red(f"FAIL: {msg}"), file=sys.stderr)
        return 1
    return 0


def cmd_pins(fb: FastbootProtocol, args: list[str], opts) -> int:
    info, ok, msg = fb.command("pins", opts.timeout)
    for line in info:
        print(line)
    if not ok:
        print(red(f"FAIL: {msg}"), file=sys.stderr)
        return 1
    return 0


def cmd_help(fb: FastbootProtocol, args: list[str], opts) -> int:
    info, ok, _ = fb.command("help", opts.timeout)
    for line in info:
        print(line)
    if not ok:
        return 1
    return 0


def cmd_set_active(fb: FastbootProtocol, args: list[str], opts) -> int:
    slot = args[0] if args else ""
    if slot not in ("a", "b", "A", "B"):
        print(red("usage: set_active <a|b>"), file=sys.stderr)
        return 1
    _, ok, msg = fb.command(f"set_active {slot}", opts.timeout)
    if ok:
        print(green(f"active slot: {slot.lower()}"))
    else:
        print(red(f"FAIL: {msg}"), file=sys.stderr)
        return 1
    return 0


def cmd_flashing(fb: FastbootProtocol, args: list[str], opts) -> int:
    sub = args[0] if args else ""
    if sub not in ("unlock", "lock", "get_unlock_ability"):
        print(red("usage: flashing unlock | lock | get_unlock_ability"), file=sys.stderr)
        return 1
    info, ok, msg = fb.command(f"flashing {sub}", opts.timeout)
    for line in info:
        print(yellow(line))   # warnings in yellow
    if ok:
        print(green(msg if msg else "OK"))
    else:
        print(red(f"FAIL: {msg}"), file=sys.stderr)
        return 1
    return 0


def cmd_oem(fb: FastbootProtocol, args: list[str], opts) -> int:
    sub_cmd = " ".join(args) if args else "help"
    info, ok, msg = fb.command(f"oem {sub_cmd}", opts.timeout)
    for line in info:
        print(line)
    if ok:
        if msg:
            print(green(msg))
    else:
        print(red(f"FAIL: {msg}"), file=sys.stderr)
        return 1
    return 0


def cmd_flash(fb: FastbootProtocol, args: list[str], opts) -> int:
    if not args:
        print(red("usage: flash <partition> [image.bin]"), file=sys.stderr)
        return 1
    partition = args[0]
    info, ok, msg = fb.command(f"flash {partition}", opts.timeout)
    for line in info:
        print(line)
    if ok:
        print(green(msg if msg else "flash OK"))
    else:
        print(red(f"FAIL: {msg}"), file=sys.stderr)
        return 1
    return 0


def cmd_erase(fb: FastbootProtocol, args: list[str], opts) -> int:
    if not args:
        print(red("usage: erase <partition>"), file=sys.stderr)
        return 1
    partition = args[0]
    info, ok, msg = fb.command(f"erase {partition}", opts.timeout)
    for line in info:
        print(line)
    if ok:
        print(green(msg if msg else "erase OK"))
    else:
        print(red(f"FAIL: {msg}"), file=sys.stderr)
        return 1
    return 0


# ── Command dispatch table ────────────────────────────────────────────────────

COMMANDS: dict[str, tuple] = {
    "devices":     (cmd_devices,    "list connected fastboot devices"),
    "getvar":      (cmd_getvar,     "getvar <var>|all  — query bootloader variable"),
    "reboot":      (cmd_reboot,     "reboot [bootloader]  — reset the device"),
    "rtc":         (cmd_rtc,        "rtc [set YYYY-MM-DD HH:MM:SS]  — read/set RTC"),
    "memory":      (cmd_memory,     "dump memory to device debug console"),
    "mem":         (cmd_memory,     "alias for memory"),
    "pins":        (cmd_pins,       "show GPIO pin map"),
    "help":        (cmd_help,       "list all device commands"),
    "set_active":  (cmd_set_active, "set_active <a|b>  — select A/B slot"),
    "flashing":    (cmd_flashing,   "flashing unlock|lock|get_unlock_ability"),
    "oem":         (cmd_oem,        "oem <sub-command>  — OEM commands"),
    "flash":       (cmd_flash,      "flash <partition>  — write partition (stub)"),
    "erase":       (cmd_erase,      "erase <partition>  — erase partition (stub)"),
}

# ── CLI ───────────────────────────────────────────────────────────────────────

def print_host_help():
    print(bold("fastboot_tool.py") + "  —  ZephyrWatch fastboot host tool\n")
    print("Usage:")
    print("  fastboot_tool.py [--port PORT] <command> [args...]\n")
    print("Options:")
    print("  --port  / -p   serial port    (auto-detect if omitted)")
    print("  --baud  / -b   baud rate      (default 115200)")
    print("  --timeout / -t response timeout in seconds (default 3.0)")
    print("  --verbose / -v show raw protocol lines\n")
    print("Commands:")
    seen = set()
    for name, (_, desc) in COMMANDS.items():
        fn_id = id(COMMANDS[name][0])
        if fn_id not in seen:
            seen.add(fn_id)
            print(f"  {bold(name):22} {desc}")
    print()
    print("Examples:")
    print("  fastboot_tool.py --port /dev/ttyACM0 devices")
    print("  fastboot_tool.py --port COM3 getvar all")
    print("  fastboot_tool.py --port /dev/ttyACM0 rtc set 2026-05-03 14:30:00")
    print("  fastboot_tool.py --port /dev/ttyACM0 oem version")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="ZephyrWatch fastboot host tool",
        add_help=False,
    )
    p.add_argument("--port",    "-p", default=None,  help="serial port")
    p.add_argument("--baud",    "-b", type=int, default=115_200)
    p.add_argument("--timeout", "-t", type=float, default=3.0)
    p.add_argument("--verbose", "-v", action="store_true")
    p.add_argument("--help",    "-h", action="store_true")
    p.add_argument("command",   nargs="*")
    return p


def main() -> int:
    parser = build_parser()
    opts   = parser.parse_args()

    if opts.help or not opts.command:
        print_host_help()
        return 0

    verb      = opts.command[0]
    cmd_args  = opts.command[1:]

    if verb not in COMMANDS:
        print(red(f"unknown command: {verb}"), file=sys.stderr)
        print(f"Run  fastboot_tool.py --help  for the command list.", file=sys.stderr)
        return 1

    # ── Resolve port ──────────────────────────────────────────────────────────
    port = opts.port
    if port is None:
        port = find_device_port()
        if port is None:
            print(red("ERROR: no serial port found.  Use --port to specify one."),
                  file=sys.stderr)
            return 1
        print(f"auto-detected port: {cyan(port)}", file=sys.stderr)

    # ── Execute ───────────────────────────────────────────────────────────────
    try:
        with FastbootProtocol(port, opts.baud, opts.timeout, opts.verbose) as fb:
            fn = COMMANDS[verb][0]
            return fn(fb, cmd_args, opts)
    except serial.SerialException as e:
        print(red(f"serial error: {e}"), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print()
        return 0


if __name__ == "__main__":
    sys.exit(main())
