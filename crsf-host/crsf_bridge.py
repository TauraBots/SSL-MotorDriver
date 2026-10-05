#!/usr/bin/env python3
"""Minimal CRSF bridge for the first 3-robot ELRS match-mode test."""

from __future__ import annotations

import argparse
import json
import queue
import signal
import sys
import threading
import time
from typing import TextIO

from crsf_match import RobotCommand, encode_rc_channels_frame, encode_team_channels


DEFAULT_BRIDGE_BAUD = 921600
DEFAULT_RATE_HZ = 333.0
STARTUP_SAFE_SECONDS = 0.25
HOST_COMMAND_TIMEOUT_MS = 100
SAFE_SHUTDOWN_FRAMES = 20


def test_commands() -> dict[str, RobotCommand]:
    return {
        "A": RobotCommand(vx=1.0, vy=0.0, omega=0.0),
        "B": RobotCommand(vx=0.0, vy=1.0, omega=0.0),
        "C": RobotCommand(vx=0.0, vy=0.0, omega=1.0),
    }


def disabled_commands() -> dict[str, RobotCommand]:
    return {
        "A": RobotCommand(brake=True),
        "B": RobotCommand(brake=True),
        "C": RobotCommand(brake=True),
    }


def load_json_commands(line: str) -> dict[str, RobotCommand]:
    raw = json.loads(line)
    commands: dict[str, RobotCommand] = {}
    for robot_id in ("A", "B", "C"):
        item = raw.get(robot_id, {})
        commands[robot_id] = RobotCommand(
            vx=float(item.get("vx", 0.0)),
            vy=float(item.get("vy", 0.0)),
            omega=float(item.get("omega", 0.0)),
            kick=bool(item.get("kick", False)),
            chip=bool(item.get("chip", False)),
            brake=bool(item.get("brake", False)),
            dribbler=bool(item.get("dribbler", False)),
            kick_power=int(item.get("kick_power", 0)),
        )
    return commands


class CommandSource:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.current = disabled_commands()
        self.enabled = False

    def startup_safe_frame(self) -> bytes:
        return encode_rc_channels_frame(encode_team_channels(disabled_commands(), enabled=False))

    def active_frame(self) -> bytes:
        if self.args.disable:
            self.current = disabled_commands()
            self.enabled = False
        elif self.args.test:
            self.current = test_commands()
            self.enabled = True
        return encode_rc_channels_frame(encode_team_channels(self.current, enabled=self.enabled))


class JsonCommandSource(CommandSource):
    def __init__(self, args: argparse.Namespace, stream: TextIO) -> None:
        super().__init__(args)
        self.stream = stream
        self.last_update = 0.0
        self.updates: queue.Queue[dict[str, RobotCommand]] = queue.Queue()
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()

    def _reader_loop(self) -> None:
        while True:
            line = self.stream.readline()
            if line == "":
                return
            try:
                self.updates.put(load_json_commands(line))
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                print(f"ignoring invalid JSON command: {exc}", file=sys.stderr)

    def _consume_updates(self) -> None:
        latest = None
        while True:
            try:
                latest = self.updates.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            self.current = latest
            self.enabled = bool(self.args.enable)
            self.last_update = time.perf_counter()

    def active_frame(self) -> bytes:
        if self.args.disable:
            return super().active_frame()
        self._consume_updates()
        timeout_s = max(0.0, self.args.command_timeout_ms / 1000.0)
        if self.last_update <= 0.0 or (time.perf_counter() - self.last_update) > timeout_s:
            self.current = disabled_commands()
            self.enabled = False
        return encode_rc_channels_frame(encode_team_channels(self.current, enabled=self.enabled))


def open_serial(port: str, baud: int):
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: python -m pip install pyserial") from exc
    return serial.Serial(port=port, baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=0)


def send_safe_shutdown(serial_port, source: CommandSource, frames: int) -> None:
    safe = source.startup_safe_frame()
    for _ in range(max(0, frames)):
        serial_port.write(safe)
    serial_port.flush()


def transmit_loop(serial_port, source: CommandSource, rate_hz: float,
                  startup_safe_seconds: float, shutdown_safe_frames: int) -> None:
    period = 1.0 / rate_hz
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        start = time.perf_counter()
        next_tick = start
        while running:
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(min(next_tick - now, period / 2.0))
                continue
            if now - start < startup_safe_seconds:
                frame = source.startup_safe_frame()
            else:
                frame = source.active_frame()
            serial_port.write(frame)
            next_tick += period
            if next_tick < now - period:
                next_tick = now + period
    finally:
        send_safe_shutdown(serial_port, source, shutdown_safe_frames)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Send CRSF RC_CHANNELS_PACKED frames to an ELRS TX module.")
    parser.add_argument("--port", required=True, help="Serial port connected to the ELRS TX module")
    parser.add_argument("--baud", type=int, default=DEFAULT_BRIDGE_BAUD,
                        help=f"Bridge -> ELRS TX baud rate (default: {DEFAULT_BRIDGE_BAUD})")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ,
                        help=f"Transmit rate in frames/s (default: {DEFAULT_RATE_HZ:g})")
    parser.add_argument("--test", action="store_true",
                        help="Send A vx=+1, B vy=+1, C omega=+1 with CH16 enabled after startup safe frames")
    parser.add_argument("--disable", action="store_true",
                        help="Keep CH16 disabled and command all robots to stop")
    parser.add_argument("--stdin-json", action="store_true",
                        help="Read one JSON command object per frame from stdin")
    parser.add_argument("--enable", action="store_true",
                        help="Enable CH16 for --stdin-json commands after startup safe frames")
    parser.add_argument("--startup-safe-seconds", type=float, default=STARTUP_SAFE_SECONDS,
                        help=f"Seconds to transmit CH16 disabled on startup (default: {STARTUP_SAFE_SECONDS})")
    parser.add_argument("--command-timeout-ms", type=int, default=HOST_COMMAND_TIMEOUT_MS,
                        help=f"Disable CH16 if --stdin-json has no valid update within this timeout (default: {HOST_COMMAND_TIMEOUT_MS})")
    parser.add_argument("--shutdown-safe-frames", type=int, default=SAFE_SHUTDOWN_FRAMES,
                        help=f"Number of CH16-disabled frames to send before closing serial (default: {SAFE_SHUTDOWN_FRAMES})")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if args.rate <= 0:
        raise SystemExit("--rate must be positive")
    if args.disable and args.test:
        raise SystemExit("--disable and --test are mutually exclusive")
    if args.command_timeout_ms < 0:
        raise SystemExit("--command-timeout-ms must be non-negative")

    source: CommandSource
    if args.stdin_json:
        source = JsonCommandSource(args, sys.stdin)
    else:
        source = CommandSource(args)

    with open_serial(args.port, args.baud) as serial_port:
        transmit_loop(serial_port, source, args.rate, max(0.0, args.startup_safe_seconds),
                      args.shutdown_safe_frames)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
