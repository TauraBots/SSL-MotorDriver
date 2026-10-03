#!/usr/bin/env python3
"""Binary D1 bridge for the ExpressLRS AirPort Team transport."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import signal
import struct
import time


SOF0 = 0x55
SOF1 = 0xAA
SERIAL_TYPE_TEAM_VELOCITY = 0xD1
TEAM_VELOCITY_VERSION = 0x01
TEAM_FRAME_SIZE = 32

FLAG_KICK = 1 << 0
FLAG_CHIP = 1 << 1
FLAG_BRAKE = 1 << 2
FLAG_DRIBBLER = 1 << 3
FLAG_ENABLED = 1 << 4

DEFAULT_BAUD = 9600
DEFAULT_RATE_HZ = 20.0
STARTUP_SAFE_FRAMES = 10
SHUTDOWN_SAFE_FRAMES = 20


@dataclass(frozen=True)
class RobotCommand:
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    kick_power: int = 0
    kick: bool = False
    chip: bool = False
    brake: bool = False
    dribbler: bool = False
    enabled: bool = False


def crc16(data: bytes) -> int:
    """Match firmware SerialProtocol_Crc16() (CRC16-CCITT-FALSE)."""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _milli_i16(value: float) -> int:
    scaled = int(round(float(value) * 1000.0))
    return max(-32768, min(32767, scaled))


def _flags(command: RobotCommand) -> int:
    flags = 0
    flags |= FLAG_KICK if command.kick else 0
    flags |= FLAG_CHIP if command.chip else 0
    flags |= FLAG_BRAKE if command.brake else 0
    flags |= FLAG_DRIBBLER if command.dribbler else 0
    flags |= FLAG_ENABLED if command.enabled else 0
    return flags


def _pack_robot(command: RobotCommand) -> bytes:
    kick_power = max(0, min(100, int(command.kick_power)))
    return struct.pack(
        "<hhhBB",
        _milli_i16(command.vx),
        _milli_i16(command.vy),
        _milli_i16(command.omega),
        kick_power,
        _flags(command),
    )


def build_team_frame(sequence: int, robot_a: RobotCommand,
                     robot_b: RobotCommand, robot_c: RobotCommand) -> bytes:
    payload = bytearray((SOF0, SOF1, SERIAL_TYPE_TEAM_VELOCITY, TEAM_VELOCITY_VERSION))
    payload.extend(struct.pack("<H", int(sequence) & 0xFFFF))
    payload.extend(_pack_robot(robot_a))
    payload.extend(_pack_robot(robot_b))
    payload.extend(_pack_robot(robot_c))
    payload.extend(struct.pack("<H", crc16(payload)))
    frame = bytes(payload)
    assert len(frame) == TEAM_FRAME_SIZE
    return frame


def next_sequence(sequence: int) -> int:
    return (int(sequence) + 1) & 0xFFFF


def safe_command() -> RobotCommand:
    return RobotCommand(brake=True, enabled=False)


def safe_commands() -> tuple[RobotCommand, RobotCommand, RobotCommand]:
    safe = safe_command()
    return safe, safe, safe


def test_commands() -> tuple[RobotCommand, RobotCommand, RobotCommand]:
    return (
        RobotCommand(vx=1.0, enabled=True),
        RobotCommand(vy=1.0, enabled=True),
        RobotCommand(omega=1.0, enabled=True),
    )


def enabled_stop_commands() -> tuple[RobotCommand, RobotCommand, RobotCommand]:
    enabled = RobotCommand(enabled=True)
    return enabled, enabled, enabled


def frame_hex(frame: bytes) -> str:
    return " ".join(f"{value:02X}" for value in frame)


def open_serial(port: str, baud: int):
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required: python -m pip install pyserial") from exc
    return serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=0,
        write_timeout=1,
    )


def _write_frame(serial_port, frame: bytes) -> None:
    written = serial_port.write(frame)
    if written != len(frame):
        raise IOError(f"short serial write: {written}/{len(frame)} bytes")


def send_safe_frames(serial_port, sequence: int, count: int,
                     period: float = 0.0) -> int:
    commands = safe_commands()
    for index in range(max(0, count)):
        _write_frame(serial_port, build_team_frame(sequence, *commands))
        sequence = next_sequence(sequence)
        if period > 0.0 and index + 1 < count:
            time.sleep(period)
    return sequence


def _describe_command(robot_id: str, command: RobotCommand) -> str:
    return (
        f"{robot_id}: vx={command.vx:+.3f} vy={command.vy:+.3f} "
        f"omega={command.omega:+.3f} kick_power={command.kick_power} "
        f"kick={command.kick} chip={command.chip} brake={command.brake} "
        f"dribbler={command.dribbler} enabled={command.enabled}"
    )


def transmit_loop(serial_port, commands: tuple[RobotCommand, RobotCommand, RobotCommand],
                  rate_hz: float, verbose: bool = False) -> None:
    period = 1.0 / rate_hz
    sequence = 0
    frames_sent = 0
    running = True
    next_report = time.perf_counter() + 1.0

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        next_tick = time.perf_counter()
        while running:
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(min(next_tick - now, period / 2.0))
                continue

            current = safe_commands() if frames_sent < STARTUP_SAFE_FRAMES else commands
            _write_frame(serial_port, build_team_frame(sequence, *current))
            sequence = next_sequence(sequence)
            frames_sent += 1

            if verbose and now >= next_report:
                print(f"frames sent: {frames_sent} | current sequence: {sequence}")
                next_report = now + 1.0

            next_tick += period
            if next_tick < now - period:
                next_tick = now + period
    finally:
        try:
            sequence = send_safe_frames(
                serial_port, sequence, SHUTDOWN_SAFE_FRAMES, period=period
            )
            serial_port.flush()
            if verbose:
                print(
                    f"shutdown safety: {SHUTDOWN_SAFE_FRAMES} disabled frames | "
                    f"next sequence: {sequence}"
                )
        except Exception as exc:
            if verbose:
                print(f"shutdown safety write failed: {exc}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send binary Quad-MD D1 frames through ExpressLRS AirPort."
    )
    parser.add_argument("--port", required=True, help="Serial port connected to the AION Nano TX")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help=f"AirPort serial baud (default: {DEFAULT_BAUD})")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ,
                        help=f"TeamFrame rate in Hz (default: {DEFAULT_RATE_HZ:g})")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--test", action="store_true",
                      help="Move A in +vx, B in +vy and C in +omega")
    mode.add_argument("--disable", action="store_true",
                      help="Keep every robot disabled in safe state")
    mode.add_argument("--enable", action="store_true",
                      help="Enable every robot with zero velocity")
    parser.add_argument("--verbose", action="store_true", help="Print status about once per second")
    parser.add_argument("--show-hex", action="store_true",
                        help="Print one D1 frame in hexadecimal before opening the port")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    if args.baud <= 0:
        raise SystemExit("--baud must be positive")
    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")

    if args.test:
        commands = test_commands()
    elif args.enable:
        commands = enabled_stop_commands()
    else:
        commands = safe_commands()

    if args.verbose:
        print("AirPort Team bridge")
        print(f"Port: {args.port}")
        print(f"Baud: {args.baud}")
        print(f"Rate: {args.rate:g} Hz")
        print(f"Frame size: {TEAM_FRAME_SIZE} bytes")
        print("Transport: binary D1")
        for robot_id, command in zip("ABC", commands):
            print(_describe_command(robot_id, command))
    if args.show_hex:
        print(f"D1 frame: {frame_hex(build_team_frame(0, *commands))}")

    with open_serial(args.port, args.baud) as serial_port:
        transmit_loop(serial_port, commands, args.rate, verbose=args.verbose)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
