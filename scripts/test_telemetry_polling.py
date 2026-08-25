#!/usr/bin/env python3
"""Hardware integration checks for directed telemetry polling.

Run with a single configured robot connected to the serial bus.
"""

import argparse
import importlib.util
import sys
import time
from pathlib import Path

import serial


SCRIPT_DIR = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("ssl_backend", SCRIPT_DIR / "ssl-configurator.py")
BACKEND = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BACKEND)


def receive(ser, robot_id, timeout=0.35):
    deadline = time.monotonic() + timeout
    rx = bytearray()
    while time.monotonic() < deadline:
        rx.extend(ser.read(ser.in_waiting or 1))
        telemetry = BACKEND.parse_telemetry(rx, robot_id)
        if telemetry is not None:
            return telemetry
    return None


def expect_silence(ser, request, label):
    ser.reset_input_buffer()
    ser.write(request)
    ser.flush()
    raw = ser.read(ser.in_waiting or 1)
    deadline = time.monotonic() + 0.25
    while time.monotonic() < deadline:
        raw += ser.read(ser.in_waiting or 1)
    if b"\x55\xAA\xE1" in raw:
        raise AssertionError(f"{label}: resposta E1 inesperada")
    print(f"PASS {label}: silêncio")


def main():
    parser = argparse.ArgumentParser(description="Testa polling direcionado E0/E1 em uma placa")
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--robot-id", default="A")
    parser.add_argument("--other-id", default="Z")
    args = parser.parse_args()
    robot_id = args.robot_id.upper()
    other_id = args.other_id.upper()
    if robot_id == other_id:
        parser.error("--other-id deve ser diferente de --robot-id")

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        time.sleep(0.25)
        ser.reset_input_buffer()

        request = BACKEND.encode_telemetry_request(robot_id, 100)
        ser.write(request); ser.flush()
        reply = receive(ser, robot_id)
        assert reply is not None and reply["request_sequence"] == 100
        print("PASS ID correto: resposta correlacionada")

        time.sleep(0.12)
        expect_silence(ser, BACKEND.encode_telemetry_request(other_id, 101), "ID diferente")

        time.sleep(0.12)
        bad_crc = bytearray(BACKEND.encode_telemetry_request(robot_id, 102))
        bad_crc[-1] ^= 0xFF
        expect_silence(ser, bytes(bad_crc), "CRC inválido")

        time.sleep(0.12)
        expect_silence(ser, BACKEND.encode_telemetry_request("*", 103), "broadcast")

        sequence = int(time.time() * 1000) & 0xFFFFFFFF
        ser.write(BACKEND.encode_robot_velocity_packet(robot_id, sequence, 0.0, 0.0, 0.0, brake=1))
        ser.flush(); time.sleep(0.12); ser.reset_input_buffer()
        ser.write(BACKEND.encode_telemetry_request(robot_id, 104)); ser.flush()
        active = receive(ser, robot_id)
        assert active is not None and active.get("watchdog_ok") == 1

        time.sleep(0.25)
        ser.reset_input_buffer()
        ser.write(BACKEND.encode_telemetry_request(robot_id, 105)); ser.flush()
        expired = receive(ser, robot_id)
        assert expired is not None and expired.get("watchdog_ok") == 0
        print("PASS watchdog: polling não renovou comandos")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AssertionError as exc:
        print(f"FAIL {exc}", file=sys.stderr)
        sys.exit(1)
