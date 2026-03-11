#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
import random
import queue
import struct
import threading
import time

import serial


TELEM_HEADER = [
    "host_time_iso",
    "mcu_time_ms",
    "rpm1_x10",
    "rpm2_x10",
    "rpm3_x10",
    "rpm4_x10",
    "battery_mV",
    "battery_adc",
    "cmd1",
    "cmd2",
    "cmd3",
    "cmd4",
    "brake",
]
SOF = 0xAA55
FRAME_FMT = "<HHIhhhhhhhhHHBB"
FRAME_SIZE = struct.calcsize(FRAME_FMT)


def parse_args():
    p = argparse.ArgumentParser(description="UART telemetry logger + command console")
    p.add_argument("--port", required=True, help="Serial port (ex: COM7)")
    p.add_argument("--baud", type=int, default=1000000, help="Baud rate")
    p.add_argument("--out", default="telemetry_log.csv", help="CSV output path")
    p.add_argument("--duration", type=float, default=0.0, help="Seconds to run (0 = until Ctrl+C)")
    p.add_argument("--send-ping", action="store_true", help="Send PING at startup")
    p.add_argument("--auto-ramp", action="store_true", help="Generate random positive ramps automatically")
    p.add_argument("--cmd-min", type=int, default=0, help="Minimum command for auto ramp (>=0)")
    p.add_argument("--cmd-max", type=int, default=530, help="Maximum command for auto ramp (RPM)")
    p.add_argument("--ramp-segment-s", type=float, default=0.8, help="Seconds per random ramp segment")
    p.add_argument("--cmd-rate-hz", type=float, default=30.0, help="Command update rate for auto ramp")
    p.add_argument("--seed", type=int, default=0, help="Random seed (0 = time-based)")
    p.add_argument("--text-telemetry", action="store_true", help="Use old text telemetry parser (T,...)")
    return p.parse_args()


def input_thread(cmd_q: queue.Queue):
    print("Console commands: ping | stop | cmd m1 m2 m3 m4 brake | vel vx vy vtheta")
    while True:
        try:
            line = input("> ").strip()
        except EOFError:
            return
        if not line:
            continue
        cmd_q.put(line)


def encode_console_cmd(line: str):
    tokens = line.split()
    if not tokens:
        return None
    head = tokens[0].lower()
    if head == "ping":
        return "PING\r\n"
    if head == "stop":
        return "STOP\r\n"
    if head == "cmd" and len(tokens) == 6:
        return f"CMD,{tokens[1]},{tokens[2]},{tokens[3]},{tokens[4]},{tokens[5]}\r\n"
    if head == "vel" and len(tokens) == 4:
        return f"VEL,{tokens[1]},{tokens[2]},{tokens[3]}\r\n"
    print("Invalid command")
    return None


class RampGenerator:
    def __init__(self, cmd_min: int, cmd_max: int, segment_s: float, cmd_rate_hz: float):
        self.cmd_min = max(0, cmd_min)
        self.cmd_max = max(self.cmd_min, cmd_max)
        self.segment_s = max(0.05, segment_s)
        self.cmd_period_s = 1.0 / max(1.0, cmd_rate_hz)
        self.last_send = 0.0
        self.seg_t0 = 0.0
        self.start = float(self.cmd_min)
        self.target = float(self.cmd_min)

    def start_now(self, now: float):
        self.last_send = 0.0
        self.seg_t0 = now
        self.start = float(self.cmd_min)
        self.target = float(self.cmd_max)

    def _pick_target(self) -> float:
        return float(random.randint(self.cmd_min, self.cmd_max))

    def maybe_next_cmd(self, now: float):
        if (now - self.last_send) < self.cmd_period_s:
            return None

        elapsed = now - self.seg_t0
        if elapsed >= self.segment_s:
            self.start = self.target
            self.target = self._pick_target()
            self.seg_t0 = now
            elapsed = 0.0

        alpha = min(1.0, max(0.0, elapsed / self.segment_s))
        value = int(round(self.start + (self.target - self.start) * alpha))
        value = max(self.cmd_min, min(self.cmd_max, value))
        self.last_send = now
        return f"CMD,{value},{value},{value},{value},0\r\n"


def main():
    args = parse_args()
    cmd_q = queue.Queue()
    if args.seed != 0:
        random.seed(args.seed)

    ramp = None
    if args.auto_ramp:
        ramp = RampGenerator(args.cmd_min, args.cmd_max, args.ramp_segment_s, args.cmd_rate_hz)

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser, open(args.out, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(TELEM_HEADER)

        if not args.auto_ramp:
            t = threading.Thread(target=input_thread, args=(cmd_q,), daemon=True)
            t.start()
        else:
            print("Auto-ramp mode enabled")
            print(f"Range: {max(0, args.cmd_min)}..{max(max(0, args.cmd_min), args.cmd_max)}")
            print(f"Segment: {max(0.05, args.ramp_segment_s):.3f}s, cmd-rate: {max(1.0, args.cmd_rate_hz):.1f} Hz")

        print(f"Connected: {args.port} @ {args.baud}")
        print(f"Logging to: {args.out}")

        if args.send_ping:
            ser.write(b"PING\r\n")

        t0 = time.time()
        rows = 0
        rx_buf = bytearray()
        if ramp is not None:
            ramp.start_now(t0)
        try:
            while True:
                now = time.time()
                if args.duration > 0 and (now - t0) >= args.duration:
                    break

                if ramp is None:
                    while not cmd_q.empty():
                        txt = encode_console_cmd(cmd_q.get())
                        if txt is not None:
                            ser.write(txt.encode("ascii"))
                else:
                    txt = ramp.maybe_next_cmd(now)
                    if txt is not None:
                        ser.write(txt.encode("ascii"))

                if args.text_telemetry:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("ascii", errors="replace").strip()
                    if not line:
                        continue
                    if line.startswith("T,"):
                        parts = line.split(",")
                        if len(parts) == 13:
                            row = [dt.datetime.now().isoformat()] + parts[1:]
                            writer.writerow(row)
                            rows += 1
                            if rows % 50 == 0:
                                f.flush()
                        else:
                            print(f"[WARN] Bad telemetry frame: {line}")
                    else:
                        print(f"[UART] {line}")
                    continue

                chunk = ser.read(ser.in_waiting or 64)
                if not chunk:
                    continue
                rx_buf.extend(chunk)

                while True:
                    idx = rx_buf.find(b"\x55\xAA")
                    if idx < 0:
                        if len(rx_buf) > (FRAME_SIZE * 2):
                            del rx_buf[:-FRAME_SIZE]
                        break
                    if idx > 0:
                        del rx_buf[:idx]
                    if len(rx_buf) < FRAME_SIZE:
                        break

                    frame_bytes = bytes(rx_buf[:FRAME_SIZE])
                    del rx_buf[:FRAME_SIZE]
                    vals = struct.unpack(FRAME_FMT, frame_bytes)
                    (
                        sof,
                        _seq,
                        mcu_time_ms,
                        rpm1_x10,
                        rpm2_x10,
                        rpm3_x10,
                        rpm4_x10,
                        cmd1,
                        cmd2,
                        cmd3,
                        cmd4,
                        battery_mV,
                        battery_adc,
                        brake,
                        crc_rx,
                    ) = vals
                    crc_calc = 0
                    for b in frame_bytes[:-1]:
                        crc_calc ^= b
                    if sof != SOF or crc_calc != crc_rx:
                        continue

                    row = [
                        dt.datetime.now().isoformat(),
                        mcu_time_ms,
                        rpm1_x10,
                        rpm2_x10,
                        rpm3_x10,
                        rpm4_x10,
                        battery_mV,
                        battery_adc,
                        cmd1,
                        cmd2,
                        cmd3,
                        cmd4,
                        brake,
                    ]
                    writer.writerow(row)
                    rows += 1
                    if rows % 200 == 0:
                        f.flush()
        except KeyboardInterrupt:
            pass
        finally:
            ser.write(b"STOP\r\n")
            f.flush()
            print(f"Done. Rows logged: {rows}")


if __name__ == "__main__":
    main()
