#!/usr/bin/env python3
import argparse
import csv
import datetime as dt
import math
import os
import struct
import sys
import threading
import time

import serial
from serial.tools import list_ports


RX_SOF0 = 0x55
RX_SOF1 = 0xAA
ROBOT_VELOCITY_TYPE = 0xD0
ROBOT_VELOCITY_VERSION = 1
TELEMETRY_REQUEST_TYPE = 0xE0
TELEMETRY_RESPONSE_TYPE = 0xE1
TELEMETRY_VERSION = 1
TELEMETRY_FLAG_BASIC = 1 << 0
TELEMETRY_FLAG_MOTORS = 1 << 1
TELEMETRY_FLAG_BATTERY = 1 << 2
TELEMETRY_FLAG_DIAGNOSTICS = 1 << 3
TELEMETRY_FLAGS_FULL = 0x0F
TELEMETRY_REPLY_WINDOW_S = 0.080
TELEMETRY_RESPONSE_BASE_SIZE = 11
CONFIG_DISCOVER_TYPE = 0xF0
CONFIG_SET_ID_TYPE = 0xF1
CONFIG_DISCOVER_RESPONSE_TYPE = 0xF2
CONFIG_SET_ID_RESPONSE_TYPE = 0xF3
CONFIG_SET_MOTION_TYPE = 0xF4
CONFIG_SET_MOTION_RESPONSE_TYPE = 0xF5
CONFIG_KEY = 0x46434449
CONFIG_RESPONSE_FMT = "<HB12sBBIH"
CONFIG_RESPONSE_SIZE = struct.calcsize(CONFIG_RESPONSE_FMT)
CSV_HEADER = [
    "host_time_iso", "robot_id", "request_sequence", "flags", "status",
    "mcu_time_ms", "rpm1_x10", "rpm2_x10", "rpm3_x10", "rpm4_x10",
    "battery_mV", "battery_adc", "cmd1", "cmd2", "cmd3", "cmd4",
    "brake", "command_sequence", "communication_ok", "kick_power",
]


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def packet_with_crc(payload: bytes) -> bytes:
    return payload + struct.pack("<H", crc16_ccitt_false(payload))


def parse_uid(text: str) -> bytes:
    compact = text.replace(":", "").replace("-", "").replace(" ", "")
    if len(compact) != 24:
        raise ValueError("UID must contain exactly 24 hexadecimal characters")
    return bytes.fromhex(compact)


def read_config_responses(ser: serial.Serial, duration_s: float):
    deadline = time.monotonic() + duration_s
    rx = bytearray()
    responses = []
    received_bytes = 0
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        received_bytes += len(chunk)
        rx.extend(chunk)
        start = rx.find(b"\x55\xAA")
        while start >= 0 and len(rx) >= start + CONFIG_RESPONSE_SIZE:
            if start:
                del rx[:start]
            frame = bytes(rx[:CONFIG_RESPONSE_SIZE])
            del rx[:CONFIG_RESPONSE_SIZE]
            values = struct.unpack(CONFIG_RESPONSE_FMT, frame)
            if (crc16_ccitt_false(frame[:-2]) == values[-1] and
                    values[1] in (CONFIG_DISCOVER_RESPONSE_TYPE, CONFIG_SET_ID_RESPONSE_TYPE,
                                  CONFIG_SET_MOTION_RESPONSE_TYPE)):
                responses.append(values)
            start = rx.find(b"\x55\xAA")
    read_config_responses.last_rx_bytes = received_bytes
    return responses


read_config_responses.last_rx_bytes = 0


def print_board(values):
    _, response_type, uid, status, robot_id, generation, _ = values
    id_text = chr(robot_id) if ord("A") <= robot_id <= ord("Z") else "UNCONFIGURED"
    if response_type == CONFIG_DISCOVER_RESPONSE_TYPE:
        print(f"UID={uid.hex().upper()} ID={id_text} configured={'YES' if status else 'NO'} generation={generation}")
    else:
        print(f"UID={uid.hex().upper()} ID={id_text} generation={generation} status={'OK' if status else 'ERROR'}")


def run_config_action(args):
    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        time.sleep(0.25)
        ser.reset_input_buffer()
        if args.action == "discover":
            payload = struct.pack("<HBI", 0xAA55, CONFIG_DISCOVER_TYPE, int(time.time() * 1000) & 0xFFFFFFFF)
            ser.write(packet_with_crc(payload))
            ser.flush()
            responses = read_config_responses(ser, 1.5)
            seen = set()
            for values in responses:
                if values[1] == CONFIG_DISCOVER_RESPONSE_TYPE and values[2] not in seen:
                    print_board(values)
                    seen.add(values[2])
            if not seen:
                raise RuntimeError("no boards responded")
            return 0

        if args.action == "set-motion":
            uid = parse_uid(args.uid)
            frame = encode_motion_config_packet(uid, args.linear_accel, args.angular_accel)
            ser.write(frame)
            ser.flush()
            for values in read_config_responses(ser, 1.0):
                if values[1] == CONFIG_SET_MOTION_RESPONSE_TYPE and values[2] == uid:
                    print_board(values)
                    if values[3] == 0:
                        raise RuntimeError("board rejected motion configuration")
                    return 0
            raise RuntimeError("target board did not acknowledge motion configuration")

        robot_id = args.robot_id.upper()
        if len(robot_id) != 1 or not ("A" <= robot_id <= "Z"):
            raise ValueError("--robot-id must be one letter A-Z")
        uid = parse_uid(args.uid)
        payload = struct.pack("<HB12sBI", 0xAA55, CONFIG_SET_ID_TYPE, uid, ord(robot_id), CONFIG_KEY)
        ser.write(packet_with_crc(payload))
        ser.flush()
        for values in read_config_responses(ser, 1.0):
            if values[1] == CONFIG_SET_ID_RESPONSE_TYPE and values[2] == uid:
                print_board(values)
                if values[3] == 0:
                    raise RuntimeError("board rejected configuration; reset it before setting the ID")
                return 0
        raise RuntimeError("target board did not acknowledge configuration")


def encode_motion_config_packet(uid: bytes, linear_accel: float,
                                angular_accel: float) -> bytes:
    if len(uid) != 12:
        raise ValueError("UID must contain exactly 12 bytes")
    if not math.isfinite(linear_accel) or not 0.1 <= linear_accel <= 20.0:
        raise ValueError("linear acceleration must be between 0.1 and 20.0 m/s²")
    if not math.isfinite(angular_accel) or not 0.1 <= angular_accel <= 50.0:
        raise ValueError("angular acceleration must be between 0.1 and 50.0 rad/s²")
    payload = struct.pack(
        "<HB12sffI", 0xAA55, CONFIG_SET_MOTION_TYPE, uid,
        linear_accel, angular_accel, CONFIG_KEY,
    )
    return packet_with_crc(payload)


def encode_robot_velocity_packet(robot_id: str, sequence: int, vx: float, vy: float,
                                 omega: float, kick_power: int = 0, brake: int = 0) -> bytes:
    kick_power = max(0, min(100, kick_power))
    values = tuple(max(-32768, min(32767, int(round(value * 1000.0))))
                   for value in (vx, vy, omega))
    payload = struct.pack(
        "<HBBBIhhhBB", 0xAA55, ROBOT_VELOCITY_TYPE,
        ROBOT_VELOCITY_VERSION, ord(robot_id), sequence & 0xFFFFFFFF,
        *values, kick_power, 1 if brake else 0,
    )
    return payload + struct.pack("<H", crc16_ccitt_false(payload))


def encode_telemetry_request(robot_id: str, request_sequence: int,
                             flags: int = TELEMETRY_FLAGS_FULL) -> bytes:
    payload = struct.pack(
        "<HBBBHB", 0xAA55, TELEMETRY_REQUEST_TYPE, TELEMETRY_VERSION,
        ord(robot_id), request_sequence & 0xFFFF, flags & TELEMETRY_FLAGS_FULL
    )
    return payload + struct.pack("<H", crc16_ccitt_false(payload))


def parse_telemetry(rx_buffer: bytearray, robot_id: str):
    latest = None
    marker = bytes((0x55, 0xAA, TELEMETRY_RESPONSE_TYPE))
    while True:
        start = rx_buffer.find(marker)
        if start < 0:
            if len(rx_buffer) > TELEMETRY_RESPONSE_BASE_SIZE:
                del rx_buffer[:-2]
            break
        if start > 0:
            del rx_buffer[:start]
        if len(rx_buffer) < 9:
            break
        flags = rx_buffer[8] & TELEMETRY_FLAGS_FULL
        frame_size = TELEMETRY_RESPONSE_BASE_SIZE
        frame_size += 7 if flags & TELEMETRY_FLAG_BASIC else 0
        frame_size += 16 if flags & TELEMETRY_FLAG_MOTORS else 0
        frame_size += 4 if flags & TELEMETRY_FLAG_BATTERY else 0
        frame_size += 13 if flags & TELEMETRY_FLAG_DIAGNOSTICS else 0
        if len(rx_buffer) < frame_size:
            break
        frame = bytes(rx_buffer[:frame_size])
        crc_rx = struct.unpack_from("<H", frame, frame_size - 2)[0]
        if crc16_ccitt_false(frame[:-2]) != crc_rx:
            del rx_buffer[0]
            continue
        del rx_buffer[:frame_size]
        if frame[3] != TELEMETRY_VERSION or frame[4] != ord(robot_id):
            continue
        request_sequence = struct.unpack_from("<H", frame, 5)[0]
        status = frame[7]
        offset = 9
        latest = {
            "robot_id": chr(frame[4]),
            "flags": flags,
            "status": status,
            "fault_status": status >> 1,
            "request_sequence": request_sequence,
            "received_at": time.monotonic(),
        }
        if flags & TELEMETRY_FLAG_BASIC:
            latest["time_ms"] = struct.unpack_from("<I", frame, offset)[0]
            offset += 4
            latest["comm_ok"], latest["brake"], latest["kick_power"] = frame[offset:offset + 3]
            offset += 3
        if flags & TELEMETRY_FLAG_MOTORS:
            motor_values = struct.unpack_from("<hhhhhhhh", frame, offset)
            latest["rpm"] = tuple(value / 10.0 for value in motor_values[:4])
            latest["cmd"] = tuple(motor_values[4:])
            offset += 16
        if flags & TELEMETRY_FLAG_BATTERY:
            battery_mv, latest["battery_adc"] = struct.unpack_from("<HH", frame, offset)
            latest["battery_v"] = battery_mv / 1000.0
            offset += 4
        if flags & TELEMETRY_FLAG_DIAGNOSTICS:
            latest["crc_errors"], latest["received_packets"] = struct.unpack_from("<II", frame, offset)
            offset += 8
            latest["watchdog_ok"] = frame[offset]
            offset += 1
            latest["command_sequence"] = struct.unpack_from("<I", frame, offset)[0]
    return latest


def parse_args():
    argv = sys.argv[1:]
    if not argv:
        argv.append("app")
    elif argv[0] not in ("app", "drive", "discover", "set-id", "set-motion", "-h", "--help"):
        argv.insert(0, "drive")

    parser = argparse.ArgumentParser(description="TauraBots motor-driver application")
    actions = parser.add_subparsers(dest="action", required=True)

    app = actions.add_parser("app", help="Open the graphical configurator")
    app.add_argument("--port", help="Initial serial port")
    app.add_argument("--baud", type=int, default=9600)

    drive = actions.add_parser("drive", help="Drive one robot with live telemetry")
    drive.add_argument("--port", required=True, help="Serial port, for example COM3")
    drive.add_argument("--baud", type=int, default=9600, help="Baud rate")
    drive.add_argument("--rate-hz", type=float, default=20.0, help="Command send rate")
    drive.add_argument("--telemetry-hz", type=float, default=5.0, help="Telemetry polling rate, 0 disables")
    drive.add_argument("--robot-id", default="A", help="Destination ID A-Z, or * without telemetry")
    drive.add_argument("--kick-power", type=int, default=80, help="Kick power for K, 0-100")
    drive.add_argument("--linear", type=float, default=2.0, help="Linear velocity in m/s")
    drive.add_argument("--angular", type=float, default=5.0, help="Angular velocity in rad/s")
    drive.add_argument("--accel-tau", type=float, default=0.18, help="Acceleration time constant")
    drive.add_argument("--stop-tau", type=float, default=0.08, help="Stop time constant")
    drive.add_argument("--log", help="Optional telemetry CSV output")

    discover = actions.add_parser("discover", help="Discover boards by immutable STM32 UID")
    discover.add_argument("--port", required=True)
    discover.add_argument("--baud", type=int, default=9600)

    set_id = actions.add_parser("set-id", help="Assign a logical ID to one UID")
    set_id.add_argument("--port", required=True)
    set_id.add_argument("--baud", type=int, default=9600)
    set_id.add_argument("--uid", required=True)
    set_id.add_argument("--robot-id", required=True)

    set_motion = actions.add_parser("set-motion", help="Configure persistent acceleration limits")
    set_motion.add_argument("--port", required=True)
    set_motion.add_argument("--baud", type=int, default=9600)
    set_motion.add_argument("--uid", required=True)
    set_motion.add_argument("--linear-accel", type=float, required=True)
    set_motion.add_argument("--angular-accel", type=float, required=True)
    return parser.parse_args(argv)


def draw(screen, font, vx: float, vy: float, omega: float, port: str, baud: int,
         robot_id: str, kick_power: int, telemetry):
    screen.fill((20, 24, 28))
    lines = [
        "TauraBots Control",
        f"{port} @ {baud}",
        f"Robot {robot_id} | kick {kick_power}%",
        "",
        "W/S: forward/back",
        "A/D: left/right",
        "Q/E: rotate",
        "Space: stop",
        "K: kick (one packet)",
        "Esc or X: quit",
        "",
        f"vx    {vx:+.2f} m/s",
        f"vy    {vy:+.2f} m/s",
        f"omega {omega:+.2f} rad/s",
    ]
    if telemetry is None:
        lines.extend(["", "Telemetry: waiting..."])
    else:
        age_ms = int((time.monotonic() - telemetry["received_at"]) * 1000.0)
        rpm = telemetry["rpm"]
        lines.extend([
            "",
            f"Telemetry: {'OK' if telemetry['comm_ok'] else 'LOST'} age={age_ms}ms",
            f"RPM {rpm[0]:+.0f} {rpm[1]:+.0f} {rpm[2]:+.0f} {rpm[3]:+.0f}",
            f"Battery {telemetry['battery_v']:.2f}V brake={telemetry['brake']}",
            f"cmd_seq={telemetry['command_sequence']} kick={telemetry['kick_power']}%",
        ])

    y = 18
    for i, line in enumerate(lines):
        color = (235, 238, 240) if i in (0, 1, 2, 11, 12, 13) else (165, 172, 178)
        text = font.render(line, True, color)
        screen.blit(text, (20, y))
        y += 28
    pygame.display.flip()


def telemetry_csv_row(telemetry):
    rpm_x10 = [int(round(value * 10.0)) for value in telemetry["rpm"]]
    return [
        dt.datetime.now().isoformat(), telemetry["robot_id"], telemetry["request_sequence"],
        telemetry["flags"], telemetry["status"], telemetry["time_ms"], *rpm_x10,
        int(round(telemetry["battery_v"] * 1000.0)), telemetry["battery_adc"],
        *telemetry["cmd"], telemetry["brake"], telemetry["command_sequence"],
        telemetry["comm_ok"], telemetry["kick_power"],
    ]


class ConfiguratorApp:
    BG = "#0b1220"
    PANEL = "#121c2e"
    PANEL_2 = "#18253a"
    TEXT = "#edf4ff"
    MUTED = "#8fa3bd"
    CYAN = "#20d3ee"
    GREEN = "#42d392"
    RED = "#ff6577"

    def __init__(self, args):
        self.root = tk.Tk()
        self.root.title("SSL Configurator · TauraBots")
        self.root.geometry("1040x700")
        self.root.minsize(900, 620)
        self.root.configure(bg=self.BG)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.ser = None
        self.connected_robot_id = None
        self.running = False
        self.keys = set()
        self.sequence = int(time.time() * 1000) & 0xFFFFFFFF
        self.request_sequence = 0
        self.next_command = 0.0
        self.next_telemetry = 0.0
        self.rx = bytearray()
        self.kick_pending = False
        self.last_motion = time.monotonic()
        self.last_telemetry_received = 0.0
        self.vx = self.vy = self.omega = 0.0
        self.log_file = None
        self.log_writer = None
        self.config_busy = False
        self.discovered_boards = []

        self.port_var = tk.StringVar(value=args.port or "")
        self.baud_var = tk.StringVar(value=str(args.baud))
        self.robot_var = tk.StringVar(value="A")
        self.kick_var = tk.IntVar(value=80)
        self.telemetry_hz_var = tk.DoubleVar(value=5.0)
        self.status_var = tk.StringVar(value="DESCONECTADO")
        self.comm_var = tk.StringVar(value="—")
        self.seq_var = tk.StringVar(value="—")
        self.battery_var = tk.StringVar(value="—")
        self.rpm_var = [tk.StringVar(value="0") for _ in range(4)]
        self.uid_var = tk.StringVar()
        self.new_id_var = tk.StringVar(value="A")
        self.log_var = tk.StringVar(value="CSV desligado")
        self.analysis_csv_var = tk.StringVar()
        self.analysis_status_var = tk.StringVar(value="Selecione um arquivo gravado pelo configurador.")
        self.discovery_summary_var = tk.StringVar(value="Nenhuma descoberta executada")
        self._style()
        self._build()
        self.refresh_ports()
        # bind_all keeps driving keys working even after a button or tab receives focus.
        self.root.bind_all("<KeyPress>", self.key_down, add="+")
        self.root.bind_all("<KeyRelease>", self.key_up, add="+")

    def _style(self):
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure("TNotebook", background=self.BG, borderwidth=0)
        style.configure("TNotebook.Tab", background=self.PANEL, foreground=self.MUTED,
                        padding=(24, 12), borderwidth=0, font=("Segoe UI", 10, "bold"))
        style.map("TNotebook.Tab", background=[("selected", self.PANEL_2)],
                  foreground=[("selected", self.CYAN)])
        style.configure("TCombobox", fieldbackground=self.PANEL_2, background=self.PANEL_2,
                        foreground=self.TEXT, arrowcolor=self.CYAN)

    def label(self, parent, text=None, variable=None, size=10, color=None, bold=False):
        return tk.Label(parent, text=text, textvariable=variable, bg=parent.cget("bg"),
                        fg=color or self.TEXT, font=("Segoe UI", size, "bold" if bold else "normal"))

    def button(self, parent, text, command, accent=False):
        return tk.Button(parent, text=text, command=command, relief="flat", bd=0,
                         bg=self.CYAN if accent else self.PANEL_2,
                         fg="#051018" if accent else self.TEXT, activebackground=self.GREEN,
                         font=("Segoe UI", 10, "bold"), padx=18, pady=9, cursor="hand2")

    def card(self, parent):
        return tk.Frame(parent, bg=self.PANEL, highlightthickness=1,
                        highlightbackground="#263752", padx=20, pady=18)

    def _build(self):
        header = tk.Frame(self.root, bg=self.BG, padx=28, pady=20)
        header.pack(fill="x")
        self.label(header, "SSL CONFIGURATOR", size=20, color=self.CYAN, bold=True).pack(side="left")
        self.label(header, "TauraBots · Motor Driver", size=11, color=self.MUTED).pack(side="left", padx=16)
        self.status_badge = self.label(header, variable=self.status_var, size=10, color=self.RED, bold=True)
        self.status_badge.pack(side="right")

        connection = self.card(self.root)
        connection.pack(fill="x", padx=28, pady=(0, 16))
        self.label(connection, "Porta", color=self.MUTED).pack(side="left")
        self.port_box = ttk.Combobox(connection, textvariable=self.port_var, width=17)
        self.port_box.pack(side="left", padx=(8, 18))
        self.label(connection, "Baud", color=self.MUTED).pack(side="left")
        ttk.Combobox(connection, textvariable=self.baud_var,
                     values=("9600", "14400", "19200", "38400", "115200",
                             "921600", "1000000"),
                     width=10).pack(side="left", padx=8)
        self.button(connection, "↻", self.refresh_ports).pack(side="left", padx=6)
        self.connect_button = self.button(connection, "CONECTAR", self.toggle_connection, accent=True)
        self.connect_button.pack(side="right")

        tabs = ttk.Notebook(self.root)
        tabs.pack(fill="both", expand=True, padx=28, pady=(0, 28))
        self.control_tab = tk.Frame(tabs, bg=self.BG, padx=2, pady=18)
        self.config_tab = tk.Frame(tabs, bg=self.BG, padx=2, pady=18)
        self.analysis_tab = tk.Frame(tabs, bg=self.BG, padx=2, pady=18)
        tabs.add(self.control_tab, text="CONTROLE E TELEMETRIA")
        tabs.add(self.config_tab, text="CONFIGURAÇÃO DA PLACA")
        tabs.add(self.analysis_tab, text="ANÁLISE CSV")
        self._build_control()
        self._build_config()
        self._build_analysis()

    def _build_control(self):
        left = self.card(self.control_tab)
        left.pack(side="left", fill="both", expand=True, padx=(0, 8))
        right = self.card(self.control_tab)
        right.pack(side="left", fill="both", expand=True, padx=(8, 0))
        self.label(left, "CONTROLE", size=13, bold=True).pack(anchor="w")
        row = tk.Frame(left, bg=self.PANEL)
        row.pack(fill="x", pady=16)
        self.label(row, "Robô", color=self.MUTED).pack(side="left")
        self.robot_box = ttk.Combobox(row, textvariable=self.robot_var,
                                      values=tuple("ABCDEFGHIJKLMNOPQRSTUVWXYZ"), width=5,
                                      state="readonly", takefocus=False)
        self.robot_box.pack(side="left", padx=10)
        self.label(row, "Kick %", color=self.MUTED).pack(side="left", padx=(20, 0))
        tk.Spinbox(row, from_=0, to=100, textvariable=self.kick_var, width=5,
                   bg=self.PANEL_2, fg=self.TEXT, buttonbackground=self.PANEL_2,
                   relief="flat", state="readonly", takefocus=False).pack(side="left", padx=10)
        pad = tk.Frame(left, bg=self.PANEL)
        pad.pack(pady=12)
        self.key_labels = {}
        for key, text, r, c in (("q", "Q ↺", 0, 0), ("w", "W ↑", 0, 1), ("e", "E ↻", 0, 2),
                                ("a", "A ←", 1, 0), ("s", "S ↓", 1, 1), ("d", "D →", 1, 2)):
            label = tk.Label(pad, text=text, width=7, height=2, bg=self.PANEL_2, fg=self.TEXT,
                             font=("Segoe UI", 11, "bold"))
            label.grid(row=r, column=c, padx=5, pady=5)
            self.key_labels[key] = label
        self.label(left, "SPACE freia · K envia um chute", color=self.MUTED).pack(pady=10)
        self.button(left, "CHUTAR", self.queue_kick, accent=True).pack(fill="x", pady=(12, 5))

        self.label(right, "TELEMETRIA AO VIVO", size=13, bold=True).pack(anchor="w")
        summary = tk.Frame(right, bg=self.PANEL)
        summary.pack(fill="x", pady=15)
        for title, var in (("COM", self.comm_var), ("SEQ", self.seq_var), ("BATERIA", self.battery_var)):
            cell = tk.Frame(summary, bg=self.PANEL_2, padx=12, pady=10)
            cell.pack(side="left", fill="x", expand=True, padx=3)
            self.label(cell, title, size=8, color=self.MUTED, bold=True).pack()
            self.label(cell, variable=var, size=12, bold=True).pack()
        for index, var in enumerate(self.rpm_var, 1):
            row = tk.Frame(right, bg=self.PANEL)
            row.pack(fill="x", pady=5)
            self.label(row, f"Motor {index}", color=self.MUTED).pack(side="left")
            self.label(row, variable=var, size=12, bold=True).pack(side="right")
            self.label(row, "RPM", color=self.MUTED).pack(side="right", padx=7)
        self.button(right, "GRAVAR CSV", self.toggle_log).pack(fill="x", pady=(18, 5))
        self.label(right, variable=self.log_var, color=self.MUTED).pack()

    def _build_config(self):
        panel = self.card(self.config_tab)
        panel.pack(fill="both", expand=True)
        self.label(panel, "IDENTIDADE DA PLACA", size=13, bold=True).pack(anchor="w")
        self.label(panel, "Descubra o UID físico e associe um ID lógico A–Z.", color=self.MUTED).pack(anchor="w", pady=(4, 16))
        self.discover_button = self.button(panel, "DESCOBRIR PLACAS", self.discover, accent=True)
        self.discover_button.pack(anchor="w")
        self.label(panel, variable=self.discovery_summary_var, color=self.MUTED).pack(anchor="w", pady=(10, 0))
        self.board_list = tk.Listbox(panel, height=8, bg=self.PANEL_2, fg=self.TEXT,
                                     selectbackground=self.CYAN, selectforeground="#061018",
                                     relief="flat", bd=0, font=("Consolas", 10))
        self.board_list.pack(fill="x", pady=16)
        self.board_list.bind("<<ListboxSelect>>", self.select_board)
        row = tk.Frame(panel, bg=self.PANEL)
        row.pack(fill="x")
        self.label(row, "UID", color=self.MUTED).pack(side="left")
        tk.Entry(row, textvariable=self.uid_var, bg=self.PANEL_2, fg=self.TEXT,
                 insertbackground=self.TEXT, relief="flat", width=34).pack(side="left", padx=10, ipady=7)
        self.label(row, "Novo ID", color=self.MUTED).pack(side="left", padx=(18, 0))
        ttk.Combobox(row, textvariable=self.new_id_var, values=tuple("ABCDEFGHIJKLMNOPQRSTUVWXYZ"),
                     width=5, state="readonly").pack(side="left", padx=10)
        self.set_id_button = self.button(row, "SALVAR ID", self.set_id)
        self.set_id_button.pack(side="right")

    def _build_analysis(self):
        panel = self.card(self.analysis_tab)
        panel.pack(fill="both", expand=True)
        self.label(panel, "ANÁLISE DE TELEMETRIA", size=13, bold=True).pack(anchor="w")
        self.label(panel, "Gere gráficos de RPM, comandos e bateria a partir de um CSV.",
                   color=self.MUTED).pack(anchor="w", pady=(4, 22))
        row = tk.Frame(panel, bg=self.PANEL)
        row.pack(fill="x")
        tk.Entry(row, textvariable=self.analysis_csv_var, bg=self.PANEL_2, fg=self.TEXT,
                 insertbackground=self.TEXT, relief="flat").pack(side="left", fill="x", expand=True, ipady=9)
        self.button(row, "ESCOLHER CSV", self.choose_analysis_csv).pack(side="left", padx=(10, 0))
        self.button(panel, "GERAR GRÁFICOS", self.generate_plots, accent=True).pack(anchor="w", pady=20)
        self.label(panel, variable=self.analysis_status_var, color=self.MUTED).pack(anchor="w")
        self.label(panel, "Saídas: rpm.png · cmd.png · battery.png · rpm_vs_cmd_m1.png",
                   color=self.MUTED).pack(anchor="w", pady=8)

    def refresh_ports(self):
        ports = [item.device for item in list_ports.comports()]
        self.port_box["values"] = ports
        if not self.port_var.get() and ports:
            self.port_var.set(ports[0])

    def serial_settings(self):
        if not self.port_var.get():
            raise ValueError("Selecione uma porta serial")
        return self.port_var.get(), int(self.baud_var.get())

    def toggle_connection(self):
        if self.ser:
            self.disconnect()
            return
        try:
            robot_id = self.robot_var.get().upper()
            if len(robot_id) != 1 or not ("A" <= robot_id <= "Z"):
                raise ValueError("Escolha um ID de robô entre A e Z")
            self.robot_var.set(robot_id)
            port, baud = self.serial_settings()
            self.ser = serial.Serial(port, baud, timeout=0)
            self.ser.reset_input_buffer()
            self.connected_robot_id = robot_id
            self.rx.clear()
            self.request_sequence = 0
            now = time.monotonic()
            self.next_command = now
            self.next_telemetry = now + 0.10
            self.last_motion = now
            self.vx = self.vy = self.omega = 0.0
            self.last_telemetry_received = 0.0
            self.comm_var.set("AGUARDANDO")
            self.seq_var.set("—")
            self.battery_var.set("—")
            self.running = True
            self.status_var.set("CONECTADO")
            self.status_badge.configure(fg=self.GREEN)
            self.connect_button.configure(text="DESCONECTAR", bg=self.PANEL_2, fg=self.TEXT)
            self.robot_box.configure(state="disabled")
            self.keys.clear()
            self.root.focus_set()
            self.root.after(0, self.io_tick)
        except Exception as exc:
            messagebox.showerror("Conexão", str(exc))

    def disconnect(self):
        self.running = False
        if self.ser:
            try:
                self.sequence = (self.sequence + 1) & 0xFFFFFFFF
                if self.connected_robot_id:
                    self.ser.write(encode_robot_velocity_packet(
                        self.connected_robot_id, self.sequence, 0.0, 0.0, 0.0, brake=1))
                self.ser.close()
            except Exception:
                try:
                    self.ser.close()
                except Exception:
                    pass
        self.ser = None
        self.connected_robot_id = None
        self.keys.clear()
        for label in self.key_labels.values():
            label.configure(bg=self.PANEL_2, fg=self.TEXT)
        self.status_var.set("DESCONECTADO")
        self.status_badge.configure(fg=self.RED)
        self.connect_button.configure(text="CONECTAR", bg=self.CYAN, fg="#051018")
        self.robot_box.configure(state="readonly")

    def key_down(self, event):
        key = event.keysym.lower()
        if key in ("w", "a", "s", "d", "q", "e", "space"):
            self.keys.add(key)
            if key in self.key_labels:
                self.key_labels[key].configure(bg=self.CYAN, fg="#051018")
        if key == "k":
            self.queue_kick()

    def key_up(self, event):
        key = event.keysym.lower()
        self.keys.discard(key)
        if key in self.key_labels:
            self.key_labels[key].configure(bg=self.PANEL_2, fg=self.TEXT)

    def queue_kick(self):
        self.kick_pending = True

    def motion_command(self, now):
        vx = (2.0 if "d" in self.keys else 0.0) - (2.0 if "a" in self.keys else 0.0)
        vy = (2.0 if "w" in self.keys else 0.0) - (2.0 if "s" in self.keys else 0.0)
        omega = (5.0 if "q" in self.keys else 0.0) - (5.0 if "e" in self.keys else 0.0)
        if "space" in self.keys:
            vx = vy = omega = 0.0
        elapsed = max(0.0, now - self.last_motion)
        self.last_motion = now
        tau = 0.08 if (vx == vy == omega == 0.0) else 0.18
        alpha = 1.0 - math.exp(-elapsed / tau)
        self.vx += (vx - self.vx) * alpha
        self.vy += (vy - self.vy) * alpha
        self.omega += (omega - self.omega) * alpha
        return self.vx, self.vy, self.omega

    def io_tick(self):
        if not self.running or not self.ser:
            return
        try:
            now = time.monotonic()
            robot_id = self.connected_robot_id
            if robot_id is None:
                raise RuntimeError("ID do robô não está definido para esta conexão")
            if now >= self.next_command:
                vx, vy, omega = self.motion_command(now)
                self.sequence = (self.sequence + 1) & 0xFFFFFFFF
                kick = max(0, min(100, self.kick_var.get())) if self.kick_pending else 0
                stopped = vx == vy == omega == 0.0
                self.ser.write(encode_robot_velocity_packet(
                    robot_id, self.sequence, vx, vy, omega, kick, stopped))
                self.kick_pending = False
                self.next_command = now + 0.05
            hz = max(0.0, self.telemetry_hz_var.get())
            if hz and now >= self.next_telemetry:
                self.request_sequence = (self.request_sequence + 1) & 0xFFFF
                self.ser.write(encode_telemetry_request(robot_id, self.request_sequence))
                self.next_telemetry = now + 1.0 / hz
                self.next_command = max(self.next_command, now + TELEMETRY_REPLY_WINDOW_S)
            if self.ser.in_waiting:
                self.rx.extend(self.ser.read(self.ser.in_waiting))
                telemetry = parse_telemetry(self.rx, robot_id)
                if telemetry:
                    self.update_telemetry(telemetry)
            if (self.last_telemetry_received != 0.0 and
                    (now - self.last_telemetry_received) > 1.0):
                self.comm_var.set("SEM DADOS")
            self.root.after(10, self.io_tick)
        except Exception as exc:
            self.disconnect()
            messagebox.showerror("Comunicação", str(exc))

    def update_telemetry(self, telemetry):
        self.last_telemetry_received = time.monotonic()
        self.comm_var.set("OK" if telemetry["comm_ok"] else "LOST")
        self.seq_var.set(str(telemetry["command_sequence"]))
        self.battery_var.set(f"{telemetry['battery_v']:.2f} V")
        for var, rpm in zip(self.rpm_var, telemetry["rpm"]):
            var.set(f"{rpm:+.0f}")
        if self.log_writer:
            self.log_writer.writerow(telemetry_csv_row(telemetry))
            self.log_file.flush()

    def toggle_log(self):
        if self.log_file:
            self.log_file.close()
            self.log_file = self.log_writer = None
            self.log_var.set("CSV desligado")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=(("CSV", "*.csv"),))
        if path:
            self.log_file = open(path, "w", newline="")
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow(CSV_HEADER)
            self.log_var.set(path)

    def choose_analysis_csv(self):
        path = filedialog.askopenfilename(filetypes=(("CSV", "*.csv"), ("Todos", "*.*")))
        if path:
            self.analysis_csv_var.set(path)

    def generate_plots(self):
        path = self.analysis_csv_var.get().strip()
        if not path:
            messagebox.showerror("Análise", "Selecione um arquivo CSV")
            return
        self.analysis_status_var.set("Gerando gráficos…")
        threading.Thread(target=self.plot_worker, args=(path,), daemon=True).start()

    def plot_worker(self, path):
        try:
            from pathlib import Path
            import matplotlib.pyplot as plt
            import pandas as pd
            csv_path = Path(path)
            output = csv_path.parent / f"{csv_path.stem}_plots"
            output.mkdir(parents=True, exist_ok=True)
            frame = pd.read_csv(csv_path)
            if frame.empty:
                raise ValueError("O CSV está vazio")
            frame["mcu_time_ms"] = pd.to_numeric(frame["mcu_time_ms"], errors="coerce")
            frame = frame.dropna(subset=["mcu_time_ms"]).copy()
            if frame.empty:
                raise ValueError("O CSV não contém amostras válidas")
            timeline = (frame["mcu_time_ms"] - frame["mcu_time_ms"].iloc[0]) / 1000.0
            for index in range(1, 5):
                frame[f"rpm{index}"] = pd.to_numeric(frame[f"rpm{index}_x10"], errors="coerce") / 10.0
                frame[f"cmd{index}"] = pd.to_numeric(frame[f"cmd{index}"], errors="coerce")
            frame["battery_v"] = pd.to_numeric(frame["battery_mV"], errors="coerce") / 1000.0

            def save(name, title, ylabel, columns):
                plt.figure(figsize=(12, 5))
                for column in columns:
                    plt.plot(timeline, frame[column], label=column, linewidth=1.2)
                plt.title(title)
                plt.xlabel("Tempo [s]")
                plt.ylabel(ylabel)
                plt.grid(True, alpha=0.3)
                plt.legend(loc="best")
                plt.tight_layout()
                plt.savefig(output / name, dpi=140)
                plt.close()

            save("rpm.png", "Velocidade dos motores", "RPM", [f"rpm{i}" for i in range(1, 5)])
            save("cmd.png", "Comandos dos motores", "Comando", [f"cmd{i}" for i in range(1, 5)])
            save("battery.png", "Tensão da bateria", "Tensão [V]", ["battery_v"])
            save("rpm_vs_cmd_m1.png", "Motor 1: RPM × comando", "Valor", ["rpm1", "cmd1"])
            self.root.after(0, lambda: self.analysis_status_var.set(f"{len(frame)} amostras · {output}"))
            self.root.after(0, lambda: os.startfile(output))
        except Exception as exc:
            self.root.after(0, lambda error=str(exc): self.analysis_status_var.set(f"Erro: {error}"))
            self.root.after(0, lambda error=str(exc): messagebox.showerror("Análise", error))

    def config_worker(self, action, uid=None, robot_id=None):
        try:
            port, baud = self.serial_settings()
            class Args: pass
            args = Args()
            args.action, args.port, args.baud = action, port, baud
            args.uid, args.robot_id = uid, robot_id
            with serial.Serial(port, baud, timeout=0.05) as ser:
                time.sleep(0.25)
                ser.reset_input_buffer()
                if action == "discover":
                    values = []
                    received_bytes = 0
                    for attempt in range(2):
                        nonce = (int(time.time() * 1000) + attempt) & 0xFFFFFFFF
                        payload = struct.pack("<HBI", 0xAA55, CONFIG_DISCOVER_TYPE, nonce)
                        ser.write(packet_with_crc(payload))
                        ser.flush()
                        values.extend(read_config_responses(ser, 1.2))
                        received_bytes += read_config_responses.last_rx_bytes
                    self.root.after(0, self.show_boards, values, received_bytes)
                else:
                    target = parse_uid(uid)
                    payload = struct.pack("<HB12sBI", 0xAA55, CONFIG_SET_ID_TYPE, target, ord(robot_id), CONFIG_KEY)
                    ser.write(packet_with_crc(payload))
                    ser.flush()
                    values = read_config_responses(ser, 1.0)
                    reply = next((v for v in values if v[1] == CONFIG_SET_ID_RESPONSE_TYPE and v[2] == target), None)
                    if reply is None:
                        raise RuntimeError("A placa não respondeu ao pedido de configuração")
                    if reply[3] == 0:
                        raise RuntimeError(
                            "A placa recebeu o pedido, mas não conseguiu gravar o ID na Flash.")
                    self.root.after(0, self.configuration_succeeded, robot_id)
        except Exception as exc:
            self.root.after(0, lambda error=str(exc): messagebox.showerror("Configuração", error))
        finally:
            self.root.after(0, self.finish_config_action)

    def begin_config_action(self, action, uid=None, robot_id=None):
        if self.config_busy:
            return
        if self.ser:
            self.disconnect()
        self.config_busy = True
        self.discover_button.configure(state="disabled", text="AGUARDE…")
        self.set_id_button.configure(state="disabled")
        # Let Windows release the COM handle before the worker opens it again.
        delay_ms = 150
        self.root.after(
            delay_ms,
            lambda: threading.Thread(
                target=self.config_worker,
                args=(action, uid, robot_id),
                daemon=True,
            ).start(),
        )

    def finish_config_action(self):
        self.config_busy = False
        self.discover_button.configure(state="normal", text="DESCOBRIR PLACAS")
        self.set_id_button.configure(state="normal")

    def configuration_succeeded(self, robot_id):
        self.robot_var.set(robot_id)
        self.new_id_var.set(robot_id)
        current_ids = set(self.robot_box["values"])
        current_ids.add(robot_id)
        self.robot_box["values"] = tuple(sorted(current_ids))
        messagebox.showinfo(
            "Configuração",
            f"Robô {robot_id} configurado com sucesso. O controle foi ajustado para o mesmo ID.",
        )

    def discover(self):
        if self.config_busy:
            return
        self.board_list.delete(0, "end")
        self.board_list.insert("end", "Procurando placas…")
        self.discovery_summary_var.set("Descoberta em andamento…")
        self.begin_config_action("discover")

    def show_boards(self, values, received_bytes=0):
        self.board_list.delete(0, "end")
        unique = {}
        for value in values:
            if value[1] == CONFIG_DISCOVER_RESPONSE_TYPE:
                unique[value[2]] = value
        boards = list(unique.values())
        self.discovered_boards = boards
        if not boards:
            self.board_list.insert(
                "end", f"Nenhuma resposta válida · {received_bytes} byte(s) recebidos")
            self.discovery_summary_var.set("0 placas encontradas")
            return

        configured_ids = [chr(value[4]) for value in boards
                          if ord("A") <= value[4] <= ord("Z")]
        id_counts = {robot_id: configured_ids.count(robot_id) for robot_id in set(configured_ids)}
        duplicates = sorted(robot_id for robot_id, count in id_counts.items() if count > 1)
        available_ids = tuple(sorted(id_counts))
        if available_ids:
            self.robot_box["values"] = available_ids
        unconfigured_count = len(boards) - len(configured_ids)
        summary = f"{len(boards)} placa(s) · IDs: {', '.join(available_ids) or 'nenhum'}"
        if unconfigured_count:
            summary += f" · {unconfigured_count} sem ID"
        if duplicates:
            summary += f" · IDs DUPLICADOS: {', '.join(duplicates)}"
        self.discovery_summary_var.set(summary)

        for value in boards:
            uid = value[2].hex().upper()
            robot = chr(value[4]) if ord("A") <= value[4] <= ord("Z") else "NÃO CONFIGURADO"
            self.board_list.insert("end", f"{uid}    ID: {robot}    geração: {value[5]}")

    def select_board(self, _event=None):
        selection = self.board_list.curselection()
        if selection:
            index = selection[0]
            if index >= len(self.discovered_boards):
                return
            value = self.discovered_boards[index]
            self.uid_var.set(value[2].hex().upper())
            discovered_id = chr(value[4]) if ord("A") <= value[4] <= ord("Z") else None
            if discovered_id:
                self.robot_var.set(discovered_id)
                self.new_id_var.set(discovered_id)

    def set_id(self):
        uid, robot_id = self.uid_var.get().strip(), self.new_id_var.get().upper()
        if len(robot_id) != 1 or not ("A" <= robot_id <= "Z"):
            messagebox.showerror("Configuração", "Escolha um ID entre A e Z")
            return
        self.begin_config_action("set-id", uid, robot_id)

    def close(self):
        self.disconnect()
        if self.log_file:
            self.log_file.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()
        return 0


def main():
    args = parse_args()
    if args.action == "app":
        try:
            from ssl_configurator_qt import QtConfiguratorApp
        except ImportError as exc:
            if exc.name == "PySide6":
                raise RuntimeError(
                    "A interface gráfica requer PySide6. Instale com: "
                    "python -m pip install PySide6"
                ) from exc
            raise
        return QtConfiguratorApp(args, sys.modules[__name__]).run()
    if args.action in ("discover", "set-id", "set-motion"):
        return run_config_action(args)

    global pygame
    import pygame

    robot_id = args.robot_id.upper()
    if (len(robot_id) != 1 or (robot_id != "*" and not ("A" <= robot_id <= "Z"))):
        raise ValueError("--robot-id must be one letter A-Z or *")
    if robot_id == "*" and args.telemetry_hz > 0.0:
        raise ValueError("directed telemetry cannot use broadcast ID; set --telemetry-hz 0")
    kick_power = max(0, min(100, args.kick_power))
    period_s = 1.0 / max(1.0, args.rate_hz)
    vx = 0.0
    vy = 0.0
    omega = 0.0
    last_update = time.monotonic()
    log_file = open(args.log, "w", newline="") if args.log else None
    log_writer = csv.writer(log_file) if log_file else None
    if log_writer:
        log_writer.writerow(CSV_HEADER)

    pygame.init()
    pygame.display.set_caption("TauraBots Control")
    screen = pygame.display.set_mode((500, 560))
    font = pygame.font.SysFont("consolas", 18)
    clock = pygame.time.Clock()

    with serial.Serial(args.port, args.baud, timeout=0.02) as ser:
        sequence = 0
        ser.write(encode_robot_velocity_packet(robot_id, sequence, 0.0, 0.0, 0.0, brake=1))
        next_send = time.monotonic()
        running = True
        kick_pending = False
        telemetry_period_s = 0.0 if args.telemetry_hz <= 0.0 else 1.0 / args.telemetry_hz
        next_telemetry_request = time.monotonic()
        telemetry_request_sequence = 0
        telemetry_rx_buffer = bytearray()
        telemetry = None

        try:
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    elif event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_x):
                        running = False
                    elif event.type == pygame.KEYDOWN and event.key == pygame.K_k:
                        kick_pending = True

                keys = pygame.key.get_pressed()

                target_vx = 0.0
                target_vy = 0.0
                target_omega = 0.0

                if keys[pygame.K_SPACE]:
                    pass
                else:
                    if keys[pygame.K_w]:
                        target_vy += args.linear
                    if keys[pygame.K_s]:
                        target_vy -= args.linear
                    if keys[pygame.K_d]:
                        target_vx += args.linear
                    if keys[pygame.K_a]:
                        target_vx -= args.linear
                    if keys[pygame.K_q]:
                        target_omega += args.angular
                    if keys[pygame.K_e]:
                        target_omega -= args.angular

                now = time.monotonic()
                dt = max(0.0, now - last_update)
                last_update = now

                target_is_zero = (target_vx == 0.0) and (target_vy == 0.0) and (target_omega == 0.0)
                tau = max(0.001, args.stop_tau if target_is_zero else args.accel_tau)
                alpha = 1.0 - math.exp(-dt / tau)

                vx += (target_vx - vx) * alpha
                vy += (target_vy - vy) * alpha
                omega += (target_omega - omega) * alpha

                if abs(vx) < 1.0e-4:
                    vx = 0.0
                if abs(vy) < 1.0e-4:
                    vy = 0.0
                if abs(omega) < 1.0e-4:
                    omega = 0.0

                if now >= next_send:
                    sequence = (sequence + 1) & 0xFFFFFFFF
                    brake = int(vx == vy == omega == 0.0)
                    packet_kick_power = kick_power if kick_pending else 0
                    ser.write(encode_robot_velocity_packet(
                        robot_id, sequence, vx, vy, omega, packet_kick_power, brake))
                    kick_pending = False
                    next_send = now + period_s

                if telemetry_period_s > 0.0 and now >= next_telemetry_request:
                    telemetry_request_sequence = (telemetry_request_sequence + 1) & 0xFFFF
                    ser.write(encode_telemetry_request(robot_id, telemetry_request_sequence))
                    next_telemetry_request = now + telemetry_period_s
                    next_send = max(next_send, now + TELEMETRY_REPLY_WINDOW_S)

                waiting = ser.in_waiting
                if waiting:
                    telemetry_rx_buffer.extend(ser.read(waiting))
                    new_telemetry = parse_telemetry(telemetry_rx_buffer, robot_id)
                    if new_telemetry is not None:
                        telemetry = new_telemetry
                        if log_writer:
                            log_writer.writerow(telemetry_csv_row(telemetry))
                            log_file.flush()

                draw(screen, font, vx, vy, omega, args.port, args.baud, robot_id, kick_power, telemetry)
                clock.tick(60)
        finally:
            sequence = (sequence + 1) & 0xFFFFFFFF
            ser.write(encode_robot_velocity_packet(robot_id, sequence, 0.0, 0.0, 0.0, brake=1))
            pygame.quit()
            if log_file:
                log_file.close()
            print("Stopped")

    return 0


if __name__ == "__main__":
    sys.exit(main())
