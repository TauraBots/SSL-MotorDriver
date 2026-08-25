"""Binary protocol codec shared by the Qt backend.

This module intentionally preserves the firmware packet layouts byte-for-byte.
"""

import math
import struct
import time


class Protocol:
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

    @staticmethod
    def crc16(data: bytes) -> int:
        crc = 0xFFFF
        for value in data:
            crc ^= value << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        return crc

    @classmethod
    def with_crc(cls, payload: bytes) -> bytes:
        return payload + struct.pack("<H", cls.crc16(payload))

    @staticmethod
    def parse_uid(text: str) -> bytes:
        compact = text.replace(":", "").replace("-", "").replace(" ", "")
        if len(compact) != 24:
            raise ValueError("UID deve conter exatamente 24 caracteres hexadecimais")
        return bytes.fromhex(compact)

    @classmethod
    def encode_velocity(cls, robot_id, sequence, vx, vy, omega, kick_power=0, brake=0):
        kick_power = max(0, min(100, kick_power))
        values = tuple(max(-32768, min(32767, int(round(v * 1000.0)))) for v in (vx, vy, omega))
        payload = struct.pack("<HBBBIhhhBB", 0xAA55, cls.ROBOT_VELOCITY_TYPE,
                              cls.ROBOT_VELOCITY_VERSION, ord(robot_id), sequence & 0xFFFFFFFF,
                              *values, kick_power, 1 if brake else 0)
        return cls.with_crc(payload)

    @classmethod
    def encode_telemetry_request(cls, robot_id, request_sequence, flags=TELEMETRY_FLAGS_FULL):
        payload = struct.pack("<HBBBHB", 0xAA55, cls.TELEMETRY_REQUEST_TYPE,
                              cls.TELEMETRY_VERSION, ord(robot_id), request_sequence & 0xFFFF,
                              flags & cls.TELEMETRY_FLAGS_FULL)
        return cls.with_crc(payload)

    @classmethod
    def encode_discovery(cls, nonce):
        return cls.with_crc(struct.pack("<HBI", 0xAA55, cls.CONFIG_DISCOVER_TYPE, nonce & 0xFFFFFFFF))

    @classmethod
    def encode_set_id(cls, uid, robot_id):
        return cls.with_crc(struct.pack("<HB12sBI", 0xAA55, cls.CONFIG_SET_ID_TYPE,
                                        uid, ord(robot_id), cls.CONFIG_KEY))

    @classmethod
    def encode_motion_config(cls, uid, linear_accel, angular_accel):
        if len(uid) != 12:
            raise ValueError("UID deve conter exatamente 12 bytes")
        if not math.isfinite(linear_accel) or not 0.1 <= linear_accel <= 20.0:
            raise ValueError("aceleração linear deve estar entre 0.1 e 20.0 m/s²")
        if not math.isfinite(angular_accel) or not 0.1 <= angular_accel <= 50.0:
            raise ValueError("aceleração angular deve estar entre 0.1 e 50.0 rad/s²")
        return cls.with_crc(struct.pack("<HB12sffI", 0xAA55, cls.CONFIG_SET_MOTION_TYPE,
                                        uid, linear_accel, angular_accel, cls.CONFIG_KEY))

    @classmethod
    def parse_telemetry(cls, rx_buffer, robot_id):
        latest = None
        marker = bytes((0x55, 0xAA, cls.TELEMETRY_RESPONSE_TYPE))
        while True:
            start = rx_buffer.find(marker)
            if start < 0:
                if len(rx_buffer) > cls.TELEMETRY_RESPONSE_BASE_SIZE:
                    del rx_buffer[:-2]
                break
            if start:
                del rx_buffer[:start]
            if len(rx_buffer) < 9:
                break
            flags = rx_buffer[8] & cls.TELEMETRY_FLAGS_FULL
            size = cls.TELEMETRY_RESPONSE_BASE_SIZE
            size += 7 if flags & cls.TELEMETRY_FLAG_BASIC else 0
            size += 16 if flags & cls.TELEMETRY_FLAG_MOTORS else 0
            size += 4 if flags & cls.TELEMETRY_FLAG_BATTERY else 0
            size += 13 if flags & cls.TELEMETRY_FLAG_DIAGNOSTICS else 0
            if len(rx_buffer) < size:
                break
            frame = bytes(rx_buffer[:size])
            if cls.crc16(frame[:-2]) != struct.unpack_from("<H", frame, size - 2)[0]:
                del rx_buffer[0]
                continue
            del rx_buffer[:size]
            if frame[3] != cls.TELEMETRY_VERSION or frame[4] != ord(robot_id):
                continue
            offset = 9
            latest = {"robot_id": chr(frame[4]), "flags": flags, "status": frame[7],
                      "fault_status": frame[7] >> 1,
                      "request_sequence": struct.unpack_from("<H", frame, 5)[0],
                      "received_at": time.monotonic()}
            if flags & cls.TELEMETRY_FLAG_BASIC:
                latest["time_ms"] = struct.unpack_from("<I", frame, offset)[0]; offset += 4
                latest["comm_ok"], latest["brake"], latest["kick_power"] = frame[offset:offset + 3]; offset += 3
            if flags & cls.TELEMETRY_FLAG_MOTORS:
                motors = struct.unpack_from("<hhhhhhhh", frame, offset); offset += 16
                latest["rpm"] = tuple(v / 10.0 for v in motors[:4]); latest["cmd"] = tuple(motors[4:])
            if flags & cls.TELEMETRY_FLAG_BATTERY:
                millivolts, latest["battery_adc"] = struct.unpack_from("<HH", frame, offset); offset += 4
                latest["battery_v"] = millivolts / 1000.0
            if flags & cls.TELEMETRY_FLAG_DIAGNOSTICS:
                latest["crc_errors"], latest["received_packets"] = struct.unpack_from("<II", frame, offset); offset += 8
                latest["watchdog_ok"] = frame[offset]; offset += 1
                latest["command_sequence"] = struct.unpack_from("<I", frame, offset)[0]
        return latest

    @classmethod
    def read_config_responses(cls, serial_port, duration_s):
        deadline, rx, responses, received = time.monotonic() + duration_s, bytearray(), [], 0
        while time.monotonic() < deadline:
            chunk = serial_port.read(serial_port.in_waiting or 1); received += len(chunk); rx.extend(chunk)
            start = rx.find(b"\x55\xAA")
            while start >= 0 and len(rx) >= start + cls.CONFIG_RESPONSE_SIZE:
                if start: del rx[:start]
                frame = bytes(rx[:cls.CONFIG_RESPONSE_SIZE]); del rx[:cls.CONFIG_RESPONSE_SIZE]
                values = struct.unpack(cls.CONFIG_RESPONSE_FMT, frame)
                if cls.crc16(frame[:-2]) == values[-1] and values[1] in (
                        cls.CONFIG_DISCOVER_RESPONSE_TYPE, cls.CONFIG_SET_ID_RESPONSE_TYPE,
                        cls.CONFIG_SET_MOTION_RESPONSE_TYPE):
                    responses.append(values)
                start = rx.find(b"\x55\xAA")
        return responses, received
