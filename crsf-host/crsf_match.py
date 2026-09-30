"""CRSF RC_CHANNELS_PACKED helpers for ELRS multi-robot match mode."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional


CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8
CRSF_TYPE_RC_CHANNELS_PACKED = 0x16
CRSF_CHANNEL_COUNT = 16
CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_CENTER = 992
CRSF_CHANNEL_MAX = 1811
MAX_VX = 2.5
MAX_VY = 2.5
MAX_OMEGA = 8.0

ACTION_KICK = 1 << 0
ACTION_CHIP = 1 << 1
ACTION_BRAKE = 1 << 2
ACTION_DRIBBLER = 1 << 3


@dataclass
class RobotCommand:
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    kick: bool = False
    chip: bool = False
    brake: bool = False
    dribbler: bool = False
    kick_power: int = 0


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def _clamp_channel(value: int) -> int:
    return max(CRSF_CHANNEL_MIN, min(CRSF_CHANNEL_MAX, int(value)))


def quantize_unsigned(value: float, minimum: float, maximum: float) -> int:
    if maximum <= minimum:
        return CRSF_CHANNEL_MIN
    value = max(minimum, min(maximum, float(value)))
    normalized = (value - minimum) / (maximum - minimum)
    return _clamp_channel(round(CRSF_CHANNEL_MIN + normalized * (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)))


def quantize_signed(value: float, maximum_abs: float) -> int:
    return quantize_unsigned(value, -maximum_abs, maximum_abs)


def dequantize_signed(channel: int, maximum_abs: float) -> float:
    channel = _clamp_channel(channel)
    normalized = (channel - CRSF_CHANNEL_MIN) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)
    return (normalized * 2.0 - 1.0) * maximum_abs


def _encode_action(command: RobotCommand) -> int:
    bits = 0
    bits |= ACTION_KICK if command.kick else 0
    bits |= ACTION_CHIP if command.chip else 0
    bits |= ACTION_BRAKE if command.brake else 0
    bits |= ACTION_DRIBBLER if command.dribbler else 0
    return quantize_unsigned(bits, 0, 15)


def _decode_action(channel: int) -> int:
    channel = _clamp_channel(channel)
    return round((channel - CRSF_CHANNEL_MIN) * 15 / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN))


def pack_channels(channels: Iterable[int]) -> bytes:
    values = [_clamp_channel(v) & 0x7FF for v in channels]
    if len(values) != CRSF_CHANNEL_COUNT:
        raise ValueError("expected exactly 16 channels")
    bit_buffer = 0
    bits = 0
    out = bytearray()
    for value in values:
        bit_buffer |= value << bits
        bits += 11
        while bits >= 8:
            out.append(bit_buffer & 0xFF)
            bit_buffer >>= 8
            bits -= 8
    if len(out) != 22:
        raise AssertionError("CRSF RC payload must be 22 bytes")
    return bytes(out)


def unpack_channels(payload: bytes) -> List[int]:
    if len(payload) != 22:
        raise ValueError("RC_CHANNELS_PACKED payload must be 22 bytes")
    bit_buffer = 0
    bits = 0
    pos = 0
    channels = []
    for _ in range(CRSF_CHANNEL_COUNT):
        while bits < 11:
            bit_buffer |= payload[pos] << bits
            bits += 8
            pos += 1
        channels.append(bit_buffer & 0x7FF)
        bit_buffer >>= 11
        bits -= 11
    return channels


def encode_rc_channels_frame(channels: Iterable[int]) -> bytes:
    payload = bytes([CRSF_TYPE_RC_CHANNELS_PACKED]) + pack_channels(channels)
    length = len(payload) + 1
    frame = bytes([CRSF_ADDRESS_FLIGHT_CONTROLLER, length]) + payload
    return frame + bytes([crc8_dvb_s2(payload)])


def parse_rc_channels_stream(buffer: bytearray) -> List[List[int]]:
    frames = []
    while len(buffer) >= 4:
        if buffer[0] != CRSF_ADDRESS_FLIGHT_CONTROLLER:
            del buffer[0]
            continue
        length = buffer[1]
        total = length + 2
        if length < 2 or total > 64:
            del buffer[0]
            continue
        if len(buffer) < total:
            break
        frame = bytes(buffer[:total])
        del buffer[:total]
        payload = frame[2:-1]
        if crc8_dvb_s2(payload) != frame[-1]:
            continue
        if payload[0] == CRSF_TYPE_RC_CHANNELS_PACKED and len(payload) == 23:
            frames.append(unpack_channels(payload[1:]))
    return frames


def encode_team_channels(commands: dict[str, RobotCommand], enabled: bool = True) -> List[int]:
    channels = [CRSF_CHANNEL_CENTER] * CRSF_CHANNEL_COUNT
    for robot_id, base, extra in (("A", 0, 12), ("B", 4, 13), ("C", 8, 14)):
        command = commands.get(robot_id, RobotCommand())
        channels[base] = quantize_signed(command.vx, MAX_VX)
        channels[base + 1] = quantize_signed(command.vy, MAX_VY)
        channels[base + 2] = quantize_signed(command.omega, MAX_OMEGA)
        channels[base + 3] = _encode_action(command)
        channels[extra] = quantize_unsigned(command.kick_power, 0, 100)
    channels[15] = CRSF_CHANNEL_MAX if enabled else CRSF_CHANNEL_MIN
    return channels


def decode_robot_from_channels(channels: List[int], robot_id: str, previous_action_bits: int = 0) -> tuple[RobotCommand, int]:
    index = "ABC".find(robot_id)
    if index < 0:
        raise ValueError("robot_id must be A, B or C")
    sequence_enabled = channels[15] > CRSF_CHANNEL_CENTER
    if not sequence_enabled:
        return RobotCommand(brake=True), 0
    base = index * 4
    action = _decode_action(channels[base + 3])
    command = RobotCommand(
        vx=dequantize_signed(channels[base], MAX_VX),
        vy=dequantize_signed(channels[base + 1], MAX_VY),
        omega=dequantize_signed(channels[base + 2], MAX_OMEGA),
        kick=bool(action & ACTION_KICK) and not bool(previous_action_bits & ACTION_KICK),
        chip=bool(action & ACTION_CHIP),
        brake=bool(action & ACTION_BRAKE),
        dribbler=bool(action & ACTION_DRIBBLER),
        kick_power=round((_clamp_channel(channels[12 + index]) - CRSF_CHANNEL_MIN) * 100 / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MIN)),
    )
    if not command.kick:
        command.kick_power = 0
    return command, action


class ActionEdgeFilter:
    def __init__(self) -> None:
        self.previous = {"A": 0, "B": 0, "C": 0}
        self.valid = {"A": False, "B": False, "C": False}

    def decode(self, channels: List[int], robot_id: str) -> RobotCommand:
        command, action = decode_robot_from_channels(channels, robot_id, self.previous[robot_id])
        if not self.valid[robot_id]:
            command.kick = False
            command.kick_power = 0
        self.previous[robot_id] = action
        self.valid[robot_id] = True
        return command


def make_match_frame(commands: dict[str, RobotCommand], enabled: bool = True) -> bytes:
    return encode_rc_channels_frame(encode_team_channels(commands, enabled=enabled))


def crc8_simple(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


@dataclass
class TeamFrameFragment:
    sequence: int
    part: int
    data: bytes


class TeamFrameAssembler:
    """Experimental 4-fragment, 128-bit atomic TeamFrame assembler."""

    def __init__(self) -> None:
        self.sequence: Optional[int] = None
        self.parts: dict[int, bytes] = {}

    def push(self, fragment: TeamFrameFragment) -> Optional[bytes]:
        if fragment.part not in range(4) or len(fragment.data) != 4:
            self.sequence = None
            self.parts.clear()
            return None
        if self.sequence is None or fragment.sequence != self.sequence:
            self.sequence = fragment.sequence
            self.parts.clear()
        self.parts[fragment.part] = fragment.data
        if len(self.parts) != 4:
            return None
        frame = b"".join(self.parts[i] for i in range(4))
        self.sequence = None
        self.parts.clear()
        if crc8_simple(frame[:-1]) != frame[-1]:
            return None
        return frame
