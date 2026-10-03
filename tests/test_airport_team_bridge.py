import struct
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "crsf-host"))

from airport_team_bridge import (  # noqa: E402
    FLAG_BRAKE,
    FLAG_CHIP,
    FLAG_DRIBBLER,
    FLAG_ENABLED,
    FLAG_KICK,
    RobotCommand,
    SHUTDOWN_SAFE_FRAMES,
    STARTUP_SAFE_FRAMES,
    build_team_frame,
    crc16,
    next_sequence,
    safe_commands,
    send_safe_frames,
)
from configurator.core.protocol import Protocol  # noqa: E402


class AirportTeamBridgeTests(unittest.TestCase):
    def setUp(self):
        self.a = RobotCommand(
            vx=1.0, vy=-0.5, omega=2.0, kick_power=17,
            kick=True, chip=True, brake=False, dribbler=True, enabled=True,
        )
        self.b = RobotCommand(
            vx=-1.25, vy=0.25, omega=-3.0, kick_power=42,
            brake=True, enabled=True,
        )
        self.c = RobotCommand(vx=40.0, vy=-40.0, enabled=True)
        self.frame = build_team_frame(0x1234, self.a, self.b, self.c)

    def test_frame_header_size_version_and_little_endian_sequence(self):
        self.assertEqual(len(self.frame), 32)
        self.assertEqual(self.frame[0:2], b"\x55\xAA")
        self.assertEqual(self.frame[2], 0xD1)
        self.assertEqual(self.frame[3], 0x01)
        self.assertEqual(self.frame[4:6], b"\x34\x12")

    def test_robot_slots_are_at_exact_offsets(self):
        self.assertEqual(
            struct.unpack_from("<hhhBB", self.frame, 6),
            (1000, -500, 2000, 17,
             FLAG_KICK | FLAG_CHIP | FLAG_DRIBBLER | FLAG_ENABLED),
        )
        self.assertEqual(
            struct.unpack_from("<hhhBB", self.frame, 14),
            (-1250, 250, -3000, 42, FLAG_BRAKE | FLAG_ENABLED),
        )
        self.assertEqual(
            struct.unpack_from("<hhhBB", self.frame, 22),
            (32767, -32768, 0, 0, FLAG_ENABLED),
        )

    def test_negative_int16_encoding_is_little_endian(self):
        self.assertEqual(self.frame[8:10], struct.pack("<h", -500))
        self.assertEqual(self.frame[14:16], struct.pack("<h", -1250))

    def test_flags_are_independent_and_exact(self):
        command = RobotCommand(
            kick=True, chip=True, brake=True, dribbler=True, enabled=True
        )
        frame = build_team_frame(0, command, RobotCommand(), RobotCommand())
        self.assertEqual(frame[13], 0x1F)

    def test_crc_matches_firmware_algorithm(self):
        crc_rx = struct.unpack_from("<H", self.frame, 30)[0]
        self.assertEqual(crc16(b"123456789"), 0x29B1)
        self.assertEqual(crc_rx, crc16(self.frame[:30]))
        self.assertEqual(crc_rx, Protocol.crc16(self.frame[:30]))

    def test_sequence_wraps_from_65535_to_zero(self):
        self.assertEqual(next_sequence(0xFFFF), 0)
        wrapped = build_team_frame(next_sequence(0xFFFF), self.a, self.b, self.c)
        self.assertEqual(wrapped[4:6], b"\x00\x00")

    def test_safe_frame_disables_all_robots_and_brakes(self):
        frame = build_team_frame(7, *safe_commands())
        for flags_offset in (13, 21, 29):
            self.assertEqual(frame[flags_offset] & FLAG_ENABLED, 0)
            self.assertNotEqual(frame[flags_offset] & FLAG_BRAKE, 0)
        for slot_offset in (6, 14, 22):
            self.assertEqual(struct.unpack_from("<hhhB", frame, slot_offset), (0, 0, 0, 0))

    def test_startup_and_shutdown_safety_frame_counts(self):
        class FakeSerial:
            def __init__(self):
                self.frames = []

            def write(self, frame):
                self.frames.append(frame)
                return len(frame)

        self.assertGreaterEqual(STARTUP_SAFE_FRAMES, 10)
        self.assertGreaterEqual(SHUTDOWN_SAFE_FRAMES, 20)
        serial_port = FakeSerial()
        next_value = send_safe_frames(serial_port, 0xFFF8, SHUTDOWN_SAFE_FRAMES)
        self.assertEqual(len(serial_port.frames), SHUTDOWN_SAFE_FRAMES)
        self.assertEqual(next_value, (0xFFF8 + SHUTDOWN_SAFE_FRAMES) & 0xFFFF)
        for frame in serial_port.frames:
            for flags_offset in (13, 21, 29):
                self.assertEqual(frame[flags_offset] & FLAG_ENABLED, 0)
                self.assertNotEqual(frame[flags_offset] & FLAG_BRAKE, 0)


if __name__ == "__main__":
    unittest.main()
