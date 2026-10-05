import struct
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "configurator"))

from core.protocol import Protocol

BACKEND = Protocol


class SerialProtocolTests(unittest.TestCase):
    @staticmethod
    def telemetry_frame(flags, status=1):
        frame = bytearray(struct.pack("<HBBBHBB", 0xAA55, 0xE1, 1, ord("A"), 7, status, flags))
        if flags & BACKEND.TELEMETRY_FLAG_BASIC:
            frame.extend(struct.pack("<IBBB", 1000, 1, 0, 60))
        if flags & BACKEND.TELEMETRY_FLAG_MOTORS:
            frame.extend(struct.pack("<hhhhhhhh", 100, 200, 300, 400, 1, 2, 3, 4))
        if flags & BACKEND.TELEMETRY_FLAG_BATTERY:
            frame.extend(struct.pack("<HH", 12000, 2048))
        if flags & BACKEND.TELEMETRY_FLAG_DIAGNOSTICS:
            frame.extend(struct.pack("<IIBI", 2, 30, 1, 22))
        frame.extend(struct.pack("<H", BACKEND.crc16(frame)))
        return bytes(frame)

    def test_crc_known_vector(self):
        self.assertEqual(BACKEND.crc16(b"123456789"), 0x29B1)

    def test_robot_velocity_frame_layout(self):
        frame = BACKEND.encode_velocity(
            "B", 0x12345678, 1.25, -0.5, 3.0, kick_power=80, brake=1
        )
        self.assertEqual(len(frame), 19)
        fields = struct.unpack("<HBBBIhhhBBH", frame)
        self.assertEqual(
            fields[:-1],
            (0xAA55, 0xD0, 1, ord("B"), 0x12345678, 1250, -500, 3000, 80, 1),
        )
        self.assertEqual(fields[-1], BACKEND.crc16(frame[:-2]))

    def test_robot_velocity_frame_clamps_wire_values(self):
        frame = BACKEND.encode_velocity(
            "A", 1, 100.0, -100.0, 100.0, kick_power=200
        )
        fields = struct.unpack("<HBBBIhhhBBH", frame)
        self.assertEqual(fields[5:8], (32767, -32768, 32767))
        self.assertEqual(fields[8], 100)

    def test_motion_configuration_frame_layout(self):
        uid = bytes.fromhex("0123456789ABCDEF01234567")
        frame = BACKEND.encode_motion_config(uid, 4.0, 10.0)
        self.assertEqual(len(frame), 29)
        fields = struct.unpack("<HB12sffIH", frame)
        self.assertEqual(fields[:3], (0xAA55, BACKEND.CONFIG_SET_MOTION_TYPE, uid))
        self.assertAlmostEqual(fields[3], 4.0)
        self.assertAlmostEqual(fields[4], 10.0)
        self.assertEqual(fields[5], BACKEND.CONFIG_KEY)
        self.assertEqual(fields[6], BACKEND.crc16(frame[:-2]))

    def test_motion_configuration_rejects_invalid_limits(self):
        uid = bytes(12)
        with self.assertRaises(ValueError):
            BACKEND.encode_motion_config(uid, 0.0, 10.0)
        with self.assertRaises(ValueError):
            BACKEND.encode_motion_config(uid, 4.0, 100.0)

    def test_telemetry_fault_bits_are_exposed(self):
        status = 1 | ((0x01 | 0x04 | 0x10) << 1)
        frame = self.telemetry_frame(BACKEND.TELEMETRY_FLAGS_FULL, status)
        telemetry = BACKEND.parse_telemetry(bytearray(frame), "A")
        self.assertEqual(telemetry["fault_status"], 0x15)
        self.assertEqual(telemetry["rpm"], (10.0, 20.0, 30.0, 40.0))

    def test_telemetry_request_layout_and_flags(self):
        frame = BACKEND.encode_telemetry_request("C", 100, BACKEND.TELEMETRY_FLAG_BATTERY)
        self.assertEqual(len(frame), 10)
        self.assertEqual(struct.unpack("<HBBBHBH", frame)[:-1],
                         (0xAA55, 0xE0, 1, ord("C"), 100, BACKEND.TELEMETRY_FLAG_BATTERY))
        self.assertEqual(struct.unpack_from("<H", frame, 8)[0], BACKEND.crc16(frame[:8]))

    def test_each_optional_group_is_independent(self):
        cases = (
            (BACKEND.TELEMETRY_FLAG_BASIC, {"time_ms", "comm_ok", "brake", "kick_power"}),
            (BACKEND.TELEMETRY_FLAG_MOTORS, {"rpm", "cmd"}),
            (BACKEND.TELEMETRY_FLAG_BATTERY, {"battery_v", "battery_adc"}),
            (BACKEND.TELEMETRY_FLAG_DIAGNOSTICS,
             {"crc_errors", "received_packets", "watchdog_ok", "command_sequence"}),
        )
        optional = set().union(*(keys for _, keys in cases))
        for flag, expected in cases:
            with self.subTest(flag=flag):
                telemetry = BACKEND.parse_telemetry(bytearray(self.telemetry_frame(flag)), "A")
                self.assertTrue(expected.issubset(telemetry))
                self.assertFalse((optional - expected) & telemetry.keys())


if __name__ == "__main__":
    unittest.main()
