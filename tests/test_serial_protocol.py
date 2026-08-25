import importlib.util
import struct
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "ssl_configurator_backend", ROOT / "scripts" / "ssl-configurator.py"
)
BACKEND = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BACKEND)


class SerialProtocolTests(unittest.TestCase):
    def test_crc_known_vector(self):
        self.assertEqual(BACKEND.crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_robot_velocity_frame_layout(self):
        frame = BACKEND.encode_robot_velocity_packet(
            "B", 0x12345678, 1.25, -0.5, 3.0, kick_power=80, brake=1
        )
        self.assertEqual(len(frame), 19)
        fields = struct.unpack("<HBBBIhhhBBH", frame)
        self.assertEqual(
            fields[:-1],
            (0xAA55, 0xD0, 1, ord("B"), 0x12345678, 1250, -500, 3000, 80, 1),
        )
        self.assertEqual(fields[-1], BACKEND.crc16_ccitt_false(frame[:-2]))

    def test_robot_velocity_frame_clamps_wire_values(self):
        frame = BACKEND.encode_robot_velocity_packet(
            "A", 1, 100.0, -100.0, 100.0, kick_power=200
        )
        fields = struct.unpack("<HBBBIhhhBBH", frame)
        self.assertEqual(fields[5:8], (32767, -32768, 32767))
        self.assertEqual(fields[8], 100)

    def test_telemetry_fault_bits_are_exposed(self):
        status = 1 | ((0x01 | 0x04 | 0x10) << 1)
        values = (
            0xAA55, 0xE1, 1, ord("A"), 0x0F, status, 7, 1000, 22,
            100, 200, 300, 400, 1, 2, 3, 4, 12000, 2048, 0, 1, 0, 0,
        )
        payload = struct.pack(BACKEND.TELEMETRY_FMT, *values)
        frame = payload[:-2] + struct.pack("<H", BACKEND.crc16_ccitt_false(payload[:-2]))
        telemetry = BACKEND.parse_telemetry(bytearray(frame), "A")
        self.assertEqual(telemetry["fault_status"], 0x15)
        self.assertEqual(telemetry["rpm"], (10.0, 20.0, 30.0, 40.0))


if __name__ == "__main__":
    unittest.main()
