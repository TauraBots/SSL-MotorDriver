import struct
import sys
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from core.protocol import Protocol
from core.radio_manager import RadioManager


class FakeSerial:
    is_open = True

    def __init__(self):
        self.frames = []

    def write(self, frame):
        self.frames.append(bytes(frame)); return len(frame)


def telemetry_frame(robot_id, sequence, battery_mv, rpm):
    flags = Protocol.TELEMETRY_FLAGS_FULL
    payload = bytearray(struct.pack("<HBBBHBB", 0xAA55, Protocol.TELEMETRY_RESPONSE_TYPE,
                                    Protocol.TELEMETRY_VERSION, ord(robot_id), sequence, 1, flags))
    payload.extend(struct.pack("<IBBB", 1000, 1, 0, 0))
    payload.extend(struct.pack("<hhhhhhhh", *(int(value * 10) for value in rpm), 1, 2, 3, 4))
    payload.extend(struct.pack("<HH", battery_mv, 2048))
    payload.extend(struct.pack("<IIBI", 0, 10, 1, sequence))
    return Protocol.with_crc(payload)


class MultiRobotTelemetryTests(unittest.TestCase):
    def setUp(self):
        self.radio = RadioManager(); self.radio._serial = FakeSerial(); self.radio.state.robot_id = "A"
        for robot_id in "ABC":
            robot = self.radio.robots.ensure_robot(robot_id); robot.discovered = True
            robot.connected = True; robot.status = "ONLINE"; robot.last_seen = time.monotonic()

    def test_polling_round_robin_is_independent_from_control_target(self):
        targets = []
        for _ in range(3):
            self.radio.send_telemetry_request(); targets.append(chr(self.radio._serial.frames[-1][4]))
        self.assertEqual(targets, ["A", "B", "C"])
        self.radio.select_robot("B")
        self.radio.send_telemetry_request()
        self.assertEqual(chr(self.radio._serial.frames[-1][4]), "A")
        self.assertEqual(self.radio.robots.system_state.active_robot_id, "B")

    def test_all_robot_states_update_while_b_is_control_target(self):
        self.radio.select_robot("B")
        expected = {"A": (11800, (10, 20, 30, 40)),
                    "B": (11600, (50, 60, 70, 80)),
                    "C": (11400, (90, 100, 110, 120))}
        for robot_id in "ABC":
            self.radio.send_telemetry_request()
            sequence = struct.unpack_from("<H", self.radio._serial.frames[-1], 5)[0]
            battery, rpm = expected[robot_id]
            self.radio._rx.extend(telemetry_frame(robot_id, sequence, battery, rpm))
            self.radio._consume_rx(time.monotonic())
        for robot_id, (battery, rpm) in expected.items():
            state = self.radio.robots.ensure_robot(robot_id)
            self.assertAlmostEqual(state.battery, battery / 1000.0)
            self.assertEqual(state.rpm, rpm)
        self.assertEqual(self.radio.robots.system_state.active_robot_id, "B")
        self.assertEqual(self.radio.state.robot_id, "B")


if __name__ == "__main__":
    unittest.main()
