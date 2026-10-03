import struct
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "configurator"))

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
        for robot_id in "AB":
            robot = self.radio.robots.ensure_robot(robot_id); robot.discovered = True
            robot.connected = False; robot.status = "PENDING"

    def _send_reply(self, robot_id, battery_mv=11800, rpm=(10, 20, 30, 40), now_ns=10_000_000):
        request = self.radio._serial.frames[-1]
        sequence = struct.unpack_from("<H", request, 5)[0]
        self.radio._rx.extend(telemetry_frame(robot_id, sequence, battery_mv, rpm))
        self.radio._consume_rx(now_ns)

    def test_valid_response_locks_polling_to_uplink_not_control_target(self):
        with patch("core.radio_manager.time.monotonic_ns", return_value=0):
            self.assertTrue(self.radio.send_telemetry_request())
        self.assertEqual(chr(self.radio._serial.frames[-1][4]), "A")
        self._send_reply("A")
        self.assertEqual(self.radio.uplink_robot_id, "A")

        self.radio.select_robot("B")
        self.assertTrue(self.radio.send_telemetry_request())
        self.assertEqual(chr(self.radio._serial.frames[-1][4]), "A")
        self.assertEqual(self.radio.robots.system_state.active_robot_id, "B")

    def test_uplink_loss_probes_next_robot_without_removing_registration(self):
        with patch("core.radio_manager.time.monotonic_ns", return_value=0):
            self.radio.send_telemetry_request()
        self._send_reply("A")
        state = self.radio.robots.system_state
        self.assertEqual((state.registered_robot_count, state.online_robot_count), (2, 1))

        robot_a = self.radio.robots.ensure_robot("A")
        robot_a.last_seen = 0.0
        with patch("core.robot_manager.time.monotonic", return_value=2.0):
            self.radio._expire_stale_robots()
        self.assertIsNone(self.radio.robots.uplink_robot_id)
        self.assertIsNone(self.radio.robots.system_state.latency_ms)
        self.assertEqual(robot_a.status, "NO TELEMETRY")
        self.assertTrue(robot_a.discovered)

        with patch("core.radio_manager.time.monotonic_ns", return_value=2_000_000_000):
            self.assertTrue(self.radio.send_telemetry_request())
        self.assertEqual(chr(self.radio._serial.frames[-1][4]), "B")
        self._send_reply("B", now_ns=2_010_000_000)
        self.assertEqual(self.radio.uplink_robot_id, "B")
        self.assertTrue(robot_a.discovered)

    def test_registered_robot_without_telemetry_remains_controllable(self):
        self.radio.state.connected = True
        self.radio.select_robot("B")
        self.assertFalse(self.radio.robots.active_robot.connected)
        self.assertTrue(self.radio.robots.active_robot.can_control)
        self.radio.send_command(0.3, 0.0, 0.0)
        frame = self.radio._serial.frames[-1]
        self.assertEqual(frame[2], Protocol.TEAM_VELOCITY_TYPE)
        self.assertEqual(struct.unpack_from("<h", frame, 14)[0], 300)
        self.assertTrue(frame[21] & Protocol.TEAM_FLAG_ENABLED)

    def test_manual_expected_robots_are_polled_without_discovery(self):
        radio = RadioManager(); radio._serial = FakeSerial(); radio.state.robot_id = "A"
        radio.robots.register_expected_robots(["A", "B"])
        self.assertEqual([robot.robot_id for robot in radio.robots.discovered_robots], ["A", "B"])
        self.assertEqual([robot.connected for robot in radio.robots.discovered_robots], [False, False])
        self.assertEqual([robot.can_control for robot in radio.robots.discovered_robots], [True, True])
        with patch("core.radio_manager.time.monotonic_ns", side_effect=(0, 500_000_000)):
            radio.send_telemetry_request(); radio.send_telemetry_request()
        targets = [chr(frame[4]) for frame in radio._serial.frames]
        self.assertEqual(targets, ["A", "B"])

    def test_manual_expected_robot_accepts_commands_without_telemetry(self):
        radio = RadioManager(); radio._serial = FakeSerial(); radio.state.connected = True
        radio.robots.register_expected_robots(["A"]); radio.select_robot("A")
        radio.send_command(0.3, 0.0, 0.0)
        frame = radio._serial.frames[-1]
        self.assertEqual(frame[2], Protocol.TEAM_VELOCITY_TYPE)
        self.assertEqual(struct.unpack_from("<h", frame, 6)[0], 300)
        self.assertTrue(frame[13] & Protocol.TEAM_FLAG_ENABLED)


if __name__ == "__main__":
    unittest.main()
