import struct
import sys
import time
import unittest
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "configurator"))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from core.protocol import Protocol
from core.robot_manager import RobotManager
from PySide6.QtWidgets import QApplication
from ui.fleet_panel import FleetPanel


def discovery_frame(robot_id, sequence=7, uid=None, firmware=(1, 2, 3),
                    status=1, battery_mv=11800):
    uid = uid or bytes([ord(robot_id)]) * 12
    payload = struct.pack("<HBBBH12sBBBBH", 0xAA55,
                          Protocol.DISCOVERY_RESPONSE_TYPE,
                          Protocol.DISCOVERY_VERSION, ord(robot_id), sequence,
                          uid, *firmware, status, battery_mv)
    return Protocol.with_crc(payload)


class RobotDiscoveryTests(unittest.TestCase):
    def setUp(self):
        self.manager = RobotManager()

    def discover(self, *robot_ids):
        seen = set()
        for robot_id in robot_ids:
            data = Protocol.parse_discovery_response(discovery_frame(robot_id))
            self.manager.handle_discovery_response(data, 20)
            seen.add(robot_id)
        self.manager.complete_discovery(seen)

    def test_no_robot_response_keeps_fleet_empty(self):
        self.manager.complete_discovery(set())
        self.assertEqual(self.manager.discovered_robots, ())
        self.assertEqual(self.manager.system_state.online_robot_count, 0)

    def test_one_robot_response_creates_robot_a(self):
        self.discover("A")
        self.assertEqual([robot.robot_id for robot in self.manager.discovered_robots], ["A"])
        robot = self.manager.discovered_robots[0]
        self.assertEqual(robot.uid, "41" * 12)
        self.assertEqual(robot.firmware_version, "v1.2.3")
        self.assertEqual(robot.battery, 11.8)
        self.assertEqual(robot.status, "ONLINE")

    def test_three_responses_create_a_b_c(self):
        self.discover("A", "B", "C")
        self.assertEqual([robot.robot_id for robot in self.manager.discovered_robots],
                         ["A", "B", "C"])
        self.assertEqual(self.manager.system_state.online_robot_count, 3)

    def test_no_control_target_is_ready_when_radio_and_fleet_are_healthy(self):
        self.manager.set_radio_connected(True)
        self.discover("A")
        self.assertIsNone(self.manager.system_state.active_robot_id)
        self.assertEqual(self.manager.system_state.system_status, "READY")

    def test_robot_fault_sets_global_warning(self):
        self.manager.set_radio_connected(True)
        data = Protocol.parse_discovery_response(discovery_frame("A", status=0x03))
        self.manager.handle_discovery_response(data, 20)
        self.assertEqual(self.manager.discovered_robots[0].status, "WARNING")
        self.assertEqual(self.manager.system_state.system_status, "WARNING")

    def test_missing_robot_becomes_offline_after_next_timeout(self):
        self.discover("A", "B")
        robot_b = self.manager.ensure_robot("B")
        robot_b.last_seen = time.monotonic() - 2.0
        self.discover("A")
        robots = {robot.robot_id: robot for robot in self.manager.discovered_robots}
        self.assertTrue(robots["A"].connected)
        self.assertFalse(robots["B"].connected)
        self.assertEqual(robots["B"].status, "NO TELEMETRY")
        self.assertTrue(robots["B"].can_control)
        self.assertEqual(self.manager.system_state.online_robot_count, 1)
        self.assertEqual(self.manager.system_state.registered_robot_count, 2)
        self.assertEqual(self.manager.uplink_robot_id, "A")

    def test_single_discovery_reply_keeps_registered_fleet_and_ui_states(self):
        self.manager.register_expected_robots(["A", "B"])
        data = Protocol.parse_discovery_response(discovery_frame("A"))
        self.manager.handle_discovery_response(data, 20)
        self.manager.complete_discovery({"A"})

        robot_a = self.manager.ensure_robot("A")
        robot_b = self.manager.ensure_robot("B")
        self.assertTrue(robot_a.connected)
        self.assertFalse(robot_b.connected)
        self.assertTrue(robot_b.discovered)
        self.assertEqual(robot_b.status, "NO TELEMETRY")
        self.assertEqual(self.manager.uplink_robot_id, "A")
        self.assertEqual((self.manager.system_state.registered_robot_count,
                          self.manager.system_state.online_robot_count), (2, 1))

        app = QApplication.instance() or QApplication([])
        panel = FleetPanel()
        try:
            panel.set_fleet(self.manager.discovered_robots, None,
                            self.manager.uplink_robot_id)
            self.assertEqual(panel.summary.text(), "2 REGISTERED / 1 TELEMETRY")
            self.assertEqual(panel._cards_by_id["A"].status.text(), "TELEMETRY ONLINE")
            self.assertTrue(panel._cards_by_id["A"].uplink.isVisibleTo(panel))
            self.assertEqual(panel._cards_by_id["B"].status.text(), "NO TELEMETRY")
            self.assertFalse(panel._cards_by_id["B"].uplink.isVisible())
        finally:
            panel.close()

    def test_missed_discovery_reply_does_not_drop_recent_robot_or_selection(self):
        self.manager.set_radio_connected(True)
        self.discover("A")
        self.manager.select_robot("A")
        self.manager.complete_discovery(set())
        self.assertTrue(self.manager.ensure_robot("A").connected)
        self.assertEqual(self.manager.system_state.active_robot_id, "A")

    def test_request_and_response_crc_and_sequence(self):
        request = Protocol.encode_discovery_request(0x1234)
        self.assertEqual(len(request), Protocol.DISCOVERY_REQUEST_SIZE)
        self.assertEqual(struct.unpack("<HBBHH", request),
                         (0xAA55, 0xE2, 1, 0x1234, Protocol.crc16(request[:-2])))
        response = Protocol.parse_discovery_response(discovery_frame("C", 0x1234))
        self.assertEqual(response["request_sequence"], 0x1234)
        self.assertEqual(response["robot_id"], "C")

    def test_same_uid_migrates_from_b_to_a_without_stale_registration(self):
        uid = bytes.fromhex("35FEDC054E58383412781043")
        first = Protocol.parse_discovery_response(discovery_frame("B", uid=uid))
        self.manager.handle_discovery_response(first, 20); self.manager.select_robot("B")
        migrated = Protocol.parse_discovery_response(discovery_frame("A", uid=uid))
        self.manager.handle_discovery_response(migrated, 18)
        self.manager.complete_discovery({"A"})
        self.assertEqual([robot.robot_id for robot in self.manager.discovered_robots], ["A"])
        self.assertNotIn("B", self.manager._robots)
        self.assertEqual(self.manager.system_state.active_robot_id, "A")
        self.assertEqual(self.manager.active_robot.uid, uid.hex().upper())


if __name__ == "__main__":
    unittest.main()
