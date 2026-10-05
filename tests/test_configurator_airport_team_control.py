import importlib.util
import struct
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "configurator"))

from core.protocol import Protocol, TeamRobotCommand
from core.radio_manager import RadioManager


def load_bridge():
    path = ROOT / "crsf-host" / "airport_team_bridge.py"
    spec = importlib.util.spec_from_file_location("airport_team_bridge_reference", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


BRIDGE = load_bridge()


class FakeSerial:
    is_open = True

    def __init__(self):
        self.frames = []

    def write(self, frame):
        self.frames.append(bytes(frame))
        return len(frame)

    def close(self):
        self.is_open = False


def slot(frame, robot_id):
    offset = 6 + "ABC".index(robot_id) * 8
    return struct.unpack_from("<hhhBB", frame, offset)


class TeamProtocolTests(unittest.TestCase):
    def test_layout_crc_negative_values_flags_and_reference_encoder(self):
        commands = {
            "A": TeamRobotCommand(vx=1.0, vy=-0.5, omega=40.0, kick_power=80,
                                  kick=True, brake=True, enabled=True),
            "B": TeamRobotCommand(vy=1.25, chip=True, dribbler=True, enabled=True),
            "C": TeamRobotCommand(omega=-1.0),
        }
        frame = Protocol.encode_team_velocity(0x1234, commands)
        self.assertEqual(len(frame), 32)
        self.assertEqual(frame[:4], bytes((0x55, 0xAA, 0xD1, 0x01)))
        self.assertEqual(frame[4:6], b"\x34\x12")
        self.assertEqual(slot(frame, "A")[:4], (1000, -500, 32767, 80))
        self.assertEqual(slot(frame, "B")[:3], (0, 1250, 0))
        self.assertEqual(slot(frame, "C")[:3], (0, 0, -1000))
        self.assertEqual(slot(frame, "A")[4],
                         Protocol.TEAM_FLAG_KICK | Protocol.TEAM_FLAG_BRAKE |
                         Protocol.TEAM_FLAG_ENABLED)
        self.assertEqual(slot(frame, "B")[4],
                         Protocol.TEAM_FLAG_CHIP | Protocol.TEAM_FLAG_DRIBBLER |
                         Protocol.TEAM_FLAG_ENABLED)
        self.assertEqual(struct.unpack_from("<H", frame, 30)[0], Protocol.crc16(frame[:-2]))

        reference = BRIDGE.build_team_frame(
            0x1234,
            BRIDGE.RobotCommand(vx=1.0, vy=-0.5, omega=40.0, kick_power=80,
                                kick=True, brake=True, enabled=True),
            BRIDGE.RobotCommand(vy=1.25, chip=True, dribbler=True, enabled=True),
            BRIDGE.RobotCommand(omega=-1.0),
        )
        self.assertEqual(frame, reference)


class TeamControlTests(unittest.TestCase):
    def setUp(self):
        self.radio = RadioManager()
        self.serial = FakeSerial()
        self.radio._serial = self.serial
        self.radio.state.connected = True
        self.radio.robots.register_expected_robots(["A", "B", "C"])

    def test_initial_frame_is_safe_and_sequence_wraps(self):
        self.radio._team_sequence = 0xFFFF
        self.radio.send_command(9.0, 9.0, 9.0)
        first = self.serial.frames[-1]
        self.assertEqual(struct.unpack_from("<H", first, 4)[0], 0xFFFF)
        self.assertEqual(self.radio._team_sequence, 0)
        for robot_id in "ABC":
            self.assertEqual(slot(first, robot_id), (0, 0, 0, 0, Protocol.TEAM_FLAG_BRAKE))

    def test_selected_zero_is_enabled_coast_not_brake(self):
        self.radio.select_robot("A")
        self.radio.send_command(0.0, 0.0, 0.0,
                                brake=self.radio._command_brake((0.0, 0.0, 0.0)))
        command = slot(self.serial.frames[-1], "A")
        self.assertEqual(command[:4], (0, 0, 0, 0))
        self.assertEqual(command[4], Protocol.TEAM_FLAG_ENABLED)

    def test_selection_updates_only_selected_slot_and_no_telemetry_allows_control(self):
        self.radio.select_robot("A")
        self.radio.send_command(1.0, -0.5, 0.25)
        a_frame = self.serial.frames[-1]
        self.assertEqual(slot(a_frame, "A")[:3], (1000, -500, 250))
        self.assertTrue(slot(a_frame, "A")[4] & Protocol.TEAM_FLAG_ENABLED)
        self.assertEqual(slot(a_frame, "B"), (0, 0, 0, 0, Protocol.TEAM_FLAG_BRAKE))

        self.radio.select_robot("B")
        self.radio.send_command(0.0, 1.0, 0.0)
        b_frame = self.serial.frames[-1]
        self.assertEqual(slot(b_frame, "A"), slot(a_frame, "A"))
        self.assertEqual(slot(b_frame, "B")[:3], (0, 1000, 0))
        self.assertTrue(slot(b_frame, "B")[4] & Protocol.TEAM_FLAG_ENABLED)

    def test_kick_is_one_slot_one_frame_edge(self):
        self.radio.select_robot("B")
        self.radio.send_command(0.0, 0.0, 0.0, kick_power=75)
        kicked = self.serial.frames[-1]
        self.assertEqual(slot(kicked, "B")[3], 75)
        self.assertTrue(slot(kicked, "B")[4] & Protocol.TEAM_FLAG_KICK)
        self.assertFalse(slot(kicked, "A")[4] & Protocol.TEAM_FLAG_KICK)
        self.assertFalse(slot(kicked, "C")[4] & Protocol.TEAM_FLAG_KICK)

        self.radio.send_command(0.0, 0.0, 0.0)
        cleared = self.serial.frames[-1]
        self.assertEqual(slot(cleared, "B")[3], 0)
        self.assertFalse(slot(cleared, "B")[4] & Protocol.TEAM_FLAG_KICK)

    def test_deselect_disables_only_selected_robot(self):
        self.radio.select_robot("A"); self.radio.send_command(0.5, 0.0, 0.0)
        self.radio.select_robot("B"); self.radio.send_command(0.0, 0.5, 0.0)
        self.radio.clear_robot_selection()
        frame = self.serial.frames[-1]
        self.assertTrue(slot(frame, "A")[4] & Protocol.TEAM_FLAG_ENABLED)
        self.assertEqual(slot(frame, "B"), (0, 0, 0, 0, Protocol.TEAM_FLAG_BRAKE))
        self.assertIsNone(self.radio.robots.active_robot)

    def test_emergency_stop_sends_three_safe_d1_frames_and_never_d0(self):
        self.radio.select_robot("C"); self.radio.send_command(0.0, 0.0, 1.0)
        self.radio.emergency_stop()
        safe_frames = self.serial.frames[-self.radio.EMERGENCY_SAFE_FRAMES:]
        self.assertEqual(len(safe_frames), 3)
        self.assertTrue(all(frame[2] == Protocol.TEAM_VELOCITY_TYPE
                            for frame in self.serial.frames))
        for frame in safe_frames:
            for robot_id in "ABC":
                self.assertEqual(slot(frame, robot_id),
                                 (0, 0, 0, 0, Protocol.TEAM_FLAG_BRAKE))

    def test_disconnect_safety_uses_d1_and_never_base_d0(self):
        self.radio.select_robot("A")
        self.radio.send_command(0.5, 0.0, 0.0)
        self.radio.disconnect_serial()
        self.assertFalse(self.serial.is_open)
        self.assertTrue(all(frame[2] == Protocol.TEAM_VELOCITY_TYPE
                            for frame in self.serial.frames))
        self.assertEqual(len(self.serial.frames), 1 + self.radio.EMERGENCY_SAFE_FRAMES)

    def test_telemetry_discovery_and_configuration_encoders_remain_available(self):
        self.assertEqual(Protocol.encode_telemetry_request("A", 1)[2], 0xE0)
        self.assertEqual(Protocol.encode_discovery_request(1)[2], 0xE2)
        self.assertEqual(Protocol.encode_discovery(1)[2], 0xF0)


if __name__ == "__main__":
    unittest.main()
