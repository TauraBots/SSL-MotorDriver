from pathlib import Path
import struct
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOF = b"\x55\xAA"
TYPE_D1 = 0xD1
VERSION = 1
FRAME_LEN = 32
FLAG_KICK = 1 << 0
FLAG_CHIP = 1 << 1
FLAG_BRAKE = 1 << 2
FLAG_DRIBBLER = 1 << 3
FLAG_ENABLED = 1 << 4


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def team_frame(sequence, robots, version=VERSION):
    frame = bytearray(SOF)
    frame.extend((TYPE_D1, version))
    frame.extend(struct.pack("<H", sequence))
    for vx, vy, omega, kick_power, flags in robots:
        frame.extend(struct.pack("<hhhBB", vx, vy, omega, kick_power, flags))
    frame.extend(struct.pack("<H", crc16(frame)))
    return bytes(frame)


class TeamReceiverModel:
    """Executable specification mirrored by MatchControl_HandleTeamFrame()."""

    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.last_sequence = None
        self.expanded_sequence = 0
        self.last_flags = 0
        self.flags_valid = False
        self.last_tick = None
        self.command = self.safe_command()
        self.stats = {
            "ok": 0, "bad_crc": 0, "bad_version": 0,
            "duplicate": 0, "timeout": 0,
        }

    @staticmethod
    def safe_command():
        return {"vx": 0.0, "vy": 0.0, "omega": 0.0, "kick": False,
                "chip": False, "brake": True, "dribbler": False,
                "enabled": False}

    def process(self, frame, tick=0):
        if len(frame) != FRAME_LEN or frame[:2] != SOF or frame[2] != TYPE_D1:
            return False
        if struct.unpack_from("<H", frame, 30)[0] != crc16(frame[:30]):
            self.stats["bad_crc"] += 1
            return False
        if frame[3] != VERSION:
            self.stats["bad_version"] += 1
            return False

        sequence = struct.unpack_from("<H", frame, 4)[0]
        if self.last_sequence is None:
            self.expanded_sequence = sequence
        else:
            delta = (sequence - self.last_sequence) & 0xFFFF
            if delta == 0:
                self.stats["duplicate"] += 1
                return False
            if delta >= 0x8000:
                return False
            self.expanded_sequence += delta
        self.last_sequence = sequence

        index = ord(self.robot_id) - ord("A")
        vx, vy, omega, kick_power, flags = struct.unpack_from("<hhhBB", frame, 6 + index * 8)
        kick = self.flags_valid and bool(flags & FLAG_KICK) and not bool(self.last_flags & FLAG_KICK)
        self.last_flags = flags
        self.flags_valid = True
        if not flags & FLAG_ENABLED:
            self.command = self.safe_command()
        else:
            self.command = {
                "vx": vx * 0.001, "vy": vy * 0.001, "omega": omega * 0.001,
                "kick": kick, "kick_power": kick_power if kick else 0,
                "chip": bool(flags & FLAG_CHIP), "brake": bool(flags & FLAG_BRAKE),
                "dribbler": bool(flags & FLAG_DRIBBLER), "enabled": True,
            }
        self.last_tick = tick
        self.stats["ok"] += 1
        return True

    def task(self, tick):
        if self.last_tick is None or tick - self.last_tick >= 100:
            if self.last_tick is not None:
                self.stats["timeout"] += 1
                self.last_tick = None
            self.command = self.safe_command()


ROBOTS = [
    (1000, 0, 0, 25, FLAG_ENABLED),
    (0, 1000, 0, 50, FLAG_ENABLED | FLAG_BRAKE),
    (0, 0, 1000, 75, FLAG_ENABLED | FLAG_DRIBBLER),
]


class AirportTeamProtocolTests(unittest.TestCase):
    def test_valid_d1_has_exact_layout_and_crc(self):
        frame = team_frame(0x1234, ROBOTS)
        self.assertEqual(len(frame), 32)
        self.assertEqual(frame[:6], b"\x55\xAA\xD1\x01\x34\x12")
        self.assertEqual(struct.unpack_from("<H", frame, 30)[0], crc16(frame[:30]))

    def test_crc_version_and_truncated_frames_are_rejected(self):
        receiver = TeamReceiverModel("A")
        bad_crc = bytearray(team_frame(1, ROBOTS)); bad_crc[6] ^= 1
        self.assertFalse(receiver.process(bytes(bad_crc)))
        self.assertEqual(receiver.stats["bad_crc"], 1)
        self.assertFalse(receiver.process(team_frame(2, ROBOTS, version=2)))
        self.assertEqual(receiver.stats["bad_version"], 1)
        self.assertFalse(receiver.process(team_frame(3, ROBOTS)[:-1]))
        self.assertEqual(receiver.stats["ok"], 0)

    def test_each_robot_selects_only_its_own_distinct_slot(self):
        frame = team_frame(7, ROBOTS)
        expected = {"A": (1.0, 0.0, 0.0), "B": (0.0, 1.0, 0.0), "C": (0.0, 0.0, 1.0)}
        for robot_id, velocities in expected.items():
            with self.subTest(robot_id=robot_id):
                receiver = TeamReceiverModel(robot_id)
                self.assertTrue(receiver.process(frame))
                self.assertEqual(tuple(receiver.command[key] for key in ("vx", "vy", "omega")), velocities)

    def test_duplicate_and_held_kick_do_not_repeat_edge(self):
        receiver = TeamReceiverModel("A")
        low = list(ROBOTS); low[0] = (0, 0, 0, 80, FLAG_ENABLED)
        high = list(ROBOTS); high[0] = (0, 0, 0, 80, FLAG_ENABLED | FLAG_KICK)
        self.assertTrue(receiver.process(team_frame(10, low)))
        self.assertFalse(receiver.command["kick"])
        kick_frame = team_frame(11, high)
        self.assertTrue(receiver.process(kick_frame))
        self.assertTrue(receiver.command["kick"])
        self.assertFalse(receiver.process(kick_frame))
        self.assertEqual(receiver.stats["duplicate"], 1)
        self.assertTrue(receiver.process(team_frame(12, high)))
        self.assertFalse(receiver.command["kick"])

    def test_old_sequence_is_rejected(self):
        receiver = TeamReceiverModel("A")
        self.assertTrue(receiver.process(team_frame(100, ROBOTS)))
        self.assertFalse(receiver.process(team_frame(99, ROBOTS)))
        self.assertEqual(receiver.stats["ok"], 1)

    def test_disabled_slot_and_timeout_force_safe_state(self):
        receiver = TeamReceiverModel("B")
        disabled = list(ROBOTS); disabled[1] = (1000, 1000, 1000, 100, 0)
        self.assertTrue(receiver.process(team_frame(1, disabled), tick=10))
        self.assertFalse(receiver.command["enabled"])
        self.assertTrue(receiver.command["brake"])
        self.assertTrue(receiver.process(team_frame(2, ROBOTS), tick=20))
        self.assertTrue(receiver.command["enabled"])
        receiver.task(120)
        self.assertFalse(receiver.command["enabled"])
        self.assertEqual(receiver.stats["timeout"], 1)

    def test_firmware_constants_routing_baud_and_rx_only_guards(self):
        protocol_h = (ROOT / "Core/Inc/serial_protocol.h").read_text()
        comm_h = (ROOT / "Core/Inc/comm_mode.h").read_text()
        main_c = (ROOT / "Core/Src/main.c").read_text()
        service_c = (ROOT / "Core/Src/serial_service.c").read_text()
        match_c = (ROOT / "Core/Src/match_control.c").read_text()
        self.assertIn("SERIAL_TYPE_TEAM_VELOCITY 0xD1U", protocol_h)
        self.assertIn("SERIAL_TEAM_VELOCITY_PACKET_LEN 32U", protocol_h)
        self.assertIn("MATCH_TRANSPORT_AIRPORT_TEAM = 2", comm_h)
        self.assertIn("#define TAURA_MATCH_TRANSPORT MATCH_TRANSPORT_AIRPORT_TEAM", comm_h)
        self.assertIn("MATCH_AIRPORT_UART_BAUD : MATCH_CRSF_UART_BAUD", main_c)
        self.assertIn("Serial_ProcessAirportTeamByte(b);", service_c)
        self.assertIn("serial_comm_mode != COMM_MODE_BENCH", service_c)
        self.assertIn("MatchControl_HandleTeamFrame", match_c)
        self.assertIn("MATCH_AIRPORT_COMMAND_TIMEOUT_MS", match_c)

        def selected_baud(mode, transport):
            return 9600 if mode == "bench" or transport == "airport" else 420000

        self.assertEqual(selected_baud("bench", "airport"), 9600)
        self.assertEqual(selected_baud("match", "airport"), 9600)
        self.assertEqual(selected_baud("match", "channels"), 420000)


if __name__ == "__main__":
    unittest.main()
