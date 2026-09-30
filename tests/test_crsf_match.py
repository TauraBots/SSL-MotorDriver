import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "crsf-host"))

from crsf_match import (  # noqa: E402
    ACTION_BRAKE,
    ACTION_DRIBBLER,
    ACTION_KICK,
    ActionEdgeFilter,
    CRSF_CHANNEL_CENTER,
    CRSF_CHANNEL_MAX,
    CRSF_CHANNEL_MIN,
    CRSF_HOST_TO_TX_MODULE_ADDRESS,
    CRSF_RX_TO_STM32_SYNC_ADDRESS,
    RobotCommand,
    TeamFrameAssembler,
    TeamFrameFragment,
    crc8_simple,
    decode_robot_from_channels,
    encode_rc_channels_frame,
    encode_team_channels,
    make_match_frame,
    parse_rc_channels_stream,
    quantize_signed,
    unpack_channels,
)
from crsf_bridge import CommandSource, build_arg_parser  # noqa: E402


class CrsfMatchTests(unittest.TestCase):
    def test_crc_valid_invalid_truncated_noise_and_concatenated_frames(self):
        channels = [CRSF_CHANNEL_CENTER] * 16
        channels[0] = quantize_signed(1.0, 2.5)
        frame = encode_rc_channels_frame(channels)
        rx = bytearray(b"\x01\x02garbage" + frame + frame)
        parsed = parse_rc_channels_stream(rx)
        self.assertEqual(len(parsed), 2)
        self.assertAlmostEqual(parsed[0][0], channels[0], delta=0)

        bad = bytearray(frame)
        bad[-1] ^= 0x55
        rx = bytearray(bad + frame[:6])
        self.assertEqual(parse_rc_channels_stream(rx), [])
        self.assertEqual(bytes(rx), frame[:6])

    def test_directional_crsf_addresses_are_explicit(self):
        channels = [CRSF_CHANNEL_CENTER] * 16
        host_frame = encode_rc_channels_frame(channels)
        rx_frame = encode_rc_channels_frame(channels, address=CRSF_RX_TO_STM32_SYNC_ADDRESS)
        self.assertEqual(host_frame[0], CRSF_HOST_TO_TX_MODULE_ADDRESS)
        self.assertEqual(rx_frame[0], CRSF_RX_TO_STM32_SYNC_ADDRESS)
        self.assertEqual(len(parse_rc_channels_stream(bytearray(host_frame + rx_frame))), 2)

    def test_robot_mapping_allows_different_commands_and_negative_values(self):
        channels = encode_team_channels({
            "A": RobotCommand(vx=1.0, vy=0.0, omega=0.0),
            "B": RobotCommand(vx=0.0, vy=-1.0, omega=0.0),
            "C": RobotCommand(vx=0.0, vy=0.0, omega=1.0),
        })
        a, _ = decode_robot_from_channels(channels, "A")
        b, _ = decode_robot_from_channels(channels, "B")
        c, _ = decode_robot_from_channels(channels, "C")
        self.assertAlmostEqual(a.vx, 1.0, delta=0.01)
        self.assertAlmostEqual(b.vy, -1.0, delta=0.01)
        self.assertAlmostEqual(c.omega, 1.0, delta=0.02)

    def test_saturation_and_enable_off_safe_command(self):
        channels = encode_team_channels({"A": RobotCommand(vx=99, vy=-99, omega=99)}, enabled=True)
        a, _ = decode_robot_from_channels(channels, "A")
        self.assertAlmostEqual(a.vx, 2.5, delta=0.01)
        self.assertAlmostEqual(a.vy, -2.5, delta=0.01)
        self.assertAlmostEqual(a.omega, 8.0, delta=0.02)

        channels[15] = CRSF_CHANNEL_MIN
        safe, _ = decode_robot_from_channels(channels, "A")
        self.assertEqual((safe.vx, safe.vy, safe.omega, safe.kick, safe.dribbler, safe.brake),
                         (0.0, 0.0, 0.0, False, False, True))

    def test_kick_occurs_once_while_action_bit_stays_high(self):
        command = RobotCommand(kick=True, kick_power=80, dribbler=True, brake=True)
        channels = encode_team_channels({"A": command})
        low_channels = encode_team_channels({"A": RobotCommand(kick=False)})
        _, low_action = decode_robot_from_channels(low_channels, "A", previous_action_bits=0)
        first, action = decode_robot_from_channels(channels, "A", previous_action_bits=low_action)
        second, _ = decode_robot_from_channels(channels, "A", previous_action_bits=action)
        self.assertTrue(first.kick)
        self.assertEqual(first.kick_power, 80)
        self.assertFalse(second.kick)
        self.assertEqual(second.kick_power, 0)
        self.assertTrue(action & ACTION_KICK)
        self.assertTrue(action & ACTION_BRAKE)
        self.assertTrue(action & ACTION_DRIBBLER)

    def test_kick_requires_observed_low_baseline_after_reconnect(self):
        edge = ActionEdgeFilter()
        high = encode_team_channels({"A": RobotCommand(kick=True, kick_power=80)})
        low = encode_team_channels({"A": RobotCommand(kick=False)})
        self.assertFalse(edge.decode(high, "A").kick)
        self.assertFalse(edge.decode(low, "A").kick)
        self.assertTrue(edge.decode(high, "A").kick)

    def test_match_frame_roundtrip(self):
        frame = make_match_frame({"C": RobotCommand(omega=1.0)}, enabled=True)
        parsed = parse_rc_channels_stream(bytearray(frame))
        self.assertEqual(len(parsed), 1)
        channels = unpack_channels(frame[3:-1])
        self.assertEqual(parsed[0], channels)

    def test_bridge_starts_disabled_before_test_enable(self):
        args = build_arg_parser().parse_args(["--port", "dummy", "--test"])
        source = CommandSource(args)
        safe = parse_rc_channels_stream(bytearray(source.startup_safe_frame()))[0]
        active = parse_rc_channels_stream(bytearray(source.active_frame()))[0]
        self.assertLessEqual(safe[15], CRSF_CHANNEL_CENTER)
        self.assertGreater(active[15], CRSF_CHANNEL_CENTER)


class TeamFrameTests(unittest.TestCase):
    def make_frame(self):
        payload = bytes(range(15))
        return payload + bytes([crc8_simple(payload)])

    def test_four_fragments_commit_atomically_out_of_order(self):
        frame = self.make_frame()
        assembler = TeamFrameAssembler()
        self.assertIsNone(assembler.push(TeamFrameFragment(7, 2, frame[8:12])))
        self.assertIsNone(assembler.push(TeamFrameFragment(7, 0, frame[0:4])))
        self.assertIsNone(assembler.push(TeamFrameFragment(7, 3, frame[12:16])))
        self.assertEqual(assembler.push(TeamFrameFragment(7, 1, frame[4:8])), frame)

    def test_missing_duplicate_mixed_sequence_and_bad_crc_do_not_commit(self):
        frame = self.make_frame()
        assembler = TeamFrameAssembler()
        self.assertIsNone(assembler.push(TeamFrameFragment(1, 0, frame[0:4])))
        self.assertIsNone(assembler.push(TeamFrameFragment(1, 0, frame[0:4])))
        self.assertIsNone(assembler.push(TeamFrameFragment(2, 1, frame[4:8])))
        self.assertIsNone(assembler.push(TeamFrameFragment(2, 2, frame[8:12])))
        self.assertIsNone(assembler.push(TeamFrameFragment(2, 3, frame[12:16])))
        bad = bytearray(frame)
        bad[-1] ^= 1
        assembler = TeamFrameAssembler()
        for part in range(3):
            self.assertIsNone(assembler.push(TeamFrameFragment(3, part, bad[part * 4:(part + 1) * 4])))
        self.assertIsNone(assembler.push(TeamFrameFragment(3, 3, bad[12:16])))


if __name__ == "__main__":
    unittest.main()
