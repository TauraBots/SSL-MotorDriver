import math
from pathlib import Path
import struct
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOF = b"\x55\xAA"
F0, F1, F2, F3, F4, F5 = range(0xF0, 0xF6)
E0, E1, E2, E3 = range(0xE0, 0xE4)
D1 = 0xD1
CONFIG_KEY = 0x46434449
LENGTHS = {D1: 32, E0: 10, E2: 8, F0: 9, F1: 22, F4: 29}
UID = bytes.fromhex("35FEDC054E58383412781043")


def crc16(data):
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def with_crc(payload):
    return payload + struct.pack("<H", crc16(payload))


def config_discover(nonce=7):
    return with_crc(struct.pack("<HBI", 0xAA55, F0, nonce))


def config_set_id(uid=UID, robot_id="B", key=CONFIG_KEY):
    return with_crc(struct.pack("<HB12sBI", 0xAA55, F1, uid, ord(robot_id), key))


def config_set_motion(uid=UID, linear=4.0, angular=10.0, key=CONFIG_KEY):
    return with_crc(struct.pack("<HB12sffI", 0xAA55, F4, uid, linear, angular, key))


def team_frame(sequence=1):
    payload = bytearray(SOF + bytes((D1, 1)))
    payload.extend(struct.pack("<H", sequence))
    payload.extend(bytes(24))
    return with_crc(bytes(payload))


def telemetry_request(robot_id="A", sequence=1):
    return with_crc(struct.pack("<HBBBHB", 0xAA55, E0, 1, ord(robot_id), sequence, 0x0F))


def discovery_request(sequence=1):
    return with_crc(struct.pack("<HBBH", 0xAA55, E2, 1, sequence))


class MatchAirportConfigModel:
    """Executable specification for the existing F0/F1/F4 service handlers."""

    def __init__(self):
        self.uid = UID
        self.robot_id = "A"
        self.linear = 4.0
        self.angular = 10.0
        self.safe_count = 0
        self.pending = None
        self.tx = []
        self.events = []

    @staticmethod
    def tx_allowed(packet_type, length):
        return ((packet_type == E1 and 11 <= length <= 51) or
                (packet_type == E3 and length == 27) or
                (packet_type in (F2, F3, F5) and length == 23))

    def process(self, frame):
        if len(frame) != LENGTHS.get(frame[2], -1):
            return
        if struct.unpack_from("<H", frame, len(frame) - 2)[0] != crc16(frame[:-2]):
            return
        packet_type = frame[2]
        self.events.append(packet_type)
        if packet_type == F0:
            nonce = struct.unpack_from("<I", frame, 3)[0]
            slot_hash = nonce ^ 0x9E3779B9
            for value in self.uid:
                slot_hash ^= value
                slot_hash = (slot_hash * 0x85EBCA6B) & 0xFFFFFFFF
                slot_hash ^= slot_hash >> 13
            self.pending = (F2, (slot_hash & 0x1F) * 30)
        elif packet_type == F1:
            uid, robot_id, key = struct.unpack_from("<12sBI", frame, 3)
            if uid != self.uid or key != CONFIG_KEY:
                return
            self.safe_count += 1
            status = ord("A") <= robot_id <= ord("Z")
            if status:
                self.robot_id = chr(robot_id)
            self.pending = (F3, 0)
        elif packet_type == F4:
            uid, linear, angular, key = struct.unpack_from("<12sffI", frame, 3)
            if uid != self.uid or key != CONFIG_KEY:
                return
            self.safe_count += 1
            status = (math.isfinite(linear) and math.isfinite(angular) and
                      0.1 <= linear <= 20.0 and 0.1 <= angular <= 50.0)
            if status:
                self.linear, self.angular = linear, angular
            self.pending = (F5, 0)

    def task(self):
        if self.pending is None:
            return
        packet_type, _ = self.pending
        if self.tx_allowed(packet_type, 23):
            self.tx.append(packet_type)
            self.pending = None


class MatchAirportParserModel:
    def __init__(self, service):
        self.service = service
        self.frame = bytearray()
        self.expected = 0

    def feed(self, data):
        for value in data:
            if not self.frame:
                if value == 0x55:
                    self.frame.append(value)
                continue
            if len(self.frame) == 1:
                if value == 0xAA:
                    self.frame.append(value)
                elif value != 0x55:
                    self.frame.clear()
                continue
            if len(self.frame) == 2:
                expected = LENGTHS.get(value)
                if expected is None:
                    self.frame.clear()
                    if value == 0x55:
                        self.frame.append(value)
                    continue
                self.frame.append(value)
                self.expected = expected
                continue
            self.frame.append(value)
            if len(self.frame) == self.expected:
                self.service.process(bytes(self.frame))
                self.frame.clear()
                self.expected = 0


class AirportMatchConfigTests(unittest.TestCase):
    def setUp(self):
        self.service = MatchAirportConfigModel()
        self.parser = MatchAirportParserModel(self.service)

    def test_f0_valid_schedules_f2_and_invalid_crc_is_ignored(self):
        invalid = bytearray(config_discover()); invalid[-1] ^= 1
        self.parser.feed(invalid)
        self.assertIsNone(self.service.pending)
        self.parser.feed(config_discover(0x12345678))
        self.assertEqual(self.service.pending[0], F2)
        self.assertIn(self.service.pending[1], range(0, 931, 30))
        self.service.task()
        self.assertEqual(self.service.tx, [F2])

    def test_f1_wrong_uid_is_ignored(self):
        self.parser.feed(config_set_id(bytes(12), "B"))
        self.assertEqual(self.service.robot_id, "A")
        self.assertEqual(self.service.safe_count, 0)
        self.assertIsNone(self.service.pending)

    def test_f1_correct_uid_forces_safe_sets_id_and_generates_f3(self):
        self.parser.feed(config_set_id(robot_id="B"))
        self.assertEqual(self.service.robot_id, "B")
        self.assertEqual(self.service.safe_count, 1)
        self.service.task()
        self.assertEqual(self.service.tx, [F3])

    def test_f1_invalid_id_does_not_change_identity(self):
        self.parser.feed(config_set_id(robot_id="*"))
        self.assertEqual(self.service.robot_id, "A")
        self.assertEqual(self.service.safe_count, 1)

    def test_f4_correct_uid_forces_safe_sets_limits_and_generates_f5(self):
        self.parser.feed(config_set_motion(linear=6.5, angular=12.0))
        self.assertEqual((self.service.linear, self.service.angular), (6.5, 12.0))
        self.assertEqual(self.service.safe_count, 1)
        self.service.task()
        self.assertEqual(self.service.tx, [F5])

    def test_f4_invalid_limits_do_not_change_limits(self):
        self.parser.feed(config_set_motion(linear=0.0, angular=100.0))
        self.assertEqual((self.service.linear, self.service.angular), (4.0, 10.0))
        self.assertEqual(self.service.safe_count, 1)

    def test_match_tx_whitelist_is_exact(self):
        valid_lengths = {E1: 11, E3: 27, F2: 23, F3: 23, F5: 23}
        allowed = {value for value in range(256)
                   if self.service.tx_allowed(value, valid_lengths.get(value, 23))}
        self.assertEqual(allowed, {E1, E3, F2, F3, F5})
        self.assertFalse(self.service.tx_allowed(D1, 32))
        self.assertFalse(self.service.tx_allowed(0xD0, 19))
        self.assertFalse(self.service.tx_allowed(F0, 9))

    def test_mixed_d1_e0_f0_d1_stream_keeps_synchronization(self):
        stream = team_frame(1) + telemetry_request() + config_discover() + team_frame(2)
        self.parser.feed(stream)
        self.assertEqual(self.service.events, [D1, E0, F0, D1])

    def test_e2_and_parser_resynchronization_remain_available(self):
        self.parser.feed(b"noise\x55\xAA\x99" + discovery_request())
        self.assertEqual(self.service.events, [E2])

    def test_firmware_routes_config_through_match_parser_and_dma_queue(self):
        source = (ROOT / "Core/Src/serial_service.c").read_text(encoding="utf-8")
        for assignment in (
                "team_rx_expected_len = CONFIG_DISCOVER_PACKET_LEN",
                "team_rx_expected_len = CONFIG_SET_ID_PACKET_LEN",
                "team_rx_expected_len = CONFIG_SET_MOTION_PACKET_LEN"):
            self.assertIn(assignment, source)
        for call in ("Serial_ProcessDiscoverPacket(team_rx_frame);",
                     "Serial_ProcessSetIdPacket(team_rx_frame);",
                     "Serial_ProcessSetMotionPacket(team_rx_frame);"):
            self.assertIn(call, source)
        self.assertIn("return Serial_QueueTx(response, sizeof(response), 0U);", source)
        self.assertIn("Serial_ConfigResponseTask();\n      Serial_DiscoveryTask();", source)
        self.assertIn("AppC_ForceSafeState();", source)
        self.assertNotIn("HAL_UART_Transmit(serial_uart, response", source)

    def test_firmware_match_tx_guard_contains_only_response_types(self):
        source = (ROOT / "Core/Src/serial_service.c").read_text(encoding="utf-8")
        start = source.index(
            "static uint8_t Serial_TxAllowed(const uint8_t *data, uint16_t len)\n{")
        end = source.index("static uint8_t Serial_QueueTx", start)
        guard = source[start:end]
        for packet_type in ("TX_TYPE_TELEMETRY_RESPONSE",
                            "TX_TYPE_DISCOVERY_RESPONSE",
                            "TX_TYPE_CONFIG_DISCOVER_RESPONSE",
                            "TX_TYPE_CONFIG_SET_ID_RESPONSE",
                            "TX_TYPE_CONFIG_SET_MOTION_RESPONSE"):
            self.assertIn(packet_type, guard)
        for packet_type in ("RX_TYPE_TEAM_VELOCITY", "RX_TYPE_ROBOT_VELOCITY",
                            "RX_TYPE_CONFIG_DISCOVER", "RX_TYPE_CONFIG_SET_ID",
                            "RX_TYPE_CONFIG_SET_MOTION"):
            self.assertNotIn(packet_type, guard)

    def test_match_has_no_ascii_startup_and_exposes_debug_counters(self):
        source = (ROOT / "Core/Src/serial_service.c").read_text(encoding="utf-8")
        header = (ROOT / "Core/Inc/serial_service.h").read_text(encoding="utf-8")
        self.assertIn("if (serial_comm_mode == COMM_MODE_BENCH)", source)
        for name in ("match_config_discover_requests", "match_config_discover_responses",
                     "match_config_set_id_requests", "match_config_set_id_responses",
                     "match_config_motion_requests", "match_config_motion_responses"):
            self.assertIn(f"volatile uint32_t {name}", source)
            self.assertIn(f"extern volatile uint32_t {name}", header)

    def test_configurator_shows_airport_service_warning(self):
        source = (ROOT / "configurator/ui/config_panel.py").read_text(encoding="utf-8")
        self.assertIn("mantenha apenas o", source)
        self.assertIn("robô/RX alvo ativo no uplink", source)


if __name__ == "__main__":
    unittest.main()
