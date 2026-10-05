import os
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "configurator"))

from core.protocol import Protocol
from core.radio_manager import RadioManager
from core.radio_profile import (NORMAL, RADIO_PROFILES, TEST_50_20,
                                VALIDATION_120, airport_ota_capacity_message,
                                estimate_protocol_traffic)
from core.radio_scheduler import RadioScheduler
from PySide6.QtWidgets import QApplication
from ui.main_window import MainWindow, SERIAL_BAUD_OPTIONS


def telemetry_frame(robot_id, sequence, flags):
    payload = bytearray(struct.pack(
        "<HBBBHBB", 0xAA55, Protocol.TELEMETRY_RESPONSE_TYPE,
        Protocol.TELEMETRY_VERSION, ord(robot_id), sequence, 1, flags))
    if flags & Protocol.TELEMETRY_FLAG_BASIC:
        payload.extend(struct.pack("<IBBB", 1000, 1, 0, 0))
    if flags & Protocol.TELEMETRY_FLAG_MOTORS:
        payload.extend(struct.pack("<hhhhhhhh", 10, 20, 30, 40, 1, 2, 3, 4))
    if flags & Protocol.TELEMETRY_FLAG_BATTERY:
        payload.extend(struct.pack("<HH", 12000, 2048))
    if flags & Protocol.TELEMETRY_FLAG_DIAGNOSTICS:
        payload.extend(struct.pack("<IIBI", 0, 10, 1, sequence))
    return Protocol.with_crc(payload)


class SimulatedClock:
    now_ns = 0

    def monotonic_ns(self):
        return self.now_ns

    def monotonic(self):
        return self.now_ns / 1_000_000_000


class ResponsiveFakeSerial:
    is_open = True

    def __init__(self, clock, latency_ns=5_000_000):
        self.clock = clock; self.latency_ns = latency_ns
        self.frames = []; self.pending = []; self.rx = bytearray()

    def write(self, frame):
        frame = bytes(frame); self.frames.append((self.clock.now_ns, frame))
        if frame[2] == Protocol.TELEMETRY_REQUEST_TYPE:
            robot_id = chr(frame[4]); sequence = struct.unpack_from("<H", frame, 5)[0]
            self.pending.append((self.clock.now_ns + self.latency_ns,
                                 telemetry_frame(robot_id, sequence, frame[7])))
        return len(frame)

    def _release(self):
        ready = [item for item in self.pending if item[0] <= self.clock.now_ns]
        self.pending = [item for item in self.pending if item[0] > self.clock.now_ns]
        for _, frame in ready: self.rx.extend(frame)

    @property
    def in_waiting(self):
        self._release(); return len(self.rx)

    def read(self, count):
        value = bytes(self.rx[:count]); del self.rx[:count]; return value


class RadioProfileAndSchedulerTests(unittest.TestCase):
    def test_profiles_preserve_normal_and_define_validation(self):
        self.assertEqual((NORMAL.command_hz, NORMAL.telemetry_fast_hz), (20, 5))
        self.assertTrue(NORMAL.telemetry_reply_guard)
        self.assertEqual((VALIDATION_120.command_hz, VALIDATION_120.telemetry_fast_hz,
                          VALIDATION_120.telemetry_full_hz), (120, 120, 10))
        self.assertFalse(VALIDATION_120.telemetry_reply_guard)

    def test_all_profiles_use_safe_airport_default_not_direct_uart_baud(self):
        self.assertTrue(all(profile.default_airport_baud == 9600
                            for profile in RADIO_PROFILES))
        self.assertTrue(all(profile.default_airport_baud != 921600
                            for profile in RADIO_PROFILES))

    def test_command_is_written_before_telemetry_when_both_are_due(self):
        clock = SimulatedClock(); radio = RadioManager(); serial = ResponsiveFakeSerial(clock)
        radio._serial = serial; radio.state.robot_id = "A"; radio.state.connected = True
        robot = radio.robots.ensure_robot("A"); robot.discovered = robot.connected = True
        radio.select_robot("A")
        with patch("core.serial_manager.time.monotonic_ns", clock.monotonic_ns), \
             patch("core.serial_manager.time.monotonic", clock.monotonic), \
             patch("core.radio_manager.time.monotonic_ns", clock.monotonic_ns):
            radio.set_radio_profile(TEST_50_20)
            radio.scheduler.next_command_ns = radio.scheduler.next_telemetry_ns = 0
            radio._tick()
        self.assertEqual([frame[2] for _, frame in serial.frames[:2]],
                         [Protocol.TEAM_VELOCITY_TYPE, Protocol.TELEMETRY_REQUEST_TYPE])

    def test_normal_telemetry_does_not_guard_airport_team_command_deadline(self):
        clock = SimulatedClock(); radio = RadioManager(); serial = ResponsiveFakeSerial(clock)
        radio._serial = serial; radio.state.connected = True
        radio.robots.register_expected_robots(["A"]); radio.select_robot("A")
        with patch("core.serial_manager.time.monotonic_ns", clock.monotonic_ns), \
             patch("core.serial_manager.time.monotonic", clock.monotonic), \
             patch("core.radio_manager.time.monotonic_ns", clock.monotonic_ns):
            radio.set_radio_profile(NORMAL)
            for now_ns in range(0, 301_000_000, 1_000_000):
                clock.now_ns = now_ns; radio._tick()
        command_times = [timestamp for timestamp, frame in serial.frames
                         if frame[2] == Protocol.TEAM_VELOCITY_TYPE]
        self.assertEqual(command_times, list(range(0, 301_000_000, 50_000_000)))
        self.assertEqual(radio.link_stats.max_command_tx_gap_ms, 50.0)
        self.assertEqual(radio.link_stats.command_frames_sent, len(command_times))

    def test_120_hz_accumulative_deadlines_do_not_drift(self):
        scheduler = RadioScheduler(VALIDATION_120); scheduler.reset(0)
        sends = 0
        for now_ns in range(0, 10_000_000_000, 1_000_000):
            sends += scheduler.take_due(now_ns).command
        self.assertEqual(sends, 1200)
        self.assertEqual(scheduler.next_command_ns, 1200 * scheduler.command_period_ns)

    def test_delay_sends_one_sample_and_counts_missed_deadlines(self):
        scheduler = RadioScheduler(VALIDATION_120); scheduler.reset(0)
        first = scheduler.take_due(40_000_000)
        self.assertTrue(first.command); self.assertEqual(first.command_missed, 4)
        self.assertTrue(first.telemetry); self.assertEqual(first.telemetry_missed, 4)
        self.assertFalse(scheduler.take_due(40_000_000).command)

    def test_full_replaces_fast_and_is_periodic_per_robot(self):
        clock = SimulatedClock(); radio = RadioManager(); radio._serial = ResponsiveFakeSerial(clock)
        radio.state.robot_id = "A"
        robot = radio.robots.ensure_robot("A"); robot.discovered = True
        radio.robots.set_uplink_robot("A", 0)
        with patch("core.serial_manager.time.monotonic_ns", clock.monotonic_ns), \
             patch("core.radio_manager.time.monotonic_ns", clock.monotonic_ns):
            radio.set_radio_profile(VALIDATION_120)
            self.assertEqual(radio._telemetry_flags_for_next_request(0), Protocol.TELEMETRY_FLAGS_FULL)
            radio.send_telemetry_request(Protocol.TELEMETRY_FLAGS_FULL)
            self.assertEqual(radio._telemetry_flags_for_next_request(50_000_000), Protocol.TELEMETRY_FLAGS_FAST)
            self.assertEqual(radio._telemetry_flags_for_next_request(100_000_000), Protocol.TELEMETRY_FLAGS_FULL)
        requests = [frame for _, frame in radio._serial.frames if frame[2] == Protocol.TELEMETRY_REQUEST_TYPE]
        self.assertEqual(len(requests), 1)

    def test_sequence_wraps_as_uint16(self):
        clock = SimulatedClock(); radio = RadioManager(); radio._serial = ResponsiveFakeSerial(clock)
        radio.state.robot_id = "A"; radio._request_sequence = 0xFFFF
        with patch("core.radio_manager.time.monotonic_ns", clock.monotonic_ns), \
             patch("core.serial_manager.time.monotonic_ns", clock.monotonic_ns):
            radio.send_telemetry_request(Protocol.TELEMETRY_FLAGS_FAST)
        self.assertEqual(radio._request_sequence, 0)
        self.assertIn(0, radio._telemetry_requests)

    def test_airport_and_direct_debug_bauds_are_available(self):
        for baud in (9600, 14400, 19200, 38400, 115200, 921600, 1000000):
            self.assertIn(baud, SERIAL_BAUD_OPTIONS)

    def test_protocol_traffic_estimate_uses_actual_frame_sizes(self):
        normal = estimate_protocol_traffic(NORMAL)
        validation = estimate_protocol_traffic(VALIDATION_120)
        self.assertEqual((normal.command_frame_bytes,
                          normal.telemetry_request_frame_bytes,
                          normal.telemetry_fast_response_bytes,
                          normal.telemetry_full_response_bytes), (32, 10, 34, 51))
        self.assertEqual((normal.estimated_protocol_tx_bytes_per_s,
                          normal.estimated_protocol_rx_bytes_per_s), (690, 255))
        self.assertEqual((validation.estimated_protocol_tx_bytes_per_s,
                          validation.estimated_protocol_rx_bytes_per_s), (5040, 4250))
        self.assertIn("capacity unknown", airport_ota_capacity_message(NORMAL))
        self.assertIn("exceeds", airport_ota_capacity_message(VALIDATION_120, 1000))

    def test_profile_selection_does_not_change_serial_baud_and_ui_separates_rates(self):
        app = QApplication.instance() or QApplication([])
        window = MainWindow(SimpleNamespace(port=None, baud=9600))
        try:
            self.assertEqual(window.header_profile.currentData(), "NORMAL")
            window.header_baud.setCurrentText("921600")
            window.header_profile.setCurrentIndex(4)
            self.assertEqual(window.manager.profile, VALIDATION_120)
            self.assertEqual(window.header_baud.currentText(), "921600")
            window.update_link_metrics()
            diagnostics = window.diagnostics.link_metrics.text()
            self.assertIn("Transport: AirPort · Serial baud: 921600", diagnostics)
            self.assertIn("Command — Target: 120 Hz · Measured:", diagnostics)
            self.assertIn("Telemetry — Target: 120 Hz · Requests:", diagnostics)
            self.assertIn("AirPort OTA capacity unknown", diagnostics)
        finally:
            window.close()


class RadioFakeSerialIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.clock = SimulatedClock(); self.radio = RadioManager()
        self.serial = ResponsiveFakeSerial(self.clock); self.radio._serial = self.serial
        self.radio.state.robot_id = "A"; self.radio.state.connected = True
        robot = self.radio.robots.ensure_robot("A")
        robot.discovered = robot.connected = True; robot.status = "ONLINE"
        self.radio.select_robot("A")

    def test_validation_120_for_five_simulated_seconds(self):
        with patch("core.serial_manager.time.monotonic_ns", self.clock.monotonic_ns), \
             patch("core.serial_manager.time.monotonic", self.clock.monotonic), \
             patch("core.radio_manager.time.monotonic_ns", self.clock.monotonic_ns):
            self.radio.set_radio_profile(VALIDATION_120)
            for now_ns in range(0, 5_000_000_000, 1_000_000):
                self.clock.now_ns = now_ns; self.radio._tick()
            snapshot = self.radio.link_stats.snapshot(self.clock.now_ns)
        self.assertAlmostEqual(snapshot.command_tx_hz, 120, delta=1)
        self.assertAlmostEqual(snapshot.telemetry_request_tx_hz, 120, delta=1)
        self.assertAlmostEqual(snapshot.telemetry_response_rx_hz, 120, delta=2)
        self.assertLessEqual(len(self.radio._telemetry_requests), 2)
        self.assertEqual(snapshot.command_deadlines_missed, 0)
        self.assertAlmostEqual(snapshot.latency_mean_ms, 5.0, delta=1.0)
        request_frames = [frame for _, frame in self.serial.frames
                          if frame[2] == Protocol.TELEMETRY_REQUEST_TYPE]
        full_count = sum(frame[7] == Protocol.TELEMETRY_FLAGS_FULL for frame in request_frames)
        self.assertAlmostEqual(full_count / 5, 10, delta=1)
        kinds = [frame[2] for timestamp, frame in self.serial.frames if timestamp == 0]
        self.assertTrue(not kinds or kinds[0] == Protocol.TEAM_VELOCITY_TYPE)

    def test_timeout_and_unmatched_late_response_are_counted(self):
        self.serial.latency_ns = 200_000_000
        with patch("core.serial_manager.time.monotonic_ns", self.clock.monotonic_ns), \
             patch("core.serial_manager.time.monotonic", self.clock.monotonic), \
             patch("core.radio_manager.time.monotonic_ns", self.clock.monotonic_ns):
            self.radio.set_radio_profile(TEST_50_20)
            for now_ns in range(0, 400_000_000, 1_000_000):
                self.clock.now_ns = now_ns; self.radio._tick()
            snapshot = self.radio.link_stats.snapshot(self.clock.now_ns)
        self.assertEqual(snapshot.telemetry_timed_out_requests, 0)
        self.assertGreater(snapshot.telemetry_probe_timeouts, 0)
        self.assertGreater(snapshot.telemetry_unmatched_responses, 0)
        self.assertEqual(snapshot.telemetry_response_loss_percent, 0)

    def test_polling_locks_to_the_uplink_robot_with_two_registered(self):
        robot_b = self.radio.robots.ensure_robot("B")
        robot_b.discovered = robot_b.connected = True; robot_b.status = "ONLINE"
        with patch("core.serial_manager.time.monotonic_ns", self.clock.monotonic_ns), \
             patch("core.serial_manager.time.monotonic", self.clock.monotonic), \
             patch("core.radio_manager.time.monotonic_ns", self.clock.monotonic_ns):
            self.radio.set_radio_profile(VALIDATION_120)
            for now_ns in range(0, 1_000_000_000, 1_000_000):
                self.clock.now_ns = now_ns; self.radio._tick()
        requests = [frame for _, frame in self.serial.frames
                    if frame[2] == Protocol.TELEMETRY_REQUEST_TYPE]
        targets = [chr(frame[4]) for frame in requests]
        self.assertAlmostEqual(len(requests), 120, delta=1)
        self.assertEqual(set(targets), {"A"})
        self.assertEqual(self.radio.robots.uplink_robot_id, "A")

    def test_profile_change_resets_deadlines_without_guarding_team_commands(self):
        with patch("core.serial_manager.time.monotonic_ns", self.clock.monotonic_ns), \
             patch("core.serial_manager.time.monotonic", self.clock.monotonic), \
             patch("core.radio_manager.time.monotonic_ns", self.clock.monotonic_ns):
            self.radio.set_radio_profile(NORMAL); self.radio._tick()
            self.clock.now_ns = 100_000_000; self.radio._tick()
            self.assertEqual(self.radio.scheduler.next_command_ns, 150_000_000)
            self.clock.now_ns = 1_000_000_000
            self.radio.set_radio_profile(VALIDATION_120); self.radio._tick()
            self.assertLess(self.radio.scheduler.next_command_ns, 1_020_000_000)


if __name__ == "__main__":
    unittest.main()
