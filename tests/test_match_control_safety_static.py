from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class MatchControlSafetyStaticTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.match_control = (ROOT / "Core/Src/match_control.c").read_text()
        cls.serial_service = (ROOT / "Core/Src/serial_service.c").read_text()
        cls.app_c_api = (ROOT / "Core/Src/app_c_api.cpp").read_text()
        cls.motor = (ROOT / "Core/Src/motor.cpp").read_text()

    def body_of(self, source, function_name):
        match = re.search(rf"void {function_name}\([^)]*\)\s*\{{(?P<body>.*?)\n\}}", source, re.S)
        self.assertIsNotNone(match, function_name)
        return match.group("body")

    def test_match_init_forces_safe_state(self):
        body = self.body_of(self.match_control, "MatchControl_Init")
        self.assertIn("has_match_command = 0U", body)
        self.assertIn("AppC_ForceSafeState();", body)

    def test_bench_to_match_reinitializes_match_and_forces_safe(self):
        body = self.body_of(self.serial_service, "SerialService_SetCommMode")
        self.assertIn("CrsfParser_Init(&crsf_parser);", body)
        self.assertIn("MatchControl_Init();", body)

    def test_uart_recovery_restarts_match_parser_safely(self):
        body = self.body_of(self.serial_service, "Serial_UartRecoveryTask")
        self.assertIn("serial_comm_mode == COMM_MODE_MATCH", body)
        self.assertIn("CrsfParser_Init(&crsf_parser);", body)
        self.assertIn("MatchControl_Init();", body)

    def test_no_crsf_received_keeps_safe(self):
        body = self.body_of(self.match_control, "MatchControl_Task")
        self.assertIn("if (has_match_command == 0U)", body)
        self.assertIn("AppC_ForceSafeState();", body)

    def test_watchdog_forces_safe_after_command_timeout(self):
        body = self.body_of(self.match_control, "MatchControl_Task")
        self.assertIn("command_timeout_ms", body)
        self.assertIn("match_dbg_watchdog_trips++", body)
        self.assertIn("AppC_ForceSafeState();", body)

    def test_zero_velocity_uses_linear_limit_unless_brake_is_explicit(self):
        body = self.body_of(self.app_c_api, "AppC_FastTick1kHz")
        self.assertIn("const bool braking = (stop_mode_brake != 0U);", body)
        self.assertNotIn("g_desiredVx == 0.0f", body)

    def test_motor_stop_keeps_distinct_coast_and_electrical_brake_outputs(self):
        body = re.search(r"void Motor::Stop\(bool brakeMode\) const\s*\{(?P<body>.*?)\n\}",
                         self.motor, re.S).group("body")
        self.assertIn("if (brakeMode)", body)
        self.assertIn("channelA_, kArr30kHz", body)
        self.assertIn("channelB_, kArr30kHz", body)
        self.assertIn("channelA_, 0", body)
        self.assertIn("channelB_, 0", body)

    def test_safe_state_still_requests_brake(self):
        body = self.body_of(self.app_c_api, "AppC_ForceSafeState")
        self.assertIn("EnterSafeState();", body)
        safe_body = re.search(r"void EnterSafeState\(\)\s*\{(?P<body>.*?)\n\}",
                              self.app_c_api, re.S).group("body")
        self.assertIn("stop_mode_brake = 1U", safe_body)
        self.assertIn("g_app->ForceSafeOutputs();", safe_body)


if __name__ == "__main__":
    unittest.main()
