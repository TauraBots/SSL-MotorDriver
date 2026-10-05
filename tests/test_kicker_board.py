from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class KickerBoardStaticTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.header = (ROOT / "Core/Inc/kicker_board.h").read_text()
        cls.driver = (ROOT / "Core/Src/kicker_board.c").read_text()
        cls.app = (ROOT / "Core/Src/app_c_api.cpp").read_text()
        cls.main = (ROOT / "Core/Src/main.c").read_text()

    def function_body(self, source, name):
        start = re.search(rf"\b{name}\([^)]*\)\s*\{{", source)
        self.assertIsNotNone(start, name)
        opening = source.index("{", start.start())
        depth = 0
        for index in range(opening, len(source)):
            if source[index] == "{":
                depth += 1
            elif source[index] == "}":
                depth -= 1
                if depth == 0:
                    return source[opening + 1:index]
        self.fail(f"unterminated body: {name}")

    def test_i2c_uses_shifted_7_bit_address(self):
        self.assertIn("#define KICKER_BOARD_I2C_ADDRESS_7BIT 0x42U", self.header)
        self.assertIn("(KICKER_BOARD_I2C_ADDRESS_7BIT << 1U)", self.header)
        self.assertIn("HAL_I2C_Master_Transmit(kicker_i2c, KICKER_BOARD_I2C_ADDRESS", self.driver)
        self.assertIn("HAL_I2C_Master_Receive(kicker_i2c, KICKER_BOARD_I2C_ADDRESS", self.driver)

    def test_tx_and_status_timeouts_are_separate(self):
        self.assertIn("#define KICKER_I2C_TX_TIMEOUT_MS 2U", self.driver)
        self.assertIn("#define KICKER_I2C_STATUS_TIMEOUT_MS 10U", self.driver)
        send = self.function_body(self.driver, "SendCommand")
        read = self.function_body(self.driver, "KickerBoard_ReadStatus")
        self.assertIn("KICKER_I2C_TX_TIMEOUT_MS", send)
        self.assertIn("KICKER_I2C_STATUS_TIMEOUT_MS", read)

    def test_simple_command_opcodes(self):
        self.assertIn("KICKER_COMMAND_START_CHARGE 0x01U", self.driver)
        self.assertIn("KICKER_COMMAND_STOP_CHARGE 0x02U", self.driver)
        self.assertIn("KICKER_COMMAND_IMMEDIATE_KICK 0x03U", self.driver)

    def test_setpoint_is_range_checked_and_little_endian(self):
        body = self.function_body(self.driver, "KickerBoard_SetVoltage")
        self.assertIn("(decivolt < 200U) || (decivolt > 2000U)", body)
        self.assertIn("command[0] = KICKER_COMMAND_SET_VOLTAGE", body)
        self.assertIn("command[1] = (uint8_t)(decivolt & 0xFFU)", body)
        self.assertIn("command[2] = (uint8_t)(decivolt >> 8U)", body)
        value = 1500
        self.assertEqual([0x04, value & 0xFF, value >> 8], [0x04, 0xDC, 0x05])

    def test_auto_kick_range_and_direct_percent_mapping(self):
        request = self.function_body(self.driver, "KickerBoard_RequestAutoKick")
        task = self.function_body(self.driver, "KickerBoard_Task")
        self.assertIn("(percent < 1U) || (percent > 100U)", request)
        self.assertIn("pending_auto_percent = percent", request)
        self.assertIn("{KICKER_COMMAND_AUTO_KICK, pending_auto_percent}", task)
        self.assertEqual([0x05, 80], [0x05, 0x50])

    def test_crc8_known_vector(self):
        crc = 0
        for value in b"123456789":
            crc ^= value
            for _ in range(8):
                crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        self.assertEqual(crc, 0xF4)
        crc_body = self.function_body(self.driver, "KickerBoard_Crc8")
        self.assertIn("crc = 0U", crc_body)
        self.assertIn("^ 0x07U", crc_body)
        self.assertIn("crc & 0x80U", crc_body)

    def test_status_validation_and_little_endian_decode(self):
        body = self.function_body(self.driver, "KickerBoard_ReadStatus")
        self.assertIn("buffer[0] != KICKER_STATUS_HEADER", body)
        self.assertIn("buffer[1] != KICKER_STATUS_VERSION", body)
        self.assertIn("KickerBoard_Crc8(buffer, KICKER_BOARD_STATUS_SIZE - 1U) != buffer[13]", body)
        for offset in (4, 6, 8, 10):
            self.assertIn(f"ReadLe16(&buffer[{offset}])", body)

    def test_kick_transmit_has_no_retry_path(self):
        body = self.function_body(self.driver, "KickerBoard_Task")
        auto_start = body.index("if (kicker_state == KICKER_STATE_AUTO_KICK_PENDING)")
        immediate_start = body.index("if (kicker_state == KICKER_STATE_IMMEDIATE_KICK_PENDING)")
        safe_start = body.index("if (kicker_state == KICKER_STATE_SAFE_STOP_PENDING)")
        auto_kick = body[auto_start:immediate_start]
        immediate_kick = body[immediate_start:safe_start]
        self.assertEqual(auto_kick.count("SendCommand(command, sizeof(command))"), 1)
        self.assertEqual(immediate_kick.count("SendCommand(&command, 1U)"), 1)
        self.assertIn("kicker_state = KICKER_STATE_WAITING_STATUS", auto_kick)
        self.assertIn("kicker_state = KICKER_STATE_WAITING_STATUS", immediate_kick)
        self.assertNotIn("kicker_state = KICKER_STATE_AUTO_KICK_PENDING", auto_kick)
        self.assertNotIn("kicker_state = KICKER_STATE_IMMEDIATE_KICK_PENDING", immediate_kick)

    def test_unknown_state_blocks_next_kick_until_valid_status(self):
        request = self.function_body(self.driver, "KickerBoard_RequestAutoKick")
        task = self.function_body(self.driver, "KickerBoard_Task")
        self.assertIn("kicker_state != KICKER_STATE_IDLE", request)
        self.assertIn("KICKER_STATE_WAITING_STATUS", task)
        self.assertIn("KICKER_STATE_ERROR", task)
        self.assertIn("KICKER_STATE_IDLE : KICKER_STATE_ERROR", task)

    def test_execution_debug_counters(self):
        init = self.function_body(self.driver, "KickerBoard_Init")
        task = self.function_body(self.driver, "KickerBoard_Task")
        read = self.function_body(self.driver, "KickerBoard_ReadStatus")
        self.assertIn("volatile uint32_t kicker_dbg_task_calls", self.driver)
        self.assertIn("volatile uint32_t kicker_dbg_status_reads", self.driver)
        self.assertIn("kicker_dbg_task_calls = 0U", init)
        self.assertIn("kicker_dbg_status_reads = 0U", init)
        self.assertLess(task.index("kicker_dbg_task_calls++"), task.index("HAL_GetTick()"))
        self.assertLess(read.index("kicker_dbg_status_reads++"),
                        read.index("HAL_I2C_Master_Receive"))

    def test_safe_stop_has_priority_from_every_state(self):
        request = self.function_body(self.driver, "KickerBoard_RequestSafeStop")
        self.assertIn("pending_auto_percent = 0U", request)
        self.assertIn("kicker_state = KICKER_STATE_SAFE_STOP_PENDING", request)
        self.assertNotIn("kicker_state != KICKER_STATE_IDLE", request)
        self.assertNotIn("KICKER_STATE_WAITING_STATUS", request)
        self.assertNotIn("KICKER_STATE_ERROR", request)

    def test_safe_stop_task_sends_only_stop_once(self):
        task = self.function_body(self.driver, "KickerBoard_Task")
        start = task.index("if (kicker_state == KICKER_STATE_SAFE_STOP_PENDING)")
        end = task.index("if (kicker_state == KICKER_STATE_WAITING_STATUS)", start)
        safe_stop = task[start:end]
        self.assertIn("uint8_t command = KICKER_COMMAND_STOP_CHARGE", safe_stop)
        self.assertEqual(safe_stop.count("SendCommand(&command, 1U)"), 1)
        self.assertNotIn("KICKER_COMMAND_IMMEDIATE_KICK", safe_stop)
        self.assertNotIn("KICKER_COMMAND_AUTO_KICK", safe_stop)
        self.assertIn("kicker_state = KICKER_STATE_WAITING_STATUS", safe_stop)

    def test_app_queues_only_an_accepted_rising_edge(self):
        body = self.function_body(self.app, "AppC_ApplyRobotCommand")
        accept = body.index("if (AcceptCommandLocked")
        request = body.index("KickerBoard_RequestAutoKick")
        self.assertGreater(request, accept)
        self.assertIn("command->kick != 0U", body)
        self.assertIn("command->kick_power >= 1U", body)

    def test_safe_paths_cancel_unsent_kick_and_request_stop(self):
        safe = self.function_body(self.app, "EnterSafeState")
        emergency = self.function_body(self.app, "AppC_EmergencyStop")
        cancel = self.function_body(self.driver, "KickerBoard_CancelPendingKick")
        self.assertIn("KickerBoard_CancelPendingKick();", safe)
        self.assertIn("KickerBoard_RequestSafeStop();", safe)
        self.assertIn("KickerBoard_CancelPendingKick();", emergency)
        self.assertIn("KickerBoard_RequestSafeStop();", emergency)
        self.assertNotIn("KickerBoard_RequestAutoKick", safe)
        self.assertNotIn("KickerBoard_RequestImmediateKick", safe)
        self.assertNotIn("KickerBoard_RequestAutoKick", emergency)
        self.assertNotIn("KickerBoard_RequestImmediateKick", emergency)
        self.assertNotIn("SendCommand", cancel)

    def test_repeated_safe_loop_does_not_queue_repeated_stops(self):
        safe = self.function_body(self.app, "EnterSafeState")
        accept = self.function_body(self.app, "AcceptCommandLocked")
        self.assertIn("if (g_safeStateActive == 0U)", safe)
        self.assertEqual(safe.count("KickerBoard_RequestSafeStop()"), 1)
        self.assertIn("g_safeStateActive = 1U", safe)
        self.assertIn("g_safeStateActive = 0U", accept)

    def test_status_polling_remains_ten_hz(self):
        self.assertIn("#define KICKER_STATUS_PERIOD_MS 100U", self.driver)

    def test_main_initializes_and_services_after_serial(self):
        self.assertIn("KickerBoard_Init(&hi2c2);", self.main)
        loop = self.main[self.main.index("while (1)"):]
        self.assertLess(loop.index("SerialService_Task();"), loop.index("KickerBoard_Task();"))


if __name__ == "__main__":
    unittest.main()
