import os
import sys
import unittest
from pathlib import Path


os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "configurator"))

from PySide6.QtWidgets import QApplication
from ui.autotune_panel import AutoTunePanel


class AutoTunePanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.panel = AutoTunePanel()
        self.panel.set_available("A")

    def tearDown(self):
        self.panel.deleteLater()

    def test_old_aborted_result_is_presented_as_retryable(self):
        self.panel.update_telemetry({
            "rpm": (0.0, 0.0, 0.0, 0.0), "cmd": (0, 0, 0, 0),
            "autotune": {"motor": 1, "state": 3, "error": 9, "active": 0,
                         "tu": 0.0, "ku": 0.0, "kp": 0.0, "ki": 0.0, "kd": 0.0},
        })
        self.assertEqual(self.panel.state.text(), "ABORTED")
        self.assertTrue(self.panel.start_button.isEnabled())
        self.assertFalse(self.panel.abort_button.isEnabled())
        self.assertEqual(self.panel.metrics["kp"].text(), "—")
        self.assertFalse(self.panel.progress.isTextVisible())

    def test_running_state_locks_start_and_enables_abort(self):
        self.panel.set_running("A", 2)
        self.panel.update_telemetry({
            "rpm": (0.0, 45.0, 0.0, 0.0), "cmd": (0, 600, 0, 0),
            "autotune": {"motor": 2, "state": 1, "error": 0, "active": 1,
                         "tu": 0.0, "ku": 0.0, "kp": 0.0, "ki": 0.0, "kd": 0.0},
        })
        self.assertEqual(self.panel.state.text(), "RUNNING")
        self.assertFalse(self.panel.start_button.isEnabled())
        self.assertTrue(self.panel.abort_button.isEnabled())
        self.assertEqual(self.panel.metrics["tu"].text(), "—")
        self.assertIn("/ 30 s", self.panel.detail.text())
        self.assertEqual(self.panel.progress.maximum(), 100)

    def test_preview_is_the_safe_default_mode(self):
        requested = []
        self.panel.start_requested.connect(
            lambda motor, mode: requested.append((motor, mode)))
        self.panel.start_button.click()
        self.assertEqual(requested, [(1, self.panel.MODE_PREVIEW)])

    def test_finished_preview_does_not_claim_flash_was_written(self):
        self.panel._requested_mode = self.panel.MODE_PREVIEW
        self.panel.update_telemetry({
            "rpm": (0.0, 0.0, 0.0, 0.0), "cmd": (0, 0, 0, 0),
            "autotune": {"motor": 1, "state": 2, "error": 0, "active": 0,
                         "tu": 0.2, "ku": 30.0, "kp": 13.5, "ki": 81.0, "kd": 0.0},
        })
        self.assertIn("não aplicados nem salvos", self.panel.detail.text())

    def test_terminal_failure_survives_following_idle_ack(self):
        terminal = {
            "rpm": (0.0, 0.0, 0.0, 0.0), "cmd": (0, 0, 0, 0),
            "autotune": {"motor": 1, "state": 3, "error": 6, "active": 0,
                         "tu": 0.0, "ku": 0.0, "kp": 0.0, "ki": 0.0, "kd": 0.0},
        }
        self.panel.update_telemetry(terminal)
        self.assertEqual(self.panel.state.text(), "FAILED")
        self.assertIn("INVALID OSCILLATION", self.panel.detail.text())

        terminal["autotune"] = {
            "motor": 0, "state": 0, "error": 0, "active": 0,
            "tu": 0.0, "ku": 0.0, "kp": 0.0, "ki": 0.0, "kd": 0.0,
        }
        self.panel.update_telemetry(terminal)
        self.assertEqual(self.panel.state.text(), "FAILED")
        self.assertIn("INVALID OSCILLATION", self.panel.detail.text())


if __name__ == "__main__":
    unittest.main()
