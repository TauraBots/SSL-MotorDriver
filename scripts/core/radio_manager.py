"""AirPort-facing manager prepared for a multi-robot transport.

For now it preserves the exact single-target SerialManager transport and only
adds fleet/state observation on top of existing signals.
"""

import time

from .robot_manager import RobotManager
from .serial_manager import SerialManager


class RadioManager(SerialManager):
    def __init__(self, parent=None):
        super().__init__(parent); self.robots = RobotManager(self)
        self.connected.connect(lambda port, baud, robot_id: self.robots.set_radio_connected(True, robot_id, port, baud))
        self.disconnected.connect(lambda: self.robots.set_radio_connected(False))
        self.telemetry_received.connect(self._track_telemetry)
        self.telemetry_lost.connect(self.robots.mark_active_lost)

    def _track_telemetry(self, data):
        latency = max(0, int((time.monotonic() - data.get("received_at", time.monotonic())) * 1000))
        self.robots.update_telemetry(data, latency)

    def select_robot(self, robot_id):
        self.robots.select_robot(robot_id)
        # The current protocol remains single-target; selection updates the
        # destination used by the unchanged encoder and polling loop.
        self.state.robot_id = robot_id
