"""Fleet registry and active robot selection."""

import time

from PySide6.QtCore import QObject, Signal

from .robot_state import RobotState
from .system_state import SystemState


class RobotManager(QObject):
    system_state_changed = Signal(object)
    fleet_changed = Signal(object)
    active_robot_changed = Signal(object)
    robot_updated = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent); self.system_state = SystemState(); self._robots = {}

    @property
    def robots(self):
        return tuple(self._robots[key] for key in sorted(self._robots))

    @property
    def active_robot(self):
        return self._robots.get(self.system_state.active_robot_id)

    def ensure_robot(self, robot_id):
        if robot_id not in self._robots: self._robots[robot_id] = RobotState(robot_id=robot_id)
        return self._robots[robot_id]

    def set_radio_connected(self, connected, robot_id=None, port="", baud=0):
        self.system_state.radio_connected = connected
        if robot_id:
            robot = self.ensure_robot(robot_id); robot.connected = connected; robot.port = port; robot.baud = baud
            robot.status = "ONLINE" if connected else "OFFLINE"
            if self.system_state.active_robot_id is None: self.system_state.active_robot_id = robot_id
        if not connected:
            for robot in self._robots.values(): robot.connected = False; robot.status = "OFFLINE"
        self._refresh_system(); self.fleet_changed.emit(self.robots)

    def update_telemetry(self, data, latency_ms):
        robot = self.ensure_robot(data["robot_id"]); robot.apply_telemetry(data); robot.latency_ms = latency_ms; robot.last_seen = time.monotonic()
        self.system_state.latency_ms = latency_ms; self._refresh_system(); self.robot_updated.emit(robot); self.fleet_changed.emit(self.robots)

    def mark_active_lost(self):
        robot = self.active_robot
        if robot: robot.connected = False; robot.status = "OFFLINE"
        self._refresh_system(); self.fleet_changed.emit(self.robots)

    def select_robot(self, robot_id):
        robot = self.ensure_robot(robot_id); self.system_state.active_robot_id = robot_id
        self._refresh_system(); self.active_robot_changed.emit(robot)

    def _refresh_system(self):
        self.system_state.online_robot_count = sum(robot.connected for robot in self._robots.values())
        if not self.system_state.radio_connected: self.system_state.system_status = "OFFLINE"
        elif not self.system_state.active_robot_id: self.system_state.system_status = "WARNING"
        elif any(robot.status == "WARNING" for robot in self._robots.values() if robot.connected): self.system_state.system_status = "WARNING"
        else: self.system_state.system_status = "READY"
        self.system_state_changed.emit(self.system_state)
