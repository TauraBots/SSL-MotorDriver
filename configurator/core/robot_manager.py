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
        if not connected:
            for robot in self._robots.values(): robot.connected = False; robot.status = "OFFLINE"
            self.system_state.active_robot_id = None
        self._refresh_system(); self.fleet_changed.emit(self.robots)

    def update_telemetry(self, data, latency_ms):
        robot = self.ensure_robot(data["robot_id"]); robot.apply_telemetry(data); robot.latency_ms = latency_ms; robot.last_seen = time.monotonic()
        self.system_state.latency_ms = latency_ms; self._refresh_system(); self.robot_updated.emit(robot); self.fleet_changed.emit(self.robots)

    def handle_discovery_response(self, data, latency_ms):
        robot_id, uid = data["robot_id"], data["uid"]
        robot = next((item for item in self._robots.values() if item.uid == uid), None)
        previous_id = robot.robot_id if robot is not None else None
        if robot is None:
            robot = self.ensure_robot(robot_id)
        elif previous_id != robot_id:
            self._robots.pop(previous_id, None)
            displaced = self._robots.get(robot_id)
            if displaced is not None and displaced is not robot:
                displaced.connected = False; displaced.status = "ID CONFLICT"
            robot.robot_id = robot_id; self._robots[robot_id] = robot
            if self.system_state.active_robot_id == previous_id:
                self.system_state.active_robot_id = robot_id
        robot.uid = uid
        robot.firmware_version = data["firmware_version"]; robot.battery_voltage = data.get("battery_v")
        robot.last_seen = data.get("received_at", time.monotonic()); robot.latency_ms = latency_ms
        robot.connected = True; robot.discovered = True
        discovery_status = data.get("status", 0)
        robot.fault_status = discovery_status >> 1
        robot.status = "WARNING" if robot.fault_status else "ONLINE"
        self._refresh_system(); self.robot_updated.emit(robot)
        if previous_id is not None and previous_id != robot_id:
            self.active_robot_changed.emit(robot if self.system_state.active_robot_id == robot_id else self.active_robot)
        self.fleet_changed.emit(self.discovered_robots)
        return robot

    @property
    def discovered_robots(self):
        return tuple(robot for robot in self.robots if robot.discovered)

    def complete_discovery(self, seen_robot_ids, timeout_s=1.5, now=None):
        now = time.monotonic() if now is None else now
        for robot in self._robots.values():
            stale = robot.last_seen is None or now - robot.last_seen > timeout_s
            if robot.discovered and robot.robot_id not in seen_robot_ids and stale:
                robot.connected = False; robot.status = "OFFLINE"
        active = self.active_robot
        if active is not None and not active.connected:
            self.system_state.active_robot_id = None; self.active_robot_changed.emit(None)
        self._refresh_system(); self.fleet_changed.emit(self.discovered_robots)

    def mark_active_lost(self):
        robot = self.active_robot
        if robot: robot.connected = False; robot.status = "OFFLINE"
        self._refresh_system(); self.fleet_changed.emit(self.robots)

    def expire_stale(self, timeout_s=1.5, now=None):
        now = time.monotonic() if now is None else now; changed = False
        for robot in self._robots.values():
            if (robot.discovered and robot.connected and robot.last_seen is not None and
                    now - robot.last_seen > timeout_s):
                robot.connected = False; robot.status = "OFFLINE"; changed = True
        if changed:
            active = self.active_robot
            if active is not None and not active.connected:
                self.system_state.active_robot_id = None; self.active_robot_changed.emit(None)
            self._refresh_system(); self.fleet_changed.emit(self.discovered_robots)

    def select_robot(self, robot_id):
        robot = self.ensure_robot(robot_id); self.system_state.active_robot_id = robot_id
        self._refresh_system(); self.active_robot_changed.emit(robot)

    def clear_selection(self):
        if self.system_state.active_robot_id is None: return
        self.system_state.active_robot_id = None
        self._refresh_system(); self.active_robot_changed.emit(None)

    def _refresh_system(self):
        self.system_state.online_robot_count = sum(robot.connected and robot.discovered
                                                   for robot in self._robots.values())
        if not self.system_state.radio_connected: self.system_state.system_status = "OFFLINE"
        elif any(robot.status == "WARNING" for robot in self._robots.values() if robot.connected): self.system_state.system_status = "WARNING"
        else: self.system_state.system_status = "READY"
        self.system_state_changed.emit(self.system_state)
