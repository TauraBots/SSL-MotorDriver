"""Observable-independent state model for one robot."""

from dataclasses import dataclass, field
import time


@dataclass
class RobotState:
    robot_id: str = "A"
    connected: bool = False
    port: str = ""
    baud: int = 0
    battery_voltage: float | None = None
    motor_rpm: tuple = field(default_factory=lambda: (0.0,) * 4)
    motor_command: tuple = field(default_factory=lambda: (0,) * 4)
    sequence: int = 0
    watchdog_ok: bool = False
    communication_status: str = "offline"
    last_seen: float | None = None
    latency_ms: int | None = None
    status: str = "OFFLINE"
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0

    def apply_telemetry(self, data):
        self.battery_voltage = data.get("battery_v", self.battery_voltage)
        self.motor_rpm = tuple(data.get("rpm", self.motor_rpm))
        self.motor_command = tuple(data.get("cmd", self.motor_command))
        self.sequence = data.get("command_sequence", self.sequence)
        self.watchdog_ok = bool(data.get("watchdog_ok", data.get("comm_ok", False)))
        self.communication_status = "online" if self.watchdog_ok else "lost"
        self.connected = True
        self.last_seen = data.get("received_at", time.monotonic())
        self.status = "ONLINE" if self.watchdog_ok else "WARNING"

    @property
    def battery(self):
        return self.battery_voltage

    @property
    def rpm(self):
        return self.motor_rpm
