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
    fault_status: int = 0
    communication_status: str = "offline"
    last_seen: float | None = None
    latency_ms: int | None = None
    status: str = "OFFLINE"
    uid: str | None = None
    firmware_version: str | None = None
    discovered: bool = False
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0

    def apply_telemetry(self, data):
        self.battery_voltage = data.get("battery_v", self.battery_voltage)
        self.motor_rpm = tuple(data.get("rpm", self.motor_rpm))
        self.motor_command = tuple(data.get("cmd", self.motor_command))
        self.sequence = data.get("command_sequence", self.sequence)
        self.watchdog_ok = bool(data.get("watchdog_ok", data.get("comm_ok", False)))
        self.fault_status = int(data.get("fault_status", self.fault_status))
        self.communication_status = "control_active" if self.watchdog_ok else "control_inactive"
        self.connected = True
        self.last_seen = data.get("received_at", time.monotonic())
        # Receiving valid telemetry proves that the robot is online. The
        # command watchdog only says whether control packets arrived recently;
        # it is expected to be inactive for robots that are not selected.
        self.status = "WARNING" if self.fault_status else "ONLINE"

    @property
    def battery(self):
        return self.battery_voltage

    @property
    def rpm(self):
        return self.motor_rpm
