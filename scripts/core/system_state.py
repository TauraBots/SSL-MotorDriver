"""Global operator-facing state for the radio and robot fleet."""

from dataclasses import dataclass


@dataclass
class SystemState:
    radio_connected: bool = False
    online_robot_count: int = 0
    active_robot_id: str | None = None
    latency_ms: int | None = None
    system_status: str = "OFFLINE"
