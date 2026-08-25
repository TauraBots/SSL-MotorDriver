"""Telemetry history, statistics and CSV serialization."""

import datetime as dt
from collections import deque

CSV_HEADER = ["host_time_iso", "robot_id", "request_sequence", "flags", "status",
              "mcu_time_ms", "rpm1_x10", "rpm2_x10", "rpm3_x10", "rpm4_x10",
              "battery_mV", "battery_adc", "cmd1", "cmd2", "cmd3", "cmd4",
              "brake", "command_sequence", "communication_ok", "kick_power"]


def telemetry_csv_row(data):
    rpm = [int(round(value * 10.0)) for value in data["rpm"]]
    return [dt.datetime.now().isoformat(), data["robot_id"], data["request_sequence"],
            data["flags"], data["status"], data["time_ms"], *rpm,
            int(round(data["battery_v"] * 1000.0)), data["battery_adc"], *data["cmd"],
            data["brake"], data["command_sequence"], data["comm_ok"], data["kick_power"]]


class TelemetryHistory:
    def __init__(self, capacity=180):
        self.samples = deque(maxlen=capacity)

    def append(self, sample):
        self.samples.append(dict(sample))

    def clear(self):
        self.samples.clear()

    def latest(self):
        return self.samples[-1] if self.samples else None
