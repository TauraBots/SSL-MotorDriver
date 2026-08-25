"""Telemetry history, statistics and CSV serialization."""

import datetime as dt
from collections import deque

CSV_HEADER = ["host_time_iso", "robot_id", "request_sequence", "flags", "status",
              "mcu_time_ms", "rpm1_x10", "rpm2_x10", "rpm3_x10", "rpm4_x10",
              "battery_mV", "battery_adc", "cmd1", "cmd2", "cmd3", "cmd4",
              "brake", "command_sequence", "communication_ok", "kick_power",
              "autotune_motor", "autotune_state", "autotune_error", "autotune_active",
              "autotune_tu_s", "autotune_ku", "autotune_kp", "autotune_ki", "autotune_kd",
              "autotune_elapsed_ms", "autotune_completed_periods", "autotune_usable_periods",
              "autotune_stability_flags", "autotune_stable_windows",
              "autotune_average_high_rpm",
              "autotune_average_low_rpm", "autotune_period_spread",
              "autotune_high_peak_spread", "autotune_low_peak_spread"]


def telemetry_csv_row(data):
    rpm = [int(round(value * 10.0)) for value in data["rpm"]]
    tune = data.get("autotune", {})
    detail = data.get("autotune_detail", {})
    return [dt.datetime.now().isoformat(), data["robot_id"], data["request_sequence"],
            data["flags"], data["status"], data["time_ms"], *rpm,
            int(round(data["battery_v"] * 1000.0)), data["battery_adc"], *data["cmd"],
            data["brake"], data["command_sequence"], data["comm_ok"], data["kick_power"],
            tune.get("motor", 0), tune.get("state", 0), tune.get("error", 0),
            tune.get("active", 0), tune.get("tu", 0.0), tune.get("ku", 0.0),
            tune.get("kp", 0.0), tune.get("ki", 0.0), tune.get("kd", 0.0),
            detail.get("elapsed_ms", 0), detail.get("completed_periods", 0),
            detail.get("usable_periods", 0), detail.get("stability_flags", 0),
            detail.get("stable_windows", 0),
            detail.get("average_high_rpm", 0.0), detail.get("average_low_rpm", 0.0),
            detail.get("period_spread", 0.0), detail.get("high_peak_spread", 0.0),
            detail.get("low_peak_spread", 0.0)]


class TelemetryHistory:
    def __init__(self, capacity=180):
        self.samples = deque(maxlen=capacity)

    def append(self, sample):
        self.samples.append(dict(sample))

    def clear(self):
        self.samples.clear()

    def latest(self):
        return self.samples[-1] if self.samples else None
