"""Nanosecond deadline scheduler independent from Qt and serial hardware."""

from dataclasses import dataclass

from .radio_profile import NORMAL, RadioProfile


@dataclass(frozen=True)
class DueWork:
    command: bool
    telemetry: bool
    command_missed: int = 0
    telemetry_missed: int = 0


class RadioScheduler:
    """Accumulative deadlines with at most one send of each kind per wake-up."""

    def __init__(self, profile: RadioProfile = NORMAL):
        self.profile = profile
        self.command_period_ns = 0
        self.telemetry_period_ns = 0
        self.next_command_ns = 0
        self.next_telemetry_ns = 0
        self.set_profile(profile, 0)

    def set_profile(self, profile: RadioProfile, now_ns: int):
        self.profile = profile
        self.command_period_ns = round(1_000_000_000 / profile.command_hz)
        self.telemetry_period_ns = round(1_000_000_000 / profile.telemetry_fast_hz)
        self.reset(now_ns)

    def reset(self, now_ns: int):
        self.next_command_ns = int(now_ns)
        phase_ns = round(self.telemetry_period_ns * self.profile.telemetry_phase_fraction)
        self.next_telemetry_ns = int(now_ns) + phase_ns

    @staticmethod
    def _advance(now_ns, deadline_ns, period_ns):
        if now_ns < deadline_ns:
            return deadline_ns, 0
        elapsed_periods = (now_ns - deadline_ns) // period_ns
        return deadline_ns + (elapsed_periods + 1) * period_ns, int(elapsed_periods)

    def take_due(self, now_ns: int) -> DueWork:
        command_due = now_ns >= self.next_command_ns
        telemetry_due = now_ns >= self.next_telemetry_ns
        command_missed = telemetry_missed = 0
        if command_due:
            self.next_command_ns, command_missed = self._advance(
                now_ns, self.next_command_ns, self.command_period_ns)
        if telemetry_due:
            self.next_telemetry_ns, telemetry_missed = self._advance(
                now_ns, self.next_telemetry_ns, self.telemetry_period_ns)
        return DueWork(command_due, telemetry_due, command_missed, telemetry_missed)

    def guard_command_until(self, deadline_ns: int):
        if deadline_ns > self.next_command_ns:
            self.next_command_ns = int(deadline_ns)
