"""Bounded, monotonic radio-link measurements."""

from collections import deque
from dataclasses import dataclass
import math
import time


@dataclass(frozen=True)
class RadioLinkSnapshot:
    command_tx_total: int
    telemetry_request_tx_total: int
    telemetry_response_rx_total: int
    command_tx_hz: float
    telemetry_request_tx_hz: float
    telemetry_response_rx_hz: float
    command_deadlines_missed: int
    telemetry_deadlines_missed: int
    tx_bytes_total: int
    rx_bytes_total: int
    tx_bytes_per_s: float
    rx_bytes_per_s: float
    telemetry_unmatched_responses: int
    telemetry_timed_out_requests: int
    telemetry_probe_timeouts: int
    telemetry_response_loss_percent: float
    latency_last_ms: float | None
    latency_mean_ms: float | None
    latency_min_ms: float | None
    latency_max_ms: float | None
    latency_p95_ms: float | None


class RadioLinkStats:
    def __init__(self, window_s=10.0, now_ns=None):
        self.window_ns = round(window_s * 1_000_000_000)
        self.reset(time.monotonic_ns() if now_ns is None else now_ns)

    def reset(self, now_ns=None):
        self.started_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        self.command_tx_total = self.telemetry_request_tx_total = 0
        self.telemetry_response_rx_total = 0
        self.command_deadlines_missed = self.telemetry_deadlines_missed = 0
        self.tx_bytes_total = self.rx_bytes_total = 0
        self.telemetry_unmatched_responses = self.telemetry_timed_out_requests = 0
        self.telemetry_probe_timeouts = 0
        self._commands = deque(); self._requests = deque(); self._responses = deque()
        self._tx_bytes = deque(); self._rx_bytes = deque(); self._latencies = deque()
        self._outcomes = deque()

    def record_command(self, now_ns):
        self.command_tx_total += 1; self._commands.append(int(now_ns))

    def record_request(self, now_ns):
        self.telemetry_request_tx_total += 1; self._requests.append(int(now_ns))

    def record_response(self, now_ns, latency_ms):
        now_ns = int(now_ns); latency_ms = float(latency_ms)
        self.telemetry_response_rx_total += 1; self._responses.append(now_ns)
        self._latencies.append((now_ns, latency_ms)); self._outcomes.append((now_ns, False))

    def record_timeout(self, now_ns):
        self.telemetry_timed_out_requests += 1; self._outcomes.append((int(now_ns), True))

    def record_probe_timeout(self):
        self.telemetry_probe_timeouts += 1

    def reset_uplink_window(self):
        """Drop latency/loss samples that belong to a previous uplink robot."""
        self._latencies.clear(); self._outcomes.clear()

    def record_unmatched(self):
        self.telemetry_unmatched_responses += 1

    def record_tx_bytes(self, now_ns, count):
        self.tx_bytes_total += count; self._tx_bytes.append((int(now_ns), count))

    def record_rx_bytes(self, now_ns, count):
        self.rx_bytes_total += count; self._rx_bytes.append((int(now_ns), count))

    def record_missed(self, command=0, telemetry=0):
        self.command_deadlines_missed += int(command)
        self.telemetry_deadlines_missed += int(telemetry)

    def _trim(self, now_ns):
        cutoff = now_ns - self.window_ns
        for queue in (self._commands, self._requests, self._responses):
            while queue and queue[0] < cutoff: queue.popleft()
        for queue in (self._tx_bytes, self._rx_bytes, self._latencies, self._outcomes):
            while queue and queue[0][0] < cutoff: queue.popleft()

    def _rate(self, events, now_ns):
        if not events: return 0.0
        duration_ns = min(self.window_ns, max(1, now_ns - self.started_ns))
        return len(events) * 1_000_000_000 / duration_ns

    def snapshot(self, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else int(now_ns); self._trim(now_ns)
        latencies = [value for _, value in self._latencies]
        ordered = sorted(latencies)
        p95 = ordered[min(len(ordered) - 1, max(0, math.ceil(len(ordered) * 0.95) - 1))] if ordered else None
        losses = sum(lost for _, lost in self._outcomes); considered = len(self._outcomes)
        duration_ns = min(self.window_ns, max(1, now_ns - self.started_ns))
        byte_scale = 1_000_000_000 / duration_ns
        return RadioLinkSnapshot(
            self.command_tx_total, self.telemetry_request_tx_total, self.telemetry_response_rx_total,
            self._rate(self._commands, now_ns), self._rate(self._requests, now_ns),
            self._rate(self._responses, now_ns), self.command_deadlines_missed,
            self.telemetry_deadlines_missed, self.tx_bytes_total, self.rx_bytes_total,
            sum(value for _, value in self._tx_bytes) * byte_scale,
            sum(value for _, value in self._rx_bytes) * byte_scale,
            self.telemetry_unmatched_responses, self.telemetry_timed_out_requests,
            self.telemetry_probe_timeouts,
            (100.0 * losses / considered) if considered else 0.0,
            latencies[-1] if latencies else None,
            (sum(latencies) / len(latencies)) if latencies else None,
            min(latencies) if latencies else None, max(latencies) if latencies else None, p95,
        )
