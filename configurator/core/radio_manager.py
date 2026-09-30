"""AirPort-facing manager prepared for a multi-robot transport.

For now it preserves the exact single-target SerialManager transport and only
adds fleet/state observation on top of existing signals.
"""

import time

from PySide6.QtCore import QTimer, Signal

from .protocol import Protocol
from .robot_manager import RobotManager
from .serial_manager import SerialManager


class RadioManager(SerialManager):
    DISCOVERY_TIMEOUT_MS = 500
    MAX_FRAMES_PER_TICK = 32
    discovery_started = Signal()
    discovery_finished = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent); self.robots = RobotManager(self); self._discovery_sequence = 0
        self._discovery_sent_at = {}; self._discovery_seen = set(); self._discovery_generation = 0
        self._config_action = None; self._config_generation = 0
        self._config_responses = []; self._config_received_bytes = 0
        self._fleet_poll_index = 0; self._telemetry_requests = {}; self._last_full_request_ns = {}
        self._fleet_health_timer = QTimer(self); self._fleet_health_timer.setInterval(250)
        self._fleet_health_timer.timeout.connect(self._expire_stale_robots); self._fleet_health_timer.start()
        self.connected.connect(lambda port, baud, robot_id: self.robots.set_radio_connected(True, port=port, baud=baud))
        self.disconnected.connect(lambda: self.robots.set_radio_connected(False))
        self.telemetry_received.connect(self._track_telemetry)
        self.telemetry_lost.connect(self._expire_stale_robots)

    def _track_telemetry(self, data):
        latency = data.get("latency_ms")
        if latency is None: return
        self.robots.update_telemetry(data, latency)

    def _expire_stale_robots(self):
        count = max(1, len(self.robots.discovered_robots))
        self.robots.expire_stale(timeout_s=max(1.5, count * 0.5))

    def connect_serial(self, port, baud, robot_id):
        self._telemetry_requests.clear(); self._last_full_request_ns.clear()
        self._fleet_poll_index = 0
        super().connect_serial(port, baud, robot_id)

    def _next_poll_robot_id(self):
        candidates = self.robots.discovered_robots
        if candidates:
            return candidates[self._fleet_poll_index % len(candidates)].robot_id
        return self.state.robot_id

    def _telemetry_flags_for_next_request(self, now_ns):
        if not self.profile.split_telemetry:
            return Protocol.TELEMETRY_FLAGS_FULL
        robot_id = self._next_poll_robot_id()
        full_period_ns = round(1_000_000_000 / self.profile.telemetry_full_hz)
        last_full_ns = self._last_full_request_ns.get(robot_id)
        if last_full_ns is None or now_ns - last_full_ns >= full_period_ns:
            return Protocol.TELEMETRY_FLAGS_FULL
        return Protocol.TELEMETRY_FLAGS_FAST

    def send_telemetry_request(self, flags=Protocol.TELEMETRY_FLAGS_FULL):
        if not self.is_connected: return
        robot_id = self._next_poll_robot_id()
        candidates = self.robots.discovered_robots
        if candidates:
            self._fleet_poll_index = (self._fleet_poll_index + 1) % len(candidates)
        self._request_sequence = (self._request_sequence + 1) & 0xFFFF
        sent_ns = time.monotonic_ns()
        self._telemetry_requests[self._request_sequence] = {
            "request_sequence": self._request_sequence, "robot_id": robot_id,
            "sent_time_ns": sent_ns, "flags": flags,
        }
        if flags == Protocol.TELEMETRY_FLAGS_FULL:
            self._last_full_request_ns[robot_id] = sent_ns
        self._write_frame(Protocol.encode_telemetry_request(
            robot_id, self._request_sequence, flags), "telemetry_request", sent_ns)

    def set_radio_profile(self, profile):
        super().set_radio_profile(profile)
        self._telemetry_requests.clear(); self._last_full_request_ns.clear()

    def _expire_telemetry_requests(self, now_ns):
        timeout_ns = self.profile.response_timeout_ms * 1_000_000
        expired = [sequence for sequence, request in self._telemetry_requests.items()
                   if now_ns - request["sent_time_ns"] > timeout_ns]
        for sequence in expired:
            del self._telemetry_requests[sequence]
            self.link_stats.record_timeout(now_ns)

    def send_command(self, vx, vy, omega, kick_power=0, brake=False):
        active = self.robots.active_robot
        if active is None or not active.can_control:
            return
        self.state.robot_id = active.robot_id
        super().send_command(vx, vy, omega, kick_power, brake)

    def disconnect_serial(self, send_brake=True):
        active = self.robots.active_robot if hasattr(self, "robots") else None
        if active is not None: self.state.robot_id = active.robot_id
        super().disconnect_serial(send_brake=send_brake and active is not None)

    def discover_robots(self, timeout_ms=DISCOVERY_TIMEOUT_MS):
        if not self.is_connected:
            self.error.emit("Radio link is offline")
            return
        self._discovery_sequence = (self._discovery_sequence + 1) & 0xFFFF
        sequence = self._discovery_sequence; self._discovery_generation += 1; generation = self._discovery_generation
        self._discovery_seen = set(); self._discovery_sent_at[sequence] = time.monotonic()
        try:
            self._write_frame(Protocol.encode_discovery_request(sequence)); self.discovery_started.emit()
            QTimer.singleShot(max(1, int(timeout_ms)), lambda: self._finish_discovery(generation))
        except Exception as exc: self.error.emit(str(exc))

    def discover_boards(self, port, baud):
        if not self.is_connected:
            super().discover_boards(port, baud); return
        self._begin_live_config("discover", 1200)
        nonce = int(time.time() * 1000) & 0xFFFFFFFF
        self._write_frame(Protocol.encode_discovery(nonce))

    def configure_robot_id(self, port, baud, uid, robot_id):
        if not self.is_connected:
            super().configure_robot_id(port, baud, uid, robot_id); return
        try:
            target = Protocol.parse_uid(uid); self._begin_live_config("set-id", 1000, target=target, robot_id=robot_id)
            self._write_frame(Protocol.encode_set_id(target, robot_id))
        except Exception as exc: self._finish_live_config_error(str(exc))

    def configure_motion(self, port, baud, uid, linear_accel, angular_accel):
        if not self.is_connected:
            super().configure_motion(port, baud, uid, linear_accel, angular_accel); return
        try:
            target = Protocol.parse_uid(uid); self._begin_live_config("set-motion", 1000, target=target,
                                                                     linear=linear_accel, angular=angular_accel)
            self._write_frame(Protocol.encode_motion_config(target, linear_accel, angular_accel))
        except Exception as exc: self._finish_live_config_error(str(exc))

    def _begin_live_config(self, action, timeout_ms, **context):
        self._config_generation += 1; generation = self._config_generation
        self._config_action = (action, context); self._config_responses = []; self._config_received_bytes = 0
        QTimer.singleShot(timeout_ms, lambda: self._finish_live_config(generation))

    def _finish_live_config(self, generation):
        if generation != self._config_generation or self._config_action is None: return
        action, context = self._config_action; self._config_action = None
        if action == "discover":
            self.boards_discovered.emit(self._config_responses, self._config_received_bytes)
        elif action == "set-id":
            reply = next((v for v in self._config_responses if v[1] == Protocol.CONFIG_SET_ID_RESPONSE_TYPE and v[2] == context["target"]), None)
            if reply is None: self.error.emit("A placa não respondeu ao pedido de configuração")
            elif reply[3] == 0: self.error.emit("A placa não conseguiu gravar o ID na Flash")
            else: self.board_configured.emit(context["robot_id"])
        else:
            reply = next((v for v in self._config_responses if v[1] == Protocol.CONFIG_SET_MOTION_RESPONSE_TYPE and v[2] == context["target"]), None)
            if reply is None: self.error.emit("A placa não respondeu à configuração de movimento")
            elif reply[3] == 0: self.error.emit("A placa rejeitou os limites de aceleração")
            else: self.motion_configured.emit(context["linear"], context["angular"])
        self.configuration_finished.emit()

    def _finish_live_config_error(self, message):
        self._config_action = None; self._config_generation += 1; self.error.emit(message); self.configuration_finished.emit()

    def _finish_discovery(self, generation):
        if generation != self._discovery_generation: return
        self.robots.complete_discovery(self._discovery_seen)
        self.discovery_finished.emit(self.robots.discovered_robots)

    def _consume_rx(self, now):
        now_ns = self._as_ns(now); now_s = now_ns / 1_000_000_000
        frames_processed = 0
        while frames_processed < self.MAX_FRAMES_PER_TICK:
            start = self._rx.find(b"\x55\xAA")
            if start < 0:
                if len(self._rx) > 2: del self._rx[:-2]
                return
            if start: del self._rx[:start]
            if len(self._rx) < 3: return
            packet_type = self._rx[2]
            if packet_type == Protocol.DISCOVERY_RESPONSE_TYPE:
                size = Protocol.DISCOVERY_RESPONSE_SIZE
            elif packet_type in (Protocol.CONFIG_DISCOVER_RESPONSE_TYPE,
                                  Protocol.CONFIG_SET_ID_RESPONSE_TYPE,
                                  Protocol.CONFIG_SET_MOTION_RESPONSE_TYPE):
                size = Protocol.CONFIG_RESPONSE_SIZE
            elif packet_type == Protocol.TELEMETRY_RESPONSE_TYPE:
                if len(self._rx) < 9: return
                flags = self._rx[8] & Protocol.TELEMETRY_FLAGS_FULL
                size = Protocol.TELEMETRY_RESPONSE_BASE_SIZE
                size += 7 if flags & Protocol.TELEMETRY_FLAG_BASIC else 0
                size += 16 if flags & Protocol.TELEMETRY_FLAG_MOTORS else 0
                size += 4 if flags & Protocol.TELEMETRY_FLAG_BATTERY else 0
                size += 13 if flags & Protocol.TELEMETRY_FLAG_DIAGNOSTICS else 0
            else:
                next_start = self._rx.find(b"\x55\xAA", 2)
                if next_start < 0:
                    del self._rx[:-2]
                    return
                del self._rx[:next_start]
                continue
            if len(self._rx) < size: return
            frame = bytes(self._rx[:size]); del self._rx[:size]
            frames_processed += 1
            if packet_type == Protocol.DISCOVERY_RESPONSE_TYPE:
                data = Protocol.parse_discovery_response(frame)
                if not data: continue
                sent_at = self._discovery_sent_at.get(data["request_sequence"])
                if sent_at is None: continue
                latency = max(0, int((now_s - sent_at) * 1000)); self._discovery_seen.add(data["robot_id"])
                self.robots.handle_discovery_response(data, latency)
            elif packet_type in (Protocol.CONFIG_DISCOVER_RESPONSE_TYPE,
                                  Protocol.CONFIG_SET_ID_RESPONSE_TYPE,
                                  Protocol.CONFIG_SET_MOTION_RESPONSE_TYPE):
                values = Protocol.parse_config_response(frame)
                if values and self._config_action is not None:
                    self._config_received_bytes += len(frame); self._config_responses.append(values)
                    action, context = self._config_action
                    expected = ((action == "set-id" and packet_type == Protocol.CONFIG_SET_ID_RESPONSE_TYPE) or
                                (action == "set-motion" and packet_type == Protocol.CONFIG_SET_MOTION_RESPONSE_TYPE))
                    if expected: self._finish_live_config(self._config_generation)
            else:
                response_robot_id = chr(frame[4]) if len(frame) > 4 else ""
                data = Protocol.parse_telemetry(bytearray(frame), response_robot_id)
                if not data: continue
                request = self._telemetry_requests.get(data["request_sequence"])
                if request is None or request["robot_id"] != data["robot_id"]:
                    self.link_stats.record_unmatched(); continue
                del self._telemetry_requests[data["request_sequence"]]
                data["latency_ms"] = max(0.0, (now_ns - request["sent_time_ns"]) / 1_000_000)
                self.link_stats.record_response(now_ns, data["latency_ms"])
                self._last_telemetry = now_s; self._lost_emitted = False
                if data["robot_id"] == self.state.robot_id: self.state.apply_telemetry(data)
                self.history.append(data); self.telemetry_received.emit(data)

    def select_robot(self, robot_id):
        self.robots.select_robot(robot_id)
        # Selection changes only the command destination. Fleet telemetry is
        # scheduled independently by send_telemetry_request().
        self.state.robot_id = robot_id

    def clear_robot_selection(self):
        if self.robots.active_robot is not None:
            self.emergency_stop()
        self._target = (0.0, 0.0, 0.0); self._brake = True
        self.robots.clear_selection()
