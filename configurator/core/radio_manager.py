"""AirPort Team manager with D1 control and time-slotted uplink handling."""

import time
from dataclasses import replace

from PySide6.QtCore import QTimer, Signal

from .protocol import Protocol, TeamRobotCommand
from .robot_manager import RobotManager
from .serial_manager import SerialManager


class RadioManager(SerialManager):
    DISCOVERY_TIMEOUT_MS = 500
    MAX_FRAMES_PER_TICK = 32
    UPLINK_TIMEOUT_S = 1.5
    UPLINK_PROBE_INTERVAL_MS = 500
    EMERGENCY_SAFE_FRAMES = 3
    discovery_started = Signal()
    discovery_finished = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent); self.robots = RobotManager(self); self._discovery_sequence = 0
        self._discovery_sent_at = {}; self._discovery_seen = set(); self._discovery_generation = 0
        self._config_action = None; self._config_generation = 0
        self._config_responses = []; self._config_received_bytes = 0
        self._uplink_probe_index = 0; self._last_probe_ns = None
        self._telemetry_requests = {}; self._last_full_request_ns = {}
        self._team_sequence = 0
        self._team_commands = self._safe_team_commands()
        self.uplink_timeout_s = self.UPLINK_TIMEOUT_S
        self.uplink_probe_interval_ms = self.UPLINK_PROBE_INTERVAL_MS
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

    @property
    def uplink_robot_id(self):
        return self.robots.uplink_robot_id

    def _expire_stale_robots(self):
        previous_uplink = self.robots.uplink_robot_id
        self.robots.expire_stale(timeout_s=self.uplink_timeout_s)
        if previous_uplink is not None and self.robots.uplink_robot_id is None:
            self.link_stats.reset_uplink_window()

    def connect_serial(self, port, baud, robot_id):
        self._telemetry_requests.clear(); self._last_full_request_ns.clear()
        self._uplink_probe_index = 0; self._last_probe_ns = None
        self._team_sequence = 0; self._team_commands = self._safe_team_commands()
        super().connect_serial(port, baud, robot_id)

    @staticmethod
    def _safe_team_commands():
        return {robot_id: TeamRobotCommand(brake=True)
                for robot_id in Protocol.TEAM_ROBOT_IDS}

    def _send_team_frame(self):
        if not self.is_connected:
            return False
        sequence = self._team_sequence
        frame = Protocol.encode_team_velocity(sequence, self._team_commands)
        self._write_frame(frame, "command")
        self._team_sequence = (sequence + 1) & 0xFFFF
        return sequence

    def _command_brake(self, motion):
        # A normal zero command coasts after the linear acceleration ramp.
        # Brake is reserved for an explicit stop, disable or failsafe.
        return self._brake

    def _telemetry_may_guard_commands(self):
        # D1 has absolute priority. Losing an E1 is preferable to stretching a
        # 50 ms command period toward the firmware watchdog threshold.
        return False

    def _next_poll_robot_id(self):
        if self.robots.uplink_robot_id is not None:
            return self.robots.uplink_robot_id
        candidates = self._probe_robot_ids()
        return candidates[self._uplink_probe_index % len(candidates)]

    def _probe_robot_ids(self):
        candidates = tuple(robot.robot_id for robot in self.robots.discovered_robots
                           if robot.robot_id in ("A", "B", "C"))
        if not candidates:
            candidates = ("A", "B", "C")
        return candidates

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
        if not self.is_connected: return False
        sent_ns = time.monotonic_ns()
        probing = self.robots.uplink_robot_id is None
        probe_interval_ns = self.uplink_probe_interval_ms * 1_000_000
        if (probing and self._last_probe_ns is not None and
                sent_ns - self._last_probe_ns < probe_interval_ns):
            return False
        robot_id = self._next_poll_robot_id()
        if probing:
            candidates = self._probe_robot_ids()
            self._uplink_probe_index = (self._uplink_probe_index + 1) % len(candidates)
            self._last_probe_ns = sent_ns
        self._request_sequence = (self._request_sequence + 1) & 0xFFFF
        self._telemetry_requests[self._request_sequence] = {
            "request_sequence": self._request_sequence, "robot_id": robot_id,
            "sent_time_ns": sent_ns, "flags": flags, "probe": probing,
        }
        if flags == Protocol.TELEMETRY_FLAGS_FULL:
            self._last_full_request_ns[robot_id] = sent_ns
        self._write_frame(Protocol.encode_telemetry_request(
            robot_id, self._request_sequence, flags), "telemetry_request", sent_ns)
        return True

    def set_radio_profile(self, profile):
        super().set_radio_profile(profile)
        self._telemetry_requests.clear(); self._last_full_request_ns.clear()

    def _expire_telemetry_requests(self, now_ns):
        timeout_ns = self.profile.response_timeout_ms * 1_000_000
        expired = [sequence for sequence, request in self._telemetry_requests.items()
                   if now_ns - request["sent_time_ns"] > timeout_ns]
        for sequence in expired:
            request = self._telemetry_requests.pop(sequence)
            if request.get("probe"):
                self.link_stats.record_probe_timeout()
            else:
                self.link_stats.record_timeout(now_ns)

    def send_command(self, vx, vy, omega, kick_power=0, brake=False):
        active = self.robots.active_robot
        kick = int(kick_power) > 0
        if active is not None and active.can_control and active.robot_id in self._team_commands:
            self.state.robot_id = active.robot_id
            self._team_commands[active.robot_id] = TeamRobotCommand(
                vx=float(vx), vy=float(vy), omega=float(omega),
                kick_power=kick_power, kick=kick, chip=False,
                brake=bool(brake), dribbler=False, enabled=True,
            )
        sequence = self._send_team_frame()
        if sequence is False:
            return
        self.command_sent.emit(vx, vy, omega, sequence)
        # Kick is a one-frame edge. Keep the slot enabled, but never latch its
        # action bit or power into subsequent team frames.
        if active is not None and kick and active.robot_id in self._team_commands:
            self._team_commands[active.robot_id] = replace(
                self._team_commands[active.robot_id], kick_power=0, kick=False)

    def emergency_stop(self):
        self._target = (0.0, 0.0, 0.0); self._brake = True
        self._kick_pending = False
        self.state.vx = self.state.vy = self.state.omega = 0.0
        self._team_commands = self._safe_team_commands()
        try:
            for _ in range(self.EMERGENCY_SAFE_FRAMES):
                self._send_team_frame()
        except Exception as exc:
            self.error.emit(str(exc))

    def disconnect_serial(self, send_brake=True):
        if send_brake and self.is_connected:
            self.emergency_stop()
        # The base disconnect safety packet is D0, so it must stay disabled for
        # MATCH_TRANSPORT_AIRPORT_TEAM. Safety D1 frames were sent above.
        super().disconnect_serial(send_brake=False)

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
                if self.robots.uplink_robot_id != data["robot_id"]:
                    self.link_stats.reset_uplink_window()
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
                if self.robots.uplink_robot_id != data["robot_id"]:
                    self.link_stats.reset_uplink_window()
                self.link_stats.record_response(now_ns, data["latency_ms"])
                self._last_telemetry = now_s; self._lost_emitted = False
                if data["robot_id"] == self.state.robot_id: self.state.apply_telemetry(data)
                self.history.append(data); self.telemetry_received.emit(data)

    def select_robot(self, robot_id):
        if robot_id not in self._team_commands:
            raise ValueError("AirPort Team control supports robot IDs A, B and C")
        self.robots.select_robot(robot_id)
        # Selection changes only the command destination. Fleet telemetry is
        # scheduled independently by send_telemetry_request().
        self.state.robot_id = robot_id
        command = self._team_commands[robot_id]
        self.state.vx, self.state.vy, self.state.omega = command.vx, command.vy, command.omega
        self._brake = False

    def clear_robot_selection(self):
        active = self.robots.active_robot
        if active is not None and active.robot_id in self._team_commands:
            self._team_commands[active.robot_id] = TeamRobotCommand(brake=True)
            try:
                self._send_team_frame()
            except Exception as exc:
                self.error.emit(str(exc))
        self._target = (0.0, 0.0, 0.0); self._brake = True
        self._kick_pending = False
        self.state.vx = self.state.vy = self.state.omega = 0.0
        self.robots.clear_selection()
