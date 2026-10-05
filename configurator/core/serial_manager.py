"""Qt serial backend; this is the only module that owns serial ports."""

import math
import threading
import time

import serial
from serial.tools import list_ports
from PySide6.QtCore import QObject, Qt, QTimer, Signal

from .protocol import Protocol
from .radio_profile import NORMAL, get_radio_profile
from .radio_scheduler import RadioScheduler
from .radio_stats import RadioLinkStats
from .robot_state import RobotState
from .telemetry import TelemetryHistory


class SerialManager(QObject):
    MAX_RX_READ_PER_TICK = 512
    MAX_RX_BUFFER_SIZE = 4096
    connected = Signal(str, int, str)
    disconnected = Signal()
    telemetry_received = Signal(dict)
    # Transport sequence (D0 is uint32; AirPort Team D1 is uint16).
    command_sent = Signal(float, float, float, object)
    telemetry_lost = Signal()
    error = Signal(str)
    boards_discovered = Signal(object, int)
    board_configured = Signal(str)
    motion_configured = Signal(float, float)
    configuration_finished = Signal()
    profile_changed = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.state = RobotState()
        self.history = TelemetryHistory()
        self._serial = None
        self._rx = bytearray()
        self._timer = QTimer(self)
        self._timer.setTimerType(Qt.TimerType.PreciseTimer)
        self._timer.setInterval(1)
        self._timer.timeout.connect(self._tick)
        self._sequence = int(time.time() * 1000) & 0xFFFFFFFF
        self._request_sequence = 0
        self._telemetry_sent_at = {}
        self.profile = NORMAL
        self.scheduler = RadioScheduler(self.profile)
        self.link_stats = RadioLinkStats()
        self._last_motion = time.monotonic()
        self._last_telemetry = 0.0
        self._lost_emitted = False
        self._target = (0.0, 0.0, 0.0)
        self._kick_power = 0
        self._kick_pending = False
        self._brake = True

    @property
    def is_connected(self):
        return self._serial is not None and self._serial.is_open

    @staticmethod
    def available_ports():
        return [port.device for port in list_ports.comports()]

    @staticmethod
    def validate_uid(uid_text):
        Protocol.parse_uid(uid_text)

    def connect_serial(self, port, baud, robot_id):
        self.disconnect_serial(send_brake=False)
        candidate = None
        try:
            candidate = serial.Serial(port, baud, timeout=0, write_timeout=0.10)
            candidate.reset_input_buffer()
            self._serial = candidate
            self.state = RobotState(robot_id=robot_id, connected=True, port=port, baud=baud,
                                    communication_status="waiting")
            self._rx.clear(); self.history.clear(); self._request_sequence = 0
            self._telemetry_sent_at.clear()
            now_ns = time.monotonic_ns(); now = now_ns / 1_000_000_000
            self.scheduler.reset(now_ns); self.link_stats.reset(now_ns)
            self._last_motion = now
            self._last_telemetry = 0.0; self._lost_emitted = False
            self._target = (0.0, 0.0, 0.0); self._brake = True
            self._timer.start()
            self.connected.emit(port, baud, robot_id)
        except Exception as exc:
            if candidate is not None:
                try:
                    candidate.close()
                except Exception:
                    pass
            self._serial = None
            self.state.connected = False
            self.state.communication_status = "offline"
            self.error.emit(str(exc))

    def reconnect(self):
        """Reopen the last selected endpoint using the current robot identity."""
        port, baud, robot_id = self.state.port, self.state.baud, self.state.robot_id
        if not port or not baud:
            self.error.emit("Nenhuma conexão anterior disponível para reconectar")
            return
        self.connect_serial(port, baud, robot_id)

    def disconnect_serial(self, send_brake=True):
        self._timer.stop()
        port = self._serial
        if port:
            try:
                if send_brake and self.state.robot_id:
                    self._sequence = (self._sequence + 1) & 0xFFFFFFFF
                    port.write(Protocol.encode_velocity(self.state.robot_id, self._sequence,
                                                        0.0, 0.0, 0.0, brake=1))
                port.close()
            except Exception:
                try: port.close()
                except Exception: pass
        was_connected = self.state.connected
        self._serial = None; self.state.connected = False; self.state.communication_status = "offline"
        if was_connected: self.disconnected.emit()

    def set_motion_target(self, vx, vy, omega, brake=False):
        self._target = (float(vx), float(vy), float(omega)); self._brake = bool(brake)

    def queue_kick(self, power):
        self._kick_power = max(0, min(100, int(power))); self._kick_pending = True

    def send_command(self, vx, vy, omega, kick_power=0, brake=False):
        if not self.is_connected: return
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        self._write_frame(Protocol.encode_velocity(self.state.robot_id, self._sequence,
                                                   vx, vy, omega, kick_power, brake), "command")
        self.command_sent.emit(vx, vy, omega, self._sequence)

    def _command_brake(self, motion):
        """Legacy D0 stops request brake; AirPort Team overrides this policy."""
        return self._brake or motion == (0.0, 0.0, 0.0)

    def _telemetry_may_guard_commands(self):
        """Legacy transports may reserve a telemetry reply window."""
        return self.profile.telemetry_reply_guard

    def emergency_stop(self):
        self._target = (0.0, 0.0, 0.0); self._brake = True
        self.state.vx = self.state.vy = self.state.omega = 0.0
        try: self.send_command(0.0, 0.0, 0.0, brake=True)
        except Exception as exc: self.error.emit(str(exc))

    def send_telemetry_request(self, flags=Protocol.TELEMETRY_FLAGS_FULL):
        if not self.is_connected: return False
        self._request_sequence = (self._request_sequence + 1) & 0xFFFF
        sent_ns = time.monotonic_ns()
        self._telemetry_sent_at[self._request_sequence] = {
            "request_sequence": self._request_sequence, "robot_id": self.state.robot_id,
            "sent_time_ns": sent_ns, "flags": flags,
        }
        self._write_frame(Protocol.encode_telemetry_request(
            self.state.robot_id, self._request_sequence, flags), "telemetry_request", sent_ns)
        return True

    def set_radio_profile(self, profile):
        profile = get_radio_profile(profile)
        now_ns = time.monotonic_ns()
        self.profile = profile
        self.scheduler.set_profile(profile, now_ns)
        self._telemetry_sent_at.clear()
        self.link_stats.reset(now_ns)
        self.profile_changed.emit(profile)

    def _write_frame(self, frame, kind=None, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else int(now_ns)
        written = self._serial.write(frame)
        self.link_stats.record_tx_bytes(now_ns, written)
        if kind == "command": self.link_stats.record_command(now_ns)
        elif kind == "telemetry_request": self.link_stats.record_request(now_ns)
        return written

    def _limited_motion(self, now):
        vx, vy, omega = self._target
        elapsed = max(0.0, now - self._last_motion); self._last_motion = now
        stopping = vx == vy == omega == 0.0
        alpha = 1.0 - math.exp(-elapsed / (0.08 if stopping else 0.18))
        self.state.vx += (vx - self.state.vx) * alpha
        self.state.vy += (vy - self.state.vy) * alpha
        self.state.omega += (omega - self.state.omega) * alpha
        for name in ("vx", "vy", "omega"):
            if abs(getattr(self.state, name)) < 1e-4: setattr(self.state, name, 0.0)
        return self.state.vx, self.state.vy, self.state.omega

    def _tick(self):
        if not self.is_connected: return
        try:
            now_ns = time.monotonic_ns(); now = now_ns / 1_000_000_000
            # Drain available RX first; never wait for bytes in the Qt thread.
            waiting = min(self._serial.in_waiting, self.MAX_RX_READ_PER_TICK)
            if waiting:
                chunk = self._serial.read(waiting); self._rx.extend(chunk)
                self.link_stats.record_rx_bytes(now_ns, len(chunk))
                if len(self._rx) > self.MAX_RX_BUFFER_SIZE:
                    del self._rx[:-self.MAX_RX_BUFFER_SIZE]
            if self._rx:
                self._consume_rx(now_ns)
            self._expire_telemetry_requests(now_ns)

            due = self.scheduler.take_due(now_ns)
            self.link_stats.record_missed(due.command_missed, due.telemetry_missed)
            # The command frame (D1 TeamFrame in RadioManager) always wins when
            # command and telemetry share a scheduler wake-up.
            if due.command:
                motion = self._limited_motion(now)
                kick = self._kick_power if self._kick_pending else 0
                self.send_command(*motion, kick_power=kick,
                                  brake=self._command_brake(motion))
                self._kick_pending = False
            if due.telemetry:
                telemetry_sent = self.send_telemetry_request(
                    self._telemetry_flags_for_next_request(now_ns))
                if telemetry_sent and self._telemetry_may_guard_commands():
                    guard_ns = round(Protocol.TELEMETRY_REPLY_WINDOW_S * 1_000_000_000)
                    self.scheduler.guard_command_until(now_ns + guard_ns)
            if self._last_telemetry and now - self._last_telemetry > 1.0 and not self._lost_emitted:
                self._lost_emitted = True; self.telemetry_lost.emit()
        except Exception as exc:
            self.disconnect_serial(send_brake=False); self.error.emit(str(exc))

    def _telemetry_flags_for_next_request(self, now_ns):
        return Protocol.TELEMETRY_FLAGS_FULL

    def _expire_telemetry_requests(self, now_ns):
        timeout_ns = self.profile.response_timeout_ms * 1_000_000
        expired = [sequence for sequence, request in self._telemetry_sent_at.items()
                   if now_ns - request["sent_time_ns"] > timeout_ns]
        for sequence in expired:
            del self._telemetry_sent_at[sequence]
            self.link_stats.record_timeout(now_ns)

    @staticmethod
    def _as_ns(value):
        # Compatibility for callers/tests that still pass time.monotonic().
        return int(value * 1_000_000_000) if isinstance(value, float) else int(value)

    def _consume_rx(self, now):
        now_ns = self._as_ns(now)
        data = Protocol.parse_telemetry(self._rx, self.state.robot_id)
        if not data: return
        request = self._telemetry_sent_at.pop(data["request_sequence"], None)
        if request is None or request["robot_id"] != data["robot_id"]:
            data["latency_ms"] = None; self.link_stats.record_unmatched()
        else:
            data["latency_ms"] = max(0.0, (now_ns - request["sent_time_ns"]) / 1_000_000)
            self.link_stats.record_response(now_ns, data["latency_ms"])
        self._last_telemetry = now_ns / 1_000_000_000; self._lost_emitted = False
        self.state.apply_telemetry(data); self.history.append(data)
        self.telemetry_received.emit(data)

    def _configure_async(self, action, port, baud, **values):
        threading.Thread(target=self._configuration_worker,
                         args=(action, port, baud, values), daemon=True).start()

    def discover_boards(self, port, baud):
        self._configure_async("discover", port, baud)

    def configure_robot_id(self, port, baud, uid, robot_id):
        self._configure_async("set-id", port, baud, uid=uid, robot_id=robot_id)

    def configure_motion(self, port, baud, uid, linear_accel, angular_accel):
        self._configure_async("set-motion", port, baud, uid=uid,
                              linear_accel=linear_accel, angular_accel=angular_accel)

    def _configuration_worker(self, action, port, baud, values):
        try:
            with serial.Serial(port, baud, timeout=0.05, write_timeout=0.10) as config_port:
                time.sleep(0.25); config_port.reset_input_buffer()
                if action == "discover":
                    replies, received = [], 0
                    for attempt in range(2):
                        config_port.write(Protocol.encode_discovery(int(time.time() * 1000) + attempt)); config_port.flush()
                        found, count = Protocol.read_config_responses(config_port, 1.2)
                        replies.extend(found); received += count
                    unique = {reply[2]: reply for reply in replies
                              if reply[1] == Protocol.CONFIG_DISCOVER_RESPONSE_TYPE}
                    self.boards_discovered.emit(list(unique.values()), received)
                elif action == "set-id":
                    uid = Protocol.parse_uid(values["uid"])
                    config_port.write(Protocol.encode_set_id(uid, values["robot_id"])); config_port.flush()
                    replies, _ = Protocol.read_config_responses(config_port, 1.0)
                    reply = next((v for v in replies if v[1] == Protocol.CONFIG_SET_ID_RESPONSE_TYPE and v[2] == uid), None)
                    if reply is None: raise RuntimeError("A placa não respondeu ao pedido de configuração")
                    if reply[3] == 0: raise RuntimeError("A placa não conseguiu gravar o ID na Flash")
                    self.board_configured.emit(values["robot_id"])
                else:
                    uid = Protocol.parse_uid(values["uid"])
                    config_port.write(Protocol.encode_motion_config(uid, values["linear_accel"], values["angular_accel"])); config_port.flush()
                    replies, _ = Protocol.read_config_responses(config_port, 1.0)
                    reply = next((v for v in replies if v[1] == Protocol.CONFIG_SET_MOTION_RESPONSE_TYPE and v[2] == uid), None)
                    if reply is None: raise RuntimeError("A placa não respondeu à configuração de movimento")
                    if reply[3] == 0: raise RuntimeError("A placa rejeitou os limites de aceleração")
                    self.motion_configured.emit(values["linear_accel"], values["angular_accel"])
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            self.configuration_finished.emit()
