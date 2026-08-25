"""Qt serial backend; this is the only module that owns serial ports."""

import math
import os
import threading
import time

import serial
from serial.tools import list_ports
from PySide6.QtCore import QObject, QTimer, Signal

from .protocol import Protocol
from .robot_state import RobotState
from .telemetry import TelemetryHistory


def _open_serial(port, baud, **kwargs):
    if os.name == "posix":
        kwargs["exclusive"] = True
    return serial.Serial(port, baud, **kwargs)


class SerialManager(QObject):
    MAX_RX_READ_PER_TICK = 512
    MAX_RX_BUFFER_SIZE = 4096
    connected = Signal(str, int, str)
    disconnected = Signal()
    telemetry_received = Signal(dict)
    command_sent = Signal(float, float, float, int)
    telemetry_lost = Signal()
    error = Signal(str)
    boards_discovered = Signal(object, int)
    board_configured = Signal(str)
    motion_configured = Signal(float, float)
    configuration_finished = Signal()
    autotune_started = Signal(str, int)
    autotune_finished = Signal(str, dict)
    autotune_aborted = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.state = RobotState()
        self.history = TelemetryHistory()
        self._serial = None
        self._rx = bytearray()
        self._timer = QTimer(self)
        self._timer.setInterval(10)
        self._timer.timeout.connect(self._tick)
        self._sequence = int(time.time() * 1000) & 0xFFFFFFFF
        self._request_sequence = 0
        self._telemetry_sent_at = {}
        self._next_command = self._next_telemetry = 0.0
        self._last_motion = time.monotonic()
        self._last_telemetry = 0.0
        self._lost_emitted = False
        self._target = (0.0, 0.0, 0.0)
        self._kick_power = 0
        self._kick_pending = False
        self._brake = True
        self._autotune_motor_id = None
        self._autotune_robot_id = None
        self._autotune_start_sequence = None
        self._autotune_seen_running = False
        self._autotune_action = Protocol.AUTOTUNE_ACTION_START

    @property
    def is_connected(self):
        return self._serial is not None and self._serial.is_open

    @property
    def autotune_active(self):
        return self._autotune_motor_id is not None

    @property
    def autotune_robot_id(self):
        return self._autotune_robot_id

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
            candidate = _open_serial(port, baud, timeout=0, write_timeout=0.10)
            candidate.reset_input_buffer()
            self._serial = candidate
            self.state = RobotState(robot_id=robot_id, connected=True, port=port, baud=baud,
                                    communication_status="waiting")
            self._rx.clear(); self.history.clear(); self._request_sequence = 0
            now = time.monotonic()
            self._next_command, self._next_telemetry, self._last_motion = now, now + 0.1, now
            self._last_telemetry = 0.0; self._lost_emitted = False
            self._target = (0.0, 0.0, 0.0); self._brake = True
            self._autotune_motor_id = None; self._autotune_robot_id = None
            self._autotune_start_sequence = None; self._autotune_seen_running = False
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
                if self.autotune_active:
                    self.abort_autotune()
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

    def _send_autotune_control(self, action):
        if not self.is_connected or not self._autotune_robot_id:
            return None
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        self._serial.write(Protocol.encode_autotune_control(
            self._autotune_robot_id, self._sequence,
            int(self._autotune_motor_id or 0), action))
        return self._sequence

    def start_autotune(self, motor_id, action=Protocol.AUTOTUNE_ACTION_START):
        if not self.is_connected:
            self.error.emit("Radio link is offline")
            return False
        if motor_id not in (0, 1, 2, 3, 4):
            self.error.emit("Motor de Auto-Tune inválido")
            return False
        if action not in (Protocol.AUTOTUNE_ACTION_START,
                          Protocol.AUTOTUNE_ACTION_PREVIEW,
                          Protocol.AUTOTUNE_ACTION_STAGE):
            self.error.emit("Modo de Auto-Tune inválido")
            return False
        self._target = (0.0, 0.0, 0.0); self._brake = True
        self.state.vx = self.state.vy = self.state.omega = 0.0
        self._autotune_robot_id = self.state.robot_id
        self._autotune_motor_id = int(motor_id)
        self._autotune_seen_running = False
        self._autotune_action = int(action)
        self._next_command = time.monotonic() + 0.05
        # Clear a terminal result from a previous run before START. Otherwise
        # the firmware deliberately latches it and treats START as heartbeat.
        self._send_autotune_control(Protocol.AUTOTUNE_ACTION_ABORT)
        self._autotune_start_sequence = self._send_autotune_control(
            self._autotune_action)
        self.autotune_started.emit(self._autotune_robot_id, int(motor_id))
        return True

    def restore_pid_configs(self):
        if not self.is_connected:
            self.error.emit("Radio link is offline")
            return False
        if self.autotune_active:
            self.error.emit("Aborte o Auto-Tune antes de restaurar os ganhos")
            return False
        self._autotune_robot_id = self.state.robot_id
        self._autotune_motor_id = 0
        self._send_autotune_control(Protocol.AUTOTUNE_ACTION_ABORT)
        self._send_autotune_control(Protocol.AUTOTUNE_ACTION_RESTORE)
        self._autotune_robot_id = None
        self._autotune_motor_id = None
        return True

    def commit_staged_pid_configs(self, motor_id):
        if not self.is_connected:
            self.error.emit("Radio link is offline")
            return False
        if self.autotune_active or motor_id not in (0, 1, 2, 3, 4):
            self.error.emit("Candidato RAM indisponível para esse motor")
            return False
        self._autotune_robot_id = self.state.robot_id
        self._autotune_motor_id = int(motor_id)
        self._send_autotune_control(Protocol.AUTOTUNE_ACTION_COMMIT_STAGED)
        self._autotune_robot_id = None
        self._autotune_motor_id = None
        return True

    def abort_autotune(self):
        if not self.autotune_active:
            return False
        robot_id = self._autotune_robot_id
        self._send_autotune_control(Protocol.AUTOTUNE_ACTION_ABORT)
        # The first frame stops an active run; the second acknowledges and
        # clears its terminal status so the next telemetry returns IDLE.
        self._send_autotune_control(Protocol.AUTOTUNE_ACTION_ABORT)
        self._autotune_motor_id = None; self._autotune_robot_id = None
        self._autotune_start_sequence = None; self._autotune_seen_running = False
        self._target = (0.0, 0.0, 0.0); self._brake = True
        self.autotune_aborted.emit(robot_id)
        return True

    def _finish_autotune_from_telemetry(self, data):
        tune = data.get("autotune")
        if (not self.autotune_active or not tune or
                data.get("robot_id") != self._autotune_robot_id):
            return
        state = int(tune.get("state", 0))
        if tune.get("active") or state == 1:
            self._autotune_seen_running = True
            return
        if state not in (2, 3):
            return

        acknowledges_start = self.autotune_telemetry_is_current(data)
        if not self._autotune_seen_running and not acknowledges_start:
            return

        if not tune.get("active"):
            robot_id = self._autotune_robot_id
            result = dict(tune)
            self._send_autotune_control(Protocol.AUTOTUNE_ACTION_ABORT)
            self._autotune_motor_id = None; self._autotune_robot_id = None
            self._autotune_start_sequence = None; self._autotune_seen_running = False
            self.autotune_finished.emit(robot_id, result)

    def autotune_telemetry_is_current(self, data):
        if not self.autotune_active:
            return True
        if data.get("robot_id") != self._autotune_robot_id:
            return False
        tune = data.get("autotune")
        if not tune:
            return False
        if tune.get("active") or int(tune.get("state", 0)) == 1:
            return True
        reported_sequence = data.get("command_sequence")
        if reported_sequence is None or self._autotune_start_sequence is None:
            return False
        delta = (int(reported_sequence) - self._autotune_start_sequence) & 0xFFFFFFFF
        return delta < 0x80000000

    def set_motion_target(self, vx, vy, omega, brake=False):
        self._target = (float(vx), float(vy), float(omega)); self._brake = bool(brake)

    def queue_kick(self, power):
        self._kick_power = max(0, min(100, int(power))); self._kick_pending = True

    def send_command(self, vx, vy, omega, kick_power=0, brake=False):
        if not self.is_connected: return
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        self._serial.write(Protocol.encode_velocity(self.state.robot_id, self._sequence,
                                                    vx, vy, omega, kick_power, brake))
        self.command_sent.emit(vx, vy, omega, self._sequence)

    def emergency_stop(self):
        self._target = (0.0, 0.0, 0.0); self._brake = True
        self.state.vx = self.state.vy = self.state.omega = 0.0
        try:
            self.abort_autotune()
            self.send_command(0.0, 0.0, 0.0, brake=True)
        except Exception as exc: self.error.emit(str(exc))

    def send_telemetry_request(self):
        if not self.is_connected: return
        self._request_sequence = (self._request_sequence + 1) & 0xFFFF
        self._telemetry_sent_at[self._request_sequence] = time.monotonic()
        self._serial.write(Protocol.encode_telemetry_request(self.state.robot_id,
                                                             self._request_sequence))

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
            now = time.monotonic()
            if now >= self._next_command:
                if self.autotune_active:
                    self._send_autotune_control(self._autotune_action)
                else:
                    motion = self._limited_motion(now)
                    kick = self._kick_power if self._kick_pending else 0
                    self.send_command(*motion, kick_power=kick,
                                      brake=self._brake or motion == (0.0, 0.0, 0.0))
                self._kick_pending = False; self._next_command = now + 0.05
            if now >= self._next_telemetry:
                self.send_telemetry_request(); self._next_telemetry = now + 0.2
                self._next_command = max(self._next_command, now + Protocol.TELEMETRY_REPLY_WINDOW_S)
            waiting = min(self._serial.in_waiting, self.MAX_RX_READ_PER_TICK)
            if waiting:
                self._rx.extend(self._serial.read(waiting))
                if len(self._rx) > self.MAX_RX_BUFFER_SIZE:
                    del self._rx[:-self.MAX_RX_BUFFER_SIZE]
                self._consume_rx(now)
            if self._last_telemetry and now - self._last_telemetry > 1.0 and not self._lost_emitted:
                self._lost_emitted = True
                self.telemetry_lost.emit()
                if self.autotune_active:
                    self.abort_autotune()
        except Exception as exc:
            self.disconnect_serial(send_brake=False); self.error.emit(str(exc))

    def _consume_rx(self, now):
        data = Protocol.parse_telemetry(self._rx, self.state.robot_id)
        if not data: return
        sent_at = self._telemetry_sent_at.pop(data["request_sequence"], None)
        data["latency_ms"] = None if sent_at is None else max(0, int((now - sent_at) * 1000))
        self._telemetry_sent_at = {seq: sent for seq, sent in self._telemetry_sent_at.items()
                                   if now - sent < 2.0}
        self._last_telemetry = now; self._lost_emitted = False
        self.state.apply_telemetry(data); self.history.append(data)
        self._finish_autotune_from_telemetry(data)
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
            with _open_serial(port, baud, timeout=0.05, write_timeout=0.10) as config_port:
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
