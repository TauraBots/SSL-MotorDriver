"""Application composition root. UI code never accesses serial bytes."""

import csv
import os
import threading
import time
from pathlib import Path

from PySide6.QtCore import QEvent, Qt, QTimer, Signal
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import QApplication, QComboBox, QFileDialog, QFrame, QHBoxLayout, QLabel, QMainWindow, QMessageBox, QPushButton, QSizePolicy, QStackedWidget, QVBoxLayout, QWidget

from core import (RADIO_PROFILES, RadioManager, airport_ota_capacity_message,
                  estimate_protocol_traffic, telemetry_csv_row)
from core.telemetry import CSV_HEADER
from .analysis_panel import AnalysisPanel
from .config_panel import ConfigPanel
from .control_panel import ControlPanel
from .dashboard import DashboardPanel
from .diagnostics_panel import DiagnosticsPanel
from .fleet_panel import FleetPanel
from .telemetry_panel import TelemetryPanel
from .widgets import HeaderStatusItem

SERIAL_BAUD_OPTIONS = (9600, 14400, 19200, 38400, 115200, 921600, 1000000)


class MainWindow(QMainWindow):
    plots_ready = Signal(str)

    def __init__(self, args, parent=None):
        super().__init__(parent); self.args = args; self.manager = RadioManager(self); self.pressed_keys = set(); self.discovered_boards = []
        self.manual_fleet_ids = self._parse_fleet_ids(getattr(args, "fleet_ids", ""))
        self.log_file = self.log_writer = None; self.link_log_file = self.link_log_writer = None
        self._last_link_log_ns = 0; self.config_busy = False; self._last_target_warning = 0.0
        self.setWindowTitle("TAURABOTS - Ground Control Center"); self.resize(1120, 740); self.setMinimumSize(920, 640)
        icon_dir = Path(__file__).parent / "assets" / "icons"
        self.setWindowIcon(QIcon(str(icon_dir / "icon-white.svg")))
        theme = Path(__file__).parents[1] / "styles" / "theme.qss"; self.setStyleSheet(theme.read_text(encoding="utf-8"))
        self._build(); self._connect_signals(); self.refresh_ports(); QApplication.instance().installEventFilter(self)
        if self.manual_fleet_ids:
            self.manager.robots.register_expected_robots(self.manual_fleet_ids)
        self._link_ui_timer = QTimer(self); self._link_ui_timer.setInterval(500)
        self._link_ui_timer.timeout.connect(self.update_link_metrics); self._link_ui_timer.start()

    @staticmethod
    def _parse_fleet_ids(value):
        if not value:
            return ()
        compact = value.replace(",", "").replace(" ", "").upper()
        seen, result = set(), []
        for robot_id in compact:
            if len(robot_id) != 1 or not ("A" <= robot_id <= "Z"):
                raise ValueError("--fleet-ids deve conter apenas IDs A-Z, exemplo: A,B")
            if robot_id not in seen:
                seen.add(robot_id); result.append(robot_id)
        return tuple(result)

    def _build(self):
        root = QWidget(); outer = QVBoxLayout(root); outer.setContentsMargins(0, 0, 0, 0); outer.setSpacing(0)
        header = QFrame(); header.setObjectName("header"); header_layout = QVBoxLayout(header); header_layout.setContentsMargins(24, 10, 24, 10); header_layout.setSpacing(8); row = QHBoxLayout(); header_layout.addLayout(row)
        icon_dir = Path(__file__).parent / "assets" / "icons"
        brand_icon = QLabel(); brand_icon.setObjectName("brandIcon"); brand_icon.setPixmap(QIcon(str(icon_dir / "icon-white.svg")).pixmap(46, 46)); brand_icon.setFixedSize(52, 52); row.addWidget(brand_icon)
        #brand = QVBoxLayout(); title = QLabel("GROUND CONTROL CENTER"); title.setObjectName("brand"); brand.addWidget(title)
        #subtitle = QLabel("SSL MOTOR DRIVER"); subtitle.setObjectName("eyebrow"); brand.addWidget(subtitle); row.addLayout(brand)
        row.addStretch(1)
        controls = QFrame(); controls.setObjectName("headerControls"); controls.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Preferred); control_row = QHBoxLayout(controls); control_row.setContentsMargins(10, 6, 10, 6); control_row.setSpacing(7)
        self.header_port = QComboBox(); self.header_port.setEditable(False); self.header_port.setFixedWidth(120)
        self.header_baud = QComboBox(); self.header_baud.setEditable(False); self.header_baud.addItems([str(value) for value in SERIAL_BAUD_OPTIONS]); self.header_baud.setCurrentText(str(self.args.baud)); self.header_baud.setFixedWidth(92)
        self.header_baud.setToolTip("AirPort serial baud must match on PC/TX, RX and STM32 UART.")
        self.header_profile = QComboBox(); self.header_profile.setFixedWidth(190)
        self.header_profile.setToolTip("Logical D0/E0 targets; this does not configure serial baud or ELRS packet rate.")
        for profile in RADIO_PROFILES: self.header_profile.addItem(profile.display_name, profile.name)
        if self.args.port: self.header_port.setCurrentText(self.args.port)
        for label_text, widget in (("SERIAL PORT", self.header_port), ("BAUD RATE", self.header_baud), ("RADIO PROFILE", self.header_profile)):
            field = QVBoxLayout(); field.setContentsMargins(0, 0, 0, 0); field.setSpacing(2)
            label = QLabel(label_text); label.setObjectName("headerControlLabel"); label.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Preferred)
            field.addWidget(label); field.addWidget(widget); control_row.addLayout(field)
        self.header_refresh = QPushButton("REFRESH"); self.header_refresh.setFixedWidth(88); control_row.addWidget(self.header_refresh, 0, Qt.AlignmentFlag.AlignBottom)
        self.header_connect = QPushButton("CONNECT"); self.header_connect.setFixedWidth(112); self.header_connect.setProperty("accent", True); control_row.addWidget(self.header_connect, 0, Qt.AlignmentFlag.AlignBottom); row.addWidget(controls)
        self.radio_status = HeaderStatusItem("Radio status", "OFFLINE"); self.registered_status = HeaderStatusItem("Registered", "0"); self.robots_status = HeaderStatusItem("Telemetry online", "0"); self.uplink_status = HeaderStatusItem("Uplink", "NONE"); self.latency_status = HeaderStatusItem("Link latency", "—"); self.system_status = HeaderStatusItem("System status", "OFFLINE")
        status_row = QHBoxLayout(); status_row.setSpacing(8)
        for widget in (self.radio_status, self.registered_status, self.robots_status,
                       self.uplink_status, self.latency_status, self.system_status):
            status_row.addWidget(widget, 1)
        header_layout.addLayout(status_row)
        outer.addWidget(header)
        body = QHBoxLayout(); body.setContentsMargins(0, 0, 0, 0); body.setSpacing(0)
        sidebar = QFrame(); sidebar.setObjectName("sidebar"); sidebar.setFixedWidth(205); navigation = QVBoxLayout(sidebar); navigation.setContentsMargins(14, 20, 14, 18)
        svg = lambda name: QIcon(str(icon_dir / f"{name}.svg"))
        icons = {"connection": svg("radio"), "battery": svg("battery"), "robot": svg("robot"),
                 "warning": svg("diagnostics"), "motor": svg("motor")}
        self.stack = QStackedWidget(); self.dashboard = DashboardPanel(icons); self.fleet = FleetPanel(); self.control = ControlPanel(self.args); self.telemetry = TelemetryPanel(); self.config = ConfigPanel(); self.diagnostics = DiagnosticsPanel(); self.analysis = AnalysisPanel()
        pages = (("VISÃO GERAL", self.dashboard, svg("robot")),
                 ("FLEET", self.fleet, svg("radio")),
                 ("CONTROLE", self.control, svg("motor")),
                 ("TELEMETRIA", self.telemetry, svg("chart")),
                 ("CONFIGURAÇÃO", self.config, svg("settings")),
                 ("DIAGNÓSTICO", self.diagnostics, svg("diagnostics")),
                 ("ANÁLISE DE DADOS", self.analysis, svg("chart"))); self.nav_buttons = []
        for index, (name, page, icon) in enumerate(pages):
            button = QPushButton(icon, name); button.setObjectName("nav"); button.setCheckable(True); button.clicked.connect(lambda checked=False, i=index: self.navigate(i)); navigation.addWidget(button); self.nav_buttons.append(button); self.stack.addWidget(page)
        navigation.addStretch(); stop = QPushButton("EMERGENCY STOP"); stop.setProperty("danger", True); stop.setMinimumHeight(48); stop.clicked.connect(self.emergency_stop); navigation.addWidget(stop)
        body.addWidget(sidebar); body.addWidget(self.stack, 1); outer.addLayout(body, 1); self.setCentralWidget(root)
        self.footer_status = QLabel("NO PORT  |  — BAUD  |  ROBOT —  |  FIRMWARE —  |  OFFLINE"); self.footer_status.setObjectName("footerStatus")
        self.statusBar().addWidget(self.footer_status, 1); self.navigate(0)

    def _connect_signals(self):
        self.header_connect.clicked.connect(self.toggle_connection); self.header_refresh.clicked.connect(self.refresh_ports)
        self.header_profile.currentIndexChanged.connect(self.change_radio_profile)
        self.header_baud.currentTextChanged.connect(lambda: self.update_link_metrics())
        self.control.kick_requested.connect(self.queue_kick); self.control.virtual_key.connect(self.set_virtual_key); self.control.joystick_changed.connect(self.set_joystick_command); self.control.stop_requested.connect(self.emergency_stop)
        self.telemetry.log_requested.connect(self.toggle_log); self.fleet.robot_selected.connect(self.select_fleet_robot); self.fleet.discovery_requested.connect(self.manager.discover_robots); self.config.discover_requested.connect(self.discover); self.config.set_id_requested.connect(self.set_id); self.config.set_motion_requested.connect(self.set_motion); self.config.board_selected.connect(self.select_board); self.analysis.generate_requested.connect(self.generate_plots)
        self.manager.connected.connect(self.on_connected); self.manager.disconnected.connect(self.on_disconnected); self.manager.telemetry_received.connect(self.update_telemetry); self.manager.command_sent.connect(self.command_sent); self.manager.telemetry_lost.connect(self.telemetry_lost); self.manager.error.connect(self.on_communication_error)
        self.manager.boards_discovered.connect(self.show_boards); self.manager.board_configured.connect(self.configuration_succeeded); self.manager.motion_configured.connect(self.motion_configuration_succeeded); self.manager.configuration_finished.connect(self.finish_config_action); self.plots_ready.connect(self.on_plots_ready)
        self.manager.robots.system_state_changed.connect(self.update_system_header); self.manager.robots.fleet_changed.connect(self.update_fleet); self.manager.robots.active_robot_changed.connect(self.active_robot_changed)
        self.manager.discovery_started.connect(lambda: self.fleet.set_discovering(True)); self.manager.discovery_finished.connect(self.discovery_finished)
        self.diagnostics.metrics_log_requested.connect(self.toggle_link_metrics_log)

    def change_radio_profile(self):
        profile = RADIO_PROFILES[self.header_profile.currentIndex()]
        self.manager.set_radio_profile(profile)
        if self.manager.is_connected and self.manager.state.baud != profile.default_airport_baud:
            self.warn_airport_baud(self.manager.state.baud)
        self.update_link_metrics()

    def warn_airport_baud(self, baud):
        QMessageBox.warning(
            self, "AirPort serial baud",
            f"Current serial baud is {baud}. AirPort serial baud must match on PC/TX, RX "
            "and STM32 UART. 9600 is the current test default, not a baud imposed by the "
            "radio profile target.")

    def update_link_metrics(self):
        now_ns = time.monotonic_ns(); stats = self.manager.link_stats.snapshot(now_ns)
        baud_text = self.header_baud.currentText().strip()
        serial_baud = int(baud_text) if baud_text else None
        estimate = estimate_protocol_traffic(self.manager.profile)
        capacity_note = airport_ota_capacity_message(self.manager.profile)
        self.diagnostics.update_link_stats(
            self.manager.profile, stats, serial_baud, estimate, capacity_note)
        if self.link_log_writer and now_ns - self._last_link_log_ns >= 1_000_000_000:
            self._last_link_log_ns = now_ns
            self.link_log_writer.writerow([
                now_ns, self.manager.profile.name, len(self.manager.robots.discovered_robots),
                self.manager.profile.command_hz, stats.command_tx_hz,
                self.manager.profile.telemetry_fast_hz, stats.telemetry_request_tx_hz,
                stats.telemetry_response_rx_hz, stats.telemetry_response_loss_percent,
                stats.latency_mean_ms, stats.latency_min_ms, stats.latency_max_ms,
                stats.latency_p95_ms, stats.command_deadlines_missed,
                stats.telemetry_deadlines_missed, stats.tx_bytes_per_s, stats.rx_bytes_per_s,
            ]); self.link_log_file.flush()

    def toggle_link_metrics_log(self):
        if self.link_log_file:
            self.link_log_file.close(); self.link_log_file = self.link_log_writer = None
            self.diagnostics.metrics_log_button.setText("GRAVAR MÉTRICAS CSV"); return
        path, _ = QFileDialog.getSaveFileName(
            self, "Gravar métricas do link", "radio_link_metrics.csv", "CSV (*.csv)")
        if not path: return
        self.link_log_file = open(path, "w", newline="", encoding="utf-8")
        self.link_log_writer = csv.writer(self.link_log_file)
        self.link_log_writer.writerow([
            "host_time_ns", "profile", "robot_count", "command_target_hz", "command_actual_hz",
            "telemetry_target_hz", "telemetry_request_actual_hz", "telemetry_response_actual_hz",
            "telemetry_response_loss_percent", "latency_mean_ms", "latency_min_ms", "latency_max_ms",
            "latency_p95_ms", "command_deadlines_missed", "telemetry_deadlines_missed",
            "tx_bytes_per_s", "rx_bytes_per_s",
        ])
        self._last_link_log_ns = 0; self.diagnostics.metrics_log_button.setText("PARAR MÉTRICAS CSV")

    def navigate(self, index):
        self.stack.setCurrentIndex(index)
        for i, button in enumerate(self.nav_buttons): button.setChecked(i == index)

    def update_system_header(self, state):
        self.radio_status.set_status("CONNECTED" if state.radio_connected else "OFFLINE", "ok" if state.radio_connected else "error")
        self.registered_status.set_status(str(state.registered_robot_count), "ok" if state.registered_robot_count else "off")
        self.robots_status.set_status(str(state.online_robot_count), "ok" if state.online_robot_count else "off")
        self.uplink_status.set_status(state.uplink_robot_id or "NONE", "ok" if state.uplink_robot_id else "off")
        self.latency_status.set_status(f"{state.latency_ms} ms" if state.latency_ms is not None else "NO DATA", "ok" if state.latency_ms is not None and state.latency_ms < 100 else "warning")
        system_state = "ok" if state.system_status == "READY" else "warning" if state.system_status == "WARNING" else "error"
        self.system_status.set_status(state.system_status, system_state)

    def update_fleet(self, robots):
        self.fleet.set_fleet(robots, self.manager.robots.system_state.active_robot_id,
                             self.manager.robots.uplink_robot_id)
        active = self.manager.robots.active_robot
        self.dashboard.set_robot(active)
        available = active is not None and active.can_control and self.manager.is_connected
        self.control.set_active_robot(active.robot_id if available else None)
        if active is not None and not active.connected:
            self.telemetry.communication_card.update_status("NO DATA", "TELEMETRY LOST", "warning")

    def discovery_finished(self, robots):
        telemetry = sum(robot.connected for robot in robots)
        self.fleet.set_discovering(False)
        self.toast(f"Discovery complete: {telemetry} telemetry / {len(robots)} registered", "info")

    def active_robot_changed(self, robot):
        available = robot is not None and robot.can_control and self.manager.is_connected
        self.dashboard.set_robot(robot); self.control.set_active_robot(robot.robot_id if available else None)
        self.update_fleet(self.manager.robots.robots)

    def select_fleet_robot(self, robot_id):
        robot = self.manager.robots.ensure_robot(robot_id)
        if not robot.can_control:
            self.warn_no_control_target(); return
        if self.manager.robots.system_state.active_robot_id == robot_id:
            self.manager.clear_robot_selection(); self.toast(f"Robot {robot_id} deselected", "info"); return
        self.manager.select_robot(robot_id); self.toast(f"Control target changed to Robot {robot_id}", "info")

    def can_control(self):
        robot = self.manager.robots.active_robot
        return self.manager.is_connected and robot is not None and robot.can_control

    def queue_kick(self, power):
        if self.can_control(): self.manager.queue_kick(power)
        else: self.warn_no_control_target()

    def warn_no_control_target(self):
        now = time.monotonic()
        if now - self._last_target_warning < 1.0: return
        self._last_target_warning = now
        QMessageBox.warning(self, "No control target",
                            "Select a registered robot in the Fleet before sending commands.")

    def refresh_ports(self):
        current = self.header_port.currentText(); ports = self.manager.available_ports()
        self.header_port.clear(); self.header_port.addItems(ports)
        if current: self.header_port.setCurrentText(current)
        elif self.args.port: self.header_port.setCurrentText(self.args.port)

    def serial_settings(self):
        port = self.header_port.currentText().strip()
        if not port: raise ValueError("Selecione uma porta serial")
        baud = int(self.header_baud.currentText());
        if baud <= 0: raise ValueError("Baud rate inválido")
        return port, baud

    def toggle_connection(self):
        if self.manager.is_connected: self.manager.disconnect_serial(); return
        try: port, baud = self.serial_settings()
        except Exception as exc: self.show_error("Conexão", str(exc)); return
        if baud != self.manager.profile.default_airport_baud:
            self.warn_airport_baud(baud)
        self.manager.connect_serial(port, baud, "A")

    def on_connected(self, port, baud, robot_id):
        self.header_connect.setText("DISCONNECT"); self.header_port.setEnabled(False); self.header_baud.setEnabled(False); self.header_refresh.setEnabled(False); self.control.status.setText("SELECT A ROBOT IN FLEET"); self.footer_status.setText(f"{port}  |  {baud} BAUD  |  AIRPORT ONLINE"); self.active_robot_changed(None)
        if self.manual_fleet_ids:
            self.manager.robots.register_expected_robots(self.manual_fleet_ids)
            self.toast(f"Radio connected; manual fleet: {', '.join(self.manual_fleet_ids)}", "success")
        else:
            self.toast("Radio connected; discovering robots", "success"); QTimer.singleShot(100, self.manager.discover_robots)

    def on_disconnected(self):
        self.pressed_keys.clear(); [self.control.set_key(key, False) for key in self.control.keys]
        self.header_connect.setText("CONNECT"); self.header_port.setEnabled(True); self.header_baud.setEnabled(True); self.header_refresh.setEnabled(True); self.control.status.setText("AGUARDANDO CONEXÃO"); self.footer_status.setText("NO RADIO LINK  |  AIRPORT OFFLINE"); self.control.set_active_robot(None); self.dashboard.set_robot(None)

    def on_communication_error(self, message):
        if not self.manager.is_connected:
            self.on_disconnected()
        self.radio_status.set_status("OFFLINE", "error")
        self.system_status.set_status("CONNECTION ERROR", "error")
        self.toast(f"Connection error: {message}", "error")

    def _update_target(self):
        if not self.can_control(): return
        vx = (2.0 if "d" in self.pressed_keys else 0.0) - (2.0 if "a" in self.pressed_keys else 0.0)
        vy = (2.0 if "w" in self.pressed_keys else 0.0) - (2.0 if "s" in self.pressed_keys else 0.0)
        omega = (5.0 if "q" in self.pressed_keys else 0.0) - (5.0 if "e" in self.pressed_keys else 0.0)
        if "space" in self.pressed_keys: vx = vy = omega = 0.0
        self.manager.set_motion_target(vx, vy, omega, brake=(vx == vy == omega == 0.0))

    def eventFilter(self, obj, event):
        if event.type() in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease) and not event.isAutoRepeat():
            key = {Qt.Key.Key_W: "w", Qt.Key.Key_A: "a", Qt.Key.Key_S: "s", Qt.Key.Key_D: "d", Qt.Key.Key_Q: "q", Qt.Key.Key_E: "e", Qt.Key.Key_Space: "space", Qt.Key.Key_K: "k"}.get(event.key())
            if key:
                pressed = event.type() == QEvent.Type.KeyPress
                if pressed and not self.can_control():
                    self.warn_no_control_target(); return True
                if key == "k" and pressed: self.queue_kick(self.control.kick.value())
                elif pressed: self.pressed_keys.add(key)
                else: self.pressed_keys.discard(key)
                self.control.set_key(key, pressed); self._update_target(); return True
        return super().eventFilter(obj, event)

    def set_virtual_key(self, key, pressed):
        self.pressed_keys.add(key) if pressed else self.pressed_keys.discard(key); self.control.set_key(key, pressed); self._update_target()

    def set_joystick_command(self, vx, vy, omega):
        if self.can_control() and not self.pressed_keys: self.manager.set_motion_target(vx, vy, omega, brake=(vx == vy == omega == 0.0))

    def emergency_stop(self):
        self.pressed_keys.clear(); [self.control.set_key(key, False) for key in self.control.keys]; self.control.reset_joystick(); self.manager.emergency_stop(); self.toast("STOP ROBOT sent · brake active", "error")

    def command_sent(self, vx, vy, omega, sequence):
        self.control.set_command(vx, vy, omega); self.dashboard.set_command(vx, vy, omega); self.telemetry.append_command(vx, vy, omega); self.diagnostics.tx.setText(str(sequence))

    def update_telemetry(self, data):
        latency = data.get("latency_ms")
        if latency is None: latency = 0
        if data.get("robot_id") == self.manager.robots.system_state.active_robot_id:
            self.telemetry.update_telemetry(data, latency); self.dashboard.set_telemetry(data, latency); self.diagnostics.update_telemetry(data, latency)
        if self.log_writer: self.log_writer.writerow(telemetry_csv_row(data)); self.log_file.flush()

    def telemetry_lost(self):
        active = self.manager.robots.active_robot
        if active is not None and not active.connected:
            self.telemetry.communication_card.update_status("NO DATA", "TELEMETRY LOST", "warning"); self.dashboard.watchdog_card.update_status("WARNING", "TELEMETRY LOST", "warning"); self.diagnostics.watchdog.setText("TELEMETRIA PERDIDA"); self.toast("Telemetry lost", "warning")

    def toggle_log(self):
        if self.log_file:
            self.log_file.close(); self.log_file = self.log_writer = None; self.telemetry.log_status.setText("CSV desligado"); self.telemetry.log_button.setText("GRAVAR CSV"); return
        path, _ = QFileDialog.getSaveFileName(self, "Gravar telemetria", "telemetry.csv", "CSV (*.csv)")
        if path:
            self.log_file = open(path, "w", newline="", encoding="utf-8"); self.log_writer = csv.writer(self.log_file); self.log_writer.writerow(CSV_HEADER); self.telemetry.log_status.setText(path); self.telemetry.log_button.setText("PARAR GRAVAÇÃO")

    def _begin_config(self):
        if self.config_busy: return None
        try: settings = self.serial_settings()
        except Exception as exc: self.show_error("Configuração", str(exc)); return None
        self.config_busy = True; self.config.set_busy(True); return settings

    def discover(self):
        settings = self._begin_config()
        if settings: self.config.boards.clear(); self.config.boards.addItem("Procurando placas…"); self.manager.discover_boards(*settings)

    def set_id(self):
        try: self.manager.validate_uid(self.config.uid.text().strip())
        except ValueError as exc: self.show_error("Configuração", str(exc)); return
        settings = self._begin_config()
        if settings: self.manager.configure_robot_id(*settings, self.config.uid.text().strip(), self.config.robot_id.currentText())

    def set_motion(self):
        try: self.manager.validate_uid(self.config.uid.text().strip())
        except ValueError as exc: self.show_error("Configuração", str(exc)); return
        settings = self._begin_config()
        if settings: self.manager.configure_motion(*settings, self.config.uid.text().strip(), self.config.linear.value(), self.config.angular.value())

    def finish_config_action(self): self.config_busy = False; self.config.set_busy(False)

    def show_boards(self, values, received):
        self.config.boards.clear(); self.discovered_boards = list(values)
        if not self.discovered_boards: self.config.boards.addItem(f"Nenhuma resposta válida · {received} byte(s) recebidos"); self.config.discovery_status.setText("0 placas encontradas"); return
        ids = [chr(v[4]) for v in self.discovered_boards if ord("A") <= v[4] <= ord("Z")]; self.config.discovery_status.setText(f"{len(self.discovered_boards)} placa(s) · IDs: {', '.join(sorted(set(ids))) or 'nenhum'}")
        for value in self.discovered_boards: self.config.boards.addItem(f"{value[2].hex().upper()}    ID: {chr(value[4]) if ord('A') <= value[4] <= ord('Z') else 'NÃO CONFIGURADO'}    geração: {value[5]}")

    def select_board(self, index):
        if 0 <= index < len(self.discovered_boards):
            value = self.discovered_boards[index]; self.config.uid.setText(value[2].hex().upper()); self.config.set_actions_enabled(True)
            if ord("A") <= value[4] <= ord("Z"): self.config.robot_id.setCurrentText(chr(value[4]))

    def configuration_succeeded(self, robot_id): self.config.robot_id.setCurrentText(robot_id); self.toast(f"Robot ID {robot_id} saved", "success")
    def motion_configuration_succeeded(self, linear, angular): self.toast(f"Motion limits saved · {linear:.2f} m/s² · {angular:.2f} rad/s²", "success")

    def generate_plots(self, path):
        if not path: self.show_error("Análise", "Selecione um arquivo CSV"); return
        self.analysis.status.setText("Gerando gráficos…"); threading.Thread(target=self._plot_worker, args=(path,), daemon=True).start()

    def _plot_worker(self, path):
        try:
            import matplotlib.pyplot as plt
            import pandas as pd
            csv_path = Path(path); output = csv_path.parent / f"{csv_path.stem}_plots"; output.mkdir(parents=True, exist_ok=True); frame = pd.read_csv(csv_path)
            if frame.empty: raise ValueError("O CSV está vazio")
            timeline = (frame["mcu_time_ms"] - frame["mcu_time_ms"].iloc[0]) / 1000.0
            for i in range(1, 5): frame[f"rpm{i}"] = frame[f"rpm{i}_x10"] / 10.0
            frame["battery_v"] = frame["battery_mV"] / 1000.0
            for name, columns in (("rpm.png", [f"rpm{i}" for i in range(1, 5)]), ("cmd.png", [f"cmd{i}" for i in range(1, 5)]), ("battery.png", ["battery_v"])):
                plt.figure(figsize=(12, 5)); [plt.plot(timeline, frame[column], label=column) for column in columns]; plt.grid(True, alpha=.3); plt.legend(); plt.tight_layout(); plt.savefig(output / name, dpi=140); plt.close()
            self.plots_ready.emit(f"{len(frame)} amostras · {output}")
        except Exception as exc: self.plots_ready.emit(f"Erro: {exc}")

    def on_plots_ready(self, message):
        self.analysis.status.setText(message)
        if not message.startswith("Erro:") and hasattr(os, "startfile"): os.startfile(message.split(" · ", 1)[1])

    def toast(self, message, level="info"):
        colors = {"info": "#2bc3dc", "success": "#43d39e", "warning": "#ffb454", "error": "#ff6577"}
        if not hasattr(self, "toast_label"): self.toast_label = QLabel(self); self.toast_label.setMinimumWidth(320); self.toast_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.toast_label.setText(message); self.toast_label.setStyleSheet(f"background:#111d2e;color:white;border-left:4px solid {colors[level]};padding:12px;border-radius:6px;font-weight:600"); self.toast_label.adjustSize(); self.toast_label.move(self.width() - self.toast_label.width() - 28, 76); self.toast_label.show(); self.toast_label.raise_(); QTimer.singleShot(3200, self.toast_label.hide)

    def show_error(self, title, message): self.toast(f"{title}: {message}", "error"); QMessageBox.critical(self, title, message)

    def closeEvent(self, event):
        self.manager.disconnect_serial()
        if self.log_file: self.log_file.close()
        if self.link_log_file: self.link_log_file.close()
        event.accept()
