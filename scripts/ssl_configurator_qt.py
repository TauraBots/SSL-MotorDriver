"""Professional Qt 6 interface for ssl-configurator.py."""

import csv
import os
import struct
import threading
import time
from pathlib import Path

import serial
from serial.tools import list_ports
from PySide6.QtCore import QObject, QEvent, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QPalette
from PySide6.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QFrame, QGridLayout, QHBoxLayout,
    QLabel, QLineEdit, QListWidget, QMainWindow, QMessageBox, QPushButton,
    QSpinBox, QTabWidget, QVBoxLayout, QWidget,
)


STYLE = """
QWidget { background: #09111f; color: #edf4ff; font-family: 'Segoe UI'; font-size: 10pt; }
QMainWindow { background: #09111f; }
QFrame#card { background: #111d30; border: 1px solid #263854; border-radius: 12px; }
QLabel#title { color: #36d7ef; font-size: 21pt; font-weight: 700; }
QLabel#section { font-size: 12pt; font-weight: 700; }
QLabel#muted { color: #91a6c1; }
QLabel#metric { background: #182740; border-radius: 8px; padding: 10px; font-size: 13pt; font-weight: 700; }
QLabel#key { background: #182740; border: 1px solid #2b4161; border-radius: 8px; padding: 13px; font-weight: 700; }
QLabel#key[pressed="true"] { background: #36d7ef; color: #061018; }
QPushButton { background: #1a2a43; border: 1px solid #304766; border-radius: 8px; padding: 9px 16px; font-weight: 600; }
QPushButton:hover { background: #243957; border-color: #36d7ef; }
QPushButton:pressed { background: #16243a; }
QPushButton:disabled { color: #62738a; background: #121d2d; }
QPushButton[accent="true"] { background: #36d7ef; color: #061018; border: none; }
QPushButton[accent="true"]:hover { background: #62e5f7; }
QLineEdit, QComboBox, QSpinBox, QListWidget { background: #182740; border: 1px solid #304766; border-radius: 7px; padding: 7px; selection-background-color: #36d7ef; selection-color: #061018; }
QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QListWidget:focus { border-color: #36d7ef; }
QTabWidget::pane { border: none; top: -1px; }
QTabBar::tab { background: #111d30; color: #91a6c1; padding: 11px 20px; margin-right: 3px; border-top-left-radius: 7px; border-top-right-radius: 7px; }
QTabBar::tab:selected { background: #182740; color: #36d7ef; }
"""


class WorkerSignals(QObject):
    boards = Signal(object, int)
    configured = Signal(str)
    error = Signal(str, str)
    finished = Signal()
    plots_ready = Signal(str)


class QtConfiguratorApp(QMainWindow):
    def __init__(self, args, backend):
        self.app = QApplication.instance() or QApplication([])
        super().__init__()
        self.b = backend
        self.ser = None
        self.connected_robot_id = None
        self.running = False
        self.keys = set()
        self.sequence = int(time.time() * 1000) & 0xFFFFFFFF
        self.request_sequence = 0
        self.next_command = self.next_telemetry = 0.0
        self.rx = bytearray()
        self.kick_pending = False
        self.last_motion = time.monotonic()
        self.last_telemetry_received = 0.0
        self.vx = self.vy = self.omega = 0.0
        self.log_file = self.log_writer = None
        self.config_busy = False
        self.discovered_boards = []
        self.signals = WorkerSignals()
        self.signals.boards.connect(self.show_boards)
        self.signals.configured.connect(self.configuration_succeeded)
        self.signals.error.connect(self.show_error)
        self.signals.finished.connect(self.finish_config_action)
        self.signals.plots_ready.connect(self.plots_ready)

        self.setWindowTitle("SSL Configurator · TauraBots")
        self.resize(1120, 740)
        self.setMinimumSize(920, 640)
        self.setStyleSheet(STYLE)
        self._build(args)
        self.refresh_ports()
        self.app.installEventFilter(self)
        self.io_timer = QTimer(self)
        self.io_timer.setInterval(10)
        self.io_timer.timeout.connect(self.io_tick)

    def label(self, text="", kind=None):
        widget = QLabel(text)
        if kind:
            widget.setObjectName(kind)
        return widget

    def button(self, text, slot, accent=False):
        widget = QPushButton(text)
        widget.setProperty("accent", accent)
        widget.clicked.connect(slot)
        widget.setCursor(Qt.CursorShape.PointingHandCursor)
        return widget

    def card(self):
        frame = QFrame()
        frame.setObjectName("card")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(22, 20, 22, 20)
        layout.setSpacing(12)
        return frame, layout

    def _build(self, args):
        root = QWidget()
        outer = QVBoxLayout(root)
        outer.setContentsMargins(28, 22, 28, 28)
        outer.setSpacing(16)
        header = QHBoxLayout()
        header.addWidget(self.label("SSL CONFIGURATOR", "title"))
        header.addWidget(self.label("TauraBots · Motor Driver", "muted"))
        header.addStretch()
        self.status_badge = self.label("●  DESCONECTADO")
        self.status_badge.setStyleSheet("color: #ff6577; font-weight: 700")
        header.addWidget(self.status_badge)
        outer.addLayout(header)

        connection, row = self.card()
        row.setDirection(QVBoxLayout.Direction.LeftToRight)
        row.addWidget(self.label("Porta", "muted"))
        self.port_box = QComboBox(); self.port_box.setEditable(True); self.port_box.setMinimumWidth(170)
        if args.port: self.port_box.setCurrentText(args.port)
        row.addWidget(self.port_box)
        row.addWidget(self.label("Baud", "muted"))
        self.baud_box = QComboBox(); self.baud_box.setEditable(True)
        self.baud_box.addItems(["9600", "115200", "1000000"]); self.baud_box.setCurrentText(str(args.baud))
        row.addWidget(self.baud_box)
        row.addWidget(self.button("↻", self.refresh_ports))
        row.addStretch()
        self.connect_button = self.button("CONECTAR", self.toggle_connection, True)
        row.addWidget(self.connect_button)
        outer.addWidget(connection)

        tabs = QTabWidget()
        tabs.addTab(self._control_tab(), "CONTROLE E TELEMETRIA")
        tabs.addTab(self._config_tab(), "CONFIGURAÇÃO DA PLACA")
        tabs.addTab(self._analysis_tab(), "ANÁLISE CSV")
        outer.addWidget(tabs, 1)
        self.setCentralWidget(root)

    def _control_tab(self):
        tab = QWidget(); layout = QHBoxLayout(tab); layout.setContentsMargins(0, 16, 0, 0); layout.setSpacing(16)
        left, control = self.card(); right, telemetry = self.card()
        control.addWidget(self.label("CONTROLE", "section"))
        options = QHBoxLayout(); options.addWidget(self.label("Robô", "muted"))
        self.robot_box = QComboBox(); self.robot_box.addItems(list("ABCDEFGHIJKLMNOPQRSTUVWXYZ")); options.addWidget(self.robot_box)
        options.addSpacing(18); options.addWidget(self.label("Kick %", "muted"))
        self.kick_spin = QSpinBox(); self.kick_spin.setRange(0, 100); self.kick_spin.setValue(80); self.kick_spin.setSuffix(" %"); options.addWidget(self.kick_spin); options.addStretch()
        control.addLayout(options)
        pad = QGridLayout(); self.key_labels = {}
        for key, text, r, c in (("q", "Q  ↺", 0, 0), ("w", "W  ↑", 0, 1), ("e", "E  ↻", 0, 2), ("a", "A  ←", 1, 0), ("s", "S  ↓", 1, 1), ("d", "D  →", 1, 2)):
            label = self.label(text, "key"); label.setAlignment(Qt.AlignmentFlag.AlignCenter); label.setProperty("pressed", False)
            pad.addWidget(label, r, c); self.key_labels[key] = label
        control.addLayout(pad); control.addWidget(self.label("SPACE freia · K envia um chute", "muted"), alignment=Qt.AlignmentFlag.AlignCenter)
        control.addStretch(); control.addWidget(self.button("CHUTAR", self.queue_kick, True))

        telemetry.addWidget(self.label("TELEMETRIA AO VIVO", "section"))
        metrics = QHBoxLayout(); self.comm_label = self.label("—", "metric"); self.seq_label = self.label("—", "metric"); self.battery_label = self.label("—", "metric")
        for title, value in (("COM", self.comm_label), ("SEQ", self.seq_label), ("BATERIA", self.battery_label)):
            box = QVBoxLayout(); box.addWidget(self.label(title, "muted")); box.addWidget(value); metrics.addLayout(box)
        telemetry.addLayout(metrics); self.rpm_labels = []
        for index in range(1, 5):
            line = QHBoxLayout(); line.addWidget(self.label(f"Motor {index}", "muted")); line.addStretch(); value = self.label("0", "section"); line.addWidget(value); line.addWidget(self.label("RPM", "muted")); telemetry.addLayout(line); self.rpm_labels.append(value)
        telemetry.addStretch(); self.log_button = self.button("GRAVAR CSV", self.toggle_log); telemetry.addWidget(self.log_button)
        self.log_label = self.label("CSV desligado", "muted"); self.log_label.setWordWrap(True); telemetry.addWidget(self.log_label)
        layout.addWidget(left, 1); layout.addWidget(right, 1); return tab

    def _config_tab(self):
        tab = QWidget(); layout = QVBoxLayout(tab); layout.setContentsMargins(0, 16, 0, 0)
        panel, content = self.card(); content.addWidget(self.label("IDENTIDADE DA PLACA", "section")); content.addWidget(self.label("Descubra o UID físico e associe um ID lógico A–Z.", "muted"))
        self.discover_button = self.button("DESCOBRIR PLACAS", self.discover, True); content.addWidget(self.discover_button, alignment=Qt.AlignmentFlag.AlignLeft)
        self.discovery_label = self.label("Nenhuma descoberta executada", "muted"); content.addWidget(self.discovery_label)
        self.board_list = QListWidget(); self.board_list.setMinimumHeight(180); self.board_list.currentRowChanged.connect(self.select_board); content.addWidget(self.board_list)
        row = QHBoxLayout(); row.addWidget(self.label("UID", "muted")); self.uid_edit = QLineEdit(); self.uid_edit.setPlaceholderText("24 caracteres hexadecimais"); row.addWidget(self.uid_edit, 1)
        row.addWidget(self.label("Novo ID", "muted")); self.new_id_box = QComboBox(); self.new_id_box.addItems(list("ABCDEFGHIJKLMNOPQRSTUVWXYZ")); row.addWidget(self.new_id_box)
        self.set_id_button = self.button("SALVAR ID", self.set_id); row.addWidget(self.set_id_button); content.addLayout(row); layout.addWidget(panel); return tab

    def _analysis_tab(self):
        tab = QWidget(); layout = QVBoxLayout(tab); layout.setContentsMargins(0, 16, 0, 0)
        panel, content = self.card(); content.addWidget(self.label("ANÁLISE DE TELEMETRIA", "section")); content.addWidget(self.label("Gere gráficos de RPM, comandos e bateria a partir de um CSV.", "muted"))
        row = QHBoxLayout(); self.csv_edit = QLineEdit(); self.csv_edit.setPlaceholderText("Selecione um arquivo de telemetria .csv"); row.addWidget(self.csv_edit, 1); row.addWidget(self.button("ESCOLHER CSV", self.choose_analysis_csv)); content.addLayout(row)
        content.addWidget(self.button("GERAR GRÁFICOS", self.generate_plots, True), alignment=Qt.AlignmentFlag.AlignLeft)
        self.analysis_label = self.label("Selecione um arquivo gravado pelo configurador.", "muted"); self.analysis_label.setWordWrap(True); content.addWidget(self.analysis_label); content.addStretch()
        content.addWidget(self.label("Saídas: rpm.png · cmd.png · battery.png · rpm_vs_cmd_m1.png", "muted")); layout.addWidget(panel); return tab

    def refresh_ports(self):
        current = self.port_box.currentText() if hasattr(self, "port_box") else ""
        ports = [item.device for item in list_ports.comports()]
        self.port_box.clear(); self.port_box.addItems(ports)
        self.port_box.setCurrentText(current or (ports[0] if ports else ""))

    def serial_settings(self):
        port = self.port_box.currentText().strip()
        if not port: raise ValueError("Selecione uma porta serial")
        return port, int(self.baud_box.currentText())

    def toggle_connection(self):
        if self.ser: self.disconnect(); return
        try:
            robot_id = self.robot_box.currentText(); port, baud = self.serial_settings()
            self.ser = serial.Serial(port, baud, timeout=0); self.ser.reset_input_buffer(); self.connected_robot_id = robot_id
            self.rx.clear(); self.request_sequence = 0; now = time.monotonic(); self.next_command = now; self.next_telemetry = now + .1; self.last_motion = now
            self.vx = self.vy = self.omega = 0.; self.last_telemetry_received = 0.; self.comm_label.setText("AGUARDANDO"); self.running = True
            self.status_badge.setText("●  CONECTADO"); self.status_badge.setStyleSheet("color:#42d392;font-weight:700"); self.connect_button.setText("DESCONECTAR"); self.connect_button.setProperty("accent", False); self.connect_button.style().polish(self.connect_button); self.robot_box.setEnabled(False); self.io_timer.start(); self.setFocus()
        except Exception as exc: self.show_error("Conexão", str(exc))

    def disconnect(self):
        self.running = False; self.io_timer.stop()
        if self.ser:
            try:
                self.sequence = (self.sequence + 1) & 0xFFFFFFFF
                if self.connected_robot_id: self.ser.write(self.b.encode_robot_velocity_packet(self.connected_robot_id, self.sequence, 0., 0., 0., brake=1))
                self.ser.close()
            except Exception:
                try: self.ser.close()
                except Exception: pass
        self.ser = None; self.connected_robot_id = None; self.keys.clear()
        for label in self.key_labels.values(): self._set_key(label, False)
        self.status_badge.setText("●  DESCONECTADO"); self.status_badge.setStyleSheet("color:#ff6577;font-weight:700"); self.connect_button.setText("CONECTAR"); self.connect_button.setProperty("accent", True); self.connect_button.style().polish(self.connect_button); self.robot_box.setEnabled(True)

    def eventFilter(self, obj, event):
        if event.type() in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease) and not event.isAutoRepeat():
            keymap = {Qt.Key.Key_W:"w", Qt.Key.Key_A:"a", Qt.Key.Key_S:"s", Qt.Key.Key_D:"d", Qt.Key.Key_Q:"q", Qt.Key.Key_E:"e", Qt.Key.Key_Space:"space", Qt.Key.Key_K:"k"}; key = keymap.get(event.key())
            if key:
                if event.type() == QEvent.Type.KeyPress:
                    if key == "k": self.queue_kick()
                    else: self.keys.add(key)
                else: self.keys.discard(key)
                if key in self.key_labels: self._set_key(self.key_labels[key], event.type() == QEvent.Type.KeyPress)
                return True
        return super().eventFilter(obj, event)

    def _set_key(self, label, pressed):
        label.setProperty("pressed", pressed); label.style().unpolish(label); label.style().polish(label)

    def queue_kick(self): self.kick_pending = True

    def motion_command(self, now):
        vx = (2. if "d" in self.keys else 0.) - (2. if "a" in self.keys else 0.); vy = (2. if "w" in self.keys else 0.) - (2. if "s" in self.keys else 0.); omega = (5. if "q" in self.keys else 0.) - (5. if "e" in self.keys else 0.)
        if "space" in self.keys: vx = vy = omega = 0.
        elapsed = max(0., now - self.last_motion); self.last_motion = now; alpha = 1. - self.b.math.exp(-elapsed / (.08 if vx == vy == omega == 0. else .18))
        self.vx += (vx-self.vx)*alpha; self.vy += (vy-self.vy)*alpha; self.omega += (omega-self.omega)*alpha
        if abs(self.vx) < 1.e-4: self.vx = 0.
        if abs(self.vy) < 1.e-4: self.vy = 0.
        if abs(self.omega) < 1.e-4: self.omega = 0.
        return self.vx, self.vy, self.omega

    def io_tick(self):
        if not self.running or not self.ser: return
        try:
            now = time.monotonic(); robot_id = self.connected_robot_id
            if now >= self.next_command:
                vx, vy, omega = self.motion_command(now); self.sequence = (self.sequence + 1) & 0xFFFFFFFF; kick = self.kick_spin.value() if self.kick_pending else 0
                self.ser.write(self.b.encode_robot_velocity_packet(robot_id, self.sequence, vx, vy, omega, kick, int(vx == vy == omega == 0.))); self.kick_pending = False; self.next_command = now + .05
            if now >= self.next_telemetry:
                self.request_sequence = (self.request_sequence + 1) & 0xFFFF; self.ser.write(self.b.encode_telemetry_request(robot_id, self.request_sequence)); self.next_telemetry = now + .2; self.next_command = max(self.next_command, now + self.b.TELEMETRY_REPLY_WINDOW_S)
            if self.ser.in_waiting:
                self.rx.extend(self.ser.read(self.ser.in_waiting)); telemetry = self.b.parse_telemetry(self.rx, robot_id)
                if telemetry: self.update_telemetry(telemetry)
            if self.last_telemetry_received and now-self.last_telemetry_received > 1.: self.comm_label.setText("SEM DADOS")
        except Exception as exc: self.disconnect(); self.show_error("Comunicação", str(exc))

    def update_telemetry(self, telemetry):
        self.last_telemetry_received = time.monotonic(); self.comm_label.setText("OK" if telemetry["comm_ok"] else "LOST"); self.seq_label.setText(str(telemetry["command_sequence"])); self.battery_label.setText(f'{telemetry["battery_v"]:.2f} V')
        for label, rpm in zip(self.rpm_labels, telemetry["rpm"]): label.setText(f"{rpm:+.0f}")
        if self.log_writer: self.log_writer.writerow(self.b.telemetry_csv_row(telemetry)); self.log_file.flush()

    def toggle_log(self):
        if self.log_file: self.log_file.close(); self.log_file = self.log_writer = None; self.log_label.setText("CSV desligado"); self.log_button.setText("GRAVAR CSV"); return
        path, _ = QFileDialog.getSaveFileName(self, "Gravar telemetria", "telemetry.csv", "CSV (*.csv)")
        if path:
            self.log_file = open(path, "w", newline="", encoding="utf-8"); self.log_writer = csv.writer(self.log_file); self.log_writer.writerow(self.b.CSV_HEADER); self.log_label.setText(path); self.log_button.setText("PARAR GRAVAÇÃO")

    def choose_analysis_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "Abrir telemetria", "", "CSV (*.csv);;Todos (*.*)")
        if path: self.csv_edit.setText(path)

    def generate_plots(self):
        path = self.csv_edit.text().strip()
        if not path: self.show_error("Análise", "Selecione um arquivo CSV"); return
        self.analysis_label.setText("Gerando gráficos…"); threading.Thread(target=self.plot_worker, args=(path,), daemon=True).start()

    def plot_worker(self, path):
        try:
            import matplotlib.pyplot as plt
            import pandas as pd
            csv_path = Path(path); output = csv_path.parent / f"{csv_path.stem}_plots"; output.mkdir(parents=True, exist_ok=True); frame = pd.read_csv(csv_path)
            if frame.empty: raise ValueError("O CSV está vazio")
            frame["mcu_time_ms"] = pd.to_numeric(frame["mcu_time_ms"], errors="coerce"); frame = frame.dropna(subset=["mcu_time_ms"]).copy()
            if frame.empty: raise ValueError("O CSV não contém amostras válidas")
            timeline = (frame["mcu_time_ms"]-frame["mcu_time_ms"].iloc[0])/1000.
            for i in range(1,5): frame[f"rpm{i}"] = pd.to_numeric(frame[f"rpm{i}_x10"], errors="coerce")/10.; frame[f"cmd{i}"] = pd.to_numeric(frame[f"cmd{i}"], errors="coerce")
            frame["battery_v"] = pd.to_numeric(frame["battery_mV"], errors="coerce")/1000.
            for name,title,ylabel,columns in (("rpm.png","Velocidade dos motores","RPM",[f"rpm{i}" for i in range(1,5)]),("cmd.png","Comandos dos motores","Comando",[f"cmd{i}" for i in range(1,5)]),("battery.png","Tensão da bateria","Tensão [V]",["battery_v"]),("rpm_vs_cmd_m1.png","Motor 1: RPM × comando","Valor",["rpm1","cmd1"])):
                plt.figure(figsize=(12,5))
                for column in columns: plt.plot(timeline, frame[column], label=column, linewidth=1.2)
                plt.title(title); plt.xlabel("Tempo [s]"); plt.ylabel(ylabel); plt.grid(True, alpha=.3); plt.legend(loc="best"); plt.tight_layout(); plt.savefig(output/name, dpi=140); plt.close()
            self.signals.plots_ready.emit(f"{len(frame)} amostras · {output}")
        except Exception as exc: self.signals.error.emit("Análise", str(exc)); self.signals.plots_ready.emit(f"Erro: {exc}")

    def plots_ready(self, message):
        self.analysis_label.setText(message)
        if not message.startswith("Erro:"): os.startfile(message.split(" · ", 1)[1])

    def discover(self): self.board_list.clear(); self.board_list.addItem("Procurando placas…"); self.discovery_label.setText("Descoberta em andamento…"); self.begin_config_action("discover")

    def begin_config_action(self, action, uid=None, robot_id=None):
        if self.config_busy: return
        if self.ser: self.disconnect()
        try:
            port, baud = self.serial_settings()
        except Exception as exc:
            self.show_error("Configuração", str(exc)); return
        self.config_busy = True; self.discover_button.setEnabled(False); self.discover_button.setText("AGUARDE…"); self.set_id_button.setEnabled(False)
        QTimer.singleShot(150, lambda: threading.Thread(target=self.config_worker, args=(action, port, baud, uid, robot_id), daemon=True).start())

    def config_worker(self, action, port, baud, uid, robot_id):
        try:
            with serial.Serial(port, baud, timeout=.05) as ser:
                time.sleep(.25); ser.reset_input_buffer()
                if action == "discover":
                    values=[]; received=0
                    for attempt in range(2):
                        payload=struct.pack("<HBI",0xAA55,self.b.CONFIG_DISCOVER_TYPE,(int(time.time()*1000)+attempt)&0xFFFFFFFF); ser.write(self.b.packet_with_crc(payload)); ser.flush(); values.extend(self.b.read_config_responses(ser,1.2)); received += self.b.read_config_responses.last_rx_bytes
                    self.signals.boards.emit(values, received)
                else:
                    target=self.b.parse_uid(uid); payload=struct.pack("<HB12sBI",0xAA55,self.b.CONFIG_SET_ID_TYPE,target,ord(robot_id),self.b.CONFIG_KEY); ser.write(self.b.packet_with_crc(payload)); ser.flush(); values=self.b.read_config_responses(ser,1.)
                    reply=next((v for v in values if v[1]==self.b.CONFIG_SET_ID_RESPONSE_TYPE and v[2]==target),None)
                    if reply is None: raise RuntimeError("A placa não respondeu ao pedido de configuração")
                    if reply[3]==0: raise RuntimeError("A placa recebeu o pedido, mas não conseguiu gravar o ID na Flash.")
                    self.signals.configured.emit(robot_id)
        except Exception as exc: self.signals.error.emit("Configuração", str(exc))
        finally: self.signals.finished.emit()

    def finish_config_action(self): self.config_busy=False; self.discover_button.setEnabled(True); self.discover_button.setText("DESCOBRIR PLACAS"); self.set_id_button.setEnabled(True)

    def show_boards(self, values, received=0):
        self.board_list.clear(); unique={v[2]:v for v in values if v[1]==self.b.CONFIG_DISCOVER_RESPONSE_TYPE}; self.discovered_boards=list(unique.values())
        if not self.discovered_boards: self.board_list.addItem(f"Nenhuma resposta válida · {received} byte(s) recebidos"); self.discovery_label.setText("0 placas encontradas"); return
        ids=[chr(v[4]) for v in self.discovered_boards if ord("A")<=v[4]<=ord("Z")]; duplicates=sorted(i for i in set(ids) if ids.count(i)>1); summary=f"{len(self.discovered_boards)} placa(s) · IDs: {', '.join(sorted(set(ids))) or 'nenhum'}"
        if duplicates: summary += f" · IDs DUPLICADOS: {', '.join(duplicates)}"
        self.discovery_label.setText(summary)
        for v in self.discovered_boards: self.board_list.addItem(f"{v[2].hex().upper()}    ID: {chr(v[4]) if ord('A')<=v[4]<=ord('Z') else 'NÃO CONFIGURADO'}    geração: {v[5]}")

    def select_board(self, index):
        if 0 <= index < len(self.discovered_boards):
            value=self.discovered_boards[index]; self.uid_edit.setText(value[2].hex().upper())
            if ord("A")<=value[4]<=ord("Z"): self.robot_box.setCurrentText(chr(value[4])); self.new_id_box.setCurrentText(chr(value[4]))

    def set_id(self):
        try: self.b.parse_uid(self.uid_edit.text().strip())
        except ValueError as exc: self.show_error("Configuração", str(exc)); return
        self.begin_config_action("set-id", self.uid_edit.text().strip(), self.new_id_box.currentText())

    def configuration_succeeded(self, robot_id): self.robot_box.setCurrentText(robot_id); self.new_id_box.setCurrentText(robot_id); QMessageBox.information(self, "Configuração", f"Robô {robot_id} configurado com sucesso.")
    def show_error(self, title, message): QMessageBox.critical(self, title, message)
    def closeEvent(self, event):
        self.disconnect()
        if self.log_file: self.log_file.close()
        event.accept()
    def run(self): self.show(); return self.app.exec()
