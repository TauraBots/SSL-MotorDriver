"""Embedded PID auto-tune controls and live result presentation."""

import time

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (QComboBox, QGridLayout, QHBoxLayout, QLabel,
                               QProgressBar, QPushButton, QScrollArea,
                               QVBoxLayout, QWidget)

from widgets import CardWidget, metric_card


class AutoTunePanel(QWidget):
    AUTOTUNE_MINIMUM_S = 8.0
    AUTOTUNE_TIMEOUT_S = 30.0
    start_requested = Signal(int, int)
    abort_requested = Signal()
    restore_requested = Signal()
    commit_requested = Signal(int)

    MODE_SAVE = 1
    MODE_PREVIEW = 3
    MODE_STAGE = 4

    STATE_NAMES = {0: "IDLE", 1: "RUNNING", 2: "FINISHED", 3: "FAILED"}
    ERROR_NAMES = {
        0: "NONE",
        1: "INVALID MOTOR",
        2: "BATTERY LOW",
        3: "ENCODER NO MOVEMENT",
        4: "TIMEOUT",
        5: "COMMUNICATION LOST",
        6: "INVALID OSCILLATION",
        7: "OVERSPEED",
        8: "FLASH WRITE",
        9: "ABORTED",
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self._available = False
        self._running = False
        self._motor_started_at = None
        self._active_motor = 0
        self._terminal_latched = False
        self._requested_mode = self.MODE_PREVIEW

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        scroll.setWidget(content)
        outer.addWidget(scroll)
        layout = QVBoxLayout(content)
        layout.setContentsMargins(28, 24, 28, 24)
        layout.setSpacing(16)

        title = QLabel("AUTO-TUNE PID")
        title.setObjectName("pageTitle")
        layout.addWidget(title)
        subtitle = QLabel("Relay Auto-Tune embarcado · calibração PI individual por motor")
        subtitle.setObjectName("muted")
        layout.addWidget(subtitle)

        warning = CardWidget()
        warning_title = QLabel("CONDIÇÃO DE SEGURANÇA")
        warning_title.setObjectName("section")
        warning_text = QLabel(
            "Mantenha o robô suspenso, sem contato das rodas com o piso. "
            "O motor selecionado alternará o sentido automaticamente. "
            "Para ganhos finais, calibre com as rodas instaladas."
        )
        warning_text.setWordWrap(True)
        warning_text.setObjectName("muted")
        warning.content.addWidget(warning_title)
        warning.content.addWidget(warning_text)
        layout.addWidget(warning)

        controls = CardWidget()
        control_row = QHBoxLayout()
        self.robot = QLabel("ROBOT —")
        self.robot.setObjectName("controlTarget")
        self.motor = QComboBox()
        self.motor.addItem("Motor 1", 1)
        self.motor.addItem("Motor 2", 2)
        self.motor.addItem("Motor 3", 3)
        self.motor.addItem("Motor 4", 4)
        self.motor.addItem("Todos · 1 → 4", 0)
        self.mode = QComboBox()
        self.mode.addItem("Prévia segura · não salva", self.MODE_PREVIEW)
        self.mode.addItem("Aplicar somente em RAM", self.MODE_STAGE)
        self.mode.addItem("Aplicar e salvar na Flash", self.MODE_SAVE)
        self.start_button = QPushButton("INICIAR AUTO-TUNE")
        self.start_button.setProperty("accent", True)
        self.start_button.clicked.connect(self._request_start)
        self.abort_button = QPushButton("ABORTAR")
        self.abort_button.setProperty("danger", True)
        self.abort_button.clicked.connect(self.abort_requested)
        self.restore_button = QPushButton("RESTAURAR FLASH")
        self.restore_button.clicked.connect(self.restore_requested)
        self.commit_button = QPushButton("SALVAR RAM")
        self.commit_button.clicked.connect(
            lambda: self.commit_requested.emit(int(self.motor.currentData())))
        control_row.addWidget(self.robot)
        control_row.addWidget(QLabel("Motor"))
        control_row.addWidget(self.motor)
        control_row.addWidget(QLabel("Modo"))
        control_row.addWidget(self.mode)
        control_row.addStretch()
        control_row.addWidget(self.restore_button)
        control_row.addWidget(self.commit_button)
        control_row.addWidget(self.start_button)
        control_row.addWidget(self.abort_button)
        controls.content.addLayout(control_row)

        status_row = QHBoxLayout()
        self.state = QLabel("IDLE")
        self.state.setObjectName("autotuneState")
        self.detail = QLabel("Selecione um robô online na Fleet.")
        self.detail.setObjectName("muted")
        status_row.addWidget(self.state)
        status_row.addWidget(self.detail, 1)
        controls.content.addLayout(status_row)
        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.progress.setTextVisible(False)
        controls.content.addWidget(self.progress)
        layout.addWidget(controls)

        metrics = QGridLayout()
        self.metrics = {}
        for index, (key, label) in enumerate((
                ("tu", "Tu · período"), ("ku", "Ku · ganho último"),
                ("kp", "Kp"), ("ki", "Ki"), ("kd", "Kd"))):
            card, value = metric_card(label)
            self.metrics[key] = value
            metrics.addWidget(card, index // 3, index % 3)
        layout.addLayout(metrics)

        motor_card = CardWidget()
        motor_title = QLabel("RESPOSTA DOS MOTORES")
        motor_title.setObjectName("section")
        motor_card.content.addWidget(motor_title)
        grid = QGridLayout()
        self.motor_values = []
        for index in range(4):
            heading = QLabel(f"M{index + 1}")
            heading.setObjectName("eyebrow")
            value = QLabel("0.0 RPM · PWM 0")
            value.setObjectName("autotuneMotorValue")
            grid.addWidget(heading, index, 0, Qt.AlignmentFlag.AlignLeft)
            grid.addWidget(value, index, 1, Qt.AlignmentFlag.AlignLeft)
            self.motor_values.append(value)
        grid.setColumnStretch(0, 0)
        grid.setColumnStretch(1, 1)
        motor_card.content.addLayout(grid)
        layout.addWidget(motor_card)
        layout.addStretch()
        self.set_available(None)

    def set_available(self, robot_id):
        self._available = bool(robot_id)
        self.robot.setText(f"ROBOT {robot_id}" if robot_id else "ROBOT —")
        if not self._running and not self._terminal_latched:
            self.detail.setText("Pronto para calibrar." if robot_id else
                                "Selecione um robô online na Fleet.")
        self._update_actions()

    def set_running(self, robot_id, motor_id):
        self._running = True
        # Start the visible timer only after telemetry confirms RUNNING. Radio
        # discovery/terminal acknowledgement can delay the actual firmware start.
        self._motor_started_at = None
        self._active_motor = 0
        self._terminal_latched = False
        self.robot.setText(f"ROBOT {robot_id}")
        self._set_state("STARTING", "running")
        target = "todos os motores" if motor_id == 0 else f"motor {motor_id}"
        self.detail.setText(f"Iniciando {target}…")
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self._update_actions()

    def set_restored(self):
        self._terminal_latched = True
        self._set_state("RESTORED", "success")
        self.detail.setText("Ganhos ativos restaurados da Flash; nenhum valor foi gravado.")
        self._update_actions()

    def set_commit_requested(self):
        self._terminal_latched = True
        self._set_state("COMMIT", "success")
        self.detail.setText(
            "Solicitação para salvar o candidato RAM enviada; confirme a geração na Fleet.")
        self._update_actions()

    def set_aborted(self):
        self._running = False
        self._motor_started_at = None
        self._active_motor = 0
        self._terminal_latched = True
        self._set_state("ABORTED", "warning")
        self.detail.setText("Ensaio interrompido; motores freados.")
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self._update_actions()

    def reset(self):
        self._running = False
        self._motor_started_at = None
        self._active_motor = 0
        self._terminal_latched = False
        self._set_state("IDLE", "idle")
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        for value in self.metrics.values():
            value.setText("—")
        for value in self.motor_values:
            value.setText("0.0 RPM · PWM 0")
        self.set_available(None)

    def update_telemetry(self, data):
        rpm = data.get("rpm", ())
        pwm = data.get("cmd", ())
        if len(rpm) == 4 and len(pwm) == 4:
            for index, value in enumerate(self.motor_values):
                value.setText(f"{rpm[index]:+.1f} RPM · PWM {pwm[index]:+d}")

        tune = data.get("autotune")
        if not tune:
            return
        state = int(tune.get("state", 0))
        error = int(tune.get("error", 0))
        active = bool(tune.get("active", 0))
        motor = int(tune.get("motor", 0))
        self._running = active
        if active:
            self._terminal_latched = False
            now = time.monotonic()
            if self._motor_started_at is None or motor != self._active_motor:
                self._motor_started_at = now
                self._active_motor = motor
            detail = data.get("autotune_detail", {})
            elapsed = (float(detail["elapsed_ms"]) * 0.001
                       if "elapsed_ms" in detail else
                       max(0.0, now - self._motor_started_at))
            usable = int(detail.get("usable_periods", 0))
            completed = int(detail.get("completed_periods", 0))
            stable_windows = int(detail.get("stable_windows", 0))
            period_spread = 100.0 * float(detail.get("period_spread", 0.0))
            peak_spread = 100.0 * max(
                float(detail.get("high_peak_spread", 0.0)),
                float(detail.get("low_peak_spread", 0.0)))
            self._set_state("RUNNING", "running")
            diagnostic_text = (f" · ciclos {usable}/8 (total {completed})"
                               f" · confirmação {stable_windows}/3"
                               f" · σTu {period_spread:.1f}% · σpico {peak_spread:.1f}%"
                               if detail else "")
            self.detail.setText(
                f"Motor {motor} · {elapsed:.1f} / {self.AUTOTUNE_TIMEOUT_S:.0f} s"
                f" · mínimo {self.AUTOTUNE_MINIMUM_S:.0f} s{diagnostic_text}")
            self.progress.setRange(0, 100)
            self.progress.setValue(min(100, int(
                100.0 * elapsed / self.AUTOTUNE_TIMEOUT_S)))
        else:
            self._motor_started_at = None
            self._active_motor = 0
            if state == 0 and error == 0 and self._terminal_latched:
                # Firmware returns to IDLE after the host acknowledges a
                # terminal result. Preserve that result until the next run.
                self._update_actions()
                return
            self.progress.setRange(0, 100)
            self.progress.setValue(100 if state == 2 else 0)
            if state == 2:
                self._terminal_latched = True
                self._set_state("FINISHED", "success")
                messages = {
                    self.MODE_PREVIEW: "medido; ganhos não aplicados nem salvos.",
                    self.MODE_STAGE: "medido e aplicado somente em RAM.",
                    self.MODE_SAVE: "calibrado e salvo na Flash.",
                }
                self.detail.setText(
                    f"Motor {motor} {messages.get(self._requested_mode, 'calibrado.')}")
            elif state == 3:
                self._terminal_latched = True
                if error == 9:
                    self._set_state("ABORTED", "warning")
                    self.detail.setText("Último ensaio interrompido. Pronto para iniciar novamente.")
                else:
                    self._set_state("FAILED", "error")
                    self.detail.setText(f"Falha: {self.ERROR_NAMES.get(error, error)}")
            elif error:
                self._set_state(self.STATE_NAMES.get(state, f"STATE {state}"), "error")
                self.detail.setText(self.ERROR_NAMES.get(error, str(error)))
            else:
                self._set_state(self.STATE_NAMES.get(state, f"STATE {state}"), "idle")

        formats = {"tu": ".4f", "ku": ".3f", "kp": ".3f",
                   "ki": ".3f", "kd": ".3f"}
        has_result = state == 2 or any(abs(float(tune.get(key, 0.0))) > 1e-9
                                       for key in ("tu", "ku", "kp", "ki"))
        for key, spec in formats.items():
            self.metrics[key].setText(
                format(float(tune.get(key, 0.0)), spec) if has_result else "—")
        self._update_actions()

    def _set_state(self, text, visual_state):
        self.state.setText(text)
        self.state.setProperty("state", visual_state)
        self.state.style().unpolish(self.state)
        self.state.style().polish(self.state)

    def _request_start(self):
        self._requested_mode = int(self.mode.currentData())
        self.start_requested.emit(int(self.motor.currentData()), self._requested_mode)

    def _update_actions(self):
        self.start_button.setEnabled(self._available and not self._running)
        self.abort_button.setEnabled(self._running)
        self.motor.setEnabled(not self._running)
        self.mode.setEnabled(not self._running)
        self.restore_button.setEnabled(self._available and not self._running)
        self.commit_button.setEnabled(self._available and not self._running)
