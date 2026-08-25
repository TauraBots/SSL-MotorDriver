from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QProgressBar, QVBoxLayout

from .indicators import LedIndicator


class StatusCard(QFrame):
    def __init__(self, title, value="—", status="NO DATA", icon=None, parent=None):
        super().__init__(parent); self.setObjectName("statusCard"); self.setMinimumWidth(170)
        layout = QVBoxLayout(self); layout.setContentsMargins(18, 16, 18, 16); layout.setSpacing(8)
        top = QHBoxLayout(); self.icon = QLabel(); self.icon.setObjectName("cardIcon")
        if icon: self.icon.setPixmap(icon.pixmap(18, 18))
        self.title = QLabel(title.upper()); self.title.setObjectName("cardTitle"); top.addWidget(self.icon); top.addWidget(self.title); top.addStretch(); layout.addLayout(top)
        self.value = QLabel(value); self.value.setObjectName("cardValue"); layout.addWidget(self.value)
        bottom = QHBoxLayout(); self.led = LedIndicator(); self.status = QLabel(status); self.status.setObjectName("cardStatus"); bottom.addWidget(self.led); bottom.addWidget(self.status); bottom.addStretch(); layout.addLayout(bottom)

    def update_status(self, value=None, status=None, state=None):
        if value is not None: self.value.setText(str(value))
        if status is not None: self.status.setText(str(status).upper())
        if state is not None: self.led.set_state(state)


class MotorCard(StatusCard):
    def __init__(self, index, icon=None, parent=None):
        super().__init__(f"Motor {index}", "0 RPM", "IDLE", icon, parent); self.setObjectName("motorCard")
        self.index = index; self.command = QLabel("COMMAND   0%"); self.command.setObjectName("motorCommand")
        self.layout().insertWidget(2, self.command)

    def set_motor_data(self, rpm, command, healthy=True):
        percent = min(100, round(abs(command) * 100 / 1000))
        state = "ok" if healthy else "error"; status = "OK" if healthy else "FAULT"
        self.update_status(f"{rpm:+.0f} RPM", status, state); self.command.setText(f"COMMAND   {percent}%")


class BatteryWidget(StatusCard):
    def __init__(self, icon=None, parent=None):
        super().__init__("Battery", "—", "NO DATA", icon, parent)
        self.level = QProgressBar(); self.level.setRange(0, 100); self.level.setTextVisible(False); self.layout().insertWidget(2, self.level)

    def set_voltage(self, voltage):
        level = max(0, min(100, round((voltage - 9.6) / (12.6 - 9.6) * 100)))
        state = "ok" if voltage >= 11.0 else "warning" if voltage >= 10.2 else "error"
        self.level.setValue(level); self.update_status(f"{voltage:.2f} V", f"{level}% · {'NORMAL' if state == 'ok' else 'LOW'}", state)


class CommunicationWidget(StatusCard):
    def __init__(self, icon=None, parent=None):
        super().__init__("Communication", "OFFLINE", "DISCONNECTED", icon, parent)
        self.details = QLabel("SEQ —   ·   0 ms   ·   PKT —"); self.details.setObjectName("cardDetails"); self.layout().insertWidget(2, self.details)

    def set_connection(self, connected, sequence=None, latency_ms=None, packets=None):
        state = "ok" if connected else "error"
        self.update_status("CONNECTED" if connected else "OFFLINE", "LINK OK" if connected else "DISCONNECTED", state)
        self.details.setText(f"SEQ {sequence if sequence is not None else '—'}   ·   {latency_ms if latency_ms is not None else '—'} ms   ·   PKT {packets if packets is not None else '—'}")
