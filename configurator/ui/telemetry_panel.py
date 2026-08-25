import time

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from .widgets import RealtimePlot, StatusCard


class TelemetryPanel(QWidget):
    log_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent); layout = QVBoxLayout(self); layout.setContentsMargins(28, 24, 28, 24); layout.setSpacing(16)
        header = QHBoxLayout(); title_box = QVBoxLayout(); title = QLabel("REAL-TIME TELEMETRY"); title.setObjectName("pageTitle"); title_box.addWidget(title)
        subtitle = QLabel("Interactive plots · mouse wheel zoom · drag to inspect"); subtitle.setObjectName("muted"); title_box.addWidget(subtitle); header.addLayout(title_box); header.addStretch()
        self.log_button = QPushButton("START CSV RECORDING"); self.log_button.clicked.connect(self.log_requested); header.addWidget(self.log_button); layout.addLayout(header)
        cards = QHBoxLayout(); self.communication_card = StatusCard("Communication", "—", "NO DATA"); self.sequence_card = StatusCard("Sequence", "—", "WAITING"); self.battery_card = StatusCard("Battery", "—", "NO DATA"); self.latency_card = StatusCard("Latency", "—", "NO DATA")
        for card in (self.communication_card, self.sequence_card, self.battery_card, self.latency_card): cards.addWidget(card)
        layout.addLayout(cards)
        self.communication = self.communication_card.value; self.sequence = self.sequence_card.value; self.battery = self.battery_card.value; self.latency = self.latency_card.value
        plots = QGridLayout(); colors = (("Motor 1", "#00A8FF"), ("Motor 2", "#3FB950"), ("Motor 3", "#D29922"), ("Motor 4", "#A371F7"))
        self.rpm_plot = RealtimePlot("Motor RPM", colors); self.battery_plot = RealtimePlot("Battery Voltage", (("Voltage", "#3FB950"),)); self.command_plot = RealtimePlot("Cartesian Command", (("vx", "#00A8FF"), ("vy", "#3FB950"), ("omega", "#D29922")))
        plots.addWidget(self.rpm_plot, 0, 0, 1, 2); plots.addWidget(self.battery_plot, 1, 0); plots.addWidget(self.command_plot, 1, 1); layout.addLayout(plots, 1)
        self.log_status = QLabel("Recording inactive"); self.log_status.setObjectName("muted"); layout.addWidget(self.log_status)

    def append_command(self, vx, vy, omega): self.command_plot.append(time.monotonic(), (vx, vy, omega))

    def update_telemetry(self, data, latency_ms):
        faults = data.get("fault_status", 0); ok = bool(data.get("comm_ok", data.get("watchdog_ok", 0)))
        self.communication_card.update_status("FAULT" if faults else ("OK" if ok else "LOST"), "LINK ACTIVE" if ok else "WARNING", "error" if faults else ("ok" if ok else "warning"))
        self.sequence_card.update_status(data.get("command_sequence", "—"), "RECEIVING", "ok")
        self.latency_card.update_status(f"{latency_ms} ms", "NORMAL" if latency_ms < 100 else "HIGH", "ok" if latency_ms < 100 else "warning")
        if "battery_v" in data: self.battery_card.update_status(f"{data['battery_v']:.2f} V", "LIVE", "ok")
        timestamp = time.monotonic(); rpm = data.get("rpm", ())
        if len(rpm) == 4: self.rpm_plot.append(timestamp, rpm)
        if "battery_v" in data: self.battery_plot.append(timestamp, (data["battery_v"],))
