from PySide6.QtCore import Signal
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QVBoxLayout

from .indicators import LedIndicator


class FleetRobotCard(QFrame):
    selected = Signal(str)

    def __init__(self, robot_state, active=False, parent=None):
        super().__init__(parent); self.robot_id = robot_state.robot_id; self.setObjectName("fleetCard"); self.setProperty("active", active)
        layout = QVBoxLayout(self); layout.setContentsMargins(18, 16, 18, 16); layout.setSpacing(9)
        header = QHBoxLayout(); title = QLabel(f"ROBOT {self.robot_id}"); title.setObjectName("fleetRobotTitle"); header.addWidget(title); header.addStretch(); self.led = LedIndicator(); header.addWidget(self.led); self.status = QLabel(); self.status.setObjectName("cardStatus"); header.addWidget(self.status); layout.addLayout(header)
        self.battery = QLabel(); self.battery.setObjectName("fleetMetric"); self.latency = QLabel(); self.latency.setObjectName("fleetMetric"); layout.addWidget(self.battery); layout.addWidget(self.latency)
        self.select_button = QPushButton("SELECTED" if active else "SELECT"); self.select_button.setEnabled(not active); self.select_button.clicked.connect(lambda: self.selected.emit(self.robot_id)); layout.addWidget(self.select_button)
        self.update_state(robot_state, active)

    def update_state(self, robot, active=False):
        self.setProperty("active", active); self.status.setText(robot.status); self.led.set_state("ok" if robot.connected else "off")
        self.battery.setText(f"BATTERY   {robot.battery:.2f} V" if robot.battery is not None else "BATTERY   NO DATA")
        self.latency.setText(f"LATENCY   {robot.latency_ms} ms" if robot.latency_ms is not None else "LATENCY   NO DATA")
