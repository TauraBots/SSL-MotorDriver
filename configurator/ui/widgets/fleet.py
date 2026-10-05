from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QVBoxLayout

from .indicators import LedIndicator


class FleetRobotCard(QFrame):
    selected = Signal(str)

    def __init__(self, robot_state, active=False, uplink=False, parent=None):
        super().__init__(parent); self.robot_id = robot_state.robot_id; self.setObjectName("fleetCard"); self.setProperty("active", active); self.setMinimumWidth(300); self.setMaximumWidth(420)
        layout = QVBoxLayout(self); layout.setContentsMargins(14, 12, 14, 12); layout.setSpacing(6)
        header = QHBoxLayout(); title = QLabel(f"ROBOT {self.robot_id}"); title.setObjectName("fleetRobotTitle"); header.addWidget(title); header.addStretch(); self.led = LedIndicator(); header.addWidget(self.led); self.status = QLabel(); self.status.setObjectName("cardStatus"); header.addWidget(self.status); layout.addLayout(header)
        self.registration = QLabel("REGISTERED"); self.registration.setObjectName("fleetMeta"); layout.addWidget(self.registration)
        self.uplink = QLabel("UPLINK"); self.uplink.setObjectName("cardStatus"); layout.addWidget(self.uplink)
        self.battery = QLabel(); self.battery.setObjectName("fleetMetric"); self.latency = QLabel(); self.latency.setObjectName("fleetMetric"); layout.addWidget(self.battery); layout.addWidget(self.latency)
        self.uid = QLabel(); self.uid.setObjectName("fleetMeta"); self.firmware = QLabel(); self.firmware.setObjectName("fleetMeta"); layout.addWidget(self.uid); layout.addWidget(self.firmware)
        self.select_button = QPushButton("DESELECT" if active else "SELECT"); self.select_button.setObjectName("fleetSelect"); self.select_button.setFixedSize(104, 28); self.select_button.setEnabled(robot_state.can_control); self.select_button.clicked.connect(lambda: self.selected.emit(self.robot_id)); layout.addWidget(self.select_button, 0, Qt.AlignmentFlag.AlignRight)
        self.update_state(robot_state, active, uplink)

    def update_state(self, robot, active=False, uplink=False):
        active_changed = self.property("active") != active
        self.setProperty("active", active)
        telemetry_status = "TELEMETRY ONLINE"
        if robot.connected and robot.status == "WARNING":
            telemetry_status += " / WARNING"
        self.status.setText(telemetry_status if robot.connected else "NO TELEMETRY")
        self.registration.setText("REGISTERED" if robot.discovered else "PENDING")
        self.uplink.setVisible(uplink)
        self.led.set_state("warning" if robot.connected and robot.status == "WARNING" else
                           "ok" if robot.connected else "off")
        self.battery.setText(f"BATTERY   {robot.battery:.2f} V" if robot.battery is not None else "BATTERY   NO DATA")
        self.latency.setText(f"LATENCY   {robot.latency_ms} ms" if robot.latency_ms is not None else "LATENCY   NO DATA")
        self.uid.setText(f"UID   {robot.uid or 'NO DATA'}"); self.firmware.setText(f"FIRMWARE   {robot.firmware_version or 'NO DATA'}")
        self.select_button.setText("DESELECT" if active else "SELECT"); self.select_button.setEnabled(robot.can_control)
        if active_changed:
            self.style().unpolish(self); self.style().polish(self); self.update()
