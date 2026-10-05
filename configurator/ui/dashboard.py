from PySide6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from .widgets import BatteryWidget, CommunicationWidget, MotorCard, StatusCard


class DashboardPanel(QWidget):
    def __init__(self, icons=None, parent=None):
        super().__init__(parent); icons = icons or {}; layout = QVBoxLayout(self); layout.setContentsMargins(28, 24, 28, 24); layout.setSpacing(18)
        heading = QHBoxLayout(); title_box = QVBoxLayout(); self.title = QLabel("NO ROBOT SELECTED"); self.title.setObjectName("pageTitle"); title_box.addWidget(self.title)
        subtitle = QLabel("Live operational status and drivetrain telemetry"); subtitle.setObjectName("muted"); title_box.addWidget(subtitle); heading.addLayout(title_box); heading.addStretch()
        self.global_status = QLabel("OFFLINE"); self.global_status.setObjectName("globalStatus"); heading.addWidget(self.global_status); layout.addLayout(heading)

        summary = QGridLayout(); summary.setHorizontalSpacing(14); summary.setVerticalSpacing(14)
        self.battery_card = BatteryWidget(icons.get("battery")); self.communication_card = CommunicationWidget(icons.get("connection"))
        self.robot_card = StatusCard("Robot ID", "—", "NOT SELECTED", icons.get("robot")); self.watchdog_card = StatusCard("Watchdog", "—", "NO DATA", icons.get("warning"))
        for column, card in enumerate((self.battery_card, self.communication_card, self.robot_card, self.watchdog_card)): summary.addWidget(card, 0, column)
        layout.addLayout(summary)

        section = QLabel("DRIVETRAIN"); section.setObjectName("section"); layout.addWidget(section)
        motors = QGridLayout(); motors.setHorizontalSpacing(14); motors.setVerticalSpacing(14); self.motor_cards = []
        for index in range(4):
            card = MotorCard(index + 1, icons.get("motor")); motors.addWidget(card, 0, index); self.motor_cards.append(card)
        layout.addLayout(motors); layout.addStretch()

        # Compatibility labels used by MainWindow while preserving encapsulated cards.
        self.connection = self.communication_card.value; self.battery = self.battery_card.value
        self.packet = self.communication_card.details; self.watchdog = self.watchdog_card.value

    def set_connected(self, connected, robot_id="—"):
        self.global_status.setText("CONNECTED" if connected else "OFFLINE"); self.global_status.setProperty("online", connected)
        self.global_status.style().unpolish(self.global_status); self.global_status.style().polish(self.global_status)
        self.communication_card.set_connection(connected); self.robot_card.update_status(robot_id if connected else "—", "ACTIVE" if connected else "NOT SELECTED", "ok" if connected else "off")

    def set_robot(self, robot):
        if robot is None:
            self.title.setText("NO ROBOT SELECTED"); self.set_connected(False)
            self.battery_card.update_status("—", "NO DATA", "off"); self.battery_card.level.setValue(0)
            self.watchdog_card.update_status("—", "NO DATA", "off")
            for card in self.motor_cards: card.set_motor_data(0.0, 0, True); card.update_status("0 RPM", "NO DATA", "off")
            return
        self.title.setText(f"ROBOT {robot.robot_id} OVERVIEW")
        telemetry_online = robot.connected
        control_ready = robot.can_control
        self.global_status.setText("TELEMETRY OK" if telemetry_online else
                                   "COMMAND ONLY" if control_ready else "OFFLINE")
        self.global_status.setProperty("online", telemetry_online)
        self.global_status.style().unpolish(self.global_status); self.global_status.style().polish(self.global_status)
        self.communication_card.set_connection(telemetry_online)
        if control_ready and not telemetry_online:
            self.communication_card.update_status("NO TELEMETRY", "COMMAND ONLY", "warning")
        self.robot_card.update_status(robot.robot_id,
                                      "CONTROL READY" if control_ready else "NOT SELECTED",
                                      "ok" if telemetry_online else "warning" if control_ready else "off")
        if robot.battery is not None: self.battery_card.set_voltage(robot.battery)
        else: self.battery_card.update_status("—", "NO DATA", "off"); self.battery_card.level.setValue(0)
        self.watchdog_card.update_status("OK" if robot.watchdog_ok else "NO DATA", robot.status, "ok" if robot.watchdog_ok else "off")
        for card, rpm, command in zip(self.motor_cards, robot.rpm, robot.motor_command): card.set_motor_data(rpm, command, robot.status != "ERROR")

    def set_command(self, vx, vy, omega):
        self.robot_card.status.setText(f"VX {vx:+.2f}  VY {vy:+.2f}  OMEGA {omega:+.2f}")

    def set_telemetry(self, data, latency_ms):
        if "battery_v" in data: self.battery_card.set_voltage(data["battery_v"])
        ok = bool(data.get("comm_ok", data.get("watchdog_ok", 0))); faults = data.get("fault_status", 0)
        self.communication_card.set_connection(True, data.get("command_sequence"), latency_ms, data.get("received_packets"))
        self.watchdog_card.update_status("OK" if ok else "LOST", "HEALTHY" if ok else "WARNING", "ok" if ok else "warning")
        for index, (card, rpm, command) in enumerate(zip(self.motor_cards, data.get("rpm", ()), data.get("cmd", ()))):
            card.set_motor_data(rpm, command, not bool(faults & (1 << index)))
