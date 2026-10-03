from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QGridLayout, QHBoxLayout, QLabel, QPushButton, QScrollArea, QVBoxLayout, QWidget

from .widgets import FleetRobotCard


class FleetPanel(QWidget):
    robot_selected = Signal(str)
    discovery_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent); layout = QVBoxLayout(self); layout.setContentsMargins(28, 24, 28, 24); layout.setSpacing(16)
        title = QLabel("FLEET"); title.setObjectName("pageTitle"); layout.addWidget(title)
        subtitle = QLabel("Robots detected through the AirPort radio link"); subtitle.setObjectName("muted"); layout.addWidget(subtitle)
        actions = QHBoxLayout(); self.summary = QLabel("NO ROBOTS AVAILABLE"); self.summary.setObjectName("section"); actions.addWidget(self.summary); actions.addStretch()
        self.discover_button = QPushButton("DISCOVER ROBOTS"); self.discover_button.setObjectName("discoverButton"); self.discover_button.setFixedHeight(30); self.discover_button.setProperty("accent", True); self.discover_button.clicked.connect(self.discovery_requested); actions.addWidget(self.discover_button); layout.addLayout(actions)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); content = QWidget(); self.grid = QGridLayout(content); self.grid.setAlignment(self.grid.alignment()); scroll.setWidget(content); layout.addWidget(scroll, 1); self.cards = []; self._cards_by_id = {}

    def set_fleet(self, robots, active_robot_id, uplink_robot_id=None):
        robots = tuple(robots)
        robot_ids = {robot.robot_id for robot in robots}
        topology_changed = robot_ids != set(self._cards_by_id)

        for robot_id in set(self._cards_by_id) - robot_ids:
            card = self._cards_by_id.pop(robot_id)
            self.grid.removeWidget(card); card.deleteLater()

        for robot in robots:
            card = self._cards_by_id.get(robot.robot_id)
            if card is None:
                card = FleetRobotCard(robot, robot.robot_id == active_robot_id,
                                      robot.robot_id == uplink_robot_id)
                card.selected.connect(self.robot_selected)
                self._cards_by_id[robot.robot_id] = card
            else:
                card.update_state(robot, robot.robot_id == active_robot_id,
                                  robot.robot_id == uplink_robot_id)

        # Reposition cards only when robots enter or leave the fleet. Normal
        # telemetry updates must not replace a button during a mouse click.
        if topology_changed:
            for card in self._cards_by_id.values(): self.grid.removeWidget(card)
            for index, robot in enumerate(robots):
                self.grid.addWidget(self._cards_by_id[robot.robot_id], index // 3, index % 3,
                                    Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)

        self.cards = [self._cards_by_id[robot.robot_id] for robot in robots]
        telemetry = sum(robot.connected for robot in robots)
        self.summary.setText(f"{len(robots)} REGISTERED / {telemetry} TELEMETRY")
        self.grid.setRowStretch((len(robots) + 2) // 3, 1)

    def set_discovering(self, discovering):
        self.discover_button.setEnabled(not discovering); self.discover_button.setText("DISCOVERING..." if discovering else "DISCOVER ROBOTS")
