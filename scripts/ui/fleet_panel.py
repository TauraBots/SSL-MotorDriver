from PySide6.QtCore import Signal
from PySide6.QtWidgets import QGridLayout, QLabel, QScrollArea, QVBoxLayout, QWidget

from .widgets import FleetRobotCard


class FleetPanel(QWidget):
    robot_selected = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent); layout = QVBoxLayout(self); layout.setContentsMargins(28, 24, 28, 24); layout.setSpacing(16)
        title = QLabel("FLEET"); title.setObjectName("pageTitle"); layout.addWidget(title)
        subtitle = QLabel("Robots detected through the AirPort radio link"); subtitle.setObjectName("muted"); layout.addWidget(subtitle)
        self.summary = QLabel("NO ROBOTS AVAILABLE"); self.summary.setObjectName("section"); layout.addWidget(self.summary)
        scroll = QScrollArea(); scroll.setWidgetResizable(True); content = QWidget(); self.grid = QGridLayout(content); self.grid.setAlignment(self.grid.alignment()); scroll.setWidget(content); layout.addWidget(scroll, 1); self.cards = []

    def set_fleet(self, robots, active_robot_id):
        while self.grid.count():
            item = self.grid.takeAt(0)
            if item.widget(): item.widget().deleteLater()
        self.cards = []; online = sum(robot.connected for robot in robots); self.summary.setText(f"{online} ONLINE / {len(robots)} REGISTERED")
        for index, robot in enumerate(robots):
            card = FleetRobotCard(robot, robot.robot_id == active_robot_id); card.selected.connect(self.robot_selected); self.grid.addWidget(card, index // 3, index % 3); self.cards.append(card)
        self.grid.setRowStretch((len(robots) + 2) // 3, 1)
