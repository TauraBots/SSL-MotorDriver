from PySide6.QtCore import Signal
from PySide6.QtWidgets import QComboBox, QDoubleSpinBox, QHBoxLayout, QLabel, QLineEdit, QListWidget, QPushButton, QVBoxLayout, QWidget

from widgets import CardWidget


class ConfigPanel(QWidget):
    discover_requested = Signal()
    set_id_requested = Signal()
    set_motion_requested = Signal()
    board_selected = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent); layout = QVBoxLayout(self); layout.setContentsMargins(28, 24, 28, 24); layout.setSpacing(16)
        title = QLabel("CONFIGURAÇÃO"); title.setObjectName("pageTitle"); layout.addWidget(title)
        subtitle = QLabel("Identidade persistente e limites de movimento por placa."); subtitle.setObjectName("muted"); layout.addWidget(subtitle)
        service_notice = QLabel(
            "Para configuração confiável via AirPort, mantenha apenas o "
            "robô/RX alvo ativo no uplink.")
        service_notice.setObjectName("muted"); service_notice.setWordWrap(True)
        layout.addWidget(service_notice)
        discovery = CardWidget(); row = QHBoxLayout(); self.discover_button = QPushButton("DESCOBRIR PLACAS"); self.discover_button.clicked.connect(self.discover_requested); row.addWidget(self.discover_button)
        self.discovery_status = QLabel("Aguardando descoberta"); row.addWidget(self.discovery_status, 1); discovery.content.addLayout(row)
        self.boards = QListWidget(); self.boards.currentRowChanged.connect(self.board_selected); discovery.content.addWidget(self.boards); layout.addWidget(discovery)
        identity = CardWidget(); row = QHBoxLayout(); self.uid = QLineEdit(); self.uid.setPlaceholderText("UID STM32 (24 hex)")
        self.robot_id = QComboBox(); self.robot_id.addItems(list("ABCDEFGHIJKLMNOPQRSTUVWXYZ")); self.set_id_button = QPushButton("SALVAR ID"); self.set_id_button.clicked.connect(self.set_id_requested)
        row.addWidget(QLabel("UID")); row.addWidget(self.uid, 1); row.addWidget(QLabel("Novo ID")); row.addWidget(self.robot_id); row.addWidget(self.set_id_button); identity.content.addLayout(row); layout.addWidget(identity)
        motion = CardWidget(); row = QHBoxLayout(); self.linear = QDoubleSpinBox(); self.linear.setRange(.1, 20); self.linear.setValue(4); self.linear.setSuffix(" m/s²")
        self.angular = QDoubleSpinBox(); self.angular.setRange(.1, 50); self.angular.setValue(10); self.angular.setSuffix(" rad/s²")
        self.set_motion_button = QPushButton("SALVAR LIMITES"); self.set_motion_button.clicked.connect(self.set_motion_requested)
        row.addWidget(QLabel("Aceleração linear")); row.addWidget(self.linear); row.addWidget(QLabel("Aceleração angular")); row.addWidget(self.angular); row.addStretch(); row.addWidget(self.set_motion_button); motion.content.addLayout(row); layout.addWidget(motion); layout.addStretch()
        self.set_actions_enabled(False)

    def set_actions_enabled(self, enabled):
        self.set_id_button.setEnabled(enabled); self.set_motion_button.setEnabled(enabled)

    def set_busy(self, busy):
        self.discover_button.setEnabled(not busy); self.discover_button.setText("AGUARDE…" if busy else "DESCOBRIR PLACAS")
        self.set_actions_enabled(not busy and bool(self.uid.text().strip()))
