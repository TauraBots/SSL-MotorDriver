from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout

from .indicators import LedIndicator


class HeaderStatusItem(QFrame):
    def __init__(self, label, value="—", parent=None):
        super().__init__(parent); self.setObjectName("headerStatusItem"); layout = QVBoxLayout(self); layout.setContentsMargins(12, 7, 12, 7); layout.setSpacing(2)
        caption = QLabel(label.upper()); caption.setObjectName("headerStatusLabel"); layout.addWidget(caption)
        row = QHBoxLayout(); row.setSpacing(6); self.led = LedIndicator("off", 8); row.addWidget(self.led); self.value = QLabel(value); self.value.setObjectName("headerStatusValue"); row.addWidget(self.value); row.addStretch(); layout.addLayout(row)

    def set_status(self, value, state="off"):
        self.value.setText(str(value)); self.led.set_state(state)
