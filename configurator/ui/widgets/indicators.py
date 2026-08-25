from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import QWidget


class LedIndicator(QWidget):
    COLORS = {"ok": "#3FB950", "warning": "#D29922", "error": "#F85149", "off": "#6E7681"}

    def __init__(self, state="off", diameter=10, parent=None):
        super().__init__(parent); self._state = state; self.setFixedSize(diameter + 4, diameter + 4)

    def set_state(self, state):
        self._state = state if state in self.COLORS else "off"; self.update()

    def paintEvent(self, event):
        painter = QPainter(self); painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        color = QColor(self.COLORS[self._state]); painter.setPen(Qt.PenStyle.NoPen); painter.setBrush(color)
        painter.drawEllipse(self.rect().adjusted(2, 2, -2, -2))
