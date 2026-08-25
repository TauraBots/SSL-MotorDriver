import math

from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget


class JoystickWidget(QWidget):
    changed = Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent); self.setMinimumSize(230, 230); self.setMaximumSize(330, 330)
        self._position = QPointF(0.0, 0.0); self._dragging = False

    def _set_from_point(self, point):
        center = QPointF(self.width() / 2, self.height() / 2); radius = max(1.0, min(self.width(), self.height()) / 2 - 28)
        delta = point - center; length = math.hypot(delta.x(), delta.y())
        if length > radius: delta *= radius / length
        self._position = QPointF(delta.x() / radius, delta.y() / radius); self.changed.emit(self._position.x(), -self._position.y()); self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton: self._dragging = True; self._set_from_point(event.position())

    def mouseMoveEvent(self, event):
        if self._dragging: self._set_from_point(event.position())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton: self._dragging = False; self.reset()

    def reset(self):
        self._position = QPointF(); self.changed.emit(0.0, 0.0); self.update()

    def paintEvent(self, event):
        painter = QPainter(self); painter.setRenderHint(QPainter.RenderHint.Antialiasing); center = QPointF(self.width() / 2, self.height() / 2); radius = min(self.width(), self.height()) / 2 - 28
        painter.setPen(QPen(QColor("#30363D"), 2)); painter.setBrush(QColor("#161B22")); painter.drawEllipse(center, radius, radius)
        painter.setPen(QPen(QColor("#30363D"), 1, Qt.PenStyle.DashLine)); painter.drawLine(QPointF(center.x() - radius, center.y()), QPointF(center.x() + radius, center.y())); painter.drawLine(QPointF(center.x(), center.y() - radius), QPointF(center.x(), center.y() + radius))
        knob = center + QPointF(self._position.x() * radius, self._position.y() * radius); painter.setPen(QPen(QColor("#58C8FF"), 3)); painter.setBrush(QColor("#00A8FF")); painter.drawEllipse(knob, 25, 25)
