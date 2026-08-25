from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QDial, QGridLayout, QHBoxLayout, QLabel, QPushButton, QSpinBox, QVBoxLayout, QWidget

from widgets import CardWidget
from .widgets import JoystickWidget, LedIndicator


class ControlPanel(QWidget):
    kick_requested = Signal(int); virtual_key = Signal(str, bool)
    joystick_changed = Signal(float, float, float); stop_requested = Signal()

    def __init__(self, args, parent=None):
        super().__init__(parent); self._joystick_x = self._joystick_y = self._rotation = 0.0
        layout = QVBoxLayout(self); layout.setContentsMargins(28, 24, 28, 24); layout.setSpacing(18)
        title_row = QHBoxLayout(); title = QLabel("MANUAL CONTROL"); title.setObjectName("pageTitle"); title_row.addWidget(title); title_row.addStretch()
        self.control_target = QLabel("NO CONTROL TARGET"); self.control_target.setObjectName("controlTarget"); title_row.addWidget(self.control_target); layout.addLayout(title_row)
        subtitle = QLabel("Joystick and keyboard control · WASD movement · Q/E rotation"); subtitle.setObjectName("muted"); layout.addWidget(subtitle)
        body = QHBoxLayout(); body.setSpacing(18); joystick_card = CardWidget(); self.joystick_card = joystick_card; label = QLabel("TRANSLATION JOYSTICK"); label.setObjectName("section"); joystick_card.content.addWidget(label)
        joystick_row = QHBoxLayout(); self.joystick = JoystickWidget(); self.joystick.changed.connect(self._on_joystick); joystick_row.addWidget(self.joystick, 1, Qt.AlignmentFlag.AlignCenter)
        rotation_box = QVBoxLayout(); rotation_title = QLabel("ROTATION"); rotation_title.setObjectName("cardTitle"); rotation_box.addWidget(rotation_title)
        self.rotation = QDial(); self.rotation.setRange(-100, 100); self.rotation.setValue(0); self.rotation.setNotchesVisible(True); self.rotation.valueChanged.connect(self._on_rotation); rotation_box.addWidget(self.rotation)
        self.rotation_value = QLabel("0.00 rad/s"); self.rotation_value.setAlignment(Qt.AlignmentFlag.AlignCenter); rotation_box.addWidget(self.rotation_value); rotation_box.addStretch(); joystick_row.addLayout(rotation_box); joystick_card.content.addLayout(joystick_row)
        keys = QGridLayout(); self.keys = {}
        for key, text, row, column in (("q", "Q", 0, 0), ("w", "W", 0, 1), ("e", "E", 0, 2), ("a", "A", 1, 0), ("s", "S", 1, 1), ("d", "D", 1, 2)):
            item = QLabel(text); item.setObjectName("key"); item.setAlignment(Qt.AlignmentFlag.AlignCenter); item.setProperty("pressed", False); keys.addWidget(item, row, column); self.keys[key] = item
        joystick_card.content.addLayout(keys)

        command_card = CardWidget(); self.command_card = command_card; header = QHBoxLayout(); heading = QLabel("ACTIVE COMMAND"); heading.setObjectName("section"); header.addWidget(heading); header.addStretch(); header.addWidget(LedIndicator("off")); self.status = QLabel("DISCONNECTED"); self.status.setObjectName("cardStatus"); header.addWidget(self.status); command_card.content.addLayout(header); self.command = []
        for name, unit in (("VX", "m/s"), ("VY", "m/s"), ("OMEGA", "rad/s")):
            row = QHBoxLayout(); name_label = QLabel(name); name_label.setObjectName("cardTitle"); row.addWidget(name_label); row.addStretch(); value = QLabel(f"0.00 {unit}"); value.setObjectName("commandValue"); row.addWidget(value); command_card.content.addLayout(row); self.command.append((value, unit))
        command_card.content.addStretch(); kick_row = QHBoxLayout(); self.kick = QSpinBox(); self.kick.setRange(0, 100); self.kick.setValue(80); self.kick.setSuffix(" %"); kick_row.addWidget(QLabel("KICK POWER")); kick_row.addWidget(self.kick)
        kick_button = QPushButton("KICK"); kick_button.clicked.connect(lambda: self.kick_requested.emit(self.kick.value())); kick_row.addWidget(kick_button); command_card.content.addLayout(kick_row)
        stop = QPushButton("STOP ROBOT"); stop.setObjectName("stopButton"); stop.clicked.connect(self.stop_requested); command_card.content.addWidget(stop)
        body.addWidget(joystick_card, 3); body.addWidget(command_card, 2); layout.addLayout(body, 1)
        self.set_active_robot(None)

    def _emit_joystick(self): self.joystick_changed.emit(self._joystick_x * 2.0, self._joystick_y * 2.0, self._rotation * 5.0)
    def _on_joystick(self, x, y): self._joystick_x, self._joystick_y = x, y; self._emit_joystick()
    def _on_rotation(self, value): self._rotation = value / 100.0; self.rotation_value.setText(f"{self._rotation * 5.0:+.2f} rad/s"); self._emit_joystick()

    def reset_joystick(self): self.rotation.blockSignals(True); self.rotation.setValue(0); self.rotation.blockSignals(False); self._rotation = 0.0; self.joystick.reset()
    def set_active_robot(self, robot_id):
        enabled = bool(robot_id); self.control_target.setText(f"CONTROLLING: ROBOT {robot_id}" if enabled else "NO CONTROL TARGET")
        self.joystick_card.setEnabled(enabled); self.command_card.setEnabled(enabled)
    def set_key(self, key, pressed):
        if key not in self.keys: return
        label = self.keys[key]; label.setProperty("pressed", pressed); label.style().unpolish(label); label.style().polish(label)
    def set_command(self, vx, vy, omega):
        for (label, unit), value in zip(self.command, (vx, vy, omega)): label.setText(f"{value:+.2f} {unit}")
