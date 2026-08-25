from PySide6.QtWidgets import QFrame, QLabel, QVBoxLayout


class CardWidget(QFrame):
    def __init__(self, object_name="card", parent=None):
        super().__init__(parent); self.setObjectName(object_name)
        self.content = QVBoxLayout(self); self.content.setContentsMargins(20, 18, 20, 18); self.content.setSpacing(10)


class MotorCard(CardWidget):
    def __init__(self, index, parent=None):
        super().__init__("motorCard", parent); self.content.addWidget(QLabel(f"MOTOR {index}"))
        self.rpm = QLabel("0 RPM"); self.rpm.setObjectName("metricSmall")
        self.command = QLabel("PWM 0"); self.command.setObjectName("muted")
        self.content.addWidget(self.rpm); self.content.addWidget(self.command)


def metric_card(title, value="—"):
    card = CardWidget(); card.content.setContentsMargins(18, 16, 18, 16)
    heading = QLabel(title.upper()); heading.setObjectName("eyebrow"); card.content.addWidget(heading)
    metric = QLabel(value); metric.setObjectName("metric"); card.content.addWidget(metric)
    return card, metric
