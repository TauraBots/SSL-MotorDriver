from PySide6.QtWidgets import QLabel


class StatusIndicator(QLabel):
    def set_state(self, text, state="neutral"):
        colors = {"good": "#43d39e", "warning": "#ffb454", "error": "#ff6577", "neutral": "#8296b0"}
        self.setText(text); self.setStyleSheet(f"color:{colors[state]};font-weight:700")
