from PySide6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

from widgets import CardWidget, metric_card


class DiagnosticsPanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent); layout = QVBoxLayout(self); layout.setContentsMargins(28, 24, 28, 24); layout.setSpacing(16)
        title = QLabel("DIAGNÓSTICO"); title.setObjectName("pageTitle"); layout.addWidget(title)
        subtitle = QLabel("Saúde da comunicação, watchdog e falhas reportadas."); subtitle.setObjectName("muted"); layout.addWidget(subtitle)
        for group in ((("Fault status", "fault"), ("Watchdog", "watchdog"), ("Último pacote", "packet")),
                      (("RX packets", "rx"), ("TX commands", "tx"), ("CRC errors", "crc"))):
            row = QHBoxLayout()
            for name, attr in group:
                card, value = metric_card(name); setattr(self, attr, value); row.addWidget(card)
            layout.addLayout(row)
        card = CardWidget(); self.serial_status = QLabel("Conecte um robô para iniciar o diagnóstico."); self.serial_status.setObjectName("muted"); card.content.addWidget(self.serial_status); layout.addWidget(card); layout.addStretch()

    def update_telemetry(self, data, latency_ms):
        faults = data.get("fault_status", 0); names = [f"M{i + 1}" for i in range(4) if faults & (1 << i)]
        if faults & 0x10: names.append("BATERIA")
        ok = data.get("comm_ok", data.get("watchdog_ok", 0))
        self.fault.setText(", ".join(names) if names else "NENHUMA"); self.watchdog.setText("OK" if ok else "LOST"); self.packet.setText(f"{latency_ms} ms atrás")
        self.rx.setText(str(data.get("received_packets", "—"))); self.crc.setText(str(data.get("crc_errors", "—")))
        self.serial_status.setText(f"Sequência recebida: {data.get('command_sequence', '—')} · polling ativo")
