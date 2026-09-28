from PySide6.QtCore import Signal
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from widgets import CardWidget, metric_card


class DiagnosticsPanel(QWidget):
    metrics_log_requested = Signal()

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
        card = CardWidget(); self.serial_status = QLabel("Conecte um robô para iniciar o diagnóstico."); self.serial_status.setObjectName("muted"); card.content.addWidget(self.serial_status); layout.addWidget(card)
        link_card = CardWidget(); self.link_metrics = QLabel("Perfil: NORMAL · aguardando medições")
        self.link_metrics.setObjectName("muted"); self.link_metrics.setWordWrap(True)
        link_card.content.addWidget(self.link_metrics)
        self.metrics_log_button = QPushButton("GRAVAR MÉTRICAS CSV")
        self.metrics_log_button.clicked.connect(self.metrics_log_requested.emit)
        link_card.content.addWidget(self.metrics_log_button); layout.addWidget(link_card); layout.addStretch()

    def update_telemetry(self, data, latency_ms):
        faults = data.get("fault_status", 0); names = [f"M{i + 1}" for i in range(4) if faults & (1 << i)]
        if faults & 0x10: names.append("BATERIA")
        ok = data.get("comm_ok", data.get("watchdog_ok", 0))
        self.fault.setText(", ".join(names) if names else "NENHUMA"); self.watchdog.setText("OK" if ok else "LOST"); self.packet.setText(f"{latency_ms} ms atrás")
        self.rx.setText(str(data.get("received_packets", "—"))); self.crc.setText(str(data.get("crc_errors", "—")))
        self.serial_status.setText(f"Sequência recebida: {data.get('command_sequence', '—')} · polling ativo")

    def update_link_stats(self, profile, stats, serial_baud=None, traffic=None,
                          capacity_note="AirPort OTA capacity unknown — validate experimentally."):
        value = lambda number: "—" if number is None else f"{number:.1f}"
        baud = "—" if serial_baud is None else str(serial_baud)
        traffic_text = ""
        if traffic is not None:
            traffic_text = (
                f"\nEstimated protocol traffic — PC→robot "
                f"{traffic.estimated_protocol_tx_bytes_per_s:.0f} B/s · robot→PC "
                f"{traffic.estimated_protocol_rx_bytes_per_s:.0f} B/s")
        self.link_metrics.setText(
            f"Transport: AirPort · Serial baud: {baud}\n"
            f"Profile: {profile.display_name}\n"
            f"Command — Target: {profile.command_hz:g} Hz · Measured: {stats.command_tx_hz:.1f} Hz\n"
            f"Telemetry — Target: {profile.telemetry_fast_hz:g} Hz · Requests: {stats.telemetry_request_tx_hz:.1f} Hz · Responses: {stats.telemetry_response_rx_hz:.1f} Hz\n"
            f"Response loss: {stats.telemetry_response_loss_percent:.1f}%\n"
            f"Latency — last {value(stats.latency_last_ms)} ms · mean {value(stats.latency_mean_ms)} ms · max {value(stats.latency_max_ms)} ms · p95 {value(stats.latency_p95_ms)} ms\n"
            f"Missed deadlines — CMD {stats.command_deadlines_missed} · TEL {stats.telemetry_deadlines_missed}\n"
            f"Measured throughput — TX {stats.tx_bytes_per_s / 1000:.2f} kB/s · RX {stats.rx_bytes_per_s / 1000:.2f} kB/s"
            f"{traffic_text}\n{capacity_note}"
        )
