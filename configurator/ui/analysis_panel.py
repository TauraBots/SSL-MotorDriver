import csv
from pathlib import Path

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QFileDialog, QHBoxLayout, QLabel, QLineEdit, QPushButton, QVBoxLayout, QWidget

from widgets import CardWidget


class AnalysisPanel(QWidget):
    generate_requested = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent); layout = QVBoxLayout(self); layout.setContentsMargins(28, 24, 28, 24); layout.setSpacing(16)
        title = QLabel("ANÁLISE DE DADOS"); title.setObjectName("pageTitle"); layout.addWidget(title)
        subtitle = QLabel("Abra uma gravação CSV e exporte gráficos para análise."); subtitle.setObjectName("muted"); layout.addWidget(subtitle)
        card = CardWidget(); row = QHBoxLayout(); self.path = QLineEdit(); self.path.setPlaceholderText("Arquivo telemetry.csv")
        choose = QPushButton("SELECIONAR CSV"); choose.clicked.connect(self.choose_file); generate = QPushButton("GERAR GRÁFICOS"); generate.clicked.connect(lambda: self.generate_requested.emit(self.path.text().strip()))
        row.addWidget(self.path, 1); row.addWidget(choose); row.addWidget(generate); card.content.addLayout(row)
        self.status = QLabel("Nenhum arquivo selecionado"); self.status.setObjectName("muted"); card.content.addWidget(self.status); layout.addWidget(card); layout.addStretch()

    def choose_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "Abrir telemetria", "", "CSV (*.csv);;Todos (*.*)")
        if not path: return
        self.path.setText(path)
        try:
            with open(path, newline="", encoding="utf-8") as stream:
                reader = csv.reader(stream); header = next(reader, []); preview = [row for _, row in zip(range(5), reader)]
            self.status.setText(f"{len(header)} colunas · preview de {len(preview)} amostras · {Path(path).name}")
        except Exception as exc: self.status.setText(f"Não foi possível ler o preview: {exc}")
