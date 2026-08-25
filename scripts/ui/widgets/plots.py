from PySide6.QtWidgets import QLabel, QVBoxLayout, QWidget

try:
    import pyqtgraph as pg
except ImportError:
    pg = None


class RealtimePlot(QWidget):
    def __init__(self, title, series, parent=None):
        super().__init__(parent); self.setObjectName("plotCard"); layout = QVBoxLayout(self); layout.setContentsMargins(14, 12, 14, 12)
        heading = QLabel(title.upper()); heading.setObjectName("cardTitle"); layout.addWidget(heading); self.x = []; self.values = [[] for _ in series]
        if pg:
            self.graph = pg.PlotWidget(); self.graph.setBackground("#161B22"); self.graph.showGrid(x=True, y=True, alpha=.2); self.graph.addLegend(offset=(8, 8)); self.graph.setMouseEnabled(x=True, y=True)
            self.curves = [self.graph.plot(name=name, pen=pg.mkPen(color, width=2)) for name, color in series]; layout.addWidget(self.graph)
        else:
            fallback = QLabel("PyQtGraph não está instalado"); fallback.setObjectName("muted"); layout.addWidget(fallback); self.curves = []

    def append(self, timestamp, values):
        if not self.curves: return
        if not self.x: self._origin = timestamp
        self.x.append(timestamp - self._origin); self.x = self.x[-300:]
        for index, value in enumerate(values):
            self.values[index].append(value); self.values[index] = self.values[index][-300:]; self.curves[index].setData(self.x, self.values[index])
