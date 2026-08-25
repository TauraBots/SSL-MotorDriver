from PySide6.QtWidgets import QLabel

from .cards import CardWidget

try:
    import pyqtgraph as pg
except ImportError:
    pg = None


class TelemetryPlot(CardWidget):
    def __init__(self, title, series, parent=None):
        super().__init__(parent=parent); self.content.addWidget(QLabel(title)); self.x = []; self.values = [[] for _ in series]
        if pg:
            self.plot = pg.PlotWidget(); self.plot.setBackground("#0c1523"); self.plot.showGrid(x=True, y=True, alpha=.18)
            self.curves = [self.plot.plot(pen=pg.mkPen(color, width=2), name=name) for name, color in series]
            self.content.addWidget(self.plot)
        else:
            self.plot = QLabel("Instale pyqtgraph para gráficos interativos"); self.plot.setObjectName("muted")
            self.content.addWidget(self.plot); self.curves = []

    def append(self, timestamp, values):
        if not self.curves: return
        self.x.append(timestamp); self.x = self.x[-180:]
        for index, value in enumerate(values):
            self.values[index].append(value); self.values[index] = self.values[index][-180:]
            self.curves[index].setData(self.x, self.values[index])
