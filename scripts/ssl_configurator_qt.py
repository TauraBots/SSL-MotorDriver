"""Qt 6 entry point for the TauraBots SSL Configurator."""

from PySide6.QtWidgets import QApplication

from ui.main_window import MainWindow


class QtConfiguratorApp(MainWindow):
    """Compatibility adapter used by the existing CLI ``app`` action."""

    def __init__(self, args, backend=None):
        self.app = QApplication.instance() or QApplication([])
        super().__init__(args)

    def run(self):
        self.show()
        return self.app.exec()


__all__ = ["QtConfiguratorApp", "MainWindow"]
