from __future__ import annotations

import logging
from typing import List

from PySide6 import QtCore, QtWidgets

from .components import ComponentRegistry, ComponentSpec, ComponentWidget
from .simulator import SimulatorComponent
from .style import build_stylesheet
from .tester import TesterComponent


class AxonControlCenter(QtWidgets.QMainWindow):
    def __init__(self, components: List[ComponentSpec]) -> None:
        super().__init__()
        self.setWindowTitle("Axon Control Center")
        self.resize(1100, 720)

        self._components = components
        self._widgets: List[ComponentWidget] = []

        container = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)

        self.nav = QtWidgets.QListWidget()
        self.nav.setFixedWidth(220)
        self.stack = QtWidgets.QStackedWidget()

        layout.addWidget(self.nav)
        layout.addWidget(self.stack, 1)
        self.setCentralWidget(container)

        for spec in self._components:
            widget = spec.factory()
            self._widgets.append(widget)
            self.nav.addItem(spec.name)
            self.stack.addWidget(widget)

        self.nav.currentRowChanged.connect(self._swap_component)
        self.nav.setCurrentRow(0)

    def _swap_component(self, index: int) -> None:
        for idx, widget in enumerate(self._widgets):
            if idx == index:
                widget.on_activate()
            else:
                widget.on_deactivate()


class DefaultRegistry(ComponentRegistry):
    def __init__(self) -> None:
        super().__init__()
        self.register(
            ComponentSpec(
                name="Chassis Simulator",
                description="Local TCP simulator",
                factory=SimulatorComponent,
            )
        )
        self.register(
            ComponentSpec(
                name="Chassis Tester",
                description="Command + feedback console",
                factory=TesterComponent,
            )
        )


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    app = QtWidgets.QApplication([])
    app.setStyleSheet(build_stylesheet())
    registry = DefaultRegistry()
    window = AxonControlCenter(registry.specs())
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
