from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List

from PySide6 import QtWidgets


class ComponentWidget(QtWidgets.QWidget):
    """Base widget for an Axon Control Center component."""

    component_name: str = "Component"
    component_description: str = ""

    def on_activate(self) -> None:
        """Hook called when the component is shown."""

    def on_deactivate(self) -> None:
        """Hook called when the component is hidden."""


@dataclass(frozen=True)
class ComponentSpec:
    name: str
    description: str
    factory: Callable[[], ComponentWidget]


class ComponentRegistry:
    def __init__(self) -> None:
        self._components: List[ComponentSpec] = []

    def register(self, component: ComponentSpec) -> None:
        self._components.append(component)

    def specs(self) -> List[ComponentSpec]:
        return list(self._components)
