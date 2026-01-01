"""ROS 2 parameter utilities."""
from __future__ import annotations

from typing import Any, Dict

from rclpy.node import Node


def declare_parameters(node: Node, params: Dict[str, Any]) -> None:
    """Declare parameters on a node with defaults."""
    for name, value in params.items():
        node.declare_parameter(name, value)


def get_params(node: Node, keys: Dict[str, Any]) -> Dict[str, Any]:
    """Get parameter values from a node."""
    values = {}
    for name in keys:
        values[name] = node.get_parameter(name).value
    return values
