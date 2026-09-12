"""Shared application state: one QObject store that all pages subscribe to."""

from __future__ import annotations

from typing import Any

from PySide6.QtCore import QObject, Signal


class AppState(QObject):
    """Connection/device/node store; every mutation emits a signal."""

    connectionChanged = Signal(bool)
    deviceInfoChanged = Signal(object)
    nodesChanged = Signal(object)
    selectedNodeChanged = Signal(int)
    errorRaised = Signal(str)
    logEmitted = Signal(str, str)  # (level, message)
    telemetryChanged = Signal(object)  # {vbus, ibus, temp} summary, values may be None

    def __init__(self, parent: QObject | None = None) -> None:
        super().__init__(parent)
        self.connected = False
        self.device_info: dict[str, Any] = {}
        self.nodes: list[dict[str, Any]] = []
        self.selected_node = 0
        self.error_count = 0
        self.telemetry: dict[str, float | None] = {"vbus": None, "ibus": None, "temp": None}

    def set_connected(self, connected: bool) -> None:
        if self.connected == connected:
            return
        self.connected = connected
        if not connected:
            self.device_info = {}
            self.nodes = []
            self.deviceInfoChanged.emit({})
            self.nodesChanged.emit([])
        self.connectionChanged.emit(connected)

    def set_device_info(self, info: dict[str, Any]) -> None:
        self.device_info = dict(info)
        self.deviceInfoChanged.emit(dict(self.device_info))

    def set_nodes(self, nodes: list[dict[str, Any]]) -> None:
        self.nodes = list(nodes)
        self.nodesChanged.emit(list(self.nodes))

    def update_node(self, status: dict[str, Any]) -> None:
        """Merge one node's live status (mode/fault/latency) into the list."""
        node = status.get("node")
        for entry in self.nodes:
            if entry.get("node") == node:
                entry.update(status)
                break
        else:
            self.nodes.append(dict(status))
        self.nodesChanged.emit(list(self.nodes))

    def select_node(self, node: int) -> None:
        if self.selected_node == node:
            return
        self.selected_node = node
        self.selectedNodeChanged.emit(node)

    def set_telemetry(self, vbus: float | None, ibus: float | None, temp: float | None) -> None:
        self.telemetry = {"vbus": vbus, "ibus": ibus, "temp": temp}
        self.telemetryChanged.emit(dict(self.telemetry))

    def raise_error(self, message: str) -> None:
        self.error_count += 1
        self.errorRaised.emit(message)
        self.log("ERROR", message)

    def log(self, level: str, message: str) -> None:
        self.logEmitted.emit(level, message)

    def online_nodes(self) -> list[int]:
        return sorted(int(entry["node"]) for entry in self.nodes if entry.get("online", True))
