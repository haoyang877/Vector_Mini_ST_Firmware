"""Schema-driven parameter table model for the online params tab."""

from __future__ import annotations

import math
from typing import Any

from PySide6.QtCore import QAbstractTableModel, QModelIndex, Qt, Signal

from mdrive_core.schema.params import PARAMS, ParamDef

from mdrive_gui import theme

HEADERS = ("ID", "参数名", "当前值", "单位", "新值", "范围", "权限", "状态")
COL_ID, COL_NAME, COL_CURRENT, COL_UNIT, COL_NEW, COL_RANGE, COL_ACCESS, COL_STATUS = range(len(HEADERS))

ACCESS_TEXT = {"rw": "R/W", "ro": "R", "cmd": "cmd"}
ENCODING_TEXT = {"f32": "float32", "ma16": "mA16（×1000）", "mrad32": "mrad32（×1000）", "crad32": "crad32（×100）"}


def format_value(value: Any) -> str:
    """Current-value cell text; None marks a no-response read."""
    if value is None:
        return "✗"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def format_range(definition: ParamDef) -> str:
    if definition.access == "cmd":
        return "命令"
    if definition.minimum is None and definition.maximum is None:
        return "—"
    low = "-∞" if definition.minimum is None else f"{definition.minimum:g}"
    high = "+∞" if definition.maximum is None else f"{definition.maximum:g}"
    return f"{low} ~ {high}"


class ParamTableModel(QAbstractTableModel):
    """Table over PARAMS with edit validation, dirty highlight, and filters."""

    changedCountChanged = Signal(int)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._ids: list[int] = sorted(PARAMS)
        self._values: dict[int, float | None] = {}
        self._pending: dict[int, float] = {}
        self._invalid: set[int] = set()
        self._status: dict[int, str] = {}
        self._filter_group: str | None = None
        self._filter_text = ""

    # ------------------------------------------------------------- filtering
    def set_filter(self, group: str | None, text: str) -> None:
        self.beginResetModel()
        self._filter_group = group
        self._filter_text = text.strip().lower()
        self._ids = [
            identifier
            for identifier, definition in sorted(PARAMS.items())
            if (group is None or definition.group == group)
            and (
                not self._filter_text
                or self._filter_text in definition.name.lower()
                or self._filter_text in f"{identifier:#04x}"
            )
        ]
        self.endResetModel()

    def param_at(self, row: int) -> ParamDef:
        return PARAMS[self._ids[row]]

    def pending_items(self, only_selected: list[int] | None = None) -> list[list[float]]:
        ids = self._ids if only_selected is None else only_selected
        return [[identifier, self._pending[identifier]] for identifier in ids if identifier in self._pending]

    def pending_count(self) -> int:
        return len(self._pending)

    def readonly_count(self) -> int:
        return sum(1 for identifier in self._ids if PARAMS[identifier].access != "rw")

    # ------------------------------------------------------------ Qt model
    def rowCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return 0 if parent.isValid() else len(self._ids)

    def columnCount(self, parent: QModelIndex = QModelIndex()) -> int:
        return len(HEADERS)

    def headerData(self, section: int, orientation: Qt.Orientation, role: int = Qt.ItemDataRole.DisplayRole) -> Any:
        if role == Qt.ItemDataRole.DisplayRole and orientation == Qt.Orientation.Horizontal:
            return HEADERS[section]
        return None

    def flags(self, index: QModelIndex) -> Qt.ItemFlag:
        flags = Qt.ItemFlag.ItemIsEnabled | Qt.ItemFlag.ItemIsSelectable
        definition = self.param_at(index.row())
        if index.column() == COL_NEW and definition.access == "rw":
            flags |= Qt.ItemFlag.ItemIsEditable
        return flags

    def data(self, index: QModelIndex, role: int = Qt.ItemDataRole.DisplayRole) -> Any:
        if not index.isValid():
            return None
        definition = self.param_at(index.row())
        identifier = definition.id
        column = index.column()
        if role in (Qt.ItemDataRole.DisplayRole, Qt.ItemDataRole.EditRole):
            if column == COL_ID:
                return f"0x{identifier:02X}"
            if column == COL_NAME:
                return definition.name
            if column == COL_CURRENT:
                return format_value(self._values.get(identifier))
            if column == COL_UNIT:
                return definition.unit or "–"
            if column == COL_NEW:
                if definition.access != "rw":
                    return "—"
                return f"{self._pending[identifier]:g}" if identifier in self._pending else ""
            if column == COL_RANGE:
                return format_range(definition)
            if column == COL_ACCESS:
                return ACCESS_TEXT[definition.access]
            if column == COL_STATUS:
                return self._status_text(definition)
        if role == Qt.ItemDataRole.BackgroundRole:
            if identifier in self._invalid:
                return theme.color(theme.ERR_BG)
            if identifier in self._pending:
                return theme.color(theme.DIRTY_BG)
        if role == Qt.ItemDataRole.ForegroundRole:
            if definition.access != "rw":
                return theme.color(theme.MUTED)
            if column == COL_ACCESS and definition.access == "rw":
                return theme.color(theme.SEL_TEXT)
        if role == Qt.ItemDataRole.TextAlignmentRole and column in (COL_CURRENT, COL_NEW, COL_ID):
            return int(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        if role == Qt.ItemDataRole.ToolTipRole and definition.note:
            return definition.note
        return None

    def setData(self, index: QModelIndex, value: Any, role: int = Qt.ItemDataRole.EditRole) -> bool:
        if role != Qt.ItemDataRole.EditRole or index.column() != COL_NEW:
            return False
        definition = self.param_at(index.row())
        if definition.access != "rw":
            return False
        text = str(value).strip()
        identifier = definition.id
        self._invalid.discard(identifier)
        if not text:
            self._pending.pop(identifier, None)
        else:
            try:
                number = float(text)
            except ValueError:
                self._invalid.add(identifier)
            else:
                if not self._range_ok(definition, number):
                    self._invalid.add(identifier)
                else:
                    self._pending[identifier] = number
        self.dataChanged.emit(index, index, [Qt.ItemDataRole.DisplayRole, Qt.ItemDataRole.BackgroundRole])
        self.changedCountChanged.emit(self.pending_count())
        return True

    @staticmethod
    def _range_ok(definition: ParamDef, number: float) -> bool:
        if not math.isfinite(number):
            return False
        if definition.minimum is not None and number < definition.minimum:
            return False
        if definition.maximum is not None and number > definition.maximum:
            return False
        return True

    def set_pending(self, param_id: int, value: float) -> bool:
        """Stage a new value programmatically (import); True when accepted."""
        definition = PARAMS.get(param_id)
        if definition is None or definition.access != "rw" or not self._range_ok(definition, value):
            return False
        self._pending[param_id] = value
        self._invalid.discard(param_id)
        self._emit_row(param_id)
        self.changedCountChanged.emit(self.pending_count())
        return True

    def _status_text(self, definition: ParamDef) -> str:
        identifier = definition.id
        if identifier in self._invalid:
            return "无效"
        if self._status.get(identifier):
            return self._status[identifier]
        if identifier in self._pending:
            return "待写入"
        if definition.access == "cmd":
            return "命令"
        if definition.access == "ro":
            return "只读"
        return ""

    # ----------------------------------------------------------------- slots
    def set_value(self, param_id: int, value: Any) -> None:
        self._values[param_id] = value if value is None else float(value)
        self._status[param_id] = "✓" if value is not None else "✗"
        self._emit_row(param_id)

    def set_values(self, values: dict[int, Any]) -> None:
        for param_id, value in values.items():
            self._values[param_id] = value if value is None else float(value)
            self._status[param_id] = "✓" if value is not None else "✗"
        if self._ids:
            top = self.index(0, 0)
            bottom = self.index(len(self._ids) - 1, len(HEADERS) - 1)
            self.dataChanged.emit(top, bottom)

    def clear_pending(self, param_id: int | None = None) -> None:
        if param_id is None:
            self._pending.clear()
            self._invalid.clear()
        else:
            self._pending.pop(param_id, None)
            self._invalid.discard(param_id)
        self.changedCountChanged.emit(self.pending_count())
        if self._ids:
            self.dataChanged.emit(self.index(0, 0), self.index(len(self._ids) - 1, len(HEADERS) - 1))

    def values_snapshot(self) -> dict[int, float]:
        return {identifier: value for identifier, value in self._values.items() if value is not None}

    def _emit_row(self, param_id: int) -> None:
        if param_id in self._ids:
            row = self._ids.index(param_id)
            self.dataChanged.emit(self.index(row, 0), self.index(row, len(HEADERS) - 1))
