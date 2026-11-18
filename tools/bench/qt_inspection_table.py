"""生产检测结果表、异常提示和行内连续角度图。"""

from collections import Counter
from collections.abc import Callable

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from qt_production_icons import icon
from qt_speed_plot import InspectionAnglePlot

RESULT_COLORS = {
    "passed": "#18765b",
    "failed": "#b03b42",
    "review": "#875313",
    "pending": "#64748b",
    "excluded": "#64748b",
}
ROW_COLORS = {"failed": "#fff0f0", "review": "#fff7e9", "running": "#e9f4fb"}


class InspectionTable(QTableWidget):
    """每次阶段消息更新结果；连续采样只更新同一个曲线控件。"""

    def __init__(self, show_curve: Callable[[str, str], None]) -> None:
        super().__init__(0, 5)
        self.setHorizontalHeaderLabels(["序号", "测试项", "实测结果", "判定标准", "结果"])
        self.verticalHeader().hide()
        self.verticalHeader().setDefaultSectionSize(34)
        self.setAlternatingRowColors(True)
        self.setShowGrid(False)
        self.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeMode.Stretch)
        self.setColumnWidth(0, 52)
        self.setColumnWidth(1, 162)
        self.setColumnWidth(3, 218)
        self.setColumnWidth(4, 94)
        self.plot = InspectionAnglePlot(self)
        self.plot.setMinimumHeight(170)
        self.plot.hide()
        self.show_curve = show_curve
        self.curve_row: int | None = None
        self.curve_button: QPushButton | None = None
        self.curve_expanded = False

    def display(
        self,
        results: list[dict],
        *,
        available_curves: frozenset[str] = frozenset(),
    ) -> str:
        # 结果刷新先移开绘图控件，避免旧表格单元销毁同一轮已采集的样本。
        self.plot.hide()
        self.plot.setParent(self)
        self.clearContents()
        self.clearSpans()
        self.curve_row = None
        self.curve_button = None
        has_angle = any(result.get("evidence") == "angle" for result in results)
        self.setRowCount(len(results) + int(has_angle))
        for index in range(self.rowCount()):
            self.setRowHidden(index, False)
            self.setRowHeight(index, 34)
        row = 0
        section_number = detail_number = 0
        for result in results:
            if result.get("subitem"):
                detail_number += 1
                number = f"{section_number}.{detail_number}"
                self.setRowHeight(row, 31)
            else:
                section_number += 1
                detail_number = 0
                number = str(section_number)
            kind = result.get("evidence", "")
            has_curve = kind == "angle" or (
                kind in available_curves and result["outcome"] != "excluded"
            )
            self.display_result(row, number, result, has_curve, compact=has_angle)
            row += 1
            if result.get("evidence") == "angle":
                self.insert_plot(row)
                row += 1
        counts = Counter(item["outcome"] for item in results if not item.get("group"))
        return f"通过 {counts['passed']} · 失败 {counts['failed']} · 待复核 {counts['review']}"

    def display_result(
        self, row: int, number: str, result: dict, has_curve: bool, *, compact: bool
    ) -> None:
        status = result["outcome"]
        color = RESULT_COLORS[status]
        background = ROW_COLORS.get("running" if result["result"] == "进行中" else status)
        values = (number, result["item"], result["measured"], result["limit"], result["result"])
        for column, text in enumerate(values):
            value = QTableWidgetItem(text)
            if column == 0:
                value.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            value.setToolTip(text)
            if not compact or column == 4 or status in ("failed", "review", "pending", "excluded"):
                value.setForeground(QColor(color))
            if background:
                value.setBackground(QColor(background))
            if column == 4 or (column == 1 and not result.get("subitem")):
                font = self.font()
                font.setBold(True)
                value.setFont(font)
            self.setItem(row, column, value)

        if not has_curve:
            return
        kind = result["evidence"]
        evidence = QWidget()
        layout = QHBoxLayout(evidence)
        layout.setContentsMargins(3, 1, 4, 1)
        layout.setSpacing(4)
        measured = QLabel(result["measured"])
        measured.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
        measured.setToolTip(result["measured"])
        if status != "passed":
            measured.setStyleSheet(f"color:{color};")
        measured.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(measured, 1)
        action = QPushButton(
            {"angle": "展开曲线", "encoder": "补偿曲线", "hall": "查看波形"}.get(kind, "查看曲线")
        )
        action.setAccessibleName(result["item"] + " · " + action.text())
        action.setIcon(icon("chart"))
        action.setObjectName("rowAction")
        if kind == "angle":
            self.curve_button = action
            action.clicked.connect(self.toggle_curve)
            action.setVisible(bool(self.plot.samples))
        else:
            action.clicked.connect(lambda: self.show_curve(kind, result["item"]))
        layout.addWidget(action)
        self.setCellWidget(row, 2, evidence)

    def insert_plot(self, row: int) -> None:
        self.curve_row = row
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(4, 0, 4, 2)
        layout.addWidget(self.plot)
        self.setSpan(row, 0, 1, self.columnCount())
        self.setItem(row, 0, QTableWidgetItem())
        self.setCellWidget(row, 0, panel)
        self.plot.show()
        self.setRowHeight(row, self.plot.minimumHeight() + 2)
        self.set_curve_expanded(self.curve_expanded and bool(self.plot.samples))

    def toggle_curve(self) -> None:
        self.set_curve_expanded(not self.curve_expanded)
        if self.curve_expanded and self.curve_row is not None:
            self.scrollToItem(self.item(self.curve_row, 0))

    def set_curve_expanded(self, expanded: bool) -> None:
        self.curve_expanded = expanded
        if self.curve_row is not None and self.curve_button is not None:
            self.setRowHidden(self.curve_row, not expanded)
            self.curve_button.setText("收起曲线" if expanded else "展开曲线")
            self.curve_button.setAccessibleDescription("曲线已展开" if expanded else "曲线已收起")

    def append_angle(self, sample: dict) -> None:
        first = not self.plot.samples
        self.plot.append(sample)
        if first and self.plot.samples and self.curve_button is not None:
            self.curve_button.show()
            self.toggle_curve()

    def clear_angles(self) -> None:
        self.plot.clear()
        self.set_curve_expanded(False)
        if self.curve_button is not None:
            self.curve_button.hide()
