"""Log page: application event/frame log with clear and save-to-file."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

from PySide6.QtWidgets import QFileDialog, QHBoxLayout, QLabel, QListWidget, QListWidgetItem, QPushButton, QVBoxLayout, QWidget

from mdrive_gui import theme
from mdrive_gui.app_state import AppState
from mdrive_gui.worker import CanWorker

MAX_ENTRIES = 5000


class LogPage(QWidget):
    """Tab 5: every AppState log entry, newest at the bottom."""

    def __init__(self, state: AppState, worker: CanWorker, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.state = state
        self.worker = worker

        self.count_label = QLabel("0 条")
        self.count_label.setProperty("muted", True)
        clear_button = QPushButton("清空")
        clear_button.clicked.connect(self._clear)
        save_button = QPushButton("保存到文件")
        save_button.clicked.connect(self._save)

        row = QHBoxLayout()
        row.addWidget(self.count_label)
        row.addStretch(1)
        row.addWidget(clear_button)
        row.addWidget(save_button)

        self.list = QListWidget()
        layout = QVBoxLayout(self)
        layout.addLayout(row)
        layout.addWidget(self.list, 1)

        state.logEmitted.connect(self._on_log)

    def _on_log(self, level: str, message: str) -> None:
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        item = QListWidgetItem(f"{stamp}  [{level:<5}]  {message}")
        if level == "ERROR":
            item.setForeground(theme.color(theme.ERROR))
        elif level == "WARN":
            item.setForeground(theme.color(theme.WARN))
        self.list.addItem(item)
        while self.list.count() > MAX_ENTRIES:
            self.list.takeItem(0)
        self.list.scrollToBottom()
        self.count_label.setText(f"{self.list.count()} 条")

    def _clear(self) -> None:
        self.list.clear()
        self.count_label.setText("0 条")

    def _save(self) -> None:
        if self.list.count() == 0:
            self.state.log("WARN", "日志为空，无需保存")
            return
        path, _ = QFileDialog.getSaveFileName(self, "保存日志", "mdrive_gui.log", "日志文件 (*.log *.txt)")
        if not path:
            return
        lines = [self.list.item(row).text() for row in range(self.list.count())]
        try:
            Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")
        except OSError as exc:
            self.state.raise_error(f"保存日志失败: {exc}")
            return
        self.state.log("INFO", f"日志已保存到 {path}")
