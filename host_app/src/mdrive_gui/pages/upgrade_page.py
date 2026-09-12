"""Firmware upgrade page: image inspection + Loader-driven CAN upgrade with progress."""

from __future__ import annotations

import json
from pathlib import Path

from PySide6.QtWidgets import (
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from mdrive_core.services.image_prep import ImagePrepError, prepare_image_file
from mdrive_gui import theme
from mdrive_gui.app_state import AppState
from mdrive_gui.worker import CanWorker


def _card(title: str, step: int | None = None) -> tuple[QWidget, QVBoxLayout]:
    card = QWidget()
    card.setObjectName("card")
    layout = QVBoxLayout(card)
    head = QLabel((f"{step} " if step else "") + title)
    head.setProperty("cardTitle", True)
    layout.addWidget(head)
    return card, layout


def _muted(text: str) -> QLabel:
    label = QLabel(text)
    label.setProperty("muted", True)
    label.setWordWrap(True)
    return label


class UpgradePage(QWidget):
    """Tab 2: ①选择镜像 ②目标设备 ③执行 ④结果."""

    def __init__(self, state: AppState, worker: CanWorker, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.state = state
        self.worker = worker
        self._image_path: Path | None = None
        self._prepared_size = 0
        self._prepared_crc = 0
        self._last_summary: dict | None = None

        card1, c1 = _card("选择镜像", step=1)
        row = QHBoxLayout()
        self.pick_button = QPushButton("选择 .bin …")
        self.pick_button.clicked.connect(self._pick_image)
        self.file_label = _muted("未选择文件")
        row.addWidget(self.pick_button)
        row.addWidget(self.file_label, 1)
        row.addWidget(_muted("版本号"))
        self.version_edit = QLineEdit("1")
        self.version_edit.setMaximumWidth(90)
        row.addWidget(self.version_edit)
        c1.addLayout(row)
        self.size_label = _muted("大小 —")
        self.crc_label = _muted("CRC32 —")
        self.check_label = _muted("校验 —")
        c1.addWidget(self.size_label)
        c1.addWidget(self.crc_label)
        c1.addWidget(self.check_label)

        card2, c2 = _card("目标设备", step=2)
        row2 = QHBoxLayout()
        self.node_label = _muted("节点 —")
        row2.addWidget(self.node_label)
        row2.addWidget(_muted("升级将自动发送 0x66 使设备进入 Loader；不擦除参数/标定区"))
        row2.addStretch(1)
        c2.addLayout(row2)

        card3, c3 = _card("执行", step=3)
        row3 = QHBoxLayout()
        self.start_button = QPushButton("开始升级")
        self.start_button.setProperty("variant", "primary")
        self.start_button.setEnabled(False)
        self.start_button.clicked.connect(self._start_upgrade)
        row3.addWidget(self.start_button)
        self.cancel_button = QPushButton("中止")
        self.cancel_button.setEnabled(False)
        self.cancel_button.clicked.connect(self._cancel_upgrade)
        row3.addWidget(self.cancel_button)
        self.phase_label = _muted("阶段：待命")
        row3.addWidget(self.phase_label, 1)
        c3.addLayout(row3)
        self.phase_chips = _muted(self._phase_text(None, set()))
        c3.addWidget(self.phase_chips)
        self.progress = QProgressBar()
        self.progress.setValue(0)
        self.progress.setVisible(False)
        c3.addWidget(self.progress)

        card4, c4 = _card("结果")
        row4 = QHBoxLayout()
        self.result_label = _muted("升级结果与阶段耗时将在此显示。")
        row4.addWidget(self.result_label, 1)
        self.export_button = QPushButton("导出日志")
        self.export_button.setEnabled(False)
        self.export_button.clicked.connect(self._export_log)
        row4.addWidget(self.export_button)
        c4.addLayout(row4)

        layout = QVBoxLayout(self)
        for card in (card1, card2, card3, card4):
            layout.addWidget(card)
        layout.addStretch(1)

        worker.progress.connect(self._on_progress)
        worker.finished.connect(self._on_finished)
        worker.failed.connect(self._on_failed)
        worker.upgradePhase.connect(self._on_phase)
        state.connectionChanged.connect(self._on_connection)
        state.selectedNodeChanged.connect(self._on_node)
        self._on_connection(state.connected)
        self._on_node(state.selected_node)

    # ------------------------------------------------------------- actions
    def _pick_image(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "选择固件镜像", "", "固件镜像 (*.bin);;所有文件 (*)")
        if not path:
            return
        try:
            prepared = prepare_image_file(Path(path))
        except ImagePrepError as exc:
            self.state.raise_error(f"镜像检查失败: {exc}")
            return
        self._image_path = Path(path)
        self._prepared_size = prepared.size
        self._prepared_crc = prepared.crc32
        self.file_label.setText(self._image_path.name)
        self.size_label.setText(f"大小 {prepared.size:,} B（去除尾部 0xFF 后）")
        self.crc_label.setText(f"CRC32 0x{prepared.crc32:08X}")
        self.check_label.setText("✓ 8 字节对齐  ✓ 未超 96 KiB")
        self.check_label.setStyleSheet(f"color: {theme.OK}")
        self._update_start_enabled()
        self.state.log("INFO", f"已加载镜像 {self._image_path.name}：{prepared.size:,} B，CRC32 0x{prepared.crc32:08X}")

    def _update_start_enabled(self) -> None:
        self.start_button.setEnabled(
            self.state.connected and self._image_path is not None and not self.progress.isVisible()
        )

    def _start_upgrade(self) -> None:
        if self._image_path is None:
            return
        try:
            version = int(self.version_edit.text(), 0)
        except ValueError:
            self.state.raise_error("版本号必须是整数")
            return
        answer = QMessageBox.warning(
            self,
            "确认升级",
            f"即将擦除 APP 区（不影响参数/标定）并下载固件：\n\n{self._image_path.name}\n"
            f"版本 {version}，{self._prepared_size:,} B，CRC32 0x{self._prepared_crc:08X}\n\n"
            "擦除期间请勿断电；中断后设备将停留在 Loader 恢复模式。是否继续？",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self._last_summary = None
        self._phase_done = set()
        self._phase_current: str | None = None
        self.phase_chips.setText(self._phase_text(None, self._phase_done))
        self.cancel_button.setEnabled(True)
        self.export_button.setEnabled(False)
        self.result_label.setText("升级中…")
        self.progress.setVisible(True)
        self.progress.setRange(0, 100)
        self.progress.setValue(0)
        self.phase_label.setText("阶段：进入 Boot / 下载中")
        self.start_button.setEnabled(False)
        self.worker.submit(
            "upgrade",
            image_path=str(self._image_path),
            version=version,
            node=self.state.selected_node,
        )

    PHASE_ORDER = ("begin", "erase", "program", "verify", "activate", "app_alive")
    PHASE_LABELS = {
        "begin": "BEGIN",
        "erase": "ERASE",
        "program": "PROGRAM",
        "verify": "VERIFY",
        "activate": "ACTIVATE",
        "app_alive": "APP",
    }

    def _phase_text(self, current: str | None, done: set) -> str:
        parts = []
        for phase in self.PHASE_ORDER:
            mark = "✓" if phase in done else ("▶" if phase == current else "○")
            parts.append(f"{self.PHASE_LABELS[phase]} {mark}")
        return "  →  ".join(parts)

    def _on_phase(self, phase: str) -> None:
        if phase not in self.PHASE_ORDER:
            return
        index = self.PHASE_ORDER.index(phase)
        self._phase_done.update(self.PHASE_ORDER[:index])
        self._phase_current = phase
        if phase == "app_alive":
            self._phase_done.add("app_alive")
            self._phase_current = None
        self.phase_chips.setText(self._phase_text(self._phase_current, self._phase_done))
        self.phase_label.setText(f"阶段：{self.PHASE_LABELS[phase]}")

    def _cancel_upgrade(self) -> None:
        self.cancel_button.setEnabled(False)
        self.phase_label.setText("阶段：正在中止…")
        self.worker.request_upgrade_abort()

    def _export_log(self) -> None:
        if self._last_summary is None:
            return
        path, _ = QFileDialog.getSaveFileName(self, "导出升级日志", "upgrade_log.json", "JSON (*.json)")
        if not path:
            return
        Path(path).write_text(json.dumps(self._last_summary, ensure_ascii=False, indent=2), encoding="utf-8")
        self.state.log("INFO", f"升级日志已导出到 {path}")

    # -------------------------------------------------------------- events
    def _on_progress(self, task: str, done: int, total: int) -> None:
        if task != "upgrade" or total <= 0:
            return
        self.progress.setRange(0, total)
        self.progress.setValue(min(done, total))
        percent = done * 100 / total
        if not self.phase_label.text().startswith("阶段：校验"):
            self.phase_label.setText(f"阶段：下载中 {percent:.0f}%")

    def _on_finished(self, task: str, payload: dict) -> None:
        if task != "upgrade":
            return
        self.progress.setVisible(False)
        self.cancel_button.setEnabled(False)
        self._last_summary = payload
        if payload.get("status") == "aborted":
            self.result_label.setText("⚠ 升级已中止：Loader 会话已复位，设备停留在恢复模式（可重新升级）")
            self.result_label.setStyleSheet(f"color: {theme.WARN}")
            self.phase_label.setText("阶段：已中止")
            self._update_start_enabled()
            return
        self.export_button.setEnabled(True)
        timings = payload.get("timings_ms", {})
        timing_text = " ｜ ".join(f"{key} {value:.0f}ms" for key, value in timings.items()) if timings else "-"
        self.result_label.setText(
            f"✅ 升级成功：{payload.get('image_size', 0):,} B，session {payload.get('session', 0)}"
        )
        self.result_label.setStyleSheet(f"color: {theme.OK}")
        self.phase_label.setText(f"阶段：完成 ｜ {timing_text}")
        self._update_start_enabled()

    def _on_failed(self, message: str) -> None:
        if not self.progress.isVisible():
            return
        self.progress.setVisible(False)
        self.cancel_button.setEnabled(False)
        self.result_label.setText(f"❌ 升级失败：{message}")
        self.result_label.setStyleSheet(f"color: {theme.ERROR}")
        self.phase_label.setText("阶段：失败")
        self._update_start_enabled()

    def _on_connection(self, connected: bool) -> None:
        self._update_start_enabled()

    def _on_node(self, node: int) -> None:
        self.node_label.setText(f"节点 {node}")
