"""Plots page: live status-stream curves (0x64 stream, 48-byte frames) with CSV recording."""

from __future__ import annotations

from collections import deque
from datetime import datetime
import math
from pathlib import Path

import pyqtgraph as pg
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from mdrive_gui import theme
from mdrive_gui.app_state import AppState
from mdrive_gui.worker import CanWorker

CHANNELS = (
    ("位置反馈", "position_feedback_rad", "°", theme.ACCENT, 180.0 / math.pi),
    ("位置目标", "position_target_rad", "°", "#BA68C8", 180.0 / math.pi),
    ("速度", "speed_feedback_rad_s", "rad/s", theme.OK, 1.0),
    ("Iq", "iq_feedback_A", "A", theme.WARN, 1.0),
    ("Iq指令", "iq_reference_A", "A", "#FF8A65", 1.0),
    ("温度", "temperature_C", "°C", "#4DD0E1", 1.0),
    ("母线", "bus_voltage_V", "V", "#AED581", 1.0),
)


def _muted(text: str) -> QLabel:
    label = QLabel(text)
    label.setProperty("muted", True)
    return label


class PlotsPage(QWidget):
    """Tab 4: configure the CAN status stream and render rolling curves."""

    def __init__(self, state: AppState, worker: CanWorker, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.state = state
        self.worker = worker
        self._curves: dict[str, pg.PlotDataItem] = {}
        self._buffers: dict[str, deque[tuple[float, float]]] = {}
        self._checks: dict[str, QCheckBox] = {}
        self._last_sample_time = 0.0
        self._window_s = 10.0
        self._frames = 0
        self._recording_path: Path | None = None
        self._streaming = False

        # ------------------------------------------------------------ config
        config = QWidget()
        config.setObjectName("card")
        config_layout = QVBoxLayout(config)
        title = QLabel("配置")
        title.setProperty("cardTitle", True)
        config_layout.addWidget(title)

        rate_row = QHBoxLayout()
        rate_row.addWidget(_muted("速率 (Hz):"))
        self.rate_spin = QSpinBox()
        self.rate_spin.setRange(10, 200)
        self.rate_spin.setValue(100)
        rate_row.addWidget(self.rate_spin)
        rate_row.addStretch(1)
        config_layout.addLayout(rate_row)

        control_row = QHBoxLayout()
        self.start_button = QPushButton("▶ 启动")
        self.start_button.setProperty("variant", "primary")
        self.start_button.clicked.connect(self._start)
        self.stop_button = QPushButton("■ 停止")
        self.stop_button.clicked.connect(self._stop)
        self.stop_button.setEnabled(False)
        clear_button = QPushButton("清空")
        clear_button.clicked.connect(self._clear)
        control_row.addWidget(self.start_button)
        control_row.addWidget(self.stop_button)
        control_row.addWidget(clear_button)
        control_row.addStretch(1)
        config_layout.addLayout(control_row)

        window_row = QHBoxLayout()
        window_row.addWidget(_muted("时间窗:"))
        self.window_combo = QComboBox()
        self.window_combo.addItems(["5 s", "10 s", "30 s", "60 s"])
        self.window_combo.setCurrentIndex(1)
        self.window_combo.currentIndexChanged.connect(self._update_window)
        window_row.addWidget(self.window_combo)
        window_row.addStretch(1)
        config_layout.addLayout(window_row)

        config_layout.addWidget(_muted("通道:"))
        for caption, key, unit, color, _scale in CHANNELS:
            box = QCheckBox(f"{caption} ({unit})")
            box.setChecked(caption in ("位置反馈", "位置目标", "速度"))
            box.toggled.connect(lambda checked, k=key: self._channel_toggled(k, checked))
            self._checks[key] = box
            config_layout.addWidget(box)

        self.record_box = QCheckBox("录制到 CSV")
        config_layout.addWidget(self.record_box)
        config_layout.addStretch(1)
        note = _muted("数据源：CAN 状态流 0x7F0+node（48B 帧）。统计基于主机接收时间；无效哨兵自动断开。")
        note.setWordWrap(True)
        config_layout.addWidget(note)
        config.setMaximumWidth(260)

        # -------------------------------------------------------------- plot
        self.plot = pg.PlotWidget(background=theme.PLOT_BG)
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "时间", units="s", color=theme.MUTED)
        self.plot.setLabel("left", "通道值", color=theme.MUTED)
        for axis_name in ("bottom", "left"):
            axis = self.plot.getPlotItem().getAxis(axis_name)
            axis.setPen(pg.mkPen(color=theme.MUTED))
            axis.setTextPen(pg.mkPen(color=theme.MUTED))
        legend = self.plot.getPlotItem().addLegend(offset=(8, 8))
        legend.setLabelTextColor(theme.MUTED)
        self._legend = legend
        self._legend_names: dict[str, str] = {}
        for caption, key, unit, color, _scale in CHANNELS:
            curve = self.plot.plot([], [], pen=pg.mkPen(color=color, width=2), name=f"{caption} ({unit})")
            self._curves[key] = curve
            self._buffers[key] = deque()
            self._legend_names[key] = f"{caption} ({unit})"
        # Apply the initial checkbox state to curve visibility.
        for key, box in self._checks.items():
            self._channel_toggled(key, box.isChecked())

        self.stats = _muted("统计: — Hz ｜ 帧 0 ｜ 录制 关")
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.addWidget(self.plot, 1)
        right_layout.addWidget(self.stats)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(config)
        splitter.addWidget(right)
        splitter.setStretchFactor(1, 1)

        layout = QVBoxLayout(self)
        layout.addWidget(splitter)

        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(50)
        self._refresh_timer.timeout.connect(self._refresh_curves)
        self._refresh_timer.start()

        worker.streamSamples.connect(self._on_samples)
        worker.streamStats.connect(self._on_stats)
        worker.streamStopped.connect(self._on_stopped)
        state.connectionChanged.connect(self._on_connection)
        self._on_connection(state.connected)

    # ------------------------------------------------------------- actions
    def _update_window(self) -> None:
        self._window_s = float(self.window_combo.currentText().split()[0])

    def _channel_toggled(self, key: str, checked: bool) -> None:
        self._curves[key].setVisible(checked)
        self._legend.removeItem(self._curves[key])
        if checked:
            self._legend.addItem(self._curves[key], self._legend_names[key])
        else:
            self._buffers[key].clear()
            self._curves[key].setData([], [])

    def _start(self) -> None:
        if not self.state.connected:
            self.state.raise_error("CAN 设备未打开，请先在连接页打开设备")
            return
        self._recording_path = None
        if self.record_box.isChecked():
            default = f"stream_{datetime.now():%Y%m%d_%H%M%S}.csv"
            path, _ = QFileDialog.getSaveFileName(self, "录制到 CSV", default, "CSV (*.csv)")
            if not path:
                return
            self._recording_path = Path(path)
        self._clear()
        self._frames = 0
        self._streaming = True
        self.start_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.worker.submit(
            "stream_start",
            node=self.state.selected_node,
            rate_hz=self.rate_spin.value(),
            record_path=str(self._recording_path) if self._recording_path else None,
        )
        self.state.log("INFO", f"启动状态流：节点 {self.state.selected_node}，{self.rate_spin.value()} Hz")

    def _stop(self) -> None:
        self.worker.request_stream_stop()

    def _clear(self) -> None:
        for key, buffer in self._buffers.items():
            buffer.clear()
            self._curves[key].setData([], [])

    # -------------------------------------------------------------- events
    def _on_samples(self, samples: list) -> None:
        now = 0.0
        for sample in samples:
            now = sample["time_s"]
            for caption, key, unit, color, scale in CHANNELS:
                value = sample.get(key)
                if value is None:
                    continue
                self._buffers[key].append((now, value * scale))
            self._frames += 1
        self._last_sample_time = now

    def _on_stats(self, stats: dict) -> None:
        record = f"开 ({self._recording_path.name})" if self._recording_path else "关"
        self.stats.setText(
            f"统计: {stats.get('actual_hz', 0):.1f} Hz ｜ 帧 {stats.get('frames', 0)} ｜ 录制 {record}"
        )

    def _on_stopped(self, summary: dict) -> None:
        self._streaming = False
        self.start_button.setEnabled(self.state.connected)
        self.stop_button.setEnabled(False)
        rows = summary.get("recorded_rows", 0)
        text = f"状态流停止：{summary.get('frames', 0)} 帧，实际 {summary.get('actual_hz', 0):.1f} Hz"
        if rows:
            text += f"，已写 CSV {rows} 行"
        self.state.log("INFO", text)

    def _refresh_curves(self) -> None:
        if self._last_sample_time <= 0.0:
            return
        cutoff = self._last_sample_time - self._window_s
        for key, buffer in self._buffers.items():
            while buffer and buffer[0][0] < cutoff:
                buffer.popleft()
            if buffer:
                times, values = zip(*buffer)
                self._curves[key].setData(times, values)

    def _on_connection(self, connected: bool) -> None:
        self.start_button.setEnabled(connected and not self._streaming)
