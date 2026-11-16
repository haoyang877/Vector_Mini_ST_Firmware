"""Qt实时曲线：有效数据断点、有界缓存、10Hz绘制，不读取设备。"""

import math
from collections import deque

from PySide6.QtCore import QPointF, Qt, QTimer
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import QWidget

CHANNELS = (
    ("位置 / rad", "reference_position_rad", "position_rad"),
    ("速度 / rad/s", "reference_speed_rad_s", "actual_rad_s"),
    ("Iq / A", "reference_iq_a", "iq_a"),
)


class SpeedPlot(QWidget):
    """GUI线程持有，最多1200点、30秒，缺失值不会连线。"""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.samples = deque(maxlen=1200)
        self.setMinimumHeight(260)
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update)

    def clear(self) -> None:
        self.samples.clear()
        self.update()

    def append(self, sample: dict) -> None:
        stamp = sample.get("elapsed_s")
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            return
        if self.samples and stamp < self.samples[-1]["elapsed_s"]:
            self.samples.clear()
        self.samples.append(dict(sample))
        while self.samples and stamp - self.samples[0]["elapsed_s"] > 30:
            self.samples.popleft()
        if not self.timer.isActive():
            self.timer.start()

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#ffffff"))
        painter.setPen(QColor("#64748b"))
        painter.drawText(14, 23, "最近30秒   蓝色：参考   青色：反馈")
        end = max(30, self.samples[-1]["elapsed_s"] if self.samples else 30)
        begin = end - 30
        left, right = 108, self.width() - 20
        height = (self.height() - 55) / 3
        for index, (label, reference, feedback) in enumerate(CHANNELS):
            top, bottom = 42 + index * height, 32 + (index + 1) * height
            values = [
                point[key]
                for point in self.samples
                for key in (reference, feedback)
                if point.get(key) is not None and math.isfinite(point[key])
            ]
            low, high = (min(values), max(values)) if values else (-1, 1)
            margin = max((high - low) * 0.1, 0.01)
            low, high = low - margin, high + margin
            painter.setPen(QColor("#64748b"))
            painter.drawText(12, int((top + bottom) / 2), label)
            for fraction in (0, 0.5, 1):
                y = bottom - fraction * (bottom - top)
                painter.setPen(QColor("#e6edf4"))
                painter.drawLine(QPointF(left, y), QPointF(right, y))
                painter.setPen(QColor("#64748b"))
                painter.drawText(65, int(y), f"{low + fraction * (high - low):.2g}")
            for key, color in ((reference, "#5685e6"), (feedback, "#009b92")):
                pen = QPen(QColor(color), 1.6)
                if key == reference:
                    pen.setStyle(Qt.PenStyle.DashLine)
                painter.setPen(pen)
                previous = None
                for point in self.samples:
                    value = point.get(key)
                    if value is None or not math.isfinite(value):
                        previous = None
                        continue
                    current = QPointF(
                        left + (point["elapsed_s"] - begin) / 30 * (right - left),
                        bottom - (value - low) / (high - low) * (bottom - top),
                    )
                    if previous is not None:
                        painter.drawLine(previous, current)
                    else:
                        painter.drawPoint(current)
                    previous = current
        if not self.samples:
            painter.setPen(QColor("#64748b"))
            painter.drawText(
                self.rect(), Qt.AlignmentFlag.AlignCenter, "使能速度控制后显示实时反馈"
            )
