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

    channels = CHANNELS
    caption = "最近30秒   蓝色：参考   青色：反馈"
    empty_text = "使能速度控制后显示实时反馈"
    minimum_span_s = 30
    channel_columns = 1

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.samples = deque(maxlen=1200)
        self.setMinimumHeight(300)
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
        painter.drawText(14, 23, self.caption)
        end = max(self.minimum_span_s, self.samples[-1]["elapsed_s"] if self.samples else 0)
        begin = max(0, end - 30)
        columns = self.channel_columns
        count = math.ceil(len(self.channels) / columns)
        height = (self.height() - 70) / count
        width = self.width() / columns
        for index, (label, reference, feedback) in enumerate(self.channels):
            row, column = divmod(index, columns)
            offset = int(column * width)
            left, right = offset + 68, int((column + 1) * width) - 20
            top, bottom = 58 + row * height, 32 + (row + 1) * height
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
            painter.drawText(left, int(top - 8), label)
            for fraction in (0, 0.5, 1):
                y = bottom - fraction * (bottom - top)
                painter.setPen(QColor("#e6edf4"))
                painter.drawLine(QPointF(left, y), QPointF(right, y))
                painter.setPen(QColor("#64748b"))
                painter.drawText(offset + 12, int(y + 4), f"{low + fraction * (high - low):.2g}")
            for key, color in ((reference, "#5685e6"), (feedback, "#009b92")):
                if key is None:
                    continue
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
                        left + (point["elapsed_s"] - begin) / (end - begin) * (right - left),
                        bottom - (value - low) / (high - low) * (bottom - top),
                    )
                    if previous is not None:
                        painter.drawLine(previous, current)
                    else:
                        painter.drawPoint(current)
                    previous = current
            if self.samples and row == count - 1:
                painter.setPen(QColor("#64748b"))
                painter.drawText(left, self.height() - 10, f"{begin:.1f} s")
                painter.drawText(right - 44, self.height() - 10, f"{end:.1f} s")
        if not self.samples:
            painter.setPen(QColor("#64748b"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, self.empty_text)


class InspectionAnglePlot(SpeedPlot):
    """同一次高速会话的轴角，108 坐标由本轮固件能力确定。"""

    caption = "本轮相对角度 · 有效样本连续绘制"
    empty_text = "高速检测开始后显示角度"
    minimum_span_s = 1
    channels = (("轴角 / °", None, "reference_deg"),)

    def append(self, sample: dict) -> None:
        name = "输出跟踪角" if sample["reference_source"] == "output_tracking" else "电机编码器"
        self.channels = ((f"{name} / °", None, "reference_deg"),)
        super().append(sample)
