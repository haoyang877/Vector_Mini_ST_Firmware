"""总览图始终从原始 PNG 绘制，并按屏幕像素计算缩放。"""

from PySide6.QtCore import QEvent, Qt, Signal
from PySide6.QtGui import QPainter, QPixmap
from PySide6.QtWidgets import QScrollArea, QWidget


class _WaveformImage(QWidget):
    def __init__(self, pixmap: QPixmap) -> None:
        super().__init__()
        self.pixmap = pixmap

    def paintEvent(self, event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.SmoothPixmapTransform)
        painter.drawPixmap(self.rect(), self.pixmap)


class WaveformView(QScrollArea):
    """以 100% 表示一个源像素对应一个屏幕像素。"""

    scale_changed = Signal(float)

    def __init__(self, pixmap: QPixmap) -> None:
        super().__init__()
        self.image = _WaveformImage(pixmap)
        self.requested_scale: float | None = None
        self.scale = 1.0
        self.setAlignment(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignTop)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.setWidget(self.image)
        self._layout_image()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._layout_image()

    def event(self, event) -> bool:
        handled = super().event(event)
        if event.type() == QEvent.Type.DevicePixelRatioChange:
            self._layout_image()
        return handled

    def set_scale(self, scale: float | None) -> None:
        # 放大时保留视口中心附近的波形位置。
        horizontal, vertical = self.horizontalScrollBar(), self.verticalScrollBar()
        width, height = self.viewport().width(), self.viewport().height()
        old_width, old_height = self.image.width(), self.image.height()
        x = (horizontal.value() + min(width, old_width) / 2) / old_width
        y = (vertical.value() + min(height, old_height) / 2) / old_height
        self.requested_scale = scale
        self._layout_image()
        horizontal.setValue(round(x * self.image.width() - self.viewport().width() / 2))
        vertical.setValue(round(y * self.image.height() - self.viewport().height() / 2))

    def zoom(self, factor: float) -> None:
        self.set_scale(max(0.10, min(2.0, self.scale * factor)))

    def _layout_image(self) -> None:
        pixel_ratio = self.devicePixelRatioF()
        source_width, source_height = self.image.pixmap.width(), self.image.pixmap.height()
        scale = self.requested_scale
        if scale is None:
            scale = min(1.0, self.viewport().width() * pixel_ratio / source_width)
        self.scale = scale
        self.image.resize(
            max(1, round(source_width * scale / pixel_ratio)),
            max(1, round(source_height * scale / pixel_ratio)),
        )
        self.scale_changed.emit(scale)
