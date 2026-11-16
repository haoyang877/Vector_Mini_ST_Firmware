"""标定导出图的只读预览，按可用宽度缩放并纵向滚动。"""

from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import QLabel, QScrollArea


class CalibrationPreview(QScrollArea):
    """持有本地图片；调整窗口大小只重绘，不重新读取文件或设备。"""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMinimumSize(300, 280)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.image = QPixmap()
        self.message = "标定完成并导出后，在此显示补偿曲线"
        self.label = QLabel()
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label.setWordWrap(True)
        self.setWidget(self.label)
        self.render()

    def display(self, path: Path | None) -> None:
        """加载导出PNG，空路径或无效图片清空上轮结果。"""
        self.image = QPixmap(str(path)) if path else QPixmap()
        self.message = (
            "曲线文件无法读取，请检查本次记录" if path else "标定完成并导出后，在此显示补偿曲线"
        )
        self.verticalScrollBar().setValue(0)
        self.render()

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self.render()

    def render(self) -> None:
        width = max(1, self.viewport().width())
        if self.image.isNull():
            self.label.clear()
            self.label.setText(self.message)
            self.label.setFixedSize(width, max(1, self.viewport().height()))
        else:
            scaled = self.image.scaledToWidth(width, Qt.TransformationMode.SmoothTransformation)
            self.label.setPixmap(scaled)
            self.label.setFixedSize(scaled.size())
