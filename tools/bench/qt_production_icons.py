"""生产图标唯一加载入口；按颜色与状态缓存，不参与任何设备操作。"""

import sys
from functools import lru_cache
from pathlib import Path

from PySide6.QtCore import QByteArray, QRectF, QSize
from PySide6.QtGui import QIcon, QPainter, QPixmap
from PySide6.QtSvg import QSvgRenderer

ICON_SIZE = QSize(20, 20)


@lru_cache(maxsize=96)
def icon(name: str, emphasis: bool = False) -> QIcon:
    """从同一SVG生成正常/禁用态及高分屏尺寸，主按钮用白色保持对比。"""
    root = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
    source = (root / "assets/icons" / (name + ".svg")).read_text(encoding="utf-8")
    result = QIcon()
    for mode in (QIcon.Mode.Normal, QIcon.Mode.Disabled):
        svg = source
        if mode == QIcon.Mode.Disabled or emphasis:
            color = "#8798a9" if mode == QIcon.Mode.Disabled else "#ffffff"
            for original in ("#126f94", "#159c88", "#c34c57", "#be8b25"):
                svg = svg.replace(original, color)
        renderer = QSvgRenderer(QByteArray(svg.encode("utf-8")))
        if not renderer.isValid():
            raise ValueError(f"图标无效：{name}")
        for size in (16, 20, 24, 32, 40, 48, 64):
            pixmap = QPixmap(size, size)
            pixmap.fill("transparent")
            painter = QPainter(pixmap)
            renderer.render(painter, QRectF(0, 0, size, size))
            painter.end()
            result.addPixmap(pixmap, mode)
    return result
