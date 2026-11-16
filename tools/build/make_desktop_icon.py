"""将生产工作台SVG渲染为Windows多尺寸PNG容器ICO。"""

import argparse
import os
import struct
from pathlib import Path


def main() -> None:
    """离屏渲染16～256像素图标，不访问设备。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PySide6.QtCore import QBuffer, QIODevice, QRectF
    from PySide6.QtGui import QGuiApplication, QImage, QPainter
    from PySide6.QtSvg import QSvgRenderer

    app = QGuiApplication.instance() or QGuiApplication([])
    renderer = QSvgRenderer(str(args.source))
    if not renderer.isValid():
        raise ValueError("图标SVG无效")
    sizes = (16, 24, 32, 48, 64, 128, 256)
    entries, images = [], []
    offset = 6 + 16 * len(sizes)
    for size in sizes:
        image = QImage(size, size, QImage.Format.Format_ARGB32)
        image.fill(0)
        painter = QPainter(image)
        renderer.render(painter, QRectF(0, 0, size, size))
        painter.end()
        buffer = QBuffer()
        buffer.open(QIODevice.OpenModeFlag.WriteOnly)
        if not image.save(buffer, "PNG"):
            raise OSError("图标PNG编码失败")
        data = bytes(buffer.data())
        entries.append(
            struct.pack("<4B2H2I", size % 256, size % 256, 0, 0, 1, 32, len(data), offset)
        )
        images.append(data)
        offset += len(data)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(struct.pack("<3H", 0, 1, len(sizes)) + b"".join(entries + images))
    app.quit()


if __name__ == "__main__":
    main()
