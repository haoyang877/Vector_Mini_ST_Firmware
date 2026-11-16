"""Qt生产工作台入口；创建窗口不会访问CAN或J-Link。"""

import multiprocessing as mp
import sys
from pathlib import Path


def main() -> None:
    """先处理Windows子进程入口，再加载Qt；避免工作进程创建窗口。"""
    mp.freeze_support()
    from PySide6.QtWidgets import QApplication
    from qt_production_window import ProductionWindow

    app = QApplication(sys.argv)
    app.setApplicationName("电机生产测试上位机")
    app.setOrganizationName("Vector Mini")
    window = ProductionWindow()
    window.show()
    if len(sys.argv) == 3 and sys.argv[1] == "--offline-smoke":
        # 交付包自检仅渲染控件，不发出设备命令。
        import json

        from PySide6.QtCore import QTimer

        def capture():
            destination = Path(sys.argv[2])
            destination.mkdir(parents=True, exist_ok=True)
            window.refresh_timer.stop()
            for index in range(window.stack.count()):
                window.navigation.setCurrentRow(index)
                app.processEvents()
                window.grab().save(str(destination / f"page-{index}.png"))
            (destination / "summary.json").write_text(
                json.dumps(
                    dict(
                        title=window.windowTitle(),
                        pages=window.stack.count(),
                        hardware_contacted=False,
                        connected=window.controller.connected,
                    )
                ),
                encoding="utf-8",
            )
            app.quit()

        QTimer.singleShot(150, capture)
    raise SystemExit(app.exec())


if __name__ == "__main__":
    main()
