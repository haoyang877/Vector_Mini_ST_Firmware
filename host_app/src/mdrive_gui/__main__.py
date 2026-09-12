"""GUI entry point: ``python -m mdrive_gui [--selftest|--selftest-live]``."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import os
from pathlib import Path
import sys

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true", help="build the full window offscreen and exit")
    parser.add_argument("--selftest-live", action="store_true", help="open the real CAN device, scan, print nodes, exit")
    parser.add_argument("--screenshot", type=Path, default=None, help="save a window screenshot (with --selftest)")
    parser.add_argument("--dll", type=str, default=None, help="override ControlCANFD.dll path for --selftest-live")
    return parser


def _run_gui() -> int:
    from PySide6.QtWidgets import QApplication

    from mdrive_gui.app_state import AppState
    from mdrive_gui.main_window import MainWindow
    from mdrive_gui import theme
    from mdrive_gui.worker import CanWorker

    app = QApplication(sys.argv)
    theme.apply_theme(app)
    state = AppState()
    worker = CanWorker()
    window = MainWindow(state, worker)
    worker.start()
    window.show()
    try:
        return app.exec()
    finally:
        worker.stop()


def _run_selftest(screenshot: Path | None) -> int:
    """Construct the whole window offscreen, iterate tabs, optionally screenshot."""
    from PySide6.QtWidgets import QApplication

    from mdrive_gui.app_state import AppState
    from mdrive_gui.main_window import MainWindow
    from mdrive_gui import theme
    from mdrive_gui.worker import CanWorker

    app = QApplication(sys.argv)
    theme.apply_theme(app)
    state = AppState()
    worker = CanWorker()  # not started: selftest performs no CAN I/O
    window = MainWindow(state, worker)
    window.resize(1360, 840)
    window.show()
    for index in range(window.tabs.count()):
        window.tabs.setCurrentIndex(index)
        app.processEvents()
    # The params tab is the densest view (full schema table) — grab that one.
    window.tabs.setCurrentIndex(2)
    app.processEvents()
    if screenshot is not None:
        screenshot.parent.mkdir(parents=True, exist_ok=True)
        pixmap = window.grab()
        # PNG quality 100 = minimal compression: keeps the artifact inspectable.
        if not pixmap.save(str(screenshot), "PNG", 100):
            print(f"SELFTEST FAILED: cannot save screenshot to {screenshot}", file=sys.stderr)
            return 1
        size = screenshot.stat().st_size
        print(f"screenshot: {screenshot} ({size} bytes)")
        if size < 30 * 1024:
            print("SELFTEST FAILED: screenshot smaller than 30 KB", file=sys.stderr)
            return 1
    window.close()
    print("SELFTEST OK")
    return 0


def _run_selftest_live(dll: str | None) -> int:
    """Open the real adapter through the worker, scan nodes 0-7, print, exit."""
    from PySide6.QtCore import QEventLoop, QTimer
    from PySide6.QtWidgets import QApplication

    from mdrive_gui.worker import CanWorker

    app = QApplication(sys.argv)
    worker = CanWorker()
    outcome: dict[str, object] = {"done": False, "ok": False, "nodes": []}
    loop = QEventLoop()

    def on_connected(info: dict) -> None:
        print(f"device open: serial={info.get('serial')} hardware={info.get('hardware')} fw={info.get('firmware_version')}")
        worker.submit("scan")

    def on_nodes(nodes: list) -> None:
        outcome["nodes"] = nodes

    def on_finished(task: str, _payload: dict) -> None:
        if task == "scan":
            outcome["done"] = True
            outcome["ok"] = True
            loop.quit()

    def on_failed(message: str) -> None:
        print(f"error: {message}", file=sys.stderr)
        outcome["done"] = True
        outcome["ok"] = False
        loop.quit()

    worker.connected.connect(on_connected)
    worker.nodes.connect(on_nodes)
    worker.finished.connect(on_finished)
    worker.failed.connect(on_failed)

    timeout = QTimer()
    timeout.setSingleShot(True)
    timeout.setInterval(20000)
    timeout.timeout.connect(loop.quit)

    worker.start()
    if dll:
        worker.submit("open", dll_path=dll)
    else:
        from mdrive_core.transport.controlcanfd import ControlCanFdConfig

        worker.submit("open", dll_path=str(ControlCanFdConfig().dll_path))
    timeout.start()
    loop.exec()
    nodes = outcome["nodes"] if isinstance(outcome["nodes"], list) else []
    print("NODE  PROTOCOL_REVISION")
    for entry in nodes:
        print(f"{entry['node']:>4}  {entry['revision']:>17}")
    worker.stop()
    if not outcome["done"] or not outcome["ok"]:
        print("SELFTEST-LIVE FAILED: scan did not complete", file=sys.stderr)
        return 1
    if not any(entry.get("node") == 0 and entry.get("revision") == 2 for entry in nodes):
        print("SELFTEST-LIVE FAILED: node 0 with revision 2 not found", file=sys.stderr)
        return 1
    print("SELFTEST-LIVE OK")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.selftest_live:
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
        return _run_selftest_live(args.dll)
    if args.selftest:
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
        return _run_selftest(args.screenshot)
    return _run_gui()


if __name__ == "__main__":
    raise SystemExit(main())
