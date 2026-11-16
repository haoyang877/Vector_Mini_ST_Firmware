"""Qt主线程任务调度：复用常驻CAN，其他设备任务运行于独立进程。"""

import multiprocessing as mp
import queue
import time
from pathlib import Path

from can_session import CanSession
from can_upgrade_stress import run_upgrade_test
from five_axis_jlink import run_jlink
from production_backend import acquire
from PySide6.QtCore import QObject, QTimer, Signal


class ProductionController(QObject):
    """一个在途设备任务；释放确认前不交接SDK，关闭前等待关相与队列回收。"""

    event = Signal(str, object)
    completed = Signal(object, object, bool)
    busy_changed = Signal(bool)
    closed = Signal()

    def __init__(self, dll: Path, parent=None) -> None:
        super().__init__(parent)
        self.dll = dll
        self.session = None
        self.job = None
        self.events = None
        self.cancel = None
        self.commands = None
        self.settings = None
        self.directory = None
        self.automatic = False
        self.pending = None
        self.report = None
        self.exit_polls = 0
        self.closing = False
        self.waiting_shutdown = False
        self.timer = QTimer(self)
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.poll)

    @property
    def connected(self) -> bool:
        return bool(self.session and self.session.connected and self.session.process.is_alive())

    @property
    def busy(self) -> bool:
        return self.job is not None

    def start(self, settings: dict, directory: Path | None, *, automatic=False) -> None:
        """显式提交；自动查询被手动动作抢占时先完成当前读操作。"""
        if self.closing:
            raise RuntimeError("正在等待设备安全关闭")
        if self.busy:
            if self.automatic and not automatic:
                self.pending = (settings, directory, False)
                self.cancel.set()
                return
            raise RuntimeError("设备任务正在执行")
        profile = settings["profile"]
        if self.connected and profile not in (
            "parameters",
            "can_scan",
            "device_status",
            "can_release",
            "can_shutdown",
        ):
            self.pending = (settings, directory, automatic)
            self._launch(dict(profile="can_release"), None, False)
            return
        self._launch(settings, directory, automatic)

    def _launch(self, settings: dict, directory: Path | None, automatic: bool) -> None:
        profile = settings["profile"]
        self.commands = None
        if profile in ("parameters", "can_scan", "device_status", "can_release", "can_shutdown"):
            if self.session and not self.session.process.is_alive():
                self.session.dispose()
                self.session = None
            if self.session is None:
                self.session = CanSession(self.dll)
            self.job = self.session.submit(settings, directory)
            self.events = self.job
            self.cancel = self.session.cancel
        else:
            context = mp.get_context("spawn")
            self.events = context.Queue()
            self.cancel = context.Event()
            self.commands = context.Queue(maxsize=8) if profile == "motor_speed" else None
            args = (settings, directory, self.events, self.cancel)
            if self.commands is not None:
                args += (self.commands,)
            target = (
                run_jlink
                if profile.startswith("jlink_")
                else run_upgrade_test
                if profile == "can_upgrade"
                else acquire
            )
            self.job = context.Process(target=target, args=args)
            self.job.start()
        self.settings, self.directory, self.automatic = settings, directory, automatic
        self.report = None
        self.exit_polls = 0
        self.busy_changed.emit(True)
        self.timer.start()

    def stop(self) -> None:
        """烧录不能中断；运动/检测/升级按原工作进程约定结束。"""
        self.pending = None
        if self.busy and not self.settings["profile"].startswith("jlink_"):
            self.cancel.set()

    def update_speed(self, value: float) -> None:
        if not self.busy or self.settings["profile"] != "motor_speed" or self.commands is None:
            raise RuntimeError("速度会话未启动")
        self.commands.put_nowait(value)

    def shutdown(self) -> None:
        """保持Qt事件循环，等待任务退出后再关闭常驻进程。"""
        self.closing = True
        self.pending = None
        if self.waiting_shutdown:
            return
        if self.busy:
            self.stop()
        elif self.session and self.session.process.is_alive():
            self._launch(dict(profile="can_shutdown"), None, False)
        else:
            if self.session:
                self.session.dispose()
                self.session = None
            self.closed.emit()

    def poll(self) -> None:
        """每次最多8ms/64条，避免事件洪峰阻塞绘制；收到报告后仍等进程退出。"""
        if self.job is None:
            return
        deadline = time.monotonic() + 0.008
        drained = False
        for _ in range(64):
            try:
                tag, value = self.events.get_nowait()
            except queue.Empty:
                drained = True
                break
            if tag == "done":
                self.report = value
            else:
                self.event.emit(tag, value)
            if time.monotonic() >= deadline:
                break
        if self.job.is_alive() or not drained:
            return
        if self.report is None:
            self.exit_polls += 1
            if self.exit_polls < 5:
                return
            self.report = dict(
                profile=self.settings["profile"], fatal="工作进程异常退出，未取得完成确认"
            )
        report, directory, automatic = self.report, self.directory, self.automatic
        profile = self.settings["profile"]
        self.job.join()
        self.job.close()
        self.events.close()
        if self.commands is not None:
            self.commands.close()
        self.job = self.events = self.commands = None
        self.timer.stop()
        if profile == "can_shutdown" and self.session:
            if self.session.process.is_alive():
                # 消息入队后进程可能尚未退出；定时继续等待，不阻塞GUI。
                self.waiting_shutdown = True
                self.timer.timeout.disconnect(self.poll)
                self.timer.timeout.connect(self._finish_shutdown)
                self.timer.start()
                return
            self.session.dispose()
            self.session = None
        self.completed.emit(report, directory, automatic)
        self.busy_changed.emit(False)
        if self.closing:
            self.shutdown()
        elif self.pending is not None:
            pending, self.pending = self.pending, None
            if profile != "can_release" or report.get("passed"):
                try:
                    self.start(pending[0], pending[1], automatic=pending[2])
                except (OSError, RuntimeError) as exc:
                    self.completed.emit(
                        dict(profile=pending[0]["profile"], fatal=str(exc)), pending[1], False
                    )

    def _finish_shutdown(self) -> None:
        if self.session.process.is_alive():
            return
        self.timer.stop()
        self.timer.timeout.disconnect(self._finish_shutdown)
        self.timer.timeout.connect(self.poll)
        self.session.dispose()
        self.session = None
        self.waiting_shutdown = False
        self.busy_changed.emit(False)
        self.closed.emit()
