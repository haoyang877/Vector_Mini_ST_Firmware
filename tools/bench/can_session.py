"""参数与状态共用的常驻 CAN 进程；UI 仅提交任务和接收带编号的结果。"""

import json
import multiprocessing as mp
import os
import queue
import time
from contextlib import ExitStack
from pathlib import Path
from typing import Any

from can_motor_operations import TraceBus, read_device_status
from can_parameters import run_parameters
from ports.controlcanfd import ControlCanFd


class CanConnection:
    """只由设备进程访问；相同总线配置复用句柄，失败或显式释放后关闭。"""

    def __init__(self) -> None:
        self.stack = ExitStack()
        self.port = None
        self.key = None

    def open(self, dll: Path, settings: dict) -> tuple[Any, bool]:
        """返回独占端口及是否复用；配置变化时先关闭旧端口。"""
        key = (str(dll), tuple(sorted(settings.items())))
        if self.port is not None and key == self.key:
            return self.port, True
        self.close()
        try:
            self.stack.enter_context(os.add_dll_directory(str(dll.parent)))
            self.port = self.stack.enter_context(ControlCanFd(dll, **settings))
            self.key = key
            return self.port, False
        except Exception:
            self.close()
            raise

    def close(self) -> None:
        """同步完成 SDK 关闭，调用者才能报告释放成功。"""
        self.port = self.key = None
        self.stack.close()


def run_session(dll: Path, requests: Any, events: Any, cancel: Any) -> None:
    """设备进程串行处理只读/参数事务；不自动重试或发起运动。"""
    parent = mp.parent_process()
    connection = CanConnection()
    try:
        while parent is None or parent.is_alive():
            try:
                number, settings, out = requests.get(timeout=0.2)
            except queue.Empty:
                continue
            mode = settings["profile"]
            started = time.monotonic()
            report = dict(profile=mode, passed=False)
            try:
                if mode in ("can_release", "can_shutdown"):
                    connection.close()
                    report["passed"] = True
                else:
                    port, reused = connection.open(dll, settings["bus"])
                    opened = time.monotonic()
                    with (out / "frames.csv").open("w", newline="", encoding="utf-8") as stream:
                        bus = TraceBus(port, stream)
                        if mode == "parameters":
                            report = run_parameters(
                                bus,
                                settings["node"],
                                settings["expected_uid"],
                                settings["group"],
                                changes=settings["changes"],
                                snapshot=settings["snapshot"],
                                stopped=lambda: cancel.is_set()
                                or (parent is not None and not parent.is_alive()),
                                progress=lambda done, total, task=number: events.put(
                                    (task, "parameter_progress", (done, total))
                                ),
                            )
                        elif mode == "device_status":
                            if cancel.is_set():
                                raise RuntimeError("操作已停止")
                            report.update(
                                device=read_device_status(
                                    bus, settings["node"], settings["expected_uid"]
                                ),
                                passed=True,
                            )
                        else:
                            raise ValueError("常驻会话不支持此任务")
                    report.update(connection_reused=reused, open_elapsed_s=opened - started)
                    if not report.get("passed"):
                        connection.close()
            except Exception as exc:
                report["fatal"] = str(exc)
                connection.close()
            report["total_elapsed_s"] = time.monotonic() - started
            if out is not None:
                (out / "summary.json").write_text(
                    json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
                )
            events.put((number, "done", report))
            if mode == "can_shutdown":
                break
    finally:
        connection.close()


class SessionTask:
    """一个逻辑任务的事件和生命周期，独立于常驻进程的寿命。"""

    def __init__(self, session: "CanSession", number: int) -> None:
        self.session, self.number = session, number
        self.complete = False

    def is_alive(self) -> bool:
        """收到本任务完成消息前保持忙；进程异常退出则由 UI 收敛。"""
        return not self.complete and self.session.process.is_alive()

    def get_nowait(self) -> tuple[str, Any]:
        """过滤旧任务消息，只有匹配编号的完成消息才能结束本任务。"""
        while True:
            number, tag, value = self.session.events.get_nowait()
            if number != self.number:
                continue
            if tag == "done":
                self.complete = True
                self.session.active = False
                if not value.get("passed") or value["profile"] in ("can_release", "can_shutdown"):
                    self.session.connected = False
            return tag, value

    def join(self) -> None:
        """任务完成不退出设备进程。"""

    def close(self) -> None:
        """任务不拥有共享队列；队列随设备进程整体回收。"""


class CanSession:
    """UI 持有的设备进程入口；最多一个在途任务，停止走独立事件。"""

    def __init__(self, dll: Path) -> None:
        context = mp.get_context("spawn")
        self.requests = context.Queue(maxsize=2)
        self.events = context.Queue(maxsize=128)
        self.cancel = context.Event()
        self.process = context.Process(
            target=run_session, args=(dll, self.requests, self.events, self.cancel), daemon=True
        )
        self.number = 0
        self.active = False
        self.connected = False
        self.process.start()

    def submit(self, settings: dict, out: Path | None) -> SessionTask:
        """提交显式操作；失败不在后台重放请求。"""
        if self.active or not self.process.is_alive():
            raise RuntimeError("CAN 会话忙或已退出，请重新连接")
        self.number += 1
        self.cancel.clear()
        self.requests.put_nowait((self.number, settings, out))
        self.active = True
        if settings["profile"] not in ("can_release", "can_shutdown"):
            self.connected = True
        return SessionTask(self, self.number)

    def dispose(self) -> None:
        """仅在进程已退出时回收 IPC 资源。"""
        if self.process.is_alive():
            raise RuntimeError("CAN 进程尚未结束")
        self.process.join()
        self.process.close()
        self.requests.close()
        self.events.close()
