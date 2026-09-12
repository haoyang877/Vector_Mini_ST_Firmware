"""Single worker thread owning the CAN transport and every bus access.

The UI thread never touches the DLL or the transport: it submits named tasks
and consumes results via Qt signals. The protocol has no transaction ID, so
all tasks run serially in this one thread.
"""

from __future__ import annotations

from pathlib import Path
import queue
from threading import Event
import time
from typing import Any

from PySide6.QtCore import QThread, Signal

from mdrive_core.device.session import DeviceSession
from mdrive_core.protocol.param import layout, raw_get
from mdrive_core.protocol.loader_client import UpgradeAborted
from mdrive_core.schema.flash_params import lut_statistics, parse_record
from mdrive_core.services.telemetry import StreamSession
from mdrive_core.schema.params import PARAMS
from mdrive_core.transport.base import TransportError
from mdrive_core.transport.controlcanfd import ControlCanFdConfig, ControlCanFdTransport

# Writes to these IDs can change motor behaviour and need UI-side confirmation.
DANGEROUS_WRITE_IDS = frozenset({0x00, 0x02, 0x04, 0x06})

_TASKS = (
    "open",
    "close",
    "scan",
    "node_status",
    "read_param",
    "read_all",
    "write_param",
    "write_params",
    "write_and_save",
    "save_params",
    "estop",
    "flash_read",
    "stream_start",
    "upgrade",
)


class CanWorker(QThread):
    """Serial task executor; the only place a transport may live."""

    # Container payloads use Signal(object): Signal(dict/list) goes through
    # QVariantMap/QVariantList conversion, which rejects int-keyed dicts.
    connected = Signal(object)  # transport info: serial/hardware/firmware/dll_sha256/...
    disconnected = Signal()
    nodes = Signal(object)  # list of {node, revision, latency_ms, online}
    nodeStatus = Signal(object)  # {node, mode, fault, latency_ms}
    paramRead = Signal(int, object)  # (param_id, value)
    paramsRead = Signal(object)  # {param_id: value | None}
    progress = Signal(str, int, int)  # (task name, done, total)
    failed = Signal(str)  # human-readable failure message
    flashRead = Signal(object)  # {node, info, parsed, lut, blob}
    streamStarted = Signal(object)  # {node, rate_hz, recording}
    streamSamples = Signal(object)  # list of {time_s, <status fields>}
    streamStats = Signal(object)  # {frames, elapsed_s, actual_hz}
    streamStopped = Signal(object)  # final summary
    upgradePhase = Signal(str)  # begin/erase/program/verify/activate/app_alive
    finished = Signal(str, object)  # (task name, result payload)
    logged = Signal(str, str)  # (level, message)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._queue: queue.Queue[tuple[str, dict[str, Any]] | None] = queue.Queue()
        self._transport: ControlCanFdTransport | None = None
        self._stream_session: StreamSession | None = None
        self._upgrade_abort = Event()
        self._running = True

    # ------------------------------------------------------------- lifecycle
    def submit(self, name: str, **kwargs: Any) -> None:
        """Queue a task by name (see _TASKS); safe to call from the UI thread."""
        if name not in _TASKS:
            raise ValueError(f"unknown worker task: {name}")
        self._queue.put((name, kwargs))

    def request_stream_stop(self) -> None:
        """Ask a running stream task to finish (safe from the UI thread)."""
        session = self._stream_session
        if session is not None:
            session.stop()

    def request_upgrade_abort(self) -> None:
        """Ask a running upgrade to stop between chunks (safe from the UI thread)."""
        self._upgrade_abort.set()

    def stop(self) -> None:
        """Ask the thread to exit after the current task, then wait for it."""
        self.request_stream_stop()
        self._running = False
        self._queue.put(None)
        self.wait(5000)

    def run(self) -> None:
        while self._running:
            try:
                item = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if item is None:
                break
            name, kwargs = item
            try:
                self._dispatch(name, kwargs)
            except Exception as exc:  # noqa: BLE001 - the thread must never crash
                message = f"{self._describe(name)}: {exc}"
                self.failed.emit(message)
                self.logged.emit("ERROR", message)
        self._close_transport(silent=True)

    # -------------------------------------------------------------- dispatch
    def _dispatch(self, name: str, kwargs: dict[str, Any]) -> None:
        handler = getattr(self, f"_task_{name}")
        handler(**kwargs)

    @staticmethod
    def _describe(name: str) -> str:
        return {
            "open": "打开设备失败",
            "close": "关闭设备失败",
            "scan": "扫描失败",
            "node_status": "读取节点状态失败",
            "read_param": "读取参数失败",
            "read_all": "读取全部参数失败",
            "write_param": "写入参数失败",
            "write_params": "写入参数失败",
            "write_and_save": "写入并保存失败",
            "save_params": "保存到设备失败",
            "estop": "紧急停止失败",
            "flash_read": "读取 Flash 参数失败",
            "stream_start": "状态流采集失败",
            "upgrade": "固件升级失败",
        }.get(name, f"任务 {name} 失败")

    # --------------------------------------------------------------- helpers
    def _require_transport(self) -> ControlCanFdTransport:
        if self._transport is None:
            raise TransportError("CAN 设备未打开，请先在连接页打开设备")
        return self._transport

    def _session(self, node: int) -> DeviceSession:
        transport = self._require_transport()
        return DeviceSession(transport, node=node, channel=transport.config.channel)

    def _close_transport(self, silent: bool = False) -> None:
        if self._transport is None:
            return
        try:
            self._transport.close()
        except (OSError, TransportError) as exc:
            if not silent:
                self.failed.emit(f"关闭设备时出错: {exc}")
        self._transport = None

    # ----------------------------------------------------------------- tasks
    def _task_open(
        self,
        dll_path: str,
        channel: int = 0,
        nominal_bps: int = 1_000_000,
        data_bps: int = 1_000_000,
        iso: bool = True,
    ) -> None:
        self._close_transport(silent=True)
        config = ControlCanFdConfig(
            dll_path=Path(dll_path), channel=channel, nominal_bps=nominal_bps, data_bps=data_bps, iso=iso
        )
        transport = ControlCanFdTransport(config)
        transport.open()
        self._transport = transport
        info = transport.info
        self.connected.emit(info)
        self.finished.emit("open", info)
        self.logged.emit(
            "INFO", f"设备已打开（{'ISO' if iso else 'non-ISO'} FD，仲裁 {nominal_bps // 1000}k / 数据 {data_bps // 1000}k）"
        )

    def _task_close(self) -> None:
        had_transport = self._transport is not None
        self._close_transport()
        if had_transport:
            self.disconnected.emit()
            self.finished.emit("close", {})
            self.logged.emit("INFO", "设备已关闭")

    def _task_scan(self) -> None:
        transport = self._require_transport()
        channel = transport.config.channel
        found: list[dict[str, Any]] = []
        for node in range(8):
            started = time.perf_counter()
            try:
                revision = raw_get(transport, channel, node, 0x67, "f32", 0.08)
            except TimeoutError:
                self.progress.emit("scan", node + 1, 8)
                continue
            latency_ms = (time.perf_counter() - started) * 1000.0
            found.append({"node": node, "revision": int(revision), "latency_ms": round(latency_ms, 1), "online": True})
            self.logged.emit("INFO", f"节点 {node} 在线：协议 rev{int(revision)}")
            self.progress.emit("scan", node + 1, 8)
        self.nodes.emit(found)
        self.finished.emit("scan", {"count": len(found)})

    def _task_node_status(self, node: int = 0) -> None:
        session = self._session(node)
        started = time.perf_counter()
        mode = session.read_raw(0x01)  # GET_MODE
        fault = session.read_raw(0x4D)  # GET_ERROR
        latency_ms = (time.perf_counter() - started) * 1000.0
        status = {"node": node, "mode": mode, "fault": fault, "latency_ms": round(latency_ms, 1), "online": True}
        self.nodeStatus.emit(status)
        self.finished.emit("node_status", status)

    def _task_read_param(self, param_id: int, node: int = 0) -> None:
        session = self._session(node)
        definition = PARAMS.get(param_id)
        if definition is None:
            raise ValueError(f"未知参数 ID: {param_id:#x}")
        if definition.access == "rw":
            value = session.read(param_id)
        elif definition.access == "ro":
            value = session.read_raw(param_id)
        else:
            raise ValueError(f"{definition.name} 是命令项，不可读取")
        self.paramRead.emit(param_id, value)
        self.finished.emit("read_param", {"id": param_id, "value": value})

    def _task_read_all(self, node: int = 0) -> None:
        session = self._session(node)
        values = session.read_all(progress=lambda done, total: self.progress.emit("read_all", done, total))
        self.paramsRead.emit(values)
        ok = sum(1 for value in values.values() if value is not None)
        self.finished.emit("read_all", {"ok": ok, "total": len(values)})
        self.logged.emit("INFO", f"读取全部参数完成：{ok}/{len(values)} 项有响应")

    def _write_one(self, session: DeviceSession, param_id: int, value: float) -> float:
        """Write one parameter and verify by read-back (fixed-point tolerance)."""
        definition = PARAMS.get(param_id)
        if definition is None or definition.access != "rw":
            raise ValueError(f"参数 {param_id:#x} 不可写")
        session.write(param_id, value)
        actual = session.read(param_id)
        wire_format, scale = layout(definition.get_id or param_id + 1, reply=True)
        tolerance = 1e-5 if wire_format == ">f" else 1 / scale + 1e-6
        if abs(actual - value) > tolerance:
            raise ValueError(f"{definition.name} 写后校验失败：期望 {value:g}，回读 {actual:g}")
        return actual

    def _task_write_param(self, param_id: int, value: float, node: int = 0) -> None:
        self._task_write_params(items=[[param_id, value]], node=node)

    def _task_write_params(self, items: list, node: int = 0) -> None:
        session = self._session(node)
        results: dict[int, float] = {}
        for index, item in enumerate(items, 1):
            param_id, value = int(item[0]), float(item[1])
            results[param_id] = self._write_one(session, param_id, value)
            self.paramRead.emit(param_id, results[param_id])
            self.progress.emit("write_params", index, len(items))
            self.logged.emit("INFO", f"已写入 {PARAMS[param_id].name} = {value:g}（回读 {results[param_id]:g}）")
        self.finished.emit("write_params", {"results": results})

    def _task_save_params(self, node: int = 0) -> None:
        session = self._session(node)
        try:
            session.save_params()
        except TimeoutError as exc:
            raise RuntimeError(f"固件未就绪：当前固件未实现 0x68 SAVE_PARAM，无法保存到 Flash（{exc}）") from exc
        self.finished.emit("save_params", {"ok": True})
        self.logged.emit("INFO", "参数已保存到设备 Flash")

    def _task_write_and_save(self, items: list, node: int = 0) -> None:
        if items:
            self._task_write_params(items=items, node=node)
        self._task_save_params(node=node)

    def _task_flash_read(self, node: int = 0) -> None:
        """Read, CRC-verify, and parse the whole flash parameter record."""
        session = self._session(node)
        info = session.flash_info()
        blob = session.flash_read(progress=lambda done, total: self.progress.emit("flash_read", done, total))
        parsed = parse_record(blob)
        stats = lut_statistics(blob)
        payload = {
            "node": node,
            "info": info.to_dict(),
            "parsed": parsed,
            "lut": stats,
            "blob": blob,
        }
        self.flashRead.emit(payload)
        self.finished.emit("flash_read", {"size": len(blob), "schema": info.schema, "crc32": info.crc32})
        self.logged.emit(
            "INFO",
            f"Flash 参数读取完成：{len(blob)}B（schema v{info.schema}，CRC32 校验通过）",
        )

    def _task_stream_start(self, node: int = 0, rate_hz: int = 100, record_path: str | None = None) -> None:
        """Sample the status stream until stopped; optionally record to CSV."""
        transport = self._require_transport()
        session = StreamSession(
            transport,
            channel=transport.config.channel,
            node=node,
            rate_hz=float(rate_hz),
            record_path=Path(record_path) if record_path else None,
            on_sample=lambda sample: self.streamSamples.emit([{"time_s": sample.time_s, **sample.values}]),
            on_stats=lambda stats: self.streamStats.emit(stats),
        )
        self._stream_session = session
        self.streamStarted.emit({"node": node, "rate_hz": rate_hz, "recording": record_path is not None})
        self.logged.emit("INFO", f"状态流已启动：节点 {node}，{rate_hz} Hz" + ("（录制中）" if record_path else ""))
        try:
            summary = session.run()
        finally:
            self._stream_session = None
        self.finished.emit("stream_start", summary)
        self.streamStopped.emit(summary)
        self.logged.emit("INFO", f"状态流停止：{summary['frames']} 帧，实际 {summary['actual_hz']:.1f} Hz")

    def _task_upgrade(self, image_path: str, version: int, node: int = 0) -> None:
        """Run the full Loader upgrade flow with byte progress, phases, and abort."""
        session = self._session(node)
        self._upgrade_abort.clear()
        try:
            summary = session.upgrade(
                Path(image_path),
                version,
                progress=lambda done, total: self.progress.emit("upgrade", done, total),
                on_phase=lambda phase: self.upgradePhase.emit(phase),
                abort_event=self._upgrade_abort,
            )
        except UpgradeAborted:
            summary = {"ok": False, "status": "aborted"}
            self.finished.emit("upgrade", summary)
            self.logged.emit("WARN", "升级已被中止（Loader 会话已复位，设备停留在恢复模式）")
            return
        self.finished.emit("upgrade", summary)
        self.logged.emit("INFO", f"固件升级完成：版本 {version}，{summary.get('image_size', 0)} B")

    def _task_estop(self, node: int = 0) -> None:
        session = self._session(node)
        session.write(0x00, 0.0)  # SET_MODE = 0 (Motor_Disable)
        session.write(0x02, 0.0)  # SET_CURRENT = 0
        self.finished.emit("estop", {"node": node})
        self.logged.emit("WARN", f"节点 {node} 紧急停止：SET_MODE=0，SET_CURRENT=0")
