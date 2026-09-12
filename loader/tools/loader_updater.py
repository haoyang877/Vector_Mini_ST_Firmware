"""Update one motor-drive Loader image over a Chuangxin USBCANFD adapter."""
# noqa: SIZE_OK - frozen deliverable combines the required adapter, client, flow, and CLI.

import argparse
from collections.abc import Callable
import ctypes as C
import json
from pathlib import Path
import struct
import time
from typing import Protocol, TypedDict

from loader_proto import (
    APP_BASE_ADDRESS,
    APP_REQUEST_ENTER_BOOT,
    APP_REQUEST_GET_MODE,
    FLASH_PAGE_SIZE,
    MAX_PROGRAM_CHUNK,
    OP_ABORT,
    OP_ACTIVATE,
    OP_BEGIN_UPDATE,
    OP_ERASE,
    OP_GET_INFO,
    OP_PROGRAM,
    OP_VERIFY,
    PROGRAM_ALIGNMENT,
    REQUEST_ID_BASE,
    RESPONSE_ID_BASE,
    RESULT_NAMES,
    RESULT_OK,
    Reply,
    crc32,
    frame_bytes,
    parse_reply,
)
import sys as _sys
from pathlib import Path as _Path

_REPO_TOOLS = _Path(__file__).resolve().parents[2] / "tools"
if str(_REPO_TOOLS) not in _sys.path:
    _sys.path.insert(0, str(_REPO_TOOLS))

from usbcan_adapter import CanFD, RxCAN, RxFD, TxFD

DEFAULT_DLL = Path("D:/Work/Code/motor_ctrl_app/.local/canfd-sdk/ControlCANFD.dll")
Trace = Callable[[str, int, bytes, int], None]
Progress = Callable[[int, int], None]
ReceivedFrame = tuple[int, bytes, int]


class Bus(Protocol):
    def send(self, channel: int, identifier: int, data: bytes) -> None: ...
    def receive(self, channel: int, timeout: float) -> list[ReceivedFrame]: ...


class Info(TypedDict):
    loader_version: int
    app_base: int
    app_capacity: int
    node: int


class BeginReply(TypedDict):
    session: int
    app_base: int


class UpdateResult(TypedDict):
    ok: bool
    status: str
    image_size: int
    image_crc: int
    session: int
    timings_ms: dict[str, float]


class LoaderResultError(RuntimeError):
    """The Loader returned a nonzero result code."""

    def __init__(self, opcode: int, result: int) -> None:
        self.opcode = opcode
        self.result = result
        super().__init__(f"Loader opcode {opcode:#04x}: {RESULT_NAMES.get(result, f'UNKNOWN_{result}')}")


class LoaderValueError(ValueError):
    """A Loader API value violates its wire or update-flow contract."""


class LoaderBus(CanFD):
    """Add CAN-FD transmit and combined FD/classic receive to ``CanFD``."""

    def send(self, channel: int, identifier: int, data: bytes) -> None:
        if channel not in self.channels:
            raise LoaderValueError(f"channel is not open: {channel}")
        if not 0 <= identifier <= 0x7FF:
            raise LoaderValueError(f"not a standard 11-bit CAN ID: {identifier:#x}")
        if len(data) > 64:
            raise LoaderValueError(f"CAN-FD payload too long: {len(data)}")
        message = TxFD()
        message.frame.can_id = identifier
        message.frame.length = len(data)
        message.frame.flags = 1
        message.frame.data[:len(data)] = data
        self._ok(self.dll.ZCAN_TransmitFD(self.channels[channel], C.byref(message), 1), "transmit FD")

    def receive(self, channel: int, timeout: float) -> list[ReceivedFrame]:
        if channel not in self.channels:
            raise LoaderValueError(f"channel is not open: {channel}")
        if timeout < 0:
            raise LoaderValueError(f"negative receive timeout: {timeout}")
        frames: list[ReceivedFrame] = []
        calls = ((RxFD, "ZCAN_ReceiveFD", 64, max(0, round(timeout * 1000))),
                 (RxCAN, "ZCAN_Receive", 8, 0))
        for frame_type, name, limit, wait_ms in calls:
            messages = (frame_type * 64)()
            count = getattr(self.dll, name)(self.channels[channel], messages, 64, wait_ms)
            if count > 64:
                raise OSError(f"{name} failed: SDK returned {count}")
            for message in messages[:count]:
                length = message.frame.length
                if length > limit:
                    raise OSError(f"{name} returned invalid length {length}")
                flags = message.frame.flags if frame_type is RxFD else 0
                frames.append((message.frame.can_id, bytes(message.frame.data[:length]), flags))
        return frames


class LoaderClient:
    """Serialize Loader requests and retain communication errors for diagnostics."""

    def __init__(self, bus: Bus, node: int = 0, timeout: float = 1.0, trace: Trace | None = None) -> None:
        if not 0 <= node <= 15:
            raise LoaderValueError(f"node outside Loader ID range: {node}")
        if timeout <= 0:
            raise LoaderValueError(f"timeout must be positive: {timeout}")
        self.bus = bus
        self.node = node
        self.timeout = timeout
        self.trace = trace
        self.session = 0
        self.errors: list[str] = []
        self._seq = 0

    def _record(self, detail: str) -> None:
        self.errors.append(detail)

    def _request(self, opcode: int, payload: bytes = b"", timeout: float | None = None,
                 retries: int = 5) -> Reply:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        request_session = self.session
        raw = frame_bytes(opcode, seq, request_session, payload)
        try:
            self.bus.receive(0, 0.0)
        except (OSError, ValueError) as exc:
            self._record(f"opcode={opcode:#x} pre-drain: {exc!r}")
        request_timeout = self.timeout if timeout is None else timeout
        for attempt in range(retries + 1):
            try:
                self.bus.send(0, REQUEST_ID_BASE + self.node, raw)
                if self.trace:
                    self.trace("tx", REQUEST_ID_BASE + self.node, raw, 1)
            except (OSError, ValueError) as exc:
                self._record(f"opcode={opcode:#x} attempt={attempt + 1} send: {exc!r}")
                continue
            deadline = time.perf_counter() + request_timeout
            while time.perf_counter() < deadline:
                try:
                    frames = self.bus.receive(0, min(0.05, max(0.0, deadline - time.perf_counter())))
                except (OSError, ValueError) as exc:
                    self._record(f"opcode={opcode:#x} attempt={attempt + 1} receive: {exc!r}")
                    break
                for identifier, data, flags in frames:
                    if self.trace:
                        self.trace("rx", identifier, data, flags)
                    if identifier & 0x20000000:
                        self._record(f"opcode={opcode:#x} CAN error frame id={identifier:#x}")
                        continue
                    if identifier != RESPONSE_ID_BASE + self.node:
                        continue
                    if not flags & 1:
                        self._record(f"opcode={opcode:#x} reply was not CAN-FD with BRS")
                        continue
                    try:
                        reply = parse_reply(data)
                    except ValueError as exc:
                        self._record(f"opcode={opcode:#x} malformed reply: {exc!r}")
                        continue
                    if (reply["opcode"], reply["seq"], reply["session"]) != (opcode, seq, request_session):
                        self._record(f"opcode={opcode:#x} mismatched reply")
                        continue
                    if reply["result"] != RESULT_OK:
                        raise LoaderResultError(opcode, reply["result"])
                    return reply
            self._record(f"opcode={opcode:#x} attempt={attempt + 1} timeout")
        raise TimeoutError(f"Loader opcode {opcode:#04x} timed out after {retries + 1} attempts")

    @staticmethod
    def _payload(reply: Reply, length: int) -> bytes:
        payload = reply["payload"]
        if len(payload) != length:
            raise LoaderValueError(f"opcode {reply['opcode']:#x} reply length {len(payload)}, expected {length}")
        return payload

    def get_info(self) -> Info:
        loader_version, app_base, capacity, node = struct.unpack(">IIIB3x", self._payload(self._request(OP_GET_INFO), 18)[2:])
        return {"loader_version": loader_version, "app_base": app_base, "app_capacity": capacity, "node": node}

    def begin(self, size: int, crc: int, version: int, img_type: int = 1) -> BeginReply:
        payload = struct.pack(">IIIH2x", size, crc, version, img_type)
        session, app_base = struct.unpack(">II", self._payload(self._request(OP_BEGIN_UPDATE, payload), 10)[2:])
        self.session = session
        return {"session": session, "app_base": app_base}

    def erase(self) -> None:
        self._payload(self._request(OP_ERASE, timeout=20.0), 2)

    def program(self, offset: int, data: bytes) -> int:
        if (offset % PROGRAM_ALIGNMENT or not data or len(data) > MAX_PROGRAM_CHUNK
                or len(data) % PROGRAM_ALIGNMENT):
            raise LoaderValueError("PROGRAM requires aligned offset and 8..40 aligned data bytes")
        reply = self._request(OP_PROGRAM, struct.pack(">I", offset) + data)
        return struct.unpack(">I", self._payload(reply, 6)[2:])[0]

    def verify(self) -> int:
        reply = self._request(OP_VERIFY, timeout=10.0)
        return struct.unpack(">I", self._payload(reply, 6)[2:])[0]

    def activate(self) -> None:
        self._payload(self._request(OP_ACTIVATE), 2)
        self.session = 0

    def abort(self) -> None:
        self._payload(self._request(OP_ABORT), 2)
        self.session = 0

    def enter_boot(self) -> None:
        identifier = (self.node << 8) | APP_REQUEST_ENTER_BOOT
        data = bytes(4)
        self.bus.send(0, identifier, data)
        if self.trace:
            self.trace("tx", identifier, data, 1)

    def wait_loader_alive(self, timeout: float = 5.0) -> Info:
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            poll_started = time.perf_counter()
            try:
                reply = self._request(OP_GET_INFO, timeout=min(0.05, max(0.001, deadline - time.perf_counter())), retries=0)
                loader_version, app_base, capacity, node = struct.unpack(">IIIB3x", self._payload(reply, 18)[2:])
                return {"loader_version": loader_version, "app_base": app_base, "app_capacity": capacity, "node": node}
            except (TimeoutError, OSError, ValueError) as exc:
                self._record(f"wait_loader_alive: {exc!r}")
            delay = min(0.05 - (time.perf_counter() - poll_started), deadline - time.perf_counter())
            if delay > 0:
                time.sleep(delay)
        raise TimeoutError(f"Loader node {self.node} not alive after {timeout:.3f}s")

    def wait_app_alive(self, timeout: float = 5.0) -> bytes:
        deadline = time.perf_counter() + timeout
        identifier = (self.node << 8) | APP_REQUEST_GET_MODE
        while time.perf_counter() < deadline:
            poll_started = time.perf_counter()
            try:
                data = bytes(4)
                self.bus.send(0, identifier, data)
                if self.trace:
                    self.trace("tx", identifier, data, 1)
                for reply_id, data, flags in self.bus.receive(0, min(0.05, max(0.0, deadline - time.perf_counter()))):
                    if self.trace:
                        self.trace("rx", reply_id, data, flags)
                    if reply_id == identifier and len(data) == 4:
                        return data
            except (OSError, ValueError) as exc:
                self._record(f"wait_app_alive: {exc!r}")
            delay = min(0.05 - (time.perf_counter() - poll_started), deadline - time.perf_counter())
            if delay > 0:
                time.sleep(delay)
        raise TimeoutError(f"APP node {self.node} not alive after {timeout:.3f}s")


def update_firmware(client: LoaderClient, image_bytes: bytes, version: int,
                    progress: Progress | None = None) -> UpdateResult:
    """Run BEGIN, ERASE, PROGRAM, VERIFY, ACTIVATE, and APP-alive phases."""
    if not image_bytes:
        raise LoaderValueError("firmware image is empty")
    image_crc = crc32(image_bytes)
    timings: dict[str, float] = {}

    total_started = time.perf_counter()
    started = total_started
    begin = client.begin(len(image_bytes), image_crc, version)
    timings["begin"] = (time.perf_counter() - started) * 1000
    if begin["app_base"] != APP_BASE_ADDRESS:
        raise LoaderValueError(f"Loader APP base {begin['app_base']:#x}, expected {APP_BASE_ADDRESS:#x}")

    started = time.perf_counter()
    client.erase()
    timings["erase"] = (time.perf_counter() - started) * 1000
    program_started = time.perf_counter()
    page_size = FLASH_PAGE_SIZE
    offset = 0
    while offset < len(image_bytes):
        to_page_end = page_size - (offset % page_size)
        step = min(MAX_PROGRAM_CHUNK, to_page_end)
        chunk = image_bytes[offset:offset + step]
        aligned = (len(chunk) + PROGRAM_ALIGNMENT - 1) // PROGRAM_ALIGNMENT * PROGRAM_ALIGNMENT
        padded = chunk.ljust(aligned, b"\xFF")
        next_offset = client.program(offset, padded)
        if next_offset != offset + len(padded):
            raise LoaderValueError(f"Loader next offset {next_offset}, expected {offset + len(padded)}")
        if progress:
            progress(min(offset + len(chunk), len(image_bytes)), len(image_bytes))
        offset += len(padded)
    timings["program"] = (time.perf_counter() - program_started) * 1000

    started = time.perf_counter()
    computed_crc = client.verify()
    timings["verify"] = (time.perf_counter() - started) * 1000
    if computed_crc != image_crc:
        raise LoaderValueError(f"Loader CRC {computed_crc:#010x}, expected {image_crc:#010x}")
    started = time.perf_counter()
    client.activate()
    timings["activate"] = (time.perf_counter() - started) * 1000
    started = time.perf_counter()
    client.wait_app_alive()
    timings["app_alive"] = (time.perf_counter() - started) * 1000
    timings["total"] = (time.perf_counter() - total_started) * 1000
    return {"ok": True, "status": "activated", "image_size": len(image_bytes), "image_crc": image_crc,
            "session": begin["session"], "timings_ms": timings}


def main() -> int:  # noqa: BROAD_EXCEPT_OK
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, required=True)
    parser.add_argument("--node", type=int, default=0)
    parser.add_argument("--version", type=lambda value: int(value, 0), required=True)
    parser.add_argument("--dll", type=Path, default=DEFAULT_DLL)
    parser.add_argument("--logdir", type=Path)
    args = parser.parse_args()
    trace_rows: list[dict[str, str | int | float]] = []
    trace = lambda direction, identifier, data, flags: trace_rows.append(
        {"direction": direction, "id": f"0x{identifier:X}", "data": data.hex(), "flags": flags, "time": time.time()})
    bus: LoaderBus | None = None
    client: LoaderClient | None = None
    summary: dict[str, bool | str | int | dict[str, float]]
    try:
        image = args.image.read_bytes()
        bus = LoaderBus(args.dll)
        client = LoaderClient(bus, node=args.node, trace=trace)
        client.enter_boot()
        client.wait_loader_alive()
        summary = update_firmware(client, image, args.version)
        summary["communication_errors"] = client.errors
    except Exception as exc:  # noqa: BROAD_EXCEPT_OK
        summary = {"ok": False, "status": "failed", "error": repr(exc)}
        if client is not None:
            summary["communication_errors"] = client.errors
    finally:
        if bus is not None:
            try:
                bus.close()
            except OSError as exc:
                summary = {**summary, "ok": False, "close_error": repr(exc)}
    if args.logdir:
        args.logdir.mkdir(parents=True, exist_ok=True)
        (args.logdir / "trace.json").write_text(json.dumps(trace_rows, indent=2), encoding="utf-8")
        (args.logdir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
