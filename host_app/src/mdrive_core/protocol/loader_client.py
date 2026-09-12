"""Stop-and-wait Loader client and complete firmware update flow."""
# noqa: SIZE_OK - the requested frozen migration keeps LoaderClient and update_firmware together.

from collections.abc import Callable
import struct
import time
from typing import Protocol, TypedDict

from mdrive_core.transport.frame import CanFrame

from .loader import (
    APP_BASE_ADDRESS,
    APP_REQUEST_ENTER_BOOT,
    APP_REQUEST_GET_MODE,
    OP_ABORT,
    OP_ACTIVATE,
    OP_BEGIN_UPDATE,
    OP_ERASE,
    OP_GET_INFO,
    OP_GET_STATUS,
    OP_PROGRAM,
    OP_VERIFY,
    REQUEST_ID_BASE,
    RESPONSE_ID_BASE,
    RESULT_NAMES,
    RESULT_OK,
    Reply,
    crc32,
    frame_bytes,
    parse_reply,
)

Trace = Callable[[str, int, bytes, int], None]
Progress = Callable[[int, int], None]
PhaseCallback = Callable[[str], None]


class LoaderBus(Protocol):
    def send(self, channel: int, identifier: int, data: bytes) -> None: ...

    def receive(self, channel: int, timeout: float) -> list[CanFrame]: ...


class Info(TypedDict):
    loader_version: int
    app_base: int
    app_capacity: int
    node: int


class LoaderStatus(TypedDict):
    state: int
    last_error: int
    next_offset: int
    image_size: int
    image_crc: int
    session: int


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


class UpgradeAborted(RuntimeError):
    """The host aborted the upgrade; the Loader session was reset."""


class LoaderValueError(ValueError):
    """A Loader API value violates its wire or update-flow contract."""


class LoaderClient:
    """Serialize Loader requests and retain communication errors for diagnostics."""

    def __init__(
        self,
        bus: LoaderBus,
        node: int = 0,
        timeout: float = 1.0,
        trace: Trace | None = None,
        channel: int = 0,
    ) -> None:
        if not 0 <= node <= 15:
            raise LoaderValueError(f"node outside Loader ID range: {node}")
        if timeout <= 0:
            raise LoaderValueError(f"timeout must be positive: {timeout}")
        self.bus = bus
        self.node = node
        self.timeout = timeout
        self.trace = trace
        self.channel = channel
        self.session = 0
        self.errors: list[str] = []
        self._seq = 0

    def _record(self, detail: str) -> None:
        self.errors.append(detail)

    def _request(self, opcode: int, payload: bytes = b"", timeout: float | None = None, retries: int = 5) -> Reply:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFF
        request_session = self.session
        raw = frame_bytes(opcode, seq, request_session, payload)
        try:
            self.bus.receive(self.channel, 0.0)
        except (OSError, ValueError) as exc:
            self._record(f"opcode={opcode:#x} pre-drain: {exc!r}")
        request_timeout = self.timeout if timeout is None else timeout
        for attempt in range(retries + 1):
            try:
                self.bus.send(self.channel, REQUEST_ID_BASE + self.node, raw)
                if self.trace is not None:
                    self.trace("tx", REQUEST_ID_BASE + self.node, raw, 1)
            except (OSError, ValueError) as exc:
                self._record(f"opcode={opcode:#x} attempt={attempt + 1} send: {exc!r}")
                continue
            deadline = time.perf_counter() + request_timeout
            while time.perf_counter() < deadline:
                try:
                    frames = self.bus.receive(self.channel, min(0.05, max(0.0, deadline - time.perf_counter())))
                except (OSError, ValueError) as exc:
                    self._record(f"opcode={opcode:#x} attempt={attempt + 1} receive: {exc!r}")
                    break
                for frame in frames:
                    if self.trace is not None:
                        self.trace("rx", frame.identifier, frame.data, frame.flags)
                    if frame.identifier & 0x20000000:
                        self._record(f"opcode={opcode:#x} CAN error frame id={frame.identifier:#x}")
                        continue
                    if frame.identifier != RESPONSE_ID_BASE + self.node:
                        continue
                    if not frame.flags & 1:
                        self._record(f"opcode={opcode:#x} reply was not CAN-FD with BRS")
                        continue
                    try:
                        reply = parse_reply(frame.data)
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
        payload = self._payload(self._request(OP_GET_INFO), 18)
        loader_version, app_base, capacity, node = struct.unpack(">IIIB3x", payload[2:])
        return {"loader_version": loader_version, "app_base": app_base, "app_capacity": capacity, "node": node}

    def get_status(self) -> LoaderStatus:
        payload = self._payload(self._request(OP_GET_STATUS), 22)
        state, last_error, next_offset, image_size, image_crc, session = struct.unpack(">BB2xIIII", payload[2:])
        return {
            "state": state,
            "last_error": last_error,
            "next_offset": next_offset,
            "image_size": image_size,
            "image_crc": image_crc,
            "session": session,
        }

    def begin(self, size: int, crc: int, version: int, img_type: int = 1) -> BeginReply:
        payload = struct.pack(">IIIH2x", size, crc, version, img_type)
        session, app_base = struct.unpack(">II", self._payload(self._request(OP_BEGIN_UPDATE, payload), 10)[2:])
        self.session = session
        return {"session": session, "app_base": app_base}

    def erase(self) -> None:
        self._payload(self._request(OP_ERASE, timeout=20.0), 2)

    def program(self, offset: int, data: bytes) -> int:
        if offset % 8 or not data or len(data) > 40 or len(data) % 8:
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
        self.bus.send(self.channel, identifier, data)
        if self.trace is not None:
            self.trace("tx", identifier, data, 1)

    def wait_loader_alive(self, timeout: float = 5.0) -> Info:
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            poll_started = time.perf_counter()
            try:
                request_timeout = min(0.05, max(0.001, deadline - time.perf_counter()))
                reply = self._request(OP_GET_INFO, timeout=request_timeout, retries=0)
                payload = self._payload(reply, 18)
                loader_version, app_base, capacity, node = struct.unpack(">IIIB3x", payload[2:])
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
                self.bus.send(self.channel, identifier, data)
                if self.trace is not None:
                    self.trace("tx", identifier, data, 1)
                wait = min(0.05, max(0.0, deadline - time.perf_counter()))
                for frame in self.bus.receive(self.channel, wait):
                    if self.trace is not None:
                        self.trace("rx", frame.identifier, frame.data, frame.flags)
                    if frame.identifier == identifier and len(frame.data) == 4:
                        return frame.data
            except (OSError, ValueError) as exc:
                self._record(f"wait_app_alive: {exc!r}")
            delay = min(0.05 - (time.perf_counter() - poll_started), deadline - time.perf_counter())
            if delay > 0:
                time.sleep(delay)
        raise TimeoutError(f"APP node {self.node} not alive after {timeout:.3f}s")


def update_firmware(
    client: LoaderClient,
    image: bytes,
    version: int,
    progress: Progress | None = None,
    on_phase: PhaseCallback | None = None,
    abort_event=None,
) -> UpdateResult:
    """Run BEGIN, ERASE, PROGRAM, VERIFY, ACTIVATE, and APP-alive phases.

    ``on_phase`` receives each phase name at its start: begin / erase / program /
    verify / activate / app_alive. ``abort_event`` is checked at phase
    boundaries and between program chunks; when set, the Loader session is
    reset (ABORT) and :class:`UpgradeAborted` is raised.
    """
    if not image:
        raise LoaderValueError("firmware image is empty")
    image_crc = crc32(image)
    timings: dict[str, float] = {}
    total_started = time.perf_counter()

    def check_abort() -> None:
        if abort_event is not None and abort_event.is_set():
            try:
                client.abort()
            finally:
                raise UpgradeAborted("upgrade aborted by user request")

    def phase(name: str) -> None:
        if on_phase is not None:
            on_phase(name)

    check_abort()
    phase("begin")
    started = time.perf_counter()
    begin = client.begin(len(image), image_crc, version)
    timings["begin"] = (time.perf_counter() - started) * 1000
    if begin["app_base"] != APP_BASE_ADDRESS:
        raise LoaderValueError(f"Loader APP base {begin['app_base']:#x}, expected {APP_BASE_ADDRESS:#x}")
    check_abort()
    phase("erase")
    started = time.perf_counter()
    client.erase()
    timings["erase"] = (time.perf_counter() - started) * 1000
    phase("program")
    program_started = time.perf_counter()
    page_size = 2048
    offset = 0
    while offset < len(image):
        check_abort()
        to_page_end = page_size - (offset % page_size)
        step = min(40, to_page_end)
        chunk = image[offset : offset + step]
        padded = chunk.ljust((len(chunk) + 7) // 8 * 8, b"\xFF")
        next_offset = client.program(offset, padded)
        if next_offset != offset + len(padded):
            raise LoaderValueError(f"Loader next offset {next_offset}, expected {offset + len(padded)}")
        if progress is not None:
            progress(min(offset + len(chunk), len(image)), len(image))
        offset += len(padded)
    timings["program"] = (time.perf_counter() - program_started) * 1000
    check_abort()
    phase("verify")
    started = time.perf_counter()
    computed_crc = client.verify()
    timings["verify"] = (time.perf_counter() - started) * 1000
    if computed_crc != image_crc:
        raise LoaderValueError(f"Loader CRC {computed_crc:#010x}, expected {image_crc:#010x}")
    check_abort()
    phase("activate")
    started = time.perf_counter()
    client.activate()
    timings["activate"] = (time.perf_counter() - started) * 1000
    phase("app_alive")
    started = time.perf_counter()
    client.wait_app_alive()
    timings["app_alive"] = (time.perf_counter() - started) * 1000
    timings["total"] = (time.perf_counter() - total_started) * 1000
    return {
        "ok": True,
        "status": "activated",
        "image_size": len(image),
        "image_crc": image_crc,
        "session": begin["session"],
        "timings_ms": timings,
    }
