"""Revision-2 standard-ID parameter wire format using SI public values."""

from collections.abc import Callable
import math
import struct
import time
from typing import Protocol

from mdrive_core.transport.frame import CanFrame

PROTOCOL_REVISION = 2
CURRENT_SET = frozenset({0x02, 0x0E, 0x10})
POSITION_SET = frozenset({0x06})
SPEED_SET = frozenset({0x04, 0x12, 0x14, 0x16, 0x1C, 0x1E, 0x20})
CURRENT_GET = frozenset({0x03, 0x0F, 0x11, 0x2F, 0x31, 0x33, 0x35, 0x37, 0x39, 0x5B, 0x5C, 0x5F, 0x60})
POSITION_GET = frozenset({0x07, 0x41})
SPEED_GET = frozenset({0x05, 0x13, 0x15, 0x17, 0x1D, 0x1F, 0x21, 0x3F})

Trace = Callable[[str, int, int, bytes, float, int | None], None]
Poll = Callable[[], None]


class ParameterBus(Protocol):
    def send(self, channel: int, identifier: int, data: bytes) -> None: ...

    def receive(self, channel: int, timeout: float) -> list[CanFrame]: ...


class ParameterValueError(ValueError):
    """A parameter value or frame violates the revision-2 wire contract."""


def layout(param: int, reply: bool = False) -> tuple[str, int]:
    """Return the validated big-endian struct format and SI scale."""
    if param in (CURRENT_GET if reply else CURRENT_SET):
        return ">h", 1000
    if param in (POSITION_GET if reply else POSITION_SET):
        return ">i", 1000
    if param in (SPEED_GET if reply else SPEED_SET):
        return ">i", 100
    return ">f", 1


def encode(param: int, value: float) -> bytes:
    """Encode a public SI value for a SET or command ID."""
    if not math.isfinite(value):
        raise ParameterValueError("nonfinite command")
    wire_format, scale = layout(param)
    wire_value: float | int = value
    if wire_format != ">f":
        wire_value = math.trunc(value * scale)
        limit = 32767 if wire_format == ">h" else 2147483647
        if not -limit <= wire_value <= limit:
            raise ParameterValueError("command outside wire range")
    return struct.pack(wire_format, wire_value)


def decode(param: int, data: bytes) -> float:
    """Decode one reply and reject the firmware's invalid sentinels."""
    wire_format, scale = layout(param, reply=True)
    if len(data) != struct.calcsize(wire_format):
        raise ParameterValueError("unexpected reply length")
    value = struct.unpack(wire_format, data)[0]
    sentinel = -(1 << (8 * len(data) - 1))
    if not math.isfinite(value) or (wire_format != ">f" and value == sentinel):
        raise ParameterValueError("invalid reply sentinel")
    return float(value) / scale


def decode_as(encoding: str, data: bytes) -> float:
    """Decode a reply using an explicit schema encoding."""
    formats = {"f32": (">f", 1), "ma16": (">h", 1000), "mrad32": (">i", 1000), "crad32": (">i", 100)}
    try:
        wire_format, scale = formats[encoding]
    except KeyError as exc:
        raise ParameterValueError(f"unknown parameter encoding: {encoding}") from exc
    if len(data) != struct.calcsize(wire_format):
        raise ParameterValueError("unexpected reply length")
    value = struct.unpack(wire_format, data)[0]
    sentinel = -(1 << (8 * len(data) - 1))
    if not math.isfinite(value) or (wire_format != ">f" and value == sentinel):
        raise ParameterValueError("invalid reply sentinel")
    return float(value) / scale


class Client:
    """Serialize parameter queries because the protocol has no transaction ID."""

    def __init__(
        self,
        bus: ParameterBus,
        timeout: float = 0.08,
        trace: Trace | None = None,
        read_attempts: int = 1,
        poll: Poll | None = None,
    ) -> None:
        if type(read_attempts) is not int or not 1 <= read_attempts <= 3:
            raise ParameterValueError("read_attempts must be 1..3")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ParameterValueError("positive finite query timeout required")
        self.bus = bus
        self.timeout = timeout
        self.trace = trace
        self.read_attempts = read_attempts
        self.poll = poll
        self.retry_events: list[dict[str, int | float]] = []

    def _receive(self, channel: int, timeout: float) -> list[CanFrame]:
        frames = self.bus.receive(channel, timeout)
        for frame in frames:
            if self.trace is not None:
                received_at = frame.timestamp if frame.timestamp is not None else time.perf_counter()
                self.trace("rx", channel, frame.identifier, frame.data, received_at, None)
        if self.poll is not None:
            self.poll()
        return [frame for frame in frames if not frame.identifier & 0x20000000]

    def _send(self, channel: int, node: int, param: int, data: bytes) -> None:
        if not 0 <= node <= 7 or not 0 <= param <= 255:
            raise ParameterValueError("invalid address")
        identifier = (node << 8) | param
        self.bus.send(channel, identifier, data)
        if self.trace is not None:
            self.trace("tx", channel, identifier, data, time.perf_counter(), None)

    def write(self, channel: int, node: int, param: int, value: float) -> None:
        """Send one write without retrying it."""
        self._send(channel, node, param, encode(param, value))

    def read(self, channel: int, node: int, set_param: int) -> tuple[float, CanFrame]:
        """Read the GET paired with a supported SET parameter."""
        if set_param & 1 or (not 0 <= set_param < 0x58 and set_param not in (0x64, 0x66)):
            raise ParameterValueError("read requires a supported paired SET parameter")
        self._receive(channel, 0.0)
        reply_id = set_param + 1
        for attempt in range(self.read_attempts):
            if self.poll is not None:
                self.poll()
            if attempt:
                self.retry_events.append(
                    {"channel": channel, "node": node, "param": reply_id, "attempt": attempt + 1, "time": time.perf_counter()}
                )
            self._send(channel, node, reply_id, bytes(4))
            deadline = time.perf_counter() + self.timeout
            while time.perf_counter() < deadline:
                remaining = max(0.0, deadline - time.perf_counter())
                for frame in self._receive(channel, min(0.01, remaining)):
                    if frame.identifier == (node << 8) | reply_id:
                        return decode(reply_id, frame.data), frame
                time.sleep(0.001)
        raise TimeoutError(f"node {node} parameter {reply_id:#x} reply timeout")

    def require_revision(self, channel: int, node: int) -> float:
        """Require the explicit SI/fixed/48-byte protocol revision."""
        value, _ = self.read(channel, node, 0x66)
        if value != PROTOCOL_REVISION:
            raise ParameterValueError(f"node {node}: protocol revision {value}, expected {PROTOCOL_REVISION}")
        return value

    def verify(self, channel: int, node: int, param: int, expected: float, tolerance: float = 1e-5) -> float:
        """Read a parameter and compare with fixed-point quantization tolerance."""
        actual, _ = self.read(channel, node, param)
        wire_format, scale = layout(param + 1, reply=True)
        accepted_tolerance = max(tolerance, 1 / scale + 1e-6) if wire_format != ">f" else tolerance
        if abs(actual - expected) > accepted_tolerance:
            raise ParameterValueError(f"node {node} parameter {param:#x}: {actual} != {expected}")
        return actual


def raw_get(
    bus: ParameterBus,
    channel: int,
    node: int,
    get_id: int,
    encoding: str,
    timeout: float,
) -> float:
    """Query an unpaired read-only GET ID using schema-selected decoding."""
    if not 0 <= node <= 7 or not 0 <= get_id <= 0xFF:
        raise ParameterValueError("invalid address")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ParameterValueError("positive finite query timeout required")
    bus.receive(channel, 0.0)
    identifier = (node << 8) | get_id
    bus.send(channel, identifier, bytes(4))
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        for frame in bus.receive(channel, min(0.01, max(0.0, deadline - time.perf_counter()))):
            if not frame.identifier & 0x20000000 and frame.identifier == identifier:
                return decode_as(encoding, frame.data)
        time.sleep(0.001)
    raise TimeoutError(f"node {node} parameter {get_id:#x} reply timeout")
