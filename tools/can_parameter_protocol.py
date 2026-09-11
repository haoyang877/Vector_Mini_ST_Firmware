"""Current standard-ID parameter wire format. All public values use SI units.

No automatic fallback to the incompatible float/turns protocol. The client
serializes queries; this legacy protocol has no transaction sequence number.
"""
from dataclasses import dataclass
import math
import struct
import time

CURRENT_SET = {0x02, 0x0e, 0x10}
POSITION_SET = {0x06}
SPEED_SET = {0x04, 0x12, 0x14, 0x16, 0x1c, 0x1e, 0x20}
CURRENT_GET = {0x03, 0x0f, 0x11, 0x2f, 0x31, 0x33, 0x35, 0x37, 0x39, 0x5b, 0x5c, 0x5f, 0x60}
POSITION_GET = {0x07, 0x41}
SPEED_GET = {0x05, 0x13, 0x15, 0x17, 0x1d, 0x1f, 0x21, 0x3f}


def layout(param, reply=False):
    """Return big-endian packing and scale for the parameter on this wire."""
    if param in (CURRENT_GET if reply else CURRENT_SET): return '>h', 1000
    if param in (POSITION_GET if reply else POSITION_SET): return '>i', 1000
    if param in (SPEED_GET if reply else SPEED_SET): return '>i', 100
    return '>f', 1


def encode(param, value):
    if not math.isfinite(value): raise ValueError('nonfinite command')
    fmt, scale = layout(param)
    if fmt != '>f':
        value = math.trunc(value * scale)
        limit = 32767 if fmt == '>h' else 2147483647
        if not -limit <= value <= limit: raise ValueError('command outside wire range')
    return struct.pack(fmt, value)


def decode(param, data):
    fmt, scale = layout(param, reply=True)
    if len(data) != struct.calcsize(fmt): raise ValueError('unexpected reply length')
    value = struct.unpack(fmt, data)[0]
    if not math.isfinite(value) or (fmt != '>f' and value == -(1 << (8 * len(data) - 1))):
        raise ValueError('invalid reply sentinel')
    return value / scale


@dataclass(frozen=True)
class Frame:
    channel: int
    identifier: int
    data: bytes
    device_us: int
    received_at: float


class Client:
    """Single outstanding query; trace actual wire bytes including status frames."""
    def __init__(self, bus, timeout=.08, trace=None):
        self.bus, self.timeout, self.trace = bus, timeout, trace

    def _receive(self, channel):
        frames = self.bus.receive(channel)
        for f in frames:
            if self.trace:
                self.trace('rx', f.channel, f.identifier, f.data, f.received_at, f.device_us)
        return frames

    def _send(self, channel, node, param, data):
        if not 0 <= node <= 7 or not 0 <= param <= 255: raise ValueError('invalid address')
        identifier = (node << 8) | param
        self.bus.send(channel, identifier, data)
        if self.trace: self.trace('tx', channel, identifier, data, time.perf_counter(), None)

    def write(self, channel, node, param, value):
        self._send(channel, node, param, encode(param, value))

    def read(self, channel, node, set_param):
        if set_param & 1 or not 0 <= set_param < 0x58 and set_param != 0x64:
            raise ValueError('read requires a supported paired SET parameter')
        self._receive(channel)  # Discard queued replies before issuing the query.
        reply = set_param + 1
        self._send(channel, node, reply, bytes(4))
        deadline = time.perf_counter() + self.timeout
        while time.perf_counter() < deadline:
            for f in self._receive(channel):
                if f.identifier == (node << 8) | reply:
                    return decode(reply, f.data), f
            time.sleep(.001)
        raise TimeoutError(f'node {node} parameter {reply:#x} reply timeout')

    def verify(self, channel, node, param, expected, tolerance=1e-5):
        actual, _ = self.read(channel, node, param)
        fmt, scale = layout(param + 1, reply=True)
        # Float-to-fixed truncation may lose one wire LSB on a round trip.
        tolerance = max(tolerance, 1 / scale + 1e-6) if fmt != '>f' else tolerance
        if abs(actual - expected) > tolerance:
            raise ValueError(f'node {node} parameter {param:#x}: {actual} != {expected}')
        return actual
