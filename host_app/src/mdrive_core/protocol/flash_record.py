"""Readback of the flash parameter record (firmware package B: 0x69 / 0x6A).

Wire contract (standard ID ``(node << 8) | op``, CAN-FD + BRS, stop-and-wait):

* ``0x69 FLASH_PARAM_INFO``: request 4 zero bytes; reply 16 bytes:
  ``magic u32 BE, schema u32 BE, size u32 BE, record_crc32 u32 BE``.
* ``0x6A FLASH_PARAM_CHUNK``: request ``float32 offset`` (integer-valued);
  reply ``offset u32 BE, len u8, data[len]`` with ``len <= 56``.

The record data itself is the raw flash bytes (little-endian C structure,
2240 bytes on schema v10). The adapter occasionally tags correctly received
frames with ``0x20000000`` (bus glitch / ESI); those are counted and skipped.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import struct
import time
import zlib

from mdrive_core.transport.base import CanTransport

OP_INFO = 0x69
OP_CHUNK = 0x6A
CHUNK_DATA_MAX = 56
INFO_REPLY_LEN = 16
ADAPTER_ERROR_FLAG = 0x2000_0000
RECORD_MAGIC = 0x454E4332


class FlashRecordError(ValueError):
    """Flash record readback failed, is inconsistent, or is unsupported."""


@dataclass(frozen=True)
class FlashRecordInfo:
    """Header of the flash parameter record."""

    magic: int
    schema: int
    size: int
    crc32: int

    def to_dict(self) -> dict[str, int]:
        return {"magic": self.magic, "schema": self.schema, "size": self.size, "crc32": self.crc32}


@dataclass
class ReadStats:
    """Transport troubleshooting counters for one readback session."""

    attempts: int = 0
    error_frames: int = 0
    timeouts: int = 0


def _request(
    transport: CanTransport,
    channel: int,
    node: int,
    op: int,
    payload: bytes,
    timeout: float,
    retries: int,
    stats: ReadStats,
) -> bytes | None:
    identifier = (node << 8) | op
    for _ in range(retries):
        stats.attempts += 1
        transport.send(channel, identifier, payload)
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            wait = min(0.03, max(0.0, deadline - time.perf_counter()))
            for frame in transport.receive(channel, wait):
                if frame.identifier & ADAPTER_ERROR_FLAG:
                    stats.error_frames += 1
                    continue
                if (frame.identifier & 0x7FF) == identifier:
                    return frame.data
    stats.timeouts += 1
    return None


def read_info(
    transport: CanTransport,
    channel: int = 0,
    node: int = 0,
    timeout: float = 0.35,
    retries: int = 8,
    stats: ReadStats | None = None,
) -> FlashRecordInfo:
    """Read the 16-byte INFO header; raise when unsupported or unanswered."""
    stats = stats or ReadStats()
    data = _request(transport, channel, node, OP_INFO, bytes(4), timeout, retries, stats)
    if data is None:
        raise FlashRecordError(
            "FLASH_PARAM_INFO (0x69) timed out; firmware package B not flashed? "
            f"(error_frames={stats.error_frames})"
        )
    if len(data) != INFO_REPLY_LEN:
        raise FlashRecordError(f"INFO reply length {len(data)}, expected {INFO_REPLY_LEN}")
    magic, schema, size, crc32 = struct.unpack(">IIII", data)
    if magic != RECORD_MAGIC:
        raise FlashRecordError(f"record magic 0x{magic:08X}, expected 0x{RECORD_MAGIC:08X}")
    if size == 0 or size > 4096:
        raise FlashRecordError(f"record size {size} outside 1..4096")
    return FlashRecordInfo(magic=magic, schema=schema, size=size, crc32=crc32)


def read_record(
    transport: CanTransport,
    channel: int = 0,
    node: int = 0,
    info: FlashRecordInfo | None = None,
    timeout: float = 0.35,
    retries: int = 8,
    progress: Callable[[int, int], None] | None = None,
    stats: ReadStats | None = None,
) -> bytes:
    """Read the whole record chunk by chunk and verify the INFO CRC32."""
    stats = stats or ReadStats()
    info = info or read_info(transport, channel, node, timeout, retries, stats)
    blob = bytearray()
    offset = 0
    while offset < info.size:
        data = _request(
            transport, channel, node, OP_CHUNK, struct.pack(">f", float(offset)), timeout, retries, stats
        )
        if data is None:
            raise FlashRecordError(f"CHUNK (0x6A) timed out at offset {offset} (error_frames={stats.error_frames})")
        if len(data) < 5:
            raise FlashRecordError(f"CHUNK reply too short at offset {offset}: {len(data)} bytes")
        reply_offset, length = struct.unpack(">IB", data[:5])
        if reply_offset != offset:
            raise FlashRecordError(f"CHUNK offset mismatch: got {reply_offset}, expected {offset}")
        if length > CHUNK_DATA_MAX or len(data) < 5 + length:
            raise FlashRecordError(f"CHUNK length {length} invalid at offset {offset}")
        blob += data[5 : 5 + length]
        offset += length
        if progress is not None:
            progress(min(offset, info.size), info.size)
    result = bytes(blob)
    calc = zlib.crc32(result) & 0xFFFFFFFF
    if calc != info.crc32:
        raise FlashRecordError(f"record CRC mismatch: computed 0x{calc:08X}, INFO says 0x{info.crc32:08X}")
    return result
