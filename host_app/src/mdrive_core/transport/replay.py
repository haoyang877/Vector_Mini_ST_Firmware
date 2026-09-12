"""Offline JSONL replay transport.

RX rows are returned in recorded timestamp order. With the default time scale
of zero, each receive call drains all remaining RX rows immediately. TX rows
are documentary markers; sent frames are always retained in ``captured_tx``.
"""

from dataclasses import dataclass
import json
from pathlib import Path
import time

from .base import TransportError, TransportStats
from .frame import CanFrame


@dataclass(frozen=True, slots=True)
class _ReplayFrame:
    time: float
    direction: str
    frame: CanFrame


class ReplayTransport:
    """A deterministic, mutable playback cursor over a JSONL capture."""

    def __init__(self, path: Path, time_scale: float = 0.0) -> None:
        if time_scale < 0:
            raise TransportError("replay time scale cannot be negative")
        self.path = path
        self.time_scale = time_scale
        self.stats = TransportStats()
        self.captured_tx: list[CanFrame] = []
        self._records: list[_ReplayFrame] = []
        self._cursor = 0
        self._opened_at: float | None = None

    @property
    def info(self) -> dict[str, str | int]:
        return {"source": str(self.path), "records": len(self._records)}

    def open(self) -> None:
        """Parse the capture and reset playback to its first RX row."""
        records: list[_ReplayFrame] = []
        try:
            with self.path.open("r", encoding="utf-8") as capture:
                for line_number, line in enumerate(capture, 1):
                    if not line.strip():
                        continue
                    row = json.loads(line)
                    direction = str(row["dir"])
                    if direction not in {"rx", "tx"}:
                        raise TransportError(f"line {line_number}: dir must be rx or tx")
                    identifier = int(row["id"])
                    data = bytes.fromhex(str(row["data"]))
                    flags = int(row.get("flags", 0))
                    recorded_at = float(row["t"])
                    records.append(_ReplayFrame(recorded_at, direction, CanFrame(identifier, data, flags, recorded_at)))
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise TransportError(f"invalid replay capture {self.path}: {exc}") from exc
        self._records = sorted(records, key=lambda record: record.time)
        self._cursor = 0
        self._opened_at = time.perf_counter()

    def close(self) -> None:
        self._opened_at = None

    def send(self, channel: int, identifier: int, data: bytes) -> None:
        if self._opened_at is None:
            raise TransportError("replay transport is not open")
        self.captured_tx.append(CanFrame(identifier, bytes(data), 1, time.perf_counter()))
        self.stats.tx_count += 1
        for index, record in enumerate(self._records[self._cursor :], self._cursor):
            if record.direction == "tx" and record.frame.identifier == identifier and record.frame.data == data:
                self._cursor = index + 1
                break

    def receive(self, channel: int, timeout: float) -> list[CanFrame]:
        if self._opened_at is None:
            raise TransportError("replay transport is not open")
        if timeout < 0:
            raise TransportError("receive timeout cannot be negative")
        if self._cursor >= len(self._records):
            return []
        first_rx = next((record for record in self._records if record.direction == "rx"), None)
        if first_rx is None:
            return []
        cutoff = float("inf")
        if self.time_scale != 0:
            cutoff = first_rx.time + (time.perf_counter() - self._opened_at + timeout) / self.time_scale
        selected: list[_ReplayFrame] = []
        while self._cursor < len(self._records):
            record = self._records[self._cursor]
            if record.direction == "tx" or record.time > cutoff:
                break
            selected.append(record)
            self._cursor += 1
        frames = [record.frame for record in selected]
        self.stats.rx_count += len(frames)
        self.stats.error_frames += sum(bool(frame.identifier & 0x20000000) for frame in frames)
        return frames
