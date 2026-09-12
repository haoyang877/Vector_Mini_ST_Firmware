"""Telemetry collection: status-stream configuration, sampling, and CSV recording.

The firmware status stream sends 48-byte CAN-FD frames on ``0x7F0 + node``;
the rate is configured with ``0x64`` (float32 Hz: 0=off, 1=resume, 10..200=set)
and queried with ``0x65``. Samples are decoded by
``mdrive_core.protocol.status``; invalid sentinels become ``None``.
"""

from __future__ import annotations

from collections.abc import Callable
import csv
from dataclasses import dataclass
import struct
import threading
import time

from mdrive_core.protocol import status
from mdrive_core.transport.base import CanTransport

STATUS_ID_BASE = 0x7F0
STATUS_FRAME_LEN = 48
STREAM_CMD_ID = 0x64
STREAM_QUERY_ID = 0x65
ADAPTER_ERROR_FLAG = 0x2000_0000

CSV_COLUMNS = (
    "time_s",
    "node",
    "fault",
    "mode",
    "position_target_rad",
    "position_feedback_rad",
    "speed_target_rad_s",
    "speed_feedback_rad_s",
    "iq_reference_A",
    "iq_feedback_A",
    "position_planned_rad",
    "speed_planned_rad_s",
    "temperature_C",
    "bus_voltage_V",
)


class TelemetryError(ValueError):
    """The stream could not be configured or no samples arrived."""


def configure_stream(
    transport: CanTransport,
    channel: int = 0,
    node: int = 0,
    rate_hz: float = 0.0,
    timeout: float = 0.35,
    retries: int = 6,
) -> float | None:
    """Set (0/1/10..200) or query the stream; returns the reported rate or None."""
    identifier = (node << 8) | STREAM_CMD_ID
    for _ in range(retries):
        transport.send(channel, identifier, struct.pack(">f", rate_hz))
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            for frame in transport.receive(channel, min(0.03, max(0.0, deadline - time.perf_counter()))):
                if frame.identifier & ADAPTER_ERROR_FLAG:
                    continue
                if (frame.identifier & 0x7FF) == ((node << 8) | STREAM_QUERY_ID):
                    if len(frame.data) == 4:
                        return struct.unpack(">f", frame.data)[0]
    return None


@dataclass(frozen=True)
class StreamSample:
    """One decoded status sample with host receipt time."""

    time_s: float
    node: int
    values: dict[str, float | None]


def _sample_from_frame(frame, node: int, time_s: float) -> StreamSample:
    """Decode one 48-byte status frame into a sample (temperature normalized)."""
    decoded = status.decode(frame.identifier, frame.data)
    values = {key: decoded.get(key) for key in CSV_COLUMNS[2:]}
    # Firmware NTC formula yields -273.15 C when the sensor reads zero
    # (no NTC / disconnected); treat it as "no reading".
    temperature = values.get("temperature_C")
    if temperature is not None and temperature <= -273.0:
        values["temperature_C"] = None
    return StreamSample(time_s=time_s, node=node, values=values)


def collect(
    transport: CanTransport,
    channel: int = 0,
    node: int = 0,
    duration_s: float = 1.0,
    on_sample: Callable[[StreamSample], None] | None = None,
    stop_event=None,
    poll_timeout: float = 0.05,
) -> tuple[int, float]:
    """Receive the status stream for ``duration_s``; returns (frames, elapsed)."""
    started = time.perf_counter()
    deadline = started + duration_s
    frames = 0
    while time.perf_counter() < deadline:
        if stop_event is not None and stop_event.is_set():
            break
        for frame in transport.receive(channel, poll_timeout):
            if frame.identifier & ADAPTER_ERROR_FLAG:
                continue
            if frame.identifier != STATUS_ID_BASE + node or len(frame.data) != STATUS_FRAME_LEN:
                continue
            frames += 1
            if on_sample is not None:
                on_sample(_sample_from_frame(frame, node, time.perf_counter() - started))
    return frames, time.perf_counter() - started


class StreamSession:
    """Open-ended status sampling with periodic statistics and CSV recording."""

    def __init__(
        self,
        transport: CanTransport,
        channel: int = 0,
        node: int = 0,
        rate_hz: float = 100.0,
        record_path=None,
        on_sample: Callable[[StreamSample], None] | None = None,
        on_stats: Callable[[dict], None] | None = None,
        poll_timeout: float = 0.05,
        stats_interval_s: float = 0.5,
        max_duration_s: float | None = None,
    ) -> None:
        self.transport = transport
        self.channel = channel
        self.node = node
        self.rate_hz = rate_hz
        self.record_path = record_path
        self.on_sample = on_sample
        self.on_stats = on_stats
        self.poll_timeout = poll_timeout
        self.stats_interval_s = stats_interval_s
        self.max_duration_s = max_duration_s
        self._stop = threading.Event()

    def stop(self) -> None:
        """Request a cooperative stop (safe from any thread)."""
        self._stop.set()

    def run(self) -> dict[str, float | int]:
        """Configure, sample until stopped, then disable the stream."""
        reported = configure_stream(self.transport, self.channel, self.node, self.rate_hz)
        if reported is None:
            raise TelemetryError("status stream did not acknowledge 0x64/0x65")
        recorder = CsvRecorder(self.record_path) if self.record_path else None
        started = time.perf_counter()
        frames = 0
        invalid = 0
        last_stats = started
        try:
            while not self._stop.is_set():
                if self.max_duration_s is not None and time.perf_counter() - started >= self.max_duration_s:
                    break
                for frame in self.transport.receive(self.channel, self.poll_timeout):
                    if self._stop.is_set():
                        break
                    if frame.identifier & ADAPTER_ERROR_FLAG:
                        continue
                    if frame.identifier != STATUS_ID_BASE + self.node or len(frame.data) != STATUS_FRAME_LEN:
                        continue
                    sample = _sample_from_frame(frame, self.node, time.perf_counter() - started)
                    invalid += len(status.decode(frame.identifier, frame.data).get("invalid_fields", []))
                    frames += 1
                    if recorder is not None:
                        recorder.write(sample)
                    if self.on_sample is not None:
                        self.on_sample(sample)
                now = time.perf_counter()
                if self.on_stats is not None and now - last_stats >= self.stats_interval_s:
                    elapsed = now - started
                    self.on_stats(
                        {"frames": frames, "elapsed_s": elapsed, "actual_hz": frames / elapsed if elapsed else 0.0}
                    )
                    last_stats = now
        finally:
            if recorder is not None:
                recorder.close()
            configure_stream(self.transport, self.channel, self.node, 0.0)
        elapsed = time.perf_counter() - started
        return {
            "frames": frames,
            "elapsed_s": elapsed,
            "actual_hz": frames / elapsed if elapsed else 0.0,
            "invalid_fields": invalid,
            "recorded_rows": recorder.rows if recorder is not None else 0,
            "requested_hz": self.rate_hz,
        }


class CsvRecorder:
    """Write stream samples as CSV columns compatible with ``feedback.csv``."""

    def __init__(self, path) -> None:
        self._stream = open(path, "w", newline="", encoding="utf-8")  # noqa: SIM115 - owned until close()
        self._writer = csv.DictWriter(self._stream, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self.rows = 0

    def write(self, sample: StreamSample) -> None:
        row = {"time_s": f"{sample.time_s:.6f}", "node": sample.node}
        for key in CSV_COLUMNS[2:]:
            value = sample.values.get(key)
            row[key] = "" if value is None else repr(value)
        self._writer.writerow(row)
        self.rows += 1

    def close(self) -> None:
        self._stream.close()

    def __enter__(self) -> CsvRecorder:
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()
