"""Offline tests for telemetry collection/CSV recording and image preparation."""

import csv
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from mdrive_core.services.image_prep import ImagePrepError, prepare_image  # noqa: E402
from mdrive_core.services.telemetry import (  # noqa: E402
    CSV_COLUMNS,
    STATUS_ID_BASE,
    CsvRecorder,
    collect,
    configure_stream,
)
from mdrive_core.transport.frame import CanFrame  # noqa: E402


def build_status_payload() -> bytes:
    return struct.pack(
        ">HHiiiihhiihh12x",
        1,      # fault
        3,      # mode
        1234,   # position target (mrad)
        1200,   # position feedback
        50,     # speed target (centi rad/s)
        40,     # speed feedback
        100,    # iq reference (mA)
        90,     # iq feedback
        1230,   # planned position
        50,     # planned speed
        3050,   # temperature (centi °C)
        2400,   # vbus (centi V)
    )


class StreamBus:
    """Feeds a fixed number of status frames and answers stream-config writes."""

    def __init__(self, frames: int = 5) -> None:
        self.remaining = frames
        self.sent: list[tuple[int, bytes]] = []
        self.pending: list[CanFrame] = []

    def send(self, channel: int, identifier: int, data: bytes) -> None:
        self.sent.append((identifier, data))
        if (identifier & 0xFF) == 0x64:
            self.pending.append(CanFrame((identifier & ~0xFF) | 0x65, struct.pack(">f", 100.0), 1, None))

    def receive(self, channel: int, timeout: float) -> list[CanFrame]:
        out: list[CanFrame] = []
        while self.remaining > 0:
            self.remaining -= 1
            out.append(CanFrame(STATUS_ID_BASE, build_status_payload(), 1, None))
        out += self.pending
        self.pending = []
        return out


class TelemetryTest(unittest.TestCase):
    def test_configure_stream_encodes_and_reports_rate(self) -> None:
        bus = StreamBus(frames=0)
        reported = configure_stream(bus, node=0, rate_hz=100.0, retries=2)
        self.assertAlmostEqual(reported, 100.0)
        identifier, payload = bus.sent[-1]
        self.assertEqual(identifier, 0x64)
        self.assertAlmostEqual(struct.unpack(">f", payload)[0], 100.0)

    def test_collect_decodes_samples(self) -> None:
        bus = StreamBus(frames=5)
        samples = []
        frames, elapsed = collect(bus, duration_s=0.05, on_sample=samples.append)
        self.assertEqual(frames, 5)
        self.assertGreaterEqual(elapsed, 0.0)
        self.assertEqual(len(samples), 5)
        values = samples[0].values
        self.assertEqual(values["fault"], 1)
        self.assertEqual(values["mode"], 3)
        self.assertAlmostEqual(values["position_target_rad"], 1.234)
        self.assertAlmostEqual(values["speed_feedback_rad_s"], 0.4)
        self.assertAlmostEqual(values["temperature_C"], 30.5)
        self.assertAlmostEqual(values["bus_voltage_V"], 24.0)

    def test_absolute_zero_temperature_is_treated_as_invalid(self) -> None:
        class ColdBus(StreamBus):
            def receive(self, channel: int, timeout: float) -> list[CanFrame]:
                frames = []
                while self.remaining > 0:
                    self.remaining -= 1
                    payload = build_status_payload()
                    payload = payload[:32] + struct.pack(">h", -27315) + payload[34:]
                    frames.append(CanFrame(STATUS_ID_BASE, payload, 1, None))
                return frames

        samples = []
        collect(ColdBus(frames=1), duration_s=0.05, on_sample=samples.append)
        self.assertIsNone(samples[0].values["temperature_C"])

    def test_stream_session_stops_records_and_disables(self) -> None:
        from mdrive_core.services.telemetry import StreamSession

        class TrickleBus:
            """Answers the stream config immediately, then trickles one frame per receive."""

            def __init__(self, frames: int) -> None:
                self.remaining = frames
                self.sent: list[tuple[int, bytes]] = []
                self._reply: CanFrame | None = None

            def send(self, channel: int, identifier: int, data: bytes) -> None:
                self.sent.append((identifier, data))
                if (identifier & 0xFF) == 0x64:
                    self._reply = CanFrame((identifier & ~0xFF) | 0x65, struct.pack(">f", 100.0), 1, None)

            def receive(self, channel: int, timeout: float) -> list[CanFrame]:
                if self._reply is not None:
                    reply, self._reply = self._reply, None
                    return [reply]
                if self.remaining > 0:
                    self.remaining -= 1
                    return [CanFrame(STATUS_ID_BASE, build_status_payload(), 1, None)]
                return []

        bus = TrickleBus(frames=10)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "session.csv"
            session = StreamSession(bus, node=0, rate_hz=100, record_path=path, max_duration_s=10.0)
            counter = {"n": 0}

            def on_sample(sample) -> None:
                counter["n"] += 1
                if counter["n"] >= 3:
                    session.stop()

            session.on_sample = on_sample
            summary = session.run()
            self.assertEqual(summary["frames"], 3)
            self.assertEqual(summary["recorded_rows"], 3)
            rows = path.read_text(encoding="utf-8").strip().splitlines()
            self.assertEqual(len(rows), 4)  # header + 3 samples
            last_identifier, last_payload = bus.sent[-1]
            self.assertEqual(last_identifier, 0x64)
            self.assertAlmostEqual(struct.unpack(">f", last_payload)[0], 0.0)

    def test_csv_recorder_columns_and_rows(self) -> None:
        bus = StreamBus(frames=3)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "capture.csv"
            with CsvRecorder(path) as recorder:
                collect(bus, duration_s=0.05, on_sample=recorder.write)
            with path.open(encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(recorder.rows, 3)
            self.assertEqual(list(rows[0].keys()), list(CSV_COLUMNS))
            self.assertAlmostEqual(float(rows[0]["position_target_rad"]), 1.234)
            self.assertAlmostEqual(float(rows[0]["iq_reference_A"]), 0.1)


class ImagePrepTest(unittest.TestCase):
    def test_trim_pad_and_crc(self) -> None:
        raw = b"\x01\x02\x03" + b"\xFF" * 20
        prepared = prepare_image(raw)
        self.assertEqual(prepared.size, 8)
        # padding keeps the erased-flash value 0xFF (no zero programming)
        self.assertEqual(prepared.data, b"\x01\x02\x03" + b"\xFF" * 5)
        import zlib

        self.assertEqual(prepared.crc32, zlib.crc32(prepared.data) & 0xFFFFFFFF)

    def test_pad_when_no_trailing_erased_bytes(self) -> None:
        prepared = prepare_image(b"\x01\x02\x03\x04\x05")
        self.assertEqual(prepared.size, 8)
        self.assertEqual(prepared.data, b"\x01\x02\x03\x04\x05" + b"\xFF" * 3)

    def test_empty_and_erased_rejected(self) -> None:
        with self.assertRaises(ImagePrepError):
            prepare_image(b"")
        with self.assertRaises(ImagePrepError):
            prepare_image(b"\xFF" * 64)

    def test_oversize_rejected(self) -> None:
        with self.assertRaises(ImagePrepError):
            prepare_image(b"\x01" * 100, max_bytes=64)


if __name__ == "__main__":
    unittest.main()
