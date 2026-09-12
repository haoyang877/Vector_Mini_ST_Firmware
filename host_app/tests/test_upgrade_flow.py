"""Offline tests for the phased/abortable firmware update flow."""

import struct
import sys
import threading
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from mdrive_core.protocol.loader import (  # noqa: E402
    APP_BASE_ADDRESS,
    OP_ABORT,
    OP_ACTIVATE,
    OP_BEGIN_UPDATE,
    OP_ERASE,
    OP_PROGRAM,
    OP_VERIFY,
    crc32,
    frame_bytes,
)
from mdrive_core.protocol.loader_client import (  # noqa: E402
    LoaderClient,
    UpgradeAborted,
    update_firmware,
)
from mdrive_core.transport.frame import CanFrame  # noqa: E402

SESSION = 0x1234


class FakeLoaderBus:
    """Simulates a Loader device: answers BEGIN/ERASE/PROGRAM/VERIFY/ACTIVATE/ABORT."""

    def __init__(self, image_crc: int) -> None:
        self.image_crc = image_crc
        self.pending: list[CanFrame] = []
        self.ops: list[int] = []

    def send(self, channel: int, identifier: int, data: bytes) -> None:
        if (identifier & 0xFF) == 0x01 and len(data) == 4:  # legacy GET_MODE poll
            self.pending.append(CanFrame(identifier, bytes(4), 1, None))
            return
        if len(data) < 12:
            return
        op = data[2]
        seq = (data[4] << 8) | data[5]
        if op == OP_BEGIN_UPDATE:
            self.ops.append(op)
            self._reply(op, seq, struct.pack(">HII", 0, SESSION, APP_BASE_ADDRESS))
        elif op == OP_ERASE:
            self.ops.append(op)
            self._reply(op, seq, struct.pack(">H", 0))
        elif op == OP_PROGRAM:
            self.ops.append(op)
            offset = int.from_bytes(data[12:16], "big")
            plen = (data[6] << 8) | data[7]
            self._reply(op, seq, struct.pack(">HI", 0, offset + (plen - 4)))
        elif op == OP_VERIFY:
            self.ops.append(op)
            self._reply(op, seq, struct.pack(">HI", 0, self.image_crc))
        elif op == OP_ACTIVATE:
            self.ops.append(op)
            self._reply(op, seq, struct.pack(">H", 0))
        elif op == OP_ABORT:
            self.ops.append(op)
            self._reply(op, seq, struct.pack(">H", 0))

    def _reply(self, op: int, seq: int, payload: bytes) -> None:
        session = 0 if op == OP_BEGIN_UPDATE else SESSION
        self.pending.append(CanFrame(0x7E0, frame_bytes(op, seq, session, payload), 1, None))

    def receive(self, channel: int, timeout: float) -> list[CanFrame]:
        out, self.pending = self.pending, []
        return out


def make_image(size: int = 5000) -> bytes:
    return bytes((index * 37 + 11) & 0xFF for index in range(size))


class UpdateFlowTest(unittest.TestCase):
    def test_phased_update_succeeds(self) -> None:
        image = make_image()
        bus = FakeLoaderBus(crc32(image))
        client = LoaderClient(bus, node=0)
        phases: list[str] = []
        progress: list[tuple[int, int]] = []
        result = update_firmware(
            client,
            image,
            version=7,
            progress=lambda done, total: progress.append((done, total)),
            on_phase=phases.append,
        )
        self.assertEqual(
            phases, ["begin", "erase", "program", "verify", "activate", "app_alive"]
        )
        self.assertTrue(result["ok"])
        self.assertEqual(result["session"], SESSION)
        self.assertEqual(result["image_size"], len(image))
        self.assertEqual(progress[-1], (len(image), len(image)))
        self.assertIn(OP_ACTIVATE, bus.ops)
        self.assertNotIn(OP_ABORT, bus.ops)

    def test_abort_before_erase(self) -> None:
        image = make_image()
        bus = FakeLoaderBus(crc32(image))
        client = LoaderClient(bus, node=0)
        event = threading.Event()

        def on_phase(name: str) -> None:
            if name == "begin":
                event.set()

        with self.assertRaises(UpgradeAborted):
            update_firmware(client, image, version=7, on_phase=on_phase, abort_event=event)
        self.assertNotIn(OP_ERASE, bus.ops)
        self.assertIn(OP_ABORT, bus.ops)

    def test_abort_mid_program(self) -> None:
        image = make_image(20000)
        bus = FakeLoaderBus(crc32(image))
        client = LoaderClient(bus, node=0)
        event = threading.Event()
        calls = {"n": 0}

        def progress(done: int, total: int) -> None:
            calls["n"] += 1
            if calls["n"] >= 3:
                event.set()

        with self.assertRaises(UpgradeAborted):
            update_firmware(client, image, version=7, progress=progress, abort_event=event)
        self.assertIn(OP_PROGRAM, bus.ops)
        self.assertNotIn(OP_VERIFY, bus.ops)
        self.assertNotIn(OP_ACTIVATE, bus.ops)
        self.assertIn(OP_ABORT, bus.ops)


if __name__ == "__main__":
    unittest.main()
