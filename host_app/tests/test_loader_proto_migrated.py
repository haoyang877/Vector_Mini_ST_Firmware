"""Offline regression tests for the migrated Loader protocol."""

from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from mdrive_core.protocol.loader import (  # noqa: E402
    APP_BASE_ADDRESS,
    OP_GET_INFO,
    RESULT_BAD_STATE,
    crc32,
    frame_bytes,
    parse_reply,
    valid_fd_lengths,
)
from mdrive_core.protocol.loader_client import LoaderClient, update_firmware  # noqa: E402
from mdrive_core.transport.frame import CanFrame  # noqa: E402


class LoaderProtocolTests(unittest.TestCase):
    def test_crc32_iso_hdlc_golden_vector(self) -> None:
        self.assertEqual(crc32(b"123456789"), 0xCBF43926)

    def test_frame_roundtrip_preserves_reply_fields(self) -> None:
        raw = frame_bytes(OP_GET_INFO, 0x1234, 0x89ABCDEF, b"\x00\x00info")

        reply = parse_reply(raw)

        self.assertEqual(reply["opcode"], OP_GET_INFO)
        self.assertEqual(reply["seq"], 0x1234)
        self.assertEqual(reply["session"], 0x89ABCDEF)
        self.assertEqual(reply["payload"], b"\x00\x00info")
        self.assertEqual(reply["result"], 0)

    def test_corrupted_crc_is_rejected(self) -> None:
        raw = bytearray(frame_bytes(OP_GET_INFO, 1, 0, b"\x00\x00"))
        raw[12] ^= 0x80

        with self.assertRaises(ValueError):
            parse_reply(bytes(raw))

    def test_program_payload_uses_64_byte_dlc_and_ff_padding(self) -> None:
        raw = frame_bytes(0x22, 2, 3, bytes(40))

        self.assertEqual(len(raw), 64)
        self.assertEqual(raw[56:], b"\xFF" * 8)

    def test_maximum_payload_fills_64_byte_frame(self) -> None:
        payload = bytes(range(48))

        raw = frame_bytes(0x22, 0xFFFF, 0xFFFFFFFF, payload)

        self.assertEqual(len(raw), 64)
        self.assertEqual(raw[12:60], payload)
        self.assertEqual(valid_fd_lengths(64), (64,))

    def test_result_code_is_parsed(self) -> None:
        raw = frame_bytes(0x25, 7, 9, RESULT_BAD_STATE.to_bytes(2, "big"))

        reply = parse_reply(raw)

        self.assertEqual(reply["result"], RESULT_BAD_STATE)

    def test_client_retry_reuses_identical_sequence_and_frame(self) -> None:
        class RetryBus:
            def __init__(self) -> None:
                self.sent: list[bytes] = []
                self.pending: list[CanFrame] = []

            def send(self, channel: int, identifier: int, data: bytes) -> None:
                self.sent.append(data)
                if len(self.sent) == 2:
                    payload = struct.pack(">HIIIB3x", 0, 3, APP_BASE_ADDRESS, 0x20000, 0)
                    self.pending.append(CanFrame(0x7E0, frame_bytes(OP_GET_INFO, 0, 0, payload), 1))

            def receive(self, channel: int, timeout: float) -> list[CanFrame]:
                frames, self.pending = self.pending, []
                return frames

        bus = RetryBus()

        info = LoaderClient(bus, timeout=0.001).get_info()

        self.assertEqual(info["app_base"], APP_BASE_ADDRESS)
        self.assertEqual(len(bus.sent), 2)
        self.assertEqual(bus.sent[0], bus.sent[1])

    def test_update_chunks_are_aligned_and_do_not_cross_flash_pages(self) -> None:
        class UpdateClient:
            def __init__(self, image: bytes) -> None:
                self.image = image
                self.programmed: list[tuple[int, bytes]] = []

            def begin(self, size: int, crc: int, version: int) -> dict[str, int]:
                return {"session": 7, "app_base": APP_BASE_ADDRESS}

            def erase(self) -> None:
                return None

            def program(self, offset: int, data: bytes) -> int:
                self.programmed.append((offset, data))
                return offset + len(data)

            def verify(self) -> int:
                return crc32(self.image)

            def activate(self) -> None:
                return None

            def wait_app_alive(self) -> bytes:
                return bytes(4)

        image = bytes(index % 251 for index in range(2050))
        client = UpdateClient(image)
        progress: list[tuple[int, int]] = []

        result = update_firmware(client, image, 9, lambda done, total: progress.append((done, total)))

        self.assertTrue(result["ok"])
        self.assertEqual(progress[-1], (len(image), len(image)))
        self.assertTrue(all(offset % 8 == 0 and len(data) % 8 == 0 for offset, data in client.programmed))
        self.assertTrue(all(len(data) <= 40 for _, data in client.programmed))
        self.assertTrue(all(offset // 2048 == (offset + len(data) - 1) // 2048 for offset, data in client.programmed))
        self.assertEqual(client.programmed[-1], (2048, image[2048:].ljust(8, b"\xFF")))


if __name__ == "__main__":
    unittest.main()
