"""Offline Loader wire-format tests; never opens a CAN adapter."""

from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "loader" / "tools"))

from loader_proto import (  # noqa: E402
    APP_BASE_ADDRESS,
    OP_GET_INFO,
    RESULT_BAD_STATE,
    crc32,
    frame_bytes,
    parse_reply,
    valid_fd_lengths,
)
from loader_updater import LoaderClient  # noqa: E402


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
                self.pending: list[tuple[int, bytes, int]] = []

            def send(self, channel: int, identifier: int, data: bytes) -> None:
                self.sent.append(data)
                if len(self.sent) == 2:
                    payload = struct.pack(">HIIIB3x", 0, 3, APP_BASE_ADDRESS, 0x20000, 0)
                    self.pending.append((0x7E0, frame_bytes(OP_GET_INFO, 0, 0, payload), 1))

            def receive(self, channel: int, timeout: float) -> list[tuple[int, bytes, int]]:
                frames, self.pending = self.pending, []
                return frames

        bus = RetryBus()

        info = LoaderClient(bus, timeout=0.001).get_info()

        self.assertEqual(info["app_base"], APP_BASE_ADDRESS)
        self.assertEqual(len(bus.sent), 2)
        self.assertEqual(bus.sent[0], bus.sent[1])


if __name__ == "__main__":
    unittest.main()
