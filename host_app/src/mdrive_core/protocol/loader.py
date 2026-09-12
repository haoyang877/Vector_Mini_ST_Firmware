"""Pure framing for the motor-drive Loader CAN-FD protocol."""

import struct
from typing import Final, TypedDict
import zlib

MAGIC: Final = 0xB1
PROTOCOL_VERSION: Final = 0x01
HEADER_SIZE: Final = 12
CRC_SIZE: Final = 4
MAX_PAYLOAD_SIZE: Final = 48
FD_LENGTHS: Final = (16, 20, 24, 32, 48, 64)

REQUEST_ID_BASE: Final = 0x7D0
RESPONSE_ID_BASE: Final = 0x7E0
APP_REQUEST_ENTER_BOOT: Final = 0x66
APP_REQUEST_GET_MODE: Final = 0x01
APP_BASE_ADDRESS: Final = 0x08004000

OP_GET_INFO: Final = 0x01
OP_GET_STATUS: Final = 0x02
OP_BEGIN_UPDATE: Final = 0x20
OP_ERASE: Final = 0x21
OP_PROGRAM: Final = 0x22
OP_VERIFY: Final = 0x23
OP_ACTIVATE: Final = 0x24
OP_ABORT: Final = 0x25

RESULT_OK: Final = 0
RESULT_BAD_FRAME: Final = 1
RESULT_BAD_CRC: Final = 2
RESULT_BAD_STATE: Final = 3
RESULT_BAD_OFFSET: Final = 4
RESULT_BAD_SIZE: Final = 5
RESULT_FLASH_ERR: Final = 6
RESULT_VERIFY_FAIL: Final = 7
RESULT_BAD_OP: Final = 8

RESULT_NAMES: Final = {
    RESULT_OK: "OK",
    RESULT_BAD_FRAME: "BAD_FRAME",
    RESULT_BAD_CRC: "BAD_CRC",
    RESULT_BAD_STATE: "BAD_STATE",
    RESULT_BAD_OFFSET: "BAD_OFFSET",
    RESULT_BAD_SIZE: "BAD_SIZE",
    RESULT_FLASH_ERR: "FLASH_ERR",
    RESULT_VERIFY_FAIL: "VERIFY_FAIL",
    RESULT_BAD_OP: "BAD_OP",
}


class Reply(TypedDict):
    opcode: int
    flags: int
    seq: int
    payload_len: int
    session: int
    payload: bytes
    result: int


class ProtocolFrameError(ValueError):
    """A received frame does not satisfy the frozen wire format."""


def crc32(data: bytes) -> int:
    """Return the unsigned ISO-HDLC CRC32 used on the wire."""
    return zlib.crc32(data) & 0xFFFFFFFF


def valid_fd_lengths(minimum: int = 0) -> tuple[int, ...]:
    """Return valid CAN-FD lengths that contain at least ``minimum`` bytes."""
    return tuple(length for length in FD_LENGTHS if length >= minimum)


def frame_bytes(op: int, seq: int, session: int, payload: bytes = b"") -> bytes:
    """Encode and pad one Loader frame to the smallest valid CAN-FD length."""
    if not 0 <= op <= 0xFF:
        raise ProtocolFrameError(f"opcode outside u8 range: {op}")
    if not 0 <= seq <= 0xFFFF:
        raise ProtocolFrameError(f"sequence outside u16 range: {seq}")
    if not 0 <= session <= 0xFFFFFFFF:
        raise ProtocolFrameError(f"session outside u32 range: {session}")
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ProtocolFrameError(f"payload exceeds {MAX_PAYLOAD_SIZE} bytes")
    header = struct.pack(">BBBBHHI", MAGIC, PROTOCOL_VERSION, op, 0, seq, len(payload), session)
    protected = header + payload
    encoded = protected + struct.pack(">I", crc32(protected))
    lengths = valid_fd_lengths(len(encoded))
    if not lengths:
        raise ProtocolFrameError("frame exceeds 64-byte CAN-FD limit")
    return encoded.ljust(lengths[0], b"\xFF")


def parse_reply(raw: bytes) -> Reply:
    """Parse and CRC-check a padded Loader reply."""
    if len(raw) not in FD_LENGTHS:
        raise ProtocolFrameError(f"invalid CAN-FD frame length: {len(raw)}")
    magic, version, opcode, flags, seq, payload_len, session = struct.unpack(">BBBBHHI", raw[:HEADER_SIZE])
    if magic != MAGIC or version != PROTOCOL_VERSION or flags != 0:
        raise ProtocolFrameError("invalid Loader header")
    if payload_len < 2 or payload_len > MAX_PAYLOAD_SIZE:
        raise ProtocolFrameError(f"invalid reply payload length: {payload_len}")
    crc_offset = HEADER_SIZE + payload_len
    frame_end = crc_offset + CRC_SIZE
    if frame_end > len(raw):
        raise ProtocolFrameError("declared payload exceeds frame length")
    if raw[frame_end:] != b"\xFF" * (len(raw) - frame_end):
        raise ProtocolFrameError("invalid CAN-FD padding")
    protected = raw[:crc_offset]
    expected_crc = int.from_bytes(raw[crc_offset:frame_end], "big")
    if crc32(protected) != expected_crc:
        raise ProtocolFrameError("Loader frame CRC mismatch")
    payload = raw[HEADER_SIZE:crc_offset]
    return {
        "opcode": opcode,
        "flags": flags,
        "seq": seq,
        "payload_len": payload_len,
        "session": session,
        "payload": payload,
        "result": int.from_bytes(payload[:2], "big"),
    }
