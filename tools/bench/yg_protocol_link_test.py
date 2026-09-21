"""对一块已刷入目标固件的板卡执行只读 CAN FD 连通性冒烟测试。

脚本只发送 GET_CAPS，不发送电机控制、参数写入或 Flash 操作。运行前必须明确
适配器、目标 Node ID、固件 SHA-256 和无动力台架条件；输出保存原始收发帧及校验结果。
"""

from __future__ import annotations

import argparse
import ctypes as C
import hashlib
import json
import struct
import time
from pathlib import Path

VCI_USBCAN2 = 41
STATUS_OK = 1
CANFD_TYPE = 1
NOMINAL_BPS = 1_000_000
DATA_BPS = 5_000_000
PROTOCOL_MAGIC = 0xA55A
PROTOCOL_VERSION = 1
FLAGS_ACK_REQUEST = 0x20
FLAGS_RESPONSE = 0x10
MESSAGE_GET_CAPS = 0x0002
PF = 0xEF


class _CANFD_INIT(C.Structure):
    _fields_ = [
        ("acc_code", C.c_uint),
        ("acc_mask", C.c_uint),
        ("abit_timing", C.c_uint),
        ("dbit_timing", C.c_uint),
        ("brp", C.c_uint),
        ("filter", C.c_ubyte),
        ("mode", C.c_ubyte),
        ("pad", C.c_ushort),
        ("reserved", C.c_uint),
    ]


class _CHANNEL_INIT(C.Union):
    _fields_ = [("canfd", _CANFD_INIT)]


class _CHANNEL_CONFIG(C.Structure):
    _fields_ = [("can_type", C.c_uint), ("config", _CHANNEL_INIT)]


class _CANFD_FRAME(C.Structure):
    _fields_ = [
        ("can_id", C.c_uint32, 29),
        ("err", C.c_uint32, 1),
        ("rtr", C.c_uint32, 1),
        ("eff", C.c_uint32, 1),
        ("length", C.c_ubyte),
        ("brs", C.c_ubyte, 1),
        ("esi", C.c_ubyte, 1),
        ("reserved", C.c_ubyte, 6),
        ("reserved0", C.c_ubyte),
        ("reserved1", C.c_ubyte),
        ("data", C.c_ubyte * 64),
    ]


class _TXFD(C.Structure):
    _fields_ = [("frame", _CANFD_FRAME), ("transmit_type", C.c_uint)]


class _RXFD(C.Structure):
    _fields_ = [("frame", _CANFD_FRAME), ("timestamp", C.c_ulonglong)]


def crc8(data: bytes) -> int:
    value = 0
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ 0x9B) & 0xFF if value & 0x80 else (value << 1) & 0xFF
    return value


def crc16(data: bytes) -> int:
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ 0xBAAD) & 0xFFFF if value & 0x8000 else (value << 1) & 0xFFFF
    return value


def can_id(priority: int, source: int, destination: int) -> int:
    if not 0 <= priority <= 7 or source == 0xFF or not 0 <= source <= 0xFF:
        raise ValueError("invalid priority/source")
    if not 0 <= destination <= 0xFF:
        raise ValueError("invalid destination")
    return (priority << 26) | (PF << 16) | (destination << 8) | source


def encode_request(source: int, destination: int, sequence: int) -> tuple[int, bytes]:
    payload = struct.pack("<IIHH", 0x11223344, sequence, 0, 0)
    header = struct.pack(
        "<HBBBBHHIB",
        PROTOCOL_MAGIC,
        PROTOCOL_VERSION,
        FLAGS_ACK_REQUEST,
        source,
        destination,
        MESSAGE_GET_CAPS,
        sequence,
        len(payload),
        0,
    )
    header += bytes([crc8(header)])
    application = header + payload
    application += struct.pack("<H", crc16(application))
    return can_id(2, source, destination), application + bytes(32 - len(application))


def decode_response(
    identifier: int, data: bytes, source: int, destination: int, sequence: int
) -> dict:
    expected_id = can_id(3, destination, source)
    if identifier != expected_id:
        raise RuntimeError(
            f"unexpected response CAN ID 0x{identifier:08X}, expected 0x{expected_id:08X}"
        )
    if len(data) not in (20, 24, 32, 48, 64) or data[0:2] != struct.pack("<H", PROTOCOL_MAGIC):
        raise RuntimeError("response is not a valid yg_protocol frame")
    payload_length = struct.unpack_from("<I", data, 10)[0]
    frame_length = 16 + payload_length + 2
    if payload_length > 46 or frame_length > len(data) or data[15] != crc8(data[:15]):
        raise RuntimeError("response header CRC or length failed")
    if struct.unpack_from("<H", data, frame_length - 2)[0] != crc16(data[: frame_length - 2]):
        raise RuntimeError("response frame CRC failed")
    if data[3] & FLAGS_RESPONSE == 0 or struct.unpack_from("<H", data, 6)[0] != MESSAGE_GET_CAPS:
        raise RuntimeError("response flags or message type failed")
    if (
        data[4] != destination
        or data[5] != source
        or struct.unpack_from("<H", data, 8)[0] != sequence
    ):
        raise RuntimeError("response addressing or sequence failed")
    if payload_length != 16:
        raise RuntimeError(f"unexpected capabilities payload length {payload_length}")
    payload = data[16 : 16 + payload_length]
    return {
        "identifier": f"0x{identifier:08X}",
        "data": data.hex(" "),
        "message_type": MESSAGE_GET_CAPS,
        "sequence": sequence,
        "features": struct.unpack_from("<I", payload, 0)[0],
        "supported_modes": struct.unpack_from("<H", payload, 4)[0],
        "max_full_frame": struct.unpack_from("<H", payload, 6)[0],
        "max_block_data": struct.unpack_from("<H", payload, 8)[0],
    }


def _ok(code: int, operation: str) -> None:
    if code != STATUS_OK:
        raise OSError(f"{operation}: SDK returned {code}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adapter-root", type=Path, required=True)
    parser.add_argument("--node-id", type=int, required=True)
    parser.add_argument("--source-id", type=int, default=1)
    parser.add_argument("--board", required=True)
    parser.add_argument("--firmware-sha256", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=1.0)
    parser.add_argument("--no-motor-power", action="store_true", required=True)
    args = parser.parse_args()
    if not 1 <= args.node_id <= 254 or not 1 <= args.source_id <= 254:
        parser.error("node IDs must be 1..254")
    if len(args.firmware_sha256) != 64:
        parser.error("--firmware-sha256 must be a 64-character SHA-256")
    if args.timeout_s <= 0 or args.timeout_s > 10:
        parser.error("--timeout-s must be in (0, 10]")
    dll_path = (args.adapter_root / ".local/canfd-sdk/ControlCANFD.dll").resolve()
    if not dll_path.is_file():
        parser.error(f"adapter DLL not found: {dll_path}")
    args.out.mkdir(parents=True, exist_ok=False)
    dll = C.WinDLL(str(dll_path))
    dll.ZCAN_OpenDevice.restype = C.c_void_p
    dll.ZCAN_InitCAN.restype = C.c_void_p
    dll.ZCAN_SetAbitBaud.argtypes = (C.c_void_p, C.c_ulong, C.c_ulong)
    dll.ZCAN_SetDbitBaud.argtypes = (C.c_void_p, C.c_ulong, C.c_ulong)
    dll.ZCAN_SetCANFDStandard.argtypes = (C.c_void_p, C.c_ulong, C.c_ulong)
    dll.ZCAN_InitCAN.argtypes = (C.c_void_p, C.c_ulong, C.c_void_p)
    dll.ZCAN_StartCAN.argtypes = (C.c_void_p,)
    dll.ZCAN_TransmitFD.argtypes = (C.c_void_p, C.c_void_p, C.c_ulong)
    dll.ZCAN_ReceiveFD.argtypes = (C.c_void_p, C.c_void_p, C.c_ulong, C.c_long)
    dll.ZCAN_CloseDevice.argtypes = (C.c_void_p,)
    device = dll.ZCAN_OpenDevice(VCI_USBCAN2, 0, 0)
    if not device:
        raise OSError("CAN adapter open failed")
    channel = None
    request_id, request_data = encode_request(args.source_id, args.node_id, 1)
    report = {
        "scenario": "logic_only_canfd_readonly",
        "board": args.board,
        "node_id": args.node_id,
        "source_id": args.source_id,
        "nominal_bps": NOMINAL_BPS,
        "data_bps": DATA_BPS,
        "no_motor_power": args.no_motor_power,
        "expected_firmware_sha256": args.firmware_sha256.lower(),
        "adapter_dll_sha256": hashlib.sha256(dll_path.read_bytes()).hexdigest(),
        "request": {"identifier": f"0x{request_id:08X}", "data": request_data.hex(" ")},
        "response": None,
        "success": False,
    }
    return_code = 1
    try:
        _ok(dll.ZCAN_SetAbitBaud(device, 0, NOMINAL_BPS), "nominal bitrate")
        _ok(dll.ZCAN_SetDbitBaud(device, 0, DATA_BPS), "data bitrate")
        _ok(dll.ZCAN_SetCANFDStandard(device, 0, 0), "ISO CAN FD")
        config = _CHANNEL_CONFIG()
        config.can_type = CANFD_TYPE
        channel = dll.ZCAN_InitCAN(device, 0, C.byref(config))
        if not channel:
            raise OSError("CAN channel init failed")
        _ok(dll.ZCAN_StartCAN(channel), "CAN start")
        message = _TXFD()
        message.frame.can_id = request_id
        message.frame.eff = 1
        message.frame.rtr = 0
        message.frame.brs = 1
        message.frame.length = len(request_data)
        message.frame.data[: len(request_data)] = request_data
        _ok(dll.ZCAN_TransmitFD(channel, C.byref(message), 1), "GET_CAPS transmit")
        deadline = time.monotonic() + args.timeout_s
        while time.monotonic() < deadline:
            batch = (_RXFD * 32)()
            received = dll.ZCAN_ReceiveFD(channel, C.byref(batch), 32, 0)
            if received > 0:
                for item in batch[:received]:
                    frame = item.frame
                    raw = bytes(frame.data[: frame.length])
                    if (
                        frame.eff
                        and frame.brs
                        and frame.can_id == can_id(3, args.node_id, args.source_id)
                    ):
                        report["response"] = decode_response(
                            frame.can_id, raw, args.source_id, args.node_id, 1
                        )
                        report["success"] = True
                        return_code = 0
                        raise StopIteration
            time.sleep(0.001)
        raise TimeoutError("GET_CAPS response timeout")
    except StopIteration:
        pass
    except BaseException as exc:
        report["failure"] = repr(exc)
    finally:
        if channel:
            try:
                dll.ZCAN_ResetCAN(channel)
            except Exception:
                pass
        dll.ZCAN_CloseDevice(device)
        (args.out / "run.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2, ensure_ascii=False))
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
