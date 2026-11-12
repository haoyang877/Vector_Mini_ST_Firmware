"""读写 AT32F456 电机的 CAN FD 仲裁及数据速率，保存后重启生效。"""

import argparse
import os
import secrets
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "host_app"))
from ports.controlcanfd import ControlCanFd
from upgrade_client import decode_reply, encode_frames

READ = bytes.fromhex("71 00 00 00")
REPLY = struct.Struct("<HHBBBBIII")
NOMINAL = (500, 1000)
DATA = (500, 1000, 2000, 3000, 4000, 5000)


def request(bus, node, kind, payload):
    """在当前波特率发送，并检查超时、响应身份、序号及 CRC。"""
    seq = secrets.randbelow(65536)
    for identifier, frame in encode_frames(node, kind, payload, seq):
        bus.send(identifier, frame)
    deadline = time.monotonic() + 1
    while (remaining := deadline - time.monotonic()) > 0:
        frame = bus.receive(remaining)
        if frame is None:
            break
        try:
            return decode_reply(*frame, node, kind, seq)
        except ValueError:
            continue
    raise TimeoutError(f"节点 {node} 命令 {kind} 等待应答超时")


def parameter(bus, node, kind, payload):
    """解码参数服务响应，错误回复只有四字节。"""
    data = request(bus, node, kind, payload)
    if len(data) < 4:
        raise ValueError(f"命令 {kind} 回复过短")
    status, detail = struct.unpack_from("<HH", data)
    if status or detail:
        raise RuntimeError(f"节点 {node} 命令 {kind} 返回 result={status} detail={detail}")
    if len(data) != REPLY.size:
        raise ValueError(f"命令 {kind} 响应长度错误")
    _, _, item, value_type, access, flags, value, revision, saved_revision = REPLY.unpack(data)
    if kind != 122 and (item, value_type, access) != (0x71, 2, 3):
        raise ValueError(f"命令 {kind} 参数描述不匹配")
    return value, revision, saved_revision, flags


def configure(bus, node, target=None):
    """读取活动配置，写入时按实际 revision 保存，再从 Flash 核对。"""
    value, revision, saved_revision, _ = parameter(bus, node, 120, READ)
    if target is None:
        print(f"节点 {node} RAM {value >> 16}/{value & 0xFFFF} kbit/s，revision={revision}")

    else:
        value = target[0] << 16 | target[1]
        payload = struct.pack("<BBBBII", 0x71, 2, 4, 0, revision, value)
        written, revision, _, _ = parameter(bus, node, 121, payload)
        if written != value:
            raise RuntimeError("RAM 写入回读不匹配")
        _, active, saved_revision, flags = parameter(bus, node, 122, struct.pack("<I", revision))
        if (active, saved_revision, flags & 7) != (revision, revision, 1):
            raise RuntimeError("Flash 保存应答的版本或状态不匹配")
    saved, _, stored_revision, flags = parameter(bus, node, 123, READ)
    if stored_revision != saved_revision or (
        target is not None and (not flags & 1 or saved != value)
    ):
        raise RuntimeError("Flash 参数回读不匹配")
    print(f"节点 {node} Flash {saved >> 16}/{saved & 0xFFFF} kbit/s，保存版本={stored_revision}")
    if target:
        print("已保存。共线节点全部回读成功后统一重启，并按新速率重开适配器。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dll", required=True, type=Path, help="ControlCANFD 厂商 DLL 路径")
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--node", type=int, required=True, choices=range(1, 255), metavar="1..254")
    parser.add_argument(
        "--current",
        nargs=2,
        type=int,
        default=(500, 2000),
        metavar=("ARB", "DATA"),
        help="适配器当前仲裁和数据速率，单位 kbit/s，默认 500 2000",
    )
    parser.add_argument(
        "--set",
        dest="target",
        nargs=2,
        type=int,
        metavar=("ARB", "DATA"),
        help="写入并保存目标速率，单位 kbit/s。省略时只读",
    )
    args = parser.parse_args()
    for pair in (args.current, args.target):
        if pair and (pair[0] not in NOMINAL or pair[1] not in DATA):
            parser.error(f"支持的仲裁速率为 {NOMINAL}，数据速率为 {DATA}，单位 kbit/s")
    dll = args.dll.resolve()
    try:
        with (
            os.add_dll_directory(str(dll.parent)),
            ControlCanFd(
                dll,
                args.device_index,
                nominal_bps=args.current[0] * 1000,
                data_bps=args.current[1] * 1000,
            ) as bus,
        ):
            configure(bus, args.node, args.target)
    except (OSError, RuntimeError, ValueError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    main()
