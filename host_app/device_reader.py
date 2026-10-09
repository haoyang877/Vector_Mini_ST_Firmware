"""Qt 只读 CAN FD 辅助进程；仅显式启动时打开现有 ControlCANFD 端口。"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import time
from pathlib import Path
from typing import TextIO

from ports.controlcanfd import ControlCanFd
from upgrade_client import CanTransport, decode_reply, encode_frames

READ_TYPES = {1, 2, 3, 108}
REPLY_LENGTHS = {1: 32, 2: 16, 3: 16, 108: 45}


def observe_once(
    port: CanTransport, node: int, kind: int, sequence: int, page: int = 0
) -> tuple[int, bytes]:
    """发送一条 GET 并在 250ms 内返回严格关联的完整原始帧；绝不重试写命令。"""
    if not 1 <= node <= 254 or kind not in READ_TYPES or not 1 <= sequence <= 65535:
        raise ValueError("仅允许单播节点 1..254 的 Type1/2/3/108 非零序号查询")
    if (kind == 1 and not 0 <= page <= 2) or (kind != 1 and page != 0):
        raise ValueError("页号不属于该只读查询")
    payload = struct.pack("<IIHH", 0, sequence, page, 0) if kind in (1, 2) else b""
    request = encode_frames(node, kind, payload, sequence)
    if len(request) != 1:
        raise ValueError("只读查询意外分片")
    port.send(*request[0])
    deadline = time.monotonic() + 0.25
    while time.monotonic() < deadline:
        frame = port.receive(max(0.0, deadline - time.monotonic()))
        if frame is None:
            break
        try:
            reply = decode_reply(*frame, node, kind, sequence)
        except ValueError:
            continue
        if len(reply) != REPLY_LENGTHS[kind]:
            raise ValueError(f"Type{kind} 回复长度错误")
        return frame
    raise TimeoutError(f"Type{kind} 查询超时；断开并使页面读数过期")


def serve(port: CanTransport, node: int, commands: TextIO, responses: TextIO) -> None:
    """stdin 每行仅接收 GET 指令，stdout 每行仅返回一条已校验的原始回复。"""
    print(json.dumps({"event": "ready", "node": node}), file=responses, flush=True)
    for line in commands:
        request = json.loads(line)
        if not isinstance(request, dict) or set(request) != {"type", "sequence", "page"}:
            raise ValueError("辅助进程收到非只读请求格式")
        kind, sequence, page = (request[key] for key in ("type", "sequence", "page"))
        if any(type(value) is not int for value in (kind, sequence, page)):
            raise ValueError("查询参数必须为整数")
        identifier, data = observe_once(port, node, kind, sequence, page)
        print(
            json.dumps(
                {
                    "event": "frame",
                    "type": kind,
                    "sequence": sequence,
                    "can_id": identifier,
                    "data": data.hex(),
                }
            ),
            file=responses,
            flush=True,
        )


def main() -> int:
    """仅由用户在 Qt 连接页明确选择 Python 与 SDK DLL 后启动。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dll", type=Path, required=True)
    parser.add_argument("--node", type=int, required=True)
    args = parser.parse_args()
    if not 1 <= args.node <= 254:
        parser.error("目标节点必须为 1..254")
    try:
        with ControlCanFd(args.dll) as port:
            serve(port, args.node, sys.stdin, sys.stdout)
        return 0
    except (OSError, TimeoutError, ValueError, json.JSONDecodeError) as error:
        print(json.dumps({"event": "error", "message": str(error)}), flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
