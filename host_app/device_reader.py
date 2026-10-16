"""Qt CAN FD 辅助进程；仅显式启动时打开端口，控制限固定速度和 STOP。"""

from __future__ import annotations

import argparse
import json
import math
import struct
import sys
import time
from pathlib import Path
from typing import TextIO

from ports.controlcanfd import ControlCanFd
from upgrade_client import CanTransport, decode_reply, encode_frames

READ_TYPES = {1, 2, 3, 108}
REPLY_LENGTHS = {1: 32, 2: 16, 3: 16, 108: 45}
CONTROL_TYPES = {"start_speed": 117, "set_speed": 116, "stop": 110}
RESULTS = {0, 1, 2, 3, 4, 5, 6, 7, 8, 11, 17}


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


def control_once(
    port: CanTransport, node: int, operation: str, sequence: int, rpm: int = 0
) -> tuple[int, int]:
    """只发送固定模式的117/116/110；结果是受理/执行回复，不能代替108状态。"""
    if not 1 <= node <= 254 or operation not in CONTROL_TYPES or not 1 <= sequence <= 65535:
        raise ValueError("控制类型、节点或序号非法")
    if type(rpm) is not int or not 0 <= rpm <= 300 or (operation != "set_speed" and rpm != 0):
        raise ValueError("本轮架空轮电机只允许0..300 rpm速度目标")
    kind = CONTROL_TYPES[operation]
    payload = (
        b"\x02\x00\x00\x00"
        if operation == "start_speed"
        else struct.pack("<BBHiiii", 2, 0, 0, 0, round(rpm * math.pi * 1000 / 30), 0, 0)
        if operation == "set_speed"
        else b""
    )
    identifier, data = encode_frames(node, kind, payload, sequence)[0]
    port.send(identifier, data)
    deadline = time.monotonic() + 0.25
    while time.monotonic() < deadline:
        frame = port.receive(max(0.0, deadline - time.monotonic()))
        if frame is None:
            break
        try:
            reply = decode_reply(*frame, node, kind, sequence)
        except ValueError:
            continue
        if len(reply) != 4:
            raise ValueError(f"Type{kind} 控制回复长度错误")
        result, detail = struct.unpack("<HH", reply)
        if result not in RESULTS:
            raise ValueError(f"Type{kind} 未知业务结果")
        return result, detail
    raise TimeoutError(f"Type{kind} 控制回复超时；必须人工确认STOP及关相")


def serve(port: CanTransport, node: int, commands: TextIO, responses: TextIO) -> None:
    """stdin 仅接收固定GET或速度/STOP指令；退出活动会话前尽力发送STOP。"""
    print(json.dumps({"event": "ready", "node": node}), file=responses, flush=True)
    active = False
    last_control_sequence = 0
    try:
        for line in commands:
            request = json.loads(line)
            if not isinstance(request, dict):
                raise ValueError("辅助进程收到非法请求")
            if set(request) == {"type", "sequence", "page"}:
                kind, sequence, page = (request[key] for key in ("type", "sequence", "page"))
                if any(type(value) is not int for value in (kind, sequence, page)):
                    raise ValueError("查询参数必须为整数")
                identifier, data = observe_once(port, node, kind, sequence, page)
                message = {
                    "event": "frame",
                    "type": kind,
                    "sequence": sequence,
                    "can_id": identifier,
                    "data": data.hex(),
                }
            elif set(request) == {"op", "sequence", "rpm"}:
                operation, sequence, rpm = (request[key] for key in ("op", "sequence", "rpm"))
                if (
                    type(sequence) is not int
                    or type(rpm) is not int
                    or not isinstance(operation, str)
                ):
                    raise ValueError("控制参数格式非法")
                if operation == "start_speed":
                    active = True
                last_control_sequence = sequence
                result, detail = control_once(port, node, operation, sequence, rpm)
                if operation == "start_speed" and result not in (0, 1):
                    active = False
                if operation == "stop" and result == 0 and detail == 0:
                    active = False
                message = {
                    "event": "result",
                    "type": CONTROL_TYPES[operation],
                    "sequence": sequence,
                    "result": result,
                    "detail": detail,
                }
            else:
                raise ValueError("辅助进程收到未开放的请求格式")
            print(json.dumps(message), file=responses, flush=True)
    finally:
        if active:
            try:
                sequence = 1 if last_control_sequence == 65535 else last_control_sequence + 1
                result, detail = control_once(port, node, "stop", sequence)
                print(
                    json.dumps({"event": "exit_stop", "result": result, "detail": detail}),
                    file=responses,
                    flush=True,
                )
            except (OSError, ValueError, TimeoutError) as error:
                print(
                    json.dumps({"event": "exit_stop_failed", "message": str(error)}),
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
