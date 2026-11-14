"""CAN 标定任务。仅设备返回本票据的完成及已保存终态才通过。"""

import struct
import time

CALIBRATIONS = {1: "编码器 LUT", 2: "电气零位", 6: "机械零位"}
STAGES = {
    0x1000: "准备标定",
    0x1001: "采集编码器 LUT",
    0x1002: "对齐电气零位",
    0x1003: "设置机械零位",
    0x1004: "联合电零位标定",
    0x1010: "等待保存",
    0x1011: "已保存，等待标定结束",
}
STATUS = struct.Struct("<HHI4iH8h3B")
JOB = struct.Struct("<HHIHBB")


class CalibrationCancelled(Exception):
    """操作员停止且设备已确认取消，或尚未发起标定。"""


def idle(client):
    fields = STATUS.unpack(client.request(108))
    if fields[0] or fields[-3:] != (client.node, 5, 0) or fields[2]:
        raise RuntimeError(f"需要 IDLE、零故障，当前状态 {fields[-2]}，故障 0x{fields[2]:08x}")


def start_ticket(client, kind):
    payload = struct.pack("<HH", kind, 0)
    # 固件以最近启动序号的前半圈判断新旧。仅在明确拒绝或命中已结束的旧票据时换序号。
    for _ in range(3):
        result, detail, ticket, actual, phase, effects = JOB.unpack(client.request(160, payload))
        if result == 1 and actual == kind and ticket and phase == 1:
            return ticket
        if result == 0 and actual == kind and ticket and client.trace[-1]["attempt"] > 0:
            if phase == 1:
                return ticket
            if phase in (2, 4, 5):
                raise RuntimeError(f"启动应答丢失，票据 {ticket} 已结束，无法确认本次标定")
        if result > 1 and detail == 5:
            client.sequence = (client.sequence + 0x7FFF) & 0xFFFF
        elif not ((result > 1 and detail == 4) or (result == 0 and phase in (2, 4, 5))):
            break
    raise RuntimeError(f"设备未接受新标定，状态 {result}，明细 0x{detail:04x}")


def run_job(client, kind, *, stopped=lambda: False, log=print):
    """跟踪本次票据直到固件确认保存。启动应答丢失或中断时同样请求停机。"""
    idle(client)
    if stopped():
        raise CalibrationCancelled("标定已停止")
    try:
        ticket = start_ticket(client, kind)
        log(f"{CALIBRATIONS[kind]} · 票据 {ticket}")
        cancel_requested = False
        last_detail = None
        deadline = time.monotonic() + 100
        while time.monotonic() < deadline:
            time.sleep(0.2)
            command = 161
            if stopped() and not cancel_requested:
                command = 162
                cancel_requested = True
                log("已请求取消，等待电机关相")
            status, detail, current, actual, phase, effects = JOB.unpack(
                client.request(command, struct.pack("<I", ticket))
            )
            accepted = status == 0 or (command == 162 and status == 1)
            if not accepted or current != ticket or actual != kind:
                raise RuntimeError(f"标定票据不符，状态 {status}，明细 0x{detail:04x}")
            if phase == 2 and effects == 2:
                return ticket
            if phase == 4 and cancel_requested:
                raise CalibrationCancelled("标定已取消")
            if phase == 3 and cancel_requested:
                continue
            if phase != 1:
                reason = {7: "设备执行失败", 8: "Flash 保存失败"}.get(
                    detail, "标定未确认完成并保存"
                )
                raise RuntimeError(
                    f"{reason}，阶段 {phase}，保存标志 {effects}，明细 0x{detail:04X}"
                )
            if last_detail != detail:
                log(STAGES.get(detail, f"标定中（0x{detail:04x}）"))
                last_detail = detail
        raise TimeoutError("标定超过 100 秒")
    finally:
        result, detail = struct.unpack("<HH", client.request(110))
        if result not in (0, 1):
            raise RuntimeError(f"停机被拒，状态 {result}，明细 {detail}")
        deadline = time.monotonic() + 1
        while True:
            f = STATUS.unpack(client.request(108))
            if f[0] == 0 and f[-3] == client.node and f[-2] in (5, 8) and f[-1] == 0:
                break
            if time.monotonic() > deadline:
                raise RuntimeError("停机未确认，请检查工装")
            time.sleep(0.03)
