"""CAN标定票据和速度会话；调用方独占端口，本模块不打开设备或修改控制参数。"""

import csv
import math
import queue
import struct
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any, TextIO

from hardware_check import COUNTER_DB, Link, Rejected, reserve_counters
from ports.controlcanfd import ControlCanFd

JOB = struct.Struct("<HHIHBB")
STAGES = {
    0x1000: "准备",
    0x1001: "编码器/Hall LUT",
    0x1002: "电角零位",
    0x1003: "机械零位",
    0x1004: "联合电角零位",
    0x1010: "等待保存",
    0x1011: "已保存，等待完成",
}


def read_device_status(bus, node: int, expected_uid: str) -> dict:
    """读取实际版本和运行状态；身份不匹配时不沿用先前设备的数据。"""
    link = Link(bus, node, timeout_s=0.5)
    info = link.request(1, page=0)
    identity = link.request(1, page=1)
    if len(info) != 32 or info[:12].hex() != expected_uid:
        raise ValueError("设备UID已改变，请重新识别")
    if len(identity) != 32 or identity[21] != node or identity[22] not in (1, 2):
        raise ValueError("设备身份页无效")
    result = dict(
        node=node,
        uid=expected_uid,
        app_version=int.from_bytes(info[20:24], "little"),
        loader_version=int.from_bytes(info[24:28], "little"),
        role=identity[22],
        boot_state=identity[23],
        checked_at=time.time(),
    )
    if identity[22] == 1:
        status = link.status()
        result.update(
            state=status[43],
            mode=status[44],
            faults=int.from_bytes(status[4:8], "little"),
            **status_measurements(status),
        )
    return result


def status_measurements(status: bytes) -> dict:
    """解析108中的母线电压、电流和电机NTC，缺失有效位/无效哨兵均保留为空。"""
    flags = int.from_bytes(status[24:26], "little")
    values = {}
    for key, offset, bit, scale in (
        ("bus_voltage_v", 26, 4, 100),
        ("bus_current_a", 28, 5, 1000),
        ("motor_temperature_c", 36, 9, 100),
    ):
        raw = struct.unpack_from("<h", status, offset)[0]
        values[key] = raw / scale if flags & (1 << bit) and raw != -32768 else None
    return values


def speed_feedback(status: bytes, target: float, elapsed_s: float) -> dict:
    """按108有效位解码六路参考/反馈；无效哨兵不进入曲线和CSV数值列。"""
    flags = int.from_bytes(status[24:26], "little")
    row = dict(
        elapsed_s=elapsed_s,
        target_rad_s=target,
        valid_bits=flags,
        state=status[43],
        mode=status[44],
        faults=int.from_bytes(status[4:8], "little"),
        **status_measurements(status),
    )
    for key, offset, bit, fmt in (
        ("reference_position_rad", 8, 0, "i"),
        ("position_rad", 12, 1, "i"),
        ("reference_speed_rad_s", 16, 2, "i"),
        ("actual_rad_s", 20, 3, "i"),
        ("reference_iq_a", 30, 6, "h"),
        ("iq_a", 32, 7, "h"),
    ):
        raw = struct.unpack_from("<" + fmt, status, offset)[0]
        sentinel = -2147483648 if fmt == "i" else -32768
        row[key] = raw / 1000 if flags & (1 << bit) and raw != sentinel else None
    return row


def describe_faults(bits: int) -> str:
    """按APP线路位解析类别；共享位不推断唯一内部故障，保留未知位。"""
    names = []
    known = 0
    for bit, name in (
        (0, "过流"),
        (1, "过压"),
        (2, "欠压"),
        (4, "MCU过温"),
        (5, "编码器故障"),
        (6, "CAN断连"),
        (9, "参数/极对数/编码器未标定"),
        (12, "温度传感器或其他未分类故障"),
        (14, "电流校零/角度标定故障"),
        (15, "电气辨识故障"),
    ):
        known |= 1 << bit
        if bits & (1 << bit):
            names.append(name)
    if bits & ~known:
        names.append(f"未知故障位0x{bits & ~known:08X}")
    return "、".join(names) if names else "无故障"


def fault_snapshot(data: bytes) -> dict:
    bits = int.from_bytes(data[4:8], "little")
    return dict(
        state=data[43], mode=data[44], faults=bits, fault_text=describe_faults(bits), raw=data.hex()
    )


def run_clear_fault(bus, node, expected_uid, *, stopped=lambda: False, progress=lambda _: None):
    """显式清错只发送一次107；身份、静止状态及108回读决定结果，不启相。"""
    link = Link(bus, node, timeout_s=0.5)
    report = dict(profile="clear_fault", node=node, passed=False, cleared=False, clear_sent=False)
    try:
        info = link.request(1, page=0)
        if len(info) != 32 or info[:12].hex() != expected_uid:
            report["identity_changed"] = True
            raise ValueError("当前节点UID已改变，请重新扫描识别；未发送清除命令")
        report["uid"] = info[:12].hex()
        identity = link.request(1, page=1)
        if len(identity) != 32 or identity[21:23] != bytes((node, 1)):
            raise ValueError("所选节点未运行APP；未发送清除命令")
        before = report["before"] = fault_snapshot(link.status())
        if before["state"] not in (5, 8) or before["mode"] != 0:
            raise ValueError("请先停止电机并结束标定，再清除故障；未发送清除命令")
        if stopped():
            report["cancelled"] = True
            return report
        if before["faults"] == 0 and before["state"] == 5:
            report.update(after=before, passed=True, cleared=True)
            return report
        progress("正在清除故障，随后回读确认；不会启动电机")
        report["clear_sent"] = True
        try:
            reply = link.request(107)
            if len(reply) != 4:
                raise ValueError("Type107清除应答长度错误")
            result, detail = struct.unpack("<HH", reply)
            report.update(result=result, detail=detail)
            if result not in (0, 1):
                report["fatal"] = f"Type107拒绝清除，result={result} detail={detail}"
        except (TimeoutError, ValueError) as exc:
            report["fatal"] = str(exc)
        deadline = time.monotonic() + 2
        while not stopped() and time.monotonic() < deadline:
            try:
                after = report["after"] = fault_snapshot(link.status())
                report["cleared"] = (
                    after["faults"] == 0 and after["state"] == 5 and after["mode"] == 0
                )
                if report["cleared"] or "fatal" in report:
                    break
            except TimeoutError:
                pass
            time.sleep(0.05)
        report["cancelled"] = stopped()
        report["passed"] = report["cleared"] and "fatal" not in report
        if not report["cleared"] and "fatal" not in report:
            report["fatal"] = "清除未确认：故障源仍存在或尚未恢复空闲，请检查电机/编码器连接"
    except (OSError, ValueError, RuntimeError, TimeoutError) as exc:
        report["fatal"] = str(exc)
    finally:
        report["communication_timeouts"] = link.timeouts
        report["reply_timeout_ms"] = 500
    return report


def speed_value(text: str) -> float:
    """解析电机轴rad/s，按线路有符号毫弧度范围限制外部输入。"""
    try:
        value = float(text)
    except ValueError:
        raise ValueError("目标速度须填写有效数值，单位为电机轴rad/s") from None
    if not math.isfinite(value) or abs(value) > 2147483.647:
        raise ValueError("目标速度须为有限数值且在线路表示范围内")
    return value


class TraceBus:
    """原始帧流式写盘，长期速度会话不在内存累积报文。"""

    def __init__(self, bus: ControlCanFd, stream: TextIO) -> None:
        self.bus = bus
        self.writer = csv.writer(stream)
        self.writer.writerow(("host_s", "direction", "can_id", "wire_hex"))

    def send(self, identifier: int, data: bytes) -> None:
        self.writer.writerow((time.monotonic(), "tx", hex(identifier), data.hex()))
        self.bus.send(identifier, data)

    def receive(self, timeout: float) -> tuple[int, bytes] | None:
        frame = self.bus.receive(timeout)
        if frame:
            self.writer.writerow((time.monotonic(), "rx", hex(frame[0]), frame[1].hex()))
        return frame


class MotionCounters:
    """分别补充UID预留的续租/目标号，续租补块不消耗目标号的半圈窗口。"""

    def __init__(self, path: Path, uid: str, previous: int | None, target: int | None) -> None:
        self.path, self.uid = path, uid
        self.lease_value, self.target_value = reserve_counters(path, uid, previous, target)
        self.leases_left, self.targets_left = 512, 16

    def lease(self) -> int:
        while True:
            if not self.leases_left:
                self.lease_value, _ = reserve_counters(
                    self.path, self.uid, None, None, lease_count=512, target_count=0
                )
                self.leases_left = 512
            self.lease_value = (self.lease_value + 1) & 0xFFFFFFFF
            self.leases_left -= 1
            if self.lease_value:
                return self.lease_value

    def target(self) -> int:
        if not self.targets_left:
            _, self.target_value = reserve_counters(
                self.path, self.uid, None, None, lease_count=0, target_count=16
            )
            self.targets_left = 16
        self.target_value = (self.target_value + 1) & 65535
        self.targets_left -= 1
        return self.target_value


def _scalar(link, identifier, *, saved=False):
    data = link.request(123 if saved else 120, bytes((identifier, 0, 0, 0)))
    if len(data) != 20 or data[:4] != bytes(4) or data[4] != identifier or data[5] not in (1, 2):
        raise Rejected(f"参数0x{identifier:02X}回读失败")
    return {
        "value": int.from_bytes(data[8:12], "little", signed=data[5] == 1),
        "revision": int.from_bytes(data[12:16], "little"),
        "saved_revision": int.from_bytes(data[16:20], "little"),
    }


def _speed_limit(link, requested, report):
    current = _scalar(link, 1)
    if requested is None:
        return current["value"] / 1000
    requested = speed_value(str(requested))
    if requested < 0.001:
        raise ValueError("速度上限至少为0.001 rad/s")
    raw = math.floor(requested * 1000 + 0.5)
    report["requested_speed_limit_rad_s"] = raw / 1000
    if raw != current["value"]:
        data = link.request(121, struct.pack("<4BIi", 1, 1, 4, 0, current["revision"], raw))
        if len(data) != 20 or data[:6] != b"\0\0\0\0\x01\x01":
            detail = struct.unpack_from("<HH", data) if len(data) >= 4 else None
            raise Rejected(f"速度上限写入被拒绝：{detail}；请核对板端轴配置包络")
        current = _scalar(link, 1)
        if current["value"] != raw:
            raise Rejected("速度上限写入后回读不匹配，未使能")
    report["speed_limit_revision"] = current["revision"]
    return current["value"] / 1000


def _identify(link, report, feature):
    info = link.request(1, page=0)
    identity = link.request(1, page=1)
    caps = link.request(2, page=0)
    if len(info) != 32 or info[:12] in (bytes(12), b"\xff" * 12):
        raise Rejected("芯片身份无效")
    if len(identity) != 32 or identity[21] != link.node or identity[22] != 1:
        raise Rejected("所选节点未运行APP")
    if len(caps) != 16 or not int.from_bytes(caps[:4], "little") & (1 << feature):
        raise Rejected("固件未声明所选操作的能力")
    report.update(
        uid=info[:12].hex(),
        app_version=int.from_bytes(info[20:24], "little"),
        loader_version=int.from_bytes(info[24:28], "little"),
        role=1,
        boot_state=identity[23],
    )
    status = link.status()
    if status[4:8] != bytes(4) or status[43:] != bytes((5, 0)):
        report["initial_status"] = fault_snapshot(status)
        bits = report["initial_status"]["faults"]
        raise Rejected(
            f"操作需要IDLE、模式NONE及零故障；当前{describe_faults(bits)}（0x{bits:08X}）"
        )
    return caps, status


def _stop(link, report):
    report["stop_confirmed"] = False
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        try:
            code = link.command(110)
            status = link.status()
            if code == 0 and status[43] in (5, 8) and status[44] == 0:
                report["final_status"] = dict(
                    state=status[43], mode=status[44], faults=int.from_bytes(status[4:8], "little")
                )
                report["stop_confirmed"] = True
                report["stop_faults"] = int.from_bytes(status[4:8], "little")
                if report["stop_faults"]:
                    report.update(
                        passed=False,
                        fatal=f"已关相，{describe_faults(report['stop_faults'])}"
                        f"（0x{report['stop_faults']:08X}）",
                    )
                return
        except (TimeoutError, OSError, ValueError, Rejected):
            pass
        time.sleep(0.02)
    raise TimeoutError("STOP后未确认实际关相")


def _job(link, ticket, expected_kind, report):
    for attempt in range(3):
        try:
            data = link.request(161, struct.pack("<I", ticket))
            break
        except TimeoutError:
            if attempt == 2:
                raise
            # Flash保存可能暂停前台；只重查同一票据，保留每次超时。
            report["job_status_retries"] = report.get("job_status_retries", 0) + 1
            time.sleep(0.1)
    if len(data) != JOB.size:
        raise Rejected("标定票据回复长度错误")
    result, detail, current, kind, phase, effects = JOB.unpack(data)
    if result or current != ticket or kind != expected_kind:
        raise Rejected("标定票据已失效或与本次任务不符")
    return dict(ticket=current, kind=kind, phase=phase, effects=effects, detail=detail)


def run_angle_calibration(
    bus: ControlCanFd | TraceBus,
    node: int,
    *,
    kind: int = 1,
    stopped: Callable[[], bool] = lambda: False,
    progress: Callable[[str], None] = lambda _: None,
    on_identified: Callable[[dict], None] = lambda _: None,
) -> dict:
    """执行既有角度或电零位任务，完成和保存回读匹配才成功；未知启动或取消收敛到STOP。"""
    link = Link(bus, node, timeout_s=0.5)
    report = dict(
        profile="motor_calibration", node=node, passed=False, cancelled=False, reply_timeout_ms=500
    )
    started = False
    try:
        if kind not in (1, 2):
            raise ValueError("仅支持编码器/Hall角度及电角零位标定")
        _identify(link, report, 15)
        report["before_revision"] = _scalar(link, 0)
        on_identified(report)
        if stopped():
            report["cancelled"] = True
            return report
        for _ in range(3):
            started = True
            data = link.request(160, struct.pack("<HH", kind, 0))
            if len(data) != JOB.size:
                raise Rejected("标定启动回复长度错误")
            result, detail, ticket, actual_kind, phase, effects = JOB.unpack(data)
            if result == 1 and ticket and actual_kind == kind and phase == 1 and effects == 1:
                report.update(ticket=ticket, kind=actual_kind, phase=phase, effects=effects)
                break
            if result > 1 and detail == 5:
                # 明确拒绝的旧序号才能换号；丢ACK不重放启动。
                link.sequence = (link.sequence + 0x7FFF) & 65535
                continue
            if (result > 1 and detail == 4) or (result == 0 and phase in (2, 4, 5)):
                continue
            raise Rejected(f"标定未受理，结果{result}，明细0x{detail:04X}")
        else:
            raise Rejected("未取得新标定票据")
        deadline = time.monotonic() + 100
        cancelled = False
        while time.monotonic() < deadline:
            if stopped() and not cancelled:
                cancel = link.request(162, struct.pack("<I", ticket))
                if len(cancel) != JOB.size:
                    raise Rejected("标定取消回复长度错误")
                result, _, current, actual_kind, _, _ = JOB.unpack(cancel)
                if result not in (0, 1) or current != ticket or actual_kind != kind:
                    raise Rejected("标定取消未受理")
                cancelled = True
                report["cancel_requested"] = True
                deadline = min(deadline, time.monotonic() + 2)
            job = _job(link, ticket, kind, report)
            report.update(job)
            progress(f"票据{ticket} · {STAGES.get(job['detail'], '处理')} · 阶段{job['phase']}")
            if job["phase"] == 2 and job["effects"] == 2:
                active, saved = _scalar(link, 0), _scalar(link, 0, saved=True)
                report.update(active_revision=active, saved_revision=saved)
                if not active["revision"] or active["revision"] != saved["saved_revision"]:
                    raise Rejected("标定终态完成，但Flash保存修订回读不匹配")
                report["passed"] = True
                break
            if job["phase"] == 4:
                report["cancelled"] = True
                break
            if job["phase"] not in (1, 3):
                raise Rejected(f"标定失败，阶段{job['phase']}，明细0x{job['detail']:04X}")
            time.sleep(0.1)
        else:
            raise TimeoutError("标定或取消超过期限")
    except (TimeoutError, OSError, ValueError, Rejected) as exc:
        report["fatal"] = str(exc)
    finally:
        if started:
            try:
                _stop(link, report)
            except (TimeoutError, OSError, ValueError, Rejected) as exc:
                report.update(passed=False, fatal=str(exc))
        report["communication_timeouts"] = link.timeouts
        report["communication_reliability_passed"] = not bool(link.timeouts)
    return report


def run_speed_control(
    bus: ControlCanFd | TraceBus,
    node: int,
    target: float,
    travel_deg: float | None,
    commands: Any,
    out: Path,
    *,
    previous: int | None = 0,
    previous_target: int | None = 0,
    counter_db: Path = COUNTER_DB,
    speed_limit: float | None = None,
    stopped: Callable[[], bool] = lambda: False,
    progress: Callable[[str], None] = lambda _: None,
    feedback: Callable[[dict], None] = lambda _: None,
) -> dict:
    """保持模式2直到停止；目标队列、续租与反馈同线程串行，异常不恢复运动。"""
    link = Link(bus, node)
    report = dict(
        profile="motor_speed",
        node=node,
        passed=False,
        samples=0,
        updates=0,
        reply_timeout_ms=50,
        enable_attempted=False,
    )
    started = False
    try:
        target = speed_value(str(target))
        if travel_deg is not None and (not math.isfinite(travel_deg) or travel_deg <= 0):
            raise ValueError("两侧可用输出行程须为正数")
        caps, status = _identify(link, report, 13)
        if not int.from_bytes(caps[4:6], "little") & (1 << 2):
            raise Rejected("固件不支持速度闭环模式2")
        active_limit = _scalar(link, 1)["value"] / 1000
        limit = active_limit
        if speed_limit is not None:
            limit = speed_value(str(speed_limit))
        report["current_limit_a"] = _scalar(link, 2)["value"] / 1000
        geometry = link.request(126)
        if len(geometry) != 32 or geometry[:6] != b"\0\0\0\0\x01\0":
            raise Rejected("减速比回读无效")
        ratio = struct.unpack_from("<i", geometry, 24)[0] / 1000
        if limit <= 0 or ratio < 1 or not status[24] & 2:
            raise Rejected("速度限额、减速比或机械位置无效")
        report.update(speed_limit_rad_s=active_limit, gear_ratio=ratio)
        if abs(target) > limit:
            raise ValueError(f"目标速度超出板端限值±{limit:g} rad/s")
        if stopped():
            report["cancelled"] = True
            return report
        if speed_limit is not None and not int.from_bytes(caps[:4], "little") & (1 << 14):
            raise Rejected("固件未声明速度参数配置能力")
        limit = _speed_limit(link, speed_limit, report)
        report["speed_limit_rad_s"] = limit
        counters = MotionCounters(counter_db, report["uid"], previous, previous_target)
        origin = struct.unpack_from("<i", status, 12)[0] / 1000
        if stopped():
            report["cancelled"] = True
            return report
        started = True
        report["enable_attempted"] = True
        link.command(117, bytes((2, 0, 0, 0)))
        deadline = time.monotonic() + 0.25
        while True:
            status = link.status()
            if status[43:] == bytes((6, 2)) and status[4:8] == bytes(4):
                break
            if stopped() or time.monotonic() >= deadline or status[4:8] != bytes(4):
                raise Rejected("速度使能后未进入运行态")
            time.sleep(0.005)

        def set_target(value):
            nonlocal target
            value = speed_value(str(value))
            if abs(value) > limit:
                raise ValueError(f"目标速度超出板端限值±{limit:g} rad/s")
            magnitude = math.floor(abs(value) * 1000 + 0.5)
            scaled = -magnitude if value < 0 else magnitude
            link.command(116, struct.pack("<B3x4i", 2, 0, scaled, 0, 0), sequence=counters.target())
            target = value
            report["updates"] += 1
            progress(f"速度模式已使能 · 电机轴目标{target:g} rad/s")

        set_target(target)
        collection_started = time.monotonic()
        with (out / "speed.csv").open("w", newline="", encoding="utf-8") as stream:
            columns = (
                "host_s",
                "elapsed_s",
                "target_rad_s",
                "reference_position_rad",
                "position_rad",
                "reference_speed_rad_s",
                "actual_rad_s",
                "reference_iq_a",
                "iq_a",
                "valid_bits",
                "state",
                "mode",
                "faults",
                "bus_voltage_v",
                "bus_current_a",
                "motor_temperature_c",
            )
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            next_flush = time.monotonic() + 0.25
            while not stopped():
                link.command(118, struct.pack("<I", counters.lease()))
                try:
                    value = commands.get_nowait()
                except queue.Empty:
                    value = None
                if value is not None and not stopped():
                    set_target(value)
                status = link.status()
                flags = int.from_bytes(status[24:26], "little")
                if status[4:8] != bytes(4) or status[43:] != bytes((6, 2)) or flags & 10 != 10:
                    raise Rejected("速度反馈失效、故障或运行模式改变")
                position = struct.unpack_from("<i", status, 12)[0] / 1000
                if travel_deg is not None and abs(position - origin) / ratio > math.radians(
                    travel_deg
                ):
                    raise Rejected("速度运动到达本次可用输出行程")
                now = time.monotonic()
                row = speed_feedback(status, target, now - collection_started)
                if row["actual_rad_s"] is None or row["position_rad"] is None:
                    raise Rejected("速度反馈含无效位置或速度")
                writer.writerow(dict(host_s=now, **row))
                if now >= next_flush:
                    stream.flush()
                    next_flush = now + 0.25
                report["samples"] += 1
                report["last_feedback"] = row
                feedback(row)
                time.sleep(0.05)
        report["passed"] = True
    except (TimeoutError, OSError, ValueError, Rejected) as exc:
        report["fatal"] = str(exc)
    finally:
        if started:
            try:
                _stop(link, report)
            except (TimeoutError, OSError, ValueError, Rejected) as exc:
                report.update(passed=False, fatal=str(exc))
        report["communication_timeouts"] = link.timeouts
        report["communication_reliability_passed"] = not bool(link.timeouts)
    return report
