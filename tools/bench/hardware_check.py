"""AT32F456 硬件功能检测，默认只查询通信并启动关相采样。"""

import argparse
import json
import math
import os
import secrets
import sqlite3
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "host_app"))
from ports.controlcanfd import ControlCanFd
from upgrade_client import decode_reply, encode_frames

ITEMS = (
    "Ia",
    "Ib",
    "Ic",
    "母线电流",
    "母线电压",
    "电机 NTC",
    "电机角静态",
    "霍尔静态",
    "电机角动态",
    "霍尔动态",
)
STATES = ("未测", "正常", "异常", "未装")
STAGES = ("未开始", "静态采样", "待驱动", "驱动中", "结束", "中止", "相电阻测量")
COUNTER_DB = Path.home() / "CAN-Link-Test" / "hardware-check.sqlite3"


class Rejected(RuntimeError):
    """对端有合法业务应答，但未受理相应操作。"""


class Link:
    """单节点串行往返。忽略 CRC、地址、命令、序号不匹配的数据。"""

    def __init__(self, bus, node, *, timeout_s=0.050):
        if not 1 <= node <= 254 or node == 2:
            raise ValueError("请选择与主站 2 不同的单播节点")
        self.bus, self.node = bus, node
        self.timeout_s = timeout_s
        self.sequence = secrets.randbelow(65535)
        self.timeouts = {}
        self.status_busy_replies = 0

    def request(self, kind, payload=b"", *, page=None, sequence=None):
        if sequence is None:
            self.sequence = (self.sequence + 1) & 65535
            sequence = self.sequence
        if page is not None:
            payload = struct.pack("<IIHH", 0, sequence, page, 0)
        self.bus.send(*encode_frames(self.node, kind, payload, sequence)[0])
        deadline = time.monotonic() + self.timeout_s
        while (remaining := deadline - time.monotonic()) > 0:
            frame = self.bus.receive(remaining)
            if frame is None:
                break
            try:
                return decode_reply(*frame, self.node, kind, sequence)
            except ValueError:
                continue
        self.timeouts[kind] = self.timeouts.get(kind, 0) + 1
        raise TimeoutError(f"节点 {self.node} Type {kind} 未收到匹配应答")

    def _read(self, kind: int) -> bytes:
        """只读快照丢应答时最多重读一次；每次仍按 50ms 记录超时。"""
        try:
            return self.request(kind)
        except TimeoutError:
            return self.request(kind)

    def command(self, kind, payload=b"", *, sequence=None):
        reply = self.request(kind, payload, sequence=sequence)
        if len(reply) != 4:
            raise ValueError(f"Type {kind} 结果长度错误")
        code, detail = struct.unpack("<HH", reply)
        if code not in (0, 1):
            raise Rejected(f"Type {kind} 拒绝，result={code} detail={detail}")
        return code

    def status(self):
        for _ in range(3):
            data = self._read(108)
            if data != b"\x06\0\0\0" and not (len(data) == 45 and data[:2] == b"\x06\0"):
                break
            self.status_busy_replies += 1
        else:
            raise Rejected("状态采样忙")
        if len(data) != 45 or data[:4] != bytes(4) or data[42] != self.node:
            raise ValueError("Type 108 状态帧无效")
        return data

    def board(self, payload=b"", sequence=None):
        data = (
            self.request(127, payload, sequence=sequence)
            if payload or sequence is not None
            else self._read(127)
        )
        if len(data) == 4:
            raise Rejected("Type 127 拒绝，result=%d detail=%d" % struct.unpack("<HH", data))
        if len(data) != 21 or data[0] != bool(payload) or data[1:4] != bytes(3) or data[4] != 2:
            raise ValueError("Type 127 格式或结果不匹配")
        items = list(data[10:20])
        if data[5] >= len(STAGES) or any(
            v > (3 if i in (7, 9) else 2) for i, v in enumerate(items)
        ):
            raise ValueError("Type 127 项目或阶段值无效")
        return dict(
            run_id=int.from_bytes(data[6:10], "little"),
            stage=data[5],
            board_items=items,
            directions=data[20],
        )

    def measurement(self, item: int, run_id: int) -> dict:
        """读取已停止采集的同轮实测页，缺失值保留为None。"""
        data = self.request(127, struct.pack("<4BI", 2, 2, item, 0, run_id))
        if len(data) == 4:
            code, detail = struct.unpack("<HH", data)
            raise Rejected(f"数值页{item}拒绝，result={code} detail={detail}")
        if (
            len(data) != 32
            or data[:4] != bytes(4)
            or data[4:6] != bytes((2, item))
            or data[6] & ~15
            or data[7] != 0
            or int.from_bytes(data[8:12], "little") != run_id
        ):
            raise ValueError(f"数值页{item}格式或测试轮次不符")
        values = struct.unpack_from("<4f", data, 16)
        if any(data[6] & (1 << i) and not math.isfinite(v) for i, v in enumerate(values)):
            raise ValueError(f"数值页{item}包含非有限有效值")
        return {
            "samples": int.from_bytes(data[12:16], "little"),
            "values": [v if data[6] & (1 << i) else None for i, v in enumerate(values)],
        }

    def resistance(self, run_id: int) -> dict:
        """读取相电阻终态；有效位外的值保持缺失。"""
        data = self.request(127, struct.pack("<4BI", 2, 3, 0, 0, run_id))
        if len(data) == 4:
            raise Rejected("相电阻结果读取被拒绝")
        if (
            len(data) != 44
            or data[:5] != bytes((0, 0, 0, 0, 2))
            or int.from_bytes(data[8:12], "little") != run_id
            or data[5] > 3
            or data[14] > 10
            or data[15] & ~63
            or any(data[i] & ~7 for i in (6, 7, 12, 13))
        ):
            raise ValueError("相电阻结果格式或轮次无效")
        values = struct.unpack_from("<7f", data, 16)
        if not all(math.isfinite(v) for v in values) or values[0] < 0:
            raise ValueError("相电阻结果包含无效数值")
        return dict(
            result=data[5],
            open_phases=data[6],
            low_phases=data[7],
            high_phases=data[12],
            open_lines=data[13],
            failure=data[14],
            nominal_ohm=values[0],
            phase_ohm=[v if data[15] & (1 << i) else None for i, v in enumerate(values[1:4])],
            line_ohm=[v if data[15] & (8 << i) else None for i, v in enumerate(values[4:])],
        )


@dataclass
class Drive:
    """台架选定的限流、低电频、每段时长和起点两侧实际可用输出行程。"""

    current_a: float
    frequency_hz: float
    leg_ms: int
    travel_deg: float

    def __post_init__(self):
        if not all(math.isfinite(x) for x in (self.current_a, self.frequency_hz, self.travel_deg)):
            raise ValueError("驱动参数须为有限数值")
        if not (
            0.001 <= self.current_a <= 3
            and 0.001 <= self.frequency_hz <= 10
            and 700 <= self.leg_ms <= 3000
            and self.travel_deg > 0
        ):
            raise ValueError(
                "应急模式限流 0.001～3 A，电频 0.001～10 Hz，单程 700～3000 ms，可用行程须大于零"
            )


def reserve_counters(path, uid, previous, previous_target, *, lease_count=512, target_count=16):
    """在发送前为本轮预留序号。断电或丢应答后跳过已预留的号码。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(path)
    try:
        with connection:
            connection.execute("BEGIN IMMEDIATE")
            connection.execute(
                "CREATE TABLE IF NOT EXISTS counter(uid TEXT PRIMARY KEY, lease INTEGER, target INTEGER)"
            )
            row = connection.execute(
                "SELECT lease, target FROM counter WHERE uid=?", (uid,)
            ).fetchone()
            if row is None and (previous is None or previous_target is None):
                raise Rejected("此 UID 首次驱动需填写上次续租计数和目标序号，已确认新上电可均填 0")
            lease, target = row if row is not None else (previous, previous_target)
            if not 0 <= lease <= 0xFFFFFFFF or not 0 <= target <= 65535:
                raise Rejected("续租计数须为 u32，目标序号须为 u16")
            connection.execute(
                "INSERT OR REPLACE INTO counter VALUES(?,?,?)",
                (uid, (lease + lease_count) & 0xFFFFFFFF, (target + target_count) & 65535),
            )
            return lease, target
    finally:
        connection.close()


def direction_results(report, motor=None, hall=(0, 0)):
    """电机默认比较正/负命令；霍尔独立看双向趋势，实物参照仅作可选附加核对。"""
    values = report["board_items"].copy()
    for sensor, reference in enumerate((motor if motor is not None else (1, 2), hall)):
        measured = [(report["directions"] >> (4 * sensor + 2 * leg)) & 3 for leg in (0, 1)]
        index = 8 + sensor
        if values[index] in (2, 3):
            continue
        if sensor == 1 and reference == (0, 0):
            if report["stage"] != 4 or values[index] != 1:
                values[index] = 0
            elif measured[0] not in (1, 2) or measured[1] not in (1, 2):
                values[index] = 0
            elif measured[0] == measured[1]:
                values[index] = 2
            continue
        if any(m and r and m != r for m, r in zip(measured, reference, strict=True)):
            values[index] = 2
        elif (
            report["stage"] != 4
            or values[index] != 1
            or any(r not in (1, 2) for r in reference)
            or reference[0] == reference[1]
            or list(reference) != measured
        ):
            values[index] = 0
    if report.get("ntc_mismatch"):
        values[5] = 2
    report["observed"] = {"motor": list(motor or (0, 0)), "hall": list(hall)}
    report["direction_reference"] = {
        "motor": list(motor if motor is not None else (1, 2)),
        "hall": list(hall),
    }
    report["items"] = dict(zip(ITEMS, (STATES[v] for v in values), strict=True))
    expected = report["direction_reference"]["motor"]
    measured = [(report["directions"] >> (2 * leg)) & 3 for leg in (0, 1)]
    opposite = [
        leg
        for leg, (actual, reference) in enumerate(zip(measured, expected, strict=True))
        if actual in (1, 2) and reference in (1, 2) and actual != reference
    ]
    names = ("未取得", "正转", "反转", "无可辨位移")
    reference_names = ("未知", "正向", "反向", "无效")
    reference_source = "命令" if motor is None else "现场参照"
    legs = [
        f"{'去程' if leg == 0 else '回程'}：{reference_names[reference]}{reference_source}→编码器{names[actual]}"
        for leg, (actual, reference) in enumerate(zip(measured, expected, strict=True))
    ]
    if len(opposite) == 2:
        result, reason = "reversed", f"编码器方向反向，{reference_source}与编码器方向不一致"
    elif opposite:
        result, reason = "mismatch", "编码器方向不一致（部分行程反向）"
    elif any(actual not in (1, 2) for actual in measured):
        result, reason = "incomplete", "方向数据不足，不能确认方向关系"
    elif values[8] != 1:
        result, reason = "dynamic_failure", "方向符号一致，但动态检测未通过"
    else:
        result, reason = "consistent", f"{reference_source}与编码器方向一致"
    report["motor_direction"] = dict(
        result=result,
        reason=reason,
        expected=expected,
        measured=measured,
        opposite_legs=opposite,
        reference_source=reference_source,
        detail="；".join(legs),
    )


def read_temperature(link, report, ambient):
    if report["board_items"][5] == 0 and all(v in (1, 2) for v in report["board_items"][:4]):
        # 完整静态窗中的 0 表示至少一个有效 NTC 样本超界。108 读到的是最新温度。
        report["ntc_mismatch"] = ambient is not None and 0 <= ambient <= 60
        report["note"] = (
            "静态窗 NTC 超出 0～60 °C，已确认室温"
            if report["ntc_mismatch"]
            else "NTC 超出 0～60 °C，测试工况不满足"
        )
    status = link.status()
    if int.from_bytes(status[24:26], "little") & (1 << 9):
        temp = struct.unpack_from("<h", status, 36)[0] / 100
        report["motor_temperature_c"] = temp
        if report["board_items"][5] == 0 and not 0 <= temp <= 60:
            if ambient is not None and 0 <= ambient <= 60:
                report["ntc_mismatch"] = True
            else:
                report["note"] = "NTC 超出 0～60 °C，测试工况不满足"


def drive_geometry(link, drive):
    params = link.request(120, b"\x05\0\0\0")
    feedback = link.request(126)
    if (
        len(params) != 20
        or params[:4] != bytes(4)
        or params[4:6] != b"\x05\x02"
        or len(feedback) != 32
        or feedback[:6] != b"\0\0\0\0\x01\0"
    ):
        raise Rejected("未取得活动极对数或减速比")
    poles = struct.unpack_from("<I", params, 8)[0]
    ratio = struct.unpack_from("<i", feedback, 24)[0] / 1000
    if poles < 1 or ratio < 1:
        raise Rejected("活动极对数或减速比无效")
    # 附加一段租约超时滑行前的指令余量，并在每个往返检查实际位置。
    if 360 * drive.frequency_hz * (drive.leg_ms / 1000 + 0.5) / (poles * ratio) > drive.travel_deg:
        raise Rejected("所选电频和时长超过台架实际可用输出行程")
    return ratio


def execute_drive(link, report, drive, start_seq, refresh, stopped, lease, target_seq):
    ratio = drive_geometry(link, drive)
    before = link.status()
    report["drive_precheck"] = dict(
        faults=int.from_bytes(before[4:8], "little"), state=before[43], mode=before[44]
    )
    if before[43:] != bytes((5, 0)) or before[4:8] != bytes(4) or not before[24] & 2:
        raise Rejected(
            "驱动需要轴空闲、无保护故障和有效机械位置；"
            f"当前固件故障码0x{report['drive_precheck']['faults']:08X}，"
            "尚未验证本轮旋向"
        )
    origin = struct.unpack_from("<i", before, 12)[0]
    refresh()
    if stopped() or report["stage"] != 2:
        raise Rejected("本轮已离开待驱动阶段")
    try:
        link.command(117, bytes((10, 0, 0, 0)), sequence=start_seq)
    except (TimeoutError, OSError, ValueError):
        # 应答丢失时启动可能已经执行，继续走同一停机路径。
        confirm_stop(link, report)
        raise
    try:
        deadline = time.monotonic() + 0.25
        while True:
            if stopped():
                raise Rejected("操作员停止")
            status = link.status()
            refresh()
            if status[43:] == bytes((6, 10)) and status[4:8] == bytes(4) and report["stage"] == 3:
                break
            if (
                time.monotonic() >= deadline
                or status[4:8] != bytes(4)
                or report["stage"] not in (2, 3)
            ):
                raise Rejected("未能进入开环限流模式")
            time.sleep(0.005)
        # 正向、定角保持等待稳定、反向。118 只续同一会话。
        for frequency, duration in (
            (drive.frequency_hz, drive.leg_ms),
            (0, 500),
            (-drive.frequency_hz, drive.leg_ms),
        ):
            if stopped():
                raise Rejected("操作员停止")
            target_seq = (target_seq + 1) & 65535
            link.command(
                116,
                struct.pack(
                    "<B3x7i",
                    10,
                    0,
                    0,
                    0,
                    0,
                    round(frequency * 1000),
                    round(drive.current_a * 1000),
                    0,
                ),
                sequence=target_seq,
            )
            deadline = time.monotonic() + duration / 1000
            while True:
                if stopped():
                    raise Rejected("操作员停止")
                for attempt in range(2):
                    lease = (lease + 1) & 0xFFFFFFFF or 1
                    try:
                        link.command(118, struct.pack("<I", lease))
                        break
                    except TimeoutError:
                        if attempt:
                            raise
                refresh()
                if report["stage"] != 3:
                    raise Rejected("板端已结束驱动检测")
                status = link.status()
                offset = (struct.unpack_from("<i", status, 12)[0] - origin) * 0.001
                if (
                    status[4:8] != bytes(4)
                    or status[43:] != bytes((6, 10))
                    or not status[24] & 2
                    or abs(math.degrees(offset) / ratio) > drive.travel_deg
                ):
                    raise Rejected("轴状态、位置有效性或机械行程超出本轮条件")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                time.sleep(min(0.040, remaining))
    except Rejected as exc:
        # STOP会改变阶段；先读取拒绝时的板端报告，避免将主动保护误报为续租根因。
        try:
            refresh()
        except (Rejected, ValueError, TimeoutError, OSError):
            pass
        if report["stage"] == 5:
            failed = [
                name for name, state in zip(ITEMS, report["board_items"], strict=True) if state == 2
            ]
            if failed:
                report["board_abort_items"] = failed
                report["drive_command_error"] = str(exc)
                raise Rejected(f"板端已中止，异常项：{'、'.join(failed)}；{exc}") from exc
        raise
    finally:
        confirm_stop(link, report)


def confirm_stop(link, report):
    report["stop_confirmed"] = False
    answered = False
    for _ in range(3):
        try:
            if link.command(110) == 0:
                report["stop_confirmed"] = True
                return
            answered = True
        except Rejected:
            answered = True
        except (TimeoutError, OSError, ValueError):
            pass
        time.sleep(0.015)
    if answered:
        raise Rejected("STOP 已应答，实际关相未确认")
    raise TimeoutError("STOP 后未确认关相")


def empty_report(node, ambient=None, note=""):
    report = dict(
        profile="hardware_check",
        node=node,
        communication="异常",
        supported=None,
        run_id=None,
        stage=0,
        board_items=[0] * 10,
        directions=0,
        note=note,
        ambient_c=ambient,
    )
    direction_results(report)
    return report


def run_check(
    bus,
    node,
    *,
    drive=None,
    resistance_only=False,
    ambient=None,
    previous=None,
    previous_target=None,
    counter_db=COUNTER_DB,
    stopped=lambda: False,
    progress=lambda _: None,
    phase_progress=lambda _: None,
):
    """返回通信结论、本轮板端快照和命令方向核对结果；此处不打开设备。"""
    link = Link(bus, node, timeout_s=0.5)
    report = empty_report(node, ambient)
    report["reply_timeout_ms"] = 500
    if drive:
        report["drive"] = vars(drive).copy()
    powered = bool(drive) or resistance_only
    report["scope"] = "static_resistance" if resistance_only else "full" if drive else "static"
    active_run = None
    phases = dict.fromkeys(
        ("communication", "static", "resistance", "dynamic", "finalize"), "待执行"
    )
    if not powered:
        phases["resistance"] = "本轮不测"
    if not drive or resistance_only:
        phases["dynamic"] = "本轮不测"
    report["phase_states"] = phases

    def phase(key, state="进行中"):
        phases[key] = state
        phase_progress(phases.copy())

    def refresh():
        current = link.board()
        if current["run_id"] != active_run:
            raise Rejected("检测序号已改变")
        report.update(current)

    try:
        phase("communication")
        progress("检查主控与板端通信")
        for _ in range(3):
            try:
                info = link.request(1, page=0)
            except TimeoutError:
                continue
            if len(info) == 32 or info == b"\x06\0\0\0":
                break
        else:
            raise TimeoutError("三次 500 ms 探测均无有效应答")
        report["communication"] = "正常"
        if len(info) == 4:
            raise Rejected("设备忙，已确认通信")
        report["uid"] = info[:12].hex()
        report["app_version"] = int.from_bytes(info[20:24], "little")
        report["loader_version"] = int.from_bytes(info[24:28], "little")
        report["test_currents_a"] = [1.0, 2.0] if report["app_version"] >= 0x01080C else [0.5, 1.0]
        identity = link.request(1, page=1)
        if identity == b"\x06\0\0\0":
            raise Rejected("身份页忙")
        if len(identity) != 32 or identity[21] != node:
            raise ValueError("身份页与选择的节点不符")
        if identity[22] == 2:
            report["supported"] = False
            raise Rejected("节点当前运行 Loader")
        if identity[22] != 1:
            raise Rejected("节点未提供有效 APP 身份")
        caps = link.request(2, page=0)
        if caps == b"\x06\0\0\0":
            raise Rejected("能力页忙")
        if len(caps) != 16:
            raise ValueError("能力页长度错误")
        features, modes = struct.unpack_from("<IH", caps)
        report["supported"] = bool(features & (1 << 19))
        report["measurements_supported"] = bool(features & (1 << 20))
        report["resistance_supported"] = bool(features & (1 << 21))
        if powered and not report["resistance_supported"]:
            report["upgrade_required"] = True
            version = report["app_version"]
            actual = f"{version >> 16}.{(version >> 8) & 255}.{version & 255}"
            raise Rejected(
                f"当前设备 APP v{actual} 不支持相电阻检查；请升级到随包 APP v1.8.12。"
                "本次未启动静态、相电阻或动态检测；也可选择关相静态检查。"
            )
        if not report["supported"]:
            raise Rejected("该 APP 未提供硬件功能检测")
        if ambient is not None and (not math.isfinite(ambient) or not 0 <= ambient <= 60):
            raise Rejected("实测环境温度不在 0～60 °C，工况不满足")
        if drive and (not features & (1 << 13) or not modes & (1 << 10)):
            raise Rejected("驱动需要模式 10 和运动租约")
        phase("communication", "已完成")
        previous_run = link.board()["run_id"]
        if stopped():
            raise Rejected("操作员停止")
        phase("static")
        start = struct.pack("<4BI", 2, 1, 3 if powered else 1, 0, previous_run)
        start_seq = (link.sequence + 1) & 65535
        link.sequence = start_seq
        try:
            current = link.board(start, start_seq)
        except TimeoutError:
            current = link.board(start, start_seq)
        if current["run_id"] != previous_run + 1:
            raise Rejected("启动应答不属于预期检测轮次")
        active_run = current["run_id"]
        report.update(current)
        progress("关相采集 100 ms 静态窗口")
        deadline = time.monotonic() + 1
        while report["stage"] == 1:
            if stopped() or time.monotonic() > deadline:
                raise Rejected("静态采集已取消或超时")
            time.sleep(0.012)
            refresh()
        phase("static", "未完成" if report["stage"] == 5 else "已完成")
        if powered and report["stage"] == 6:
            phase("resistance")
            currents = " / ".join(f"{value:g}A" for value in report["test_currents_a"])
            progress(f"相电阻：AB/BC/CA 正反向 {currents} 测量")
            deadline = time.monotonic() + 27
            while report["stage"] == 6:
                if stopped() or time.monotonic() > deadline:
                    raise Rejected("相电阻测量已取消或超时")
                time.sleep(0.012)
                refresh()
            phase("resistance", "未完成" if report["stage"] == 5 else "已完成")
        if resistance_only and report["stage"] == 2:
            progress("静态和相电阻采集完成，正在停止并确认关相；不执行动态检测")
            confirm_stop(link, report)
            refresh()
        if drive and not resistance_only and report["stage"] == 2:
            if stopped():
                raise Rejected("操作员停止")
            try:
                lease, target = reserve_counters(
                    counter_db, report["uid"], previous, previous_target
                )
            except (OSError, sqlite3.Error) as exc:
                raise Rejected(f"无法保存本轮运动计数：{exc}") from exc
            phase("dynamic")
            progress("低速正向、停稳、反向")
            # 运动中保持50ms请求期限，避免一次长等待耗尽500ms板端租约。
            link.timeout_s = 0.05
            report["drive_reply_timeout_ms"] = 50
            try:
                execute_drive(link, report, drive, start_seq, refresh, stopped, lease, target)
            finally:
                link.timeout_s = 0.5
            deadline = time.monotonic() + 0.5
            while report["stage"] in (2, 3):
                if time.monotonic() > deadline:
                    raise Rejected("关相后未取得最终板端报告")
                time.sleep(0.01)
                refresh()
            phase("dynamic", "已完成" if report["stage"] == 4 else "未完成")
        if report["stage"] == 5 and not (resistance_only and report.get("stop_confirmed")):
            report["note"] = "板端中止，未完成项目保留未测"
        if report["stage"] in (4, 5):
            read_temperature(link, report, ambient)
    except (Rejected, ValueError, TimeoutError, OSError) as exc:
        for key, state in phases.copy().items():
            if state == "进行中":
                phase(key, "未完成")
        report["note"] = str(exc)
        if isinstance(exc, (TimeoutError, OSError, ValueError)):
            report["communication"] = "异常"
        if active_run is not None and report["stage"] in (2, 3):
            try:
                refresh()
            except (Rejected, ValueError, TimeoutError, OSError):
                pass
    phase("finalize")
    progress("确认关相并读取本轮结果")
    if active_run is not None and powered and not report.get("stop_confirmed"):
        try:
            confirm_stop(link, report)
            refresh()
        except (Rejected, ValueError, TimeoutError, OSError) as exc:
            report["note"] = f"{report.get('note', '')}；{exc}"
    if active_run is not None and powered and report.get("stop_confirmed"):
        try:
            report["resistance"] = link.resistance(active_run)
        except (Rejected, ValueError, TimeoutError, OSError) as exc:
            report["resistance_note"] = str(exc)
    report["measurements"] = {}
    if not report.get("measurements_supported"):
        report["measurement_note"] = "当前固件未提供测试数值，判定结果保留"
    elif report["stage"] not in (4, 5) or (powered and not report.get("stop_confirmed")):
        report["measurement_note"] = "本轮尚未结束或关相未确认，未读取数值"
    else:
        try:
            for item in range(11):
                report["measurements"][str(item)] = link.measurement(item, active_run)
        except (Rejected, ValueError, TimeoutError, OSError) as exc:
            report["measurement_note"] = f"测试数值读取不完整：{exc}"
    report["communication_timeouts"] = link.timeouts.copy()
    report["communication_reliability_passed"] = not link.timeouts
    report["status_busy_replies"] = link.status_busy_replies
    if resistance_only:
        report["board_items"][8:] = [0, 0]
        report["directions"] = 0
    direction_results(report)
    if resistance_only:
        report["subset_passed"] = bool(
            report.get("stop_confirmed")
            and all(v == 1 or (i == 7 and v == 3) for i, v in enumerate(report["board_items"][:8]))
            and report.get("resistance", {}).get("result") == 1
            and report.get("resistance", {}).get("failure") == 0
            and not report.get("note")
            and not report.get("measurement_note")
        )
        report["motor_direction"] = {}
        report["scope_note"] = "本轮仅静态和相电阻；动态检测未执行"
    phase(
        "finalize",
        "已完成"
        if not report.get("measurement_note") and (not powered or report.get("stop_confirmed"))
        else "未完成",
    )
    for key, state in phases.copy().items():
        if state == "待执行":
            phase(key, "未执行")
    return report


def result_rows(report: dict) -> list[tuple[str, str, str, str]]:
    """投影本轮测量和原判据供GUI与CLI共用，不重新判定通过状态。"""
    limits = (
        "RMS <10 cnt；偏置偏差 <300 cnt",
        "RMS <10 cnt；偏置偏差 <300 cnt",
        "RMS <10 cnt；偏置偏差 <300 cnt",
        "RMS <50 cnt；偏置偏差 <300 cnt",
        "全程 27～29 V（含边界）",
        "0～60 °C；有效且不超过30 ms",
        "峰峰角 <0.01°；帧有效",
        "sin/cos RMS 均 <10 cnt",
        "去回符合命令方向；同向趋势 ≥2/3",
        "去回方向相反；同向趋势 ≥2/3；原码95～4000",
    )
    rows = []
    measurements = report.get("measurements", {})
    for item, name in enumerate(ITEMS):
        page = measurements.get(str(item), {})
        values = page.get("values", [None] * 4)
        state = report["items"][name]

        def value(index: int, digits: int = 2, suffix: str = "", measured_values=values) -> str:
            v = measured_values[index]
            return "—" if v is None else f"{v:.{digits}f}{suffix}"

        measured = "未采集" if report.get("measurements_supported") else "固件未提供数值"
        if state == "未装":
            measured = "未装"
        elif any(v is not None for v in values):
            if item <= 3:
                measured = f"均值 {value(0)}；RMS {value(1)} cnt"
            elif item == 4:
                measured = f"最低 {value(0)}；最高 {value(1)} V；越界 {value(2, 0)}/{page.get('samples', 0)}"
                if values[3]:
                    measured += f"；无效 {value(3, 0)}"
            elif item == 5:
                measured = f"最近 {value(0)}；范围 {value(1)}～{value(2)} °C"
            elif item == 6:
                measured = f"峰峰角 {value(0, 5)}°"
            elif item == 7:
                measured = (
                    f"sin 均值{value(0)} / RMS{value(1)}；cos 均值{value(2)} / RMS{value(3)} cnt"
                )
            else:
                ratios = ["—" if v is None else f"{v * 100:.1f}%" for v in values[2:]]
                measured = f"去程 {value(0, 3)}° ({ratios[0]})；回程 {value(1, 3)}° ({ratios[1]})"
        if item == 9 and state != "未装":
            raw = measurements.get("10", {}).get("values", [None] * 4)
            if all(v is not None for v in raw):
                measured += (
                    f"；原码 sin {raw[0]:.0f}～{raw[1]:.0f} / cos {raw[2]:.0f}～{raw[3]:.0f}"
                )
        if page.get("samples", 0) > 0 and state == "未测":
            measured += "（部分采样，未完成判定）"
        if item >= 8 and report.get("scope") in ("static", "static_resistance"):
            measured, state = "本轮未执行", "不在本轮范围"
        rows.append((name, measured, limits[item], state))
    resistance = report.get("resistance")
    currents = "/".join(f"{value:g}A" for value in report.get("test_currents_a", (1.0, 2.0)))
    measured, limit, state = "未测", f"标称相电阻的50%～150%（含边界）；{currents}", "未测"
    if resistance:
        nominal = resistance["nominal_ohm"]
        if nominal > 0:
            limit = f"{nominal * 0.5:.4f}～{nominal * 1.5:.4f} Ω；{currents}"

        def values(names, readings):
            return " / ".join(
                f"{name}={'—' if v is None else f'{v:.4f} Ω'}"
                for name, v in zip(names, readings, strict=True)
            )

        measured = values(("A", "B", "C"), resistance["phase_ohm"])
        measured += "；线间 " + values(("AB", "BC", "CA"), resistance["line_ohm"])
        faults = []
        for key, label, names in (
            ("open_phases", "相回路缺相", "ABC"),
            ("low_phases", "相电阻异常小", "ABC"),
            ("high_phases", "相电阻偏高", "ABC"),
        ):
            faults.extend(
                name + label for i, name in enumerate(names) if resistance[key] & (1 << i)
            )
        if resistance["open_lines"]:
            loops = [
                name
                for i, name in enumerate(("AB", "BC", "CA"))
                if resistance["open_lines"] & (1 << i)
            ]
            faults.append("/".join(loops) + "回路开路/未建立电流")
        failures = (
            "",
            "过流",
            "转子位移超过0.5°",
            "电流未稳定/超时",
            "无效测量",
            "检测取消或保护中止",
            "三相电流不平衡",
            "母线电压异常",
            "启动条件不满足",
            "正反向结果不一致",
            "阻值偏高",
        )
        if resistance["failure"]:
            faults.append(failures[resistance["failure"]])
        state = "正常" if resistance["result"] == 1 and not faults else "；".join(faults) or "未测"
    elif report.get("resistance_supported") is False:
        measured, limit, state = "当前固件不支持，未执行", "需升级支持相电阻检查的固件", "不支持"
    elif report.get("resistance_note"):
        measured = report["resistance_note"]
    rows.insert(8, ("相电阻", measured, limit, state))
    return rows


def summarize(report):
    lines = [f"节点 {report['node']}　通信{report['communication']}　{STAGES[report['stage']]}"]
    for name, measured, limit, state in result_rows(report):
        lines.append(f"{name}　{state}　{measured}　判据：{limit}")
        if name == "电机角动态" and report.get("motor_direction"):
            direction = report["motor_direction"]
            lines.append(f"  {direction['reason']}")
            lines.append(f"  {direction['detail']}")
    if "motor_temperature_c" in report:
        lines.append("结束后NTC读取 %.2f °C（独立于测试窗口）" % report["motor_temperature_c"])
    if report.get("measurement_note"):
        lines.append(report["measurement_note"])
    if "stop_confirmed" in report:
        lines.append("关相已确认" if report["stop_confirmed"] else "关相待现场确认")
    if report.get("communication_reliability_passed") is False:
        lines.append(
            "%d ms 静态应答可靠性未通过：超时 %d 次，驱动仍按50 ms，全部记录保留"
            % (report.get("reply_timeout_ms", 500), sum(report["communication_timeouts"].values()))
        )
    if report["note"]:
        lines.append(report["note"])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dll", type=Path, required=True)
    parser.add_argument("--node", type=int, required=True)
    parser.add_argument("--channel-index", type=int, choices=(0, 1), default=0)
    parser.add_argument(
        "--bitrate", nargs=2, type=int, default=(1000, 5000), metavar=("ARB_K", "DATA_K")
    )
    parser.add_argument("--ambient-c", type=float, help="独立温度计实测室温")
    parser.add_argument("--drive", action="store_true", help="显式请求双向限流驱动")
    parser.add_argument("--current-a", type=float, default=3.0, help="模式 10 拖动相电流，默认 3 A")
    parser.add_argument("--frequency-hz", type=float)
    parser.add_argument("--leg-ms", type=int)
    parser.add_argument("--travel-deg", type=float, help="距当前位置两侧可用的输出轴行程")
    parser.add_argument("--last-lease", type=int, help="本工具首次驱动该 UID 前已知的续租高水位")
    parser.add_argument("--last-target", type=int, help="首次驱动前已知的 Type 116 目标序号")
    parser.add_argument("--observe", action="store_true", help="关相后录入实际观察的两轴去回旋向")
    parser.add_argument("--out", type=Path, help="保存本轮 JSON")
    args = parser.parse_args()
    drive = None
    if args.drive:
        if None in (args.current_a, args.frequency_hz, args.leg_ms, args.travel_deg):
            parser.error("--drive 需同时指定 --current-a --frequency-hz --leg-ms --travel-deg")
        drive = Drive(args.current_a, args.frequency_hz, args.leg_ms, args.travel_deg)
    try:
        with (
            os.add_dll_directory(str(args.dll.resolve().parent)),
            ControlCanFd(
                args.dll,
                channel_index=args.channel_index,
                nominal_bps=args.bitrate[0] * 1000,
                data_bps=args.bitrate[1] * 1000,
            ) as bus,
        ):
            report = run_check(
                bus,
                args.node,
                drive=drive,
                ambient=args.ambient_c,
                previous=args.last_lease,
                previous_target=args.last_target,
                progress=print,
            )
    except OSError as exc:
        report = empty_report(args.node, args.ambient_c, str(exc))
    if args.observe and report.get("stop_confirmed") and report["stage"] == 4:
        lookup = {"?": 0, "+": 1, "-": 2, "0": 3}
        refs = []
        for label, index in (("电机", 8), ("霍尔输出轴", 9)):
            values = (
                input(f"{label}实物去程、回程 (+/-/0/? 各一字符)：")
                if report["board_items"][index] != 3
                else "??"
            )
            refs.append(
                tuple(lookup[c] for c in values)
                if len(values) == 2 and all(c in lookup for c in values)
                else (0, 0)
            )
        direction_results(report, *refs)
    print(summarize(report))
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    required = ITEMS if args.drive else ITEMS[:8]
    return int(
        report["communication"] != "正常"
        or not report.get("communication_reliability_passed", False)
        or report["stage"] != 4
        or any(report["items"][name] not in ("正常", "未装") for name in required)
    )


if __name__ == "__main__":
    raise SystemExit(main())
