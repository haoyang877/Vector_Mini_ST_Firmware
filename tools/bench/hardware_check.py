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
STAGES = ("未开始", "静态采样", "待驱动", "驱动中", "结束", "中止")
COUNTER_DB = Path.home() / "CAN-Link-Test" / "hardware-check.sqlite3"


class Rejected(RuntimeError):
    """对端有合法业务应答，但未受理相应操作。"""


class Link:
    """单节点串行往返。忽略 CRC、地址、命令、序号不匹配的数据。"""

    def __init__(self, bus, node):
        if not 1 <= node <= 254 or node == 2:
            raise ValueError("请选择与主站 2 不同的单播节点")
        self.bus, self.node = bus, node
        self.sequence = secrets.randbelow(65535)

    def request(self, kind, payload=b"", *, page=None, sequence=None):
        if sequence is None:
            self.sequence = (self.sequence + 1) & 65535
            sequence = self.sequence
        if page is not None:
            payload = struct.pack("<IIHH", 0, sequence, page, 0)
        self.bus.send(*encode_frames(self.node, kind, payload, sequence)[0])
        deadline = time.monotonic() + 0.050
        while (remaining := deadline - time.monotonic()) > 0:
            frame = self.bus.receive(remaining)
            if frame is None:
                break
            try:
                return decode_reply(*frame, self.node, kind, sequence)
            except ValueError:
                continue
        raise TimeoutError(f"节点 {self.node} Type {kind} 未收到匹配应答")

    def command(self, kind, payload=b"", *, sequence=None):
        reply = self.request(kind, payload, sequence=sequence)
        if len(reply) != 4:
            raise ValueError(f"Type {kind} 结果长度错误")
        code, detail = struct.unpack("<HH", reply)
        if code not in (0, 1):
            raise Rejected(f"Type {kind} 拒绝，result={code} detail={detail}")
        return code

    def status(self):
        data = self.request(108)
        if data == b"\x06\0\0\0" or (len(data) == 45 and data[:2] == b"\x06\0"):
            raise Rejected("状态采样忙")
        if len(data) != 45 or data[:4] != bytes(4) or data[42] != self.node:
            raise ValueError("Type 108 状态帧无效")
        return data

    def board(self, payload=b"", sequence=None):
        data = self.request(127, payload, sequence=sequence)
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
            and 100 <= self.leg_ms <= 3000
            and self.travel_deg > 0
        ):
            raise ValueError(
                "应急模式限流 0.001～3 A，电频 0.001～10 Hz，单程 100～3000 ms，可用行程须大于零"
            )


def reserve_counters(path, uid, previous, previous_target):
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
                (uid, (lease + 512) & 0xFFFFFFFF, (target + 16) & 65535),
            )
            return lease, target
    finally:
        connection.close()


def direction_results(report, motor=(0, 0), hall=(0, 0)):
    """0 无物理参照、1 正、2 负、3 实物静止。去回两段分别比较。"""
    values = report["board_items"].copy()
    for sensor, reference in enumerate((motor, hall)):
        measured = [(report["directions"] >> (4 * sensor + 2 * leg)) & 3 for leg in (0, 1)]
        index = 8 + sensor
        if values[index] in (2, 3):
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
    report["observed"] = {"motor": list(motor), "hall": list(hall)}
    report["items"] = dict(zip(ITEMS, (STATES[v] for v in values), strict=True))


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
    if before[43:] != bytes((5, 0)) or before[4:8] != bytes(4) or not before[24] & 2:
        raise Rejected("驱动需要轴空闲、无保护故障和有效机械位置")
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
            (0, 250),
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
                lease = (lease + 1) & 0xFFFFFFFF or 1
                link.command(118, struct.pack("<I", lease))
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
    ambient=None,
    previous=None,
    previous_target=None,
    counter_db=COUNTER_DB,
    stopped=lambda: False,
    progress=lambda _: None,
):
    """返回通信结论、本轮板端快照和结合实物参照的结果；此处不打开设备。"""
    link = Link(bus, node)
    report = empty_report(node, ambient)
    if drive:
        report["drive"] = vars(drive).copy()
    active_run = None

    def refresh():
        current = link.board()
        if current["run_id"] != active_run:
            raise Rejected("检测序号已改变")
        report.update(current)

    try:
        progress("检查主控与板端通信")
        for _ in range(3):
            try:
                info = link.request(1, page=0)
            except TimeoutError:
                continue
            if len(info) == 32 or info == b"\x06\0\0\0":
                break
        else:
            raise TimeoutError("三次 50 ms 探测均无有效应答")
        report["communication"] = "正常"
        if len(info) == 4:
            raise Rejected("设备忙，已确认通信")
        report["uid"] = info[:12].hex()
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
        if not report["supported"]:
            raise Rejected("该 APP 未提供硬件功能检测")
        if ambient is not None and (not math.isfinite(ambient) or not 0 <= ambient <= 60):
            raise Rejected("实测环境温度不在 0～60 °C，工况不满足")
        if drive and (identity[20] != 1 or not features & (1 << 13) or not modes & (1 << 10)):
            raise Rejected("驱动需要有效装配身份、模式 10 和运动租约")
        previous_run = link.board()["run_id"]
        if stopped():
            raise Rejected("操作员停止")
        start = struct.pack("<4BI", 2, 1, 2 if drive else 1, 0, previous_run)
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
        if drive and report["stage"] == 2:
            if stopped():
                raise Rejected("操作员停止")
            try:
                lease, target = reserve_counters(
                    counter_db, report["uid"], previous, previous_target
                )
            except (OSError, sqlite3.Error) as exc:
                raise Rejected(f"无法保存本轮运动计数：{exc}") from exc
            progress("低速正向、停稳、反向")
            execute_drive(link, report, drive, start_seq, refresh, stopped, lease, target)
            deadline = time.monotonic() + 0.5
            while report["stage"] in (2, 3):
                if time.monotonic() > deadline:
                    raise Rejected("关相后未取得最终板端报告")
                time.sleep(0.01)
                refresh()
        if report["stage"] == 5:
            report["note"] = "板端中止，未完成项目保留未测"
        if report["stage"] in (4, 5):
            read_temperature(link, report, ambient)
    except (Rejected, ValueError, TimeoutError, OSError) as exc:
        report["note"] = str(exc)
        if isinstance(exc, (TimeoutError, OSError, ValueError)):
            report["communication"] = "异常"
        if active_run is not None and report["stage"] in (2, 3):
            try:
                refresh()
            except (Rejected, ValueError, TimeoutError, OSError):
                pass
    direction_results(report)
    return report


def summarize(report):
    lines = [f"节点 {report['node']}　通信{report['communication']}　{STAGES[report['stage']]}"]
    lines.extend(f"{name}　{state}" for name, state in report["items"].items())
    if "motor_temperature_c" in report:
        lines.append("电机 NTC 实测 %.2f °C" % report["motor_temperature_c"])
    if "stop_confirmed" in report:
        lines.append("关相已确认" if report["stop_confirmed"] else "关相待现场确认")
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
    parser.add_argument("--current-a", type=float)
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
        args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    required = ITEMS if args.drive else ITEMS[:8]
    return int(
        report["communication"] != "正常"
        or report["stage"] != 4
        or any(report["items"][name] not in ("正常", "未装") for name in required)
    )


if __name__ == "__main__":
    raise SystemExit(main())
