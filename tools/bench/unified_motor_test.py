"""统一电机台架入口：默认生成方案/离线报告，只有run --execute才连接CAN。"""

from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import html
import io
import json
import math
import struct
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from host_app.ports.controlcanfd import ControlCanFd  # noqa: E402
from host_app.upgrade_client import decode_reply, encode_frames  # noqa: E402

BENCH_TYPE = 0x7F00
MAGIC = 0x3154424D
FREQUENCY = 20000
GROUPS = {
    "phase": (1, ["raw_a", "raw_b", "raw_c", "ia_a", "ib_a", "ic_a", "theta_e_rad"]),
    "dq": (2, ["id_ref_a", "iq_ref_a", "id_a", "iq_a", "vd_v", "vq_v", "speed_rad_s"]),
    "speed": (
        3,
        [
            "speed_ref_rad_s",
            "speed_ramp_rad_s",
            "speed_rad_s",
            "iq_ref_a",
            "iq_a",
            "bus_v",
            "position_rad",
        ],
    ),
    "sincos": (
        4,
        ["sin_counts", "cos_counts", "speed_rad_s", "motor_q15", "bus_v", "reserved0", "reserved1"],
    ),
    "angle": (
        5,
        [
            "raw_q15",
            "directed_q15",
            "linearized_q15",
            "observer_mech_rad",
            "theta_e_rad",
            "id_a",
            "iq_a",
        ],
    ),
}
CASE_NAMES = {
    "TC01": "三相电流采样纹波",
    "TC02": "输出轴sin/cos采样纹波",
    "TC03": "Id/Iq闭环纹波",
    "TC04": "Id/Iq阶跃响应",
    "TC05": "空载速度闭环稳定性",
    "TC06": "角度标定与波形",
    "TC07": "通信协议",
}


def save_json(path: Path, value: object) -> None:
    """保存可移植的记录；NaN留在CSV，JSON不伪装成有效数值。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )


def plan_template() -> dict:
    """生成未批准、无默认动力限值的七项采集模板。"""
    return {
        "version": 1,
        "reviewed": False,
        "target": "AT32F456 + YQ-5708 + 8:1",
        "gear_ratio": 8,
        "test_speed_rad_s": None,
        "expected_info_hex": None,
        "firmware_sha256": None,
        "limits": {
            "current_a": None,
            "speed_rad_s": None,
            "travel_rad": None,
            "duration_ms": None,
            "lease_ms": 500,
        },
        "criteria": {
            "raw_std_counts": 2.0,
            "raw_pp_counts": 12.0,
            "current_fraction": 0.02,
            "current_pp_fraction": 0.10,
            "rise_ms": 1.0,
            "settle_ms": 3.0,
            "overshoot_fraction": 0.10,
            "cross_fraction": 0.05,
            "sincos_residual_fraction": 0.005,
            "sincos_amplitude_imbalance": 0.05,
            "sincos_quadrature_deg": 5.0,
            "speed_steady_s": 20.0,
            "speed_bias_fraction": 0.01,
            "speed_bias_rad_s": 0.02,
            "speed_rms_fraction": 0.02,
            "speed_rms_rad_s": 0.03,
            "speed_pp_fraction": 0.10,
            "speed_pp_rad_s": 0.10,
            "protocol_p99_ms": 20.0,
            "protocol_timeout_ms": 100.0,
            "angle_rms_counts": 256.0,
            "angle_peak_counts": 1024.0,
        },
        "required_steps": {
            "TC01": ["TC01_off", "TC01_dynamic"],
            "TC02": ["TC02_static", "TC02_dynamic"],
            "TC03": ["TC03_id", "TC03_running"],
            "TC04": ["TC04_id", "TC04_iq"],
            "TC05": ["TC05_forward", "TC05_reverse"],
            "TC06": ["TC06_lut", "TC06_zero", "TC06_after"],
            "TC07": ["TC07_readonly", "TC07_lifecycle", "TC07_negative"],
        },
        "steps": [
            {"id": "TC07_readonly", "case": "TC07", "operation": "protocol", "requests": 1000},
            {"id": "TC01_off", "case": "TC01", "group": "phase", "stride": 1, "trigger": 0},
            {
                "id": "TC02_static",
                "case": "TC02",
                "group": "sincos",
                "stride": 10,
                "trigger": 0,
                "confirm": "确认输出轴自然停稳；四个位置需分别重复记录",
            },
            {
                "id": "TC06_lut",
                "case": "TC06",
                "group": "angle",
                "stride": 500,
                "trigger": 3,
                "phase": 26,
                "drive": {"kind": "lut"},
                "confirm": "确认安装、防护、线缆行程和标定限值",
            },
            {
                "id": "TC06_zero",
                "case": "TC06",
                "group": "angle",
                "stride": 300,
                "trigger": 1,
                "drive": {"kind": "zero"},
            },
            {
                "id": "TC03_id",
                "case": "TC03",
                "group": "dq",
                "stride": 1,
                "trigger": 2,
                "delay_ms": 100,
                "drive": {"kind": "id", "fraction": 0.10, "duration_ms": 1000},
            },
            {
                "id": "TC04_id",
                "case": "TC04",
                "group": "dq",
                "stride": 1,
                "trigger": 1,
                "pre": 32,
                "drive": {"kind": "id", "fraction": 0.10, "duration_ms": 200},
            },
            {
                "id": "TC04_iq",
                "case": "TC04",
                "group": "dq",
                "stride": 2,
                "trigger": 1,
                "pre": 32,
                "drive": {"kind": "iq_pulse", "fraction": 0.10, "duration_ms": 5},
                "confirm": "确认空载短脉冲幅值、速度/位移截止已批准",
            },
            {
                "id": "TC05_forward",
                "case": "TC05",
                "group": "speed",
                "stride": 100,
                "trigger": 1,
                "drive": {"kind": "speed", "fraction": 0.10, "duration_ms": 25000},
            },
            {
                "id": "TC05_reverse",
                "case": "TC05",
                "group": "speed",
                "stride": 100,
                "trigger": 1,
                "drive": {"kind": "speed", "fraction": -0.10, "duration_ms": 25000},
                "confirm": "确认上一次已停止且轴静止，可以反向",
            },
            {
                "id": "TC06_after",
                "case": "TC06",
                "group": "angle",
                "trigger": 2,
                "delay_ms": 500,
                "auto_cycles": 3.2,
                "cycle_axis": "motor",
                "drive": {"kind": "speed", "fraction": 1.0, "duration_ms": 25000},
            },
            {
                "id": "TC03_running",
                "case": "TC03",
                "group": "dq",
                "stride": 1,
                "trigger": 2,
                "delay_ms": 1000,
                "drive": {"kind": "speed", "fraction": 0.10, "duration_ms": 2000},
            },
            {
                "id": "TC01_dynamic",
                "case": "TC01",
                "group": "phase",
                "trigger": 2,
                "delay_ms": 500,
                "auto_cycles": 3.2,
                "cycle_axis": "electrical",
                "drive": {"kind": "speed", "fraction": 1.0, "duration_ms": 10000},
            },
            {
                "id": "TC02_dynamic",
                "case": "TC02",
                "group": "sincos",
                "trigger": 2,
                "delay_ms": 500,
                "auto_cycles": 3.2,
                "cycle_axis": "output",
                "drive": {"kind": "speed", "fraction": 1.0, "duration_ms": 60000},
            },
            {
                "id": "TC07_negative",
                "case": "TC07",
                "operation": "protocol",
                "scenario": "negative",
            },
        ],
        "deferred": {
            "TC01": "无示波器/电流探头，物理PWM纹波未测",
            "TC03": "无固定装置，非零Iq锁轴稳态未测",
            "TC04": "无固定装置，固定转子Iq阶跃未测",
            "TC05": "无加载装置，负载扰动未测",
            "TC06": "无独立参考；输出轴角度标定尚未实现",
        },
        "pending_scope": {
            "TC01": "补齐关相1秒统计、五个短窗、零给定启相比较和三次重复",
            "TC02": "补齐四个静止位置各2秒、零给定启相比较、双向和三次重复",
            "TC03": "补齐零参考及三个电流档、至少1秒多窗口和三次重复",
            "TC04": "补齐各档正负/下降阶跃及三次重复",
            "TC05": "补齐三个转速档、速度切换/停止响应和三次重复",
            "TC06": "补齐独立参考误差、重复性和重启后的参数检查",
            "TC07": "补齐普通启动/停止生命周期、错误帧及重复请求注入",
        },
    }


class Client:
    """只使用显式打开的现有CAN端口；所有请求单次发送，不自动重放动力命令。"""

    def __init__(self, bus: ControlCanFd, node: int, log: Path):
        self.bus, self.node, self.log = bus, node, log
        self.sequence = 0

    def record(self, direction: str, identifier: int, data: bytes) -> None:
        """逐帧保存主机时间和原始字节。"""
        with self.log.open("a", encoding="utf-8") as stream:
            stream.write(
                json.dumps(
                    {
                        "time": time.monotonic(),
                        "direction": direction,
                        "id": identifier,
                        "data": data.hex(),
                    }
                )
                + "\n"
            )

    def request(self, kind: int, payload: bytes = b"", timeout: float = 0.2) -> bytes:
        """请求一次应答；身份/CRC/序号由已有编解码器检查，超时停止事务。"""
        self.sequence = (self.sequence + 1) & 0xFFFF
        for identifier, data in encode_frames(self.node, kind, payload, self.sequence):
            self.record("tx", identifier, data)
            self.bus.send(identifier, data)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = self.bus.receive(max(0.0, deadline - time.monotonic()))
            if frame is None:
                break
            self.record("rx", *frame)
            try:
                return decode_reply(*frame, self.node, kind, self.sequence)
            except ValueError:
                continue
        raise TimeoutError(f"0x{kind:04X}应答超时；动力命令不会重发")

    def bench(self, operation: int, data: bytes = b"") -> bytes:
        """检查测试协议版本和返回结果，保留单条命令的明确失败。"""
        reply = self.request(BENCH_TYPE, struct.pack("<HH", operation, 1) + data)
        if len(reply) < 4:
            raise ValueError("测试应答过短")
        code, returned = struct.unpack_from("<HH", reply)
        if returned != operation or code not in (0, 1):
            raise RuntimeError(f"测试操作{operation}失败：result={code}, reply={reply.hex()}")
        return reply

    def info(self) -> dict:
        """读取实际运行、采集和数字关相状态，不推断转子已静止。"""
        reply = self.bench(0)
        if len(reply) != 46:
            raise ValueError("测试状态长度错误")
        values = struct.unpack("<HH4I13H", reply)
        names = [
            "result",
            "op",
            "magic",
            "tick",
            "run_id",
            "capture_id",
            "run_state",
            "capture_state",
            "run_kind",
            "reason",
            "count",
            "group",
            "invalid",
            "calibration_flags",
            "zero_q15",
            "pole_pairs",
            "fault",
            "flags",
            "last_sequence",
        ]
        result = dict(zip(names, values, strict=True))
        if result["magic"] != MAGIC:
            raise ValueError("目标不是匹配的统一测试镜像")
        return result

    def stop(self) -> None:
        """先发测试停止，再复核数字关相；失败不会自动恢复运行。"""
        self.bench(6)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if self.info()["flags"] & 4:
                return
            time.sleep(0.01)
        raise RuntimeError("未确认数字关相，需要独立断能；禁止继续")


def validate_plan(plan: dict, firmware: Path) -> None:
    """在打开适配器前检查批准记录、镜像哈希和全部动力边界。"""
    if plan.get("version") != 1 or plan.get("reviewed") is not True:
        raise ValueError("方案尚未批准：reviewed必须显式为true")
    digest = hashlib.sha256(firmware.read_bytes()).hexdigest()
    if plan.get("firmware_sha256") != digest:
        raise ValueError("本地镜像SHA256与批准方案不一致")
    if not plan.get("expected_info_hex"):
        raise ValueError("缺少已核对的GET_INFO page0身份原始字节")
    if len(bytes.fromhex(plan["expected_info_hex"])) != 32:
        raise ValueError("GET_INFO page0身份必须恰好32字节")
    limits = plan["limits"]
    for name in ("current_a", "speed_rad_s", "travel_rad", "duration_ms", "lease_ms"):
        value = limits.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"缺少明确的正数限值：{name}")
    if not 100 <= limits["lease_ms"] <= 1000 or limits["duration_ms"] > 120000:
        raise ValueError("主机租期或通电时间超出板端范围")
    if limits["speed_rad_s"] > 500 or limits["travel_rad"] > 10000:
        raise ValueError("速度或行程超出板端范围")
    if type(plan.get("gear_ratio")) not in (int, float) or plan["gear_ratio"] != 8:
        raise ValueError("本批次只接受已确认的8:1减速比")
    for name, value in plan["criteria"].items():
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"指标必须为正有限值：{name}")
    speed = plan.get("test_speed_rad_s")
    if (
        type(speed) not in (int, float)
        or not math.isfinite(speed)
        or not 0 < speed <= limits["speed_rad_s"]
    ):
        raise ValueError("须独立给出不高于速度截止值的test_speed_rad_s")
    identities = set()
    for step in plan["steps"]:
        if step["id"] in identities or step["case"] not in CASE_NAMES:
            raise ValueError("步骤ID重复或用例编号无效")
        identities.add(step["id"])
        if Path(step["id"]).name != step["id"] or "/" in step["id"] or "\\" in step["id"]:
            raise ValueError("步骤ID不能包含路径")
        if step.get("operation") == "protocol":
            continue
        if step.get("group") not in GROUPS or not 1 <= step.get("stride", 1) <= 10000:
            raise ValueError("采集组或抽取系数无效")
        drive = step.get("drive", {})
        if (
            not 0 <= step.get("trigger", 0) <= 3
            or not 0 <= step.get("pre", 0) <= 32
            or not 0 <= step.get("delay_ms", 0) <= 120000
        ):
            raise ValueError("触发参数越界")
        if "auto_cycles" in step and (
            type(step["auto_cycles"]) not in (int, float)
            or not math.isfinite(step["auto_cycles"])
            or step["auto_cycles"] < 3
            or step.get("cycle_axis") not in ("electrical", "motor", "output")
            or drive.get("kind") != "speed"
            or not drive.get("fraction")
        ):
            raise ValueError("周期采集必须是有效的速度工况，且不少于三个周期")
        if (
            step.get("group") == "sincos"
            and "auto_cycles" not in step
            and step.get("stride", 1) < 10
        ):
            raise ValueError("sin/cos采样抽取系数不能小于10")
        if drive.get("kind") in ("id", "iq_pulse", "speed"):
            fraction = drive.get("fraction")
            duration = drive.get("duration_ms")
            if (
                not isinstance(fraction, (int, float))
                or not math.isfinite(fraction)
                or abs(fraction) > 1
            ):
                raise ValueError("目标必须是限值内的有限比例")
            if (
                not isinstance(duration, (int, float))
                or not math.isfinite(duration)
                or duration <= 0
                or ("auto_cycles" not in step and duration > limits["duration_ms"])
            ):
                raise ValueError("运行时长超出批准限值")
            if drive["kind"] == "iq_pulse" and duration > 5:
                raise ValueError("空载Iq脉冲不能超过5ms")
            if drive["kind"] == "id" and fraction < 0:
                raise ValueError("本方案不支持负Id激励")
        elif drive and drive.get("kind") not in ("lut", "zero", "mechanical_zero"):
            raise ValueError("未知激励")


def write_samples(path: Path, group: str, rows: list[tuple]) -> None:
    """保存原始板端tick与七通道数据，不平滑、不拼接窗口。"""
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["tick", *GROUPS[group][1]])
        writer.writerows(rows)


def acquire(client: Client, plan: dict, step: dict, directory: Path) -> dict:
    """采集单个已批准工况；动力终止后才读取冻结缓冲。"""
    limits = plan["limits"]
    step = json.loads(json.dumps(step))
    if "auto_cycles" in step:
        frequency = abs(step["drive"]["fraction"] * plan["test_speed_rad_s"])
        info = client.info()
        if step["cycle_axis"] == "electrical":
            frequency *= info["pole_pairs"]
        elif step["cycle_axis"] == "output":
            frequency /= plan["gear_ratio"]
        window = step["auto_cycles"] * 2 * math.pi / frequency if frequency > 0 else math.inf
        stride = math.ceil(window * FREQUENCY / 127) if math.isfinite(window) else 10001
        duration_ms = (
            math.ceil(window * 1000 + step.get("delay_ms", 0) + 1000)
            if math.isfinite(window)
            else 120001
        )
        if stride > 10000 or duration_ms > limits["duration_ms"]:
            record = {
                "step": step,
                "status": "BLOCKED",
                "reason": "当前低速/时限内无法覆盖足够周期，未发送动力请求",
            }
            save_json(directory / (step["id"] + ".json"), record)
            return record
        step["stride"] = max(10 if step["group"] == "sincos" else 1, stride)
        step["drive"]["duration_ms"] = duration_ms
    client.bench(
        1,
        struct.pack(
            "<fffII",
            limits["current_a"],
            limits["speed_rad_s"],
            limits["travel_rad"],
            int(limits["duration_ms"]),
            int(limits["lease_ms"]),
        ),
    )
    group = step["group"]
    reply = client.bench(
        3,
        struct.pack(
            "<4HI2H",
            GROUPS[group][0],
            step.get("stride", 1),
            step.get("trigger", 0),
            step.get("pre", 0),
            step.get("delay_ms", 0),
            step.get("phase", 0),
            0,
        ),
    )
    capture_id = struct.unpack_from("<I", reply, 4)[0]
    drive = step.get("drive")
    if drive:
        kind = drive["kind"]
        if kind in ("lut", "zero", "mechanical_zero"):
            payload = struct.pack("<HH", {"lut": 1, "zero": 2, "mechanical_zero": 6}[kind], 0)
            response = client.request(0x00A0, payload)
            if len(response) != 12 or struct.unpack_from("<H", response)[0] != 1:
                raise RuntimeError("标定请求未受理")
        else:
            scale = plan["test_speed_rad_s"] if kind == "speed" else limits["current_a"]
            client.bench(
                2,
                struct.pack(
                    "<HHfII",
                    {"id": 1, "iq_pulse": 2, "speed": 3}[kind],
                    0,
                    drive["fraction"] * scale,
                    round(drive["duration_ms"] * 1000),
                    10000,
                ),
            )
    timeout = max(
        limits["duration_ms"] / 1000 + 5,
        128 * step.get("stride", 1) / FREQUENCY + step.get("delay_ms", 0) / 1000 + 5,
    )
    deadline = time.monotonic() + timeout
    trend, states = [], []
    while time.monotonic() < deadline:
        info = client.info()
        states.append(info)
        client.bench(5, struct.pack("<I", info["run_id"]))
        snapshot = client.bench(8)
        if not drive or struct.unpack_from("<H", snapshot, 38)[0] == 2:
            trend.append(struct.unpack_from("<I7f", snapshot, 4))
        if info["run_state"] == 4:
            break
        if info["capture_state"] == 3 and (
            not drive or (info["run_state"] == 3 and info["flags"] & 4)
        ):
            break
        time.sleep(0.02)
    else:
        raise TimeoutError("采集或运行未按期完成")
    failed = info["run_state"] == 4 or info["fault"] != 0
    # 停止已明确受理但尚未收尾的动作，确保READ满足原生命周期的数字关相条件。
    client.stop()
    final = client.info()
    if final["capture_id"] != capture_id:
        raise RuntimeError("采集代际改变，拒绝把旧数据拼入新工况")
    rows = []
    for index in range(final["count"]):
        response = client.bench(4, struct.pack("<IHH", capture_id, index, 0))
        if len(response) != 44 or struct.unpack_from("<IHH", response, 4) != (
            capture_id,
            index,
            final["count"],
        ):
            raise ValueError("冻结数据的代际、索引或长度不一致")
        rows.append(struct.unpack_from("<I7f", response, 12))
    write_samples(directory / (step["id"] + ".csv"), group, rows)
    write_samples(directory / (step["id"] + "_trend.csv"), group, trend)
    record = {
        "step": step,
        "info": info,
        "final": final,
        "states": states,
        "status": "FAIL" if failed else "RECORDED",
        "rows": len(rows),
    }
    save_json(directory / (step["id"] + ".json"), record)
    if drive and drive["kind"] in ("lut", "zero") and not failed:
        table = []
        for start in range(0, 1024, 16):
            reply = client.bench(7, struct.pack("<HH", start, 16))
            if len(reply) != 40 or struct.unpack_from("<HH", reply, 4) != (start, 16):
                raise ValueError("LUT分块身份不一致")
            table.extend(struct.unpack_from("<16h", reply, 8))
        save_json(directory / (step["id"] + "_lut.json"), {"values": table, "info": final})
    return record


def protocol_check(client: Client, step: dict, directory: Path) -> dict:
    """关相完成只读命令检查，不用该结果代替真实电流/速度测试。"""
    client.stop()
    if step.get("scenario") == "negative":
        checks = []
        for op, version, expected in ((0, 2, 4), (65535, 1, 4)):
            reply = client.request(BENCH_TYPE, struct.pack("<HH", op, version))
            if len(reply) != 4 or struct.unpack_from("<H", reply)[0] != expected:
                raise ValueError("非法版本/操作未按约定拒绝")
            checks.append({"op": op, "version": version, "result": expected})
        for kind, payload in (
            (0xA0, struct.pack("<HH", 3, 0)),
            (0xA1, struct.pack("<I", 1)),
            (0xA2, struct.pack("<I", 1)),
        ):
            reply = client.request(kind, payload)
            if len(reply) != 12 or struct.unpack_from("<H", reply)[0] != 2:
                raise ValueError("不支持的标定类型/查询未明确拒绝")
            checks.append({"type": kind, "result": 2})
        record = {
            "step": step,
            "status": "RECORDED",
            "checks": checks,
            "scope": "版本/未知操作/不支持标定类型/查询取消；物理错误帧注入未覆盖",
        }
        save_json(directory / (step["id"] + ".json"), record)
        return record
    timings = []
    for index in range(int(step.get("requests", 1000))):
        kind = (0x0001, 0x0002, 0x0003, 0x006C)[index % 4]
        before = time.monotonic()
        response = client.request(
            kind, struct.pack("<IIHH", 0, index, 0, 0) if kind in (1, 2) else b""
        )
        expected_size = {1: 32, 2: 16, 3: 16, 0x6C: 45}[kind]
        if len(response) != expected_size:
            raise ValueError(f"0x{kind:04X}业务应答长度错误：{len(response)}")
        timings.append((time.monotonic() - before) * 1000)
        time.sleep(0.1)
    record = {
        "step": step,
        "status": "RECORDED",
        "timings_ms": timings,
        "scope": "只读响应/身份/CRC/序号；动力及异常帧场景未据此判定通过",
    }
    save_json(directory / (step["id"] + ".json"), record)
    return record


def run_plan(args: argparse.Namespace) -> None:
    """显式执行已批准方案；任一错误停止后终止，不自动重试后续动作。"""
    if not args.execute:
        raise ValueError("连接硬件必须显式传入--execute；本命令不会默认执行")
    if not 1 <= args.node < 255 or args.node == 2:
        raise ValueError("目标必须是有效的设备单播地址，不能使用主站2或广播255")
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    validate_plan(plan, args.firmware)
    if args.output.exists():
        raise ValueError("输出目录已存在，请为新批次使用新目录")
    raw = args.output / "raw"
    raw.mkdir(parents=True)
    save_json(args.output / "plan.json", plan)
    save_json(
        args.output / "manifest.json",
        {
            "node": args.node,
            "target": plan["target"],
            "firmware_sha256": plan["firmware_sha256"],
            "target_image_readback_verified": False,
            "dll_sha256": hashlib.sha256(args.dll.read_bytes()).hexdigest(),
            "status": "IN_PROGRESS",
        },
    )
    with ControlCanFd(args.dll, args.device_index) as bus:
        client = Client(bus, args.node, raw / "frames.jsonl")
        identity = client.request(1, struct.pack("<IIHH", 0, 0, 0, 0))
        if identity.hex() != plan["expected_info_hex"].lower():
            raise ValueError("设备身份与批准方案不符，未发送测试动作")
        initial = client.info()
        if initial["flags"] & 16:
            client.sequence = initial["last_sequence"]
        failure = None
        active_step = None
        try:
            for step in plan["steps"]:
                active_step = step
                if step.get("confirm") and input(step["confirm"] + "；键入YES继续：") != "YES":
                    raise RuntimeError("人工检查点未确认")
                print(step["id"], flush=True)
                result = (
                    protocol_check(client, step, raw)
                    if step.get("operation") == "protocol"
                    else acquire(client, plan, step, raw)
                )
                if result["status"] == "FAIL":
                    raise RuntimeError("本工况失败，后续动作已取消")
                if result["status"] == "RECORDED":
                    status, metrics, note = assess_record(result, raw, plan)
                    result["assessment"] = {"status": status, "metrics": metrics, "note": note}
                    save_json(raw / (step["id"] + ".json"), result)
                    if status == "FAIL":
                        raise RuntimeError("本工况指标未通过，后续动作已取消：" + step["id"])
        except Exception as exc:
            failure = exc
            save_json(
                args.output / "failure.json",
                {
                    "error": str(exc),
                    "status": "FAIL",
                    "step": active_step,
                },
            )
        finally:
            try:
                client.stop()
            except Exception as exc:
                failure = failure or exc
                save_json(args.output / "stop_failure.json", {"error": str(exc), "status": "FAIL"})
    manifest_path = args.output / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    manifest["status"] = "FAILED" if failure is not None else "EXECUTION_COMPLETE"
    save_json(manifest_path, manifest)
    render_report(args.output)
    if failure is not None:
        raise RuntimeError(f"本批次中止，已保留报告和原始帧：{failure}") from failure


def statistics(values) -> dict:
    """保留偏差、总RMSE、去均值纹波及峰峰值的不同定义。"""
    import numpy as np

    return {
        "mean": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(values**2))),
        "ripple_rms": float(np.std(values)),
        "peak_to_peak": float(np.ptp(values)),
    }


def evaluate(step: dict, rows, plan: dict) -> tuple[str, dict, str]:
    """按建议标准计算已覆盖数据；样本/周期不足返回BLOCKED，不伪造精度结论。"""
    import numpy as np

    if len(rows) < 8 or not np.isfinite(rows).all():
        return "BLOCKED", {}, "样本不足或存在无效值"
    tick = rows[:, 0].astype(np.uint32)
    dt = np.diff(tick).astype(np.uint32) / FREQUENCY
    if (dt <= 0).any():
        return "FAIL", {}, "采样时间不连续或重复"
    x = rows[:, 1:]
    group, case = step["group"], step["case"]
    criteria = plan["criteria"]
    current = plan["limits"].get("current_a")
    if group in ("phase", "sincos"):
        raw = x[:, : 3 if group == "phase" else 2]
        if np.any(raw <= 0) or np.any(raw >= 4095):
            return "FAIL", {}, "ADC原码触及量程边界，不能用小纹波掩盖削顶或未采样"
    if group == "phase":
        metrics = {name: statistics(x[:, i]) for i, name in enumerate(GROUPS[group][1][:6])}
        if step.get("drive"):
            angle = np.unwrap(x[:, 6])
            if np.ptp(angle) < 6 * np.pi:
                return "BLOCKED", metrics, "不足三个电周期，未把基波幅值当作纹波"
            fit = np.column_stack((np.sin(angle), np.cos(angle), np.ones(len(angle))))
            residuals = x[:, 3:6] - fit @ np.linalg.lstsq(fit, x[:, 3:6], rcond=None)[0]
            metrics["residual_a"] = [statistics(residuals[:, i]) for i in range(3)]
            passed = current is not None and all(
                m["ripple_rms"] <= criteria["current_fraction"] * current
                and m["peak_to_peak"] <= criteria["current_pp_fraction"] * current
                for m in metrics["residual_a"]
            )
        else:
            passed = all(
                metrics[n]["ripple_rms"] <= criteria["raw_std_counts"]
                and metrics[n]["peak_to_peak"] <= criteria["raw_pp_counts"]
                for n in GROUPS[group][1][:3]
            )
        return "PASS" if passed else "FAIL", metrics, "板载采样范围；物理PWM纹波未测"
    if group == "dq" and case == "TC03":
        metrics = {
            "id_error": statistics(x[:, 2] - x[:, 0]),
            "iq_error": statistics(x[:, 3] - x[:, 1]),
        }
        if current is None:
            return "BLOCKED", metrics, "缺少批准电流上限"
        passed = all(
            abs(m["mean"]) <= criteria["current_fraction"] * current
            and m["ripple_rms"] <= criteria["current_fraction"] * current
            and m["peak_to_peak"] <= criteria["current_pp_fraction"] * current
            for m in metrics.values()
        )
        return "PASS" if passed else "FAIL", metrics, "仅本次空载工况，锁轴稳态未测"
    if group == "dq" and case == "TC04":
        axis = 1 if step["drive"]["kind"] == "iq_pulse" else 0
        reference, actual = x[:, axis], x[:, axis + 2]
        transitions = np.flatnonzero(np.abs(np.diff(reference)) > 1e-6) + 1
        if not len(transitions) or current is None:
            return "BLOCKED", {}, "没有捕获参考阶跃或缺少电流上限"
        start = int(transitions[0])
        end = int(transitions[1]) if len(transitions) > 1 else len(x)
        baseline = float(np.mean(actual[:start]))
        change = reference[start] - reference[start - 1]
        elapsed = ((tick[start:end] - tick[start]).astype(np.uint32)) / FREQUENCY
        progress = (actual[start:end] - baseline) / change
        ten, ninety = np.flatnonzero(progress >= 0.1), np.flatnonzero(progress >= 0.9)
        metrics = {
            "overshoot_fraction": float(max(0, np.max(progress) - 1)),
            "cross_peak_a": float(
                np.max(np.abs(x[start:end, 3 - axis] - np.mean(x[:start, 3 - axis])))
            ),
        }
        if not len(ten) or not len(ninety):
            return "FAIL", metrics, "未达到90%参考"
        metrics["rise_ms"] = float((elapsed[ninety[0]] - elapsed[ten[0]]) * 1000)
        error = np.abs(actual[start:end] - reference[start])
        band = max(0.02 * abs(change), 0.01 * current)
        outside = np.flatnonzero(error > band)
        stable = int(outside[-1] + 1) if len(outside) else 0
        if stable >= len(elapsed) or elapsed[-1] - elapsed[stable] < 0.001:
            return "BLOCKED", metrics, "窗口没有覆盖至少1ms的持续稳定带"
        metrics["settle_ms"] = float(elapsed[stable] * 1000)
        passed = (
            metrics["rise_ms"] <= criteria["rise_ms"]
            and metrics["settle_ms"] <= criteria["settle_ms"]
            and metrics["overshoot_fraction"] <= criteria["overshoot_fraction"]
            and metrics["cross_peak_a"] <= criteria["cross_fraction"] * current
        )
        return "PASS" if passed else "FAIL", metrics, "板载采样的空载瞬态，未验证锁轴条件"
    if group == "speed":
        elapsed = ((tick - tick[0]).astype(np.uint32)) / FREQUENCY
        if elapsed[-1] < criteria["speed_steady_s"]:
            return "BLOCKED", {}, "不足20秒稳态数据"
        selected = elapsed >= elapsed[-1] - criteria["speed_steady_s"]
        error = x[selected, 2] - x[selected, 1]
        metrics = statistics(error)
        reference = float(np.mean(np.abs(x[selected, 1])))
        if np.ptp(x[selected, 1]) > max(0.01 * reference, 0.02):
            return "BLOCKED", metrics, "末20秒仍在参考斜坡中"
        passed = (
            abs(metrics["mean"])
            <= max(criteria["speed_bias_fraction"] * reference, criteria["speed_bias_rad_s"])
            and metrics["rmse"]
            <= max(criteria["speed_rms_fraction"] * reference, criteria["speed_rms_rad_s"])
            and metrics["peak_to_peak"]
            <= max(criteria["speed_pp_fraction"] * reference, criteria["speed_pp_rad_s"])
        )
        return "PASS" if passed else "FAIL", metrics, "只评价空载稳态，未证明负载恢复或无持续振荡"
    if group == "sincos":
        metrics = {"sin": statistics(x[:, 0]), "cos": statistics(x[:, 1])}
        if step.get("drive"):
            angle = np.unwrap(x[:, 3] * (2 * np.pi / 65536)) / plan["gear_ratio"]
            if np.ptp(angle) < 6 * np.pi:
                return "BLOCKED", metrics, "不足三个输出轴周期"
            fit = np.column_stack((np.sin(angle), np.cos(angle), np.ones(len(angle))))
            coefficients = np.linalg.lstsq(fit, x[:, :2], rcond=None)[0]
            amplitudes = np.hypot(coefficients[0], coefficients[1])
            phase = np.arctan2(coefficients[1], coefficients[0])
            separation = abs(float(np.angle(np.exp(1j * (phase[1] - phase[0])))))
            metrics["amplitude_imbalance"] = float(
                abs(amplitudes[0] - amplitudes[1]) / max(np.mean(amplitudes), 1e-9)
            )
            metrics["quadrature_error_deg"] = abs(math.degrees(separation) - 90)
            residual = x[:, :2] - fit @ coefficients
            metrics["residual_rms_counts"] = np.sqrt(np.mean(residual**2, axis=0)).tolist()
            passed = (
                metrics["amplitude_imbalance"] <= criteria["sincos_amplitude_imbalance"]
                and metrics["quadrature_error_deg"] <= criteria["sincos_quadrature_deg"]
                and max(metrics["residual_rms_counts"])
                <= criteria["sincos_residual_fraction"] * 4095
            )
        else:
            passed = all(
                m["ripple_rms"] <= criteria["raw_std_counts"]
                and m["peak_to_peak"] <= criteria["raw_pp_counts"]
                for m in metrics.values()
            )
        return "PASS" if passed else "FAIL", metrics, "原码/几何一致性；不代表输出轴角度已标定"
    if group == "angle":
        angle = np.unwrap(x[:, 2] * (2 * np.pi / 65536))
        error = np.angle(np.exp(1j * (angle - x[:, 3])))
        origin = np.angle(np.mean(np.exp(1j * error)))
        residual = np.angle(np.exp(1j * (error - origin))) * (65536 / (2 * np.pi))
        metrics = statistics(residual)
        metrics["aligned_origin_rad"] = float(origin)
        if np.ptp(angle) < 6 * np.pi:
            return "BLOCKED", metrics, "不足三个机械圈；可显示标定/电零位波形，不判定角度精度"
        passed = (
            metrics["rmse"] <= criteria["angle_rms_counts"]
            and float(np.max(np.abs(residual))) <= criteria["angle_peak_counts"]
        )
        return "PASS" if passed else "FAIL", metrics, "去除原点差的内部一致性，不是独立绝对角度精度"
    return "BLOCKED", {}, "该数据未覆盖对应判据"


def assess_record(record: dict, raw: Path, plan: dict) -> tuple[str, dict | list, str]:
    """用报告的同一判据在下一次激励前判定本项，指标失败不得继续批次。"""
    import numpy as np

    step = record["step"]
    if record["status"] == "FAIL":
        return "FAIL", {}, record.get("error", "本项执行失败")
    if step.get("operation") == "protocol":
        if "timings_ms" not in record:
            return "PASS", record.get("checks", []), record["scope"]
        timing = np.asarray(record["timings_ms"], dtype=float)
        if not len(timing) or not np.isfinite(timing).all():
            return "FAIL", {}, "没有有效通信时间记录"
        metrics = {
            "requests": len(timing),
            "p99_ms": float(np.percentile(timing, 99)),
            "maximum_ms": float(np.max(timing)),
        }
        passed = (
            len(timing) >= 1000
            and metrics["p99_ms"] <= plan["criteria"]["protocol_p99_ms"]
            and metrics["maximum_ms"] <= plan["criteria"]["protocol_timeout_ms"]
        )
        return "PASS" if passed else "FAIL", metrics, record["scope"]
    suffix = "_trend.csv" if step["group"] == "speed" else ".csv"
    with (raw / (step["id"] + suffix)).open(encoding="utf-8") as stream:
        reader = csv.reader(stream)
        next(reader)
        rows = np.array([[float(value) for value in row] for row in reader])
    calibration = {"lut": (4, 1), "zero": (5, 3), "mechanical_zero": (6, 4)}
    kind = step.get("drive", {}).get("kind")
    if kind in calibration:
        run_kind, required_flags = calibration[kind]
        info = record.get("info", {})
        complete = (
            info.get("run_kind") == run_kind
            and info.get("run_state") == 3
            and info.get("reason") == 0
            and info.get("fault") == 0
            and info.get("calibration_flags", 0) & required_flags == required_flags
        )
        metrics = {
            "completion_verified": complete,
            "calibration_flags": info.get("calibration_flags"),
            "sample_count": len(rows),
        }
        if not complete or info.get("invalid", 0):
            return "FAIL", metrics, "标定未完成、未发布、被截止或采集含无效值"
        if len(rows) < 8 or not np.isfinite(rows).all():
            return "BLOCKED", metrics, "标定完成但有效波形不足"
        return "PASS", metrics, "板端标定校验及完成已确认；启动过程不混入独立角度精度判定"
    return evaluate(step, rows, plan)


def embed_plot(figure, name: str, directory: Path) -> str:
    """保存实际数据生成的PNG，并内嵌到可离线打开的报告。"""
    import matplotlib.pyplot as plt

    png = io.BytesIO()
    figure.savefig(png, format="png", dpi=130)
    plt.close(figure)
    plots = directory / "plots"
    plots.mkdir(exist_ok=True)
    (plots / (name + ".png")).write_bytes(png.getvalue())
    return (
        '<img alt="实测波形" src="data:image/png;base64,'
        + base64.b64encode(png.getvalue()).decode()
        + '">'
    )


def calibration_plots(rows, record: dict, directory: Path) -> str:
    """显示角度、前后修正误差、电零位与实际阶段；不冒充独立角度精度。"""
    import matplotlib.pyplot as plt
    import numpy as np

    x = rows[:, 1:]
    t = (rows[:, 0].astype(np.uint32) - np.uint32(rows[0, 0])).astype(np.uint32) / FREQUENCY
    name = record["step"]["id"]
    text = ""
    figure, axis = plt.subplots(figsize=(10, 3))
    for index, label in ((0, "raw"), (1, "directed"), (2, "linearized")):
        axis.plot(t, x[:, index] * 360 / 65536, label=label)
    axis.set(xlabel="board time (s)", ylabel="mechanical angle (deg)")
    axis.legend()
    axis.grid(True)
    text += embed_plot(figure, name + "_angles", directory)
    figure, axis = plt.subplots(figsize=(10, 3))
    for index, label in ((1, "before LUT"), (2, "after LUT")):
        error = np.angle(np.exp(1j * (x[:, index] * 2 * np.pi / 65536 - x[:, 3])))
        origin = np.angle(np.mean(np.exp(1j * error)))
        residual = np.angle(np.exp(1j * (error - origin))) * 180 / np.pi
        axis.scatter(x[:, 1] * 360 / 65536, residual, s=8, label=label)
    axis.set(xlabel="directed mechanical angle (deg)", ylabel="origin-aligned error (deg)")
    axis.legend()
    axis.grid(True)
    text += embed_plot(figure, name + "_errors", directory)
    figure, axes = plt.subplots(2, 1, figsize=(10, 5), constrained_layout=True)
    axes[0].plot(t, np.angle(np.exp(1j * x[:, 4])) * 180 / np.pi, label="electrical angle")
    axes[0].axhline(0, color="gray", linestyle="--")
    axes[0].set_ylabel("electrical angle (deg)")
    axes[1].plot(t, x[:, 5], label="Id")
    axes[1].plot(t, x[:, 6], label="Iq")
    axes[1].set(xlabel="board time (s)", ylabel="current (A)")
    for axis in axes:
        axis.legend()
        axis.grid(True)
    text += embed_plot(figure, name + "_zero", directory)
    states = record.get("states", [])
    if states:
        figure, axis = plt.subplots(figsize=(10, 2.5))
        ticks = np.array([state["tick"] for state in states], dtype=np.uint32)
        seconds = (ticks - ticks[0]).astype(np.uint32) / FREQUENCY
        axis.step(seconds, [state["flags"] >> 8 for state in states], where="post")
        axis.set(xlabel="board time (s), status polling", ylabel="calibration phase")
        axis.grid(True)
        text += embed_plot(figure, name + "_process", directory)
    return text


def render_report(directory: Path) -> None:
    """离线生成七项结果和实际波形；缺项固定显示NOT_RUN/BLOCKED。"""
    import matplotlib
    import numpy as np

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plan = json.loads((directory / "plan.json").read_text(encoding="utf-8"))
    results = {
        case: {
            "name": name,
            "status": "NOT_RUN",
            "steps": [],
            "deferred": plan.get("deferred", {}).get(case),
            "pending_scope": plan.get("pending_scope", {}).get(case),
        }
        for case, name in CASE_NAMES.items()
    }
    cards = []
    for metadata in sorted((directory / "raw").glob("*.json")):
        record = json.loads(metadata.read_text(encoding="utf-8"))
        if "step" not in record:
            continue
        step = record["step"]
        image_html, metrics = "", {}
        if record["status"] == "BLOCKED":
            item = {"id": step["id"], "status": "BLOCKED", "metrics": {}, "note": record["reason"]}
            results[step["case"]]["steps"].append(item)
            cards.append(
                "<article><h3>"
                + html.escape(step["id"])
                + " / BLOCKED</h3><p>"
                + html.escape(record["reason"])
                + "</p></article>"
            )
            continue
        if step.get("operation") == "protocol" and "timings_ms" not in record:
            status, metrics, note = assess_record(record, directory / "raw", plan)
            item = {
                "id": step["id"],
                "status": status,
                "metrics": metrics,
                "note": note,
            }
            results[step["case"]]["steps"].append(item)
            cards.append(
                "<article><h3>"
                + html.escape(step["id"])
                + "</h3><pre>"
                + html.escape(json.dumps(item, ensure_ascii=False, indent=2))
                + "</pre></article>"
            )
            continue
        if step.get("operation") == "protocol":
            timing = np.array(record["timings_ms"])
            status, metrics, note = assess_record(record, directory / "raw", plan)
            figure, axis = plt.subplots(figsize=(9, 3))
            axis.plot(timing)
            axis.set(xlabel="request", ylabel="latency (ms)")
        else:
            source = metadata.with_suffix(".csv")
            if step["group"] == "speed":
                source = source.with_name(source.stem + "_trend.csv")
            with source.open(encoding="utf-8") as stream:
                reader = csv.reader(stream)
                header = next(reader)
                rows = np.array([[float(v) for v in row] for row in reader])
            if rows.size == 0:
                status, note = "BLOCKED", "没有样本"
                figure = None
            else:
                status, metrics, note = assess_record(record, directory / "raw", plan)
                figure, axes = plt.subplots(2, 1, figsize=(10, 6), constrained_layout=True)
                elapsed = (rows[:, 0].astype(np.uint32) - np.uint32(rows[0, 0])).astype(
                    np.uint32
                ) / FREQUENCY
                for index in range(1, 8):
                    axes[0 if index <= 3 else 1].plot(elapsed, rows[:, index], label=header[index])
                for axis in axes:
                    axis.set_xlabel("board sample time (s)")
                    axis.legend(fontsize=8)
                    axis.grid(True)
                if step["group"] == "dq":
                    for axis in axes:
                        axis.clear()
                        axis.grid(True)
                    for index in (0, 2, 1, 3):
                        axes[0].plot(elapsed, rows[:, index + 1], label=header[index + 1])
                    axes[0].set_ylabel("current (A)")
                    axes[0].legend()
                    axes[1].plot(elapsed, rows[:, 3] - rows[:, 1], label="Id error")
                    axes[1].plot(elapsed, rows[:, 4] - rows[:, 2], label="Iq error")
                    axes[1].set(xlabel="board sample time (s)", ylabel="error (A)")
                    axes[1].legend()
                    if step["case"] == "TC04":
                        channel = 2 if step["drive"]["kind"] == "iq_pulse" else 1
                        changes = np.flatnonzero(np.abs(np.diff(rows[:, channel])) > 1e-6) + 1
                        if len(changes):
                            for axis in axes:
                                axis.axvline(elapsed[changes[0]], color="black", linestyle="--")
                elif step["group"] == "speed":
                    axes[1].clear()
                    axes[1].grid(True)
                    axes[1].plot(elapsed, rows[:, 4], label="Iq reference")
                    axes[1].plot(elapsed, rows[:, 5], label="Iq feedback")
                    axes[1].set(xlabel="board sample time (s)", ylabel="current (A)")
                    axes[1].legend()
                elif step["group"] == "phase":
                    axes[0].set_ylabel("ADC counts")
                    axes[1].clear()
                    for index in (4, 5, 6):
                        axes[1].plot(elapsed, rows[:, index], label=header[index])
                    axes[1].set(xlabel="board sample time (s)", ylabel="phase current (A)")
                    axes[1].grid(True)
                    axes[1].legend()
                elif step["group"] == "sincos":
                    axes[0].clear()
                    axes[0].plot(elapsed, rows[:, 1], color="C0", label="sin")
                    axes[0].set(xlabel="board sample time (s)", ylabel="sin ADC counts")
                    other = axes[0].twinx()
                    other.plot(elapsed, rows[:, 2], color="C1", label="cos")
                    other.set_ylabel("cos ADC counts")
                    axes[0].grid(True)
                    axes[0].legend(loc="upper left")
                    other.legend(loc="upper right")
                    axes[1].clear()
                    axes[1].plot(rows[:, 1], rows[:, 2])
                    axes[1].set(xlabel="sin (counts)", ylabel="cos (counts)")
                    axes[1].axis("equal")
                if (
                    record["status"] == "FAIL"
                    or record["final"]["fault"]
                    or record["final"]["invalid"]
                ):
                    status = "FAIL"
                    note += "；板端报告故障/截止/无效样本"
        if record.get("status") == "FAIL":
            status = "FAIL"
            note += "；本次执行已失败，原始数据不足不撤销失败结论"
        if figure is not None:
            figure.suptitle(step["id"] + " — measured data")
            png = io.BytesIO()
            figure.savefig(png, format="png", dpi=130)
            plt.close(figure)
            plots = directory / "plots"
            plots.mkdir(exist_ok=True)
            (plots / (step["id"] + ".png")).write_bytes(png.getvalue())
            image_html = (
                '<img alt="实测波形" src="data:image/png;base64,'
                + base64.b64encode(png.getvalue()).decode()
                + '">'
            )
        item = {"id": step["id"], "status": status, "metrics": metrics, "note": note}
        if step.get("group") == "angle" and rows.size:
            image_html += calibration_plots(rows, record, directory)
        results[step["case"]]["steps"].append(item)
        cards.append(
            "<article><h3>"
            + html.escape(step["id"] + " / " + status)
            + "</h3><p>"
            + html.escape(note)
            + "</p>"
            + '<p><a href="raw/'
            + html.escape(metadata.name, quote=True)
            + '">原始记录</a></p>'
            + image_html
            + "<pre>"
            + html.escape(json.dumps(metrics, ensure_ascii=False, indent=2))
            + "</pre></article>"
        )
    # LUT必须单独展示，不能把“受理/完成”文字当作角度波形。
    for source in sorted((directory / "raw").glob("*_lut.json")):
        table = json.loads(source.read_text(encoding="utf-8"))["values"]
        figure, axis = plt.subplots(figsize=(10, 3))
        axis.plot(np.arange(len(table)) * 360 / 1024, np.array(table) * 360 / 65536)
        axis.set(xlabel="mechanical angle (deg)", ylabel="LUT correction (deg)")
        cards.append(
            "<article><h3>"
            + html.escape(source.stem)
            + "</h3>"
            + embed_plot(figure, source.stem, directory)
            + "</article>"
        )
    for case, result in results.items():
        statuses = [step["status"] for step in result["steps"]]
        present = {step["id"] for step in result["steps"]}
        result["missing_steps"] = sorted(
            set(plan.get("required_steps", {}).get(case, [])) - present
        )
        result["status"] = (
            "FAIL"
            if "FAIL" in statuses
            else "NOT_RUN"
            if not statuses
            else "BLOCKED"
            if "BLOCKED" in statuses or result["missing_steps"] or result["pending_scope"]
            else "PASS"
        )
    failure_path = directory / "failure.json"
    if failure_path.exists():
        failure = json.loads(failure_path.read_text(encoding="utf-8"))
        if failure.get("step"):
            results[failure["step"]["case"]]["status"] = "FAIL"
        cards.insert(
            0,
            "<article><h3>本批次中止 / FAIL</h3><p>"
            + html.escape(failure["error"])
            + "</p></article>",
        )
    stop_failure_path = directory / "stop_failure.json"
    if stop_failure_path.exists():
        failure = json.loads(stop_failure_path.read_text(encoding="utf-8"))
        cards.insert(
            0,
            "<article><h3>停止未确认 / FAIL</h3><p>"
            + html.escape(failure["error"])
            + "</p></article>",
        )
    save_json(directory / "results.json", results)
    summary = "".join(
        "<tr><td>"
        + case
        + "</td><td>"
        + html.escape(value["name"])
        + "</td><td>"
        + value["status"]
        + "</td><td>"
        + html.escape(
            "；".join(
                filter(
                    None,
                    [
                        value["deferred"],
                        value["pending_scope"],
                        "缺少步骤：" + ", ".join(value["missing_steps"])
                        if value["missing_steps"]
                        else "",
                    ],
                )
            )
        )
        + "</td></tr>"
        for case, value in results.items()
    )
    document = (
        '<!doctype html><html lang="zh"><meta charset="utf-8"><title>统一电机测试报告</title><style>body{font:16px system-ui;margin:32px;max-width:1100px}td,th{padding:10px;border-bottom:1px solid #ddd;text-align:left}table{border-collapse:collapse;width:100%}img{max-width:100%}article{margin-top:28px;padding:16px;background:#f6f8fa}pre{white-space:pre-wrap}</style><h1>统一电机测试报告</h1><p>仅报告实际采集与已覆盖判据。延期项保留，未测不等于通过。</p><table><tr><th>编号</th><th>项目</th><th>已采集范围</th><th>延期/能力边界</th></tr>'
        + summary
        + "</table>"
        + "".join(cards)
        + "</html>"
    )
    (directory / "report.html").write_text(document, encoding="utf-8")


def main() -> None:
    """默认不打开设备；模板、报告和实机执行使用独立子命令。"""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    template = commands.add_parser("template", help="生成未批准的方案模板，不连接硬件")
    template.add_argument("--output", type=Path, required=True)
    report = commands.add_parser("report", help="仅从现有原始数据生成报告")
    report.add_argument("--input", type=Path, required=True)
    run = commands.add_parser("run", help="显式执行已批准方案")
    run.add_argument("--execute", action="store_true")
    run.add_argument("--plan", type=Path, required=True)
    run.add_argument("--firmware", type=Path, required=True)
    run.add_argument("--dll", type=Path, required=True)
    run.add_argument("--node", type=lambda value: int(value, 0), required=True)
    run.add_argument("--device-index", type=int, default=0)
    run.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "template":
        if args.output.exists():
            parser.error("模板目标已存在，拒绝覆盖")
        save_json(args.output, plan_template())
    elif args.command == "report":
        render_report(args.input)
    else:
        run_plan(args)


if __name__ == "__main__":
    main()
