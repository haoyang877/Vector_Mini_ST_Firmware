"""Windows USB-CAN 桌面工具。界面只负责参数与显示，后台任务独占 SDK。"""

import csv
import ctypes as C
import json
import math
import multiprocessing as mp
import os
import sys
import time
from pathlib import Path

from can_calibration_capture import HallRawCapture
from can_calibration_curves import (
    export_calibration_curves,
)
from can_motor_operations import (
    TraceBus,
    describe_faults,
    run_angle_calibration,
    run_clear_fault,
    run_speed_control,
    speed_value,
)
from can_upgrade_stress import read_node_uids, scan_upgrade_motors
from five_axis_can_stability import (
    ControlCanFd,
    batch_versions,
    controlcan_dll_path,
    run_queries,
)
from five_axis_jlink import FIRST_FIT_AXES
from hardware_check import Drive, Link, empty_report, run_check
from hardware_limits import read_limits
from motor_can_bitrate import configure as configure_bitrate
from ports.controlcanfd import _RXFD, _TXFD
from production_report import export_report

STATUS_REFRESH_S = 1.0

TITLE = "电机生产测试上位机 v2.1.6"
AXES = {3: "左轮", 4: "右轮", 5: "ROLL", 6: "PITCH", 7: "YAW", 8: "Lift"}
MODES = {"常规查询": "fixed", "阶梯压测": "ladder", "五轴稳定性验收": "acceptance"}
RATES = {"500K": 500000, "1M": 1000000, "2M": 2000000, "3M": 3000000, "4M": 4000000, "5M": 5000000}
AXIS_SPEED_LIMITS = {3: 30, 4: 30, 5: 30, 6: 300, 7: 300}
FD_LENGTHS = (*range(9), 12, 16, 20, 24, 32, 48, 64)
CHECK_DEFAULTS = {
    "node": "6",
    "mode": "静态和双向驱动",
    "ambient": "20",
    "current": "3",
    "frequency": "3",
    "leg": "1500",
    "travel": "360",
    "lease": "0",
    "target": "0",
}


def _number_field(text, label, kind=float):
    try:
        value = kind(text)
    except ValueError:
        unit = "整数" if kind is int else "数值"
        raise ValueError(f"{label}须填写有效{unit}") from None
    if kind is float and not math.isfinite(value):
        raise ValueError(f"{label}须为有限数值")
    return value


def motor_speed_fields(fields: dict[str, str]) -> dict:
    """解析独立速度面板，使用电机轴单位，不依赖开环检测的电流/电频默认值。"""
    result = dict(speed=speed_value(fields["speed"]), travel=None)
    limit = speed_value(fields.get("limit", "30"))
    if limit < 0.001:
        raise ValueError("速度上限至少为0.001 rad/s")
    if abs(result["speed"]) > limit:
        raise ValueError(f"目标速度超出本次速度上限±{limit:g} rad/s")
    result["speed_limit"] = limit
    for key, label, upper in (
        ("lease", "首次续租计数", 0xFFFFFFFF),
        ("target", "首次目标序号", 65535),
    ):
        text = fields.get(key, "0").strip()
        value = _number_field(text, label, int) if text else None
        if value is not None and not 0 <= value <= upper:
            raise ValueError(f"{label}须在0～{upper}内")
        result["last_" + key] = value
    return result


def hardware_check_fields(fields: dict[str, str], driven: bool) -> dict:
    """在打开适配器前解析检测输入；静态模式忽略拖动字段，空白必填项给出中文提示。"""
    values = {}
    for key, label, kind, required in (
        ("current", "模式10相电流（A）", float, True),
        ("frequency", "电频（Hz）", float, True),
        ("leg", "单程时间（ms）", int, True),
        ("travel", "两侧可用行程（输出轴角度，°）", float, True),
        ("ambient", "实测室温（°C）", float, False),
        ("lease", "上次续租计数", int, False),
        ("target", "上次目标序号", int, False),
    ):
        if required and not driven:
            continue
        text = fields.get(key, "0").strip()
        if not text:
            if required:
                raise ValueError(f"双向测试前请填写{label}")
            values[key] = None
            continue
        values[key] = _number_field(text, label, kind)
    drive = None
    if driven:
        drive = tuple(values[key] for key in ("current", "frequency", "leg", "travel"))
        Drive(*drive)
    if values["ambient"] is not None and not -40 <= values["ambient"] <= 120:
        raise ValueError("实测室温须在-40～120 °C内，检测时再按阈值配置核对")
    if values["lease"] is not None and not 0 <= values["lease"] <= 0xFFFFFFFF:
        raise ValueError("上次续租计数须在0～4294967295内")
    if values["target"] is not None and not 0 <= values["target"] <= 65535:
        raise ValueError("上次目标序号须在0～65535内")
    return dict(
        drive=drive,
        ambient=values["ambient"],
        last_lease=values["lease"],
        last_target=values["target"],
    )


def upgrade_motor_status(motor):
    if motor["role"] == 2:
        return "Loader · 等待升级"
    if motor.get("state") is None:
        return "APP · 已确认"
    states = {3: "自检", 4: "标定", 5: "空闲", 6: "运行", 8: "故障"}
    state = states.get(motor["state"], f"未知状态 {motor['state']}")
    return f"APP · {state}　{describe_faults(motor['faults'])}（0x{motor['faults']:08X}）"


def parse_raw(identifier: str, payload: str) -> tuple[int, bool, bytes]:
    """按输入宽度区分标准与扩展 ID，长度使用 CAN FD 实际 DLC。"""
    text = identifier.strip().lower().removeprefix("0x")
    if not 1 <= len(text) <= 8 or any(c not in "0123456789abcdef" for c in text):
        raise ValueError("CAN ID 请输入 1 至 8 位十六进制数")
    value = int(text, 16)
    if value > 0x1FFFFFFF:
        raise ValueError("CAN ID 超出 29 位范围")
    data = bytes.fromhex(payload)
    if len(data) not in FD_LENGTHS:
        raise ValueError("CAN FD 长度可为 0–8、12、16、20、24、32、48 或 64 字节")
    return value, len(text) > 3 or value > 0x7FF, data


def send_manual(dll: Path, out: Path, settings: dict, stop) -> dict:
    """发送用户指定的单帧并记录一秒内适配器交付的所有帧。"""
    identifier, extended, data = settings["frame"]
    received = []
    with os.add_dll_directory(str(dll.parent)), ControlCanFd(dll, **settings["bus"]) as port:
        tx = _TXFD()
        tx.frame.can_id = identifier
        tx.frame.eff = extended
        tx.frame.brs = 1
        tx.frame.length = len(data)
        tx.frame.data[: len(data)] = data
        if port.dll.ZCAN_TransmitFD(port.channel, C.byref(tx), 1) != 1:
            raise OSError("SDK 未受理手动发送")
        arr = (_RXFD * 64)()
        deadline = time.monotonic() + 1
        while not stop.is_set() and time.monotonic() < deadline:
            count = port.dll.ZCAN_ReceiveFD(port.channel, C.byref(arr), len(arr), 0)
            if not 0 <= count <= len(arr):
                raise OSError("SDK 接收失败")
            for item in arr[:count]:
                f = item.frame
                if f.length > 64:
                    raise ValueError("SDK 返回非法帧长")
                received.append(
                    (
                        item.timestamp,
                        hex(f.can_id),
                        f.eff,
                        f.brs,
                        f.err,
                        f.esi,
                        f.rtr,
                        bytes(f.data[: f.length]).hex(" "),
                    )
                )
            if not count:
                stop.wait(0.001)
    with (out / "frames.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("adapter_us", "can_id", "extended", "brs", "err", "esi", "rtr", "data"))
        writer.writerows(received)
    return {
        "profile": "manual",
        "sdk_accepted": True,
        "can_id": hex(identifier),
        "extended": extended,
        "data": data.hex(" "),
        "channel": settings["bus"]["channel_index"],
        "can_nominal_bps": settings["bus"]["nominal_bps"],
        "can_data_bps": settings["bus"]["data_bps"],
        "transmit_type": 0,
        "cancelled": stop.is_set(),
        "received": len(received),
        "sample": received[:12],
    }


def desktop_uids(resource, log_root):
    batch = json.loads((resource / "five_axis_can_stability.json").read_text(encoding="utf-8"))
    uids = {r["node"]: r["uid"] for r in batch["motors"]}
    for path in sorted(log_root.glob("first-fit-*.json"), key=lambda p: p.stat().st_mtime_ns):
        binding = json.loads(path.read_text(encoding="utf-8"))
        uids[binding["node"]] = binding["uid"]
    return uids


def check_current_motor(bus, settings: dict) -> None:
    """启相前核对共用选择绑定的UID，换板后要求重新识别。"""
    if settings.get("expected_uid"):
        info = Link(bus, settings["node"], timeout_s=0.5).request(1, page=0)
        if len(info) != 32 or info[:12].hex() != settings["expected_uid"]:
            raise ValueError("当前节点的电机UID已改变，请重新扫描识别")


def acquire(settings: dict, out: Path, events, stop, commands=None) -> None:
    """子进程入口。适配器和日志释放之后才通知界面结束。"""
    try:
        resource = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
        dll = controlcan_dll_path(resource)
        if settings["profile"] in ("motor_calibration", "motor_speed"):
            parent = mp.parent_process()

            def operation_stopped():
                return stop.is_set() or (parent is not None and not parent.is_alive())

            with (
                (out / "frames.csv").open("w", newline="", encoding="utf-8") as stream,
                os.add_dll_directory(str(dll.parent)),
                ControlCanFd(dll, **settings["bus"]) as port,
            ):
                bus = TraceBus(port, stream)
                check_current_motor(bus, settings)
                if settings["profile"] == "motor_calibration":
                    capture = HallRawCapture(out)
                    try:
                        report = run_angle_calibration(
                            bus,
                            settings["node"],
                            kind=settings["calibration_kind"],
                            stopped=operation_stopped,
                            progress=lambda value: events.put(("stage", value)),
                            on_identified=capture.start,
                        )
                    finally:
                        raw_capture = capture.finish()
                    report["raw_capture"] = raw_capture
                    report["archive"] = settings.get("archive", {})
                    if report.get("passed") and report.get("stop_confirmed"):
                        events.put(("stage", "标定已保存，正在只读回读补偿表并绘图"))
                        try:
                            if capture.thread is not None and capture.thread.is_alive():
                                raise OSError(raw_capture["error"])
                            report["curves"] = export_calibration_curves(
                                bus, report, out, report["archive"].get("name", "")
                            )
                        except Exception as exc:
                            report["curve_error"] = str(exc)
                else:
                    report = run_speed_control(
                        bus,
                        settings["node"],
                        settings["speed"],
                        settings["travel"],
                        commands,
                        out,
                        previous=settings["last_lease"],
                        previous_target=settings["last_target"],
                        speed_limit=settings.get("speed_limit"),
                        stopped=operation_stopped,
                        progress=lambda value: events.put(("stage", value)),
                        feedback=lambda value: events.put(("motor_feedback", value)),
                    )
            (out / "summary.json").write_text(
                json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        elif settings["profile"] == "clear_fault":
            with (
                (out / "frames.csv").open("w", newline="", encoding="utf-8") as stream,
                os.add_dll_directory(str(dll.parent)),
                ControlCanFd(dll, **settings["bus"]) as port,
            ):
                report = run_clear_fault(
                    TraceBus(port, stream),
                    settings["node"],
                    settings["expected_uid"],
                    stopped=stop.is_set,
                    progress=lambda value: events.put(("stage", value)),
                )
            (out / "summary.json").write_text(
                json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        elif settings["profile"] == "can_scan":
            with os.add_dll_directory(str(dll.parent)), ControlCanFd(dll, **settings["bus"]) as bus:
                motors, issues = scan_upgrade_motors(
                    bus, stop.is_set, lambda name: events.put(("stage", name))
                )
            report = dict(settings, motors=motors, issues=issues, cancelled=stop.is_set())
            (out / "summary.json").write_text(
                json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        elif settings["profile"] == "can_bitrate":
            report = dict(settings, results=[], **{"pass": False})
            try:
                with (
                    os.add_dll_directory(str(dll.parent)),
                    ControlCanFd(dll, **settings["bus"]) as bus,
                ):
                    for target in settings["targets"]:
                        if stop.is_set():
                            break
                        node = target["node"]
                        if read_node_uids(bus, node) != {bytes.fromhex(target["uid"])}:
                            raise ValueError(f"节点 {node} 身份与扫描结果不符，请重新扫描")
                        value = configure_bitrate(
                            bus, node, settings["set"], log=lambda msg: events.put(("stage", msg))
                        )
                        report["results"].append({"node": node, **value})
                report["cancelled"] = len(report["results"]) != len(settings["targets"])
                report["pass"] = not report["cancelled"]
            except (OSError, ValueError, RuntimeError) as exc:
                report["fatal"] = str(exc)
            (out / "summary.json").write_text(
                json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
            )
        elif settings["profile"] == "hardware_check":
            limits = read_limits()
            (out / "hardware_check_limits.json").write_text(
                json.dumps(limits, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            with (
                (out / "frames.csv").open("w", newline="", encoding="utf-8") as stream,
                os.add_dll_directory(str(dll.parent)),
                ControlCanFd(dll, **settings["bus"]) as port,
            ):
                bus = TraceBus(port, stream)
                check_current_motor(bus, settings)
                report = run_check(
                    bus,
                    settings["node"],
                    drive=Drive(*settings["drive"]) if settings["drive"] else None,
                    resistance_only=settings.get("resistance_only", False),
                    ambient=settings["ambient"],
                    previous=settings["last_lease"],
                    previous_target=settings["last_target"],
                    stopped=stop.is_set,
                    progress=lambda msg: events.put(("stage", msg)),
                    phase_progress=lambda phases: events.put(("hardware_phases", phases)),
                    static_progress=lambda snapshot: events.put(("hardware_static", snapshot)),
                    limits=limits,
                )
            selection = FIRST_FIT_AXES.get(settings["node"])
            report.update(
                axis=selection[0] if selection else "未知",
                model=selection[1] if selection else "未知",
            )
            try:
                export_report(report, out, tool_version="2.1.6")
                report["production_report"] = "production-report.html"
            except (OSError, ValueError, KeyError) as exc:
                report["report_error"] = str(exc)
            (out / "summary.json").write_text(
                json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
            )
        elif settings["profile"] == "manual":
            report = send_manual(dll, out, settings, stop)
            (out / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        else:
            batch = json.loads(
                (resource / "five_axis_can_stability.json").read_text(encoding="utf-8")
            )
            uids = desktop_uids(resource, out.parent)
            uids.update(settings.get("node_uids", {}))
            run_queries(
                dll,
                out,
                {n: bytes.fromhex(uids.get(n, "")) for n in settings["nodes"]},
                batch["hex_sha256"],
                versions=batch_versions(batch),
                profile=settings["profile"],
                per_node=settings["count"],
                rate_hz=settings["rate"],
                **settings["bus"],
                stopped=stop.is_set,
                on_progress=lambda value: events.put(("progress", value)),
            )
            report = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    except Exception as exc:
        report = (
            empty_report(settings["node"], settings["ambient"], str(exc))
            if settings["profile"] == "hardware_check"
            else {"profile": settings["profile"], "fatal": str(exc), "pass": False}
        )
        (out / "summary.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )
    events.put(("done", report))
