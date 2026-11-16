"""Windows USB-CAN 桌面工具。界面只负责参数与显示，后台任务独占 SDK。"""

import csv
import ctypes as C
import json
import math
import multiprocessing as mp
import os
import queue
import sys
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from can_calibration_capture import HallRawCapture
from can_calibration_curves import (
    ARCHIVE_TYPES,
    calibration_archive,
    export_calibration_curves,
    model_directory_name,
)
from can_motor_operations import (
    TraceBus,
    describe_faults,
    run_angle_calibration,
    run_clear_fault,
    run_speed_control,
    speed_value,
)
from can_parameter_page import ParameterPage
from can_parameters import value_text
from can_session import CanSession, SessionTask
from can_speed_plot import SpeedPlot
from can_upgrade_stress import read_node_uids, run_upgrade_test, scan_upgrade_motors
from five_axis_can_stability import (
    PER_NODE,
    ControlCanFd,
    batch_versions,
    controlcan_dll_path,
    encode_frames,
    run_queries,
    version_text,
)
from five_axis_jlink import FIRST_FIT_AXES, run_jlink
from hardware_check import Drive, Link, empty_report, run_check, summarize
from motor_can_bitrate import configure as configure_bitrate
from ports.controlcanfd import _RXFD, _TXFD
from production_flow import ProductionFlow
from production_report import build_report, export_report, failure_summary, report_rows

STATUS_REFRESH_S = 1.0

TITLE = "电机生产测试上位机 v1.23.0"
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
    if values["ambient"] is not None and not 0 <= values["ambient"] <= 60:
        raise ValueError("实测室温须在0～60 °C内")
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
                )
            selection = FIRST_FIT_AXES.get(settings["node"])
            report.update(
                axis=selection[0] if selection else "未知",
                model=selection[1] if selection else "未知",
            )
            try:
                export_report(report, out, tool_version="1.23.0")
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


class CanTestWindow:
    """通信压测与手动帧共用设备选择、进度和结果目录。"""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title(TITLE)
        width = min(1200, self.root.winfo_screenwidth() - 80)
        height = min(900, self.root.winfo_screenheight() - 100)
        self.root.minsize(min(1080, width), min(700, height))
        self.root.geometry(f"{width}x{height}")
        self.process = None
        self.can_session = None
        self.pending_start = None
        self.release_ok = False
        self.exit_polls = 0
        self.auto_status_task = False
        self.next_status_refresh = 0.0
        self.production_flow = ProductionFlow()
        self.pending_next_device = False
        self.status_timer = None
        self.events = None
        self.commands = None
        self.active_profile = None
        self.stop_signal = None
        self.production_report_path = None
        self.report_dir = None
        self.close_pending = False
        self.curve_path = None
        self.done = False
        self.controls = []
        self.identity = None
        self.current_motor = None
        self.known_motors = {}
        self.previous_node = 6
        self.plot_identity = None
        self.first_fit_selected = False
        self.swd_busy = False
        self.scanning = False
        frame = ttk.Frame(root, padding=16)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="电机生产测试上位机", font=("Microsoft YaHei UI", 17, "bold")).pack(
            anchor="w", pady=(0, 8)
        )
        header = ttk.Frame(frame)
        header.pack(fill="x", pady=(0, 8))
        header.columnconfigure(0, weight=1)
        header.columnconfigure(1, weight=1)
        header.columnconfigure(2, weight=1)
        connection = ttk.LabelFrame(header, text="1. 通信连接", padding=8)
        connection.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        motor_panel = ttk.LabelFrame(header, text="2. 当前电机", padding=8)
        motor_panel.grid(row=0, column=1, sticky="nsew", padx=(0, 8))
        summary = ttk.LabelFrame(header, text="3. 设备状态", padding=8)
        summary.grid(row=0, column=2, sticky="nsew")
        self.device_versions = tk.StringVar(value="APP：—    Loader：—")
        self.device_state = tk.StringVar(value="状态：未读取")
        self.device_measurements = tk.StringVar(
            value="母线电压：— V    母线电流：— A\n电机温度：— °C"
        )
        self.device_checked = tk.StringVar(value="读取设备后显示实际版本与状态")
        ttk.Label(summary, textvariable=self.device_versions).pack(anchor="w")
        ttk.Label(summary, textvariable=self.device_state, wraplength=360).pack(anchor="w", pady=2)
        ttk.Label(summary, textvariable=self.device_measurements).pack(anchor="w", pady=2)
        ttk.Label(summary, textvariable=self.device_checked).pack(anchor="w")
        state_actions = ttk.Frame(summary)
        state_actions.pack(anchor="w", pady=(5, 0))
        self.status_refresh = ttk.Button(
            state_actions, text="刷新状态", command=lambda: self.start(status_only=True)
        )
        self.status_refresh.pack(side="left")
        self.clear_fault_button = ttk.Button(
            state_actions, text="清除故障", command=lambda: self.start(clear_fault=True)
        )
        self.clear_fault_button.pack(side="left", padx=(6, 0))
        self.can_disconnect = ttk.Button(connection, text="断开 CAN", command=self.release_can)
        self.auto_status = tk.BooleanVar(value=True)
        self.connection_state = tk.StringVar(value="CAN：未连接")
        top = ttk.Frame(connection)
        top.pack(fill="x")

        def combo(parent, label, values, default, width):
            ttk.Label(parent, text=label).pack(side="left")
            var = tk.StringVar(value=default)
            w = ttk.Combobox(parent, textvariable=var, values=values, width=width, state="readonly")
            w.pack(side="left", padx=(4, 12))
            self.controls.append((w, "readonly"))
            return var

        self.channel = combo(top, "通道", ("1", "2"), "1", 3)
        rates = ttk.Frame(connection)
        rates.pack(fill="x", pady=5)
        self.nominal = combo(rates, "仲裁段", ("500K", "1M"), "1M", 5)
        self.data = combo(rates, "数据段", tuple(RATES), "5M", 5)
        ttk.Label(connection, text="ISO CAN FD · BRS").pack(anchor="w")
        link_actions = ttk.Frame(connection)
        link_actions.pack(fill="x", pady=4)
        self.can_disconnect.pack(in_=link_actions, side="left")
        ttk.Label(link_actions, textvariable=self.connection_state).pack(side="left", padx=8)
        ttk.Checkbutton(connection, text="连接后自动刷新（1秒）", variable=self.auto_status).pack(
            anchor="w"
        )
        self.axis_choices = {
            f"{role} · {model} · 节点 {node}": node
            for node, (role, model, _) in FIRST_FIT_AXES.items()
        }
        motor_row = ttk.Frame(motor_panel)
        motor_row.pack(fill="x", pady=(0, 8))
        self.motor_axis = combo(
            motor_row,
            "电机",
            tuple(self.axis_choices),
            next(label for label, node in self.axis_choices.items() if node == 6),
            26,
        )
        self.motor_selector = next(
            child for child in motor_row.winfo_children() if isinstance(child, ttk.Combobox)
        )
        self.motor_selector.bind("<<ComboboxSelected>>", self.select_first_fit)
        scan_motor = ttk.Button(motor_row, text="扫描识别", command=lambda: self.start(scan=True))
        scan_motor.pack(side="left")
        self.controls.append((scan_motor, "normal"))
        self.current_motor_label = tk.StringVar(value="未识别；请选择电机后扫描。")
        ttk.Label(motor_panel, textvariable=self.current_motor_label, wraplength=370).pack(
            anchor="w", pady=4
        )
        flow = ttk.LabelFrame(frame, text="本台生产流程（只统计本次操作记录）", padding=6)
        flow.pack(fill="x", pady=(0, 8))
        self.flow_summary = tk.StringVar(value=self.production_flow.summary(None))
        ttk.Label(flow, textvariable=self.flow_summary, wraplength=950).pack(
            side="left", fill="x", expand=True
        )
        self.next_device_button = ttk.Button(
            flow, text="归档并开始下一台", command=self.next_device
        )
        self.next_device_button.pack(side="right", padx=6)
        self.workspace = ttk.Panedwindow(frame, orient="vertical")
        self.workspace.pack(fill="both", expand=True)
        self.task_area = ttk.Frame(self.workspace)
        self.record_area = ttk.LabelFrame(
            self.workspace, text="运行记录 / 检测结果（拖动上方分隔条调整）", padding=6
        )
        self.workspace.add(self.task_area, weight=3)
        self.workspace.add(self.record_area, weight=1)
        navigation_style = ttk.Style(root)
        navigation_style.configure("Production.TNotebook", tabposition="wn")
        self.tabs = ttk.Notebook(self.task_area, style="Production.TNotebook")
        self.tabs.pack(fill="both", expand=True)
        self.sections = {}
        for key, title in (
            ("maintenance", "设备维护"),
            ("production", "生产检测"),
            ("calibration", "电机标定"),
            ("parameters", "参数配置"),
            ("records", "测试记录"),
        ):
            section = self.sections[key] = ttk.Frame(self.tabs, padding=8)
            self.tabs.add(section, text=title)
        self.maintenance_tabs = ttk.Notebook(self.sections["maintenance"])
        self.maintenance_tabs.pack(fill="both", expand=True)
        self.calibration_tabs = ttk.Notebook(self.sections["calibration"])
        self.calibration_tabs.pack(fill="both", expand=True)
        self.pages = {}
        for key, title in (
            ("jlink", "整包烧录"),
            ("upgrade", "CAN 固件升级"),
            ("communication", "通信测试"),
            ("advanced", "高级维护"),
        ):
            page = self.pages[key] = ttk.Frame(self.maintenance_tabs, padding=10)
            self.maintenance_tabs.add(page, text=title)
        for key, title in (("calibration", "角度与零位标定"), ("speed", "速度调试")):
            page = self.pages[key] = ttk.Frame(self.calibration_tabs, padding=10)
            self.calibration_tabs.add(page, text=title)
        self.pages["hardware"] = self.sections["production"]
        self.pages["parameters"] = self.sections["parameters"]
        self.pages["records"] = self.sections["records"]
        record_actions = ttk.Frame(self.pages["records"])
        record_actions.pack(fill="x", pady=5)
        ttk.Label(record_actions, text="本次启动后完成的任务；历史记录可打开记录根目录查看。").pack(
            side="left"
        )
        ttk.Button(record_actions, text="打开记录根目录", command=self.open_record_root).pack(
            side="right"
        )
        self.record_paths = {}
        self.record_table = ttk.Treeview(
            self.pages["records"],
            columns=("time", "task", "node", "result"),
            show="headings",
            height=10,
        )
        for key, title in (
            ("time", "完成时间"),
            ("task", "任务"),
            ("node", "节点"),
            ("result", "结果摘要"),
        ):
            self.record_table.heading(key, text=title)
            self.record_table.column(key, width=150 if key != "result" else 400)
        self.record_table.pack(fill="both", expand=True)
        self.record_table.bind("<Double-1>", lambda _: self.open_selected_record())
        ttk.Button(
            self.pages["records"], text="打开选中记录目录", command=self.open_selected_record
        ).pack(anchor="w", pady=5)
        self.go_detection = ttk.Button(
            self.pages["jlink"],
            text="烧录及回读通过 · 进入生产检测",
            command=lambda: self.navigate("hardware"),
            state="disabled",
        )
        self.go_detection.pack(anchor="w", side="bottom", pady=8)
        self.install_verified_uid = None
        self.swd_tab = self.pages["jlink"]
        check_tab = self.pages["hardware"]
        upgrade_tab = self.pages["upgrade"]
        self.communication_tabs = ttk.Notebook(self.pages["communication"])
        self.communication_tabs.pack(fill="both", expand=True)
        self.query_tab = ttk.Frame(self.communication_tabs, padding=10)
        self.raw_tab = ttk.Frame(self.communication_tabs, padding=10)
        self.communication_tabs.add(self.query_tab, text="通信压测")
        self.communication_tabs.add(self.raw_tab, text="协议发送")
        stages = ttk.LabelFrame(check_tab, text="本轮阶段 · 阶段结束不代表检测合格", padding=7)
        stages.pack(fill="x", pady=(0, 8))
        self.hardware_phases = {}
        for index, (key, name) in enumerate(
            (
                ("communication", "通信"),
                ("static", "静态"),
                ("resistance", "相电阻"),
                ("dynamic", "双向动态"),
                ("finalize", "关相 / 结果"),
            )
        ):
            var = tk.StringVar(value=f"{index + 1}. {name} · 待执行")
            ttk.Label(stages, textvariable=var).grid(row=0, column=index, sticky="w", padx=6)
            stages.columnconfigure(index, weight=1)
            self.hardware_phases[key] = (name, var)
        self.hardware_failure = tk.StringVar(value="尚未开始本轮检测")
        failure_box = ttk.LabelFrame(check_tab, text="本轮结论 / 失败摘要", padding=7)
        failure_box.pack(fill="x", pady=(0, 8))
        self.hardware_failure_label = tk.Text(
            failure_box,
            height=4,
            wrap="word",
            state="disabled",
            foreground="#8a6100",
            font=("Microsoft YaHei UI", 10),
            borderwidth=0,
        )
        summary_scroll = ttk.Scrollbar(failure_box, command=self.hardware_failure_label.yview)
        self.hardware_failure_label.configure(yscrollcommand=summary_scroll.set)
        summary_scroll.pack(side="right", fill="y")
        self.hardware_failure_label.pack(side="left", fill="x", expand=True)
        self.hardware_failure.trace_add("write", self.update_failure_text)
        self.update_failure_text()
        check_row = ttk.Frame(check_tab)
        check_row.pack(anchor="w")
        self.check_mode = combo(
            check_row,
            "检测方式",
            ("关相静态", "静态＋相电阻", "静态和双向驱动"),
            CHECK_DEFAULTS["mode"],
            16,
        )
        self.check_fields = {}
        for row in (
            (
                ("ambient", "实测室温 °C（可空）"),
                ("current", "模式10相电流 A"),
                ("frequency", "电频 Hz"),
            ),
            (
                ("leg", "单程 ms"),
                ("travel", "两侧可用行程 °（双向必填）"),
            ),
        ):
            box = ttk.Frame(check_tab)
            box.pack(anchor="w", pady=3)
            for key, label in row:
                ttk.Label(box, text=label).pack(side="left")
                entry = ttk.Entry(box, width=7)
                entry.insert(0, CHECK_DEFAULTS[key])
                entry.pack(side="left", padx=(3, 9))
                self.check_fields[key] = entry
                self.controls.append((entry, "normal"))
        self.resource = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
        calibration_tab = self.pages["calibration"]
        calibration_row = ttk.Frame(calibration_tab)
        calibration_row.pack(anchor="w", pady=5)
        ttk.Label(calibration_row, text="样机序列号").pack(side="left")
        self.archive_serial = tk.StringVar(value="0001")
        model_entry = ttk.Entry(calibration_row, textvariable=self.archive_serial, width=14)
        model_entry.pack(side="left", padx=5)
        self.controls.append((model_entry, "normal"))
        ttk.Label(calibration_row, text="测试序列号").pack(side="left")
        self.test_serial = tk.StringVar(value="0001")
        test_entry = ttk.Entry(calibration_row, textvariable=self.test_serial, width=14)
        test_entry.pack(side="left", padx=5)
        self.controls.append((test_entry, "normal"))
        calibration_actions = ttk.Frame(calibration_tab)
        calibration_actions.pack(anchor="w", pady=8)
        for kind, label in ((1, "编码器 / Hall 角度标定并保存"), (2, "电角零位标定并保存")):
            button = ttk.Button(
                calibration_actions,
                text=label,
                command=lambda k=kind: self.start(calibration_kind=k),
            )
            button.pack(side="left", padx=(0, 12))
            self.controls.append((button, "normal"))
        self.curve_button = ttk.Button(
            calibration_actions, text="查看本次曲线", command=self.show_calibration_curves
        )
        self.curve_button.pack(side="left")
        ttk.Label(
            calibration_tab,
            text="编码器/Hall角度沿用固件算法；减速轴自动联合电角零位。电机会旋转，完成后自动保存。",
        ).pack(anchor="w", pady=5)
        ttk.Label(
            calibration_tab,
            text="按实际票据和Flash修订确认结果；标定请求超时500 ms。点击停止可取消并确认关相。",
        ).pack(anchor="w", pady=5)
        ttk.Label(
            calibration_tab,
            text="连接J-Link可采集Hall原码，并绘制补偿表、零位、对齐角和两路极值；保存PNG/CSV。",
        ).pack(anchor="w", pady=5)
        speed_tab = self.pages["speed"]
        speed_top = ttk.Frame(speed_tab)
        speed_top.pack(fill="x", pady=5)
        speed_actions = ttk.Frame(speed_top)
        speed_actions.pack(side="right", anchor="n")
        self.speed_start = ttk.Button(speed_actions, text="使能速度控制", command=self.start)
        self.speed_start.pack(side="left", padx=4)
        self.speed_stop = ttk.Button(speed_actions, text="停止", command=self.stop)
        self.speed_stop.pack(side="left", padx=4)
        self.speed_open = ttk.Button(
            speed_actions, text="打开本次记录", command=lambda: os.startfile(self.report_dir)
        )
        self.speed_open.pack(side="left", padx=4)
        motor_values = ttk.Frame(speed_top)
        motor_values.pack(side="left", anchor="n")
        self.motor_fields = {}
        for key, label, default in (
            ("speed", "电机轴目标速度 rad/s", "3"),
            ("limit", "速度上限 rad/s", "300"),
        ):
            ttk.Label(motor_values, text=label).pack(side="left")
            entry = ttk.Entry(motor_values, width=8)
            entry.insert(0, default)
            entry.pack(side="left", padx=(3, 8))
            self.motor_fields[key] = entry
            self.controls.append((entry, "normal"))
        self.speed_update = ttk.Button(speed_tab, text="更新速度目标", command=self.update_speed)
        self.speed_update.pack(anchor="w", pady=5)
        self.motor_feedback = tk.StringVar(value="使能前保持停机；速度以电机轴rad/s表示。")
        ttk.Label(speed_tab, textvariable=self.motor_feedback).pack(anchor="w", pady=5)
        ttk.Label(
            speed_tab,
            text="持续旋转直到点击停止；目标0为停转保持。断联或故障停机，电流限值和PI保持。",
        ).pack(anchor="w", pady=3)
        ttk.Label(
            speed_tab,
            text="速度上限按轴配置：ROLL/左右轮30，PITCH/YAW300 rad/s；计数按UID自动保存。",
        ).pack(anchor="w", pady=3)
        self.speed_plot = SpeedPlot(speed_tab)
        self.speed_plot.pack(fill="both", expand=True, pady=(6, 0))
        ttk.Label(
            speed_tab,
            text="参考/反馈来自同一帧；速度模式无位置参考时显示缺失。原始点保存到speed.csv。",
        ).pack(anchor="w")
        self.parameter_page = ParameterPage(
            self.pages["parameters"],
            lambda group, save: self.start(parameter_action=(group, save)),
        )
        self.parameter_page.pack(fill="both", expand=True)
        self.builtin_package = (
            self.resource / "can-app"
            if getattr(sys, "frozen", False)
            else self.resource.parents[1] / "outputs/AT32F456-FULL-v1.8.12/app-package"
        )
        self.upgrade_package = self.builtin_package
        self.upgrade_package_name = tk.StringVar(value="内置 APP 1.8.12")
        pkg_row = ttk.Frame(upgrade_tab)
        pkg_row.pack(anchor="w")
        ttk.Label(pkg_row, textvariable=self.upgrade_package_name).pack(side="left")
        choose_pkg = ttk.Button(
            pkg_row, text="选择 CAN 升级包", command=self.choose_upgrade_package
        )
        choose_pkg.pack(side="left", padx=6)
        self.controls.append((choose_pkg, "normal"))
        axis_row = ttk.Frame(upgrade_tab)
        axis_row.pack(anchor="w", pady=5)
        self.upgrade_targets = {}
        scan = ttk.Button(axis_row, text="扫描在线电机", command=lambda: self.start(scan=True))
        scan.pack(side="left", padx=(0, 8))
        self.controls.append((scan, "normal"))
        self.batch_upgrade = tk.BooleanVar(value=False)
        batch_upgrade = ttk.Checkbutton(
            axis_row, text="批量升级", variable=self.batch_upgrade, command=self.update_controls
        )
        batch_upgrade.pack(side="left", padx=5)
        self.controls.append((batch_upgrade, "normal"))
        self.upgrade_info = tk.StringVar(
            value="默认使用顶部当前电机；多台操作请扫描后启用批量升级。"
        )
        ttk.Label(axis_row, textvariable=self.upgrade_info).pack(side="left")
        grid = ttk.Frame(upgrade_tab)
        grid.pack(fill="x", pady=(0, 5))
        self.upgrade_selected = {}
        self.upgrade_checks = {}
        self.upgrade_lights = {}
        self.upgrade_details = {}
        self.upgrade_results = {}
        for col, title in enumerate(("升级目标", "连接", "最近识别", "芯片 UID", "本台进度")):
            ttk.Label(grid, text=title).grid(row=0, column=col, sticky="w", padx=(0, 10))
        self.upgrade_uids = {}
        for row, node in enumerate(FIRST_FIT_AXES, start=1):
            var = self.upgrade_selected[node] = tk.BooleanVar(value=False)
            check = self.upgrade_checks[node] = ttk.Checkbutton(
                grid, text=f"{AXES[node]} · {node}", variable=var, command=self.update_controls
            )
            check.grid(row=row, column=0, sticky="w", padx=(0, 8))
            light = self.upgrade_lights[node] = ttk.Label(
                grid, text="● 未扫描", foreground="#656565"
            )
            light.grid(row=row, column=1, sticky="w", padx=(0, 10))
            for col, labels in (
                (2, self.upgrade_details),
                (3, self.upgrade_uids),
                (4, self.upgrade_results),
            ):
                labels[node] = ttk.Label(grid, text="—")
                labels[node].grid(row=row, column=col, sticky="w", padx=(0, 8))
        bitrate_row = ttk.Frame(self.pages["advanced"])
        bitrate_row.pack(anchor="w", pady=4)
        self.target_nominal = combo(bitrate_row, "电机仲裁段", ("500K", "1M"), "1M", 6)
        self.target_data = combo(bitrate_row, "数据段", tuple(RATES), "5M", 6)
        self.bitrate_read = ttk.Button(
            bitrate_row, text="读取波特率", command=lambda: self.start(bitrate_action="read")
        )
        self.bitrate_save = ttk.Button(
            bitrate_row, text="保存波特率", command=lambda: self.start(bitrate_action="save")
        )
        self.bitrate_read.pack(side="left", padx=4)
        self.bitrate_save.pack(side="left", padx=4)
        ttk.Label(bitrate_row, text="保存后重启生效").pack(side="left", padx=4)
        count_row = ttk.Frame(upgrade_tab)
        count_row.pack(anchor="w")
        ttk.Label(count_row, text="每台升级次数").pack(side="left")
        self.upgrade_rounds = tk.StringVar(value="1")
        rounds_entry = ttk.Entry(count_row, textvariable=self.upgrade_rounds, width=8)
        rounds_entry.pack(side="left", padx=5)
        self.controls.append((rounds_entry, "normal"))
        self.single_frame_writes = tk.BooleanVar(value=False)
        compatibility = ttk.Checkbutton(
            count_row, text="兼容单帧写入", variable=self.single_frame_writes
        )
        compatibility.pack(side="left", padx=5)
        self.controls.append((compatibility, "normal"))
        ttk.Label(count_row, text="升级期间请暂停其他主站轮询。").pack(side="left")
        install = ttk.LabelFrame(self.swd_tab, text="新板首次烧录 / 已配板更新", padding=10)
        install.pack(fill="x", pady=(0, 8))
        self.install_phase = tk.StringVar(
            value="① 识别板卡 → ② 确认电机类型 → ③ 烧录整包 → ④ 启动与回读"
        )
        ttk.Label(
            install, textvariable=self.install_phase, font=("Microsoft YaHei UI", 11, "bold")
        ).pack(anchor="w")
        self.install_hint = tk.StringVar(value="先连接 J-Link 和板卡电源，再点击“1. 识别板卡”。")
        ttk.Label(install, textvariable=self.install_hint, wraplength=1000).pack(anchor="w", pady=5)
        self.install_flash = ttk.Button(
            install, text="3. 烧录整包", command=self.start, state="disabled"
        )
        self.install_flash.pack(side="right", padx=5)
        ttk.Label(
            self.swd_tab, text="内置整包：AT32F456 · Loader 2.3.4 + APP 1.8.12 · CAN FD 1M/5M"
        ).pack(anchor="w")
        embedded = (
            Path(getattr(sys, "_MEIPASS", Path(__file__).parent)) / "firmware" / "five-axis.hex"
        )
        self.hex_path = embedded if embedded.is_file() else None
        self.hex_name = tk.StringVar(
            value="本批统一烧录包（内置）" if self.hex_path else "请选择本批合并 HEX"
        )
        files = ttk.Frame(self.swd_tab)
        files.pack(anchor="w", pady=5)
        ttk.Label(files, textvariable=self.hex_name).pack(side="left")
        choose = ttk.Button(files, text="选择 HEX", command=self.choose_hex)
        choose.pack(side="left", padx=8)
        self.controls.append((choose, "normal"))
        ttk.Label(
            self.swd_tab, text="每次重新烧录并校验；完成后自动复位，读取 APP / Loader 版本。"
        ).pack(anchor="w")
        self.firmware_version = tk.StringVar(value="板端版本：尚未查询")
        ttk.Label(self.swd_tab, textvariable=self.firmware_version).pack(anchor="w")
        self.motor_label = tk.StringVar(
            value="接好已停转电机的 J-Link，点击“烧录固件”；自动识别已配板或新板。"
        )
        ttk.Label(self.swd_tab, textvariable=self.motor_label).pack(anchor="w")
        setup_row = ttk.Frame(self.swd_tab)
        setup_row.pack(anchor="w", pady=4)
        self.rescan = ttk.Button(
            setup_row, text="1. 识别板卡 / 更换板卡", command=self.identify_again
        )
        self.rescan.pack(side="left")
        ttk.Label(
            setup_row, text="已配板保留参数；新板识别后在下方选择类型并初装，无需先擦除。"
        ).pack(side="left", padx=8)
        self.initial_setup = ttk.LabelFrame(
            self.swd_tab, text="2. 新板初装：选择实际电机类型（必选）", padding=8
        )
        self.initial_axis = tk.StringVar(value="")
        self.initial_buttons = []
        for label in self.axis_choices:
            button = ttk.Radiobutton(
                self.initial_setup,
                text=label,
                value=label,
                variable=self.initial_axis,
                command=self.select_initial_axis,
            )
            button.pack(anchor="w")
            self.controls.append((button, "normal"))
            self.initial_buttons.append(button)
        self.setup_anchor = ttk.Frame(self.swd_tab)
        self.setup_anchor.pack(fill="x")
        erase_row = self.erase_row = ttk.Frame(self.pages["advanced"])
        erase_row.pack(anchor="w", pady=5)
        self.erase_buttons = {}
        for scope, label in (
            ("app", "擦除APP（保留Loader和参数）"),
            ("all", "整片擦除（含参数和身份）"),
        ):
            button = ttk.Button(
                erase_row, text=label, command=lambda value=scope: self.confirm_erase(value)
            )
            button.pack(side="left", padx=(0, 8))
            self.erase_buttons[scope] = button
        ttk.Label(erase_row, text="擦除后保持停机，需重新烧录。").pack(side="left")
        self.batch_query = tk.BooleanVar(value=False)
        batch_query = ttk.Checkbutton(
            self.query_tab, text="多轴查询", variable=self.batch_query, command=self.update_controls
        )
        batch_query.pack(anchor="w")
        self.controls.append((batch_query, "normal"))
        self.query_axes = ttk.LabelFrame(self.query_tab, text="批量查询节点", padding=7)
        self.nodes = {}
        for node, label in AXES.items():
            self.nodes[node] = tk.BooleanVar(value=node != 8)
            w = ttk.Checkbutton(
                self.query_axes, text=f"{label} 0x{node:02X}", variable=self.nodes[node]
            )
            w.pack(side="left", padx=4)
            self.controls.append((w, "normal"))
        options = self.query_options = ttk.Frame(self.query_tab)
        options.pack(fill="x", pady=(14, 5))
        self.mode = combo(options, "测试模式", tuple(MODES), "常规查询", 15)
        self.count = tk.StringVar(value="1000")
        self.rate = tk.StringVar(value="50")
        self.entries = []
        for label, var in (("每节点笔数", self.count), ("总笔/秒", self.rate)):
            ttk.Label(options, text=label).pack(side="left")
            entry = ttk.Entry(options, textvariable=var, width=8)
            entry.pack(side="left", padx=(3, 12))
            self.entries.append(entry)
        self.mode_hint = tk.StringVar()
        ttk.Label(self.query_tab, textvariable=self.mode_hint).pack(anchor="w", pady=(4, 0))
        self.identifier = tk.StringVar()
        self.payload = tk.StringVar()
        for row, (label, var) in enumerate(
            (("CAN ID（HEX）", self.identifier), ("原始数据（HEX）", self.payload))
        ):
            ttk.Label(self.raw_tab, text=label).grid(row=row, column=0, sticky="w")
            entry = ttk.Entry(self.raw_tab, textvariable=var, width=66)
            entry.grid(row=row, column=1, sticky="ew", pady=2)
            self.controls.append((entry, "normal"))
        self.raw_tab.columnconfigure(1, weight=1)
        ttk.Label(self.raw_tab, text="标准 ID 如 123，扩展 ID 如 00000123；数据如 01 02 FF。").grid(
            row=2, columnspan=2, sticky="w"
        )
        samples = ttk.Frame(self.raw_tab)
        samples.grid(row=3, columnspan=2, sticky="w", pady=4)
        ttk.Label(samples, text="状态查询示例").pack(side="left")
        w = ttk.Button(
            samples, text="填入当前电机", command=lambda: self.fill_query(self.current_node())
        )
        w.pack(side="left", padx=1)
        self.controls.append((w, "normal"))
        self.fill_query(6)
        bar = ttk.Frame(self.task_area)
        bar.pack(fill="x", pady=(12, 5))
        self.bar = ttk.Progressbar(bar, maximum=100)
        self.bar.pack(side="left", fill="x", expand=True)
        self.progress = tk.StringVar(value="0%")
        ttk.Label(bar, textvariable=self.progress, width=8, anchor="e").pack(side="left")
        buttons = ttk.Frame(self.task_area)
        buttons.pack(side="bottom", fill="x")
        self.start_button = ttk.Button(buttons, text="开始测试", command=self.start)
        self.start_button.pack(side="left")
        self.stop_button = ttk.Button(buttons, text="停止", command=self.stop)
        self.stop_button.pack(side="left", padx=5)
        self.open_button = ttk.Button(
            buttons, text="打开本次记录", command=lambda: os.startfile(self.report_dir)
        )
        self.open_button.pack(side="left")
        self.production_report_button = ttk.Button(
            buttons, text="打开单台报告", command=self.open_production_report, state="disabled"
        )
        self.production_report_button.pack(side="left", padx=6)
        self.log_toggle = ttk.Button(buttons, text="收起日志", command=self.toggle_log)
        self.log_toggle.pack(side="left", padx=6)
        self.log_open = True
        self.state = tk.StringVar(value="就绪")
        ttk.Label(buttons, textvariable=self.state).pack(side="right")
        self.text_record = ttk.Frame(self.record_area)
        self.text_record.pack(fill="both", expand=True)
        self.output = tk.Text(
            self.text_record,
            height=12,
            wrap="word",
            state="disabled",
            font=("Microsoft YaHei UI", 10),
        )
        record_scroll = ttk.Scrollbar(
            self.text_record, orient="vertical", command=self.output.yview
        )
        self.output.configure(yscrollcommand=record_scroll.set)
        record_scroll.pack(side="right", fill="y")
        self.output.pack(side="left", fill="both", expand=True)
        self.hardware_results = ttk.Frame(self.pages["hardware"])
        self.hardware_result_note = tk.StringVar()
        ttk.Label(self.hardware_results, text="本轮实测明细（异常行已标色）").pack(
            fill="x", pady=(2, 4)
        )
        grid = ttk.Frame(self.hardware_results)
        grid.pack(fill="both", expand=True)
        self.hardware_table = ttk.Treeview(
            grid, columns=("item", "measured", "limit", "result"), show="headings", height=7
        )
        for key, title, width in (
            ("item", "测试项目", 100),
            ("measured", "本轮实测数值", 500),
            ("limit", "判定标准", 280),
            ("result", "结果", 65),
        ):
            self.hardware_table.heading(key, text=title)
            self.hardware_table.column(key, width=width, minwidth=65, stretch=key == "measured")
        self.hardware_table.tag_configure("异常", foreground="#b42318")
        self.hardware_table.tag_configure("attention", foreground="#b42318", background="#fff0e8")
        self.hardware_table.tag_configure("正常", foreground="#16734b")
        self.hardware_table.tag_configure("未测", foreground="#8a6100")
        self.hardware_table.grid(row=0, column=0, sticky="nsew")
        vertical = ttk.Scrollbar(grid, orient="vertical", command=self.hardware_table.yview)
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal = ttk.Scrollbar(grid, orient="horizontal", command=self.hardware_table.xview)
        horizontal.grid(row=1, column=0, sticky="ew")
        self.hardware_table.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        grid.rowconfigure(0, weight=1)
        grid.columnconfigure(0, weight=1)
        buttons.pack_forget()
        buttons.pack(side="bottom", fill="x")
        bar.pack_forget()
        bar.pack(side="bottom", fill="x", pady=(8, 5))
        self.tabs.pack_forget()
        self.tabs.pack(fill="both", expand=True)
        self.mode.trace_add("write", lambda *_: self.update_controls())
        self.motor_axis.trace_add("write", self.select_current_motor)
        for var in (self.channel, self.nominal, self.data):
            var.trace_add("write", self.connection_changed)
        self.tabs.bind("<<NotebookTabChanged>>", lambda _: self.update_controls())
        self.maintenance_tabs.bind("<<NotebookTabChanged>>", lambda _: self.update_controls())
        self.calibration_tabs.bind("<<NotebookTabChanged>>", lambda _: self.update_controls())
        self.communication_tabs.bind("<<NotebookTabChanged>>", lambda _: self.update_controls())
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.update_controls()
        self.status_timer = self.root.after(100, self.status_heartbeat)
        self.root.bind("<Destroy>", self.cancel_status_timer, add="+")
        self.split_timer = self.root.after(150, self.place_record_split)

    def toggle_log(self) -> None:
        """折叠/展开底部记录，不删除已有日志或测试结果。"""
        if self.log_open:
            self.workspace.forget(self.record_area)
            self.log_toggle.configure(text="展开日志")
        else:
            self.workspace.add(self.record_area, weight=1)
            self.workspace.sashpos(0, round(self.workspace.winfo_height() * 0.75))
            self.log_toggle.configure(text="收起日志")
        self.log_open = not self.log_open

    def place_record_split(self) -> None:
        """首次布局分配约四成高度给记录区，后续位置交由用户拖动。"""
        self.split_timer = None
        self.root.update_idletasks()
        height = self.workspace.winfo_height()
        if height > 100:
            self.workspace.sashpos(0, round(height * 0.75))

    def cancel_status_timer(self, event=None) -> None:
        """关闭或销毁主窗口时撤销定时回调，避免访问已销毁控件。"""
        if event is not None and event.widget is not self.root:
            return
        if getattr(self, "split_timer", None) is not None:
            self.root.after_cancel(self.split_timer)
            self.split_timer = None
        if self.status_timer is not None:
            self.root.after_cancel(self.status_timer)
            self.status_timer = None

    def status_heartbeat(self) -> None:
        """仅已连接且空闲时刷新；失败不自动重连，不打断任何设备任务。"""
        self.status_timer = None
        if self.close_pending:
            return
        self.update_device_summary()
        session = self.can_session
        focus = self.root.focus_get()
        editing = isinstance(focus, (tk.Entry, ttk.Entry, tk.Text, ttk.Combobox))
        if (
            not editing
            and self.auto_status.get()
            and self.process is None
            and session is not None
            and session.connected
            and session.process.is_alive()
            and self.current_motor
            and time.monotonic() >= self.next_status_refresh
        ):
            settings = dict(
                profile="device_status",
                node=self.current_node(),
                expected_uid=self.current_motor["uid"],
                bus=dict(
                    channel_index=int(self.channel.get()) - 1,
                    nominal_bps=RATES[self.nominal.get()],
                    data_bps=RATES[self.data.get()],
                ),
            )
            try:
                self.process = session.submit(settings, None)
                self.events = self.process
                self.stop_signal = session.cancel
                self.active_profile = "device_status"
                self.auto_status_task = True
                self.done = False
                self.exit_polls = 0
                self.next_status_refresh = time.monotonic() + STATUS_REFRESH_S
                self.root.after(20, self.poll)
            except (RuntimeError, OSError, queue.Full) as exc:
                session.connected = False
                self.current_motor["status_error"] = str(exc)
        self.status_timer = self.root.after(100, self.status_heartbeat)

    def choose_hex(self) -> None:
        name = filedialog.askopenfilename(
            parent=self.root, title="选择本批统一烧录 HEX", filetypes=[("Intel HEX", "*.hex")]
        )
        if name:
            self.hex_path = Path(name)
            self.hex_name.set(self.hex_path.name)
            self.update_controls()

    def choose_upgrade_package(self):
        path = filedialog.askopenfilename(
            parent=self.root,
            title="选择含 image.bin 和 manifest.bin 的 package.json",
            filetypes=[("CAN 升级清单", "package.json")],
        )
        if path:
            self.upgrade_package = Path(path).parent
            self.upgrade_package_name.set(self.upgrade_package.name)

    def clear_upgrade_scan(self):
        self.upgrade_targets.clear()
        for node in self.upgrade_selected:
            self.upgrade_selected[node].set(False)
            self.upgrade_lights[node].configure(text="● 未扫描", foreground="#656565")
            for labels in (self.upgrade_details, self.upgrade_uids, self.upgrade_results):
                labels[node].configure(text="—")
        self.upgrade_info.set("默认使用顶部当前电机；多台操作请扫描后启用批量升级。")
        self.update_controls()

    def show_online_motor(self, motor):
        node = motor["node"]
        model = motor.get("model") or f"{FIRST_FIT_AXES[node][1]}（预设）"
        version = motor["app_version"] if motor["role"] == 1 else motor["loader_version"]
        self.upgrade_lights[node].configure(text="● 在线", foreground="#167044")
        self.upgrade_uids[node].configure(text=motor["uid"])
        self.upgrade_details[node].configure(
            text=f"{model} · v{version >> 16}.{version >> 8 & 255}.{version & 255} · {upgrade_motor_status(motor)}"
        )

    def identify_again(self) -> None:
        self.identity = None
        self.first_fit_selected = False
        self.known_motors.clear()
        self.select_current_motor()
        self.motor_label.set("识别当前连接的电机。")
        self.navigate("jlink")
        self.start(identify_only=True)

    def navigate(self, page: str) -> None:
        """只切换页面，不发出设备命令。"""
        if page in ("jlink", "upgrade", "communication", "advanced"):
            self.tabs.select(self.sections["maintenance"])
            self.maintenance_tabs.select(self.pages[page])
        elif page in ("calibration", "speed"):
            self.tabs.select(self.sections["calibration"])
            self.calibration_tabs.select(self.pages[page])
        else:
            self.tabs.select(self.pages[page])

    def current_page(self) -> str:
        section = next(
            key for key, page in self.sections.items() if str(page) == self.tabs.select()
        )
        if section == "maintenance":
            return next(
                key
                for key in ("jlink", "upgrade", "communication", "advanced")
                if str(self.pages[key]) == self.maintenance_tabs.select()
            )
        if section == "calibration":
            return (
                "speed"
                if str(self.pages["speed"]) == self.calibration_tabs.select()
                else "calibration"
            )
        return "hardware" if section == "production" else section

    def open_record_root(self) -> None:
        """打开本机历史记录目录，不扫描或改写原记录。"""
        root = Path.home() / "CAN-Link-Test"
        root.mkdir(parents=True, exist_ok=True)
        os.startfile(root)

    def open_selected_record(self) -> None:
        """只打开列表选择的已存在任务目录。"""
        selection = self.record_table.selection()
        if selection:
            path = self.record_paths.get(selection[0])
            if path and path.is_dir():
                os.startfile(path)

    def current_node(self) -> int:
        return self.axis_choices[self.motor_axis.get()]

    def select_first_fit(self, _event=None) -> None:
        if self.identity and self.identity["node"] is None:
            self.first_fit_selected = True
            self.initial_axis.set(self.motor_axis.get())
        self.update_controls()

    def select_initial_axis(self) -> None:
        self.motor_axis.set(self.initial_axis.get())
        self.select_first_fit()

    def select_current_motor(self, *_args) -> None:
        node = self.current_node()
        if node != self.previous_node:
            self.motor_fields["limit"].delete(0, "end")
            self.motor_fields["limit"].insert(0, str(AXIS_SPEED_LIMITS.get(node, 30)))
        self.current_motor = self.known_motors.get(node)
        plot_identity = (node, self.current_motor.get("uid") if self.current_motor else None)
        if plot_identity != self.plot_identity:
            self.speed_plot.clear()
            self.parameter_page.reset(plot_identity)
            self.plot_identity = plot_identity
        if self.identity and self.identity["node"] not in (None, node):
            self.identity = None
        if (
            self.identity
            and self.current_motor
            and self.identity["uid"] != self.current_motor["uid"]
        ):
            self.identity = None
        if self.current_motor:
            self.current_motor_label.set(
                f"UID {self.current_motor['uid']}\n来源：{self.current_motor['source']}"
            )
        else:
            self.current_motor_label.set("当前轴尚未识别；可先烧录或点击扫描识别。")
        old_ident, old_body = encode_frames(self.previous_node, 108, b"", 1)[0]
        if (
            self.identifier.get() == f"0x{old_ident:08X}"
            and self.payload.get() == old_body.hex(" ").upper()
        ):
            self.fill_query(node)
        self.previous_node = node
        self.update_device_summary()
        self.flow_summary.set(
            self.production_flow.summary(self.current_motor["uid"] if self.current_motor else None)
        )
        self.update_controls()

    def next_device(self) -> None:
        """先归档本次证据再清理设备缓存；不烧录、扫描或启动电机。"""
        if self.pending_start is not None:
            return
        if self.process is not None:
            if self.auto_status_task:
                self.pending_next_device = True
                self.stop_signal.set()
            return
        if self.production_flow.devices:
            directory = Path.home() / "CAN-Link-Test" / "production-sessions"
            path = directory / (datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".json")
            try:
                directory.mkdir(parents=True, exist_ok=True)
                path.write_text(
                    json.dumps(
                        dict(
                            tool_version="1.23.0",
                            created_at=datetime.now().isoformat(),
                            devices=self.production_flow.devices,
                            summaries={
                                uid: self.production_flow.summary(uid)
                                for uid in self.production_flow.devices
                            },
                        ),
                        ensure_ascii=False,
                        indent=2,
                    ),
                    encoding="utf-8",
                )
            except OSError as exc:
                messagebox.showerror("归档失败，当前设备已保留", str(exc), parent=self.root)
                return
            item = self.record_table.insert(
                "",
                0,
                values=(
                    datetime.now().strftime("%H:%M:%S"),
                    "生产会话归档",
                    "—",
                    "查看流程摘要",
                ),
            )
            self.record_paths[item] = directory
        self.production_flow = ProductionFlow()
        self.install_verified_uid = None
        self.identity = None
        self.first_fit_selected = False
        self.initial_axis.set("")
        self.known_motors.clear()
        self.clear_upgrade_scan()
        self.select_current_motor()
        self.production_report_path = None
        self.report_dir = None
        self.curve_path = None
        self.archive_serial.set("")
        self.test_serial.set("")
        self.firmware_version.set("板端版本：尚未查询")
        self.motor_label.set("下一台：请重新识别板卡。")
        self.hardware_failure_label.configure(foreground="#8a6100")
        self.hardware_table.delete(*self.hardware_table.get_children())
        self.hardware_result_note.set("")
        self.hardware_failure.set("下一台尚未开始检测")
        self.show_hardware_phases({})
        self.install_phase.set("① 识别板卡 → ② 确认电机类型 → ③ 烧录整包 → ④ 启动与回读")
        self.install_hint.set("下一台：连接板卡后重新识别；新板仍须确认实际电机类型。")
        self.show("上一台结果已清理；本批电机型号、连接和检测配置已保留。")
        self.state.set("下一台待识别")
        self.navigate("jlink")
        self.update_controls()

    def update_device_summary(self) -> None:
        motor = self.current_motor or {}

        def version(key):
            value = motor.get(key)
            return "—" if value is None else "未提供" if value == 0 else "v" + version_text(value)

        self.device_versions.set(
            f"APP：{version('app_version')}    Loader：{version('loader_version')}"
        )
        session = self.can_session
        connected = session is not None and session.connected and session.process.is_alive()
        self.connection_state.set("CAN：已连接" if connected else "CAN：未连接 / 已释放")
        stamp = motor.get("checked_at")
        stale = stamp is not None and time.time() - stamp > 5
        if motor.get("status_error"):
            state = "读取失败；请刷新状态"
        elif stale:
            state = (
                "数据已过期；历史故障：" + describe_faults(motor.get("faults", 0)) + "（非实时）"
            )
        elif motor.get("role") == 2:
            state = "Loader · 等待升级"
        elif motor.get("state") is not None:
            names = {3: "自检", 4: "标定", 5: "空闲", 6: "运行", 8: "故障"}
            state = names.get(motor["state"], f"未知状态{motor['state']}")
            state += f" · 模式{motor.get('mode', '—')} · {describe_faults(motor.get('faults', 0))}"
        else:
            state = "未读取"
        current_values = not stale and not motor.get("status_error") and motor.get("role") == 1

        def measured(key, unit):
            value = motor.get(key) if current_values else None
            return (
                (f"{value:.3f} {unit}" if key == "bus_current_a" else f"{value:.2f} {unit}")
                if value is not None
                else f"— {unit}"
            )

        self.device_measurements.set(
            "母线电压："
            + measured("bus_voltage_v", "V")
            + "    母线电流："
            + measured("bus_current_a", "A")
            + "\n电机温度："
            + measured("motor_temperature_c", "°C")
        )
        self.device_state.set("状态：" + state)
        stamp = motor.get("checked_at")
        self.device_checked.set(
            "最后读取 " + datetime.fromtimestamp(stamp).strftime("%H:%M:%S")
            if stamp
            else "点击刷新状态读取实际设备"
        )

    def set_current_motor(self, node: int, uid: str, source: str, *, role=None, **metadata) -> None:
        if node not in FIRST_FIT_AXES:
            return
        previous = self.known_motors.get(node, {})
        if previous.get("uid") != uid:
            previous = {}
        self.known_motors[node] = dict(
            previous, **metadata, node=node, uid=uid, source=source, role=role
        )
        self.motor_axis.set(
            next(label for label, value in self.axis_choices.items() if value == node)
        )

    def connection_changed(self, *_args) -> None:
        self.known_motors.clear()
        self.identity = None
        self.select_current_motor()
        self.clear_upgrade_scan()

    def selected_upgrade_motors(self) -> list[dict]:
        if self.batch_upgrade.get():
            return [m for n, m in self.upgrade_targets.items() if self.upgrade_selected[n].get()]
        return [self.current_motor] if self.current_motor else []

    def fill_query(self, node: int) -> None:
        ident, body = encode_frames(node, 108, b"", 1)[0]
        self.identifier.set(f"0x{ident:08X}")
        self.payload.set(body.hex(" ").upper())

    def update_controls(self) -> None:
        busy = self.process is not None
        mode = MODES[self.mode.get()]
        current = self.current_page()
        swd = current == "jlink"
        for control, state in self.controls:
            control.configure(state="disabled" if busy else state)
        for entry in self.entries:
            entry.configure(state="normal" if not busy and mode == "fixed" else "disabled")
        for index in range(len(self.tabs.tabs())):
            self.tabs.tab(
                index,
                state="disabled"
                if busy
                and self.active_profile not in ("parameters", "device_status")
                and self.tabs.tabs()[index] != self.tabs.select()
                else "normal",
            )
        for notebook in (self.maintenance_tabs, self.calibration_tabs):
            for tab in notebook.tabs():
                notebook.tab(
                    tab,
                    state="disabled"
                    if busy
                    and self.active_profile not in ("parameters", "device_status")
                    and tab != notebook.select()
                    else "normal",
                )
        for tab in self.communication_tabs.tabs():
            self.communication_tabs.tab(
                tab,
                state="disabled" if busy and tab != self.communication_tabs.select() else "normal",
            )
        action = {
            "communication": "发送一帧"
            if self.communication_tabs.select() == str(self.raw_tab)
            else "开始通信测试",
            "jlink": "烧录固件（自动识别）",
            "upgrade": "开始 CAN 升级",
            "hardware": "开始硬件功能检测",
            "calibration": "开始角度标定并保存",
            "speed": "使能速度控制",
            "parameters": "读取参数",
            "advanced": "高级维护",
            "records": "测试记录",
        }[current]
        self.tabs.configure(
            height=self.pages[current].winfo_reqheight() if current == "hardware" else 0
        )
        self.next_device_button.configure(
            state="normal" if not busy or self.auto_status_task else "disabled"
        )
        speed_active = busy and self.active_profile == "motor_speed"
        for key, entry in self.motor_fields.items():
            entry.configure(
                state="normal" if (speed_active and key == "speed") or not busy else "disabled"
            )
        self.speed_update.configure(state="normal" if speed_active else "disabled")
        self.clear_fault_button.configure(
            state="normal"
            if not busy and self.current_motor and self.current_motor.get("role") == 1
            else "disabled"
        )
        self.status_refresh.configure(
            state="normal" if not busy and self.current_motor else "disabled"
        )
        self.can_disconnect.configure(
            state="normal"
            if not busy and self.can_session and self.can_session.connected
            else "disabled"
        )
        self.curve_button.configure(state="normal" if self.curve_path and not busy else "disabled")
        self.parameter_page.update_controls(
            busy, bool(self.current_motor and self.current_motor.get("role") == 1)
        )
        self.speed_start.configure(state="disabled" if busy else "normal")
        self.speed_stop.configure(state="normal" if speed_active else "disabled")
        self.speed_open.configure(state="normal" if self.report_dir else "disabled")
        if current == "speed":
            self.stop_button.pack_forget()
            self.open_button.pack_forget()
        else:
            if not self.stop_button.winfo_manager():
                self.stop_button.pack(side="left", padx=5)
            if not self.open_button.winfo_manager():
                self.open_button.pack(side="left")
        if current in ("calibration", "speed", "parameters", "advanced", "records"):
            self.start_button.pack_forget()
        elif not self.start_button.winfo_manager():
            self.start_button.pack(side="left", before=self.stop_button)
        for node, check in self.upgrade_checks.items():
            check.configure(
                state="normal"
                if self.batch_upgrade.get() and node in self.upgrade_targets and not busy
                else "disabled"
            )
        if self.batch_query.get():
            self.query_axes.pack(fill="x", before=self.query_options)
        else:
            self.query_axes.pack_forget()
        selected_motors = self.selected_upgrade_motors()
        selected = len(selected_motors)
        bitrate_state = (
            "normal"
            if not busy and selected and all(m.get("role") == 1 for m in selected_motors)
            else "disabled"
        )
        self.bitrate_read.configure(state=bitrate_state)
        self.bitrate_save.configure(state=bitrate_state)
        if current == "upgrade" and selected:
            action = f"开始升级（{selected} 台）"
        blank = self.identity is not None and self.identity["node"] is None
        if blank:
            self.initial_setup.pack(fill="x", pady=5, before=self.setup_anchor)
        else:
            self.initial_setup.pack_forget()
        if blank and "resume_node" in self.identity:
            self.motor_selector.configure(state="disabled")
            for button in self.initial_buttons:
                button.configure(state="disabled")
        if swd and self.identity:
            action = (
                (
                    f"初始化并烧录 · {self.motor_axis.get()}"
                    if self.first_fit_selected
                    else "请先选择上方电机类型"
                )
                if blank
                else f"烧录固件 · 节点 {self.identity['node']}（保留参数）"
            )
        self.start_button.configure(
            text=action,
            state="disabled"
            if busy
            or (current == "upgrade" and not selected)
            or (swd and (not self.hex_path or (blank and not self.first_fit_selected)))
            else "normal",
        )
        self.rescan.configure(state="normal" if not busy else "disabled")
        ready = (
            bool(self.identity) and bool(self.hex_path) and (not blank or self.first_fit_selected)
        )
        self.install_flash.configure(state="normal" if ready and not busy else "disabled")
        self.go_detection.configure(
            state="normal"
            if self.install_verified_uid
            and self.current_motor
            and self.current_motor["uid"] == self.install_verified_uid
            and not busy
            else "disabled"
        )
        if not busy and self.identity:
            if blank:
                self.install_phase.set("① 已识别新板 → ② 确认电机类型 → ③ 烧录整包 → ④ 启动与回读")
                self.install_hint.set(
                    "已选择：" + self.initial_axis.get() + "；下一步点击“3. 烧录整包”。"
                    if self.first_fit_selected
                    else "请选择下方实际电机类型；新板无需先擦除。"
                )
            elif not self.install_verified_uid:
                self.install_phase.set("① 已识别已配板 → ② 沿用原参数 → ③ 烧录整包 → ④ 启动与回读")
                self.install_hint.set("已有参数和标定数据将保留；点击“3. 烧录整包”。")
        for scope, button in self.erase_buttons.items():
            allowed = (
                not busy and self.identity and (scope == "all" or self.identity["node"] is not None)
            )
            button.configure(state="normal" if allowed else "disabled")
        self.stop_button.configure(state="normal" if busy and not self.swd_busy else "disabled")
        self.stop_button.configure(
            text="本台完成后停止" if self.active_profile == "can_upgrade" else "停止"
        )
        self.open_button.configure(state="normal" if self.report_dir else "disabled")
        self.production_report_button.configure(
            state="normal" if self.production_report_path else "disabled"
        )
        self.mode_hint.set(
            {
                "fixed": "状态查询 · 首答 50 ms",
                "ladder": "5 → 50 → 200 → 500 → 5 笔/秒 · 首答 50 ms",
                "acceptance": "五轴各 70,000 笔 · 轮询和连续发送 · 首答 50 ms",
            }[mode]
            + "　｜　控制器正常发送 · 应用层重试 0 次"
        )

    def open_production_report(self) -> None:
        """只打开本次已生成的报告，不沿用前一台设备的路径。"""
        if self.production_report_path and self.production_report_path.is_file():
            os.startfile(self.production_report_path)

    def update_failure_text(self, *_args) -> None:
        """摘要区域限制高度，完整失败列表可滚动阅读。"""
        self.hardware_failure_label.configure(state="normal")
        self.hardware_failure_label.delete("1.0", "end")
        self.hardware_failure_label.insert("1.0", self.hardware_failure.get())
        self.hardware_failure_label.configure(state="disabled")

    def show_hardware_phases(self, phases: dict) -> None:
        """只展示工作线程已报告阶段；进度表示流程推进，不估算耗时。"""
        finished = 0
        requested = 0
        for index, (key, (name, var)) in enumerate(self.hardware_phases.items()):
            state = phases.get(key, "待执行")
            var.set(f"{index + 1}. {name} · {state}")
            if state != "本轮不测":
                requested += 1
                finished += state == "已完成"
        self.bar["value"] = 100 * finished / max(1, requested)
        self.progress.set(f"{finished}/{requested} 阶段")

    def show_hardware_result(self, report: dict) -> None:
        """展示本轮实测值和原始判定，缺失项保持可见。"""
        self.hardware_results.pack(fill="both", expand=True, pady=(8, 0))
        self.hardware_table.delete(*self.hardware_table.get_children())
        results = report_rows(report)
        for row in results:
            self.hardware_table.insert(
                "",
                "end",
                values=tuple(row[key] for key in ("item", "measured", "limit", "result")),
                tags=("attention" if row["attention"] else row["result"],),
            )
        notes = [summarize(report).splitlines()[0]]
        for key in ("note", "measurement_note", "scope_note", "report_error"):
            if report.get(key):
                notes.append(report[key])
        if report.get("motor_direction"):
            notes.append(report["motor_direction"]["detail"])
        failures = [
            f"{row['item']}：{row['result']}；实测 {row['measured']}；要求 {row['limit']}；{row['suggestion']}"
            for row in results
            if row["attention"]
        ]
        self.hardware_result_note.set("\n".join([*notes, *failures[:4]]))
        data = build_report(report, tool_version="1.23.0", created_at="")
        self.hardware_failure.set(
            data["scope_name"] + " · " + data["verdict"] + "\n" + failure_summary(report)
        )
        self.hardware_failure_label.configure(
            foreground="#16734b" if data["verdict"] == "本范围通过" else "#b42318"
        )
        if report.get("phase_states"):
            self.show_hardware_phases(report["phase_states"])

    def show(self, text: str) -> None:
        self.hardware_results.pack_forget()
        self.text_record.pack(fill="both", expand=True)
        self.output.configure(state="normal")
        self.output.delete("1.0", "end")
        self.output.insert("end", text)
        self.output.configure(state="disabled")

    def update_speed(self) -> None:
        if self.active_profile != "motor_speed" or self.commands is None:
            return
        try:
            self.commands.put_nowait(speed_value(self.motor_fields["speed"].get()))
        except (ValueError, queue.Full) as exc:
            messagebox.showerror("速度目标未提交", str(exc) or "目标队列已满", parent=self.root)

    def show_calibration_curves(self) -> None:
        if self.curve_path is None:
            return
        path = self.curve_path
        try:
            picture = tk.PhotoImage(master=self.root, file=str(path))
        except (OSError, tk.TclError) as exc:
            messagebox.showerror("曲线无法显示", str(exc), parent=self.root)
            return
        window = tk.Toplevel(self.root)
        window.title(f"标定曲线与参数 · {path.parent.name}")
        window.curve_image = picture
        width = min(picture.width(), self.root.winfo_screenwidth() - 100)
        height = min(picture.height(), self.root.winfo_screenheight() - 180)
        controls = ttk.Frame(window, padding=6)
        controls.grid(row=0, column=0, sticky="ew")
        ttk.Button(controls, text="打开PNG", command=lambda: os.startfile(path)).pack(side="left")
        csv_path = path.with_suffix(".csv")
        ttk.Button(controls, text="打开补偿CSV", command=lambda: os.startfile(csv_path)).pack(
            side="left"
        )
        for filename, label in (
            ("hall-raw.csv", "打开Hall原码"),
            ("calibration-values.csv", "打开标定参数"),
        ):
            data_path = path.parent / filename
            if data_path.is_file():
                ttk.Button(controls, text=label, command=lambda p=data_path: os.startfile(p)).pack(
                    side="left"
                )
        canvas = tk.Canvas(window, width=width, height=height, background="white")
        canvas.grid(row=1, column=0, sticky="nsew")
        vertical = ttk.Scrollbar(window, orient="vertical", command=canvas.yview)
        horizontal = ttk.Scrollbar(window, orient="horizontal", command=canvas.xview)
        vertical.grid(row=1, column=1, sticky="ns")
        horizontal.grid(row=2, column=0, sticky="ew")
        canvas.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        canvas.create_image(0, 0, anchor="nw", image=picture)
        canvas.configure(scrollregion=(0, 0, picture.width(), picture.height()))
        window.rowconfigure(1, weight=1)
        window.columnconfigure(0, weight=1)

    def confirm_erase(self, scope: str) -> None:
        if self.process is not None or self.identity is None:
            return
        labels = {
            "app": "仅APP区（96 KiB）；保留Loader、标定参数和生产身份。",
            "all": "全部物理Flash（256 KiB）；Loader、APP、标定参数和生产身份均会清空。",
        }
        if scope not in labels or (scope == "app" and self.identity["node"] is None):
            return
        if messagebox.askyesno(
            "确认Flash擦除",
            f"芯片UID：{self.identity['uid']}\n范围：{labels[scope]}\n\n"
            "执行前保存完整Flash备份；完成后不启动电机，必须重新烧录。\n确认擦除？",
            icon="warning",
            default="no",
            parent=self.root,
        ):
            self.start(erase_scope=scope, erase_confirmed=True)

    def operation_mode(
        self,
        *,
        scan=False,
        bitrate_action=None,
        clear_fault=False,
        erase_scope=None,
        identify_only=False,
        status_only=False,
        parameter_action=None,
    ) -> str:
        if parameter_action is not None:
            return "parameters"
        if status_only:
            return "device_status"
        if identify_only:
            return "jlink_identify"
        if erase_scope is not None:
            return "jlink_erase"
        if clear_fault:
            return "clear_fault"
        if scan:
            return "can_scan"
        if bitrate_action:
            return "can_bitrate"
        page = self.current_page()
        if page == "communication":
            return (
                "manual"
                if self.communication_tabs.select() == str(self.raw_tab)
                else MODES[self.mode.get()]
            )
        return {
            "jlink": "jlink_flash",
            "upgrade": "can_upgrade",
            "hardware": "hardware_check",
            "calibration": "motor_calibration",
            "speed": "motor_speed",
            "parameters": "parameters",
            "advanced": "none",
            "records": "none",
        }[page]

    def start(
        self,
        *,
        scan=False,
        bitrate_action=None,
        calibration_kind=1,
        clear_fault=False,
        erase_scope=None,
        erase_confirmed=False,
        identify_only=False,
        status_only=False,
        parameter_action=None,
    ) -> None:
        if self.process is not None:
            if self.auto_status_task:
                self.pending_start = dict(
                    scan=scan,
                    bitrate_action=bitrate_action,
                    calibration_kind=calibration_kind,
                    clear_fault=clear_fault,
                    erase_scope=erase_scope,
                    erase_confirmed=erase_confirmed,
                    identify_only=identify_only,
                    status_only=status_only,
                    parameter_action=parameter_action,
                )
                self.stop_signal.set()
            return
        mode = self.operation_mode(
            scan=scan,
            bitrate_action=bitrate_action,
            clear_fault=clear_fault,
            erase_scope=erase_scope,
            identify_only=identify_only,
            status_only=status_only,
            parameter_action=parameter_action,
        )
        if (
            mode not in ("parameters", "device_status")
            and self.can_session
            and self.can_session.connected
        ):
            self.pending_start = dict(
                scan=scan,
                bitrate_action=bitrate_action,
                calibration_kind=calibration_kind,
                clear_fault=clear_fault,
                erase_scope=erase_scope,
                erase_confirmed=erase_confirmed,
                identify_only=identify_only,
                status_only=status_only,
                parameter_action=parameter_action,
            )
            self.release_can()
            return
        if mode == "none":
            return
        swd = mode.startswith("jlink_")
        settings = {
            "profile": mode,
            "bus": {
                "channel_index": int(self.channel.get()) - 1,
                "nominal_bps": RATES[self.nominal.get()],
                "data_bps": RATES[self.data.get()],
            },
        }
        try:
            if swd:
                if mode == "jlink_erase" and (
                    not self.identity or not erase_confirmed or erase_scope not in ("app", "all")
                ):
                    raise ValueError("请先识别当前芯片并确认擦除范围")
                if mode in ("jlink_flash", "jlink_erase"):
                    self.clear_upgrade_scan()
                settings = {
                    "profile": mode,
                    "identity": self.identity,
                    "hex": str(self.hex_path or ""),
                    "restart_after_flash": True,
                    "force_reflash": True,
                    "erase_scope": erase_scope,
                    "erase_confirmed": erase_confirmed,
                    "first_fit_node": self.current_node()
                    if self.identity and self.identity["node"] is None and self.first_fit_selected
                    else None,
                    "bus": settings["bus"],
                }
            elif mode == "parameters":
                if not self.current_motor or self.current_motor.get("role") != 1:
                    raise ValueError("请先识别运行APP的当前电机")
                if parameter_action is None:
                    raise ValueError("请选择参数组的读取或保存操作")
                settings.update(
                    node=self.current_node(),
                    expected_uid=self.current_motor["uid"],
                    **self.parameter_page.request(*parameter_action),
                )
            elif mode == "device_status":
                if not self.current_motor:
                    raise ValueError("请先识别当前电机")
                settings.update(node=self.current_node(), expected_uid=self.current_motor["uid"])
            elif mode == "clear_fault":
                if not self.current_motor or self.current_motor.get("role") != 1:
                    raise ValueError("请先识别运行APP的当前电机")
                settings.update(node=self.current_node(), expected_uid=self.current_motor["uid"])
            elif mode in ("motor_calibration", "motor_speed"):
                if not self.current_motor:
                    raise ValueError("请先烧录或扫描识别当前电机")
                settings["node"] = self.current_node()
                settings["expected_uid"] = self.current_motor["uid"] if self.current_motor else None
                if mode == "motor_calibration":
                    self.curve_path = None
                    settings["calibration_kind"] = calibration_kind
                    settings["archive"] = calibration_archive(
                        ARCHIVE_TYPES[self.current_node()],
                        self.archive_serial.get(),
                        self.test_serial.get(),
                    )
                if mode == "motor_speed":
                    fields = {key: entry.get().strip() for key, entry in self.motor_fields.items()}
                    settings.update(motor_speed_fields(fields))
            elif mode == "hardware_check":
                if not self.current_motor:
                    raise ValueError("请先烧录或扫描识别当前电机")
                fields = {key: entry.get().strip() for key, entry in self.check_fields.items()}
                settings.update(
                    node=self.current_node(),
                    expected_uid=self.current_motor["uid"] if self.current_motor else None,
                    resistance_only=self.check_mode.get() == "静态＋相电阻",
                    **hardware_check_fields(fields, self.check_mode.get() == "静态和双向驱动"),
                )
            elif mode == "can_scan":
                self.upgrade_info.set("正在扫描当前通道…")
            elif mode in ("can_upgrade", "can_bitrate"):
                targets = [
                    {"node": n, "uid": m["uid"]}
                    for m in self.selected_upgrade_motors()
                    for n in (m["node"],)
                ]
                if not targets:
                    raise ValueError("请先扫描并勾选在线电机")
                settings["targets"] = targets
                if mode == "can_bitrate":
                    settings["set"] = (
                        (
                            RATES[self.target_nominal.get()] // 1000,
                            RATES[self.target_data.get()] // 1000,
                        )
                        if bitrate_action == "save"
                        else None
                    )
                else:
                    rounds = int(self.upgrade_rounds.get())
                    if rounds < 1:
                        raise ValueError("每台升级次数须为正整数")
                    settings.update(
                        rounds=rounds,
                        single_frame_writes=self.single_frame_writes.get(),
                        package=str(self.upgrade_package),
                    )
            elif mode == "manual":
                settings["frame"] = parse_raw(self.identifier.get(), self.payload.get())
            else:
                nodes = (
                    [3, 4, 5, 6, 7]
                    if mode == "acceptance"
                    else [n for n, selected in self.nodes.items() if selected.get()]
                    if self.batch_query.get()
                    else [self.current_node()]
                )
                if not nodes:
                    raise ValueError("请至少选择一个查询节点")
                if mode == "acceptance" and (
                    nodes != [3, 4, 5, 6, 7]
                    or self.nominal.get() != "1M"
                    or self.data.get() != "5M"
                ):
                    raise ValueError("五轴验收请选择节点 3 至 7，并使用 1M/5M")
                count = int(self.count.get()) if mode == "fixed" else PER_NODE
                rate = int(self.rate.get()) if mode == "fixed" else 50
                if not 1 <= count <= 1000000 or not 1 <= rate <= 10000:
                    raise ValueError("每节点笔数需为 1 至 1,000,000，总笔/秒需为 1 至 10,000")
                settings.update(nodes=nodes, count=count, rate=rate)
                settings["node_uids"] = {n: m["uid"] for n, m in self.known_motors.items()}
            directory = Path.home() / "CAN-Link-Test"
            if mode == "motor_calibration":
                directory = (
                    directory
                    / "calibration"
                    / model_directory_name(settings["archive"]["name"])
                    / ("测试-" + settings["archive"]["test_serial"])
                )
            directory = directory / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            directory.mkdir(parents=True, exist_ok=False)
            if mode in ("parameters", "device_status"):
                if self.can_session is not None and not self.can_session.process.is_alive():
                    self.can_session.dispose()
                    self.can_session = None
                if self.can_session is None:
                    resource = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
                    self.can_session = CanSession(controlcan_dll_path(resource))
                job = self.can_session.submit(settings, directory)
                self.events = job
                self.stop_signal = self.can_session.cancel
                self.commands = None
            else:
                context = mp.get_context("spawn")
                self.events = context.Queue()
                self.stop_signal = context.Event()
                self.commands = context.Queue(maxsize=8) if mode == "motor_speed" else None
                arguments = (settings, directory, self.events, self.stop_signal)
                if mode == "motor_speed":
                    arguments += (self.commands,)
                job = context.Process(
                    target=run_upgrade_test
                    if mode == "can_upgrade"
                    else run_jlink
                    if swd
                    else acquire,
                    args=arguments,
                )
                job.start()
        except (ValueError, OSError, RuntimeError) as exc:
            messagebox.showerror("无法启动", str(exc), parent=self.root)
            return
        self.production_flow.begin(
            self.current_motor["uid"] if self.current_motor else (self.identity or {}).get("uid"),
            mode,
            kind=calibration_kind,
            full=bool(settings.get("drive")),
        )
        self.flow_summary.set(
            self.production_flow.summary(
                self.current_motor["uid"]
                if self.current_motor
                else (self.identity or {}).get("uid")
            )
        )
        self.auto_status_task = False
        self.next_status_refresh = time.monotonic() + STATUS_REFRESH_S
        self.swd_busy = swd
        self.scanning = mode == "can_scan"
        if mode == "can_upgrade":
            for target in targets:
                self.upgrade_results[target["node"]].configure(text="排队中")
        self.process = job
        self.active_profile = mode
        if mode == "motor_speed":
            self.speed_plot.clear()
        self.production_report_path = None
        self.report_dir = directory
        self.done = False
        self.exit_polls = 0
        self.bar["value"] = 0
        self.progress.set("0%")
        self.state.set(
            "检查 CAN 升级包" if mode == "can_upgrade" else "连接 J-Link" if swd else "正在采集"
        )
        if swd:
            self.install_verified_uid = None
            self.install_hint.set("正在操作当前板卡；详细步骤记录在下方日志。")
        if mode == "hardware_check":
            self.show_hardware_phases({})
            self.hardware_failure.set("检测进行中；等待本轮结果。")
            self.hardware_failure_label.configure(foreground="#8a6100")
        self.show(f"记录目录　{directory}")
        self.update_controls()
        self.root.after(80, self.poll)

    def release_can(self, shutdown: bool = False) -> None:
        """异步释放常驻 CAN；释放应答前不启动其他设备任务。"""
        if self.process is not None or self.can_session is None:
            return
        if not self.can_session.process.is_alive():
            self.can_session.dispose()
            self.can_session = None
            self.update_controls()
            return
        mode = "can_shutdown" if shutdown else "can_release"
        self.process = self.can_session.submit(dict(profile=mode), None)
        self.events = self.process
        self.stop_signal = self.can_session.cancel
        self.active_profile = mode
        self.swd_busy = False
        self.done = self.release_ok = False
        self.exit_polls = 0
        self.state.set("正在释放 CAN 连接")
        self.update_controls()
        self.root.after(20, self.poll)

    def stop(self) -> None:
        if self.process and not self.swd_busy:
            self.stop_signal.set()
            self.stop_button.configure(state="disabled")
            self.state.set(
                "将在当前电机确认后停止" if self.active_profile == "can_upgrade" else "正在停止"
            )

    def close(self) -> None:
        self.cancel_status_timer()
        if self.process:
            self.close_pending = True
            if self.swd_busy:
                self.state.set("等待 SWD 操作完成后关闭")
            else:
                self.stop()
        elif self.can_session is not None:
            self.close_pending = True
            if self.can_session.process.is_alive():
                self.release_can(shutdown=True)
            else:
                self.can_session.dispose()
                self.can_session = None
                self.root.destroy()
        else:
            self.root.destroy()

    def poll(self) -> None:
        deadline = time.monotonic() + 0.008
        drained = False
        for _ in range(64):
            try:
                tag, msg = self.events.get_nowait()
            except queue.Empty:
                drained = True
                break
            if tag == "stage":
                self.state.set(msg.split(" s  ", 1)[-1])
                self.output.configure(state="normal")
                self.output.insert("end", "\n" + msg)
                self.output.see("end")
                self.output.configure(state="disabled")
            elif tag == "hardware_phases":
                self.show_hardware_phases(msg)
            elif tag == "install_phase":
                self.install_phase.set(
                    {
                        "identify": "① 正在识别板卡",
                        "select": "① 已识别新板 → ② 等待确认电机类型",
                        "flash": "① 已识别 → ② 已确认 → ③ 正在备份、烧录与校验",
                        "verify": "③ 镜像已核验 → ④ 正在启动与 CAN 回读",
                    }[msg]
                )
            elif tag == "parameter_progress":
                done, total = msg
                self.bar["value"] = 100 * done / total
                self.progress.set(f"{done}/{total}")
                self.state.set(f"正在读取参数 {done}/{total}")
            elif tag == "axis_status":
                node, status = msg
                self.upgrade_results[node].configure(text=status)
            elif tag == "motor_feedback":
                self.motor_feedback.set(
                    f"电机轴：目标{msg['target_rad_s']:+.3f} rad/s，实际{msg['actual_rad_s']:+.3f} rad/s"
                )
                self.speed_plot.append(msg)
                if self.current_motor:
                    self.current_motor.update(
                        bus_voltage_v=msg.get("bus_voltage_v"),
                        bus_current_a=msg.get("bus_current_a"),
                        motor_temperature_c=msg.get("motor_temperature_c"),
                        state=msg["state"],
                        mode=msg["mode"],
                        faults=msg["faults"],
                        checked_at=time.time(),
                        status_error=None,
                    )
                    self.update_device_summary()
            elif tag == "upgrade_progress":
                number, planned, percent = msg
                self.bar["value"] = 100 * ((number - 1) + percent / 100) / planned
                self.progress.set(f"{percent}%")
                self.state.set(f"第 {number}/{planned} 次 · 下载 {percent}%")
            elif tag == "progress":
                pct = 100 * msg["attempted"] / msg["planned"]
                self.bar["value"] = pct
                self.progress.set(f"{pct:.1f}%")
                self.state.set(
                    f"{msg['stage']} · {msg['attempted']:,}/{msg['planned']:,} · 失败 {msg['failures']}"
                )
            else:
                self.done = True
                self.display_report(msg)
            if time.monotonic() >= deadline:
                break
        finished = not self.process.is_alive()
        if self.active_profile == "can_shutdown" and self.can_session.process.is_alive():
            finished = False
        if finished and not self.done:
            self.exit_polls += 1
        if not finished or not drained or (not self.done and self.exit_polls < 5):
            self.root.after(20 if isinstance(self.process, SessionTask) else 80, self.poll)
            return
        self.process.join()
        self.process.close()
        self.events.close()
        if self.commands is not None:
            self.commands.close()
            self.commands = None
        was_automatic = self.auto_status_task
        self.auto_status_task = False
        self.process = None
        self.active_profile = None
        if not was_automatic:
            self.next_status_refresh = time.monotonic() + STATUS_REFRESH_S
        if not self.done:
            self.identity = None
            self.known_motors.clear()
            self.select_current_motor()
            self.motor_label.set("请重新识别电机。")
            if self.current_page() == "upgrade":
                if self.scanning:
                    self.clear_upgrade_scan()
                else:
                    for node in self.upgrade_targets:
                        if self.upgrade_selected[node].get():
                            self.upgrade_lights[node].configure(
                                text="● 待复核", foreground="#995900"
                            )
                            self.upgrade_results[node].configure(text="任务中断")
            self.state.set("工作进程异常退出")
            self.show(f"请检查记录目录 {self.report_dir}")
        if self.close_pending:
            self.close()
        else:
            if not was_automatic or not self.done:
                self.update_controls()
            if self.pending_next_device:
                self.pending_next_device = False
                self.next_device()
                return
            if self.pending_start is not None:
                pending, self.pending_start = self.pending_start, None
                if was_automatic or self.release_ok:
                    self.start(**pending)

    def display_report(self, r: dict) -> None:
        self.production_flow.record(r, str(self.report_dir) if self.report_dir else "")
        self.flow_summary.set(
            self.production_flow.summary(
                self.current_motor["uid"]
                if self.current_motor
                else r.get("uid") or r.get("identity", {}).get("uid")
            )
        )
        if (
            not self.auto_status_task
            and self.report_dir
            and r.get("profile") not in ("can_release", "can_shutdown")
        ):
            result = (
                "通过"
                if r.get("passed", r.get("success", False))
                else r.get("fatal", r.get("note", "查看详情"))
            )
            item = self.record_table.insert(
                "",
                0,
                values=(
                    datetime.now().strftime("%H:%M:%S"),
                    r.get("profile", "任务"),
                    r.get("node", (r.get("identity") or {}).get("node", "—")),
                    result,
                ),
            )
            self.record_paths[item] = self.report_dir
        if "submitted_at" in r:
            r["display_elapsed_s"] = time.monotonic() - r["submitted_at"]
        if r["profile"] in ("can_release", "can_shutdown"):
            self.release_ok = bool(r.get("passed"))
            self.state.set("CAN 已断开" if self.release_ok else r.get("fatal", "CAN 释放失败"))
            return
        if r["profile"] == "parameters":
            self.parameter_page.display(r)
            if r.get("passed") and 1 in r.get("rows", {}):
                self.motor_fields["limit"].configure(state="normal")
                self.motor_fields["limit"].delete(0, "end")
                self.motor_fields["limit"].insert(0, value_text(1, r["rows"][1]["raw"]))
            status = "参数读取完成" if r.get("passed") else "参数操作失败"
            if r.get("persisted"):
                status = "参数已保存，Flash回读一致"
            status += f" · {r.get('display_elapsed_s', r.get('total_elapsed_s', r.get('elapsed_s', 0))):.2f} 秒"
            if r.get("passed"):
                status += " · 复用 CAN" if r.get("connection_reused") else " · CAN 已连接"
            self.state.set(status)
            self.bar["value"] = 100 if r.get("passed") else 0
            self.progress.set(f"{self.bar['value']:.0f}%")
            self.show(
                status
                + "\n"
                + self.parameter_page.message.get()
                + ("\n已确认的RAM修改：" + str(list(r["applied"])) if r.get("applied") else "")
                + (
                    "\n保存已发送但未确认，请重新读取核查。"
                    if r.get("save_sent") and not r.get("persisted")
                    else ""
                )
                + f"\n记录目录　{self.report_dir}"
            )
            return
        if r["profile"] == "device_status":
            automatic = self.auto_status_task
            if "device" in r:
                device = dict(r["device"])
                node, uid, role = (device.pop(key) for key in ("node", "uid", "role"))
                if automatic:
                    if self.current_motor and (self.current_node(), self.current_motor["uid"]) == (
                        node,
                        uid,
                    ):
                        self.current_motor.update(device, role=role, status_error=None)
                        self.update_device_summary()
                else:
                    self.set_current_motor(
                        node, uid, "CAN状态读取", role=role, status_error=None, **device
                    )
                if not automatic:
                    self.state.set("设备版本和状态已更新")
            else:
                if "UID" in r.get("fatal", ""):
                    self.known_motors.pop(self.current_node(), None)
                    self.identity = None
                    self.select_current_motor()
                elif self.current_motor:
                    self.current_motor.update(status_error=r.get("fatal", "读取失败"))
                self.update_device_summary()
                if not automatic:
                    self.state.set("设备状态读取失败")
            if automatic:
                return
            self.show(r.get("fatal", self.device_versions.get() + "\n" + self.device_state.get()))
            return
        if r["profile"] == "jlink_flash":
            verified = (
                r.get("success")
                and r.get("app_started")
                and r.get("can_test", {}).get("version_verified")
            )
            self.install_verified_uid = r.get("identity", {}).get("uid") if verified else None
            self.go_detection.configure(state="normal" if verified else "disabled")
            if verified:
                self.install_phase.set("① 已识别 → ② 已确认 → ③ 烧录通过 → ④ 启动与版本回读通过")
                self.install_hint.set("本台烧录完成。下一步可进入生产检测；不会自动启动检测。")
            elif not r.get("success"):
                self.install_phase.set("烧录流程未完成")
                self.install_hint.set(r.get("fatal", "请查看本次记录并重新识别板卡。"))
        if r["profile"] == "jlink_erase":
            self.identity = None
            self.first_fit_selected = False
            self.initial_axis.set("")
            self.known_motors.clear()
            self.clear_upgrade_scan()
            self.select_current_motor()
            if r.get("success"):
                self.identity = {key: r["identity"][key] for key in ("uid", "serial", "node")}
                if r["erase_scope"] == "all":
                    self.identity["node"] = None
                    message = "整片Flash擦除并回读通过；请选择实际电机类型后重新初装。"
                else:
                    message = "APP擦除并回读通过；Loader、参数和身份保留，可直接重新烧录。"
                    self.set_current_motor(self.identity["node"], self.identity["uid"], "APP已擦除")
                self.firmware_version.set("APP已擦除；等待重新烧录和版本查询")
                self.motor_label.set(message)
                self.state.set("Flash擦除完成 · CPU保持暂停")
            else:
                message = r.get("fatal", "擦除结果未知")
                self.state.set("Flash擦除未完成")
                self.firmware_version.set("本次擦除未确认；需重新识别")
                self.motor_label.set("保留备份，排除故障后重新识别，勿直接启动。")
            lines = [*r.get("steps", []), message]
            if "erase_start" in r:
                lines.append(f"范围：0x{r['erase_start']:08X} 起，{r['erase_bytes']} 字节")
            lines.append(f"记录和备份目录　{self.report_dir}")
            self.show("\n".join(lines))
            self.bar["value"] = 100 if r.get("success") else 0
            self.progress.set(f"{self.bar['value']:.0f}%")
            return
        if r["profile"] == "clear_fault":
            status = "故障清除已确认" if r.get("passed") else "故障清除未通过"
            if r.get("cleared") and r.get("fatal"):
                status = "已回读无故障，清除请求未获确认"
            lines = [status]
            if "node" in r:
                lines.append(f"节点 {r['node']}　UID {r.get('uid', '未确认')}")
            for key, name in (("before", "清除前"), ("after", "清除后")):
                if key in r:
                    snap = r[key]
                    lines.append(
                        f"{name}：{describe_faults(snap['faults'])}（0x{snap['faults']:08X}）"
                        f"　state={snap['state']} mode={snap['mode']}"
                    )
            if r.get("result") is not None:
                lines.append(f"Type107 result={r['result']} detail={r['detail']}")
            if r.get("fatal"):
                lines.append(r["fatal"])
            if r.get("communication_timeouts"):
                lines.append(f"500 ms超时记录：{r['communication_timeouts']}")
            lines.append("本操作不启动电机；原始报文与清除前后状态已保存。")
            motor = self.known_motors.get(r.get("node"))
            if r.get("identity_changed"):
                self.known_motors.pop(r.get("node"), None)
                self.identity = None
                self.select_current_motor()
            if motor and motor["uid"] == r.get("uid") and "after" in r:
                motor.update(
                    state=r["after"]["state"],
                    faults=r["after"]["faults"],
                    mode=r["after"]["mode"],
                    checked_at=time.time(),
                    status_error=None,
                )
                self.select_current_motor()
                if motor["node"] in self.upgrade_targets:
                    self.upgrade_targets[motor["node"]].update(motor)
                    self.show_online_motor(motor)
            self.state.set(status)
            self.show("\n".join(lines))
            self.bar["value"] = 100 if r.get("passed") else 0
            self.progress.set(f"{self.bar['value']:.0f}%")
            return
        if r["profile"] in ("motor_calibration", "motor_speed"):
            if self.current_motor and self.current_motor["uid"] == r.get("uid"):
                self.current_motor.update(
                    {key: r[key] for key in ("app_version", "loader_version", "role") if key in r}
                )
                if r.get("final_status"):
                    self.current_motor.update(
                        r["final_status"], checked_at=time.time(), status_error=None
                    )
                self.update_device_summary()
            title = "角度标定" if r["profile"] == "motor_calibration" else "速度控制"
            status = (
                "失败"
                if r.get("fatal")
                else "完成"
                if r.get("passed")
                else "已取消"
                if r.get("cancelled")
                else "失败"
            )
            lines = [f"节点{r['node']}　{title}{status}" if "node" in r else title + status]
            if "uid" in r:
                lines.append(f"UID {r['uid']}")
            if r.get("archive"):
                archive = r["archive"]
                lines.append(f"样机 {archive['name']}　测试序列号 {archive['test_serial']}")
            if "ticket" in r:
                lines.append(f"票据{r['ticket']}　阶段{r['phase']}　保存状态{r['effects']}")
            if "saved_revision" in r:
                lines.append(f"Flash保存修订 {r['saved_revision']['saved_revision']}")
            if r.get("curves"):
                curves = r["curves"]
                lines.append("补偿曲线已生成：angle-calibration.png / angle-calibration.csv")
                lines.append("原始标定BIN、电机配置JSON及文件校验清单已保存。")
                lines.append(f"补偿峰峰值（°）：{curves['peak_to_peak_deg']}")
                values = curves["calibration_values"]
                lines.append(
                    f"电角零位Q15：{values['electrical_zero_q15']}；"
                    f"编码器坐标：{values['electrical_zero_encoder_deg']:.5f}°"
                )
                if "hall_alignment_motor_deg" in values:
                    lines.append(
                        f"Hall对齐角（电机坐标）：{values['hall_alignment_motor_deg']:.5f}°"
                    )
                    lines.append(
                        f"Hall两路标定极值（cnt）：{values['hall_calibration_extrema_counts']}"
                    )
                if not curves["table_updated"]:
                    lines.append("本次只更新电角零位；曲线为当前已保存LUT，并非本次重新扫表。")
                self.curve_path = self.report_dir / curves["png"]
                if not self.close_pending:
                    self.show_calibration_curves()
            if r.get("curve_error"):
                self.curve_path = None
                lines.append("标定已保存，但曲线未生成：" + r["curve_error"])
            if r.get("raw_capture"):
                capture = r["raw_capture"]
                lines.append(f"Hall原码已接收{capture['samples']}点。")
                if capture.get("error"):
                    lines.append("原码采集异常：" + capture["error"])
            if "samples" in r:
                lines.append(
                    f"反馈{r['samples']}次，目标更新{r['updates']}次；速度曲线见speed.csv"
                    if r["samples"]
                    else "尚未开始速度反馈采集"
                )
            if r["profile"] == "motor_speed" and r.get("enable_attempted") is False:
                lines.append("未发送使能命令，本次未启动电机")
            else:
                lines.append("已确认关相" if r.get("stop_confirmed") else "未取得关相确认")
            if r.get("fatal"):
                lines.append(r["fatal"])
            if r.get("communication_timeouts"):
                lines.append(f"{r['reply_timeout_ms']} ms超时记录：{r['communication_timeouts']}")
            reliability = (
                f" · {r['reply_timeout_ms']}ms应答异常" if r.get("communication_timeouts") else ""
            )
            self.state.set(title + status + reliability)
            if r.get("curve_error"):
                self.state.set(self.state.get() + " · 曲线未生成")
            self.show("\n".join(lines))
            self.bar["value"] = 100 if r.get("passed") else 0
            self.progress.set(f"{self.bar['value']:.0f}%")
            return
        if r["profile"] == "hardware_check":
            if self.report_dir and r.get("production_report") == "production-report.html":
                self.production_report_path = self.report_dir / "production-report.html"
            if "fatal" in r:
                self.hardware_failure.set("检测未完成\n" + r["fatal"])
                self.hardware_failure_label.configure(foreground="#b42318")
                self.state.set("通信异常")
                self.show(r["fatal"])
                return
            title = "硬件功能检测 · 通信" + r["communication"]
            if r.get("scope") == "static_resistance":
                title = (
                    "静态＋相电阻："
                    + ("通过" if r.get("subset_passed") else "未通过")
                    + " · 动态未执行"
                )
            self.state.set(title)
            direction = r.get("motor_direction", {})
            if direction.get("result") in ("reversed", "mismatch"):
                self.state.set(self.state.get() + " · " + direction["reason"])
            if r.get("communication_reliability_passed") is False:
                self.state.set(self.state.get() + " · 50ms首答异常")
            self.show_hardware_result(r)
            if not r.get("phase_states"):
                self.progress.set("结果已返回")
            return
        if r["profile"] == "can_bitrate":
            lines = []
            for item in r.get("results", []):
                ram, flash = item["ram"], item["flash"]
                lines.append(
                    f"节点 {item['node']}　RAM {ram >> 16}/{ram & 65535}　Flash {flash >> 16}/{flash & 65535} kbit/s"
                )
            status = r.get("fatal") or (
                "波特率操作已停止" if r.get("cancelled") else "波特率回读完成"
            )
            if r.get("set") and r.get("results"):
                lines.append("已保存的节点在复位后采用新速率。")
            self.state.set(status)
            self.show("\n".join([status, *lines]))
            self.bar["value"] = 100 if r.get("pass") else 0
            self.progress.set(f"{self.bar['value']:.0f}%")
            return
        if r["profile"] == "can_scan":
            motors = [] if r.get("cancelled") or r.get("fatal") else r["motors"]
            previous = {
                n: m["uid"]
                for n, m in self.upgrade_targets.items()
                if self.upgrade_selected[n].get()
            }
            self.clear_upgrade_scan()
            self.upgrade_targets = {m["node"]: m for m in motors}
            self.known_motors = {
                m["node"]: dict(m, source="CAN扫描", checked_at=time.time()) for m in motors
            }
            if not r.get("cancelled") and not r.get("fatal"):
                for n in self.upgrade_lights:
                    self.upgrade_lights[n].configure(text="● 未识别", foreground="#656565")
            for motor in motors:
                n = motor["node"]
                self.show_online_motor(motor)
                self.upgrade_selected[n].set(len(motors) == 1 or previous.get(n) == motor["uid"])
            if len(motors) == 1:
                self.motor_axis.set(
                    next(label for label, n in self.axis_choices.items() if n == motors[0]["node"])
                )
            else:
                self.select_current_motor()
            status = "扫描已停止" if r.get("cancelled") else f"发现 {len(motors)} 台在线电机"
            if r.get("fatal"):
                status = "扫描失败"
            self.state.set(status)
            self.upgrade_info.set(
                f"{datetime.now():%H:%M:%S} 扫描 · {status}。默认当前电机，批量模式可勾选多台。"
            )
            lines = [
                f"{AXES[n]} · 节点 {n}　{upgrade_motor_status(motor)}　UID {motor['uid']}"
                for n, motor in self.upgrade_targets.items()
            ]
            self.show("\n".join([r.get("fatal") or status, *lines, *r.get("issues", [])]))
            self.bar["value"] = 0 if r.get("fatal") or r.get("cancelled") else 100
            self.progress.set(f"{self.bar['value']:.0f}%")
            return
        if r["profile"] == "can_upgrade":
            for key, result in r.get("motors", {}).items():
                n = int(key)
                motor = self.upgrade_targets.get(n) or self.known_motors.get(n)
                if motor is None or motor["uid"] != result["uid"]:
                    continue
                progress = f"{result['passed']}/{r['rounds_per_motor']} 次通过"
                if result.get("identity_changed_or_missing"):
                    self.upgrade_targets.pop(n, None)
                    self.known_motors.pop(n, None)
                    self.upgrade_selected[n].set(False)
                    self.upgrade_lights[n].configure(text="● 待识别", foreground="#995900")
                    self.upgrade_details[n].configure(text="地址或 UID 需重新扫描")
                    self.upgrade_uids[n].configure(text="—")
                elif result["failed"]:
                    self.upgrade_lights[n].configure(text="● 待复核", foreground="#995900")
                elif result["passed"]:
                    motor.update(role=1, boot_state=1, app_version=r["version"], state=None)
                    self.known_motors[n] = dict(motor, source="CAN升级")
                    self.show_online_motor(motor)
                if result["failed"]:
                    progress += " · 失败"
                elif result["passed"] == 0:
                    progress = "未执行"
                self.upgrade_results[n].configure(text=progress)
            self.state.set(f"CAN 升级 · {r['status']}")
            self.bar["value"] = 100 * r["passed"] / max(1, r["planned"])
            self.progress.set(f"{self.bar['value']:.0f}%")
            self.output.configure(state="normal")
            self.output.insert(
                "end",
                f"\n计划 {r['planned']} 次，成功 {r['passed']}，失败 {r['failed']}，总耗时 {r['elapsed_s']:.2f} 秒。",
            )
            if r.get("fatal"):
                self.output.insert("end", f"\n{r['fatal']}")
            self.output.insert("end", f"\n逐轮记录　{self.report_dir}")
            self.output.see("end")
            self.output.configure(state="disabled")
            self.select_current_motor()
            return
        if r["profile"].startswith("jlink_"):
            self.identity = None
            self.first_fit_selected = False
            self.known_motors.clear()
            self.select_current_motor()
            versions = r.get("can_test", {}).get("device_versions")
            self.firmware_version.set(
                f"板端版本：APP v{version_text(versions['app'])} / "
                f"Loader v{version_text(versions['loader'])}"
                + ("（已核对）" if r["can_test"].get("version_verified") else "（与烧录包不一致）")
                if versions
                else "板端版本：本次未查询到"
            )
            if r.get("protected_regions_verified") and r.get("identity", {}).get("node"):
                self.identity = {key: r["identity"][key] for key in ("uid", "serial", "node")}
            if r.get("success"):
                identity = r["identity"]
                self.identity = (
                    {key: identity[key] for key in ("uid", "serial", "node")}
                    if identity["node"] is not None
                    else identity
                )
                name = (
                    "待初装板"
                    if identity["node"] is None
                    else f"节点 {identity['node']}（{AXES[identity['node']]}）"
                )
                if r["profile"] == "jlink_identify" or r.get("needs_initial_setup"):
                    self.identity = identity
                    detail = f"{name}　UID {identity['uid']}　J-Link {identity['serial']}"
                    if identity["node"] is None:
                        self.initial_axis.set("")
                        detail += "　请在本页选择实际电机类型，再点击初始化并烧录。"
                        if "resume_node" in identity:
                            self.motor_axis.set(
                                next(
                                    label
                                    for label, n in self.axis_choices.items()
                                    if n == identity["resume_node"]
                                )
                            )
                            self.first_fit_selected = True
                            self.initial_axis.set(self.motor_axis.get())
                            detail = (
                                f"节点 {identity['resume_node']} 初装待恢复　UID {identity['uid']}"
                            )
                    self.motor_label.set(detail)
                    self.state.set("已识别")
                else:
                    detail = (
                        "目标已装有相同镜像，已核对全片备份。"
                        if r.get("already_installed")
                        else "Loader、APP 和参数保护区回读通过。"
                    )
                    if r.get("first_fit_node"):
                        detail = f"{name} 初装参数回读通过，编码器待本机标定。"
                    if r.get("app_started"):
                        self.state.set(f"{name} · 通信通过")
                        self.motor_label.set(
                            f"{name} 已启动，可直接再次烧录；每次都会重新核对UID。"
                        )
                    else:
                        self.state.set(f"{name} · 已核验")
                        self.motor_label.set(
                            "芯片已暂停，请重新上电后测试通信。"
                            if r.get("cpu_halted")
                            else "已跳过重启。"
                        )
                if identity["node"] is not None:
                    metadata = dict(r.get("device_status", {}))
                    for key in ("node", "uid", "role"):
                        metadata.pop(key, None)
                    if versions:
                        metadata.update(
                            app_version=versions["app"], loader_version=versions["loader"]
                        )
                    if r.get("device_status_error"):
                        metadata["status_error"] = r["device_status_error"]
                    self.set_current_motor(
                        identity["node"],
                        identity["uid"],
                        "J-Link识别" if r["profile"] == "jlink_identify" else "J-Link烧录",
                        role=r.get("device_status", {}).get(
                            "role", 1 if r.get("app_started") else None
                        ),
                        **metadata,
                    )
                self.bar["value"] = 100
                self.progress.set("100%")
            else:
                self.state.set("SWD 操作失败")
                self.motor_label.set("排除故障后重新识别当前电机。")
                detail = r.get("fatal", "未取得烧录结果")
                if r.get("app_started"):
                    self.state.set("APP 已启动 · CAN 查询失败")
                    self.motor_label.set("固件已核验；请检查 USB-CAN 连接，可再次烧录或重新识别。")
                elif r.get("protected_regions_verified"):
                    detail += "\n固件已核验，请重新识别后重试启动。"
                elif r.get("write_started"):
                    detail += "\n已开始写入，须重新校验镜像后再上电。"
            self.show(
                "\n".join(
                    [
                        *r.get("steps", []),
                        detail,
                        self.firmware_version.get(),
                        f"用时 {r['elapsed_s']:.2f} 秒　记录目录　{self.report_dir}",
                    ]
                )
            )
            self.output.see("end")
            return
        if r.get("fatal") and "nodes" not in r:
            self.state.set("失败")
            self.show(r["fatal"] + f"\n记录目录　{self.report_dir}")
            return
        if r["profile"] == "manual":
            self.state.set("已停止" if r["cancelled"] else "采集完成")
            self.show(
                f"SDK 已受理 1 帧，收到 {r['received']} 帧\n"
                + "\n".join(f"{v[1]}  {v[-1]}" for v in r["sample"])
                + f"\n记录目录　{self.report_dir}"
            )
            if not r["cancelled"]:
                self.progress.set("100%")
                self.bar["value"] = 100
            return
        self.state.set("已停止" if r["cancelled"] else ("通过" if r["pass"] else "未通过"))
        passed = sum(v["passed"] for v in r["nodes"].values())
        lines = [
            f"首次有效应答 {passed:,}/{r['attempted']:,}　失败 {r['failed']}　未关联 {sum(r['anomalies'].values())}"
        ]
        for n, row in r["nodes"].items():
            lines.append(
                f"0x{int(n):02X}　{AXES[int(n)]}　{row['passed']:,}/{row['attempted']:,}　"
                f"仅异常帧 {row['errors'].get('flagged_only', 0)}　超时 {row['errors'].get('timeout', 0)}　P99 {row['latency_ms'].get('P99', '—')} ms"
            )
        if r.get("fatal"):
            lines.insert(0, r["fatal"])
        lines.append(
            f"适配器错误标志 {sum(r['rx_flags'].values())} 帧　未完成请求 {r['unresolved']} 笔"
        )
        lines.extend(
            f"{s['target_hz']} 笔/秒阶段　实测 {s['actual_hz']} 笔/秒　{s['passed']}/{s['attempted']}"
            for s in r["stages"]
        )
        if r["profile"] == "acceptance" and r["complete"]:
            lines.append(
                f"各节点失败率同步 95% 上限的最大值　{max(v['upper_pct_simultaneous95'] for v in r['nodes'].values()):.6f}%"
            )
        self.show("\n".join(lines) + f"\n记录目录　{self.report_dir}")


def main() -> None:
    """打开界面，用户点击开始或发送之后才访问适配器。"""
    mp.freeze_support()
    root = tk.Tk()
    CanTestWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
