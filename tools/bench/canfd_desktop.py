"""Windows USB-CAN 桌面工具。界面只负责参数与显示，采集进程独占 SDK。"""

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
from can_motor_operations import TraceBus, run_angle_calibration, run_speed_control, speed_value
from can_upgrade_stress import read_node_uids, run_upgrade_test, scan_upgrade_motors
from five_axis_can_stability import (
    PER_NODE,
    ControlCanFd,
    batch_versions,
    controlcan_dll_path,
    encode_frames,
    run_queries,
)
from five_axis_jlink import FIRST_FIT_AXES, run_jlink
from hardware_check import Drive, Link, empty_report, run_check, summarize
from motor_can_bitrate import configure as configure_bitrate
from ports.controlcanfd import _RXFD, _TXFD

TITLE = "CAN FD 通信测试 v1.10.0"
AXES = {3: "左轮", 4: "右轮", 5: "ROLL", 6: "PITCH", 7: "YAW", 8: "Lift"}
MODES = {"常规查询": "fixed", "阶梯压测": "ladder", "五轴稳定性验收": "acceptance"}
RATES = {"500K": 500000, "1M": 1000000, "2M": 2000000, "3M": 3000000, "4M": 4000000, "5M": 5000000}
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
    travel = _number_field(fields["travel"].strip(), "两侧可用输出行程")
    if travel <= 0:
        raise ValueError("两侧可用输出行程须大于零")
    result = dict(speed=speed_value(fields["speed"]), travel=travel)
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
        text = fields[key].strip()
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
        text = fields[key].strip()
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
    return f"APP · {state}　故障码 0x{motor['faults']:08X}"


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
                    ambient=settings["ambient"],
                    previous=settings["last_lease"],
                    previous_target=settings["last_target"],
                    stopped=stop.is_set,
                    progress=lambda msg: events.put(("stage", msg)),
                )
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
        self.root.minsize(820, 565)
        self.root.geometry("1000x630")
        self.process = None
        self.events = None
        self.commands = None
        self.active_profile = None
        self.stop_signal = None
        self.report_dir = None
        self.close_pending = False
        self.curve_path = None
        self.done = False
        self.controls = []
        self.identity = None
        self.current_motor = None
        self.known_motors = {}
        self.previous_node = 6
        self.first_fit_selected = False
        self.swd_busy = False
        self.scanning = False
        frame = ttk.Frame(root, padding=16)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="CAN FD 通信测试", font=("Microsoft YaHei UI", 17, "bold")).pack(
            anchor="w"
        )
        top = ttk.Frame(frame)
        top.pack(fill="x", pady=(6, 12))

        def combo(parent, label, values, default, width):
            ttk.Label(parent, text=label).pack(side="left")
            var = tk.StringVar(value=default)
            w = ttk.Combobox(parent, textvariable=var, values=values, width=width, state="readonly")
            w.pack(side="left", padx=(4, 12))
            self.controls.append((w, "readonly"))
            return var

        self.channel = combo(top, "USB-CAN 通道", ("1", "2"), "1", 3)
        self.nominal = combo(top, "仲裁段", ("500K", "1M"), "1M", 6)
        self.data = combo(top, "数据段", tuple(RATES), "5M", 6)
        ttk.Label(top, text="ISO FD + BRS").pack(side="left")
        self.axis_choices = {
            f"{role} · {model} · 节点 {node}": node
            for node, (role, model, _) in FIRST_FIT_AXES.items()
        }
        motor_row = ttk.Frame(frame)
        motor_row.pack(fill="x", pady=(0, 8))
        self.motor_axis = combo(
            motor_row,
            "当前电机",
            tuple(self.axis_choices),
            next(label for label, node in self.axis_choices.items() if node == 6),
            28,
        )
        self.motor_selector = next(
            child for child in motor_row.winfo_children() if isinstance(child, ttk.Combobox)
        )
        self.motor_selector.bind("<<ComboboxSelected>>", self.select_first_fit)
        scan_motor = ttk.Button(motor_row, text="扫描识别", command=lambda: self.start(scan=True))
        scan_motor.pack(side="left")
        self.controls.append((scan_motor, "normal"))
        self.current_motor_label = tk.StringVar(value="未识别；烧录后自动沿用轴类型和节点。")
        ttk.Label(frame, textvariable=self.current_motor_label).pack(anchor="w", pady=(0, 6))
        self.tabs = ttk.Notebook(frame)
        self.tabs.pack(fill="x")
        self.pages = {}
        for key, title in (
            ("jlink", "J-Link 烧录"),
            ("hardware", "硬件测试"),
            ("communication", "通信测试"),
            ("calibration", "角度标定"),
            ("upgrade", "升级压测"),
            ("speed", "速度控制"),
        ):
            page = self.pages[key] = ttk.Frame(self.tabs, padding=10)
            self.tabs.add(page, text=title)
        self.swd_tab = self.pages["jlink"]
        check_tab = self.pages["hardware"]
        upgrade_tab = self.pages["upgrade"]
        self.communication_tabs = ttk.Notebook(self.pages["communication"])
        self.communication_tabs.pack(fill="both", expand=True)
        self.query_tab = ttk.Frame(self.communication_tabs, padding=10)
        self.raw_tab = ttk.Frame(self.communication_tabs, padding=10)
        self.communication_tabs.add(self.query_tab, text="通信压测")
        self.communication_tabs.add(self.raw_tab, text="协议发送")
        check_row = ttk.Frame(check_tab)
        check_row.pack(anchor="w")
        self.check_mode = combo(
            check_row, "检测方式", ("关相静态", "静态和双向驱动"), CHECK_DEFAULTS["mode"], 14
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
                ("lease", "上次续租计数"),
                ("target", "上次目标序号"),
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
        ttk.Label(
            check_tab,
            text="双向驱动前确认从起点两侧可用的输出轴行程。首次驱动填写已知计数，确认刚上电可填 0。",
        ).pack(anchor="w")
        ttk.Label(
            check_tab,
            text="APP v1.8.4：正命令角度增加、反命令角度减少；按50ms均值趋势自动验收，无需现场旋向。",
        ).pack(anchor="w")
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
        self.motor_fields = {}
        for index, (key, label, default) in enumerate(
            (
                ("speed", "电机轴目标速度 rad/s", "0"),
                ("limit", "速度上限 rad/s", "30"),
                ("travel", "两侧可用输出行程 °", "360"),
                ("lease", "首次续租计数", "0"),
                ("target", "首次目标序号", "0"),
            )
        ):
            if index in (0, 3):
                motor_values = ttk.Frame(speed_tab)
                motor_values.pack(anchor="w", pady=5)
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
            text="使能前应用速度上限到RAM，不自动保存Flash；保持电流限值和PI，断联后停止。",
        ).pack(anchor="w", pady=3)
        self.builtin_package = (
            self.resource / "can-app"
            if getattr(sys, "frozen", False)
            else self.resource.parents[1] / "outputs/AT32F456-FULL-v1.8.4/app-package"
        )
        self.upgrade_package = self.builtin_package
        self.upgrade_package_name = tk.StringVar(value="内置 APP 1.8.4")
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
        bitrate_row = ttk.Frame(upgrade_tab)
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
        ttk.Label(
            self.swd_tab, text="AT32F456 · Loader 2.3.4 + APP 1.8.4 · 默认 CAN FD 1M/5M"
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
        self.restart_after_flash = tk.BooleanVar(value=True)
        restart = ttk.Checkbutton(
            self.swd_tab,
            text="校验通过后复位，并通过 USB-CAN 查询当前电机",
            variable=self.restart_after_flash,
        )
        restart.pack(anchor="w")
        self.controls.append((restart, "normal"))
        self.motor_label = tk.StringVar(value="接好已停转电机的 J-Link 后，点击“识别电机”。")
        ttk.Label(self.swd_tab, textvariable=self.motor_label).pack(anchor="w")
        setup_row = ttk.Frame(self.swd_tab)
        setup_row.pack(anchor="w", pady=4)
        self.rescan = ttk.Button(setup_row, text="重新识别", command=self.identify_again)
        self.rescan.pack(side="left")
        ttk.Label(setup_row, text="空白板初装：在顶部选择实际电机类型一次。").pack(
            side="left", padx=8
        )
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
        bar = ttk.Frame(frame)
        bar.pack(fill="x", pady=(12, 5))
        self.bar = ttk.Progressbar(bar, maximum=100)
        self.bar.pack(side="left", fill="x", expand=True)
        self.progress = tk.StringVar(value="0%")
        ttk.Label(bar, textvariable=self.progress, width=8, anchor="e").pack(side="left")
        buttons = ttk.Frame(frame)
        buttons.pack(fill="x")
        self.start_button = ttk.Button(buttons, text="开始测试", command=self.start)
        self.start_button.pack(side="left")
        self.stop_button = ttk.Button(buttons, text="停止", command=self.stop)
        self.stop_button.pack(side="left", padx=5)
        self.open_button = ttk.Button(
            buttons, text="打开本次记录", command=lambda: os.startfile(self.report_dir)
        )
        self.open_button.pack(side="left")
        self.state = tk.StringVar(value="就绪")
        ttk.Label(buttons, textvariable=self.state).pack(side="right")
        self.output = tk.Text(
            frame, height=8, wrap="word", state="disabled", font=("Microsoft YaHei UI", 10)
        )
        self.output.pack(fill="both", expand=True, pady=(8, 0))
        self.mode.trace_add("write", lambda *_: self.update_controls())
        self.motor_axis.trace_add("write", self.select_current_motor)
        for var in (self.channel, self.nominal, self.data):
            var.trace_add("write", self.connection_changed)
        self.tabs.bind("<<NotebookTabChanged>>", lambda _: self.update_controls())
        self.communication_tabs.bind("<<NotebookTabChanged>>", lambda _: self.update_controls())
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.update_controls()

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
        self.tabs.select(self.swd_tab)
        self.start()

    def current_page(self) -> str:
        return next(key for key, page in self.pages.items() if str(page) == self.tabs.select())

    def current_node(self) -> int:
        return self.axis_choices[self.motor_axis.get()]

    def select_first_fit(self, _event=None) -> None:
        if self.identity and self.identity["node"] is None:
            self.first_fit_selected = True
        self.update_controls()

    def select_current_motor(self, *_args) -> None:
        node = self.current_node()
        self.current_motor = self.known_motors.get(node)
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
                f"UID {self.current_motor['uid']}　来源：{self.current_motor['source']}"
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
        self.update_controls()

    def set_current_motor(self, node: int, uid: str, source: str, *, role=None, **metadata) -> None:
        if node not in FIRST_FIT_AXES:
            return
        self.known_motors[node] = dict(metadata, node=node, uid=uid, source=source, role=role)
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
                if busy and self.tabs.tabs()[index] != self.tabs.select()
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
            "jlink": "识别电机",
            "upgrade": "开始 CAN 升级",
            "hardware": "开始硬件功能检测",
            "calibration": "开始角度标定并保存",
            "speed": "使能速度控制",
        }[current]
        speed_active = busy and self.active_profile == "motor_speed"
        for key, entry in self.motor_fields.items():
            entry.configure(
                state="normal" if (speed_active and key == "speed") or not busy else "disabled"
            )
        self.speed_update.configure(state="normal" if speed_active else "disabled")
        self.curve_button.configure(state="normal" if self.curve_path and not busy else "disabled")
        if current == "calibration":
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
        if blank and "resume_node" in self.identity:
            self.motor_selector.configure(state="disabled")
        if swd and self.identity:
            action = "按所选型号初装" if blank else f"烧录节点 {self.identity['node']}"
        self.start_button.configure(
            text=action,
            state="disabled"
            if busy
            or (current == "upgrade" and not selected)
            or (
                swd
                and self.identity
                and (not self.hex_path or (blank and not self.first_fit_selected))
            )
            else "normal",
        )
        self.rescan.configure(state="normal" if not busy and self.identity else "disabled")
        self.stop_button.configure(state="normal" if busy and not self.swd_busy else "disabled")
        self.stop_button.configure(
            text="本台完成后停止" if self.active_profile == "can_upgrade" else "停止"
        )
        self.open_button.configure(state="normal" if self.report_dir else "disabled")
        self.mode_hint.set(
            {
                "fixed": "状态查询 · 首答 50 ms",
                "ladder": "5 → 50 → 200 → 500 → 5 笔/秒 · 首答 50 ms",
                "acceptance": "五轴各 70,000 笔 · 轮询和连续发送 · 首答 50 ms",
            }[mode]
            + "　｜　控制器正常发送 · 应用层重试 0 次"
        )

    def show(self, text: str) -> None:
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

    def operation_mode(self, *, scan=False, bitrate_action=None) -> str:
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
            "jlink": "jlink_flash" if self.identity else "jlink_identify",
            "upgrade": "can_upgrade",
            "hardware": "hardware_check",
            "calibration": "motor_calibration",
            "speed": "motor_speed",
        }[page]

    def start(self, *, scan=False, bitrate_action=None, calibration_kind=1) -> None:
        if self.process is not None:
            return
        mode = self.operation_mode(scan=scan, bitrate_action=bitrate_action)
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
                if mode == "jlink_flash":
                    self.clear_upgrade_scan()
                settings = {
                    "profile": mode,
                    "identity": self.identity,
                    "hex": str(self.hex_path or ""),
                    "restart_after_flash": self.restart_after_flash.get(),
                    "first_fit_node": self.current_node()
                    if self.identity and self.identity["node"] is None and self.first_fit_selected
                    else None,
                    "bus": settings["bus"],
                }
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
            context = mp.get_context("spawn")
            self.events = context.Queue()
            self.stop_signal = context.Event()
            self.commands = context.Queue(maxsize=8) if mode == "motor_speed" else None
            arguments = (settings, directory, self.events, self.stop_signal)
            if mode == "motor_speed":
                arguments += (self.commands,)
            job = context.Process(
                target=run_upgrade_test if mode == "can_upgrade" else run_jlink if swd else acquire,
                args=arguments,
            )
            job.start()
        except (ValueError, OSError, RuntimeError) as exc:
            messagebox.showerror("无法启动", str(exc), parent=self.root)
            return
        self.swd_busy = swd
        self.scanning = mode == "can_scan"
        if mode == "can_upgrade":
            for target in targets:
                self.upgrade_results[target["node"]].configure(text="排队中")
        self.process = job
        self.active_profile = mode
        self.report_dir = directory
        self.done = False
        self.bar["value"] = 0
        self.progress.set("0%")
        self.state.set(
            "检查 CAN 升级包" if mode == "can_upgrade" else "连接 J-Link" if swd else "正在采集"
        )
        self.show(f"记录目录　{directory}")
        self.update_controls()
        self.root.after(80, self.poll)

    def stop(self) -> None:
        if self.process and not self.swd_busy:
            self.stop_signal.set()
            self.stop_button.configure(state="disabled")
            self.state.set(
                "将在当前电机确认后停止" if self.active_profile == "can_upgrade" else "正在停止"
            )

    def close(self) -> None:
        if self.process:
            self.close_pending = True
            if self.swd_busy:
                self.state.set("等待 SWD 操作完成后关闭")
            else:
                self.stop()
        else:
            self.root.destroy()

    def poll(self) -> None:
        finished = not self.process.is_alive()
        while True:
            try:
                tag, msg = self.events.get_nowait()
            except queue.Empty:
                break
            if tag == "stage":
                self.state.set(msg.split(" s  ", 1)[-1])
                self.output.configure(state="normal")
                self.output.insert("end", "\n" + msg)
                self.output.see("end")
                self.output.configure(state="disabled")
            elif tag == "axis_status":
                node, status = msg
                self.upgrade_results[node].configure(text=status)
            elif tag == "motor_feedback":
                self.motor_feedback.set(
                    f"电机轴：目标{msg['target_rad_s']:+.3f} rad/s，实际{msg['actual_rad_s']:+.3f} rad/s"
                )
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
        if not finished:
            self.root.after(80, self.poll)
            return
        self.process.join()
        self.process.close()
        self.events.close()
        if self.commands is not None:
            self.commands.close()
            self.commands = None
        self.process = None
        self.active_profile = None
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
            self.root.destroy()
        else:
            self.update_controls()

    def display_report(self, r: dict) -> None:
        if r["profile"] in ("motor_calibration", "motor_speed"):
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
            if "fatal" in r:
                self.state.set("通信异常")
                self.show(r["fatal"])
                return
            self.state.set("硬件功能检测 · 通信" + r["communication"])
            if r.get("communication_reliability_passed") is False:
                self.state.set(self.state.get() + " · 50ms首答异常")
            self.show(summarize(r))
            self.bar["value"] = 100 if r["stage"] in (4, 5) else 0
            self.progress.set(f"{self.bar['value']:.0f}%")
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
            self.known_motors = {m["node"]: dict(m, source="CAN扫描") for m in motors}
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
            if r.get("success"):
                identity = r["identity"]
                name = (
                    "待初装板"
                    if identity["node"] is None
                    else f"节点 {identity['node']}（{AXES[identity['node']]}）"
                )
                if r["profile"] == "jlink_identify":
                    self.identity = identity
                    detail = f"{name}　UID {identity['uid']}　J-Link {identity['serial']}"
                    if identity["node"] is None:
                        detail += "　请选择与实物相符的轴和型号。"
                        if "resume_node" in identity:
                            self.motor_axis.set(
                                next(
                                    label
                                    for label, n in self.axis_choices.items()
                                    if n == identity["resume_node"]
                                )
                            )
                            self.first_fit_selected = True
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
                        self.motor_label.set(f"{name} 已启动，身份及 10 次状态查询通过。")
                    else:
                        self.state.set(f"{name} · 已核验")
                        self.motor_label.set(
                            "芯片已暂停，请重新上电后测试通信。"
                            if r.get("cpu_halted")
                            else "已跳过重启。"
                        )
                if identity["node"] is not None:
                    self.set_current_motor(
                        identity["node"],
                        identity["uid"],
                        "J-Link识别" if r["profile"] == "jlink_identify" else "J-Link烧录",
                        role=1 if r.get("app_started") else None,
                    )
                self.bar["value"] = 100
                self.progress.set("100%")
            else:
                self.state.set("SWD 操作失败")
                self.motor_label.set("排除故障后重新识别当前电机。")
                detail = r.get("fatal", "未取得烧录结果")
                if r.get("app_started"):
                    self.state.set("APP 已启动 · CAN 查询失败")
                    self.motor_label.set("请检查 USB-CAN 连接及通道，再进行通信查询。")
                elif r.get("protected_regions_verified"):
                    detail += "\n固件已核验，请重新识别后重试启动。"
                elif r.get("write_started"):
                    detail += "\n已开始写入，须重新校验镜像后再上电。"
            self.show(
                "\n".join(
                    [
                        *r.get("steps", []),
                        detail,
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
