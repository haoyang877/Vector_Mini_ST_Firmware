"""Windows USB-CAN 桌面工具。界面只负责参数与显示，采集进程独占 SDK。"""

import csv
import ctypes as C
import json
import multiprocessing as mp
import os
import queue
import sys
import time
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

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
from motor_can_bitrate import configure as configure_bitrate
from ports.controlcanfd import _RXFD, _TXFD

TITLE = "CAN FD 通信测试 v1.6.1"
AXES = {3: "左轮", 4: "右轮", 5: "ROLL", 6: "PITCH", 7: "YAW", 8: "Lift"}
MODES = {"常规查询": "fixed", "阶梯压测": "ladder", "五轴稳定性验收": "acceptance"}
RATES = {"500K": 500000, "1M": 1000000, "2M": 2000000, "3M": 3000000, "4M": 4000000, "5M": 5000000}
FD_LENGTHS = (*range(9), 12, 16, 20, 24, 32, 48, 64)


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


def acquire(settings: dict, out: Path, events, stop) -> None:
    """子进程入口。适配器和日志释放之后才通知界面结束。"""
    try:
        resource = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
        dll = controlcan_dll_path(resource)
        if settings["profile"] == "can_scan":
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
        elif settings["profile"] == "manual":
            report = send_manual(dll, out, settings, stop)
            (out / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        else:
            batch = json.loads(
                (resource / "five_axis_can_stability.json").read_text(encoding="utf-8")
            )
            uids = desktop_uids(resource, out.parent)
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
        report = {"profile": settings["profile"], "fatal": str(exc), "pass": False}
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
        self.stop_signal = None
        self.report_dir = None
        self.close_pending = False
        self.done = False
        self.controls = []
        self.identity = None
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
        self.tabs = ttk.Notebook(frame)
        self.tabs.pack(fill="x")
        self.query_tab = ttk.Frame(self.tabs, padding=10)
        self.raw_tab = ttk.Frame(self.tabs, padding=10)
        self.tabs.add(self.query_tab, text="通信压测")
        self.tabs.add(self.raw_tab, text="协议发送")
        self.swd_tab = ttk.Frame(self.tabs, padding=10)
        self.tabs.add(self.swd_tab, text="J-Link 烧录")
        upgrade_tab = ttk.Frame(self.tabs, padding=10)
        self.tabs.add(upgrade_tab, text="CAN 升级与参数")
        self.resource = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
        self.builtin_package = (
            self.resource / "can-app"
            if getattr(sys, "frozen", False)
            else self.resource.parents[1] / "outputs/AT32F456-FULL-v0.7.2/app-package"
        )
        self.upgrade_package = self.builtin_package
        self.upgrade_package_name = tk.StringVar(value="内置 APP 0.7.2")
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
        self.upgrade_info = tk.StringVar(value="请扫描当前通道上的五轴电机（节点 3–7）。")
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
            self.swd_tab, text="AT32F456 · Loader 2.3.2 + APP 0.7.2 · 默认 CAN FD 1M/5M"
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
        self.axis_choices = {
            f"{role} · {model} · 节点 {node}": node
            for node, (role, model, _) in FIRST_FIT_AXES.items()
        }
        self.new_axis = ttk.Combobox(
            setup_row, values=tuple(self.axis_choices), width=28, state="disabled"
        )
        self.new_axis.pack(side="left", padx=8)
        self.new_axis.bind("<<ComboboxSelected>>", lambda _: self.update_controls())
        axes = ttk.LabelFrame(self.query_tab, text="查询节点", padding=7)
        axes.pack(fill="x")
        self.nodes = {}
        for node, label in AXES.items():
            self.nodes[node] = tk.BooleanVar(value=node != 8)
            w = ttk.Checkbutton(axes, text=f"{label} 0x{node:02X}", variable=self.nodes[node])
            w.pack(side="left", padx=4)
            self.controls.append((w, "normal"))
        options = ttk.Frame(self.query_tab)
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
        for node, label in AXES.items():
            w = ttk.Button(samples, text=label, width=6, command=lambda n=node: self.fill_query(n))
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
        for var in (self.channel, self.nominal, self.data):
            var.trace_add("write", lambda *_: self.clear_upgrade_scan())
        self.tabs.bind("<<NotebookTabChanged>>", lambda _: self.update_controls())
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
        self.upgrade_info.set("请扫描当前通道上的五轴电机（节点 3–7）。")
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
        self.new_axis.set("")
        self.motor_label.set("识别当前连接的电机。")
        self.start()

    def fill_query(self, node: int) -> None:
        ident, body = encode_frames(node, 108, b"", 1)[0]
        self.identifier.set(f"0x{ident:08X}")
        self.payload.set(body.hex(" ").upper())

    def update_controls(self) -> None:
        busy = self.process is not None
        mode = MODES[self.mode.get()]
        current = self.tabs.index("current")
        swd = current == 2
        for control, state in self.controls:
            control.configure(state="disabled" if busy else state)
        for entry in self.entries:
            entry.configure(state="normal" if not busy and mode == "fixed" else "disabled")
        for index in range(len(self.tabs.tabs())):
            self.tabs.tab(index, state="disabled" if busy and index != current else "normal")
        action = ("开始测试", "发送一帧", "识别电机", "开始 CAN 升级")[current]
        for node, check in self.upgrade_checks.items():
            check.configure(
                state="normal" if node in self.upgrade_targets and not busy else "disabled"
            )
        selected_motors = [
            m for n, m in self.upgrade_targets.items() if self.upgrade_selected[n].get()
        ]
        selected = len(selected_motors)
        bitrate_state = (
            "normal"
            if not busy and selected and all(m["role"] == 1 for m in selected_motors)
            else "disabled"
        )
        self.bitrate_read.configure(state=bitrate_state)
        self.bitrate_save.configure(state=bitrate_state)
        if current == 3 and selected:
            action = f"开始升级（{selected} 台）"
        blank = self.identity is not None and self.identity["node"] is None
        self.new_axis.configure(
            state="readonly"
            if blank and not busy and "resume_node" not in self.identity
            else "disabled"
        )
        if swd and self.identity:
            action = "按所选型号初装" if blank else f"烧录节点 {self.identity['node']}"
        self.start_button.configure(
            text=action,
            state="disabled"
            if busy
            or (current == 3 and not selected)
            or (
                swd
                and self.identity
                and (not self.hex_path or (blank and self.new_axis.get() not in self.axis_choices))
            )
            else "normal",
        )
        self.rescan.configure(state="normal" if not busy and self.identity else "disabled")
        self.stop_button.configure(state="normal" if busy and not self.swd_busy else "disabled")
        self.stop_button.configure(
            text="本台完成后停止" if current == 3 and not self.scanning else "停止"
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

    def start(self, *, scan=False, bitrate_action=None) -> None:
        if self.process is not None:
            return
        mode = (
            MODES[self.mode.get()],
            "manual",
            "jlink_flash" if self.identity else "jlink_identify",
            "can_bitrate" if bitrate_action else "can_scan" if scan else "can_upgrade",
        )[self.tabs.index("current")]
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
                    "first_fit_node": self.axis_choices.get(self.new_axis.get())
                    if self.identity and self.identity["node"] is None
                    else None,
                    "bus": settings["bus"],
                }
            elif mode == "can_scan":
                self.upgrade_info.set("正在扫描当前通道…")
            elif mode in ("can_upgrade", "can_bitrate"):
                targets = [
                    {"node": n, "uid": m["uid"]}
                    for n, m in self.upgrade_targets.items()
                    if self.upgrade_selected[n].get()
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
                nodes = [n for n, selected in self.nodes.items() if selected.get()]
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
            directory = Path.home() / "CAN-Link-Test" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            directory.mkdir(parents=True, exist_ok=False)
            context = mp.get_context("spawn")
            self.events = context.Queue()
            self.stop_signal = context.Event()
            job = context.Process(
                target=run_upgrade_test if mode == "can_upgrade" else run_jlink if swd else acquire,
                args=(settings, directory, self.events, self.stop_signal),
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
                "将在当前电机确认后停止"
                if self.tabs.index("current") == 3 and not self.scanning
                else "正在停止"
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
        self.process = None
        if not self.done:
            self.identity = None
            self.motor_label.set("请重新识别电机。")
            if self.tabs.index("current") == 3:
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
            if not r.get("cancelled") and not r.get("fatal"):
                for n in self.upgrade_lights:
                    self.upgrade_lights[n].configure(text="● 未识别", foreground="#656565")
            for motor in motors:
                n = motor["node"]
                self.show_online_motor(motor)
                self.upgrade_selected[n].set(len(motors) == 1 or previous.get(n) == motor["uid"])
            status = "扫描已停止" if r.get("cancelled") else f"发现 {len(motors)} 台在线电机"
            if r.get("fatal"):
                status = "扫描失败"
            self.state.set(status)
            self.upgrade_info.set(f"{datetime.now():%H:%M:%S} 扫描 · {status}。可勾选多台。")
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
                motor = self.upgrade_targets.get(n)
                if motor is None or motor["uid"] != result["uid"]:
                    continue
                progress = f"{result['passed']}/{r['rounds_per_motor']} 次通过"
                if result.get("identity_changed_or_missing"):
                    del self.upgrade_targets[n]
                    self.upgrade_selected[n].set(False)
                    self.upgrade_lights[n].configure(text="● 待识别", foreground="#995900")
                    self.upgrade_details[n].configure(text="地址或 UID 需重新扫描")
                    self.upgrade_uids[n].configure(text="—")
                elif result["failed"]:
                    self.upgrade_lights[n].configure(text="● 待复核", foreground="#995900")
                elif result["passed"]:
                    motor.update(role=1, boot_state=1, app_version=r["version"], state=None)
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
            return
        if r["profile"].startswith("jlink_"):
            self.identity = None
            self.new_axis.set("")
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
                            self.new_axis.current(
                                list(self.axis_choices.values()).index(identity["resume_node"])
                            )
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
