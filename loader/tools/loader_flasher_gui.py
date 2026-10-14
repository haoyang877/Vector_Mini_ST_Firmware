"""Windows图形初装工具；按钮触发探针读取/烧录，导入及离线检查不接触硬件。"""

import argparse
from datetime import datetime, timedelta, timezone
import json
import os
from pathlib import Path
import queue
import struct
import sys
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from types import SimpleNamespace

from loader_factory import check_power_off, flash_package, load_hex
from maintenance_backend import (
    can_snapshot,
    flash_snapshot,
    load_upgrade,
    run_can_upgrade,
)


def find_vendor_dll(name: str) -> str:
    """优先使用冻结EXE内置DLL；开发态可使用EXE旁的同名文件。"""
    roots = [
        Path(getattr(sys, "_MEIPASS", Path(__file__).parent)),
        Path(sys.executable).parent,
    ]
    for root in roots:
        for path in (root / "drivers" / name, root / name):
            if path.is_file():
                return str(path)
    return ""


def find_jlink_dll() -> str:
    """自动使用内置J-Link DLL；开发态回退到本机已安装的SEGGER库。"""
    bundled = find_vendor_dll("JLink_x64.dll")
    if bundled:
        return bundled
    program_files = os.environ.get("ProgramFiles")
    if program_files:
        candidates = sorted(
            (Path(program_files) / "SEGGER").glob("JLink*/JLink_x64.dll")
        )
        if candidates:
            return str(candidates[-1])
    return ""


def check_bundled_drivers() -> dict:
    """显式离线验证内置DLL的哈希及导出，不调用打开设备、枚举或发送接口。"""
    import ctypes
    import hashlib

    directory = Path(getattr(sys, "_MEIPASS", Path(__file__).parent)) / "drivers"
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    reports = []
    for name, export in (
        ("JLink_x64.dll", "JLINKARM_Open"),
        ("ControlCANFD.dll", "ZCAN_OpenDevice"),
    ):
        path = directory / name
        if find_vendor_dll(name) != str(path):
            raise ValueError("未自动选择内置DLL：" + name)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != manifest["files"][name]["sha256"]:
            raise ValueError("内置DLL哈希不一致：" + name)
        library = ctypes.WinDLL(str(path))
        getattr(library, export)
        reports.append(
            {"name": name, "sha256": digest, "loaded": True, "export": export}
        )
    return {
        "status": "PASS",
        "hardware_accessed": False,
        "device_open_called": False,
        "drivers": reports,
    }


def open_probe(dll: Path):
    """创建显式DLL的探针对象；本函数不打开探针或连接MCU。"""
    import pylink

    return pylink.JLink(lib=pylink.library.Library(dllpath=str(dll)))


def inspect_target(dll: Path, serial: int, speed_khz: int = 1000) -> dict:
    """读取指定探针的芯片身份及关相状态，不复位、不烧录、不恢复原已暂停CPU。"""
    import pylink

    probe = open_probe(dll)
    opened, was_running = False, False
    try:
        probe.open(serial_no=serial)
        opened = True
        probe.exec_command("SetRestartOnClose = 0")
        probe.set_tif(pylink.enums.JLinkInterfaces.SWD)
        probe.connect("STM32G431CB", speed=speed_khz)
        was_running = not probe.halted()
        flash_kib = probe.memory_read16(0x1FFF75E0, 1)[0]
        device_id = probe.memory_read32(0xE0042000, 1)[0] & 0xFFF
        if flash_kib != 128 or device_id != 0x468:
            raise RuntimeError("目标不是128 KiB STM32G431，不能烧录")
        power = check_power_off(probe)
        return {
            "serial": serial,
            "uid": struct.pack("<3I", *probe.memory_read32(0x1FFF7590, 3)).hex(),
            "reference_voltage_mv": probe.hardware_status.VTarget,
            "flash_kib": flash_kib,
            "power": power,
            "speed_khz": speed_khz,
        }
    finally:
        if opened:
            try:
                # 只读检查保留进入前的运行态；原已暂停目标不启动。
                probe.exec_command(
                    "SetRestartOnClose = " + ("1" if was_running else "0")
                )
            finally:
                probe.close()


def version_text(code: int) -> str:
    """将固件0xMMmmppbb版本编码显示为三段版本及非零build。"""
    text = ".".join(str((code >> shift) & 255) for shift in (24, 16, 8))
    return text + (f" (build {code & 255})" if code & 255 else "")


class FlasherWindow:
    """单次操作锁定输入；所有Tk调用归主线程，硬件及文件IO归后台线程。"""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.busy = False
        self.binding = None
        self.can_binding = None
        self.firmware = None
        self.upgrade_firmware = None
        self.calibration = None
        self.last_out = None
        self.events = queue.Queue()
        self.controls = []
        self.page_titles = []
        self.nav_buttons = []
        self.logs_visible = False
        root.title("Vector Mini · 设备维护")
        root.tk.call("tk", "scaling", 96 / 72)
        width = min(1120, root.winfo_screenwidth() - 60)
        height = min(810, root.winfo_screenheight() - 100)
        root.geometry(f"{width}x{height}+30+30")
        root.minsize(940, 740)
        root.configure(background="#f5f7fb")
        self.configure_style()
        self.hex_path = tk.StringVar()
        self.dll_path = tk.StringVar(value=find_jlink_dll())
        self.probe_serial = tk.StringVar()
        self.info = tk.StringVar(value="选择完整 APP + Loader 合并HEX")
        self.target_info = tk.StringVar(value="尚未绑定J-Link目标")
        self.status = tk.StringVar(value="就绪")
        self.run_after = tk.BooleanVar(value=True)
        self.speed_khz = tk.StringVar(value="1000")
        self.can_dll = tk.StringVar(value=find_vendor_dll("ControlCANFD.dll"))
        self.can_index = tk.StringVar(value="0")
        self.can_channel = tk.StringVar(value="0")
        self.can_node = tk.StringVar(value="0")
        self.can_info = tk.StringVar(value="尚未绑定CAN设备 · 1M / 1M")
        self.upgrade_path = tk.StringVar()
        self.upgrade_info = tk.StringVar(value="可选合并HEX，或APP包的manifest.json")
        self.query_transport = tk.StringVar(value="J-Link")
        self.snapshot_info = tk.StringVar(value="按需读取；不自动轮询或切换电机模式")
        self.calibration_info = tk.StringVar(
            value="读取Flash中保存的标定参数；不启动标定"
        )
        self.calibration_group = tk.StringVar(value="全部")
        self.page_heading = tk.StringVar(value="初装烧录")
        self.page_subtitle = tk.StringVar()
        self.footer = None
        self.shell = tk.Frame(root, background="#f5f7fb")
        self.shell.pack(fill="both", expand=True)
        self.build_sidebar()
        main = ttk.Frame(self.shell, style="Workspace.TFrame", padding=(26, 22, 26, 18))
        main.pack(side="left", fill="both", expand=True)
        header = ttk.Frame(main, style="Workspace.TFrame")
        header.pack(fill="x", pady=(0, 20))
        ttk.Label(
            header, text="设备管理  /  固件与标定", style="Breadcrumb.TLabel"
        ).pack(anchor="w")
        title_row = ttk.Frame(header, style="Workspace.TFrame")
        title_row.pack(fill="x", pady=(7, 4))
        ttk.Label(
            title_row, textvariable=self.page_heading, style="PageTitle.TLabel"
        ).pack(side="left")
        self.status_badge = tk.Label(
            title_row,
            textvariable=self.status,
            font=("Microsoft YaHei UI", 10),
            foreground="#047857",
            background="#e7f7ef",
            padx=13,
            pady=6,
        )
        self.status_badge.pack(side="right")
        self.status.trace_add("write", lambda *_: self.update_status_badge())
        ttk.Label(
            header, textvariable=self.page_subtitle, style="Breadcrumb.TLabel"
        ).pack(anchor="w")
        self.build_connections(main)
        self.tabs = ttk.Notebook(main, style="Pages.TNotebook")
        self.tabs.pack(fill="both", expand=True, pady=(18, 0))
        self.build_write_pages()
        self.build_read_pages()
        self.tabs.bind("<<NotebookTabChanged>>", self.navigation_changed)
        self.build_footer(main)
        self.navigation_changed()
        self.write_log("已准备就绪。请选择设备和固件，再读取目标。")
        self.draw_lut()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self.poll)

    def configure_style(self):
        """去掉默认立体边框，统一颜色、字体和可见的键盘焦点。"""
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(
            ".",
            font=("Microsoft YaHei UI", 10),
            background="white",
            foreground="#334155",
        )
        style.configure("Workspace.TFrame", background="#f5f7fb")
        style.configure("Card.TFrame", background="white")
        style.configure("Card.TLabel", background="white")
        style.configure(
            "Muted.TLabel",
            background="white",
            foreground="#7c8ba1",
            font=("Microsoft YaHei UI", 9),
        )
        style.configure(
            "Breadcrumb.TLabel",
            background="#f5f7fb",
            foreground="#8793a6",
            font=("Microsoft YaHei UI", 9),
        )
        style.configure(
            "PageTitle.TLabel",
            background="#f5f7fb",
            foreground="#14213d",
            font=("Microsoft YaHei UI", 21, "bold"),
        )
        style.configure(
            "CardTitle.TLabel",
            background="white",
            foreground="#16243e",
            font=("Microsoft YaHei UI", 11, "bold"),
        )
        style.configure(
            "TEntry",
            padding=(10, 9),
            fieldbackground="#f8fafc",
            bordercolor="#e5eaf1",
            lightcolor="#e5eaf1",
            darkcolor="#e5eaf1",
            borderwidth=1,
        )
        style.map(
            "TEntry",
            fieldbackground=[("readonly", "#f8fafc"), ("disabled", "#f8fafc")],
            bordercolor=[("focus", "#8baefa")],
        )
        style.configure(
            "TCombobox",
            padding=(9, 7),
            fieldbackground="white",
            background="white",
            arrowcolor="#8190a5",
            bordercolor="#e5eaf1",
            lightcolor="#e5eaf1",
            darkcolor="#e5eaf1",
            borderwidth=1,
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", "white"), ("disabled", "#f3f5f8")],
            selectbackground=[("readonly", "white")],
            selectforeground=[("readonly", "#334155")],
            background=[("readonly", "white")],
        )
        style.layout(
            "TButton",
            [
                (
                    "Button.padding",
                    {
                        "sticky": "nswe",
                        "children": [
                            (
                                "Button.focus",
                                {
                                    "sticky": "nswe",
                                    "children": [("Button.label", {"sticky": "nswe"})],
                                },
                            )
                        ],
                    },
                )
            ],
        )
        style.configure(
            "TButton",
            padding=(14, 10),
            background="#eef3fa",
            foreground="#4b6385",
            borderwidth=0,
            focuscolor="#97b7eb",
        )
        style.map(
            "TButton",
            background=[
                ("disabled", "#f0f3f7"),
                ("active", "#e3ebf6"),
                ("pressed", "#d7e3f5"),
            ],
            foreground=[("disabled", "#a5afbd")],
        )
        style.configure(
            "Accent.TButton",
            background="#3268f1",
            foreground="white",
            padding=(22, 12),
            font=("Microsoft YaHei UI", 10, "bold"),
        )
        style.map(
            "Accent.TButton",
            background=[
                ("disabled", "#e8edf5"),
                ("active", "#2558d8"),
                ("pressed", "#1e4cbf"),
            ],
            foreground=[("disabled", "#a0aec1"), ("!disabled", "white")],
        )
        style.configure(
            "Link.TButton", background="#f5f7fb", foreground="#3268f1", padding=(6, 4)
        )
        style.map(
            "Link.TButton", background=[("active", "#eaf0fa"), ("disabled", "#f5f7fb")]
        )
        style.configure(
            "TCheckbutton",
            background="white",
            indicatorbackground="white",
            indicatorforeground="#3268f1",
            indicatormargin=6,
            indicatorrelief="flat",
        )
        style.map(
            "TCheckbutton",
            background=[("active", "white")],
            indicatorbackground=[("selected", "#3268f1"), ("!selected", "#f1f5f9")],
            indicatorforeground=[("selected", "white")],
        )
        style.configure(
            "TRadiobutton",
            background="white",
            indicatorbackground="white",
            indicatorforeground="#3268f1",
        )
        style.map("TRadiobutton", background=[("active", "white")])
        self.checkbox_images = []
        for selected in (False, True):
            image = tk.PhotoImage(master=self.root, width=18, height=18)
            for y in range(2, 16):
                for x in range(2, 16):
                    if x in (2, 15) and y in (2, 15):
                        continue
                    edge = x in (2, 15) or y in (2, 15)
                    image.put(
                        "#3268f1" if selected else "#ced9e9" if edge else "white",
                        (x, y),
                    )
            if selected:
                for x, y in (
                    (5, 8),
                    (6, 9),
                    (7, 10),
                    (8, 9),
                    (9, 8),
                    (10, 7),
                    (11, 6),
                    (12, 5),
                ):
                    image.put("white", (x, y))
                    image.put("white", (x, y + 1))
            self.checkbox_images.append(image)
        style.element_create(
            "VM.Check.indicator",
            "image",
            self.checkbox_images[0],
            ("selected", self.checkbox_images[1]),
            sticky="w",
        )
        style.layout(
            "TCheckbutton",
            [
                (
                    "Checkbutton.padding",
                    {
                        "sticky": "nswe",
                        "children": [
                            ("VM.Check.indicator", {"side": "left", "sticky": "w"}),
                            (
                                "Checkbutton.focus",
                                {
                                    "side": "left",
                                    "sticky": "w",
                                    "children": [
                                        ("Checkbutton.label", {"sticky": "nswe"})
                                    ],
                                },
                            ),
                        ],
                    },
                )
            ],
        )
        style.configure(
            "TSeparator",
            background="#edf1f7",
            lightcolor="#edf1f7",
            darkcolor="#edf1f7",
            borderwidth=0,
        )
        style.layout("Pages.TNotebook", [])
        style.layout("Pages.TNotebook.Tab", [])
        style.configure(
            "Pages.TNotebook", background="#f5f7fb", borderwidth=0, padding=0
        )
        style.configure(
            "Treeview",
            rowheight=32,
            background="white",
            fieldbackground="white",
            borderwidth=0,
            bordercolor="white",
            lightcolor="white",
            darkcolor="white",
            font=("Microsoft YaHei UI", 10),
        )
        style.configure(
            "Treeview.Heading",
            padding=(8, 10),
            background="#f4f7fc",
            foreground="#7a8aa2",
            relief="flat",
            font=("Microsoft YaHei UI", 9),
        )
        style.map(
            "Treeview",
            background=[("selected", "#eaf1ff")],
            foreground=[("selected", "#2458c2")],
        )
        style.configure(
            "Vertical.TScrollbar",
            background="#d6dfeb",
            troughcolor="white",
            borderwidth=0,
            arrowcolor="#9aa8bc",
            width=10,
            lightcolor="white",
            darkcolor="white",
        )
        style.configure(
            "Horizontal.TScrollbar",
            background="#d6dfeb",
            troughcolor="white",
            borderwidth=0,
            arrowcolor="#9aa8bc",
            width=10,
            lightcolor="white",
            darkcolor="white",
        )
        style.configure(
            "Horizontal.TProgressbar",
            background="#3268f1",
            troughcolor="#e8edf5",
            bordercolor="#e8edf5",
            lightcolor="#3268f1",
            darkcolor="#3268f1",
            borderwidth=0,
            thickness=4,
        )

    def build_sidebar(self):
        """固定宽度导航与工作区分离，不触发任何设备查询。"""
        sidebar = tk.Frame(self.shell, background="#111d35", width=184)
        sidebar.pack(side="left", fill="y")
        sidebar.pack_propagate(False)
        brand = tk.Frame(sidebar, background="#111d35")
        brand.pack(fill="x", padx=20, pady=(28, 32))
        tk.Label(
            brand,
            text="VM",
            background="#3268f1",
            foreground="white",
            font=("Segoe UI", 16, "bold"),
            padx=10,
            pady=9,
        ).pack(anchor="w")
        tk.Label(
            brand,
            text="Vector Mini",
            background="#111d35",
            foreground="white",
            font=("Segoe UI", 15, "bold"),
        ).pack(anchor="w", pady=(14, 4))
        tk.Label(
            brand,
            text="设备维护工作台",
            background="#111d35",
            foreground="#8192b0",
            font=("Microsoft YaHei UI", 9),
        ).pack(anchor="w")
        tk.Label(
            sidebar,
            text="工作空间",
            background="#111d35",
            foreground="#627697",
            font=("Microsoft YaHei UI", 9),
        ).pack(anchor="w", padx=22, pady=(0, 10))
        for index, (title, detail) in enumerate(
            (
                ("初装烧录", "J-Link / 完整固件"),
                ("CAN 固件升级", "USB-CAN / APP"),
                ("版本与状态", "固件 / 运行信息"),
                ("标定参数", "零位 / 编码器 / 摩擦"),
            )
        ):
            frame = tk.Frame(sidebar, background="#111d35")
            frame.pack(fill="x", padx=12, pady=4)
            button = tk.Button(
                frame,
                text=f"{index + 1:02d}    {title}",
                command=lambda i=index: self.tabs.select(i),
                anchor="w",
                relief="flat",
                borderwidth=0,
                highlightthickness=0,
                background="#111d35",
                foreground="#a3b0c6",
                activebackground="#223352",
                activeforeground="white",
                font=("Microsoft YaHei UI", 10),
                padx=12,
                pady=12,
                cursor="hand2",
                takefocus=True,
            )
            button.pack(fill="x")
            self.nav_buttons.append(button)
            tk.Label(
                frame,
                text=detail,
                background="#111d35",
                foreground="#627697",
                font=("Microsoft YaHei UI", 8),
                anchor="w",
                padx=12,
            ).pack(fill="x", pady=(0, 6))
        footer = tk.Frame(sidebar, background="#111d35")
        footer.pack(side="bottom", fill="x", padx=22, pady=24)
        tk.Label(
            footer,
            text="STM32G431",
            background="#111d35",
            foreground="#a3b0c6",
            font=("Segoe UI", 9, "bold"),
        ).pack(anchor="w")
        tk.Label(
            footer,
            text="参数保留 · 单设备操作",
            background="#111d35",
            foreground="#627697",
            font=("Microsoft YaHei UI", 8),
        ).pack(anchor="w", pady=(5, 0))

    def card(self, parent, padding=18):
        """单像素浅边卡片，内部由调用方决定布局。"""
        border = tk.Frame(parent, background="#e6ebf2", padx=1, pady=1)
        inner = ttk.Frame(border, padding=padding, style="Card.TFrame")
        inner.pack(fill="both", expand=True)
        return border, inner

    def build_connections(self, parent):
        """连接卡片只展示设备选择；DLL由安装包自动提供。"""
        row = ttk.Frame(parent, style="Workspace.TFrame")
        row.pack(fill="x")
        row.columnconfigure(0, weight=1, uniform="connection")
        row.columnconfigure(1, weight=1, uniform="connection")
        border, link = self.card(row, 14)
        border.grid(row=0, column=0, sticky="nsew", padx=(0, 7))
        link.columnconfigure(0, weight=1)
        ttk.Label(link, text="J-Link", style="CardTitle.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(link, text="SWD", style="Muted.TLabel").grid(
            row=0, column=1, sticky="e"
        )
        self.probes = ttk.Combobox(
            link, textvariable=self.probe_serial, state="readonly", width=15
        )
        self.probes.grid(row=1, column=0, sticky="ew", pady=(12, 10), padx=(0, 8))
        self.probes.bind("<<ComboboxSelected>>", lambda event: self.clear_target())
        self.button(link, "刷新", self.refresh_probes).grid(
            row=1, column=1, pady=(12, 10)
        )
        ttk.Label(link, text="通讯速度", style="Muted.TLabel").grid(
            row=2, column=0, sticky="w"
        )
        speed_row = ttk.Frame(link, style="Card.TFrame")
        speed_row.grid(row=2, column=1, sticky="e")
        speed = ttk.Combobox(
            speed_row,
            textvariable=self.speed_khz,
            values=("100", "1000", "4000"),
            state="readonly",
            width=5,
        )
        speed.pack(side="left")
        speed.bind("<<ComboboxSelected>>", lambda event: self.clear_target())
        ttk.Label(speed_row, text=" kHz", style="Muted.TLabel").pack(side="left")
        self.controls.extend(((self.probes, "readonly"), (speed, "readonly")))
        border, can = self.card(row, 14)
        border.grid(row=0, column=1, sticky="nsew", padx=(7, 0))
        ttk.Label(can, text="USB-CAN", style="CardTitle.TLabel").pack(
            side="left", anchor="n"
        )
        fields = ttk.Frame(can, style="Card.TFrame")
        fields.pack(fill="both", expand=True, padx=(20, 0))
        ttk.Label(fields, text="CAN-FD  ·  1M / 1M", style="Muted.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="e", pady=(0, 9)
        )
        for column, (label, variable, values) in enumerate(
            (
                ("设备", self.can_index, tuple(map(str, range(8)))),
                ("通道", self.can_channel, ("0", "1")),
                ("节点", self.can_node, tuple(map(str, range(8)))),
            )
        ):
            fields.columnconfigure(column, weight=1)
            ttk.Label(fields, text=label, style="Muted.TLabel").grid(
                row=1, column=column, sticky="w", padx=4, pady=(0, 6)
            )
            choice = ttk.Combobox(
                fields, textvariable=variable, values=values, state="readonly", width=3
            )
            choice.grid(row=2, column=column, sticky="ew", padx=4)
            choice.bind("<<ComboboxSelected>>", lambda event: self.clear_can())
            self.controls.append((choice, "readonly"))

    def build_write_pages(self):
        """两种写入沿用原回调；以文件、目标、执行三步呈现。"""
        for is_can, title, heading, subtitle in (
            (
                False,
                "初装烧录",
                "初装烧录",
                "通过 J-Link 安装 APP + Loader，保留现有标定参数。",
            ),
            (
                True,
                "CAN 固件升级",
                "CAN 固件升级",
                "通过 CAN 更新 APP，自动核对设备身份与参数 CRC。",
            ),
        ):
            page = self.page(title, heading, subtitle)
            page.columnconfigure(0, weight=1)
            border, content = self.card(page, 22)
            border.grid(row=0, column=0, sticky="ew")
            content.columnconfigure(0, weight=1)
            top = ttk.Frame(content, style="Card.TFrame")
            top.grid(row=0, column=0, sticky="ew", pady=(0, 20))
            ttk.Label(
                top, text="升级准备" if is_can else "烧录准备", style="CardTitle.TLabel"
            ).pack(side="left")
            tk.Label(
                top,
                text="参数区保留",
                background="#eaf8f2",
                foreground="#16816a",
                font=("Microsoft YaHei UI", 9),
                padx=9,
                pady=5,
            ).pack(side="right")
            self.file_row(
                content,
                1,
                "01   选择固件",
                self.upgrade_path if is_can else self.hex_path,
                self.choose_upgrade if is_can else self.choose_hex,
            )
            ttk.Label(
                content,
                textvariable=self.upgrade_info if is_can else self.info,
                style="Muted.TLabel",
            ).grid(row=2, column=0, sticky="w", pady=(8, 20))
            ttk.Separator(content).grid(row=3, column=0, sticky="ew", pady=(0, 18))
            target = ttk.Frame(content, style="Card.TFrame")
            target.grid(row=4, column=0, sticky="ew")
            ttk.Label(target, text="02   识别设备", style="CardTitle.TLabel").pack(
                side="left"
            )
            self.button(
                target,
                "读取 CAN 设备" if is_can else "读取目标",
                self.read_can if is_can else self.read_target,
            ).pack(side="right")
            ttk.Label(
                content,
                textvariable=self.can_info if is_can else self.target_info,
                style="Muted.TLabel",
                wraplength=620,
            ).grid(row=5, column=0, sticky="w", pady=(10, 20))
            ttk.Separator(content).grid(row=6, column=0, sticky="ew", pady=(0, 18))
            action = ttk.Frame(content, style="Card.TFrame")
            action.grid(row=7, column=0, sticky="ew")
            if not is_can:
                start = ttk.Checkbutton(
                    action, text="校验成功后启动 MCU", variable=self.run_after
                )
                start.pack(side="left")
                self.controls.append((start, "normal"))
            else:
                ttk.Label(
                    action, text="升级前请保持电机停机", style="Muted.TLabel"
                ).pack(side="left")
            button = ttk.Button(
                action,
                text="开始升级  →" if is_can else "开始烧录  →",
                command=self.upgrade if is_can else self.flash,
                state="disabled",
                style="Accent.TButton",
            )
            button.pack(side="right")
            if is_can:
                self.upgrade_button = button
            else:
                self.flash_button = button
            note = ttk.Frame(page, padding=(3, 15), style="Workspace.TFrame")
            note.grid(row=1, column=0, sticky="ew")
            ttk.Label(
                note,
                text="文件与目标校验通过后，开始按钮才会启用。",
                style="Breadcrumb.TLabel",
            ).pack(side="left")
            ttk.Label(
                note,
                text="CAN-FD 1M / 1M" if is_can else "完整备份  ·  回读验证",
                style="Breadcrumb.TLabel",
            ).pack(side="right")

    def build_read_pages(self):
        """状态表与标定表独立展示，查询和数据操作沿用原方法。"""
        for calibration, title, subtitle in (
            (False, "版本与状态", "按需查询固件版本和设备状态，不改变电机运行模式。"),
            (True, "标定参数", "单独查看已保存的电机标定参数和编码器线性化曲线。"),
        ):
            page = self.page(title, title, subtitle)
            page.columnconfigure(0, weight=1)
            page.rowconfigure(0, weight=1)
            border, content = self.card(page, 18)
            border.grid(row=0, column=0, sticky="nsew")
            content.columnconfigure(0, weight=1)
            content.rowconfigure(2, weight=1)
            actions = ttk.Frame(content, style="Card.TFrame")
            actions.grid(row=0, column=0, sticky="ew", pady=(0, 12))
            if calibration:
                ttk.Label(actions, text="标定快照", style="CardTitle.TLabel").pack(
                    side="left", padx=(0, 15)
                )
                group = ttk.Combobox(
                    actions,
                    textvariable=self.calibration_group,
                    values=(
                        "全部",
                        "编码器",
                        "电流采样",
                        "摩擦模型",
                        "电机",
                        "限制",
                        "身份 / 范围",
                        "控制参数",
                        "存储",
                    ),
                    state="readonly",
                    width=11,
                )
                group.pack(side="left")
                group.bind(
                    "<<ComboboxSelected>>", lambda event: self.show_calibration()
                )
                self.controls.append((group, "readonly"))
                self.button(actions, "LUT 明细", self.show_lut).pack(
                    side="left", padx=10
                )
                button = self.button(actions, "读取标定参数", self.read_calibration)
            else:
                ttk.Label(actions, text="查询通道", style="CardTitle.TLabel").pack(
                    side="left", padx=(0, 15)
                )
                for transport in ("J-Link", "CAN"):
                    choice = ttk.Radiobutton(
                        actions,
                        text=transport,
                        variable=self.query_transport,
                        value=transport,
                    )
                    choice.pack(side="left", padx=(0, 15))
                    self.controls.append((choice, "normal"))
                button = self.button(actions, "读取版本与状态", self.read_status)
            button.configure(style="Accent.TButton")
            button.pack(side="right")
            ttk.Label(
                content,
                textvariable=self.calibration_info
                if calibration
                else self.snapshot_info,
                style="Muted.TLabel",
            ).grid(row=1, column=0, sticky="w", pady=(0, 14))
            table = self.table(content)
            table.master.grid(row=2, column=0, sticky="nsew")
            if calibration:
                self.calibration_table = table
                self.curve = tk.Canvas(
                    content, height=104, background="#f7faff", highlightthickness=0
                )
                self.curve.grid(row=3, column=0, sticky="ew", pady=(14, 0))
                self.curve.bind("<Configure>", lambda event: self.draw_lut())
            else:
                self.state_table = table

    def build_footer(self, parent):
        """任务进度只在执行中展示，日志默认折叠以保留内容空间。"""
        self.footer = ttk.Frame(parent, style="Workspace.TFrame")
        self.footer.pack(side="bottom", before=self.tabs, fill="x", pady=(14, 0))
        tools = ttk.Frame(self.footer, style="Workspace.TFrame")
        tools.pack(fill="x")
        self.log_toggle = ttk.Button(
            tools,
            text="查看操作记录  ▾",
            command=self.toggle_logs,
            style="Link.TButton",
        )
        self.log_toggle.pack(side="left")
        self.record_button = ttk.Button(
            tools,
            text="打开记录目录",
            command=self.open_records,
            state="disabled",
            style="Link.TButton",
        )
        self.record_button.pack(side="right")
        self.progress = ttk.Progressbar(self.footer, mode="indeterminate")
        self.log_box = ttk.Frame(self.footer, style="Card.TFrame", padding=(10, 8))
        self.log = tk.Text(
            self.log_box,
            height=3,
            wrap="word",
            state="disabled",
            relief="flat",
            borderwidth=0,
            highlightthickness=0,
            background="white",
            foreground="#70819b",
            font=("Microsoft YaHei UI", 9),
        )
        self.log.pack(side="left", fill="x", expand=True)
        scroll = ttk.Scrollbar(self.log_box, orient="vertical", command=self.log.yview)
        scroll.pack(side="right", fill="y")
        self.log.configure(yscrollcommand=scroll.set)

    def toggle_logs(self):
        """展开历史日志不执行设备操作。"""
        self.logs_visible = not self.logs_visible
        if self.logs_visible:
            self.log_box.pack(fill="x", pady=(8, 0))
        else:
            self.log_box.pack_forget()
        self.log_toggle.configure(
            text="收起操作记录  ▴" if self.logs_visible else "查看操作记录  ▾"
        )

    def update_status_badge(self):
        """按任务状态使用柔和的就绪、运行和错误色。"""
        error = "失败" in self.status.get()
        foreground, background = (
            ("#c24146", "#fff0f0")
            if error
            else ("#a16207", "#fff6df")
            if self.busy
            else ("#047857", "#e7f7ef")
        )
        self.status_badge.configure(foreground=foreground, background=background)

    def navigation_changed(self, event=None):
        """同步侧边选中态与页标题，不访问硬件。"""
        index = self.tabs.index(self.tabs.select())
        heading, subtitle = self.page_titles[index]
        self.page_heading.set(heading)
        self.page_subtitle.set(subtitle)
        for i, button in enumerate(self.nav_buttons):
            button.configure(
                background="#243858" if i == index else "#111d35",
                foreground="white" if i == index else "#a3b0c6",
            )

    def button(self, parent, text, command):
        """创建参与全局操作互斥的按钮。"""
        button = ttk.Button(parent, text=text, command=command)
        self.controls.append((button, "normal"))
        return button

    def page(self, title: str, heading: str, subtitle: str):
        """页面由侧边导航选择；保持Notebook原有选择与索引语义。"""
        page = ttk.Frame(self.tabs, style="Workspace.TFrame")
        self.tabs.add(page, text=title)
        self.page_titles.append((heading, subtitle))
        return page

    def file_row(self, parent, row: int, label: str, variable, command):
        """文件输入使用独立小节，路径只通过选择按钮修改。"""
        frame = ttk.Frame(parent, style="Card.TFrame")
        frame.grid(row=row, column=0, sticky="ew")
        frame.columnconfigure(0, weight=1)
        ttk.Label(frame, text=label, style="CardTitle.TLabel").grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 10)
        )
        ttk.Entry(frame, textvariable=variable, state="readonly").grid(
            row=1, column=0, sticky="ew", padx=(0, 12)
        )
        self.button(frame, "选择固件", command).grid(row=1, column=1)

    def table(self, parent):
        """带滚动条的分组参数表；长值可横向滚动。"""
        frame = ttk.Frame(parent, style="Card.TFrame")
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        tree = ttk.Treeview(
            frame,
            columns=("group", "field", "value", "unit"),
            show="headings",
            height=6,
        )
        for key, label, width in (
            ("group", "分类", 110),
            ("field", "参数", 185),
            ("value", "读取值", 380),
            ("unit", "单位", 95),
        ):
            tree.heading(key, text=label)
            tree.column(key, width=width, minwidth=75, stretch=key == "value")
        tree.tag_configure("odd", background="#fafcff")
        tree.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        horizontal = ttk.Scrollbar(frame, orient="horizontal", command=tree.xview)
        horizontal.grid(row=1, column=0, sticky="ew")
        tree.configure(yscrollcommand=scroll.set, xscrollcommand=horizontal.set)
        return tree

    def populate(self, table, rows):
        """替换旧快照；有限数值保留足够的小数位。"""
        table.delete(*table.get_children())
        for index, row in enumerate(rows):
            values = tuple(
                format(value, ".7g") if isinstance(value, float) else str(value)
                for value in row
            )
            table.insert("", "end", values=values, tags=("odd",) if index % 2 else ())

    def output_dir(self, operation: str) -> Path:
        """每次操作使用独立目录，不覆盖旧证据。"""
        storage = (
            Path(os.environ.get("LOCALAPPDATA", str(Path.home())))
            / "VectorMiniFlasher"
            / "outputs"
        )
        stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")
        return storage / (stamp + "_" + operation)

    def clear_can(self):
        """连接选择变化后撤销CAN身份，旧状态表明确标为过期。"""
        self.can_binding = None
        self.can_info.set("尚未绑定CAN设备 · 1M / 1M")
        self.snapshot_info.set("连接已修改，请重新读取")
        self.update_flash_button()

    def can_settings(self) -> dict:
        """主线程捕获连接选择；设备序列号绑定在成功只读识别之后。"""
        if not self.can_dll.get() or not Path(self.can_dll.get()).is_file():
            raise ValueError("未找到内置USB-CAN库，请使用完整发布版EXE")
        return {
            "dll": self.can_dll.get(),
            "index": int(self.can_index.get()),
            "channel": int(self.can_channel.get()),
            "node": int(self.can_node.get()),
        }

    def read_can(self):
        """识别所选通道的目标；不扫描、不自动进入Loader。"""
        self.clear_can()
        try:
            settings = self.can_settings()
        except ValueError as exc:
            messagebox.showinfo("连接USB-CAN", str(exc))
            return
        self.last_out = self.output_dir("can_read")
        out = self.last_out

        def work():
            result = can_snapshot(settings)
            out.mkdir(parents=True, exist_ok=False)
            (out / "summary.json").write_text(
                json.dumps({"status": "PASS", "read_only": True, **result}, indent=2)
                + "\n",
                encoding="utf-8",
            )
            return result

        def done(report):
            self.can_binding = report
            self.can_info.set(
                f"{report['runtime']} · 节点 {report['node']} · UID {report['uid']}\n适配器 {report['adapter']['serial']} · 通道 {settings['channel']}"
            )
            self.populate(self.state_table, report["rows"])
            self.snapshot_info.set(
                "CAN读取完成 · "
                + datetime.now(timezone(timedelta(hours=8))).strftime("%H:%M:%S")
            )
            self.write_log("CAN设备已识别：" + report["uid"])

        self.start_job("正在读取CAN设备…", work, done)

    def choose_upgrade(self):
        """在访问CAN之前校验镜像及清单。"""
        path = filedialog.askopenfilename(filetypes=[("HEX或升级清单", "*.hex *.json")])
        if path:
            self.upgrade_firmware = None
            self.upgrade_path.set(path)
            try:
                _, manifest, digest = load_upgrade(Path(path))
                self.upgrade_firmware = {"manifest": manifest, "hash": digest}
                self.upgrade_info.set(
                    f"APP v{version_text(manifest['app_version'])} · {manifest['image_bytes']} B · 校验通过"
                )
            except Exception as exc:
                self.upgrade_info.set("升级文件无效")
                messagebox.showerror("升级文件无效", str(exc))
            self.update_flash_button()

    def upgrade(self):
        """仅升级此前读取的设备，后台只向主线程队列报告进度。"""
        if self.busy or not self.can_binding or not self.upgrade_firmware:
            return
        settings = self.can_settings()
        binding = self.can_binding
        path = Path(self.upgrade_path.get())
        digest = self.upgrade_firmware["hash"]
        self.last_out = self.output_dir("can_upgrade")
        out = self.last_out
        last_progress = -1

        def progress(percent, message):
            nonlocal last_progress
            if int(percent) > last_progress:
                last_progress = int(percent)
                self.events.put(("progress", percent, message))

        def done(report):
            self.write_log(
                f"CAN升级成功，耗时 {report['elapsed_seconds']:.2f} s；版本、UID、STOP和参数CRC通过。"
            )
            self.clear_can()
            self.status.set("升级成功")

        self.start_job(
            "正在执行CAN升级…",
            lambda: run_can_upgrade(settings, binding, path, digest, out, progress),
            done,
            determinate=True,
        )

    def read_flash(self, calibration: bool):
        """后台读取所选探针快照，同时更新状态与标定两个只读页。"""
        if not self.probe_serial.get():
            messagebox.showinfo("选择探针", "请先刷新并选择J-Link编号")
            return
        dll, serial, speed = (
            Path(self.dll_path.get()),
            int(self.probe_serial.get()),
            int(self.speed_khz.get()),
        )
        self.last_out = self.output_dir("flash_read")
        out = self.last_out

        def done(report):
            self.populate(self.state_table, report["rows"])
            self.calibration = report["calibration"]
            self.show_calibration()
            stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%H:%M:%S")
            self.snapshot_info.set("J-Link读取完成 · " + stamp)
            self.calibration_info.set(self.calibration["state"] + " · " + stamp)
            self.write_log("只读快照：UID " + report["uid"] + "，原CPU状态保持。")
            if calibration:
                self.tabs.select(3)

        self.start_job(
            "正在只读Flash快照…", lambda: flash_snapshot(dll, serial, speed, out), done
        )

    def read_status(self):
        """选择传输后仅执行查询，不为读取版本进入Loader。"""
        if self.query_transport.get() == "CAN":
            self.read_can()
        else:
            self.read_flash(False)

    def read_calibration(self):
        """读取完整持久化参数并保留二进制原始备份。"""
        self.read_flash(True)

    def show_calibration(self):
        """按分类展示标定字段，独立于版本/运行状态表。"""
        rows = self.calibration["rows"] if self.calibration else []
        group = self.calibration_group.get()
        self.populate(
            self.calibration_table,
            [row for row in rows if group == "全部" or row[0] == group],
        )
        self.draw_lut()

    def draw_lut(self):
        """用原生Canvas绘制完整1024点偏心修正，不引入绘图库。"""
        self.curve.delete("all")
        values = self.calibration.get("lut_deg", []) if self.calibration else []
        width, height = max(self.curve.winfo_width(), 300), 104
        if not values:
            self.curve.create_text(
                15, 25, text="编码器线性化曲线 · 尚未读取", fill="#64748b", anchor="w"
            )
            return
        low, high = min(values), max(values)
        span = max(high - low, 0.01)
        self.curve.create_text(
            12,
            12,
            text=f"线性化LUT修正 / °    最小 {low:.5f}    最大 {high:.5f}",
            fill="#475569",
            anchor="w",
        )
        self.curve.create_line(46, 30, 46, height - 22, fill="#cbd5e1")
        self.curve.create_line(46, height - 22, width - 18, height - 22, fill="#cbd5e1")
        points = []
        for index, value in enumerate(values):
            points.extend(
                (
                    46 + index / (len(values) - 1) * (width - 64),
                    height - 27 - (value - low) / span * (height - 61),
                )
            )
        self.curve.create_line(*points, fill="#2563eb", width=2)
        self.curve.create_text(46, height - 10, text="0°", fill="#64748b")
        self.curve.create_text(width - 25, height - 10, text="360°", fill="#64748b")

    def show_lut(self):
        """独立明细窗只显示已经读取的快照，不再次访问探针。"""
        if not self.calibration or not self.calibration.get("lut_deg"):
            messagebox.showinfo("LUT明细", "请先读取标定参数")
            return
        window = tk.Toplevel(self.root)
        window.title("编码器线性化 · 1024点明细")
        window.geometry("730x460")
        tree = self.table(window)
        tree.master.pack(fill="both", expand=True, padx=15, pady=15)
        self.populate(
            tree,
            [
                ("编码器LUT", f"点 {i} / 输入 {i * 360 / 1024:.5f}°", v, "°")
                for i, v in enumerate(self.calibration["lut_deg"])
            ],
        )

    def write_log(self, text: str):
        """仅主线程调用，显示操作及最终记录路径。"""
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def clear_target(self):
        """探针或DLL选择变更后撤销此前绑定。"""
        self.binding = None
        self.target_info.set("尚未绑定J-Link目标")
        self.snapshot_info.set("连接已修改，请重新读取")
        self.calibration_info.set("连接已修改，请重新读取")
        self.update_flash_button()

    def update_flash_button(self):
        """固件及目标绑定完整且空闲时才允许烧录。"""
        self.flash_button.configure(
            state="normal"
            if self.firmware and self.binding and not self.busy
            else "disabled"
        )
        self.upgrade_button.configure(
            state="normal"
            if self.can_binding and self.upgrade_firmware and not self.busy
            else "disabled"
        )

    def set_firmware(self, path: Path):
        """完整校验文件后更新固件快照；坏文件撤销旧选择。"""
        self.firmware = None
        self.hex_path.set(str(path))
        try:
            _, self.firmware = load_hex(path)
            app = self.firmware["app"]
            self.info.set(
                f"APP v{version_text(app['app_version'])} · {app['image_bytes']} B · HEX校验通过"
            )
            self.write_log("已验证固件：" + path.name)
        except Exception:
            self.info.set("固件校验失败，请选择当前布局的完整合并HEX")
            raise
        finally:
            self.update_flash_button()

    def choose_hex(self):
        """只打开文件，不访问硬件。"""
        path = filedialog.askopenfilename(filetypes=[("APP + Loader HEX", "*.hex")])
        if path:
            try:
                self.set_firmware(Path(path))
            except Exception as exc:
                messagebox.showerror("固件无效", str(exc))

    def start_job(self, label: str, work, done, determinate: bool = False):
        """后台任务不调用Tk；异常由主线程展示，烧录不自动重试。"""
        if self.busy:
            return
        self.busy = True
        for widget, _ in self.controls:
            widget.configure(state="disabled")
        self.record_button.configure(state="disabled")
        self.update_flash_button()
        self.status.set(label)
        self.write_log(label)
        self.progress.pack(fill="x", pady=(8, 0))
        self.progress.configure(
            mode="determinate" if determinate else "indeterminate", value=0
        )
        if not determinate:
            self.progress.start()

        def worker():
            try:
                self.events.put((done, work(), None))
            except Exception as exc:
                self.events.put((done, None, str(exc)))

        threading.Thread(target=worker, daemon=False).start()

    def poll(self):
        """在主线程处理有界进度队列与结果，不在工作线程调用Tk。"""
        for _ in range(20):
            try:
                done, value, error = self.events.get_nowait()
            except queue.Empty:
                break
            if done == "progress":
                self.progress["value"] = value
                self.status.set(error)
                continue
            self.busy = False
            self.progress.stop()
            self.progress.pack_forget()
            for widget, state in self.controls:
                widget.configure(state=state)
            self.record_button.configure(
                state="normal" if self.last_out else "disabled"
            )
            if error:
                self.clear_target()
                self.clear_can()
                self.status.set("操作失败")
                self.write_log("失败：" + error)
                messagebox.showerror("操作失败", error)
            else:
                self.status.set("就绪")
                try:
                    done(value)
                except Exception as exc:
                    self.clear_target()
                    self.clear_can()
                    self.status.set("显示失败，请查看记录")
                    self.write_log(str(exc))
            self.update_flash_button()
        self.root.after(100, self.poll)

    def refresh_probes(self):
        """枚举USB探针；多个探针时必须由用户明确选择。"""
        self.clear_target()
        dll = Path(self.dll_path.get())

        def done(serials):
            self.probes["values"] = serials
            self.probe_serial.set(str(serials[0]) if len(serials) == 1 else "")
            self.write_log(f"发现{len(serials)}个J-Link，请选择目标探针。")

        self.start_job(
            "正在枚举J-Link…",
            lambda: sorted(
                {
                    e.SerialNumber
                    for e in open_probe(dll).connected_emulators()
                    if e.SerialNumber > 0
                }
            ),
            done,
        )

    def read_target(self):
        """读取所选芯片的UID，不借用此前板子的UID。"""
        self.clear_target()
        if not self.probe_serial.get():
            messagebox.showinfo("选择探针", "请先刷新并选择J-Link编号")
            return
        dll, serial = Path(self.dll_path.get()), int(self.probe_serial.get())
        speed_khz = int(self.speed_khz.get())

        def done(binding):
            self.binding = binding
            self.target_info.set(
                f"UID：{binding['uid']}\nSTM32G431 · {binding['flash_kib']} KiB · 参考电压 {binding['reference_voltage_mv'] / 1000:.2f} V · 关相已确认"
            )
            self.write_log("目标芯片已绑定；烧录前会再次核对UID。")

        self.start_job(
            "正在读取目标芯片…", lambda: inspect_target(dll, serial, speed_khz), done
        )

    def flash(self):
        """点击后只烧录此前读取的目标；Flash函数重新校验HEX、UID和保护区。"""
        if not self.firmware or not self.binding or self.busy:
            return
        if str(self.binding["serial"]) != self.probe_serial.get():
            self.clear_target()
            return
        storage = (
            Path(os.environ.get("LOCALAPPDATA", str(Path.home())))
            / "VectorMiniFlasher"
            / "outputs"
        )
        stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S_%f")
        self.last_out = storage / (stamp + "_" + self.binding["uid"])
        args = SimpleNamespace(
            hex=Path(self.hex_path.get()),
            package=None,
            uid=self.binding["uid"],
            jlink_serial=self.binding["serial"],
            jlink_dll=Path(self.dll_path.get()),
            speed_khz=int(self.speed_khz.get()),
            out=self.last_out,
            pylink_dir=None,
            run=self.run_after.get(),
            expected_hex_sha256=self.firmware["hex_sha256"],
        )

        def done(report):
            self.write_log("烧录成功：完整回读通过，参数区及保留页未改变。")
            self.write_log("MCU已启动。" if report["cpu_running"] else "MCU保持暂停。")
            self.write_log("记录目录：" + str(self.last_out))
            self.status.set("烧录成功")
            self.clear_target()

        self.start_job(
            "正在备份、烧录并回读校验，请等待…", lambda: flash_package(args), done
        )

    def open_records(self):
        """按用户点击打开当前作业记录目录。"""
        if self.last_out and self.last_out.is_dir():
            os.startfile(self.last_out)

    def close(self):
        """禁止通过关闭窗口中断正在执行的Flash作业。"""
        if self.busy:
            messagebox.showinfo("操作进行中", "请等待当前操作结束后关闭窗口")
        else:
            self.root.destroy()


def main():
    """默认启动GUI；显式离线检查和UI冒烟模式均不调用探针。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-hex", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--smoke-ui", type=Path)
    parser.add_argument("--check-drivers", type=Path)
    args = parser.parse_args()
    if args.check_drivers:
        try:
            report = check_bundled_drivers()
        except Exception as exc:
            report = {"status": "FAIL", "hardware_accessed": False, "error": str(exc)}
        args.check_drivers.write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
        return 0 if report["status"] == "PASS" else 1
    if args.check_hex and not args.smoke_ui:
        result = {"status": "FAIL", "hardware_accessed": False}
        try:
            _, manifest = load_hex(args.check_hex)
            result.update(status="PASS", firmware=manifest)
        except Exception as exc:
            result["error"] = str(exc)
        if args.report:
            args.report.write_text(
                json.dumps(result, indent=2) + "\n", encoding="utf-8"
            )
        return 0 if result["status"] == "PASS" else 1
    root = tk.Tk()
    window = FlasherWindow(root)
    if args.check_hex:
        window.set_firmware(args.check_hex)
    if args.smoke_ui:

        def finish():
            args.smoke_ui.write_text(
                json.dumps(
                    {
                        "status": "PASS",
                        "hardware_accessed": False,
                        "window_title": root.title(),
                        "firmware": window.firmware,
                        "pages": [
                            window.tabs.tab(tab, "text") for tab in window.tabs.tabs()
                        ],
                        "upgrade_button_disabled": str(window.upgrade_button["state"])
                        == "disabled",
                        "flash_button_disabled": str(window.flash_button["state"])
                        == "disabled",
                    },
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )
            root.destroy()

        root.after(1000, finish)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
