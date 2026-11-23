"""Windows 桌面入口。测试线程独占 USB-CAN，界面线程负责显示结果。"""

import os
import queue
import sys
import threading
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

import can_link_test as bench
from _can_raw_send import parse_frame, send_and_capture


class CanTestWindow:
    def __init__(self, root):
        self.root = root
        root.title(f"CAN FD 通信测试 v{bench.TOOL_VERSION}")
        root.minsize(690, 435)
        frame = ttk.Frame(root, padding=18)
        frame.pack(fill="both", expand=True)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(5, weight=1)
        ttk.Label(frame, text="CAN FD 通信测试", font=("Microsoft YaHei UI", 17, "bold")).grid(
            row=0, column=0, sticky="w"
        )
        connection = ttk.Frame(frame)
        connection.grid(row=1, column=0, sticky="ew", pady=(2, 12))
        ttk.Label(connection, text="USB-CAN 通道").pack(side="left")
        self.channel = tk.StringVar(value="1")
        channel_box = ttk.Combobox(
            connection, state="readonly", values=("1", "2"), textvariable=self.channel, width=3
        )
        channel_box.pack(side="left", padx=(4, 14))
        self.controls = [channel_box]
        self.arb_rate = tk.StringVar(value="1M")
        self.data_rate = tk.StringVar(value="5M")
        for label, variable, rates in [
            ("仲裁段", self.arb_rate, bench.ARB_RATES),
            ("数据段", self.data_rate, bench.DATA_RATES),
        ]:
            ttk.Label(connection, text=label).pack(side="left")
            box = ttk.Combobox(
                connection, state="readonly", values=tuple(rates), textvariable=variable, width=6
            )
            box.pack(side="left", padx=(4, 14))
            self.controls.append(box)
        ttk.Label(connection, text="ISO FD + BRS").pack(side="left")
        self.tabs = ttk.Notebook(frame)
        self.tabs.grid(row=2, column=0, sticky="ew", pady=(0, 8))
        query_page = ttk.Frame(self.tabs, padding=8)
        self.raw_page = ttk.Frame(self.tabs, padding=8)
        self.tabs.add(query_page, text="通信压测")
        self.tabs.add(self.raw_page, text="协议发送")
        nodes = ttk.LabelFrame(query_page, text="查询节点", padding=8)
        nodes.grid(row=2, column=0, sticky="ew")
        self.selected = {}
        for node, name in bench.AXES.items():
            selected = tk.BooleanVar(value=True)
            self.selected[node] = selected
            check = ttk.Checkbutton(nodes, text=f"{name}  0x{node:02X}", variable=selected)
            check.pack(side="left", padx=6)
            self.controls.append(check)
        settings = ttk.Frame(query_page)
        settings.grid(row=3, column=0, sticky="ew", pady=12)
        self.mode = tk.StringVar(value="常规查询")
        ttk.Label(settings, text="测试模式").pack(side="left")
        box = ttk.Combobox(
            settings,
            state="readonly",
            values=("常规查询", "阶梯压测"),
            textvariable=self.mode,
            width=9,
        )
        box.pack(side="left", padx=(4, 14))
        self.controls.append(box)
        self.count = tk.StringVar(value="1000")
        self.rate = tk.StringVar(value="50")
        self.entries = []
        for label, var in [("每节点笔数", self.count), ("总笔/秒", self.rate)]:
            ttk.Label(settings, text=label).pack(side="left")
            field = ttk.Entry(settings, width=7, textvariable=var)
            field.pack(side="left", padx=(4, 10))
            self.entries.append(field)
        self.mode.trace_add("write", lambda *_: self.update_controls())
        self.id_text = tk.StringVar(value="0x0CEF0602")
        self.data_text = tk.StringVar()
        self.raw_page.columnconfigure(1, weight=1)
        for row, (label, variable) in enumerate(
            [("CAN ID（HEX）", self.id_text), ("原始数据（HEX）", self.data_text)]
        ):
            ttk.Label(self.raw_page, text=label).grid(row=row, column=0, sticky="w", pady=3)
            entry = ttk.Entry(self.raw_page, textvariable=variable, width=64)
            entry.grid(row=row, column=1, sticky="ew", padx=(8, 0))
            self.controls.append(entry)
        ttk.Label(
            self.raw_page,
            text="标准 ID 可填 123；同一 ID 发扩展帧请填 00000123。数据示例：01 02 FF，留空发送 0 字节。",
            wraplength=640,
        ).grid(row=2, column=0, columnspan=2, sticky="w", pady=(5, 0))
        examples = ttk.LabelFrame(self.raw_page, text="状态查询示例 · Type 108 · 序号 1", padding=6)
        examples.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        for node, name in bench.AXES.items():
            button = ttk.Button(
                examples, text=f"{name} 0x{node:02X}", command=lambda n=node: self.fill_query(n)
            )
            button.pack(side="left", padx=(0, 5))
            self.controls.append(button)
        actions = ttk.Frame(frame)
        actions.grid(row=4, column=0, sticky="ew", pady=(0, 8))
        self.start_button = ttk.Button(actions, text="开始测试", command=self.start)
        self.stop_button = ttk.Button(actions, text="停止", command=self.stop)
        self.folder_button = ttk.Button(actions, text="打开本次记录", command=self.open_report)
        for button in [self.start_button, self.stop_button, self.folder_button]:
            button.pack(side="left", padx=(0, 7))
        self.status = tk.StringVar(value="就绪")
        ttk.Label(actions, textvariable=self.status).pack(side="right")
        progress_row = ttk.Frame(frame)
        progress_row.grid(row=3, column=0, sticky="ew", pady=(0, 8))
        self.progress = ttk.Progressbar(progress_row, maximum=100, mode="determinate")
        self.progress.pack(side="left", fill="x", expand=True)
        self.progress_text = tk.StringVar(value="0%")
        ttk.Label(progress_row, textvariable=self.progress_text).pack(side="left", padx=(8, 0))
        self.log = tk.Text(frame, width=82, height=12, wrap="word", state="disabled")
        self.log.grid(row=5, column=0, sticky="nsew")
        self.mail = queue.Queue()
        self.cancel = None
        self.directory = None
        self.closing = False
        self.tabs.bind("<<NotebookTabChanged>>", lambda *_: self.update_controls())
        self.update_controls()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self.poll)

    def fill_query(self, node):
        """把与常规查询相同的只读状态帧填入两个可编辑输入框。"""
        ((identifier, data),) = bench.encode_frames(node, 108, b"", 1)
        self.id_text.set(f"0x{identifier:08X}")
        self.data_text.set(data.hex(" ").upper())

    def update_controls(self):
        busy = self.cancel is not None
        raw = self.tabs.select() == str(self.raw_page)
        self.start_button.configure(text="发送一帧" if raw else "开始测试")
        for tab in self.tabs.tabs():
            self.tabs.tab(tab, state="disabled" if busy and tab != self.tabs.select() else "normal")
        for widget in self.controls + [self.start_button]:
            widget.state(["disabled" if busy else "!disabled"])
        for entry in self.entries:
            entry.state(["disabled" if busy or self.mode.get() == "阶梯压测" else "!disabled"])
        self.stop_button.state(["!disabled" if busy else "disabled"])
        self.folder_button.state(
            ["!disabled" if self.directory and self.directory.is_dir() else "disabled"]
        )

    def start(self):
        if self.tabs.select() == str(self.raw_page):
            try:
                frame = parse_frame(self.id_text.get(), self.data_text.get())
            except ValueError as exc:
                messagebox.showerror("输入错误", str(exc), parent=self.root)
                return
            self.launch(
                self.raw_worker,
                (
                    int(self.channel.get()),
                    frame,
                    bench.ARB_RATES[self.arb_rate.get()],
                    bench.DATA_RATES[self.data_rate.get()],
                ),
                "正在发送",
            )
            return
        nodes = [hex(n) for n, selected in self.selected.items() if selected.get()]
        if not nodes:
            messagebox.showinfo("选择节点", "请至少选择一个查询节点。", parent=self.root)
            return
        args = [
            *nodes,
            "--channel",
            self.channel.get(),
            "--arb-rate",
            self.arb_rate.get(),
            "--data-rate",
            self.data_rate.get(),
        ]
        if self.mode.get() == "阶梯压测":
            args.append("--stress")
        else:
            try:
                count, rate = int(self.count.get()), int(self.rate.get())
                if count < 1 or count * len(nodes) > 65535 or rate < 1:
                    raise ValueError
            except ValueError:
                messagebox.showerror(
                    "参数错误", "笔数与速率须为正整数，总笔数最多 65535。", parent=self.root
                )
                return
            args.extend(["--count", str(count), "--rate", str(rate)])
        if getattr(sys, "frozen", False):
            args.extend(["--dll", str(Path(sys._MEIPASS) / "ControlCANFD.dll")])
        self.launch(self.worker, (args,), "测试中")

    def launch(self, worker, inputs, status):
        self.directory = Path.home() / "CAN-Link-Test" / datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        self.cancel = threading.Event()
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")
        self.status.set(status)
        self.progress.configure(value=0)
        self.progress_text.set("0% · 连接通道")
        self.update_controls()
        threading.Thread(target=worker, args=(*inputs, self.directory, self.cancel)).start()

    def raw_worker(self, channel, frame, arb_bps, data_bps, directory, stop):
        if getattr(sys, "frozen", False):
            dll = Path(sys._MEIPASS) / "ControlCANFD.dll"
        else:
            sdk_root = Path(
                os.environ.get("MOTOR_CTRL_APP_ROOT", bench.ROOT.parent / "motor_ctrl_app")
            )
            dll = (sdk_root / ".local/canfd-sdk/ControlCANFD.dll").resolve()
        try:
            status = send_and_capture(
                dll,
                channel,
                frame,
                directory,
                self.mail.put,
                stop,
                arb_bps=arb_bps,
                data_bps=data_bps,
                progress=self.post_progress,
            )
        except Exception as exc:
            self.mail.put(f"运行失败  {exc}")
            status = "未完成"
        self.mail.put(("done", status, status.startswith("已结束")))

    def post_progress(self, fraction, label):
        self.mail.put(("progress", fraction, label))

    def worker(self, args, directory, stop):
        try:
            code = bench.main(
                args,
                output_dir=directory,
                log=self.mail.put,
                stop=stop,
                progress=self.post_progress,
            )
            status = ("全部有效应答", "存在拒收或缺少应答", "未完成")[code]
        except (Exception, SystemExit) as exc:
            self.mail.put(f"运行失败  {exc}")
            status = "未完成"
            code = 2
        self.mail.put(("done", status, code in (0, 1)))

    def poll(self):
        while not self.mail.empty():
            msg = self.mail.get_nowait()
            if isinstance(msg, tuple):
                if msg[0] == "progress":
                    percent = max(0, min(99, int(100 * msg[1])))
                    self.progress.configure(value=percent)
                    self.progress_text.set(f"{percent}% · {msg[2]}")
                    continue
                _, status, completed = msg
                self.status.set(status)
                if completed:
                    self.progress.configure(value=100)
                percent = int(self.progress["value"])
                self.progress_text.set(f"{percent}% · {'完成' if completed else status}")
                self.cancel = None
                self.update_controls()
                if self.closing:
                    self.root.destroy()
                    return
            else:
                self.log.configure(state="normal")
                self.log.insert("end", msg + "\n")
                self.log.see("end")
                self.log.configure(state="disabled")
        self.root.after(100, self.poll)

    def open_report(self):
        if self.directory:
            os.startfile(self.directory)

    def stop(self):
        if self.cancel is not None:
            self.cancel.set()
            self.status.set("正在保存")
            self.stop_button.state(["disabled"])

    def close(self):
        if self.cancel is None:
            self.root.destroy()
        else:
            self.closing = True
            self.stop()


if __name__ == "__main__":
    root = tk.Tk()
    CanTestWindow(root)
    root.mainloop()
