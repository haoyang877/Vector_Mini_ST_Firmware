"""参数页只维护当前 UID 的读回快照与编辑值，协议和设备访问交给工作进程。"""

import tkinter as tk
from collections.abc import Callable
from tkinter import ttk

from can_parameters import CATALOG, GROUPS, encode_value, value_text


class ParameterPage(ttk.Frame):
    """四组参数平铺显示，保存成功以 Flash 回读为准。"""

    def __init__(self, master: tk.Misc, action: Callable[[str, bool], None]) -> None:
        super().__init__(master)
        self.snapshots: dict[str, dict] = {}
        self.entries: dict[int, ttk.Entry] = {}
        self.buttons: list[tuple[ttk.Button, str, bool]] = []
        self.identity = None
        self.busy = False
        self.available = False
        read_all = ttk.Button(self, text="读取全部参数", command=lambda: action("all", False))
        read_all.pack(anchor="w", pady=(0, 5))
        self.buttons.append((read_all, "all", False))
        self.message = tk.StringVar(value="先识别当前电机，再读取需要调整的参数组。")
        ttk.Label(self, textvariable=self.message).pack(anchor="w")
        ttk.Label(
            self, text="只读项由固件或标定生成。写入并保存会保存整轴活动配置；不启动电机。"
        ).pack(anchor="w", pady=(2, 6))
        scroll = ttk.Frame(self)
        scroll.pack(fill="both", expand=True)
        canvas = tk.Canvas(scroll, height=410, highlightthickness=0)
        bar = ttk.Scrollbar(scroll, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        body = ttk.Frame(canvas)
        window = canvas.create_window(0, 0, anchor="nw", window=body)
        body.bind("<Configure>", lambda _: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window, width=event.width))
        for col in range(2):
            body.columnconfigure(col, weight=1)
        for index, (group, (title, fields)) in enumerate(GROUPS.items()):
            panel = ttk.LabelFrame(body, text=title, padding=8)
            panel.grid(row=index // 2, column=index % 2, sticky="nsew", padx=4, pady=4)
            panel.columnconfigure(1, weight=1)
            actions = ttk.Frame(panel)
            actions.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 5))
            for save, label in ((False, "读取"), (True, "写入并保存")):
                button = ttk.Button(
                    actions, text=label, command=lambda g=group, s=save: action(g, s)
                )
                button.pack(side="left", padx=(0, 6))
                self.buttons.append((button, group, save))
            for row, (identifier, label) in enumerate(fields, 1):
                spec = CATALOG[identifier]
                readonly = not spec["access"] & 2
                ttk.Label(panel, text=label + ("（只读）" if readonly else "")).grid(
                    row=row, column=0, sticky="w", pady=2
                )
                entry = self.entries[identifier] = ttk.Entry(panel, width=16, state="readonly")
                entry.grid(row=row, column=1, sticky="ew", padx=8, pady=2)
                unit = {"pole_pairs": "对", "encoder_q15_counts": "Q15", "bool": "0 / 1"}.get(
                    spec["unit"], spec["unit"]
                )
                ttk.Label(panel, text=unit).grid(row=row, column=2, sticky="w")

    def reset(self, identity: tuple | None) -> None:
        if identity == self.identity:
            return
        self.identity = identity
        self.snapshots.clear()
        for entry in self.entries.values():
            entry.configure(state="normal")
            entry.delete(0, "end")
            entry.configure(state="readonly")
        self.message.set("先读取当前电机参数；不同电机的编辑值不沿用。")

    def update_controls(self, busy: bool, available: bool) -> None:
        self.busy, self.available = busy, available
        for button, group, save in self.buttons:
            allowed = not busy and available and (not save or group in self.snapshots)
            button.configure(state="normal" if allowed else "disabled")
        for group, (_, fields) in GROUPS.items():
            for identifier, _ in fields:
                editable = (
                    not busy
                    and available
                    and group in self.snapshots
                    and CATALOG[identifier]["access"] & 2
                    and self.snapshots[group]["rows"][identifier]["access"] & 2
                )
                self.entries[identifier].configure(state="normal" if editable else "readonly")

    def request(self, group: str, save: bool) -> dict:
        """提交编辑组与读取快照；只发送真正变化的可写字段。"""
        settings = dict(group=group, changes=None, snapshot=None)
        if save:
            snapshot = self.snapshots.get(group)
            if snapshot is None:
                raise ValueError("请先读取本组参数")
            changes = {}
            for identifier, _ in GROUPS[group][1]:
                if CATALOG[identifier]["access"] & 2:
                    raw = encode_value(identifier, self.entries[identifier].get())
                    if raw != snapshot["rows"][identifier]["raw"]:
                        changes[identifier] = raw
            settings.update(changes=changes, snapshot=snapshot)
        return settings

    def display(self, report: dict) -> None:
        """保存后其他组修订缓存失效；失败保留编辑文本但要求重新读取。"""
        if report.get("group") == "all":
            if not report.get("passed"):
                self.snapshots.clear()
                self.message.set(report.get("fatal", "全部读取失败") + "；请重新读取。")
                self.update_controls(self.busy, self.available)
                return
            rows = {int(key): value for key, value in report["rows"].items()}
            for group, (_, fields) in GROUPS.items():
                self.display(dict(report, group=group, rows={i: rows[i] for i, _ in fields}))
            self.message.set(
                f"已读取全部 {len(rows)} 项参数 · 活动修订 {report['revision']} · 未保存编辑保留"
            )
            return
        drafts = {}
        old = self.snapshots.get(report.get("group"))
        if report.get("action") == "read" and old:
            for identifier, row in old["rows"].items():
                text = self.entries[identifier].get()
                if CATALOG[identifier]["access"] & 2 and text != value_text(identifier, row["raw"]):
                    drafts[identifier] = text
        if report.get("action") == "save" or "group" not in report:
            self.snapshots.clear()
        if report.get("passed"):
            group = report["group"]
            rows = {int(key): value for key, value in report["rows"].items()}
            self.snapshots[group] = dict(
                node=report["node"], uid=report["uid"], revision=report["revision"], rows=rows
            )
            for identifier, row in rows.items():
                entry = self.entries[identifier]
                entry.configure(state="normal")
                entry.delete(0, "end")
                entry.insert(0, drafts.get(identifier, value_text(identifier, row["raw"])))
            action = "已保存并回读验证" if report["action"] == "save" else "已读取"
            self.message.set(
                f"{GROUPS[group][0]}{action} · 活动修订 {report['revision']}"
                + (" · 已保留未保存编辑" if drafts else "")
            )
        else:
            self.snapshots.pop(report.get("group"), None)
            self.message.set(report.get("fatal", "参数操作失败") + "；请重新读取。")
        self.update_controls(self.busy, self.available)
