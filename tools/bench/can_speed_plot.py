"""速度页实时曲线：只显示有效反馈，有界缓存，不接触通信或电机控制。"""

import math
import tkinter as tk
from collections import deque
from tkinter import ttk

CHANNELS = (
    ("位置 / rad", "reference_position_rad", "position_rad"),
    ("速度 / rad/s", "reference_speed_rad_s", "actual_rad_s"),
    ("Iq / A", "reference_iq_a", "iq_a"),
)


class SpeedPlot(ttk.Frame):
    """GUI线程持有；最多1200点、30秒窗口，重绘不超过10Hz。"""

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(master)
        self.samples = deque(maxlen=1200)
        self.pending = None
        self.canvas = tk.Canvas(
            self,
            height=280,
            background="#ffffff",
            highlightthickness=1,
            highlightbackground="#cbd5e1",
        )
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _: self.schedule())
        self.bind("<Destroy>", self.close)

    def close(self, event: tk.Event) -> None:
        if event.widget is self and self.pending is not None:
            self.after_cancel(self.pending)
            self.pending = None

    def clear(self) -> None:
        self.samples.clear()
        self.schedule()

    def append(self, sample: dict) -> None:
        stamp = sample.get("elapsed_s")
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
            return
        if self.samples and stamp < self.samples[-1]["elapsed_s"]:
            self.samples.clear()
        self.samples.append(dict(sample))
        while self.samples and stamp - self.samples[0]["elapsed_s"] > 30:
            self.samples.popleft()
        self.schedule()

    def schedule(self) -> None:
        if self.pending is None:
            self.pending = self.after(100, self.redraw)

    def _draw_segment(
        self, segment: list[float], color: str, dash: tuple[int, ...], tags: tuple[str, str]
    ) -> None:
        if len(segment) >= 4:
            self.canvas.create_line(*segment, fill=color, dash=dash, width=1.6, tags=tags)
        elif segment:
            x, y = segment
            self.canvas.create_oval(
                x - 2, y - 2, x + 2, y + 2, fill=color, outline=color, tags=tags
            )

    def redraw(self) -> None:
        self.pending = None
        canvas = self.canvas
        canvas.delete("all")
        width, height = max(canvas.winfo_width(), 300), max(canvas.winfo_height(), 240)
        left, right, top = 124, width - 18, 40
        row_height = (height - top - 22) / 3
        end = max(30.0, self.samples[-1]["elapsed_s"] if self.samples else 30.0)
        begin = end - 30
        canvas.create_text(
            12,
            13,
            anchor="w",
            text="最近30秒 · 蓝色虚线：参考 · 橙色实线：反馈",
            fill="#334155",
            tags="legend",
        )
        for index, (title, reference, feedback) in enumerate(CHANNELS):
            y0, y1 = top + index * row_height, top + (index + 1) * row_height - 18
            values = [
                point[key]
                for point in self.samples
                for key in (reference, feedback)
                if point.get(key) is not None and math.isfinite(point[key])
            ]
            low, high = (min(values), max(values)) if values else (-1.0, 1.0)
            margin = max((high - low) * 0.1, max(abs(low), abs(high)) * 0.02, 0.01)
            low, high = low - margin, high + margin
            canvas.create_text(8, (y0 + y1) / 2, anchor="w", text=title, fill="#334155")
            for fraction in (0, 0.5, 1):
                y = y1 - fraction * (y1 - y0)
                canvas.create_line(left, y, right, y, fill="#e2e8f0")
                canvas.create_text(
                    left - 6,
                    y,
                    anchor="e",
                    text=f"{low + fraction * (high - low):.3g}",
                    fill="#64748b",
                    font=("Segoe UI", 8),
                )
            for key, color, dash, tag in (
                (reference, "#2563eb", (5, 3), "reference"),
                (feedback, "#ea580c", (), "feedback"),
            ):
                segment = []

                for point in self.samples:
                    value = point.get(key)
                    if value is None or not math.isfinite(value):
                        self._draw_segment(segment, color, dash, (tag, key))
                        segment = []
                        continue
                    segment.extend(
                        (
                            left + (point["elapsed_s"] - begin) / 30 * (right - left),
                            y1 - (value - low) / (high - low) * (y1 - y0),
                        )
                    )
                self._draw_segment(segment, color, dash, (tag, key))
                if self.samples and self.samples[-1].get(key) is None:
                    canvas.create_text(
                        right - 4,
                        y0 - 8 if tag == "reference" else y1 + 9,
                        anchor="e",
                        text=("参考" if tag == "reference" else "反馈") + "：无有效数据",
                        fill=color,
                        font=("Segoe UI", 8),
                    )
        canvas.create_text(left, height - 8, text=f"{begin:.1f}s", anchor="w", fill="#64748b")
        canvas.create_text(right, height - 8, text=f"{end:.1f}s", anchor="e", fill="#64748b")
        if not self.samples:
            canvas.create_text(
                (left + right) / 2,
                height / 2,
                text="使能速度控制后显示实时反馈",
                fill="#64748b",
                tags="empty",
            )
