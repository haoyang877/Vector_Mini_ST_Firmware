"""将生产高速试验的原始遥测归档为两张共享时间轴的 PNG。"""

import math
from bisect import bisect_left, bisect_right
from itertools import groupby
from pathlib import Path
from statistics import fmean
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from matplotlib.axes import Axes
    from matplotlib.figure import Figure


POWER_AVERAGE_S = 0.5


def _power_average(times: list[float], values: list[float]) -> list[float]:
    """按实际时间居中取样，缺点和通信空档两侧独立滤波。"""
    half_window = POWER_AVERAGE_S / 2
    filtered = []
    for valid, rows in groupby(
        zip(times, values, strict=True), key=lambda row: math.isfinite(row[1])
    ):
        if not valid:
            filtered.extend(math.nan for _ in rows)
            continue
        segment_times, segment_values = zip(*rows, strict=True)
        for time in segment_times:
            left = bisect_left(segment_times, time - half_window)
            right = bisect_right(segment_times, time + half_window)
            filtered.append(fmean(segment_values[left:right]))
    return filtered


def telemetry_series(samples: list[dict], field: str, max_gap_s: float) -> tuple[list, list]:
    """在缺样处断开曲线，也让功率滤波在空档两侧分开计算。"""
    times, values = [], []
    for sample in samples:
        elapsed, value = sample["elapsed_s"], sample.get(field)
        if times and elapsed - times[-1] > max_gap_s:
            times.append(elapsed)
            values.append(math.nan)
        times.append(elapsed)
        values.append(value if value is not None and math.isfinite(value) else math.nan)
    return times, values


def save_high_speed_curves(
    samples: list[dict],
    out: Path,
    config: dict,
    *,
    window: tuple[float, float] | None,
    max_gap_s: float,
    speed_error_fraction: float,
) -> dict:
    specs = (
        ("speed", "output_rpm", "output-speed.png"),
        ("power", "bus_input_power_w", "input-power.png"),
    )
    curves, errors = {}, []
    for key, field, filename in specs:
        times, values = telemetry_series(samples, field, max_gap_s)
        if not any(math.isfinite(value) for value in values):
            continue
        path = out / filename
        try:
            from matplotlib.backends.backend_agg import FigureCanvasAgg
            from matplotlib.figure import Figure

            if key == "speed":
                figure = Figure(
                    figsize=(8.8, 5.8 if window else 3.8), dpi=115, layout="constrained"
                )
                _draw_speed(
                    figure, times, values, window, config["output_rpm"], speed_error_fraction
                )
            else:
                figure = Figure(figsize=(8.8, 3.8), dpi=115, layout="constrained")
                draw_power_axis(
                    figure.subplots(),
                    times,
                    values,
                    window,
                    config["power_min_w"],
                    config["power_max_w"],
                )
                figure.legend(loc="outside upper right", ncols=2, frameon=False)
            FigureCanvasAgg(figure).print_png(path)
        except (ImportError, OSError, RuntimeError, ValueError) as error:
            errors.append(f"{filename}: {error}")
        else:
            curves[key] = path.relative_to(out.parent).as_posix()
    result = {"curves": curves} if curves else {}
    if errors:
        result["curve_error"] = "高速曲线保存失败 · " + " · ".join(errors)
    return result


def _full_run_axis(
    chart: "Axes", times: list[float], window: tuple[float, float] | None, unit: str
) -> None:
    end = 0
    if window:
        start, end = window
        chart.axvspan(start, end, color="#dcecf8")
    chart.set(xlim=(0, max(end, times[-1])), xlabel="Elapsed time (s)", ylabel=unit)
    chart.grid(alpha=0.2)


def _draw_speed(
    figure: "Figure",
    times: list[float],
    values: list[float],
    window: tuple[float, float] | None,
    target_rpm: float,
    tolerance: float,
) -> None:
    if window:
        full, zoom = figure.subplots(2, 1, height_ratios=(1, 2))
    else:
        full, zoom = figure.subplots(), None
    draw_speed_axes(full, zoom, times, values, window, target_rpm, tolerance)


def draw_speed_axes(
    full: "Axes",
    zoom: "Axes | None",
    times: list[float],
    values: list[float],
    window: tuple[float, float] | None,
    target_rpm: float,
    tolerance: float,
) -> None:
    _full_run_axis(full, times, window, "rpm")
    full.plot(times, values, color="#126c8b", linewidth=1, marker=".", markersize=2.5)
    full.set_title(
        "Output shaft speed | Full run"
        if window
        else "Output shaft speed | No steady-state window",
        loc="left",
    )
    if window is None or zoom is None:
        return

    start, end = window
    half_band = target_rpm * tolerance
    lower, upper = target_rpm - half_band, target_rpm + half_band
    view_lower, view_upper = target_rpm - 1.5 * half_band, target_rpm + 1.5 * half_band
    zoom.axhspan(lower, upper, color="#e2f1e9")
    for level, style in ((lower, ":"), (target_rpm, "--"), (upper, ":")):
        zoom.axhline(level, color="#456654", linestyle=style, linewidth=1)
    steady = [
        (time, value) for time, value in zip(times, values, strict=True) if start <= time <= end
    ]
    zoom.plot(
        [time for time, _ in steady],
        [value for _, value in steady],
        color="#126c8b",
        linewidth=1.3,
        marker=".",
        markersize=3,
    )
    valid = [(time, value) for time, value in steady if math.isfinite(value)]
    below = [time for time, value in valid if value < view_lower]
    above = [time for time, value in valid if value > view_upper]
    for points, level, marker in (
        (below, view_lower + 0.10 * half_band, "v"),
        (above, view_upper - 0.10 * half_band, "^"),
    ):
        if points:
            zoom.scatter(
                points, [level] * len(points), color="#b03b42", marker=marker, s=25, zorder=4
            )
    summary = "No valid samples in the steady-state window"
    if valid:
        measured = [value for _, value in valid]
        low, high = min(measured), max(measured)
        summary = f"Measured {low:.2f}–{high:.2f} rpm | Peak to peak {high - low:.2f} rpm"
    if below or above:
        summary += f" | Edge markers: {len(below) + len(above)} outside view"
    zoom.set_title(
        f"Target {target_rpm:g} rpm | Acceptance band {lower:g}–{upper:g} rpm (±{100 * tolerance:g}%)\n{summary}",
        loc="left",
    )
    zoom.set(xlim=window, ylim=(view_lower, view_upper), xlabel="Elapsed time (s)", ylabel="rpm")
    zoom.grid(alpha=0.2)


def draw_power_axis(
    chart: "Axes",
    times: list[float],
    values: list[float],
    window: tuple[float, float] | None,
    power_min_w: float,
    power_max_w: float,
) -> None:
    _full_run_axis(chart, times, window, "W")
    chart.plot(
        times,
        values,
        color="#8292a4",
        alpha=0.55,
        linewidth=0.9,
        marker=".",
        markersize=2.5,
        label="Raw",
    )
    chart.plot(
        times,
        _power_average(times, values),
        color="#126c8b",
        linewidth=1.6,
        marker=".",
        markersize=2,
        label=f"{POWER_AVERAGE_S:g} s moving average",
    )
    timing = (
        f"Shaded: {window[0]:.2f}–{window[1]:.2f} s steady state"
        if window
        else "No steady-state window"
    )
    chart.set_title(
        f"DC bus input power\nLimits: {power_min_w:g} to {power_max_w:g} W | {timing}",
        loc="left",
    )
