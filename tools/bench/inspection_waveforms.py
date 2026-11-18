"""从本轮已归档的数据生成一张生产检测波形总览。"""

import csv
import math
from pathlib import Path

from _calibration_curves import draw_compensation_axis, read_hall_samples
from high_speed_curves import draw_power_axis, draw_speed_axes, telemetry_series
from production_power_check import MAX_GAP_S, SPEED_ERROR_FRACTION

FONT = ("Microsoft YaHei", "DejaVu Sans")


def _rows(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _unavailable(axis, title, reason):
    axis.set_axis_off()
    axis.text(0, 0.94, title, transform=axis.transAxes, fontfamily=FONT, fontsize=12, va="top")
    axis.text(
        0.5,
        0.45,
        reason,
        transform=axis.transAxes,
        fontfamily=FONT,
        color="#526477",
        ha="center",
        va="center",
        wrap=True,
    )


def _draw_hall(raw, xy, samples, capture, *, installed):
    raw_title, xy_title = "Hall 原码波形", "Hall XY 轨迹"
    if not samples:
        reason = "本轴无 Hall" if not installed else capture.get("error") or "本轮未采到 Hall 原码"
        _unavailable(raw, raw_title, reason)
        _unavailable(xy, xy_title, reason)
        return
    times = [float(row["nominal_elapsed_s"]) for row in samples]
    sin = [int(row["hall_sin_counts"]) for row in samples]
    cos = [int(row["hall_cos_counts"]) for row in samples]
    xlabel = (
        "设备读数时间 (s)" if capture.get("source") == "CAN Hall snapshots" else "名义采样时间 (s)"
    )
    for channel, values, color in (("sin", sin, "#126c8b"), ("cos", cos, "#aa491d")):
        raw.plot(
            times,
            values,
            color=color,
            linewidth=0.9,
            label=f"{channel}  最小 {min(values)} · 最大 {max(values)} cnt",
        )
    raw.set_title(raw_title, fontfamily=FONT, loc="left")
    raw.set(ylim=(0, 4095), ylabel="ADC cnt")
    raw.set_xlabel(xlabel, fontfamily=FONT)
    raw.legend(
        loc="lower right",
        bbox_to_anchor=(1, 1),
        borderaxespad=0.3,
        ncols=2,
        frameon=False,
        prop={"family": FONT, "size": 9},
    )
    raw.grid(alpha=0.2)
    xy.plot(sin, cos, linewidth=0.8, color="#286788")
    xy.set_title(xy_title, fontfamily=FONT, loc="left")
    xy.set(xlim=(0, 4095), ylim=(0, 4095), xlabel="Hall sin (cnt)", ylabel="Hall cos (cnt)")
    xy.set_aspect("equal", adjustable="box")
    xy.grid(alpha=0.2)


def _draw_compensation(figure, grid, rows, *, installed, diagnostic_hall):
    for index, key, title, name, color, reason in (
        (
            1,
            "hall",
            "Hall 角度误差（诊断）" if diagnostic_hall else "Hall 角度误差",
            "Output Hall",
            "#aa491d",
            "本轮没有有效的 Hall 补偿表" if installed else "本轴无 Hall",
        ),
        (2, "motor", "编码器角度误差", "Motor encoder", "#215888", "本轮未取得编码器补偿表"),
    ):
        axis = figure.add_subplot(grid[index, :])
        field = f"{key}_raw_minus_corrected_deg"
        if not rows or field not in rows[0]:
            _unavailable(axis, title, reason)
            continue
        angles = [float(row["raw_angle_deg"]) for row in rows]
        values = [float(row[field]) for row in rows]
        draw_compensation_axis(axis, angles, values, name, color)
        span = max(values) - min(values)
        axis.set_title(f"{title}  |  峰峰值 {span:.3f}°", fontfamily=FONT, loc="left")


def _draw_high_speed(figure, grid, rows, power):
    full, steady, input_power = (figure.add_subplot(grid[index, :]) for index in (3, 4, 5))
    window = tuple(power["steady_window_s"]) if power.get("steady_window_s") else None
    samples = [
        {
            "elapsed_s": float(row["elapsed_s"]),
            "output_rpm": float(row["output_rpm"]) if row.get("output_rpm") else None,
            "bus_input_power_w": float(row["bus_input_power_w"])
            if row.get("bus_input_power_w")
            else None,
        }
        for row in rows
    ]
    speed_time, speed = telemetry_series(samples, "output_rpm", MAX_GAP_S)
    power_time, values = telemetry_series(samples, "bus_input_power_w", MAX_GAP_S)
    has_speed = any(math.isfinite(value) for value in speed)
    if has_speed:
        draw_speed_axes(
            full, steady, speed_time, speed, window, power["output_rpm"], SPEED_ERROR_FRACTION
        )
        full.set_title("输出轴转速稳定性  |  全程", fontfamily=FONT, loc="left")
    else:
        _unavailable(full, "输出轴转速稳定性  |  全程", "本轮未采到输出轴转速")
    if window is None:
        _unavailable(steady, "输出轴转速稳定性  |  判定窗口", "本轮未取得稳速判定窗口")
    elif not has_speed:
        _unavailable(steady, "输出轴转速稳定性  |  判定窗口", "本轮未采到输出轴转速")
    if any(math.isfinite(value) for value in values):
        draw_power_axis(
            input_power, power_time, values, window, power["power_min_w"], power["power_max_w"]
        )
        input_power.set_title(
            "母线输入功率稳定性\n" + input_power.get_title(loc="left").split("\n", maxsplit=1)[-1],
            fontfamily=FONT,
            loc="left",
        )
        input_power.legend(loc="upper right", ncols=2, frameon=False)
    else:
        _unavailable(input_power, "母线输入功率稳定性", "本轮未采到母线输入功率")


def export_waveform_overview(report: dict, run: Path, path: Path) -> bool:
    """汇总六行曲线，缺测保留原槽位，不使用其他轮次的数据。"""
    calibration = report.get("calibration", {})
    capture = calibration.get("raw_capture", {})
    calibration_dir = run / "calibration"
    hall = (
        read_hall_samples(calibration_dir, capture)
        if capture.get("csv") and (calibration_dir / capture["csv"]).is_file()
        else []
    )
    compensation = _rows(calibration_dir / "angle-calibration.csv")
    power = report.get("power_check", {})
    telemetry = _rows(run / power["csv"]) if power.get("csv") else []
    if not (hall or compensation or telemetry):
        return False

    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    installed = report.get("preflight", {}).get("board_items", [0] * 8)[7] != 3
    figure = Figure(figsize=(13, 15), dpi=300, layout="constrained", facecolor="#f5f8fa")
    figure.suptitle(
        f"生产检测波形总览  |  电机序列号 {report['motor_serial']}  ·  测试需求号 {report['test_requirement']}",
        fontfamily=FONT,
        fontsize=19,
    )
    grid = figure.add_gridspec(6, 3, height_ratios=(1.35, 1, 1, 1, 1.18, 1.18))
    _draw_hall(
        figure.add_subplot(grid[0, :2]),
        figure.add_subplot(grid[0, 2]),
        hall,
        capture,
        installed=installed,
    )
    _draw_compensation(
        figure,
        grid,
        compensation,
        installed=installed,
        diagnostic_hall=calibration.get("saved_calibration_fault", {}).get(
            "hall_diagnostic_curve", False
        ),
    )
    _draw_high_speed(figure, grid, telemetry, power)
    FigureCanvasAgg(figure).print_png(path)
    return True
