"""从已回读的 schema14 / OEN3 标定表导出单圈角度补偿曲线。"""

import csv
import math
import struct
from pathlib import Path


def export_curves(record, axis, has_hall, uid, run, *, raw_capture=None, diagnostic_hall=False):
    """各轴独立原始角坐标，符号统一为原始角减校正角。"""
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    scale = 360.0 / 65536
    raw = {"motor": struct.unpack_from("<1024h", record, 44)}
    if has_hall or diagnostic_hall:
        raw["hall"] = struct.unpack_from("<1024h", record, 2320)
    curves = {
        key: [v * scale * (-1 if key == "hall" else 1) for v in values]
        for key, values in raw.items()
    }

    angles = [i * 360.0 / 1024 for i in range(1024)]
    csv_path = run / "angle-calibration.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "table_index",
                "raw_angle_deg",
                *(
                    field
                    for key in curves
                    for field in (f"{key}_lut_q15", f"{key}_raw_minus_corrected_deg")
                ),
            ]
        )
        for index, angle in enumerate(angles):
            writer.writerow(
                [
                    index,
                    angle,
                    *(value for key in curves for value in (raw[key][index], curves[key][index])),
                ]
            )

    zero = struct.unpack_from("<H", record, 36)[0]
    values = {
        "electrical_zero_q15": zero,
        "electrical_zero_encoder_deg": zero * scale,
        "electrical_zero_valid": bool(record[40] & 2),
    }
    if has_hall:
        sin_offset, cos_offset, sin_amp, cos_amp = struct.unpack_from("<4f", record, 2188)
        phase = struct.unpack_from("<f", record, 2300)[0]
        values.update(
            hall_alignment_motor_rad=phase,
            hall_alignment_motor_deg=math.degrees(phase),
            hall_calibration_extrema_counts={
                "sin": {"min": sin_offset - sin_amp, "max": sin_offset + sin_amp},
                "cos": {"min": cos_offset - cos_amp, "max": cos_offset + cos_amp},
            },
        )
    values_csv = run / "calibration-values.csv"
    with values_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["parameter", "value", "unit", "source"])
        writer.writerow(["electrical_zero", zero, "Q15", "Flash offset 36"])
        writer.writerow(["electrical_zero_encoder", zero * scale, "deg", "Q15 * 360 / 65536"])
        writer.writerow(
            ["electrical_zero_valid", values["electrical_zero_valid"], "bool", "Flash flag bit 1"]
        )
        if has_hall:
            writer.writerow(["hall_alignment_motor", phase, "rad", "Flash offset 2300"])
            writer.writerow(["hall_alignment_motor", math.degrees(phase), "deg", "rad * 180 / pi"])
            for channel, bounds in values["hall_calibration_extrema_counts"].items():
                for bound, value in bounds.items():
                    writer.writerow(
                        [f"hall_{channel}_{bound}", value, "count", "saved offset +/- amplitude"]
                    )
    rows = len(curves) + 1 + (3 if has_hall else 0)
    figure = Figure(figsize=(10, 1.2 + 2.8 * rows), layout="constrained")
    canvas = FigureCanvasAgg(figure)
    figure.suptitle(
        f"{axis.model}   /   0x{axis.node:02X}   /   {uid.hex().upper()}",
        fontsize=15,
        fontfamily=["Microsoft YaHei", "DejaVu Sans"],
    )
    spans = {}
    for i, (key, curve_values) in enumerate(curves.items(), 1):
        spans[key] = max(curve_values) - min(curve_values)
        name = "Motor encoder" if key == "motor" else "Output Hall"
        if key == "hall" and diagnostic_hall:
            name += " (diagnostic)"
        color = "#215888" if key == "motor" else "#aa491d"
        draw_compensation_axis(figure.add_subplot(rows, 1, i), angles, curve_values, name, color)
    angles_plot = figure.add_subplot(rows, 1, len(curves) + 1)
    labels = ["Zero\n(encoder)"]
    angle_values = [values["electrical_zero_encoder_deg"]]
    if has_hall:
        labels.append("Hall phase\n(motor)")
        angle_values.append(values["hall_alignment_motor_deg"])
    angles_plot.barh(labels, angle_values, color=["#215888", "#aa491d"][: len(labels)])
    for index, value in enumerate(angle_values):
        angles_plot.text(value, index, f"  {value:.5f} deg", va="center")
    angles_plot.set_xlabel("Saved angle (deg)")
    angles_plot.margins(x=0.2)
    angles_plot.set_title(
        f"Commutation zero Q15={zero}; valid={values['electrical_zero_valid']}", loc="left"
    )
    angles_plot.grid(axis="x", alpha=0.22)
    if has_hall:
        extrema = values["hall_calibration_extrema_counts"]
        bounds_plot = figure.add_subplot(rows, 1, len(curves) + 2)
        for index, (channel, bounds) in enumerate(extrema.items()):
            low, high = bounds["min"], bounds["max"]
            bounds_plot.plot(
                [low, high],
                [index, index],
                "o-",
                linewidth=4,
                label=f"{channel}: min={low:g}, max={high:g}",
            )
        bounds_plot.set(
            yticks=[0, 1], yticklabels=["sin", "cos"], xlim=(0, 4095), xlabel="ADC counts"
        )
        bounds_plot.set_title(
            "Calibration extrema reconstructed from saved offset +/- amplitude", loc="left"
        )
        bounds_plot.legend(loc="upper right")
        bounds_plot.grid(alpha=0.22)
        raw_plot = figure.add_subplot(rows, 1, len(curves) + 3)
        xy_plot = figure.add_subplot(rows, 1, len(curves) + 4)
        _draw_hall_waveforms(
            raw_plot, xy_plot, read_hall_samples(run, raw_capture), raw_capture or {}
        )
    png_path = run / "angle-calibration.png"
    canvas.print_png(str(png_path))
    return {
        "png": png_path.name,
        "csv": csv_path.name,
        "points_per_curve": len(angles),
        "peak_to_peak_deg": spans,
        "calibration_values": values,
        "values_csv": values_csv.name,
    }


def draw_compensation_axis(plot, angles, curve_values, name, color):
    from matplotlib.ticker import MaxNLocator

    span = max(curve_values) - min(curve_values)
    plot.plot([*angles, 360], [*curve_values, curve_values[0]], color=color, linewidth=1.5)
    plot.set_title(f"{name}   |   peak-to-peak {span:.3f}°", loc="left", fontsize=12)
    plot.set(
        xlim=(0, 360),
        xticks=range(0, 361, 45),
        xlabel=f"{name} raw angle (°)",
        ylabel="Raw − corrected (°)",
    )
    plot.yaxis.set_major_locator(MaxNLocator(6))
    plot.spines[["top", "right"]].set_visible(False)
    plot.grid(alpha=0.22)


def read_hall_samples(run: Path, raw_capture: dict | None) -> list[dict]:
    if not raw_capture or not raw_capture.get("samples"):
        return []
    if raw_capture.get("error") and raw_capture.get("source") != "CAN Hall snapshots":
        return []
    with (run / raw_capture["csv"]).open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _draw_hall_waveforms(raw_plot, xy_plot, samples: list[dict], raw_capture: dict) -> None:
    if samples:
        times = [float(row["nominal_elapsed_s"]) for row in samples]
        sin = [int(row["hall_sin_counts"]) for row in samples]
        cos = [int(row["hall_cos_counts"]) for row in samples]
        raw_plot.plot(times, sin, label=f"sin: min={min(sin)}, max={max(sin)}", linewidth=0.6)
        raw_plot.plot(times, cos, label=f"cos: min={min(cos)}, max={max(cos)}", linewidth=0.6)
        raw_plot.legend(loc="upper right")
        raw_plot.set(
            xlabel="Device read elapsed (s)"
            if (raw_capture or {}).get("source") == "CAN Hall snapshots"
            else "Nominal elapsed (s, sample index / 2000)",
            ylabel="ADC counts",
            ylim=(0, 4095),
        )
        xy_plot.plot(sin, cos, linewidth=0.5)
        xy_plot.set(
            xlabel="Hall sin raw ADC counts",
            ylabel="Hall cos raw ADC counts",
            xlim=(0, 4095),
            ylim=(0, 4095),
        )
        xy_plot.set_aspect("equal", adjustable="box")
    else:
        reason = "Raw waveform unavailable; no synthetic samples"
        raw_plot.text(0.5, 0.5, reason, ha="center", transform=raw_plot.transAxes)
        xy_plot.text(0.5, 0.5, reason, ha="center", transform=xy_plot.transAxes)
    raw_plot.set_title(
        (
            "Hall raw ADC waveform (CAN snapshots; incomplete)"
            if raw_capture.get("error")
            else "Hall raw ADC waveform (CAN snapshots; not a continuous ADC stream)"
        )
        if (raw_capture or {}).get("source") == "CAN Hall snapshots"
        else "Hall raw ADC waveform (RTT received samples; gaps cannot be detected)",
        loc="left",
    )
    xy_plot.set_title("Hall raw ADC XY trace", loc="left")
    raw_plot.grid(alpha=0.22)
    xy_plot.grid(alpha=0.22)


def export_raw_hall_waveform(run: Path, raw_capture: dict) -> str | None:
    """标定失败时保存真实采样和 XY 轨迹，沿用成功标定图的原码绘制。"""
    samples = read_hall_samples(run, raw_capture)
    if not samples:
        return None
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    figure = Figure(figsize=(8.8, 5.8), dpi=115, layout="constrained")
    raw_plot, xy_plot = figure.subplots(2, 1)
    _draw_hall_waveforms(raw_plot, xy_plot, samples, raw_capture)
    path = run / "hall-raw.png"
    FigureCanvasAgg(figure).print_png(path)
    return path.name
