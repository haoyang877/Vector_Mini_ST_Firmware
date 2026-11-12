"""从已回读的 schema14 / OEN3 标定表导出单圈角度补偿曲线。"""

import csv
import struct


def export_curves(record, axis, has_hall, uid, run):
    """各轴独立原始角坐标，符号统一为原始角减校正角。"""
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure
    from matplotlib.ticker import MaxNLocator

    scale = 360.0 / 65536
    curves = {"motor": [v * scale for v in struct.unpack_from("<1024h", record, 44)]}
    if has_hall:
        curves["hall"] = [-v * scale for v in struct.unpack_from("<1024h", record, 2320)]

    angles = [i * 360.0 / 1024 for i in range(1024)]
    csv_path = run / "angle-calibration.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["raw_angle_deg", *(f"{key}_raw_minus_corrected_deg" for key in curves)])
        writer.writerows(zip(angles, *curves.values(), strict=True))

    figure = Figure(figsize=(9, 2.2 + 2.6 * len(curves)), layout="constrained")
    canvas = FigureCanvasAgg(figure)
    figure.suptitle(f"{axis.model}   /   0x{axis.node:02X}   /   {uid.hex().upper()}", fontsize=15)
    spans = {}
    for i, (key, values) in enumerate(curves.items(), 1):
        spans[key] = max(values) - min(values)
        name = "Motor encoder" if key == "motor" else "Output Hall"
        color = "#215888" if key == "motor" else "#aa491d"
        plot = figure.add_subplot(len(curves), 1, i)
        plot.plot([*angles, 360], [*values, values[0]], color=color, linewidth=1.5)
        plot.set_title(f"{name}   |   peak-to-peak {spans[key]:.3f}°", loc="left", fontsize=12)
        plot.set(
            xlim=(0, 360),
            xticks=range(0, 361, 45),
            xlabel=f"{name} raw angle (°)",
            ylabel="Raw − corrected (°)",
        )
        plot.yaxis.set_major_locator(MaxNLocator(6))
        plot.spines[["top", "right"]].set_visible(False)
        plot.grid(alpha=0.22)
    png_path = run / "angle-calibration.png"
    canvas.print_png(str(png_path))
    return {
        "png": png_path.name,
        "csv": csv_path.name,
        "points_per_curve": len(angles),
        "peak_to_peak_deg": spans,
    }
