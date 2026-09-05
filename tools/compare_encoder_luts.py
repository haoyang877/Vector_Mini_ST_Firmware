#!/usr/bin/env python3
"""Compare two 1024-point encoder calibration LUTs and render a PNG report."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def load_lut(path: Path) -> list[tuple[float, float]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = [
            (float(row["raw_deg"]), float(row["error_deg"]))
            for row in csv.DictReader(stream)
        ]
    if len(rows) != 1024:
        raise ValueError(f"{path}: expected 1024 rows, received {len(rows)}")
    return rows


def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = [
        Path("C:/Windows/Fonts/segoeuib.ttf" if bold else "C:/Windows/Fonts/segoeui.ttf"),
        Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


def padded_extent(values: list[float], minimum_span: float = 0.1) -> tuple[float, float]:
    low, high = min(values), max(values)
    span = max(high - low, minimum_span)
    return low - 0.08 * span, high + 0.08 * span


def pearson(left: list[float], right: list[float]) -> float:
    numerator = sum(a * b for a, b in zip(left, right))
    denominator = math.sqrt(sum(a * a for a in left) * sum(b * b for b in right))
    return numerator / denominator if denominator else 0.0


def draw_panel(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    x_values: list[float],
    series: list[tuple[str, list[float], tuple[int, int, int]]],
    title: str,
    y_label: str,
) -> None:
    x0, y0, x1, y1 = box
    plot_left, plot_top = x0 + 105, y0 + 58
    plot_right, plot_bottom = x1 - 28, y1 - 54
    y_values = [value for _, values, _ in series for value in values]
    y_min, y_max = padded_extent(y_values)
    title_font, label_font, tick_font = font(18, True), font(14), font(13)

    def px(value: float) -> float:
        return plot_left + value / 360.0 * (plot_right - plot_left)

    def py(value: float) -> float:
        return plot_bottom - (value - y_min) / (y_max - y_min) * (plot_bottom - plot_top)

    draw.text((x0 + 8, y0 + 5), title, fill=(25, 31, 40), font=title_font)
    draw.text((x0 + 8, y0 + 33), y_label, fill=(40, 48, 60), font=label_font)
    for index in range(6):
        ratio = index / 5.0
        y = plot_bottom - ratio * (plot_bottom - plot_top)
        value = y_min + ratio * (y_max - y_min)
        draw.line((plot_left, y, plot_right, y), fill=(220, 225, 232), width=1)
        draw.text((x0 + 8, y - 8), f"{value:+.2f}", fill=(75, 85, 99), font=tick_font)
    for value in range(0, 361, 60):
        x = px(float(value))
        draw.line((x, plot_top, x, plot_bottom), fill=(235, 238, 242), width=1)
        draw.text((x - 12, plot_bottom + 8), str(value), fill=(75, 85, 99), font=tick_font)
    if y_min <= 0.0 <= y_max:
        draw.line((plot_left, py(0.0), plot_right, py(0.0)), fill=(115, 125, 138), width=1)
    draw.rectangle((plot_left, plot_top, plot_right, plot_bottom), outline=(120, 130, 145), width=1)
    draw.text(
        ((plot_left + plot_right) / 2 - 82, plot_bottom + 29),
        "raw mechanical angle (deg)",
        fill=(40, 48, 60),
        font=label_font,
    )

    legend_x = plot_right - 10
    for label, _, color in reversed(series):
        width = int(draw.textlength(label, font=label_font))
        legend_x -= width + 34
        draw.line((legend_x, y0 + 19, legend_x + 18, y0 + 19), fill=color, width=3)
        draw.text((legend_x + 23, y0 + 10), label, fill=(40, 48, 60), font=label_font)

    for _, values, color in series:
        points = [(px(x), py(y)) for x, y in zip(x_values, values)]
        draw.line(points, fill=color, width=3, joint="curve")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode5_csv", type=Path)
    parser.add_argument("mode13_csv", type=Path)
    parser.add_argument("output_directory", type=Path)
    args = parser.parse_args()

    mode5_rows = load_lut(args.mode5_csv)
    mode13_rows = load_lut(args.mode13_csv)
    x5 = [row[0] for row in mode5_rows]
    x13 = [row[0] for row in mode13_rows]
    max_raw_angle_delta = max(abs(a - b) for a, b in zip(x5, x13))
    if max_raw_angle_delta > 0.01:
        raise ValueError(f"raw-angle grids do not match: max delta {max_raw_angle_delta} deg")

    mode5 = [row[1] for row in mode5_rows]
    mode13 = [row[1] for row in mode13_rows]
    mode5_mean = statistics.fmean(mode5)
    mode13_mean = statistics.fmean(mode13)
    mode5_shape = [value - mode5_mean for value in mode5]
    mode13_shape = [value - mode13_mean for value in mode13]
    shape_delta = [b - a for a, b in zip(mode5_shape, mode13_shape)]

    def rms(values: list[float]) -> float:
        return math.sqrt(statistics.fmean(value * value for value in values))

    summary = {
        "samples": len(mode5),
        "maximum_raw_angle_grid_delta_deg": max_raw_angle_delta,
        "mode5": {
            "mean_correction_deg": mode5_mean,
            "peak_to_peak_deg": max(mode5) - min(mode5),
            "demeaned_rms_deg": rms(mode5_shape),
            "demeaned_peak_abs_deg": max(abs(value) for value in mode5_shape),
        },
        "mode13": {
            "mean_correction_deg": mode13_mean,
            "peak_to_peak_deg": max(mode13) - min(mode13),
            "demeaned_rms_deg": rms(mode13_shape),
            "demeaned_peak_abs_deg": max(abs(value) for value in mode13_shape),
        },
        "comparison": {
            "constant_offset_mode13_minus_mode5_deg": mode13_mean - mode5_mean,
            "demeaned_waveform_correlation": pearson(mode5_shape, mode13_shape),
            "demeaned_difference_rms_deg": rms(shape_delta),
            "demeaned_difference_mean_abs_deg": statistics.fmean(abs(value) for value in shape_delta),
            "demeaned_difference_peak_abs_deg": max(abs(value) for value in shape_delta),
        },
    }

    args.output_directory.mkdir(parents=True, exist_ok=True)
    (args.output_directory / "mode5_mode13_comparison.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    with (args.output_directory / "mode5_mode13_comparison.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "raw_deg",
                "mode5_correction_deg",
                "mode13_correction_deg",
                "mode5_demeaned_deg",
                "mode13_demeaned_deg",
                "mode13_minus_mode5_demeaned_deg",
            ]
        )
        writer.writerows(zip(x5, mode5, mode13, mode5_shape, mode13_shape, shape_delta))

    image = Image.new("RGB", (1600, 1240), (250, 251, 253))
    draw = ImageDraw.Draw(image)
    draw.text((42, 20), "Mode 5 vs Mode 13 encoder calibration", fill=(18, 24, 33), font=font(28, True))
    subtitle = (
        f"1024 matched LUT bins | shape correlation {summary['comparison']['demeaned_waveform_correlation']:.3f}"
        f" | de-meaned difference RMS {summary['comparison']['demeaned_difference_rms_deg']:.3f} deg"
    )
    draw.text((42, 61), subtitle, fill=(70, 79, 91), font=font(16))
    draw_panel(
        draw,
        (30, 92, 1570, 445),
        x5,
        [("Mode 5", mode5, (17, 94, 163)), ("Mode 13", mode13, (196, 56, 63))],
        "Stored correction, including reference offset",
        "correction (deg)",
    )
    draw_panel(
        draw,
        (30, 450, 1570, 830),
        x5,
        [("Mode 5", mode5_shape, (17, 94, 163)), ("Mode 13", mode13_shape, (196, 56, 63))],
        "Encoder nonlinearity after removing each method's constant offset",
        "de-meaned correction (deg)",
    )
    draw_panel(
        draw,
        (30, 835, 1570, 1205),
        x5,
        [("Mode 13 - Mode 5", shape_delta, (90, 70, 170))],
        "Residual difference between calibration methods",
        "difference (deg)",
    )
    image.save(args.output_directory / "mode5_mode13_comparison.png")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
