#!/usr/bin/env python3
"""Render a dependency-free PNG waveform plot for an exported encoder LUT."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def load_rows(path: Path) -> list[tuple[float, float]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = [
            (float(row["raw_deg"]), float(row["error_deg"]))
            for row in csv.DictReader(stream)
        ]
    if len(rows) != 1024:
        raise ValueError(f"expected 1024 LUT rows, received {len(rows)}")
    return rows


def padded_range(values: list[float], minimum_span: float = 0.1) -> tuple[float, float]:
    low = min(values)
    high = max(values)
    span = max(high - low, minimum_span)
    return low - span * 0.08, high + span * 0.08


def render_panel(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    x_values: list[float],
    y_values: list[float],
    title: str,
    y_label: str,
    color: tuple[int, int, int],
) -> None:
    left, top, right, bottom = box
    plot_left, plot_top = left + 90, top + 35
    plot_right, plot_bottom = right - 25, bottom - 60
    y_min, y_max = padded_range(y_values)
    x_min, x_max = 0.0, 360.0

    def px(x: float) -> float:
        return plot_left + (x - x_min) * (plot_right - plot_left) / (x_max - x_min)

    def py(y: float) -> float:
        return plot_bottom - (y - y_min) * (plot_bottom - plot_top) / (y_max - y_min)

    draw.text((left, top), title, fill=(25, 25, 25))
    draw.rectangle((plot_left, plot_top, plot_right, plot_bottom), outline=(100, 100, 100), width=1)

    for tick in range(0, 361, 60):
        x = px(float(tick))
        draw.line((x, plot_top, x, plot_bottom), fill=(225, 225, 225), width=1)
        draw.text((x - 10, plot_bottom + 8), str(tick), fill=(60, 60, 60))
    for index in range(5):
        value = y_min + (y_max - y_min) * index / 4.0
        y = py(value)
        draw.line((plot_left, y, plot_right, y), fill=(225, 225, 225), width=1)
        draw.text((left + 8, y - 7), f"{value:+.2f}", fill=(60, 60, 60))
    if y_min <= 0.0 <= y_max:
        draw.line((plot_left, py(0.0), plot_right, py(0.0)), fill=(120, 120, 120), width=1)

    points = [(px(x), py(y)) for x, y in zip(x_values, y_values)]
    draw.line(points, fill=color, width=2, joint="curve")
    draw.text(((plot_left + plot_right) // 2 - 65, bottom - 30), "Raw mechanical angle (deg)", fill=(25, 25, 25))
    draw.text((left + 8, plot_top - 20), y_label, fill=(25, 25, 25))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("png_path", type=Path)
    parser.add_argument("--summary", type=Path)
    parser.add_argument("--title", default="Encoder linearization result")
    args = parser.parse_args()

    rows = load_rows(args.csv_path)
    x_values = [row[0] for row in rows]
    errors = [row[1] for row in rows]
    mean_error = statistics.fmean(errors)
    shape = [value - mean_error for value in errors]
    rms_shape = math.sqrt(statistics.fmean(value * value for value in shape))

    image = Image.new("RGB", (1500, 1000), (250, 250, 250))
    draw = ImageDraw.Draw(image)
    draw.text((35, 18), args.title, fill=(15, 15, 15))
    render_panel(
        draw,
        (35, 55, 1465, 505),
        x_values,
        errors,
        "Stored LUT correction",
        "Correction (deg)",
        (32, 103, 178),
    )
    render_panel(
        draw,
        (35, 525, 1465, 975),
        x_values,
        shape,
        "Nonlinearity after removing constant phase offset",
        "De-meaned correction (deg)",
        (213, 94, 0),
    )
    args.png_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(args.png_path)

    summary = {
        "samples": len(errors),
        "minimum_error_deg": min(errors),
        "maximum_error_deg": max(errors),
        "peak_to_peak_error_deg": max(errors) - min(errors),
        "mean_error_deg": mean_error,
        "demeaned_rms_error_deg": rms_shape,
        "demeaned_peak_abs_error_deg": max(abs(value) for value in shape),
    }
    if args.summary:
        args.summary.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
