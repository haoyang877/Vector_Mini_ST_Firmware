#!/usr/bin/env python3
"""Decode the 9x int16 Mode 13 RTT frame and render a dependency-free plot."""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


FRAME = struct.Struct("<9h")
SAMPLE_PERIOD_S = 0.01
POLE_PAIRS = 21.0
STEP_NAMES = {
    22: "align_origin",
    23: "wait_closed_loop",
    24: "speed_stable",
    25: "find_origin",
    26: "sample_cw",
    27: "build_lut",
    28: "verify_cw",
    29: "stop_decel",
    30: "stop_current",
    31: "clear_samples",
}
STARTUP_NAMES = {
    0: "idle",
    1: "align",
    2: "open_loop",
    3: "speed_lock",
    4: "handoff",
    5: "closed_loop",
}


def decode(path: Path) -> list[dict[str, float | int]]:
    raw = path.read_bytes()
    if len(raw) % FRAME.size:
        raise ValueError(f"RTT file size {len(raw)} is not divisible by {FRAME.size}")
    frames = [FRAME.unpack_from(raw, offset) for offset in range(0, len(raw), FRAME.size)]
    active = [i for i, f in enumerate(frames) if 22 <= f[0] <= 31 and 0 <= f[1] <= 5]
    if not active:
        raise ValueError("No instrumented Mode 13 frames found")
    first, last = active[0], active[-1]
    rows = []
    for index, f in enumerate(frames[first : last + 1]):
        rows.append(
            {
                "time_s": round(index * SAMPLE_PERIOD_S, 4),
                "calibration_step": f[0],
                "startup_state": f[1],
                "mechanical_speed_rad_s": f[2] / 100.0,
                "actual_electrical_speed_rad_s": f[2] / 100.0 * POLE_PAIRS,
                "open_loop_electrical_speed_rad_s": f[3] / 10.0,
                "observer_electrical_speed_rad_s": f[4] / 10.0,
                "lock_electrical_speed_rad_s": f[5] / 10.0,
                "iq_reference_a": f[6] / 1000.0,
                "iq_actual_a": f[7] / 1000.0,
                "phase_error_deg": f[8] * 180.0 / 32768.0,
            }
        )
    return rows


def transitions(rows: list[dict[str, float | int]], key: str, names: dict[int, str]) -> list[dict[str, object]]:
    result = []
    previous = None
    for row in rows:
        value = int(row[key])
        if value != previous:
            result.append({"time_s": row["time_s"], "value": value, "name": names.get(value, str(value))})
            previous = value
    return result


def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = [
        Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
        Path("C:/Windows/Fonts/segoeuib.ttf" if bold else "C:/Windows/Fonts/segoeui.ttf"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


def padded_extent(values: list[float]) -> tuple[float, float]:
    lo, hi = min(values), max(values)
    if math.isclose(lo, hi):
        return lo - 1.0, hi + 1.0
    pad = (hi - lo) * 0.08
    return lo - pad, hi + pad


def draw_panel(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    rows: list[dict[str, float | int]],
    series: list[tuple[str, str, tuple[int, int, int]]],
    title: str,
    y_label: str,
    y_limits: tuple[float, float] | None = None,
) -> None:
    x0, y0, x1, y1 = box
    left, right, top, bottom = 92, 22, 55, 48
    px0, py0, px1, py1 = x0 + left, y0 + top, x1 - right, y1 - bottom
    label_font, small_font = font(18, True), font(14)
    draw.text((x0 + 8, y0 + 5), title, fill=(25, 31, 40), font=label_font)
    all_values = [float(row[key]) for key, _, _ in series for row in rows]
    ymin, ymax = y_limits or padded_extent(all_values)
    tmax = max(float(row["time_s"]) for row in rows) or 1.0

    for i in range(6):
        ratio = i / 5
        y = py1 - ratio * (py1 - py0)
        value = ymin + ratio * (ymax - ymin)
        draw.line((px0, y, px1, y), fill=(220, 225, 232), width=1)
        draw.text((x0 + 8, y - 8), f"{value:.1f}", fill=(75, 85, 99), font=small_font)
    for i in range(6):
        ratio = i / 5
        x = px0 + ratio * (px1 - px0)
        draw.line((x, py0, x, py1), fill=(235, 238, 242), width=1)
        draw.text((x - 14, py1 + 8), f"{tmax * ratio:.1f}", fill=(75, 85, 99), font=small_font)
    draw.rectangle((px0, py0, px1, py1), outline=(120, 130, 145), width=1)
    draw.text((px0 + (px1 - px0) / 2 - 24, py1 + 27), "time (s)", fill=(40, 48, 60), font=small_font)
    draw.text((x0 + 8, y0 + 31), y_label, fill=(40, 48, 60), font=small_font)

    legend_x = px1 - 10
    for key, label, color in reversed(series):
        width = draw.textlength(label, font=small_font)
        legend_x -= int(width + 32)
        draw.line((legend_x, y0 + 17, legend_x + 18, y0 + 17), fill=color, width=3)
        draw.text((legend_x + 22, y0 + 8), label, fill=(40, 48, 60), font=small_font)

    for key, _, color in series:
        points = []
        for row in rows:
            x = px0 + float(row["time_s"]) / tmax * (px1 - px0)
            value = float(row[key])
            y = py1 - (value - ymin) / (ymax - ymin) * (py1 - py0)
            points.append((x, max(py0, min(py1, y))))
        if len(points) >= 2:
            draw.line(points, fill=color, width=3)


def render(rows: list[dict[str, float | int]], output: Path) -> None:
    width, height = 1600, 1480
    image = Image.new("RGB", (width, height), (250, 251, 253))
    draw = ImageDraw.Draw(image)
    draw.text((42, 22), "Mode 13 observer calibration diagnostic", fill=(18, 24, 33), font=font(28, True))
    last = rows[-1]
    subtitle = (
        f"{len(rows)} RTT samples @ 100 Hz  |  final step {STEP_NAMES.get(int(last['calibration_step']), last['calibration_step'])}"
        f"  |  final startup {STARTUP_NAMES.get(int(last['startup_state']), last['startup_state'])}"
    )
    draw.text((42, 62), subtitle, fill=(70, 79, 91), font=font(16))

    panels = [(30, 95, 1570, 500), (30, 505, 1570, 850), (30, 855, 1570, 1175)]
    speed_values = [
        float(row[key])
        for row in rows
        if float(row["time_s"]) >= 0.10
        for key in (
            "actual_electrical_speed_rad_s",
            "open_loop_electrical_speed_rad_s",
            "observer_electrical_speed_rad_s",
            "lock_electrical_speed_rad_s",
        )
    ]
    draw_panel(
        draw,
        panels[0],
        rows,
        [
            ("actual_electrical_speed_rad_s", "encoder actual", (17, 94, 163)),
            ("open_loop_electrical_speed_rad_s", "open-loop command", (230, 126, 34)),
            ("observer_electrical_speed_rad_s", "observer", (196, 56, 63)),
            ("lock_electrical_speed_rad_s", "lock filter", (90, 70, 170)),
        ],
        "Electrical-speed lock",
        "rad/s",
        padded_extent(speed_values),
    )
    draw_panel(
        draw,
        panels[1],
        rows,
        [("iq_reference_a", "Iq reference", (230, 126, 34)), ("iq_actual_a", "Iq actual", (17, 94, 163))],
        "Torque-producing current",
        "A",
    )
    draw_panel(
        draw,
        panels[2],
        rows,
        [("phase_error_deg", "encoder - observer", (196, 56, 63))],
        "Electrical phase error",
        "degrees",
        (-180.0, 180.0),
    )

    y = 1210
    draw.text((42, y), "State transitions", fill=(25, 31, 40), font=font(18, True))
    y += 32
    for label, key, names in (("calibration", "calibration_step", STEP_NAMES), ("startup", "startup_state", STARTUP_NAMES)):
        parts = [f"{item['time_s']:.2f}s {item['name']}" for item in transitions(rows, key, names)]
        draw.text((42, y), f"{label}: " + "  →  ".join(parts), fill=(50, 59, 71), font=font(15))
        y += 31
    image.save(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("rtt_binary", type=Path)
    parser.add_argument("output_directory", type=Path)
    args = parser.parse_args()
    args.output_directory.mkdir(parents=True, exist_ok=True)
    rows = decode(args.rtt_binary)

    csv_path = args.output_directory / "mode13_rtt.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    observer = [float(r["observer_electrical_speed_rad_s"]) for r in rows]
    actual = [float(r["actual_electrical_speed_rad_s"]) for r in rows]
    lock = [float(r["lock_electrical_speed_rad_s"]) for r in rows]
    summary = {
        "sample_count": len(rows),
        "duration_s": rows[-1]["time_s"],
        "calibration_transitions": transitions(rows, "calibration_step", STEP_NAMES),
        "startup_transitions": transitions(rows, "startup_state", STARTUP_NAMES),
        "maximum_actual_electrical_speed_rad_s": max(actual),
        "maximum_observer_electrical_speed_rad_s": max(observer),
        "maximum_lock_electrical_speed_rad_s": max(lock),
        "final_actual_electrical_speed_rad_s": actual[-1],
        "final_observer_electrical_speed_rad_s": observer[-1],
        "final_lock_electrical_speed_rad_s": lock[-1],
        "final_speed_ratio": observer[-1] / actual[-1] if actual[-1] else None,
    }
    (args.output_directory / "mode13_rtt_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    render(rows, args.output_directory / "mode13_rtt.png")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
