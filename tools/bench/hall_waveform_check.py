"""从本轮 CAN Hall 原码判定 XY 轨迹中重复的局部内凹。"""

import csv
import math
from collections import defaultdict
from itertools import pairwise
from pathlib import Path
from statistics import median

BIN_DEG = 5
BINS_PER_TURN = 360 // BIN_DEG
MIN_SAMPLES_PER_BIN = 3
MIN_REPEATS = 2
CHORD_OFFSET_BINS = 2
MIN_CHORD_DEPTH = 0.020
ADC_MAX_COUNTS = 4095


def inspect_hall_waveform(directory: Path, capture: dict) -> dict:
    """分析已关闭的采样文件，按实际角覆盖区分未检出与未测。"""
    result = dict(
        status="not_measured",
        bin_deg=BIN_DEG,
        min_samples_per_bin=MIN_SAMPLES_PER_BIN,
        min_repeats=MIN_REPEATS,
        chord_offset_deg=CHORD_OFFSET_BINS * BIN_DEG,
        min_chord_depth=MIN_CHORD_DEPTH,
        trial=True,
    )
    if capture.get("source") != "CAN Hall snapshots" or not capture.get("samples"):
        return dict(result, reason=capture.get("error") or "没有本轮 CAN Hall 原码")
    try:
        with (directory / capture["csv"]).open(newline="", encoding="utf-8") as stream:
            samples = [
                (int(row["hall_sin_counts"]), int(row["hall_cos_counts"]))
                for row in csv.DictReader(stream)
            ]
    except (OSError, csv.Error, KeyError, TypeError, ValueError):
        return dict(result, reason="Hall 原码文件缺失或格式无效")
    if len(samples) != capture["samples"] or any(
        not 0 <= value <= ADC_MAX_COUNTS for pair in samples for value in pair
    ):
        return dict(result, reason="Hall 原码条数或数值无效")

    sin, cos = zip(*samples, strict=True)
    sin_lo, sin_hi, cos_lo, cos_hi = min(sin), max(sin), min(cos), max(cos)
    result["extrema_counts"] = dict(sin=[sin_lo, sin_hi], cos=[cos_lo, cos_hi])
    if capture.get("error") or capture.get("complete") is False:
        return dict(result, reason="Hall 原码采集不完整")
    if min(sin_lo, cos_lo) == 0 or max(sin_hi, cos_hi) == ADC_MAX_COUNTS:
        return dict(result, reason="Hall 原码触及 ADC 端点，无法判定凹坑")
    if sin_hi == sin_lo or cos_hi == cos_lo:
        return dict(result, reason="Hall 原码没有可归一化的幅值")
    polar = []
    for sin_raw, cos_raw in samples:
        x = (2 * sin_raw - sin_hi - sin_lo) / (sin_hi - sin_lo)
        y = (2 * cos_raw - cos_hi - cos_lo) / (cos_hi - cos_lo)
        polar.append((math.degrees(math.atan2(x, y)), math.hypot(x, y)))

    unfolded = [polar[0][0]]
    for (previous, _), (angle, _) in pairwise(polar):
        unfolded.append(unfolded[-1] + (angle - previous + 180) % 360 - 180)
    direction = 1 if unfolded[-1] >= unfolded[0] else -1
    bins: dict[int, list[float]] = defaultdict(list)
    last = math.floor(direction * unfolded[0] / BIN_DEG)
    for angle, (_, radius) in zip(unfolded, polar, strict=True):
        index = math.floor(direction * angle / BIN_DEG)
        # 已经扫过的箱不收回退时的样本，避免在同一处往返被当作复现。
        if index >= last:
            bins[index].append(radius)
            last = index
    # 排除轨迹起停处的两个未扫完的箱。
    del bins[next(iter(bins))]
    bins.pop(last, None)
    medians = {
        index: median(values)
        for index, values in bins.items()
        if len(values) >= MIN_SAMPLES_PER_BIN
    }
    # 每个相位箱都至少被有效扫过两次，才允许报告未检出。
    depths = _chord_depths(medians)
    coverage = [0] * BINS_PER_TURN
    repeated_pairs: dict[int, list[float]] = defaultdict(list)
    for index, depth in depths.items():
        following = depths.get(index + 1)
        if following is None:
            continue
        first = (index if direction > 0 else -index - 2) % BINS_PER_TURN
        coverage[first] += 1
        if min(depth, following) >= MIN_CHORD_DEPTH:
            repeated_pairs[first].append(max(depth, following))
    result["twice_covered_deg"] = BIN_DEG * sum(count >= MIN_REPEATS for count in coverage)
    if result["twice_covered_deg"] < 360:
        return dict(result, reason="局部形状可比较的有效相位覆盖不足两圈")

    confirmed = {index: hits for index, hits in repeated_pairs.items() if len(hits) >= MIN_REPEATS}
    return dict(
        result,
        status="detected" if confirmed else "clear",
        sectors=_merge_sectors(confirmed),
    )


def _chord_depths(medians: dict[int, float]) -> dict[int, float]:
    """比较区间中位半径与两侧端点连线在该相位的交点。"""
    depths = {}
    offset = CHORD_OFFSET_BINS
    cosine = math.cos(math.radians(offset * BIN_DEG))
    for index, radius in medians.items():
        if any(point not in medians for point in range(index - offset, index + offset + 1)):
            continue
        left, right = medians[index - offset], medians[index + offset]
        if left + right > 0:
            # 两侧相位对称，弦在中间射线上的半径是两端半径的调和平均乘余弦。
            depths[index] = 2 * left * right * cosine / (left + right) - radius
    return depths


def hall_waveform_criterion(waveform: dict) -> str:
    bins = waveform.get("bin_deg", BIN_DEG)
    repeats = waveform.get("min_repeats", MIN_REPEATS)
    if "radius_limit" in waveform:
        return f"相邻两个 {bins}° 区间半径中位数均 < {waveform['radius_limit']:.2f} · 同相位 ≥ {repeats} 次"
    depth = waveform.get("min_chord_depth", MIN_CHORD_DEPTH)
    return f"相邻两个 {bins}° 区间局部弦深度均 ≥ {depth:.3f} · 同相位 ≥ {repeats} 次"


def hall_signal_limits(waveform: dict, limits: dict) -> dict:
    """两路各自核对允许区间与 ADC 全标度的一半。"""
    lower, upper = limits["hall_raw_min"], limits["hall_raw_max"]
    extrema = waveform.get("extrema_counts")
    result = dict(min_counts=lower, max_counts=upper, adc_span_counts=ADC_MAX_COUNTS)
    if not extrema:
        return result
    peak_to_peak = {key: high - low for key, (low, high) in extrema.items()}
    return dict(
        result,
        extrema_counts=extrema,
        peak_to_peak_counts=peak_to_peak,
        range_passed=all(lower <= low <= high <= upper for low, high in extrema.values()),
        amplitude_passed=all(2 * value > ADC_MAX_COUNTS for value in peak_to_peak.values()),
    )


def _merge_sectors(confirmed: dict[int, list[float]]) -> list[dict]:
    # 相邻证据对合并成区段，保留跨相位零点的区段。
    affected = set(confirmed) | {(index + 1) % BINS_PER_TURN for index in confirmed}
    starts = sorted(index for index in affected if (index - 1) % BINS_PER_TURN not in affected)
    if len(affected) == BINS_PER_TURN:
        starts = [0]
    sectors = []
    for start in starts:
        width = 1
        while width < BINS_PER_TURN and (start + width) % BINS_PER_TURN in affected:
            width += 1
        evidence = [
            confirmed[(start + step) % BINS_PER_TURN]
            for step in range(width)
            if (start + step) % BINS_PER_TURN in confirmed
        ]
        sectors.append(
            dict(
                start_deg=start * BIN_DEG,
                width_deg=width * BIN_DEG,
                max_chord_depth=round(max(map(max, evidence)), 4),
                repeat_count=max(map(len, evidence)),
            )
        )
    return sectors
