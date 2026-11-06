"""离线验证 Hall LUT 方案；只生成合成原码，不访问设备、不修改固件。"""

import argparse
import hashlib
import json
import math
import random
from pathlib import Path

TAU = math.tau
SIZE = 1024
SAMPLES = 2048
UNIT = TAU / 65536
STEP = TAU / SIZE


def signed16(value: int) -> int:
    """将模 65536 的差恢复为有符号计数。"""
    return (value + 32768) % 65536 - 32768


def motor_lut(raw: int) -> int:
    """模拟新电机表的真实整数插值和减号，包含跨零修正。"""
    index, fraction = (raw & 65535) >> 6, raw & 63
    a = int(500 + 120 * math.sin(TAU * index / 1024))
    b = int(500 + 120 * math.sin(TAU * ((index + 1) % 1024) / 1024))
    return (raw - a - (((b - a) * fraction) >> 6)) & 65535


def corrected_motor(raw: int) -> int:
    """保留圈数，使用本次新表校正连续原码。"""
    return raw + signed16(motor_lut(raw) - (raw & 65535))


def adc(theta: float, distortion: str, rng: random.Random, noise: float) -> tuple[int, int]:
    """生成两路独立偏置/幅值的 12 位原码，Hall 沿用负 atan2 方向。"""
    angle = theta
    if distortion in ("harmonic", "electrical_ripple"):
        angle += 0.06 * math.sin(2 * theta) + 0.025 * math.cos(3 * theta)
    elif distortion == "strong":
        angle += 0.4 * math.sin(theta)
    phase = math.radians(12) if distortion == "phase" else 0.0
    sine = 1900 - 1750 * math.sin(angle + phase) + rng.uniform(-noise, noise)
    cosine = 2000 + 1800 * math.cos(angle) + rng.uniform(-noise, noise)
    if distortion == "electrical_ripple":
        cosine += 7 * math.cos(112 * theta)
    return round(sine), round(cosine)


def hall_angle(pair: tuple[int, int], coeff: tuple[float, ...]) -> float:
    """按固件归一化、限幅和方向约定求原角。"""
    sine = max(-1.0, min(1.0, (pair[0] - coeff[0]) / coeff[2]))
    cosine = max(-1.0, min(1.0, (pair[1] - coeff[1]) / coeff[3]))
    return -math.atan2(sine, cosine) % TAU


def coefficients(distortion: str) -> tuple[float, ...]:
    """模拟现有整段极值记录，不拿稀疏配对点替代原幅值标定。"""
    rng = random.Random(0)
    pairs = [adc(TAU * i / 20000, distortion, rng, 0) for i in range(20000)]
    minima = [min(p[i] for p in pairs) for i in (0, 1)]
    maxima = [max(p[i] for p in pairs) for i in (0, 1)]
    return (
        (minima[0] + maxima[0]) / 2,
        (minima[1] + maxima[1]) / 2,
        (maxima[0] - minima[0]) / 2,
        (maxima[1] - minima[1]) / 2,
    )


def fit_lut(
    raw: list[int], angles: list[float], ratio: float, smooth: bool = False
) -> tuple[list[int], int]:
    """在 Hall 原角域插值节点，去常量相位，再量化为 1024 点修正。"""
    if len(raw) != SAMPLES:
        raise ValueError("coverage")
    raw, angles = raw[4:-4], angles[4:-4]
    unwrapped = [angles[0]]
    for a, b in zip(angles, angles[1:], strict=False):
        unwrapped.append(unwrapped[-1] + math.remainder(b - a, TAU))
    travel = unwrapped[-1] - unwrapped[0]
    motor = [corrected_motor(r) for r in raw]
    motor_travel = (motor[-1] - motor[0]) * UNIT
    direction = 1 if travel * motor_travel >= 0 else -1
    order = 1 if travel > 0 else -1
    if any((b - a) * order <= 0 for a, b in zip(unwrapped, unwrapped[1:], strict=False)):
        raise ValueError("mapping")
    errors = [
        direction * (m - motor[0]) * UNIT / ratio - (h - unwrapped[0])
        for m, h in zip(motor, unwrapped, strict=True)
    ]
    errors = [(errors[i - 1] + 2 * errors[i] + errors[i + 1]) / 4 for i in range(1, len(raw) - 1)]
    unwrapped = unwrapped[1:-1]
    sums, counts = [0.0] * SIZE, [0] * SIZE
    for i in range(len(unwrapped) - 1):
        a, b = unwrapped[i : i + 2]
        for node in range(math.ceil(min(a, b) / STEP), math.floor(max(a, b) / STEP) + 1):
            fraction = (node * STEP - a) / (b - a)
            error = errors[i] + fraction * (errors[i + 1] - errors[i])
            if counts[node % SIZE] == 0:
                sums[node % SIZE] = round(error / UNIT) * UNIT
                counts[node % SIZE] = 1
    if min(counts) == 0:
        raise ValueError("coverage")
    if abs(travel - direction * motor_travel / ratio) > 0.15:
        raise ValueError("relation")
    table = [s / n for s, n in zip(sums, counts, strict=True)]
    if smooth:
        table = [
            (table[(i - 1) % SIZE] + 2 * table[i] + table[(i + 1) % SIZE]) / 4 for i in range(SIZE)
        ]
    mean = round(sum(table) / SIZE / UNIT) * UNIT
    quantized = [round((e - mean) / UNIT) for e in table]
    if any(not -32768 <= v <= 32767 for v in quantized):
        raise ValueError("mapping")
    if not monotonic(quantized):
        raise ValueError("mapping")
    return quantized, direction


def monotonic(table: list[int]) -> bool:
    """检查相邻输出节点严格递增，包括最后到第一段。"""
    return all(65536 // SIZE + table[(i + 1) % SIZE] - table[i] > 0 for i in range(SIZE))


def correction(table: list[int], angle: float) -> float:
    """运行时周期线性插值；调用者提供有限角。"""
    coordinate = angle % TAU / STEP
    index = int(coordinate)
    fraction = coordinate - index
    return (table[index] + fraction * (table[(index + 1) % SIZE] - table[index])) * UNIT


def metrics(errors: list[float]) -> dict[str, float]:
    """常量相位自由，报告去均值后的非线性峰值及 RMS。"""
    center = sum(errors) / len(errors)
    residual = [math.degrees(e - center) for e in errors]
    return {
        "peak_deg": max(map(abs, residual)),
        "rms_deg": math.sqrt(sum(e * e for e in residual) / len(residual)),
        "constant_phase_deg": math.degrees(center),
    }


def paired_samples(
    ratio: int, direction: int, rotation: int, start: float, distortion: str, noise: float
) -> tuple[list[int], list[float], tuple[float, ...]]:
    """按现有 4 kHz 累计原码/距离阈值保存 2048 点，保留采样时刻量化。"""
    rng = random.Random(126)
    coeff = coefficients(distortion)
    spacing = int(ratio * 65536 * 1.01 / (SAMPLES - 1))
    raw, pairs = [], []
    first = 65400
    # 约 24 rad/s 电机转速的 4 kHz 采样，速度变化不改变保存点数。
    tick = 0
    motor_sum = sin_sum = cos_sum = interval_samples = 0
    while len(raw) < SAMPLES:
        value = first + rotation * round(64 * (tick + 0.08 * math.sin(0.03 * tick)))
        theta = direction * corrected_motor(value) * UNIT / ratio + start
        sine, cosine = adc(theta, distortion, rng, noise)
        motor_sum += value
        sin_sum += sine
        cos_sum += cosine
        interval_samples += 1
        if abs(value - first) >= len(raw) * spacing:
            raw.append(round(motor_sum / interval_samples))
            pairs.append((round(sin_sum / interval_samples), round(cos_sum / interval_samples)))
            motor_sum = sin_sum = cos_sum = interval_samples = 0
        tick += 1
    filtered_raw = raw.copy()
    filtered_pairs = pairs.copy()
    weights = (1, 8, 28, 56, 70, 56, 28, 8, 1)
    for i in range(4, SAMPLES - 4):
        filtered_raw[i] = round(sum(raw[i + k - 4] * w for k, w in enumerate(weights)) / 256)
        filtered_pairs[i] = tuple(
            (sum(pairs[i + k - 4][j] * w for k, w in enumerate(weights)) + 128) // 256
            for j in (0, 1)
        )
    return filtered_raw, [hall_angle(p, coeff) for p in filtered_pairs], coeff


def verify_case(
    ratio: int, direction: int, rotation: int, start: float, distortion: str, noise: float
) -> dict[str, object]:
    """用独立密集角度验证拟合，检查辨圈、节点和接缝。"""
    raw, angles, coeff = paired_samples(ratio, direction, rotation, start, distortion, noise)
    table, fitted_direction = fit_lut(raw, angles, ratio)
    smoothed, _ = fit_lut(raw, angles, ratio, smooth=True)
    assert fitted_direction == direction
    rng = random.Random(793)
    before, after, filtered = [], [], []
    for i in range(8192):
        theta = TAU * (i + 0.37) / 8192
        angle = hall_angle(adc(theta, distortion, rng, noise), coeff)
        error = math.remainder(angle - theta, TAU)
        before.append(error)
        after.append(error + correction(table, angle))
        filtered.append(error + correction(smoothed, angle))
    old, new = metrics(before), metrics(after)
    # 电周期扰动参与整圈极值，归一化附近的限幅也计入误差；独立门槛仍严于实机 0.5°。
    limit = 0.25 if distortion == "electrical_ripple" else 0.15
    assert new["peak_deg"] < limit, (distortion, new)
    if distortion != "ideal":
        assert new["rms_deg"] < old["rms_deg"] * 0.05
    assert abs(correction(table, 1e-8) - correction(table, TAU - 1e-8)) < 1e-7
    assert monotonic(table)
    # 原位置关系残差仍需足够小以辨圈；不把训练误差当独立精度。
    phases = [
        math.remainder(motor_lut(r) * UNIT - direction * ratio * (h + correction(table, h)), TAU)
        for r, h in zip(raw, angles, strict=True)
    ]
    phase = math.atan2(sum(map(math.sin, phases)), sum(map(math.cos, phases)))
    peak = max(abs(math.remainder(p - phase, TAU)) for p in phases)
    assert peak < math.pi / 2
    return {
        "ratio": ratio,
        "direction": direction,
        "rotation": rotation,
        "start_rad": start,
        "distortion": distortion,
        "noise_adc_counts": noise,
        "before": old,
        "after": new,
        "smoothing_comparison": metrics(filtered),
        "relation_peak_motor_rad": peak,
    }


def rejected_cases() -> list[str]:
    """验证缺样、覆盖缺失、折返、错误减速比及坏接缝不能通过。"""
    raw, angles, _ = paired_samples(8, 1, 1, 0, "harmonic", 0)
    folded = angles.copy()
    folded[50] = folded[49] - 0.03
    cases = [
        (raw[:-1], angles[:-1], 8, "coverage"),
        ([round(r / 2) for r in raw], [a / 2 for a in angles], 8, "mapping"),
        (raw, folded, 8, "mapping"),
        (raw, angles, 9, "relation"),
    ]
    # 同样 2048 点但只有半圈的平滑信号，无折返也必须拒绝。
    half = [TAU * i / (2 * (SAMPLES - 1)) for i in range(SAMPLES)]
    cases.append(([round(a * 8 / UNIT) for a in half], half, 8, "coverage"))
    results = []
    for samples, values, ratio, expected in cases:
        try:
            fit_lut(samples, values, ratio)
        except ValueError as error:
            assert str(error) == expected, (str(error), expected)
            results.append(expected)
        else:
            raise AssertionError("非法输入未拒绝")
    bad = [0] * SIZE
    bad[-1] = 65536 // SIZE
    assert not monotonic(bad)
    results.append("seam_fold")
    return results


def main() -> None:
    """运行方案检查并记录参数、独立验证结果和可复现模型哈希。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    cases = []
    for ratio in (3, 8, 12):
        for direction in (-1, 1):
            for rotation in (-1, 1):
                for start in (0.0, 0.23, 6.27):
                    for distortion in ("ideal", "harmonic", "phase", "strong", "electrical_ripple"):
                        cases.append(verify_case(ratio, direction, rotation, start, distortion, 1))
    failures = rejected_cases()
    # 跨零后的修正必须保留原圈数，不能先按单圈除以减速比。
    # 整数 LUT 斜率可能使相邻原码映射到同一个计数，不能要求步长恰好为 1。
    assert 0 <= corrected_motor(65536) - corrected_motor(65535) <= 2
    assert corrected_motor(65536) == 65536 - 500
    assert corrected_motor(0) < 0
    summary = {
        "scope": "synthetic_model_only_not_firmware_or_hardware_accuracy",
        "model_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "sample_count": SAMPLES,
        "lut_size": SIZE,
        "passed_cases": len(cases),
        "rejected_cases": failures,
        "worst_peak_deg": max(c["after"]["peak_deg"] for c in cases),
        "worst_rms_deg": max(c["after"]["rms_deg"] for c in cases),
        "cases": cases,
    }
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({k: v for k, v in summary.items() if k != "cases"}, ensure_ascii=False))


if __name__ == "__main__":
    main()
