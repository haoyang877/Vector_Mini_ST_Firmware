"""整轮生产检测的高速母线输入功率采样和判定。"""

import math
from itertools import pairwise
from pathlib import Path
from queue import SimpleQueue
from statistics import fmean

from can_motor_operations import OutputSpeedTest, SpeedSettling, run_speed_control
from high_speed_curves import save_high_speed_curves
from inspection_angle_trace import angle_preview, rotor_continuity

MOTOR_MAX_RPM = {3: 300, 4: 300, 5: 300, 6: 2500, 7: 2500}
OUTPUT_MAX_RPM = {3: 300, 4: 300, 5: 300, 6: 62.5, 7: 62.5}
SUGGESTED_OUTPUT_RPM = {3: 200, 4: 200, 5: 200, 6: 50, 7: 50}
SETTLE_HOLD_S = 0.5
SETTLE_TIMEOUT_S = 10.0
RECORD_S = 10.0
MAX_GAP_S = 0.2
SPEED_ERROR_FRACTION = 0.05
DEFAULT_POWER_MIN_W = -1000.0
DEFAULT_POWER_MAX_W = 1000.0


def power_check_fields(node: int, *, rpm: str = "", min_w: str = "", max_w: str = "") -> dict:
    """按轴预设输出转速，功率边界缺省为初始采集范围。"""
    limit = OUTPUT_MAX_RPM.get(node)
    if limit is None:
        raise ValueError("该轴尚未配置高速试验")
    try:
        output_rpm = float(rpm.strip() or str(SUGGESTED_OUTPUT_RPM[node]))
        power_min_w = float(min_w.strip() or str(DEFAULT_POWER_MIN_W))
        power_max_w = float(max_w.strip() or str(DEFAULT_POWER_MAX_W))
    except ValueError as error:
        raise ValueError("请填写有效的高速目标和功率上下限") from error
    if not math.isfinite(output_rpm) or output_rpm <= 0:
        raise ValueError("高速目标须为有限正数")
    if not (
        math.isfinite(power_min_w) and math.isfinite(power_max_w) and power_min_w < power_max_w
    ):
        raise ValueError("功率上下限须为有限数值，且下限小于上限")
    if output_rpm > limit:
        raise ValueError(f"高速目标超出该轴的输出轴上限（{limit:g} rpm）")
    return dict(
        output_rpm=output_rpm,
        power_min_w=power_min_w,
        power_max_w=power_max_w,
    )


def steady_feedback(feedback: list[dict], window: tuple[float, float]) -> list[dict]:
    start, end = window
    return [row for row in feedback if start <= row["elapsed_s"] <= end]


def window_covered(steady: list[dict], window: tuple[float, float]) -> bool:
    times = [row["elapsed_s"] for row in steady]
    start, end = window
    return (
        len(times) >= 2
        and times[0] - start <= MAX_GAP_S
        and end - times[-1] <= MAX_GAP_S
        and all(0 < b - a <= MAX_GAP_S for a, b in pairwise(times))
    )


def power_window(
    feedback: list[dict],
    output_scale: float,
    target_rpm: float,
    *,
    window: tuple[float, float],
    min_w: float,
    max_w: float,
) -> dict:
    """从同一个时间窗分别判定输出轴转速与母线输入功率。"""
    steady = steady_feedback(feedback, window)
    covered = window_covered(steady, window)
    speeds = [
        row["actual_rad_s"] * output_scale * 60 / math.tau
        for row in steady
        if row["actual_rad_s"] is not None
    ]
    speed = dict(
        status="incomplete", samples=len(speeds), reason="稳速区间的转速反馈或时间覆盖不足"
    )
    if speeds:
        max_error = max(abs(value - target_rpm) / target_rpm for value in speeds)
        speed.update(min_rpm=min(speeds), max_rpm=max(speeds), max_error_pct=100 * max_error)
        if covered and len(speeds) == len(steady):
            if max_error > SPEED_ERROR_FRACTION and not math.isclose(
                max_error, SPEED_ERROR_FRACTION, abs_tol=1e-12, rel_tol=0
            ):
                speed.update(status="failed", reason="存在超出目标 ±5% 的输出轴转速样本")
            else:
                speed.update(status="passed", reason="")
    power = [
        row["bus_voltage_v"] * row["bus_current_a"]
        for row in steady
        if row["bus_voltage_v"] is not None and row["bus_current_a"] is not None
    ]
    result = dict(samples=len(power), window_s=RECORD_S, speed_stability=speed)
    if power:
        result.update(
            mean_w=fmean(power),
            min_w=min(power),
            max_w=max(power),
            peak_to_peak_w=max(power) - min(power),
        )
    if not covered or len(power) != len(steady):
        return dict(result, status="incomplete", reason="稳速区间的母线反馈或时间覆盖不足")
    if speed["status"] != "passed":
        return dict(result, status="incomplete", reason="本轮未取得符合转速要求的功率窗口")
    violations = []
    if result["min_w"] < min_w:
        violations.append("最小功率低于下限")
    if result["max_w"] > max_w:
        violations.append("最大功率超过上限")
    if violations:
        return dict(result, status="failed", reason="、".join(violations))
    return dict(result, status="passed")


def run_power_check(
    bus,
    settings: dict,
    out: Path,
    *,
    saved_revision: int,
    stopped,
    progress,
    angle_sample,
    hall_fault: int = 0,
) -> dict:
    """复用现有速度会话和关相回读，保存稳速前后的原始遥测。"""
    config = settings["power_check"]
    work = out / "high-speed"
    work.mkdir()
    samples = []

    def collect(sample: dict) -> None:
        samples.append(sample)
        angle_sample(angle_preview(sample, samples[0]["position_rad"]))

    run = run_speed_control(
        bus,
        settings["node"],
        config["output_rpm"] * math.tau / 60,
        None,
        SimpleQueue(),
        work,
        expected_uid=settings["expected_uid"],
        output_test=OutputSpeedTest(
            RECORD_S,
            MOTOR_MAX_RPM[settings["node"]] * math.tau / 60,
            saved_revision,
            hall_fault=hall_fault,
            settling=SpeedSettling(
                SETTLE_HOLD_S, SETTLE_TIMEOUT_S, SPEED_ERROR_FRACTION, MAX_GAP_S
            ),
        ),
        previous=settings["last_lease"],
        previous_target=settings["last_target"],
        stopped=stopped,
        progress=progress,
        feedback=collect,
    )
    evidence = dict(**config, run=run, stop_confirmed=run.get("stop_confirmed"))
    window = run.get("steady_window_s")
    if window is not None:
        evidence["steady_window_s"] = window
    if (work / "speed.csv").is_file():
        evidence["csv"] = "high-speed/speed.csv"
    evidence.update(
        save_high_speed_curves(
            samples,
            work,
            config,
            window=window,
            max_gap_s=MAX_GAP_S,
            speed_error_fraction=SPEED_ERROR_FRACTION,
        )
    )
    if stopped():
        return dict(evidence, status="cancelled", reason="高速测试已停止")
    if (
        run.get("settling_timed_out")
        and run.get("stop_confirmed")
        and run.get("communication_reliability_passed")
    ):
        return dict(
            evidence,
            status="not_measured",
            reason="本轮未进入稳速区，未取得功率判定窗口",
            speed_stability=dict(status="failed", reason=run["fatal"]),
        )
    if not (
        run.get("passed")
        and run.get("stop_confirmed")
        and run.get("communication_reliability_passed")
    ):
        return dict(evidence, status="failed", reason=run.get("fatal") or "高速测试或关相确认失败")
    if window is None:
        return dict(evidence, status="incomplete", reason="本轮未取得稳速判定窗口")
    steady = steady_feedback(samples, window)
    evidence["rotor_continuity"] = rotor_continuity(
        steady, covered=window_covered(steady, window), max_gap_s=MAX_GAP_S
    )
    output_scale = 1.0 if run["motion_coordinate"] == "output_shaft" else 1 / run["gear_ratio"]
    return dict(
        evidence,
        **power_window(
            samples,
            output_scale,
            config["output_rpm"],
            window=window,
            min_w=config["power_min_w"],
            max_w=config["power_max_w"],
        ),
    )
