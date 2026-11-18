"""用本轮定时速度会话的实际轴位置生成曲线及连续性证据。"""

import math
from itertools import pairwise


def angle_preview(row: dict, origin_rad: float) -> dict:
    """连续绘出实际位置，保留固件反馈的轴坐标。"""
    return dict(
        elapsed_s=row["elapsed_s"],
        reference_source="motor_encoder"
        if row["motion_coordinate"] == "motor_shaft"
        else "output_tracking",
        reference_deg=math.degrees(row["position_rad"] - origin_rad),
    )


def rotor_continuity(samples: list[dict], *, covered: bool, max_gap_s: float) -> dict:
    """按相邻真实电机轴角与同一坐标的速度目标检查跳变。"""
    if not samples:
        return dict(status="not_measured", reason="未取得稳速段角度采样")
    if samples[0]["motion_coordinate"] != "motor_shaft":
        return dict(status="not_measured", reason="108 提供输出跟踪角，缺少电机轴角样本")
    jumps = gaps = pairs = 0
    for before, after in pairwise(samples):
        elapsed_s = after["elapsed_s"] - before["elapsed_s"]
        first, last = before["position_rad"], after["position_rad"]
        if first is None or last is None or not 0 < elapsed_s <= max_gap_s:
            gaps += 1
            continue
        pairs += 1
        step_limit_deg = max(1.0, 3 * math.degrees(abs(after["target_rad_s"])) * elapsed_s + 0.5)
        jumps += abs(math.degrees(last - first)) > step_limit_deg
    start, end = samples[0]["position_rad"], samples[-1]["position_rad"]
    moved = start is not None and end is not None and start != end
    status = (
        "jump" if jumps else "incomplete" if gaps or not (covered and pairs and moved) else "clear"
    )
    return dict(status=status, pairs=pairs, jumps=jumps, gaps=gaps, max_gap_s=max_gap_s)
