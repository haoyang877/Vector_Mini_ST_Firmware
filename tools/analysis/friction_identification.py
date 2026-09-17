"""Offline friction-fit analysis for previously captured samples; no device I/O."""

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import asdict, dataclass
from pathlib import Path


class ProtocolError(RuntimeError):
    pass


@dataclass
class Sample:
    index: int
    target_rps: float
    speed_rps: float
    iq_a: float
    sample_count: int


@dataclass
class Fit:
    coulomb_a: float
    viscous_a_per_rad_s: float
    rmse_a: float


def extract_number(response: str, field: str) -> float:
    match = re.search(rf"{re.escape(field)}=(-?\d+(?:\.\d+)?)", response)
    if not match:
        raise ProtocolError(f"Cannot parse {field!r} from: {response}")
    return float(match.group(1))


def parse_status(response: str) -> dict[str, float]:
    return {
        field: extract_number(response, field)
        for field in ("friction_state", "reason", "point", "progress", "candidate")
    }


def constrained_fit(samples: list[Sample], positive: bool) -> Fit:
    selected = [sample for sample in samples if (sample.speed_rps > 0.0) == positive]
    if len(selected) < 2:
        raise ProtocolError("Not enough directional samples for regression")
    x = [abs(sample.speed_rps) * 2.0 * math.pi for sample in selected]
    y = [sample.iq_a if positive else -sample.iq_a for sample in selected]
    if any(value <= 0.0 or not math.isfinite(value) for value in x + y):
        raise ProtocolError("The no-load current does not oppose motion in every sample")
    count = float(len(x))
    sum_x, sum_y = sum(x), sum(y)
    sum_xx = sum(value * value for value in x)
    sum_xy = sum(xv * yv for xv, yv in zip(x, y))
    denominator = count * sum_xx - sum_x * sum_x
    if denominator <= 1e-12:
        raise ProtocolError("Speed points are degenerate")
    slope = (count * sum_xy - sum_x * sum_y) / denominator
    intercept = (sum_y - slope * sum_x) / count
    if slope < 0.0:
        slope = 0.0
        intercept = sum_y / count
    if intercept < 0.0:
        intercept = 0.0
        slope = sum_xy / sum_xx
    rmse = math.sqrt(
        sum((yv - (intercept + slope * xv)) ** 2 for xv, yv in zip(x, y)) / count
    )
    return Fit(intercept, slope, rmse)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('samples', type=Path, help='JSON array of Sample records, or an archived report containing samples')
    args = parser.parse_args()
    data = json.loads(args.samples.read_text(encoding='utf-8'))
    records = data['samples'] if isinstance(data, dict) else data
    samples = [Sample(**item) for item in records]
    print(json.dumps({label: asdict(constrained_fit(samples, positive))
                      for label, positive in [('positive', True), ('negative', False)]}, indent=2))


if __name__ == '__main__':
    main()
