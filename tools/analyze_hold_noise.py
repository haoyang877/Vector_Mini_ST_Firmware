"""Summarize contiguous mode-3 RTT HOLD intervals; never connect to hardware.

Input: the documented 12-column raw RTT TSV. AC RMS is standard deviation
after subtracting the mean. Correlation cannot distinguish sensor noise from
real mechanical motion in a closed loop, or establish acoustic noise level.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from rtt_control_frame import decode


def analyze(path, minimum_samples=2000, tail_samples=4000):
    if minimum_samples < 2 or tail_samples < 2:
        raise ValueError("Sample counts must be at least two")
    data = np.loadtxt(path, skiprows=1, ndmin=2)
    decoded = decode(data)
    status = decoded['flags']
    valid = decoded['valid'] & ((status & 7) == 6) & ((status & 0x68) == 0)
    valid &= ~decoded['encoding_saturated']
    edges = np.flatnonzero(np.diff(np.r_[False, valid, False]))
    intervals = []
    for start, stop in zip(edges[::2], edges[1::2]):
        if stop - start < minimum_samples:
            continue
        first = max(start, stop - tail_samples)
        tail = data[first:stop]
        velocity = decoded['speed_feedback_rad_s'][first:stop]
        reference = decoded['iq_reference_A'][first:stop]
        feedback = decoded['iq_feedback_A'][first:stop]
        correlation = (float(np.corrcoef(velocity, reference)[0, 1])
                       if velocity.std() > 0 and reference.std() > 0 else None)
        intervals.append({
            "first_sample_zero_based": int(first), "stop_sample_exclusive": int(stop),
            "samples": len(tail),
            "position_error_peak_to_peak_deg": float(np.ptp(decoded['error_deg'][first:stop])),
            "speed_ac_rms_rad_s": float(velocity.std()),
            "iq_reference_ac_rms_A": float(reference.std()),
            "iq_feedback_ac_rms_A": float(feedback.std()),
            "iq_tracking_rms_A": float(np.sqrt(np.mean((feedback - reference) ** 2))),
            "velocity_iq_reference_correlation": correlation,
            "hold_integral_ac_rms_A": float(decoded['integral_A'][first:stop].std()),
            "feedforward_peak_A": float(np.max(np.abs(decoded['feedforward_A'][first:stop]))),
        })
    return {"source": str(Path(path).resolve()),
            "sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest(),
            "total_samples": len(data), "intervals": intervals,
            "limitations": "Historical sensor/current data, not acoustic measurement. "
            "No per-frame timestamps; reported gaps excluded, undetected gaps remain possible. "
            "Feedback semantics depend on firmware version. Correlation is not causal proof."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("captures", type=Path, nargs="+")
    parser.add_argument("--minimum-samples", type=int, default=2000)
    parser.add_argument("--tail-samples", type=int, default=4000)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    result = [analyze(p, args.minimum_samples, args.tail_samples) for p in args.captures]
    encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(encoded + "\n", encoding="utf-8")
    else:
        print(encoded)


if __name__ == "__main__":
    main()
