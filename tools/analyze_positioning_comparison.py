"""Positioning metrics from real 12-channel HIL captures and command events.

Times use host receipt anchors and include RTT batching uncertainty. A target
already inside a tolerance may have a zero settling time without actual motion.
No external torque is measured; these results do not establish load stiffness.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def settle_time(times, error, tolerance, command_time):
    outside = np.flatnonzero(np.abs(error) > tolerance)
    if not len(outside):
        return 0.0
    first = int(outside[-1]) + 1
    if first >= len(error):
        return None
    return float(max(0, times[first] - command_time))


def analyze(path):
    trial = json.loads((path / 'trial.json').read_text())
    data = np.loadtxt(path / 'capture.tsv', skiprows=1, ndmin=2)
    if data.shape[1] != 12 or not np.isfinite(data).all():
        raise ValueError('Expected finite 12-channel RTT data')
    polls = trial['polls']
    times = np.interp(np.arange(len(data)), [p['frame'] for p in polls],
                      [p['time'] for p in polls])
    position = np.unwrap(data[:, 1] * np.pi / 32768) * 180 / np.pi
    reference = np.unwrap(data[:, 0] * np.pi / 32768) * 180 / np.pi
    flags = data[:, 11].astype(np.int64) & 65535
    commands = [e for e in trial['events'] if e['opcode'] == 3]
    moves = []
    for i, event in enumerate(commands):
        start = event['frame']
        end = commands[i + 1]['frame'] if i + 1 < len(commands) else len(data)
        if not 0 <= start < end <= len(data):
            raise ValueError('Missing samples for a position command')
        indexes = np.arange(start, end)
        target = float(np.degrees(event['value']))
        error = target - position[start:end]
        direction = np.sign(target - position[start])
        tail = indexes[times[start:end] >= times[end - 1] - 2]
        tail_error = target - position[tail]
        held = ((flags[start:end] & 0x87) == 0x86) & (np.abs(reference[start:end] - target) <= .02)
        first_hold = np.flatnonzero(held & (times[start:end] >= event['time']))
        moves.append(dict(
            target_deg=target, precondition=i == 0,
            commanded_step_deg=None if i == 0 else target - float(np.degrees(commands[i - 1]['value'])),
            actual_start_deg=float(position[start]),
            actual_end_deg=float(np.median(position[tail])),
            settle_0p3_s=settle_time(times[start:end], error, .3, event['time']),
            settle_0p1_s=settle_time(times[start:end], error, .1, event['time']),
            first_hold_s=float(max(0, times[start + first_hold[0]] - event['time'])) if len(first_hold) else None,
            overshoot_deg=float(max(0, np.max(direction * (position[start:end] - target)))),
            tail_error_mean_deg=float(np.mean(tail_error)),
            tail_error_abs_max_deg=float(np.max(np.abs(tail_error))),
            tail_position_pp_deg=float(np.ptp(position[tail])),
            tail_hold_fraction=float(np.mean((flags[tail] & 0x87) == 0x86)),
            tail_iq_ac_rms_A=float(data[tail, 7].std() / 1000),
            iq_peak_A=float(np.max(np.abs(data[start:end, 7])) / 1000),
            invalid_samples=int(np.count_nonzero((flags[start:end] & 128) == 0)),
            drop_flag_samples=int(np.count_nonzero(flags[start:end] & 64)),
            saturation_samples=int(np.count_nonzero(flags[start:end] & 8))))
    result = dict(trial=path.name, definition=__doc__,
                  setting=trial['runtime_parameters'].get('hold_filter_setting'),
                  failure=trial['failure'], shutdown_verified=trial['shutdown_verified'],
                  timing=trial['timing'], moves=moves)
    (path / 'positioning_summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', type=Path, nargs='+')
    args = parser.parse_args()
    for path in args.trials:
        result = analyze(path)
        print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
