"""Measure pre-capture slowdowns from actual mode-3 HIL data, not a plant model.

Near target: 0.3 < |target - encoder| < 5 deg, after speed first exceeds 5 deg/s.
Slow: |filtered speed| < 2 deg/s. Report contiguous runs and subsequent speed
to distinguish a slowdown/restart from a single continuous final deceleration.
Durations use host receive anchors, with the same batching limits as HIL reports.
"""
import json
from pathlib import Path
import sys
import numpy as np
from rtt_control_frame import decode


def analyze(path):
    trial = json.loads((path/'trial.json').read_text())
    data = np.loadtxt(path/'capture.tsv', skiprows=1)
    if decode(data)['version'] != 1:
        raise ValueError('Historical v1 report only; use analyze_positioning_comparison.py for v2')
    flags = data[:,11].astype(int) & 65535
    position = np.unwrap(data[:,1] * np.pi / 32768) * 180 / np.pi
    speed = np.abs(data[:,5]) * 180 / np.pi / 10000
    polls = trial['polls']
    time = np.interp(np.arange(len(data)), [p['frame'] for p in polls],
                     [p['time'] for p in polls])
    events = [e for e in trial['events'] if e['opcode'] == 3]
    moves = []
    for i, event in enumerate(events):
        a = event['frame']
        b = events[i+1]['frame'] if i+1 < len(events) else len(data)
        target = np.degrees(event['value'])
        direction = np.sign(target-position[a])
        error = np.abs(target-position[a:b])
        moving = np.flatnonzero(speed[a:b] > 5)
        near = (error > .3) & (error < 5) & ((flags[a:b] & 128) != 0)
        near[:moving[0]+1 if len(moving) else len(near)] = False
        low = near & (speed[a:b] < 2)
        indexes = np.flatnonzero(low)
        groups = np.split(indexes, np.flatnonzero(np.diff(indexes)>1)+1)
        runs = []
        for group in groups:
            if not len(group):
                continue
            first, last = int(group[0]), int(group[-1])
            duration = float(time[a+last] - time[a+first])
            if duration >= .05:
                future = speed[a:b][near & (np.arange(b-a) > last)]
                runs.append(dict(start_s=float(time[a+first]-event['time']),
                                 duration_s=duration,
                                 start_error_deg=float(error[first]),
                                 end_error_deg=float(error[last]),
                                 start_signed_remaining_deg=float(direction*(target-position[a+first])),
                                 end_signed_remaining_deg=float(direction*(target-position[a+last])),
                                 later_near_speed_peak_deg_s=float(future.max()) if len(future) else 0))
        recovery = near & ((flags[a:b] & 1024) != 0)
        dt = np.maximum(0, np.diff(time[a:b], prepend=time[a]))
        moves.append(dict(target_deg=float(target),
            slow_runs=runs, max_slow_run_s=max([r['duration_s'] for r in runs]+[0]),
            recovery_duration_s=float(dt[recovery].sum()),
            recovery_ff_peak_A=float(np.max(np.abs(data[a:b,9][recovery]))/1000) if recovery.any() else 0))
    result = dict(trial=path.name, definition=__doc__, moves=moves)
    (path/'landing.json').write_text(json.dumps(result,indent=2))
    return result


if __name__ == '__main__':
    for value in sys.argv[1:]:
        print(json.dumps(analyze(Path(value)),indent=2))
