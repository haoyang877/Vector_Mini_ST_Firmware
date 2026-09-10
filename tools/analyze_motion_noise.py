"""Windowed moving-stage spectra from 2 kHz mode-3 RTT captures.

Closed-loop correlation does not distinguish encoder noise from mechanical motion.
Band RMS is not an acoustic measurement; sample timing assumes nominal 2 kHz.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def analyze(path):
    trial = json.loads((path / 'trial.json').read_text())
    data = np.loadtxt(path / 'capture.tsv', skiprows=1, ndmin=2)
    if (data.shape[1] != 12 or not np.isfinite(data).all() or
            np.any(data != np.trunc(data)) or np.any(data < -32768) or np.any(data > 32767)):
        raise ValueError('Expected finite signed-int16 12-channel RTT data')
    events = [e for e in trial['events'] if e['opcode'] == 3]
    length, rate = 1024, 2000.0
    window = np.hanning(length)
    frequencies = np.fft.rfftfreq(length, 1 / rate)
    scale = 2 / (rate * np.sum(window**2))
    powers, cross = [], []
    for i, event in enumerate(events):
        stop = events[i + 1]['frame'] if i + 1 < len(events) else len(data)
        for start in range(event['frame'], stop - length + 1, length // 4):
            segment = data[start:start + length]
            flags = segment[:, 11].astype(np.int64) & 65535
            # Entire window in valid MOVE, no trip/drop/saturation; planned
            # speed > 0.08 rad/s excludes the static breakaway/landing region.
            valid = ((flags & 0x180) == 0x180) & ((flags & 0x6b) == 0)
            valid &= np.abs(segment[:, 3]) > 800
            valid &= np.all((segment[:, :11] > -32768) & (segment[:, :11] < 32767), axis=1)
            if not np.all(valid):
                continue
            signals = np.column_stack((segment[:, 5] / 10000,
                segment[:, 6] / 1000, segment[:, 7] / 1000,
                (segment[:, 7] - segment[:, 6]) / 1000, segment[:, 10] / 1000))
            spectrum = np.fft.rfft((signals - signals.mean(axis=0)) * window[:, None], axis=0)
            powers.append(np.abs(spectrum)**2 * scale)
            cross.append(spectrum[:, 0] * np.conj(spectrum[:, 1]) * scale)
    if not powers:
        raise ValueError('No complete eligible MOVE windows')
    power, cross_power = np.mean(powers, axis=0), np.mean(cross, axis=0)
    bands = []
    for low, high in ((20, 80), (80, 300), (300, 800), (40, 800)):
        mask = (frequencies >= low) & (frequencies < high)
        total = power[mask].sum(axis=0)
        rms = np.sqrt(total * rate / length)
        denominator = np.sqrt(total[0] * total[1])
        bands.append(dict(low_hz=low, high_hz=high, speed_rms_rad_s=float(rms[0]),
            iq_reference_rms_A=float(rms[1]), iq_feedback_rms_A=float(rms[2]),
            iq_tracking_rms_A=float(rms[3]), integral_rms_A=float(rms[4]),
            speed_iq_reference_band_correlation=float(cross_power[mask].real.sum() / denominator)
                if denominator > 0 else None))
    result = dict(trial=path.name, windows=len(powers), window_samples=length,
        window_overlap_fraction=.75, nominal_sample_rate_hz=rate,
        runtime_parameters=trial['runtime_parameters'], image=trial.get('image'),
        failure=trial['failure'], shutdown_verified=trial['shutdown_verified'],
        bands=bands, limitations=__doc__)
    (path / 'motion_noise_summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trials', type=Path, nargs='+')
    for directory in parser.parse_args().trials:
        print(json.dumps(analyze(directory), indent=2))
