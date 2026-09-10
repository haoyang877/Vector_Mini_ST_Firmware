"""Decode raw 12 x int16 RTT v1/v2 frames into named physical quantities.

Never rewrite captures or infer a version from signal values. V1 is bit8;
v2 is bit14, with bit8 clear. Unknown valid frames and mixed versions fail.
"""
import numpy as np


def decode(data):
    raw = np.asarray(data, dtype=float)
    if (raw.ndim != 2 or raw.shape[1] != 12 or not np.isfinite(raw).all() or
            np.any(raw != np.trunc(raw)) or np.any(raw < -32768) or np.any(raw > 32767)):
        raise ValueError('Expected finite raw signed-int16 12-channel RTT frames')
    flags = raw[:, 11].astype(np.int64) & 65535
    tag = flags & 0x4100
    known = np.isin(tag, [0x100, 0x4000])
    if np.any(((flags & 128) != 0) & ~known):
        raise ValueError('Unknown or conflicting RTT format version')
    versions = np.unique(tag[known])
    if len(versions) > 1:
        raise ValueError('Mixed RTT versions require separate captures')
    version = 2 if len(versions) and versions[0] == 0x4000 else 1
    valid = known & ((flags & 128) != 0)
    result = dict(version=version, flags=flags, valid=valid,
                  encoding_saturated=np.any((raw[:, :11] == -32768) |
                                             (raw[:, :11] == 32767), axis=1))
    if version == 2:
        result.update(target_deg=raw[:, 0] / 100, reference_deg=raw[:, 1] / 100,
                      position_deg=raw[:, 2] / 100, error_deg=raw[:, 3] / 100,
                      speed_reference_rad_s=np.radians(raw[:, 4] / 100),
                      speed_feedback_rad_s=np.radians(raw[:, 5] / 100),
                      iq_reference_A=raw[:, 6] / 1000, feedforward_A=raw[:, 7] / 1000,
                      iq_feedback_A=raw[:, 8] / 1000, pi_current_A=raw[:, 9] / 1000)
    else:
        result.update(target_deg=None,
                      reference_deg=np.degrees(np.unwrap(raw[:, 0] * np.pi / 32768)),
                      position_deg=np.degrees(np.unwrap(raw[:, 1] * np.pi / 32768)),
                      error_deg=np.degrees(raw[:, 2] / 10000),
                      speed_reference_rad_s=raw[:, 3] / 10000,
                      speed_feedback_rad_s=raw[:, 5] / 10000,
                      iq_reference_A=raw[:, 6] / 1000, feedforward_A=raw[:, 9] / 1000,
                      iq_feedback_A=raw[:, 7] / 1000, pi_current_A=raw[:, 8] / 1000)
    result['integral_A'] = raw[:, 10] / 1000
    return result
