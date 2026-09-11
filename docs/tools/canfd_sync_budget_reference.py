"""Proposal-only CAN FD vectors and error-free conservative timing model.

Does not modify the adopted protocol catalog or MCU firmware.
Bit field reference: Linux include/linux/can/length.h.
Model applies to ISO CAN FD extended frames, BRS, error-active nodes,
20..64 physical data bytes, and data rate >= nominal rate.
"""
from pathlib import Path
import json
import runpy
import struct

DOCS = Path(__file__).resolve().parent.parent
CODEC = runpy.run_path(str(DOCS / 'tools/protocol_vectors_reference.py'))


def frame_time_us(data_bytes, nominal_bps, data_bps=5_000_000):
    assert data_bytes in (20, 24, 32, 48, 64)
    assert 0 < nominal_bps <= data_bps
    # 36 bits through BRS; ceil(n/4) bounds stuffing including boundary carry.
    # Conservatively charge all of BRS and CRC delimiter at nominal rate.
    nominal_bits = 36 + (36 + 3) // 4 + 1 + 9 + 3
    data_dynamic = 5 + 8 * data_bytes  # ESI + DLC + physical data
    # CRC21 + stuff count/parity + fixed stuffing, excluding CRC delimiter.
    data_bits = data_dynamic + (data_dynamic + 3) // 4 + 32
    return 1e6 * (nominal_bits / nominal_bps + data_bits / data_bps)


def main():
    group = CODEC['fd_frames'](
        193, 100, struct.pack('<IHBBI5i', 0x22334455, 100, 5, 0,
                             1002000, 1000, 2000, 3000, 4000, 5000),
        flags=0, src=2, dst=255, priority=2)
    # Candidate 125 explicitly uses HEADER seq as applied_cycle_seq.
    feedback = CODEC['fd_frames'](
        125, 100, struct.pack('<IBBihh', 0x22334455, 0x33, 0, 1000, 1200, 300),
        flags=0, src=3, dst=2, priority=3)
    assert group[0]['fd_data_bytes'] == 64
    assert feedback[0]['logical_frame_bytes'] == 32
    assert feedback[0]['fd_data_bytes'] == 32
    raw = bytes.fromhex(feedback[0]['data'])
    assert struct.unpack_from('<H', raw, 8)[0] == 100
    assert struct.unpack_from('<IBBihh', raw, 16) == (
        0x22334455, 0x33, 0, 1000, 1200, 300)
    estimates = []
    for nominal in (500_000, 1_000_000):
        for axes_on_bus in (2, 3, 4, 5, 8):
            cyclic = frame_time_us(64, nominal) + axes_on_bus * frame_time_us(32, nominal)
            estimates.append({
                'nominal_bps': nominal, 'axes_on_bus': axes_on_bus,
                'cyclic_us_per_1ms': round(cyclic, 3),
                'cyclic_percent': round(cyclic / 10, 3),
                'with_clock_50Hz_two_frames_and_heartbeat_1Hz_percent': round(
                    cyclic / 10 + frame_time_us(32, nominal) * 100 / 10000
                    + axes_on_bus * frame_time_us(48, nominal) / 10000, 3)})
    result = {
        'status': 'proposal_not_adopted',
        'requirements': {'minimum_axes': 5, 'target_hz': 1000, 'per_axis_feedback_hz': 1000},
        'group_position': group, 'feedback': feedback,
        'feedback_header_seq_semantics': 'applied_cycle_seq; candidate 125 only',
        'timing_model': {
            'kind': 'conservative_error_free_engineering_model_not_measured',
            'reference': 'https://raw.githubusercontent.com/torvalds/linux/master/include/linux/can/length.h',
            'nominal_bits_bound': 58,
            'data_bits_bound': '5+8*N+ceil((5+8*N)/4)+32',
            'includes': ['ISO CRC21', 'dynamic/fixed stuffing bounds', 'ACK', 'EOF', 'intermission'],
            'excludes': ['error recovery/retries', 'software latency', 'extra idle gaps',
                         'background blocking', 'error-passive suspend'],
            'frame_time_us': {str(rate): {str(n): round(frame_time_us(n, rate), 3)
                                         for n in (32, 48, 64)}
                              for rate in (500_000, 1_000_000)}},
        'estimates': estimates}
    (DOCS / 'canfd_1khz_sync_proposal_vectors.json').write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result['timing_model']['frame_time_us']))
    print(json.dumps(estimates))


if __name__ == '__main__':
    main()
