"""J-Link bench runner: validated firmware mailbox, live RTT, always stop on exit.

Requires the dedicated Vector_Mini_ST_HIL image and its matching symbols.json.
Uses a command mailbox while CPU runs; never writes motor/PI state.
If STOP fails, attempt hardware output disable and only then halt for diagnosis.
"""
import argparse
import ctypes
import json
import math
from pathlib import Path
import struct
import sys
import time
from servo_hil_emergency import disable_outputs
from motor_axis_record import decode as decode_axis_record

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/servo_hil_20260908'
sys.path.insert(0, str(OUT / '.deps'))
try:
    import pylink
except ImportError:
    pylink = None  # Offline tests can import the module without the bench dependency.


def main():
    if not __debug__:
        raise RuntimeError('Bench safety checks require Python without -O/-OO')
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--name', required=True)
    p.add_argument('--kp', type=float, default=8.0)
    p.add_argument('--kd', type=float, default=2.0)
    p.add_argument('--speed-kp', type=float, default=.5)
    p.add_argument('--speed-ki', type=float, default=1.0)
    p.add_argument('--max-speed-deg', type=float, default=45)
    p.add_argument('--targets', default='0,5,0,-5,0')
    p.add_argument('--seconds', type=float, default=6)
    p.add_argument('--axis-profile', type=Path,
                   help='Verify persisted roll/pitch record and enforce its narrower host travel limits')
    p.add_argument('--keep-parameters', action='store_true',
                   help='Verify expected RAM gains and run without writing tuning parameters')
    args = p.parse_args()
    if pylink is None:
        p.error('Install pylink-square in the bench Python environment; no probe was opened')
    targets = [float(x) for x in args.targets.split(',')]
    axis_profile = json.loads(args.axis_profile.read_text(encoding='utf-8')) if args.axis_profile else None
    target_low, target_high, stop_low, stop_high, stop_current = -85.0, 85.0, -88.0, 88.0, 4.0
    if axis_profile:
        motion = axis_profile['motion']
        margin = motion['target_margin_deg']
        assert math.isfinite(margin) and margin > 0
        target_low = motion['minimum_deg'] + margin
        target_high = motion['maximum_deg'] - margin
        assert -85 <= target_low < target_high <= 85
        stop_low = max(-88, motion['minimum_deg'] + min(2, margin / 2))
        stop_high = min(88, motion['maximum_deg'] - min(2, margin / 2))
        assert stop_low < target_low < target_high < stop_high
        stop_current = min(4.0, axis_profile['acceptance']['maximum_iq_A'])
        assert math.isfinite(stop_current) and stop_current > 0
        assert args.max_speed_deg <= motion['cruise_deg_s'] <= 45
    assert all(target_low <= x <= target_high for x in targets)
    assert all(math.isfinite(x) and abs(x) <= 85 for x in targets)
    assert math.isfinite(args.max_speed_deg) and 0 < args.max_speed_deg <= 45
    assert 1 <= args.seconds <= 20
    destination = OUT / args.name
    destination.mkdir(exist_ok=False)
    symbols = json.loads((OUT / 'symbols.json').read_text())
    base = symbols['servo_hil_mailbox']
    desc = symbols['_SEGGER_RTT'] + 24 + 24
    events, polls = [], []
    j = pylink.JLink(lib=pylink.library.Library(
        dllpath='C:/Program Files/SEGGER/JLink_V964/JLink_x64.dll'))
    seq = 0
    heartbeat = 100
    beginning = time.monotonic()
    last_hb = -1
    armed = False
    raw = bytearray()
    runtime_parameters = {}
    last_tick = None
    last_tick_time = time.monotonic()

    def snapshot():
        b = struct.pack('<14I', *j.memory_read32(base, 14))
        x = struct.unpack('<III f IIIII fff II', b)
        assert x[0] == 0x48494c31, 'wrong firmware mailbox'
        return dict(ack=x[5], result=x[6], active=x[7], tick=x[8],
                    position=x[9], speed=x[10], iq=x[11], mode=x[12], error=x[13])

    def beat():
        nonlocal heartbeat, last_hb
        if time.monotonic() - last_hb >= .08:
            heartbeat += 1
            j.memory_write32(base + 16, [heartbeat])
            last_hb = time.monotonic()

    def drain():
        raw.extend(j.rtt_read(1, 8192))

    def command(op, value=0):
        nonlocal seq
        beat()
        seq += 1
        j.memory_write32(base + 8, [op, struct.unpack('<I',struct.pack('<f',value))[0]])
        j.memory_write32(base + 4, [seq])
        deadline = time.monotonic() + .25
        while time.monotonic() < deadline:
            x = snapshot()
            if x['ack'] == seq:
                events.append(dict(time=time.monotonic()-beginning, frame=len(raw)//24,
                                   opcode=op, value=value, **x))
                assert x['result'] == 0, ('command rejected', op, x)
                return x
            time.sleep(.002)
        raise RuntimeError('command acknowledgement timeout')

    def collect(duration):
        nonlocal last_tick, last_tick_time
        end = time.monotonic() + duration
        while time.monotonic() < end:
            beat()
            drain()
            x = snapshot()
            polls.append(dict(time=time.monotonic()-beginning, frame=len(raw)//24, **x))
            now = time.monotonic()
            if x['tick'] != last_tick:
                last_tick, last_tick_time = x['tick'], now
            assert now - last_tick_time < .1, ('control tick stalled for 100 ms', x)
            assert not x['error'] and x['active'] and x['mode'] == 3, ('stopped/fault', x)
            assert math.radians(stop_low) < x['position'] < math.radians(stop_high), ('host position stop', x)
            assert abs(x['speed']) < math.radians(85), ('host speed stop', x)
            assert abs(x['iq']) < stop_current, ('host current stop', x)
            time.sleep(.006)

    failure = None
    timing = {}
    shutdown_error = None
    shutdown_verified = False
    try:
        ctypes.windll.winmm.timeBeginPeriod(1)
        j.open(602722271)
        j.set_tif(pylink.enums.JLinkInterfaces.SWD)
        j.connect('STM32G431CB', speed=4000)
        assert not j.halted()
        seq = j.memory_read32(base + 4, 1)[0]
        command(1)
        time.sleep(.02)
        x = snapshot()
        assert x['mode'] == 0 and not x['error']
        assert abs(x['position']) <= math.radians(85)
        specs = [('cascade_pos_Kp',4,args.kp), ('cascade_pos_Kd',8,args.kd),
                 ('speed_Kp',5,args.speed_kp), ('speed_Ki',6,args.speed_ki),
                 ('pos_maxspeed',7,math.radians(args.max_speed_deg))]
        if not args.keep_parameters:
            for _, op, value in specs:
                command(op, value)
        offsets = json.loads((OUT/'member_offsets.json').read_text())['MotorControl_TypeDef']
        if axis_profile:
            motor_address = symbols['MotorControl']
            assert j.memory_read8(motor_address + offsets['axis_profile_valid'], 1)[0] == 1
            axis = decode_axis_record(bytes(j.memory_read8(motor_address + offsets['axis_profile'], 32)))
            assert axis['name'] == axis_profile['name'], ('wrong persisted motor axis', axis)
            for actual_key, expected_key in [('minimum_deg', 'minimum_deg'),
                                             ('maximum_deg', 'maximum_deg'),
                                             ('maximum_speed_deg_s', 'cruise_deg_s')]:
                assert math.isclose(axis[actual_key], motion[expected_key], abs_tol=1e-4), ('wrong axis envelope', axis)
            assert math.radians(target_low) <= x['position'] <= math.radians(target_high)
            for key, expected in [('current_limit', axis_profile['runtime']['current_limit_A']),
                                  ('posAcc', math.radians(motion['acceleration_deg_s2'])),
                                  ('posDec', math.radians(motion['stored_deceleration_deg_s2']))]:
                actual = struct.unpack('<f', bytes(j.memory_read8(motor_address + offsets[key], 4)))[0]
                assert math.isclose(actual, expected, abs_tol=1e-6), ('unexpected axis parameter', key, actual, expected)
        for key, _, value in specs:
            bits = j.memory_read32(symbols['MotorControl']+offsets[key],1)[0]
            actual = struct.unpack('<f',struct.pack('<I',bits))[0]
            assert math.isclose(actual,value,abs_tol=1e-6), ('unexpected gain',key,actual,value)
            runtime_parameters[key] = actual
        # Flush stale bytes to the producer's complete-frame boundary.
        wr = j.memory_read32(desc + 12, 1)[0]
        j.memory_write32(desc + 16, [wr])
        j.rtt_start(symbols['_SEGGER_RTT'])
        time.sleep(.05)
        assert j.rtt_get_num_up_buffers() >= 2
        for _ in range(50):
            j.rtt_read(1,8192)
            time.sleep(.005)
        beginning = time.monotonic()
        if 'hil_irq_histogram' in symbols:
            j.memory_write32(symbols['hil_irq_histogram'],[0,0,0,0])
            j.memory_write32(symbols['hil_irq_max_cycles'],[0])
        command(2)
        armed = True
        collect(.5)
        if 'hil_irq_histogram' in symbols:
            timing['startup_max_cycles'] = j.memory_read32(symbols['hil_irq_max_cycles'],1)[0]
            j.memory_write32(symbols['hil_irq_histogram'],[0,0,0,0])
            j.memory_write32(symbols['hil_irq_max_cycles'],[0])
        for target in targets:
            command(3, math.radians(target))
            print('target', target, 'deg', flush=True)
            collect(args.seconds)
        if 'hil_irq_histogram' in symbols:
            timing.update(max_cycles=j.memory_read32(symbols['hil_irq_max_cycles'],1)[0],
                duration_bins_50us=j.memory_read32(symbols['hil_irq_histogram'],4),
                scope='commanded moves and hold, read before STOP; ARM tracked separately')
        print('trial complete', len(raw)//24, 'frames', flush=True)
    except BaseException as exc:
        failure = repr(exc)
        print('STOP:', failure, flush=True)
        raise
    finally:
        try:
            if j.opened():
                command(1)
                time.sleep(.03)
                final = snapshot()
                assert final['mode'] == 0 and not final['active']
                assert j.memory_read32(0x40012c20,1)[0] & 0x555 == 0
                shutdown_verified = True
                if 'hil_irq_histogram' in symbols:
                    timing['including_stop_max_cycles'] = j.memory_read32(symbols['hil_irq_max_cycles'],1)[0]
                print('verified disabled at', round(math.degrees(final['position']),3), 'deg', flush=True)
        except BaseException as exc:
            shutdown_error = repr(exc)
            print('STOP NOT VERIFIED: disconnect motor power; ' + shutdown_error, flush=True)
            try:
                emergency = disable_outputs(j)
                (destination/'emergency.json').write_text(json.dumps(emergency,indent=2))
                print('Emergency phase disable verified; CPU halted for diagnosis', flush=True)
                (destination/'fault_ram.bin').write_bytes(bytes(j.memory_read8(0x20000000,0x8000)))
                (destination/'fault_scb.json').write_text(json.dumps(j.memory_read32(0xe000ed00,16)))
            except BaseException as emergency_exc:
                shutdown_error += '; emergency: ' + repr(emergency_exc)
        finally:
            try:
                j.rtt_stop()
            except BaseException as exc:
                shutdown_error = shutdown_error or repr(exc)
            try:
                j.close()
            except BaseException as exc:
                shutdown_error = shutdown_error or repr(exc)
        ctypes.windll.winmm.timeEndPeriod(1)
        (destination/'capture.bin').write_bytes(raw)
        (destination/'trial.json').write_text(json.dumps(dict(
            arguments={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
            events=events, polls=polls, failure=failure,
            shutdown_error=shutdown_error, shutdown_verified=shutdown_verified,
            timing=timing,
            runtime_parameters=runtime_parameters,
            image=json.loads((OUT/'active_image.json').read_text())
                if (OUT/'active_image.json').exists() else None,
            frame_bytes=24, nominal_sample_rate_hz=2000),indent=2))
        if len(raw)%24 == 0:
            with (destination/'capture.tsv').open('w') as f:
                f.write('\t'.join(f'rtt_channel1.data{i}' for i in range(12))+'\n')
                for row in struct.iter_unpack('<12h',raw):
                    f.write('\t'.join(map(str,row))+'\n')
        if shutdown_error and failure is None:
            raise RuntimeError('Shutdown/connection cleanup failed: ' + shutdown_error)

if __name__ == '__main__':
    main()
