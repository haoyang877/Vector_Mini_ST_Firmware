"""Bounded pitch/roll CAN bench cases; no Flash writes or gain changes.

Uses the inspected Chuangxin SDK adapter in the neighboring PC controller repo.
Each invocation runs one case and stops both axes before returning.
"""
import argparse
import csv
import ctypes
import hashlib
import json
import math
from pathlib import Path
import sys
import time

AXES = {'roll': {'node': 3, 'bounds': (-90.,90.), 'ki': 1.},
        'pitch': {'node': 4, 'bounds': (math.degrees(-.3),math.degrees(.9)), 'ki': 2.}}
FIELDS = {'position':0x40, 'speed':0x3e, 'iq':0x38, 'mode':0, 'fault':0x4c, 'accepted':6}


def sequence(case):
    if case == 'hold': return [(0.,0.,3.)]
    if case == 'small': return [(v,v,6.) for v in (1.,0.,-1.,0.)]
    if case == 'same': return [(v,v,6.) for v in (5.,0.,-5.,0.)*2]
    if case == 'opposite': return [(v,-v,6.) for v in (5.,0.,-5.,0.)]
    if case in ('sine01','sine02'): return []
    raise ValueError(case)


def targets(case, elapsed):
    if not math.isfinite(elapsed) or elapsed < 0: raise ValueError('invalid time')
    if case in ('sine01','sine02'):
        freq = .1 if case == 'sine01' else .2
        end = 2/freq
        if elapsed >= end+3: return None
        value = 5*math.sin(2*math.pi*freq*elapsed) if elapsed < end else 0.
        return ('sine' if elapsed < end else 'tail', {'roll':value,'pitch':value})
    edge = 0.
    for index,(r,p,duration) in enumerate(sequence(case)):
        edge += duration
        if elapsed < edge: return str(index), {'roll':r,'pitch':p}
    return None


def validate_centers(centers):
    if set(centers) != set(AXES): raise ValueError('two verified centers required')
    for axis,c in centers.items():
        lo,hi = AXES[axis]['bounds']
        if not math.isfinite(c) or c-5 < lo+5 or c+5 > hi-5:
            raise ValueError(f'{axis}: center +/-5 exceeds target envelope')


def validate_feedback(axis, values, reference, expected_mode):
    if any(not math.isfinite(v) for v in values.values()): raise RuntimeError('nonfinite feedback')
    if values['mode'] != expected_mode or values['fault'] != 0:
        raise RuntimeError(f'{axis}: mode/fault {values["mode"]}/{values["fault"]}')
    lo,hi = AXES[axis]['bounds']
    if not lo+2 < math.degrees(values['position']) < hi-2:
        raise RuntimeError(f'{axis}: actual position outside margin')
    if abs(math.degrees(values['speed'])) > 45 or abs(values['iq']) >= 4:
        raise RuntimeError(f'{axis}: host speed/current stop')
    if reference is not None and abs(values['accepted']*360-reference) > .02:
        raise RuntimeError(f'{axis}: accepted target differs')


def main():
    if not __debug__: raise RuntimeError('Do not run with Python -O')
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('case',choices=['preflight','hold','small','same','opposite','sine01','sine02'])
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--centers',type=Path,help='Verified preflight run.json')
    ap.add_argument('--adapter-root',type=Path,default=Path('D:/Work/Code/motor_ctrl_app'))
    args = ap.parse_args()
    if args.case != 'preflight' and not args.centers: ap.error('--centers required for motion')
    sys.path.insert(0,str(args.adapter_root.resolve()))
    from pc_replay.canfd import CanFD
    from pc_replay.protocol import Client
    args.out.mkdir(parents=True,exist_ok=False)
    report = {'case':args.case,'success':False,'motion_enabled':False,'stop_verified':{},
              'centers':{},'preflight':{},'failure':None,'restore_errors':[],
              'command_hz':20,'feedback_hz':20,'scheduling_limit_s':.05,
              'adapter_source_sha256':hashlib.sha256((args.adapter_root/'pc_replay/canfd.py').read_bytes()).hexdigest()}
    streams = {}
    writers = {}
    columns = {
        'frames':['time_s','direction','channel','id','data','device_us'],
        'commands':['time_s','case_time_s','cycle','segment','axis','target_deg','lateness_s'],
        'feedback':['time_s','case_time_s','cycle','segment','axis','target_deg','position_deg','speed_deg_s','iq_A','mode','fault','accepted_deg','error_deg','speed_time_s','iq_time_s','mode_time_s','fault_time_s','accepted_time_s','position_device_us']}
    for key,cols in columns.items():
        streams[key] = (args.out/(key+'.csv')).open('w',newline='',encoding='utf-8',buffering=1)
        writers[key] = csv.writer(streams[key]);writers[key].writerow(cols)
    epoch = time.perf_counter()
    reject_external = True
    last_tx = {}
    bus = None
    original_speeds = {}
    latest_targets = {}
    def trace(direction,ch,can_id,data,when,device):
        writers['frames'].writerow([when-epoch,direction,ch,hex(can_id),data.hex(),device])
        if direction == 'tx': last_tx[can_id] = when
        if reject_external and direction == 'rx' and not (can_id & 1):
            raise RuntimeError(f'External CAN command detected: {can_id:#05x} {data.hex()}')
    ctypes.windll.winmm.timeBeginPeriod(1)
    try:
        bus = CanFD(str(args.adapter_root/'.local/canfd-sdk/ControlCANFD.dll'),channels=(0,))
        report['adapter'] = bus.info
        c = Client(bus,timeout=.05,trace=trace)
        def read(axis,param):
            time.sleep(.001)
            return c.read(0,AXES[axis]['node'],param)
        def write(axis,param,value): c.write(0,AXES[axis]['node'],param,value)
        def verify(axis,param,wanted):
            actual = read(axis,param)[0]
            if not math.isclose(actual,wanted,abs_tol=1e-5):
                raise RuntimeError(f'{axis} parameter {param:#x}: {actual} != {wanted}')
            return actual
        # Listen before writing anything; never fight a second controller.
        until = time.perf_counter()+2
        while time.perf_counter()<until:
            c._receive(0);time.sleep(.005)
        for axis in AXES: write(axis,0,0.)
        for axis,config in AXES.items():
            verify(axis,8,config['node']);verify(axis,0,0);verify(axis,0x4c,0);verify(axis,0x0c,1)
            row = {}
            for key,param,value in [('position_kp',0x54,8),('position_kd',0x56,2),
                                    ('speed_kp',0x18,.5),('speed_ki',0x1a,config['ki']),
                                    ('current_limit',0x10,6),('acc_turn_s2',0x1c,.125),('dec_turn_s2',0x1e,.125)]:
                row[key] = verify(axis,param,value)
            original_speeds[axis] = read(axis,0x20)[0]
            if not 0 < original_speeds[axis] <= .1250001: raise RuntimeError('unexpected max speed')
            report['centers'][axis] = math.degrees(read(axis,0x40)[0])
            row['original_max_speed_turn_s'] = original_speeds[axis]
            write(axis,0x2a,500);verify(axis,0x2a,500)
            report['preflight'][axis] = row
        if args.centers:
            prior = json.loads(args.centers.read_text())
            if prior['case'] != 'preflight' or not prior['success']: raise RuntimeError('invalid preflight')
            report['centers'] = prior['centers']
        validate_centers(report['centers'])
        for axis in AXES:
            if abs(math.degrees(read(axis,0x40)[0])-report['centers'][axis]) > .5:
                raise RuntimeError(f'{axis}: current position moved away from test center')
        def poll(cycle,segment,start,mode):
            for axis in AXES:
                values,times,devices = {},{},{}
                for name,param in FIELDS.items():
                    values[name],times[name],devices[name] = read(axis,param)
                ref = latest_targets.get(axis)
                validate_feedback(axis,values,ref,mode)
                position = math.degrees(values['position'])
                writers['feedback'].writerow([times['position']-epoch,times['position']-start,cycle,segment,axis,
                    ref,position,math.degrees(values['speed']),values['iq'],values['mode'],values['fault'],values['accepted']*360,
                    None if ref is None else ref-position,*[times[k]-epoch for k in ('speed','iq','mode','fault','accepted')],devices['position']])
        # Static stability must pass before ARM, even on repeat invocations.
        start = time.perf_counter()
        for cycle in range(40):
            deadline = start+cycle*.05
            time.sleep(max(0,deadline-time.perf_counter()))
            if time.perf_counter()-deadline > .05: raise RuntimeError('static scheduler late')
            poll(cycle,'static',start,0)
        if args.case != 'preflight':
            for axis in AXES:
                write(axis,0x20,15/360);verify(axis,0x20,15/360)
            report['motion_enabled'] = True
            for axis in AXES: write(axis,0,3.)
            for axis in AXES: verify(axis,0,3);verify(axis,0x4c,0)
            start = time.perf_counter();cycle=0;segment_last=None
            report['case_start_s'] = start-epoch
            while True:
                deadline = start+cycle*.05
                time.sleep(max(0,deadline-time.perf_counter()))
                now = time.perf_counter();late=now-deadline
                if late > .05: raise RuntimeError('command scheduler over 50 ms late')
                target = targets(args.case,now-start)
                if target is None: break
                segment,relative = target
                if segment != segment_last:
                    print(args.case,'segment',segment,'relative',relative,flush=True);segment_last=segment
                for axis in AXES:
                    absolute = report['centers'][axis]+relative[axis]
                    lo,hi = AXES[axis]['bounds']
                    if not lo+5 <= absolute <= hi-5: raise RuntimeError('target exceeds margin')
                    write(axis,6,absolute/360);latest_targets[axis]=absolute
                    sent = last_tx[(AXES[axis]['node']<<8)|6]
                    writers['commands'].writerow([sent-epoch,sent-start,cycle,segment,axis,absolute,late])
                poll(cycle,segment,start,3);cycle+=1
            report['cycles'] = cycle
        report['success'] = True
    except BaseException as exc:
        report['failure'] = repr(exc)
        print('STOP:',repr(exc),flush=True)
    finally:
        reject_external = False
        if bus is not None:
            # Send both STOPs before querying either, even if one axis is missing.
            for axis,config in AXES.items():
                try: c.write(0,config['node'],0,0.)
                except BaseException as exc: report['stop_verified'][axis]=repr(exc)
            for axis,config in AXES.items():
                try:
                    c.verify(0,config['node'],0,0.)
                    report['stop_verified'][axis] = True
                    if axis in original_speeds:
                        c.write(0,config['node'],0x20,original_speeds[axis])
                        c.verify(0,config['node'],0x20,original_speeds[axis])
                    report.setdefault('final_fault',{})[axis] = c.read(0,config['node'],0x4c)[0]
                except BaseException as exc:
                    report['restore_errors'].append(f'{axis}: {exc!r}')
                    if axis not in report['stop_verified']: report['stop_verified'][axis]=False
            bus.close()
        report['success'] = report['success'] and all(report['stop_verified'].get(a) is True for a in AXES) and not report['restore_errors']
        ctypes.windll.winmm.timeEndPeriod(1)
        for stream in streams.values(): stream.close()
        (args.out/'run.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2),flush=True)
    return 0 if report['success'] else 1


if __name__ == '__main__': sys.exit(main())
