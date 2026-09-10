"""Execute one bounded two-axis CAN case, logging the 32-byte status stream.

Requires both boards to support 0x64/0x65. Does not write Flash or controller gains.
Command timing is host scheduled, not synchronized hardware triggering.
"""
import argparse
import csv
import ctypes
import json
import math
from pathlib import Path
import sys
import time

from can_motor_status import decode, FIELDS
from canfd_diagnostics import error_record, read_channel_diagnostics
from dual_axis_can_test import AXES, targets, validate_centers


def check_snapshot(axis, sample, mode=None, position_margin=2., position_bounds=None):
    required=['position_feedback_rad','speed_feedback_rad_s','iq_feedback_A','iq_reference_A']
    if mode == 3:
        required += ['position_target_rad','position_planned_rad','speed_planned_rad_s']
    if any(sample[k] is None or not math.isfinite(sample[k]) for k in required):
        raise RuntimeError(f'{axis}: invalid telemetry')
    if sample['fault'] or mode is not None and sample['mode'] != mode:
        raise RuntimeError(f'{axis}: mode/fault {sample["mode"]}/{sample["fault"]}, expected {mode}')
    low,high=AXES[axis]['bounds']
    if not low+position_margin < math.degrees(sample['position_feedback_rad']) < high-position_margin:
        raise RuntimeError(f'{axis}: position margin')
    if position_bounds and not position_bounds[0]<math.degrees(sample['position_feedback_rad'])<position_bounds[1]:
        raise RuntimeError(f'{axis}: restricted bench travel exceeded')
    if abs(sample['iq_feedback_A']) >= 4 or abs(sample['iq_reference_A']) >= 4:
        raise RuntimeError(f'{axis}: 4 A host stop threshold')
    if abs(math.degrees(sample['speed_feedback_rad_s'])) > 45:
        raise RuntimeError(f'{axis}: 45 deg/s host stop threshold')
    if mode==3 and abs(sample['position_planned_rad']-sample['position_feedback_rad'])>math.radians(5):
        raise RuntimeError(f'{axis}: planned tracking error >5 deg')


def accepted_target(sample, recent):
    """A stream sample can precede the latest command; account for 1 mrad truncation."""
    target=sample['position_target_rad']
    return target is not None and any(abs(target-math.radians(x)) <= .001001 for x in recent)


def validate_start(positions, centers, centering=False):
    validate_centers(centers)
    for axis in AXES:
        low,high=AXES[axis]['bounds']
        value=positions[axis]
        if not math.isfinite(value) or not low+5<=value<=high-5:
            raise RuntimeError(f'{axis}: start outside target margin')
        limit=10. if centering else .5
        if abs(value-centers[axis])>limit:
            raise RuntimeError(f'{axis}: start differs from center by >{limit} deg')


def validate_waypoints(plan):
    """Absolute-angle bench plan; reserve command margin and return to zero."""
    speed=plan['max_speed_deg_s'];points=plan['points']
    if not math.isfinite(speed) or not 0<speed<=15 or not isinstance(points,list) or not points:
        raise ValueError('invalid waypoint speed/list')
    previous={a:0. for a in AXES}
    restricted=plan.get('feedback_bounds_deg')
    if restricted:
        if set(restricted)!=set(AXES):raise ValueError('feedback bounds require both axes')
        for axis,(low,high) in restricted.items():
            configured=AXES[axis]['bounds']
            if not all(math.isfinite(x) for x in (low,high)) or not configured[0]<=low<0<high<=configured[1]:
                raise ValueError('feedback bounds must narrow the configured travel and contain zero')
    for point in points:
        if set(point)!= {'roll','pitch','duration_s'}:raise ValueError('invalid waypoint fields')
        if any(not math.isfinite(value) for value in point.values()):raise ValueError('nonfinite waypoint')
        for axis in AXES:
            low,high=AXES[axis]['bounds']
            if not low+2<=point[axis]<=high-2:raise ValueError(f'{axis}: waypoint exceeds 2 deg margin')
            if restricted and not restricted[axis][0]+1<=point[axis]<=restricted[axis][1]-1:
                raise ValueError('waypoint exceeds restricted bench margin')
        minimum=max(abs(point[a]-previous[a]) for a in AXES)/speed+3
        if not minimum<=point['duration_s']<=60:raise ValueError('insufficient movement/settling time')
        previous={a:point[a] for a in AXES}
    if any(previous.values()):raise ValueError('last waypoint must return both axes to zero')


def waypoint_target(plan,elapsed):
    if not math.isfinite(elapsed) or elapsed<0:raise ValueError('invalid time')
    edge=0.
    for index,point in enumerate(plan['points']):
        edge+=point['duration_s']
        if elapsed<edge:return str(index),{a:point[a] for a in AXES}
    return None


def main():
    if not __debug__: raise RuntimeError('Python -O is unsupported')
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('case',choices=['preflight','center','hold','small','same','opposite','sine01','sine02','waypoints'])
    ap.add_argument('--out',required=True,type=Path)
    ap.add_argument('--centers',type=Path)
    ap.add_argument('--zero-center',action='store_true',help='Preflight for absolute zero, without changing encoder zero')
    ap.add_argument('--plan',type=Path,help='Validated absolute-angle waypoints, only with case=waypoints')
    ap.add_argument('--rate',type=int,default=20,choices=range(10,201),metavar='10..200')
    ap.add_argument('--adapter-root',type=Path,default=Path('D:/Work/Code/motor_ctrl_app'))
    args=ap.parse_args()
    if (args.case=='waypoints') != bool(args.plan):ap.error('--plan required only for waypoints')
    plan=json.loads(args.plan.read_text()) if args.plan else None
    if plan:validate_waypoints(plan)
    if args.zero_center and args.case!='preflight':ap.error('--zero-center is a preflight option')
    if args.case != 'preflight' and args.centers is None: ap.error('--centers required')
    sys.path.insert(0,str(args.adapter_root.resolve()))
    from pc_replay.canfd import CanFD, RxFD, TxFD, Frame
    from pc_replay.protocol import Client
    class StatusBus(CanFD):
        def send(self,channel,identifier,data):
            if not 0<=identifier<=0x7ff or len(data)!=4:
                raise ValueError('standard-ID four-byte command required')
            message=TxFD()
            # Normal CAN transmission retries arbitration loss. Discovery's
            # single-shot mode can lose a command when periodic status competes.
            # One bounded case owns the adapter; close/reset clears pending TX.
            message.transmit_type=0
            message.frame.can_id=identifier;message.frame.length=4;message.frame.flags=1
            message.frame.data[:4]=data
            self._ok(self.dll.ZCAN_TransmitFD(self.channels[channel],ctypes.byref(message),1),'send')
        def receive(self,channel):
            messages=(RxFD*64)()
            count=self.dll.ZCAN_ReceiveFD(self.channels[channel],messages,64,0)
            if count>64: raise OSError(f'CAN receive failed: {count}')
            now=time.perf_counter()
            frames=[]
            for m in messages[:count]:
                if m.frame.length>64: raise OSError('invalid CAN FD length')
                error=error_record(m.frame.can_id,m.frame.flags,
                                   bytes(m.frame.data[:m.frame.length]),m.timestamp,now)
                if error is not None:
                    errors=report.setdefault('sdk_error_records',[])
                    if len(errors)<32: errors.append(error)
                    report['sdk_error_record_count']=report.get('sdk_error_record_count',0)+1
                frames.append(Frame(channel,m.frame.can_id,bytes(m.frame.data[:m.frame.length]),m.timestamp,now))
            return frames
    args.out.mkdir(parents=True,exist_ok=False)
    report={'case':args.case,'success':False,'command_hz':20,'feedback_hz':args.rate,
            'centers':{},'preflight':{},'stop_verified':{},'restore_errors':[],
            'motion_enabled':False,'current_stop_A':4,'feedback_timeout_s':.2,'failure':None}
    if plan:report['waypoint_plan']=plan
    files={};writers={}
    columns={'frames':['time_s','direction','channel','id','data','device_us'],
             'commands':['time_s','case_time_s','cycle','segment','axis','target_deg','lateness_s'],
             'feedback':['time_s','case_time_s','segment','axis','device_us',*FIELDS]}
    for key,cols in columns.items():
        files[key]=(args.out/(key+'.csv')).open('w',newline='',encoding='utf-8',buffering=1)
        writers[key]=csv.writer(files[key]);writers[key].writerow(cols)
    epoch=time.perf_counter();case_start=None;segment='setup';bus=None;c=None
    latest={};history={a:[] for a in AXES};original_speeds={};safety=True;expected_mode=None
    last_tx={};counts={a:0 for a in AXES};configured_streams=set()
    by_node={v['node']:a for a,v in AXES.items()}
    def trace(direction,ch,identifier,data,when,device):
        writers['frames'].writerow([when-epoch,direction,ch,hex(identifier),data.hex(),device])
        if direction=='tx': last_tx[identifier]=when;return
        if safety and identifier & 0x20000000:
            raise RuntimeError(f'CAN adapter error frame: {identifier:#x}')
        if 0x7f0 <= identifier <= 0x7f7 and len(data)==32:
            sample=decode(identifier,data)
            if sample['node'] not in by_node: return
            axis=by_node[sample['node']];latest[axis]=(when,sample);counts[axis]+=1
            writers['feedback'].writerow([when-epoch,None if case_start is None else when-case_start,segment,axis,device,*[sample[k] for k in FIELDS]])
            if safety:
                check_snapshot(axis,sample,expected_mode,1. if plan else 2.,
                               plan.get('feedback_bounds_deg',{}).get(axis) if plan else None)
                recent=[value for sent,value in history[axis] if when-sent < .2]
                if expected_mode==3 and recent and not accepted_target(sample,recent):
                    raise RuntimeError(f'{axis}: reported CAN target does not match recent commands')
        elif safety and not(identifier & 1):
            raise RuntimeError(f'External CAN command: {identifier:#x} {data.hex()}')
    def write(axis,param,value): c.write(0,AXES[axis]['node'],param,value)
    def read(axis,param): return c.read(0,AXES[axis]['node'],param)[0]
    def verify(axis,param,value): return c.verify(0,AXES[axis]['node'],param,value,tolerance=1e-5)
    def fresh():
        now=time.perf_counter()
        for axis in AXES:
            if axis not in latest or now-latest[axis][0]>.2:
                raise RuntimeError(f'{axis}: status feedback timeout')
    def static(seconds):
        end=time.perf_counter()+seconds;heartbeat=0
        while time.perf_counter()<end:
            c._receive(0)
            if time.perf_counter()>heartbeat:
                for axis in AXES: verify(axis,0,0)
                heartbeat=time.perf_counter()+.15
            fresh();time.sleep(.001)
    ctypes.windll.winmm.timeBeginPeriod(1)
    try:
        bus=StatusBus(str(args.adapter_root/'.local/canfd-sdk/ControlCANFD.dll'),channels=(0,))
        report['adapter']=bus.info;c=Client(bus,timeout=.08,trace=trace)
        end=time.perf_counter()+2
        while time.perf_counter()<end: c._receive(0);time.sleep(.005)
        for axis in AXES: write(axis,0,0.)
        for axis,config in AXES.items():
            verify(axis,0,0);verify(axis,0x4c,0);verify(axis,8,config['node']);verify(axis,0xc,1)
            row={}
            for key,param,wanted in [('position_kp',0x54,8),('position_kd',0x56,2),('speed_kp',0x18,.5),('speed_ki',0x1a,config['ki']),('current_limit',0x10,6),('acc_turn_s2',0x1c,.125),('dec_turn_s2',0x1e,.125)]:
                row[key]=verify(axis,param,wanted)
            original_speeds[axis]=read(axis,0x20)
            if not 0<original_speeds[axis]<=.1250001: raise RuntimeError('unexpected speed configuration')
            row['original_max_speed_turn_s']=original_speeds[axis]
            report['centers'][axis]=math.degrees(read(axis,0x40))
            write(axis,0x2a,500);verify(axis,0x2a,500)
            write(axis,0x64,args.rate);configured_streams.add(axis);verify(axis,0x64,args.rate)
            report['preflight'][axis]=row
        expected_mode=0
        end=time.perf_counter()+.2
        while len(latest)<2 and time.perf_counter()<end: c._receive(0);time.sleep(.001)
        segment='static';static(3)
        report['measured_start_deg']=dict(report['centers'])
        if args.zero_center:
            report['centers']={axis:0. for axis in AXES}
            report['center_source']='absolute_zero_requested'
        if args.centers:
            prior=json.loads(args.centers.read_text())
            if prior['case']!='preflight' or not prior['success']: raise RuntimeError('invalid preflight file')
            report['centers']=prior['centers']
        if plan and any(report['centers'].values()):raise RuntimeError('waypoint plans require absolute zero centers')
        validate_start({axis:math.degrees(read(axis,0x40)) for axis in AXES},report['centers'],
                       args.case=='center' or args.zero_center)
        if args.case!='preflight':
            test_speed=plan['max_speed_deg_s'] if plan else (5 if args.case=='center' else 15)
            report['test_max_speed_deg_s']=test_speed
            for axis in AXES: write(axis,0x20,test_speed/360);verify(axis,0x20,test_speed/360)
            expected_mode=None
            report['motion_enabled']=True
            for axis in AXES: write(axis,0,3)
            for axis in AXES: verify(axis,0,3);verify(axis,0x4c,0)
            for axis in AXES:
                history[axis]=[(time.perf_counter(),read(axis,6)*360)]
            # Drain frames sampled during mode transition before enforcing mode 3.
            c._receive(0);expected_mode=3
            case_start=time.perf_counter();report['case_start_s']=case_start-epoch;cycle=0
            prior_segment=None
            while True:
                deadline=case_start+cycle*.05
                while time.perf_counter()<deadline:
                    c._receive(0);fresh();time.sleep(.001)
                now=time.perf_counter();late=now-deadline
                if late>.05: raise RuntimeError('command scheduling >50 ms late')
                fresh()
                target=waypoint_target(plan,now-case_start) if plan else ((('0',{axis:0. for axis in AXES}) if now-case_start<8 else None) if args.case=='center' else targets(args.case,now-case_start))
                if target is None: break
                segment,relative=target
                if segment!=prior_segment:
                    print(args.case,'segment',segment,relative,flush=True);prior_segment=segment
                for axis in AXES:
                    value=report['centers'][axis]+relative[axis]
                    low,high=AXES[axis]['bounds']
                    margin=2 if plan else 5
                    if not low+margin<=value<=high-margin: raise RuntimeError('command outside test bounds')
                    write(axis,6,value/360)
                    sent=last_tx[(AXES[axis]['node']<<8)|6]
                    history[axis]=[(t,v) for t,v in history[axis] if sent-t<.3]+[(sent,value)]
                    writers['commands'].writerow([sent-epoch,sent-case_start,cycle,segment,axis,value,late])
                c._receive(0);cycle+=1
            report['cycles']=cycle
        report['success']=True
    except BaseException as exc:
        report['failure']=repr(exc);print('STOP:',repr(exc),flush=True)
    finally:
        safety=False;expected_mode=None;segment='stop'
        if c is not None:
            for axis in AXES:
                try: write(axis,0,0)
                except BaseException as e: report.setdefault('stop_send_errors',{})[axis]=repr(e)
            # Send STOP first, then capture before close/reset destroys evidence.
            report['can_diagnostics_after_stop_sent']=read_channel_diagnostics(bus)
            for axis in AXES:
                report['stop_verified'][axis]=False
                for attempt in range(3):
                    try:
                        if attempt: write(axis,0,0)
                        verify(axis,0,0);report['stop_verified'][axis]=True;break
                    except BaseException as e: report.setdefault('stop_read_errors',{}).setdefault(axis,[]).append(repr(e))
                if report['stop_verified'][axis]:
                    try:
                        if axis in original_speeds:
                            write(axis,0x20,original_speeds[axis]);verify(axis,0x20,original_speeds[axis])
                        report.setdefault('final_fault',{})[axis]=read(axis,0x4c)
                        if axis in configured_streams: write(axis,0x64,0);verify(axis,0x64,0)
                    except BaseException as e: report['restore_errors'].append(f'{axis}: {e!r}')
        if bus is not None:
            report['can_diagnostics_before_close']=read_channel_diagnostics(bus)
            bus.close()
        report['feedback_counts']=counts
        report['success']=report['success'] and all(report['stop_verified'].get(a) is True for a in AXES) and not report['restore_errors']
        ctypes.windll.winmm.timeEndPeriod(1)
        for f in files.values(): f.close()
        (args.out/'run.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps(report,indent=2),flush=True)
    return 0 if report['success'] else 1


if __name__=='__main__':sys.exit(main())
