"""Summarize HIL trial against commanded targets, using host timing anchors."""
from pathlib import Path
import json, sys, math
import numpy as np
from rtt_control_frame import decode

def analyze(path):
    r=json.loads((path/'trial.json').read_text())
    d=np.loadtxt(path/'capture.tsv',skiprows=1)
    if decode(d)['version'] != 1:
        raise ValueError('Historical v1 report only; use analyze_positioning_comparison.py for v2')
    s=d[:,11].astype(int)&65535;v=(s&128)!=0
    q=np.unwrap(d[:,1]*np.pi/32768)*180/np.pi
    # Host receipt timing is approximate; RTT batches introduce several ms delay.
    polls=r['polls'];frames=np.array([p['frame'] for p in polls]);times=np.array([p['time'] for p in polls])
    ts=np.interp(np.arange(len(d)),frames,times)
    events=[x for x in r['events'] if x['opcode']==3]
    result={'name':path.name,'parameters':r['arguments'],'failure':r['failure'],
            'irq_rate_hz':(polls[-1]['tick']-polls[0]['tick'])/(times[-1]-times[0]),
            'irq_timing':r.get('timing'),
            'samples':len(d),'valid_samples':int(v.sum()),
            'drop_flag_samples':int(np.count_nonzero(s&64)),
            'saturation_samples':int(np.count_nonzero(v&((s&8)!=0))),
            'iq_peak_A':float(np.max(abs(d[v,7]))/1000) if v.any() else None,
            'iq_tracking_rms_A':float(np.sqrt(np.mean((d[v,6]-d[v,7])**2))/1000) if v.any() else None,
            'timing':'Host receipt anchors, approximate batch latency; no per-frame timestamps.',
            'moves':[]}
    for i,e in enumerate(events):
        a=e['frame'];b=events[i+1]['frame'] if i+1<len(events) else len(d)
        target=math.degrees(e['value']);err=target-q[a:b]
        outside=np.flatnonzero(abs(err)>.3)
        settled=(outside[-1]+1) if len(outside) else 0
        tail=np.arange(a,b)[ts[a:b]>=ts[b-1]-2]
        direction=np.sign(target-q[a])
        c={'target_deg':target,'elapsed_s':float(ts[b-1]-e['time']),
           'settle_s':float(max(0,ts[a+settled]-e['time'])) if settled<len(err) else None,
           'overshoot_deg':float(max(0,np.max(direction*(q[a:b]-target)))),
           'tail_error_range_deg':[float(np.min(target-q[tail])),float(np.max(target-q[tail]))],
           'tail_motion_pp_deg':float(np.ptp(q[tail])),
           'tail_hold_fraction':float(np.mean((s[tail]&3)==2)),
           'tail_ff_peak_A':float(np.max(abs(d[tail,9]))/1000),
           'tail_iq_peak_A':float(np.max(abs(d[tail,7]))/1000),
           'tail_integral_range_A':[float(d[tail,10].min()/1000),float(d[tail,10].max()/1000)],
           'tracking_peak_deg':float(np.max(abs(d[a:b,2]))/10000*180/np.pi)}
        result['moves'].append(c)
    (path/'summary.json').write_text(json.dumps(result,indent=2))
    return result

if __name__=='__main__':
    for arg in sys.argv[1:]:print(json.dumps(analyze(Path(arg)),indent=2))
