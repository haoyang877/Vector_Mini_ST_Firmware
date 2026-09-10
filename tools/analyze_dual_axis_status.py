"""Analyze measured dual-axis CAN cases; no device access."""
import argparse
import csv
import json
import math
from pathlib import Path
import sys
import numpy as np


def rows(path):
    with path.open(encoding='utf-8') as f: return list(csv.DictReader(f))


def analyze(directory,plot=True):
    run=json.loads((directory/'run.json').read_text())
    feedback=rows(directory/'feedback.csv'); commands=rows(directory/'commands.csv')
    result={'case':run['case'],'runtime_pass':run['success'],'acceptance_pass':run['success'],'axes':{}}
    datasets={}
    for axis in ('roll','pitch'):
        all_samples=[r for r in feedback if r['axis']==axis]
        time_all=np.array([float(r['time_s']) for r in all_samples])
        gaps=np.diff(time_all)
        metric={'total_frames':len(all_samples),'max_receive_gap_s':float(max(gaps)) if len(gaps) else None}
        data=[r for r in all_samples if r['case_time_s'] and float(r['case_time_s'])>=0 and r['segment']!='stop']
        sent=[r for r in commands if r['axis']==axis]
        if not data or not sent:
            result['axes'][axis]=metric;continue
        t=np.array([float(r['case_time_s']) for r in data]);absolute=np.array([float(r['time_s']) for r in data])
        ct=np.array([float(r['time_s']) for r in sent]);indices=np.searchsorted(ct,absolute,side='right')-1
        valid=indices>=0;t=t[valid];indices=indices[valid];data=[r for r,v in zip(data,valid) if v]
        target=np.array([float(sent[i]['target_deg']) for i in indices])
        actual=np.degrees([float(r['position_feedback_rad']) for r in data])
        planned=np.degrees([float(r['position_planned_rad']) for r in data])
        speed=np.degrees([float(r['speed_feedback_rad_s']) for r in data])
        iq=np.array([float(r['iq_feedback_A']) for r in data])
        iqref=np.array([float(r['iq_reference_A']) for r in data])
        error=target-actual;planerror=planned-actual
        metric.update(samples=len(t),target_rmse_deg=float(np.sqrt(np.mean(error**2))),
                      target_max_error_deg=float(max(abs(error))),planned_rmse_deg=float(np.sqrt(np.mean(planerror**2))),
                      iq_peak_A=float(max(abs(iq))),iq_reference_peak_A=float(max(abs(iqref))),
                      iq_tracking_rmse_A=float(np.sqrt(np.mean((iqref-iq)**2))),
                      speed_peak_deg_s=float(max(abs(speed))))
        if run['case'].startswith('sine'):
            freq=.1 if run['case']=='sine01' else .2;end=2/freq;mask=t<end
            metric['sine_rmse_deg']=float(np.sqrt(np.mean(error[mask]**2)))
            metric['sine_max_error_deg']=float(max(abs(error[mask])))
            metric['sine_planned_tracking_rmse_deg']=float(np.sqrt(np.mean(planerror[mask]**2)))
            metric['sine_target_to_planned_rmse_deg']=float(np.sqrt(np.mean((target[mask]-planned[mask])**2)))
            lags=np.arange(0,2.001,.005);command_t=np.array([float(r['case_time_s']) for r in sent]);command_y=np.array([float(r['target_deg']) for r in sent])
            stable=(t>=2)&mask
            scores=[];planned_scores=[]
            for lag in lags:
                idx=np.searchsorted(command_t,t[stable]-lag,side='right')-1
                scores.append(np.mean((command_y[idx]-actual[stable])**2))
                planned_scores.append(np.mean((command_y[idx]-planned[stable])**2))
            metric['estimated_lag_s']=float(lags[np.argmin(scores)])
            metric['estimated_planned_lag_s']=float(lags[np.argmin(planned_scores)])
            metric['pass']=metric['sine_rmse_deg']<=1 and metric['sine_max_error_deg']<=2
        else:
            labels=np.array([sent[i]['segment'] for i in indices]);segments=[]
            prior_target=run['centers'][axis]
            for label in dict.fromkeys(labels):
                mask=labels==label;segment_cmd=[r for r in sent if r['segment']==label]
                start=float(segment_cmd[0]['case_time_s']);end=float(segment_cmd[-1]['case_time_s'])+.05
                complete=True
                if 'waypoint_plan' in run:
                    planned_end=sum(p['duration_s'] for p in run['waypoint_plan']['points'][:int(label)+1])
                    complete=end>=planned_end-.06
                    end=planned_end
                tail=mask&(t>=end-2)&complete;goal=float(segment_cmd[-1]['target_deg'])
                bad=mask&(abs(error)>.3)
                last_bad=max(t[bad]) if np.any(bad) else start
                settle_candidates=t[mask&(t>last_bad)]
                settle=float(settle_candidates[0]-start) if len(settle_candidates) and complete else None
                direction=np.sign(goal-prior_target)
                overshoot=float(max(0,max(direction*(actual[mask]-goal)))) if direction else 0.
                entry={'segment':label,'target_deg':goal,'completed':complete,'tail_max_error_deg':float(max(abs(error[tail]))) if np.any(tail) else None,
                       'tail_max_speed_deg_s':float(max(abs(speed[tail]))) if np.any(tail) else None,
                       'settling_time_to_0p3deg_s':settle,'overshoot_deg':overshoot}
                entry['pass']=bool(np.count_nonzero(tail)>=run['feedback_hz']*1.5 and entry['tail_max_error_deg']<=.3 and entry['tail_max_speed_deg_s']<=1 and overshoot<=.5)
                segments.append(entry);prior_target=goal
            metric['segments']=segments;metric['pass']=all(s['pass'] for s in segments)
        result['acceptance_pass'] &= metric['pass']
        result['axes'][axis]=metric
        datasets[axis]=(t,target,planned,actual,error,planerror,iq,iqref)
    if len(datasets)==2:
        rt,_,_,ra,*_=datasets['roll'];pt,_,_,pa,*_=datasets['pitch']
        mask=(rt>=pt[0])&(rt<=pt[-1]);pitch=np.interp(rt[mask],pt,pa)-run['centers']['pitch']
        relative=(ra[mask]-run['centers']['roll'])-(-pitch if run['case']=='opposite' else pitch)
        if run['case']!='waypoints':
            result['axis_relative_rmse_deg']=float(np.sqrt(np.mean(relative**2)))
            result['axis_relative_max_deg']=float(max(abs(relative)))
        else:
            error_difference=datasets['roll'][4][mask]-np.interp(rt[mask],pt,datasets['pitch'][4])
            result['axis_tracking_error_difference_rmse_deg']=float(np.sqrt(np.mean(error_difference**2)))
    pairs={}
    for r in commands: pairs.setdefault(r['cycle'],{})[r['axis']]=float(r['time_s'])
    skews=[abs(p['roll']-p['pitch']) for p in pairs.values() if len(p)==2]
    if skews: result['command_pair_skew_ms']={'mean':float(np.mean(skews)*1000),'max':float(max(skews)*1000)}
    (directory/'metrics.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    if plot and datasets:
        root=Path(__file__).resolve().parents[1]
        sys.path.insert(0,str(root/'outputs/servo_capture_03_pid3_20260908/.plot_deps'))
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(3,2,figsize=(14,9),sharex='col')
        for col,(axis,data) in enumerate(datasets.items()):
            t,target,planned,actual,error,planerror,iq,iqref=data
            axes[0,col].step(t,target,where='post',label='CAN target',color='#65738c',linewidth=1.3)
            axes[0,col].plot(t,planned,label='Planned',color='#ec9f05',linewidth=1.4)
            axes[0,col].plot(t,actual,label='Feedback',color='#087e8b',linewidth=1.3)
            axes[0,col].set_title(axis);axes[0,col].set_ylabel('Position (deg)')
            axes[1,col].plot(t,error,label='Target - feedback',color='#c44536')
            axes[1,col].plot(t,planerror,label='Planned - feedback',color='#7b4fa3',alpha=.75)
            axes[1,col].set_ylabel('Error (deg)')
            axes[2,col].plot(t,iqref,label='Iq reference',color='#ec9f05')
            axes[2,col].plot(t,iq,label='Iq feedback',color='#087e8b',alpha=.85)
            axes[2,col].set_ylabel('Current (A)');axes[2,col].set_xlabel('Time (s)')
        for ax in axes.flat: ax.grid(alpha=.2);ax.legend(loc='best',fontsize=8)
        fig.suptitle(f"Dual-axis CAN: {run['case']} | feedback {run['feedback_hz']} Hz | measured bench data")
        fig.tight_layout();fig.savefig(directory/'tracking.png',dpi=160);plt.close(fig)
    return result


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('directory',type=Path);ap.add_argument('--no-plot',action='store_true')
    args=ap.parse_args();print(json.dumps(analyze(args.directory,not args.no_plot),indent=2))
