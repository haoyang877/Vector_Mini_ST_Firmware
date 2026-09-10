"""Reproduce roll noise/trajectory report figures from archived real HIL frames."""
import csv
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'outputs/servo_capture_03_pid3_20260908/.plot_deps'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

SESSION = ROOT / 'outputs/hold_noise_20260910/roll_connected'
OUT = ROOT / 'docs/assets/noise_optimization_20260910'
OUT.mkdir(parents=True, exist_ok=True)
FONT = Path('C:/Windows/Fonts/msyh.ttc')
if FONT.exists():
    matplotlib.rcParams['font.family'] = FontProperties(fname=str(FONT)).get_name()
matplotlib.rcParams.update({'axes.unicode_minus': False, 'font.size': 10,
    'axes.spines.top': False, 'axes.spines.right': False, 'figure.dpi': 160,
    'savefig.dpi': 160, 'axes.titlepad': 12, 'svg.fonttype': 'none'})
COLORS = ['#64748b', '#d28b24', '#087f8c']
PLAN, ACTUAL, TARGET = '#c87d1c', '#087f8c', '#64748b'


def load(name):
    path = SESSION / name
    trial = json.loads((path/'trial.json').read_text())
    assert trial['failure'] is None and trial['shutdown_verified']
    data = np.loadtxt(path/'capture.tsv', skiprows=1)
    assert data.shape[1] == 12 and np.isfinite(data).all()
    flags = data[:, 11].astype(np.int64) & 65535
    events = [e for e in trial['events'] if e['opcode'] == 3]
    motion_flags = flags[events[0]['frame']:]
    assert np.all((motion_flags & 0x180) == 0x180) and not np.any(motion_flags & 0x68)
    times = np.interp(np.arange(len(data)), [p['frame'] for p in trial['polls']],
                      [p['time'] for p in trial['polls']])
    origin = events[0]['time']
    target = np.full(len(data), np.nan)
    for i, event in enumerate(events):
        end = events[i+1]['frame'] if i+1 < len(events) else len(data)
        target[event['frame']:end] = np.degrees(event['value'])
    return dict(path=path, trial=trial, data=data, flags=flags, events=events,
                t=times-origin, origin=origin, target=target,
                q=np.unwrap(data[:,1]*np.pi/32768)*180/np.pi,
                ref=np.unwrap(data[:,0]*np.pi/32768)*180/np.pi,
                summary=json.loads((path/'positioning_summary.json').read_text()))


def style(axes):
    for ax in np.asarray(axes).ravel():
        ax.grid(alpha=.17, axis='both')
        ax.set_axisbelow(True)


def save(fig, name, footer):
    fig.text(.035, .02, footer, fontsize=8.5, color='#475569')
    fig.savefig(OUT/(name+'.png'), facecolor='white')
    plt.close(fig)


def noise_comparison():
    names = ['motion_noise_40hz_1', 'motion_noise_20hz_1', 'single_10hz_motion_1']
    labels = ['40 Hz +\nHOLD 10 Hz', '20 Hz +\nHOLD 10 Hz', '统一单个\n10 Hz']
    levels, settling, errors, tracking_errors = [], [], [], []
    for name in names:
        path = SESSION/name
        n = json.loads((path/'motion_noise_summary.json').read_text())
        band = next(b for b in n['bands'] if b['low_hz'] == 40 and b['high_hz'] == 800)
        levels.append(band['iq_feedback_rms_A']*1000)
        p = json.loads((path/'positioning_summary.json').read_text())
        moves = p['moves'][1:]
        settling.append([m['settle_0p3_s'] for m in moves if abs(abs(m['commanded_step_deg'])-40)<1e-5])
        errors.append(max(m['tail_error_abs_max_deg'] for m in moves))
        trial = json.loads((path/'trial.json').read_text())
        start = [e['frame'] for e in trial['events'] if e['opcode'] == 3][1]
        data = np.loadtxt(path/'capture.tsv', skiprows=1)
        tracking_errors.append(float(np.max(abs(data[start:,2]))/10000*180/np.pi))
    fig, axes = plt.subplots(1, 3, figsize=(13.6, 4.8))
    fig.subplots_adjust(left=.065, right=.975, bottom=.23, top=.76, wspace=.37)
    fig.suptitle('运动更安静，代价是到位略慢、末段误差略增', x=.045, ha='left', y=.97, fontsize=17, weight='bold')
    means = [float(np.mean(v)) for v in settling]
    values = [levels, means, errors]
    titles = ['运动段电流波动', '40°移动到位时间', '末段最大绝对位置误差']
    units = ['Iq反馈频带RMS / mA', '时间 / s', '误差 / °']
    for index, ax in enumerate(axes):
        ax.bar(np.arange(3), values[index], color=COLORS, width=.58, zorder=3)
        ax.set_xticks(np.arange(3), labels)
        ax.set_title(titles[index], fontsize=11)
        ax.set_ylabel(units[index])
        for i, value in enumerate(values[index]):
            ax.text(i, value, f'{value:.2f}' if index==0 else f'{value:.3f}',
                    ha='center', va='bottom', fontsize=11)
    axes[0].set_ylim(0, 68)
    axes[0].text(.5,.9,f'40 → 10 Hz：下降 {(1-levels[-1]/levels[0])*100:.0f}%',
                 transform=axes[0].transAxes, ha='center', color=ACTUAL, weight='bold')
    axes[1].set_ylim(0, 3.1)
    axes[1].errorbar(range(3), means,
        yerr=np.array([[np.mean(v)-min(v), max(v)-np.mean(v)] for v in settling]).T,
        fmt='none', capsize=4, color='#0f172a', zorder=4)
    axes[2].set_ylim(0,.36)
    axes[2].axhline(.3, color='#b45309', ls='--', lw=1)
    axes[2].text(.5,.307,'当前到位尺度 0.3°',color='#92400e',ha='center',fontsize=9)
    style(axes)
    save(fig, '01_noise_tradeoff',
        '同一往返序列，每目标5秒；电流统计频带40～800 Hz，不是声压。到位时间：进入并保持±0.3°；右图统计末2秒。\n'
        '三轮顺序实测，未随机化；图中数值是本次台架观测，不代表全工况性能保证。')
    return dict(motion_iq_band_rms_mA=levels, settling_40deg_mean_s=means,
                tail_error_abs_max_deg=errors, tracking_error_abs_max_deg=tracking_errors, sources=names)


def trajectory():
    r = load('single_10hz_default_accept')
    t, data, ref, q = r['t'], r['data'], r['ref'], r['q']
    valid = np.isfinite(r['target']) & (t >= 0)
    t = t[valid]
    fig, axes = plt.subplots(4, 1, figsize=(13.6, 9.3), sharex=True,
                             gridspec_kw={'height_ratios':[2.1,1.2,1.1,.55]})
    fig.subplots_adjust(left=.085, right=.97, bottom=.105, top=.86, hspace=.22)
    fig.suptitle('最终10 Hz固件：重启后的真实轨迹跟随',x=.055,ha='left',y=.975,fontsize=17,weight='bold')
    fig.text(.055,.929,'目标序列 0° → +20° → −20° → 0°，每目标6秒；规划参考和编码器反馈直接取自RTT。',fontsize=10,color='#475569')
    axes[0].plot(t,r['target'][valid],ls='--',lw=1.1,color=TARGET,label='下发目标（阶跃）')
    axes[0].plot(t,ref[valid],lw=2.1,color=PLAN,label='规划位置参考')
    axes[0].plot(t,q[valid],lw=1.0,color=ACTUAL,label='编码器实际位置')
    axes[0].set_ylabel('位置 / °'); axes[0].legend(ncol=3,loc='upper right')
    error = data[valid,2]/10000*180/np.pi
    axes[1].plot(t,error,color=ACTUAL,lw=.8)
    axes[1].axhline(0,color=TARGET,lw=.7)
    axes[1].set_ylabel('轨迹跟随误差 / °')
    axes[1].text(.01,.85,f'规划参考 − 实际位置；全程最大绝对值 {np.max(abs(error)):.3f}°',
                  transform=axes[1].transAxes,fontsize=9)
    axes[2].plot(t,data[valid,7]/1000,color=ACTUAL,lw=.7,label='实际Iq')
    axes[2].set_ylabel('Iq反馈 / A')
    hold = ((r['flags'][valid]&0x187)==0x186).astype(int)
    axes[3].fill_between(t,0,hold,step='post',color=ACTUAL,alpha=.45)
    axes[3].set_yticks([0,1],['未保持','HOLD']);axes[3].set_ylim(-.05,1.1)
    axes[3].set_xlabel('从首个目标命令计时 / s（主机接收锚点）')
    for ax in axes: ax.set_xlim(0,t[-1])
    style(axes)
    save(fig,'02_final_trajectory',
         '轨迹跟随误差相对连续规划参考；到位误差相对最终目标，两者不能混用。未对误差/电流曲线额外平滑。\n'
         '来源：single_10hz_default_accept；2 kHz遥测，时间由主机接收锚点估计，存在RTT批量接收误差。')
    metrics = []
    for i,event in enumerate(r['events']):
        end = r['events'][i+1]['frame'] if i+1<len(r['events']) else len(data)
        sl = slice(event['frame'],end)
        tracking = data[sl,2]/10000*180/np.pi
        moving = abs(data[sl,3]) > 50
        m = dict(r['summary']['moves'][i])
        m.update(tracking_error_abs_max_deg=float(np.max(abs(tracking))),
                 moving_tracking_error_rms_deg=float(np.sqrt(np.mean(tracking[moving]**2))) if moving.any() else None)
        metrics.append(m)
    return r, metrics


def landing(r):
    event, stop_event = r['events'][2], r['events'][3]
    ids = np.arange(event['frame'],stop_event['frame'])
    t = r['t'][ids]+r['origin']-event['time']
    error = -20-r['q'][ids]
    fig, axes = plt.subplots(1,2,figsize=(13.6,4.9))
    fig.subplots_adjust(left=.075,right=.97,bottom=.22,top=.76,wspace=.24)
    fig.suptitle('40°跨零运动：轨迹跟随与到位保持分开看',x=.045,ha='left',y=.97,fontsize=17,weight='bold')
    axes[0].plot(t,r['ref'][ids],lw=2,color=PLAN,label='规划参考')
    axes[0].plot(t,r['q'][ids],lw=1,color=ACTUAL,label='实际位置')
    axes[0].axhline(-20,color=TARGET,ls='--',lw=1,label='最终目标 −20°')
    axes[0].set_xlim(0,3.6);axes[0].set_ylabel('位置 / °')
    axes[0].set_title('+20° → −20°');axes[0].legend(fontsize=9)
    m = r['summary']['moves'][2]
    after = t>=m['settle_0p3_s']
    axes[1].axhspan(-.3,.3,color='#e2f2e8')
    axes[1].axhline(0,color=TARGET,lw=.7)
    axes[1].plot(t[after],error[after],color=ACTUAL,lw=.9)
    axes[1].set_xlim(m['settle_0p3_s'],t[-1]);axes[1].set_ylim(-.35,.35)
    axes[1].set_title(f'进入±0.3°后局部放大（约{m["settle_0p3_s"]:.2f}秒起）')
    axes[1].set_ylabel('到位误差：最终目标 − 实际 / °')
    axes[1].text(.04,.89,f'末2秒最大绝对误差：{m["tail_error_abs_max_deg"]:.3f}°',transform=axes[1].transAxes,fontsize=10)
    for ax in axes:ax.set_xlabel('相对本次目标命令 / s')
    style(axes)
    save(fig,'03_landing_detail',
         '右图只显示已进入到位范围后的记录，不展示此前大误差；绿色为当前±0.3°到位尺度。\n'
         '位置仍存在约0.2°残差，不能据左图曲线接近重合宣称零误差跟随。来源：single_10hz_default_accept。')


def small_moves():
    r = load('single_10hz_position_1')
    fig, axes = plt.subplots(1,2,figsize=(13.6,5.0))
    fig.subplots_adjust(left=.075,right=.97,bottom=.22,top=.75,wspace=.25)
    fig.suptitle('小幅定位：0.1°微动仍是未解决项',x=.045,ha='left',y=.97,fontsize=17,weight='bold')
    for ax, begin, end, title in [(axes[0],0,5,'±0.1°指令：实际仍停在零点附近'),
                                  (axes[1],5,9,'±1°指令：能够运动，但保留停点误差')]:
        ids = np.arange(r['events'][begin]['frame'],r['events'][end]['frame'])
        t = r['t'][ids]+r['origin']-r['events'][begin]['time']
        ax.plot(t,r['target'][ids],color=TARGET,ls='--',lw=1.1,label='下发目标')
        ax.plot(t,r['ref'][ids],color=PLAN,lw=1.7,label='规划参考')
        ax.plot(t,r['q'][ids],color=ACTUAL,lw=.9,label='实际位置')
        ax.set_title(title,fontsize=11);ax.set_xlabel('区段时间 / s');ax.set_ylabel('位置 / °')
        ax.set_xlim(0,t[-1]);ax.legend(ncol=3,fontsize=8.5,loc='upper right')
    axes[0].set_ylim(-.16,.16);axes[1].set_ylim(-1.3,1.5)
    style(axes)
    save(fig,'04_small_positioning',
         '每个目标保持8秒；图中波动是原始编码器反馈，没有人为平滑。来源：single_10hz_position_1。\n'
         '当前HOLD进入/退出窗口0.18°/0.26°；微小指令响应还受保持策略与摩擦影响，不能只靠降噪滤波解决。')


def main():
    comparison = noise_comparison()
    record, metrics = trajectory()
    landing(record)
    small_moves()
    (OUT/'metrics.json').write_text(json.dumps(dict(comparison=comparison, final_moves=metrics),indent=2,allow_nan=False)+'\n')
    keys = ['target_deg','commanded_step_deg','settle_0p3_s','first_hold_s',
            'tracking_error_abs_max_deg','moving_tracking_error_rms_deg',
            'tail_error_abs_max_deg','tail_position_pp_deg','overshoot_deg','iq_peak_A']
    with (OUT/'final_tracking_metrics.csv').open('w',newline='',encoding='utf-8-sig') as stream:
        writer = csv.DictWriter(stream,fieldnames=keys)
        writer.writeheader();writer.writerows({k:m[k] for k in keys} for m in metrics)
    names = comparison['sources']+['single_10hz_position_1','single_10hz_default_accept']
    manifest = {name:dict(capture_sha256=hashlib.sha256((SESSION/name/'capture.tsv').read_bytes()).hexdigest(),
                          image=json.loads((SESSION/name/'trial.json').read_text())['image']) for name in names}
    (OUT/'sources.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(dict(figures=str(OUT),comparison=comparison,final_moves=metrics),indent=2))


if __name__ == '__main__':
    main()
