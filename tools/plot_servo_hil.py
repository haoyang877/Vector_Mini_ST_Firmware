"""Static figures for the direct-drive HIL report, from raw frames and host time anchors."""
from pathlib import Path
import sys, json

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'outputs/servo_capture_03_pid3_20260908/.plot_deps'))
import numpy as np
from rtt_control_frame import decode
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

OUT = ROOT / 'outputs/servo_hil_20260908'
FONT = Path('C:/Windows/Fonts/msyh.ttc')
if FONT.exists():
    matplotlib.rcParams['font.family'] = FontProperties(fname=str(FONT)).get_name()
matplotlib.rcParams.update({'axes.unicode_minus': False, 'font.size': 10,
    'axes.spines.top': False, 'axes.spines.right': False, 'figure.dpi': 150})


def load(name):
    p = OUT / name
    r = json.loads((p / 'trial.json').read_text())
    d = np.loadtxt(p / 'capture.tsv', skiprows=1)
    if decode(d)['version'] != 1:
        raise ValueError('Historical v1 report only; use analyze_positioning_comparison.py for v2')
    polls = r['polls']
    t = np.interp(np.arange(len(d)), [x['frame'] for x in polls], [x['time'] for x in polls])
    events = [x for x in r['events'] if x['opcode'] == 3]
    target = np.full(len(d), np.nan)
    for i, e in enumerate(events):
        end = events[i+1]['frame'] if i+1 < len(events) else len(d)
        target[e['frame']:end] = np.degrees(e['value'])
    q = np.unwrap(d[:,1] * np.pi / 32768) * 180 / np.pi
    return t-t[events[0]['frame']], d, q, target


def overview(name):
    t,d,q,target = load(name)
    fig, ax = plt.subplots(4, 1, figsize=(12, 8), sharex=True,
                           gridspec_kw={'height_ratios':[2,1.3,1.2,.6]})
    ax[0].plot(t,target, label='下发目标', color='#334155', lw=1.4)
    ax[0].plot(t,d[:,0]*180/32768, label='规划参考', color='#db8e25', lw=1)
    ax[0].plot(t,q, label='编码器位置', color='#087f8c', lw=1)
    ax[0].set_ylabel('位置 / °'); ax[0].legend(ncol=3,loc='upper right')
    ax[0].set_title(name+'：真实下发目标与实测反馈',loc='left')
    ax[1].axhspan(-.3,.3,color='#d6f1e2')
    ax[1].plot(t,target-q,color='#087f8c',lw=.7)
    ax[1].set_ylim(-.4,.4);ax[1].set_ylabel('到位误差 / °')
    ax[1].text(.01,.9,'放大显示 ±0.3° 验收区；运动段误差在图外',transform=ax[1].transAxes,fontsize=8)
    ax[2].plot(t,d[:,7]/1000,label='实际 Iq',lw=.7,color='#087f8c')
    ax[2].plot(t,d[:,9]/1000,label='前馈',lw=.7,color='#db8e25')
    ax[2].set_ylabel('电流 / A');ax[2].legend(ncol=2)
    flags=d[:,11].astype(int)&65535
    hold=((flags&3)==2)&((flags&128)!=0)
    ax[3].fill_between(t,0,hold.astype(int),step='post',color='#529b77')
    ax[3].set_yticks([0,1],['未保持','HOLD']);ax[3].set_xlabel('时间 / s（主机接收锚点）')
    for a in ax:a.grid(alpha=.18);a.set_xlim(0,max(t))
    fig.tight_layout(); fig.savefig(OUT / (name+'.png'));plt.close(fig)


def comparison():
    names=['trial04_poskp_0p5','trial25_final_repeat']
    fig,axes=plt.subplots(2,2,figsize=(12,6),sharex='col')
    for row,name in enumerate(names):
        t,d,q,target=load(name)
        labels=['早期实机：位置 Kp=0.5，旧捕获逻辑','最终：位置 Kp=8，捕获修复与速度修正余量']
        axes[row,0].plot(t,target,color='#334155',lw=1,label='下发目标')
        axes[row,0].plot(t,q,color='#087f8c',lw=1,label='反馈')
        axes[row,0].set_title(labels[row],loc='left',fontsize=10)
        axes[row,0].set_ylabel('位置 / °');axes[row,0].set_xlim(0,24)
        axes[row,1].axhspan(-.3,.3,color='#d6f1e2')
        axes[row,1].plot(t,target-q,color='#087f8c',lw=.7)
        axes[row,1].set_ylim(-.6,.6);axes[row,1].set_xlim(0,24)
        axes[row,1].set_title('目标误差局部放大',loc='left',fontsize=10)
        axes[row,1].set_ylabel('误差 / °')
    for a in axes.flat:a.grid(alpha=.18)
    for a in axes[-1]:a.set_xlabel('时间 / s')
    axes[0,0].legend(ncol=2)
    fig.text(.02,.01,'早期每点7 s，最终每点6 s；角度与时钟锚点来自各轮记录，不能把两条曲线当作同一输入回放。',fontsize=8)
    fig.tight_layout(rect=(0,.035,1,1));fig.savefig(OUT/'before_after.png');plt.close(fig)


if __name__ == '__main__':
    for name in sys.argv[1:]:overview(name)
    if (OUT/'trial25_final_repeat'/'trial.json').exists():comparison()
