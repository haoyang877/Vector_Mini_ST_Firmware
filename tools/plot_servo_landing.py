"""Compare measured pre-capture motion for matching targets before/after tuning."""
from pathlib import Path
import json
import sys
from plot_servo_hil import load, plt, OUT, np


def compare(before, after):
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex='col')
    for name, label, color in [(before, '修改前', '#d88625'), (after, '修改后', '#087f8c')]:
        time, data, position, targets = load(name)
        trial = json.loads((OUT/name/'trial.json').read_text())
        events = [e for e in trial['events'] if e['opcode'] == 3]
        for col in range(2):
            a, b = events[col]['frame'], events[col+1]['frame']
            direction = np.sign(targets[a]-position[a])
            t = time[a:b]-time[a]
            axes[0,col].plot(t, direction*(targets[a:b]-position[a:b]), color=color, label=label)
            axes[1,col].plot(t, direction*data[a:b,5]*180/np.pi/10000, color=color, label=label)
            axes[2,col].plot(t, direction*data[a:b,9]/1000, color=color, label=label)
    for col, title in enumerate(['到 +85°：着陆过程', '+85° → −85°：170°换向着陆']):
        axes[0,col].set_title(title, loc='left')
        axes[0,col].axhspan(-.3,.3,color='#d6f1e2',alpha=.6)
        axes[0,col].set_ylim(-.4,5)
        axes[1,col].axhline(2,color='#94a3b8',ls=':',lw=.8)
        axes[2,col].set_xlabel('下发目标后的时间 / s')
        for row in range(3):
            axes[row,col].set_xlim((2.4,4.1) if col == 0 else (4.2,5.95))
            axes[row,col].grid(alpha=.2)
    axes[0,0].set_ylabel('到目标剩余角度 / °')
    axes[1,0].set_ylabel('沿目标方向速度 / °/s')
    axes[2,0].set_ylabel('沿目标方向前馈 / A')
    axes[0,0].legend(ncol=2)
    fig.suptitle('相同45°/s巡航配置的实机波形对比；速度来自连续滤波反馈',fontsize=12)
    fig.tight_layout(); fig.savefig(OUT/'landing_comparison.png'); plt.close(fig)


if __name__ == '__main__':
    compare(sys.argv[1],sys.argv[2])
