"""离线比较当前 C 测速器；仅生成宿主程序和证据，不连接硬件。

相同角度序列输入两种算法。带宽模型假设刚性惯量、电流环一阶 750 Hz、
Ki/Kp=10 s^-1；计入速度环保持与可配置附加延迟，不能替代台架 FRF。
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests/unit/native"))
from run_position_servo_tests import function_source

ENCODER = ROOT / "firmware/platform/stm32g4/bsp/encoder"
DT = 1 / 20000


def verify_configuration():
    """此实验的解析模型有明确适用配置；配置变化时拒绝输出过期结论。"""
    header = ENCODER.with_suffix(".h").read_text(encoding="utf-8")
    hardware = ENCODER.with_name("hw_conf.h").read_text(encoding="utf-8")
    expected = {
        "ENCODER_VELOCITY_WINDOW4_TAPS": 40,
        "ENCODER_PLL_OMEGA_N_LOW_SPEED_HZ": 600,
        "ENCODER_PLL_OMEGA_N_HIGH_SPEED_HZ": 300,
        "ENCODER_PLL_ZETA": 0.707,
        "ENCODER_PLL_SCHED_LOW_RAD_S": 5,
        "ENCODER_PLL_SCHED_HIGH_RAD_S": 10,
        "PWM_TIM_FREQ": 20000,
        "SPEED_LOOP_FREQ": 2000,
    }
    for name, value in expected.items():
        match = re.search(r"^#define\s+" + name + r"\s+([\d.]+)", header + "\n" + hardware, re.M)
        if match is None or float(match[1]) != value:
            raise RuntimeError(f"{name} 已变化，需要重新核对实验模型")


def build_runner(cc, out):
    """抽取当前结构体、宏和实际 C 函数，避免另写一套估计器。"""
    header = ENCODER.with_suffix(".h").read_text(encoding="utf-8")
    header = header[:header.index("#define brd_enc_spi")]
    header = header.replace('#include "main.h"', "").replace('#include "data_type.h"', "")
    source = ENCODER.with_suffix(".c").read_text(encoding="utf-8")
    fixture = header + "\n#endif\n" + r'''
#include <stdio.h>
#include <string.h>
#include <io.h>
#include <fcntl.h>
#define _PI 3.14159265358979323846f
#define _2PI (2.0f * _PI)
#define Current_Ts (1.0f / 20000.0f)
#define SPEED_LOOP_DIVIDER 10U
''' + "\n".join(function_source(source, name) for name in (
        "Encoder_ResetVelocity", "Encoder_UpdateVelocityPll", "Encoder_UpdateVelocityWindow4"
    )) + r'''
int main(void)
{
    Encoder_TypeDef pll = {0}, win = {0};
    uint16_t sample, previous = 0;
    int first = 1;
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);
    Encoder_ResetVelocity(&pll);
    Encoder_ResetVelocity(&win);
    while (fread(&sample, sizeof(sample), 1, stdin) == 1) {
        int32_t delta = (int32_t)sample - previous;
        float values[3];
        if (delta > ENCODER_Q15_HALF_TURN) { delta -= ENCODER_Q15_CPR; }
        if (delta < -ENCODER_Q15_HALF_TURN) { delta += ENCODER_Q15_CPR; }
        pll.linearized_q15 = sample;
        win.linearized_q15 = sample;
        if (!first) {
            Encoder_UpdateVelocityPll(&pll, 21);
            Encoder_UpdateVelocityPll(&win, 21);
            Encoder_UpdateVelocityWindow4(&win, 21, delta);
        }
        previous = sample;
        first = 0;
        values[0] = pll.vel_mech_fast;
        values[1] = win.vel_mech_fast;
        values[2] = win.vel_mech;
        if (fwrite(values, sizeof(values), 1, stdout) != 1) { return 2; }
    }
    return ferror(stdin) ? 3 : 0;
}
'''
    path = out / "estimator.c"
    path.write_text(fixture, encoding="utf-8")
    exe = out / "estimator.exe"
    command = [cc] + (["cc"] if Path(cc).stem == "zig" else [])
    subprocess.run(command + ["-std=c99", "-O2", "-Wall", "-Wextra", "-Werror",
                             str(path), "-o", str(exe)], check=True)
    return exe


def replay(exe, angle_rad):
    """Q15 为线性化后的角度；原始 TLE5012B 有效角为 15 位，LUT 可产生奇数值。"""
    samples = (np.rint(angle_rad * 65536 / (2 * np.pi)).astype(np.int64) % 65536).astype("<u2")
    result = subprocess.run([str(exe)], input=samples.tobytes(), capture_output=True, check=True)
    return np.frombuffer(result.stdout, dtype="<f4").reshape(-1, 3).copy()


def phasor(t, values, frequency_hz):
    """含直流和线性趋势的最小二乘；不要求捕获包含整周期。"""
    w = 2 * np.pi * frequency_hz
    basis = np.column_stack((np.ones(len(t)), t - np.mean(t), np.cos(w * t), np.sin(w * t)))
    fit = np.linalg.lstsq(basis, values, rcond=None)[0]
    return fit[2] - 1j * fit[3]


def response(frequency_hz, wn_hz=None):
    """由当前离散递推推导的线性频响；PLL 的输出是更新后的 omega。"""
    w = 2 * np.pi * np.asarray(frequency_hz)
    if wn_hz is None:
        return (1 - np.exp(-1j * w * 40 * DT)) / (1j * w * 40 * DT)
    wn = 2 * np.pi * wn_hz
    z = np.exp(1j * w * DT)
    q = z - 1
    kp, ki = 2 * 0.707 * wn * DT, wn * wn * DT
    return z * ki * q / (q * q + kp * q + DT * ki) / (1j * w)


def bandwidth_model(wn_hz, loop_hz, extra_delay_s):
    """相位余量 60 度处的候选穿越频率；机械参数未知，增益用归一化量表示。"""
    f = np.geomspace(1, loop_hz * 0.45, 10000)
    w = 2 * np.pi * f
    z = np.exp(1j * w / loop_hz)
    controller = 1 + (10 / loop_hz) / (1 - 1 / z)
    forward = controller / (1j * w) / (1 + 1j * f / 750)
    forward *= np.sinc(f / loop_hz) * np.exp(-1j * w * (0.5 / loop_hz + extra_delay_s))
    loop = forward * response(f, wn_hz)
    pm = 180 + np.degrees(np.unwrap(np.angle(loop)))
    # PI 积分项使低频裕度也小于 60 度，取裕度峰值之后的高频下降交点。
    peak = int(np.argmax(pm))
    index = peak + np.flatnonzero(pm[peak:] <= 60)[0]
    if pm[peak] <= 60 or f[index] < 20:
        raise RuntimeError("没有可用的高频相位裕度交点")
    gain = 1 / abs(loop[index])
    actual = gain * forward / (1 + gain * loop)
    measured = actual * response(f, wn_hz)
    crossings = np.flatnonzero(abs(actual) <= 1 / np.sqrt(2))
    return {"estimator_wn_hz": wn_hz, "speed_loop_hz": loop_hz,
            "extra_delay_us": extra_delay_s * 1e6, "pm_deg": float(pm[index]),
            "crossover_hz": float(f[index]),
            "model_true_speed_minus3db_hz": float(f[crossings[0]]) if len(crossings) else None,
            "model_true_speed_peak_db": float(20 * np.log10(max(abs(actual)))),
            "model_measured_speed_peak_db": float(20 * np.log10(max(abs(measured))))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    verify_configuration()
    exe = build_runner(args.cc, out)
    t = np.arange(40000) * DT
    records, checks = [], []
    for speed in (0, 0.02, 0.1, 3, -3, 7.5, 15):
        values = replay(exe, 1.2 + speed * t)
        means = values[len(t) // 2:].mean(axis=0)
        checks.append({"speed_rad_s": speed, "mean_rad_s": means.tolist()})
        # 0.02 rad/s 用于揭示窗口死区，不强行视作线性估计器。
        if abs(speed) >= 3 and np.max(abs(means - speed)) > 0.015:
            raise RuntimeError(f"恒速/回绕检查失败：{speed}: {means}")
        if speed == 0 and np.max(abs(values)) != 0:
            raise RuntimeError("静止初始化产生了非零速度")
    for speed, wn in ((3, 600), (-3, 600), (7.5, 450), (15, 300)):
        for frequency in (5, 20, 50, 100, 150, 200, 250, 300):
            amp = 0.3
            angle = 1.2 + speed * t + amp * np.sin(2 * np.pi * frequency * t) / (2 * np.pi * frequency)
            values = replay(exe, angle)
            for col, name, bandwidth in ((0, "pll", wn), (1, "window2ms", None)):
                h = phasor(t[20000:], values[20000:, col], frequency) / amp
                expected = response(frequency, bandwidth)
                # 过渡调度有非线性，量化对高频小角度激励有误差；允许 6% 复数偏差。
                if abs(h - expected) > 0.06:
                    raise RuntimeError(f"C/解析频响不一致：{speed}, {frequency}, {name}, {h}, {expected}")
                records.append({"speed_rad_s": speed, "frequency_hz": frequency,
                                "estimator": name, "gain_db": float(20 * np.log10(abs(h))),
                                "phase_deg": float(np.degrees(np.angle(h)))})
    rng = np.random.default_rng(20260924)
    noise = rng.normal(0, 1e-4, len(t))
    noise_records = []
    for speed in (3, 15):
        clean = replay(exe, 1.2 + speed * t)
        noisy = replay(exe, 1.2 + speed * t + noise)
        noise_records.append({"speed_rad_s": speed, "angle_white_noise_sigma_rad": 1e-4,
                              "delta_velocity_std_rad_s": np.std((noisy-clean)[20000:], axis=0).tolist()})
    models = [bandwidth_model(wn, rate, delay) for wn in (None, 300, 450, 600)
              for rate in (2000, 4000) for delay in (0, 0.00015)]
    result = {"kind": "offline_actual_C_and_assumed_linear_plant", "checks": checks,
              "bode": records, "same_input_noise": noise_records, "models": models,
              "source_sha256": {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                for p in (ENCODER.with_suffix('.c'), ENCODER.with_suffix('.h'))}}
    (out / "offline_summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    np.savez(out / "noise_inputs.npz", time_s=t, noise_rad=noise)
    print(json.dumps({"checks": checks, "noise": noise_records,
                      "models_2khz_extra150us": [m for m in models if m['speed_loop_hz']==2000 and m['extra_delay_us']>0]}, indent=2))


if __name__ == "__main__":
    main()
