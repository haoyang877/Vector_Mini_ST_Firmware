"""离线验收：把测速估计器延迟折算到 2 kHz 转速环的可用带宽（不访问硬件）。

对照两代估计器（均按固件离散实现）：
- 旧：2 kHz 16 点割线（8 ms 窗口），`v=(theta[k]-theta[k-16])/(16*0.5ms)`；
- 新：20 kHz 二阶角度跟踪观测器（PLL），默认 ωn=150 Hz、ζ=0.707。

方法：扫频测出"真实转速 -> 估计转速"的群延迟；以**同一连续噪声 PSD**（逐样本 σ 按
sqrt(fs) 缩放）比较噪声；再按延迟预算（含 2 kHz 半拍与电流环滞后）估算转速环
crossover 与相位裕度的关系。给出"默认带宽点"和"与旧估计器等噪声点"两组结论。

局限：角度噪声按白噪声建模；真实磁编码器噪声经内部更新/滤波后相关，会低估 20 kHz
采样下的噪声，故本验收对 PLL 偏保守。
"""

import unittest

import numpy as np

T_FAST = 1.0 / 20000.0
T_SLOW = 1.0 / 2000.0
PLL_WN_HZ = 300.0  # 与固件 ENCODER_PLL_OMEGA_N_HZ 一致（带宽优先默认点）
PLL_ZETA = 0.707
SECANT_N = 16
CURRENT_LOOP_LAG_S = 1.0e-4  # 近似：电流环等效滞后


def secant_velocity(theta_slow):
    """旧估计器：2 kHz 割线，等价于 16 点增量和的滑动平均。"""
    n = len(theta_slow)
    out = np.zeros(n)
    out[SECANT_N:] = (theta_slow[SECANT_N:] - theta_slow[:-SECANT_N]) / (SECANT_N * T_SLOW)
    return out


def pll_velocity(theta_fast, wn_hz=PLL_WN_HZ):
    """新估计器：与固件 Encoder_UpdateVelocityPll 相同的二阶角度跟踪观测器。"""
    wn = 2.0 * np.pi * wn_hz
    kp = 2.0 * PLL_ZETA * wn * T_FAST
    ki = wn * wn * T_FAST
    theta_est = 0.0
    omega = 0.0
    out = np.zeros(len(theta_fast))
    for i, theta_meas in enumerate(theta_fast):
        err = theta_meas - theta_est
        err = (err + np.pi) % (2.0 * np.pi) - np.pi
        theta_est += T_FAST * omega + kp * err
        omega += ki * err
        out[i] = omega
    return out


def velocity_response(kind, freq, wn_hz=PLL_WN_HZ):
    """返回 (gain, phase_rad)：真实转速 -> 估计转速。phase<0 表示滞后。"""
    dt = T_FAST if kind == "pll" else T_SLOW
    w = 2.0 * np.pi * freq
    n = int(max(4.0 / freq, 0.5) / dt)
    t = np.arange(n) * dt
    theta = -np.cos(w * t) / w  # 使 d(theta)/dt = sin(w t)
    v_est = pll_velocity(theta, wn_hz) if kind == "pll" else secant_velocity(theta)
    s = slice(n // 2, n)
    a = 2.0 * np.mean(v_est[s] * np.sin(w * t[s]))
    b = 2.0 * np.mean(v_est[s] * np.cos(w * t[s]))
    return float(np.hypot(a, b)), float(np.arctan2(b, a))


def group_delay_s(kind, freq=5.0, wn_hz=PLL_WN_HZ):
    _, phase = velocity_response(kind, freq, wn_hz)
    return -phase / (2.0 * np.pi * freq)


def noise_std(kind, wn_hz=PLL_WN_HZ, sigma_slow=1.0e-4, samples=200000, seed=12345):
    """同一连续噪声 PSD：逐样本 sigma 按 sqrt(fs) 缩放。sigma_slow 取真实角度噪声量级。"""
    rng = np.random.default_rng(seed)
    dt = T_FAST if kind == "pll" else T_SLOW
    sigma = sigma_slow * np.sqrt(T_SLOW / dt)
    theta = rng.normal(0.0, sigma, samples)
    v = pll_velocity(theta, wn_hz) if kind == "pll" else secant_velocity(theta)
    return float(np.std(v[samples // 2 :]))


def total_delay_s(kind, wn_hz=PLL_WN_HZ):
    return (
        (T_FAST if kind == "pll" else T_SLOW) / 2.0
        + group_delay_s(kind, 5.0, wn_hz)
        + CURRENT_LOOP_LAG_S
    )


def crossover_hz(kind, phase_margin_deg, wn_hz=PLL_WN_HZ):
    """延迟受限的 crossover：仅计积分器 -90° 与延迟，留出相位裕度。"""
    allowed = np.deg2rad(180.0 - 90.0 - phase_margin_deg)
    return float(allowed / total_delay_s(kind, wn_hz) / (2.0 * np.pi))


def equal_noise_wn_hz():
    """二分求 PLL 噪声等于旧割线时的 ωn。"""
    target = noise_std("secant")
    lo, hi = 5.0, PLL_WN_HZ
    for _ in range(24):
        mid = 0.5 * (lo + hi)
        if noise_std("pll", mid) > target:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


class SpeedLoopBandwidth(unittest.TestCase):
    def test_pll_has_lower_delay_than_secant(self):
        tau_old = group_delay_s("secant")
        tau_new = group_delay_s("pll")
        # 旧割线群延迟约 (N-1)/2 拍 = 3.75 ms；新 PLL(90 Hz) 约 2.5 ms。
        self.assertGreater(tau_old, 3.0e-3)
        self.assertLess(tau_new, 3.0e-3)
        self.assertLess(tau_new, 0.7 * tau_old)

    def test_pll_raises_delay_limited_bandwidth(self):
        for pm in (45.0, 60.0):
            self.assertGreater(crossover_hz("pll", pm), 1.5 * crossover_hz("secant", pm))

    def test_type2_loop_tracks_constant_speed_without_lag(self):
        # 常速下 PLL 稳态误差为 0（type-2），低频增益接近 1。
        gain, _ = velocity_response("pll", 1.0)
        self.assertGreater(gain, 0.99)

    def test_equal_noise_point_still_faster(self):
        wn = equal_noise_wn_hz()
        # 与旧割线等噪声时，PLL 延迟仍明显更低、带宽更高。
        self.assertLess(group_delay_s("pll", 5.0, wn), group_delay_s("secant"))
        self.assertGreater(crossover_hz("pll", 45.0, wn), crossover_hz("secant", 45.0))

    def test_estimator_bode(self):
        # 估计器波特图：真实角速度 -> 估计角速度（旧割线 vs 新 PLL）。
        lines = [
            "",
            "estimator Bode (true velocity -> estimated velocity):",
            "   f[Hz]   |H|old[dB]  ph_old[deg]   |H|new[dB]  ph_new[deg]",
        ]
        for f in (1.0, 2.0, 5.0, 10.0, 20.0, 40.0, 60.0, 80.0, 100.0):
            go, po = velocity_response("secant", f)
            gn, pn = velocity_response("pll", f)
            lines.append(
                "  %5.0f   %9.2f  %11.1f   %8.2f  %11.1f"
                % (f, 20.0 * np.log10(go), np.degrees(po), 20.0 * np.log10(gn), np.degrees(pn))
            )
        print("\n".join(lines))
        # 低频两者增益都接近 1；新 PLL 在 40 Hz 处相位滞后应明显小于旧割线。
        self.assertLess(abs(velocity_response("pll", 5.0)[0] - 1.0), 0.05)
        self.assertGreater(velocity_response("pll", 40.0)[1], velocity_response("secant", 40.0)[1])

    def test_summary(self):
        agg = 150.0
        lines = [
            "",
            "speed-loop bandwidth (delay-limited, offline; white angle noise):",
            f"  group delay : secant {group_delay_s('secant') * 1e3:.2f} ms"
            f" | PLL {PLL_WN_HZ:.0f}Hz {group_delay_s('pll') * 1e3:.2f} ms"
            f" | PLL {agg:.0f}Hz {group_delay_s('pll', 5.0, agg) * 1e3:.2f} ms",
            f"  total delay : {total_delay_s('secant') * 1e3:.2f} ms"
            f" -> {total_delay_s('pll') * 1e3:.2f} ms -> {total_delay_s('pll', agg) * 1e3:.2f} ms",
            f"  velocity noise vs secant : {noise_std('pll') / noise_std('secant'):.2f}x"
            f" @{PLL_WN_HZ:.0f}Hz, {noise_std('pll', agg) / noise_std('secant'):.2f}x @{agg:.0f}Hz",
        ]
        for pm in (45.0, 60.0):
            lines.append(
                f"  crossover @PM {pm:.0f} : {crossover_hz('secant', pm):.0f} Hz"
                f" -> {crossover_hz('pll', pm):.0f} Hz ({PLL_WN_HZ:.0f}Hz)"
                f" -> {crossover_hz('pll', pm, agg):.0f} Hz ({agg:.0f}Hz)"
            )
        print("\n".join(lines))
        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
