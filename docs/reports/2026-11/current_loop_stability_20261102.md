# 电流环稳定性与采样噪声实测 2026-11-02

当前Id/Iq实际PI与1kHz目标整定一致，更新周期50μs（20kHz）。关相三相采样噪声为7.67～9.31mA RMS。
本轮零电流以及Id=0.3A/0.75A、Iq=0的已测工况通过；未进行扫频或持续非零Iq锁轴试验。

## 当前带宽和参数

从正在运行的板卡只读取得配置及FOC控制器字段，结构尺寸/字段偏移使用与ARM一致的短枚举布局校验。
整段Flash与本地HEX覆盖字节一致，未依赖旧报告推断当前固件。

| 参数 | Id轴 | Iq轴 |
| --- | ---: | ---: |
| Kp，V/A | 0.895354 | 0.895354 |
| Ki，V/(A·s) | 3455.752 | 3455.752 |
| PI更新周期 | 50μs | 50μs |
| 电感 | 142.5μH | 142.5μH |
| 相电阻 | 0.55Ω | 0.55Ω |
| Kp/(2πL)、Ki/(2πR) | 约1000Hz | 约1000Hz |

1000Hz是按当前模型和增益得到的整定带宽，不是已测得的闭环−3dB带宽；ADC/PWM延迟和真实电机参数影响频响。
独立扫频、幅相曲线及实际相位裕度本轮未测。

## 当日实测

AT32F456＋YQ-5708（8:1），节点1；固件SHA256
`4cdc9ac03294aff63d94ab3e82d9e85fc28917ad6e1b6046f4c960a2c7a9a33a`。
本次上电已从Flash恢复电零位18681、标志3；10月31日的临时RAM零位14162不再生效。
本轮以实际启动标定为基线，没有重做标定、修改PI、烧录或写入Flash。

| 关相通道 | 去均值RMS范围 | 最大峰峰值 |
| --- | ---: | ---: |
| Ia | 7.78～8.76mA | 47.85mA |
| Ib | 7.76～9.31mA | 52.88mA |
| Ic | 7.67～9.19mA | 50.37mA |

原码标准差0.76～0.92counts，最大峰峰值5.25counts，满足原建议≤2/≤12counts。
这是板载采样噪声，不是外部电流探头测得的绕组PWM纹波。

| 工况 | Id误差去均值RMS | Iq误差去均值RMS | 已测结论 |
| --- | ---: | ---: | --- |
| Id=0、Iq=0 | 8.15mA | 8.67mA | 通过 |
| Id=0.3A、Iq=0 | 8.11～8.63mA | 7.68～8.93mA | 三次独立5秒运行通过 |
| Id=0.75A、Iq=0 | 7.96～8.51mA | 8.27～9.46mA | 三次独立5秒运行通过 |

以上RMS来自20kHz快窗；各Id档的三个窗口分别在独立运行的约0.2、2、4秒捕获，不能拼成连续快采样。
每窗128点、6.35ms，所有tick间隔经核对；另有约1.016秒的关相抽取窗及每次完整运行的主机慢趋势。
慢趋势最大去均值RMS9.75mA、最大峰峰值57.98mA；快窗最大平均误差绝对值0.38mA。
在这些窗口及慢趋势中未见持续增长振荡，满足原建议：误差/RMS≤60mA、峰峰值≤300mA。

## 边界与收尾

用户相电流3A、速度50rad/s上限保持；本轮额外限制速度5rad/s、行程1rad、时限8秒、租约500ms。
实际只施加0/0.3/0.75A的Id，Iq目标为0。输出轴没有锁定装置，不能外推非零Iq锁轴稳定性。
最终数字关相、故障0、无效样本0，1024项LUT、标定标志和电零位与开始一致。
速度闭环未在本轮继续测试。

## 原始证据

- [逐项结果与波形](../../../outputs/current-loop-stability-20261102/report.html)
- [本轮汇总](../../../outputs/current-loop-stability-20261102/summary.json)
- [实际PI及电机参数](../../../outputs/current-loop-stability-20261102/active_parameters.json)
- [采样噪声与电流快窗](../../../outputs/current-loop-stability-20261102/current_noise_and_ripple.png)
- [完整运行慢趋势](../../../outputs/current-loop-stability-20261102/current_long_trend.png)
- [原始窗口和CSV](../../../outputs/current-loop-stability-20261102/raw/)、[原始CAN帧](../../../outputs/current-loop-stability-20261102/frames.jsonl)
- [最终关相与标定核对](../../../outputs/current-loop-stability-20261102/final_state.json)
- [本轮quick验证](../../../outputs/runs/current-loop-stability-20261102/summary.json)：格式、lint、工程布局和离线测试通过。
