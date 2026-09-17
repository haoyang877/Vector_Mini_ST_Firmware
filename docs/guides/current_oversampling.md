# 电流采样：4×硬件过采样

## 实现与配置归属

ADC2 在 `firmware/platform/stm32g4/cubemx/Core/Src/adc.c` 的 `USER CODE ADC2_Init 2` 中启用注入组
4×硬件过采样、不右移。每次原有 TIM1 CC4 触发，硬件为每个注入
rank 连续完成4次采样和转换，将完整累加结果写入对应 JDR。
顺序仍为 Ia、Ib、Ic、Vbus；Vbus 同属此组，因此也使用4×。
ADC1 温度采样不变。PWM及电流环的名义频率保持20 kHz。

此配置属于当前板级硬件初始化适配，不属于FOC算法。没有新增上层
Vendor依赖，没有修改厂家SDK。后续架构迁移时，该适配应进入
STM32G4 motor HAL Port，并由BSP绑定具体ADC和采样参数。

CubeMX `.ioc` 中的 `EnableInjectedOversampling=DISABLE` 是**生成基线**，
不是最终运行配置：初始化末尾的USER CODE覆盖为4×。这样不依赖手工
猜测CubeMX私有配置字段，重新生成并保留USER CODE后仍会应用4×。
审查配置时必须检查最终初始化代码；不能仅依据 `.ioc` 判断是否启用。
若以后改由CubeMX原生配置此功能，应同时迁移该覆盖逻辑和回归测试。

已有工作区改动已经使用 `ADC_IT_JEOS` 并关闭 `ADC_IT_JEOC`；
回调还检查ADC2的JEOS标志。本次保留这些改动，避免在尚未完成
Ia/Ib/Ic/Vbus整组转换时执行FOC。没有按每个子样本运行FOC。

## 数值兼容性

- JDR 输出为 `sample0 + sample1 + sample2 + sample3`，最大 16380。
- 软件乘 `ADC2_SUM_TO_COUNTS=0.25f`，保留 0.25 个原 ADC count 的分辨率。
- 电流和 Vbus 均先乘 0.25；换算系数与持久化偏置仍使用原 12 位单位，零电流约 2048。
- 偏置累加使用整数、最终除法与运行偏置使用 float，避免截断小数。参数原本即以 float 保存，schema 不变。
- 2 mΩ 配置的数字电流步进由约 40.29 mA 降到 10.07 mA；这不等同于真实有效位或绝对精度保证。不会纠正增益误差、
  零偏误差、无效采样窗口或同步开关干扰。

## 时序预算与未验证项

按当前代码的170 MHz HCLK、ADC同步4分频、12位分辨率计算：

- ADC时钟42.5 MHz。
- 单次相电流转换：`(6.5 + 12.5) / 42.5 MHz = 0.4471 us`。
- 4×三相转换合计：`3 * 4 * 19 / 42.5 MHz = 5.3647 us`。
- 加入4× Vbus：`4 * (3 * 19 + 25) / 42.5 MHz = 7.7176 us`。
- 相邻两相的采样时间偏差约1.7882 us；三相仍然不是同时采样。
- 整组完成相对原配置推迟约5.7882 us。即使中断频率不变，
  采样到PWM生效的延迟以及可用运算时间也发生了变化。

以上仅为采样和转换周期计算，不包含触发同步、IRQ响应与FOC运算。
12位转换周期及注入过采样行为参考
[ST RM0440，第21章](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)。

**当前变更不是全占空比范围的实机验收。** 三相低侧分流、单ADC顺序
转换和固定CC4触发的原有窗口限制仍然存在；4×会扩大采样跨度。
不能因为7.72 us小于50 us就判断高占空比采样有效，也不能直接沿用
1×镜像的IRQ计时结论。未以推测的运放建立时间或死区裕量修改占空比
上限；高调制度闭环使用前必须完成以下验证。

1. 只读核对ADC2最终CFGR2：JOVSE=1、OVSR=1（4×编码）、OVSS=0、
   ROVSE=0、TROVS=0；确认IER只使能JEOS完成事件，非JEOC。
2. 同步观察PWM、分流运放输出、ADC采样时刻和FOC起止。
   运放/RC建立时间、实际触发延迟、最窄有效采样窗口当前均为UNKNOWN。
3. 覆盖各扇区和占空比，确认每相的全部4个采样窗口位于该相有效
   低侧导通区间，观察电流和残差及异常尖峰。
4. 验证20 kHz完整样本节奏、FOC最坏执行时间和实际CCR预装载截止时间，
   覆盖运行模式、校准和通信负载；主机测试不能证明这些硬件时序。
5. 若窗口不够，依据实测建立占空比/调制度限制、移动采样点，或核实
   PCB引脚映射后改为双ADC同步采两相并重构第三相。

## 软件验证

`tests/unit/native/test_current_oversampling.py --cc <native C compiler or zig.exe>`
提取并执行实际ADC2初始化函数，使用实际Vendor头文件和LL寄存器操作、
RAM中的模拟寄存器及HAL记录桩，检查：

- 四个rank的通道、次序、采样时间和硬件触发配置；
- 初始化末尾确实覆盖生成基线，注入组4×、不右移、规则组不启用；
- 重复初始化、无关寄存器位和序列保持；
- 全部12位直流码及零偏附近样本的数值尺度。

`test_current_precision.py` 执行实际启动偏置和电流换算代码，覆盖 2/6 mΩ、浮点偏置、四分之一 ADC 步进、实际 1 A 换算和无效偏置保护。2026-09-17 的升级没有增加 ADC 转换次数或改变触发时序；实测与日志见 `outputs/cogging_precision_20260917/`。

已有 `tests/unit/native/run_position_servo_tests.py` 覆盖JEOS完整序列回调、ADC共享
中断分派和板级启动，另构建普通及HIL两个Keil目标。2026-09-09 的日志放在
`outputs/current_oversampling/`，当时未执行烧录、电机运行或模拟量实测。
2026-09-17 的浮点精度改版已进行低速齿槽标定实机验证，日志另存于
`outputs/cogging_precision_20260917/`；这不替代高调制度采样窗口验证。

历史 2026-09-09（当时右移 2 位）验证结果：新增寄存器配置测试及已有主机回归测试全部通过；
`Vector_Mini_ST` 和 `Vector_Mini_ST_HIL` 使用ARMCC 5.06 update 7完整
重编译，均为0错误、0警告，并生成HEX。实机时序与噪声改善尚未验收。
