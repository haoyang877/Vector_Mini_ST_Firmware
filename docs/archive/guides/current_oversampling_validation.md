# 过采样的历史验证与结果

> 从当前采样说明提取的历史附录；结果只对原基线有效。当前实现见[采样指南](../../guides/current_oversampling.md)。



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
