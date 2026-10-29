# 编码器反馈、标定与保存

核对日期：2026-10-21。适用当前 `codex/system-refactor` 工作树。
当前入口是 CAN 单项维护请求，不使用旧 USB、HIL 或直接写 Watch 模式启动。

## 当前数据流

[`angle_feedback.c`](../../firmware/motor/feedback/angle_feedback.c)消费平台读帧状态并完成方向、
1024点LUT插值、电角度、多圈位置与速度估计。原始/校正环形角以65536 counts/转表示；
机械位置和速度对控制算法使用rad和rad/s。20kHz快路径更新角度PLL速度，慢拍维护位置。
LUT、电零位和机械零位由 `MotorConfiguration.device.encoder` 保存，反馈运行状态不复制表。

当前[TLE5012B端口](../../firmware/platform/devices/encoder_tle5012b.c)只读角度字，
未读取运行安全字/CRC；`crc_ok=true`是当前驱动返回值，不能作为硬件CRC已经校验的证据。
初始化的STAT等配置快照仅供诊断，不是逐帧运行健康检查。连续SPI失败可触发反馈离线，
但不能据此声称已覆盖磁场状态、所有STAT故障或角度安全字失效。

## 请求顺序与结果

| 业务 | 当前请求 | 完成后的核对 |
| --- | --- | --- |
| LUT线性化 | Type160 kind1，Type161查询原ticket | 任务完成、输出关闭、工作区归还；LUT有效，旧电/机械零位有效位清除 |
| 电角零位 | Type160 kind2；要求已有有效LUT | 电零位有效；普通编码器闭环要求LUT与电零位两位同时有效 |
| 机械零位 | Type160 kind6 | 仅改变机械坐标参考；不能代替电角零位 |
| 维护SAVE | Type160 kind7 | 按原ticket核对真实持久效果 |
| DEFAULTS | Type160 kind8 | 当前设备node保留，其余配置按默认恢复，之后独立保存 |
| 活动参数保存 | Type122，携带expected_revision | 核对真实persisted_revision；与维护SAVE的ticket不同 |

全部请求、阶段、effects和拒绝原因见[维护线路v1.2](../protocols/yg_protocol_calibration_wire_v1_2.md)。
START受理不等于标定完成；查询成功也可能携带FAILED阶段。
任务完成和参数保存是两条独立事实：LUT/零位成功后自动派生独立SAVE，
原ticket的 `COMPLETED/UNCOMMITTED` 不能代替Flash提交。正常收尾在关相下等待保存和空闲，
核对标志及记录；异常立即STOP，不为等待保存延迟停机。
STOP可能取消尚未开始的派生SAVE；RAM结果仍有效时可在满足准入后显式保存当前revision。

## 算法与所有权

LUT任务依次执行静态对齐、无感启动和稳定、找原点、正向采样、分批构表、残差验证、
发布结果与卸流。电零位在已有效LUT上执行d轴对齐及角度均值。
计算由 `encoder_calibration_core` 处理Q15差值/均值、累计、构表和残差；
`encoder_calibration_task` 持有阶段和时间并发布结果。
APP维护协调器负责准入、取消和固定6KiB工作区；主状态机决定功率与资源释放；前台执行Flash。
计算核心不访问HAL、Flash，不拥有第二套运行状态机。

功率标定需满足采样、模型、电压和热保护资格。当前没有齿槽、摩擦、相电阻辨识或整套自动标定。
采样失效、STOP或故障撤销当前动作；释放后才允许工作区复用。详细边界见
[核心维护](../architecture/motor_maintenance_core_v3.md)和[准备资格](../architecture/motor_readiness_v1.md)。

## 存储与证据

保存使用[schema14](../protocols/parameter_schema14_storage_v1.md)，兼容读取有效schema12/13。
旧文档中“旧格式必定拒绝”或固定历史Flash偏移不作为当前恢复规则。
标定一致性不是外部角度计量。历史通过范围见[APP1.5.1实机报告](../reports/2026-10-21-core-bench-acceptance-v1.md)；
后续源码的实机、复位重装载和时序验证仍须匹配候选重新建立证据。
