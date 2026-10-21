> 历史归档（2026-10-21）：重复标定说明合并到当前标定指南。 本文的“当前”指原始基线；现行入口见[当前说明](../../guides/calibration_process.md)。归档不新增执行授权。

# 编码器 LUT 与电角零位标定

日期：2026-09-29。当前入口为显式维护任务，旧 `ModeNow=13/15` 和 `Save_Param` 模式入口已删除。

## 入口与生命周期

编码器线性化使用 `MOTOR_MAINTENANCE_ENCODER_LUT`，电角零位使用
`MOTOR_MAINTENANCE_ELECTRICAL_ZERO`。请求由 `MotorStateMachine_RequestMaintenance()`
或带来源身份的请求入口受理，返回 ticket；主状态机拥有准入、功率许可、取消和保存衔接。
worker 只返回功能结果，不改写主状态或自行启相关断。

当前 CAN Type 160 的线路 kind 1 对应 LUT、kind 2 对应电角零位；Type 161 查询 ticket，
Type 162 取消 ticket。线路 kind 与内部枚举分别定义，必须显式映射。
任务 COMPLETED 与 Flash COMMITTED 分开；只有效果状态确认提交，才能认为结果已经持久化。
详见 [标定线路契约](../protocols/yg_protocol_calibration_wire_v1.md) 和
[主状态机](../../architecture/motor_state_machine_v1.md)。

## LUT 参考坐标与算法

1024 点机械角 LUT 以 `encoder.directed_q15` 为输入，即统一安装方向后的原始角度，
与运行期查表使用同一坐标。标定复用无感静态对齐、开环拖动、观测器切换及速度闭环；
只在观测器有效且转速稳定后采样。

对齐末尾 0.1 s 的圆周均值定义本次机械原点，随后捕获编码器正向通过该原点的事件，
插值得到连续观测器电角位置 `observer_position_origin`。极对数为 `p` 时：

```text
reference_q15 = uint16((theta_obs - observer_position_origin) * 65536 / (2*pi*p))
```

此位置仅在本次连续观测器运行期间有效，不是断电保持的绝对多圈位置。
观测器复位导致 `position_epoch` 变化时，中止任务，不提交该次候选。

启动及采样默认值归 `firmware/motor/motor_startup_profile.h`：带阻尼环装配当前为
20 rad/s、5 个采集圈，无阻尼环装配为 15 rad/s、10 个采集圈。
稳态检查窗口为 0.5 s、速度误差上限 20%；每个 LUT 桶至少 16 个样本。
候选构建每拍处理数量有界，之后验证一机械圈。RMS 上限为 256 Q15 counts，
峰值上限为 1024 Q15 counts；满足条件才替换运行 LUT。

采样误差和占 4096 B，样本计数与候选 LUT 复用 2048 B，工作区总计 6144 B，
从项目有界内存池借用。编译检查保证容量，不依赖旧工程的 16 KiB 堆假设。
任务结束或取消须归还工作区；减速与退转矩后由主状态机执行关相并认领保存。

## 后续校准

LUT 成功只设置 `ENC_CALIB_LINEARIZED`，清除电角零位、机械零位及其有效标志。
之后需单独执行电角零位任务和机械零位设置，建立相应有效标志，才能满足完整标定条件。
电角零位任务依赖已经线性化的编码器；旧电压开环扫角模式不再提供兼容入口。
当前参数只装载 schema 12，烧录或实机运动须按独立台架流程验收。
