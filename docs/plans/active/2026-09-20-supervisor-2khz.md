# 监督时基 2 kHz 与三层环路架构 v1.0

日期：2026-09-20。状态：**阶段 1–4 与状态机上移已实施；PR 档全绿 + 实机台架验收通过**；release 档待提交后执行。

## 1. 目标架构（用户 2026-09-20 裁决）

| 层 | 载体 | 内容 |
| --- | --- | --- |
| 高频 | ADC JEOS 中断（20 kHz） | 采样、电角度、电流环、快速故障检测与紧急关断快车道 |
| 中频 | TIM7 中断（2 kHz） | 测速、速度/位置/轨迹控制、主运行状态机、温度等缓变量、慢故障监督、指示器、CAN 服务（S5） |
| 后台 | `while(1)` | 参数读写、通信发送、异步会话 |

## 2. 阶段 1：TIM7 1 kHz → 2 kHz 与内容重标定

- 板级：`.ioc` 周期改为 `500-1`；`MX_TIM7_Init` 在 USER CODE 段覆写 ARR（不手改生成区，
  重新生成后生成值即为 2 kHz，覆写可删除）。
- 契约：`platform/api/control_config.h` 新增 `SUPERVISOR_FREQ 2000U` 与
  `SUPERVISOR_TICKS_PER_MS`，作为 ms 级超时与分频的唯一换算依据。
- 命名：`BSP1kHzIRQHandler`→`BSP2kHzIRQHandler`，`FOC1kHzSupervisor`→`FOC2kHzSupervisor`。
- 分频：LED 5 Hz、RGB 20 Hz、CAN 波特率维护 10 Hz（100 ms）节拍不变，按 `SUPERVISOR_FREQ`
  整数派生；bus-off 恢复仍为 100 ms 调用 + 10 拍分频（约 1 Hz）。
- 心跳：`can_hb_set`（ms）经 `SUPERVISOR_TICKS_PER_MS` 换算，超时逐毫秒一致，计数粒度 0.5 ms。
- 温度：2 kHz 更新；IIR 系数 0.02→0.01（保持约 50 ms 时间常数）；失效门限 200 tick = 100 ms；
  遥测字段 `missed_ms`→`missed_ticks`。

## 3. 不变行为与可观测变化

- 不变：电流环 20 kHz；LED/RGB/波特率维护/bus-off 恢复节拍；心跳与温度超时的毫秒语义；
  过温判定使用未滤波温度；故障锁存与恢复规则；协议与参数 ABI。
- 变化：温度采样与过温跳闸检测由 1 kHz 提至 2 kHz（最坏检测延迟 1 ms→0.5 ms）；
  监督中断次数翻倍；`McuTemperature.missed_ms` 更名（无协议暴露）。

## 4. 阶段 2/3 与状态机上移（已实施）与后续

- **阶段 2（完成）**：编码器慢估计（多圈/机械角/速度）移入 2 kHz tick。快路径只做帧读取、
  方向/LUT 校正与电角度；`Encoder_UpdateSlowEstimate()` 是慢状态的唯一写者，快路径通过
  `rebase_requested`/`zero_requested`/`velocity_restart_requested` 请求变更；RTT 不再与慢估计叠加。
- **阶段 3（完成）**：位置级联/轨迹与速度 PI 从 PendSV 延迟作业邮箱改为 TIM7 2 kHz 慢拍直接执行
  （`MotorOuterLoop_SlowTick`）；删除作业队列、纪元、截止期故障与外环内存屏障平台接口。
  仍留在快速环内分频执行的项：位置阻抗控制、无感启动、各标定会话（后续可另行迁移）。
- **状态机上移（完成）**：主运行状态机（`FocRunState_Tick`）移入 2 kHz 慢拍。快环只上报
  worker 结果（`FocRunState_PostOutcome`，短临界区邮箱，只保留最新非 RUNNING 结果）并执行
  “故障 → 立即关断”快车道（`FocRunState_FastFaultStop`）：本板 TIM1 BKIN 关闭且无驱动器
  故障引脚，软件是唯一关断路径，故该安全动作保留在 20 kHz 时延内；状态机在慢拍对齐功率级
  记录并执行停机清理。
- **阶段 4（完成，方案 A）**：S5 CAN 队列派发落在 2 kHz tick；全部命令统一入队，
  中断只做取帧/解码/入队；写路径短临界区、应答优先于状态流；实机回归待台架。
  实现与证据见 [S5 计划](2026-09-19-can-isr-slimming.md) §10。

## 5. 验收

- 离线：`verify --profile pr` 全绿；温度原生夹具按 2 kHz 节拍（200 tick）更新并通过。
- 实机（2026-09-20，node 1 轮式台架，方案 A 全量执行）：
  - 烧录并运行本工作树构建的固件（0 Error / 0 Warning；Code 86436 / RO 4964 / RW 396 /
    ZI 31060，较改造前 Code 86920 / ZI 31344 更小）；
  - 只读预检通过（vbus 27.86 V、温度 52.0 °C、限值查询正常）；
  - 速度闭环 +10.005 / −9.989 rad/s（iq +0.091 / −0.104 A）；
  - 位置闭环去程误差 0.0110 rad、回程误差 0.0230 rad；
  - 主机停发帧（心跳租约 500 ms）后 0.404 s 内退出运动模式并关断；链路恢复后命令链可用。
- 延后：release 档（提交后执行）；温度超时/跳闸注入与 bus-off/波特率切换未重复执行
  （本次重构未改变其实现与节拍节流，2026-09-19 已台架验证）。

## 6. 回滚

单提交 revert：`.ioc` 与 TIM7 覆写、`SUPERVISOR_*` 契约、命名与测试断言同步还原；
无持久化数据或协议痕迹。

## 7. 证据

- 阶段 1（时基与内容重标定）：`outputs/runs/20260919T170225709451Z-90982812/summary.json`。
- 阶段 2（编码器慢估计迁移）：`outputs/runs/20260919T171626371502Z-90982812/summary.json`；
  夹具更新：encoder 启动/测速夹具按慢拍驱动、齿槽标定改读 `Encoder_GetMecPos()`、
  编码器观测器标定改为重定多圈基准请求。
- 阶段 3（外环迁入 2 kHz 慢拍）：`outputs/runs/20260919T172338527437Z-90982812/summary.json`；
  `test_outer_loop_runtime` 重写为慢拍直驱（发布所有权、模式/复位、无效输入、故障保持、
  PI/斜坡等价、堆容量）；板级启动编排夹具同步去除外环初始化步骤。
- 状态机上移：`outputs/runs/20260919T173126321554Z-90982812/summary.json`；`test_run_state`
  差分扩为 46,464 组比对全一致（含结果信箱投递/消费），新增紧急关断快车道路径。
- S5 阶段 4：`outputs/runs/20260919T175256075981Z-90982812/summary.json`。
- 全部为 PR 档（离线）；release 档需先提交。
- 实机：`outputs/bench_20260920/verify_refactor_result.json`（含固件 hex SHA256 与闪存前
  参数区备份 `params_before_flash.bin`、烧录脚本 `flash_app.jlink`、验证脚本）。
