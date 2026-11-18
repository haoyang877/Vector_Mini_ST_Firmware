# APP 与 Loader 状态机设计

本页定义当前共用APP和Loader的状态、守卫与交接。两板每根轴各持有一个 `MotorStateMachine`。上电进入 IDLE。第五个枚举 SELF_TEST 保留线路映射，生产代码没有迁入该状态的事件。上电电流零偏在关相 IDLE 内采样。

## APP 电机状态与功率许可

上位机位置会话策略：仅Type117显式不限位标志通过普通启动门禁后，
由本轴状态机发布`MotorControlState.host_position_unlimited`；外环快照只读此权限。
`host_feedback_ms`由来源正确的Type108在前台续期，监督拍按无符号毫秒差检测500ms失联并停机。
STOP、故障结束和初始化清除权限；普通RequestStart保持原行程策略。
位置目标、外环实测与OTG参考使用同一权限；不改轴记录、持久参数或其他动态保护。

```mermaid
stateDiagram-v2
    [*] --> IDLE
    IDLE --> RUNNING: 启动请求，外环发布许可，ADC 确认启相
    IDLE --> CALIBRATION: 标定受理
    RUNNING --> IDLE: STOP 且关相确认
    CALIBRATION --> IDLE: STOP 或保存成功后完成
    IDLE --> FAULT: 故障
    RUNNING --> FAULT: 故障
    CALIBRATION --> FAULT: 故障
    FAULT --> IDLE: 显式清错且恢复条件满足
```

### 详细状态与执行时序图

上面的 Mermaid 用于总览；详细设计使用下列 PlantUML 图源，按 2026-11-17 当前源码核对。转换标签统一写为
`事件 [条件] / 动作`，方括号表示允许提交该转换的条件。图中描述的是当前实现，
不新增主状态、回调或状态机框架；同名字段和调用可直接对照源码。

| 图源 | 阅读重点 |
| --- | --- |
| [电机详细状态图](diagrams/motor_state_machine.puml) | 五个枚举、有效转换、提交条件与故障收尾；SELF_TEST 保留但未接线 |
| [启动时序图](diagrams/motor_start_sequence.puml) | 前台受理、外环发布许可、ADC 复核及 PWM 启用读回；ACCEPTED 时仍可能为 IDLE |
| [STOP 时序图](diagrams/motor_stop_sequence.puml) | 撤销许可、快速关相、监督或外环收尾，以及拒绝迟到的旧会话结果 |
| [硬件检测关联状态图](diagrams/hardware_check_state_machine.puml) | Type127 的七个检测阶段及其与 IDLE、CALIBRATION、RUNNING、FAULT 的关联 |

`start_pending`、`phase_enable_epoch` 和 `calibration_handoff` 都是运行字段，
不能从图中推导为额外主状态。时序图给出合法的事件先后与分支，
不承诺 ADC、监督、外环和前台之间不存在抢占。看门狗判据另见
[运行期健康与看门狗](#运行期健康与看门狗)，不把 FAULT 直接画成复位。

使用本地 Java 与 PlantUML，先检查语法，再导出到忽略的 `outputs`。以下命令从仓库根目录执行，
`path/to/plantuml.jar` 替换为本机工具位置；不需要向在线渲染服务发送图源。

```text
java -jar path/to/plantuml.jar -charset UTF-8 -checkonly docs/architecture/diagrams/*.puml
java -jar path/to/plantuml.jar -charset UTF-8 -tsvg -o ../../../outputs/state-machine-diagrams docs/architecture/diagrams/*.puml
```

| 主线 | 谁提出 | 何时改变主状态 |
| --- | --- | --- |
| 空闲 → 运行 | 前台受理启动，登记模式、起始目标和会话 | 外环发布启相意图，ADC 拍首复核本会话、采样、模式、STOP 和故障。只有 PWM 启相且读回确认后，才提交 `RUN` |
| 运行或标定 → 空闲 | 前台 STOP 或快拍任务请求停机 | 请求立即撤销许可。ADC 遇到待停请求先关相；监督拍或外环提交入口消费 STOP、清目标、撤销会话、确认关相，再提交 `STOP`。关相失败时进入故障 |
| 空闲 → 标定 | 前台受理指定标定类型 | 从关相空闲且输入有效的轴发起，受理时提交 `CALIBRATE`。需要通电的标定由 ADC 再复核并启相；机械零位保持关相 |
| 标定 → 空闲 | 快拍报告算法完成 | 先关相。需要安装 LUT 时由 ADC 在读角前复核会话；前台复核并保存，成功后由监督拍提交 `COMPLETE`。联合标定在同一会话转入电零位 |
| 任意状态 → 故障 | 保护逻辑锁存故障，或启相关相确认失败 | 快环看到故障立即抑制功率，慢拍处理故障并提交 `FAULT`。故障锁存和主状态写入发生在各自的执行上下文 |
| 故障 → 空闲 | 前台显式 CLEAR | 必须关相、输入有效、故障源满足恢复条件。若原子清故障字失败则恢复 `FAULT`；成功只回空闲，启动需重新请求 |

## 硬件检测阶段与电机主状态

AT32F456 APP 的硬件检测由独立任务维护 `s_report.stage`，通过 Type127 显式启动和查询。
它与每轴 `MotorStateMachine.state` 同时存在；电机 `SELF_TEST` 枚举仍为占位，
不能据此判断硬件检测是否实现。当前 STM32G4 入口未注册该检测任务。
[板端检测实现](../../projects/at32f456/application/hardware_check.c) ·
[详细关联图](diagrams/hardware_check_state_machine.puml)

```mermaid
stateDiagram-v2
    [*] --> CHECK_IDLE
    CHECK_IDLE: IDLE 0：检测未开始
    STATIC: STATIC 1：关相静态，电机保持 IDLE/FAULT
    RESISTANCE: RESISTANCE 6：测阻，电机进入 CALIBRATION
    ARMED: ARMED 2：待驱动，电机通常为 IDLE
    DRIVING: DRIVING 3：已启相，电机为 RUNNING
    DONE: DONE 4：采集结束，按项目判定
    ABORTED: ABORTED 5：检测中止，关相另行确认
    CHECK_IDLE --> STATIC: Type127 受理新一轮
    STATIC --> DONE: mode1，静态完成且NTC取得判定
    STATIC --> RESISTANCE: mode3，动力检查和标定准入通过
    STATIC --> ARMED: mode2，动力检查通过
    RESISTANCE --> ARMED: 本轮测阻通过且关相收尾完成
    ARMED --> DRIVING: 本轮模式10启相采样确认
    DRIVING --> DONE: STOP收尾，双向和代际证据满足
    STATIC --> ABORTED: 当前阶段中止条件成立
    RESISTANCE --> ABORTED: 当前阶段中止条件成立
    ARMED --> ABORTED: 取消、超时或会话不符等
    DRIVING --> ABORTED: 保护、超时或会话不符等
    DONE --> STATIC: Type127 受理下一轮
    ABORTED --> STATIC: Type127 受理下一轮
```

| 检测阶段 | 电机状态及推进条件 |
| --- | --- |
| `IDLE=0` | 检测报告初值，和电机 `IDLE=0` 属于不同枚举；上电不自动开始检测。 |
| `STATIC=1` | Type127 要求轴已停机、无启停待办或维护冲突，接受已关相的 IDLE 或 FAULT。保持原主状态，逐项采集电流、母线、NTC、转子和 Hall；静态异常项不直接截断后续静态采集。 |
| `RESISTANCE=6` | mode3 在静态动力检查通过后请求 `PHASE_RESISTANCE` 标定；仍须从无故障 IDLE 通过正常准入，进入 CALIBRATION。测量完成、关相收尾及本轮阻值分类通过后转 ARMED。 |
| `ARMED=2` | 通常保持 IDLE，等待 Type117 模式10。受理必须匹配本轮启动序号及新运动代际，随后 ADC 复核启相；仅收到受理回复不代表已进入 DRIVING。 |
| `DRIVING=3` | 已采到本轮开环电流模式的启相证据，电机处于 RUNNING；Type116 给定正向、保持、反向，Type118 续租。STOP 收尾后的快采样还须满足本轮代际和双向采集证据才结束检测。 |
| `DONE=4` / `ABORTED=5` | 冻结实测值，停止采集并释放维护区；仍匹配本轮动力会话且未停机时请求 STOP。终态与电机收尾不是原子操作，需另外确认数字关相；电机最终可为 IDLE 或 FAULT。 |

`DONE` 只表示采集结束，单项结果仍可能异常；`ABORTED` 也不等同于所有已测项失败。
Qt 工具的“静态＋相电阻”在 ARMED 主动 STOP，板端可记为 ABORTED，
主机按静态和测阻范围的完整证据判断通过，动态项保持未测。
每轮结果绑定 `run_id`，不能拼接不同轮次。检测阶段、项目判据和阈值的线路契约见
[AT32F456 硬件功能检测](../protocols/README.md#at32f456-硬件功能检测-v2)，
主机范围判定见 [Qt 生产工具](qt_production_design_v1.md)。

ADC 中断通过 `HardwareCheck_CaptureFast` 有界入队，前台通过 `HardwareCheck_Process`
推进阶段和发布报告；`maintenance_capture` 在采集期间占用维护区，与参数事务和其他标定互斥，
本轮相电阻按已有特例借用标定执行器。任务绑定和调度见
[AT32F456 入口](../../projects/at32f456/application/main.c)。

## 从 IDLE 判断请求能否受理

普通 START 要求无故障、无待处理启停或启动锁、已有数字关相确认、配置有效且已确认、采样有效、母线在允许范围、编码器在线。闭环模式还要求相应编码器标定标志齐全；位置模式额外核对当前角度的行程资格。受理后模式与 `start_pending` 可先更新，ADC 实际启相时才写入 RUNNING。

标定准入按类型判断。LUT 和电零位使用功率许可，电零位要求已线性化；机械零位保持关相，执行时检查输出轴就绪。带 Hall 的 LUT 可在一个 `epoch` 内继续联合电零位。`epoch` 用于拒绝旧会话的外环或标定结果。逐项判定见 [MotorStateMachine_RequestStart / RequestCalibration](../../firmware/app/motor_state_machine.c)，线路值见[启停协议](../protocols/README.md#control)。

### 各类请求的附加条件

所有运动和标定请求都经过前台状态机。基础采样检查包含快拍计数在上一个监督间隔有进展、当次电流零偏有效、三相电流与母线读数有限，以及板型要求的 MCU 温度新鲜且有效。功率准入同时检查滤波母线位于使能窗口，原始母线低于硬过压阈值。

| 请求 | 对配置、反馈和任务的附加要求 | 受理后的第一步 |
| --- | --- | --- |
| 117 电流、速度、位置、Vq | 配置有效且确认，编码器在线，具备完整的转子标定；位置模式额外检查当前位置在许可行程内 | 清理上一会话的参考和控制器。位置模式把当前位置作为初始参考，其余模式从零目标开始。 |
| 117 开环电压或开环电流 | 检查轴配置、母线与编码器在线；开环路径允许转子标定尚未齐全 | 登记模式和待启动标志；后续 116 须同时满足电角频率和电压或电流矢量限幅。 |
| 160 编码器 LUT | 轴空闲、关相、编码器在线、任务空闲、配置和功率条件满足；有轴预设时，标定电流预算须覆盖对齐电流及启动 d/q 电流幅值 | 进入 CALIBRATION，登记本代际的启相许可。 |
| 160 电零位 | 有功率标定的通用条件，另外要求已有线性化标志 | ADC 复核后对齐，完成后先关相。 |
| 160 机械零位 | 满足基础采样、轴配置、编码器及关相条件，快拍执行时输出轴反馈必须就绪 | 保持关相，生成机械零位并交给前台保存。 |

请求遇已有故障可返回 FAULT，已有待启动或交接锁可返回 BUSY，状态、模式或资格不符可返回 DENIED。116 还区分非法字段与超范围值；被拒时保留原目标。完整次序见[准入、目标检查和状态迁移](../../firmware/app/motor_state_machine.c)。

## 启动与 STOP 相遇时的事件顺序

以速度模式为例，初始 `state=IDLE`、`control_mode=NONE`、三相关闭。以下每行表示一次可能被其他上下文打断的事件，调用者不能把多行合并成一个“已启动”判断。

| 时刻 | 写入者 | 已发生的动作 | 此时能得出的结论 |
| --- | --- | --- | --- |
| 117 通过准入 | 前台 | 代际增加为 E，登记 SPEED，置 `start_pending` | 回复可为 ACCEPTED。尚未发出启相许可，主状态仍是 IDLE。 |
| 第一次有效外环提交 | 外环 | 复核 owner、E、模式和故障；速度环就绪后写 `phase_enable_epoch=E` | ADC 下一拍可以消费这次许可。 |
| ADC 本拍复核成功 | ADC | 取出并清空一次性许可，核对 E、输入、锁与外环就绪，操作 PWM 并读回 | 确认输出已启用后才转 RUNNING，清除 `start_pending`。启相复核或读回失败可锁存控制时序故障。 |
| STOP 恰好在许可发出后、ADC 消费前到达 | 前台 → ADC | 前台置 `stop_pending`、清许可和待启动意图、撤销控制模式。ADC 先处理 STOP 和故障 | 本次启动被撤销。外环或监督收尾后旧 E 失效，旧计算结果不能再启相。 |
| STOP 在已经 RUNNING 后到达 | 前台 → ADC / 慢拍 | 前台根据当时功率读回回复；ADC 遇 STOP 时关闭仍活动的输出，慢拍清控制器、结束会话并核对关相 | 0/0 表示数字输出已关，1/1 表示受理待关。若同时锁存故障，最终主状态进入 FAULT。 |

定位“看见模式、看不见出力”时，应在同一轴观察 `state`、`start_pending`、`phase_enable_epoch`、`stop_pending`、`fault_code` 和输出读回。三者的作用不同：模式选择控制算法，代际许可授权一次硬件操作，主状态记录已提交的结果。`epoch` 达到 UINT32_MAX 后若还需推进，代码锁存故障，避免计数回绕后误认旧工作。

## 标定阶段与故障恢复

| 交接标志 | 设置条件 | 谁消费，什么情况下拒绝 |
| --- | --- | --- |
| `MOTOR_CALIBRATION_NO_HANDOFF` | 标定正在执行，或联合标定开始下一阶段 | ADC 只在任务种类、功率状态、STOP、故障和启动锁均允许时执行一个快拍。 |
| `MOTOR_CALIBRATION_AWAITING_PUBLICATION` | 快拍任务已完成或 LUT 待发布，关相读回成功 | 前台准备候选。LUT 安装前由 ADC 检查原代际；最终保存前还核对原类型，防止同一 E 内的下一阶段误用上阶段候选。 |
| `MOTOR_CALIBRATION_AWAITING_COMPLETION` | 当前会话保存成功 | 监督拍消费 COMPLETE。若 STOP 或故障抢先发生，按停止或故障路径收尾。 |

带 Hall 的 LUT 有一段停稳等待。前台观察连续机械速度绝对值低于 1 rad/s 达 200 ms 后请求安装 LUT；在尚未满足停稳条件且等待达 6000 ms 时报告失败。ADC 安装成功后沿同一 E 转入联合电零位，待最终关相才保存整轴记录。Flash 写失败会报告存储故障。完整扫圈后记录 Hall 标定不合格原因、中途取消旧候选、存储失败是三条不同的退出路径。[标定发布实现](../../firmware/app/motor_calibration.c)

CLEAR 由前台执行。先核对实际关相、输入有效、标定会话空闲、配置合法及故障源恢复，再用原子比较交换清除已检查的故障字；期间若故障字已改变则保留 FAULT。不同故障有不同恢复条件：过流需要监督拍累计恢复时间，电压故障要求滤波母线回到使能窗口，编码器故障要求新鲜采样和编码器在线。关相读回失败导致的 `fault_locked` 阻止普通 CLEAR。[恢复判据](../../firmware/motor/protection/motor_fault.c)

## 标定执行与结果发布

两板 APP 的 160 请求支持编码器线性化、电气零位和机械零位。协议适配层把 kind 1、2、6 映射为内部标定类型；本轴状态机受理并建立会话后进入 CALIBRATION。受理回复中的 ticket 记录请求，持久化走以下路径。[标定命令](../protocols/README.md#parameters) · [命令映射](../../firmware/app/app_protocol.c)

| 上下文 | 所属模块 | 转交条件 |
| --- | --- | --- |
| ADC 快拍 | `MotorAxis_ControlFast → MotorCalibration_StepFast` | LUT 和电零位在许可启相后执行，机械零位保持关相。任务报完成后，状态机关闭输出并读回，设置待发布阶段。 |
| 前台与 ADC | `MotorCalibration_ProcessPublication / PublishLutFast` | 对 LUT，前台登记待安装的会话，带 Hall 的轴先等转子停稳；ADC 在本拍编码器读回前复核代际并安装。联合标定在原代际继续电零位。 |
| 停机前台 | `MotorCalibration_ProcessPublication` | 准备轴记录，在最终提交前检查原代际、标定类型、STOP、故障和实际关相，再写 Flash 并回读。写入失败锁存存储故障。 |
| 监督拍 | `MotorStateMachine_SupervisorTick` | 保存成功后消费完成事件并回 IDLE。中途停机或故障会使旧候选失效。 |

完整扫圈后的 Hall 候选不合格时，保留失败原因并保存诊断记录。中途取消走 STOP，不提交部分标定。在线参数 SAVE 与自动标定保存是两个独立入口，两者的协议准入与执行结果分别核对，不以标定自动保存代替 SAVE 命令的结果。[发布实现](../../firmware/app/motor_calibration.c) · [操作与判定](../guides/calibration_process.md) · [记录布局](../../firmware/parameters/README.md#storage)

## 运行期健康与看门狗

适用于两板配套 LoaderApp。电机运行许可、运行期看门狗健康和新镜像确认资格分别判断。
`MotorStateMachine_ReadRuntimeHealth` 在前台短临界区读取本轴的运行期健康与
`MotorRuntimeProgress`，其中保存快环、外环、监督任务各自的完成计数。
计数由各执行者维护，快照不推进任务或改变状态，也不承诺三个任务在同一拍完成。
`AppLoader_Process` 每 100 ms 检查三项计数相对上次均有变化，并检查通信模块就绪；
前台停止调用或任一任务不再推进，均不能继续喂狗。计数使用无符号自然回绕，
不把主机是否持续在线作为程序活性条件。

| 轴的当前情况 | 运行期健康要求 |
| --- | --- |
| IDLE、SELF_TEST、FAULT，或任意已知状态已锁存电机故障 | `fault_locked` 为 false，且数字读回确认三相输出关闭；零偏未完成或测量无效不单独否决 |
| 无故障的 RUNNING、CALIBRATION | `fault_locked` 为 false，且基础采样有效，继续采用现行电流零偏、有限值与新鲜度要求 |
| 非法状态，或关相确认失败、会话号耗尽等已置 `fault_locked` | 不健康 |

已确认镜像满足上述运行期健康、三项任务进展和通信就绪时继续维护看门狗。
因此上电早期电压故障即使使零偏采样停住，只要功率可靠关闭且程序继续工作，
APP 仍可提供故障查询和诊断。这里不修改零偏采样调度、故障锁存、CLEAR 或再次启动的条件；
保持通信不代表故障已经恢复。

新镜像确认使用 `MotorStateMachine_ReadBootHealth` 的严格条件：基础采样完整有效、
无电机故障且不在 FAULT、没有 `fault_locked`。AppLoader 只有在严格条件、运行期健康、
任务进展和通信就绪均成立时，才累计连续 11 个健康检查间隔；任一次失败即清零。
57 确认请求还须复核当前健康、实际关相和无已登记的 56 请求，BootHw 再核对版本、
CRC、激活/尝试标记及 5 s 确认期限。未确认镜像的喂狗仍要求严格健康，
超出确认期限后停止喂狗，实际复位时刻取决于硬件看门狗。

实际判断见[状态机健康接口](../../firmware/app/motor_state_machine.c)与
[APP 健康检查和确认](../../firmware/app/app_loader.c)。

## 从代码跟一条路径

- 从 [motor_state_machine.c](../../firmware/app/motor_state_machine.c) 的 `transition_motor_state` 开始读。它按 RUN、CALIBRATE、COMPLETE、STOP、FAULT、CLEAR 处理实际事件。初始化之后的主状态写入全部在这里。`MotorStateMachine_NextState` 给出五态之间允许的转换。
- 启动请求登记后，外环批准本次启相，ADC 在 `BeginFastTick` 处理 RUN。STOP、标定保存成功后的 COMPLETE、首次 FAULT 共用关相和会话收尾。CLEAR 在关相和故障源恢复后才回空闲，若同时发生新故障则仍锁存 FAULT。
- 每次 ADC 采样后，[MotorAxis_ControlFast](../../firmware/app/motor_axis.c) 先调用 `BeginFastTick` 检查故障和启相许可，再读取主状态。运行态执行 `MotorControl_Step`，可执行的标定态执行 `MotorCalibration_StepFast`。最后 `FinishFastTick` 接收本拍结果并检查新故障。
- 前台在 [motor_state_machine.c](../../firmware/app/motor_state_machine.c) 校验目标并直接写本轴，也从本轴读取 CAN 反馈。外环在 `MotorStateMachine_CommitOuterResult` 直接写有效结果。ADC 单独复核启相条件。
- 标定的停稳、LUT 安装和 Flash 保存见 [motor_calibration.c](../../firmware/app/motor_calibration.c)，线路状态和命令编号见[协议说明](../protocols/README.md)。除 Flash 事务外，这些交接不暂缓 ADC。

## Loader 下载与启动状态

APP 的 `epoch` 标记一次电机运行或标定会话。复位后由 Loader 创建下载会话，按来源节点、主机 nonce 和 Loader 分配的 session 核对写入权限。下载阶段保存在 RAM，激活、试运行尝试和确认三个标记保存在镜像的 Flash 描述符中。

| 当前阶段或条件 | 事件与守卫 | 动作与可观察结果 |
| --- | --- | --- |
| IDLE | 50 BEGIN 的清单与目标匹配，输出关闭且链路就绪 | 建立绑定，经过 READY 进入 ERASING；主循环按页擦除 APP 槽，完成后进入 RECEIVING。 |
| RECEIVING | 51 WRITE 的会话、偏移、长度和对齐有效 | 写入并回读，推进下一偏移。若重传已写区间，比较 Flash 内容；数据冲突返回错误。 |
| RECEIVING → VERIFYING | 52 FINISH，镜像已收齐 | 分段检查完整性和描述符。通过后进入 VERIFIED，镜像校验失败进入 ERROR。 |
| VERIFIED | 54 ACTIVATE，由匹配票据报告真实发送成功 | 重新核对镜像和提交条件，写激活标记并复位。事务进行中，55 ABORT、租约过期或链路丢失可撤销写权限。 |
| 复位后启动检查 | 救援窗口结束，允许启动且镜像有效 | 已确认镜像可跳转；首次试运行先写尝试标记；已尝试且未确认的镜像留在 Loader。 |
| LoaderApp 试运行 | 57 的版本、CRC、健康窗口及关相条件满足 | APP 写确认标记。通过同 UID 设备的 GET_INFO 回读角色和确认状态。 |

56 的 READY 表示原 APP 可以交接；54 的发送完成使 Loader 可以提交激活；57 成功才确认新 APP。重复 54 和截止点附近的已知实现行为见[Loader 交接时序及边界](loader/loader_product_v3.md)。状态迁移可用[双板电机用例](../../tests/unit/native/test_run_state.py)与[Loader 事务用例](../../tests/unit/native/test_loader_upgrade.py)分别验证。

### 复位后如何决定跳转

Loader 先确认复位原因、功率关闭及链路条件。看门狗复位、带有效 ENTER 邮箱的软件复位，或启动窗口内的有效驻留请求，均可使设备留在 Loader。常规启动等待 50 ms 后检查镜像完整性、向量和激活标记，再检查下表的两个持久标记。

| 尝试标记 | 确认标记 | 镜像已激活且通过检查时的决定 |
| --- | --- | --- |
| 擦除态 | 擦除态 | 写入并读回尝试标记，成功后跳转一次试运行。 |
| 有效 | 擦除态 | 上次试运行已消耗机会，留在 Loader，等待重新下载。 |
| 有效 | 有效 | 满足其他启动条件时可启动已确认 APP。 |
| 其他组合、半写或读取失败 | 任意 | 拒绝跳转。标记检查同时核对 magic 和反码。 |

57 由试运行 APP 处理，其严格健康窗口和期限见[运行期健康与看门狗](#运行期健康与看门狗)。镜像启动判断由[镜像内核](../../shared/image/image_transfer.c)负责；普通故障下继续维护已确认镜像的看门狗，不改变 Loader 的看门狗复位驻留策略。

### 设计变更时的验证轨迹

| 起点与注入事件 | 应检查的结果 |
| --- | --- |
| 117 已受理，启相前收到 STOP | 不产生该会话的后续使能；允许暂时看到模式与主状态处于不同阶段。 |
| 标定拟合已结束，在最终提交前撤销原会话 | 候选丢弃，旧会话不能写 Flash 或上报本次成功。 |
| 下载已 VERIFIED，一个其他票据报告发送成功 | 保持未激活；只有当前事务的物理完成事件才能进入标记写入路径。 |
| 首次启动已写尝试标记，确认前复位 | 不重复试运行。只有重新下载或满足已确认镜像的启动条件才能启动。 |

电机事件与抢占见[状态集成用例](../../tests/unit/native/motor_state_machine_integration_test.c)，掉电与标记组合见[Loader 内存用例](../../tests/unit/loader_upgrade_test.c)。

## 相电阻检查

完整硬件检测新增内部标定类型 PHASE_RESISTANCE。与其他有功率标定共用准入、ADC启相、STOP和故障关断；仅本检查允许BSP在零电流时选择AB、BC、CA双相，第三相上下桥均关闭。快环工作区归MotorCalibration；完成直接关相退出，不发布LUT、不写Flash。检查结果由板级检测在终态复制，失败不得进入动态。

### 自由转子测阻

相电阻允许转子随直流磁场对齐，不按相对起点位移中止。转子及电流稳定后采样；每次极性激励最多4s，板级总看护25s。专用7A过流及原有故障/STOP仍直接关相，不改变普通运行和Flash参数。

当前测阻档位为1A/2A，需活动电流上限至少2A。首次对齐2A/s、同向升档5A/s、撤流10A/s，仍在转子及电流稳定后采样，7A中止及原保护不变。试验线电压上限12.5V且不超过实测母线45%，不改变运行控制及存储参数。

## 故障与恢复边界

motor_sensing产生采样资格，motor_fault锁存故障并判断恢复条件；状态机掌握功率资格及实际关相。
快环故障不等待前台或低频监督才关闭输出。故障源消失不自动清锁存、不恢复旧目标。
MotorFault_Report保留首个故障，协议位图由适配层构造，不直接发送内部故障枚举。
FAULT中的CLEAR要求实际关相、有效配置和采样、标定空闲、故障源恢复且无锁定条件；
成功仅返回IDLE。过流恢复需驻留时间，电压须回到恢复窗口，MCU温度须低于恢复门限。
关相失败和代际耗尽等锁定故障不允许普通CLEAR绕过。

母线的使能、过压/欠压和恢复阈值来自活动配置，原始值与滤波值用于各自明确的判据。
STM32 MCU温度来自片内TS/VREF，AT32电机NTC是独立来源，不能互相替代。
绕组热模型接口及参数保留，当前APP不调度；有效温度、模型估计和电流限额不是同一事实。
已确认LoaderApp在普通故障但可靠关相、关键任务仍推进时可以继续维护看门狗；
该健康策略不放宽CLEAR、重新启动或试运行镜像确认条件。
条件的唯一实现入口为[motor_fault.c](../../firmware/motor/protection/motor_fault.c)及本页状态机。

### Hall 极值超限时的诊断拟合

只有 Hall 极值超限且样本齐全、幅值合格时，使用本轮系数额外尝试诊断拟合。
同拍双路在限幅区保持原码且参考角已移动时拒绝拟合，随后仍需检查整圈覆盖、角映射单调和相位余量。
拟合成功时保存[诊断表](../protocols/output_encoder_v3.md)供只读回读，发布到运行态前移除该表，保留原系数和超限故障。
