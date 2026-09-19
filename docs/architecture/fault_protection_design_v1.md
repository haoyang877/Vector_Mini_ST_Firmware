# 故障保护模块、接口与参数设计

版本：v1.0 设计稿；日期：2026-09-18。

范围：当前 STM32G4 APP 的检测、关断、故障状态、恢复、遥测与参数生命周期。
本文是后续实现契约，不表示这些接口已经存在，也不宣称硬件保护或功能安全验证完成。
接口采用 C 语义示意；类型大小、错误码和线上编号须在各阶段实现时通过测试冻结。
不修改当前参数 ABI、CAN 报文或运行行为。

关联：[架构边界](../../ARCHITECTURE.md)、[代码结构](code_structure.md)、
[实施计划](../plans/active/2026-09-18-fault-protection-v1.md)。

## 1. 当前基线与复用原则

| 已有实现 | 本设计的处理 |
| --- | --- |
| `motor/protection/bus_voltage_profile.h`：8S 母线配置 | 保留为板卡/供电 profile 输入；不再把旧 10/30V 配置当作当前基线 |
| `motor/foc/foc_sensing.c`：电流、电压、MCU 温度保护 | 分离采样转换与条件检测；逐项迁移，不一次重写采样调度 |
| 默认 2mΩ、40A、连续 5 次软件过流 | 记录为当前代码值；实际构建宏覆盖及实物匹配另行确认 |
| MCU 90℃跳闸、采样超时100ms | 明确温度来源为 MCU die，不能作为绕组或 MOS 测温 |
| `ControlOverrun_Error`、`CoggingCalibration_Error`、`TemperatureSensor_Error` | 保留旧故障编号，增加独立子原因和内部类型 |
| 异步外环任务与配置缓存 | 复用 epoch 失效机制，故障、配置切换和重新使能均不能消费旧结果 |
| `services/telemetry/motor_status.*`、CAN 48B 状态流 | 保持旧帧布局，增加版本化诊断接口；快照和事件分别传输 |
| `PARAM_SCHEMA_VERSION=11`，轴配置及齿槽图记录 | 不直接扩张或重新解释旧结构；新记录需独立版本、迁移和容量评估 |
| 单个 ErrorNow、直接清除、CAN 接收自动解除断联 | 作为迁移目标；全局搜索清除旁路并逐一替换 |

当前源码入口：
[采样](../../firmware/motor/foc/foc_sensing.c)、
[故障处理](../../firmware/motor/protection/foc_errhandle.c)、
[快速环](../../firmware/app/foc_task.c)、
[平台接口](../../firmware/platform/api/motor_hw.h)、
[状态快照](../../firmware/services/telemetry/motor_status.h)、
[参数](../../firmware/services/parameters/foc_param.h)。

## 2. 模块边界与依赖

```text
平台采样/硬件事件 -> app 组装 SI 单位输入 -> 独立检测器
                                              |
                                      DetectorObservation
                                              |
命令/清除请求 -> 服务邮箱 -> app -> FaultCore -> ActionRequest
                                              |          |
                                   Snapshot/Event    app动作执行器
                                              |          |
                                        遥测/协议     platform API

紧急硬件路径：比较器/驱动故障 -> 硬件禁止输出
CPU异常路径：异常入口 -> 最小平台关断；故障核心不可用时也必须执行
```

| 建议位置/模块 | 负责 | 禁止依赖或承担 |
| --- | --- | --- |
| `motor/protection/protection_types.h` | 纯类型、单位、证据枚举、位图操作 | HAL、main.h、CAN、Flash、全局 MotorControl |
| `motor/protection/detectors/` | 单项算法、计时、恢复证据 | 改模式、操作PWM、清总故障、发送报文 |
| `motor/protection/fault_core.*` | 状态归并、锁存、首发、策略裁决、条件清除 | 读寄存器、采样、运动轨迹计算、存储和协议 |
| `motor/protection/protection_policy.*` | 校验后的故障策略与模式能力表 | 运行中接受未经校验的动态阈值 |
| `app/protection_runtime.*` | 调度、输入拼装、邮箱、模式/epoch协调、动作执行 | 把检测公式或报文编解码重新复制一份 |
| `platform/api/power_stage_hw.h` | 关断、使能、输出状态、硬件事件接口 | 电机故障编号、运动策略、APP状态类型 |
| `platform/stm32g4/ports/power_stage_hw.*` | 实际寄存器与驱动引脚、复位默认状态 | 依赖app头或在这里判跟随误差 |
| `services/protection/` | 清除/诊断/恢复请求及完成结果 | 反向调用app；直接改FaultCore |
| `services/parameters/` | 参数分类、验证、发布、存储迁移 | 在检测器中查Flash，在线绕过硬上限 |
| `services/telemetry/` | 有界快照/事件传输、后台记录 | 控制关断时序或阻塞生产者 |
| `communication/protocol/`、`communication/can/` | 版本化编解码、请求校验、传输调度 | 决定物理故障是否解除 |

所有路径均相对 `firmware/`。新增文件是建议，不是本次创建的代码。
`motor` 不得 include `services`；共享参数输入类型归 motor 纯头，参数服务构造它们。
`services` 不得 include app；通过服务自有邮箱，由 app 在周期边界取请求并回填完成结果。
不复制 `MotorControl_TypeDef` 为新的万能结构，不扩大历史架构债务。

## 3. 数据契约

### 3.1 输入与观测

每个检测器只接收自己需要的窄输入，例如 `CurrentInput`、`BusInput`、`EncoderInput`。
公共样本元数据包含 `sample_seq`、`sample_time_us`、`valid_bits`、`config_revision`。
数值单位统一为 A、V、℃、rad、rad/s、s 或明确后缀的 us；注明峰值/RMS、电角/机械角和电机/输出轴坐标。
q轴电流不能无标定直接称为 N·m。电流重构标志、原始/滤波值、实测/估算来源分别表示。

```c
typedef enum {
    EVIDENCE_NOT_APPLICABLE, /* 工况不适用，不等价于恢复 */
    EVIDENCE_UNKNOWN,        /* 缺失、陈旧或不可信 */
    EVIDENCE_HEALTHY,
    EVIDENCE_WARNING,
    EVIDENCE_TRIP
} EvidenceState;

typedef struct {
    uint16_t fault_id;       /* 内部稳定目录ID，不是旧ErrorNow枚举 */
    uint16_t detail_code;
    EvidenceState state;
    uint32_t sample_time_us;
    uint32_t sample_seq;
    uint32_t config_revision;
    uint32_t evidence_flags; /* 已确认/恢复已验证/需要维护等 */
    float value;            /* 单位由该fault/detail schema定义 */
    float threshold;
} DetectorObservation;
```

一个检测器可以输出固定上限的小型观测数组，例如母线过压、欠压、测量失效分别输出。
故障目录保存单位和数据含义；不将 C 结构直接 memcpy 到报文或 Flash。
未知值会触发相应“监测能力丢失”策略，不能当健康值清除故障。
已禁止输出后电流为零，只能证明当前电流小；短路恢复仍需独立恢复证据。

### 3.2 故障状态与策略

每个 fault 的运行状态至少有：`last_evidence`、`active`、`latched`、`first_seen`、
`last_seen`、`occurrence_count`、`recovery_verified`、`last_detail`。
全局发布：`active_faults`、`latched_faults`、`warnings`、`unknown_monitors`、
`first_fault`、`primary_fault`、`action_state`、`run_epoch`、`event_seq`。
位图用固定数量 `uint32_t` word，不假定64位读写在M4上原子；目录超容量为编译/配置错误。
检测到unknown后保持已有锁存，失效监测单独阻止恢复。

策略表按 fault/subreason 和明确的工况选择：

| 字段 | 含义 |
| --- | --- |
| `severity` | L1告警、L2运行故障、L3严重故障、L4确认损坏/完整性失败 |
| `response` | 告警、降额、受控停止、禁止驱动、禁止启动、维修锁定 |
| `recovery_class` | R0自动撤告警、R1明确清除、R2服务验证、R3维修验证 |
| `required_capabilities` | 电流反馈、编码器、抱闸反馈、制动单元等 |
| `applicability` | 有功率/依赖编码器/运动中/标定阶段等；不散落硬编码模式列表 |
| `stop_profile` | 停车包络与备用动作的配置引用 |
| `recovery_evidence` | 健康持续时间、自检、回零、标定或维护记录要求 |
| `event_priority` | 事件队列优先级，与CAN标识符编码分开 |

首发故障记录“首个被核心接收的故障”，同时保留原始检测时间及来源；不能声称总是物理根因。
并发动作不是简单 max(severity)：禁止驱动优先否决任何转矩请求；抱闸和再生管理可同时需要。
未经验证的停车/机械保护能力不可通过软件配置假装存在。

## 4. 核心接口与执行规则

下列名称为设计契约。除明确的异步服务/平台入口外，同一ctx均由单一上下文调用。

| 接口示意 | 输入/输出及实现逻辑 |
| --- | --- |
| `Qualifier_Step(ctx, raw_state, now, cfg)` | 通用连续确认、窗口统计或恢复回差；每种模式显式选择，计数饱和；unknown/不适用不能累计健康恢复时间 |
| `XDetector_Init(ctx, cfg)` | 校验后的不可变cfg；初始化计时和历史，不分配内存 |
| `XDetector_Step(ctx, input, now, out)` | 有界计算，输出观测及恢复证据，不调用FaultCore或平台 |
| `XDetector_Reset(ctx, reason)` | 区分上电、配置改变、工况切换；热积累与维修证据不能随普通清除归零 |
| `FaultCore_Apply(ctx, observations, count, now)` | 故障进入/恢复状态机、锁存、事件产生；拒绝配置版本不匹配和过期观测 |
| `FaultCore_Evaluate(ctx, capabilities, action_out)` | 纯策略裁决；给出必要动作、转矩许可、限制及失效原因 |
| `FaultCore_TryClear(ctx, request, recovery, result)` | owner边界逐项检查mask、请求代号、恢复证据、输出状态和权限；返回已清、剩余和拒绝原因 |
| `FaultCore_GetSnapshot(ctx, out)` | 只由owner拷贝完整快照，再通过邮箱发布；不允许后台直接遍历活动ctx |
| `ProtectionService_RequestClear(request)` | 将有界请求放入邮箱，返回accepted/busy/invalid；accepted不表示已清除 |
| `ProtectionService_GetCompletion(request_id, out)` | 读取owner发布的结果；包含当前epoch和剩余故障 |
| `ProtectionService_RequestRecovery(request)` | 申请有限恢复任务；明确方向、电流、速度、时间和行程范围；不能普通使能越过故障 |
| `PowerStage_ForceOff(reason_bits)` | 平台同步、幂等、有界关断；硬件/异常入口可调用；不依赖FaultCore、系统tick、日志或HAL超时等待 |
| `PowerStage_TryEnable(generation)` | 只允许owner调用；核对当前许可、平台禁止锁存、硬件输入及代号；失败保持禁止 |
| `PowerStage_ReadStatus(out)` | 返回寄存器/引脚可观测状态与valid位；软件“已调用Stop”不等于物理已关断 |
| `PowerStage_TryRearm(expected_generation)` | 仅在独立清除与恢复检查后尝试解除软件禁止锁存；条件不满足返回失败；此操作本身不使能 |
| `StopController_Begin/Tick/Cancel(ctx, ...)` | app组装现有轨迹与控制器；持续检查反馈可信度、停车时间和包络；结束或故障使旧控制结果失效 |
| `FaultEvents_TryTake(out)` | 后台非阻塞消费；不影响关断。TX失败保留/重试由事件传输策略负责 |

`PowerStage_*` 必须定义强制关断优先的线性化语义：关断与使能并发时，结束状态必须为禁止。
generation防止陈旧使能请求，但单独比较代号不能消除“比较后被关断、随后又使能”的竞态。
平台实现需使用经验证的短临界区/硬件锁存与Break联锁；明确可屏蔽中断、NMI/异常、硬件信号的优先关系。
HardFault发生后不返回正常使能路径；新启动保持禁止。禁止仅靠普通bool读写保证互锁。

## 5. 每项检测功能的窄接口与逻辑

约定：D=设计参数，B=板级参数，M=电机/机构参数，C=标定参数，R=运行状态。
时间均按物理时间配置；固定频率检测可预计算周期数，频率变更须重新校验。
未给出的阈值标记TBD；依赖TBD的保护能力不得宣称已实现或批准进入相应工况。

### 5.1 电气与热

| 功能/接口 | 输入 | 实现逻辑和输出 | 设计参数/标定及依赖 |
| --- | --- | --- | --- |
| `CurrentLimitDetector_Step` | 三相瞬时电流、有效窗口、时间 | 最大绝对值分档确认；软件严重阈值走快速请求；非有限值归采样失效 | D：trip/确认/恢复；B：可靠量程；C：各通道offset/gain；不依赖控制器增益 |
| `HwTripDetector_Step` | 平台锁存事件、驱动器诊断字 | 先硬件关断，软件仅分类记录；区分VDS、欠压、过温及未知；瞬态不因轮询错过 | B：比较器连接、门限、消隐、驱动器型号；D：子原因映射；无需软件滤波推迟关断 |
| `PowerStageDiagnostic_Evaluate` | 维护试验结果、禁止状态、相电压/电流有效位 | 返回疑似/确认损坏/未验证；排除回馈和续流后才认定损坏 | B：实际具备的电压/电流测量；D：受限试验规程与通过范围；不是在线故障自动重试器 |
| `BusVoltageDetector_Step` | 原始/滤波母线电压、年龄、输出状态 | 原始快速过压、滤波过压/欠压分别计时；停机仍检测测量有效性与使能窗口 | D：告警/跳闸/恢复/上电稳定时间；B：器件允许母线上限/供电profile；C：分压增益/偏移 |
| `SupplyRailDetector_Step` | UVLO/PVD/BOR/电源good、valid | 栅极/逻辑电源异常直接否决有关能力；正常上电窗口结束前禁止启动，不误报已运行欠压 | B：电源监测能力/阈值；D：启动时限/稳定时间；BOR复位原因启动后记录 |
| `TemperatureDetector_Step` | 单个温度源、raw/filtered值、valid/年龄 | 每个源独立ctx；告警、降额、跳闸、恢复回差；无效值转sensor fault | D：各源阈值/时间；B：测点及热滞后；C：该源转换校准；严禁MCU值替代绕组值 |
| `TemperatureValidity_Step` | 原始ADC/参考电压、转换结果和时间戳 | 越轨、转换非有限、超时、变化率诊断；变化率仅用于合理性，不擅自忽略高温 | B：NTC接法/有效ADC区间或MCU工厂系数；D：年龄上限；C：NTC/通道修正 |
| `ThermalLoad_Step` | 实际电流、速度/冷却条件、温度边界、dt | 电机与逆变器各自ctx；更新经验证的热模型，输出热余量和降额请求；停机继续冷却 | M/B：连续/峰值能力曲线；C：热常数/模型误差；R：热状态。禁止用普通Reset清热量 |
| `RegenDetector_Step` | 母线、制动命令/电流/温度、能量积分 | 检查有激励时开短路疑似、脉冲/平均功率、母线响应；缺少电流反馈则能力标记降级 | B：电阻、开关及供电回馈能力；D：确认窗口；C：制动电流/温度测量修正 |

### 5.2 采样、反馈与运动

| 功能/接口 | 输入 | 实现逻辑和输出 | 设计参数/标定及依赖 |
| --- | --- | --- | --- |
| `CurrentCalibration_Check` | 已确认零电流条件下的原始窗口 | 求均值/方差/通道范围；产生候选offset和质量结果；未满足零电流条件拒绝标定 | B：ADC归一化规则/放大倍数；D：窗口/噪声及offset边界；C：offset/gain |
| `AdcValidity_Step` | 样本序号、年龄、外设错误、越轨、有效采样窗口 | 检测漏采/陈旧/饱和；三相独立测量才启用和电流残差；重构相不可作为独立证据 | D：新鲜度/残差/窗口；B：触发及过采样配置；C：通道误差预算 |
| `EncoderValidity_Step` | 协议层帧结果、CRC/状态字、有效样本时间 | 协议驱动先拒收坏帧；检测器独立维护连续失败与最大年龄，输出可信角度能力 | B：编码器型号/读取周期；D：坏帧窗口、电角度不确定量；C：极对数/方向影响匹配 |
| `EncoderPlausibility_Step` | 机械角度、实际dt、速度界、可选独立运动证据 | 环形差值与物理运动包络比较；冻结需要独立运动证据，否则只报运动异常疑似 | M：速度/加速度界、传动关系；C：分辨率/方向；D：裕量；不强依赖无感观测器 |
| `PositionReference_Check` | 上电/复位、多圈连续性、零位与配置版本 | 判断位置基准有效性；丢失则要求回零或受限重建，不把角度帧正常等同位置有效 | C：机械零位/多圈基准；M：坐标系与机构；R：连续性状态 |
| `OverspeedDetector_Step` | 可信实际机械速度、年龄 | 告警/跳闸/紧急包络；与目标限幅解耦；速度无效交给反馈故障 | M：电机/传动/机构最小允许上限；D：确认/裕量；C：比例/方向 |
| `TravelDetector_Step` | 目标/实际位置、速度、位置有效位、停车能力 | 非法目标拒绝；实际越界跳闸；剩余距离小于最坏停车距离提前停止；方向独立 | M：软限位/硬边界/制动能力；C：机械零位；D：裕量；接入现有AxisProfile，不再存第二份边界 |
| `LimitSwitchDetector_Step` | 硬限位输入及诊断、方向 | 规定边沿触发；断线/通道冲突独立报错；故障退出只能走受限恢复任务 | B：输入极性和诊断能力；D：去抖与响应预算；C：限位位置验证记录 |
| `FollowingErrorDetector_Step` | 同时刻内部轨迹参考、实际位置/速度、模式能力 | 误差窗口＋时间；阻抗模式采用专门允许包络；滤波/参考延迟计入误差预算 | D：位置/速度软硬阈值、时间；M：允许跟踪误差；不使用远端最终目标代替轨迹参考 |
| `StallDetector_Step` | 确认运动意图、实际速度、电流、抱闸状态 | 要求运动、抱闸已释放、低速和高负载持续才报受阻；位置保持/纯转矩模式不默认启用 | D：意图阈值/低速/电流/时间；M：负载与正常启动包络；无反馈可信度则停止诊断而非判健康 |
| `PhaseResponseDetector_Step` | 有效激励、独立相电流、观察窗口/角度覆盖 | 比较应有响应和实际响应；跨充分窗口确认缺相疑似；零交越不能触发 | D：最小激励/响应比/窗口；M：R/L模型范围；C：电流通道；缺独立观测则不上线该功能 |
| `BrakeMonitor_Step` | 抱闸命令、线圈电流、机械反馈、位置/速度 | 动作超时与保持滑移分开；通电不等于释放；承载能力由受保护测试证明 | B/M：抱闸动作曲线、容量；C：验证的时间/保持能力；D：漂移窗口与测试约束 |
| `StopMonitor_Step` | 停车开始时间、可信位置/速度、反馈能力 | 监测时间、减速/位移包络、反馈丢失；输出完成/继续/升级；不自产生控制转矩 | M：保证停车能力；D：最长停车时间/包络；R：停车状态与run_epoch |

### 5.3 通信、计算与任务

| 功能/接口 | 输入 | 实现逻辑和输出 | 设计参数/标定及依赖 |
| --- | --- | --- | --- |
| `CommandWatchdog_OnAccepted` / `Step` | 已通过业务校验的新目标元数据、owner接受时刻 | 明确从使能前新鲜目标开始计时；仅新目标续期；接收排队年龄也校验；不解析CAN | D：目标周期/抖动/超时；R：session/seq/boot/run_epoch；序号不是标定参数 |
| `TaskKeepalive_OnAccepted` / `Step` | 自主任务ID、新保活序号、时刻 | 有限轨迹/回零/标定独立保活；查询和重复包不刷新；停止后任务不可自动续跑 | D：保活预算/任务期限；R：任务代号；与实时目标watchdog明确互斥适用 |
| `TransportHealth_Step` | 平台bus-off、RX/TX溢出、消息年龄 | bus-off即时报告；孤立丢包计数；持续过载按控制新鲜度预算判定 | D：统计窗口/上限；B：总线配置；与任何传输重连自动清故障解耦 |
| `DeadlineMonitor_Step` | release/start/finish时间、实际PWM截止点、外环epoch | 区分快环截止失败与外环mailbox漏完成；完成时间晚于截止不能再应用结果 | D：WCET/截止裕量；B：计时源/频率；C：无，实测时序是验证证据而非任意可写标定 |
| `HealthSupervisor_Step` | 各关键任务进度代号/年龄 | 只有必需任务在窗口内推进才允许喂独立看门狗；普通周期中断不能独自续命 | D：任务健康窗口；B：看门狗时基误差/复位行为；R：上次进度 |
| `RuntimeValue_Check` | 关键目标、角度、电流、控制输出 | 非有限值、非法范围拒绝进入控制/执行；CRC不能替代运行数值检查 | D：字段范围；B/M：可达上限；高优先级不得调用格式化日志 |
| `Integrity_Check` | 配置/镜像元数据、CRC结果、硬件自检结果 | 数据无效与确认存储硬件损坏分开；自检破坏性操作不在运行中执行 | D：schema/算法/覆盖范围；C：记录CRC与身份；签名需求独立评审 |
| `CalibrationMonitor_Step` | 任务状态、时间/行程/电流、质量结果 | 极对数/参数无效、编码器/R/L/摩擦/齿槽质量失败分别子码；测量算法留原模块 | D：阶段期限/激励上限/质量门限；M：物理允许区间；C：只在验证通过后生成候选记录 |
| `HomingMonitor_Step` | 阶段、原点/index、绝对累计行程、时间、位置有效位 | 退出/搜索/退回/精定位逐阶段限时限程；仅完成并验证后设置基准有效 | M：行程方向；D：速度/时间/重复性；C：原点偏移 |
| `SafetyInputMonitor_Step` | 独立安全回路反馈、通道状态 | 只记录停止请求和通道故障；实际安全动作由独立路径执行，不等待APP | B：实际安全硬件；D：经验证的通道差异预算；不存在此硬件就声明不支持 |

旧 `Large_Phase_Resistance` 可对应“稳态等待超时”，不能以名字推断阻值超限；
旧 `Sensorless_Error`、`Encoder_Error` 等按任务/运行/通信子原因拆分观测。
相电阻不平衡标志变成跳闸前必须明确适用阶段和质量标准，不能悄悄改变现有运行行为。

## 6. 参数分层、所有权与发布

### 6.1 分层

| 参数类别 | 示例 | 来源/保存与权限 |
| --- | --- | --- |
| B 板级设计 | 分流电阻、放大倍数、ADC归一化、分压比、硬件限值、保护引脚、驱动器特性 | 编译/板卡profile；与硬件版本绑定；普通运行接口不可改 |
| D 保护设计 | 阈值、确认/恢复时间、跳闸动作、恢复权限、检测适用条件 | 版本化ProtectionPolicy；出厂验证后冻结；可调字段仅允许受审范围 |
| M 电机/机构 | 极对数、额定/峰值能力、传动比、关节边界、保证减速度、抱闸能力 | Motor/Axis profile；设计资料及应用验证；身份改变触发兼容检查 |
| C 单机标定 | 电流offset/gain、母线gain/offset、编码器LUT与零位、温度修正 | 独立版本CalibrationRecord；有样本质量、身份和CRC；不改变保护硬边界 |
| R 运行状态 | 热积累、连续确认计数、最后有效目标、故障锁存、epoch | RAM；热重启策略和维修锁定持久化单独设计；不混入标定record |

“用于生成标定数据的配置”属于D/M，测得并验证的结果才属于C。
控制器PID、摩擦/齿槽补偿系数不由保护模块维护；保护仅接收其有效性或版本。

### 6.2 参数目录（首版至少覆盖）

| 参数组 | 字段/单位 | 校验关系及失效规则 |
| --- | --- | --- |
| Current | `sw_trip_a, severe_trip_a, confirm_us, recover_a, recover_us` | 峰值定义一致；普通目标上限不得超过允许控制包络；测量误差/纹波和硬件容差单独预算，软件阈值不能作为硬件短路保证 |
| Bus | `uv_trip_v, ov_trip_v, hard_ov_v, enable_min_v, enable_max_v, uv_us, ov_us, hard_us, stable_us` | 恢复/使能窗口在允许工作区内；阈值＋误差＋动态过冲小于系统允许边界；滤波延迟纳入总预算 |
| Temperature[source] | `warn_c, derate_c, trip_c, recover_c, max_age_us, stable_us` | 来源独立；通常recover低于trip；无传感器不能填0℃冒充有效 |
| Thermal[object] | `i_cont_a, peak_curve, tau_s/model, warn_load, trip_load, recover_load` | 电流单位一致；模型版本/适用温速区间有效；初始热状态未知采用保守策略 |
| Encoder | `max_age_us, max_bad_streak, max_elec_uncertainty_rad, angle_margin_rad` | 依据最高机械速度与极对数反推失效预算；坏帧计数与年龄任一越限即失效 |
| Motion | `max_speed_rad_s, pos_error_rad, speed_error_rad_s, confirm_us, stop_timeout_us, stop_margin_rad` | 与AxisProfile和模式匹配；停止预算包含检测/调度/执行及机械响应 |
| Communication | `target_period_us, target_timeout_us, keepalive_timeout_us, max_queue_age_us` | 正常最坏间隔＋裕量小于超时；超时＋停车不超过系统允许响应 |
| Execution | `fast_deadline_us, outer_deadline_us, health_window_us` | 与实际PWM截止点及外环调度一致；看门狗最坏误差不能超过允许失控时间 |
| Brake/Regen | `release_us, engage_us, drift_rad, energy_j, power_w` | 依硬件能力启用；关键能力缺失时不允许依赖该策略的轴运行 |
| Recovery | `required_checks, stable_us, restricted_speed/current/travel/time` | 恢复动作包络比正常运行更受限；不能解除硬件故障/急停等不可旁路条件 |

当前可复用的代码值：母线24/34/34.5V、100ms/2ms/3周期、使能25.6～33.8V；
默认2mΩ软件过流40A/5周期；MCU90℃/100ms失效；电流offset范围1948～2148个归一化12位计数。
这些是基线，不构成新增阈值的验证依据。回差、传感器误差、硬件短路时序、热参数和机械停车包络仍需验证。
34.5V到源码注释所述35V边界的余量尤其需要计入测量误差与真实关断过冲，不能只比较常数大小。

### 6.3 发布与持久化

1. 服务将输入解码到候选配置；检查数值有限、类型、单位、身份、版本、范围和关联关系。
2. 与板级硬边界和电机/轴包络求交；非法配置整条拒绝，返回字段与原因，不静默截断。
3. 配置能力检查：required但不支持的监测/动作使对应轴或模式不可使能。
4. 计算周期数、比例和模式表，得到只读RuntimeProtectionConfig；不在快环计算CRC或分配内存。
5. 通过带revision的邮箱交owner，在未使能且无活动维护运动时于边界切换；返回applied确认后才能复用旧缓冲。
6. 不跨越两个配置累计同一连续确认；但热积累和维修锁存不能清零。热模型切换需要保守状态映射或冷却条件。
7. save是独立操作。新记录含magic/schema/长度/身份/算法版本/CRC/generation，定义断电写入与选择规则。
8. 写Flash仅在输出已验证禁止且调度允许时执行。保护记录容量、旧记录保留和迁移需先验证，不能假设有额外存储空间。
9. 旧配置迁移失败进入禁止使能并给出诊断；不能默认恢复出厂值后继续运动。

### 6.4 标定流程与依赖失效

所有标定均为显式任务：PRECHECK → ACQUIRE → COMPUTE → VALIDATE → CANDIDATE → APPLY → 可选SAVE。
关键保护始终有效；候选与当前有效记录分离；失败保留旧记录但标记其适用性，绝不部分覆盖。

| 标定项 | 前提与方法 | 输出/验证 | 需要失效的关联数据 |
| --- | --- | --- | --- |
| 电流零偏 | 驱动禁止且实际无电流、参考电源稳定；采集窗口均值/方差 | 三相offset及噪声质量；范围通过 | 分流/增益/ADC配置改变后旧电流域标定需重验 |
| 电流增益 | 受控台架、可追溯电流参考、多点正负电流 | 每相gain/offset/残差与温度 | 电流域摩擦/齿槽补偿和限值换算需重新验证 |
| 母线测量 | 多个安全电压点与参考表比对 | gain/offset、全区间残差 | 分压/参考源/ADC变化使其失效；不能靠调高跳闸阈值掩盖误差 |
| 编码器LUT/电角零位 | 相应低电流受控任务、行程/时间约束、完整样本覆盖 | LUT/方向/电角零位、RMS/峰值残差 | 更换编码器/电机、安装方向或极对数改变时重新验证 |
| 机械零位/回零 | 已知原点、受限行程及重复性验证 | 坐标基准与质量 | 依赖该坐标的限位、轨迹与补偿映射需匹配 |
| 温度测量 | MCU使用工厂系数；外部传感器按实际电路与参考点 | 修正系数、误差及适用范围 | 传感器/接法改变使旧记录失效；MCU系数不应用到其他测点 |
| 热模型 | 受限负载、温度参考、不同工况升温/冷却试验 | 模型、误差界、有效工况 | 电机/散热/安装/冷却改变后重验 |
| 停车/抱闸能力 | 明确台架、负载、供电、独立机械防护 | 最坏停止时间/距离、抱闸时间与保持能力 | 负载/机构/电源/制动器变化后重验；不是自动辨识随意覆盖的参数 |

标定记录至少带board/motor/encoder/axis身份或配置指纹、算法revision、测量质量、有效标志与generation。
无可信实时时钟时保存boot_id和单调时标；不伪造绝对时间。

## 7. 调度、并发与故障时序

### 7.1 执行域

| 域 | 工作 | 有界要求 |
| --- | --- | --- |
| 独立硬件/异常 | 立即禁止驱动，锁存最小原始原因 | 无需等待快环/主循环；CPU死机仍需硬件路径承担其设计范围 |
| 20kHz owner | 电流/快速电压/样本有效性、接收观测、FaultCore、最终输出许可、快照首发捕获 | 不等待慢任务、Flash、CAN或日志；度量增加的WCET |
| 1kHz/慢监督 | 温度/热模型/通信健康等独立检测器 | 只拥有自己的ctx；输出交邮箱，不直接改FaultCore |
| 异步外环 | 位置/速度/轨迹控制及相关观测 | 返回结果带run_epoch/config_revision；过期结果拒绝应用 |
| 主循环 | 协议编码/发送、记录存储、非实时诊断 | 拥塞不改变保护时序；对每轮工作量设上限 |

### 7.2 邮箱与事件

- 每个异步生产者使用自己的固定容量SPSC通道，禁止多生产者偷偷共用SPSC队列。
- critical平台关断不依赖队列成功。事件满时保留本来源的sticky trip与原始时间，owner确认后释放；溢出另报诊断。
- 健康状态可合并，但不可用新健康值覆盖尚未接收的trip。恢复时间按样本时标/连续观测算，不按队列出队次数算。
- 核心只由owner修改；后台读取复制快照。32位控制字及release/acquire语义必须在目标编译器实现并测试；volatile本身不是同步协议。
- owner不得使用等待低优先级writer完成的自旋seqlock；读到未完成发布就本周期跳过，数据超龄按unknown处理。
- 快照包含fault_seq、sample_seq、run_epoch、config_revision，诊断异步信号还包含各自采样年龄，不能声称不同时间采样完全同步。
- 事件输出队列是核心单生产者、后台单消费者；首发现场独立保留，队列满不能抹掉首发证据。

### 7.3 快速环顺序

```text
取得可用采样及外部sticky trip
→ 执行快速检测并接收有界异步观测
→ 核心裁决；若禁止输出则先ForceOff
→ 仅在允许时执行正常控制或受控停止
→ 接收控制计算错误/超期，再次检查输出许可与epoch
→ 写入合法PWM或保持禁止
→ 捕获/发布有界状态，结束周期
```

运行故障使run_epoch失效，撤销待应用目标和旧外环任务。慢任务后到的“成功完成”不能重新使能。
受控停止使用新的本地停车上下文；仍每周期检查反馈与功率级，严重故障抢占停车。
启动/复位时默认输出禁止，先验证配置、采样、联锁、目标新鲜度和必要基准，再单独使能。

## 8. 清除、恢复与状态机例外

- R0仅撤告警；R1明确清除；R2需要服务验证；R3需要维修记录/验证且防普通重启绕过。
- 清除请求必须在owner再次检查实时故障与平台sticky状态之后处理；同周期新故障优先，清除不得覆盖新trip。
- 恢复基于有效、版本匹配且足够新鲜的证据；unknown、不适用和停机后未受激励均不能自动证明修复。
- 普通Clear不允许仍有功率输出或活动恢复运动；返回cleared/remaining/rejected_reason，不把排队成功当清除成功。
- Clear不解除所有启动条件，不恢复旧运动；重新enable必须使用新run_epoch及新鲜目标。
- 零偏错误时允许PWM禁止的采样校准；不能因“有故障禁止所有任务”形成死锁。
- 软限位退出、回零/有激励标定走独立RECOVERY任务，指定故障白名单和受限包络，保留故障锁存直到验证完成。
- RECOVERY不可旁路硬件短路、关键反馈失效、外部安全输入或缺失的机械保持能力；拒绝未定义的恢复路径。
- 抱闸不具备反馈时不能发布“已确认抱紧”；停机需要抱闸但能力不存在时该轴不具备相应运行资格。
- 无法验证STOP或输出禁止时，台架进入硬失败并要求物理介入，禁止工具自动续测。

## 9. 遥测与协议兼容

1. 旧ErrorNow是FaultCore发布的兼容投影；固定映射和优先顺序，保留首发字段独立查询。
2. 未分配旧编号的新故障需明确兼容策略和主机支持版本；禁止映射成No_Error或截断丢失。
3. 现有48B状态流保持原字段含义；通过新命令/新版本发布完整位图、valid/unknown、子原因、动作和epoch。
4. 周期摘要允许合并；故障事件有独立优先队列、event_seq、限频及受限重试，旧事件查询支持发现缺口。
5. ACK只代表请求处理结果；清除、保存、标定有accepted与completed区别。
6. 不因CAN断联而停止本地保护，也不因总线恢复清除故障。
7. 协议扩展必须同时更新版本文档、golden vectors、旧主机兼容测试；本设计不分配线上编号。
8. 事件Flash持久化后台执行并设磨损预算；写失败单独报告，不能阻塞关断或用旧日志冒充本次现场。

## 10. 验收与实施分批

| 阶段 | 交付 | 关键验证 |
| --- | --- | --- |
| P0-A | 平台关断契约、最小FaultCore、统一输出许可 | ForceOff幂等；使能/关断竞争；故障前旧外环结果被拒绝；单故障及并发故障 |
| P0-B | 条件清除、恢复证据、目标watchdog | 查询不续命；重复/过期消息不续命（新协议）；自动重连不清除；清除/新trip竞态 |
| P0-C | 编码器完整诊断、硬件关断/异常与健康监督 | 原生坏帧向量、失效年龄；快环WCET；经授权台架验证关断路径与最坏延迟 |
| P1 | 采样有效性、模式/轴表、运动边界、事件服务 | NaN/饱和/陈旧；限位恢复；队列满；快照一致性；协议版本及字节向量 |
| P2 | 热、缺相/堵转、维护诊断及持久记录 | 模型边界/热重启；工况误报率；断电记录选择；实际硬件能力匹配 |

每个检测器离线测试：阈值上下侧、等于阈值、确认边界、回差、计时回绕、计数饱和、unknown、工况退出/进入、配置更换和错误输入。
时标用无符号差值，所有比较窗口受小于半个计数周期的约束；明确us时基回绕和长期停机复位规则。
故障核心另测并发trip、首发不覆盖、动作组合、恢复白名单、请求重放、配置/运行epoch变更。
原生测试通过不证明输出物理安全，硬件验收必须记录fault输入到功率输出禁止的完整延迟和残余运动。

文档改动运行quick及链接检查；后续固件/协议/参数改动运行PR；CubeMX/硬件配置/构建输入改动运行release。
release不烧录；硬件台架操作须显式bench身份、电机profile、镜像hash和场景，输出归outputs。
本设计未设任意“通用安全阈值”，所有TBD参数和不具备能力的功能应在各阶段完成前关闭对应运行资格。
