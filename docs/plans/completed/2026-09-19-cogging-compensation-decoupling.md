# 齿槽转矩补偿解耦与运行路径优化 v1.0

日期：2026-09-19。状态：已完成；PR 验证通过（未提交、未烧录）。

## 目标与范围

将齿槽转矩补偿从“标定适配、运行补偿、台架保护、两套遥测混装”的单文件拆为
“纯算法核心 + 薄适配层”：

- 新增纯核心 `cogging_compensation`（准入、查表、渐变、限幅）与
  `cogging_torque_guard`（台架租约/测速/指令判定）。
- 纯核心只收原始类型，不包含平台、服务或应用头文件，可在主机上独立编译测试。
- 删除适配层与核心之间重复的准入判据、重复的 blend 清零、别名宏和魔法数。
- 为全部公共接口补齐中文 Doxygen 契约，为补偿关键不变量补充中文注释。

不改公共符号名、结构体布局、参数 ABI、CAN 行为、RTT 布局，不改渐变率、钳位次序、
准入条件集合与身份复核策略；调用方 `foc_run.c`、`interface_can.c`、`foc_task.c`、
`foc_errhandle.c`、`foc_param.c`、`main.c` 保持逐字节不变。

## 现状问题（证据）

1. `foc_cogging_calibration.c` 单文件混装四类职责：标定适配、运行补偿（:35-94）、
   台架保护（:96-116）、两套遥测（:118-133）。
2. 准入判据重复：`FocCogging_SetCompensation`（:41-48）与 `FocCogging_Apply`（:64-68）
   各维护一份“模式/错误/无感/表有效/身份”判据，需人工同步。
3. 冗余：`blend=0` 三处（:71、:113、:314/:316）；`SETTLE_TICKS`/`SAMPLE_TICKS`
   别名宏（`cogging_calibration.c`:5-6）；插值魔法数 `>>6`、`&63`、`/64`、`&1023`、
   `±1.0f`、`5.0f*Current_Ts`（:80-87）。
4. 耦合：补偿路径直接读 `CURRENT_SENSE_PROFILE_FULL_SCALE_A`、`Encoder_TypeDef`
   字段与 `Current_Ts`，无法脱离硬件单测；换算使用编译期满量程而非表内保存值。
5. 注释：两个头部共 11 个公共接口处于 `interface_debt.json` 基线，无中文契约。

## 目标结构

| 文件 | 职责 | 改动 |
| --- | --- | --- |
| `firmware/motor/foc/cogging_compensation.{c,h}` | 运行补偿纯核心：准入身份快照、每拍判据、查表、渐变、限幅 | 新增 |
| `firmware/motor/foc/cogging_torque_guard.{c,h}` | 台架保护纯判定与力矩遥测类型 | 新增 |
| `firmware/motor/identification/cogging_calibration.{c,h}` | 标定状态机与记录（原职责）+ `CoggingMap_LookupA` 纯插值 | 修改 |
| `firmware/motor/identification/foc_cogging_calibration.{c,h}` | 唯一适配层：采集硬件事实、调用核心、执行停机；保留全部 `FocCogging_*` 公共接口 | 修改 |

约束：适配层必须保留现有文件名，因为架构债务按“源文件 -> 目标文件”精确记账，
任何新 `.c` 直接包含 `hw_conf.h`/`encoder.h` 都会新增越层违规；因此新文件保持纯核心，
硬件胶水只在既有适配文件中。

## 接口设计（锁定）

### `cogging_calibration.h`

```c
#define COGGING_MAP_POINTS_LOG2 10U
#define COGGING_MAP_INDEX_SHIFT (16U - COGGING_MAP_POINTS_LOG2)
#define COGGING_MAP_INDEX_MASK ((1U << COGGING_MAP_INDEX_SHIFT) - 1U)
float CoggingMap_LookupA(const CoggingMapRecord *record, uint16_t position_q15);
```

`LookupA` 按 16 位线性化机械角在 1024 点表上跨圈线性插值，使用表内 `full_scale_a`
换算为安培；20 kHz 热路径，不做校验、不访问硬件。

### `cogging_compensation.h`（新增，纯核心）

```c
#define COGGING_COMPENSATION_BLEND_RATE_PER_S 5.0f  /* 200 ms 满幅 */
#define COGGING_COMPENSATION_LIMIT_A 1.0f            /* 查表输出保护上限 */

typedef struct { uint32_t request, enabled, rejected;
    float blend, table_a, applied_a, total_a; } CoggingCompensationControl;
extern volatile CoggingCompensationControl CoggingCompensation;

typedef struct { uint8_t reverse; uint16_t electrical_zero_q15;
    int32_t pole_pairs; uint32_t table_crc; } CoggingCompensationIdentity;

typedef struct { bool mode_accepts_request, error_clear, sensorless_off,
    table_valid, scale_matches; CoggingCompensationIdentity identity;
} CoggingCompensationAdmitInfo;

typedef struct { bool mode_is_current, error_clear, sensorless_off, encoder_usable;
    bool guard_enabled; float guard_limit_a, command_a, limit_a, tick_s;
    uint16_t position_q15; CoggingCompensationIdentity identity;
} CoggingCompensationTick;

bool CoggingCompensation_Admit(volatile CoggingCompensationControl *control,
    const CoggingCompensationAdmitInfo *info);
void CoggingCompensation_Disable(volatile CoggingCompensationControl *control);
void CoggingCompensation_Shutdown(volatile CoggingCompensationControl *control);
void CoggingCompensation_ZeroBlend(volatile CoggingCompensationControl *control);
float CoggingCompensation_Update(volatile CoggingCompensationControl *control,
    const CoggingMapRecord *record, const CoggingCompensationTick *tick);
```

- `Admit`：全部准入事实成立时快照身份并置 `request=enabled=1`、`rejected=0`；
  否则撤销准入、清 `request/enabled`、置 `rejected=1`。与现有
  `FocCogging_SetCompensation(true)` 逐分支等价。
- `Disable`：`request=enabled=0`，保留准入身份，使 `Update` 继续渐出（现有
  平滑关闭语义）。
- `Shutdown`：`request=enabled=0` 且 `blend=0`（现有台架 trip 与离开允许模式语义）。
- `ZeroBlend`：仅 `blend=0`（现有 `Motor_Disable` 模式下的预开启保持语义）。
- `Update`：按现有 `FocCogging_Apply` 的顺序执行：基础限幅 -> 台架限幅叠加 ->
  资格判定（不合格：`blend=0`、`enabled=0`、必要时清 request）-> 插值 + ±1 A 钳位 +
  渐变 -> 指令钳位 -> 总指令钳位 -> `applied_a` 回写，返回总指令。
  资格判定使用缓存 CRC 字段比较，禁止每拍重算整表 CRC。

### `cogging_torque_guard.h`（新增，纯判定）

```c
#define COGGING_TORQUE_GUARD_OK 0U
#define COGGING_TORQUE_GUARD_TRIP_LEASE 1U
#define COGGING_TORQUE_GUARD_TRIP_SPEED 2U
#define COGGING_TORQUE_GUARD_TRIP_CONFIG 3U

typedef struct { uint32_t enabled, lease_ticks, trip;
    float speed_limit_rad_s, current_limit_a; } CoggingTorqueGuard;
extern volatile CoggingTorqueGuard TorqueGuard;

typedef struct { uint32_t tick; float position_rad, velocity_rad_s, command_a,
    compensation_a, total_a, feedback_a, vbus_v, blend; } CoggingTorqueFrame;
extern volatile uint32_t TorqueTelemetryState;
extern volatile CoggingTorqueFrame TorqueTelemetry;

uint32_t CoggingTorqueGuard_Check(volatile CoggingTorqueGuard *guard,
    float speed_rad_s, float command_a, bool sensorless);
```

`Check` 只做判定与租约递减并记录 `trip`，不操作 PWM/模式；动作留给适配层。

## 行为保持清单

- 渐变率：`5.0f * tick_s`（`tick_s = Current_Ts`），先钳位后加；`enabled` 目标值 0/1。
- 限幅次序：先 `fmaxf(0, current_limit)`（非有限值取 0），再与台架限流取小，
  再对指令和总指令分别钳位；`applied_a = total_a - command`。
- 准入条件集合：模式 ∈ {Disable, Current_Mode}、无故障、非无感、表 CRC/签名有效、
  表满量程等于当前采样配置。
- 运行条件集合：已准入、模式 == Current_Mode、无故障、非无感、编码器在线且标定完整、
  方向/电零位/极对数/表 CRC 与准入快照一致。
- 表满量程换算改用 `record->full_scale_a`；准入已强制其等于
  `CURRENT_SENSE_PROFILE_FULL_SCALE_A`，已准入状态下数值逐位相同。
- 公共符号、`CoggingMapRecord`、`CoggingCompensationControl`、`CoggingTorqueGuard`
  布局与名称不变；CAN 0x26/0x27 路径不变。
- 适配层公共函数名单：`FocCogging_CanStart/Task/Abort/Service/TableValid/GetState/
  SaveResult/SetCompensation/Apply/TorqueGuard/TorqueObserve` 全部保留。

## 验收标准

1. 新核心不包含 `firmware/platform`、`firmware/services`、`firmware/app` 头；
   `check_architecture` 0 新增违规，`check_project_layout` 通过。
2. 调用方文件与参数 ABI 不变；公共接口契约（`check_interfaces`）0 新增违规。
3. 原生测试扩展通过：插值跨圈/中点/符号/满量程、准入与渐出、身份失配即退、
   台架三种 trip；原有标定与 FOC 适配用例保持通过。
4. `verify --profile pr` 全绿；涉及文件的 style/interface 债务条目清理。
5. 双 Keil 工程列入新源文件；可用时完成本地双目标构建，否则如实记录未构建范围。
6. 文档更新：本计划、指南模块归属说明、计划索引、债务清单；RTT/J-Link 变量名不变，
   RAM 地址变化按构建记录说明。

## 进度

- [x] 纯核心 `cogging_compensation`、`cogging_torque_guard`
- [x] `cogging_calibration` 插值助手、清理与契约
- [x] 适配层重写与调用方兼容
- [x] Keil 工程与原生测试
- [x] 文档与债务清理
- [x] PR 验证与证据

## 决策记录

- 2026-09-19：适配层保留 `FocCogging_*` 接口与文件名，调用方零改动；解耦通过纯核心达成，
  避免 4 个 legacy 文件近 3000 行机械重排。
- 2026-09-19：准入身份快照放在核心文件内静态存储，不改变公共结构体布局。
- 2026-09-19：运行换算改用表内 `full_scale_a`，去掉对编译期采样常量的热路径依赖。
- 2026-09-19：`Current_Ts` 由适配层以 `tick_s` 传入，核心不依赖平台头文件。
- 2026-09-19：台架保护判定与遥测类型独立成模块，生产补偿核心不再引用台架状态。

## 证据

- 原生 C 测试（合成编码器/电流，含新增纯核心用例）：
  `tests/unit/native/test_cogging_calibration.py` 六组断言全部 PASS；新增覆盖插值
  中点/子步/跨圈/表内满量程、台架三类 trip、未准入限幅与台架限流收窄。
- `verify --profile pr`：PASS（doctor、format、lint、project-layout、tests、architecture、
  interfaces、docs、hygiene 全部通过）。证据：
  `outputs/runs/20260918T170930645580Z-3a84df1c/summary.json`，`hardware_contacted=false`。
- 调用方逐字节不变：`git diff --stat` 对 `foc_run.c`、`interface_can.c`、
  `foc_errhandle.c`、`foc_param.c`、`main.c` 为空；公共符号、结构体布局与参数 ABI 未改。
- 两个 Keil 工程均列入 `cogging_compensation.c` 与 `cogging_torque_guard.c`；
  `check_project_layout` 通过（普通/HIL 分别 78/79 个源文件，0 错误）。
- 债务清理：删除 `style_debt.json` 5 项与 `interface_debt.json` 2 个文件的已解决条目；
  架构债务 0 新增（38 项已知）。
- 首次 PR 运行中 `run_position_servo_tests` 失败为并行会话共享 Zig 全局缓存的
  目标文件竞态；设置独立 `ZIG_GLOBAL_CACHE_DIR` 后该用例与整套 verify 均通过，
  与本次改动无关。
- 本轮未提交、未烧录、未访问硬件；RTT/J-Link 变量名与参数 schema 不变，
  全局变量 RAM 地址随构建变化，调试需使用配套 AXF。

## 后续变更：台架保护内联精简（2026-09-19）

按“最低必要”原则收敛台架保护，生产补偿核心与调用方行为不变：

- 删除 `motor/foc/cogging_torque_guard.{c,h}`（约 100 行）与两个 Keil 源文件条目；
- `CoggingTorqueGuard` 收敛为 `{enabled, lease_ticks, trip, speed_limit_rad_s}`，删除
  `current_limit_a`、配置/指令校验（原 trip 3）及其在补偿核心中的限流收窄分支
  （`CoggingCompensationTick` 不再携带台架字段）；
- 租约与测速判定内联到适配层 `FocCogging_TorqueGuard`（约 10 行）：`trip=1` 租约到期、
  `trip=2` 测速超限；测试电流包络由 `MotorControl.current_limit` 保证；
- `TorqueGuard`、`TorqueTelemetryState`、`TorqueTelemetry` 符号名与帧布局不变，
  J-Link/RTT 观察方式不变；定义位置移回适配层。

理由：台架保护唯一不可替代的能力是“主机心跳租约”，其余判定与生产保护或测试配置重复；
剩余逻辑不值得独立模块、独立测试与两处工程条目。本变更取代“决策记录”中
“台架保护判定与遥测类型独立成模块”的组织方式。

## 后续变更：CRC32 共用（2026-09-19）

消除 `Cogging_Crc32` 与 `motor_axis_profile.c:record_crc` 的重复实现（算法逐位相同）：

- 新增 `firmware/common/crc32.{c,h}`（`Crc32_Compute(data, bytes, seed)`），两份调用方都改用它；
- `Cogging_Crc32` 从 motor 公共接口移除；`RecordCrc`/`Cogging_EncoderSignature` 改用共用实现，
  魔法数改为 `offsetof(CoggingMapRecord, crc32)` 与 `COGGING_MAP_POINTS`；
- `motor_axis_profile.c` 触碰后完成格式/可读性/中文注释清理，并删除其样式债务条目；
- 两个 Keil 工程均列入 `common/crc32.c`；相关测试编译清单同步更新；
- 正确性由 AXS1 黄金向量（zlib 对拍）与齿槽表 CRC 用例双重验证。

## 后续变更：辨识/适配层简化（2026-09-19，已实施部分）

按“档位1 + 诊断字段删减”无损包实施，行为逐位不变（策略与安全门保留）：

- `CoggingCalibration_Update` 126 → 59 行：提取 `DeadlineExceeded`、`SaturationLimitReached`、
  `SampleIsStable`、`RejectTick`、`CompletePoint`；
- `FocCogging_Task` 170 → 34 行编排：提取 `StartSession`、`SessionIsSafe`、`ServiceTick2kHz`，
  另提取 `BusIsSafe`/`TemperatureIsSafe`/`LimitsAreUsable`/`SessionIdentityHolds`/
  `FillFaultSnapshot`/`PublishCalibrationFrame`/`HoldCurrentPoint`/`TryBeginSave`；
- 诊断删减：`CoggingCalibration` 删除 `accepted_sample_ticks`、`sample_restarts`、
  `last_sample_accepted`、`rejected_ticks`、`max_sample_error_rad`、`sample_error_sq_sum`；
  `CoggingTelemetryFrame` 删除 `accepted`；J-Link 字段清单同步更新；
- 补偿核心 119 → 95 行：`Shutdown` 并入 `Disable`+`ZeroBlend` 组合，身份比较内联；
- 测试同步：cogging 原生测试与 wheel 限速测试 PASS；format/architecture/interfaces/docs 单项门禁 0 新问题。

最终收敛（同日）：`Update` 与适配层进一步线性化——删除 `Active` 与全部单次使用助手，
`CoggingCalibration_Update` 为单函数线性实现；删除 `CoggingFault` 快照与
`CoggingTelemetry` 冻结帧（CAN 0x68-0x6D 保留状态/进度/表读取）；核心 13 → 8 个函数。

分层收敛（同日）：表记录/持久化契约从标定过程中拆出为 `cogging_map.{c,h}`
（表格式、签名、CRC、查表、成表），`cogging_calibration.{c,h}` 只保留过程状态机；
电机控制（保持/监督/会话/保存编排）保持在适配层，参数持久化保持在 `foc_param.c`。

全量 `verify --profile pr`：PASS（9/9），证据
`outputs/runs/20260918T180859247338Z-4fd5b952/summary.json`，`hardware_contacted=false`。

未复用评估：`Stop`/`FocCogging_Abort`/`TryBeginSave` 在项目内没有同契约实现
（摩擦辨识 `StopOutput` 不做停 PWM/切模式/CalibStep；`foc_calibration.c` 的保存序列各自附带
模块专属步骤且上下文不同），按 STYLE 的“语义不一致时保留小规模重复”维持独立。

**剩余待办**：齿槽补偿向速度/位置/阻抗模式的扩展见
[active 计划](../active/2026-09-19-cogging-compensation-torque-modes.md)（需台架授权与逐模式 A/B 证据）。
