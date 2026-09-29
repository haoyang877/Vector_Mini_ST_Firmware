# 代码框架与目录规范

2026-09-17：完成第二阶段目录整理及 MCU USB 移除。APP、Loader、PC 上位机的工程边界独立。

## 当前布局

| 路径 | 职责 |
| --- | --- |
| `firmware/app/` | board_config、bsp_task、foc_task、motor_execution、motor_axis、motor_maintenance、motor_state_machine、rtt_telemetry 等启动与运行组装 |
| `firmware/motor/foc/` | foc、PID、采样及观测器 |
| `firmware/motor/position/` | 位置伺服与阻抗控制 |
| `firmware/motor/trajectory/` | 平滑轨迹及梯形轨迹 |
| `firmware/motor/identification/` | 电机、编码器与摩擦辨识/校准 |
| `firmware/motor/protection/` | 故障处理与母线保护配置 |
| `firmware/motor/axis_config.h` | 轴身份、控制基线及运行范围的值契约 |
| `firmware/services/parameters/` | 参数读写、默认配置、轴记录 CRC 与身份解析 |
| `firmware/services/telemetry/` | 电机状态快照 |
| `firmware/communication/can/`、`protocol/` | CAN 接入与协议编码 |
| `firmware/platform/api/` | 硬件能力接口，包括可选快环计时钩子 |
| `firmware/platform/stm32g4/bsp/`、`ports/` | 板级驱动、板级启动/指示器、CAN/时间/外环调度实现 |
| `firmware/platform/stm32g4/cubemx/` | .ioc、.mxproject、Core、Drivers、MDK-ARM |
| `firmware/common/` | 数学工具、内存池 |
| `firmware/third_party/segger_rtt/` | 第三方 RTT |
| `tests/unit/`、`integration/` | 单元、离线集成 |
| `tools/build/`、`flash/`、`bench/`、`analysis/` | 构建、下载、采集操作、离线分析 |
| `docs/architecture/`、`protocols/`、`guides/`、`hardware/`、`reports/` | 架构、协议、使用说明、硬件资料、历史报告 |
| `outputs/build/keil/` | Keil 主工程编译输出 |

原 `software/`、`hal/`、顶层生成目录已迁入 `firmware/`。`System` 杂项目录取消：
初始化归 app，参数归 parameters，工具归 common，共享电机类型归 motor（原 HIL 支持目录已随
2026-09-19 HIL 退役删除）。
源码与头文件在所属模块内相邻存放。完整文件迁移清单见 [path_migration.json](path_migration.json)。

APP 当前模块、对象所有权、函数与物理量名称以
[APP 命名规范 v1](app_naming_v1.md) 为准；`motor_axis` 是单轴组装所有者，
`motor_state_machine` 是主生命周期，辨识功能适配位于 `*_task` 模块。

`outputs/` 为生成内容；原有 `data_recoder/`、`tmp/` 和本机工具缓存保留原位置，避免破坏历史实验引用。
新构建和验证结果进入 outputs，现有数据未清空。

## 工程入口

2026-09-29 起 Keil 按 APP、Motor、Services、Communication、Platform、Shared、Common
与 Vendor 合并为八组；普通和 Loader 配套 APP 一致，不在工程树登记 `.h`。
头文件仍位于原目录并通过既有 include 路径使用。实际目录与编译、调试设置保留，规则见
[Keil APP 精简分组 v2](../plans/completed/2026-09-29-keil-app-groups-v2.md)。

Keil 主工程位于 `firmware/platform/stm32g4/cubemx/MDK-ARM/`。
CubeMX 的内部 Core/Drivers 相对关系保持一致，用户源码通过相对 include 路径接入。
工程保留原文件编译顺序、逐文件优化配置和原有调试选项，删除 USB 相关输入。

```powershell
python tools/run.py --list
python tools/run.py check_project_layout
python tools/run.py build_firmware --target all
python tests/run.py
uv run python tools/run.py verify --profile pr
```

统一 CLI 根据名称定位工具，不要求依赖当前工作目录。各目录内脚本也可直接运行。
`tools/project_paths.py` 管理仓库路径、模块搜索位置及原生测试头文件路径。
测试、构建和硬件操作入口分开，离线验证不会烧录或操作电机。

CubeMX 再生成后仍需审查用户代码区、输出目录和自定义源文件组，再执行工程检查与全量编译。
本次校验了生成输入的结构和路径，没有运行 CubeMX GUI 再生成。

## 后续 Loader 与 PC 接入

已阅读 D-loader 的总设计、通用 MCU/CAN-FD/UART 框架、内存方案、空间优化与发布规范。
目录边界按其独立构建和共享契约要求预留：

- [loader/](../../loader/README.md)：core、protocol、transport、port、MCU/Board/target 分离。
- [host_app/](../../host_app/README.md)：GUI/CLI 复用 UpgradeService，CAN-FD/UART 后端独立。
- [shared/](../../shared/README.md)：协议、镜像、启动交接、目标布局的单一契约来源。
- APP 进 Loader 服务归 `firmware/app/board_config.c`（唯一生命周期 owner 接点），无需把 Loader 核心链接进 APP。

2026-09-28 最简方案已实施：APP 从 0x08005000 链接，Loader 跳转前现场校验内嵌描述符与整包 CRC32；
保留 RAM 顶 32B 进入邮箱与参数区保护，不保留持久启动记录或 boot epoch。详见
[最简启动/升级方案 v1](../plans/completed/2026-09-28-simplified-boot-handoff.md)。

## USB 移除

详见 [USB 移除说明](../guides/usb_removal.md)。固件 USB 命令、打印、Device 中间件、
PCD/LL USB、专用缓冲、中断实现、HSI48 使能及生成配置已清理。
PC 的 USB-CAN 驱动保留；CAN、RTT、校准与辨识核心保留。
USB 独有的 LUT/原始摩擦采样导出目前缺少 CAN 替代 API，已明确记录，未宣称功能完全等价。

## 验证结果

1. 先完成纯目录迁移：普通/HIL 全量编译均 0 错误、0 警告，HEX 分别与原基线逐字节一致。
2. 再移除 USB：普通/HIL 全量编译均 0 错误、0 警告；实际 ROM 分别为 83224/84976 B，
   静态 RAM 分别为 25488/25752 B。当前 Flash/RAM 链接区域与参数 ABI 未更改。
3. Python 离线测试：75 项，74 项通过；原有示波器单位字段测试失败保留。
   新增的 5 项检查覆盖工程 USB 输入、启动/调度、IRQ 槽位、CubeMX 和 HAL 配置。
4. 11 组原生 C 验证通过，覆盖位置伺服、轨迹、启动、CAN、外环、限速、母线保护、ADC、
   编码器、快速数学、配置缓存。等价性比较以本轮迁移前源码为参照，两个入口各 790068 tick 私有状态与输出一致。
5. 历史默认 Git 3980f92 等价性测试在上一轮已确认存在失败；未替换其历史基线以掩盖结果。
6. 用户原有示波器布局和单位修改保留，文件随目录迁移；提交前复查修正两份配置中的 AXF 路径，
   指向 `outputs/build/keil/` 下对应普通/HIL 镜像。目录整理阶段未烧录；随后经用户授权完成普通 APP
   烧录、参数保留、CAN 查询与状态流回归，见[实机报告](../reports/2026-09/usb_removal_hardware_20260917.md)。
   未进行实机 USB 拔插和闭环运动测试。

证据目录：`outputs/layout_v2_20260917/`；原输入快照为 before，目录迁移的含 USB 镜像为
relocated_with_usb，最终离线日志为 final_tests。构建 map 位于 outputs/build/keil。

## 内部所有权（2026-09-28 E3）

目录继续使用本页当前布局；这一轮不迁移产品名称，也不新增双编码器、Loader 或 PC 功能。
代码边界已进一步落实到实际调用：

| 所有者 | 状态与调用边界 |
| --- | --- |
| `app/motor_axis.c` | 私有 motor_state、foc、速度 PI、encoder、Observer、startup；按获准生命周期选择维护、控制或空闲预载，执行快慢调度与前台维护，生成值快照 |
| `app/motor_state_machine.c` | 唯一请求、正常启相关断和任务衔接所有者；Watch/CAN 共用命令邮箱，串行生命周期事件，保存认领与取消身份 |
| `app/motor_maintenance.c` | 按维护 kind 调用实际功能和取消；返回功能事实及准入原因，不使用控制模式反推任务 |
| `app/motor_execution.c` | 有界电流/速度/位置/vq_v 控制及外环；只使用显式传入对象，不反查状态机或调度维护 |
| `motor/protection/foc_errhandle.c` | 显式故障锁存和恢复资格；不拥有输出使能或清错提交权 |
| `motor/foc/foc_sensing.c` | 显式采样目标，温度诊断私有，提供窄新鲜度查询 |
| `motor/identification/foc_cogging_calibration.c` | 私有标定采样、表和补偿状态；候选构建与发布分开，取消不改写前台正在读取的样本 |
| `services/parameters/foc_param.c` | 默认、schema 11 记录解释与导出，实际控制对象由 APP 显式传入 |
| `platform/api/param_store.h` | 仅字节读写能力；STM32 Flash 实现不解释参数、不分配堆、不引用控制对象 |

`foc_task.c` 是 ISR 入口薄层；状态/RTT 输出使用值快照或显式标量，不返回内部对象地址。
生成 main 只调用前台服务；合法 schema11 装载不重写，旧 schema 拒绝并回退默认值，不自动写 Flash。
本轮没有新增持久化 ABI、CAN 编号或大型常驻参数副本。

维护完成与保存提交独立记录，前台使用 `{epoch, kind}` 认领；保存前再次核对身份、
无故障和关相。取消等待编码器堆工作区、齿槽借用、摩擦和相电阻会话实际释放。
内部 FAULT/FATAL 映射为非零既有故障，FATAL 在线状态优先，CLEAR 后需重新发起命令。
过流恢复计数保留原实际 2 kHz 的 2000 拍（1 s），未按旧注释改成 100 ms。

2026-09-29 收回齿槽四个公开可写对象：补偿通过值请求受理，台架租约原子设置，
保护与同拍遥测通过独立值快照读取；旧SWD符号与写字段入口删除，不提供兼容。
模块私有静态状态仍由其所属模块拥有，不将私有状态改成可跨层写入的大上下文。
具体接口、验收和回退见 [E3 计划](../plans/completed/2026-09-28-internal-decoupling-e3.md)。
本次状态收口见[齿槽状态所有权计划](../plans/completed/2026-09-29-cogging-state-owner-v1.md)。
原 `FocBenchSpeed` 台架邮箱已退役，见 [退役记录](../guides/watch_motor_commands.md)。

2026-09-29 框架边界收尾：`motor/motor_types.h` 只依赖电机域轴值类型，
不再引用参数服务。轴记录服务保留 CRC、记录构造与启动身份解析。
摩擦及相电阻辨识配置由 `app/motor_maintenance.c` 装配为私有只读常量，
每次任务调用显式传入；算法启动时复制核心配置，不持有服务对象或配置全局指针。
快环计时接口归 `platform/api/fast_loop_profile.h`，板级实现仍拥有时钟和统计，
关闭计时时仍为空操作。五项已登记反向依赖全部删除，具体证据见
[框架收尾计划](../plans/completed/2026-09-29-framework-closure-v1.md)。

## 历史 E1 与后续优化

2026-09-28 E1：运行所有者删除 `power_on` 副本，关相条件直接使用平台数字使能回读。
ADC2 JEOS 快环末尾登记感测处理进展，监督拍统一读取有限电流/电压值和温度有效期；
基础自检不要求编码器标定，模式启动继续检查其反馈和标定依赖。维护 worker 等待会话
受理后执行；恢复默认参数不再直接改模式，完成由结果协议请求保存。
CLEAR 保留原故障直到核心受理，故障不会因兼容值已经为零自动清除。
数字回读不代表物理栅极验证；该历史批次尚未接入完整资源释放与结果代际，现已由 E3 补齐。
来源、删减和验证见
[E1 批次说明](../plans/completed/2026-09-28-lifecycle-guards-e1.md)。

2026-09-28 E4：参数写入核对停机资格和活动revision，成功不自动保存；SAVE按原始身份
等待Flash回读与实际提交。160～162按明确种类受理/查询/取消，不把内部枚举直接投到线路。
同来源序号重放原ticket，不重复执行；标定终态不被派生SAVE覆盖；取消等待实际归还。
共享帧/CRC/分片代码现归`shared/protocol/`，APP和Loader使用同一源码，不维护兼容副本。
本批提交`d505acdc`，未测试、未构建；升级服务与D端口后续接线进度见[E4计划](../plans/completed/2026-09-28-app-boundaries-e4.md)。

2026-09-28 最简方案：56进Loader服务内联在`app/board_config.c`的composition层，通过唯一运行所有者
复核停机、关相与精确TX完成后再写ENTER邮箱复位；无持久确认。平台BootHw只提供识别Loader启动、
设置VTOR、ENTER复位与继承看门狗能力，普通独立APP链接不可用端口，两种实现互斥。
Type3启动值投影归`app/yg_protocol_status_adapter.c`，与协议Service串行；去启动记录后恒报
APP角色且无epoch（不伪造身份），线格式不变。ENTER事务即使失败也持续禁止输出，直至明确重新启动。

文件归属已经明确，但目录本身不会消除耦合。`app/common_inc.h` 聚合头已于 2026-09-19 删除：
app/cubemx 消费者全部改为最小显式 include，`foc_param.c` 经 `platform/api` 与
`param_comm_bridge.h` 窄桥访问通信运行态；`motor/motor_types.h` 已改用 `<stdint.h>`。后续优先
拆小接口与纯类型，再提取公共命令服务，控制执行与异步外环上下文现归 motor_execution。
每步独立验证实时性和状态一致性。

新增 Loader/PC 业务时遵守既定边界；不要重新建立 System 杂项目录，也不要让主机 UI 直接处理 Flash 升级状态机。

当前主状态机完整契约见[功能任务驱动的主状态机 v1](motor_state_machine_v1.md)。

2026-09-29：快环任务选择归运行组装，旧 `MotorExecution_FastStep` 删除，
控制入口为 `MotorExecution_ControlStep`；维护仍直接调用 `MotorMaintenance_Step`。
故障优先、执行前身份捕获、空闲零矢量预载与结果入箱位置保持，见
[边界收口计划](../plans/completed/2026-09-29-execution-boundary-v1.md)。

## 2026-09-29 重复实现收敛

生产位置/速度控制只由 MotorOuterLoop_SlowTick 执行，20 kHz 入口消费其电流参考。
旧 Task_Position_Mode、Task_Speed_Mode 已删除；外环复位请求为
MotorOuterLoop_RequestReset。摩擦辨识仍使用有实际调用的 SpeedMode_Run。

位置核心只公开 UpdateControl、UpdateTargetControl 和独立遥测接口；
场景夹具中的丰富视图由公开结果组合，不进入固件。缓存对照测试验证真实 2 kHz 外环。

shared/checksum/crc32.c/.h 是 APP/Loader 的唯一标准 CRC32 核心：
持久记录、邮箱和镜像共用，镜像事务继续保存原始续算状态并保留读取/喂狗/字段置零。
shared/protocol/yg_protocol_byte_order.h 与 yg_protocol_request.h 分别拥有
小端固定宽度读写和要求确认的单播请求规则；长度、业务权限和安全校验仍由原边界负责。

标量数学统一头文件经 ARMCC 构建出现多份非内联实例，未带来容量收益，已回退此项。
公共数学仍归 common/utils；不同 NaN/负零或高速内联语义的局部小函数保持文件私有。
FOC 电流与 Vq 策略在同一文件共享采样及调制输出，保持两个控制策略与诊断钩子顺序。
详细验收见[去重计划](../plans/completed/2026-09-29-duplicate-removal-v1.md)。
