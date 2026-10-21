# 代码框架与目录规范

2026-09-17：完成第二阶段目录整理及 MCU USB 移除。APP、Loader、PC 上位机的工程边界独立。

## 当前布局

| 路径 | 职责 |
| --- | --- |
| `firmware/app/` | board_config、bsp_task、foc_task、foc_run 等启动与运行组装 |
| `firmware/motor/foc/` | FOC、PID、采样及观测器 |
| `firmware/motor/position/` | 位置伺服与阻抗控制 |
| `firmware/motor/trajectory/` | 平滑轨迹及梯形轨迹 |
| `firmware/motor/identification/` | 电机、编码器与摩擦辨识/校准 |
| `firmware/motor/protection/` | 故障处理与母线保护配置 |
| `firmware/services/parameters/` | 参数读写、默认配置、轴身份与范围 |
| `firmware/services/telemetry/` | 电机状态快照、快速环诊断接口 |
| `firmware/communication/can/`、`protocol/` | CAN 接入与协议编码 |
| `firmware/platform/api/` | 硬件接口 |
| `firmware/platform/stm32g4/bsp/`、`ports/` | 板级驱动、CAN/时间/外环调度实现 |
| `firmware/platform/stm32g4/cubemx/` | .ioc、.mxproject、Core、Drivers、MDK-ARM |
| `firmware/common/` | 数学工具 |
| `firmware/third_party/segger_rtt/` | 第三方 RTT |
| `tests/unit/`、`integration/`、`hil/` | 单元、离线集成、实机支持与场景配置 |
| `tools/build/`、`flash/`、`bench/`、`analysis/` | 构建、下载、采集操作、离线分析 |
| `docs/architecture/`、`protocols/`、`guides/`、`hardware/`、`reports/` | 架构、协议、使用说明、硬件资料、历史报告 |
| `outputs/build/keil/` | 普通/HIL 编译输出 |

原 `software/`、`hal/`、顶层生成目录已迁入 `firmware/`。`System` 杂项目录取消：
初始化归 app，参数归 parameters，工具归 common，共享电机类型归 motor，HIL 支持归 tests/hil/firmware。
源码与头文件在所属模块内相邻存放。完整文件迁移清单见 [path_migration.json](path_migration.json)。

`outputs/` 为生成内容；原有 `data_recoder/`、`tmp/` 和本机工具缓存保留原位置，避免破坏历史实验引用。
新构建和验证结果进入 outputs，现有数据未清空。

## 工程入口

普通与 HIL 工程位于 `firmware/platform/stm32g4/cubemx/MDK-ARM/`。
CubeMX 的内部 Core/Drivers 相对关系保持一致，用户源码通过相对 include 路径接入。
两个工程保留原文件编译顺序、逐文件优化配置和原有调试选项，删除 USB 相关输入。

```powershell
python tools/run.py --list
python tools/run.py check_project_layout
python tools/run.py build_firmware --target all
python tests/run.py
python tests/run.py --cc 'C:/path/to/zig.exe'
```

统一 CLI 根据名称定位工具，不要求依赖当前工作目录。各目录内脚本也可直接运行。
`tools/project_paths.py` 管理仓库路径、模块搜索位置及原生测试头文件路径。
测试、构建和硬件操作入口分开，离线验证不会烧录或操作电机。

CubeMX 再生成后仍需审查用户代码区、输出目录和自定义源文件组，再执行工程检查与双目标全量编译。
本次校验了生成输入的结构和路径，没有运行 CubeMX GUI 再生成。

## STM32G431 热点执行布局（2026-10-15 增量）

本轮 APP 加载区为 `0x08004000..0x0801bfff`，普通版与 HIL 共用平台
`Vector_Mini_ST.sct`；下文 2026-09 目录迁移的统计与地址说明保留为历史记录。
G431 的 10 KiB CCM 是 32 KiB RAM 的一部分，
原生地址 `0x10000000` 与 `0x20005800` 起始的区域互为别名，不能重复计入容量。
当前在顶部保留 2 KiB：热点执行地址 `0x10002000..0x100027ff`，对应普通地址
`0x20007800..0x20007fff`；RW/ZI 从邮箱之后的 `0x20000020` 开始，必须止于该别名区之前。
scatter 用区间容量及 `ScatterAssert` 拒绝重叠，原 3 KiB 栈和 512 B 堆保持不变。

仅电流环、调制、四个 PI 算子、热采样累计及热更新共八个函数迁入 CCM，不移动 DMA 数据。
函数通过链接节选择，算法源码不加硬件属性；加载内容仍在 APP Flash 内，由启动 `__main`
的 scatter loader 在进入 `main` 前复制。资源统计必须使用 **RW/ZI + CCM 执行代码**，
再单列 32 B 邮箱，不能仅引用 map 的 Total RW。构建后核对复制表、两个地址的回读字节和
APP/BOOT CRC；normal/HIL 共用布局及别名边界由现有 Loader 离线测试覆盖。
过程与验收见 [快环整机复核 v3](../reports/2026-10-15-fast-irq-40us-bench-v3.md)。

2026-10-16 内存优化：mode 5/13 的标定工作区共 6 KiB，其中 4 KiB 为误差和，
另 2 KiB 在全部计数读取完成后转换为候选 LUT；有效编码器 LUT 不复用该区域。
Flash 参数启动读取与停机保存独占一份 2384 B 固定缓冲，重入时拒绝操作，
不再链接自有通用堆。CCM、栈、C 库堆和参数 Flash ABI 保持不变。

## CAN 功能边界（2026-10-16）

`interface_can.c` 负责命令编码/分派、回复槽及保活；`comm_hw.h` 提供收发、
节点过滤和速率配置能力。FDCAN 句柄、HAL、过滤器字段及分频配置全部留在
`comm_status_stm32g4.c`，协议文件不再直接配置外设。回复槽由 RX 中断生产、
前台消费，忙时不能覆盖旧帧；所有权和兼容细节见
[CAN 回复补充](../protocols/can_reply_handoff_v1.md)。

本次保留启动顺序、默认速率和原 100 ms 波特率检查调用。
运行期重配仍含同步 HAL 操作，原 Error_Handler 也没有主动关闭 PWM；
将重配移到前台并明确故障停机策略应作为后续行为变更独立验证。
命令分派仍依赖电机应用状态，本次只完成硬件配置边界收口。

## 参数持久化边界（2026-10-16）

- `app/parameter_persistence.c`：唯一持有参数快照、忙状态与保存结果；协调快环关相确认、
  启动迁移和保存失败后的停机/故障。主循环只调用 `ParameterPersistence_Poll()`。
- `services/parameters/foc_param.c/.h`：保留参数结构、默认值、校验和编解码，拥有记录魔数。
- `platform/api/parameter_storage_hw.h` 与 `bsp/flash.c`：只提供固定参数区原始字节读写及
  参数事务临界区。平台不再调用 `Param_Upload/Download`，也不访问运行模式或发布电机故障。

原 `flash_read_param/flash_write_param` 入口保留在应用模块，避免引入多套保存接口。
旧 BSP `flash.h` 删除，平台地址改引用现有 `boot_contract.h` 单一契约；地址和容量不变。
完整流程、错误结果与单副本边界见 [参数保存说明](../guides/parameter_flash_save_v1.md)。

## 后续 Loader 与 PC 接入

已阅读 D-loader 的总设计、通用 MCU/CAN-FD/UART 框架、内存方案、空间优化与发布规范。
目录边界按其独立构建和共享契约要求预留：

- [loader/](../../loader/README.md)：core、protocol、transport、port、MCU/Board/target 分离。
- [host_app/](../../host_app/README.md)：GUI/CLI 复用 UpgradeService，CAN-FD/UART 后端独立。
- [shared/](../../shared/README.md)：协议、镜像、启动交接、目标布局的单一契约来源。
- APP 未来交接服务归 `firmware/services/boot_handoff/`，无需把 Loader 核心链接进 APP。

当前只是目录与接口归属约定；没有恢复历史 stash，也没有把设计中的升级功能标记为已实现。
2026-09-28 合并主分支 Loader：普通 APP 从 0x08004000 链接，前 32 字节 RAM
保留给启动邮箱；分区契约见 `shared/flash_layout/flash_layout.h`，链接脚本位于
`firmware/platform/stm32g4/cubemx/MDK-ARM/Vector_Mini_ST.sct`。
HIL 仍保留原独立镜像布局，不在本次验证范围。

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

## 下一阶段代码优化

2026-10-16 运行态清理收口：`foc_task.c` 的私有 `task_reset_runtime` 协调标定取消、
摩擦辨识退出、外环/电流 PI 和无感启动状态复位；只有快环 STOP 边沿及停机装配档切换调用。
清理完成后仍由原快环位置关相。该函数不写 Flash、不清热状态，也不删除已提交的 LUT/零位。
`foc_errhandle` 不再向应用公开整机清理函数；故障检测、清错和模式准入职责保持原有实现。
本步没有解开 `common_inc.h` 或全部间接头文件依赖，见
[运行态清理计划](../plans/2026-10-16-motor-runtime-reset-v1.md)。

文件归属已经明确，但目录本身不会消除耦合。`app/common_inc.h` 仍是历史聚合头，
`motor/data_type.h` 仍依赖 MCU main.h。后续优先拆小接口与纯类型，再提取公共命令服务，
最后细化 foc_run 中的模式执行与异步外环上下文。每步独立验证实时性和状态一致性。

新增 Loader/PC 业务时遵守既定边界；不要重新建立 System 杂项目录，也不要让主机 UI 直接处理 Flash 升级状态机。
