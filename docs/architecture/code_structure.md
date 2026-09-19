# 代码框架与目录规范

2026-09-17：完成第二阶段目录整理及 MCU USB 移除。APP、Loader、PC 上位机的工程边界独立。

## 当前布局

| 路径 | 职责 |
| --- | --- |
| `firmware/app/` | board_config、bsp_task、foc_task、foc_run、motor_state、foc_mode_dispatch、foc_run_state、rtt_telemetry 等启动与运行组装 |
| `firmware/motor/foc/` | FOC、PID、采样及观测器 |
| `firmware/motor/position/` | 位置伺服与阻抗控制 |
| `firmware/motor/trajectory/` | 平滑轨迹及梯形轨迹 |
| `firmware/motor/identification/` | 电机、编码器与摩擦辨识/校准 |
| `firmware/motor/protection/` | 故障处理与母线保护配置 |
| `firmware/services/parameters/` | 参数读写、默认配置、轴身份与范围 |
| `firmware/services/telemetry/` | 电机状态快照、快速环诊断接口 |
| `firmware/communication/can/`、`protocol/` | CAN 接入与协议编码 |
| `firmware/platform/api/` | 硬件接口 |
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

`outputs/` 为生成内容；原有 `data_recoder/`、`tmp/` 和本机工具缓存保留原位置，避免破坏历史实验引用。
新构建和验证结果进入 outputs，现有数据未清空。

## 工程入口

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
- APP 未来交接服务归 `firmware/services/boot_handoff/`，无需把 Loader 核心链接进 APP。

当前只是目录与接口归属约定；没有恢复历史 stash，也没有把设计中的升级功能标记为已实现。
当前 APP 仍从 0x08000000 链接。后续接入 Loader 时再处理 0x08004000 APP 入口、
保留 RAM 邮箱、参数区保护和版本/兼容性契约。

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

文件归属已经明确，但目录本身不会消除耦合。`app/common_inc.h` 聚合头已于 2026-09-19 删除：
app/cubemx 消费者全部改为最小显式 include，`foc_param.c` 经 `platform/api` 与
`param_comm_bridge.h` 窄桥访问通信运行态；`motor/data_type.h` 已改用 `<stdint.h>`。后续优先
拆小接口与纯类型，再提取公共命令服务，最后细化 foc_run 中的模式执行与异步外环上下文。
每步独立验证实时性和状态一致性。

新增 Loader/PC 业务时遵守既定边界；不要重新建立 System 杂项目录，也不要让主机 UI 直接处理 Flash 升级状态机。
