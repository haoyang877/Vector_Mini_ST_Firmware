# 电机驱动 APP 源码导航

| 目录 | 在这里查什么 |
| --- | --- |
| [app](app/README.md) | 启动组装、单轴运行所有者、主状态机 |
| [motor](motor/README.md) | 控制执行、FOC/外环、反馈、单项标定与保护 |
| [parameters](parameters/README.md) | 配置默认值、标量读写和参数记录转换 |
| communication | CAN 接入与 APP 业务协议 |
| platform | 硬件能力接口，以及 STM32G4 实现和生成工程 |
| third_party | 第三方源码，保持原有布局 |

APP/Loader 共同契约只在仓库 shared 目录实现。
完整文件归属、依赖方向和工程入口见[目录职责](../docs/architecture/code_structure.md)。
主工程为 ../projects/stm32g4/MDK-ARM/ESC-APP-STM32G431.uvprojx。
Keil 仅显示源码，实际目录中的头文件与实现相邻。

## Keil Watch 调试观测

主工程 `ESC-APP-STM32G431` 的观测对象如下。按用户要求，MotorAxis 运行对象和主状态机上下文
提供非静态全局符号，名称保持不变以兼容已有 Watch。运行写入仍由各所属模块负责，
观测符号不作为绕过命令受理的操作接口。

| 对象 | 所属源码 | 观测内容 |
| --- | --- | --- |
| `s_motor_configuration` | `app/runtime/motor_axis.c` | model/application/device配置、配置来源及确认状态，含设备LUT/零位 |
| `s_motor_state` | `app/runtime/motor_axis.c` | 模式、故障、速度/电流/位置目标及动态限幅，不包含配置 |
| `s_foc_state` | `app/runtime/motor_axis.c` | 母线电压/电流、三相电流、d/q 电流、温度 |
| `s_rotor_encoder` | `app/runtime/motor_axis.c` | 机械位置/速度、采样有效性及读帧错误，不持有LUT/零位 |
| `s_context` | `app/motor_state_machine/motor_state_machine.c` | 主状态、自检、维护结果、后台工作及控制执行进度 |

上电保持三相关闭，MotorAxis调度foc_sensing累计1秒新鲜ADC帧校正三相电流偏置；完成前自检不就绪。
电流与速度PI实例在启动时配置已加载的活动参数，校正不自动保存或启相。
源码修复不等于实机通过，见[初始化修复计划](../docs/archive/plans/2026-09-29-startup-current-offset-pi-fix-v1.md)。

具体 Watch 表达式、重构前后参数名称差异和安全操作步骤见
[Keil Watch 只读观测指南](../docs/guides/watch_motor_commands.md)。
进入调试前确认 AXF 与板上镜像匹配；电机出力时不使用断点、单步或 CPU 暂停来停车。

## 两板入口与平台装配

两板各在`platform/<MCU>/application/main.c`定义真实main，模板相同；
相邻的`platform_app.c`实现平台初始化和维护，入口约定声明于`app/composition/app_platform.h`。
共用`app_startup.c`仅保留App_Init/App_Process及业务组装；BSP接口与控制中间层不变。
