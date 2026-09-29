# 电机驱动 APP 源码导航

| 目录 | 在这里查什么 |
| --- | --- |
| [app](app/README.md) | 启动组装、单轴运行所有者、主状态机 |
| [motor](motor/README.md) | 电机算法、反馈、轨迹、标定、辨识与保护 |
| services | 参数、只读状态、控制/维护值请求 |
| communication | CAN 接入与 APP 业务协议 |
| platform | 硬件能力接口，以及 STM32G4 实现和生成工程 |
| third_party | 第三方源码，保持原有布局 |

APP/Loader 共同契约只在仓库 shared 目录实现。
完整文件归属、依赖方向和工程入口见[目录职责](../docs/architecture/code_structure.md)。
主工程为 platform/stm32g4/cubemx/MDK-ARM/YG_ESC_APP.uvprojx。
Keil 仅显示源码，实际目录中的头文件与实现相邻。
