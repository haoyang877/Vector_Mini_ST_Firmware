# Architecture Map

当前软件结构见[电机驱动软件架构](docs/architecture/software_architecture.md)，
文件归属见[代码框架](docs/architecture/code_structure.md)。本页维护稳定边界，不复制历史阶段说明。

## 产品与共享契约

- `firmware/`：STM32G4 / AT32F456 共用的单轴电机 APP。
- `loader/`：独立的 G431 升级与启动产品，不链接进 APP。
- `host_app/`：独立主机应用，包含 Rust GUI 候选与 Qt6 CAN FD 工具。
- `shared/`：跨产品协议、CRC32、镜像、启动邮箱、生产身份和目标布局契约。
- `projects/`：板级 main、Keil 工程、启动汇编、scatter 和 CubeMX 输入。
- `tests/`、`tools/`：离线验证和显式台架工具；`outputs/` 保存生成证据。

## 固件依赖边界

APP 组装硬件、算法、参数和协议。MotorAxis 持有单轴对象；MotorLifecycle 负责状态和动作；
MotorStateMachine 只提供五态纯迁移。通信使用显式绑定的服务，不访问 APP 运行实现或 Watch 对象。
Motor 和参数代码不依赖 APP 实现；shared 不依赖固件或硬件。

```text
projects/main → app/composition → runtime → motor algorithms → BSP API
                     ↓             ↓
                 communication   parameters → configuration/value contracts
                     ↓
                 shared protocol

BSP API ← corresponding MCU implementation ← vendor SDK/generated code
```

硬件只通过 `firmware/platform/bsp/include` 暴露能力与单位，不向上层暴露 HAL 类型、寄存器、
外设句柄或板卡引脚。硬件实现全部位于对应 MCU；BSP 根目录仅保留内部声明头。
板型、测量尺度和电机默认值集中在 `firmware/config`，不保留空的 devices 或转发层。
详细规则见[BSP 职责约定](docs/architecture/bsp_design_rules.md)。

## 运行时边界

- 两板 main 依次调用 Bsp_Init、App_Init、Bsp_Start、App_Start；主循环调用 App_Process。
- ADC 快中断执行 20 kHz 采样、反馈、控制或单项标定；监督中断以 2 kHz 更新外环和生命周期。
- 前台处理协议、关相后的 LUT 发布和 Loader 交接；Flash 操作不进入快慢环。
- 活动配置、运行状态和存储记录分别建模，内部枚举显式映射线上编号。
- 固定存储替代固件堆分配；标定、外环和反馈仍是单轴实现。
- STOP 请求受理不代表已关相；故障和过期会话结果不能恢复旧输出。
- 当前 APP 不接入普通目标、在线参数读写/保存、标定进度服务或绕组热调度。

当前配置和未完成条件见[配置说明](firmware/config/README.md)；
具体线上行为见[核心 v1.8](docs/protocols/README.md#implementation)。

## 验证边界

`tools/run.py` 是统一命令入口。PR 验证检查全部工程路径、格式、lint、离线测试、分层、接口和文档。
`doctor`、quick、PR 不接触硬件；release 构建也不下载固件。
架构检查没有违规不代表工程可编译，编译通过也不代表时序、电气或电机行为已经验收。
源码、协议和参数 ABI 改动须保留适当的比较证据；旧 outputs 不能作为当前 PASS。

本轮残留清理及基线见[清理计划](docs/plans/active/2026-10-30-firmware-cleanup-v1.md)。
