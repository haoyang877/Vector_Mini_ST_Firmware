# APP / Loader / PC 共享契约边界

这是后续集成的契约入口，目前只有目录约定，没有可用的升级 ABI 实现。

- `protocol/`：版本、操作码、错误码、端序、CRC 定义及黄金向量。
- `image/`：镜像清单、兼容标识、参数 ABI、提交记录格式。
- `boot/`：APP 请求升级的交接格式，包含字段语义和版本。
- `targets/<target>/`：目标存储几何与布局的单一来源，生成 APP/Loader 链接输入和主机元数据。

共享内容应为稳定数据契约及生成输入，不包含 STM32 HAL、设备句柄、GUI 依赖、
APP 的电机类型或 Loader 的运行状态。物理地址与跳转方式属于目标配置，
公共升级状态机不固定 G431 地址或 Cortex-M 启动规则。

首个目标设计和发布约束见
[D-loader 架构](../docs/architecture/loader/d_loader_architecture_memory_20260917.md)、
[通用框架](../docs/architecture/loader/d_loader_portable_framework_20260917.md)、
[固件发布规范](../docs/guides/firmware_release_standard.md)。
设计地址不自动适用于当前独立 APP 镜像。
