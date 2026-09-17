# PC 上位机工程入口

PC 上位机与 APP、Loader 独立开发和发布。本次整理确定以下职责边界；
目录中的历史打包产物不作为新升级流程已实现的依据。

```text
host_app/
  src/mdrive_host/
    ui/                       GUI
    cli/                      命令行入口
    services/                 设备操作与唯一 UpgradeService
    protocol/                 APP/Loader 编解码
    transport/                CAN-FD、UART 后端
    image/                    包清单、目标匹配、完整性检查
  tests/                      假链路、协议向量、升级流程测试
```

GUI、CLI 和自动测试共用服务层；传输后端负责收发，UpgradeService 负责设备 UID 绑定、
镜像预检、会话、重试与最终 APP 身份核对。CAN-FD 与 UART 共用升级状态机。
使用 USB-CAN 分析仪的主机驱动保留，设备 USB CDC 服务已经退出当前 APP。

跨工程契约来源为 [shared/](../shared/README.md)，布局和兼容性依据目标清单，
不能仅通过文件名或大小判断镜像是否可升级。详细约束见
[D-loader 通用框架](../docs/architecture/loader/d_loader_portable_framework_20260917.md)。

当前 `tools/bench/` 是现场工具集合，`tools/analysis/` 是离线分析工具；
后续提取稳定协议代码进入主机包时，应让这些入口调用包接口，避免复制维护多套协议。
虚拟环境、构建缓存和历史可执行产物已加入忽略规则。新产物建议归 `outputs/build/host_app/`。
