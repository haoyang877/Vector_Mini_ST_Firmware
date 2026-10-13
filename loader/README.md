# Loader 独立工程入口

本次 `codex/ovp-36v` 接入依据为
[CAN Loader 完整设计方案 v1](../docs/plans/2026-10-13-can-loader-minimal-design-v1.md)：
12 KiB Loader、单 APP、1M/1M CAN-FD、参数区保留及中断后完整重下载。
已接入最小实现、独立APP/Loader构建和离线测试；在线连续1000次验收尚未完成，不能视为已发布。

此目录用于后续 `YG2026-ESC-LOADER` 实现。设计依据为
[D-loader 通用框架](../docs/architecture/loader/d_loader_portable_framework_20260917.md)。
当前目录中的历史本机构建产物不代表该设计已经实现或通过验收。

建议源码结构：

```text
loader/
  core/                       启动、升级会话、镜像、记录、存储策略
  protocol/                   公共命令、编解码、CRC
  transport/canfd/             CAN-FD 消息适配
  transport/uart/              UART 流式组帧
  port/include/               Flash、启动、时间、链路、系统接口
  ports/stm32g4/               MCU 实现
  boards/vector_mini_st/       板级时钟、引脚和功率安全态
  targets/vector_mini_st_g431/  配置、启动文件、链接脚本
  tests/native/               模拟 Flash、时钟、链路
  tests/integration/          幂等、掉电恢复、多链路互斥
```

Loader 与 `firmware/` 中的电机 APP 分别编译、分别链接。Loader 核心不依赖
APP 的 `common_inc.h`、电机全局状态、控制循环或动态内存。
未来跨工程的协议版本、布局标识、镜像描述和启动交接契约归 [shared/](../shared/README.md)。
APP 的升级交接服务归 `firmware/services/boot_handoff/`，需在停机并确认功率输出关闭后再复位。

首个目标按设计规划 CAN-FD，UART 为后续链路。当前 APP 已移除 MCU USB，
但仍以 `0x08000000` 为入口，没有实现 `0x08004000` APP 重定位、邮箱或升级命令。
接入 Loader 时必须生成新的链接布局并验证向量表、RAM 保留区、参数保护与完整升级流程。

构建产物统一放 `outputs/build/loader/`，发布按
[固件发布规范](../docs/guides/firmware_release_standard.md)执行。

当前源码入口：`firmware/`；独立Keil工程：`mdk/Vector_Mini_ST_Loader.uvprojx`。
主机工具：`tools/loader_updater.py`和`tools/loader_cycle_test.py`。
离线验证：`python tests/loader/run_tests.py --cc <native compiler>`。
