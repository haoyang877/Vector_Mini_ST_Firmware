# Keil 双板统一工作区

仓库根目录的 [YG-ESC.uvmpw](../../YG-ESC.uvmpw) 是双板 APP 的统一管理入口。
打开后包含两个与 Loader 配套的 APP 子工程。

| 子工程 | 用途 |
| --- | --- |
| ESC-APP-STM32G431-LoaderApp | STM32G4 配套 Loader 的 APP |
| ESC-APP-AT32F456 | AT32F456 配套 Loader 的 APP，从 0x08004000 启动，工作区默认选中 |

在工程根节点选择 `Set as Active Project` 切换目标。Build/Rebuild 和调试操作针对当前工程。
各工程分别使用本板的 `application/main.c` 与 BSP 实现；两板共用应用、控制和通信源码。
当前 BSP 系统实现位于对应 MCU 目录的 `system.c`。

独立启动版本位于 [YG_ESC.uvmpw](../../YG_ESC.uvmpw)。其中 AT32 工程名为 `ESC-APP-AT32F456-Standalone`，从 0x08000000 启动。
命令行使用 `at32-loader-app` 构建 AT32 配套 APP，使用 `at32f456-app-off` 构建独立 APP。两个工程的产物分别位于 `outputs/build/keil/<工程名>/`。

2026-11-03 按用户要求删除 `ESC-APP-AT32F456-Commissioning` 工程及其构建选择器、
专用 scatter 文件和 [MotorBench 功能](unified_motor_bench.md)。已打开的 Keil 窗口需重新加载工作区
才能刷新工程树。历史实测哈希对应当时的镜像。

各工程已显式设置 BSP_BOARD：STM32 为 1U，AT32 为 2U，无需手动切换 `config.h`。
独立 BSP 测试工程仍在 `projects/at32f456/bench/`，不加入正式 APP 工作区。
