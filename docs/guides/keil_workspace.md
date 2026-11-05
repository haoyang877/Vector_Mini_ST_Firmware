# Keil 双板统一工作区

仓库根目录的 [YG-ESC.uvmpw](../../YG-ESC.uvmpw) 是双板 APP 的统一管理入口。
打开后包含两个子工程；`YG_ESC.uvmpw` 也已同步移除退役工程。

| 子工程 | 用途 |
| --- | --- |
| ESC-APP-STM32G431 | STM32G4 旧板 APP |
| ESC-APP-AT32F456 | AT32F456 新板 APP，工作区默认选中 |

在工程根节点选择 `Set as Active Project` 切换目标。Build/Rebuild 和调试操作针对当前工程。
各工程分别使用本板的 `application/main.c` 与 BSP 实现；两板共用应用、控制和通信源码。
当前 BSP 系统实现位于对应 MCU 目录的 `system.c`。

正式构建另有独立的 `ESC-APP-STM32G431-LoaderApp`，不加入上述工作区。
三个完整 APP 的目标名和输出名均与工程名一致，产物位于 `outputs/build/keil/<工程名>/`。
命令行使用 `normal` 构建 STM32 APP，使用 `at32f456-app-off` 构建 AT32F456 APP。例如在已配置 Keil 的环境中运行 `uv run python tools/run.py build_firmware --target at32f456-app-off`。

2026-11-03 按用户要求删除 `ESC-APP-AT32F456-Commissioning` 工程及其构建选择器、
专用 scatter 文件和 [MotorBench 功能](unified_motor_bench.md)。已打开的 Keil 窗口需重新加载工作区
才能刷新工程树。历史实测哈希对应当时的镜像。

各工程已显式设置 BSP_BOARD：STM32 为 1U，AT32 为 2U，无需手动切换 `config.h`。
独立 BSP 测试工程仍在 `projects/at32f456/bench/`，不加入正式 APP 工作区。
