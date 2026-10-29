# Keil 双板统一工作区

仓库根目录的 [YG-ESC.uvmpw](../../YG-ESC.uvmpw) 是双板 APP 的统一管理入口。
双击此文件，或在 Keil 的 Project → Open Project 中选择它，即可同时查看三个子工程。

| 子工程 | 用途 |
| --- | --- |
| ESC-APP-STM32G431 | STM32G4 旧板 APP |
| ESC-APP-AT32F456 | AT32F456 新板完整 APP，永久关相；工作区默认选中 |
| ESC-APP-AT32F456-Commissioning | AT32F456 新板完整 APP，限时调试 |

## 使用

在左侧工程根节点上右键，选择 `Set as Active Project` 切换当前子工程。
普通 Build/Rebuild 和调试操作针对当前工程，执行前确认工程名及目标。
通过 Project → Manage → Multi-Project Workspace 可以增减、排序子工程。
以上菜单语义见 [Keil 官方说明](https://www.keil.com/support/man/docs/uv4cl/uv4cl_ca_managemultiprojectfile.htm)。

工作区初始不勾选批量构建；需要时在 Project → Batch Build 中明确选择目标。
这个入口只引用已有 `.uvprojx`，不合并输出镜像，不复制公共代码，也不改变子工程的
芯片、参数、编译器、Flash 分区、下载或调试配置。路径全部相对于仓库，可随仓库移动。

两个新板目标与旧板保持相同七组结构和公共编译选项，详见
[工程对齐记录](../plans/active/2026-10-24-keil-project-alignment-v1.md)。
统一入口的新板关相APP已通过启动、ADC、MT6835和CAN只读实测，快环峰值40.42us，
2kHz RTT连续采集零丢帧；见[关相验收记录](../plans/active/2026-10-24-armcc-off-hil-v1.md)。
静态验收不代替限时版转动和闭环验证。

上述实机记录针对当时的共享入口镜像。当前两板各自的application/main.c为真实入口，
平台实现移至platform_app.c；本次[入口拆分](../plans/active/2026-10-24-board-application-entry-v1.md)
只做构建和离线验证，不沿用旧镜像的实机通过结论。

四个完整APP的目标与输出名均与工程文件名一致，构建产物统一位于
`outputs/build/keil/<工程名>/`。旧板Loader配套APP为`ESC-APP-STM32G431-LoaderApp`，
独立构建，不加入上述三工程工作区。命令行仍沿用`normal`、`r40-app-off`、
`r40-app-commissioning`选择器；历史实测报告保留当时文件名和哈希。
