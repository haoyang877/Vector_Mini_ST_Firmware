# Keil APP 调试入口方案 v1

日期：2026-10-15。分支：`codex/ovp-36v`。

状态：历史方案，已由 [v2 无脚本启动](keil_app_debug_v2.md) 替代。
以下保留实施历史，不再作为当前配置步骤。

目标：普通 APP 与 HIL 可通过 Keil 调试入口从 APP 的 Reset_Handler 启动，
不依赖 Loader 的当前镜像 CRC 记录，并保留使用说明。

最小修改：共享 `DebugApp.ini`；两个 `.uvoptx` 引用该脚本；修正普通工程被写回的
Target IROM1 起点。保留用户已有的探针、编辑器、编译优化及其他未提交修改。
调试脚本先 RESET，再设 VTOR/SP/PC；Keil 软件复位通过 OnResetExec 重设入口。

不变项：APP 与 Loader 代码、参数 ABI、Flash 分区、Loader 校验规则、独立启动、
运行时状态/故障/输出/时序均不修改。变化只在调试会话的启动及复位入口。

基线：修改前备份两个 `.uvoptx` 与两个 `.uvprojx`，记录当前所有已跟踪输入的哈希。
验收：XML 引用、内存布局与 scatter 一致；逐字段确认仅预期配置变化；普通/HIL/Loader
分别构建及 map 边界检查，项目路径检查通过；所有证据放在
`outputs/keil_app_debug_20261015/summary.json` 同目录。

本分支没有统一 verify/format/lint profile，使用现有构建与目录检查入口。
本次不下载、不启动电机；脚本语法依据 Keil 官方示例，实机调试执行仍待现场验证。
回退仅撤销新增脚本引用及本次文件，不恢复整个工作目录或覆盖用户原有设置。

## 后续授权的在线验收

用户随后明确授权 J-Link/Keil 下载并仅运行到 main，不启动电机。
在线场景：只读识别目标、确认三相关闭、备份 Flash；核对并启用下载校验、
关闭独立下载后的 Reset and Run；进入 Debug 自动停在 main，读取暂停寄存器，
回读保护分区并保持目标暂停。本轮不修改 APP 有效记录或测试独立启动。
普通 APP 已自动停在 main，完整结果记录在
`outputs/keil_app_debug_bench_20261015/summary.json`；HIL 无实机执行。
