# Keil 直接调试 APP v1

日期：2026-10-15。适用：`codex/ovp-36v` 的普通 APP、HIL 工程。

## 配置与使用

打开 `firmware/platform/stm32g4/cubemx/MDK-ARM/Vector_Mini_ST.uvprojx`。
HIL 工程为同目录的 `Vector_Mini_ST_HIL.uvprojx`，两个工程均不使用初始化脚本；
本次不改变两个工程各自已有的调试器、探针选择或编译优化配置。

磁盘工程已经在 Keil 中打开时，先取消旧 Options 窗口，再关闭并重新打开工程，
避免旧配置覆盖磁盘修改。其他源码编辑内容应另行保留，不使用 Save All 覆盖工程。

先停止电机并确认功率输出关闭，再进入调试；暂停其他工具对同一目标板的下载和运动命令。
连接配置包含复位，不适合附加观察一个正在运行的电机。暂停 CPU 时不能依赖软件保护继续执行。

1. 首次使用时，在仓库根目录用实际安装的 Python 3.10+ 执行 `python -m venv .venv`。
   构建后处理只使用标准库，不要求安装额外依赖；不要使用 WindowsApps 的占位 Python。
2. 编译当前目标，确认输出 `APP record synchronized`，且最终为 0 Error(s)。
   后处理失败必须先修复，不能继续下载失败构建留下的文件。
3. 在 `Options for Target → Debug` 核对下表，选择实际连接的探针。
4. 在调试器 Settings 中使用复位内核与外设并暂停的方式，避免仅复位内核。
   J-Link 可使用 `Reset → Normal`；不能使用只保留外设状态的 Core-only reset。
   勾选 `Reset after Connect`、`Verify Code Download`、`Download to Flash`，
   避免当前 AXF 与芯片 APP 不一致。
5. `Flash Download` 保持 `Erase Sectors`、`Program`、`Verify`；取消 `Reset and Run`，
   避免独立下载后自动运行，禁止全片擦除。
6. 普通下载使用 F8；进入调试使用 `Start/Stop Debug Session`，预计停在 APP 的 `main()`。
   独立下载不会自动运行；确认安全后才能另行复位或继续。

| 页面/配置 | 设定 |
| --- | --- |
| Debug / Load Application at Startup | 勾选 |
| Debug / Run to main() | 勾选 |
| Debug / Initialization File | 留空 |
| Utilities / Update Target before Debugging | 勾选，下载当前 APP |
| Target / IROM1 | 起点 `0x08004000`，大小 `0x18000` |
| Target / IRAM1 | 起点 `0x20000020`，大小 `0x7FE0` |
| Linker | 保留独立 `Vector_Mini_ST.sct` |

`Initialization File` 保存在对应 `.uvoptx` 的 `DebugOpt/tIfile`，现为空；
`tLdApp=1`、`tGomain=1`。`.uvoptx` 也是本功能必需的工程配置，不能只复制 `.uvprojx`。

## 启动过程

Keil 加载当前 APP 与符号，正常复位从芯片向量表进入 Loader。
Loader 校验有效记录、APP CRC 和向量后设置 VTOR/MSP，跳到 APP Reset_Handler，
再经 SystemInit、C 运行时初始化进入 main。Run to main 负责在主函数处暂停。
不再通过调试命令强行写 PC/SP/VTOR；原 DebugApp.ini 已删除。
点击继续运行后，实际行为由 APP 和连接的命令源决定。

## 正常启动与 Loader 升级

调试复位与独立启动统一经过 Loader 校验。无有效 Loader、APP 记录不匹配，
或有有效升级请求时，不应期待自动进入 APP main；先定位原因，不绕过 CRC 检查。
Keil Reset 后可能先停在 Loader；在 main 设置断点再继续可观察完整启动链路。

两个 APP 工程现已自动同步有效记录：scatter 增加 `0x08003000` 的 48 B 只读占位段，
After Build 调用 `tools/build/keil_app_record.py`，从实际 AXF 读取 APP、版本符号，
复用共享协议的记录编码填写长度/CRC/版本。Keil 下载的是已经填好记录的 AXF。
后处理失败返回非零并令构建失败，不能删除该挂钩或关闭 Loader CRC 检查。

APP 链接时使用 `--pad=0xFF`，填充至 `0x0801BFFF`；有效长度固定为 98304 B，
确保 CRC 覆盖范围全部下载，旧镜像尾部不会残留。参数区从 `0x0801C000` 开始，
不在 AXF/HEX 加载段内。记录页 6 和 APP 页 8～55 可更新；Loader 页 0～5、
保留页 7、参数页 56～63 不应擦写。算法显示的可用范围不是每次实际擦除范围。

同目录输出 `.hex`（记录 + APP）、`.bin`（纯 APP，用于 CAN 升级）、`.boot.json`
（版本、CRC、记录及文件 SHA-256）。这个 HEX 不含 Loader，不能作为空板初装大包；
空板仍使用 [Loader 合包及参数保护流程](loader_factory_package_v1.md)。
既有 Loader 与记录 ABI 未改变，可继续使用 CAN 升级。

要调试 Loader 或观察真实复位流程：保持 Initialization File 留空，
按需取消 Run to main，使用对应 Loader 符号及有效的完整固件。
复位暂停后确认 PC 位于 `0x08000000～0x08002FFF`，在 APP main 首条指令设断点，
继续后应到达 main，且 `VTOR=0x08004000`。仅运行到 main 不能替代电机控制验收。

## 排查与验证边界

- 仍停在 Loader：检查有效记录、APP CRC/向量和升级邮箱；确认构建后处理及下载成功。
- 仍提示初始化脚本不存在：Keil 缓存了旧配置，取消旧 Options，重新打开磁盘工程；不要新建同名脚本。
- 无法到 `main()`：检查 AXF 与芯片 APP 是否一致、APP 向量是否有效，以及复位是否包含外设。
- 刚改的 IROM1 又变为 `0x08000000`：旧 Options 或其他任务保存了工程，重新加载磁盘配置。

2026-10-15 已完成普通 APP 的 J-Link/Keil 在线验证：进入 Debug 后自动停在
`main.c` 第 74 行，PC 为 `0x0801979C`，SP 为 `0x20006D70`，VTOR 为
`0x08004000`。TIM1 CCER、BDTR、CFSR、HFSR 均为 0；未补发 Run/单步/运动命令。
界面刚进入 Debug 时可能短暂显示 Reset_Handler，应等待界面和 Run to main 完成后再判定。
HIL 本轮仅完成离线构建与配置检查，不作为 HIL 实机调试证据。

上述第一次调试验证发现旧记录 CRC 不匹配，已由本次自动记录同步修复。
后续使用 Keil 原生 F8 下载成功，完整回读确认 APP 与有效记录匹配，
Loader、保留页 7 和全部 16 KiB 参数逐字节不变。
不执行 `DebugApp.ini`、不写 PC/SP/VTOR，连续两次正常复位从 Loader 的
`0x08000294` 自动进入 APP 的 `main`（本次镜像 `0x080195C4`），VTOR 为
`0x08004000`。硬件断点停在 main 入口，TIM1 CCER/BDTR、CFSR/HFSR 均为 0，
未执行 APP 外设初始化或电机运动。测试结束保持暂停。
证据与构建产物哈希见 `outputs/keil_app_record_sync_20261015/summary.json`；
HIL 仍仅完成构建和离线检查，不作为 HIL 实机验证。

随后用户手动完成无脚本复位验证，停在新镜像 main 的 `0x0801978C`。
J-Link 回读确认 CRC 为 `C6779B59`，与当前 AXF 及有效记录匹配；
Loader、保留页与 16 KiB 参数仍与原备份一致，CCER/BDTR 为 0。
证据位于 `outputs/keil_app_record_sync_20261015/manual_retest_readback/summary.json`。
删除脚本后普通/HIL 均重新构建为 0 错误、0 警告；Loader 离线套件和工程路径检查通过。
通过 J-Link SDK 连续两次复位，均从 Loader `0x08000294` 到达 main `0x0801978C`，
VTOR 为 `0x08004000`，CCER/BDTR/CFSR/HFSR 均为 0；全 Flash 前后完全一致。
本次未下载或启动电机，结束保持暂停；没有重新执行 Keil GUI 的自动 Run to main。
两个磁盘工程的引用已清空，当前打开的 Keil 需重新打开工程加载配置。
配置、构建与复位证据见 `outputs/keil_without_ini_20261015/summary.json`。

参考：[Keil Debug 执行顺序](https://www.keil.com/support/man/docs/uv4cl/uv4cl_dg_debug.htm)、
[Keil 初始化示例](https://www.keil.com/support/man/docs/mdk_gs/gs_MDK5_5_en.pdf)、
[RESET 命令](https://www.keil.com/support/man/docs/uv4cl/uv4cl_cm_reset.htm)。
构建挂钩参考：[User commands](https://www.keil.com/support/man/docs/uv4cl/uv4cl_dg_user.htm)、
[输出路径占位符](https://www.keil.com/support/man/docs/uv4/uv4_ut_keysequence.asp)、
[固定长度填充](https://www.keil.com/support/docs/3407.htm)。
