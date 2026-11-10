# Keil 单独下载 LoaderApp

本流程用于已经安装匹配 Loader 的板卡，只更新 APP 槽。LoaderApp 工程链接在
`0x08004000..0x0801BFFF` APP 区；G431为layout 6，AT32为layout 7。
Loader与`0x0801C000..0x0801FFFF`整个参数/身份区不属于这个AXF的加载段。
首次部署AT32新布局必须先按[双板指南](dual_board_loader.md)完成原样迁移及配套初装。

1. 使用当前共同目标清单重新生成 LoaderApp 工程，并打开
   `projects/stm32g4/MDK-ARM/ESC-APP-STM32G431-LoaderApp.uvprojx`。
   AT32使用 `projects/at32f456/application/ESC-APP-AT32F456.uvprojx`。
   根工作区`YG-ESC.uvmpw`只列出这两套日常下载工程；不要选择从0地址开始的普通APP工程。
   工程的 After Build 会调用当前工作树 `.venv` 中的 Python；构建失败时停止。
2. 构建完成后检查 `outputs/build/keil/ESC-APP-STM32G431-LoaderApp/` 中的
   `ESC-APP-STM32G431-LoaderApp.debug.json`。它记录 AXF、HEX、BIN 和 `app-package/`
   的哈希。无源码变更的重复构建会严格核对并复用这些文件；核对失败则构建失败，
   旧派生文件会失效。调试下载使用该目录的 AXF；`app-package/image.bin` 只供 CAN 升级。
   AT32产物位于`outputs/build/keil/ESC-APP-AT32F456/`；保留SRAM快代码的Flash装载副本，计入96 KiB预算。
3. 台架上使用 J-Link 时，在 Keil Flash Download 中选择按扇区擦除和下载后校验，
   确认设备 Flash 算法覆盖 APP 槽，不选择全片擦除。关闭自动 Run to main 和任何
   强制设置 PC、SP、VTOR 的调试初始化。AT32 工程默认选择 J-Link/SWD。先 Build（F7）和
   Flash → Download（F8），再用 Debug → Start/Stop Debug Session（Ctrl+F5）进入硬件调试。
   项目的初始化脚本从复位位置执行到 APP 的 `main`，然后置位 AT32 的调试冻结位，
   使独立看门狗在 CPU 停在断点期间暂停计数。F11 单步进入，F5 继续执行。
4. Loader 等待短暂升级窗口后验证 APP 向量、描述符、CRC 和激活标记，检查通过
   才跳转。若没有跳转，先读取 Loader 状态与构建清单，不用调试器强行跳过检查。
5. 首次确认Keil配置时，下载前后用维护工具各读取两次完整Flash，再执行`verify`。
   必须确认Loader和整个16 KiB尾区逐字节不变；仅看CRC升级成功不能证明保护区未变。

Keil 直下镜像的三个启动标记在构建时已置为有效，便于反复调试；它不是 CAN
升级成功或 APP 健康确认的证据。正式 CAN 升级使用 `app-package/`：其中镜像标记
保持擦除态，Loader 负责写入、验证、激活和试运行确认。两类文件不可互换。

2026-10-17 的 80,360 B APP 属于旧 layout 5 冻结包；其台架记录见
[本轮集成记录](../reports/2026-10/delivery_feature_integration_20261017.md)。
新 layout 6 必须重新链接和验证，不能将旧包改名或单独与新 Loader 混装。
2026-11-10 已在 AT32 PITCH 节点 6 用 Keil µVision 和 J-Link 验证 APP 下载。
Keil 报告擦除、编程和校验成功。下载前后读取全部 256 KiB，APP 与 77,240 字节调试镜像一致，
APP 槽外的 163,840 字节完全相同，包括 Loader、参数、标定和身份页。
调试器在 `main`（0x0800D180）停留约 34 秒后，仍可单步进入 `Bsp_Init`（0x080056FC）。
退出并复位后，CAN 回读节点 6、已确认 APP、零故障，SWD 回读向量表为 0x08004000、六路输出关闭。
原始下载日志、断点日志及复位回读位于 `outputs/keil-pitch-app-debug-20261110`。
