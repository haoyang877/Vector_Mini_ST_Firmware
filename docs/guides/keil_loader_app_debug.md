# Keil 单独下载 LoaderApp

本流程用于已经安装匹配 Loader 的板卡，只更新 APP 槽。LoaderApp 工程链接在
`0x08004000` 起的 layout 6 APP 区；Loader、参数与设备身份区不属于这个 AXF 的加载段。

1. 使用当前共同目标清单重新生成 LoaderApp 工程，并打开
   `projects/stm32g4/MDK-ARM/ESC-APP-STM32G431-LoaderApp.uvprojx`。
   工程的 After Build 会调用当前工作树 `.venv` 中的 Python；构建失败时停止。
2. 构建完成后检查 `outputs/build/keil/ESC-APP-STM32G431-LoaderApp/` 中的
   `ESC-APP-STM32G431-LoaderApp.debug.json`。它记录 AXF、HEX、BIN 和 `app-package/`
   的哈希。无源码变更的重复构建会严格核对并复用这些文件；核对失败则构建失败，
   旧派生文件会失效。调试下载使用该目录的 AXF；`app-package/image.bin` 只供 CAN 升级。
3. 台架上使用 J-Link 时，在 Keil Flash Download 中选择按扇区擦除和下载后校验，
   确认设备 Flash 算法覆盖 APP 槽，不选择全片擦除。关闭自动 Run to main 和任何
   强制设置 PC、SP、VTOR 的调试初始化；下载后通过硬件复位从 Loader 入口启动。
4. Loader 等待短暂升级窗口后验证 APP 向量、描述符、CRC 和激活标记，检查通过
   才跳转。若没有跳转，先读取 Loader 状态与构建清单，不用调试器强行跳过检查。

Keil 直下镜像的三个启动标记在构建时已置为有效，便于反复调试；它不是 CAN
升级成功或 APP 健康确认的证据。正式 CAN 升级使用 `app-package/`：其中镜像标记
保持擦除态，Loader 负责写入、验证、激活和试运行确认。两类文件不可互换。

2026-10-17 的 80,360 B APP 属于旧 layout 5 冻结包；其台架记录见
[本轮集成记录](../reports/2026-10/delivery_feature_integration_20261017.md)。
新 layout 6 必须重新链接和验证，不能将旧包改名或单独与新 Loader 混装。
本指南中的操作设置仍需在目标板确认，离线构建不是实机通过证据。
