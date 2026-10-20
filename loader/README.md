# YG2026-ESC-LOADER

当前实现是独立G431 Loader、CAN FD 1M/5M、公司Type1与50..55、单槽CRC32升级，
含持久激活门槛与一次试运行确认。配套APP支持56停机交接及57健康确认。
源码与离线验证不等于实机验收；当前候选的实机范围见[验收记录](../docs/reports/2026-10/loader_storage_optimization_20261020.md)。

- [软件框图、紧凑流程图与内存分配](../docs/architecture/loader/loader_product_v3.md)
- [升级线路、设备身份页及容器v3](../docs/protocols/yg_protocol_upgrade_wire_v3.md)
- [实施、验收与回退计划](../docs/plans/active/2026-09-29-loader-p0-p1-v1.md)
- [Keil 单独下载 LoaderApp 指南](../docs/guides/keil_loader_app_debug.md)
- `core/`：单事务、5s租约、主机nonce/session与精确ACTIVATE发送票据。
- `runtime/`：安全启动、50ms窗口、现场校验、GET_INFO与升级路由。
- `ports/`、`boards/`：固定APP写域、ECC读取、CAN物理事件、复位和跳转。
- `targets/`：独立 Keil 工程及 layout 6 的 16 KiB Loader/96 KiB APP/16 KiB 参数与身份布局。
- `tools/package_image.py`：96B描述符回填、CRC与目标检查；原始占位BIN不能直接升级。
- `tools/build/keil_loader_app.py`：LoaderApp Keil 构建后生成可直下的调试 AXF/HEX/BIN，
  同时生成标记未激活的 CAN 升级包；两者用途不同。
- `tools/factory_identity.py`：独立生产页的SN/电机类型；默认只生成文件，显式SWD写入需
  备份、UID及数字关相核对，CAN没有生产写命令。CAN ID由当前参数 schema 所有者管理。

目标清单修改后运行`python loader/tools/generate_target.py`及
`python tools/build/generate_loader_app_project.py`；只生成工程输入，不构建或下载。
`python tools/run.py build_loader_bundle --uv4 ... --fromelf ... --token ... --output outputs/...`
独立构建两端，并生成双版本命名的整机 SWD HEX、单件工件、CAN 包与 SHA-256 清单。
默认只产出 debug 候选，
`release_approved=false`；需完成当前release与台架验收后再按[发布规范](../docs/guides/firmware_release_standard.md)发布。

升级工具：`python host_app/upgrade_client.py inspect --package outputs/.../app-package`为离线检查；
`upgrade`才打开用户指定SDK与节点，须显式UID、独占及无电机动力条件。默认验证/CI不接触设备。
有多只 USB-CAN 时可通过 `--device-index 1` 选择第 2 只适配器；默认索引为 0，
CAN 通道固定为 0。设备索引和通道索引不是同一个编号，不能以适配器可打开代替目标 UID 核对。
不支持USB、UART、第二MCU、跨复位续传、旧APP回滚、维护角色或签名。
layout 5 到 layout 6 必须通过受控 SWD 同时迁移 Loader 与 APP；不能混用旧址 APP
或以 CAN 改写 Loader。当前候选为 Loader v2.1.0、APP v1.5.0，参数使用[schema14](../docs/protocols/parameter_schema14_storage_v1.md)。
旧版本的二十轮实测不计入新候选次数。当前候选已完成三次连续升级及三项故障恢复，
真实掉电和参数保存断电仍未覆盖。
APP 的 READY 回复必须关联队列入队位置，并等待该帧真实发送完成后才能复位；
修复与验证见[交接队列修复计划](../docs/plans/active/2026-10-20-loader-handoff-queue-v1.md)。

升级客户端的 summary.json 现含 timing：按命令统计发送耗时、回复等待和重试，失败请求也计入。
发送耗时包含传输适配器以及外层 J-Link 检查；不能称作纯 CAN 总线耗时。
