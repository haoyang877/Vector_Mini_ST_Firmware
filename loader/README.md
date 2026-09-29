# YG2026-ESC-LOADER

当前实现是独立G431 Loader、CAN FD 1M/5M、公司Type1与50..55、单槽CRC32升级，
含持久激活门槛与一次试运行确认。配套APP支持56停机交接及57健康确认。
源码与离线验证不等于实机验收；本轮不连接用户正在验证APP的硬件。

- [软件框图、紧凑流程图与内存分配](../docs/architecture/loader/loader_product_v3.md)
- [升级线路、设备身份页及容器v3](../docs/protocols/yg_protocol_upgrade_wire_v3.md)
- [实施、验收与回退计划](../docs/plans/active/2026-09-29-loader-p0-p1-v1.md)
- `core/`：单事务、5s租约、主机nonce/session与精确ACTIVATE发送票据。
- `runtime/`：安全启动、50ms窗口、现场校验、GET_INFO与升级路由。
- `ports/`、`boards/`：固定APP写域、ECC读取、CAN物理事件、复位和跳转。
- `targets/`：独立Keil工程及20KiB Loader/92KiB APP/16KiB参数布局。
- `tools/package_image.py`：96B描述符回填、CRC与目标检查；原始占位BIN不能直接升级。
- `tools/factory_identity.py`：独立生产页的SN/电机类型；默认只生成文件，显式SWD写入需
  备份、UID及数字关相核对，CAN没有生产写命令。CAN ID继续由schema12参数所有者管理。

目标清单修改后运行`python loader/tools/generate_target.py`及
`python tools/build/generate_loader_app_project.py`；只生成工程输入，不构建或下载。
`python tools/run.py build_loader_bundle --uv4 ... --fromelf ... --token ... --output outputs/...`
独立构建两端并冻结工件、版本、schema12和SHA256。默认只产出debug候选，
`release_approved=false`；需完成当前release与台架验收后再按[发布规范](../docs/guides/firmware_release_standard.md)发布。

升级工具：`python host_app/upgrade_client.py inspect --package outputs/.../app-package`为离线检查；
`upgrade`才打开用户指定SDK与节点，须显式UID、独占及无电机动力条件。默认验证/CI不接触设备。
不支持USB、UART、第二MCU、跨复位续传、旧APP回滚、维护角色或签名。
v2/layout4与v3/layout5必须配套迁移，不能混用旧Loader与新APP。
