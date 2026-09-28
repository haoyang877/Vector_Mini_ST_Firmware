# YG2026-ESC-LOADER

当前源码实现：独立 G431 Loader、公司协议 50～55、CAN-FD 1M/5M、单槽整包写入与跳前 CRC32 校验。
2026-09-28审查发现的看门狗、维护角色与协议兼容问题已完成源码修复，仍待发布与台架验收，详见
[Loader 设计 v2](../docs/architecture/loader/loader_product_v1.md)和
[实施计划](../docs/plans/completed/2026-09-28-simplified-boot-handoff.md)。

[简化审查与修复计划](../docs/plans/active/2026-09-28-loader-simplification-review-v1.md)
记录修复与本次离线证据；跳前CRC不能证明APP可正常运行。
[升级载荷 v2](../docs/protocols/yg_protocol_upgrade_wire_v2.md)采用主机随机nonce及CRC32新profile，
旧v1主机/SHA包不兼容，维护角色暂不支持。APP仅在监督有进展和本地健康时喂继承看门狗，
看门狗复位驻留Loader。

- `core/`：唯一升级事务、租约与精确 ACK 票据；不使用持久启动记录。
- `runtime/`：公司协议与产品启动流程组合；上电读邮箱、现场校验 APP 后跳转。
- `ports/stm32g4/`：有界Flash/ECC、实际CAN发送事件、系统跳转。
- `boards/vector_mini_st/`：板级时钟和安全引脚。
- `targets/vector_mini_st_g431/`：可打开的独立Keil工程、Loader/APP scatter、最小HAL配置。
- `tools/`：`generate_target.py` 目标生成与 `package_image.py` 离线镜像打包/CRC32 回填；均无硬件访问。

`shared/image` 提供 96B 清单/64B 描述符与整包 CRC32 校验，Loader 接收与跳前校验共用；
`shared/boot` 只保留 32B ENTER 邮箱。`shared/protocol` 来自 H 单份编解码，工程不链接 APP 电机或通信运行态。
Loader不含USB/UART，UART仍待后续实现。设备参数16KiB只读，首版不增加SN或改写已有校准。

目标输入修改后运行 `python loader/tools/generate_target.py`，只更新生成源码，不构建或下载。
正式构建/发布必须恢复仓库format/lint与release验证，产物放 `outputs/build/loader/`；文件命名按
[固件发布规范](../docs/guides/firmware_release_standard.md)。旧独立APP不可直接当成此布局镜像。
