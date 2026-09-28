# YG2026-ESC-LOADER

当前源码实现：独立G431 Loader、公司协议50～55、CAN-FD 1M/5M、单槽镜像提交与试启动救援。
本轮按要求只开发，未构建、未测试；APP容量及E/H集成未验收。详见
[软件框架、内存与故障策略 v1](../docs/architecture/loader/loader_product_v1.md)和
[实施计划](../docs/plans/active/2026-09-28-loader-product-refactor.md)。

- `core/`：唯一升级事务、租约/ACK票据、boot双页高水位。
- `runtime/`：公司协议与产品启动流程组合。
- `ports/stm32g4/`：有界Flash/ECC、实际CAN发送事件、系统跳转。
- `boards/vector_mini_st/`：板级时钟和安全引脚。
- `targets/vector_mini_st_g431/`：可打开的独立Keil工程、Loader/APP scatter、最小HAL配置。
- `tools/`：目标生成、离线镜像打包与首次boot供应文件生成；均无硬件访问。

`shared/image`与APP确认路径共用；`shared/protocol`来自H单份编解码，工程不链接APP电机或通信运行态。
Loader不含USB/UART，UART仍待后续实现。设备参数16KiB只读，首版不增加SN或改写已有校准。

目标输入修改后运行 `python loader/tools/generate_target.py`，只更新生成源码，不构建或下载。
正式构建/发布必须恢复仓库format/lint与release验证，产物放 `outputs/build/loader/`；文件命名按
[固件发布规范](../docs/guides/firmware_release_standard.md)。旧独立APP不可直接当成此布局镜像。
