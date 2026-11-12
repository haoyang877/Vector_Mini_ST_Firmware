# BSP API v1（修订6）

当前范围：保留PWM、ADC、Encoder（SPI）、CAN与Flash及必要系统支撑。LED、UART已删除，
不保留接口、占位实现或工程引用。见[删除记录](../../plans/active/2026-10-30-remove-led-uart-v1.md)。

2026-10-30更新：CAN两板已统一最简接口，本目录副本与当前公开头同步；
旧票据/保护约定由[当前CAN链路](../can_chain_v1.md)取代，本轮未编译、未测试。

2026-11-10 增加 `BspSystem_Reset`，用于停机型号配置的显式重启；调用者先确认关相、禁止启动及回复发送完成。
当前为8个公开头、40个函数声明（含3个应用回调）；内部初始化和单相台架接口不计入公开面。正式声明在[firmware/platform/bsp/include](../../../firmware/platform/bsp/include)，
本目录保留一致副本供接口文档引用。完整[分层、函数索引和两板差异](../bsp_interfaces_v1.md)。

- [执行约定](EXECUTION.md)：生命周期、快帧、许可、事务、错误及恢复。
- [信号与功能覆盖](COVERAGE.md)：九个ADC通道及十个模块。
- [历史评审](REVIEW.md)：修订前发现的问题，处理情况由当前契约和测试判断。
- [Loader只读查询](BOOT_AVAILABILITY_REVIEW.md)：已获准实现，避免二次初始化。

接口变更不再逐项等待用户评审；实现者须同步声明、文档、消费者和回归。
硬件版本ADC尚未接线；LED与UART接口已经删除。
AT32母线电流为PWM同步注入快采样，INIT零偏校正与三相共用2048个快拍窗口。
按修订6的接口和职责重构已接入两板，双板构建及干净快照发布检查通过；统一实机验收尚未完成。
电流零偏满窗后在INIT快拍统一提交，正常采样与校正分离，详见执行约定第2节。
当前结果及逐模块对应见[离线验收](../../reports/2026-10/bsp_architecture_offline_20261026.md)；
此前过程见[实施记录](../../reports/2026-10/bsp_api_v1_implementation_20261026.md)。
后续[私有头合并与直接查询收敛](../../reports/2026-10/bsp_private_headers_20261026.md)保持本修订公开接口不变，
九工程及完整离线验收已通过。
