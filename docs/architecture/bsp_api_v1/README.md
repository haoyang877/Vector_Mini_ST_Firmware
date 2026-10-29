# BSP API v1（修订6）

十个头文件、57个函数已接入两板。正式声明在[firmware/platform/bsp/include](../../../firmware/platform/bsp/include)，
本目录保留一致副本供接口文档引用。完整[分层、函数索引和两板差异](../bsp_interfaces_v1.md)。

- [执行约定](EXECUTION.md)：生命周期、快帧、许可、事务、错误及恢复。
- [信号与功能覆盖](COVERAGE.md)：九个ADC通道及十个模块。
- [历史评审](REVIEW.md)：修订前发现的问题，处理情况由当前契约和测试判断。
- [Loader只读查询](BOOT_AVAILABILITY_REVIEW.md)：已获准实现，避免二次初始化。

接口变更不再逐项等待用户评审；实现者须同步声明、文档、消费者和回归。
UART和硬件版本ADC尚未接线；UART统一返回NOT_SUPPORTED，不提供虚构硬件实现。
AT32母线电流已改为PWM同步注入快采样，原2kHz零点/滤波时间窗保留。
按修订6的接口和职责重构已接入两板，双板构建及干净快照发布检查通过；统一实机验收尚未完成。
相电流零偏满窗后的收尾已移至2kHz监督任务，属于有记录的调度调整，详见执行约定第2节。
当前结果及逐模块对应见[离线验收](../../reports/2026-10/bsp_architecture_offline_20261026.md)；
此前过程见[实施记录](../../reports/2026-10/bsp_api_v1_implementation_20261026.md)。
后续[私有头合并与直接查询收敛](../../reports/2026-10/bsp_private_headers_20261026.md)保持本修订公开接口不变，
九工程及完整离线验收已通过。
