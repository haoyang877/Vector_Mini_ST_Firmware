# 当前未结计划

核对日期：2026-10-22。此目录只保留以下事项；已完成实施移至completed，旧阶段移至archive。
版本与工作树仍可能变化，执行前重新冻结源码；本索引不新增硬件操作授权。

| 计划 | 当前状态 |
| --- | --- |
| [新板MCU温度可选策略](2026-10-23-mcu-temperature-optional-v1.md) | 关相调试目标临时可选；默认限流修正为3A，实机待机/故障0及120次CAN读取通过 |
| [PA4母线电流APP接入](2026-10-23-pa4-bus-current-app-v1.md) | 按硬件能力选择反馈；关相偏置及120次CAN读取通过，带载绝对精度待验证 |
| [STM32G4 APP入口统一](2026-10-23-stm32-application-entry-v1.md) | 普通APP和Loader配套APP统一使用application入口；仅离线验证 |
| [电机配置分离与1kHz默认值](2026-10-23-motor-profiles-1khz-v1.md) | HT有/无阻尼环、独立YQ型号配置；仅编译和离线验证 |
| [Rust GUI实机验收](2026-10-21-host-gui-bench-v1.md) | 当前host_app/rust候选；原计划未记录GUI实机终态，不沿用CLI结果 |
| [当前候选剩余验收](2026-10-21-current-candidate-acceptance-v1.md) | 合并旧计划中仍有效的当前候选/复位/时序证据缺口，未测保持NOT_RUN |
| [AT32F456 最小改动适配](2026-10-22-at32f456-port-v1.md) | 接口契约冻结，已分离旧板配置；先平台与MT6835，输出轴最后；底层已实现，完整APP关相接入；未完成项见计划正文 |

阶段完成和未测边界分别记录，详见[当前限制](../tech-debt-tracker.md)。
