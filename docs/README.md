# Documentation Map

This directory is the versioned system of record. `AGENTS.md` and `ARCHITECTURE.md`
provide the short entry map; details belong here.

软件架构阅读入口：[电机驱动软件架构说明](architecture/software_architecture.md)。
按产品职责、APP 分工、命令流转、周期调度、参数与状态所有权组织，包含当前架构图片。

核心阅读入口：[模块核心职责与删减清单](architecture/module_core_responsibilities.md)，
模块细节见[核心控制执行](architecture/motor_execution_core_v3.md)、[核心维护与标定](architecture/motor_maintenance_core_v3.md)和[参数所有权](architecture/parameter_ownership_v1.md)。历史提取结果见[2026-10-08 核心解耦集成记录](reports/2026-10/module_core_integration_20261008.md)。

当前收口：[核心电机工程 v3](architecture/core_only_v3.md)，
按[核心删减决定](plans/active/2026-10-08-core-only-v3.md)删除扩展功能。
核心线路：[SPEED能力声明 v2](protocols/yg_protocol_core_speed_caps_v2.md)、
[普通控制启动 v0.2](protocols/yg_protocol_ordinary_start_wire_v0_2.md)、
[单项标定/保存 v1.2](protocols/yg_protocol_calibration_wire_v1_2.md)。

运行会话与整计划线路v1.4已退役。历史设计：[启动身份页与恢复边界](protocols/boot_epoch_record_v1.md)。
当前上位机为[单页核心工具](../host_app/README.md)，独立部署收口见
[上位机升级依赖收口](plans/active/2026-10-16-host-portable-upgrade-v1.md)；
[只读主机端口记录](plans/active/2026-10-08-host-get-only-port-v1.md)保留为历史阶段。

当前源码候选：[APP v1.3.0核心范围](releases/architecture-app-v1.3.0-candidate.md)。

进行中的交付功能移植：[集成计划](plans/active/2026-10-17-delivery-feature-integration-v1.md)、
[schema13参数与清标定契约](protocols/parameter_schema13_delivery_v1.md)。
新包及实机结果须以本轮冻结记录为准，不能沿用v1.3.0结论。

Loader 16 KiB / APP 96 KiB 的布局迁移见
[layout 6 计划](plans/active/2026-10-17-loader-16k-layout-v1.md)与
[当前 Loader 设计](architecture/loader/loader_product_v3.md)。
APP 的 READY 发送位置修复及实测见
[交接队列修复 v1](plans/active/2026-10-20-loader-handoff-queue-v1.md)。

历史候选：[APP v1.2.0 收口范围和真实限制](releases/architecture-app-v1.2.0-candidate.md)。

架构发布候选：[APP v1.1.0 实际范围与未测项](releases/architecture-app-v1.1.0-candidate.md)。
八状态完整入口：[状态职责与当前总图](architecture/motor_state_machine_v1.md)、
[全部跳转及逐状态输入](architecture/motor_state_flow_simplification_v1.md)、
[整机调用流程](architecture/motor_platform_lifecycle_v1.md)。
线路恢复行为见[状态反馈 v1.2](protocols/yg_protocol_motor_state_v1_2.md)。

| Area | Purpose |
| --- | --- |
| [architecture/](architecture/) | Current structure, design decisions, and future architecture |
| [protocols/](protocols/) | CAN/CAN-FD contracts, command catalogs, and golden vectors |
| [guides/](guides/) | Operator, calibration, control, safety, and release procedures |
| [hardware/](hardware/) | Board manuals and component performance references |
| [analysis/](analysis/) | Focused technical investigations |
| [reports/](reports/) | Immutable dated validation and bench evidence summaries |
| [releases/](releases/) | Delivered firmware release records |
| [plans/](plans/) | Active/completed execution plans, decisions, and technical debt |

Start with [the code structure](architecture/code_structure.md), the
[firmware release standard](guides/firmware_release_standard.md), and the
[project harness guide](guides/project_harness.md), then the
[test guide](../tests/README.md). Reports describe what happened at a point in time;
they are not substitutes for current `verify` evidence.

All local Markdown links are checked by the PR profile. Add new durable documents to
this map or to the closest indexed sub-area.

Keil debugging: [Watch 只读观测变量与 SWD 台架邮箱退役记录](guides/watch_motor_commands.md).

E/H/D historical snapshot: [整体重构初始候选（2026-09-28，历史）](reports/2026-09/system_refactor_candidate_20260928.md).

Protection design: [故障保护模块、接口与参数设计 v1.0](architecture/fault_protection_design_v1.md)
and its [implementation plan](plans/active/2026-09-18-fault-protection-v1.md).

Communication layering: [通信分层说明（CAN 接入）](architecture/communication_layering.md).

Current protocol overview: [yg_protocol 通信协议总览与当前状态 v0.1](protocols/yg_protocol_system_overview_v0_1.md).

CAN FD link smoke test: [yg_protocol CAN FD 硬件连通性冒烟测试 v0.1](protocols/yg_protocol_link_smoke_test_v0_1.md).

Business type allocation proposal: [yg_protocol 业务 Type 分配提案 v0.3](protocols/yg_protocol_business_type_allocation_v0_3.md).

Historical ordinary motor start proposal: [普通控制启动线路提案 v0.1](protocols/yg_protocol_ordinary_start_proposal_v0_1.md).

Motor payload design: [yg_protocol 电机控制、参数与反馈 Payload 细化 v0.1](protocols/yg_protocol_motor_payload_design_v0_1.md).

Speed estimation optimization: [编码器测速估计优化：20 kHz PLL 替换 8 ms 滑窗](reports/2026-09/encoder_speed_pll_optimization_20260923.md).

Current protocol entry: [当前设备发现与支持范围](protocols/yg_protocol_device_discovery_v1.md),
[参数 schema12 装载政策](protocols/parameter_schema12_loading_policy_v1.md), and
[电机线路契约](protocols/yg_protocol_motor_wire_v1.md).

Historical ordinary speed start candidate: [CMD ID 0x0075 线路与黄金向量 v0.1](protocols/yg_protocol_ordinary_start_wire_v0_1.md)
and [implementation plan](plans/active/2026-09-30-ordinary-start-wire-v1.md).

Historical motor redesign proposal: [yg_protocol 简化电调协议 v0.1](protocols/yg_protocol_simplified_motor_v0_1.md)
and its [implementation decision](plans/active/2026-09-23-yg-protocol-simplification.md).

CMD ID audit before implementation: [yg_protocol CMD ID 核对表 v0.1](protocols/yg_protocol_cmd_id_review_v0_1.md).

CMD behavior review: [yg_protocol CMD 功能契约 v0.1](protocols/yg_protocol_cmd_function_contract_v0_1.md).

Per-command motor feedback revision: [电机逐命令反馈契约 v0.2](protocols/yg_protocol_motor_feedback_v0_2.md).

Motor parameter read/write catalog: [电调读写参数表 v0.1](protocols/yg_protocol_parameter_catalog_v0_1.md).

Single-board hardware verification: [yg_protocol 单板 CAN FD 实机验证 2026-09-28](reports/2026-09/yg_protocol_canfd_bench_20260928.md).

Historical loader implementation: [简化候选设计 v2（独立分区 + 跳前 CRC32 校验）](architecture/loader/loader_product_v1.md)
and [历史简化审查与修复记录 v1](plans/completed/2026-09-28-loader-simplification-review-v1.md).

Historical loader wire contract: [升级载荷 v2：主机 nonce 与 CRC32 镜像](protocols/yg_protocol_upgrade_wire_v2.md).

APP capacity baseline: [新 APP 编译占用与缩减方向（2026-09-28）](reports/2026-09/app_capacity_baseline_20260928.md).

APP Flash investigation: [超限原因、旧模式调用链与隔离编译对照（2026-09-28）](reports/2026-09/app_flash_root_cause_20260928.md).

Retired modes: [无当前入口的旧电机模式退役契约 v1](protocols/retired_motor_modes_v1.md).

APP Flash details: [按功能、驱动和函数拆分的 Flash 占用（2026-09-28）](reports/2026-09/app_flash_by_function_20260928.md).

Current parameter loading: [schema12：持久配置确认、完整性 CRC 与回退策略 v1](protocols/parameter_schema12_loading_policy_v1.md).

Numeric encoding: [CAN 与参数整数编码的等价实现 v1](protocols/numeric_encoding_v1.md).

Latest APP capacity: [第一批容量优化结果及验证限制](reports/2026-09/app_flash_batch1_20260928.md).

Unified trigonometry: [统一正余弦查表与容量验证（2026-09-28）](reports/2026-09/unified_trig_20260928.md).

APP capacity continuation: [标定、平台与配置校验空间优化](reports/2026-09/app_flash_continuation_20260928.md).

Motor state machine: [八状态职责、当前总图、启停与任务边界](architecture/motor_state_machine_v1.md).

Motor state machine validation: [重构验收与未验证项](reports/2026-09/motor_state_machine_refactor_20260928.md).

Current framework closure: [边界、门禁、容量与离线验收（2026-09-29）](plans/completed/2026-09-29-framework-closure-v1.md)。

Legacy cleanup: [删除重复实现、退役运行字段与过期入口（源码已清理，统一验证待进行）](plans/active/2026-09-29-legacy-cleanup-v1.md)。

APP naming: [模块、类型、变量、单位与对象组织 v1](architecture/app_naming_v1.md)。

自检与准备资格：[基础自检与模式就绪 v1](architecture/motor_readiness_v1.md)。

状态机可读性：[按状态分支、无 goto 的优化与验收](reports/2026-09/motor_state_readability_20260929.md)。

状态机冗余删减：[保留全部电机功能的精简与容量验收](reports/2026-09/motor_state_core_20260929.md)。

Loader current implementation: [Loader软件设计v3](architecture/loader/loader_product_v3.md) and
[升级线路v3](protocols/yg_protocol_upgrade_wire_v3.md).
