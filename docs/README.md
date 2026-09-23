# Documentation Map

This directory is the versioned system of record. `AGENTS.md` and `ARCHITECTURE.md`
provide the short entry map; details belong here.

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

Protection design: [故障保护模块、接口与参数设计 v1.0](architecture/fault_protection_design_v1.md)
and its [implementation plan](plans/active/2026-09-18-fault-protection-v1.md).

Communication layering: [通信分层说明（CAN 接入）](architecture/communication_layering.md).

CAN FD link smoke test: [yg_protocol CAN FD 硬件连通性冒烟测试 v0.1](protocols/yg_protocol_link_smoke_test_v0_1.md).

Business type allocation proposal: [yg_protocol 业务 Type 分配提案 v0.3](protocols/yg_protocol_business_type_allocation_v0_3.md).

Motor payload design: [yg_protocol 电机控制、参数与反馈 Payload 细化 v0.1](protocols/yg_protocol_motor_payload_design_v0_1.md).

Current motor redesign proposal: [yg_protocol 简化电调协议 v0.1](protocols/yg_protocol_simplified_motor_v0_1.md)
and its [implementation decision](plans/active/2026-09-23-yg-protocol-simplification.md).

CMD ID audit before implementation: [yg_protocol CMD ID 核对表 v0.1](protocols/yg_protocol_cmd_id_review_v0_1.md).

CMD behavior review: [yg_protocol CMD 功能契约 v0.1](protocols/yg_protocol_cmd_function_contract_v0_1.md).
