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
