# Technical Debt Tracker

| ID | Debt | Enforcement and exit condition |
| --- | --- | --- |
| ARCH-001 | Motor/common headers still depend on CubeMX `main.h`. | Listed in `tools/harness/architecture_debt.json`; remove entries as pure types and platform interfaces are extracted. |
| ARCH-002 | Some motor and service code reaches APP/BSP/CubeMX details. | Architecture ratchet forbids any new include-level violation; refactor existing entries incrementally. |
| TOOL-001 | Keil requires a licensed Windows installation. | `doctor release` requires `KEIL_UV4`; CI uses a labeled self-hosted runner. |
| TEST-001 | Historical state-equivalence comparison needs an external pre-change source. | Keep optional `--reference-source`; release evidence states whether it was supplied. |

Closed: HIL-001（HIL 台架与工装已于 2026-09-19 退役，见
[HIL 退役计划](active/2026-09-19-hil-retirement.md)）。

Debt entries must describe an automated guard and a concrete deletion condition. Do not
use this file to waive newly introduced violations.
