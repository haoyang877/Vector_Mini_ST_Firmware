# Technical Debt Tracker

| ID | Debt | Enforcement and exit condition |
| --- | --- | --- |
| ARCH-001 | Motor headers still reach services profiles (`motor_axis_profile` / `foc_param_profile` / `fast_loop_profile`) as include-level reverse dependencies. | Listed in `tools/harness/architecture_debt.json` (5 entries); remove entries as those profiles are inverted into `platform/api` or pure types. |
| ARCH-002 | Some motor code reaches APP telemetry/service details. | Architecture ratchet forbids any new include-level violation; refactor existing entries incrementally. The hardware boundary is now closed: only `firmware/platform/stm32g4/**` includes CubeMX/HAL. |
| TOOL-001 | Keil requires a licensed Windows installation. | `doctor release` requires `KEIL_UV4`; CI uses a labeled self-hosted runner. |
| TEST-001 | Historical state-equivalence comparison needs an external pre-change source. | Keep optional `--reference-source`; release evidence states whether it was supplied. |

Closed: HIL-001（HIL 台架与工装已于 2026-09-19 退役，见
[HIL 退役计划](active/2026-09-19-hil-retirement.md)）。

Debt entries must describe an automated guard and a concrete deletion condition. Do not
use this file to waive newly introduced violations.
