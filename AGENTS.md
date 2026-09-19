# Vector Mini ST Firmware Agent Guide

This file is the repository map and working agreement. Keep detailed knowledge in
the linked documents instead of expanding this file into a handbook.

## Repository map

- `firmware/`: STM32G4 motor-control application. See `ARCHITECTURE.md`.
- `tests/`: offline unit/integration/native checks plus explicitly invoked HIL support.
- `tools/`: build, flash, bench, analysis, and project-harness commands.
- `docs/`: architecture, protocols, guides, hardware references, reports, and plans.
- `loader/`, `host_app/`, `shared/`: future independent products and shared contracts.
- `outputs/`: ignored generated evidence; never treat an old output as current proof.

Authoritative entry points: `README.md`, `ARCHITECTURE.md`, `docs/README.md`,
`STYLE.md`, `docs/architecture/code_structure.md`, and
`docs/guides/firmware_release_standard.md`.

## Environment and validation

Install uv, then run `uv sync --locked --extra dev`.

- Quick: `uv run python tools/run.py verify --profile quick`
- Pull request: `uv run python tools/run.py verify --profile pr`
- Release: `uv run python tools/run.py verify --profile release`
- Diagnose only: `uv run python tools/run.py doctor --profile <profile>`
- HIL environment: `uv run python tools/run.py doctor --profile hil`

`doctor` never contacts hardware. Quick and PR verification never flash firmware or
command a motor. Release builds firmware but does not download it. Hardware/HIL
commands require an explicit bench identity, motor profile, image hash, and scenario.

## Change rules

- Preserve unrelated user changes and ignored experiment data.
- Do not hand-edit CubeMX-generated sections outside `USER CODE` markers. After a
  CubeMX regeneration, review project inputs and run release verification.
- Protocol or parameter ABI changes require versioned documentation, golden vectors,
  compatibility notes, and corresponding tests.
- Keep APP, Loader, host application, and shared contracts independently buildable.
- Do not add absolute workstation paths, probe serials, or credentials to tracked files.
- Never use Python `assert` for a hardware safety precondition or shutdown verification.
- Generated logs, binaries, captures, and manifests belong under `outputs/`.
- Update durable docs when public behavior, architecture, safety limits, or workflows change.
- Significant work needs a versioned plan or decision record under `docs/plans/`.

## Code quality

- Follow `STYLE.md` and start new modules from the checked examples under `templates/`.
- All modules inherit the root `.editorconfig`, `.clang-format`, and Ruff configuration.
- Do not add competing module-local format rules without a documented repository decision.
- Use unit-bearing names for physical values and braces for every control-flow body.
- Prefer small single-purpose functions, explicit errors, bounded work, and comments that explain
  invariants or hardware risk rather than syntax.
- Repository-owned comments and docstrings use concise Chinese; public APIs and non-trivial modules
  must document their contract, units, ownership, or failure behavior.
- Public C/C++ declarations require a Chinese Doxygen `@brief`, every `@param`, and non-void
  `@return`; use `@note` for ISR context, side effects, blocking, ownership, or safety constraints.
- Never reformat vendor, third-party, or CubeMX-generated files.
- Run `uv run python tools/run.py format --check` and `uv run python tools/run.py lint`;
  the selected verify profile runs both again.

## Simplification, coupling, and reuse

- Before adding code, search for an existing implementation and inspect every affected caller.
- Prefer deletion, an existing project facility, or the standard library over new code or dependencies.
- Put behavior in the narrowest layer that owns it; cross-layer access uses documented interfaces.
- Do not introduce hidden dependencies through mutable globals, implicit initialization order, or
  direct hardware access outside the platform layer.
- Motor, service, and communication code must not expose STM32 HAL types, CubeMX handles, registers,
  board pins, or vendor headers. Platform APIs describe capabilities, units, timing, and failures.
- Keep board/MCU variants and vendor adaptation in BSP, ports, or composition code; do not scatter
  hardware `#ifdef` branches through portable modules.
- Separate deterministic computation from I/O and scheduling. Give mutable state one owner and
  exchange commands, snapshots, or explicit state objects across module boundaries.
- Reuse only when units, timing, ownership, errors, and safety semantics match. Prefer small obvious
  duplication over a generic abstraction that hides different behavior.
- Do not add speculative interfaces, factories, registries, or configuration. A single-implementation
  interface is justified only by a real hardware boundary, test seam, or stable ABI boundary.
- Shared code needs real callers and a focused test. Keep substantial behavior-preserving refactors
  separate from functional changes.
- Keep helpers file-local first. Promote them to a narrowly named common module only after unrelated
  real callers need the same complete contract; never create a catch-all `utils` module.

## Required checks by change

- Documentation/tooling only: quick; use PR profile when harness rules change.
- Firmware logic, protocol, parameters, build inputs, or tests: PR profile.
- CubeMX, linker, release, flash, or HIL changes: release profile plus the applicable
  explicitly authorized bench procedure.

## Code review rules

- Reject newly introduced architecture violations or expansion of the debt baseline.
- Reject hardware actions reachable from default verification or CI.
- Reject release claims without a current `summary.json` and matching artifact hashes.
- Treat failure to verify STOP or power-output disable as a hard failure requiring
  physical intervention; never automatically resume a trial.
