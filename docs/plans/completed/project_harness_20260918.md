# Project harness implementation — 2026-09-18

## Intent

Make repository knowledge, environment setup, verification, architecture constraints,
release evidence, and HIL safety legible and mechanically enforceable.

## Acceptance criteria

- Locked Python environment and clean-checkout setup.
- Offline quick/PR verification with machine-readable summaries.
- Keil release build evidence without flashing.
- Architecture, documentation, and repository-hygiene ratchets.
- Hardware tools use explicit local configuration and non-optimizable safety checks.
- GitHub CI separates offline, licensed Keil, scheduled governance, and manual HIL work.

## Decision log

- uv is the Python environment and lock manager.
- Zig is installed from the pinned `ziglang` Python distribution for cross-platform CI.
- Existing architecture violations are a checked-in include-level baseline.
- HIL remains local/manual; CI can validate its code and evidence schema but cannot arm it.

## Evidence

Final evidence is produced by `python tools/run.py verify --profile pr` and, on a configured
Windows host, `python tools/run.py verify --profile release`. Evidence is current only when
its recorded Git SHA and artifact hashes match the checkout.
