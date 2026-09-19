# 0001 — Repository-native project harness

Status: accepted, 2026-09-18.

## Context

The repository had useful build, test, bench, and analysis scripts, but a fresh environment
could not discover or reproduce the complete validation loop without workstation knowledge.

## Decision

Use a short `AGENTS.md` as the map, uv for a locked Python environment, `harness.toml` as
the validation configuration, `tools/run.py doctor/verify` as stable entry points, GitHub
Actions for offline and Keil gates, and manually authorized bench execution with evidence.

Architecture and documentation rules are mechanical ratchets. Existing violations are
recorded as debt; new violations fail verification. CI never flashes or moves a motor.

## Consequences

Developers and agents can validate from a clean checkout and receive actionable failures.
The Keil build needs a licensed self-hosted Windows runner. HIL was retired on 2026-09-19
(see the [HIL retirement plan](../active/2026-09-19-hil-retirement.md)); any remaining
hardware bench work stays manually authorized by a physically present operator and is not
a merge-time automation.
