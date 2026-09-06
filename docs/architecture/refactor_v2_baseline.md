# Refactor V2 Baseline

This record freezes the known-good input to the configurable firmware
architecture migration.  Structural refactoring must preserve these externally
observable behaviours until a phase explicitly replaces them.

## Source baseline

- Branch: `codex/production-firmware-refactor`
- Commit: `5a851dea4dcc51627aed90fb4ff31b3fc5b81dbf`
- Keil target: `Vector_Mini_ST`
- Compiler: ARMCC 5.06 update 7, build 960
- Baseline build result: 0 errors, 0 warnings
- Baseline architecture check: 132 project entries, 0 failures, 0 warnings

## Binary fingerprint

| Artifact | Size | SHA-256 |
| --- | ---: | --- |
| `Vector_Mini_ST.axf` | 2,373,900 bytes | `9B1003F61B34F1CF461C118E21A7D46DA783A3CCBFF0193B3B25B4855ADFE3C3` |
| `Vector_Mini_ST.hex` | 321,884 bytes | `5DA71388654CE020A2FDA3468519666C2E72C32D651A1C8005B829D7B04FB554` |

The artifact timestamps precede this record because the baseline command was
an incremental no-change build.  Keil still resolved and linked the target with
zero diagnostics.

## Preserved runtime contract

- ADC-injected 20 kHz motor-control interrupt remains the fast-loop trigger.
- TIM7 1 kHz remains the supervisor trigger.
- Power output is disabled until composition and configuration validation pass.
- Mode 21 remains the unified commissioning entry point during migration.
- Existing mode 13 calibration, classic CAN, USB, parameter A/B storage, and
  closed-loop speed behaviour remain compatibility requirements.
- Active hardware is Vector Mini ST + HT8115-4 + one rotor angle sensor + the
  1.5 Nm damping-ring load profile.

## Migration rule

Each structural phase must pass the host suite, architecture checks, and a Keil
build before it can replace the previous implementation.  Hardware-in-the-loop
tests are repeated when a phase changes an ISR path, peripheral adapter,
calibration procedure, persisted representation, or control law.
