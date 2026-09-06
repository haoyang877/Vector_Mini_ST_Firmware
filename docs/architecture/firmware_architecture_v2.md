# Configurable Motor Firmware Architecture

## Purpose

The firmware is a product-line implementation, not a single-board program.  A
build selects exactly one typed `ProductConfig`.  That configuration composes a
board, motor, load, sensing topology, feedback routes, feature policy, and
commissioning policy without changing control algorithms or protocol code.

The architecture follows a functional-core / imperative-shell rule: Core owns
decisions and calculations; BSP and Platform own physical effects.

This document defines the target state. During the P1 migration checkpoint the
complete `ProductConfig` is validated by the host gate, while the production
binary still runs the validated legacy `ProductVariant`. A compact immutable
selection record links the configuration fingerprint to that legacy selection
and the BSP board identity before any PWM or periodic interrupt is enabled.
This dual-model bridge is deleted when all runtime consumers use
`ProductConfig`; it is not the completed architecture.

## Physical layout

```text
Firmware/
├── Core/
│   ├── Application/       scheduling, state machine, use-case orchestration
│   ├── Services/          motor control, feedback, measurement, safety,
│   │                      commissioning, parameters, and math
│   ├── Communication/     protocol codecs, command routing, sessions
│   ├── Config/            typed catalog, product selection, validation
│   └── Infrastructure/    diagnostics, metrics, repositories
├── Drivers/               MCU-independent device protocols
├── Bsp/
│   ├── Api/               five narrow hardware contracts
│   └── Boards/            board resources, endpoint maps, safe-state binding
└── Platform/              MCU-family implementations and Simulation
```

Vendor-generated `Core/`, `Drivers/`, `Middlewares/`, and `USB_Device/` at the
repository root are build inputs owned by STM32 tooling.  They are outside the
product firmware namespace and must only be reached through Platform/BSP code.

## Dependency rule

```text
Core/Communication ------> Core/Application use-case contracts
Core/Application
      |---> Core/Services
      |---> Core/Config
      `---> Core/Infrastructure contracts

Core/Services -----------> Core-local contracts and value types
Drivers -----------------> BSP bus/resource contracts
Bsp/Boards --------------> Bsp/Api + Drivers + Platform
Platform ----------------> vendor HAL/CMSIS/generated code
Composition -------------> all concrete selections (the only such location)
```

Every arrow points toward a dependency.  In particular:

- Core never includes STM32, HAL, CMSIS, CubeMX, GPIO/pin, timer/ADC instance,
  or concrete sensor-driver headers.
- Communication decodes external messages and invokes Application use-case
  contracts.  Application never includes a protocol, transport, or concrete
  Communication implementation; bootstrap/composition schedules both peers.
- Drivers know a device protocol but not MCU registers, physical pins, or a
  product configuration.
- `Bsp/Api` contains no board, MCU, vendor, or device name.
- A service receives only its narrow interface and immutable configuration; it
  cannot reach a global hardware bundle or another service context.
- Only Composition may see the complete `BspDeviceSet` and instantiate the
  selected product.

## Five BSP contracts

The public BSP surface is intentionally small:

1. `bsp_motor_drive.h`: synchronized PWM/current capture, current-sampling
   plan, bus voltage, hardware-fault state, and immediate safe disable.
2. `bsp_angle_sensor.h`: initialized, timestamped angle observations with
   source-independent quality/status.
3. `bsp_temperature.h`: readings tagged with their physical thermal zone.
4. `bsp_communication.h`: raw frame/byte transport; no command decoding.
5. `bsp_system.h`: time, critical sections, identity, reset, diagnostics, and
   nonvolatile block primitives.

This avoids a giant `hw_interface.h` while keeping the number of hardware
boundaries understandable.

## Product and capability model

The selected `ProductConfig` declares bounded arrays of sensor instances and
explicit feedback routing.  Absence is represented by a zero count or `NONE`,
never by a dummy driver.  Sensorless estimation is a feedback source, not a
pretend encoder.

Supported current-sense topologies are closed and validated:

- inline three-shunt;
- low-side three-shunt;
- low-side two-shunt with phase reconstruction;
- DC-link single-shunt with PWM-coupled sample planning.

All strategies normalize into a `PhaseCurrentSample` containing `ia`, `ib`,
`ic`, a valid-phase mask, quality, and sequence number.  Single-shunt support
also requires atomic PWM plus ADC-trigger-plan commit; merely selecting an enum
is insufficient.

Capabilities are derived from both the board endpoint map and product choices.
Required features that cannot be satisfied are configuration errors.  Startup
must keep PWM disabled and expose the validation report.

## Commissioning and persistence

The general commissioning order is fixed, while the executable plan is derived
from capabilities and policy.  Each step has an instance target, prerequisites,
required capabilities, policy (`REQUIRED`, `AUTO`, or `DISABLED`), and explicit
result (`PASSED`, `FAILED`, `SKIPPED`, or `NOT_APPLICABLE`).  Safety monitoring
remains active throughout every step.

Only unit-specific artifacts are persisted: current offsets/gains, per-sensor
direction/LUT/zero/alignment, observer commissioning data when needed, friction,
cogging, and final consistency metadata.  Motor design values, board ratios,
limits, and control recipes remain immutable code configuration.  Persistence
uses versioned per-component records and an atomic A/B commit.

## Migration policy

Migration is behaviour-preserving and phase gated.  Compatibility adapters may
temporarily translate old profiles and ports, but every exception is named in
the architecture checker.  A compatibility adapter cannot become a new public
API.  It is deleted as soon as its consumer moves to the new contract.

Completion requires all of the following:

- Core and Drivers build for the Simulation platform without STM32 headers.
- Current hardware builds in Keil and passes its 20 kHz timing, commissioning,
  CAN, USB, Flash, and closed-loop speed regression checks.
- Host tests cover valid and invalid zero/one/two-angle-sensor products,
  optional temperature zones, and every supported current topology.
- No legacy `Firmware/Application`, `Domain`, `Ports`, `Product`, `Runtime`, or
  old communication implementation remains in the production target.
