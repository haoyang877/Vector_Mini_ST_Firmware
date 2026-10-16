# Architecture Map

This is the stable top-level map. Detailed design and historical evidence live under
[`docs/`](docs/README.md); current directory ownership is documented in
[`docs/architecture/code_structure.md`](docs/architecture/code_structure.md).

For a plain-language walkthrough of products, control flow, scheduling, and data ownership,
read [电机驱动软件架构说明](docs/architecture/software_architecture.md).

## Products and contracts

- `firmware/` is the current STM32G4 motor-control APP.
- `loader/` is the independently built G431 upgrade product.
- `host_app/` is a single-page Qt6 CAN FD tool for real readback, explicit speed/STOP commands,
  and matched APP-package upgrade through an independent Python client. It opens disconnected;
  GUI control and upgrade still require matching hardware acceptance. Neither product is linked into APP.
- `shared/` owns only cross-product protocol, image, boot, target-layout contracts, and the pure CRC32 core.
  Motor code may consume the shared CRC32 contract; shared never depends on firmware or hardware.

## Firmware dependency direction

```text
shared pure contracts + motor/math + motor value types
    ↓
platform API + motor algorithms
    ↓
services + protocol + communication
    ↓
app composition

STM32G4 BSP/ports implement platform API.
CubeMX Core is the generated hardware entry and composition boundary.
```

New dependencies must follow this direction. The 2026-09-29 closure removes all recorded
reverse dependencies; the machine-readable architecture debt baseline is empty and must not expand.
The architecture checker resolves local quoted includes and rejects new violations.

## Runtime boundaries

- The fast motor loop owns deterministic sensing, foc, protection, and actuation paths.
- Services expose parameters and telemetry without owning platform peripherals.
- Communication validates wire data at the boundary and invokes explicit services.
- The motor main state machine and its private transition rules form one APP module at
  `firmware/app/motor_state_machine/`; services publish requests, not a second lifecycle core.
- Application code assembles tasks and modes; it must not become a second protocol layer.
- Hardware bench tools are outside normal verification and require explicit physical context.

## Embedded boundary contracts

### Hardware independence

- `firmware/platform/api/` defines the capabilities required by portable firmware. Contracts use
  project-owned types and physical units, and state timing, blocking, ownership, and failure behavior.
- `firmware/platform/stm32g4/bsp/` owns chip and board primitives. `ports/` adapts those primitives to
  platform APIs. CubeMX owns generated startup and peripheral composition.
- `motor/`, `services/`, and `communication/` must not expose STM32 HAL types, peripheral handles,
  registers, pin mappings, or vendor headers. Hardware revision and MCU selection stay below the API
  or in application composition rather than being scattered as feature `#ifdef` branches.
- A platform API models a capability needed by its caller; it is not a one-for-one wrapper around
  every HAL call. Raw encodings and electrical details are converted at the boundary.

### Functional independence

- Motor algorithms prefer deterministic data-in/data-out functions. Platform code performs I/O,
  application code schedules and composes, and services own policy or persistence.
- Mutable state has one owning module. Other modules use commands, immutable snapshots, or an
  explicitly passed state object instead of writing another module's globals.
- APP owns one fixed maintenance workspace for parameter records and encoder
  calibration samples. Algorithms borrow explicit typed objects; no firmware heap allocator remains.
- Interrupt handlers acknowledge hardware, capture bounded inputs, and perform required immediate
  safety actions; encoding, logging, storage, and unbounded work remain outside the ISR.
- Callbacks are used only when control must genuinely be inverted. Their context, lifetime,
  reentrancy, ISR eligibility, and ownership are part of the public contract.

### Substitution evidence

A hardware boundary is complete only when the portable behavior can be exercised through a native
fake/test seam or an explicitly scoped hardware check. Substitution must preserve units, timing and error
semantics; merely compiling a second implementation is not sufficient evidence.

These rules adapt the layered hardware independence of
[AUTOSAR Classic](https://www.autosar.org/standards/classic-platform), the uniform peripheral
contracts of [CMSIS-Driver](https://arm-software.github.io/CMSIS_6/main/Driver/index.html), and the
[Zephyr device model](https://docs.zephyrproject.org/latest/kernel/drivers/index.html) without
importing their framework complexity. Interface and state rules also follow the
[C++ Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines).

## Verification boundary

`tools/run.py` is the stable command router. `doctor` inspects prerequisites without
hardware access; `verify` emits current, machine-readable evidence under `outputs/runs/`.
Quick and PR profiles are offline. Release may invoke Keil but never flashes a device.
