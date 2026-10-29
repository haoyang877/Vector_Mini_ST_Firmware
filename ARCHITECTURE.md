# Architecture Map

This is the stable top-level map. Detailed design and historical evidence live under
[`docs/`](docs/README.md); current directory ownership is documented in
[`docs/architecture/code_structure.md`](docs/architecture/code_structure.md).

For a plain-language walkthrough of products, control flow, scheduling, and data ownership,
read [电机驱动软件架构说明](docs/architecture/software_architecture.md).

新旧板适配对象、允许修改与独立评审项见
[统一框架适配范围草案](docs/architecture/dual_board_adaptation_scope_v1.md)。
该页区分既有用户约束、待讨论变更与验证结果，不将当前工作树自动视为已接受基线。

platform只保留BSP、板级config和原样厂商SDK；项目专用入口装配在projects，台架源码在tests/bench。
BSP公共接口、私有实现与配置的实际对应关系见
[BSP逻辑与文件总图](docs/architecture/bsp_architecture_v1.md)。

2026-10-17 integration adds a deterministic winding-thermal core and optional speed
feedforward without restoring the legacy runtime or wire protocol. APP schedules thermal
updates; motor/protection owns the thermal history and explicit current limits. See the
[integration plan](docs/archive/plans/2026-10-17-delivery-feature-integration-v1.md) and
[schema13 extension](docs/protocols/parameter_schema13_delivery_v1.md) for feature semantics;
current storage writes [schema14](docs/protocols/parameter_schema14_storage_v1.md) and reads valid schema12/13.

## Products and contracts

- `firmware/` is the current STM32G4 motor-control APP.
- `loader/` is the independently built G431 upgrade product.
- `host_app/` contains the Rust GUI candidate and the Qt6 core CAN FD tool for real readback,
  explicit speed/STOP commands, and matched APP-package upgrade through an independent Python client. They open disconnected;
  GUI control and upgrade still require matching hardware acceptance. Neither product is linked into APP.
- `shared/` owns only cross-product protocol, image, boot, target-layout contracts, and the pure CRC32 core.
  Motor code may consume the shared CRC32 contract; shared never depends on firmware or hardware.

## Firmware dependency direction

```text
shared pure contracts + motor/math + motor value types
    ↓
platform API + motor algorithms
    ↓
parameters + app/interfaces (value/owner contracts)
    ↓
protocol + communication
    ↓
app composition

Both targets link the same sampling, PWM, encoder, and CAN BSP implementations.
MCU backends operate peripherals; device modules decode sensor protocols;
its explicit App_Off target links shared APP with a compile-time power lock.
Each board owns projects/<board>/application/main.c with the same entry template.
BSP system.c owns hardware startup; app_startup.c and app_tasks.c own application services;
CubeMX Core retains generated STM32 peripheral and interrupt implementations.
```

New dependencies must follow this direction. The 2026-09-29 closure removes all recorded
reverse dependencies; the machine-readable architecture debt baseline is empty and must not expand.
The architecture checker resolves local quoted includes and rejects new violations.

## Runtime boundaries

- `app/composition/app.h` exposes application lifecycle and task entries: `App_Init`, `App_Start`, `App_Process`,
  `App_FastTick`, and `App_SupervisorTick`. Startup/background assembly lives in
  `app_startup.c`; bounded ADC/TIM scheduling lives in `app_tasks.c`. See the
  [APP reading order](firmware/app/README.md).
- Each board's `application/main.c` calls `Bsp_Init`, `App_Init`, `Bsp_Start`, `App_Start`,
  then `App_Process`. MCU system code owns hardware startup; application composition
  owns protocol binding and service scheduling. There is no AppPlatform forwarding layer.
- The fast motor loop owns deterministic sensing, foc, protection, and actuation paths.
- Parameters own configuration values and record conversion; app/interfaces owns command,
  STOP-confirmation and read-only status contracts without platform peripherals.
- Communication validates wire data at the boundary and invokes explicit services.
- The motor main state machine and its private transition rules form one APP module at
  `firmware/app/motor_state_machine/`; public interfaces carry requests, not a second lifecycle core.
- Communication may include app/interfaces, but never app/runtime or private state-machine headers.
  Motor and parameters never depend on APP implementations; board BSPs implement hardware only.
- Application code assembles tasks and modes; it must not become a second protocol layer.
- Hardware bench tools are outside normal verification and require explicit physical context.

## Embedded boundary contracts

### Hardware independence

- `firmware/platform/bsp/include/` defines the capabilities required by portable firmware. Contracts use
  project-owned types and physical units, and state timing, blocking, ownership, and failure behavior.
- `firmware/platform/config/boards/` holds read-only electrical resources, scaling inputs and measurement limits.
  Sampling scales derive from these data without changing capability contracts; board data must not depend
  on motor, application, or peripheral implementations.
- Board `bsp/` directories own chip and board primitives and directly implement
  platform APIs. `application/main.c` owns the standalone and Loader-linked APP entry;
  `mcu/*/system.c` owns hand-maintained hardware startup. CubeMX owns generated peripheral code.
- `motor/`, `parameters/`, `app/interfaces/`, and `communication/` must not expose STM32 HAL types, peripheral handles,
  registers, pin mappings, or vendor headers. Hardware revision and MCU selection stay below the API
  or in application composition rather than being scattered as feature `#ifdef` branches.
- A platform API models a capability needed by its caller; it is not a one-for-one wrapper around
  every HAL call. Raw encodings and electrical details are converted at the boundary.

### Functional independence

- Motor algorithms prefer deterministic data-in/data-out functions. Platform code performs I/O,
  application code schedules and composes, and parameter code owns configuration conversion.
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
