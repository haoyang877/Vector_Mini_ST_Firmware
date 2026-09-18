# Architecture Map

This is the stable top-level map. Detailed design and historical evidence live under
[`docs/`](docs/README.md); current directory ownership is documented in
[`docs/architecture/code_structure.md`](docs/architecture/code_structure.md).

## Products and contracts

- `firmware/` is the current STM32G4 motor-control APP.
- `loader/` and `host_app/` are independent future products, not libraries linked into APP.
- `shared/` owns only stable cross-product protocol, image, boot, and target-layout contracts.

## Firmware dependency direction

```text
common/types
    ↓
platform API + motor algorithms
    ↓
services + protocol + communication
    ↓
app composition

STM32G4 BSP/ports implement platform API.
CubeMX Core is the generated hardware entry and composition boundary.
```

New dependencies must follow this direction. Existing reverse dependencies are recorded
in the machine-readable architecture debt baseline and may be removed but not expanded.
The architecture checker resolves local quoted includes and rejects new violations.

## Runtime boundaries

- The fast motor loop owns deterministic sensing, FOC, protection, and actuation paths.
- Services expose parameters and telemetry without owning platform peripherals.
- Communication validates wire data at the boundary and invokes explicit services.
- Application code assembles tasks and modes; it must not become a second protocol layer.
- Hardware/HIL tools are outside normal verification and require explicit physical context.

## Verification boundary

`tools/run.py` is the stable command router. `doctor` inspects prerequisites without
hardware access; `verify` emits current, machine-readable evidence under `outputs/runs/`.
Quick and PR profiles are offline. Release may invoke Keil but never flashes a device.
