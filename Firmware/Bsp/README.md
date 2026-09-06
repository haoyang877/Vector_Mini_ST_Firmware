# BSP skeleton

`Api/` defines small C contracts for motor drive, angle sensing, temperature,
communication, clocks, critical sections, identity, storage, and reset. These
headers use only standard C types and never expose a vendor handle or interrupt
type.

`Boards/` describes physical endpoints and capabilities. `VectorMiniSt` is data
only: it declares one three-shunt motor endpoint, two angle-sensor slots, an
available processor-temperature source, an unpopulated power-stage temperature
option, and two communication endpoints.

Composition remains the only place that may combine ProductConfig, a board
descriptor, device adapters, and concrete ports. It projects ProductConfig into
the bounded `BspBoardBindingRequest` solely for validation; BSP metadata is not
another configuration source.

Motor-loop calls are explicitly non-blocking. A motor cycle commits normalized
PWM outputs and its current-sampling plan atomically. The BSP returns raw
per-window current samples; topology-specific phase reconstruction belongs to a
measurement strategy above this boundary.
