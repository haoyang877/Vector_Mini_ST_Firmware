# ADR 0001: Board Bootstrap Composition Root and Forward-Only Migration

- Status: Accepted
- Date: 2026-09-06

## Context

The target firmware tree has no top-level `Composition` directory, but one
module must select the concrete product, board mapping, portable drivers, and
MCU Platform adapters. Placing that module in `Core/Application` would make
Core depend on Platform. Treating all of `Bsp/Boards` as composition would let
ordinary board descriptors depend on every Core subsystem.

Migration also has to proceed in small, buildable changes. The earlier
dependency table allowed legacy layers to reference only other legacy layers.
Moving one dependency to its target owner therefore created a temporary
forbidden edge and encouraged a high-risk all-at-once directory move.

Finally, `Core/Config` must identify explicit products and component models.
Rejecting every concrete product, board, motor, or sensor name forced catalog
entries to use ambiguous names such as `Current`, while including a concrete
driver or MCU implementation in Config must remain forbidden.

## Decision

1. Each board's production composition root lives at
   `Firmware/Bsp/Boards/<board>/Bootstrap/` and is classified as the distinct
   `CompositionRoot` architecture layer before the general `BspBoards` rule.
2. `CompositionRoot` is the only target layer allowed to depend on all target
   layers. Ordinary `BspBoards` retain their narrow dependency set. No target
   layer may depend on `CompositionRoot` or on a legacy layer.
3. While `Firmware/Composition` exists it remains the active legacy
   composition root. After it is removed, the architecture gate requires at
   least one board Bootstrap containing a C implementation. Multiple boards
   may coexist in the repository, but a concrete build target must list exactly
   one Bootstrap; that selection must be checked by the build project/manifest.
4. Legacy layers may depend on the target layer that will own a migrated
   contract or implementation. These forward-only edges are not new debt.
   Target-to-legacy edges remain forbidden, and existing legacy-to-legacy
   exceptions keep their frozen occurrence ceilings.
5. `Core/Config` may use explicit product, board, motor, load, and sensor model
   names as immutable catalog data. It may not contain MCU SDK/HAL/register
   dependencies, and the dependency graph prevents it from including Drivers,
   Platform, or BspBoards.
6. Portable Drivers may depend on their own code and generic `Bsp/Api` bus or
   resource contracts only. They may not depend on `Core/Config`; product
   selection and driver instantiation belong to board binding and Bootstrap.

## Consequences

- Source migration can be committed and verified one ownership boundary at a
  time without temporarily allowing a target layer to point backward.
- Product catalogs can use stable, searchable variant names rather than a
  mutable `Current` identity.
- A board Bootstrap is intentionally powerful and therefore unique within one
  build target. Its directory is reviewed as the only place in that target
  that may assemble the complete `BspDeviceSet` and selected `ProductConfig`.
- A second board or simulation target can add its own Bootstrap without
  weakening ordinary `BspBoards` dependencies.
- Moving the legacy composition root is not complete until all of its legacy
  dependencies have first moved to target layers.

## Rejected alternatives

- A top-level `Firmware/Composition` was rejected because it would add a fifth
  permanent top-level architecture area outside the agreed target tree.
- `Core/Application` was rejected because it cannot legally depend on Platform
  or concrete board and driver implementations.
- Broadening all `BspBoards` dependencies was rejected because capability and
  endpoint descriptors must stay independently testable and free of Core
  orchestration dependencies.
