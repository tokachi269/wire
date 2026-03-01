# Core Model Architecture

## Purpose
This document fixes the architecture rules used by `core` so feature growth does not reintroduce hidden mutation paths or layer violations.

## Layer Model

1. `Definition`
- Template/static definitions.
- Example: `PoleTypeDefinition`, `PortSlotTemplate`, `AnchorSlotTemplate`.

2. `Entity` (authoritative persist core)
- Runtime authoritative network objects.
- Example: `Pole`, `Port`, `Anchor`, `WireGroup`, `WireLane`, `Span`, `Attachment`, `Bundle`.

3. `Workflow` (operation input/output, non-authoritative)
- Generation/edit requests and transient planning structures.
- Example: `RoadSegment`, `GuidePath`, `GenerationRequest`, `ConductorGroupSpec`.

4. `Cache/Debug` (derived/session)
- Rebuildable cache and diagnostics.
- Example: `CurveCache`, `BoundsCache`, `DirtyQueue`, `SpanRuntimeState`, path/slot debug records.

## Dependency Rules (fixed)

- `Definition` must not depend on `Entity`.
- `Entity` must not depend on `Workflow` or `Debug`.
- `Workflow` may read `Definition` and `Entity`, but must not mutate entity state directly.
- `Cache/Debug` may read `Entity`, but must not become authoritative.
- Viewer/tool state never belongs to entity model.

## Mutation Rules (fixed)

- Authoritative updates must go through `CoreState` edit APIs.
- External mutable references to internal stores are prohibited.
- `ObjectStore` mutable backing (`items_mutable`) is restricted to `CoreState` internals.
- Direct store mutation in viewer/tools is prohibited by design.

## Invariants (minimum required)

1. Reference integrity
- `Span.port_a_id`/`port_b_id` point to existing ports.
- `Port.owner_pole_id` points to existing pole when set.
- `Span.anchor_*` point to existing anchors when set.

2. Group/lane integrity
- If `Span.wire_lane_id` is set, `Span.wire_group_id` must be set.
- `WireLane.wire_group_id` must exist.
- Lane index must be unique per group.

3. Index/cache integrity
- `ConnectionIndex` must match expected Span relations.
- `SpanRuntimeState` exists for every span and no dangling runtime remains.

4. Manual/Auto precedence
- `Pole.placement_mode=Manual` means user-intent placement (regeneration should respect it).
- `Port.position_mode=Manual` means position is not auto-overwritten.
- `Auto` state is eligible for regeneration/reprojection.

## Edit Priority Rules

- Manual user edits take precedence over auto generation.
- Regeneration targets auto parts first; manual parts are preserved by default.
- Dirty propagation is local (connected spans/anchors), not global.

## Naming Rules

- `slot`: template candidate position, not runtime endpoint.
- `port`: runtime connection endpoint.
- `span`: runtime edge between ports.
- `bundle`: visual/attribute grouping aid, not logical network authority.
- `wire_group`/`wire_lane`: logical grouped wiring authority above spans.

