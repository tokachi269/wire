# Core Model Inventory (Phase4.x Design Audit)

## 1. Scope
- Purpose: lock data responsibilities before adding new features and save/load.
- Priority: clarify boundaries, not add behavior.
- Code baseline: `core/include/wire/core/*.hpp`, `core/src/core_state.cpp`.

## 2. Layer Model (adopted)
```text
Definition Layer
  -> Entity Layer
    -> (read by) Operation/Workflow Layer
    -> (read by) Cache/Derived Layer

Forbidden reverse dependencies:
  Entity -> Workflow, Entity -> Cache, Definition -> Entity
```

## 3. Dependency Rules (fixed)
- Definition types do not reference runtime entity IDs.
- Entity types do not own debug/session logs.
- Cache types are derived only; never treated as source of truth.
- Workflow/debug types may reference entities, but entities must not depend on them.
- Viewer state is never embedded into core entity structs.

## 4. Type Inventory

### 4.1 Definition Layer
| Type | File | Responsibility (one line) | Main deps | Persist class |
|---|---|---|---|---|
| `PortSlotTemplate` | `core/include/wire/core/entities.hpp` | Defines candidate attachment points used to create `Port` instances. | `ConnectionCategory`, `Vec3d`, `Frame3d` | `PersistCore` |
| `AnchorSlotTemplate` | `core/include/wire/core/entities.hpp` | Defines candidate attachment points used to create `Anchor` instances. | `AnchorSupportKind`, `Vec3d` | `PersistCore` |
| `PoleTypeDefinition` | `core/include/wire/core/entities.hpp` | Defines reusable pole template (slot set and metadata). | `PoleTypeId`, slot templates | `PersistCore` |
| `LayoutSettings` | `core/include/wire/core/core_state.hpp` | Defines placement correction policy parameters. | scalar params | `PersistCore` (config) |
| `PathDirectionCostWeights` | `core/include/wire/core/core_state.hpp` | Defines path direction scoring weights. | scalar params | `PersistCore` (config) |
| Core enums (`ConnectionCategory`, `SlotSide`, `SlotRole`, ...) | `core/include/wire/core/entities.hpp` | Define categorical vocabulary for templates/entities/edit operations. | none | `PersistCore` |

### 4.2 Entity Layer
| Type | File | Responsibility (one line) | Main deps | Persist class |
|---|---|---|---|---|
| `Pole` | `core/include/wire/core/entities.hpp` | Stores support structure identity, transform, and template reference. | `Transformd`, `PoleTypeId` | `PersistCore` |
| `Port` | `core/include/wire/core/entities.hpp` | Stores a concrete connectable endpoint in world space including Auto/Manual position mode. | `ObjectId`, `Vec3d`, category/layer, position mode | `PersistCore` |
| `Span` | `core/include/wire/core/entities.hpp` | Stores a concrete connection between two ports. | `port_a_id`, `port_b_id`, `bundle_id` | `PersistCore` |
| `WireGroup` | `core/include/wire/core/entities.hpp` | Stores logical/generated grouping for multiple spans. | kind/tags/session refs | `PersistCore` |
| `WireLane` | `core/include/wire/core/entities.hpp` | Stores one lane identity/order inside a wire group. | `wire_group_id`, `lane_index`, role | `PersistCore` |
| `Anchor` | `core/include/wire/core/entities.hpp` | Stores a concrete support point used by spans. | `owner_pole_id`, `Vec3d` | `PersistCore` |
| `Bundle` | `core/include/wire/core/entities.hpp` | Stores conductor bundle attributes shared across spans. | `conductor_count`, `kind` | `PersistCore` |
| `Attachment` | `core/include/wire/core/entities.hpp` | Stores an item attached to a span at parametric position `t`. | `span_id`, `t` | `PersistCore` |
| `EditState` | `core/include/wire/core/core_state.hpp` | Stores all editable entities in ID-indexed stores. | `ObjectStore<T>` | `PersistCore` |
| `ConnectionIndex` | `core/include/wire/core/core_state.hpp` | Stores connectivity lookup maps for entity relations. | entity IDs | `PersistCore` (rebuildable but authoritative for fast queries) |
| `IdGenerator` | `core/include/wire/core/id.hpp` | Issues monotonic unique object IDs. | none | `PersistCore` |
| `make_display_id` + display ID counters | `core/include/wire/core/id.hpp` / `core_state.hpp` | Generates human-readable IDs by prefix sequence. | prefix + counter | `PersistCore` |

### 4.3 Operation/Workflow Layer
| Type | File | Responsibility (one line) | Main deps | Persist class |
|---|---|---|---|---|
| `RoadSegment` | `core/include/wire/core/entities.hpp` | Carries polyline input used for generation workflows. | `RoadId`, `Vec3d[]` | `SessionDebug` |
| `ConductorGroupSpec` | `core/include/wire/core/entities.hpp` | Describes grouped connection intent for a generation run. | category/count/kind | `SessionDebug` |
| `ConductorLaneId`, `ConductorGroupState` | `core/include/wire/core/entities.hpp` | Holds lane-order state used by grouped generation workflow. | bundle ref + lane order | `SessionDebug` |
| `ChangeSet`, `EditResult<T>` | `core/include/wire/core/core_state.hpp` | Reports externally observable effects of an operation. | entity IDs + error | `SessionDebug` |
| `GenerateSimpleLine*`, `GenerateGroupedLine*` option/result structs | `core/include/wire/core/core_state.hpp` | Encapsulate generation command I/O. | entity IDs + workflow params | `SessionDebug` |
| `GenerateWireGroupFromPath*` option/result structs | `core/include/wire/core/core_state.hpp` | Encapsulate DrawPath-oriented group generation I/O. | polyline/category/lanes + generated IDs | `SessionDebug` |
| `PathDirectionCostBreakdown`, `PathDirectionEvaluationDebug` | `core/include/wire/core/core_state.hpp` | Stores path direction evaluation diagnostics. | scoring values | `SessionDebug` |
| `SegmentLaneAssignment` | `core/include/wire/core/core_state.hpp` | Stores lane mapping diagnostics per generated segment. | poles/ports/slot IDs | `SessionDebug` |
| `SlotCandidateDebug`, `SlotSelectionDebugRecord` | `core/include/wire/core/core_state.hpp` | Stores slot selection diagnostics and score breakdown. | score fields + IDs | `SessionDebug` |
| `ValidationIssue`, `ValidationResult` | `core/include/wire/core/core_state.hpp` | Reports structural validity and diagnostics. | code/message/object_id | `SessionDebug` |

### 4.4 Cache/Derived Layer
| Type | File | Responsibility (one line) | Main deps | Persist class |
|---|---|---|---|---|
| `SpanRuntimeState` | `core/include/wire/core/core_state.hpp` | Tracks dirty flags and version follow state per span. | span ID + versions | `DerivedCache` |
| `DirtyQueue` | `core/include/wire/core/core_state.hpp` | Stores incremental recomputation worklists. | span ID lists | `DerivedCache` |
| `RecalcStats` | `core/include/wire/core/core_state.hpp` | Reports per-frame recomputation counters. | counts | `DerivedCache` |
| `GeometrySettings` | `core/include/wire/core/core_state.hpp` | Defines curve generation parameters for caches. | scalar params | `PersistCore` (config) |
| `CurveCacheEntry`, `CurveCache` | `core/include/wire/core/core_state.hpp` | Stores generated polyline points for spans. | span IDs -> points | `DerivedCache` |
| `BoundsCacheEntry`, `BoundsCache` | `core/include/wire/core/core_state.hpp` | Stores generated span/segment AABBs. | span IDs -> AABBs | `DerivedCache` |
| `CacheState` | `core/include/wire/core/core_state.hpp` | Bundles derived caches plus geometry settings. | cache structs | mixed (`PersistCore` settings + `DerivedCache` data) |

## 5. Mixed Responsibility Findings (split candidates)
| Current type/field | Why mixed | Proposed split direction |
|---|---|---|
| `Pole::context` (`PoleContextInfo`) | Path-analysis hint is generation/workflow data embedded in entity. | Move to workflow table keyed by `pole_id`, keep minimal persisted hint only if needed. |
| `Port` fields `source_slot_id`, `template_layer/side/role`, correction flags | Template provenance + generation diagnostics mixed with concrete endpoint entity. | Introduce `PortPlacementMeta` (persist subset) and `PortPlacementDebug` (session). |
| `GenerationMeta` inside `Pole`/`Span` | Session order and source metadata mixed with durable topology. | Keep `generated/source` if needed; move session/order to workflow history. |
| `CacheState` bundles both settings and derived caches | Config and cache lifetime differ. | Split into `GenerationSettingsState` and `DerivedCacheState`. |
| `ConnectionIndex` classification | Could be rebuilt, but currently part of authoritative editing state. | Keep as entity-adjacent index for now; mark as rebuildable in Phase5 serializer. |

## 6. Naming Clarification (fixed vocabulary)
- `slot`: template candidate point (`PortSlotTemplate`, `AnchorSlotTemplate`), not a runtime connection object.
- `port`: runtime connection endpoint (`Port`), may originate from a template slot.
- `span`: runtime edge connecting two ports.
- `bundle`: conductor attributes shared by one or more spans.
- `wire_group`: logical/generated parent for grouped spans.
- `wire_lane`: per-group lane identity and order.
- `road/path/guide`: workflow input path (`RoadSegment` today, UI label may be `DrawPath`).
- `debug record`: session-only diagnostics (`SlotSelectionDebugRecord`, `PathDirectionEvaluationDebug`, `SegmentLaneAssignment`).

## 7. Persist/Derived/Session Policy (Phase5 preparation)

### 7.1 Default classification
- `PersistCore`: entity stores (`poles`, `ports`, `spans`, `anchors`, `bundles`, `attachments`), template definitions (`pole_types`), settings (`layout`, `geometry`), ID state (`next_id`, display counters).
- `DerivedCache`: curve cache, bounds cache, dirty queues, span runtime versions, recalc stats.
- `SessionDebug`: slot/path/lane debug records, last debug snapshots, operation temporary options/results.

### 7.2 Explicit "do not persist" list (locked for Phase5)
- `CurveCache` / `BoundsCache`
- `DirtyQueue` / `RecalcStats` / `SpanRuntimeState`
- `slot_selection_debug_records_`
- `path_direction_debug_records_`
- `last_path_direction_debug_`
- `last_lane_assignments_`

## 8. Minimal Refactor Applied in this phase
- Added layer boundary comments in core headers to reduce naming ambiguity (`slot` vs `port`).
- Added session/debug separation comments to `CoreState` debug/cache fields.
- Added regression tests that enforce:
  - clearing debug records does not mutate entity state,
  - cache recomputation does not mutate entity identity/counts.

## 9. Ready-to-continue checklist
- Team can consistently read `slot` as template and `port` as entity.
- Debug/session data is treated as non-authoritative.
- Phase5 serializer can ignore DerivedCache and SessionDebug without data loss of core topology.
- WireGroup/WireLane are now in Entity layer and remain responsibility-separated from visual Bundle.
