# Wire core architecture

## Authority chain

`BackboneSpec`
→ backbone generation
→ `SavedBackboneGraph`
→ pair/open/row
→ `SpanLayoutRules`
→ support group / `SpanLayoutEntry`
→ `DetailCurve` / bounds
→ visual / render cache
→ viewer or export adapter

The owner of each decision is fixed:

- `SavedBackboneGraph`: topology and identity.
- pair/open/row: connectivity.
- support group: placement and lowering offset only.
- layout: world support and endpoint positions.
- geom: curve shape and bounds from layout endpoints.
- draw: abstract visual/render output from layout and geom.
- viewer/inspection: read-only consumers.

Generated span, layout, curve, bounds, visual, port position, or inspection output must not be used to reconstruct topology.

## Update boundary

backbone uses four coarse update kinds:

- `kRegenerate`: topology or identity may change.
- `kReposition`: layout → geom → draw.
- `kReshape`: geom → draw.
- `kRedraw`: visual/render only.

Operation-specific update kinds are not added. The old `DirtyBits` marker family is deleted; updates use `UpdatePlan`, direct derive, or pre-mutation rejection.

Generation and update timing are diagnostic snapshots only. They do not affect decisions.

## Wire-domain boundary

Wire core accepts resolved world points and wire-specific specs/templates. It does not include road, rail, building, terrain, city, UE, Blender, or viewer domain headers.

External systems choose world positions and wire templates before calling core. Stable external identity, when introduced, must be opaque to wire and paired with a fallback world position. `SavedBackboneGraph` remains wire topology, not city topology.

Current wire profiles are `ContextProfile`, `CableTemplate`, `BundleTemplate`, pole definitions, layout settings, and visual settings. External adapters may choose them, but city-domain semantics do not enter core state.

## Render/export boundary

The current backend-neutral output is:

- `DetailCurve` and bounds for wire geometry.
- `SpanVisualCacheEntry` for abstract support primitives.
- `SpanRenderCacheEntry` for radius, color, material style, and curve-distance attributes.

Viewer and future export/engine adapters consume these outputs. Geometry does not know mesh assets. Future asset/material/profile references must be opaque keys resolved by the backend; adding a full render engine to core is out of scope.

## Guardrails

- Public API: `core/include/wire/core/**`.
- Private backbone scratch: `core/src/generation/backbone/**`.
- Writes go through `CoreState` operations; reads use `CoreView` or const queries.
- No global mutable `CoreState`, SavedGraph, generation context, cache manager, validation manager, or settings singleton.
- `tools/arch_lint.py` fails on unclassified source/header files, forbidden dependency directions, old recalc/support-layout symbols, and city-domain identity types.
- `tools/test_family_lint.py` fails when registered tests have no family owner.

Old recalc, support-layout authority/seed/projection/materialization, grouped span generation, and span-derived backbone reconstruction stay deleted.
