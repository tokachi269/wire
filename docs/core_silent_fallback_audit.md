# Core Silent Fallback Audit

R5 classifies core paths that replace missing or degenerate geometry with a default value. This is not a permission to add more fallback. New fallback-like code must be classified here first, and behavior changes need a separate fail-first test.

## Classification

| Area | Source | Fallback shape | Classification | Reason / next action |
|---|---|---|---|---|
| Pole/layout yaw normalization | `core/include/wire/core/coord_utils.hpp` `HorizontalNormalizedOr` | zero horizontal vector becomes world forward | Upstream inconsistency candidate | Supported backbone rows should have non-zero axes before layout. Existing call sites are kept for compatibility, but new backbone row-axis users should validate before calling. |
| Backbone port placement | `core/src/generation/backbone/emit_shared.cpp` | `HorizontalNormalizedOr(row_axis)` in `PortLayoutYawDeg`, `PortLocalPosition`, `PortWorldPosition` | Upstream inconsistency candidate | A generated backbone edge or row with zero axis is already rejected in pair/row construction in normal paths. If this fallback is reached for supported generation, it indicates a missing earlier guard. |
| Endpoint semantic side axis | `core/src/generation/backbone/pipeline.cpp` | `HorizontalNormalizedOr(source->group_axis)` | Deferred | This is derived from support-group geometry. It should become an internal error if a focused fail-first test proves zero group axis can reach supported output. No behavior change in R5. |
| Model assembly cached plan fallback | `core/src/generation/backbone/model_assembly.cpp` | missing fixture placement plan recomputes endpoint placement and increments instrumentation counters | Upstream inconsistency candidate | Normal generation and reposition paths are guarded by C791/C784 to keep endpoint and row fixture fallback counters at zero. Keep as diagnostic safety only; do not rely on it for supported paths. |
| Span visual frame sampling | `core/src/generation/backbone/span_visual_assembly.cpp` | `unit_or` returns a fixed axis for zero segment/tangent | Legitimate degenerate handling | Visual assembly may sample a one-point or clamped support path while producing finite debug/visual output. Existing geometry tests cover deterministic finite output. |
| Detail curve tangent blend | `core/src/geometry/detail_curve.cpp` | `BlendDirections` returns caller fallback for cancelling tangent blend | Legitimate degenerate handling | This is an explicit quality fallback for acute/conflicting tangents, recorded in `DetailCurveQuality.fallback_iterations` and covered by geometry tests. |
| Detail curve continuity downgrade | `core/src/geometry/detail_curve.cpp` | bounded retry lowers continuity constraints | Legitimate degenerate handling | This is an explicit solver policy with observable quality metadata, not silent topology inference. |
| Curve frame construction | `core/src/geometry/curve/curve.cpp` | `normalized_or` and least-aligned-axis fallback for vertical/zero frame vectors | Legitimate degenerate handling | Cable curve construction must remain finite for vertical spans and degenerate test cases; unsupported method still rejects explicitly. |
| Template port solver overflow | `core/src/state/template/ports.cpp` | constrained fallback port candidate when preferred placement cannot fit | Deferred | This belongs to template-owned manual port solving, outside R5 behavior changes. It is not a backbone continuity fallback, but needs a dedicated template-port contract before being tightened. |
| Template registry spacing | `core/src/state/template/registry.cpp` | default support fanout spacing when cable template lookup fails | Deferred | Default registry construction is allowed to bootstrap built-in templates. Do not reuse this pattern in generated state paths. |

## Guarded today

- `model_assembly.cpp` endpoint and row fixture fallback counters are expected to remain zero in the large-route operation guard.
- Backbone input NaN/inf validation rejects non-finite external coordinates before fallback-heavy geometry code sees them.
- Unsupported backbone inputs return explicit `unsupported` or `invalid input`; they must not fall back to older generation paths.

## Required follow-up before behavior changes

- Convert `HorizontalNormalizedOr` backbone row-axis call sites to explicit validation only after a fail-first scenario shows a supported request reaches them with a zero axis.
- Convert `pipeline.cpp` group-axis fallback to an internal error only after the responsible support-group invariant is isolated.
- Classify template port constrained fallback separately with template-port tests; do not change it under backbone robustness work.
