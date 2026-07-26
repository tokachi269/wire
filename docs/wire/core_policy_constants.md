# Core Policy Constants

T3 classifies constants that are easy to confuse with each other. This file is an audit, not a migration. Do not move or retune these constants as part of T3.

## Physical / numeric identity

| Constant | Source | Classification | Notes |
|---|---|---|---|
| `kPi` / `kTwoPi` / `kRadiansToDegrees` | `coord_utils.hpp`, geometry/backbone sources | Physical / numeric identity | Mathematical conversion constants. Duplication can be cleaned separately, but values are not visual policy. |
| `kLengthToleranceM`, `kLengthSquaredToleranceM2`, `kStrictLengthToleranceM` | `numeric_tolerances.hpp` | Fixed contract | Numeric comparison policy. Values are compatibility-preserving tolerances, not appearance tuning. |
| `kInv` | variation/style random sources | Physical / numeric identity | Converts 53 random bits to a unit double. |

## Fixed contract

| Constant | Source | Classification | Notes |
|---|---|---|---|
| `kDefaultCornerThresholdDeg` | `core_runtime_types.hpp` | Fixed contract | Default layout setting exposed through `LayoutSettings`; tune via settings, not a hidden code tweak. |
| `kMaxCornerSideScale` | `core_runtime_types.hpp` | Fixed contract | Public layout bound. Changing it changes generated geometry policy. |
| `kSharpCornerInteriorAngleMaxDeg` | `row_representation.cpp` | Fixed contract | Defines normal-vs-sharp representation boundary until moved to a user-facing setting. |
| `kRegenerateChangeMask` | `state/template/update.cpp` | Fixed contract | Bitmask contract for template update policy. Not visual. |
| `kMaxDebugRecords` | `state/template/ports.cpp` | Fixed contract | Debug storage cap. Not generation geometry policy. |

## Visual policy

| Constant | Source | Classification | Notes |
|---|---|---|---|
| `kNodePatchHorizontalLengthM` | `curve_parts.cpp` | Visual policy | NodePatch boundary length. Likely cable/category dependent later. |
| `kNodePatchMaxSpanFraction` | `curve_parts.cpp` | Visual policy | Prevents patch boundary from consuming too much of a short span. |
| `kPatchMetersPerSegment` | `curve_parts.cpp` | Visual policy | Tessellation density for patch sampling. |
| `kPatchRadiansPerSegment` | `curve_parts.cpp` | Visual policy | Angular tessellation density for patch sampling. |
| `kNormalizedCatenarySteepness` | `detail_curve.cpp` | Visual policy | Shape approximation for detail curve sag presentation. |
| `kFallbackTangentScale` | `detail_curve.cpp` | Visual policy | Quality fallback handle scale for hard tangents. |

## Tuning candidate

| Constant | Source | Classification | Notes |
|---|---|---|---|
| `kRowHeightSeparationM` | `pipeline.cpp` | Tuning candidate | Support-level vertical separation. User-visible and likely should become layout policy if adjusted often. |
| `kAvoidClearanceM` | `pipeline.cpp` | Tuning candidate | Route avoid detour clearance. Input/constraint policy rather than physical law. |
| `kAutoCollapseDistanceM` | `pipeline.cpp` | Tuning candidate | Automatic near-node collapse distance. User interaction and topology policy. |
| `kG2MinChordLengthM` | `detail_curve.cpp` | Tuning candidate | Minimum length for attempting G2-like detail continuity. |
| `kG2EndpointOffsetRatioLimit` | `detail_curve.cpp` | Tuning candidate | Detail curve quality threshold. |
| `kG2EndpointOffsetMetersLimit` | `detail_curve.cpp` | Tuning candidate | Detail curve quality threshold. |
| `kCandidateAttempts` | `population.cpp` | Tuning candidate | Population search budget. It is deterministic but affects distribution quality. |

## Current decision

- T3 does not relocate constants into config/template fields.
- Visual policy and tuning candidates should not be changed incidentally while fixing topology, model assembly, or persistence bugs.
- If a new constant affects visible geometry, add it to this table in the same change.
