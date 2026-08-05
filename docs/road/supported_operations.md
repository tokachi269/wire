# Road supported operations

This document is the support boundary for explicit Road commit attempts. Pointer
movement only updates a lightweight guide and does not classify a commit.

## Normal drawing fixtures

A road is drawn with a registered cross section. Core ships none: the catalogue
of sections a user can pick lives in `web/src/road_templates.ts`, and a new
workspace registers it through `AddRoadLayoutTemplate` and keeps the IDs Core
returned. Reopening a saved workspace uses the sections in the archive and does
not register the catalogue again.

The following are normal supported fixtures and must not be rejected as product
requirements: an isolated straight or Bezier road, repeated independent roads,
a degree-two pass-through or corner, and supported degree-three/four junctions
including T and cross layouts. Endpoint extension uses the explicit endpoint ID;
segment branching uses the explicit segment ID and segment distance.

The regression suite covers repeated straight and Bezier sessions, Enter,
Escape, tool switching, retry after a rejected commit, endpoint extension, T and
cross generation, angled junctions, and real WASM scene updates.

## Requirement constraints

`RequirementConstraint` is reserved for a documented product rule. Road
currently has one such rule: a newly submitted path may not self-intersect
(`road_path_self_intersection`). A fixed angle range, insufficient resolver
coverage, or a geometry implementation limit is not a requirement constraint.

## Failure ownership

| Owner | Input condition | Category | Normal draw impact |
|---|---|---|---|
| operation preflight (`road.cpp`) | missing ID, non-finite input, zero-length span, malformed request | `InvalidInput` | shown only after Click or Enter |
| operation preflight (`road.cpp`) | documented path self-intersection | `RequirementConstraint` | user must change the submitted path |
| connection resolution (`generation/connections.cpp`) | valid topology outside current resolver coverage, insufficient supported setback, unsupported section combination | `NotImplemented` | bug/coverage backlog, not user input error |
| segment/section/marking resolution | valid operation whose transition or marking merge/split is not implemented | `NotImplemented` | operation remains atomic and retryable |
| generation and geometry | a valid authoritative reference or required derived value is missing | `InternalError` | supported-path defect |
| persistence | duplicate/missing/unknown fields, invalid IDs or non-finite values | `InvalidInput` | load is rejected before state replacement |
| draw session | an explicit request references state changed after anchoring | `StateConflict` | refresh anchor and retry; no current synchronous Road path emits this category |

Every failed commit has a non-empty reason code. The UI must display
`NotImplemented` as missing implementation, not as invalid input. Internal
invariant failures must not be converted to a requirement constraint.

## Unsupported families

Current implementation limits include splits inside a section transition, connection
layouts outside the implemented section combinations, and unresolved marking
merge/split cases. These remain `NotImplemented` until their semantic operation
and geometry resolver are implemented. No nearest approach, nearest boundary,
or fallback shape is selected.

Splits before or after a ratio-based section transition are supported and
re-normalize the transition `t` onto the segment that retains it. A split inside
the transition is rejected with a specific reason and leaves state unchanged.

Lane Branch and Merge are not available. Their editor asked the user to pick a
raw boundary ID and auto-selected a default, so the operation was removed rather
than hidden. Lane connections inside a junction are still derived by Core. See
`backlog.md` for the conditions under which branch and merge return.

## Session behavior

Click and Enter return an explicit draw action outcome. A rejected commit keeps
the anchor and guide. Pointer movement does not replace the last commit failure.
Enter with an anchor but no guide ends the transient session explicitly; Enter
with no active session returns `ignored/session-inactive`.
