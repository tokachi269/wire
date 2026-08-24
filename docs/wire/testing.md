# Wire verification

Wire verification follows the repository-wide policy in [`../testing.md`](../testing.md). This document owns only
Wire-specific test families, fixtures, coverage machinery, and diagnostic entry points; it does not redefine the
common verification policy.

## Test families

| Family | Purpose |
|---|---|
| default core tests | public behavior, validation, geometry, and state services |
| backbone acceptance | generation, SavedGraph, bindings, layout/geometry/draw, and update boundaries |
| viewer tests | input policy, coordinate conversion, selection, and representative scene output |
| architecture lint | source layer classification and forbidden dependencies |
| test family lint | registered test source ownership |

The backbone acceptance filter is `backbone`; there is no legacy `bb2` alias.

```powershell
build-vs18-coretests\domains\wire\Debug\wire_core_tests.exe backbone
```

## Fixtures and family manifest

Backbone fixtures live in `domains/wire/tests/backbone/fixtures.*`. Tests do not construct authoritative input from
the v1 topology API, existing span geometry, or positional proximity. Representative scenes verify decision owners
for topology, connectivity, and placement in addition to final output.

`domains/wire/tests/test_family_manifest.json` assigns each registered test source to exactly one family. The lint
fails for unclassified or multiply classified sources. C numbers remain historical identifiers, not progress
metrics.

## Failure diagnostics

New tests use `WIRE_TEST_EXPECT(condition, reason)` for major preconditions and invariants. When an existing test is
touched, migrate the important operation boundaries in that function to reason-bearing assertions. Do not treat a
mechanical suite-wide assertion rewrite as progress.

## Operation-state coverage

Operation-state coverage proves that required semantics cells were actually reached. Common invariants prove state
correctness at each stable observation point rather than being encoded as coverage metadata.

- `Observe`, `ObserveEmpty`, and `ObserveMidspan` check row-frame coherence before recording `(cell, entry)`.
- Core executes every required cell through `core_api`.
- WASM and viewer tests read the `入口境界` table in `backbone_operation_semantics.md` and execute real payloads.
- A case that records only `derived_equality` is not independent evidence. Full core test output lists such cases.

### Canonical backbone acceptance

A successful `CoreState` generation, update, regenerate, or load checks each stable observation point in this order:

```text
canonical successful backbone scenario
  -> WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state)
  -> scenario-specific oracle
```

`WIRE_TEST_EXPECT_BACKBONE_INVARIANTS` is the normal test entry to `backbone_common_invariants_pass()`. It records
the scene-wide minimum as an Anchor. Shape, identity, differential, count, and direction assertions remain in each
scenario as independent oracles.

Representative success scenarios include straight, corner, branch, cross, multi-lane, incremental, regenerate,
save/load, model/socket, midair, and production-like configurations. Observe stable final states, not every
intentional intermediate state. Do not apply the common invariant mechanically to invalid input, rejection,
SourceGuard, pure unit, parser/template-only, or intentional intermediate-state tests.

The current common invariant covers authoritative references, layout/endpoints, row/frame, model/cache, visual
geometry, connection, and high-voltage crossing. Concrete multi-level heights and curve shapes remain owned by
scenario-specific oracles.

## Wire architecture checks

`tools/arch_lint.py` and `tools/arch_manifest.json` check Wire source classification, forbidden private dependencies,
retired source families, domain-identity leakage, operation-state/BOS coverage mapping, and authority guard owners.
Stable source checks remain Structural evidence and do not replace behavior tests.

## Legacy tests

Do not treat a legacy test expectation as the Backbone acceptance source of truth. Extract its contract and classify
it as retained, rewritten against current authority/derived output, retired with an implementation family, or
removed because it conflicts with the current design. Unsupported-operation tests also verify unchanged state;
post-edit tests verify actual derived output updates or rejection before mutation.
