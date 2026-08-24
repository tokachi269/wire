# Verification policy

This document is the repository-wide, project-independent source of truth for verification policy.
Architecture semantics belong to `docs/architecture.md` and domain architecture documents; operation meaning
belongs to operation semantics documents. Tests, ledgers, manifests, and coverage reports are evidence, not the
semantic source of truth.

Project-specific machinery is documented separately. In this repository, see
[`wire/testing.md`](wire/testing.md) and [`road/testing.md`](road/testing.md).

## Contract-centered verification

The management unit is a contract, not a test case count. Each critical contract should normally have:

- Primary proof: the single most direct proof of the contract
- Secondary proof: one or two alternate production-like paths when they detect a distinct risk
- Structural guard: a dependency, type, compile/link, module, or stable source boundary when behavior alone
  cannot prove the structure

Do not replace a missing Primary proof with many indirect scenarios or SourceGuards. Before adding another case
for the same fault, consider strengthening, replacing, or retiring existing evidence.

## Protection hierarchy

Prefer the strongest mechanism that prevents the relevant fault class:

```text
capability / type / module boundary
  > architecture structural lint
  > behavioral Primary proof
  > metamorphic / differential proof
  > Scenario / End-to-end proof
  > full regression suite
```

This is not a rule that higher layers make all lower layers unnecessary. Keep mechanisms that detect different
fault classes. Evaluate quality by whether every critical contract has independent proof, not by test count or
coverage percentage.

## Evidence classes

| Class | What it proves |
|---|---|
| Structural | dependency, type, compile/link boundary, stable source architecture |
| Invariant / Contract | authoritative state and semantic invariants |
| Metamorphic | meaning preserved across input reversal, operation order, or representation changes |
| Differential | agreement between scoped/full or equivalent entry paths |
| Scenario | an important incident reproducer or production-like configuration |
| End-to-end | adapter, transport, runtime, and user-facing boundary |

SourceGuard is Structural evidence. It does not substitute for behavioral proof.

## Oracle independence

A Primary proof must not compute its expected result by calling the same production decision or helper that it is
checking. That proves self-consistency, not correctness. Prefer an independent formula or oracle, a known anchor, a
metamorphic relation, or a differential implementation/path. Reusing a production helper can still be useful as a
Secondary consistency assertion when the Primary proof has an independent basis.

## Behavioral probe catalog

Select probes from system characteristics and credible fault models. This catalog is not a mandatory checklist or a
test-count quota.

| Characteristic | Representative probes |
|---|---|
| Stateful mutation | failure atomicity; unrelated-state non-interference; deterministic repeat when the operation promises it |
| Persistence | save -> load -> save; derived rebuild equivalence after load; failed load leaves identities, IDs, and counters unchanged |
| Incremental or cached result | incremental result vs clean/full rebuild differential; update followed by stale-cache detection |
| Stable identity | reorder arrays without changing identity; change geometry without changing identity; reject missing identity instead of guessing from proximity, name, or order |
| Independent composition | compare A with the A portion of A+B; permute independent component order |
| Equivalent entries | compare semantic request and result across API, adapter, UI, or transport entries |
| Geometry or directional data | reverse input, symmetry, and permutation relations that preserve the defined meaning |
| Stateful sequence | deterministic seeded operations; invariant at every step; periodic persistence roundtrip; invalid operation leaves state unchanged |

### Pattern selection guide

| Feature characteristic | Consider first |
|---|---|
| authoritative mutation | atomicity and non-interference |
| cache or incremental derivation | full-vs-incremental differential and stale-cache probe |
| stable identity | reorder and geometry-change metamorphic probes |
| persistence | roundtrip, derived rebuild, and failed-load counter stability |
| multiple adapters or entries | semantic entry differential |
| order-independent composition | component permutation and A-vs-A+B |
| directional geometry | reverse, symmetry, or defined permutation relation |
| multi-step state machine | seeded sequence, per-step invariant, periodic roundtrip, invalid-step unchanged |

Choose only probes that correspond to actual contracts and plausible faults. A characteristic can require more than
one probe when the probes have independent oracles.

## Fail-first and diagnostics

For a new scenario or bug fix, first show that the current production behavior violates the contract. Adding a
case that already passes is not regression proof. Assertions should identify the first failed operation boundary
or invariant rather than returning an unexplained boolean.

When a change claims zero behavior change, state the equivalence proof: authoritative byte equality, bit equality,
unchanged behavioral tests, or another contract-specific comparison. A skipped test is not a pass.

## Test effectiveness

Line and branch coverage show execution, not fault detection. Evaluate a verification family by injecting a small,
representative semantic fault and observing whether it fails at a meaningful contract boundary. Syntactic mutation
is useful for finding assertions that prove nothing, but it does not cover state ownership, stale derived output,
partial mutation, transport drift, or silent fallback failures.

Historical incidents are high-value fault models. Reproduce the same semantic fault in current code when practical;
building an old commit is optional. Mutation and semantic fault injection are temporary verification-audit work and
are not normally committed as permanent tests. Store useful investigation history in an audit log, not in the
current contract specification.

If a regression escaped a green suite, classify why: missing invariant, insufficient production fidelity, missing
scenario, weak semantic assertion, condition complexity, implementation-detail assertion, or an old fix trapped in
one fixture. Strengthen the contract owner rather than only adding another symptom case.

## Regression lifecycle

Use this lifecycle for a bug fix:

```text
incident reproducer
  -> root cause
  -> underlying contract
  -> create or strengthen the Primary proof
  -> re-inject the semantic fault
  -> confirm the stronger proof detects it
  -> implement the fix
  -> check the counterfactual and unrelated state
  -> delete or shrink a fully subsumed reproducer
```

Do not default to one permanent test per bug. Retain an incident-specific Scenario only when its input or oracle has
independent value after the underlying contract is directly protected.

When a production fix causes a new failure, first ask whether the previous change can be narrowed or reverted.
Do not make compensating behavior the first response.

## Architecture structural guards

A project architecture manifest may define scan roots, source extensions, exclusions, exactly-one-layer patterns,
and per-layer forbidden tokens. Structural lint protects dependency and ownership boundaries; it must not copy the
project's production semantics into the manifest or claim semantic correctness from token scans alone.

Retire source grep or regex guards when a stronger type system, compile boundary, module dependency, capability, or
architecture lint makes the invalid state unrepresentable. Do not permanently freeze an obsolete implementation
shape.

## Test retirement

An existing test may be removed as verification consolidation only when all conditions hold:

1. Its contract is identified.
2. Another Primary proof exists.
3. The fault caught by the old test can be reproduced.
4. The Primary proof catches that fault.
5. The old test has no independent oracle.

Keep a suspected duplicate when these conditions are not proven. Deletion count is not progress.

## Full suite

Focused proof establishes root cause and the direct contract. The full suite is a secondary safety net for
cross-domain effects, registration gaps, transports, and conflicts with existing contracts. It is not a substitute
for fail-first evidence or a focused Primary proof.

Completion requires the requested behavior, its architecture contract, focused proof, relevant structural checks,
no unexplained compensation, and scope containment. A green full suite alone is insufficient.
