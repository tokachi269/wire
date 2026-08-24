# Portable Agent Engineering Harness

This playbook is a project-independent workflow for agent-assisted engineering. It supplies boundary and
verification discipline; project architecture documents continue to own product semantics.

## New concept protocol

Before implementation, record only the decisions that already exist:

```text
Decision
  -> Owner
  -> Inputs
  -> Authoritative output
  -> downstream consumers
  -> failure owner
  -> transaction boundary
```

One Decision has one Owner. Consumers use the authoritative output rather than repeating the decision from the
same inputs. Do not design abstractions for concepts that do not yet exist.

For each contract, choose protection in this order:

1. Can the invalid state be structurally unrepresentable?
2. Can a type, module, or capability prevent it?
3. Can architecture lint protect the stable boundary?
4. Is a behavioral Primary proof required?
5. Is there a meaningful metamorphic relation?
6. Is there an equivalent path for differential proof?
7. Is a user-facing Scenario or End-to-end proof required?

Different fault classes can justify evidence at multiple levels. A token scan does not prove behavior, and an E2E
scenario does not prove dependency ownership. Select behavioral probes from the system characteristics catalog in
`docs/testing.md`; do not turn the catalog into a quota.

## Guardrail Promotion Ladder

Do not accumulate defenses forever. Promote them when the underlying contract becomes clearer:

```text
incident-specific assertion
  -> generalized behavioral contract
  -> architecture lint
  -> type / module / capability boundary
```

After promotion, re-inject the original fault. If the stronger mechanism detects or prevents it and the weaker
guard has no independent oracle, evaluate the weaker test or guard for retirement.

## Bug fix protocol

```text
fail-first
  -> incident reproducer
  -> root cause
  -> underlying contract
  -> stronger Primary proof
  -> semantic fault injection
  -> minimal causal fix
  -> counterfactual and unrelated-state check
  -> reproducer retirement evaluation
```

Every production edit must be explainable as root cause, why the edit is necessary, and which focused proof fails
without it. If a new failure appears after the fix, first narrow or revert the previous change before adding
compensating behavior. Do not weaken an existing contract merely to make implementation pass.

## Independent verification

The implementer's statement that tests pass is not completion evidence. When practical, use a separate context or
agent to audit:

- the complete diff and architecture boundaries
- causal necessity of each production edit
- the counterfactual or injected fault
- unrelated changes and scope expansion
- weakened, deleted, or implementation-coupled tests

An independent audit reports evidence and uncertainty; it does not replace focused proof.

## Stop rule

A green full suite alone is insufficient. Stop only when evidence proves:

- requested behavior
- architecture contract and Decision owner
- focused Primary proof
- relevant structural checks
- no unexplained compensation
- scope containment

Stop at the requested boundary. Do not continue into cleanup, adjacent audits, or speculative frameworks.

## Reusable task templates

Use these as small decision records, not as a completion checklist game. Remove a field only when it genuinely does
not apply; do not fill it with invented detail.

### New concept / feature

```text
Goal:
Decision owners:
Root cause / hypothesis: user need and unresolved design assumptions
In scope:
Out of scope:
Forbidden compensations:
Expected production shape:
Primary proof:
Structural proof:
Counterfactual: invalid owner/path or feature-disabled behavior
Stop condition:
```

### Bug fix / regression

```text
Goal:
Decision owners:
Root cause / hypothesis:
In scope:
Out of scope:
Forbidden compensations: fallback, silent success, unrelated rebuild, test weakening
Expected production shape: minimal causal edit
Primary proof: fail before, pass after
Structural proof:
Counterfactual: re-injected semantic fault is detected
Stop condition:
```

### Architecture refactor

```text
Goal:
Decision owners:
Root cause / hypothesis: duplicated authority or unenforced boundary
In scope:
Out of scope:
Forbidden compensations: behavior change, compatibility layer, duplicated specification
Expected production shape: one owner and explicit consumers
Primary proof: behavior equivalence appropriate to the contract
Structural proof: invalid dependency or duplicate owner is rejected
Counterfactual: temporary boundary violation is detected
Stop condition:
```

## New project bootstrap

Start with the smallest harness that protects real decisions:

- a short `AGENTS.md` that maps to canonical documents and commands
- `docs/architecture.md` for state layers, identity, ownership, dependencies, and transactions
- `docs/testing.md` for contract-centered verification policy
- a domain architecture document when a domain first appears
- an architecture manifest plus the generic architecture lint engine
- one representative End-to-end path
- focused Primary proofs for critical contracts
- formatter, lint, build, and test commands
- an independent review or audit path

Do not design the complete future architecture at bootstrap. Add boundaries only for Decisions and dependencies that
exist. Keep project semantics in project architecture, not in the portable harness or manifest.

## Goodhart protection

The harness is not a score. Do not mass-annotate tests, duplicate every contract into JSON/YAML, set test-count or
coverage thresholds as quality targets, claim semantics from source tokens, or treat completed template fields as
success. Measure whether critical contracts have independent fault-detecting evidence and whether invalid ownership
is prevented at the strongest practical boundary.
