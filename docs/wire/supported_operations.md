# Wire supported operations

This document is the support boundary for explicit Wire commit attempts. The
pointer guide does not run backbone generation, model assembly, or scene output.

## Normal drawing fixtures

Supported normal workflows include a new isolated route, extension from the
last generated endpoint, a new independent route after ending a session,
explicit pole endpoint reuse, supported midair branch picks, T/cross generation,
and model-aware multi-bundle generation. A completed route does not retain
transient anchor, preview, or pending support state.

The real WASM regression fixture creates twelve independent model-aware routes,
each with two committed intervals, and verifies preview atomicity and final
scene output. ViewerActions separately exercises repeated Click, Enter, Escape,
and tool-switch sessions.

## Requirement constraints

`RequirementConstraint` is limited to explicit template or topology policy,
such as a bundle template that disallows a requested midair branch. A stale row,
missing model fixture, unresolved saved binding, or session state mismatch is
not a requirement constraint.

## Failure ownership

| Owner | Input condition | Category | Normal draw impact |
|---|---|---|---|
| anchor/request validation | missing target ID, non-finite point, endpoint equal to anchor | `InvalidInput` | shown only for explicit Click or Enter |
| template policy | explicitly disallowed midair node/branch or topology policy | `RequirementConstraint` | use an allowed template or target |
| topology/layout resolver | meaningful topology not implemented | `NotImplemented` | implementation backlog, not invalid input |
| draw session | referenced anchor/version became stale | `StateConflict` | refresh anchor and retry |
| backbone/model assembly | saved binding, row, fixture, or generated endpoint violates an invariant | `InternalError` | supported-path defect |

The former repeated-route failure was an `InternalError`: local generation
context replaced the connected endpoint row frame, after which model assembly
reported inconsistent fixture data. Endpoint context is now preserved, and the
model-aware repeated-route fixture prevents recurrence.

## Session behavior

The first Click accepts an anchor. The next Click commits one interval and keeps
its resolved endpoint as the continuation anchor. Enter commits the displayed
guide and ends the session. Escape discards only transient state. Tool switching
ends the previous transient session. Every primary action returns an explicit
outcome; a drawable click is never silently ignored.
