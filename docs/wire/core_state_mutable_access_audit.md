# CoreState Mutable Access Audit

Status: frozen audit record. This file records the completed R8 review and is not a living coverage matrix.

R8 keeps mutable access paths classified. The normal rule is that production code mutates authoritative state through `CoreState` operation APIs, and tests use `CoreStateTestHook` for white-box setup.

## Removed from `core_state.hpp`

These private accessors had no production or test call sites and were removed:

- `connection_index_access`
- `relation_index_access`
- `span_runtime_states_access`
- `cache_state_access`
- `next_generation_session_id_access`
- `path_direction_debug_records_access`

## Remaining production friend access

| Storage | Caller | Classification | Reason / follow-up |
|---|---|---|---|
| `authoritative_.edit_state` | `domains/wire/src/generation/backbone/pipeline.cpp` | Production friend implementation detail, audited | The backbone pipeline is a declared `CoreState` friend and mutates ports during `emit_ports`. R8 closes the generic mutable accessor and leaves this explicit friend access as the remaining production bridge. A narrower generation-owned operation is a future refactor, not part of R8. |

No new `*_access()` accessor should be added to `core_state.hpp`. Test-only access belongs in `CoreStateTestHook`.
