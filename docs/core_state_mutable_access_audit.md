# CoreState Mutable Access Audit

R8 keeps mutable access paths classified. The normal rule is that production code mutates authoritative state through `CoreState` operation APIs, and tests use `CoreStateTestHook` for white-box setup.

## Removed from `core_state.hpp`

These private accessors had no production or test call sites and were removed:

- `connection_index_access`
- `relation_index_access`
- `span_runtime_states_access`
- `cache_state_access`
- `next_generation_session_id_access`
- `path_direction_debug_records_access`

## Remaining production accessor

| Accessor | Caller | Classification | Reason / follow-up |
|---|---|---|---|
| `edit_state_access` | `core/src/generation/backbone/pipeline.cpp` | Production bridge, audited | The backbone pipeline currently runs as a `CoreState` friend-like implementation detail and needs direct mutation during `emit_ports`. Removing it requires moving that mutation behind a narrower generation-owned operation, which is a behavior-risking refactor and is not part of R8. |

No new `*_access()` accessor should be added to `core_state.hpp` without extending this audit and a source-scan guard. Test-only access belongs in `CoreStateTestHook`.
