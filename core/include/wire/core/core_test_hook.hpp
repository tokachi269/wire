#pragma once

#include "wire/core/core_state.hpp"

namespace wire::core {

#if defined(WIRE_INTERNAL) || defined(WIRE_TESTING)
// Test-only backdoor for exceptional white-box checks.
// Production code must use CoreState public APIs only.
struct CoreStateTestHook {
  static EditState& edit_state(CoreState& state) { return state.edit_state_; }
  static CacheState& cache_state(CoreState& state) { return state.cache_state_; }
  static IdGenerator& id_generator(CoreState& state) { return state.id_generator_; }
};
#endif

} // namespace wire::core
