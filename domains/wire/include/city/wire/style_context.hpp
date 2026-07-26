#pragma once

#include "city/wire/workflow_types.hpp"

namespace city::wire {

// Resolve downstream-only realism/style context from stable workflow keys.
[[nodiscard]] ResolvedStyleContext ResolveStyleContext(const ContextProfile& profile, const StyleRouteKey& route_key,
                                                       const StyleObjectKey& object_key);

} // namespace city::wire
