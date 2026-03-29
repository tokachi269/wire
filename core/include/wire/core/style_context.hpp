#pragma once

#include "wire/core/workflow_types.hpp"

namespace wire::core {

// Resolve downstream-only realism/style context from stable workflow keys.
[[nodiscard]] ResolvedStyleContext ResolveStyleContext(const ContextProfile& profile, const StyleRouteKey& route_key,
                                                       const StyleObjectKey& object_key);

} // namespace wire::core
