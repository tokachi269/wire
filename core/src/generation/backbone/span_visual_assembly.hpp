#pragma once

#include "wire/core/core_state.hpp"

#include <unordered_map>

namespace wire::core::generation::backbone {

struct SpanVisualAssemblyEndpoints {
  Vec3d start{};
  Vec3d end{};
};

using SpanVisualAssemblyEndpointMap =
    std::unordered_map<ObjectId, SpanVisualAssemblyEndpoints>;

// Applies visual-only member transforms to base/population sections grouped by logical span.
// Endpoint input comes from the same placed layout consumed by body construction.
// It does not create or modify authoritative objects.
void apply_span_visual_assemblies(const CoreState& state,
                                  const SpanVisualAssemblyEndpointMap& member_endpoints,
                                  VisualCurvePartCache* cache);

} // namespace wire::core::generation::backbone
