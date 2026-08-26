#pragma once

#include "city/wire/core_state.hpp"

#include <unordered_map>

namespace city::wire::generation::backbone {

struct SpanVisualAssemblyEndpoints {
  Vec3d start{};
  Vec3d end{};
};

using SpanVisualAssemblyEndpointMap =
    std::unordered_map<ObjectId, SpanVisualAssemblyEndpoints>;

// Resolves center-path variation and visual members from each source logical span.
// Endpoint input comes from the same placed layout consumed by body construction.
// It does not create or modify authoritative objects.
void apply_span_visual_assemblies(const CoreState& state,
                                  const SpanVisualAssemblyEndpointMap& member_endpoints,
                                  VisualCurvePartCache* cache);

} // namespace city::wire::generation::backbone
