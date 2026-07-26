#pragma once

#include "model_assembly.hpp"

#include "city/wire/core_state.hpp"

#include <functional>

namespace city::wire::generation::backbone {

using span_layout_endpoint_resolver = std::function<EditResult<Vec3d>(const EndpointLayoutRule&)>;

EditResult<Vec3d> resolve_span_layout_endpoint(const CoreState& state, const EditState& edit_state,
                                                const EndpointLayoutRule& rule,
                                                const FixturePlacementPlanByPort* fixture_plan = nullptr);
EditResult<SpanLayoutEntry> derive_span_layout(const SpanLayoutRule& rule,
                                                const span_layout_endpoint_resolver& endpoint_resolver,
                                                std::uint64_t source_version);

} // namespace city::wire::generation::backbone
