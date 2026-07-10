#pragma once

#include "wire/core/core_state.hpp"

#include <functional>

namespace wire::core::generation::backbone {

using span_layout_endpoint_resolver = std::function<EditResult<Vec3d>(const EndpointLayoutRule&)>;

EditResult<Vec3d> resolve_span_layout_endpoint(const CoreState& state, const EditState& edit_state,
                                                const EndpointLayoutRule& rule);
EditResult<SpanLayoutEntry> derive_span_layout(const SpanLayoutRule& rule,
                                                const span_layout_endpoint_resolver& endpoint_resolver,
                                                std::uint64_t source_version);

} // namespace wire::core::generation::backbone
