#pragma once

#include "wire/core/core_state.hpp"

namespace wire::core::generation::backbone {

// Applies visual-only member transforms to base/population sections grouped by logical span.
// It does not create or modify authoritative objects.
void apply_span_visual_assemblies(const CoreState& state, VisualCurvePartCache* cache);

} // namespace wire::core::generation::backbone
