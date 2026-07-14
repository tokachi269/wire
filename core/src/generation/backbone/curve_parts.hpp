#pragma once

#include "pipeline.hpp"

namespace wire::core::generation::backbone {

[[nodiscard]] VisualCurvePartCache make_visual_curve_parts(
    const CoreState& state, const layout& made, const std::vector<ObjectId>& scope_span_ids = {},
    const curve* built_curves = nullptr);

} // namespace wire::core::generation::backbone
