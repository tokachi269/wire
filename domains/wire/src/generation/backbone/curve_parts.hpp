#pragma once

#include "pipeline.hpp"

namespace city::wire::generation::backbone {

[[nodiscard]] EditResult<VisualCurvePartCache> make_visual_curve_parts(
    const CoreState& state, const layout& made, const std::vector<ObjectId>& scope_span_ids = {},
    const curve* built_curves = nullptr);

} // namespace city::wire::generation::backbone
