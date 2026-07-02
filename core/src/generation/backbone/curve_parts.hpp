#pragma once

#include "pipeline.hpp"

namespace wire::core::generation::backbone {

[[nodiscard]] VisualCurvePartCache make_visual_curve_parts(const CoreState& state, const layout& made);

} // namespace wire::core::generation::backbone
