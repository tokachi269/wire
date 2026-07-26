#pragma once

#include "city/wire/detail_curve.hpp"

#include <optional>

namespace city::wire {

class CoreState;

[[nodiscard]] std::optional<std::pair<Vec3d, Vec3d>> resolve_pole_band_chord_endpoints(
    const CoreState& state, const Span& span, int pole_band_id);

void apply_attachment_line_effects_to_curve(const CoreState& state, ObjectId span_id, DetailCurve* curve);

} // namespace city::wire
