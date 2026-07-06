#pragma once

#include "wire/core/detail_curve.hpp"

namespace wire::core {

class CoreState;

void apply_attachment_line_effects_to_curve(const CoreState& state, ObjectId span_id, DetailCurve* curve);

// Shared helix expansion around a carrier curve, in the carrier's arc-length domain
// [start_s, end_s]. Used by carried wrap sections; the phase advances with arc length so the
// wrap follows the carrier's sag instead of having its own.
[[nodiscard]] std::vector<Vec3d> sample_wrap_helix_points(const DetailCurve& carrier, double start_s,
                                                          double end_s, double radius_m,
                                                          double turns_per_meter, double phase,
                                                          double direction_sign);

} // namespace wire::core
