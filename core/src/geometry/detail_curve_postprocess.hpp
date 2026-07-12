#pragma once

#include "wire/core/detail_curve.hpp"

namespace wire::core {

class CoreState;

void apply_attachment_line_effects_to_curve(const CoreState& state, ObjectId span_id, DetailCurve* curve);

} // namespace wire::core
