#pragma once

#include "wire/core/core_state.hpp"

namespace wire::core::generation::backbone {

[[nodiscard]] EditResult<DetailCurve> make_curve(const CoreState& state, ObjectId span_id,
                                                 const SpanLayoutEntry& layout);
[[nodiscard]] EditResult<DetailCurve> make_curve_between(const CoreState& state, ObjectId span_id,
                                                         const Vec3d& start, const Vec3d& end);
[[nodiscard]] BoundsCacheEntry bounds(const DetailCurve& curve, std::uint64_t source_version = 0);
[[nodiscard]] SpanRenderCacheEntry render(const CoreState& state, ObjectId span_id, const DetailCurve& detail);
[[nodiscard]] SpanVisualCacheEntry visual(const VisualSettings& settings, const SpanLayoutEntry& layout);

} // namespace wire::core::generation::backbone
