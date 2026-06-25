#pragma once

#include "wire/core/core_state.hpp"

namespace wire::core::generation::bb2 {

[[nodiscard]] DetailCurve line(const Vec3d& a, const Vec3d& b);
[[nodiscard]] BoundsCacheEntry bounds(const DetailCurve& curve, std::uint64_t source_version = 0);
[[nodiscard]] SpanRenderCacheEntry render(const CoreState& state, ObjectId span_id, const DetailCurve& detail);
[[nodiscard]] SpanVisualCacheEntry visual(const VisualSettings& settings, const SpanLayoutEntry& layout);

} // namespace wire::core::generation::bb2
