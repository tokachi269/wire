#pragma once

#include "city/wire/core_state.hpp"

#include <optional>

namespace city::wire::generation::backbone {

[[nodiscard]] EditResult<DetailCurve> make_curve(const CoreState& state, ObjectId span_id,
                                                 const SpanLayoutEntry& layout);
[[nodiscard]] EditResult<DetailCurve> make_curve_between(const CoreState& state, ObjectId span_id,
                                                         const Vec3d& start, const Vec3d& end);
[[nodiscard]] EditResult<DetailCurve> make_primary_curve_between(
    const CoreState& state, ObjectId span_id, const Vec3d& start, const Vec3d& end,
    const Vec3d* start_tangent = nullptr, const Vec3d* end_tangent = nullptr);
[[nodiscard]] std::optional<Vec3d> source_edge_projection_world(const CoreState& state,
                                                                const SourceEdgeProjectionRef& ref);
[[nodiscard]] const SavedBackboneSpanBinding* source_span_binding_for(
    const CoreState& state, const SourceEdgeProjectionRef& ref);
[[nodiscard]] BoundsCacheEntry bounds(const DetailCurve& curve, std::uint64_t source_version = 0);
[[nodiscard]] SpanRenderCacheEntry render(const CoreState& state, ObjectId span_id, const DetailCurve& detail);
[[nodiscard]] SpanVisualCacheEntry visual(const VisualSettings& settings, const SpanLayoutEntry& layout);

} // namespace city::wire::generation::backbone
