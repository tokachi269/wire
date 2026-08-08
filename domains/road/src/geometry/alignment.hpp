#pragma once

#include "city/road/road.hpp"

// Path arithmetic the road operations share: 2D vectors, cubic Bezier spans,
// splitting a path at a distance, and turning a path into the SegmentShape the
// authoritative graph stores. Nothing here reads or writes the graph.
namespace city::road::internal {

inline constexpr int kCurveSamples = 24;

[[nodiscard]] double distance(Vec2d a, Vec2d b);
[[nodiscard]] bool almost_same(Vec2d a, Vec2d b);

[[nodiscard]] Vec2d span_start(const BezierSpan& span);
[[nodiscard]] Vec2d span_end(const BezierSpan& span);
[[nodiscard]] Vec2d span_eval(const BezierSpan& span, double t);
[[nodiscard]] Vec2d span_derivative(const BezierSpan& span, double t);
[[nodiscard]] double span_length(const BezierSpan& span);
[[nodiscard]] double span_parameter_at_length(const BezierSpan& span,
                                              double target_length);
[[nodiscard]] bool linear_span_controls_match(Vec2d start, Vec2d end,
                                              const BezierSpan& span);
[[nodiscard]] Vec2d path_start(const Path& path);
[[nodiscard]] Vec2d path_end(const Path& path);
[[nodiscard]] Result<bool> validate_no_self_intersection(const Path& path);

struct PathSplit {
  Path before{};
  Path after{};
  Vec2d point{};
};

[[nodiscard]] Result<PathSplit> split_path_at_distance(const Path& path,
                                                       double distance_m);

// Owner-local extents of a manual marking, used when a split or deletion has to
// decide which side keeps it.
[[nodiscard]] std::pair<double, double>
manual_line_distance_bounds(const ManualLineMarking& marking);
[[nodiscard]] std::pair<double, double>
manual_area_distance_bounds(const ManualAreaMarking& marking);
void shift_manual_line_distance(ManualLineMarking& marking, double delta_m);

// Handle shaping for the draw preview. Commit operations preserve the control
// points they receive.
void align_first_span_start(Path& path, Vec2d start);
void align_last_span_end(Path& path, Vec2d end);
[[nodiscard]] Result<SegmentShape> make_linear_shape(Vec2d start, Vec2d end);
void apply_inherited_arc(Vec2d tangent, Vec2d chord, SegmentShape* shape);
[[nodiscard]] std::optional<Vec2d>
corridor_terminal_tangent(const SavedRoadGraph& graph,
                          RoadCorridorId corridor_id,
                          RoadNodeId endpoint_node_id);

} // namespace city::road::internal
