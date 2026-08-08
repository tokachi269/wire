#include "alignment.hpp"

#include "geometry.hpp"
#include "../lookup.hpp"

#include <algorithm>
#include <cmath>

namespace city::road {
namespace internal {

[[nodiscard]] double distance(Vec2d a, Vec2d b) {
  return magnitude(subtract(a, b));
}

[[nodiscard]] bool almost_same(Vec2d a, Vec2d b) {
  return distance(a, b) <= 1e-6;
}

[[nodiscard]] bool linear_span_controls_match(Vec2d start, Vec2d end,
                                              const BezierSpan& span) {
  const Vec2d delta = subtract(end, start);
  return almost_same(span.p0, start) &&
         almost_same(span.p1, add(start, scale(delta, 1.0 / 3.0))) &&
         almost_same(span.p2, add(start, scale(delta, 2.0 / 3.0))) &&
         almost_same(span.p3, end);
}

[[nodiscard]] Vec2d span_start(const BezierSpan& span) {
  return span.p0;
}

[[nodiscard]] Vec2d span_end(const BezierSpan& span) {
  return span.p3;
}

[[nodiscard]] Vec2d span_eval(const BezierSpan& span, double t) {
  t = std::clamp(t, 0.0, 1.0);
  const double u = 1.0 - t;
  return add(add(scale(span.p0, u * u * u), scale(span.p1, 3.0 * u * u * t)),
             add(scale(span.p2, 3.0 * u * t * t), scale(span.p3, t * t * t)));
}

[[nodiscard]] Vec2d span_derivative(const BezierSpan& span, double t) {
  t = std::clamp(t, 0.0, 1.0);
  const double u = 1.0 - t;
  return scale(
      add(add(scale(subtract(span.p1, span.p0), u * u),
              scale(subtract(span.p2, span.p1), 2.0 * u * t)),
          scale(subtract(span.p3, span.p2), t * t)),
      3.0);
}

[[nodiscard]] double span_speed(const BezierSpan& span, double t) {
  const double u = 1.0 - t;
  const Vec2d a = subtract(span.p1, span.p0);
  const Vec2d b = subtract(span.p2, span.p1);
  const Vec2d c = subtract(span.p3, span.p2);
  const Vec2d derivative =
      scale(add(add(scale(a, u * u), scale(b, 2.0 * u * t)),
              scale(c, t * t)),
          3.0);
  return magnitude(derivative);
}

[[nodiscard]] double span_length_between(const BezierSpan& span, double a,
                                         double b) {
  if (b <= a) return 0.0;
  constexpr std::array<double, 8> nodes{
      0.09501250983763744, 0.2816035507792589, 0.4580167776572274,
      0.6178762444026438,  0.755404408355003,  0.8656312023878318,
      0.9445750230732326,  0.9894009349916499};
  constexpr std::array<double, 8> weights{
      0.1894506104550685,  0.1826034150449236,  0.16915651939500254,
      0.14959598881657673, 0.12462897125553388, 0.09515851168249278,
      0.06225352393864789, 0.027152459411754096};
  const double midpoint = (a + b) * 0.5;
  const double half = (b - a) * 0.5;
  double sum = 0.0;
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const double offset = half * nodes[index];
    sum += weights[index] *
           (span_speed(span, midpoint - offset) +
            span_speed(span, midpoint + offset));
  }
  return half * sum;
}

[[nodiscard]] double span_length(const BezierSpan& span) {
  return span_length_between(span, 0.0, 1.0);
}

[[nodiscard]] double span_parameter_at_length(
    const BezierSpan& span, double distance_along_span_m) {
  if (distance_along_span_m <= 0.0) return 0.0;
  const double total = span_length(span);
  if (distance_along_span_m >= total) return 1.0;
  double low = 0.0;
  double high = 1.0;
  for (int iteration = 0; iteration < 40; ++iteration) {
    const double middle = (low + high) * 0.5;
    if (span_length_between(span, 0.0, middle) <
        distance_along_span_m) {
      low = middle;
    } else {
      high = middle;
    }
  }
  return (low + high) * 0.5;
}

[[nodiscard]] std::pair<BezierSpan, BezierSpan> split_span(const BezierSpan& span, double t) {
  const Vec2d p01 = add(scale(span.p0, 1.0 - t), scale(span.p1, t));
  const Vec2d p12 = add(scale(span.p1, 1.0 - t), scale(span.p2, t));
  const Vec2d p23 = add(scale(span.p2, 1.0 - t), scale(span.p3, t));
  const Vec2d p012 = add(scale(p01, 1.0 - t), scale(p12, t));
  const Vec2d p123 = add(scale(p12, 1.0 - t), scale(p23, t));
  const Vec2d point = add(scale(p012, 1.0 - t), scale(p123, t));
  return {BezierSpan{span.p0, p01, p012, point}, BezierSpan{point, p123, p23, span.p3}};
}


[[nodiscard]] Result<PathSplit> split_path_at_distance(
    const Path& path, double distance_along_path_m) {
  const Result<double> total = PathLength(path);
  if (!total.ok) return Result<PathSplit>::Fail(total.failure_category, total.error);
  if (!is_finite(distance_along_path_m) ||
      distance_along_path_m <= distance_epsilon ||
      distance_along_path_m >= total.value - distance_epsilon) {
    return Result<PathSplit>::Fail(CommitFailureCategory::kInvalidInput, "road segment split distance is outside its interior");
  }
  double remaining = distance_along_path_m;
  for (std::size_t index = 0; index < path.spans.size(); ++index) {
    const BezierSpan& span = path.spans[index];
    const double length_m = span_length(span);
    if (index + 1 < path.spans.size() && std::abs(remaining - length_m) <= distance_epsilon) {
      PathSplit result{};
      result.before.spans.insert(result.before.spans.end(), path.spans.begin(), path.spans.begin() + index + 1);
      result.after.spans.insert(result.after.spans.end(), path.spans.begin() + index + 1, path.spans.end());
      result.point = span.p3;
      return Result<PathSplit>::Ok(std::move(result));
    }
    if (remaining < length_m || index + 1 == path.spans.size()) {
      const double t = span_parameter_at_length(span, remaining);
      auto [left, right] = split_span(span, t);
      PathSplit result{};
      result.before.spans.insert(result.before.spans.end(), path.spans.begin(), path.spans.begin() + index);
      result.before.spans.push_back(left);
      result.after.spans.push_back(right);
      result.after.spans.insert(result.after.spans.end(), path.spans.begin() + index + 1, path.spans.end());
      result.point = left.p3;
      return Result<PathSplit>::Ok(std::move(result));
    }
    remaining -= length_m;
  }
  return Result<PathSplit>::Fail(CommitFailureCategory::kInternalError, "road segment split distance resolution fell through");
}

[[nodiscard]] std::pair<double, double>
manual_line_distance_bounds(const ManualLineMarking& marking) {
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  for (const BezierSpan& span : marking.path.spans) {
    for (const Vec2d point :
         std::array<Vec2d, 4>{span.p0, span.p1, span.p2, span.p3}) {
      minimum = std::min(minimum, point.x);
      maximum = std::max(maximum, point.x);
    }
  }
  return {minimum, maximum};
}

[[nodiscard]] std::pair<double, double>
manual_area_distance_bounds(const ManualAreaMarking& marking) {
  const double half_width = marking.width_m * 0.5;
  const double half_length = marking.length_m * 0.5;
  const double c = std::cos(marking.rotation_rad);
  const double s = std::sin(marking.rotation_rad);
  double minimum = std::numeric_limits<double>::infinity();
  double maximum = -std::numeric_limits<double>::infinity();
  for (const Vec2d local :
       std::array<Vec2d, 4>{Vec2d{-half_length, -half_width},
                            Vec2d{half_length, -half_width},
                            Vec2d{half_length, half_width},
                            Vec2d{-half_length, half_width}}) {
    const double distance =
        marking.frame_origin.x + local.x * c - local.y * s;
    minimum = std::min(minimum, distance);
    maximum = std::max(maximum, distance);
  }
  return {minimum, maximum};
}

void shift_manual_line_distance(ManualLineMarking& marking, double delta_m) {
  for (BezierSpan& span : marking.path.spans) {
    for (Vec2d* point :
         std::array<Vec2d*, 4>{&span.p0, &span.p1, &span.p2, &span.p3}) {
      point->x += delta_m;
    }
  }
}

[[nodiscard]] bool segments_intersect(Vec2d a, Vec2d b, Vec2d c, Vec2d d) {
  const auto orient = [](Vec2d p, Vec2d q, Vec2d r) {
    const double v = cross(subtract(q, p), subtract(r, p));
    if (std::abs(v) <= 1e-8) {
      return 0;
    }
    return v > 0.0 ? 1 : -1;
  };
  const auto on_segment = [](Vec2d p, Vec2d q, Vec2d r) {
    return std::min(p.x, r.x) - 1e-8 <= q.x && q.x <= std::max(p.x, r.x) + 1e-8 &&
           std::min(p.y, r.y) - 1e-8 <= q.y && q.y <= std::max(p.y, r.y) + 1e-8;
  };
  const int o1 = orient(a, b, c);
  const int o2 = orient(a, b, d);
  const int o3 = orient(c, d, a);
  const int o4 = orient(c, d, b);
  if (o1 != o2 && o3 != o4) {
    return true;
  }
  return (o1 == 0 && on_segment(a, c, b)) || (o2 == 0 && on_segment(a, d, b)) ||
         (o3 == 0 && on_segment(c, a, d)) || (o4 == 0 && on_segment(c, b, d));
}

[[nodiscard]] Result<bool> validate_no_self_intersection(const Path& path) {
  const std::vector<Vec2d> points = FlattenPath(path);
  if (points.size() < 4) {
    return Result<bool>::Ok(true);
  }
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    for (std::size_t j = i + 2; j + 1 < points.size(); ++j) {
      if (i == 0 && j + 1 == points.size() - 1 && almost_same(points.front(), points.back())) {
        continue;
      }
      if (segments_intersect(points[i], points[i + 1], points[j], points[j + 1])) {
        return Result<bool>::Fail(CommitFailureCategory::kRequirementConstraint,
                                  "road path self-intersection is prohibited",
                                  "road_path_self_intersection");
      }
    }
  }
  return Result<bool>::Ok(true);
}

[[nodiscard]] Vec2d path_start(const Path& path) {
  return span_start(path.spans.front());
}

[[nodiscard]] Vec2d path_end(const Path& path) {
  return span_end(path.spans.back());
}

// The tangent an interval needs at its far end to stay on the same arc as the
// tangent already decided at this end. A terminal endpoint has no neighbour to
// average with, so it follows the interval instead of its own chord.
[[nodiscard]] Vec2d mirror_tangent_across(Vec2d tangent, Vec2d chord);

// The forward heading the next drawn interval inherits at a corridor end.
[[nodiscard]] std::optional<Vec2d> corridor_terminal_tangent(
    const SavedRoadGraph& graph, RoadCorridorId corridor_id,
    RoadNodeId endpoint_node_id) {
  const RoadCorridor* corridor = FindRoadCorridor(graph, corridor_id);
  if (corridor == nullptr || corridor->segments.empty()) return std::nullopt;
  const DirectedSegmentRef last_ref = corridor->segments.back();
  const RoadSegment* source = find_segment(graph, last_ref.segment_id);
  if (source == nullptr) return std::nullopt;
  const RoadNodeId corridor_end = last_ref.reversed ? source->node_a : source->node_b;
  if (corridor_end != endpoint_node_id) return std::nullopt;
  const Vec2d inward_handle =
      last_ref.reversed ? source->shape.start_handle : source->shape.end_handle;
  if (magnitude(inward_handle) <= distance_epsilon) return std::nullopt;
  return scale(inward_handle, -1.0);
}

// An interval leaves in the heading it inherits and turns to the new point as
// one arc, so the drawn curve keeps a single bend instead of an S.
void apply_inherited_arc(Vec2d tangent, Vec2d chord, SegmentShape* shape) {
  const double inherited_length = magnitude(tangent);
  const double chord_length = magnitude(chord);
  // Degenerate input keeps the chord handles the caller already set.
  if (inherited_length <= distance_epsilon || chord_length <= distance_epsilon) return;
  const Vec2d heading = scale(tangent, 1.0 / inherited_length);
  shape->start_handle = scale(heading, chord_length / 3.0);
  shape->end_handle = scale(mirror_tangent_across(heading, chord), -chord_length / 3.0);
}

[[nodiscard]] Vec2d mirror_tangent_across(Vec2d tangent, Vec2d chord) {
  const double chord_length = magnitude(chord);
  if (chord_length <= distance_epsilon) return tangent;
  const Vec2d axis = scale(chord, 1.0 / chord_length);
  const double projection = tangent.x * axis.x + tangent.y * axis.y;
  return Vec2d{2.0 * projection * axis.x - tangent.x,
               2.0 * projection * axis.y - tangent.y};
}


void align_first_span_start(Path& path, Vec2d start) {
  BezierSpan& first_span = path.spans.front();
  if (IsLinearSpan(first_span)) {
    first_span = MakeLine(start, first_span.p3);
    return;
  }
  const Vec2d correction = subtract(start, first_span.p0);
  first_span.p0 = start;
  first_span.p1 = add(first_span.p1, correction);
}

void align_last_span_end(Path& path, Vec2d end) {
  BezierSpan& last_span = path.spans.back();
  if (IsLinearSpan(last_span)) {
    last_span = MakeLine(last_span.p0, end);
    return;
  }
  const Vec2d correction = subtract(end, last_span.p3);
  last_span.p2 = add(last_span.p2, correction);
  last_span.p3 = end;
}

Result<SegmentShape> make_linear_shape(Vec2d start, Vec2d end) {
  return SegmentShapeFromPath(MakePath({MakeLine(start, end)}));
}

Path PreviewDrawnInterval(const SavedRoadGraph& graph, RoadCorridorId corridor_id,
                          RoadNodeId endpoint_node_id, Vec2d start, Vec2d end,
                          SegmentShapeIntent intent) {
  const Vec2d chord = subtract(end, start);
  SegmentShape shape{};
  shape.intent = intent;
  shape.start_handle = scale(chord, 1.0 / 3.0);
  shape.end_handle = scale(chord, -1.0 / 3.0);
  if (intent == SegmentShapeIntent::kCurve) {
    if (const std::optional<Vec2d> inherited =
            corridor_terminal_tangent(graph, corridor_id, endpoint_node_id)) {
      apply_inherited_arc(*inherited, chord, &shape);
    }
  }
  return MakePath({MakeBezier(start, add(start, shape.start_handle),
                              add(end, shape.end_handle), end)});
}

} // namespace internal

using internal::add;
using internal::almost_same;
using internal::cross;
using internal::distance;
using internal::dot;
using internal::is_finite;
using internal::kCurveSamples;
using internal::distance_epsilon;
using internal::magnitude;
using internal::scale;
using internal::span_end;
using internal::span_eval;
using internal::span_derivative;
using internal::span_length;
using internal::span_parameter_at_length;
using internal::span_start;
using internal::subtract;
using internal::linear_span_controls_match;
using internal::validate_no_self_intersection;

Result<SegmentShape> SegmentShapeFromPath(const Path& path) {
  const Result<bool> valid = ValidatePath(path);
  if (!valid.ok) return Result<SegmentShape>::Fail(valid.failure_category, valid.error);
  SegmentShape shape{};
  if (path.spans.size() == 1 &&
      linear_span_controls_match(path.spans.front().p0, path.spans.front().p3,
                                 path.spans.front())) {
    shape.intent = SegmentShapeIntent::kStraight;
  }
  shape.start_handle = subtract(path.spans.front().p1, path.spans.front().p0);
  shape.end_handle = subtract(path.spans.back().p2, path.spans.back().p3);
  shape.internal_knots.reserve(path.spans.size() - 1);
  for (std::size_t i = 0; i + 1 < path.spans.size(); ++i) {
    const BezierSpan& before = path.spans[i];
    const BezierSpan& after = path.spans[i + 1];
    shape.internal_knots.push_back(
        SegmentKnot{before.p3, subtract(before.p2, before.p3), subtract(after.p1, after.p0)});
  }
  return Result<SegmentShape>::Ok(std::move(shape));
}

Result<Path> DeriveCanonicalAlignment(Vec2d start, Vec2d end, const SegmentShape& shape) {
  if (!is_finite(start) || !is_finite(end) || !is_finite(shape.start_handle) || !is_finite(shape.end_handle)) {
    return Result<Path>::Fail(CommitFailureCategory::kInvalidInput, "road segment shape contains a non-finite point");
  }
  Path path{};
  Vec2d span_start = start;
  Vec2d handle_out = shape.start_handle;
  for (const SegmentKnot& knot : shape.internal_knots) {
    if (!is_finite(knot.position) || !is_finite(knot.handle_in) || !is_finite(knot.handle_out)) {
      return Result<Path>::Fail(CommitFailureCategory::kInvalidInput, "road segment shape contains a non-finite knot");
    }
    path.spans.push_back(
        MakeBezier(span_start, add(span_start, handle_out), add(knot.position, knot.handle_in), knot.position));
    span_start = knot.position;
    handle_out = knot.handle_out;
  }
  path.spans.push_back(MakeBezier(span_start, add(span_start, handle_out), add(end, shape.end_handle), end));
  const Result<bool> valid = ValidatePath(path);
  if (!valid.ok) return Result<Path>::Fail(valid.failure_category, valid.error);
  return Result<Path>::Ok(std::move(path));
}

// The interval the draw tool would commit next. Preview and commit read the
// same rule, so the guide cannot disagree with the road it produces.
Path PreviewDrawnInterval(const SavedRoadGraph& graph, RoadCorridorId corridor_id,
                          RoadNodeId endpoint_node_id, Vec2d start, Vec2d end,
                          SegmentShapeIntent intent) {
  const Vec2d chord = subtract(end, start);
  SegmentShape shape{};
  shape.intent = intent;
  shape.start_handle = scale(chord, 1.0 / 3.0);
  shape.end_handle = scale(chord, -1.0 / 3.0);
  if (intent == SegmentShapeIntent::kCurve) {
    if (const std::optional<Vec2d> inherited =
            internal::corridor_terminal_tangent(graph, corridor_id,
                                                endpoint_node_id)) {
      internal::apply_inherited_arc(*inherited, chord, &shape);
    }
  }
  return MakePath({MakeBezier(start, add(start, shape.start_handle),
                              add(end, shape.end_handle), end)});
}

bool IsLinearSpan(const BezierSpan& span) {
  return almost_same(span.p1, add(span.p0, scale(subtract(span.p3, span.p0), 1.0 / 3.0))) &&
         almost_same(span.p2, add(span.p0, scale(subtract(span.p3, span.p0), 2.0 / 3.0)));
}

Result<bool> ValidatePath(const Path& path) {
  if (path.spans.empty()) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road path has no spans");
  }
  for (std::size_t i = 0; i < path.spans.size(); ++i) {
    const BezierSpan& span = path.spans[i];
    if (!is_finite(span.p0) || !is_finite(span.p1) || !is_finite(span.p2) || !is_finite(span.p3)) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road path contains a non-finite control point");
    }
    if (span_length(span) <= distance_epsilon) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road span has zero length");
    }
    if (i > 0 && !almost_same(span_start(span), span_end(path.spans[i - 1]))) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road path spans are not continuous");
    }
  }
  return validate_no_self_intersection(path);
}

Result<double> PathLength(const Path& path) {
  const Result<bool> valid = ValidatePath(path);
  if (!valid.ok) {
    return Result<double>::Fail(valid.failure_category, valid.error);
  }
  double out = 0.0;
  for (const BezierSpan& span : path.spans) {
    out += span_length(span);
  }
  return Result<double>::Ok(out);
}

Result<Vec2d> EvaluatePath(const Path& path, double distance_along_path_m) {
  const Result<double> length_result = PathLength(path);
  if (!length_result.ok) {
    return Result<Vec2d>::Fail(length_result.failure_category, length_result.error);
  }
  double remaining =
      std::clamp(distance_along_path_m, 0.0, length_result.value);
  for (const BezierSpan& span : path.spans) {
    const double len = span_length(span);
    if (remaining <= len || &span == &path.spans.back()) {
      return Result<Vec2d>::Ok(span_eval(span, span_parameter_at_length(span, remaining)));
    }
    remaining -= len;
  }
  return Result<Vec2d>::Fail(CommitFailureCategory::kInternalError, "road path evaluation fell through");
}

std::vector<Vec2d> FlattenPath(const Path& path) {
  std::vector<Vec2d> points{};
  for (const BezierSpan& span : path.spans) {
    const int samples = IsLinearSpan(span) ? 1 : kCurveSamples;
    if (points.empty()) {
      points.push_back(span_start(span));
    }
    for (int i = 1; i <= samples; ++i) {
      points.push_back(span_eval(span, static_cast<double>(i) / samples));
    }
  }
  return points;
}

BezierSpan MakeLine(Vec2d a, Vec2d b) {
  const Vec2d delta = subtract(b, a);
  return BezierSpan{a, add(a, scale(delta, 1.0 / 3.0)), add(a, scale(delta, 2.0 / 3.0)), b};
}

BezierSpan MakeBezier(Vec2d p0, Vec2d p1, Vec2d p2, Vec2d p3) {
  return BezierSpan{p0, p1, p2, p3};
}

Path MakePath(std::vector<BezierSpan> spans) {
  Path out{};
  out.spans = std::move(spans);
  return out;
}

} // namespace city::road
