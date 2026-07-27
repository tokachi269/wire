#include "city/road/road.hpp"

#include "build/stages.hpp"
#include "operations/operation_plan.hpp"
#include "persistence/road_archive.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <unordered_set>

namespace city::road {
namespace {

constexpr double kEpsilon = 1e-9;
constexpr int kCurveSamples = 24;
constexpr double kP1MinSegmentLengthM = 8.0;
constexpr double kSnapStationPointToleranceM = 0.6;

[[nodiscard]] bool finite(double value) {
  return std::isfinite(value);
}

[[nodiscard]] bool finite(Vec2d value) {
  return finite(value.x) && finite(value.y);
}

[[nodiscard]] double dot(Vec2d a, Vec2d b) {
  return a.x * b.x + a.y * b.y;
}

[[nodiscard]] double cross(Vec2d a, Vec2d b) {
  return a.x * b.y - a.y * b.x;
}

[[nodiscard]] Vec2d add(Vec2d a, Vec2d b) {
  return {a.x + b.x, a.y + b.y};
}

[[nodiscard]] Vec2d sub(Vec2d a, Vec2d b) {
  return {a.x - b.x, a.y - b.y};
}

[[nodiscard]] Vec2d mul(Vec2d a, double scale) {
  return {a.x * scale, a.y * scale};
}

[[nodiscard]] double length(Vec2d value) {
  return std::sqrt(dot(value, value));
}

[[nodiscard]] double distance(Vec2d a, Vec2d b) {
  return length(sub(a, b));
}

[[nodiscard]] bool almost_same(Vec2d a, Vec2d b) {
  return distance(a, b) <= 1e-6;
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
  return add(add(mul(span.p0, u * u * u), mul(span.p1, 3.0 * u * u * t)),
             add(mul(span.p2, 3.0 * u * t * t), mul(span.p3, t * t * t)));
}

[[nodiscard]] double span_length(const BezierSpan& span) {
  double out = 0.0;
  Vec2d prev = span.p0;
  for (int i = 1; i <= kCurveSamples; ++i) {
    const Vec2d current = span_eval(span, static_cast<double>(i) / kCurveSamples);
    out += distance(prev, current);
    prev = current;
  }
  return out;
}

[[nodiscard]] double span_parameter_at_length(const BezierSpan& span, double station_m) {
  if (station_m <= 0.0) return 0.0;
  double accumulated = 0.0;
  Vec2d previous = span.p0;
  for (int i = 1; i <= kCurveSamples; ++i) {
    const double t = static_cast<double>(i) / kCurveSamples;
    const Vec2d current = span_eval(span, t);
    const double piece_length = distance(previous, current);
    if (accumulated + piece_length >= station_m || i == kCurveSamples) {
      const double ratio = piece_length <= kEpsilon ? 0.0 : (station_m - accumulated) / piece_length;
      return (static_cast<double>(i - 1) + std::clamp(ratio, 0.0, 1.0)) / kCurveSamples;
    }
    accumulated += piece_length;
    previous = current;
  }
  return 1.0;
}

[[nodiscard]] std::pair<BezierSpan, BezierSpan> split_span(const BezierSpan& span, double t) {
  const Vec2d p01 = add(mul(span.p0, 1.0 - t), mul(span.p1, t));
  const Vec2d p12 = add(mul(span.p1, 1.0 - t), mul(span.p2, t));
  const Vec2d p23 = add(mul(span.p2, 1.0 - t), mul(span.p3, t));
  const Vec2d p012 = add(mul(p01, 1.0 - t), mul(p12, t));
  const Vec2d p123 = add(mul(p12, 1.0 - t), mul(p23, t));
  const Vec2d point = add(mul(p012, 1.0 - t), mul(p123, t));
  return {BezierSpan{span.p0, p01, p012, point}, BezierSpan{point, p123, p23, span.p3}};
}

struct PathSplit {
  Path before{};
  Path after{};
  Vec2d point{};
};

[[nodiscard]] Result<PathSplit> split_path_at_station(const Path& path, double station_m) {
  const Result<double> total = PathLength(path);
  if (!total.ok) return Result<PathSplit>::Fail(total.error_kind, total.error);
  if (!finite(station_m) || station_m <= kEpsilon || station_m >= total.value - kEpsilon) {
    return Result<PathSplit>::Fail(ErrorKind::kValidation, "road segment split station is outside its interior");
  }
  double remaining = station_m;
  for (std::size_t index = 0; index < path.spans.size(); ++index) {
    const BezierSpan& span = path.spans[index];
    const double length_m = span_length(span);
    if (index + 1 < path.spans.size() && std::abs(remaining - length_m) <= kEpsilon) {
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
  return Result<PathSplit>::Fail(ErrorKind::kInternal, "road segment split station resolution fell through");
}

[[nodiscard]] bool segments_intersect(Vec2d a, Vec2d b, Vec2d c, Vec2d d) {
  const auto orient = [](Vec2d p, Vec2d q, Vec2d r) {
    const double v = cross(sub(q, p), sub(r, p));
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
        return Result<bool>::Fail(ErrorKind::kUnsupported, "road path self-intersection is unsupported");
      }
    }
  }
  return Result<bool>::Ok(true);
}

[[nodiscard]] const CrossSectionTemplate* find_template(const SavedRoadGraph& graph, CrossSectionTemplateId id) {
  const auto it = std::find_if(graph.section_templates.begin(), graph.section_templates.end(),
                               [id](const CrossSectionTemplate& item) { return item.id == id; });
  return it == graph.section_templates.end() ? nullptr : &*it;
}

[[nodiscard]] const SectionTransition* find_transition(const SavedRoadGraph& graph, SectionTransitionId id) {
  const auto it = std::find_if(graph.transitions.begin(), graph.transitions.end(),
                               [id](const SectionTransition& item) { return item.id == id; });
  return it == graph.transitions.end() ? nullptr : &*it;
}

[[nodiscard]] const RoadSegment* find_segment(const SavedRoadGraph& graph, RoadSegmentId id) {
  const auto it = std::find_if(graph.segments.begin(), graph.segments.end(),
                               [id](const RoadSegment& item) { return item.id == id; });
  return it == graph.segments.end() ? nullptr : &*it;
}

[[nodiscard]] RoadSegment* find_segment(SavedRoadGraph& graph, RoadSegmentId id) {
  const auto it = std::find_if(graph.segments.begin(), graph.segments.end(),
                               [id](const RoadSegment& item) { return item.id == id; });
  return it == graph.segments.end() ? nullptr : &*it;
}

[[nodiscard]] const RoadNode* find_node(const SavedRoadGraph& graph, RoadNodeId id) {
  const auto it = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                               [id](const RoadNode& item) { return item.id == id; });
  return it == graph.nodes.end() ? nullptr : &*it;
}

[[nodiscard]] const NodeConnectionPolicyOverride* find_connection_policy_override(const SavedRoadGraph& graph,
                                                                                   RoadNodeId node_id) {
  const auto it = std::find_if(graph.connection_policy_overrides.begin(), graph.connection_policy_overrides.end(),
                               [node_id](const auto& item) { return item.node_id == node_id; });
  return it == graph.connection_policy_overrides.end() ? nullptr : &*it;
}

[[nodiscard]] RoadNode* find_node(SavedRoadGraph& graph, RoadNodeId id) {
  const auto it = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                               [id](const RoadNode& item) { return item.id == id; });
  return it == graph.nodes.end() ? nullptr : &*it;
}

[[nodiscard]] std::size_t node_degree(const SavedRoadGraph& graph, RoadNodeId id) {
  return static_cast<std::size_t>(std::count_if(graph.segments.begin(), graph.segments.end(), [id](const RoadSegment& segment) {
    return segment.node_a == id || segment.node_b == id;
  }));
}


[[nodiscard]] const SurfaceBand* find_band(const CrossSectionTemplate& section, std::uint64_t id) {
  const auto it = std::find_if(section.bands.begin(), section.bands.end(),
                               [id](const SurfaceBand& band) { return band.element_id == id; });
  return it == section.bands.end() ? nullptr : &*it;
}


[[nodiscard]] double station_value(StationRef ref, double total) {
  if (ref.kind == StationRefKind::kFromEnd) {
    return total - ref.value;
  }
  if (ref.kind == StationRefKind::kRatio) {
    return total * ref.value;
  }
  return ref.value;
}



[[nodiscard]] Vec2d path_start(const Path& path) {
  return span_start(path.spans.front());
}

[[nodiscard]] Vec2d path_end(const Path& path) {
  return span_end(path.spans.back());
}


[[nodiscard]] Result<bool> validate_transition(const SavedRoadGraph& graph, const SectionTransition& transition) {
  const CrossSectionTemplate* from = find_template(graph, transition.from_template);
  const CrossSectionTemplate* to = find_template(graph, transition.to_template);
  if (from == nullptr || to == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section transition references a missing template");
  }
  if (!finite(transition.start.value) || !finite(transition.end.value)) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section transition station is not finite");
  }
  const auto valid_station = [](StationRef station) {
    return station.value >= 0.0 &&
           (station.kind != StationRefKind::kRatio || station.value <= 1.0);
  };
  if (!valid_station(transition.start) || !valid_station(transition.end)) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section transition station reference is outside its range");
  }
  if (transition.rules.empty()) {
    return Result<bool>::Fail(ErrorKind::kUnsupported, "section transition must define element actions");
  }
  std::unordered_set<std::uint64_t> ruled_elements{};
  for (const SectionTransitionRule& rule : transition.rules) {
    if (rule.element_id == 0 || !ruled_elements.insert(rule.element_id).second ||
        (find_band(*from, rule.element_id) == nullptr && find_band(*to, rule.element_id) == nullptr)) {
      return Result<bool>::Fail(ErrorKind::kValidation, "section transition rule element is invalid");
    }
    if (rule.action == TransitionAction::kUnsupported) {
      return Result<bool>::Fail(ErrorKind::kUnsupported, "section transition contains unsupported element action");
    }
  }
  const auto action_for = [&transition](std::uint64_t id) -> std::optional<TransitionAction> {
    const auto it = std::find_if(transition.rules.begin(), transition.rules.end(),
                                 [id](const SectionTransitionRule& rule) { return rule.element_id == id; });
    return it == transition.rules.end() ? std::nullopt : std::optional<TransitionAction>(it->action);
  };
  for (const SurfaceBand& band : to->bands) {
    if (find_band(*from, band.element_id) == nullptr && action_for(band.element_id) != TransitionAction::kTaperIn) {
      return Result<bool>::Fail(ErrorKind::kUnsupported, "appearing section element requires TaperIn");
    }
  }
  for (const SurfaceBand& band : from->bands) {
    if (find_band(*to, band.element_id) != nullptr) continue;
    const std::optional<TransitionAction> action = action_for(band.element_id);
    const TransitionAction required = band.role == SurfaceRole::kMedian ? TransitionAction::kEndCap
                                                                        : TransitionAction::kTaperOut;
    if (action != required) {
      return Result<bool>::Fail(ErrorKind::kUnsupported,
                                band.role == SurfaceRole::kMedian
                                    ? "disappearing median requires EndCap"
                                    : "disappearing section element requires TaperOut");
    }
  }
  return Result<bool>::Ok(true);
}

[[nodiscard]] Result<bool> validate_section_template(const CrossSectionTemplate& section) {
  if (section.bands.empty() || section.boundaries.size() + 1 != section.bands.size()) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section template chain is incomplete");
  }
  std::unordered_set<std::uint64_t> ids{};
  for (const SurfaceBand& band : section.bands) {
    if (band.element_id == 0 || !ids.insert(band.element_id).second || !finite(band.width_m) ||
        !finite(band.cross_slope) || band.width_m <= 0.0) {
      return Result<bool>::Fail(ErrorKind::kValidation, "section template surface band is invalid");
    }
    if (!IsKnownSurfaceStyle(band.style_id)) {
      return Result<bool>::Fail(ErrorKind::kValidation, "section template surface style is unknown");
    }
  }
  ids.clear();
  for (const BoundaryProfile& boundary : section.boundaries) {
    if (boundary.boundary_id == 0 || !ids.insert(boundary.boundary_id).second || !finite(boundary.width_m) ||
        !finite(boundary.height_m) || boundary.width_m < 0.0) {
      return Result<bool>::Fail(ErrorKind::kValidation, "section template boundary is invalid");
    }
  }
  return Result<bool>::Ok(true);
}

} // namespace

BezierSpan MakeLine(Vec2d a, Vec2d b) {
  const Vec2d delta = sub(b, a);
  return BezierSpan{a, add(a, mul(delta, 1.0 / 3.0)), add(a, mul(delta, 2.0 / 3.0)), b};
}

BezierSpan MakeBezier(Vec2d p0, Vec2d p1, Vec2d p2, Vec2d p3) {
  return BezierSpan{p0, p1, p2, p3};
}

Path MakePath(std::vector<BezierSpan> spans) {
  Path out{};
  out.spans = std::move(spans);
  return out;
}

Result<Path> NormalizeRoadPath(Path path) {
  const Result<bool> input_valid = ValidatePath(path);
  if (!input_valid.ok) {
    return Result<Path>::Fail(input_valid.error_kind, input_valid.error);
  }
  for (std::size_t index = 0; index + 1 < path.spans.size(); ++index) {
    BezierSpan& before = path.spans[index];
    BezierSpan& after = path.spans[index + 1];
    const Vec2d join = before.p3;
    const Vec2d correction = sub(join, after.p0);
    after.p0 = join;
    after.p1 = add(after.p1, correction);

    Vec2d incoming = sub(join, before.p2);
    Vec2d outgoing = sub(after.p1, join);
    double incoming_length = length(incoming);
    double outgoing_length = length(outgoing);
    if (incoming_length <= kEpsilon) {
      incoming = sub(join, before.p0);
      incoming_length = length(incoming) / 3.0;
    }
    if (outgoing_length <= kEpsilon) {
      outgoing = sub(after.p3, join);
      outgoing_length = length(outgoing) / 3.0;
    }
    if (incoming_length <= kEpsilon || outgoing_length <= kEpsilon) {
      return Result<Path>::Fail(
          ErrorKind::kUnsupported,
          "road path normalization has no corner tangent");
    }
    const Vec2d incoming_direction =
        mul(incoming, 1.0 / length(incoming));
    const Vec2d outgoing_direction =
        mul(outgoing, 1.0 / length(outgoing));
    const double direction_dot =
        std::clamp(dot(incoming_direction, outgoing_direction), -1.0, 1.0);
    if (direction_dot <= -1.0 + 1e-9) {
      return Result<Path>::Fail(ErrorKind::kUnsupported,
                                "road path U-turn is unsupported");
    }
    if (direction_dot >= 1.0 - 1e-12) {
      continue;
    }
    const Vec2d tangent_sum =
        add(incoming_direction, outgoing_direction);
    const double tangent_length = length(tangent_sum);
    if (tangent_length <= kEpsilon) {
      return Result<Path>::Fail(
          ErrorKind::kUnsupported,
          "road path corner tangent is unsupported");
    }
    const Vec2d tangent = mul(tangent_sum, 1.0 / tangent_length);
    before.p2 = sub(join, mul(tangent, incoming_length));
    after.p1 = add(join, mul(tangent, outgoing_length));
  }
  const Result<bool> normalized_valid = ValidatePath(path);
  if (!normalized_valid.ok) {
    return Result<Path>::Fail(normalized_valid.error_kind,
                              normalized_valid.error);
  }
  return Result<Path>::Ok(std::move(path));
}

Result<SegmentShape> SegmentShapeFromPath(const Path& path) {
  const Result<bool> valid = ValidatePath(path);
  if (!valid.ok) return Result<SegmentShape>::Fail(valid.error_kind, valid.error);
  SegmentShape shape{};
  shape.start_handle = sub(path.spans.front().p1, path.spans.front().p0);
  shape.end_handle = sub(path.spans.back().p2, path.spans.back().p3);
  shape.internal_knots.reserve(path.spans.size() - 1);
  for (std::size_t i = 0; i + 1 < path.spans.size(); ++i) {
    const BezierSpan& before = path.spans[i];
    const BezierSpan& after = path.spans[i + 1];
    shape.internal_knots.push_back(
        SegmentKnot{before.p3, sub(before.p2, before.p3), sub(after.p1, after.p0)});
  }
  return Result<SegmentShape>::Ok(std::move(shape));
}

Result<Path> BuildCanonicalAlignment(Vec2d start, Vec2d end, const SegmentShape& shape) {
  if (!finite(start) || !finite(end) || !finite(shape.start_handle) || !finite(shape.end_handle)) {
    return Result<Path>::Fail(ErrorKind::kValidation, "road segment shape contains a non-finite point");
  }
  Path path{};
  Vec2d span_start = start;
  Vec2d handle_out = shape.start_handle;
  for (const SegmentKnot& knot : shape.internal_knots) {
    if (!finite(knot.position) || !finite(knot.handle_in) || !finite(knot.handle_out)) {
      return Result<Path>::Fail(ErrorKind::kValidation, "road segment shape contains a non-finite knot");
    }
    path.spans.push_back(
        MakeBezier(span_start, add(span_start, handle_out), add(knot.position, knot.handle_in), knot.position));
    span_start = knot.position;
    handle_out = knot.handle_out;
  }
  path.spans.push_back(MakeBezier(span_start, add(span_start, handle_out), add(end, shape.end_handle), end));
  const Result<bool> valid = ValidatePath(path);
  if (!valid.ok) return Result<Path>::Fail(valid.error_kind, valid.error);
  return Result<Path>::Ok(std::move(path));
}

const Path* FindCanonicalAlignment(const DerivedRoad& derived, RoadSegmentId segment_id) {
  const auto found = std::find_if(derived.canonical_alignments.begin(), derived.canonical_alignments.end(),
                                  [segment_id](const CanonicalAlignment& item) {
                                    return item.segment_id == segment_id;
                                  });
  return found == derived.canonical_alignments.end() ? nullptr : &found->path;
}

bool IsLinearSpan(const BezierSpan& span) {
  return almost_same(span.p1, add(span.p0, mul(sub(span.p3, span.p0), 1.0 / 3.0))) &&
         almost_same(span.p2, add(span.p0, mul(sub(span.p3, span.p0), 2.0 / 3.0)));
}

RoadToolDraft PreviewRoadToolPath(Vec2d start, Vec2d end, std::optional<Vec2d> handle_a, std::optional<Vec2d> handle_b) {
  RoadToolDraft draft{};
  draft.has_live_preview = true;
  draft.supports_bezier_handles = true;
  if (handle_a.has_value() && handle_b.has_value()) {
    draft.preview_path = MakePath({MakeBezier(start, *handle_a, *handle_b, end)});
  } else {
    draft.preview_path = MakePath({MakeLine(start, end)});
  }
  return draft;
}

CrossSectionTemplate JapaneseUrbanTwoLaneTemplate(CrossSectionTemplateId id) {
  CrossSectionTemplate section{};
  section.id = id;
  section.bands = {
      {10, SurfaceRole::kSidewalk, 2.0, 0.01, builtin_surface_styles::kSidewalk},
      {20, SurfaceRole::kCarriageway, 3.0, 0.02, builtin_surface_styles::kAsphalt},
      {30, SurfaceRole::kCarriageway, 3.0, -0.02, builtin_surface_styles::kAsphalt},
      {40, SurfaceRole::kSidewalk, 2.0, -0.01, builtin_surface_styles::kSidewalk},
  };
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, MarkingRule::kOuterLine},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, MarkingRule::kCenterLine},
      {300, BoundaryRole::kCurb, 0.2, 0.15, MarkingRule::kOuterLine},
  };
  return section;
}

CrossSectionTemplate ThreeLaneTemplate(CrossSectionTemplateId id) {
  CrossSectionTemplate section = JapaneseUrbanTwoLaneTemplate(id);
  section.bands = {
      {10, SurfaceRole::kSidewalk, 2.0, 0.01, builtin_surface_styles::kSidewalk},
      {20, SurfaceRole::kCarriageway, 3.0, 0.02, builtin_surface_styles::kAsphalt},
      {30, SurfaceRole::kCarriageway, 3.0, 0.0, builtin_surface_styles::kAsphalt},
      {35, SurfaceRole::kCarriageway, 3.0, -0.02, builtin_surface_styles::kAsphalt},
      {40, SurfaceRole::kSidewalk, 2.0, -0.01, builtin_surface_styles::kSidewalk},
  };
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, MarkingRule::kOuterLine},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, MarkingRule::kCenterLine},
      {250, BoundaryRole::kLaneDivider, 0.0, 0.0, MarkingRule::kCenterLine},
      {300, BoundaryRole::kCurb, 0.2, 0.15, MarkingRule::kOuterLine},
  };
  return section;
}

CrossSectionTemplate NoLeftSidewalkTemplate(CrossSectionTemplateId id) {
  CrossSectionTemplate section = JapaneseUrbanTwoLaneTemplate(id);
  section.bands.erase(section.bands.begin());
  section.boundaries.erase(section.boundaries.begin());
  return section;
}

CrossSectionTemplate MedianTwoLaneTemplate(CrossSectionTemplateId id) {
  CrossSectionTemplate section = JapaneseUrbanTwoLaneTemplate(id);
  section.bands.insert(section.bands.begin() + 2,
                       {25, SurfaceRole::kMedian, 2.0, 0.0, builtin_surface_styles::kMedian});
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, MarkingRule::kOuterLine},
      {210, BoundaryRole::kMedianEdge, 0.2, 0.12, MarkingRule::kNone},
      {220, BoundaryRole::kMedianEdge, 0.2, -0.12, MarkingRule::kNone},
      {300, BoundaryRole::kCurb, 0.2, 0.15, MarkingRule::kOuterLine},
  };
  return section;
}

RoadState::RoadState() {
  graph_.section_templates.push_back(JapaneseUrbanTwoLaneTemplate(1));
  graph_.section_templates.push_back(ThreeLaneTemplate(2));
  graph_.section_templates.push_back(NoLeftSidewalkTemplate(3));
  graph_.section_templates.push_back(MedianTwoLaneTemplate(4));
  next_id_ = 5;
}

const SavedRoadGraph& RoadState::graph() const {
  return graph_;
}

const DerivedRoad& RoadState::derived() const {
  return derived_;
}

Result<bool> RoadState::Execute(const operations::OperationPlan& plan) {
  RoadState trial = *this;
  const Result<bool> applied = operations::Apply(plan, trial.graph_, trial.next_id_);
  if (!applied.ok) return applied;
  const Result<bool> authoritative_valid =
      persistence::ValidateAuthoritativeGraph(trial.graph_, trial.next_id_);
  if (!authoritative_valid.ok) return authoritative_valid;
  const Result<bool> built = trial.BuildDerived();
  if (!built.ok) return built;
  *this = std::move(trial);
  return Result<bool>::Ok(true);
}

Result<RoadSegmentId> RoadState::AddSegment(AddSegmentRequest request) {
  Path alignment = std::move(request.alignment);
  const CrossSectionTemplateId section_template = request.section_template;
  Result<Path> normalized = NormalizeRoadPath(std::move(alignment));
  if (!normalized.ok) {
    return Result<RoadSegmentId>::Fail(normalized.error_kind,
                                       normalized.error);
  }
  alignment = std::move(normalized.value);
  if (find_template(graph_, section_template) == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "road segment references a missing section template");
  }
  const Result<SegmentShape> shape = SegmentShapeFromPath(alignment);
  if (!shape.ok) return Result<RoadSegmentId>::Fail(shape.error_kind, shape.error);
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId node_a = next_id++;
  const RoadNodeId node_b = next_id++;
  const RoadSegmentId segment_id = next_id++;
  plan.next_id_after = next_id;
  plan.add_nodes = {RoadNode{node_a, path_start(alignment)}, RoadNode{node_b, path_end(alignment)}};
  plan.add_segments.push_back(
      RoadSegment{segment_id, node_a, node_b, shape.value, section_template, std::nullopt});
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.error_kind, executed.error);
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::ExtendSegment(ExtendSegmentRequest request) {
  const RoadSegment* source = find_segment(graph_, request.segment_id);
  const RoadNode* endpoint = find_node(graph_, request.endpoint_node_id);
  if (source == nullptr || endpoint == nullptr) {
    return Result<RoadSegmentId>::Fail(
        ErrorKind::kValidation,
        "road extension segment or endpoint does not exist");
  }
  const RoadSegmentId segment_id = source->id;
  const bool prepend = source->node_a == endpoint->id;
  const bool append = source->node_b == endpoint->id;
  if (!prepend && !append) {
    return Result<RoadSegmentId>::Fail(
        ErrorKind::kValidation,
        "road extension node is not a segment endpoint");
  }
  if (node_degree(graph_, endpoint->id) != 1) {
    return Result<RoadSegmentId>::Fail(
        ErrorKind::kUnsupported,
        "road extension requires a degree-one endpoint");
  }
  if (source->section_template != request.section_template) {
    return Result<RoadSegmentId>::Fail(
        ErrorKind::kUnsupported,
        "road extension requires the existing section template");
  }
  if (find_connection_policy_override(graph_, endpoint->id) != nullptr) {
    return Result<RoadSegmentId>::Fail(
        ErrorKind::kUnsupported,
        "road extension endpoint has an explicit connection policy");
  }
  if (prepend &&
      (source->transition.has_value() ||
       std::any_of(graph_.manual_lines.begin(), graph_.manual_lines.end(),
                   [source](const ManualLineMarking& marking) {
                     return marking.owner_segment_id == source->id;
                   }) ||
       std::any_of(graph_.manual_areas.begin(), graph_.manual_areas.end(),
                   [source](const ManualAreaMarking& marking) {
                     return marking.owner_segment_id == source->id;
                   }))) {
    return Result<RoadSegmentId>::Fail(
        ErrorKind::kUnsupported,
        "road prepend with station-owned state is unsupported");
  }

  Path extension = std::move(request.extension);
  const Result<bool> extension_valid = ValidatePath(extension);
  if (!extension_valid.ok) {
    return Result<RoadSegmentId>::Fail(extension_valid.error_kind,
                                       extension_valid.error);
  }
  BezierSpan& first_span = extension.spans.front();
  const Vec2d correction = sub(endpoint->position, first_span.p0);
  first_span.p0 = endpoint->position;
  first_span.p1 = add(first_span.p1, correction);

  const Path* existing = FindCanonicalAlignment(derived_, source->id);
  if (existing == nullptr) {
    return Result<RoadSegmentId>::Fail(
        ErrorKind::kInternal,
        "road extension canonical alignment is missing");
  }
  Path combined{};
  if (append) {
    combined = *existing;
    combined.spans.insert(combined.spans.end(), extension.spans.begin(),
                          extension.spans.end());
  } else {
    combined.spans.reserve(extension.spans.size() + existing->spans.size());
    for (auto it = extension.spans.rbegin(); it != extension.spans.rend();
         ++it) {
      combined.spans.push_back(
          BezierSpan{it->p3, it->p2, it->p1, it->p0});
    }
    combined.spans.insert(combined.spans.end(), existing->spans.begin(),
                          existing->spans.end());
  }
  Result<Path> normalized = NormalizeRoadPath(std::move(combined));
  if (!normalized.ok) {
    return Result<RoadSegmentId>::Fail(normalized.error_kind,
                                       normalized.error);
  }
  Result<SegmentShape> shape = SegmentShapeFromPath(normalized.value);
  if (!shape.ok) {
    return Result<RoadSegmentId>::Fail(shape.error_kind, shape.error);
  }

  RoadNode replacement_node = *endpoint;
  replacement_node.position = path_end(extension);
  RoadSegment replacement_segment = *source;
  replacement_segment.shape = std::move(shape.value);
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.replace_nodes.push_back(std::move(replacement_node));
  plan.replace_segments.push_back(std::move(replacement_segment));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) {
    return Result<RoadSegmentId>::Fail(executed.error_kind, executed.error);
  }
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::AddSegmentConnectedTo(AddSegmentConnectedToRequest request) {
  Path alignment = std::move(request.alignment);
  const CrossSectionTemplateId section_template = request.section_template;
  const RoadNodeId start_node = request.start_node;
  const RoadNode* node = find_node(graph_, start_node);
  if (node == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "road segment start node does not exist");
  }
  if (find_template(graph_, section_template) == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation,
                                       "connected road references a missing section template");
  }
  const Result<double> length_result = PathLength(alignment);
  if (!length_result.ok) {
    return Result<RoadSegmentId>::Fail(length_result.error_kind, length_result.error);
  }
  if (length_result.value < kP1MinSegmentLengthM) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported, "connected road segment is shorter than P1 minimum");
  }
  BezierSpan& first_span = alignment.spans.front();
  const Vec2d correction = sub(node->position, first_span.p0);
  first_span.p0 = node->position;
  first_span.p1 = add(first_span.p1, correction);
  const Result<SegmentShape> shape = SegmentShapeFromPath(alignment);
  if (!shape.ok) return Result<RoadSegmentId>::Fail(shape.error_kind, shape.error);
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId end_node = next_id++;
  const RoadSegmentId segment_id = next_id++;
  plan.add_nodes.push_back(RoadNode{end_node, path_end(alignment)});
  plan.add_segments.push_back(
      RoadSegment{segment_id, start_node, end_node, shape.value, section_template, std::nullopt});
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.error_kind, executed.error);
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::AddSegmentConnectedToSegment(AddSegmentConnectedToSegmentRequest request) {
  Path alignment = std::move(request.alignment);
  const CrossSectionTemplateId section_template = request.section_template;
  const RoadSegmentId start_segment = request.start_segment;
  const double station_m = request.station_m;
  const Result<bool> alignment_valid = ValidatePath(alignment);
  if (!alignment_valid.ok) {
    return Result<RoadSegmentId>::Fail(alignment_valid.error_kind, alignment_valid.error);
  }
  const RoadSegment* source = find_segment(graph_, start_segment);
  if (source == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "road segment snap target does not exist");
  }
  if (source->transition.has_value()) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported,
                                       "splitting a transitioning road segment is unsupported");
  }
  const Path* source_path = FindCanonicalAlignment(derived_, source->id);
  if (source_path == nullptr) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kInternal, "road segment canonical alignment is missing");
  }
  const Result<PathSplit> path_split = split_path_at_station(*source_path, station_m);
  if (!path_split.ok) {
    return Result<RoadSegmentId>::Fail(path_split.error_kind, path_split.error);
  }
  if (distance(path_start(alignment), path_split.value.point) > kSnapStationPointToleranceM) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation,
                                       "road segment input start does not match its explicit snap station");
  }
  BezierSpan& input_start = alignment.spans.front();
  if (IsLinearSpan(input_start)) {
    input_start = MakeLine(path_split.value.point, input_start.p3);
  } else {
    const Vec2d correction = sub(path_split.value.point, input_start.p0);
    input_start.p0 = path_split.value.point;
    input_start.p1 = add(input_start.p1, correction);
  }
  const Result<double> branch_length = PathLength(alignment);
  if (!branch_length.ok) return Result<RoadSegmentId>::Fail(branch_length.error_kind, branch_length.error);
  if (branch_length.value < kP1MinSegmentLengthM) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kUnsupported, "connected road segment is shorter than P1 minimum");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId split_node = next_id++;
  const RoadSegmentId second_id = next_id++;
  const RoadNodeId branch_end_node = next_id++;
  const RoadSegmentId branch_id = next_id++;
  const Result<SegmentShape> first_shape = SegmentShapeFromPath(path_split.value.before);
  const Result<SegmentShape> second_shape = SegmentShapeFromPath(path_split.value.after);
  const Result<SegmentShape> branch_shape = SegmentShapeFromPath(alignment);
  if (!first_shape.ok || !second_shape.ok || !branch_shape.ok) {
    return Result<RoadSegmentId>::Fail(ErrorKind::kValidation, "road segment split shape is invalid");
  }
  RoadSegment first = *source;
  first.node_b = split_node;
  first.shape = first_shape.value;
  plan.replace_segments.push_back(std::move(first));
  plan.add_nodes = {RoadNode{split_node, path_split.value.point}, RoadNode{branch_end_node, path_end(alignment)}};
  plan.add_segments = {
      RoadSegment{second_id, split_node, source->node_b, second_shape.value, source->section_template,
                  source->transition},
      RoadSegment{branch_id, split_node, branch_end_node, branch_shape.value, section_template, std::nullopt},
  };
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.error_kind, executed.error);
  return Result<RoadSegmentId>::Ok(branch_id);
}

Result<bool> RoadState::EditSegmentShape(EditSegmentShapeRequest request) {
  const RoadSegmentId segment_id = request.segment_id;
  SegmentShape shape = std::move(request.shape);
  const RoadSegment* segment = find_segment(graph_, segment_id);
  if (segment == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road segment does not exist");
  }
  const RoadNode* node_a = find_node(graph_, segment->node_a);
  const RoadNode* node_b = find_node(graph_, segment->node_b);
  const Result<Path> alignment = BuildCanonicalAlignment(node_a->position, node_b->position, shape);
  if (!alignment.ok) return Result<bool>::Fail(alignment.error_kind, alignment.error);
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  RoadSegment replacement = *segment;
  replacement.shape = std::move(shape);
  plan.replace_segments.push_back(std::move(replacement));
  return Execute(plan);
}

Result<bool> RoadState::MoveNode(MoveNodeRequest request) {
  const RoadNodeId node_id = request.node_id;
  const Vec2d position = request.position;
  if (!finite(position)) return Result<bool>::Fail(ErrorKind::kValidation, "road node position is non-finite");
  const RoadNode* node = find_node(graph_, node_id);
  if (node == nullptr) return Result<bool>::Fail(ErrorKind::kValidation, "road node does not exist");
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  RoadNode replacement = *node;
  replacement.position = position;
  plan.replace_nodes.push_back(replacement);
  return Execute(plan);
}

Result<bool> RoadState::DeleteSegment(DeleteSegmentRequest request) {
  const RoadSegmentId segment_id = request.segment_id;
  const RoadSegment* target = find_segment(graph_, segment_id);
  if (target == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road segment does not exist");
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.remove_segments.push_back(segment_id);
  for (const RoadNodeId node_id : std::array<RoadNodeId, 2>{target->node_a, target->node_b}) {
    if (node_degree(graph_, node_id) == 1) plan.remove_nodes.push_back(node_id);
  }
  for (const NodeConnectionPolicyOverride& policy : graph_.connection_policy_overrides) {
    const std::size_t degree_after = node_degree(graph_, policy.node_id) -
                                     ((target->node_a == policy.node_id || target->node_b == policy.node_id) ? 1 : 0);
    if (degree_after == 0) plan.remove_connection_policy_overrides.push_back(policy.id);
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    if (marking.owner_segment_id == segment_id) plan.remove_manual_lines.push_back(marking.id);
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    if (marking.owner_segment_id == segment_id) plan.remove_manual_areas.push_back(marking.id);
  }
  const std::optional<SectionTransitionId> transition = target->transition;
  if (transition.has_value() &&
      std::none_of(graph_.segments.begin(), graph_.segments.end(), [segment_id, transition](const RoadSegment& segment) {
        return segment.id != segment_id && segment.transition == transition;
      })) {
    plan.remove_transitions.push_back(*transition);
  }
  return Execute(plan);
}

Result<CrossSectionTemplateId> RoadState::AddSectionTemplate(AddSectionTemplateRequest request) {
  CrossSectionTemplate section_template = std::move(request.section_template);
  const Result<bool> valid = validate_section_template(section_template);
  if (!valid.ok) {
    return Result<CrossSectionTemplateId>::Fail(valid.error_kind, valid.error);
  }
  if (section_template.id != 0 && find_template(graph_, section_template.id) != nullptr) {
    return Result<CrossSectionTemplateId>::Fail(ErrorKind::kValidation, "section template id already exists");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (section_template.id == 0) section_template.id = next_id++;
  const CrossSectionTemplateId id = section_template.id;
  plan.next_id_after = next_id;
  plan.add_section_templates.push_back(std::move(section_template));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<CrossSectionTemplateId>::Fail(executed.error_kind, executed.error);
  return Result<CrossSectionTemplateId>::Ok(id);
}

Result<bool> RoadState::EditSectionTemplate(EditSectionTemplateRequest request) {
  CrossSectionTemplate section_template = std::move(request.section_template);
  const Result<bool> valid = validate_section_template(section_template);
  if (!valid.ok) return valid;
  auto it = std::find_if(graph_.section_templates.begin(), graph_.section_templates.end(),
                         [&section_template](const CrossSectionTemplate& item) { return item.id == section_template.id; });
  if (it == graph_.section_templates.end()) {
    return Result<bool>::Fail(ErrorKind::kValidation, "section template does not exist");
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.replace_section_templates.push_back(std::move(section_template));
  return Execute(plan);
}

Result<SectionTransitionId> RoadState::AddTransition(SectionTransitionRequest request) {
  SectionTransition transition{0, request.from_template, request.to_template, request.start, request.end,
                               request.anchor, std::move(request.rules)};
  const Result<bool> valid = validate_transition(graph_, transition);
  if (!valid.ok) {
    return Result<SectionTransitionId>::Fail(valid.error_kind, valid.error);
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (transition.id == 0) transition.id = next_id++;
  const SectionTransitionId id = transition.id;
  plan.next_id_after = next_id;
  plan.add_transitions.push_back(std::move(transition));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<SectionTransitionId>::Fail(executed.error_kind, executed.error);
  return Result<SectionTransitionId>::Ok(id);
}

Result<SectionTransitionId> RoadState::AddTransitionToSegment(AddTransitionToSegmentRequest operation) {
  const RoadSegmentId segment_id = operation.segment_id;
  SectionTransitionRequest request = std::move(operation.transition);
  const RoadSegment* segment = find_segment(graph_, segment_id);
  if (segment == nullptr) {
    return Result<SectionTransitionId>::Fail(ErrorKind::kValidation, "road transition segment does not exist");
  }
  SectionTransition transition{0, segment->section_template, request.to_template, request.start, request.end,
                               request.anchor, std::move(request.rules)};
  const Result<bool> valid = validate_transition(graph_, transition);
  if (!valid.ok) return Result<SectionTransitionId>::Fail(valid.error_kind, valid.error);
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (transition.id == 0) transition.id = next_id++;
  const SectionTransitionId id = transition.id;
  const std::optional<SectionTransitionId> old_transition = segment->transition;
  RoadSegment replacement = *segment;
  replacement.transition = id;
  plan.replace_segments.push_back(std::move(replacement));
  plan.add_transitions.push_back(std::move(transition));
  if (old_transition.has_value()) {
    plan.remove_transitions.push_back(*old_transition);
  }
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<SectionTransitionId>::Fail(executed.error_kind, executed.error);
  return Result<SectionTransitionId>::Ok(id);
}

Result<bool> RoadState::AttachSectionTransition(AttachSectionTransitionRequest request) {
  const RoadSegmentId segment_id = request.segment_id;
  const SectionTransitionId transition_id = request.transition_id;
  const RoadSegment* segment = find_segment(graph_, segment_id);
  const SectionTransition* transition = find_transition(graph_, transition_id);
  if (segment == nullptr || transition == nullptr) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road transition attachment reference is missing");
  }
  if (segment->section_template != transition->from_template) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road transition from-template does not match segment");
  }
  const Path* alignment = FindCanonicalAlignment(derived_, segment_id);
  if (alignment == nullptr) return Result<bool>::Fail(ErrorKind::kInternal, "road segment canonical alignment is missing");
  const auto total = PathLength(*alignment);
  if (!total.ok) return Result<bool>::Fail(total.error_kind, total.error);
  const double start = station_value(transition->start, total.value);
  const double end = station_value(transition->end, total.value);
  if (start < 0.0 || end > total.value || end - start <= kEpsilon) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road transition station range is invalid");
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  RoadSegment replacement = *segment;
  replacement.transition = transition_id;
  plan.replace_segments.push_back(std::move(replacement));
  return Execute(plan);
}

Result<ManualMarkingId> RoadState::AddManualLine(ManualLineRequest request) {
  ManualLineMarking marking{0, request.owner_segment_id, std::move(request.path), request.style_id};
  if (!IsKnownMarkingStyle(marking.style_id)) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual line style is unknown");
  }
  const RoadSegment* owner = find_segment(graph_, marking.owner_segment_id);
  if (owner == nullptr) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual line owner segment does not exist");
  }
  const Result<bool> valid = ValidatePath(marking.path);
  if (!valid.ok) {
    return Result<ManualMarkingId>::Fail(valid.error_kind, valid.error);
  }
  const Path* alignment = FindCanonicalAlignment(derived_, owner->id);
  if (alignment == nullptr) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kInternal, "road segment canonical alignment is missing");
  }
  const auto length_result = PathLength(*alignment);
  for (const Vec2d point : FlattenPath(marking.path)) {
    if (!length_result.ok || point.x < 0.0 || point.x > length_result.value || !finite(point.y)) {
      return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual line lies outside owner station range");
    }
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (marking.id == 0) marking.id = next_id++;
  const ManualMarkingId id = marking.id;
  plan.next_id_after = next_id;
  plan.add_manual_lines.push_back(std::move(marking));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<ManualMarkingId>::Fail(executed.error_kind, executed.error);
  return Result<ManualMarkingId>::Ok(id);
}

Result<ManualMarkingId> RoadState::AddManualArea(ManualAreaRequest request) {
  ManualAreaMarking marking{0, request.owner_segment_id, request.frame_origin, request.width_m, request.length_m,
                            request.style_id};
  if (!IsKnownMarkingStyle(marking.style_id)) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual area style is unknown");
  }
  const RoadSegment* owner = find_segment(graph_, marking.owner_segment_id);
  if (owner == nullptr) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual area owner segment does not exist");
  }
  if (!finite(marking.frame_origin) || !finite(marking.width_m) || !finite(marking.length_m) ||
      marking.width_m <= 0.0 || marking.length_m <= 0.0) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual area shape is invalid");
  }
  const Path* alignment = FindCanonicalAlignment(derived_, owner->id);
  if (alignment == nullptr) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kInternal, "road segment canonical alignment is missing");
  }
  const auto owner_length = PathLength(*alignment);
  if (!owner_length.ok || marking.frame_origin.x - marking.length_m * 0.5 < 0.0 ||
      marking.frame_origin.x + marking.length_m * 0.5 > owner_length.value) {
    return Result<ManualMarkingId>::Fail(ErrorKind::kValidation, "manual area lies outside owner station range");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (marking.id == 0) marking.id = next_id++;
  const ManualMarkingId id = marking.id;
  plan.next_id_after = next_id;
  plan.add_manual_areas.push_back(std::move(marking));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<ManualMarkingId>::Fail(executed.error_kind, executed.error);
  return Result<ManualMarkingId>::Ok(id);
}

Result<bool> RoadState::BuildDerived() {
  Result<DerivedRoad> built = build::BuildRoad(graph_);
  if (!built.ok) {
    return Result<bool>::Fail(built.error_kind, built.error);
  }
  derived_ = std::move(built.value);
  return Result<bool>::Ok(true);
}

Result<std::string> RoadState::Save() const {
  return persistence::SaveRoad(graph_, next_id_);
}

Result<RoadState> RoadState::Load(const std::string& text) {
  Result<persistence::LoadedRoad> loaded = persistence::LoadRoad(text);
  if (!loaded.ok) {
    return Result<RoadState>::Fail(loaded.error_kind, loaded.error);
  }
  RoadState state{};
  state.graph_ = std::move(loaded.value.graph);
  state.next_id_ = loaded.value.next_id;
  Result<bool> rebuilt = state.BuildDerived();
  if (!rebuilt.ok) {
    return Result<RoadState>::Fail(rebuilt.error_kind, rebuilt.error);
  }
  return Result<RoadState>::Ok(std::move(state));
}

Result<bool> ValidateGraphInvariants(const SavedRoadGraph& graph, const DerivedRoad& derived) {
  if (std::any_of(derived.build_stage_runs.begin(), derived.build_stage_runs.end(),
                  [](std::size_t runs) { return runs != 1; })) {
    return Result<bool>::Fail(ErrorKind::kInternal, "road build stage count invariant failed");
  }
  std::unordered_set<std::uint64_t> ids{};
  for (const CrossSectionTemplate& section : graph.section_templates) {
    if (!ids.insert(section.id).second) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road section template ID invariant failed");
    }
  }
  for (const RoadNode& node : graph.nodes) {
    if (!ids.insert(node.id).second || !finite(node.position)) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road node invariant failed");
    }
  }
  for (const RoadSegment& segment : graph.segments) {
    if (!ids.insert(segment.id).second || find_node(graph, segment.node_a) == nullptr ||
        find_node(graph, segment.node_b) == nullptr || find_template(graph, segment.section_template) == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road segment reference invariant failed");
    }
  }
  std::size_t expected_approaches = 0;
  for (const RoadSegment& segment : graph.segments) {
    for (const RoadNodeId node_id :
         std::array<RoadNodeId, 2>{segment.node_a, segment.node_b}) {
      if (node_degree(graph, node_id) >= 2 ||
          find_connection_policy_override(graph, node_id) != nullptr) {
        ++expected_approaches;
      }
    }
  }
  std::size_t decision_approaches = 0;
  for (const NodeConnectionDecision& decision : derived.node_connection_decisions) {
    if (find_node(graph, decision.node_id) == nullptr ||
        decision.approaches.size() != decision.ordered_approaches.size()) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road node connection decision invariant failed");
    }
    decision_approaches += decision.approaches.size();
    for (const ApproachConnectionDecision& approach : decision.approaches) {
      const RoadSegment* segment = find_segment(graph, approach.key.segment_id);
      const bool endpoint_matches =
          segment != nullptr && approach.key.node_id == decision.node_id &&
          ((approach.key.endpoint_role == EndpointRole::kStart &&
            segment->node_a == approach.key.node_id) ||
           (approach.key.endpoint_role == EndpointRole::kEnd &&
            segment->node_b == approach.key.node_id));
      if (!endpoint_matches || approach.endpoint_template_id == 0 ||
          !finite(approach.setback_m) || approach.setback_m < 0.0 ||
          !finite(approach.gate_station_m)) {
        return Result<bool>::Fail(
            ErrorKind::kInternal,
            "road approach connection decision invariant failed");
      }
      const std::size_t same_decision_rows = static_cast<std::size_t>(
          std::count_if(decision.approaches.begin(), decision.approaches.end(),
                        [&approach](const ApproachConnectionDecision& candidate) {
                          return candidate.key == approach.key;
                        }));
      const std::size_t same_order_rows = static_cast<std::size_t>(
          std::count(decision.ordered_approaches.begin(),
                     decision.ordered_approaches.end(), approach.key));
      if (same_decision_rows != 1 || same_order_rows != 1) {
        return Result<bool>::Fail(ErrorKind::kInternal,
                                  "road ApproachKey decision identity invariant failed");
      }
    }
  }
  if (decision_approaches != expected_approaches ||
      derived.setback_calculation_count != expected_approaches ||
      derived.connection_gates.size() != expected_approaches) {
    return Result<bool>::Fail(ErrorKind::kInternal,
                              "road approach single-decision count invariant failed");
  }
  if (derived.section_evaluation_count != derived.section_evaluations.size()) {
    return Result<bool>::Fail(ErrorKind::kInternal,
                              "road section evaluation count invariant failed");
  }
  for (const SectionEvaluation& section : derived.section_evaluations) {
    double previous = -std::numeric_limits<double>::infinity();
    for (const SectionBoundarySample& boundary : section.boundaries) {
      if (!finite(boundary.lateral_m) || !finite(boundary.height_m) || boundary.lateral_m < previous) {
        return Result<bool>::Fail(ErrorKind::kInternal, "section boundary order invariant failed");
      }
      previous = boundary.lateral_m;
    }
  }
  for (const ConnectionGate& gate : derived.connection_gates) {
    const RoadSegment* segment = find_segment(graph, gate.approach.segment_id);
    const bool endpoint_matches =
        segment != nullptr && gate.segment_id == gate.approach.segment_id &&
        gate.node_id == gate.approach.node_id &&
        ((gate.approach.endpoint_role == EndpointRole::kStart &&
          segment->node_a == gate.approach.node_id) ||
         (gate.approach.endpoint_role == EndpointRole::kEnd &&
          segment->node_b == gate.approach.node_id));
    if (!endpoint_matches || !finite(gate.position.x) ||
        !finite(gate.position.y) || !finite(gate.position.z) ||
        !finite(gate.tangent.x) || !finite(gate.tangent.y) ||
        !finite(gate.tangent.z) || !finite(gate.lateral.x) ||
        !finite(gate.lateral.y) || !finite(gate.lateral.z) ||
        !finite(gate.normal.x) || !finite(gate.normal.y) ||
        !finite(gate.normal.z)) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road connection gate frame invariant failed");
    }
    const NodeConnectionDecision* decision = nullptr;
    for (const NodeConnectionDecision& candidate :
         derived.node_connection_decisions) {
      if (candidate.node_id == gate.approach.node_id) {
        if (decision != nullptr) {
          return Result<bool>::Fail(
              ErrorKind::kInternal,
              "road node has duplicate connection decisions");
        }
        decision = &candidate;
      }
    }
    if (decision == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road connection gate decision is missing");
    }
    const auto approach = std::find_if(
        decision->approaches.begin(), decision->approaches.end(),
        [&gate](const ApproachConnectionDecision& candidate) {
          return candidate.key == gate.approach;
        });
    if (approach == decision->approaches.end()) {
      return Result<bool>::Fail(ErrorKind::kInternal,
                                "road connection gate approach is missing");
    }
    const SectionEvaluation* matched_section = nullptr;
    for (const SectionEvaluation& section : derived.section_evaluations) {
      if (section.segment_id != gate.approach.segment_id ||
          std::abs(section.station_m - approach->gate_station_m) > kEpsilon) {
        continue;
      }
      if (matched_section != nullptr) {
        return Result<bool>::Fail(
            ErrorKind::kInternal,
            "road connection gate has duplicate SectionEvaluation rows");
      }
      matched_section = &section;
    }
    if (matched_section == nullptr ||
        matched_section->boundaries.size() != gate.boundaries.size()) {
      return Result<bool>::Fail(
          ErrorKind::kInternal,
          "road connection gate SectionEvaluation invariant failed");
    }
    for (std::size_t index = 0; index < gate.boundaries.size(); ++index) {
      const SectionBoundarySample& gate_boundary = gate.boundaries[index];
      const SectionBoundarySample& section_boundary =
          matched_section->boundaries[index];
      if (gate_boundary.boundary_id != section_boundary.boundary_id ||
          gate_boundary.role != section_boundary.role ||
          gate_boundary.lateral_m != section_boundary.lateral_m ||
          gate_boundary.height_m != section_boundary.height_m ||
          gate_boundary.marking_rule != section_boundary.marking_rule) {
        return Result<bool>::Fail(
            ErrorKind::kInternal,
            "road connection gate boundary copy invariant failed");
      }
    }
  }
  for (const NodeConnectionPolicyOverride& policy : graph.connection_policy_overrides) {
    if (!ids.insert(policy.id).second || find_node(graph, policy.node_id) == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road connection policy override invariant failed");
    }
  }
  for (const SectionTransition& transition : graph.transitions) {
    if (!ids.insert(transition.id).second) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road transition ID invariant failed");
    }
  }
  for (const ManualLineMarking& marking : graph.manual_lines) {
    if (!ids.insert(marking.id).second || find_segment(graph, marking.owner_segment_id) == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal, "manual line ID or owner invariant failed");
    }
  }
  for (const ManualAreaMarking& marking : graph.manual_areas) {
    if (!ids.insert(marking.id).second || find_segment(graph, marking.owner_segment_id) == nullptr) {
      return Result<bool>::Fail(ErrorKind::kInternal, "manual area ID or owner invariant failed");
    }
  }
  const auto valid_mesh = [](const Mesh& mesh) {
    for (const Vec3d& vertex : mesh.vertices) {
      if (!finite(vertex.x) || !finite(vertex.y) || !finite(vertex.z)) {
        return false;
      }
    }
    for (std::uint32_t index : mesh.indices) {
      if (index >= mesh.vertices.size()) return false;
    }
    return true;
  };
  const std::array<const std::vector<Mesh>*, 6> mesh_families{
      &derived.segment_meshes, &derived.marking_meshes, &derived.connection_meshes,
      &derived.junction_meshes, &derived.junction_marking_meshes, &derived.manual_marking_meshes};
  for (const auto* family : mesh_families) {
    if (std::any_of(family->begin(), family->end(), [&valid_mesh](const Mesh& mesh) { return !valid_mesh(mesh); })) {
      return Result<bool>::Fail(ErrorKind::kInternal, "road mesh contains invalid geometry");
    }
  }
  return Result<bool>::Ok(true);
}

Result<bool> ValidatePath(const Path& path) {
  if (path.spans.empty()) {
    return Result<bool>::Fail(ErrorKind::kValidation, "road path has no spans");
  }
  for (std::size_t i = 0; i < path.spans.size(); ++i) {
    const BezierSpan& span = path.spans[i];
    if (!finite(span.p0) || !finite(span.p1) || !finite(span.p2) || !finite(span.p3)) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road path contains a non-finite control point");
    }
    if (span_length(span) <= kEpsilon) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road span has zero length");
    }
    if (i > 0 && !almost_same(span_start(span), span_end(path.spans[i - 1]))) {
      return Result<bool>::Fail(ErrorKind::kValidation, "road path spans are not continuous");
    }
  }
  return validate_no_self_intersection(path);
}

Result<double> PathLength(const Path& path) {
  const Result<bool> valid = ValidatePath(path);
  if (!valid.ok) {
    return Result<double>::Fail(valid.error_kind, valid.error);
  }
  double out = 0.0;
  for (const BezierSpan& span : path.spans) {
    out += span_length(span);
  }
  return Result<double>::Ok(out);
}

Result<Vec2d> EvaluatePath(const Path& path, double station_m) {
  const Result<double> length_result = PathLength(path);
  if (!length_result.ok) {
    return Result<Vec2d>::Fail(length_result.error_kind, length_result.error);
  }
  double remaining = std::clamp(station_m, 0.0, length_result.value);
  for (const BezierSpan& span : path.spans) {
    const double len = span_length(span);
    if (remaining <= len || &span == &path.spans.back()) {
      return Result<Vec2d>::Ok(span_eval(span, span_parameter_at_length(span, remaining)));
    }
    remaining -= len;
  }
  return Result<Vec2d>::Fail(ErrorKind::kInternal, "road path evaluation fell through");
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

} // namespace city::road
