#include "city/road/road.hpp"

#include "generation/generation.hpp"
#include "geometry/geometry.hpp"
#include "geometry/section.hpp"
#include "lookup.hpp"
#include "operations/operation_plan.hpp"
#include "persistence/road_archive.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <unordered_map>
#include <unordered_set>

namespace city::road {
namespace {

constexpr double kEpsilon = 1e-9;
constexpr int kCurveSamples = 24;
constexpr double kP1MinSegmentLengthM = 8.0;
constexpr double kSnapDistancePointToleranceM = 0.6;

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

[[nodiscard]] bool linear_span_controls_match(Vec2d start, Vec2d end,
                                              const BezierSpan& span) {
  const Vec2d delta = sub(end, start);
  return almost_same(span.p0, start) &&
         almost_same(span.p1, add(start, mul(delta, 1.0 / 3.0))) &&
         almost_same(span.p2, add(start, mul(delta, 2.0 / 3.0))) &&
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
  return add(add(mul(span.p0, u * u * u), mul(span.p1, 3.0 * u * u * t)),
             add(mul(span.p2, 3.0 * u * t * t), mul(span.p3, t * t * t)));
}

[[nodiscard]] double span_speed(const BezierSpan& span, double t) {
  const double u = 1.0 - t;
  const Vec2d a = sub(span.p1, span.p0);
  const Vec2d b = sub(span.p2, span.p1);
  const Vec2d c = sub(span.p3, span.p2);
  const Vec2d derivative =
      mul(add(add(mul(a, u * u), mul(b, 2.0 * u * t)),
              mul(c, t * t)),
          3.0);
  return length(derivative);
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

[[nodiscard]] Result<PathSplit> split_path_at_distance(
    const Path& path, double distance_along_path_m) {
  const Result<double> total = PathLength(path);
  if (!total.ok) return Result<PathSplit>::Fail(total.failure_category, total.error);
  if (!finite(distance_along_path_m) ||
      distance_along_path_m <= kEpsilon ||
      distance_along_path_m >= total.value - kEpsilon) {
    return Result<PathSplit>::Fail(CommitFailureCategory::kInvalidInput, "road segment split distance is outside its interior");
  }
  double remaining = distance_along_path_m;
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
        return Result<bool>::Fail(CommitFailureCategory::kRequirementConstraint,
                                  "road path self-intersection is prohibited",
                                  "road_path_self_intersection");
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

[[nodiscard]] const ApproachGeometryOverride* find_approach_geometry_override(const SavedRoadGraph& graph,
                                                                               const ApproachKey& key) {
  const auto it = std::find_if(graph.approach_geometry_overrides.begin(), graph.approach_geometry_overrides.end(),
                               [&key](const ApproachGeometryOverride& item) { return item.key == key; });
  return it == graph.approach_geometry_overrides.end() ? nullptr : &*it;
}

[[nodiscard]] bool endpoint_matches(const RoadSegment& segment, const ApproachKey& key) {
  return key.segment_id == segment.id &&
         ((key.endpoint_role == EndpointRole::kStart && key.node_id == segment.node_a) ||
          (key.endpoint_role == EndpointRole::kEnd && key.node_id == segment.node_b));
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


[[nodiscard]] const SectionStrip* find_strip(const CrossSectionTemplate& section, SectionStripId id) {
  const auto it = std::find_if(section.strips.begin(), section.strips.end(),
                               [id](const SectionStrip& strip) { return strip.id == id; });
  return it == section.strips.end() ? nullptr : &*it;
}

[[nodiscard]] BoundaryProfile* find_boundary(CrossSectionTemplate& section, std::uint64_t id) {
  const auto it = std::find_if(section.boundaries.begin(), section.boundaries.end(),
                               [id](const BoundaryProfile& boundary) { return boundary.boundary_id == id; });
  return it == section.boundaries.end() ? nullptr : &*it;
}

[[nodiscard]] const BoundaryProfile* find_boundary(const CrossSectionTemplate& section, std::uint64_t id) {
  const auto it = std::find_if(section.boundaries.begin(), section.boundaries.end(),
                               [id](const BoundaryProfile& boundary) { return boundary.boundary_id == id; });
  return it == section.boundaries.end() ? nullptr : &*it;
}

[[nodiscard]] bool valid_marking_role(MarkingRole role) {
  return static_cast<int>(role) >= 0 && static_cast<int>(role) <= 5;
}

[[nodiscard]] Result<bool> validate_marking_policy(const AutoMarkingPolicy& policy) {
  if (!valid_marking_role(policy.role) ||
      (policy.enabled && !IsKnownMarkingStyle(policy.style_id))) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "marking policy is invalid");
  }
  return Result<bool>::Ok(true);
}

[[nodiscard]] Result<bool> validate_auto_marking_key(const SavedRoadGraph& graph,
                                                     const AutoMarkingKey& key) {
  if (!valid_marking_role(key.role)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "auto marking key role is invalid");
  }
  if (key.owner.kind == MarkingOwner::Kind::kRoadSegment) {
    const RoadSegment* segment = find_segment(graph, key.owner.segment_id);
    if (segment == nullptr || key.owner.node_id != 0 || key.owner.manual_id != 0) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "auto marking segment owner is invalid");
    }
    if (!key.track.has_value() || key.approach.has_value() ||
        key.track->segment_id != segment->id || key.track->role != key.role) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "auto marking segment key is invalid");
    }
    const CrossSectionTemplate* section = find_template(graph, segment->section_template);
    if (section == nullptr || find_boundary(*section, key.track->boundary_id) == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "auto marking boundary is missing");
    }
    return Result<bool>::Ok(true);
  }
  if (key.owner.kind == MarkingOwner::Kind::kJunction) {
    if (find_node(graph, key.owner.node_id) == nullptr || key.owner.segment_id != 0 ||
        key.owner.manual_id != 0 || key.track.has_value() || !key.approach.has_value()) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "auto marking junction key is invalid");
    }
    const RoadSegment* segment = find_segment(graph, key.approach->segment_id);
    if (segment == nullptr || !endpoint_matches(*segment, *key.approach) ||
        key.approach->node_id != key.owner.node_id) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "auto marking junction approach is invalid");
    }
    return Result<bool>::Ok(true);
  }
  return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "auto marking manual owner cannot be suppressed");
}

[[nodiscard]] Result<bool> validate_junction_marking_endpoint(const SavedRoadGraph& graph,
                                                              RoadNodeId node_id,
                                                              const JunctionMarkingEndpoint& endpoint) {
  if (!valid_marking_role(endpoint.role)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "junction marking endpoint role is invalid");
  }
  const RoadSegment* segment = find_segment(graph, endpoint.approach.segment_id);
  if (segment == nullptr || endpoint.approach.node_id != node_id ||
      !endpoint_matches(*segment, endpoint.approach)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "junction marking endpoint approach is invalid");
  }
  const CrossSectionTemplate* section = find_template(graph, segment->section_template);
  if (section == nullptr || find_boundary(*section, endpoint.boundary_id) == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "junction marking endpoint boundary is missing");
  }
  return Result<bool>::Ok(true);
}

[[nodiscard]] Result<bool> validate_junction_marking_override(const SavedRoadGraph& graph,
                                                              const JunctionMarkingOverride& override) {
  if (override.id == 0 || find_node(graph, override.node_id) == nullptr ||
      static_cast<int>(override.action) < 0 || static_cast<int>(override.action) > 2) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "junction marking override is invalid");
  }
  Result<bool> source = validate_junction_marking_endpoint(graph, override.node_id, override.source);
  if (!source.ok) return source;
  if (override.action == JunctionMarkingAction::kConnectToApproach) {
    if (!override.target.has_value()) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "junction marking override target is missing");
    }
    return validate_junction_marking_endpoint(graph, override.node_id, *override.target);
  }
  if (override.target.has_value()) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "junction marking override target is invalid");
  }
  return Result<bool>::Ok(true);
}


[[nodiscard]] double distance_value(DistanceRef ref, double total) {
  if (ref.kind == DistanceRefKind::kFromEnd) {
    return total - ref.value;
  }
  if (ref.kind == DistanceRefKind::kRatio) {
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
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section transition references a missing template");
  }
  if (!finite(transition.start.value) || !finite(transition.end.value)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section transition distance is not finite");
  }
  const auto valid_distance = [](DistanceRef distance) {
    return distance.value >= 0.0 &&
           (distance.kind != DistanceRefKind::kRatio || distance.value <= 1.0);
  };
  if (!valid_distance(transition.start) || !valid_distance(transition.end)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section transition distance reference is outside its range");
  }
  if (transition.anchor == TransitionAnchor::kBoundary) {
    const BoundaryProfile* from_anchor =
        find_boundary(*from, transition.anchor_boundary_id);
    const BoundaryProfile* to_anchor =
        find_boundary(*to, transition.anchor_boundary_id);
    if (transition.anchor_boundary_id == 0 || from_anchor == nullptr ||
        to_anchor == nullptr) {
      return Result<bool>::Fail(
          CommitFailureCategory::kInvalidInput,
          "section transition anchor boundary is missing");
    }
    if (!IsSinglePositionBoundary(*from_anchor) ||
        !IsSinglePositionBoundary(*to_anchor)) {
      return Result<bool>::Fail(
          CommitFailureCategory::kNotImplemented,
          "section transition anchor boundary must have one position");
    }
  } else if (transition.anchor_boundary_id != 0) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInvalidInput,
        "section transition boundary anchor is set for a different anchor mode");
  }
  if (transition.rules.empty()) {
    return Result<bool>::Fail(CommitFailureCategory::kNotImplemented, "section transition must define element actions");
  }
  std::unordered_set<SectionStripId> ruled_strips{};
  for (const SectionTransitionRule& rule : transition.rules) {
    if (rule.strip_id == 0 || !ruled_strips.insert(rule.strip_id).second ||
        (find_strip(*from, rule.strip_id) == nullptr && find_strip(*to, rule.strip_id) == nullptr)) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section transition rule strip is invalid");
    }
    if (rule.action == TransitionAction::kUnsupported) {
      return Result<bool>::Fail(CommitFailureCategory::kNotImplemented, "section transition contains unsupported element action");
    }
  }
  const auto action_for = [&transition](SectionStripId id) -> std::optional<TransitionAction> {
    const auto it = std::find_if(transition.rules.begin(), transition.rules.end(),
                                 [id](const SectionTransitionRule& rule) { return rule.strip_id == id; });
    return it == transition.rules.end() ? std::nullopt : std::optional<TransitionAction>(it->action);
  };
  for (const SectionStrip& strip : to->strips) {
    if (find_strip(*from, strip.id) == nullptr && action_for(strip.id) != TransitionAction::kTaperIn) {
      return Result<bool>::Fail(CommitFailureCategory::kNotImplemented, "appearing section strip requires TaperIn");
    }
  }
  for (const SectionStrip& strip : from->strips) {
    if (find_strip(*to, strip.id) != nullptr) continue;
    const std::optional<TransitionAction> action = action_for(strip.id);
    const TransitionAction required = strip.function == StripFunction::kMedian ? TransitionAction::kEndCap
                                                                              : TransitionAction::kTaperOut;
    if (action != required) {
      return Result<bool>::Fail(CommitFailureCategory::kNotImplemented,
                                strip.function == StripFunction::kMedian
                                    ? "disappearing median requires EndCap"
                                    : "disappearing section strip requires TaperOut");
    }
  }
  return Result<bool>::Ok(true);
}

[[nodiscard]] Result<bool> validate_section_template(const CrossSectionTemplate& section) {
  if (section.strips.empty() || section.boundaries.size() + 1 != section.strips.size()) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template chain is incomplete");
  }
  std::unordered_set<SectionStripId> ids{};
  for (const SectionStrip& strip : section.strips) {
    if (strip.id == 0 || !ids.insert(strip.id).second || !finite(strip.width_m) ||
        !finite(strip.cross_slope) || strip.width_m <= 0.0 ||
        static_cast<int>(strip.function) < 0 || static_cast<int>(strip.function) > 3) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template strip is invalid");
    }
    if (!IsKnownSurfaceStyle(strip.style_id)) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template surface style is unknown");
    }
    for (const AutoMarkingPolicy& side : {strip.side_marking.left, strip.side_marking.right}) {
      if (!validate_marking_policy(side).ok) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "section template lane side marking policy is invalid");
      }
    }
  }
  std::unordered_set<LaneId> lane_ids{};
  for (const LaneBand& lane : section.lane_bands) {
    const SectionStrip* strip = find_strip(section, lane.surface_strip_id);
    if (lane.id == 0 || !lane_ids.insert(lane.id).second || strip == nullptr ||
        strip->function != StripFunction::kCarriageway ||
        !finite(lane.lateral_start_m) || !finite(lane.lateral_end_m) ||
        lane.lateral_start_m < 0.0 ||
        lane.lateral_end_m <= lane.lateral_start_m ||
        lane.lateral_end_m > strip->width_m) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "section template lane allocation is invalid");
    }
  }
  ids.clear();
  for (const BoundaryProfile& boundary : section.boundaries) {
    if (boundary.boundary_id == 0 || !ids.insert(boundary.boundary_id).second || !finite(boundary.width_m) ||
        !finite(boundary.height_m) || boundary.width_m < 0.0) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template boundary is invalid");
    }
    if (static_cast<int>(boundary.marking.role) < 0 ||
        static_cast<int>(boundary.marking.role) > 5 ||
        (boundary.marking.enabled &&
         !IsKnownMarkingStyle(boundary.marking.style_id))) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                "section template marking policy is invalid");
    }
  }
  return Result<bool>::Ok(true);
}

} // namespace

const RoadCorridor* FindRoadCorridor(const SavedRoadGraph& graph,
                                     RoadCorridorId corridor_id) {
  const auto it =
      std::find_if(graph.corridors.begin(), graph.corridors.end(),
                   [corridor_id](const RoadCorridor& corridor) {
                     return corridor.id == corridor_id;
                   });
  return it == graph.corridors.end() ? nullptr : &*it;
}

const RoadCorridor* FindCorridorForSegment(const SavedRoadGraph& graph,
                                           RoadSegmentId segment_id) {
  const auto it =
      std::find_if(graph.corridors.begin(), graph.corridors.end(),
                   [segment_id](const RoadCorridor& corridor) {
                     return std::any_of(
                         corridor.segments.begin(), corridor.segments.end(),
                         [segment_id](const DirectedSegmentRef& ref) {
                           return ref.segment_id == segment_id;
                         });
                   });
  return it == graph.corridors.end() ? nullptr : &*it;
}

Result<ResolvedSegmentDistance>
ResolveCorridorDistance(const SavedRoadGraph& graph, const DerivedRoad& derived,
                       CorridorDistanceRef distance) {
  if (!finite(distance.corridor_distance_m) ||
      distance.corridor_distance_m < 0.0) {
    return Result<ResolvedSegmentDistance>::Fail(
        CommitFailureCategory::kInvalidInput, "road corridor distance is invalid");
  }
  const RoadCorridor* corridor =
      FindRoadCorridor(graph, distance.corridor_id);
  if (corridor == nullptr || corridor->segments.empty()) {
    return Result<ResolvedSegmentDistance>::Fail(
        CommitFailureCategory::kInvalidInput, "road corridor does not exist");
  }
  double accumulated = 0.0;
  for (std::size_t index = 0; index < corridor->segments.size(); ++index) {
    const DirectedSegmentRef& ref = corridor->segments[index];
    const DerivedSegment* segment = FindDerivedSegment(derived, ref.segment_id);
    if (segment == nullptr || !finite(segment->length_m) ||
        segment->length_m <= 0.0) {
      return Result<ResolvedSegmentDistance>::Fail(
          CommitFailureCategory::kInternalError, "road corridor segment length is missing");
    }
    const double end = accumulated + segment->length_m;
    const bool last = index + 1 == corridor->segments.size();
    if (distance.corridor_distance_m < end ||
        (last && distance.corridor_distance_m <= end + kEpsilon)) {
      const double corridor_local =
          std::clamp(distance.corridor_distance_m - accumulated, 0.0,
                     segment->length_m);
      return Result<ResolvedSegmentDistance>::Ok(ResolvedSegmentDistance{
          ref.segment_id,
          ref.reversed ? segment->length_m - corridor_local : corridor_local,
          ref.reversed});
    }
    accumulated = end;
  }
  return Result<ResolvedSegmentDistance>::Fail(
      CommitFailureCategory::kInvalidInput, "road corridor distance exceeds its length");
}

Result<RoadSideRef>
ResolveCorridorSideRef(const SavedRoadGraph& graph, const DerivedRoad& derived,
                       CorridorSideRef reference) {
  if (!finite(reference.lateral_offset_m) ||
      reference.lateral_offset_m < 0.0) {
    return Result<RoadSideRef>::Fail(
        CommitFailureCategory::kInvalidInput, "road corridor side offset is invalid");
  }
  const Result<ResolvedSegmentDistance> resolved = ResolveCorridorDistance(
      graph, derived,
      CorridorDistanceRef{reference.corridor_id,
                          reference.corridor_distance_m});
  if (!resolved.ok) {
    return Result<RoadSideRef>::Fail(resolved.failure_category, resolved.error);
  }
  const RoadSide segment_side =
      resolved.value.reversed
          ? (reference.side == RoadSide::kLeft ? RoadSide::kRight
                                               : RoadSide::kLeft)
          : reference.side;
  return Result<RoadSideRef>::Ok(
      RoadSideRef{resolved.value.segment_id, segment_side,
                  resolved.value.segment_distance_m, reference.lateral_offset_m});
}

Result<Vec3d> ResolveRoadSidePosition(const DerivedRoad& derived,
                                      RoadSideRef reference) {
  if (!finite(reference.segment_distance_m) ||
      !finite(reference.lateral_offset_m) ||
      reference.segment_distance_m < 0.0 ||
      reference.lateral_offset_m < 0.0) {
    return Result<Vec3d>::Fail(CommitFailureCategory::kInvalidInput,
                               "road side reference is invalid");
  }
  const Path* alignment =
      FindCanonicalAlignment(derived, reference.segment_id);
  if (alignment == nullptr) {
    return Result<Vec3d>::Fail(CommitFailureCategory::kInvalidInput,
                               "road side segment does not exist");
  }
  const Result<Vec2d> center =
      EvaluatePath(*alignment, reference.segment_distance_m);
  const Result<Vec2d> tangent =
      internal::tangent_at(*alignment, reference.segment_distance_m);
  if (!center.ok) {
    return Result<Vec3d>::Fail(center.failure_category, center.error);
  }
  if (!tangent.ok) {
    return Result<Vec3d>::Fail(tangent.failure_category, tangent.error);
  }
  const double sign = reference.side == RoadSide::kLeft ? 1.0 : -1.0;
  return Result<Vec3d>::Ok(
      Vec3d{center.value.x - tangent.value.y *
                                 reference.lateral_offset_m * sign,
            center.value.y + tangent.value.x *
                                 reference.lateral_offset_m * sign,
            0.0});
}

Result<std::vector<double>>
DeriveRepeatingPlacementDistances(const SavedRoadGraph& graph,
                                 const DerivedRoad& derived,
                                 RoadCorridorId corridor_id,
                                 RepeatingPlacementPolicy policy) {
  if (!finite(policy.spacing_m) || !finite(policy.phase_m) ||
      policy.spacing_m <= 0.0 || policy.phase_m < 0.0) {
    return Result<std::vector<double>>::Fail(
        CommitFailureCategory::kInvalidInput, "road repeating placement policy is invalid");
  }
  const RoadCorridor* corridor = FindRoadCorridor(graph, corridor_id);
  if (corridor == nullptr || corridor->segments.empty()) {
    return Result<std::vector<double>>::Fail(
        CommitFailureCategory::kInvalidInput, "road corridor does not exist");
  }
  double total = 0.0;
  for (const DirectedSegmentRef& ref : corridor->segments) {
    const DerivedSegment* segment = FindDerivedSegment(derived, ref.segment_id);
    if (segment == nullptr || !finite(segment->length_m) ||
        segment->length_m <= 0.0) {
      return Result<std::vector<double>>::Fail(
          CommitFailureCategory::kInternalError, "road corridor segment length is missing");
    }
    total += segment->length_m;
  }
  std::vector<double> distances{};
  for (double value = policy.phase_m; value <= total + kEpsilon;
       value += policy.spacing_m) {
    distances.push_back(std::min(value, total));
    if (distances.size() > 1000000) {
      return Result<std::vector<double>>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road repeating placement count is unsupported");
    }
  }
  return Result<std::vector<double>>::Ok(std::move(distances));
}

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

Result<SegmentShape> SegmentShapeFromPath(const Path& path) {
  const Result<bool> valid = ValidatePath(path);
  if (!valid.ok) return Result<SegmentShape>::Fail(valid.failure_category, valid.error);
  SegmentShape shape{};
  if (path.spans.size() == 1 &&
      linear_span_controls_match(path.spans.front().p0, path.spans.front().p3,
                                 path.spans.front())) {
    shape.intent = SegmentShapeIntent::kStraight;
  }
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

Result<Path> DeriveCanonicalAlignment(Vec2d start, Vec2d end, const SegmentShape& shape) {
  if (!finite(start) || !finite(end) || !finite(shape.start_handle) || !finite(shape.end_handle)) {
    return Result<Path>::Fail(CommitFailureCategory::kInvalidInput, "road segment shape contains a non-finite point");
  }
  Path path{};
  Vec2d span_start = start;
  Vec2d handle_out = shape.start_handle;
  for (const SegmentKnot& knot : shape.internal_knots) {
    if (!finite(knot.position) || !finite(knot.handle_in) || !finite(knot.handle_out)) {
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

bool IsLinearSpan(const BezierSpan& span) {
  return almost_same(span.p1, add(span.p0, mul(sub(span.p3, span.p0), 1.0 / 3.0))) &&
         almost_same(span.p2, add(span.p0, mul(sub(span.p3, span.p0), 2.0 / 3.0)));
}

void align_first_span_start(Path& path, Vec2d start) {
  BezierSpan& first_span = path.spans.front();
  if (IsLinearSpan(first_span)) {
    first_span = MakeLine(start, first_span.p3);
    return;
  }
  const Vec2d correction = sub(start, first_span.p0);
  first_span.p0 = start;
  first_span.p1 = add(first_span.p1, correction);
}

Result<SegmentShape> make_linear_shape(Vec2d start, Vec2d end) {
  return SegmentShapeFromPath(MakePath({MakeLine(start, end)}));
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
  const AutoMarkingPolicy outer_line{
      true, MarkingRole::kCarriagewayEdge, builtin_marking_styles::kWhiteSolid};
  const AutoMarkingPolicy center_line{
      true, MarkingRole::kCenterLine, builtin_marking_styles::kCenterLine};
  CrossSectionTemplate section{};
  section.id = id;
  section.strips = {
      {10, StripFunction::kSidewalk, 2.0, 0.01, builtin_surface_styles::kSidewalk},
      {20, StripFunction::kCarriageway, 3.0, 0.02, builtin_surface_styles::kAsphalt},
      {30, StripFunction::kCarriageway, 3.0, -0.02, builtin_surface_styles::kAsphalt},
      {40, StripFunction::kSidewalk, 2.0, -0.01, builtin_surface_styles::kSidewalk},
  };
  section.lane_bands = {
      {1000, 20, 0.0, 3.0, LaneTravelDirection::kAgainstSegment},
      {1010, 30, 0.0, 3.0, LaneTravelDirection::kAlongSegment},
  };
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, outer_line},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, center_line},
      {300, BoundaryRole::kCurb, 0.2, 0.15, outer_line},
  };
  return section;
}

CrossSectionTemplate ThreeLaneTemplate(CrossSectionTemplateId id) {
  const AutoMarkingPolicy outer_line{
      true, MarkingRole::kCarriagewayEdge, builtin_marking_styles::kWhiteSolid};
  const AutoMarkingPolicy center_line{
      true, MarkingRole::kCenterLine, builtin_marking_styles::kCenterLine};
  CrossSectionTemplate section = JapaneseUrbanTwoLaneTemplate(id);
  section.strips = {
      {10, StripFunction::kSidewalk, 2.0, 0.01, builtin_surface_styles::kSidewalk},
      {20, StripFunction::kCarriageway, 3.0, 0.02, builtin_surface_styles::kAsphalt},
      {30, StripFunction::kCarriageway, 3.0, 0.0, builtin_surface_styles::kAsphalt},
      {35, StripFunction::kCarriageway, 3.0, -0.02, builtin_surface_styles::kAsphalt},
      {40, StripFunction::kSidewalk, 2.0, -0.01, builtin_surface_styles::kSidewalk},
  };
  section.lane_bands = {
      {1000, 20, 0.0, 3.0, LaneTravelDirection::kAgainstSegment},
      {1010, 30, 0.0, 3.0, LaneTravelDirection::kAlongSegment},
      {1020, 35, 0.0, 3.0, LaneTravelDirection::kAlongSegment},
  };
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, outer_line},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, center_line},
      {250, BoundaryRole::kLaneDivider, 0.0, 0.0, center_line},
      {300, BoundaryRole::kCurb, 0.2, 0.15, outer_line},
  };
  return section;
}

CrossSectionTemplate NoLeftSidewalkTemplate(CrossSectionTemplateId id) {
  CrossSectionTemplate section = JapaneseUrbanTwoLaneTemplate(id);
  section.strips.erase(section.strips.begin());
  section.boundaries.erase(section.boundaries.begin());
  return section;
}

CrossSectionTemplate MedianTwoLaneTemplate(CrossSectionTemplateId id) {
  const AutoMarkingPolicy outer_line{
      true, MarkingRole::kCarriagewayEdge, builtin_marking_styles::kWhiteSolid};
  CrossSectionTemplate section = JapaneseUrbanTwoLaneTemplate(id);
  section.strips.insert(section.strips.begin() + 2,
                       {25, StripFunction::kMedian, 2.0, 0.0, builtin_surface_styles::kMedian});
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, outer_line},
      {210, BoundaryRole::kMedianEdge, 0.2, 0.12, {}},
      {220, BoundaryRole::kMedianEdge, 0.2, -0.12, {}},
      {300, BoundaryRole::kCurb, 0.2, 0.15, outer_line},
  };
  return section;
}

CrossSectionTemplate ShoulderedTwoLaneTemplate(CrossSectionTemplateId id) {
  const AutoMarkingPolicy outer_line{
      true, MarkingRole::kCarriagewayEdge,
      builtin_marking_styles::kWhiteSolid};
  const AutoMarkingPolicy center_line{
      true, MarkingRole::kCenterLine,
      builtin_marking_styles::kCenterLine};
  CrossSectionTemplate section{};
  section.id = id;
  section.strips = {
      {10, StripFunction::kSidewalk, 2.0, 0.01,
       builtin_surface_styles::kSidewalk},
      {15, StripFunction::kShoulder, 0.75, 0.02,
       builtin_surface_styles::kAsphalt},
      {20, StripFunction::kCarriageway, 3.0, 0.02,
       builtin_surface_styles::kAsphalt},
      {30, StripFunction::kCarriageway, 3.0, -0.02,
       builtin_surface_styles::kAsphalt},
      {35, StripFunction::kShoulder, 0.75, -0.02,
       builtin_surface_styles::kAsphalt},
      {40, StripFunction::kSidewalk, 2.0, -0.01,
       builtin_surface_styles::kSidewalk},
  };
  section.lane_bands = {
      {1000, 20, 0.0, 3.0, LaneTravelDirection::kAgainstSegment},
      {1010, 30, 0.0, 3.0, LaneTravelDirection::kAlongSegment},
  };
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, {}},
      {150, BoundaryRole::kOuterEdge, 0.0, 0.0, outer_line},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, center_line},
      {250, BoundaryRole::kOuterEdge, 0.0, 0.0, outer_line},
      {300, BoundaryRole::kCurb, 0.2, 0.15, {}},
  };
  return section;
}

RoadState::RoadState() {
  graph_.section_templates.push_back(JapaneseUrbanTwoLaneTemplate(1));
  graph_.section_templates.push_back(ThreeLaneTemplate(2));
  graph_.section_templates.push_back(NoLeftSidewalkTemplate(3));
  graph_.section_templates.push_back(MedianTwoLaneTemplate(4));
  graph_.section_templates.push_back(ShoulderedTwoLaneTemplate(5));
  next_id_ = 6;
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
  Result<DerivedRoad> generated = generation::generate_road(trial.graph_);
  if (!generated.ok) {
    return Result<bool>::Fail(generated.failure_category, generated.error);
  }
  trial.derived_ = std::move(generated.value);
  const Result<bool> derived_valid = ValidateGraphInvariants(trial.graph_, trial.derived_);
  if (!derived_valid.ok) return derived_valid;
  *this = std::move(trial);
  return Result<bool>::Ok(true);
}

Result<RoadSegmentId> RoadState::AddSegment(AddSegmentRequest request) {
  Path alignment = std::move(request.alignment);
  const CrossSectionTemplateId section_template = request.section_template;
  const Result<bool> path_valid = ValidatePath(alignment);
  if (!path_valid.ok) {
    return Result<RoadSegmentId>::Fail(path_valid.failure_category,
                                       path_valid.error,
                                       path_valid.reason_code);
  }
  if (find_template(graph_, section_template) == nullptr) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kInvalidInput, "road segment references a missing section template");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId node_a = next_id++;
  const RoadNodeId node_b = next_id++;
  const RoadSegmentId segment_id = next_id++;
  const RoadCorridorId corridor_id = next_id++;
  plan.add_nodes = {RoadNode{node_a, alignment.spans.front().p0},
                    RoadNode{node_b, alignment.spans.back().p3}};
  const Result<SegmentShape> shape = SegmentShapeFromPath(alignment);
  if (!shape.ok) {
    return Result<RoadSegmentId>::Fail(shape.failure_category, shape.error);
  }
  plan.add_segments.push_back(RoadSegment{
      segment_id, node_a, node_b, shape.value, section_template,
      std::nullopt});
  plan.add_corridors.push_back(RoadCorridor{
      corridor_id, section_template,
      {DirectedSegmentRef{segment_id, false}}});
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.failure_category, executed.error);
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::ExtendCorridorFromEnd(
    ExtendCorridorFromEndRequest request) {
  const RoadCorridor* source_corridor =
      FindRoadCorridor(graph_, request.corridor_id);
  const RoadNode* endpoint = find_node(graph_, request.endpoint_node_id);
  if (source_corridor == nullptr || source_corridor->segments.empty() ||
      endpoint == nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road extension corridor or endpoint does not exist");
  }
  const DirectedSegmentRef last_ref = source_corridor->segments.back();
  const RoadSegment* source = find_segment(graph_, last_ref.segment_id);
  if (source == nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInternalError, "road corridor terminal segment is missing");
  }
  const RoadNodeId corridor_end =
      last_ref.reversed ? source->node_a : source->node_b;
  if (corridor_end != endpoint->id) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road extension node is not the corridor end");
  }
  if (node_degree(graph_, endpoint->id) != 1) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road extension requires a degree-one endpoint");
  }
  const EndpointRole source_endpoint_role =
      last_ref.reversed ? EndpointRole::kStart : EndpointRole::kEnd;
  const CrossSectionTemplate* source_endpoint_section =
      internal::find_endpoint_template(graph_, *source, source_endpoint_role);
  const CrossSectionTemplate* extension_section =
      find_template(graph_, request.section_template);
  if (source_endpoint_section == nullptr || extension_section == nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road extension endpoint section does not exist");
  }
  if (!internal::equivalent_section_definition(*source_endpoint_section,
                                               *extension_section)) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road extension endpoint lane layouts differ");
  }
  if (find_connection_policy_override(graph_, endpoint->id) != nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road extension endpoint has an explicit connection policy");
  }
  Path extension = std::move(request.extension);
  const Result<bool> extension_valid = ValidatePath(extension);
  if (!extension_valid.ok) {
    return Result<RoadSegmentId>::Fail(extension_valid.failure_category,
                                       extension_valid.error);
  }
  align_first_span_start(extension, endpoint->position);

  if (extension.spans.size() != 1) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road corridor extension requires one local span");
  }
  Result<SegmentShape> shape = SegmentShapeFromPath(extension);
  if (!shape.ok) {
    return Result<RoadSegmentId>::Fail(shape.failure_category, shape.error);
  }

  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId end_node = next_id++;
  const RoadSegmentId segment_id = next_id++;
  plan.add_nodes.push_back(RoadNode{end_node, path_end(extension)});
  plan.add_segments.push_back(
      RoadSegment{segment_id, endpoint->id, end_node, shape.value,
                  request.section_template, std::nullopt});
  RoadCorridor replacement = *source_corridor;
  replacement.section_template_id = request.section_template;
  replacement.segments.push_back(DirectedSegmentRef{segment_id, false});
  plan.replace_corridors.push_back(std::move(replacement));
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) {
    return Result<RoadSegmentId>::Fail(executed.failure_category, executed.error);
  }
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::AddSegmentConnectedTo(AddSegmentConnectedToRequest request) {
  Path alignment = std::move(request.alignment);
  const CrossSectionTemplateId section_template = request.section_template;
  const RoadNodeId start_node = request.start_node;
  const RoadNode* node = find_node(graph_, start_node);
  if (node == nullptr) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kInvalidInput, "road segment start node does not exist");
  }
  if (find_template(graph_, section_template) == nullptr) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kInvalidInput,
                                       "connected road references a missing section template");
  }
  const Result<double> length_result = PathLength(alignment);
  if (!length_result.ok) {
    return Result<RoadSegmentId>::Fail(length_result.failure_category, length_result.error);
  }
  if (length_result.value < kP1MinSegmentLengthM) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kNotImplemented, "connected road segment is shorter than P1 minimum");
  }
  if (alignment.spans.size() != 1) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road branch creation requires one local span");
  }
  align_first_span_start(alignment, node->position);
  const Result<SegmentShape> shape = SegmentShapeFromPath(alignment);
  if (!shape.ok) return Result<RoadSegmentId>::Fail(shape.failure_category, shape.error);
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId end_node = next_id++;
  const RoadSegmentId segment_id = next_id++;
  const RoadCorridorId corridor_id = next_id++;
  plan.add_nodes.push_back(RoadNode{end_node, path_end(alignment)});
  plan.add_segments.push_back(
      RoadSegment{segment_id, start_node, end_node, shape.value, section_template, std::nullopt});
  plan.add_corridors.push_back(
      RoadCorridor{corridor_id, section_template,
                   {DirectedSegmentRef{segment_id, false}}});
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.failure_category, executed.error);
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::AddConnectedLaneSegment(
    AddConnectedLaneSegmentRequest request) {
  const RoadNode* node = find_node(graph_, request.start_node);
  const CrossSectionTemplate* target_template =
      find_template(graph_, request.section_template);
  if (node == nullptr || target_template == nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "connected lane road node or section template does not exist");
  }
  if (request.lane_connections.empty() &&
      request.source_lane_connections.empty()) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "connected lane road requires explicit lane topology");
  }
  const Result<bool> path_valid = ValidatePath(request.alignment);
  if (!path_valid.ok) {
    return Result<RoadSegmentId>::Fail(path_valid.failure_category,
                                       path_valid.error,
                                       path_valid.reason_code);
  }
  if (request.alignment.spans.size() != 1) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "connected lane road requires one local span");
  }
  align_first_span_start(request.alignment, node->position);
  const Result<double> length = PathLength(request.alignment);
  if (!length.ok || length.value < kP1MinSegmentLengthM) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "connected lane road is shorter than the supported minimum");
  }
  for (const LaneTargetConnection& connection : request.lane_connections) {
    const internal::LaneEndpointLookup source =
        internal::find_lane_endpoint(graph_, connection.source);
    const auto target_lane = std::find_if(
        target_template->lane_bands.begin(), target_template->lane_bands.end(),
        [&connection](const LaneBand& lane) {
          return lane.id == connection.target_lane_id;
        });
    if (source.lane == nullptr || source.node_id != request.start_node ||
        target_lane == target_template->lane_bands.end()) {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kInvalidInput,
          "connected lane road lane endpoint is invalid");
    }
  }
  for (const BoundaryTargetContinuation& continuation :
       request.boundary_continuations) {
    const internal::BoundaryEndpointLookup source =
        internal::find_boundary_endpoint(graph_, continuation.source);
    const auto target_boundary = std::find_if(
        target_template->boundaries.begin(), target_template->boundaries.end(),
        [&continuation](const BoundaryProfile& boundary) {
          return boundary.boundary_id == continuation.target_boundary_id;
        });
    if (source.boundary == nullptr || source.node_id != request.start_node ||
        target_boundary == target_template->boundaries.end()) {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kInvalidInput,
          "connected lane road boundary endpoint is invalid");
    }
  }
  for (const LaneSourceConnection& connection :
       request.source_lane_connections) {
    const auto source_lane = std::find_if(
        target_template->lane_bands.begin(), target_template->lane_bands.end(),
        [&connection](const LaneBand& lane) {
          return lane.id == connection.source_lane_id;
        });
    const internal::LaneEndpointLookup target =
        internal::find_lane_endpoint(graph_, connection.target);
    if (source_lane == target_template->lane_bands.end() ||
        target.lane == nullptr || target.node_id != request.start_node) {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kInvalidInput,
          "connected lane road source lane endpoint is invalid");
    }
  }
  for (const BoundarySourceContinuation& continuation :
       request.source_boundary_continuations) {
    const auto source_boundary = std::find_if(
        target_template->boundaries.begin(), target_template->boundaries.end(),
        [&continuation](const BoundaryProfile& boundary) {
          return boundary.boundary_id == continuation.source_boundary_id;
        });
    const internal::BoundaryEndpointLookup target =
        internal::find_boundary_endpoint(graph_, continuation.target);
    if (source_boundary == target_template->boundaries.end() ||
        target.boundary == nullptr || target.node_id != request.start_node) {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kInvalidInput,
          "connected lane road source boundary endpoint is invalid");
    }
  }
  const Result<SegmentShape> shape = SegmentShapeFromPath(request.alignment);
  if (!shape.ok) {
    return Result<RoadSegmentId>::Fail(shape.failure_category, shape.error);
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId end_node = next_id++;
  const RoadSegmentId segment_id = next_id++;
  const RoadCorridorId corridor_id = next_id++;
  plan.add_nodes.push_back(RoadNode{end_node, path_end(request.alignment)});
  plan.add_segments.push_back(RoadSegment{
      segment_id, request.start_node, end_node, shape.value,
      request.section_template, std::nullopt});
  plan.add_corridors.push_back(RoadCorridor{
      corridor_id, request.section_template,
      {DirectedSegmentRef{segment_id, false}}});
  for (const LaneTargetConnection& connection : request.lane_connections) {
    plan.add_lane_connections.push_back(LaneConnection{
        next_id++, connection.source,
        LaneEndpointKey{segment_id, connection.target_lane_id,
                        EndpointRole::kStart},
        connection.kind});
  }
  for (const BoundaryTargetContinuation& continuation :
       request.boundary_continuations) {
    plan.add_boundary_continuations.push_back(BoundaryContinuation{
        next_id++, continuation.source,
        BoundaryEndpointKey{segment_id, continuation.target_boundary_id,
                            EndpointRole::kStart},
        continuation.kind});
  }
  for (const LaneSourceConnection& connection :
       request.source_lane_connections) {
    plan.add_lane_connections.push_back(LaneConnection{
        next_id++,
        LaneEndpointKey{segment_id, connection.source_lane_id,
                        EndpointRole::kStart},
        connection.target, connection.kind});
  }
  for (const BoundarySourceContinuation& continuation :
       request.source_boundary_continuations) {
    plan.add_boundary_continuations.push_back(BoundaryContinuation{
        next_id++,
        BoundaryEndpointKey{segment_id, continuation.source_boundary_id,
                            EndpointRole::kStart},
        continuation.target, continuation.kind});
  }
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) {
    return Result<RoadSegmentId>::Fail(executed.failure_category, executed.error);
  }
  return Result<RoadSegmentId>::Ok(segment_id);
}

Result<RoadSegmentId> RoadState::AddSegmentConnectedToSegment(AddSegmentConnectedToSegmentRequest request) {
  Path alignment = std::move(request.alignment);
  const CrossSectionTemplateId section_template = request.section_template;
  const RoadSegmentId start_segment = request.start_segment;
  const double segment_distance_m = request.segment_distance_m;
  const Result<bool> alignment_valid = ValidatePath(alignment);
  if (!alignment_valid.ok) {
    return Result<RoadSegmentId>::Fail(alignment_valid.failure_category, alignment_valid.error);
  }
  if (alignment.spans.size() != 1) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "road branch creation requires one local span");
  }
  const RoadSegment* source = find_segment(graph_, start_segment);
  if (source == nullptr) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kInvalidInput, "road segment snap target does not exist");
  }
  if (source->transition.has_value()) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kNotImplemented,
                                       "splitting a transitioning road segment is unsupported");
  }
  const Path* source_path = FindCanonicalAlignment(derived_, source->id);
  if (source_path == nullptr) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kInternalError, "road segment canonical alignment is missing");
  }
  const Result<PathSplit> path_split =
      split_path_at_distance(*source_path, segment_distance_m);
  if (!path_split.ok) {
    return Result<RoadSegmentId>::Fail(path_split.failure_category, path_split.error);
  }
  if (distance(path_start(alignment), path_split.value.point) > kSnapDistancePointToleranceM) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kInvalidInput,
                                       "road segment input start does not match its explicit snap distance");
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    if (marking.owner_segment_id != source->id) continue;
    const auto [minimum, maximum] = manual_line_distance_bounds(marking);
    if (minimum < segment_distance_m - kEpsilon &&
        maximum > segment_distance_m + kEpsilon) {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road branch split crosses a manual line marking");
    }
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    if (marking.owner_segment_id != source->id) continue;
    const auto [minimum, maximum] = manual_area_distance_bounds(marking);
    if (minimum < segment_distance_m - kEpsilon &&
        maximum > segment_distance_m + kEpsilon) {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road branch split crosses a manual area marking");
    }
  }
  align_first_span_start(alignment, path_split.value.point);
  const Result<double> branch_length = PathLength(alignment);
  if (!branch_length.ok) return Result<RoadSegmentId>::Fail(branch_length.failure_category, branch_length.error);
  if (branch_length.value < kP1MinSegmentLengthM) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kNotImplemented, "connected road segment is shorter than P1 minimum");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId split_node = next_id++;
  const RoadSegmentId second_id = next_id++;
  const RoadNodeId branch_end_node = next_id++;
  const RoadSegmentId branch_id = next_id++;
  const RoadCorridorId branch_corridor_id = next_id++;
  const Result<SegmentShape> first_shape = SegmentShapeFromPath(path_split.value.before);
  const Result<SegmentShape> second_shape = SegmentShapeFromPath(path_split.value.after);
  const Result<SegmentShape> branch_shape = SegmentShapeFromPath(alignment);
  if (!first_shape.ok || !second_shape.ok || !branch_shape.ok) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kInvalidInput, "road segment split shape is invalid");
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
  const RoadCorridor* source_corridor =
      FindCorridorForSegment(graph_, source->id);
  if (source_corridor == nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInternalError, "road split source corridor is missing");
  }
  RoadCorridor updated_corridor = *source_corridor;
  const auto source_ref =
      std::find_if(updated_corridor.segments.begin(),
                   updated_corridor.segments.end(),
                   [source](const DirectedSegmentRef& ref) {
                     return ref.segment_id == source->id;
                   });
  if (source_ref == updated_corridor.segments.end()) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInternalError, "road split corridor reference is missing");
  }
  const bool reversed = source_ref->reversed;
  const std::size_t source_index =
      static_cast<std::size_t>(
          std::distance(updated_corridor.segments.begin(), source_ref));
  updated_corridor.segments.erase(updated_corridor.segments.begin() +
                                  static_cast<std::ptrdiff_t>(source_index));
  const std::array<DirectedSegmentRef, 2> replacements =
      reversed
          ? std::array<DirectedSegmentRef, 2>{
                DirectedSegmentRef{second_id, true},
                DirectedSegmentRef{source->id, true}}
          : std::array<DirectedSegmentRef, 2>{
                DirectedSegmentRef{source->id, false},
                DirectedSegmentRef{second_id, false}};
  updated_corridor.segments.insert(
      updated_corridor.segments.begin() +
          static_cast<std::ptrdiff_t>(source_index),
      replacements.begin(), replacements.end());
  plan.replace_corridors.push_back(std::move(updated_corridor));
  plan.add_corridors.push_back(
      RoadCorridor{branch_corridor_id, section_template,
                   {DirectedSegmentRef{branch_id, false}}});
  const ApproachKey old_end_key{source->node_b, source->id, EndpointRole::kEnd};
  if (const ApproachGeometryOverride* old_end_override =
          find_approach_geometry_override(graph_, old_end_key)) {
    ApproachGeometryOverride mapped = *old_end_override;
    mapped.key = ApproachKey{source->node_b, second_id, EndpointRole::kEnd};
    plan.remove_approach_geometry_overrides.push_back(old_end_key);
    plan.add_approach_geometry_overrides.push_back(mapped);
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    if (marking.owner_segment_id != source->id ||
        manual_line_distance_bounds(marking).first <
            segment_distance_m - kEpsilon) {
      continue;
    }
    ManualLineMarking mapped = marking;
    mapped.owner_segment_id = second_id;
    shift_manual_line_distance(mapped, -segment_distance_m);
    plan.remove_manual_lines.push_back(marking.id);
    plan.add_manual_lines.push_back(std::move(mapped));
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    if (marking.owner_segment_id != source->id ||
        manual_area_distance_bounds(marking).first <
            segment_distance_m - kEpsilon) {
      continue;
    }
    ManualAreaMarking mapped = marking;
    mapped.owner_segment_id = second_id;
    mapped.frame_origin.x -= segment_distance_m;
    plan.remove_manual_areas.push_back(marking.id);
    plan.add_manual_areas.push_back(std::move(mapped));
  }
  for (const AutoMarkingOverride& override : graph_.auto_marking_overrides) {
    if (override.key.owner.kind != MarkingOwner::Kind::kRoadSegment ||
        override.key.owner.segment_id != source->id ||
        !override.key.track.has_value()) {
      continue;
    }
    AutoMarkingOverride mapped = override;
    mapped.key.owner.segment_id = second_id;
    mapped.key.track->segment_id = second_id;
    plan.add_auto_marking_overrides.push_back(std::move(mapped));
  }
  for (const JunctionMarkingOverride& override :
       graph_.junction_marking_overrides) {
    const bool source_end =
        override.source.approach == old_end_key;
    const bool target_end =
        override.target.has_value() &&
        override.target->approach == old_end_key;
    if (!source_end && !target_end) continue;
    JunctionMarkingOverride mapped = override;
    if (source_end) {
      mapped.source.approach =
          ApproachKey{source->node_b, second_id, EndpointRole::kEnd};
    }
    if (target_end) {
      mapped.target->approach =
          ApproachKey{source->node_b, second_id, EndpointRole::kEnd};
    }
    plan.remove_junction_marking_overrides.push_back(override.id);
    plan.add_junction_marking_overrides.push_back(std::move(mapped));
  }
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.failure_category, executed.error);
  return Result<RoadSegmentId>::Ok(branch_id);
}

Result<RoadSegmentId> RoadState::SplitSegmentAtDistance(
    SplitSegmentAtDistanceRequest request) {
  const RoadSegment* source = find_segment(graph_, request.segment_id);
  if (source == nullptr || !finite(request.segment_distance_m)) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInvalidInput, "road split request is invalid");
  }
  if (source->transition.has_value()) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "splitting a transitioning road segment is unsupported");
  }
  const Path* source_path = FindCanonicalAlignment(derived_, source->id);
  if (source_path == nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInternalError,
        "road split canonical alignment is missing");
  }
  const Result<PathSplit> split =
      split_path_at_distance(*source_path, request.segment_distance_m);
  if (!split.ok) {
    return Result<RoadSegmentId>::Fail(split.failure_category, split.error);
  }
  const Result<SegmentShape> first_shape =
      SegmentShapeFromPath(split.value.before);
  const Result<SegmentShape> second_shape =
      SegmentShapeFromPath(split.value.after);
  if (!first_shape.ok || !second_shape.ok) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInternalError, "road split shape derivation failed");
  }

  for (const ManualLineMarking& marking : graph_.manual_lines) {
    if (marking.owner_segment_id != source->id) continue;
    const auto [minimum, maximum] = manual_line_distance_bounds(marking);
    if (minimum < request.segment_distance_m - kEpsilon &&
        maximum > request.segment_distance_m + kEpsilon) {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road split crosses a manual line marking");
    }
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    if (marking.owner_segment_id != source->id) continue;
    const auto [minimum, maximum] = manual_area_distance_bounds(marking);
    if (minimum < request.segment_distance_m - kEpsilon &&
        maximum > request.segment_distance_m + kEpsilon) {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road split crosses a manual area marking");
    }
  }

  const RoadCorridor* corridor =
      FindCorridorForSegment(graph_, source->id);
  if (corridor == nullptr) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInternalError, "road split source corridor is missing");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId split_node = next_id++;
  const RoadSegmentId second_id = next_id++;
  RoadSegment first = *source;
  first.node_b = split_node;
  first.shape = first_shape.value;
  plan.replace_segments.push_back(std::move(first));
  plan.add_nodes.push_back(RoadNode{split_node, split.value.point});
  plan.add_segments.push_back(RoadSegment{
      second_id, split_node, source->node_b, second_shape.value,
      source->section_template, source->transition});

  RoadCorridor corridor_replacement = *corridor;
  const auto ref_it =
      std::find_if(corridor_replacement.segments.begin(),
                   corridor_replacement.segments.end(),
                   [source](const DirectedSegmentRef& ref) {
                     return ref.segment_id == source->id;
                   });
  if (ref_it == corridor_replacement.segments.end()) {
    return Result<RoadSegmentId>::Fail(
        CommitFailureCategory::kInternalError, "road split corridor reference is missing");
  }
  const std::size_t index = static_cast<std::size_t>(
      std::distance(corridor_replacement.segments.begin(), ref_it));
  const bool reversed = ref_it->reversed;
  corridor_replacement.segments.erase(
      corridor_replacement.segments.begin() +
      static_cast<std::ptrdiff_t>(index));
  const std::array<DirectedSegmentRef, 2> refs =
      reversed
          ? std::array<DirectedSegmentRef, 2>{
                DirectedSegmentRef{second_id, true},
                DirectedSegmentRef{source->id, true}}
          : std::array<DirectedSegmentRef, 2>{
                DirectedSegmentRef{source->id, false},
                DirectedSegmentRef{second_id, false}};
  corridor_replacement.segments.insert(
      corridor_replacement.segments.begin() +
          static_cast<std::ptrdiff_t>(index),
      refs.begin(), refs.end());
  plan.replace_corridors.push_back(std::move(corridor_replacement));

  const ApproachKey old_end_key{source->node_b, source->id,
                                EndpointRole::kEnd};
  if (const ApproachGeometryOverride* old_end =
          find_approach_geometry_override(graph_, old_end_key)) {
    ApproachGeometryOverride mapped = *old_end;
    mapped.key =
        ApproachKey{source->node_b, second_id, EndpointRole::kEnd};
    plan.remove_approach_geometry_overrides.push_back(old_end_key);
    plan.add_approach_geometry_overrides.push_back(std::move(mapped));
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    if (marking.owner_segment_id != source->id) continue;
    const double minimum = manual_line_distance_bounds(marking).first;
    if (minimum + kEpsilon < request.segment_distance_m) continue;
    ManualLineMarking mapped = marking;
    mapped.owner_segment_id = second_id;
    shift_manual_line_distance(mapped, -request.segment_distance_m);
    plan.remove_manual_lines.push_back(marking.id);
    plan.add_manual_lines.push_back(std::move(mapped));
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    if (marking.owner_segment_id != source->id) continue;
    const double minimum = manual_area_distance_bounds(marking).first;
    if (minimum + kEpsilon < request.segment_distance_m) continue;
    ManualAreaMarking mapped = marking;
    mapped.owner_segment_id = second_id;
    mapped.frame_origin.x -= request.segment_distance_m;
    plan.remove_manual_areas.push_back(marking.id);
    plan.add_manual_areas.push_back(std::move(mapped));
  }
  for (const AutoMarkingOverride& override : graph_.auto_marking_overrides) {
    if (override.key.owner.kind != MarkingOwner::Kind::kRoadSegment ||
        override.key.owner.segment_id != source->id ||
        !override.key.track.has_value()) {
      continue;
    }
    AutoMarkingOverride mapped = override;
    mapped.key.owner.segment_id = second_id;
    mapped.key.track->segment_id = second_id;
    plan.add_auto_marking_overrides.push_back(std::move(mapped));
  }
  for (const JunctionMarkingOverride& override :
       graph_.junction_marking_overrides) {
    const bool source_end =
        override.source.approach == old_end_key;
    const bool target_end =
        override.target.has_value() &&
        override.target->approach == old_end_key;
    if (!source_end && !target_end) continue;
    JunctionMarkingOverride mapped = override;
    if (source_end) {
      mapped.source.approach =
          ApproachKey{source->node_b, second_id, EndpointRole::kEnd};
    }
    if (target_end) {
      mapped.target->approach =
          ApproachKey{source->node_b, second_id, EndpointRole::kEnd};
    }
    plan.remove_junction_marking_overrides.push_back(override.id);
    plan.add_junction_marking_overrides.push_back(std::move(mapped));
  }
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) {
    return Result<RoadSegmentId>::Fail(executed.failure_category, executed.error);
  }
  return Result<RoadSegmentId>::Ok(second_id);
}

Result<bool> RoadState::DeleteSegmentRange(DeleteSegmentRangeRequest request) {
  const RoadSegment* source = find_segment(graph_, request.segment_id);
  if (source == nullptr || !finite(request.start_segment_distance_m) ||
      !finite(request.end_segment_distance_m) ||
      request.start_segment_distance_m >= request.end_segment_distance_m) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "road delete range is invalid");
  }
  if (source->transition.has_value()) {
    return Result<bool>::Fail(
        CommitFailureCategory::kNotImplemented,
        "deleting a range from a transitioning segment is unsupported");
  }
  const Path* source_path = FindCanonicalAlignment(derived_, source->id);
  if (source_path == nullptr) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInternalError,
        "road delete range canonical alignment is missing");
  }
  const Result<double> source_length = PathLength(*source_path);
  if (!source_length.ok || request.start_segment_distance_m <= kEpsilon ||
      request.end_segment_distance_m >= source_length.value - kEpsilon) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInvalidInput,
        "road delete range must be inside one segment");
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    if (marking.owner_segment_id != source->id) continue;
    const auto [minimum, maximum] = manual_line_distance_bounds(marking);
    if (maximum > request.start_segment_distance_m + kEpsilon &&
        minimum < request.end_segment_distance_m - kEpsilon) {
      return Result<bool>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road delete range intersects a manual line marking");
    }
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    if (marking.owner_segment_id != source->id) continue;
    const auto [minimum, maximum] = manual_area_distance_bounds(marking);
    if (maximum > request.start_segment_distance_m + kEpsilon &&
        minimum < request.end_segment_distance_m - kEpsilon) {
      return Result<bool>::Fail(
          CommitFailureCategory::kNotImplemented,
          "road delete range intersects a manual area marking");
    }
  }
  const Result<PathSplit> at_start =
      split_path_at_distance(*source_path, request.start_segment_distance_m);
  if (!at_start.ok) {
    return Result<bool>::Fail(at_start.failure_category, at_start.error);
  }
  const Result<PathSplit> at_end = split_path_at_distance(
      at_start.value.after,
      request.end_segment_distance_m - request.start_segment_distance_m);
  if (!at_end.ok) {
    return Result<bool>::Fail(at_end.failure_category, at_end.error);
  }
  const Result<SegmentShape> before_shape =
      SegmentShapeFromPath(at_start.value.before);
  const Result<SegmentShape> after_shape =
      SegmentShapeFromPath(at_end.value.after);
  if (!before_shape.ok || !after_shape.ok) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInternalError, "road delete range shape derivation failed");
  }
  const RoadCorridor* corridor =
      FindCorridorForSegment(graph_, source->id);
  if (corridor == nullptr) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInternalError, "road delete range corridor is missing");
  }
  const auto ref_it =
      std::find_if(corridor->segments.begin(), corridor->segments.end(),
                   [source](const DirectedSegmentRef& ref) {
                     return ref.segment_id == source->id;
                   });
  if (ref_it == corridor->segments.end()) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInternalError,
        "road delete range corridor reference is missing");
  }

  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId before_end_node = next_id++;
  const RoadNodeId after_start_node = next_id++;
  const RoadSegmentId after_segment_id = next_id++;
  const RoadCorridorId end_corridor_id = next_id++;
  RoadSegment before = *source;
  before.node_b = before_end_node;
  before.shape = before_shape.value;
  plan.replace_segments.push_back(std::move(before));
  plan.add_nodes.push_back(
      RoadNode{before_end_node, at_start.value.point});
  plan.add_nodes.push_back(
      RoadNode{after_start_node, at_end.value.point});
  plan.add_segments.push_back(RoadSegment{
      after_segment_id, after_start_node, source->node_b, after_shape.value,
      source->section_template, std::nullopt});

  const std::size_t ref_index = static_cast<std::size_t>(
      std::distance(corridor->segments.begin(), ref_it));
  std::vector<DirectedSegmentRef> prefix(
      corridor->segments.begin(), corridor->segments.begin() +
                                      static_cast<std::ptrdiff_t>(ref_index));
  std::vector<DirectedSegmentRef> suffix(
      corridor->segments.begin() +
          static_cast<std::ptrdiff_t>(ref_index + 1),
      corridor->segments.end());
  RoadCorridor start_side = *corridor;
  RoadCorridor end_side{end_corridor_id, corridor->section_template_id, {}};
  if (ref_it->reversed) {
    prefix.push_back(DirectedSegmentRef{after_segment_id, true});
    end_side.segments.push_back(
        DirectedSegmentRef{source->id, true});
  } else {
    prefix.push_back(DirectedSegmentRef{source->id, false});
    end_side.segments.push_back(
        DirectedSegmentRef{after_segment_id, false});
  }
  end_side.segments.insert(end_side.segments.end(), suffix.begin(),
                           suffix.end());
  start_side.segments = std::move(prefix);
  plan.replace_corridors.push_back(std::move(start_side));
  plan.add_corridors.push_back(std::move(end_side));

  const ApproachKey old_end{source->node_b, source->id, EndpointRole::kEnd};
  if (const ApproachGeometryOverride* override =
          find_approach_geometry_override(graph_, old_end)) {
    ApproachGeometryOverride mapped = *override;
    mapped.key =
        ApproachKey{source->node_b, after_segment_id, EndpointRole::kEnd};
    plan.remove_approach_geometry_overrides.push_back(old_end);
    plan.add_approach_geometry_overrides.push_back(std::move(mapped));
  }
  for (const JunctionMarkingOverride& override :
       graph_.junction_marking_overrides) {
    const bool source_end = override.source.approach == old_end;
    const bool target_end =
        override.target.has_value() &&
        override.target->approach == old_end;
    if (!source_end && !target_end) continue;
    JunctionMarkingOverride mapped = override;
    if (source_end) {
      mapped.source.approach =
          ApproachKey{source->node_b, after_segment_id,
                      EndpointRole::kEnd};
    }
    if (target_end) {
      mapped.target->approach =
          ApproachKey{source->node_b, after_segment_id,
                      EndpointRole::kEnd};
    }
    plan.remove_junction_marking_overrides.push_back(override.id);
    plan.add_junction_marking_overrides.push_back(std::move(mapped));
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    if (marking.owner_segment_id != source->id ||
        manual_line_distance_bounds(marking).first <
            request.end_segment_distance_m - kEpsilon) {
      continue;
    }
    ManualLineMarking mapped = marking;
    mapped.owner_segment_id = after_segment_id;
    shift_manual_line_distance(mapped, -request.end_segment_distance_m);
    plan.remove_manual_lines.push_back(marking.id);
    plan.add_manual_lines.push_back(std::move(mapped));
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    if (marking.owner_segment_id != source->id ||
        manual_area_distance_bounds(marking).first <
            request.end_segment_distance_m - kEpsilon) {
      continue;
    }
    ManualAreaMarking mapped = marking;
    mapped.owner_segment_id = after_segment_id;
    mapped.frame_origin.x -= request.end_segment_distance_m;
    plan.remove_manual_areas.push_back(marking.id);
    plan.add_manual_areas.push_back(std::move(mapped));
  }
  for (const AutoMarkingOverride& override : graph_.auto_marking_overrides) {
    if (override.key.owner.kind != MarkingOwner::Kind::kRoadSegment ||
        override.key.owner.segment_id != source->id ||
        !override.key.track.has_value()) {
      continue;
    }
    AutoMarkingOverride mapped = override;
    mapped.key.owner.segment_id = after_segment_id;
    mapped.key.track->segment_id = after_segment_id;
    plan.add_auto_marking_overrides.push_back(std::move(mapped));
  }
  plan.next_id_after = next_id;
  return Execute(plan);
}

Result<bool> RoadState::EditSegmentShape(EditSegmentShapeRequest request) {
  const RoadSegmentId segment_id = request.segment_id;
  SegmentShape shape = std::move(request.shape);
  const RoadSegment* segment = find_segment(graph_, segment_id);
  if (segment == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road segment does not exist");
  }
  const RoadNode* node_a = find_node(graph_, segment->node_a);
  const RoadNode* node_b = find_node(graph_, segment->node_b);
  const Result<Path> alignment = DeriveCanonicalAlignment(node_a->position, node_b->position, shape);
  if (!alignment.ok) return Result<bool>::Fail(alignment.failure_category, alignment.error);
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
  if (!finite(position)) return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road node position is non-finite");
  const RoadNode* node = find_node(graph_, node_id);
  if (node == nullptr) return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road node does not exist");
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  RoadNode replacement = *node;
  replacement.position = position;
  plan.replace_nodes.push_back(replacement);
  for (const RoadSegment& segment : graph_.segments) {
    if (segment.shape.intent != SegmentShapeIntent::kStraight ||
        (segment.node_a != node_id && segment.node_b != node_id)) {
      continue;
    }
    const RoadNode* node_a = find_node(graph_, segment.node_a);
    const RoadNode* node_b = find_node(graph_, segment.node_b);
    if (node_a == nullptr || node_b == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                "road segment endpoint node is missing");
    }
    const Vec2d start = segment.node_a == node_id ? position : node_a->position;
    const Vec2d end = segment.node_b == node_id ? position : node_b->position;
    Result<SegmentShape> linear_shape = make_linear_shape(start, end);
    if (!linear_shape.ok) {
      return Result<bool>::Fail(linear_shape.failure_category, linear_shape.error);
    }
    RoadSegment updated = segment;
    updated.shape = std::move(linear_shape.value);
    plan.replace_segments.push_back(std::move(updated));
  }
  return Execute(plan);
}

Result<bool> RoadState::DeleteSegment(DeleteSegmentRequest request) {
  const RoadSegmentId segment_id = request.segment_id;
  const RoadSegment* target = find_segment(graph_, segment_id);
  if (target == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "road segment does not exist");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  plan.remove_segments.push_back(segment_id);
  const RoadCorridor* corridor = FindCorridorForSegment(graph_, segment_id);
  if (corridor == nullptr) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInternalError, "road segment corridor is missing");
  }
  const auto ref_it =
      std::find_if(corridor->segments.begin(), corridor->segments.end(),
                   [segment_id](const DirectedSegmentRef& ref) {
                     return ref.segment_id == segment_id;
                   });
  if (ref_it == corridor->segments.end()) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInternalError, "road corridor segment reference is missing");
  }
  std::vector<DirectedSegmentRef> before(corridor->segments.begin(), ref_it);
  std::vector<DirectedSegmentRef> after(ref_it + 1,
                                        corridor->segments.end());
  if (before.empty() && after.empty()) {
    plan.remove_corridors.push_back(corridor->id);
  } else if (before.empty() || after.empty()) {
    RoadCorridor replacement = *corridor;
    replacement.segments =
        before.empty() ? std::move(after) : std::move(before);
    plan.replace_corridors.push_back(std::move(replacement));
  } else {
    RoadCorridor start_side = *corridor;
    start_side.segments = std::move(before);
    RoadCorridor end_side{next_id++, corridor->section_template_id,
                          std::move(after)};
    plan.replace_corridors.push_back(std::move(start_side));
    plan.add_corridors.push_back(std::move(end_side));
  }
  for (const RoadNodeId node_id :
       std::array<RoadNodeId, 2>{target->node_a, target->node_b}) {
    if (node_degree(graph_, node_id) == 1) {
      plan.remove_nodes.push_back(node_id);
    }
  }
  for (const NodeConnectionPolicyOverride& policy :
       graph_.connection_policy_overrides) {
    const std::size_t degree_after =
        node_degree(graph_, policy.node_id) -
        ((target->node_a == policy.node_id ||
          target->node_b == policy.node_id)
             ? 1
             : 0);
    if (degree_after == 0) {
      plan.remove_connection_policy_overrides.push_back(policy.id);
    }
  }
  for (const ManualLineMarking& marking : graph_.manual_lines) {
    if (marking.owner_segment_id == segment_id) {
      plan.remove_manual_lines.push_back(marking.id);
    }
  }
  for (const ManualAreaMarking& marking : graph_.manual_areas) {
    if (marking.owner_segment_id == segment_id) {
      plan.remove_manual_areas.push_back(marking.id);
    }
  }
  const auto removes_node = [&plan](RoadNodeId node_id) {
    return std::find(plan.remove_nodes.begin(), plan.remove_nodes.end(),
                     node_id) != plan.remove_nodes.end();
  };
  for (const ApproachGeometryOverride& override :
       graph_.approach_geometry_overrides) {
    if (override.key.segment_id == segment_id) {
      plan.remove_approach_geometry_overrides.push_back(override.key);
    }
  }
  for (const AutoMarkingOverride& override :
       graph_.auto_marking_overrides) {
    const bool segment_owner =
        override.key.owner.kind == MarkingOwner::Kind::kRoadSegment &&
        override.key.owner.segment_id == segment_id;
    const bool removed_junction_owner =
        override.key.owner.kind == MarkingOwner::Kind::kJunction &&
        removes_node(override.key.owner.node_id);
    const bool approach_owner =
        override.key.approach.has_value() &&
        override.key.approach->segment_id == segment_id;
    const bool track_owner =
        override.key.track.has_value() &&
        override.key.track->segment_id == segment_id;
    if (segment_owner || removed_junction_owner || approach_owner ||
        track_owner) {
      plan.remove_auto_marking_overrides.push_back(override.key);
    }
  }
  for (const JunctionMarkingOverride& override :
       graph_.junction_marking_overrides) {
    const bool removed_node_owner = removes_node(override.node_id);
    const bool source_owner =
        override.source.approach.segment_id == segment_id;
    const bool target_owner =
        override.target.has_value() &&
        override.target->approach.segment_id == segment_id;
    if (removed_node_owner || source_owner || target_owner) {
      plan.remove_junction_marking_overrides.push_back(override.id);
    }
  }
  const std::optional<SectionTransitionId> transition = target->transition;
  if (transition.has_value() &&
      std::none_of(
          graph_.segments.begin(), graph_.segments.end(),
          [segment_id, transition](const RoadSegment& segment) {
            return segment.id != segment_id &&
                   segment.transition == transition;
          })) {
    plan.remove_transitions.push_back(*transition);
  }
  plan.next_id_after = next_id;
  return Execute(plan);
}

Result<bool> RoadState::SetApproachSetbackOverride(SetApproachSetbackOverrideRequest request) {
  if (!finite(request.setback_m) || request.setback_m < 0.0) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road approach setback override is invalid");
  }
  const RoadSegment* segment = find_segment(graph_, request.key.segment_id);
  if (segment == nullptr || find_node(graph_, request.key.node_id) == nullptr ||
      !endpoint_matches(*segment, request.key)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road approach override key is invalid");
  }
  if (FindResolvedApproach(derived_, request.key) == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kNotImplemented, "road approach has no resolved connection");
  }
  ApproachGeometryOverride override =
      find_approach_geometry_override(graph_, request.key) != nullptr
          ? *find_approach_geometry_override(graph_, request.key)
          : ApproachGeometryOverride{request.key, {}, {}};
  override.setback_m = ManualDoubleOverride{true, request.setback_m};
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  if (find_approach_geometry_override(graph_, request.key) != nullptr) {
    plan.remove_approach_geometry_overrides.push_back(request.key);
  }
  plan.add_approach_geometry_overrides.push_back(override);
  return Execute(plan);
}

Result<bool> RoadState::SetApproachLateralShiftOverride(SetApproachLateralShiftOverrideRequest request) {
  if (!finite(request.lateral_shift_m)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road approach lateral shift override is invalid");
  }
  const RoadSegment* segment = find_segment(graph_, request.key.segment_id);
  if (segment == nullptr || find_node(graph_, request.key.node_id) == nullptr ||
      !endpoint_matches(*segment, request.key)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road approach override key is invalid");
  }
  if (FindResolvedApproach(derived_, request.key) == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kNotImplemented, "road approach has no resolved connection");
  }
  ApproachGeometryOverride override =
      find_approach_geometry_override(graph_, request.key) != nullptr
          ? *find_approach_geometry_override(graph_, request.key)
          : ApproachGeometryOverride{request.key, {}, {}};
  override.lateral_shift_m = ManualDoubleOverride{true, request.lateral_shift_m};
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  if (find_approach_geometry_override(graph_, request.key) != nullptr) {
    plan.remove_approach_geometry_overrides.push_back(request.key);
  }
  plan.add_approach_geometry_overrides.push_back(override);
  return Execute(plan);
}

Result<bool> RoadState::ResetApproachOverrideField(ResetApproachOverrideFieldRequest request) {
  const RoadSegment* segment = find_segment(graph_, request.key.segment_id);
  if (segment == nullptr || find_node(graph_, request.key.node_id) == nullptr ||
      !endpoint_matches(*segment, request.key)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road approach override key is invalid");
  }
  const ApproachGeometryOverride* existing = find_approach_geometry_override(graph_, request.key);
  if (existing == nullptr) return Result<bool>::Ok(true);
  ApproachGeometryOverride replacement = *existing;
  if (request.field == ApproachOverrideField::kSetback) {
    replacement.setback_m = {};
  } else {
    replacement.lateral_shift_m = {};
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.remove_approach_geometry_overrides.push_back(request.key);
  if (replacement.setback_m.has_value || replacement.lateral_shift_m.has_value) {
    plan.add_approach_geometry_overrides.push_back(replacement);
  }
  return Execute(plan);
}

Result<bool> RoadState::ResetAllApproachOverrides(ResetAllApproachOverridesRequest request) {
  const RoadSegment* segment = find_segment(graph_, request.key.segment_id);
  if (segment == nullptr || find_node(graph_, request.key.node_id) == nullptr ||
      !endpoint_matches(*segment, request.key)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road approach override key is invalid");
  }
  if (find_approach_geometry_override(graph_, request.key) == nullptr) return Result<bool>::Ok(true);
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.remove_approach_geometry_overrides.push_back(request.key);
  return Execute(plan);
}

Result<CrossSectionTemplateId> RoadState::AddSectionTemplate(AddSectionTemplateRequest request) {
  CrossSectionTemplate section_template = std::move(request.section_template);
  const Result<bool> valid = validate_section_template(section_template);
  if (!valid.ok) {
    return Result<CrossSectionTemplateId>::Fail(valid.failure_category, valid.error);
  }
  if (section_template.id != 0 && find_template(graph_, section_template.id) != nullptr) {
    return Result<CrossSectionTemplateId>::Fail(CommitFailureCategory::kInvalidInput, "section template id already exists");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (section_template.id == 0) section_template.id = next_id++;
  const CrossSectionTemplateId id = section_template.id;
  plan.next_id_after = next_id;
  plan.add_section_templates.push_back(std::move(section_template));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<CrossSectionTemplateId>::Fail(executed.failure_category, executed.error);
  return Result<CrossSectionTemplateId>::Ok(id);
}

Result<bool> RoadState::EditSectionTemplate(EditSectionTemplateRequest request) {
  CrossSectionTemplate section_template = std::move(request.section_template);
  const Result<bool> valid = validate_section_template(section_template);
  if (!valid.ok) return valid;
  auto it = std::find_if(graph_.section_templates.begin(), graph_.section_templates.end(),
                         [&section_template](const CrossSectionTemplate& item) { return item.id == section_template.id; });
  if (it == graph_.section_templates.end()) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template does not exist");
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.replace_section_templates.push_back(std::move(section_template));
  return Execute(plan);
}

Result<bool> RoadState::SetBoundaryMarkingPolicy(SetBoundaryMarkingPolicyRequest request) {
  const Result<bool> valid_policy = validate_marking_policy(request.policy);
  if (!valid_policy.ok) return valid_policy;
  const CrossSectionTemplate* existing = find_template(graph_, request.section_template_id);
  if (existing == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template does not exist");
  }
  CrossSectionTemplate replacement = *existing;
  BoundaryProfile* boundary = find_boundary(replacement, request.boundary_id);
  if (boundary == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section boundary does not exist");
  }
  boundary->marking = request.policy;
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.replace_section_templates.push_back(std::move(replacement));
  return Execute(plan);
}

Result<bool> RoadState::ResetBoundaryMarkingPolicy(ResetBoundaryMarkingPolicyRequest request) {
  return SetBoundaryMarkingPolicy(SetBoundaryMarkingPolicyRequest{
      request.section_template_id, request.boundary_id, AutoMarkingPolicy{}});
}

Result<bool> RoadState::SetLaneSideMarkingPolicy(SetLaneSideMarkingPolicyRequest request) {
  const Result<bool> valid_policy = validate_marking_policy(request.policy);
  if (!valid_policy.ok) return valid_policy;
  const CrossSectionTemplate* existing = find_template(graph_, request.section_template_id);
  if (existing == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section template does not exist");
  }
  CrossSectionTemplate replacement = *existing;
  const auto strip = std::find_if(replacement.strips.begin(), replacement.strips.end(),
                                  [&request](const SectionStrip& candidate) {
                                    return candidate.id == request.strip_id;
                                  });
  if (strip == replacement.strips.end()) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "section strip does not exist");
  }
  if (strip->function != StripFunction::kCarriageway) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "lane side marking requires a carriageway strip");
  }
  if (request.side == LaneSide::kLeft) {
    strip->side_marking.left = request.policy;
  } else {
    strip->side_marking.right = request.policy;
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.replace_section_templates.push_back(std::move(replacement));
  return Execute(plan);
}

Result<bool> RoadState::ResetLaneSideMarkingPolicy(ResetLaneSideMarkingPolicyRequest request) {
  return SetLaneSideMarkingPolicy(SetLaneSideMarkingPolicyRequest{
      request.section_template_id, request.strip_id, request.side, AutoMarkingPolicy{}});
}

Result<SectionTransitionId> RoadState::AddTransition(SectionTransitionRequest request) {
  SectionTransition transition{0, request.from_template, request.to_template,
                               request.start, request.end, request.anchor,
                               request.anchor_boundary_id,
                               std::move(request.rules)};
  const Result<bool> valid = validate_transition(graph_, transition);
  if (!valid.ok) {
    return Result<SectionTransitionId>::Fail(valid.failure_category, valid.error);
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (transition.id == 0) transition.id = next_id++;
  const SectionTransitionId id = transition.id;
  plan.next_id_after = next_id;
  plan.add_transitions.push_back(std::move(transition));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<SectionTransitionId>::Fail(executed.failure_category, executed.error);
  return Result<SectionTransitionId>::Ok(id);
}

Result<SectionTransitionId> RoadState::AddTransitionToSegment(AddTransitionToSegmentRequest operation) {
  const RoadSegmentId segment_id = operation.segment_id;
  SectionTransitionRequest request = std::move(operation.transition);
  const RoadSegment* segment = find_segment(graph_, segment_id);
  if (segment == nullptr) {
    return Result<SectionTransitionId>::Fail(CommitFailureCategory::kInvalidInput, "road transition segment does not exist");
  }
  SectionTransition transition{0, segment->section_template,
                               request.to_template, request.start, request.end,
                               request.anchor, request.anchor_boundary_id,
                               std::move(request.rules)};
  const Result<bool> valid = validate_transition(graph_, transition);
  if (!valid.ok) return Result<SectionTransitionId>::Fail(valid.failure_category, valid.error);
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
  if (!executed.ok) return Result<SectionTransitionId>::Fail(executed.failure_category, executed.error);
  return Result<SectionTransitionId>::Ok(id);
}

Result<LaneId> RoadState::AddLane(AddLaneRequest request) {
  if (!finite(request.taper_start_corridor_distance_m) ||
      !finite(request.full_width_corridor_distance_m)) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInvalidInput,
                                "lane addition distances must be finite");
  }
  if (request.taper_start_corridor_distance_m < 0.0) {
    return Result<LaneId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "lane addition start is before the corridor start");
  }
  if (request.full_width_corridor_distance_m <=
      request.taper_start_corridor_distance_m) {
    return Result<LaneId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "lane full-width point must be after the taper start");
  }
  if (!finite(request.lane_width_m) || request.lane_width_m <= 0.0) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInvalidInput,
                                "lane width must be finite and positive");
  }
  const RoadCorridor* corridor =
      FindRoadCorridor(graph_, request.corridor_id);
  if (corridor == nullptr || corridor->segments.empty()) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInvalidInput,
                                "lane transition corridor does not exist");
  }
  const Result<ResolvedSegmentDistance> start = ResolveCorridorDistance(
      graph_, derived_, CorridorDistanceRef{request.corridor_id,
                                            request.taper_start_corridor_distance_m});
  const Result<ResolvedSegmentDistance> full = ResolveCorridorDistance(
      graph_, derived_, CorridorDistanceRef{
                            request.corridor_id,
                            request.full_width_corridor_distance_m});
  if (!start.ok) {
    return Result<LaneId>::Fail(start.failure_category, start.error);
  }
  if (!full.ok) {
    return Result<LaneId>::Fail(full.failure_category, full.error);
  }
  const RoadSegment* segment = find_segment(graph_, start.value.segment_id);
  if (segment == nullptr) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                "lane transition segment is missing");
  }
  const auto selected_ref = std::find_if(
      corridor->segments.begin(), corridor->segments.end(),
      [segment](const DirectedSegmentRef& ref) {
        return ref.segment_id == segment->id;
      });
  if (selected_ref == corridor->segments.end()) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                "lane transition segment is not in its corridor");
  }
  const LaneTravelDirection local_direction =
      selected_ref->reversed
          ? (request.direction == LaneTravelDirection::kAlongSegment
                 ? LaneTravelDirection::kAgainstSegment
                 : LaneTravelDirection::kAlongSegment)
          : request.direction;
  const RoadSide local_side =
      selected_ref->reversed
          ? (request.side == RoadSide::kLeft ? RoadSide::kRight
                                             : RoadSide::kLeft)
          : request.side;
  const CrossSectionTemplate* source =
      find_template(graph_, segment->section_template);
  if (source == nullptr) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                "lane transition section is missing");
  }
  if (segment->transition.has_value()) {
    return Result<LaneId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "lane transition cannot overlap an existing section transition");
  }
  struct IndexedLane {
    const LaneBand* lane = nullptr;
    std::size_t strip_index = 0;
    double center_in_strip_m = 0.0;
  };
  std::vector<IndexedLane> candidates{};
  for (const LaneBand& lane : source->lane_bands) {
    if (lane.direction != local_direction) continue;
    const auto strip = std::find_if(
        source->strips.begin(), source->strips.end(),
        [&lane](const SectionStrip& candidate) {
          return candidate.id == lane.surface_strip_id;
        });
    if (strip == source->strips.end()) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInternalError,
          "lane transition source lane strip is missing");
    }
    candidates.push_back(IndexedLane{
        &lane, static_cast<std::size_t>(
                   std::distance(source->strips.begin(), strip)),
        (lane.lateral_start_m + lane.lateral_end_m) * 0.5});
  }
  if (candidates.empty()) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInvalidInput,
                                "selected direction has no lane to extend");
  }
  const auto extremes = std::minmax_element(
      candidates.begin(), candidates.end(),
       [](const IndexedLane& a, const IndexedLane& b) {
         return std::tie(a.strip_index, a.center_in_strip_m) <
                std::tie(b.strip_index, b.center_in_strip_m);
       });
  const IndexedLane selected = local_side == RoadSide::kRight
                                    ? *extremes.second
                                    : *extremes.first;
  const std::size_t lane_strip_index = selected.strip_index;
  std::optional<std::size_t> anchor_index{};
  if (local_side == RoadSide::kRight) {
    for (std::size_t index = lane_strip_index; index > 0; --index) {
      const std::size_t candidate = index - 1;
      if (IsSinglePositionBoundary(source->boundaries[candidate])) {
        anchor_index = candidate;
        break;
      }
    }
  } else {
    for (std::size_t candidate = lane_strip_index;
         candidate < source->boundaries.size(); ++candidate) {
      if (IsSinglePositionBoundary(source->boundaries[candidate])) {
        anchor_index = candidate;
        break;
      }
    }
  }
  if (!anchor_index.has_value()) {
    return Result<LaneId>::Fail(
        CommitFailureCategory::kNotImplemented,
        "selected side has no stable boundary for a lane taper");
  }
  const BoundaryId anchor_boundary_id =
      source->boundaries[*anchor_index].boundary_id;

  CrossSectionTemplate target = *source;
  std::uint64_t next_local_id = 1;
  for (const SectionStrip& strip : target.strips)
    next_local_id = std::max(next_local_id, strip.id + 1);
  for (const LaneBand& lane : target.lane_bands)
    next_local_id = std::max(next_local_id, lane.id + 1);
  for (const BoundaryProfile& boundary : target.boundaries)
    next_local_id = std::max(next_local_id, boundary.boundary_id + 1);
  const SectionStripId added_strip_id = next_local_id++;
  const LaneId added_lane_id = next_local_id++;
  const BoundaryId added_boundary_id = next_local_id++;
  SectionStrip added_strip = source->strips[lane_strip_index];
  added_strip.id = added_strip_id;
  added_strip.width_m = request.lane_width_m;
  added_strip.side_marking = {};
  const AutoMarkingPolicy lane_divider{
      true, MarkingRole::kLaneSeparator,
      builtin_marking_styles::kWhiteDashed};
  const BoundaryProfile added_boundary{added_boundary_id,
                                       BoundaryRole::kLaneDivider, 0.0, 0.0,
                                       lane_divider};
  if (local_side == RoadSide::kRight) {
    target.strips.insert(target.strips.begin() + lane_strip_index + 1,
                         added_strip);
    target.boundaries.insert(target.boundaries.begin() + lane_strip_index,
                             added_boundary);
  } else {
    target.strips.insert(target.strips.begin() + lane_strip_index, added_strip);
    target.boundaries.insert(target.boundaries.begin() + lane_strip_index,
                             added_boundary);
  }
  target.lane_bands.push_back(
      LaneBand{added_lane_id, added_strip_id, 0.0, request.lane_width_m,
               local_direction});

  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  target.id = next_id++;
  const auto full_ref = std::find_if(
      selected_ref, corridor->segments.end(),
      [&full](const DirectedSegmentRef& ref) {
        return ref.segment_id == full.value.segment_id;
      });
  if (full_ref == corridor->segments.end()) {
    return Result<LaneId>::Fail(
        CommitFailureCategory::kInternalError,
        "lane transition full-width segment is not in its corridor");
  }

  // A completed taper is a section boundary, not permanent state on the
  // whole user segment. Isolating it lets later, non-overlapping lane edits
  // start from the resolved full-width section.
  if (selected_ref == full_ref && !segment->transition.has_value()) {
    const Path* source_path = FindCanonicalAlignment(derived_, segment->id);
    const DerivedSegment* source_derived =
        FindDerivedSegment(derived_, segment->id);
    if (source_path == nullptr || source_derived == nullptr) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInternalError,
          "lane addition source geometry is missing");
    }
    const double first_cut = std::min(start.value.segment_distance_m,
                                      full.value.segment_distance_m);
    const double second_cut = std::max(start.value.segment_distance_m,
                                       full.value.segment_distance_m);
    if (first_cut > kEpsilon &&
        second_cut < source_derived->length_m - kEpsilon) {
      for (const ManualLineMarking& marking : graph_.manual_lines) {
        if (marking.owner_segment_id != segment->id) continue;
        const auto [minimum, maximum] = manual_line_distance_bounds(marking);
        if ((minimum < first_cut - kEpsilon &&
             maximum > first_cut + kEpsilon) ||
            (minimum < second_cut - kEpsilon &&
             maximum > second_cut + kEpsilon)) {
          return Result<LaneId>::Fail(
              CommitFailureCategory::kNotImplemented,
              "lane addition boundary crosses a manual line marking");
        }
      }
      for (const ManualAreaMarking& marking : graph_.manual_areas) {
        if (marking.owner_segment_id != segment->id) continue;
        const auto [minimum, maximum] = manual_area_distance_bounds(marking);
        if ((minimum < first_cut - kEpsilon &&
             maximum > first_cut + kEpsilon) ||
            (minimum < second_cut - kEpsilon &&
             maximum > second_cut + kEpsilon)) {
          return Result<LaneId>::Fail(
              CommitFailureCategory::kNotImplemented,
              "lane addition boundary crosses a manual area marking");
        }
      }
      const Result<PathSplit> first_split =
          split_path_at_distance(*source_path, first_cut);
      if (!first_split.ok) {
        return Result<LaneId>::Fail(first_split.failure_category,
                                    first_split.error);
      }
      const Result<PathSplit> second_split = split_path_at_distance(
          first_split.value.after, second_cut - first_cut);
      if (!second_split.ok) {
        return Result<LaneId>::Fail(second_split.failure_category,
                                    second_split.error);
      }
      const Result<SegmentShape> first_shape =
          SegmentShapeFromPath(first_split.value.before);
      const Result<SegmentShape> taper_shape =
          SegmentShapeFromPath(second_split.value.before);
      const Result<SegmentShape> last_shape =
          SegmentShapeFromPath(second_split.value.after);
      if (!first_shape.ok || !taper_shape.ok || !last_shape.ok) {
        return Result<LaneId>::Fail(
            CommitFailureCategory::kInternalError,
            "lane addition split shape derivation failed");
      }

      const RoadNodeId first_node = next_id++;
      const RoadNodeId second_node = next_id++;
      const RoadSegmentId taper_segment_id = next_id++;
      const RoadSegmentId last_segment_id = next_id++;
      const SectionTransitionId transition_id = next_id++;
      const bool reversed = selected_ref->reversed;
      const CrossSectionTemplateId local_first_template =
          reversed ? target.id : source->id;
      const CrossSectionTemplateId local_taper_from =
          reversed ? target.id : source->id;
      const CrossSectionTemplateId local_taper_to =
          reversed ? source->id : target.id;
      const CrossSectionTemplateId local_last_template =
          reversed ? source->id : target.id;
      const TransitionAction action =
          reversed ? TransitionAction::kTaperOut
                   : TransitionAction::kTaperIn;

      RoadSegment first_piece = *segment;
      first_piece.node_b = first_node;
      first_piece.shape = first_shape.value;
      first_piece.section_template = local_first_template;
      first_piece.transition.reset();
      plan.replace_segments.push_back(std::move(first_piece));
      plan.add_nodes.push_back(RoadNode{first_node, first_split.value.point});
      plan.add_nodes.push_back(RoadNode{second_node, second_split.value.point});
      plan.add_segments.push_back(RoadSegment{
          taper_segment_id, first_node, second_node, taper_shape.value,
          local_taper_from, transition_id});
      plan.add_segments.push_back(RoadSegment{
          last_segment_id, second_node, segment->node_b, last_shape.value,
          local_last_template, std::nullopt});
      plan.add_transitions.push_back(SectionTransition{
          transition_id, local_taper_from, local_taper_to,
          DistanceRef{DistanceRefKind::kRatio, 0.0},
          DistanceRef{DistanceRefKind::kRatio, 1.0},
          TransitionAnchor::kBoundary, anchor_boundary_id,
          {SectionTransitionRule{added_strip_id, action}}});

      for (auto ref = full_ref + 1; ref != corridor->segments.end(); ++ref) {
        const RoadSegment* following = find_segment(graph_, ref->segment_id);
        const CrossSectionTemplate* following_section =
            following == nullptr
                ? nullptr
                : find_template(graph_, following->section_template);
        if (following == nullptr || following_section == nullptr) {
          return Result<LaneId>::Fail(
              CommitFailureCategory::kInternalError,
              "lane addition following corridor section is missing");
        }
        if (following->transition.has_value()) {
          return Result<LaneId>::Fail(
              CommitFailureCategory::kNotImplemented,
              "added lane conflicts with a later section transition");
        }
        if (!internal::equivalent_section_definition(*following_section,
                                                     *source)) {
          return Result<LaneId>::Fail(
              CommitFailureCategory::kNotImplemented,
              "added lane reaches a different unresolved section");
        }
        RoadSegment replacement = *following;
        replacement.section_template = target.id;
        plan.replace_segments.push_back(std::move(replacement));
      }

      RoadCorridor corridor_replacement = *corridor;
      const std::size_t ref_index = static_cast<std::size_t>(
          std::distance(corridor->segments.begin(), selected_ref));
      corridor_replacement.segments.erase(
          corridor_replacement.segments.begin() +
          static_cast<std::ptrdiff_t>(ref_index));
      const std::array<DirectedSegmentRef, 3> replacement_refs =
          reversed
              ? std::array<DirectedSegmentRef, 3>{
                    DirectedSegmentRef{last_segment_id, true},
                    DirectedSegmentRef{taper_segment_id, true},
                    DirectedSegmentRef{segment->id, true}}
              : std::array<DirectedSegmentRef, 3>{
                    DirectedSegmentRef{segment->id, false},
                    DirectedSegmentRef{taper_segment_id, false},
                    DirectedSegmentRef{last_segment_id, false}};
      corridor_replacement.segments.insert(
          corridor_replacement.segments.begin() +
              static_cast<std::ptrdiff_t>(ref_index),
          replacement_refs.begin(), replacement_refs.end());
      corridor_replacement.section_template_id = target.id;
      plan.replace_corridors.push_back(std::move(corridor_replacement));

      const ApproachKey old_end{segment->node_b, segment->id,
                                EndpointRole::kEnd};
      const ApproachKey new_end{segment->node_b, last_segment_id,
                                EndpointRole::kEnd};
      if (const ApproachGeometryOverride* override =
              find_approach_geometry_override(graph_, old_end)) {
        ApproachGeometryOverride mapped = *override;
        mapped.key = new_end;
        plan.remove_approach_geometry_overrides.push_back(old_end);
        plan.add_approach_geometry_overrides.push_back(std::move(mapped));
      }
      for (const JunctionMarkingOverride& override :
           graph_.junction_marking_overrides) {
        const bool source_end = override.source.approach == old_end;
        const bool target_end = override.target.has_value() &&
                                override.target->approach == old_end;
        if (!source_end && !target_end) continue;
        JunctionMarkingOverride mapped = override;
        if (source_end) mapped.source.approach = new_end;
        if (target_end) mapped.target->approach = new_end;
        plan.remove_junction_marking_overrides.push_back(override.id);
        plan.add_junction_marking_overrides.push_back(std::move(mapped));
      }
      for (const ManualLineMarking& marking : graph_.manual_lines) {
        if (marking.owner_segment_id != segment->id) continue;
        const double minimum = manual_line_distance_bounds(marking).first;
        if (minimum + kEpsilon < first_cut) continue;
        ManualLineMarking mapped = marking;
        plan.remove_manual_lines.push_back(marking.id);
        if (minimum + kEpsilon < second_cut) {
          mapped.owner_segment_id = taper_segment_id;
          shift_manual_line_distance(mapped, -first_cut);
        } else {
          mapped.owner_segment_id = last_segment_id;
          shift_manual_line_distance(mapped, -second_cut);
        }
        plan.add_manual_lines.push_back(std::move(mapped));
      }
      for (const ManualAreaMarking& marking : graph_.manual_areas) {
        if (marking.owner_segment_id != segment->id) continue;
        const double minimum = manual_area_distance_bounds(marking).first;
        if (minimum + kEpsilon < first_cut) continue;
        ManualAreaMarking mapped = marking;
        plan.remove_manual_areas.push_back(marking.id);
        if (minimum + kEpsilon < second_cut) {
          mapped.owner_segment_id = taper_segment_id;
          mapped.frame_origin.x -= first_cut;
        } else {
          mapped.owner_segment_id = last_segment_id;
          mapped.frame_origin.x -= second_cut;
        }
        plan.add_manual_areas.push_back(std::move(mapped));
      }
      for (const AutoMarkingOverride& override :
           graph_.auto_marking_overrides) {
        if (override.key.owner.kind != MarkingOwner::Kind::kRoadSegment ||
            override.key.owner.segment_id != segment->id ||
            !override.key.track.has_value()) {
          continue;
        }
        for (const RoadSegmentId split_id :
             {taper_segment_id, last_segment_id}) {
          AutoMarkingOverride mapped = override;
          mapped.key.owner.segment_id = split_id;
          mapped.key.track->segment_id = split_id;
          plan.add_auto_marking_overrides.push_back(std::move(mapped));
        }
      }
      for (const LaneConnection& connection : graph_.lane_connections) {
        const bool source_end =
            connection.source.segment_id == segment->id &&
            connection.source.endpoint_role == EndpointRole::kEnd;
        const bool target_end =
            connection.target.segment_id == segment->id &&
            connection.target.endpoint_role == EndpointRole::kEnd;
        if (!source_end && !target_end) continue;
        LaneConnection mapped = connection;
        if (source_end) mapped.source.segment_id = last_segment_id;
        if (target_end) mapped.target.segment_id = last_segment_id;
        plan.remove_lane_connections.push_back(connection.id);
        plan.add_lane_connections.push_back(std::move(mapped));
      }
      for (const BoundaryContinuation& continuation :
           graph_.boundary_continuations) {
        const bool source_end =
            continuation.source.segment_id == segment->id &&
            continuation.source.endpoint_role == EndpointRole::kEnd;
        const bool target_end =
            continuation.target.segment_id == segment->id &&
            continuation.target.endpoint_role == EndpointRole::kEnd;
        if (!source_end && !target_end) continue;
        BoundaryContinuation mapped = continuation;
        if (source_end) mapped.source.segment_id = last_segment_id;
        if (target_end) mapped.target.segment_id = last_segment_id;
        plan.remove_boundary_continuations.push_back(continuation.id);
        plan.add_boundary_continuations.push_back(std::move(mapped));
      }
      plan.add_section_templates.push_back(std::move(target));
      plan.next_id_after = next_id;
      const Result<bool> executed = Execute(plan);
      if (!executed.ok) {
        return Result<LaneId>::Fail(executed.failure_category,
                                    executed.error);
      }
      return Result<LaneId>::Ok(added_lane_id);
    }
  }
  const double taper_length = request.full_width_corridor_distance_m -
                               request.taper_start_corridor_distance_m;
  CrossSectionTemplateId from_template_id = source->id;
  RoadSegmentId full_width_split_segment_id = 0;
  bool full_width_split_reversed = false;
  double corridor_begin = 0.0;
  for (auto ref = corridor->segments.begin(); ref != selected_ref; ++ref) {
    const DerivedSegment* before = FindDerivedSegment(derived_, ref->segment_id);
    if (before == nullptr) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInternalError,
          "lane transition corridor length is missing");
    }
    corridor_begin += before->length_m;
  }
  for (auto ref = selected_ref; ref <= full_ref; ++ref) {
    const RoadSegment* current = find_segment(graph_, ref->segment_id);
    const DerivedSegment* current_derived =
        FindDerivedSegment(derived_, ref->segment_id);
    if (current == nullptr || current_derived == nullptr) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInternalError,
          "lane transition corridor segment is missing");
    }
    if (current->transition.has_value()) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "lane transition cannot overlap an existing section transition");
    }
    const CrossSectionTemplate* current_section =
        find_template(graph_, current->section_template);
    if (current_section == nullptr) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInternalError,
          "lane transition corridor section is missing");
    }
    if (!internal::equivalent_section_definition(*current_section, *source)) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "added lane reaches a different unresolved section");
    }
    const double corridor_end = corridor_begin + current_derived->length_m;
    const double corridor_overlap_start = std::max(
        corridor_begin, request.taper_start_corridor_distance_m);
    const double corridor_overlap_end =
        std::min(corridor_end, request.full_width_corridor_distance_m);
    const double corridor_local_start = corridor_overlap_start - corridor_begin;
    const double corridor_local_end = corridor_overlap_end - corridor_begin;
    const double local_start =
        ref->reversed
            ? current_derived->length_m - corridor_local_end
            : corridor_local_start;
    const double local_end =
        ref->reversed
            ? current_derived->length_m - corridor_local_start
            : corridor_local_end;
    const double end_fraction = std::clamp(
        (std::min(corridor_end, request.full_width_corridor_distance_m) -
         request.taper_start_corridor_distance_m) /
            taper_length,
        0.0, 1.0);
    CrossSectionTemplateId to_template_id = target.id;
    if (end_fraction < 1.0 - kEpsilon) {
      CrossSectionTemplate intermediate = target;
      intermediate.id = next_id++;
      auto intermediate_strip = std::find_if(
          intermediate.strips.begin(), intermediate.strips.end(),
          [added_strip_id](const SectionStrip& strip) {
            return strip.id == added_strip_id;
          });
      auto intermediate_lane = std::find_if(
          intermediate.lane_bands.begin(), intermediate.lane_bands.end(),
          [added_lane_id](const LaneBand& lane) {
            return lane.id == added_lane_id;
          });
      intermediate_strip->width_m = request.lane_width_m * end_fraction;
      intermediate_lane->lateral_end_m = intermediate_strip->width_m;
      to_template_id = intermediate.id;
      plan.add_section_templates.push_back(std::move(intermediate));
    }
    const CrossSectionTemplateId corridor_entry_template_id =
        from_template_id;
    const CrossSectionTemplateId corridor_exit_template_id = to_template_id;
    const CrossSectionTemplateId local_from_template_id =
        ref->reversed ? corridor_exit_template_id
                      : corridor_entry_template_id;
    const CrossSectionTemplateId local_to_template_id =
        ref->reversed ? corridor_entry_template_id
                      : corridor_exit_template_id;
    const bool local_from_has_lane = local_from_template_id != source->id;
    const bool local_to_has_lane = local_to_template_id != source->id;
    const TransitionAction action =
        !local_from_has_lane && local_to_has_lane
            ? TransitionAction::kTaperIn
            : (local_from_has_lane && !local_to_has_lane
                   ? TransitionAction::kTaperOut
                   : TransitionAction::kChangeWidthHeightOffset);
    const SectionTransitionId transition_id = next_id++;
    const bool split_at_full_width =
        ref == full_ref &&
        ((!ref->reversed && local_end < current_derived->length_m - kEpsilon) ||
         (ref->reversed && local_start > kEpsilon));
    if (split_at_full_width) {
      const double split_distance = ref->reversed ? local_start : local_end;
      const Path* current_path = FindCanonicalAlignment(derived_, current->id);
      if (current_path == nullptr) {
        return Result<LaneId>::Fail(
            CommitFailureCategory::kInternalError,
            "lane full-width split geometry is missing");
      }
      for (const ManualLineMarking& marking : graph_.manual_lines) {
        if (marking.owner_segment_id != current->id) continue;
        const auto [minimum, maximum] = manual_line_distance_bounds(marking);
        if (minimum < split_distance - kEpsilon &&
            maximum > split_distance + kEpsilon) {
          return Result<LaneId>::Fail(
              CommitFailureCategory::kNotImplemented,
              "lane full-width boundary crosses a manual line marking");
        }
      }
      for (const ManualAreaMarking& marking : graph_.manual_areas) {
        if (marking.owner_segment_id != current->id) continue;
        const auto [minimum, maximum] = manual_area_distance_bounds(marking);
        if (minimum < split_distance - kEpsilon &&
            maximum > split_distance + kEpsilon) {
          return Result<LaneId>::Fail(
              CommitFailureCategory::kNotImplemented,
              "lane full-width boundary crosses a manual area marking");
        }
      }
      const Result<PathSplit> split =
          split_path_at_distance(*current_path, split_distance);
      if (!split.ok) {
        return Result<LaneId>::Fail(split.failure_category, split.error);
      }
      const Result<SegmentShape> before_shape =
          SegmentShapeFromPath(split.value.before);
      const Result<SegmentShape> after_shape =
          SegmentShapeFromPath(split.value.after);
      if (!before_shape.ok || !after_shape.ok) {
        return Result<LaneId>::Fail(
            CommitFailureCategory::kInternalError,
            "lane full-width split shape derivation failed");
      }
      const RoadNodeId split_node = next_id++;
      full_width_split_segment_id = next_id++;
      full_width_split_reversed = ref->reversed;
      plan.add_nodes.push_back(RoadNode{split_node, split.value.point});
      RoadSegment before = *current;
      before.node_b = split_node;
      before.shape = before_shape.value;
      RoadSegment after{full_width_split_segment_id, split_node,
                        current->node_b, after_shape.value, target.id,
                        std::nullopt};
      if (ref->reversed) {
        before.section_template = target.id;
        before.transition.reset();
        after.section_template = local_from_template_id;
        after.transition = transition_id;
      } else {
        before.section_template = local_from_template_id;
        before.transition = transition_id;
      }
      plan.replace_segments.push_back(std::move(before));
      plan.add_segments.push_back(std::move(after));
      plan.add_transitions.push_back(SectionTransition{
          transition_id, local_from_template_id, local_to_template_id,
          DistanceRef{DistanceRefKind::kRatio, 0.0},
          DistanceRef{DistanceRefKind::kRatio, 1.0},
          TransitionAnchor::kBoundary, anchor_boundary_id,
          {SectionTransitionRule{added_strip_id, action}}});

      const ApproachKey old_end{current->node_b, current->id,
                                EndpointRole::kEnd};
      const ApproachKey new_end{current->node_b,
                                full_width_split_segment_id,
                                EndpointRole::kEnd};
      if (const ApproachGeometryOverride* override =
              find_approach_geometry_override(graph_, old_end)) {
        ApproachGeometryOverride mapped = *override;
        mapped.key = new_end;
        plan.remove_approach_geometry_overrides.push_back(old_end);
        plan.add_approach_geometry_overrides.push_back(std::move(mapped));
      }
      for (const JunctionMarkingOverride& override :
           graph_.junction_marking_overrides) {
        const bool source_end = override.source.approach == old_end;
        const bool target_end = override.target.has_value() &&
                                override.target->approach == old_end;
        if (!source_end && !target_end) continue;
        JunctionMarkingOverride mapped = override;
        if (source_end) mapped.source.approach = new_end;
        if (target_end) mapped.target->approach = new_end;
        plan.remove_junction_marking_overrides.push_back(override.id);
        plan.add_junction_marking_overrides.push_back(std::move(mapped));
      }
      for (const ManualLineMarking& marking : graph_.manual_lines) {
        if (marking.owner_segment_id != current->id ||
            manual_line_distance_bounds(marking).first + kEpsilon <
                split_distance) {
          continue;
        }
        ManualLineMarking mapped = marking;
        mapped.owner_segment_id = full_width_split_segment_id;
        shift_manual_line_distance(mapped, -split_distance);
        plan.remove_manual_lines.push_back(marking.id);
        plan.add_manual_lines.push_back(std::move(mapped));
      }
      for (const ManualAreaMarking& marking : graph_.manual_areas) {
        if (marking.owner_segment_id != current->id ||
            manual_area_distance_bounds(marking).first + kEpsilon <
                split_distance) {
          continue;
        }
        ManualAreaMarking mapped = marking;
        mapped.owner_segment_id = full_width_split_segment_id;
        mapped.frame_origin.x -= split_distance;
        plan.remove_manual_areas.push_back(marking.id);
        plan.add_manual_areas.push_back(std::move(mapped));
      }
      for (const AutoMarkingOverride& override :
           graph_.auto_marking_overrides) {
        if (override.key.owner.kind != MarkingOwner::Kind::kRoadSegment ||
            override.key.owner.segment_id != current->id ||
            !override.key.track.has_value()) {
          continue;
        }
        AutoMarkingOverride mapped = override;
        mapped.key.owner.segment_id = full_width_split_segment_id;
        mapped.key.track->segment_id = full_width_split_segment_id;
        plan.add_auto_marking_overrides.push_back(std::move(mapped));
      }
      for (const LaneConnection& connection : graph_.lane_connections) {
        const bool source_end =
            connection.source.segment_id == current->id &&
            connection.source.endpoint_role == EndpointRole::kEnd;
        const bool target_end =
            connection.target.segment_id == current->id &&
            connection.target.endpoint_role == EndpointRole::kEnd;
        if (!source_end && !target_end) continue;
        LaneConnection mapped = connection;
        if (source_end)
          mapped.source.segment_id = full_width_split_segment_id;
        if (target_end)
          mapped.target.segment_id = full_width_split_segment_id;
        plan.remove_lane_connections.push_back(connection.id);
        plan.add_lane_connections.push_back(std::move(mapped));
      }
      for (const BoundaryContinuation& continuation :
           graph_.boundary_continuations) {
        const bool source_end =
            continuation.source.segment_id == current->id &&
            continuation.source.endpoint_role == EndpointRole::kEnd;
        const bool target_end =
            continuation.target.segment_id == current->id &&
            continuation.target.endpoint_role == EndpointRole::kEnd;
        if (!source_end && !target_end) continue;
        BoundaryContinuation mapped = continuation;
        if (source_end)
          mapped.source.segment_id = full_width_split_segment_id;
        if (target_end)
          mapped.target.segment_id = full_width_split_segment_id;
        plan.remove_boundary_continuations.push_back(continuation.id);
        plan.add_boundary_continuations.push_back(std::move(mapped));
      }
    } else {
      plan.add_transitions.push_back(SectionTransition{
          transition_id, local_from_template_id, local_to_template_id,
          DistanceRef{DistanceRefKind::kFromStart, local_start},
          DistanceRef{DistanceRefKind::kFromStart, local_end},
          TransitionAnchor::kBoundary, anchor_boundary_id,
          {SectionTransitionRule{added_strip_id, action}}});
      RoadSegment replacement = *current;
      replacement.section_template = local_from_template_id;
      replacement.transition = transition_id;
      plan.replace_segments.push_back(std::move(replacement));
    }
    from_template_id = corridor_exit_template_id;
    corridor_begin = corridor_end;
  }
  for (auto ref = full_ref + 1; ref != corridor->segments.end(); ++ref) {
    const RoadSegment* following = find_segment(graph_, ref->segment_id);
    if (following == nullptr) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInternalError,
          "lane transition following corridor segment is missing");
    }
    if (following->transition.has_value()) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "added lane conflicts with a later section transition");
    }
    const CrossSectionTemplate* following_section =
        find_template(graph_, following->section_template);
    if (following_section == nullptr) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInternalError,
          "lane addition following corridor section is missing");
    }
    if (!internal::equivalent_section_definition(*following_section, *source)) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "added lane reaches a different unresolved section");
    }
    RoadSegment replacement = *following;
    replacement.section_template = target.id;
    plan.replace_segments.push_back(std::move(replacement));
  }
  RoadCorridor corridor_replacement = *corridor;
  if (full_width_split_segment_id != 0) {
    const auto split_ref = std::find_if(
        corridor_replacement.segments.begin(),
        corridor_replacement.segments.end(),
        [full_ref](const DirectedSegmentRef& ref) {
          return ref.segment_id == full_ref->segment_id;
        });
    if (split_ref == corridor_replacement.segments.end()) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInternalError,
          "lane full-width corridor reference is missing");
    }
    const std::size_t split_index = static_cast<std::size_t>(std::distance(
        corridor_replacement.segments.begin(), split_ref));
    corridor_replacement.segments.erase(split_ref);
    const std::array<DirectedSegmentRef, 2> split_refs =
        full_width_split_reversed
            ? std::array<DirectedSegmentRef, 2>{
                  DirectedSegmentRef{full_width_split_segment_id, true},
                  DirectedSegmentRef{full_ref->segment_id, true}}
            : std::array<DirectedSegmentRef, 2>{
                  DirectedSegmentRef{full_ref->segment_id, false},
                  DirectedSegmentRef{full_width_split_segment_id, false}};
    corridor_replacement.segments.insert(
        corridor_replacement.segments.begin() +
            static_cast<std::ptrdiff_t>(split_index),
        split_refs.begin(), split_refs.end());
  }
  corridor_replacement.section_template_id = target.id;
  plan.replace_corridors.push_back(std::move(corridor_replacement));
  plan.add_section_templates.push_back(std::move(target));
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) {
    return Result<LaneId>::Fail(executed.failure_category, executed.error);
  }
  return Result<LaneId>::Ok(added_lane_id);
}

Result<LaneConnectionId> RoadState::AddLaneConnection(
    AddLaneConnectionRequest request) {
  const internal::LaneEndpointLookup source =
      internal::find_lane_endpoint(graph_, request.source);
  const internal::LaneEndpointLookup target =
      internal::find_lane_endpoint(graph_, request.target);
  if (source.lane == nullptr || target.lane == nullptr) {
    return Result<LaneConnectionId>::Fail(
        CommitFailureCategory::kInvalidInput, "lane connection endpoint does not exist");
  }
  if (static_cast<int>(request.kind) < 0 ||
      static_cast<int>(request.kind) > 4) {
    return Result<LaneConnectionId>::Fail(
        CommitFailureCategory::kInvalidInput, "lane connection kind is invalid");
  }
  operations::OperationPlan plan{};
  const LaneConnectionId id = next_id_;
  plan.add_lane_connections.push_back(
      LaneConnection{id, request.source, request.target, request.kind});
  plan.next_id_after = next_id_ + 1;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) {
    return Result<LaneConnectionId>::Fail(executed.failure_category,
                                          executed.error);
  }
  return Result<LaneConnectionId>::Ok(id);
}

Result<BoundaryContinuationId> RoadState::AddBoundaryContinuation(
    AddBoundaryContinuationRequest request) {
  const internal::BoundaryEndpointLookup source =
      internal::find_boundary_endpoint(graph_, request.source);
  const internal::BoundaryEndpointLookup target =
      internal::find_boundary_endpoint(graph_, request.target);
  if (source.boundary == nullptr || target.boundary == nullptr) {
    return Result<BoundaryContinuationId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "boundary continuation endpoint does not exist");
  }
  operations::OperationPlan plan{};
  const BoundaryContinuationId id = next_id_;
  plan.add_boundary_continuations.push_back(
      BoundaryContinuation{id, request.source, request.target, request.kind});
  plan.next_id_after = next_id_ + 1;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) {
    return Result<BoundaryContinuationId>::Fail(executed.failure_category,
                                                executed.error);
  }
  return Result<BoundaryContinuationId>::Ok(id);
}

Result<bool> RoadState::AttachSectionTransition(AttachSectionTransitionRequest request) {
  const RoadSegmentId segment_id = request.segment_id;
  const SectionTransitionId transition_id = request.transition_id;
  const RoadSegment* segment = find_segment(graph_, segment_id);
  const SectionTransition* transition = find_transition(graph_, transition_id);
  if (segment == nullptr || transition == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road transition attachment reference is missing");
  }
  if (segment->section_template != transition->from_template) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road transition from-template does not match segment");
  }
  const Path* alignment = FindCanonicalAlignment(derived_, segment_id);
  if (alignment == nullptr) return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road segment canonical alignment is missing");
  const auto total = PathLength(*alignment);
  if (!total.ok) return Result<bool>::Fail(total.failure_category, total.error);
  const double start = distance_value(transition->start, total.value);
  const double end = distance_value(transition->end, total.value);
  if (start < 0.0 || end > total.value || end - start <= kEpsilon) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road transition distance range is invalid");
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
    return Result<ManualMarkingId>::Fail(CommitFailureCategory::kInvalidInput, "manual line style is unknown");
  }
  const RoadSegment* owner = find_segment(graph_, marking.owner_segment_id);
  if (owner == nullptr) {
    return Result<ManualMarkingId>::Fail(CommitFailureCategory::kInvalidInput, "manual line owner segment does not exist");
  }
  const Result<bool> valid = ValidatePath(marking.path);
  if (!valid.ok) {
    return Result<ManualMarkingId>::Fail(valid.failure_category, valid.error);
  }
  const Path* alignment = FindCanonicalAlignment(derived_, owner->id);
  if (alignment == nullptr) {
    return Result<ManualMarkingId>::Fail(CommitFailureCategory::kInternalError, "road segment canonical alignment is missing");
  }
  const auto length_result = PathLength(*alignment);
  for (const Vec2d point : FlattenPath(marking.path)) {
    if (!length_result.ok || point.x < 0.0 || point.x > length_result.value || !finite(point.y)) {
      return Result<ManualMarkingId>::Fail(CommitFailureCategory::kInvalidInput, "manual line lies outside owner distance range");
    }
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (marking.id == 0) marking.id = next_id++;
  const ManualMarkingId id = marking.id;
  plan.next_id_after = next_id;
  plan.add_manual_lines.push_back(std::move(marking));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<ManualMarkingId>::Fail(executed.failure_category, executed.error);
  return Result<ManualMarkingId>::Ok(id);
}

Result<ManualMarkingId> RoadState::AddManualArea(ManualAreaRequest request) {
  ManualAreaMarking marking{0, request.owner_segment_id, request.frame_origin,
                            request.rotation_rad, request.width_m,
                            request.length_m, request.style_id};
  if (!IsKnownMarkingStyle(marking.style_id)) {
    return Result<ManualMarkingId>::Fail(CommitFailureCategory::kInvalidInput, "manual area style is unknown");
  }
  const RoadSegment* owner = find_segment(graph_, marking.owner_segment_id);
  if (owner == nullptr) {
    return Result<ManualMarkingId>::Fail(CommitFailureCategory::kInvalidInput, "manual area owner segment does not exist");
  }
  if (!finite(marking.frame_origin) || !finite(marking.rotation_rad) ||
      !finite(marking.width_m) || !finite(marking.length_m) ||
      marking.width_m <= 0.0 || marking.length_m <= 0.0) {
    return Result<ManualMarkingId>::Fail(CommitFailureCategory::kInvalidInput, "manual area shape is invalid");
  }
  const Path* alignment = FindCanonicalAlignment(derived_, owner->id);
  if (alignment == nullptr) {
    return Result<ManualMarkingId>::Fail(CommitFailureCategory::kInternalError, "road segment canonical alignment is missing");
  }
  const auto owner_length = PathLength(*alignment);
  if (!owner_length.ok || marking.frame_origin.x - marking.length_m * 0.5 < 0.0 ||
      marking.frame_origin.x + marking.length_m * 0.5 > owner_length.value) {
    return Result<ManualMarkingId>::Fail(CommitFailureCategory::kInvalidInput, "manual area lies outside owner distance range");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  if (marking.id == 0) marking.id = next_id++;
  const ManualMarkingId id = marking.id;
  plan.next_id_after = next_id;
  plan.add_manual_areas.push_back(std::move(marking));
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<ManualMarkingId>::Fail(executed.failure_category, executed.error);
  return Result<ManualMarkingId>::Ok(id);
}

Result<bool> RoadState::SuppressAutoMarking(SuppressAutoMarkingRequest request) {
  const Result<bool> valid = validate_auto_marking_key(graph_, request.key);
  if (!valid.ok) return valid;
  if (std::any_of(graph_.auto_marking_overrides.begin(),
                  graph_.auto_marking_overrides.end(),
                  [&request](const AutoMarkingOverride& override) {
                    return override.key == request.key && override.suppressed;
                  })) {
    return Result<bool>::Ok(true);
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.add_auto_marking_overrides.push_back(AutoMarkingOverride{request.key, true});
  return Execute(plan);
}

Result<bool> RoadState::ResetAutoMarkingSuppression(ResetAutoMarkingSuppressionRequest request) {
  const Result<bool> valid = validate_auto_marking_key(graph_, request.key);
  if (!valid.ok) return valid;
  const auto existing =
      std::find_if(graph_.auto_marking_overrides.begin(),
                   graph_.auto_marking_overrides.end(),
                   [&request](const AutoMarkingOverride& override) {
                     return override.key == request.key;
                   });
  if (existing == graph_.auto_marking_overrides.end()) {
    return Result<bool>::Ok(true);
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.remove_auto_marking_overrides.push_back(request.key);
  return Execute(plan);
}

Result<JunctionMarkingOverrideId> RoadState::SetJunctionMarkingOverride(
    SetJunctionMarkingOverrideRequest request) {
  JunctionMarkingOverride override = request.override;
  std::uint64_t next_id = next_id_;
  if (override.id == 0) {
    override.id = next_id++;
  }
  const Result<bool> valid = validate_junction_marking_override(graph_, override);
  if (!valid.ok) {
    return Result<JunctionMarkingOverrideId>::Fail(valid.failure_category, valid.error);
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id;
  if (std::any_of(graph_.junction_marking_overrides.begin(),
                  graph_.junction_marking_overrides.end(),
                  [&override](const JunctionMarkingOverride& item) {
                    return item.id == override.id;
                  })) {
    plan.remove_junction_marking_overrides.push_back(override.id);
  }
  plan.add_junction_marking_overrides.push_back(std::move(override));
  const JunctionMarkingOverrideId id = plan.add_junction_marking_overrides.front().id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) {
    return Result<JunctionMarkingOverrideId>::Fail(executed.failure_category, executed.error);
  }
  return Result<JunctionMarkingOverrideId>::Ok(id);
}

Result<bool> RoadState::DeleteJunctionMarkingOverride(DeleteJunctionMarkingOverrideRequest request) {
  if (request.id == 0) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "junction marking override id is invalid");
  }
  if (std::none_of(graph_.junction_marking_overrides.begin(),
                   graph_.junction_marking_overrides.end(),
                   [&request](const JunctionMarkingOverride& item) {
                     return item.id == request.id;
                   })) {
    return Result<bool>::Ok(true);
  }
  operations::OperationPlan plan{};
  plan.next_id_after = next_id_;
  plan.remove_junction_marking_overrides.push_back(request.id);
  return Execute(plan);
}


Result<std::string> RoadState::Save() const {
  return persistence::SaveRoad(graph_, next_id_);
}

Result<RoadState> RoadState::Load(const std::string& text) {
  Result<persistence::LoadedRoad> loaded = persistence::LoadRoad(text);
  if (!loaded.ok) {
    return Result<RoadState>::Fail(loaded.failure_category, loaded.error);
  }
  RoadState state{};
  state.graph_ = std::move(loaded.value.graph);
  state.next_id_ = loaded.value.next_id;
  const Result<bool> authoritative_valid =
      persistence::ValidateAuthoritativeGraph(state.graph_, state.next_id_);
  if (!authoritative_valid.ok) {
    return Result<RoadState>::Fail(authoritative_valid.failure_category,
                                   authoritative_valid.error);
  }
  Result<DerivedRoad> generated = generation::generate_road(state.graph_);
  if (!generated.ok) {
    return Result<RoadState>::Fail(generated.failure_category, generated.error);
  }
  state.derived_ = std::move(generated.value);
  const Result<bool> derived_valid = ValidateGraphInvariants(state.graph_, state.derived_);
  if (!derived_valid.ok) {
    return Result<RoadState>::Fail(derived_valid.failure_category, derived_valid.error);
  }
  return Result<RoadState>::Ok(std::move(state));
}

Result<bool> ValidateGraphInvariants(const SavedRoadGraph& graph, const DerivedRoad& derived) {
  std::unordered_set<std::uint64_t> ids{};
  for (const CrossSectionTemplate& section : graph.section_templates) {
    if (!ids.insert(section.id).second) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road section template ID invariant failed");
    }
  }
  for (const RoadNode& node : graph.nodes) {
    if (!ids.insert(node.id).second || !finite(node.position)) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road node invariant failed");
    }
  }
  for (const RoadSegment& segment : graph.segments) {
    if (!ids.insert(segment.id).second || find_node(graph, segment.node_a) == nullptr ||
        find_node(graph, segment.node_b) == nullptr || find_template(graph, segment.section_template) == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road segment reference invariant failed");
    }
    if (FindDerivedSegment(derived, segment.id) == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road derived segment is missing");
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

  std::size_t resolved_approaches = 0;
  for (const ResolvedConnection& connection : derived.connections) {
    if (find_node(graph, connection.node_id) == nullptr ||
        connection.approaches.size() != connection.ordered_approaches.size()) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                "road resolved connection invariant failed");
    }
    resolved_approaches += connection.approaches.size();
    for (const ResolvedApproach& approach : connection.approaches) {
      const RoadSegment* segment = find_segment(graph, approach.key.segment_id);
      const bool endpoint_matches =
          segment != nullptr && approach.key.node_id == connection.node_id &&
          ((approach.key.endpoint_role == EndpointRole::kStart &&
            segment->node_a == approach.key.node_id) ||
           (approach.key.endpoint_role == EndpointRole::kEnd &&
            segment->node_b == approach.key.node_id));
      if (!endpoint_matches || approach.endpoint_template_id == 0 ||
          !finite(approach.auto_setback_m) || approach.auto_setback_m < 0.0 ||
          !finite(approach.resolved_setback_m) || approach.resolved_setback_m < 0.0 ||
          !finite(approach.resolved_lateral_shift_m) ||
          !finite(approach.gate_segment_distance_m)) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                  "road resolved approach invariant failed");
      }
      const std::size_t same_approach_rows = static_cast<std::size_t>(
          std::count_if(connection.approaches.begin(), connection.approaches.end(),
                        [&approach](const ResolvedApproach& candidate) {
                          return candidate.key == approach.key;
                        }));
      const std::size_t same_order_rows = static_cast<std::size_t>(
          std::count(connection.ordered_approaches.begin(),
                     connection.ordered_approaches.end(), approach.key));
      if (same_approach_rows != 1 || same_order_rows != 1) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                  "road ApproachKey identity invariant failed");
      }

      const ConnectionGate& gate = approach.gate;
      if (gate.segment_id != approach.key.segment_id ||
          gate.node_id != approach.key.node_id || gate.approach != approach.key ||
          !finite(gate.position.x) || !finite(gate.position.y) || !finite(gate.position.z) ||
          !finite(gate.tangent.x) || !finite(gate.tangent.y) || !finite(gate.tangent.z) ||
          !finite(gate.lateral.x) || !finite(gate.lateral.y) || !finite(gate.lateral.z) ||
          !finite(gate.normal.x) || !finite(gate.normal.y) || !finite(gate.normal.z)) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                  "road connection gate frame invariant failed");
      }
      const DerivedSegment* owner = FindDerivedSegment(derived, approach.key.segment_id);
      const SectionEvaluation* section =
          owner == nullptr ? nullptr : FindSectionAt(*owner, approach.gate_segment_distance_m);
      if (section == nullptr || section->boundaries.size() != gate.boundaries.size()) {
        return Result<bool>::Fail(
            CommitFailureCategory::kInternalError,
            "road connection gate section evaluation invariant failed");
      }
      for (std::size_t index = 0; index < gate.boundaries.size(); ++index) {
        const SectionBoundarySample& gate_boundary = gate.boundaries[index];
        const SectionBoundarySample& section_boundary = section->boundaries[index];
        if (gate_boundary.boundary_id != section_boundary.boundary_id ||
            gate_boundary.role != section_boundary.role ||
            gate_boundary.lateral_m != section_boundary.lateral_m ||
            gate_boundary.height_m != section_boundary.height_m ||
            gate_boundary.marking != section_boundary.marking) {
          return Result<bool>::Fail(
              CommitFailureCategory::kInternalError,
              "road connection gate boundary copy invariant failed");
        }
      }
    }
  }
  if (resolved_approaches != expected_approaches) {
    return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                              "road approach coverage invariant failed");
  }
  for (const DerivedSegment& segment : derived.segments) {
    if (find_segment(graph, segment.id) == nullptr || !finite(segment.length_m) ||
        segment.length_m <= 0.0 || segment.alignment.spans.empty()) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road derived segment invariant failed");
    }
    for (const SectionEvaluation& section : segment.sections) {
      if (section.segment_id != segment.id) {
        return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road section owner invariant failed");
      }
      double previous = -std::numeric_limits<double>::infinity();
      for (const SectionBoundarySample& boundary : section.boundaries) {
        if (!finite(boundary.lateral_m) || !finite(boundary.height_m) || boundary.lateral_m < previous) {
          return Result<bool>::Fail(CommitFailureCategory::kInternalError, "section boundary order invariant failed");
        }
        previous = boundary.lateral_m;
      }
    }
  }

  for (const DerivedMarking& marking : derived.markings) {
    const bool owner_exists =
        marking.owner.kind == MarkingOwner::Kind::kRoadSegment
            ? find_segment(graph, marking.owner.segment_id) != nullptr
            : (marking.owner.kind == MarkingOwner::Kind::kJunction
                   ? find_node(graph, marking.owner.node_id) != nullptr
                   : marking.owner.manual_id != 0);
    if (!owner_exists || !IsKnownMarkingStyle(marking.style_id) ||
        !finite(marking.width_m) || marking.width_m <= 0.0 ||
        (marking.points.empty() && marking.polygon.empty())) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road derived marking invariant failed");
    }
  }

  for (const NodeConnectionPolicyOverride& policy : graph.connection_policy_overrides) {
    if (!ids.insert(policy.id).second || find_node(graph, policy.node_id) == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road connection policy override invariant failed");
    }
  }
  for (const ApproachGeometryOverride& override : graph.approach_geometry_overrides) {
    const RoadSegment* segment = find_segment(graph, override.key.segment_id);
    const bool endpoint_matches =
        segment != nullptr &&
        ((override.key.endpoint_role == EndpointRole::kStart &&
          segment->node_a == override.key.node_id) ||
         (override.key.endpoint_role == EndpointRole::kEnd &&
          segment->node_b == override.key.node_id));
    if (!endpoint_matches ||
        (!override.setback_m.has_value && !override.lateral_shift_m.has_value) ||
        (override.setback_m.has_value &&
         (!finite(override.setback_m.value) || override.setback_m.value < 0.0)) ||
        (override.lateral_shift_m.has_value &&
         !finite(override.lateral_shift_m.value))) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                "road approach geometry override invariant failed");
    }
  }
  for (const SectionTransition& transition : graph.transitions) {
    if (!ids.insert(transition.id).second) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road transition ID invariant failed");
    }
  }
  for (const ManualLineMarking& marking : graph.manual_lines) {
    if (!ids.insert(marking.id).second || find_segment(graph, marking.owner_segment_id) == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "manual line ID or owner invariant failed");
    }
  }
  for (const ManualAreaMarking& marking : graph.manual_areas) {
    if (!ids.insert(marking.id).second || find_segment(graph, marking.owner_segment_id) == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "manual area ID or owner invariant failed");
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
  const std::array<const std::vector<Mesh>*, 4> mesh_families{
      &derived.segment_meshes, &derived.marking_meshes, &derived.connection_meshes,
      &derived.junction_meshes};
  for (const auto* family : mesh_families) {
    if (std::any_of(family->begin(), family->end(), [&valid_mesh](const Mesh& mesh) { return !valid_mesh(mesh); })) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError, "road mesh contains invalid geometry");
    }
  }
  return Result<bool>::Ok(true);
}

Result<bool> ValidatePath(const Path& path) {
  if (path.spans.empty()) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road path has no spans");
  }
  for (std::size_t i = 0; i < path.spans.size(); ++i) {
    const BezierSpan& span = path.spans[i];
    if (!finite(span.p0) || !finite(span.p1) || !finite(span.p2) || !finite(span.p3)) {
      return Result<bool>::Fail(CommitFailureCategory::kInvalidInput, "road path contains a non-finite control point");
    }
    if (span_length(span) <= kEpsilon) {
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

} // namespace city::road
