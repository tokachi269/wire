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

[[nodiscard]] EndpointRole endpoint_role_at(const RoadSegment& segment,
                                            RoadNodeId node_id) {
  return segment.node_a == node_id ? EndpointRole::kStart
                                   : EndpointRole::kEnd;
}

[[nodiscard]] bool lane_exits(const LaneBand& lane, EndpointRole role) {
  return (lane.direction == LaneTravelDirection::kAlongSegment &&
          role == EndpointRole::kEnd) ||
         (lane.direction == LaneTravelDirection::kAgainstSegment &&
          role == EndpointRole::kStart);
}

[[nodiscard]] bool lane_enters(const LaneBand& lane, EndpointRole role) {
  return (lane.direction == LaneTravelDirection::kAlongSegment &&
          role == EndpointRole::kStart) ||
         (lane.direction == LaneTravelDirection::kAgainstSegment &&
          role == EndpointRole::kEnd);
}

struct OrderedLaneEndpoint {
  LaneEndpointKey key{};
  double lateral_m = 0.0;
};

[[nodiscard]] Result<std::vector<OrderedLaneEndpoint>> ordered_lane_endpoints(
    const CrossSectionTemplate& section, RoadSegmentId segment_id,
    EndpointRole role, bool exits) {
  std::vector<OrderedLaneEndpoint> result{};
  const double endpoint_sign = role == EndpointRole::kStart ? 1.0 : -1.0;
  for (const LaneBand& lane : section.lane_bands) {
    if ((exits && !lane_exits(lane, role)) ||
        (!exits && !lane_enters(lane, role))) {
      continue;
    }
    const Result<double> lateral =
        internal::lane_template_lateral(section, lane);
    if (!lateral.ok) {
      return Result<std::vector<OrderedLaneEndpoint>>::Fail(
          lateral.failure_category, lateral.error);
    }
    result.push_back({LaneEndpointKey{segment_id, lane.id, role},
                      lateral.value * endpoint_sign});
  }
  std::sort(result.begin(), result.end(),
            [](const OrderedLaneEndpoint& a, const OrderedLaneEndpoint& b) {
              return std::tie(a.lateral_m, a.key.lane_id) <
                     std::tie(b.lateral_m, b.key.lane_id);
            });
  return Result<std::vector<OrderedLaneEndpoint>>::Ok(std::move(result));
}

struct OrderedBoundaryEndpoint {
  BoundaryEndpointKey key{};
  BoundaryRole role = BoundaryRole::kCurb;
  StripFunction left_function = StripFunction::kCarriageway;
  StripFunction right_function = StripFunction::kCarriageway;
  double lateral_order = 0.0;
};

[[nodiscard]] std::vector<OrderedBoundaryEndpoint> ordered_boundary_endpoints(
    const CrossSectionTemplate& section, RoadSegmentId segment_id,
    EndpointRole role) {
  std::vector<OrderedBoundaryEndpoint> result{};
  const double endpoint_sign = role == EndpointRole::kStart ? 1.0 : -1.0;
  for (std::size_t index = 0; index < section.boundaries.size(); ++index) {
    const BoundaryProfile& boundary = section.boundaries[index];
    const StripFunction first = section.strips[index].function;
    const StripFunction second = section.strips[index + 1].function;
    result.push_back({BoundaryEndpointKey{segment_id, boundary.boundary_id, role},
                      boundary.role,
                      endpoint_sign > 0.0 ? first : second,
                      endpoint_sign > 0.0 ? second : first,
                      static_cast<double>(index) * endpoint_sign});
  }
  std::sort(result.begin(), result.end(),
            [](const OrderedBoundaryEndpoint& a,
               const OrderedBoundaryEndpoint& b) {
              return std::tie(a.lateral_order, a.key.boundary_id) <
                     std::tie(b.lateral_order, b.key.boundary_id);
            });
  return result;
}

[[nodiscard]] Result<bool> plan_unique_junction_topology(
    const SavedRoadGraph& graph, RoadSegmentId source_segment_id,
    EndpointRole source_role, LaneId added_lane_id, std::uint64_t& next_id,
    operations::OperationPlan& plan) {
  const RoadSegment* source_segment =
      internal::find_segment(graph, source_segment_id);
  const CrossSectionTemplate* source_section =
      source_segment == nullptr
          ? nullptr
          : internal::find_endpoint_template(graph, *source_segment,
                                             source_role);
  if (source_segment == nullptr || source_section == nullptr) {
    return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                              "added lane junction source is missing");
  }
  const RoadNodeId node_id = source_role == EndpointRole::kStart
                                 ? source_segment->node_a
                                 : source_segment->node_b;
  std::vector<const RoadSegment*> incident{};
  for (const RoadSegment& segment : graph.segments) {
    if (segment.node_a == node_id || segment.node_b == node_id)
      incident.push_back(&segment);
  }
  if (incident.size() < 3) return Result<bool>::Ok(true);

  const Result<std::vector<OrderedLaneEndpoint>> source_lanes =
      ordered_lane_endpoints(*source_section, source_segment_id, source_role,
                             true);
  if (!source_lanes.ok) {
    return Result<bool>::Fail(source_lanes.failure_category, source_lanes.error);
  }
  if (std::none_of(source_lanes.value.begin(), source_lanes.value.end(),
                   [added_lane_id](const OrderedLaneEndpoint& endpoint) {
                     return endpoint.key.lane_id == added_lane_id;
                   })) {
    return Result<bool>::Ok(true);
  }
  const std::vector<OrderedBoundaryEndpoint> source_boundaries =
      ordered_boundary_endpoints(*source_section, source_segment_id,
                                 source_role);
  struct Candidate {
    std::vector<OrderedLaneEndpoint> lanes{};
    std::vector<OrderedBoundaryEndpoint> boundaries{};
  };
  std::vector<Candidate> candidates{};
  for (const RoadSegment* candidate_segment : incident) {
    if (candidate_segment->id == source_segment_id) continue;
    const EndpointRole role = endpoint_role_at(*candidate_segment, node_id);
    const CrossSectionTemplate* section = internal::find_endpoint_template(
        graph, *candidate_segment, role);
    if (section == nullptr) {
      return Result<bool>::Fail(CommitFailureCategory::kInternalError,
                                "junction candidate section is missing");
    }
    Result<std::vector<OrderedLaneEndpoint>> lanes = ordered_lane_endpoints(
        *section, candidate_segment->id, role, false);
    if (!lanes.ok) {
      return Result<bool>::Fail(lanes.failure_category, lanes.error);
    }
    std::vector<OrderedBoundaryEndpoint> boundaries =
        ordered_boundary_endpoints(*section, candidate_segment->id, role);
    if (lanes.value.size() != source_lanes.value.size() ||
        boundaries.size() != source_boundaries.size()) {
      continue;
    }
    const bool boundary_roles_match = std::equal(
        source_boundaries.begin(), source_boundaries.end(), boundaries.begin(),
        [](const OrderedBoundaryEndpoint& a,
           const OrderedBoundaryEndpoint& b) {
          return a.role == b.role &&
                 a.left_function == b.left_function &&
                 a.right_function == b.right_function;
        });
    if (boundary_roles_match)
      candidates.push_back({std::move(lanes.value), std::move(boundaries)});
  }
  if (candidates.empty()) return Result<bool>::Ok(true);
  if (candidates.size() > 1) {
    return Result<bool>::Fail(
        CommitFailureCategory::kNotImplemented,
        "added lane junction destination is ambiguous; choose straight, left, "
        "or right");
  }
  const Candidate& target = candidates.front();
  for (std::size_t index = 0; index < source_lanes.value.size(); ++index) {
    const LaneEndpointKey& source = source_lanes.value[index].key;
    const LaneEndpointKey& destination = target.lanes[index].key;
    const auto exact = std::find_if(
        graph.lane_connections.begin(), graph.lane_connections.end(),
        [&source, &destination](const LaneConnection& connection) {
          return connection.source == source &&
                 connection.target == destination;
        });
    if (exact != graph.lane_connections.end()) continue;
    const bool conflicts = std::any_of(
        graph.lane_connections.begin(), graph.lane_connections.end(),
        [&source, &destination](const LaneConnection& connection) {
          return connection.source == source ||
                 connection.target == destination;
        });
    if (conflicts) {
      return Result<bool>::Fail(
          CommitFailureCategory::kNotImplemented,
          "added lane conflicts with existing junction lane topology");
    }
    plan.add_lane_connections.push_back(LaneConnection{
        next_id++, source, destination,
        LaneConnectionKind::kJunctionMovement});
  }
  for (std::size_t index = 0; index < source_boundaries.size(); ++index) {
    const BoundaryEndpointKey& source = source_boundaries[index].key;
    const BoundaryEndpointKey& destination = target.boundaries[index].key;
    const auto exact = std::find_if(
        graph.boundary_continuations.begin(),
        graph.boundary_continuations.end(),
        [&source, &destination](const BoundaryContinuation& continuation) {
          return continuation.source == source &&
                 continuation.target == destination;
        });
    if (exact != graph.boundary_continuations.end()) continue;
    const bool conflicts = std::any_of(
        graph.boundary_continuations.begin(),
        graph.boundary_continuations.end(),
        [&source, &destination](const BoundaryContinuation& continuation) {
          return continuation.source == source ||
                 continuation.target == destination;
        });
    if (conflicts) {
      return Result<bool>::Fail(
          CommitFailureCategory::kNotImplemented,
          "added lane conflicts with existing junction boundary topology");
    }
    plan.add_boundary_continuations.push_back(BoundaryContinuation{
        next_id++, source, destination,
        BoundaryContinuationKind::kContinuation});
  }
  return Result<bool>::Ok(true);
}

struct OuterLaneSelection {
  std::size_t strip_index = 0;
};

[[nodiscard]] Result<OuterLaneSelection> select_outer_lane_strip(
    const CrossSectionTemplate& section, LaneTravelDirection direction,
    RoadSide side, CommitFailureCategory empty_category,
    const char* empty_error, const char* missing_strip_error) {
  struct Candidate {
    std::size_t strip_index = 0;
    double center_in_strip_m = 0.0;
  };
  std::vector<Candidate> candidates{};
  for (const LaneBand& lane : section.lane_bands) {
    if (lane.direction != direction) continue;
    const auto strip = std::find_if(
        section.strips.begin(), section.strips.end(),
        [&lane](const SectionStrip& item) {
          return item.id == lane.surface_strip_id;
        });
    if (strip == section.strips.end()) {
      return Result<OuterLaneSelection>::Fail(
          CommitFailureCategory::kInternalError, missing_strip_error);
    }
    candidates.push_back(Candidate{
        static_cast<std::size_t>(
            std::distance(section.strips.begin(), strip)),
        (lane.lateral_start_m + lane.lateral_end_m) * 0.5});
  }
  if (candidates.empty()) {
    return Result<OuterLaneSelection>::Fail(empty_category, empty_error);
  }
  const auto extremes = std::minmax_element(
      candidates.begin(), candidates.end(),
      [](const Candidate& a, const Candidate& b) {
        return std::tie(a.strip_index, a.center_in_strip_m) <
               std::tie(b.strip_index, b.center_in_strip_m);
      });
  return Result<OuterLaneSelection>::Ok(
      OuterLaneSelection{side == RoadSide::kRight
                             ? extremes.second->strip_index
                             : extremes.first->strip_index});
}

struct LaneSectionIds {
  SectionStripId strip_id = 0;
  LaneId lane_id = 0;
  BoundaryId boundary_id = 0;
};

[[nodiscard]] std::uint64_t next_section_member_id(
    const SavedRoadGraph& graph) {
  std::uint64_t next_id = 1;
  for (const CrossSectionTemplate& section : graph.section_templates) {
    for (const SectionStrip& strip : section.strips)
      next_id = std::max(next_id, strip.id + 1);
    for (const LaneBand& lane : section.lane_bands)
      next_id = std::max(next_id, lane.id + 1);
    for (const BoundaryProfile& boundary : section.boundaries)
      next_id = std::max(next_id, boundary.boundary_id + 1);
  }
  return next_id;
}

[[nodiscard]] Result<CrossSectionTemplate> make_extended_lane_section(
    const CrossSectionTemplate& base, LaneTravelDirection direction,
    RoadSide side, double lane_width_m, LaneSectionIds ids,
    CommitFailureCategory empty_category, const char* empty_error,
    const char* missing_strip_error) {
  const Result<OuterLaneSelection> selected = select_outer_lane_strip(
      base, direction, side, empty_category, empty_error,
      missing_strip_error);
  if (!selected.ok) {
    return Result<CrossSectionTemplate>::Fail(selected.failure_category,
                                               selected.error);
  }

  CrossSectionTemplate extended = base;
  SectionStrip strip = base.strips[selected.value.strip_index];
  strip.id = ids.strip_id;
  strip.width_m = lane_width_m;
  strip.side_marking = {};
  const BoundaryProfile divider{
      ids.boundary_id, BoundaryRole::kLaneDivider, 0.0, 0.0,
      AutoMarkingPolicy{true, MarkingRole::kLaneSeparator,
                        builtin_marking_styles::kWhiteDashed}};
  if (side == RoadSide::kRight) {
    extended.strips.insert(
        extended.strips.begin() + selected.value.strip_index + 1, strip);
    extended.boundaries.insert(
        extended.boundaries.begin() + selected.value.strip_index, divider);
  } else {
    extended.strips.insert(
        extended.strips.begin() + selected.value.strip_index, strip);
    extended.boundaries.insert(
        extended.boundaries.begin() + selected.value.strip_index, divider);
  }
  extended.lane_bands.push_back(
      LaneBand{ids.lane_id, ids.strip_id, 0.0, lane_width_m, direction});
  return Result<CrossSectionTemplate>::Ok(std::move(extended));
}

struct LaneSectionExtension {
  CrossSectionTemplateId source_id = 0;
  LaneTravelDirection direction = LaneTravelDirection::kAlongSegment;
  RoadSide side = RoadSide::kRight;
  CrossSectionTemplateId target_id = 0;
};

[[nodiscard]] Result<CrossSectionTemplateId> ensure_extended_lane_section(
    const CrossSectionTemplate& base, LaneTravelDirection direction,
    RoadSide side, double lane_width_m, LaneSectionIds ids,
    std::vector<LaneSectionExtension>& extensions,
    operations::OperationPlan& plan, std::uint64_t& next_id) {
  const auto existing = std::find_if(
      extensions.begin(), extensions.end(),
      [&base, direction, side](const LaneSectionExtension& extension) {
        return extension.source_id == base.id &&
               extension.direction == direction && extension.side == side;
      });
  if (existing != extensions.end()) {
    return Result<CrossSectionTemplateId>::Ok(existing->target_id);
  }
  Result<CrossSectionTemplate> extended = make_extended_lane_section(
      base, direction, side, lane_width_m, ids,
      CommitFailureCategory::kNotImplemented,
      "later section has no lane in the selected direction",
      "lane extension source strip is missing");
  if (!extended.ok) {
    return Result<CrossSectionTemplateId>::Fail(extended.failure_category,
                                                extended.error);
  }
  extended.value.id = next_id++;
  const CrossSectionTemplateId id = extended.value.id;
  plan.add_section_templates.push_back(std::move(extended.value));
  extensions.push_back(LaneSectionExtension{base.id, direction, side, id});
  return Result<CrossSectionTemplateId>::Ok(id);
}

[[nodiscard]] Result<bool> validate_add_lane_request(
    const AddLaneRequest& request) {
  if (!finite(request.transition_start.t) ||
      !finite(request.transition_complete.t)) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "lane transition positions must be finite");
  }
  if (request.transition_start.segment_id == 0 ||
      request.transition_complete.segment_id == 0 ||
      request.continuation_end_node_id == 0 ||
      request.transition_start.t < 0.0 ||
      request.transition_start.t > 1.0 ||
      request.transition_complete.t < 0.0 ||
      request.transition_complete.t > 1.0) {
    return Result<bool>::Fail(
        CommitFailureCategory::kInvalidInput,
        "lane transition positions or continuation endpoint are invalid");
  }
  if (request.transition_start.segment_id !=
      request.transition_complete.segment_id) {
    return Result<bool>::Fail(
        CommitFailureCategory::kNotImplemented,
        "lane transition start and completion must use the same road segment");
  }
  if (!finite(request.lane_width_m) || request.lane_width_m <= 0.0) {
    return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                              "lane width must be finite and positive");
  }
  return Result<bool>::Ok(true);
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




[[nodiscard]] Vec2d path_start(const Path& path) {
  return span_start(path.spans.front());
}

[[nodiscard]] Vec2d path_end(const Path& path) {
  return span_end(path.spans.back());
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
      // A lane side line belongs to a lane, so only a carriageway strip can ask
      // for one.
      if (side.enabled && strip.function != StripFunction::kCarriageway) {
        return Result<bool>::Fail(CommitFailureCategory::kInvalidInput,
                                  "lane side marking requires a carriageway strip");
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

namespace {

// The tangent an interval needs at its far end to stay on the same arc as the
// tangent already decided at this end. A terminal endpoint has no neighbour to
// average with, so it follows the interval instead of its own chord.
[[nodiscard]] Vec2d mirror_tangent_across(Vec2d tangent, Vec2d chord);

// The heading the next interval inherits at a corridor end, expressed as the
// terminal handle of the interval that already ends there.
[[nodiscard]] std::optional<Vec2d> corridor_terminal_handle(const SavedRoadGraph& graph,
                                                            RoadCorridorId corridor_id,
                                                            RoadNodeId endpoint_node_id) {
  const RoadCorridor* corridor = FindRoadCorridor(graph, corridor_id);
  if (corridor == nullptr || corridor->segments.empty()) return std::nullopt;
  const DirectedSegmentRef last_ref = corridor->segments.back();
  const RoadSegment* source = find_segment(graph, last_ref.segment_id);
  if (source == nullptr) return std::nullopt;
  const RoadNodeId corridor_end = last_ref.reversed ? source->node_a : source->node_b;
  if (corridor_end != endpoint_node_id) return std::nullopt;
  if (source->shape.intent != SegmentShapeIntent::kCurve) return std::nullopt;
  return last_ref.reversed ? source->shape.start_handle : source->shape.end_handle;
}

// An interval leaves in the heading it inherits and turns to the new point as
// one arc, so the drawn curve keeps a single bend instead of an S.
void apply_inherited_arc(Vec2d inherited, Vec2d chord, SegmentShape* shape) {
  const double inherited_length = length(inherited);
  const double chord_length = length(chord);
  // Degenerate input keeps the chord handles the caller already set.
  if (inherited_length <= kEpsilon || chord_length <= kEpsilon) return;
  const Vec2d heading = mul(inherited, -1.0 / inherited_length);
  shape->start_handle = mul(heading, chord_length / 3.0);
  shape->end_handle = mul(mirror_tangent_across(heading, chord), -chord_length / 3.0);
}

[[nodiscard]] Vec2d mirror_tangent_across(Vec2d tangent, Vec2d chord) {
  const double chord_length = length(chord);
  if (chord_length <= kEpsilon) return tangent;
  const Vec2d axis = mul(chord, 1.0 / chord_length);
  const double projection = tangent.x * axis.x + tangent.y * axis.y;
  return Vec2d{2.0 * projection * axis.x - tangent.x,
               2.0 * projection * axis.y - tangent.y};
}

} // namespace

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

void align_last_span_end(Path& path, Vec2d end) {
  BezierSpan& last_span = path.spans.back();
  if (IsLinearSpan(last_span)) {
    last_span = MakeLine(last_span.p0, end);
    return;
  }
  const Vec2d correction = sub(end, last_span.p3);
  last_span.p2 = add(last_span.p2, correction);
  last_span.p3 = end;
}

Result<SegmentShape> make_linear_shape(Vec2d start, Vec2d end) {
  return SegmentShapeFromPath(MakePath({MakeLine(start, end)}));
}

Path PreviewDrawnInterval(const SavedRoadGraph& graph, RoadCorridorId corridor_id,
                          RoadNodeId endpoint_node_id, Vec2d start, Vec2d end,
                          SegmentShapeIntent intent) {
  const Vec2d chord = sub(end, start);
  SegmentShape shape{};
  shape.intent = intent;
  shape.start_handle = mul(chord, 1.0 / 3.0);
  shape.end_handle = mul(chord, -1.0 / 3.0);
  if (intent == SegmentShapeIntent::kCurve) {
    if (const std::optional<Vec2d> inherited =
            corridor_terminal_handle(graph, corridor_id, endpoint_node_id)) {
      apply_inherited_arc(*inherited, chord, &shape);
    }
  }
  return MakePath({MakeBezier(start, add(start, shape.start_handle),
                              add(end, shape.end_handle), end)});
}

// A new road starts with no section at all. Which cross sections a product
// offers, and what they measure, belongs to whoever presents them.
RoadState::RoadState() = default;

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
  Result<SegmentShape> shape = SegmentShapeFromPath(alignment);
  if (!shape.ok) {
    return Result<RoadSegmentId>::Fail(shape.failure_category, shape.error);
  }
  if (request.intent.has_value()) {
    shape.value.intent = *request.intent;
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
  if (request.intent.has_value()) {
    shape.value.intent = *request.intent;
  }

  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId end_node = next_id++;
  const RoadSegmentId segment_id = next_id++;

  // The interval leaves in the heading it arrived with and turns to the new
  // point as one arc, so the drawn curve keeps a single bend instead of an S.
  if (shape.value.intent == SegmentShapeIntent::kCurve) {
    if (const std::optional<Vec2d> inherited =
            corridor_terminal_handle(graph_, request.corridor_id, endpoint->id)) {
      apply_inherited_arc(*inherited, sub(path_end(extension), endpoint->position),
                          &shape.value);
    }
  }

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
  const RoadNodeId connected_node = request.start_node;
  const RoadNode* node = find_node(graph_, connected_node);
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
  if (request.connected_endpoint == EndpointRole::kStart) {
    align_first_span_start(alignment, node->position);
  } else {
    align_last_span_end(alignment, node->position);
  }
  const Result<SegmentShape> shape = SegmentShapeFromPath(alignment);
  if (!shape.ok) return Result<RoadSegmentId>::Fail(shape.failure_category, shape.error);
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId free_node = next_id++;
  const RoadSegmentId segment_id = next_id++;
  const RoadCorridorId corridor_id = next_id++;
  const bool connects_at_start =
      request.connected_endpoint == EndpointRole::kStart;
  plan.add_nodes.push_back(RoadNode{
      free_node, connects_at_start ? path_end(alignment) : path_start(alignment)});
  plan.add_segments.push_back(RoadSegment{
      segment_id,
      connects_at_start ? connected_node : free_node,
      connects_at_start ? free_node : connected_node,
      shape.value, section_template, std::nullopt});
  plan.add_corridors.push_back(
      RoadCorridor{corridor_id, section_template,
                   {DirectedSegmentRef{segment_id, false}}});
  plan.next_id_after = next_id;
  const Result<bool> executed = Execute(plan);
  if (!executed.ok) return Result<RoadSegmentId>::Fail(executed.failure_category, executed.error);
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
  const bool connects_at_start =
      request.connected_endpoint == EndpointRole::kStart;
  const Vec2d connected_point =
      connects_at_start ? path_start(alignment) : path_end(alignment);
  if (distance(connected_point, path_split.value.point) > kSnapDistancePointToleranceM) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kInvalidInput,
                                       "road segment input endpoint does not match its explicit snap distance");
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
  if (connects_at_start) {
    align_first_span_start(alignment, path_split.value.point);
  } else {
    align_last_span_end(alignment, path_split.value.point);
  }
  const Result<double> branch_length = PathLength(alignment);
  if (!branch_length.ok) return Result<RoadSegmentId>::Fail(branch_length.failure_category, branch_length.error);
  if (branch_length.value < kP1MinSegmentLengthM) {
    return Result<RoadSegmentId>::Fail(CommitFailureCategory::kNotImplemented, "connected road segment is shorter than P1 minimum");
  }
  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  const RoadNodeId split_node = next_id++;
  const RoadSegmentId second_id = next_id++;
  const RoadNodeId branch_free_node = next_id++;
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
  plan.add_nodes = {
      RoadNode{split_node, path_split.value.point},
      RoadNode{branch_free_node,
               connects_at_start ? path_end(alignment) : path_start(alignment)}};
  plan.add_segments = {
      RoadSegment{second_id, split_node, source->node_b, second_shape.value, source->section_template,
                  source->transition},
      RoadSegment{branch_id,
                  connects_at_start ? split_node : branch_free_node,
                  connects_at_start ? branch_free_node : split_node,
                  branch_shape.value, section_template, std::nullopt},
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
  std::optional<SectionTransition> remapped_transition{};
  bool transition_moves_to_second = false;
  if (source->transition.has_value()) {
    const SectionTransition* transition =
        find_transition(graph_, *source->transition);
    const Result<double> source_length = PathLength(*source_path);
    if (transition == nullptr || !source_length.ok ||
        source_length.value <= kEpsilon) {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kInternalError,
          "transitioning road split data is incomplete");
    }
    if (transition->start.kind != DistanceRefKind::kRatio ||
        transition->end.kind != DistanceRefKind::kRatio) {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "only segment-local ratio transitions can be split");
    }
    const double split_t = request.segment_distance_m / source_length.value;
    const double start_t = transition->start.value;
    const double complete_t = transition->end.value;
    remapped_transition = *transition;
    if (split_t < start_t - kEpsilon) {
      transition_moves_to_second = true;
      remapped_transition->start.value =
          (start_t - split_t) / (1.0 - split_t);
      remapped_transition->end.value =
          (complete_t - split_t) / (1.0 - split_t);
    } else if (split_t > complete_t + kEpsilon) {
      remapped_transition->start.value = start_t / split_t;
      remapped_transition->end.value = complete_t / split_t;
    } else {
      return Result<RoadSegmentId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "a road cannot be split inside its section transition; choose a position before or after the lane change");
    }
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
  if (remapped_transition.has_value() && transition_moves_to_second) {
    first.transition.reset();
  }
  plan.replace_segments.push_back(std::move(first));
  plan.add_nodes.push_back(RoadNode{split_node, split.value.point});
  RoadSegment second{second_id, split_node, source->node_b, second_shape.value,
                     source->section_template, source->transition};
  if (remapped_transition.has_value()) {
    if (transition_moves_to_second) {
      second.section_template = remapped_transition->from_template;
    } else {
      second.section_template = remapped_transition->to_template;
      second.transition.reset();
    }
    plan.remove_transitions.push_back(remapped_transition->id);
    plan.add_transitions.push_back(*remapped_transition);
  }
  plan.add_segments.push_back(std::move(second));

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

Result<LaneId> RoadState::AddLane(AddLaneRequest request) {
  const Result<bool> valid_request = validate_add_lane_request(request);
  if (!valid_request.ok) {
    return Result<LaneId>::Fail(valid_request.failure_category,
                                valid_request.error);
  }
  const RoadCorridor* corridor =
      FindRoadCorridor(graph_, request.corridor_id);
  if (corridor == nullptr || corridor->segments.empty()) {
    return Result<LaneId>::Fail(CommitFailureCategory::kInvalidInput,
                                "lane transition corridor does not exist");
  }
  const RoadSegment* segment =
      find_segment(graph_, request.transition_start.segment_id);
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
        CommitFailureCategory::kInvalidInput,
        "this road segment already has a section transition; overlapping lane changes are not supported");
  }
  const Result<OuterLaneSelection> selected = select_outer_lane_strip(
      *source, local_direction, local_side,
      CommitFailureCategory::kInvalidInput,
      "selected direction has no lane to extend",
      "lane transition source lane strip is missing");
  if (!selected.ok) {
    return Result<LaneId>::Fail(selected.failure_category, selected.error);
  }
  const std::size_t lane_strip_index = selected.value.strip_index;
  const std::optional<std::size_t> anchor_index =
      local_side == RoadSide::kRight
          ? (lane_strip_index > 0
                 ? std::optional<std::size_t>{lane_strip_index - 1}
                 : std::nullopt)
          : (lane_strip_index < source->boundaries.size()
                 ? std::optional<std::size_t>{lane_strip_index}
                 : std::nullopt);
  const TransitionAnchor anchor =
      anchor_index.has_value()
          ? TransitionAnchor::kBoundary
          : (local_side == RoadSide::kRight ? TransitionAnchor::kLeftEdge
                                             : TransitionAnchor::kRightEdge);
  const BoundaryId anchor_boundary_id =
      anchor_index.has_value()
          ? source->boundaries[*anchor_index].boundary_id
          : 0;

  std::uint64_t next_local_id = next_section_member_id(graph_);
  const LaneSectionIds lane_ids{next_local_id++, next_local_id++,
                                next_local_id++};
  Result<CrossSectionTemplate> target_result = make_extended_lane_section(
      *source, local_direction, local_side, request.lane_width_m, lane_ids,
      CommitFailureCategory::kInvalidInput,
      "selected direction has no lane to extend",
      "lane transition source lane strip is missing");
  if (!target_result.ok) {
    return Result<LaneId>::Fail(target_result.failure_category,
                                target_result.error);
  }
  CrossSectionTemplate target = std::move(target_result.value);
  const LaneId added_lane_id = lane_ids.lane_id;

  operations::OperationPlan plan{};
  std::uint64_t next_id = next_id_;
  target.id = next_id++;
  const auto full_ref = std::find_if(
      selected_ref, corridor->segments.end(),
      [&request](const DirectedSegmentRef& ref) {
        return ref.segment_id == request.transition_complete.segment_id;
      });
  if (full_ref == corridor->segments.end()) {
    return Result<LaneId>::Fail(
        CommitFailureCategory::kInternalError,
        "lane transition completion segment is not in its corridor");
  }
  std::vector<LaneSectionExtension> section_extensions{
      LaneSectionExtension{source->id, local_direction, local_side,
                           target.id}};
  const auto continuation_ref = std::find_if(
      full_ref, corridor->segments.end(), [&](const DirectedSegmentRef& ref) {
        const RoadSegment* item = find_segment(graph_, ref.segment_id);
        if (item == nullptr) return false;
        const RoadNodeId exit_node = ref.reversed ? item->node_a : item->node_b;
        return exit_node == request.continuation_end_node_id;
      });
  if (continuation_ref == corridor->segments.end()) {
    return Result<LaneId>::Fail(
        CommitFailureCategory::kInvalidInput,
        "lane continuation endpoint is not reachable on the corridor");
  }
  const RoadSegmentId terminal_segment_id = continuation_ref->segment_id;
  const EndpointRole terminal_role = continuation_ref->reversed
                                         ? EndpointRole::kStart
                                         : EndpointRole::kEnd;
  const auto execute_lane_plan = [&]() -> Result<LaneId> {
    plan.next_id_after = next_id;
    SavedRoadGraph planned_graph = graph_;
    std::uint64_t planned_next_id = next_id_;
    const Result<bool> planned =
        operations::Apply(plan, planned_graph, planned_next_id);
    if (!planned.ok) {
      return Result<LaneId>::Fail(planned.failure_category, planned.error);
    }
    const RoadCorridor* planned_corridor =
        FindRoadCorridor(planned_graph, request.corridor_id);
    if (planned_corridor == nullptr || planned_corridor->segments.empty()) {
      return Result<LaneId>::Fail(CommitFailureCategory::kInternalError,
                                  "planned lane corridor is missing");
    }
    const Result<bool> topology = plan_unique_junction_topology(
        planned_graph, terminal_segment_id, terminal_role, added_lane_id,
        next_id, plan);
    if (!topology.ok) {
      return Result<LaneId>::Fail(topology.failure_category, topology.error);
    }
    plan.next_id_after = next_id;
    const Result<bool> executed = Execute(plan);
    if (!executed.ok) {
      return Result<LaneId>::Fail(executed.failure_category, executed.error);
    }
    return Result<LaneId>::Ok(added_lane_id);
  };

  {
    if (selected_ref != full_ref) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kNotImplemented,
          "lane transition start and completion must use the same road segment");
    }
    const DerivedSegment* selected_derived =
        FindDerivedSegment(derived_, segment->id);
    if (selected_derived == nullptr || selected_derived->length_m <= kEpsilon) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInternalError,
          "lane transition segment geometry is missing");
    }
    const double first_t = request.transition_start.t;
    const double second_t = request.transition_complete.t;
    const double transition_start_t = std::min(first_t, second_t);
    const double transition_complete_t = std::max(first_t, second_t);
    if (transition_start_t < 0.0 || transition_complete_t > 1.0 ||
        transition_complete_t - transition_start_t <= kEpsilon) {
      return Result<LaneId>::Fail(CommitFailureCategory::kInvalidInput,
                                  "lane transition positions are invalid");
    }

    const bool reversed = selected_ref->reversed;
    const bool positions_follow_corridor = reversed
                                               ? first_t > second_t
                                               : first_t < second_t;
    if (!positions_follow_corridor) {
      return Result<LaneId>::Fail(
          CommitFailureCategory::kInvalidInput,
          "lane transition completion must follow its start along the corridor");
    }
    const CrossSectionTemplateId local_from_template_id =
        reversed ? target.id : source->id;
    const CrossSectionTemplateId local_to_template_id =
        reversed ? source->id : target.id;
    const TransitionAction action =
        reversed ? TransitionAction::kTaperOut : TransitionAction::kTaperIn;
    const SectionTransitionId transition_id = next_id++;
    plan.add_transitions.push_back(SectionTransition{
        transition_id, local_from_template_id, local_to_template_id,
        DistanceRef{DistanceRefKind::kRatio, transition_start_t},
        DistanceRef{DistanceRefKind::kRatio, transition_complete_t}, anchor,
        anchor_boundary_id,
        {SectionTransitionRule{lane_ids.strip_id, action}}});
    RoadSegment replacement = *segment;
    replacement.section_template = local_from_template_id;
    replacement.transition = transition_id;
    plan.replace_segments.push_back(std::move(replacement));

    CrossSectionTemplateId terminal_template_id = target.id;
    for (auto ref = full_ref + 1; ref <= continuation_ref; ++ref) {
      const RoadSegment* following = find_segment(graph_, ref->segment_id);
      if (following == nullptr) {
        return Result<LaneId>::Fail(
            CommitFailureCategory::kInternalError,
            "lane addition following corridor segment is missing");
      }
      if (following->transition.has_value()) {
        return Result<LaneId>::Fail(
            CommitFailureCategory::kNotImplemented,
            "lane addition conflicts with an existing section transition");
      }
      const CrossSectionTemplate* following_section =
          find_template(graph_, following->section_template);
      if (following_section == nullptr) {
        return Result<LaneId>::Fail(
            CommitFailureCategory::kInternalError,
            "lane addition following corridor section is missing");
      }
      const LaneTravelDirection following_direction =
          ref->reversed
              ? (request.direction == LaneTravelDirection::kAlongSegment
                     ? LaneTravelDirection::kAgainstSegment
                     : LaneTravelDirection::kAlongSegment)
              : request.direction;
      const RoadSide following_side =
          ref->reversed
              ? (request.side == RoadSide::kLeft ? RoadSide::kRight
                                                 : RoadSide::kLeft)
              : request.side;
      const Result<CrossSectionTemplateId> mapped =
          ensure_extended_lane_section(
              *following_section, following_direction, following_side,
              request.lane_width_m, lane_ids, section_extensions, plan,
              next_id);
      if (!mapped.ok) {
        return Result<LaneId>::Fail(mapped.failure_category, mapped.error);
      }
      RoadSegment following_replacement = *following;
      following_replacement.section_template = mapped.value;
      plan.replace_segments.push_back(std::move(following_replacement));
      terminal_template_id = mapped.value;
    }
    if (continuation_ref + 1 == corridor->segments.end()) {
      RoadCorridor corridor_replacement = *corridor;
      corridor_replacement.section_template_id = terminal_template_id;
      plan.replace_corridors.push_back(std::move(corridor_replacement));
    }
    plan.add_section_templates.push_back(std::move(target));
    return execute_lane_plan();
  }
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
