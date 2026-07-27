#include "stage_support.hpp"

#include <algorithm>
#include <cmath>

namespace city::road::build {

bool IsFinite(double value) {
  return std::isfinite(value);
}

bool IsFinite(Vec2d value) {
  return IsFinite(value.x) && IsFinite(value.y);
}

Vec2d Add(Vec2d a, Vec2d b) {
  return {a.x + b.x, a.y + b.y};
}

Vec2d Subtract(Vec2d a, Vec2d b) {
  return {a.x - b.x, a.y - b.y};
}

Vec2d Scale(Vec2d value, double scale) {
  return {value.x * scale, value.y * scale};
}

double Dot(Vec2d a, Vec2d b) {
  return a.x * b.x + a.y * b.y;
}

double Cross(Vec2d a, Vec2d b) {
  return a.x * b.y - a.y * b.x;
}

double Length(Vec2d value) {
  return std::hypot(value.x, value.y);
}

Vec2d Normalize(Vec2d value) {
  const double length = Length(value);
  if (length <= kStationEpsilon) return {};
  return {value.x / length, value.y / length};
}

Vec3d To3(Vec2d value, double z) {
  return {value.x, value.y, z};
}

const RoadNode* FindNode(const SavedRoadGraph& graph, RoadNodeId id) {
  const auto found = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                                  [id](const RoadNode& node) { return node.id == id; });
  return found == graph.nodes.end() ? nullptr : &*found;
}

const RoadSegment* FindSegment(const SavedRoadGraph& graph, RoadSegmentId id) {
  const auto found = std::find_if(graph.segments.begin(), graph.segments.end(),
                                  [id](const RoadSegment& segment) { return segment.id == id; });
  return found == graph.segments.end() ? nullptr : &*found;
}

const CrossSectionTemplate* FindTemplate(const SavedRoadGraph& graph, CrossSectionTemplateId id) {
  const auto found = std::find_if(graph.section_templates.begin(), graph.section_templates.end(),
                                  [id](const CrossSectionTemplate& section) { return section.id == id; });
  return found == graph.section_templates.end() ? nullptr : &*found;
}

const SectionTransition* FindTransition(const SavedRoadGraph& graph, SectionTransitionId id) {
  const auto found = std::find_if(graph.transitions.begin(), graph.transitions.end(),
                                  [id](const SectionTransition& transition) { return transition.id == id; });
  return found == graph.transitions.end() ? nullptr : &*found;
}

const NodeConnectionPolicyOverride* FindPolicyOverride(const SavedRoadGraph& graph,
                                                       RoadNodeId node_id) {
  const auto found = std::find_if(
      graph.connection_policy_overrides.begin(), graph.connection_policy_overrides.end(),
      [node_id](const NodeConnectionPolicyOverride& policy) { return policy.node_id == node_id; });
  return found == graph.connection_policy_overrides.end() ? nullptr : &*found;
}

const ApproachGeometryOverride* FindApproachGeometryOverride(const SavedRoadGraph& graph,
                                                             const ApproachKey& key) {
  const auto found = std::find_if(
      graph.approach_geometry_overrides.begin(), graph.approach_geometry_overrides.end(),
      [&key](const ApproachGeometryOverride& value) { return value.key == key; });
  return found == graph.approach_geometry_overrides.end() ? nullptr : &*found;
}

const Path* FindAlignment(const DerivedRoad& derived, RoadSegmentId segment_id) {
  return FindCanonicalAlignment(derived, segment_id);
}

const NodeConnectionDecision* FindDecision(const DerivedRoad& derived, RoadNodeId node_id) {
  const auto found = std::find_if(
      derived.node_connection_decisions.begin(), derived.node_connection_decisions.end(),
      [node_id](const NodeConnectionDecision& decision) { return decision.node_id == node_id; });
  return found == derived.node_connection_decisions.end() ? nullptr : &*found;
}

const ApproachConnectionDecision* FindApproachDecision(const NodeConnectionDecision& decision,
                                                       const ApproachKey& key) {
  const auto found = std::find_if(
      decision.approaches.begin(), decision.approaches.end(),
      [&key](const ApproachConnectionDecision& approach) { return approach.key == key; });
  return found == decision.approaches.end() ? nullptr : &*found;
}

const AutoNodeLayout* FindAutoNodeLayout(const DerivedRoad& derived, RoadNodeId node_id) {
  const auto found = std::find_if(
      derived.auto_node_layouts.begin(), derived.auto_node_layouts.end(),
      [node_id](const AutoNodeLayout& layout) { return layout.node_id == node_id; });
  return found == derived.auto_node_layouts.end() ? nullptr : &*found;
}

const AutoApproachLayout* FindAutoApproachLayout(const AutoNodeLayout& layout,
                                                 const ApproachKey& key) {
  const auto found = std::find_if(
      layout.approaches.begin(), layout.approaches.end(),
      [&key](const AutoApproachLayout& approach) { return approach.key == key; });
  return found == layout.approaches.end() ? nullptr : &*found;
}

const ResolvedNodeLayout* FindResolvedNodeLayout(const DerivedRoad& derived, RoadNodeId node_id) {
  const auto found = std::find_if(
      derived.resolved_node_layouts.begin(), derived.resolved_node_layouts.end(),
      [node_id](const ResolvedNodeLayout& layout) { return layout.node_id == node_id; });
  return found == derived.resolved_node_layouts.end() ? nullptr : &*found;
}

const ResolvedApproachLayout* FindResolvedApproachLayout(const ResolvedNodeLayout& layout,
                                                         const ApproachKey& key) {
  const auto found = std::find_if(
      layout.approaches.begin(), layout.approaches.end(),
      [&key](const ResolvedApproachLayout& approach) { return approach.key == key; });
  return found == layout.approaches.end() ? nullptr : &*found;
}

const SegmentSamplingPlan* FindSamplingPlan(const DerivedRoad& derived,
                                            RoadSegmentId segment_id) {
  const auto found = std::find_if(
      derived.sampling_plans.begin(), derived.sampling_plans.end(),
      [segment_id](const SegmentSamplingPlan& plan) { return plan.segment_id == segment_id; });
  return found == derived.sampling_plans.end() ? nullptr : &*found;
}

const SectionEvaluation* FindSectionEvaluation(const DerivedRoad& derived,
                                               RoadSegmentId segment_id,
                                               double station_m) {
  const SectionEvaluation* match = nullptr;
  for (const SectionEvaluation& section : derived.section_evaluations) {
    if (section.segment_id != segment_id ||
        std::abs(section.station_m - station_m) > kStationEpsilon) {
      continue;
    }
    if (match != nullptr) return nullptr;
    match = &section;
  }
  return match;
}

ApproachKey MakeApproachKey(const RoadSegment& segment, RoadNodeId node_id) {
  return ApproachKey{
      node_id,
      segment.id,
      node_id == segment.node_a ? EndpointRole::kStart : EndpointRole::kEnd,
  };
}

Result<Vec2d> InwardTangent(const RoadSegment& segment,
                            const Path& alignment,
                            const ApproachKey& key) {
  if (key.segment_id != segment.id ||
      (key.endpoint_role == EndpointRole::kStart && key.node_id != segment.node_a) ||
      (key.endpoint_role == EndpointRole::kEnd && key.node_id != segment.node_b)) {
    return Result<Vec2d>::Fail(ErrorKind::kInternal, "road approach key does not match segment endpoint");
  }
  const BezierSpan& span = key.endpoint_role == EndpointRole::kStart
                               ? alignment.spans.front()
                               : alignment.spans.back();
  const std::array<Vec2d, 3> candidates =
      key.endpoint_role == EndpointRole::kStart
          ? std::array<Vec2d, 3>{Subtract(span.p1, span.p0),
                                 Subtract(span.p2, span.p0),
                                 Subtract(span.p3, span.p0)}
          : std::array<Vec2d, 3>{Subtract(span.p2, span.p3),
                                 Subtract(span.p1, span.p3),
                                 Subtract(span.p0, span.p3)};
  for (const Vec2d candidate : candidates) {
    const Vec2d tangent = Normalize(candidate);
    if (Length(tangent) > kStationEpsilon) {
      return Result<Vec2d>::Ok(tangent);
    }
  }
  return Result<Vec2d>::Fail(ErrorKind::kUnsupported,
                             "road approach tangent is degenerate");
}

Result<Vec2d> TangentAt(const Path& alignment, double station_m) {
  const Result<double> length = PathLength(alignment);
  if (!length.ok) return Result<Vec2d>::Fail(length.error_kind, length.error);
  const double delta = std::min(0.1, length.value);
  const double before_station = std::max(0.0, station_m - delta);
  const double after_station = std::min(length.value, station_m + delta);
  if (after_station - before_station <= kStationEpsilon) {
    return Result<Vec2d>::Fail(ErrorKind::kInternal, "road tangent station is degenerate");
  }
  const Result<Vec2d> before = EvaluatePath(alignment, before_station);
  const Result<Vec2d> after = EvaluatePath(alignment, after_station);
  if (!before.ok || !after.ok) {
    return Result<Vec2d>::Fail(ErrorKind::kInternal, "road tangent could not be evaluated");
  }
  return Result<Vec2d>::Ok(Normalize(Subtract(after.value, before.value)));
}

double StationForEndpoint(const ApproachKey& key, double length_m) {
  return key.endpoint_role == EndpointRole::kStart ? 0.0 : length_m;
}

void SortUniqueStations(std::vector<double>& stations) {
  std::sort(stations.begin(), stations.end());
  stations.erase(
      std::unique(stations.begin(), stations.end(), [](double a, double b) {
        return std::abs(a - b) <= kStationEpsilon;
      }),
      stations.end());
}

} // namespace city::road::build
