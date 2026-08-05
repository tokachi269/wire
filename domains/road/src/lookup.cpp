#include "lookup.hpp"

#include "geometry/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace city::road::internal {

const RoadNode *find_node(const SavedRoadGraph &graph, RoadNodeId id) {
  const auto found =
      std::find_if(graph.nodes.begin(), graph.nodes.end(),
                   [id](const RoadNode &node) { return node.id == id; });
  return found == graph.nodes.end() ? nullptr : &*found;
}

const RoadSegment *find_segment(const SavedRoadGraph &graph, RoadSegmentId id) {
  const auto found = std::find_if(
      graph.segments.begin(), graph.segments.end(),
      [id](const RoadSegment &segment) { return segment.id == id; });
  return found == graph.segments.end() ? nullptr : &*found;
}

const RoadLayoutTemplate *find_template(const SavedRoadGraph &graph,
                                          RoadLayoutTemplateId id) {
  const auto found = std::find_if(
      graph.layout_templates.begin(), graph.layout_templates.end(),
      [id](const RoadLayoutTemplate &section) { return section.id == id; });
  return found == graph.layout_templates.end() ? nullptr : &*found;
}

const RoadLayoutTransition *find_transition(const SavedRoadGraph &graph,
                                         RoadLayoutTransitionId id) {
  const auto found =
      std::find_if(graph.transitions.begin(), graph.transitions.end(),
                   [id](const RoadLayoutTransition &transition) {
                     return transition.id == id;
                   });
  return found == graph.transitions.end() ? nullptr : &*found;
}

const NodeConnectionPolicyOverride *
find_policy_override(const SavedRoadGraph &graph, RoadNodeId node_id) {
  const auto found =
      std::find_if(graph.connection_policy_overrides.begin(),
                   graph.connection_policy_overrides.end(),
                   [node_id](const NodeConnectionPolicyOverride &policy) {
                     return policy.node_id == node_id;
                   });
  return found == graph.connection_policy_overrides.end() ? nullptr : &*found;
}

const ApproachGeometryOverride *
find_approach_override(const SavedRoadGraph &graph, const ApproachKey &key) {
  const auto found =
      std::find_if(graph.approach_geometry_overrides.begin(),
                   graph.approach_geometry_overrides.end(),
                   [&key](const ApproachGeometryOverride &value) {
                     return value.key == key;
                   });
  return found == graph.approach_geometry_overrides.end() ? nullptr : &*found;
}

const RoadLayoutTemplate *
find_endpoint_template(const SavedRoadGraph &graph, const RoadSegment &segment,
                       EndpointRole endpoint_role) {
  if (!segment.transition.has_value()) {
    return find_template(graph, segment.layout_template);
  }
  const RoadLayoutTransition *transition =
      find_transition(graph, *segment.transition);
  if (transition == nullptr) {
    return nullptr;
  }
  return find_template(graph, endpoint_role == EndpointRole::kStart
                                  ? transition->from_template
                                  : transition->to_template);
}

LaneEndpointLookup find_lane_endpoint(const SavedRoadGraph &graph,
                                      const LaneEndpointKey &key) {
  LaneEndpointLookup result{};
  result.segment = find_segment(graph, key.segment_id);
  if (result.segment == nullptr) {
    return result;
  }
  result.node_id = key.endpoint_role == EndpointRole::kStart
                       ? result.segment->node_a
                       : result.segment->node_b;
  result.section = find_endpoint_template(graph, *result.segment,
                                          key.endpoint_role);
  if (result.section == nullptr) {
    return result;
  }
  const auto lane = std::find_if(
      result.section->lane_bands.begin(), result.section->lane_bands.end(),
      [&key](const LaneBand &candidate) { return candidate.id == key.lane_id; });
  result.lane = lane == result.section->lane_bands.end() ? nullptr : &*lane;
  return result;
}

BoundaryEndpointLookup
find_boundary_endpoint(const SavedRoadGraph &graph,
                       const BoundaryEndpointKey &key) {
  BoundaryEndpointLookup result{};
  result.segment = find_segment(graph, key.segment_id);
  if (result.segment == nullptr) {
    return result;
  }
  result.node_id = key.endpoint_role == EndpointRole::kStart
                       ? result.segment->node_a
                       : result.segment->node_b;
  result.section = find_endpoint_template(graph, *result.segment,
                                          key.endpoint_role);
  if (result.section == nullptr) {
    return result;
  }
  const auto boundary = std::find_if(
      result.section->boundaries.begin(), result.section->boundaries.end(),
      [&key](const BoundaryProfile &candidate) {
        return candidate.boundary_id == key.boundary_id;
      });
  result.boundary =
      boundary == result.section->boundaries.end() ? nullptr : &*boundary;
  return result;
}

std::size_t node_degree(const SavedRoadGraph &graph, RoadNodeId id) {
  return static_cast<std::size_t>(std::count_if(graph.segments.begin(), graph.segments.end(), [id](const RoadSegment& segment) {
    return segment.node_a == id || segment.node_b == id;
  }));
}
} // namespace city::road::internal

namespace city::road {

using internal::distance_epsilon;
using internal::is_finite;

const DerivedSegment *FindDerivedSegment(const DerivedRoad &derived,
                                         RoadSegmentId segment_id) {
  const auto found = std::find_if(derived.segments.begin(),
                                  derived.segments.end(),
                                  [segment_id](const DerivedSegment &segment) {
                                    return segment.id == segment_id;
                                  });
  return found == derived.segments.end() ? nullptr : &*found;
}

const ResolvedConnection *FindResolvedConnection(const DerivedRoad &derived,
                                                 RoadNodeId node_id) {
  const auto found =
      std::find_if(derived.connections.begin(), derived.connections.end(),
                   [node_id](const ResolvedConnection &connection) {
                     return connection.node_id == node_id;
                   });
  return found == derived.connections.end() ? nullptr : &*found;
}

const ResolvedApproach *FindResolvedApproach(const DerivedRoad &derived,
                                             const ApproachKey &key) {
  for (const ResolvedConnection &connection : derived.connections) {
    for (const ResolvedApproach &approach : connection.approaches) {
      if (approach.key == key)
        return &approach;
    }
  }
  return nullptr;
}

const SectionEvaluation *FindSectionAt(const DerivedSegment &segment,
                                       double segment_distance_m) {
  const SectionEvaluation *match = nullptr;
  for (const SectionEvaluation &section : segment.sections) {
    if (std::abs(section.segment_distance_m - segment_distance_m) >
        internal::distance_epsilon)
      continue;
    if (match != nullptr)
      return nullptr;
    match = &section;
  }
  return match;
}

const Path *FindCanonicalAlignment(const DerivedRoad &derived,
                                   RoadSegmentId segment_id) {
  const DerivedSegment *segment = FindDerivedSegment(derived, segment_id);
  return segment == nullptr ? nullptr : &segment->alignment;
}

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
  if (!is_finite(distance.corridor_distance_m) ||
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
    if (segment == nullptr || !is_finite(segment->length_m) ||
        segment->length_m <= 0.0) {
      return Result<ResolvedSegmentDistance>::Fail(
          CommitFailureCategory::kInternalError, "road corridor segment length is missing");
    }
    const double end = accumulated + segment->length_m;
    const bool last = index + 1 == corridor->segments.size();
    if (distance.corridor_distance_m < end ||
        (last && distance.corridor_distance_m <= end + distance_epsilon)) {
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

} // namespace city::road
