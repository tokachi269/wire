#pragma once

#include "city/road/road.hpp"

namespace city::road::internal {

[[nodiscard]] const RoadNode *find_node(const SavedRoadGraph &graph,
                                        RoadNodeId id);
[[nodiscard]] const RoadSegment *find_segment(const SavedRoadGraph &graph,
                                              RoadSegmentId id);
[[nodiscard]] const RoadLayoutTemplate *
find_template(const SavedRoadGraph &graph, RoadLayoutTemplateId id);
[[nodiscard]] const RoadLayoutTransition *
find_transition(const SavedRoadGraph &graph, RoadLayoutTransitionId id);
[[nodiscard]] std::size_t node_degree(const SavedRoadGraph &graph,
                                     RoadNodeId id);
[[nodiscard]] const NodeConnectionPolicyOverride *
find_policy_override(const SavedRoadGraph &graph, RoadNodeId node_id);
[[nodiscard]] const ApproachGeometryOverride *
find_approach_override(const SavedRoadGraph &graph, const ApproachKey &key);

struct LaneEndpointLookup {
  const RoadSegment *segment = nullptr;
  const RoadLayoutTemplate *section = nullptr;
  const LaneBand *lane = nullptr;
  RoadNodeId node_id = 0;
};

struct BoundaryEndpointLookup {
  const RoadSegment *segment = nullptr;
  const RoadLayoutTemplate *section = nullptr;
  const BoundaryProfile *boundary = nullptr;
  RoadNodeId node_id = 0;
};

[[nodiscard]] const RoadLayoutTemplate *
find_endpoint_template(const SavedRoadGraph &graph, const RoadSegment &segment,
                       EndpointRole endpoint_role);
[[nodiscard]] LaneEndpointLookup
find_lane_endpoint(const SavedRoadGraph &graph, const LaneEndpointKey &key);
[[nodiscard]] BoundaryEndpointLookup find_boundary_endpoint(
    const SavedRoadGraph &graph, const BoundaryEndpointKey &key);

} // namespace city::road::internal
