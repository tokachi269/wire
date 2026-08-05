#pragma once

#include "city/road/road.hpp"

namespace city::road::internal {

[[nodiscard]] const RoadNode *find_node(const SavedRoadGraph &graph,
                                        RoadNodeId id);
[[nodiscard]] const RoadSegment *find_segment(const SavedRoadGraph &graph,
                                              RoadSegmentId id);
[[nodiscard]] const CrossSectionTemplate *
find_template(const SavedRoadGraph &graph, CrossSectionTemplateId id);
[[nodiscard]] const SectionTransition *
find_transition(const SavedRoadGraph &graph, SectionTransitionId id);
[[nodiscard]] std::size_t node_degree(const SavedRoadGraph &graph,
                                     RoadNodeId id);
[[nodiscard]] const NodeConnectionPolicyOverride *
find_policy_override(const SavedRoadGraph &graph, RoadNodeId node_id);
[[nodiscard]] const ApproachGeometryOverride *
find_approach_override(const SavedRoadGraph &graph, const ApproachKey &key);

struct LaneEndpointLookup {
  const RoadSegment *segment = nullptr;
  const CrossSectionTemplate *section = nullptr;
  const LaneBand *lane = nullptr;
  RoadNodeId node_id = 0;
};

struct BoundaryEndpointLookup {
  const RoadSegment *segment = nullptr;
  const CrossSectionTemplate *section = nullptr;
  const BoundaryProfile *boundary = nullptr;
  RoadNodeId node_id = 0;
};

[[nodiscard]] const CrossSectionTemplate *
find_endpoint_template(const SavedRoadGraph &graph, const RoadSegment &segment,
                       EndpointRole endpoint_role);
[[nodiscard]] LaneEndpointLookup
find_lane_endpoint(const SavedRoadGraph &graph, const LaneEndpointKey &key);
[[nodiscard]] BoundaryEndpointLookup find_boundary_endpoint(
    const SavedRoadGraph &graph, const BoundaryEndpointKey &key);

} // namespace city::road::internal
