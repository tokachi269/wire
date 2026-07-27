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
[[nodiscard]] const NodeConnectionPolicyOverride *
find_policy_override(const SavedRoadGraph &graph, RoadNodeId node_id);
[[nodiscard]] const ApproachGeometryOverride *
find_approach_override(const SavedRoadGraph &graph, const ApproachKey &key);

} // namespace city::road::internal
