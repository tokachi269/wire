#pragma once

#include "pipeline.hpp"

namespace city::road::build {

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
[[nodiscard]] const Path *find_alignment(const DerivedRoad &out,
                                         RoadSegmentId segment_id);
[[nodiscard]] const ApproachConnectionDecision *
find_approach_connection(const NodeConnectionDecision &connection,
                         const ApproachKey &key);
[[nodiscard]] const ResolvedNodeLayout *find_layout(const DerivedRoad &out,
                                                    RoadNodeId node_id);
[[nodiscard]] const ResolvedApproachLayout *
find_approach_layout(const ResolvedNodeLayout &layout, const ApproachKey &key);
[[nodiscard]] const SegmentSamplingPlan *
find_sampling(const DerivedRoad &out, RoadSegmentId segment_id);
[[nodiscard]] const SectionEvaluation *find_section(const DerivedRoad &out,
                                                    RoadSegmentId segment_id,
                                                    double station_m);

} // namespace city::road::build