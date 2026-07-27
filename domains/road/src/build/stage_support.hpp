#pragma once

#include "build_context.hpp"

#include <optional>

namespace city::road::build {

inline constexpr double kStationEpsilon = 1e-9;
inline constexpr double kSurfaceSampleStepM = 2.0;

[[nodiscard]] bool IsFinite(double value);
[[nodiscard]] bool IsFinite(Vec2d value);
[[nodiscard]] Vec2d Add(Vec2d a, Vec2d b);
[[nodiscard]] Vec2d Subtract(Vec2d a, Vec2d b);
[[nodiscard]] Vec2d Scale(Vec2d value, double scale);
[[nodiscard]] double Dot(Vec2d a, Vec2d b);
[[nodiscard]] double Cross(Vec2d a, Vec2d b);
[[nodiscard]] double Length(Vec2d value);
[[nodiscard]] Vec2d Normalize(Vec2d value);
[[nodiscard]] Vec3d To3(Vec2d value, double z = 0.0);

[[nodiscard]] const RoadNode* FindNode(const SavedRoadGraph& graph, RoadNodeId id);
[[nodiscard]] const RoadSegment* FindSegment(const SavedRoadGraph& graph, RoadSegmentId id);
[[nodiscard]] const CrossSectionTemplate* FindTemplate(const SavedRoadGraph& graph, CrossSectionTemplateId id);
[[nodiscard]] const SectionTransition* FindTransition(const SavedRoadGraph& graph, SectionTransitionId id);
[[nodiscard]] const NodeConnectionPolicyOverride* FindPolicyOverride(const SavedRoadGraph& graph,
                                                                     RoadNodeId node_id);
[[nodiscard]] const ApproachGeometryOverride* FindApproachGeometryOverride(const SavedRoadGraph& graph,
                                                                           const ApproachKey& key);
[[nodiscard]] const Path* FindAlignment(const DerivedRoad& derived, RoadSegmentId segment_id);
[[nodiscard]] const NodeConnectionDecision* FindDecision(const DerivedRoad& derived, RoadNodeId node_id);
[[nodiscard]] const ApproachConnectionDecision* FindApproachDecision(const NodeConnectionDecision& decision,
                                                                     const ApproachKey& key);
[[nodiscard]] const AutoNodeLayout* FindAutoNodeLayout(const DerivedRoad& derived, RoadNodeId node_id);
[[nodiscard]] const AutoApproachLayout* FindAutoApproachLayout(const AutoNodeLayout& layout,
                                                               const ApproachKey& key);
[[nodiscard]] const ResolvedNodeLayout* FindResolvedNodeLayout(const DerivedRoad& derived, RoadNodeId node_id);
[[nodiscard]] const ResolvedApproachLayout* FindResolvedApproachLayout(const ResolvedNodeLayout& layout,
                                                                       const ApproachKey& key);
[[nodiscard]] const SegmentSamplingPlan* FindSamplingPlan(const DerivedRoad& derived,
                                                          RoadSegmentId segment_id);
[[nodiscard]] const SectionEvaluation* FindSectionEvaluation(const DerivedRoad& derived,
                                                             RoadSegmentId segment_id,
                                                             double station_m);

[[nodiscard]] ApproachKey MakeApproachKey(const RoadSegment& segment, RoadNodeId node_id);
[[nodiscard]] Result<Vec2d> InwardTangent(const RoadSegment& segment,
                                         const Path& alignment,
                                         const ApproachKey& key);
[[nodiscard]] Result<Vec2d> TangentAt(const Path& alignment, double station_m);
[[nodiscard]] double StationForEndpoint(const ApproachKey& key, double length_m);
void SortUniqueStations(std::vector<double>& stations);

} // namespace city::road::build
