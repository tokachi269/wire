#pragma once

#include "build_context.hpp"

namespace city::road::build {

[[nodiscard]] Result<bool> BuildTopologyIndex(BuildContext& context);
[[nodiscard]] Result<bool> BuildCanonicalAlignments(BuildContext& context);
[[nodiscard]] Result<bool> BuildNodeConnectionDecisions(BuildContext& context);
[[nodiscard]] Result<bool> BuildAutoNodeLayouts(BuildContext& context);
[[nodiscard]] Result<bool> BuildResolvedNodeLayouts(BuildContext& context);
[[nodiscard]] Result<bool> BuildSamplingPlans(BuildContext& context);
[[nodiscard]] Result<bool> BuildSectionEvaluations(BuildContext& context);
[[nodiscard]] Result<bool> BuildConnectionGates(BuildContext& context);
[[nodiscard]] Result<bool> BuildJunctionGeometries(BuildContext& context);
[[nodiscard]] Result<bool> BuildMarkingAnchors(BuildContext& context);
[[nodiscard]] Result<DerivedRoad> BuildRoad(const SavedRoadGraph& authoritative);

[[nodiscard]] Result<SectionEvaluation> ResolveSectionAt(const SavedRoadGraph& graph,
                                                         const RoadSegment& segment,
                                                         double station_m,
                                                         double total_m);
[[nodiscard]] Result<CrossSectionTemplate> ResolveTemplateAt(const SavedRoadGraph& graph,
                                                             const RoadSegment& segment,
                                                             double station_m,
                                                             double total_m);

} // namespace city::road::build
