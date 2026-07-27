#pragma once

#include "city/road/road.hpp"

namespace city::road::internal {

// Cross section evaluated from the authoritative template and, when the segment
// carries one, its transition. Marking requests are merged here so every
// consumer reads one effective policy per boundary.
[[nodiscard]] Result<CrossSectionTemplate> template_at(const SavedRoadGraph &graph,
                                                       const RoadSegment &segment,
                                                       double station_m,
                                                       double total_m);
[[nodiscard]] Result<SectionEvaluation> section_at(const SavedRoadGraph &graph,
                                                   const RoadSegment &segment,
                                                   double station_m,
                                                   double total_m);

} // namespace city::road::internal
