#pragma once

#include "city/road/road.hpp"

namespace city::road::internal {

// Cross section evaluated from the authoritative template and, when the segment
// carries one, its transition. Marking requests are merged here so every
// consumer reads one effective policy per boundary.
[[nodiscard]] Result<CrossSectionTemplate> template_at(const SavedRoadGraph &graph,
                                                       const RoadSegment &segment,
                                                       double segment_distance_m,
                                                       double total_m);
[[nodiscard]] Result<SectionEvaluation> section_at(const SavedRoadGraph &graph,
                                                   const RoadSegment &segment,
                                                   double segment_distance_m,
                                                   double total_m);

struct LaneSectionPosition {
  double lateral_m = 0.0;
  double height_m = 0.0;
};

[[nodiscard]] Result<LaneSectionPosition>
lane_position(const CrossSectionTemplate &section, const LaneBand &lane,
              const SectionEvaluation &evaluation);

} // namespace city::road::internal
