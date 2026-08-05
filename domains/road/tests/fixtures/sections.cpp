#include "sections.hpp"

namespace road_fixture {
namespace {

using city::road::AutoMarkingPolicy;
using city::road::BoundaryRole;
using city::road::LaneTravelDirection;
using city::road::MarkingRole;
using city::road::StripFunction;
namespace builtin_marking_styles = city::road::builtin_marking_styles;
namespace builtin_surface_styles = city::road::builtin_surface_styles;

const AutoMarkingPolicy outer_line{
    true, MarkingRole::kCarriagewayEdge, builtin_marking_styles::kWhiteSolid};
const AutoMarkingPolicy center_line{
    true, MarkingRole::kCenterLine, builtin_marking_styles::kCenterLine};

} // namespace

CrossSectionTemplate UrbanTwoLane(CrossSectionTemplateId id) {
  CrossSectionTemplate section{};
  section.id = id;
  section.strips = {
      {10, StripFunction::kSidewalk, 2.0, 0.01, builtin_surface_styles::kSidewalk},
      {20, StripFunction::kCarriageway, 3.0, 0.02, builtin_surface_styles::kAsphalt},
      {30, StripFunction::kCarriageway, 3.0, -0.02, builtin_surface_styles::kAsphalt},
      {40, StripFunction::kSidewalk, 2.0, -0.01, builtin_surface_styles::kSidewalk},
  };
  section.lane_bands = {
      {1000, 20, 0.0, 3.0, LaneTravelDirection::kAgainstSegment},
      {1010, 30, 0.0, 3.0, LaneTravelDirection::kAlongSegment},
  };
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, outer_line},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, center_line},
      {300, BoundaryRole::kCurb, 0.2, 0.15, outer_line},
  };
  return section;
}

CrossSectionTemplate ThreeLane(CrossSectionTemplateId id) {
  CrossSectionTemplate section = UrbanTwoLane(id);
  section.strips = {
      {10, StripFunction::kSidewalk, 2.0, 0.01, builtin_surface_styles::kSidewalk},
      {20, StripFunction::kCarriageway, 3.0, 0.02, builtin_surface_styles::kAsphalt},
      {30, StripFunction::kCarriageway, 3.0, 0.0, builtin_surface_styles::kAsphalt},
      {35, StripFunction::kCarriageway, 3.0, -0.02, builtin_surface_styles::kAsphalt},
      {40, StripFunction::kSidewalk, 2.0, -0.01, builtin_surface_styles::kSidewalk},
  };
  section.lane_bands = {
      {1000, 20, 0.0, 3.0, LaneTravelDirection::kAgainstSegment},
      {1010, 30, 0.0, 3.0, LaneTravelDirection::kAlongSegment},
      {1020, 35, 0.0, 3.0, LaneTravelDirection::kAlongSegment},
  };
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, outer_line},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, center_line},
      {250, BoundaryRole::kLaneDivider, 0.0, 0.0, center_line},
      {300, BoundaryRole::kCurb, 0.2, 0.15, outer_line},
  };
  return section;
}

CrossSectionTemplate NoLeftSidewalk(CrossSectionTemplateId id) {
  CrossSectionTemplate section = UrbanTwoLane(id);
  section.strips.erase(section.strips.begin());
  section.boundaries.erase(section.boundaries.begin());
  return section;
}

CrossSectionTemplate MedianTwoLane(CrossSectionTemplateId id) {
  CrossSectionTemplate section = UrbanTwoLane(id);
  section.strips.insert(section.strips.begin() + 2,
                        {25, StripFunction::kMedian, 2.0, 0.0,
                         builtin_surface_styles::kMedian});
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, outer_line},
      {210, BoundaryRole::kMedianEdge, 0.2, 0.12, {}},
      {220, BoundaryRole::kMedianEdge, 0.2, -0.12, {}},
      {300, BoundaryRole::kCurb, 0.2, 0.15, outer_line},
  };
  return section;
}

CrossSectionTemplate ShoulderedTwoLane(CrossSectionTemplateId id) {
  CrossSectionTemplate section{};
  section.id = id;
  section.strips = {
      {10, StripFunction::kSidewalk, 2.0, 0.01,
       builtin_surface_styles::kSidewalk},
      {15, StripFunction::kShoulder, 0.75, 0.02,
       builtin_surface_styles::kAsphalt},
      {20, StripFunction::kCarriageway, 3.0, 0.02,
       builtin_surface_styles::kAsphalt},
      {30, StripFunction::kCarriageway, 3.0, -0.02,
       builtin_surface_styles::kAsphalt},
      {35, StripFunction::kShoulder, 0.75, -0.02,
       builtin_surface_styles::kAsphalt},
      {40, StripFunction::kSidewalk, 2.0, -0.01,
       builtin_surface_styles::kSidewalk},
  };
  section.lane_bands = {
      {1000, 20, 0.0, 3.0, LaneTravelDirection::kAgainstSegment},
      {1010, 30, 0.0, 3.0, LaneTravelDirection::kAlongSegment},
  };
  section.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, {}},
      {150, BoundaryRole::kOuterEdge, 0.0, 0.0, outer_line},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, center_line},
      {250, BoundaryRole::kOuterEdge, 0.0, 0.0, outer_line},
      {300, BoundaryRole::kCurb, 0.2, 0.15, {}},
  };
  return section;
}

CrossSectionTemplateId AddSection(RoadState& state,
                                  CrossSectionTemplate section) {
  const auto added = state.AddSectionTemplate(
      city::road::AddSectionTemplateRequest{std::move(section)});
  return added.ok ? added.value : 0;
}

} // namespace road_fixture
