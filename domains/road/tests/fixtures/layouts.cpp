#include "layouts.hpp"

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

RoadLayoutTemplate BidirectionalLayout(RoadLayoutTemplateId id) {
  RoadLayoutTemplate layout{};
  layout.id = id;
  layout.strips = {
      {10, StripFunction::kSidewalk, 2.0, 0.01, builtin_surface_styles::kSidewalk},
      {20, StripFunction::kCarriageway, 3.0, 0.02, builtin_surface_styles::kAsphalt},
      {30, StripFunction::kCarriageway, 3.0, -0.02, builtin_surface_styles::kAsphalt},
      {40, StripFunction::kSidewalk, 2.0, -0.01, builtin_surface_styles::kSidewalk},
  };
  layout.lane_bands = {
      {1000, 20, 0.0, 3.0, LaneTravelDirection::kAgainstSegment},
      {1010, 30, 0.0, 3.0, LaneTravelDirection::kAlongSegment},
  };
  layout.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, outer_line},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, center_line},
      {300, BoundaryRole::kCurb, 0.2, 0.15, outer_line},
  };
  layout.alignment_offset_from_left_m = CentredAlignmentOffset(layout);
  return layout;
}

RoadLayoutTemplate ExtraLaneLayout(RoadLayoutTemplateId id) {
  RoadLayoutTemplate layout = BidirectionalLayout(id);
  layout.strips = {
      {10, StripFunction::kSidewalk, 2.0, 0.01, builtin_surface_styles::kSidewalk},
      {20, StripFunction::kCarriageway, 3.0, 0.02, builtin_surface_styles::kAsphalt},
      {30, StripFunction::kCarriageway, 3.0, 0.0, builtin_surface_styles::kAsphalt},
      {35, StripFunction::kCarriageway, 3.0, -0.02, builtin_surface_styles::kAsphalt},
      {40, StripFunction::kSidewalk, 2.0, -0.01, builtin_surface_styles::kSidewalk},
  };
  layout.lane_bands = {
      {1000, 20, 0.0, 3.0, LaneTravelDirection::kAgainstSegment},
      {1010, 30, 0.0, 3.0, LaneTravelDirection::kAlongSegment},
      {1020, 35, 0.0, 3.0, LaneTravelDirection::kAlongSegment},
  };
  layout.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, outer_line},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, center_line},
      {250, BoundaryRole::kLaneDivider, 0.0, 0.0, center_line},
      {300, BoundaryRole::kCurb, 0.2, 0.15, outer_line},
  };
  layout.alignment_offset_from_left_m = CentredAlignmentOffset(layout);
  return layout;
}

RoadLayoutTemplate AsymmetricLayout(RoadLayoutTemplateId id) {
  RoadLayoutTemplate layout = BidirectionalLayout(id);
  layout.strips.erase(layout.strips.begin());
  layout.boundaries.erase(layout.boundaries.begin());
  layout.alignment_offset_from_left_m = CentredAlignmentOffset(layout);
  return layout;
}

RoadLayoutTemplate MedianLayout(RoadLayoutTemplateId id) {
  RoadLayoutTemplate layout = BidirectionalLayout(id);
  layout.strips.insert(layout.strips.begin() + 2,
                        {25, StripFunction::kMedian, 2.0, 0.0,
                         builtin_surface_styles::kMedian});
  layout.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, outer_line},
      {210, BoundaryRole::kMedianEdge, 0.2, 0.12, {}},
      {220, BoundaryRole::kMedianEdge, 0.2, -0.12, {}},
      {300, BoundaryRole::kCurb, 0.2, 0.15, outer_line},
  };
  layout.alignment_offset_from_left_m = CentredAlignmentOffset(layout);
  return layout;
}

RoadLayoutTemplate ShoulderedLayout(RoadLayoutTemplateId id) {
  RoadLayoutTemplate layout{};
  layout.id = id;
  layout.strips = {
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
  layout.lane_bands = {
      {1000, 20, 0.0, 3.0, LaneTravelDirection::kAgainstSegment},
      {1010, 30, 0.0, 3.0, LaneTravelDirection::kAlongSegment},
  };
  layout.boundaries = {
      {100, BoundaryRole::kCurb, 0.2, -0.15, {}},
      {150, BoundaryRole::kOuterEdge, 0.0, 0.0, outer_line},
      {200, BoundaryRole::kLaneDivider, 0.0, 0.0, center_line},
      {250, BoundaryRole::kOuterEdge, 0.0, 0.0, outer_line},
      {300, BoundaryRole::kCurb, 0.2, 0.15, {}},
  };
  layout.alignment_offset_from_left_m = CentredAlignmentOffset(layout);
  return layout;
}

double CentredAlignmentOffset(const RoadLayoutTemplate& layout) {
  double total_width = 0.0;
  for (const auto& strip : layout.strips) total_width += strip.width_m;
  for (const auto& boundary : layout.boundaries) total_width += boundary.width_m;
  return total_width * 0.5;
}

RoadLayoutTemplateId AddLayout(RoadState& state,
                                  RoadLayoutTemplate layout) {
  const auto added = state.AddRoadLayoutTemplate(
      city::road::AddRoadLayoutTemplateRequest{std::move(layout)});
  return added.ok ? added.value : 0;
}

} // namespace road_fixture
