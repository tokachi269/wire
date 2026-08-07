#include "layouts.hpp"

#include <algorithm>
#include <utility>
#include <vector>

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

// `outward_m` is where the top face reaches: negative on the section's left.
[[nodiscard]] city::road::BoundaryProfile Curb(city::road::BoundaryId id,
                                               double outward_m, double height_m,
                                               AutoMarkingPolicy marking);
[[nodiscard]] city::road::BoundaryProfile PaintedLine(city::road::BoundaryId id,
                                                      BoundaryRole role,
                                                      AutoMarkingPolicy marking);

city::road::BoundaryProfile Curb(city::road::BoundaryId id, double outward_m,
                                 double height_m, AutoMarkingPolicy marking) {
  city::road::BoundaryProfile boundary{};
  boundary.boundary_id = id;
  boundary.role = BoundaryRole::kCurb;
  boundary.marking = marking;
  if (outward_m < 0.0) {
    boundary.contour = {{outward_m, height_m}, {0.0, 0.0}};
  } else {
    boundary.contour = {{0.0, 0.0}, {outward_m, height_m}};
  }
  boundary.segment_styles = {builtin_surface_styles::kCurb};
  return boundary;
}

[[nodiscard]] city::road::BoundaryProfile MedianEdge(city::road::BoundaryId id,
                                                     double outward_m,
                                                     double height_m) {
  city::road::BoundaryProfile boundary = Curb(id, outward_m, height_m, {});
  boundary.role = BoundaryRole::kMedianEdge;
  boundary.segment_styles = {builtin_surface_styles::kMedian};
  return boundary;
}

[[nodiscard]] city::road::BoundaryProfile Gutter(city::road::BoundaryId id,
                                                 bool left_side,
                                                 AutoMarkingPolicy marking) {
  city::road::BoundaryProfile boundary{};
  boundary.boundary_id = id;
  boundary.role = BoundaryRole::kCurb;
  boundary.marking = marking;
  const double sign = left_side ? 1.0 : -1.0;
  std::vector<city::road::ProfilePoint> outward{
      {-0.1 * sign, 0.0}, {0.0, 0.0}, {0.0, -0.1},
      {0.25 * sign, -0.075}, {0.35 * sign, -0.07}};
  if (!left_side) std::reverse(outward.begin(), outward.end());
  boundary.contour = std::move(outward);
  boundary.segment_styles.assign(boundary.contour.size() - 1,
                                 builtin_surface_styles::kCurb);
  return boundary;
}

city::road::BoundaryProfile PaintedLine(city::road::BoundaryId id,
                                        BoundaryRole role,
                                        AutoMarkingPolicy marking) {
  city::road::BoundaryProfile boundary{};
  boundary.boundary_id = id;
  boundary.role = role;
  boundary.marking = marking;
  boundary.contour = {{0.0, 0.0}};
  return boundary;
}

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
      Curb(100, -0.2, 0.15, outer_line),
      PaintedLine(200, BoundaryRole::kLaneDivider, center_line),
      Curb(300, 0.2, 0.15, outer_line),
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
      Curb(100, -0.2, 0.15, outer_line),
      PaintedLine(200, BoundaryRole::kLaneDivider, center_line),
      PaintedLine(250, BoundaryRole::kLaneDivider, center_line),
      Curb(300, 0.2, 0.15, outer_line),
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
      Curb(100, -0.2, 0.15, outer_line),
      MedianEdge(210, 0.2, 0.12),
      MedianEdge(220, -0.2, 0.12),
      Curb(300, 0.2, 0.15, outer_line),
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
      Curb(100, -0.2, 0.15, {}),
      PaintedLine(150, BoundaryRole::kOuterEdge, outer_line),
      PaintedLine(200, BoundaryRole::kLaneDivider, center_line),
      PaintedLine(250, BoundaryRole::kOuterEdge, outer_line),
      Curb(300, 0.2, 0.15, {}),
  };
  layout.alignment_offset_from_left_m = CentredAlignmentOffset(layout);
  return layout;
}

RoadLayoutTemplate GutteredLayout(RoadLayoutTemplateId id) {
  RoadLayoutTemplate layout = BidirectionalLayout(id);
  layout.boundaries = {
      Gutter(100, true, outer_line),
      PaintedLine(200, BoundaryRole::kLaneDivider, center_line),
      Gutter(300, false, outer_line),
  };
  layout.alignment_offset_from_left_m = CentredAlignmentOffset(layout);
  return layout;
}

double CentredAlignmentOffset(const RoadLayoutTemplate& layout) {
  double total_width = 0.0;
  for (const auto& strip : layout.strips) total_width += strip.width_m;
  return total_width * 0.5;
}

city::road::BoundaryProfile CurbBoundary(city::road::BoundaryId id,
                                         double outward_m, double height_m,
                                         city::road::AutoMarkingPolicy marking) {
  return Curb(id, outward_m, height_m, marking);
}

city::road::BoundaryProfile PaintedLineBoundary(
    city::road::BoundaryId id, city::road::BoundaryRole role,
    city::road::AutoMarkingPolicy marking) {
  return PaintedLine(id, role, marking);
}

RoadLayoutTemplateId AddLayout(RoadState& state,
                                  RoadLayoutTemplate layout) {
  const auto added = state.AddRoadLayoutTemplate(
      city::road::AddRoadLayoutTemplateRequest{std::move(layout)});
  return added.ok ? added.value : 0;
}

} // namespace road_fixture
