#pragma once

#include "city/road/road.hpp"

// Road layouts the tests draw with, named by the structure each one provides.
// Core ships no product catalogue, so a test registers the layouts it needs and
// uses the IDs AddRoadLayoutTemplate returns. Each builder takes the ID a test
// wants, or 0 to let Core assign one.
//
// The measurements are the tests' own. They are chosen so a layout has a
// non-zero cross slope, a structural curb and a lane divider to assert against.
// They are not the product catalogue, which lives in web/src/road_templates.ts.
namespace road_fixture {

using city::road::RoadLayoutTemplate;
using city::road::RoadLayoutTemplateId;
using city::road::RoadState;

// Two lanes running opposite ways, a walkway and curb on each side, one divider.
[[nodiscard]] RoadLayoutTemplate BidirectionalLayout(RoadLayoutTemplateId id);
// Bidirectional plus one more lane in the along-segment direction.
[[nodiscard]] RoadLayoutTemplate ExtraLaneLayout(RoadLayoutTemplateId id);
// One side has no walkway, so the outermost strip is a lane.
[[nodiscard]] RoadLayoutTemplate AsymmetricLayout(RoadLayoutTemplateId id);
// A median strip between the two directions, bounded by median edges.
[[nodiscard]] RoadLayoutTemplate MedianLayout(RoadLayoutTemplateId id);
// Shoulders between the lanes and the walkways, with their own outer edges.
[[nodiscard]] RoadLayoutTemplate ShoulderedLayout(RoadLayoutTemplateId id);
// Bidirectional, but the edge structure is an L-shaped gutter: a top face in
// the walkway, a vertical face on the datum, then a channel and a lip reaching
// back into the roadway. Neither declared width grows to hold it.
[[nodiscard]] RoadLayoutTemplate GutteredLayout(RoadLayoutTemplateId id);

// Puts the alignment on the middle of the layout's own total width. The layouts
// above use it because they are symmetric; a test that wants the alignment
// somewhere else sets `alignment_offset_from_left_m` itself.
[[nodiscard]] double CentredAlignmentOffset(const RoadLayoutTemplate& layout);

// Boundary profiles the layouts above are built from, for tests that assemble
// their own sections. `outward_m` is where a curb's top face reaches: negative
// puts it on the section's left.
[[nodiscard]] city::road::BoundaryProfile CurbBoundary(
    city::road::BoundaryId id, double outward_m, double height_m,
    city::road::AutoMarkingPolicy marking);
[[nodiscard]] city::road::BoundaryProfile PaintedLineBoundary(
    city::road::BoundaryId id, city::road::BoundaryRole role,
    city::road::AutoMarkingPolicy marking);

// Registers one layout and reports the ID Core assigned, or 0 on failure.
[[nodiscard]] RoadLayoutTemplateId AddLayout(RoadState& state,
                                             RoadLayoutTemplate layout);

} // namespace road_fixture
