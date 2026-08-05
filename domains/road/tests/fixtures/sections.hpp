#pragma once

#include "city/road/road.hpp"

// Road layouts the tests draw with. Core ships no product presets, so a
// test registers the layouts it needs and uses the IDs AddRoadLayoutTemplate
// returns. Each builder takes the ID a test wants, or 0 to let Core assign one.
namespace road_fixture {

using city::road::RoadLayoutTemplate;
using city::road::RoadLayoutTemplateId;
using city::road::RoadState;

[[nodiscard]] RoadLayoutTemplate UrbanTwoLane(RoadLayoutTemplateId id);
[[nodiscard]] RoadLayoutTemplate ThreeLane(RoadLayoutTemplateId id);
[[nodiscard]] RoadLayoutTemplate NoLeftSidewalk(RoadLayoutTemplateId id);
[[nodiscard]] RoadLayoutTemplate MedianTwoLane(RoadLayoutTemplateId id);
[[nodiscard]] RoadLayoutTemplate ShoulderedTwoLane(RoadLayoutTemplateId id);

// Registers one layout and reports the ID Core assigned, or 0 on failure.
[[nodiscard]] RoadLayoutTemplateId AddLayout(RoadState& state,
                                                RoadLayoutTemplate section);

} // namespace road_fixture
