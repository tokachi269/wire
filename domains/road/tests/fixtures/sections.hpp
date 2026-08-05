#pragma once

#include "city/road/road.hpp"

// Cross sections the road tests draw with. Core ships no product presets, so a
// test registers the sections it needs and uses the IDs AddSectionTemplate
// returns. Each builder takes the ID a test wants, or 0 to let Core assign one.
namespace road_fixture {

using city::road::CrossSectionTemplate;
using city::road::CrossSectionTemplateId;
using city::road::RoadState;

[[nodiscard]] CrossSectionTemplate UrbanTwoLane(CrossSectionTemplateId id);
[[nodiscard]] CrossSectionTemplate ThreeLane(CrossSectionTemplateId id);
[[nodiscard]] CrossSectionTemplate NoLeftSidewalk(CrossSectionTemplateId id);
[[nodiscard]] CrossSectionTemplate MedianTwoLane(CrossSectionTemplateId id);
[[nodiscard]] CrossSectionTemplate ShoulderedTwoLane(CrossSectionTemplateId id);

// Registers one section and reports the ID Core assigned, or 0 on failure.
[[nodiscard]] CrossSectionTemplateId AddSection(RoadState& state,
                                                CrossSectionTemplate section);

} // namespace road_fixture
