#include "city/road/road.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

namespace {

#define ROAD_TEST_EXPECT(condition, message) \
  do {                                       \
    if (!(condition)) {                      \
      failure = (message);                   \
      return false;                          \
    }                                        \
  } while (false)

using city::road::ErrorKind;
using city::road::BoundaryRole;
using city::road::JapaneseUrbanTwoLaneTemplate;
using city::road::MarkingRule;
using city::road::MakeBezier;
using city::road::MakeLine;
using city::road::MakePath;
using city::road::ManualAreaMarking;
using city::road::ManualLineMarking;
using city::road::PreviewRoadToolPath;
using city::road::RoadState;
using city::road::SectionTransition;
using city::road::SectionTransitionRule;
using city::road::StationRef;
using city::road::StationRefKind;
using city::road::SurfaceRole;
using city::road::ThreeLaneTemplate;
using city::road::TransitionAnchor;
using city::road::TransitionAction;
using city::road::ValidateGraphInvariants;
using city::road::Vec2d;

bool P0_generates_two_lane_segment(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1);
  ROAD_TEST_EXPECT(added.ok, added.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 1, "P0 did not save one authoritative segment");
  ROAD_TEST_EXPECT(state.derived().section_evaluations.size() > 2, "P0 did not derive section samples");
  ROAD_TEST_EXPECT(!state.derived().segment_meshes.empty(), "P0 did not derive a surface mesh");
  ROAD_TEST_EXPECT(!state.derived().terrain_masks.empty(), "P0 did not derive a terrain mask");
  const auto section = JapaneseUrbanTwoLaneTemplate(1);
  ROAD_TEST_EXPECT(section.bands.size() == 4, "Japanese two-lane template should have sidewalk/car/car/sidewalk");
  ROAD_TEST_EXPECT(std::abs(section.bands[1].width_m - 3.0) < 1e-9, "lane width is not 3.0m");
  ROAD_TEST_EXPECT(ValidateGraphInvariants(state.graph(), state.derived()).ok, "P0 invariants failed");
  return true;
}

bool P0_two_lane_mesh_shows_sidewalks_curbs_and_markings(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1);
  ROAD_TEST_EXPECT(added.ok, added.error);
  ROAD_TEST_EXPECT(!state.derived().section_evaluations.empty(), "P0 did not evaluate the section");
  const auto& boundaries = state.derived().section_evaluations.front().boundaries;
  ROAD_TEST_EXPECT(boundaries.size() >= 7, "P0 section does not expose sidewalk/curb/carriageway edges");
  ROAD_TEST_EXPECT(std::abs(boundaries.front().height_m - 0.13) < 1e-9,
                   "left sidewalk outer edge does not apply the 1% outward slope");
  ROAD_TEST_EXPECT(std::abs(boundaries[1].height_m - 0.15) < 1e-9,
                   "left curb top is not 0.15m above the carriageway edge");
  ROAD_TEST_EXPECT(std::abs(boundaries.back().height_m - 0.13) < 1e-9,
                   "right sidewalk outer edge does not apply the 1% outward slope");
  ROAD_TEST_EXPECT(std::abs(boundaries[5].height_m - 0.15) < 1e-9,
                   "right curb top is not 0.15m above the carriageway edge");
  ROAD_TEST_EXPECT(std::abs(boundaries[2].height_m) < 1e-9, "left carriageway edge is not at road height");
  ROAD_TEST_EXPECT(std::abs(boundaries[4].height_m) < 1e-9, "right carriageway edge is not at road height");
  ROAD_TEST_EXPECT(std::abs(boundaries[3].height_m - 0.06) < 1e-9,
                   "two-lane carriageway does not have a 2% center crown");
  ROAD_TEST_EXPECT(std::abs((boundaries[2].lateral_m - boundaries[1].lateral_m) - 0.2) < 1e-9,
                   "left curb width is not 0.2m");
  ROAD_TEST_EXPECT(std::abs((boundaries[5].lateral_m - boundaries[4].lateral_m) - 0.2) < 1e-9,
                   "right curb width is not 0.2m");
  ROAD_TEST_EXPECT(!state.derived().marking_meshes.empty(), "P0 did not derive marking meshes");
  const auto& marking = state.derived().marking_meshes.front();
  ROAD_TEST_EXPECT(!marking.vertices.empty(), "P0 marking mesh has no vertices");
  ROAD_TEST_EXPECT(!marking.indices.empty(), "P0 marking mesh has no triangles");
  std::set<std::string> materials{};
  for (const auto& mesh : state.derived().segment_meshes) materials.insert(mesh.material);
  ROAD_TEST_EXPECT(materials.contains("asphalt") && materials.contains("sidewalk") && materials.contains("curb"),
                   "P0 surface meshes are not separated by core material semantics");
  return true;
}

bool P0_angled_segment_keeps_final_section_perpendicular(std::string& failure) {
  RoadState state{};
  const Vec2d start{0.0, 0.0};
  const Vec2d end{24.0, 12.0};
  const auto added = state.AddSegment(MakePath({MakeLine(start, end)}), 1);
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto& vertices = state.derived().segment_meshes.front().vertices;
  const std::size_t row_count = static_cast<std::size_t>(std::ceil(std::hypot(24.0, 12.0) / 2.0)) + 1;
  ROAD_TEST_EXPECT(vertices.size() % row_count == 0, "surface mesh rows are not stable");
  const std::size_t row_width = vertices.size() / row_count;
  const auto& left = vertices[(row_count - 1) * row_width];
  const auto& right = vertices[row_count * row_width - 1];
  const double tangent_length = std::hypot(end.x - start.x, end.y - start.y);
  const double dot = (right.x - left.x) * (end.x - start.x) / tangent_length +
                     (right.y - left.y) * (end.y - start.y) / tangent_length;
  ROAD_TEST_EXPECT(std::abs(dot) < 1e-9, "final cross section is not perpendicular to the road tangent");
  return true;
}

bool P0_rejects_self_intersection_without_mutation(std::string& failure) {
  RoadState state{};
  const auto first = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1);
  ROAD_TEST_EXPECT(first.ok, first.error);
  const std::size_t before_segments = state.graph().segments.size();
  const auto bad = state.AddSegment(
      MakePath({MakeLine({0.0, 0.0}, {10.0, 10.0}), MakeLine({10.0, 10.0}, {0.0, 10.0}),
                MakeLine({0.0, 10.0}, {10.0, 0.0})}),
      1);
  ROAD_TEST_EXPECT(!bad.ok && bad.error_kind == ErrorKind::kUnsupported, "self-intersection was not unsupported");
  ROAD_TEST_EXPECT(state.graph().segments.size() == before_segments, "failed preflight mutated authoritative graph");
  return true;
}

bool P0_save_load_is_authoritative_and_bit_stable(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(MakePath({MakeBezier({0.0, 0.0}, {10.0, 12.0}, {30.0, -12.0}, {40.0, 0.0})}), 1);
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  const auto saved_again = loaded.value.Save();
  ROAD_TEST_EXPECT(saved_again.ok, saved_again.error);
  ROAD_TEST_EXPECT(saved.value == saved_again.value, "road save/load was not bit stable");
  return true;
}

bool P0_tool_preview_includes_bezier_handles(std::string& failure) {
  const auto draft = PreviewRoadToolPath({0.0, 0.0}, {40.0, 0.0}, Vec2d{10.0, 15.0}, Vec2d{30.0, -15.0});
  ROAD_TEST_EXPECT(draft.has_live_preview, "road tool lacks live preview");
  ROAD_TEST_EXPECT(draft.supports_bezier_handles, "road tool does not expose Bezier handles");
  ROAD_TEST_EXPECT(draft.preview_path.primitives.size() == 1, "road tool did not produce a preview primitive");
  return true;
}

bool P0_edit_and_delete_preserve_graph_ownership(std::string& failure) {
  RoadState state{};
  const auto isolated = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1);
  ROAD_TEST_EXPECT(isolated.ok, isolated.error);
  const auto edited = state.EditSegmentPath(isolated.value, MakePath({MakeLine({2.0, 3.0}, {24.0, 3.0})}));
  ROAD_TEST_EXPECT(edited.ok, edited.error);
  ROAD_TEST_EXPECT(std::abs(state.graph().nodes[0].position.x - 2.0) < 1e-9,
                   "isolated road edit did not move its owned endpoint node");

  const auto shared_node = state.graph().segments.front().node_a;
  const auto branch = state.AddSegmentConnectedTo(MakePath({MakeLine({2.0, 3.0}, {2.0, 23.0})}), 1, shared_node);
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  const auto before = state.Save();
  const auto invalid = state.EditSegmentPath(isolated.value, MakePath({MakeLine({4.0, 3.0}, {24.0, 3.0})}));
  ROAD_TEST_EXPECT(!invalid.ok && invalid.error_kind == ErrorKind::kUnsupported,
                   "connected road edit moved a shared endpoint node");
  ROAD_TEST_EXPECT(state.Save().value == before.value, "rejected connected road edit mutated authority");

  const auto deleted = state.DeleteSegment(branch.value);
  ROAD_TEST_EXPECT(deleted.ok, deleted.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 1, "road delete removed the wrong segment");
  ROAD_TEST_EXPECT(state.graph().nodes.size() == 2, "road delete left an orphan endpoint node");
  ROAD_TEST_EXPECT(state.graph().junctions.empty(), "road delete left a junction with fewer than two approaches");
  return true;
}

bool P1_connected_segments_create_gates_and_junction(std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1);
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto node = state.graph().segments.front().node_a;
  const auto branch = state.AddSegmentConnectedTo(MakePath({MakeLine({0.0, 0.0}, {0.0, 30.0})}), 1, node);
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  ROAD_TEST_EXPECT(!state.graph().junctions.empty(), "P1 did not save a JunctionDefinition");
  ROAD_TEST_EXPECT(!state.derived().junction_areas.empty(), "P1 did not derive a JunctionArea");
  ROAD_TEST_EXPECT(state.derived().junction_areas.front().gates.size() == 2, "P1 junction does not share two gates");
  const auto too_sharp = state.AddSegmentConnectedTo(MakePath({MakeLine({0.0, 0.0}, {30.0, 1.0})}), 1, node);
  ROAD_TEST_EXPECT(!too_sharp.ok && too_sharp.error_kind == ErrorKind::kUnsupported,
                   "P1 accepted a connection angle outside the fixed range");
  return true;
}

bool P1_segment_snap_splits_straight_road_for_t_junction(std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1);
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto branch = state.AddSegmentConnectedToSegment(MakePath({MakeLine({20.0, 0.0}, {20.0, 24.0})}), 1,
                                                         base.value);
  ROAD_TEST_EXPECT(branch.ok, branch.error);
  ROAD_TEST_EXPECT(state.graph().segments.size() == 3, "T junction did not split base road and add branch");
  ROAD_TEST_EXPECT(state.graph().nodes.size() == 4, "T junction did not create one shared middle node");
  ROAD_TEST_EXPECT(state.graph().junctions.size() == 1, "T junction did not save one JunctionDefinition");
  ROAD_TEST_EXPECT(state.derived().junction_areas.size() == 1, "T junction did not derive one JunctionArea");
  ROAD_TEST_EXPECT(state.derived().junction_areas.front().gates.size() == 3, "T junction does not have three gates");
  ROAD_TEST_EXPECT(state.derived().junction_meshes.size() == 1,
                   "T junction did not derive a visible junction surface mesh");
  ROAD_TEST_EXPECT(!state.derived().junction_meshes.front().indices.empty(),
                   "T junction surface mesh has no triangles");
  ROAD_TEST_EXPECT(state.derived().junction_marking_meshes.size() >= 6,
                   "T junction did not derive a stop line and zebra for every approach");
  for (const auto& gate : state.derived().junction_areas.front().gates) {
    ROAD_TEST_EXPECT(std::abs(std::hypot(gate.position.x - 20.0, gate.position.y) - 4.0) < 1e-6,
                     "T junction gate is not offset by the 4m corner radius");
  }
  ROAD_TEST_EXPECT(ValidateGraphInvariants(state.graph(), state.derived()).ok, "T junction invariants failed");
  return true;
}

bool P1_cross_junction_accepts_opposite_approaches(std::string& failure) {
  RoadState state{};
  const auto base = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {40.0, 0.0})}), 1);
  ROAD_TEST_EXPECT(base.ok, base.error);
  const auto north = state.AddSegmentConnectedToSegment(MakePath({MakeLine({20.0, 0.0}, {20.0, 24.0})}), 1,
                                                        base.value);
  ROAD_TEST_EXPECT(north.ok, north.error);
  const auto junction_node = state.graph().junctions.front().node_id;
  const auto south = state.AddSegmentConnectedTo(MakePath({MakeLine({20.0, 0.0}, {20.0, -24.0})}), 1,
                                                 junction_node);
  ROAD_TEST_EXPECT(south.ok, south.error);
  ROAD_TEST_EXPECT(state.derived().junction_areas.front().gates.size() == 4,
                   "cross junction does not have four approaches");
  ROAD_TEST_EXPECT(state.derived().junction_meshes.size() == 1,
                   "cross junction did not derive one shared surface");
  return true;
}

bool P2_section_transition_and_manual_markings(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(MakePath({MakeLine({100.0, 50.0}, {160.0, 50.0})}), 1);
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto template_id = state.AddSectionTemplate(ThreeLaneTemplate(0));
  ROAD_TEST_EXPECT(template_id.ok, template_id.error);
  SectionTransition transition{};
  transition.from_template = 1;
  transition.to_template = template_id.value;
  transition.start = StationRef{StationRefKind::kFromEnd, 25.0};
  transition.end = StationRef{StationRefKind::kFromEnd, 5.0};
  transition.anchor = TransitionAnchor::kLeftEdge;
  transition.rules = {
      SectionTransitionRule{35, TransitionAction::kTaperIn},
      SectionTransitionRule{10, TransitionAction::kContinue},
  };
  const auto transition_id = state.AddTransition(transition);
  ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
  const auto attached = state.AttachSectionTransition(added.value, transition_id.value);
  ROAD_TEST_EXPECT(attached.ok, attached.error);
  ManualLineMarking line{};
  line.owner_segment_id = added.value;
  line.path = MakePath({MakeLine({5.0, 0.5}, {20.0, 0.5})});
  line.style = "white";
  const auto line_id = state.AddManualLine(line);
  ROAD_TEST_EXPECT(line_id.ok, line_id.error);
  ManualAreaMarking area{};
  area.owner_segment_id = added.value;
  area.frame_origin = {30.0, 0.0};
  area.width_m = 4.0;
  area.length_m = 8.0;
  area.style = "zebra";
  const auto area_id = state.AddManualArea(area);
  ROAD_TEST_EXPECT(area_id.ok, area_id.error);
  ROAD_TEST_EXPECT(state.graph().transitions.size() == 1, "P2 did not save transition authority");
  ROAD_TEST_EXPECT(state.graph().manual_lines.size() == 1, "P2 did not save manual line authority");
  ROAD_TEST_EXPECT(state.graph().manual_areas.size() == 1, "P2 did not save manual area authority");
  ROAD_TEST_EXPECT(state.derived().manual_marking_meshes.size() == 2, "P2 did not derive manual marking meshes");
  ROAD_TEST_EXPECT(state.graph().segments.front().transition == transition_id.value,
                   "P2 transition is not attached to the segment authority");

  const auto& sections = state.derived().section_evaluations;
  const auto at_station = [&sections](double station) -> const city::road::SectionEvaluation* {
    const auto it = std::find_if(sections.begin(), sections.end(), [station](const auto& item) {
      return std::abs(item.station_m - station) < 1e-6;
    });
    return it == sections.end() ? nullptr : &*it;
  };
  const auto* before = at_station(0.0);
  const auto* after = at_station(60.0);
  ROAD_TEST_EXPECT(before != nullptr && after != nullptr, "P2 transition endpoints were not evaluated");
  const double before_width = before->boundaries.back().lateral_m - before->boundaries.front().lateral_m;
  const double after_width = after->boundaries.back().lateral_m - after->boundaries.front().lateral_m;
  ROAD_TEST_EXPECT(std::abs(before_width - 10.4) < 1e-6, "P2 transition changed the from-section before its start");
  ROAD_TEST_EXPECT(std::abs(after_width - 13.4) < 1e-6, "P2 transition did not reach the three-lane section");
  ROAD_TEST_EXPECT(std::abs(before->boundaries.front().lateral_m - after->boundaries.front().lateral_m) < 1e-6,
                   "P2 left-edge anchor moved during one-sided widening");

  const auto& line_mesh = state.derived().manual_marking_meshes[0];
  ROAD_TEST_EXPECT(!line_mesh.indices.empty(), "P2 manual line is not a drawable ribbon mesh");
  ROAD_TEST_EXPECT(std::abs(line_mesh.vertices.front().x - 105.0) < 1e-6 &&
                       std::abs(line_mesh.vertices.front().y - 50.5) < 0.1,
                   "P2 manual line was not transformed from owner-local coordinates");
  const auto& area_mesh = state.derived().manual_marking_meshes[1];
  ROAD_TEST_EXPECT(std::abs(area_mesh.vertices.front().x - 126.0) < 1e-6 &&
                       std::abs(area_mesh.vertices.front().y - 48.0) < 1e-6,
                   "P2 manual area was not transformed from owner-local coordinates");

  const auto saved = state.Save();
  ROAD_TEST_EXPECT(saved.ok, saved.error);
  const auto loaded = RoadState::Load(saved.value);
  ROAD_TEST_EXPECT(loaded.ok, loaded.error);
  ROAD_TEST_EXPECT(loaded.value.graph().transitions.size() == 1 &&
                       loaded.value.graph().manual_lines.size() == 1 &&
                       loaded.value.graph().manual_areas.size() == 1,
                   "P2 authority did not survive save/load");
  return true;
}

bool P2_supports_taper_lane_reduction_and_median_end(std::string& failure) {
  {
    RoadState state{};
    auto no_left_sidewalk = JapaneseUrbanTwoLaneTemplate(0);
    no_left_sidewalk.bands.erase(no_left_sidewalk.bands.begin());
    no_left_sidewalk.boundaries.erase(no_left_sidewalk.boundaries.begin());
    const auto target = state.AddSectionTemplate(no_left_sidewalk);
    ROAD_TEST_EXPECT(target.ok, target.error);
    const auto segment = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), 1);
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    SectionTransition transition{};
    transition.from_template = 1;
    transition.to_template = target.value;
    transition.start = StationRef{StationRefKind::kFromStart, 10.0};
    transition.end = StationRef{StationRefKind::kRatio, 0.5};
    transition.anchor = TransitionAnchor::kRightEdge;
    transition.rules = {SectionTransitionRule{10, TransitionAction::kTaperOut}};
    const auto transition_id = state.AddTransition(transition);
    ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
    ROAD_TEST_EXPECT(state.AttachSectionTransition(segment.value, transition_id.value).ok,
                     "sidewalk taper could not be attached");
    ROAD_TEST_EXPECT(!state.derived().segment_meshes.empty(), "sidewalk taper produced no material meshes");
  }

  {
    RoadState state{};
    const auto three_lane = state.AddSectionTemplate(ThreeLaneTemplate(0));
    ROAD_TEST_EXPECT(three_lane.ok, three_lane.error);
    const auto segment = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), three_lane.value);
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    SectionTransition transition{};
    transition.from_template = three_lane.value;
    transition.to_template = 1;
    transition.start = StationRef{StationRefKind::kFromEnd, 30.0};
    transition.end = StationRef{StationRefKind::kFromEnd, 5.0};
    transition.rules = {SectionTransitionRule{35, TransitionAction::kTaperOut}};
    const auto transition_id = state.AddTransition(transition);
    ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
    ROAD_TEST_EXPECT(state.AttachSectionTransition(segment.value, transition_id.value).ok,
                     "lane reduction could not be attached");
  }

  {
    RoadState state{};
    auto median = JapaneseUrbanTwoLaneTemplate(0);
    median.bands.insert(median.bands.begin() + 2, {25, SurfaceRole::kMedian, 2.0, 0.0, "median"});
    median.boundaries = {
        {100, BoundaryRole::kCurb, 0.2, -0.15, MarkingRule::kOuterLine},
        {210, BoundaryRole::kMedianEdge, 0.2, 0.12, MarkingRule::kNone},
        {220, BoundaryRole::kMedianEdge, 0.2, -0.12, MarkingRule::kNone},
        {300, BoundaryRole::kCurb, 0.2, 0.15, MarkingRule::kOuterLine},
    };
    const auto median_id = state.AddSectionTemplate(median);
    ROAD_TEST_EXPECT(median_id.ok, median_id.error);
    const auto segment = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), median_id.value);
    ROAD_TEST_EXPECT(segment.ok, segment.error);
    SectionTransition invalid{};
    invalid.from_template = median_id.value;
    invalid.to_template = 1;
    invalid.start = StationRef{StationRefKind::kFromStart, 10.0};
    invalid.end = StationRef{StationRefKind::kFromStart, 30.0};
    invalid.rules = {SectionTransitionRule{25, TransitionAction::kTaperOut}};
    ROAD_TEST_EXPECT(!state.AddTransition(invalid).ok, "median disappearance accepted TaperOut instead of EndCap");
    invalid.rules = {SectionTransitionRule{25, TransitionAction::kEndCap}};
    const auto transition_id = state.AddTransition(invalid);
    ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
    ROAD_TEST_EXPECT(state.AttachSectionTransition(segment.value, transition_id.value).ok,
                     "median end cap could not be attached");
  }
  return true;
}

bool P2_requires_transition_for_mixed_section_connection(std::string& failure) {
  {
    RoadState state{};
    const auto base = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), 1);
    ROAD_TEST_EXPECT(base.ok, base.error);
    const auto endpoint = state.graph().segments.front().node_b;
    const auto direct = state.AddSegmentConnectedTo(
        MakePath({MakeLine({60.0, 0.0}, {60.0, 20.0})}), 2, endpoint);
    ROAD_TEST_EXPECT(!direct.ok && direct.error_kind == ErrorKind::kUnsupported,
                     "P2 accepted a mixed-section node connection without a transition");
  }

  {
    RoadState state{};
    const auto base = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), 1);
    ROAD_TEST_EXPECT(base.ok, base.error);
    SectionTransition transition{};
    transition.to_template = 2;
    transition.start = StationRef{StationRefKind::kFromEnd, 20.0};
    transition.end = StationRef{StationRefKind::kFromEnd, 0.0};
    transition.rules = {SectionTransitionRule{35, TransitionAction::kTaperIn}};
    const auto transition_id = state.AddTransitionToSegment(base.value, transition);
    ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
    const auto endpoint = state.graph().segments.front().node_b;
    const auto connected = state.AddSegmentConnectedTo(
        MakePath({MakeLine({60.0, 0.0}, {60.0, 20.0})}), 2, endpoint);
    ROAD_TEST_EXPECT(connected.ok, connected.error);
  }
  return true;
}

bool road_does_not_enter_wire_core(std::string& failure) {
  const std::filesystem::path root = std::filesystem::current_path();
  const std::filesystem::path wire_domain = root / "domains" / "wire";
  if (!std::filesystem::exists(wire_domain)) {
    return true;
  }
  for (const auto& entry : std::filesystem::recursive_directory_iterator(wire_domain)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    const auto ext = entry.path().extension().string();
    if (ext != ".hpp" && ext != ".cpp" && ext != ".txt") {
      continue;
    }
    std::ifstream in(entry.path());
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    ROAD_TEST_EXPECT(text.find("city/road") == std::string::npos,
                     "wire core references road include: " + entry.path().string());
  }
  return true;
}

struct Test {
  const char* name;
  bool (*run)(std::string&);
};

} // namespace

int main() {
  const Test tests[] = {
      {"P0_generates_two_lane_segment", P0_generates_two_lane_segment},
      {"P0_two_lane_mesh_shows_sidewalks_curbs_and_markings", P0_two_lane_mesh_shows_sidewalks_curbs_and_markings},
      {"P0_angled_segment_keeps_final_section_perpendicular", P0_angled_segment_keeps_final_section_perpendicular},
      {"P0_rejects_self_intersection_without_mutation", P0_rejects_self_intersection_without_mutation},
      {"P0_save_load_is_authoritative_and_bit_stable", P0_save_load_is_authoritative_and_bit_stable},
      {"P0_tool_preview_includes_bezier_handles", P0_tool_preview_includes_bezier_handles},
      {"P0_edit_and_delete_preserve_graph_ownership", P0_edit_and_delete_preserve_graph_ownership},
      {"P1_connected_segments_create_gates_and_junction", P1_connected_segments_create_gates_and_junction},
      {"P1_segment_snap_splits_straight_road_for_t_junction", P1_segment_snap_splits_straight_road_for_t_junction},
      {"P1_cross_junction_accepts_opposite_approaches", P1_cross_junction_accepts_opposite_approaches},
      {"P2_section_transition_and_manual_markings", P2_section_transition_and_manual_markings},
      {"P2_supports_taper_lane_reduction_and_median_end", P2_supports_taper_lane_reduction_and_median_end},
      {"P2_requires_transition_for_mixed_section_connection", P2_requires_transition_for_mixed_section_connection},
      {"road_does_not_enter_wire_core", road_does_not_enter_wire_core},
  };
  int failed = 0;
  for (const Test& test : tests) {
    std::string failure;
    const bool ok = test.run(failure);
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << test.name << "\n";
    if (!ok) {
      std::cerr << "  reason: " << failure << "\n";
      ++failed;
    }
  }
  if (failed != 0) {
    return 1;
  }
  std::cout << "road tests passed (" << std::size(tests) << " cases)\n";
  return 0;
}
