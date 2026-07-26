#include "city/road/road.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
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
using city::road::JapaneseUrbanTwoLaneTemplate;
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
using city::road::ThreeLaneTemplate;
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

bool P2_section_transition_and_manual_markings(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(MakePath({MakeLine({0.0, 0.0}, {60.0, 0.0})}), 1);
  ROAD_TEST_EXPECT(added.ok, added.error);
  const auto template_id = state.AddSectionTemplate(ThreeLaneTemplate(0));
  ROAD_TEST_EXPECT(template_id.ok, template_id.error);
  SectionTransition transition{};
  transition.from_template = 1;
  transition.to_template = template_id.value;
  transition.start = StationRef{StationRefKind::kFromEnd, 25.0};
  transition.end = StationRef{StationRefKind::kFromEnd, 5.0};
  transition.rules = {
      SectionTransitionRule{35, TransitionAction::kTaperIn},
      SectionTransitionRule{10, TransitionAction::kContinue},
  };
  const auto transition_id = state.AddTransition(transition);
  ROAD_TEST_EXPECT(transition_id.ok, transition_id.error);
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
      {"P0_angled_segment_keeps_final_section_perpendicular", P0_angled_segment_keeps_final_section_perpendicular},
      {"P0_rejects_self_intersection_without_mutation", P0_rejects_self_intersection_without_mutation},
      {"P0_save_load_is_authoritative_and_bit_stable", P0_save_load_is_authoritative_and_bit_stable},
      {"P0_tool_preview_includes_bezier_handles", P0_tool_preview_includes_bezier_handles},
      {"P1_connected_segments_create_gates_and_junction", P1_connected_segments_create_gates_and_junction},
      {"P2_section_transition_and_manual_markings", P2_section_transition_and_manual_markings},
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
