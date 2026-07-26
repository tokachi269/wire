#include "city/road/road.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <locale>
#include <random>
#include <sstream>
#include <string>
#include <string_view>

namespace {

#define ROAD_CONTRACT_EXPECT(condition, message) \
  do {                                           \
    if (!(condition)) {                          \
      failure = (message);                       \
      return false;                              \
    }                                            \
  } while (false)

using namespace city::road;

void append_boundary(std::ostringstream& out, const SectionBoundarySample& value) {
  out << value.boundary_id << ',' << static_cast<int>(value.role) << ',' << value.lateral_m << ','
      << value.height_m << ',' << static_cast<int>(value.marking_rule) << ';';
}

void append_gate(std::ostringstream& out, const ConnectionGate& value) {
  out << value.segment_id << ',' << value.node_id << ',' << value.position.x << ',' << value.position.y << ','
      << value.position.z << ',' << value.tangent.x << ',' << value.tangent.y << ',' << value.tangent.z << ':';
  for (const auto& boundary : value.boundaries) append_boundary(out, boundary);
}

void append_meshes(std::ostringstream& out, const std::vector<Mesh>& meshes) {
  out << meshes.size() << ':';
  for (const auto& mesh : meshes) {
    out << mesh.owner_segment_id << ',' << mesh.material.size() << ':' << mesh.material << ',' << mesh.vertices.size()
        << ',' << mesh.indices.size() << ':';
    for (const auto& vertex : mesh.vertices) out << vertex.x << ',' << vertex.y << ',' << vertex.z << ';';
    for (const auto index : mesh.indices) out << index << ',';
  }
}

std::string derived_observation(const DerivedRoad& derived) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::hexfloat;
  for (const std::size_t runs : derived.build_stage_runs) out << runs << ',';
  out << ':';
  out << derived.canonical_alignments.size() << ':';
  for (const auto& alignment : derived.canonical_alignments) {
    out << alignment.segment_id << ',' << alignment.path.spans.size() << ':';
    for (const auto& span : alignment.path.spans) {
      out << span.p0.x << ',' << span.p0.y << ',' << span.p1.x << ',' << span.p1.y << ','
          << span.p2.x << ',' << span.p2.y << ',' << span.p3.x << ',' << span.p3.y << ';';
    }
  }
  out << derived.section_evaluations.size() << ':';
  for (const auto& section : derived.section_evaluations) {
    out << section.segment_id << ',' << section.station_m << ':';
    for (const auto& boundary : section.boundaries) append_boundary(out, boundary);
    for (const auto& material : section.surface_materials) out << material.size() << ':' << material << ';';
  }
  append_meshes(out, derived.segment_meshes);
  append_meshes(out, derived.marking_meshes);
  out << derived.terrain_masks.size() << ':';
  for (const auto& mask : derived.terrain_masks) {
    out << mask.segment_id << ',' << mask.points.size() << ':';
    for (const auto& point : mask.points) out << point.x << ',' << point.y << ';';
  }
  out << derived.connection_gates.size() << ':';
  for (const auto& gate : derived.connection_gates) append_gate(out, gate);
  out << derived.connection_areas.size() << ':';
  for (const auto& area : derived.connection_areas) {
    out << area.node_id << ',' << area.gates.size() << ':';
    for (const auto& gate : area.gates) append_gate(out, gate);
  }
  append_meshes(out, derived.connection_meshes);
  out << derived.junction_areas.size() << ':';
  for (const auto& area : derived.junction_areas) {
    out << area.policy_override_id << ',' << area.node_id << ',' << area.gates.size() << ':';
    for (const auto& gate : area.gates) append_gate(out, gate);
  }
  append_meshes(out, derived.junction_meshes);
  append_meshes(out, derived.junction_marking_meshes);
  append_meshes(out, derived.manual_marking_meshes);
  return out.str();
}

std::uint64_t next_id_observation(std::string_view authoritative) {
  const std::string_view key = "next_id=";
  const std::size_t begin = authoritative.find(key);
  if (begin == std::string_view::npos) return 0;
  const std::size_t value_begin = begin + key.size();
  const std::size_t end = authoritative.find('\n', value_begin);
  return static_cast<std::uint64_t>(std::stoull(std::string(authoritative.substr(value_begin, end - value_begin))));
}

std::string query_observation(const RoadState& state) {
  std::ostringstream out;
  out << state.graph().nodes.size() << ',' << state.graph().segments.size() << ','
      << state.graph().section_templates.size() << ',' << state.graph().transitions.size() << ','
      << state.graph().connection_policy_overrides.size() << ',' << state.graph().manual_lines.size() << ','
      << state.graph().manual_areas.size() << ':';
  for (const auto& node : state.graph().nodes) out << 'n' << node.id << ';';
  for (const auto& segment : state.graph().segments) out << 's' << segment.id << ';';
  for (const auto& gate : state.derived().connection_gates) out << 'g' << gate.segment_id << ',' << gate.node_id << ';';
  return out.str();
}

struct StateObservation {
  std::string authoritative{};
  std::string derived{};
  std::uint64_t next_id = 0;
  std::string query{};

  bool operator==(const StateObservation&) const = default;
};

StateObservation observe(const RoadState& state) {
  const auto saved = state.Save();
  if (!saved.ok) return {};
  return StateObservation{saved.value, derived_observation(state.derived()), next_id_observation(saved.value),
                          query_observation(state)};
}

template <typename Operation>
bool expect_failed_unchanged(RoadState& state, Operation&& operation, std::string_view label, std::string& failure) {
  const StateObservation before = observe(state);
  const auto result = operation();
  if (result.ok) {
    failure = std::string(label) + " unexpectedly succeeded";
    return false;
  }
  if (!(observe(state) == before)) {
    failure = std::string(label) + " changed authoritative, derived, next ID, or query state after failure";
    return false;
  }
  return true;
}

bool all_public_operation_validation_failures_are_atomic(std::string& failure) {
  RoadState state{};
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(state, [&] { return state.AddSegment(city::road::AddSegmentRequest{{}, 1}); }, "AddSegment", failure),
                       failure);
  const Path line = MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})});
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{line, 1, 999999}); },
                           "AddSegmentConnectedTo", failure), failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddSegmentConnectedToSegment(city::road::AddSegmentConnectedToSegmentRequest{line, 1, 999999, 10.0}); },
                           "AddSegmentConnectedToSegment", failure), failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.EditSegmentShape(city::road::EditSegmentShapeRequest{999999, {}}); }, "EditSegmentShape", failure),
                       failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.MoveNode(city::road::MoveNodeRequest{999999, {0.0, 0.0}}); }, "MoveNode", failure), failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.DeleteSegment(city::road::DeleteSegmentRequest{999999}); }, "DeleteSegment", failure), failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddSectionTemplate(city::road::AddSectionTemplateRequest{{}}); }, "AddSectionTemplate", failure),
                       failure);
  CrossSectionTemplate missing = JapaneseUrbanTwoLaneTemplate(999999);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.EditSectionTemplate(city::road::EditSectionTemplateRequest{missing}); }, "EditSectionTemplate", failure),
                       failure);
  SectionTransitionRequest invalid_transition{};
  invalid_transition.from_template = 1;
  invalid_transition.to_template = 999999;
  invalid_transition.start = {StationRefKind::kFromStart, 1.0};
  invalid_transition.end = {StationRefKind::kFromStart, 2.0};
  invalid_transition.rules = {{10, TransitionAction::kContinue}};
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddTransition(invalid_transition); }, "AddTransition", failure),
                       failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddTransitionToSegment(city::road::AddTransitionToSegmentRequest{999999, invalid_transition}); },
                           "AddTransitionToSegment", failure), failure);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AttachSectionTransition(city::road::AttachSectionTransitionRequest{999999, 999998}); },
                           "AttachSectionTransition", failure), failure);
  ManualLineRequest line_marking{};
  line_marking.owner_segment_id = 999999;
  line_marking.path = line;
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddManualLine(line_marking); }, "AddManualLine", failure),
                       failure);
  ManualAreaRequest area_marking{};
  area_marking.owner_segment_id = 999999;
  area_marking.width_m = 1.0;
  area_marking.length_m = 1.0;
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state, [&] { return state.AddManualArea(area_marking); }, "AddManualArea", failure),
                       failure);
  return true;
}

bool add_segment_build_failure_is_atomic(std::string& failure) {
  RoadState state{};
  CrossSectionTemplate unusable{};
  unusable.bands = {
      SurfaceBand{10, SurfaceRole::kSidewalk, 1.0e308, 0.0, "sidewalk"},
      SurfaceBand{20, SurfaceRole::kCarriageway, 1.0e308, 0.0, "asphalt"},
      SurfaceBand{30, SurfaceRole::kSidewalk, 1.0e308, 0.0, "sidewalk"},
  };
  unusable.boundaries = {
      BoundaryProfile{11, BoundaryRole::kCurb, 0.0, 0.15, MarkingRule::kNone},
      BoundaryProfile{21, BoundaryRole::kCurb, 0.0, -0.15, MarkingRule::kNone},
  };
  const auto template_id = state.AddSectionTemplate(city::road::AddSectionTemplateRequest{std::move(unusable)});
  ROAD_CONTRACT_EXPECT(template_id.ok, template_id.error);
  ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                           state,
                           [&] {
                             return state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}),
                                                     template_id.value});
                           },
                           "AddSegment build failure", failure),
                       failure);
  return true;
}

bool node_identity_does_not_come_from_position(std::string& failure) {
  RoadState state{};
  const auto first = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  const auto second = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {0.0, 20.0})}), 1});
  const auto near = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0000001}, {-20.0, 0.0000001})}), 1});
  ROAD_CONTRACT_EXPECT(first.ok && second.ok && near.ok, "independent segments could not be created");
  ROAD_CONTRACT_EXPECT(state.graph().nodes.size() == 6, "equal or near node positions were merged by geometry");
  ROAD_CONTRACT_EXPECT(state.graph().segments[0].node_a != state.graph().segments[1].node_a &&
                           state.graph().segments[0].node_a != state.graph().segments[2].node_a,
                       "node identity was inferred from equal or epsilon-near positions");
  return true;
}

bool segment_shape_edit_does_not_move_endpoint_authority(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);
  const RoadNode before_a = state.graph().nodes[0];
  const RoadNode before_b = state.graph().nodes[1];
  const auto shape = SegmentShapeFromPath(
      MakePath({MakeBezier({3.0, 4.0}, {7.0, 10.0}, {16.0, -5.0}, {24.0, 2.0})}));
  ROAD_CONTRACT_EXPECT(shape.ok, shape.error);
  const auto edited = state.EditSegmentShape(city::road::EditSegmentShapeRequest{added.value, shape.value});
  ROAD_CONTRACT_EXPECT(edited.ok, edited.error);
  ROAD_CONTRACT_EXPECT(state.graph().nodes[0].id == before_a.id && state.graph().nodes[1].id == before_b.id &&
                           state.graph().nodes[0].position.x == before_a.position.x &&
                           state.graph().nodes[0].position.y == before_a.position.y &&
                           state.graph().nodes[1].position.x == before_b.position.x &&
                           state.graph().nodes[1].position.y == before_b.position.y,
                       "segment shape edit changed RoadNode endpoint authority");
  return true;
}

bool move_node_rederives_incident_alignment_endpoints(std::string& failure) {
  RoadState state{};
  const auto first = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(first.ok, first.error);
  const RoadNodeId shared = state.graph().segments.front().node_b;
  const auto second = state.AddSegmentConnectedTo(city::road::AddSegmentConnectedToRequest{MakePath({MakeLine({20.0, 0.0}, {40.0, 0.0})}), 1, shared});
  ROAD_CONTRACT_EXPECT(second.ok, second.error);
  const auto moved = state.MoveNode(city::road::MoveNodeRequest{shared, {20.0, 5.0}});
  ROAD_CONTRACT_EXPECT(moved.ok, moved.error);
  const Path* first_path = FindCanonicalAlignment(state.derived(), first.value);
  const Path* second_path = FindCanonicalAlignment(state.derived(), second.value);
  ROAD_CONTRACT_EXPECT(first_path != nullptr && second_path != nullptr, "incident canonical alignment is missing");
  ROAD_CONTRACT_EXPECT(first_path->spans.back().p3.x == 20.0 && first_path->spans.back().p3.y == 5.0 &&
                           second_path->spans.front().p0.x == 20.0 && second_path->spans.front().p0.y == 5.0,
                       "MoveNode did not rederive every incident canonical endpoint");
  ROAD_CONTRACT_EXPECT(state.graph().segments[0].node_b == shared && state.graph().segments[1].node_a == shared,
                       "MoveNode changed segment connectivity");
  return true;
}

bool build_runs_each_stage_exactly_once(std::string& failure) {
  RoadState state{};
  const auto added = state.AddSegment(city::road::AddSegmentRequest{MakePath({MakeLine({0.0, 0.0}, {20.0, 0.0})}), 1});
  ROAD_CONTRACT_EXPECT(added.ok, added.error);
  ROAD_CONTRACT_EXPECT(std::all_of(state.derived().build_stage_runs.begin(), state.derived().build_stage_runs.end(),
                                   [](std::size_t runs) { return runs == 1; }),
                       "Build did not execute every stage exactly once");
  return true;
}

bool seeded_operation_sequences_preserve_contracts(std::string& failure) {
  for (std::uint32_t seed = 1; seed <= 4; ++seed) {
    std::mt19937 random(seed);
    RoadState state{};
    for (int step = 0; step < 8; ++step) {
      const double x = static_cast<double>(step * 30);
      const double y = static_cast<double>(random() % 21) - 10.0;
      const auto added = state.AddSegment(AddSegmentRequest{
          MakePath({MakeLine({x, y}, {x + 20.0, y + static_cast<double>(random() % 5)})}), 1});
      ROAD_CONTRACT_EXPECT(added.ok, "seed " + std::to_string(seed) + " AddSegment: " + added.error);
      const RoadSegment& segment = state.graph().segments.back();
      if (step % 3 == 0) {
        const auto node = std::find_if(state.graph().nodes.begin(), state.graph().nodes.end(),
                                       [&segment](const auto& item) { return item.id == segment.node_b; });
        ROAD_CONTRACT_EXPECT(node != state.graph().nodes.end(), "seed node lookup failed");
        const auto moved = state.MoveNode(MoveNodeRequest{node->id, {node->position.x, node->position.y + 0.5}});
        ROAD_CONTRACT_EXPECT(moved.ok, "seed " + std::to_string(seed) + " MoveNode: " + moved.error);
      }
      if (step % 4 == 0) {
        const Path* path = FindCanonicalAlignment(state.derived(), added.value);
        ROAD_CONTRACT_EXPECT(path != nullptr, "seed canonical alignment lookup failed");
        const auto shape = SegmentShapeFromPath(*path);
        ROAD_CONTRACT_EXPECT(shape.ok, "seed shape conversion failed");
        SegmentShape edited_shape = shape.value;
        edited_shape.start_handle.y += 0.25;
        edited_shape.end_handle.y -= 0.25;
        const auto edited = state.EditSegmentShape(EditSegmentShapeRequest{added.value, edited_shape});
        ROAD_CONTRACT_EXPECT(edited.ok, "seed " + std::to_string(seed) + " EditSegmentShape: " + edited.error);
      }
      ROAD_CONTRACT_EXPECT(ValidateGraphInvariants(state.graph(), state.derived()).ok,
                           "seed " + std::to_string(seed) + " invariant failed at step " + std::to_string(step));
      const auto saved = state.Save();
      ROAD_CONTRACT_EXPECT(saved.ok, "seed save failed");
      const auto loaded = RoadState::Load(saved.value);
      ROAD_CONTRACT_EXPECT(loaded.ok && loaded.value.Save().value == saved.value,
                           "seed " + std::to_string(seed) + " round-trip failed at step " + std::to_string(step));
      ROAD_CONTRACT_EXPECT(expect_failed_unchanged(
                               state, [&state] { return state.MoveNode(MoveNodeRequest{999999, {0.0, 0.0}}); },
                               "seed invalid MoveNode", failure),
                           failure);
    }
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
      {"all_public_operation_validation_failures_are_atomic", all_public_operation_validation_failures_are_atomic},
      {"add_segment_build_failure_is_atomic", add_segment_build_failure_is_atomic},
      {"node_identity_does_not_come_from_position", node_identity_does_not_come_from_position},
      {"segment_shape_edit_does_not_move_endpoint_authority", segment_shape_edit_does_not_move_endpoint_authority},
      {"move_node_rederives_incident_alignment_endpoints", move_node_rederives_incident_alignment_endpoints},
      {"build_runs_each_stage_exactly_once", build_runs_each_stage_exactly_once},
      {"seeded_operation_sequences_preserve_contracts", seeded_operation_sequences_preserve_contracts},
  };
  int failed = 0;
  for (const auto& test : tests) {
    std::string failure;
    const bool ok = test.run(failure);
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << test.name << '\n';
    if (!ok) {
      std::cerr << "  reason: " << failure << '\n';
      ++failed;
    }
  }
  if (failed != 0) return 1;
  std::cout << "road architecture contract tests passed (" << std::size(tests) << " cases)\n";
  return 0;
}
