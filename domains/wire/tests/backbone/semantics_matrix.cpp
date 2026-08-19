#include "cases.hpp"
#include "fixtures.hpp"
#include "semantics_coverage.hpp"

#include "city/wire/core_test_hook.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <string>

namespace backbone_tests {
namespace {

using semantics_coverage::Entry;
using semantics_coverage::Operation;

enum class MatrixState { kS1, kS2, kS3, kS4, kS5 };

struct MatrixFixture {
  city::wire::CoreState state{};
  city::wire::ObjectId pole_id = city::wire::kInvalidObjectId;
  city::wire::ObjectId reference_edge_bundle_id = city::wire::kInvalidObjectId;
};

bool configure_matrix_fixture_models(city::wire::CoreState* state) {
  if (state == nullptr) return false;
  constexpr city::wire::ModelAssemblyTemplateId kRowAssembly = 9836;
  constexpr city::wire::ModelAssemblyTemplateId kEndpointAssembly = 9837;

  city::wire::ModelAssemblyTemplate row{};
  row.id = kRowAssembly;
  city::wire::ModelAssemblyPart row_part{};
  row_part.part_id = 1;
  row_part.model_key = "semantics_row_fixture";
  row_part.sockets.push_back(
      {"endpoint_mount", {0.0, 0.0, 0.04}, {0.0, 0.0, 1.0}});
  row.parts.push_back(row_part);
  row.endpoint_mount_socket =
      city::wire::AssemblySocketRef{1, "endpoint_mount"};

  city::wire::ModelAssemblyTemplate endpoint{};
  endpoint.id = kEndpointAssembly;
  city::wire::ModelAssemblyPart endpoint_part{};
  endpoint_part.part_id = 1;
  endpoint_part.model_key = "semantics_endpoint_fixture";
  endpoint_part.sockets.push_back(
      {"wire", {0.0, 0.0, 0.20}, {1.0, 0.0, 0.0}});
  endpoint.parts.push_back(endpoint_part);
  endpoint.wire_socket = city::wire::AssemblySocketRef{1, "wire"};

  if (!state->RegisterModelAssemblyTemplate(row).ok ||
      !state->RegisterModelAssemblyTemplate(endpoint).ok) {
    return false;
  }
  const city::wire::BundleTemplateId template_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage);
  city::wire::BundleTemplate bundle = state->view().bundle_templates().at(template_id);
  bundle.row_fixture_assembly_id = kRowAssembly;
  bundle.endpoint_fixture_assembly_id = kEndpointAssembly;
  return state->UpdateBundleTemplate(bundle).ok;
}

city::wire::ObjectId first_lane_edge_bundle_at_pole(const city::wire::CoreState& state,
                                                     city::wire::ObjectId pole_id) {
  const city::wire::SavedBackboneNode* node = state.view().backbone_node_for_pole(pole_id);
  if (node == nullptr) return city::wire::kInvalidObjectId;
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.row_key.node_id == node->node_id && binding.lane_index == 0) {
      return binding.edge_bundle_id;
    }
  }
  return city::wire::kInvalidObjectId;
}

bool add_edge(MatrixFixture* fixture, const city::wire::Vec3d& endpoint) {
  const city::wire::Pole* pole = fixture->state.view().poles().find(fixture->pole_id);
  if (pole == nullptr) return false;
  city::wire::BackboneSpec request = line_req(fixture->state);
  request.path.polyline = {pole->world_transform.position, endpoint};
  request.path.node_specs = {pole_spec(0, fixture->pole_id)};
  return fixture->state.GenerateFromBackboneSpec(request).ok;
}

bool add_two_edges(MatrixFixture* fixture, const city::wire::Vec3d& a,
                   const city::wire::Vec3d& b) {
  const city::wire::Pole* pole = fixture->state.view().poles().find(fixture->pole_id);
  if (pole == nullptr) return false;
  city::wire::BackboneSpec request = poly3_req(fixture->state);
  request.path.polyline = {a, pole->world_transform.position, b};
  request.path.node_specs = {pole_spec(1, fixture->pole_id)};
  return fixture->state.GenerateFromBackboneSpec(request).ok;
}

bool make_matrix_fixture(MatrixState target, MatrixFixture* out) {
  if (out == nullptr) return false;
  if (!configure_matrix_fixture_models(&out->state)) return false;
  city::wire::BackboneSpec request = poly3_req(out->state);
  switch (target) {
  case MatrixState::kS1:
    request = line_req(out->state);
    request.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}};
    break;
  case MatrixState::kS2:
  case MatrixState::kS4:
  case MatrixState::kS5:
    request.path.polyline = {{-10.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
    break;
  case MatrixState::kS3:
    request.path.polyline = {{12.0, -8.0, 0.0}, {0.0, 0.0, 0.0}, {13.0, -10.0, 0.0}};
    break;
  }
  const auto generated = out->state.GenerateFromBackboneSpec(request);
  if (!generated.ok) return false;
  out->pole_id = target == MatrixState::kS1
                     ? generated.value.generated_pole_ids.front()
                     : generated.value.generated_pole_ids[1];
  if (target == MatrixState::kS4 || target == MatrixState::kS5) {
    if (!add_edge(out, {0.0, 12.0, 0.0})) return false;
  }
  if (target == MatrixState::kS5) {
    if (!add_two_edges(out, {-8.0, 14.0, 0.0}, {8.0, 14.0, 0.0})) return false;
  }
  out->reference_edge_bundle_id = first_lane_edge_bundle_at_pole(out->state, out->pole_id);
  return out->reference_edge_bundle_id != city::wire::kInvalidObjectId;
}

bool observe(MatrixFixture& fixture, Operation operation) {
  std::string error;
  WIRE_TEST_EXPECT_ANCHOR(
      semantics_coverage::Observe(operation, Entry::kCoreApi, fixture.state,
                                  fixture.pole_id, fixture.reference_edge_bundle_id,
                                  0, &error),
      error);
  return true;
}

bool expect_common_invariants(const city::wire::CoreState& state) {
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state);
  return true;
}

bool exercise_add_one(MatrixState initial) {
  MatrixFixture fixture{};
  WIRE_TEST_EXPECT_PRESENCE(make_matrix_fixture(initial, &fixture), "matrix state fixture failed");
  WIRE_TEST_EXPECT(observe(fixture, Operation::kAddOneEdge), "matrix state classification failed");
  const std::size_t spans_before = fixture.state.view().spans().size();
  const std::size_t continuity_before = fixture.state.view().backbone().row_continuities.size();
  const city::wire::Vec3d endpoint = initial == MatrixState::kS1
                                        ? city::wire::Vec3d{0.0, -10.0, 0.0}
                                        : city::wire::Vec3d{24.0, 18.0, 0.0};
  WIRE_TEST_EXPECT_PRESENCE(add_edge(&fixture, endpoint), "add_one_edge matrix operation failed");
  WIRE_TEST_EXPECT_PRESENCE(fixture.state.view().spans().size() == spans_before + 1,
                            "add_one_edge silently dropped its span");
  const bool connects = initial == MatrixState::kS1 || initial == MatrixState::kS4;
  WIRE_TEST_EXPECT_ANCHOR(
      fixture.state.view().backbone().row_continuities.size() == continuity_before + (connects ? 1U : 0U),
      "add_one_edge continuity delta does not match the semantics matrix");
  return expect_common_invariants(fixture.state);
}

bool exercise_add_two(MatrixState initial) {
  MatrixFixture fixture{};
  WIRE_TEST_EXPECT_PRESENCE(make_matrix_fixture(initial, &fixture), "matrix state fixture failed");
  WIRE_TEST_EXPECT(observe(fixture, Operation::kAddTwoEdges), "matrix state classification failed");
  const std::size_t spans_before = fixture.state.view().spans().size();
  const std::size_t continuity_before = fixture.state.view().backbone().row_continuities.size();
  WIRE_TEST_EXPECT_PRESENCE(add_two_edges(&fixture, {-18.0, 20.0, 0.0}, {18.0, 20.0, 0.0}),
                            "add_two_edges matrix operation failed");
  WIRE_TEST_EXPECT_PRESENCE(fixture.state.view().spans().size() == spans_before + 2,
                            "add_two_edges silently dropped a span");
  const bool connects = initial == MatrixState::kS2 || initial == MatrixState::kS3;
  WIRE_TEST_EXPECT_ANCHOR(
      fixture.state.view().backbone().row_continuities.size() == continuity_before + (connects ? 1U : 0U),
      "add_two_edges continuity delta does not match the semantics matrix");
  return expect_common_invariants(fixture.state);
}

bool exercise_move(MatrixState initial) {
  MatrixFixture fixture{};
  WIRE_TEST_EXPECT_PRESENCE(make_matrix_fixture(initial, &fixture), "matrix state fixture failed");
  WIRE_TEST_EXPECT(observe(fixture, Operation::kMovePoleAngle), "matrix state classification failed");
  const city::wire::Pole* pole = fixture.state.view().poles().find(fixture.pole_id);
  WIRE_TEST_EXPECT_PRESENCE(pole != nullptr, "matrix pole is missing");
  const std::size_t continuity_before = fixture.state.view().backbone().row_continuities.size();
  city::wire::Transformd moved = pole->world_transform;
  moved.position = moved.position + city::wire::Vec3d{0.2, 0.1, 0.0};
  WIRE_TEST_EXPECT_PRESENCE(fixture.state.MovePole(fixture.pole_id, moved).ok,
                            "move_pole_angle matrix operation failed");
  WIRE_TEST_EXPECT_ANCHOR(fixture.state.view().backbone().row_continuities.size() == continuity_before,
                          "move_pole_angle changed recorded connectivity");
  return expect_common_invariants(fixture.state);
}

bool exercise_update_placement(MatrixState initial) {
  MatrixFixture fixture{};
  WIRE_TEST_EXPECT_PRESENCE(make_matrix_fixture(initial, &fixture), "matrix state fixture failed");
  WIRE_TEST_EXPECT(observe(fixture, Operation::kUpdatePlacement), "matrix state classification failed");
  const city::wire::SavedBackboneEdgeBundle* edge_bundle =
      fixture.state.view().backbone_edge_bundle(fixture.reference_edge_bundle_id);
  const city::wire::Bundle* bundle =
      edge_bundle == nullptr ? nullptr : fixture.state.view().bundles().find(edge_bundle->bundle_id);
  WIRE_TEST_EXPECT_PRESENCE(bundle != nullptr, "matrix bundle is missing");
  const std::size_t continuity_before = fixture.state.view().backbone().row_continuities.size();
  WIRE_TEST_EXPECT_PRESENCE(
      fixture.state.UpdateBackboneBundlePlacement(bundle->id, true, bundle->height_m + 0.05,
                                                  bundle->lateral_m, bundle->phase_spacing_m).ok,
      "update_placement matrix operation failed");
  WIRE_TEST_EXPECT_ANCHOR(fixture.state.view().backbone().row_continuities.size() == continuity_before,
                          "update_placement changed recorded connectivity");
  return expect_common_invariants(fixture.state);
}

bool exercise_save_load(MatrixState initial) {
  MatrixFixture fixture{};
  WIRE_TEST_EXPECT_PRESENCE(make_matrix_fixture(initial, &fixture), "matrix state fixture failed");
  WIRE_TEST_EXPECT(observe(fixture, Operation::kSaveLoad), "matrix state classification failed");
  const std::size_t continuity_before = fixture.state.view().backbone().row_continuities.size();
  std::string saved;
  WIRE_TEST_EXPECT_PRESENCE(fixture.state.SerializeAuthoritative(&saved).ok, "matrix save failed");
  city::wire::CoreState loaded;
  const auto loaded_result = loaded.DeserializeAuthoritative(saved);
  WIRE_TEST_EXPECT_PRESENCE(loaded_result.ok, loaded_result.error);
  WIRE_TEST_EXPECT_ANCHOR(loaded.view().backbone().row_continuities.size() == continuity_before,
                          "save_load changed recorded connectivity");
  return expect_common_invariants(loaded);
}

bool exercise_regenerate(MatrixState initial) {
  MatrixFixture fixture{};
  WIRE_TEST_EXPECT_PRESENCE(make_matrix_fixture(initial, &fixture), "matrix state fixture failed");
  WIRE_TEST_EXPECT(observe(fixture, Operation::kRegenerate), "matrix state classification failed");
  const std::size_t continuity_before = fixture.state.view().backbone().row_continuities.size();
  city::wire::LayoutSettings settings = fixture.state.view().layout_settings();
  settings.min_side_scale += 0.01;
  WIRE_TEST_EXPECT_PRESENCE(fixture.state.UpdateLayoutSettings(settings).ok,
                            "regenerate matrix operation failed");
  WIRE_TEST_EXPECT_ANCHOR(fixture.state.view().backbone().row_continuities.size() == continuity_before,
                          "regenerate changed recorded connectivity");
  return expect_common_invariants(fixture.state);
}

struct MidspanFixture {
  city::wire::CoreState state{};
  city::wire::ObjectId source_edge_id = city::wire::kInvalidObjectId;
  city::wire::ObjectId source_edge_bundle_id = city::wire::kInvalidObjectId;
  city::wire::ObjectId source_node_a = city::wire::kInvalidObjectId;
  city::wire::ObjectId source_node_b = city::wire::kInvalidObjectId;
  city::wire::ObjectId source_node_id = city::wire::kInvalidObjectId;
};

bool make_midspan_fixture(bool add_branch, MidspanFixture* out) {
  if (out == nullptr) return false;
  const auto base = out->state.GenerateFromBackboneSpec(line_req(out->state));
  if (!base.ok || out->state.view().backbone().edges.size() != 1) return false;
  const city::wire::SavedBackboneEdge& source_edge = out->state.view().backbone().edges.front();
  out->source_edge_id = source_edge.edge_id;
  out->source_node_a = source_edge.node_a;
  out->source_node_b = source_edge.node_b;
  for (const city::wire::SavedBackboneEdgeBundle& edge_bundle : out->state.view().backbone().edge_bundles) {
    if (edge_bundle.edge_id == out->source_edge_id) {
      out->source_edge_bundle_id = edge_bundle.edge_bundle_id;
      break;
    }
  }
  if (out->source_edge_bundle_id == city::wire::kInvalidObjectId) return false;

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = source_edge.node_a;
  pick.segment_node_b_id = source_edge.node_b;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {12.0, 0.0, 0.0};
  city::wire::ResolveBranchPickOptions options{};
  options.selected_bundle_template_ids = {
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage)};
  const auto resolved = out->state.ResolveBranchPick(pick, options);
  if (!resolved.ok || resolved.value.resolved_node_id == city::wire::kInvalidObjectId) return false;
  out->source_node_id = resolved.value.resolved_node_id;
  if (!add_branch) return true;

  city::wire::BackboneSpec branch = line_req(out->state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  city::wire::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto generated = out->state.GenerateFromBackboneSpec(branch);
  return generated.ok && !generated.value.generated_span_ids.empty();
}

bool source_node_is_saved(const city::wire::CoreState& state, const MidspanFixture& fixture) {
  const auto matches_source = [&](city::wire::ObjectId node_a, city::wire::ObjectId node_b) {
    return (node_a == fixture.source_node_a && node_b == fixture.source_node_b) ||
           (node_a == fixture.source_node_b && node_b == fixture.source_node_a);
  };
  return std::any_of(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                     [&](const city::wire::SavedBackboneNode& node) {
                       return node.has_source_edge && matches_source(node.source_edge_node_a,
                                                                     node.source_edge_node_b);
                     });
}

bool midspan_connection_visual_exists(const city::wire::CoreState& state,
                                      const MidspanFixture& fixture) {
  const auto matches_source = [&](city::wire::ObjectId node_a, city::wire::ObjectId node_b) {
    return (node_a == fixture.source_node_a && node_b == fixture.source_node_b) ||
           (node_a == fixture.source_node_b && node_b == fixture.source_node_a);
  };
  std::vector<city::wire::ObjectId> saved_source_nodes{};
  for (const city::wire::SavedBackboneNode& node : state.view().backbone().nodes) {
    if (node.has_source_edge && matches_source(node.source_edge_node_a, node.source_edge_node_b)) {
      saved_source_nodes.push_back(node.node_id);
    }
  }
  return std::any_of(state.view().visual_curve_parts().parts.begin(),
                     state.view().visual_curve_parts().parts.end(),
                     [&](const city::wire::VisualCurvePart& part) {
                       return (part.kind == city::wire::VisualCurvePartKind::kNodePatch ||
                               part.kind == city::wire::VisualCurvePartKind::kJumper ||
                               part.kind == city::wire::VisualCurvePartKind::kLead) &&
                              std::find(saved_source_nodes.begin(), saved_source_nodes.end(),
                                        part.source_node_id) != saved_source_nodes.end() &&
                              part.incident_edge_ids.size() >= 2;
                     });
}

bool observe_midspan(MidspanFixture& fixture, Operation operation) {
  std::string error;
  WIRE_TEST_EXPECT_ANCHOR(
      semantics_coverage::ObserveMidspan(operation, Entry::kCoreApi, fixture.state,
                                         fixture.source_edge_id, &error),
      error);
  return true;
}

bool midspan_classifier_rejects_unbound_source() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  WIRE_TEST_EXPECT_PRESENCE(generated.ok, generated.error);
  WIRE_TEST_EXPECT_PRESENCE(state.view().backbone().edges.size() == 1,
                            "SM classifier fixture has no source edge");
  std::string error;
  const bool observed = semantics_coverage::ObserveMidspan(
      Operation::kAddOneEdge, Entry::kCoreApi, state,
      state.view().backbone().edges.front().edge_id, &error);
  WIRE_TEST_EXPECT_ORACLE(!observed, "SM classifier accepted an edge without a source node");
  return true;
}

bool exercise_midspan_add_one() {
  MidspanFixture fixture{};
  WIRE_TEST_EXPECT_PRESENCE(make_midspan_fixture(false, &fixture), "SM source edge fixture failed");
  WIRE_TEST_EXPECT(observe_midspan(fixture, Operation::kAddOneEdge), "SM add_one_edge observation failed");
  const std::size_t spans_before = fixture.state.view().spans().size();

  const city::wire::SupportNode* pending = nullptr;
  for (const city::wire::SupportNode& candidate : city::wire::CoreStateTestHook::pending_support_nodes(fixture.state)) {
    if (candidate.node_id == fixture.source_node_id) {
      pending = &candidate;
      break;
    }
  }
  WIRE_TEST_EXPECT_PRESENCE(pending != nullptr, "SM pending source node is missing");
  city::wire::BackboneSpec branch = line_req(fixture.state);
  branch.path.polyline = {pending->position, {6.0, 8.0, 0.0}};
  city::wire::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = pending->support_kind;
  node.node_id = pending->node_id;
  branch.path.node_specs = {node};
  const auto generated = fixture.state.GenerateFromBackboneSpec(branch);
  WIRE_TEST_EXPECT_PRESENCE(generated.ok, generated.error);
  WIRE_TEST_EXPECT_PRESENCE(fixture.state.view().spans().size() == spans_before + 1,
                            "SM add_one_edge silently dropped its span");
  WIRE_TEST_EXPECT_ANCHOR(source_node_is_saved(fixture.state, fixture), "SM add_one_edge did not preserve source identity");
  WIRE_TEST_EXPECT_ANCHOR(midspan_connection_visual_exists(fixture.state, fixture),
                          "SM add_one_edge did not derive a midspan connection visual");
  return expect_common_invariants(fixture.state);
}

bool exercise_midspan_update() {
  MidspanFixture fixture{};
  WIRE_TEST_EXPECT_PRESENCE(make_midspan_fixture(true, &fixture), "SM branch fixture failed");
  WIRE_TEST_EXPECT(observe_midspan(fixture, Operation::kUpdatePlacement), "SM update observation failed");
  const city::wire::SavedBackboneEdgeBundle* edge_bundle =
      fixture.state.view().backbone_edge_bundle(fixture.source_edge_bundle_id);
  const city::wire::Bundle* bundle =
      edge_bundle == nullptr ? nullptr : fixture.state.view().bundles().find(edge_bundle->bundle_id);
  WIRE_TEST_EXPECT_PRESENCE(bundle != nullptr, "SM source bundle is missing");
  const auto updated = fixture.state.UpdateBackboneBundlePlacement(
      bundle->id, true, bundle->height_m + 0.05, bundle->lateral_m, bundle->phase_spacing_m);
  WIRE_TEST_EXPECT_PRESENCE(updated.ok, updated.error);
  WIRE_TEST_EXPECT_ANCHOR(source_node_is_saved(fixture.state, fixture), "SM update lost source identity");
  WIRE_TEST_EXPECT_ANCHOR(midspan_connection_visual_exists(fixture.state, fixture),
                          "SM update lost midspan connection visual");
  return expect_common_invariants(fixture.state);
}

bool exercise_midspan_save_load() {
  MidspanFixture fixture{};
  WIRE_TEST_EXPECT_PRESENCE(make_midspan_fixture(true, &fixture), "SM branch fixture failed");
  WIRE_TEST_EXPECT(observe_midspan(fixture, Operation::kSaveLoad), "SM save_load observation failed");
  std::string saved;
  const auto serialized = fixture.state.SerializeAuthoritative(&saved);
  WIRE_TEST_EXPECT_PRESENCE(serialized.ok, serialized.error);
  city::wire::CoreState loaded;
  const auto loaded_result = loaded.DeserializeAuthoritative(saved);
  WIRE_TEST_EXPECT_PRESENCE(loaded_result.ok, loaded_result.error);
  WIRE_TEST_EXPECT_ANCHOR(source_node_is_saved(loaded, fixture),
                          "SM save_load lost source identity");
  WIRE_TEST_EXPECT_ANCHOR(midspan_connection_visual_exists(loaded, fixture),
                          "SM save_load lost midspan connection visual");
  return expect_common_invariants(loaded);
}

bool exercise_midspan_regenerate() {
  MidspanFixture fixture{};
  WIRE_TEST_EXPECT_PRESENCE(make_midspan_fixture(true, &fixture), "SM branch fixture failed");
  WIRE_TEST_EXPECT(observe_midspan(fixture, Operation::kRegenerate), "SM regenerate observation failed");
  city::wire::LayoutSettings settings = fixture.state.view().layout_settings();
  settings.min_side_scale += 0.01;
  const auto updated = fixture.state.UpdateLayoutSettings(settings);
  WIRE_TEST_EXPECT_PRESENCE(updated.ok, updated.error);
  WIRE_TEST_EXPECT_ANCHOR(source_node_is_saved(fixture.state, fixture), "SM regenerate lost source identity");
  WIRE_TEST_EXPECT_ANCHOR(midspan_connection_visual_exists(fixture.state, fixture),
                          "SM regenerate lost midspan connection visual");
  return expect_common_invariants(fixture.state);
}
bool exercise_empty(Operation operation) {
  city::wire::CoreState state;
  std::string error;
  WIRE_TEST_EXPECT(semantics_coverage::ObserveEmpty(operation, Entry::kCoreApi, state, &error), error);
  if (operation == Operation::kAddOneEdge) {
    const auto result = state.GenerateFromBackboneSpec(line_req(state));
    WIRE_TEST_EXPECT_PRESENCE(result.ok, result.error);
    WIRE_TEST_EXPECT_PRESENCE(result.value.generated_span_ids.size() == 1,
                              "S0 add_one_edge silently dropped its span");
  } else if (operation == Operation::kAddTwoEdges) {
    const auto result = state.GenerateFromBackboneSpec(poly3_req(state));
    WIRE_TEST_EXPECT_PRESENCE(result.ok, result.error);
    WIRE_TEST_EXPECT_PRESENCE(result.value.generated_span_ids.size() == 2,
                              "S0 add_two_edges silently dropped a span");
    WIRE_TEST_EXPECT_ANCHOR(state.view().backbone().row_continuities.size() == 1,
                            "S0 add_two_edges did not record its pair");
  } else if (operation == Operation::kSaveLoad) {
    std::string saved;
    WIRE_TEST_EXPECT_PRESENCE(state.SerializeAuthoritative(&saved).ok, "S0 save failed");
    city::wire::CoreState loaded;
    WIRE_TEST_EXPECT_PRESENCE(loaded.DeserializeAuthoritative(saved).ok, "S0 load failed");
    WIRE_TEST_EXPECT_ANCHOR(loaded.view().backbone().port_bindings.empty(),
                            "S0 save_load created endpoint bindings");
  } else if (operation == Operation::kRegenerate) {
    city::wire::LayoutSettings settings = state.view().layout_settings();
    settings.min_side_scale += 0.01;
    WIRE_TEST_EXPECT_PRESENCE(state.UpdateLayoutSettings(settings).ok, "S0 regenerate failed");
    WIRE_TEST_EXPECT_ANCHOR(state.view().backbone().port_bindings.empty(),
                            "S0 regenerate created endpoint bindings");
  } else {
    WIRE_TEST_EXPECT_ORACLE(false, "operation is not defined for S0");
  }
  return expect_common_invariants(state);
}

} // namespace

bool C836_backbone_operation_matrix_executes_declared_states() {
  WIRE_TEST_EXPECT(midspan_classifier_rejects_unbound_source(),
                   "SM classifier accepted a caller-provided state tag");
  WIRE_TEST_EXPECT(exercise_empty(Operation::kAddOneEdge), "S0 add_one_edge failed");
  WIRE_TEST_EXPECT(exercise_empty(Operation::kAddTwoEdges), "S0 add_two_edges failed");
  WIRE_TEST_EXPECT(exercise_empty(Operation::kSaveLoad), "S0 save_load failed");
  WIRE_TEST_EXPECT(exercise_empty(Operation::kRegenerate), "S0 regenerate failed");

  WIRE_TEST_EXPECT(exercise_midspan_add_one(), "SM add_one_edge failed");
  WIRE_TEST_EXPECT(exercise_midspan_update(), "SM update_placement failed");
  WIRE_TEST_EXPECT(exercise_midspan_save_load(), "SM save_load failed");
  WIRE_TEST_EXPECT(exercise_midspan_regenerate(), "SM regenerate failed");

  constexpr std::array<MatrixState, 5> states{MatrixState::kS1, MatrixState::kS2, MatrixState::kS3,
                                               MatrixState::kS4, MatrixState::kS5};
  for (MatrixState state : states) {
    WIRE_TEST_EXPECT(exercise_add_one(state), "add_one_edge matrix row failed");
    WIRE_TEST_EXPECT(exercise_add_two(state), "add_two_edges matrix row failed");
    WIRE_TEST_EXPECT(exercise_move(state), "move_pole_angle matrix row failed");
    WIRE_TEST_EXPECT(exercise_update_placement(state), "update_placement matrix row failed");
    WIRE_TEST_EXPECT(exercise_save_load(state), "save_load matrix row failed");
    WIRE_TEST_EXPECT(exercise_regenerate(state), "regenerate matrix row failed");
  }
  return true;
}

} // namespace backbone_tests
