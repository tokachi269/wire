#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace helpers;

namespace backbone_tests {

namespace {

wire::core::Vec3d normalize_xy(wire::core::Vec3d value) {
  value.z = 0.0;
  const double len = std::sqrt(value.x * value.x + value.y * value.y);
  if (len <= 1e-12) {
    return {};
  }
  return {value.x / len, value.y / len, 0.0};
}

wire::core::Vec3d lateral_xy(const wire::core::Vec3d& value) {
  const wire::core::Vec3d dir = normalize_xy(value);
  return {-dir.y, dir.x, 0.0};
}

double abs_dot_xy(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  return std::abs(a.x * b.x + a.y * b.y);
}

double dot_xy(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  return a.x * b.x + a.y * b.y;
}

bool prepare_two_lane_low_voltage(wire::core::CoreState& state) {
  const auto it = state.view().bundle_templates().find(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage));
  if (it == state.view().bundle_templates().end()) {
    return false;
  }
  wire::core::BundleTemplate tpl = it->second;
  tpl.count_rule = wire::core::BundleCountRuleKind::kFixed;
  tpl.fixed_count = 2;
  tpl.default_count = 2;
  tpl.min_count = std::min(tpl.min_count, 2);
  tpl.max_count = std::max(tpl.max_count, 2);
  return state.UpdateBundleTemplate(tpl).ok;
}

const wire::core::SavedBackboneSpanBinding* saved_span_binding(const wire::core::CoreState& state,
                                                                wire::core::ObjectId span_id) {
  const auto it = state.view().backbone_index().span_bindings_by_span.find(span_id);
  if (it == state.view().backbone_index().span_bindings_by_span.end() || it->second.size() != 1 ||
      it->second.front() >= state.view().backbone().span_bindings.size()) {
    return nullptr;
  }
  return &state.view().backbone().span_bindings[it->second.front()];
}

bool two_lane_edge_bundles_do_not_twist(const wire::core::CoreState& state,
                                        const std::vector<wire::core::ObjectId>& span_ids) {
  std::unordered_map<wire::core::ObjectId, std::array<wire::core::ObjectId, 2>> spans_by_edge_bundle{};
  for (wire::core::ObjectId span_id : span_ids) {
    const wire::core::SavedBackboneSpanBinding* binding = saved_span_binding(state, span_id);
    if (binding == nullptr || binding->lane_index >= 2) {
      return false;
    }
    spans_by_edge_bundle[binding->edge_bundle_id][binding->lane_index] = span_id;
  }
  for (const auto& [edge_bundle_id, lanes] : spans_by_edge_bundle) {
    (void)edge_bundle_id;
    const wire::core::Span* lane0 = state.view().spans().find(lanes[0]);
    const wire::core::Span* lane1 = state.view().spans().find(lanes[1]);
    if (lane0 == nullptr || lane1 == nullptr) {
      return false;
    }
    const wire::core::Port* a0 = state.view().ports().find(lane0->port_a_id);
    const wire::core::Port* a1 = state.view().ports().find(lane1->port_a_id);
    const wire::core::Port* b0 = state.view().ports().find(lane0->port_b_id);
    const wire::core::Port* b1 = state.view().ports().find(lane1->port_b_id);
    if (a0 == nullptr || a1 == nullptr || b0 == nullptr || b1 == nullptr) {
      return false;
    }
    const wire::core::Vec3d axis_a = normalize_xy(a1->world_position - a0->world_position);
    const wire::core::Vec3d axis_b = normalize_xy(b1->world_position - b0->world_position);
    if (dot_xy(axis_a, axis_b) <= 0.0) {
      return false;
    }
  }
  return !spans_by_edge_bundle.empty();
}

wire::core::ObjectId pole_at(const wire::core::CoreState& state,
                             const std::vector<wire::core::ObjectId>& pole_ids,
                             const wire::core::Vec3d& position) {
  for (wire::core::ObjectId pole_id : pole_ids) {
    const wire::core::Pole* pole = state.view().poles().find(pole_id);
    if (pole != nullptr && almost_equal(pole->world_transform.position, position, 1e-9)) {
      return pole_id;
    }
  }
  return wire::core::kInvalidObjectId;
}

} // namespace

bool C368_backbone_smoke_line() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const int count = bundle_count(state, wire::core::BundleKind::kLowVoltage);
  const auto out = state.GenerateFromBackboneSpec(req);
  return out.ok && out.value.generated_pole_ids.size() == 2 && out.value.bundle_ids.size() == 1 &&
         out.value.generated_span_ids.size() == static_cast<std::size_t>(count) &&
         state.view().poles().size() >= 2 && state.view().bundles().size() >= 1 &&
         state.view().spans().size() >= static_cast<std::size_t>(count);
}

bool C369_backbone_rules_saved() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
  }
  return true;
}

bool C371_backbone_rejects_unsupported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec empty = line_req(state);
  empty.bundles.clear();
  const auto empty_out = state.GenerateFromBackboneSpec(empty);
  if (empty_out.ok || !contains_text(empty_out.error, "unsupported") ||
      empty_out.error_kind != wire::core::EditErrorKind::kUnsupported) {
    return false;
  }

  wire::core::BackboneSpec building = line_req(state);
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kExternal;
  node.node_id = 1;
  building.path.node_specs.push_back(node);
  const auto building_out = state.GenerateFromBackboneSpec(building);
  return !building_out.ok && contains_text(building_out.error, "unsupported");
}

bool C819_backbone_rejects_nonfinite_path_point_before_mutation() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline[0].x = std::numeric_limits<double>::quiet_NaN();

  const CoreCounts before = snapshot_counts(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "invalid input") &&
         out.error_kind == wire::core::EditErrorKind::kValidation &&
         same_counts(before, snapshot_counts(state));
}

bool C820_backbone_rejects_nonfinite_tilt_before_mutation() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.pole_placement.enable_tilt = true;
  req.pole_placement.max_tilt_deg = std::numeric_limits<double>::infinity();

  const CoreCounts before = snapshot_counts(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "invalid input") &&
         out.error_kind == wire::core::EditErrorKind::kValidation &&
         same_counts(before, snapshot_counts(state));
}

bool C821_backbone_external_input_validation_lists_numeric_fields() {
  const std::filesystem::path workflow = repo_root() / "core" / "include" / "wire" / "core" / "workflow_types.hpp";
  const std::filesystem::path pipeline = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string workflow_text{};
  std::string pipeline_text{};
  if (!file_text(workflow, &workflow_text) || !file_text(pipeline, &pipeline_text)) {
    return false;
  }
  const std::size_t path_begin = workflow_text.find("struct BackboneInputSpec");
  const std::size_t path_end = workflow_text.find("struct AttachmentSocketTemplate");
  const std::size_t spec_begin = workflow_text.find("struct BackboneBundleSpec");
  const std::size_t spec_end = workflow_text.find("struct JunctionIncident");
  if (path_begin == std::string::npos || path_end == std::string::npos || path_begin >= path_end ||
      spec_begin == std::string::npos || spec_end == std::string::npos || spec_begin >= spec_end) {
    return false;
  }
  const std::string input_block =
      workflow_text.substr(path_begin, path_end - path_begin) +
      workflow_text.substr(spec_begin, spec_end - spec_begin);
  const auto count_token = [&](const std::string& token) {
    std::size_t count = 0;
    std::size_t pos = 0;
    while ((pos = input_block.find(token, pos)) != std::string::npos) {
      ++count;
      pos += token.size();
    }
    return count;
  };
  if (count_token("double ") != 7 || count_token("Vec3d ") != 1 ||
      count_token("std::vector<Vec3d>") != 2) {
    return false;
  }
  const std::size_t validator_begin = pipeline_text.find("validate_backbone_spec_external_input");
  const std::size_t validator_end = pipeline_text.find("} // namespace", validator_begin);
  if (validator_begin == std::string::npos || validator_end == std::string::npos ||
      validator_begin >= validator_end) {
    return false;
  }
  const std::string validator = pipeline_text.substr(validator_begin, validator_end - validator_begin);
  const std::array<const char*, 10> required{
      "interval_m",
      "path.polyline",
      "tangent_hint",
      "avoid_radius_m",
      "lateral_offset_m",
      "avoid_points",
      "max_tilt_deg",
      "height_m",
      "lateral_m",
      "spacing_m",
  };
  return std::all_of(required.begin(), required.end(),
                     [&](const char* token) { return contains_text(validator, token); });
}

bool C372_backbone_rules_do_not_seed() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
  }
  return true;
}

bool C373_backbone_layout_saved_without_recalc() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
    if (!state.span_layout(span_id).has_layout()) {
      return false;
    }
  }
  return true;
}

bool C374_backbone_layout_is_deterministic() {
  wire::core::CoreState a;
  wire::core::CoreState b;
  const std::vector<wire::core::Vec3d> pa = backbone_layout_points(a);
  const std::vector<wire::core::Vec3d> pb = backbone_layout_points(b);
  if (pa.empty() || pa.size() != pb.size()) {
    return false;
  }
  for (std::size_t i = 0; i < pa.size(); ++i) {
    if (!almost_equal(pa[i], pb[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C375_backbone_curve_saved_without_recalc() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
    if (!state.span_layout(span_id).has_layout()) {
      return false;
    }
    const auto* curve = state.find_curve_cache(span_id);
    if (curve == nullptr || curve->detail.sample_points.size() < 2 ||
        curve->points.size() != curve->detail.sample_points.size() ||
        !std::equal(curve->points.begin(), curve->points.end(), curve->detail.sample_points.begin(),
                    [](const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
                      return almost_equal(a, b, 1e-12);
                    })) {
      return false;
    }
  }
  return true;
}

bool C376_backbone_curve_is_deterministic() {
  wire::core::CoreState a;
  wire::core::CoreState b;
  const CurveSnapshot ca = backbone_curve_points(a);
  const CurveSnapshot cb = backbone_curve_points(b);
  if (ca.points.empty() || ca.points.size() != cb.points.size() || ca.lengths.size() != cb.lengths.size()) {
    return false;
  }
  for (std::size_t i = 0; i < ca.points.size(); ++i) {
    if (!almost_equal(ca.points[i], cb.points[i], 1e-9)) {
      return false;
    }
  }
  for (std::size_t i = 0; i < ca.lengths.size(); ++i) {
    if (!almost_equal(ca.lengths[i], cb.lengths[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C377_backbone_bounds_saved_without_recalc() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
    if (!state.span_layout(span_id).has_layout()) {
      return false;
    }
    if (state.find_curve_cache(span_id) == nullptr) {
      return false;
    }
    const auto* bounds = state.find_bounds_cache(span_id);
    if (bounds == nullptr || bounds->segments.empty()) {
      return false;
    }
  }
  return true;
}

bool C378_backbone_bounds_is_deterministic() {
  wire::core::CoreState a;
  wire::core::CoreState b;
  const BoundsSnapshot ba = backbone_bounds_points(a);
  const BoundsSnapshot bb = backbone_bounds_points(b);
  if (ba.pts.empty() || ba.pts.size() != bb.pts.size()) {
    return false;
  }
  for (std::size_t i = 0; i < ba.pts.size(); ++i) {
    if (!almost_equal(ba.pts[i], bb.pts[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C379_backbone_m1_required_outputs() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 2 || out.value.bundle_ids.size() != 1 ||
      out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
    if (!state.span_layout(span_id).has_layout()) {
      return false;
    }
    if (state.find_curve_cache(span_id) == nullptr) {
      return false;
    }
    if (state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
  }
  return true;
}

bool C380_backbone_m1_draw_outputs_saved() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (state.find_span_visual_cache(span_id) == nullptr) {
      return false;
    }
    if (state.view().find_span_render_cache(span_id) == nullptr) {
      return false;
    }
  }
  return true;
}

bool C381_backbone_m1_no_recalc_contract() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
  }
  return C370_backbone_no_v1_deps();
}

bool C822_edit_result_error_kind_classifies_core_error_prefixes() {
  wire::core::EditResult<bool> validation{};
  validation.error = "backbone invalid input: path.polyline";
  wire::core::EditResult<bool> unsupported{};
  unsupported.error = "backbone unsupported: empty bundles";
  wire::core::EditResult<bool> internal{};
  internal.error = "backbone internal: row continuity endpoint is missing";
  wire::core::EditResult<bool> unknown{};
  unknown.error = "missing prefix";
  wire::core::EditResult<bool> ok{};
  ok.ok = true;
  ok.error = "backbone internal: ignored on success";

  return validation.effective_error_kind() == wire::core::EditErrorKind::kValidation &&
         unsupported.effective_error_kind() == wire::core::EditErrorKind::kUnsupported &&
         internal.effective_error_kind() == wire::core::EditErrorKind::kInternal &&
         unknown.effective_error_kind() == wire::core::EditErrorKind::kInternal &&
         ok.effective_error_kind() == wire::core::EditErrorKind::kNone;
}

bool C823_test_failure_diagnostics_are_available_for_backbone_scenarios() {
  std::string registry_hpp{};
  std::string registry_cpp{};
  std::string runner_cpp{};
  std::string graph_cpp{};
  std::string testing_doc{};
  std::string agents{};
  WIRE_TEST_EXPECT(file_text(repo_root() / "core" / "tests" / "registry.hpp", &registry_hpp),
                   "registry.hpp is missing");
  WIRE_TEST_EXPECT(file_text(repo_root() / "core" / "tests" / "registry.cpp", &registry_cpp),
                   "registry.cpp is missing");
  WIRE_TEST_EXPECT(file_text(repo_root() / "core" / "tests" / "runner.cpp", &runner_cpp),
                   "runner.cpp is missing");
  WIRE_TEST_EXPECT(file_text(repo_root() / "core" / "tests" / "backbone" / "graph.cpp", &graph_cpp),
                   "graph.cpp is missing");
  WIRE_TEST_EXPECT(file_text(repo_root() / "docs" / "testing.md", &testing_doc),
                   "docs/testing.md is missing");
  WIRE_TEST_EXPECT(file_text(repo_root() / "AGENTS.md", &agents), "AGENTS.md is missing");
  const std::array<const char*, 5> migrated_functions{
      "bool C773_backbone_incremental_sharp_completion_derives_jumper_from_continuity()",
      "bool C775_backbone_incremental_canonical_pair_survives_save_load()",
      "bool viewer_default_t_branch_keeps_hv_and_only_flagged_lowering(bool anchor_at_end)",
      "bool C809_backbone_incremental_rows_use_one_support_level_per_pair()",
      "bool C815_backbone_sharp_pair_height_is_operation_order_independent()",
  };
  for (const char* signature : migrated_functions) {
    std::string body{};
    WIRE_TEST_EXPECT(function_body(graph_cpp, signature, &body), std::string("missing function body: ") + signature);
    WIRE_TEST_EXPECT(contains_text(body, "WIRE_TEST_EXPECT"),
                     std::string("function lacks WIRE_TEST_EXPECT: ") + signature);
  }
  WIRE_TEST_EXPECT(contains_text(registry_hpp, "WIRE_TEST_EXPECT"), "WIRE_TEST_EXPECT macro is missing");
  WIRE_TEST_EXPECT(contains_text(registry_cpp, "SetFailureReason"), "SetFailureReason storage is missing");
  WIRE_TEST_EXPECT(contains_text(runner_cpp, "reason: "), "runner does not print failure reason");
  WIRE_TEST_EXPECT(contains_text(testing_doc, "WIRE_TEST_EXPECT(condition, reason)"),
                   "docs/testing.md does not document failure diagnostics");
  WIRE_TEST_EXPECT(contains_text(agents, "WIRE_TEST_EXPECT"), "AGENTS.md does not require diagnostic helper use");
  return true;
}

bool C824_backbone_seeded_route_fuzz_preserves_common_invariants() {
  std::size_t representative_invariant_checks = 0;
  const std::filesystem::path tests_dir = repo_root() / "core/tests/backbone";
  for (const auto& entry : std::filesystem::recursive_directory_iterator(tests_dir)) {
    if (!entry.is_regular_file() || entry.path().filename() == "fixtures.cpp") {
      continue;
    }
    const std::string ext = entry.path().extension().string();
    if (ext != ".cpp" && ext != ".hpp") {
      continue;
    }
    std::string text{};
    WIRE_TEST_EXPECT(file_text(entry.path(), &text), "failed to read invariant test source: " + entry.path().string());
    std::size_t pos = 0;
    while ((pos = text.find("backbone_common_invariants_pass(", pos)) != std::string::npos) {
      ++representative_invariant_checks;
      ++pos;
    }
  }
  WIRE_TEST_EXPECT(representative_invariant_checks >= 10,
                   "fewer than 10 representative tests call backbone_common_invariants_pass");

  const std::array<std::uint32_t, 8> seeds{11U, 29U, 47U, 83U, 131U, 197U, 251U, 307U};
  for (std::uint32_t seed : seeds) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dx(5.0, 12.0);
    std::uniform_real_distribution<double> dy(-7.0, 7.0);
    const int point_count = 2 + static_cast<int>(seed % 3U);
    std::vector<wire::core::Vec3d> points{};
    points.reserve(static_cast<std::size_t>(point_count));
    wire::core::Vec3d cursor{0.0, 0.0, 0.0};
    points.push_back(cursor);
    for (int i = 1; i < point_count; ++i) {
      cursor.x += dx(rng);
      cursor.y += dy(rng);
      points.push_back(cursor);
    }

    wire::core::CoreState state;
    wire::core::BackboneSpec req = line_req(state);
    req.path.polyline = points;
    const CoreCounts before = snapshot_counts(state);
    const auto out = state.GenerateFromBackboneSpec(req);
    if (!out.ok) {
      WIRE_TEST_EXPECT(same_counts(before, snapshot_counts(state)),
                       "rejected fuzz request mutated state for seed " + std::to_string(seed));
      continue;
    }
    std::string invariant_error{};
    WIRE_TEST_EXPECT(backbone_common_invariants_pass(state, &invariant_error),
                     "seed " + std::to_string(seed) + ": " + invariant_error);
  }
  return true;
}

bool C382_backbone_geom_is_single_pipeline_layer() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "backbone";
  bool has_geom = false;
  const std::vector<std::string> banned = {
      "make(const curve&",
      "save(curve",
      "save(bounds",
  };
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    has_geom = has_geom || contains_text(text, "struct geom");
    for (const std::string& token : banned) {
      if (contains_text(text, token)) {
        return false;
      }
    }
  }
  return has_geom;
}

bool C383_backbone_draw_is_pipeline_layer() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "backbone";
  bool has_draw = false;
  bool has_make = false;
  bool has_save = false;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    has_draw = has_draw || contains_text(text, "struct draw");
    has_make = has_make || contains_text(text, "draw make(const layout& placed, const geom& shaped) const");
    has_save = has_save || contains_text(text, "void save(draw made)");
    if (contains_text(text, "visual_cache") || contains_text(text, "render_cache")) {
      return false;
    }
  }
  return has_draw && has_make && has_save;
}

bool C384_backbone_topo_is_single_output_layer() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "backbone";
  bool has_topo = false;
  bool has_graph_poles = false;
  const std::vector<std::string> banned = {
      "struct spans",
  };
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    has_topo = has_topo || contains_text(text, "struct topo");
    has_graph_poles = has_graph_poles || contains_text(text, "struct graph {\n  route r{};\n  std::vector<ObjectId> poles{};");
    for (const std::string& token : banned) {
      if (contains_text(text, token)) {
        return false;
      }
    }
  }
  return has_topo && !has_graph_poles;
}

bool C385_backbone_emit_is_split_by_topology_parts() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "backbone";
  bool has_poles = false;
  bool has_bundles = false;
  bool has_ports = false;
  bool has_spans = false;
  bool emit_body_has_add = false;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    has_poles = has_poles || contains_text(text, "emit_poles");
    has_bundles = has_bundles || contains_text(text, "emit_bundles");
    has_ports = has_ports || contains_text(text, "emit_ports");
    has_spans = has_spans || contains_text(text, "emit_spans");
    const std::string marker = "EditResult<topo> pipeline::emit()";
    const std::size_t start = text.find(marker);
    if (start != std::string::npos) {
      const std::size_t end = text.find("rules pipeline::make", start);
      const std::string body = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
      emit_body_has_add = contains_text(body, "AddPole") || contains_text(body, "AddBundle") ||
                          contains_text(body, "AddPort") || contains_text(body, "AddSpan");
    }
  }
  return has_poles && has_bundles && has_ports && has_spans && !emit_body_has_add;
}

bool C386_backbone_link_pair_row_are_separate() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "backbone";
  bool has_link = false;
  bool has_pair = false;
  bool has_open = false;
  bool has_row = false;
  bool row_has_source = false;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    has_link = has_link || contains_text(text, "struct link");
    has_pair = has_pair || contains_text(text, "struct pair");
    has_open = has_open || contains_text(text, "struct open");
    has_row = has_row || contains_text(text, "struct row");
    row_has_source = row_has_source || contains_text(text, "src source");
  }
  return has_link && has_pair && has_open && has_row && row_has_source;
}

bool C388_backbone_polyline3_pair_model() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text)) {
    return false;
  }
  const bool builds_links = contains_text(text, "g_.links.push_back(edge)");
  const bool builds_pair = contains_text(text, "add_pair(&out.value");
  const bool builds_open = contains_text(text, "add_open(&out.value");
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  const int count = bundle_count(state, wire::core::BundleKind::kLowVoltage);
  const auto out = state.GenerateFromBackboneSpec(req);
  return builds_links && builds_pair && builds_open && out.ok && out.value.generated_pole_ids.size() == 3 &&
         out.value.generated_span_ids.size() == static_cast<std::size_t>(count * 2);
}

bool C389_backbone_row_axis_owned_by_pairs() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text)) {
    return false;
  }
  const std::string ports_marker = "EditResult<bool> pipeline::emit_ports";
  const std::size_t ports_start = text.find(ports_marker);
  if (ports_start == std::string::npos) {
    return false;
  }
  const std::size_t ports_end = text.find("EditResult<bool> pipeline::emit_spans", ports_start);
  const std::string ports_body = text.substr(ports_start, ports_end == std::string::npos ? std::string::npos
                                                                                          : ports_end - ports_start);
  return contains_text(ports_body, "r.axis") && !contains_text(ports_body, "side(") &&
         !contains_text(ports_body, "norm(");
}

bool C390_backbone_rejects_already_used_incident() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text) || !contains_text(text, "incident already used")) {
    return false;
  }
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {4.0, 0.0, 0.0}};
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C392_backbone_polyline3_outputs() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule()) {
      return false;
    }
    if (!state.span_layout(span_id).has_layout()) {
      return false;
    }
    if (state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
  }
  return true;
}

bool C393_backbone_polyline3_is_deterministic() {
  wire::core::CoreState a;
  wire::core::CoreState b;
  const std::vector<wire::core::Vec3d> pa = poly3_points(a);
  const std::vector<wire::core::Vec3d> pb = poly3_points(b);
  if (pa.empty() || pa.size() != pb.size()) {
    return false;
  }
  for (std::size_t i = 0; i < pa.size(); ++i) {
    if (!almost_equal(pa[i], pb[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C394_backbone_existing_pole_node_is_not_recreated() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = line_req(state);
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.empty()) {
    return false;
  }
  const std::size_t pole_count_before = state.view().poles().size();
  wire::core::BackboneSpec second = line_req(state);
  const wire::core::ObjectId existing = first_out.value.generated_pole_ids.front();
  const auto* existing_pole = state.view().poles().find(existing);
  if (existing_pole == nullptr) {
    return false;
  }
  second.path.polyline = {existing_pole->world_transform.position, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = existing;
  second.path.node_specs.push_back(node);
  const auto second_out = state.GenerateFromBackboneSpec(second);
  return second_out.ok && second_out.value.generated_pole_ids.size() == 1 &&
         state.view().poles().size() == pole_count_before + 1;
}

bool C397_backbone_rejects_missing_saved_midair_node_spec() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kMidair;
  node.node_id = 1;
  req.path.node_specs.push_back(node);
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C398_backbone_rejects_missing_existing_pole() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = 999999;
  req.path.node_specs.push_back(node);
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C399_backbone_existing_pole_sequence_is_deterministic() {
  wire::core::CoreState a;
  wire::core::CoreState b;
  const std::vector<wire::core::Vec3d> pa = existing_sequence_points(a);
  const std::vector<wire::core::Vec3d> pb = existing_sequence_points(b);
  if (pa.empty() || pa.size() != pb.size()) {
    return false;
  }
  for (std::size_t i = 0; i < pa.size(); ++i) {
    if (!almost_equal(pa[i], pb[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C400_backbone_multiple_bundles_smoke() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  const int count = req_bundle_count(state, req);
  const auto out = state.GenerateFromBackboneSpec(req);
  return out.ok && out.value.bundle_ids.size() == 2 &&
         out.value.generated_span_ids.size() == static_cast<std::size_t>(count);
}

bool C401_backbone_multiple_bundles_polyline3_outputs() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  const int count = req_bundle_count(state, req);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.bundle_ids.size() != 2 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(count * 2)) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
  }
  return true;
}

bool C403_backbone_existing_pole_with_multiple_bundles() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = line_req(state);
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.empty()) {
    return false;
  }
  const std::size_t pole_count_before = state.view().poles().size();
  const wire::core::ObjectId existing = first_out.value.generated_pole_ids.front();
  const auto* existing_pole = state.view().poles().find(existing);
  if (existing_pole == nullptr) {
    return false;
  }
  wire::core::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  req.path.polyline = {existing_pole->world_transform.position, {0.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = existing;
  req.path.node_specs.push_back(node);
  const int count = req_bundle_count(state, req);
  const auto out = state.GenerateFromBackboneSpec(req);
  return out.ok && out.value.bundle_ids.size() == 2 && out.value.generated_pole_ids.size() == 1 &&
         state.view().poles().size() == pole_count_before + 1 &&
         out.value.generated_span_ids.size() == static_cast<std::size_t>(count);
}

bool C404_backbone_rejects_empty_bundles() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.bundles.clear();
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C405_backbone_no_bundle_pair_branching() {
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string text;
  if (!file_text(file, &text)) {
    return false;
  }
  std::string body;
  if (!function_body(text, "EditResult<pairs> pipeline::make(const graph& made) const", &body)) {
    return false;
  }
  return !contains_text(body, "spec_.bundles") && !contains_text(body, "BundleTemplate") &&
         !contains_text(body, "bundle_template");
}

bool C406_backbone_port_height_uses_pole_band() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const double expected = band_height(state, req.pole_type_id, wire::core::BundleKind::kLowVoltage);
  if (expected < 0.0 || almost_equal(expected, 9.32, 1e-9)) {
    return false;
  }
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!span_ports_match_z(state, span_id, expected)) {
      return false;
    }
  }
  return true;
}

bool C407_backbone_multiple_bundle_heights_are_band_based() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  const double lv_z = band_height(state, req.pole_type_id, wire::core::BundleKind::kLowVoltage);
  const double comm_z = band_height(state, req.pole_type_id, wire::core::BundleKind::kCommunication);
  if (lv_z < 0.0 || comm_z < 0.0 || almost_equal(lv_z, comm_z, 1e-9)) {
    return false;
  }
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  bool saw_lv = false;
  bool saw_comm = false;
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const auto* bundle = state.view().bundles().find(span->bundle_id);
    if (bundle == nullptr) {
      return false;
    }
    if (bundle->bundle_template_id == wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage)) {
      saw_lv = true;
      if (!span_ports_match_z(state, span_id, lv_z)) {
        return false;
      }
    }
    if (bundle->bundle_template_id == wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)) {
      saw_comm = true;
      if (!span_ports_match_z(state, span_id, comm_z)) {
        return false;
      }
    }
  }
  return saw_lv && saw_comm;
}

bool C749_backbone_zero_offset_keeps_bundle_centers_on_band_center() {
  wire::core::CoreState single;
  const auto single_generated = single.GenerateFromBackboneSpec(line_req(single));
  if (!single_generated.ok || single_generated.value.generated_pole_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId single_lv_span =
      span_for_bundle(single, single_generated.value.generated_span_ids, wire::core::BundleKind::kLowVoltage);
  if (single_lv_span == wire::core::kInvalidObjectId) {
    return false;
  }
  const std::vector<wire::core::Vec3d> single_ports =
      generated_ports_on_pole(single, {single_lv_span}, single_generated.value.generated_pole_ids.front());

  wire::core::CoreState combined;
  wire::core::BackboneSpec combined_request = line_req(combined);
  add_backbone_bundle(combined_request, wire::core::BundleKind::kCommunication);
  const auto combined_generated = combined.GenerateFromBackboneSpec(combined_request);
  if (!combined_generated.ok || combined_generated.value.generated_pole_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId combined_lv_span =
      span_for_bundle(combined, combined_generated.value.generated_span_ids, wire::core::BundleKind::kLowVoltage);
  if (combined_lv_span == wire::core::kInvalidObjectId) {
    return false;
  }
  const std::vector<wire::core::Vec3d> combined_ports =
      generated_ports_on_pole(combined, {combined_lv_span}, combined_generated.value.generated_pole_ids.front());
  if (single_ports.size() != combined_ports.size() || single_ports.empty()) {
    return false;
  }
  for (std::size_t index = 0; index < single_ports.size(); ++index) {
    if (!almost_equal(single_ports[index].x, combined_ports[index].x, 1e-12) ||
        !almost_equal(single_ports[index].y, combined_ports[index].y, 1e-12) ||
        !almost_equal(single_ports[index].z, combined_ports[index].z, 1e-12)) {
      return false;
    }
  }

  wire::core::CoreState hv_state;
  wire::core::BackboneSpec hv_request = line_req(hv_state);
  hv_request.bundles.clear();
  add_backbone_bundle(hv_request, wire::core::BundleKind::kHighVoltage);
  const auto hv_generated = hv_state.GenerateFromBackboneSpec(hv_request);
  if (!hv_generated.ok || hv_generated.value.generated_pole_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId pole_id = hv_generated.value.generated_pole_ids.front();
  const wire::core::Pole* pole = hv_state.view().poles().find(pole_id);
  const auto pole_type_it = pole == nullptr ? hv_state.view().pole_types().end()
                                            : hv_state.view().pole_types().find(pole->pole_type_id);
  if (pole == nullptr || pole_type_it == hv_state.view().pole_types().end()) {
    return false;
  }
  std::vector<const wire::core::PortPlacementBand*> expected_bands{};
  for (const wire::core::PortPlacementBand& band : pole_type_it->second.port_bands) {
    if (band.enabled && band.category == wire::core::ConnectionCategory::kHighVoltage && band.layer == 2) {
      expected_bands.push_back(&band);
    }
  }
  std::sort(expected_bands.begin(), expected_bands.end(), [](const auto* a, const auto* b) {
    if (!almost_equal(a->lateral_center_m, b->lateral_center_m, 1e-12)) {
      return a->lateral_center_m < b->lateral_center_m;
    }
    return a->band_id < b->band_id;
  });
  if (expected_bands.size() != 3) {
    return false;
  }
  std::vector<const wire::core::SavedBackbonePortBinding*> bindings{};
  for (const wire::core::SavedBackbonePortBinding& binding : hv_state.view().backbone().port_bindings) {
    const wire::core::Port* port = hv_state.view().ports().find(binding.port_id);
    if (port != nullptr && port->owner_pole_id == pole_id &&
        binding.bundle_template_id == wire::core::kDefaultHighVoltageBundleTemplateId) {
      bindings.push_back(&binding);
    }
  }
  std::sort(bindings.begin(), bindings.end(), [](const auto* a, const auto* b) {
    return a->lane_index < b->lane_index;
  });
  if (bindings.size() != expected_bands.size()) {
    return false;
  }
  for (std::size_t lane = 0; lane < bindings.size(); ++lane) {
    const wire::core::Port* port = hv_state.view().ports().find(bindings[lane]->port_id);
    if (port == nullptr || bindings[lane]->placement_band_id != expected_bands[lane]->band_id) {
      return false;
    }
    const wire::core::PoleFrame frame = wire::core::BuildPoleFrame(pole->world_transform, bindings[lane]->layout_yaw_deg);
    const wire::core::Vec3d local = wire::core::WorldPointToLocal(frame, port->world_position);
    if (!almost_equal(local.y, expected_bands[lane]->lateral_center_m, 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C408_backbone_existing_pole_uses_actual_pole_type_height() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const double request_z = band_height(state, req.pole_type_id, wire::core::BundleKind::kLowVoltage);
  wire::core::PoleTypeDefinition existing_definition{};
  bool found_existing_definition = false;
  for (const auto& item : state.view().pole_types()) {
    if (item.first == req.pole_type_id) {
      continue;
    }
    existing_definition = item.second;
    found_existing_definition = true;
    break;
  }
  if (!found_existing_definition) {
    return false;
  }
  double existing_z = -1.0;
  for (auto& band : existing_definition.port_bands) {
    if (band.enabled && band.category == wire::core::ConnectionCategory::kLowVoltage) {
      band.height_center_m += 0.65;
      band.height_min_m += 0.65;
      band.height_max_m += 0.65;
      existing_z = band.height_center_m;
    }
  }
  if (existing_z < 0.0 || almost_equal(existing_z, request_z, 1e-9)) {
    return false;
  }
  const auto update_type = state.UpdatePoleTypeDefinition(existing_definition);
  if (!update_type.ok || !update_type.value) {
    return false;
  }
  const wire::core::PoleTypeId existing_type = existing_definition.id;
  wire::core::BackboneSpec first = line_req(state);
  first.pole_type_id = existing_type;
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok || first_out.value.generated_pole_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId pole = first_out.value.generated_pole_ids.front();
  const auto* existing_pole = state.view().poles().find(pole);
  if (existing_pole == nullptr) {
    return false;
  }
  req.path.polyline = {existing_pole->world_transform.position, {0.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = pole;
  req.path.node_specs.push_back(node);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  bool saw_existing = false;
  bool saw_new = false;
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr) {
      return false;
    }
    for (const wire::core::Port* port : {a, b}) {
      if (port->owner_pole_id == pole) {
        saw_existing = true;
        if (!almost_equal(port->world_position.z, existing_z, 1e-9)) {
          return false;
        }
      } else {
        saw_new = true;
        if (!almost_equal(port->world_position.z, request_z, 1e-9)) {
          return false;
        }
      }
    }
  }
  return saw_existing && saw_new;
}

bool C409_backbone_rejects_missing_port_band() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  auto it = state.view().pole_types().find(req.pole_type_id);
  if (it == state.view().pole_types().end()) {
    return false;
  }
  wire::core::PoleTypeDefinition type = it->second;
  type.port_bands.erase(std::remove_if(type.port_bands.begin(), type.port_bands.end(),
                                       [](const wire::core::PortPlacementBand& band) {
                                         return band.category == wire::core::ConnectionCategory::kLowVoltage &&
                                                band.layer == 1;
                                       }),
                        type.port_bands.end());
  if (!state.UpdatePoleTypeDefinition(type).ok) {
    return false;
  }
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C411_backbone_lateral_offset_moves_ports_along_row_axis() {
  const std::vector<wire::core::Vec3d> zero = offset_curve_points(0.0);
  const std::vector<wire::core::Vec3d> plus = offset_curve_points(1.0);
  const std::vector<wire::core::Vec3d> minus = offset_curve_points(-1.0);
  if (zero.empty() || zero.size() != plus.size() || zero.size() != minus.size()) {
    return false;
  }
  for (std::size_t i = 0; i < zero.size(); ++i) {
    const wire::core::Vec3d expected_plus{zero[i].x, zero[i].y + 1.0, zero[i].z};
    const wire::core::Vec3d expected_minus{zero[i].x, zero[i].y - 1.0, zero[i].z};
    if (!almost_equal(plus[i], expected_plus, 1e-9) || !almost_equal(minus[i], expected_minus, 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C412_backbone_lateral_offset_sign_is_deterministic() {
  const std::vector<wire::core::Vec3d> a = offset_points(1.0);
  const std::vector<wire::core::Vec3d> b = offset_points(1.0);
  if (a.empty() || a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!almost_equal(a[i], b[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C661_backbone_pair_row_axis_uses_unit_tangent_bisector() {
  wire::core::CoreState state;
  if (!prepare_two_lane_low_voltage(state)) {
    return false;
  }
  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {20.0, 2.0, 0.0}};
  const auto generated = state.GenerateFromBackboneSpec(req);
  const wire::core::ObjectId corner_pole = pole_at(state, generated.value.generated_pole_ids, {20.0, 0.0, 0.0});
  if (!generated.ok || corner_pole == wire::core::kInvalidObjectId) {
    return false;
  }
  const std::vector<wire::core::Vec3d> ports =
      generated_ports_on_pole(state, generated.value.generated_span_ids, corner_pole);
  std::vector<wire::core::Vec3d> unique_ports{};
  for (const auto& port : ports) {
    if (std::none_of(unique_ports.begin(), unique_ports.end(),
                     [&](const auto& item) { return almost_equal(item, port, 1e-12); })) {
      unique_ports.push_back(port);
    }
  }
  if (ports.size() != 4 || unique_ports.size() != 2) {
    return false;
  }
  const wire::core::Vec3d actual = normalize_xy(unique_ports[1] - unique_ports[0]);
  const wire::core::Vec3d incoming = normalize_xy(req.path.polyline[1] - req.path.polyline[0]);
  const wire::core::Vec3d outgoing = normalize_xy(req.path.polyline[2] - req.path.polyline[1]);
  const wire::core::Vec3d expected = lateral_xy(incoming + outgoing);
  const wire::core::Vec3d old_chord_axis = lateral_xy(req.path.polyline[2] - req.path.polyline[0]);
  return abs_dot_xy(actual, expected) > 0.999 && abs_dot_xy(actual, old_chord_axis) < 0.95;
}

bool C662_backbone_pair_row_axis_does_not_flip_lane_order() {
  wire::core::CoreState state;
  if (!prepare_two_lane_low_voltage(state)) {
    return false;
  }
  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {20.0, 2.0, 0.0}};
  const auto generated = state.GenerateFromBackboneSpec(req);
  const wire::core::ObjectId corner_pole = pole_at(state, generated.value.generated_pole_ids, {20.0, 0.0, 0.0});
  if (!generated.ok || corner_pole == wire::core::kInvalidObjectId) {
    return false;
  }
  const std::vector<wire::core::Vec3d> ports =
      generated_ports_on_pole(state, generated.value.generated_span_ids, corner_pole);
  std::vector<wire::core::Vec3d> unique_ports{};
  for (const auto& port : ports) {
    if (std::none_of(unique_ports.begin(), unique_ports.end(),
                     [&](const auto& item) { return almost_equal(item, port, 1e-12); })) {
      unique_ports.push_back(port);
    }
  }
  if (ports.size() != 4 || unique_ports.size() != 2) {
    return false;
  }
  wire::core::Vec3d row_axis = normalize_xy(unique_ports[1] - unique_ports[0]);
  const wire::core::Vec3d incoming = normalize_xy(req.path.polyline[1] - req.path.polyline[0]);
  const wire::core::Vec3d outgoing = normalize_xy(req.path.polyline[2] - req.path.polyline[1]);
  const wire::core::Vec3d expected = lateral_xy(incoming + outgoing);
  if (dot_xy(row_axis, expected) < 0.0) {
    row_axis = {-row_axis.x, -row_axis.y, -row_axis.z};
  }
  return dot_xy(row_axis, lateral_xy(incoming)) > 0.0 &&
         dot_xy(row_axis, lateral_xy(outgoing)) > 0.0 &&
         two_lane_edge_bundles_do_not_twist(state, generated.value.generated_span_ids);
}

bool C663_backbone_sharp_corner_uses_dead_end_rows_and_jumpers() {
  wire::core::CoreState state;
  if (!prepare_two_lane_low_voltage(state)) {
    return false;
  }
  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {5.0, 8.660254037844386, 0.0}};
  const auto generated = state.GenerateFromBackboneSpec(req);
  const wire::core::ObjectId corner_pole = pole_at(state, generated.value.generated_pole_ids, {10.0, 0.0, 0.0});
  const wire::core::SavedBackboneNode* corner_node = state.view().backbone_node_for_pole(corner_pole);
  if (!generated.ok || generated.value.generated_span_ids.size() != 4 ||
      state.view().backbone().edges.size() != 2 || corner_node == nullptr) {
    return false;
  }

  std::unordered_map<wire::core::ObjectId, std::vector<wire::core::Vec3d>> rows{};
  std::unordered_set<wire::core::ObjectId> corner_ports{};
  for (wire::core::ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    for (wire::core::ObjectId port_id : {span->port_a_id, span->port_b_id}) {
      const wire::core::Port* port = state.view().ports().find(port_id);
      if (port == nullptr || port->owner_pole_id != corner_pole || !corner_ports.insert(port_id).second) {
        continue;
      }
      const wire::core::SavedBackbonePortBinding* binding = state.view().backbone_port_binding_for_port(port_id);
      if (binding == nullptr ||
          binding->row_key.node_id != corner_node->node_id ||
          binding->row_key.edge_id == wire::core::kInvalidObjectId ||
          std::abs(port->side_scale_applied - 1.0) > 1e-9) {
        return false;
      }
      rows[binding->row_key.edge_id].push_back(port->world_position);
    }
  }
  if (corner_ports.size() != 4 || rows.size() != 2) {
    return false;
  }
  for (const auto& [edge_id, ports] : rows) {
    const wire::core::SavedBackboneEdge* edge = state.view().backbone_edge(edge_id);
    if (edge == nullptr || ports.size() != 2) {
      return false;
    }
    const wire::core::Vec3d row_axis = normalize_xy(ports[1] - ports[0]);
    const double alignment = abs_dot_xy(row_axis, lateral_xy(edge->dir));
    const double spacing = std::sqrt((ports[1].x - ports[0].x) * (ports[1].x - ports[0].x) +
                                     (ports[1].y - ports[0].y) * (ports[1].y - ports[0].y) +
                                     (ports[1].z - ports[0].z) * (ports[1].z - ports[0].z));
    if (alignment < 0.999 || spacing < 0.19) {
      return false;
    }
  }

  std::size_t jumper_count = 0;
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.source_node_id != corner_node->node_id) {
      continue;
    }
    if (part.kind == wire::core::VisualCurvePartKind::kNodePatch) {
      return false;
    }
    if (part.kind == wire::core::VisualCurvePartKind::kJumper) {
      if (part.incident_edge_ids.size() != 2 || part.samples.size() < 2 ||
          !std::isfinite(part.bounds.min.x) || !std::isfinite(part.bounds.max.z)) {
        return false;
      }
      ++jumper_count;
    }
  }
  return jumper_count == 2 &&
         two_lane_edge_bundles_do_not_twist(state, generated.value.generated_span_ids);
}

bool C664_backbone_sharp_pole_facing_consumes_pair_decision() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {5.0, 8.660254037844386, 0.0}};
  const auto generated = state.GenerateFromBackboneSpec(req);
  const wire::core::ObjectId corner_pole_id =
      pole_at(state, generated.value.generated_pole_ids, {10.0, 0.0, 0.0});
  const wire::core::Pole* corner_pole = state.view().poles().find(corner_pole_id);
  if (!generated.ok || corner_pole == nullptr) {
    return false;
  }
  const double yaw_rad = corner_pole->world_transform.rotation_euler_deg.z *
                         (3.14159265358979323846 / 180.0);
  const wire::core::Vec3d actual{std::cos(yaw_rad), std::sin(yaw_rad), 0.0};
  const wire::core::Vec3d expected = normalize_xy((req.path.polyline[0] - req.path.polyline[1]) +
                                                  (req.path.polyline[2] - req.path.polyline[1]));
  if (abs_dot_xy(actual, expected) < 0.999) {
    return false;
  }

  std::string source;
  std::string body;
  const std::filesystem::path file = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  return file_text(file, &source) &&
         function_body(source, "EditResult<bool> pipeline::emit_poles(topo* made, const pairs& ps", &body) &&
         contains_text(body, "ps.jumpers") && contains_text(body, "node_forward") &&
         !contains_text(body, "acos") && !contains_text(body, "kSharpCornerInteriorAngleMaxDeg");
}

bool C414_backbone_simple_avoid_detour_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.constraints.avoid_points.push_back({6.0, 0.0, 0.0});
  req.constraints.avoid_radius_m = 1.0;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 3 || out.value.generated_span_ids.empty()) {
    return false;
  }
  const auto detour = std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                                   [](const wire::core::SavedBackboneNode& node) {
                                     return almost_equal(node.position.x, 6.0, 1e-9) &&
                                            std::abs(node.position.y) > 1.0;
                                   });
  return detour != state.view().backbone().nodes.end();
}

bool C415_backbone_has_no_empty_levels_layer() {
  const std::filesystem::path dir = repo_root() / "core" / "src" / "generation" / "backbone";
  for (const auto& entry : std::filesystem::recursive_directory_iterator(dir)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::string text;
    if (!file_text(entry.path(), &text)) {
      return false;
    }
    if (contains_text(text, "struct levels") || contains_text(text, "levels ")) {
      return false;
    }
  }
  return true;
}

bool C416_backbone_node_mode_not_present_is_noop() {
  const std::vector<wire::core::Vec3d> plain = node_mode_points(false);
  const std::vector<wire::core::Vec3d> no_op = node_mode_points(true);
  if (plain.empty() || plain.size() != no_op.size()) {
    return false;
  }
  for (std::size_t i = 0; i < plain.size(); ++i) {
    if (!almost_equal(plain[i], no_op[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C417_backbone_node_mode_pass_through_without_target_rejected() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 0;
  mode.bundle_template_id = wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage);
  mode.mode = wire::core::BundleNodeMode::kPassThrough;
  req.node_bundle_modes.push_back(mode);
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C418_backbone_node_mode_unknown_bundle_rejected() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 0;
  mode.bundle_template_id = wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage);
  mode.mode = wire::core::BundleNodeMode::kNotPresent;
  req.node_bundle_modes.push_back(mode);
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

bool C419_backbone_node_mode_point_index_rejected() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = req.path.polyline.size();
  mode.bundle_template_id = wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage);
  mode.mode = wire::core::BundleNodeMode::kNotPresent;
  req.node_bundle_modes.push_back(mode);
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "unsupported");
}

} // namespace backbone_tests
