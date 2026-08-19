#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "city/wire/core_test_hook.hpp"
#include "city/wire/coord_utils.hpp"
#include "city/wire/core_view.hpp"

#include "../../src/generation/backbone/curve_parts.hpp"
#include "../../src/generation/backbone/model_assembly.hpp"
#include "../../src/generation/backbone/out.hpp"
#include "../../src/geometry/detail_curve_postprocess.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace helpers;

namespace backbone_tests {

bool C506_backbone_support_group_is_placement_layer() {
  const std::filesystem::path header = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.hpp";
  std::string h;
  if (!file_text(header, &h)) {
    return false;
  }
  return contains_text(h, "struct group_member") && contains_text(h, "struct group") &&
         contains_text(h, "std::vector<group_member> row_members") && contains_text(h, "Vec3d group_axis") &&
         contains_text(h, "int vertical_order") && contains_text(h, "double endpoint_offset_m");
}

bool C507_backbone_support_group_built_after_intent() {
  const std::filesystem::path header = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.hpp";
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string h;
  std::string cpp;
  if (!file_text(header, &h) || !file_text(source, &cpp)) {
    return false;
  }
  return contains_text(h, "EditResult<groups> make(const pairs& ps, const intent& intents) const") &&
         contains_text(h, "rules make(const topo& made, const pairs& ps, const groups& placement) const") &&
         !contains_text(h, "rules make(const topo& made, const intent& intents) const") &&
         contains_text(cpp, "return make(ps.value, intents.value)");
}

bool C508_backbone_support_group_drives_lowered_rules() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    const city::wire::SpanLayoutRulesView rules = state.span_layout_rules(span_id);
    if (!rules.has_rule()) {
      return false;
    }
    const auto has_lowered_group = [](const city::wire::EndpointLayoutRule& endpoint) {
      return endpoint.default_lower_required && endpoint.semantic.lower_required &&
             endpoint.semantic.support_group_id >= 0 && endpoint.branch_down_offset_m > 0.0;
    };
    if (has_lowered_group(rules.rule->start) || has_lowered_group(rules.rule->end)) {
      return true;
    }
  }
  return false;
}

bool C509_backbone_support_group_avoids_visual_terms() {
  const std::filesystem::path header = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.hpp";
  std::string h;
  if (!file_text(header, &h)) {
    return false;
  }
  const std::size_t group_pos = h.find("struct group {");
  const std::size_t next_pos = h.find("struct groups", group_pos);
  if (group_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = h.substr(group_pos, next_pos - group_pos);
  return !contains_text(body, "mount") && !contains_text(body, "tip") && !contains_text(body, "arm") &&
         !contains_text(body, "insulator") && !contains_text(body, "attachment");
}

bool C510_backbone_layout_consumes_group_offset() {
  const std::filesystem::path header = repo_root() / "domains" / "wire" / "include" / "city" / "wire" / "span_layout_types.hpp";
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "derive_span_layout.cpp";
  const std::filesystem::path plan_header = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "model_assembly.hpp";
  std::string h;
  std::string cpp;
  std::string plan_h;
  if (!file_text(header, &h) || !file_text(source, &cpp) || !file_text(plan_header, &plan_h)) {
    return false;
  }
  const std::size_t fn_pos = h.find("inline void ApplyEndpointLayoutRule");
  const std::size_t next_pos = h.find("struct SpanLayoutRule", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = h.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "rule.endpoint_offset_z_m") &&
         !contains_text(body, "dst.endpoint_world.z += endpoint_offset") &&
         contains_text(cpp, "const FixturePlacementPlanByPort* fixture_plan") &&
         contains_text(cpp, "plan_it->second.wire_endpoint") &&
         contains_text(cpp, "endpoint_down_offset(rule)") &&
         contains_text(plan_h, "RowFixturePlacementPlan row_fixture") &&
         !contains_text(body, "kLowerOffsetM");
}

bool C511_backbone_draw_saved_from_geom() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span_id);
    const city::wire::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    const city::wire::SpanRenderCacheEntry* render = state.find_span_render_cache(span_id);
    if (curve == nullptr || visual == nullptr || render == nullptr) {
      return false;
    }
    if (render->arc_length_m_by_point.size() != curve->detail.sample_points.size() ||
        render->arc_length_normalized_by_point.size() != curve->detail.sample_points.size()) {
      return false;
    }
    if (!curve->detail.sample_points.empty() && render->arc_length_m_by_point.empty()) {
      return false;
    }
    if (curve->detail.sample_points.size() >= 2 &&
        render->segment_length_m.size() + 1 != curve->detail.sample_points.size()) {
      return false;
    }
  }
  return true;
}

bool C513_backbone_lowered_layout_does_not_emit_support_arm_placeholder() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    const city::wire::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    const city::wire::SpanLayoutView layout = state.span_layout(span_id);
    if (visual == nullptr || !layout.has_layout()) {
      return false;
    }
    if (layout.entry->start.default_lower_required || layout.entry->end.default_lower_required) {
      return visual->parts.empty();
    }
  }
  return false;
}

bool C514_backbone_draw_save_is_direct() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "state" / "span_runtime.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t visual_pos = cpp.find("void CoreState::cache_span_visual");
  const std::size_t render_pos = cpp.find("void CoreState::cache_span_render");
  const std::size_t next_pos = cpp.find("void CoreState::remove_span_from_caches", render_pos);
  if (visual_pos == std::string::npos || render_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(visual_pos, next_pos - visual_pos);
  return contains_text(body, "visual_cache.by_span") && contains_text(body, "render_cache.by_span") &&
         !contains_text(body, "rebuild_span_visual") && !contains_text(body, "rebuild_span_bounds") &&
         !contains_text(body, "cache_rebuilt_span_geometry") && !contains_text(body, "dirty");
}

bool C740_visual_curve_part_stats_count_full_curve_builds() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::VisualCurvePartCache& cache = state.visual_curve_parts();
  const std::size_t expected_sections = out.value.generated_span_ids.size();
  const std::size_t support_count = static_cast<std::size_t>(std::count_if(
      cache.parts.begin(), cache.parts.end(), [](const city::wire::VisualCurvePart& part) {
        return part.supplemental_kind == city::wire::VisualSupplementalKind::kSupportPath;
      }));
  if (cache.stats.sections != expected_sections ||
      cache.stats.curve_builds != expected_sections + support_count) {
    return false;
  }

  const city::wire::ObjectId span_id = out.value.generated_span_ids.front();
  const city::wire::SpanLayoutView span_layout = state.span_layout(span_id);
  const city::wire::CurveCacheEntry* cached = state.find_curve_cache(span_id);
  if (!span_layout.has_layout() || cached == nullptr || cached->detail.sample_points.size() < 3) {
    return false;
  }
  city::wire::generation::backbone::layout made{};
  made.entries.push_back(*span_layout.entry);
  city::wire::generation::backbone::curve built{};
  city::wire::DetailCurve injected = cached->detail;
  const std::size_t middle = injected.sample_points.size() / 2;
  injected.sample_points[middle].z += 0.123;
  const city::wire::Vec3d expected = injected.sample_points[middle];
  built.data.push_back({span_id, std::move(injected)});
  const city::wire::EditResult<city::wire::VisualCurvePartCache> supplied =
      city::wire::generation::backbone::make_visual_curve_parts(state, made, {span_id}, &built);
  if (!supplied.ok) {
    return false;
  }
  for (const city::wire::VisualCurvePart& part : supplied.value.parts) {
    if (part.kind == city::wire::VisualCurvePartKind::kEdgeBody && part.source_span_id == span_id &&
        part.section_key.is_base() &&
        std::find_if(part.samples.begin(), part.samples.end(), [&](const city::wire::Vec3d& point) {
          return almost_equal(point, expected, 1e-12);
        }) != part.samples.end()) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> visual_part_snapshot(const city::wire::VisualCurvePartCache& cache) {
  std::vector<std::string> out{};
  out.reserve(cache.parts.size());
  for (const city::wire::VisualCurvePart& part : cache.parts) {
    std::ostringstream ss;
    ss << static_cast<int>(part.kind) << ":" << part.source_node_id << ":" << part.source_edge_id << ":"
       << part.source_span_id << ":" << part.source_bundle_id << ":" << part.bundle_template_id << ":"
       << part.lane_index << ":" << part.has_section_key << ":";
    if (part.has_section_key) {
      ss << part.section_key.logical_span_id << ":" << part.section_key.edge_bundle_id << ":"
         << part.section_key.rule_owner_id << ":" << part.section_key.rule_id << ":"
         << part.section_key.instance_index;
    }
    ss << ":" << part.cable_run_id << ":" << part.samples.size() << ":" << part.bezier_control_points.size();
    if (!part.samples.empty()) {
      const city::wire::Vec3d& first = part.samples.front();
      const city::wire::Vec3d& last = part.samples.back();
      ss << ":" << first.x << "," << first.y << "," << first.z << ":" << last.x << "," << last.y << ","
         << last.z;
    }
    out.push_back(ss.str());
  }
  return out;
}

bool C741_scoped_visual_curve_rebuild_matches_full_rebuild() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() < 2) {
    return false;
  }
  const city::wire::ObjectId terminal_pole = first.value.generated_pole_ids.back();
  const auto* pole = state.view().poles().find(terminal_pole);
  if (pole == nullptr) {
    return false;
  }
  city::wire::BackboneSpec extension = line_req(state);
  extension.path.polyline = {pole->world_transform.position, {24.0, 0.0, 0.0}};
  extension.path.node_specs = {pole_spec(0, terminal_pole)};
  const auto second = state.GenerateFromBackboneSpec(extension);
  if (!second.ok || second.value.generated_span_ids.size() != 1) {
    return false;
  }
  const city::wire::VisualCurvePartCache scoped = state.visual_curve_parts();
  const city::wire::EditResult<city::wire::VisualCurvePartCache> full =
      city::wire::generation::backbone::make_visual_curve_parts(state, {});
  if (!full.ok) {
    return false;
  }
  const std::size_t rebuilt_support_count = static_cast<std::size_t>(std::count_if(
      scoped.parts.begin(), scoped.parts.end(), [](const city::wire::VisualCurvePart& part) {
        return part.supplemental_kind == city::wire::VisualSupplementalKind::kSupportPath;
      }));
  return scoped.stats.curve_builds == scoped.stats.sections + rebuilt_support_count &&
         visual_part_snapshot(scoped) == visual_part_snapshot(full.value);
}

bool C515_backbone_rejects_existing_pole_without_saved_graph() {
  city::wire::CoreState state;
  city::wire::Transformd tf{};
  tf.position = {0.0, 0.0, 0.0};
  const auto pole = state.AddPole(tf);
  if (!pole.ok) {
    return false;
  }
  const std::size_t pole_count = state.view().poles().size();
  const std::size_t span_count = state.view().spans().size();
  const std::size_t graph_nodes = state.view().backbone().nodes.size();
  const std::size_t graph_edges = state.view().backbone().edges.size();
  city::wire::BackboneSpec req = line_req(state);
  req.path.polyline = {tf.position, {12.0, 0.0, 0.0}};
  req.path.node_specs = {pole_spec(0, pole.value)};
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "saved backbone graph missing") &&
         state.view().poles().size() == pole_count && state.view().spans().size() == span_count &&
         state.view().backbone().nodes.size() == graph_nodes && state.view().backbone().edges.size() == graph_edges;
}

bool C516_backbone_generated_pole_with_saved_graph_still_connects() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr || state.view().backbone_node_for_pole(b) == nullptr) {
    return false;
  }
  city::wire::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  return second.ok && !second.value.generated_span_ids.empty() && state.view().pole_frontier(b).edge_ids.size() == 3;
}

bool C518_backbone_lowered_layout_places_support_and_endpoint_at_final_height() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    const city::wire::SpanLayoutView layout = state.span_layout(span_id);
    if (!layout.has_layout()) {
      return false;
    }
    const auto endpoint_ok = [&](const city::wire::LayoutEndpoint& endpoint) {
      if (!endpoint.default_lower_required && !endpoint.lower_required) {
        return false;
      }
      const city::wire::Port* port = state.view().ports().find(endpoint.port_id);
      if (port == nullptr) {
        return false;
      }
      const double lower_offset =
          endpoint.branch_down_offset_m > 0.0 ? endpoint.branch_down_offset_m : endpoint.automatic_branch_down_offset_m;
      return lower_offset > 0.0 && almost_equal(endpoint.support_world, endpoint.endpoint_world, 1e-9) &&
             endpoint.endpoint_world.z < port->world_position.z - 0.1;
    };
    if (endpoint_ok(layout.entry->start) || endpoint_ok(layout.entry->end)) {
      return true;
    }
  }
  return false;
}

bool C519_backbone_draw_does_not_emit_lowering_placeholder() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("draw pipeline::make(const layout& placed, const geom& shaped) const");
  const std::size_t next_pos = cpp.find("void pipeline::save(const rules& made)", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  if (contains_text(body, "branch_down_offset_m")) {
    return false;
  }

  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    const city::wire::SpanLayoutView layout = state.span_layout(span_id);
    const city::wire::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    if (!layout.has_layout() || visual == nullptr) {
      return false;
    }
    auto lowered_without_placeholder = [&](const city::wire::LayoutEndpoint& endpoint) {
      if (!endpoint.default_lower_required && !endpoint.lower_required) {
        return false;
      }
      return almost_equal(endpoint.support_world, endpoint.endpoint_world, 1e-9) && visual->parts.empty();
    };
    if (lowered_without_placeholder(layout.entry->start) || lowered_without_placeholder(layout.entry->end)) {
      return true;
    }
  }
  return false;
}

bool C521_backbone_context_link_preserves_saved_dir() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t prepare_pos = cpp.find("EditResult<bool> pipeline::prepare()");
  const std::size_t check_pos = cpp.find("EditResult<bool> pipeline::check() const", prepare_pos);
  const std::size_t make_pos = cpp.find("EditResult<pairs> pipeline::make(const graph& made) const");
  const std::size_t intent_pos = cpp.find("EditResult<intent> pipeline::make(const pairs& ps) const", make_pos);
  if (prepare_pos == std::string::npos || check_pos == std::string::npos || make_pos == std::string::npos ||
      intent_pos == std::string::npos) {
    return false;
  }
  const std::string prepare_body = cpp.substr(prepare_pos, check_pos - prepare_pos);
  const std::string make_body = cpp.substr(make_pos, intent_pos - make_pos);
  return contains_text(prepare_body, "edge.dir = g_.nodes[i + 1].pos - g_.nodes[i].pos") &&
         contains_text(prepare_body, "edge.dir = saved->dir") &&
         !contains_text(make_body, "edge.dir = made.nodes");
}

bool C524_backbone_scenario_simple_line_mainline() {
  return C379_backbone_m1_required_outputs() && C511_backbone_draw_saved_from_geom() &&
         C424_backbone_saves_backbone_graph_nodes_edges() && C370_backbone_no_v1_deps();
}

bool C525_backbone_scenario_polyline3_connectivity_once() {
  return C388_backbone_polyline3_pair_model() && C392_backbone_polyline3_outputs() &&
         C422_backbone_rules_consume_topo_and_groups();
}

bool C526_backbone_scenario_multiple_bundles_share_connectivity() {
  return C400_backbone_multiple_bundles_smoke() && C401_backbone_multiple_bundles_polyline3_outputs() &&
         C402_backbone_bundle_spec_does_not_affect_pairs();
}

bool C527_backbone_scenario_existing_pole_continuation_uses_saved_graph() {
  return C516_backbone_generated_pole_with_saved_graph_still_connects() &&
         C515_backbone_rejects_existing_pole_without_saved_graph() &&
         C396_backbone_existing_pole_does_not_read_existing_spans();
}

bool C528_backbone_scenario_branch_emits_new_link_only() {
  return C458_backbone_existing_branch_BD_on_ABC() && C460_backbone_context_links_are_not_emitted() &&
         C499_backbone_context_link_is_not_saved();
}

bool C529_backbone_scenario_cross_without_kind_label() {
  return C459_backbone_existing_cross_DBE_on_ABC() && C462_backbone_no_junction_kind_after_existing_context() &&
         C477_backbone_cross_rows_are_separated_without_cross_kind();
}

bool C530_backbone_scenario_same_edge_different_bundle() {
  return C464_backbone_different_bundle_on_same_edge_allowed() &&
         C432_backbone_multiple_bundles_create_multiple_edge_bundles() &&
         C487_backbone_port_resolution_requires_bundle_compatible_scope();
}

bool C531_backbone_scenario_duplicate_reject_unchanged() {
  return C490_backbone_duplicate_same_edge_bundle_lane_rejected() &&
         C466_backbone_duplicate_reject_keeps_state_unchanged() &&
         C520_backbone_duplicate_span_binding_preflight_before_emit();
}

bool C532_backbone_scenario_pass_through_lowering_consumer_chain() {
  return C491_backbone_branch_lowering_v1_affects_geom() && C493_backbone_pass_through_does_not_change_pair_open() &&
         C519_backbone_draw_does_not_emit_lowering_placeholder();
}

bool C534_backbone_invalid_inputs_stop_before_emit() {
  return C409_backbone_rejects_missing_port_band() && C414_backbone_simple_avoid_detour_supported() &&
         C515_backbone_rejects_existing_pole_without_saved_graph();
}

bool C536_backbone_draw_consumer_outputs_are_minimal() {
  return C511_backbone_draw_saved_from_geom() && C513_backbone_lowered_layout_does_not_emit_support_arm_placeholder() &&
         C519_backbone_draw_does_not_emit_lowering_placeholder();
}

bool C539_backbone_supported_request_creates_saved_graph_outputs() {
  return C524_backbone_scenario_simple_line_mainline() && C523_backbone_scope_gate_matches_entrypoint();
}

namespace {

std::size_t visual_curve_part_count(const city::wire::CoreState& state, city::wire::VisualCurvePartKind kind) {
  std::size_t count = 0;
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == kind) {
      ++count;
    }
  }
  return count;
}

bool tangent_compatible(const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
  auto length = [](const city::wire::Vec3d& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
  };
  const double al = length(a);
  const double bl = length(b);
  if (al <= 1e-9 || bl <= 1e-9) {
    return false;
  }
  const double dot = (a.x * b.x + a.y * b.y + a.z * b.z) / (al * bl);
  return dot > 0.999;
}

bool sampled_boundary_is_g1(const city::wire::VisualCurvePart& a, const city::wire::VisualCurvePart& b,
                            const city::wire::Vec3d& boundary) {
  auto direction_into_part = [&](const city::wire::VisualCurvePart& part, city::wire::Vec3d* out) {
    if (part.samples.size() < 4 || out == nullptr) {
      return false;
    }
    if (almost_equal(part.samples.front(), boundary, 1e-9)) {
      *out = {-11.0 * part.samples[0].x + 18.0 * part.samples[1].x - 9.0 * part.samples[2].x +
                  2.0 * part.samples[3].x,
              -11.0 * part.samples[0].y + 18.0 * part.samples[1].y - 9.0 * part.samples[2].y +
                  2.0 * part.samples[3].y,
              -11.0 * part.samples[0].z + 18.0 * part.samples[1].z - 9.0 * part.samples[2].z +
                  2.0 * part.samples[3].z};
      return true;
    }
    if (almost_equal(part.samples.back(), boundary, 1e-9)) {
      const std::size_t last = part.samples.size() - 1;
      *out = {-11.0 * part.samples[last].x + 18.0 * part.samples[last - 1].x -
                  9.0 * part.samples[last - 2].x + 2.0 * part.samples[last - 3].x,
              -11.0 * part.samples[last].y + 18.0 * part.samples[last - 1].y -
                  9.0 * part.samples[last - 2].y + 2.0 * part.samples[last - 3].y,
              -11.0 * part.samples[last].z + 18.0 * part.samples[last - 1].z -
                  9.0 * part.samples[last - 2].z + 2.0 * part.samples[last - 3].z};
      return true;
    }
    return false;
  };

  city::wire::Vec3d a_direction{};
  city::wire::Vec3d b_direction{};
  if (!direction_into_part(a, &a_direction) || !direction_into_part(b, &b_direction)) {
    return false;
  }
  return tangent_compatible(a_direction, {-b_direction.x, -b_direction.y, -b_direction.z});
}

bool boundary_matches_source_curve(const city::wire::CoreState& state,
                                   const city::wire::VisualCurvePart& body,
                                   const city::wire::Vec3d& boundary) {
  const city::wire::CurveCacheEntry* source = state.find_curve_cache(body.source_span_id);
  if (source == nullptr) return false;
  const city::wire::DetailCurve& curve = source->detail;
  const city::wire::Vec3d start = curve.EvaluatePosition(0.0);
  const city::wire::Vec3d end = curve.EvaluatePosition(1.0);
  const auto distance = [](const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
    return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
  };
  const bool from_start = distance(boundary, start) < distance(boundary, end);
  const city::wire::Vec3d attachment = from_start ? start : end;
  const double target_xy = std::hypot(boundary.x - attachment.x, boundary.y - attachment.y);
  double low = from_start ? 0.0 : 0.5;
  double high = from_start ? 0.5 : 1.0;
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double u = (low + high) * 0.5;
    const city::wire::Vec3d point = curve.EvaluatePosition(u);
    const double distance_xy = std::hypot(point.x - attachment.x, point.y - attachment.y);
    if ((distance_xy < target_xy) == from_start) low = u;
    else high = u;
  }
  const double u = (low + high) * 0.5;
  city::wire::Vec3d expected_tangent = curve.EvaluateTangent(u);
  if (!from_start) expected_tangent = {-expected_tangent.x, -expected_tangent.y, -expected_tangent.z};
  const city::wire::Vec3d body_tangent =
      almost_equal(body.boundary_a, boundary, 1e-9) ? body.tangent_a : body.tangent_b;
  return almost_equal(boundary, curve.EvaluatePosition(u), 1e-6) &&
         tangent_compatible(body_tangent, expected_tangent);
}

bool body_samples_follow_source_curve(const city::wire::CoreState& state,
                                      const city::wire::VisualCurvePart& body) {
  if (!body.has_section_key || !body.section_key.is_base() || body.samples.size() < 2) return false;
  const city::wire::CurveCacheEntry* source = state.find_curve_cache(body.source_span_id);
  if (source == nullptr) return false;
  const city::wire::DetailCurve& curve = source->detail;
  const city::wire::Vec3d start = curve.EvaluatePosition(0.0);
  const city::wire::Vec3d end = curve.EvaluatePosition(1.0);
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double horizontal_length2 = dx * dx + dy * dy;
  if (horizontal_length2 <= 1e-12) return false;
  for (const city::wire::Vec3d& sample : body.samples) {
    const double u = std::clamp(((sample.x - start.x) * dx + (sample.y - start.y) * dy) /
                                    horizontal_length2,
                                0.0, 1.0);
    if (!almost_equal(sample, curve.EvaluatePosition(u), 1e-7)) return false;
  }
  return true;
}

double sampled_curve_length(const city::wire::VisualCurvePart& part) {
  double length = 0.0;
  for (std::size_t i = 1; i < part.samples.size(); ++i) {
    const city::wire::Vec3d delta{part.samples[i].x - part.samples[i - 1].x,
                                  part.samples[i].y - part.samples[i - 1].y,
                                  part.samples[i].z - part.samples[i - 1].z};
    length += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
  }
  return length;
}

bool finite_visual_curve_parts(const city::wire::CoreState& state) {
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    for (const city::wire::Vec3d& p : part.samples) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        return false;
      }
    }
  }
  return true;
}

double distance_to_chord(const city::wire::Vec3d& p, const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
  const city::wire::Vec3d ab{b.x - a.x, b.y - a.y, b.z - a.z};
  const city::wire::Vec3d ap{p.x - a.x, p.y - a.y, p.z - a.z};
  const double ab_len2 = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
  if (ab_len2 <= 1e-12) {
    return 0.0;
  }
  const double t = std::clamp((ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / ab_len2, 0.0, 1.0);
  const city::wire::Vec3d closest{a.x + ab.x * t, a.y + ab.y * t, a.z + ab.z * t};
  const city::wire::Vec3d d{p.x - closest.x, p.y - closest.y, p.z - closest.z};
  return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}

double cross_xy(const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
  return a.x * b.y - a.y * b.x;
}

bool point_in_xy_triangle(const city::wire::Vec3d& point, const city::wire::Vec3d& a,
                          const city::wire::Vec3d& b, const city::wire::Vec3d& c) {
  const city::wire::Vec3d ab{b.x - a.x, b.y - a.y, 0.0};
  const city::wire::Vec3d bc{c.x - b.x, c.y - b.y, 0.0};
  const city::wire::Vec3d ca{a.x - c.x, a.y - c.y, 0.0};
  const city::wire::Vec3d ap{point.x - a.x, point.y - a.y, 0.0};
  const city::wire::Vec3d bp{point.x - b.x, point.y - b.y, 0.0};
  const city::wire::Vec3d cp{point.x - c.x, point.y - c.y, 0.0};
  const double c0 = cross_xy(ab, ap);
  const double c1 = cross_xy(bc, bp);
  const double c2 = cross_xy(ca, cp);
  return (c0 >= -1e-8 && c1 >= -1e-8 && c2 >= -1e-8) ||
         (c0 <= 1e-8 && c1 <= 1e-8 && c2 <= 1e-8);
}

bool sampled_xy_turn_is_monotonic(const city::wire::VisualCurvePart& patch) {
  if (patch.samples.size() < 3) {
    return false;
  }
  const city::wire::Vec3d incoming{-patch.tangent_a.x, -patch.tangent_a.y, 0.0};
  const city::wire::Vec3d outgoing{patch.tangent_b.x, patch.tangent_b.y, 0.0};
  const double expected_turn = cross_xy(incoming, outgoing);
  if (std::abs(expected_turn) <= 1e-9) {
    return true;
  }
  for (std::size_t i = 1; i + 1 < patch.samples.size(); ++i) {
    const city::wire::Vec3d before{patch.samples[i].x - patch.samples[i - 1].x,
                                   patch.samples[i].y - patch.samples[i - 1].y, 0.0};
    const city::wire::Vec3d after{patch.samples[i + 1].x - patch.samples[i].x,
                                  patch.samples[i + 1].y - patch.samples[i].y, 0.0};
    if (cross_xy(before, after) * expected_turn < -1e-9) {
      return false;
    }
  }
  return true;
}

bool touches_patch_boundary(const city::wire::VisualCurvePart& body, const city::wire::VisualCurvePart& patch) {
  return almost_equal(body.boundary_a, patch.boundary_a, 1e-9) ||
         almost_equal(body.boundary_b, patch.boundary_a, 1e-9) ||
         almost_equal(body.boundary_a, patch.boundary_b, 1e-9) ||
         almost_equal(body.boundary_b, patch.boundary_b, 1e-9);
}

bool same_visual_cable_section_family(const city::wire::CableSectionKey& a, const city::wire::CableSectionKey& b) {
  return a.is_base() == b.is_base() && a.rule_owner_id == b.rule_owner_id &&
         a.rule_id == b.rule_id && a.instance_index == b.instance_index;
}

city::wire::CablePopulationRule two_extra_lv_population_rule() {
  city::wire::CablePopulationRule rule{};
  rule.rule_id = 11;
  rule.explicit_seed = 1234;
  rule.priority = 10;
  rule.min_extra_count = 2;
  rule.max_extra_count = 2;
  rule.min_spacing_m = 0.01;
  rule.lateral_min_m = -2.0;
  rule.lateral_max_m = 2.0;
  rule.height_min_m = 0.0;
  rule.height_max_m = 20.0;
  rule.randomness = 0.6;
  return rule;
}

bool enable_two_extra_lv_population(city::wire::CoreState& state) {
  city::wire::BundleTemplate lv_template = state.view().bundle_templates().at(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  lv_template.population_rules.push_back(two_extra_lv_population_rule());
  return state.UpdateBundleTemplate(lv_template).ok;
}

bool prepare_two_lane_low_voltage_for_run_test(city::wire::CoreState& state) {
  city::wire::BundleTemplate lv_template = state.view().bundle_templates().at(city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  lv_template.count_rule = city::wire::BundleCountRuleKind::kFixed;
  lv_template.fixed_count = 2;
  lv_template.default_count = 2;
  return state.UpdateBundleTemplate(lv_template).ok;
}

std::vector<const city::wire::VisualCurvePart*> base_edge_bodies(const city::wire::CoreState& state) {
  std::vector<const city::wire::VisualCurvePart*> bodies{};
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == city::wire::VisualCurvePartKind::kEdgeBody && part.has_section_key &&
        part.section_key.is_base()) {
      bodies.push_back(&part);
    }
  }
  return bodies;
}

std::vector<const city::wire::VisualCurvePart*> extra_edge_bodies(const city::wire::CoreState& state,
                                                                  std::size_t instance_index) {
  std::vector<const city::wire::VisualCurvePart*> bodies{};
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == city::wire::VisualCurvePartKind::kEdgeBody && part.has_section_key &&
        !part.section_key.is_base() && part.section_key.instance_index == instance_index) {
      bodies.push_back(&part);
    }
  }
  return bodies;
}

bool all_same_nonzero_run(const std::vector<const city::wire::VisualCurvePart*>& parts) {
  if (parts.empty() || parts.front()->cable_run_id == 0) {
    return false;
  }
  const city::wire::CableRunId run_id = parts.front()->cable_run_id;
  return std::all_of(parts.begin(), parts.end(), [&](const city::wire::VisualCurvePart* part) {
    return part->cable_run_id == run_id;
  });
}

bool all_distinct_nonzero_runs(const std::vector<const city::wire::VisualCurvePart*>& parts) {
  std::unordered_set<city::wire::CableRunId> seen{};
  for (const city::wire::VisualCurvePart* part : parts) {
    if (part->cable_run_id == 0 || !seen.insert(part->cable_run_id).second) {
      return false;
    }
  }
  return !parts.empty();
}

bool contains_span(const std::vector<city::wire::ObjectId>& ids, city::wire::ObjectId id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

} // namespace

bool C634_backbone_terminal_nodes_create_no_node_patch_curve() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  return out.ok && visual_curve_part_count(state, city::wire::VisualCurvePartKind::kNodePatch) == 0 &&
         visual_curve_part_count(state, city::wire::VisualCurvePartKind::kEdgeBody) ==
             out.value.generated_span_ids.size();
}

bool C760_backbone_terminal_edge_body_ends_at_port() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) return false;
  std::size_t terminal_bodies = 0;
  for (const city::wire::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
    if (body.kind != city::wire::VisualCurvePartKind::kEdgeBody || !body.has_section_key ||
        !body.section_key.is_base()) continue;
    const city::wire::Span* span = state.view().spans().find(body.source_span_id);
    const city::wire::Port* port_a = span == nullptr ? nullptr : state.view().ports().find(span->port_a_id);
    const city::wire::Port* port_b = span == nullptr ? nullptr : state.view().ports().find(span->port_b_id);
    if (port_a == nullptr || port_b == nullptr || !almost_equal(body.boundary_a, port_a->world_position, 1e-9) ||
        !almost_equal(body.boundary_b, port_b->world_position, 1e-9)) return false;
    ++terminal_bodies;
  }
  return terminal_bodies == out.value.generated_span_ids.size();
}

bool C761_default_optical_bundle_emits_helix() {
  city::wire::CoreState state;
  city::wire::BackboneSpec request = line_req(state);
  const auto optical_template = state.view().bundle_templates().find(city::wire::kDefaultOpticalBundleTemplateId);
  const auto communication_template =
      state.view().bundle_templates().find(city::wire::kDefaultCommunicationBundleTemplateId);
  if (optical_template == state.view().bundle_templates().end() ||
      communication_template == state.view().bundle_templates().end() ||
      !optical_template->second.span_visual_assembly.support_path_enabled ||
      !optical_template->second.span_visual_assembly.helix_enabled ||
      optical_template->second.span_visual_assembly.helix_samples_per_turn != 6 ||
      optical_template->second.support_wire_pole_band_id != 0 ||
      communication_template->second.span_visual_assembly.helix_enabled ||
      communication_template->second.support_wire_pole_band_id != 0) return false;
  request.bundles.clear();
  request.pole_type_id = optical_template->second.related_pole_type_id;
  request.pole_placement.max_tilt_deg = 0.0;
  add_backbone_bundle(request, city::wire::BundleKind::kOptical);
  const auto out = state.GenerateFromBackboneSpec(request);
  if (!out.ok || out.value.generated_span_ids.empty()) return false;
  std::size_t support_count = 0;
  std::size_t helix_count = 0;
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    support_count += part.kind == city::wire::VisualCurvePartKind::kSupplemental &&
        part.supplemental_kind == city::wire::VisualSupplementalKind::kSupportPath;
    helix_count += part.kind == city::wire::VisualCurvePartKind::kSupplemental &&
        part.supplemental_kind == city::wire::VisualSupplementalKind::kHelix;
  }
  if (support_count != out.value.generated_span_ids.size() || helix_count != support_count) return false;

  using PartSnapshot = std::pair<std::string, std::vector<city::wire::Vec3d>>;
  const auto snapshot_parts = [](const city::wire::CoreState& source) {
    std::vector<PartSnapshot> snapshot{};
    for (const city::wire::VisualCurvePart& part : source.view().visual_curve_parts().parts) {
      if (part.source_span_id == city::wire::kInvalidObjectId) continue;
      std::ostringstream key;
      key << static_cast<int>(part.kind) << ':' << static_cast<int>(part.supplemental_kind) << ':'
          << part.source_span_id << ':' << part.lane_index;
      snapshot.emplace_back(key.str(), part.samples);
    }
    std::sort(snapshot.begin(), snapshot.end(), [](const PartSnapshot& a, const PartSnapshot& b) {
      return a.first < b.first;
    });
    return snapshot;
  };
  const std::vector<PartSnapshot> before = snapshot_parts(state);
  city::wire::PoleTypeDefinition moved = state.view().pole_types().at(request.pole_type_id);
  for (city::wire::PortPlacementBand& band : moved.port_bands) {
    if (band.category != city::wire::ConnectionCategory::kOptical) continue;
    band.height_center_m += 1.0;
    band.height_min_m += 1.0;
    band.height_max_m += 1.0;
  }
  const auto updated = state.UpdatePoleTypeDefinition(moved);
  const std::vector<PartSnapshot> after = snapshot_parts(state);
  if (!updated.ok || before.size() != after.size()) return false;
  for (std::size_t part_index = 0; part_index < before.size(); ++part_index) {
    if (before[part_index].first != after[part_index].first ||
        before[part_index].second.size() != after[part_index].second.size() ||
        before[part_index].second.empty()) return false;
    const city::wire::Vec3d translation =
        after[part_index].second.front() - before[part_index].second.front();
    if (std::abs(translation.z - 1.0) > 1e-8) return false;
    for (std::size_t sample_index = 0; sample_index < before[part_index].second.size(); ++sample_index) {
      const city::wire::Vec3d& a = before[part_index].second[sample_index];
      const city::wire::Vec3d& b = after[part_index].second[sample_index];
      if (city::wire::Length((b - a) - translation) > 1e-8) return false;
    }
  }
  return true;
}

bool C767_default_hv_emits_one_support_path_per_phase_span() {
  city::wire::CoreState state;
  const auto hv_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage);
  const city::wire::BundleTemplate& hv = state.view().bundle_templates().at(hv_id);
  if (!hv.span_visual_assembly.support_path_enabled ||
      hv.span_visual_assembly.helix_enabled || hv.support_wire_pole_band_id != 0) {
    return false;
  }
  city::wire::BackboneSpec request = poly3_req(state);
  request.bundles.clear();
  add_backbone_bundle(request, city::wire::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(request);
  if (!generated.ok || generated.value.generated_span_ids.size() != 6) return false;
  const std::size_t initial_curve_builds = state.view().visual_curve_parts().stats.curve_builds;
  auto support_cable_it = state.view().cable_templates().find(
      city::wire::kDefaultSupportWireCableTemplateId);
  if (support_cable_it == state.view().cable_templates().end() ||
      support_cable_it->second.name != "SUPPORT_WIRE") {
    return false;
  }
  const city::wire::CableTemplate support_cable = support_cable_it->second;

  std::unordered_set<city::wire::ObjectId> support_span_ids{};
  std::vector<city::wire::Vec3d> support_starts{};
  std::size_t helix_count = 0;
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kSupplemental) continue;
    if (part.supplemental_kind == city::wire::VisualSupplementalKind::kHelix) {
      ++helix_count;
      continue;
    }
    if (part.supplemental_kind != city::wire::VisualSupplementalKind::kSupportPath) continue;
    if (!almost_equal(part.wire_radius_m, support_cable.outer_diameter_m * 0.5, 1e-12) ||
        part.color_rgba != support_cable.color_rgba ||
        part.material_style != support_cable.material_style) {
      return false;
    }
    const city::wire::SpanLayoutEntry* layout = state.span_layout(part.source_span_id).entry;
    if (layout == nullptr || !almost_equal(part.samples.front(), layout->start.endpoint_world, 1e-9) ||
        !almost_equal(part.samples.back(), layout->end.endpoint_world, 1e-9)) return false;
    const auto primary = city::wire::generation::backbone::make_primary_curve_between(
        state, part.source_span_id, layout->start.endpoint_world, layout->end.endpoint_world);
    if (!primary.ok || primary.value.sample_points.size() != part.samples.size()) return false;
    bool reached_separation = false;
    for (std::size_t index = 0; index < part.samples.size(); ++index) {
      const double separation =
          city::wire::Length(part.samples[index] - primary.value.sample_points[index]);
      if (separation > part.wire_radius_m * 2.0 + 1e-9) return false;
      reached_separation = reached_separation ||
          separation >= part.wire_radius_m * 2.0 - 1e-9;
    }
    if (!reached_separation) return false;
    support_span_ids.insert(part.source_span_id);
    support_starts.push_back(part.boundary_a);
  }
  if (support_span_ids.size() != generated.value.generated_span_ids.size() ||
      helix_count != 0 || support_starts.size() != 6) {
    return false;
  }

  city::wire::CableTemplate edited_support_cable = support_cable;
  edited_support_cable.outer_diameter_m += 0.004;
  edited_support_cable.color_rgba ^= 0x0000FF00u;
  const auto updated_support_cable = state.UpdateCableTemplate(edited_support_cable);
  if (!updated_support_cable.ok || !updated_support_cable.value) return false;
  std::size_t updated_support_count = 0;
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.supplemental_kind != city::wire::VisualSupplementalKind::kSupportPath) continue;
    if (!almost_equal(part.wire_radius_m, edited_support_cable.outer_diameter_m * 0.5, 1e-12) ||
        part.color_rgba != edited_support_cable.color_rgba) {
      return false;
    }
    ++updated_support_count;
  }
  if (updated_support_count != support_span_ids.size()) return false;

  for (std::size_t i = 0; i < support_starts.size(); ++i) {
    for (std::size_t j = i + 1; j < support_starts.size(); ++j) {
      if (city::wire::Length(support_starts[i] - support_starts[j]) < 0.1) return false;
    }
  }

  city::wire::CoreState without_support;
  city::wire::BundleTemplate disabled = without_support.view().bundle_templates().at(hv_id);
  disabled.span_visual_assembly.support_path_enabled = false;
  if (!without_support.UpdateBundleTemplate(disabled).ok) return false;
  city::wire::BackboneSpec baseline_request = poly3_req(without_support);
  baseline_request.bundles.clear();
  add_backbone_bundle(baseline_request, city::wire::BundleKind::kHighVoltage);
  const auto baseline = without_support.GenerateFromBackboneSpec(baseline_request);
  if (!baseline.ok || state.view().poles().size() != without_support.view().poles().size() ||
      state.view().ports().size() != without_support.view().ports().size() ||
      state.view().spans().size() != without_support.view().spans().size() ||
      state.view().bundles().size() != without_support.view().bundles().size()) {
    return false;
  }
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kEdgeBody || !part.has_section_key) continue;
    const auto baseline_part = std::find_if(
        without_support.view().visual_curve_parts().parts.begin(),
        without_support.view().visual_curve_parts().parts.end(),
        [&](const city::wire::VisualCurvePart& candidate) {
          return candidate.kind == part.kind && candidate.has_section_key &&
                 candidate.section_key.logical_span_id == part.section_key.logical_span_id &&
                 candidate.section_key.edge_bundle_id == part.section_key.edge_bundle_id &&
                 candidate.section_key.rule_owner_id == part.section_key.rule_owner_id &&
                 candidate.section_key.rule_id == part.section_key.rule_id &&
                 candidate.section_key.instance_index == part.section_key.instance_index;
        });
    if (baseline_part == without_support.view().visual_curve_parts().parts.end() ||
        baseline_part->samples.size() != part.samples.size() ||
        !std::equal(part.samples.begin(), part.samples.end(), baseline_part->samples.begin(),
                    [](const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
                      return almost_equal(a, b, 0.0);
                    })) {
      return false;
    }
  }
  if (initial_curve_builds != without_support.view().visual_curve_parts().stats.curve_builds + support_span_ids.size()) {
    return false;
  }

  city::wire::CoreState lv_state;
  const city::wire::BundleTemplate& lv = lv_state.view().bundle_templates().at(
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage));
  if (!lv.span_visual_assembly.support_path_enabled ||
      lv.span_visual_assembly.helix_enabled || lv.support_wire_pole_band_id != 0) return false;
  city::wire::BackboneSpec lv_request = poly3_req(lv_state);
  lv_request.bundles.clear();
  add_backbone_bundle(lv_request, city::wire::BundleKind::kLowVoltage);
  const auto lv_generated = lv_state.GenerateFromBackboneSpec(lv_request);
  if (!lv_generated.ok) return false;
  std::unordered_set<city::wire::ObjectId> lv_support_span_ids{};
  for (const city::wire::VisualCurvePart& part : lv_state.view().visual_curve_parts().parts) {
    if (part.supplemental_kind != city::wire::VisualSupplementalKind::kSupportPath) continue;
    const city::wire::SpanLayoutEntry* layout = lv_state.span_layout(part.source_span_id).entry;
    if (layout == nullptr || !almost_equal(part.boundary_a, layout->start.endpoint_world, 1e-9) ||
        !almost_equal(part.boundary_b, layout->end.endpoint_world, 1e-9)) return false;
    lv_support_span_ids.insert(part.source_span_id);
  }
  if (lv_support_span_ids.size() != lv_generated.value.generated_span_ids.size()) return false;
  return true;
}

bool C635_backbone_simple_continuous_node_creates_node_patch_curve() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok || out.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::SavedBackboneNode* node = state.view().backbone_node_for_pole(out.value.generated_pole_ids[1]);
  if (node == nullptr) {
    return false;
  }
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == city::wire::VisualCurvePartKind::kNodePatch && part.source_node_id == node->node_id &&
        part.node_patch_classification == city::wire::NodePatchClassification::kSimpleContinuous &&
        part.incident_edge_ids.size() == 2 && part.source_span_id == city::wire::kInvalidObjectId) {
      return true;
    }
  }
  return false;
}

bool C636_backbone_edge_body_stops_at_node_patch_boundaries() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok || out.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::SavedBackboneNode* node = state.view().backbone_node_for_pole(out.value.generated_pole_ids[1]);
  if (node == nullptr) {
    return false;
  }
  const city::wire::Vec3d node_pos = node->position;
  for (const city::wire::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != city::wire::VisualCurvePartKind::kNodePatch || patch.source_node_id != node->node_id) {
      continue;
    }
    bool boundary_a_used = false;
    bool boundary_b_used = false;
    for (const city::wire::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != city::wire::VisualCurvePartKind::kEdgeBody) {
        continue;
      }
      boundary_a_used = boundary_a_used || almost_equal(body.boundary_a, patch.boundary_a, 1e-9) ||
                        almost_equal(body.boundary_b, patch.boundary_a, 1e-9);
      boundary_b_used = boundary_b_used || almost_equal(body.boundary_a, patch.boundary_b, 1e-9) ||
                        almost_equal(body.boundary_b, patch.boundary_b, 1e-9);
      if (almost_equal(body.boundary_a, node_pos, 1e-9) || almost_equal(body.boundary_b, node_pos, 1e-9)) {
        return false;
      }
    }
    return boundary_a_used && boundary_b_used;
  }
  return false;
}

bool C637_backbone_node_patch_edge_body_boundary_tangents_are_g1() {
  city::wire::CoreState state;
  city::wire::GeometrySettings settings = state.view().geometry_settings();
  settings.curve_samples = 8;
  settings.sag_enabled = true;
  settings.sag_factor = 0.03;
  if (!state.UpdateGeometrySettings(settings).ok) {
    return false;
  }
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok) {
    return false;
  }
  for (const city::wire::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != city::wire::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    bool a_ok = false;
    bool b_ok = false;
    bool a_source_ok = false;
    bool b_source_ok = false;
    bool a_shape_ok = false;
    bool b_shape_ok = false;
    for (const city::wire::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != city::wire::VisualCurvePartKind::kEdgeBody) {
        continue;
      }
      if (almost_equal(body.boundary_a, patch.boundary_a, 1e-9)) {
        a_ok = a_ok || sampled_boundary_is_g1(body, patch, patch.boundary_a);
        a_source_ok = a_source_ok || boundary_matches_source_curve(state, body, patch.boundary_a);
        a_shape_ok = a_shape_ok || body_samples_follow_source_curve(state, body);
      }
      if (almost_equal(body.boundary_b, patch.boundary_a, 1e-9)) {
        a_ok = a_ok || sampled_boundary_is_g1(body, patch, patch.boundary_a);
        a_source_ok = a_source_ok || boundary_matches_source_curve(state, body, patch.boundary_a);
        a_shape_ok = a_shape_ok || body_samples_follow_source_curve(state, body);
      }
      if (almost_equal(body.boundary_a, patch.boundary_b, 1e-9)) {
        b_ok = b_ok || sampled_boundary_is_g1(body, patch, patch.boundary_b);
        b_source_ok = b_source_ok || boundary_matches_source_curve(state, body, patch.boundary_b);
        b_shape_ok = b_shape_ok || body_samples_follow_source_curve(state, body);
      }
      if (almost_equal(body.boundary_b, patch.boundary_b, 1e-9)) {
        b_ok = b_ok || sampled_boundary_is_g1(body, patch, patch.boundary_b);
        b_source_ok = b_source_ok || boundary_matches_source_curve(state, body, patch.boundary_b);
        b_shape_ok = b_shape_ok || body_samples_follow_source_curve(state, body);
      }
    }
    return a_ok && b_ok && a_source_ok && b_source_ok && a_shape_ok && b_shape_ok;
  }
  return false;
}

bool C755_backbone_sharp_jumper_boundaries_are_g1_with_edge_bodies() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {5.0, 8.660254037844386, 0.0}};
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) return false;
  for (const city::wire::VisualCurvePart& jumper : state.view().visual_curve_parts().parts) {
    if (jumper.kind != city::wire::VisualCurvePartKind::kJumper) continue;
    bool a_ok = false;
    bool b_ok = false;
    for (const city::wire::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != city::wire::VisualCurvePartKind::kEdgeBody) continue;
      if (almost_equal(body.boundary_a, jumper.boundary_a, 1e-9) ||
          almost_equal(body.boundary_b, jumper.boundary_a, 1e-9)) {
        a_ok = a_ok || sampled_boundary_is_g1(body, jumper, jumper.boundary_a);
      }
      if (almost_equal(body.boundary_a, jumper.boundary_b, 1e-9) ||
          almost_equal(body.boundary_b, jumper.boundary_b, 1e-9)) {
        b_ok = b_ok || sampled_boundary_is_g1(body, jumper, jumper.boundary_b);
      }
    }
    return a_ok && b_ok;
  }
  return false;
}

bool C638_backbone_visual_curve_parts_are_finite() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = poly3_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.5, 0.0}};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || state.view().visual_curve_parts().parts.empty() || !finite_visual_curve_parts(state)) {
    return false;
  }
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    const city::wire::Vec3d chord{part.boundary_b.x - part.boundary_a.x, part.boundary_b.y - part.boundary_a.y,
                                  part.boundary_b.z - part.boundary_a.z};
    const double chord_length = std::sqrt(chord.x * chord.x + chord.y * chord.y + chord.z * chord.z);
    if (chord_length > 1e-9 && sampled_curve_length(part) > chord_length * 2.0) {
      return false;
    }
  }
  return true;
}

bool C639_backbone_node_patch_curve_is_not_straight_chord() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok) {
    return false;
  }
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    if (part.samples.size() < 5) {
      return false;
    }
    double max_distance = 0.0;
    for (std::size_t i = 1; i + 1 < part.samples.size(); ++i) {
      max_distance = std::max(max_distance, distance_to_chord(part.samples[i], part.boundary_a, part.boundary_b));
    }
    return max_distance > 0.02;
  }
  return false;
}

bool C640_backbone_node_patch_exposes_bezier_debug_controls() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok) {
    return false;
  }
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    if (part.section_count != 1 || !part.has_attachment_point || part.passes_attachment_point ||
        part.bezier_control_points.size() != 4) {
      return false;
    }
    return almost_equal(part.bezier_control_points.front(), part.boundary_a, 1e-9) &&
           almost_equal(part.bezier_control_points.back(), part.boundary_b, 1e-9);
  }
  return false;
}

bool C641_backbone_pole_tilt_refreshes_visual_curve_parts() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_pole_ids.empty() || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId pole_id = generated.value.generated_pole_ids.front();
  const city::wire::ObjectId span_id = generated.value.generated_span_ids.front();
  const city::wire::VisualCurvePart* before_body = nullptr;
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == city::wire::VisualCurvePartKind::kEdgeBody && part.source_span_id == span_id) {
      before_body = &part;
      break;
    }
  }
  if (before_body == nullptr) {
    return false;
  }
  const city::wire::Vec3d before_boundary = before_body->boundary_a;
  const auto tilted = state.ApplyPoleTilt({pole_id}, 12.0);
  if (!tilted.ok || !tilted.value) {
    return false;
  }
  const city::wire::SpanLayoutView layout = state.span_layout(span_id);
  if (!layout.has_layout()) {
    return false;
  }
  const city::wire::VisualCurvePart* after_body = nullptr;
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == city::wire::VisualCurvePartKind::kEdgeBody && part.source_span_id == span_id) {
      after_body = &part;
      break;
    }
  }
  if (after_body == nullptr) {
    return false;
  }
  const bool pole_is_start = layout.entry->start.endpoint_node_id == pole_id;
  const city::wire::Vec3d expected =
      pole_is_start ? layout.entry->start.endpoint_world : layout.entry->end.endpoint_world;
  const city::wire::Vec3d actual = pole_is_start ? after_body->boundary_a : after_body->boundary_b;
  return !almost_equal(before_boundary, actual, 1e-9) && almost_equal(actual, expected, 1e-9);
}

bool C659_backbone_draw_time_tilt_materializes_ports_before_spans() {
  city::wire::CoreState untilted_state;
  city::wire::BackboneSpec untilted_req = line_req(untilted_state);
  const auto untilted = untilted_state.GenerateFromBackboneSpec(untilted_req);
  if (!untilted.ok || untilted.value.generated_pole_ids.empty() || untilted.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId untilted_pole_id = untilted.value.generated_pole_ids.front();
  const city::wire::ObjectId untilted_span_id = untilted.value.generated_span_ids.front();
  const city::wire::Pole* untilted_pole = untilted_state.view().poles().find(untilted_pole_id);
  const city::wire::SpanLayoutView untilted_layout = untilted_state.span_layout(untilted_span_id);
  if (untilted_pole == nullptr || !untilted_layout.has_layout() || untilted_pole->tilt_magnitude_deg != 0.0) {
    return false;
  }

  city::wire::CoreState generated_state;
  city::wire::BackboneSpec generated_req = line_req(generated_state);
  generated_req.pole_placement.enable_tilt = true;
  generated_req.pole_placement.max_tilt_deg = 12.0;
  const auto generated = generated_state.GenerateFromBackboneSpec(generated_req);
  if (!generated.ok || generated.value.generated_pole_ids.empty() || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId generated_pole_id = generated.value.generated_pole_ids.front();
  const city::wire::ObjectId generated_span_id = generated.value.generated_span_ids.front();
  const city::wire::Pole* generated_pole = generated_state.view().poles().find(generated_pole_id);
  const city::wire::Span* generated_span = generated_state.view().spans().find(generated_span_id);
  const city::wire::SpanLayoutView generated_layout = generated_state.span_layout(generated_span_id);
  if (generated_pole == nullptr || generated_span == nullptr || !generated_layout.has_layout() ||
      generated_pole->tilt_magnitude_deg <= 0.0) {
    return false;
  }
  const city::wire::Port* generated_port_a = generated_state.view().ports().find(generated_span->port_a_id);
  const city::wire::Port* generated_port_b = generated_state.view().ports().find(generated_span->port_b_id);
  if (generated_port_a == nullptr || generated_port_b == nullptr) {
    return false;
  }
  const bool generated_pole_is_start = generated_port_a->owner_pole_id == generated_pole_id;
  const city::wire::Port* generated_port = generated_pole_is_start ? generated_port_a : generated_port_b;
  if (generated_port->owner_pole_id != generated_pole_id) {
    return false;
  }
  const city::wire::Vec3d generated_endpoint =
      generated_pole_is_start ? generated_layout.entry->start.endpoint_world : generated_layout.entry->end.endpoint_world;
  const city::wire::Span* untilted_span = untilted_state.view().spans().find(untilted_span_id);
  if (untilted_span == nullptr) {
    return false;
  }
  const city::wire::Port* untilted_port_a = untilted_state.view().ports().find(untilted_span->port_a_id);
  const bool untilted_pole_is_start = untilted_port_a != nullptr && untilted_port_a->owner_pole_id == untilted_pole_id;
  const city::wire::Vec3d untilted_endpoint =
      untilted_pole_is_start ? untilted_layout.entry->start.endpoint_world : untilted_layout.entry->end.endpoint_world;
  if (!almost_equal(generated_endpoint, generated_port->world_position, 1e-9) ||
      almost_equal(generated_endpoint, untilted_endpoint, 1e-6)) {
    return false;
  }

  city::wire::CoreState apply_state;
  const auto apply_generated = apply_state.GenerateFromBackboneSpec(line_req(apply_state));
  if (!apply_generated.ok || apply_generated.value.generated_pole_ids.empty()) {
    return false;
  }
  const auto applied = apply_state.ApplyPoleTilt({apply_generated.value.generated_pole_ids.front()}, 12.0);
  const city::wire::Pole* applied_pole = apply_state.view().poles().find(apply_generated.value.generated_pole_ids.front());
  if (!applied.ok || !applied.value || applied_pole == nullptr) {
    return false;
  }
  return almost_equal(generated_pole->tilt_magnitude_deg, applied_pole->tilt_magnitude_deg, 1e-9) &&
         almost_equal(generated_pole->world_transform.rotation_euler_deg.x,
                      applied_pole->world_transform.rotation_euler_deg.x, 1e-9) &&
         almost_equal(generated_pole->world_transform.rotation_euler_deg.y,
                      applied_pole->world_transform.rotation_euler_deg.y, 1e-9);
}

bool C642_backbone_edge_body_uses_formal_sag_curve() {
  city::wire::CoreState state;
  city::wire::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  settings.sag_factor = 0.03;
  settings.curve_samples = 8;
  if (!state.UpdateGeometrySettings(settings).ok) {
    return false;
  }
  city::wire::BackboneSpec req = line_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId span_id = generated.value.generated_span_ids.front();
  const city::wire::CurveCacheEntry* formal = state.find_curve_cache(span_id);
  if (formal == nullptr) {
    return false;
  }
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kEdgeBody || part.source_span_id != span_id) {
      continue;
    }
    if (part.sag_method != city::wire::VisualCurveSagMethod::kParabolic || part.sag_m <= 0.0 ||
        part.tangent_a.z >= -1e-6 || part.tangent_b.z >= -1e-6 ||
        part.samples.size() <= 8 || part.samples.size() != formal->points.size()) {
      return false;
    }
    for (std::size_t i = 0; i < part.samples.size(); ++i) {
      if (!almost_equal(part.samples[i], formal->points[i], 1e-9)) {
        return false;
      }
    }
    return true;
  }
  return false;
}

bool C643_backbone_node_patch_uses_inner_fillet_with_attachment_reference() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || generated.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId middle_pole_id = generated.value.generated_pole_ids[1];
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    const bool attachment_is_owned_port =
        std::any_of(state.view().edit_state().ports.items().begin(), state.view().edit_state().ports.items().end(),
                    [&](const city::wire::Port& port) {
                      return port.owner_pole_id == middle_pole_id &&
                             almost_equal(port.world_position, part.attachment_point, 1e-9);
                    });
    const bool samples_attachment =
        std::any_of(part.samples.begin(), part.samples.end(),
                    [&](const city::wire::Vec3d& point) { return almost_equal(point, part.attachment_point, 1e-9); });
    return part.has_attachment_point && !part.passes_attachment_point && part.section_count == 1 &&
           attachment_is_owned_port && !samples_attachment &&
           !almost_equal(part.attachment_point, part.boundary_a, 1e-9) &&
           !almost_equal(part.attachment_point, part.boundary_b, 1e-9);
  }
  return false;
}

bool C644_backbone_patch_boundaries_extend_sag_tangents_below_attachment() {
  city::wire::CoreState state;
  city::wire::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  settings.sag_factor = 0.03;
  if (!state.UpdateGeometrySettings(settings).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  for (const city::wire::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != city::wire::VisualCurvePartKind::kNodePatch || !patch.has_attachment_point) {
      continue;
    }
    const city::wire::Vec3d attachment_to_a{patch.boundary_a.x - patch.attachment_point.x,
                                            patch.boundary_a.y - patch.attachment_point.y,
                                            patch.boundary_a.z - patch.attachment_point.z};
    const city::wire::Vec3d attachment_to_b{patch.boundary_b.x - patch.attachment_point.x,
                                            patch.boundary_b.y - patch.attachment_point.y,
                                            patch.boundary_b.z - patch.attachment_point.z};
    return patch.boundary_a.z < patch.attachment_point.z - 1e-6 &&
           patch.boundary_b.z < patch.attachment_point.z - 1e-6 &&
           tangent_compatible(attachment_to_a, patch.tangent_a) &&
           tangent_compatible(attachment_to_b, patch.tangent_b);
  }
  return false;
}

bool C645_backbone_node_patch_inner_fillet_keeps_curvature() {
  city::wire::CoreState state;
  city::wire::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  if (!state.UpdateGeometrySettings(settings).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  for (const city::wire::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != city::wire::VisualCurvePartKind::kNodePatch || patch.section_count != 1 ||
        patch.bezier_control_points.size() != 4) {
      continue;
    }
    return distance_to_chord(patch.bezier_control_points[1], patch.boundary_a, patch.boundary_b) > 1e-4 &&
           distance_to_chord(patch.bezier_control_points[2], patch.boundary_a, patch.boundary_b) > 1e-4;
  }
  return false;
}

bool C646_backbone_node_patch_turns_monotonically_inside_corner() {
  city::wire::CoreState state;
  city::wire::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  if (!state.UpdateGeometrySettings(settings).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  for (const city::wire::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != city::wire::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    const bool samples_inside =
        std::all_of(patch.samples.begin(), patch.samples.end(), [&](const city::wire::Vec3d& point) {
          return point_in_xy_triangle(point, patch.boundary_a, patch.attachment_point, patch.boundary_b);
        });
    return samples_inside && sampled_xy_turn_is_monotonic(patch);
  }
  return false;
}

bool C647_backbone_node_patch_uses_incident_cable_appearance() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  for (const city::wire::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != city::wire::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    const auto bundle_template_it = state.view().bundle_templates().find(patch.bundle_template_id);
    WIRE_TEST_EXPECT(bundle_template_it != state.view().bundle_templates().end(),
                     "NodePatch bundle template is missing");
    const auto cable_template_it =
        state.view().cable_templates().find(bundle_template_it->second.cable_template_id);
    WIRE_TEST_EXPECT(cable_template_it != state.view().cable_templates().end(),
                     "NodePatch cable template is missing");
    WIRE_TEST_EXPECT(
        almost_equal(patch.wire_radius_m, cable_template_it->second.outer_diameter_m * 0.5, 1e-12) &&
            patch.color_rgba == cable_template_it->second.color_rgba &&
            patch.material_style == cable_template_it->second.material_style,
        "NodePatch appearance does not match its authoritative CableTemplate");
    std::size_t matching_bodies = 0;
    for (const city::wire::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != city::wire::VisualCurvePartKind::kEdgeBody ||
          body.source_bundle_id != patch.source_bundle_id ||
          body.bundle_template_id != patch.bundle_template_id || body.lane_index != patch.lane_index) {
        continue;
      }
      const bool touches_patch =
          almost_equal(body.boundary_a, patch.boundary_a, 1e-9) ||
          almost_equal(body.boundary_b, patch.boundary_a, 1e-9) ||
          almost_equal(body.boundary_a, patch.boundary_b, 1e-9) ||
          almost_equal(body.boundary_b, patch.boundary_b, 1e-9);
      if (!touches_patch) {
        continue;
      }
      if (!almost_equal(body.wire_radius_m, patch.wire_radius_m, 1e-12) ||
          body.color_rgba != patch.color_rgba || body.material_style != patch.material_style) {
        return false;
      }
      ++matching_bodies;
    }
    return matching_bodies == 2;
  }
  return false;
}

bool C655_backbone_node_patch_grouping_uses_band_identity() {
  const std::filesystem::path source = repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "curve_parts.cpp";
  const std::filesystem::path pipeline_source =
      repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  std::string pipeline;
  if (!file_text(source, &cpp) || !file_text(pipeline_source, &pipeline)) {
    return false;
  }
  std::string key_body;
  if (!function_body(cpp, "bool same_key(const curve_patch_key& a, const curve_patch_key& b)", &key_body)) {
    return false;
  }
  return contains_text(cpp, "int band_id = 0") && contains_text(cpp, "PoleTypeId pole_type_id") &&
         contains_text(cpp, "placement_band_id") && !contains_text(cpp, "band_id_for_port") &&
         contains_text(key_body, "a.band_id == b.band_id") &&
         contains_text(key_body, "a.pole_type_id == b.pole_type_id") &&
         contains_text(cpp, "state.view().backbone().row_continuities") &&
         contains_text(cpp, "continuity_pairs_by_patch_key") &&
         !contains_text(cpp, "same_bundle") &&
         !contains_text(cpp, "shared_port") &&
         !contains_text(cpp, "same_endpoint_point") &&
         !contains_text(cpp, "explicit_fallback_candidates") &&
         contains_text(pipeline, "edge_bundle_by_link_bundle") &&
         !contains_text(pipeline, "pair_bindings") &&
         !contains_text(pipeline, "binding_bundle_id") &&
         contains_text(cpp, "backbone internal: row continuity endpoint is missing") &&
         contains_text(cpp, "backbone internal: patch endpoint tangent missing") &&
         !contains_text(cpp, "\"row continuity endpoint is missing\"") &&
         !contains_text(cpp, "\"patch endpoint tangent missing\"") &&
         !contains_text(cpp, "\"multiple overlapping connectivity-owned patch pairs\"");
}

bool C656_backbone_node_patch_does_not_mix_base_and_extra_sections() {
  city::wire::CoreState state;
  if (!enable_two_extra_lv_population(state)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  bool saw_patch = false;
  for (const city::wire::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != city::wire::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    saw_patch = true;
    bool have_reference = false;
    city::wire::CableSectionKey reference{};
    for (const city::wire::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != city::wire::VisualCurvePartKind::kEdgeBody || !body.has_section_key ||
          !touches_patch_boundary(body, patch)) {
        continue;
      }
      if (!have_reference) {
        reference = body.section_key;
        have_reference = true;
      } else if (!same_visual_cable_section_family(reference, body.section_key)) {
        return false;
      }
    }
  }
  return saw_patch;
}

bool C657_backbone_node_patch_does_not_mix_extra_instance_indices() {
  city::wire::CoreState state;
  if (!enable_two_extra_lv_population(state)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  bool saw_extra_body = false;
  for (const city::wire::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
    saw_extra_body = saw_extra_body || (body.kind == city::wire::VisualCurvePartKind::kEdgeBody &&
                                        body.has_section_key && !body.section_key.is_base());
  }
  if (!saw_extra_body) {
    return false;
  }
  bool saw_extra_patch = false;
  for (const city::wire::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != city::wire::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    std::size_t extra_instance_index = 0;
    std::size_t matching_extra_bodies = 0;
    bool saw_extra = false;
    for (const city::wire::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != city::wire::VisualCurvePartKind::kEdgeBody || !body.has_section_key ||
          body.section_key.is_base() || !touches_patch_boundary(body, patch)) {
        continue;
      }
      if (!saw_extra) {
        extra_instance_index = body.section_key.instance_index;
        saw_extra = true;
      } else if (extra_instance_index != body.section_key.instance_index) {
        return false;
      }
      ++matching_extra_bodies;
    }
    if (saw_extra && matching_extra_bodies == 2) {
      saw_extra_patch = true;
    } else if (saw_extra) {
      return false;
    }
  }
  return saw_extra_patch;
}

bool C665_backbone_midair_attachment_uses_derived_curve() {
  struct Result {
    bool ok = false;
    int stage = 0;
    city::wire::Vec3d source_tangent{};
  };
  const auto run = [](city::wire::PathDirectionMode direction_mode) {
    Result result{};
    city::wire::CoreState state;
    city::wire::GeometrySettings geometry = state.view().geometry_settings();
    geometry.sag_enabled = true;
    if (!state.UpdateGeometrySettings(geometry).ok) return result;
    result.stage = 1;
    city::wire::BackboneSpec source_request = line_req(state);
    source_request.direction_mode = direction_mode;
    const auto generated = state.GenerateFromBackboneSpec(source_request);
    if (!generated.ok || generated.value.generated_span_ids.size() != 1 ||
        state.view().backbone().edges.size() != 1) return result;
    result.stage = 2;

    const city::wire::ObjectId source_span_id = generated.value.generated_span_ids.front();
    const city::wire::CurveCacheEntry* source_curve = state.find_curve_cache(source_span_id);
    const city::wire::SavedBackboneEdge edge = state.view().backbone().edges.front();
    const city::wire::SavedBackboneNode* node_a = state.view().backbone_node(edge.node_a);
    const city::wire::SavedBackboneNode* node_b = state.view().backbone_node(edge.node_b);
    const auto attachment = state.view().source_edge_projection_world(
        edge.edge_id, edge.node_a,
        city::wire::kDefaultLowVoltageBundleTemplateId, 0, 0.5);
    if (source_curve == nullptr || node_a == nullptr || node_b == nullptr ||
        !attachment.has_value()) return result;
    result.stage = 3;
    const city::wire::Span* source_span = state.view().spans().find(source_span_id);
    const city::wire::Port* port_a = source_span == nullptr
                                         ? nullptr
                                         : state.view().ports().find(source_span->port_a_id);
    const city::wire::Port* port_b = source_span == nullptr
                                         ? nullptr
                                         : state.view().ports().find(source_span->port_b_id);
    if (port_a == nullptr || port_b == nullptr) return result;
    const city::wire::Vec3d chord_mid{
        (port_a->world_position.x + port_b->world_position.x) * 0.5,
        (port_a->world_position.y + port_b->world_position.y) * 0.5,
        (port_a->world_position.z + port_b->world_position.z) * 0.5};
    if (!almost_equal(*attachment, source_curve->detail.EvaluatePosition(0.5), 1e-9) ||
        attachment->z >= chord_mid.z - 1e-4) return result;
    result.stage = 4;

    city::wire::PickResult pick{};
    pick.hit_kind = city::wire::PickHitKind::kSegment;
    pick.hit_pos_world = city::wire::ScaleVec(node_a->position + node_b->position, 0.5);
    pick.has_segment_endpoints = true;
    pick.segment_node_a_id = edge.node_a;
    pick.segment_node_b_id = edge.node_b;
    pick.segment_endpoint_a_world = node_a->position;
    pick.segment_endpoint_b_world = node_b->position;
    city::wire::ResolveBranchPickOptions options{};
    options.selected_bundle_template_ids = {
        city::wire::kDefaultLowVoltageBundleTemplateId};
    const auto resolved = state.ResolveBranchPick(pick, options);
    if (!resolved.ok) return result;
    result.stage = 5;

    city::wire::BackboneSpec branch = line_req(state);
    branch.path.polyline = {resolved.value.position,
                            resolved.value.position + city::wire::Vec3d{-4.0, 8.0, 0.0}};
    city::wire::BackboneInputSpec::NodeSpec source_node{};
    source_node.point_index = 0;
    source_node.support_kind = resolved.value.support_kind;
    source_node.node_id = resolved.value.resolved_node_id;
    branch.path.node_specs = {source_node};
    const auto branched = state.GenerateFromBackboneSpec(branch);
    if (!branched.ok || branched.value.generated_span_ids.size() != 1) return result;
    result.stage = 6;
    const city::wire::ObjectId branch_span_id = branched.value.generated_span_ids.front();
    const city::wire::SpanLayoutView branch_layout = state.span_layout(branch_span_id);
    if (!branch_layout.has_layout()) return result;
    const city::wire::LayoutEndpoint* source_endpoint =
        branch_layout.entry->start.source_projection.valid()
            ? &branch_layout.entry->start
            : branch_layout.entry->end.source_projection.valid()
                  ? &branch_layout.entry->end
                  : nullptr;
    if (source_endpoint == nullptr) return result;
    result.stage = 7;

    const city::wire::VisualCurvePart* lead = nullptr;
    const city::wire::VisualCurvePart* branch_body = nullptr;
    for (const city::wire::VisualCurvePart& part :
         state.view().visual_curve_parts().parts) {
      if (part.source_span_id != branch_span_id) continue;
      if (part.kind == city::wire::VisualCurvePartKind::kLead) lead = &part;
      if (part.kind == city::wire::VisualCurvePartKind::kEdgeBody) branch_body = &part;
    }
    if (lead == nullptr || branch_body == nullptr) return result;
    result.stage = 71;
    if (lead->samples.size() < 2 || !lead->has_attachment_point ||
        !lead->passes_attachment_point) return result;
    result.stage = 72;
    if (!almost_equal(lead->boundary_a, lead->attachment_point, 1e-9) ||
        !almost_equal(lead->boundary_a, *attachment, 1e-9)) return result;
    result.stage = 73;
    if (!almost_equal(branch_body->boundary_a, lead->boundary_b, 1e-9) &&
        !almost_equal(branch_body->boundary_b, lead->boundary_b, 1e-9)) {
      result.stage = 731;
      return result;
    }
    result.stage = 74;
    const city::wire::Vec3d body_tangent =
        almost_equal(branch_body->boundary_a, lead->boundary_b, 1e-9)
            ? branch_body->tangent_a
            : branch_body->tangent_b;
    if (!tangent_compatible(lead->tangent_b, body_tangent)) return result;
    result.stage = 8;
    for (const city::wire::Vec3d& point : lead->samples) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) return result;
    }

    const city::wire::SourceEdgeProjectionRef& projection =
        source_endpoint->source_projection;
    const bool source_forward = projection.from_node_id == edge.node_a;
    const double u = source_forward ? projection.t : 1.0 - projection.t;
    source_curve = state.find_curve_cache(source_span_id);
    if (source_curve == nullptr) return result;
    city::wire::Vec3d expected_tangent = source_curve->detail.EvaluateTangent(u);
    if (!source_forward) expected_tangent = city::wire::ScaleVec(expected_tangent, -1.0);
    result.source_tangent = lead->tangent_a;
    if (!tangent_compatible(lead->tangent_a, expected_tangent)) return result;
    result.ok = true;
    result.stage = 9;
    result.source_tangent = lead->tangent_a;
    return result;
  };

  const Result forward = run(city::wire::PathDirectionMode::kForward);
  const Result reverse = run(city::wire::PathDirectionMode::kReverse);
  if (!forward.ok || !reverse.ok) {
    test_registry::SetFailureReason(
        "midair lead contract failed at forward stage " +
        std::to_string(forward.stage) + " reverse stage " +
        std::to_string(reverse.stage));
  }
  return forward.ok && reverse.ok &&
         !tangent_compatible(forward.source_tangent, reverse.source_tangent);
}

bool C666_backbone_terminal_extension_creates_connectivity_patch() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const auto endpoint_it = std::max_element(
      first.value.generated_pole_ids.begin(), first.value.generated_pole_ids.end(),
      [&](city::wire::ObjectId a, city::wire::ObjectId b) {
        return state.view().poles().find(a)->world_transform.position.x <
               state.view().poles().find(b)->world_transform.position.x;
      });
  const city::wire::ObjectId endpoint_pole_id = *endpoint_it;
  const city::wire::Pole* endpoint_pole = state.view().poles().find(endpoint_pole_id);
  const city::wire::SavedBackboneNode* endpoint_node =
      state.view().backbone_node_for_pole(endpoint_pole_id);
  if (endpoint_pole == nullptr || endpoint_node == nullptr) {
    return false;
  }
  const city::wire::ObjectId endpoint_node_id = endpoint_node->node_id;
  city::wire::BackboneSpec extension = line_req(state);
  extension.path.polyline = {endpoint_pole->world_transform.position, {24.0, 6.0, 0.0}};
  extension.path.node_specs = {pole_spec(0, endpoint_pole_id)};
  const auto second = state.GenerateFromBackboneSpec(extension);
  if (!second.ok) {
    return false;
  }
  const bool found = std::any_of(
      state.view().visual_curve_parts().parts.begin(), state.view().visual_curve_parts().parts.end(),
      [&](const city::wire::VisualCurvePart& part) {
        return part.kind == city::wire::VisualCurvePartKind::kNodePatch &&
               part.source_node_id == endpoint_node_id && part.incident_edge_ids.size() == 2;
      });
  return found;
}

bool C667_backbone_branch_preserves_through_patch() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId branch_pole_id = first.value.generated_pole_ids[1];
  const city::wire::Pole* branch_pole = state.view().poles().find(branch_pole_id);
  const city::wire::SavedBackboneNode* branch_node =
      state.view().backbone_node_for_pole(branch_pole_id);
  if (branch_pole == nullptr || branch_node == nullptr) {
    return false;
  }
  const city::wire::ObjectId branch_node_id = branch_node->node_id;
  std::vector<city::wire::ObjectId> through_edge_ids{};
  for (city::wire::ObjectId span_id : first.value.generated_span_ids) {
    const auto binding_it = state.view().backbone_index().span_bindings_by_span.find(span_id);
    if (binding_it == state.view().backbone_index().span_bindings_by_span.end() ||
        binding_it->second.empty()) {
      return false;
    }
    const city::wire::SavedBackboneSpanBinding& binding =
        state.view().backbone().span_bindings[binding_it->second.front()];
    const city::wire::SavedBackboneEdgeBundle* edge_bundle =
        state.view().backbone_edge_bundle(binding.edge_bundle_id);
    if (edge_bundle == nullptr) return false;
    through_edge_ids.push_back(edge_bundle->edge_id);
  }
  std::sort(through_edge_ids.begin(), through_edge_ids.end());
  city::wire::BackboneSpec branch = line_req(state);
  branch.path.polyline = {branch_pole->world_transform.position, {20.0, -6.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, branch_pole_id)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok || state.view().pole_frontier(branch_pole_id).edge_ids.size() != 3) {
    return false;
  }
  const bool found = std::any_of(
      state.view().visual_curve_parts().parts.begin(), state.view().visual_curve_parts().parts.end(),
      [&](const city::wire::VisualCurvePart& part) {
        return part.kind == city::wire::VisualCurvePartKind::kNodePatch &&
               part.source_node_id == branch_node_id && part.incident_edge_ids == through_edge_ids;
      });
  return found;
}

bool C691_cable_run_id_connects_through_sections() {
  city::wire::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  const auto bodies = base_edge_bodies(state);
  if (bodies.size() != 2 || !all_same_nonzero_run(bodies)) {
    return false;
  }
  const city::wire::CableRunId run_id = bodies.front()->cable_run_id;
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == city::wire::VisualCurvePartKind::kNodePatch && part.cable_run_id != run_id) {
      return false;
    }
  }
  return visual_curve_part_count(state, city::wire::VisualCurvePartKind::kNodePatch) > 0;
}

bool C692_cable_run_id_connects_terminal_extension() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const auto endpoint_it = std::max_element(
      first.value.generated_pole_ids.begin(), first.value.generated_pole_ids.end(),
      [&](city::wire::ObjectId a, city::wire::ObjectId b) {
        return state.view().poles().find(a)->world_transform.position.x <
               state.view().poles().find(b)->world_transform.position.x;
      });
  const city::wire::ObjectId endpoint_pole_id = *endpoint_it;
  const city::wire::Pole* endpoint_pole = state.view().poles().find(endpoint_pole_id);
  if (endpoint_pole == nullptr) {
    return false;
  }
  city::wire::BackboneSpec extension = line_req(state);
  extension.path.polyline = {endpoint_pole->world_transform.position, {24.0, 6.0, 0.0}};
  extension.path.node_specs = {pole_spec(0, endpoint_pole_id)};
  const auto second = state.GenerateFromBackboneSpec(extension);
  const auto bodies = base_edge_bodies(state);
  return second.ok && bodies.size() >= 2 && all_same_nonzero_run(bodies);
}

bool C693_cable_run_id_keeps_branch_and_dead_end_separate() {
  city::wire::CoreState branch_state;
  const auto first = branch_state.GenerateFromBackboneSpec(poly3_req(branch_state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const auto through_bodies = base_edge_bodies(branch_state);
  if (through_bodies.size() != 2 || !all_same_nonzero_run(through_bodies)) {
    return false;
  }
  const city::wire::CableRunId through_run_id = through_bodies.front()->cable_run_id;
  const city::wire::ObjectId branch_pole_id = first.value.generated_pole_ids[1];
  const city::wire::Pole* branch_pole = branch_state.view().poles().find(branch_pole_id);
  if (branch_pole == nullptr) {
    return false;
  }
  city::wire::BackboneSpec branch = line_req(branch_state);
  branch.path.polyline = {branch_pole->world_transform.position, {20.0, -6.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, branch_pole_id)};
  const auto second = branch_state.GenerateFromBackboneSpec(branch);
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (const city::wire::VisualCurvePart& part : branch_state.view().visual_curve_parts().parts) {
    if (part.kind == city::wire::VisualCurvePartKind::kEdgeBody &&
        contains_span(second.value.generated_span_ids, part.source_span_id) &&
        part.cable_run_id == through_run_id) {
      return false;
    }
  }

  city::wire::CoreState sharp_state;
  if (!prepare_two_lane_low_voltage_for_run_test(sharp_state)) {
    return false;
  }
  city::wire::BackboneSpec sharp = line_req(sharp_state);
  sharp.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {5.0, 8.660254037844386, 0.0}};
  const auto sharp_generated = sharp_state.GenerateFromBackboneSpec(sharp);
  const auto sharp_bodies = base_edge_bodies(sharp_state);
  return sharp_generated.ok && sharp_bodies.size() == 4 && all_distinct_nonzero_runs(sharp_bodies);
}

bool C694_cable_run_id_connects_population_instances() {
  city::wire::CoreState state;
  if (!enable_two_extra_lv_population(state)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  const auto base = base_edge_bodies(state);
  const auto extra1 = extra_edge_bodies(state, 1);
  const auto extra2 = extra_edge_bodies(state, 2);
  if (base.size() != 2 || extra1.size() != 2 || extra2.size() != 2 ||
      !all_same_nonzero_run(base) || !all_same_nonzero_run(extra1) || !all_same_nonzero_run(extra2)) {
    return false;
  }
  return base.front()->cable_run_id != extra1.front()->cable_run_id &&
         base.front()->cable_run_id != extra2.front()->cable_run_id &&
         extra1.front()->cable_run_id != extra2.front()->cable_run_id;
}

bool C695_cable_run_id_is_deterministic() {
  city::wire::CoreState a;
  city::wire::CoreState b;
  if (!enable_two_extra_lv_population(a) || !enable_two_extra_lv_population(b)) {
    return false;
  }
  const auto generated_a = a.GenerateFromBackboneSpec(poly3_req(a));
  const auto generated_b = b.GenerateFromBackboneSpec(poly3_req(b));
  if (!generated_a.ok || !generated_b.ok ||
      a.view().visual_curve_parts().parts.size() != b.view().visual_curve_parts().parts.size()) {
    return false;
  }
  std::vector<std::string> snapshot_a{};
  std::vector<std::string> snapshot_b{};
  const auto collect = [](const city::wire::CoreState& state, std::vector<std::string>* out) {
    for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
      if (part.kind == city::wire::VisualCurvePartKind::kSupplemental) {
        continue;
      }
      if (part.cable_run_id == 0) {
        return false;
      }
      std::ostringstream ss;
      ss << static_cast<int>(part.kind) << ":" << part.source_span_id << ":" << part.lane_index << ":"
         << (part.has_section_key ? part.section_key.instance_index : 0) << ":" << part.cable_run_id;
      out->push_back(ss.str());
    }
    std::sort(out->begin(), out->end());
    return true;
  };
  return collect(a, &snapshot_a) && collect(b, &snapshot_b) && snapshot_a == snapshot_b;
}

bool C696_cable_run_id_is_visual_derived_only() {
  const std::filesystem::path source =
      repo_root() / "domains" / "wire" / "src" / "generation" / "backbone" / "curve_parts.cpp";
  const std::filesystem::path runtime_header =
      repo_root() / "domains" / "wire" / "include" / "city" / "wire" / "core_runtime_types.hpp";
  std::string cpp;
  std::string runtime;
  if (!file_text(source, &cpp) || !file_text(runtime_header, &runtime)) {
    return false;
  }
  std::string derive_body;
  std::string lookup_body;
  if (!function_body(cpp, "cable_run_assignments derive_cable_run_ids", &derive_body) ||
      !function_body(cpp, "CableRunId run_id_for_section", &lookup_body)) {
    return false;
  }
  return contains_text(cpp, "cable_run_id") && contains_text(runtime, "CableRunId cable_run_id") &&
         contains_text(derive_body, "patch_specs") && contains_text(derive_body, "index_by_section") &&
         contains_text(lookup_body, "index_by_section.find(key)") && !contains_text(lookup_body, "for (") &&
         !contains_text(derive_body, "SavedBackboneGraph") &&
         !contains_text(derive_body, "state.view()") && !contains_text(derive_body, "save_backbone");
}

} // namespace backbone_tests
namespace backbone_tests {

bool C764_straight_hv_model_assemblies_own_fixture_and_wire_placement() {
  constexpr city::wire::ModelAssemblyTemplateId kPoleAssembly = 9101;
  constexpr city::wire::ModelAssemblyTemplateId kRowAssembly = 9102;
  constexpr city::wire::ModelAssemblyTemplateId kEndpointAssembly = 9103;

  city::wire::CoreState baseline;
  city::wire::BackboneSpec baseline_request = line_req(baseline);
  baseline_request.bundles.clear();
  add_backbone_bundle(baseline_request, city::wire::BundleKind::kHighVoltage);
  baseline_request.bundles.front().placement_explicit = true;
  baseline_request.bundles.front().height_m = 9.2;
  baseline_request.bundles.front().lateral_m = -0.2;
  baseline_request.bundles.front().spacing_m = 0.45;
  const auto baseline_generated = baseline.GenerateFromBackboneSpec(baseline_request);
  if (!baseline_generated.ok) return false;

  city::wire::CoreState state;
  city::wire::BackboneSpec request = line_req(state);
  request.bundles.clear();
  add_backbone_bundle(request, city::wire::BundleKind::kHighVoltage);
  request.bundles.front().placement_explicit = true;
  request.bundles.front().height_m = 9.2;
  request.bundles.front().lateral_m = -0.2;
  request.bundles.front().spacing_m = 0.45;

  city::wire::ModelAssemblyTemplate pole_assembly{};
  pole_assembly.id = kPoleAssembly;
  pole_assembly.parts.push_back({1, "pole_body", 3, {}, city::wire::ModelFitMode::kPoleHeight, {}});
  city::wire::ModelAssemblyTemplate row_assembly{};
  row_assembly.id = kRowAssembly;
  city::wire::ModelAssemblyPart row_part{};
  row_part.part_id = 1;
  row_part.model_key = "hv_crossarm";
  row_part.descriptor_version = 5;
  row_part.fit_mode = city::wire::ModelFitMode::kRigid;
  row_part.sockets.push_back({"endpoint_mount", {0.0, 0.0, 0.04}, {0.0, 0.0, 1.0}});
  row_assembly.parts.push_back(row_part);
  row_assembly.endpoint_mount_socket = city::wire::AssemblySocketRef{1, "endpoint_mount"};
  constexpr double kMeshLowerEndHeightM = -2.0;
  constexpr double kPoleGroundRadiusM = 0.16;
  constexpr double kPoleTopRadiusM = 0.10;
  constexpr double kPoleHeightM = 10.0;
  const double mesh_lower_radius_m =
      kPoleGroundRadiusM + (kPoleTopRadiusM - kPoleGroundRadiusM) *
                              (kMeshLowerEndHeightM / kPoleHeightM);
  city::wire::Transformd belt_transform{};
  belt_transform.scale = {1.0 / mesh_lower_radius_m, 1.0 / mesh_lower_radius_m, 1.0};
  row_assembly.parts.push_back(
      {2, "pole_belt", 7, belt_transform, city::wire::ModelFitMode::kPoleRadial, {}});
  city::wire::ModelAssemblyTemplate endpoint_assembly{};
  endpoint_assembly.id = kEndpointAssembly;
  city::wire::ModelAssemblyPart endpoint_part{};
  endpoint_part.part_id = 1;
  endpoint_part.model_key = "hv_insulator";
  endpoint_part.descriptor_version = 11;
  endpoint_part.sockets.push_back({"wire", {0.0, 0.0, 0.18}, {1.0, 0.0, 0.0}});
  endpoint_assembly.parts.push_back(endpoint_part);
  endpoint_assembly.wire_socket = city::wire::AssemblySocketRef{1, "wire"};
  if (!state.RegisterModelAssemblyTemplate(pole_assembly).ok ||
      !state.RegisterModelAssemblyTemplate(row_assembly).ok ||
      !state.RegisterModelAssemblyTemplate(endpoint_assembly).ok) {
    return false;
  }

  city::wire::PoleTypeDefinition pole_type = state.view().pole_types().at(request.pole_type_id);
  pole_type.pole_visual_assembly_id = kPoleAssembly;
  pole_type.radius_base_m = kPoleGroundRadiusM;
  pole_type.radius_top_m = kPoleTopRadiusM;
  if (!state.UpdatePoleTypeDefinition(pole_type).ok) return false;
  const city::wire::BundleTemplateId hv_template_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage);
  city::wire::BundleTemplate hv = state.view().bundle_templates().at(hv_template_id);
  hv.row_fixture_assembly_id = kRowAssembly;
  hv.endpoint_fixture_assembly_id = kEndpointAssembly;
  if (!state.UpdateBundleTemplate(hv).ok) return false;

  const auto generated = state.GenerateFromBackboneSpec(request);
  if (!generated.ok || generated.value.generated_pole_ids.size() != 2 ||
      generated.value.generated_span_ids.size() != 3 ||
      state.view().poles().size() != baseline.view().poles().size() ||
      state.view().ports().size() != baseline.view().ports().size() ||
      state.view().spans().size() != baseline.view().spans().size() ||
      state.view().bundles().size() != baseline.view().bundles().size()) {
    return false;
  }
  std::string visual_geometry_error{};
  WIRE_TEST_EXPECT(
      hv_edge_body_xy_intersections_absent(state, &visual_geometry_error),
      visual_geometry_error);

  std::unordered_set<city::wire::ObjectId> unique_ports{};
  for (city::wire::ObjectId span_id : generated.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) return false;
    unique_ports.insert(span->port_a_id);
    unique_ports.insert(span->port_b_id);
    const city::wire::SpanLayoutEntry* layout = state.span_layout(span_id).entry;
    const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span_id);
    if (layout == nullptr || curve == nullptr || curve->detail.sample_points.size() < 2) return false;
    const auto endpoint_matches_socket = [&](const city::wire::LayoutEndpoint& endpoint) {
      const city::wire::Port* port = state.view().ports().find(endpoint.port_id);
      if (port == nullptr) return false;
      const auto fixture = std::find_if(
          state.view().visual_model_instances().instances.begin(),
          state.view().visual_model_instances().instances.end(),
          [&](const city::wire::VisualModelInstance& instance) {
            if (instance.model_key != "hv_insulator") return false;
            const city::wire::Vec3d local_socket =
                endpoint_part.sockets.front().local_position;
            const city::wire::Vec3d socket =
                instance.world_transform.position +
                city::wire::RotateEulerXYZDeg(
                    {local_socket.x * instance.world_transform.scale.x,
                     local_socket.y * instance.world_transform.scale.y,
                     local_socket.z * instance.world_transform.scale.z},
                    instance.world_transform.rotation_euler_deg);
            return almost_equal(socket, endpoint.endpoint_world, 1e-9);
          });
      if (fixture == state.view().visual_model_instances().instances.end()) return false;
      const city::wire::Vec3d local_socket = endpoint_part.sockets.front().local_position;
      const city::wire::Vec3d visible_socket =
          fixture->world_transform.position +
          city::wire::RotateEulerXYZDeg(
              {local_socket.x * fixture->world_transform.scale.x,
               local_socket.y * fixture->world_transform.scale.y,
               local_socket.z * fixture->world_transform.scale.z},
              fixture->world_transform.rotation_euler_deg);
      return almost_equal(endpoint.support_world, visible_socket, 1e-9) &&
             almost_equal(endpoint.endpoint_world, visible_socket, 1e-9);
    };
    if (!endpoint_matches_socket(layout->start) || !endpoint_matches_socket(layout->end) ||
        !almost_equal(curve->detail.sample_points.front(), layout->start.endpoint_world, 1e-9) ||
        !almost_equal(curve->detail.sample_points.back(), layout->end.endpoint_world, 1e-9)) {
      return false;
    }
  }
  if (unique_ports.size() != 6) return false;
  std::unordered_map<city::wire::ObjectId, std::vector<city::wire::Vec3d>> endpoint_roots_by_pole{};
  std::unordered_map<city::wire::ObjectId, std::vector<city::wire::Vec3d>> sockets_by_pole{};

  std::unordered_map<std::string, std::size_t> model_counts{};
  std::vector<std::string> stable_keys{};
  std::unordered_map<std::string, std::uint64_t> versions{};
  for (const city::wire::VisualModelInstance& instance : state.view().visual_model_instances().instances) {
    ++model_counts[instance.model_key];
    stable_keys.push_back(instance.stable_key);
    versions.emplace(instance.stable_key, instance.content_version);
    if (instance.model_key == "pole_body") {
      const std::size_t pole_id_begin = instance.stable_key.find(':') + 1;
      const std::size_t pole_id_end = instance.stable_key.find(':', pole_id_begin);
      if (pole_id_begin == 0 || pole_id_end == std::string::npos) return false;
      const city::wire::ObjectId pole_id = static_cast<city::wire::ObjectId>(
          std::stoull(instance.stable_key.substr(pole_id_begin, pole_id_end - pole_id_begin)));
      const city::wire::Pole* pole = state.view().poles().find(pole_id);
      if (pole == nullptr) return false;
      if (!almost_equal(instance.world_transform.scale.x, 1.0, 1e-9) ||
          !almost_equal(instance.world_transform.scale.y, 1.0, 1e-9) ||
          !almost_equal(state.view().pole_radius_at_height_m(*pole, 0.0), kPoleGroundRadiusM, 1e-9) ||
          !almost_equal(state.view().pole_radius_at_height_m(*pole, pole->height_m), kPoleTopRadiusM, 1e-9) ||
          !almost_equal(state.view().pole_radius_at_height_m(*pole, kMeshLowerEndHeightM),
                        kPoleGroundRadiusM, 1e-9)) {
        return false;
      }
    }
    if (instance.model_key == "pole_belt") {
      const std::size_t pole_id_begin = instance.stable_key.find(':') + 1;
      const std::size_t pole_id_end = instance.stable_key.find(':', pole_id_begin);
      if (pole_id_begin == 0 || pole_id_end == std::string::npos) return false;
      const city::wire::ObjectId pole_id = static_cast<city::wire::ObjectId>(
          std::stoull(instance.stable_key.substr(pole_id_begin, pole_id_end - pole_id_begin)));
      const city::wire::Pole* pole = state.view().poles().find(pole_id);
      if (pole == nullptr) return false;
      const city::wire::PoleFrame frame = city::wire::BuildPoleFrame(pole->world_transform, 0.0);
      const double placement_height_m =
          city::wire::WorldPointToLocal(frame, instance.world_transform.position).z;
      const double radius_at_placement_m =
          state.view().pole_radius_at_height_m(*pole, placement_height_m);
      const double radial_scale = radius_at_placement_m / mesh_lower_radius_m;
      if (!almost_equal(instance.world_transform.scale.x, radial_scale, 1e-9) ||
          !almost_equal(instance.world_transform.scale.y, radial_scale, 1e-9) ||
          !almost_equal(instance.world_transform.scale.z, 1.0, 1e-9)) {
        return false;
      }
    }
    if (instance.model_key == "hv_insulator") {
      const std::size_t port_id_begin = instance.stable_key.find(':') + 1;
      const std::size_t port_id_end = instance.stable_key.find(':', port_id_begin);
      if (port_id_begin == 0 || port_id_end == std::string::npos) return false;
      const city::wire::ObjectId port_id = static_cast<city::wire::ObjectId>(
          std::stoull(instance.stable_key.substr(port_id_begin, port_id_end - port_id_begin)));
      const city::wire::Port* port = state.view().ports().find(port_id);
      if (port == nullptr) return false;
      endpoint_roots_by_pole[port->owner_pole_id].push_back(instance.world_transform.position);
      const city::wire::Vec3d local_socket = endpoint_part.sockets.front().local_position;
      sockets_by_pole[port->owner_pole_id].push_back(
          instance.world_transform.position +
          city::wire::RotateEulerXYZDeg(
              {local_socket.x * instance.world_transform.scale.x,
               local_socket.y * instance.world_transform.scale.y,
               local_socket.z * instance.world_transform.scale.z},
              instance.world_transform.rotation_euler_deg));
    }
  }
  std::sort(stable_keys.begin(), stable_keys.end());
  if (model_counts["pole_body"] != 2 || model_counts["hv_crossarm"] != 2 ||
      model_counts["pole_belt"] != 2 || model_counts["hv_insulator"] != unique_ports.size()) {
    return false;
  }
  const auto separated_three = [](const std::vector<city::wire::Vec3d>& points) {
    if (points.size() != 3) return false;
    std::vector<double> ys{};
    for (const city::wire::Vec3d& point : points) ys.push_back(point.y);
    std::sort(ys.begin(), ys.end());
    return almost_equal(ys[1] - ys[0], 0.45, 1e-9) &&
           almost_equal(ys[2] - ys[1], 0.45, 1e-9);
  };
  for (const auto& [_, points] : endpoint_roots_by_pole) {
    if (!separated_three(points)) return false;
  }
  for (const auto& [_, points] : sockets_by_pole) {
    if (!separated_three(points)) return false;
  }

  city::wire::CoreState lower_end_state;
  if (!lower_end_state.RegisterModelAssemblyTemplate(row_assembly).ok ||
      !lower_end_state.RegisterModelAssemblyTemplate(endpoint_assembly).ok) {
    return false;
  }
  city::wire::PoleTypeDefinition lower_pole_type =
      lower_end_state.view().pole_types().at(request.pole_type_id);
  lower_pole_type.radius_base_m = kPoleGroundRadiusM;
  lower_pole_type.radius_top_m = kPoleTopRadiusM;
  if (!lower_end_state.UpdatePoleTypeDefinition(lower_pole_type).ok) return false;
  city::wire::BundleTemplate lower_hv = lower_end_state.view().bundle_templates().at(hv_template_id);
  lower_hv.row_fixture_assembly_id = kRowAssembly;
  lower_hv.endpoint_fixture_assembly_id = kEndpointAssembly;
  if (!lower_end_state.UpdateBundleTemplate(lower_hv).ok) return false;
  city::wire::BackboneSpec lower_request = line_req(lower_end_state);
  lower_request.bundles.clear();
  add_backbone_bundle(lower_request, city::wire::BundleKind::kHighVoltage);
  for (city::wire::BackboneBundleSpec& bundle : lower_request.bundles) {
    bundle.placement_explicit = true;
    bundle.height_m = 0.0;
    bundle.lateral_m = 0.0;
    bundle.spacing_m = 0.75;
  }
  const auto lower_generated = lower_end_state.GenerateFromBackboneSpec(lower_request);
  if (!lower_generated.ok) return false;
  std::size_t lower_belts = 0;
  for (const city::wire::VisualModelInstance& instance :
       lower_end_state.view().visual_model_instances().instances) {
    if (instance.model_key != "pole_belt") continue;
    ++lower_belts;
    const double expected_ground_scale = kPoleGroundRadiusM / mesh_lower_radius_m;
    if (!almost_equal(instance.world_transform.scale.x, expected_ground_scale, 1e-9) ||
        !almost_equal(instance.world_transform.scale.y, expected_ground_scale, 1e-9) ||
        !almost_equal(instance.world_transform.scale.z, 1.0, 1e-9)) {
      return false;
    }
  }
  if (lower_belts != 2) return false;

  const city::wire::ObjectId moved_pole_id = generated.value.generated_pole_ids.front();
  const city::wire::Pole* moved_pole = state.view().poles().find(moved_pole_id);
  if (moved_pole == nullptr) return false;
  city::wire::Transformd moved_transform = moved_pole->world_transform;
  moved_transform.position = moved_transform.position + city::wire::Vec3d{1.0, 2.0, 0.5};
  moved_transform.rotation_euler_deg = {7.0, -4.0, 13.0};
  if (!state.MovePole(moved_pole_id, moved_transform).ok) return false;
  std::vector<std::string> moved_keys{};
  bool moved_body_version_changed = false;
  bool moved_body_frame_matches = false;
  const std::string moved_body_prefix = "pole:" + std::to_string(moved_pole_id) + ":";
  const std::string moved_row_prefix = "row:" + std::to_string(moved_pole_id) + ":";
  std::unordered_set<city::wire::ObjectId> moved_port_ids{};
  for (city::wire::ObjectId port_id : unique_ports) {
    const city::wire::Port* port = state.view().ports().find(port_id);
    if (port != nullptr && port->owner_pole_id == moved_pole_id) moved_port_ids.insert(port_id);
  }
  std::size_t moved_endpoint_instances = 0;
  for (const city::wire::VisualModelInstance& instance : state.view().visual_model_instances().instances) {
    moved_keys.push_back(instance.stable_key);
    if (instance.stable_key.rfind(moved_body_prefix, 0) == 0) {
      const auto before = versions.find(instance.stable_key);
      moved_body_version_changed = before != versions.end() && before->second != instance.content_version &&
                                   almost_equal(instance.world_transform.position,
                                                 moved_transform.position, 1e-9);
      moved_body_frame_matches =
          almost_equal(instance.world_transform.rotation_euler_deg,
                       moved_transform.rotation_euler_deg, 1e-9);
    }
    const bool moved_row = instance.stable_key.rfind(moved_row_prefix, 0) == 0;
    bool moved_endpoint = false;
    for (city::wire::ObjectId port_id : moved_port_ids) {
      if (instance.stable_key.rfind("port:" + std::to_string(port_id) + ":", 0) != 0) continue;
      const city::wire::Port* port = state.view().ports().find(port_id);
      if (port == nullptr) return false;
      ++moved_endpoint_instances;
      moved_endpoint = true;
      break;
    }
    if (moved_row || moved_endpoint) {
      const auto before = versions.find(instance.stable_key);
      if (before == versions.end() || before->second == instance.content_version) {
        return false;
      }
    }
  }
  const city::wire::Pole* moved_pole_after = state.view().poles().find(moved_pole_id);
  if (moved_pole_after == nullptr) return false;
  for (city::wire::ObjectId span_id : generated.value.generated_span_ids) {
    const city::wire::SpanLayoutEntry* layout = state.span_layout(span_id).entry;
    if (layout == nullptr) return false;
    for (const city::wire::LayoutEndpoint* endpoint : {&layout->start, &layout->end}) {
      if (!moved_port_ids.contains(endpoint->port_id)) continue;
      const city::wire::Port* port = state.view().ports().find(endpoint->port_id);
      const city::wire::SavedBackbonePortBinding* binding =
          port == nullptr ? nullptr : state.view().backbone_port_binding_for_port(port->id);
      if (port == nullptr || binding == nullptr) return false;
      const std::string fixture_prefix = "port:" + std::to_string(port->id) + ":";
      const auto fixture = std::find_if(
          state.view().visual_model_instances().instances.begin(),
          state.view().visual_model_instances().instances.end(),
          [&](const city::wire::VisualModelInstance& instance) {
            return instance.model_key == "hv_insulator" &&
                   instance.stable_key.rfind(fixture_prefix, 0) == 0;
          });
      if (fixture == state.view().visual_model_instances().instances.end()) return false;
      const city::wire::Vec3d local_socket = endpoint_part.sockets.front().local_position;
      const city::wire::Vec3d visible_socket =
          fixture->world_transform.position +
          city::wire::RotateEulerXYZDeg(
              {local_socket.x * fixture->world_transform.scale.x,
               local_socket.y * fixture->world_transform.scale.y,
               local_socket.z * fixture->world_transform.scale.z},
              fixture->world_transform.rotation_euler_deg);
      if (!almost_equal(endpoint->support_world, visible_socket, 1e-9) ||
          !almost_equal(endpoint->endpoint_world, visible_socket, 1e-9)) {
        return false;
      }
    }
  }
  std::sort(moved_keys.begin(), moved_keys.end());
  return moved_keys == stable_keys && moved_body_version_changed && moved_body_frame_matches &&
         moved_endpoint_instances == moved_port_ids.size();
}

bool C765_branch_lowering_places_final_model_socket_on_curve_endpoint() {
  constexpr city::wire::ModelAssemblyTemplateId kRowAssembly = 9109;
  constexpr city::wire::ModelAssemblyTemplateId kEndpointAssembly = 9110;
  city::wire::CoreState state;

  city::wire::ModelAssemblyTemplate row_assembly{};
  row_assembly.id = kRowAssembly;
  city::wire::ModelAssemblyPart row_part{};
  row_part.part_id = 1;
  row_part.model_key = "hv_crossarm";
  row_part.descriptor_version = 12;
  row_part.fit_mode = city::wire::ModelFitMode::kRigid;
  row_part.sockets.push_back({"endpoint_mount", {0.0, 0.0, 0.04}, {0.0, 0.0, 1.0}});
  row_assembly.parts.push_back(row_part);
  row_assembly.endpoint_mount_socket =
      city::wire::AssemblySocketRef{1, "endpoint_mount"};
  if (!state.RegisterModelAssemblyTemplate(row_assembly).ok) return false;

  city::wire::ModelAssemblyTemplate endpoint_assembly{};
  endpoint_assembly.id = kEndpointAssembly;
  city::wire::ModelAssemblyPart endpoint_part{};
  endpoint_part.part_id = 1;
  endpoint_part.model_key = "hv_insulator";
  endpoint_part.descriptor_version = 13;
  endpoint_part.sockets.push_back({"wire", {0.0, 0.0, -0.25}, {1.0, 0.0, 0.0}});
  endpoint_assembly.parts.push_back(endpoint_part);
  endpoint_assembly.wire_socket = city::wire::AssemblySocketRef{1, "wire"};
  if (!state.RegisterModelAssemblyTemplate(endpoint_assembly).ok) return false;

  const city::wire::BundleTemplateId hv_template_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage);
  city::wire::BundleTemplate hv = state.view().bundle_templates().at(hv_template_id);
  hv.row_fixture_assembly_id = kRowAssembly;
  hv.endpoint_fixture_assembly_id = kEndpointAssembly;
  if (!state.UpdateBundleTemplate(hv).ok) return false;

  const std::vector<city::wire::ObjectId> branch_spans = lowering_branch_spans(state);
  if (branch_spans.empty()) return false;

  std::unordered_map<city::wire::ObjectId, double> port_down_offsets{};
  std::unordered_map<city::wire::ObjectId, std::vector<city::wire::Vec3d>> visible_sockets_by_pole{};
  bool saw_lowered_row = false;
  bool saw_lowered_socket = false;
  for (city::wire::ObjectId span_id : branch_spans) {
    const city::wire::SpanLayoutEntry* layout = state.span_layout(span_id).entry;
    const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span_id);
    if (layout == nullptr || curve == nullptr || curve->detail.sample_points.size() < 2) return false;
    for (const auto& endpoint_and_curve : {
             std::pair<const city::wire::LayoutEndpoint*, const city::wire::Vec3d*>{
                 &layout->start, &curve->detail.sample_points.front()},
             std::pair<const city::wire::LayoutEndpoint*, const city::wire::Vec3d*>{
                 &layout->end, &curve->detail.sample_points.back()}}) {
      const city::wire::LayoutEndpoint& endpoint = *endpoint_and_curve.first;
      const city::wire::Port* port = state.view().ports().find(endpoint.port_id);
      const city::wire::Pole* pole = port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
      const city::wire::SavedBackbonePortBinding* binding =
          port == nullptr ? nullptr : state.view().backbone_port_binding_for_port(port->id);
      if (port == nullptr || pole == nullptr || binding == nullptr ||
          !almost_equal(*endpoint_and_curve.second, endpoint.endpoint_world, 1e-9)) {
        return false;
      }
      const std::string fixture_prefix = "port:" + std::to_string(port->id) + ":";
      const auto fixture = std::find_if(
          state.view().visual_model_instances().instances.begin(),
          state.view().visual_model_instances().instances.end(),
          [&](const city::wire::VisualModelInstance& instance) {
            return instance.model_key == "hv_insulator" &&
                   instance.stable_key.rfind(fixture_prefix, 0) == 0;
          });
      if (fixture == state.view().visual_model_instances().instances.end()) return false;
      const city::wire::Vec3d local_socket = endpoint_part.sockets.front().local_position;
      const city::wire::Vec3d visible_socket =
          fixture->world_transform.position +
          city::wire::RotateEulerXYZDeg(
              {local_socket.x * fixture->world_transform.scale.x,
               local_socket.y * fixture->world_transform.scale.y,
               local_socket.z * fixture->world_transform.scale.z},
              fixture->world_transform.rotation_euler_deg);
      const city::wire::PoleFrame frame =
          city::wire::BuildPoleFrame(pole->world_transform, binding->layout_yaw_deg);
      const city::wire::Vec3d final_anchor =
          port->world_position - city::wire::ScaleVec(frame.up, endpoint.branch_down_offset_m);
      const city::wire::Vec3d expected_fixture_root =
          final_anchor + city::wire::ScaleVec(frame.up, row_part.sockets.front().local_position.z);
      const city::wire::Vec3d expected_wire_socket =
          expected_fixture_root + city::wire::ScaleVec(frame.up, endpoint_part.sockets.front().local_position.z);
      auto& pole_sockets = visible_sockets_by_pole[port->owner_pole_id];
      if (std::none_of(pole_sockets.begin(), pole_sockets.end(),
                       [&](const auto& value) {
                         return almost_equal(value, visible_socket, 1e-9);
                       })) {
        pole_sockets.push_back(visible_socket);
      }
      const auto [offset_it, inserted] =
          port_down_offsets.emplace(endpoint.port_id, endpoint.branch_down_offset_m);
      if (!inserted && !almost_equal(offset_it->second, endpoint.branch_down_offset_m, 1e-9)) {
        return false;
      }
      if (endpoint.default_lower_required) {
        if (endpoint.branch_down_offset_m <= 0.1 ||
            !almost_equal(fixture->world_transform.position, expected_fixture_root, 1e-9) ||
            !almost_equal(visible_socket, expected_wire_socket, 1e-9) ||
            almost_equal(endpoint.endpoint_world, port->world_position, 1e-9) ||
            !almost_equal(endpoint.support_world, endpoint.endpoint_world, 1e-9) ||
            !almost_equal(endpoint.endpoint_world, visible_socket, 1e-9) ||
            !almost_equal(*endpoint_and_curve.second, visible_socket, 1e-9)) {
          return false;
        }
        saw_lowered_row = saw_lowered_row || std::any_of(
            state.view().visual_model_instances().instances.begin(),
            state.view().visual_model_instances().instances.end(),
            [&](const city::wire::VisualModelInstance& instance) {
              return instance.model_key == "hv_crossarm" &&
                     instance.world_transform.position.z <
                         port->world_position.z - endpoint.branch_down_offset_m * 0.5;
            });
        saw_lowered_socket = true;
      } else if (!almost_equal(fixture->world_transform.position, expected_fixture_root, 1e-9) ||
                 !almost_equal(visible_socket, expected_wire_socket, 1e-9) ||
                 almost_equal(endpoint.endpoint_world, port->world_position, 1e-9) ||
                 !almost_equal(endpoint.support_world, endpoint.endpoint_world, 1e-9) ||
                 !almost_equal(endpoint.endpoint_world, visible_socket, 1e-9)) {
        return false;
      }
    }
  }

  std::unordered_set<std::string> derived_endpoint_keys{};
  for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.bundle_template_id != hv_template_id) continue;
    const city::wire::SavedBackboneEdgeBundle* edge_bundle =
        state.view().backbone_edge_bundle(binding.edge_bundle_id);
    if (edge_bundle == nullptr) return false;
    std::ostringstream key{};
    key << binding.row_key.node_id << ':' << binding.row_key.edge_id << ':'
        << edge_bundle->bundle_id << ':' << binding.lane_index;
    derived_endpoint_keys.insert(key.str());
  }
  std::unordered_set<std::string> model_keys{};
  std::size_t endpoint_instance_count = 0;
  for (const city::wire::VisualModelInstance& instance : state.view().visual_model_instances().instances) {
    if (!model_keys.insert(instance.stable_key).second) return false;
    if (instance.stable_key.rfind("port:", 0) == 0) ++endpoint_instance_count;
  }
  bool saw_three_distinct_sockets = false;
  for (const auto& [_, sockets] : visible_sockets_by_pole) {
    if (sockets.size() < 3) continue;
    for (std::size_t a = 0; a < sockets.size(); ++a) {
      for (std::size_t b = a + 1; b < sockets.size(); ++b) {
        if (city::wire::Length(sockets[a] - sockets[b]) <= 0.1) return false;
      }
    }
    saw_three_distinct_sockets = true;
  }
  return saw_lowered_socket && saw_lowered_row && saw_three_distinct_sockets &&
         endpoint_instance_count > 0 &&
         endpoint_instance_count <= derived_endpoint_keys.size();
}

bool C833_branch_down_override_zero_reaches_model_socket_plan() {
  constexpr city::wire::ModelAssemblyTemplateId kRowAssembly = 9133;
  constexpr city::wire::ModelAssemblyTemplateId kEndpointAssembly = 9134;
  city::wire::CoreState state;

  city::wire::ModelAssemblyTemplate row_assembly{};
  row_assembly.id = kRowAssembly;
  city::wire::ModelAssemblyPart row_part{};
  row_part.part_id = 1;
  row_part.model_key = "hv_crossarm";
  row_part.sockets.push_back({"endpoint_mount", {0.0, 0.0, 0.04}, {0.0, 0.0, 1.0}});
  row_assembly.parts.push_back(row_part);
  row_assembly.endpoint_mount_socket =
      city::wire::AssemblySocketRef{1, "endpoint_mount"};
  WIRE_TEST_EXPECT(state.RegisterModelAssemblyTemplate(row_assembly).ok,
                   "failed to register row assembly");

  city::wire::ModelAssemblyTemplate endpoint_assembly{};
  endpoint_assembly.id = kEndpointAssembly;
  city::wire::ModelAssemblyPart endpoint_part{};
  endpoint_part.part_id = 1;
  endpoint_part.model_key = "hv_insulator";
  endpoint_part.descriptor_version = 1;
  endpoint_part.sockets.push_back({"wire", {0.0, 0.0, -0.25}, {1.0, 0.0, 0.0}});
  endpoint_assembly.parts.push_back(endpoint_part);
  endpoint_assembly.wire_socket = city::wire::AssemblySocketRef{1, "wire"};
  WIRE_TEST_EXPECT(state.RegisterModelAssemblyTemplate(endpoint_assembly).ok,
                   "failed to register endpoint assembly");

  const city::wire::BundleTemplateId hv_template_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage);
  city::wire::BundleTemplate hv = state.view().bundle_templates().at(hv_template_id);
  hv.row_fixture_assembly_id = kRowAssembly;
  hv.endpoint_fixture_assembly_id = kEndpointAssembly;
  WIRE_TEST_EXPECT(state.UpdateBundleTemplate(hv).ok, "failed to attach HV fixture assemblies");

  const std::vector<city::wire::ObjectId> branch_spans = lowering_branch_spans(state);
  WIRE_TEST_EXPECT(!branch_spans.empty(), "lowering branch spans were not generated");

  city::wire::ObjectId span_id = city::wire::kInvalidObjectId;
  city::wire::ObjectId port_id = city::wire::kInvalidObjectId;
  bool is_start = false;
  double automatic_down = 0.0;
  for (city::wire::ObjectId candidate : branch_spans) {
    const city::wire::SpanLayoutEntry* layout = state.span_layout(candidate).entry;
    if (layout == nullptr) continue;
    for (const auto& endpoint_pair : {
             std::pair<const city::wire::LayoutEndpoint*, bool>{&layout->start, true},
             std::pair<const city::wire::LayoutEndpoint*, bool>{&layout->end, false}}) {
      const city::wire::LayoutEndpoint& endpoint = *endpoint_pair.first;
      if (!endpoint.default_lower_required || endpoint.branch_down_offset_m <= 0.1) continue;
      span_id = candidate;
      port_id = endpoint.port_id;
      is_start = endpoint_pair.second;
      automatic_down = endpoint.branch_down_offset_m;
      break;
    }
    if (span_id != city::wire::kInvalidObjectId) break;
  }
  WIRE_TEST_EXPECT(span_id != city::wire::kInvalidObjectId,
                   "no lowered endpoint was available for zero override");
  const auto override_result = state.SetSpanBranchDownOffsetOverride(span_id, 0.0);
  WIRE_TEST_EXPECT(override_result.ok, override_result.error);

  const city::wire::SpanLayoutEntry* layout = state.span_layout(span_id).entry;
  const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  WIRE_TEST_EXPECT(layout != nullptr && curve != nullptr && curve->detail.sample_points.size() >= 2,
                   "layout or curve missing after zero override");
  const city::wire::LayoutEndpoint& endpoint = is_start ? layout->start : layout->end;
  const city::wire::Vec3d& curve_endpoint =
      is_start ? curve->detail.sample_points.front() : curve->detail.sample_points.back();
  WIRE_TEST_EXPECT(endpoint.port_id == port_id, "override changed the target endpoint port");
  WIRE_TEST_EXPECT(almost_equal(endpoint.branch_down_offset_m, 0.0, 1e-9),
                   "layout final branch-down offset is not zero");
  WIRE_TEST_EXPECT(automatic_down > 0.1, "test did not capture an automatic lowering candidate");

  const city::wire::Port* port = state.view().ports().find(port_id);
  const city::wire::Pole* pole = port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
  const city::wire::SavedBackbonePortBinding* binding =
      port == nullptr ? nullptr : state.view().backbone_port_binding_for_port(port->id);
  WIRE_TEST_EXPECT(port != nullptr && pole != nullptr && binding != nullptr,
                   "port, pole, or backbone binding missing after zero override");
  const std::string fixture_prefix = "port:" + std::to_string(port_id) + ":";
  const auto fixture = std::find_if(
      state.view().visual_model_instances().instances.begin(),
      state.view().visual_model_instances().instances.end(),
      [&](const city::wire::VisualModelInstance& instance) {
        return instance.model_key == "hv_insulator" &&
               instance.stable_key.rfind(fixture_prefix, 0) == 0;
      });
  WIRE_TEST_EXPECT(fixture != state.view().visual_model_instances().instances.end(),
                   "endpoint fixture missing after zero override");

  const city::wire::PoleFrame frame =
      city::wire::BuildPoleFrame(pole->world_transform, binding->layout_yaw_deg);
  const city::wire::Vec3d expected_fixture_root =
      port->world_position + city::wire::ScaleVec(frame.up, row_part.sockets.front().local_position.z);
  const city::wire::Vec3d expected_wire_socket =
      expected_fixture_root + city::wire::ScaleVec(frame.up, endpoint_part.sockets.front().local_position.z);
  const city::wire::Vec3d automatic_wire_socket =
      port->world_position - city::wire::ScaleVec(frame.up, automatic_down) +
      city::wire::ScaleVec(frame.up, row_part.sockets.front().local_position.z +
                                         endpoint_part.sockets.front().local_position.z);
  const city::wire::Vec3d local_socket = endpoint_part.sockets.front().local_position;
  const city::wire::Vec3d visible_socket =
      fixture->world_transform.position +
      city::wire::RotateEulerXYZDeg(
          {local_socket.x * fixture->world_transform.scale.x,
           local_socket.y * fixture->world_transform.scale.y,
           local_socket.z * fixture->world_transform.scale.z},
          fixture->world_transform.rotation_euler_deg);
  WIRE_TEST_EXPECT(!almost_equal(expected_wire_socket, automatic_wire_socket, 1e-9),
                   "zero and automatic lowering expectations are indistinguishable");
  WIRE_TEST_EXPECT(almost_equal(fixture->world_transform.position, expected_fixture_root, 1e-9),
                   "fixture root still uses automatic lowering after zero override");
  WIRE_TEST_EXPECT(almost_equal(visible_socket, expected_wire_socket, 1e-9),
                   "visible socket does not use the zero override lowering");
  WIRE_TEST_EXPECT(almost_equal(endpoint.endpoint_world, expected_wire_socket, 1e-9),
                   "layout endpoint does not use the zero override socket");
  WIRE_TEST_EXPECT(almost_equal(curve_endpoint, expected_wire_socket, 1e-9),
                   "curve endpoint does not use the zero override socket");
  WIRE_TEST_EXPECT(!almost_equal(endpoint.endpoint_world, automatic_wire_socket, 1e-9),
                   "endpoint fell back to automatic lowering after zero override");
  return true;
}

bool C834_backbone_sharp_pair_lowering_reaches_model_socket() {
  constexpr city::wire::ModelAssemblyTemplateId kRowAssembly = 9817;
  constexpr city::wire::ModelAssemblyTemplateId kEndpointAssembly = 9818;
  constexpr double kRowMountZ = 0.04;
  constexpr double kWireSocketZ = 0.18;
  const city::wire::Vec3d d{12.0, -8.0, 0.0};
  const city::wire::Vec3d e{13.0, -10.0, 0.0};

  const auto configure = [](city::wire::CoreState* state) {
    if (state == nullptr) return false;
    city::wire::ModelAssemblyTemplate row_assembly{};
    row_assembly.id = kRowAssembly;
    city::wire::ModelAssemblyPart row_part{};
    row_part.part_id = 1;
    row_part.model_key = "c817_crossarm";
    row_part.descriptor_version = 1;
    row_part.fit_mode = city::wire::ModelFitMode::kRigid;
    row_part.sockets.push_back({"endpoint_mount", {0.0, 0.0, kRowMountZ}, {0.0, 0.0, 1.0}});
    row_assembly.parts.push_back(row_part);
    row_assembly.endpoint_mount_socket = city::wire::AssemblySocketRef{1, "endpoint_mount"};

    city::wire::ModelAssemblyTemplate endpoint_assembly{};
    endpoint_assembly.id = kEndpointAssembly;
    city::wire::ModelAssemblyPart endpoint_part{};
    endpoint_part.part_id = 1;
    endpoint_part.model_key = "c817_insulator";
    endpoint_part.descriptor_version = 1;
    endpoint_part.sockets.push_back({"wire", {0.0, 0.0, kWireSocketZ}, {1.0, 0.0, 0.0}});
    endpoint_assembly.parts.push_back(endpoint_part);
    endpoint_assembly.wire_socket = city::wire::AssemblySocketRef{1, "wire"};
    WIRE_TEST_EXPECT(state->RegisterModelAssemblyTemplate(row_assembly).ok,
                     "failed to register sharp row assembly");
    WIRE_TEST_EXPECT(state->RegisterModelAssemblyTemplate(endpoint_assembly).ok,
                     "failed to register sharp endpoint assembly");

    const city::wire::BundleTemplateId hv_template_id =
        city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage);
    city::wire::BundleTemplate hv = state->view().bundle_templates().at(hv_template_id);
    hv.row_fixture_assembly_id = kRowAssembly;
    hv.endpoint_fixture_assembly_id = kEndpointAssembly;
    WIRE_TEST_EXPECT(hv.enable_branch_down_offset,
                     "HV branch-down flag is disabled");
    WIRE_TEST_EXPECT(state->UpdateBundleTemplate(hv).ok,
                     "failed to attach sharp HV assemblies");
    return true;
  };

  const auto use_explicit_hv_placement = [](city::wire::BackboneSpec* request) {
    if (request == nullptr || request->bundles.size() != 1) return false;
    city::wire::BackboneBundleSpec& bundle = request->bundles.front();
    bundle.placement_key = 817;
    bundle.placement_explicit = true;
    bundle.count = 3;
    bundle.height_m = 9.2;
    bundle.lateral_m = -0.2;
    bundle.spacing_m = 0.45;
    return true;
  };

  const auto hv_request = [&](city::wire::CoreState& state) {
    city::wire::BackboneSpec request = line_req(state);
    request.bundles.clear();
    add_backbone_bundle(request, city::wire::BundleKind::kHighVoltage);
    static_cast<void>(use_explicit_hv_placement(&request));
    return request;
  };

  const auto visible_socket_for_port =
      [](const city::wire::CoreState& state, city::wire::ObjectId port_id,
         city::wire::Vec3d* out) {
    if (out == nullptr) return false;
    const std::string fixture_prefix = "port:" + std::to_string(port_id) + ":";
    const auto fixture = std::find_if(
        state.view().visual_model_instances().instances.begin(),
        state.view().visual_model_instances().instances.end(),
        [&](const city::wire::VisualModelInstance& instance) {
          return instance.model_key == "c817_insulator" &&
                 instance.stable_key.rfind(fixture_prefix, 0) == 0;
        });
    if (fixture == state.view().visual_model_instances().instances.end()) {
      return false;
    }
    const city::wire::Vec3d local_socket{0.0, 0.0, kWireSocketZ};
    *out = fixture->world_transform.position +
           city::wire::RotateEulerXYZDeg(
               {local_socket.x * fixture->world_transform.scale.x,
                local_socket.y * fixture->world_transform.scale.y,
                local_socket.z * fixture->world_transform.scale.z},
               fixture->world_transform.rotation_euler_deg);
    return true;
  };

  const auto check_sharp_lowering =
      [&](const city::wire::CoreState& state, city::wire::ObjectId junction,
          const char* label) {
    const city::wire::BundleTemplateId hv_template_id =
        city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage);
    const city::wire::SavedBackboneNode* node =
        state.view().backbone_node_for_pole(junction);
    WIRE_TEST_EXPECT(node != nullptr,
                     std::string(label) + ": junction backbone node is missing");
    const double step = std::max(
        0.0, -state.view().bundle_templates().at(hv_template_id).branch_endpoint_offset_m);
    WIRE_TEST_EXPECT(step > 0.0,
                     std::string(label) + ": branch-down step is not positive");

    std::size_t lowered_endpoint_count = 0;
    std::size_t baseline_endpoint_count = 0;
    for (const city::wire::Span& span : state.view().spans().items()) {
      const city::wire::SpanLayoutEntry* layout = state.span_layout(span.id).entry;
      const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span.id);
      WIRE_TEST_EXPECT(layout != nullptr && curve != nullptr &&
                           curve->detail.sample_points.size() >= 2,
                       std::string(label) + ": layout or curve is missing");
      for (const auto& endpoint_pair : {
               std::pair<const city::wire::LayoutEndpoint*, const city::wire::Vec3d*>{
                   &layout->start, &curve->detail.sample_points.front()},
               std::pair<const city::wire::LayoutEndpoint*, const city::wire::Vec3d*>{
                   &layout->end, &curve->detail.sample_points.back()}}) {
        const city::wire::LayoutEndpoint& endpoint = *endpoint_pair.first;
        const city::wire::Port* port = state.view().ports().find(endpoint.port_id);
        if (port == nullptr || port->owner_pole_id != junction) continue;
        const city::wire::SavedBackbonePortBinding* binding =
            state.view().backbone_port_binding_for_port(port->id);
        if (binding == nullptr || binding->row_key.node_id != node->node_id ||
            binding->bundle_template_id != hv_template_id) {
          continue;
        }
        const city::wire::Pole* pole =
            state.view().poles().find(port->owner_pole_id);
        WIRE_TEST_EXPECT(pole != nullptr,
                         std::string(label) + ": endpoint pole is missing");
        const city::wire::PoleFrame frame =
            city::wire::BuildPoleFrame(pole->world_transform, binding->layout_yaw_deg);
        const bool lowered = endpoint.default_lower_required ||
                             endpoint.lower_required ||
                             endpoint.branch_down_offset_m > 0.0;
        if (!lowered) {
          ++baseline_endpoint_count;
          continue;
        }
        ++lowered_endpoint_count;
        const double expected_down =
            step * static_cast<double>(binding->support_level);
        WIRE_TEST_EXPECT(binding->support_level > 0,
                         std::string(label) + ": lowered endpoint has no positive support level");
        city::wire::Vec3d visible_socket{};
        WIRE_TEST_EXPECT(visible_socket_for_port(state, port->id, &visible_socket),
                         std::string(label) + ": lowered endpoint fixture is missing");
        const city::wire::Vec3d expected_fixture_root =
            port->world_position - city::wire::ScaleVec(frame.up, expected_down) +
            city::wire::ScaleVec(frame.up, kRowMountZ);
        const city::wire::Vec3d expected_wire_socket =
            expected_fixture_root + city::wire::ScaleVec(frame.up, kWireSocketZ);
        const std::string fixture_prefix = "port:" + std::to_string(port->id) + ":";
        const auto fixture = std::find_if(
            state.view().visual_model_instances().instances.begin(),
            state.view().visual_model_instances().instances.end(),
            [&](const city::wire::VisualModelInstance& instance) {
              return instance.model_key == "c817_insulator" &&
                     instance.stable_key.rfind(fixture_prefix, 0) == 0;
            });
        WIRE_TEST_EXPECT(fixture != state.view().visual_model_instances().instances.end(),
                         std::string(label) + ": lowered fixture instance is missing");
        WIRE_TEST_EXPECT(almost_equal(endpoint.branch_down_offset_m, expected_down, 1e-9),
                         std::string(label) + ": lowered endpoint offset does not match its support level");
        WIRE_TEST_EXPECT(almost_equal(fixture->world_transform.position, expected_fixture_root, 1e-9),
                         std::string(label) + ": lowered fixture root does not use its support level");
        WIRE_TEST_EXPECT(almost_equal(visible_socket, expected_wire_socket, 1e-9),
                         std::string(label) + ": visible socket does not use its support level");
        WIRE_TEST_EXPECT(almost_equal(endpoint.endpoint_world, visible_socket, 1e-9),
                         std::string(label) + ": layout endpoint is not the visible socket");
        WIRE_TEST_EXPECT(almost_equal(*endpoint_pair.second, visible_socket, 1e-9),
                         std::string(label) + ": curve endpoint is not the visible socket");
      }
    }
    WIRE_TEST_EXPECT(baseline_endpoint_count == 6,
                     std::string(label) + ": base through endpoints changed height");
    WIRE_TEST_EXPECT(lowered_endpoint_count == 6,
                     std::string(label) + ": sharp pair did not lower all three phases on both edges");
    return true;
  };

  city::wire::CoreState one_shot;
  WIRE_TEST_EXPECT(configure(&one_shot), "failed to configure one-shot state");
  city::wire::BackboneSpec one_base = hv_poly3_req(one_shot);
  WIRE_TEST_EXPECT(use_explicit_hv_placement(&one_base),
                   "failed to configure one-shot base HV placement");
  const auto one_base_result = one_shot.GenerateFromBackboneSpec(one_base);
  WIRE_TEST_EXPECT(one_base_result.ok, one_base_result.error);
  WIRE_TEST_EXPECT(one_base_result.value.generated_pole_ids.size() == 3,
                   "one-shot base did not generate 3 poles");
  const city::wire::ObjectId one_b = one_base_result.value.generated_pole_ids[1];
  const city::wire::Pole* one_pole = one_shot.view().poles().find(one_b);
  WIRE_TEST_EXPECT(one_pole != nullptr, "one-shot junction pole is missing");
  city::wire::BackboneSpec one_pair = hv_request(one_shot);
  one_pair.path.polyline = {d, one_pole->world_transform.position, e};
  one_pair.path.node_specs = {pole_spec(1, one_b)};
  const auto one_pair_result = one_shot.GenerateFromBackboneSpec(one_pair);
  WIRE_TEST_EXPECT(one_pair_result.ok, one_pair_result.error);
  WIRE_TEST_EXPECT(check_sharp_lowering(one_shot, one_b, "one-shot"),
                   "one-shot sharp lowering check failed");

  city::wire::CoreState incremental;
  WIRE_TEST_EXPECT(configure(&incremental), "failed to configure incremental state");
  city::wire::BackboneSpec incremental_base = hv_poly3_req(incremental);
  WIRE_TEST_EXPECT(use_explicit_hv_placement(&incremental_base),
                   "failed to configure incremental base HV placement");
  const auto incremental_base_result =
      incremental.GenerateFromBackboneSpec(incremental_base);
  WIRE_TEST_EXPECT(incremental_base_result.ok, incremental_base_result.error);
  WIRE_TEST_EXPECT(incremental_base_result.value.generated_pole_ids.size() == 3,
                   "incremental base did not generate 3 poles");
  const city::wire::ObjectId incremental_b =
      incremental_base_result.value.generated_pole_ids[1];
  const city::wire::Pole* incremental_pole =
      incremental.view().poles().find(incremental_b);
  WIRE_TEST_EXPECT(incremental_pole != nullptr,
                   "incremental junction pole is missing");
  city::wire::BackboneSpec first = hv_request(incremental);
  first.path.polyline = {incremental_pole->world_transform.position, d};
  first.path.node_specs = {pole_spec(0, incremental_b)};
  const auto first_result = incremental.GenerateFromBackboneSpec(first);
  WIRE_TEST_EXPECT(first_result.ok, first_result.error);
  city::wire::BackboneSpec second = hv_request(incremental);
  second.path.polyline = {e, incremental_pole->world_transform.position};
  second.path.node_specs = {pole_spec(1, incremental_b)};
  const auto second_result = incremental.GenerateFromBackboneSpec(second);
  WIRE_TEST_EXPECT(second_result.ok, second_result.error);
  WIRE_TEST_EXPECT(check_sharp_lowering(incremental, incremental_b, "incremental"),
                   "incremental sharp lowering check failed");
  return true;
}

bool C766_row_fixture_and_wire_follow_port_band_lateral_change() {
  constexpr city::wire::ModelAssemblyTemplateId kRowAssembly = 9120;
  constexpr city::wire::ModelAssemblyTemplateId kEndpointAssembly = 9121;
  constexpr double kDelta = 0.3;
  city::wire::CoreState state;

  city::wire::ModelAssemblyTemplate row_assembly{};
  row_assembly.id = kRowAssembly;
  city::wire::ModelAssemblyPart row_part{};
  row_part.part_id = 1;
  row_part.model_key = "hv_crossarm";
  row_part.local_transform.position = {0.17, 0.0, 0.0};
  row_part.sockets.push_back({"endpoint_mount", {0.0, 0.0, 0.04}, {0.0, 0.0, 1.0}});
  row_assembly.parts.push_back(row_part);
  row_assembly.parts.push_back(
      {2, "pole_belt", 1, {}, city::wire::ModelFitMode::kPoleRadial, {}});
  row_assembly.endpoint_mount_socket =
      city::wire::AssemblySocketRef{1, "endpoint_mount"};
  city::wire::ModelAssemblyTemplate endpoint_assembly{};
  endpoint_assembly.id = kEndpointAssembly;
  city::wire::ModelAssemblyPart endpoint_part{};
  endpoint_part.part_id = 1;
  endpoint_part.model_key = "hv_insulator";
  endpoint_part.sockets.push_back({"wire", {0.0, 0.0, 0.25}, {1.0, 0.0, 0.0}});
  endpoint_assembly.parts.push_back(endpoint_part);
  endpoint_assembly.parts.push_back(
      {2, "pole_belt", 1, {}, city::wire::ModelFitMode::kPoleRadial, {}});
  endpoint_assembly.wire_socket = city::wire::AssemblySocketRef{1, "wire"};
  if (!state.RegisterModelAssemblyTemplate(row_assembly).ok ||
      !state.RegisterModelAssemblyTemplate(endpoint_assembly).ok) {
    return false;
  }
  const city::wire::BundleTemplateId hv_template_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage);
  city::wire::BundleTemplate hv = state.view().bundle_templates().at(hv_template_id);
  hv.row_fixture_assembly_id = kRowAssembly;
  hv.endpoint_fixture_assembly_id = kEndpointAssembly;
  if (!state.UpdateBundleTemplate(hv).ok) return false;

  city::wire::BackboneSpec request = line_req(state);
  request.bundles.clear();
  add_backbone_bundle(request, city::wire::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(request);
  if (!generated.ok) return false;

  std::unordered_map<std::string, std::uint64_t> model_versions{};
  for (const city::wire::VisualModelInstance& instance :
       state.view().visual_model_instances().instances) {
    model_versions.emplace(instance.stable_key, instance.content_version);
  }
  city::wire::ModelAssemblyTemplate moved_row = row_assembly;
  moved_row.version = 2;
  moved_row.parts.front().local_transform.position.x += 0.05;
  const auto moved = state.UpdateModelAssemblyTemplate(moved_row);
  if (!moved.ok || moved.value == false ||
      moved.change_set.updated_ids.size() != generated.value.generated_span_ids.size()) return false;
  for (const city::wire::VisualModelInstance& instance :
       state.view().visual_model_instances().instances) {
    const auto before_version = model_versions.find(instance.stable_key);
    if (before_version == model_versions.end()) return false;
    if ((instance.model_key == "hv_crossarm" || instance.model_key == "hv_insulator") &&
        before_version->second == instance.content_version) return false;
  }

  std::unordered_map<std::string, city::wire::Vec3d> fixture_positions{};
  for (const city::wire::VisualModelInstance& instance :
       state.view().visual_model_instances().instances) {
    if (instance.model_key == "hv_crossarm" || instance.model_key == "pole_belt" ||
        instance.model_key == "hv_insulator") {
      fixture_positions.emplace(instance.stable_key, instance.world_transform.position);
    }
  }
  std::unordered_map<city::wire::ObjectId, city::wire::Vec3d> port_positions{};
  for (city::wire::ObjectId span_id : generated.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) return false;
    for (city::wire::ObjectId port_id : {span->port_a_id, span->port_b_id}) {
      const city::wire::Port* port = state.view().ports().find(port_id);
      if (port == nullptr) return false;
      port_positions.emplace(port_id, port->world_position);
      const auto placement = city::wire::generation::backbone::resolve_endpoint_placement(
          state, *port);
      if (!placement.ok) return false;
      const std::string prefix = "port:" + std::to_string(port_id) + ":";
      const auto fixture = std::find_if(
          state.view().visual_model_instances().instances.begin(),
          state.view().visual_model_instances().instances.end(),
          [&](const city::wire::VisualModelInstance& instance) {
            return instance.stable_key.rfind(prefix, 0) == 0 &&
                   instance.model_key == "hv_insulator";
          });
      if (fixture == state.view().visual_model_instances().instances.end() ||
          !almost_equal(fixture->world_transform.position,
                        placement.value.fixture_root.position, 1e-9)) return false;
    }
  }

  city::wire::PoleTypeDefinition pole_type =
      state.view().pole_types().at(request.pole_type_id);
  for (city::wire::PortPlacementBand& band : pole_type.port_bands) {
    if (band.category != city::wire::ConnectionCategory::kHighVoltage) continue;
    band.lateral_center_m += kDelta;
    band.lateral_min_m += kDelta;
    band.lateral_max_m += kDelta;
  }
  if (!state.UpdatePoleTypeDefinition(pole_type).ok) return false;

  for (const auto& [port_id, before_position] : port_positions) {
    const city::wire::Port* port = state.view().ports().find(port_id);
    const city::wire::SavedBackbonePortBinding* binding =
        state.view().backbone_port_binding_for_port(port_id);
    const city::wire::Pole* pole =
        port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
    if (port == nullptr || binding == nullptr || pole == nullptr) return false;
    const city::wire::PoleFrame frame =
        city::wire::BuildPoleFrame(pole->world_transform, binding->layout_yaw_deg);
    if (!almost_equal(port->world_position - before_position,
                      city::wire::ScaleVec(frame.lateral, kDelta), 1e-9)) {
      return false;
    }
  }

  for (const city::wire::VisualModelInstance& instance :
       state.view().visual_model_instances().instances) {
    const auto before = fixture_positions.find(instance.stable_key);
    if (before == fixture_positions.end()) continue;
    if (instance.model_key == "pole_belt") {
      if (!almost_equal(instance.world_transform.position, before->second, 1e-9)) return false;
      continue;
    }
    const std::size_t pole_id_begin = instance.stable_key.find(':') + 1;
    const std::size_t pole_id_end = instance.stable_key.find(':', pole_id_begin);
    city::wire::ObjectId pole_id = city::wire::kInvalidObjectId;
    if (instance.stable_key.rfind("row:", 0) == 0) {
      pole_id = static_cast<city::wire::ObjectId>(
          std::stoull(instance.stable_key.substr(pole_id_begin, pole_id_end - pole_id_begin)));
    } else if (instance.stable_key.rfind("port:", 0) == 0) {
      const city::wire::ObjectId port_id = static_cast<city::wire::ObjectId>(
          std::stoull(instance.stable_key.substr(pole_id_begin, pole_id_end - pole_id_begin)));
      const city::wire::Port* port = state.view().ports().find(port_id);
      pole_id = port == nullptr ? city::wire::kInvalidObjectId : port->owner_pole_id;
    }
    const city::wire::Pole* pole = state.view().poles().find(pole_id);
    if (pole == nullptr) return false;
    const city::wire::SavedBackbonePortBinding* pole_binding = nullptr;
    for (const city::wire::SavedBackbonePortBinding& binding :
         state.view().backbone().port_bindings) {
      const city::wire::Port* port = state.view().ports().find(binding.port_id);
      if (port != nullptr && port->owner_pole_id == pole_id &&
          binding.bundle_template_id == hv_template_id) {
        pole_binding = &binding;
        break;
      }
    }
    if (pole_binding == nullptr) return false;
    const city::wire::PoleFrame frame =
        city::wire::BuildPoleFrame(pole->world_transform, pole_binding->layout_yaw_deg);
    const city::wire::Vec3d displacement = instance.world_transform.position - before->second;
    if (!almost_equal(displacement, city::wire::ScaleVec(frame.lateral, kDelta), 1e-9)) {
      return false;
    }
  }

  for (city::wire::ObjectId span_id : generated.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    const city::wire::SpanLayoutEntry* layout = state.span_layout(span_id).entry;
    const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span_id);
    const city::wire::Port* port_a = span == nullptr ? nullptr : state.view().ports().find(span->port_a_id);
    const city::wire::Port* port_b = span == nullptr ? nullptr : state.view().ports().find(span->port_b_id);
    if (port_a == nullptr || port_b == nullptr) return false;
    const auto placement_a =
        city::wire::generation::backbone::resolve_endpoint_placement(state, *port_a);
    const auto placement_b =
        city::wire::generation::backbone::resolve_endpoint_placement(state, *port_b);
    if (layout == nullptr || curve == nullptr || !placement_a.ok || !placement_b.ok ||
        curve->detail.sample_points.size() < 2 ||
        !almost_equal(curve->detail.sample_points.front(), layout->start.endpoint_world, 1e-9) ||
        !almost_equal(curve->detail.sample_points.back(), layout->end.endpoint_world, 1e-9) ||
        !almost_equal(layout->start.support_world, placement_a.value.wire_endpoint, 1e-9) ||
        !almost_equal(layout->end.support_world, placement_b.value.wire_endpoint, 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C769_bundle_placements_duplicate_template_as_independent_bundles() {
  city::wire::CoreState state;
  const city::wire::BundleTemplateId template_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kOptical);
  const auto template_it = state.view().bundle_templates().find(template_id);
  if (template_it == state.view().bundle_templates().end()) return false;

  city::wire::BackboneSpec request{};
  request.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {12.0, 8.0, 0.0}};
  request.pole_type_id = template_it->second.related_pole_type_id;
  city::wire::BackboneBundleSpec first{};
  first.bundle_template_id = template_id;
  first.layer = template_it->second.default_layer;
  first.spacing_m = 0.31;
  city::wire::BackboneBundleSpec second = first;
  first.placement_explicit = true;
  first.height_m = 7.4;
  first.lateral_m = 0.0;
  second.placement_explicit = true;
  second.height_m = 8.4;
  second.lateral_m = 0.4;
  second.spacing_m = 0.47;
  request.bundles = {first, second};

  const auto generated = state.GenerateFromBackboneSpec(request);
  if (!generated.ok || generated.value.generated_span_ids.size() != 4) return false;

  std::vector<const city::wire::Span*> spans{};
  for (city::wire::ObjectId span_id : generated.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) return false;
    spans.push_back(span);
  }
  std::vector<city::wire::ObjectId> distinct_bundle_ids{};
  for (const city::wire::Span* span : spans) {
    if (std::find(distinct_bundle_ids.begin(), distinct_bundle_ids.end(), span->bundle_id) ==
        distinct_bundle_ids.end()) {
      distinct_bundle_ids.push_back(span->bundle_id);
    }
  }
  if (distinct_bundle_ids.size() != 2) return false;
  const city::wire::Bundle* bundle_a = state.view().bundles().find(distinct_bundle_ids[0]);
  const city::wire::Bundle* bundle_b = state.view().bundles().find(distinct_bundle_ids[1]);
  if (bundle_a == nullptr || bundle_b == nullptr || bundle_a->id == bundle_b->id ||
      bundle_a->bundle_template_id != template_id || bundle_b->bundle_template_id != template_id ||
      std::abs(bundle_a->phase_spacing_m - 0.31) > 1e-12 ||
      std::abs(bundle_b->phase_spacing_m - 0.47) > 1e-12 ||
      !bundle_a->placement_explicit || !bundle_b->placement_explicit ||
      std::abs(bundle_a->height_m - 7.4) > 1e-12 ||
      std::abs(bundle_b->height_m - 8.4) > 1e-12 ||
      std::abs(bundle_a->lateral_m) > 1e-12 ||
      std::abs(bundle_b->lateral_m - 0.4) > 1e-12) {
    return false;
  }
  const city::wire::ObjectId bundle_a_id = bundle_a->id;
  const city::wire::ObjectId bundle_b_id = bundle_b->id;

  auto first_edge_span = [&](city::wire::ObjectId bundle_id) -> const city::wire::Span* {
    const city::wire::Span* best = nullptr;
    double best_mid_x = std::numeric_limits<double>::infinity();
    for (const city::wire::Span* span : spans) {
      if (span->bundle_id != bundle_id) continue;
      const city::wire::Port* a = state.view().ports().find(span->port_a_id);
      const city::wire::Port* b = state.view().ports().find(span->port_b_id);
      if (a == nullptr || b == nullptr) continue;
      const double mid_x = (a->world_position.x + b->world_position.x) * 0.5;
      if (mid_x < best_mid_x) {
        best_mid_x = mid_x;
        best = span;
      }
    }
    return best;
  };
  const city::wire::Span* bundle_a_first_span = first_edge_span(bundle_a_id);
  const city::wire::Span* bundle_b_first_span = first_edge_span(bundle_b_id);
  if (bundle_a_first_span == nullptr || bundle_b_first_span == nullptr) return false;

  auto local_port = [&](const city::wire::Span& span, bool start) -> std::optional<city::wire::Vec3d> {
    const city::wire::ObjectId port_id = start ? span.port_a_id : span.port_b_id;
    const city::wire::Port* port = state.view().ports().find(port_id);
    const city::wire::SavedBackbonePortBinding* binding =
        state.view().backbone_port_binding_for_port(port_id);
    const city::wire::Pole* pole =
        port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
    if (port == nullptr || binding == nullptr || pole == nullptr) return std::nullopt;
    return city::wire::WorldPointToLocal(
        city::wire::BuildPoleFrame(pole->world_transform, binding->layout_yaw_deg),
        port->world_position);
  };
  for (bool start : {true, false}) {
    const auto a = local_port(*bundle_a_first_span, start);
    const auto b = local_port(*bundle_b_first_span, start);
    if (!a.has_value() || !b.has_value() ||
        std::abs((b->y - a->y) - 0.4) > 1e-9 ||
        std::abs((b->z - a->z) - 1.0) > 1e-9) {
      return false;
    }
  }

  for (const city::wire::Bundle* bundle : {bundle_a, bundle_b}) {
    std::size_t support_count = 0;
    std::size_t helix_count = 0;
    std::size_t patch_count = 0;
    for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
      if (part.source_bundle_id != bundle->id) continue;
      support_count += part.supplemental_kind == city::wire::VisualSupplementalKind::kSupportPath ? 1U : 0U;
      helix_count += part.supplemental_kind == city::wire::VisualSupplementalKind::kHelix ? 1U : 0U;
      patch_count += part.kind == city::wire::VisualCurvePartKind::kNodePatch ? 1U : 0U;
    }
    if (support_count != 2 || helix_count != 2 || patch_count != 1) return false;
  }

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_id = bundle_a_first_span->id;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  city::wire::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {template_id};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == city::wire::kInvalidObjectId) return false;

  city::wire::BackboneSpec branch{};
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  branch.pole_type_id = request.pole_type_id;
  city::wire::BackboneInputSpec::NodeSpec source_node{};
  source_node.point_index = 0;
  source_node.support_kind = resolved.value.support_kind;
  source_node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {source_node};
  first.source_bundle_id = bundle_a_id;
  second.source_bundle_id = bundle_b_id;
  branch.bundles = {first, second};
  const auto branch_out = state.GenerateFromBackboneSpec(branch);
  if (!branch_out.ok || branch_out.value.bundle_ids.size() != 2 ||
      branch_out.value.generated_span_ids.size() != 2) {
    return false;
  }
  const std::array<city::wire::ObjectId, 2> expected_source_bundles{bundle_a_id, bundle_b_id};
  for (std::size_t index = 0; index < branch_out.value.bundle_ids.size(); ++index) {
    const auto span_it = std::find_if(branch_out.value.generated_span_ids.begin(),
                                      branch_out.value.generated_span_ids.end(),
                                      [&](city::wire::ObjectId span_id) {
      const city::wire::Span* span = state.view().spans().find(span_id);
      return span != nullptr && span->bundle_id == branch_out.value.bundle_ids[index];
    });
    if (span_it == branch_out.value.generated_span_ids.end()) return false;
    const city::wire::SpanLayoutView layout = state.span_layout(*span_it);
    if (!layout.has_layout()) return false;
    const city::wire::LayoutEndpoint& endpoint =
        layout.entry->start.source_projection.valid() ? layout.entry->start : layout.entry->end;
    const city::wire::SavedBackboneEdgeBundle* source_edge_bundle =
        state.view().backbone_edge_bundle(endpoint.source_projection.source_edge_bundle_id);
    if (!endpoint.source_projection.valid() || source_edge_bundle == nullptr ||
        source_edge_bundle->bundle_id != expected_source_bundles[index]) {
      return false;
    }
  }
  return true;
}

bool C770_backbone_bundle_placement_update_preserves_cross_row_height() {
  constexpr city::wire::ModelAssemblyTemplateId kRowAssembly = 9130;
  constexpr city::wire::ModelAssemblyTemplateId kEndpointAssembly = 9131;
  constexpr double kHeightDelta = 0.6;
  city::wire::CoreState state;

  city::wire::ModelAssemblyTemplate row_assembly{};
  row_assembly.id = kRowAssembly;
  city::wire::ModelAssemblyPart row_part{};
  row_part.part_id = 1;
  row_part.model_key = "hv_crossarm";
  row_part.sockets.push_back({"endpoint_mount", {0.0, 0.0, 0.04}, {0.0, 0.0, 1.0}});
  row_assembly.parts.push_back(row_part);
  row_assembly.endpoint_mount_socket = city::wire::AssemblySocketRef{1, "endpoint_mount"};

  city::wire::ModelAssemblyTemplate endpoint_assembly{};
  endpoint_assembly.id = kEndpointAssembly;
  city::wire::ModelAssemblyPart endpoint_part{};
  endpoint_part.part_id = 1;
  endpoint_part.model_key = "hv_insulator";
  endpoint_part.sockets.push_back({"wire", {0.0, 0.0, 0.20}, {1.0, 0.0, 0.0}});
  endpoint_assembly.parts.push_back(endpoint_part);
  endpoint_assembly.wire_socket = city::wire::AssemblySocketRef{1, "wire"};

  if (!state.RegisterModelAssemblyTemplate(row_assembly).ok ||
      !state.RegisterModelAssemblyTemplate(endpoint_assembly).ok) {
    return false;
  }
  const city::wire::BundleTemplateId hv_template_id =
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage);
  city::wire::BundleTemplate hv = state.view().bundle_templates().at(hv_template_id);
  hv.row_fixture_assembly_id = kRowAssembly;
  hv.endpoint_fixture_assembly_id = kEndpointAssembly;
  if (!state.UpdateBundleTemplate(hv).ok) return false;

  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() < 2) return false;
  const city::wire::ObjectId pole_id = first.value.generated_pole_ids[1];
  const city::wire::Pole* pole = state.view().poles().find(pole_id);
  if (pole == nullptr) return false;

  city::wire::BackboneSpec cross = line_req(state);
  cross.bundles.clear();
  add_backbone_bundle(cross, city::wire::BundleKind::kHighVoltage);
  cross.path.polyline = {{12.0, -8.0, 0.0}, pole->world_transform.position, {20.0, 0.0, 0.0}};
  cross.path.node_specs = {pole_spec(1, pole_id)};
  const auto second = state.GenerateFromBackboneSpec(cross);
  if (!second.ok || second.value.bundle_ids.empty()) return false;
  const city::wire::ObjectId bundle_id = second.value.bundle_ids.front();
  const city::wire::Bundle* bundle = state.view().bundles().find(bundle_id);
  if (bundle == nullptr) return false;

  const auto incident_layouts_match_fixtures = [&]() {
    for (const city::wire::Span& span : state.view().spans().items()) {
      const city::wire::Port* port_a = state.view().ports().find(span.port_a_id);
      const city::wire::Port* port_b = state.view().ports().find(span.port_b_id);
      const city::wire::SpanLayoutEntry* layout = state.span_layout(span.id).entry;
      const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span.id);
      if (port_a == nullptr || port_b == nullptr || layout == nullptr || curve == nullptr ||
          curve->detail.sample_points.size() < 2 ||
          !almost_equal(curve->detail.sample_points.front(), layout->start.endpoint_world, 1e-9) ||
          !almost_equal(curve->detail.sample_points.back(), layout->end.endpoint_world, 1e-9)) {
        return false;
      }
      const auto matches = [&](const city::wire::Port& port,
                               const city::wire::LayoutEndpoint& endpoint) {
        if (port.owner_pole_id != pole_id) return true;
        const auto fixture = std::find_if(
            state.view().visual_model_instances().instances.begin(),
            state.view().visual_model_instances().instances.end(),
            [&](const city::wire::VisualModelInstance& instance) {
              if (instance.model_key != "hv_insulator") return false;
              const city::wire::Vec3d local_socket =
                  endpoint_part.sockets.front().local_position;
              const city::wire::Vec3d socket =
                  instance.world_transform.position +
                  city::wire::RotateEulerXYZDeg(
                      {local_socket.x * instance.world_transform.scale.x,
                       local_socket.y * instance.world_transform.scale.y,
                       local_socket.z * instance.world_transform.scale.z},
                      instance.world_transform.rotation_euler_deg);
              return almost_equal(socket, endpoint.endpoint_world, 1e-9);
            });
        if (fixture == state.view().visual_model_instances().instances.end()) return false;
        const city::wire::Vec3d local_socket = endpoint_part.sockets.front().local_position;
        const city::wire::Vec3d visible_socket =
            fixture->world_transform.position +
            city::wire::RotateEulerXYZDeg(
                {local_socket.x * fixture->world_transform.scale.x,
                 local_socket.y * fixture->world_transform.scale.y,
                 local_socket.z * fixture->world_transform.scale.z},
                fixture->world_transform.rotation_euler_deg);
        const bool ok = almost_equal(endpoint.support_world, endpoint.endpoint_world, 1e-9) &&
                        almost_equal(endpoint.endpoint_world, visible_socket, 1e-9);
        return ok;
      };
      if (!matches(*port_a, layout->start) || !matches(*port_b, layout->end)) {
        return false;
      }
    }
    return true;
  };
  if (!incident_layouts_match_fixtures()) return false;

  auto average_port_height = [&]() -> std::optional<double> {
    double sum = 0.0;
    std::size_t count = 0;
    for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
      const city::wire::SavedBackboneEdgeBundle* edge_bundle =
          state.view().backbone_edge_bundle(binding.edge_bundle_id);
      const city::wire::Port* port = state.view().ports().find(binding.port_id);
      const city::wire::Pole* owner =
          port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
      if (edge_bundle == nullptr || edge_bundle->bundle_id != bundle_id ||
          port == nullptr || owner == nullptr || owner->id != pole_id) {
        continue;
      }
      const city::wire::PoleFrame frame =
          city::wire::BuildPoleFrame(owner->world_transform, binding.layout_yaw_deg);
      sum += city::wire::WorldPointToLocal(frame, port->world_position).z;
      ++count;
    }
    if (count == 0) return std::nullopt;
    return sum / static_cast<double>(count);
  };
  auto average_base_height = [&]() -> std::optional<double> {
    double sum = 0.0;
    std::size_t count = 0;
    for (const city::wire::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
      const city::wire::SavedBackboneEdgeBundle* edge_bundle =
          state.view().backbone_edge_bundle(binding.edge_bundle_id);
      const city::wire::Port* port = state.view().ports().find(binding.port_id);
      const city::wire::Pole* owner =
          port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
      const auto pole_type_it =
          owner == nullptr ? state.view().pole_types().end() : state.view().pole_types().find(owner->pole_type_id);
      const city::wire::PoleTypeDefinition* pole_type =
          pole_type_it == state.view().pole_types().end() ? nullptr : &pole_type_it->second;
      const city::wire::PortPlacementBand* band = nullptr;
      if (pole_type != nullptr) {
        for (const city::wire::PortPlacementBand& candidate : pole_type->port_bands) {
          if (candidate.band_id == binding.placement_band_id) {
            band = &candidate;
            break;
          }
        }
      }
      if (edge_bundle == nullptr || edge_bundle->bundle_id != bundle_id ||
          port == nullptr || owner == nullptr || owner->id != pole_id || band == nullptr) {
        continue;
      }
      sum += bundle->placement_explicit ? bundle->height_m : band->height_center_m;
      ++count;
    }
    if (count == 0) return std::nullopt;
    return sum / static_cast<double>(count);
  };
  auto row_model_height = [&]() -> std::optional<double> {
    double sum = 0.0;
    std::size_t count = 0;
    const std::string prefix = "row:" + std::to_string(pole_id) + ":";
    const std::string bundle_token = ":" + std::to_string(bundle_id) + ":";
    for (const city::wire::VisualModelInstance& instance : state.view().visual_model_instances().instances) {
      if (instance.model_key != "hv_crossarm" ||
          instance.stable_key.rfind(prefix, 0) != 0 ||
          instance.stable_key.find(bundle_token, prefix.size()) == std::string::npos) {
        continue;
      }
      const city::wire::Pole* current_pole = state.view().poles().find(pole_id);
      if (current_pole == nullptr) return std::nullopt;
      const city::wire::PoleFrame frame =
          city::wire::BuildPoleFrame(current_pole->world_transform, 0.0);
      sum += city::wire::WorldPointToLocal(frame, instance.world_transform.position).z;
      ++count;
    }
    if (count == 0) return std::nullopt;
    return sum / static_cast<double>(count);
  };

  const std::optional<double> port_height_before = average_port_height();
  const std::optional<double> row_height_before = row_model_height();
  const std::optional<double> base_height_before = average_base_height();
  if (!port_height_before.has_value() || !row_height_before.has_value() || !base_height_before.has_value()) {
    return false;
  }
  const double preserved_cross_offset = *port_height_before - *base_height_before;
  if (std::abs(preserved_cross_offset) < 0.1) return false;

  const double next_height = *base_height_before + kHeightDelta;
  const auto updated = state.UpdateBackboneBundlePlacement(
      bundle_id, true, next_height, bundle->lateral_m, bundle->phase_spacing_m);
  if (!updated.ok || !updated.value) return false;

  const city::wire::Bundle* after_bundle = state.view().bundles().find(bundle_id);
  const std::optional<double> port_height_after = average_port_height();
  const std::optional<double> row_height_after = row_model_height();
  if (after_bundle == nullptr || !port_height_after.has_value() || !row_height_after.has_value()) return false;
  const city::wire::EditResult<city::wire::VisualCurvePartCache> full_visual =
      city::wire::generation::backbone::make_visual_curve_parts(state, {});
  WIRE_TEST_EXPECT(full_visual.ok, full_visual.error);
  WIRE_TEST_EXPECT(
      visual_part_snapshot(state.view().visual_curve_parts()) ==
          visual_part_snapshot(full_visual.value),
      "bundle placement update left stale visual curve parts");

  return almost_equal((*port_height_after - after_bundle->height_m), preserved_cross_offset, 1e-9) &&
         almost_equal(*port_height_after - *port_height_before, kHeightDelta, 1e-9) &&
         almost_equal(*row_height_after - *row_height_before, kHeightDelta, 1e-9) &&
         incident_layouts_match_fixtures();
}

namespace {

struct D1CornerFixture {
  city::wire::GenerateBundleFromPathResult generated{};
  city::wire::ObjectId middle_pole_id = city::wire::kInvalidObjectId;
  city::wire::ObjectId moving_pole_id = city::wire::kInvalidObjectId;
  city::wire::Transformd moving_pole_transform{};
};

bool make_d1_model_corner(city::wire::CoreState* state,
                          D1CornerFixture* fixture) {
  if (state == nullptr || fixture == nullptr) return false;
  constexpr city::wire::ModelAssemblyTemplateId kRowAssembly = 9811;
  constexpr city::wire::ModelAssemblyTemplateId kEndpointAssembly = 9812;

  city::wire::ModelAssemblyTemplate row_assembly{};
  row_assembly.id = kRowAssembly;
  city::wire::ModelAssemblyPart row_part{};
  row_part.part_id = 1;
  row_part.model_key = "d1_row";
  row_part.descriptor_version = 1;
  row_part.sockets.push_back(
      {"endpoint_mount", {0.0, 0.0, 0.04}, {0.0, 0.0, 1.0}});
  row_assembly.parts.push_back(row_part);
  row_assembly.endpoint_mount_socket =
      city::wire::AssemblySocketRef{1, "endpoint_mount"};

  city::wire::ModelAssemblyTemplate endpoint_assembly{};
  endpoint_assembly.id = kEndpointAssembly;
  city::wire::ModelAssemblyPart endpoint_part{};
  endpoint_part.part_id = 1;
  endpoint_part.model_key = "d1_fixture";
  endpoint_part.descriptor_version = 1;
  endpoint_part.sockets.push_back(
      {"wire", {0.0, 0.0, 0.18}, {1.0, 0.0, 0.0}});
  endpoint_assembly.parts.push_back(endpoint_part);
  endpoint_assembly.wire_socket =
      city::wire::AssemblySocketRef{1, "wire"};
  if (!state->RegisterModelAssemblyTemplate(row_assembly).ok ||
      !state->RegisterModelAssemblyTemplate(endpoint_assembly).ok) {
    return false;
  }

  const city::wire::BundleTemplateId lv_template_id =
      city::wire::DefaultBundleTemplateId(
          city::wire::BundleKind::kLowVoltage);
  city::wire::BundleTemplate lv =
      state->view().bundle_templates().at(lv_template_id);
  lv.row_fixture_assembly_id = kRowAssembly;
  lv.endpoint_fixture_assembly_id = kEndpointAssembly;
  if (!state->UpdateBundleTemplate(lv).ok) return false;

  const auto generated = state->GenerateFromBackboneSpec(poly3_req(*state));
  if (!generated.ok || generated.value.generated_pole_ids.size() != 3 ||
      generated.value.generated_span_ids.size() != 2) {
    return false;
  }
  fixture->generated = generated.value;
  fixture->middle_pole_id = generated.value.generated_pole_ids[1];
  fixture->moving_pole_id = generated.value.generated_pole_ids[2];
  const city::wire::Pole* moving =
      state->view().poles().find(fixture->moving_pole_id);
  if (moving == nullptr) return false;
  fixture->moving_pole_transform = moving->world_transform;
  return true;
}

std::vector<city::wire::ObjectId> endpoint_ports_on_pole(
    const city::wire::CoreState& state,
    const std::vector<city::wire::ObjectId>& span_ids,
    city::wire::ObjectId pole_id) {
  std::vector<city::wire::ObjectId> out{};
  for (city::wire::ObjectId span_id : span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) return {};
    for (city::wire::ObjectId port_id :
         {span->port_a_id, span->port_b_id}) {
      const city::wire::Port* port = state.view().ports().find(port_id);
      if (port != nullptr && port->owner_pole_id == pole_id) {
        out.push_back(port_id);
      }
    }
  }
  std::sort(out.begin(), out.end());
  return out;
}

bool bit_equal(const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
  return std::bit_cast<std::uint64_t>(a.x) ==
             std::bit_cast<std::uint64_t>(b.x) &&
         std::bit_cast<std::uint64_t>(a.y) ==
             std::bit_cast<std::uint64_t>(b.y) &&
         std::bit_cast<std::uint64_t>(a.z) ==
             std::bit_cast<std::uint64_t>(b.z);
}

std::size_t model_count(const city::wire::CoreState& state,
                        std::string_view model_key) {
  return static_cast<std::size_t>(std::count_if(
      state.view().visual_model_instances().instances.begin(),
      state.view().visual_model_instances().instances.end(),
      [&](const city::wire::VisualModelInstance& instance) {
        return instance.model_key == model_key;
      }));
}

std::size_t curve_part_count(
    const city::wire::CoreState& state,
    city::wire::VisualCurvePartKind kind) {
  return static_cast<std::size_t>(std::count_if(
      state.visual_curve_parts().parts.begin(),
      state.visual_curve_parts().parts.end(),
      [&](const city::wire::VisualCurvePart& part) {
        return part.kind == kind;
      }));
}

void append_vec_bits(std::ostringstream* out, const city::wire::Vec3d& value) {
  *out << std::bit_cast<std::uint64_t>(value.x) << ','
       << std::bit_cast<std::uint64_t>(value.y) << ','
       << std::bit_cast<std::uint64_t>(value.z);
}

std::string d1_derived_signature(const city::wire::CoreState& state) {
  std::vector<std::string> records{};
  for (const city::wire::SavedBackboneSpanBinding& binding :
       state.view().backbone().span_bindings) {
    const city::wire::SpanLayoutView layout =
        state.span_layout(binding.span_id);
    const city::wire::CurveCacheEntry* curve =
        state.find_curve_cache(binding.span_id);
    if (!layout.has_layout() || curve == nullptr ||
        curve->detail.sample_points.size() < 2) {
      return {};
    }
    std::ostringstream item{};
    item << "span:" << binding.edge_bundle_id << ':' << binding.lane_index
         << ':';
    append_vec_bits(&item, layout.entry->start.support_world);
    item << ':';
    append_vec_bits(&item, layout.entry->start.endpoint_world);
    item << ':';
    append_vec_bits(&item, layout.entry->end.support_world);
    item << ':';
    append_vec_bits(&item, layout.entry->end.endpoint_world);
    item << ':';
    append_vec_bits(&item, curve->detail.sample_points.front());
    item << ':';
    append_vec_bits(&item, curve->detail.sample_points.back());
    records.push_back(item.str());
  }
  for (const city::wire::VisualCurvePart& part :
       state.visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kNodePatch &&
        part.kind != city::wire::VisualCurvePartKind::kJumper) {
      continue;
    }
    std::ostringstream item{};
    item << "curve:" << static_cast<int>(part.kind) << ':'
         << part.source_node_id << ':' << part.lane_index;
    for (const city::wire::Vec3d& sample : part.samples) {
      item << ':';
      append_vec_bits(&item, sample);
    }
    records.push_back(item.str());
  }
  for (const city::wire::VisualModelInstance& instance :
       state.view().visual_model_instances().instances) {
    std::ostringstream item{};
    item << "model:" << instance.model_key << ':';
    append_vec_bits(&item, instance.world_transform.position);
    item << ':';
    append_vec_bits(&item, instance.world_transform.rotation_euler_deg);
    item << ':';
    append_vec_bits(&item, instance.world_transform.scale);
    records.push_back(item.str());
  }
  std::sort(records.begin(), records.end());
  std::ostringstream out{};
  for (const std::string& record : records) out << record << '\n';
  return out.str();
}

} // namespace

bool C810_backbone_normal_pair_uses_edge_ports_and_derived_fixture() {
  city::wire::CoreState state;
  D1CornerFixture fixture{};
  WIRE_TEST_EXPECT(make_d1_model_corner(&state, &fixture),
                   "failed to build D1 model corner");

  const std::vector<city::wire::ObjectId> before_ports =
      endpoint_ports_on_pole(state, fixture.generated.generated_span_ids,
                             fixture.middle_pole_id);
  WIRE_TEST_EXPECT(before_ports.size() == 2 && before_ports[0] != before_ports[1],
                   "normal pair did not keep two distinct endpoint ports");
  const city::wire::Port* before_a =
      state.view().ports().find(before_ports[0]);
  const city::wire::Port* before_b =
      state.view().ports().find(before_ports[1]);
  WIRE_TEST_EXPECT(before_a != nullptr && before_b != nullptr,
                   "normal pair endpoint port is missing");
  WIRE_TEST_EXPECT(bit_equal(before_a->world_position, before_b->world_position),
                   "normal pair endpoint ports no longer bit-match");
  WIRE_TEST_EXPECT(state.view().backbone_port_bindings_for_port(before_ports[0]).size() == 1,
                   "first normal pair port has wrong binding count");
  WIRE_TEST_EXPECT(state.view().backbone_port_bindings_for_port(before_ports[1]).size() == 1,
                   "second normal pair port has wrong binding count");
  WIRE_TEST_EXPECT(model_count(state, "d1_fixture") == 3,
                   "normal pair fixture count is " +
                       std::to_string(model_count(state, "d1_fixture")) +
                       " instead of 3");
  WIRE_TEST_EXPECT(curve_part_count(state,
                                    city::wire::VisualCurvePartKind::kNodePatch) == 1,
                   "normal pair did not derive one node patch");
  WIRE_TEST_EXPECT(curve_part_count(state, city::wire::VisualCurvePartKind::kJumper) == 0,
                   "normal pair derived a jumper");

  city::wire::Transformd moved = fixture.moving_pole_transform;
  moved.position.x += 1.0;
  if (!state.MovePole(fixture.moving_pole_id, moved).ok) return false;
  const std::vector<city::wire::ObjectId> moved_ports =
      endpoint_ports_on_pole(state, fixture.generated.generated_span_ids,
                             fixture.middle_pole_id);
  if (moved_ports != before_ports || model_count(state, "d1_fixture") != 3 ||
      curve_part_count(state,
                       city::wire::VisualCurvePartKind::kNodePatch) != 1 ||
      curve_part_count(state, city::wire::VisualCurvePartKind::kJumper) != 0) {
    return false;
  }
  const city::wire::Port* moved_a =
      state.view().ports().find(before_ports[0]);
  const city::wire::Port* moved_b =
      state.view().ports().find(before_ports[1]);
  return moved_a != nullptr && moved_b != nullptr &&
         bit_equal(moved_a->world_position, moved_b->world_position) &&
         model_count(state, "d1_fixture") == 3 &&
         curve_part_count(state,
                          city::wire::VisualCurvePartKind::kNodePatch) == 1 &&
         curve_part_count(state, city::wire::VisualCurvePartKind::kJumper) ==
             0;
}

bool C811_authoritative_v2_migrates_shared_pair_ports_without_visual_change() {
  std::string legacy{};
  WIRE_TEST_EXPECT(
      file_text(repo_root() / "domains" / "wire" / "tests" / "fixtures" /
                    "legacy_shared_pair_v2.txt",
                &legacy),
      "legacy shared-port fixture is missing");
  city::wire::CoreState loaded;
  const auto loaded_result = loaded.DeserializeAuthoritative(legacy);
  WIRE_TEST_EXPECT(loaded_result.ok, loaded_result.error);
  const std::string actual_signature = d1_derived_signature(loaded);
  WIRE_TEST_EXPECT(!actual_signature.empty(), "migrated derived signature is empty");

  const std::filesystem::path expected_path =
      repo_root() / "domains" / "wire" / "tests" / "fixtures" /
      "legacy_shared_pair_v2.expected.txt";
  std::string expected_signature{};
  WIRE_TEST_EXPECT(file_text(expected_path, &expected_signature),
                   "legacy expected signature is missing");
  const std::size_t signature_difference =
      std::mismatch(actual_signature.begin(), actual_signature.end(),
                    expected_signature.begin(), expected_signature.end())
          .first -
      actual_signature.begin();
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      actual_signature == expected_signature,
      "migrated derived signature changed at byte " +
          std::to_string(signature_difference) + " actual_size=" +
          std::to_string(actual_signature.size()) + " expected_size=" +
          std::to_string(expected_signature.size()) + " actual=" +
          actual_signature.substr(signature_difference, 96) + " expected=" +
          expected_signature.substr(signature_difference, 96));

  for (const city::wire::SavedBackbonePortBinding& binding :
       loaded.view().backbone().port_bindings) {
    WIRE_TEST_EXPECT_ANCHOR(
        loaded.view()
                .backbone_port_bindings_for_port(binding.port_id)
                .size() == 1,
        "migrated Port " + std::to_string(binding.port_id) +
            " is not owned by exactly one edge endpoint binding");
  }
  std::string migrated{};
  const auto migrated_save = loaded.SerializeAuthoritative(&migrated);
  WIRE_TEST_EXPECT(migrated_save.ok, migrated_save.error);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      migrated != legacy, "v2 shared-Port fixture was not migrated");
  city::wire::CoreState reloaded;
  std::string resaved{};
  const auto reloaded_result = reloaded.DeserializeAuthoritative(migrated);
  WIRE_TEST_EXPECT(reloaded_result.ok, reloaded_result.error);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      d1_derived_signature(reloaded) == expected_signature,
      "migrated signature changed after reload");
  const auto resaved_result = reloaded.SerializeAuthoritative(&resaved);
  WIRE_TEST_EXPECT(resaved_result.ok, resaved_result.error);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      resaved == migrated, "migrated authoritative bytes are not stable");
  return true;
}

bool C812_authoritative_v2_rejects_ambiguous_shared_port_migration() {
  std::string legacy{};
  if (!file_text(repo_root() / "domains" / "wire" / "tests" / "fixtures" /
                     "legacy_shared_pair_v2.txt",
                 &legacy)) {
    return false;
  }
  std::istringstream lines(legacy);
  std::ostringstream ambiguous{};
  std::string line{};
  while (std::getline(lines, line)) {
    if (line.starts_with("authoritative.backbone.row_continuities.")) {
      continue;
    }
    ambiguous << line << '\n';
  }
  ambiguous << "authoritative.backbone.row_continuities.count=0\n";

  city::wire::CoreState state;
  std::string before{};
  std::string after{};
  if (!state.SerializeAuthoritative(&before).ok) return false;
  const auto loaded = state.DeserializeAuthoritative(ambiguous.str());
  return !loaded.ok &&
         loaded.error.find(
             "authoritative migration unsupported: shared pair port cannot be split exactly") !=
             std::string::npos &&
         state.SerializeAuthoritative(&after).ok && after == before;
}

bool C813_backbone_move_pole_rederives_pair_representation() {
  city::wire::CoreState state;
  D1CornerFixture fixture{};
  if (!make_d1_model_corner(&state, &fixture)) return false;

  const std::vector<city::wire::ObjectId> ports =
      endpoint_ports_on_pole(state, fixture.generated.generated_span_ids,
                             fixture.middle_pole_id);
  if (ports.size() != 2 || ports[0] == ports[1]) return false;
  std::vector<city::wire::SavedBackboneRowKey> row_keys{};
  for (city::wire::ObjectId port_id : ports) {
    const auto bindings = state.view().backbone_port_bindings_for_port(port_id);
    if (bindings.size() != 1 || bindings.front() == nullptr) return false;
    const city::wire::SavedBackboneEdgeBundle* edge_bundle =
        state.view().backbone_edge_bundle(bindings.front()->edge_bundle_id);
    if (edge_bundle == nullptr ||
        bindings.front()->row_key.node_id == city::wire::kInvalidObjectId ||
        bindings.front()->row_key.edge_id != edge_bundle->edge_id) {
      return false;
    }
    row_keys.push_back(bindings.front()->row_key);
  }
  const city::wire::Port* normal_a = state.view().ports().find(ports[0]);
  const city::wire::Port* normal_b = state.view().ports().find(ports[1]);
  if (normal_a == nullptr || normal_b == nullptr ||
      !bit_equal(normal_a->world_position, normal_b->world_position) ||
      model_count(state, "d1_fixture") != 3 ||
      curve_part_count(state, city::wire::VisualCurvePartKind::kNodePatch) !=
          1 ||
      curve_part_count(state, city::wire::VisualCurvePartKind::kJumper) != 0) {
    return false;
  }

  city::wire::Transformd sharp = fixture.moving_pole_transform;
  sharp.position = {4.0, 2.0, sharp.position.z};
  if (!state.MovePole(fixture.moving_pole_id, sharp).ok) return false;
  const std::vector<city::wire::ObjectId> sharp_ports =
      endpoint_ports_on_pole(state, fixture.generated.generated_span_ids,
                             fixture.middle_pole_id);
  if (sharp_ports != ports || model_count(state, "d1_fixture") != 4 ||
      curve_part_count(state, city::wire::VisualCurvePartKind::kNodePatch) !=
          0 ||
      curve_part_count(state, city::wire::VisualCurvePartKind::kJumper) != 1) {
    return false;
  }
  const city::wire::Port* sharp_a = state.view().ports().find(ports[0]);
  const city::wire::Port* sharp_b = state.view().ports().find(ports[1]);
  if (sharp_a == nullptr || sharp_b == nullptr ||
      bit_equal(sharp_a->world_position, sharp_b->world_position)) {
    return false;
  }
  for (std::size_t index = 0; index < ports.size(); ++index) {
    const auto bindings =
        state.view().backbone_port_bindings_for_port(ports[index]);
    if (bindings.size() != 1 || bindings.front() == nullptr ||
        !(bindings.front()->row_key == row_keys[index])) {
      return false;
    }
  }

  if (!state.MovePole(fixture.moving_pole_id,
                      fixture.moving_pole_transform).ok) {
    return false;
  }
  const city::wire::Port* restored_a = state.view().ports().find(ports[0]);
  const city::wire::Port* restored_b = state.view().ports().find(ports[1]);
  return restored_a != nullptr && restored_b != nullptr &&
         bit_equal(restored_a->world_position, restored_b->world_position) &&
         model_count(state, "d1_fixture") == 3 &&
         curve_part_count(state,
                          city::wire::VisualCurvePartKind::kNodePatch) == 1 &&
         curve_part_count(state, city::wire::VisualCurvePartKind::kJumper) ==
             0;
}

} // namespace backbone_tests
