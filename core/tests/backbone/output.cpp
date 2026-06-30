#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace helpers;

namespace backbone_tests {

bool C506_backbone_support_group_is_placement_layer() {
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.hpp";
  std::string h;
  if (!file_text(header, &h)) {
    return false;
  }
  return contains_text(h, "struct group_member") && contains_text(h, "struct group") &&
         contains_text(h, "std::vector<group_member> row_members") && contains_text(h, "Vec3d group_axis") &&
         contains_text(h, "int vertical_order") && contains_text(h, "double lower_offset_m");
}

bool C507_backbone_support_group_built_after_intent() {
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.hpp";
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string h;
  std::string cpp;
  if (!file_text(header, &h) || !file_text(source, &cpp)) {
    return false;
  }
  return contains_text(h, "EditResult<groups> make(const pairs& ps, const intent& intents) const") &&
         contains_text(h, "rules make(const topo& made, const groups& placement) const") &&
         !contains_text(h, "rules make(const topo& made, const intent& intents) const") &&
         contains_text(cpp, "EditResult<groups> placement = make(ps.value, intents.value)");
}

bool C508_backbone_support_group_drives_lowered_rules() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanLayoutRulesView rules = state.span_layout_rules(span_id);
    if (!rules.has_rule()) {
      return false;
    }
    const auto has_lowered_group = [](const wire::core::EndpointLayoutRule& endpoint) {
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
  const std::filesystem::path header = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.hpp";
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
  const std::filesystem::path header = repo_root() / "core" / "include" / "wire" / "core" / "span_layout_types.hpp";
  std::string h;
  if (!file_text(header, &h)) {
    return false;
  }
  const std::size_t fn_pos = h.find("inline void ApplyEndpointLayoutRule");
  const std::size_t next_pos = h.find("struct SpanLayoutRule", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = h.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "rule.branch_down_offset_m") && contains_text(body, "dst.endpoint_world.z -= lower_offset") &&
         !contains_text(body, "kLowerOffsetM");
}

bool C511_backbone_draw_saved_from_geom() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span_id);
    const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    const wire::core::SpanRenderCacheEntry* render = state.find_span_render_cache(span_id);
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

bool C513_backbone_support_visual_placeholder_from_layout() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    if (visual == nullptr || !layout.has_layout()) {
      return false;
    }
    if (layout.entry->start.default_lower_required || layout.entry->end.default_lower_required) {
      return !visual->parts.empty();
    }
  }
  return false;
}

bool C514_backbone_draw_save_is_direct() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "state" / "span_runtime.cpp";
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

bool C515_backbone_rejects_existing_pole_without_saved_graph() {
  wire::core::CoreState state;
  wire::core::Transformd tf{};
  tf.position = {0.0, 0.0, 0.0};
  const auto pole = state.AddPole(tf);
  if (!pole.ok) {
    return false;
  }
  const std::size_t pole_count = state.view().poles().size();
  const std::size_t span_count = state.view().spans().size();
  const std::size_t graph_nodes = state.view().backbone().nodes.size();
  const std::size_t graph_edges = state.view().backbone().edges.size();
  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {tf.position, {12.0, 0.0, 0.0}};
  req.path.node_specs = {pole_spec(0, pole.value)};
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && contains_text(out.error, "saved backbone graph missing") &&
         state.view().poles().size() == pole_count && state.view().spans().size() == span_count &&
         state.view().backbone().nodes.size() == graph_nodes && state.view().backbone().edges.size() == graph_edges;
}

bool C516_backbone_generated_pole_with_saved_graph_still_connects() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr || state.view().backbone_node_for_pole(b) == nullptr) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  return second.ok && !second.value.generated_span_ids.empty() && state.view().pole_frontier(b).edge_ids.size() == 3;
}

bool C518_backbone_lowered_layout_keeps_support_world_at_port_height() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    if (!layout.has_layout()) {
      return false;
    }
    const auto endpoint_ok = [&](const wire::core::LayoutEndpoint& endpoint) {
      if (!endpoint.default_lower_required && !endpoint.lower_required) {
        return false;
      }
      const wire::core::Port* port = state.view().ports().find(endpoint.port_id);
      if (port == nullptr) {
        return false;
      }
      const double lower_offset =
          endpoint.branch_down_offset_m > 0.0 ? endpoint.branch_down_offset_m : endpoint.automatic_branch_down_offset_m;
      return lower_offset > 0.0 && almost_equal(endpoint.support_world.z, port->world_position.z, 1e-9) &&
             almost_equal(endpoint.endpoint_world.z, port->world_position.z - lower_offset, 1e-9);
    };
    if (endpoint_ok(layout.entry->start) || endpoint_ok(layout.entry->end)) {
      return true;
    }
  }
  return false;
}

bool C519_backbone_draw_placeholder_uses_layout_points() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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

  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    if (!layout.has_layout() || visual == nullptr) {
      return false;
    }
    auto part_matches = [&](const wire::core::LayoutEndpoint& endpoint) {
      if (!endpoint.default_lower_required && !endpoint.lower_required) {
        return false;
      }
      for (const wire::core::VisualPart& part : visual->parts) {
        if (almost_equal(part.a, endpoint.support_world, 1e-9) &&
            almost_equal(part.b, endpoint.endpoint_world, 1e-9)) {
          return true;
        }
      }
      return false;
    };
    if (part_matches(layout.entry->start) || part_matches(layout.entry->end)) {
      return true;
    }
  }
  return false;
}

bool C521_backbone_context_link_preserves_saved_dir() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
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
         C519_backbone_draw_placeholder_uses_layout_points();
}

bool C534_backbone_invalid_inputs_stop_before_emit() {
  return C409_backbone_rejects_missing_port_band() && C414_backbone_simple_avoid_detour_supported() &&
         C515_backbone_rejects_existing_pole_without_saved_graph();
}

bool C536_backbone_draw_consumer_outputs_are_minimal() {
  return C511_backbone_draw_saved_from_geom() && C513_backbone_support_visual_placeholder_from_layout() &&
         C519_backbone_draw_placeholder_uses_layout_points();
}

bool C539_backbone_supported_request_creates_saved_graph_outputs() {
  return C524_backbone_scenario_simple_line_mainline() && C523_backbone_scope_gate_matches_entrypoint();
}

} // namespace backbone_tests
