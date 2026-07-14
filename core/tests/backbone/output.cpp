#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "wire/core/core_test_hook.hpp"
#include "wire/core/coord_utils.hpp"
#include "wire/core/core_view.hpp"

#include "../../src/generation/backbone/curve_parts.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
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
         contains_text(h, "int vertical_order") && contains_text(h, "double endpoint_offset_m");
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
         contains_text(h, "rules make(const topo& made, const pairs& ps, const groups& placement) const") &&
         !contains_text(h, "rules make(const topo& made, const intent& intents) const") &&
         contains_text(cpp, "return make(ps.value, intents.value)");
}

bool C508_backbone_support_group_drives_lowered_rules() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
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
  return contains_text(body, "rule.endpoint_offset_z_m") && contains_text(body, "dst.endpoint_world.z += endpoint_offset") &&
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
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
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

bool C740_visual_curve_part_stats_count_full_curve_builds() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::VisualCurvePartCache& cache = state.visual_curve_parts();
  const std::size_t expected_sections = out.value.generated_span_ids.size();
  if (cache.stats.sections != expected_sections || cache.stats.curve_builds != expected_sections) {
    return false;
  }

  const wire::core::ObjectId span_id = out.value.generated_span_ids.front();
  const wire::core::SpanLayoutView span_layout = state.span_layout(span_id);
  const wire::core::CurveCacheEntry* cached = state.find_curve_cache(span_id);
  if (!span_layout.has_layout() || cached == nullptr || cached->detail.sample_points.size() < 3) {
    return false;
  }
  wire::core::generation::backbone::layout made{};
  made.entries.push_back(*span_layout.entry);
  wire::core::generation::backbone::curve built{};
  wire::core::DetailCurve injected = cached->detail;
  const std::size_t middle = injected.sample_points.size() / 2;
  injected.sample_points[middle].z += 0.123;
  const wire::core::Vec3d expected = injected.sample_points[middle];
  built.data.push_back({span_id, std::move(injected)});
  const wire::core::VisualCurvePartCache supplied =
      wire::core::generation::backbone::make_visual_curve_parts(state, made, {span_id}, &built);
  for (const wire::core::VisualCurvePart& part : supplied.parts) {
    if (part.kind == wire::core::VisualCurvePartKind::kEdgeBody && part.source_span_id == span_id &&
        part.section_key.is_base() &&
        std::find_if(part.samples.begin(), part.samples.end(), [&](const wire::core::Vec3d& point) {
          return almost_equal(point, expected, 1e-12);
        }) != part.samples.end()) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> visual_part_snapshot(const wire::core::VisualCurvePartCache& cache) {
  std::vector<std::string> out{};
  out.reserve(cache.parts.size());
  for (const wire::core::VisualCurvePart& part : cache.parts) {
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
      const wire::core::Vec3d& first = part.samples.front();
      const wire::core::Vec3d& last = part.samples.back();
      ss << ":" << first.x << "," << first.y << "," << first.z << ":" << last.x << "," << last.y << ","
         << last.z;
    }
    out.push_back(ss.str());
  }
  return out;
}

bool C741_scoped_visual_curve_rebuild_matches_full_rebuild() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() < 2) {
    return false;
  }
  const wire::core::ObjectId terminal_pole = first.value.generated_pole_ids.back();
  const auto* pole = state.view().poles().find(terminal_pole);
  if (pole == nullptr) {
    return false;
  }
  wire::core::BackboneSpec extension = line_req(state);
  extension.path.polyline = {pole->world_transform.position, {24.0, 0.0, 0.0}};
  extension.path.node_specs = {pole_spec(0, terminal_pole)};
  const auto second = state.GenerateFromBackboneSpec(extension);
  if (!second.ok || second.value.generated_span_ids.size() != 1) {
    return false;
  }
  const wire::core::VisualCurvePartCache scoped = state.visual_curve_parts();
  const wire::core::VisualCurvePartCache full = wire::core::generation::backbone::make_visual_curve_parts(state, {});
  return scoped.stats.curve_builds == 2 && visual_part_snapshot(scoped) == visual_part_snapshot(full);
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
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
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
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
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
  const auto first = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(hv_branch_req(state, b, pole_b->world_transform.position));
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

namespace {

std::size_t visual_curve_part_count(const wire::core::CoreState& state, wire::core::VisualCurvePartKind kind) {
  std::size_t count = 0;
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == kind) {
      ++count;
    }
  }
  return count;
}

bool tangent_compatible(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  auto length = [](const wire::core::Vec3d& value) {
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

bool sampled_boundary_is_g1(const wire::core::VisualCurvePart& a, const wire::core::VisualCurvePart& b,
                            const wire::core::Vec3d& boundary) {
  auto direction_into_part = [&](const wire::core::VisualCurvePart& part, wire::core::Vec3d* out) {
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

  wire::core::Vec3d a_direction{};
  wire::core::Vec3d b_direction{};
  if (!direction_into_part(a, &a_direction) || !direction_into_part(b, &b_direction)) {
    return false;
  }
  return tangent_compatible(a_direction, {-b_direction.x, -b_direction.y, -b_direction.z});
}

bool boundary_matches_source_curve(const wire::core::CoreState& state,
                                   const wire::core::VisualCurvePart& body,
                                   const wire::core::Vec3d& boundary) {
  const wire::core::CurveCacheEntry* source = state.find_curve_cache(body.source_span_id);
  if (source == nullptr) return false;
  const wire::core::DetailCurve& curve = source->detail;
  const wire::core::Vec3d start = curve.EvaluatePosition(0.0);
  const wire::core::Vec3d end = curve.EvaluatePosition(1.0);
  const auto distance = [](const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
    return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
  };
  const bool from_start = distance(boundary, start) < distance(boundary, end);
  const wire::core::Vec3d attachment = from_start ? start : end;
  const double target_xy = std::hypot(boundary.x - attachment.x, boundary.y - attachment.y);
  double low = from_start ? 0.0 : 0.5;
  double high = from_start ? 0.5 : 1.0;
  for (int iteration = 0; iteration < 60; ++iteration) {
    const double u = (low + high) * 0.5;
    const wire::core::Vec3d point = curve.EvaluatePosition(u);
    const double distance_xy = std::hypot(point.x - attachment.x, point.y - attachment.y);
    if ((distance_xy < target_xy) == from_start) low = u;
    else high = u;
  }
  const double u = (low + high) * 0.5;
  wire::core::Vec3d expected_tangent = curve.EvaluateTangent(u);
  if (!from_start) expected_tangent = {-expected_tangent.x, -expected_tangent.y, -expected_tangent.z};
  const wire::core::Vec3d body_tangent =
      almost_equal(body.boundary_a, boundary, 1e-9) ? body.tangent_a : body.tangent_b;
  return almost_equal(boundary, curve.EvaluatePosition(u), 1e-6) &&
         tangent_compatible(body_tangent, expected_tangent);
}

bool body_samples_follow_source_curve(const wire::core::CoreState& state,
                                      const wire::core::VisualCurvePart& body) {
  if (!body.has_section_key || !body.section_key.is_base() || body.samples.size() < 2) return false;
  const wire::core::CurveCacheEntry* source = state.find_curve_cache(body.source_span_id);
  if (source == nullptr) return false;
  const wire::core::DetailCurve& curve = source->detail;
  const wire::core::Vec3d start = curve.EvaluatePosition(0.0);
  const wire::core::Vec3d end = curve.EvaluatePosition(1.0);
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double horizontal_length2 = dx * dx + dy * dy;
  if (horizontal_length2 <= 1e-12) return false;
  for (const wire::core::Vec3d& sample : body.samples) {
    const double u = std::clamp(((sample.x - start.x) * dx + (sample.y - start.y) * dy) /
                                    horizontal_length2,
                                0.0, 1.0);
    if (!almost_equal(sample, curve.EvaluatePosition(u), 1e-7)) return false;
  }
  return true;
}

double sampled_curve_length(const wire::core::VisualCurvePart& part) {
  double length = 0.0;
  for (std::size_t i = 1; i < part.samples.size(); ++i) {
    const wire::core::Vec3d delta{part.samples[i].x - part.samples[i - 1].x,
                                  part.samples[i].y - part.samples[i - 1].y,
                                  part.samples[i].z - part.samples[i - 1].z};
    length += std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
  }
  return length;
}

bool finite_visual_curve_parts(const wire::core::CoreState& state) {
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    for (const wire::core::Vec3d& p : part.samples) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        return false;
      }
    }
  }
  return true;
}

double distance_to_chord(const wire::core::Vec3d& p, const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  const wire::core::Vec3d ab{b.x - a.x, b.y - a.y, b.z - a.z};
  const wire::core::Vec3d ap{p.x - a.x, p.y - a.y, p.z - a.z};
  const double ab_len2 = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
  if (ab_len2 <= 1e-12) {
    return 0.0;
  }
  const double t = std::clamp((ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / ab_len2, 0.0, 1.0);
  const wire::core::Vec3d closest{a.x + ab.x * t, a.y + ab.y * t, a.z + ab.z * t};
  const wire::core::Vec3d d{p.x - closest.x, p.y - closest.y, p.z - closest.z};
  return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}

double cross_xy(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  return a.x * b.y - a.y * b.x;
}

bool point_in_xy_triangle(const wire::core::Vec3d& point, const wire::core::Vec3d& a,
                          const wire::core::Vec3d& b, const wire::core::Vec3d& c) {
  const wire::core::Vec3d ab{b.x - a.x, b.y - a.y, 0.0};
  const wire::core::Vec3d bc{c.x - b.x, c.y - b.y, 0.0};
  const wire::core::Vec3d ca{a.x - c.x, a.y - c.y, 0.0};
  const wire::core::Vec3d ap{point.x - a.x, point.y - a.y, 0.0};
  const wire::core::Vec3d bp{point.x - b.x, point.y - b.y, 0.0};
  const wire::core::Vec3d cp{point.x - c.x, point.y - c.y, 0.0};
  const double c0 = cross_xy(ab, ap);
  const double c1 = cross_xy(bc, bp);
  const double c2 = cross_xy(ca, cp);
  return (c0 >= -1e-8 && c1 >= -1e-8 && c2 >= -1e-8) ||
         (c0 <= 1e-8 && c1 <= 1e-8 && c2 <= 1e-8);
}

bool sampled_xy_turn_is_monotonic(const wire::core::VisualCurvePart& patch) {
  if (patch.samples.size() < 3) {
    return false;
  }
  const wire::core::Vec3d incoming{-patch.tangent_a.x, -patch.tangent_a.y, 0.0};
  const wire::core::Vec3d outgoing{patch.tangent_b.x, patch.tangent_b.y, 0.0};
  const double expected_turn = cross_xy(incoming, outgoing);
  if (std::abs(expected_turn) <= 1e-9) {
    return true;
  }
  for (std::size_t i = 1; i + 1 < patch.samples.size(); ++i) {
    const wire::core::Vec3d before{patch.samples[i].x - patch.samples[i - 1].x,
                                   patch.samples[i].y - patch.samples[i - 1].y, 0.0};
    const wire::core::Vec3d after{patch.samples[i + 1].x - patch.samples[i].x,
                                  patch.samples[i + 1].y - patch.samples[i].y, 0.0};
    if (cross_xy(before, after) * expected_turn < -1e-9) {
      return false;
    }
  }
  return true;
}

bool touches_patch_boundary(const wire::core::VisualCurvePart& body, const wire::core::VisualCurvePart& patch) {
  return almost_equal(body.boundary_a, patch.boundary_a, 1e-9) ||
         almost_equal(body.boundary_b, patch.boundary_a, 1e-9) ||
         almost_equal(body.boundary_a, patch.boundary_b, 1e-9) ||
         almost_equal(body.boundary_b, patch.boundary_b, 1e-9);
}

bool same_visual_cable_section_family(const wire::core::CableSectionKey& a, const wire::core::CableSectionKey& b) {
  return a.is_base() == b.is_base() && a.rule_owner_id == b.rule_owner_id &&
         a.rule_id == b.rule_id && a.instance_index == b.instance_index;
}

wire::core::CablePopulationRule two_extra_lv_population_rule() {
  wire::core::CablePopulationRule rule{};
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

bool enable_two_extra_lv_population(wire::core::CoreState& state) {
  wire::core::BundleTemplate lv_template = state.view().bundle_templates().at(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage));
  lv_template.population_rules.push_back(two_extra_lv_population_rule());
  return state.UpdateBundleTemplate(lv_template).ok;
}

bool prepare_two_lane_low_voltage_for_run_test(wire::core::CoreState& state) {
  wire::core::BundleTemplate lv_template = state.view().bundle_templates().at(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage));
  lv_template.count_rule = wire::core::BundleCountRuleKind::kFixed;
  lv_template.fixed_count = 2;
  lv_template.default_count = 2;
  return state.UpdateBundleTemplate(lv_template).ok;
}

std::vector<const wire::core::VisualCurvePart*> base_edge_bodies(const wire::core::CoreState& state) {
  std::vector<const wire::core::VisualCurvePart*> bodies{};
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == wire::core::VisualCurvePartKind::kEdgeBody && part.has_section_key &&
        part.section_key.is_base()) {
      bodies.push_back(&part);
    }
  }
  return bodies;
}

std::vector<const wire::core::VisualCurvePart*> extra_edge_bodies(const wire::core::CoreState& state,
                                                                  std::size_t instance_index) {
  std::vector<const wire::core::VisualCurvePart*> bodies{};
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == wire::core::VisualCurvePartKind::kEdgeBody && part.has_section_key &&
        !part.section_key.is_base() && part.section_key.instance_index == instance_index) {
      bodies.push_back(&part);
    }
  }
  return bodies;
}

bool all_same_nonzero_run(const std::vector<const wire::core::VisualCurvePart*>& parts) {
  if (parts.empty() || parts.front()->cable_run_id == 0) {
    return false;
  }
  const wire::core::CableRunId run_id = parts.front()->cable_run_id;
  return std::all_of(parts.begin(), parts.end(), [&](const wire::core::VisualCurvePart* part) {
    return part->cable_run_id == run_id;
  });
}

bool all_distinct_nonzero_runs(const std::vector<const wire::core::VisualCurvePart*>& parts) {
  std::unordered_set<wire::core::CableRunId> seen{};
  for (const wire::core::VisualCurvePart* part : parts) {
    if (part->cable_run_id == 0 || !seen.insert(part->cable_run_id).second) {
      return false;
    }
  }
  return !parts.empty();
}

bool contains_span(const std::vector<wire::core::ObjectId>& ids, wire::core::ObjectId id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

} // namespace

bool C634_backbone_terminal_nodes_create_no_node_patch_curve() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  return out.ok && visual_curve_part_count(state, wire::core::VisualCurvePartKind::kNodePatch) == 0 &&
         visual_curve_part_count(state, wire::core::VisualCurvePartKind::kEdgeBody) ==
             out.value.generated_span_ids.size();
}

bool C760_backbone_terminal_edge_body_ends_at_port() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) return false;
  std::size_t terminal_bodies = 0;
  for (const wire::core::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
    if (body.kind != wire::core::VisualCurvePartKind::kEdgeBody || !body.has_section_key ||
        !body.section_key.is_base()) continue;
    const wire::core::Span* span = state.view().spans().find(body.source_span_id);
    const wire::core::Port* port_a = span == nullptr ? nullptr : state.view().ports().find(span->port_a_id);
    const wire::core::Port* port_b = span == nullptr ? nullptr : state.view().ports().find(span->port_b_id);
    if (port_a == nullptr || port_b == nullptr || !almost_equal(body.boundary_a, port_a->world_position, 1e-9) ||
        !almost_equal(body.boundary_b, port_b->world_position, 1e-9)) return false;
    ++terminal_bodies;
  }
  return terminal_bodies == out.value.generated_span_ids.size();
}

bool C761_default_optical_bundle_emits_helix() {
  wire::core::CoreState state;
  wire::core::BackboneSpec request = line_req(state);
  const auto optical_template = state.view().bundle_templates().find(wire::core::kDefaultOpticalBundleTemplateId);
  const auto communication_template =
      state.view().bundle_templates().find(wire::core::kDefaultCommunicationBundleTemplateId);
  if (optical_template == state.view().bundle_templates().end() ||
      communication_template == state.view().bundle_templates().end() ||
      !optical_template->second.span_visual_assembly.helix_enabled ||
      optical_template->second.span_visual_assembly.helix_samples_per_turn != 6 ||
      optical_template->second.support_wire_pole_band_id != 600 ||
      communication_template->second.span_visual_assembly.helix_enabled ||
      communication_template->second.support_wire_pole_band_id != 0) return false;
  request.bundles.clear();
  request.pole_type_id = optical_template->second.related_pole_type_id;
  add_backbone_bundle(request, wire::core::BundleKind::kOptical);
  const auto out = state.GenerateFromBackboneSpec(request);
  if (!out.ok || out.value.generated_span_ids.empty()) return false;
  std::size_t support_count = 0;
  std::size_t helix_count = 0;
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    support_count += part.kind == wire::core::VisualCurvePartKind::kSupplemental &&
        part.supplemental_kind == wire::core::VisualSupplementalKind::kSupportPath;
    helix_count += part.kind == wire::core::VisualCurvePartKind::kSupplemental &&
        part.supplemental_kind == wire::core::VisualSupplementalKind::kHelix;
  }
  return support_count == out.value.generated_span_ids.size() && helix_count == support_count;
}

bool C635_backbone_simple_continuous_node_creates_node_patch_curve() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok || out.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::SavedBackboneNode* node = state.view().backbone_node_for_pole(out.value.generated_pole_ids[1]);
  if (node == nullptr) {
    return false;
  }
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == wire::core::VisualCurvePartKind::kNodePatch && part.source_node_id == node->node_id &&
        part.node_patch_classification == wire::core::NodePatchClassification::kSimpleContinuous &&
        part.incident_edge_ids.size() == 2 && part.source_span_id == wire::core::kInvalidObjectId) {
      return true;
    }
  }
  return false;
}

bool C636_backbone_edge_body_stops_at_node_patch_boundaries() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok || out.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::SavedBackboneNode* node = state.view().backbone_node_for_pole(out.value.generated_pole_ids[1]);
  if (node == nullptr) {
    return false;
  }
  const wire::core::Vec3d node_pos = node->position;
  for (const wire::core::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != wire::core::VisualCurvePartKind::kNodePatch || patch.source_node_id != node->node_id) {
      continue;
    }
    bool boundary_a_used = false;
    bool boundary_b_used = false;
    for (const wire::core::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != wire::core::VisualCurvePartKind::kEdgeBody) {
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
  wire::core::CoreState state;
  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  settings.sag_factor = 0.03;
  if (!state.UpdateGeometrySettings(settings).ok) {
    return false;
  }
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok) {
    return false;
  }
  for (const wire::core::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != wire::core::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    bool a_ok = false;
    bool b_ok = false;
    bool a_source_ok = false;
    bool b_source_ok = false;
    bool a_shape_ok = false;
    bool b_shape_ok = false;
    for (const wire::core::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != wire::core::VisualCurvePartKind::kEdgeBody) {
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
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {5.0, 8.660254037844386, 0.0}};
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) return false;
  for (const wire::core::VisualCurvePart& jumper : state.view().visual_curve_parts().parts) {
    if (jumper.kind != wire::core::VisualCurvePartKind::kJumper) continue;
    bool a_ok = false;
    bool b_ok = false;
    for (const wire::core::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != wire::core::VisualCurvePartKind::kEdgeBody) continue;
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
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.5, 0.0}};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || state.view().visual_curve_parts().parts.empty() || !finite_visual_curve_parts(state)) {
    return false;
  }
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    const wire::core::Vec3d chord{part.boundary_b.x - part.boundary_a.x, part.boundary_b.y - part.boundary_a.y,
                                  part.boundary_b.z - part.boundary_a.z};
    const double chord_length = std::sqrt(chord.x * chord.x + chord.y * chord.y + chord.z * chord.z);
    if (chord_length > 1e-9 && sampled_curve_length(part) > chord_length * 2.0) {
      return false;
    }
  }
  return true;
}

bool C639_backbone_node_patch_curve_is_not_straight_chord() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok) {
    return false;
  }
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != wire::core::VisualCurvePartKind::kNodePatch) {
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
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok) {
    return false;
  }
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != wire::core::VisualCurvePartKind::kNodePatch) {
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
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_pole_ids.empty() || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId pole_id = generated.value.generated_pole_ids.front();
  const wire::core::ObjectId span_id = generated.value.generated_span_ids.front();
  const wire::core::VisualCurvePart* before_body = nullptr;
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == wire::core::VisualCurvePartKind::kEdgeBody && part.source_span_id == span_id) {
      before_body = &part;
      break;
    }
  }
  if (before_body == nullptr) {
    return false;
  }
  const wire::core::Vec3d before_boundary = before_body->boundary_a;
  const auto tilted = state.ApplyPoleTilt({pole_id}, 12.0);
  if (!tilted.ok || !tilted.value) {
    return false;
  }
  const wire::core::SpanLayoutView layout = state.span_layout(span_id);
  if (!layout.has_layout()) {
    return false;
  }
  const wire::core::VisualCurvePart* after_body = nullptr;
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == wire::core::VisualCurvePartKind::kEdgeBody && part.source_span_id == span_id) {
      after_body = &part;
      break;
    }
  }
  if (after_body == nullptr) {
    return false;
  }
  const bool pole_is_start = layout.entry->start.endpoint_node_id == pole_id;
  const wire::core::Vec3d expected =
      pole_is_start ? layout.entry->start.endpoint_world : layout.entry->end.endpoint_world;
  const wire::core::Vec3d actual = pole_is_start ? after_body->boundary_a : after_body->boundary_b;
  return !almost_equal(before_boundary, actual, 1e-9) && almost_equal(actual, expected, 1e-9);
}

bool C659_backbone_draw_time_tilt_materializes_ports_before_spans() {
  wire::core::CoreState untilted_state;
  wire::core::BackboneSpec untilted_req = line_req(untilted_state);
  const auto untilted = untilted_state.GenerateFromBackboneSpec(untilted_req);
  if (!untilted.ok || untilted.value.generated_pole_ids.empty() || untilted.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId untilted_pole_id = untilted.value.generated_pole_ids.front();
  const wire::core::ObjectId untilted_span_id = untilted.value.generated_span_ids.front();
  const wire::core::Pole* untilted_pole = untilted_state.view().poles().find(untilted_pole_id);
  const wire::core::SpanLayoutView untilted_layout = untilted_state.span_layout(untilted_span_id);
  if (untilted_pole == nullptr || !untilted_layout.has_layout() || untilted_pole->tilt_magnitude_deg != 0.0) {
    return false;
  }

  wire::core::CoreState generated_state;
  wire::core::BackboneSpec generated_req = line_req(generated_state);
  generated_req.pole_placement.enable_tilt = true;
  generated_req.pole_placement.max_tilt_deg = 12.0;
  const auto generated = generated_state.GenerateFromBackboneSpec(generated_req);
  if (!generated.ok || generated.value.generated_pole_ids.empty() || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId generated_pole_id = generated.value.generated_pole_ids.front();
  const wire::core::ObjectId generated_span_id = generated.value.generated_span_ids.front();
  const wire::core::Pole* generated_pole = generated_state.view().poles().find(generated_pole_id);
  const wire::core::Span* generated_span = generated_state.view().spans().find(generated_span_id);
  const wire::core::SpanLayoutView generated_layout = generated_state.span_layout(generated_span_id);
  if (generated_pole == nullptr || generated_span == nullptr || !generated_layout.has_layout() ||
      generated_pole->tilt_magnitude_deg <= 0.0) {
    return false;
  }
  const wire::core::Port* generated_port_a = generated_state.view().ports().find(generated_span->port_a_id);
  const wire::core::Port* generated_port_b = generated_state.view().ports().find(generated_span->port_b_id);
  if (generated_port_a == nullptr || generated_port_b == nullptr) {
    return false;
  }
  const bool generated_pole_is_start = generated_port_a->owner_pole_id == generated_pole_id;
  const wire::core::Port* generated_port = generated_pole_is_start ? generated_port_a : generated_port_b;
  if (generated_port->owner_pole_id != generated_pole_id) {
    return false;
  }
  const wire::core::Vec3d generated_endpoint =
      generated_pole_is_start ? generated_layout.entry->start.endpoint_world : generated_layout.entry->end.endpoint_world;
  const wire::core::Span* untilted_span = untilted_state.view().spans().find(untilted_span_id);
  if (untilted_span == nullptr) {
    return false;
  }
  const wire::core::Port* untilted_port_a = untilted_state.view().ports().find(untilted_span->port_a_id);
  const bool untilted_pole_is_start = untilted_port_a != nullptr && untilted_port_a->owner_pole_id == untilted_pole_id;
  const wire::core::Vec3d untilted_endpoint =
      untilted_pole_is_start ? untilted_layout.entry->start.endpoint_world : untilted_layout.entry->end.endpoint_world;
  if (!almost_equal(generated_endpoint, generated_port->world_position, 1e-9) ||
      almost_equal(generated_endpoint, untilted_endpoint, 1e-6)) {
    return false;
  }

  wire::core::CoreState apply_state;
  const auto apply_generated = apply_state.GenerateFromBackboneSpec(line_req(apply_state));
  if (!apply_generated.ok || apply_generated.value.generated_pole_ids.empty()) {
    return false;
  }
  const auto applied = apply_state.ApplyPoleTilt({apply_generated.value.generated_pole_ids.front()}, 12.0);
  const wire::core::Pole* applied_pole = apply_state.view().poles().find(apply_generated.value.generated_pole_ids.front());
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
  wire::core::CoreState state;
  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  settings.sag_factor = 0.03;
  settings.curve_samples = 8;
  if (!state.UpdateGeometrySettings(settings).ok) {
    return false;
  }
  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId span_id = generated.value.generated_span_ids.front();
  const wire::core::CurveCacheEntry* formal = state.find_curve_cache(span_id);
  if (formal == nullptr) {
    return false;
  }
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != wire::core::VisualCurvePartKind::kEdgeBody || part.source_span_id != span_id) {
      continue;
    }
    if (part.sag_method != wire::core::VisualCurveSagMethod::kParabolic || part.sag_m <= 0.0 ||
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
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || generated.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId middle_pole_id = generated.value.generated_pole_ids[1];
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != wire::core::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    const bool attachment_is_owned_port =
        std::any_of(state.view().edit_state().ports.items().begin(), state.view().edit_state().ports.items().end(),
                    [&](const wire::core::Port& port) {
                      return port.owner_pole_id == middle_pole_id &&
                             almost_equal(port.world_position, part.attachment_point, 1e-9);
                    });
    const bool samples_attachment =
        std::any_of(part.samples.begin(), part.samples.end(),
                    [&](const wire::core::Vec3d& point) { return almost_equal(point, part.attachment_point, 1e-9); });
    return part.has_attachment_point && !part.passes_attachment_point && part.section_count == 1 &&
           attachment_is_owned_port && !samples_attachment &&
           !almost_equal(part.attachment_point, part.boundary_a, 1e-9) &&
           !almost_equal(part.attachment_point, part.boundary_b, 1e-9);
  }
  return false;
}

bool C644_backbone_patch_boundaries_extend_sag_tangents_below_attachment() {
  wire::core::CoreState state;
  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  settings.sag_factor = 0.03;
  if (!state.UpdateGeometrySettings(settings).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  for (const wire::core::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != wire::core::VisualCurvePartKind::kNodePatch || !patch.has_attachment_point) {
      continue;
    }
    const wire::core::Vec3d attachment_to_a{patch.boundary_a.x - patch.attachment_point.x,
                                            patch.boundary_a.y - patch.attachment_point.y,
                                            patch.boundary_a.z - patch.attachment_point.z};
    const wire::core::Vec3d attachment_to_b{patch.boundary_b.x - patch.attachment_point.x,
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
  wire::core::CoreState state;
  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  if (!state.UpdateGeometrySettings(settings).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  for (const wire::core::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != wire::core::VisualCurvePartKind::kNodePatch || patch.section_count != 1 ||
        patch.bezier_control_points.size() != 4) {
      continue;
    }
    return distance_to_chord(patch.bezier_control_points[1], patch.boundary_a, patch.boundary_b) > 1e-4 &&
           distance_to_chord(patch.bezier_control_points[2], patch.boundary_a, patch.boundary_b) > 1e-4;
  }
  return false;
}

bool C646_backbone_node_patch_turns_monotonically_inside_corner() {
  wire::core::CoreState state;
  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  if (!state.UpdateGeometrySettings(settings).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  for (const wire::core::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != wire::core::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    const bool samples_inside =
        std::all_of(patch.samples.begin(), patch.samples.end(), [&](const wire::core::Vec3d& point) {
          return point_in_xy_triangle(point, patch.boundary_a, patch.attachment_point, patch.boundary_b);
        });
    return samples_inside && sampled_xy_turn_is_monotonic(patch);
  }
  return false;
}

bool C647_backbone_node_patch_uses_incident_cable_appearance() {
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  for (const wire::core::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != wire::core::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    std::size_t matching_bodies = 0;
    for (const wire::core::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != wire::core::VisualCurvePartKind::kEdgeBody ||
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
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "curve_parts.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  std::string key_body;
  if (!function_body(cpp, "bool same_key(const curve_patch_key& a, const curve_patch_key& b)", &key_body)) {
    return false;
  }
  return contains_text(cpp, "int band_id = 0") && contains_text(cpp, "PoleTypeId pole_type_id") &&
         contains_text(cpp, "placement_band_id") && !contains_text(cpp, "band_id_for_port") &&
         contains_text(key_body, "a.band_id == b.band_id") &&
         contains_text(key_body, "a.pole_type_id == b.pole_type_id");
}

bool C656_backbone_node_patch_does_not_mix_base_and_extra_sections() {
  wire::core::CoreState state;
  if (!enable_two_extra_lv_population(state)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  bool saw_patch = false;
  for (const wire::core::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != wire::core::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    saw_patch = true;
    bool have_reference = false;
    wire::core::CableSectionKey reference{};
    for (const wire::core::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != wire::core::VisualCurvePartKind::kEdgeBody || !body.has_section_key ||
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
  wire::core::CoreState state;
  if (!enable_two_extra_lv_population(state)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  bool saw_extra_body = false;
  for (const wire::core::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
    saw_extra_body = saw_extra_body || (body.kind == wire::core::VisualCurvePartKind::kEdgeBody &&
                                        body.has_section_key && !body.section_key.is_base());
  }
  if (!saw_extra_body) {
    return false;
  }
  bool saw_extra_patch = false;
  for (const wire::core::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != wire::core::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    std::size_t extra_instance_index = 0;
    std::size_t matching_extra_bodies = 0;
    bool saw_extra = false;
    for (const wire::core::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != wire::core::VisualCurvePartKind::kEdgeBody || !body.has_section_key ||
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
  wire::core::CoreState state;
  wire::core::GeometrySettings geometry = state.view().geometry_settings();
  geometry.sag_enabled = true;
  if (!state.UpdateGeometrySettings(geometry).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.size() != 1 ||
      state.view().backbone().edges.size() != 1) {
    return false;
  }
  const wire::core::ObjectId span_id = generated.value.generated_span_ids.front();
  const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  const wire::core::SavedBackboneEdge& edge = state.view().backbone().edges.front();
  const auto attachment =
      state.view().source_edge_projection_world(edge.edge_id, edge.node_a,
                                                wire::core::kDefaultLowVoltageBundleTemplateId, 0, 0.5);
  if (curve == nullptr || !attachment.has_value()) {
    return false;
  }
  const wire::core::Vec3d expected = curve->detail.EvaluatePosition(0.5);
  const wire::core::Span* span = state.view().spans().find(span_id);
  const wire::core::Port* a = span == nullptr ? nullptr : state.view().ports().find(span->port_a_id);
  const wire::core::Port* b = span == nullptr ? nullptr : state.view().ports().find(span->port_b_id);
  if (a == nullptr || b == nullptr) {
    return false;
  }
  const wire::core::Vec3d chord_mid{
      (a->world_position.x + b->world_position.x) * 0.5,
      (a->world_position.y + b->world_position.y) * 0.5,
      (a->world_position.z + b->world_position.z) * 0.5,
  };
  return almost_equal(*attachment, expected, 1e-9) && attachment->z < chord_mid.z - 1e-4;
}

bool C666_backbone_terminal_extension_creates_connectivity_patch() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const auto endpoint_it = std::max_element(
      first.value.generated_pole_ids.begin(), first.value.generated_pole_ids.end(),
      [&](wire::core::ObjectId a, wire::core::ObjectId b) {
        return state.view().poles().find(a)->world_transform.position.x <
               state.view().poles().find(b)->world_transform.position.x;
      });
  const wire::core::ObjectId endpoint_pole_id = *endpoint_it;
  const wire::core::Pole* endpoint_pole = state.view().poles().find(endpoint_pole_id);
  const wire::core::SavedBackboneNode* endpoint_node =
      state.view().backbone_node_for_pole(endpoint_pole_id);
  if (endpoint_pole == nullptr || endpoint_node == nullptr) {
    return false;
  }
  const wire::core::ObjectId endpoint_node_id = endpoint_node->node_id;
  wire::core::BackboneSpec extension = line_req(state);
  extension.path.polyline = {endpoint_pole->world_transform.position, {24.0, 6.0, 0.0}};
  extension.path.node_specs = {pole_spec(0, endpoint_pole_id)};
  const auto second = state.GenerateFromBackboneSpec(extension);
  if (!second.ok) {
    return false;
  }
  const bool found = std::any_of(
      state.view().visual_curve_parts().parts.begin(), state.view().visual_curve_parts().parts.end(),
      [&](const wire::core::VisualCurvePart& part) {
        return part.kind == wire::core::VisualCurvePartKind::kNodePatch &&
               part.source_node_id == endpoint_node_id && part.incident_edge_ids.size() == 2;
      });
  return found;
}

bool C667_backbone_branch_preserves_through_patch() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId branch_pole_id = first.value.generated_pole_ids[1];
  const wire::core::Pole* branch_pole = state.view().poles().find(branch_pole_id);
  const wire::core::SavedBackboneNode* branch_node =
      state.view().backbone_node_for_pole(branch_pole_id);
  if (branch_pole == nullptr || branch_node == nullptr) {
    return false;
  }
  const wire::core::ObjectId branch_node_id = branch_node->node_id;
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {branch_pole->world_transform.position, {20.0, -6.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, branch_pole_id)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok || state.view().pole_frontier(branch_pole_id).edge_ids.size() != 3) {
    return false;
  }
  const bool found = std::any_of(
      state.view().visual_curve_parts().parts.begin(), state.view().visual_curve_parts().parts.end(),
      [&](const wire::core::VisualCurvePart& part) {
        return part.kind == wire::core::VisualCurvePartKind::kNodePatch &&
               part.source_node_id == branch_node_id && part.incident_edge_ids.size() == 2;
      });
  return found;
}

bool C691_cable_run_id_connects_through_sections() {
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  const auto bodies = base_edge_bodies(state);
  if (bodies.size() != 2 || !all_same_nonzero_run(bodies)) {
    return false;
  }
  const wire::core::CableRunId run_id = bodies.front()->cable_run_id;
  for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == wire::core::VisualCurvePartKind::kNodePatch && part.cable_run_id != run_id) {
      return false;
    }
  }
  return visual_curve_part_count(state, wire::core::VisualCurvePartKind::kNodePatch) > 0;
}

bool C692_cable_run_id_connects_terminal_extension() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const auto endpoint_it = std::max_element(
      first.value.generated_pole_ids.begin(), first.value.generated_pole_ids.end(),
      [&](wire::core::ObjectId a, wire::core::ObjectId b) {
        return state.view().poles().find(a)->world_transform.position.x <
               state.view().poles().find(b)->world_transform.position.x;
      });
  const wire::core::ObjectId endpoint_pole_id = *endpoint_it;
  const wire::core::Pole* endpoint_pole = state.view().poles().find(endpoint_pole_id);
  if (endpoint_pole == nullptr) {
    return false;
  }
  wire::core::BackboneSpec extension = line_req(state);
  extension.path.polyline = {endpoint_pole->world_transform.position, {24.0, 6.0, 0.0}};
  extension.path.node_specs = {pole_spec(0, endpoint_pole_id)};
  const auto second = state.GenerateFromBackboneSpec(extension);
  const auto bodies = base_edge_bodies(state);
  return second.ok && bodies.size() >= 2 && all_same_nonzero_run(bodies);
}

bool C693_cable_run_id_keeps_branch_and_dead_end_separate() {
  wire::core::CoreState branch_state;
  const auto first = branch_state.GenerateFromBackboneSpec(poly3_req(branch_state));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const auto through_bodies = base_edge_bodies(branch_state);
  if (through_bodies.size() != 2 || !all_same_nonzero_run(through_bodies)) {
    return false;
  }
  const wire::core::CableRunId through_run_id = through_bodies.front()->cable_run_id;
  const wire::core::ObjectId branch_pole_id = first.value.generated_pole_ids[1];
  const wire::core::Pole* branch_pole = branch_state.view().poles().find(branch_pole_id);
  if (branch_pole == nullptr) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(branch_state);
  branch.path.polyline = {branch_pole->world_transform.position, {20.0, -6.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, branch_pole_id)};
  const auto second = branch_state.GenerateFromBackboneSpec(branch);
  if (!second.ok || second.value.generated_span_ids.empty()) {
    return false;
  }
  for (const wire::core::VisualCurvePart& part : branch_state.view().visual_curve_parts().parts) {
    if (part.kind == wire::core::VisualCurvePartKind::kEdgeBody &&
        contains_span(second.value.generated_span_ids, part.source_span_id) &&
        part.cable_run_id == through_run_id) {
      return false;
    }
  }

  wire::core::CoreState sharp_state;
  if (!prepare_two_lane_low_voltage_for_run_test(sharp_state)) {
    return false;
  }
  wire::core::BackboneSpec sharp = line_req(sharp_state);
  sharp.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {5.0, 8.660254037844386, 0.0}};
  const auto sharp_generated = sharp_state.GenerateFromBackboneSpec(sharp);
  const auto sharp_bodies = base_edge_bodies(sharp_state);
  return sharp_generated.ok && sharp_bodies.size() == 4 && all_distinct_nonzero_runs(sharp_bodies);
}

bool C694_cable_run_id_connects_population_instances() {
  wire::core::CoreState state;
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
  wire::core::CoreState a;
  wire::core::CoreState b;
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
  const auto collect = [](const wire::core::CoreState& state, std::vector<std::string>* out) {
    for (const wire::core::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
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
      repo_root() / "core" / "src" / "generation" / "backbone" / "curve_parts.cpp";
  const std::filesystem::path runtime_header =
      repo_root() / "core" / "include" / "wire" / "core" / "core_runtime_types.hpp";
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
  constexpr wire::core::ModelAssemblyTemplateId kPoleAssembly = 9101;
  constexpr wire::core::ModelAssemblyTemplateId kRowAssembly = 9102;
  constexpr wire::core::ModelAssemblyTemplateId kEndpointAssembly = 9103;

  wire::core::CoreState baseline;
  wire::core::BackboneSpec baseline_request = line_req(baseline);
  baseline_request.bundles.clear();
  add_backbone_bundle(baseline_request, wire::core::BundleKind::kHighVoltage);
  const auto baseline_generated = baseline.GenerateFromBackboneSpec(baseline_request);
  if (!baseline_generated.ok) return false;

  wire::core::CoreState state;
  wire::core::BackboneSpec request = line_req(state);
  request.bundles.clear();
  add_backbone_bundle(request, wire::core::BundleKind::kHighVoltage);

  wire::core::ModelAssemblyTemplate pole_assembly{};
  pole_assembly.id = kPoleAssembly;
  pole_assembly.parts.push_back({1, "pole_body", 3, {}, wire::core::ModelFitMode::kPoleHeight, {}});
  wire::core::ModelAssemblyTemplate row_assembly{};
  row_assembly.id = kRowAssembly;
  row_assembly.parts.push_back({1, "hv_crossarm", 5, {}, wire::core::ModelFitMode::kRigid, {}});
  wire::core::Transformd belt_transform{};
  belt_transform.scale = {4.0, 4.0, 1.0};
  row_assembly.parts.push_back(
      {2, "pole_belt", 7, belt_transform, wire::core::ModelFitMode::kPoleRadial, {}});
  wire::core::ModelAssemblyTemplate endpoint_assembly{};
  endpoint_assembly.id = kEndpointAssembly;
  wire::core::ModelAssemblyPart endpoint_part{};
  endpoint_part.part_id = 1;
  endpoint_part.model_key = "hv_insulator";
  endpoint_part.descriptor_version = 11;
  endpoint_part.sockets.push_back({"wire", {0.0, 0.0, -0.25}, {1.0, 0.0, 0.0}});
  endpoint_assembly.parts.push_back(endpoint_part);
  endpoint_assembly.wire_socket = wire::core::AssemblySocketRef{1, "wire"};
  if (!state.RegisterModelAssemblyTemplate(pole_assembly).ok ||
      !state.RegisterModelAssemblyTemplate(row_assembly).ok ||
      !state.RegisterModelAssemblyTemplate(endpoint_assembly).ok) {
    return false;
  }

  wire::core::PoleTypeDefinition pole_type = state.view().pole_types().at(request.pole_type_id);
  pole_type.pole_visual_assembly_id = kPoleAssembly;
  if (!state.UpdatePoleTypeDefinition(pole_type).ok) return false;
  const wire::core::BundleTemplateId hv_template_id =
      wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage);
  wire::core::BundleTemplate hv = state.view().bundle_templates().at(hv_template_id);
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

  std::unordered_set<wire::core::ObjectId> unique_ports{};
  for (wire::core::ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) return false;
    unique_ports.insert(span->port_a_id);
    unique_ports.insert(span->port_b_id);
    const wire::core::SpanLayoutEntry* layout = state.span_layout(span_id).entry;
    const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span_id);
    if (layout == nullptr || curve == nullptr || curve->detail.sample_points.size() < 2) return false;
    const auto endpoint_matches_socket = [&](const wire::core::LayoutEndpoint& endpoint) {
      const wire::core::Port* port = state.view().ports().find(endpoint.port_id);
      const wire::core::SavedBackbonePortBinding* binding =
          port == nullptr ? nullptr : state.view().backbone_port_binding_for_port(port->id);
      const wire::core::Pole* pole = port == nullptr ? nullptr : state.view().poles().find(port->owner_pole_id);
      if (port == nullptr || binding == nullptr || pole == nullptr) return false;
      const wire::core::PoleFrame frame =
          wire::core::BuildPoleFrame(pole->world_transform, binding->layout_yaw_deg);
      const wire::core::Vec3d expected = port->world_position + wire::core::ScaleVec(frame.up, -0.25);
      return almost_equal(endpoint.support_world, expected, 1e-9) &&
             almost_equal(endpoint.endpoint_world, expected, 1e-9);
    };
    if (!endpoint_matches_socket(layout->start) || !endpoint_matches_socket(layout->end) ||
        !almost_equal(curve->detail.sample_points.front(), layout->start.endpoint_world, 1e-9) ||
        !almost_equal(curve->detail.sample_points.back(), layout->end.endpoint_world, 1e-9)) {
      return false;
    }
  }
  if (unique_ports.size() != 6) return false;

  std::unordered_map<std::string, std::size_t> model_counts{};
  std::vector<std::string> stable_keys{};
  std::unordered_map<std::string, std::uint64_t> versions{};
  for (const wire::core::VisualModelInstance& instance : state.view().visual_model_instances().instances) {
    ++model_counts[instance.model_key];
    stable_keys.push_back(instance.stable_key);
    versions.emplace(instance.stable_key, instance.content_version);
  }
  std::sort(stable_keys.begin(), stable_keys.end());
  if (model_counts["pole_body"] != 2 || model_counts["hv_crossarm"] != 2 ||
      model_counts["pole_belt"] != 2 || model_counts["hv_insulator"] != unique_ports.size()) {
    return false;
  }

  const wire::core::ObjectId moved_pole_id = generated.value.generated_pole_ids.front();
  const wire::core::Pole* moved_pole = state.view().poles().find(moved_pole_id);
  if (moved_pole == nullptr) return false;
  wire::core::Transformd moved_transform = moved_pole->world_transform;
  moved_transform.position = moved_transform.position + wire::core::Vec3d{1.0, 2.0, 0.5};
  moved_transform.rotation_euler_deg = {7.0, -4.0, 13.0};
  if (!state.MovePole(moved_pole_id, moved_transform).ok) return false;
  std::vector<std::string> moved_keys{};
  bool moved_body_version_changed = false;
  bool moved_body_frame_matches = false;
  const std::string moved_body_prefix = "pole:" + std::to_string(moved_pole_id) + ":";
  const std::string moved_row_prefix = "row:" + std::to_string(moved_pole_id) + ":";
  std::unordered_set<wire::core::ObjectId> moved_port_ids{};
  for (wire::core::ObjectId port_id : unique_ports) {
    const wire::core::Port* port = state.view().ports().find(port_id);
    if (port != nullptr && port->owner_pole_id == moved_pole_id) moved_port_ids.insert(port_id);
  }
  std::size_t moved_endpoint_instances = 0;
  for (const wire::core::VisualModelInstance& instance : state.view().visual_model_instances().instances) {
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
    for (wire::core::ObjectId port_id : moved_port_ids) {
      if (instance.stable_key.rfind("port:" + std::to_string(port_id) + ":", 0) != 0) continue;
      const wire::core::Port* port = state.view().ports().find(port_id);
      if (port == nullptr || !almost_equal(instance.world_transform.position,
                                           port->world_position, 1e-9)) {
        return false;
      }
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
  const wire::core::Pole* moved_pole_after = state.view().poles().find(moved_pole_id);
  if (moved_pole_after == nullptr) return false;
  for (wire::core::ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::SpanLayoutEntry* layout = state.span_layout(span_id).entry;
    if (layout == nullptr) return false;
    for (const wire::core::LayoutEndpoint* endpoint : {&layout->start, &layout->end}) {
      if (!moved_port_ids.contains(endpoint->port_id)) continue;
      const wire::core::Port* port = state.view().ports().find(endpoint->port_id);
      const wire::core::SavedBackbonePortBinding* binding =
          port == nullptr ? nullptr : state.view().backbone_port_binding_for_port(port->id);
      if (port == nullptr || binding == nullptr) return false;
      const wire::core::PoleFrame frame =
          wire::core::BuildPoleFrame(moved_pole_after->world_transform, binding->layout_yaw_deg);
      const wire::core::Vec3d expected = port->world_position + wire::core::ScaleVec(frame.up, -0.25);
      if (!almost_equal(endpoint->support_world, expected, 1e-9) ||
          !almost_equal(endpoint->endpoint_world, expected, 1e-9)) {
        return false;
      }
    }
  }
  std::sort(moved_keys.begin(), moved_keys.end());
  return moved_keys == stable_keys && moved_body_version_changed && moved_body_frame_matches &&
         moved_endpoint_instances == moved_port_ids.size();
}

} // namespace backbone_tests
