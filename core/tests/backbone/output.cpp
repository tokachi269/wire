#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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
         contains_text(h, "rules make(const topo& made, const groups& placement) const") &&
         !contains_text(h, "rules make(const topo& made, const intent& intents) const") &&
         contains_text(cpp, "EditResult<groups> placement = make(ps.value, intents.value)");
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

bool same_visual_member_family(const wire::core::SpanMemberKey& a, const wire::core::SpanMemberKey& b) {
  return a.is_base() == b.is_base() && a.rule_owner_id == b.rule_owner_id &&
         a.rule_id == b.rule_id && a.instance_index == b.instance_index;
}

wire::core::ExperimentalSpanMemberPopulationConfig two_extra_lv_population_config() {
  wire::core::ExperimentalSpanMemberPopulationConfig config{};
  config.enabled = true;
  config.explicit_seed = 1234;
  wire::core::ExperimentalSpanMemberRule rule{};
  rule.rule_id = 11;
  rule.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  rule.priority = 10;
  rule.min_extra_count = 2;
  rule.max_extra_count = 2;
  rule.min_spacing_m = 0.01;
  rule.lateral_min_m = -2.0;
  rule.lateral_max_m = 2.0;
  rule.height_min_m = 0.0;
  rule.height_max_m = 20.0;
  rule.randomness = 0.6;
  config.rules.push_back(rule);
  return config;
}

} // namespace

bool C634_backbone_terminal_nodes_create_no_node_patch_curve() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  return out.ok && visual_curve_part_count(state, wire::core::VisualCurvePartKind::kNodePatch) == 0 &&
         visual_curve_part_count(state, wire::core::VisualCurvePartKind::kEdgeBody) ==
             out.value.generated_span_ids.size();
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
    for (const wire::core::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != wire::core::VisualCurvePartKind::kEdgeBody) {
        continue;
      }
      if (almost_equal(body.boundary_a, patch.boundary_a, 1e-9)) {
        a_ok = a_ok || sampled_boundary_is_g1(body, patch, patch.boundary_a);
      }
      if (almost_equal(body.boundary_b, patch.boundary_a, 1e-9)) {
        a_ok = a_ok || sampled_boundary_is_g1(body, patch, patch.boundary_a);
      }
      if (almost_equal(body.boundary_a, patch.boundary_b, 1e-9)) {
        b_ok = b_ok || sampled_boundary_is_g1(body, patch, patch.boundary_b);
      }
      if (almost_equal(body.boundary_b, patch.boundary_b, 1e-9)) {
        b_ok = b_ok || sampled_boundary_is_g1(body, patch, patch.boundary_b);
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
         contains_text(key_body, "a.band_id == b.band_id") &&
         contains_text(key_body, "a.pole_type_id == b.pole_type_id");
}

bool C656_backbone_node_patch_does_not_mix_base_and_extra_members() {
  wire::core::CoreState state;
  if (!state.UpdateExperimentalSpanMemberPopulationConfig(two_extra_lv_population_config()).ok) {
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
    wire::core::SpanMemberKey reference{};
    for (const wire::core::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != wire::core::VisualCurvePartKind::kEdgeBody || !body.has_span_member_key ||
          !touches_patch_boundary(body, patch)) {
        continue;
      }
      if (!have_reference) {
        reference = body.span_member_key;
        have_reference = true;
      } else if (!same_visual_member_family(reference, body.span_member_key)) {
        return false;
      }
    }
  }
  return saw_patch;
}

bool C657_backbone_node_patch_does_not_mix_extra_instance_indices() {
  wire::core::CoreState state;
  if (!state.UpdateExperimentalSpanMemberPopulationConfig(two_extra_lv_population_config()).ok) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok) {
    return false;
  }
  bool saw_extra_body = false;
  for (const wire::core::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
    saw_extra_body = saw_extra_body || (body.kind == wire::core::VisualCurvePartKind::kEdgeBody &&
                                        body.has_span_member_key && !body.span_member_key.is_base());
  }
  if (!saw_extra_body) {
    return false;
  }
  for (const wire::core::VisualCurvePart& patch : state.view().visual_curve_parts().parts) {
    if (patch.kind != wire::core::VisualCurvePartKind::kNodePatch) {
      continue;
    }
    std::size_t extra_instance_index = 0;
    bool saw_extra = false;
    for (const wire::core::VisualCurvePart& body : state.view().visual_curve_parts().parts) {
      if (body.kind != wire::core::VisualCurvePartKind::kEdgeBody || !body.has_span_member_key ||
          body.span_member_key.is_base() || !touches_patch_boundary(body, patch)) {
        continue;
      }
      if (!saw_extra) {
        extra_instance_index = body.span_member_key.instance_index;
        saw_extra = true;
      } else if (extra_instance_index != body.span_member_key.instance_index) {
        return false;
      }
    }
    // The current experimental population is span-local. If an extra member endpoint does not
    // resolve to the exact same support position across adjacent spans, continuity is unproven,
    // so no node patch should connect that extra member.
    if (saw_extra) {
      return false;
    }
  }
  return true;
}

} // namespace backbone_tests
