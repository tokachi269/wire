#include "backbone/fixtures.hpp"
#include "registry.hpp"

#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

namespace persistence_tests {
namespace {

bool C750_authoritative_save_is_deterministic_and_changes_after_edit() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = backbone_tests::line_req(state);
  first.path.polyline = {{0.0, 0.0, 0.0}, {650.0, 0.0, 0.0}};
  first.interval_m = 10.0;
  const auto populated = state.GenerateFromBackboneSpec(first);
  if (!populated.ok || populated.value.generated_pole_ids.size() < 66) {
    return false;
  }

  std::string first_save{};
  std::string repeated_save{};
  const auto saved = state.SerializeAuthoritative(&first_save);
  const auto repeated = state.SerializeAuthoritative(&repeated_save);
  if (!saved.ok || !repeated.ok || first_save.empty() || first_save.rfind("wire_state_v1\n", 0) != 0 ||
      first_save != repeated_save) {
    return false;
  }

  wire::core::BackboneSpec second = backbone_tests::line_req(state);
  second.path.polyline = {{0.0, 100.0, 0.0}, {650.0, 100.0, 0.0}};
  second.interval_m = 10.0;
  const auto edited = state.GenerateFromBackboneSpec(second);
  if (!edited.ok || edited.value.generated_pole_ids.size() < 66) {
    return false;
  }
  std::string edited_save{};
  const auto saved_after_edit = state.SerializeAuthoritative(&edited_save);
  return saved_after_edit.ok && edited_save != first_save;
}

bool same_double(double a, double b) {
  return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

bool same_vec3(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  return same_double(a.x, b.x) && same_double(a.y, b.y) && same_double(a.z, b.z);
}

bool same_aabb(const wire::core::AABBd& a, const wire::core::AABBd& b) {
  return same_vec3(a.min, b.min) && same_vec3(a.max, b.max);
}

bool same_layout_endpoint(const wire::core::LayoutEndpoint& a, const wire::core::LayoutEndpoint& b) {
  return a.owner_pole_id == b.owner_pole_id && a.relation_kind == b.relation_kind &&
         a.continuity_class == b.continuity_class && a.in_through_pair == b.in_through_pair &&
         a.support_pair_peer_low == b.support_pair_peer_low && a.support_pair_peer_high == b.support_pair_peer_high &&
         a.support_group_id == b.support_group_id && a.lower_required == b.lower_required &&
         a.lowering_blocked_by_policy == b.lowering_blocked_by_policy &&
         a.side_assignment_rule == b.side_assignment_rule &&
         a.support_orientation_rule == b.support_orientation_rule &&
         a.support_orientation_basis == b.support_orientation_basis && a.has_side_axis == b.has_side_axis &&
         same_vec3(a.side_axis, b.side_axis) && same_double(a.chosen_side_sign, b.chosen_side_sign) &&
         a.endpoint_node_id == b.endpoint_node_id && a.port_id == b.port_id &&
         a.jumper_peer_port_id == b.jumper_peer_port_id &&
         a.source_projection.source_edge_id == b.source_projection.source_edge_id &&
         a.source_projection.from_node_id == b.source_projection.from_node_id &&
         a.source_projection.bundle_template_id == b.source_projection.bundle_template_id &&
         a.source_projection.lane_index == b.source_projection.lane_index &&
         same_double(a.source_projection.t, b.source_projection.t) &&
         a.resolved_socket_id == b.resolved_socket_id && a.flow_kind == b.flow_kind && a.origin == b.origin &&
         a.endpoint_source == b.endpoint_source && a.port_source == b.port_source && a.side == b.side &&
         a.endpoint_mode == b.endpoint_mode && a.has_visual_arm_geometry == b.has_visual_arm_geometry &&
         same_vec3(a.visual_arm_mount_world, b.visual_arm_mount_world) &&
         same_vec3(a.visual_arm_tip_world, b.visual_arm_tip_world) &&
         same_vec3(a.visual_insulator_base_world, b.visual_insulator_base_world) &&
         same_vec3(a.support_world, b.support_world) && same_vec3(a.endpoint_world, b.endpoint_world) &&
         same_vec3(a.departure_dir, b.departure_dir) && same_vec3(a.endpoint_offset, b.endpoint_offset) &&
         same_double(a.local_departure_length_m, b.local_departure_length_m) &&
         same_double(a.automatic_branch_down_offset_m, b.automatic_branch_down_offset_m) &&
         same_double(a.branch_down_offset_m, b.branch_down_offset_m) &&
         same_double(a.automatic_endpoint_offset_z_m, b.automatic_endpoint_offset_z_m) &&
         same_double(a.endpoint_offset_z_m, b.endpoint_offset_z_m) &&
         a.default_lower_required == b.default_lower_required && a.same_level_feasible == b.same_level_feasible &&
         a.unresolved_same_level_conflict == b.unresolved_same_level_conflict &&
         a.same_level_reason == b.same_level_reason &&
         same_double(a.projected_spacing_topview_m, b.projected_spacing_topview_m) &&
         same_double(a.required_clearance_m, b.required_clearance_m) &&
         a.solver_used_same_level_constraint == b.solver_used_same_level_constraint &&
         a.used_special_case_ports == b.used_special_case_ports &&
         a.order_decision_policy == b.order_decision_policy && a.order_decision_choice == b.order_decision_choice &&
         a.order_decision_choice_reason == b.order_decision_choice_reason && a.chosen_side == b.chosen_side &&
         a.used_junction_pair_side_assignment == b.used_junction_pair_side_assignment;
}

bool same_layout(const wire::core::SpanLayoutEntry& a, const wire::core::SpanLayoutEntry& b) {
  if (a.span_id != b.span_id || a.flow_kind != b.flow_kind || a.pass_mode != b.pass_mode ||
      a.detail_curve_profile_hint != b.detail_curve_profile_hint || !same_double(a.basis_length_m, b.basis_length_m) ||
      !same_double(a.effective_sag_ratio, b.effective_sag_ratio) ||
      a.continuity_preference != b.continuity_preference ||
      !same_double(a.bend_stiffness_hint, b.bend_stiffness_hint) ||
      !same_double(a.min_bend_radius_hint_m, b.min_bend_radius_hint_m) ||
      a.variation_flow_key != b.variation_flow_key || a.lowering_kind != b.lowering_kind ||
      !same_layout_endpoint(a.start, b.start) || !same_layout_endpoint(a.end, b.end) ||
      a.lowered_support_group_keys.size() != b.lowered_support_group_keys.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.lowered_support_group_keys.size(); ++i) {
    if (!(a.lowered_support_group_keys[i] == b.lowered_support_group_keys[i])) return false;
  }
  return true;
}

struct SpanDerivedSnapshot {
  wire::core::ObjectId span_id = wire::core::kInvalidObjectId;
  wire::core::SpanLayoutEntry layout{};
  std::vector<wire::core::Vec3d> curve_samples{};
  wire::core::AABBd bounds{};
  std::vector<wire::core::AABBd> bound_segments{};
};

struct DerivedSnapshot {
  std::vector<SpanDerivedSnapshot> spans{};
  wire::core::VisualCurvePartCache visual{};
};

bool snapshot_derived(const wire::core::CoreState& state, DerivedSnapshot* out) {
  if (out == nullptr) return false;
  std::vector<wire::core::ObjectId> span_ids{};
  for (const wire::core::Span& span : state.view().spans().items()) span_ids.push_back(span.id);
  std::sort(span_ids.begin(), span_ids.end());
  for (wire::core::ObjectId span_id : span_ids) {
    const auto layout = state.span_layout(span_id);
    const auto* curve = state.find_curve_cache(span_id);
    const auto* bounds = state.find_bounds_cache(span_id);
    if (!layout.has_layout() || curve == nullptr || bounds == nullptr) return false;
    out->spans.push_back({span_id, *layout.entry, curve->detail.sample_points, bounds->whole, bounds->segments});
  }
  out->visual = state.visual_curve_parts();
  return true;
}

bool same_visual_part(const wire::core::VisualCurvePart& a, const wire::core::VisualCurvePart& b) {
  if (a.kind != b.kind || a.node_patch_classification != b.node_patch_classification ||
      a.source_node_id != b.source_node_id || a.source_edge_id != b.source_edge_id ||
      a.source_span_id != b.source_span_id || a.source_bundle_id != b.source_bundle_id ||
      a.bundle_template_id != b.bundle_template_id || a.lane_index != b.lane_index ||
      a.incident_edge_ids != b.incident_edge_ids || !same_vec3(a.boundary_a, b.boundary_a) ||
      !same_vec3(a.boundary_b, b.boundary_b) || !same_vec3(a.tangent_a, b.tangent_a) ||
      !same_vec3(a.tangent_b, b.tangent_b) || !same_vec3(a.attachment_point, b.attachment_point) ||
      a.has_attachment_point != b.has_attachment_point || a.passes_attachment_point != b.passes_attachment_point ||
      a.has_explicit_attachment_orientation != b.has_explicit_attachment_orientation ||
      a.section_count != b.section_count || a.sag_method != b.sag_method || !same_double(a.sag_m, b.sag_m) ||
      a.has_section_key != b.has_section_key || a.cable_run_id != b.cable_run_id ||
      a.endpoint_a_pole_type_id != b.endpoint_a_pole_type_id ||
      a.endpoint_b_pole_type_id != b.endpoint_b_pole_type_id || a.endpoint_a_band_id != b.endpoint_a_band_id ||
      a.endpoint_b_band_id != b.endpoint_b_band_id || !same_double(a.wire_radius_m, b.wire_radius_m) ||
      a.color_rgba != b.color_rgba || a.material_style != b.material_style ||
      a.bezier_control_points.size() != b.bezier_control_points.size() || a.samples.size() != b.samples.size() ||
      !same_aabb(a.bounds, b.bounds)) {
    return false;
  }
  for (std::size_t i = 0; i < a.bezier_control_points.size(); ++i) {
    if (!same_vec3(a.bezier_control_points[i], b.bezier_control_points[i])) return false;
  }
  for (std::size_t i = 0; i < a.samples.size(); ++i) {
    if (!same_vec3(a.samples[i], b.samples[i])) return false;
  }
  return true;
}

bool same_derived(const DerivedSnapshot& a, const DerivedSnapshot& b) {
  if (a.spans.size() != b.spans.size() || a.visual.parts.size() != b.visual.parts.size()) return false;
  for (std::size_t i = 0; i < a.spans.size(); ++i) {
    if (a.spans[i].span_id != b.spans[i].span_id || !same_layout(a.spans[i].layout, b.spans[i].layout) ||
        a.spans[i].curve_samples.size() != b.spans[i].curve_samples.size() ||
        !same_aabb(a.spans[i].bounds, b.spans[i].bounds) ||
        a.spans[i].bound_segments.size() != b.spans[i].bound_segments.size()) return false;
    for (std::size_t j = 0; j < a.spans[i].curve_samples.size(); ++j) {
      if (!same_vec3(a.spans[i].curve_samples[j], b.spans[i].curve_samples[j])) return false;
    }
    for (std::size_t j = 0; j < a.spans[i].bound_segments.size(); ++j) {
      if (!same_aabb(a.spans[i].bound_segments[j], b.spans[i].bound_segments[j])) return false;
    }
  }
  for (std::size_t i = 0; i < a.visual.parts.size(); ++i) {
    if (!same_visual_part(a.visual.parts[i], b.visual.parts[i])) return false;
  }
  return true;
}

bool make_roundtrip_source(wire::core::CoreState* state, std::string* saved, DerivedSnapshot* snapshot) {
  wire::core::GeometrySettings geometry = state->view().geometry_settings();
  geometry.curve_samples = 12;
  geometry.sag_enabled = true;
  geometry.sag_factor = 0.041;
  if (!state->UpdateGeometrySettings(geometry).ok) return false;
  wire::core::VisualSettings visual = state->view().visual_settings();
  visual.support_arm_radius_m = 0.037;
  if (!state->UpdateVisualSettings(visual).ok) return false;
  const auto generated = state->GenerateFromBackboneSpec(backbone_tests::poly3_req(*state));
  return generated.ok && state->SerializeAuthoritative(saved).ok && snapshot_derived(*state, snapshot);
}

bool C751_authoritative_load_roundtrip_rederives_bit_exact_outputs() {
  wire::core::CoreState source;
  std::string saved{};
  DerivedSnapshot before{};
  if (!make_roundtrip_source(&source, &saved, &before)) return false;
  wire::core::CoreState loaded;
  const auto result = loaded.DeserializeAuthoritative(saved);
  if (!result.ok) {
    return false;
  }
  DerivedSnapshot after{};
  return snapshot_derived(loaded, &after) && same_derived(before, after);
}

bool C752_authoritative_load_resave_is_byte_identical() {
  wire::core::CoreState source;
  std::string saved{};
  DerivedSnapshot ignored{};
  if (!make_roundtrip_source(&source, &saved, &ignored)) return false;
  wire::core::CoreState loaded;
  std::string resaved{};
  return loaded.DeserializeAuthoritative(saved).ok && loaded.SerializeAuthoritative(&resaved).ok && resaved == saved;
}

bool C753_authoritative_load_continues_editing_without_id_collision() {
  wire::core::CoreState source;
  const auto made = source.GenerateFromBackboneSpec(backbone_tests::line_req(source));
  std::string saved{};
  if (!made.ok || made.value.generated_pole_ids.empty() || !source.SerializeAuthoritative(&saved).ok) return false;
  const wire::core::ObjectId next_id = source.next_id();
  wire::core::CoreState loaded;
  if (!loaded.DeserializeAuthoritative(saved).ok) return false;
  const auto* end_pole = loaded.view().poles().find(made.value.generated_pole_ids.back());
  if (end_pole == nullptr) return false;
  wire::core::BackboneSpec extension = backbone_tests::line_req(loaded);
  extension.path.polyline = {end_pole->world_transform.position,
                             {end_pole->world_transform.position.x + 12.0,
                              end_pole->world_transform.position.y + 8.0, 0.0}};
  extension.path.node_specs.push_back(backbone_tests::pole_spec(0, end_pole->id));
  const auto extended = loaded.GenerateFromBackboneSpec(extension);
  if (!extended.ok) return false;
  std::unordered_set<wire::core::ObjectId> ids{};
  for (const wire::core::Pole& pole : loaded.view().poles().items()) {
    if (!ids.insert(pole.id).second) return false;
  }
  wire::core::CableTemplate cable = loaded.view().cable_templates().begin()->second;
  cable.outer_diameter_m += 0.001;
  return loaded.next_id() > next_id && loaded.UpdateCableTemplate(cable).ok;
}

bool C754_authoritative_load_rejects_invalid_text_without_mutation() {
  wire::core::CoreState state;
  if (!state.GenerateFromBackboneSpec(backbone_tests::line_req(state)).ok) return false;
  std::string saved{};
  if (!state.SerializeAuthoritative(&saved).ok) return false;
  wire::core::CoreState valid_probe;
  if (!valid_probe.DeserializeAuthoritative(saved).ok) return false;
  const std::vector<std::string> invalid = {
      "wire_state_v2\n",
      saved + "unknown.key=1\n",
      saved.substr(0, saved.find_last_of('\n', saved.size() - 2) + 1)};
  for (const std::string& text : invalid) {
    const auto loaded = state.DeserializeAuthoritative(text);
    std::string after{};
    if (loaded.ok || !state.SerializeAuthoritative(&after).ok || after != saved) return false;
  }
  return true;
}

void register_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C750_authoritative_save_is_deterministic_and_changes_after_edit",
                         "authoritative save is versioned, deterministic, and changes after state edit",
                         "Invariant", false, C750_authoritative_save_is_deterministic_and_changes_after_edit);
  test_registry::AddTest(tests, "C751_authoritative_load_roundtrip_rederives_bit_exact_outputs",
                         "authoritative load rederives bit-exact layout, curve, bounds, visual, and run ids",
                         "Invariant", false, C751_authoritative_load_roundtrip_rederives_bit_exact_outputs);
  test_registry::AddTest(tests, "C752_authoritative_load_resave_is_byte_identical",
                         "authoritative load resaves to identical bytes", "Exact", false,
                         C752_authoritative_load_resave_is_byte_identical);
  test_registry::AddTest(tests, "C753_authoritative_load_continues_editing_without_id_collision",
                         "loaded authoritative state continues generation and template editing without id collision",
                         "Invariant", false, C753_authoritative_load_continues_editing_without_id_collision);
  test_registry::AddTest(tests, "C754_authoritative_load_rejects_invalid_text_without_mutation",
                         "authoritative load rejects version, unknown-key, and truncation errors without mutation",
                         "Boundary", true, C754_authoritative_load_rejects_invalid_text_without_mutation);
}

WIRE_REGISTER_TEST_SUITE(register_tests);

} // namespace
} // namespace persistence_tests
