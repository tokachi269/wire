#include "backbone/fixtures.hpp"
#include "registry.hpp"

#include "city/wire/core_state.hpp"
#include "city/wire/core_test_hook.hpp"
#include "city/wire/core_view.hpp"
#include "city/wire/model_descriptor.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <filesystem>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

namespace persistence_tests {
namespace {

struct DerivedSnapshot;
bool make_roundtrip_source(city::wire::CoreState* state, std::string* saved, DerivedSnapshot* snapshot);

bool C750_authoritative_save_is_deterministic_and_changes_after_edit() {
  city::wire::CoreState state;
  std::string first_save{};
  std::string repeated_save{};
  if (!make_roundtrip_source(&state, &first_save, nullptr)) return false;
  const auto saved = state.SerializeAuthoritative(&repeated_save);
  if (!saved.ok || first_save.empty() || first_save.rfind("wire_state_v4\n", 0) != 0 ||
      first_save != repeated_save) {
    return false;
  }
  if (state.view().poles().items().empty()) return false;
  const city::wire::Pole& pole = state.view().poles().items().front();
  const auto edited = state.AddPort(pole.id, pole.world_transform.position + city::wire::Vec3d{0.3, 0.2, 2.0});
  if (!edited.ok) return false;
  std::string edited_save{};
  const auto saved_after_edit = state.SerializeAuthoritative(&edited_save);
  return saved_after_edit.ok && edited_save != first_save;
}

bool same_double(double a, double b) {
  return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b);
}

bool same_vec3(const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
  return same_double(a.x, b.x) && same_double(a.y, b.y) && same_double(a.z, b.z);
}

bool same_aabb(const city::wire::AABBd& a, const city::wire::AABBd& b) {
  return same_vec3(a.min, b.min) && same_vec3(a.max, b.max);
}

bool same_layout_endpoint(const city::wire::LayoutEndpoint& a, const city::wire::LayoutEndpoint& b) {
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
         a.source_projection.source_edge_id == b.source_projection.source_edge_id &&
         a.source_projection.source_edge_bundle_id == b.source_projection.source_edge_bundle_id &&
         a.source_projection.from_node_id == b.source_projection.from_node_id &&
         a.source_projection.bundle_template_id == b.source_projection.bundle_template_id &&
         a.source_projection.lane_index == b.source_projection.lane_index &&
         same_double(a.source_projection.t, b.source_projection.t) &&
         a.resolved_socket_id == b.resolved_socket_id && a.flow_kind == b.flow_kind && a.origin == b.origin &&
         a.endpoint_source == b.endpoint_source && a.port_source == b.port_source && a.side == b.side &&
         a.endpoint_mode == b.endpoint_mode &&
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

bool same_layout(const city::wire::SpanLayoutEntry& a, const city::wire::SpanLayoutEntry& b) {
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
  city::wire::ObjectId span_id = city::wire::kInvalidObjectId;
  city::wire::SpanLayoutEntry layout{};
  std::vector<city::wire::Vec3d> curve_samples{};
  city::wire::AABBd bounds{};
  std::vector<city::wire::AABBd> bound_segments{};
};

struct DerivedSnapshot {
  std::vector<SpanDerivedSnapshot> spans{};
  city::wire::VisualCurvePartCache visual{};
};

bool snapshot_derived(const city::wire::CoreState& state, DerivedSnapshot* out) {
  if (out == nullptr) return false;
  std::vector<city::wire::ObjectId> span_ids{};
  for (const city::wire::Span& span : state.view().spans().items()) span_ids.push_back(span.id);
  std::sort(span_ids.begin(), span_ids.end());
  for (city::wire::ObjectId span_id : span_ids) {
    const auto layout = state.span_layout(span_id);
    const auto* curve = state.find_curve_cache(span_id);
    const auto* bounds = state.find_bounds_cache(span_id);
    if (!layout.has_layout() || curve == nullptr || bounds == nullptr) return false;
    out->spans.push_back({span_id, *layout.entry, curve->detail.sample_points, bounds->whole, bounds->segments});
  }
  out->visual = state.visual_curve_parts();
  return true;
}

bool same_visual_part(const city::wire::VisualCurvePart& a, const city::wire::VisualCurvePart& b) {
  const bool same_section_key =
      a.section_key.logical_span_id == b.section_key.logical_span_id &&
      a.section_key.edge_bundle_id == b.section_key.edge_bundle_id &&
      a.section_key.rule_owner_id == b.section_key.rule_owner_id &&
      a.section_key.rule_id == b.section_key.rule_id &&
      a.section_key.instance_index == b.section_key.instance_index;
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
      a.has_section_key != b.has_section_key || !same_section_key || a.cable_run_id != b.cable_run_id ||
      a.endpoint_a_pole_type_id != b.endpoint_a_pole_type_id ||
      a.endpoint_b_pole_type_id != b.endpoint_b_pole_type_id || a.endpoint_a_band_id != b.endpoint_a_band_id ||
      a.endpoint_b_band_id != b.endpoint_b_band_id || !same_double(a.wire_radius_m, b.wire_radius_m) ||
      a.color_rgba != b.color_rgba || a.material_style != b.material_style ||
      a.bezier_control_points.size() != b.bezier_control_points.size() || a.samples.size() != b.samples.size() ||
      !same_aabb(a.bounds, b.bounds) || a.source_version != b.source_version) {
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
  if (a.spans.size() != b.spans.size() || a.visual.parts.size() != b.visual.parts.size() ||
      a.visual.diagnostics.size() != b.visual.diagnostics.size() ||
      a.visual.population_diagnostics.size() != b.visual.population_diagnostics.size() ||
      a.visual.stats.curve_builds != b.visual.stats.curve_builds ||
      a.visual.stats.sections != b.visual.stats.sections) {
    return false;
  }
  for (std::size_t i = 0; i < a.spans.size(); ++i) {
    if (a.spans[i].span_id != b.spans[i].span_id || !same_layout(a.spans[i].layout, b.spans[i].layout) ||
        a.spans[i].curve_samples.size() != b.spans[i].curve_samples.size() ||
        !same_aabb(a.spans[i].bounds, b.spans[i].bounds) ||
        a.spans[i].bound_segments.size() != b.spans[i].bound_segments.size()) {
      return false;
    }
    for (std::size_t j = 0; j < a.spans[i].curve_samples.size(); ++j) {
      if (!same_vec3(a.spans[i].curve_samples[j], b.spans[i].curve_samples[j])) {
        return false;
      }
    }
    for (std::size_t j = 0; j < a.spans[i].bound_segments.size(); ++j) {
      if (!same_aabb(a.spans[i].bound_segments[j], b.spans[i].bound_segments[j])) {
        return false;
      }
    }
  }
  for (std::size_t i = 0; i < a.visual.parts.size(); ++i) {
    if (!same_visual_part(a.visual.parts[i], b.visual.parts[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < a.visual.diagnostics.size(); ++i) {
    const auto& x = a.visual.diagnostics[i];
    const auto& y = b.visual.diagnostics[i];
    if (x.source_node_id != y.source_node_id || x.source_span_id != y.source_span_id ||
        x.bundle_template_id != y.bundle_template_id || x.lane_index != y.lane_index || x.reason != y.reason) {
      return false;
    }
  }
  for (std::size_t i = 0; i < a.visual.population_diagnostics.size(); ++i) {
    const auto& x = a.visual.population_diagnostics[i];
    const auto& y = b.visual.population_diagnostics[i];
    if (x.logical_span_id != y.logical_span_id || x.edge_bundle_id != y.edge_bundle_id ||
        x.rule_id != y.rule_id || x.extra_count_requested != y.extra_count_requested ||
        x.extra_count_accepted != y.extra_count_accepted || x.omitted_count != y.omitted_count ||
        x.reason != y.reason) {
      return false;
    }
  }
  return true;
}

city::wire::AttachmentTemplateId replace_attachment_template_id(const city::wire::CoreState& state) {
  for (const auto& [id, value] : state.view().attachment_templates()) {
    if (value.line_interaction_mode == city::wire::AttachmentLineInteractionMode::kReplaceWithInternalPath &&
        !value.internal_paths.empty()) return id;
  }
  return city::wire::kInvalidAttachmentTemplateId;
}

city::wire::CablePopulationRule population_rule(std::uint64_t id) {
  city::wire::CablePopulationRule rule{};
  rule.rule_id = id;
  rule.explicit_seed = 100 + id;
  rule.priority = static_cast<int>(10 + id);
  rule.min_extra_count = 1;
  rule.max_extra_count = 1;
  rule.min_spacing_m = 0.04;
  rule.lateral_min_m = -0.4;
  rule.lateral_max_m = 0.4;
  rule.height_min_m = 4.0;
  rule.height_max_m = 9.0;
  rule.randomness = 0.25;
  return rule;
}

bool full_fat_fixture_is_non_default(const city::wire::CoreState& state) {
  const auto& view = state.view();
  const auto lv_it = view.bundle_templates().find(city::wire::kDefaultLowVoltageBundleTemplateId);
  if (lv_it == view.bundle_templates().end() || lv_it->second.population_rules.size() < 2 ||
      !lv_it->second.span_visual_assembly.helix_enabled) return false;
  const auto comm_it = view.bundle_templates().find(
      city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kCommunication));
  const auto& overrides = city::wire::CoreStateTestHook::override_state(
      const_cast<city::wire::CoreState&>(state));
  const bool has_manual_port = std::any_of(view.ports().items().begin(), view.ports().items().end(),
      [](const city::wire::Port& port) { return !port.generated_from_template && !port.generated_by_rule; });
  const bool has_tilt = std::any_of(view.poles().items().begin(), view.poles().items().end(),
      [](const city::wire::Pole& pole) { return pole.tilt_magnitude_deg > 0.0; });
  const bool has_midair = std::any_of(view.backbone().nodes.begin(), view.backbone().nodes.end(),
      [](const city::wire::SavedBackboneNode& node) { return node.support_kind == city::wire::SupportKind::kMidair; });
  const bool has_offset_echo = std::any_of(view.backbone().edges.begin(), view.backbone().edges.end(),
      [](const city::wire::SavedBackboneEdge& edge) { return edge.lateral_offset_m == 0.35; });
  const bool has_route_echo = std::any_of(view.backbone().edges.begin(), view.backbone().edges.end(),
      [](const city::wire::SavedBackboneEdge& edge) { return edge.route != 0 || edge.order != 0; });
  return comm_it != view.bundle_templates().end() &&
         comm_it->second.count_rule == city::wire::BundleCountRuleKind::kRange &&
         view.context_profile().style_seed == 4242 && !view.layout_settings().angle_correction_enabled &&
         view.variation_settings().enabled && !overrides.pole_orientation_by_pole.empty() &&
         !overrides.span_endpoint_by_span.empty() && !overrides.span_support_by_span.empty() &&
         has_manual_port && has_tilt && has_midair && has_offset_echo && has_route_echo &&
         !view.attachments().empty();
}
bool make_roundtrip_source(city::wire::CoreState* state, std::string* saved, DerivedSnapshot* snapshot) {
  if (state == nullptr || saved == nullptr) return false;

  city::wire::ContextProfile context = state->view().context_profile();
  context.age = 0.21;
  context.clutter = 0.73;
  context.regularity = 0.38;
  context.service_mix = 0.84;
  context.style_seed = 4242;
  if (!state->UpdateContextProfile(context).ok) return false;
  city::wire::LayoutSettings layout = state->view().layout_settings();
  layout.angle_correction_enabled = false;
  layout.corner_threshold_deg = 63.0;
  layout.min_side_scale = 0.91;
  layout.max_side_scale = 1.42;
  if (!state->UpdateLayoutSettings(layout).ok) return false;
  city::wire::VariationSettings variation = state->view().variation_settings();
  variation.enabled = true;
  variation.global_seed = 987654;
  variation.world_cell_size_m = 31.0;
  variation.sag_variation_scale = 0.13;
  variation.branch_down_offset_variation_scale = 0.17;
  if (!state->UpdateVariationSettings(variation).ok) return false;
  city::wire::GeometrySettings geometry = state->view().geometry_settings();
  geometry.curve_samples = 12;
  geometry.sag_enabled = true;
  geometry.sag_factor = 0.041;
  geometry.pole_clearance_m = 0.19;
  if (!state->UpdateGeometrySettings(geometry).ok) return false;
  city::wire::VisualSettings visual = state->view().visual_settings();
  visual.insulator_length_m = 0.27;
  if (!state->UpdateVisualSettings(visual).ok) return false;

  const city::wire::AttachmentTemplateId attachment_template_id = replace_attachment_template_id(*state);
  if (attachment_template_id == city::wire::kInvalidAttachmentTemplateId) return false;
  city::wire::ModelDescriptor descriptor{};
  descriptor.measurement.name = "persistence_fixture_insulator";
  descriptor.measurement.version = 17;
  descriptor.measurement.replace_length_m = 0.34;
  descriptor.measurement.sockets = {
      {"line_in", city::wire::ModelSocketRole::kLineIn, {-0.17, 0.01, 0.0}, {-1.0, 0.0, 0.0}},
      {"line_out", city::wire::ModelSocketRole::kLineOut, {0.17, -0.01, 0.0}, {1.0, 0.0, 0.0}}};
  const auto built = city::wire::build_attachment_template(descriptor, attachment_template_id);
  if (!built.report.conflicts.empty() || !state->UpdateAttachmentTemplate(built.attachment_template).ok) return false;

  city::wire::BundleTemplate lv = state->view().bundle_templates().at(
      city::wire::kDefaultLowVoltageBundleTemplateId);
  lv.population_rules = {population_rule(31), population_rule(32)};
  lv.support_wire_pole_band_id = 100;
  lv.span_visual_assembly.support_path_enabled = true;
  lv.span_visual_assembly.helix_enabled = true;
  lv.span_visual_assembly.helix_turns_per_meter = 0.5;
  lv.span_visual_assembly.helix_samples_per_turn = 12;
  lv.span_visual_assembly.endpoint_trim_m = 0.2;
  lv.span_visual_assembly.visual_member_count_min = 2;
  lv.span_visual_assembly.visual_member_count_max = 3;
  lv.span_visual_assembly.visual_member_spacing_m = 0.04;
  lv.span_visual_assembly.center_wander_amplitude_m = 0.06;
  lv.span_visual_assembly.center_wander_wavelength_m = 14.0;
  lv.span_visual_assembly.member_wander_ratio = 0.6;
  lv.span_visual_assembly.member_wander_wavelength_m = 3.0;
  lv.span_visual_assembly.member_twist_turns_per_meter = 0.2;
  if (!state->UpdateBundleTemplate(lv).ok) return false;
  city::wire::CableTemplate cable = state->view().cable_templates().at(lv.cable_template_id);
  cable.default_endpoint_attachment_template_id = attachment_template_id;
  cable.sag_factor = 0.029;
  if (!state->UpdateCableTemplate(cable).ok) return false;
  const auto comm_id = city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kCommunication);
  city::wire::BundleTemplate comm = state->view().bundle_templates().at(comm_id);
  comm.count_rule = city::wire::BundleCountRuleKind::kRange;
  comm.fixed_count = 0;
  comm.min_count = 1;
  comm.max_count = 4;
  comm.default_count = 2;
  if (!state->UpdateBundleTemplate(comm).ok) return false;

  city::wire::BackboneSpec sharp = backbone_tests::line_req(*state);
  sharp.path.polyline = {{0.0, -30.0, 0.0}, {10.0, -30.0, 0.0},
                         {5.0, -21.339745962155614, 0.0}};
  sharp.constraints.lateral_offset_m = 0.35;
  sharp.pole_placement.enable_tilt = true;
  sharp.pole_placement.max_tilt_deg = 9.0;
  const auto generated = state->GenerateFromBackboneSpec(sharp);
  if (!generated.ok || generated.value.generated_span_ids.empty() || generated.value.generated_pole_ids.empty()) return false;

  const city::wire::ObjectId first_pole_id = generated.value.generated_pole_ids.front();
  const city::wire::ObjectId first_span_id = generated.value.generated_span_ids.front();
  if (!state->SetPoleManualYawOverride(first_pole_id, 17.0).ok) return false;
  if (!state->SetSpanEndpointSocketOverride(first_span_id, true, 0).ok) return false;
  if (!state->SetSpanBranchDownOffsetOverride(first_span_id, 0.62).ok) return false;
  const city::wire::ObjectId branch_pole_id = generated.value.generated_pole_ids.back();
  const city::wire::Pole* branch_pole = state->view().poles().find(branch_pole_id);
  if (branch_pole == nullptr) return false;
  const city::wire::Vec3d branch_position = branch_pole->world_transform.position;
  city::wire::BackboneSpec midair_branch = backbone_tests::line_req(*state);
  midair_branch.path.polyline = {branch_position, branch_position + city::wire::Vec3d{4.0, 12.0, 6.0}};
  midair_branch.constraints.lateral_offset_m = 0.35;
  city::wire::BackboneInputSpec::NodeSpec existing_pole{};
  existing_pole.point_index = 0;
  existing_pole.support_kind = city::wire::SupportKind::kPole;
  existing_pole.node_id = branch_pole_id;
  city::wire::BackboneInputSpec::NodeSpec midair_node{};
  midair_node.point_index = 1;
  midair_node.support_kind = city::wire::SupportKind::kMidair;
  midair_branch.path.node_specs = {existing_pole, midair_node};
  if (!state->GenerateFromBackboneSpec(midair_branch).ok) return false;
  const city::wire::Pole* first_pole = state->view().poles().find(first_pole_id);
  if (first_pole == nullptr) return false;
  const city::wire::Vec3d manual_position = first_pole->world_transform.position + city::wire::Vec3d{0.3, -0.2, 3.4};
  if (!state->AddPort(first_pole_id, manual_position, city::wire::PortKind::kGeneric,
                      city::wire::PortLayer::kUnknown).ok) return false;
  const city::wire::AttachmentTemplate* attachment_template = state->find_attachment_template(attachment_template_id);
  if (attachment_template == nullptr ||
      !state->AddAttachment(first_span_id, 0.43, attachment_template->kind, 0.08, attachment_template_id).ok ||
      !state->DeriveGeneratedSpanOutputs(first_span_id).ok) return false;

  return full_fat_fixture_is_non_default(*state) && state->SerializeAuthoritative(saved).ok &&
         (snapshot == nullptr || snapshot_derived(*state, snapshot));
}

bool C751_authoritative_load_roundtrip_rederives_bit_exact_outputs() {
  city::wire::CoreState source;
  std::string saved{};
  if (!make_roundtrip_source(&source, &saved, nullptr)) return false;
  city::wire::CoreState first_load;
  const auto first_loaded = first_load.DeserializeAuthoritative(saved);
  WIRE_TEST_EXPECT_PRESENCE(first_loaded.ok, first_loaded.error);
  DerivedSnapshot before{};
  std::string canonical{};
  if (!snapshot_derived(first_load, &before)) return false;
  if (!first_load.SerializeAuthoritative(&canonical).ok) return false;
  city::wire::CoreState second_load;
  DerivedSnapshot after{};
  const auto second_loaded = second_load.DeserializeAuthoritative(canonical);
  WIRE_TEST_EXPECT_PRESENCE(second_loaded.ok, second_loaded.error);
  if (!snapshot_derived(second_load, &after)) return false;
  WIRE_TEST_EXPECT_DIFFERENTIAL(same_derived(before, after),
                                "derived output changed across repeated load");
  return true;
}

bool C752_authoritative_load_resave_is_byte_identical() {
  city::wire::CoreState source;
  std::string saved{};
  DerivedSnapshot ignored{};
  if (!make_roundtrip_source(&source, &saved, &ignored)) return false;
  city::wire::CoreState loaded;
  std::string resaved{};
  return loaded.DeserializeAuthoritative(saved).ok &&
         loaded.SerializeAuthoritative(&resaved).ok && resaved == saved;
}

bool C753_authoritative_load_continues_editing_without_id_collision() {
  city::wire::CoreState source;
  const auto made = source.GenerateFromBackboneSpec(backbone_tests::line_req(source));
  std::string saved{};
  if (!made.ok || made.value.generated_pole_ids.empty() || !source.SerializeAuthoritative(&saved).ok) return false;
  const city::wire::ObjectId next_id = source.next_id();
  city::wire::CoreState loaded;
  if (!loaded.DeserializeAuthoritative(saved).ok) return false;
  const auto* end_pole = loaded.view().poles().find(made.value.generated_pole_ids.back());
  if (end_pole == nullptr) return false;
  city::wire::BackboneSpec extension = backbone_tests::line_req(loaded);
  extension.path.polyline = {end_pole->world_transform.position,
                             {end_pole->world_transform.position.x + 12.0,
                              end_pole->world_transform.position.y + 8.0, 0.0}};
  extension.path.node_specs.push_back(backbone_tests::pole_spec(0, end_pole->id));
  const auto extended = loaded.GenerateFromBackboneSpec(extension);
  WIRE_TEST_EXPECT_PRESENCE(extended.ok, extended.error);
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(loaded);
  std::unordered_set<city::wire::ObjectId> ids{};
  for (const city::wire::Pole& pole : loaded.view().poles().items()) {
    if (!ids.insert(pole.id).second) return false;
  }
  city::wire::CableTemplate cable = loaded.view().cable_templates().begin()->second;
  cable.outer_diameter_m += 0.001;
  return loaded.next_id() > next_id && loaded.UpdateCableTemplate(cable).ok;
}

bool C754_authoritative_load_rejects_invalid_text_without_mutation() {
  city::wire::CoreState state;
  if (!state.GenerateFromBackboneSpec(backbone_tests::line_req(state)).ok) return false;
  std::string saved{};
  if (!state.SerializeAuthoritative(&saved).ok) return false;
  city::wire::CoreState valid_probe;
  if (!valid_probe.DeserializeAuthoritative(saved).ok) return false;
  const std::vector<std::string> invalid = {
      "wire_state_v5\n",
      saved + "unknown.key=1\n",
      saved.substr(0, saved.find_last_of('\n', saved.size() - 2) + 1)};
  for (const std::string& text : invalid) {
    const auto loaded = state.DeserializeAuthoritative(text);
    std::string after{};
    if (loaded.ok || !state.SerializeAuthoritative(&after).ok || after != saved) return false;
  }
  return true;
}

bool C799_authoritative_v1_load_migrates_row_continuity_to_current() {
  std::string saved_v2{};
  if (!backbone_tests::file_text(
          backbone_tests::repo_root() / "domains" / "wire" / "tests" / "fixtures" /
              "legacy_shared_pair_v2.txt",
          &saved_v2) ||
      saved_v2.rfind("wire_state_v2\n", 0) != 0) {
    return false;
  }

  std::string legacy = "wire_state_v1\n" + saved_v2.substr(std::string("wire_state_v2\n").size());
  std::string stripped{};
  stripped.reserve(legacy.size());
  for (std::size_t line_begin = 0; line_begin < legacy.size();) {
    const std::size_t line_end = legacy.find('\n', line_begin);
    if (line_end == std::string::npos) return false;
    const std::string_view line(legacy.data() + line_begin, line_end - line_begin);
    if (!line.starts_with("authoritative.backbone.row_continuities.")) {
      stripped.append(line);
      stripped.push_back('\n');
    }
    line_begin = line_end + 1;
  }
  if (stripped.find("authoritative.backbone.row_continuities.count=") != std::string::npos) {
    return false;
  }

  city::wire::CoreState loaded;
  if (!loaded.DeserializeAuthoritative(stripped).ok) return false;
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(loaded);
  if (loaded.view().backbone().row_continuities.size() != 3) {
    return false;
  }
  std::string migrated_v4{};
  if (!loaded.SerializeAuthoritative(&migrated_v4).ok ||
      migrated_v4.rfind("wire_state_v4\n", 0) != 0 ||
      migrated_v4.find("authoritative.backbone.row_continuities.count=3\n") == std::string::npos) {
    return false;
  }
  city::wire::CoreState reloaded;
  if (!reloaded.DeserializeAuthoritative(migrated_v4).ok) return false;
  std::string resaved{};
  return reloaded.SerializeAuthoritative(&resaved).ok && resaved == migrated_v4;
}

bool C801_authoritative_v2_rejects_broken_row_continuity_reference() {
  city::wire::CoreState source;
  const auto generated = source.GenerateFromBackboneSpec(backbone_tests::hv_poly3_req(source));
  if (!generated.ok || generated.value.generated_pole_ids.size() != 3) return false;
  std::string saved{};
  if (!source.SerializeAuthoritative(&saved).ok ||
      saved.find("authoritative.backbone.row_continuities.count=3\n") == std::string::npos) {
    return false;
  }
  const std::string field = "authoritative.backbone.row_continuities.0.a.edge_bundle_id=";
  const std::size_t field_pos = saved.find(field);
  if (field_pos == std::string::npos) return false;
  const std::size_t value_begin = field_pos + field.size();
  const std::size_t value_end = saved.find('\n', value_begin);
  if (value_end == std::string::npos) return false;
  std::string broken = saved;
  broken.replace(value_begin, value_end - value_begin, "999999999999");

  city::wire::CoreState loaded;
  const auto out = loaded.DeserializeAuthoritative(broken);
  return !out.ok;
}

bool C806_authoritative_v2_does_not_persist_backbone_route_order() {
  city::wire::CoreState source;
  std::string saved{};
  DerivedSnapshot ignored{};
  if (!make_roundtrip_source(&source, &saved, &ignored)) return false;

  std::istringstream lines(saved);
  std::string line{};
  while (std::getline(lines, line)) {
    const bool backbone_edge =
        line.rfind("authoritative.backbone.edges.", 0) == 0 ||
        line.rfind("authoritative.backbone.edge_bundles.", 0) == 0;
    if (backbone_edge &&
        (line.find(".route=") != std::string::npos ||
         line.find(".order=") != std::string::npos)) {
      return false;
    }
  }
  return true;
}

bool C756_persistence_has_no_type_specific_write_read_wrappers() {
  const std::filesystem::path source = backbone_tests::repo_root() / "domains" / "wire" / "src" / "state" / "persistence.cpp";
  std::string text{};
  if (!backbone_tests::file_text(source, &text)) return false;
  return text.find("#define WRITE_") == std::string::npos &&
         text.find("#define READ_") == std::string::npos &&
         text.find("void write_pole(") == std::string::npos &&
         text.find("bool read_pole(") == std::string::npos &&
         text.find("void write_bundle_template(") == std::string::npos &&
         text.find("bool read_bundle_template(") == std::string::npos;
}

bool C757_authoritative_roundtrip_compares_fields_directly() {
  city::wire::CoreState source;
  std::string saved{};
  DerivedSnapshot ignored{};
  if (!make_roundtrip_source(&source, &saved, &ignored)) return false;
  city::wire::CoreState loaded;
  return loaded.DeserializeAuthoritative(saved).ok &&
         city::wire::CoreStateTestHook::authoritative_equals(source, loaded);
}

bool C768_legacy_state_preserves_implicit_helix_support() {
  city::wire::CoreState source;
  if (!source.GenerateFromBackboneSpec(backbone_tests::line_req(source)).ok) return false;
  std::string saved{};
  if (!source.SerializeAuthoritative(&saved).ok) return false;

  static constexpr std::string_view kSupportField =
      ".span_visual_assembly.support_path_enabled=";
  static constexpr std::string_view kBundlePrefix =
      "authoritative.edit_state.bundles.";
  static constexpr std::array<std::string_view, 5> kBundleFields = {
      ".spacing_override_m=", ".placement_explicit=", ".height_m=", ".lateral_m=", ".placement_key="};
  std::string legacy{};
  legacy.reserve(saved.size());
  std::size_t removed = 0;
  std::size_t line_begin = 0;
  while (line_begin < saved.size()) {
    const std::size_t line_end = saved.find('\n', line_begin);
    if (line_end == std::string::npos) return false;
    const std::string_view line(saved.data() + line_begin, line_end - line_begin);
    const bool remove = line.find(kSupportField) != std::string_view::npos ||
        (line.starts_with(kBundlePrefix) &&
         std::any_of(kBundleFields.begin(), kBundleFields.end(), [&](std::string_view field) {
           return line.find(field) != std::string_view::npos;
         }));
    if (!remove) {
      legacy.append(line);
      legacy.push_back('\n');
    } else {
      ++removed;
    }
    line_begin = line_end + 1;
  }
  if (removed == 0) return false;
  static constexpr std::array<std::string_view, 4> kLegacySupportArmFields = {
      "authoritative.visual_settings.enable_support_structures=1\n",
      "authoritative.visual_settings.support_center_threshold_m=0x1p+0\n",
      "authoritative.visual_settings.support_arm_extra_m=0x1p-1\n",
      "authoritative.visual_settings.support_arm_radius_m=0x1.47ae147ae147bp-7\n"};
  for (std::string_view field : kLegacySupportArmFields) {
    legacy.append(field);
  }

  city::wire::CoreState loaded;
  if (!loaded.DeserializeAuthoritative(legacy).ok) return false;
  for (const auto& [id, bundle] : loaded.view().bundle_templates()) {
    static_cast<void>(id);
    if (bundle.span_visual_assembly.support_path_enabled !=
        bundle.span_visual_assembly.helix_enabled) return false;
  }
  for (const city::wire::Bundle& bundle : loaded.view().bundles().items()) {
    if (bundle.spacing_override_m != 0.0 || bundle.placement_explicit ||
        bundle.height_m != 0.0 || bundle.lateral_m != 0.0 ||
        bundle.placement_key != 0) return false;
  }

  std::string migrated{};
  if (!loaded.SerializeAuthoritative(&migrated).ok) return false;
  return std::count(migrated.begin(), migrated.end(), '\n') ==
             std::count(legacy.begin(), legacy.end(), '\n') + static_cast<std::ptrdiff_t>(removed) -
                 static_cast<std::ptrdiff_t>(kLegacySupportArmFields.size()) &&
         migrated.find(kSupportField) != std::string::npos &&
         migrated.find("enable_support_structures") == std::string::npos &&
         migrated.find("support_center_threshold_m") == std::string::npos &&
         migrated.find("support_arm_extra_m") == std::string::npos &&
         migrated.find("support_arm_radius_m") == std::string::npos &&
         std::all_of(kBundleFields.begin(), kBundleFields.end(), [&](std::string_view field) {
           return migrated.find(kBundlePrefix) != std::string::npos &&
                  migrated.find(field) != std::string::npos;
         });
}

bool C763_model_assembly_registration_is_transactional_and_persistent() {
  city::wire::CoreState state;
  city::wire::ModelAssemblyTemplate assembly{};
  assembly.id = 9001;
  assembly.version = 4;
  city::wire::ModelAssemblyPart part{};
  part.part_id = 7;
  part.model_key = "hv_phase_1_disc_3";
  part.descriptor_version = 11;
  part.local_transform.position = {0.1, -0.2, 0.3};
  part.sockets.push_back({"wire", {0.35, 0.0, 0.0}, {1.0, 0.0, 0.0}});
  part.sockets.push_back({"mount", {0.0, 0.0, 0.1}, {0.0, 0.0, 1.0}});
  assembly.parts.push_back(part);
  assembly.wire_socket = city::wire::AssemblySocketRef{7, "wire"};
  assembly.endpoint_mount_socket = city::wire::AssemblySocketRef{7, "mount"};
  if (!state.RegisterModelAssemblyTemplate(assembly).ok) return false;

  std::string saved{};
  if (!state.SerializeAuthoritative(&saved).ok) return false;
  city::wire::CoreState loaded;
  if (!loaded.DeserializeAuthoritative(saved).ok ||
      loaded.view().model_assembly_templates().at(assembly.id) != assembly) {
    return false;
  }
  std::string legacy = saved;
  for (const std::string& field : {
           "authoritative.model_assembly_templates.9001.endpoint_mount_socket.has",
           "authoritative.model_assembly_templates.9001.endpoint_mount_socket.part_id",
           "authoritative.model_assembly_templates.9001.endpoint_mount_socket.socket_name"}) {
    const std::size_t begin = legacy.find(field + "=");
    if (begin == std::string::npos) return false;
    const std::size_t end = legacy.find('\n', begin);
    if (end == std::string::npos) return false;
    legacy.erase(begin, end - begin + 1);
  }
  city::wire::CoreState legacy_loaded;
  if (!legacy_loaded.DeserializeAuthoritative(legacy).ok ||
      legacy_loaded.view().model_assembly_templates().at(assembly.id)
          .endpoint_mount_socket.has_value()) return false;

  city::wire::ModelAssemblyTemplate invalid = assembly;
  invalid.id = 9002;
  invalid.wire_socket = city::wire::AssemblySocketRef{7, "missing"};
  const auto rejected = state.RegisterModelAssemblyTemplate(invalid);
  std::string after{};
  return !rejected.ok && rejected.error.find("ModelAssemblyWireSocketMissing") != std::string::npos &&
         state.SerializeAuthoritative(&after).ok && after == saved &&
         !state.RegisterModelAssemblyTemplate(assembly).ok;
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
  test_registry::AddTest(tests, "C799_authoritative_v1_load_migrates_row_continuity_to_current",
                         "authoritative v1 load migrates row continuity and resaves in the current format",
                         "Boundary", false,
                         C799_authoritative_v1_load_migrates_row_continuity_to_current);
  test_registry::AddTest(tests, "C801_authoritative_v2_rejects_broken_row_continuity_reference",
                         "authoritative v2 load rejects broken row continuity references",
                         "Boundary", true,
                         C801_authoritative_v2_rejects_broken_row_continuity_reference);
  test_registry::AddSourceGuardTest(tests, "C806_authoritative_v2_does_not_persist_backbone_route_order",
                         "authoritative v2 does not persist backbone route/order helpers",
                         "Boundary", false,
                         C806_authoritative_v2_does_not_persist_backbone_route_order);
  test_registry::AddSourceGuardTest(tests, "C756_persistence_has_no_type_specific_write_read_wrappers",
                         "persistence derives write and read from type archive visitors",
                         "Boundary", false, C756_persistence_has_no_type_specific_write_read_wrappers);
  test_registry::AddTest(tests, "C757_authoritative_roundtrip_compares_fields_directly",
                         "authoritative roundtrip compares every archived field directly",
                         "Invariant", false, C757_authoritative_roundtrip_compares_fields_directly);
  test_registry::AddTest(tests, "C768_legacy_state_preserves_implicit_helix_support",
                         "legacy authoritative states preserve implicit helix support and resave the explicit field",
                         "Boundary", true, C768_legacy_state_preserves_implicit_helix_support);
  test_registry::AddTest(tests, "C763_model_assembly_registration_is_transactional_and_persistent",
                         "model assembly registration validates before mutation and survives save/load",
                         "Boundary", true, C763_model_assembly_registration_is_transactional_and_persistent);
}

WIRE_REGISTER_TEST_SUITE(register_tests);

} // namespace
} // namespace persistence_tests
