#include "backbone/fixtures.hpp"
#include "registry.hpp"

#include "wire/core/core_state.hpp"
#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/model_descriptor.hpp"

#include <algorithm>
#include <bit>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <vector>

namespace persistence_tests {
namespace {

struct DerivedSnapshot;
bool make_roundtrip_source(wire::core::CoreState* state, std::string* saved, DerivedSnapshot* snapshot);

bool C750_authoritative_save_is_deterministic_and_changes_after_edit() {
  wire::core::CoreState state;
  std::string first_save{};
  std::string repeated_save{};
  if (!make_roundtrip_source(&state, &first_save, nullptr)) return false;
  const auto saved = state.SerializeAuthoritative(&repeated_save);
  if (!saved.ok || first_save.empty() || first_save.rfind("wire_state_v1\n", 0) != 0 ||
      first_save != repeated_save) {
    return false;
  }
  if (state.view().poles().items().empty()) return false;
  const wire::core::Pole& pole = state.view().poles().items().front();
  const auto edited = state.AddPort(pole.id, pole.world_transform.position + wire::core::Vec3d{0.3, 0.2, 2.0});
  if (!edited.ok) return false;
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
      a.visual.stats.sections != b.visual.stats.sections) return false;
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
  for (std::size_t i = 0; i < a.visual.diagnostics.size(); ++i) {
    const auto& x = a.visual.diagnostics[i];
    const auto& y = b.visual.diagnostics[i];
    if (x.source_node_id != y.source_node_id || x.source_span_id != y.source_span_id ||
        x.bundle_template_id != y.bundle_template_id || x.lane_index != y.lane_index || x.reason != y.reason) return false;
  }
  for (std::size_t i = 0; i < a.visual.population_diagnostics.size(); ++i) {
    const auto& x = a.visual.population_diagnostics[i];
    const auto& y = b.visual.population_diagnostics[i];
    if (x.logical_span_id != y.logical_span_id || x.edge_bundle_id != y.edge_bundle_id ||
        x.rule_id != y.rule_id || x.extra_count_requested != y.extra_count_requested ||
        x.extra_count_accepted != y.extra_count_accepted || x.omitted_count != y.omitted_count ||
        x.reason != y.reason) return false;
  }
  return true;
}

wire::core::AttachmentTemplateId replace_attachment_template_id(const wire::core::CoreState& state) {
  for (const auto& [id, value] : state.view().attachment_templates()) {
    if (value.line_interaction_mode == wire::core::AttachmentLineInteractionMode::kReplaceWithInternalPath &&
        !value.internal_paths.empty()) return id;
  }
  return wire::core::kInvalidAttachmentTemplateId;
}

wire::core::CablePopulationRule population_rule(std::uint64_t id) {
  wire::core::CablePopulationRule rule{};
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

bool full_fat_fixture_is_non_default(const wire::core::CoreState& state) {
  const auto& view = state.view();
  const auto lv_it = view.bundle_templates().find(wire::core::kDefaultLowVoltageBundleTemplateId);
  if (lv_it == view.bundle_templates().end() || lv_it->second.population_rules.size() < 2) return false;
  const auto comm_it = view.bundle_templates().find(
      wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication));
  const auto& overrides = wire::core::CoreStateTestHook::override_state(
      const_cast<wire::core::CoreState&>(state));
  const bool has_manual_port = std::any_of(view.ports().items().begin(), view.ports().items().end(),
      [](const wire::core::Port& port) { return !port.generated_from_template && !port.generated_by_rule; });
  const bool has_tilt = std::any_of(view.poles().items().begin(), view.poles().items().end(),
      [](const wire::core::Pole& pole) { return pole.tilt_magnitude_deg > 0.0; });
  const bool has_midair = std::any_of(view.backbone().nodes.begin(), view.backbone().nodes.end(),
      [](const wire::core::SavedBackboneNode& node) { return node.support_kind == wire::core::SupportKind::kMidair; });
  const bool has_offset_echo = std::any_of(view.backbone().edges.begin(), view.backbone().edges.end(),
      [](const wire::core::SavedBackboneEdge& edge) { return edge.lateral_offset_m == 0.35; });
  const bool has_route_echo = std::any_of(view.backbone().edges.begin(), view.backbone().edges.end(),
      [](const wire::core::SavedBackboneEdge& edge) { return edge.route != 0 || edge.order != 0; });
  return comm_it != view.bundle_templates().end() &&
         comm_it->second.count_rule == wire::core::BundleCountRuleKind::kRange &&
         view.context_profile().style_seed == 4242 && !view.layout_settings().angle_correction_enabled &&
         view.variation_settings().enabled && !overrides.pole_orientation_by_pole.empty() &&
         !overrides.span_endpoint_by_span.empty() && !overrides.span_support_by_span.empty() &&
         has_manual_port && has_tilt && has_midair && has_offset_echo && has_route_echo &&
         !view.attachments().empty();
}
bool make_roundtrip_source(wire::core::CoreState* state, std::string* saved, DerivedSnapshot* snapshot) {
  if (state == nullptr || saved == nullptr) return false;

  wire::core::ContextProfile context = state->view().context_profile();
  context.age = 0.21;
  context.clutter = 0.73;
  context.regularity = 0.38;
  context.service_mix = 0.84;
  context.style_seed = 4242;
  if (!state->UpdateContextProfile(context).ok) return false;
  wire::core::LayoutSettings layout = state->view().layout_settings();
  layout.angle_correction_enabled = false;
  layout.corner_threshold_deg = 63.0;
  layout.min_side_scale = 0.91;
  layout.max_side_scale = 1.42;
  if (!state->UpdateLayoutSettings(layout).ok) return false;
  wire::core::VariationSettings variation = state->view().variation_settings();
  variation.enabled = true;
  variation.global_seed = 987654;
  variation.world_cell_size_m = 31.0;
  variation.sag_variation_scale = 0.13;
  variation.branch_down_offset_variation_scale = 0.17;
  if (!state->UpdateVariationSettings(variation).ok) return false;
  wire::core::GeometrySettings geometry = state->view().geometry_settings();
  geometry.curve_samples = 12;
  geometry.sag_enabled = true;
  geometry.sag_factor = 0.041;
  geometry.pole_clearance_m = 0.19;
  if (!state->UpdateGeometrySettings(geometry).ok) return false;
  wire::core::VisualSettings visual = state->view().visual_settings();
  visual.support_arm_radius_m = 0.037;
  visual.insulator_length_m = 0.27;
  if (!state->UpdateVisualSettings(visual).ok) return false;

  const wire::core::AttachmentTemplateId attachment_template_id = replace_attachment_template_id(*state);
  if (attachment_template_id == wire::core::kInvalidAttachmentTemplateId) return false;
  wire::core::ModelDescriptor descriptor{};
  descriptor.measurement.name = "persistence_fixture_insulator";
  descriptor.measurement.version = 17;
  descriptor.measurement.replace_length_m = 0.34;
  descriptor.measurement.sockets = {
      {"line_in", wire::core::ModelSocketRole::kLineIn, {-0.17, 0.01, 0.0}, {-1.0, 0.0, 0.0}},
      {"line_out", wire::core::ModelSocketRole::kLineOut, {0.17, -0.01, 0.0}, {1.0, 0.0, 0.0}}};
  const auto built = wire::core::build_attachment_template(descriptor, attachment_template_id);
  if (!built.report.conflicts.empty() || !state->UpdateAttachmentTemplate(built.attachment_template).ok) return false;

  wire::core::BundleTemplate lv = state->view().bundle_templates().at(
      wire::core::kDefaultLowVoltageBundleTemplateId);
  lv.population_rules = {population_rule(31), population_rule(32)};
  if (!state->UpdateBundleTemplate(lv).ok) return false;
  wire::core::CableTemplate cable = state->view().cable_templates().at(lv.cable_template_id);
  cable.default_endpoint_attachment_template_id = attachment_template_id;
  cable.sag_factor = 0.029;
  if (!state->UpdateCableTemplate(cable).ok) return false;
  const auto comm_id = wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication);
  wire::core::BundleTemplate comm = state->view().bundle_templates().at(comm_id);
  comm.count_rule = wire::core::BundleCountRuleKind::kRange;
  comm.fixed_count = 0;
  comm.min_count = 1;
  comm.max_count = 4;
  comm.default_count = 2;
  if (!state->UpdateBundleTemplate(comm).ok) return false;

  wire::core::BackboneSpec sharp = backbone_tests::line_req(*state);
  sharp.path.polyline = {{0.0, -30.0, 0.0}, {10.0, -30.0, 0.0},
                         {5.0, -21.339745962155614, 0.0}};
  sharp.constraints.lateral_offset_m = 0.35;
  sharp.pole_placement.enable_tilt = true;
  sharp.pole_placement.max_tilt_deg = 9.0;
  const auto generated = state->GenerateFromBackboneSpec(sharp);
  if (!generated.ok || generated.value.generated_span_ids.empty() || generated.value.generated_pole_ids.empty()) return false;

  const wire::core::ObjectId first_pole_id = generated.value.generated_pole_ids.front();
  const wire::core::ObjectId first_span_id = generated.value.generated_span_ids.front();
  if (!state->SetPoleManualYawOverride(first_pole_id, 17.0).ok) return false;
  if (!state->SetSpanEndpointSocketOverride(first_span_id, true, 0).ok) return false;
  if (!state->SetSpanBranchDownOffsetOverride(first_span_id, 0.62).ok) return false;
  const wire::core::ObjectId branch_pole_id = generated.value.generated_pole_ids.back();
  const wire::core::Pole* branch_pole = state->view().poles().find(branch_pole_id);
  if (branch_pole == nullptr) return false;
  const wire::core::Vec3d branch_position = branch_pole->world_transform.position;
  wire::core::BackboneSpec midair_branch = backbone_tests::line_req(*state);
  midair_branch.path.polyline = {branch_position, branch_position + wire::core::Vec3d{4.0, 12.0, 6.0}};
  midair_branch.constraints.lateral_offset_m = 0.35;
  wire::core::BackboneInputSpec::NodeSpec existing_pole{};
  existing_pole.point_index = 0;
  existing_pole.support_kind = wire::core::SupportKind::kPole;
  existing_pole.node_id = branch_pole_id;
  wire::core::BackboneInputSpec::NodeSpec midair_node{};
  midair_node.point_index = 1;
  midair_node.support_kind = wire::core::SupportKind::kMidair;
  midair_branch.path.node_specs = {existing_pole, midair_node};
  if (!state->GenerateFromBackboneSpec(midair_branch).ok) return false;
  const wire::core::Pole* first_pole = state->view().poles().find(first_pole_id);
  if (first_pole == nullptr) return false;
  const wire::core::Vec3d manual_position = first_pole->world_transform.position + wire::core::Vec3d{0.3, -0.2, 3.4};
  if (!state->AddPort(first_pole_id, manual_position, wire::core::PortKind::kGeneric,
                      wire::core::PortLayer::kUnknown).ok) return false;
  const wire::core::AttachmentTemplate* attachment_template = state->find_attachment_template(attachment_template_id);
  if (attachment_template == nullptr ||
      !state->AddAttachment(first_span_id, 0.43, attachment_template->kind, 0.08, attachment_template_id).ok ||
      !state->DeriveGeneratedSpanOutputs(first_span_id).ok) return false;

  return full_fat_fixture_is_non_default(*state) && state->SerializeAuthoritative(saved).ok &&
         (snapshot == nullptr || snapshot_derived(*state, snapshot));
}

bool C751_authoritative_load_roundtrip_rederives_bit_exact_outputs() {
  wire::core::CoreState source;
  std::string saved{};
  if (!make_roundtrip_source(&source, &saved, nullptr)) return false;
  wire::core::CoreState first_load;
  if (!first_load.DeserializeAuthoritative(saved).ok) return false;
  DerivedSnapshot before{};
  std::string canonical{};
  if (!snapshot_derived(first_load, &before) || !first_load.SerializeAuthoritative(&canonical).ok) return false;
  wire::core::CoreState second_load;
  DerivedSnapshot after{};
  return second_load.DeserializeAuthoritative(canonical).ok && snapshot_derived(second_load, &after) &&
         same_derived(before, after);
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

bool C756_persistence_has_no_type_specific_write_read_wrappers() {
  const std::filesystem::path source = backbone_tests::repo_root() / "core" / "src" / "state" / "persistence.cpp";
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
  wire::core::CoreState source;
  std::string saved{};
  DerivedSnapshot ignored{};
  if (!make_roundtrip_source(&source, &saved, &ignored)) return false;
  wire::core::CoreState loaded;
  return loaded.DeserializeAuthoritative(saved).ok &&
         wire::core::CoreStateTestHook::authoritative_equals(source, loaded);
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
  test_registry::AddTest(tests, "C756_persistence_has_no_type_specific_write_read_wrappers",
                         "persistence derives write and read from type archive visitors",
                         "Boundary", false, C756_persistence_has_no_type_specific_write_read_wrappers);
  test_registry::AddTest(tests, "C757_authoritative_roundtrip_compares_fields_directly",
                         "authoritative roundtrip compares every archived field directly",
                         "Invariant", false, C757_authoritative_roundtrip_compares_fields_directly);
}

WIRE_REGISTER_TEST_SUITE(register_tests);

} // namespace
} // namespace persistence_tests
