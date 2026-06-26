#include "registry.hpp"

#include "../src/generation/support_policy.hpp"
#include "../src/state/internal_services.hpp"
#include "helpers.hpp"

#include <iostream>

namespace {

using helpers::contains_id;
using helpers::sorted_pole_type_ids;
using wire::core::AddConnectionByPoleOptions;
using wire::core::AttachmentLineInteractionMode;
using wire::core::BundleKind;
using wire::core::ChangeSet;
using wire::core::ConnectionCategory;
using wire::core::CoreState;
using wire::core::CoreStateTestHook;
using wire::core::ObjectId;
using wire::core::PlacementMode;
using wire::core::PoleKind;
using wire::core::Transformd;
using wire::core::Vec3d;

bool has_validation_issue(const wire::core::ValidationResult& validation, wire::core::ValidationSeverity severity,
                         const char* code) {
  return std::any_of(validation.issues.begin(), validation.issues.end(),
                     [&](const wire::core::ValidationIssue& issue) {
                       return issue.severity == severity && issue.code == code;
                     });
}

double template_layer_base_z_for_test(const wire::core::CoreState& state, ObjectId pole_id,
                                      wire::core::ConnectionCategory category) {
  const auto& view = state.view();
  const wire::core::Pole* pole = view.edit_state().poles.find(pole_id);
  if (pole == nullptr) {
    return 0.0;
  }
  const auto pole_type_it = view.pole_types().find(pole->pole_type_id);
  if (pole_type_it == view.pole_types().end()) {
    return std::max(0.5, pole->height_m * 0.8);
  }
  double best_z = -std::numeric_limits<double>::infinity();
  const int target_layer = wire::core::generation::detail::TemplateLayerForCategory(category);
  for (const wire::core::PortPlacementBand& band : pole_type_it->second.port_bands) {
    if (band.enabled && band.layer == target_layer) {
      best_z = std::max(best_z, band.height_max_m);
    }
  }
  if (!std::isfinite(best_z)) {
    for (const wire::core::PortPlacementBand& band : pole_type_it->second.port_bands) {
      if (band.enabled && band.category == category) {
        best_z = std::max(best_z, band.height_max_m);
      }
    }
  }
  return std::isfinite(best_z) ? best_z : std::max(0.5, pole->height_m * 0.8);
}

const wire::core::OverrideEntryView* find_override_entry(const wire::core::OverrideInspectionView& view,
                                                         const char* name) {
  const auto it = std::find_if(view.entries.begin(), view.entries.end(),
                               [&](const wire::core::OverrideEntryView& entry) { return entry.name == name; });
  return it == view.entries.end() ? nullptr : &(*it);
}

std::vector<ObjectId> collect_ids_from_ports(const std::vector<const wire::core::Port*>& ports) {
  std::vector<ObjectId> ids;
  ids.reserve(ports.size());
  for (const auto* port : ports) {
    if (port != nullptr) {
      ids.push_back(port->id);
    }
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

std::vector<ObjectId> collect_ids_from_anchors(const std::vector<const wire::core::Anchor*>& anchors) {
  std::vector<ObjectId> ids;
  ids.reserve(anchors.size());
  for (const auto* anchor : anchors) {
    if (anchor != nullptr) {
      ids.push_back(anchor->id);
    }
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

bool prepare_single_low_voltage_span(CoreState& state, ObjectId* span_id, wire::core::CableTemplateId* cable_template_id) {
  Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  Transformd b_tf{};
  b_tf.position = {12.0, 0.0, 0.0};
  const auto add_a = state.AddPole(a_tf, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto);
  const auto add_b = state.AddPole(b_tf, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto);
  if (!add_a.ok || !add_b.ok) {
    return false;
  }
  const ObjectId pole_a = add_a.value;
  const ObjectId pole_b = add_b.value;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }
  if (!state.ApplyPoleType(pole_a, pole_type_ids.front()).ok || !state.ApplyPoleType(pole_b, pole_type_ids.front()).ok) {
    return false;
  }

  auto connect =
      helpers::add_connection_by_category(state, pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage);
  if (!connect.ok) {
    return false;
  }
  const auto commit = state.Commit();
  if (!commit.validation.ok()) {
    return false;
  }
  const wire::core::Span* span = state.view().spans().find(connect.value.span_id);
  if (span == nullptr) {
    return false;
  }
  const wire::core::Bundle* bundle = state.view().bundles().find(span->bundle_id);
  if (bundle == nullptr) {
    return false;
  }
  const auto bundle_template_it = state.view().bundle_templates().find(bundle->bundle_template_id);
  if (bundle_template_it == state.view().bundle_templates().end()) {
    return false;
  }
  auto runtime_it = wire::core::CoreStateTestHook::span_runtime_states(state).find(span->id);
  if (runtime_it != wire::core::CoreStateTestHook::span_runtime_states(state).end()) {
    runtime_it->second.dirty_bits = wire::core::DirtyBits::kNone;
  }
  if (span_id != nullptr) {
    *span_id = span->id;
  }
  if (cable_template_id != nullptr) {
    *cable_template_id = bundle_template_it->second.cable_template_id;
  }
  return true;
}

bool test_get_pole_detail_exposes_only_owned_endpoints() {
  CoreState state;

  Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  Transformd b_tf{};
  b_tf.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;

  const ObjectId port_a = state.AddPort(pole_a, {0.0, 0.4, 6.0}).value;
  const ObjectId port_b = state.AddPort(pole_b, {10.0, 0.4, 6.0}).value;
  const ObjectId anchor_a = state.AddAnchor(pole_a, {0.0, -0.6, 1.0}).value;
  const ObjectId anchor_b = state.AddAnchor(pole_b, {10.0, -0.6, 1.0}).value;

  const auto detail_a = state.GetPoleDetail(pole_a);
  const auto detail_b = state.GetPoleDetail(pole_b);
  const auto ports_a = collect_ids_from_ports(detail_a.owned_ports);
  const auto ports_b = collect_ids_from_ports(detail_b.owned_ports);
  const auto anchors_a = collect_ids_from_anchors(detail_a.owned_anchors);
  const auto anchors_b = collect_ids_from_anchors(detail_b.owned_anchors);

  return detail_a.pole != nullptr && detail_b.pole != nullptr && contains_id(ports_a, port_a) &&
         !contains_id(ports_a, port_b) && contains_id(ports_b, port_b) && !contains_id(ports_b, port_a) &&
         contains_id(anchors_a, anchor_a) && !contains_id(anchors_a, anchor_b) &&
         contains_id(anchors_b, anchor_b) && !contains_id(anchors_b, anchor_a);
}

bool test_move_pole_updates_only_target_pole_owned_endpoints() {
  CoreState state;

  Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  Transformd b_tf{};
  b_tf.position = {15.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;

  const ObjectId port_a = state.AddPort(pole_a, {0.2, 0.5, 5.5}).value;
  const ObjectId anchor_a = state.AddAnchor(pole_a, {0.0, -0.5, 1.2}).value;
  const ObjectId port_b = state.AddPort(pole_b, {15.2, 0.5, 5.5}).value;
  const ObjectId anchor_b = state.AddAnchor(pole_b, {15.0, -0.5, 1.2}).value;

  const auto* old_port_a = state.view().ports().find(port_a);
  const auto* old_anchor_a = state.view().anchors().find(anchor_a);
  const auto* old_port_b = state.view().ports().find(port_b);
  const auto* old_anchor_b = state.view().anchors().find(anchor_b);
  if (old_port_a == nullptr || old_anchor_a == nullptr || old_port_b == nullptr || old_anchor_b == nullptr) {
    return false;
  }
  const Vec3d old_port_a_pos = old_port_a->world_position;
  const Vec3d old_anchor_a_pos = old_anchor_a->world_position;
  const Vec3d old_port_b_pos = old_port_b->world_position;
  const Vec3d old_anchor_b_pos = old_anchor_b->world_position;

  Transformd moved = a_tf;
  moved.position.x += 3.0;
  moved.position.y += 1.0;
  const auto move = state.MovePole(pole_a, moved);
  if (!move.ok) {
    return false;
  }

  const auto* new_port_a = state.view().ports().find(port_a);
  const auto* new_anchor_a = state.view().anchors().find(anchor_a);
  const auto* new_port_b = state.view().ports().find(port_b);
  const auto* new_anchor_b = state.view().anchors().find(anchor_b);
  if (new_port_a == nullptr || new_anchor_a == nullptr || new_port_b == nullptr || new_anchor_b == nullptr) {
    return false;
  }

  const bool a_moved = !helpers::almost_equal(old_port_a_pos, new_port_a->world_position) &&
                       !helpers::almost_equal(old_anchor_a_pos, new_anchor_a->world_position);
  const bool b_unchanged = helpers::almost_equal(old_port_b_pos, new_port_b->world_position) &&
                           helpers::almost_equal(old_anchor_b_pos, new_anchor_b->world_position);

  return a_moved && b_unchanged && contains_id(move.change_set.updated_ids, port_a) &&
         contains_id(move.change_set.updated_ids, anchor_a) && !contains_id(move.change_set.updated_ids, port_b) &&
         !contains_id(move.change_set.updated_ids, anchor_b);
}

bool test_public_override_surfaces_match_mutation_state() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  Transformd b_tf{};
  b_tf.position = {8.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;
  (void)state.ApplyPoleType(pole_a, pole_type_ids.front());
  (void)state.ApplyPoleType(pole_b, pole_type_ids.front());

  AddConnectionByPoleOptions options{};
  options.use_bundle_template = true;
  options.bundle_template_id = BundleKind::kLowVoltage;
  const auto connect = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage, options);
  if (!connect.ok) {
    return false;
  }
  const ObjectId span_id = connect.value.span_id;

  if (!state.SetPoleManualYawOverride(pole_a, 37.0).ok || !state.SetPoleFlip180(pole_a, true).ok ||
      !state.SetSpanEndpointSocketOverride(span_id, true, 3).ok ||
      !state.SetSpanBranchDownOffsetOverride(span_id, 0.42).ok) {
    return false;
  }

  const auto commit = state.Commit();
  if (!commit.validation.ok()) {
    return false;
  }

  const auto pole_view = state.view().inspect_pole(pole_a);
  const auto pole_overrides = state.view().inspect_overrides({wire::core::EntityKind::kPole, pole_a});
  const auto span_overrides = state.view().inspect_overrides({wire::core::EntityKind::kSpan, span_id});
  if (!pole_view.has_value() || !pole_overrides.has_value() || !span_overrides.has_value()) {
    return false;
  }

  const auto* manual_yaw = find_override_entry(*pole_overrides, "manualYaw");
  const auto* flip = find_override_entry(*pole_overrides, "flip180");
  const auto* socket_a = find_override_entry(*span_overrides, "endpointSocketA");
  const auto* branch_down = find_override_entry(*span_overrides, "branchDownOffset");
  if (manual_yaw == nullptr || flip == nullptr || socket_a == nullptr || branch_down == nullptr) {
    return false;
  }

  return pole_view->orientation_override && pole_view->manual_yaw_override_deg.has_value() &&
         helpers::almost_equal(*pole_view->manual_yaw_override_deg, 37.0) &&
         pole_view->flip_180_override.value_or(false) && manual_yaw->active && flip->active && socket_a->active &&
         socket_a->resolved_value == "3" && branch_down->active && branch_down->resolved_value == "0.420000";
}

bool test_apply_pole_type_reuses_only_target_pole_owned_endpoints() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  Transformd b_tf{};
  b_tf.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;

  if (!state.ApplyPoleType(pole_a, pole_type_ids.front()).ok || !state.ApplyPoleType(pole_b, pole_type_ids.front()).ok) {
    return false;
  }

  const auto detail_a_before = state.GetPoleDetail(pole_a);
  const auto detail_b_before = state.GetPoleDetail(pole_b);
  const auto ports_a_ids = collect_ids_from_ports(detail_a_before.owned_ports);
  const auto ports_b_ids = collect_ids_from_ports(detail_b_before.owned_ports);
  const auto anchors_a_ids = collect_ids_from_anchors(detail_a_before.owned_anchors);
  const auto anchors_b_ids = collect_ids_from_anchors(detail_b_before.owned_anchors);
  const std::size_t port_count_before = state.view().ports().items().size();
  const std::size_t anchor_count_before = state.view().anchors().items().size();

  if (!state.ApplyPoleType(pole_a, pole_type_ids.front()).ok) {
    return false;
  }

  const auto detail_a_after = state.GetPoleDetail(pole_a);
  const auto detail_b_after = state.GetPoleDetail(pole_b);

  return collect_ids_from_ports(detail_a_after.owned_ports) == ports_a_ids &&
         collect_ids_from_ports(detail_b_after.owned_ports) == ports_b_ids &&
         collect_ids_from_anchors(detail_a_after.owned_anchors) == anchors_a_ids &&
         collect_ids_from_anchors(detail_b_after.owned_anchors) == anchors_b_ids &&
         state.view().ports().items().size() == port_count_before &&
         state.view().anchors().items().size() == anchor_count_before && state.ValidateFast().ok();
}

bool test_template_mutation_service_marks_only_matching_bundle_regeneration() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  Transformd b_tf{};
  b_tf.position = {8.0, 0.0, 0.0};
  Transformd c_tf{};
  c_tf.position = {16.0, 0.0, 0.0};

  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_c = state.AddPole(c_tf, 10.0, "C", PoleKind::kGeneric, PlacementMode::kAuto).value;
  (void)state.ApplyPoleType(pole_a, pole_type_ids.front());
  (void)state.ApplyPoleType(pole_b, pole_type_ids.front());
  (void)state.ApplyPoleType(pole_c, pole_type_ids.front());

  AddConnectionByPoleOptions lv_options{};
  lv_options.use_bundle_template = true;
  lv_options.bundle_template_id = BundleKind::kLowVoltage;
  const auto lv_connect = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage, lv_options);
  if (!lv_connect.ok) {
    return false;
  }

  AddConnectionByPoleOptions comm_options{};
  comm_options.use_bundle_template = true;
  comm_options.bundle_template_id = BundleKind::kCommunication;
  const auto comm_connect = state.AddConnectionByPole(pole_b, pole_c, ConnectionCategory::kCommunication, comm_options);
  if (!comm_connect.ok) {
    return false;
  }

  const wire::core::Span* lv_span = state.view().spans().find(lv_connect.value.span_id);
  const wire::core::Span* comm_span = state.view().spans().find(comm_connect.value.span_id);
  if (lv_span == nullptr || comm_span == nullptr) {
    return false;
  }
  const wire::core::Bundle* lv_bundle = state.view().bundles().find(lv_span->bundle_id);
  const wire::core::Bundle* comm_bundle = state.view().bundles().find(comm_span->bundle_id);
  const wire::core::BundleTemplate* lv_template = state.view().bundle_templates().contains(BundleKind::kLowVoltage)
                                                      ? &state.view().bundle_templates().at(BundleKind::kLowVoltage)
                                                      : nullptr;
  if (lv_bundle == nullptr || comm_bundle == nullptr || lv_template == nullptr) {
    return false;
  }

  wire::core::BundleTemplate edited = *lv_template;
  edited.default_spacing_m += 0.1;
  const auto update = wire::core::state_internal::TemplateMutationService::UpdateBundleTemplate(state, edited);
  if (!update.ok || !update.value) {
    return false;
  }

  const wire::core::Bundle* lv_bundle_after = state.view().bundles().find(lv_bundle->id);
  const wire::core::Bundle* comm_bundle_after = state.view().bundles().find(comm_bundle->id);
  if (lv_bundle_after == nullptr || comm_bundle_after == nullptr) {
    return false;
  }

  return lv_bundle_after->regeneration_required && !comm_bundle_after->regeneration_required &&
         contains_id(update.change_set.updated_ids, lv_bundle_after->id) &&
         !contains_id(update.change_set.updated_ids, comm_bundle_after->id) &&
         contains_id(state.view().template_dependency_state().bundles_requiring_regeneration, lv_bundle_after->id) &&
         !contains_id(state.view().template_dependency_state().bundles_requiring_regeneration, comm_bundle_after->id);
}

bool test_template_mutation_service_marks_only_attached_span_dirty() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty() || state.view().attachment_templates().empty()) {
    return false;
  }

  Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  Transformd b_tf{};
  b_tf.position = {8.0, 0.0, 0.0};
  Transformd c_tf{};
  c_tf.position = {16.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_c = state.AddPole(c_tf, 10.0, "C", PoleKind::kGeneric, PlacementMode::kAuto).value;
  (void)state.ApplyPoleType(pole_a, pole_type_ids.front());
  (void)state.ApplyPoleType(pole_b, pole_type_ids.front());
  (void)state.ApplyPoleType(pole_c, pole_type_ids.front());

  AddConnectionByPoleOptions options{};
  options.use_bundle_template = true;
  options.bundle_template_id = BundleKind::kLowVoltage;
  const auto span_ab = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage, options);
  const auto span_bc = state.AddConnectionByPole(pole_b, pole_c, ConnectionCategory::kLowVoltage, options);
  if (!span_ab.ok || !span_bc.ok) {
    return false;
  }

  const wire::core::AttachmentTemplateId attachment_template_id = state.view().attachment_templates().begin()->first;
  const auto attach = state.AddAttachment(span_ab.value.span_id, 0.5, wire::core::AttachmentKind::kGeneric, 0.0,
                                          attachment_template_id);
  if (!attach.ok) {
    return false;
  }

  const wire::core::AttachmentTemplate* original = state.view().find_attachment_template(attachment_template_id);
  if (original == nullptr) {
    return false;
  }
  wire::core::AttachmentTemplate edited = *original;
  edited.line_interaction_mode = (edited.line_interaction_mode == AttachmentLineInteractionMode::kPassThrough)
                                     ? AttachmentLineInteractionMode::kHideSegment
                                     : AttachmentLineInteractionMode::kPassThrough;
  const auto update =
      wire::core::state_internal::TemplateMutationService::UpdateAttachmentTemplate(state, edited, true);
  if (!update.ok || !update.value) {
    return false;
  }

  return contains_id(update.change_set.dirty_span_ids, span_ab.value.span_id) &&
         !contains_id(update.change_set.dirty_span_ids, span_bc.value.span_id) &&
         contains_id(update.change_set.updated_ids, span_ab.value.span_id) &&
         !contains_id(update.change_set.updated_ids, span_bc.value.span_id);
}

bool test_update_bundle_template_treats_branch_down_offset_policy_as_topology_change() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  Transformd b_tf{};
  b_tf.position = {8.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;
  (void)state.ApplyPoleType(pole_a, pole_type_ids.front());
  (void)state.ApplyPoleType(pole_b, pole_type_ids.front());

  AddConnectionByPoleOptions hv_options{};
  hv_options.use_bundle_template = true;
  hv_options.bundle_template_id = BundleKind::kHighVoltage;
  const auto hv_connect = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kHighVoltage, hv_options);
  if (!hv_connect.ok) {
    return false;
  }

  const wire::core::Span* hv_span = state.view().spans().find(hv_connect.value.span_id);
  if (hv_span == nullptr) {
    return false;
  }
  const wire::core::Bundle* hv_bundle = state.view().bundles().find(hv_span->bundle_id);
  const wire::core::BundleTemplate* hv_template = state.view().bundle_templates().contains(BundleKind::kHighVoltage)
                                                      ? &state.view().bundle_templates().at(BundleKind::kHighVoltage)
                                                      : nullptr;
  if (hv_bundle == nullptr || hv_template == nullptr) {
    return false;
  }

  wire::core::BundleTemplate edited = *hv_template;
  edited.enable_branch_down_offset = !edited.enable_branch_down_offset;
  const auto update = state.UpdateBundleTemplate(edited);
  if (!update.ok || !update.value) {
    return false;
  }

  const wire::core::Bundle* hv_bundle_after = state.view().bundles().find(hv_bundle->id);
  return hv_bundle_after != nullptr && hv_bundle_after->regeneration_required &&
         contains_id(update.change_set.updated_ids, hv_bundle_after->id) &&
         contains_id(state.view().template_dependency_state().bundles_requiring_regeneration, hv_bundle_after->id) &&
         !contains_id(update.change_set.dirty_span_ids, hv_span->id);
}

bool test_materialization_reads_layout_owned_support_group_decision() {
  CoreState state;
  const ObjectId pole_a =
      state.AddPole(Transformd{{0.0, 0.0, 0.0}}, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b =
      state.AddPole(Transformd{{10.0, 0.0, 0.0}}, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId port_a =
      state.AddPort(pole_a, {0.0, 0.3, 5.0}, wire::core::PortKind::kPower, wire::core::PortLayer::kHighVoltage).value;
  const ObjectId port_b =
      state.AddPort(pole_b, {10.0, 0.3, 5.0}, wire::core::PortKind::kPower, wire::core::PortLayer::kHighVoltage).value;
  const ObjectId span =
      state.AddSpan(port_a, port_b, wire::core::SpanKind::kDistribution, wire::core::SpanLayer::kHighVoltage).value;
  (void)state.Commit();
  const double expected_support_z =
      template_layer_base_z_for_test(state, pole_a, wire::core::ConnectionCategory::kHighVoltage) - 1.25;

  wire::core::SpanSupportLayoutEntry layout{};
  if (const auto cached_projection = CoreStateTestHook::cache_state(state).span_layout_cache.projection_view(span);
      cached_projection.has_projection()) {
    layout = *cached_projection.layout;
  }
  layout.span_id = span;
  layout.start.owner_pole_id = pole_a;
  layout.start.port_id = port_a;
  layout.start.endpoint_world = {0.0, 0.5, expected_support_z};
  layout.start.owner_pole_id = pole_a;
  layout.start.relation_kind = wire::core::JunctionRelationKind::kSideBranch;
  layout.start.continuity_class = wire::core::ContinuityCategoryClass::kBundleLike;
  layout.start.lower_required = true;
  layout.start.default_lower_required = true;
  layout.start.same_level_feasible = false;
  layout.start.support_group_id = 777;
  layout.start.support_pair_peer_low = pole_a;
  layout.start.support_pair_peer_high = pole_b;
  layout.start.side_assignment_rule = wire::core::SideAssignmentRuleKind::kChord;
  layout.start.support_orientation_rule = wire::core::SupportOrientationRuleKind::kChord;
  layout.start.support_orientation_basis = wire::core::SupportOrientationBasisKind::kChordForward;
  layout.start.chosen_side = wire::core::LateralSideChoiceKind::kRight;
  layout.start.chosen_side_sign = 1.0;
  layout.start.has_side_axis = true;
  layout.start.side_axis = {1.0, 0.0, 0.0};
  layout.start.branch_down_offset_m = 9.0;
  layout.start.down_offset_variation = {};
  layout.start.support_world = layout.start.endpoint_world;

  wire::core::SupportGroupDecision authoritative{};
  authoritative.owner_pole_id = pole_a;
  authoritative.relation_kind = wire::core::JunctionRelationKind::kSideBranch;
  authoritative.continuity_class = wire::core::ContinuityCategoryClass::kBundleLike;
  authoritative.lower_required = true;
  authoritative.support_pair_peer_low = pole_a;
  authoritative.support_pair_peer_high = pole_b;
  authoritative.side_assignment_rule = wire::core::SideAssignmentRuleKind::kBisector;
  authoritative.support_orientation_rule = wire::core::SupportOrientationRuleKind::kBisector;
  authoritative.support_orientation_basis = wire::core::SupportOrientationBasisKind::kBisectorForward;
  authoritative.chosen_side = wire::core::LateralSideChoiceKind::kLeft;
  authoritative.chosen_side_sign = -1.0;
  authoritative.support_group_id = 777;
  authoritative.has_side_axis = true;
  authoritative.side_axis = {0.0, 1.0, 0.0};
  authoritative.side = wire::core::SlotSide::kLeft;
  authoritative.origin = wire::core::SupportLayoutOriginKind::kBranchSupport;
  CoreStateTestHook::cache_span_support_layout(state, std::move(layout));

  auto& cache = CoreStateTestHook::cache_state(state).span_layout_cache;
  cache.support_groups.authority.by_key[{pole_a, 777}] = authoritative;
  const auto decision_it = cache.support_groups.authority.by_key.find({pole_a, 777});
  const auto cached_projection = cache.projection_view(span);
  if (decision_it == cache.support_groups.authority.by_key.end() || !cached_projection.has_projection()) {
    return false;
  }

  const auto validation = helpers::validate_now(state);
  const auto refreshed_decision_it = cache.support_groups.authority.by_key.find({pole_a, 777});
  const auto refreshed_projection = cache.projection_view(span);
  if (refreshed_decision_it == cache.support_groups.authority.by_key.end() || !refreshed_projection.has_projection()) {
    return false;
  }
  const auto* refreshed_layout = refreshed_projection.layout;
  const bool ok_validation = validation.ok();
  const bool ok_decision_side =
      refreshed_decision_it->second.side_assignment_rule == wire::core::SideAssignmentRuleKind::kBisector;
  const bool ok_decision_orient =
      refreshed_decision_it->second.support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector;
  const bool ok_decision_basis =
      refreshed_decision_it->second.support_orientation_basis ==
      wire::core::SupportOrientationBasisKind::kBisectorForward;
  const bool ok_decision_sign = helpers::almost_equal(refreshed_decision_it->second.chosen_side_sign, -1.0);
  const bool ok_decision_axis =
      helpers::almost_equal(refreshed_decision_it->second.side_axis, Vec3d{0.0, 1.0, 0.0});
  const bool ok_keys = refreshed_layout->lowered_support_group_keys.size() == 1 &&
                       refreshed_layout->lowered_support_group_keys.front() ==
                           wire::core::LoweredSupportGroupKey{pole_a, 777};
  const bool ok_layout_side =
      refreshed_layout->start.side_assignment_rule == wire::core::SideAssignmentRuleKind::kBisector;
  const bool ok_layout_orient =
      refreshed_layout->start.support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector;
  const bool ok_layout_basis =
      refreshed_layout->start.support_orientation_basis ==
      wire::core::SupportOrientationBasisKind::kBisectorForward;
  const bool ok_layout_sign = helpers::almost_equal(refreshed_layout->start.chosen_side_sign, -1.0);
  const bool ok_layout_axis =
      helpers::almost_equal(refreshed_layout->start.side_axis, Vec3d{0.0, 1.0, 0.0});
  const bool ok_layout_down = helpers::almost_equal(refreshed_layout->start.branch_down_offset_m, 1.25);
  const auto placement_it = cache.support_groups.placement.by_key.find({pole_a, 777});
  const bool ok_group_down =
      placement_it != cache.support_groups.placement.by_key.end() &&
      helpers::almost_equal(placement_it->second.down_offset_m, 1.25);
  return ok_validation && ok_decision_side && ok_decision_orient && ok_decision_basis && ok_decision_sign &&
         ok_decision_axis && ok_group_down && ok_keys && ok_layout_side && ok_layout_orient && ok_layout_basis &&
         ok_layout_sign && ok_layout_axis && ok_layout_down;
}

bool test_validation_treats_grouped_endpoint_semantics_as_derived_copies() {
  CoreState state;
  const ObjectId pole_a =
      state.AddPole(Transformd{{0.0, 0.0, 0.0}}, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b =
      state.AddPole(Transformd{{10.0, 0.0, 0.0}}, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId port_a =
      state.AddPort(pole_a, {0.0, 0.3, 5.0}, wire::core::PortKind::kPower, wire::core::PortLayer::kHighVoltage).value;
  const ObjectId port_b =
      state.AddPort(pole_b, {10.0, 0.3, 5.0}, wire::core::PortKind::kPower, wire::core::PortLayer::kHighVoltage).value;
  const ObjectId span =
      state.AddSpan(port_a, port_b, wire::core::SpanKind::kDistribution, wire::core::SpanLayer::kHighVoltage).value;
  (void)state.Commit();
  const double expected_support_z =
      template_layer_base_z_for_test(state, pole_a, wire::core::ConnectionCategory::kHighVoltage) - 1.25;

  wire::core::SpanSupportLayoutEntry layout{};
  layout.span_id = span;
  layout.start.owner_pole_id = pole_a;
  layout.start.port_id = port_a;
  layout.start.endpoint_world = {0.0, 0.5, expected_support_z};
  layout.start.owner_pole_id = pole_a;
  layout.start.relation_kind = wire::core::JunctionRelationKind::kSideBranch;
  layout.start.continuity_class = wire::core::ContinuityCategoryClass::kBundleLike;
  layout.start.lower_required = true;
  layout.start.default_lower_required = true;
  layout.start.same_level_feasible = false;
  layout.start.support_group_id = 777;
  layout.start.support_pair_peer_low = pole_a;
  layout.start.support_pair_peer_high = pole_b;
  layout.start.side_assignment_rule = wire::core::SideAssignmentRuleKind::kBisector;
  layout.start.support_orientation_rule = wire::core::SupportOrientationRuleKind::kBisector;
  layout.start.support_orientation_basis = wire::core::SupportOrientationBasisKind::kBisectorForward;
  layout.start.chosen_side = wire::core::LateralSideChoiceKind::kLeft;
  layout.start.chosen_side_sign = -1.0;
  layout.start.has_side_axis = true;
  layout.start.side_axis = {0.0, 1.0, 0.0};
  layout.start.branch_down_offset_m = 1.25;
  layout.start.support_world = layout.start.endpoint_world;
  layout.end.owner_pole_id = pole_b;
  layout.end.port_id = port_b;
  layout.end.endpoint_world = {10.0, 0.5, expected_support_z};
  layout.end.support_world = layout.end.endpoint_world;

  wire::core::SupportGroupDecision authoritative{};
  static_cast<wire::core::SupportLayoutSemanticDecision&>(authoritative) = layout.start;
  authoritative.side = wire::core::SlotSide::kLeft;
  authoritative.origin = wire::core::SupportLayoutOriginKind::kBranchSupport;
  CoreStateTestHook::cache_span_support_layout(state, std::move(layout));
  CoreStateTestHook::cache_state(state).span_layout_cache.support_groups.authority.by_key[{pole_a, 777}] = authoritative;
  auto& cache = CoreStateTestHook::cache_state(state).span_layout_cache;
  auto projection = cache.edit_projection(span);
  if (!projection.has_projection()) {
    return false;
  }
  auto* cached_layout = projection.layout;
  cached_layout->start.side_assignment_rule = wire::core::SideAssignmentRuleKind::kChord;
  cached_layout->start.support_orientation_rule = wire::core::SupportOrientationRuleKind::kChord;
  cached_layout->start.support_orientation_basis = wire::core::SupportOrientationBasisKind::kChordForward;
  cached_layout->start.chosen_side = wire::core::LateralSideChoiceKind::kRight;
  cached_layout->start.chosen_side_sign = 1.0;
  cached_layout->start.side_axis = {1.0, 0.0, 0.0};

  const auto validation = helpers::validate_now(state);
  if (validation.ok()) {
    return false;
  }
  return std::any_of(validation.issues.begin(), validation.issues.end(), [](const wire::core::ValidationIssue& issue) {
    return issue.code == "SupportGroupEndpointSemanticProjectionMismatch";
  });
}

bool test_update_cable_template_marks_render_refresh_only_for_render_change() {
  CoreState state;
  ObjectId span_id = wire::core::kInvalidObjectId;
  wire::core::CableTemplateId cable_id = wire::core::kInvalidCableTemplateId;
  if (!prepare_single_low_voltage_span(state, &span_id, &cable_id)) {
    return false;
  }
  auto cable_it = state.view().cable_templates().find(cable_id);
  if (cable_it == state.view().cable_templates().end()) {
    return false;
  }
  wire::core::CableTemplate edited = cable_it->second;
  edited.color_rgba ^= 0x0000FF00u;
  const auto update = state.UpdateCableTemplate(edited);
  if (!update.ok) {
    return false;
  }
  const auto runtime_it = wire::core::CoreStateTestHook::span_runtime_states(state).find(span_id);
  if (runtime_it == wire::core::CoreStateTestHook::span_runtime_states(state).end()) {
    return false;
  }
  return contains_id(update.change_set.dirty_span_ids, span_id) &&
         helpers::has_dirty(&runtime_it->second, wire::core::DirtyBits::kRenderRefresh) &&
         !helpers::has_dirty(&runtime_it->second, wire::core::DirtyBits::kGeometryRefresh) &&
         !helpers::has_dirty(&runtime_it->second, wire::core::DirtyBits::kDecision);
}

bool test_update_cable_template_marks_geometry_refresh_only_for_geometry_change() {
  CoreState state;
  ObjectId span_id = wire::core::kInvalidObjectId;
  wire::core::CableTemplateId cable_id = wire::core::kInvalidCableTemplateId;
  if (!prepare_single_low_voltage_span(state, &span_id, &cable_id)) {
    return false;
  }
  auto cable_it = state.view().cable_templates().find(cable_id);
  if (cable_it == state.view().cable_templates().end()) {
    return false;
  }
  wire::core::CableTemplate edited = cable_it->second;
  edited.sag_factor += 0.01;
  const auto update = state.UpdateCableTemplate(edited);
  if (!update.ok) {
    return false;
  }
  const auto runtime_it = wire::core::CoreStateTestHook::span_runtime_states(state).find(span_id);
  if (runtime_it == wire::core::CoreStateTestHook::span_runtime_states(state).end()) {
    return false;
  }
  return contains_id(update.change_set.dirty_span_ids, span_id) &&
         helpers::has_dirty(&runtime_it->second, wire::core::DirtyBits::kGeometryRefresh) &&
         !helpers::has_dirty(&runtime_it->second, wire::core::DirtyBits::kRenderRefresh) &&
         !helpers::has_dirty(&runtime_it->second, wire::core::DirtyBits::kDecision);
}

bool test_update_cable_template_marks_decision_for_policy_change() {
  CoreState state;
  ObjectId span_id = wire::core::kInvalidObjectId;
  wire::core::CableTemplateId cable_id = wire::core::kInvalidCableTemplateId;
  if (!prepare_single_low_voltage_span(state, &span_id, &cable_id)) {
    return false;
  }
  auto cable_it = state.view().cable_templates().find(cable_id);
  if (cable_it == state.view().cable_templates().end()) {
    return false;
  }
  wire::core::CableTemplate edited = cable_it->second;
  edited.continuity_policy = (edited.continuity_policy == wire::core::CableContinuityPolicyHint::kPreferG1)
                                 ? wire::core::CableContinuityPolicyHint::kPreferG2
                                 : wire::core::CableContinuityPolicyHint::kPreferG1;
  const auto update = state.UpdateCableTemplate(edited);
  if (!update.ok) {
    return false;
  }
  const auto runtime_it = wire::core::CoreStateTestHook::span_runtime_states(state).find(span_id);
  if (runtime_it == wire::core::CoreStateTestHook::span_runtime_states(state).end()) {
    return false;
  }
  return contains_id(update.change_set.dirty_span_ids, span_id) &&
         helpers::has_dirty(&runtime_it->second, wire::core::DirtyBits::kDecision) &&
         !helpers::has_dirty(&runtime_it->second, wire::core::DirtyBits::kGeometryRefresh) &&
         !helpers::has_dirty(&runtime_it->second, wire::core::DirtyBits::kRenderRefresh);
}

void RegisterStateServiceTests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C181_CoreStateService_CollectOwnedEndpoints",
                         "GetPoleDetail exposes only the requested pole owned ports and anchors",
                         "Invariant", false, &test_get_pole_detail_exposes_only_owned_endpoints);
  test_registry::AddTest(tests, "C182_CoreStateService_RefreshOwnedEndpoints",
                         "MovePole updates only the target pole owned ports and anchors",
                         "Invariant", false, &test_move_pole_updates_only_target_pole_owned_endpoints);
  test_registry::AddTest(tests, "C183_CoreStateService_OverrideResolution",
                         "public override mutations are reflected coherently on pole/span inspection surfaces",
                         "Invariant", false, &test_public_override_surfaces_match_mutation_state);
  test_registry::AddTest(tests, "C184_CoreStateService_ApplyPoleType_ReusesOwnedEndpoints",
                         "template application reuses the target pole owned ports and anchors without touching other poles",
                         "Invariant", false, &test_apply_pole_type_reuses_only_target_pole_owned_endpoints);
  test_registry::AddTest(tests, "C186_CoreStateService_TemplateMutation_LocalizesBundleImpact",
                         "template mutation service marks only bundles matching the edited template for regeneration",
                         "Invariant", false, &test_template_mutation_service_marks_only_matching_bundle_regeneration);
  test_registry::AddTest(tests, "C187_CoreStateService_TemplateMutation_LocalizesAttachmentImpact",
                         "template mutation service dirties only spans that actually use the edited attachment template",
                         "Invariant", false, &test_template_mutation_service_marks_only_attached_span_dirty);
  test_registry::AddTest(tests, "C200_CoreStateService_TemplateMutation_BranchDownOffsetPolicyIsTopology",
                         "bundle template branch-down-offset policy changes force regeneration instead of being ignored or treated as visual-only",
                         "Invariant", false,
                         &test_update_bundle_template_treats_branch_down_offset_policy_as_topology_change);
  test_registry::AddTest(tests, "C280_CoreStateService_GroupMaterializationConsumesLayoutAuthority",
                         "grouped support materialization consumes cache-projected support-group decisions instead of reconstructing authority from endpoint copies",
                         "Invariant", false, &test_materialization_reads_layout_owned_support_group_decision);
  test_registry::AddTest(tests, "C281_Validation_GroupAuthorityBeatsEndpointSemanticCopies",
                         "validation rejects grouped endpoint semantic copies that diverge from support-group authority",
                         "Invariant", false, &test_validation_treats_grouped_endpoint_semantics_as_derived_copies);
  test_registry::AddTest(tests, "C355_CoreStateService_CableTemplateRenderChangeMarksRenderRefresh",
                         "render-only cable template changes dirty spans with RenderRefresh only",
                         "Invariant", false, &test_update_cable_template_marks_render_refresh_only_for_render_change);
  test_registry::AddTest(tests, "C356_CoreStateService_CableTemplateGeometryChangeMarksGeometryRefresh",
                         "geometry-only cable template changes dirty spans with GeometryRefresh only",
                         "Invariant", false, &test_update_cable_template_marks_geometry_refresh_only_for_geometry_change);
  test_registry::AddTest(tests, "C357_CoreStateService_CableTemplatePolicyChangeMarksDecision",
                         "decision-bearing cable template changes dirty spans with Decision only",
                         "Invariant", false, &test_update_cable_template_marks_decision_for_policy_change);
}

WIRE_REGISTER_TEST_SUITE(RegisterStateServiceTests);

} // namespace
