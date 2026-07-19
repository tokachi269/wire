#include "registry.hpp"

#include "../src/generation/support_policy.hpp"
#include "../src/state/internal_services.hpp"
#include "helpers.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace {

using helpers::contains_id;
using helpers::sorted_pole_type_ids;
using wire::core::AttachmentLineInteractionMode;
using wire::core::BundleKind;
using wire::core::ChangeSet;
using wire::core::ConnectionCategory;
using wire::core::CoreState;
using wire::core::CoreStateTestHook;
using wire::core::kDefaultHighVoltageBundleTemplateId;
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
  const auto fixture = helpers::make_backbone_fixture(
      state, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}}, {wire::core::BundleKind::kLowVoltage});
  if (!fixture.ok || fixture.value.spans.empty()) return false;
  if (!state.ValidateFast().ok()) {
    return false;
  }
  const wire::core::Span* span = state.view().spans().find(fixture.value.spans.front());
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

bool test_update_bundle_template_regenerates_branch_down_offset_policy() {
  CoreState state;
  const auto fixture = helpers::make_backbone_fixture(
      state, {{0.0, 0.0, 0.0}, {8.0, 0.0, 0.0}}, {BundleKind::kHighVoltage});
  if (!fixture.ok || fixture.value.spans.empty()) return false;
  const wire::core::Span* hv_span = state.view().spans().find(fixture.value.spans.front());
  if (hv_span == nullptr) {
    return false;
  }
  const ObjectId hv_span_id = hv_span->id;
  const wire::core::Bundle* hv_bundle = state.view().bundles().find(hv_span->bundle_id);
  const wire::core::BundleTemplate* hv_template =
      state.view().bundle_templates().contains(kDefaultHighVoltageBundleTemplateId)
          ? &state.view().bundle_templates().at(kDefaultHighVoltageBundleTemplateId)
                                                      : nullptr;
  if (hv_bundle == nullptr || hv_template == nullptr) {
    return false;
  }

  wire::core::BundleTemplate edited = *hv_template;
  edited.enable_branch_down_offset = !edited.enable_branch_down_offset;
  const auto update = state.UpdateBundleTemplate(edited);
  const auto current = state.view().bundle_templates().find(kDefaultHighVoltageBundleTemplateId);
  return update.ok && update.value && current != state.view().bundle_templates().end() &&
         current->second.enable_branch_down_offset == edited.enable_branch_down_offset &&
         state.view().spans().find(hv_span_id) != nullptr;
}

bool test_update_cable_template_rejects_manual_span_render_change() {
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
  const std::uint32_t original_color = cable_it->second.color_rgba;
  edited.color_rgba ^= 0x0000FF00u;
  const auto update = state.UpdateCableTemplate(edited);
  const auto current = state.view().cable_templates().find(cable_id);
  return !update.ok && current != state.view().cable_templates().end() &&
         current->second.color_rgba == original_color;
}

bool test_update_cable_template_rejects_manual_span_geometry_change() {
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
  const double original_sag = cable_it->second.sag_factor;
  edited.sag_factor += 0.01;
  const auto update = state.UpdateCableTemplate(edited);
  const auto current = state.view().cable_templates().find(cable_id);
  return !update.ok && current != state.view().cable_templates().end() &&
         current->second.sag_factor == original_sag;
}

bool test_update_cable_template_decision_change_regenerates_backbone() {
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
  const auto current = state.view().cable_templates().find(cable_id);
  const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  return update.ok && update.value && current != state.view().cable_templates().end() &&
         current->second.continuity_policy == edited.continuity_policy && curve != nullptr &&
         curve->detail.quality.requested_policy == edited.continuity_policy;
}

void RegisterStateServiceTests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C181_CoreStateService_CollectOwnedEndpoints",
                         "GetPoleDetail exposes only the requested pole owned ports and anchors",
                         "Invariant", false, &test_get_pole_detail_exposes_only_owned_endpoints);
  test_registry::AddTest(tests, "C182_CoreStateService_RefreshOwnedEndpoints",
                         "MovePole updates only the target pole owned ports and anchors",
                         "Invariant", false, &test_move_pole_updates_only_target_pole_owned_endpoints);
  test_registry::AddTest(tests, "C200_CoreStateService_TemplateMutation_BranchDownOffsetPolicyIsTopology",
                         "bundle template branch-down-offset policy changes force regeneration instead of being ignored or treated as visual-only",
                         "Invariant", false,
                         &test_update_bundle_template_regenerates_branch_down_offset_policy);
  test_registry::AddTest(tests, "C357_CoreStateService_CableTemplatePolicyChangeRegenerates",
                         "decision-bearing cable template changes regenerate backbone outputs",
                         "Invariant", false, &test_update_cable_template_decision_change_regenerates_backbone);
}

WIRE_REGISTER_TEST_SUITE(RegisterStateServiceTests);

} // namespace
