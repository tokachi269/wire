#include <vector>

#include <iostream>
#include "registry.hpp"
#include "helpers.hpp"

using namespace helpers;
using wire::core::SpanKind;
using wire::core::SpanLayer;

bool test_backbone_hv_rejects_midair_branch_mode() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const auto it_template = state.view().bundle_templates().find(wire::core::BundleKind::kHighVoltage);
  if (it_template == state.view().bundle_templates().end()) {
    return false;
  }
  if (it_template->second.allow_midair_branch) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  req.path.node_specs.push_back(midair);
  wire::core::BackboneSpec::NodeBundleModeSpec hv_branch{};
  hv_branch.point_index = 1;
  hv_branch.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  hv_branch.mode = static_cast<wire::core::BundleNodeMode>(2);
  req.node_bundle_modes.push_back(hv_branch);
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);

  const auto generated = state.GenerateFromBackboneSpec(req);
  return !generated.ok && regex_contains(generated.error, "unsupported bundle node mode");
}

// Intent: Bundle-specific two-state modes can coexist at one support node.
bool test_backbone_support_node_allows_per_bundle_mode_mix() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  req.path.node_specs.push_back(midair);
  wire::core::BackboneSpec::NodeBundleModeSpec hv_pass{};
  hv_pass.point_index = 1;
  hv_pass.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  hv_pass.mode = wire::core::BundleNodeMode::kPassThrough;
  req.node_bundle_modes.push_back(hv_pass);
  wire::core::BackboneSpec::NodeBundleModeSpec comm_not_present{};
  comm_not_present.point_index = 1;
  comm_not_present.bundle_template_id = wire::core::BundleKind::kCommunication;
  comm_not_present.mode = wire::core::BundleNodeMode::kNotPresent;
  req.node_bundle_modes.push_back(comm_not_present);
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);

  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }
  const auto backbone = state.BuildBackboneResult();
  const auto* node = find_support_node_by_point_index(backbone, 1);
  if (node == nullptr) {
    return false;
  }
  bool hv_ok = false;
  bool comm_ok = false;
  for (const auto& mode : node->bundle_modes) {
    if (mode.bundle_template_id == wire::core::BundleKind::kHighVoltage &&
        mode.mode == wire::core::BundleNodeMode::kPassThrough) {
      hv_ok = true;
    }
    if (mode.bundle_template_id == wire::core::BundleKind::kCommunication &&
        mode.mode == wire::core::BundleNodeMode::kNotPresent) {
      comm_ok = true;
    }
  }
  return hv_ok && comm_ok;
}

bool test_bundle_template_topology_change_marks_regeneration_required() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b{};
  b.position = {10.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = true;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  const auto add = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage, options);
  if (!add.ok) {
    return false;
  }

  const auto tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  if (tpl_it == state.view().bundle_templates().end()) {
    return false;
  }
  wire::core::BundleTemplate tpl = tpl_it->second;
  tpl.default_layer = wire::core::SpanLayer::kCommunication;
  const auto apply = state.UpdateBundleTemplate(tpl);
  if (!apply.ok || !apply.value) {
    return false;
  }
  const auto* span = state.view().edit_state().spans.find(add.value.span_id);
  if (span == nullptr) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
  if (bundle == nullptr || !bundle->regeneration_required) {
    return false;
  }
  const auto& deps = state.view().template_dependency_state();
  return std::find(deps.bundles_requiring_regeneration.begin(), deps.bundles_requiring_regeneration.end(), bundle->id) !=
         deps.bundles_requiring_regeneration.end();
}

bool test_bundle_template_visual_change_updates_dirty_spans_without_regeneration() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b{};
  b.position = {10.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = true;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  const auto add = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage, options);
  if (!add.ok) {
    return false;
  }

  const auto tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  if (tpl_it == state.view().bundle_templates().end()) {
    return false;
  }
  wire::core::BundleTemplate tpl = tpl_it->second;
  tpl.cable_template_id = wire::core::CableTemplateId{3};
  const auto apply = state.UpdateBundleTemplate(tpl);
  if (!apply.ok || !apply.value) {
    return false;
  }
  const auto* span = state.view().edit_state().spans.find(add.value.span_id);
  if (span == nullptr) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
  if (bundle == nullptr || bundle->regeneration_required) {
    return false;
  }
  const auto* runtime = state.view().find_span_runtime_state(span->id);
  if (!has_dirty(runtime, wire::core::DirtyBits::kGeometry | wire::core::DirtyBits::kRender)) {
    return false;
  }
  const auto& deps = state.view().template_dependency_state();
  return deps.bundles_requiring_regeneration.empty() && deps.sessions_requiring_regeneration.empty() &&
         contains_id(apply.change_set.dirty_span_ids, span->id);
}

// Intent: Detailed generation should not crash when path includes non-pole support nodes.

bool test_backbone_generation_requires_non_empty_bundles() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  req.interval_m = 5.0;
  req.pole_type_id = type_ids.front();
  const auto result = state.GenerateFromBackboneSpec(req);
  return !result.ok && regex_contains(result.error, "bundles\\[\\]");
}

// Intent: Auto bundle creation on AddConnectionByPole must require explicit bundle template.
bool test_add_connection_requires_bundle_template_when_auto_create_enabled() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b{};
  b.position = {10.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = true;
  options.use_bundle_template = false;
  options.bundle_id = wire::core::kInvalidObjectId;
  const auto result = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage, options);
  return !result.ok && regex_contains(result.error, "bundle_template_id");
}

// Intent: AddConnectionByPole should derive behavior from template/bundle, not caller category.
bool test_add_connection_template_profile_overrides_category_fallback() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b{};
  b.position = {10.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = true;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  options.span_kind = SpanKind::kDistribution;

  // Intentionally pass a mismatched category; template must win.
  const auto add = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage, options);
  if (!add.ok) {
    return false;
  }

  const auto* span = state.view().edit_state().spans.find(add.value.span_id);
  if (span == nullptr || span->layer != SpanLayer::kHighVoltage) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
  if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage ||
      bundle->conductor_count != 3) {
    return false;
  }
  const auto* port_a = state.view().edit_state().ports.find(add.value.port_a_id);
  const auto* port_b = state.view().edit_state().ports.find(add.value.port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    return false;
  }
  return port_a->category == ConnectionCategory::kHighVoltage && port_b->category == ConnectionCategory::kHighVoltage;
}

// Intent: Explicit bundle_id and bundle_template_id mismatch must be rejected.
bool test_add_connection_rejects_bundle_id_template_mismatch() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b{};
  b.position = {10.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());
  const auto bundle = state.AddBundle(1, 0.2, wire::core::BundleKind::kLowVoltage);
  if (!bundle.ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.bundle_id = bundle.value;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  options.auto_create_bundle = false;
  const auto add = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage, options);
  return !add.ok && regex_contains(add.error, "mismatch");
}

// Intent: use_bundle_template with auto_create disabled must require explicit bundle_id.
bool test_add_connection_template_requires_bundle_id_when_auto_create_disabled() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b{};
  b.position = {10.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = false;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  const auto add = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage, options);
  return !add.ok && regex_contains(add.error, "bundle_id is required");
}

// Intent: span_layer override must not conflict with bundle template default layer.
bool test_add_connection_rejects_span_layer_override_conflict() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b{};
  b.position = {10.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = true;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  options.span_layer = SpanLayer::kLowVoltage;
  const auto add = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage, options);
  return !add.ok && regex_contains(add.error, "span_layer override");
}

// Intent: Drop generation should use bundle template defaults (not hardcoded spacing/category mapping).
bool test_drop_generation_uses_template_defaults() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const ObjectId pole = state.AddPole({}, 10.0, "DropSrc").value;
  (void)state.ApplyPoleType(pole, type_ids.front());
  const auto drop = state.AddDropFromPole(pole, {8.0, 2.0, 3.0}, ConnectionCategory::kDrop);
  if (!drop.ok) {
    return false;
  }
  const auto* span = state.view().edit_state().spans.find(drop.value.span_id);
  if (span == nullptr || span->bundle_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
  const auto tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kDrop);
  const auto lv_tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  if (bundle == nullptr || tpl_it == state.view().bundle_templates().end() ||
      lv_tpl_it == state.view().bundle_templates().end()) {
    return false;
  }
  const auto& tpl = tpl_it->second;
  const auto& lv_tpl = lv_tpl_it->second;
  return span->layer == wire::core::SpanLayer::kDrop && span->layer == tpl.default_layer &&
         bundle->bundle_template_id == wire::core::BundleKind::kDrop && bundle->bundle_template_id == tpl.id &&
         almost_equal(bundle->phase_spacing_m, tpl.default_spacing_m, 1e-9) &&
         tpl.cable_template_id != lv_tpl.cable_template_id;
}

// Intent: Fixed bundle template must reject explicit count override inputs.
bool test_backbone_bundle_template_fixed_count_rejects_override() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {16.0, 0.0, 0.0}};
  req.interval_m = 8.0;
  req.pole_type_id = type_ids.front();
  req.direction_mode = wire::core::PathDirectionMode::kAuto;
  wire::core::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  bundle.count = 3; // fixed template should reject any explicit count input.
  req.bundles.push_back(bundle);
  const auto result = state.GenerateFromBackboneSpec(req);
  return !result.ok && regex_contains(result.error, "count override");
}

// Intent: Non-HV templates should default to fixed single conductor generation.
bool test_backbone_bundle_template_default_single_generates_one_span_per_segment() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 0.0, 0.0}};
  req.interval_m = 6.0;
  req.pole_type_id = type_ids.front();
  wire::core::BackboneBundleSpec lv{};
  lv.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  req.bundles.push_back(lv);
  const auto result = state.GenerateFromBackboneSpec(req);
  if (!result.ok || result.value.generated_pole_ids.size() < 2 || result.value.bundle_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(result.value.bundle_id);
  if (bundle == nullptr || bundle->conductor_count != 1) {
    return false;
  }
  const std::size_t segment_count = result.value.generated_pole_ids.size() - 1;
  return result.value.generated_span_ids.size() == segment_count;
}

// Intent: Non-HV fixed single templates must reject count override.
bool test_backbone_bundle_template_default_single_rejects_count_override() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {16.0, 0.0, 0.0}};
  req.interval_m = 8.0;
  req.pole_type_id = type_ids.front();
  wire::core::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  bundle.count = 2;
  req.bundles.push_back(bundle);
  const auto result = state.GenerateFromBackboneSpec(req);
  return !result.ok && regex_contains(result.error, "count override");
}

// Intent: Multiple bundle template requests should generate multiple bundles and combined spans.
bool test_backbone_bundle_template_multi_request_generates_multiple_bundles() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {16.0, 0.0, 0.0}};
  req.interval_m = 8.0;
  req.pole_type_id = type_ids.front();

  wire::core::BackboneBundleSpec lv{};
  lv.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  req.bundles.push_back(lv);

  wire::core::BackboneBundleSpec comm{};
  comm.bundle_template_id = wire::core::BundleKind::kCommunication;
  req.bundles.push_back(comm);

  const auto result = state.GenerateFromBackboneSpec(req);
  if (!result.ok) {
    return false;
  }
  if (result.value.bundle_ids.size() != 2 || result.value.generated_pole_ids.size() < 2) {
    return false;
  }

  const std::size_t segment_count = result.value.generated_pole_ids.size() - 1;
  const std::size_t expected_span_count = segment_count * 2; // LV1 + COMM1
  if (result.value.generated_span_ids.size() != expected_span_count) {
    return false;
  }

  std::size_t low_voltage_bundle_count = 0;
  std::size_t communication_bundle_count = 0;
  for (ObjectId bundle_id : result.value.bundle_ids) {
    const auto* bundle = state.view().edit_state().bundles.find(bundle_id);
    if (bundle == nullptr) {
      return false;
    }
    if (bundle->bundle_template_id == wire::core::BundleKind::kLowVoltage) {
      ++low_voltage_bundle_count;
    } else if (bundle->bundle_template_id == wire::core::BundleKind::kCommunication) {
      ++communication_bundle_count;
    }
  }
  return low_voltage_bundle_count == 1 && communication_bundle_count == 1;
}

bool test_multilane_identity_template_uses_policy_based_offset_endpoints() {
  CoreState state;

  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C191 no communication pole type\n";
    return false;
  }

  const auto tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kCommunication);
  if (tpl_it == state.view().bundle_templates().end()) {
    return false;
  }
  wire::core::BundleTemplate tpl = tpl_it->second;
  tpl.preserve_conductor_identity = true;
  tpl.default_count = 3;
  tpl.default_spacing_m = 0.45;
  const auto apply = state.UpdateBundleTemplate(tpl);
  if (!apply.ok) {
    std::cerr << "[DBG] C191 UpdateBundleTemplate failed: " << apply.error << "\n";
    return false;
  }
  const auto cable_it = state.view().cable_templates().find(tpl.cable_template_id);
  if (cable_it == state.view().cable_templates().end()) {
    std::cerr << "[DBG] C191 cable template missing\n";
    return false;
  }
  wire::core::CableTemplate cable = cable_it->second;
  cable.attachment_style = wire::core::CableAttachmentStyleHint::kAuto;
  const auto cable_apply = state.UpdateCableTemplate(cable);
  if (!cable_apply.ok) {
    std::cerr << "[DBG] C191 UpdateCableTemplate failed: " << cable_apply.error << "\n";
    return false;
  }

  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd pole_b_tf{};
  pole_b_tf.position = {12.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(pole_b_tf, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, communication_pole_type_id).ok || !state.ApplyPoleType(pole_b, communication_pole_type_id).ok) {
    std::cerr << "[DBG] C191 ApplyPoleType failed\n";
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = true;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kCommunication;
  const auto add =
      add_connection_by_category(state, pole_a, pole_b, wire::core::ConnectionCategory::kCommunication, options);
  if (!add.ok) {
    std::cerr << "[DBG] C191 AddConnection failed: " << add.error << "\n";
    return false;
  }
  (void)state.Commit().recalc_stats;

  const ObjectId span_id = add.value.span_id;
  const auto* span = state.view().edit_state().spans.find(span_id);
  const auto* bundle = (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
  const auto* support_layout = state.view().support_layout_projection(span_id).layout;
  if (support_layout == nullptr || span == nullptr || bundle == nullptr) {
    std::cerr << "[DBG] C191 missing derived span=" << (span != nullptr) << " bundle=" << (bundle != nullptr)
              << " layout=" << (support_layout != nullptr) << "\n";
    return false;
  }

  const bool ok = bundle->bundle_template_id == wire::core::BundleKind::kCommunication &&
                  bundle->conductor_count == 3 &&
                  support_layout->start.endpoint_mode == wire::core::CurveEndpointMode::kOffsetEndpoint &&
                  support_layout->end.endpoint_mode == wire::core::CurveEndpointMode::kOffsetEndpoint;
  if (!ok) {
    const auto current_tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kCommunication);
    std::cerr << "[DBG] C191 preserve="
              << ((current_tpl_it != state.view().bundle_templates().end() && current_tpl_it->second.preserve_conductor_identity)
                      ? 1
                      : 0)
              << " count=" << bundle->conductor_count << " startMode=" << static_cast<int>(support_layout->start.endpoint_mode)
              << " endMode=" << static_cast<int>(support_layout->end.endpoint_mode) << "\n";
  }
  return ok;
}

namespace {

void register_template_policy_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C73_Template_FixedCountReject",
                         "Fixed bundle templates reject explicit count override", "Exact", true,
                         test_backbone_bundle_template_fixed_count_rejects_override);
  test_registry::AddTest(tests, "C74_Template_DefaultSingleOneSpan",
                         "Default single-conductor template generates one span per segment", "Invariant", false,
                         test_backbone_bundle_template_default_single_generates_one_span_per_segment);
  test_registry::AddTest(tests, "C75_Template_DefaultSingleRejectCount",
                         "Default single-conductor template rejects explicit count override", "Exact", true,
                         test_backbone_bundle_template_default_single_rejects_count_override);
  test_registry::AddTest(tests, "C77_Template_MultiBundleRequest",
                         "Multiple bundle templates in one backbone request generate combined results", "Invariant",
                         false, test_backbone_bundle_template_multi_request_generates_multiple_bundles);
  test_registry::AddTest(tests, "C90_Backbone_BundlesRequired",
                         "Backbone generation requires bundles[] and rejects legacy-only fields", "Exact", true,
                         test_backbone_generation_requires_non_empty_bundles);
  test_registry::AddTest(tests, "C91_AddConnection_TemplateRequired",
                         "Auto-created bundle connections require an explicit template", "Exact", true,
                         test_add_connection_requires_bundle_template_when_auto_create_enabled);
  test_registry::AddTest(tests, "C92_AddConnection_TemplateWins",
                         "Bundle template drives connection behavior even if category argument mismatches",
                         "Invariant", false, test_add_connection_template_profile_overrides_category_fallback);
  test_registry::AddTest(tests, "C93_AddConnection_BundleTemplateMismatchReject",
                         "Explicit bundle_id and bundle_template_id mismatch is rejected", "Exact", true,
                         test_add_connection_rejects_bundle_id_template_mismatch);
  test_registry::AddTest(tests, "C94_AddConnection_TemplateNeedsBundleId",
                         "Template use without auto-create still requires explicit bundle_id", "Exact", true,
                         test_add_connection_template_requires_bundle_id_when_auto_create_disabled);
  test_registry::AddTest(tests, "C95_AddConnection_SpanLayerConflictReject",
                         "Template span layer conflicts are rejected", "Exact", true,
                         test_add_connection_rejects_span_layer_override_conflict);
  test_registry::AddTest(tests, "C96_Drop_UsesTemplateDefaults",
                         "Drop generation uses template defaults for spacing and layer", "Invariant", false,
                         test_drop_generation_uses_template_defaults);
  test_registry::AddTest(tests, "C101_Backbone_HVRejectMidairBranch",
                         "HV template keeps midair branch disabled", "Exact", true,
                         test_backbone_hv_rejects_midair_branch_mode);
  test_registry::AddTest(tests, "C102_Backbone_PerBundleModeMix",
                         "One support node can hold different two-state modes per bundle", "Invariant", false,
                         test_backbone_support_node_allows_per_bundle_mode_mix);
  test_registry::AddTest(tests, "C123_BundleTemplate_TopologyChangeMarksRegeneration",
                         "Topology-affecting bundle template edits mark dependent bundles for regeneration",
                         "Invariant", false, test_bundle_template_topology_change_marks_regeneration_required);
  test_registry::AddTest(tests, "C124_BundleTemplate_VisualChangeMarksDirtyOnly",
                         "Visual-only bundle template edits dirty dependent spans without forcing regeneration",
                         "Invariant", false, test_bundle_template_visual_change_updates_dirty_spans_without_regeneration);
  test_registry::AddTest(tests, "C191_Template_PreservedIdentityOffsetEndpoints",
                         "Preserved multi-lane bundles use offset endpoints by template policy instead of HV-only category checks",
                         "Invariant", false, test_multilane_identity_template_uses_policy_based_offset_endpoints);
}

WIRE_REGISTER_TEST_SUITE(register_template_policy_tests);

} // namespace
