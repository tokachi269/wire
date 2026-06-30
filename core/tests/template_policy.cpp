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
  return !generated.ok && regex_contains(generated.error, "node bundle mode");
}

bool test_bundle_template_topology_change_is_rejected_before_mutation() {
  CoreState state;
  const auto made = make_bb2_fixture(state, {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}});
  if (!made.ok || made.value.spans.empty()) {
    return false;
  }

  const auto tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  if (tpl_it == state.view().bundle_templates().end()) {
    return false;
  }
  wire::core::BundleTemplate tpl = tpl_it->second;
  const wire::core::SpanLayer original_layer = tpl.default_layer;
  tpl.default_layer = wire::core::SpanLayer::kCommunication;
  const auto apply = state.UpdateBundleTemplate(tpl);
  if (apply.ok || !regex_contains(apply.error, "unsupported")) return false;
  const auto* span = state.view().edit_state().spans.find(made.value.spans.front());
  if (span == nullptr) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
  const auto current = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  return bundle != nullptr && current != state.view().bundle_templates().end() &&
         current->second.default_layer == original_layer;
}

bool test_bundle_template_output_change_rejects_manual_span_before_mutation() {
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

  const auto add = add_connection_by_category(state, pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!add.ok) {
    return false;
  }

  const auto tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  if (tpl_it == state.view().bundle_templates().end()) {
    return false;
  }
  wire::core::BundleTemplate tpl = tpl_it->second;
  const wire::core::CableTemplateId original_cable = tpl.cable_template_id;
  tpl.cable_template_id = wire::core::CableTemplateId{3};
  const auto apply = state.UpdateBundleTemplate(tpl);
  if (apply.ok) return false;
  const auto* span = state.view().edit_state().spans.find(add.value.span_id);
  if (span == nullptr) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
  const auto current = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  return bundle != nullptr && current != state.view().bundle_templates().end() &&
         current->second.cable_template_id == original_cable;
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
  return !result.ok && regex_contains(result.error, "bundle");
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

namespace {

void register_template_policy_tests(test_registry::TestRegistry& tests) {

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
  test_registry::AddTest(tests, "C101_Backbone_HVRejectMidairBranch",
                         "HV template keeps midair branch disabled", "Exact", true,
                         test_backbone_hv_rejects_midair_branch_mode);

  test_registry::AddTest(tests, "C123_BundleTemplate_TopologyChangeRejected",
                         "Topology-affecting bundle template edits reject before mutation",
                         "Invariant", true, test_bundle_template_topology_change_is_rejected_before_mutation);
  test_registry::AddTest(tests, "C124_BundleTemplate_OutputChangeRejectsManualSpan",
                         "Bundle output changes reject when the span has no bb2 derive state",
                         "Invariant", true, test_bundle_template_output_change_rejects_manual_span_before_mutation);
}

WIRE_REGISTER_TEST_SUITE(register_template_policy_tests);

} // namespace
