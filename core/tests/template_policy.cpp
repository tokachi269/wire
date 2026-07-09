#include <vector>

#include <iostream>
#include "wire/core/model_descriptor.hpp"

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
  const auto it_template = state.view().bundle_templates().find(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage));
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
  hv_branch.bundle_template_id = wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage);
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
  const auto made = make_backbone_fixture(state, {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}});
  if (!made.ok || made.value.spans.empty()) {
    return false;
  }

  const auto tpl_it = state.view().bundle_templates().find(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage));
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
  const auto current = state.view().bundle_templates().find(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage));
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

  const auto tpl_it = state.view().bundle_templates().find(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage));
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
  const auto current = state.view().bundle_templates().find(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage));
  return bundle != nullptr && current != state.view().bundle_templates().end() &&
         current->second.cable_template_id == original_cable;
}

bool C678_model_descriptor_merge_is_deterministic_and_reports_conflicts() {
  wire::core::ModelMeasurement measurement{};
  measurement.name = "terminal_box";
  measurement.version = 7;
  measurement.total_height_m = 1.2;
  measurement.replace_length_m = 0.4;
  measurement.sockets.push_back({"line_in", wire::core::ModelSocketRole::kLineIn, {-0.2, 0.0, 0.1}, {-1.0, 0.0, 0.0}});
  measurement.sockets.push_back({"line_out", wire::core::ModelSocketRole::kLineOut, {0.2, 0.0, 0.1}, {1.0, 0.0, 0.0}});
  measurement.sections.push_back({0.0, wire::core::ModelSectionShape::kRect, 0.0, 0.3, 0.2});
  measurement.keep_out_zones.push_back({0.0, 0.2, -15.0, 15.0});

  const wire::core::ModelMeasurement original_measurement = measurement;
  wire::core::ModelOverride override_values{};
  override_values.socket_position_overrides.push_back({"line_in", {-0.3, 0.0, 0.12}});
  override_values.socket_position_overrides.push_back({"deleted_socket", {1.0, 2.0, 3.0}});
  const wire::core::ModelOverride original_override = override_values;

  const wire::core::ModelMergeResult first = wire::core::merge(measurement, override_values);
  const wire::core::ModelMergeResult second = wire::core::merge(measurement, override_values);
  if (first.report.conflicts.size() != 1 || second.report.conflicts.size() != 1) {
    return false;
  }
  if (first.report.conflicts.front().marker_name != "deleted_socket" ||
      first.report.conflicts.front().item_name != "socket.local_position") {
    return false;
  }
  if (first.descriptor.measurement.sockets.size() != 2 || second.descriptor.measurement.sockets.size() != 2) {
    return false;
  }
  if (first.descriptor.measurement.sockets.front().local_position.x != -0.3 ||
      second.descriptor.measurement.sockets.front().local_position.x != -0.3) {
    return false;
  }
  if (measurement.sockets.front().local_position.x != original_measurement.sockets.front().local_position.x ||
      override_values.socket_position_overrides != original_override.socket_position_overrides) {
    return false;
  }
  return first.descriptor.measurement.sockets == second.descriptor.measurement.sockets &&
         first.report.conflicts.front().message == second.report.conflicts.front().message;
}

wire::core::AttachmentTemplateId find_replace_attachment_template_id(const CoreState& state) {
  for (const auto& [template_id, attachment_template] : state.view().attachment_templates()) {
    if (attachment_template.line_interaction_mode == wire::core::AttachmentLineInteractionMode::kReplaceWithInternalPath &&
        !attachment_template.internal_paths.empty()) {
      return template_id;
    }
  }
  return wire::core::kInvalidAttachmentTemplateId;
}

bool replacement_first_point(const CoreState& state, ObjectId span_id, wire::core::Vec3d* out) {
  const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  if (curve == nullptr || curve->detail.replacement_paths.empty() ||
      curve->detail.replacement_paths.front().points.empty() || out == nullptr) {
    return false;
  }
  *out = curve->detail.replacement_paths.front().points.front();
  return true;
}

bool C679_attachment_template_geometry_update_rederives_used_span() {
  CoreState state;
  const auto made = make_backbone_fixture(state, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}});
  if (!made.ok || made.value.spans.empty()) {
    return false;
  }
  const wire::core::AttachmentTemplateId template_id = find_replace_attachment_template_id(state);
  const wire::core::AttachmentTemplate* before_template = state.find_attachment_template(template_id);
  if (before_template == nullptr || before_template->sockets.empty()) {
    return false;
  }
  const auto attachment =
      state.AddAttachment(made.value.spans.front(), 0.5, before_template->kind, 0.0, template_id);
  if (!attachment.ok || !state.DeriveGeneratedSpanOutputs(made.value.spans.front()).ok) {
    return false;
  }

  wire::core::Vec3d before_point{};
  if (!replacement_first_point(state, made.value.spans.front(), &before_point)) {
    return false;
  }

  wire::core::AttachmentTemplate changed = *before_template;
  changed.sockets.front().local_position.x -= 0.08;
  const auto update = state.UpdateAttachmentTemplate(changed);
  if (!update.ok || !update.value) {
    return false;
  }

  wire::core::Vec3d after_point{};
  if (!replacement_first_point(state, made.value.spans.front(), &after_point)) {
    return false;
  }
  if (almost_equal(before_point, after_point, 1e-9)) {
    return false;
  }
  return true;
}

bool C680_attachment_template_structure_update_rejects_used_span_unchanged() {
  CoreState state;
  const auto made = make_backbone_fixture(state, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}});
  if (!made.ok || made.value.spans.empty()) {
    return false;
  }
  const wire::core::AttachmentTemplateId template_id = find_replace_attachment_template_id(state);
  const wire::core::AttachmentTemplate* before_template = state.find_attachment_template(template_id);
  if (before_template == nullptr || before_template->sockets.empty()) {
    return false;
  }
  const auto attachment =
      state.AddAttachment(made.value.spans.front(), 0.5, before_template->kind, 0.0, template_id);
  if (!attachment.ok || !state.DeriveGeneratedSpanOutputs(made.value.spans.front()).ok) {
    return false;
  }
  wire::core::Vec3d before_point{};
  if (!replacement_first_point(state, made.value.spans.front(), &before_point)) {
    return false;
  }

  wire::core::AttachmentTemplate changed = *before_template;
  wire::core::AttachmentSocketTemplate added = changed.sockets.front();
  added.id += 100;
  changed.sockets.push_back(added);
  const auto update = state.UpdateAttachmentTemplate(changed);
  const wire::core::AttachmentTemplate* after_template = state.find_attachment_template(template_id);
  wire::core::Vec3d after_point{};
  if (update.ok || !regex_contains(update.error, "structural") || after_template == nullptr ||
      after_template->sockets.size() != before_template->sockets.size() ||
      !replacement_first_point(state, made.value.spans.front(), &after_point)) {
    return false;
  }
  return almost_equal(before_point, after_point, 1e-9);
}

bool has_validation_issue(const wire::core::ValidationResult& result, const std::string& code) {
  return std::any_of(result.issues.begin(), result.issues.end(), [&](const wire::core::ValidationIssue& issue) {
    return issue.code == code;
  });
}

bool install_pathless_replace_template(CoreState& state, wire::core::AttachmentTemplateId* out_template_id) {
  const wire::core::AttachmentTemplateId template_id = find_replace_attachment_template_id(state);
  const wire::core::AttachmentTemplate* before_template = state.find_attachment_template(template_id);
  if (before_template == nullptr) {
    return false;
  }
  wire::core::AttachmentTemplate pathless = *before_template;
  pathless.internal_paths.clear();
  const auto update = state.UpdateAttachmentTemplate(pathless);
  if (!update.ok) {
    return false;
  }
  if (out_template_id != nullptr) {
    *out_template_id = template_id;
  }
  return true;
}

bool C681_pathless_replace_hides_interval_without_replacement_path() {
  CoreState state;
  wire::core::AttachmentTemplateId template_id = wire::core::kInvalidAttachmentTemplateId;
  if (!install_pathless_replace_template(state, &template_id)) {
    return false;
  }
  const auto made = make_backbone_fixture(state, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}});
  const wire::core::AttachmentTemplate* attachment_template = state.find_attachment_template(template_id);
  if (!made.ok || made.value.spans.empty() || attachment_template == nullptr) {
    return false;
  }
  const auto attachment =
      state.AddAttachment(made.value.spans.front(), 0.5, attachment_template->kind, 0.0, template_id);
  if (!attachment.ok || !state.DeriveGeneratedSpanOutputs(made.value.spans.front()).ok) {
    return false;
  }
  const wire::core::CurveCacheEntry* curve = state.find_curve_cache(made.value.spans.front());
  return curve != nullptr && !curve->detail.hidden_intervals.empty() && curve->detail.replacement_paths.empty();
}

bool C682_replaced_intervals_overlap_is_validation_error() {
  CoreState state;
  wire::core::AttachmentTemplateId template_id = wire::core::kInvalidAttachmentTemplateId;
  if (!install_pathless_replace_template(state, &template_id)) {
    return false;
  }
  const auto made = make_backbone_fixture(state, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}});
  const wire::core::AttachmentTemplate* attachment_template = state.find_attachment_template(template_id);
  if (!made.ok || made.value.spans.empty() || attachment_template == nullptr) {
    return false;
  }
  const auto first = state.AddAttachment(made.value.spans.front(), 0.50, attachment_template->kind, 0.0, template_id);
  const auto second = state.AddAttachment(made.value.spans.front(), 0.51, attachment_template->kind, 0.0, template_id);
  if (!first.ok || !second.ok) {
    return false;
  }
  return has_validation_issue(wire::core::CoreStateTestHook::validate(state), "AttachmentReplacementIntervalOverlap");
}

wire::core::ModelDescriptor make_ball_insulator_descriptor() {
  wire::core::ModelDescriptor descriptor{};
  descriptor.measurement.name = "ball_insulator";
  descriptor.measurement.version = 1;
  descriptor.measurement.replace_length_m = 0.32;
  descriptor.measurement.sockets.push_back(
      {"line_in", wire::core::ModelSocketRole::kLineIn, {-0.16, 0.0, 0.0}, {-1.0, 0.0, 0.0}});
  descriptor.measurement.sockets.push_back(
      {"line_out", wire::core::ModelSocketRole::kLineOut, {0.16, 0.0, 0.0}, {1.0, 0.0, 0.0}});
  return descriptor;
}

wire::core::ModelDescriptor make_terminal_box_descriptor() {
  wire::core::ModelDescriptor descriptor{};
  descriptor.measurement.name = "terminal_box";
  descriptor.measurement.version = 3;
  descriptor.measurement.replace_length_m = 0.46;
  descriptor.measurement.sockets.push_back(
      {"line_in", wire::core::ModelSocketRole::kLineIn, {-0.23, 0.0, 0.0}, {-1.0, 0.0, 0.0}});
  descriptor.measurement.sockets.push_back(
      {"line_out", wire::core::ModelSocketRole::kLineOut, {0.23, 0.0, 0.0}, {1.0, 0.0, 0.0}});
  descriptor.measurement.sockets.push_back(
      {"drop", wire::core::ModelSocketRole::kDrop, {0.0, -0.12, -0.18}, {0.0, -1.0, -0.2}});
  return descriptor;
}

bool C683_model_descriptor_builds_pathless_attachment_template() {
  const wire::core::ModelDescriptor descriptor = make_ball_insulator_descriptor();
  const wire::core::ModelAttachmentTemplateBuildResult built =
      wire::core::build_attachment_template(descriptor, wire::core::AttachmentTemplateId{42});
  if (!built.report.conflicts.empty()) {
    return false;
  }
  return built.attachment_template.id == wire::core::AttachmentTemplateId{42} &&
         built.attachment_template.name == "ball_insulator" &&
         built.attachment_template.kind == wire::core::AttachmentKind::kSpacer &&
         built.attachment_template.line_interaction_mode ==
             wire::core::AttachmentLineInteractionMode::kReplaceWithInternalPath &&
         built.attachment_template.sockets.size() == 2 && built.attachment_template.internal_paths.empty() &&
         built.attachment_template.sockets[0].id == 0 && built.attachment_template.sockets[1].id == 1;
}

bool C684_terminal_box_descriptor_keeps_drop_socket_without_internal_path() {
  const wire::core::ModelDescriptor descriptor = make_terminal_box_descriptor();
  const wire::core::ModelAttachmentTemplateBuildResult built =
      wire::core::build_attachment_template(descriptor, wire::core::AttachmentTemplateId{43});
  if (!built.report.conflicts.empty() || built.attachment_template.sockets.size() != 3 ||
      !built.attachment_template.internal_paths.empty()) {
    return false;
  }
  return built.attachment_template.sockets[0].id == 0 && built.attachment_template.sockets[1].id == 1 &&
         built.attachment_template.sockets[2].kind == wire::core::AttachmentSocketKind::kGeneric;
}

bool C685_descriptor_attachment_template_updates_used_span_and_reload_geometry() {
  CoreState state;
  const auto made = make_backbone_fixture(state, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}});
  const wire::core::AttachmentTemplateId template_id = find_replace_attachment_template_id(state);
  if (!made.ok || made.value.spans.empty()) {
    return false;
  }

  wire::core::ModelDescriptor descriptor = make_ball_insulator_descriptor();
  wire::core::ModelAttachmentTemplateBuildResult built = wire::core::build_attachment_template(descriptor, template_id);
  if (!built.report.conflicts.empty() || !state.UpdateAttachmentTemplate(built.attachment_template).ok) {
    return false;
  }
  const wire::core::AttachmentTemplate* first_template = state.find_attachment_template(template_id);
  if (first_template == nullptr) {
    return false;
  }
  const auto attachment =
      state.AddAttachment(made.value.spans.front(), 0.5, first_template->kind, 0.0, template_id);
  if (!attachment.ok || !state.DeriveGeneratedSpanOutputs(made.value.spans.front()).ok) {
    return false;
  }
  const wire::core::CurveCacheEntry* before_curve = state.find_curve_cache(made.value.spans.front());
  if (before_curve == nullptr || before_curve->detail.hidden_intervals.empty() ||
      !before_curve->detail.replacement_paths.empty()) {
    return false;
  }
  const double before_start = before_curve->detail.hidden_intervals.front().start_m;

  descriptor.measurement.version += 1;
  descriptor.measurement.sockets[0].local_position.x = -0.24;
  built = wire::core::build_attachment_template(descriptor, template_id);
  const auto update = state.UpdateAttachmentTemplate(built.attachment_template);
  const wire::core::CurveCacheEntry* after_curve = state.find_curve_cache(made.value.spans.front());
  if (!update.ok || after_curve == nullptr || after_curve->detail.hidden_intervals.empty()) {
    return false;
  }
  return std::abs(after_curve->detail.hidden_intervals.front().start_m - before_start) > 1e-6;
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
  lv.bundle_template_id = wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage);
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
  bundle.bundle_template_id = wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage);
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
  lv.bundle_template_id = wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage);
  req.bundles.push_back(lv);

  wire::core::BackboneBundleSpec comm{};
  comm.bundle_template_id = wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication);
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
    if (bundle->bundle_template_id == wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage)) {
      ++low_voltage_bundle_count;
    } else if (bundle->bundle_template_id == wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)) {
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
                         "Bundle output changes reject when the span has no backbone derive state",
                         "Invariant", true, test_bundle_template_output_change_rejects_manual_span_before_mutation);
  test_registry::AddTest(tests, "C678_ModelDescriptor_Merge",
                         "ModelDescriptor merge applies valid overrides and reports missing marker conflicts",
                         "Invariant", true, C678_model_descriptor_merge_is_deterministic_and_reports_conflicts);
  test_registry::AddTest(tests, "C679_AttachmentTemplate_GeometryUpdateRederivesUsedSpan",
                         "Attachment template geometry edits rederive curves for used spans",
                         "Invariant", false, C679_attachment_template_geometry_update_rederives_used_span);
  test_registry::AddTest(tests, "C680_AttachmentTemplate_StructureUpdateRejectsUsedSpan",
                         "Attachment template structural edits reject before mutation with used spans unchanged",
                         "Invariant", true, C680_attachment_template_structure_update_rejects_used_span_unchanged);
  test_registry::AddTest(tests, "C681_AttachmentTemplate_PathlessReplaceHidesInterval",
                         "Pathless ReplaceWithInternalPath hides the replaced interval without replacement points",
                         "Invariant", false, C681_pathless_replace_hides_interval_without_replacement_path);
  test_registry::AddTest(tests, "C682_AttachmentReplacementIntervalOverlap",
                         "Overlapping replacement intervals on one span are validation errors",
                         "Invariant", true, C682_replaced_intervals_overlap_is_validation_error);
  test_registry::AddTest(tests, "C683_ModelDescriptor_BuildsPathlessAttachmentTemplate",
                         "Model descriptor builds a pathless replacement AttachmentTemplate for inline fixtures",
                         "Invariant", false, C683_model_descriptor_builds_pathless_attachment_template);
  test_registry::AddTest(tests, "C684_ModelDescriptor_TerminalBoxKeepsDropSocket",
                         "Terminal box descriptor keeps drop socket while replacing only the main line interval",
                         "Invariant", false, C684_terminal_box_descriptor_keeps_drop_socket_without_internal_path);
  test_registry::AddTest(tests, "C685_ModelDescriptor_AttachmentTemplateReloadUpdatesCurve",
                         "Descriptor-built attachment template updates an existing span through the normal template path",
                         "Invariant", false, C685_descriptor_attachment_template_updates_used_span_and_reload_geometry);
}

WIRE_REGISTER_TEST_SUITE(register_template_policy_tests);

} // namespace
