#include <array>
#include <sstream>
#include <vector>

#include "registry.hpp"
#include "helpers.hpp"

using namespace helpers;

bool test_backbone_generation_includes_midair_support_nodes() {
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
  midair.has_tangent_hint = true;
  midair.tangent_hint = {1.0, 0.0, 0.0};
  req.path.node_specs.push_back(midair);
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);

  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }
  const auto& backbone = state.view().last_generation_backbone();
  const auto* node = find_support_node_by_point_index(backbone, 1);
  return node != nullptr && node->support_kind == wire::core::SupportKind::kMidair && node->has_tangent_hint &&
         node->bundle_modes.size() == 1 && node->bundle_modes.front().mode == wire::core::BundleNodeMode::kNotPresent;
}

// Intent: Explicit node_ids from DrawPath picks should reuse an existing pole node instead of creating a duplicate.
bool test_backbone_generation_reuses_explicit_pole_node_id() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok) {
    return false;
  }

  const ObjectId anchor_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (anchor_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec second{};
  second.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec anchor{};
  anchor.point_index = 0;
  anchor.support_kind = wire::core::SupportKind::kPole;
  anchor.node_id = anchor_id;
  second.path.node_specs.push_back(anchor);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok) {
    return false;
  }
  if (generated_second.value.generated_pole_ids.size() != 1 || contains_id(generated_second.value.generated_pole_ids, anchor_id)) {
    return false;
  }

  const ObjectId new_end_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  if (new_end_id == wire::core::kInvalidObjectId || new_end_id == anchor_id) {
    return false;
  }
  const auto route = state.FindBackboneRoute(anchor_id, new_end_id);
  return route.size() == 2 && route.front() == anchor_id && route.back() == new_end_id;
}

// Intent: Explicit node_ids from DrawPath picks should reuse an existing non-pole support node in the new backbone.
bool test_backbone_generation_reuses_explicit_support_node_id() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.node_specs.push_back(midair);
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok) {
    return false;
  }
  const auto* existing_midair = find_support_node_by_point_index(state.view().last_generation_backbone(), 1);
  if (existing_midair == nullptr || existing_midair->node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const ObjectId existing_midair_id = existing_midair->node_id;

  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {10.0, 12.0, 0.0}, {20.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = existing_midair_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok) {
    return false;
  }
  const auto* reused = find_support_node_by_point_index(state.view().last_generation_backbone(), 0);
  return reused != nullptr && reused->node_id == existing_midair_id &&
         reused->support_kind == wire::core::SupportKind::kMidair;
}

// Intent: Extending DrawPath from a reused midair support node should still realize detail poles/spans on the new leg.
bool test_backbone_midair_extension_generates_detail_chain() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.node_specs.push_back(midair);
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok) {
    return false;
  }
  const auto* existing_midair = find_support_node_by_point_index(state.view().last_generation_backbone(), 1);
  if (existing_midair == nullptr || existing_midair->node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const ObjectId existing_midair_id = existing_midair->node_id;

  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {22.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = existing_midair_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok) {
    return false;
  }

  return generated_second.value.generated_pole_ids.size() >= 1 && generated_second.value.generated_span_ids.size() >= 1;
}

// Intent: Midair-origin extension must include the first support-to-detail segment in the detailed chain.
bool test_backbone_midair_extension_includes_first_support_segment() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.node_specs.push_back(midair);
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok) {
    return false;
  }
  const auto* existing_midair = find_support_node_by_point_index(state.view().last_generation_backbone(), 1);
  if (existing_midair == nullptr || existing_midair->node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const ObjectId existing_midair_id = existing_midair->node_id;

  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {34.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = existing_midair_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 6.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok || generated_second.value.generated_pole_ids.empty()) {
    return false;
  }

  const ObjectId terminal_pole_id = find_pole_id_by_position(state, {34.0, 0.0, 0.0});
  if (terminal_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const std::vector<ObjectId> route = state.FindBackboneRoute(existing_midair_id, terminal_pole_id);
  return !generated_second.value.generated_span_ids.empty() &&
         generated_second.value.generated_span_ids.size() == generated_second.value.generated_pole_ids.size() &&
         route.size() == generated_second.value.generated_pole_ids.size() + 1 &&
         route.front() == existing_midair_id && route.back() == terminal_pole_id;
}

// Intent: Midair picked on backbone can stay at backbone height while detailed branch starts from source span height.
bool test_backbone_midair_extension_single_click_stays_single_segment() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.node_specs.push_back(midair);
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok) {
    return false;
  }
  const auto* existing_midair = find_support_node_by_point_index(state.view().last_generation_backbone(), 1);
  if (existing_midair == nullptr || existing_midair->node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {18.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = existing_midair->node_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok) {
    return false;
  }
  if (generated_second.value.generated_pole_ids.size() != 1 || generated_second.value.generated_span_ids.size() != 1) {
    return false;
  }

  const ObjectId terminal_pole_id = find_pole_id_by_position(state, {18.0, 8.0, 0.0});
  if (terminal_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const std::vector<ObjectId> route = state.FindBackboneRoute(existing_midair->node_id, terminal_pole_id);
  return route.size() == 2 && route.front() == existing_midair->node_id && route.back() == terminal_pole_id;
}

bool test_backbone_midair_branch_reuses_source_span_height() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok || generated_first.value.generated_span_ids.size() != 1) {
    return false;
  }
  const ObjectId source_span_id = generated_first.value.generated_span_ids.front();
  const wire::core::Span* source_span = state.view().edit_state().spans.find(source_span_id);
  if (source_span == nullptr) {
    return false;
  }
  const wire::core::Port* source_port_a = state.view().edit_state().ports.find(source_span->port_a_id);
  const wire::core::Port* source_port_b = state.view().edit_state().ports.find(source_span->port_b_id);
  if (source_port_a == nullptr || source_port_b == nullptr) {
    return false;
  }
  const double expected_z = 0.5 * (source_port_a->world_position.z + source_port_b->world_position.z);

  const ObjectId start_pole_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId end_pole_id = find_pole_id_by_position(state, {20.0, 0.0, 0.0});
  if (start_pole_id == wire::core::kInvalidObjectId || end_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span_id;
  pick.hit_pos_world = {10.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = start_pole_id;
  pick.segment_node_b_id = end_pole_id;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {20.0, 0.0, 0.0};

  wire::core::CoreState::ResolveBranchPickOptions resolve{};
  resolve.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  resolve.snap_radius_world = 0.75;
  resolve.create_midair_node = true;
  const auto picked_midair = state.ResolveBranchPick(pick, resolve);
  if (!picked_midair.ok || picked_midair.value.resolved_node_id == wire::core::kInvalidObjectId ||
      picked_midair.value.support_kind != wire::core::SupportKind::kMidair) {
    return false;
  }
  const ObjectId midair_id = picked_midair.value.resolved_node_id;
  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {20.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = midair_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok || generated_second.value.generated_span_ids.empty()) {
    return false;
  }

  const wire::core::Span* branch_span = nullptr;
  for (ObjectId span_id : generated_second.value.generated_span_ids) {
    const wire::core::Span* candidate = state.view().edit_state().spans.find(span_id);
    if (candidate == nullptr) {
      continue;
    }
    if (candidate->endpoint_node_a_id == midair_id || candidate->endpoint_node_b_id == midair_id) {
      branch_span = candidate;
      break;
    }
  }
  if (branch_span == nullptr) {
    return false;
  }
  const bool midair_is_a = branch_span->endpoint_node_a_id == midair_id;
  const bool midair_is_b = branch_span->endpoint_node_b_id == midair_id;
  if (!midair_is_a && !midair_is_b) {
    return false;
  }
  const ObjectId branch_port_id = midair_is_a ? branch_span->port_a_id : branch_span->port_b_id;
  const wire::core::Port* branch_port = state.view().edit_state().ports.find(branch_port_id);
  if (branch_port == nullptr) {
    return false;
  }
  return std::abs(branch_port->world_position.z - expected_z) <= 1e-6;
}

// Intent: Disallowed templates should not connect through a source-edge Midair branch, but the request should still succeed.
bool test_backbone_midair_branch_skips_disallowed_template_generation() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok || generated_first.value.generated_span_ids.size() != 1) {
    return false;
  }

  const ObjectId source_span_id = generated_first.value.generated_span_ids.front();
  const ObjectId start_pole_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId end_pole_id = find_pole_id_by_position(state, {20.0, 0.0, 0.0});
  if (start_pole_id == wire::core::kInvalidObjectId || end_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span_id;
  pick.hit_pos_world = {10.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = start_pole_id;
  pick.segment_node_b_id = end_pole_id;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {20.0, 0.0, 0.0};

  wire::core::CoreState::ResolveBranchPickOptions resolve{};
  resolve.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  resolve.snap_radius_world = 0.75;
  resolve.create_midair_node = true;
  const auto picked_midair = state.ResolveBranchPick(pick, resolve);
  if (!picked_midair.ok || picked_midair.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {20.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = picked_midair.value.resolved_node_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kHighVoltage);
  const CoreCounts before_second = snapshot_counts(state);
  const auto generated_second = state.GenerateFromBackboneSpec(second);
  return generated_second.ok && generated_second.value.generated_pole_ids.empty() &&
         generated_second.value.generated_span_ids.empty() && generated_second.value.bundle_ids.empty() &&
         same_counts(before_second, snapshot_counts(state));
}

// Intent: Mixed template generation from a Midair branch should generate only bundles whose template allows midair branch.
bool test_backbone_midair_branch_generates_only_allowed_templates() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec first{};
  first.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  first.interval_m = 1000.0;
  first.pole_type_id = type_ids.front();
  add_backbone_bundle(first, wire::core::BundleKind::kLowVoltage);
  const auto generated_first = state.GenerateFromBackboneSpec(first);
  if (!generated_first.ok || generated_first.value.generated_span_ids.size() != 1) {
    return false;
  }

  const ObjectId source_span_id = generated_first.value.generated_span_ids.front();
  const ObjectId start_pole_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId end_pole_id = find_pole_id_by_position(state, {20.0, 0.0, 0.0});
  if (start_pole_id == wire::core::kInvalidObjectId || end_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span_id;
  pick.hit_pos_world = {10.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = start_pole_id;
  pick.segment_node_b_id = end_pole_id;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {20.0, 0.0, 0.0};

  wire::core::CoreState::ResolveBranchPickOptions resolve{};
  resolve.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  resolve.snap_radius_world = 0.75;
  resolve.create_midair_node = true;
  const auto picked_midair = state.ResolveBranchPick(pick, resolve);
  if (!picked_midair.ok || picked_midair.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec second{};
  second.path.polyline = {{10.0, 0.0, 0.0}, {20.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec reused_midair{};
  reused_midair.point_index = 0;
  reused_midair.support_kind = wire::core::SupportKind::kMidair;
  reused_midair.node_id = picked_midair.value.resolved_node_id;
  second.path.node_specs.push_back(reused_midair);
  second.interval_m = 1000.0;
  second.pole_type_id = type_ids.front();
  add_backbone_bundle(second, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(second, wire::core::BundleKind::kHighVoltage);

  const auto generated_second = state.GenerateFromBackboneSpec(second);
  if (!generated_second.ok || generated_second.value.generated_span_ids.empty() || generated_second.value.bundle_ids.size() != 1) {
    return false;
  }
  const wire::core::Bundle* bundle = state.view().edit_state().bundles.find(generated_second.value.bundle_ids.front());
  if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kLowVoltage) {
    return false;
  }
  for (const ObjectId span_id : generated_second.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    const wire::core::Bundle* span_bundle = state.view().edit_state().bundles.find(span->bundle_id);
    if (span_bundle == nullptr || span_bundle->bundle_template_id != wire::core::BundleKind::kLowVoltage) {
      return false;
    }
  }
  return true;
}

// Intent: HV template keeps midair-branch policy disabled and rejects unsupported legacy mode values.

bool test_backbone_detail_generation_handles_building_support_node() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec building{};
  building.point_index = 1;
  building.support_kind = wire::core::SupportKind::kBuilding;
  req.path.node_specs.push_back(building);
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);

  const auto generated = state.GenerateFromBackboneSpec(req);
  return generated.ok && !generated.value.generated_span_ids.empty();
}

// Intent: Segment pick near endpoint snaps to node and does not add extra midair support node.

bool test_generate_grouped_line_high_voltage_three_phase() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  BackbonePathGenerateOptions options{};
  options.road.id = 1001;
  options.road.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 2.0, 0.0}, {30.0, 2.0, 0.0}};
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  options.direction_mode = wire::core::PathDirectionMode::kAuto;

  const auto result = generate_from_backbone_options(state, options);
  if (!result.ok) {
    return false;
  }
  if (result.value.pole_ids.size() != options.road.polyline.size()) {
    return false;
  }
  if (result.value.bundle_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const std::size_t expected_spans = (result.value.pole_ids.size() - 1) * 3;
  if (result.value.span_ids.size() != expected_spans) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(result.value.bundle_id);
  if (bundle == nullptr || bundle->conductor_count != 3) {
    return false;
  }
  if (result.value.lane_assignments.size() != result.value.pole_ids.size() - 1) {
    return false;
  }
  for (const auto& assignment : result.value.lane_assignments) {
    if (assignment.port_ids_a.size() != 3 || assignment.port_ids_b.size() != 3) {
      return false;
    }
    if (assignment.slot_ids_a.size() != 3 || assignment.slot_ids_b.size() != 3) {
      return false;
    }
  }
  for (const ObjectId span_id : result.value.span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    if (span->bundle_id != result.value.bundle_id) {
      return false;
    }
  }
  const auto validation = validate_now(state);
  if (!validation.ok()) {
    return false;
  }
  return true;
}

// Intent: Direction mode force flags should deterministically choose path orientation.
bool test_generate_grouped_line_direction_forced_reverse() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  BackbonePathGenerateOptions options{};
  options.road.id = 1002;
  options.road.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  options.direction_mode = wire::core::PathDirectionMode::kReverse;

  const auto reverse = generate_from_backbone_options(state, options);
  if (!reverse.ok || reverse.value.pole_ids.empty()) {
    return false;
  }
  const auto* first_reverse = state.view().edit_state().poles.find(reverse.value.pole_ids.front());
  if (first_reverse == nullptr ||
      !almost_equal(first_reverse->world_transform.position, options.road.polyline.back())) {
    return false;
  }
  if (reverse.value.direction_debug.chosen != wire::core::PathDirectionChosen::kReverse) {
    return false;
  }
  return true;
}

// Intent: Group lane assignment on a U-shaped path should avoid lane-order inversions per segment.
bool test_grouped_line_lane_order_no_inversion_on_u_path() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  BackbonePathGenerateOptions options{};
  options.road.id = 1101;
  options.road.polyline = {
      {-18.0, -6.0, 0.0},
      {-6.0, -6.0, 0.0},
      {-6.0, 6.0, 0.0},
      {6.0, 6.0, 0.0},
      {6.0, -6.0, 0.0},
      {18.0, -6.0, 0.0},
  };
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  options.direction_mode = wire::core::PathDirectionMode::kAuto;

  const auto generated = generate_from_backbone_options(state, options);
  if (!generated.ok) {
    return false;
  }
  const LaneOrderMetrics metrics = compute_lane_order_metrics(state, generated.value.lane_assignments);
  return metrics.y_inversions == 0;
}

// Intent: Group lane assignment on an acute corner path should avoid lane-order inversion.
bool test_grouped_line_acute_corner_no_inversion() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  BackbonePathGenerateOptions options{};
  options.road.id = 1103;
  options.road.polyline = {
      {-20.0, 0.0, 0.0},
      {-8.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {-2.0, 6.0, 0.0},
      {10.0, 6.0, 0.0},
  };
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kCommunication;
  options.bundle_count = 4;
  options.override_allow_mirror = true;
  options.allow_mirror = true;
  options.direction_mode = wire::core::PathDirectionMode::kAuto;

  const auto generated = generate_from_backbone_options(state, options);
  if (!generated.ok) {
    return false;
  }
  const LaneOrderMetrics metrics = compute_lane_order_metrics(state, generated.value.lane_assignments);
  return metrics.y_inversions == 0;
}

// Intent: Acute/zigzag path variants should avoid lane-order inversion under mirror two-choice policy.
bool test_grouped_line_acute_pattern_suite_no_inversion() {
  const std::vector<std::vector<wire::core::Vec3d>> paths = {
      {
          {-20.0, 0.0, 0.0},
          {-8.0, 0.0, 0.0},
          {2.0, 0.0, 0.0},
          {-2.0, 6.0, 0.0},
          {10.0, 6.0, 0.0},
      },
      {
          {-20.0, 0.0, 0.0},
          {-8.0, 0.0, 0.0},
          {2.0, 0.0, 0.0},
          {-2.0, -6.0, 0.0},
          {10.0, -6.0, 0.0},
      },
      {
          {-20.0, 0.0, 0.0},
          {-8.0, 0.0, 0.0},
          {0.0, 6.0, 0.0},
          {8.0, 0.0, 0.0},
          {16.0, 6.0, 0.0},
      },
      {
          {-18.0, -4.0, 0.0},
          {-8.0, -4.0, 0.0},
          {-2.0, 3.0, 0.0},
          {6.0, -3.0, 0.0},
          {14.0, 4.0, 0.0},
      },
  };

  for (std::size_t i = 0; i < paths.size(); ++i) {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return false;
    }
    BackbonePathGenerateOptions options{};
    options.road.id = 1200 + static_cast<wire::core::RoadId>(i);
    options.road.polyline = paths[i];
    options.interval = 0.0;
    options.pole_type_id = type_ids.front();
    options.bundle_template_id = wire::core::BundleKind::kCommunication;
    options.bundle_count = 4;
    options.override_allow_mirror = true;
    options.allow_mirror = true;
    const auto generated = generate_from_backbone_options(state, options);
    if (!generated.ok) {
      return false;
    }
    const LaneOrderMetrics metrics = compute_lane_order_metrics(state, generated.value.lane_assignments);
    if (metrics.y_inversions != 0) {
      std::ostringstream tag;
      tag << "C86_path" << i;
      dump_lane_assignment_debug(state, generated.value.lane_assignments, tag.str().c_str());
      return false;
    }
  }
  return true;
}

// Intent: Acute high-voltage 3-phase path should also suppress lane-order inversion like other bundle types.
bool test_grouped_line_hv3_acute_no_phase_twist() {
  const std::vector<std::vector<wire::core::Vec3d>> paths = {
      {
          {-20.0, 0.0, 0.0},
          {-8.0, 0.0, 0.0},
          {2.0, 0.0, 0.0},
          {-2.0, 6.0, 0.0},
          {10.0, 6.0, 0.0},
      },
      {
          {-20.0, 0.0, 0.0},
          {-8.0, 0.0, 0.0},
          {2.0, 0.0, 0.0},
          {-2.0, -6.0, 0.0},
          {10.0, -6.0, 0.0},
      },
      {
          {-18.0, -4.0, 0.0},
          {-8.0, -4.0, 0.0},
          {-2.0, 3.0, 0.0},
          {6.0, -3.0, 0.0},
          {14.0, 4.0, 0.0},
      },
  };

  for (std::size_t i = 0; i < paths.size(); ++i) {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return false;
    }
    BackbonePathGenerateOptions options{};
    options.road.id = 1300 + static_cast<wire::core::RoadId>(i);
    options.road.polyline = paths[i];
    options.interval = 0.0;
    options.pole_type_id = type_ids.front();
    options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
    options.override_allow_mirror = true;
    options.allow_mirror = true;
    const auto generated = generate_from_backbone_options(state, options);
    if (!generated.ok) {
      return false;
    }
    const LaneOrderMetrics metrics = compute_lane_order_metrics(state, generated.value.lane_assignments);
    if (metrics.y_inversions != 0) {
      return false;
    }
  }
  return true;
}

// Intent: Backbone HV template should allow mirror-two-choice on acute path when it improves continuity.
bool test_backbone_hv3_template_acute_no_phase_twist() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {15.2954, -10.2356, 0.0},
      {2.87944, -2.16948, 0.0},
      {12.0546, -16.1829, 0.0},
      {-2.43956, -5.96523, 0.0},
      {-14.3612, -6.28665, 0.0},
      {-14.1685, 6.36328, 0.0},
      {-2.66804, 5.17324, 0.0},
      {-13.2896, 13.4663, 0.0},
      {-20.9067, 19.9314, 0.0},
      {-24.7745, 14.455, 0.0},
  };
  req.interval_m = 8.0;
  req.pole_type_id = type_ids.front();
  wire::core::BackboneBundleSpec hv{};
  hv.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  req.bundles.push_back(hv);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }
  const auto tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kHighVoltage);
  if (tpl_it == state.view().bundle_templates().end() || !tpl_it->second.allow_mirror) {
    return false;
  }
  const auto& orientations = state.view().last_generation_backbone().edge_orientations;
  if (orientations.empty()) {
    return false;
  }
  for (const auto& orientation : orientations) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    if (orientation.flipped_from_previous) {
      return false;
    }
  }
  return true;
}

// Intent: Captured zigzag DrawPath shape should keep HV3 bundle free of lane-order inversion.
bool test_backbone_hv3_capture_shape_no_inversion() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {19.6087, -16.6408, 0.0},
      {8.22759, -11.9276, 0.0},
      {16.8051, -20.9148, 0.0},
      {8.62249, -16.7209, 0.0},
      {12.2073, -24.74, 0.0},
      {4.69953, -21.4095, 0.0},
  };
  req.interval_m = 1000.0; // clicked points only
  req.pole_type_id = type_ids.front();
  wire::core::BackboneBundleSpec hv{};
  hv.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  req.bundles.push_back(hv);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }
  const auto& orientations = state.view().last_generation_backbone().edge_orientations;
  if (orientations.empty()) {
    return false;
  }
  for (const auto& orientation : orientations) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    if (orientation.flipped_from_previous) {
      return false;
    }
  }
  return true;
}

// Intent: Captured DrawPath shape should keep HV3 adjacent-segment polylines free of XY crossings.
bool test_backbone_hv3_capture_shape_no_adjacent_crossings() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {14.6925, -11.8125, 0.0},
      {0.711913, -17.8685, 0.0},
      {10.1084, -9.46792, 0.0},
      {-4.61348, -8.99765, 0.0},
      {-15.6287, -0.227707, 0.0},
      {-22.195, 6.65811, 0.0},
      {-12.9124, 15.285, 0.0},
      {-37.5524, 7.58507, 0.0},
  };
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  wire::core::BackboneBundleSpec hv{};
  hv.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  req.bundles.push_back(hv);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const auto& assignments = wire::core::CoreStateTestHook::last_lane_assignments(state);
  const int adjacent_crossings = count_bundle_lane_adjacent_xy_intersections(state, assignments);
  if (adjacent_crossings != 0) {
    dump_lane_assignment_debug(state, assignments, "C109_capture_adjacent");
    return false;
  }
  return true;
}

// Intent: ThreePhase group-kind should still permit mirror-two-choice when allow_lane_mirror=true.
bool test_grouped_line_threephase_policy_is_category_agnostic() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  BackbonePathGenerateOptions options{};
  options.road.id = 1308;
  options.road.polyline = {
      {-20.0, 0.0, 0.0},
      {-8.0, 0.0, 0.0},
      {2.0, 0.0, 0.0},
      {-2.0, 6.0, 0.0},
      {10.0, 6.0, 0.0},
  };
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  options.override_allow_mirror = true;
  options.allow_mirror = true;
  const auto generated = generate_from_backbone_options(state, options);
  if (!generated.ok) {
    return false;
  }
  const LaneOrderMetrics metrics = compute_lane_order_metrics(state, generated.value.lane_assignments);
  return metrics.y_inversions == 0;
}

// Intent: Path extension should preserve lane order at boundary pole between existing and newly generated segment.
bool test_backbone_extension_preserves_boundary_lane_order() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  auto make_request = [&](const std::vector<wire::core::Vec3d>& path) {
    wire::core::BackboneSpec req{};
    req.path.polyline = path;
    req.interval_m = 1000.0; // keep clicked vertices only
    req.pole_type_id = type_ids.front();
    wire::core::BackboneBundleSpec hv{};
    hv.bundle_template_id = wire::core::BundleKind::kHighVoltage;
    req.bundles.push_back(hv);
    return req;
  };

  const std::vector<wire::core::Vec3d> base_path = {
      {19.6087, -16.6408, 0.0},
      {8.22759, -11.9276, 0.0},
      {16.8051, -20.9148, 0.0},
      {8.62249, -16.7209, 0.0},
      {12.2073, -24.74, 0.0},
  };
  const std::vector<wire::core::Vec3d> extended_path = {
      {19.6087, -16.6408, 0.0},
      {8.22759, -11.9276, 0.0},
      {16.8051, -20.9148, 0.0},
      {8.62249, -16.7209, 0.0},
      {12.2073, -24.74, 0.0},
      {4.69953, -21.4095, 0.0},
  };

  const auto first = state.GenerateFromBackboneSpec(make_request(base_path));
  if (!first.ok) {
    return false;
  }
  const auto& first_orientations = state.view().last_generation_backbone().edge_orientations;
  if (first_orientations.empty()) {
    return false;
  }
  const auto* tail = &first_orientations.back();

  const auto second = state.GenerateFromBackboneSpec(make_request(extended_path));
  if (!second.ok) {
    return false;
  }
  const auto& second_orientations = state.view().last_generation_backbone().edge_orientations;
  const wire::core::BackboneEdgeOrientation* boundary = nullptr;
  for (const auto& orientation : second_orientations) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    if (orientation.node_a_id == tail->node_a_id && orientation.node_b_id == tail->node_b_id) {
      boundary = &orientation;
      break;
    }
  }
  if (boundary == nullptr) {
    for (const auto& orientation : second_orientations) {
      if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
        continue;
      }
      if (orientation.flipped_from_previous) {
        return false;
      }
    }
    return !second_orientations.empty();
  }
  return boundary->orientation == tail->orientation;
}

// Intent: Enabling lane mirror must not worsen grouped-lane quality metrics (Y/Z order and layer continuity).
bool test_grouped_line_mirror_metric_non_regression() {
  const auto run_with_mirror = [](bool allow_mirror) -> std::pair<bool, LaneOrderMetrics> {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return {false, {}};
    }
    BackbonePathGenerateOptions options{};
    options.road.id = 1102;
    options.road.polyline = {
        {-20.0, 0.0, 0.0},
        {-8.0, 0.0, 0.0},
        {-8.0, 10.0, 0.0},
        {8.0, 10.0, 0.0},
        {8.0, -2.0, 0.0},
        {20.0, -2.0, 0.0},
    };
    options.interval = 0.0;
    options.pole_type_id = type_ids.front();
    options.bundle_template_id = wire::core::BundleKind::kCommunication;
    options.bundle_count = 4;
    options.override_allow_mirror = true;
    options.allow_mirror = allow_mirror;
    options.direction_mode = wire::core::PathDirectionMode::kAuto;

    const auto generated = generate_from_backbone_options(state, options);
    if (!generated.ok) {
      return {false, {}};
    }
    return {true, compute_lane_order_metrics(state, generated.value.lane_assignments)};
  };

  const auto without_mirror = run_with_mirror(false);
  const auto with_mirror = run_with_mirror(true);
  if (!without_mirror.first || !with_mirror.first) {
    return false;
  }

  return with_mirror.second.weighted_score() <= without_mirror.second.weighted_score() &&
         with_mirror.second.y_inversions <= without_mirror.second.y_inversions &&
         with_mirror.second.z_inversions <= without_mirror.second.z_inversions;
}

// Intent: In T-junction, earlier DrawPath session should keep order=0 primary incident.

// Intent: Grouped mirror handling should keep lane ordering monotonic (identity/reverse only).
bool test_grouped_lane_mirror_is_two_choice_only() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  BackbonePathGenerateOptions options{};
  options.road.id = 1199;
  options.road.polyline = {
      {-20.0, 0.0, 0.0},
      {-6.0, 0.0, 0.0},
      {-6.0, 8.0, 0.0},
      {8.0, 8.0, 0.0},
      {8.0, -2.0, 0.0},
      {20.0, -2.0, 0.0},
  };
  options.interval = 0.0;
  options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kCommunication;
  options.bundle_count = 4;
  options.override_allow_mirror = true;
  options.allow_mirror = true;

  const auto generated = generate_from_backbone_options(state, options);
  if (!generated.ok) {
    return false;
  }

  for (const auto& assignment : generated.value.lane_assignments) {
    const auto* pole_b = state.view().edit_state().poles.find(assignment.pole_b_id);
    if (pole_b == nullptr) {
      return false;
    }
    std::vector<double> y_values{};
    for (ObjectId port_id : assignment.port_ids_b) {
      const auto* port = state.view().edit_state().ports.find(port_id);
      if (port == nullptr) {
        return false;
      }
      y_values.push_back(to_local_on_pole_test(*pole_b, port->world_position).y);
    }
    if (!is_monotonic(y_values)) {
      return false;
    }
  }
  return true;
}

bool test_generate_from_backbone_spec_basic_hv_default_lanes() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 4.0, 0.0}};
  req.interval_m = 6.0;
  req.pole_type_id = type_ids.front();
  req.direction_mode = wire::core::PathDirectionMode::kAuto;
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);

  const auto result = state.GenerateFromBackboneSpec(req);
  if (!result.ok) {
    return false;
  }
  if (result.value.bundle_id == wire::core::kInvalidObjectId) {
    return false;
  }
  if (result.value.generated_pole_ids.size() < 2 || result.value.generated_span_ids.empty()) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(result.value.bundle_id);
  if (bundle == nullptr || bundle->conductor_count != 3) {
    return false;
  }
  for (const ObjectId span_id : result.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr || span->bundle_id != result.value.bundle_id) {
      return false;
    }
  }
  return validate_now(state).ok();
}

// Intent: Direction modes on path generation should all execute without failure.
bool test_generate_from_backbone_spec_direction_modes_nonfailing() {
  const std::array<wire::core::PathDirectionMode, 3> modes = {
      wire::core::PathDirectionMode::kForward,
      wire::core::PathDirectionMode::kReverse,
      wire::core::PathDirectionMode::kAuto,
  };
  for (const auto mode : modes) {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return false;
    }
    wire::core::BackboneSpec req{};
    req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 2.0, 0.0}};
    req.interval_m = 5.0;
    req.pole_type_id = type_ids.front();
    req.direction_mode = mode;
    add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
    const auto result = state.GenerateFromBackboneSpec(req);
    if (!result.ok || result.value.generated_span_ids.empty()) {
      return false;
    }
  }
  return true;
}

// Intent: Backbone generation should reject invalid input and keep state recoverable.
bool test_generate_from_backbone_spec_invalid_inputs_fail() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const CoreCounts before = snapshot_counts(state);

  wire::core::BackboneSpec too_short{};
  too_short.path.polyline = {{0.0, 0.0, 0.0}};
  too_short.interval_m = 5.0;
  too_short.pole_type_id = type_ids.front();
  add_backbone_bundle(too_short, wire::core::BundleKind::kLowVoltage);
  const auto r_short = state.GenerateFromBackboneSpec(too_short);
  if (r_short.ok || !regex_contains(r_short.error, "at least 2 points")) {
    return false;
  }

  wire::core::BackboneSpec bad_interval{};
  bad_interval.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  bad_interval.interval_m = 0.0;
  bad_interval.pole_type_id = type_ids.front();
  add_backbone_bundle(bad_interval, wire::core::BundleKind::kLowVoltage);
  const auto r_interval = state.GenerateFromBackboneSpec(bad_interval);
  if (r_interval.ok || !regex_contains(r_interval.error, "interval_m")) {
    return false;
  }

  wire::core::BackboneSpec bad_template{};
  bad_template.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  bad_template.interval_m = 5.0;
  bad_template.pole_type_id = type_ids.front();
  add_backbone_bundle(bad_template, static_cast<wire::core::BundleKind>(255));
  const auto r_category = state.GenerateFromBackboneSpec(bad_template);
  if (r_category.ok || !regex_contains(r_category.error, "template")) {
    return false;
  }

  wire::core::BackboneSpec bad_type{};
  bad_type.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  bad_type.interval_m = 5.0;
  bad_type.pole_type_id = 999999;
  add_backbone_bundle(bad_type, wire::core::BundleKind::kLowVoltage);
  const auto r_type = state.GenerateFromBackboneSpec(bad_type);
  if (r_type.ok || !regex_contains(r_type.error, "pole type")) {
    return false;
  }

  if (!same_counts(before, snapshot_counts(state))) {
    return false;
  }

  wire::core::BackboneSpec recover{};
  recover.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 0.0, 0.0}};
  recover.interval_m = 6.0;
  recover.pole_type_id = type_ids.front();
  add_backbone_bundle(recover, wire::core::BundleKind::kLowVoltage);
  return state.GenerateFromBackboneSpec(recover).ok;
}

bool test_backbone_reused_junction_pole_keeps_main_chain_forward() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  const auto main_generated = state.GenerateFromBackboneSpec(main_req);
  if (!main_generated.ok) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (shared.node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok) {
    return false;
  }

  const auto* center = state.view().edit_state().poles.find(shared.node_id);
  if (center == nullptr) {
    return false;
  }
  const double yaw = effective_pole_yaw_deg_test(*center);
  const bool horizontal =
      std::min(angle_diff_abs_deg(yaw, 0.0), angle_diff_abs_deg(yaw, 180.0)) <= 1e-6;
  const auto it_debug = state.view().pole_orientation_debug_records().find(shared.node_id);
  return horizontal && it_debug != state.view().pole_orientation_debug_records().end() &&
         it_debug->second.rule == wire::core::PoleForwardRule::kMainChainBisector;
}

bool test_backbone_branch_bundle_uses_branch_support_ports() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  const auto main_generated = state.GenerateFromBackboneSpec(main_req);
  if (!main_generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.empty()) {
    return false;
  }

  bool has_branch_support_port = false;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    const bool center_is_a = span->endpoint_node_a_id == center_id;
    const bool center_is_b = span->endpoint_node_b_id == center_id;
    if (!center_is_a && !center_is_b) {
      continue;
    }
    const ObjectId port_id = center_is_a ? span->port_a_id : span->port_b_id;
    const auto* port = state.view().edit_state().ports.find(port_id);
    if (port == nullptr) {
      return false;
    }
    if (port->placement_source == wire::core::PortPlacementSourceKind::kBranchSupport) {
      has_branch_support_port = true;
    }
  }
  if (!has_branch_support_port) {
    return false;
  }
  const auto& assignments = state.view().last_lane_assignments();
  return !assignments.empty() &&
         std::all_of(assignments.begin(), assignments.end(), [](const wire::core::SegmentLaneAssignment& assignment) {
           return assignment.flow_kind == wire::core::BackboneFlowKind::kBranch &&
                  assignment.uses_branch_support && assignment.branch_down_offset_m > 0.0;
         });
}

bool test_backbone_branch_support_offsets_height_without_changing_layer() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  const auto main_generated = state.GenerateFromBackboneSpec(main_req);
  if (!main_generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::vector<const wire::core::Port*> main_ports{};
  for (ObjectId span_id : main_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    if (span->endpoint_node_a_id == center_id) {
      if (const auto* port = state.view().edit_state().ports.find(span->port_a_id); port != nullptr) {
        main_ports.push_back(port);
      }
    }
    if (span->endpoint_node_b_id == center_id) {
      if (const auto* port = state.view().edit_state().ports.find(span->port_b_id); port != nullptr) {
        main_ports.push_back(port);
      }
    }
  }
  if (main_ports.empty()) {
    return false;
  }
  double main_min_z = std::numeric_limits<double>::infinity();
  int main_layer = -1;
  for (const auto* port : main_ports) {
    main_min_z = std::min(main_min_z, port->world_position.z);
    if (main_layer < 0) {
      main_layer = port->template_layer;
    }
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.empty()) {
    return false;
  }

  const wire::core::Port* branch_port = nullptr;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    if (span->endpoint_node_a_id == center_id) {
      branch_port = state.view().edit_state().ports.find(span->port_a_id);
      break;
    }
    if (span->endpoint_node_b_id == center_id) {
      branch_port = state.view().edit_state().ports.find(span->port_b_id);
      break;
    }
  }
  return branch_port != nullptr && branch_port->placement_source == wire::core::PortPlacementSourceKind::kBranchSupport &&
         branch_port->world_position.z + 1e-6 < main_min_z && branch_port->template_layer == main_layer;
}

bool test_backbone_branch_generation_preserves_existing_hv3_main_ports() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  const auto main_generated = state.GenerateFromBackboneSpec(main_req);
  if (!main_generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::vector<ObjectId> before_center_ports{};
  for (ObjectId span_id : main_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    if (span->endpoint_node_a_id == center_id) {
      before_center_ports.push_back(span->port_a_id);
    }
    if (span->endpoint_node_b_id == center_id) {
      before_center_ports.push_back(span->port_b_id);
    }
  }
  std::sort(before_center_ports.begin(), before_center_ports.end());
  before_center_ports.erase(std::unique(before_center_ports.begin(), before_center_ports.end()), before_center_ports.end());
  if (before_center_ports.size() != 3) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok) {
    return false;
  }

  std::vector<ObjectId> after_center_ports{};
  for (ObjectId span_id : main_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    if (span->endpoint_node_a_id == center_id) {
      after_center_ports.push_back(span->port_a_id);
    }
    if (span->endpoint_node_b_id == center_id) {
      after_center_ports.push_back(span->port_b_id);
    }
  }
  std::sort(after_center_ports.begin(), after_center_ports.end());
  after_center_ports.erase(std::unique(after_center_ports.begin(), after_center_ports.end()), after_center_ports.end());
  return before_center_ports == after_center_ports;
}

bool test_backbone_mixed_route_uses_edge_level_flow_classification() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    return false;
  }

  const ObjectId left_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (left_id == wire::core::kInvalidObjectId || center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec mixed_req{};
  mixed_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  mixed_req.interval_m = 1000.0;
  mixed_req.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec left_spec{};
  left_spec.point_index = 0;
  left_spec.support_kind = wire::core::SupportKind::kPole;
  left_spec.node_id = left_id;
  mixed_req.path.node_specs.push_back(left_spec);
  wire::core::BackboneInputSpec::NodeSpec center_spec{};
  center_spec.point_index = 1;
  center_spec.support_kind = wire::core::SupportKind::kPole;
  center_spec.node_id = center_id;
  mixed_req.path.node_specs.push_back(center_spec);
  add_backbone_bundle(mixed_req, wire::core::BundleKind::kLowVoltage);
  const auto mixed_generated = state.GenerateFromBackboneSpec(mixed_req);
  if (!mixed_generated.ok) {
    return false;
  }

  const auto& assignments = state.view().last_lane_assignments();
  if (assignments.size() != 2) {
    return false;
  }
  return assignments[0].flow_kind == wire::core::BackboneFlowKind::kMain &&
         assignments[0].flow_decision_rule == wire::core::BackboneFlowDecisionRule::kExistingChainMain &&
         !assignments[0].uses_branch_support &&
         assignments[1].flow_kind == wire::core::BackboneFlowKind::kBranch &&
         assignments[1].flow_decision_rule == wire::core::BackboneFlowDecisionRule::kExistingChainBranch &&
         assignments[1].uses_branch_support;
}

bool test_backbone_branch_support_visual_cache_contains_support_placement() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);

  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto* visual = state.view().find_span_visual_cache(span_id);
    if (visual == nullptr || visual->branch_supports.empty()) {
      continue;
    }
    return std::any_of(visual->branch_supports.begin(), visual->branch_supports.end(),
                       [center_id](const wire::core::BranchSupportPlacement& placement) {
                         return placement.owner_pole_id == center_id && placement.down_offset_m > 0.0 &&
                                placement.side != wire::core::SlotSide::kCenter;
                       });
  }
  return false;
}

bool test_backbone_near_straight_branch_still_classifies_as_branch() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch_req{};
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {11.5, 0.8, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch_req.path.node_specs.push_back(shared);
  branch_req.interval_m = 1000.0;
  branch_req.pole_type_id = type_ids.front();
  add_backbone_bundle(branch_req, wire::core::BundleKind::kLowVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch_req);
  if (!branch_generated.ok) {
    return false;
  }

  const auto& assignments = state.view().last_lane_assignments();
  return assignments.size() == 1 && assignments.front().flow_kind == wire::core::BackboneFlowKind::kBranch &&
         assignments.front().flow_decision_rule == wire::core::BackboneFlowDecisionRule::kExistingChainBranch;
}

bool test_backbone_new_chain_uses_fallback_orientation_without_existing_main_context() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto it_debug = state.view().pole_orientation_debug_records().find(center_id);
  return it_debug != state.view().pole_orientation_debug_records().end() &&
         it_debug->second.rule == wire::core::PoleForwardRule::kFallback;
}

bool test_variation_settings_do_not_change_topology_flow_or_mirror() {
  auto generate_assignments = [](std::uint64_t seed) {
    CoreState state;
    wire::core::VariationSettings variation = state.view().variation_settings();
    variation.enabled = true;
    variation.global_seed = seed;
    variation.sag_variation_scale = 0.35;
    variation.branch_down_offset_variation_scale = 0.10;
    if (!state.UpdateVariationSettings(variation, true).ok) {
      return std::vector<wire::core::SegmentLaneAssignment>{};
    }

    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return std::vector<wire::core::SegmentLaneAssignment>{};
    }

    wire::core::BackboneSpec trunk{};
    trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
    trunk.interval_m = 1000.0;
    trunk.pole_type_id = type_ids.front();
    add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
    if (!state.GenerateFromBackboneSpec(trunk).ok) {
      return std::vector<wire::core::SegmentLaneAssignment>{};
    }

    const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
    if (center_id == wire::core::kInvalidObjectId) {
      return std::vector<wire::core::SegmentLaneAssignment>{};
    }

    wire::core::BackboneSpec branch{};
    branch.path.polyline = {{0.0, 0.0, 0.0}, {9.0, 6.0, 0.0}};
    wire::core::BackboneInputSpec::NodeSpec shared{};
    shared.point_index = 0;
    shared.support_kind = wire::core::SupportKind::kPole;
    shared.node_id = center_id;
    branch.path.node_specs.push_back(shared);
    branch.interval_m = 1000.0;
    branch.pole_type_id = type_ids.front();
    add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
    if (!state.GenerateFromBackboneSpec(branch).ok) {
      return std::vector<wire::core::SegmentLaneAssignment>{};
    }
    return state.view().last_lane_assignments();
  };

  const auto a = generate_assignments(1001);
  const auto b = generate_assignments(2002);
  if (a.empty() || a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i].flow_kind != b[i].flow_kind || a[i].flow_decision_rule != b[i].flow_decision_rule ||
        a[i].mirrored != b[i].mirrored || a[i].flipped_from_previous != b[i].flipped_from_previous ||
        a[i].variation_flow_key != b[i].variation_flow_key) {
      return false;
    }
  }
  return true;
}

// Intent: Backbone generation must require bundles[] and reject legacy-only fields.
namespace {

void register_generation_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C38_GroupedLine_HV3", "Grouped line generation creates 3-lane high-voltage spans",
                         "Invariant", false, test_generate_grouped_line_high_voltage_three_phase);
  test_registry::AddTest(tests, "C39_Direction_ForcedReverse", "Grouped line honors forced reverse direction", "Exact",
                         false, test_generate_grouped_line_direction_forced_reverse);
  test_registry::AddTest(tests, "C47_Backbone_HVDefaultLanes",
                         "Backbone generation creates default HV bundle spans", "Invariant", false,
                         test_generate_from_backbone_spec_basic_hv_default_lanes);
  test_registry::AddTest(tests, "C48_Backbone_DirectionModes",
                         "Backbone generation supports Forward/Reverse/Auto direction modes", "Invariant", false,
                         test_generate_from_backbone_spec_direction_modes_nonfailing);
  test_registry::AddTest(tests, "C49_Backbone_InvalidInputs",
                         "Backbone generation rejects invalid input and remains recoverable", "Exact", true,
                         test_generate_from_backbone_spec_invalid_inputs_fail);
  test_registry::AddTest(tests, "C62_GroupedLine_NoInversionUPath",
                         "U-shaped grouped path keeps conductor order inversion-free", "Invariant", false,
                         test_grouped_line_lane_order_no_inversion_on_u_path);
  test_registry::AddTest(tests, "C63_GroupedLine_MirrorMetricNonRegression",
                         "Mirror enablement does not worsen grouped ordering metrics", "Invariant", false,
                         test_grouped_line_mirror_metric_non_regression);
  test_registry::AddTest(tests, "C76_GroupedLine_AcuteCornerNoInversion",
                         "Acute corner grouped path keeps conductor order stable", "Invariant", false,
                         test_grouped_line_acute_corner_no_inversion);
  test_registry::AddTest(tests, "C85_GroupedLine_TwoChoiceMirror",
                         "Grouped mirror handling stays within the two-choice model", "Invariant", false,
                         test_grouped_lane_mirror_is_two_choice_only);
  test_registry::AddTest(tests, "C86_GroupedLine_AcutePatternSuite",
                         "Acute pattern suite stays inversion-free", "Invariant", false,
                         test_grouped_line_acute_pattern_suite_no_inversion);
  test_registry::AddTest(tests, "C87_GroupedLine_HV3AcuteNoTwist",
                         "HV3 acute grouped path avoids phase twist", "Invariant", false,
                         test_grouped_line_hv3_acute_no_phase_twist);
  test_registry::AddTest(tests, "C88_Backbone_HV3TemplateAcuteNoTwist",
                         "Backbone HV3 acute path avoids phase twist under template generation", "Invariant", false,
                         test_backbone_hv3_template_acute_no_phase_twist);
  test_registry::AddTest(tests, "C89_ThreePhasePolicy_CategoryAgnostic",
                         "Three-phase policy remains category agnostic", "Invariant", false,
                         test_grouped_line_threephase_policy_is_category_agnostic);
  test_registry::AddTest(tests, "C98_Backbone_ExtensionBoundaryOrder",
                         "Backbone extension preserves boundary conductor ordering", "Invariant", false,
                         test_backbone_extension_preserves_boundary_lane_order);
  test_registry::AddTest(tests, "C99_Backbone_HV3CaptureNoInversion",
                         "Captured HV3 backbone shape stays inversion-free", "Invariant", false,
                         test_backbone_hv3_capture_shape_no_inversion);
  test_registry::AddTest(tests, "C100_Backbone_MidairSupportNode",
                         "Backbone generation keeps explicit Midair support nodes", "Invariant", false,
                         test_backbone_generation_includes_midair_support_nodes);
  test_registry::AddTest(tests, "C103_Backbone_BuildingSupportDetailStable",
                         "Detail generation remains stable with building support nodes", "Invariant", false,
                         test_backbone_detail_generation_handles_building_support_node);
  test_registry::AddTest(tests, "C109_Backbone_HV3CaptureNoAdjacentCrossings",
                         "Captured HV3 backbone shape avoids adjacent XY crossings", "Invariant", false,
                         test_backbone_hv3_capture_shape_no_adjacent_crossings);
  test_registry::AddTest(tests, "C110_Backbone_ReuseExplicitPoleNode",
                         "Backbone generation reuses explicitly picked pole nodes", "Invariant", false,
                         test_backbone_generation_reuses_explicit_pole_node_id);
  test_registry::AddTest(tests, "C111_Backbone_ReuseExplicitSupportNode",
                         "Backbone generation reuses explicitly picked non-pole support nodes", "Invariant", false,
                         test_backbone_generation_reuses_explicit_support_node_id);
  test_registry::AddTest(tests, "C112_Backbone_MidairExtensionDetailChain",
                         "Midair extension still produces detail poles and spans", "Invariant", false,
                         test_backbone_midair_extension_generates_detail_chain);
  test_registry::AddTest(tests, "C113_Backbone_MidairExtensionFirstSupportSegment",
                         "Midair extension includes the first support-to-detail segment in the route",
                         "Invariant", false, test_backbone_midair_extension_includes_first_support_segment);
  test_registry::AddTest(tests, "C114_Backbone_MidairBranchUsesSourceSpanHeight",
                         "Midair branch keeps backbone pick at abstract height while detail starts from source span height",
                         "Invariant", false, test_backbone_midair_branch_reuses_source_span_height);
  test_registry::AddTest(tests, "C115_Backbone_MidairSingleClickNoExtraBridge",
                         "Midair extension from one clicked endpoint keeps a single direct segment", "Invariant", false,
                         test_backbone_midair_extension_single_click_stays_single_segment);
  test_registry::AddTest(tests, "C116_Backbone_TemplateSkipsMidairBranchGeneration",
                         "Disallowed template skips source-edge midair branch generation without failing request",
                         "Invariant", false, test_backbone_midair_branch_skips_disallowed_template_generation);
  test_registry::AddTest(tests, "C118_Backbone_MidairBranchMixedTemplates",
                         "Midair branch generation keeps only templates that allow midair branch", "Invariant", false,
                         test_backbone_midair_branch_generates_only_allowed_templates);
  test_registry::AddTest(tests, "C133_Backbone_MainChainPoleForward",
                         "Reused junction pole keeps forward aligned to the existing main chain", "Invariant", false,
                         test_backbone_reused_junction_pole_keeps_main_chain_forward);
  test_registry::AddTest(tests, "C134_Backbone_BranchSupportPorts",
                         "Branch bundle uses dedicated branch-support ports and debug classification", "Invariant",
                         false, test_backbone_branch_bundle_uses_branch_support_ports);
  test_registry::AddTest(tests, "C135_Backbone_BranchSupportDownOffset",
                         "Branch support lowers attachment height without rewriting layer semantics", "Invariant",
                         false, test_backbone_branch_support_offsets_height_without_changing_layer);
  test_registry::AddTest(tests, "C136_Backbone_HV3MainPortsStableAfterBranch",
                         "Adding an HV3 branch keeps existing main-chain ports stable", "Invariant", false,
                         test_backbone_branch_generation_preserves_existing_hv3_main_ports);
  test_registry::AddTest(tests, "C138_Backbone_MixedRouteEdgeFlow",
                         "Mixed route classifies main and branch per edge instead of one route-level flow",
                         "Invariant", false, test_backbone_mixed_route_uses_edge_level_flow_classification);
  test_registry::AddTest(tests, "C139_Backbone_BranchSupportVisualPlacement",
                         "Branch support placement is derived into visual cache for minimal support structure",
                         "Invariant", false, test_backbone_branch_support_visual_cache_contains_support_placement);
  test_registry::AddTest(tests, "C140_Backbone_BranchClassificationIgnoresNearStraightAngle",
                         "Existing-chain branch stays branch even when geometry is nearly straight", "Invariant",
                         false, test_backbone_near_straight_branch_still_classifies_as_branch);
  test_registry::AddTest(tests, "C150_Backbone_NewChainOrientationFallback",
                         "Poles without existing chain or primary context use the explicit fallback orientation rule",
                         "Invariant", false, test_backbone_new_chain_uses_fallback_orientation_without_existing_main_context);
  test_registry::AddTest(tests, "C155_Variation_DoesNotAffectTopologyOrMirror",
                         "Variation settings do not change deterministic flow classification or mirror decisions",
                         "Invariant", false, test_variation_settings_do_not_change_topology_flow_or_mirror);
}

WIRE_REGISTER_TEST_SUITE(register_generation_tests);

} // namespace
