#include <array>
#include <cmath>
#include <iostream>
#include <sstream>
#include <unordered_set>
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

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage};
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

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage, wire::core::BundleKind::kHighVoltage};
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

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage, wire::core::BundleKind::kHighVoltage};
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
  if (metrics.y_inversions != 0) {
    dump_lane_assignment_debug(state, generated.value.lane_assignments, "C62_upath");
  }
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
      std::ostringstream tag;
      tag << "C87_path" << i;
      dump_lane_assignment_debug(state, generated.value.lane_assignments, tag.str().c_str());
      return false;
    }
  }
  return true;
}

// Intent: Backbone HV template path should expose support-axis/layout-yaw decisions on all route poles.
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
    std::cerr << "[DBG] C88 generate_failed\n";
    return false;
  }
  const auto tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kHighVoltage);
  if (tpl_it == state.view().bundle_templates().end() || !tpl_it->second.allow_mirror) {
    std::cerr << "[DBG] C88 template_missing_or_mirror_disabled\n";
    return false;
  }
  const auto& orientations = state.view().last_generation_backbone().edge_orientations;
  if (orientations.empty()) {
    std::cerr << "[DBG] C88 orientations_empty\n";
    return false;
  }
  const auto& assignments = state.view().last_lane_assignments();
  if (assignments.empty()) {
    std::cerr << "[DBG] C88 assignments_empty\n";
    return false;
  }
  std::unordered_set<wire::core::ObjectId> route_poles{};
  for (const auto& orientation : orientations) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    route_poles.insert(orientation.node_a_id);
    route_poles.insert(orientation.node_b_id);
  }
  if (route_poles.empty()) {
    std::cerr << "[DBG] C88 route_poles_empty\n";
    return false;
  }
  for (wire::core::ObjectId pole_id : route_poles) {
    const auto pole_view = state.view().inspect_pole(pole_id);
    if (!pole_view.has_value() || !pole_view->has_support_axis || !pole_view->has_layout_yaw) {
      std::cerr << "[DBG] C88 pole_trace_missing pole=" << pole_id << "\n";
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
    std::cerr << "[DBG] C99 generate_failed\n";
    return false;
  }
  const auto& orientations = state.view().last_generation_backbone().edge_orientations;
  if (orientations.empty()) {
    std::cerr << "[DBG] C99 orientations_empty\n";
    return false;
  }
  for (const auto& orientation : orientations) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    if (orientation.flipped_from_previous) {
      std::cerr << "[DBG] C99 flipped edge=" << orientation.node_a_id << "->" << orientation.node_b_id
                << " orientation=" << static_cast<int>(orientation.orientation) << " turn=" << orientation.turn_angle_deg
                << " flow=" << static_cast<int>(orientation.flow_kind) << "\n";
      return false;
    }
  }
  return true;
}

// Intent: Captured DrawPath shape should keep interior HV3 shared-pole lane order continuous.
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
  const int adjacent_discontinuities = count_bundle_lane_adjacent_order_discontinuities(state, assignments);
  if (adjacent_discontinuities != 0) {
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
  if (metrics.y_inversions != 0) {
    dump_lane_assignment_debug(state, generated.value.lane_assignments, "C89_hv_agnostic");
  }
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

// Intent: Interval-driven extension should preserve boundary lane order without relying on stale cached assignments.
bool test_backbone_interval_extension_preserves_boundary_lane_order() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  auto make_request = [&](const std::vector<wire::core::Vec3d>& path) {
    wire::core::BackboneSpec req{};
    req.path.polyline = path;
    req.interval_m = 8.0;
    req.pole_type_id = type_ids.front();
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
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
  const auto first_assignments = wire::core::CoreStateTestHook::last_lane_assignments(state);
  if (count_bundle_lane_adjacent_order_discontinuities(state, first_assignments) != 0) {
    return false;
  }

  const auto second = state.GenerateFromBackboneSpec(make_request(extended_path));
  if (!second.ok) {
    return false;
  }
  const auto second_assignments = wire::core::CoreStateTestHook::last_lane_assignments(state);
  const LaneOrderMetrics second_metrics = compute_lane_order_metrics(state, second_assignments);
  const int second_adjacent_discontinuities = count_bundle_lane_adjacent_order_discontinuities(state, second_assignments);
  if (second_metrics.y_inversions != 0 || second_adjacent_discontinuities != 0) {
    dump_lane_assignment_debug(state, second_assignments, "C208_interval_extension");
    return false;
  }
  return true;
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
    std::cerr << "[DBG] C174 center_missing_after_horizontal\n";
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

bool test_backbone_branch_support_lowers_hv3_bundle_uniformly() {
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
  double main_min_z = std::numeric_limits<double>::infinity();
  for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
    if (port.owner_pole_id != center_id || port.layer != wire::core::PortLayer::kHighVoltage) {
      continue;
    }
    main_min_z = std::min(main_min_z, port.world_position.z);
  }
  if (!std::isfinite(main_min_z)) {
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
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.size() != 3) {
    return false;
  }

  std::vector<const wire::core::Port*> branch_ports{};
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      return false;
    }
    if (span->endpoint_node_a_id == center_id) {
      branch_ports.push_back(state.view().edit_state().ports.find(span->port_a_id));
    } else if (span->endpoint_node_b_id == center_id) {
      branch_ports.push_back(state.view().edit_state().ports.find(span->port_b_id));
    }
  }
  if (branch_ports.size() != 3 ||
      std::any_of(branch_ports.begin(), branch_ports.end(), [](const wire::core::Port* p) { return p == nullptr; })) {
    return false;
  }

  const ObjectId branch_tip_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  if (branch_tip_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto* center_pole = state.view().edit_state().poles.find(center_id);
  const auto* branch_tip_pole = state.view().edit_state().poles.find(branch_tip_id);
  if (center_pole == nullptr || branch_tip_pole == nullptr) {
    return false;
  }
  const wire::core::Vec3d branch_dir =
      helpers::normalize_xy_safe(branch_tip_pole->world_transform.position - center_pole->world_transform.position);
  const wire::core::Vec3d row_axis{-branch_dir.y, branch_dir.x, 0.0};
  std::sort(branch_ports.begin(), branch_ports.end(), [&](const wire::core::Port* a, const wire::core::Port* b) {
    const double da = helpers::dot_xy(a->world_position - center_pole->world_transform.position, row_axis);
    const double db = helpers::dot_xy(b->world_position - center_pole->world_transform.position, row_axis);
    return da < db;
  });

  const double z0 = branch_ports[0]->world_position.z;
  const double z1 = branch_ports[1]->world_position.z;
  const double z2 = branch_ports[2]->world_position.z;
  const double min_z = std::min({z0, z1, z2});
  const double max_z = std::max({z0, z1, z2});
  const double x_span =
      std::max({branch_ports[0]->world_position.x, branch_ports[1]->world_position.x, branch_ports[2]->world_position.x}) -
      std::min({branch_ports[0]->world_position.x, branch_ports[1]->world_position.x, branch_ports[2]->world_position.x});
  const double y_span =
      std::max({branch_ports[0]->world_position.y, branch_ports[1]->world_position.y, branch_ports[2]->world_position.y}) -
      std::min({branch_ports[0]->world_position.y, branch_ports[1]->world_position.y, branch_ports[2]->world_position.y});
  const auto& assignments = state.view().last_lane_assignments();
  const auto it_branch = std::find_if(assignments.begin(), assignments.end(),
                                      [&](const wire::core::SegmentLaneAssignment& assignment) {
                                        if (assignment.flow_kind != wire::core::BackboneFlowKind::kBranch ||
                                            !assignment.uses_branch_support) {
                                          return false;
                                        }
                                        const wire::core::Bundle* bundle =
                                            state.view().edit_state().bundles.find(assignment.bundle_id);
                                        return bundle != nullptr &&
                                               bundle->bundle_template_id == wire::core::BundleKind::kHighVoltage;
                                      });
  const double expected_down_offset = 0.275;
  const bool has_expected_policy =
      it_branch != assignments.end() && std::abs(it_branch->branch_down_offset_m - expected_down_offset) <= 1e-9;
  const bool perpendicular_row = x_span > y_span * 2.0;
  const bool uniform_height = (max_z - min_z) <= 1e-6;
  if (!uniform_height || !has_expected_policy) {
    std::cerr << "[DBG] C193 z0=" << z0 << " z1=" << z1 << " z2=" << z2 << " zSpan=" << (max_z - min_z)
              << " xSpan=" << x_span << " ySpan=" << y_span
              << " expectedDown=" << expected_down_offset
              << " actualDown=" << (it_branch != assignments.end() ? it_branch->branch_down_offset_m : -1.0) << "\n";
    }
  return uniform_height && perpendicular_row && max_z + 1e-6 < main_min_z && has_expected_policy;
}

bool test_backbone_branch_support_stays_local_to_root_pole() {
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
  branch_req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}, {0.0, 24.0, 0.0}};
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

  const ObjectId mid_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  if (mid_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto& assignments = state.view().last_lane_assignments();
  const wire::core::SegmentLaneAssignment* root_assignment = nullptr;
  const wire::core::SegmentLaneAssignment* downstream_assignment = nullptr;
  for (const auto& assignment : assignments) {
    const bool touches_center = assignment.pole_a_id == center_id || assignment.pole_b_id == center_id;
    const bool touches_mid = assignment.pole_a_id == mid_id || assignment.pole_b_id == mid_id;
    if (touches_center && touches_mid) {
      root_assignment = &assignment;
    } else if (!touches_center && touches_mid) {
      downstream_assignment = &assignment;
    }
  }
  if (root_assignment == nullptr || downstream_assignment == nullptr) {
    return false;
  }
  if (!root_assignment->uses_branch_support || downstream_assignment->uses_branch_support) {
    std::cerr << "[DBG] C195 rootBranchSupport=" << (root_assignment->uses_branch_support ? 1 : 0)
              << " downstreamBranchSupport=" << (downstream_assignment->uses_branch_support ? 1 : 0) << "\n";
    return false;
  }

  std::vector<const wire::core::Port*> downstream_mid_ports{};
  const auto& downstream_mid_ids =
      (downstream_assignment->pole_a_id == mid_id) ? downstream_assignment->port_ids_a : downstream_assignment->port_ids_b;
  for (ObjectId port_id : downstream_mid_ids) {
    const auto* port = state.view().edit_state().ports.find(port_id);
    if (port == nullptr) {
      return false;
    }
    downstream_mid_ports.push_back(port);
  }
  if (downstream_mid_ports.size() != 3) {
    return false;
  }

  double min_z = std::numeric_limits<double>::infinity();
  double max_z = -std::numeric_limits<double>::infinity();
  double min_x = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const auto* port : downstream_mid_ports) {
    min_z = std::min(min_z, port->world_position.z);
    max_z = std::max(max_z, port->world_position.z);
    min_x = std::min(min_x, port->world_position.x);
    max_x = std::max(max_x, port->world_position.x);
    min_y = std::min(min_y, port->world_position.y);
    max_y = std::max(max_y, port->world_position.y);
    if (port->placement_source == wire::core::PortPlacementSourceKind::kBranchSupport) {
      std::cerr << "[DBG] C195 downstream port still branch support id=" << port->id << "\n";
      return false;
    }
  }
  const bool flat_height = (max_z - min_z) <= 1e-6;
  const bool perpendicular_row = (max_x - min_x) > (max_y - min_y) * 2.0;
  if (!flat_height || !perpendicular_row) {
    std::cerr << "[DBG] C195 zSpan=" << (max_z - min_z) << " xSpan=" << (max_x - min_x)
              << " ySpan=" << (max_y - min_y) << "\n";
  }
  return flat_height && perpendicular_row;
}

bool test_backbone_branch_support_visual_stays_perpendicular_to_branch() {
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
  const auto* center_pole = state.view().edit_state().poles.find(center_id);
  if (center_pole == nullptr) {
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

  const wire::core::Vec3d branch_dir = normalize_xy_safe(wire::core::Vec3d{0.0, 1.0, 0.0});
  bool found = false;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto* visual = state.view().find_span_visual_cache(span_id);
    if (visual == nullptr) {
      continue;
    }
    for (const wire::core::BranchSupportPlacement& placement : visual->branch_supports) {
      if (placement.owner_pole_id != center_id) {
        continue;
      }
      found = true;
      const wire::core::Vec3d support_axis = normalize_xy_safe(placement.tip_world - placement.mount_world);
      const double axis_alignment = std::abs(dot_xy(support_axis, branch_dir));
      const wire::core::Vec3d hanger_delta = placement.attachment_world - placement.tip_world;
      const double hanger_xy = std::sqrt(hanger_delta.x * hanger_delta.x + hanger_delta.y * hanger_delta.y);
      const double support_start_xy =
          std::sqrt(std::pow(placement.mount_world.x - center_pole->world_transform.position.x, 2.0) +
                    std::pow(placement.mount_world.y - center_pole->world_transform.position.y, 2.0));
      if (axis_alignment > 0.15 || hanger_xy > 1e-6 || support_start_xy <= 0.15) {
        std::cerr << "[DBG] C196 axisAlignment=" << axis_alignment << " hangerXY=" << hanger_xy
                  << " supportStartXY=" << support_start_xy << "\n";
        return false;
      }
    }
  }
  return found;
}

bool test_backbone_default_single_branch_stays_flat_without_branch_support() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.size() != 1) {
    return false;
  }

  const auto& assignments = state.view().last_lane_assignments();
  if (assignments.empty()) {
    return false;
  }
  const wire::core::SegmentLaneAssignment* branch_assignment = &assignments.back();
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);
  const auto span_view = state.view().inspect_span(branch_generated.value.generated_span_ids.front());
  if (!span_view.has_value()) {
    return false;
  }
  if (!(branch_assignment->flow_kind == wire::core::BackboneFlowKind::kBranch &&
        !branch_assignment->uses_branch_support &&
        branch_assignment->branch_down_offset_m == 0.0 &&
        span_view->flow_kind == wire::core::BackboneFlowKind::kBranch &&
        !span_view->uses_branch_support)) {
    std::cerr << "[DBG] C197 flow=" << static_cast<int>(branch_assignment->flow_kind)
              << " usesBranchSupport=" << (branch_assignment->uses_branch_support ? 1 : 0)
              << " downOffset=" << branch_assignment->branch_down_offset_m
              << " spanBranchSupport=" << (span_view->uses_branch_support ? 1 : 0) << "\n";
  }
  return branch_assignment->flow_kind == wire::core::BackboneFlowKind::kBranch &&
         !branch_assignment->uses_branch_support &&
         branch_assignment->branch_down_offset_m == 0.0 &&
         span_view->flow_kind == wire::core::BackboneFlowKind::kBranch &&
         !span_view->uses_branch_support;
}

bool test_backbone_communication_bundle_branch_stays_flat_without_branch_support() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kCommunication, 3);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kCommunication, 3);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.size() != 3) {
    return false;
  }

  const auto& assignments = state.view().last_lane_assignments();
  if (assignments.empty()) {
    return false;
  }
  const wire::core::SegmentLaneAssignment* branch_assignment = &assignments.back();
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);
  const auto span_view = state.view().inspect_span(branch_generated.value.generated_span_ids.front());
  if (!span_view.has_value()) {
    return false;
  }
  if (!(branch_assignment->flow_kind == wire::core::BackboneFlowKind::kBranch &&
        !branch_assignment->uses_branch_support &&
        branch_assignment->branch_down_offset_m == 0.0 &&
        span_view->flow_kind == wire::core::BackboneFlowKind::kBranch &&
        !span_view->uses_branch_support)) {
    std::cerr << "[DBG] C198 flow=" << static_cast<int>(branch_assignment->flow_kind)
              << " usesBranchSupport=" << (branch_assignment->uses_branch_support ? 1 : 0)
              << " downOffset=" << branch_assignment->branch_down_offset_m
              << " spanBranchSupport=" << (span_view->uses_branch_support ? 1 : 0)
              << " generatedSpanCount=" << branch_generated.value.generated_span_ids.size() << "\n";
  }
  return branch_assignment->flow_kind == wire::core::BackboneFlowKind::kBranch &&
         !branch_assignment->uses_branch_support &&
         branch_assignment->branch_down_offset_m == 0.0 &&
         span_view->flow_kind == wire::core::BackboneFlowKind::kBranch &&
         !span_view->uses_branch_support;
}

bool test_backbone_hv3_branch_support_policy_applies_on_both_default_pole_types() {
  auto run_case = [](const char* pole_type_name) -> bool {
    CoreState state;
    PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
    for (const auto& [id, pole_type] : state.view().pole_types()) {
      if (pole_type.name == pole_type_name) {
        pole_type_id = id;
        break;
      }
    }
    if (pole_type_id == wire::core::kInvalidPoleTypeId) {
      std::cerr << "[DBG] C199 pole_type_missing name=" << pole_type_name << "\n";
      return false;
    }

    wire::core::BackboneSpec trunk{};
    trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
    trunk.interval_m = 1000.0;
    trunk.pole_type_id = pole_type_id;
    add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
    if (!state.GenerateFromBackboneSpec(trunk).ok) {
      std::cerr << "[DBG] C199 trunk_generate_failed type=" << pole_type_name << "\n";
      return false;
    }

    const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
    if (center_id == wire::core::kInvalidObjectId) {
      return false;
    }

    wire::core::BackboneSpec branch{};
    branch.path.polyline = {{0.0, 0.0, 0.0}, {8.0, 12.0, 0.0}};
    wire::core::BackboneInputSpec::NodeSpec shared{};
    shared.point_index = 0;
    shared.support_kind = wire::core::SupportKind::kPole;
    shared.node_id = center_id;
    branch.path.node_specs.push_back(shared);
    branch.interval_m = 1000.0;
    branch.pole_type_id = pole_type_id;
    add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
    const auto branch_generated = state.GenerateFromBackboneSpec(branch);
    if (!branch_generated.ok || branch_generated.value.generated_span_ids.size() != 3) {
      std::cerr << "[DBG] C199 branch_generate_failed type=" << pole_type_name
                << " error=" << branch_generated.error
                << " count=" << branch_generated.value.generated_span_ids.size() << "\n";
      return false;
    }

    const auto& assignments = state.view().last_lane_assignments();
    const auto it_branch = std::find_if(assignments.begin(), assignments.end(),
                                        [&](const wire::core::SegmentLaneAssignment& assignment) {
                                          return assignment.flow_kind == wire::core::BackboneFlowKind::kBranch;
                                        });
    if (it_branch == assignments.end() || !it_branch->uses_branch_support || it_branch->branch_down_offset_m <= 1e-6) {
      std::cerr << "[DBG] C199 branch_policy_missing type=" << pole_type_name
                << " found=" << (it_branch != assignments.end() ? 1 : 0)
                << " uses=" << ((it_branch != assignments.end() && it_branch->uses_branch_support) ? 1 : 0)
                << " down=" << (it_branch != assignments.end() ? it_branch->branch_down_offset_m : 0.0) << "\n";
      return false;
    }

    const auto* center_pole = state.view().edit_state().poles.find(center_id);
    if (center_pole == nullptr) {
      return false;
    }

    std::vector<const wire::core::Port*> root_ports{};
    double main_min_z = std::numeric_limits<double>::infinity();
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id != center_id || port.layer != wire::core::PortLayer::kHighVoltage) {
        continue;
      }
      if (port.placement_source == wire::core::PortPlacementSourceKind::kBranchSupport) {
        root_ports.push_back(&port);
      } else {
        main_min_z = std::min(main_min_z, port.world_position.z);
      }
    }
    if (root_ports.size() != 3 || !std::isfinite(main_min_z)) {
      std::cerr << "[DBG] C199 root_port_collection_failed type=" << pole_type_name
                << " branchPorts=" << root_ports.size() << " mainMinZ=" << main_min_z << "\n";
      return false;
    }

    const double min_z = std::min({root_ports[0]->world_position.z, root_ports[1]->world_position.z,
                                   root_ports[2]->world_position.z});
    const double max_z = std::max({root_ports[0]->world_position.z, root_ports[1]->world_position.z,
                                   root_ports[2]->world_position.z});
    if ((max_z - min_z) > 1e-6 || !(max_z + 1e-6 < main_min_z)) {
      std::cerr << "[DBG] C199 root_port_z_failed type=" << pole_type_name << " minZ=" << min_z
                << " maxZ=" << max_z << " mainMinZ=" << main_min_z << "\n";
      return false;
    }

    wire::core::CommitOptions options{};
    options.run_recalc = true;
    (void)state.Commit(options);

    const wire::core::Vec3d branch_dir = normalize_xy_safe(wire::core::Vec3d{8.0, 12.0, 0.0});
    bool found_support = false;
    for (ObjectId span_id : branch_generated.value.generated_span_ids) {
      const auto* visual = state.view().find_span_visual_cache(span_id);
      if (visual == nullptr) {
        continue;
      }
      for (const wire::core::BranchSupportPlacement& placement : visual->branch_supports) {
        if (placement.owner_pole_id != center_id) {
          continue;
        }
        found_support = true;
        const wire::core::Vec3d support_axis = normalize_xy_safe(placement.tip_world - placement.mount_world);
        const double axis_alignment = std::abs(dot_xy(support_axis, branch_dir));
        if (axis_alignment > 0.10) {
          std::cerr << "[DBG] C199 support_axis_failed type=" << pole_type_name
                    << " axisAlignment=" << axis_alignment << "\n";
          return false;
        }
      }
    }
    return found_support;
  };

  return run_case("DistributionPole") && run_case("CommunicationPole");
}

bool test_backbone_hv3_acute_corner_lowers_corner_bundle_without_branch_support() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {-10.0, 2.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() != 6) {
    std::cerr << "[DBG] C204 generate_failed error=" << generated.error
              << " spanCount=" << generated.value.generated_span_ids.size() << "\n";
    return false;
  }

  const ObjectId prev_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId next_id = find_pole_id_by_position(state, {-10.0, 2.0, 0.0});
  if (prev_id == wire::core::kInvalidObjectId || center_id == wire::core::kInvalidObjectId ||
      next_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::unordered_set<ObjectId> generated_port_ids{};
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      continue;
    }
    generated_port_ids.insert(span->port_a_id);
    generated_port_ids.insert(span->port_b_id);
  }

  auto collect_hv_ports = [&](ObjectId pole_id) {
    std::vector<const wire::core::Port*> ports{};
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id == pole_id && port.layer == wire::core::PortLayer::kHighVoltage &&
          generated_port_ids.contains(port.id)) {
        ports.push_back(&port);
      }
    }
    return ports;
  };
  const auto prev_ports = collect_hv_ports(prev_id);
  const auto center_ports = collect_hv_ports(center_id);
  const auto next_ports = collect_hv_ports(next_id);
  if (prev_ports.size() != 3 || center_ports.size() != 3 || next_ports.size() != 3) {
    std::cerr << "[DBG] C204 port_count_failed prev=" << prev_ports.size() << " center=" << center_ports.size()
              << " next=" << next_ports.size() << "\n";
    return false;
  }

  auto min_z = [](const std::vector<const wire::core::Port*>& ports) {
    double z = std::numeric_limits<double>::infinity();
    for (const auto* port : ports) {
      z = std::min(z, port->world_position.z);
    }
    return z;
  };
  auto max_z = [](const std::vector<const wire::core::Port*>& ports) {
    double z = -std::numeric_limits<double>::infinity();
    for (const auto* port : ports) {
      z = std::max(z, port->world_position.z);
    }
    return z;
  };

  const double prev_min_z = min_z(prev_ports);
  const double center_min_z = min_z(center_ports);
  const double center_max_z = max_z(center_ports);
  const double next_min_z = min_z(next_ports);
  const bool center_uniform_height = (center_max_z - center_min_z) <= 1e-6;
  const bool center_lowered = center_max_z + 1e-6 < std::min(prev_min_z, next_min_z);
  const bool center_not_branch_support = std::none_of(center_ports.begin(), center_ports.end(), [](const wire::core::Port* port) {
    return port != nullptr && port->placement_source == wire::core::PortPlacementSourceKind::kBranchSupport;
  });

  const auto& assignments = state.view().last_lane_assignments();
  const double expected_down_offset = 0.275;
  int acute_touch_count = 0;
  bool assignment_offset_ok = true;
  for (const auto& assignment : assignments) {
    if (assignment.pole_a_id != center_id && assignment.pole_b_id != center_id) {
      continue;
    }
    if (assignment.uses_branch_support) {
      std::cerr << "[DBG] C204 unexpected_branch_support segment=" << assignment.segment_index << "\n";
      return false;
    }
    if (std::abs(assignment.branch_down_offset_m - expected_down_offset) > 1e-9) {
      std::cerr << "[DBG] C204 wrong_down_offset segment=" << assignment.segment_index
                << " down=" << assignment.branch_down_offset_m
                << " turn=" << assignment.turn_angle_deg << "\n";
      assignment_offset_ok = false;
    }
    ++acute_touch_count;
  }

  if (!(center_uniform_height && center_lowered && center_not_branch_support && acute_touch_count == 2 &&
        assignment_offset_ok)) {
    std::cerr << "[DBG] C204 centerMinZ=" << center_min_z << " centerMaxZ=" << center_max_z
              << " prevMinZ=" << prev_min_z << " nextMinZ=" << next_min_z
              << " uniform=" << (center_uniform_height ? 1 : 0)
              << " lowered=" << (center_lowered ? 1 : 0)
              << " nonBranch=" << (center_not_branch_support ? 1 : 0)
              << " acuteSegments=" << acute_touch_count
              << " offsetOk=" << (assignment_offset_ok ? 1 : 0) << "\n";
  }
  return center_uniform_height && center_lowered && center_not_branch_support && acute_touch_count == 2 &&
         assignment_offset_ok;
}

bool test_backbone_hv3_acute_corner_lowering_survives_pole_refresh() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {-10.0, 2.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId prev_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId next_id = find_pole_id_by_position(state, {-10.0, 2.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId || prev_id == wire::core::kInvalidObjectId ||
      next_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::unordered_set<ObjectId> generated_port_ids{};
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      continue;
    }
    generated_port_ids.insert(span->port_a_id);
    generated_port_ids.insert(span->port_b_id);
  }

  auto min_generated_hv_z = [&](ObjectId pole_id) {
    double z = std::numeric_limits<double>::infinity();
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id == pole_id && port.layer == wire::core::PortLayer::kHighVoltage &&
          generated_port_ids.contains(port.id)) {
        z = std::min(z, port.world_position.z);
      }
    }
    return z;
  };

  const double before_center_z = min_generated_hv_z(center_id);
  const double before_neighbor_z = std::min(min_generated_hv_z(prev_id), min_generated_hv_z(next_id));
  if (!(std::isfinite(before_center_z) && std::isfinite(before_neighbor_z) && before_center_z + 1e-6 < before_neighbor_z)) {
    std::cerr << "[DBG] C205 before centerZ=" << before_center_z << " neighborZ=" << before_neighbor_z << "\n";
    return false;
  }

  const auto yaw_result = state.SetPoleManualYawOverride(center_id, 15.0);
  if (!yaw_result.ok) {
    std::cerr << "[DBG] C205 yaw_override_failed error=" << yaw_result.error << "\n";
    return false;
  }

  const double after_center_z = min_generated_hv_z(center_id);
  const double after_neighbor_z = std::min(min_generated_hv_z(prev_id), min_generated_hv_z(next_id));
  bool center_source_ok = true;
  for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
    if (port.owner_pole_id == center_id && port.layer == wire::core::PortLayer::kHighVoltage &&
        generated_port_ids.contains(port.id) &&
        port.placement_source == wire::core::PortPlacementSourceKind::kPlacementBand) {
      center_source_ok = false;
    }
  }

  const bool ok = std::isfinite(after_center_z) && std::isfinite(after_neighbor_z) &&
                  after_center_z + 1e-6 < after_neighbor_z && center_source_ok;
  if (!ok) {
    std::cerr << "[DBG] C205 after centerZ=" << after_center_z << " neighborZ=" << after_neighbor_z
              << " sourceOk=" << (center_source_ok ? 1 : 0) << "\n";
  }
  return ok;
}

bool test_backbone_hv3_moderate_acute_corner_lowers_bundle_at_default_threshold() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {-8.0, 6.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() != 6) {
    std::cerr << "[DBG] C209 generate_failed error=" << generated.error
              << " spanCount=" << generated.value.generated_span_ids.size() << "\n";
    return false;
  }

  const ObjectId prev_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId next_id = find_pole_id_by_position(state, {-8.0, 6.0, 0.0});
  if (prev_id == wire::core::kInvalidObjectId || center_id == wire::core::kInvalidObjectId ||
      next_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::unordered_set<ObjectId> generated_port_ids{};
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      continue;
    }
    generated_port_ids.insert(span->port_a_id);
    generated_port_ids.insert(span->port_b_id);
  }

  auto collect_hv_ports = [&](ObjectId pole_id) {
    std::vector<const wire::core::Port*> ports{};
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id == pole_id && port.layer == wire::core::PortLayer::kHighVoltage &&
          generated_port_ids.contains(port.id)) {
        ports.push_back(&port);
      }
    }
    return ports;
  };
  const auto prev_ports = collect_hv_ports(prev_id);
  const auto center_ports = collect_hv_ports(center_id);
  const auto next_ports = collect_hv_ports(next_id);
  if (prev_ports.size() != 3 || center_ports.size() != 3 || next_ports.size() != 3) {
    return false;
  }

  auto min_z = [](const std::vector<const wire::core::Port*>& ports) {
    double z = std::numeric_limits<double>::infinity();
    for (const auto* port : ports) {
      z = std::min(z, port->world_position.z);
    }
    return z;
  };
  auto max_z = [](const std::vector<const wire::core::Port*>& ports) {
    double z = -std::numeric_limits<double>::infinity();
    for (const auto* port : ports) {
      z = std::max(z, port->world_position.z);
    }
    return z;
  };

  const double prev_min_z = min_z(prev_ports);
  const double center_min_z = min_z(center_ports);
  const double center_max_z = max_z(center_ports);
  const double next_min_z = min_z(next_ports);
  const bool center_uniform_height = (center_max_z - center_min_z) <= 1e-6;
  const bool center_lowered = center_max_z + 1e-6 < std::min(prev_min_z, next_min_z);

  int acute_touch_count = 0;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if (assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) {
      ++acute_touch_count;
    }
  }

  if (!(center_uniform_height && center_lowered && acute_touch_count == 2)) {
    std::cerr << "[DBG] C209 centerMinZ=" << center_min_z << " centerMaxZ=" << center_max_z
              << " prevMinZ=" << prev_min_z << " nextMinZ=" << next_min_z
              << " acuteSegments=" << acute_touch_count << "\n";
  }
  return center_uniform_height && center_lowered && acute_touch_count == 2;
}

bool test_backbone_junction_prefers_straighter_pair_over_first_drawn_primary() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec bent_main{};
  bent_main.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  bent_main.interval_m = 1000.0;
  bent_main.pole_type_id = type_ids.front();
  add_backbone_bundle(bent_main, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(bent_main).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec west{};
  west.path.polyline = {{0.0, 0.0, 0.0}, {-12.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  west.path.node_specs.push_back(shared);
  west.interval_m = 1000.0;
  west.pole_type_id = type_ids.front();
  add_backbone_bundle(west, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(west).ok) {
    return false;
  }

  const auto pole_view = state.view().inspect_pole(center_id);
  if (!pole_view.has_value()) {
    return false;
  }
  const bool horizontal =
      std::min(angle_diff_abs_deg(pole_view->final_yaw_deg, 0.0), angle_diff_abs_deg(pole_view->final_yaw_deg, 180.0)) <=
      1e-6;
  const bool pair_rule = pole_view->forward_rule == wire::core::PoleForwardRule::kMainChainBisector &&
                         pole_view->support_axis_rule == wire::core::PoleSupportAxisRule::kConnectedDirectionFit;
  if (!(horizontal && pair_rule)) {
    std::cerr << "[DBG] C194 yaw=" << pole_view->final_yaw_deg
              << " rule=" << static_cast<int>(pole_view->forward_rule)
              << " supportRule=" << static_cast<int>(pole_view->support_axis_rule) << "\n";
  }
  return horizontal && pair_rule;
}

bool test_backbone_cross_junction_nonmain_line_uses_underpass() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    std::cerr << "[DBG] C201 no pole types\n";
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = type_ids.front();
  add_backbone_bundle(main_req, wire::core::BundleKind::kLowVoltage);
  const auto main_generated = state.GenerateFromBackboneSpec(main_req);
  if (!main_generated.ok) {
    std::cerr << "[DBG] C201 main generate failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C201 center pole missing after main\n";
    return false;
  }

  double main_center_z = -1.0;
  for (ObjectId span_id : main_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      std::cerr << "[DBG] C201 main span missing\n";
      return false;
    }
    const ObjectId center_port_id = (span->endpoint_node_a_id == center_id) ? span->port_a_id
                                   : (span->endpoint_node_b_id == center_id) ? span->port_b_id
                                                                             : wire::core::kInvalidObjectId;
    if (center_port_id == wire::core::kInvalidObjectId) {
      continue;
    }
    const auto* center_port = state.view().edit_state().ports.find(center_port_id);
    if (center_port == nullptr) {
      std::cerr << "[DBG] C201 main center port missing\n";
      return false;
    }
    main_center_z = std::max(main_center_z, center_port->world_position.z);
  }
  if (main_center_z <= 0.0) {
    std::cerr << "[DBG] C201 invalid main_center_z=" << main_center_z << "\n";
    return false;
  }

  wire::core::BackboneSpec cross_req{};
  cross_req.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross_req.path.node_specs.push_back(shared);
  cross_req.interval_m = 1000.0;
  cross_req.pole_type_id = type_ids.front();
  add_backbone_bundle(cross_req, wire::core::BundleKind::kLowVoltage);
  const auto cross_generated = state.GenerateFromBackboneSpec(cross_req);
  if (!cross_generated.ok || cross_generated.value.generated_span_ids.size() != 2) {
    std::cerr << "[DBG] C201 cross generate failed ok=" << (cross_generated.ok ? 1 : 0)
              << " spanCount=" << cross_generated.value.generated_span_ids.size() << "\n";
    return false;
  }

  const auto pole_view = state.view().inspect_pole(center_id);
  if (!pole_view.has_value()) {
    std::cerr << "[DBG] C201 pole inspect missing\n";
    return false;
  }

  int underpass_assignment_count = 0;
  double min_cross_center_z = std::numeric_limits<double>::infinity();
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const bool touches_center = assignment.pole_a_id == center_id || assignment.pole_b_id == center_id;
    if (!touches_center) {
      continue;
    }
    if (assignment.flow_kind != wire::core::BackboneFlowKind::kBranch ||
        assignment.flow_decision_rule != wire::core::BackboneFlowDecisionRule::kJunctionOrderBranch ||
        !assignment.uses_branch_support || assignment.branch_down_offset_m <= 1e-6) {
      continue;
    }
    ++underpass_assignment_count;
  }

  for (ObjectId span_id : cross_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      std::cerr << "[DBG] C201 cross span missing\n";
      return false;
    }
    const ObjectId center_port_id = (span->endpoint_node_a_id == center_id) ? span->port_a_id
                                   : (span->endpoint_node_b_id == center_id) ? span->port_b_id
                                                                             : wire::core::kInvalidObjectId;
    if (center_port_id == wire::core::kInvalidObjectId) {
      return false;
    }
    const auto* center_port = state.view().edit_state().ports.find(center_port_id);
    if (center_port == nullptr) {
      std::cerr << "[DBG] C201 cross center port missing\n";
      return false;
    }
    min_cross_center_z = std::min(min_cross_center_z, center_port->world_position.z);
  }
  const bool ok = underpass_assignment_count == 2 && std::isfinite(min_cross_center_z) &&
                  min_cross_center_z + 1e-6 < main_center_z;
  if (!ok) {
    std::cerr << "[DBG] C201 underpassCount=" << underpass_assignment_count
              << " minCrossCenterZ=" << min_cross_center_z
              << " mainCenterZ=" << main_center_z
              << " centerYaw=" << pole_view->final_yaw_deg << "\n";
    for (const auto& assignment : state.view().last_lane_assignments()) {
      std::cerr << "[DBG] C201 assignment poles=" << assignment.pole_a_id << "->" << assignment.pole_b_id
                << " flow=" << static_cast<int>(assignment.flow_kind)
                << " rule=" << static_cast<int>(assignment.flow_decision_rule)
                << " usesBranchSupport=" << (assignment.uses_branch_support ? 1 : 0)
                << " down=" << assignment.branch_down_offset_m << "\n";
    }
  }
  return ok;
}

bool test_backbone_all_templates_branch_keeps_hv_down_offset_on_communication_pole() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C210 missing CommunicationPole\n";
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = pole_type_id;
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(trunk, wire::core::BundleKind::kOptical);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C210 trunk generate failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C210 center missing\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {8.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = pole_type_id;
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(branch, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    std::cerr << "[DBG] C210 branch generate failed error=" << generated.error << "\n";
    return false;
  }

  bool hv_branch_found = false;
  bool hv_branch_lowered = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    hv_branch_found = true;
    if (assignment.flow_kind == wire::core::BackboneFlowKind::kBranch && assignment.uses_branch_support &&
        assignment.branch_down_offset_m > 1e-6) {
      hv_branch_lowered = true;
      break;
    }
  }
  if (!(hv_branch_found && hv_branch_lowered)) {
    std::cerr << "[DBG] C210 hvBranchFound=" << (hv_branch_found ? 1 : 0)
              << " hvBranchLowered=" << (hv_branch_lowered ? 1 : 0) << "\n";
    for (const auto& assignment : state.view().last_lane_assignments()) {
      const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
      std::cerr << "[DBG] C210 assignment bundle="
                << (bundle != nullptr ? static_cast<int>(bundle->bundle_template_id) : -1)
                << " flow=" << static_cast<int>(assignment.flow_kind)
                << " uses=" << (assignment.uses_branch_support ? 1 : 0)
                << " down=" << assignment.branch_down_offset_m << "\n";
    }
  }
  return hv_branch_found && hv_branch_lowered;
}

bool test_backbone_all_templates_cross_keeps_underpass_on_communication_pole() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C211 missing CommunicationPole\n";
    return false;
  }

  wire::core::BackboneSpec main_req{};
  main_req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  main_req.interval_m = 1000.0;
  main_req.pole_type_id = pole_type_id;
  add_backbone_bundle(main_req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(main_req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(main_req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(main_req, wire::core::BundleKind::kOptical);
  if (!state.GenerateFromBackboneSpec(main_req).ok) {
    std::cerr << "[DBG] C211 main generate failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C211 center missing\n";
    return false;
  }

  wire::core::BackboneSpec cross_req{};
  cross_req.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross_req.path.node_specs.push_back(shared);
  cross_req.interval_m = 1000.0;
  cross_req.pole_type_id = pole_type_id;
  add_backbone_bundle(cross_req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(cross_req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(cross_req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(cross_req, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(cross_req);
  if (!generated.ok) {
    std::cerr << "[DBG] C211 cross generate failed error=" << generated.error << "\n";
    return false;
  }

  int lowered_assignments = 0;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const bool touches_center = assignment.pole_a_id == center_id || assignment.pole_b_id == center_id;
    if (!touches_center) {
      continue;
    }
    if (assignment.flow_kind == wire::core::BackboneFlowKind::kBranch && assignment.branch_down_offset_m > 1e-6) {
      ++lowered_assignments;
    }
  }
  if (lowered_assignments == 0) {
    std::cerr << "[DBG] C211 loweredAssignments=0\n";
    for (const auto& assignment : state.view().last_lane_assignments()) {
      const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
      std::cerr << "[DBG] C211 assignment bundle="
                << (bundle != nullptr ? static_cast<int>(bundle->bundle_template_id) : -1)
                << " poles=" << assignment.pole_a_id << "->" << assignment.pole_b_id
                << " flow=" << static_cast<int>(assignment.flow_kind)
                << " uses=" << (assignment.uses_branch_support ? 1 : 0)
                << " down=" << assignment.branch_down_offset_m << "\n";
    }
  }
  return lowered_assignments > 0;
}

bool test_backbone_capture_branch_then_acute_lowering_on_communication_pole() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C212 missing CommunicationPole\n";
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {
      {-18.59678, 11.0534, 0.0},
      {-6.59678, 11.0534, 0.0},
      {5.40322, 11.0534, 0.0},
  };
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = pole_type_id;
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(trunk, wire::core::BundleKind::kOptical);
  const auto trunk_generated = state.GenerateFromBackboneSpec(trunk);
  if (!trunk_generated.ok) {
    std::cerr << "[DBG] C212 trunk generate failed error=" << trunk_generated.error << "\n";
    return false;
  }

  const ObjectId node0 = find_pole_id_by_position(state, {-6.59678, 11.0534, 0.0}, 1e-4);
  auto min_hv_z_at_pole = [&](ObjectId pole_id) {
    double best = std::numeric_limits<double>::infinity();
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id == pole_id && port.layer == wire::core::PortLayer::kHighVoltage) {
        best = std::min(best, port.world_position.z);
      }
    }
    return best;
  };
  const double trunk_root_hv_z = min_hv_z_at_pole(node0);
  if (node0 == wire::core::kInvalidObjectId || !std::isfinite(trunk_root_hv_z)) {
    std::cerr << "[DBG] C212 shared root lookup failed\n";
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {-6.59678, 11.0534, 0.0},
      {-4.93216, 6.7054, 0.0},
      {-13.6709, -1.24875, 0.0},
      {-8.49996, -0.441201, 0.0},
  };
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = node0;
  req.path.node_specs.push_back(shared);
  req.interval_m = 8.0;
  req.pole_type_id = pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C212 branch generate failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId node1 = find_pole_id_by_position(state, {-4.93216, 6.7054, 0.0}, 1e-4);
  const ObjectId node2 = find_pole_id_by_position(state, {-13.6709, -1.24875, 0.0}, 1e-4);
  const ObjectId node3 = find_pole_id_by_position(state, {-8.49996, -0.441201, 0.0}, 1e-4);
  if (node0 == wire::core::kInvalidObjectId || node1 == wire::core::kInvalidObjectId ||
      node2 == wire::core::kInvalidObjectId || node3 == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C212 node lookup failed\n";
    return false;
  }

  std::vector<const wire::core::SegmentLaneAssignment*> hv_assignments{};
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == wire::core::BundleKind::kHighVoltage) {
      hv_assignments.push_back(&assignment);
    }
  }
  if (hv_assignments.size() < 3) {
    std::cerr << "[DBG] C212 hvAssignments=" << hv_assignments.size() << "\n";
    return false;
  }

  const wire::core::SegmentLaneAssignment* first_assignment = hv_assignments.front();
  const bool first_is_branch_support =
      first_assignment->segment_index == 0 &&
      first_assignment->pole_a_id == node0 && first_assignment->pole_b_id == node1 &&
      first_assignment->uses_branch_support &&
      first_assignment->lowering_kind == wire::core::BackboneLoweringKind::kBranchSupport &&
      first_assignment->branch_down_offset_m > 1e-6;
  int acute_after_root_count = 0;
  bool acute_touches_explicit_corner = false;
  for (const auto* assignment : hv_assignments) {
    if (assignment->segment_index == 0) {
      continue;
    }
    if (assignment->uses_branch_support ||
        assignment->lowering_kind != wire::core::BackboneLoweringKind::kAcuteCorner ||
        assignment->branch_down_offset_m <= 1e-6) {
      continue;
    }
    ++acute_after_root_count;
    if (assignment->pole_a_id == node2 || assignment->pole_b_id == node2) {
      acute_touches_explicit_corner = true;
    }
  }

  auto min_generated_hv_z = [&](ObjectId pole_id) {
    double best = std::numeric_limits<double>::infinity();
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id != pole_id || port.layer != wire::core::PortLayer::kHighVoltage) {
        continue;
      }
      best = std::min(best, port.world_position.z);
    }
    return best;
  };

  double root_branch_hv_z = std::numeric_limits<double>::infinity();
  if (first_is_branch_support) {
    const auto& root_port_ids = (first_assignment->pole_a_id == node0) ? first_assignment->port_ids_a
                                                                        : first_assignment->port_ids_b;
    for (ObjectId port_id : root_port_ids) {
      const wire::core::Port* port = state.view().edit_state().ports.find(port_id);
      if (port != nullptr) {
        root_branch_hv_z = std::min(root_branch_hv_z, port->world_position.z);
      }
    }
  }
  const double node1_z = min_generated_hv_z(node1);
  const double node2_z = min_generated_hv_z(node2);
  const double node3_z = min_generated_hv_z(node3);
  const bool root_lowered =
      std::isfinite(root_branch_hv_z) && root_branch_hv_z + 1e-6 < trunk_root_hv_z;
  const bool acute_center_lowered =
      std::isfinite(node1_z) && std::isfinite(node2_z) && std::isfinite(node3_z) && node2_z + 1e-6 < node1_z &&
      node2_z + 1e-6 < node3_z;

  if (!(first_is_branch_support && acute_after_root_count >= 1 && acute_touches_explicit_corner && root_lowered &&
        acute_center_lowered)) {
    std::cerr << "[DBG] C212 firstBranch=" << (first_is_branch_support ? 1 : 0)
              << " acuteAfterRoot=" << acute_after_root_count
              << " acuteTouchesCorner=" << (acute_touches_explicit_corner ? 1 : 0)
              << " rootLowered=" << (root_lowered ? 1 : 0)
              << " trunkRootZ=" << trunk_root_hv_z << " branchRootZ=" << root_branch_hv_z
              << " node1Z=" << node1_z << " node2Z=" << node2_z << " node3Z=" << node3_z << "\n";
    for (const auto* assignment : hv_assignments) {
      std::cerr << "[DBG] C212 hv assignment poles=" << assignment->pole_a_id << "->" << assignment->pole_b_id
                << " flow=" << static_cast<int>(assignment->flow_kind)
                << " lowering=" << static_cast<int>(assignment->lowering_kind)
                << " branchSupport=" << (assignment->uses_branch_support ? 1 : 0)
                << " down=" << assignment->branch_down_offset_m << "\n";
    }
  }

  return first_is_branch_support && acute_after_root_count >= 1 && acute_touches_explicit_corner && root_lowered &&
         acute_center_lowered;
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

bool test_backbone_hv3_terminal_poles_use_distinct_template_bands() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const ObjectId start_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId end_id = find_pole_id_by_position(state, {12.0, 0.0, 0.0});
  if (start_id == wire::core::kInvalidObjectId || end_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const wire::core::SegmentLaneAssignment* assignment = nullptr;
  for (const auto& candidate : state.view().last_lane_assignments()) {
    if ((candidate.pole_a_id == start_id && candidate.pole_b_id == end_id) ||
        (candidate.pole_a_id == end_id && candidate.pole_b_id == start_id)) {
      assignment = &candidate;
      break;
    }
  }
  if (assignment == nullptr) {
    return false;
  }
  const VisualSeparationMetrics visual_metrics = measure_lane_visual_separation_metrics(state, *assignment);
  const AxisRelationMetrics start_axis_metrics =
      measure_pole_axis_relation_metrics(state, start_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const AxisRelationMetrics end_axis_metrics =
      measure_pole_axis_relation_metrics(state, end_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});

  auto pole_is_visually_distinct = [&](ObjectId pole_id) {
    const wire::core::SegmentLaneAssignment* assignment_for_pole = nullptr;
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id == pole_id || assignment.pole_b_id == pole_id) {
        assignment_for_pole = &assignment;
        break;
      }
    }
    if (assignment_for_pole == nullptr) {
      return false;
    }
    const std::vector<ObjectId>& used_port_ids =
        (assignment_for_pole->pole_a_id == pole_id) ? assignment_for_pole->port_ids_a : assignment_for_pole->port_ids_b;
    if (used_port_ids.size() != 3) {
      return false;
    }
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr) {
      return false;
    }
    const wire::core::Vec3d axis{1.0, 0.0, 0.0};
    const wire::core::Vec3d n = normalize_xy_safe(axis);
    const wire::core::Vec3d p{-n.y, n.x, 0.0};
    double min_along = std::numeric_limits<double>::infinity();
    double max_along = -std::numeric_limits<double>::infinity();
    double min_perp = std::numeric_limits<double>::infinity();
    double max_perp = -std::numeric_limits<double>::infinity();
    for (ObjectId port_id : used_port_ids) {
      const wire::core::Port* port = state.view().edit_state().ports.find(port_id);
      if (port == nullptr) {
        return false;
      }
      const wire::core::Vec3d delta = port->world_position - pole->world_transform.position;
      const double along = dot_xy(delta, n);
      const double perp = dot_xy(delta, p);
      min_along = std::min(min_along, along);
      max_along = std::max(max_along, along);
      min_perp = std::min(min_perp, perp);
      max_perp = std::max(max_perp, perp);
    }
    const double along_span = max_along - min_along;
    const double perp_span = max_perp - min_perp;
    return perp_span > 0.2 && perp_span > along_span * 2.0;
  };

  const bool ok = pole_is_visually_distinct(start_id) && pole_is_visually_distinct(end_id) && visual_metrics.topology_distinct &&
                  visual_metrics.visual_distinct && visual_metrics.projected_min_spacing_topview_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_start_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_end_m >= 0.10 && start_axis_metrics.valid &&
                  end_axis_metrics.valid && start_axis_metrics.angle_row_vs_span_deg >= 70.0 &&
                  end_axis_metrics.angle_row_vs_span_deg >= 70.0;
  if (!ok) {
    std::cerr << "[DBG] C185 visual=" << describe_visual_separation_metrics(visual_metrics)
              << " startAxis=" << describe_axis_relation_metrics(start_axis_metrics)
              << " endAxis=" << describe_axis_relation_metrics(end_axis_metrics) << "\n";
  }
  return ok;
}

bool test_backbone_hv3_terminal_fallback_ports_still_spread_perpendicular() {
  CoreState state;

  PoleTypeId fallback_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    bool has_hv_band = false;
    for (const auto& band : pole_type.port_bands) {
      if (band.enabled && band.category == wire::core::ConnectionCategory::kHighVoltage) {
        has_hv_band = true;
        break;
      }
    }
    if (!has_hv_band) {
      fallback_pole_type_id = pole_type.id;
      break;
    }
  }
  if (fallback_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = fallback_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const ObjectId start_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId end_id = find_pole_id_by_position(state, {12.0, 0.0, 0.0});
  if (start_id == wire::core::kInvalidObjectId || end_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const wire::core::SegmentLaneAssignment* assignment = nullptr;
  for (const auto& candidate : state.view().last_lane_assignments()) {
    if ((candidate.pole_a_id == start_id && candidate.pole_b_id == end_id) ||
        (candidate.pole_a_id == end_id && candidate.pole_b_id == start_id)) {
      assignment = &candidate;
      break;
    }
  }
  if (assignment == nullptr) {
    return false;
  }
  const VisualSeparationMetrics visual_metrics = measure_lane_visual_separation_metrics(state, *assignment);
  const AxisRelationMetrics start_axis_metrics =
      measure_pole_axis_relation_metrics(state, start_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const AxisRelationMetrics end_axis_metrics =
      measure_pole_axis_relation_metrics(state, end_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});

  auto pole_uses_perpendicular_generated_row = [&](ObjectId pole_id) {
    const wire::core::SegmentLaneAssignment* assignment_for_pole = nullptr;
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id == pole_id || assignment.pole_b_id == pole_id) {
        assignment_for_pole = &assignment;
        break;
      }
    }
    if (assignment_for_pole == nullptr) {
      return false;
    }

    const std::vector<ObjectId>& used_port_ids =
        (assignment_for_pole->pole_a_id == pole_id) ? assignment_for_pole->port_ids_a : assignment_for_pole->port_ids_b;
    if (used_port_ids.size() != 3) {
      return false;
    }

    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr) {
      return false;
    }

    bool saw_generated_row_port = false;
    const wire::core::Vec3d route_dir = normalize_xy_safe({1.0, 0.0, 0.0});
    const wire::core::Vec3d row_dir{-route_dir.y, route_dir.x, 0.0};
    double min_along = std::numeric_limits<double>::infinity();
    double max_along = -std::numeric_limits<double>::infinity();
    double min_perp = std::numeric_limits<double>::infinity();
    double max_perp = -std::numeric_limits<double>::infinity();
    for (ObjectId port_id : used_port_ids) {
      const wire::core::Port* port = state.view().edit_state().ports.find(port_id);
      if (port == nullptr) {
        return false;
      }
      if (port->generated_by_rule) {
        saw_generated_row_port = true;
      }
      const wire::core::Vec3d delta = port->world_position - pole->world_transform.position;
      const double along = dot_xy(delta, route_dir);
      const double perp = dot_xy(delta, row_dir);
      min_along = std::min(min_along, along);
      max_along = std::max(max_along, along);
      min_perp = std::min(min_perp, perp);
      max_perp = std::max(max_perp, perp);
    }
    const double along_span = max_along - min_along;
    const double perp_span = max_perp - min_perp;
    return saw_generated_row_port && perp_span > 0.2 && perp_span > along_span * 2.0;
  };

  const bool ok = pole_uses_perpendicular_generated_row(start_id) &&
                  pole_uses_perpendicular_generated_row(end_id) && visual_metrics.topology_distinct &&
                  visual_metrics.visual_distinct && visual_metrics.projected_min_spacing_topview_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_start_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_end_m >= 0.10 && start_axis_metrics.valid &&
                  end_axis_metrics.valid && start_axis_metrics.angle_row_vs_span_deg >= 70.0 &&
                  end_axis_metrics.angle_row_vs_span_deg >= 70.0;
  if (!ok) {
    std::cerr << "[DBG] C188 visual=" << describe_visual_separation_metrics(visual_metrics)
              << " startAxis=" << describe_axis_relation_metrics(start_axis_metrics)
              << " endAxis=" << describe_axis_relation_metrics(end_axis_metrics) << "\n";
  }
  return ok;
}

bool test_backbone_hv3_terminals_stay_perpendicular_on_communication_pole_with_all_templates() {
  CoreState state;

  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = communication_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const ObjectId start_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId end_id = find_pole_id_by_position(state, {12.0, 0.0, 0.0});
  if (start_id == wire::core::kInvalidObjectId || end_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const wire::core::SegmentLaneAssignment* assignment = nullptr;
  for (const auto& candidate : state.view().last_lane_assignments()) {
    if (!((candidate.pole_a_id == start_id && candidate.pole_b_id == end_id) ||
          (candidate.pole_a_id == end_id && candidate.pole_b_id == start_id))) {
      continue;
    }
    const auto* bundle = state.view().edit_state().bundles.find(candidate.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == wire::core::BundleKind::kHighVoltage) {
      assignment = &candidate;
      break;
    }
  }
  if (assignment == nullptr) {
    return false;
  }

  const VisualSeparationMetrics visual_metrics = measure_lane_visual_separation_metrics(state, *assignment);
  const AxisRelationMetrics start_axis_metrics =
      measure_pole_axis_relation_metrics(state, start_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const AxisRelationMetrics end_axis_metrics =
      measure_pole_axis_relation_metrics(state, end_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const bool ok = visual_metrics.topology_distinct && visual_metrics.visual_distinct &&
                  visual_metrics.projected_min_spacing_topview_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_start_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_end_m >= 0.10 && start_axis_metrics.valid &&
                  end_axis_metrics.valid && start_axis_metrics.angle_row_vs_span_deg >= 70.0 &&
                  end_axis_metrics.angle_row_vs_span_deg >= 70.0;
  if (!ok) {
    std::cerr << "[DBG] C189 visual=" << describe_visual_separation_metrics(visual_metrics)
              << " startAxis=" << describe_axis_relation_metrics(start_axis_metrics)
              << " endAxis=" << describe_axis_relation_metrics(end_axis_metrics) << "\n";
  }
  return ok;
}

bool test_backbone_communication_multilane_terminals_stay_perpendicular() {
  CoreState state;

  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = communication_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const ObjectId start_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId end_id = find_pole_id_by_position(state, {12.0, 0.0, 0.0});
  if (start_id == wire::core::kInvalidObjectId || end_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const wire::core::SegmentLaneAssignment* assignment = nullptr;
  for (const auto& candidate : state.view().last_lane_assignments()) {
    if (!((candidate.pole_a_id == start_id && candidate.pole_b_id == end_id) ||
          (candidate.pole_a_id == end_id && candidate.pole_b_id == start_id))) {
      continue;
    }
    const auto* bundle = state.view().edit_state().bundles.find(candidate.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == wire::core::BundleKind::kCommunication) {
      assignment = &candidate;
      break;
    }
  }
  if (assignment == nullptr) {
    return false;
  }

  const VisualSeparationMetrics visual_metrics = measure_lane_visual_separation_metrics(state, *assignment);
  const AxisRelationMetrics start_axis_metrics =
      measure_pole_axis_relation_metrics(state, start_id, wire::core::PortLayer::kCommunication, {1.0, 0.0, 0.0});
  const AxisRelationMetrics end_axis_metrics =
      measure_pole_axis_relation_metrics(state, end_id, wire::core::PortLayer::kCommunication, {1.0, 0.0, 0.0});
  const bool ok = visual_metrics.topology_distinct && visual_metrics.visual_distinct &&
                  visual_metrics.projected_min_spacing_topview_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_start_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_end_m >= 0.10 && start_axis_metrics.valid &&
                  end_axis_metrics.valid && start_axis_metrics.angle_row_vs_span_deg >= 70.0 && end_axis_metrics.angle_row_vs_span_deg >= 70.0;
  if (!ok) {
    std::cerr << "[DBG] C190 visual=" << describe_visual_separation_metrics(visual_metrics)
              << " startAxis=" << describe_axis_relation_metrics(start_axis_metrics)
              << " endAxis=" << describe_axis_relation_metrics(end_axis_metrics) << "\n";
  }
  return ok;
}

bool test_backbone_clicked_existing_communication_poles_all_templates_keep_hv_terminal_separation() {
  CoreState state;

  PoleTypeId communication_pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [_, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      communication_pole_type_id = pole_type.id;
      break;
    }
  }
  if (communication_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }

  wire::core::Transformd left_tf{};
  left_tf.position = {-12.0, 0.0, 0.0};
  wire::core::Transformd right_tf{};
  right_tf.position = {12.0, 0.0, 0.0};
  const auto left_add = state.AddPole(left_tf, 10.0, "Left");
  const auto right_add = state.AddPole(right_tf, 10.0, "Right");
  if (!left_add.ok || !right_add.ok) {
    return false;
  }
  const ObjectId left_id = left_add.value;
  const ObjectId right_id = right_add.value;
  if (!state.ApplyPoleType(left_id, communication_pole_type_id).ok ||
      !state.ApplyPoleType(right_id, communication_pole_type_id).ok) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec left_spec{};
  left_spec.point_index = 0;
  left_spec.support_kind = wire::core::SupportKind::kPole;
  left_spec.node_id = left_id;
  req.path.node_specs.push_back(left_spec);
  wire::core::BackboneInputSpec::NodeSpec right_spec{};
  right_spec.point_index = 1;
  right_spec.support_kind = wire::core::SupportKind::kPole;
  right_spec.node_id = right_id;
  req.path.node_specs.push_back(right_spec);
  req.interval_m = 1000.0;
  req.pole_type_id = communication_pole_type_id;
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 1);
  add_backbone_bundle(req, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const wire::core::SegmentLaneAssignment* assignment = nullptr;
  for (const auto& candidate : state.view().last_lane_assignments()) {
    if (!((candidate.pole_a_id == left_id && candidate.pole_b_id == right_id) ||
          (candidate.pole_a_id == right_id && candidate.pole_b_id == left_id))) {
      continue;
    }
    const auto* bundle = state.view().edit_state().bundles.find(candidate.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == wire::core::BundleKind::kHighVoltage) {
      assignment = &candidate;
      break;
    }
  }
  if (assignment == nullptr) {
    return false;
  }

  const VisualSeparationMetrics visual_metrics = measure_lane_visual_separation_metrics(state, *assignment);
  const AxisRelationMetrics left_axis_metrics =
      measure_pole_axis_relation_metrics(state, left_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const AxisRelationMetrics right_axis_metrics =
      measure_pole_axis_relation_metrics(state, right_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const bool ok = visual_metrics.topology_distinct && visual_metrics.visual_distinct &&
                  visual_metrics.projected_min_spacing_topview_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_start_m >= 0.10 &&
                  visual_metrics.min_wire_spacing_near_end_m >= 0.10 && left_axis_metrics.valid &&
                  right_axis_metrics.valid && left_axis_metrics.angle_row_vs_span_deg >= 70.0 && right_axis_metrics.angle_row_vs_span_deg >= 70.0;
  if (!ok) {
    std::cerr << "[DBG] C192 visual=" << describe_visual_separation_metrics(visual_metrics)
              << " leftAxis=" << describe_axis_relation_metrics(left_axis_metrics)
              << " rightAxis=" << describe_axis_relation_metrics(right_axis_metrics) << "\n";
  }
  return ok;
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
    std::cerr << "[DBG] C138 assignmentCount=" << assignments.size() << "\n";
    return false;
  }
  const bool ok = assignments[0].flow_kind == wire::core::BackboneFlowKind::kMain &&
                  assignments[0].flow_decision_rule == wire::core::BackboneFlowDecisionRule::kExistingChainMain &&
                  !assignments[0].uses_branch_support &&
                  assignments[1].flow_kind == wire::core::BackboneFlowKind::kBranch &&
                  assignments[1].flow_decision_rule == wire::core::BackboneFlowDecisionRule::kExistingChainBranch &&
                  !assignments[1].uses_branch_support &&
                  assignments[1].branch_down_offset_m == 0.0;
  if (!ok) {
    for (std::size_t i = 0; i < assignments.size(); ++i) {
      std::cerr << "[DBG] C138 idx=" << i
                << " flow=" << static_cast<int>(assignments[i].flow_kind)
                << " rule=" << static_cast<int>(assignments[i].flow_decision_rule)
                << " branchSupport=" << (assignments[i].uses_branch_support ? 1 : 0)
                << " down=" << assignments[i].branch_down_offset_m << "\n";
    }
  }
  return ok;
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

  bool found_branch_support = false;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto* visual = state.view().find_span_visual_cache(span_id);
    if (visual == nullptr || visual->branch_supports.empty()) {
      continue;
    }
    found_branch_support = found_branch_support ||
                           std::any_of(visual->branch_supports.begin(), visual->branch_supports.end(),
                                       [center_id](const wire::core::BranchSupportPlacement& placement) {
                                         return placement.owner_pole_id == center_id && placement.down_offset_m > 0.0 &&
                                                placement.side != wire::core::SlotSide::kCenter;
                                       });
  }
  return found_branch_support;
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
  const bool ok = assignments.size() == 1 && assignments.front().flow_kind == wire::core::BackboneFlowKind::kBranch &&
                  assignments.front().flow_decision_rule == wire::core::BackboneFlowDecisionRule::kExistingChainBranch;
  if (!ok) {
    std::cerr << "[DBG] C140 assignmentCount=" << assignments.size();
    if (!assignments.empty()) {
      std::cerr << " flow=" << static_cast<int>(assignments.front().flow_kind)
                << " rule=" << static_cast<int>(assignments.front().flow_decision_rule);
    }
    std::cerr << "\n";
  }
  return ok;
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

double support_axis_alignment_ratio(const CoreState& state, ObjectId pole_id, const wire::core::Vec3d& axis) {
  const auto pole_view = state.view().inspect_pole(pole_id);
  if (!pole_view.has_value() || !pole_view->has_support_axis) {
    return 0.0;
  }
  return std::abs(dot_xy(normalize_xy_safe(pole_view->support_axis_dir), normalize_xy_safe(axis)));
}

bool unique_pole_ports_spread_along_axis(const CoreState& state, ObjectId pole_id, wire::core::PortLayer layer,
                                         const wire::core::Vec3d& axis, double* out_along_span = nullptr,
                                         double* out_perp_span = nullptr) {
  std::vector<const wire::core::Port*> ports{};
  for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
    if (port.owner_pole_id == pole_id && port.layer == layer &&
        port.placement_source != wire::core::PortPlacementSourceKind::kBranchSupport) {
      ports.push_back(&port);
    }
  }
  if (ports.size() < 2) {
    return false;
  }
  const wire::core::Vec3d n = normalize_xy_safe(axis);
  const wire::core::Vec3d p{-n.y, n.x, 0.0};
  double min_along = std::numeric_limits<double>::infinity();
  double max_along = -std::numeric_limits<double>::infinity();
  double min_perp = std::numeric_limits<double>::infinity();
  double max_perp = -std::numeric_limits<double>::infinity();
  const auto* pole = state.view().edit_state().poles.find(pole_id);
  if (pole == nullptr) {
    return false;
  }
  for (const wire::core::Port* port : ports) {
    const wire::core::Vec3d delta = port->world_position - pole->world_transform.position;
    const double along = dot_xy(delta, n);
    const double perp = dot_xy(delta, p);
    min_along = std::min(min_along, along);
    max_along = std::max(max_along, along);
    min_perp = std::min(min_perp, perp);
    max_perp = std::max(max_perp, perp);
  }
  if (out_along_span != nullptr) {
    *out_along_span = max_along - min_along;
  }
  if (out_perp_span != nullptr) {
    *out_perp_span = max_perp - min_perp;
  }
  return (max_along - min_along) > (max_perp - min_perp) * 2.0;
}

double max_curve_lateral_overshoot_xy(const wire::core::DetailCurve& curve, std::size_t* out_peak_index = nullptr) {
  const wire::core::Vec3d start = curve.EvaluatePosition(0.0);
  const wire::core::Vec3d end = curve.EvaluatePosition(1.0);
  const wire::core::Vec3d chord_dir = normalize_xy_safe(end - start);
  if (std::abs(chord_dir.x) <= 1e-6 && std::abs(chord_dir.y) <= 1e-6) {
    return 0.0;
  }
  const wire::core::Vec3d lateral_axis{-chord_dir.y, chord_dir.x, 0.0};
  double max_abs_lateral = 0.0;
  std::size_t peak_index = 0;
  for (std::size_t i = 0; i < curve.sample_points.size(); ++i) {
    const double abs_lateral = std::abs(dot_xy(curve.sample_points[i] - start, lateral_axis));
    if (abs_lateral > max_abs_lateral) {
      max_abs_lateral = abs_lateral;
      peak_index = i;
    }
  }
  if (out_peak_index != nullptr) {
    *out_peak_index = peak_index;
  }
  return max_abs_lateral;
}

bool trace_contains_summary(const std::vector<wire::core::DecisionTraceEntry>& trace, wire::core::DecisionTraceTopic topic,
                            const std::string& rule, const std::string& token) {
  for (const auto& entry : trace) {
    if (entry.topic != topic) {
      continue;
    }
    if (!rule.empty() && entry.rule != rule) {
      continue;
    }
    if (entry.summary.find(token) != std::string::npos) {
      return true;
    }
  }
  return false;
}

bool test_backbone_straight_chain_support_axis_stays_perpendicular_to_route() {
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
  if (!state.GenerateFromBackboneSpec(req).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto pole_view = state.view().inspect_pole(center_id);
  if (!pole_view.has_value()) {
    return false;
  }
  const AxisRelationMetrics metrics =
      measure_pole_axis_relation_metrics(state, center_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  double along = 0.0;
  double perp = 0.0;
  const double axis_align = support_axis_alignment_ratio(state, center_id, {0.0, 1.0, 0.0});
  const bool spread = unique_pole_ports_spread_along_axis(state, center_id, wire::core::PortLayer::kHighVoltage,
                                                          {0.0, 1.0, 0.0}, &along, &perp);
  if (!(axis_align > 0.99 && spread && along > 0.2 && perp < along && metrics.valid &&
        metrics.angle_row_vs_span_deg >= 70.0 && metrics.angle_forward_vs_span_deg <= 20.0)) {
    std::cerr << "[DBG] C173 pole=" << center_id << " rule=" << static_cast<int>(pole_view->support_axis_rule)
              << " axis=" << pole_view->support_axis_dir.x << "," << pole_view->support_axis_dir.y
              << " layoutYaw=" << pole_view->layout_yaw_deg << " finalYaw=" << pole_view->final_yaw_deg
              << " align=" << axis_align << " spread=" << (spread ? 1 : 0) << " along=" << along
              << " perp=" << perp << " metrics=" << describe_axis_relation_metrics(metrics) << "\n";
  }
  return axis_align > 0.99 && spread && along > 0.2 && perp < along && metrics.valid &&
         metrics.angle_row_vs_span_deg >= 70.0 && metrics.angle_forward_vs_span_deg <= 20.0;
}

bool test_backbone_cross_junction_support_axis_avoids_diagonal() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec horizontal{};
  horizontal.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  horizontal.interval_m = 1000.0;
  horizontal.pole_type_id = type_ids.front();
  add_backbone_bundle(horizontal, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(horizontal).ok) {
    std::cerr << "[DBG] C174 horizontal_generate_failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec vertical{};
  vertical.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  vertical.interval_m = 1000.0;
  vertical.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  vertical.path.node_specs.push_back(shared);
  add_backbone_bundle(vertical, wire::core::BundleKind::kHighVoltage);
  const auto vertical_generated = state.GenerateFromBackboneSpec(vertical);
  if (!vertical_generated.ok) {
    std::cerr << "[DBG] C174 vertical_generate_failed error=" << vertical_generated.error << "\n";
    return false;
  }

  const auto pole_view = state.view().inspect_pole(center_id);
  if (!pole_view.has_value() || !pole_view->has_support_axis) {
    std::cerr << "[DBG] C174 pole_view_missing pole=" << center_id << "\n";
    return false;
  }
  const AxisRelationMetrics metrics =
      measure_pole_axis_relation_metrics(state, center_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const wire::core::Vec3d diag_a = normalize_xy_safe({1.0, 1.0, 0.0});
  const wire::core::Vec3d diag_b = normalize_xy_safe({1.0, -1.0, 0.0});
  const double diag_dot = std::max(std::abs(dot_xy(normalize_xy_safe(pole_view->support_axis_dir), diag_a)),
                                   std::abs(dot_xy(normalize_xy_safe(pole_view->support_axis_dir), diag_b)));
  if (!(pole_view->support_axis_rule == wire::core::PoleSupportAxisRule::kConnectedDirectionFit && diag_dot > 0.9 &&
        metrics.valid && metrics.angle_row_vs_span_deg > 25.0 && metrics.angle_row_vs_span_deg < 65.0)) {
    std::cerr << "[DBG] C174 pole=" << center_id << " rule=" << static_cast<int>(pole_view->support_axis_rule)
              << " axis=" << pole_view->support_axis_dir.x << "," << pole_view->support_axis_dir.y
              << " layoutYaw=" << pole_view->layout_yaw_deg << " finalYaw=" << pole_view->final_yaw_deg
              << " diagDot=" << diag_dot
              << " metrics=" << describe_axis_relation_metrics(metrics) << "\n";
  }
  return pole_view->support_axis_rule == wire::core::PoleSupportAxisRule::kConnectedDirectionFit && diag_dot > 0.9 &&
         metrics.valid && metrics.angle_row_vs_span_deg > 25.0 && metrics.angle_row_vs_span_deg < 65.0;
}

bool test_backbone_orthogonal_route_support_axis_stays_cardinal() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}, {12.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(req).ok) {
    return false;
  }
  const ObjectId corner_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  const auto pole_view = state.view().inspect_pole(corner_id);
  if (corner_id == wire::core::kInvalidObjectId || !pole_view.has_value() || !pole_view->has_support_axis) {
    return false;
  }
  const AxisRelationMetrics metrics =
      measure_pole_axis_relation_metrics(state, corner_id, wire::core::PortLayer::kLowVoltage, {1.0, 0.0, 0.0});
  const wire::core::Vec3d diag_a = normalize_xy_safe({1.0, 1.0, 0.0});
  const wire::core::Vec3d diag_b = normalize_xy_safe({1.0, -1.0, 0.0});
  const double diag_dot = std::max(std::abs(dot_xy(normalize_xy_safe(pole_view->support_axis_dir), diag_a)),
                                   std::abs(dot_xy(normalize_xy_safe(pole_view->support_axis_dir), diag_b)));
  const bool ok = pole_view->support_axis_rule == wire::core::PoleSupportAxisRule::kConnectedDirectionFit &&
                  diag_dot > 0.9 && metrics.valid &&
                  metrics.angle_row_vs_span_deg > 25.0 && metrics.angle_row_vs_span_deg < 65.0;
  if (!ok) {
    std::cerr << "[DBG] C175 diagDot=" << diag_dot << " metrics=" << describe_axis_relation_metrics(metrics) << "\n";
  }
  return ok;
}

bool test_backbone_branch_keeps_main_support_axis_non_diagonal() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C176 trunk_generate_failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C176 center_missing_after_trunk\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok) {
    std::cerr << "[DBG] C176 branch_generate_failed error=" << branch_generated.error << "\n";
    return false;
  }

  const auto pole_view = state.view().inspect_pole(center_id);
  const AxisRelationMetrics metrics =
      measure_pole_axis_relation_metrics(state, center_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const wire::core::Vec3d support_axis =
      pole_view.has_value() ? normalize_xy_safe(pole_view->support_axis_dir) : wire::core::Vec3d{0.0, 0.0, 0.0};
  const wire::core::Vec3d diag_a = normalize_xy_safe({1.0, 1.0, 0.0});
  const wire::core::Vec3d diag_b = normalize_xy_safe({1.0, -1.0, 0.0});
  const double diag_dot =
      std::max(std::abs(dot_xy(support_axis, diag_a)), std::abs(dot_xy(support_axis, diag_b)));
  if (!(diag_dot > 0.9 && metrics.valid && metrics.angle_row_vs_span_deg >= 70.0)) {
    std::cerr << "[DBG] C176 pole=" << center_id << " rule="
              << static_cast<int>(pole_view.has_value() ? pole_view->support_axis_rule
                                                        : wire::core::PoleSupportAxisRule::kFallback)
              << " axis="
              << (pole_view.has_value() ? pole_view->support_axis_dir.x : 0.0) << ","
              << (pole_view.has_value() ? pole_view->support_axis_dir.y : 0.0)
              << " layoutYaw=" << (pole_view.has_value() ? pole_view->layout_yaw_deg : 0.0)
              << " finalYaw=" << (pole_view.has_value() ? pole_view->final_yaw_deg : 0.0)
              << " diagDot=" << diag_dot << " metrics=" << describe_axis_relation_metrics(metrics) << "\n";
  }
  return diag_dot > 0.9 && metrics.valid && metrics.angle_row_vs_span_deg >= 70.0;
}

bool test_backbone_drawpath_plain_endpoint_fallback_without_attachment_input() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      return false;
    }
    const auto check_endpoint = [](const wire::core::SupportLayoutEndpointView& endpoint) {
      return endpoint.endpoint_source == wire::core::SupportLayoutEndpointSourceKind::kPlainSupport &&
             endpoint.attachment_request.kind == wire::core::EndpointAttachmentRequestKind::kNone &&
             !endpoint.attachment_request.attachment_id.has_value() &&
             !endpoint.attachment_request.requested_socket_id.has_value() && !endpoint.resolved_socket_id.has_value();
    };
    if (!check_endpoint(layout_view->start_endpoint) || !check_endpoint(layout_view->end_endpoint)) {
      return false;
    }
  }
  return true;
}

bool test_backbone_drawpath_attachment_trace_reports_unconnected_input() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);
  const auto trace = state.view().collect_decision_trace(
      {wire::core::EntityKind::kSpan, static_cast<std::uint64_t>(generated.value.generated_span_ids.front())});
  for (const auto& entry : trace) {
    if (entry.topic != wire::core::DecisionTraceTopic::kSupportLayoutSelection ||
        entry.rule != "AttachmentEndpointSelection") {
      continue;
    }
    return entry.summary.find("start=PlainSupport") != std::string::npos &&
           entry.summary.find("end=PlainSupport") != std::string::npos &&
           entry.summary.find("request=None") != std::string::npos;
  }
  return false;
}

bool test_backbone_drawpath_branch_curve_stays_local_to_support_departure() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    std::cerr << "[DBG] C179 trunk_generate_failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C179 center_missing_after_trunk\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 18.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C179 branch_generate_failed error=" << branch_generated.error << "\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);

  const ObjectId span_id = branch_generated.value.generated_span_ids.front();
  const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
  const auto span_view = state.view().inspect_span(span_id);
  const auto curve_view = state.view().inspect_detail_curve(span_id);
  const auto* curve = state.find_curve_cache(span_id);
  if (span == nullptr || !span_view.has_value() || !curve_view.has_value() || curve == nullptr) {
    return false;
  }

  const bool center_is_start = span->endpoint_node_a_id == center_id;
  const auto tangent_rule = center_is_start ? curve_view->start_tangent_rule : curve_view->end_tangent_rule;
  const double support_weight = center_is_start ? curve_view->start_support_weight : curve_view->end_support_weight;
  const double chord_weight = center_is_start ? curve_view->start_chord_weight : curve_view->end_chord_weight;
  const double departure_length_m =
      center_is_start ? curve_view->start_departure_length_m : curve_view->end_departure_length_m;
  const double lateral_ratio_limit =
      center_is_start ? curve_view->start_lateral_ratio_limit : curve_view->end_lateral_ratio_limit;
  const auto branch_trace =
      state.view().collect_decision_trace({wire::core::EntityKind::kSpan, static_cast<std::uint64_t>(span_id)});
  const bool branch_trace_has_tangent_policy =
      trace_contains_summary(branch_trace, wire::core::DecisionTraceTopic::kTangentGeneration, "BranchTangentRule",
                             "suppress=");
  std::size_t peak_index = 0;
  const double max_abs_lateral = max_curve_lateral_overshoot_xy(curve->detail, &peak_index);
  const bool peaks_locally =
      max_abs_lateral <= 0.05 || peak_index < (curve->detail.sample_points.size() / 2);
  const BranchRunoutMetrics runout_metrics = measure_branch_runout_metrics(state, span_id);
  if (!(span_view->flow_kind == wire::core::BackboneFlowKind::kBranch && span_view->uses_branch_support &&
          curve_view->shape_policy == wire::core::CurveShapePolicyKind::kBranchPass &&
          tangent_rule == wire::core::DetailCurveEndpointTangentRule::kBranchChordPriority &&
          support_weight < chord_weight && departure_length_m <= 1.10 + 1e-6 &&
          lateral_ratio_limit <= 0.05 + 1e-6 && curve_view->lateral_suppression >= 0.80 &&
          max_abs_lateral <= departure_length_m + 0.05 && peaks_locally && branch_trace_has_tangent_policy &&
          runout_metrics.lateral_runout_ratio <= 0.08 && runout_metrics.local_departure_dominates)) {
    std::cerr << "[DBG] C179 span=" << span_id << " flow=" << static_cast<int>(span_view->flow_kind)
              << " branchSupport=" << (span_view->uses_branch_support ? 1 : 0)
              << " shape=" << static_cast<int>(curve_view->shape_policy)
              << " tangentRule=" << static_cast<int>(tangent_rule) << " supportWeight=" << support_weight
              << " chordWeight=" << chord_weight << " dep=" << departure_length_m
              << " latLimit=" << lateral_ratio_limit << " suppress=" << curve_view->lateral_suppression
              << " maxLat=" << max_abs_lateral << " peakIndex=" << peak_index
              << " samples=" << curve->detail.sample_points.size()
              << " branchTraceHasPolicy=" << (branch_trace_has_tangent_policy ? 1 : 0)
              << " runout=" << describe_branch_runout_metrics(runout_metrics) << "\n";
  }
  return span_view->flow_kind == wire::core::BackboneFlowKind::kBranch && span_view->uses_branch_support &&
         curve_view->shape_policy == wire::core::CurveShapePolicyKind::kBranchPass &&
         tangent_rule == wire::core::DetailCurveEndpointTangentRule::kBranchChordPriority &&
         support_weight < chord_weight && departure_length_m <= 1.10 + 1e-6 &&
         lateral_ratio_limit <= 0.05 + 1e-6 && curve_view->lateral_suppression >= 0.80 &&
         max_abs_lateral <= departure_length_m + 0.05 && peaks_locally && branch_trace_has_tangent_policy &&
         runout_metrics.lateral_runout_ratio <= 0.08 && runout_metrics.local_departure_dominates;
}

bool test_backbone_drawpath_main_and_branch_are_distinct_in_trace_and_inspection() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  const auto trunk_generated = state.GenerateFromBackboneSpec(trunk);
  if (!trunk_generated.ok || trunk_generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C180 trunk_generate_failed error=" << trunk_generated.error << "\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C180 center_missing_after_trunk\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.empty()) {
    std::cerr << "[DBG] C180 branch_generate_failed error=" << branch_generated.error << "\n";
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);

  const ObjectId main_span_id = trunk_generated.value.generated_span_ids.front();
  const ObjectId branch_span_id = branch_generated.value.generated_span_ids.front();
  const auto main_view = state.view().inspect_span(main_span_id);
  const auto branch_view = state.view().inspect_span(branch_span_id);
  const auto branch_curve_view = state.view().inspect_detail_curve(branch_span_id);
  if (!main_view.has_value() || !branch_view.has_value() || !branch_curve_view.has_value()) {
    return false;
  }
  const BranchRunoutMetrics branch_runout_metrics = measure_branch_runout_metrics(state, branch_span_id);

  const auto main_trace =
      state.view().collect_decision_trace({wire::core::EntityKind::kSpan, static_cast<std::uint64_t>(main_span_id)});
  const auto branch_trace =
      state.view().collect_decision_trace({wire::core::EntityKind::kSpan, static_cast<std::uint64_t>(branch_span_id)});
  const bool branch_has_branch_tangent =
      trace_contains_summary(branch_trace, wire::core::DecisionTraceTopic::kTangentGeneration, "BranchTangentRule",
                               "support/chord=");
  const bool branch_has_branch_flow =
      trace_contains_summary(branch_trace, wire::core::DecisionTraceTopic::kFlowClassification, "", "flow=Branch");
  const bool main_has_main_flow = trace_contains_summary(main_trace, wire::core::DecisionTraceTopic::kFlowClassification,
                                                         "", "flow=Main");
  const bool main_has_main_tangent = trace_contains_summary(main_trace, wire::core::DecisionTraceTopic::kTangentGeneration,
                                                            "MainTangentRule", "support/chord=");
  const bool branch_has_lateral_policy =
      trace_contains_summary(branch_trace, wire::core::DecisionTraceTopic::kTangentGeneration, "BranchTangentRule",
                             "suppress=");
  if (!(main_view->flow_kind == wire::core::BackboneFlowKind::kMain && !main_view->uses_branch_support &&
          branch_view->flow_kind == wire::core::BackboneFlowKind::kBranch && branch_view->uses_branch_support &&
          branch_curve_view->shape_policy == wire::core::CurveShapePolicyKind::kBranchPass && main_has_main_flow &&
          main_has_main_tangent && branch_has_branch_flow && branch_has_branch_tangent &&
          branch_curve_view->lateral_suppression >= 0.80 && branch_has_lateral_policy &&
          branch_runout_metrics.lateral_runout_ratio <= 0.10)) {
    std::cerr << "[DBG] C180 mainSpan=" << main_span_id << " mainFlow=" << static_cast<int>(main_view->flow_kind)
              << " mainBranchSupport=" << (main_view->uses_branch_support ? 1 : 0) << " branchSpan=" << branch_span_id
              << " branchFlow=" << static_cast<int>(branch_view->flow_kind)
              << " branchSupport=" << (branch_view->uses_branch_support ? 1 : 0)
              << " branchShape=" << static_cast<int>(branch_curve_view->shape_policy)
              << " branchSuppress=" << branch_curve_view->lateral_suppression
              << " mainHasFlow=" << (main_has_main_flow ? 1 : 0)
              << " mainHasTangent=" << (main_has_main_tangent ? 1 : 0)
              << " branchHasFlow=" << (branch_has_branch_flow ? 1 : 0)
              << " branchHasTangent=" << (branch_has_branch_tangent ? 1 : 0)
              << " branchHasLateralPolicy=" << (branch_has_lateral_policy ? 1 : 0)
              << " branchRunout=" << describe_branch_runout_metrics(branch_runout_metrics) << "\n";
  }
  return main_view->flow_kind == wire::core::BackboneFlowKind::kMain && !main_view->uses_branch_support &&
         branch_view->flow_kind == wire::core::BackboneFlowKind::kBranch && branch_view->uses_branch_support &&
         branch_curve_view->shape_policy == wire::core::CurveShapePolicyKind::kBranchPass && main_has_main_flow &&
         main_has_main_tangent && branch_has_branch_flow && branch_has_branch_tangent &&
         branch_curve_view->lateral_suppression >= 0.80 && branch_has_lateral_policy &&
         branch_runout_metrics.lateral_runout_ratio <= 0.10;
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
  test_registry::AddTest(tests, "C208_Backbone_IntervalExtensionBoundaryOrder",
                         "Interval-driven backbone extension keeps conductor ordering continuous at reused poles",
                         "Invariant", false, test_backbone_interval_extension_preserves_boundary_lane_order);
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
                         "Captured HV3 backbone shape keeps interior shared-pole lane order continuous",
                         "Invariant", false,
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
  test_registry::AddTest(tests, "C193_Backbone_BranchSupportLowersHV3BundleUniformly",
                         "HV3 branch support lowers the whole branch bundle uniformly while keeping a perpendicular row",
                         "Invariant", false, test_backbone_branch_support_lowers_hv3_bundle_uniformly);
  test_registry::AddTest(tests, "C195_Backbone_BranchSupportStaysLocalToRootPole",
                         "Branch support down-offset stays on the branch root and downstream branch poles return to a flat perpendicular row",
                         "Invariant", false, test_backbone_branch_support_stays_local_to_root_pole);
  test_registry::AddTest(tests, "C196_Backbone_BranchSupportVisualPerpendicular",
                         "Branch support visual placement stays perpendicular to the branch and hangs vertically under the lane attachment",
                         "Invariant", false, test_backbone_branch_support_visual_stays_perpendicular_to_branch);
  test_registry::AddTest(tests, "C197_Backbone_DefaultSingleBranchStaysFlat",
                         "DefaultSingle branch stays branch-flow without automatic branch support or height drop",
                         "Invariant", false, test_backbone_default_single_branch_stays_flat_without_branch_support);
  test_registry::AddTest(tests, "C198_Backbone_CommunicationBundleBranchStaysFlat",
                         "Communication multi-bundle branch stays branch-flow without automatic branch support or height drop",
                         "Invariant", false, test_backbone_communication_bundle_branch_stays_flat_without_branch_support);
  test_registry::AddTest(tests, "C199_Backbone_HV3BranchSupportWorksOnBothPoleTypes",
                         "HV3 branch support stays enabled, uniform-height, and branch-perpendicular on both default pole types",
                         "Invariant", false, test_backbone_hv3_branch_support_policy_applies_on_both_default_pole_types);
  test_registry::AddTest(tests, "C204_Backbone_HV3AcuteCornerLowersBundle",
                         "HV3 acute corner lowers the middle bundle uniformly without turning it into branch support",
                         "Invariant", false, test_backbone_hv3_acute_corner_lowers_corner_bundle_without_branch_support);
  test_registry::AddTest(tests, "C205_Backbone_HV3AcuteCornerLoweringSurvivesPoleRefresh",
                         "HV3 acute-corner lowered ports survive pole refresh without being remapped back to template bands",
                         "Invariant", false, test_backbone_hv3_acute_corner_lowering_survives_pole_refresh);
  test_registry::AddTest(tests, "C209_Backbone_HV3ModerateAcuteCornerLowersBundle",
                         "HV3 moderate acute corner lowers the middle bundle under the default corner threshold",
                         "Invariant", false, test_backbone_hv3_moderate_acute_corner_lowers_bundle_at_default_threshold);
  test_registry::AddTest(tests, "C194_Backbone_JunctionPrefersStraighterMainPair",
                         "Junction main selection prefers the straighter continuation pair over first-drawn primary",
                         "Invariant", false, test_backbone_junction_prefers_straighter_pair_over_first_drawn_primary);
  test_registry::AddTest(tests, "C201_Backbone_CrossJunctionNonMainUsesUnderpass",
                         "Cross junction keeps the opposite pair as main and lowers the non-main line under it",
                         "Invariant", false, test_backbone_cross_junction_nonmain_line_uses_underpass);
  test_registry::AddTest(tests, "C210_Backbone_AllTemplatesBranchStillLowersHVOnCommunicationPole",
                         "All-template DrawPath branch on communication poles still classifies HV as branch support and lowers it",
                         "Invariant", false,
                         test_backbone_all_templates_branch_keeps_hv_down_offset_on_communication_pole);
  test_registry::AddTest(tests, "C211_Backbone_AllTemplatesCrossStillUsesUnderpassOnCommunicationPole",
                         "All-template DrawPath cross on communication poles still keeps at least one non-main line lowered",
                         "Invariant", false,
                         test_backbone_all_templates_cross_keeps_underpass_on_communication_pole);
  test_registry::AddTest(tests, "C212_Backbone_CaptureBranchThenAcuteLoweringOnCommunicationPole",
                         "Captured all-template communication-pole path keeps HV branch lowering on the first segment and acute lowering on later segments",
                         "Invariant", false, test_backbone_capture_branch_then_acute_lowering_on_communication_pole);
  test_registry::AddTest(tests, "C136_Backbone_HV3MainPortsStableAfterBranch",
                         "Adding an HV3 branch keeps existing main-chain ports stable", "Invariant", false,
                         test_backbone_branch_generation_preserves_existing_hv3_main_ports);
  test_registry::AddTest(tests, "C185_Backbone_HV3TerminalSlotsDistinct",
                         "HV3 DrawPath terminals realize distinct template bands instead of fallback category ports",
                         "Invariant", false, test_backbone_hv3_terminal_poles_use_distinct_template_bands);
  test_registry::AddTest(tests, "C188_Backbone_HV3TerminalFallbackRowPerpendicular",
                         "HV3 DrawPath terminals without HV bands still realize a perpendicular generated row without generic fallback",
                         "Invariant", false, test_backbone_hv3_terminal_fallback_ports_still_spread_perpendicular);
  test_registry::AddTest(tests, "C189_Backbone_HV3CommunicationPoleAllTemplatesTerminalRow",
                         "HV3 remains visually separated and perpendicular on communication poles even when all DrawPath templates are selected",
                         "Invariant", false,
                         test_backbone_hv3_terminals_stay_perpendicular_on_communication_pole_with_all_templates);
  test_registry::AddTest(tests, "C190_Backbone_CommunicationTerminalRowPerpendicular",
                         "Communication multi-lane terminals stay visually separated and perpendicular on communication poles",
                         "Invariant", false, test_backbone_communication_multilane_terminals_stay_perpendicular);
  test_registry::AddTest(tests, "C192_Backbone_ClickedExistingCommunicationPoles_AllTemplatesStaySeparated",
                         "Clicked existing communication poles keep HV terminals visually separated and perpendicular when all DrawPath templates are selected",
                         "Invariant", false,
                         test_backbone_clicked_existing_communication_poles_all_templates_keep_hv_terminal_separation);
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
  test_registry::AddTest(tests, "C173_Backbone_StraightSupportAxisPerpendicularToRoute",
                         "Straight DrawPath keeps pole support axis and port row perpendicular to the route axis",
                         "Invariant", false, test_backbone_straight_chain_support_axis_stays_perpendicular_to_route);
  test_registry::AddTest(tests, "C174_Backbone_CrossSupportAxisNotDiagonal",
                         "Cross junction fits support axis to connected directions and resolves to a diagonal support heading",
                         "Invariant", false, test_backbone_cross_junction_support_axis_avoids_diagonal);
  test_registry::AddTest(tests, "C175_Backbone_OrthogonalSupportAxisStaysCardinal",
                         "Orthogonal DrawPath corner fits support axis to connected directions instead of freezing to a cardinal axis",
                         "Invariant", false, test_backbone_orthogonal_route_support_axis_stays_cardinal);
  test_registry::AddTest(tests, "C176_Backbone_BranchKeepsMainSupportAxis",
                         "Adding a branch turns support heading toward connected directions while keeping the row geometry readable",
                         "Invariant", false, test_backbone_branch_keeps_main_support_axis_non_diagonal);
  test_registry::AddTest(tests, "C177_Backbone_DrawPathPlainEndpointFallback",
                         "DrawPath backbone without attachment input falls back to plain support endpoints",
                         "Invariant", false, test_backbone_drawpath_plain_endpoint_fallback_without_attachment_input);
  test_registry::AddTest(tests, "C178_Backbone_DrawPathAttachmentTraceReportsNoInput",
                         "DrawPath backbone trace reports plain support endpoint selection when attachment input is absent",
                         "Invariant", false, test_backbone_drawpath_attachment_trace_reports_unconnected_input);
  test_registry::AddTest(tests, "C179_Backbone_DrawPathBranchCurveStaysLocal",
                         "DrawPath branch curve keeps support departure local and converges back toward chord",
                         "Invariant", false, test_backbone_drawpath_branch_curve_stays_local_to_support_departure);
  test_registry::AddTest(tests, "C180_Backbone_DrawPathMainBranchTraceReadable",
                         "DrawPath span inspection and trace make main/branch differences explicit",
                         "Invariant", false, test_backbone_drawpath_main_and_branch_are_distinct_in_trace_and_inspection);
  test_registry::AddTest(tests, "C155_Variation_DoesNotAffectTopologyOrMirror",
                         "Variation settings do not change deterministic flow classification or mirror decisions",
                         "Invariant", false, test_variation_settings_do_not_change_topology_flow_or_mirror);
}

WIRE_REGISTER_TEST_SUITE(register_generation_tests);

} // namespace
