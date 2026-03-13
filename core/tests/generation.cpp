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

  wire::core::ResolveBranchPickOptions resolve{};
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

  wire::core::ResolveBranchPickOptions resolve{};
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

bool test_backbone_hv3_terminal_poles_use_distinct_template_slots() {
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
    bool has_hv_slot = false;
    for (const auto& slot : pole_type.port_slots) {
      if (slot.enabled && slot.category == wire::core::ConnectionCategory::kHighVoltage) {
        has_hv_slot = true;
        break;
      }
    }
    if (!has_hv_slot) {
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
    if (port.owner_pole_id == pole_id && port.layer == layer) {
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
  const wire::core::Vec3d diag = normalize_xy_safe({1.0, 1.0, 0.0});
  const double axis_align = support_axis_alignment_ratio(state, center_id, {0.0, 1.0, 0.0});
  const double diag_dot = std::abs(dot_xy(normalize_xy_safe(pole_view->support_axis_dir), diag));
  if (!(pole_view->support_axis_rule == wire::core::PoleSupportAxisRule::kMainChainPair && axis_align > 0.99 &&
        diag_dot < 0.8 && metrics.valid && metrics.angle_row_vs_span_deg >= 70.0)) {
    std::cerr << "[DBG] C174 pole=" << center_id << " rule=" << static_cast<int>(pole_view->support_axis_rule)
              << " axis=" << pole_view->support_axis_dir.x << "," << pole_view->support_axis_dir.y
              << " layoutYaw=" << pole_view->layout_yaw_deg << " finalYaw=" << pole_view->final_yaw_deg
              << " align=" << axis_align << " diagDot=" << diag_dot
              << " metrics=" << describe_axis_relation_metrics(metrics) << "\n";
  }
  return pole_view->support_axis_rule == wire::core::PoleSupportAxisRule::kMainChainPair && axis_align > 0.99 &&
         diag_dot < 0.8 && metrics.valid && metrics.angle_row_vs_span_deg >= 70.0;
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
  const double card_x = support_axis_alignment_ratio(state, corner_id, {1.0, 0.0, 0.0});
  const double card_y = support_axis_alignment_ratio(state, corner_id, {0.0, 1.0, 0.0});
  const bool ok = pole_view->support_axis_rule == wire::core::PoleSupportAxisRule::kMainChainPair &&
                  std::max(card_x, card_y) > 0.99 && metrics.valid;
  if (!ok) {
    std::cerr << "[DBG] C175 metrics=" << describe_axis_relation_metrics(metrics) << " cardX=" << card_x
              << " cardY=" << card_y << "\n";
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

  double along = 0.0;
  double perp = 0.0;
  const auto pole_view = state.view().inspect_pole(center_id);
  const AxisRelationMetrics metrics =
      measure_pole_axis_relation_metrics(state, center_id, wire::core::PortLayer::kHighVoltage, {1.0, 0.0, 0.0});
  const double axis_align = support_axis_alignment_ratio(state, center_id, {0.0, 1.0, 0.0});
  const bool spread = unique_pole_ports_spread_along_axis(state, center_id, wire::core::PortLayer::kHighVoltage,
                                                          {0.0, 1.0, 0.0}, &along, &perp);
  if (!(axis_align > 0.99 && spread && along > 0.2 && perp < along && metrics.valid &&
        metrics.angle_row_vs_span_deg >= 70.0)) {
    std::cerr << "[DBG] C176 pole=" << center_id << " rule="
              << static_cast<int>(pole_view.has_value() ? pole_view->support_axis_rule
                                                        : wire::core::PoleSupportAxisRule::kFallback)
              << " axis="
              << (pole_view.has_value() ? pole_view->support_axis_dir.x : 0.0) << ","
              << (pole_view.has_value() ? pole_view->support_axis_dir.y : 0.0)
              << " layoutYaw=" << (pole_view.has_value() ? pole_view->layout_yaw_deg : 0.0)
              << " finalYaw=" << (pole_view.has_value() ? pole_view->final_yaw_deg : 0.0)
              << " align=" << axis_align << " spread=" << (spread ? 1 : 0) << " along=" << along
              << " perp=" << perp << " metrics=" << describe_axis_relation_metrics(metrics) << "\n";
  }
  return axis_align > 0.99 && spread && along > 0.2 && perp < along && metrics.valid &&
         metrics.angle_row_vs_span_deg >= 70.0;
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
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
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
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.size() != 1) {
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
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
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
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
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
  test_registry::AddTest(tests, "C136_Backbone_HV3MainPortsStableAfterBranch",
                         "Adding an HV3 branch keeps existing main-chain ports stable", "Invariant", false,
                         test_backbone_branch_generation_preserves_existing_hv3_main_ports);
  test_registry::AddTest(tests, "C185_Backbone_HV3TerminalSlotsDistinct",
                         "HV3 DrawPath terminals realize distinct template slots instead of fallback category ports",
                         "Invariant", false, test_backbone_hv3_terminal_poles_use_distinct_template_slots);
  test_registry::AddTest(tests, "C188_Backbone_HV3TerminalFallbackRowPerpendicular",
                         "HV3 DrawPath terminals without HV slots still realize a perpendicular generated row without generic fallback",
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
                         "Cross junction keeps the main support axis cardinal instead of diagonal",
                         "Invariant", false, test_backbone_cross_junction_support_axis_avoids_diagonal);
  test_registry::AddTest(tests, "C175_Backbone_OrthogonalSupportAxisStaysCardinal",
                         "Orthogonal DrawPath corner chooses a cardinal support axis instead of a bisector",
                         "Invariant", false, test_backbone_orthogonal_route_support_axis_stays_cardinal);
  test_registry::AddTest(tests, "C176_Backbone_BranchKeepsMainSupportAxis",
                         "Adding a branch does not diagonalize the existing main support axis or port row",
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
