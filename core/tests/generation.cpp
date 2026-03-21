#include <array>
#include <cmath>
#include <iostream>
#include <optional>
#include <sstream>
#include <unordered_set>
#include <vector>

#include "registry.hpp"
#include "helpers.hpp"

using namespace helpers;

std::optional<wire::core::SupportLayoutEndpointView> layout_endpoint_for_owner(
    const wire::core::SupportLayoutInspectionView& layout_view, wire::core::ObjectId owner_pole_id);
std::optional<wire::core::LoweredSupportGroupInspectionView> lowered_support_group_for_owner(
  const wire::core::SupportLayoutInspectionView& layout_view, wire::core::ObjectId owner_pole_id);
std::optional<wire::core::SegmentLaneAssignment> find_assignment_for_span(const wire::core::CoreState& state,
                                                                           wire::core::ObjectId span_id);

bool endpoint_has_authoritative_lowering(const wire::core::SupportLayoutEndpointView& endpoint) {
  return endpoint.decision.lower_required && !endpoint.decision.lowering_blocked_by_policy &&
         endpoint.decision.support_group_id >= 0;
}

const wire::core::EndpointContinuityDecision* assignment_decision_for_pole(
    const wire::core::SegmentLaneAssignment& assignment, wire::core::ObjectId pole_id) {
  if (assignment.pole_a_id == pole_id) {
    return &assignment.decision_a;
  }
  if (assignment.pole_b_id == pole_id) {
    return &assignment.decision_b;
  }
  return nullptr;
}

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
  const auto backbone = state.BuildBackboneResult();
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
  const auto first_backbone = state.BuildBackboneResult();
  const auto* existing_midair = find_support_node_by_point_index(first_backbone, 1);
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
  const auto second_backbone = state.BuildBackboneResult();
  const auto* reused = find_support_node_by_point_index(second_backbone, 0);
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
  const auto first_backbone = state.BuildBackboneResult();
  const auto* existing_midair = find_support_node_by_point_index(first_backbone, 1);
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
  const auto first_backbone = state.BuildBackboneResult();
  const auto* existing_midair = find_support_node_by_point_index(first_backbone, 1);
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
  const auto first_backbone = state.BuildBackboneResult();
  const auto* existing_midair = find_support_node_by_point_index(first_backbone, 1);
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
  const auto& orientations = state.view().last_generation_edge_orientations();
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
  const auto& orientations = state.view().last_generation_edge_orientations();
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

  const auto& assignments = state.view().last_lane_assignments();
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
  const auto& first_orientations = state.view().last_generation_edge_orientations();
  if (first_orientations.empty()) {
    return false;
  }
  const auto* tail = &first_orientations.back();

  const auto second = state.GenerateFromBackboneSpec(make_request(extended_path));
  if (!second.ok) {
    return false;
  }
  const auto& second_orientations = state.view().last_generation_edge_orientations();
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
  const auto first_assignments = state.view().last_lane_assignments();
  if (count_bundle_lane_adjacent_order_discontinuities(state, first_assignments) != 0) {
    return false;
  }

  const auto second = state.GenerateFromBackboneSpec(make_request(extended_path));
  if (!second.ok) {
    return false;
  }
  const auto second_assignments = state.view().last_lane_assignments();
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

  int lowered_center_endpoints = 0;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      return false;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    const auto group = lowered_support_group_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || !group.has_value() || !endpoint_has_authoritative_lowering(*endpoint) ||
        endpoint->relation_kind != wire::core::JunctionRelationKind::kSideBranch ||
        group->support_group_id != endpoint->decision.support_group_id || endpoint->branch_down_offset_m <= 1e-6) {
      return false;
    }
    ++lowered_center_endpoints;
  }
  return lowered_center_endpoints == 3;
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
  int support_group_id = -1;
  double down_offset_m = -1.0;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      return false;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    const auto group = lowered_support_group_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || !group.has_value() || !endpoint_has_authoritative_lowering(*endpoint) ||
        endpoint->relation_kind != wire::core::JunctionRelationKind::kSideBranch) {
      return false;
    }
    if (support_group_id < 0) {
      support_group_id = endpoint->decision.support_group_id;
      down_offset_m = group->down_offset_m;
    } else if (support_group_id != endpoint->decision.support_group_id ||
               !almost_equal(down_offset_m, group->down_offset_m, 1e-9)) {
      return false;
    }
  }
  const bool perpendicular_row = x_span > y_span * 2.0;
  const bool uniform_height = (max_z - min_z) <= 1e-6;
  const bool has_grouped_lowering = support_group_id >= 0 && down_offset_m > 1e-6;
  if (!uniform_height || !has_grouped_lowering) {
    std::cerr << "[DBG] C193 z0=" << z0 << " z1=" << z1 << " z2=" << z2 << " zSpan=" << (max_z - min_z)
              << " xSpan=" << x_span << " ySpan=" << y_span << " groupId=" << support_group_id
              << " downOffset=" << down_offset_m << "\n";
    }
  return uniform_height && perpendicular_row && max_z + 1e-6 < main_min_z && has_grouped_lowering;
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

  std::vector<const wire::core::Port*> downstream_mid_ports{};
  int root_lowered_count = 0;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (span == nullptr || !layout_view.has_value()) {
      return false;
    }
    if (const auto root_endpoint = layout_endpoint_for_owner(*layout_view, center_id); root_endpoint.has_value()) {
      if (!endpoint_has_authoritative_lowering(*root_endpoint) ||
          root_endpoint->relation_kind != wire::core::JunctionRelationKind::kSideBranch ||
          !lowered_support_group_for_owner(*layout_view, center_id).has_value()) {
        return false;
      }
      ++root_lowered_count;
    }
    if (const auto mid_endpoint = layout_endpoint_for_owner(*layout_view, mid_id); mid_endpoint.has_value()) {
      if (endpoint_has_authoritative_lowering(*mid_endpoint) ||
          lowered_support_group_for_owner(*layout_view, mid_id).has_value()) {
        return false;
      }
    }
    const bool touches_mid = span->endpoint_node_a_id == mid_id || span->endpoint_node_b_id == mid_id;
    const bool touches_center = span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id;
    if (touches_mid && !touches_center) {
      const ObjectId port_id = span->endpoint_node_a_id == mid_id ? span->port_a_id : span->port_b_id;
      const auto* port = state.view().edit_state().ports.find(port_id);
      if (port == nullptr) {
        return false;
      }
      downstream_mid_ports.push_back(port);
    }
  }
  if (root_lowered_count != 3 || downstream_mid_ports.size() != 3) {
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

  bool found = false;
  for (ObjectId span_id : branch_generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto placement = lowered_support_group_for_owner(*layout_view, center_id);
    if (!placement.has_value()) {
      continue;
    }
    found = true;
    const wire::core::Vec3d support_axis = normalize_xy_safe(placement->tip_world - placement->mount_world);
    const wire::core::Vec3d expected_axis = normalize_xy_safe(placement->side_axis);
    const double axis_alignment = std::abs(dot_xy(support_axis, expected_axis));
    if (axis_alignment < 0.95) {
      const double support_start_xy =
          std::sqrt(std::pow(placement->mount_world.x - center_pole->world_transform.position.x, 2.0) +
                    std::pow(placement->mount_world.y - center_pole->world_transform.position.y, 2.0));
      const wire::core::Vec3d attachment_world =
          placement->attachment_worlds.empty() ? placement->tip_world : placement->attachment_worlds.front();
      const wire::core::Vec3d hanger_delta = attachment_world - placement->tip_world;
      const double hanger_xy = std::sqrt(hanger_delta.x * hanger_delta.x + hanger_delta.y * hanger_delta.y);
      std::cerr << "[DBG] C196 axisAlignment=" << axis_alignment << " hangerXY=" << hanger_xy
                << " supportStartXY=" << support_start_xy << " mount=(" << placement->mount_world.x << ","
                << placement->mount_world.y << "," << placement->mount_world.z << ") tip=(" << placement->tip_world.x
                << "," << placement->tip_world.y << "," << placement->tip_world.z << ") attach=("
                << attachment_world.x << "," << attachment_world.y << "," << attachment_world.z << ") sideSign="
                << placement->chosen_side_sign << " origin=" << placement->origin
                << " sideRule=" << static_cast<int>(placement->side_assignment_rule)
                << " orientRule=" << static_cast<int>(placement->support_orientation_rule)
                << " hasSideAxis=" << placement->has_side_axis << " sideAxis=(" << placement->side_axis.x << ","
                << placement->side_axis.y << "," << placement->side_axis.z << ")\n";
      return false;
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

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);
  const auto span_view = state.view().inspect_span(branch_generated.value.generated_span_ids.front());
  const auto layout_view = state.view().inspect_support_layout(branch_generated.value.generated_span_ids.front());
  if (!span_view.has_value() || !layout_view.has_value()) {
    return false;
  }
  const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
  if (!endpoint.has_value()) {
    return false;
  }
  if (!(span_view->flow_kind == wire::core::BackboneFlowKind::kBranch && !endpoint->decision.lower_required &&
        endpoint->decision.support_group_id < 0 && endpoint->branch_down_offset_m == 0.0 &&
        !lowered_support_group_for_owner(*layout_view, center_id).has_value())) {
    std::cerr << "[DBG] C197 flow=" << static_cast<int>(span_view->flow_kind)
              << " lowerRequired=" << endpoint->decision.lower_required
              << " groupId=" << endpoint->decision.support_group_id
              << " downOffset=" << endpoint->branch_down_offset_m << "\n";
  }
  return span_view->flow_kind == wire::core::BackboneFlowKind::kBranch && !endpoint->decision.lower_required &&
         endpoint->decision.support_group_id < 0 && endpoint->branch_down_offset_m == 0.0 &&
         !lowered_support_group_for_owner(*layout_view, center_id).has_value();
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

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  (void)state.Commit(options);
  const auto span_view = state.view().inspect_span(branch_generated.value.generated_span_ids.front());
  const auto layout_view = state.view().inspect_support_layout(branch_generated.value.generated_span_ids.front());
  if (!span_view.has_value() || !layout_view.has_value()) {
    return false;
  }
  const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
  if (!endpoint.has_value()) {
    return false;
  }
  if (!(span_view->flow_kind == wire::core::BackboneFlowKind::kBranch && endpoint->decision.lower_required &&
        endpoint->decision.support_group_id < 0 && endpoint->decision.lowering_blocked_by_policy &&
        endpoint->branch_down_offset_m == 0.0 &&
        !lowered_support_group_for_owner(*layout_view, center_id).has_value())) {
    std::cerr << "[DBG] C198 flow=" << static_cast<int>(span_view->flow_kind)
              << " lowerRequired=" << endpoint->decision.lower_required
              << " groupId=" << endpoint->decision.support_group_id
              << " blocked=" << endpoint->decision.lowering_blocked_by_policy
              << " downOffset=" << endpoint->branch_down_offset_m
              << " generatedSpanCount=" << branch_generated.value.generated_span_ids.size() << "\n";
  }
  return span_view->flow_kind == wire::core::BackboneFlowKind::kBranch && endpoint->decision.lower_required &&
         endpoint->decision.support_group_id < 0 && endpoint->decision.lowering_blocked_by_policy &&
         endpoint->branch_down_offset_m == 0.0 &&
         !lowered_support_group_for_owner(*layout_view, center_id).has_value();
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
    const auto trunk_generated = state.GenerateFromBackboneSpec(trunk);
    if (!trunk_generated.ok) {
      std::cerr << "[DBG] C199 trunk_generate_failed type=" << pole_type_name << "\n";
      return false;
    }

    const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
    if (center_id == wire::core::kInvalidObjectId) {
      return false;
    }
    std::unordered_set<ObjectId> main_port_ids{};
    for (ObjectId span_id : trunk_generated.value.generated_span_ids) {
      const auto* span = state.view().edit_state().spans.find(span_id);
      if (span == nullptr) {
        return false;
      }
      if (span->endpoint_node_a_id == center_id) {
        main_port_ids.insert(span->port_a_id);
      }
      if (span->endpoint_node_b_id == center_id) {
        main_port_ids.insert(span->port_b_id);
      }
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

    const auto* center_pole = state.view().edit_state().poles.find(center_id);
    if (center_pole == nullptr) {
      return false;
    }

    std::vector<const wire::core::Port*> root_ports{};
    double main_min_z = std::numeric_limits<double>::infinity();
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id != center_id || port.layer != wire::core::PortLayer::kHighVoltage ||
          !main_port_ids.contains(port.id)) {
        continue;
      }
      main_min_z = std::min(main_min_z, port.world_position.z);
    }
    int support_group_id = -1;
    double down_offset_m = -1.0;
    for (ObjectId span_id : branch_generated.value.generated_span_ids) {
      const auto* span = state.view().edit_state().spans.find(span_id);
      const auto layout_view = state.view().inspect_support_layout(span_id);
      if (span == nullptr || !layout_view.has_value()) {
        return false;
      }
      const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
      const auto group = lowered_support_group_for_owner(*layout_view, center_id);
      if (!endpoint.has_value() || !group.has_value() || !endpoint_has_authoritative_lowering(*endpoint) ||
          endpoint->relation_kind != wire::core::JunctionRelationKind::kSideBranch) {
        std::cerr << "[DBG] C199 branch_policy_missing type=" << pole_type_name << "\n";
        return false;
      }
      if (support_group_id < 0) {
        support_group_id = endpoint->decision.support_group_id;
        down_offset_m = group->down_offset_m;
      } else if (support_group_id != endpoint->decision.support_group_id ||
                 !almost_equal(down_offset_m, group->down_offset_m, 1e-9)) {
        return false;
      }
      const ObjectId port_id = span->endpoint_node_a_id == center_id ? span->port_a_id : span->port_b_id;
      const auto* port = state.view().edit_state().ports.find(port_id);
      if (port == nullptr) {
        return false;
      }
      root_ports.push_back(port);
    }
    if (root_ports.size() != 3 || !std::isfinite(main_min_z) || support_group_id < 0 || down_offset_m <= 1e-6) {
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

    bool found_support = false;
    for (ObjectId span_id : branch_generated.value.generated_span_ids) {
      const auto layout_view = state.view().inspect_support_layout(span_id);
      if (!layout_view.has_value()) {
        continue;
      }
      const auto placement = lowered_support_group_for_owner(*layout_view, center_id);
      if (!placement.has_value()) {
        continue;
      }
      found_support = true;
      const wire::core::Vec3d support_axis = normalize_xy_safe(placement->tip_world - placement->mount_world);
      const wire::core::Vec3d expected_axis = normalize_xy_safe(placement->side_axis);
      const double axis_alignment = std::abs(dot_xy(support_axis, expected_axis));
      if (axis_alignment < 0.97) {
        std::cerr << "[DBG] C199 support_axis_failed type=" << pole_type_name
                  << " axisAlignment=" << axis_alignment << " mount=(" << placement->mount_world.x << ","
                  << placement->mount_world.y << "," << placement->mount_world.z << ") tip=(" << placement->tip_world.x
                  << "," << placement->tip_world.y << "," << placement->tip_world.z << ") sideSign="
                  << placement->chosen_side_sign << " origin=" << placement->origin
                  << " sideRule=" << static_cast<int>(placement->side_assignment_rule)
                  << " orientRule=" << static_cast<int>(placement->support_orientation_rule)
                  << " hasSideAxis=" << placement->has_side_axis << " sideAxis=(" << placement->side_axis.x << ","
                  << placement->side_axis.y << "," << placement->side_axis.z << ")\n";
        return false;
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

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr &&
        ((span->endpoint_node_a_id == center_id && span->endpoint_node_b_id == next_id) ||
         (span->endpoint_node_a_id == next_id && span->endpoint_node_b_id == center_id))) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto before_layout = state.view().inspect_support_layout(target_span_id);
  const auto before_endpoint = before_layout.has_value() ? layout_endpoint_for_owner(*before_layout, center_id) : std::nullopt;
  const auto before_group = before_layout.has_value() ? lowered_support_group_for_owner(*before_layout, center_id) : std::nullopt;
  if (!before_endpoint.has_value() || !before_group.has_value()) {
    return false;
  }

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
  const auto after_layout = state.view().inspect_support_layout(target_span_id);
  const auto after_endpoint = after_layout.has_value() ? layout_endpoint_for_owner(*after_layout, center_id) : std::nullopt;
  const auto after_group = after_layout.has_value() ? lowered_support_group_for_owner(*after_layout, center_id) : std::nullopt;

  const bool ok = std::isfinite(after_center_z) && std::isfinite(after_neighbor_z) &&
                  after_center_z + 1e-6 < after_neighbor_z && after_endpoint.has_value() && after_group.has_value() &&
                  endpoint_has_authoritative_lowering(*after_endpoint) &&
                  before_endpoint->decision.support_group_id == after_endpoint->decision.support_group_id &&
                  almost_equal(before_group->mount_world.z, after_group->mount_world.z, 1e-6) &&
                  almost_equal(before_group->tip_world.z, after_group->tip_world.z, 1e-6);
  if (!ok) {
    std::cerr << "[DBG] C205 after centerZ=" << after_center_z << " neighborZ=" << after_neighbor_z
              << " beforeGroup=" << before_endpoint->decision.support_group_id
              << " afterGroup=" << (after_endpoint.has_value() ? after_endpoint->decision.support_group_id : -1) << "\n";
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

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    std::cerr << "[DBG] C201 junction inspect missing\n";
    return false;
  }

  bool has_point_like_branch = false;
  bool has_same_level_cross_relation = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const bool touches_center = assignment.pole_a_id == center_id || assignment.pole_b_id == center_id;
    if (!touches_center) {
      continue;
    }
    has_point_like_branch =
        has_point_like_branch ||
        (assignment.flow_kind == wire::core::BackboneFlowKind::kBranch &&
         assignment.flow_decision_rule == wire::core::BackboneFlowDecisionRule::kJunctionOrderBranch &&
         assignment.continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
         assignment.same_level_feasible &&
         assignment.lowering_kind == wire::core::BackboneLoweringKind::kNone);
  }

  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCrossUnderpass &&
        relation.continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
        !relation.default_lower_required && relation.same_level_feasible) {
      has_same_level_cross_relation = true;
      break;
    }
  }
  const bool ok = has_point_like_branch && has_same_level_cross_relation && std::isfinite(main_center_z);
  if (!ok) {
    std::cerr << "[DBG] C201 pointLikeBranch=" << (has_point_like_branch ? 1 : 0)
              << " sameLevelCrossRelation=" << (has_same_level_cross_relation ? 1 : 0)
              << " mainCenterZ=" << main_center_z
              << " centerYaw=" << pole_view->final_yaw_deg << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C201 relation neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " class=" << static_cast<int>(relation.continuity_class)
                << " defaultLower=" << (relation.default_lower_required ? 1 : 0)
                << " sameLevel=" << (relation.same_level_feasible ? 1 : 0)
                << " reason=" << static_cast<int>(relation.infeasible_reason) << "\n";
    }
    for (const auto& assignment : state.view().last_lane_assignments()) {
      std::cerr << "[DBG] C201 assignment poles=" << assignment.pole_a_id << "->" << assignment.pole_b_id
                << " flow=" << static_cast<int>(assignment.flow_kind)
                << " rule=" << static_cast<int>(assignment.flow_decision_rule)
                << " class=" << static_cast<int>(assignment.continuity_class)
                << " defaultLower=" << (assignment.default_lower_required ? 1 : 0)
                << " usesBranchSupport=" << (assignment.uses_branch_support ? 1 : 0)
                << " solver=" << (assignment.solver_used_same_level_constraint ? 1 : 0)
                << " special=" << (assignment.used_special_case_ports ? 1 : 0)
                << " lowering=" << static_cast<int>(assignment.lowering_kind)
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
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr ||
        (span->endpoint_node_a_id != center_id && span->endpoint_node_b_id != center_id)) {
      continue;
    }
    const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    hv_branch_found = true;
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    const auto group = lowered_support_group_for_owner(*layout_view, center_id);
    if (endpoint.has_value() && group.has_value() && endpoint_has_authoritative_lowering(*endpoint) &&
        endpoint->relation_kind == wire::core::JunctionRelationKind::kSideBranch &&
        endpoint->branch_down_offset_m > 1e-6) {
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
                << " lowerA=" << assignment.decision_a.lower_required
                << " lowerB=" << assignment.decision_b.lower_required
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
  const wire::core::EndpointContinuityDecision* first_root_decision = assignment_decision_for_pole(*first_assignment, node0);
  const bool first_is_branch_support =
      first_assignment->segment_index == 0 &&
      first_assignment->pole_a_id == node0 && first_assignment->pole_b_id == node1 &&
      first_root_decision != nullptr && first_root_decision->relation_kind == wire::core::JunctionRelationKind::kSideBranch &&
      first_root_decision->lower_required && first_root_decision->support_group_id >= 0 &&
      first_assignment->branch_down_offset_m > 1e-6;
  int acute_after_root_count = 0;
  bool acute_touches_explicit_corner = false;
  for (const auto* assignment : hv_assignments) {
    if (assignment->segment_index == 0) {
      continue;
    }
    const bool has_corner_lower =
        (assignment->decision_a.relation_kind == wire::core::JunctionRelationKind::kCornerContinuation &&
         assignment->decision_a.lower_required && assignment->decision_a.support_group_id >= 0) ||
        (assignment->decision_b.relation_kind == wire::core::JunctionRelationKind::kCornerContinuation &&
         assignment->decision_b.lower_required && assignment->decision_b.support_group_id >= 0);
    if (!has_corner_lower || assignment->branch_down_offset_m <= 1e-6) {
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
                << " relA=" << static_cast<int>(assignment->decision_a.relation_kind)
                << " relB=" << static_cast<int>(assignment->decision_b.relation_kind)
                << " lowerA=" << assignment->decision_a.lower_required
                << " lowerB=" << assignment->decision_b.lower_required
                << " down=" << assignment->branch_down_offset_m << "\n";
    }
  }

  return first_is_branch_support && acute_after_root_count >= 1 && acute_touches_explicit_corner && root_lowered &&
         acute_center_lowered;
}

bool test_inspection_all_templates_branch_keeps_hv_lowering_on_communication_pole() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C264 missing CommunicationPole\n";
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
    std::cerr << "[DBG] C264 trunk generate failed\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C264 center missing\n";
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
    std::cerr << "[DBG] C264 branch generate failed error=" << generated.error << "\n";
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr ||
        (span->endpoint_node_a_id != center_id && span->endpoint_node_b_id != center_id)) {
      continue;
    }
    const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == wire::core::BundleKind::kHighVoltage) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C264 target HV span missing\n";
    return false;
  }

  const auto span_view = state.view().inspect_span(target_span_id);
  const auto layout_view = state.view().inspect_support_layout(target_span_id);
  if (!span_view.has_value() || !layout_view.has_value()) {
    std::cerr << "[DBG] C264 inspection missing span=" << (span_view.has_value() ? 1 : 0)
              << " layout=" << (layout_view.has_value() ? 1 : 0) << "\n";
    return false;
  }

  const auto endpoint_is_branch_root = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
    return endpoint.owner_pole_id == center_id && endpoint.relation_kind == wire::core::JunctionRelationKind::kSideBranch &&
           endpoint_has_authoritative_lowering(endpoint) && endpoint.branch_down_offset_m > 1e-6;
  };

  const auto group = lowered_support_group_for_owner(*layout_view, center_id);
  const wire::core::SupportLayoutEndpointView* lowered_endpoint =
      endpoint_is_branch_root(layout_view->start_endpoint)
          ? &layout_view->start_endpoint
          : (endpoint_is_branch_root(layout_view->end_endpoint) ? &layout_view->end_endpoint : nullptr);
  const bool ok = span_view->flow_kind == wire::core::BackboneFlowKind::kBranch &&
                  span_view->branch_down_offset_m > 1e-6 &&
                  layout_view->flow_kind == wire::core::BackboneFlowKind::kBranch &&
                  group.has_value() && lowered_endpoint != nullptr &&
                  group->support_group_id == lowered_endpoint->decision.support_group_id &&
                  almost_equal(group->down_offset_m, lowered_endpoint->branch_down_offset_m, 1e-6) &&
                  almost_equal(lowered_endpoint->support_world.z, group->mount_world.z, 1e-6) &&
                  almost_equal(lowered_endpoint->support_world.z, group->tip_world.z, 1e-6);
  if (!ok) {
    std::cerr << "[DBG] C264 spanFlow=" << static_cast<int>(span_view->flow_kind)
              << " spanDown=" << span_view->branch_down_offset_m
              << " layoutFlow=" << static_cast<int>(layout_view->flow_kind)
              << " startRelation=" << static_cast<int>(layout_view->start_endpoint.relation_kind)
              << " startDown=" << layout_view->start_endpoint.branch_down_offset_m
              << " endRelation=" << static_cast<int>(layout_view->end_endpoint.relation_kind)
              << " endDown=" << layout_view->end_endpoint.branch_down_offset_m
              << " groupId=" << (group.has_value() ? group->support_group_id : -1)
              << " endpointZ=" << (lowered_endpoint != nullptr ? lowered_endpoint->support_world.z : -1.0)
              << " mountZ=" << (group.has_value() ? group->mount_world.z : -1.0)
              << " tipZ=" << (group.has_value() ? group->tip_world.z : -1.0) << "\n";
  }
  return ok;
}

bool test_inspection_capture_keeps_branch_then_acute_lowering_on_communication_pole() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
    std::cerr << "[DBG] C265 missing CommunicationPole\n";
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
    std::cerr << "[DBG] C265 trunk generate failed error=" << trunk_generated.error << "\n";
    return false;
  }

  const ObjectId node0 = find_pole_id_by_position(state, {-6.59678, 11.0534, 0.0}, 1e-4);
  if (node0 == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C265 shared root missing\n";
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
    std::cerr << "[DBG] C265 branch generate failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId node1 = find_pole_id_by_position(state, {-4.93216, 6.7054, 0.0}, 1e-4);
  const ObjectId node2 = find_pole_id_by_position(state, {-13.6709, -1.24875, 0.0}, 1e-4);
  const ObjectId node3 = find_pole_id_by_position(state, {-8.49996, -0.441201, 0.0}, 1e-4);
  if (node1 == wire::core::kInvalidObjectId || node2 == wire::core::kInvalidObjectId ||
      node3 == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C265 node lookup failed\n";
    return false;
  }

  ObjectId branch_span_id = wire::core::kInvalidObjectId;
  ObjectId corner_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      continue;
    }
    const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    const bool is_root_segment =
        (span->endpoint_node_a_id == node0 && span->endpoint_node_b_id == node1) ||
        (span->endpoint_node_a_id == node1 && span->endpoint_node_b_id == node0);
    const bool is_corner_segment =
        ((span->endpoint_node_a_id == node1 && span->endpoint_node_b_id == node2) ||
         (span->endpoint_node_a_id == node2 && span->endpoint_node_b_id == node1) ||
         (span->endpoint_node_a_id == node2 && span->endpoint_node_b_id == node3) ||
         (span->endpoint_node_a_id == node3 && span->endpoint_node_b_id == node2));
    if (is_root_segment) {
      branch_span_id = span_id;
    } else if (corner_span_id == wire::core::kInvalidObjectId && is_corner_segment) {
      corner_span_id = span_id;
    }
  }
  if (branch_span_id == wire::core::kInvalidObjectId || corner_span_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C265 target spans missing branch=" << branch_span_id << " corner=" << corner_span_id << "\n";
    return false;
  }

  const auto branch_span = state.view().inspect_span(branch_span_id);
  const auto branch_layout = state.view().inspect_support_layout(branch_span_id);
  const auto corner_span = state.view().inspect_span(corner_span_id);
  const auto corner_layout = state.view().inspect_support_layout(corner_span_id);
  if (!branch_span.has_value() || !branch_layout.has_value() || !corner_span.has_value() || !corner_layout.has_value()) {
    std::cerr << "[DBG] C265 missing inspection branchSpan=" << (branch_span.has_value() ? 1 : 0)
              << " branchLayout=" << (branch_layout.has_value() ? 1 : 0)
              << " cornerSpan=" << (corner_span.has_value() ? 1 : 0)
              << " cornerLayout=" << (corner_layout.has_value() ? 1 : 0) << "\n";
    return false;
  }

  struct LoweredSnapshot {
    double support_z = std::numeric_limits<double>::quiet_NaN();
    double mount_z = std::numeric_limits<double>::quiet_NaN();
    double tip_z = std::numeric_limits<double>::quiet_NaN();
    double down_offset_m = 0.0;
  };
  const auto collect_lowered_snapshot = [&](const wire::core::SupportLayoutInspectionView& layout, ObjectId owner_pole_id,
                                            wire::core::JunctionRelationKind relation_kind)
      -> std::optional<LoweredSnapshot> {
    const wire::core::SupportLayoutEndpointView* endpoint = nullptr;
    if (layout.start_endpoint.owner_pole_id == owner_pole_id && layout.start_endpoint.relation_kind == relation_kind &&
        endpoint_has_authoritative_lowering(layout.start_endpoint)) {
      endpoint = &layout.start_endpoint;
    } else if (layout.end_endpoint.owner_pole_id == owner_pole_id && layout.end_endpoint.relation_kind == relation_kind &&
               endpoint_has_authoritative_lowering(layout.end_endpoint)) {
      endpoint = &layout.end_endpoint;
    }
    if (endpoint == nullptr) {
      return std::nullopt;
    }
    const auto group = lowered_support_group_for_owner(layout, owner_pole_id);
    if (!group.has_value()) {
      return std::nullopt;
    }
    LoweredSnapshot snapshot{};
    snapshot.support_z = endpoint->support_world.z;
    snapshot.mount_z = group->mount_world.z;
    snapshot.tip_z = group->tip_world.z;
    snapshot.down_offset_m = endpoint->branch_down_offset_m;
    return snapshot;
  };

  const auto branch_group = lowered_support_group_for_owner(*branch_layout, node0);
  const auto corner_group_node1 = lowered_support_group_for_owner(*corner_layout, node1);
  const auto corner_group_node2 = lowered_support_group_for_owner(*corner_layout, node2);
  const auto branch_snapshot =
      collect_lowered_snapshot(*branch_layout, node0, wire::core::JunctionRelationKind::kSideBranch);
  const auto corner_snapshot_node1 =
      collect_lowered_snapshot(*corner_layout, node1, wire::core::JunctionRelationKind::kCornerContinuation);
  const auto corner_snapshot_node2 =
      collect_lowered_snapshot(*corner_layout, node2, wire::core::JunctionRelationKind::kCornerContinuation);
  const std::optional<LoweredSnapshot> corner_snapshot =
      corner_snapshot_node1.has_value() ? corner_snapshot_node1 : corner_snapshot_node2;
  const bool branch_ok = branch_span->flow_kind == wire::core::BackboneFlowKind::kBranch &&
                         branch_span->branch_down_offset_m > 1e-6 &&
                         branch_group.has_value() && branch_snapshot.has_value();
  const bool corner_ok = corner_span->branch_down_offset_m > 1e-6 &&
                         std::max(corner_layout->start_endpoint.branch_down_offset_m,
                                  corner_layout->end_endpoint.branch_down_offset_m) > 1e-6 &&
                         (corner_group_node1.has_value() || corner_group_node2.has_value()) &&
                         corner_snapshot.has_value();
  const bool shared_one_step_height =
      branch_snapshot.has_value() && corner_snapshot.has_value() &&
      almost_equal(branch_snapshot->support_z, branch_snapshot->mount_z, 1e-6) &&
      almost_equal(branch_snapshot->support_z, branch_snapshot->tip_z, 1e-6) &&
      almost_equal(corner_snapshot->support_z, corner_snapshot->mount_z, 1e-6) &&
      almost_equal(corner_snapshot->support_z, corner_snapshot->tip_z, 1e-6) &&
      almost_equal(branch_snapshot->support_z, corner_snapshot->support_z, 1e-6) &&
      almost_equal(branch_snapshot->mount_z, corner_snapshot->mount_z, 1e-6) &&
      almost_equal(branch_snapshot->tip_z, corner_snapshot->tip_z, 1e-6) &&
      almost_equal(branch_snapshot->down_offset_m, corner_snapshot->down_offset_m, 1e-6);
  if (!(branch_ok && corner_ok && shared_one_step_height)) {
    std::cerr << "[DBG] C265 branchFlow=" << static_cast<int>(branch_span->flow_kind)
              << " branchDown=" << branch_span->branch_down_offset_m
              << " branchRelA=" << static_cast<int>(branch_layout->start_endpoint.relation_kind)
              << " branchRelB=" << static_cast<int>(branch_layout->end_endpoint.relation_kind)
              << " cornerSpanDown=" << corner_span->branch_down_offset_m
              << " cornerStartDown=" << corner_layout->start_endpoint.branch_down_offset_m
              << " cornerEndDown=" << corner_layout->end_endpoint.branch_down_offset_m
              << " branchZ=" << (branch_snapshot.has_value() ? branch_snapshot->support_z : -1.0)
              << " cornerZ=" << (corner_snapshot.has_value() ? corner_snapshot->support_z : -1.0)
              << " branchOffset=" << (branch_snapshot.has_value() ? branch_snapshot->down_offset_m : -1.0)
              << " cornerOffset=" << (corner_snapshot.has_value() ? corner_snapshot->down_offset_m : -1.0) << "\n";
  }

  return branch_ok && corner_ok && shared_one_step_height;
}

bool test_backbone_right_angle_junction_has_no_through_pair() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C213 generate_failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const auto junction = state.view().inspect_junction(center_id);
  if (center_id == wire::core::kInvalidObjectId || !junction.has_value()) {
    std::cerr << "[DBG] C213 center_or_junction_missing center=" << center_id
              << " centerNode=" << static_cast<long long>(center_id)
              << " hasJunction=" << (junction.has_value() ? 1 : 0) << "\n";
    return false;
  }

  bool has_corner = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCornerContinuation) {
      has_corner = true;
      break;
    }
  }
  const bool ok = !junction->through_pair_accepted && junction->through_pair_straightness_score >= 0.0 &&
                  junction->through_pair_straightness_score < 0.3 && has_corner;
  if (!ok) {
    std::cerr << "[DBG] C213 accepted=" << (junction->through_pair_accepted ? 1 : 0)
              << " score=" << junction->through_pair_straightness_score
              << " routeCount=" << junction->route_incident_count << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C213 rel neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " inRoute=" << (relation.in_route ? 1 : 0)
                << " same=" << (relation.same_level_feasible ? 1 : 0) << "\n";
    }
  }
  return ok;
}

bool test_inspection_span_reads_flow_and_turn_from_lane_snapshot() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() < 2) {
    std::cerr << "[DBG] C266 generate_failed ok=" << (generated.ok ? 1 : 0)
              << " spanCount=" << generated.value.generated_span_ids.size()
              << " error=" << generated.error << "\n";
    return false;
  }

  const wire::core::ObjectId target_span_id = generated.value.generated_span_ids.back();
  const auto assignment = find_assignment_for_span(state, target_span_id);
  const auto span_view = state.view().inspect_span(target_span_id);
  if (!assignment.has_value() || !span_view.has_value()) {
    std::cerr << "[DBG] C266 missing assignment=" << (assignment.has_value() ? 1 : 0)
              << " spanView=" << (span_view.has_value() ? 1 : 0) << "\n";
    return false;
  }

  const bool ok = span_view->flow_kind == assignment->flow_kind &&
                  span_view->flow_rule == assignment->flow_decision_rule &&
                  span_view->flipped_from_previous == assignment->flipped_from_previous &&
                  almost_equal(span_view->turn_angle_deg, assignment->turn_angle_deg, 1e-9);
  if (!ok) {
    std::cerr << "[DBG] C266 spanFlow=" << static_cast<int>(span_view->flow_kind)
              << " assignmentFlow=" << static_cast<int>(assignment->flow_kind)
              << " spanRule=" << static_cast<int>(span_view->flow_rule)
              << " assignmentRule=" << static_cast<int>(assignment->flow_decision_rule)
              << " spanFlip=" << (span_view->flipped_from_previous ? 1 : 0)
              << " assignmentFlip=" << (assignment->flipped_from_previous ? 1 : 0)
              << " spanTurn=" << span_view->turn_angle_deg
              << " assignmentTurn=" << assignment->turn_angle_deg << "\n";
  }
  return ok;
}

bool test_backbone_local_corner_projects_to_main_without_local_through() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C214 generate_failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const auto junction = state.view().inspect_junction(center_id);
  if (center_id == wire::core::kInvalidObjectId || !junction.has_value()) {
    std::cerr << "[DBG] C214 center_or_junction_missing center=" << center_id
              << " centerNode=" << static_cast<long long>(center_id)
              << " hasJunction=" << (junction.has_value() ? 1 : 0) << "\n";
    return false;
  }

  bool has_corner = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCornerContinuation) {
      has_corner = true;
      break;
    }
  }

  int main_touch_count = 0;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        assignment.flow_kind == wire::core::BackboneFlowKind::kMain) {
      ++main_touch_count;
    }
  }
  const bool ok = !junction->through_pair_accepted && has_corner && main_touch_count == 2;
  if (!ok) {
    std::cerr << "[DBG] C214 accepted=" << (junction->through_pair_accepted ? 1 : 0)
              << " mainTouchCount=" << main_touch_count << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C214 rel neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " inRoute=" << (relation.in_route ? 1 : 0) << "\n";
    }
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) {
        std::cerr << "[DBG] C214 assignment flow=" << static_cast<int>(assignment.flow_kind)
                  << " rule=" << static_cast<int>(assignment.flow_decision_rule)
                  << " lowering=" << static_cast<int>(assignment.lowering_kind) << "\n";
      }
    }
  }
  return ok;
}

bool test_backbone_separate_route_merge_keeps_corner_continuation_relation() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-18.59678, 11.0534, 0.0}, {-6.59678, 11.0534, 0.0}, {5.40322, 11.0534, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  const auto trunk_generated = state.GenerateFromBackboneSpec(trunk);
  if (!trunk_generated.ok) {
    std::cerr << "[DBG] C215 trunk_generate_failed error=" << trunk_generated.error << "\n";
    return false;
  }

  const ObjectId root_id = find_pole_id_by_position(state, {-6.59678, 11.0534, 0.0}, 1e-4);
  if (root_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C215 root_missing\n";
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{-6.59678, 11.0534, 0.0}, {-4.93216, 6.7054, 0.0}, {-13.6709, -1.24875, 0.0}, {-8.49996, -0.441201, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = root_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 8.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    std::cerr << "[DBG] C215 branch_generate_failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId corner_id = find_pole_id_by_position(state, {-13.6709, -1.24875, 0.0}, 1e-4);
  const auto junction = state.view().inspect_junction(corner_id);
  if (corner_id == wire::core::kInvalidObjectId || !junction.has_value()) {
    std::cerr << "[DBG] C215 corner_or_junction_missing corner=" << corner_id
              << " cornerNode=" << static_cast<long long>(corner_id)
              << " hasJunction=" << (junction.has_value() ? 1 : 0) << "\n";
    return false;
  }

  bool has_corner = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCornerContinuation) {
      has_corner = true;
      break;
    }
  }

  bool has_acute_lowering = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == corner_id || assignment.pole_b_id == corner_id) &&
        assignment.lowering_kind == wire::core::BackboneLoweringKind::kAcuteCorner &&
        !assignment.same_level_feasible) {
      has_acute_lowering = true;
      break;
    }
  }
  const bool ok = !junction->through_pair_accepted && has_corner && has_acute_lowering;
  if (!ok) {
    std::cerr << "[DBG] C215 accepted=" << (junction->through_pair_accepted ? 1 : 0)
              << " routeCount=" << junction->route_incident_count << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C215 rel neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " inRoute=" << (relation.in_route ? 1 : 0)
                << " same=" << (relation.same_level_feasible ? 1 : 0)
                << " reason=" << static_cast<int>(relation.infeasible_reason) << "\n";
    }
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id == corner_id || assignment.pole_b_id == corner_id) {
        std::cerr << "[DBG] C215 assignment flow=" << static_cast<int>(assignment.flow_kind)
                  << " relA=" << static_cast<int>(assignment.relation_a)
                  << " relB=" << static_cast<int>(assignment.relation_b)
                  << " same=" << (assignment.same_level_feasible ? 1 : 0)
                  << " lowering=" << static_cast<int>(assignment.lowering_kind) << "\n";
      }
    }
  }
  return ok;
}

bool test_backbone_explicit_middle_bent_route_stays_corner_main_against_existing_chain() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {12.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec bent{};
  bent.path.polyline = {{4.0, -2.0, 0.0}, {12.0, 0.0, 0.0}, {16.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  bent.path.node_specs.push_back(shared);
  bent.interval_m = 1000.0;
  bent.pole_type_id = type_ids.front();
  add_backbone_bundle(bent, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(bent);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    return false;
  }

  int route_main_like_count = 0;
  int route_cross_count = 0;
  int route_branch_count = 0;
  for (const auto& relation : junction->local_relations) {
    if (!relation.in_route) {
      continue;
    }
    if (relation.kind == wire::core::JunctionRelationKind::kCornerContinuation ||
        relation.kind == wire::core::JunctionRelationKind::kThroughMain) {
      ++route_main_like_count;
    }
    if (relation.kind == wire::core::JunctionRelationKind::kCrossUnderpass) {
      ++route_cross_count;
    }
    if (relation.kind == wire::core::JunctionRelationKind::kSideBranch) {
      ++route_branch_count;
    }
  }

  int generated_touch_count = 0;
  int generated_main_count = 0;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const bool touches_center = assignment.pole_a_id == center_id || assignment.pole_b_id == center_id;
    if (!touches_center) {
      continue;
    }
    ++generated_touch_count;
    if (assignment.flow_kind == wire::core::BackboneFlowKind::kMain) {
      ++generated_main_count;
    }
  }

  const bool ok = route_main_like_count == 2 && route_cross_count == 0 && route_branch_count == 0 &&
                  generated_touch_count > 0 && generated_main_count == generated_touch_count;
  if (!ok) {
    std::cerr << "[DBG] C270 accepted=" << (junction->through_pair_accepted ? 1 : 0)
              << " routeMainLikeCount=" << route_main_like_count
              << " routeCrossCount=" << route_cross_count
              << " routeBranchCount=" << route_branch_count
              << " generatedTouchCount=" << generated_touch_count
              << " generatedMainCount=" << generated_main_count << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C270 rel neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " inRoute=" << (relation.in_route ? 1 : 0) << "\n";
    }
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) {
        std::cerr << "[DBG] C270 assignment flow=" << static_cast<int>(assignment.flow_kind)
                  << " relA=" << static_cast<int>(assignment.relation_a)
                  << " relB=" << static_cast<int>(assignment.relation_b)
                  << " lowerA=" << assignment.decision_a.lower_required
                  << " lowerB=" << assignment.decision_b.lower_required << "\n";
      }
    }
  }
  return ok;
}

bool test_backbone_mirror_does_not_change_relation_or_lowering_root() {
  struct MirrorSnapshot {
    bool ok = false;
    bool through_pair_accepted = false;
    std::vector<wire::core::JunctionRelationKind> relations{};
    std::vector<wire::core::BackboneLoweringKind> lowering_kinds{};
    std::vector<bool> same_level_flags{};
    std::vector<wire::core::SameLevelFeasibilityReason> same_level_reasons{};
    std::vector<double> down_offsets{};
  };

  auto run_case = [](bool allow_mirror) {
    MirrorSnapshot snapshot{};
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return snapshot;
    }

    auto tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kHighVoltage);
    if (tpl_it == state.view().bundle_templates().end()) {
      std::cerr << "[DBG] C216 template_missing allowMirror=" << (allow_mirror ? 1 : 0) << "\n";
      return snapshot;
    }
    wire::core::BundleTemplate tpl = tpl_it->second;
    tpl.allow_mirror = allow_mirror;
    const auto apply = state.UpdateBundleTemplate(tpl);
    if (!apply.ok) {
      std::cerr << "[DBG] C216 template_update_failed allowMirror=" << (allow_mirror ? 1 : 0)
                << " error=" << apply.error << "\n";
      return snapshot;
    }

    wire::core::BackboneSpec req{};
    req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
    req.interval_m = 1000.0;
    req.pole_type_id = type_ids.front();
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    const auto generated = state.GenerateFromBackboneSpec(req);
    if (!generated.ok) {
      std::cerr << "[DBG] C216 generate_failed allowMirror=" << (allow_mirror ? 1 : 0)
                << " error=" << generated.error << "\n";
      return snapshot;
    }

    const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
    const auto junction = state.view().inspect_junction(center_id);
    if (center_id == wire::core::kInvalidObjectId || !junction.has_value()) {
      std::cerr << "[DBG] C216 center_or_junction_missing allowMirror=" << (allow_mirror ? 1 : 0)
                << " center=" << center_id
                << " centerNode=" << static_cast<long long>(center_id)
                << " hasJunction=" << (junction.has_value() ? 1 : 0) << "\n";
      return snapshot;
    }

    snapshot.ok = true;
    snapshot.through_pair_accepted = junction->through_pair_accepted;
    for (const auto& relation : junction->local_relations) {
      if (relation.in_route) {
        snapshot.relations.push_back(relation.kind);
      }
    }
    std::sort(snapshot.relations.begin(), snapshot.relations.end());

    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id != center_id && assignment.pole_b_id != center_id) {
        continue;
      }
      snapshot.lowering_kinds.push_back(assignment.lowering_kind);
      snapshot.same_level_flags.push_back(assignment.same_level_feasible);
      snapshot.same_level_reasons.push_back(assignment.same_level_reason);
      snapshot.down_offsets.push_back(assignment.branch_down_offset_m);
    }
    return snapshot;
  };

  const MirrorSnapshot no_mirror = run_case(false);
  const MirrorSnapshot with_mirror = run_case(true);
  const bool ok = no_mirror.ok && with_mirror.ok &&
         no_mirror.through_pair_accepted == with_mirror.through_pair_accepted &&
         no_mirror.relations == with_mirror.relations &&
         no_mirror.lowering_kinds == with_mirror.lowering_kinds &&
         no_mirror.same_level_flags == with_mirror.same_level_flags &&
         no_mirror.same_level_reasons == with_mirror.same_level_reasons &&
         no_mirror.down_offsets == with_mirror.down_offsets;
  if (!ok) {
    std::cerr << "[DBG] C216 ok0=" << (no_mirror.ok ? 1 : 0) << " ok1=" << (with_mirror.ok ? 1 : 0)
              << " accepted0=" << (no_mirror.through_pair_accepted ? 1 : 0)
              << " accepted1=" << (with_mirror.through_pair_accepted ? 1 : 0)
              << " relCount0=" << no_mirror.relations.size()
              << " relCount1=" << with_mirror.relations.size()
              << " lowerCount0=" << no_mirror.lowering_kinds.size()
              << " lowerCount1=" << with_mirror.lowering_kinds.size() << "\n";
  }
  return ok;
}

bool test_backbone_cross_through_pair_can_still_be_same_level_infeasible() {
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
  if (!trunk_generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    return false;
  }

  int infeasible_cross_relations = 0;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCrossUnderpass &&
        !relation.same_level_feasible) {
      ++infeasible_cross_relations;
    }
  }

  int lowered_cross_assignments = 0;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        assignment.lowering_kind == wire::core::BackboneLoweringKind::kCrossUnderpass &&
        !assignment.same_level_feasible) {
      ++lowered_cross_assignments;
    }
  }
  return junction->through_pair_accepted && infeasible_cross_relations >= 1 && lowered_cross_assignments >= 1;
}

bool test_backbone_comm_branch_same_level_can_be_blocked_by_policy() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    return false;
  }

  bool has_infeasible_side_branch = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kSideBranch &&
        !relation.same_level_feasible) {
      has_infeasible_side_branch = true;
      break;
    }
  }

  bool has_policy_block = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kCommunication) {
      continue;
    }
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        !assignment.same_level_feasible &&
        assignment.same_level_reason == wire::core::SameLevelFeasibilityReason::kCategoryPolicyDisabled &&
        assignment.lowering_blocked_by_policy &&
        assignment.lowering_kind == wire::core::BackboneLoweringKind::kNone) {
      has_policy_block = true;
      break;
    }
  }
  return has_infeasible_side_branch && has_policy_block;
}

bool test_backbone_hv3_corner_feasibility_lowering_keeps_semantic_main() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    std::cerr << "[DBG] C219 generate_failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const auto junction = state.view().inspect_junction(center_id);
  if (center_id == wire::core::kInvalidObjectId || !junction.has_value()) {
    std::cerr << "[DBG] C219 center_or_junction_missing center=" << center_id
              << " centerNode=" << static_cast<long long>(center_id)
              << " hasJunction=" << (junction.has_value() ? 1 : 0) << "\n";
    return false;
  }

  bool has_corner = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCornerContinuation &&
        !relation.same_level_feasible) {
      has_corner = true;
      break;
    }
  }

  int acute_main_assignments = 0;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        assignment.flow_kind == wire::core::BackboneFlowKind::kMain &&
        assignment.lowering_kind == wire::core::BackboneLoweringKind::kAcuteCorner &&
        !assignment.same_level_feasible) {
      ++acute_main_assignments;
    }
  }
  const bool ok = !junction->through_pair_accepted && has_corner && acute_main_assignments == 2;
  if (!ok) {
    std::cerr << "[DBG] C219 accepted=" << (junction->through_pair_accepted ? 1 : 0)
              << " acuteMainAssignments=" << acute_main_assignments << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C219 rel neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " inRoute=" << (relation.in_route ? 1 : 0)
                << " same=" << (relation.same_level_feasible ? 1 : 0)
                << " reason=" << static_cast<int>(relation.infeasible_reason) << "\n";
    }
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) {
        std::cerr << "[DBG] C219 assignment flow=" << static_cast<int>(assignment.flow_kind)
                  << " relA=" << static_cast<int>(assignment.relation_a)
                  << " relB=" << static_cast<int>(assignment.relation_b)
                  << " same=" << (assignment.same_level_feasible ? 1 : 0)
                  << " lowering=" << static_cast<int>(assignment.lowering_kind)
                  << " down=" << assignment.branch_down_offset_m << "\n";
      }
    }
  }
  return ok;
}

bool test_backbone_acute_merge_feasibility_applies_across_route_boundary() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-18.59678, 11.0534, 0.0}, {-6.59678, 11.0534, 0.0}, {5.40322, 11.0534, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId root_id = find_pole_id_by_position(state, {-6.59678, 11.0534, 0.0}, 1e-4);
  if (root_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{-6.59678, 11.0534, 0.0}, {-4.93216, 6.7054, 0.0}, {-13.6709, -1.24875, 0.0}, {-8.49996, -0.441201, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = root_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 8.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    return false;
  }

  bool found_acute_infeasible = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if (assignment.lowering_kind == wire::core::BackboneLoweringKind::kAcuteCorner &&
        !assignment.same_level_feasible &&
        (assignment.relation_a == wire::core::JunctionRelationKind::kCornerContinuation ||
         assignment.relation_b == wire::core::JunctionRelationKind::kCornerContinuation)) {
      found_acute_infeasible = true;
      break;
    }
  }
  return found_acute_infeasible;
}

bool test_backbone_recalc_keeps_same_level_lowering_origin() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && (span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id)) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto span_view = state.view().inspect_span(target_span_id);
  const auto layout_view = state.view().inspect_support_layout(target_span_id);
  if (!span_view.has_value() || !layout_view.has_value()) {
    return false;
  }

  return !span_view->same_level_feasible &&
         span_view->lowering_kind == wire::core::BackboneLoweringKind::kAcuteCorner &&
         !layout_view->same_level_feasible &&
         layout_view->lowering_kind == wire::core::BackboneLoweringKind::kAcuteCorner &&
         (layout_view->relation_a == wire::core::JunctionRelationKind::kCornerContinuation ||
          layout_view->relation_b == wire::core::JunctionRelationKind::kCornerContinuation);
}

bool test_backbone_hv3_corner_uses_constrained_band_solver() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  bool constrained_assignment = false;
  bool constrained_port = false;
  bool special_case_used = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id != center_id && assignment.pole_b_id != center_id) ||
        assignment.lowering_kind != wire::core::BackboneLoweringKind::kAcuteCorner ||
        assignment.same_level_feasible) {
      continue;
    }
    constrained_assignment =
        constrained_assignment || (assignment.solver_used_same_level_constraint && !assignment.used_special_case_ports);
    special_case_used = special_case_used || assignment.used_special_case_ports;
    for (ObjectId port_id : assignment.port_ids_a) {
      const auto* port = state.view().edit_state().ports.find(port_id);
      if (port != nullptr && port->owner_pole_id == center_id &&
          port->placement_source == wire::core::PortPlacementSourceKind::kPlacementBandConstrained) {
        constrained_port = true;
      }
    }
    for (ObjectId port_id : assignment.port_ids_b) {
      const auto* port = state.view().edit_state().ports.find(port_id);
      if (port != nullptr && port->owner_pole_id == center_id &&
          port->placement_source == wire::core::PortPlacementSourceKind::kPlacementBandConstrained) {
        constrained_port = true;
      }
    }
  }
  return constrained_assignment && constrained_port && !special_case_used;
}

bool test_backbone_cross_same_level_infeasible_can_use_constrained_solver() {
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
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value() || !junction->through_pair_accepted) {
    return false;
  }

  bool constrained_cross_assignment = false;
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id != center_id && assignment.pole_b_id != center_id) ||
        assignment.lowering_kind != wire::core::BackboneLoweringKind::kCrossUnderpass ||
        assignment.same_level_feasible) {
      continue;
    }
    if (assignment.solver_used_same_level_constraint && !assignment.used_special_case_ports) {
      constrained_cross_assignment = true;
      break;
    }
  }
  if (!constrained_cross_assignment) {
    std::cerr << "[DBG] C223 no constrained cross assignment\n";
    for (const auto& assignment : state.view().last_lane_assignments()) {
      const bool touches_center = assignment.pole_a_id == center_id || assignment.pole_b_id == center_id;
      std::cerr << "[DBG] C223 assignment poles=" << assignment.pole_a_id << "->" << assignment.pole_b_id
                << " touchesCenter=" << (touches_center ? 1 : 0)
                << " lowering=" << static_cast<int>(assignment.lowering_kind)
                << " sameLevel=" << (assignment.same_level_feasible ? 1 : 0)
                << " solver=" << (assignment.solver_used_same_level_constraint ? 1 : 0)
                << " special=" << (assignment.used_special_case_ports ? 1 : 0)
                << " relA=" << static_cast<int>(assignment.relation_a)
                << " relB=" << static_cast<int>(assignment.relation_b) << "\n";
    }
  }
  return constrained_cross_assignment;
}

bool test_backbone_policy_blocked_unresolved_survives_recalc_inspection() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && (span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id)) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto span_view = state.view().inspect_span(target_span_id);
  const auto layout_view = state.view().inspect_support_layout(target_span_id);
  if (!span_view.has_value() || !layout_view.has_value()) {
    return false;
  }

  return !span_view->same_level_feasible &&
         span_view->same_level_reason == wire::core::SameLevelFeasibilityReason::kCategoryPolicyDisabled &&
         span_view->lowering_blocked_by_policy &&
         span_view->unresolved_same_level_conflict &&
         span_view->lowering_kind == wire::core::BackboneLoweringKind::kNone &&
         !layout_view->same_level_feasible &&
         layout_view->same_level_reason == wire::core::SameLevelFeasibilityReason::kCategoryPolicyDisabled &&
         layout_view->lowering_blocked_by_policy &&
         layout_view->unresolved_same_level_conflict &&
         layout_view->lowering_kind == wire::core::BackboneLoweringKind::kNone &&
         (layout_view->start_endpoint.unresolved_same_level_conflict ||
          layout_view->end_endpoint.unresolved_same_level_conflict);
}

bool test_backbone_refresh_keeps_placement_constraint_origin() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }
  if (!state.SetPoleManualYawOverride(center_id, 15.0).ok) {
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && (span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id)) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto layout_view = state.view().inspect_support_layout(target_span_id);
  if (!layout_view.has_value()) {
    return false;
  }

  const bool start_constrained = layout_view->start_endpoint.origin == "PlacementConstraint" &&
                                 layout_view->start_endpoint.port_source == "PlacementBandConstrained";
  const bool end_constrained = layout_view->end_endpoint.origin == "PlacementConstraint" &&
                               layout_view->end_endpoint.port_source == "PlacementBandConstrained";
  return layout_view->solver_used_same_level_constraint &&
         !layout_view->used_special_case_ports &&
         layout_view->lowering_kind == wire::core::BackboneLoweringKind::kAcuteCorner &&
         (layout_view->relation_a == wire::core::JunctionRelationKind::kCornerContinuation ||
          layout_view->relation_b == wire::core::JunctionRelationKind::kCornerContinuation) &&
         (start_constrained || end_constrained);
}

bool test_backbone_mirror_does_not_change_constrained_solver_usage() {
  struct MirrorSolverSnapshot {
    bool ok = false;
    std::vector<bool> solver_flags{};
    std::vector<bool> special_flags{};
    std::vector<bool> unresolved_flags{};
    std::vector<wire::core::SameLevelFeasibilityReason> reasons{};
    std::vector<wire::core::BackboneLoweringKind> lowering{};
  };

  auto run_case = [](bool allow_mirror) {
    MirrorSolverSnapshot snapshot{};
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return snapshot;
    }

    auto tpl_it = state.view().bundle_templates().find(wire::core::BundleKind::kHighVoltage);
    if (tpl_it == state.view().bundle_templates().end()) {
      return snapshot;
    }
    wire::core::BundleTemplate tpl = tpl_it->second;
    tpl.allow_mirror = allow_mirror;
    if (!state.UpdateBundleTemplate(tpl).ok) {
      return snapshot;
    }

    wire::core::BackboneSpec req{};
    req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
    req.interval_m = 1000.0;
    req.pole_type_id = type_ids.front();
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    if (!state.GenerateFromBackboneSpec(req).ok) {
      return snapshot;
    }

    const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
    if (center_id == wire::core::kInvalidObjectId) {
      return snapshot;
    }

    snapshot.ok = true;
    for (const auto& assignment : state.view().last_lane_assignments()) {
      if (assignment.pole_a_id != center_id && assignment.pole_b_id != center_id) {
        continue;
      }
      snapshot.solver_flags.push_back(assignment.solver_used_same_level_constraint);
      snapshot.special_flags.push_back(assignment.used_special_case_ports);
      snapshot.unresolved_flags.push_back(assignment.unresolved_same_level_conflict);
      snapshot.reasons.push_back(assignment.same_level_reason);
      snapshot.lowering.push_back(assignment.lowering_kind);
    }
    return snapshot;
  };

  const MirrorSolverSnapshot no_mirror = run_case(false);
  const MirrorSolverSnapshot with_mirror = run_case(true);
  return no_mirror.ok && with_mirror.ok &&
         no_mirror.solver_flags == with_mirror.solver_flags &&
         no_mirror.special_flags == with_mirror.special_flags &&
         no_mirror.unresolved_flags == with_mirror.unresolved_flags &&
         no_mirror.reasons == with_mirror.reasons &&
         no_mirror.lowering == with_mirror.lowering;
}

bool test_backbone_cross_relation_survives_support_layout_recalc() {
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
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && (span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id)) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto span_view = state.view().inspect_span(target_span_id);
  const auto layout_view = state.view().inspect_support_layout(target_span_id);
  if (!span_view.has_value() || !layout_view.has_value()) {
    return false;
  }

  return !span_view->same_level_feasible &&
         span_view->lowering_kind == wire::core::BackboneLoweringKind::kCrossUnderpass &&
         !layout_view->same_level_feasible &&
         layout_view->lowering_kind == wire::core::BackboneLoweringKind::kCrossUnderpass &&
         (layout_view->relation_a == wire::core::JunctionRelationKind::kCrossUnderpass ||
          layout_view->relation_b == wire::core::JunctionRelationKind::kCrossUnderpass);
}

bool test_backbone_hv3_branch_default_lower_required() {
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
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(branch).ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    return false;
  }

  bool has_bundle_rule_branch = false;
  bool has_lowered_branch_assignment = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kSideBranch &&
        relation.continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
        relation.default_lower_required && !relation.same_level_feasible &&
        relation.infeasible_reason == wire::core::SameLevelFeasibilityReason::kBundleRule) {
      has_bundle_rule_branch = true;
      break;
    }
  }
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        assignment.continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
        assignment.default_lower_required &&
        ((assignment.decision_a.owner_pole_id == center_id && assignment.decision_a.lower_required &&
          assignment.decision_a.support_group_id >= 0) ||
         (assignment.decision_b.owner_pole_id == center_id && assignment.decision_b.lower_required &&
          assignment.decision_b.support_group_id >= 0)) &&
        !assignment.same_level_feasible) {
      has_lowered_branch_assignment = true;
      break;
    }
  }
  return has_bundle_rule_branch && has_lowered_branch_assignment;
}

bool test_backbone_hv3_corner_continuation_default_lower_required() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(req).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const auto junction = state.view().inspect_junction(center_id);
  if (center_id == wire::core::kInvalidObjectId || !junction.has_value()) {
    return false;
  }

  bool has_bundle_rule_corner = false;
  int lowered_corner_assignments = 0;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kCornerContinuation &&
        relation.continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
        relation.default_lower_required && !relation.same_level_feasible &&
        relation.infeasible_reason == wire::core::SameLevelFeasibilityReason::kBundleRule) {
      has_bundle_rule_corner = true;
      break;
    }
  }
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        assignment.continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
        assignment.default_lower_required &&
        ((assignment.decision_a.owner_pole_id == center_id &&
          assignment.decision_a.relation_kind == wire::core::JunctionRelationKind::kCornerContinuation &&
          assignment.decision_a.lower_required && assignment.decision_a.support_group_id >= 0) ||
         (assignment.decision_b.owner_pole_id == center_id &&
          assignment.decision_b.relation_kind == wire::core::JunctionRelationKind::kCornerContinuation &&
          assignment.decision_b.lower_required && assignment.decision_b.support_group_id >= 0)) &&
        !assignment.same_level_feasible) {
      ++lowered_corner_assignments;
    }
  }
  return !junction->through_pair_accepted && has_bundle_rule_corner && lowered_corner_assignments == 2;
}

bool test_backbone_hv3_cross_only_through_pair_stays_same_level_candidate() {
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
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(cross).ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value() || !junction->through_pair_accepted) {
    return false;
  }

  int through_candidates = 0;
  int lowered_nonthrough = 0;
  for (const auto& relation : junction->local_relations) {
    if (relation.continuity_class != wire::core::ContinuityCategoryClass::kBundleLike) {
      continue;
    }
    if (relation.in_through_pair && relation.kind == wire::core::JunctionRelationKind::kThroughMain &&
        !relation.default_lower_required && relation.same_level_feasible) {
      ++through_candidates;
    } else if (!relation.in_through_pair &&
               relation.kind == wire::core::JunctionRelationKind::kCrossUnderpass &&
        relation.default_lower_required && !relation.same_level_feasible &&
               relation.infeasible_reason == wire::core::SameLevelFeasibilityReason::kBundleRule) {
      ++lowered_nonthrough;
    }
  }
  return through_candidates == 2 && lowered_nonthrough == 2;
}

bool test_backbone_point_like_branch_can_keep_same_level_when_clear() {
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
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(branch).ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    return false;
  }

  bool has_point_like_same_level_branch = false;
  bool has_unlowered_assignment = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kSideBranch &&
        relation.continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
        !relation.default_lower_required && relation.same_level_feasible) {
      has_point_like_same_level_branch = true;
      break;
    }
  }
  for (const auto& assignment : state.view().last_lane_assignments()) {
    if ((assignment.pole_a_id == center_id || assignment.pole_b_id == center_id) &&
        assignment.continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
        !assignment.default_lower_required && assignment.same_level_feasible &&
        !assignment.decision_a.lower_required && !assignment.decision_b.lower_required &&
        assignment.decision_a.support_group_id < 0 && assignment.decision_b.support_group_id < 0) {
      has_unlowered_assignment = true;
      break;
    }
  }
  return has_point_like_same_level_branch && has_unlowered_assignment;
}

bool test_backbone_bundle_rule_policy_block_stays_unresolved() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    return false;
  }

  const auto junction = state.view().inspect_junction(center_id);
  if (!junction.has_value()) {
    return false;
  }

  bool has_bundle_rule_relation = false;
  for (const auto& relation : junction->local_relations) {
    if (relation.in_route && relation.kind == wire::core::JunctionRelationKind::kSideBranch &&
        relation.continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
        relation.default_lower_required && !relation.same_level_feasible &&
        relation.infeasible_reason == wire::core::SameLevelFeasibilityReason::kBundleRule) {
      has_bundle_rule_relation = true;
      break;
    }
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && (span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id)) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto span_view = state.view().inspect_span(target_span_id);
  if (!span_view.has_value()) {
    return false;
  }
      return has_bundle_rule_relation &&
        span_view->continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
        span_view->default_lower_required &&
        !span_view->same_level_feasible &&
        span_view->same_level_reason == wire::core::SameLevelFeasibilityReason::kCategoryPolicyDisabled &&
        span_view->unresolved_same_level_conflict;
}

bool test_backbone_refresh_keeps_bundle_rule_origin() {
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
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok) {
    return false;
  }
  if (!state.SetPoleManualYawOverride(center_id, 15.0).ok) {
    return false;
  }

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span != nullptr && (span->endpoint_node_a_id == center_id || span->endpoint_node_b_id == center_id)) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const auto span_view = state.view().inspect_span(target_span_id);
  const auto layout_view = state.view().inspect_support_layout(target_span_id);
  if (!span_view.has_value() || !layout_view.has_value()) {
    return false;
  }

  const bool ok = span_view->continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
         span_view->default_lower_required &&
         !span_view->same_level_feasible &&
         span_view->same_level_reason == wire::core::SameLevelFeasibilityReason::kBundleRule &&
         layout_view->continuity_class == wire::core::ContinuityCategoryClass::kBundleLike &&
         layout_view->default_lower_required &&
         !layout_view->same_level_feasible &&
         layout_view->same_level_reason == wire::core::SameLevelFeasibilityReason::kBundleRule;
  return ok;
}

bool test_backbone_cross_lowered_pair_uses_opposite_junction_pair_sides() {
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
  const auto generated = state.GenerateFromBackboneSpec(vertical);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  bool saw_non_center = false;
  bool saw_pair_side = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value() ||
        layout_view->lowering_kind != wire::core::BackboneLoweringKind::kCrossUnderpass) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value()) {
      continue;
    }
    if (endpoint->used_junction_pair_side_assignment) {
      saw_pair_side = true;
    }
    if (std::abs(endpoint->chosen_side_sign) > 0.5) {
      saw_non_center = true;
    }
  }
  if (!(saw_pair_side && saw_non_center)) {
    std::cerr << "[DBG] C234 pairSide=" << saw_pair_side << " nonCenter=" << saw_non_center << "\n";
  }
  return saw_pair_side && saw_non_center;
}

bool test_backbone_constrained_lowered_support_prefers_line_direction() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const wire::core::Vec3d pair_normal = normalize_xy_safe({0.0, 1.0, 0.0});
  bool saw_constrained_visual = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->origin != "PlacementConstraint") {
      continue;
    }
    if (endpoint->support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial) {
      std::cerr << "[DBG] C235 radial endpoint remained constrained\n";
      continue;
    }
    const auto placement = lowered_support_group_for_owner(*layout_view, center_id);
    if (!placement.has_value() || placement->origin != "PlacementConstraint") {
      continue;
    }
    const wire::core::Vec3d support_axis = normalize_xy_safe(placement->tip_world - placement->mount_world);
    if (std::abs(dot_xy(support_axis, pair_normal)) >= 0.97 &&
        placement->decision.support_orientation_basis != wire::core::SupportOrientationBasisKind::kRadial) {
      saw_constrained_visual = true;
    } else {
      std::cerr << "[DBG] C235 bad axis align=" << std::abs(dot_xy(support_axis, pair_normal)) << " mount=("
                << placement->mount_world.x << "," << placement->mount_world.y << "," << placement->mount_world.z
                << ") tip=(" << placement->tip_world.x << "," << placement->tip_world.y << ","
                << placement->tip_world.z << ") sideSign=" << placement->chosen_side_sign << " origin="
                << placement->origin << " sideRule="
                << static_cast<int>(placement->side_assignment_rule) << " orientRule="
                << static_cast<int>(placement->support_orientation_rule) << "\n";
    }
  }
  return saw_constrained_visual;
}

bool test_backbone_bundle_branch_support_orientation_uses_bisector_when_available() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 14.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const wire::core::Vec3d branch_dir = normalize_xy_safe({10.0, 14.0, 0.0});
  bool found = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() ||
        endpoint->support_orientation_rule != wire::core::SupportOrientationRuleKind::kBisector) {
      continue;
    }
    const auto placement = lowered_support_group_for_owner(*layout_view, center_id);
    if (!placement.has_value()) {
      continue;
    }
    const wire::core::Vec3d trunk_dir = normalize_xy_safe({12.0, 0.0, 0.0});
    const wire::core::Vec3d expected_axis = normalize_xy_safe(branch_dir + trunk_dir);
    const wire::core::Vec3d support_axis = normalize_xy_safe(placement->tip_world - placement->mount_world);
    if (std::abs(dot_xy(support_axis, expected_axis)) >= 0.97 &&
        placement->decision.support_orientation_basis != wire::core::SupportOrientationBasisKind::kRadial) {
      found = true;
    }
  }
  if (!found) {
    for (ObjectId span_id : generated.value.generated_span_ids) {
      const auto layout_view = state.view().inspect_support_layout(span_id);
      if (!layout_view.has_value()) {
        continue;
      }
      const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
      if (!endpoint.has_value()) {
        continue;
      }
      if (endpoint->decision.lower_required &&
          endpoint->decision.relation_kind == wire::core::JunctionRelationKind::kSideBranch &&
          endpoint->support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector &&
          (endpoint->decision.support_orientation_basis ==
               wire::core::SupportOrientationBasisKind::kBisectorForward ||
           endpoint->decision.support_orientation_basis ==
               wire::core::SupportOrientationBasisKind::kBisectorReverse)) {
        return true;
      }
    }
  }
  return found;
}

bool test_backbone_bundle_branch_lowering_stays_pole_local() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}, {12.0, 12.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.size() < 2) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  int grouped_segments = 0;
  int locally_lowered_segments = 0;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    if (!layout_view->lowered_support_groups.empty()) {
      ++grouped_segments;
    }
    const bool local_lower =
        layout_view->start_endpoint.decision.lower_required || layout_view->end_endpoint.decision.lower_required;
    if (local_lower) {
      ++locally_lowered_segments;
    }
  }
  if (!(grouped_segments >= 2 && grouped_segments == locally_lowered_segments)) {
    std::cerr << "[DBG] C250 grouped=" << grouped_segments << " local=" << locally_lowered_segments << "\n";
  }
  return grouped_segments >= 2 && grouped_segments == locally_lowered_segments;
}

bool test_backbone_cross_underpass_supports_share_one_side_group() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  std::vector<wire::core::LoweredSupportGroupInspectionView> placements{};
  std::vector<wire::core::Vec3d> support_worlds{};
  std::vector<wire::core::EndpointContinuityDecision> lowered_endpoint_decisions{};
  for (const auto& span_entry : state.view().spans().items()) {
    const ObjectId span_id = span_entry.id;
    const auto support_layout = state.view().inspect_support_layout(span_id);
    if (!support_layout.has_value()) {
      continue;
    }
    for (const auto& placement : support_layout->lowered_support_groups) {
      if (placement.owner_pole_id == center_id &&
          placement.decision.relation_kind == wire::core::JunctionRelationKind::kCrossUnderpass) {
        placements.push_back(placement);
      }
    }
    const auto collect_endpoint = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
      if (endpoint.owner_pole_id == center_id &&
          endpoint.decision.relation_kind == wire::core::JunctionRelationKind::kCrossUnderpass &&
          endpoint.decision.lower_required) {
        support_worlds.push_back(endpoint.support_world);
        lowered_endpoint_decisions.push_back(endpoint.decision);
      }
    };
    collect_endpoint(support_layout->start_endpoint);
    collect_endpoint(support_layout->end_endpoint);
    }
  if (placements.empty()) {
    std::cerr << "[DBG] C251 missing grouped lowered cross support\n";
    return false;
  }
  std::unordered_set<std::uint64_t> unique_supports{};
  for (const auto& placement : placements) {
    const std::uint64_t support_key =
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(placement.owner_pole_id)) << 32) ^
        static_cast<std::uint32_t>(placement.support_group_id);
    unique_supports.insert(support_key);
  }
  if (unique_supports.size() != 1) {
    std::cerr << "[DBG] C251 expected one unique grouped lowered cross support count=" << unique_supports.size()
              << " placements=" << placements.size() << "\n";
    return false;
  }
  const int group_id = placements.front().support_group_id;
  const double side_sign = placements.front().chosen_side_sign;
  const auto side_rule = placements.front().side_assignment_rule;
  const auto orientation_rule = placements.front().support_orientation_rule;
  const auto group_basis = placements.front().decision.support_orientation_basis;
  const auto group_side = placements.front().decision.chosen_side;
  for (const auto& placement : placements) {
    if (placement.grouping_rule != wire::core::SupportGroupingRuleKind::kDecisionGroup ||
        placement.support_group_id != group_id ||
        placement.side_assignment_rule != side_rule ||
        placement.support_orientation_rule != orientation_rule ||
        placement.side_assignment_rule == wire::core::SideAssignmentRuleKind::kPoleLocal ||
        placement.support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial ||
        std::abs(placement.chosen_side_sign - side_sign) > 1e-9 ||
        !almost_equal(placement.side_axis.x, placements.front().side_axis.x, 1e-6) ||
        !almost_equal(placement.side_axis.y, placements.front().side_axis.y, 1e-6) ||
        !almost_equal(placement.mount_world.x, placements.front().mount_world.x, 1e-6) ||
        !almost_equal(placement.mount_world.y, placements.front().mount_world.y, 1e-6) ||
        !almost_equal(placement.tip_world.x, placements.front().tip_world.x, 1e-6) ||
        !almost_equal(placement.tip_world.y, placements.front().tip_world.y, 1e-6)) {
        std::cerr << "[DBG] C251 split support group=" << placement.support_group_id << " ref=" << group_id
                  << " side=" << placement.chosen_side_sign << " refSide=" << side_sign << " mount=("
                  << placement.mount_world.x << "," << placement.mount_world.y << "," << placement.mount_world.z
                  << ") refMount=(" << placements.front().mount_world.x << "," << placements.front().mount_world.y
                  << "," << placements.front().mount_world.z << ")\n";
        return false;
      }
  }
  for (const auto& decision : lowered_endpoint_decisions) {
    if (decision.side_assignment_rule != side_rule ||
        decision.support_orientation_rule != orientation_rule ||
        decision.support_orientation_basis != group_basis ||
        decision.chosen_side != group_side ||
        std::abs(decision.chosen_side_sign - side_sign) > 1e-9) {
      std::cerr << "[DBG] C251 endpoint decision mismatch basis="
                << static_cast<int>(decision.support_orientation_basis) << " groupBasis="
                << static_cast<int>(group_basis) << " side=" << static_cast<int>(decision.chosen_side)
                << " groupSide=" << static_cast<int>(group_side) << "\n";
      return false;
    }
  }
  if (support_worlds.size() >= 2) {
    const wire::core::Vec3d ref = support_worlds.front();
    for (const wire::core::Vec3d& support_world : support_worlds) {
      if (!almost_equal(support_world.x, ref.x, 1e-6) || !almost_equal(support_world.y, ref.y, 1e-6) ||
          !almost_equal(support_world.z, ref.z, 1e-6)) {
        std::cerr << "[DBG] C251 support_world mismatch (" << support_world.x << "," << support_world.y
                  << "," << support_world.z << ") ref=(" << ref.x << "," << ref.y << "," << ref.z << ")\n";
        return false;
      }
    }
  }
  for (const auto& decision : lowered_endpoint_decisions) {
    if (!decision.lower_required || decision.lowering_blocked_by_policy || decision.support_group_id != group_id) {
      std::cerr << "[DBG] C251 lower decision mismatch group=" << decision.support_group_id
                << " expected=" << group_id << " lowerRequired=" << decision.lower_required
                << " blocked=" << decision.lowering_blocked_by_policy << "\n";
      return false;
    }
  }
  return true;
}

bool test_backbone_refresh_keeps_local_lower_and_grouped_support() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  auto grouped_support_count_for_center = [&](ObjectId span_id) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      return 0;
    }
    int grouped_count = 0;
    for (const auto& placement : layout_view->lowered_support_groups) {
      if (placement.owner_pole_id == center_id &&
          placement.grouping_rule == wire::core::SupportGroupingRuleKind::kDecisionGroup) {
        ++grouped_count;
      }
    }
    return grouped_count;
  };

  struct EndpointSnapshot {
    int support_group_id = -1;
    wire::core::Vec3d support_world{};
    wire::core::SideAssignmentRuleKind side_assignment_rule = wire::core::SideAssignmentRuleKind::kPoleLocal;
    wire::core::SupportOrientationRuleKind support_orientation_rule = wire::core::SupportOrientationRuleKind::kRadial;
    wire::core::SupportOrientationBasisKind support_orientation_basis = wire::core::SupportOrientationBasisKind::kRadial;
    wire::core::Vec3d side_axis{};
    double chosen_side_sign = 0.0;
    bool lower_required = false;
    bool default_lower_required = false;
    wire::core::JunctionRelationKind relation_kind = wire::core::JunctionRelationKind::kNone;
  };
  struct GroupSnapshot {
    int support_group_id = -1;
    wire::core::Vec3d mount_world{};
    wire::core::Vec3d tip_world{};
    wire::core::SideAssignmentRuleKind side_assignment_rule = wire::core::SideAssignmentRuleKind::kPoleLocal;
    wire::core::SupportOrientationRuleKind support_orientation_rule = wire::core::SupportOrientationRuleKind::kRadial;
    wire::core::SupportOrientationBasisKind support_orientation_basis = wire::core::SupportOrientationBasisKind::kRadial;
    wire::core::Vec3d side_axis{};
    double chosen_side_sign = 0.0;
    bool lower_required = false;
    bool default_lower_required = false;
    wire::core::JunctionRelationKind relation_kind = wire::core::JunctionRelationKind::kNone;
  };
  auto snapshot_endpoint = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
    EndpointSnapshot s{};
    s.support_group_id = endpoint.decision.support_group_id;
    s.support_world = endpoint.support_world;
    s.side_assignment_rule = endpoint.side_assignment_rule;
    s.support_orientation_rule = endpoint.support_orientation_rule;
    s.support_orientation_basis = endpoint.decision.support_orientation_basis;
    s.side_axis = endpoint.side_axis;
    s.chosen_side_sign = endpoint.chosen_side_sign;
    s.lower_required = endpoint.decision.lower_required;
    s.default_lower_required = endpoint.decision.default_lower_required;
    s.relation_kind = endpoint.relation_kind;
    return s;
  };
  auto snapshot_group_for_center = [&](const wire::core::SupportLayoutInspectionView& layout) {
    GroupSnapshot s{};
    for (const auto& g : layout.lowered_support_groups) {
      if (g.owner_pole_id != center_id) {
        continue;
      }
      s.support_group_id = g.support_group_id;
      s.mount_world = g.mount_world;
      s.tip_world = g.tip_world;
      s.side_assignment_rule = g.side_assignment_rule;
      s.support_orientation_rule = g.support_orientation_rule;
      s.support_orientation_basis = g.decision.support_orientation_basis;
      s.side_axis = g.side_axis;
      s.chosen_side_sign = g.chosen_side_sign;
      s.lower_required = g.decision.lower_required;
      s.default_lower_required = g.decision.default_lower_required;
      s.relation_kind = g.decision.relation_kind;
      break;
    }
    return s;
  };

  ObjectId target_span_id = wire::core::kInvalidObjectId;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    if (grouped_support_count_for_center(span_id) > 0) {
      target_span_id = span_id;
      break;
    }
  }
  if (target_span_id == wire::core::kInvalidObjectId) {
    return false;
  }
    const auto before = state.view().inspect_support_layout(target_span_id);
    const int before_grouped = grouped_support_count_for_center(target_span_id);
    if (!before.has_value() || before_grouped == 0) {
      return false;
    }
    const auto before_endpoint = before->start_endpoint.owner_pole_id == center_id
                                     ? snapshot_endpoint(before->start_endpoint)
                                     : snapshot_endpoint(before->end_endpoint);
    const auto before_group = snapshot_group_for_center(*before);
    if (!state.SetPoleManualYawOverride(center_id, 19.0).ok) {
      return false;
    }
    const auto after = state.view().inspect_support_layout(target_span_id);
    const int after_grouped = grouped_support_count_for_center(target_span_id);
    if (!after.has_value()) {
      return false;
    }
    const auto after_endpoint = after->start_endpoint.owner_pole_id == center_id
                                    ? snapshot_endpoint(after->start_endpoint)
                                    : snapshot_endpoint(after->end_endpoint);
    const auto after_group = snapshot_group_for_center(*after);
    return after.has_value() &&
         before->start_endpoint.decision.lower_required == after->start_endpoint.decision.lower_required &&
         before->end_endpoint.decision.lower_required == after->end_endpoint.decision.lower_required &&
         before_grouped == after_grouped && after_grouped > 0 &&
         before_endpoint.support_group_id == after_endpoint.support_group_id &&
         before_group.support_group_id == after_group.support_group_id &&
         almost_equal(before_endpoint.support_world.x, after_endpoint.support_world.x, 1e-6) &&
         almost_equal(before_endpoint.support_world.y, after_endpoint.support_world.y, 1e-6) &&
         almost_equal(before_endpoint.support_world.z, after_endpoint.support_world.z, 1e-6) &&
         almost_equal(before_group.mount_world.x, after_group.mount_world.x, 1e-6) &&
         almost_equal(before_group.mount_world.y, after_group.mount_world.y, 1e-6) &&
         almost_equal(before_group.mount_world.z, after_group.mount_world.z, 1e-6) &&
         almost_equal(before_group.tip_world.x, after_group.tip_world.x, 1e-6) &&
         almost_equal(before_group.tip_world.y, after_group.tip_world.y, 1e-6) &&
         almost_equal(before_group.tip_world.z, after_group.tip_world.z, 1e-6) &&
         before_endpoint.side_assignment_rule == after_endpoint.side_assignment_rule &&
         before_endpoint.support_orientation_rule == after_endpoint.support_orientation_rule &&
         before_endpoint.support_orientation_basis == after_endpoint.support_orientation_basis &&
         almost_equal(before_endpoint.side_axis.x, after_endpoint.side_axis.x, 1e-6) &&
         almost_equal(before_endpoint.side_axis.y, after_endpoint.side_axis.y, 1e-6) &&
         almost_equal(before_endpoint.chosen_side_sign, after_endpoint.chosen_side_sign, 1e-9) &&
         before_group.side_assignment_rule == after_group.side_assignment_rule &&
         before_group.support_orientation_rule == after_group.support_orientation_rule &&
         before_group.support_orientation_basis == after_group.support_orientation_basis &&
         almost_equal(before_group.side_axis.x, after_group.side_axis.x, 1e-6) &&
         almost_equal(before_group.side_axis.y, after_group.side_axis.y, 1e-6) &&
         almost_equal(before_group.chosen_side_sign, after_group.chosen_side_sign, 1e-9) &&
         before_endpoint.lower_required == after_endpoint.lower_required &&
         before_endpoint.default_lower_required == after_endpoint.default_lower_required &&
         before_endpoint.relation_kind == after_endpoint.relation_kind;
  }

bool test_backbone_grouped_support_visual_cache_uses_single_group_placement() {
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
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    const wire::core::SpanVisualCacheEntry* visual = state.view().find_span_visual_cache(span_id);
    if (!layout_view.has_value() || visual == nullptr) {
      continue;
    }
    const auto group = lowered_support_group_for_owner(*layout_view, center_id);
    if (!group.has_value()) {
      continue;
    }

    int matching_arms = 0;
    int matching_hangers = 0;
    for (const auto& part : visual->parts) {
      if (part.kind == wire::core::VisualPartKind::kSupportArm &&
          almost_equal(part.a.x, group->mount_world.x, 1e-6) &&
          almost_equal(part.a.y, group->mount_world.y, 1e-6) &&
          almost_equal(part.a.z, group->mount_world.z, 1e-6) &&
          almost_equal(part.b.x, group->tip_world.x, 1e-6) &&
          almost_equal(part.b.y, group->tip_world.y, 1e-6) &&
          almost_equal(part.b.z, group->tip_world.z, 1e-6)) {
        ++matching_arms;
      }
      if (part.kind == wire::core::VisualPartKind::kFitting &&
          almost_equal(part.a.x, group->tip_world.x, 1e-6) &&
          almost_equal(part.a.y, group->tip_world.y, 1e-6) &&
          almost_equal(part.a.z, group->tip_world.z, 1e-6)) {
        const bool matches_attachment = std::any_of(
            group->attachment_worlds.begin(), group->attachment_worlds.end(), [&](const wire::core::Vec3d& attachment) {
              return almost_equal(part.b.x, attachment.x, 1e-6) && almost_equal(part.b.y, attachment.y, 1e-6) &&
                     almost_equal(part.b.z, attachment.z, 1e-6);
            });
        if (matches_attachment) {
          ++matching_hangers;
        }
      }
    }
    if (matching_arms != 1 || matching_hangers != group->grouped_port_count) {
      std::cerr << "[DBG] C271 span=" << span_id << " arms=" << matching_arms
                << " hangers=" << matching_hangers << " groupedPortCount=" << group->grouped_port_count << "\n";
      return false;
    }
    return true;
  }

  return false;
}

bool test_backbone_branch_lower_required_height_survives_to_grouped_placement() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  struct Snapshot {
    ObjectId span_id = wire::core::kInvalidObjectId;
    int support_group_id = -1;
    bool lower_required = false;
    bool default_lower_required = false;
    wire::core::JunctionRelationKind relation_kind = wire::core::JunctionRelationKind::kNone;
    wire::core::Vec3d support_world{};
    wire::core::Vec3d mount_world{};
    wire::core::Vec3d tip_world{};
  };
  auto collect = [&]() {
    std::vector<Snapshot> snapshots{};
    for (const auto& span_entry : state.view().spans().items()) {
      const ObjectId span_id = span_entry.id;
      const auto layout = state.view().inspect_support_layout(span_id);
      if (!layout.has_value()) {
        continue;
      }
      const auto collect_endpoint = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
        if (endpoint.owner_pole_id != center_id ||
            endpoint.decision.relation_kind != wire::core::JunctionRelationKind::kSideBranch ||
            !endpoint.decision.lower_required || endpoint.decision.lowering_blocked_by_policy ||
            endpoint.decision.support_group_id < 0) {
          return;
        }
        for (const auto& group : layout->lowered_support_groups) {
          if (group.owner_pole_id == center_id && group.support_group_id == endpoint.decision.support_group_id) {
            Snapshot s{};
            s.span_id = span_id;
            s.support_group_id = endpoint.decision.support_group_id;
            s.lower_required = endpoint.decision.lower_required;
            s.default_lower_required = endpoint.decision.default_lower_required;
            s.relation_kind = endpoint.decision.relation_kind;
            s.support_world = endpoint.support_world;
            s.mount_world = group.mount_world;
            s.tip_world = group.tip_world;
            snapshots.push_back(s);
            return;
          }
        }
      };
      collect_endpoint(layout->start_endpoint);
      collect_endpoint(layout->end_endpoint);
    }
    std::sort(snapshots.begin(), snapshots.end(), [](const Snapshot& a, const Snapshot& b) {
      if (a.span_id != b.span_id) {
        return a.span_id < b.span_id;
      }
      return a.support_group_id < b.support_group_id;
    });
    return snapshots;
  };

  const auto before = collect();
  if (before.empty()) {
    std::cerr << "[DBG] C272 no lowered branch endpoint/group snapshots\n";
    return false;
  }
  for (const auto& s : before) {
    if (!almost_equal(s.support_world.z, s.tip_world.z, 1e-6) ||
        !almost_equal(s.mount_world.z, s.tip_world.z, 1e-6)) {
      std::cerr << "[DBG] C272 height mismatch span=" << s.span_id << " group=" << s.support_group_id
                << " supportZ=" << s.support_world.z << " mountZ=" << s.mount_world.z
                << " tipZ=" << s.tip_world.z << "\n";
      return false;
    }
  }

  if (!state.SetPoleManualYawOverride(center_id, 23.0).ok) {
    return false;
  }
  const auto after = collect();
  if (before.size() != after.size()) {
    std::cerr << "[DBG] C272 snapshot size changed before=" << before.size() << " after=" << after.size() << "\n";
    return false;
  }
  for (std::size_t i = 0; i < before.size(); ++i) {
    const Snapshot& b = before[i];
    const Snapshot& a = after[i];
    if (b.span_id != a.span_id || b.support_group_id != a.support_group_id || b.lower_required != a.lower_required ||
        b.default_lower_required != a.default_lower_required || b.relation_kind != a.relation_kind ||
        !almost_equal(b.support_world.z, a.support_world.z, 1e-6) ||
        !almost_equal(b.mount_world.z, a.mount_world.z, 1e-6) ||
        !almost_equal(b.tip_world.z, a.tip_world.z, 1e-6)) {
      std::cerr << "[DBG] C272 refresh mismatch i=" << i << " spanBefore=" << b.span_id
                << " spanAfter=" << a.span_id << " groupBefore=" << b.support_group_id
                << " groupAfter=" << a.support_group_id << " supportZBefore=" << b.support_world.z
                << " supportZAfter=" << a.support_world.z << " mountZBefore=" << b.mount_world.z
                << " mountZAfter=" << a.mount_world.z << " tipZBefore=" << b.tip_world.z
                << " tipZAfter=" << a.tip_world.z << "\n";
      return false;
    }
  }
  return true;
}

bool test_backbone_bundle_non_through_height_collapses_to_two_states() {
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
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {{0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}, {8.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 1000.0;
  branch.pole_type_id = type_ids.front();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  if (!state.GenerateFromBackboneSpec(branch).ok) {
    return false;
  }

  const ObjectId corner_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  if (corner_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  struct Snapshot {
    double through_main_z = std::numeric_limits<double>::quiet_NaN();
    double through_main_offset_m = 0.0;
    double side_branch_z = std::numeric_limits<double>::quiet_NaN();
    double side_branch_offset_m = 0.0;
    double side_branch_mount_z = std::numeric_limits<double>::quiet_NaN();
    double side_branch_tip_z = std::numeric_limits<double>::quiet_NaN();
    double corner_z = std::numeric_limits<double>::quiet_NaN();
    double corner_offset_m = 0.0;
    double corner_mount_z = std::numeric_limits<double>::quiet_NaN();
    double corner_tip_z = std::numeric_limits<double>::quiet_NaN();
  };
  auto collect = [&]() {
    Snapshot snapshot{};
    for (const auto& span : state.view().spans().items()) {
      const auto layout = state.view().inspect_support_layout(span.id);
      if (!layout.has_value()) {
        continue;
      }
      const auto absorb = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
        if (endpoint.owner_pole_id == center_id &&
            endpoint.decision.relation_kind == wire::core::JunctionRelationKind::kThroughMain &&
            !endpoint.decision.lower_required) {
          if (!std::isfinite(snapshot.through_main_z)) {
            snapshot.through_main_z = endpoint.support_world.z;
            snapshot.through_main_offset_m = endpoint.branch_down_offset_m;
          } else if (!almost_equal(snapshot.through_main_z, endpoint.support_world.z, 1e-6)) {
            snapshot.through_main_z = std::numeric_limits<double>::quiet_NaN();
          }
        }
        if (endpoint.owner_pole_id == center_id &&
            endpoint.decision.relation_kind == wire::core::JunctionRelationKind::kSideBranch &&
            endpoint.decision.lower_required && !endpoint.decision.lowering_blocked_by_policy) {
          if (!std::isfinite(snapshot.side_branch_z)) {
            snapshot.side_branch_z = endpoint.support_world.z;
            snapshot.side_branch_offset_m = endpoint.branch_down_offset_m;
          } else if (!almost_equal(snapshot.side_branch_z, endpoint.support_world.z, 1e-6) ||
                     !almost_equal(snapshot.side_branch_offset_m, endpoint.branch_down_offset_m, 1e-6)) {
            snapshot.side_branch_z = std::numeric_limits<double>::quiet_NaN();
          }
        }
        if (endpoint.owner_pole_id == corner_id &&
            endpoint.decision.relation_kind == wire::core::JunctionRelationKind::kCornerContinuation &&
            endpoint.decision.lower_required && !endpoint.decision.lowering_blocked_by_policy) {
          if (!std::isfinite(snapshot.corner_z)) {
            snapshot.corner_z = endpoint.support_world.z;
            snapshot.corner_offset_m = endpoint.branch_down_offset_m;
          } else if (!almost_equal(snapshot.corner_z, endpoint.support_world.z, 1e-6) ||
                     !almost_equal(snapshot.corner_offset_m, endpoint.branch_down_offset_m, 1e-6)) {
            snapshot.corner_z = std::numeric_limits<double>::quiet_NaN();
          }
        }
      };
      absorb(layout->start_endpoint);
      absorb(layout->end_endpoint);
      if (const auto branch_group = lowered_support_group_for_owner(*layout, center_id); branch_group.has_value()) {
        if (!std::isfinite(snapshot.side_branch_mount_z)) {
          snapshot.side_branch_mount_z = branch_group->mount_world.z;
          snapshot.side_branch_tip_z = branch_group->tip_world.z;
        } else if (!almost_equal(snapshot.side_branch_mount_z, branch_group->mount_world.z, 1e-6) ||
                   !almost_equal(snapshot.side_branch_tip_z, branch_group->tip_world.z, 1e-6)) {
          snapshot.side_branch_mount_z = std::numeric_limits<double>::quiet_NaN();
          snapshot.side_branch_tip_z = std::numeric_limits<double>::quiet_NaN();
        }
      }
      if (const auto corner_group = lowered_support_group_for_owner(*layout, corner_id); corner_group.has_value()) {
        if (!std::isfinite(snapshot.corner_mount_z)) {
          snapshot.corner_mount_z = corner_group->mount_world.z;
          snapshot.corner_tip_z = corner_group->tip_world.z;
        } else if (!almost_equal(snapshot.corner_mount_z, corner_group->mount_world.z, 1e-6) ||
                   !almost_equal(snapshot.corner_tip_z, corner_group->tip_world.z, 1e-6)) {
          snapshot.corner_mount_z = std::numeric_limits<double>::quiet_NaN();
          snapshot.corner_tip_z = std::numeric_limits<double>::quiet_NaN();
        }
      }
    }
    return snapshot;
  };

  const Snapshot before = collect();
  if (!std::isfinite(before.through_main_z) || !std::isfinite(before.side_branch_z) || !std::isfinite(before.corner_z) ||
      !std::isfinite(before.side_branch_mount_z) || !std::isfinite(before.side_branch_tip_z) ||
      !std::isfinite(before.corner_mount_z) || !std::isfinite(before.corner_tip_z) ||
      before.side_branch_offset_m <= 1e-9 || before.corner_offset_m <= 1e-9) {
    std::cerr << "[DBG] C273 missing through/branch/corner snapshot main=" << before.through_main_z
              << " branch=" << before.side_branch_z << " corner=" << before.corner_z
              << " branchMount=" << before.side_branch_mount_z << " cornerMount=" << before.corner_mount_z
              << " branchOffset=" << before.side_branch_offset_m << " cornerOffset=" << before.corner_offset_m
              << "\n";
    return false;
  }
  if (std::abs(before.through_main_offset_m) > 1e-9 || !(before.through_main_z > before.side_branch_z)) {
    std::cerr << "[DBG] C273 main state mismatch mainZ=" << before.through_main_z
              << " branchZ=" << before.side_branch_z << " mainOffset=" << before.through_main_offset_m << "\n";
    return false;
  }
  if (!almost_equal(before.side_branch_z, before.corner_z, 1e-6) ||
      !almost_equal(before.side_branch_mount_z, before.corner_mount_z, 1e-6) ||
      !almost_equal(before.side_branch_tip_z, before.corner_tip_z, 1e-6) ||
      !almost_equal(before.side_branch_offset_m, before.corner_offset_m, 1e-6)) {
    std::cerr << "[DBG] C273 branch/corner mismatch branchZ=" << before.side_branch_z
              << " cornerZ=" << before.corner_z << " branchOffset=" << before.side_branch_offset_m
              << " cornerOffset=" << before.corner_offset_m << " branchMount=" << before.side_branch_mount_z
              << " cornerMount=" << before.corner_mount_z << " branchTip=" << before.side_branch_tip_z
              << " cornerTip=" << before.corner_tip_z << "\n";
    return false;
  }

  if (!state.SetPoleManualYawOverride(center_id, 19.0).ok) {
    return false;
  }
  const Snapshot after = collect();
  if (!almost_equal(before.through_main_z, after.through_main_z, 1e-6) ||
      !almost_equal(before.side_branch_z, after.side_branch_z, 1e-6) ||
      !almost_equal(before.corner_z, after.corner_z, 1e-6) ||
      !almost_equal(before.side_branch_mount_z, after.side_branch_mount_z, 1e-6) ||
      !almost_equal(before.side_branch_tip_z, after.side_branch_tip_z, 1e-6) ||
      !almost_equal(before.corner_mount_z, after.corner_mount_z, 1e-6) ||
      !almost_equal(before.corner_tip_z, after.corner_tip_z, 1e-6) ||
      !almost_equal(before.through_main_offset_m, after.through_main_offset_m, 1e-6) ||
      !almost_equal(before.side_branch_offset_m, after.side_branch_offset_m, 1e-6) ||
      !almost_equal(before.corner_offset_m, after.corner_offset_m, 1e-6)) {
    std::cerr << "[DBG] C273 refresh mismatch mainBefore=" << before.through_main_z
              << " mainAfter=" << after.through_main_z << " branchBefore=" << before.side_branch_z
              << " branchAfter=" << after.side_branch_z << " cornerBefore=" << before.corner_z
              << " cornerAfter=" << after.corner_z << " branchMountBefore=" << before.side_branch_mount_z
              << " branchMountAfter=" << after.side_branch_mount_z << " cornerMountBefore=" << before.corner_mount_z
              << " cornerMountAfter=" << after.corner_mount_z << "\n";
    return false;
  }
  return true;
}

bool test_backbone_crosslike_reuse_keeps_existing_straight_main_pair() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec trunk{};
  trunk.path.polyline = {{8.06891, 4.30726, 0.0}, {14.9458, -0.818155, 0.0}, {20.266, -1.66916, 0.0}};
  trunk.interval_m = 1000.0;
  trunk.pole_type_id = type_ids.front();
  add_backbone_bundle(trunk, wire::core::BundleKind::kHighVoltage);
  const auto trunk_generated = state.GenerateFromBackboneSpec(trunk);
  if (!trunk_generated.ok) {
    std::cerr << "[DBG] C274 trunk generate failed error=" << trunk_generated.error << "\n";
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {14.9458, -0.818155, 0.0}, 1e-4);
  const ObjectId existing_a_id = find_pole_id_by_position(state, {8.06891, 4.30726, 0.0}, 1e-4);
  const ObjectId existing_b_id = find_pole_id_by_position(state, {20.266, -1.66916, 0.0}, 1e-4);
  if (center_id == wire::core::kInvalidObjectId || existing_a_id == wire::core::kInvalidObjectId ||
      existing_b_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C274 existing node lookup failed center=" << center_id << " a=" << existing_a_id
              << " b=" << existing_b_id << "\n";
    return false;
  }

  wire::core::BackboneSpec crossing{};
  crossing.path.polyline = {{15.6658, 4.29683, 0.0}, {14.9458, -0.818155, 0.0}, {14.2857, -6.95258, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  crossing.path.node_specs.push_back(shared);
  crossing.interval_m = 1000.0;
  crossing.pole_type_id = type_ids.front();
  add_backbone_bundle(crossing, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(crossing);
  if (!generated.ok) {
    std::cerr << "[DBG] C274 crossing generate failed error=" << generated.error << "\n";
    return false;
  }

  const ObjectId new_a_id = find_pole_id_by_position(state, {15.6658, 4.29683, 0.0}, 1e-4);
  const ObjectId new_b_id = find_pole_id_by_position(state, {14.2857, -6.95258, 0.0}, 1e-4);
  const auto junction = state.view().inspect_junction(center_id);
  if (new_a_id == wire::core::kInvalidObjectId || new_b_id == wire::core::kInvalidObjectId || !junction.has_value()) {
    std::cerr << "[DBG] C274 new node or junction lookup failed newA=" << new_a_id << " newB=" << new_b_id
              << " hasJunction=" << (junction.has_value() ? 1 : 0) << "\n";
    return false;
  }

  auto is_existing_pair = [&](ObjectId a, ObjectId b) {
    return (a == existing_a_id && b == existing_b_id) || (a == existing_b_id && b == existing_a_id);
  };
  auto relation_kind_for = [&](ObjectId neighbor_id) {
    for (const auto& relation : junction->local_relations) {
      if (relation.neighbor_node_id == neighbor_id) {
        return relation.kind;
      }
    }
    return wire::core::JunctionRelationKind::kNone;
  };

  int generated_hv_count = 0;
  int generated_cross_under_count = 0;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto span_view = state.view().inspect_span(span_id);
    const auto* span = state.view().edit_state().spans.find(span_id);
    const auto* bundle = (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
    if (!span_view.has_value() || bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    ++generated_hv_count;
    if (span_view->lowering_kind == wire::core::BackboneLoweringKind::kCrossUnderpass &&
        !span_view->same_level_feasible) {
      ++generated_cross_under_count;
    }
  }

  const bool ok = junction->through_pair_accepted &&
                  is_existing_pair(junction->through_pair_neighbor_a_id, junction->through_pair_neighbor_b_id) &&
                  relation_kind_for(existing_a_id) == wire::core::JunctionRelationKind::kThroughMain &&
                  relation_kind_for(existing_b_id) == wire::core::JunctionRelationKind::kThroughMain &&
                  relation_kind_for(new_a_id) == wire::core::JunctionRelationKind::kCrossUnderpass &&
                  relation_kind_for(new_b_id) == wire::core::JunctionRelationKind::kCrossUnderpass &&
                  generated_hv_count == 6 && generated_cross_under_count == generated_hv_count;
  if (!ok) {
    std::cerr << "[DBG] C274 accepted=" << (junction->through_pair_accepted ? 1 : 0)
              << " pair=" << junction->through_pair_neighbor_a_id << "/" << junction->through_pair_neighbor_b_id
              << " existingAkind=" << static_cast<int>(relation_kind_for(existing_a_id))
              << " existingBkind=" << static_cast<int>(relation_kind_for(existing_b_id))
              << " newAkind=" << static_cast<int>(relation_kind_for(new_a_id))
              << " newBkind=" << static_cast<int>(relation_kind_for(new_b_id))
              << " generatedHv=" << generated_hv_count << " generatedCross=" << generated_cross_under_count << "\n";
    for (const auto& relation : junction->local_relations) {
      std::cerr << "[DBG] C274 rel neighbor=" << relation.neighbor_node_id
                << " kind=" << static_cast<int>(relation.kind)
                << " inRoute=" << (relation.in_route ? 1 : 0)
                << " inPair=" << (relation.in_through_pair ? 1 : 0)
                << " same=" << (relation.same_level_feasible ? 1 : 0)
                << " defaultLower=" << (relation.default_lower_required ? 1 : 0) << "\n";
    }
    for (ObjectId span_id : generated.value.generated_span_ids) {
      const auto span_view = state.view().inspect_span(span_id);
      const auto* span = state.view().edit_state().spans.find(span_id);
      const auto* bundle = (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
      if (!span_view.has_value() || bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
        continue;
      }
      std::cerr << "[DBG] C274 span=" << span_id << " flow=" << static_cast<int>(span_view->flow_kind)
                << " lowering=" << static_cast<int>(span_view->lowering_kind)
                << " same=" << (span_view->same_level_feasible ? 1 : 0)
                << " defaultLower=" << (span_view->default_lower_required ? 1 : 0) << "\n";
    }
  }
  return ok;
}

bool test_backbone_grouped_support_membership_is_visible_on_all_bundle_lanes() {
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
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.size() < 3) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  int spans_with_group = 0;
  int expected_group_id = -1;
  wire::core::Vec3d expected_mount{};
  wire::core::Vec3d expected_tip{};
  bool expected_anchor_set = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      return false;
    }
    const auto group = lowered_support_group_for_owner(*layout_view, center_id);
    if (!group.has_value()) {
      std::cerr << "[DBG] C255 missing grouped support span=" << span_id << "\n";
      return false;
    }
    ++spans_with_group;
    if (expected_group_id < 0) {
      expected_group_id = group->support_group_id;
      expected_mount = group->mount_world;
      expected_tip = group->tip_world;
      expected_anchor_set = true;
    } else if (group->support_group_id != expected_group_id ||
               !almost_equal(group->mount_world.x, expected_mount.x, 1e-6) ||
               !almost_equal(group->mount_world.y, expected_mount.y, 1e-6) ||
               !almost_equal(group->mount_world.z, expected_mount.z, 1e-6) ||
               !almost_equal(group->tip_world.x, expected_tip.x, 1e-6) ||
               !almost_equal(group->tip_world.y, expected_tip.y, 1e-6) ||
               !almost_equal(group->tip_world.z, expected_tip.z, 1e-6)) {
      std::cerr << "[DBG] C255 mismatched support identity span=" << span_id << " group=" << group->support_group_id
                << " expected=" << expected_group_id << "\n";
      return false;
    }
    if (group->grouped_port_count < 3) {
      std::cerr << "[DBG] C255 grouped_port_count=" << group->grouped_port_count << " span=" << span_id << "\n";
      return false;
    }
  }
  return expected_anchor_set && spans_with_group == static_cast<int>(generated.value.generated_span_ids.size());
}

bool test_backbone_unrelated_generation_does_not_downgrade_existing_lowered_bundle_semantics() {
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
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto branch_generated = state.GenerateFromBackboneSpec(branch);
  if (!branch_generated.ok || branch_generated.value.generated_span_ids.size() < 3) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  auto verify_branch_bundle = [&]() {
    for (ObjectId span_id : branch_generated.value.generated_span_ids) {
      const auto layout_view = state.view().inspect_support_layout(span_id);
      if (!layout_view.has_value()) {
        return false;
      }
      if (layout_view->continuity_class != wire::core::ContinuityCategoryClass::kBundleLike ||
          layout_view->order_decision_policy != wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous) {
        std::cerr << "[DBG] C256 downgraded span=" << span_id << " class="
                  << static_cast<int>(layout_view->continuity_class) << " orderPolicy="
                  << static_cast<int>(layout_view->order_decision_policy) << "\n";
        return false;
      }
      const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
      const auto group = lowered_support_group_for_owner(*layout_view, center_id);
      if (!endpoint.has_value() || !endpoint_has_authoritative_lowering(*endpoint) || !group.has_value() ||
          group->support_group_id != endpoint->decision.support_group_id) {
        std::cerr << "[DBG] C256 missing grouped support span=" << span_id << "\n";
        return false;
      }
    }
    return true;
  };

  if (!verify_branch_bundle()) {
    return false;
  }

  wire::core::BackboneSpec unrelated{};
  unrelated.path.polyline = {{80.0, 0.0, 0.0}, {92.0, 0.0, 0.0}};
  unrelated.interval_m = 1000.0;
  unrelated.pole_type_id = type_ids.front();
  add_backbone_bundle(unrelated, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(unrelated).ok) {
    return false;
  }

  if (!state.SetPoleManualYawOverride(center_id, 12.0).ok) {
    return false;
  }

  return verify_branch_bundle();
}

bool test_backbone_corner_support_uses_connected_line_basis() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() < 2) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId corner_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (corner_id == wire::core::kInvalidObjectId) {
    std::cerr << "[DBG] C253 missing corner pole\n";
    return false;
  }

  std::vector<wire::core::LoweredSupportGroupInspectionView> placements{};
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    for (const auto& placement : layout_view->lowered_support_groups) {
      if (placement.owner_pole_id == corner_id) {
        placements.push_back(placement);
      }
    }
  }
  if (placements.empty()) {
    for (const auto& span_entry : state.view().spans().items()) {
      const auto support_layout = state.view().inspect_support_layout(span_entry.id);
      if (!support_layout.has_value()) {
        continue;
      }
      const auto endpoint_matches = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
        return endpoint.owner_pole_id == corner_id && endpoint.decision.lower_required &&
               (endpoint.decision.support_orientation_basis ==
                    wire::core::SupportOrientationBasisKind::kBisectorForward ||
                endpoint.decision.support_orientation_basis ==
                    wire::core::SupportOrientationBasisKind::kBisectorReverse);
      };
      if (endpoint_matches(support_layout->start_endpoint) || endpoint_matches(support_layout->end_endpoint)) {
        return true;
      }
    }
    std::cerr << "[DBG] C253 no corner visual placement and no non-radial lowered corner decision\n";
    return false;
  }
  const int group_id = placements.front().support_group_id;
  const double side_sign = placements.front().chosen_side_sign;
  for (const auto& placement : placements) {
    if (placement.support_group_id != group_id ||
        placement.side_assignment_rule != wire::core::SideAssignmentRuleKind::kBisector ||
        placement.support_orientation_rule != wire::core::SupportOrientationRuleKind::kBisector ||
        (placement.decision.support_orientation_basis != wire::core::SupportOrientationBasisKind::kBisectorForward &&
         placement.decision.support_orientation_basis != wire::core::SupportOrientationBasisKind::kBisectorReverse) ||
        placement.support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial ||
        std::abs(placement.chosen_side_sign - side_sign) > 1e-9 ||
        !almost_equal(placement.mount_world.x, placements.front().mount_world.x, 1e-6) ||
        !almost_equal(placement.mount_world.y, placements.front().mount_world.y, 1e-6) ||
        !almost_equal(placement.tip_world.x, placements.front().tip_world.x, 1e-6) ||
        !almost_equal(placement.tip_world.y, placements.front().tip_world.y, 1e-6)) {
      std::cerr << "[DBG] C253 corner group/sign mismatch group=" << placement.support_group_id
                << " refGroup=" << group_id << " orient=" << static_cast<int>(placement.support_orientation_rule)
                << " side=" << placement.chosen_side_sign << " refSide=" << side_sign << "\n";
      return false;
    }
  }
  return true;
}

bool test_backbone_lowered_bundle_midspan_support_uses_pair_based_orientation() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}, {12.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() < 9) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId downstream_corner_id = find_pole_id_by_position(state, {0.0, 12.0, 0.0});
  if (downstream_corner_id == wire::core::kInvalidObjectId) {
    return false;
  }

  struct Snapshot {
    ObjectId span_id = wire::core::kInvalidObjectId;
    int support_group_id = -1;
    ObjectId pair_peer_low = wire::core::kInvalidObjectId;
    ObjectId pair_peer_high = wire::core::kInvalidObjectId;
    wire::core::SideAssignmentRuleKind side_rule = wire::core::SideAssignmentRuleKind::kPoleLocal;
    wire::core::SupportOrientationRuleKind orientation_rule = wire::core::SupportOrientationRuleKind::kRadial;
    wire::core::SupportOrientationBasisKind orientation_basis = wire::core::SupportOrientationBasisKind::kRadial;
    double side_sign = 0.0;
    wire::core::Vec3d side_axis{};
  };
  auto collect = [&]() {
    std::vector<Snapshot> snapshots{};
    int expected_group_id = -1;
    ObjectId expected_pair_low = wire::core::kInvalidObjectId;
    ObjectId expected_pair_high = wire::core::kInvalidObjectId;
    double expected_side_sign = 0.0;
    wire::core::Vec3d expected_axis{};
    for (ObjectId span_id : generated.value.generated_span_ids) {
      const auto layout = state.view().inspect_support_layout(span_id);
      if (!layout.has_value()) {
        continue;
      }
      const auto authoritative_group = [&]() -> const wire::core::LoweredSupportGroupInspectionView* {
        for (const auto& group : layout->lowered_support_groups) {
          if (group.owner_pole_id == downstream_corner_id) {
            return &group;
          }
        }
        return nullptr;
      }();
      const auto inspect_endpoint = [&](const wire::core::SupportLayoutEndpointView& endpoint) {
        if (endpoint.owner_pole_id != downstream_corner_id || !endpoint.decision.lower_required ||
            endpoint.decision.support_group_id < 0) {
          return true;
        }
        if (authoritative_group == nullptr || authoritative_group->support_group_id != endpoint.decision.support_group_id) {
          return false;
        }
        const bool pair_based =
            endpoint.side_assignment_rule == wire::core::SideAssignmentRuleKind::kBisector &&
            endpoint.support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector &&
            (endpoint.decision.support_orientation_basis ==
                 wire::core::SupportOrientationBasisKind::kBisectorForward ||
             endpoint.decision.support_orientation_basis ==
                 wire::core::SupportOrientationBasisKind::kBisectorReverse);
        if (!pair_based || endpoint.support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial ||
            authoritative_group->pair_peer_low == wire::core::kInvalidObjectId ||
            authoritative_group->pair_peer_high == wire::core::kInvalidObjectId) {
          return false;
        }
        if (expected_group_id < 0) {
          expected_group_id = endpoint.decision.support_group_id;
          expected_pair_low = authoritative_group->pair_peer_low;
          expected_pair_high = authoritative_group->pair_peer_high;
          expected_side_sign = endpoint.chosen_side_sign;
          expected_axis = endpoint.side_axis;
        }
        if (endpoint.decision.support_group_id != expected_group_id ||
            authoritative_group->pair_peer_low != expected_pair_low ||
            authoritative_group->pair_peer_high != expected_pair_high ||
            std::abs(endpoint.chosen_side_sign - expected_side_sign) > 1e-9 ||
            !almost_equal(endpoint.side_axis.x, expected_axis.x, 1e-6) ||
            !almost_equal(endpoint.side_axis.y, expected_axis.y, 1e-6)) {
          return false;
        }
        Snapshot snapshot{};
        snapshot.span_id = span_id;
        snapshot.support_group_id = endpoint.decision.support_group_id;
        snapshot.pair_peer_low = authoritative_group->pair_peer_low;
        snapshot.pair_peer_high = authoritative_group->pair_peer_high;
        snapshot.side_rule = endpoint.side_assignment_rule;
        snapshot.orientation_rule = endpoint.support_orientation_rule;
        snapshot.orientation_basis = endpoint.decision.support_orientation_basis;
        snapshot.side_sign = endpoint.chosen_side_sign;
        snapshot.side_axis = endpoint.side_axis;
        snapshots.push_back(snapshot);
        return true;
      };
      if (!inspect_endpoint(layout->start_endpoint) || !inspect_endpoint(layout->end_endpoint)) {
        return std::optional<std::vector<Snapshot>>{};
      }
    }
    std::sort(snapshots.begin(), snapshots.end(), [](const Snapshot& a, const Snapshot& b) {
      if (a.span_id != b.span_id) {
        return a.span_id < b.span_id;
      }
      return a.support_group_id < b.support_group_id;
    });
    return std::optional<std::vector<Snapshot>>{snapshots};
  };

  const auto before = collect();
  if (!before.has_value()) {
    return false;
  }
  if (before->size() < 2) {
    return false;
  }
  if (!state.SetPoleManualYawOverride(downstream_corner_id, 17.0).ok) {
    return false;
  }
  const auto after = collect();
  if (!after.has_value()) {
    return false;
  }
  if (before->size() != after->size()) {
    return false;
  }
  for (std::size_t i = 0; i < before->size(); ++i) {
    const Snapshot& b = (*before)[i];
    const Snapshot& a = (*after)[i];
    if (b.span_id != a.span_id || b.support_group_id != a.support_group_id ||
        b.pair_peer_low != a.pair_peer_low || b.pair_peer_high != a.pair_peer_high ||
        b.side_rule != a.side_rule || b.orientation_rule != a.orientation_rule ||
        b.orientation_basis != a.orientation_basis || !almost_equal(b.side_sign, a.side_sign, 1e-9) ||
        !almost_equal(b.side_axis.x, a.side_axis.x, 1e-6) ||
        !almost_equal(b.side_axis.y, a.side_axis.y, 1e-6)) {
      return false;
    }
  }
  return true;
}

bool test_backbone_pair_based_orientation_allows_opposite_signs_per_pole() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}, {12.0, 12.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() < 9) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout = state.view().inspect_support_layout(span_id);
    if (!layout.has_value()) {
      continue;
    }
    const auto& start = layout->start_endpoint;
    const auto& end = layout->end_endpoint;
    const bool both_grouped = start.decision.lower_required && end.decision.lower_required &&
                              start.decision.support_group_id >= 0 && end.decision.support_group_id >= 0;
    if (!both_grouped) {
      continue;
    }
    const bool pair_based =
        start.side_assignment_rule == wire::core::SideAssignmentRuleKind::kBisector &&
        end.side_assignment_rule == wire::core::SideAssignmentRuleKind::kBisector &&
        start.support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector &&
        end.support_orientation_rule == wire::core::SupportOrientationRuleKind::kBisector &&
        start.decision.support_orientation_basis == wire::core::SupportOrientationBasisKind::kBisectorForward &&
        end.decision.support_orientation_basis == wire::core::SupportOrientationBasisKind::kBisectorForward;
    if (!pair_based) {
      continue;
    }
    if (!almost_equal(start.side_axis.x, end.side_axis.x, 1e-6) ||
        !almost_equal(start.side_axis.y, end.side_axis.y, 1e-6)) {
      return false;
    }
    if (std::abs(start.chosen_side_sign) <= 1e-9 || std::abs(end.chosen_side_sign) <= 1e-9) {
      return false;
    }
    return start.chosen_side_sign * end.chosen_side_sign < 0.0;
  }

  return false;
}

bool test_backbone_point_like_orientation_rule_non_regression() {
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
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.size() != 1) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }
  const auto layout_view = state.view().inspect_support_layout(generated.value.generated_span_ids.front());
  if (!layout_view.has_value()) {
    return false;
  }
  const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
  return endpoint.has_value() &&
         endpoint->continuity_class == wire::core::ContinuityCategoryClass::kPointLike &&
         endpoint->support_orientation_rule == wire::core::SupportOrientationRuleKind::kRadial &&
         !endpoint->used_junction_pair_side_assignment &&
      layout_view->lowered_support_groups.empty();
}

bool test_backbone_non_lowered_cross_spans_do_not_expose_lowered_support_groups() {
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
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  bool saw_lowered_cross = false;
  bool saw_non_lowered_center_span = false;
  for (const auto& span_entry : state.view().spans().items()) {
    const auto* span = state.view().edit_state().spans.find(span_entry.id);
    if (span == nullptr || (span->endpoint_node_a_id != center_id && span->endpoint_node_b_id != center_id)) {
      continue;
    }
    const auto layout_view = state.view().inspect_support_layout(span_entry.id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value()) {
      continue;
    }
    if (endpoint->decision.lower_required) {
      saw_lowered_cross = true;
      continue;
    }
    saw_non_lowered_center_span = true;
    if (!layout_view->lowered_support_groups.empty()) {
      std::cerr << "[DBG] C267 span=" << span_entry.id
                << " lowerRequired=" << endpoint->decision.lower_required
                << " groupCount=" << layout_view->lowered_support_groups.size()
                << " relationA=" << static_cast<int>(layout_view->relation_a)
                << " relationB=" << static_cast<int>(layout_view->relation_b) << "\n";
      return false;
    }
  }

  return saw_lowered_cross && saw_non_lowered_center_span;
}

bool test_backbone_non_lowered_spans_do_not_inherit_acute_corner_support_groups() {
  CoreState state;
  PoleTypeId pole_type_id = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == "CommunicationPole") {
      pole_type_id = id;
      break;
    }
  }
  if (pole_type_id == wire::core::kInvalidPoleTypeId) {
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
  if (!state.GenerateFromBackboneSpec(trunk).ok) {
    return false;
  }

  const ObjectId root_id = find_pole_id_by_position(state, {-6.59678, 11.0534, 0.0}, 1e-4);
  if (root_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch{};
  branch.path.polyline = {
      {-6.59678, 11.0534, 0.0},
      {-4.93216, 6.7054, 0.0},
      {-13.6709, -1.24875, 0.0},
      {-8.49996, -0.441201, 0.0},
  };
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 0;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = root_id;
  branch.path.node_specs.push_back(shared);
  branch.interval_m = 8.0;
  branch.pole_type_id = pole_type_id;
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(branch, wire::core::BundleKind::kOptical);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  bool saw_lowered = false;
  bool saw_flat_after_lowered = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto span_view = state.view().inspect_span(span_id);
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!span_view.has_value() || !layout_view.has_value()) {
      continue;
    }
    const auto* span = state.view().edit_state().spans.find(span_id);
    const auto* bundle = (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    const bool has_lowered_endpoint =
        layout_view->start_endpoint.decision.lower_required || layout_view->end_endpoint.decision.lower_required;
    if (has_lowered_endpoint) {
      saw_lowered = true;
      continue;
    }
    saw_flat_after_lowered = true;
      if (!layout_view->lowered_support_groups.empty()) {
        std::cerr << "[DBG] C268 span=" << span_id
                  << " groupCount=" << layout_view->lowered_support_groups.size()
                  << " flow=" << static_cast<int>(span_view->flow_kind)
                  << " startLower=" << layout_view->start_endpoint.decision.lower_required
                  << " endLower=" << layout_view->end_endpoint.decision.lower_required << "\n";
        return false;
      }
  }

  return saw_lowered && saw_flat_after_lowered;
}

bool test_backbone_refresh_keeps_lowered_side_and_orientation_origin() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  if (!state.SetPoleManualYawOverride(center_id, 15.0).ok) {
    return false;
  }

  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value()) {
      continue;
    }
    if (layout_view->lowering_kind == wire::core::BackboneLoweringKind::kCrossUnderpass &&
        endpoint->support_orientation_rule != wire::core::SupportOrientationRuleKind::kRadial &&
        endpoint->has_side_axis &&
        std::abs(endpoint->chosen_side_sign) > 0.5) {
      return true;
    }
  }
  return false;
}

bool test_backbone_hv3_same_level_order_decision_is_permutable() {
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
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  bool saw_permutable = false;
  bool saw_compared_choice = false;
  for (const auto& orientation : state.view().last_generation_edge_orientations()) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    saw_permutable = saw_permutable ||
                     orientation.order_decision_policy == wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous;
    const bool compared_a = orientation.order_decision_choice_reason_a != wire::core::OrderDecisionChoiceReason::kFixedOrder;
    const bool compared_b = orientation.order_decision_choice_reason_b != wire::core::OrderDecisionChoiceReason::kFixedOrder;
    saw_compared_choice = saw_compared_choice || compared_a || compared_b;
  }
  if (!(saw_permutable && saw_compared_choice)) {
    std::cerr << "[DBG] C239 permutable=" << saw_permutable << " compared=" << saw_compared_choice << "\n";
  }
  return saw_permutable && saw_compared_choice;
}

bool test_backbone_hv3_lowered_order_decision_is_permutable() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  bool saw_lowered = false;
  bool saw_permutable = false;
  bool saw_compared_choice = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value() ||
        layout_view->lowering_kind != wire::core::BackboneLoweringKind::kCrossUnderpass) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value()) {
      continue;
    }
    saw_lowered = true;
    saw_permutable = saw_permutable ||
                     endpoint->order_decision_policy == wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous;
    saw_compared_choice = saw_compared_choice ||
                          endpoint->order_decision_choice_reason != wire::core::OrderDecisionChoiceReason::kFixedOrder;
  }
  if (!(saw_lowered && saw_permutable && saw_compared_choice)) {
    std::cerr << "[DBG] C240 lowered=" << saw_lowered << " permutable=" << saw_permutable
              << " compared=" << saw_compared_choice << "\n";
  }
  return saw_lowered && saw_permutable && saw_compared_choice;
}

bool test_backbone_fixed_order_bundle_skips_permutation() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  auto& bundle_templates = wire::core::CoreStateTestHook::bundle_templates(state);
  const auto it = bundle_templates.find(wire::core::BundleKind::kHighVoltage);
  if (it == bundle_templates.end()) {
    return false;
  }
  it->second.preserve_conductor_identity = true;

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
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  bool saw_hv = false;
  for (const auto& orientation : state.view().last_generation_edge_orientations()) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    saw_hv = true;
    if (orientation.order_decision_policy != wire::core::OrderDecisionPolicyKind::kFixedOrder ||
        orientation.order_decision_choice_reason_a != wire::core::OrderDecisionChoiceReason::kFixedOrder ||
        orientation.order_decision_choice_reason_b != wire::core::OrderDecisionChoiceReason::kFixedOrder) {
      std::cerr << "[DBG] C241 policy=" << static_cast<int>(orientation.order_decision_policy)
                << " choiceA=" << static_cast<int>(orientation.order_decision_choice_a)
                << " reasonA=" << static_cast<int>(orientation.order_decision_choice_reason_a)
                << " choiceB=" << static_cast<int>(orientation.order_decision_choice_b)
                << " reasonB=" << static_cast<int>(orientation.order_decision_choice_reason_b) << "\n";
      return false;
    }
  }
  return saw_hv;
}

bool test_backbone_refresh_keeps_order_decision_choice() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId span_id = generated.value.generated_span_ids.front();
  const auto before = state.view().inspect_support_layout(span_id);
  if (!before.has_value()) {
    return false;
  }
  const auto before_endpoint = layout_endpoint_for_owner(*before, center_id);
  if (!before_endpoint.has_value()) {
    return false;
  }
  const auto before_choice = before_endpoint->order_decision_choice;
  const auto before_reason = before_endpoint->order_decision_choice_reason;

  if (!state.SetPoleManualYawOverride(center_id, 15.0).ok) {
    return false;
  }

  const auto after = state.view().inspect_support_layout(span_id);
  if (!after.has_value()) {
    return false;
  }
  const auto after_endpoint = layout_endpoint_for_owner(*after, center_id);
  return after_endpoint.has_value() &&
         before_endpoint->order_decision_policy == wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous &&
         after_endpoint->order_decision_policy == wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous &&
         after_endpoint->order_decision_choice == before_choice &&
         after_endpoint->order_decision_choice_reason == before_reason;
}

bool test_backbone_point_like_order_decision_non_regression() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 4.0, 0.0}};
  req.interval_m = 6.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  bool saw_lv = false;
  for (const auto& orientation : state.view().last_generation_edge_orientations()) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kLowVoltage) {
      continue;
    }
    saw_lv = true;
    if (orientation.order_decision_policy != wire::core::OrderDecisionPolicyKind::kFixedOrder ||
        orientation.order_decision_choice_reason_a != wire::core::OrderDecisionChoiceReason::kFixedOrder ||
        orientation.order_decision_choice_reason_b != wire::core::OrderDecisionChoiceReason::kFixedOrder) {
      std::cerr << "[DBG] C243 unexpected point-like bundle order policy=" << static_cast<int>(orientation.order_decision_policy)
                << " reasonA=" << static_cast<int>(orientation.order_decision_choice_reason_a)
                << " reasonB=" << static_cast<int>(orientation.order_decision_choice_reason_b) << "\n";
      return false;
    }
  }
  return saw_lv;
}

bool test_backbone_authoritative_endpoint_decision_matches_support_layout() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId span_id = generated.value.generated_span_ids.front();
  const auto layout_view = state.view().inspect_support_layout(span_id);
  const auto assignment = find_assignment_for_span(state, span_id);
  if (!layout_view.has_value() || !assignment.has_value()) {
    return false;
  }
  const bool same_start =
      layout_view->start_endpoint.decision.owner_pole_id == layout_view->start_endpoint.owner_pole_id &&
      layout_view->start_endpoint.decision.owner_pole_id == assignment->decision_a.owner_pole_id &&
      layout_view->start_endpoint.decision.order_decision_choice == assignment->decision_a.order_decision_choice &&
      layout_view->start_endpoint.decision.chosen_side == assignment->decision_a.chosen_side &&
      layout_view->start_endpoint.decision.support_orientation_basis == assignment->decision_a.support_orientation_basis &&
      layout_view->start_endpoint.decision.lower_required == assignment->decision_a.lower_required &&
      layout_view->start_endpoint.decision.relation_kind == assignment->decision_a.relation_kind &&
      !layout_view->start_endpoint.decision.downstream_overridden;
  const bool same_end =
      layout_view->end_endpoint.decision.owner_pole_id == layout_view->end_endpoint.owner_pole_id &&
      layout_view->end_endpoint.decision.owner_pole_id == assignment->decision_b.owner_pole_id &&
      layout_view->end_endpoint.decision.order_decision_choice == assignment->decision_b.order_decision_choice &&
      layout_view->end_endpoint.decision.chosen_side == assignment->decision_b.chosen_side &&
      layout_view->end_endpoint.decision.support_orientation_basis == assignment->decision_b.support_orientation_basis &&
      layout_view->end_endpoint.decision.lower_required == assignment->decision_b.lower_required &&
      layout_view->end_endpoint.decision.relation_kind == assignment->decision_b.relation_kind &&
      !layout_view->end_endpoint.decision.downstream_overridden;
  return same_start && same_end;
}

bool test_backbone_refresh_does_not_override_authoritative_endpoint_decision() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId span_id = generated.value.generated_span_ids.front();
  const auto before = state.view().inspect_support_layout(span_id);
  if (!before.has_value()) {
    return false;
  }
  const auto before_endpoint = layout_endpoint_for_owner(*before, center_id);
  if (!before_endpoint.has_value()) {
    return false;
  }
  const auto before_decision = before_endpoint->decision;

  if (!state.SetPoleManualYawOverride(center_id, 17.0).ok) {
    return false;
  }

  const auto after = state.view().inspect_support_layout(span_id);
  if (!after.has_value()) {
    return false;
  }
  const auto after_endpoint = layout_endpoint_for_owner(*after, center_id);
  return after_endpoint.has_value() &&
         after_endpoint->decision.owner_pole_id == before_decision.owner_pole_id &&
         after_endpoint->decision.owner_pole_id == after_endpoint->owner_pole_id &&
         after_endpoint->decision.order_decision_choice == before_decision.order_decision_choice &&
         after_endpoint->decision.chosen_side == before_decision.chosen_side &&
         after_endpoint->decision.support_orientation_basis == before_decision.support_orientation_basis &&
         after_endpoint->decision.lower_required == before_decision.lower_required &&
         !after_endpoint->decision.downstream_overridden;
}

bool test_backbone_authoritative_cross_pair_side_symmetry() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  std::optional<wire::core::LateralSideChoiceKind> shared_side{};
  bool saw_pair_based = false;
  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value() ||
        layout_view->lowering_kind != wire::core::BackboneLoweringKind::kCrossUnderpass) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value()) {
      continue;
    }
    saw_pair_based = saw_pair_based || endpoint->decision.used_junction_pair_side_assignment;
    if (endpoint->decision.chosen_side == wire::core::LateralSideChoiceKind::kCenter) {
      return false;
    }
    if (!shared_side.has_value()) {
      shared_side = endpoint->decision.chosen_side;
    } else if (*shared_side != endpoint->decision.chosen_side) {
      return false;
    }
  }
  return saw_pair_based && shared_side.has_value();
}

bool test_backbone_constrained_orientation_uses_authoritative_basis() {
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
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec cross{};
  cross.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  cross.interval_m = 1000.0;
  cross.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared{};
  shared.point_index = 1;
  shared.support_kind = wire::core::SupportKind::kPole;
  shared.node_id = center_id;
  cross.path.node_specs.push_back(shared);
  add_backbone_bundle(cross, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(cross);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  for (ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value()) {
      continue;
    }
    const auto endpoint = layout_endpoint_for_owner(*layout_view, center_id);
    if (!endpoint.has_value() || endpoint->origin != "PlacementConstraint") {
      continue;
    }
    const auto placement = lowered_support_group_for_owner(*layout_view, center_id);
    if (!placement.has_value() || placement->origin != "PlacementConstraint") {
      continue;
    }
    return placement->decision.support_orientation_basis == endpoint->decision.support_orientation_basis &&
           placement->decision.order_decision_choice == endpoint->decision.order_decision_choice &&
           placement->decision.chosen_side == endpoint->decision.chosen_side &&
           placement->decision.support_orientation_basis != wire::core::SupportOrientationBasisKind::kRadial;
  }
  return false;
}

bool test_backbone_hv3_authoritative_order_decision_survives_refresh() {
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
  };
  req.interval_m = 8.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  if (!state.Commit(options).validation.ok()) {
    return false;
  }

  const ObjectId span_id = generated.value.generated_span_ids.front();
  const auto before = state.view().inspect_support_layout(span_id);
  if (!before.has_value()) {
    return false;
  }
  const auto before_decision = before->start_endpoint.decision;
  if (before_decision.order_decision_policy != wire::core::OrderDecisionPolicyKind::kPermutableHomogeneous) {
    return false;
  }

  const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
  if (span == nullptr) {
    return false;
  }
  if (!state.SetPoleManualYawOverride(span->endpoint_node_a_id, 11.0).ok) {
    return false;
  }

  const auto after = state.view().inspect_support_layout(span_id);
  return after.has_value() &&
         after->start_endpoint.decision.order_decision_choice == before_decision.order_decision_choice &&
         after->start_endpoint.decision.order_decision_choice_reason == before_decision.order_decision_choice_reason &&
         after->start_endpoint.decision.downstream_overridden == false;
}

bool test_backbone_edge_orientation_uses_chosen_order_decision() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {8.0, 6.0, 0.0}, {18.0, 6.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  const auto& orientations = state.view().last_generation_edge_orientations();
  bool saw_hv = false;
  for (const auto& orientation : orientations) {
    if (orientation.bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    saw_hv = true;
    const auto expected = (orientation.order_decision_choice_a != orientation.order_decision_choice_b)
                              ? wire::core::LaneOrientation::kReversed
                              : wire::core::LaneOrientation::kNormal;
    if (orientation.orientation != expected) {
      std::cerr << "[DBG] C249 orientation mismatch edge=" << orientation.node_a_id << "->" << orientation.node_b_id
                << " orderA=" << static_cast<int>(orientation.order_decision_choice_a)
                << " orderB=" << static_cast<int>(orientation.order_decision_choice_b)
                << " orientation=" << static_cast<int>(orientation.orientation)
                << " expected=" << static_cast<int>(expected) << "\n";
      return false;
    }
  }
  return saw_hv;
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
    const auto layout_view = state.view().inspect_support_layout(span_id);
    if (!layout_view.has_value() || layout_view->lowered_support_groups.empty()) {
      continue;
    }
    found_branch_support = found_branch_support ||
                           std::any_of(layout_view->lowered_support_groups.begin(),
                                       layout_view->lowered_support_groups.end(),
                                       [center_id](const wire::core::LoweredSupportGroupInspectionView& placement) {
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

bool test_backbone_crosslike_single_edge_stays_main() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec horizontal{};
  horizontal.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  horizontal.interval_m = 1000.0;
  horizontal.pole_type_id = type_ids.front();
  add_backbone_bundle(horizontal, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(horizontal).ok) {
    return false;
  }

  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec vertical{};
  vertical.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec shared_center{};
  shared_center.point_index = 1;
  shared_center.support_kind = wire::core::SupportKind::kPole;
  shared_center.node_id = center_id;
  vertical.path.node_specs.push_back(shared_center);
  vertical.interval_m = 1000.0;
  vertical.pole_type_id = type_ids.front();
  add_backbone_bundle(vertical, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(vertical).ok) {
    return false;
  }

  wire::core::BackboneSpec single{};
  single.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec anchored{};
  anchored.point_index = 0;
  anchored.support_kind = wire::core::SupportKind::kPole;
  anchored.node_id = center_id;
  single.path.node_specs.push_back(anchored);
  single.interval_m = 1000.0;
  single.pole_type_id = type_ids.front();
  add_backbone_bundle(single, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(single);
  if (!generated.ok || generated.value.generated_span_ids.size() != 1) {
    return false;
  }

  const auto& assignments = state.view().last_lane_assignments();
  if (assignments.size() != 1) {
    return false;
  }
  const auto span_view = state.view().inspect_span(generated.value.generated_span_ids.front());
  const auto layout_view = state.view().inspect_support_layout(generated.value.generated_span_ids.front());
  return span_view.has_value() && layout_view.has_value() &&
         assignments.front().flow_kind == wire::core::BackboneFlowKind::kMain &&
         !assignments.front().decision_a.lower_required && !assignments.front().decision_b.lower_required &&
         assignments.front().decision_a.support_group_id < 0 && assignments.front().decision_b.support_group_id < 0 &&
         assignments.front().branch_down_offset_m == 0.0 &&
         layout_view->lowered_support_groups.empty() && span_view->same_level_feasible;
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

std::optional<wire::core::SupportLayoutEndpointView> layout_endpoint_for_owner(
    const wire::core::SupportLayoutInspectionView& layout_view, wire::core::ObjectId owner_pole_id) {
  if (layout_view.start_endpoint.owner_pole_id == owner_pole_id) {
    return layout_view.start_endpoint;
  }
  if (layout_view.end_endpoint.owner_pole_id == owner_pole_id) {
    return layout_view.end_endpoint;
  }
  return std::nullopt;
}

std::optional<wire::core::LoweredSupportGroupInspectionView> lowered_support_group_for_owner(
    const wire::core::SupportLayoutInspectionView& layout_view, wire::core::ObjectId owner_pole_id) {
  for (const auto& group : layout_view.lowered_support_groups) {
    if (group.owner_pole_id == owner_pole_id) {
      return group;
    }
  }
  return std::nullopt;
}

std::optional<wire::core::SegmentLaneAssignment> find_assignment_for_span(const wire::core::CoreState& state,
                                                                          wire::core::ObjectId span_id) {
  const wire::core::Span* span = state.view().edit_state().spans.find(span_id);
  if (span == nullptr) {
    return std::nullopt;
  }
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const bool same_forward = assignment.bundle_id == span->bundle_id && assignment.pole_a_id == span->endpoint_node_a_id &&
                              assignment.pole_b_id == span->endpoint_node_b_id;
    const bool same_reverse = assignment.bundle_id == span->bundle_id && assignment.pole_a_id == span->endpoint_node_b_id &&
                              assignment.pole_b_id == span->endpoint_node_a_id;
    if (same_forward || same_reverse) {
      return assignment;
    }
  }
  return std::nullopt;
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
        a[i].order_decision_choice_a != b[i].order_decision_choice_a ||
        a[i].order_decision_choice_b != b[i].order_decision_choice_b ||
        a[i].flipped_from_previous != b[i].flipped_from_previous ||
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
  test_registry::AddTest(tests, "C134_Backbone_BundleBranchCreatesGroupedLoweredSupportAtRoot",
                         "Bundle-like branch root exposes authoritative lowered decision and grouped support", "Invariant",
                         false, test_backbone_branch_bundle_uses_branch_support_ports);
  test_registry::AddTest(tests, "C135_Backbone_BranchSupportDownOffset",
                         "Branch support lowers attachment height without rewriting layer semantics", "Invariant",
                         false, test_backbone_branch_support_offsets_height_without_changing_layer);
  test_registry::AddTest(tests, "C193_Backbone_BundleBranchUsesOneSharedLowerStep",
                         "HV3 bundle branch uses one shared lower step across the whole bundle while keeping a perpendicular row",
                         "Invariant", false, test_backbone_branch_support_lowers_hv3_bundle_uniformly);
  test_registry::AddTest(tests, "C195_Backbone_BundleBranchLoweringStaysLocalToRootPole",
                         "Bundle-like branch lowering stays on the branch root and downstream branch poles return to a flat perpendicular row",
                         "Invariant", false, test_backbone_branch_support_stays_local_to_root_pole);
  test_registry::AddTest(tests, "C196_Backbone_BranchSupportVisualPerpendicular",
                         "Branch support visual placement stays perpendicular to the branch and hangs vertically under the lane attachment",
                         "Invariant", false, test_backbone_branch_support_visual_stays_perpendicular_to_branch);
  test_registry::AddTest(tests, "C197_Backbone_DefaultSingleBranchStaysFlat",
                         "DefaultSingle branch stays branch-flow without authoritative lowered decision or height drop",
                         "Invariant", false, test_backbone_default_single_branch_stays_flat_without_branch_support);
  test_registry::AddTest(tests, "C198_Backbone_CommunicationBundleBranchPolicyBlocked",
                         "Communication multi-bundle branch exposes a policy-blocked lowered decision without materializing grouped support",
                         "Invariant", false, test_backbone_communication_bundle_branch_stays_flat_without_branch_support);
  test_registry::AddTest(tests, "C199_Backbone_HV3BundleBranchUsesOneSharedLowerStepOnBothPoleTypes",
                         "HV3 bundle branch stays on one shared lower step and keeps a perpendicular grouped support on both default pole types",
                         "Invariant", false, test_backbone_hv3_branch_support_policy_applies_on_both_default_pole_types);
  test_registry::AddTest(tests, "C204_Backbone_HV3CornerUsesOneSharedLowerStep",
                         "HV3 corner continuation uses the same one-step lowered height across the whole bundle without turning it into branch support",
                         "Invariant", false, test_backbone_hv3_acute_corner_lowers_corner_bundle_without_branch_support);
  test_registry::AddTest(tests, "C205_Backbone_HV3CornerOneStepLowerSurvivesPoleRefresh",
                         "HV3 corner one-step lowered support identity survives pole refresh without dropping back to template height",
                         "Invariant", false, test_backbone_hv3_acute_corner_lowering_survives_pole_refresh);
  test_registry::AddTest(tests, "C209_Backbone_HV3ModerateCornerUsesOneSharedLowerStep",
                         "HV3 moderate corner still collapses to one shared lower step under the default corner threshold",
                         "Invariant", false, test_backbone_hv3_moderate_acute_corner_lowers_bundle_at_default_threshold);
  test_registry::AddTest(tests, "C194_Backbone_JunctionPrefersStraighterMainPair",
                         "Junction main selection prefers the straighter continuation pair over first-drawn primary",
                         "Invariant", false, test_backbone_junction_prefers_straighter_pair_over_first_drawn_primary);
  test_registry::AddTest(tests, "C201_Backbone_PointLikeCrossCanStaySameLevel",
                         "Point-like cross keeps branch classification but may stay same-level when near-node clearance allows",
                         "Invariant", false, test_backbone_cross_junction_nonmain_line_uses_underpass);
  test_registry::AddTest(tests, "C210_Backbone_AllTemplatesBranchKeepsHVAtOneStepLower",
                         "All-template DrawPath branch on communication poles keeps HV at one shared lower step",
                         "Invariant", false,
                         test_backbone_all_templates_branch_keeps_hv_down_offset_on_communication_pole);
  test_registry::AddTest(tests, "C211_Backbone_AllTemplatesCrossStillUsesUnderpassOnCommunicationPole",
                         "All-template DrawPath cross on communication poles still keeps at least one non-main line lowered",
                         "Invariant", false,
                         test_backbone_all_templates_cross_keeps_underpass_on_communication_pole);
  test_registry::AddTest(tests, "C212_Backbone_CaptureBranchAndCornerShareOneStepLowerOnCommunicationPole",
                         "Captured all-template communication-pole path keeps the HV branch root and later corner segments on the same one-step lowered height",
                         "Invariant", false, test_backbone_capture_branch_then_acute_lowering_on_communication_pole);
  test_registry::AddTest(tests, "C264_Inspection_AllTemplatesBranchKeepsHVOneStepLowerOnCommunicationPole",
                         "Inspection surface keeps HV branch lowering on the same one-step grouped height in the all-template branch case",
                         "Invariant", false,
                         test_inspection_all_templates_branch_keeps_hv_lowering_on_communication_pole);
  test_registry::AddTest(tests, "C265_Inspection_CaptureKeepsBranchAndCornerOnOneStepLower",
                         "Inspection surface keeps the first HV branch segment and later HV corner segments on the same one-step lowered height in the captured communication-pole case",
                         "Invariant", false,
                         test_inspection_capture_keeps_branch_then_acute_lowering_on_communication_pole);
  test_registry::AddTest(tests, "C266_Inspection_SpanReadsFlowAndTurnFromLaneSnapshot",
                         "inspect_span reads flow rule and turn/flip metadata from the authoritative lane snapshot instead of leaving lane-era fields stale",
                         "Invariant", false, test_inspection_span_reads_flow_and_turn_from_lane_snapshot);
  test_registry::AddTest(tests, "C213_Backbone_RightAngleJunctionHasNoThroughPair",
                         "Right-angle junction exposes a rejected through pair and corner continuation relation in inspection/trace",
                         "Invariant", false, test_backbone_right_angle_junction_has_no_through_pair);
  test_registry::AddTest(tests, "C214_Backbone_LocalCornerProjectsToMainWithoutLocalThrough",
                         "Local corner can project to semantic main while inspection still reports no local through pair",
                         "Invariant", false, test_backbone_local_corner_projects_to_main_without_local_through);
  test_registry::AddTest(tests, "C215_Backbone_SeparateRouteMergeKeepsCornerContinuationRelation",
                         "Separate-route merge still records corner continuation and acute lowering on the later corner node",
                         "Invariant", false, test_backbone_separate_route_merge_keeps_corner_continuation_relation);
  test_registry::AddTest(tests, "C216_Backbone_MirrorDoesNotChangeRelationOrLoweringRoot",
                         "Mirror enablement changes lane order choices only and does not change relation or lowering roots",
                         "Invariant", false, test_backbone_mirror_does_not_change_relation_or_lowering_root);
  test_registry::AddTest(tests, "C217_Backbone_CrossAcceptedThroughButSameLevelInfeasible",
                         "Cross junction can accept a through pair while the non-through HV3 route still fails same-level clearance and lowers",
                         "Invariant", false, test_backbone_cross_through_pair_can_still_be_same_level_infeasible);
  test_registry::AddTest(tests, "C218_Backbone_CommBranchSameLevelBlockedByPolicy",
                         "Communication bundle branch can be same-level infeasible while category policy blocks lowering and reports that block",
                         "Invariant", false, test_backbone_comm_branch_same_level_can_be_blocked_by_policy);
  test_registry::AddTest(tests, "C219_Backbone_HV3CornerFeasibilityLoweringKeepsSemanticMain",
                         "HV3 right-angle main keeps semantic main flow while local corner feasibility drives acute lowering",
                         "Invariant", false, test_backbone_hv3_corner_feasibility_lowering_keeps_semantic_main);
  test_registry::AddTest(tests, "C220_Backbone_AcuteMergeFeasibilityAppliesAcrossRouteBoundary",
                         "Acute merge keeps a corner-based lowering root even when the corner arrives from a separate route boundary",
                         "Invariant", false, test_backbone_acute_merge_feasibility_applies_across_route_boundary);
  test_registry::AddTest(tests, "C221_Backbone_RecalcKeepsSameLevelLoweringOrigin",
                         "Recalc keeps same-level infeasibility and lowering origin visible on support layout and span inspection",
                         "Invariant", false, test_backbone_recalc_keeps_same_level_lowering_origin);
  test_registry::AddTest(tests, "C222_Backbone_HV3CornerUsesConstrainedBandSolver",
                         "HV3 corner uses constrained placement-band ports instead of special-case ports when same-level feasibility fails",
                         "Invariant", false, test_backbone_hv3_corner_uses_constrained_band_solver);
  test_registry::AddTest(tests, "C223_Backbone_CrossConstraintUsesSolver",
                         "Cross underpass can keep through classification while solving near-node clearance through constrained band placement",
                         "Invariant", false, test_backbone_cross_same_level_infeasible_can_use_constrained_solver);
  test_registry::AddTest(tests, "C224_Backbone_PolicyBlockedConflictSurvivesRecalc",
                         "Policy-blocked same-level conflicts remain visible as unresolved after recalc instead of silently disappearing",
                         "Invariant", false, test_backbone_policy_blocked_unresolved_survives_recalc_inspection);
  test_registry::AddTest(tests, "C225_Backbone_RefreshKeepsPlacementConstraintOrigin",
                         "Refresh keeps placement-constraint origin and constrained band source instead of snapping lowered corner ports back to normal bands",
                         "Invariant", false, test_backbone_refresh_keeps_placement_constraint_origin);
  test_registry::AddTest(tests, "C226_Backbone_MirrorDoesNotChangeConstrainedSolverUsage",
                         "Mirror continues to affect lane-order only and does not change constrained-solver usage or unresolved same-level state",
                         "Invariant", false, test_backbone_mirror_does_not_change_constrained_solver_usage);
  test_registry::AddTest(tests, "C227_Backbone_CrossRelationSurvivesSupportLayoutRecalc",
                         "Cross underpass relation and lowering survive support-layout recalc instead of collapsing entirely into Main/Branch",
                         "Invariant", false, test_backbone_cross_relation_survives_support_layout_recalc);
  test_registry::AddTest(tests, "C228_Backbone_HV3BranchDefaultLower",
                         "Bundle-like HV3 side branch defaults to lower even when same-level clearance would otherwise look acceptable",
                         "Invariant", false, test_backbone_hv3_branch_default_lower_required);
  test_registry::AddTest(tests, "C229_Backbone_HV3CornerDefaultLower",
                         "Bundle-like HV3 corner continuation defaults to lower instead of staying on the main row",
                         "Invariant", false, test_backbone_hv3_corner_continuation_default_lower_required);
  test_registry::AddTest(tests, "C230_Backbone_HV3CrossOnlyThroughPairSameLevel",
                         "Bundle-like HV3 cross keeps same-level continuity only for the accepted ThroughPair",
                         "Invariant", false, test_backbone_hv3_cross_only_through_pair_stays_same_level_candidate);
  test_registry::AddTest(tests, "C231_Backbone_PointLikeBranchCanStaySameLevel",
                         "Point-like branch remains feasibility-driven and can stay same-level when clearance allows",
                         "Invariant", false, test_backbone_point_like_branch_can_keep_same_level_when_clear);
  test_registry::AddTest(tests, "C232_Backbone_BundleRulePolicyBlockedUnresolved",
                         "Bundle-like default lower remains visible as unresolved when category policy blocks lowering",
                         "Invariant", false, test_backbone_bundle_rule_policy_block_stays_unresolved);
  test_registry::AddTest(tests, "C233_Backbone_RefreshKeepsBundleRuleOrigin",
                         "Refresh keeps bundle-rule lowering origin instead of dropping back to generic same-level placement",
                         "Invariant", false, test_backbone_refresh_keeps_bundle_rule_origin);
  test_registry::AddTest(tests, "C234_Backbone_CrossLoweredPairSymmetricSides",
                         "Cross lowered pair uses one junction-pair side group instead of splitting into left/right supports per endpoint",
                         "Invariant", false, test_backbone_cross_lowered_pair_uses_opposite_junction_pair_sides);
  test_registry::AddTest(tests, "C235_Backbone_ConstrainedLoweredSupportOrientation",
                         "Constrained-placement lowered support uses a line/chord-oriented visual rule instead of falling back to pole-radial orientation",
                         "Invariant", false, test_backbone_constrained_lowered_support_prefers_line_direction);
  test_registry::AddTest(tests, "C236_Backbone_BundleBranchOrientationUsesBisectorWhenAvailable",
                         "Bundle-like lowered branch root uses bisector orientation when available instead of falling back to branch chord",
                         "Invariant", false, test_backbone_bundle_branch_support_orientation_uses_bisector_when_available);
  test_registry::AddTest(tests, "C237_Backbone_PointLikeOrientationNonRegression",
                         "Point-like branch keeps radial/default orientation behavior and does not inherit bundle-like lowered support rules",
                         "Invariant", false, test_backbone_point_like_orientation_rule_non_regression);
  test_registry::AddTest(tests, "C267_Backbone_NonLoweredCrossSpansDoNotExposeLoweredSupportGroups",
                         "Cross junction main spans with lower_required=false do not expose grouped lowered support",
                         "Invariant", false, test_backbone_non_lowered_cross_spans_do_not_expose_lowered_support_groups);
  test_registry::AddTest(tests, "C268_Backbone_NonLoweredSpansDoNotInheritAcuteCornerSupportGroups",
                         "Bundle spans with lower_required=false do not inherit grouped lowered support from lowered segments",
                         "Invariant", false, test_backbone_non_lowered_spans_do_not_inherit_acute_corner_support_groups);
  test_registry::AddTest(tests, "C238_Backbone_RefreshKeepsSideOrientationOrigin",
                         "Refresh keeps lowered side/orientation origin visible instead of collapsing constrained or cross-lowered spans back to generic radial support rules",
                         "Invariant", false, test_backbone_refresh_keeps_lowered_side_and_orientation_origin);
  test_registry::AddTest(tests, "C239_Backbone_HV3SameLevelOrderDecisionPermutable",
                         "HV3 ThroughPair path keeps an authoritative non-fixed order decision instead of reverting to fixed-order",
                         "Invariant", false, test_backbone_hv3_same_level_order_decision_is_permutable);
  test_registry::AddTest(tests, "C240_Backbone_HV3LoweredOrderDecisionPermutable",
                         "Lowered HV3 cross keeps an authoritative non-fixed order decision at the lowered endpoint",
                         "Invariant", false, test_backbone_hv3_lowered_order_decision_is_permutable);
  test_registry::AddTest(tests, "C241_Backbone_FixedOrderBundleUntouched",
                         "Fixed-order bundle skips non-fixed order evaluation entirely",
                         "Invariant", false, test_backbone_fixed_order_bundle_skips_permutation);
  test_registry::AddTest(tests, "C242_Backbone_RefreshKeepsOrderDecision",
                         "Refresh keeps the chosen HV3 order decision instead of re-flipping after constrained lowered generation",
                         "Invariant", false, test_backbone_refresh_keeps_order_decision_choice);
  test_registry::AddTest(tests, "C243_Backbone_PointLikeOrderDecisionNonRegression",
                         "Point-like low-voltage route stays fixed-order and does not opt into HV3 order permutation",
                         "Invariant", false, test_backbone_point_like_order_decision_non_regression);
  test_registry::AddTest(tests, "C244_Backbone_AuthoritativeDecisionMatchesSupportLayout",
                         "Support layout copies grouped endpoint decisions without reinterpreting chosen order, side, or orientation basis",
                         "Invariant", false, test_backbone_authoritative_endpoint_decision_matches_support_layout);
  test_registry::AddTest(tests, "C245_Backbone_RefreshDoesNotOverrideAuthoritativeDecision",
                         "Refresh keeps the chosen order decision, side, and orientation basis instead of re-flipping them downstream",
                         "Invariant", false, test_backbone_refresh_does_not_override_authoritative_endpoint_decision);
  test_registry::AddTest(tests, "C246_Backbone_AuthoritativeCrossPairSideSymmetry",
                         "Cross lowered pair keeps one authoritative shared side choice instead of re-splitting into endpoint-local left/right supports",
                         "Invariant", false, test_backbone_authoritative_cross_pair_side_symmetry);
  test_registry::AddTest(tests, "C247_Backbone_ConstrainedOrientationUsesAuthoritativeBasis",
                         "Constrained placement support visuals reuse the authoritative orientation basis instead of recomputing a radial rule",
                         "Invariant", false, test_backbone_constrained_orientation_uses_authoritative_basis);
  test_registry::AddTest(tests, "C248_Backbone_HV3AuthoritativeOrderDecisionSurvivesRefresh",
                         "HV3 chosen order decision survives refresh as an authoritative result without downstream override",
                         "Invariant", false, test_backbone_hv3_authoritative_order_decision_survives_refresh);
  test_registry::AddTest(tests, "C249_Backbone_EdgeOrientationUsesChosenOrderDecision",
                         "Edge orientation is derived from the chosen endpoint order decision instead of legacy mirror state",
                         "Invariant", false, test_backbone_edge_orientation_uses_chosen_order_decision);
  test_registry::AddTest(tests, "C250_Backbone_BundleBranchLoweringIsPoleLocal",
                         "Bundle-like branch lowering is decided from each pole endpoint locally instead of relying on run propagation",
                         "Invariant", false, test_backbone_bundle_branch_lowering_stays_pole_local);
  test_registry::AddTest(tests, "C251_Backbone_CrossUnderpassSupportUsesSharedSideGroup",
                         "Cross underpass lowered pair reuses one side group and one support anchor instead of splitting left/right per endpoint",
                         "Invariant", false, test_backbone_cross_underpass_supports_share_one_side_group);
  test_registry::AddTest(tests, "C252_Backbone_RefreshKeepsLocalLowerAndGroupedSupport",
                         "Refresh keeps pole-local lowering decisions and grouped lowered-support identity instead of recomputing them per-port",
                         "Invariant", false, test_backbone_refresh_keeps_local_lower_and_grouped_support);
  test_registry::AddTest(tests, "C255_Backbone_GroupedSupportVisibleOnAllBundleLanes",
                         "Grouped lowered support remains visible on every participating HV3 lane instead of only a representative span",
                         "Invariant", false, test_backbone_grouped_support_membership_is_visible_on_all_bundle_lanes);
  test_registry::AddTest(tests, "C256_Backbone_UnrelatedGenerationDoesNotDowngradeLoweredBundle",
                         "Unrelated later generation does not cause existing lowered HV3 spans to fall back to point-like/radial semantics during refresh",
                         "Invariant", false, test_backbone_unrelated_generation_does_not_downgrade_existing_lowered_bundle_semantics);
  test_registry::AddTest(tests, "C253_Backbone_CornerSupportUsesConnectedLineBasis",
                         "Bundle-like lowered corner uses only connected lowered lines to choose a non-radial support basis and shared support identity",
                         "Invariant", false, test_backbone_corner_support_uses_connected_line_basis);
  test_registry::AddTest(tests, "C275_Backbone_LoweredBundleMidspanSupportUsesPairBasedOrientation",
                         "Bundle-like lowered midspan support uses the route-pair bisector instead of falling back to pole-local radial orientation",
                         "Invariant", false, test_backbone_lowered_bundle_midspan_support_uses_pair_based_orientation);
  test_registry::AddTest(tests, "C276_Backbone_PairAxisSharedButSignFlipsPerPole",
                         "Pair-based lowered support keeps one shared axis/basis while opposite poles may take opposite side signs",
                         "Invariant", false, test_backbone_pair_based_orientation_allows_opposite_signs_per_pole);
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
  test_registry::AddTest(tests, "C269_Backbone_CrossLikeSingleEdgeStaysMain",
                         "Single-edge DrawPath from a cross-like existing node stays Main instead of inheriting branch/cross lowering from old neighbors",
                         "Invariant", false, test_backbone_crosslike_single_edge_stays_main);
  test_registry::AddTest(tests, "C270_Backbone_ExplicitMiddleBentRouteStaysMainLike",
                         "Bent route through an explicit middle anchor stays main-like against an existing straight chain instead of falling to branch/cross classification",
                         "Invariant", false,
                         test_backbone_explicit_middle_bent_route_stays_corner_main_against_existing_chain);
  test_registry::AddTest(tests, "C271_Backbone_GroupedSupportVisualUsesSinglePlacement",
                         "Grouped lowered support visual cache uses one group placement with grouped hangers instead of per-endpoint support materialization",
                         "Invariant", false, test_backbone_grouped_support_visual_cache_uses_single_group_placement);
  test_registry::AddTest(tests, "C272_Backbone_BranchLowerRequiredHeightSurvivesPlacementAndRefresh",
                         "Bundle-like non-through lower_required propagates to grouped placement height and stays unchanged after refresh/recalc",
                         "Invariant", false, test_backbone_branch_lower_required_height_survives_to_grouped_placement);
  test_registry::AddTest(tests, "C273_Backbone_NonThroughHeightCollapsesToOneLowerStep",
                         "Bundle-like non-through uses one shared lower step while ThroughMain stays at template height",
                         "Invariant", false, test_backbone_bundle_non_through_height_collapses_to_two_states);
  test_registry::AddTest(tests, "C274_Backbone_CrossLikeReuseKeepsExistingStraightMainPair",
                         "Cross-like reuse keeps the existing straight main pair instead of promoting a new crossing path to ThroughMain",
                         "Invariant", false, test_backbone_crosslike_reuse_keeps_existing_straight_main_pair);
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


