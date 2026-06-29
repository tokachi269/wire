#include <vector>

#include "registry.hpp"
#include "helpers.hpp"
#include "wire/core/core_test_hook.hpp"

using namespace helpers;

bool test_branch_pick_segment_near_endpoint_snaps_to_node() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b_tf{};
  b_tf.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  const auto connection = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, options);
  if (!connection.ok) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = connection.value.span_id;
  pick.hit_pos_world = {0.12, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = pole_a;
  pick.segment_node_b_id = pole_b;
  pick.segment_endpoint_a_world = a_tf.position;
  pick.segment_endpoint_b_world = b_tf.position;

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage};
  resolve.snap_radius_world = 0.5;
  const CoreCounts before = snapshot_counts(state);
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok) {
    return false;
  }
  return resolved.value.resolution == wire::core::PickBranchResolutionKind::kNode &&
         resolved.value.resolved_node_id == pole_a && resolved.value.snapped_from_segment_endpoint &&
         same_counts(before, snapshot_counts(state));
}

// Intent: Segment pick far from endpoints creates a Midair support node.
bool test_branch_pick_segment_midpoint_creates_midair_node() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b_tf{};
  b_tf.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  const auto connection = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, options);
  if (!connection.ok) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = connection.value.span_id;
  pick.hit_pos_world = {5.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = pole_a;
  pick.segment_node_b_id = pole_b;
  pick.segment_endpoint_a_world = a_tf.position;
  pick.segment_endpoint_b_world = b_tf.position;

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage};
  resolve.snap_radius_world = 0.5;
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok) {
    return false;
  }
  if (resolved.value.resolution != wire::core::PickBranchResolutionKind::kMidair ||
      resolved.value.support_kind != wire::core::SupportKind::kMidair ||
      resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto& nodes = wire::core::CoreStateTestHook::pending_support_nodes(state);
  if (nodes.size() != 1) {
    return false;
  }
  return nodes.front().node_id == resolved.value.resolved_node_id &&
         nodes.front().support_kind == wire::core::SupportKind::kMidair;
}

// Intent: Dry-run branch pick should resolve Midair without mutating support-node state.
bool test_branch_pick_segment_midpoint_dryrun_keeps_state_unchanged() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b_tf{};
  b_tf.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  const auto connection = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, options);
  if (!connection.ok) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = connection.value.span_id;
  pick.hit_pos_world = {5.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = pole_a;
  pick.segment_node_b_id = pole_b;
  pick.segment_endpoint_a_world = a_tf.position;
  pick.segment_endpoint_b_world = b_tf.position;

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kLowVoltage};
  resolve.snap_radius_world = 0.5;
  resolve.create_midair_node = false;
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok) {
    return false;
  }
  if (resolved.value.resolution != wire::core::PickBranchResolutionKind::kMidair ||
      resolved.value.support_kind != wire::core::SupportKind::kMidair ||
      resolved.value.resolved_node_id != wire::core::kInvalidObjectId) {
    return false;
  }
  return wire::core::CoreStateTestHook::pending_support_nodes(state).empty();
}

// Intent: HV template rule must reject midair branch picks in core.
bool test_branch_pick_hv_template_blocks_midair_branch() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b_tf{};
  b_tf.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  const auto connection = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kHighVoltage, options);
  if (!connection.ok) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = connection.value.span_id;
  pick.hit_pos_world = {5.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = pole_a;
  pick.segment_node_b_id = pole_b;
  pick.segment_endpoint_a_world = a_tf.position;
  pick.segment_endpoint_b_world = b_tf.position;

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kHighVoltage};
  resolve.snap_radius_world = 0.5;
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  return !resolved.ok && regex_contains(resolved.error, "midair branch");
}

// Intent: Path input can resolve a Midair point even when the selected template later refuses connection.
bool test_branch_pick_hv_template_allows_midair_when_policy_not_enforced() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b_tf{};
  b_tf.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kHighVoltage;
  const auto connection = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kHighVoltage, options);
  if (!connection.ok) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = connection.value.span_id;
  pick.hit_pos_world = {5.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = pole_a;
  pick.segment_node_b_id = pole_b;
  pick.segment_endpoint_a_world = a_tf.position;
  pick.segment_endpoint_b_world = b_tf.position;

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::BundleKind::kHighVoltage};
  resolve.snap_radius_world = 0.5;
  resolve.enforce_midair_template_policy = false;
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  return resolved.ok && resolved.value.resolution == wire::core::PickBranchResolutionKind::kMidair &&
         resolved.value.support_kind == wire::core::SupportKind::kMidair &&
         resolved.value.resolved_node_id != wire::core::kInvalidObjectId;
}

namespace {

void register_pick_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C104_Pick_SegmentEndpointSnap",
                         "Segment pick near endpoint snaps to an existing node", "Invariant", false,
                         test_branch_pick_segment_near_endpoint_snaps_to_node);
  test_registry::AddTest(tests, "C105_Pick_SegmentMidairCreate",
                         "Segment pick away from endpoints creates Midair support node", "Invariant", false,
                         test_branch_pick_segment_midpoint_creates_midair_node);
  test_registry::AddTest(tests, "C106_Pick_HVBlocksMidairBranch",
                         "HV template blocks midair branch pick in core", "Exact", true,
                         test_branch_pick_hv_template_blocks_midair_branch);
  test_registry::AddTest(tests, "C107_Pick_MidairDryRunNoMutation",
                         "Dry-run segment pick resolves Midair without mutating state", "Invariant", false,
                         test_branch_pick_segment_midpoint_dryrun_keeps_state_unchanged);
  test_registry::AddTest(tests, "C117_Pick_MidairPolicyBypassForPathInput",
                         "Path-input pick can resolve Midair without enforcing template branch policy", "Invariant", false,
                         test_branch_pick_hv_template_allows_midair_when_policy_not_enforced);
}

WIRE_REGISTER_TEST_SUITE(register_pick_tests);

} // namespace




