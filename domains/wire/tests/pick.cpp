#include <string>
#include <vector>

#include "registry.hpp"
#include "helpers.hpp"
#include "city/wire/core_test_hook.hpp"

using namespace helpers;

bool make_pick_fixture(CoreState& state, city::wire::BundleKind kind, ObjectId* span_id,
                       ObjectId* node_a, ObjectId* node_b) {
  const auto fixture = make_backbone_fixture(state, {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}}, {kind});
  if (!fixture.ok || fixture.value.spans.empty() || fixture.value.nodes.size() != 2) return false;
  *span_id = fixture.value.spans.front();
  *node_a = fixture.value.nodes[0];
  *node_b = fixture.value.nodes[1];
  return true;
}

bool test_branch_pick_segment_near_endpoint_snaps_to_node() {
  CoreState state;
  ObjectId span_id{}, node_a{}, node_b{};
  if (!make_pick_fixture(state, city::wire::BundleKind::kLowVoltage, &span_id, &node_a, &node_b)) return false;

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_id = span_id;
  pick.hit_pos_world = {0.12, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = node_a;
  pick.segment_node_b_id = node_b;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {10.0, 0.0, 0.0};

  city::wire::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage)};
  resolve.snap_radius_world = 0.5;
  const CoreCounts before = snapshot_counts(state);
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok) {
    return false;
  }
  return resolved.value.resolution == city::wire::PickBranchResolutionKind::kNode &&
         resolved.value.resolved_node_id == node_a &&
         same_counts(before, snapshot_counts(state));
}

// Intent: Segment pick far from endpoints creates a Midair support node.
bool test_branch_pick_segment_midpoint_creates_midair_node() {
  CoreState state;
  ObjectId span_id{}, node_a{}, node_b{};
  if (!make_pick_fixture(state, city::wire::BundleKind::kLowVoltage, &span_id, &node_a, &node_b)) return false;

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_id = span_id;
  pick.hit_pos_world = {5.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = node_a;
  pick.segment_node_b_id = node_b;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {10.0, 0.0, 0.0};

  city::wire::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage)};
  resolve.snap_radius_world = 0.5;
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok) {
    return false;
  }
  if (resolved.value.resolution != city::wire::PickBranchResolutionKind::kMidair ||
      resolved.value.support_kind != city::wire::SupportKind::kMidair ||
      resolved.value.resolved_node_id == city::wire::kInvalidObjectId) {
    return false;
  }
  const auto& nodes = city::wire::CoreStateTestHook::pending_support_nodes(state);
  if (nodes.size() != 1) {
    return false;
  }
  return nodes.front().node_id == resolved.value.resolved_node_id &&
         nodes.front().support_kind == city::wire::SupportKind::kMidair;
}

// Intent: Dry-run branch pick should resolve Midair without mutating support-node state.
bool test_branch_pick_segment_midpoint_dryrun_keeps_state_unchanged() {
  CoreState state;
  ObjectId span_id{}, node_a{}, node_b{};
  if (!make_pick_fixture(state, city::wire::BundleKind::kLowVoltage, &span_id, &node_a, &node_b)) return false;

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_id = span_id;
  pick.hit_pos_world = {5.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = node_a;
  pick.segment_node_b_id = node_b;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {10.0, 0.0, 0.0};

  city::wire::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage)};
  resolve.snap_radius_world = 0.5;
  resolve.create_midair_node = false;
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok) {
    return false;
  }
  if (resolved.value.resolution != city::wire::PickBranchResolutionKind::kMidair ||
      resolved.value.support_kind != city::wire::SupportKind::kMidair ||
      resolved.value.resolved_node_id != city::wire::kInvalidObjectId) {
    return false;
  }
  return city::wire::CoreStateTestHook::pending_support_nodes(state).empty();
}

// Intent: HV template rule must reject midair branch picks in core.
bool test_branch_pick_hv_template_blocks_midair_branch() {
  CoreState state;
  ObjectId span_id{}, node_a{}, node_b{};
  if (!make_pick_fixture(state, city::wire::BundleKind::kHighVoltage, &span_id, &node_a, &node_b)) return false;

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_id = span_id;
  pick.hit_pos_world = {5.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = node_a;
  pick.segment_node_b_id = node_b;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {10.0, 0.0, 0.0};

  city::wire::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage)};
  resolve.snap_radius_world = 0.5;
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  return !resolved.ok && regex_contains(resolved.error, "midair branch");
}

// Intent: Path input can resolve a Midair point even when the selected template later refuses connection.
bool test_branch_pick_hv_template_allows_midair_when_policy_not_enforced() {
  CoreState state;
  ObjectId span_id{}, node_a{}, node_b{};
  if (!make_pick_fixture(state, city::wire::BundleKind::kHighVoltage, &span_id, &node_a, &node_b)) return false;

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_id = span_id;
  pick.hit_pos_world = {5.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = node_a;
  pick.segment_node_b_id = node_b;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {10.0, 0.0, 0.0};

  city::wire::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kHighVoltage)};
  resolve.snap_radius_world = 0.5;
  resolve.enforce_midair_template_policy = false;
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  return resolved.ok && resolved.value.resolution == city::wire::PickBranchResolutionKind::kMidair &&
         resolved.value.support_kind == city::wire::SupportKind::kMidair &&
         resolved.value.resolved_node_id != city::wire::kInvalidObjectId;
}

bool C817_Pick_StalePendingNodeReferenceRejectedAfterLoad() {
  CoreState source;
  ObjectId span_id{}, node_a{}, node_b{};
  if (!make_pick_fixture(source, city::wire::BundleKind::kLowVoltage, &span_id, &node_a, &node_b)) return false;

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_id = span_id;
  pick.hit_pos_world = {5.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = node_a;
  pick.segment_node_b_id = node_b;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {10.0, 0.0, 0.0};

  city::wire::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage)};
  const auto resolved = source.ResolveBranchPick(pick, resolve);
  std::string saved{};
  if (!resolved.ok || resolved.value.resolved_node_id == city::wire::kInvalidObjectId ||
      !source.SerializeAuthoritative(&saved).ok) {
    return false;
  }

  CoreState loaded;
  if (!loaded.DeserializeAuthoritative(saved).ok ||
      !city::wire::CoreStateTestHook::pending_support_nodes(loaded).empty()) {
    return false;
  }
  city::wire::BackboneSpec branch{};
  branch.path.polyline = {resolved.value.position, {5.0, 8.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = 1;
  add_backbone_bundle(branch, city::wire::BundleKind::kLowVoltage);
  city::wire::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const CoreCounts before = snapshot_counts(loaded);
  const auto generated = loaded.GenerateFromBackboneSpec(branch);
  return !generated.ok && regex_contains(generated.error, "unknown node reference") &&
         same_counts(before, snapshot_counts(loaded));
}

bool C818_Pick_PendingSupportNodesAreClearedAndConsumed() {
  CoreState state;
  ObjectId span_id{}, node_a{}, node_b{};
  if (!make_pick_fixture(state, city::wire::BundleKind::kLowVoltage, &span_id, &node_a, &node_b)) return false;

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_id = span_id;
  pick.hit_pos_world = {5.0, 0.0, 0.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = node_a;
  pick.segment_node_b_id = node_b;
  pick.segment_endpoint_a_world = {0.0, 0.0, 0.0};
  pick.segment_endpoint_b_world = {10.0, 0.0, 0.0};

  city::wire::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage)};
  const auto first = state.ResolveBranchPick(pick, resolve);
  if (!first.ok || city::wire::CoreStateTestHook::pending_support_nodes(state).size() != 1) {
    return false;
  }
  const auto cleared = state.ClearPendingSupportNodes();
  if (!cleared.ok || !cleared.value ||
      !city::wire::CoreStateTestHook::pending_support_nodes(state).empty()) {
    return false;
  }
  const auto second = state.ResolveBranchPick(pick, resolve);
  if (!second.ok || city::wire::CoreStateTestHook::pending_support_nodes(state).size() != 1) {
    return false;
  }
  city::wire::BackboneSpec branch{};
  branch.path.polyline = {second.value.position, {5.0, 8.0, 0.0}};
  branch.interval_m = 1000.0;
  branch.pole_type_id = 1;
  add_backbone_bundle(branch, city::wire::BundleKind::kLowVoltage);
  city::wire::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = second.value.support_kind;
  node.node_id = second.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto generated = state.GenerateFromBackboneSpec(branch);
  return generated.ok && city::wire::CoreStateTestHook::pending_support_nodes(state).empty();
}

namespace {

void register_pick_tests(test_registry::TestRegistry& tests) {
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
  test_registry::AddTest(tests, "C817_Pick_StalePendingNodeReferenceRejectedAfterLoad",
                         "stale pending node ids from a prior session are rejected before mutation", "Boundary", true,
                         C817_Pick_StalePendingNodeReferenceRejectedAfterLoad);
  test_registry::AddTest(tests, "C818_Pick_PendingSupportNodesAreClearedAndConsumed",
                         "pending support node drafts clear on cancel and are consumed after generation", "Invariant", false,
                         C818_Pick_PendingSupportNodesAreClearedAndConsumed);
}

WIRE_REGISTER_TEST_SUITE(register_pick_tests);

} // namespace
