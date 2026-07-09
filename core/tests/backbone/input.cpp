#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace helpers;

namespace backbone_tests {

bool C579_backbone_polyline_avoid_detour_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  req.constraints.avoid_points.push_back({6.0, 0.0, 0.0});
  req.constraints.avoid_radius_m = 1.0;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 4 || out.value.generated_span_ids.empty()) {
    return false;
  }
  const auto detour = std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                                   [](const wire::core::SavedBackboneNode& node) {
                                     return almost_equal(node.position.x, 6.0, 1e-9) &&
                                            std::abs(node.position.y) > 1.0;
                                   });
  return detour != state.view().backbone().nodes.end();
}

bool C580_backbone_interval_avoid_combination_orders_inserted_points() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.interval_m = 4.0;
  req.constraints.avoid_points.push_back({6.0, 0.0, 0.0});
  req.constraints.avoid_radius_m = 1.0;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 5 || state.view().backbone().nodes.size() != 5 ||
      state.view().backbone().edges.size() != 4) {
    return false;
  }
  std::vector<double> xs{};
  xs.reserve(state.view().backbone().nodes.size());
  bool saw_detour = false;
  for (const wire::core::SavedBackboneNode& node : state.view().backbone().nodes) {
    xs.push_back(node.position.x);
    saw_detour = saw_detour || (almost_equal(node.position.x, 6.0, 1e-9) && std::abs(node.position.y) > 1.0);
  }
  std::sort(xs.begin(), xs.end());
  const std::vector<double> expected = {0.0, 4.0, 6.0, 8.0, 12.0};
  if (!saw_detour || xs.size() != expected.size()) {
    return false;
  }
  for (std::size_t i = 0; i < expected.size(); ++i) {
    if (!almost_equal(xs[i], expected[i], 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C582_backbone_multiple_avoid_points_on_one_segment_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.constraints.avoid_points.push_back({4.0, 0.0, 0.0});
  req.constraints.avoid_points.push_back({8.0, 0.0, 0.0});
  req.constraints.avoid_radius_m = 1.0;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 4 || state.view().backbone().nodes.size() != 4 ||
      state.view().backbone().edges.size() != 3) {
    return false;
  }
  bool saw_first_detour = false;
  bool saw_second_detour = false;
  for (const wire::core::SavedBackboneNode& node : state.view().backbone().nodes) {
    saw_first_detour = saw_first_detour || (almost_equal(node.position.x, 4.0, 1e-9) && std::abs(node.position.y) > 1.0);
    saw_second_detour =
        saw_second_detour || (almost_equal(node.position.x, 8.0, 1e-9) && std::abs(node.position.y) > 1.0);
  }
  return saw_first_detour && saw_second_detour;
}

bool C583_backbone_avoid_points_on_multiple_segments_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  req.constraints.avoid_points.push_back({6.0, 0.0, 0.0});
  req.constraints.avoid_points.push_back({12.0, 4.0, 0.0});
  req.constraints.avoid_radius_m = 1.0;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 5 || state.view().backbone().nodes.size() != 5 ||
      state.view().backbone().edges.size() != 4) {
    return false;
  }
  bool saw_first_detour = false;
  bool saw_second_detour = false;
  for (const wire::core::SavedBackboneNode& node : state.view().backbone().nodes) {
    saw_first_detour = saw_first_detour || (almost_equal(node.position.x, 6.0, 1e-9) && std::abs(node.position.y) > 1.0);
    saw_second_detour =
        saw_second_detour || (almost_equal(node.position.y, 4.0, 1e-9) && std::abs(node.position.x - 12.0) > 1.0);
  }
  return saw_first_detour && saw_second_detour;
}

bool C584_backbone_ownerless_interval_inserts_ownerless_nodes() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.pole_type_id = wire::core::kInvalidPoleTypeId;
  req.path.polyline = {{0.0, 0.0, 8.0}, {12.0, 0.0, 8.0}};
  req.interval_m = 6.0;
  wire::core::BackboneInputSpec::NodeSpec a{};
  a.point_index = 0;
  a.support_kind = wire::core::SupportKind::kMidair;
  wire::core::BackboneInputSpec::NodeSpec b{};
  b.point_index = 1;
  b.support_kind = wire::core::SupportKind::kMidair;
  req.path.node_specs = {a, b};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || !out.value.generated_pole_ids.empty() || state.view().backbone().nodes.size() != 3 ||
      state.view().backbone().edges.size() != 2) {
    return false;
  }
  for (const wire::core::SavedBackboneNode& node : state.view().backbone().nodes) {
    if (node.pole_id != wire::core::kInvalidObjectId || node.support_kind != wire::core::SupportKind::kMidair) {
      return false;
    }
  }
  return true;
}

bool C585_backbone_duplicate_avoid_points_are_coalesced() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.constraints.avoid_points.push_back({6.0, 0.0, 0.0});
  req.constraints.avoid_points.push_back({6.0, 0.0, 0.0});
  req.constraints.avoid_radius_m = 1.0;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 3 || state.view().backbone().nodes.size() != 3 ||
      state.view().backbone().edges.size() != 2) {
    return false;
  }
  std::size_t detour_count = 0;
  for (const wire::core::SavedBackboneNode& node : state.view().backbone().nodes) {
    if (almost_equal(node.position.x, 6.0, 1e-9) && std::abs(node.position.y) > 1.0) {
      ++detour_count;
    }
  }
  return detour_count == 1;
}

bool C594_backbone_avoid_point_at_route_endpoint_is_noop() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.constraints.avoid_points.push_back({0.0, 0.0, 0.0});
  req.constraints.avoid_radius_m = 1.0;
  const auto out = state.GenerateFromBackboneSpec(req);
  return out.ok && out.value.generated_pole_ids.size() == 2 && state.view().backbone().nodes.size() == 2 &&
         state.view().backbone().edges.size() == 1;
}

bool C586_backbone_avoid_detour_replaces_interval_at_same_t() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.interval_m = 6.0;
  req.constraints.avoid_points.push_back({6.0, 0.0, 0.0});
  req.constraints.avoid_radius_m = 1.0;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 3 || state.view().backbone().nodes.size() != 3 ||
      state.view().backbone().edges.size() != 2) {
    return false;
  }
  bool saw_detour = false;
  for (const wire::core::SavedBackboneNode& node : state.view().backbone().nodes) {
    if (almost_equal(node.position.x, 6.0, 1e-9) && std::abs(node.position.y) > 1.0) {
      saw_detour = true;
    }
    if (almost_equal(node.position.x, 6.0, 1e-9) && almost_equal(node.position.y, 0.0, 1e-9)) {
      return false;
    }
  }
  return saw_detour;
}

bool C587_backbone_create_midair_node_without_selected_bundle_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) return false;
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = base_out.value.generated_span_ids.front();
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  pick.has_segment_endpoints = false;
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids.clear();
  resolve.create_midair_node = true;
  resolve.create_midair_node_set = true;
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolution != wire::core::PickBranchResolutionKind::kMidair ||
      resolved.value.support_kind != wire::core::SupportKind::kMidair ||
      resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(branch);
  return out.ok && !out.value.generated_span_ids.empty();
}

bool C588_backbone_corner_avoid_detour_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  req.constraints.avoid_radius_m = 2.0;
  req.constraints.avoid_points = {{12.0, 0.0, 0.0}};
  const auto out = state.GenerateFromBackboneSpec(req);
  return out.ok && !out.value.generated_span_ids.empty();
}

bool C589_backbone_selected_bundle_policy_blocks_unselected_bundle() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  add_backbone_bundle(base, wire::core::BundleKind::kCommunication);
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) return false;
  const wire::core::ObjectId source_span =
      span_for_bundle(state, base_out.value.generated_span_ids, wire::core::BundleKind::kCommunication);
  if (source_span == wire::core::kInvalidObjectId) return false;
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) return false;
  wire::core::BackboneSpec branch = line_req(state);
  branch.bundles.clear();
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.bundle_ids.size() != 1 || out.value.generated_span_ids.empty()) return false;
  const auto* bundle = state.view().bundles().find(out.value.bundle_ids.front());
  return bundle != nullptr && bundle->bundle_template_id == wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication);
}

bool C590_backbone_inactive_pass_through_bundle_rejected_before_noop() {
  wire::core::CoreState state;
  const auto base_out = state.GenerateFromBackboneSpec(line_req(state));
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) return false;
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = base_out.value.generated_span_ids.front();
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) return false;
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 0;
  mode.bundle_template_id = branch.bundles.front().bundle_template_id;
  mode.mode = wire::core::BundleNodeMode::kPassThrough;
  branch.node_bundle_modes = {mode};
  const auto out = state.GenerateFromBackboneSpec(branch);
  return !out.ok && contains_text(out.error, "inactive");
}

bool C591_backbone_saved_selected_midair_continuation_uses_request_bundles() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  add_backbone_bundle(base, wire::core::BundleKind::kCommunication);
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId source_span =
      span_for_bundle(state, base_out.value.generated_span_ids, wire::core::BundleKind::kCommunication);
  if (source_span == wire::core::kInvalidObjectId) {
    return false;
  }
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.bundles.clear();
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec pending{};
  pending.point_index = 0;
  pending.support_kind = resolved.value.support_kind;
  pending.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {pending};
  const auto first_branch = state.GenerateFromBackboneSpec(branch);
  if (!first_branch.ok || first_branch.value.bundle_ids.size() != 1) {
    return false;
  }
  const auto* first_bundle = state.view().bundles().find(first_branch.value.bundle_ids.front());
  if (first_bundle == nullptr || first_bundle->bundle_template_id != wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)) {
    return false;
  }
  const auto saved_midair = std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                                         [](const wire::core::SavedBackboneNode& node) {
                                           return node.pole_id == wire::core::kInvalidObjectId &&
                                                  node.support_kind == wire::core::SupportKind::kMidair &&
                                                  node.has_source_edge;
                                         });
  if (saved_midair == state.view().backbone().nodes.end()) {
    return false;
  }
  wire::core::BackboneSpec second = branch;
  second.path.polyline = {saved_midair->position, {6.0, 16.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec existing{};
  existing.point_index = 0;
  existing.support_kind = wire::core::SupportKind::kMidair;
  existing.node_id = saved_midair->node_id;
  second.path.node_specs = {existing};
  const auto out = state.GenerateFromBackboneSpec(second);
  if (!out.ok || out.value.bundle_ids.size() != 2 || out.value.generated_span_ids.empty()) {
    return false;
  }
  std::unordered_set<wire::core::BundleTemplateId> generated{};
  for (wire::core::ObjectId bundle_id : out.value.bundle_ids) {
    const auto* bundle = state.view().bundles().find(bundle_id);
    if (bundle != nullptr) {
      generated.insert(bundle->bundle_template_id);
    }
  }
  return generated.contains(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage)) &&
         generated.contains(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication));
}

bool C592_backbone_saved_selected_midair_reverse_continuation_uses_request_bundles() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  add_backbone_bundle(base, wire::core::BundleKind::kCommunication);
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId source_span =
      span_for_bundle(state, base_out.value.generated_span_ids, wire::core::BundleKind::kCommunication);
  if (source_span == wire::core::kInvalidObjectId) {
    return false;
  }
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.bundles.clear();
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec pending{};
  pending.point_index = 0;
  pending.support_kind = resolved.value.support_kind;
  pending.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {pending};
  const auto first_branch = state.GenerateFromBackboneSpec(branch);
  if (!first_branch.ok) {
    return false;
  }
  const auto saved_midair = std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                                         [](const wire::core::SavedBackboneNode& node) {
                                           return node.pole_id == wire::core::kInvalidObjectId &&
                                                  node.support_kind == wire::core::SupportKind::kMidair &&
                                                  node.has_source_edge;
                                         });
  if (saved_midair == state.view().backbone().nodes.end()) {
    return false;
  }
  wire::core::BackboneSpec second = branch;
  second.path.polyline = {{6.0, 16.0, 0.0}, saved_midair->position};
  wire::core::BackboneInputSpec::NodeSpec existing{};
  existing.point_index = 1;
  existing.support_kind = wire::core::SupportKind::kMidair;
  existing.node_id = saved_midair->node_id;
  second.path.node_specs = {existing};
  const auto out = state.GenerateFromBackboneSpec(second);
  if (!out.ok || out.value.bundle_ids.size() != 2 || out.value.generated_span_ids.empty()) {
    return false;
  }
  std::unordered_set<wire::core::BundleTemplateId> generated{};
  for (wire::core::ObjectId bundle_id : out.value.bundle_ids) {
    const auto* bundle = state.view().bundles().find(bundle_id);
    if (bundle != nullptr) {
      generated.insert(bundle->bundle_template_id);
    }
  }
  return generated.contains(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage)) &&
         generated.contains(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication));
}

bool C593_backbone_saved_selected_midair_allows_request_pass_through() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  add_backbone_bundle(base, wire::core::BundleKind::kCommunication);
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId source_span_id =
      span_for_bundle(state, base_out.value.generated_span_ids, wire::core::BundleKind::kCommunication);
  const auto* source_span = state.view().spans().find(source_span_id);
  if (source_span == nullptr) {
    return false;
  }
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span->id;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.bundles.clear();
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec pending{};
  pending.point_index = 0;
  pending.support_kind = resolved.value.support_kind;
  pending.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {pending};
  const auto first_branch = state.GenerateFromBackboneSpec(branch);
  if (!first_branch.ok) {
    return false;
  }
  const auto saved_midair = std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                                         [](const wire::core::SavedBackboneNode& node) {
                                           return node.pole_id == wire::core::kInvalidObjectId &&
                                                  node.support_kind == wire::core::SupportKind::kMidair &&
                                                  node.has_source_edge;
                                         });
  if (saved_midair == state.view().backbone().nodes.end()) {
    return false;
  }
  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {saved_midair->position, {6.0, 16.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec existing{};
  existing.point_index = 0;
  existing.support_kind = wire::core::SupportKind::kMidair;
  existing.node_id = saved_midair->node_id;
  second.path.node_specs = {existing};
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 0;
  mode.bundle_template_id = second.bundles.front().bundle_template_id;
  mode.mode = wire::core::BundleNodeMode::kPassThrough;
  second.node_bundle_modes = {mode};
  const auto out = state.GenerateFromBackboneSpec(second);
  return out.ok && !out.value.generated_span_ids.empty();
}

bool C595_backbone_avoid_point_at_explicit_existing_support_is_noop() {
  wire::core::CoreState state;
  const auto base = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!base.ok || base.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId middle_id = base.value.generated_pole_ids[1];
  const auto* middle = state.view().poles().find(middle_id);
  if (middle == nullptr) {
    return false;
  }

  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {{12.0, -8.0, 0.0}, middle->world_transform.position, {12.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec existing{};
  existing.point_index = 1;
  existing.support_kind = wire::core::SupportKind::kPole;
  existing.node_id = middle_id;
  req.path.node_specs = {existing};
  req.constraints.avoid_points = {middle->world_transform.position};
  req.constraints.avoid_radius_m = 3.0;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 2 || out.value.generated_span_ids.empty()) {
    return false;
  }
  const auto* same_middle = state.view().poles().find(middle_id);
  return same_middle != nullptr && almost_equal(same_middle->world_transform.position.x, 12.0, 1e-9) &&
         almost_equal(same_middle->world_transform.position.y, 0.0, 1e-9);
}

bool C596_backbone_avoid_point_at_explicit_new_support_is_noop() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  wire::core::BackboneInputSpec::NodeSpec middle{};
  middle.point_index = 1;
  middle.support_kind = wire::core::SupportKind::kPole;
  middle.node_id = wire::core::kInvalidObjectId;
  req.path.node_specs = {middle};
  req.constraints.avoid_points = {req.path.polyline[1]};
  req.constraints.avoid_radius_m = 3.0;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 3 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, req) * 2)) {
    return false;
  }
  const auto* generated_middle = state.view().poles().find(out.value.generated_pole_ids[1]);
  return generated_middle != nullptr && almost_equal(generated_middle->world_transform.position.x, 12.0, 1e-9) &&
         almost_equal(generated_middle->world_transform.position.y, 0.0, 1e-9);
}

bool C581_backbone_inactive_bundle_missing_band_is_ignored() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  auto it = state.view().pole_types().find(base.pole_type_id);
  if (it == state.view().pole_types().end()) {
    return false;
  }
  wire::core::PoleTypeDefinition type = it->second;
  type.port_bands.erase(std::remove_if(type.port_bands.begin(), type.port_bands.end(),
                                       [](const wire::core::PortPlacementBand& band) {
                                         return band.category == wire::core::ConnectionCategory::kHighVoltage &&
                                                band.layer == 2;
                                       }),
                        type.port_bands.end());
  if (!state.UpdatePoleTypeDefinition(type).ok) {
    return false;
  }
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = base_out.value.generated_span_ids.front();
  pick.hit_pos_world = {6.0, 0.0, 4.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage), wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  branch.bundles.clear();
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};

  const auto out = state.GenerateFromBackboneSpec(branch);
  return out.ok &&
         out.value.generated_span_ids.size() ==
             static_cast<std::size_t>(bundle_count(state, wire::core::BundleKind::kLowVoltage));
}

bool C543_backbone_new_route_interior_pass_through_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = pass_poly3_req(state);
  const int count = req_bundle_count(state, req);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 3 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(count * 2)) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.nodes.size() != 3 || graph.edges.size() != 2 || graph.edge_bundles.size() != 2) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr ||
        state.find_span_visual_cache(span_id) == nullptr || state.view().find_span_render_cache(span_id) == nullptr) {
      return false;
    }
    if (span_has_lowered_endpoint(state, span_id)) {
      return false;
    }
  }
  return true;
}

bool C544_backbone_pole_placement_pins_generated_poles() {
  wire::core::CoreState endpoints_state;
  wire::core::BackboneSpec endpoints = poly3_req(endpoints_state);
  endpoints.pole_placement.pin_endpoints = true;
  const auto endpoints_out = endpoints_state.GenerateFromBackboneSpec(endpoints);
  if (!endpoints_out.ok || endpoints_out.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const auto* a = endpoints_state.view().poles().find(endpoints_out.value.generated_pole_ids[0]);
  const auto* b = endpoints_state.view().poles().find(endpoints_out.value.generated_pole_ids[1]);
  const auto* c = endpoints_state.view().poles().find(endpoints_out.value.generated_pole_ids[2]);
  if (a == nullptr || b == nullptr || c == nullptr || a->placement_mode != wire::core::PlacementMode::kManual ||
      b->placement_mode != wire::core::PlacementMode::kAuto || c->placement_mode != wire::core::PlacementMode::kManual) {
    return false;
  }

  wire::core::CoreState vertices_state;
  wire::core::BackboneSpec vertices = poly3_req(vertices_state);
  vertices.pole_placement.pin_vertices = true;
  const auto vertices_out = vertices_state.GenerateFromBackboneSpec(vertices);
  if (!vertices_out.ok || vertices_out.value.generated_pole_ids.size() != 3) {
    return false;
  }
  for (wire::core::ObjectId pole_id : vertices_out.value.generated_pole_ids) {
    const auto* pole = vertices_state.view().poles().find(pole_id);
    if (pole == nullptr || pole->placement_mode != wire::core::PlacementMode::kManual || !pole->user_edited) {
      return false;
    }
  }

  wire::core::CoreState existing_state;
  const auto base = existing_state.GenerateFromBackboneSpec(poly3_req(existing_state));
  if (!base.ok || base.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId existing = base.value.generated_pole_ids[1];
  const auto* before = existing_state.view().poles().find(existing);
  if (before == nullptr || before->placement_mode != wire::core::PlacementMode::kAuto) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(existing_state);
  branch.path.polyline = {before->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, existing)};
  branch.pole_placement.pin_endpoints = true;
  const auto branch_out = existing_state.GenerateFromBackboneSpec(branch);
  const auto* after = existing_state.view().poles().find(existing);
  return branch_out.ok && branch_out.value.generated_pole_ids.size() == 1 && after != nullptr &&
         after->placement_mode == wire::core::PlacementMode::kAuto;
}

bool C545_backbone_interval_generates_intermediate_poles() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.interval_m = 4.0;
  const int count = req_bundle_count(state, req);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 4 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(count * 3)) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.nodes.size() != 4 || graph.edges.size() != 3 || graph.edge_bundles.size() != 3) {
    return false;
  }
  for (std::size_t i = 0; i < out.value.generated_pole_ids.size(); ++i) {
    const auto* pole = state.view().poles().find(out.value.generated_pole_ids[i]);
    if (pole == nullptr || !almost_equal(pole->world_transform.position.x, static_cast<double>(i * 4), 1e-9) ||
        !almost_equal(pole->world_transform.position.y, 0.0, 1e-9)) {
      return false;
    }
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (!state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr ||
        state.find_span_visual_cache(span_id) == nullptr || state.view().find_span_render_cache(span_id) == nullptr) {
      return false;
    }
  }

  wire::core::CoreState pin_state;
  wire::core::BackboneSpec pinned = line_req(pin_state);
  pinned.interval_m = 4.0;
  pinned.pole_placement.pin_vertices = true;
  const auto pinned_out = pin_state.GenerateFromBackboneSpec(pinned);
  if (!pinned_out.ok || pinned_out.value.generated_pole_ids.size() != 4) {
    return false;
  }
  for (std::size_t i = 0; i < pinned_out.value.generated_pole_ids.size(); ++i) {
    const auto* pole = pin_state.view().poles().find(pinned_out.value.generated_pole_ids[i]);
    const wire::core::PlacementMode expected =
        (i == 0 || i + 1 == pinned_out.value.generated_pole_ids.size()) ? wire::core::PlacementMode::kManual
                                                                        : wire::core::PlacementMode::kAuto;
    if (pole == nullptr || pole->placement_mode != expected) {
      return false;
    }
  }
  return true;
}

bool C546_backbone_explicit_new_pole_node_spec_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 1;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = wire::core::kInvalidObjectId;
  req.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 3 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, req) * 2)) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.nodes.size() != 3 || graph.edges.size() != 2 || graph.edge_bundles.size() != 2) {
    return false;
  }
  const auto* middle = state.view().poles().find(out.value.generated_pole_ids[1]);
  return middle != nullptr && almost_equal(middle->world_transform.position.x, 12.0, 1e-9) &&
         almost_equal(middle->world_transform.position.y, 0.0, 1e-9) &&
         C398_backbone_rejects_missing_existing_pole();
}

bool C547_backbone_fixed_bundle_exact_count_is_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  if (req.bundles.empty()) {
    return false;
  }
  const auto tmpl_it = state.view().bundle_templates().find(req.bundles.front().bundle_template_id);
  if (tmpl_it == state.view().bundle_templates().end() ||
      tmpl_it->second.count_rule != wire::core::BundleCountRuleKind::kFixed || tmpl_it->second.fixed_count <= 0) {
    return false;
  }
  req.bundles.front().count = tmpl_it->second.fixed_count;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.size() != static_cast<std::size_t>(tmpl_it->second.fixed_count)) {
    return false;
  }

  wire::core::CoreState rejected;
  wire::core::BackboneSpec bad = line_req(rejected);
  bad.bundles.front().count = tmpl_it->second.fixed_count + 1;
  const std::size_t pole_count = rejected.view().poles().size();
  const std::size_t span_count = rejected.view().spans().size();
  const auto bad_out = rejected.GenerateFromBackboneSpec(bad);
  return !bad_out.ok && contains_text(bad_out.error, "unsupported") && rejected.view().poles().size() == pole_count &&
         rejected.view().spans().size() == span_count;
}

bool C548_backbone_avoid_radius_without_points_is_noop() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.constraints.avoid_radius_m = 3.0;
  const auto out = state.GenerateFromBackboneSpec(req);
  return out.ok && out.value.generated_pole_ids.size() == 2 &&
         out.value.generated_span_ids.size() == static_cast<std::size_t>(req_bundle_count(state, req));
}

bool C549_backbone_range_bundle_explicit_count_is_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.bundles.clear();
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 3);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.bundle_ids.size() != 1 || out.value.generated_span_ids.size() != 3) {
    return false;
  }
  const auto* bundle = state.view().bundles().find(out.value.bundle_ids.front());
  if (bundle == nullptr || bundle->conductor_count != 3) {
    return false;
  }

  wire::core::CoreState rejected;
  wire::core::BackboneSpec bad = line_req(rejected);
  bad.bundles.clear();
  add_backbone_bundle(bad, wire::core::BundleKind::kCommunication, wire::core::SpanLayer::kUnknown, 99);
  const std::size_t pole_count = rejected.view().poles().size();
  const std::size_t span_count = rejected.view().spans().size();
  const auto bad_out = rejected.GenerateFromBackboneSpec(bad);
  return !bad_out.ok && contains_text(bad_out.error, "unsupported") && rejected.view().poles().size() == pole_count &&
         rejected.view().spans().size() == span_count;
}

bool C550_backbone_generated_pole_uses_tangent_hint_yaw() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 1;
  node.support_kind = wire::core::SupportKind::kPole;
  node.node_id = wire::core::kInvalidObjectId;
  node.has_tangent_hint = true;
  node.tangent_hint = {1.0, 0.0, 0.0};
  req.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const auto* middle = state.view().poles().find(out.value.generated_pole_ids[1]);
  if (middle == nullptr || !almost_equal(middle->world_transform.rotation_euler_deg.z, 0.0, 1e-9)) {
    return false;
  }

  wire::core::CoreState rejected;
  wire::core::BackboneSpec bad = poly3_req(rejected);
  node.tangent_hint = {0.0, 0.0, 0.0};
  bad.path.node_specs = {node};
  const std::size_t pole_count = rejected.view().poles().size();
  const std::size_t span_count = rejected.view().spans().size();
  const auto bad_out = rejected.GenerateFromBackboneSpec(bad);
  return !bad_out.ok && contains_text(bad_out.error, "unsupported") && rejected.view().poles().size() == pole_count &&
         rejected.view().spans().size() == span_count;
}

bool C551_backbone_missing_pole_type_resolves_from_bundle_templates() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.pole_type_id = wire::core::kInvalidPoleTypeId;
  req.bundles.clear();
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  const auto tmpl_it = state.view().bundle_templates().find(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication));
  if (tmpl_it == state.view().bundle_templates().end() ||
      tmpl_it->second.related_pole_type_id == wire::core::kInvalidPoleTypeId) {
    return false;
  }
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 2 || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId pole_id : out.value.generated_pole_ids) {
    const auto* pole = state.view().poles().find(pole_id);
    if (pole == nullptr || pole->pole_type_id != tmpl_it->second.related_pole_type_id) {
      return false;
    }
  }

  wire::core::CoreState rejected;
  wire::core::BackboneSpec bad = line_req(rejected);
  bad.pole_type_id = wire::core::kInvalidPoleTypeId;
  add_backbone_bundle(bad, wire::core::BundleKind::kCommunication);
  const std::size_t pole_count = rejected.view().poles().size();
  const std::size_t span_count = rejected.view().spans().size();
  const auto bad_out = rejected.GenerateFromBackboneSpec(bad);
  return !bad_out.ok && contains_text(bad_out.error, "unsupported") && rejected.view().poles().size() == pole_count &&
         rejected.view().spans().size() == span_count;
}

bool C552_backbone_zero_radius_avoid_points_are_noop() {
  wire::core::CoreState plain;
  wire::core::BackboneSpec base = line_req(plain);
  const auto base_out = plain.GenerateFromBackboneSpec(base);
  if (!base_out.ok) {
    return false;
  }
  const std::vector<wire::core::Vec3d> base_samples = span_curve_points(plain, base_out.value.generated_span_ids);
  if (base_samples.empty()) {
    return false;
  }

  wire::core::CoreState with_avoid;
  wire::core::BackboneSpec req = line_req(with_avoid);
  req.constraints.avoid_points.push_back({6.0, 0.0, 0.0});
  req.constraints.avoid_radius_m = 0.0;
  const auto out = with_avoid.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.size() != base_out.value.generated_span_ids.size()) {
    return false;
  }
  const std::vector<wire::core::Vec3d> samples = span_curve_points(with_avoid, out.value.generated_span_ids);
  if (samples.size() != base_samples.size()) {
    return false;
  }
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (!almost_equal(samples[i].x, base_samples[i].x, 1e-9) ||
        !almost_equal(samples[i].y, base_samples[i].y, 1e-9) ||
        !almost_equal(samples[i].z, base_samples[i].z, 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C553_backbone_new_midair_route_point_is_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 8.0}, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 1;
  node.support_kind = wire::core::SupportKind::kMidair;
  node.node_id = wire::core::kInvalidObjectId;
  req.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 2 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, req) * 2)) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.nodes.size() != 3 || graph.edges.size() != 2 || graph.edge_bundles.size() != 2) {
    return false;
  }
  const auto midair_it = std::find_if(graph.nodes.begin(), graph.nodes.end(), [](const wire::core::SavedBackboneNode& n) {
    return n.pole_id == wire::core::kInvalidObjectId;
  });
  if (midair_it == graph.nodes.end() || !almost_equal(midair_it->position.z, 8.0, 1e-9)) {
    return false;
  }
  bool saw_midair_port = false;
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr || !state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr) {
      return false;
    }
    saw_midair_port = saw_midair_port || (a->owner_pole_id == wire::core::kInvalidObjectId &&
                                          almost_equal(a->world_position.z, 8.0, 1e-9)) ||
                      (b->owner_pole_id == wire::core::kInvalidObjectId &&
                       almost_equal(b->world_position.z, 8.0, 1e-9));
  }
  return saw_midair_port && C397_backbone_rejects_missing_saved_midair_node_spec();
}

bool C554_backbone_existing_midair_route_point_is_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = poly3_req(state);
  first.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 8.0}, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.node_specs = {midair};
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok) {
    return false;
  }
  const wire::core::SavedBackboneGraph& before_graph = state.view().backbone();
  const auto saved_midair = std::find_if(before_graph.nodes.begin(), before_graph.nodes.end(),
                                         [](const wire::core::SavedBackboneNode& n) {
                                           return n.pole_id == wire::core::kInvalidObjectId;
                                         });
  if (saved_midair == before_graph.nodes.end()) {
    return false;
  }
  const wire::core::ObjectId saved_midair_id = saved_midair->node_id;
  const std::size_t pole_count = state.view().poles().size();

  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {saved_midair->position, {12.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec existing{};
  existing.point_index = 0;
  existing.support_kind = wire::core::SupportKind::kMidair;
  existing.node_id = saved_midair_id;
  second.path.node_specs = {existing};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok || second_out.value.generated_pole_ids.size() != 1 ||
      second_out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, second))) {
    return false;
  }
  if (state.view().poles().size() != pole_count + 1) {
    return false;
  }
  const auto midair_again = std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                                         [&](const wire::core::SavedBackboneNode& n) {
                                           return n.node_id == saved_midair_id &&
                                                  n.pole_id == wire::core::kInvalidObjectId;
                                         });
  if (midair_again == state.view().backbone().nodes.end()) {
    return false;
  }
  for (wire::core::ObjectId span_id : second_out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr || !state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr ||
        (a->owner_pole_id != wire::core::kInvalidObjectId && b->owner_pole_id != wire::core::kInvalidObjectId)) {
      return false;
    }
  }
  return C397_backbone_rejects_missing_saved_midair_node_spec();
}

bool C555_backbone_new_building_route_point_is_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 6.0}, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 1;
  node.support_kind = wire::core::SupportKind::kExternal;
  node.node_id = wire::core::kInvalidObjectId;
  req.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 2 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, req) * 2)) {
    return false;
  }
  const wire::core::SavedBackboneGraph& graph = state.view().backbone();
  if (graph.nodes.size() != 3 || graph.edges.size() != 2 || graph.edge_bundles.size() != 2) {
    return false;
  }
  const auto building_it =
      std::find_if(graph.nodes.begin(), graph.nodes.end(), [](const wire::core::SavedBackboneNode& n) {
        return n.pole_id == wire::core::kInvalidObjectId &&
               n.support_kind == wire::core::SupportKind::kExternal;
      });
  if (building_it == graph.nodes.end() || !almost_equal(building_it->position.z, 6.0, 1e-9)) {
    return false;
  }
  bool saw_building_port = false;
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr || !state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr) {
      return false;
    }
    saw_building_port = saw_building_port || (a->owner_pole_id == wire::core::kInvalidObjectId &&
                                              almost_equal(a->world_position.z, 6.0, 1e-9)) ||
                        (b->owner_pole_id == wire::core::kInvalidObjectId &&
                         almost_equal(b->world_position.z, 6.0, 1e-9));
  }
  return saw_building_port;
}

bool C556_backbone_building_pick_feeds_new_building_route_point() {
  wire::core::CoreState state;
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kExternal;
  pick.hit_id = 42;
  pick.hit_pos_world = {12.0, 0.0, 7.0};
  const auto resolved = state.ResolveBranchPick(pick);
  if (!resolved.ok || resolved.value.resolution != wire::core::PickBranchResolutionKind::kMidair ||
      resolved.value.support_kind != wire::core::SupportKind::kExternal ||
      resolved.value.resolved_node_id != wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec req = poly3_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, resolved.value.position, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 1;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  req.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 2 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, req) * 2)) {
    return false;
  }
  const auto building_it =
      std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                   [](const wire::core::SavedBackboneNode& n) {
                     return n.pole_id == wire::core::kInvalidObjectId &&
                            n.support_kind == wire::core::SupportKind::kExternal;
                   });
  if (building_it == state.view().backbone().nodes.end() || !almost_equal(building_it->position.z, 7.0, 1e-9)) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr) {
      return false;
    }
    if ((a->owner_pole_id == wire::core::kInvalidObjectId && almost_equal(a->world_position.z, 7.0, 1e-9)) ||
        (b->owner_pole_id == wire::core::kInvalidObjectId && almost_equal(b->world_position.z, 7.0, 1e-9))) {
      return true;
    }
  }
  return false;
}

bool C557_backbone_building_pick_without_id_is_supported() {
  wire::core::CoreState state;
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kExternal;
  pick.hit_id = wire::core::kInvalidObjectId;
  pick.hit_pos_world = {12.0, 0.0, 5.0};
  const auto resolved = state.ResolveBranchPick(pick);
  if (!resolved.ok || resolved.value.support_kind != wire::core::SupportKind::kExternal ||
      resolved.value.resolved_node_id != wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec req = poly3_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, resolved.value.position, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 1;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  req.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, req) * 2)) {
    return false;
  }
  const auto building_it =
      std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                   [](const wire::core::SavedBackboneNode& n) {
                     return n.pole_id == wire::core::kInvalidObjectId &&
                            n.support_kind == wire::core::SupportKind::kExternal;
                   });
  return building_it != state.view().backbone().nodes.end() && almost_equal(building_it->position.z, 5.0, 1e-9);
}

bool C558_backbone_ground_pick_feeds_new_ground_route_point() {
  wire::core::CoreState state;
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kGround;
  pick.hit_id = wire::core::kInvalidObjectId;
  pick.hit_pos_world = {12.0, 0.0, 0.0};
  const auto resolved = state.ResolveBranchPick(pick);
  if (!resolved.ok || resolved.value.resolution != wire::core::PickBranchResolutionKind::kMidair ||
      resolved.value.support_kind != wire::core::SupportKind::kGround ||
      resolved.value.resolved_node_id != wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec req = poly3_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, resolved.value.position, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 1;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  req.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 2 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, req) * 2)) {
    return false;
  }
  const auto ground_it =
      std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                   [](const wire::core::SavedBackboneNode& n) {
                     return n.pole_id == wire::core::kInvalidObjectId &&
                            n.support_kind == wire::core::SupportKind::kGround;
                   });
  if (ground_it == state.view().backbone().nodes.end()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr || !state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr) {
      return false;
    }
    if ((a->owner_pole_id == wire::core::kInvalidObjectId && almost_equal(a->world_position.z, 0.0, 1e-9)) ||
        (b->owner_pole_id == wire::core::kInvalidObjectId && almost_equal(b->world_position.z, 0.0, 1e-9))) {
      return true;
    }
  }
  return false;
}

bool C597_backbone_selected_building_pick_generates_selected_bundle_only() {
  wire::core::CoreState state;
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kExternal;
  pick.hit_id = wire::core::kInvalidObjectId;
  pick.hit_pos_world = {12.0, 0.0, 6.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.support_kind != wire::core::SupportKind::kExternal) {
    return false;
  }

  wire::core::BackboneSpec req = poly3_req(state);
  req.bundles.clear();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  req.path.polyline = {{0.0, 0.0, 0.0}, resolved.value.position, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 1;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  req.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.bundle_ids.size() != 1 || out.value.generated_span_ids.empty()) {
    return false;
  }
  const auto* bundle = state.view().bundles().find(out.value.bundle_ids.front());
  if (bundle == nullptr || bundle->bundle_template_id != wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const auto* span_bundle = state.view().bundles().find(span->bundle_id);
    if (span_bundle == nullptr || span_bundle->bundle_template_id != wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)) {
      return false;
    }
  }
  return true;
}

bool C598_backbone_selected_saved_building_node_pick_generates_selected_bundle_only() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = poly3_req(state);
  first.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 6.0}, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec building{};
  building.point_index = 1;
  building.support_kind = wire::core::SupportKind::kExternal;
  building.node_id = wire::core::kInvalidObjectId;
  first.path.node_specs = {building};
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok) {
    return false;
  }
  const auto saved_building = std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                                           [](const wire::core::SavedBackboneNode& n) {
                                             return n.pole_id == wire::core::kInvalidObjectId &&
                                                    n.support_kind == wire::core::SupportKind::kExternal;
                                           });
  if (saved_building == state.view().backbone().nodes.end()) {
    return false;
  }
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kNode;
  pick.hit_id = saved_building->node_id;
  pick.hit_pos_world = saved_building->position;
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId ||
      resolved.value.support_kind != wire::core::SupportKind::kExternal) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.bundles.clear();
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  branch.path.polyline = {resolved.value.position, {12.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.bundle_ids.size() != 1 || out.value.generated_span_ids.empty()) {
    return false;
  }
  const auto* bundle = state.view().bundles().find(out.value.bundle_ids.front());
  return bundle != nullptr && bundle->bundle_template_id == wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication);
}

bool C599_backbone_selected_saved_building_node_policy_persists_after_branch() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = poly3_req(state);
  first.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 6.0}, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec building{};
  building.point_index = 1;
  building.support_kind = wire::core::SupportKind::kExternal;
  building.node_id = wire::core::kInvalidObjectId;
  first.path.node_specs = {building};
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok) {
    return false;
  }
  const auto saved_building = std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                                           [](const wire::core::SavedBackboneNode& n) {
                                             return n.pole_id == wire::core::kInvalidObjectId &&
                                                    n.support_kind == wire::core::SupportKind::kExternal;
                                           });
  if (saved_building == state.view().backbone().nodes.end()) {
    return false;
  }
  const wire::core::ObjectId saved_id = saved_building->node_id;
  const wire::core::Vec3d saved_pos = saved_building->position;

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kNode;
  pick.hit_id = saved_id;
  pick.hit_pos_world = saved_pos;
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.bundles.clear();
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  branch.path.polyline = {resolved.value.position, {12.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec pending{};
  pending.point_index = 0;
  pending.support_kind = resolved.value.support_kind;
  pending.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {pending};
  const auto branch_out = state.GenerateFromBackboneSpec(branch);
  if (!branch_out.ok || branch_out.value.bundle_ids.size() != 1) {
    return false;
  }
  const auto* saved_after_branch = state.view().backbone_node(saved_id);
  if (saved_after_branch == nullptr ||
      std::none_of(saved_after_branch->bundle_modes.begin(), saved_after_branch->bundle_modes.end(),
                   [](const wire::core::SupportNodeBundleMode& mode) {
                     return mode.bundle_template_id == wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication);
                   })) {
    return false;
  }

  wire::core::BackboneSpec second = branch;
  second.path.polyline = {saved_pos, {12.0, -10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec existing{};
  existing.point_index = 0;
  existing.support_kind = wire::core::SupportKind::kExternal;
  existing.node_id = saved_id;
  second.path.node_specs = {existing};
  const auto out = state.GenerateFromBackboneSpec(second);
  if (!out.ok || out.value.bundle_ids.size() != 1 || out.value.generated_span_ids.empty()) {
    return false;
  }
  const auto* bundle = state.view().bundles().find(out.value.bundle_ids.front());
  return bundle != nullptr && bundle->bundle_template_id == wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication);
}

bool C600_backbone_selected_existing_pole_pick_generates_selected_bundle_only() {
  wire::core::CoreState state;
  const auto base = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!base.ok || base.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = base.value.generated_pole_ids[1];
  const auto* pole = state.view().poles().find(b);
  if (pole == nullptr) {
    return false;
  }
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kNode;
  pick.hit_id = b;
  pick.hit_pos_world = pole->world_transform.position;
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.bundles.clear();
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kCommunication);
  branch.path.polyline = {resolved.value.position, {12.0, 12.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.bundle_ids.size() != 1 || out.value.generated_span_ids.empty()) {
    return false;
  }
  const auto* bundle = state.view().bundles().find(out.value.bundle_ids.front());
  if (bundle == nullptr || bundle->bundle_template_id != wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    const auto* span_bundle = span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
    if (span_bundle == nullptr || span_bundle->bundle_template_id != wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kCommunication)) {
      return false;
    }
  }
  const auto* saved_pole = state.view().backbone_node_for_pole(b);
  return saved_pole != nullptr && saved_pole->bundle_modes.empty();
}

bool C604_backbone_large_avoid_detour_clears_radius() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.constraints.avoid_points = {{6.0, 0.0, 0.0}};
  req.constraints.avoid_radius_m = 4.0;
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 3) {
    return false;
  }
  for (wire::core::ObjectId pole_id : out.value.generated_pole_ids) {
    const auto* pole = state.view().poles().find(pole_id);
    if (pole == nullptr) {
      return false;
    }
    const wire::core::Vec3d d = pole->world_transform.position - wire::core::Vec3d{6.0, 0.0, 0.0};
    if (d.x * d.x + d.y * d.y + d.z * d.z <= 16.0) {
      return false;
    }
  }
  return true;
}

bool C605_backbone_find_backbone_route_uses_saved_ownerless_graph() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = line_req(state);
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  first.path.node_specs = {midair};
  const auto out = state.GenerateFromBackboneSpec(first);
  if (!out.ok || out.value.generated_pole_ids.size() != 2) {
    return false;
  }
  const auto saved_midair =
      std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                   [](const wire::core::SavedBackboneNode& node) {
                     return node.support_kind == wire::core::SupportKind::kMidair;
                   });
  if (saved_midair == state.view().backbone().nodes.end()) {
    return false;
  }
  const auto route = state.FindSavedBackboneRoute(saved_midair->node_id, out.value.generated_pole_ids.back());
  return route.size() == 2 && route.front() == saved_midair->node_id && route.back() == out.value.generated_pole_ids.back();
}

bool C606_backbone_saved_backbone_result_exposes_saved_ownerless_node() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = poly3_req(state);
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 8.0}, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  req.path.node_specs = {midair};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok) {
    return false;
  }
  const auto saved = std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                                  [](const wire::core::SavedBackboneNode& node) {
                                    return node.pole_id == wire::core::kInvalidObjectId &&
                                           node.support_kind == wire::core::SupportKind::kMidair;
                                  });
  if (saved == state.view().backbone().nodes.end()) {
    return false;
  }
  const wire::core::BackboneResult result = state.SavedBackboneResult();
  const auto node = std::find_if(result.nodes.begin(), result.nodes.end(), [&](const wire::core::SupportNode& item) {
    return item.node_id == saved->node_id && item.support_kind == wire::core::SupportKind::kMidair &&
           item.pole_id == wire::core::kInvalidObjectId;
  });
  return node != result.nodes.end() && almost_equal(node->position, saved->position, 1e-9);
}

bool C607_backbone_saved_backbone_result_preserves_saved_ownerless_route_index() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = poly3_req(state);
  first.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 6.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.node_specs = {midair};
  const auto out1 = state.GenerateFromBackboneSpec(first);
  if (!out1.ok) {
    return false;
  }
  const wire::core::BackboneResult first_result = state.SavedBackboneResult();
  const auto first_midair = std::find_if(first_result.nodes.begin(), first_result.nodes.end(),
                                         [](const wire::core::SupportNode& node) {
                                           return node.support_kind == wire::core::SupportKind::kMidair &&
                                                  node.pole_id == wire::core::kInvalidObjectId &&
                                                  node.path_point_index == 1;
                                         });
  if (first_midair == first_result.nodes.end() || first_midair->node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {first_midair->position, {10.0, 12.0, 6.0}};
  wire::core::BackboneInputSpec::NodeSpec reused{};
  reused.point_index = 0;
  reused.support_kind = wire::core::SupportKind::kMidair;
  reused.node_id = first_midair->node_id;
  second.path.node_specs = {reused};
  const auto out2 = state.GenerateFromBackboneSpec(second);
  if (!out2.ok) {
    return false;
  }
  const wire::core::BackboneResult second_result = state.SavedBackboneResult();
  const auto reused_midair = std::find_if(second_result.nodes.begin(), second_result.nodes.end(),
                                          [&](const wire::core::SupportNode& node) {
                                            return node.node_id == first_midair->node_id &&
                                                   node.support_kind == wire::core::SupportKind::kMidair &&
                                                   node.path_point_index == 0;
                                          });
  return reused_midair != second_result.nodes.end();
}

bool C560_backbone_segment_pick_without_bundle_policy_feeds_midair_route_point() {
  wire::core::CoreState state;
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = wire::core::kInvalidObjectId;
  pick.hit_pos_world = {6.0, 0.0, 4.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = wire::core::kInvalidObjectId;
  pick.segment_node_b_id = wire::core::kInvalidObjectId;
  pick.segment_endpoint_a_world = {0.0, 0.0, 4.0};
  pick.segment_endpoint_b_world = {12.0, 0.0, 4.0};

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids.clear();
  resolve.create_midair_node = false;
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolution != wire::core::PickBranchResolutionKind::kMidair ||
      resolved.value.support_kind != wire::core::SupportKind::kMidair ||
      resolved.value.resolved_node_id != wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  req.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 1 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, req))) {
    return false;
  }

  const auto midair_it =
      std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                   [](const wire::core::SavedBackboneNode& n) {
                     return n.pole_id == wire::core::kInvalidObjectId &&
                            n.support_kind == wire::core::SupportKind::kMidair;
                   });
  if (midair_it == state.view().backbone().nodes.end() || !almost_equal(midair_it->position.z, 4.0, 1e-9)) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr || !state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr) {
      return false;
    }
    if ((a->owner_pole_id == wire::core::kInvalidObjectId && almost_equal(a->world_position.z, 4.0, 1e-9)) ||
        (b->owner_pole_id == wire::core::kInvalidObjectId && almost_equal(b->world_position.z, 4.0, 1e-9))) {
      return true;
    }
  }
  return false;
}

bool C561_backbone_default_segment_pick_without_bundle_policy_is_ownerless_midair() {
  wire::core::CoreState state;
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = wire::core::kInvalidObjectId;
  pick.hit_pos_world = {6.0, 0.0, 3.0};
  pick.has_segment_endpoints = true;
  pick.segment_node_a_id = wire::core::kInvalidObjectId;
  pick.segment_node_b_id = wire::core::kInvalidObjectId;
  pick.segment_endpoint_a_world = {0.0, 0.0, 3.0};
  pick.segment_endpoint_b_world = {12.0, 0.0, 3.0};

  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids.clear();
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolution != wire::core::PickBranchResolutionKind::kMidair ||
      resolved.value.support_kind != wire::core::SupportKind::kMidair ||
      resolved.value.resolved_node_id != wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec req = line_req(state);
  req.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  req.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_pole_ids.size() != 1 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, req))) {
    return false;
  }
  const auto midair_it =
      std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                   [](const wire::core::SavedBackboneNode& n) {
                     return n.pole_id == wire::core::kInvalidObjectId &&
                            n.support_kind == wire::core::SupportKind::kMidair;
                   });
  if (midair_it == state.view().backbone().nodes.end() || !almost_equal(midair_it->position.z, 3.0, 1e-9)) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr || !state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
  }
  return true;
}

bool C562_backbone_saved_midair_node_pick_extends_from_saved_graph_node() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = poly3_req(state);
  first.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 5.0}, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.node_specs = {midair};
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok) {
    return false;
  }
  const auto saved_midair =
      std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                   [](const wire::core::SavedBackboneNode& n) {
                     return n.pole_id == wire::core::kInvalidObjectId &&
                            n.support_kind == wire::core::SupportKind::kMidair;
                   });
  if (saved_midair == state.view().backbone().nodes.end()) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kNode;
  pick.hit_id = saved_midair->node_id;
  pick.hit_pos_world = {0.0, 0.0, 0.0};
  const auto resolved = state.ResolveBranchPick(pick);
  if (!resolved.ok || resolved.value.resolution != wire::core::PickBranchResolutionKind::kNode ||
      resolved.value.resolved_node_id != saved_midair->node_id ||
      resolved.value.support_kind != wire::core::SupportKind::kMidair ||
      !almost_equal(resolved.value.position, saved_midair->position, 1e-9)) {
    return false;
  }

  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {resolved.value.position, {12.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec existing{};
  existing.point_index = 0;
  existing.support_kind = resolved.value.support_kind;
  existing.node_id = resolved.value.resolved_node_id;
  second.path.node_specs = {existing};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  if (!second_out.ok || second_out.value.generated_pole_ids.size() != 1 ||
      second_out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, second))) {
    return false;
  }
  for (wire::core::ObjectId span_id : second_out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr || !state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr) {
      return false;
    }
  }
  return true;
}

bool C563_backbone_segment_pick_snaps_to_saved_ownerless_span_endpoint() {
  wire::core::CoreState state;
  wire::core::BackboneSpec first = poly3_req(state);
  first.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 5.0}, {24.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec midair{};
  midair.point_index = 1;
  midair.support_kind = wire::core::SupportKind::kMidair;
  first.path.node_specs = {midair};
  const auto first_out = state.GenerateFromBackboneSpec(first);
  if (!first_out.ok) {
    return false;
  }
  const auto saved_midair =
      std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                   [](const wire::core::SavedBackboneNode& n) {
                     return n.pole_id == wire::core::kInvalidObjectId &&
                            n.support_kind == wire::core::SupportKind::kMidair;
                   });
  if (saved_midair == state.view().backbone().nodes.end()) {
    return false;
  }

  wire::core::ObjectId hit_span = wire::core::kInvalidObjectId;
  for (wire::core::ObjectId span_id : first_out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr) {
      return false;
    }
    if ((a->owner_pole_id == wire::core::kInvalidObjectId &&
         almost_equal(a->world_position, saved_midair->position, 1e-9)) ||
        (b->owner_pole_id == wire::core::kInvalidObjectId &&
         almost_equal(b->world_position, saved_midair->position, 1e-9))) {
      hit_span = span_id;
      break;
    }
  }
  if (hit_span == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = hit_span;
  pick.hit_pos_world = saved_midair->position;
  pick.has_segment_endpoints = false;
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.snap_radius_world = 0.75;
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolution != wire::core::PickBranchResolutionKind::kNode ||
      resolved.value.resolved_node_id != saved_midair->node_id ||
      resolved.value.support_kind != wire::core::SupportKind::kMidair ||
      !resolved.value.snapped_from_segment_endpoint) {
    return false;
  }

  wire::core::BackboneSpec second = line_req(state);
  second.path.polyline = {resolved.value.position, {12.0, 10.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec existing{};
  existing.point_index = 0;
  existing.support_kind = resolved.value.support_kind;
  existing.node_id = resolved.value.resolved_node_id;
  second.path.node_specs = {existing};
  const auto second_out = state.GenerateFromBackboneSpec(second);
  return second_out.ok && second_out.value.generated_pole_ids.size() == 1 &&
         second_out.value.generated_span_ids.size() == static_cast<std::size_t>(req_bundle_count(state, second));
}

bool C564_backbone_selected_bundle_segment_pick_feeds_transient_midair_node() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = base_out.value.generated_span_ids.front();
  pick.hit_pos_world = {6.0, 0.0, 4.0};
  pick.has_segment_endpoints = false;
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolution != wire::core::PickBranchResolutionKind::kMidair ||
      resolved.value.support_kind != wire::core::SupportKind::kMidair ||
      resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.generated_pole_ids.size() != 1 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(req_bundle_count(state, branch))) {
    return false;
  }

  const auto saved_midair =
      std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                   [&](const wire::core::SavedBackboneNode& n) {
                     return n.pole_id == wire::core::kInvalidObjectId &&
                            n.support_kind == wire::core::SupportKind::kMidair &&
                            almost_equal(n.position, resolved.value.position, 1e-9);
                   });
  if (saved_midair == state.view().backbone().nodes.end()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr || !state.span_layout_rules(span_id).has_rule() || !state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr || state.find_bounds_cache(span_id) == nullptr ||
        state.find_span_visual_cache(span_id) == nullptr || state.view().find_span_render_cache(span_id) == nullptr) {
      return false;
    }
    const auto* a = state.view().ports().find(span->port_a_id);
    const auto* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr) {
      return false;
    }
    if ((a->owner_pole_id == wire::core::kInvalidObjectId &&
         almost_equal(a->world_position.z, resolved.value.position.z, 1e-9)) ||
        (b->owner_pole_id == wire::core::kInvalidObjectId &&
         almost_equal(b->world_position.z, resolved.value.position.z, 1e-9))) {
      return true;
    }
  }
  return false;
}

bool C565_backbone_mixed_selected_midair_branch_generates_allowed_bundles_only() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = base_out.value.generated_span_ids.front();
  pick.hit_pos_world = {6.0, 0.0, 4.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage), wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  branch.bundles.clear();
  add_backbone_bundle(branch, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.bundle_ids.size() != 1 ||
      out.value.generated_span_ids.size() != static_cast<std::size_t>(bundle_count(state, wire::core::BundleKind::kLowVoltage))) {
    return false;
  }
  const auto* bundle = state.view().bundles().find(out.value.bundle_ids.front());
  if (bundle == nullptr || bundle->bundle_template_id != wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage)) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const auto* span_bundle = state.view().bundles().find(span->bundle_id);
    if (span_bundle == nullptr || span_bundle->bundle_template_id != wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage)) {
      return false;
    }
  }
  return true;
}

bool C566_backbone_disallowed_selected_midair_branch_is_noop() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) {
    return false;
  }

  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = base_out.value.generated_span_ids.front();
  pick.hit_pos_world = {6.0, 0.0, 4.0};
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage), wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  branch.bundles.clear();
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};

  const std::size_t pole_count = state.view().poles().size();
  const std::size_t port_count = state.view().ports().size();
  const std::size_t bundle_count_before = state.view().bundles().size();
  const std::size_t span_count = state.view().spans().size();
  const std::size_t saved_node_count = state.view().backbone().nodes.size();
  const std::size_t saved_edge_count = state.view().backbone().edges.size();
  const std::size_t saved_edge_bundle_count = state.view().backbone().edge_bundles.size();
  const auto out = state.GenerateFromBackboneSpec(branch);
  return out.ok && out.value.bundle_ids.empty() && out.value.generated_pole_ids.empty() &&
         out.value.generated_span_ids.empty() && state.view().poles().size() == pole_count &&
         state.view().ports().size() == port_count && state.view().bundles().size() == bundle_count_before &&
         state.view().spans().size() == span_count && state.view().backbone().nodes.size() == saved_node_count &&
         state.view().backbone().edges.size() == saved_edge_count &&
         state.view().backbone().edge_bundles.size() == saved_edge_bundle_count;
}

bool C569_backbone_render_uses_cable_template_appearance() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const auto* span = state.view().spans().find(span_id);
    const wire::core::SpanRenderCacheEntry* render = state.view().find_span_render_cache(span_id);
    if (span == nullptr || render == nullptr) {
      return false;
    }
    const auto* bundle = state.view().bundles().find(span->bundle_id);
    if (bundle == nullptr) {
      return false;
    }
    const auto bundle_template_it = state.view().bundle_templates().find(bundle->bundle_template_id);
    if (bundle_template_it == state.view().bundle_templates().end()) {
      return false;
    }
    const auto cable_it = state.view().cable_templates().find(bundle_template_it->second.cable_template_id);
    if (cable_it == state.view().cable_templates().end()) {
      return false;
    }
    const double expected_radius = std::max(0.0005, cable_it->second.outer_diameter_m * 0.5);
    if (!almost_equal(render->wire_radius_m, expected_radius, 1e-12) ||
        render->color_rgba != cable_it->second.color_rgba ||
        render->material_style != cable_it->second.material_style) {
      return false;
    }
  }
  return true;
}

bool C570_backbone_support_visual_uses_visual_settings_radius() {
  wire::core::CoreState state;
  const std::vector<wire::core::ObjectId> spans = lowering_branch_spans(state);
  if (spans.empty()) {
    return false;
  }
  const double expected = state.view().visual_settings().support_arm_radius_m;
  for (wire::core::ObjectId span_id : spans) {
    const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    if (visual == nullptr) {
      return false;
    }
    for (const wire::core::VisualPart& part : visual->parts) {
      if (part.kind == wire::core::VisualPartKind::kSupportArm) {
        return almost_equal(part.radius_m, expected, 1e-12);
      }
    }
  }
  return false;
}

bool C571_backbone_support_visual_respects_enable_setting() {
  wire::core::CoreState state;
  wire::core::VisualSettings settings = state.view().visual_settings();
  settings.enable_support_structures = false;
  const auto updated = state.UpdateVisualSettings(settings, false);
  if (!updated.ok) {
    return false;
  }

  const std::vector<wire::core::ObjectId> spans = lowering_branch_spans(state);
  if (spans.empty()) {
    return false;
  }

  bool saw_lowered_endpoint = false;
  for (wire::core::ObjectId span_id : spans) {
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    if (!state.span_layout_rules(span_id).has_rule() || !layout.has_layout() || state.find_curve_cache(span_id) == nullptr ||
        state.find_bounds_cache(span_id) == nullptr || state.find_span_render_cache(span_id) == nullptr || visual == nullptr) {
      return false;
    }
    const auto lowered = [](const wire::core::LayoutEndpoint& endpoint) {
      return (endpoint.default_lower_required || endpoint.lower_required) &&
             !almost_equal(endpoint.support_world.z, endpoint.endpoint_world.z, 1e-9);
    };
    saw_lowered_endpoint = saw_lowered_endpoint || lowered(layout.entry->start) || lowered(layout.entry->end);
    for (const wire::core::VisualPart& part : visual->parts) {
      if (part.kind == wire::core::VisualPartKind::kSupportArm) {
        return false;
      }
    }
  }
  return saw_lowered_endpoint;
}

bool C572_backbone_support_visual_radius_setting_is_mutable() {
  wire::core::CoreState state;
  wire::core::VisualSettings settings = state.view().visual_settings();
  settings.support_arm_radius_m = 0.123;
  const auto updated = state.UpdateVisualSettings(settings, false);
  if (!updated.ok || !updated.value) {
    return false;
  }

  const std::vector<wire::core::ObjectId> spans = lowering_branch_spans(state);
  if (spans.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : spans) {
    const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
    if (visual == nullptr) {
      return false;
    }
    for (const wire::core::VisualPart& part : visual->parts) {
      if (part.kind == wire::core::VisualPartKind::kSupportArm) {
        return almost_equal(part.radius_m, settings.support_arm_radius_m, 1e-12);
      }
    }
  }
  return false;
}

bool C609_backbone_ordinary_bend_does_not_lower() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(hv_poly3_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    if (span == nullptr || !layout.has_layout()) {
      return false;
    }
    const wire::core::Port* a = state.view().ports().find(span->port_a_id);
    const wire::core::Port* b = state.view().ports().find(span->port_b_id);
    if (a == nullptr || b == nullptr || layout.entry->start.default_lower_required ||
        layout.entry->start.lower_required || layout.entry->end.default_lower_required ||
        layout.entry->end.lower_required || !almost_equal(layout.entry->start.support_world, a->world_position, 1e-9) ||
        !almost_equal(layout.entry->start.endpoint_world, a->world_position, 1e-9) ||
        !almost_equal(layout.entry->end.support_world, b->world_position, 1e-9) ||
        !almost_equal(layout.entry->end.endpoint_world, b->world_position, 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C610_backbone_conflict_lowers_eligible_bundle_endpoint_only() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = poly3_req(state);
  base.bundles.clear();
  add_backbone_bundle(base, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(base, wire::core::BundleKind::kHighVoltage);
  add_backbone_bundle(base, wire::core::BundleKind::kCommunication);
  add_backbone_bundle(base, wire::core::BundleKind::kOptical);
  const auto first = state.GenerateFromBackboneSpec(base);
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId junction = first.value.generated_pole_ids[1];
  const wire::core::Pole* pole = state.view().poles().find(junction);
  if (pole == nullptr) {
    return false;
  }
  wire::core::BackboneSpec branch = base;
  branch.path.polyline = {pole->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, junction)};
  branch.node_bundle_modes.clear();
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.generated_span_ids.size() != 6) {
    return false;
  }
  const auto hv_template = state.view().bundle_templates().find(wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage));
  if (hv_template == state.view().bundle_templates().end() || !hv_template->second.enable_branch_down_offset ||
      hv_template->second.branch_endpoint_offset_m >= 0.0) {
    return false;
  }

  int lowered_hv = 0;
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    const wire::core::Span* span = state.view().spans().find(span_id);
    const wire::core::SpanLayoutView layout = state.span_layout(span_id);
    const wire::core::Bundle* bundle = span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
    if (bundle == nullptr || !layout.has_layout()) {
      return false;
    }
    const bool eligible = bundle->bundle_template_id == wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kHighVoltage);
    const auto endpoint_ok = [&](const wire::core::LayoutEndpoint& endpoint) {
      const bool at_junction = endpoint.endpoint_node_id == junction;
      const bool lowered = endpoint.default_lower_required || endpoint.lower_required;
      if (eligible && at_junction) {
        return lowered &&
               almost_equal(endpoint.endpoint_offset_z_m, hv_template->second.branch_endpoint_offset_m, 1e-9) &&
               almost_equal(endpoint.endpoint_world.z,
                            endpoint.support_world.z + hv_template->second.branch_endpoint_offset_m, 1e-9);
      }
      return !lowered && almost_equal(endpoint.endpoint_world, endpoint.support_world, 1e-9);
    };
    if (!endpoint_ok(layout.entry->start) || !endpoint_ok(layout.entry->end)) {
      return false;
    }
    lowered_hv += eligible ? 1 : 0;
  }
  return lowered_hv == 3;
}

bool C573_backbone_saved_context_node_carries_support_metadata() {
  const std::filesystem::path source = repo_root() / "core" / "src" / "generation" / "backbone" / "pipeline.cpp";
  std::string cpp;
  if (!file_text(source, &cpp)) {
    return false;
  }
  const std::size_t fn_pos = cpp.find("auto local_for_saved =");
  const std::size_t next_pos = cpp.find("std::unordered_set<ObjectId> context_edges", fn_pos);
  if (fn_pos == std::string::npos || next_pos == std::string::npos) {
    return false;
  }
  const std::string body = cpp.substr(fn_pos, next_pos - fn_pos);
  return contains_text(body, "n.support = saved->support_kind") &&
         contains_text(body, "n.has_source_edge = saved->has_source_edge") &&
         contains_text(body, "n.source_edge_node_a = saved->source_edge_node_a") &&
         contains_text(body, "n.source_edge_node_b = saved->source_edge_node_b") &&
         contains_text(body, "n.source_edge_t = saved->source_edge_t");
}

bool C574_backbone_same_edge_different_bundle_with_pass_through_is_supported() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(line_req(state));
  if (!first.ok || first.value.generated_pole_ids.size() != 2 || state.view().backbone().edges.size() != 1) {
    return false;
  }
  const wire::core::ObjectId a = first.value.generated_pole_ids[0];
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const auto* pa = state.view().poles().find(a);
  const auto* pb = state.view().poles().find(b);
  if (pa == nullptr || pb == nullptr) {
    return false;
  }
  wire::core::BackboneSpec second = line_req(state);
  second.bundles.clear();
  add_backbone_bundle(second, wire::core::BundleKind::kCommunication);
  second.path.polyline = {pa->world_transform.position, pb->world_transform.position};
  second.path.node_specs = {pole_spec(0, a), pole_spec(1, b)};
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 0;
  mode.bundle_template_id = second.bundles.front().bundle_template_id;
  mode.mode = wire::core::BundleNodeMode::kPassThrough;
  second.node_bundle_modes = {mode};
  const auto out = state.GenerateFromBackboneSpec(second);
  if (!out.ok || out.value.generated_span_ids.empty() || state.view().backbone().edges.size() != 1 ||
      state.view().backbone().edge_bundles.size() != 2) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (span_has_lowered_endpoint(state, span_id)) {
      return false;
    }
  }
  return true;
}

bool C575_backbone_stale_segment_pick_midair_duplicate_rejected_unchanged() {
  wire::core::CoreState state;
  const auto base_out = state.GenerateFromBackboneSpec(line_req(state));
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) {
    return false;
  }
  const auto* source_span = state.view().spans().find(base_out.value.generated_span_ids.front());
  if (source_span == nullptr) {
    return false;
  }
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span->id;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  pick.has_segment_endpoints = false;
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto branch_out = state.GenerateFromBackboneSpec(branch);
  if (!branch_out.ok || branch_out.value.generated_pole_ids.size() != 1) {
    return false;
  }
  wire::core::BackboneSpec duplicate = branch;
  duplicate.path.node_specs.push_back(pole_spec(1, branch_out.value.generated_pole_ids.front()));
  const std::size_t pole_count = state.view().poles().size();
  const std::size_t port_count = state.view().ports().size();
  const std::size_t bundle_count_before = state.view().bundles().size();
  const std::size_t span_count = state.view().spans().size();
  const std::size_t saved_node_count = state.view().backbone().nodes.size();
  const std::size_t saved_edge_count = state.view().backbone().edges.size();
  const std::size_t saved_edge_bundle_count = state.view().backbone().edge_bundles.size();
  const auto dup = state.GenerateFromBackboneSpec(duplicate);
  return !dup.ok && contains_text(dup.error, "unsupported") && state.view().poles().size() == pole_count &&
         state.view().ports().size() == port_count && state.view().bundles().size() == bundle_count_before &&
         state.view().spans().size() == span_count && state.view().backbone().nodes.size() == saved_node_count &&
         state.view().backbone().edges.size() == saved_edge_count &&
         state.view().backbone().edge_bundles.size() == saved_edge_bundle_count;
}

bool C576_backbone_ownerless_multiple_bundles_do_not_require_pole_type() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.pole_type_id = wire::core::kInvalidPoleTypeId;
  req.path.polyline = {{0.0, 0.0, 8.0}, {10.0, 0.0, 8.0}};
  req.bundles.clear();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  wire::core::BackboneInputSpec::NodeSpec a{};
  a.point_index = 0;
  a.support_kind = wire::core::SupportKind::kMidair;
  wire::core::BackboneInputSpec::NodeSpec b{};
  b.point_index = 1;
  b.support_kind = wire::core::SupportKind::kMidair;
  req.path.node_specs = {a, b};
  const auto out = state.GenerateFromBackboneSpec(req);
  return out.ok && out.value.generated_pole_ids.empty() && out.value.bundle_ids.size() == 2 &&
         !out.value.generated_span_ids.empty() && state.view().backbone().nodes.size() == 2 &&
         state.view().backbone().edges.size() == 1;
}

bool C577_backbone_missing_port_band_rejects_before_mutation() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  auto it = state.view().pole_types().find(req.pole_type_id);
  if (it == state.view().pole_types().end()) {
    return false;
  }
  wire::core::PoleTypeDefinition type = it->second;
  type.port_bands.erase(std::remove_if(type.port_bands.begin(), type.port_bands.end(),
                                       [](const wire::core::PortPlacementBand& band) {
                                         return band.category == wire::core::ConnectionCategory::kLowVoltage &&
                                                band.layer == 1;
                                       }),
                        type.port_bands.end());
  if (!state.UpdatePoleTypeDefinition(type).ok) {
    return false;
  }
  const std::size_t pole_count = state.view().poles().size();
  const std::size_t port_count = state.view().ports().size();
  const std::size_t bundle_count_before = state.view().bundles().size();
  const std::size_t span_count = state.view().spans().size();
  const std::size_t saved_node_count = state.view().backbone().nodes.size();
  const std::size_t saved_edge_count = state.view().backbone().edges.size();
  const auto out = state.GenerateFromBackboneSpec(req);
  return !out.ok && state.view().poles().size() == pole_count && state.view().ports().size() == port_count &&
         state.view().bundles().size() == bundle_count_before && state.view().spans().size() == span_count &&
         state.view().backbone().nodes.size() == saved_node_count && state.view().backbone().edges.size() == saved_edge_count;
}

bool C578_backbone_segment_pick_midair_pass_through_supported() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base = line_req(state);
  const auto base_out = state.GenerateFromBackboneSpec(base);
  if (!base_out.ok || base_out.value.generated_span_ids.empty()) {
    return false;
  }
  const auto* source_span = state.view().spans().find(base_out.value.generated_span_ids.front());
  if (source_span == nullptr) {
    return false;
  }
  wire::core::PickResult pick{};
  pick.hit_kind = wire::core::PickHitKind::kSegment;
  pick.hit_id = source_span->id;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  pick.has_segment_endpoints = false;
  wire::core::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage)};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 0;
  mode.bundle_template_id = branch.bundles.front().bundle_template_id;
  mode.mode = wire::core::BundleNodeMode::kPassThrough;
  branch.node_bundle_modes = {mode};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : out.value.generated_span_ids) {
    if (span_has_lowered_endpoint(state, span_id)) {
      return false;
    }
  }
  return true;
}

bool C559_backbone_positive_avoid_clear_of_route_is_noop() {
  wire::core::CoreState plain;
  wire::core::BackboneSpec base = line_req(plain);
  const auto base_out = plain.GenerateFromBackboneSpec(base);
  if (!base_out.ok) {
    return false;
  }
  const std::vector<wire::core::Vec3d> base_samples = span_curve_points(plain, base_out.value.generated_span_ids);
  if (base_samples.empty()) {
    return false;
  }

  wire::core::CoreState with_avoid;
  wire::core::BackboneSpec req = line_req(with_avoid);
  req.constraints.avoid_points.push_back({6.0, 100.0, 0.0});
  req.constraints.avoid_radius_m = 1.0;
  const auto out = with_avoid.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.size() != base_out.value.generated_span_ids.size()) {
    return false;
  }
  const std::vector<wire::core::Vec3d> samples = span_curve_points(with_avoid, out.value.generated_span_ids);
  if (samples.size() != base_samples.size()) {
    return false;
  }
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (!almost_equal(samples[i].x, base_samples[i].x, 1e-9) ||
        !almost_equal(samples[i].y, base_samples[i].y, 1e-9) ||
        !almost_equal(samples[i].z, base_samples[i].z, 1e-9)) {
      return false;
    }
  }
  return true;
}

bool C737_backbone_overlay_edge_endpoint_snap_returns_pole_node_spec_id() {
  wire::core::CoreState state;
  const auto base_out = state.GenerateFromBackboneSpec(line_req(state));
  if (!base_out.ok || base_out.value.generated_span_ids.empty() || state.view().backbone().edges.empty()) {
    return false;
  }

  wire::core::PickResult source_pick{};
  source_pick.hit_kind = wire::core::PickHitKind::kSegment;
  source_pick.hit_id = base_out.value.generated_span_ids.front();
  source_pick.hit_pos_world = {6.0, 0.0, 0.0};
  wire::core::ResolveBranchPickOptions source_resolve{};
  source_resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage)};
  const auto source_resolved = state.ResolveBranchPick(source_pick, source_resolve);
  if (!source_resolved.ok || source_resolved.value.resolved_node_id == wire::core::kInvalidObjectId) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state);
  branch.path.polyline = {source_resolved.value.position, {6.0, 8.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec branch_node{};
  branch_node.point_index = 0;
  branch_node.support_kind = source_resolved.value.support_kind;
  branch_node.node_id = source_resolved.value.resolved_node_id;
  branch.path.node_specs = {branch_node};
  const auto branch_out = state.GenerateFromBackboneSpec(branch);
  if (!branch_out.ok || branch_out.value.generated_span_ids.empty()) {
    return false;
  }

  const wire::core::SavedBackboneEdge source_edge = state.view().backbone().edges.front();
  const wire::core::SavedBackboneNode* node_a = state.view().backbone_node(source_edge.node_a);
  const wire::core::SavedBackboneNode* node_b = state.view().backbone_node(source_edge.node_b);
  if (node_a == nullptr || node_b == nullptr || node_a->pole_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::PickResult endpoint_pick{};
  endpoint_pick.hit_kind = wire::core::PickHitKind::kSegment;
  endpoint_pick.hit_id = wire::core::kInvalidObjectId;
  endpoint_pick.hit_pos_world = node_a->position;
  endpoint_pick.has_segment_endpoints = true;
  endpoint_pick.segment_node_a_id = node_a->node_id;
  endpoint_pick.segment_node_b_id = node_b->node_id;
  endpoint_pick.segment_endpoint_a_world = node_a->position;
  endpoint_pick.segment_endpoint_b_world = node_b->position;
  wire::core::ResolveBranchPickOptions endpoint_resolve{};
  endpoint_resolve.selected_bundle_template_ids = {wire::core::DefaultBundleTemplateId(wire::core::BundleKind::kLowVoltage)};
  const auto endpoint_resolved = state.ResolveBranchPick(endpoint_pick, endpoint_resolve);
  if (!endpoint_resolved.ok || endpoint_resolved.value.support_kind != wire::core::SupportKind::kPole ||
      endpoint_resolved.value.resolved_node_id != node_a->pole_id ||
      !endpoint_resolved.value.snapped_from_segment_endpoint) {
    return false;
  }

  wire::core::BackboneSpec extension = line_req(state);
  extension.path.polyline = {endpoint_resolved.value.position, {-6.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec endpoint_node{};
  endpoint_node.point_index = 0;
  endpoint_node.support_kind = endpoint_resolved.value.support_kind;
  endpoint_node.node_id = endpoint_resolved.value.resolved_node_id;
  extension.path.node_specs = {endpoint_node};
  const auto extension_out = state.GenerateFromBackboneSpec(extension);
  return extension_out.ok && !extension_out.value.generated_span_ids.empty();
}

} // namespace backbone_tests
