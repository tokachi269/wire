#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"
#include "wire/core/core_view.hpp"
#include "backbone_generation_plan_internal.hpp"
#include "backbone_pole_orientation_policy.hpp"
#include "detail_utils.hpp"
#include "../pole_orientation_utils.hpp"
#include "../support_orientation_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <tuple>
#include <unordered_map>

namespace wire::core {

using namespace generation::detail;

namespace {

Vec3d normalize_forward_xy_orientation(const Vec3d& value) {
  Vec3d out{value.x, value.y, 0.0};
  if (!Normalize(&out)) {
    return {};
  }
  return out;
}

Vec3d choose_continuous_axis_orientation(const Vec3d& axis, const Vec3d& previous_forward) {
  Vec3d out = axis;
  if (!Normalize(&out)) {
    return {};
  }
  Vec3d prev = normalize_forward_xy_orientation(previous_forward);
  if (Dot(out, prev) < 0.0) {
    out = ScaleVec(out, -1.0);
  }
  return out;
}

std::optional<PortLayoutYawOverride> row_layout_yaw_override_from_debug_context(
    const PoleOrientationDebugRecord& debug_record) {
  if (debug_record.row_layout_axis_mode != RowLayoutAxisMode::kSupportAxis) {
    return std::nullopt;
  }
  Vec3d support_axis = debug_record.adopted_support_axis;
  if (!Normalize(&support_axis)) {
    return std::nullopt;
  }
  PortLayoutYawOverride override{};
  override.category = debug_record.row_layout_axis_category;
  override.yaw_deg = normalize_yaw_deg(std::atan2(support_axis.y, support_axis.x) * (180.0 / kPi) - 90.0);
  return override;
}

struct SupportAxisSelection {
  Vec3d axis{};
  bool available = false;
  PoleSupportAxisRule rule = PoleSupportAxisRule::kFallback;
  ObjectId primary_neighbor_id = kInvalidObjectId;
  ObjectId secondary_neighbor_id = kInvalidObjectId;
};

} // namespace

EditResult<BackboneOrientationPlan> CoreState::build_backbone_orientation_plan(
    const BackboneGenerationRequestPlan& request_plan, const BackboneSupportChainPlan& support_chain_plan,
    const BackboneTopologyPlan& topology_plan) const {
  EditResult<BackboneOrientationPlan> result{};
  BackboneOrientationPlan plan{};
  const auto& previous_debug_records = debug_.pole_orientation_debug_records;
  std::unordered_map<ObjectId, BackboneOrientationNodeContext> orientation_context_by_node{};

  const EditState& edit_state = edit_state_access();
  const auto& ordered_support_node_ids = support_chain_plan.ordered_support_node_ids;
  const auto& support_node_by_id = support_chain_plan.support_node_by_id;
  const auto& existing_node_position_by_id = topology_plan.existing_node_position_by_id;
  const auto& junction_relations_by_node = topology_plan.decision_phase.junction_relations_by_node;
  const std::vector<BackboneBundlePlan>& active_bundle_plans = request_plan.active_bundle_plans;

  auto current_support_position = [&](ObjectId node_id) -> Vec3d {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      return it->second.position;
    }
    if (const Pole* pole = view().poles().find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    if (const auto it = existing_node_position_by_id.find(node_id); it != existing_node_position_by_id.end()) {
      return it->second;
    }
    return {};
  };
  auto connected_neighbors_for_support_axis = [&](ObjectId node_id) {
    std::vector<ObjectId> neighbors{};
    for (const Span& span : edit_state.spans.items()) {
      const Port* port_a = edit_state.ports.find(span.port_a_id);
      const Port* port_b = edit_state.ports.find(span.port_b_id);
      if (port_a == nullptr || port_b == nullptr) {
        continue;
      }
      if (port_a->owner_pole_id == node_id && port_b->owner_pole_id != kInvalidObjectId &&
          port_b->owner_pole_id != node_id) {
        neighbors.push_back(port_b->owner_pole_id);
      }
      if (port_b->owner_pole_id == node_id && port_a->owner_pole_id != kInvalidObjectId &&
          port_a->owner_pole_id != node_id) {
        neighbors.push_back(port_a->owner_pole_id);
      }
    }
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    return neighbors;
  };

  struct BundleCategoryKey {
    ConnectionCategory category = ConnectionCategory::kLowVoltage;
    BundleKind bundle_template_id = BundleKind::kLowVoltage;
  };
  BundleCategoryKey route_bundle_category_key{};
  bool has_route_bundle_category_key = false;
  for (const BackboneBundlePlan& bundle_plan : active_bundle_plans) {
    const BundleCategoryKey candidate_key{bundle_plan.category, bundle_plan.template_id};
    if (!has_route_bundle_category_key ||
        static_cast<int>(candidate_key.category) < static_cast<int>(route_bundle_category_key.category) ||
        (candidate_key.category == route_bundle_category_key.category &&
         static_cast<int>(candidate_key.bundle_template_id) < static_cast<int>(route_bundle_category_key.bundle_template_id))) {
      route_bundle_category_key = candidate_key;
      has_route_bundle_category_key = true;
    }
  }
  auto edge_hash = [](ObjectId node_a, ObjectId node_b) {
    const std::uint64_t lo = static_cast<std::uint64_t>(std::min(node_a, node_b));
    const std::uint64_t hi = static_cast<std::uint64_t>(std::max(node_a, node_b));
    return lo ^ (hi + 0x9E3779B97F4A7C15ull + (lo << 6) + (lo >> 2));
  };
  std::unordered_map<std::uint64_t, BundleCategoryKey> bundle_category_key_by_edge{};
  for (const Span& span : edit_state.spans.items()) {
    const Port* port_a = edit_state.ports.find(span.port_a_id);
    const Port* port_b = edit_state.ports.find(span.port_b_id);
    const Bundle* bundle = edit_state.bundles.find(span.bundle_id);
    if (port_a == nullptr || port_b == nullptr || bundle == nullptr) {
      continue;
    }
    if (port_a->owner_pole_id == kInvalidObjectId || port_b->owner_pole_id == kInvalidObjectId ||
        port_a->owner_pole_id == port_b->owner_pole_id) {
      continue;
    }
    const BundleTemplate* bundle_template = find_bundle_template(bundle->bundle_template_id);
    if (bundle_template == nullptr) {
      continue;
    }
    const BundleCategoryKey candidate_key{bundle_template->category, bundle->bundle_template_id};
    const std::uint64_t edge = edge_hash(port_a->owner_pole_id, port_b->owner_pole_id);
    const auto it_existing = bundle_category_key_by_edge.find(edge);
    if (it_existing == bundle_category_key_by_edge.end()) {
      bundle_category_key_by_edge.emplace(edge, candidate_key);
      continue;
    }
    BundleCategoryKey& current_key = it_existing->second;
    if (static_cast<int>(candidate_key.category) < static_cast<int>(current_key.category) ||
        (candidate_key.category == current_key.category &&
         static_cast<int>(candidate_key.bundle_template_id) < static_cast<int>(current_key.bundle_template_id))) {
      current_key = candidate_key;
    }
  }
  auto bundle_category_key_for_neighbor = [&](ObjectId node_id, ObjectId neighbor_id) {
    BundleCategoryKey key{};
    const auto it = bundle_category_key_by_edge.find(edge_hash(node_id, neighbor_id));
    if (it != bundle_category_key_by_edge.end()) {
      key = it->second;
    }
    return key;
  };
  auto row_layout_axis_mode_for_key = [&](const BundleCategoryKey& key) {
    const BundleTemplate* bundle_template = find_bundle_template(key.bundle_template_id);
    return (bundle_template != nullptr) ? bundle_template->row_layout_axis_mode : RowLayoutAxisMode::kPoleYaw;
  };
  auto row_layout_axis_key_for_node = [&](ObjectId node_id) {
    if (row_layout_axis_mode_for_key(route_bundle_category_key) == RowLayoutAxisMode::kSupportAxis) {
      return route_bundle_category_key;
    }
    for (ObjectId neighbor_id : connected_neighbors_for_support_axis(node_id)) {
      const BundleCategoryKey candidate = bundle_category_key_for_neighbor(node_id, neighbor_id);
      if (row_layout_axis_mode_for_key(candidate) == RowLayoutAxisMode::kSupportAxis) {
        return candidate;
      }
    }
    return BundleCategoryKey{};
  };
  auto neighbor_direction = [&](ObjectId node_id, ObjectId neighbor_id) {
    return normalize_forward_xy_orientation(current_support_position(neighbor_id) - current_support_position(node_id));
  };
  auto choose_support_axis_for_layout = [&](const BackboneOrientationNodeContext& node_context) {
    SupportAxisSelection selection{};
    auto adopt_axis = [&](const Vec3d& axis, PoleSupportAxisRule rule, ObjectId primary_neighbor_id,
                          ObjectId secondary_neighbor_id) {
      Vec3d normalized_axis = normalize_forward_xy_orientation(axis);
      if (!Normalize(&normalized_axis)) {
        return false;
      }
      Vec3d row_axis = ComputeLateralAxis(normalized_axis);
      if (!Normalize(&row_axis)) {
        return false;
      }
      selection.axis = choose_continuous_axis_orientation(row_axis, node_context.previous_support_axis);
      selection.available = Normalize(&selection.axis);
      if (selection.available) {
        selection.rule = rule;
        selection.primary_neighbor_id = primary_neighbor_id;
        selection.secondary_neighbor_id = secondary_neighbor_id;
      }
      return selection.available;
    };
    const ObjectId primary_neighbor_id = node_context.primary_neighbor.neighbor_id;

    if (node_context.continuation_pair.available) {
      if (adopt_axis(node_context.continuation_pair.primary_direction, PoleSupportAxisRule::kMainChainPair,
                     node_context.continuation_pair.primary_neighbor_id,
                     node_context.continuation_pair.secondary_neighbor_id)) {
        return selection;
      }
    }

    if (primary_neighbor_id != kInvalidObjectId && node_context.primary_neighbor.available) {
      const PoleSupportAxisRule rule =
          node_context.has_active_junction ? PoleSupportAxisRule::kPrimaryIncident : PoleSupportAxisRule::kMainChainSingle;
      if (adopt_axis(node_context.primary_neighbor.direction, rule, primary_neighbor_id, kInvalidObjectId)) {
        return selection;
      }
    }

    return selection;
  };

  orientation_context_by_node.reserve(ordered_support_node_ids.size());
  for (ObjectId node_id : ordered_support_node_ids) {
    BackboneOrientationNodeContext context{};
    context.center = current_support_position(node_id);
    if (const Pole* pole = edit_state.poles.find(node_id); pole != nullptr) {
      context.previous_forward = RotateAroundWorldUpDeg(WorldForward(), effective_pole_yaw_deg(*pole));
      context.previous_layout_yaw = effective_pole_layout_yaw_deg(*pole);
      if (const auto it_prev_debug = previous_debug_records.find(pole->id);
          it_prev_debug != previous_debug_records.end()) {
        context.previous_support_axis = it_prev_debug->second.adopted_support_axis;
        context.previous_row_layout_yaw_override = row_layout_yaw_override_from_debug_context(it_prev_debug->second);
      } else {
        context.previous_support_axis = side_axis_from_yaw_deg(context.previous_layout_yaw);
      }
    }
    if (const auto it = junction_relations_by_node.find(node_id); it != junction_relations_by_node.end()) {
      const JunctionRelation& relation = it->second;
      context.has_active_junction = relation.incidents.size() >= 3;
      context.primary_neighbor.neighbor_id = relation.primary_neighbor_id;
      if (relation.through_pair.accepted) {
        context.continuation_pair.primary_neighbor_id = relation.through_pair.neighbor_a_id;
        context.continuation_pair.secondary_neighbor_id = relation.through_pair.neighbor_b_id;
      }
    }
    if (context.primary_neighbor.neighbor_id != kInvalidObjectId) {
      context.primary_neighbor.direction = neighbor_direction(node_id, context.primary_neighbor.neighbor_id);
      context.primary_neighbor.available = Normalize(&context.primary_neighbor.direction);
    }
    if (context.continuation_pair.primary_neighbor_id != kInvalidObjectId &&
        context.continuation_pair.secondary_neighbor_id != kInvalidObjectId) {
      context.continuation_pair.primary_direction =
          neighbor_direction(node_id, context.continuation_pair.primary_neighbor_id);
      context.continuation_pair.secondary_direction =
          neighbor_direction(node_id, context.continuation_pair.secondary_neighbor_id);
      context.continuation_pair.available = Normalize(&context.continuation_pair.primary_direction) &&
                                            Normalize(&context.continuation_pair.secondary_direction);
    }
    const BundleCategoryKey key = row_layout_axis_key_for_node(node_id);
    context.row_layout_axis_selection = {row_layout_axis_mode_for_key(key), key.category};

    const SupportAxisSelection support_axis_selection = choose_support_axis_for_layout(context);
    context.chosen_support_axis = support_axis_selection.axis;
    context.has_chosen_support_axis = support_axis_selection.available;
    context.support_axis_rule = support_axis_selection.rule;
    context.support_axis_primary_neighbor_id = support_axis_selection.primary_neighbor_id;
    context.support_axis_secondary_neighbor_id = support_axis_selection.secondary_neighbor_id;
    orientation_context_by_node.emplace(node_id, std::move(context));
  }
  plan.planned_pole_orientations =
      build_backbone_pole_orientation_plan(ordered_support_node_ids, orientation_context_by_node);

  result.value = std::move(plan);
  result.ok = true;
  return result;
}

void CoreState::apply_committed_backbone_orientation_plan(
    BackboneCommittedGenerationPlan* plan, ChangeSet* change_set) {
  if (plan == nullptr || change_set == nullptr) {
    return;
  }

  debug_.pole_orientation_debug_records.clear();
  BackbonePoleOrientationApplyInput input{
      plan->ordered_support_node_ids,
      plan->planned_pole_orientations,
      debug_.pole_orientation_debug_records,
      [&](ObjectId pole_id) -> Pole* { return edit_state_access().poles.find(pole_id); },
      [&](ObjectId pole_id) -> bool { return has_pole_orientation_override(pole_id); },
      [&](ObjectId pole_id, const Pole& old_pole, const PortLayoutYawOverride* previous_override) {
        refresh_owned_endpoints_from_pole(pole_id, change_set, &old_pole, previous_override);
      },
      [&](ObjectId pole_id, const Pole& old_pole) { finalize_pole_transform_update(pole_id, old_pole, change_set); }};
  apply_backbone_pole_orientation_plan(input);
}

} // namespace wire::core
