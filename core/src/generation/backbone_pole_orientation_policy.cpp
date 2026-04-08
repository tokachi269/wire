#include "backbone_pole_orientation_policy.hpp"

#include "wire/core/coord_utils.hpp"
#include "detail_utils.hpp"
#include "../pole_orientation_utils.hpp"

#include <cmath>
#include <optional>
#include <unordered_set>

namespace wire::core::generation::detail {

namespace {

Vec3d normalize_forward_xy_policy(const Vec3d& value) {
  Vec3d out{value.x, value.y, 0.0};
  if (!Normalize(&out)) {
    return {};
  }
  return out;
}

Vec3d choose_continuous_axis_policy(const Vec3d& axis, const Vec3d& previous_forward) {
  Vec3d out = axis;
  if (!Normalize(&out)) {
    return {};
  }
  Vec3d prev = normalize_forward_xy_policy(previous_forward);
  if (Dot(out, prev) < 0.0) {
    out = ScaleVec(out, -1.0);
  }
  return out;
}

std::optional<PortLayoutYawOverride> row_layout_yaw_override_from_debug_policy(
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

}  // namespace

void apply_backbone_pole_orientation_policy(const BackbonePoleOrientationPolicyInput& input) {
  std::unordered_set<ObjectId> oriented_poles{};
  for (std::size_t ordered_index = 0; ordered_index < input.ordered_support_node_ids.size(); ++ordered_index) {
    const ObjectId node_id = input.ordered_support_node_ids[ordered_index];
    if (!oriented_poles.insert(node_id).second) {
      continue;
    }
    Pole* pole = input.find_pole(node_id);
    if (pole == nullptr) {
      continue;
    }
    const auto it_context = input.orientation_context_by_node.find(node_id);
    const BackboneOrientationNodeContext node_context =
        (it_context == input.orientation_context_by_node.end()) ? BackboneOrientationNodeContext{} : it_context->second;

    PoleOrientationDebugRecord debug{};
    debug.pole_id = pole->id;
    const Vec3d center = node_context.center;
    const Vec3d previous_forward = node_context.previous_forward;
    const double previous_layout_yaw = node_context.previous_layout_yaw;
    const std::optional<PortLayoutYawOverride> previous_row_layout_yaw_override =
        node_context.previous_row_layout_yaw_override;
    const Vec3d previous_support_axis = node_context.previous_support_axis;
    Vec3d chosen_forward = normalize_forward_xy_policy(previous_forward);
    bool has_chosen_forward = Normalize(&chosen_forward);
    Vec3d chosen_support_axis = input.choose_support_axis_for_layout(node_id, node_context, &debug);
    const bool has_chosen_support_axis = Normalize(&chosen_support_axis);
    if (ordered_index > 0) {
      const ObjectId prev_node_id = input.ordered_support_node_ids[ordered_index - 1];
      if (prev_node_id != node_id) {
        const Pole* prev_pole = input.find_pole(prev_node_id);
        if (prev_pole != nullptr) {
          const auto it_prev_debug = input.debug_records.find(prev_pole->id);
          if (has_chosen_support_axis && it_prev_debug != input.debug_records.end()) {
            Vec3d previous_route_axis = normalize_forward_xy_policy(it_prev_debug->second.adopted_support_axis);
            if (Normalize(&previous_route_axis)) {
              chosen_support_axis = choose_continuous_axis_policy(chosen_support_axis, previous_route_axis);
            }
          }
        }
      }
    }
    const bool apply_main_flow_orientation = node_context.continuation_pair.available || node_context.primary_neighbor.available;
    auto adopt_forward_from_neighbor_pair = [&](ObjectId primary_neighbor_id, ObjectId secondary_neighbor_id,
                                                PoleForwardRule rule) {
      if (!node_context.continuation_pair.available || primary_neighbor_id == kInvalidObjectId ||
          secondary_neighbor_id == kInvalidObjectId || primary_neighbor_id == secondary_neighbor_id) {
        return false;
      }
      Vec3d axis = normalize_forward_xy_policy(node_context.continuation_pair.primary_direction +
                                               node_context.continuation_pair.secondary_direction);
      if (!Normalize(&axis)) {
        axis = normalize_forward_xy_policy(node_context.continuation_pair.primary_direction -
                                           node_context.continuation_pair.secondary_direction);
      }
      if (!Normalize(&axis)) {
        return false;
      }
      chosen_forward = choose_continuous_axis_policy(axis, previous_forward);
      has_chosen_forward = Normalize(&chosen_forward);
      if (has_chosen_forward) {
        debug.rule = rule;
        debug.primary_neighbor_id = primary_neighbor_id;
        debug.secondary_neighbor_id = secondary_neighbor_id;
      }
      return has_chosen_forward;
    };
    auto adopt_forward_from_primary_neighbor = [&](ObjectId primary_neighbor_id, PoleForwardRule rule) {
      if (!node_context.primary_neighbor.available || primary_neighbor_id == kInvalidObjectId) {
        return false;
      }
      Vec3d axis = normalize_forward_xy_policy(node_context.primary_neighbor.direction);
      if (!Normalize(&axis)) {
        return false;
      }
      chosen_forward = choose_continuous_axis_policy(axis, previous_forward);
      has_chosen_forward = true;
      debug.rule = rule;
      debug.primary_neighbor_id = primary_neighbor_id;
      return true;
    };
    if (apply_main_flow_orientation) {
      if (node_context.continuation_pair.available) {
        adopt_forward_from_neighbor_pair(node_context.continuation_pair.primary_neighbor_id,
                                         node_context.continuation_pair.secondary_neighbor_id,
                                         PoleForwardRule::kMainChainBisector);
      } else {
        const ObjectId primary_neighbor_id = node_context.primary_neighbor.neighbor_id;
        if (node_context.primary_neighbor.available) {
          adopt_forward_from_primary_neighbor(primary_neighbor_id,
                                              node_context.has_active_junction ? PoleForwardRule::kPrimaryIncident
                                                                               : PoleForwardRule::kMainChainSingle);
        }
      }
    }

    if (!has_chosen_forward) {
      continue;
    }
    debug.adopted_forward = chosen_forward;
    debug.adopted_support_axis = chosen_support_axis;
    const RowLayoutAxisSelection row_layout_selection = node_context.row_layout_axis_selection;
    debug.row_layout_axis_mode = row_layout_selection.mode;
    debug.row_layout_axis_category = row_layout_selection.category;
    input.debug_records[pole->id] = debug;

    const double next_layout_yaw = previous_layout_yaw;
    const double layout_yaw_delta = normalize_yaw_deg(next_layout_yaw - previous_layout_yaw);
    const std::optional<PortLayoutYawOverride> next_row_layout_yaw_override =
        row_layout_yaw_override_from_debug_policy(debug);
    const bool row_layout_override_changed =
        (!previous_row_layout_yaw_override.has_value() && next_row_layout_yaw_override.has_value()) ||
        (previous_row_layout_yaw_override.has_value() && !next_row_layout_yaw_override.has_value()) ||
        (previous_row_layout_yaw_override.has_value() && next_row_layout_yaw_override.has_value() &&
         (previous_row_layout_yaw_override->category != next_row_layout_yaw_override->category ||
          std::abs(normalize_yaw_deg(next_row_layout_yaw_override->yaw_deg - previous_row_layout_yaw_override->yaw_deg)) >
              1e-6));

    if (input.has_pole_orientation_override(pole->id)) {
      if (std::abs(layout_yaw_delta) > 1e-6 || row_layout_override_changed) {
        const Pole old_pole = *pole;
        input.refresh_owned_endpoints_from_pole(
            pole->id, old_pole,
            previous_row_layout_yaw_override.has_value() ? &*previous_row_layout_yaw_override : nullptr);
      }
      continue;
    }

    const double desired_yaw = normalize_yaw_deg(std::atan2(chosen_forward.y, chosen_forward.x) * (180.0 / kPi));
    double yaw_delta = desired_yaw - pole->world_transform.rotation_euler_deg.z;
    yaw_delta = std::fmod(yaw_delta + 540.0, 360.0) - 180.0;
    if (std::abs(yaw_delta) <= 1e-6) {
      if (std::abs(layout_yaw_delta) > 1e-6 || row_layout_override_changed) {
        const Pole old_pole = *pole;
        input.refresh_owned_endpoints_from_pole(
            pole->id, old_pole,
            previous_row_layout_yaw_override.has_value() ? &*previous_row_layout_yaw_override : nullptr);
      }
      continue;
    }

    const Pole old_pole = *pole;
    pole->world_transform.rotation_euler_deg.z = desired_yaw;
    input.finalize_pole_transform_update(pole->id, old_pole);
  }
}

}  // namespace wire::core::generation::detail
