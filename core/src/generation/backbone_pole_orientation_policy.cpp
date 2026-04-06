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
  constexpr double kMainBisectorSupportAxisMaxForwardAlignment = 0.85;
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

    PoleOrientationDebugRecord debug{};
    debug.pole_id = pole->id;
    const Vec3d center = input.current_support_position(node_id);
    const Vec3d previous_forward = RotateAroundWorldUpDeg(WorldForward(), input.effective_pole_yaw_deg(*pole));
    const double previous_layout_yaw = input.effective_pole_layout_yaw_deg(*pole);
    std::optional<PortLayoutYawOverride> previous_row_layout_yaw_override{};
    Vec3d previous_support_axis{};
    if (const auto it_prev_debug = input.previous_debug_records.find(pole->id);
        it_prev_debug != input.previous_debug_records.end()) {
      previous_support_axis = it_prev_debug->second.adopted_support_axis;
      previous_row_layout_yaw_override = row_layout_yaw_override_from_debug_policy(it_prev_debug->second);
    } else {
      previous_support_axis = side_axis_from_yaw_deg(previous_layout_yaw);
    }
    Vec3d chosen_forward = normalize_forward_xy_policy(previous_forward);
    bool has_chosen_forward = Normalize(&chosen_forward);
    Vec3d chosen_support_axis = input.choose_support_axis_for_layout(node_id, center, previous_support_axis, &debug);
    if (!Normalize(&chosen_support_axis)) {
      chosen_support_axis = normalize_forward_xy_policy(previous_support_axis);
    }
    if (ordered_index > 0) {
      const ObjectId prev_node_id = input.ordered_support_node_ids[ordered_index - 1];
      if (prev_node_id != node_id) {
        const Pole* prev_pole = input.find_pole(prev_node_id);
        if (prev_pole != nullptr) {
          const auto it_prev_debug = input.debug_records.find(prev_pole->id);
          if (it_prev_debug != input.debug_records.end()) {
            Vec3d previous_route_axis = normalize_forward_xy_policy(it_prev_debug->second.adopted_support_axis);
            if (Normalize(&previous_route_axis)) {
              chosen_support_axis = choose_continuous_axis_policy(chosen_support_axis, previous_route_axis);
            }
          }
        }
      }
    }
    const bool apply_main_flow_orientation = input.has_existing_main_flow_context(node_id);
    auto adopt_forward_from_neighbor_pair = [&](ObjectId primary_neighbor_id, ObjectId secondary_neighbor_id,
                                                PoleForwardRule rule) {
      if (primary_neighbor_id == kInvalidObjectId || secondary_neighbor_id == kInvalidObjectId ||
          primary_neighbor_id == secondary_neighbor_id) {
        return false;
      }
      const Vec3d dir_a = normalize_forward_xy_policy(input.current_support_position(primary_neighbor_id) - center);
      const Vec3d dir_b = normalize_forward_xy_policy(input.current_support_position(secondary_neighbor_id) - center);
      Vec3d axis = normalize_forward_xy_policy(dir_a + dir_b);
      if (!Normalize(&axis)) {
        axis = normalize_forward_xy_policy(dir_a - dir_b);
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
      if (primary_neighbor_id == kInvalidObjectId) {
        return false;
      }
      Vec3d axis = normalize_forward_xy_policy(input.current_support_position(primary_neighbor_id) - center);
      if (!Normalize(&axis)) {
        return false;
      }
      chosen_forward = choose_continuous_axis_policy(axis, previous_forward);
      has_chosen_forward = true;
      debug.rule = rule;
      debug.primary_neighbor_id = primary_neighbor_id;
      return true;
    };
    auto adopt_forward_from_support_axis_selection = [&]() {
      switch (debug.support_axis_rule) {
      case PoleSupportAxisRule::kMainChainPair:
        if (adopt_forward_from_neighbor_pair(debug.primary_neighbor_id, debug.secondary_neighbor_id,
                                             PoleForwardRule::kMainChainBisector)) {
          return true;
        }
        return adopt_forward_from_primary_neighbor(debug.primary_neighbor_id, PoleForwardRule::kMainChainSingle);
      case PoleSupportAxisRule::kPrimaryIncident:
        return adopt_forward_from_primary_neighbor(debug.primary_neighbor_id, PoleForwardRule::kPrimaryIncident);
      case PoleSupportAxisRule::kMainChainSingle:
        return adopt_forward_from_primary_neighbor(debug.primary_neighbor_id, PoleForwardRule::kMainChainSingle);
      case PoleSupportAxisRule::kConnectedDirectionFit:
      case PoleSupportAxisRule::kFallback:
      default:
        return false;
      }
    };

    if (apply_main_flow_orientation) {
      const std::vector<ObjectId> continuation_neighbors = input.continuation_neighbors_for_orientation(node_id);
      if (continuation_neighbors.size() >= 2) {
        adopt_forward_from_neighbor_pair(continuation_neighbors[0], continuation_neighbors[1],
                                         PoleForwardRule::kMainChainBisector);
      } else {
        const ObjectId primary_neighbor_id = input.primary_neighbor_for_orientation(node_id);
        if (primary_neighbor_id != kInvalidObjectId) {
          adopt_forward_from_primary_neighbor(primary_neighbor_id,
                                              input.has_active_junction(node_id) ? PoleForwardRule::kPrimaryIncident
                                                                                 : PoleForwardRule::kMainChainSingle);
        }
      }
    }

    if (!has_chosen_forward && apply_main_flow_orientation) {
      adopt_forward_from_support_axis_selection();
    }

    if (!has_chosen_forward) {
      chosen_forward = normalize_forward_xy_policy(previous_forward);
      debug.rule = PoleForwardRule::kFallback;
    }
    if (debug.rule == PoleForwardRule::kMainChainBisector) {
      bool preserve_pair_axis = false;
      if (debug.support_axis_rule == PoleSupportAxisRule::kMainChainPair && debug.primary_neighbor_id != kInvalidObjectId) {
        Vec3d preserved_pair_axis = normalize_forward_xy_policy(input.current_support_position(debug.primary_neighbor_id) - center);
        if (Normalize(&preserved_pair_axis)) {
          preserved_pair_axis = ComputeLateralAxis(preserved_pair_axis);
          if (Normalize(&preserved_pair_axis)) {
            preserved_pair_axis = choose_continuous_axis_policy(preserved_pair_axis, previous_support_axis);
            if (Normalize(&preserved_pair_axis)) {
              preserve_pair_axis = std::abs(Dot(preserved_pair_axis, chosen_support_axis)) >= 0.95;
            }
          }
        }
      }
      if (!preserve_pair_axis) {
        Vec3d normalized_support_axis = normalize_forward_xy_policy(chosen_support_axis);
        if (Normalize(&normalized_support_axis)) {
          const double forward_alignment = std::abs(Dot(normalized_support_axis, chosen_forward));
          if (forward_alignment > kMainBisectorSupportAxisMaxForwardAlignment) {
            Vec3d lateral_axis = ComputeLateralAxis(chosen_forward);
            if (Normalize(&lateral_axis)) {
              chosen_support_axis = choose_continuous_axis_policy(lateral_axis, normalized_support_axis);
              if (Normalize(&chosen_support_axis) && debug.secondary_neighbor_id != kInvalidObjectId) {
                debug.support_axis_rule = PoleSupportAxisRule::kMainChainPair;
              }
            }
          }
        }
      }
    }
    debug.adopted_forward = chosen_forward;
    debug.adopted_support_axis = chosen_support_axis;
    const RowLayoutAxisSelection row_layout_selection = input.row_layout_axis_selection_for_node(node_id);
    debug.row_layout_axis_mode = row_layout_selection.mode;
    debug.row_layout_axis_category = row_layout_selection.category;
    input.debug_records[pole->id] = debug;

    const double next_layout_yaw = input.effective_pole_layout_yaw_deg(*pole);
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

    if (!has_chosen_forward || input.has_pole_orientation_override(pole->id)) {
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
