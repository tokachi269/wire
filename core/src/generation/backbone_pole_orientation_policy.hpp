#pragma once

#include "wire/core/core_state.hpp"
#include "wire/core/debug_types.hpp"
#include "wire/core/id.hpp"

#include <functional>
#include <optional>
#include <unordered_map>
#include <vector>

namespace wire::core {

namespace generation::detail {

struct RowLayoutAxisSelection {
  RowLayoutAxisMode mode = RowLayoutAxisMode::kPoleYaw;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
};

struct BackbonePoleOrientationPolicyInput {
  const std::vector<ObjectId>& ordered_support_node_ids;
  std::unordered_map<ObjectId, PoleOrientationDebugRecord>& debug_records;
  const std::unordered_map<ObjectId, PoleOrientationDebugRecord>& previous_debug_records;
  std::function<Pole*(ObjectId)> find_pole;
  std::function<Vec3d(ObjectId)> current_support_position;
  std::function<Vec3d(ObjectId, const Vec3d&, const Vec3d&, PoleOrientationDebugRecord*)> choose_support_axis_for_layout;
  std::function<bool(ObjectId)> has_existing_main_flow_context;
  std::function<std::vector<ObjectId>(ObjectId)> continuation_neighbors_for_orientation;
  std::function<ObjectId(ObjectId)> primary_neighbor_for_orientation;
  std::function<bool(ObjectId)> has_active_junction;
  std::function<RowLayoutAxisSelection(ObjectId)> row_layout_axis_selection_for_node;
  std::function<bool(ObjectId)> has_pole_orientation_override;
  std::function<double(const Pole&)> effective_pole_yaw_deg;
  std::function<double(const Pole&)> effective_pole_layout_yaw_deg;
  std::function<void(ObjectId, const Pole&, const PortLayoutYawOverride*)> refresh_owned_endpoints_from_pole;
  std::function<void(ObjectId, const Pole&)> finalize_pole_transform_update;
};

void apply_backbone_pole_orientation_policy(const BackbonePoleOrientationPolicyInput& input);

}  // namespace generation::detail
}  // namespace wire::core
