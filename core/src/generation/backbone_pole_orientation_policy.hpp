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

struct BackboneOrientationNeighborContext {
  ObjectId neighbor_id = kInvalidObjectId;
  Vec3d direction{};
  bool available = false;
};

struct BackboneOrientationNeighborPairContext {
  ObjectId primary_neighbor_id = kInvalidObjectId;
  ObjectId secondary_neighbor_id = kInvalidObjectId;
  Vec3d primary_direction{};
  Vec3d secondary_direction{};
  bool available = false;
};

struct BackboneOrientationNodeContext {
  Vec3d center{};
  Vec3d previous_forward{};
  double previous_layout_yaw = 0.0;
  std::optional<PortLayoutYawOverride> previous_row_layout_yaw_override{};
  Vec3d previous_support_axis{};
  Vec3d chosen_support_axis{};
  bool has_chosen_support_axis = false;
  PoleSupportAxisRule support_axis_rule = PoleSupportAxisRule::kFallback;
  ObjectId support_axis_primary_neighbor_id = kInvalidObjectId;
  ObjectId support_axis_secondary_neighbor_id = kInvalidObjectId;
  bool has_active_junction = false;
  BackboneOrientationNeighborContext primary_neighbor{};
  BackboneOrientationNeighborPairContext continuation_pair{};
  RowLayoutAxisSelection row_layout_axis_selection{};
};

struct BackbonePoleOrientationPolicyInput {
  const std::vector<ObjectId>& ordered_support_node_ids;
  const std::unordered_map<ObjectId, BackboneOrientationNodeContext>& orientation_context_by_node;
  std::unordered_map<ObjectId, PoleOrientationDebugRecord>& debug_records;
  std::function<Pole*(ObjectId)> find_pole;
  std::function<Vec3d(ObjectId, const BackboneOrientationNodeContext&, PoleOrientationDebugRecord*)>
      choose_support_axis_for_layout;
  std::function<bool(ObjectId)> has_pole_orientation_override;
  std::function<void(ObjectId, const Pole&, const PortLayoutYawOverride*)> refresh_owned_endpoints_from_pole;
  std::function<void(ObjectId, const Pole&)> finalize_pole_transform_update;
};

void apply_backbone_pole_orientation_policy(const BackbonePoleOrientationPolicyInput& input);

}  // namespace generation::detail
}  // namespace wire::core
