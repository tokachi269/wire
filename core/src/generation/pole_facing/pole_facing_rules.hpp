#pragma once

#include "wire/core/core_state.hpp"
#include "wire/core/debug_types.hpp"
#include "wire/core/id.hpp"

#include <functional>
#include <unordered_map>
#include <vector>

namespace wire::core {

namespace generation::detail {

struct BackbonePlannedPoleOrientation {
  Vec3d adopted_forward{};
  bool has_adopted_forward = false;
  PoleForwardRule forward_rule = PoleForwardRule::kFallback;
  ObjectId forward_primary_neighbor_id = kInvalidObjectId;
  ObjectId forward_secondary_neighbor_id = kInvalidObjectId;
  PoleOrientationDebugRecord debug{};
};

struct BackbonePoleOrientationApplyInput {
  const std::vector<ObjectId>& ordered_support_node_ids;
  const std::unordered_map<ObjectId, BackbonePlannedPoleOrientation>& planned_orientations_by_node;
  std::unordered_map<ObjectId, PoleOrientationDebugRecord>& debug_records;
  std::function<Pole*(ObjectId)> find_pole;
  std::function<bool(ObjectId)> has_pole_orientation_override;
  std::function<void(ObjectId, const Pole&)> finalize_pole_transform_update;
};

void apply_backbone_pole_orientation_plan(const BackbonePoleOrientationApplyInput& input);

}  // namespace generation::detail
}  // namespace wire::core
