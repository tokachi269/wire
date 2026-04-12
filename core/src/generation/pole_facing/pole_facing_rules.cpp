#include "pole_facing_rules.hpp"

#include "wire/core/coord_utils.hpp"
#include "../detail_utils.hpp"
#include "../../pole_orientation_utils.hpp"

#include <cmath>
#include <unordered_set>

namespace wire::core::generation::detail {

void apply_backbone_pole_orientation_plan(const BackbonePoleOrientationApplyInput& input) {
  std::unordered_set<ObjectId> oriented_poles{};
  for (ObjectId node_id : input.ordered_support_node_ids) {
    if (!oriented_poles.insert(node_id).second) {
      continue;
    }
    Pole* pole = input.find_pole(node_id);
    if (pole == nullptr) {
      continue;
    }
    const auto it_planned = input.planned_orientations_by_node.find(node_id);
    if (it_planned == input.planned_orientations_by_node.end()) {
      continue;
    }
    const BackbonePlannedPoleOrientation& planned = it_planned->second;
    input.debug_records[pole->id] = planned.debug;
    if (!planned.has_adopted_forward) {
      continue;
    }

    if (input.has_pole_orientation_override(pole->id)) {
      continue;
    }

    const double desired_yaw = normalize_yaw_deg(std::atan2(planned.adopted_forward.y, planned.adopted_forward.x) *
                                                 (180.0 / kPi));
    double yaw_delta = desired_yaw - pole->world_transform.rotation_euler_deg.z;
    yaw_delta = std::fmod(yaw_delta + 540.0, 360.0) - 180.0;
    if (std::abs(yaw_delta) <= 1e-6) {
      continue;
    }

    const Pole old_pole = *pole;
    pole->world_transform.rotation_euler_deg.z = desired_yaw;
    input.finalize_pole_transform_update(pole->id, old_pole);
  }
}

}  // namespace wire::core::generation::detail
