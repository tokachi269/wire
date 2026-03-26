#include "support_orientation_utils.hpp"

#include "wire/core/coord_utils.hpp"

#include <cmath>

namespace wire::core {
bool IsFiniteXY(const Vec3d& v) {
  return std::isfinite(v.x) && std::isfinite(v.y);
}

Vec3d SafeHorizontalNormalized(Vec3d v) {
  v.z = 0.0;
  if (Normalize(&v) && IsFiniteXY(v)) {
    return v;
  }
  return {0.0, 0.0, 0.0};
}

Vec3d CanonicalSharedSupportAxis(Vec3d axis) {
  axis = SafeHorizontalNormalized(axis);
  if (!IsFiniteXY(axis) || (std::abs(axis.x) <= 1e-9 && std::abs(axis.y) <= 1e-9)) {
    return {0.0, 0.0, 0.0};
  }
  if (axis.x < -1e-9 || (std::abs(axis.x) <= 1e-9 && axis.y < -1e-9)) {
    axis = ScaleVec(axis, -1.0);
  }
  return axis;
}

Vec3d AuthoritativeSupportAxisForGroup(const SupportGroupDecision& group) {
  Vec3d axis = group.decision.side_axis;
  axis.z = 0.0;
  if (!Normalize(&axis) || !IsFiniteXY(axis)) {
    return {0.0, 0.0, 0.0};
  }
  if (std::abs(group.decision.chosen_side_sign) > 1e-9) {
    axis = ScaleVec(axis, (group.decision.chosen_side_sign >= 0.0) ? 1.0 : -1.0);
  }
  return axis;
}

} // namespace wire::core
