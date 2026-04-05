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
  if (group.support_authority.has_signed_support_axis) {
    return SafeHorizontalNormalized(group.support_authority.signed_support_axis);
  }
  Vec3d axis = group.side_axis;
  axis.z = 0.0;
  if (!Normalize(&axis) || !IsFiniteXY(axis)) {
    return {0.0, 0.0, 0.0};
  }
  if (std::abs(group.chosen_side_sign) > 1e-9) {
    axis = ScaleVec(axis, (group.chosen_side_sign >= 0.0) ? 1.0 : -1.0);
  }
  return axis;
}

ResolvedSupportAuthority ResolvedSupportAuthorityFromDecision(const SupportLayoutSemanticDecision& decision,
                                                             int height_rank) {
  ResolvedSupportAuthority authority{};
  authority.pair.pair_peer_low = decision.support_pair_peer_low;
  authority.pair.pair_peer_high = decision.support_pair_peer_high;
  authority.pair.orientation_basis =
      SupportOrientationBasisFromDecision(decision.support_orientation_rule, decision.chosen_side_sign);
  authority.pair.height_rank = height_rank;
  authority.pair.pair_axis = decision.side_axis;
  authority.pair.has_pair_axis = decision.has_side_axis;
  if (decision.has_side_axis && std::abs(decision.chosen_side_sign) > 1e-9) {
    authority.signed_support_axis = decision.side_axis;
    authority.signed_support_axis.z = 0.0;
    authority.signed_support_axis =
        ScaleVec(authority.signed_support_axis, (decision.chosen_side_sign >= 0.0) ? 1.0 : -1.0);
    authority.has_signed_support_axis = true;
  }
  return authority;
}

ResolvedSupportAuthority ResolvedSupportAuthorityFromDecision(const EndpointContinuityDecision& decision,
                                                             int height_rank) {
  return ResolvedSupportAuthorityFromDecision(MakeSupportLayoutSemanticDecision(decision), height_rank);
}

} // namespace wire::core
