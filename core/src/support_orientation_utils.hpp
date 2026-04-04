#pragma once

#include "wire/core/support_layout_types.hpp"

namespace wire::core {

[[nodiscard]] bool IsFiniteXY(const Vec3d& v);
[[nodiscard]] Vec3d SafeHorizontalNormalized(Vec3d v);
[[nodiscard]] Vec3d CanonicalSharedSupportAxis(Vec3d axis);
[[nodiscard]] Vec3d AuthoritativeSupportAxisForGroup(const SupportGroupDecision& group);
[[nodiscard]] ResolvedSupportAuthority ResolvedSupportAuthorityFromDecision(const EndpointContinuityDecision& decision,
                                                                           int height_rank = -1);

} // namespace wire::core
