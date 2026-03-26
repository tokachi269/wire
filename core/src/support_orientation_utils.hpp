#pragma once

#include "wire/core/support_layout_types.hpp"

namespace wire::core {

[[nodiscard]] bool IsFiniteXY(const Vec3d& v);
[[nodiscard]] Vec3d SafeHorizontalNormalized(Vec3d v, const Vec3d& fallback);
[[nodiscard]] Vec3d CanonicalSharedSupportAxis(Vec3d axis, const Vec3d& fallback);
[[nodiscard]] Vec3d AuthoritativeSupportAxisForEndpoint(const SupportLayoutEndpoint& endpoint);
[[nodiscard]] Vec3d AuthoritativeSupportAxisForGroup(const LoweredSupportGroupPlacement& group);

} // namespace wire::core
