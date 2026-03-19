#pragma once

#include "wire/core/core_state.hpp"

namespace wire::core {

[[nodiscard]] bool IsFiniteXY(const Vec3d& v);
[[nodiscard]] Vec3d SafeHorizontalNormalized(Vec3d v, const Vec3d& fallback);
[[nodiscard]] Vec3d CanonicalSharedSupportAxis(Vec3d axis, const Vec3d& fallback);
[[nodiscard]] Vec3d AuthoritativeSupportAxisForEndpoint(const SupportLayoutEndpoint& endpoint, const Vec3d& fallback);
[[nodiscard]] Vec3d AuthoritativeSupportAxisForGroup(const LoweredSupportGroupPlacement& group, const Vec3d& fallback);

} // namespace wire::core
