#pragma once

#include "wire/core/core_state.hpp"

namespace wire::core {

[[nodiscard]] bool IsFiniteXY(const Vec3d& v);
[[nodiscard]] bool UsesGroupedLoweredSupport(const SupportLayoutEndpoint& endpoint, BackboneLoweringKind lowering_kind);
[[nodiscard]] int SupportGroupIdForEndpoint(ObjectId owner_pole_id, const EndpointContinuityDecision& decision);
[[nodiscard]] Vec3d SafeHorizontalNormalized(Vec3d v, const Vec3d& fallback);
[[nodiscard]] Vec3d CanonicalSharedSupportAxis(Vec3d axis, const Vec3d& fallback);
[[nodiscard]] Vec3d AuthoritativeSupportAxisForEndpoint(const SupportLayoutEndpoint& endpoint, const Vec3d& fallback);
[[nodiscard]] Vec3d AuthoritativeSupportAxisForGroup(const LoweredSupportGroupPlacement& group, const Vec3d& fallback);
[[nodiscard]] Vec3d ResolveSupportAxisForEndpoint(const Port& port, const Span& span, const Port& other_port,
                                                  const Pole& pole, const SupportLayoutEndpoint* layout_endpoint,
                                                  const EditState& edit_state);

} // namespace wire::core