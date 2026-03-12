#pragma once

#include "wire/core/core_state.hpp"

#include <optional>
#include <vector>

namespace wire::core::state_internal {

struct OwnedEndpointIds {
  std::vector<ObjectId> port_ids{};
  std::vector<ObjectId> anchor_ids{};
};

struct OverrideResolutionService {
  static bool HasPoleOrientationOverride(const CoreState& state, ObjectId pole_id);
  static bool HasSpanEndpointSocketOverride(const CoreState& state, ObjectId span_id, bool is_start_endpoint);
  static bool HasSpanBranchDownOffsetOverride(const CoreState& state, ObjectId span_id);
  static std::optional<double> ResolvePoleManualYawOverride(const CoreState& state, const Pole& pole);
  static std::optional<bool> ResolvePoleFlip180Override(const CoreState& state, const Pole& pole);
  static int ResolveSpanEndpointSocketId(const CoreState& state, const Span& span, bool is_start_endpoint);
  static double ResolveSpanBranchDownOffsetM(const CoreState& state, const Span& span, double automatic_value);
};

struct EndpointRefreshService {
  static OwnedEndpointIds CollectOwnedEndpointIds(const CoreState& state, ObjectId pole_id);
  static void RefreshOwnedEndpointsFromPole(CoreState& state, ObjectId pole_id, ChangeSet* change_set,
                                            const Pole* previous_pole = nullptr,
                                            const double* previous_layout_yaw_override = nullptr);
};

} // namespace wire::core::state_internal
