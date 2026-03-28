#include "internal_services.hpp"

namespace wire::core::state_internal {

// Runtime override resolution reads only the formal override store.

bool OverrideResolutionService::HasPoleOrientationOverride(const CoreState& state, ObjectId pole_id) {
  return state.authoritative_.override_state.pole_orientation_by_pole.contains(pole_id);
}

bool OverrideResolutionService::HasSpanEndpointSocketOverride(const CoreState& state, ObjectId span_id,
                                                              bool is_start_endpoint) {
  if (const auto it = state.authoritative_.override_state.span_endpoint_by_span.find(span_id);
      it != state.authoritative_.override_state.span_endpoint_by_span.end()) {
    const std::optional<int>& socket = is_start_endpoint ? it->second.socket_a_id : it->second.socket_b_id;
    if (socket.has_value()) {
      return true;
    }
  }
  return false;
}

bool OverrideResolutionService::HasSpanBranchDownOffsetOverride(const CoreState& state, ObjectId span_id) {
  const auto it = state.authoritative_.override_state.span_support_by_span.find(span_id);
  return it != state.authoritative_.override_state.span_support_by_span.end() && it->second.branch_down_offset_m.has_value();
}

std::optional<double> OverrideResolutionService::ResolvePoleManualYawOverride(const CoreState& state, const Pole& pole) {
  if (const auto it = state.authoritative_.override_state.pole_orientation_by_pole.find(pole.id);
      it != state.authoritative_.override_state.pole_orientation_by_pole.end() && it->second.manual_yaw_deg.has_value()) {
    return it->second.manual_yaw_deg;
  }
  return std::nullopt;
}

std::optional<bool> OverrideResolutionService::ResolvePoleFlip180Override(const CoreState& state, const Pole& pole) {
  if (const auto it = state.authoritative_.override_state.pole_orientation_by_pole.find(pole.id);
      it != state.authoritative_.override_state.pole_orientation_by_pole.end() && it->second.flip_180.has_value()) {
    return it->second.flip_180;
  }
  return std::nullopt;
}

int OverrideResolutionService::ResolveSpanEndpointSocketId(const CoreState& state, const Span& span,
                                                           bool is_start_endpoint) {
  if (const auto it = state.authoritative_.override_state.span_endpoint_by_span.find(span.id);
      it != state.authoritative_.override_state.span_endpoint_by_span.end()) {
    const std::optional<int>& socket = is_start_endpoint ? it->second.socket_a_id : it->second.socket_b_id;
    if (socket.has_value()) {
      return *socket;
    }
  }
  return -1;
}

double OverrideResolutionService::ResolveSpanBranchDownOffsetM(const CoreState& state, const Span& span,
                                                               double automatic_value) {
  if (const auto it = state.authoritative_.override_state.span_support_by_span.find(span.id);
      it != state.authoritative_.override_state.span_support_by_span.end() && it->second.branch_down_offset_m.has_value()) {
    return std::max(0.0, *it->second.branch_down_offset_m);
  }
  return automatic_value;
}

} // namespace wire::core::state_internal

