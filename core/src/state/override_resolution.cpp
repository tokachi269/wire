#include "internal_services.hpp"

namespace wire::core::state_internal {

bool OverrideResolutionService::HasPoleOrientationOverride(const CoreState& state, ObjectId pole_id) {
  if (state.override_state_.pole_orientation_by_pole.contains(pole_id)) {
    return true;
  }
  const Pole* pole = state.edit_state_.poles.find(pole_id);
  return pole != nullptr &&
         (pole->orientation_control.manual_yaw_override || pole->orientation_control.flip_180 ||
          pole->orientation_override_flag);
}

bool OverrideResolutionService::HasSpanEndpointSocketOverride(const CoreState& state, ObjectId span_id,
                                                              bool is_start_endpoint) {
  if (const auto it = state.override_state_.span_endpoint_by_span.find(span_id);
      it != state.override_state_.span_endpoint_by_span.end()) {
    const std::optional<int>& socket = is_start_endpoint ? it->second.socket_a_id : it->second.socket_b_id;
    if (socket.has_value()) {
      return true;
    }
  }
  const Span* span = state.edit_state_.spans.find(span_id);
  if (span == nullptr) {
    return false;
  }
  return is_start_endpoint ? (span->endpoint_socket_a_id >= 0) : (span->endpoint_socket_b_id >= 0);
}

bool OverrideResolutionService::HasSpanBranchDownOffsetOverride(const CoreState& state, ObjectId span_id) {
  const auto it = state.override_state_.span_support_by_span.find(span_id);
  return it != state.override_state_.span_support_by_span.end() && it->second.branch_down_offset_m.has_value();
}

std::optional<double> OverrideResolutionService::ResolvePoleManualYawOverride(const CoreState& state, const Pole& pole) {
  if (const auto it = state.override_state_.pole_orientation_by_pole.find(pole.id);
      it != state.override_state_.pole_orientation_by_pole.end() && it->second.manual_yaw_deg.has_value()) {
    return it->second.manual_yaw_deg;
  }
  if (pole.orientation_control.manual_yaw_override) {
    return pole.orientation_control.manual_yaw_deg;
  }
  return std::nullopt;
}

std::optional<bool> OverrideResolutionService::ResolvePoleFlip180Override(const CoreState& state, const Pole& pole) {
  if (const auto it = state.override_state_.pole_orientation_by_pole.find(pole.id);
      it != state.override_state_.pole_orientation_by_pole.end() && it->second.flip_180.has_value()) {
    return it->second.flip_180;
  }
  if (pole.orientation_control.flip_180) {
    return true;
  }
  return std::nullopt;
}

int OverrideResolutionService::ResolveSpanEndpointSocketId(const CoreState& state, const Span& span,
                                                           bool is_start_endpoint) {
  if (const auto it = state.override_state_.span_endpoint_by_span.find(span.id);
      it != state.override_state_.span_endpoint_by_span.end()) {
    const std::optional<int>& socket = is_start_endpoint ? it->second.socket_a_id : it->second.socket_b_id;
    if (socket.has_value()) {
      return *socket;
    }
  }
  return is_start_endpoint ? span.endpoint_socket_a_id : span.endpoint_socket_b_id;
}

double OverrideResolutionService::ResolveSpanBranchDownOffsetM(const CoreState& state, const Span& span,
                                                               double automatic_value) {
  if (const auto it = state.override_state_.span_support_by_span.find(span.id);
      it != state.override_state_.span_support_by_span.end() && it->second.branch_down_offset_m.has_value()) {
    return std::max(0.0, *it->second.branch_down_offset_m);
  }
  return automatic_value;
}

} // namespace wire::core::state_internal
