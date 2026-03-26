#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "wire/core/entities.hpp"
#include "wire/core/id.hpp"
#include "wire/core/workflow_types.hpp"

namespace wire::core {

struct PoleOrientationOverride {
  std::optional<double> manual_yaw_deg{};
  std::optional<bool> flip_180{};
  std::uint64_t version = 1;
};

struct SpanEndpointOverride {
  std::optional<int> socket_a_id{};
  std::optional<int> socket_b_id{};
  std::uint64_t version = 1;
};

struct SpanSupportOverride {
  std::optional<double> branch_down_offset_m{};
  std::uint64_t version = 1;
};

struct OverrideState {
  std::unordered_map<ObjectId, PoleOrientationOverride> pole_orientation_by_pole{};
  std::unordered_map<ObjectId, SpanEndpointOverride> span_endpoint_by_span{};
  std::unordered_map<ObjectId, SpanSupportOverride> span_support_by_span{};
};

struct PortResolutionRequest {
  ObjectId pole_id = kInvalidObjectId;
  ObjectId peer_pole_id = kInvalidObjectId;
  ObjectId reference_span_id = kInvalidObjectId;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  ConnectionContext connection_context = ConnectionContext::kTrunkContinue;
  PoleContextKind pole_context = PoleContextKind::kStraight;
  double corner_angle_deg = 0.0;
  double corner_turn_sign = 0.0;
  bool allow_generate_port = true;
  bool prefer_template_match = false;
  int preferred_template_layer = 1;
  SlotSide preferred_template_side = SlotSide::kCenter;
  SlotRole preferred_template_role = SlotRole::kNeutral;
  std::uint32_t branch_index = 0;
  EndpointContinuityDecision endpoint_decision{};
  std::vector<ObjectId> excluded_port_ids{};
};

} // namespace wire::core
