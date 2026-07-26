#pragma once

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "city/wire/entities.hpp"
#include "city/wire/id.hpp"
#include "city/wire/workflow_types.hpp"

namespace city::wire {

struct PoleOrientationOverride {
  std::optional<double> base_yaw_deg{};
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

struct PortResolutionHints {
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  JunctionRelationKind relation_kind = JunctionRelationKind::kNone;
  bool default_lower_required = false;
  bool same_level_feasible = true;
  SameLevelFeasibilityReason same_level_reason = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_m = 0.0;
  double required_clearance_m = 0.0;
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
  PortResolutionHints hints{};
  std::vector<ObjectId> excluded_port_ids{};
};

} // namespace city::wire
