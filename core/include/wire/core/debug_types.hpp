#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "wire/core/entities.hpp"
#include "wire/core/workflow_types.hpp"

namespace wire::core {

struct PathDirectionCostBreakdown {
  int estimated_cross_penalty = 0;
  int side_flip_penalty = 0;
  int layer_jump_penalty = 0;
  int corner_compression_penalty = 0;
  int branch_conflict_penalty = 0;
  int total = 0;
};

struct PathDirectionEvaluationDebug {
  RoadId road_id = 0;
  PathDirectionMode requested_mode = PathDirectionMode::kAuto;
  PathDirectionChosen chosen = PathDirectionChosen::kForward;
  PathDirectionCostBreakdown forward_cost{};
  PathDirectionCostBreakdown reverse_cost{};
  std::string reason{};
};

enum class PoleForwardRule : std::uint8_t {
  kFallback = 0,
  kPrimaryIncident = 1,
  kMainChainSingle = 2,
  kMainChainBisector = 3,
};

struct PoleOrientationDebugRecord {
  ObjectId pole_id = kInvalidObjectId;
  PoleForwardRule rule = PoleForwardRule::kFallback;
  ObjectId primary_neighbor_id = kInvalidObjectId;
  ObjectId secondary_neighbor_id = kInvalidObjectId;
  Vec3d adopted_forward{};
};

struct SegmentLaneAssignment {
  std::size_t segment_index = 0;
  ObjectId pole_a_id = kInvalidObjectId;
  ObjectId pole_b_id = kInvalidObjectId;
  ObjectId bundle_id = kInvalidObjectId;
  std::uint64_t variation_flow_key = 0;
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  BackboneFlowDecisionRule flow_decision_rule = BackboneFlowDecisionRule::kDefaultMain;
  std::vector<int> slot_ids_a{};
  std::vector<int> slot_ids_b{};
  std::vector<ObjectId> port_ids_a{};
  std::vector<ObjectId> port_ids_b{};
  bool uses_branch_support = false;
  double branch_down_offset_m = 0.0;
  bool mirrored = false;
  bool flipped_from_previous = false;
  LaneFlipReason flip_reason = LaneFlipReason::kNone;
  double turn_angle_deg = 0.0;
};

struct SlotCandidateDebug {
  int slot_id = -1;
  bool eligible = false;
  int total_score = 0;
  int category_score = 0;
  int context_score = 0;
  int layer_score = 0;
  int side_score = 0;
  int role_score = 0;
  int priority_score = 0;
  int usage_score = 0;
  int congestion_score = 0;
  int tie_breaker = 0;
  std::size_t usage_count = 0;
  std::size_t congestion_count = 0;
  std::string reason{};
};

struct SlotSelectionDebugRecord {
  // Session diagnostics for slot selection; not part of entity model.
  ObjectId pole_id = kInvalidObjectId;
  ObjectId peer_pole_id = kInvalidObjectId;
  ObjectId reference_span_id = kInvalidObjectId;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  ConnectionContext connection_context = ConnectionContext::kTrunkContinue;
  PoleContextKind pole_context = PoleContextKind::kStraight;
  double corner_angle_deg = 0.0;
  double corner_turn_sign = 0.0;
  double side_scale = 1.0;
  int selected_slot_id = -1;
  std::string result{};
  std::vector<SlotCandidateDebug> candidates{};
};

} // namespace wire::core
