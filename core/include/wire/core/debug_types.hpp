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

enum class PoleSupportAxisRule : std::uint8_t {
  kFallback = 0,
  kPrimaryIncident = 1,
  kMainChainSingle = 2,
  kMainChainPair = 3,
  kConnectedDirectionFit = 4,
};

struct PoleOrientationDebugRecord {
  ObjectId pole_id = kInvalidObjectId;
  PoleForwardRule rule = PoleForwardRule::kFallback;
  PoleSupportAxisRule support_axis_rule = PoleSupportAxisRule::kFallback;
  ObjectId primary_neighbor_id = kInvalidObjectId;
  ObjectId secondary_neighbor_id = kInvalidObjectId;
  Vec3d adopted_forward{};
  Vec3d adopted_support_axis{};
};

struct SegmentLaneAssignment {
  std::size_t segment_index = 0;
  ObjectId pole_a_id = kInvalidObjectId;
  ObjectId pole_b_id = kInvalidObjectId;
  ObjectId bundle_id = kInvalidObjectId;
  std::uint64_t variation_flow_key = 0;
  BackboneFlowKind flow_kind = BackboneFlowKind::kMain;
  BackboneFlowDecisionRule flow_decision_rule = BackboneFlowDecisionRule::kDefaultMain;
  JunctionRelationKind relation_a = JunctionRelationKind::kNone;
  JunctionRelationKind relation_b = JunctionRelationKind::kNone;
  ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
  bool default_lower_required = false;
  bool lower_propagated_from_run = false;
  bool same_level_feasible = true;
  SameLevelFeasibilityReason same_level_reason = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_m = -1.0;
  double required_clearance_m = 0.0;
  bool lowering_blocked_by_policy = false;
  bool unresolved_same_level_conflict = false;
  bool solver_used_same_level_constraint = false;
  bool used_special_case_ports = false;
  BundleOrderPolicyKind bundle_order_policy = BundleOrderPolicyKind::kFixedOrder;
  BundleOrderChoiceKind bundle_order_choice_a = BundleOrderChoiceKind::kNormal;
  BundleOrderChoiceKind bundle_order_choice_b = BundleOrderChoiceKind::kNormal;
  BundleOrderChoiceReason bundle_order_choice_reason_a = BundleOrderChoiceReason::kFixedOrder;
  BundleOrderChoiceReason bundle_order_choice_reason_b = BundleOrderChoiceReason::kFixedOrder;
  EndpointContinuityDecision decision_a{};
  EndpointContinuityDecision decision_b{};
  std::vector<ObjectId> port_ids_a{};
  std::vector<ObjectId> port_ids_b{};
  bool uses_branch_support = false;
  BackboneLoweringKind lowering_kind = BackboneLoweringKind::kNone;
  double branch_down_offset_m = 0.0;
  SideAssignmentRuleKind side_assignment_rule_a = SideAssignmentRuleKind::kPoleLocal;
  SideAssignmentRuleKind side_assignment_rule_b = SideAssignmentRuleKind::kPoleLocal;
  SupportOrientationRuleKind support_orientation_rule_a = SupportOrientationRuleKind::kRadial;
  SupportOrientationRuleKind support_orientation_rule_b = SupportOrientationRuleKind::kRadial;
  bool used_junction_pair_side_assignment_a = false;
  bool used_junction_pair_side_assignment_b = false;
  bool has_side_axis_a = false;
  bool has_side_axis_b = false;
  Vec3d side_axis_a{};
  Vec3d side_axis_b{};
  double chosen_side_sign_a = 0.0;
  double chosen_side_sign_b = 0.0;
  bool flipped_from_previous = false;
  LaneFlipReason flip_reason = LaneFlipReason::kNone;
  double turn_angle_deg = 0.0;
};

struct PlacementCandidateDebug {
  int candidate_rank = -1;
  int band_id = -1;
  int band_layer = 0;
  SlotSide band_side = SlotSide::kCenter;
  SlotRole band_role = SlotRole::kNeutral;
  double band_lateral_min_m = 0.0;
  double band_lateral_max_m = 0.0;
  double band_height_min_m = 0.0;
  double band_height_max_m = 0.0;
  ObjectId resolved_port_id = kInvalidObjectId;
  double resolved_lateral_m = 0.0;
  double resolved_height_m = 0.0;
  bool min_spacing_satisfied = false;
  bool overflow_used = false;
  ContinuityCategoryClass continuity_class_hint = ContinuityCategoryClass::kPointLike;
  bool default_lower_required_hint = false;
  bool same_level_feasible_hint = true;
  SameLevelFeasibilityReason same_level_reason_hint = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_hint_m = -1.0;
  double required_clearance_hint_m = 0.0;
  JunctionRelationKind relation_kind_hint = JunctionRelationKind::kNone;
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

struct PortResolutionDebugRecord {
  // Session diagnostics for placement-band evaluation and port resolution.
  ObjectId pole_id = kInvalidObjectId;
  ObjectId peer_pole_id = kInvalidObjectId;
  ObjectId reference_span_id = kInvalidObjectId;
  ConnectionCategory category = ConnectionCategory::kLowVoltage;
  ConnectionContext connection_context = ConnectionContext::kTrunkContinue;
  PoleContextKind pole_context = PoleContextKind::kStraight;
  double corner_angle_deg = 0.0;
  double corner_turn_sign = 0.0;
  double side_scale = 1.0;
  ContinuityCategoryClass continuity_class_hint = ContinuityCategoryClass::kPointLike;
  bool default_lower_required_hint = false;
  bool same_level_feasible_hint = true;
  SameLevelFeasibilityReason same_level_reason_hint = SameLevelFeasibilityReason::kNone;
  double projected_spacing_topview_hint_m = -1.0;
  double required_clearance_hint_m = 0.0;
  JunctionRelationKind relation_kind_hint = JunctionRelationKind::kNone;
  ObjectId selected_port_id = kInvalidObjectId;
  bool created_new_port = false;
  bool overflow_triggered = false;
  bool solver_used_same_level_constraint = false;
  bool unresolved_same_level_conflict = false;
  bool fell_back_to_special_case = false;
  std::string result{};
  std::vector<PlacementCandidateDebug> candidates{};
};

} // namespace wire::core
