#pragma once

#include "backbone_seed_topology.hpp"

#include <unordered_map>

namespace wire::core::generation::detail {

struct SeedDecisionPlacementProjection {
  bool lower_required = false;
  bool default_lower_required = false;
  bool same_level_feasible = false;
  SameLevelSupportReason same_level_reason = SameLevelSupportReason::kUnknown;
  double projected_spacing_topview_m = 0.0;
  double required_clearance_m = 0.0;
  bool lowering_blocked_by_policy = false;
  bool unresolved_same_level_conflict = false;
  bool solver_used_same_level_constraint = false;
  bool used_special_case_ports = false;
  OrderDecisionPolicyKind order_decision_policy = OrderDecisionPolicyKind::kFixedOrder;
  OrderDecisionChoiceKind order_decision_choice = OrderDecisionChoiceKind::kDefault;
  OrderDecisionChoiceReason order_decision_choice_reason = OrderDecisionChoiceReason::kDefault;
  int support_group_id = -1;
  SideAssignmentRuleKind side_assignment_rule = SideAssignmentRuleKind::kPoleLocal;
  SupportOrientationRuleKind support_orientation_rule = SupportOrientationRuleKind::kRadial;
  SupportOrientationBasisKind support_orientation_basis = SupportOrientationBasisKind::kRadial;
  LateralSideChoice chosen_side = LateralSideChoice::kCenter;
  bool used_junction_pair_side_assignment = false;
  bool has_side_axis = false;
  Vec3d side_axis{0.0, 0.0, 0.0};
  double chosen_side_sign = 0.0;
};

[[nodiscard]] SeedDecisionPlacementProjection
build_seed_placement_projection(const EndpointContinuityDecision& decision);

void apply_seed_placement_projection(const SeedDecisionPlacementProjection& projection,
                                     EndpointContinuityDecision* decision);

[[nodiscard]] int pair_height_rank_from_decision(
    const EditState& edit_state, const std::unordered_map<ObjectId, JunctionRelation>& junction_relations_by_node,
    ObjectId endpoint_node_id, const EndpointContinuityDecision& decision);

[[nodiscard]] SupportLayoutDecisionSeedEndpoint build_seed_endpoint_from_decision(
    const EditState& edit_state, const std::unordered_map<ObjectId, JunctionRelation>& junction_relations_by_node,
    const SegmentLaneAssignment& assignment, ObjectId endpoint_node_id, const Port& port,
    const EndpointContinuityDecision& decision);

} // namespace wire::core::generation::detail
