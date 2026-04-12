#include "build_span_layout_rules.hpp"

#include "../detail_utils.hpp"
#include "../support_policy.hpp"
#include "../../support_orientation_utils.hpp"

#include <unordered_map>
#include <vector>

namespace wire::core::generation::detail {

void apply_seed_topology_projection(const EndpointContinuityDecision& source, ObjectId owner_pole_id,
                                    SupportLayoutSemanticDecision* decision) {
  if (decision == nullptr) {
    return;
  }
  decision->owner_pole_id = owner_pole_id;
  decision->relation_kind = source.relation_kind;
  decision->continuity_class = source.continuity_class;
  decision->in_through_pair = source.in_through_pair;
  decision->support_pair_peer_low = source.support_pair_peer_low;
  decision->support_pair_peer_high = source.support_pair_peer_high;
}

namespace {

SupportLayoutOriginKind support_layout_origin_from_port_source(PortPlacementSourceKind source) {
  switch (source) {
  case PortPlacementSourceKind::kBranchSupport:
    return SupportLayoutOriginKind::kBranchSupport;
  case PortPlacementSourceKind::kAerialBranch:
    return SupportLayoutOriginKind::kAerialBranch;
  case PortPlacementSourceKind::kPlacementBandConstrained:
    return SupportLayoutOriginKind::kPlacementConstraint;
  case PortPlacementSourceKind::kPlacementBand:
  case PortPlacementSourceKind::kGenerated:
  case PortPlacementSourceKind::kManualEdit:
  case PortPlacementSourceKind::kUnknown:
  default:
    return SupportLayoutOriginKind::kMainSupport;
  }
}

using PairKey = std::pair<ObjectId, ObjectId>;

struct PairKeyHash {
  [[nodiscard]] std::size_t operator()(const PairKey& key) const {
    const std::size_t h1 = std::hash<ObjectId>{}(key.first);
    const std::size_t h2 = std::hash<ObjectId>{}(key.second);
    return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
  }
};

[[nodiscard]] double distance_squared_xy(const Vec3d& a, const Vec3d& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

[[nodiscard]] PairKey canonical_pair_key(ObjectId a, ObjectId b) {
  return {std::min(a, b), std::max(a, b)};
}

std::unordered_map<PairKey, int, PairKeyHash> pair_height_rank_map_for_junction(const EditState& edit_state,
                                                                                const JunctionRelation& relation) {
  std::unordered_map<PairKey, int, PairKeyHash> ranks{};
  PairKey through_key{};
  bool has_through_key = false;
  if (relation.through_pair.accepted) {
    through_key = canonical_pair_key(relation.through_pair.neighbor_a_id, relation.through_pair.neighbor_b_id);
    ranks.emplace(through_key, 0);
    has_through_key = true;
  }
  (void)edit_state;
  return ranks;
}

} // namespace

void apply_seed_placement_projection(const EndpointContinuityDecision& source,
                                     SupportLayoutSemanticDecision* decision) {
  if (decision == nullptr) {
    return;
  }
  decision->lower_required = source.lower_required;
  decision->lowering_blocked_by_policy = source.lowering_blocked_by_policy;
  decision->support_group_id = source.support_group_id;
  decision->side_assignment_rule = source.side_assignment_rule;
  decision->support_orientation_rule = source.support_orientation_rule;
  decision->support_orientation_basis = source.support_orientation_basis;
  decision->has_side_axis = source.has_side_axis;
  decision->side_axis = source.side_axis;
  decision->chosen_side_sign = source.chosen_side_sign;
}

int pair_height_rank_from_decision(const EditState& edit_state,
                                   const std::unordered_map<ObjectId, JunctionRelation>& junction_relations_by_node,
                                   ObjectId endpoint_node_id, const EndpointContinuityDecision& decision) {
  if (UsesAuthoritativeGroupedLoweredSupport(decision) || !HasAuthoritativeSupportPair(decision)) {
    return -1;
  }
  const auto it_relation = junction_relations_by_node.find(endpoint_node_id);
  if (it_relation == junction_relations_by_node.end()) {
    if (decision.in_through_pair || decision.relation_kind == JunctionRelationKind::kThroughMain) {
      return 0;
    }
    return (decision.relation_kind == JunctionRelationKind::kNone) ? -1 : 1;
  }

  const JunctionRelation& relation = it_relation->second;
  if (relation.through_pair.accepted &&
      (decision.in_through_pair || decision.relation_kind == JunctionRelationKind::kThroughMain)) {
    return 0;
  }

  const auto ranks = pair_height_rank_map_for_junction(edit_state, relation);
  const PairKey decision_key = canonical_pair_key(decision.support_pair_peer_low, decision.support_pair_peer_high);
  if (const auto it_rank = ranks.find(decision_key); it_rank != ranks.end()) {
    if (relation.through_pair.accepted && it_rank->second == 0 &&
        decision.relation_kind != JunctionRelationKind::kThroughMain && !decision.in_through_pair) {
      return 1;
    }
    return it_rank->second;
  }
  return (decision.relation_kind == JunctionRelationKind::kNone) ? -1 : (relation.through_pair.accepted ? 1 : 0);
}

SupportLayoutDecisionSeedEndpoint build_seed_endpoint_from_decision(
    const EditState& edit_state, const std::unordered_map<ObjectId, JunctionRelation>& junction_relations_by_node,
    const SegmentLaneAssignment& assignment, ObjectId endpoint_node_id, const Port& port,
    const EndpointContinuityDecision& decision) {
  SupportLayoutDecisionSeedEndpoint endpoint{};
  static_cast<SupportLayoutSemanticDecision&>(endpoint) = MakeSupportLayoutSemanticDecision(decision, port.owner_pole_id);
  endpoint.endpoint_node_id = endpoint_node_id;
  endpoint.port_id = port.id;
  const int pair_height_rank =
      pair_height_rank_from_decision(edit_state, junction_relations_by_node, endpoint.endpoint_node_id, decision);
  endpoint.support_authority = ResolvedSupportAuthorityFromDecision(decision, pair_height_rank);
  endpoint.flow_kind = assignment.flow_kind;
  endpoint.origin = support_layout_origin_from_port_source(port.placement_source);
  endpoint.endpoint_source = SupportLayoutEndpointSourceKind::kPlainSupport;
  endpoint.port_source = port.placement_source;
  endpoint.side = port.template_side;
  endpoint.endpoint_mode = CurveEndpointMode::kDirectThrough;
  endpoint.default_lower_required = decision.default_lower_required;
  endpoint.same_level_feasible = decision.same_level_feasible;
  endpoint.unresolved_same_level_conflict = decision.unresolved_same_level_conflict;
  endpoint.same_level_reason = decision.same_level_reason;
  endpoint.projected_spacing_topview_m = decision.projected_spacing_topview_m;
  endpoint.required_clearance_m = decision.required_clearance_m;
  endpoint.solver_used_same_level_constraint = decision.solver_used_same_level_constraint;
  endpoint.used_special_case_ports = decision.used_special_case_ports;
  endpoint.order_decision_policy = decision.order_decision_policy;
  endpoint.order_decision_choice = decision.order_decision_choice;
  endpoint.order_decision_choice_reason = decision.order_decision_choice_reason;
  endpoint.chosen_side = decision.chosen_side;
  endpoint.used_junction_pair_side_assignment = decision.used_junction_pair_side_assignment;
  const bool uses_lowering = decision.lower_required && !decision.lowering_blocked_by_policy;
  const bool uses_pair_height = !uses_lowering && pair_height_rank > 0 && decision.same_level_feasible &&
                                HasAuthoritativeSupportPair(decision);
  const double pair_height_step_m = BranchDownOffsetForCategory(port.category);
  endpoint.automatic_branch_down_offset_m =
      uses_lowering ? assignment.branch_down_offset_m : (uses_pair_height ? pair_height_step_m * pair_height_rank : 0.0);
  endpoint.branch_down_offset_m = endpoint.automatic_branch_down_offset_m;
  return endpoint;
}

EndpointLayoutRule build_endpoint_layout_rule_from_decision(
    const EditState& edit_state, const std::unordered_map<ObjectId, JunctionRelation>& junction_relations_by_node,
    const SegmentLaneAssignment& assignment, ObjectId endpoint_node_id, const Port& port,
    const EndpointContinuityDecision& decision) {
  EndpointLayoutRule endpoint{};
  endpoint.semantic = MakeSupportLayoutSemanticDecision(decision, port.owner_pole_id);
  endpoint.endpoint_node_id = endpoint_node_id;
  endpoint.port_id = port.id;
  const int pair_height_rank =
      pair_height_rank_from_decision(edit_state, junction_relations_by_node, endpoint.endpoint_node_id, decision);
  endpoint.support_authority = ResolvedSupportAuthorityFromDecision(decision, pair_height_rank);
  endpoint.flow_kind = assignment.flow_kind;
  endpoint.origin = support_layout_origin_from_port_source(port.placement_source);
  endpoint.endpoint_source = SupportLayoutEndpointSourceKind::kPlainSupport;
  endpoint.port_source = port.placement_source;
  endpoint.side = port.template_side;
  endpoint.endpoint_mode = CurveEndpointMode::kDirectThrough;
  endpoint.default_lower_required = decision.default_lower_required;
  endpoint.same_level_feasible = decision.same_level_feasible;
  endpoint.unresolved_same_level_conflict = decision.unresolved_same_level_conflict;
  endpoint.same_level_reason = decision.same_level_reason;
  endpoint.projected_spacing_topview_m = decision.projected_spacing_topview_m;
  endpoint.required_clearance_m = decision.required_clearance_m;
  endpoint.solver_used_same_level_constraint = decision.solver_used_same_level_constraint;
  endpoint.used_special_case_ports = decision.used_special_case_ports;
  endpoint.order_decision_policy = decision.order_decision_policy;
  endpoint.order_decision_choice = decision.order_decision_choice;
  endpoint.order_decision_choice_reason = decision.order_decision_choice_reason;
  endpoint.chosen_side = decision.chosen_side;
  endpoint.used_junction_pair_side_assignment = decision.used_junction_pair_side_assignment;
  const bool uses_lowering = decision.lower_required && !decision.lowering_blocked_by_policy;
  const bool uses_pair_height = !uses_lowering && pair_height_rank > 0 && decision.same_level_feasible &&
                                HasAuthoritativeSupportPair(decision);
  const double pair_height_step_m = BranchDownOffsetForCategory(port.category);
  endpoint.automatic_branch_down_offset_m =
      uses_lowering ? assignment.branch_down_offset_m : (uses_pair_height ? pair_height_step_m * pair_height_rank : 0.0);
  endpoint.branch_down_offset_m = endpoint.automatic_branch_down_offset_m;
  return endpoint;
}

} // namespace wire::core::generation::detail
