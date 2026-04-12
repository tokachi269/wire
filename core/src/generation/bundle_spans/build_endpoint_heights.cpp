#include "build_endpoint_heights.hpp"

#include "../../support_orientation_utils.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core::generation::detail {

GroupedSpanLoweringDecider::GroupedSpanLoweringDecider(const GroupedSpanSharedContext& ctx,
                                                       const BackboneLoweringPolicy& lowering_policy,
                                                       BundleKind bundle_template_id, double spacing_m,
                                                       double grouped_support_fanout_spacing_m)
    : ctx_(ctx), lowering_policy_(lowering_policy), bundle_template_id_(bundle_template_id), spacing_m_(spacing_m),
      grouped_support_fanout_spacing_m_(grouped_support_fanout_spacing_m),
      supports_outboard_lowered_ports_(lowering_policy.offset_m > 1e-6),
      effective_branch_down_offset_m_(std::max(0.0, lowering_policy.offset_m)) {}

SegmentRelationFeasibility GroupedSpanLoweringDecider::SegmentRelationFeasibilityFor(ObjectId node_id,
                                                                                     ObjectId peer_id) const {
  SegmentRelationFeasibility info{};
  const JunctionIncidentRelation* incident = ctx_.incident_relation_for(node_id, peer_id);
  if (incident == nullptr) {
    return info;
  }
  info.kind = incident->kind;
  info.in_through_pair = incident->in_through_pair;
  if (ctx_.junction_relations_by_node != nullptr) {
    if (const auto node_it = ctx_.junction_relations_by_node->find(node_id); node_it != ctx_.junction_relations_by_node->end()) {
      info.through_pair_accepted = node_it->second.through_pair.accepted;
    }
    if (const auto peer_it = ctx_.junction_relations_by_node->find(peer_id); peer_it != ctx_.junction_relations_by_node->end()) {
      info.peer_through_pair_accepted = peer_it->second.through_pair.accepted;
    }
  }
  if (const JunctionIncidentFeasibility* feasibility = ctx_.incident_feasibility_for(node_id, peer_id);
      feasibility != nullptr) {
    info.continuity_class = feasibility->continuity_class;
    info.default_lower_required = feasibility->default_lower_required;
    info.same_level_feasible = feasibility->same_level_feasible;
    info.reason = feasibility->reason;
    info.projected_spacing_topview_m = feasibility->projected_spacing_topview_m;
    info.required_clearance_m = feasibility->required_clearance_m;
  }
  if (const JunctionIncidentRelation* peer_incident = ctx_.incident_relation_for(peer_id, node_id); peer_incident != nullptr) {
    info.peer_relation_found = true;
    info.peer_relation_kind = peer_incident->kind;
    info.peer_in_route = peer_incident->in_route;
    info.peer_in_through_pair = peer_incident->in_through_pair;
    if (const JunctionIncidentFeasibility* peer_feasibility = ctx_.incident_feasibility_for(peer_id, node_id);
        peer_feasibility != nullptr) {
      info.peer_continuity_class = peer_feasibility->continuity_class;
      info.peer_default_lower_required = peer_feasibility->default_lower_required;
      info.peer_same_level_feasible = peer_feasibility->same_level_feasible;
      info.peer_reason = peer_feasibility->reason;
    }
  }
  return info;
}

int GroupedSpanLoweringDecider::RelationRank(JunctionRelationKind kind) const {
  switch (kind) {
  case JunctionRelationKind::kCrossUnderpass:
    return 4;
  case JunctionRelationKind::kSideBranch:
    return 3;
  case JunctionRelationKind::kCornerContinuation:
    return 2;
  case JunctionRelationKind::kThroughMain:
    return 1;
  case JunctionRelationKind::kNone:
  default:
    return 0;
  }
}

JunctionRelationKind GroupedSpanLoweringDecider::NodeRelationKindAt(std::size_t node_index) const {
  if (ctx_.junction_relations_by_node != nullptr && node_index < ctx_.node_ids.size()) {
    const auto it = ctx_.junction_relations_by_node->find(ctx_.node_ids[node_index]);
    if (it != ctx_.junction_relations_by_node->end()) {
      const JunctionRelation& relation = it->second;
      JunctionRelationKind kind = JunctionRelationKind::kNone;
      for (const JunctionIncidentRelation& incident : relation.incidents) {
        if (!incident.in_route) {
          continue;
        }
        if (RelationRank(incident.kind) > RelationRank(kind)) {
          kind = incident.kind;
        }
      }
      if (kind == JunctionRelationKind::kNone && relation.route_incident_count == 1) {
        kind = JunctionRelationKind::kThroughMain;
      }
      return kind;
    }
  }
  return JunctionRelationKind::kNone;
}

bool GroupedSpanLoweringDecider::EndpointHasLoweringConflict(const SegmentRelationFeasibility& feasibility) const {
  return feasibility.default_lower_required || !feasibility.same_level_feasible;
}

bool GroupedSpanLoweringDecider::EndpointLoweringBlockedByPolicy(
    const SegmentRelationFeasibility& feasibility) const {
  return EndpointHasLoweringConflict(feasibility) && lowering_policy_.offset_m <= 1e-6 &&
         feasibility.kind != JunctionRelationKind::kThroughMain && feasibility.kind != JunctionRelationKind::kNone;
}

double GroupedSpanLoweringDecider::EndpointLoweringOffsetM(const SegmentRelationFeasibility& feasibility) const {
  if (EndpointLoweringBlockedByPolicy(feasibility)) {
    return 0.0;
  }
  if (!EndpointHasLoweringConflict(feasibility) || feasibility.kind == JunctionRelationKind::kThroughMain ||
      feasibility.kind == JunctionRelationKind::kNone) {
    return 0.0;
  }
  return effective_branch_down_offset_m_;
}

bool GroupedSpanLoweringDecider::EndpointUsesLoweredHeight(const SegmentRelationFeasibility& feasibility) const {
  return EndpointLoweringOffsetM(feasibility) > 1e-6;
}

double GroupedSpanLoweringDecider::LaneSpacingForEndpoint(const SegmentRelationFeasibility& feasibility) const {
  if (EndpointUsesLoweredHeight(feasibility)) {
    return grouped_support_fanout_spacing_m_;
  }
  return std::max(0.1, spacing_m_);
}

bool GroupedSpanLoweringDecider::EndpointRequiresOutboardLoweredPorts(ObjectId node_id, ObjectId peer_id) const {
  if (!supports_outboard_lowered_ports_) {
    return false;
  }
  const SegmentRelationFeasibility feasibility = SegmentRelationFeasibilityFor(node_id, peer_id);
  if (feasibility.kind == JunctionRelationKind::kCornerContinuation) {
    return false;
  }
  const bool uses_lowered_height =
      (feasibility.default_lower_required || !feasibility.same_level_feasible) && lowering_policy_.offset_m > 1e-6 &&
      feasibility.kind != JunctionRelationKind::kThroughMain && feasibility.kind != JunctionRelationKind::kNone;
  if (feasibility.kind == JunctionRelationKind::kCrossUnderpass && uses_lowered_height) {
    return true;
  }
  if (ctx_.junction_relations_by_node != nullptr) {
    return false;
  }
  const auto it_ports = ctx_.relation_index.ports_by_pole.find(node_id);
  if (it_ports == ctx_.relation_index.ports_by_pole.end()) {
    return false;
  }
  for (ObjectId port_id : it_ports->second) {
    const Port* port = ctx_.edit_state.ports.find(port_id);
    if (port == nullptr || port->owner_pole_id != node_id) {
      continue;
    }
    const auto it_spans = ctx_.connection_index.spans_by_port.find(port_id);
    if (it_spans == ctx_.connection_index.spans_by_port.end()) {
      continue;
    }
    for (ObjectId span_id : it_spans->second) {
      const Span* span = ctx_.edit_state.spans.find(span_id);
      if (span == nullptr || span->bundle_id == kInvalidObjectId) {
        continue;
      }
      const Bundle* bundle = ctx_.edit_state.bundles.find(span->bundle_id);
      if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id_) {
        continue;
      }
      const Port* port_a = ctx_.edit_state.ports.find(span->port_a_id);
      const Port* port_b = ctx_.edit_state.ports.find(span->port_b_id);
      if (port_a == nullptr || port_b == nullptr) {
        continue;
      }
      const ObjectId other_node_id =
          (span->port_a_id == port_id) ? ctx_.resolve_span_endpoint_node(*span, port_b, false)
                                       : ctx_.resolve_span_endpoint_node(*span, port_a, true);
      if (other_node_id != kInvalidObjectId && other_node_id != node_id && other_node_id != peer_id) {
        return true;
      }
    }
  }
  for (const Span& span : ctx_.edit_state.spans.items()) {
    if (span.bundle_id == kInvalidObjectId) {
      continue;
    }
    const Bundle* bundle = ctx_.edit_state.bundles.find(span.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id_) {
      continue;
    }
    const ObjectId endpoint_a = span.endpoint_node_a_id;
    const ObjectId endpoint_b = span.endpoint_node_b_id;
    if (endpoint_a == node_id && endpoint_b != kInvalidObjectId && endpoint_b != node_id && endpoint_b != peer_id) {
      return true;
    }
    if (endpoint_b == node_id && endpoint_a != kInvalidObjectId && endpoint_a != node_id && endpoint_a != peer_id) {
      return true;
    }
  }
  return false;
}

int GroupedSpanLoweringDecider::SupportGroupIdForEndpoint(ObjectId node_id, ObjectId peer_id,
                                                          const SegmentRelationFeasibility& feasibility,
                                                          const std::optional<LoweredSupportPairInfo>& pair_info) {
  const bool needs_lower = feasibility.default_lower_required || !feasibility.same_level_feasible;
  if (!needs_lower) {
    return -1;
  }
  SupportGroupDecisionKey key{};
  key.owner_pole_id = node_id;
  key.continuity_class = feasibility.continuity_class;
  if (pair_info.has_value() && pair_info->has_pair) {
    key.pair_peer_low = pair_info->pair_peer_low;
    key.pair_peer_high = pair_info->pair_peer_high;
  } else {
    key.nonpair_peer_id = peer_id;
    key.relation_kind = feasibility.kind;
    key.in_through_pair = feasibility.in_through_pair;
  }
  auto [it, inserted] = support_group_ids_.emplace(key, next_support_group_id_);
  if (inserted) {
    ++next_support_group_id_;
  }
  return it->second;
}

EndpointContinuityDecision GroupedSpanLoweringDecider::BuildEndpointDecision(
    const SegmentRelationFeasibility& feasibility, const std::optional<LoweredSupportPairInfo>& pair_info,
    const EndpointSideDecision& side_decision, ObjectId node_id, ObjectId peer_id,
    OrderDecisionPolicyKind order_decision_policy, OrderDecisionChoiceKind order_choice,
    OrderDecisionChoiceReason order_reason, bool solver_used_same_level_constraint, bool used_special_case_ports,
    bool lowering_blocked_by_policy, bool unresolved_same_level_conflict) {
  EndpointContinuityDecision decision{};
  decision.owner_pole_id = node_id;
  const bool endpoint_has_conflict = feasibility.default_lower_required || !feasibility.same_level_feasible;
  decision.relation_kind = feasibility.kind;
  decision.continuity_class = feasibility.continuity_class;
  decision.in_through_pair = feasibility.in_through_pair;
  if (pair_info.has_value() && pair_info->has_pair) {
    decision.support_pair_peer_low = pair_info->pair_peer_low;
    decision.support_pair_peer_high = pair_info->pair_peer_high;
  } else if (side_decision.pair_peer_low != kInvalidObjectId && side_decision.pair_peer_high != kInvalidObjectId) {
    decision.support_pair_peer_low = side_decision.pair_peer_low;
    decision.support_pair_peer_high = side_decision.pair_peer_high;
  }
  decision.support_group_id =
      lowering_blocked_by_policy ? -1 : SupportGroupIdForEndpoint(node_id, peer_id, feasibility, pair_info);
  decision.lower_required = feasibility.default_lower_required || !feasibility.same_level_feasible;
  decision.default_lower_required = feasibility.default_lower_required;
  decision.same_level_feasible = feasibility.same_level_feasible;
  decision.same_level_reason =
      (lowering_blocked_by_policy && endpoint_has_conflict) ? SameLevelFeasibilityReason::kCategoryPolicyDisabled
                                                            : feasibility.reason;
  decision.projected_spacing_topview_m = feasibility.projected_spacing_topview_m;
  decision.required_clearance_m = feasibility.required_clearance_m;
  decision.lowering_blocked_by_policy = lowering_blocked_by_policy && endpoint_has_conflict;
  decision.unresolved_same_level_conflict = unresolved_same_level_conflict && endpoint_has_conflict;
  decision.solver_used_same_level_constraint = solver_used_same_level_constraint;
  decision.used_special_case_ports = used_special_case_ports;
  decision.order_decision_policy = order_decision_policy;
  decision.order_decision_choice = order_choice;
  decision.order_decision_choice_reason = order_reason;
  decision.side_assignment_rule = side_decision.side_assignment_rule;
  decision.support_orientation_rule = side_decision.support_orientation_rule;
  decision.support_orientation_basis =
      SupportOrientationBasisFromDecision(side_decision.support_orientation_rule, side_decision.chosen_side_sign);
  decision.chosen_side = LateralSideChoiceFromSign(side_decision.chosen_side_sign);
  decision.used_junction_pair_side_assignment = side_decision.used_junction_pair_side_assignment;
  decision.has_side_axis = side_decision.has_side_axis;
  decision.side_axis = side_decision.side_axis;
  decision.chosen_side_sign = side_decision.chosen_side_sign;
  return decision;
}

BackboneLoweringKind GroupedSpanLoweringDecider::LoweringKindForSegment(JunctionRelationKind segment_relation_kind,
                                                                        bool decision_lowered_a,
                                                                        bool decision_lowered_b) const {
  const bool segment_uses_lowered_height = decision_lowered_a || decision_lowered_b;
  if (segment_uses_lowered_height && segment_relation_kind == JunctionRelationKind::kCrossUnderpass) {
    return BackboneLoweringKind::kCrossUnderpass;
  }
  if (segment_uses_lowered_height && segment_relation_kind == JunctionRelationKind::kSideBranch) {
    return BackboneLoweringKind::kBranchSupport;
  }
  if (segment_uses_lowered_height && segment_relation_kind == JunctionRelationKind::kCornerContinuation) {
    return BackboneLoweringKind::kAcuteCorner;
  }
  return BackboneLoweringKind::kNone;
}

} // namespace wire::core::generation::detail
