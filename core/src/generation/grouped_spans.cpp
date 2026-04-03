#include "wire/core/core_state.hpp"
#include "../pole_orientation_utils.hpp"
#include "../support_orientation_utils.hpp"
#include "detail_utils.hpp"
#include "grouped_span_common.hpp"
#include "grouped_span_lane_preparation.hpp"
#include "grouped_span_lowering.hpp"
#include "grouped_span_orientation.hpp"
#include "support_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace wire::core {

using namespace generation::detail;

namespace {} // namespace

EditResult<std::vector<ObjectId>>
CoreState::generate_grouped_spans_between_support_nodes(
    const std::vector<ObjectId>& node_ids, const std::unordered_map<ObjectId, SupportNode>& support_node_by_id,
    ObjectId bundle_id, ConnectionCategory category, int conductor_count, double spacing_m, bool maintain_lane_order,
    bool allow_lane_mirror, OrderDecisionPolicyKind order_decision_policy, BackboneFlowKind flow_kind,
    const BackboneLoweringPolicy& lowering_policy,
    const std::unordered_map<ObjectId, JunctionRelation>* junction_relations_by_node,
    std::vector<SegmentLaneAssignment>* out_lane_assignments,
    std::vector<BackboneEdgeOrientation>* out_edge_orientations, BundleKind bundle_template_id) {
  EditResult<std::vector<ObjectId>> result;
  if (node_ids.size() < 2) {
    result.error = "at least 2 support nodes are required";
    return result;
  }
  const int lane_count = std::max(1, conductor_count);
  const PortLayer target_port_layer = category_to_port_layer(category);
  const bool use_lane_row_geometry = maintain_lane_order && lane_count > 1;
  const GroupedSpanSharedContext grouped_span_ctx{node_ids, support_node_by_id, edit_state_access(),
                                                  relation_index_access(), connection_index_access(),
                                                  junction_relations_by_node};
  auto support_pole = [&](ObjectId node_id) -> const Pole* { return grouped_span_ctx.support_pole(node_id); };
  auto incident_relation_kind_for = [&](ObjectId node_id, ObjectId peer_id) -> JunctionRelationKind {
    const JunctionIncidentRelation* incident = grouped_span_ctx.incident_relation_for(node_id, peer_id);
    return (incident == nullptr) ? JunctionRelationKind::kNone : incident->kind;
  };
  auto span_context_for_segment = [&](ObjectId node_a, ObjectId node_b) -> ConnectionContext {
    if (category == ConnectionCategory::kDrop) {
      return ConnectionContext::kDropAdd;
    }
    auto relation_rank = [](JunctionRelationKind kind) {
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
    };
    const JunctionRelationKind relation_a = incident_relation_kind_for(node_a, node_b);
    const JunctionRelationKind relation_b = incident_relation_kind_for(node_b, node_a);
    const JunctionRelationKind segment_relation =
        (relation_rank(relation_a) >= relation_rank(relation_b)) ? relation_a : relation_b;
    if (segment_relation == JunctionRelationKind::kSideBranch ||
        segment_relation == JunctionRelationKind::kCrossUnderpass) {
      return ConnectionContext::kBranchAdd;
    }
    if (segment_relation == JunctionRelationKind::kCornerContinuation) {
      return ConnectionContext::kCornerPass;
    }
    const Pole* pole_a = support_pole(node_a);
    const Pole* pole_b = support_pole(node_b);
    const bool corner_pass = (pole_a != nullptr && pole_a->context.kind == PoleContextKind::kCorner) ||
                             (pole_b != nullptr && pole_b->context.kind == PoleContextKind::kCorner);
    return corner_pass ? ConnectionContext::kCornerPass : ConnectionContext::kTrunkContinue;
  };
  const BundleTemplate* bundle_template = find_bundle_template(bundle_template_id);
  const double grouped_support_fanout_spacing_m =
      std::max(0.1, (bundle_template == nullptr) ? spacing_m : bundle_template->grouped_support_fanout_spacing_m);
  GroupedSpanLoweringDecider lowering(grouped_span_ctx, lowering_policy, bundle_template_id, spacing_m,
                                      grouped_support_fanout_spacing_m);
  GroupedSpanOrientationDecider orientation(grouped_span_ctx);
  GroupedSpanLanePreparer lane_preparer(*this, grouped_span_ctx, lowering, orientation, bundle_id, category,
                                        bundle_template_id, target_port_layer, lane_count, spacing_m,
                                        use_lane_row_geometry, order_decision_policy, &result.change_set);
  EditResult<GroupedSpanLanePlan> lane_plan =
      lane_preparer.BuildLanePlan(lowering_policy, authoritative_.layout_settings.corner_threshold_deg);
  if (!lane_plan.ok) {
    result.error = lane_plan.error;
    return result;
  }
  const std::size_t segment_count = node_ids.size() - 1;

  for (std::size_t seg = 0; seg < segment_count; ++seg) {
    const ObjectId node_a = node_ids[seg];
    const ObjectId node_b = node_ids[seg + 1];
    const ConnectionContext span_context = span_context_for_segment(node_a, node_b);

    SegmentLaneAssignment assignment{};
    assignment.segment_index = seg;
    assignment.pole_a_id = node_a;
    assignment.pole_b_id = node_b;
    assignment.bundle_id = bundle_id;
    assignment.flow_kind = flow_kind;
    lane_preparer.PopulateAssignmentOrdering(lane_plan.value, seg, &assignment);
    const SegmentRelationFeasibility relation_a = lowering.SegmentRelationFeasibilityFor(node_a, node_b);
    const SegmentRelationFeasibility relation_b = lowering.SegmentRelationFeasibilityFor(node_b, node_a);
    SegmentRelationFeasibility effective_relation_a = relation_a;
    SegmentRelationFeasibility effective_relation_b = relation_b;
    assignment.relation_a = lowering.NodeRelationKindAt(seg);
    assignment.relation_b = lowering.NodeRelationKindAt(seg + 1);
    if (static_cast<int>(assignment.port_ids_a.size()) != lane_count ||
        static_cast<int>(assignment.port_ids_b.size()) != lane_count) {
      result.error = "failed to materialize lane assignment plan";
      return result;
    }
    const bool segment_same_level_feasible =
        effective_relation_a.same_level_feasible && effective_relation_b.same_level_feasible;
    const GroupedSpanPreparedPortUsage prepared_port_usage =
        lane_preparer.AnalyzePreparedPorts(assignment.port_ids_a, assignment.port_ids_b, segment_same_level_feasible);
    assignment.uses_branch_support = prepared_port_usage.uses_branch_support;
    assignment.solver_used_same_level_constraint = prepared_port_usage.solver_used_same_level_constraint;
    auto relation_rank = [](JunctionRelationKind kind) {
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
    };
    const JunctionRelationKind segment_relation_kind =
        (relation_rank(assignment.relation_a) >= relation_rank(assignment.relation_b)) ? assignment.relation_a
                                                                                        : assignment.relation_b;
    assignment.used_special_case_ports = prepared_port_usage.used_special_case_ports;
    const bool endpoint_policy_block_a = lowering.EndpointLoweringBlockedByPolicy(effective_relation_a);
    const bool endpoint_policy_block_b = lowering.EndpointLoweringBlockedByPolicy(effective_relation_b);
    assignment.lowering_blocked_by_policy = endpoint_policy_block_a || endpoint_policy_block_b;
    assignment.lowering_blocked_by_policy =
        endpoint_policy_block_a || endpoint_policy_block_b;
    if (assignment.lowering_blocked_by_policy) {
      assignment.same_level_reason = SameLevelFeasibilityReason::kCategoryPolicyDisabled;
    }
    assignment.unresolved_same_level_conflict = !segment_same_level_feasible && assignment.lowering_blocked_by_policy;
    const auto pair_info_a = orientation.LoweredSupportPairInfoForEndpoint(node_a, node_b, effective_relation_a);
    const auto pair_info_b = orientation.LoweredSupportPairInfoForEndpoint(node_b, node_a, effective_relation_b);
    EndpointSideDecision side_decision_a =
        orientation.FinalizeEndpointSideDecision(node_a, node_b, orientation.PreferredSideAxisForEndpoint(node_a, node_b,
                                                                                         effective_relation_a,
                                                                                         bundle_id));
    EndpointSideDecision side_decision_b =
        orientation.FinalizeEndpointSideDecision(node_b, node_a, orientation.PreferredSideAxisForEndpoint(node_b, node_a,
                                                                                         effective_relation_b,
                                                                                         bundle_id));
    if (effective_relation_a.continuity_class == ContinuityCategoryClass::kBundleLike &&
        (effective_relation_a.default_lower_required || !effective_relation_a.same_level_feasible)) {
      side_decision_a = orientation.NormalizeGroupSideDecision(side_decision_a);
    }
    if (effective_relation_b.continuity_class == ContinuityCategoryClass::kBundleLike &&
        (effective_relation_b.default_lower_required || !effective_relation_b.same_level_feasible)) {
      side_decision_b = orientation.NormalizeGroupSideDecision(side_decision_b);
    }
    orientation.FinalizeSideSignForPorts(&side_decision_a, node_a, node_b, assignment.port_ids_a);
    orientation.FinalizeSideSignForPorts(&side_decision_b, node_b, node_a, assignment.port_ids_b);
    EndpointContinuityDecision raw_decision_a =
        lowering.BuildEndpointDecision(effective_relation_a, pair_info_a, side_decision_a, node_a, node_b,
                                order_decision_policy,
                                assignment.order_decision_choice_a, assignment.order_decision_choice_reason_a,
                                assignment.solver_used_same_level_constraint, assignment.used_special_case_ports,
                                endpoint_policy_block_a, assignment.unresolved_same_level_conflict);
    EndpointContinuityDecision raw_decision_b =
        lowering.BuildEndpointDecision(effective_relation_b, pair_info_b, side_decision_b, node_b, node_a,
                                order_decision_policy,
                                assignment.order_decision_choice_b, assignment.order_decision_choice_reason_b,
                                assignment.solver_used_same_level_constraint, assignment.used_special_case_ports,
                                endpoint_policy_block_b, assignment.unresolved_same_level_conflict);
    orientation.PrimeGroupEndpointDecision(raw_decision_a, node_b, pair_info_a, side_decision_a);
    orientation.PrimeGroupEndpointDecision(raw_decision_b, node_a, pair_info_b, side_decision_b);
    assignment.decision_a = orientation.CanonicalizeGroupEndpointDecision(raw_decision_a);
    assignment.decision_b = orientation.CanonicalizeGroupEndpointDecision(raw_decision_b);
    GroupedSpanLanePreparer::SyncAssignmentFromDecisions(&assignment);
    const bool decision_lowered_a = UsesAuthoritativeGroupedLoweredSupport(assignment.decision_a);
    const bool decision_lowered_b = UsesAuthoritativeGroupedLoweredSupport(assignment.decision_b);
    assignment.lowering_kind = lowering.LoweringKindForSegment(segment_relation_kind, decision_lowered_a,
                                                               decision_lowered_b);
    assignment.branch_down_offset_m = (decision_lowered_a || decision_lowered_b) ? lowering.effective_branch_down_offset_m() : 0.0;

    for (int lane = 0; lane < lane_count; ++lane) {
      const ObjectId port_a_id = assignment.port_ids_a[static_cast<std::size_t>(lane)];
      const ObjectId port_b_id = assignment.port_ids_b[static_cast<std::size_t>(lane)];
      const auto add = AddSpan(port_a_id, port_b_id, DefaultSpanKindForCategory(category), category_to_span_layer(category),
                               bundle_id);
      if (!add.ok) {
        result.error = add.error;
        return result;
      }
      append_change_set(result.change_set, add.change_set);
      const auto ensure_attachments = ensure_default_endpoint_attachments_for_span(add.value);
      if (!ensure_attachments.ok) {
        result.error = ensure_attachments.error;
        return result;
      }
      append_change_set(result.change_set, ensure_attachments.change_set);
      result.value.push_back(add.value);

      const Port* pa = edit_state_access().ports.find(port_a_id);
      const Port* pb = edit_state_access().ports.find(port_b_id);
      (void)pa;
      (void)pb;

      Span* span = edit_state_access().spans.find(add.value);
      if (span != nullptr) {
        span->endpoint_node_a_id = node_a;
        span->endpoint_node_b_id = node_b;
        span->placement_context = span_context;
        span->generated_by_rule = true;
        span->generation.generated = true;
        add_unique_id(result.change_set.updated_ids, span->id);
      }
    }

    if (out_lane_assignments != nullptr) {
      out_lane_assignments->push_back(assignment);
    }
    if (out_edge_orientations != nullptr) {
      BackboneEdgeOrientation edge_orientation{};
      edge_orientation.node_a_id = assignment.pole_a_id;
      edge_orientation.node_b_id = assignment.pole_b_id;
      edge_orientation.bundle_template_id = bundle_template_id;
      edge_orientation.flow_decision_rule = assignment.flow_decision_rule;
      edge_orientation.flow_kind = flow_kind;
      edge_orientation.relation_a = assignment.relation_a;
      edge_orientation.relation_b = assignment.relation_b;
      edge_orientation.continuity_class = assignment.continuity_class;
      edge_orientation.default_lower_required = assignment.default_lower_required;
      edge_orientation.same_level_feasible = assignment.same_level_feasible;
      edge_orientation.same_level_reason = assignment.same_level_reason;
      edge_orientation.projected_spacing_topview_m = assignment.projected_spacing_topview_m;
      edge_orientation.required_clearance_m = assignment.required_clearance_m;
      edge_orientation.lowering_blocked_by_policy = assignment.lowering_blocked_by_policy;
      edge_orientation.unresolved_same_level_conflict = assignment.unresolved_same_level_conflict;
      edge_orientation.solver_used_same_level_constraint = assignment.solver_used_same_level_constraint;
      edge_orientation.used_special_case_ports = assignment.used_special_case_ports;
      edge_orientation.order_decision_policy = assignment.order_decision_policy;
      edge_orientation.order_decision_choice_a = assignment.order_decision_choice_a;
      edge_orientation.order_decision_choice_b = assignment.order_decision_choice_b;
      edge_orientation.order_decision_choice_reason_a = assignment.order_decision_choice_reason_a;
      edge_orientation.order_decision_choice_reason_b = assignment.order_decision_choice_reason_b;
    edge_orientation.orientation =
        (assignment.order_decision_choice_a != assignment.order_decision_choice_b) ? LaneOrientation::kReversed
                                                                               : LaneOrientation::kNormal;
      edge_orientation.uses_branch_support = assignment.uses_branch_support;
      edge_orientation.lowering_kind = assignment.lowering_kind;
      edge_orientation.branch_down_offset_m = assignment.branch_down_offset_m;
      edge_orientation.flipped_from_previous = assignment.flipped_from_previous;
      edge_orientation.flip_reason = assignment.flip_reason;
      edge_orientation.turn_angle_deg = assignment.turn_angle_deg;
      out_edge_orientations->push_back(edge_orientation);
    }
  }

  result.ok = !result.value.empty();
  if (!result.ok && result.error.empty()) {
    result.error = "failed to generate grouped spans";
  }
  return result;
}

} // namespace wire::core




