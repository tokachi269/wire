#pragma once

#include "grouped_span_common.hpp"

#include <map>

namespace wire::core::generation::detail {

class GroupedSpanLoweringDecider {
public:
  GroupedSpanLoweringDecider(const GroupedSpanSharedContext& ctx, const BackboneLoweringPolicy& lowering_policy,
                             BundleKind bundle_template_id, double spacing_m,
                             double grouped_support_fanout_spacing_m);

  [[nodiscard]] SegmentRelationFeasibility SegmentRelationFeasibilityFor(ObjectId node_id, ObjectId peer_id) const;
  [[nodiscard]] JunctionRelationKind NodeRelationKindAt(std::size_t node_index) const;
  [[nodiscard]] bool EndpointLoweringBlockedByPolicy(const SegmentRelationFeasibility& feasibility) const;
  [[nodiscard]] bool EndpointUsesLoweredHeight(const SegmentRelationFeasibility& feasibility) const;
  [[nodiscard]] double EndpointLoweringOffsetM(const SegmentRelationFeasibility& feasibility) const;
  [[nodiscard]] double LaneSpacingForEndpoint(const SegmentRelationFeasibility& feasibility) const;
  [[nodiscard]] bool EndpointRequiresOutboardLoweredPorts(ObjectId node_id, ObjectId peer_id) const;
  [[nodiscard]] double effective_branch_down_offset_m() const { return effective_branch_down_offset_m_; }
  [[nodiscard]] EndpointContinuityDecision
  BuildEndpointDecision(const SegmentRelationFeasibility& feasibility,
                        const std::optional<LoweredSupportPairInfo>& pair_info,
                        const EndpointSideDecision& side_decision, ObjectId node_id, ObjectId peer_id,
                        OrderDecisionPolicyKind order_decision_policy, OrderDecisionChoiceKind order_choice,
                        OrderDecisionChoiceReason order_reason, bool solver_used_same_level_constraint,
                        bool used_special_case_ports, bool lowering_blocked_by_policy,
                        bool unresolved_same_level_conflict);

  [[nodiscard]] BackboneLoweringKind LoweringKindForSegment(JunctionRelationKind segment_relation_kind,
                                                            bool decision_lowered_a, bool decision_lowered_b) const;

private:
  struct SupportGroupDecisionKey {
    ObjectId owner_pole_id = kInvalidObjectId;
    ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
    ObjectId pair_peer_low = kInvalidObjectId;
    ObjectId pair_peer_high = kInvalidObjectId;
    JunctionRelationKind relation_kind = JunctionRelationKind::kNone;
    bool in_through_pair = false;
    auto operator<=>(const SupportGroupDecisionKey&) const = default;
  };

  [[nodiscard]] const JunctionIncidentRelation* IncidentRelationFor(ObjectId node_id, ObjectId peer_id) const;
  [[nodiscard]] bool EndpointHasLoweringConflict(const SegmentRelationFeasibility& feasibility) const;
  [[nodiscard]] int SupportGroupIdForEndpoint(ObjectId node_id, ObjectId peer_id,
                                              const SegmentRelationFeasibility& feasibility,
                                              const std::optional<LoweredSupportPairInfo>& pair_info);
  [[nodiscard]] int RelationRank(JunctionRelationKind kind) const;

  const GroupedSpanSharedContext& ctx_;
  const BackboneLoweringPolicy& lowering_policy_;
  BundleKind bundle_template_id_ = BundleKind::kLowVoltage;
  double spacing_m_ = 0.0;
  double grouped_support_fanout_spacing_m_ = 0.0;
  bool supports_outboard_lowered_ports_ = false;
  double effective_branch_down_offset_m_ = 0.0;
  std::map<SupportGroupDecisionKey, int> support_group_ids_{};
  int next_support_group_id_ = 1;
};

} // namespace wire::core::generation::detail
