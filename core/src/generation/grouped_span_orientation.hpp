#pragma once

#include "grouped_span_common.hpp"

#include <map>
#include <optional>
#include <unordered_map>
#include <unordered_set>

namespace wire::core::generation::detail {

class GroupedSpanOrientationDecider {
public:
  explicit GroupedSpanOrientationDecider(const GroupedSpanSharedContext& ctx);

  [[nodiscard]] Vec3d CanonicalSideAxisForOrder(ObjectId node_id, ObjectId peer_id) const;
  [[nodiscard]] Vec3d GroupedLineAxisForEndpoint(ObjectId node_id, ObjectId peer_id) const;

  [[nodiscard]] std::optional<LoweredSupportPairInfo>
  LoweredSupportPairInfoForEndpoint(ObjectId node_id, ObjectId peer_id, const SegmentRelationFeasibility& feasibility);

  [[nodiscard]] EndpointSideDecision PreferredSideAxisForEndpoint(ObjectId node_id, ObjectId peer_id,
                                                                 const SegmentRelationFeasibility& feasibility,
                                                                 ObjectId bundle_id);

  [[nodiscard]] EndpointSideDecision NormalizeGroupSideDecision(EndpointSideDecision decision) const;

  [[nodiscard]] EndpointSideDecision FinalizeEndpointSideDecision(ObjectId node_id, ObjectId peer_id,
                                                                 EndpointSideDecision decision) const;

  void FinalizeSideSignForPorts(EndpointSideDecision* decision, ObjectId node_id, ObjectId peer_id,
                                const std::vector<ObjectId>& port_ids) const;

  void PrimeGroupEndpointDecision(const EndpointContinuityDecision& decision, ObjectId peer_node_id,
                                  const std::optional<LoweredSupportPairInfo>& pair_info,
                                  const EndpointSideDecision& side_decision);

  [[nodiscard]] EndpointContinuityDecision
  CanonicalizeGroupEndpointDecision(const EndpointContinuityDecision& decision) const;

private:
  struct SupportPairCandidate {
    ObjectId peer_id = kInvalidObjectId;
    Vec3d dir{};
    double angle = 0.0;
    bool lower_required = false;
  };

  struct SupportGroupDecisionKey {
    ObjectId owner_pole_id = kInvalidObjectId;
    ContinuityCategoryClass continuity_class = ContinuityCategoryClass::kPointLike;
    ObjectId pair_peer_low = kInvalidObjectId;
    ObjectId pair_peer_high = kInvalidObjectId;
    JunctionRelationKind relation_kind = JunctionRelationKind::kNone;
    bool in_through_pair = false;
    auto operator<=>(const SupportGroupDecisionKey&) const = default;
  };

  [[nodiscard]] Vec3d NormalizedOrZeroXY(Vec3d axis) const;
  [[nodiscard]] std::optional<Vec3d> RouteAxisForEndpoint(ObjectId node_id, ObjectId peer_id) const;
  [[nodiscard]] std::optional<Vec3d> ThroughPairSideAxisForNode(ObjectId node_id) const;
  [[nodiscard]] std::optional<EndpointSideDecision> ExplicitPairNormalSideDecisionForEndpoint(
      ObjectId node_id, ObjectId peer_id, ObjectId pair_a, ObjectId pair_b) const;
  [[nodiscard]] std::optional<EndpointSideDecision> ThroughPairSideDecisionForEndpoint(ObjectId node_id,
                                                                                       ObjectId peer_id) const;
  [[nodiscard]] std::optional<EndpointSideDecision> CrossPairSideDecisionForEndpoint(ObjectId node_id,
                                                                                     ObjectId peer_id) const;
  [[nodiscard]] std::optional<EndpointSideDecision> BundlePairSideDecisionForEndpoint(ObjectId node_id,
                                                                                      ObjectId peer_id,
                                                                                      ObjectId bundle_id) const;
  [[nodiscard]] std::optional<EndpointSideDecision> CrossLikePairSideDecisionForEndpoint(ObjectId node_id,
                                                                                         ObjectId peer_id) const;
  [[nodiscard]] std::optional<EndpointSideDecision> PoleDebugPairSideDecisionForEndpoint(
      ObjectId node_id, ObjectId peer_id, const SegmentRelationFeasibility& feasibility) const;
  [[nodiscard]] bool EndpointHasLoweringConflict(const SegmentRelationFeasibility& feasibility) const;
  [[nodiscard]] bool NodeHasBundleLoweringConflict(ObjectId node_id) const;
  [[nodiscard]] bool SupportsBundleSupportPairing(ObjectId node_id, ObjectId peer_id,
                                                 const SegmentRelationFeasibility& feasibility) const;
  [[nodiscard]] std::optional<LoweredSupportPairInfo> CachedLoweredSupportPairInfo(ObjectId node_id,
                                                                                   ObjectId peer_id);
  [[nodiscard]] bool BuildLoweredSupportPairsForNode(ObjectId node_id,
                                                     std::map<ObjectId, LoweredSupportPairInfo>* pairings);
  void PopulateSupportPairCandidates(ObjectId node_id, const JunctionRelation& relation,
                                     std::vector<SupportPairCandidate>* route_candidates,
                                     std::vector<SupportPairCandidate>* lowered_candidates,
                                     std::map<ObjectId, LoweredSupportPairInfo>* pairings) const;
  void SortSupportPairCandidates(std::vector<SupportPairCandidate>* candidates) const;
  void BuildAdjacentSupportPairs(const std::vector<SupportPairCandidate>& ordered,
                                 std::map<ObjectId, LoweredSupportPairInfo>* pairings) const;
  void PairUnpairedLoweredCandidates(const std::vector<SupportPairCandidate>& lowered_candidates,
                                     const std::vector<SupportPairCandidate>& route_candidates,
                                     std::map<ObjectId, LoweredSupportPairInfo>* pairings) const;
  [[nodiscard]] ObjectId AngularCompanionForLowered(const SupportPairCandidate& lowered_candidate,
                                                    const std::vector<SupportPairCandidate>& route_candidates) const;
  [[nodiscard]] double AngularGapRad(double a, double b) const;
  [[nodiscard]] std::vector<const SupportPairCandidate*>
  BuildSupportPairSequence(const std::vector<SupportPairCandidate>& ordered, int start_index, int skip_index) const;
  [[nodiscard]] double LinearSupportPairCost(const std::vector<const SupportPairCandidate*>& sequence) const;
  [[nodiscard]] std::vector<const SupportPairCandidate*>
  BestAdjacentSupportPairSequence(const std::vector<SupportPairCandidate>& ordered) const;
  void RecordSupportPair(std::map<ObjectId, LoweredSupportPairInfo>* pairings, ObjectId peer_a, ObjectId peer_b) const;
  [[nodiscard]] Vec3d PairReferenceAxisForEndpoint(ObjectId node_id, ObjectId peer_id,
                                                   const LoweredSupportPairInfo& pair_info) const;
  [[nodiscard]] ObjectId RouteLocalPairCompanionForEndpoint(ObjectId node_id, ObjectId peer_id,
                                                            const LoweredSupportPairInfo& pair_info) const;
  [[nodiscard]] std::optional<Vec3d> PairBisectorAxisForEndpoint(ObjectId node_id, ObjectId peer_id,
                                                                 const LoweredSupportPairInfo& pair_info) const;
  [[nodiscard]] std::optional<Vec3d> BisectorAxisForEndpoint(ObjectId node_id, ObjectId peer_id) const;
  [[nodiscard]] Vec3d ChordSideAxisForEndpoint(ObjectId node_id, ObjectId peer_id) const;
  [[nodiscard]] EndpointSideDecision BuildPairSideDecision(ObjectId node_id, ObjectId peer_id,
                                                           const LoweredSupportPairInfo& pair_info) const;
  [[nodiscard]] std::optional<EndpointSideDecision>
  ConnectedBundleSupportDecisionForNode(ObjectId node_id, ObjectId peer_id,
                                        const SegmentRelationFeasibility& feasibility);
  [[nodiscard]] std::optional<LoweredSupportPairInfo>
  CanonicalGroupPairDecision(const LoweredSupportGroupKey& key,
                             const std::optional<LoweredSupportPairInfo>& raw_pair_info);
  [[nodiscard]] bool BetterGroupSideDecision(const EndpointSideDecision& candidate,
                                             const EndpointSideDecision& existing) const;
  [[nodiscard]] bool GroupedSupportCandidateUsesBranchLeg(ObjectId peer_id,
                                                          const std::optional<LoweredSupportPairInfo>& pair_info,
                                                          const EndpointSideDecision& decision) const;
  [[nodiscard]] EndpointSideDecision CanonicalGroupSideDecision(const LoweredSupportGroupKey& key, ObjectId peer_id,
                                                                const std::optional<LoweredSupportPairInfo>& raw_pair_info,
                                                                const EndpointSideDecision& raw_decision);
  void BuildNodeSideAxisHints();
  [[nodiscard]] int OrientationRulePriority(SupportOrientationRuleKind rule) const;
  [[nodiscard]] int SideAssignmentRulePriority(SideAssignmentRuleKind rule) const;

  const GroupedSpanSharedContext& ctx_;
  std::unordered_map<ObjectId, Vec3d> node_side_axis_hints_{};
  std::unordered_map<ObjectId, std::map<ObjectId, LoweredSupportPairInfo>> lowered_support_pair_cache_{};
  std::unordered_set<ObjectId> lowered_support_pair_cache_built_{};
  std::unordered_map<LoweredSupportGroupKey, LoweredSupportPairInfo, LoweredSupportGroupKeyHash>
      authoritative_group_pairs_{};
  std::unordered_map<LoweredSupportGroupKey, EndpointSideDecision, LoweredSupportGroupKeyHash>
      authoritative_group_side_decisions_{};
  std::unordered_map<LoweredSupportGroupKey, bool, LoweredSupportGroupKeyHash>
      authoritative_group_prefers_branch_leg_{};
};

} // namespace wire::core::generation::detail
