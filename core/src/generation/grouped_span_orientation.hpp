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
  [[nodiscard]] std::optional<EndpointSideDecision> ExplicitPairNormalSideDecisionForEndpoint(
      ObjectId node_id, ObjectId peer_id, ObjectId pair_a, ObjectId pair_b) const;

  [[nodiscard]] EndpointSideDecision FinalizeEndpointSideDecision(ObjectId node_id, ObjectId peer_id,
                                                                 EndpointSideDecision decision) const;

  void FinalizeSideSignForPorts(EndpointSideDecision* decision, ObjectId node_id, ObjectId peer_id,
                                const std::vector<ObjectId>& port_ids) const;

private:
  struct SupportPairCandidate {
    ObjectId peer_id = kInvalidObjectId;
    Vec3d dir{};
    double angle = 0.0;
    bool lower_required = false;
  };

  [[nodiscard]] Vec3d NormalizedOrZeroXY(Vec3d axis) const;
  [[nodiscard]] std::optional<Vec3d> RouteAxisForEndpoint(ObjectId node_id, ObjectId peer_id) const;
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
  void PairUnpairedLoweredCandidates(ObjectId node_id, const JunctionRelation& relation,
                                     const std::vector<SupportPairCandidate>& lowered_candidates,
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
                                                           JunctionRelationKind relation_kind,
                                                           const LoweredSupportPairInfo& pair_info) const;
  void BuildNodeSideAxisHints();

  const GroupedSpanSharedContext& ctx_;
  std::unordered_map<ObjectId, Vec3d> node_side_axis_hints_{};
  std::unordered_map<ObjectId, std::map<ObjectId, LoweredSupportPairInfo>> lowered_support_pair_cache_{};
  std::unordered_set<ObjectId> lowered_support_pair_cache_built_{};
};

} // namespace wire::core::generation::detail
