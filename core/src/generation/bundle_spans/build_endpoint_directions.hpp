#pragma once

#include "bundle_span_context.hpp"

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
  [[nodiscard]] std::optional<Vec3d> BackboneSideAxisHint(ObjectId node_id) const;
  [[nodiscard]] std::optional<double> BackboneSideSign(ObjectId node_id, ObjectId peer_id) const;
  [[nodiscard]] Vec3d NormalizedOrZeroXY(Vec3d axis) const;
  [[nodiscard]] Vec3d ChordSideAxisForEndpoint(ObjectId node_id, ObjectId peer_id) const;
  [[nodiscard]] EndpointSideDecision BuildPairSideDecision(ObjectId node_id, ObjectId peer_id,
                                                           JunctionRelationKind relation_kind,
                                                           const LoweredSupportPairInfo& pair_info) const;
  void LoadBackboneSideAxisHints();

  const GroupedSpanSharedContext& ctx_;
  std::unordered_map<ObjectId, Vec3d> node_side_axis_hints_{};
};

} // namespace wire::core::generation::detail
