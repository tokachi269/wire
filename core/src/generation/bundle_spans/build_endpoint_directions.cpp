#include "build_endpoint_directions.hpp"

#include "../../support_orientation_utils.hpp"
#include "../detail_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>

namespace wire::core::generation::detail {

namespace {

bool HasValidExplicitPairPeers(ObjectId pair_peer_low, ObjectId pair_peer_high) {
  return pair_peer_low != kInvalidObjectId && pair_peer_high != kInvalidObjectId && pair_peer_low != pair_peer_high;
}

bool IsExplicitPairAuthorityDecision(const EndpointSideDecision& decision) {
  return HasValidExplicitPairPeers(decision.pair_peer_low, decision.pair_peer_high) &&
         decision.side_assignment_rule == SideAssignmentRuleKind::kThroughPairNormal &&
         decision.support_orientation_rule == SupportOrientationRuleKind::kThroughPairNormal;
}

bool RelationConsumesPairAuthority(JunctionRelationKind kind) {
  return kind == JunctionRelationKind::kThroughMain || kind == JunctionRelationKind::kSideBranch ||
         kind == JunctionRelationKind::kCornerContinuation || kind == JunctionRelationKind::kCrossUnderpass;
}

} // namespace

GroupedSpanOrientationDecider::GroupedSpanOrientationDecider(const GroupedSpanSharedContext& ctx) : ctx_(ctx) {
  LoadBackboneSideAxisHints();
}

Vec3d GroupedSpanOrientationDecider::NormalizedOrZeroXY(Vec3d axis) const {
  axis.z = 0.0;
  if (!normalize_xy(&axis) || !std::isfinite(axis.x) || !std::isfinite(axis.y)) {
    return Vec3d{0.0, 0.0, 0.0};
  }
  return axis;
}

void GroupedSpanOrientationDecider::LoadBackboneSideAxisHints() {
  if (ctx_.node_side_axis_by_node == nullptr) {
    return;
  }
  for (ObjectId node_id : ctx_.node_ids) {
    const auto it = ctx_.node_side_axis_by_node->find(node_id);
    if (it == ctx_.node_side_axis_by_node->end()) {
      continue;
    }
    const Vec3d axis = NormalizedOrZeroXY(it->second);
    if (axis.x != 0.0 || axis.y != 0.0) {
      node_side_axis_hints_[node_id] = axis;
    }
  }
}

std::optional<Vec3d> GroupedSpanOrientationDecider::BackboneSideAxisHint(ObjectId node_id) const {
  const auto it = node_side_axis_hints_.find(node_id);
  if (it == node_side_axis_hints_.end()) {
    return std::nullopt;
  }
  const Vec3d axis = NormalizedOrZeroXY(it->second);
  if (axis.x == 0.0 && axis.y == 0.0) {
    return std::nullopt;
  }
  return axis;
}

Vec3d GroupedSpanOrientationDecider::CanonicalSideAxisForOrder(ObjectId node_id, ObjectId peer_id) const {
  if (const auto axis = BackboneSideAxisHint(node_id); axis.has_value()) {
    return *axis;
  }
  (void)peer_id;
  return Vec3d{0.0, 0.0, 0.0};
}

std::optional<EndpointSideDecision> GroupedSpanOrientationDecider::ExplicitPairNormalSideDecisionForEndpoint(
    ObjectId node_id, ObjectId peer_id, ObjectId pair_a, ObjectId pair_b) const {
  if (node_id == kInvalidObjectId || peer_id == kInvalidObjectId || pair_a == kInvalidObjectId ||
      pair_b == kInvalidObjectId) {
    return std::nullopt;
  }
  const auto side_axis = BackboneSideAxisHint(node_id);
  if (!side_axis.has_value()) {
    return std::nullopt;
  }
  const ObjectId pair_low = std::min(pair_a, pair_b);
  const ObjectId pair_high = std::max(pair_a, pair_b);
  double chosen_side_sign = 0.0;
  if (peer_id == pair_low) {
    chosen_side_sign = -1.0;
  } else if (peer_id == pair_high) {
    chosen_side_sign = 1.0;
  }

  EndpointSideDecision decision{};
  decision.side_axis = *side_axis;
  decision.pair_peer_low = pair_low;
  decision.pair_peer_high = pair_high;
  decision.side_assignment_rule = SideAssignmentRuleKind::kThroughPairNormal;
  decision.support_orientation_rule = SupportOrientationRuleKind::kThroughPairNormal;
  decision.used_junction_pair_side_assignment = true;
  decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
  decision.chosen_side_sign = chosen_side_sign;
  return decision;
}

std::optional<LoweredSupportPairInfo>
GroupedSpanOrientationDecider::LoweredSupportPairInfoForEndpoint(ObjectId node_id, ObjectId peer_id,
                                                                 const SegmentRelationFeasibility& feasibility) {
  (void)feasibility;
  if (ctx_.junction_relations_by_node == nullptr || node_id == kInvalidObjectId || peer_id == kInvalidObjectId) {
    return std::nullopt;
  }
  if (const auto relation_it = ctx_.junction_relations_by_node->find(node_id);
      relation_it != ctx_.junction_relations_by_node->end()) {
    const JunctionRelation& relation = relation_it->second;
    auto explicit_pair_info = [&](std::vector<ObjectId> pair_peers) -> std::optional<LoweredSupportPairInfo> {
      std::sort(pair_peers.begin(), pair_peers.end());
      pair_peers.erase(std::unique(pair_peers.begin(), pair_peers.end()), pair_peers.end());
      if (pair_peers.size() != 2) {
        return std::nullopt;
      }
      LoweredSupportPairInfo info{};
      info.pair_peer_low = pair_peers[0];
      info.pair_peer_high = pair_peers[1];
      info.has_pair = true;
      if (peer_id == info.pair_peer_low) {
        info.companion_peer_id = info.pair_peer_high;
      } else if (peer_id == info.pair_peer_high) {
        info.companion_peer_id = info.pair_peer_low;
      }
      return info;
    };
    if (feasibility.kind == JunctionRelationKind::kCrossUnderpass) {
      std::vector<ObjectId> cross_peers{};
      cross_peers.reserve(relation.incidents.size());
      for (const JunctionIncidentRelation& incident : relation.incidents) {
        if (incident.kind == JunctionRelationKind::kCrossUnderpass && incident.neighbor_node_id != kInvalidObjectId) {
          cross_peers.push_back(incident.neighbor_node_id);
        }
      }
      if (const auto info = explicit_pair_info(std::move(cross_peers)); info.has_value()) {
        return info;
      }
    }
    if (relation.through_pair.accepted &&
        HasValidExplicitPairPeers(relation.through_pair.neighbor_a_id, relation.through_pair.neighbor_b_id)) {
      LoweredSupportPairInfo info{};
      info.pair_peer_low = std::min(relation.through_pair.neighbor_a_id, relation.through_pair.neighbor_b_id);
      info.pair_peer_high = std::max(relation.through_pair.neighbor_a_id, relation.through_pair.neighbor_b_id);
      info.has_pair = true;
      if (peer_id == info.pair_peer_low) {
        info.companion_peer_id = info.pair_peer_high;
      } else if (peer_id == info.pair_peer_high) {
        info.companion_peer_id = info.pair_peer_low;
      }
      return info;
    }
  }
  return std::nullopt;
}

Vec3d GroupedSpanOrientationDecider::ChordSideAxisForEndpoint(ObjectId node_id, ObjectId peer_id) const {
  if (const auto side_axis = BackboneSideAxisHint(node_id); side_axis.has_value()) {
    return *side_axis;
  }
  (void)peer_id;
  return Vec3d{0.0, 0.0, 0.0};
}

Vec3d GroupedSpanOrientationDecider::GroupedLineAxisForEndpoint(ObjectId node_id, ObjectId peer_id) const {
  if (const auto side_axis = BackboneSideAxisHint(node_id); side_axis.has_value()) {
    return *side_axis;
  }
  (void)peer_id;
  return Vec3d{0.0, 0.0, 0.0};
}

EndpointSideDecision GroupedSpanOrientationDecider::BuildPairSideDecision(
    ObjectId node_id, ObjectId peer_id, JunctionRelationKind relation_kind, const LoweredSupportPairInfo& pair_info) const {
  EndpointSideDecision decision{};
  decision.pair_peer_low = pair_info.pair_peer_low;
  decision.pair_peer_high = pair_info.pair_peer_high;
  decision.used_junction_pair_side_assignment =
      HasValidExplicitPairPeers(pair_info.pair_peer_low, pair_info.pair_peer_high);
  const bool use_pair_normal = relation_kind == JunctionRelationKind::kCrossUnderpass;
  if (use_pair_normal) {
    if (const auto pair_normal_decision = ExplicitPairNormalSideDecisionForEndpoint(
            node_id, peer_id, pair_info.pair_peer_low, pair_info.pair_peer_high);
        pair_normal_decision.has_value()) {
      return *pair_normal_decision;
    }
  }
  decision.side_assignment_rule = SideAssignmentRuleKind::kBisector;
  decision.support_orientation_rule = SupportOrientationRuleKind::kBisector;
  if (const auto side_axis = BackboneSideAxisHint(node_id); side_axis.has_value()) {
    decision.side_axis = *side_axis;
    decision.has_side_axis = true;
  }
  return decision;
}

EndpointSideDecision GroupedSpanOrientationDecider::PreferredSideAxisForEndpoint(
    ObjectId node_id, ObjectId peer_id, const SegmentRelationFeasibility& feasibility, ObjectId bundle_id) {
  (void)bundle_id;
  const bool pointlike_same_level =
      feasibility.continuity_class == ContinuityCategoryClass::kPointLike && !feasibility.default_lower_required &&
      feasibility.same_level_feasible;
  const bool bundlelike_same_level =
      feasibility.continuity_class == ContinuityCategoryClass::kBundleLike &&
      !feasibility.default_lower_required && feasibility.same_level_feasible;
  const bool pair_consuming_same_level =
      (pointlike_same_level || bundlelike_same_level) && RelationConsumesPairAuthority(feasibility.kind);
  if (!pair_consuming_same_level) {
    return EndpointSideDecision{};
  }
  if (const auto pair_info = LoweredSupportPairInfoForEndpoint(node_id, peer_id, feasibility); pair_info.has_value()) {
    return BuildPairSideDecision(node_id, peer_id, feasibility.kind, *pair_info);
  }
  return EndpointSideDecision{};
}

EndpointSideDecision GroupedSpanOrientationDecider::FinalizeEndpointSideDecision(ObjectId node_id, ObjectId peer_id,
                                                                                 EndpointSideDecision decision) const {
  if (decision.support_orientation_rule == SupportOrientationRuleKind::kChord) {
    decision.side_axis = ChordSideAxisForEndpoint(node_id, peer_id);
    Vec3d normalized_axis = decision.side_axis;
    decision.has_side_axis = normalize_xy(&normalized_axis);
    decision.side_axis = decision.has_side_axis ? normalized_axis : Vec3d{0.0, 0.0, 0.0};
    return decision;
  }
  Vec3d normalized_axis = decision.side_axis;
  decision.has_side_axis = normalize_xy(&normalized_axis);
  decision.side_axis = decision.has_side_axis ? normalized_axis : Vec3d{0.0, 0.0, 0.0};
  return decision;
}

void GroupedSpanOrientationDecider::FinalizeSideSignForPorts(EndpointSideDecision* decision, ObjectId node_id,
                                                             ObjectId peer_id,
                                                             const std::vector<ObjectId>& port_ids) const {
  if (decision == nullptr || !decision->has_side_axis) {
    return;
  }
  if (IsExplicitPairAuthorityDecision(*decision)) {
    return;
  }
  if (std::abs(decision->chosen_side_sign) > 1e-9) {
    return;
  }
  (void)node_id;
  (void)peer_id;
  (void)port_ids;
  // Do not synthesize a side sign from local geometry.
  decision->chosen_side_sign = 0.0;
}

} // namespace wire::core::generation::detail
