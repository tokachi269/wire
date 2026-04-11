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
  BuildNodeSideAxisHints();
}

Vec3d GroupedSpanOrientationDecider::NormalizedOrZeroXY(Vec3d axis) const {
  axis.z = 0.0;
  if (!normalize_xy(&axis) || !std::isfinite(axis.x) || !std::isfinite(axis.y)) {
    return Vec3d{0.0, 0.0, 0.0};
  }
  return axis;
}

void GroupedSpanOrientationDecider::BuildNodeSideAxisHints() {
  std::vector<Vec3d> side_axis_by_index(ctx_.node_ids.size(), Vec3d{0.0, 0.0, 0.0});
  for (std::size_t i = 0; i < ctx_.node_ids.size(); ++i) {
    const Vec3d center = ctx_.support_position(ctx_.node_ids[i]);
    Vec3d tangent{0.0, 0.0, 0.0};
    if (i == 0 && i + 1 < ctx_.node_ids.size()) {
      tangent = ctx_.support_position(ctx_.node_ids[i + 1]) - center;
    } else if (i + 1 == ctx_.node_ids.size() && i > 0) {
      tangent = center - ctx_.support_position(ctx_.node_ids[i - 1]);
    } else if (i > 0 && i + 1 < ctx_.node_ids.size()) {
      const Vec3d in_dir = center - ctx_.support_position(ctx_.node_ids[i - 1]);
      const Vec3d out_dir = ctx_.support_position(ctx_.node_ids[i + 1]) - center;
      tangent = in_dir + out_dir;
      if (!normalize_xy(&tangent)) {
        tangent = out_dir;
      }
    }
    if (!normalize_xy(&tangent)) {
      side_axis_by_index[i] = Vec3d{0.0, 0.0, 0.0};
      continue;
    }
    side_axis_by_index[i] = NormalizedOrZeroXY(Vec3d{-tangent.y, tangent.x, 0.0});
  }
  for (std::size_t i = 1; i < side_axis_by_index.size(); ++i) {
    if ((side_axis_by_index[i - 1].x == 0.0 && side_axis_by_index[i - 1].y == 0.0) ||
        (side_axis_by_index[i].x == 0.0 && side_axis_by_index[i].y == 0.0)) {
      continue;
    }
    if (dot_xy(side_axis_by_index[i - 1], side_axis_by_index[i]) < 0.0) {
      side_axis_by_index[i].x = -side_axis_by_index[i].x;
      side_axis_by_index[i].y = -side_axis_by_index[i].y;
    }
  }
  for (std::size_t i = 0; i < ctx_.node_ids.size(); ++i) {
    node_side_axis_hints_[ctx_.node_ids[i]] = side_axis_by_index[i];
  }
}

Vec3d GroupedSpanOrientationDecider::CanonicalSideAxisForOrder(ObjectId node_id, ObjectId peer_id) const {
  if (const auto it = node_side_axis_hints_.find(node_id); it != node_side_axis_hints_.end()) {
    const Vec3d axis = NormalizedOrZeroXY(it->second);
    if (axis.x != 0.0 || axis.y != 0.0) {
      return axis;
    }
  }
  Vec3d dir_xy = ctx_.support_position(peer_id) - ctx_.support_position(node_id);
  if (normalize_xy(&dir_xy) && std::isfinite(dir_xy.x) && std::isfinite(dir_xy.y)) {
    return NormalizedOrZeroXY(ComputeLateralAxis(dir_xy));
  }
  return Vec3d{0.0, 0.0, 0.0};
}

std::optional<Vec3d> GroupedSpanOrientationDecider::RouteAxisForEndpoint(ObjectId node_id, ObjectId peer_id) const {
  Vec3d axis = ctx_.support_position(peer_id) - ctx_.support_position(node_id);
  axis.z = 0.0;
  if (!normalize_xy(&axis)) {
    return std::nullopt;
  }
  return axis;
}

std::optional<EndpointSideDecision> GroupedSpanOrientationDecider::ExplicitPairNormalSideDecisionForEndpoint(
    ObjectId node_id, ObjectId peer_id, ObjectId pair_a, ObjectId pair_b) const {
  if (node_id == kInvalidObjectId || peer_id == kInvalidObjectId || pair_a == kInvalidObjectId ||
      pair_b == kInvalidObjectId) {
    return std::nullopt;
  }
  const ObjectId pair_low = std::min(pair_a, pair_b);
  const ObjectId pair_high = std::max(pair_a, pair_b);
  Vec3d pair_dir = ctx_.support_position(pair_high) - ctx_.support_position(pair_low);
  pair_dir.z = 0.0;
  if (!normalize_xy(&pair_dir)) {
    return std::nullopt;
  }
  Vec3d side_axis = ComputeLateralAxis(pair_dir);
  if (!normalize_xy(&side_axis)) {
    return std::nullopt;
  }
  double chosen_side_sign = 0.0;
  if (peer_id == pair_low) {
    chosen_side_sign = -1.0;
  } else if (peer_id == pair_high) {
    chosen_side_sign = 1.0;
  } else {
    Vec3d peer_dir = ctx_.support_position(peer_id) - ctx_.support_position(node_id);
    peer_dir.z = 0.0;
    if (normalize_xy(&peer_dir)) {
      const double along_pair = dot_xy(peer_dir, pair_dir);
      if (std::abs(along_pair) > 1e-9) {
        chosen_side_sign = (along_pair >= 0.0) ? 1.0 : -1.0;
      } else {
        const double lateral_to_pair = dot_xy(peer_dir, side_axis);
        chosen_side_sign = (lateral_to_pair >= 0.0) ? 1.0 : -1.0;
      }
    } else {
      chosen_side_sign = 1.0;
    }
  }

  EndpointSideDecision decision{};
  decision.side_axis = NormalizedOrZeroXY(side_axis);
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

Vec3d GroupedSpanOrientationDecider::PairReferenceAxisForEndpoint(ObjectId node_id, ObjectId peer_id,
                                                                  const LoweredSupportPairInfo& pair_info) const {
  Vec3d pair_reference = ctx_.support_position(pair_info.pair_peer_high) - ctx_.support_position(pair_info.pair_peer_low);
  pair_reference.z = 0.0;
  if (!normalize_xy(&pair_reference)) {
    pair_reference = ctx_.support_position(peer_id) - ctx_.support_position(node_id);
    pair_reference.z = 0.0;
    normalize_xy(&pair_reference);
  }
  return pair_reference;
}

ObjectId GroupedSpanOrientationDecider::RouteLocalPairCompanionForEndpoint(ObjectId node_id, ObjectId peer_id,
                                                                           const LoweredSupportPairInfo& pair_info) const {
  (void)node_id;
  if (!pair_info.has_pair) {
    return kInvalidObjectId;
  }
  if (pair_info.companion_peer_id != kInvalidObjectId) {
    return pair_info.companion_peer_id;
  }
  if (peer_id == pair_info.pair_peer_low) {
    return pair_info.pair_peer_high;
  }
  if (peer_id == pair_info.pair_peer_high) {
    return pair_info.pair_peer_low;
  }
  return kInvalidObjectId;
}

std::optional<Vec3d> GroupedSpanOrientationDecider::PairBisectorAxisForEndpoint(
    ObjectId node_id, ObjectId peer_id, const LoweredSupportPairInfo& pair_info) const {
  const ObjectId companion_id = RouteLocalPairCompanionForEndpoint(node_id, peer_id, pair_info);
  if (!pair_info.has_pair || companion_id == kInvalidObjectId) {
    return std::nullopt;
  }
  Vec3d peer_dir = ctx_.support_position(peer_id) - ctx_.support_position(node_id);
  peer_dir.z = 0.0;
  Vec3d companion_dir = ctx_.support_position(companion_id) - ctx_.support_position(node_id);
  companion_dir.z = 0.0;
  if (!normalize_xy(&peer_dir) || !normalize_xy(&companion_dir)) {
    return std::nullopt;
  }
  Vec3d pair_reference = PairReferenceAxisForEndpoint(node_id, peer_id, pair_info);
  Vec3d bisector = peer_dir + companion_dir;
  if (!normalize_xy(&bisector)) {
    return pair_reference;
  }
  if (dot_xy(bisector, pair_reference) < 0.0) {
    bisector = ScaleVec(bisector, -1.0);
  }
  return bisector;
}

Vec3d GroupedSpanOrientationDecider::ChordSideAxisForEndpoint(ObjectId node_id, ObjectId peer_id) const {
  if (const auto route_axis = RouteAxisForEndpoint(node_id, peer_id); route_axis.has_value()) {
    Vec3d side_axis = ComputeLateralAxis(*route_axis);
    if (normalize_xy(&side_axis)) {
      return side_axis;
    }
  }
  return Vec3d{0.0, 0.0, 0.0};
}

Vec3d GroupedSpanOrientationDecider::GroupedLineAxisForEndpoint(ObjectId node_id, ObjectId peer_id) const {
  if (const auto route_axis = RouteAxisForEndpoint(node_id, peer_id); route_axis.has_value()) {
    Vec3d side_axis = ComputeLateralAxis(*route_axis);
    if (normalize_xy(&side_axis)) {
      return side_axis;
    }
  }
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
  if (const auto pair_axis = PairBisectorAxisForEndpoint(node_id, peer_id, pair_info); pair_axis.has_value()) {
    decision.side_axis = NormalizedOrZeroXY(*pair_axis);
    decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
    return decision;
  }
  decision.side_axis = NormalizedOrZeroXY(PairReferenceAxisForEndpoint(node_id, peer_id, pair_info));
  decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
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
  const Vec3d base_position = ctx_.support_position(node_id);
  Vec3d peer_dir = ctx_.support_position(peer_id) - ctx_.support_position(node_id);
  peer_dir.z = 0.0;
  if (decision->side_assignment_rule == SideAssignmentRuleKind::kBisector && normalize_xy(&peer_dir)) {
    const double along = dot_xy(peer_dir, decision->side_axis);
    if (std::abs(along) > 1e-9) {
      decision->chosen_side_sign = (along >= 0.0) ? 1.0 : -1.0;
      return;
    }
  }
  (void)base_position;
  (void)port_ids;
  // Do not synthesize a side sign from local port offsets or chord fallback.
  // If pair / bisector authority cannot decide a sign, leave it at zero and let
  // downstream materialization fall back to the plain radial connector.
  decision->chosen_side_sign = 0.0;
}

} // namespace wire::core::generation::detail
