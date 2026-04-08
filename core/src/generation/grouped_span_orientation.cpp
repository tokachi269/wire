#include "grouped_span_orientation.hpp"

#include "../support_orientation_utils.hpp"
#include "detail_utils.hpp"

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

EndpointSideDecision MakePairAuthorityDecision(ObjectId pair_peer_low, ObjectId pair_peer_high,
                                               std::optional<Vec3d> side_axis = std::nullopt) {
  EndpointSideDecision decision{};
  decision.pair_peer_low = pair_peer_low;
  decision.pair_peer_high = pair_peer_high;
  decision.side_assignment_rule = SideAssignmentRuleKind::kThroughPairNormal;
  decision.support_orientation_rule = SupportOrientationRuleKind::kThroughPairNormal;
  decision.used_junction_pair_side_assignment = HasValidExplicitPairPeers(pair_peer_low, pair_peer_high);
  decision.chosen_side_sign = 0.0;
  if (side_axis.has_value()) {
    decision.side_axis = side_axis.value();
    decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
  }
  return decision;
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
  Vec3d peer_dir = ctx_.support_position(peer_id) - ctx_.support_position(node_id);
  peer_dir.z = 0.0;
  if (!normalize_xy(&peer_dir)) {
    return std::nullopt;
  }
  const double along_pair = dot_xy(peer_dir, pair_dir);
  double chosen_side_sign = 0.0;
  if (std::abs(along_pair) > 1e-9) {
    chosen_side_sign = (along_pair >= 0.0) ? 1.0 : -1.0;
  } else {
    const double lateral_to_pair = dot_xy(peer_dir, side_axis);
    if (std::abs(lateral_to_pair) <= 1e-9) {
      return std::nullopt;
    }
    chosen_side_sign = (lateral_to_pair >= 0.0) ? 1.0 : -1.0;
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

std::optional<LoweredSupportPairInfo> GroupedSpanOrientationDecider::CachedLoweredSupportPairInfo(ObjectId node_id,
                                                                                                  ObjectId peer_id) {
  if (lowered_support_pair_cache_built_.insert(node_id).second) {
    return std::nullopt;
  }
  if (const auto node_it = lowered_support_pair_cache_.find(node_id); node_it != lowered_support_pair_cache_.end()) {
    if (const auto pair_it = node_it->second.find(peer_id); pair_it != node_it->second.end()) {
      return pair_it->second;
    }
  }
  return std::nullopt;
}

double GroupedSpanOrientationDecider::AngularGapRad(double a, double b) const {
  double gap = std::abs(a - b);
  while (gap > 2.0 * kPi) {
    gap -= 2.0 * kPi;
  }
  return std::min(gap, 2.0 * kPi - gap);
}

void GroupedSpanOrientationDecider::RecordSupportPair(std::map<ObjectId, LoweredSupportPairInfo>* pairings,
                                                      ObjectId peer_a, ObjectId peer_b) const {
  if (pairings == nullptr || peer_a == kInvalidObjectId || peer_b == kInvalidObjectId || peer_a == peer_b) {
    return;
  }
  LoweredSupportPairInfo info_a{};
  info_a.companion_peer_id = peer_b;
  info_a.pair_peer_low = std::min(peer_a, peer_b);
  info_a.pair_peer_high = std::max(peer_a, peer_b);
  info_a.has_pair = true;
  LoweredSupportPairInfo info_b = info_a;
  info_b.companion_peer_id = peer_a;
  (*pairings)[peer_a] = info_a;
  (*pairings)[peer_b] = info_b;
}

std::vector<const GroupedSpanOrientationDecider::SupportPairCandidate*>
GroupedSpanOrientationDecider::BuildSupportPairSequence(const std::vector<SupportPairCandidate>& ordered, int start_index,
                                                        int skip_index) const {
  std::vector<const SupportPairCandidate*> sequence{};
  sequence.reserve(ordered.size());
  for (std::size_t offset = 0; offset < ordered.size(); ++offset) {
    const int index = (start_index + static_cast<int>(offset)) % static_cast<int>(ordered.size());
    if (index != skip_index) {
      sequence.push_back(&ordered[static_cast<std::size_t>(index)]);
    }
  }
  return sequence;
}

double GroupedSpanOrientationDecider::LinearSupportPairCost(
    const std::vector<const SupportPairCandidate*>& sequence) const {
  if (sequence.size() < 2 || (sequence.size() % 2) != 0) {
    return std::numeric_limits<double>::infinity();
  }
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < sequence.size(); i += 2) {
    total += AngularGapRad(sequence[i]->angle, sequence[i + 1]->angle);
  }
  return total;
}

std::vector<const GroupedSpanOrientationDecider::SupportPairCandidate*>
GroupedSpanOrientationDecider::BestAdjacentSupportPairSequence(const std::vector<SupportPairCandidate>& ordered) const {
  std::vector<const SupportPairCandidate*> best_sequence{};
  double best_cost = std::numeric_limits<double>::infinity();
  const auto consider_sequence = [&](std::vector<const SupportPairCandidate*> sequence) {
    const double cost = LinearSupportPairCost(sequence);
    if (cost + 1e-9 < best_cost) {
      best_cost = cost;
      best_sequence = std::move(sequence);
    }
  };
  if ((ordered.size() % 2) == 0) {
    for (int start_index : {0, 1}) {
      if (start_index < static_cast<int>(ordered.size())) {
        consider_sequence(BuildSupportPairSequence(ordered, start_index, -1));
      }
    }
  } else {
    for (int skip_index = 0; skip_index < static_cast<int>(ordered.size()); ++skip_index) {
      consider_sequence(BuildSupportPairSequence(ordered, (skip_index + 1) % static_cast<int>(ordered.size()),
                                                 skip_index));
    }
  }
  return best_sequence;
}

void GroupedSpanOrientationDecider::SortSupportPairCandidates(std::vector<SupportPairCandidate>* candidates) const {
  if (candidates == nullptr) {
    return;
  }
  std::ranges::sort(*candidates, [](const SupportPairCandidate& a, const SupportPairCandidate& b) {
    if (a.angle != b.angle) {
      return a.angle < b.angle;
    }
    return a.peer_id < b.peer_id;
  });
}

void GroupedSpanOrientationDecider::BuildAdjacentSupportPairs(const std::vector<SupportPairCandidate>& ordered,
                                                              std::map<ObjectId, LoweredSupportPairInfo>* pairings) const {
  if (pairings == nullptr || ordered.size() < 2) {
    return;
  }
  const std::vector<const SupportPairCandidate*> best_sequence = BestAdjacentSupportPairSequence(ordered);
  for (std::size_t i = 0; i + 1 < best_sequence.size(); i += 2) {
    RecordSupportPair(pairings, best_sequence[i]->peer_id, best_sequence[i + 1]->peer_id);
  }
}

void GroupedSpanOrientationDecider::PopulateSupportPairCandidates(
    ObjectId node_id, const JunctionRelation& relation, std::vector<SupportPairCandidate>* route_candidates,
    std::vector<SupportPairCandidate>* lowered_candidates, std::map<ObjectId, LoweredSupportPairInfo>* pairings) const {
  if (route_candidates == nullptr || lowered_candidates == nullptr || pairings == nullptr) {
    return;
  }
  for (const JunctionIncidentRelation& incident : relation.incidents) {
    if (!incident.in_route || incident.continuity_class != ContinuityCategoryClass::kBundleLike ||
        incident.kind == JunctionRelationKind::kNone) {
      continue;
    }
    Vec3d dir = ctx_.support_position(incident.neighbor_node_id) - ctx_.support_position(node_id);
    dir.z = 0.0;
    if (!normalize_xy(&dir)) {
      continue;
    }
    SupportPairCandidate candidate{};
    candidate.peer_id = incident.neighbor_node_id;
    candidate.dir = dir;
    candidate.angle = std::atan2(dir.y, dir.x);
    candidate.lower_required =
        incident.kind != JunctionRelationKind::kThroughMain &&
        (incident.default_lower_required || !incident.same_level_feasible);
    route_candidates->push_back(candidate);
    if (candidate.lower_required) {
      lowered_candidates->push_back(candidate);
    }
    (*pairings)[incident.neighbor_node_id] = {};
  }
}

ObjectId GroupedSpanOrientationDecider::AngularCompanionForLowered(
    const SupportPairCandidate& lowered_candidate, const std::vector<SupportPairCandidate>& route_candidates) const {
  ObjectId companion_peer_id = kInvalidObjectId;
  double best_gap = std::numeric_limits<double>::infinity();
  for (const SupportPairCandidate& route_candidate : route_candidates) {
    if (route_candidate.peer_id == lowered_candidate.peer_id) {
      continue;
    }
    const double gap = AngularGapRad(lowered_candidate.angle, route_candidate.angle);
    if (const bool equal_gap = std::abs(gap - best_gap) <= 1e-9;
        gap + 1e-9 < best_gap || (equal_gap && route_candidate.peer_id < companion_peer_id)) {
      companion_peer_id = route_candidate.peer_id;
      best_gap = gap;
    }
  }
  return companion_peer_id;
}

void GroupedSpanOrientationDecider::PairUnpairedLoweredCandidates(
    ObjectId node_id, const JunctionRelation& relation, const std::vector<SupportPairCandidate>& lowered_candidates,
    const std::vector<SupportPairCandidate>& route_candidates, std::map<ObjectId, LoweredSupportPairInfo>* pairings) const {
  if (pairings == nullptr) {
    return;
  }
  auto route_candidate_contains = [&](ObjectId candidate_id) {
    return std::any_of(route_candidates.begin(), route_candidates.end(),
                       [&](const SupportPairCandidate& candidate) { return candidate.peer_id == candidate_id; });
  };
  auto preferred_route_local_companion = [&](ObjectId lowered_peer_id) -> ObjectId {
    if (!relation.through_pair.accepted || lowered_peer_id == relation.through_pair.neighbor_a_id ||
        lowered_peer_id == relation.through_pair.neighbor_b_id) {
      return kInvalidObjectId;
    }
    if (relation.through_pair.neighbor_a_id != kInvalidObjectId &&
        relation.through_pair.neighbor_a_id != lowered_peer_id &&
        route_candidate_contains(relation.through_pair.neighbor_a_id)) {
      return relation.through_pair.neighbor_a_id;
    }
    if (relation.through_pair.neighbor_b_id != kInvalidObjectId &&
        relation.through_pair.neighbor_b_id != lowered_peer_id &&
        route_candidate_contains(relation.through_pair.neighbor_b_id)) {
      return relation.through_pair.neighbor_b_id;
    }
    return kInvalidObjectId;
  };
  for (const SupportPairCandidate& lowered_candidate : lowered_candidates) {
    if (const auto existing_pair = pairings->find(lowered_candidate.peer_id);
        existing_pair != pairings->end() && existing_pair->second.has_pair) {
      continue;
    }
    if (const ObjectId companion_peer_id = preferred_route_local_companion(lowered_candidate.peer_id);
        companion_peer_id != kInvalidObjectId) {
      RecordSupportPair(pairings, lowered_candidate.peer_id, companion_peer_id);
      continue;
    }
    if (const ObjectId companion_peer_id = AngularCompanionForLowered(lowered_candidate, route_candidates);
        companion_peer_id != kInvalidObjectId) {
      RecordSupportPair(pairings, lowered_candidate.peer_id, companion_peer_id);
    }
  }
}

bool GroupedSpanOrientationDecider::BuildLoweredSupportPairsForNode(
    ObjectId node_id, std::map<ObjectId, LoweredSupportPairInfo>* pairings) {
  if (ctx_.junction_relations_by_node == nullptr || pairings == nullptr) {
    return false;
  }
  const auto relation_it = ctx_.junction_relations_by_node->find(node_id);
  if (relation_it == ctx_.junction_relations_by_node->end()) {
    return false;
  }
  std::vector<SupportPairCandidate> route_candidates{};
  std::vector<SupportPairCandidate> lowered_candidates{};
  PopulateSupportPairCandidates(node_id, relation_it->second, &route_candidates, &lowered_candidates, pairings);
  SortSupportPairCandidates(&route_candidates);
  SortSupportPairCandidates(&lowered_candidates);
  BuildAdjacentSupportPairs(lowered_candidates, pairings);
  PairUnpairedLoweredCandidates(node_id, relation_it->second, lowered_candidates, route_candidates, pairings);
  return true;
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
  if (const auto cached_pair = CachedLoweredSupportPairInfo(node_id, peer_id); cached_pair.has_value()) {
    return cached_pair;
  }
  auto& pairings = lowered_support_pair_cache_[node_id];
  if (!BuildLoweredSupportPairsForNode(node_id, &pairings)) {
    return std::nullopt;
  }
  const auto pair_it = pairings.find(peer_id);
  if (pair_it == pairings.end()) {
    return std::nullopt;
  }
  return pair_it->second;
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
  if (!pair_info.has_pair || pair_info.companion_peer_id == kInvalidObjectId ||
      peer_id == pair_info.pair_peer_low || peer_id == pair_info.pair_peer_high ||
      ctx_.junction_relations_by_node == nullptr) {
    return pair_info.companion_peer_id;
  }
  const auto relation_it = ctx_.junction_relations_by_node->find(node_id);
  if (relation_it == ctx_.junction_relations_by_node->end()) {
    return pair_info.companion_peer_id;
  }
  if (relation_it->second.through_pair.accepted) {
    // Do not infer a route-local companion from neighbor_a/neighbor_b ordering.
    // Route-local choice should come from the incident ordering / pair authority below.
  }
  ObjectId ordered_in_route_pair_peer = kInvalidObjectId;
  std::size_t ordered_in_route_pair_peer_index = std::numeric_limits<std::size_t>::max();
  ObjectId in_route_pair_peer = kInvalidObjectId;
  for (std::size_t incident_index = 0; incident_index < relation_it->second.incidents.size(); ++incident_index) {
    const JunctionIncidentRelation& incident = relation_it->second.incidents[incident_index];
    if (!incident.in_route) {
      continue;
    }
    const ObjectId candidate_id = incident.neighbor_node_id;
    if (candidate_id != pair_info.pair_peer_low && candidate_id != pair_info.pair_peer_high) {
      continue;
    }
    if (in_route_pair_peer != kInvalidObjectId && in_route_pair_peer != candidate_id) {
      if (incident_index < ordered_in_route_pair_peer_index ||
          (incident_index == ordered_in_route_pair_peer_index && candidate_id < ordered_in_route_pair_peer)) {
        ordered_in_route_pair_peer = candidate_id;
        ordered_in_route_pair_peer_index = incident_index;
      }
      continue;
    }
    in_route_pair_peer = candidate_id;
    ordered_in_route_pair_peer = candidate_id;
    ordered_in_route_pair_peer_index = incident_index;
  }
  if (ordered_in_route_pair_peer != kInvalidObjectId) {
    return ordered_in_route_pair_peer;
  }
  if (in_route_pair_peer != kInvalidObjectId) {
    return in_route_pair_peer;
  }

  const Vec3d pair_reference = PairReferenceAxisForEndpoint(node_id, peer_id, pair_info);
  ObjectId forward_pair_peer = kInvalidObjectId;
  double best_alignment = -std::numeric_limits<double>::infinity();
  for (ObjectId candidate_id : {pair_info.pair_peer_low, pair_info.pair_peer_high}) {
    if (candidate_id == kInvalidObjectId || candidate_id == peer_id) {
      continue;
    }
    Vec3d candidate_dir = ctx_.support_position(candidate_id) - ctx_.support_position(node_id);
    candidate_dir.z = 0.0;
    if (!normalize_xy(&candidate_dir)) {
      continue;
    }
    const double alignment = dot_xy(candidate_dir, pair_reference);
    if (alignment > best_alignment + 1e-9 ||
        (std::abs(alignment - best_alignment) <= 1e-9 && candidate_id == pair_info.companion_peer_id)) {
      best_alignment = alignment;
      forward_pair_peer = candidate_id;
    }
  }
  return (forward_pair_peer == kInvalidObjectId) ? pair_info.companion_peer_id : forward_pair_peer;
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

std::optional<Vec3d> GroupedSpanOrientationDecider::BisectorAxisForEndpoint(ObjectId node_id, ObjectId peer_id) const {
  if (ctx_.junction_relations_by_node == nullptr || node_id == kInvalidObjectId || peer_id == kInvalidObjectId) {
    return std::nullopt;
  }
  const auto it = ctx_.junction_relations_by_node->find(node_id);
  if (it == ctx_.junction_relations_by_node->end()) {
    return std::nullopt;
  }
  Vec3d peer_dir = ctx_.support_position(peer_id) - ctx_.support_position(node_id);
  peer_dir.z = 0.0;
  if (!normalize_xy(&peer_dir)) {
    return std::nullopt;
  }
  auto score_neighbor = [&](ObjectId neighbor_id) {
    Vec3d dir = ctx_.support_position(neighbor_id) - ctx_.support_position(node_id);
    dir.z = 0.0;
    if (!normalize_xy(&dir)) {
      return -std::numeric_limits<double>::infinity();
    }
    return dot_xy(peer_dir, dir);
  };
  ObjectId companion_id = kInvalidObjectId;
  double companion_score = -std::numeric_limits<double>::infinity();
  const JunctionRelation& relation = it->second;
  if (relation.through_pair.accepted) {
    const bool peer_in_pair =
        (peer_id == relation.through_pair.neighbor_a_id || peer_id == relation.through_pair.neighbor_b_id);
    if (peer_in_pair) {
      companion_id = (peer_id == relation.through_pair.neighbor_a_id) ? relation.through_pair.neighbor_b_id
                                                                       : relation.through_pair.neighbor_a_id;
    } else {
      Vec3d pair_reference = ctx_.support_position(relation.through_pair.neighbor_b_id) -
                             ctx_.support_position(relation.through_pair.neighbor_a_id);
      pair_reference.z = 0.0;
      const bool has_pair_reference = normalize_xy(&pair_reference);
      for (ObjectId candidate_id : {relation.through_pair.neighbor_a_id, relation.through_pair.neighbor_b_id}) {
        Vec3d candidate_dir = ctx_.support_position(candidate_id) - ctx_.support_position(node_id);
        candidate_dir.z = 0.0;
        if (!normalize_xy(&candidate_dir)) {
          continue;
        }
        const double score = has_pair_reference ? dot_xy(candidate_dir, pair_reference) : score_neighbor(candidate_id);
        if (score > companion_score) {
          companion_score = score;
          companion_id = candidate_id;
        }
      }
    }
  }
  if (companion_id == kInvalidObjectId) {
    for (const JunctionIncidentRelation& incident : relation.incidents) {
      if (incident.neighbor_node_id == peer_id || !incident.in_route) {
        continue;
      }
      const double score = score_neighbor(incident.neighbor_node_id);
      if (score > companion_score) {
        companion_score = score;
        companion_id = incident.neighbor_node_id;
      }
    }
  }
  if (companion_id == kInvalidObjectId) {
    for (const JunctionIncidentRelation& incident : relation.incidents) {
      if (incident.neighbor_node_id == peer_id) {
        continue;
      }
      const double score = score_neighbor(incident.neighbor_node_id);
      if (score > companion_score) {
        companion_score = score;
        companion_id = incident.neighbor_node_id;
      }
    }
  }
  if (companion_id == kInvalidObjectId) {
    return std::nullopt;
  }
  Vec3d companion_dir = ctx_.support_position(companion_id) - ctx_.support_position(node_id);
  companion_dir.z = 0.0;
  if (!normalize_xy(&companion_dir)) {
    return std::nullopt;
  }
  Vec3d bisector = peer_dir + companion_dir;
  if (!normalize_xy(&bisector)) {
    return std::nullopt;
  }
  const Vec3d peer_lateral = ComputeLateralAxis(peer_dir);
  if (dot_xy(bisector, peer_lateral) < 0.0) {
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

EndpointSideDecision GroupedSpanOrientationDecider::BuildPairSideDecision(ObjectId node_id, ObjectId peer_id,
                                                                          const LoweredSupportPairInfo& pair_info) const {
  EndpointSideDecision decision{};
  decision.pair_peer_low = pair_info.pair_peer_low;
  decision.pair_peer_high = pair_info.pair_peer_high;
  decision.used_junction_pair_side_assignment =
      HasValidExplicitPairPeers(pair_info.pair_peer_low, pair_info.pair_peer_high);
  const bool peer_is_pair_peer =
      peer_id == pair_info.pair_peer_low || peer_id == pair_info.pair_peer_high;
  bool through_pair_accepted = false;
  if (ctx_.junction_relations_by_node != nullptr) {
    if (const auto relation_it = ctx_.junction_relations_by_node->find(node_id);
        relation_it != ctx_.junction_relations_by_node->end()) {
      through_pair_accepted = relation_it->second.through_pair.accepted;
    }
  }
  if (through_pair_accepted && peer_is_pair_peer) {
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
    return BuildPairSideDecision(node_id, peer_id, *pair_info);
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
