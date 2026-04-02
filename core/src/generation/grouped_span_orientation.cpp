#include "grouped_span_orientation.hpp"

#include "../support_orientation_utils.hpp"
#include "detail_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <ranges>

namespace wire::core::generation::detail {

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

std::optional<Vec3d> GroupedSpanOrientationDecider::ThroughPairSideAxisForNode(ObjectId node_id) const {
  if (ctx_.junction_relations_by_node == nullptr || node_id == kInvalidObjectId) {
    return std::nullopt;
  }
  const auto it = ctx_.junction_relations_by_node->find(node_id);
  if (it == ctx_.junction_relations_by_node->end() || !it->second.through_pair.accepted) {
    return std::nullopt;
  }
  const Vec3d a = ctx_.support_position(it->second.through_pair.neighbor_a_id);
  const Vec3d b = ctx_.support_position(it->second.through_pair.neighbor_b_id);
  Vec3d through_dir = b - a;
  through_dir.z = 0.0;
  if (!normalize_xy(&through_dir)) {
    return std::nullopt;
  }
  Vec3d side_axis = ComputeLateralAxis(through_dir);
  if (!normalize_xy(&side_axis)) {
    return std::nullopt;
  }
  return side_axis;
}

bool GroupedSpanOrientationDecider::EndpointHasLoweringConflict(const SegmentRelationFeasibility& feasibility) const {
  return feasibility.default_lower_required || !feasibility.same_level_feasible;
}

bool GroupedSpanOrientationDecider::NodeHasBundleLoweringConflict(ObjectId node_id) const {
  if (ctx_.junction_relations_by_node == nullptr || node_id == kInvalidObjectId) {
    return false;
  }
  const auto it = ctx_.junction_relations_by_node->find(node_id);
  if (it == ctx_.junction_relations_by_node->end()) {
    return false;
  }
  return std::any_of(it->second.incidents.begin(), it->second.incidents.end(),
                     [](const JunctionIncidentRelation& incident) {
                       return incident.in_route &&
                              incident.continuity_class == ContinuityCategoryClass::kBundleLike &&
                              incident.kind != JunctionRelationKind::kNone &&
                              incident.kind != JunctionRelationKind::kThroughMain &&
                              (incident.default_lower_required || !incident.same_level_feasible);
                     });
}

bool GroupedSpanOrientationDecider::SupportsBundleSupportPairing(ObjectId node_id, ObjectId peer_id,
                                                                const SegmentRelationFeasibility& feasibility) const {
  return ctx_.junction_relations_by_node != nullptr && node_id != kInvalidObjectId && peer_id != kInvalidObjectId &&
         feasibility.continuity_class == ContinuityCategoryClass::kBundleLike &&
         (EndpointHasLoweringConflict(feasibility) || NodeHasBundleLoweringConflict(node_id));
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
    const std::vector<SupportPairCandidate>& lowered_candidates, const std::vector<SupportPairCandidate>& route_candidates,
    std::map<ObjectId, LoweredSupportPairInfo>* pairings) const {
  if (pairings == nullptr) {
    return;
  }
  for (const SupportPairCandidate& lowered_candidate : lowered_candidates) {
    if (const auto existing_pair = pairings->find(lowered_candidate.peer_id);
        existing_pair != pairings->end() && existing_pair->second.has_pair) {
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
  PairUnpairedLoweredCandidates(lowered_candidates, route_candidates, pairings);
  return true;
}

std::optional<LoweredSupportPairInfo>
GroupedSpanOrientationDecider::LoweredSupportPairInfoForEndpoint(ObjectId node_id, ObjectId peer_id,
                                                                 const SegmentRelationFeasibility& feasibility) {
  if (!SupportsBundleSupportPairing(node_id, peer_id, feasibility)) {
    return std::nullopt;
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
  ObjectId in_route_pair_peer = kInvalidObjectId;
  for (const JunctionIncidentRelation& incident : relation_it->second.incidents) {
    if (!incident.in_route) {
      continue;
    }
    const ObjectId candidate_id = incident.neighbor_node_id;
    if (candidate_id != pair_info.pair_peer_low && candidate_id != pair_info.pair_peer_high) {
      continue;
    }
    if (in_route_pair_peer != kInvalidObjectId && in_route_pair_peer != candidate_id) {
      return pair_info.companion_peer_id;
    }
    in_route_pair_peer = candidate_id;
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
  decision.chosen_side_sign = 0.0;
  if (const auto pair_axis = PairBisectorAxisForEndpoint(node_id, peer_id, pair_info); pair_axis.has_value()) {
    decision.side_axis = NormalizedOrZeroXY(*pair_axis);
    decision.side_assignment_rule = SideAssignmentRuleKind::kBisector;
    decision.support_orientation_rule = SupportOrientationRuleKind::kBisector;
    decision.used_junction_pair_side_assignment = true;
    decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
    return decision;
  }
  decision.side_axis = NormalizedOrZeroXY(GroupedLineAxisForEndpoint(node_id, peer_id));
  decision.side_assignment_rule = SideAssignmentRuleKind::kChord;
  decision.support_orientation_rule = SupportOrientationRuleKind::kChord;
  decision.used_junction_pair_side_assignment = false;
  decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
  return decision;
}

std::optional<EndpointSideDecision> GroupedSpanOrientationDecider::ConnectedBundleSupportDecisionForNode(
    ObjectId node_id, ObjectId peer_id, const SegmentRelationFeasibility& feasibility) {
  if (node_id == kInvalidObjectId || peer_id == kInvalidObjectId ||
      feasibility.continuity_class != ContinuityCategoryClass::kBundleLike) {
    return std::nullopt;
  }
  const auto pair_info = LoweredSupportPairInfoForEndpoint(node_id, peer_id, feasibility);
  if (!pair_info.has_value() || !pair_info->has_pair) {
    return std::nullopt;
  }
  return BuildPairSideDecision(node_id, peer_id, *pair_info);
}

EndpointSideDecision GroupedSpanOrientationDecider::PreferredSideAxisForEndpoint(
    ObjectId node_id, ObjectId peer_id, const SegmentRelationFeasibility& feasibility) {
  EndpointSideDecision decision{};
  decision.side_axis = NormalizedOrZeroXY(CanonicalSideAxisForOrder(node_id, peer_id));
  decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
  const bool prefers_line_oriented_lowering =
      feasibility.continuity_class == ContinuityCategoryClass::kBundleLike &&
      (feasibility.default_lower_required || !feasibility.same_level_feasible);
  if (const auto connected_bundle = ConnectedBundleSupportDecisionForNode(node_id, peer_id, feasibility);
      connected_bundle.has_value()) {
    return *connected_bundle;
  }
  if (!prefers_line_oriented_lowering) {
    if (feasibility.in_through_pair) {
      if (const auto through_pair_axis = ThroughPairSideAxisForNode(node_id); through_pair_axis.has_value()) {
        decision.side_axis = NormalizedOrZeroXY(*through_pair_axis);
        decision.side_assignment_rule = SideAssignmentRuleKind::kThroughPairNormal;
        decision.support_orientation_rule = SupportOrientationRuleKind::kThroughPairNormal;
        decision.used_junction_pair_side_assignment = true;
        decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
        return decision;
      }
    }
    if (feasibility.kind == JunctionRelationKind::kSideBranch ||
        feasibility.kind == JunctionRelationKind::kCornerContinuation ||
        feasibility.kind == JunctionRelationKind::kCrossUnderpass) {
      if (const auto bisector_axis = BisectorAxisForEndpoint(node_id, peer_id); bisector_axis.has_value()) {
        decision.side_axis = NormalizedOrZeroXY(*bisector_axis);
        decision.side_assignment_rule = SideAssignmentRuleKind::kBisector;
        decision.support_orientation_rule = SupportOrientationRuleKind::kBisector;
        decision.used_junction_pair_side_assignment = true;
        decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
        return decision;
      }
      decision.side_axis = NormalizedOrZeroXY(GroupedLineAxisForEndpoint(node_id, peer_id));
      decision.side_assignment_rule = SideAssignmentRuleKind::kChord;
      decision.support_orientation_rule = SupportOrientationRuleKind::kChord;
      decision.used_junction_pair_side_assignment = false;
      decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
      return decision;
    }
    return decision;
  }
  if (feasibility.kind == JunctionRelationKind::kSideBranch ||
      feasibility.kind == JunctionRelationKind::kCornerContinuation ||
      feasibility.kind == JunctionRelationKind::kCrossUnderpass) {
    if (const auto bisector_axis = BisectorAxisForEndpoint(node_id, peer_id); bisector_axis.has_value()) {
      decision.side_axis = NormalizedOrZeroXY(*bisector_axis);
      decision.side_assignment_rule = SideAssignmentRuleKind::kBisector;
      decision.support_orientation_rule = SupportOrientationRuleKind::kBisector;
      decision.used_junction_pair_side_assignment = true;
      decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
      return decision;
    }
    decision.side_axis = NormalizedOrZeroXY(GroupedLineAxisForEndpoint(node_id, peer_id));
    decision.side_assignment_rule = SideAssignmentRuleKind::kChord;
    decision.support_orientation_rule = SupportOrientationRuleKind::kChord;
    decision.used_junction_pair_side_assignment = false;
    decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
    return decision;
  }
  if (decision.has_side_axis) {
    decision.side_assignment_rule = SideAssignmentRuleKind::kBisector;
    decision.support_orientation_rule = SupportOrientationRuleKind::kBisector;
    decision.used_junction_pair_side_assignment = false;
    return decision;
  }
  decision.side_axis = NormalizedOrZeroXY(GroupedLineAxisForEndpoint(node_id, peer_id));
  decision.side_assignment_rule = SideAssignmentRuleKind::kChord;
  decision.support_orientation_rule = SupportOrientationRuleKind::kChord;
  decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
  return decision;
}

EndpointSideDecision GroupedSpanOrientationDecider::FinalizeEndpointSideDecision(ObjectId node_id, ObjectId peer_id,
                                                                                 EndpointSideDecision decision) const {
  if (decision.support_orientation_rule == SupportOrientationRuleKind::kChord || !decision.has_side_axis) {
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
  const Vec3d base_position = ctx_.support_position(node_id);
  Vec3d peer_dir = ctx_.support_position(peer_id) - ctx_.support_position(node_id);
  peer_dir.z = 0.0;
  if ((decision->side_assignment_rule == SideAssignmentRuleKind::kThroughPairNormal ||
       decision->side_assignment_rule == SideAssignmentRuleKind::kBisector) &&
      normalize_xy(&peer_dir)) {
    const double along = dot_xy(peer_dir, decision->side_axis);
    if (std::abs(along) > 1e-9) {
      decision->chosen_side_sign = (along >= 0.0) ? 1.0 : -1.0;
      return;
    }
  }
  double sum = 0.0;
  int count = 0;
  for (ObjectId port_id : port_ids) {
    const Port* port = ctx_.edit_state.ports.find(port_id);
    if (port == nullptr) {
      continue;
    }
    sum += dot_xy(port->world_position - base_position, decision->side_axis);
    ++count;
  }
  if (count > 0) {
    const double mean = sum / static_cast<double>(count);
    if (std::abs(mean) > 1e-9) {
      decision->chosen_side_sign = (mean >= 0.0) ? 1.0 : -1.0;
      return;
    }
  }
  const Vec3d chord_side_axis = ChordSideAxisForEndpoint(node_id, peer_id);
  const double chord_dot = dot_xy(chord_side_axis, decision->side_axis);
  decision->chosen_side_sign = (std::abs(chord_dot) <= 1e-9) ? 1.0 : ((chord_dot >= 0.0) ? 1.0 : -1.0);
}

int GroupedSpanOrientationDecider::OrientationRulePriority(SupportOrientationRuleKind rule) const {
  switch (rule) {
  case SupportOrientationRuleKind::kThroughPairNormal:
    return 3;
  case SupportOrientationRuleKind::kBisector:
    return 2;
  case SupportOrientationRuleKind::kChord:
    return 1;
  case SupportOrientationRuleKind::kRadial:
  default:
    return 0;
  }
}

int GroupedSpanOrientationDecider::SideAssignmentRulePriority(SideAssignmentRuleKind rule) const {
  switch (rule) {
  case SideAssignmentRuleKind::kThroughPairNormal:
    return 3;
  case SideAssignmentRuleKind::kBisector:
    return 2;
  case SideAssignmentRuleKind::kChord:
    return 1;
  case SideAssignmentRuleKind::kPoleLocal:
  default:
    return 0;
  }
}

EndpointSideDecision GroupedSpanOrientationDecider::NormalizeGroupSideDecision(EndpointSideDecision decision) const {
  decision.side_axis = CanonicalSharedSupportAxis(decision.side_axis);
  decision.has_side_axis = (decision.side_axis.x != 0.0 || decision.side_axis.y != 0.0);
  return decision;
}

std::optional<LoweredSupportPairInfo> GroupedSpanOrientationDecider::CanonicalGroupPairDecision(
    const LoweredSupportGroupKey& key, const std::optional<LoweredSupportPairInfo>& raw_pair_info) {
  if (raw_pair_info.has_value() && raw_pair_info->has_pair) {
    auto [it, inserted] = authoritative_group_pairs_.emplace(key, *raw_pair_info);
    if (inserted || !it->second.has_pair) {
      it->second = *raw_pair_info;
      return std::optional<LoweredSupportPairInfo>{*raw_pair_info};
    }
    return std::optional<LoweredSupportPairInfo>{it->second};
  }
  const auto existing = authoritative_group_pairs_.find(key);
  if (existing == authoritative_group_pairs_.end() || !existing->second.has_pair) {
    return std::optional<LoweredSupportPairInfo>{};
  }
  return std::optional<LoweredSupportPairInfo>{existing->second};
}

bool GroupedSpanOrientationDecider::BetterGroupSideDecision(const EndpointSideDecision& candidate,
                                                            const EndpointSideDecision& existing) const {
  const int candidate_orientation = OrientationRulePriority(candidate.support_orientation_rule);
  const int existing_orientation = OrientationRulePriority(existing.support_orientation_rule);
  if (candidate_orientation != existing_orientation) {
    return candidate_orientation > existing_orientation;
  }
  const int candidate_side = SideAssignmentRulePriority(candidate.side_assignment_rule);
  const int existing_side = SideAssignmentRulePriority(existing.side_assignment_rule);
  if (candidate_side != existing_side) {
    return candidate_side > existing_side;
  }
  if (candidate.used_junction_pair_side_assignment != existing.used_junction_pair_side_assignment) {
    return candidate.used_junction_pair_side_assignment;
  }
  return false;
}

bool GroupedSpanOrientationDecider::GroupedSupportCandidateUsesBranchLeg(
    ObjectId peer_id, const std::optional<LoweredSupportPairInfo>& pair_info, const EndpointSideDecision& decision) const {
  if (!pair_info.has_value() || !pair_info->has_pair ||
      decision.support_orientation_rule != SupportOrientationRuleKind::kBisector) {
    return false;
  }
  return peer_id != pair_info->pair_peer_low && peer_id != pair_info->pair_peer_high;
}

EndpointSideDecision GroupedSpanOrientationDecider::CanonicalGroupSideDecision(
    const LoweredSupportGroupKey& key, ObjectId peer_id, const std::optional<LoweredSupportPairInfo>& raw_pair_info,
    const EndpointSideDecision& raw_decision) {
  const std::optional<LoweredSupportPairInfo> authoritative_pair = CanonicalGroupPairDecision(key, raw_pair_info);
  EndpointSideDecision candidate = NormalizeGroupSideDecision(raw_decision);
  const bool candidate_uses_branch_leg =
      GroupedSupportCandidateUsesBranchLeg(peer_id, authoritative_pair, candidate);
  auto [it, inserted] = authoritative_group_side_decisions_.emplace(key, candidate);
  auto [branch_leg_it, branch_leg_inserted] =
      authoritative_group_prefers_branch_leg_.emplace(key, candidate_uses_branch_leg);
  if (!inserted) {
    EndpointSideDecision existing = it->second;
    const bool existing_uses_branch_leg = branch_leg_it->second;
    if (candidate_uses_branch_leg != existing_uses_branch_leg) {
      if (candidate_uses_branch_leg) {
        existing = candidate;
        branch_leg_it->second = true;
      }
    } else if (BetterGroupSideDecision(candidate, existing)) {
      existing = candidate;
    } else if (!existing.has_side_axis && candidate.has_side_axis) {
      existing.side_axis = candidate.side_axis;
      existing.has_side_axis = true;
    }
    it->second = NormalizeGroupSideDecision(existing);
  } else if (!branch_leg_inserted) {
    branch_leg_it->second = candidate_uses_branch_leg;
  }
  return it->second;
}

void GroupedSpanOrientationDecider::PrimeGroupEndpointDecision(const EndpointContinuityDecision& decision,
                                                              ObjectId peer_node_id,
                                                              const std::optional<LoweredSupportPairInfo>& pair_info,
                                                              const EndpointSideDecision& side_decision) {
  if (!UsesAuthoritativeGroupedLoweredSupport(decision)) {
    return;
  }
  const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(decision);
  (void)CanonicalGroupSideDecision(key, peer_node_id, pair_info, side_decision);
}

EndpointContinuityDecision
GroupedSpanOrientationDecider::CanonicalizeGroupEndpointDecision(const EndpointContinuityDecision& decision) const {
  if (!UsesAuthoritativeGroupedLoweredSupport(decision)) {
    return decision;
  }
  const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(decision);
  const auto side_it = authoritative_group_side_decisions_.find(key);
  if (side_it == authoritative_group_side_decisions_.end()) {
    return decision;
  }
  EndpointContinuityDecision canonical = decision;
  const auto pair_it = authoritative_group_pairs_.find(key);
  if (pair_it != authoritative_group_pairs_.end() && pair_it->second.has_pair) {
    canonical.support_pair_peer_low = pair_it->second.pair_peer_low;
    canonical.support_pair_peer_high = pair_it->second.pair_peer_high;
  }
  const EndpointSideDecision& canonical_side = side_it->second;
  canonical.side_assignment_rule = canonical_side.side_assignment_rule;
  canonical.support_orientation_rule = canonical_side.support_orientation_rule;
  canonical.support_orientation_basis = CanonicalSupportOrientationBasis(canonical_side.support_orientation_rule);
  canonical.chosen_side = LateralSideChoiceFromSign(canonical.chosen_side_sign);
  canonical.used_junction_pair_side_assignment = canonical_side.used_junction_pair_side_assignment;
  canonical.has_side_axis = canonical_side.has_side_axis;
  canonical.side_axis = canonical_side.side_axis;
  return canonical;
}

} // namespace wire::core::generation::detail
