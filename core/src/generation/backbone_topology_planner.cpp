#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"
#include "backbone_generation_plan_internal.hpp"
#include "detail_utils.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace wire::core {

using namespace generation::detail;

namespace {

Vec3d normalize_forward_xy(const Vec3d& value) {
  Vec3d out{value.x, value.y, 0.0};
  if (!Normalize(&out)) {
    return {};
  }
  return out;
}

struct RawJunctionCandidate {
  ObjectId neighbor_id = kInvalidObjectId;
  Vec3d dir{};
  bool is_route_neighbor = false;
};

struct RawJunctionFacts {
  ObjectId node_id = kInvalidObjectId;
  int route_incident_count = 0;
  bool is_cross_like = false;
  std::vector<RawJunctionCandidate> candidates{};
};

struct RouteThroughPairDecision {
  ObjectId neighbor_a_id = kInvalidObjectId;
  ObjectId neighbor_b_id = kInvalidObjectId;
  double straightness_score = -2.0;
  bool accepted = false;
};

const RawJunctionCandidate* find_raw_junction_candidate(const RawJunctionFacts& facts, ObjectId neighbor_id) {
  for (const RawJunctionCandidate& candidate : facts.candidates) {
    if (candidate.neighbor_id == neighbor_id) {
      return &candidate;
    }
  }
  return nullptr;
}

double raw_junction_pair_score(const RawJunctionFacts& facts, ObjectId neighbor_a_id, ObjectId neighbor_b_id) {
  const RawJunctionCandidate* candidate_a = find_raw_junction_candidate(facts, neighbor_a_id);
  const RawJunctionCandidate* candidate_b = find_raw_junction_candidate(facts, neighbor_b_id);
  if (candidate_a == nullptr || candidate_b == nullptr) {
    return -2.0;
  }
  return Dot(candidate_a->dir, Vec3d{-candidate_b->dir.x, -candidate_b->dir.y, -candidate_b->dir.z});
}

JunctionRelation build_junction_relation_from_facts(const RawJunctionFacts& facts,
                                                    const RouteThroughPairDecision& through_pair) {
  JunctionRelation relation{};
  relation.node_id = facts.node_id;
  relation.primary_neighbor_id = kInvalidObjectId;
  relation.route_incident_count = facts.route_incident_count;
  relation.is_cross_like = facts.is_cross_like;
  relation.through_pair.neighbor_a_id = through_pair.neighbor_a_id;
  relation.through_pair.neighbor_b_id = through_pair.neighbor_b_id;
  relation.through_pair.straightness_score = through_pair.straightness_score;
  relation.through_pair.accepted = through_pair.accepted;
  relation.incidents.reserve(facts.candidates.size());
  auto in_through_pair = [&](ObjectId neighbor_id) {
    return through_pair.accepted &&
           (neighbor_id == through_pair.neighbor_a_id || neighbor_id == through_pair.neighbor_b_id);
  };
  for (const RawJunctionCandidate& candidate : facts.candidates) {
    JunctionIncidentRelation incident{};
    incident.neighbor_node_id = candidate.neighbor_id;
    incident.in_route = candidate.is_route_neighbor;
    incident.straightness_score = through_pair.straightness_score;
    incident.in_through_pair = in_through_pair(candidate.neighbor_id);
    if (incident.in_through_pair) {
      incident.kind = JunctionRelationKind::kThroughMain;
    } else if (through_pair.accepted && !facts.is_cross_like && facts.route_incident_count == 1 && candidate.is_route_neighbor) {
      incident.kind = JunctionRelationKind::kThroughMain;
    } else if (through_pair.accepted) {
      incident.kind = facts.is_cross_like ? JunctionRelationKind::kCrossUnderpass
                                          : JunctionRelationKind::kSideBranch;
    } else if (candidate.is_route_neighbor) {
      incident.kind = (facts.route_incident_count >= 2) ? JunctionRelationKind::kCornerContinuation
                                                        : JunctionRelationKind::kSideBranch;
    } else {
      incident.kind = JunctionRelationKind::kSideBranch;
    }
    relation.incidents.push_back(incident);
  }
  std::sort(relation.incidents.begin(), relation.incidents.end(),
            [&](const JunctionIncidentRelation& lhs, const JunctionIncidentRelation& rhs) {
              return std::tuple<int, int, ObjectId>{lhs.in_through_pair ? 0 : 1, lhs.in_route ? 0 : 1,
                                                    lhs.neighbor_node_id} <
                     std::tuple<int, int, ObjectId>{rhs.in_through_pair ? 0 : 1, rhs.in_route ? 0 : 1,
                                                    rhs.neighbor_node_id};
            });
  for (const JunctionIncidentRelation& incident : relation.incidents) {
    if (incident.in_route) {
      relation.primary_neighbor_id = incident.neighbor_node_id;
      break;
    }
  }
  if (relation.through_pair.accepted && relation.incidents.size() >= 2) {
    relation.through_pair.neighbor_a_id = relation.incidents[0].neighbor_node_id;
    relation.through_pair.neighbor_b_id = relation.incidents[1].neighbor_node_id;
    relation.primary_neighbor_id = relation.through_pair.neighbor_a_id;
  }
  return relation;
}


std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

std::uint64_t hash_combine(std::uint64_t seed, std::uint64_t value) {
  return splitmix64(seed ^ (value + 0x9E3779B97F4A7C15ull + (seed << 6) + (seed >> 2)));
}


struct BackboneDecisionPhaseContext {
  const std::vector<ObjectId>& ordered_support_node_ids;
  const std::unordered_map<ObjectId, std::uint64_t>& existing_prioritized_session_by_node;
  const std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint64_t>>& existing_incident_session_by_node;
  BackboneResult& generation_backbone;
  std::uint64_t session_id = 0;
  std::function<JunctionRelation(ObjectId)> classify_junction;
};

BackboneDecisionPhaseOutput run_backbone_decision_phase(const BackboneDecisionPhaseContext& context) {
  BackboneDecisionPhaseOutput phase{};
  phase.junction_relations_in_path_order.reserve(context.ordered_support_node_ids.size());
  phase.junction_relations_by_node.reserve(context.ordered_support_node_ids.size());
  for (ObjectId node_id : context.ordered_support_node_ids) {
    JunctionRelation relation = context.classify_junction(node_id);
    phase.junction_relations_by_node[node_id] = relation;
    phase.junction_relations_in_path_order.push_back(std::move(relation));
  }
  auto find_incident_relation = [&](const JunctionRelation& relation, ObjectId peer_id) -> JunctionRelationKind {
    for (const JunctionIncidentRelation& incident : relation.incidents) {
      if (incident.neighbor_node_id == peer_id) {
        return incident.kind;
      }
    }
    return JunctionRelationKind::kNone;
  };
  auto classify_edge_flow_at_node = [&](ObjectId node_id, ObjectId peer_id) {
    EdgeFlowInfo info{};
    const auto it_relation = phase.junction_relations_by_node.find(node_id);
    if (it_relation == phase.junction_relations_by_node.end()) {
      return info;
    }
    const JunctionRelation& relation = it_relation->second;
    if (relation.incidents.size() < 3) {
      return info;
    }
    const JunctionRelationKind relation_kind = find_incident_relation(relation, peer_id);
    switch (relation_kind) {
    case JunctionRelationKind::kCrossUnderpass:
      info.kind = BackboneFlowKind::kBranch;
      info.rule = BackboneFlowDecisionRule::kJunctionOrderBranch;
      info.underpass_at_cross = true;
      return info;
    case JunctionRelationKind::kSideBranch:
      info.kind = BackboneFlowKind::kBranch;
      info.rule = BackboneFlowDecisionRule::kJunctionOrderBranch;
      return info;
    case JunctionRelationKind::kCornerContinuation:
      info.kind = BackboneFlowKind::kMain;
      info.rule = BackboneFlowDecisionRule::kDefaultMain;
      return info;
    case JunctionRelationKind::kThroughMain:
      info.kind = BackboneFlowKind::kMain;
      info.rule = relation.through_pair.accepted ? BackboneFlowDecisionRule::kJunctionOrderMain
                                                 : BackboneFlowDecisionRule::kDefaultMain;
      return info;
    case JunctionRelationKind::kNone:
    default:
      return info;
    }
  };
  phase.edge_flow_by_segment.reserve((context.ordered_support_node_ids.size() > 1)
                                         ? (context.ordered_support_node_ids.size() - 1)
                                         : 0);
  for (std::size_t i = 0; i + 1 < context.ordered_support_node_ids.size(); ++i) {
    const ObjectId node_a = context.ordered_support_node_ids[i];
    const ObjectId node_b = context.ordered_support_node_ids[i + 1];
    const EdgeFlowInfo flow_a = classify_edge_flow_at_node(node_a, node_b);
    const EdgeFlowInfo flow_b = classify_edge_flow_at_node(node_b, node_a);
    if (flow_a.kind == BackboneFlowKind::kBranch && flow_b.kind != BackboneFlowKind::kBranch) {
      phase.edge_flow_by_segment.push_back(flow_a);
    } else if (flow_b.kind == BackboneFlowKind::kBranch && flow_a.kind != BackboneFlowKind::kBranch) {
      phase.edge_flow_by_segment.push_back(flow_b);
    } else if (flow_a.rule != BackboneFlowDecisionRule::kDefaultMain &&
               flow_b.rule == BackboneFlowDecisionRule::kDefaultMain) {
      phase.edge_flow_by_segment.push_back(flow_a);
    } else if (flow_b.rule != BackboneFlowDecisionRule::kDefaultMain &&
               flow_a.rule == BackboneFlowDecisionRule::kDefaultMain) {
      phase.edge_flow_by_segment.push_back(flow_b);
    } else {
      EdgeFlowInfo info{};
      info.kind = BackboneFlowKind::kMain;
      info.rule = (flow_a.rule == BackboneFlowDecisionRule::kJunctionOrderMain ||
                   flow_b.rule == BackboneFlowDecisionRule::kJunctionOrderMain)
                      ? BackboneFlowDecisionRule::kJunctionOrderMain
                      : BackboneFlowDecisionRule::kDefaultMain;
      phase.edge_flow_by_segment.push_back(info);
    }
  }

  context.generation_backbone.junctions.clear();
  context.generation_backbone.junctions.reserve(phase.junction_relations_by_node.size());
  std::unordered_set<ObjectId> rebuilt_junction_nodes{};
  for (ObjectId node_id : context.ordered_support_node_ids) {
    if (!rebuilt_junction_nodes.insert(node_id).second) {
      continue;
    }
    const auto it_relation = phase.junction_relations_by_node.find(node_id);
    if (it_relation == phase.junction_relations_by_node.end()) {
      continue;
    }
    const JunctionRelation& relation = it_relation->second;
    if (relation.incidents.size() < 3) {
      continue;
    }
    JunctionInfo junction{};
    junction.node_id = node_id;
    if (const auto it_existing_prioritized = context.existing_prioritized_session_by_node.find(node_id);
        it_existing_prioritized != context.existing_prioritized_session_by_node.end()) {
      junction.prioritized_session_id = it_existing_prioritized->second;
    } else {
      junction.prioritized_session_id = context.session_id;
    }
    junction.incidents.reserve(relation.incidents.size());
    for (std::size_t index = 0; index < relation.incidents.size(); ++index) {
      const JunctionIncidentRelation& source = relation.incidents[index];
      JunctionIncident incident{};
      incident.neighbor_node_id = source.neighbor_node_id;
      incident.order = static_cast<int>(index);
      incident.primary = (index == 0);
      incident.source_session_id = context.session_id;
      if (const auto it_existing_node = context.existing_incident_session_by_node.find(node_id);
          it_existing_node != context.existing_incident_session_by_node.end()) {
        if (const auto it_existing_source = it_existing_node->second.find(source.neighbor_node_id);
            it_existing_source != it_existing_node->second.end()) {
          incident.source_session_id = it_existing_source->second;
        }
      }
      junction.incidents.push_back(incident);
    }
    context.generation_backbone.junctions.push_back(std::move(junction));
  }
  std::sort(context.generation_backbone.junctions.begin(), context.generation_backbone.junctions.end(),
            [](const JunctionInfo& a, const JunctionInfo& b) { return a.node_id < b.node_id; });

  return phase;
}


BackboneDecisionPhaseOutput build_backbone_decision_phase(
    const CoreState& state, const BackboneGenerationRequestPlan& request_plan,
    const BackboneSupportChainPlan& support_chain_plan, BackboneTopologyPlan* plan) {
  if (plan == nullptr) {
    return {};
  }

  (void)request_plan;
  const EditState& edit_state = state.view().edit_state();
  const auto& route_neighbors_by_node = plan->route_neighbors_by_node;
  const auto& existing_node_position_by_id = plan->existing_node_position_by_id;

  auto current_support_position = [&](ObjectId node_id) -> Vec3d {
    if (const auto it = support_chain_plan.support_node_by_id.find(node_id); it != support_chain_plan.support_node_by_id.end()) {
      return it->second.position;
    }
    if (const Pole* pole = state.view().poles().find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    if (const auto it = existing_node_position_by_id.find(node_id); it != existing_node_position_by_id.end()) {
      return it->second;
    }
    return {};
  };
  auto combined_neighbors_for_node = [&](ObjectId node_id) {
    std::vector<ObjectId> neighbors{};
    for (const Span& span : edit_state.spans.items()) {
      const Port* port_a = edit_state.ports.find(span.port_a_id);
      const Port* port_b = edit_state.ports.find(span.port_b_id);
      if (port_a == nullptr || port_b == nullptr) {
        continue;
      }
      if (port_a->owner_pole_id == node_id && port_b->owner_pole_id != kInvalidObjectId && port_b->owner_pole_id != node_id &&
          std::find(neighbors.begin(), neighbors.end(), port_b->owner_pole_id) == neighbors.end()) {
        neighbors.push_back(port_b->owner_pole_id);
      }
      if (port_b->owner_pole_id == node_id && port_a->owner_pole_id != kInvalidObjectId && port_a->owner_pole_id != node_id &&
          std::find(neighbors.begin(), neighbors.end(), port_a->owner_pole_id) == neighbors.end()) {
        neighbors.push_back(port_a->owner_pole_id);
      }
    }
    if (const auto it_route = route_neighbors_by_node.find(node_id); it_route != route_neighbors_by_node.end()) {
      for (ObjectId neighbor_id : it_route->second) {
        if (std::find(neighbors.begin(), neighbors.end(), neighbor_id) == neighbors.end()) {
          neighbors.push_back(neighbor_id);
        }
      }
    }
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    return neighbors;
  };

  auto classify_junction = [&](ObjectId node_id) {
    const std::vector<ObjectId> route_neighbors =
        (route_neighbors_by_node.contains(node_id) ? route_neighbors_by_node.at(node_id) : std::vector<ObjectId>{});
    const std::vector<ObjectId> combined_neighbors = combined_neighbors_for_node(node_id);
    RawJunctionFacts facts{};
    facts.node_id = node_id;
    facts.route_incident_count = static_cast<int>(route_neighbors.size());
  facts.is_cross_like = combined_neighbors.size() >= 4 && facts.route_incident_count >= 2;

    if (combined_neighbors.empty()) {
      return build_junction_relation_from_facts(facts, {});
    }

    facts.candidates.reserve(combined_neighbors.size());
    const Vec3d center = current_support_position(node_id);
    for (ObjectId neighbor_id : combined_neighbors) {
      const Vec3d dir = normalize_forward_xy(current_support_position(neighbor_id) - center);
      if (!std::isfinite(dir.x) || !std::isfinite(dir.y)) {
        continue;
      }
      RawJunctionCandidate candidate{};
      candidate.neighbor_id = neighbor_id;
      candidate.dir = dir;
      candidate.is_route_neighbor =
          std::find(route_neighbors.begin(), route_neighbors.end(), neighbor_id) != route_neighbors.end();
      facts.candidates.push_back(candidate);
    }

    RouteThroughPairDecision through_pair{};
    auto adopt_through_pair = [&](ObjectId neighbor_a_id, ObjectId neighbor_b_id) {
      if (neighbor_a_id == kInvalidObjectId || neighbor_b_id == kInvalidObjectId || neighbor_a_id == neighbor_b_id ||
          find_raw_junction_candidate(facts, neighbor_a_id) == nullptr ||
          find_raw_junction_candidate(facts, neighbor_b_id) == nullptr) {
        return;
      }
      through_pair.neighbor_a_id = neighbor_a_id;
      through_pair.neighbor_b_id = neighbor_b_id;
      through_pair.straightness_score = raw_junction_pair_score(facts, neighbor_a_id, neighbor_b_id);
      through_pair.accepted = true;
    };
    std::vector<const RawJunctionCandidate*> through_pair_candidates{};
    through_pair_candidates.reserve(facts.candidates.size());
    if (facts.route_incident_count == 1) {
      for (const RawJunctionCandidate& candidate : facts.candidates) {
        if (!candidate.is_route_neighbor) {
          through_pair_candidates.push_back(&candidate);
        }
      }
    }
    if (through_pair_candidates.size() < 2) {
      through_pair_candidates.clear();
      for (const RawJunctionCandidate& candidate : facts.candidates) {
        through_pair_candidates.push_back(&candidate);
      }
    }
    for (std::size_t i = 0; i + 1 < through_pair_candidates.size(); ++i) {
      for (std::size_t j = i + 1; j < through_pair_candidates.size(); ++j) {
        const ObjectId neighbor_a_id = through_pair_candidates[i]->neighbor_id;
        const ObjectId neighbor_b_id = through_pair_candidates[j]->neighbor_id;
        const double score = raw_junction_pair_score(facts, neighbor_a_id, neighbor_b_id);
        if (score > through_pair.straightness_score + 1e-9 ||
            (std::abs(score - through_pair.straightness_score) <= 1e-9 &&
             std::pair<ObjectId, ObjectId>{std::min(neighbor_a_id, neighbor_b_id), std::max(neighbor_a_id, neighbor_b_id)} <
                 std::pair<ObjectId, ObjectId>{std::min(through_pair.neighbor_a_id, through_pair.neighbor_b_id),
                                              std::max(through_pair.neighbor_a_id, through_pair.neighbor_b_id)})) {
          adopt_through_pair(neighbor_a_id, neighbor_b_id);
        }
      }
    }
    return build_junction_relation_from_facts(facts, through_pair);
  };
  BackboneDecisionPhaseOutput phase = run_backbone_decision_phase({
      support_chain_plan.ordered_support_node_ids,
      plan->existing_prioritized_session_by_node,
      plan->existing_incident_session_by_node,
      plan->generation_backbone,
      support_chain_plan.session_id,
      classify_junction,
  });
  return phase;
}

} // namespace

EditResult<BackboneTopologyPlan> CoreState::build_backbone_topology_plan(
    const BackboneGenerationRequestPlan& request_plan, const BackboneSupportChainPlan& support_chain_plan) const {
  (void)request_plan;
  EditResult<BackboneTopologyPlan> result{};
  BackboneTopologyPlan plan{};
  plan.existing_network_backbone = BuildBackboneResult();

  struct ChainEdgeKey {
    ObjectId a = kInvalidObjectId;
    ObjectId b = kInvalidObjectId;
    bool operator==(const ChainEdgeKey& other) const { return a == other.a && b == other.b; }
  };
  struct ChainEdgeKeyHash {
    std::size_t operator()(const ChainEdgeKey& key) const {
      const std::size_t h1 = std::hash<ObjectId>{}(key.a);
      const std::size_t h2 = std::hash<ObjectId>{}(key.b);
      return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
    }
  };

  std::unordered_set<ChainEdgeKey, ChainEdgeKeyHash> unique_chain_edges{};
  for (std::size_t i = 0; i + 1 < support_chain_plan.ordered_support_node_ids.size(); ++i) {
    const ObjectId a = support_chain_plan.ordered_support_node_ids[i];
    const ObjectId b = support_chain_plan.ordered_support_node_ids[i + 1];
    if (a == kInvalidObjectId || b == kInvalidObjectId || a == b) {
      continue;
    }
    plan.route_neighbors_by_node[a].push_back(b);
    plan.route_neighbors_by_node[b].push_back(a);
    const ObjectId key_a = std::min(a, b);
    const ObjectId key_b = std::max(a, b);
    if (unique_chain_edges.insert({key_a, key_b}).second) {
      BackboneEdge edge{};
      edge.node_a = key_a;
      edge.node_b = key_b;
      plan.generation_backbone.edges.push_back(edge);
    }
  }
  for (auto& [_, neighbors] : plan.route_neighbors_by_node) {
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
  }
  std::sort(plan.generation_backbone.edges.begin(), plan.generation_backbone.edges.end(),
            [](const BackboneEdge& lhs, const BackboneEdge& rhs) {
              if (lhs.node_a != rhs.node_a) {
                return lhs.node_a < rhs.node_a;
              }
              return lhs.node_b < rhs.node_b;
            });

  plan.generation_backbone.nodes.reserve(support_chain_plan.support_node_by_id.size());
  for (const auto& [_, node] : support_chain_plan.support_node_by_id) {
    plan.generation_backbone.nodes.push_back(node);
  }
  std::sort(plan.generation_backbone.nodes.begin(), plan.generation_backbone.nodes.end(),
            [](const SupportNode& a, const SupportNode& b) { return a.node_id < b.node_id; });

  for (const JunctionInfo& junction : plan.existing_network_backbone.junctions) {
    plan.existing_prioritized_session_by_node[junction.node_id] = junction.prioritized_session_id;
    for (const JunctionIncident& incident : junction.incidents) {
      plan.existing_incident_session_by_node[junction.node_id][incident.neighbor_node_id] = incident.source_session_id;
    }
  }
  for (const SupportNode& node : plan.existing_network_backbone.nodes) {
    plan.existing_node_position_by_id[node.node_id] = node.position;
  }
  for (const Pole& pole : view().edit_state().poles.items()) {
    plan.existing_node_position_by_id.try_emplace(pole.id, pole.world_transform.position);
  }
  plan.decision_phase = build_backbone_decision_phase(*this, request_plan, support_chain_plan, &plan);

  result.value = std::move(plan);
  result.ok = true;
  return result;
}

} // namespace wire::core


