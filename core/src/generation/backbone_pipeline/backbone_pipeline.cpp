#include "backbone_pipeline.hpp"
#include "../detail_utils.hpp"
#include "../../pole_orientation_utils.hpp"
#include "wire/core/core_view.hpp"

#include <tuple>
#include <unordered_set>

namespace wire::core::generation::detail {

namespace {

InputDirection input_direction_from_debug(const PathDirectionEvaluationDebug& debug) {
  return (debug.chosen == PathDirectionChosen::kReverse) ? InputDirection::kReverse : InputDirection::kForward;
}

BuildDirection build_direction_from_input(InputDirection direction) {
  return (direction == InputDirection::kReverse) ? BuildDirection::kReverse : BuildDirection::kForward;
}

BackbonePair pair_from_through_pair(const ThroughPair& pair) {
  BackbonePair out{};
  if (!pair.accepted) {
    return out;
  }
  out.low = std::min(pair.neighbor_a_id, pair.neighbor_b_id);
  out.high = std::max(pair.neighbor_a_id, pair.neighbor_b_id);
  return out;
}

BackbonePair cross_pair_from_relation(const JunctionRelation& relation) {
  BackbonePair out{};
  ObjectId first = kInvalidObjectId;
  ObjectId second = kInvalidObjectId;
  for (const JunctionIncidentRelation& incident : relation.incidents) {
    if (incident.kind != JunctionRelationKind::kCrossUnderpass) {
      continue;
    }
    if (first == kInvalidObjectId) {
      first = incident.neighbor_node_id;
    } else if (second == kInvalidObjectId) {
      second = incident.neighbor_node_id;
      break;
    }
  }
  if (first == kInvalidObjectId || second == kInvalidObjectId || first == second) {
    return out;
  }
  out.low = std::min(first, second);
  out.high = std::max(first, second);
  return out;
}

Vec3d normalize_forward_xy_pipeline(const Vec3d& value) {
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
  relation.route_incident_count = facts.route_incident_count;
  relation.is_cross_like = facts.is_cross_like;
  relation.through_pair.neighbor_a_id = through_pair.neighbor_a_id;
  relation.through_pair.neighbor_b_id = through_pair.neighbor_b_id;
  relation.through_pair.straightness_score = through_pair.straightness_score;
  relation.through_pair.accepted = through_pair.accepted;
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
    } else if (through_pair.accepted) {
      incident.kind = facts.is_cross_like ? JunctionRelationKind::kCrossUnderpass : JunctionRelationKind::kSideBranch;
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
              return std::tuple<int, int, ObjectId>{lhs.in_through_pair ? 0 : 1, lhs.in_route ? 0 : 1, lhs.neighbor_node_id} <
                     std::tuple<int, int, ObjectId>{rhs.in_through_pair ? 0 : 1, rhs.in_route ? 0 : 1, rhs.neighbor_node_id};
            });
  for (const JunctionIncidentRelation& incident : relation.incidents) {
    if (incident.in_route) {
      relation.primary_neighbor_id = incident.neighbor_node_id;
      break;
    }
  }
  return relation;
}

int relation_rank(JunctionRelationKind kind) {
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
}

EdgeFlowInfo classify_edge_flow_from_relation(const JunctionRelation& relation, ObjectId peer_id) {
  EdgeFlowInfo info{};
  if (relation.incidents.size() < 3) {
    return info;
  }
  JunctionRelationKind relation_kind = JunctionRelationKind::kNone;
  for (const JunctionIncidentRelation& incident : relation.incidents) {
    if (incident.neighbor_node_id == peer_id) {
      relation_kind = incident.kind;
      break;
    }
  }
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
  case JunctionRelationKind::kThroughMain:
    info.kind = BackboneFlowKind::kMain;
    info.rule = relation.through_pair.accepted ? BackboneFlowDecisionRule::kJunctionOrderMain
                                               : BackboneFlowDecisionRule::kDefaultMain;
    return info;
  case JunctionRelationKind::kCornerContinuation:
  case JunctionRelationKind::kNone:
  default:
    return info;
  }
}

std::vector<ObjectId> combined_neighbors_for_node(const EditState& edit_state,
                                                  const std::unordered_map<ObjectId, std::vector<ObjectId>>& route_neighbors_by_node,
                                                  ObjectId node_id) {
  std::vector<ObjectId> neighbors{};
  for (const Span& span : edit_state.spans.items()) {
    const Port* port_a = edit_state.ports.find(span.port_a_id);
    const Port* port_b = edit_state.ports.find(span.port_b_id);
    if (port_a == nullptr || port_b == nullptr) {
      continue;
    }
    if (port_a->owner_pole_id == node_id && port_b->owner_pole_id != kInvalidObjectId && port_b->owner_pole_id != node_id) {
      neighbors.push_back(port_b->owner_pole_id);
    }
    if (port_b->owner_pole_id == node_id && port_a->owner_pole_id != kInvalidObjectId && port_a->owner_pole_id != node_id) {
      neighbors.push_back(port_a->owner_pole_id);
    }
  }
  if (const auto it_route = route_neighbors_by_node.find(node_id); it_route != route_neighbors_by_node.end()) {
    neighbors.insert(neighbors.end(), it_route->second.begin(), it_route->second.end());
  }
  std::sort(neighbors.begin(), neighbors.end());
  neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
  return neighbors;
}

} // namespace

EditResult<BackboneBuilderOutput> BackboneBuilder::build(const BackboneSpec& spec) const {
  EditResult<BackboneBuilderOutput> result{};
  EditResult<BackboneGenerationRequestPlan> request_out = build_backbone_generation_request_plan(state_, spec);
  if (!request_out.ok) {
    result.error = request_out.error;
    return result;
  }
  EditResult<BackboneSupportChainPlan> support_chain_out = state_.build_backbone_support_chain_plan(request_out.value);
  if (!support_chain_out.ok) {
    result.error = support_chain_out.error;
    return result;
  }

  BackboneBuilderOutput output{};
  output.request = std::move(request_out.value);
  output.support_chain = std::move(support_chain_out.value);
  output.backbone.input_direction = input_direction_from_debug(output.request.direction_debug);
  output.backbone.build_direction = build_direction_from_input(output.backbone.input_direction);

  output.backbone.nodes.reserve(output.support_chain.support_node_by_id.size());
  for (const auto& [_, node] : output.support_chain.support_node_by_id) {
    output.backbone.nodes.push_back(node);
  }
  std::sort(output.backbone.nodes.begin(), output.backbone.nodes.end(),
            [](const SupportNode& lhs, const SupportNode& rhs) { return lhs.node_id < rhs.node_id; });

  for (std::size_t i = 0; i + 1 < output.support_chain.ordered_support_node_ids.size(); ++i) {
    const ObjectId node_a = output.support_chain.ordered_support_node_ids[i];
    const ObjectId node_b = output.support_chain.ordered_support_node_ids[i + 1];
    if (node_a == kInvalidObjectId || node_b == kInvalidObjectId || node_a == node_b) {
      continue;
    }
    BackboneEdge edge{};
    edge.node_a = node_a;
    edge.node_b = node_b;
    output.backbone.edges.push_back(edge);
  }

  result.value = std::move(output);
  result.ok = true;
  return result;
}

EditResult<JunctionRoleResolverOutput> JunctionRoleResolver::resolve(const BackboneBuilderOutput& builder_output) const {
  EditResult<JunctionRoleResolverOutput> result{};
  JunctionRoleResolverOutput output{};
  output.topology.existing_network_backbone = state_.BuildBackboneResult();
  output.topology.generation_backbone.nodes = builder_output.backbone.nodes;
  output.topology.generation_backbone.edges = builder_output.backbone.edges;
  for (std::size_t i = 0; i + 1 < builder_output.support_chain.ordered_support_node_ids.size(); ++i) {
    const ObjectId a = builder_output.support_chain.ordered_support_node_ids[i];
    const ObjectId b = builder_output.support_chain.ordered_support_node_ids[i + 1];
    if (a == kInvalidObjectId || b == kInvalidObjectId || a == b) {
      continue;
    }
    output.topology.route_neighbors_by_node[a].push_back(b);
    output.topology.route_neighbors_by_node[b].push_back(a);
  }
  for (auto& [_, neighbors] : output.topology.route_neighbors_by_node) {
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
  }
  for (const JunctionInfo& junction : output.topology.existing_network_backbone.junctions) {
    output.topology.existing_prioritized_session_by_node[junction.node_id] = junction.prioritized_session_id;
    for (const JunctionIncident& incident : junction.incidents) {
      output.topology.existing_incident_session_by_node[junction.node_id][incident.neighbor_node_id] =
          incident.source_session_id;
    }
  }
  for (const SupportNode& node : output.topology.existing_network_backbone.nodes) {
    output.topology.existing_node_position_by_id[node.node_id] = node.position;
  }
  for (const Pole& pole : state_.view().edit_state().poles.items()) {
    output.topology.existing_node_position_by_id.try_emplace(pole.id, pole.world_transform.position);
  }

  auto current_support_position = [&](ObjectId node_id) -> Vec3d {
    if (const auto it = builder_output.support_chain.support_node_by_id.find(node_id);
        it != builder_output.support_chain.support_node_by_id.end()) {
      return it->second.position;
    }
    if (const Pole* pole = state_.view().poles().find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    if (const auto it = output.topology.existing_node_position_by_id.find(node_id);
        it != output.topology.existing_node_position_by_id.end()) {
      return it->second;
    }
    return {};
  };

  const EditState& edit_state = state_.view().edit_state();
  for (ObjectId node_id : builder_output.support_chain.ordered_support_node_ids) {
    const std::vector<ObjectId> route_neighbors =
        output.topology.route_neighbors_by_node.contains(node_id) ? output.topology.route_neighbors_by_node.at(node_id)
                                                                  : std::vector<ObjectId>{};
    const std::vector<ObjectId> combined_neighbors =
        combined_neighbors_for_node(edit_state, output.topology.route_neighbors_by_node, node_id);
    RawJunctionFacts facts{};
    facts.node_id = node_id;
    facts.route_incident_count = static_cast<int>(route_neighbors.size());
    facts.is_cross_like = combined_neighbors.size() >= 4 && facts.route_incident_count >= 2;
    const Vec3d center = current_support_position(node_id);
    for (ObjectId neighbor_id : combined_neighbors) {
      RawJunctionCandidate candidate{};
      candidate.neighbor_id = neighbor_id;
      candidate.dir = normalize_forward_xy_pipeline(current_support_position(neighbor_id) - center);
      candidate.is_route_neighbor =
          std::find(route_neighbors.begin(), route_neighbors.end(), neighbor_id) != route_neighbors.end();
      if (!std::isfinite(candidate.dir.x) || !std::isfinite(candidate.dir.y)) {
        continue;
      }
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

    std::vector<const RawJunctionCandidate*> route_candidates{};
    std::vector<const RawJunctionCandidate*> non_route_candidates{};
    for (const RawJunctionCandidate& candidate : facts.candidates) {
      (candidate.is_route_neighbor ? route_candidates : non_route_candidates).push_back(&candidate);
    }
    if (facts.is_cross_like && facts.route_incident_count == 2 && non_route_candidates.size() == 2) {
      adopt_through_pair(non_route_candidates[0]->neighbor_id, non_route_candidates[1]->neighbor_id);
    } else if (facts.route_incident_count == 1 && non_route_candidates.size() == 2) {
      adopt_through_pair(non_route_candidates[0]->neighbor_id, non_route_candidates[1]->neighbor_id);
    } else {
      std::vector<const RawJunctionCandidate*> pair_candidates =
          (facts.route_incident_count == 1 && non_route_candidates.size() >= 2) ? non_route_candidates
                                                                                 : std::vector<const RawJunctionCandidate*>{};
      if (pair_candidates.size() < 2) {
        for (const RawJunctionCandidate& candidate : facts.candidates) {
          pair_candidates.push_back(&candidate);
        }
      }
      for (std::size_t i = 0; i + 1 < pair_candidates.size(); ++i) {
        for (std::size_t j = i + 1; j < pair_candidates.size(); ++j) {
          const ObjectId a = pair_candidates[i]->neighbor_id;
          const ObjectId b = pair_candidates[j]->neighbor_id;
          const double score = raw_junction_pair_score(facts, a, b);
          if (score > through_pair.straightness_score + 1e-9 ||
              (std::abs(score - through_pair.straightness_score) <= 1e-9 &&
               std::pair<ObjectId, ObjectId>{std::min(a, b), std::max(a, b)} <
                   std::pair<ObjectId, ObjectId>{std::min(through_pair.neighbor_a_id, through_pair.neighbor_b_id),
                                                std::max(through_pair.neighbor_a_id, through_pair.neighbor_b_id)})) {
            adopt_through_pair(a, b);
          }
        }
      }
    }

    JunctionRelation relation = build_junction_relation_from_facts(facts, through_pair);
    output.roles.by_node[node_id] = relation;
    output.roles.ordered.push_back(relation);
    output.roles.main_pair_by_node[node_id] = pair_from_through_pair(relation.through_pair);
    output.roles.cross_pair_by_node[node_id] = cross_pair_from_relation(relation);
  }

  for (std::size_t i = 0; i + 1 < builder_output.support_chain.ordered_support_node_ids.size(); ++i) {
    const ObjectId node_a = builder_output.support_chain.ordered_support_node_ids[i];
    const ObjectId node_b = builder_output.support_chain.ordered_support_node_ids[i + 1];
    const JunctionRelation& relation_a = output.roles.by_node[node_a];
    const JunctionRelation& relation_b = output.roles.by_node[node_b];
    const EdgeFlowInfo flow_a = classify_edge_flow_from_relation(relation_a, node_b);
    const EdgeFlowInfo flow_b = classify_edge_flow_from_relation(relation_b, node_a);
    if (flow_a.kind == BackboneFlowKind::kBranch && flow_b.kind != BackboneFlowKind::kBranch) {
      output.roles.edge_flow_by_segment.push_back(flow_a);
    } else if (flow_b.kind == BackboneFlowKind::kBranch && flow_a.kind != BackboneFlowKind::kBranch) {
      output.roles.edge_flow_by_segment.push_back(flow_b);
    } else if (flow_a.rule != BackboneFlowDecisionRule::kDefaultMain &&
               flow_b.rule == BackboneFlowDecisionRule::kDefaultMain) {
      output.roles.edge_flow_by_segment.push_back(flow_a);
    } else if (flow_b.rule != BackboneFlowDecisionRule::kDefaultMain &&
               flow_a.rule == BackboneFlowDecisionRule::kDefaultMain) {
      output.roles.edge_flow_by_segment.push_back(flow_b);
    } else {
      EdgeFlowInfo info{};
      info.kind = BackboneFlowKind::kMain;
      info.rule = (flow_a.rule == BackboneFlowDecisionRule::kJunctionOrderMain ||
                   flow_b.rule == BackboneFlowDecisionRule::kJunctionOrderMain)
                      ? BackboneFlowDecisionRule::kJunctionOrderMain
                      : BackboneFlowDecisionRule::kDefaultMain;
      output.roles.edge_flow_by_segment.push_back(info);
    }
  }
  for (const JunctionRelation& relation : output.roles.ordered) {
    if (relation.incidents.size() < 3) {
      continue;
    }
    JunctionInfo junction{};
    junction.node_id = relation.node_id;
    junction.prioritized_session_id = output.topology.existing_prioritized_session_by_node.contains(relation.node_id)
                                          ? output.topology.existing_prioritized_session_by_node.at(relation.node_id)
                                          : builder_output.support_chain.session_id;
    for (std::size_t index = 0; index < relation.incidents.size(); ++index) {
      const JunctionIncidentRelation& source = relation.incidents[index];
      JunctionIncident incident{};
      incident.neighbor_node_id = source.neighbor_node_id;
      incident.order = static_cast<int>(index);
      incident.primary = (index == 0);
      incident.source_session_id = builder_output.support_chain.session_id;
      if (const auto it_existing_node = output.topology.existing_incident_session_by_node.find(relation.node_id);
          it_existing_node != output.topology.existing_incident_session_by_node.end()) {
        if (const auto it_existing_source = it_existing_node->second.find(source.neighbor_node_id);
            it_existing_source != it_existing_node->second.end()) {
          incident.source_session_id = it_existing_source->second;
        }
      }
      junction.incidents.push_back(incident);
    }
    output.topology.generation_backbone.junctions.push_back(std::move(junction));
  }
  std::sort(output.topology.generation_backbone.junctions.begin(), output.topology.generation_backbone.junctions.end(),
            [](const JunctionInfo& lhs, const JunctionInfo& rhs) { return lhs.node_id < rhs.node_id; });
  result.value = std::move(output);
  result.ok = true;
  return result;
}

EditResult<PoleFacing> PoleFacingResolver::resolve(const BackboneBuilderOutput& builder_output,
                                                   const JunctionRoleResolverOutput& role_output) const {
  EditResult<PoleFacing> result{};
  std::unordered_map<ObjectId, BackboneOrientationNodeContext> orientation_context_by_node{};
  const EditState& edit_state = state_.edit_state_access();
  const auto& ordered_support_node_ids = builder_output.support_chain.ordered_support_node_ids;
  const auto& support_node_by_id = builder_output.support_chain.support_node_by_id;
  const auto& existing_node_position_by_id = role_output.topology.existing_node_position_by_id;
  const auto& junction_relations_by_node = role_output.roles.by_node;
  const std::vector<BackboneBundlePlan>& active_bundle_plans = builder_output.request.active_bundle_plans;

  auto current_support_position = [&](ObjectId node_id) -> Vec3d {
    if (const auto it = support_node_by_id.find(node_id); it != support_node_by_id.end()) {
      return it->second.position;
    }
    if (const Pole* pole = state_.view().poles().find(node_id); pole != nullptr) {
      return pole->world_transform.position;
    }
    if (const auto it = existing_node_position_by_id.find(node_id); it != existing_node_position_by_id.end()) {
      return it->second;
    }
    return {};
  };
  auto connected_neighbors_for_support_axis = [&](ObjectId node_id) {
    std::vector<ObjectId> neighbors{};
    for (const Span& span : edit_state.spans.items()) {
      const Port* port_a = edit_state.ports.find(span.port_a_id);
      const Port* port_b = edit_state.ports.find(span.port_b_id);
      if (port_a == nullptr || port_b == nullptr) {
        continue;
      }
      if (port_a->owner_pole_id == node_id && port_b->owner_pole_id != kInvalidObjectId && port_b->owner_pole_id != node_id) {
        neighbors.push_back(port_b->owner_pole_id);
      }
      if (port_b->owner_pole_id == node_id && port_a->owner_pole_id != kInvalidObjectId && port_a->owner_pole_id != node_id) {
        neighbors.push_back(port_a->owner_pole_id);
      }
    }
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    return neighbors;
  };
  struct BundleCategoryKey {
    ConnectionCategory category = ConnectionCategory::kLowVoltage;
    BundleKind bundle_template_id = BundleKind::kLowVoltage;
  };
  BundleCategoryKey route_bundle_category_key{};
  bool has_route_bundle_category_key = false;
  for (const BackboneBundlePlan& bundle_plan : active_bundle_plans) {
    const BundleCategoryKey candidate_key{bundle_plan.category, bundle_plan.template_id};
    if (!has_route_bundle_category_key ||
        static_cast<int>(candidate_key.category) < static_cast<int>(route_bundle_category_key.category) ||
        (candidate_key.category == route_bundle_category_key.category &&
         static_cast<int>(candidate_key.bundle_template_id) < static_cast<int>(route_bundle_category_key.bundle_template_id))) {
      route_bundle_category_key = candidate_key;
      has_route_bundle_category_key = true;
    }
  }
  auto edge_hash = [](ObjectId node_a, ObjectId node_b) {
    const std::uint64_t lo = static_cast<std::uint64_t>(std::min(node_a, node_b));
    const std::uint64_t hi = static_cast<std::uint64_t>(std::max(node_a, node_b));
    return lo ^ (hi + 0x9E3779B97F4A7C15ull + (lo << 6) + (lo >> 2));
  };
  std::unordered_map<std::uint64_t, BundleCategoryKey> bundle_category_key_by_edge{};
  for (const Span& span : edit_state.spans.items()) {
    const Port* port_a = edit_state.ports.find(span.port_a_id);
    const Port* port_b = edit_state.ports.find(span.port_b_id);
    const Bundle* bundle = edit_state.bundles.find(span.bundle_id);
    if (port_a == nullptr || port_b == nullptr || bundle == nullptr) {
      continue;
    }
    if (port_a->owner_pole_id == kInvalidObjectId || port_b->owner_pole_id == kInvalidObjectId ||
        port_a->owner_pole_id == port_b->owner_pole_id) {
      continue;
    }
    const BundleTemplate* bundle_template = state_.find_bundle_template(bundle->bundle_template_id);
    if (bundle_template == nullptr) {
      continue;
    }
    const BundleCategoryKey candidate_key{bundle_template->category, bundle->bundle_template_id};
    const std::uint64_t edge = edge_hash(port_a->owner_pole_id, port_b->owner_pole_id);
    const auto it_existing = bundle_category_key_by_edge.find(edge);
    if (it_existing == bundle_category_key_by_edge.end()) {
      bundle_category_key_by_edge.emplace(edge, candidate_key);
    } else {
      BundleCategoryKey& current_key = it_existing->second;
      if (static_cast<int>(candidate_key.category) < static_cast<int>(current_key.category) ||
          (candidate_key.category == current_key.category &&
           static_cast<int>(candidate_key.bundle_template_id) < static_cast<int>(current_key.bundle_template_id))) {
        current_key = candidate_key;
      }
    }
  }
  auto bundle_category_key_for_neighbor = [&](ObjectId node_id, ObjectId neighbor_id) {
    const auto it = bundle_category_key_by_edge.find(edge_hash(node_id, neighbor_id));
    return (it == bundle_category_key_by_edge.end()) ? BundleCategoryKey{} : it->second;
  };
  auto row_layout_axis_mode_for_key = [&](const BundleCategoryKey& key) {
    const BundleTemplate* bundle_template = state_.find_bundle_template(key.bundle_template_id);
    return (bundle_template != nullptr) ? bundle_template->row_layout_axis_mode : RowLayoutAxisMode::kPoleYaw;
  };
  auto row_layout_axis_key_for_node = [&](ObjectId node_id) {
    if (row_layout_axis_mode_for_key(route_bundle_category_key) == RowLayoutAxisMode::kSupportAxis) {
      return route_bundle_category_key;
    }
    for (ObjectId neighbor_id : connected_neighbors_for_support_axis(node_id)) {
      const BundleCategoryKey candidate = bundle_category_key_for_neighbor(node_id, neighbor_id);
      if (row_layout_axis_mode_for_key(candidate) == RowLayoutAxisMode::kSupportAxis) {
        return candidate;
      }
    }
    return BundleCategoryKey{};
  };
  auto neighbor_direction = [&](ObjectId node_id, ObjectId neighbor_id) {
    return normalize_forward_xy_pipeline(current_support_position(neighbor_id) - current_support_position(node_id));
  };
  auto choose_continuous_axis = [&](const Vec3d& axis, const Vec3d& previous_forward) {
    Vec3d out = axis;
    if (!Normalize(&out)) {
      return Vec3d{};
    }
    Vec3d prev = normalize_forward_xy_pipeline(previous_forward);
    if (Dot(out, prev) < 0.0) {
      out = ScaleVec(out, -1.0);
    }
    return out;
  };
  auto choose_support_axis_for_layout = [&](const BackboneOrientationNodeContext& node_context) {
    struct SupportAxisSelection {
      Vec3d axis{};
      bool available = false;
      PoleSupportAxisRule rule = PoleSupportAxisRule::kFallback;
      ObjectId primary_neighbor_id = kInvalidObjectId;
      ObjectId secondary_neighbor_id = kInvalidObjectId;
    };
    SupportAxisSelection selection{};
    auto adopt_axis = [&](const Vec3d& axis, PoleSupportAxisRule rule, ObjectId primary_neighbor_id,
                          ObjectId secondary_neighbor_id) {
      Vec3d normalized_axis = normalize_forward_xy_pipeline(axis);
      if (!Normalize(&normalized_axis)) {
        return false;
      }
      Vec3d row_axis = ComputeLateralAxis(normalized_axis);
      if (!Normalize(&row_axis)) {
        return false;
      }
      selection.axis = choose_continuous_axis(row_axis, node_context.previous_support_axis);
      selection.available = Normalize(&selection.axis);
      if (selection.available) {
        selection.rule = rule;
        selection.primary_neighbor_id = primary_neighbor_id;
        selection.secondary_neighbor_id = secondary_neighbor_id;
      }
      return selection.available;
    };
    if (node_context.continuation_pair.available) {
      if (adopt_axis(node_context.continuation_pair.primary_direction, PoleSupportAxisRule::kMainChainPair,
                     node_context.continuation_pair.primary_neighbor_id,
                     node_context.continuation_pair.secondary_neighbor_id)) {
        return selection;
      }
    }
    if (node_context.primary_neighbor.neighbor_id != kInvalidObjectId && node_context.primary_neighbor.available) {
      const PoleSupportAxisRule rule =
          node_context.has_active_junction ? PoleSupportAxisRule::kPrimaryIncident : PoleSupportAxisRule::kMainChainSingle;
      adopt_axis(node_context.primary_neighbor.direction, rule, node_context.primary_neighbor.neighbor_id, kInvalidObjectId);
    }
    return selection;
  };

  for (ObjectId node_id : ordered_support_node_ids) {
    BackboneOrientationNodeContext context{};
    context.center = current_support_position(node_id);
    if (const Pole* pole = edit_state.poles.find(node_id); pole != nullptr) {
      context.previous_forward = RotateAroundWorldUpDeg(WorldForward(), state_.effective_pole_yaw_deg(*pole));
      context.previous_layout_yaw = state_.effective_pole_layout_yaw_deg(*pole);
      if (const auto it_prev_debug = state_.debug_.pole_orientation_debug_records.find(pole->id);
          it_prev_debug != state_.debug_.pole_orientation_debug_records.end()) {
        context.previous_support_axis = it_prev_debug->second.adopted_support_axis;
      } else {
        context.previous_support_axis = side_axis_from_yaw_deg(context.previous_layout_yaw);
      }
    }
    if (const auto it = junction_relations_by_node.find(node_id); it != junction_relations_by_node.end()) {
      const JunctionRelation& relation = it->second;
      context.has_active_junction = relation.incidents.size() >= 3;
      context.primary_neighbor.neighbor_id = relation.primary_neighbor_id;
      if (relation.through_pair.accepted) {
        context.continuation_pair.primary_neighbor_id = relation.through_pair.neighbor_a_id;
        context.continuation_pair.secondary_neighbor_id = relation.through_pair.neighbor_b_id;
      }
    }
    if (context.primary_neighbor.neighbor_id != kInvalidObjectId) {
      context.primary_neighbor.direction = neighbor_direction(node_id, context.primary_neighbor.neighbor_id);
      context.primary_neighbor.available = Normalize(&context.primary_neighbor.direction);
    }
    if (context.continuation_pair.primary_neighbor_id != kInvalidObjectId &&
        context.continuation_pair.secondary_neighbor_id != kInvalidObjectId) {
      context.continuation_pair.primary_direction = neighbor_direction(node_id, context.continuation_pair.primary_neighbor_id);
      context.continuation_pair.secondary_direction =
          neighbor_direction(node_id, context.continuation_pair.secondary_neighbor_id);
      context.continuation_pair.available = Normalize(&context.continuation_pair.primary_direction) &&
                                            Normalize(&context.continuation_pair.secondary_direction);
    }
    const BundleCategoryKey key = row_layout_axis_key_for_node(node_id);
    context.row_layout_axis_selection = {row_layout_axis_mode_for_key(key), key.category};
    const auto support_axis_selection = choose_support_axis_for_layout(context);
    context.chosen_support_axis = support_axis_selection.axis;
    context.has_chosen_support_axis = support_axis_selection.available;
    context.support_axis_rule = support_axis_selection.rule;
    context.support_axis_primary_neighbor_id = support_axis_selection.primary_neighbor_id;
    context.support_axis_secondary_neighbor_id = support_axis_selection.secondary_neighbor_id;
    orientation_context_by_node.emplace(node_id, std::move(context));
  }

  PoleFacing facing{};
  facing.by_node = build_backbone_pole_orientation_plan(ordered_support_node_ids, orientation_context_by_node);
  result.value = std::move(facing);
  result.ok = true;
  return result;
}

EditResult<BundleSpanBuilderOutput> BundleSpanBuilder::build(const BackboneBuilderOutput& builder_output,
                                                             const JunctionRoleResolverOutput& role_output,
                                                             const PoleFacing& pole_facing) {
  EditResult<BundleSpanBuilderOutput> result{};
  EditResult<RealizedBackboneSupport> support_chain_out = state_.build_real_backbone_support(builder_output.support_chain);
  if (!support_chain_out.ok) {
    result.error = support_chain_out.error;
    return result;
  }

  EditResult<BackboneRuntimeState> runtime_out = state_.remap_backbone_build_to_real_nodes(
      role_output.topology, role_output.roles, pole_facing, support_chain_out.value.session_id,
      std::move(support_chain_out.value.ordered_support_node_ids), std::move(support_chain_out.value.support_node_by_id),
      std::move(support_chain_out.value.real_node_id_by_input_node_id));
  if (!runtime_out.ok) {
    result.error = runtime_out.error;
    return result;
  }

  state_.apply_backbone_pole_facing(&runtime_out.value, &support_chain_out.value.change_set);

  EditResult<GeneratedBackboneSpans> spans_out = state_.build_bundle_spans_for_backbone(builder_output.request, runtime_out.value);
  if (!spans_out.ok) {
    result.error = spans_out.error;
    return result;
  }

  BundleSpanBuilderOutput output{};
  output.runtime = std::move(runtime_out.value);
  output.spans = std::move(spans_out.value);
  output.generated_pole_ids = std::move(support_chain_out.value.generated_pole_ids);
  output.change_set = std::move(support_chain_out.value.change_set);
  append_change_set(output.change_set, output.spans.change_set);
  result.value = std::move(output);
  result.ok = true;
  return result;
}

SpanLayoutRules SpanLayoutRuleBuilder::build(const std::vector<ObjectId>& span_ids) const {
  SpanLayoutRules rules{};
  rules.endpoints.reserve(span_ids.size());
  for (ObjectId span_id : span_ids) {
    const SpanSupportLayoutAuthorityView authority = state_.support_layout_contract(span_id).authority;
    if (!authority.has_authority()) {
      continue;
    }
    EndpointLayoutRule rule{};
    rule.span_id = span_id;
    rule.start = authority.seed->start;
    rule.end = authority.seed->end;
    rules.endpoints.push_back(std::move(rule));
  }
  return rules;
}

EditResult<bool> BackbonePipeline::prepare() {
  EditResult<bool> result{};
  BackboneBuilder builder(state_);
  EditResult<BackboneBuilderOutput> builder_out = builder.build(spec_);
  if (!builder_out.ok) {
    result.error = builder_out.error;
    return result;
  }
  builder_output_ = std::move(builder_out.value);
  prepared_ = true;
  result.value = true;
  result.ok = true;
  return result;
}

EditResult<bool> BackbonePipeline::check() const {
  EditResult<bool> result{};
  if (!prepared_) {
    result.error = "backbone pipeline is not prepared";
    return result;
  }
  if (builder_output_.request.request.path.polyline.size() < 2) {
    result.error = "backbone pipeline is missing path points";
    return result;
  }
  if (builder_output_.request.active_bundle_plans.empty()) {
    result.error = "backbone pipeline is missing active bundles";
    return result;
  }
  if (builder_output_.backbone.nodes.size() < 2 || builder_output_.backbone.edges.empty()) {
    result.error = "backbone pipeline is missing backbone graph";
    return result;
  }
  result.value = true;
  result.ok = true;
  return result;
}

EditResult<GenerateBundleFromPathResult> BackbonePipeline::build() {
  EditResult<GenerateBundleFromPathResult> result{};
  if (!prepared_) {
    result.error = "backbone pipeline is not prepared";
    return result;
  }
  state_.save_path_direction_debug(builder_output_.request);
  if (builder_output_.request.active_bundle_plans.empty()) {
    result.ok = true;
    return result;
  }

  JunctionRoleResolver role_resolver(state_);
  EditResult<JunctionRoleResolverOutput> role_out = role_resolver.resolve(builder_output_);
  if (!role_out.ok) {
    result.error = role_out.error;
    return result;
  }
  role_output_ = std::move(role_out.value);

  PoleFacingResolver facing_resolver(state_);
  EditResult<PoleFacing> facing_out = facing_resolver.resolve(builder_output_, role_output_);
  if (!facing_out.ok) {
    result.error = facing_out.error;
    return result;
  }
  pole_facing_ = std::move(facing_out.value);

  BundleSpanBuilder span_builder(state_);
  EditResult<BundleSpanBuilderOutput> spans_out = span_builder.build(builder_output_, role_output_, pole_facing_);
  if (!spans_out.ok) {
    result.error = spans_out.error;
    return result;
  }

  result.change_set = std::move(spans_out.value.change_set);
  result.value.generated_pole_ids = std::move(spans_out.value.generated_pole_ids);
  result.value.bundle_ids = spans_out.value.spans.bundle_ids;
  result.value.generated_span_ids = spans_out.value.spans.generated_span_ids;
  result.value.bundle_id = spans_out.value.spans.primary_bundle_id;

  SpanLayoutRuleBuilder rules_builder(state_);
  const SpanLayoutRules rules = rules_builder.build(result.value.generated_span_ids);
  (void)rules;
  state_.publish_backbone_debug_state(spans_out.value.runtime, &spans_out.value.spans);
  result.ok = true;
  return result;
}

} // namespace wire::core::generation::detail

namespace wire::core {

void CoreState::apply_backbone_pole_facing(
    generation::detail::BackboneRuntimeState* runtime, ChangeSet* change_set) {
  if (runtime == nullptr || change_set == nullptr) {
    return;
  }

  debug_.pole_orientation_debug_records.clear();
  generation::detail::BackbonePoleOrientationApplyInput input{
      runtime->ordered_support_node_ids,
      runtime->pole_facing.by_node,
      debug_.pole_orientation_debug_records,
      [&](ObjectId pole_id) -> Pole* { return edit_state_access().poles.find(pole_id); },
      [&](ObjectId pole_id) -> bool { return has_pole_orientation_override(pole_id); },
      [&](ObjectId pole_id, const Pole& old_pole, const PortLayoutYawOverride* previous_override) {
        refresh_owned_endpoints_from_pole(pole_id, change_set, &old_pole, previous_override);
      },
      [&](ObjectId pole_id, const Pole& old_pole) { finalize_pole_transform_update(pole_id, old_pole, change_set); }};
  apply_backbone_pole_orientation_plan(input);
}

void CoreState::publish_backbone_debug_state(
    const generation::detail::BackboneRuntimeState& runtime,
    generation::detail::GeneratedBackboneSpans* spans) {
  if (spans == nullptr) {
    return;
  }

  debug_.last_generation_support_nodes.clear();
  debug_.last_generation_support_nodes.reserve(runtime.topology.generation_backbone.nodes.size());
  for (const SupportNode& node : runtime.topology.generation_backbone.nodes) {
    if (node.support_kind != SupportKind::kPole) {
      debug_.last_generation_support_nodes.push_back(node);
    }
  }
  debug_.last_generation_lane_assignments = std::move(spans->lane_assignments);
  debug_.last_generation_edge_orientations = std::move(spans->edge_orientations);
  debug_.last_generation_junction_relations = std::move(spans->junctions);
}

void CoreState::save_path_direction_debug(const generation::detail::BackboneGenerationRequestPlan& build_request) {
  debug_.last_path_direction_debug = build_request.direction_debug;
  path_direction_debug_records_access().push_back(build_request.direction_debug);
  if (path_direction_debug_records_access().size() > 128) {
    path_direction_debug_records_access().erase(path_direction_debug_records_access().begin());
  }
}

} // namespace wire::core
