#include "backbone_pipeline.hpp"
#include "../build_backbone/build_span_layout_rules.hpp"
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

struct RouteThroughPairDecision {
  ObjectId neighbor_a_id = kInvalidObjectId;
  ObjectId neighbor_b_id = kInvalidObjectId;
  double straightness_score = -2.0;
  bool accepted = false;
};

std::vector<const JunctionInputIncident*> all_junction_incidents(const JunctionInputFacts& facts) {
  std::vector<const JunctionInputIncident*> incidents{};
  incidents.reserve(facts.route_incidents.size() + facts.external_incidents.size());
  for (const JunctionInputIncident& incident : facts.route_incidents) {
    incidents.push_back(&incident);
  }
  for (const JunctionInputIncident& incident : facts.external_incidents) {
    incidents.push_back(&incident);
  }
  return incidents;
}

const JunctionInputIncident* find_junction_incident(const JunctionInputFacts& facts, ObjectId neighbor_id) {
  for (const JunctionInputIncident* incident : all_junction_incidents(facts)) {
    if (incident != nullptr && incident->neighbor_node_id == neighbor_id) {
      return incident;
    }
  }
  return nullptr;
}

double junction_pair_score(const JunctionInputFacts& facts, ObjectId neighbor_a_id, ObjectId neighbor_b_id) {
  const JunctionInputIncident* incident_a = find_junction_incident(facts, neighbor_a_id);
  const JunctionInputIncident* incident_b = find_junction_incident(facts, neighbor_b_id);
  if (incident_a == nullptr || incident_b == nullptr) {
    return -2.0;
  }
  return Dot(incident_a->direction, Vec3d{-incident_b->direction.x, -incident_b->direction.y, -incident_b->direction.z});
}

JunctionRelation build_junction_relation_from_selection(const JunctionInputFacts& facts,
                                                    const RouteThroughPairDecision& through_pair) {
  JunctionRelation relation{};
  relation.node_id = facts.node_id;
  relation.route_incident_count = static_cast<int>(facts.route_incidents.size());
  relation.is_cross_like = facts.route_incidents.size() >= 2 && facts.external_incidents.size() >= 2;
  relation.through_pair.neighbor_a_id = through_pair.neighbor_a_id;
  relation.through_pair.neighbor_b_id = through_pair.neighbor_b_id;
  relation.through_pair.straightness_score = through_pair.straightness_score;
  relation.through_pair.accepted = through_pair.accepted;
  auto in_through_pair = [&](ObjectId neighbor_id) {
    return through_pair.accepted &&
           (neighbor_id == through_pair.neighbor_a_id || neighbor_id == through_pair.neighbor_b_id);
  };
  auto append_incident = [&](const JunctionInputIncident& source, bool in_route) {
    JunctionIncidentRelation incident{};
    incident.neighbor_node_id = source.neighbor_node_id;
    incident.in_route = in_route;
    incident.straightness_score = through_pair.straightness_score;
    incident.in_through_pair = in_through_pair(source.neighbor_node_id);
    if (incident.in_through_pair) {
      incident.kind = JunctionRelationKind::kThroughMain;
    } else if (through_pair.accepted) {
      incident.kind = relation.is_cross_like ? JunctionRelationKind::kCrossUnderpass : JunctionRelationKind::kSideBranch;
    } else if (in_route) {
      incident.kind = (facts.route_incidents.size() >= 2) ? JunctionRelationKind::kCornerContinuation
                                                          : JunctionRelationKind::kSideBranch;
    } else {
      incident.kind = JunctionRelationKind::kSideBranch;
    }
    relation.incidents.push_back(incident);
  };
  for (const JunctionInputIncident& incident : facts.route_incidents) {
    append_incident(incident, true);
  }
  for (const JunctionInputIncident& incident : facts.external_incidents) {
    append_incident(incident, false);
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

Vec3d junction_axis_from_pair(const JunctionInputFacts& facts, const BackbonePair& pair) {
  if (!pair.valid()) {
    return {};
  }
  const JunctionInputIncident* low = find_junction_incident(facts, pair.low);
  const JunctionInputIncident* high = find_junction_incident(facts, pair.high);
  if (low == nullptr || high == nullptr) {
    return {};
  }
  Vec3d axis = low->direction + ScaleVec(high->direction, -1.0);
  if (!Normalize(&axis)) {
    axis = low->direction;
  }
  return normalize_forward_xy_pipeline(axis);
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

EditResult<JunctionInputBuilderOutput> JunctionInputBuilder::build(const BackboneBuilderOutput& builder_output) const {
  EditResult<JunctionInputBuilderOutput> result{};
  JunctionInputBuilderOutput output{};
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

  for (ObjectId node_id : builder_output.support_chain.ordered_support_node_ids) {
    const std::vector<ObjectId> route_neighbors =
        output.topology.route_neighbors_by_node.contains(node_id) ? output.topology.route_neighbors_by_node.at(node_id)
                                                                  : std::vector<ObjectId>{};
    JunctionInputFacts facts{};
    facts.node_id = node_id;
    facts.build_direction = builder_output.backbone.build_direction;
    const Vec3d center = current_support_position(node_id);
    facts.support_position = center;
    for (ObjectId neighbor_id : route_neighbors) {
      JunctionInputIncident incident{};
      incident.neighbor_node_id = neighbor_id;
      incident.direction = normalize_forward_xy_pipeline(current_support_position(neighbor_id) - center);
      if (!std::isfinite(incident.direction.x) || !std::isfinite(incident.direction.y)) {
        continue;
      }
      facts.route_incidents.push_back(incident);
    }
    output.by_node[node_id] = facts;
    output.ordered.push_back(std::move(facts));
  }

  result.value = std::move(output);
  result.ok = true;
  return result;
}

EditResult<JunctionPairResolverOutput> JunctionPairResolver::resolve(
    const BackboneBuilderOutput& builder_output, const JunctionInputBuilderOutput& input_output) const {
  EditResult<JunctionPairResolverOutput> result{};
  JunctionPairResolverOutput output{};
  output.topology = input_output.topology;

  for (const JunctionInputFacts& facts : input_output.ordered) {
    RouteThroughPairDecision through_pair{};
    auto adopt_through_pair = [&](ObjectId neighbor_a_id, ObjectId neighbor_b_id) {
      if (neighbor_a_id == kInvalidObjectId || neighbor_b_id == kInvalidObjectId || neighbor_a_id == neighbor_b_id ||
          find_junction_incident(facts, neighbor_a_id) == nullptr ||
          find_junction_incident(facts, neighbor_b_id) == nullptr) {
        return;
      }
      through_pair.neighbor_a_id = neighbor_a_id;
      through_pair.neighbor_b_id = neighbor_b_id;
      through_pair.straightness_score = junction_pair_score(facts, neighbor_a_id, neighbor_b_id);
      through_pair.accepted = true;
    };

    std::vector<const JunctionInputIncident*> pair_candidates = all_junction_incidents(facts);
    if (facts.route_incidents.size() == 2) {
      adopt_through_pair(facts.route_incidents[0].neighbor_node_id, facts.route_incidents[1].neighbor_node_id);
    } else {
      for (std::size_t i = 0; i + 1 < pair_candidates.size(); ++i) {
        for (std::size_t j = i + 1; j < pair_candidates.size(); ++j) {
          const ObjectId a = pair_candidates[i]->neighbor_node_id;
          const ObjectId b = pair_candidates[j]->neighbor_node_id;
          const double score = junction_pair_score(facts, a, b);
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

    JunctionRelation relation = build_junction_relation_from_selection(facts, through_pair);
    const ObjectId node_id = relation.node_id;
    const BackbonePair through_backbone_pair = pair_from_through_pair(relation.through_pair);
    output.pairs.relation_by_node[node_id] = relation;
    output.pairs.ordered_relations.push_back(relation);
    output.pairs.through_pair_by_node[node_id] = through_backbone_pair;
    output.pairs.cross_pair_by_node[node_id] = cross_pair_from_relation(relation);
    output.pairs.junction_axis_by_node[node_id] = junction_axis_from_pair(facts, through_backbone_pair);
  }

  for (std::size_t i = 0; i + 1 < builder_output.support_chain.ordered_support_node_ids.size(); ++i) {
    const ObjectId node_a = builder_output.support_chain.ordered_support_node_ids[i];
    const ObjectId node_b = builder_output.support_chain.ordered_support_node_ids[i + 1];
    const JunctionRelation& relation_a = output.pairs.relation_by_node[node_a];
    const JunctionRelation& relation_b = output.pairs.relation_by_node[node_b];
    const EdgeFlowInfo flow_a = classify_edge_flow_from_relation(relation_a, node_b);
    const EdgeFlowInfo flow_b = classify_edge_flow_from_relation(relation_b, node_a);
    if (flow_a.kind == BackboneFlowKind::kBranch && flow_b.kind != BackboneFlowKind::kBranch) {
      output.pairs.edge_flow_by_segment.push_back(flow_a);
    } else if (flow_b.kind == BackboneFlowKind::kBranch && flow_a.kind != BackboneFlowKind::kBranch) {
      output.pairs.edge_flow_by_segment.push_back(flow_b);
    } else if (flow_a.rule != BackboneFlowDecisionRule::kDefaultMain &&
               flow_b.rule == BackboneFlowDecisionRule::kDefaultMain) {
      output.pairs.edge_flow_by_segment.push_back(flow_a);
    } else if (flow_b.rule != BackboneFlowDecisionRule::kDefaultMain &&
               flow_a.rule == BackboneFlowDecisionRule::kDefaultMain) {
      output.pairs.edge_flow_by_segment.push_back(flow_b);
    } else {
      EdgeFlowInfo info{};
      info.kind = BackboneFlowKind::kMain;
      info.rule = (flow_a.rule == BackboneFlowDecisionRule::kJunctionOrderMain ||
                   flow_b.rule == BackboneFlowDecisionRule::kJunctionOrderMain)
                      ? BackboneFlowDecisionRule::kJunctionOrderMain
                      : BackboneFlowDecisionRule::kDefaultMain;
      output.pairs.edge_flow_by_segment.push_back(info);
    }
  }
  for (const JunctionRelation& relation : output.pairs.ordered_relations) {
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

EditResult<JunctionLevelResolverOutput> JunctionLevelResolver::resolve(
    const JunctionInputBuilderOutput& input_output, const JunctionPairResolverOutput& pair_output,
    const BackboneGenerationRequestPlan& request) const {
  (void)pair_output;
  (void)request;
  EditResult<JunctionLevelResolverOutput> result{};
  JunctionLevelResolverOutput output{};
  output.level_rules.by_node.reserve(input_output.ordered.size());
  for (const JunctionInputFacts& facts : input_output.ordered) {
    JunctionLevelRule rule{};
    rule.node_id = facts.node_id;
    rule.kind = JunctionLevelRuleKind::kSameLevelAllowed;
    rule.same_level_allowed = true;
    rule.must_lower = false;
    output.level_rules.by_node.emplace(facts.node_id, rule);
  }
  result.value = std::move(output);
  result.ok = true;
  return result;
}

EditResult<PoleFacing> PoleFacingResolver::resolve(const BackboneBuilderOutput& builder_output,
                                                   const JunctionPairResolverOutput& pair_output) const {
  EditResult<PoleFacing> result{};
  const auto& ordered_support_node_ids = builder_output.support_chain.ordered_support_node_ids;
  const auto& support_node_by_id = builder_output.support_chain.support_node_by_id;
  const auto& existing_node_position_by_id = pair_output.topology.existing_node_position_by_id;
  const bool reverse_build = builder_output.backbone.build_direction == BuildDirection::kReverse;

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
  PoleFacing facing{};
  auto direction_to = [&](ObjectId node_id, ObjectId neighbor_id) {
    return normalize_forward_xy_pipeline(current_support_position(neighbor_id) - current_support_position(node_id));
  };
  auto pair_forward_neighbor = [&](const BackbonePair& pair) {
    return reverse_build ? pair.low : pair.high;
  };
  auto pair_bisector = [&](ObjectId node_id, const BackbonePair& pair) {
    if (!pair.valid()) {
      return Vec3d{};
    }
    Vec3d dir_low = direction_to(node_id, pair.low);
    Vec3d dir_high = direction_to(node_id, pair.high);
    Vec3d axis = normalize_forward_xy_pipeline(dir_low + dir_high);
    if (!Normalize(&axis)) {
      axis = direction_to(node_id, pair_forward_neighbor(pair));
    }
    return axis;
  };
  auto route_neighbor_for = [&](std::size_t index) {
    if (ordered_support_node_ids.size() < 2) {
      return kInvalidObjectId;
    }
    if (!reverse_build) {
      return (index + 1 < ordered_support_node_ids.size()) ? ordered_support_node_ids[index + 1]
                                                           : ordered_support_node_ids[index - 1];
    }
    return (index > 0) ? ordered_support_node_ids[index - 1] : ordered_support_node_ids[index + 1];
  };
  auto route_neighbor_in_pair = [&](std::size_t index, const BackbonePair& pair) {
    const ObjectId route_neighbor_id = route_neighbor_for(index);
    if (route_neighbor_id == pair.low || route_neighbor_id == pair.high) {
      return route_neighbor_id;
    }
    return pair_forward_neighbor(pair);
  };
  std::unordered_set<ObjectId> planned_nodes{};
  for (std::size_t index = 0; index < ordered_support_node_ids.size(); ++index) {
    const ObjectId node_id = ordered_support_node_ids[index];
    if (!planned_nodes.insert(node_id).second) {
      continue;
    }
    BackbonePlannedPoleOrientation planned{};
    PoleOrientationDebugRecord debug{};
    debug.pole_id = node_id;
    if (const Pole* pole = state_.view().poles().find(node_id); pole != nullptr) {
      if (state_.has_pole_orientation_override(pole->id)) {
        planned.adopted_forward = RotateAroundWorldUpDeg(WorldForward(), state_.effective_pole_yaw_deg(*pole));
        planned.has_adopted_forward = Normalize(&planned.adopted_forward);
        planned.forward_rule = PoleForwardRule::kFallback;
        debug.rule = planned.forward_rule;
        debug.adopted_forward = planned.adopted_forward;
        planned.debug = debug;
        facing.by_node.emplace(node_id, std::move(planned));
        continue;
      }
    }

    Vec3d forward{};
    PoleForwardRule rule = PoleForwardRule::kFallback;
    ObjectId primary_neighbor_id = kInvalidObjectId;
    ObjectId secondary_neighbor_id = kInvalidObjectId;
    const auto cross_pair_it = pair_output.pairs.cross_pair_by_node.find(node_id);
    const bool is_cross = cross_pair_it != pair_output.pairs.cross_pair_by_node.end() && cross_pair_it->second.valid();
    if (is_cross) {
      if (const auto pair_it = pair_output.pairs.through_pair_by_node.find(node_id);
          pair_it != pair_output.pairs.through_pair_by_node.end()) {
        primary_neighbor_id = route_neighbor_in_pair(index, pair_it->second);
        forward = direction_to(node_id, primary_neighbor_id);
        rule = PoleForwardRule::kMainChainSingle;
        secondary_neighbor_id = (primary_neighbor_id == pair_it->second.low) ? pair_it->second.high
                                                                             : pair_it->second.low;
      }
    }
    if (!Normalize(&forward)) {
      if (const auto pair_it = pair_output.pairs.through_pair_by_node.find(node_id);
          pair_it != pair_output.pairs.through_pair_by_node.end()) {
        forward = pair_bisector(node_id, pair_it->second);
        rule = PoleForwardRule::kMainChainBisector;
        primary_neighbor_id = pair_it->second.low;
        secondary_neighbor_id = pair_it->second.high;
      }
    }
    if (!Normalize(&forward)) {
      const ObjectId neighbor_id = route_neighbor_for(index);
      if (neighbor_id != kInvalidObjectId) {
        forward = direction_to(node_id, neighbor_id);
        rule = PoleForwardRule::kFallback;
        primary_neighbor_id = neighbor_id;
      }
    }
    if (!Normalize(&forward)) {
      facing.by_node.emplace(node_id, std::move(planned));
      continue;
    }
    planned.adopted_forward = forward;
    planned.has_adopted_forward = true;
    planned.forward_rule = rule;
    planned.forward_primary_neighbor_id = primary_neighbor_id;
    planned.forward_secondary_neighbor_id = secondary_neighbor_id;
    debug.rule = rule;
    debug.primary_neighbor_id = primary_neighbor_id;
    debug.secondary_neighbor_id = secondary_neighbor_id;
    debug.adopted_forward = forward;
    planned.debug = debug;
    facing.by_node.emplace(node_id, std::move(planned));
  }
  result.value = std::move(facing);
  result.ok = true;
  return result;
}

EditResult<BundleSpanBuilderOutput> BundleSpanBuilder::build(const BackboneBuilderOutput& builder_output,
                                                             const JunctionPairResolverOutput& pair_output,
                                                             const JunctionLevelResolverOutput& level_output,
                                                             const PoleFacing& pole_facing) {
  EditResult<BundleSpanBuilderOutput> result{};
  EditResult<RealizedBackboneSupport> support_chain_out = state_.build_real_backbone_support(builder_output.support_chain);
  if (!support_chain_out.ok) {
    result.error = support_chain_out.error;
    return result;
  }

  EditResult<BackboneRuntimeState> runtime_out = state_.remap_backbone_build_to_real_nodes(
      pair_output.topology, pair_output.pairs, level_output.level_rules, pole_facing, builder_output.backbone.build_direction,
      support_chain_out.value.session_id,
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

namespace {

void append_support_group_rule(const CoreState& state, const EndpointLayoutRule& endpoint, SpanLayoutRule* rule) {
  if (rule == nullptr || !UsesAuthoritativeGroupedLoweredSupport(endpoint.semantic)) {
    return;
  }
  const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint.semantic);
  if (key.owner_pole_id == kInvalidObjectId || key.support_group_id < 0) {
    return;
  }
  if (state.view().poles().find(key.owner_pole_id) == nullptr) {
    return;
  }
  auto [it, inserted] = rule->support_group_rules.try_emplace(key);
  if (!inserted) {
    return;
  }
  SupportGroupDecision& group = it->second;
  static_cast<SupportLayoutSemanticDecision&>(group) = endpoint.semantic;
  group.owner_pole_id = endpoint.semantic.owner_pole_id;
  group.support_group_id = endpoint.semantic.support_group_id;
  group.support_authority = endpoint.support_authority;
  group.side = endpoint.side;
  group.origin = endpoint.origin;
  group.order_decision_policy = endpoint.order_decision_policy;
  group.order_decision_choice = endpoint.order_decision_choice;
  group.order_decision_choice_reason = endpoint.order_decision_choice_reason;
  group.chosen_side = endpoint.chosen_side;
  group.used_junction_pair_side_assignment = endpoint.used_junction_pair_side_assignment;
}

} // namespace

SpanLayoutRules SpanLayoutRuleBuilder::build(const GeneratedBackboneSpans& spans) const {
  SpanLayoutRules rules{};
  rules.spans.reserve(spans.generated_span_ids.size());
  const EditState& edit_state = state_.view().edit_state();
  std::size_t span_index = 0;
  for (const SegmentLaneAssignment& assignment : spans.lane_assignments) {
    const std::size_t lane_count = std::min(assignment.port_ids_a.size(), assignment.port_ids_b.size());
    for (std::size_t lane = 0; lane < lane_count && span_index < spans.generated_span_ids.size(); ++lane, ++span_index) {
      const ObjectId span_id = spans.generated_span_ids[span_index];
      const Span* span = edit_state.spans.find(span_id);
      const Port* port_a = edit_state.ports.find(assignment.port_ids_a[lane]);
      const Port* port_b = edit_state.ports.find(assignment.port_ids_b[lane]);
      if (span == nullptr || port_a == nullptr || port_b == nullptr) {
        continue;
      }
      SpanLayoutRule rule{};
      rule.span_id = span_id;
      rule.flow_kind = assignment.flow_kind;
      rule.pass_mode = (span->placement_context == ConnectionContext::kBranchAdd) ? CurvePassMode::kBranch
                                                                                  : CurvePassMode::kPassThrough;
      rule.variation_flow_key = assignment.variation_flow_key;
      rule.lowering_kind = assignment.lowering_kind;
      rule.start = build_endpoint_layout_rule_from_decision(edit_state, spans.junctions, assignment,
                                                            span->endpoint_node_a_id, *port_a, assignment.decision_a);
      rule.end = build_endpoint_layout_rule_from_decision(edit_state, spans.junctions, assignment,
                                                          span->endpoint_node_b_id, *port_b, assignment.decision_b);
      append_support_group_rule(state_, rule.start, &rule);
      append_support_group_rule(state_, rule.end, &rule);
      rules.spans.push_back(std::move(rule));
    }
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

  JunctionInputBuilder input_builder(state_);
  EditResult<JunctionInputBuilderOutput> input_out = input_builder.build(builder_output_);
  if (!input_out.ok) {
    result.error = input_out.error;
    return result;
  }
  junction_input_ = std::move(input_out.value);

  JunctionPairResolver pair_resolver;
  EditResult<JunctionPairResolverOutput> pair_out = pair_resolver.resolve(builder_output_, junction_input_);
  if (!pair_out.ok) {
    result.error = pair_out.error;
    return result;
  }
  junction_pairs_ = std::move(pair_out.value);

  JunctionLevelResolver level_resolver;
  EditResult<JunctionLevelResolverOutput> level_out =
      level_resolver.resolve(junction_input_, junction_pairs_, builder_output_.request);
  if (!level_out.ok) {
    result.error = level_out.error;
    return result;
  }
  junction_levels_ = std::move(level_out.value);

  PoleFacingResolver facing_resolver(state_);
  EditResult<PoleFacing> facing_out = facing_resolver.resolve(builder_output_, junction_pairs_);
  if (!facing_out.ok) {
    result.error = facing_out.error;
    return result;
  }
  pole_facing_ = std::move(facing_out.value);

  BundleSpanBuilder span_builder(state_);
  EditResult<BundleSpanBuilderOutput> spans_out =
      span_builder.build(builder_output_, junction_pairs_, junction_levels_, pole_facing_);
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
  SpanLayoutRules rules = rules_builder.build(spans_out.value.spans);
  state_.cache_span_layout_rules(rules);
  spans_out.value.spans.layout_rules = std::move(rules);
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
