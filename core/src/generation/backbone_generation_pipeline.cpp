#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"
#include "backbone_generation_plan_internal.hpp"
#include "wire/core/core_view.hpp"
#include "../pole_orientation_utils.hpp"
#include "../support_orientation_utils.hpp"
#include "backbone_seed_placement.hpp"
#include "backbone_pole_orientation_policy.hpp"
#include "backbone_seed_topology.hpp"
#include "backbone_prepare.hpp"
#include "detail_utils.hpp"
#include "grouped_span_common.hpp"
#include "grouped_span_lowering.hpp"
#include "grouped_span_orientation.hpp"
#include "support_policy.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
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

Vec3d choose_continuous_axis(const Vec3d& axis, const Vec3d& previous_forward) {
  Vec3d out = axis;
  if (!Normalize(&out)) {
    return {};
  }
  Vec3d prev = normalize_forward_xy(previous_forward);
  if (Dot(out, prev) < 0.0) {
    out = ScaleVec(out, -1.0);
  }
  return out;
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

std::uint64_t make_flow_variation_key(std::uint64_t generation_session_id, BundleKind bundle_template_id,
                                      BackboneFlowKind flow_kind, ObjectId run_start_node_id,
                                      ObjectId run_end_node_id) {
  std::uint64_t key = hash_combine(generation_session_id, static_cast<std::uint64_t>(bundle_template_id));
  key = hash_combine(key, static_cast<std::uint64_t>(flow_kind));
  key = hash_combine(key, static_cast<std::uint64_t>(run_start_node_id));
  key = hash_combine(key, static_cast<std::uint64_t>(run_end_node_id));
  return key;
}

bool assign_nonroute_pair_for_single_route_junction(JunctionRelation* relation,
                                                    const std::function<Vec3d(ObjectId)>& support_position) {
  if (relation == nullptr || relation->route_incident_count != 1 || relation->incidents.size() < 3) {
    return false;
  }
  std::vector<JunctionIncidentRelation*> nonroute_incidents{};
  for (JunctionIncidentRelation& incident : relation->incidents) {
    if (!incident.in_route) {
      nonroute_incidents.push_back(&incident);
    }
  }
  if (nonroute_incidents.size() < 2) {
    return false;
  }
  double best_nonroute_score = -2.0;
  JunctionIncidentRelation* best_a = nullptr;
  JunctionIncidentRelation* best_b = nullptr;
  for (std::size_t i = 0; i < nonroute_incidents.size(); ++i) {
    for (std::size_t j = i + 1; j < nonroute_incidents.size(); ++j) {
      Vec3d dir_a = normalize_forward_xy(support_position(nonroute_incidents[i]->neighbor_node_id) -
                                         support_position(relation->node_id));
      Vec3d dir_b = normalize_forward_xy(support_position(nonroute_incidents[j]->neighbor_node_id) -
                                         support_position(relation->node_id));
      if (!std::isfinite(dir_a.x) || !std::isfinite(dir_a.y) || !std::isfinite(dir_b.x) || !std::isfinite(dir_b.y)) {
        continue;
      }
      const double straight_score = dot_xy(dir_a, Vec3d{-dir_b.x, -dir_b.y, 0.0});
      if (straight_score > best_nonroute_score + 1e-9) {
        best_nonroute_score = straight_score;
        best_a = nonroute_incidents[i];
        best_b = nonroute_incidents[j];
      }
    }
  }
  if (best_a == nullptr || best_b == nullptr) {
    return false;
  }
  relation->through_pair.neighbor_a_id = best_a->neighbor_node_id;
  relation->through_pair.neighbor_b_id = best_b->neighbor_node_id;
  relation->through_pair.straightness_score = best_nonroute_score;
  relation->through_pair.accepted = true;
  relation->through_pair.used_semantic_tiebreak = true;
  for (JunctionIncidentRelation& incident : relation->incidents) {
    incident.in_through_pair =
        incident.neighbor_node_id == best_a->neighbor_node_id || incident.neighbor_node_id == best_b->neighbor_node_id;
    incident.used_semantic_tiebreak = incident.in_through_pair;
    incident.straightness_score = best_nonroute_score;
    incident.kind = incident.in_through_pair ? JunctionRelationKind::kThroughMain : JunctionRelationKind::kSideBranch;
  }
  return true;
}

SupportLayoutDecisionSeedEndpoint build_seed_endpoint_record(
    const EditState& edit_state, const std::unordered_map<ObjectId, JunctionRelation>& junction_relations_by_node,
    const SegmentLaneAssignment& assignment, ObjectId endpoint_node_id, const Port& port,
    const EndpointContinuityDecision& decision) {
  return build_seed_endpoint_from_decision(edit_state, junction_relations_by_node, assignment, endpoint_node_id, port,
                                           decision);
}

std::optional<SpanSupportLayoutDecisionSeed> rebuild_existing_span_decision_seed(
    const EditState& edit_state, const Span& span, const SpanSupportLayoutDecisionSeed* cached_seed) {
  if (cached_seed == nullptr) {
    return std::nullopt;
  }
  if (cached_seed != nullptr && !cached_seed->support_group_decisions.empty()) {
    return std::nullopt;
  }

  const Port* port_a = edit_state.ports.find(span.port_a_id);
  const Port* port_b = edit_state.ports.find(span.port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    return std::nullopt;
  }
  const bool matches_current_span =
      cached_seed->span_id == span.id &&
      cached_seed->start.port_id == port_a->id &&
      cached_seed->end.port_id == port_b->id &&
      cached_seed->start.owner_pole_id == port_a->owner_pole_id &&
      cached_seed->end.owner_pole_id == port_b->owner_pole_id &&
      cached_seed->start.endpoint_node_id == span.endpoint_node_a_id &&
      cached_seed->end.endpoint_node_id == span.endpoint_node_b_id;
  if (!matches_current_span) {
    return std::nullopt;
  }
  return *cached_seed;
}

double template_layer_base_z_for_port_category_seed(const CoreState& state, const Pole& pole, ConnectionCategory category) {
  return state.view().port_category_base_z_for_pole(pole, category);
}

void append_seed_support_group_decision(const CoreState& state, const Port& port,
                                        const SupportLayoutDecisionSeedEndpoint& endpoint,
                                        SpanSupportLayoutDecisionSeed* layout) {
  if (layout == nullptr || !UsesAuthoritativeGroupedLoweredSupport(endpoint)) {
    return;
  }
  const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint);
  if (key.owner_pole_id == kInvalidObjectId || key.support_group_id < 0) {
    return;
  }
  const Pole* pole = state.view().poles().find(key.owner_pole_id);
  if (pole == nullptr) {
    return;
  }
  auto [it, inserted] = layout->support_group_decisions.try_emplace(key);
  if (inserted) {
    SupportGroupDecision& group = it->second;
    static_cast<SupportLayoutSemanticDecision&>(group) = endpoint;
    group.owner_pole_id = endpoint.owner_pole_id;
    group.support_group_id = endpoint.support_group_id;
    group.support_authority = endpoint.support_authority;
    group.side = endpoint.side;
    group.origin = endpoint.origin;
    group.order_decision_policy = endpoint.order_decision_policy;
    group.order_decision_choice = endpoint.order_decision_choice;
    group.order_decision_choice_reason = endpoint.order_decision_choice_reason;
    group.chosen_side = endpoint.chosen_side;
    group.used_junction_pair_side_assignment = endpoint.used_junction_pair_side_assignment;
  }
}

SpanSupportLayoutDecisionSeed build_seed_layout_record(
    const CoreState& state, ObjectId span_id, BackboneFlowKind flow_kind, CurvePassMode pass_mode,
    std::uint64_t variation_flow_key, BackboneLoweringKind lowering_kind, const Port& port_a, const Port& port_b,
    SupportLayoutDecisionSeedEndpoint start, SupportLayoutDecisionSeedEndpoint end) {
  SpanSupportLayoutDecisionSeed layout{};
  layout.span_id = span_id;
  layout.flow_kind = flow_kind;
  layout.pass_mode = pass_mode;
  layout.variation_flow_key = variation_flow_key;
  layout.lowering_kind = lowering_kind;
  layout.start = std::move(start);
  layout.end = std::move(end);
  append_seed_support_group_decision(state, port_a, layout.start, &layout);
  append_seed_support_group_decision(state, port_b, layout.end, &layout);
  return layout;
}

struct BackboneDecisionPhaseContext {
  const std::vector<ObjectId>& ordered_support_node_ids;
  const std::unordered_map<ObjectId, const JunctionInfo*>& existing_junction_by_node;
  const std::unordered_map<ObjectId, std::uint64_t>& existing_prioritized_session_by_node;
  const std::unordered_map<ObjectId, std::unordered_map<ObjectId, std::uint64_t>>& existing_incident_session_by_node;
  BackboneResult& generation_backbone;
  std::unordered_map<ObjectId, const JunctionInfo*>& active_junction_by_node;
  std::uint64_t session_id = 0;
  std::function<EdgeFlowInfo(ObjectId, ObjectId)> classify_edge_flow;
  std::function<JunctionRelation(ObjectId)> classify_junction;
};

BackboneDecisionPhaseOutput run_backbone_decision_phase(const BackboneDecisionPhaseContext& context) {
  BackboneDecisionPhaseOutput phase{};
  phase.edge_flow_by_segment.reserve((context.ordered_support_node_ids.size() > 1)
                                         ? (context.ordered_support_node_ids.size() - 1)
                                         : 0);
  for (std::size_t i = 0; i + 1 < context.ordered_support_node_ids.size(); ++i) {
    phase.edge_flow_by_segment.push_back(
        context.classify_edge_flow(context.ordered_support_node_ids[i], context.ordered_support_node_ids[i + 1]));
  }
  phase.junction_relations_in_path_order.reserve(context.ordered_support_node_ids.size());
  phase.junction_relations_by_node.reserve(context.ordered_support_node_ids.size());
  for (ObjectId node_id : context.ordered_support_node_ids) {
    JunctionRelation relation = context.classify_junction(node_id);
    phase.junction_relations_by_node[node_id] = relation;
    phase.junction_relations_in_path_order.push_back(std::move(relation));
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
    junction.used_neighbor_continuity = relation.through_pair.accepted &&
                                        std::any_of(relation.incidents.begin(), relation.incidents.end(),
                                                    [](const JunctionIncidentRelation& incident) {
                                                      return incident.in_through_pair && !incident.in_route;
                                                    });
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

  context.active_junction_by_node.clear();
  context.active_junction_by_node.reserve(context.existing_junction_by_node.size() +
                                          context.generation_backbone.junctions.size());
  for (const auto& [node_id, junction] : context.existing_junction_by_node) {
    context.active_junction_by_node[node_id] = junction;
  }
  for (const JunctionInfo& junction : context.generation_backbone.junctions) {
    context.active_junction_by_node[junction.node_id] = &junction;
  }

  return phase;
}

JunctionRelation remap_junction_relation(const JunctionRelation& relation,
                                         const std::function<ObjectId(ObjectId)>& remap_node_id) {
  JunctionRelation remapped = relation;
  remapped.node_id = remap_node_id(remapped.node_id);
  remapped.through_pair.neighbor_a_id = remap_node_id(remapped.through_pair.neighbor_a_id);
  remapped.through_pair.neighbor_b_id = remap_node_id(remapped.through_pair.neighbor_b_id);
  for (JunctionIncidentRelation& incident : remapped.incidents) {
    incident.neighbor_node_id = remap_node_id(incident.neighbor_node_id);
  }
  return remapped;
}

BackboneDecisionPhaseOutput remap_backbone_decision_phase(
    const BackboneDecisionPhaseOutput& phase, const std::function<ObjectId(ObjectId)>& remap_node_id) {
  BackboneDecisionPhaseOutput remapped{};
  remapped.edge_flow_by_segment = phase.edge_flow_by_segment;
  remapped.junction_relations_in_path_order.reserve(phase.junction_relations_in_path_order.size());
  for (const JunctionRelation& relation : phase.junction_relations_in_path_order) {
    remapped.junction_relations_in_path_order.push_back(remap_junction_relation(relation, remap_node_id));
  }
  for (const auto& [node_id, relation] : phase.junction_relations_by_node) {
    remapped.junction_relations_by_node.emplace(remap_node_id(node_id), remap_junction_relation(relation, remap_node_id));
  }
  return remapped;
}

BackboneDecisionPhaseOutput build_backbone_decision_phase(
    const CoreState& state, const BackboneGenerationRequestPlan& request_plan,
    const BackboneSupportChainPlan& support_chain_plan, BackboneTopologyPlan* plan) {
  if (plan == nullptr) {
    return {};
  }

  const BackboneSpec& request = request_plan.request;
  const std::vector<BackboneBundlePlan>& active_bundle_plans = request_plan.active_bundle_plans;
  const EditState& edit_state = state.view().edit_state();
  const auto& route_neighbors_by_node = plan->route_neighbors_by_node;
  const auto& existing_node_position_by_id = plan->existing_node_position_by_id;
  const auto& existing_adjacency = plan->existing_adjacency;
  const auto& existing_junction_by_node = plan->existing_junction_by_node;
  const auto& existing_primary_neighbor_by_node = plan->existing_primary_neighbor_by_node;
  std::unordered_map<ObjectId, const JunctionInfo*> active_junction_by_node = plan->active_junction_by_node;

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

  struct BundleCategoryTieBreakKey {
    int category_rank = std::numeric_limits<int>::max();
    int bundle_rank = std::numeric_limits<int>::max();
    ConnectionCategory category = ConnectionCategory::kLowVoltage;
    BundleKind bundle_template_id = BundleKind::kLowVoltage;
  };
  auto bundle_category_key_tuple = [](const BundleCategoryTieBreakKey& key) {
    return std::tuple<int, int>{key.category_rank, key.bundle_rank};
  };
  auto better_bundle_category_key = [&](const BundleCategoryTieBreakKey& lhs, const BundleCategoryTieBreakKey& rhs) {
    return bundle_category_key_tuple(lhs) < bundle_category_key_tuple(rhs);
  };
  auto find_bundle_template = [&](BundleKind bundle_template_id) -> const BundleTemplate* {
    const auto& templates = state.view().bundle_templates();
    const auto it = templates.find(bundle_template_id);
    return (it == templates.end()) ? nullptr : &it->second;
  };
  auto edge_key_for_neighbors = [&](ObjectId node_a, ObjectId node_b) {
    const std::uint64_t lo = static_cast<std::uint64_t>(std::min(node_a, node_b));
    const std::uint64_t hi = static_cast<std::uint64_t>(std::max(node_a, node_b));
    return hash_combine(lo, hi);
  };
  BundleCategoryTieBreakKey route_bundle_category_key{};
  for (const BackboneBundlePlan& bundle_plan : active_bundle_plans) {
    const BundleCategoryTieBreakKey candidate_key{
        static_cast<int>(bundle_plan.category), static_cast<int>(bundle_plan.template_id), bundle_plan.category,
        bundle_plan.template_id};
    if (better_bundle_category_key(candidate_key, route_bundle_category_key)) {
      route_bundle_category_key = candidate_key;
    }
  }
  std::unordered_map<std::uint64_t, BundleCategoryTieBreakKey> bundle_category_key_by_edge{};
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
    const BundleTemplate* bundle_template = find_bundle_template(bundle->bundle_template_id);
    if (bundle_template == nullptr) {
      continue;
    }
    const BundleCategoryTieBreakKey candidate_key{
        static_cast<int>(bundle_template->category), static_cast<int>(bundle->bundle_template_id), bundle_template->category,
        bundle->bundle_template_id};
    BundleCategoryTieBreakKey& current_key =
        bundle_category_key_by_edge[edge_key_for_neighbors(port_a->owner_pole_id, port_b->owner_pole_id)];
    if (better_bundle_category_key(candidate_key, current_key)) {
      current_key = candidate_key;
    }
  }
  auto bundle_category_key_for_neighbor = [&](ObjectId node_id, ObjectId neighbor_id, bool is_route_neighbor) {
    BundleCategoryTieBreakKey key{};
    const auto it = bundle_category_key_by_edge.find(edge_key_for_neighbors(node_id, neighbor_id));
    if (it != bundle_category_key_by_edge.end()) {
      key = it->second;
    }
    if (is_route_neighbor && better_bundle_category_key(route_bundle_category_key, key)) {
      key = route_bundle_category_key;
    }
    return key;
  };
  auto existing_continuation_neighbors_for_order = [&](ObjectId node_id) {
    std::vector<ObjectId> neighbors{};
    if (const auto it = existing_junction_by_node.find(node_id); it != existing_junction_by_node.end()) {
      const JunctionInfo* junction = it->second;
      if (junction != nullptr && junction->incidents.size() >= 2) {
        const JunctionIncident* primary_incident = nullptr;
        const JunctionIncident* continuation_incident = nullptr;
        for (const JunctionIncident& incident : junction->incidents) {
          if ((incident.primary || incident.order == 0) &&
              (primary_incident == nullptr ||
               (incident.primary && !primary_incident->primary) ||
               (incident.order >= 0 && (primary_incident->order < 0 || incident.order < primary_incident->order)))) {
            primary_incident = &incident;
          }
          if (incident.order == 1 &&
              (continuation_incident == nullptr || incident.neighbor_node_id < continuation_incident->neighbor_node_id)) {
            continuation_incident = &incident;
          }
        }
        const bool has_continuation_pair =
            primary_incident != nullptr && continuation_incident != nullptr &&
            primary_incident->neighbor_node_id != continuation_incident->neighbor_node_id &&
            (junction->used_neighbor_continuity || junction->incidents.size() == 2);
        if (has_continuation_pair) {
          neighbors.push_back(primary_incident->neighbor_node_id);
          neighbors.push_back(continuation_incident->neighbor_node_id);
          return neighbors;
        }
      }
    }
    if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 2) {
      return it->second;
    }
    return neighbors;
  };
  auto existing_primary_neighbor_for_order = [&](ObjectId node_id) -> ObjectId {
    if (const auto it = existing_junction_by_node.find(node_id); it != existing_junction_by_node.end()) {
      const JunctionInfo* junction = it->second;
      if (junction != nullptr) {
        for (const JunctionIncident& incident : junction->incidents) {
          if (incident.primary || incident.order == 0) {
            return incident.neighbor_node_id;
          }
        }
      }
    }
    const auto it_route = route_neighbors_by_node.find(node_id);
    const std::size_t route_degree = (it_route == route_neighbors_by_node.end()) ? 0 : it_route->second.size();
    if (route_degree <= 1) {
      if (const auto it = existing_adjacency.find(node_id); it != existing_adjacency.end() && it->second.size() == 1) {
        return it->second.front();
      }
    }
    const auto it_existing_primary = existing_primary_neighbor_by_node.find(node_id);
    return (it_existing_primary == existing_primary_neighbor_by_node.end()) ? kInvalidObjectId
                                                                            : it_existing_primary->second;
  };

  constexpr double kThroughPairStraightnessThreshold = 0.3;
  auto classify_junction = [&](ObjectId node_id) {
    JunctionRelation relation{};
    relation.node_id = node_id;

    const JunctionInfo* junction = nullptr;
    if (const auto it = active_junction_by_node.find(node_id); it != active_junction_by_node.end()) {
      junction = it->second;
    }

    const std::vector<ObjectId> route_neighbors =
        (route_neighbors_by_node.contains(node_id) ? route_neighbors_by_node.at(node_id) : std::vector<ObjectId>{});
    const std::vector<ObjectId> combined_neighbors = combined_neighbors_for_node(node_id);
    relation.is_cross_like = combined_neighbors.size() >= 4;

    auto explicit_middle_route_corner_score = [&]() -> std::optional<double> {
      if (request.path.node_specs.size() != 1) {
        return std::nullopt;
      }
      if (route_neighbors.size() != 2) {
        return std::nullopt;
      }

      const BackboneInputSpec::NodeSpec* explicit_node_spec = nullptr;
      const Vec3d node_position = current_support_position(node_id);
      for (const BackboneInputSpec::NodeSpec& node_spec : request.path.node_specs) {
        if (node_spec.support_kind != SupportKind::kPole) {
          continue;
        }
        if (node_spec.point_index <= 0 ||
            node_spec.point_index >= static_cast<int>(request.path.polyline.size()) - 1) {
          continue;
        }
        const Vec3d point = request.path.polyline[static_cast<std::size_t>(node_spec.point_index)];
        const Vec3d delta = point - node_position;
        const bool same_point = std::abs(delta.x) <= 1e-6 && std::abs(delta.y) <= 1e-6;
        if (!same_point) {
          continue;
        }
        if (explicit_node_spec != nullptr) {
          return std::nullopt;
        }
        explicit_node_spec = &node_spec;
      }
      if (explicit_node_spec == nullptr) {
        return std::nullopt;
      }

      const int path_point_index = explicit_node_spec->point_index;
      if (path_point_index <= 0 ||
          path_point_index >= static_cast<int>(request.path.polyline.size()) - 1) {
        return std::nullopt;
      }

      const Vec3d center = request.path.polyline[static_cast<std::size_t>(path_point_index)];
      const Vec3d first_dir =
          normalize_forward_xy(request.path.polyline[static_cast<std::size_t>(path_point_index - 1)] - center);
      const Vec3d second_dir =
          normalize_forward_xy(request.path.polyline[static_cast<std::size_t>(path_point_index + 1)] - center);
      if (!std::isfinite(first_dir.x) || !std::isfinite(first_dir.y) || !std::isfinite(second_dir.x) ||
          !std::isfinite(second_dir.y)) {
        return std::nullopt;
      }
      return Dot(first_dir, Vec3d{-second_dir.x, -second_dir.y, -second_dir.z});
    };

    if (combined_neighbors.empty()) {
      return relation;
    }

    struct Candidate {
      ObjectId neighbor_id = kInvalidObjectId;
      Vec3d dir{};
      bool is_route_neighbor = false;
      bool preserves_existing_continuity = false;
      bool continuity_primary = false;
      int order = std::numeric_limits<int>::max();
      std::uint64_t source_session_id = std::numeric_limits<std::uint64_t>::max();
      BundleCategoryTieBreakKey bundle_category_key{};
    };

    std::vector<Candidate> candidates{};
    candidates.reserve(combined_neighbors.size());
    const Vec3d center = current_support_position(node_id);
    const std::vector<ObjectId> existing_continuity_neighbors = existing_continuation_neighbors_for_order(node_id);
    const bool has_existing_continuity_pair =
        existing_continuity_neighbors.size() >= 2 &&
        existing_continuity_neighbors[0] != existing_continuity_neighbors[1];
    auto pair_straightness_score = [&](ObjectId neighbor_a_id, ObjectId neighbor_b_id) {
      const Vec3d axis_a = normalize_forward_xy(current_support_position(neighbor_a_id) - center);
      const Vec3d axis_b = normalize_forward_xy(current_support_position(neighbor_b_id) - center);
      if (!std::isfinite(axis_a.x) || !std::isfinite(axis_a.y) || !std::isfinite(axis_b.x) ||
          !std::isfinite(axis_b.y)) {
        return -2.0;
      }
      return Dot(axis_a, Vec3d{-axis_b.x, -axis_b.y, -axis_b.z});
    };
    const double existing_continuity_score =
        has_existing_continuity_pair
            ? pair_straightness_score(existing_continuity_neighbors[0], existing_continuity_neighbors[1])
            : -2.0;
    const bool preserve_existing_continuity =
        has_existing_continuity_pair &&
        existing_continuity_score + 1e-9 >= kThroughPairStraightnessThreshold;
    const ObjectId continuity_primary_neighbor = existing_primary_neighbor_for_order(node_id);
    for (ObjectId neighbor_id : combined_neighbors) {
      const Vec3d dir = normalize_forward_xy(current_support_position(neighbor_id) - center);
      if (!std::isfinite(dir.x) || !std::isfinite(dir.y)) {
        continue;
      }
      Candidate candidate{};
      candidate.neighbor_id = neighbor_id;
      candidate.dir = dir;
      candidate.is_route_neighbor =
          std::find(route_neighbors.begin(), route_neighbors.end(), neighbor_id) != route_neighbors.end();
      candidate.continuity_primary = (neighbor_id == continuity_primary_neighbor);
      candidate.preserves_existing_continuity =
          preserve_existing_continuity &&
          std::find(existing_continuity_neighbors.begin(), existing_continuity_neighbors.end(), neighbor_id) !=
              existing_continuity_neighbors.end();
      candidate.bundle_category_key =
          bundle_category_key_for_neighbor(node_id, neighbor_id, candidate.is_route_neighbor);
      if (junction != nullptr) {
        for (const JunctionIncident& incident : junction->incidents) {
          if (incident.neighbor_node_id == neighbor_id) {
            candidate.order = (incident.order >= 0) ? incident.order : candidate.order;
            candidate.source_session_id = incident.source_session_id;
            candidate.continuity_primary =
                candidate.continuity_primary || incident.primary || incident.order == 0;
            break;
          }
        }
      }
      candidates.push_back(candidate);
    }
    relation.route_incident_count = static_cast<int>(route_neighbors.size());
    relation.incidents.reserve(candidates.size());
    for (const Candidate& candidate : candidates) {
      JunctionIncidentRelation incident{};
      incident.neighbor_node_id = candidate.neighbor_id;
      incident.kind = JunctionRelationKind::kNone;
      incident.in_route = candidate.is_route_neighbor;
      relation.incidents.push_back(incident);
    }
    if (const std::optional<double> route_corner_score = explicit_middle_route_corner_score();
        route_corner_score.has_value() && *route_corner_score + 1e-9 < 0.95) {
      for (JunctionIncidentRelation& incident : relation.incidents) {
        if (incident.in_route) {
          incident.kind = JunctionRelationKind::kCornerContinuation;
        }
      }
      return relation;
    }
    if (route_neighbors.size() == 1 && relation.is_cross_like) {
      for (JunctionIncidentRelation& incident : relation.incidents) {
        if (incident.in_route) {
          incident.kind = JunctionRelationKind::kThroughMain;
        }
      }
      return relation;
    }
    if (candidates.size() == 1) {
      for (JunctionIncidentRelation& incident : relation.incidents) {
        if (incident.in_route) {
          incident.kind = JunctionRelationKind::kThroughMain;
        }
      }
      return relation;
    }
    if (candidates.size() < 2) {
      if (route_neighbors.size() == 1) {
        for (JunctionIncidentRelation& incident : relation.incidents) {
          if (incident.in_route) {
            incident.kind = JunctionRelationKind::kThroughMain;
          }
        }
      }
      return relation;
    }

    if (route_neighbors.size() == 1 && assign_nonroute_pair_for_single_route_junction(&relation, current_support_position)) {
      return relation;
    }

    double best_score = -2.0;
    int best_i = -1;
    int best_j = -1;
    bool best_used_priority_tiebreak = false;
    auto candidate_stable_key = [](const Candidate& candidate) {
      return std::tuple<int, std::uint64_t, ObjectId>{
          candidate.order, candidate.source_session_id, candidate.neighbor_id};
    };
    auto pair_matches_existing_continuity = [&](const Candidate& a, const Candidate& b) {
      if (!preserve_existing_continuity) {
        return false;
      }
      return a.preserves_existing_continuity && b.preserves_existing_continuity;
    };
    auto pair_bundle_category_key = [&](const Candidate& a, const Candidate& b) {
      const auto key_a = bundle_category_key_tuple(a.bundle_category_key);
      const auto key_b = bundle_category_key_tuple(b.bundle_category_key);
      return (key_b < key_a) ? std::tuple{key_b, key_a} : std::tuple{key_a, key_b};
    };
    auto pair_stable_key = [&](const Candidate& a, const Candidate& b) {
      const auto key_a = candidate_stable_key(a);
      const auto key_b = candidate_stable_key(b);
      return (key_b < key_a) ? std::tuple{key_b, key_a} : std::tuple{key_a, key_b};
    };
    for (std::size_t i = 0; i < candidates.size(); ++i) {
      for (std::size_t j = i + 1; j < candidates.size(); ++j) {
        const double straight_score =
            Dot(candidates[i].dir, Vec3d{-candidates[j].dir.x, -candidates[j].dir.y, -candidates[j].dir.z});
        const bool current_preserves_existing = pair_matches_existing_continuity(candidates[i], candidates[j]);
        const bool best_preserves_existing =
            (best_i >= 0 && best_j >= 0) &&
            pair_matches_existing_continuity(candidates[static_cast<std::size_t>(best_i)],
                                             candidates[static_cast<std::size_t>(best_j)]);
        bool choose_current = false;
        bool used_priority_tiebreak = false;
        if (best_i < 0 || best_j < 0) {
          choose_current = true;
        } else if (current_preserves_existing != best_preserves_existing) {
          choose_current = current_preserves_existing;
          used_priority_tiebreak = true;
        } else if (straight_score > best_score + 1e-9) {
          choose_current = true;
        } else if (std::abs(straight_score - best_score) <= 1e-9) {
          const auto current_bundle_key = pair_bundle_category_key(candidates[i], candidates[j]);
          const auto best_bundle_key =
              pair_bundle_category_key(candidates[static_cast<std::size_t>(best_i)],
                                       candidates[static_cast<std::size_t>(best_j)]);
          if (current_bundle_key != best_bundle_key) {
            choose_current = current_bundle_key < best_bundle_key;
            used_priority_tiebreak = true;
          } else {
            const auto current_stable_key = pair_stable_key(candidates[i], candidates[j]);
            const auto best_stable_key =
                pair_stable_key(candidates[static_cast<std::size_t>(best_i)],
                                candidates[static_cast<std::size_t>(best_j)]);
            if (current_stable_key < best_stable_key) {
              choose_current = true;
              used_priority_tiebreak = true;
            }
          }
        }
        if (!choose_current) {
          continue;
        }
        best_score = straight_score;
        best_i = static_cast<int>(i);
        best_j = static_cast<int>(j);
        best_used_priority_tiebreak = used_priority_tiebreak;
      }
    }

    relation.through_pair.straightness_score = best_score;
    for (JunctionIncidentRelation& incident : relation.incidents) {
      incident.straightness_score = best_score;
    }
    if (best_i < 0 || best_j < 0 || !(best_score + 1e-9 >= kThroughPairStraightnessThreshold)) {
      const bool has_existing_straightish_continuity = existing_continuation_neighbors_for_order(node_id).size() >= 2;
      for (JunctionIncidentRelation& incident : relation.incidents) {
        if (incident.in_route && route_neighbors.size() >= 2) {
          incident.kind = has_existing_straightish_continuity ? JunctionRelationKind::kSideBranch
                                                              : JunctionRelationKind::kCornerContinuation;
        } else if (incident.in_route && combined_neighbors.size() == 1) {
          incident.kind = JunctionRelationKind::kThroughMain;
        } else if (incident.in_route && route_neighbors.size() == 1) {
          incident.kind = JunctionRelationKind::kSideBranch;
        }
      }
      return relation;
    }

    relation.through_pair.neighbor_a_id = candidates[static_cast<std::size_t>(best_i)].neighbor_id;
    relation.through_pair.neighbor_b_id = candidates[static_cast<std::size_t>(best_j)].neighbor_id;
    relation.through_pair.accepted = true;
    relation.through_pair.used_semantic_tiebreak = best_used_priority_tiebreak;

    auto in_through_pair = [&](ObjectId neighbor_id) {
      return neighbor_id == relation.through_pair.neighbor_a_id || neighbor_id == relation.through_pair.neighbor_b_id;
    };
    for (JunctionIncidentRelation& incident : relation.incidents) {
      incident.in_through_pair = in_through_pair(incident.neighbor_node_id);
      incident.used_semantic_tiebreak =
          relation.through_pair.used_semantic_tiebreak && incident.in_through_pair;
      if (in_through_pair(incident.neighbor_node_id)) {
        incident.kind = JunctionRelationKind::kThroughMain;
      } else if (relation.is_cross_like) {
        incident.kind = JunctionRelationKind::kCrossUnderpass;
      } else if (incident.in_route) {
        incident.kind = JunctionRelationKind::kSideBranch;
      }
    }
    auto candidate_for_neighbor = [&](ObjectId neighbor_id) -> const Candidate* {
      for (const Candidate& candidate : candidates) {
        if (candidate.neighbor_id == neighbor_id) {
          return &candidate;
        }
      }
      return nullptr;
    };
    auto incident_order_key = [&](const JunctionIncidentRelation& incident) {
      const Candidate* candidate = candidate_for_neighbor(incident.neighbor_node_id);
      const bool preserves_existing = (candidate != nullptr) ? candidate->preserves_existing_continuity : false;
      const bool continuity_primary = (candidate != nullptr) ? candidate->continuity_primary : false;
      const auto bundle_key =
          (candidate != nullptr) ? bundle_category_key_tuple(candidate->bundle_category_key)
                                 : std::tuple<int, int>{std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};
      const auto stable_key = (candidate != nullptr)
                                  ? candidate_stable_key(*candidate)
                                  : std::tuple<int, std::uint64_t, ObjectId>{
                                        std::numeric_limits<int>::max(),
                                        std::numeric_limits<std::uint64_t>::max(),
                                        incident.neighbor_node_id};
      return std::tuple<int, int, int, decltype(bundle_key), decltype(stable_key)>{
          preserves_existing ? 0 : 1,
          incident.in_through_pair ? 0 : 1,
          continuity_primary ? 0 : 1,
          bundle_key,
          stable_key};
    };
    std::sort(relation.incidents.begin(), relation.incidents.end(),
              [&](const JunctionIncidentRelation& lhs, const JunctionIncidentRelation& rhs) {
                return incident_order_key(lhs) < incident_order_key(rhs);
              });
    if (relation.through_pair.accepted && relation.incidents.size() >= 2) {
      relation.through_pair.neighbor_a_id = relation.incidents[0].neighbor_node_id;
      relation.through_pair.neighbor_b_id = relation.incidents[1].neighbor_node_id;
    }
    return relation;
  };
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
    const JunctionRelation relation = classify_junction(node_id);
    const JunctionRelationKind relation_kind = find_incident_relation(relation, peer_id);
    const bool relation_uses_existing_continuity =
        relation.through_pair.accepted &&
        std::any_of(relation.incidents.begin(), relation.incidents.end(),
                    [](const JunctionIncidentRelation& incident) {
                      return incident.in_through_pair && !incident.in_route;
                    });
    switch (relation_kind) {
    case JunctionRelationKind::kCrossUnderpass:
      info.kind = BackboneFlowKind::kBranch;
      info.rule = BackboneFlowDecisionRule::kJunctionOrderBranch;
      info.underpass_at_cross = true;
      return info;
    case JunctionRelationKind::kSideBranch:
      info.kind = BackboneFlowKind::kBranch;
      info.rule = relation_uses_existing_continuity
                      ? BackboneFlowDecisionRule::kExistingChainBranch
                      : BackboneFlowDecisionRule::kJunctionOrderBranch;
      return info;
    case JunctionRelationKind::kCornerContinuation:
      info.kind = BackboneFlowKind::kMain;
      info.rule = BackboneFlowDecisionRule::kDefaultMain;
      return info;
    case JunctionRelationKind::kThroughMain:
      info.kind = BackboneFlowKind::kMain;
      info.rule = relation_uses_existing_continuity
                      ? BackboneFlowDecisionRule::kExistingChainMain
                      : (relation.through_pair.accepted ? BackboneFlowDecisionRule::kJunctionOrderMain
                                                        : BackboneFlowDecisionRule::kDefaultMain);
      return info;
    case JunctionRelationKind::kNone:
    default:
      return info;
    }
  };
  auto classify_edge_flow = [&](ObjectId node_a, ObjectId node_b) {
    const EdgeFlowInfo flow_a = classify_edge_flow_at_node(node_a, node_b);
    const EdgeFlowInfo flow_b = classify_edge_flow_at_node(node_b, node_a);
    if (flow_a.kind == BackboneFlowKind::kBranch) {
      return flow_a;
    }
    if (flow_b.kind == BackboneFlowKind::kBranch) {
      return flow_b;
    }
    if (flow_a.rule != BackboneFlowDecisionRule::kDefaultMain) {
      return flow_a;
    }
    if (flow_b.rule != BackboneFlowDecisionRule::kDefaultMain) {
      return flow_b;
    }
    return flow_a;
  };

  BackboneDecisionPhaseOutput phase = run_backbone_decision_phase({
      support_chain_plan.ordered_support_node_ids,
      existing_junction_by_node,
      plan->existing_prioritized_session_by_node,
      plan->existing_incident_session_by_node,
      plan->generation_backbone,
      active_junction_by_node,
      support_chain_plan.session_id,
      classify_edge_flow,
      classify_junction,
  });
  return phase;
}

std::vector<SpanSupportLayoutDecisionSeed>
build_seed_generated_support_layouts(const CoreState& state, const EditState& edit_state, const std::vector<ObjectId>& span_ids,
                                     const std::vector<SegmentLaneAssignment>& lane_assignments,
                                     const std::unordered_map<ObjectId, JunctionRelation>& junction_relations_by_node,
                                     std::uint64_t variation_flow_key) {
  std::vector<SpanSupportLayoutDecisionSeed> layouts{};
  layouts.reserve(span_ids.size());
  std::size_t span_index = 0;
  for (const SegmentLaneAssignment& assignment : lane_assignments) {
    const std::size_t lane_count = std::min(assignment.port_ids_a.size(), assignment.port_ids_b.size());
    for (std::size_t lane = 0; lane < lane_count && span_index < span_ids.size(); ++lane, ++span_index) {
      const ObjectId span_id = span_ids[span_index];
      const Span* span = edit_state.spans.find(span_id);
      const Port* port_a = edit_state.ports.find(assignment.port_ids_a[lane]);
      const Port* port_b = edit_state.ports.find(assignment.port_ids_b[lane]);
      if (span == nullptr || port_a == nullptr || port_b == nullptr) {
        continue;
      }
      const CurvePassMode pass_mode =
          (span->placement_context == ConnectionContext::kBranchAdd)
              ? CurvePassMode::kBranch
              : ((span->placement_context == ConnectionContext::kCornerPass) ? CurvePassMode::kPassThrough
                                                                             : CurvePassMode::kPassThrough);
      layouts.push_back(build_seed_layout_record(
          state, span_id, assignment.flow_kind, pass_mode, variation_flow_key, assignment.lowering_kind, *port_a, *port_b,
          build_seed_endpoint_record(edit_state, junction_relations_by_node, assignment, span->endpoint_node_a_id,
                                     *port_a, assignment.decision_a),
          build_seed_endpoint_record(edit_state, junction_relations_by_node, assignment, span->endpoint_node_b_id,
                                     *port_b, assignment.decision_b)));
    }
  }
  return layouts;
}

EditResult<BackboneGenerationRequestPlan> build_backbone_generation_request_plan(
    const CoreState& state, const BackboneSpec& spec) {
  EditResult<BackboneGenerationRequestPlan> result{};
  BackboneGenerationRequestPlan plan{};
  plan.request = spec;
  if (spec.path.polyline.size() < 2) {
    result.error = "backbone input path must contain at least 2 points";
    return result;
  }
  if (spec.interval_m <= 0.0) {
    result.error = "interval_m must be > 0";
    return result;
  }
  const CoreView core_view = state.view();
  if (core_view.pole_types().find(spec.pole_type_id) == core_view.pole_types().end()) {
    result.error = "pole type not found";
    return result;
  }
  if (!generation::detail::build_backbone_node_maps(spec, &plan.node_spec_by_index, &plan.node_bundle_mode_by_point,
                                                    &result.error)) {
    return result;
  }
  if (spec.bundles.empty()) {
    result.error = "bundles[] must contain at least one bundle request";
    return result;
  }

  plan.bundle_plans.reserve(spec.bundles.size());
  for (const BackboneBundleSpec& bundle_request : spec.bundles) {
    const auto bundle_template_it = core_view.bundle_templates().find(bundle_request.bundle_template_id);
    if (bundle_template_it == core_view.bundle_templates().end()) {
      result.error = "bundle template not found";
      return result;
    }
    const BundleTemplate* bundle_template = &bundle_template_it->second;
    BackboneBundlePlan bundle_plan{};
    bundle_plan.template_id = bundle_template->id;
    bundle_plan.category = bundle_template->category;
    bundle_plan.layer =
        (bundle_request.layer == SpanLayer::kUnknown) ? bundle_template->default_layer : bundle_request.layer;
    bundle_plan.spacing_m = bundle_template->default_spacing_m;
    bundle_plan.allow_mirror = bundle_template->allow_mirror;
    bundle_plan.preserve_conductor_identity = bundle_template->preserve_conductor_identity;
    bundle_plan.allow_midair_node = bundle_template->allow_midair_node;
    bundle_plan.allow_midair_branch = bundle_template->allow_midair_branch;
    bundle_plan.enable_branch_down_offset = bundle_template->enable_branch_down_offset;
    if (bundle_template->count_rule == BundleCountRuleKind::kFixed) {
      if (bundle_request.count > 0) {
        result.error = "count override is not allowed for fixed bundle template";
        return result;
      }
      bundle_plan.count = bundle_template->fixed_count;
    } else {
      bundle_plan.count = (bundle_request.count > 0) ? bundle_request.count : bundle_template->default_count;
      if (bundle_plan.count < bundle_template->min_count || bundle_plan.count > bundle_template->max_count) {
        result.error = "bundle count is out of template range";
        return result;
      }
    }
    if (bundle_plan.count <= 0) {
      result.error = "resolved bundle count must be > 0";
      return result;
    }
    bundle_plan.continuity_class =
        (bundle_plan.count > 1) ? ContinuityCategoryClass::kBundleLike : ContinuityCategoryClass::kPointLike;
    bundle_plan.order_decision_policy = bundle_template->order_decision_policy;
    if (bundle_plan.layer == SpanLayer::kUnknown) {
      result.error = "bundle layer could not be resolved";
      return result;
    }
    plan.bundle_plans.push_back(bundle_plan);
  }

  auto support_kind_for_point = [&](std::size_t point_index) -> SupportKind {
    const auto it = plan.node_spec_by_index.find(point_index);
    return (it == plan.node_spec_by_index.end()) ? SupportKind::kPole : it->second.support_kind;
  };
  auto node_spec_for_point = [&](std::size_t point_index) -> const BackboneInputSpec::NodeSpec* {
    const auto it = plan.node_spec_by_index.find(point_index);
    return (it == plan.node_spec_by_index.end()) ? nullptr : &it->second;
  };
  for (std::size_t point_index = 0; point_index < spec.path.polyline.size(); ++point_index) {
    if (support_kind_for_point(point_index) == SupportKind::kPole) {
      continue;
    }
    plan.request_has_non_pole_points = true;
  }

  plan.active_bundle_plans.reserve(plan.bundle_plans.size());
  for (const BackboneBundlePlan& bundle_plan : plan.bundle_plans) {
    if (plan.request_has_non_pole_points && !bundle_plan.allow_midair_node) {
      continue;
    }
    if (plan.request_has_source_edge_branch_points && !bundle_plan.allow_midair_branch) {
      continue;
    }
    plan.active_bundle_plans.push_back(bundle_plan);
  }
  for (std::size_t point_index = 0; point_index < spec.path.polyline.size(); ++point_index) {
    if (support_kind_for_point(point_index) == SupportKind::kPole) {
      continue;
    }
    const auto it_modes = plan.node_bundle_mode_by_point.find(point_index);
    if (it_modes == plan.node_bundle_mode_by_point.end()) {
      continue;
    }
    for (const auto& [bundle_template_id, mode] : it_modes->second) {
      const auto it_plan = std::find_if(plan.bundle_plans.begin(), plan.bundle_plans.end(),
                                        [&](const BackboneBundlePlan& p) { return p.template_id == bundle_template_id; });
      if (it_plan == plan.bundle_plans.end()) {
        result.error = "node_bundle_modes references bundle template that is not selected";
        return result;
      }
      if (mode != BundleNodeMode::kNotPresent && mode != BundleNodeMode::kPassThrough) {
        result.error = "unsupported bundle node mode";
        return result;
      }
    }
  }

  generation::detail::build_backbone_guide_points(spec, &plan.guide_points, &plan.direction_debug);
  if (!plan.active_bundle_plans.empty() &&
      !generation::detail::build_backbone_candidates(spec, plan.guide_points, plan.node_spec_by_index, &plan.candidates,
                                                     &result.error)) {
    return result;
  }
  result.value = std::move(plan);
  result.ok = true;
  return result;
}

} // namespace

EditResult<BackboneSupportChainPlan> CoreState::build_backbone_support_chain_plan(
    const BackboneGenerationRequestPlan& request_plan) const {
  EditResult<BackboneSupportChainPlan> result{};
  BackboneSupportChainPlan plan{};
  const CoreView core_view = view();
  const EditState& edit_state = core_view.edit_state();
  const BackboneResult existing_backbone = BuildBackboneResult();

  std::unordered_map<ObjectId, SupportNode> existing_support_nodes{};
  existing_support_nodes.reserve(existing_backbone.nodes.size());
  for (const SupportNode& node : existing_backbone.nodes) {
    existing_support_nodes.emplace(node.node_id, node);
  }

  auto node_spec_for_point = [&](std::size_t point_index) -> const BackboneInputSpec::NodeSpec* {
    const auto it = request_plan.node_spec_by_index.find(point_index);
    return (it == request_plan.node_spec_by_index.end()) ? nullptr : &it->second;
  };

  auto find_near_pole = [&](const Vec3d& world, PlacementMode preferred_mode) -> ObjectId {
    constexpr double kReuseRadius = 0.25;
    const double reuse_r2 = kReuseRadius * kReuseRadius;
    ObjectId best_id = kInvalidObjectId;
    double best_d2 = reuse_r2 + 1.0;
    bool best_mode_match = false;
    for (const Pole& pole : edit_state.poles.items()) {
      if (request_plan.request.pole_placement.restrict_reuse_to_session) {
        if (request_plan.request.pole_placement.reuse_session_id == 0 ||
            pole.generation.generation_session_id != request_plan.request.pole_placement.reuse_session_id) {
          continue;
        }
      }
      const Vec3d d = pole.world_transform.position - world;
      const double d2 = d.x * d.x + d.y * d.y + d.z * d.z;
      if (d2 > reuse_r2) {
        continue;
      }
      const bool mode_match = (pole.placement_mode == preferred_mode);
      if (best_id == kInvalidObjectId || (mode_match && !best_mode_match) ||
          (mode_match == best_mode_match && d2 < best_d2)) {
        best_id = pole.id;
        best_d2 = d2;
        best_mode_match = mode_match;
      }
    }
    return best_id;
  };

  auto ensure_support_node = [&](ObjectId node_id, const SupportNodeCandidate& candidate, ObjectId pole_id) {
    SupportNode& node = plan.support_node_by_id[node_id];
    node.node_id = node_id;
    node.support_kind = candidate.support_kind;
    node.position = candidate.world;
    node.pole_id = pole_id;
    node.path_point_index = candidate.vertex_index;
    node.has_tangent_hint = candidate.has_tangent_hint;
    node.tangent_hint = candidate.tangent_hint;
  };

  Vec3d preferred_side_dir{0.0, 0.0, 0.0};
  bool has_preferred_side_dir = false;
  ObjectId next_virtual_support_id = 0x8000000000000000ull;
  ObjectId next_planned_pole_id = 0x4000000000000000ull;

  plan.resolutions.reserve(request_plan.candidates.size());
  plan.ordered_pole_ids.reserve(request_plan.candidates.size());
  plan.ordered_support_node_ids.reserve(request_plan.candidates.size());

  for (std::size_t i = 0; i < request_plan.candidates.size(); ++i) {
    const SupportNodeCandidate& candidate = request_plan.candidates[i];
    const BackboneInputSpec::NodeSpec* explicit_node_spec =
        (candidate.vertex_index >= 0) ? node_spec_for_point(static_cast<std::size_t>(candidate.vertex_index)) : nullptr;
    const ObjectId explicit_node_id =
        (explicit_node_spec != nullptr) ? explicit_node_spec->node_id : kInvalidObjectId;

    BackboneSupportResolution resolution{};
    resolution.candidate_index = static_cast<int>(i);
    resolution.support_kind = candidate.support_kind;
    resolution.placement_mode = candidate.mode;

    if (candidate.support_kind != SupportKind::kPole) {
      ObjectId support_node_id = explicit_node_id;
      if (support_node_id == kInvalidObjectId) {
        support_node_id = next_virtual_support_id++;
        resolution.kind = BackboneSupportResolutionKind::kCreateVirtualSupport;
      } else {
        const auto it_existing = existing_support_nodes.find(support_node_id);
        if (it_existing == existing_support_nodes.end()) {
          result.error = "node_specs node_id for support node was not found";
          return result;
        }
        if (it_existing->second.support_kind != candidate.support_kind) {
          result.error = "node_specs node_id support kind does not match path node kind";
          return result;
        }
        plan.support_node_by_id[support_node_id] = it_existing->second;
        resolution.kind = BackboneSupportResolutionKind::kReuseSupportNode;
        resolution.existing_node_id = support_node_id;
      }
      resolution.planned_node_id = support_node_id;
      ensure_support_node(support_node_id, candidate, kInvalidObjectId);
      plan.ordered_support_node_ids.push_back(support_node_id);
      plan.resolutions.push_back(resolution);
      continue;
    }

    ObjectId pole_id = explicit_node_id;
    if (pole_id != kInvalidObjectId && edit_state.poles.find(pole_id) == nullptr) {
      result.error = "node_specs node_id for pole was not found";
      return result;
    }
    if (pole_id == kInvalidObjectId) {
      pole_id = find_near_pole(candidate.world, candidate.mode);
    }
    if (pole_id != kInvalidObjectId) {
      const Pole* existing_pole = edit_state.poles.find(pole_id);
      if (existing_pole == nullptr) {
        result.error = "resolved reused pole was not found";
        return result;
      }
      resolution.kind = BackboneSupportResolutionKind::kReusePole;
      resolution.existing_node_id = pole_id;
      resolution.planned_node_id = pole_id;
      Pole authored_pole = *existing_pole;
      if (candidate.mode == PlacementMode::kManual) {
        apply_pole_placement_mode(authored_pole, PlacementMode::kManual);
      }
      if (candidate.vertex_index >= 0) {
        Vec3d base_rotation_euler_deg = existing_pole->world_transform.rotation_euler_deg;
        base_rotation_euler_deg.z = 0.0;
        const AutoPoleTransformResult auto_tf =
            compute_auto_pole_transform(request_plan.guide_points, static_cast<std::size_t>(candidate.vertex_index),
                                        has_preferred_side_dir ? &preferred_side_dir : nullptr,
                                        &base_rotation_euler_deg);
        authored_pole.context =
            classify_pole_context_from_path(request_plan.guide_points, static_cast<std::size_t>(candidate.vertex_index), 0);
        apply_sharp_debug_to_context(&authored_pole.context, auto_tf.sharp);
        if (!has_pole_orientation_override(authored_pole.id)) {
          authored_pole.world_transform.rotation_euler_deg.z = auto_tf.transform.rotation_euler_deg.z;
        }
        const PoleFrame preferred_frame = BuildPoleFrame(auto_tf.transform, auto_tf.transform.rotation_euler_deg.z);
        preferred_side_dir = preferred_frame.lateral;
        has_preferred_side_dir = Normalize(&preferred_side_dir);
      } else {
        authored_pole.context.kind = PoleContextKind::kStraight;
        apply_sharp_debug_to_context(&authored_pole.context, SharpCornerOrientationDebug{});
        if (!has_pole_orientation_override(authored_pole.id)) {
          const Vec3d dir =
              request_plan.guide_points[candidate.segment_index + 1] - request_plan.guide_points[candidate.segment_index];
          if ((dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) > 1e-12) {
            authored_pole.world_transform.rotation_euler_deg.z =
                normalize_yaw_deg(std::atan2(dir.y, dir.x) * (180.0 / kPi));
      }
      resolution.authored_transform = authored_pole.world_transform;
      resolution.authored_context = authored_pole.context;
      plan.ordered_pole_ids.push_back(pole_id);
      ensure_support_node(pole_id, candidate, pole_id);
      plan.ordered_support_node_ids.push_back(pole_id);
      plan.resolutions.push_back(resolution);
      continue;
    }

    Transformd tf{};
    PoleContextInfo context{};
    tf.position = candidate.world;
    if (candidate.vertex_index >= 0) {
      const AutoPoleTransformResult auto_tf =
          compute_auto_pole_transform(request_plan.guide_points, static_cast<std::size_t>(candidate.vertex_index),
                                      has_preferred_side_dir ? &preferred_side_dir : nullptr);
      tf = auto_tf.transform;
      tf.position = candidate.world;
      context = classify_pole_context_from_path(request_plan.guide_points, static_cast<std::size_t>(candidate.vertex_index), 0);
      apply_sharp_debug_to_context(&context, auto_tf.sharp);
      const PoleFrame preferred_frame = BuildPoleFrame(tf, tf.rotation_euler_deg.z);
      preferred_side_dir = preferred_frame.lateral;
      has_preferred_side_dir = Normalize(&preferred_side_dir);
    } else {
      context.kind = PoleContextKind::kStraight;
      const Vec3d dir = request_plan.guide_points[candidate.segment_index + 1] - request_plan.guide_points[candidate.segment_index];
      if ((dir.x * dir.x + dir.y * dir.y + dir.z * dir.z) > 1e-12) {
        tf.rotation_euler_deg.z = normalize_yaw_deg(std::atan2(dir.y, dir.x) * (180.0 / kPi));
      }
    }

    const ObjectId planned_pole_id = next_planned_pole_id++;
    resolution.kind = BackboneSupportResolutionKind::kCreatePole;
    resolution.planned_node_id = planned_pole_id;
    resolution.authored_transform = tf;
    resolution.authored_context = context;
    plan.pole_creations.push_back({planned_pole_id, tf, candidate.mode, static_cast<int>(i), context,
                                   request_plan.request.pole_type_id, PoleKind::kConcrete, 10.0, "PathPole"});
    plan.generated_pole_ids.push_back(planned_pole_id);
    plan.ordered_pole_ids.push_back(planned_pole_id);
    ensure_support_node(planned_pole_id, candidate, planned_pole_id);
    plan.ordered_support_node_ids.push_back(planned_pole_id);
    plan.resolutions.push_back(resolution);
  }

  auto compact_ids = [](std::vector<ObjectId>* ids) {
    if (ids == nullptr) {
      return;
    }
    std::vector<ObjectId> compact{};
    compact.reserve(ids->size());
    for (ObjectId id : *ids) {
      if (compact.empty() || compact.back() != id) {
        compact.push_back(id);
      }
    }
    ids->swap(compact);
  };
  compact_ids(&plan.ordered_pole_ids);
  compact_ids(&plan.ordered_support_node_ids);
  if (plan.ordered_support_node_ids.size() < 2) {
    result.error = "failed to build valid support-node chain";
    return result;
  }

  result.value = std::move(plan);
  result.ok = true;
  return result;
}

namespace {

EditResult<BackboneTopologyPlan> build_backbone_topology_plan(
    const CoreState& state, const BackboneGenerationRequestPlan& request_plan, const BackboneSupportChainPlan& support_chain_plan) {
  (void)request_plan;
  EditResult<BackboneTopologyPlan> result{};
  BackboneTopologyPlan plan{};
  plan.existing_network_backbone = state.BuildBackboneResult();

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
    plan.existing_junction_by_node[junction.node_id] = &junction;
    plan.existing_prioritized_session_by_node[junction.node_id] = junction.prioritized_session_id;
    for (const JunctionIncident& incident : junction.incidents) {
      plan.existing_incident_session_by_node[junction.node_id][incident.neighbor_node_id] = incident.source_session_id;
      if (incident.primary) {
        plan.existing_primary_neighbor_by_node[junction.node_id] = incident.neighbor_node_id;
      }
    }
  }
  for (const BackboneEdge& edge : plan.existing_network_backbone.edges) {
    if (edge.node_a == kInvalidObjectId || edge.node_b == kInvalidObjectId || edge.node_a == edge.node_b) {
      continue;
    }
    plan.existing_adjacency[edge.node_a].push_back(edge.node_b);
    plan.existing_adjacency[edge.node_b].push_back(edge.node_a);
  }
  for (auto& [_, neighbors] : plan.existing_adjacency) {
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
  }
  for (const SupportNode& node : plan.existing_network_backbone.nodes) {
    plan.existing_node_position_by_id[node.node_id] = node.position;
  }
  for (const Pole& pole : state.view().edit_state().poles.items()) {
    plan.existing_node_position_by_id.try_emplace(pole.id, pole.world_transform.position);
  }
  plan.active_junction_by_node = plan.existing_junction_by_node;
  plan.decision_phase = build_backbone_decision_phase(state, request_plan, support_chain_plan, &plan);

  result.value = std::move(plan);
  result.ok = true;
  return result;
}

} // namespace

void CoreState::refresh_committed_backbone_seed_cache(
    const std::vector<ObjectId>& generated_span_ids,
    const std::unordered_map<ObjectId, JunctionRelation>& junction_relations_by_node) {
  const std::unordered_set<ObjectId> generated_span_set(generated_span_ids.begin(), generated_span_ids.end());
  const std::unordered_set<ObjectId> touched_nodes = [&]() {
    std::unordered_set<ObjectId> ids{};
    ids.reserve(junction_relations_by_node.size());
    for (const auto& [node_id, _] : junction_relations_by_node) {
      ids.insert(node_id);
    }
    return ids;
  }();
  for (const Span& span : edit_state_access().spans.items()) {
    if (generated_span_set.contains(span.id) ||
        (!touched_nodes.contains(span.endpoint_node_a_id) && !touched_nodes.contains(span.endpoint_node_b_id))) {
      continue;
    }
    const SpanSupportLayoutDecisionSeed* cached_seed = find_span_support_layout_seed(span.id);
    const auto rebuilt_seed = rebuild_existing_span_decision_seed(edit_state_access(), span, cached_seed);
    if (!rebuilt_seed.has_value()) {
      continue;
    }
    cache_span_support_layout_seed(*rebuilt_seed);
    mark_span_dirty(span.id, DirtyBits::kDecision, true);
  }
}

void CoreState::publish_committed_backbone_debug_state(const BackboneCommittedGenerationPlan& plan,
                                                       BackboneMaterializationPhaseOutput* materialization_phase) {
  if (materialization_phase == nullptr) {
    return;
  }

  debug_.last_generation_support_nodes.clear();
  debug_.last_generation_support_nodes.reserve(plan.topology_state.generation_backbone.nodes.size());
  for (const SupportNode& node : plan.topology_state.generation_backbone.nodes) {
    if (node.support_kind != SupportKind::kPole) {
      debug_.last_generation_support_nodes.push_back(node);
    }
  }
  debug_.last_generation_lane_assignments = std::move(materialization_phase->lane_assignments);
  debug_.last_generation_edge_orientations = std::move(materialization_phase->edge_orientations);
  debug_.last_generation_junction_relations = std::move(materialization_phase->junction_relations_by_node);
}

EditResult<GenerateBundleFromPathResult> CoreState::execute_committed_backbone_generation_plan(
    const BackboneGenerationRequestPlan& request_plan, const BackboneOrientationPlan& orientation_plan,
    BackboneCommittedGenerationPlan committed_plan, std::vector<ObjectId> generated_pole_ids, ChangeSet initial_change_set) {
  EditResult<GenerateBundleFromPathResult> result{};
  result.change_set = std::move(initial_change_set);
  result.value.generated_pole_ids = std::move(generated_pole_ids);

  apply_committed_backbone_orientation_plan(request_plan, orientation_plan, &committed_plan, &result.change_set);

  EditResult<BackboneMaterializationPhaseOutput> materialization_phase_result =
      run_committed_backbone_materialization_phase(request_plan, committed_plan);
  if (!materialization_phase_result.ok) {
    result.error = materialization_phase_result.error;
    return result;
  }
  BackboneMaterializationPhaseOutput materialization_phase = std::move(materialization_phase_result.value);
  append_change_set(result.change_set, materialization_phase.change_set);
  result.value.bundle_ids.insert(result.value.bundle_ids.end(), materialization_phase.bundle_ids.begin(),
                                 materialization_phase.bundle_ids.end());
  if (result.value.bundle_id == kInvalidObjectId) {
    result.value.bundle_id = materialization_phase.primary_bundle_id;
  }
  result.value.generated_span_ids.insert(result.value.generated_span_ids.end(),
                                         materialization_phase.generated_span_ids.begin(),
                                         materialization_phase.generated_span_ids.end());

  refresh_committed_backbone_seed_cache(result.value.generated_span_ids, materialization_phase.junction_relations_by_node);
  publish_committed_backbone_debug_state(committed_plan, &materialization_phase);
  result.ok = true;
  return result;
}

EditResult<BackboneCommittedSupportChain> CoreState::commit_backbone_support_chain_plan(
    const BackboneSupportChainPlan& support_chain_plan) {
  EditResult<BackboneCommittedSupportChain> result{};
  BackboneCommittedSupportChain committed{};
  committed.session_id = next_generation_session_id_access()++;
  committed.generated_pole_ids.reserve(support_chain_plan.generated_pole_ids.size());
  committed.ordered_support_node_ids.reserve(support_chain_plan.ordered_support_node_ids.size());
  committed.support_node_by_id.reserve(support_chain_plan.support_node_by_id.size());
  committed.committed_node_id_by_planned_node_id.reserve(support_chain_plan.resolutions.size());

  std::unordered_map<ObjectId, const PlannedPoleCreation*> planned_pole_creation_by_id{};
  planned_pole_creation_by_id.reserve(support_chain_plan.pole_creations.size());
  for (const PlannedPoleCreation& creation : support_chain_plan.pole_creations) {
    planned_pole_creation_by_id[creation.planned_pole_id] = &creation;
  }

  auto authored_support_node_for = [&](ObjectId planned_node_id) -> const SupportNode* {
    const auto it = support_chain_plan.support_node_by_id.find(planned_node_id);
    return (it == support_chain_plan.support_node_by_id.end()) ? nullptr : &it->second;
  };
  auto commit_support_node = [&](ObjectId planned_node_id, ObjectId committed_node_id, ObjectId committed_pole_id) {
    const SupportNode* authored_node = authored_support_node_for(planned_node_id);
    if (authored_node == nullptr) {
      result.error = "support-chain plan is missing authored support node";
      return false;
    }
    SupportNode node = *authored_node;
    node.node_id = committed_node_id;
    node.pole_id = committed_pole_id;
    committed.support_node_by_id[committed_node_id] = std::move(node);
    committed.ordered_support_node_ids.push_back(committed_node_id);
    committed.committed_node_id_by_planned_node_id[planned_node_id] = committed_node_id;
    return true;
  };

  std::size_t committed_pole_order = 0;
  for (const BackboneSupportResolution& resolution : support_chain_plan.resolutions) {
    if (resolution.support_kind != SupportKind::kPole) {
      if (resolution.kind == BackboneSupportResolutionKind::kReuseSupportNode) {
        const auto it_existing = support_chain_plan.support_node_by_id.find(resolution.planned_node_id);
        if (it_existing == support_chain_plan.support_node_by_id.end()) {
          result.error = "support-chain plan is missing reused support node";
          return result;
        }
        committed.support_node_by_id[resolution.planned_node_id] = it_existing->second;
      }
      if (!commit_support_node(resolution.planned_node_id, resolution.planned_node_id, kInvalidObjectId)) {
        return result;
      }
      continue;
    }

    if (resolution.kind == BackboneSupportResolutionKind::kReusePole) {
      Pole* pole = edit_state_access().poles.find(resolution.existing_node_id);
      if (pole == nullptr) {
        result.error = "support-chain plan references missing reused pole";
        return result;
      }
      const Pole old_pole = *pole;
      pole->world_transform = resolution.authored_transform;
      pole->context = resolution.authored_context;
      apply_pole_placement_mode(*pole, resolution.placement_mode);
      finalize_pole_transform_update(pole->id, old_pole, &committed.change_set);
      if (!commit_support_node(resolution.planned_node_id, pole->id, pole->id)) {
        return result;
      }
      ++committed_pole_order;
      continue;
    }

    if (resolution.kind != BackboneSupportResolutionKind::kCreatePole) {
      result.error = "unsupported support-chain resolution kind";
      return result;
    }

    const auto it_creation = planned_pole_creation_by_id.find(resolution.planned_node_id);
    if (it_creation == planned_pole_creation_by_id.end() || it_creation->second == nullptr) {
      result.error = "support-chain plan is missing planned pole creation";
      return result;
    }
    const PlannedPoleCreation& creation = *it_creation->second;
    EditResult<ObjectId> add_pole = AddPole(creation.transform, creation.height_m, creation.name, creation.pole_kind,
                                            creation.placement_mode);
    if (!add_pole.ok) {
      result.error = add_pole.error;
      return result;
    }
    Pole* pole = edit_state_access().poles.find(add_pole.value);
    if (pole != nullptr) {
      pole->world_transform = creation.transform;
      pole->context = creation.context;
      pole->generation.generated = true;
      pole->generation.source = GenerationSource::kRoadAuto;
      pole->generation.generation_session_id = committed.session_id;
      pole->generation.generation_order = static_cast<std::uint32_t>(committed_pole_order);
      add_unique_id(add_pole.change_set.updated_ids, pole->id);
    }
    EditResult<ObjectId> apply_type = ApplyPoleType(add_pole.value, creation.pole_type_id);
    if (!apply_type.ok) {
      result.error = apply_type.error;
      return result;
    }
    append_change_set(committed.change_set, add_pole.change_set);
    append_change_set(committed.change_set, apply_type.change_set);
    committed.generated_pole_ids.push_back(add_pole.value);
    if (!commit_support_node(resolution.planned_node_id, add_pole.value, add_pole.value)) {
      return result;
    }
    ++committed_pole_order;
  }

  {
    std::vector<ObjectId> compact_support_ids{};
    compact_support_ids.reserve(committed.ordered_support_node_ids.size());
    for (ObjectId id : committed.ordered_support_node_ids) {
      if (compact_support_ids.empty() || compact_support_ids.back() != id) {
        compact_support_ids.push_back(id);
      }
    }
    committed.ordered_support_node_ids.swap(compact_support_ids);
  }
  if (committed.ordered_support_node_ids.size() < 2) {
    result.error = "failed to commit valid support-node chain";
    return result;
  }

  result.value = std::move(committed);
  result.ok = true;
  return result;
}

EditResult<GenerateBundleFromPathResult> CoreState::legacy_generate_from_backbone_spec(const BackboneSpec& spec) {
  EditResult<GenerateBundleFromPathResult> result{};
  EditResult<BackboneGenerationRequestPlan> request_plan_result = build_backbone_generation_request_plan(*this, spec);
  if (!request_plan_result.ok) {
    result.error = request_plan_result.error;
    return result;
  }
  const BackboneGenerationRequestPlan request_plan = std::move(request_plan_result.value);
  debug_.last_path_direction_debug = request_plan.direction_debug;
  path_direction_debug_records_access().push_back(request_plan.direction_debug);
  if (path_direction_debug_records_access().size() > 128) {
    path_direction_debug_records_access().erase(path_direction_debug_records_access().begin());
  }
  if (request_plan.active_bundle_plans.empty()) {
    result.ok = true;
    return result;
  }

  EditResult<BackboneSupportChainPlan> support_chain_plan_result = build_backbone_support_chain_plan(request_plan);
  if (!support_chain_plan_result.ok) {
    result.error = support_chain_plan_result.error;
    return result;
  }
  EditResult<BackboneTopologyPlan> topology_plan_result =
      build_backbone_topology_plan(*this, request_plan, support_chain_plan_result.value);
  if (!topology_plan_result.ok) {
    result.error = topology_plan_result.error;
    return result;
  }
  BackboneOrientationPlan orientation_plan{};
  orientation_plan.previous_debug_records = debug_.pole_orientation_debug_records;

  const CoreState snapshot = *this;
  EditResult<BackboneCommittedSupportChain> committed_chain_result =
      commit_backbone_support_chain_plan(support_chain_plan_result.value);
  if (!committed_chain_result.ok) {
    *this = snapshot;
    result.error = committed_chain_result.error;
    return result;
  }

  EditResult<BackboneCommittedGenerationPlan> committed_plan_result = build_committed_backbone_generation_plan(
      topology_plan_result.value, committed_chain_result.value.session_id,
      std::move(committed_chain_result.value.ordered_support_node_ids),
      std::move(committed_chain_result.value.support_node_by_id),
      std::move(committed_chain_result.value.committed_node_id_by_planned_node_id));
  if (!committed_plan_result.ok) {
    *this = snapshot;
    result.error = committed_plan_result.error;
    return result;
  }

  result = execute_committed_backbone_generation_plan(request_plan, orientation_plan,
                                                      std::move(committed_plan_result.value),
                                                      std::move(committed_chain_result.value.generated_pole_ids),
                                                      std::move(committed_chain_result.value.change_set));
  if (!result.ok) {
    *this = snapshot;
  }
  return result;
}

EditResult<std::unique_ptr<BackboneGenerationPlan>> CoreState::build_backbone_generation_plan(
    const BackboneSpec& spec) const {
  EditResult<std::unique_ptr<BackboneGenerationPlan>> result{};
  EditResult<BackboneGenerationRequestPlan> request_plan_result = build_backbone_generation_request_plan(*this, spec);
  if (!request_plan_result.ok) {
    result.error = request_plan_result.error;
    return result;
  }

  auto plan = std::make_unique<BackboneGenerationPlan>();
  plan->request_plan = std::move(request_plan_result.value);

  EditResult<BackboneSupportChainPlan> support_chain_plan_result = build_backbone_support_chain_plan(plan->request_plan);
  if (!support_chain_plan_result.ok) {
    result.error = support_chain_plan_result.error;
    return result;
  }
  plan->support_chain_plan = std::move(support_chain_plan_result.value);

  EditResult<BackboneTopologyPlan> topology_plan_result =
      build_backbone_topology_plan(*this, plan->request_plan, plan->support_chain_plan);
  if (!topology_plan_result.ok) {
    result.error = topology_plan_result.error;
    return result;
  }
  plan->topology_plan = std::move(topology_plan_result.value);
  plan->orientation_plan.previous_debug_records = debug_.pole_orientation_debug_records;

  result.value = std::move(plan);
  result.ok = true;
  return result;
}

EditResult<bool> CoreState::validate_backbone_generation_plan(const BackboneGenerationPlan& plan) const {
  EditResult<bool> result{};
  if (plan.request_plan.request.path.polyline.size() < 2) {
    result.error = "backbone generation plan is missing path points";
    return result;
  }
  if (plan.request_plan.bundle_plans.empty()) {
    result.error = "backbone generation plan is missing bundle plans";
    return result;
  }
  if (plan.support_chain_plan.ordered_support_node_ids.size() < 2) {
    result.error = "backbone generation plan is missing support chain";
    return result;
  }
  if (plan.topology_plan.generation_backbone.edges.empty()) {
    result.error = "backbone generation plan is missing topology edges";
    return result;
  }
  result.value = true;
  result.ok = true;
  return result;
}

EditResult<GenerateBundleFromPathResult> CoreState::commit_backbone_generation_plan(
    std::unique_ptr<BackboneGenerationPlan> plan) {
  EditResult<GenerateBundleFromPathResult> result{};
  if (!plan) {
    result.error = "backbone generation plan is null";
    return result;
  }
  debug_.last_path_direction_debug = plan->request_plan.direction_debug;
  path_direction_debug_records_access().push_back(plan->request_plan.direction_debug);
  if (path_direction_debug_records_access().size() > 128) {
    path_direction_debug_records_access().erase(path_direction_debug_records_access().begin());
  }
  if (plan->request_plan.active_bundle_plans.empty()) {
    result.ok = true;
    return result;
  }

  const CoreState snapshot = *this;
  EditResult<BackboneCommittedSupportChain> committed_chain_result =
      commit_backbone_support_chain_plan(plan->support_chain_plan);
  if (!committed_chain_result.ok) {
    *this = snapshot;
    result.error = committed_chain_result.error;
    return result;
  }

  EditResult<BackboneCommittedGenerationPlan> committed_plan_result = build_committed_backbone_generation_plan(
      plan->topology_plan, committed_chain_result.value.session_id,
      std::move(committed_chain_result.value.ordered_support_node_ids),
      std::move(committed_chain_result.value.support_node_by_id),
      std::move(committed_chain_result.value.committed_node_id_by_planned_node_id));
  if (!committed_plan_result.ok) {
    *this = snapshot;
    result.error = committed_plan_result.error;
    return result;
  }

  result = execute_committed_backbone_generation_plan(plan->request_plan, plan->orientation_plan,
                                                      std::move(committed_plan_result.value),
                                                      std::move(committed_chain_result.value.generated_pole_ids),
                                                      std::move(committed_chain_result.value.change_set));
  if (!result.ok) {
    *this = snapshot;
  }
  return result;
}


} // namespace wire::core
