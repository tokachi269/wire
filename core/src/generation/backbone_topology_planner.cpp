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


std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9E3779B97F4A7C15ull;
  x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
  x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
  return x ^ (x >> 31);
}

std::uint64_t hash_combine(std::uint64_t seed, std::uint64_t value) {
  return splitmix64(seed ^ (value + 0x9E3779B97F4A7C15ull + (seed << 6) + (seed >> 2)));
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
  for (const Pole& pole : view().edit_state().poles.items()) {
    plan.existing_node_position_by_id.try_emplace(pole.id, pole.world_transform.position);
  }
  plan.active_junction_by_node = plan.existing_junction_by_node;
  plan.decision_phase = build_backbone_decision_phase(*this, request_plan, support_chain_plan, &plan);

  result.value = std::move(plan);
  result.ok = true;
  return result;
}

} // namespace wire::core


