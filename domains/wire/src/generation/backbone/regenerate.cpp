#include "city/wire/core_state.hpp"
#include "city/wire/core_view.hpp"
#include "city/wire/coord_utils.hpp"

#include "../../collection_utils.hpp"
#include "curve_parts.hpp"
#include "model_assembly.hpp"
#include "pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace city::wire {
namespace {

using detail::append_unique;

bool contains_id(const std::vector<ObjectId>& ids, ObjectId id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

const SavedBackboneNode* saved_node_by_id(const SavedBackboneGraph& graph, ObjectId node_id) {
  const auto it = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                               [&](const SavedBackboneNode& node) { return node.node_id == node_id; });
  return it == graph.nodes.end() ? nullptr : &*it;
}

const SavedBackboneEdge* saved_edge_by_id(const SavedBackboneGraph& graph, ObjectId edge_id) {
  const auto it = std::find_if(graph.edges.begin(), graph.edges.end(),
                               [&](const SavedBackboneEdge& edge) { return edge.edge_id == edge_id; });
  return it == graph.edges.end() ? nullptr : &*it;
}

const SavedBackboneEdgeBundle* saved_edge_bundle_by_id(const SavedBackboneGraph& graph, ObjectId edge_bundle_id) {
  const auto it = std::find_if(graph.edge_bundles.begin(), graph.edge_bundles.end(),
                               [&](const SavedBackboneEdgeBundle& edge_bundle) {
                                 return edge_bundle.edge_bundle_id == edge_bundle_id;
                               });
  return it == graph.edge_bundles.end() ? nullptr : &*it;
}

struct RegenerateTarget {
  ObjectId edge_bundle_id = kInvalidObjectId;
  std::vector<ObjectId> edge_bundle_ids{};
  ObjectId bundle_id = kInvalidObjectId;
  ObjectId edge_id = kInvalidObjectId;
  const SavedBackboneEdge* edge = nullptr;
  const SavedBackboneNode* node_a = nullptr;
  const SavedBackboneNode* node_b = nullptr;
  std::vector<ObjectId> retired_spans{};
  std::vector<ObjectId> retired_ports{};
};

struct ContinuityNeighbor {
  ObjectId edge_bundle_id = kInvalidObjectId;
  ObjectId via_node_id = kInvalidObjectId;
};

struct LoadedRouteEdge {
  ObjectId edge_bundle_id = kInvalidObjectId;
  const SavedBackboneEdge* edge = nullptr;
  ObjectId from_node_id = kInvalidObjectId;
  ObjectId to_node_id = kInvalidObjectId;
  Vec3d dir{};
};

struct LoadedRoute {
  std::vector<LoadedRouteEdge> edges{};
  std::vector<ObjectId> edge_bundle_ids{};
};

ObjectId other_edge_node(const SavedBackboneEdge& edge, ObjectId node_id) {
  if (edge.node_a == node_id) return edge.node_b;
  if (edge.node_b == node_id) return edge.node_a;
  return kInvalidObjectId;
}

Vec3d oriented_edge_dir(const SavedBackboneEdge& edge, ObjectId from_node_id, ObjectId to_node_id) {
  if (edge.node_a == from_node_id && edge.node_b == to_node_id) return edge.dir;
  if (edge.node_b == from_node_id && edge.node_a == to_node_id) return ScaleVec(edge.dir, -1.0);
  return edge.dir;
}

void add_continuity_neighbor(
    std::unordered_map<ObjectId, std::vector<ContinuityNeighbor>>* adjacency,
    ObjectId edge_bundle_id,
    ObjectId neighbor_edge_bundle_id,
    ObjectId via_node_id) {
  if (adjacency == nullptr || edge_bundle_id == kInvalidObjectId ||
      neighbor_edge_bundle_id == kInvalidObjectId ||
      edge_bundle_id == neighbor_edge_bundle_id ||
      via_node_id == kInvalidObjectId) {
    return;
  }
  std::vector<ContinuityNeighbor>& neighbors = (*adjacency)[edge_bundle_id];
  const auto duplicate = std::find_if(neighbors.begin(), neighbors.end(), [&](const ContinuityNeighbor& item) {
    return item.edge_bundle_id == neighbor_edge_bundle_id &&
           item.via_node_id == via_node_id;
  });
  if (duplicate == neighbors.end()) {
    neighbors.push_back(
        ContinuityNeighbor{neighbor_edge_bundle_id, via_node_id});
  }
}

EditResult<LoadedRoute> continuity_route_from_saved_graph(
    const SavedBackboneGraph& graph, ObjectId seed_edge_bundle_id) {
  EditResult<LoadedRoute> result{};
  std::unordered_map<ObjectId, const SavedBackboneEdge*> edge_by_id{};
  std::unordered_map<ObjectId, const SavedBackboneEdgeBundle*> edge_bundle_by_id{};
  for (const SavedBackboneEdge& edge : graph.edges) {
    edge_by_id[edge.edge_id] = &edge;
  }
  for (const SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
    edge_bundle_by_id[edge_bundle.edge_bundle_id] = &edge_bundle;
  }

  std::unordered_map<ObjectId, std::vector<ContinuityNeighbor>> adjacency{};
  for (const SavedBackboneRowContinuity& continuity : graph.row_continuities) {
    add_continuity_neighbor(&adjacency, continuity.a.edge_bundle_id,
                            continuity.b.edge_bundle_id, continuity.node_id);
    add_continuity_neighbor(&adjacency, continuity.b.edge_bundle_id,
                            continuity.a.edge_bundle_id, continuity.node_id);
  }

  if (!edge_bundle_by_id.contains(seed_edge_bundle_id)) {
    result.error = "authoritative invalid input: authoritative deserialization: row continuity edge bundle is missing";
    return result;
  }
  std::vector<ObjectId> component{};
  std::vector<ObjectId> pending{seed_edge_bundle_id};
  for (std::size_t i = 0; i < pending.size(); ++i) {
    const ObjectId current_id = pending[i];
    if (contains_id(component, current_id)) {
      continue;
    }
    if (!edge_bundle_by_id.contains(current_id)) {
      result.error = "authoritative invalid input: authoritative deserialization: row continuity edge bundle is missing";
      return result;
    }
    component.push_back(current_id);
    const auto neighbors_it = adjacency.find(current_id);
    if (neighbors_it == adjacency.end()) {
      continue;
    }
    for (const ContinuityNeighbor& neighbor : neighbors_it->second) {
      if (!contains_id(pending, neighbor.edge_bundle_id)) {
        pending.push_back(neighbor.edge_bundle_id);
      }
    }
  }
  std::vector<ObjectId> remaining = component;
  auto remove_remaining = [&](ObjectId edge_id) {
    remaining.erase(std::remove(remaining.begin(), remaining.end(), edge_id), remaining.end());
  };
  auto degree = [&](ObjectId edge_id) {
    const auto it = adjacency.find(edge_id);
    return it == adjacency.end() ? std::size_t{0} : it->second.size();
  };
  auto choose_start = [&]() {
    ObjectId selected = kInvalidObjectId;
    for (ObjectId edge_bundle_id : remaining) {
      if (selected == kInvalidObjectId ||
          (degree(edge_bundle_id) <= 1 &&
           (degree(selected) > 1 || edge_bundle_id < selected)) ||
          (degree(edge_bundle_id) == degree(selected) &&
           edge_bundle_id < selected)) {
        selected = edge_bundle_id;
      }
    }
    return selected;
  };

  const ObjectId start_id = choose_start();
  std::vector<const SavedBackboneEdgeBundle*> edge_bundles{};
  std::vector<const SavedBackboneEdge*> edges{};
  std::vector<ObjectId> shared_nodes{};
  ObjectId previous_edge_bundle_id = kInvalidObjectId;
  ObjectId current_edge_bundle_id = start_id;
  for (;;) {
    const auto current_bundle_it =
        edge_bundle_by_id.find(current_edge_bundle_id);
    if (current_bundle_it == edge_bundle_by_id.end()) {
      result.error = "authoritative invalid input: authoritative deserialization: row continuity edge bundle is missing";
      return result;
    }
    const auto current_edge_it =
        edge_by_id.find(current_bundle_it->second->edge_id);
    if (current_edge_it == edge_by_id.end()) {
      result.error = "authoritative invalid input: authoritative deserialization: saved route edge is missing";
      return result;
    }
    if (!contains_id(remaining, current_edge_bundle_id)) {
      result.error = "authoritative invalid input: authoritative deserialization: saved route continuity cycle is ambiguous";
      return result;
    }
    edge_bundles.push_back(current_bundle_it->second);
    edges.push_back(current_edge_it->second);
    remove_remaining(current_edge_bundle_id);

    std::vector<ContinuityNeighbor> next_candidates{};
    const auto neighbors_it = adjacency.find(current_edge_bundle_id);
    if (neighbors_it != adjacency.end()) {
      for (const ContinuityNeighbor& neighbor : neighbors_it->second) {
        if (neighbor.edge_bundle_id != previous_edge_bundle_id) {
          next_candidates.push_back(neighbor);
        }
      }
    }
    if (next_candidates.empty()) {
      break;
    }
    if (next_candidates.size() > 1) {
      result.error = "authoritative invalid input: authoritative deserialization: saved route continuity is ambiguous";
      return result;
    }
    shared_nodes.push_back(next_candidates.front().via_node_id);
    previous_edge_bundle_id = current_edge_bundle_id;
    current_edge_bundle_id = next_candidates.front().edge_bundle_id;
  }
  if (!remaining.empty()) {
    result.error = "authoritative invalid input: authoritative deserialization: saved route continuity is ambiguous";
    return result;
  }

  result.value.edges.reserve(edges.size());
  result.value.edge_bundle_ids.reserve(edge_bundles.size());
  for (std::size_t i = 0; i < edges.size(); ++i) {
    const SavedBackboneEdge& edge = *edges[i];
    ObjectId from_node_id = kInvalidObjectId;
    ObjectId to_node_id = kInvalidObjectId;
    if (edges.size() == 1) {
      from_node_id = edge.node_a;
      to_node_id = edge.node_b;
    } else if (i == 0) {
      to_node_id = shared_nodes.front();
      from_node_id = other_edge_node(edge, to_node_id);
    } else if (i + 1 == edges.size()) {
      from_node_id = shared_nodes[i - 1];
      to_node_id = other_edge_node(edge, from_node_id);
    } else {
      from_node_id = shared_nodes[i - 1];
      to_node_id = shared_nodes[i];
    }
    if (from_node_id == kInvalidObjectId ||
        to_node_id == kInvalidObjectId || from_node_id == to_node_id) {
      result.error = "authoritative invalid input: authoritative deserialization: saved route continuity endpoint is invalid";
      return result;
    }
    result.value.edge_bundle_ids.push_back(
        edge_bundles[i]->edge_bundle_id);
    result.value.edges.push_back(
        LoadedRouteEdge{edge_bundles[i]->edge_bundle_id, &edge,
                        from_node_id, to_node_id,
                        oriented_edge_dir(edge, from_node_id, to_node_id)});
  }
  result.ok = true;
  return result;
}

EditResult<std::vector<LoadedRoute>> continuity_routes_from_saved_graph(
    const SavedBackboneGraph& graph) {
  EditResult<std::vector<LoadedRoute>> result{};
  std::vector<ObjectId> remaining{};
  remaining.reserve(graph.edge_bundles.size());
  for (const SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
    remaining.push_back(edge_bundle.edge_bundle_id);
  }
  while (!remaining.empty()) {
    const ObjectId seed = *std::min_element(remaining.begin(), remaining.end());
    EditResult<LoadedRoute> route =
        continuity_route_from_saved_graph(graph, seed);
    if (!route.ok) {
      result.error = route.error;
      return result;
    }
    for (ObjectId edge_bundle_id : route.value.edge_bundle_ids) {
      remaining.erase(
          std::remove(remaining.begin(), remaining.end(), edge_bundle_id),
          remaining.end());
    }
    result.value.push_back(std::move(route.value));
  }
  for (const SavedBackboneEdge& edge : graph.edges) {
    const bool has_bundle = std::any_of(
        graph.edge_bundles.begin(), graph.edge_bundles.end(),
        [&](const SavedBackboneEdgeBundle& edge_bundle) {
          return edge_bundle.edge_id == edge.edge_id;
        });
    if (!has_bundle) {
      result.error = "authoritative invalid input: authoritative deserialization: saved route has no bundles";
      return result;
    }
  }
  std::sort(result.value.begin(), result.value.end(),
            [](const LoadedRoute& a, const LoadedRoute& b) {
              const ObjectId a_id = a.edge_bundle_ids.empty()
                                        ? kInvalidObjectId
                                        : a.edge_bundle_ids.front();
              const ObjectId b_id = b.edge_bundle_ids.empty()
                                        ? kInvalidObjectId
                                        : b.edge_bundle_ids.front();
              return a_id < b_id;
            });
  result.ok = true;
  return result;
}

bool port_is_retired_only(const CoreState& state, ObjectId port_id,
                          const std::vector<ObjectId>& retired_spans,
                          const std::vector<ObjectId>& edge_bundle_ids,
                          std::size_t next_count) {
  const auto spans_it = state.view().connection_index().spans_by_port.find(port_id);
  if (spans_it != state.view().connection_index().spans_by_port.end()) {
    for (ObjectId span_id : spans_it->second) {
      if (!contains_id(retired_spans, span_id)) {
        return false;
      }
    }
  }
  const auto bindings_it = state.view().backbone_index().port_bindings_by_port.find(port_id);
  if (bindings_it == state.view().backbone_index().port_bindings_by_port.end()) {
    return false;
  }
  for (std::size_t binding_index : bindings_it->second) {
    if (binding_index >= state.view().backbone().port_bindings.size()) {
      return false;
    }
    const SavedBackbonePortBinding& binding = state.view().backbone().port_bindings[binding_index];
    if (!contains_id(edge_bundle_ids, binding.edge_bundle_id) || binding.lane_index < next_count) {
      return false;
    }
  }
  return true;
}

} // namespace

EditResult<bool> CoreState::regenerate_backbone_edge_bundles(BundleTemplateId bundle_template_id,
                                                             const BundleTemplate& previous_template,
                                                             const BundleTemplate& next_template,
                                                             ChangeSet* change_set,
                                                             const CableTemplate* cable_template_override,
                                                             const std::vector<ObjectId>* scoped_edge_bundle_ids,
                                                             const PoleTypeDefinition* pole_type_override,
                                                             BackboneRegenerateCause cause,
                                                             const BackboneLaneCountTransition* lane_count_transition) {
  EditResult<bool> result{};
  auto fail = [&](std::string message) {
    result.error = std::move(message);
    return result;
  };

  const int previous_count = lane_count_transition == nullptr
                                 ? previous_template.fixed_count
                                 : lane_count_transition->previous_count;
  const int next_count = lane_count_transition == nullptr
                             ? next_template.fixed_count
                             : lane_count_transition->next_count;
  const bool count_changes = next_count != previous_count;
  const bool bundle_topology_change = cause == BackboneRegenerateCause::kBundleTopology;
  if ((lane_count_transition == nullptr &&
       (previous_template.count_rule != BundleCountRuleKind::kFixed ||
        next_template.count_rule != BundleCountRuleKind::kFixed)) ||
      previous_count <= 0 || next_count <= 0 ||
      (!count_changes && cable_template_override == nullptr && pole_type_override == nullptr &&
       !bundle_topology_change && cause != BackboneRegenerateCause::kSpanOverride &&
       cause != BackboneRegenerateCause::kLayoutSettings)) {
    return fail("backbone unsupported: regenerate requires fixed count, cable decision, or pole type changes");
  }
  if (cable_template_override != nullptr &&
      authoritative_.cable_templates.find(cable_template_override->id) == authoritative_.cable_templates.end()) {
    return fail("backbone regenerate: cable template override missing");
  }
  if (pole_type_override != nullptr &&
      authoritative_.pole_types.find(pole_type_override->id) == authoritative_.pole_types.end()) {
    return fail("backbone regenerate: pole type override missing");
  }
  if (next_template.default_layer == SpanLayer::kUnknown) {
    return fail("backbone unsupported: regenerate requires a known layer");
  }
  if (!std::isfinite(next_template.default_spacing_m) || next_template.default_spacing_m <= 0.0) {
    return fail("backbone unsupported: regenerate requires positive finite spacing");
  }

  std::vector<ObjectId> affected_edge_bundle_ids{};
  if (scoped_edge_bundle_ids != nullptr) {
    affected_edge_bundle_ids = *scoped_edge_bundle_ids;
    for (ObjectId edge_bundle_id : affected_edge_bundle_ids) {
      const SavedBackboneEdgeBundle* edge_bundle = saved_edge_bundle_by_id(view().backbone(), edge_bundle_id);
      const Bundle* bundle = edge_bundle == nullptr ? nullptr : view().bundles().find(edge_bundle->bundle_id);
      if (bundle == nullptr || bundle->bundle_template_id != bundle_template_id) {
        return fail("backbone regenerate: scoped edge bundle does not match bundle template");
      }
    }
  } else {
    for (const SavedBackboneEdgeBundle& edge_bundle : view().backbone().edge_bundles) {
      const Bundle* bundle = view().bundles().find(edge_bundle.bundle_id);
      if (bundle != nullptr && bundle->bundle_template_id == bundle_template_id) {
        affected_edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
      }
    }
  }
  if (affected_edge_bundle_ids.empty()) {
    result.ok = true;
    result.value = true;
    return result;
  }

  const SavedBackboneGraph& graph = view().backbone();
  RegenerateTarget target{};
  target.edge_bundle_id = affected_edge_bundle_ids.front();
  target.edge_bundle_ids = affected_edge_bundle_ids;
  const SavedBackboneEdgeBundle* edge_bundle = saved_edge_bundle_by_id(graph, target.edge_bundle_id);
  if (edge_bundle == nullptr) {
    return fail("backbone regenerate: edge bundle missing");
  }
  target.bundle_id = edge_bundle->bundle_id;
  const Bundle* bundle = view().bundles().find(edge_bundle->bundle_id);
  if (bundle == nullptr ||
      (lane_count_transition == nullptr && bundle->conductor_count != previous_count)) {
    return fail("backbone unsupported: bundle count is not synchronized with previous template");
  }
  const SavedBackboneEdge* edge = saved_edge_by_id(graph, edge_bundle->edge_id);
  if (edge == nullptr) {
    return fail("backbone regenerate: edge missing");
  }
  const EditResult<LoadedRoute> continuity_route =
      continuity_route_from_saved_graph(graph, target.edge_bundle_id);
  if (!continuity_route.ok) {
    return fail(continuity_route.error);
  }
  const std::vector<LoadedRouteEdge>& route_edges =
      continuity_route.value.edges;
  if (route_edges.empty()) {
    return fail("backbone regenerate: row continuity route missing");
  }
  for (ObjectId affected_edge_bundle_id : affected_edge_bundle_ids) {
    const SavedBackboneEdgeBundle* affected_edge_bundle = saved_edge_bundle_by_id(graph, affected_edge_bundle_id);
    const SavedBackboneEdge* affected_edge =
        affected_edge_bundle == nullptr ? nullptr : saved_edge_by_id(graph, affected_edge_bundle->edge_id);
    if (affected_edge_bundle == nullptr || affected_edge == nullptr) {
      return fail("backbone regenerate: scoped edge bundle is incomplete");
    }
    const bool in_route = std::any_of(route_edges.begin(), route_edges.end(), [&](const LoadedRouteEdge& item) {
      return item.edge_bundle_id == affected_edge_bundle_id;
    });
    if (!in_route) {
      return fail("backbone unsupported: regenerate supports one row-continuity component at a time");
    }
    if (affected_edge_bundle->bundle_id != target.bundle_id) {
      return fail("backbone unsupported: regenerate requires one route-local bundle instance");
    }
  }
  target.edge_id = edge->edge_id;
  target.edge = edge;
  std::vector<const SavedBackboneNode*> route_nodes{};
  for (std::size_t edge_index = 0; edge_index < route_edges.size(); ++edge_index) {
    const LoadedRouteEdge& route_edge = route_edges[edge_index];
    if (edge_index == 0) {
      route_nodes.push_back(saved_node_by_id(graph, route_edge.from_node_id));
    } else if (route_edges[edge_index - 1].to_node_id != route_edge.from_node_id) {
      return fail("backbone unsupported: regenerate requires contiguous saved route edges");
    }
    route_nodes.push_back(saved_node_by_id(graph, route_edge.to_node_id));
  }
  for (const SavedBackboneNode* node : route_nodes) {
    if (node == nullptr || (node->pole_id == kInvalidObjectId && !node->has_source_edge)) {
      return fail("backbone unsupported: regenerate supports pole-owned endpoints only");
    }
  }

  for (ObjectId scoped_edge_bundle_id : target.edge_bundle_ids) {
    const auto span_binding_it = runtime_.backbone_index.edge_bundle_span_bindings.find(scoped_edge_bundle_id);
    if (span_binding_it == runtime_.backbone_index.edge_bundle_span_bindings.end()) {
      return fail("backbone regenerate: span binding missing");
    }
    std::vector<bool> seen_lanes(static_cast<std::size_t>(previous_count), false);
    for (std::size_t binding_index : span_binding_it->second) {
      if (binding_index >= graph.span_bindings.size()) {
        return fail("backbone regenerate: span binding index invalid");
      }
      const SavedBackboneSpanBinding& binding = graph.span_bindings[binding_index];
      if (binding.lane_index >= seen_lanes.size()) {
        return fail("backbone unsupported: existing span lane is outside previous bundle count");
      }
      seen_lanes[binding.lane_index] = true;
      if (binding.lane_index >= static_cast<std::size_t>(next_count)) {
        target.retired_spans.push_back(binding.span_id);
        const auto attachment_it = runtime_.relation_index.attachments_by_span.find(binding.span_id);
        if (attachment_it != runtime_.relation_index.attachments_by_span.end()) {
          for (ObjectId attachment_id : attachment_it->second) {
            const Attachment* attachment = view().attachments().find(attachment_id);
            if (attachment == nullptr) {
              return fail("backbone regenerate: retired span attachment missing");
            }
            if (attachment->origin == AttachmentOrigin::kUser) {
              return fail("backbone unsupported: regenerate does not preserve user attachments on retired spans yet");
            }
          }
        }
      }
    }
    if (std::find(seen_lanes.begin(), seen_lanes.end(), false) != seen_lanes.end()) {
      return fail("backbone regenerate: previous bundle lanes are incomplete");
    }
  }

  std::vector<SavedBackboneRowKey> row_keys{};
  for (ObjectId scoped_edge_bundle_id : target.edge_bundle_ids) {
    const auto port_binding_it = runtime_.backbone_index.edge_bundle_ports.find(scoped_edge_bundle_id);
    if (port_binding_it == runtime_.backbone_index.edge_bundle_ports.end()) {
      return fail("backbone regenerate: port binding missing");
    }
    for (std::size_t binding_index : port_binding_it->second) {
      if (binding_index >= graph.port_bindings.size()) {
        return fail("backbone regenerate: port binding index invalid");
      }
      const SavedBackbonePortBinding& binding = graph.port_bindings[binding_index];
      const Port* port = view().ports().find(binding.port_id);
      if (port == nullptr) {
        return fail("backbone regenerate: bound port missing");
      }
      if (std::find(row_keys.begin(), row_keys.end(), binding.row_key) == row_keys.end()) {
        row_keys.push_back(binding.row_key);
      }
      if (binding.lane_index >= static_cast<std::size_t>(next_count)) {
        if (port->position_mode == PortPositionMode::kManual || port->user_edited_position) {
          return fail("backbone unsupported: regenerate cannot retire manual ports");
        }
        if (!contains_id(target.retired_ports, binding.port_id)) {
          target.retired_ports.push_back(binding.port_id);
        }
      }
    }
  }
  if (row_keys.size() < 2) {
    return fail("backbone unsupported: regenerate requires route rows");
  }
  for (ObjectId port_id : target.retired_ports) {
    if (!port_is_retired_only(*this, port_id, target.retired_spans, target.edge_bundle_ids,
                              static_cast<std::size_t>(next_count))) {
      return fail("backbone unsupported: regenerate requires retired ports to be lane-local");
    }
  }

  generation::backbone::graph made_graph{};
  for (std::size_t node_index = 0; node_index < route_nodes.size(); ++node_index) {
    const SavedBackboneNode& saved_node = *route_nodes[node_index];
    generation::backbone::node made_node{};
    made_node.id = static_cast<int>(node_index);
    made_node.pos = saved_node.position;
    made_node.support = saved_node.support_kind;
    made_node.pole = saved_node.pole_id;
    made_node.saved = saved_node.node_id;
    made_node.has_source_edge = saved_node.has_source_edge;
    made_node.source_edge_node_a = saved_node.source_edge_node_a;
    made_node.source_edge_node_b = saved_node.source_edge_node_b;
    made_node.source_edge_t = saved_node.source_edge_t;
    made_node.is_new = false;
    made_node.on_route = true;
    made_node.bundle_modes = saved_node.bundle_modes;
    made_graph.nodes.push_back(std::move(made_node));
  }
  for (std::size_t edge_index = 0; edge_index < route_edges.size(); ++edge_index) {
    const LoadedRouteEdge& route_edge = route_edges[edge_index];
    if (route_edge.edge == nullptr) {
      return fail("backbone regenerate: route edge is missing");
    }
    generation::backbone::link made_link{};
    made_link.id = static_cast<int>(edge_index);
    made_link.a = static_cast<int>(edge_index);
    made_link.b = static_cast<int>(edge_index + 1);
    made_link.route = 0;
    made_link.order = edge_index;
    made_link.dir = route_edge.dir;
    made_link.saved = route_edge.edge->edge_id;
    made_link.is_new = true;
    made_graph.links.push_back(std::move(made_link));
  }

  BackboneSpec spec{};
  spec.pole_type_id = kInvalidPoleTypeId;
  spec.constraints.lateral_offset_m = edge->lateral_offset_m;
  std::vector<std::size_t> active_bundle_indices{};
  const auto template_it =
      authoritative_.bundle_templates.find(bundle->bundle_template_id);
  if (template_it == authoritative_.bundle_templates.end()) {
    return fail("backbone regenerate: scoped bundle template missing");
  }
  BackboneBundleSpec bundle_spec{};
  bundle_spec.bundle_template_id = bundle->bundle_template_id;
  bundle_spec.placement_key = bundle->placement_key;
  bundle_spec.existing_bundle_id = bundle->id;
  if (next_template.count_rule == BundleCountRuleKind::kRange) {
    bundle_spec.count = next_count;
  }
  bundle_spec.layer = bundle->bundle_template_id == bundle_template_id
                          ? next_template.default_layer
                          : template_it->second.default_layer;
  bundle_spec.placement_explicit = bundle->placement_explicit;
  bundle_spec.height_m = bundle->height_m;
  bundle_spec.lateral_m = bundle->lateral_m;
  bundle_spec.spacing_m = bundle->spacing_override_m;
  spec.bundles.push_back(bundle_spec);
  active_bundle_indices.push_back(0);

  CoreState trial = *this;
  trial.authoritative_.bundle_templates[bundle_template_id] = next_template;
  if (cable_template_override != nullptr) {
    trial.authoritative_.cable_templates[cable_template_override->id] = *cable_template_override;
  }
  if (pole_type_override != nullptr) {
    trial.authoritative_.pole_types[pole_type_override->id] = *pole_type_override;
  }
  trial.remove_backbone_row_continuities_for_lanes(target.edge_bundle_ids,
                                                   static_cast<std::size_t>(next_count));
  generation::backbone::pipeline trial_pipeline(trial, spec);
  EditResult<GenerateBundleFromPathResult> replay = trial_pipeline.build(
      trial_pipeline.build_input_from_saved_scope(std::move(made_graph), std::move(active_bundle_indices)));
  if (!replay.ok) {
    return fail(replay.error);
  }

  Bundle* edited_bundle = trial.authoritative_.edit_state.bundles.find(target.bundle_id);
  if (edited_bundle != nullptr) {
    edited_bundle->conductor_count = next_count;
    if (edited_bundle->spacing_override_m == 0.0) {
      edited_bundle->phase_spacing_m = next_template.default_spacing_m;
    }
    if (change_set != nullptr) {
      CoreState::add_unique_id(change_set->updated_ids, edited_bundle->id);
    }
  }
  if (change_set != nullptr) {
    append_unique(change_set->created_ids, replay.change_set.created_ids);
    append_unique(change_set->updated_ids, replay.change_set.updated_ids);
    append_unique(change_set->deleted_ids, replay.change_set.deleted_ids);
  }
  // TODO: pass a scoped span set after regenerate stops rebuilding through the trial-state boundary.
  EditResult<VisualCurvePartCache> visual_curves =
      generation::backbone::make_visual_curve_parts(trial, {});
  if (!visual_curves.ok) {
    result.error = visual_curves.error;
    return result;
  }
  VisualModelInstanceCache model_instances = trial.view().visual_model_instances();
  trial.cache_visual_curve_parts(std::move(visual_curves.value));
  trial.cache_visual_model_instances(std::move(model_instances));

  identity_ = trial.identity_;
  authoritative_ = trial.authoritative_;
  runtime_ = trial.runtime_;
  debug_ = trial.debug_;

  result.ok = true;
  result.value = true;
  return result;
}

EditResult<ObjectId> CoreState::AddBackboneBundleInstance(
    ObjectId anchor_bundle_id, std::uint64_t placement_key,
    double height_m, double lateral_m, double spacing_m) {
  EditResult<ObjectId> result{};
  auto reject = [&](std::string error) {
    result.error = std::move(error);
    result.classify_error();
    return result;
  };
  if (placement_key == 0) {
    return reject(
        "backbone invalid input: added Bundle placement_key must be nonzero");
  }
  if (!std::isfinite(height_m) || !std::isfinite(lateral_m) ||
      !std::isfinite(spacing_m) || spacing_m < 0.0) {
    return reject(
        "backbone invalid input: added Bundle placement values must be finite and spacing >= 0");
  }
  const Bundle* anchor = view().bundles().find(anchor_bundle_id);
  if (anchor == nullptr) {
    return reject("core invalid input: anchor Bundle not found");
  }
  if (anchor->conductor_count <= 0) {
    return reject(
        "backbone invalid input: anchor Bundle conductor count is invalid");
  }
  const auto template_it =
      authoritative_.bundle_templates.find(anchor->bundle_template_id);
  if (template_it == authoritative_.bundle_templates.end()) {
    return reject("backbone invalid input: anchor Bundle template is missing");
  }
  for (const Bundle& bundle : view().bundles().items()) {
    if (bundle.bundle_template_id == anchor->bundle_template_id &&
        bundle.placement_key == placement_key) {
      return reject(
          "backbone invalid input: added Bundle placement_key already exists");
    }
  }

  const SavedBackboneGraph& graph = view().backbone();
  const std::size_t lane_count =
      static_cast<std::size_t>(anchor->conductor_count);
  std::vector<ObjectId> anchor_edge_bundle_ids{};
  std::unordered_set<ObjectId> anchor_edge_bundle_id_set{};
  std::unordered_set<ObjectId> anchor_edge_ids{};
  for (const SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
    if (edge_bundle.bundle_id != anchor_bundle_id) continue;
    if (edge_bundle.edge_bundle_id == kInvalidObjectId ||
        edge_bundle.edge_id == kInvalidObjectId ||
        !anchor_edge_bundle_id_set.insert(edge_bundle.edge_bundle_id).second ||
        !anchor_edge_ids.insert(edge_bundle.edge_id).second) {
      return reject(
          "backbone invalid input: anchor Bundle edge membership is invalid");
    }
    const SavedBackboneEdge* edge =
        saved_edge_by_id(graph, edge_bundle.edge_id);
    const SavedBackboneNode* node_a =
        edge == nullptr ? nullptr : saved_node_by_id(graph, edge->node_a);
    const SavedBackboneNode* node_b =
        edge == nullptr ? nullptr : saved_node_by_id(graph, edge->node_b);
    if (edge == nullptr || node_a == nullptr || node_b == nullptr) {
      return reject(
          "backbone invalid input: anchor Bundle physical membership is incomplete");
    }
    if (node_a->has_source_edge || node_b->has_source_edge) {
      return reject(
          "backbone unsupported: added Bundle source-edge dependency requires exact source Bundle mapping");
    }
    anchor_edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
  }
  if (anchor_edge_bundle_ids.empty()) {
    return reject(
        "backbone unsupported: anchor Bundle has no saved edge membership");
  }

  std::unordered_set<ObjectId> anchor_span_ids{};
  std::unordered_set<ObjectId> anchor_port_ids{};
  for (ObjectId edge_bundle_id : anchor_edge_bundle_ids) {
    std::vector<bool> span_lanes(lane_count, false);
    std::vector<std::size_t> port_lane_counts(lane_count, 0);
    for (const SavedBackboneSpanBinding& binding : graph.span_bindings) {
      if (binding.edge_bundle_id != edge_bundle_id) continue;
      if (binding.lane_index >= lane_count ||
          span_lanes[binding.lane_index]) {
        return reject(
            "backbone invalid input: anchor Bundle SpanBinding lanes are inconsistent");
      }
      const Span* span = view().spans().find(binding.span_id);
      if (span == nullptr || span->bundle_id != anchor_bundle_id) {
        return reject(
            "backbone invalid input: anchor Bundle SpanBinding is incomplete");
      }
      span_lanes[binding.lane_index] = true;
      anchor_span_ids.insert(binding.span_id);
    }
    if (std::find(span_lanes.begin(), span_lanes.end(), false) !=
        span_lanes.end()) {
      return reject(
          "backbone invalid input: anchor Bundle SpanBinding is missing");
    }
    for (const SavedBackbonePortBinding& binding : graph.port_bindings) {
      if (binding.edge_bundle_id != edge_bundle_id) continue;
      if (binding.lane_index >= lane_count ||
          view().ports().find(binding.port_id) == nullptr) {
        return reject(
            "backbone invalid input: anchor Bundle PortBinding is incomplete");
      }
      ++port_lane_counts[binding.lane_index];
      anchor_port_ids.insert(binding.port_id);
    }
    if (std::any_of(port_lane_counts.begin(), port_lane_counts.end(),
                    [](std::size_t count) { return count != 2; })) {
      return reject(
          "backbone invalid input: anchor Bundle PortBinding lanes are incomplete");
    }
  }

  using PhysicalContinuity =
      std::tuple<ObjectId, ObjectId, std::size_t, ObjectId, std::size_t>;
  std::vector<PhysicalContinuity> anchor_continuity{};
  std::vector<generation::backbone::row_continuity_constraint>
      all_constraints{};
  for (const SavedBackboneRowContinuity& continuity :
       graph.row_continuities) {
    const bool a_anchor =
        anchor_edge_bundle_id_set.contains(continuity.a.edge_bundle_id);
    const bool b_anchor =
        anchor_edge_bundle_id_set.contains(continuity.b.edge_bundle_id);
    if (a_anchor != b_anchor) {
      return reject(
          "backbone invalid input: anchor Bundle continuity crosses Bundle identity");
    }
    if (!a_anchor) continue;
    const SavedBackboneEdgeBundle* edge_bundle_a =
        saved_edge_bundle_by_id(graph, continuity.a.edge_bundle_id);
    const SavedBackboneEdgeBundle* edge_bundle_b =
        saved_edge_bundle_by_id(graph, continuity.b.edge_bundle_id);
    if (edge_bundle_a == nullptr || edge_bundle_b == nullptr ||
        continuity.a.lane_index >= lane_count ||
        continuity.b.lane_index >= lane_count) {
      return reject(
          "backbone invalid input: anchor Bundle continuity lane is invalid");
    }
    ObjectId edge_a = edge_bundle_a->edge_id;
    ObjectId edge_b = edge_bundle_b->edge_id;
    std::size_t lane_a = continuity.a.lane_index;
    std::size_t lane_b = continuity.b.lane_index;
    if (std::pair{edge_b, lane_b} < std::pair{edge_a, lane_a}) {
      std::swap(edge_a, edge_b);
      std::swap(lane_a, lane_b);
    }
    anchor_continuity.emplace_back(continuity.node_id, edge_a, lane_a,
                                   edge_b, lane_b);
    all_constraints.push_back({continuity.node_id, edge_a, lane_a, edge_b,
                               lane_b});
  }
  std::sort(anchor_continuity.begin(), anchor_continuity.end());

  for (std::size_t first = 0; first < anchor_continuity.size();) {
    std::size_t last = first + 1;
    while (last < anchor_continuity.size() &&
           std::get<0>(anchor_continuity[last]) ==
               std::get<0>(anchor_continuity[first]) &&
           std::get<1>(anchor_continuity[last]) ==
               std::get<1>(anchor_continuity[first]) &&
           std::get<3>(anchor_continuity[last]) ==
               std::get<3>(anchor_continuity[first])) {
      ++last;
    }
    std::vector<bool> lanes_a(lane_count, false);
    std::vector<bool> lanes_b(lane_count, false);
    for (std::size_t index = first; index < last; ++index) {
      const std::size_t lane_a = std::get<2>(anchor_continuity[index]);
      const std::size_t lane_b = std::get<4>(anchor_continuity[index]);
      if (lanes_a[lane_a] || lanes_b[lane_b]) {
        return reject(
            "backbone invalid input: anchor Bundle continuity lanes are inconsistent");
      }
      lanes_a[lane_a] = true;
      lanes_b[lane_b] = true;
    }
    if (last - first != lane_count ||
        std::find(lanes_a.begin(), lanes_a.end(), false) != lanes_a.end() ||
        std::find(lanes_b.begin(), lanes_b.end(), false) != lanes_b.end()) {
      return reject(
          "backbone invalid input: anchor Bundle continuity lanes are incomplete");
    }
    first = last;
  }

  CoreState trial = *this;
  ObjectId new_bundle_id = kInvalidObjectId;
  ChangeSet change_set{};
  std::vector<ObjectId> remaining = anchor_edge_bundle_ids;
  while (!remaining.empty()) {
    const EditResult<LoadedRoute> loaded =
        continuity_route_from_saved_graph(graph, remaining.front());
    if (!loaded.ok) return reject(loaded.error);
    if (loaded.value.edges.empty()) {
      return reject(
          "backbone invalid input: anchor Bundle continuity component is empty");
    }
    std::unordered_set<ObjectId> component_edge_ids{};
    for (ObjectId edge_bundle_id : loaded.value.edge_bundle_ids) {
      if (!anchor_edge_bundle_id_set.contains(edge_bundle_id)) {
        return reject(
            "backbone invalid input: anchor Bundle continuity component crosses identity");
      }
      remaining.erase(
          std::remove(remaining.begin(), remaining.end(), edge_bundle_id),
          remaining.end());
      const SavedBackboneEdgeBundle* edge_bundle =
          saved_edge_bundle_by_id(graph, edge_bundle_id);
      if (edge_bundle == nullptr) {
        return reject(
            "backbone invalid input: anchor Bundle component edge is missing");
      }
      component_edge_ids.insert(edge_bundle->edge_id);
    }

    generation::backbone::graph made_graph{};
    std::vector<ObjectId> route_node_ids{};
    route_node_ids.push_back(loaded.value.edges.front().from_node_id);
    for (const LoadedRouteEdge& route_edge : loaded.value.edges) {
      if (route_node_ids.back() != route_edge.from_node_id) {
        return reject(
            "backbone invalid input: anchor Bundle component is not contiguous");
      }
      route_node_ids.push_back(route_edge.to_node_id);
    }
    for (std::size_t node_index = 0; node_index < route_node_ids.size();
         ++node_index) {
      const SavedBackboneNode* saved_node =
          saved_node_by_id(graph, route_node_ids[node_index]);
      if (saved_node == nullptr || saved_node->has_source_edge) {
        return reject(
            "backbone unsupported: added Bundle requires source-edge-free membership");
      }
      generation::backbone::node node{};
      node.id = node_index;
      node.pos = saved_node->position;
      node.support = saved_node->support_kind;
      node.pole = saved_node->pole_id;
      node.saved = saved_node->node_id;
      node.is_new = false;
      node.on_route = true;
      node.bundle_modes = saved_node->bundle_modes;
      made_graph.nodes.push_back(std::move(node));
    }
    for (std::size_t edge_index = 0;
         edge_index < loaded.value.edges.size(); ++edge_index) {
      const LoadedRouteEdge& route_edge = loaded.value.edges[edge_index];
      generation::backbone::link link{};
      link.id = edge_index;
      link.a = edge_index;
      link.b = edge_index + 1;
      link.route = 0;
      link.order = edge_index;
      link.dir = route_edge.dir;
      link.saved = route_edge.edge->edge_id;
      link.is_new = true;
      made_graph.links.push_back(std::move(link));
    }

    std::vector<generation::backbone::row_continuity_constraint>
        component_constraints{};
    for (const auto& constraint : all_constraints) {
      if (component_edge_ids.contains(constraint.edge_a) &&
          component_edge_ids.contains(constraint.edge_b)) {
        component_constraints.push_back(constraint);
      }
    }

    BackboneSpec spec{};
    spec.pole_type_id = kInvalidPoleTypeId;
    spec.constraints.lateral_offset_m =
        loaded.value.edges.front().edge->lateral_offset_m;
    BackboneBundleSpec bundle_spec{};
    bundle_spec.bundle_template_id = anchor->bundle_template_id;
    bundle_spec.placement_key = placement_key;
    bundle_spec.layer = template_it->second.default_layer;
    bundle_spec.placement_explicit = true;
    bundle_spec.height_m = height_m;
    bundle_spec.lateral_m = lateral_m;
    bundle_spec.spacing_m = spacing_m;
    bundle_spec.existing_bundle_id = new_bundle_id;
    if (template_it->second.count_rule == BundleCountRuleKind::kRange) {
      bundle_spec.count = anchor->conductor_count;
    }
    spec.bundles.push_back(bundle_spec);

    generation::backbone::pipeline pipeline(trial, spec);
    EditResult<GenerateBundleFromPathResult> replay = pipeline.build(
        pipeline.build_input_from_saved_scope(
            std::move(made_graph), {0}, false, true,
            std::move(component_constraints)));
    if (!replay.ok) return reject(replay.error);
    if (replay.value.bundle_ids.size() != 1 ||
        replay.value.bundle_ids.front() == kInvalidObjectId ||
        (new_bundle_id != kInvalidObjectId &&
         replay.value.bundle_ids.front() != new_bundle_id)) {
      return reject(
          "backbone internal: added Bundle identity is inconsistent across components");
    }
    new_bundle_id = replay.value.bundle_ids.front();
    append_unique(change_set.created_ids, replay.change_set.created_ids);
    append_unique(change_set.updated_ids, replay.change_set.updated_ids);
    append_unique(change_set.deleted_ids, replay.change_set.deleted_ids);
  }

  const Bundle* added = trial.view().bundles().find(new_bundle_id);
  if (added == nullptr || new_bundle_id == anchor_bundle_id ||
      added->bundle_template_id != anchor->bundle_template_id ||
      added->conductor_count != anchor->conductor_count ||
      added->placement_key != placement_key || !added->placement_explicit ||
      added->height_m != height_m || added->lateral_m != lateral_m ||
      added->spacing_override_m != spacing_m) {
    return reject(
        "backbone internal: added Bundle identity or placement is invalid");
  }

  std::unordered_map<ObjectId, ObjectId> new_edge_bundle_by_edge{};
  std::unordered_set<ObjectId> new_edge_bundle_ids{};
  for (const SavedBackboneEdgeBundle& edge_bundle :
       trial.view().backbone().edge_bundles) {
    if (edge_bundle.bundle_id != new_bundle_id) continue;
    if (!anchor_edge_ids.contains(edge_bundle.edge_id) ||
        !new_edge_bundle_by_edge.emplace(edge_bundle.edge_id,
                                         edge_bundle.edge_bundle_id)
             .second) {
      return reject(
          "backbone internal: added Bundle physical membership is not isomorphic");
    }
    new_edge_bundle_ids.insert(edge_bundle.edge_bundle_id);
  }
  if (new_edge_bundle_by_edge.size() != anchor_edge_ids.size()) {
    return reject(
        "backbone internal: added Bundle physical membership is incomplete");
  }

  std::vector<PhysicalContinuity> added_continuity{};
  for (const SavedBackboneRowContinuity& continuity :
       trial.view().backbone().row_continuities) {
    const bool a_added =
        new_edge_bundle_ids.contains(continuity.a.edge_bundle_id);
    const bool b_added =
        new_edge_bundle_ids.contains(continuity.b.edge_bundle_id);
    if (a_added != b_added) {
      return reject(
          "backbone internal: added Bundle continuity crosses identity");
    }
    if (!a_added) continue;
    const SavedBackboneEdgeBundle* edge_bundle_a =
        trial.view().backbone_edge_bundle(continuity.a.edge_bundle_id);
    const SavedBackboneEdgeBundle* edge_bundle_b =
        trial.view().backbone_edge_bundle(continuity.b.edge_bundle_id);
    ObjectId edge_a = edge_bundle_a->edge_id;
    ObjectId edge_b = edge_bundle_b->edge_id;
    std::size_t lane_a = continuity.a.lane_index;
    std::size_t lane_b = continuity.b.lane_index;
    if (std::pair{edge_b, lane_b} < std::pair{edge_a, lane_a}) {
      std::swap(edge_a, edge_b);
      std::swap(lane_a, lane_b);
    }
    added_continuity.emplace_back(continuity.node_id, edge_a, lane_a,
                                  edge_b, lane_b);
  }
  std::sort(added_continuity.begin(), added_continuity.end());
  if (added_continuity != anchor_continuity) {
    return reject(
        "backbone internal: added Bundle row continuity is not isomorphic");
  }

  std::unordered_set<ObjectId> added_span_ids{};
  std::unordered_set<ObjectId> added_port_ids{};
  for (ObjectId edge_bundle_id : new_edge_bundle_ids) {
    std::vector<bool> span_lanes(lane_count, false);
    std::vector<std::size_t> port_lane_counts(lane_count, 0);
    for (const SavedBackboneSpanBinding& binding :
         trial.view().backbone().span_bindings) {
      if (binding.edge_bundle_id != edge_bundle_id) continue;
      if (binding.lane_index >= lane_count ||
          span_lanes[binding.lane_index] ||
          anchor_span_ids.contains(binding.span_id)) {
        return reject(
            "backbone internal: added Bundle Span identity is invalid");
      }
      span_lanes[binding.lane_index] = true;
      added_span_ids.insert(binding.span_id);
    }
    for (const SavedBackbonePortBinding& binding :
         trial.view().backbone().port_bindings) {
      if (binding.edge_bundle_id != edge_bundle_id) continue;
      if (binding.lane_index >= lane_count ||
          anchor_port_ids.contains(binding.port_id)) {
        return reject(
            "backbone internal: added Bundle Port identity is invalid");
      }
      ++port_lane_counts[binding.lane_index];
      added_port_ids.insert(binding.port_id);
    }
    if (std::find(span_lanes.begin(), span_lanes.end(), false) !=
            span_lanes.end() ||
        std::any_of(port_lane_counts.begin(), port_lane_counts.end(),
                    [](std::size_t count) { return count != 2; })) {
      return reject(
          "backbone internal: added Bundle binding topology is incomplete");
    }
  }

  EditResult<bool> rebuilt = trial.rebuild_loaded_outputs();
  if (!rebuilt.ok) return reject(rebuilt.error);
  const ValidationResult validation = trial.Validate();
  for (const ValidationIssue& issue : validation.issues) {
    if (issue.severity == ValidationSeverity::kError) {
      return reject(
          "backbone invalid input: added Bundle validation failed: " +
          issue.code + ": " + issue.message);
    }
  }

  identity_ = trial.identity_;
  authoritative_ = trial.authoritative_;
  runtime_ = trial.runtime_;
  debug_ = trial.debug_;
  result.ok = true;
  result.value = new_bundle_id;
  result.change_set = std::move(change_set);
  return result;
}

EditResult<bool> CoreState::ReconcileBackboneBundleInstances(
    const BackboneBundleReconcileInput& input) {
  EditResult<bool> result{};
  auto reject = [&](std::string error) {
    result.error = std::move(error);
    result.classify_error();
    return result;
  };

  std::unordered_set<ObjectId> current_ids{};
  std::map<std::uint64_t, ObjectId> current_by_key{};
  std::unordered_map<ObjectId, std::uint64_t> current_key_by_id{};
  for (ObjectId bundle_id : input.current_bundle_ids) {
    if (bundle_id == kInvalidObjectId || !current_ids.insert(bundle_id).second) {
      return reject(
          "backbone invalid input: reconcile current Bundle IDs must be exact and unique");
    }
    const Bundle* bundle = view().bundles().find(bundle_id);
    if (bundle == nullptr || bundle->placement_key == 0) {
      return reject(
          "backbone invalid input: reconcile current Bundle requires a nonzero placement_key");
    }
    if (!current_by_key.emplace(bundle->placement_key, bundle_id).second) {
      return reject(
          "backbone invalid input: reconcile current placement_keys must be unique");
    }
    current_key_by_id.emplace(bundle_id, bundle->placement_key);
  }

  std::map<std::uint64_t, const BackboneBundleReconcileEntry*> desired_by_key{};
  std::map<std::uint64_t, int> desired_count_by_key{};
  for (const BackboneBundleReconcileEntry& entry : input.desired_bundles) {
    const BackboneBundleSpec& desired = entry.desired;
    if (desired.placement_key == 0 ||
        !desired_by_key.emplace(desired.placement_key, &entry).second) {
      return reject(
          "backbone invalid input: reconcile desired placement_keys must be nonzero and unique");
    }
    if (!desired.placement_explicit || !std::isfinite(desired.height_m) ||
        !std::isfinite(desired.lateral_m) ||
        !std::isfinite(desired.spacing_m) || desired.spacing_m <= 0.0) {
      return reject(
          "backbone invalid input: reconcile desired placement must be explicit, finite, and positive-spaced");
    }
    if (desired.source_bundle_id != kInvalidObjectId ||
        desired.existing_bundle_id != kInvalidObjectId) {
      return reject(
          "backbone invalid input: reconcile desired spec cannot carry source or existing Bundle identity");
    }
    const auto template_it =
        authoritative_.bundle_templates.find(desired.bundle_template_id);
    if (template_it == authoritative_.bundle_templates.end() ||
        desired.layer != template_it->second.default_layer) {
      return reject(
          "backbone invalid input: reconcile desired template or layer is invalid");
    }
    int desired_count = 0;
    if (template_it->second.count_rule == BundleCountRuleKind::kFixed) {
      if (desired.count != 0 || template_it->second.fixed_count <= 0) {
        return reject(
            "backbone invalid input: reconcile fixed template rejects desired count input");
      }
      desired_count = template_it->second.fixed_count;
    } else {
      if (desired.count < template_it->second.min_count ||
          desired.count > template_it->second.max_count) {
        return reject(
            "backbone invalid input: reconcile desired count is outside template range");
      }
      desired_count = desired.count;
    }
    desired_count_by_key.emplace(desired.placement_key, desired_count);
  }

  // A desired key must not silently capture an existing Bundle outside the
  // caller-declared exact current scope.
  for (const Bundle& bundle : view().bundles().items()) {
    if (current_ids.contains(bundle.id) || bundle.placement_key == 0 ||
        !desired_by_key.contains(bundle.placement_key)) {
      continue;
    }
    return reject(
        "backbone invalid input: reconcile desired placement_key collides outside current scope");
  }

  for (const auto& [placement_key, entry] : desired_by_key) {
    const auto current_it = current_by_key.find(placement_key);
    if (current_it != current_by_key.end()) {
      if (entry->anchor_bundle_id != kInvalidObjectId) {
        return reject(
            "backbone invalid input: reconcile survivor must not specify an add anchor");
      }
      const Bundle* current = view().bundles().find(current_it->second);
      if (current == nullptr ||
          current->bundle_template_id != entry->desired.bundle_template_id) {
        return reject(
            "backbone unsupported: reconcile cannot migrate Bundle template identity");
      }
      continue;
    }

    if (!current_ids.contains(entry->anchor_bundle_id)) {
      return reject(
          "backbone unsupported: reconcile add requires an exact anchor from current scope");
    }
    const Bundle* anchor = view().bundles().find(entry->anchor_bundle_id);
    if (anchor == nullptr ||
        anchor->bundle_template_id != entry->desired.bundle_template_id) {
      return reject(
          "backbone unsupported: reconcile add anchor template is incompatible");
    }
    int anchor_final_count = anchor->conductor_count;
    const auto anchor_key_it = current_key_by_id.find(entry->anchor_bundle_id);
    if (anchor_key_it != current_key_by_id.end()) {
      const auto anchor_desired_count =
          desired_count_by_key.find(anchor_key_it->second);
      if (anchor_desired_count != desired_count_by_key.end()) {
        anchor_final_count = anchor_desired_count->second;
      }
    }
    if (anchor_final_count != desired_count_by_key.at(placement_key)) {
      return reject(
          "backbone unsupported: reconcile add anchor lane topology is incompatible");
    }
  }

  CoreState trial = *this;
  ChangeSet changes{};
  bool changed = false;
  auto merge_changes = [&](const ChangeSet& source) {
    append_unique(changes.created_ids, source.created_ids);
    append_unique(changes.updated_ids, source.updated_ids);
    append_unique(changes.deleted_ids, source.deleted_ids);
  };

  // Survivors are updated first so an exact anchor that survives with a new
  // permitted lane count has its final lane topology before instance cloning.
  for (const auto& [placement_key, entry] : desired_by_key) {
    const auto current_it = current_by_key.find(placement_key);
    if (current_it == current_by_key.end()) continue;
    const ObjectId bundle_id = current_it->second;
    EditResult<bool> count = trial.UpdateBackboneBundleConductorCount(
        bundle_id, desired_count_by_key.at(placement_key));
    if (!count.ok) return reject(count.error);
    changed = changed || count.value;
    merge_changes(count.change_set);
    EditResult<bool> placement = trial.UpdateBackboneBundlePlacement(
        bundle_id, true, entry->desired.height_m, entry->desired.lateral_m,
        entry->desired.spacing_m);
    if (!placement.ok) return reject(placement.error);
    changed = changed || placement.value;
    merge_changes(placement.change_set);
  }

  std::map<std::uint64_t, ObjectId> added_by_key{};
  for (const auto& [placement_key, entry] : desired_by_key) {
    if (current_by_key.contains(placement_key)) continue;
    EditResult<ObjectId> added = trial.AddBackboneBundleInstance(
        entry->anchor_bundle_id, placement_key, entry->desired.height_m,
        entry->desired.lateral_m, entry->desired.spacing_m);
    if (!added.ok) return reject(added.error);
    added_by_key.emplace(placement_key, added.value);
    changed = true;
    merge_changes(added.change_set);
  }

  // Retire only after all additions, because a caller may intentionally use a
  // current-only Bundle as the exact membership oracle for its replacement.
  for (const auto& [placement_key, bundle_id] : current_by_key) {
    if (desired_by_key.contains(placement_key)) continue;
    EditResult<bool> retired = trial.RetireBackboneBundle(bundle_id);
    if (!retired.ok) return reject(retired.error);
    changed = changed || retired.value;
    merge_changes(retired.change_set);
  }

  if (!changed) {
    result.ok = true;
    result.value = false;
    return result;
  }

  for (const auto& [placement_key, entry] : desired_by_key) {
    const Bundle* matched = nullptr;
    for (const Bundle& bundle : trial.view().bundles().items()) {
      if (bundle.placement_key != placement_key) continue;
      if (matched != nullptr) {
        return reject(
            "backbone internal: reconcile final placement identity is ambiguous");
      }
      matched = &bundle;
    }
    const ObjectId expected_id = current_by_key.contains(placement_key)
                                     ? current_by_key.at(placement_key)
                                     : added_by_key.at(placement_key);
    if (matched == nullptr || matched->id != expected_id ||
        matched->bundle_template_id != entry->desired.bundle_template_id ||
        matched->conductor_count != desired_count_by_key.at(placement_key) ||
        !matched->placement_explicit ||
        matched->height_m != entry->desired.height_m ||
        matched->lateral_m != entry->desired.lateral_m ||
        matched->spacing_override_m != entry->desired.spacing_m) {
      return reject(
          "backbone internal: reconcile final concrete Bundle does not match desired spec");
    }
    const bool has_edge_membership = std::any_of(
        trial.view().backbone().edge_bundles.begin(),
        trial.view().backbone().edge_bundles.end(),
        [&](const SavedBackboneEdgeBundle& edge_bundle) {
          return edge_bundle.bundle_id == matched->id;
        });
    if (!has_edge_membership) {
      return reject(
          "backbone internal: reconcile final Bundle membership is missing");
    }
  }
  for (const auto& [placement_key, bundle_id] : current_by_key) {
    if (!desired_by_key.contains(placement_key) &&
        trial.view().bundles().find(bundle_id) != nullptr) {
      return reject(
          "backbone internal: reconcile retired Bundle survived final scope");
    }
  }

  EditResult<bool> rebuilt = trial.rebuild_loaded_outputs();
  if (!rebuilt.ok) return reject(rebuilt.error);
  const ValidationResult validation = trial.Validate();
  for (const ValidationIssue& issue : validation.issues) {
    if (issue.severity == ValidationSeverity::kError) {
      return reject(
          "backbone invalid input: reconcile validation failed: " +
          issue.code + ": " + issue.message);
    }
  }

  identity_ = trial.identity_;
  authoritative_ = trial.authoritative_;
  runtime_ = trial.runtime_;
  debug_ = trial.debug_;
  result.ok = true;
  result.value = true;
  result.change_set = std::move(changes);
  return result;
}

EditResult<bool> CoreState::RetireBackboneBundle(ObjectId bundle_id) {
  EditResult<bool> result{};
  auto reject = [&](std::string error) {
    result.error = std::move(error);
    result.classify_error();
    return result;
  };

  const Bundle* bundle = view().bundles().find(bundle_id);
  if (bundle == nullptr) {
    return reject("core invalid input: bundle not found");
  }
  const SavedBackboneGraph& graph = view().backbone();
  std::vector<ObjectId> edge_bundle_ids{};
  std::unordered_set<ObjectId> edge_bundle_id_set{};
  if (bundle->conductor_count <= 0) {
    return reject("backbone invalid input: exact Bundle retirement conductor count is invalid");
  }
  for (const SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
    if (edge_bundle.bundle_id != bundle_id) continue;
    if (edge_bundle.edge_bundle_id == kInvalidObjectId ||
        edge_bundle.edge_id == kInvalidObjectId ||
        !edge_bundle_id_set.insert(edge_bundle.edge_bundle_id).second) {
      return reject("backbone invalid input: exact Bundle retirement edge scope is invalid");
    }
    edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
  }
  if (edge_bundle_ids.empty()) {
    return reject("backbone unsupported: exact Bundle retirement has no saved edge bundles");
  }

  std::unordered_set<ObjectId> retired_span_ids{};
  std::unordered_set<ObjectId> retired_port_ids{};
  for (ObjectId edge_bundle_id : edge_bundle_ids) {
    std::vector<bool> span_lanes(static_cast<std::size_t>(bundle->conductor_count), false);
    std::vector<std::size_t> port_lane_counts(
        static_cast<std::size_t>(bundle->conductor_count), 0);
    for (const SavedBackboneSpanBinding& binding : graph.span_bindings) {
      if (binding.edge_bundle_id != edge_bundle_id) continue;
      if (binding.lane_index >= span_lanes.size() || span_lanes[binding.lane_index]) {
        return reject("backbone invalid input: exact Bundle retirement span lanes are inconsistent");
      }
      const Span* span = view().spans().find(binding.span_id);
      if (span == nullptr || span->bundle_id != bundle_id) {
        return reject("backbone invalid input: exact Bundle retirement span binding is incomplete");
      }
      span_lanes[binding.lane_index] = true;
      retired_span_ids.insert(binding.span_id);
    }
    if (span_lanes.empty() ||
        std::find(span_lanes.begin(), span_lanes.end(), false) != span_lanes.end()) {
      return reject("backbone invalid input: exact Bundle retirement span binding is missing");
    }
    for (const SavedBackbonePortBinding& binding : graph.port_bindings) {
      if (binding.edge_bundle_id != edge_bundle_id) continue;
      if (binding.lane_index >= port_lane_counts.size()) {
        return reject("backbone invalid input: exact Bundle retirement port lane is inconsistent");
      }
      const Port* port = view().ports().find(binding.port_id);
      if (port == nullptr) {
        return reject("backbone invalid input: exact Bundle retirement bound Port is missing");
      }
      if (port->position_mode == PortPositionMode::kManual ||
          port->user_edited_position) {
        return reject("backbone unsupported: exact Bundle retirement cannot retire manual Ports");
      }
      ++port_lane_counts[binding.lane_index];
      retired_port_ids.insert(binding.port_id);
    }
    if (port_lane_counts.empty() ||
        std::any_of(port_lane_counts.begin(), port_lane_counts.end(),
                    [](std::size_t count) { return count != 2; })) {
      return reject("backbone invalid input: exact Bundle retirement port bindings are incomplete");
    }
  }

  for (ObjectId span_id : retired_span_ids) {
    if (authoritative_.override_state.span_endpoint_by_span.contains(span_id) ||
        authoritative_.override_state.span_support_by_span.contains(span_id)) {
      return reject("backbone unsupported: exact Bundle retirement cannot discard Span overrides");
    }
    for (const Attachment& attachment : view().attachments().items()) {
      if (attachment.span_id == span_id &&
          attachment.origin == AttachmentOrigin::kUser) {
        return reject("backbone unsupported: exact Bundle retirement cannot discard user Attachments");
      }
    }
  }
  for (ObjectId port_id : retired_port_ids) {
    const auto spans_it = runtime_.connection_index.spans_by_port.find(port_id);
    if (spans_it == runtime_.connection_index.spans_by_port.end()) continue;
    for (ObjectId span_id : spans_it->second) {
      if (!retired_span_ids.contains(span_id)) {
        return reject("backbone unsupported: exact Bundle retirement Port is used by surviving topology");
      }
    }
  }
  for (const SavedBackboneRowContinuity& continuity : graph.row_continuities) {
    const bool a_retired = edge_bundle_id_set.contains(continuity.a.edge_bundle_id);
    const bool b_retired = edge_bundle_id_set.contains(continuity.b.edge_bundle_id);
    if (a_retired != b_retired) {
      return reject("backbone unsupported: exact Bundle retirement continuity crosses Bundle identity");
    }
  }

  CoreState trial = *this;
  BackboneSpec empty_spec{};
  generation::backbone::pipeline retirement_pipeline(trial, empty_spec);
  EditResult<GenerateBundleFromPathResult> retired = retirement_pipeline.build(
      retirement_pipeline.build_input_for_bundle_retirement(
          bundle_id, edge_bundle_ids));
  if (!retired.ok) return reject(retired.error);
  if (trial.view().bundles().find(bundle_id) != nullptr ||
      std::any_of(trial.view().spans().items().begin(),
                  trial.view().spans().items().end(),
                  [&](const Span& span) { return span.bundle_id == bundle_id; }) ||
      std::any_of(trial.view().backbone().edge_bundles.begin(),
                  trial.view().backbone().edge_bundles.end(),
                  [&](const SavedBackboneEdgeBundle& edge_bundle) {
                    return edge_bundle.bundle_id == bundle_id ||
                           edge_bundle_id_set.contains(edge_bundle.edge_bundle_id);
                  }) ||
      std::any_of(trial.view().backbone().span_bindings.begin(),
                  trial.view().backbone().span_bindings.end(),
                  [&](const SavedBackboneSpanBinding& binding) {
                    return edge_bundle_id_set.contains(binding.edge_bundle_id) ||
                           retired_span_ids.contains(binding.span_id);
                  }) ||
      std::any_of(trial.view().backbone().port_bindings.begin(),
                  trial.view().backbone().port_bindings.end(),
                  [&](const SavedBackbonePortBinding& binding) {
                    return edge_bundle_id_set.contains(binding.edge_bundle_id);
                  }) ||
      std::any_of(trial.view().backbone().row_continuities.begin(),
                  trial.view().backbone().row_continuities.end(),
                  [&](const SavedBackboneRowContinuity& continuity) {
                    return edge_bundle_id_set.contains(continuity.a.edge_bundle_id) ||
                           edge_bundle_id_set.contains(continuity.b.edge_bundle_id);
                  }) ||
      std::any_of(trial.view().attachments().items().begin(),
                  trial.view().attachments().items().end(),
                  [&](const Attachment& attachment) {
                    return retired_span_ids.contains(attachment.span_id);
                  })) {
    return reject("backbone internal: exact Bundle retirement cleanup is incomplete");
  }

  EditResult<bool> rebuilt = trial.rebuild_loaded_outputs();
  if (!rebuilt.ok) return reject(rebuilt.error);
  const ValidationResult validation = trial.Validate();
  for (const ValidationIssue& issue : validation.issues) {
    if (issue.severity == ValidationSeverity::kError) {
      return reject("backbone invalid input: exact Bundle retirement validation failed: " +
                    issue.code + ": " + issue.message);
    }
  }

  identity_ = trial.identity_;
  authoritative_ = trial.authoritative_;
  runtime_ = trial.runtime_;
  debug_ = trial.debug_;
  result.ok = true;
  result.value = true;
  result.change_set = std::move(retired.change_set);
  return result;
}

EditResult<bool> CoreState::regenerate_backbone_span_override(ObjectId span_id, ChangeSet* change_set) {
  EditResult<bool> result{};
  const auto backbone_it = runtime_.backbone_index.span_edge_bundle.find(span_id);
  if (backbone_it == runtime_.backbone_index.span_edge_bundle.end()) {
    result.error = "backbone unsupported: backbone regenerate: span is not bound to an edge bundle";
    return result;
  }
  const SavedBackboneEdgeBundle* edge_bundle = view().backbone_edge_bundle(backbone_it->second);
  const Bundle* bundle = edge_bundle == nullptr ? nullptr : view().bundles().find(edge_bundle->bundle_id);
  if (edge_bundle == nullptr || bundle == nullptr) {
    result.error = "backbone unsupported: backbone regenerate: span override scope is incomplete";
    return result;
  }
  const auto bundle_template_it = authoritative_.bundle_templates.find(bundle->bundle_template_id);
  if (bundle_template_it == authoritative_.bundle_templates.end()) {
    result.error = "core invalid input: bundle template not found";
    return result;
  }
  const std::vector<ObjectId> scope{edge_bundle->edge_bundle_id};
  return regenerate_backbone_edge_bundles(bundle->bundle_template_id, bundle_template_it->second,
                                          bundle_template_it->second, change_set, nullptr, &scope, nullptr,
                                          BackboneRegenerateCause::kSpanOverride);
}

EditResult<bool> CoreState::rebuild_loaded_outputs() {
  EditResult<bool> result{};
  const SavedBackboneGraph saved = view().backbone();
  const CoreState loaded_state = *this;
  CoreState assembled = loaded_state;
  assembled.runtime_.cache_state = {};
  EditResult<std::vector<LoadedRoute>> saved_routes =
      continuity_routes_from_saved_graph(saved);
  if (!saved_routes.ok) {
    result.error = saved_routes.error;
    return result;
  }

  for (const LoadedRoute& saved_route : saved_routes.value) {
    const std::vector<LoadedRouteEdge>& edges = saved_route.edges;
    if (edges.empty()) {
      continue;
    }

    generation::backbone::graph graph{};
    for (std::size_t i = 0; i <= edges.size(); ++i) {
      const ObjectId node_id = i == 0 ? edges.front().from_node_id : edges[i - 1].to_node_id;
      if (i > 0 && i < edges.size() && edges[i].from_node_id != node_id) {
        result.error = "authoritative invalid input: authoritative deserialization: saved route is not contiguous";
        return result;
      }
      const SavedBackboneNode* saved_node = saved_node_by_id(saved, node_id);
      if (saved_node == nullptr) {
        result.error = "authoritative invalid input: authoritative deserialization: saved route node is missing";
        return result;
      }
      generation::backbone::node node{};
      node.id = static_cast<int>(i);
      node.pos = saved_node->position;
      node.support = saved_node->support_kind;
      node.pole = saved_node->pole_id;
      node.saved = saved_node->node_id;
      node.has_source_edge = saved_node->has_source_edge;
      node.source_edge_node_a = saved_node->source_edge_node_a;
      node.source_edge_node_b = saved_node->source_edge_node_b;
      node.source_edge_t = saved_node->source_edge_t;
      node.is_new = false;
      node.on_route = true;
      node.bundle_modes = saved_node->bundle_modes;
      graph.nodes.push_back(std::move(node));
    }
    for (std::size_t i = 0; i < edges.size(); ++i) {
      generation::backbone::link link{};
      link.id = static_cast<int>(i);
      link.a = static_cast<int>(i);
      link.b = static_cast<int>(i + 1);
      link.route = 0;
      link.order = i;
      link.dir = edges[i].dir;
      link.saved = edges[i].edge->edge_id;
      link.is_new = true;
      graph.links.push_back(std::move(link));
    }

    std::unordered_map<ObjectId, int> node_index{};
    for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
      node_index.emplace(graph.nodes[i].saved, static_cast<int>(i));
    }
    auto append_context_node = [&](ObjectId node_id) -> int {
      if (const auto found = node_index.find(node_id); found != node_index.end()) return found->second;
      const SavedBackboneNode* saved_node = saved_node_by_id(saved, node_id);
      if (saved_node == nullptr) return -1;
      generation::backbone::node node{};
      node.id = static_cast<int>(graph.nodes.size());
      node.pos = saved_node->position;
      node.support = saved_node->support_kind;
      node.pole = saved_node->pole_id;
      node.saved = saved_node->node_id;
      node.has_source_edge = saved_node->has_source_edge;
      node.source_edge_node_a = saved_node->source_edge_node_a;
      node.source_edge_node_b = saved_node->source_edge_node_b;
      node.source_edge_t = saved_node->source_edge_t;
      node.is_new = false;
      node.on_route = false;
      node.bundle_modes = saved_node->bundle_modes;
      graph.nodes.push_back(std::move(node));
      node_index.emplace(node_id, static_cast<int>(graph.nodes.back().id));
      return graph.nodes.back().id;
    };
    std::unordered_set<ObjectId> route_edge_ids{};
    for (const LoadedRouteEdge& route_edge : edges) route_edge_ids.insert(route_edge.edge->edge_id);
    const std::size_t context_route_offset = saved_routes.value.size() + saved.edges.size() + 1;
    for (const SavedBackboneEdge& candidate : saved.edges) {
      if (route_edge_ids.contains(candidate.edge_id) ||
          (!node_index.contains(candidate.node_a) && !node_index.contains(candidate.node_b))) continue;
      const int a = append_context_node(candidate.node_a);
      const int b = append_context_node(candidate.node_b);
      if (a < 0 || b < 0) {
        result.error = "authoritative invalid input: authoritative deserialization: saved context node is missing";
        return result;
      }
      generation::backbone::link link{};
      link.id = static_cast<int>(graph.links.size());
      link.a = a;
      link.b = b;
      link.route = context_route_offset + static_cast<std::size_t>(link.id);
      link.order = static_cast<std::size_t>(link.id);
      link.dir = candidate.dir;
      link.saved = candidate.edge_id;
      link.is_new = false;
      graph.links.push_back(std::move(link));
    }

    BackboneSpec spec{};
    spec.pole_type_id = kInvalidPoleTypeId;
    spec.constraints.lateral_offset_m = edges.front().edge->lateral_offset_m;
    std::vector<std::size_t> active_bundle_indices{};
    const SavedBackboneEdgeBundle* first_edge_bundle =
        saved_edge_bundle_by_id(saved, edges.front().edge_bundle_id);
    const Bundle* bundle = first_edge_bundle == nullptr
                               ? nullptr
                               : authoritative_.edit_state.bundles.find(
                                     first_edge_bundle->bundle_id);
    if (bundle == nullptr) {
      result.error = "authoritative invalid input: authoritative deserialization: saved edge bundle is missing its bundle";
      return result;
    }
    const auto template_it =
        authoritative_.bundle_templates.find(bundle->bundle_template_id);
    if (template_it == authoritative_.bundle_templates.end()) {
      result.error = "authoritative invalid input: authoritative deserialization: saved bundle template is missing";
      return result;
    }
    BackboneBundleSpec bundle_spec{};
    bundle_spec.bundle_template_id = bundle->bundle_template_id;
    bundle_spec.placement_key = bundle->placement_key;
    bundle_spec.existing_bundle_id = bundle->id;
    bundle_spec.layer = template_it->second.default_layer;
    bundle_spec.placement_explicit = bundle->placement_explicit;
    bundle_spec.height_m = bundle->height_m;
    bundle_spec.lateral_m = bundle->lateral_m;
    bundle_spec.spacing_m = bundle->spacing_override_m;
    if (template_it->second.count_rule == BundleCountRuleKind::kRange) {
      bundle_spec.count = bundle->conductor_count;
    }
    spec.bundles.push_back(bundle_spec);
    active_bundle_indices.push_back(0);
    CoreState route_trial = loaded_state;
    route_trial.runtime_.cache_state = assembled.runtime_.cache_state;
    generation::backbone::pipeline pipeline(route_trial, spec);
    const auto replay = pipeline.build(
        pipeline.build_input_from_saved_scope(std::move(graph), std::move(active_bundle_indices), false, false));
    if (!replay.ok) {
      result.error =
          "backbone load route " +
          std::to_string(edges.front().edge == nullptr
                             ? kInvalidObjectId
                             : edges.front().edge->edge_id) +
          ": " + replay.error;
      return result;
    }

    for (const SavedBackboneSpanBinding& binding : saved.span_bindings) {
      if (!contains_id(saved_route.edge_bundle_ids,
                       binding.edge_bundle_id)) {
        continue;
      }
      EditResult<bool> merged =
          assembled.merge_cached_span_outputs_from(route_trial, binding.span_id);
      if (!merged.ok) {
        result.error = merged.error;
        return result;
      }
    }
    assembled.merge_cached_support_groups_from(route_trial);
  }

  runtime_.cache_state = std::move(assembled.runtime_.cache_state);
  EditResult<VisualCurvePartCache> visual_curves =
      generation::backbone::make_visual_curve_parts(*this, {});
  if (!visual_curves.ok) {
    result.error = visual_curves.error;
    return result;
  }
  EditResult<generation::backbone::FixturePlacementPlanByPort> fixture_plan =
      generation::backbone::fixture_placement_plan_from_cache(*this);
  if (!fixture_plan.ok) {
    result.error = fixture_plan.error;
    return result;
  }
  EditResult<VisualModelInstanceCache> model_instances =
      generation::backbone::materialize_model_assemblies(
          *this, fixture_plan.value);
  if (!model_instances.ok) {
    result.error = model_instances.error;
    return result;
  }
  cache_visual_curve_parts(std::move(visual_curves.value));
  cache_visual_model_instances(std::move(model_instances.value));
  result.ok = true;
  result.value = true;
  return result;
}

} // namespace city::wire
