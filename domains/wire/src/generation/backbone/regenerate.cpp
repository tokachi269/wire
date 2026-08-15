#include "city/wire/core_state.hpp"
#include "city/wire/core_view.hpp"
#include "city/wire/coord_utils.hpp"

#include "../../collection_utils.hpp"
#include "curve_parts.hpp"
#include "model_assembly.hpp"
#include "pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <string>
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
  ObjectId edge_id = kInvalidObjectId;
  ObjectId via_node_id = kInvalidObjectId;
};

struct LoadedRouteEdge {
  const SavedBackboneEdge* edge = nullptr;
  ObjectId from_node_id = kInvalidObjectId;
  ObjectId to_node_id = kInvalidObjectId;
  Vec3d dir{};
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
    ObjectId edge_id,
    ObjectId neighbor_edge_id,
    ObjectId via_node_id) {
  if (adjacency == nullptr || edge_id == kInvalidObjectId ||
      neighbor_edge_id == kInvalidObjectId || edge_id == neighbor_edge_id ||
      via_node_id == kInvalidObjectId) {
    return;
  }
  std::vector<ContinuityNeighbor>& neighbors = (*adjacency)[edge_id];
  const auto duplicate = std::find_if(neighbors.begin(), neighbors.end(), [&](const ContinuityNeighbor& item) {
    return item.edge_id == neighbor_edge_id && item.via_node_id == via_node_id;
  });
  if (duplicate == neighbors.end()) {
    neighbors.push_back(ContinuityNeighbor{neighbor_edge_id, via_node_id});
  }
}

EditResult<std::vector<std::vector<LoadedRouteEdge>>> continuity_routes_from_saved_graph(
    const SavedBackboneGraph& graph) {
  EditResult<std::vector<std::vector<LoadedRouteEdge>>> result{};
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
    const auto a_it = edge_bundle_by_id.find(continuity.a.edge_bundle_id);
    const auto b_it = edge_bundle_by_id.find(continuity.b.edge_bundle_id);
    if (a_it == edge_bundle_by_id.end() || b_it == edge_bundle_by_id.end()) {
      result.error = "authoritative invalid input: authoritative deserialization: row continuity edge bundle is missing";
      return result;
    }
    add_continuity_neighbor(&adjacency, a_it->second->edge_id, b_it->second->edge_id, continuity.node_id);
    add_continuity_neighbor(&adjacency, b_it->second->edge_id, a_it->second->edge_id, continuity.node_id);
  }

  std::vector<ObjectId> remaining{};
  remaining.reserve(graph.edges.size());
  for (const SavedBackboneEdge& edge : graph.edges) {
    remaining.push_back(edge.edge_id);
  }
  auto remove_remaining = [&](ObjectId edge_id) {
    remaining.erase(std::remove(remaining.begin(), remaining.end(), edge_id), remaining.end());
  };
  auto degree = [&](ObjectId edge_id) {
    const auto it = adjacency.find(edge_id);
    return it == adjacency.end() ? std::size_t{0} : it->second.size();
  };
  auto choose_start = [&]() {
    ObjectId selected = kInvalidObjectId;
    for (ObjectId edge_id : remaining) {
      if (selected == kInvalidObjectId ||
          (degree(edge_id) <= 1 && (degree(selected) > 1 || edge_id < selected)) ||
          (degree(edge_id) == degree(selected) && edge_id < selected)) {
        selected = edge_id;
      }
    }
    return selected;
  };

  while (!remaining.empty()) {
    const ObjectId start_id = choose_start();
    const auto start_it = edge_by_id.find(start_id);
    if (start_it == edge_by_id.end()) {
      result.error = "authoritative invalid input: authoritative deserialization: saved route edge is missing";
      return result;
    }
    std::vector<const SavedBackboneEdge*> edges{};
    std::vector<ObjectId> shared_nodes{};
    ObjectId previous_edge_id = kInvalidObjectId;
    ObjectId current_edge_id = start_id;
    for (;;) {
      const auto current_it = edge_by_id.find(current_edge_id);
      if (current_it == edge_by_id.end()) {
        result.error = "authoritative invalid input: authoritative deserialization: saved route edge is missing";
        return result;
      }
      if (std::find(remaining.begin(), remaining.end(), current_edge_id) == remaining.end()) {
        result.error = "authoritative invalid input: authoritative deserialization: saved route continuity cycle is ambiguous";
        return result;
      }
      edges.push_back(current_it->second);
      remove_remaining(current_edge_id);

      std::vector<ContinuityNeighbor> next_candidates{};
      const auto neighbors_it = adjacency.find(current_edge_id);
      if (neighbors_it != adjacency.end()) {
        for (const ContinuityNeighbor& neighbor : neighbors_it->second) {
          if (neighbor.edge_id != previous_edge_id) {
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
      previous_edge_id = current_edge_id;
      current_edge_id = next_candidates.front().edge_id;
    }

    std::vector<LoadedRouteEdge> route{};
    route.reserve(edges.size());
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
      if (from_node_id == kInvalidObjectId || to_node_id == kInvalidObjectId ||
          from_node_id == to_node_id) {
        result.error = "authoritative invalid input: authoritative deserialization: saved route continuity endpoint is invalid";
        return result;
      }
      route.push_back(LoadedRouteEdge{&edge, from_node_id, to_node_id,
                                      oriented_edge_dir(edge, from_node_id, to_node_id)});
    }
    result.value.push_back(std::move(route));
  }

  std::sort(result.value.begin(), result.value.end(), [](const auto& a, const auto& b) {
    const ObjectId a_id = a.empty() || a.front().edge == nullptr ? kInvalidObjectId : a.front().edge->edge_id;
    const ObjectId b_id = b.empty() || b.front().edge == nullptr ? kInvalidObjectId : b.front().edge->edge_id;
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
                                                             BackboneRegenerateCause cause) {
  EditResult<bool> result{};
  auto fail = [&](std::string message) {
    result.error = std::move(message);
    return result;
  };

  const bool count_changes = next_template.fixed_count != previous_template.fixed_count;
  const bool bundle_topology_change = cause == BackboneRegenerateCause::kBundleTopology;
  if (previous_template.count_rule != BundleCountRuleKind::kFixed ||
      next_template.count_rule != BundleCountRuleKind::kFixed || previous_template.fixed_count <= 0 ||
      next_template.fixed_count <= 0 ||
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
  if (bundle == nullptr || bundle->conductor_count != previous_template.fixed_count) {
    return fail("backbone unsupported: bundle count is not synchronized with previous template");
  }
  const SavedBackboneEdge* edge = saved_edge_by_id(graph, edge_bundle->edge_id);
  if (edge == nullptr) {
    return fail("backbone regenerate: edge missing");
  }
  const EditResult<std::vector<std::vector<LoadedRouteEdge>>> continuity_routes =
      continuity_routes_from_saved_graph(graph);
  if (!continuity_routes.ok) {
    return fail(continuity_routes.error);
  }
  const std::vector<LoadedRouteEdge>* route_edges = nullptr;
  for (const std::vector<LoadedRouteEdge>& route : continuity_routes.value) {
    const bool contains_seed = std::any_of(route.begin(), route.end(), [&](const LoadedRouteEdge& item) {
      return item.edge != nullptr && item.edge->edge_id == edge->edge_id;
    });
    if (contains_seed) {
      route_edges = &route;
      break;
    }
  }
  if (route_edges == nullptr || route_edges->empty()) {
    return fail("backbone regenerate: row continuity route missing");
  }
  for (ObjectId affected_edge_bundle_id : affected_edge_bundle_ids) {
    const SavedBackboneEdgeBundle* affected_edge_bundle = saved_edge_bundle_by_id(graph, affected_edge_bundle_id);
    const SavedBackboneEdge* affected_edge =
        affected_edge_bundle == nullptr ? nullptr : saved_edge_by_id(graph, affected_edge_bundle->edge_id);
    if (affected_edge_bundle == nullptr || affected_edge == nullptr) {
      return fail("backbone regenerate: scoped edge bundle is incomplete");
    }
    const bool in_route = std::any_of(route_edges->begin(), route_edges->end(), [&](const LoadedRouteEdge& item) {
      return item.edge != nullptr && item.edge->edge_id == affected_edge->edge_id;
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
  for (std::size_t edge_index = 0; edge_index < route_edges->size(); ++edge_index) {
    const LoadedRouteEdge& route_edge = (*route_edges)[edge_index];
    if (edge_index == 0) {
      route_nodes.push_back(saved_node_by_id(graph, route_edge.from_node_id));
    } else if ((*route_edges)[edge_index - 1].to_node_id != route_edge.from_node_id) {
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
    std::vector<bool> seen_lanes(static_cast<std::size_t>(previous_template.fixed_count), false);
    for (std::size_t binding_index : span_binding_it->second) {
      if (binding_index >= graph.span_bindings.size()) {
        return fail("backbone regenerate: span binding index invalid");
      }
      const SavedBackboneSpanBinding& binding = graph.span_bindings[binding_index];
      if (binding.lane_index >= seen_lanes.size()) {
        return fail("backbone unsupported: existing span lane is outside previous bundle count");
      }
      seen_lanes[binding.lane_index] = true;
      if (binding.lane_index >= static_cast<std::size_t>(next_template.fixed_count)) {
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
      if (binding.lane_index >= static_cast<std::size_t>(next_template.fixed_count)) {
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
                              static_cast<std::size_t>(next_template.fixed_count))) {
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
  for (std::size_t edge_index = 0; edge_index < route_edges->size(); ++edge_index) {
    const LoadedRouteEdge& route_edge = (*route_edges)[edge_index];
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
  for (const SavedBackboneEdgeBundle& scoped_edge_bundle : graph.edge_bundles) {
    if (scoped_edge_bundle.edge_id != route_edges->front().edge->edge_id) {
      continue;
    }
    const Bundle* scoped_bundle = view().bundles().find(scoped_edge_bundle.bundle_id);
    if (scoped_bundle == nullptr) {
      return fail("backbone regenerate: scoped bundle missing");
    }
    const auto template_it = authoritative_.bundle_templates.find(scoped_bundle->bundle_template_id);
    if (template_it == authoritative_.bundle_templates.end()) {
      return fail("backbone regenerate: scoped bundle template missing");
    }
    BackboneBundleSpec bundle_spec{};
    bundle_spec.bundle_template_id = scoped_bundle->bundle_template_id;
    bundle_spec.placement_key = scoped_bundle->placement_key;
    bundle_spec.existing_bundle_id = scoped_bundle->id;
    bundle_spec.layer = scoped_bundle->bundle_template_id == bundle_template_id ? next_template.default_layer
                                                                                  : template_it->second.default_layer;
    bundle_spec.placement_explicit = scoped_bundle->placement_explicit;
    bundle_spec.height_m = scoped_bundle->height_m;
    bundle_spec.lateral_m = scoped_bundle->lateral_m;
    bundle_spec.spacing_m = scoped_bundle->spacing_override_m;
    spec.bundles.push_back(bundle_spec);
    active_bundle_indices.push_back(spec.bundles.size() - 1);
  }
  auto target_spec_it = std::find_if(spec.bundles.begin(), spec.bundles.end(),
                                    [&](const BackboneBundleSpec& item) {
                                      return item.bundle_template_id == bundle_template_id;
                                    });
  if (target_spec_it == spec.bundles.end()) {
    return fail("backbone regenerate: target bundle spec missing");
  }

  CoreState trial = *this;
  trial.authoritative_.bundle_templates[bundle_template_id] = next_template;
  if (cable_template_override != nullptr) {
    trial.authoritative_.cable_templates[cable_template_override->id] = *cable_template_override;
  }
  if (pole_type_override != nullptr) {
    trial.authoritative_.pole_types[pole_type_override->id] = *pole_type_override;
  }
  trial.remove_backbone_row_continuities_for_lanes(target.edge_bundle_ids,
                                                   static_cast<std::size_t>(next_template.fixed_count));
  generation::backbone::pipeline trial_pipeline(trial, spec);
  EditResult<GenerateBundleFromPathResult> replay = trial_pipeline.build(
      trial_pipeline.build_input_from_saved_scope(std::move(made_graph), std::move(active_bundle_indices)));
  if (!replay.ok) {
    return fail(replay.error);
  }

  Bundle* edited_bundle = trial.authoritative_.edit_state.bundles.find(target.bundle_id);
  if (edited_bundle != nullptr) {
    edited_bundle->conductor_count = next_template.fixed_count;
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
  EditResult<std::vector<std::vector<LoadedRouteEdge>>> saved_routes =
      continuity_routes_from_saved_graph(saved);
  if (!saved_routes.ok) {
    result.error = saved_routes.error;
    return result;
  }

  for (const std::vector<LoadedRouteEdge>& edges : saved_routes.value) {
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
    for (const SavedBackboneEdgeBundle& edge_bundle : saved.edge_bundles) {
      if (edge_bundle.edge_id != edges.front().edge->edge_id) continue;
      const Bundle* bundle = authoritative_.edit_state.bundles.find(edge_bundle.bundle_id);
      if (bundle == nullptr) {
        result.error = "authoritative invalid input: authoritative deserialization: saved edge bundle is missing its bundle";
        return result;
      }
      const auto template_it = authoritative_.bundle_templates.find(bundle->bundle_template_id);
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
      active_bundle_indices.push_back(spec.bundles.size() - 1);
    }
    if (spec.bundles.empty()) {
      result.error = "authoritative invalid input: authoritative deserialization: saved route has no bundles";
      return result;
    }
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
      const SavedBackboneEdgeBundle* edge_bundle =
          saved_edge_bundle_by_id(saved, binding.edge_bundle_id);
      if (edge_bundle == nullptr || !route_edge_ids.contains(edge_bundle->edge_id)) {
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
