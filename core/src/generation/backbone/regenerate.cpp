#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "../../collection_utils.hpp"
#include "curve_parts.hpp"
#include "pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

namespace wire::core {
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
  if (previous_template.count_rule != BundleCountRuleKind::kFixed ||
      next_template.count_rule != BundleCountRuleKind::kFixed || previous_template.fixed_count <= 0 ||
      next_template.fixed_count <= 0 ||
      (!count_changes && cable_template_override == nullptr && pole_type_override == nullptr &&
       cause != BackboneRegenerateCause::kSpanOverride && cause != BackboneRegenerateCause::kLayoutSettings)) {
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
  const std::size_t route_id = edge->route;
  std::vector<const SavedBackboneEdge*> route_edges{edge};
  auto find_adjacent_edge = [&](const SavedBackboneEdge& anchor, bool forward,
                                bool* ambiguous) -> const SavedBackboneEdge* {
    const SavedBackboneEdge* match = nullptr;
    if (ambiguous != nullptr) {
      *ambiguous = false;
    }
    for (const SavedBackboneEdge& candidate : graph.edges) {
      if (candidate.edge_id == anchor.edge_id || candidate.route != route_id) {
        continue;
      }
      const bool adjacent = forward ? (candidate.order == anchor.order + 1 && candidate.node_a == anchor.node_b)
                                    : (anchor.order == candidate.order + 1 && candidate.node_b == anchor.node_a);
      if (!adjacent) {
        continue;
      }
      if (match != nullptr) {
        if (ambiguous != nullptr) {
          *ambiguous = true;
        }
        return nullptr;
      }
      match = &candidate;
    }
    return match;
  };
  for (;;) {
    bool ambiguous = false;
    const SavedBackboneEdge* previous = find_adjacent_edge(*route_edges.front(), false, &ambiguous);
    if (ambiguous) {
      return fail("backbone unsupported: regenerate route adjacency is ambiguous");
    }
    if (previous == nullptr) {
      break;
    }
    route_edges.insert(route_edges.begin(), previous);
  }
  for (;;) {
    bool ambiguous = false;
    const SavedBackboneEdge* next = find_adjacent_edge(*route_edges.back(), true, &ambiguous);
    if (ambiguous) {
      return fail("backbone unsupported: regenerate route adjacency is ambiguous");
    }
    if (next == nullptr) {
      break;
    }
    route_edges.push_back(next);
  }
  for (ObjectId affected_edge_bundle_id : affected_edge_bundle_ids) {
    const SavedBackboneEdgeBundle* affected_edge_bundle = saved_edge_bundle_by_id(graph, affected_edge_bundle_id);
    const SavedBackboneEdge* affected_edge =
        affected_edge_bundle == nullptr ? nullptr : saved_edge_by_id(graph, affected_edge_bundle->edge_id);
    if (affected_edge_bundle == nullptr || affected_edge == nullptr || affected_edge->route != route_id) {
      return fail("backbone unsupported: regenerate supports one saved route at a time");
    }
    if (affected_edge_bundle->bundle_id != target.bundle_id) {
      return fail("backbone unsupported: regenerate requires one route-local bundle instance");
    }
  }
  target.edge_id = edge->edge_id;
  target.edge = edge;
  std::vector<const SavedBackboneNode*> route_nodes{};
  for (std::size_t edge_index = 0; edge_index < route_edges.size(); ++edge_index) {
    const SavedBackboneEdge& route_edge = *route_edges[edge_index];
    if (edge_index == 0) {
      route_nodes.push_back(saved_node_by_id(graph, route_edge.node_a));
    } else if (route_edges[edge_index - 1]->node_b != route_edge.node_a) {
      return fail("backbone unsupported: regenerate requires contiguous saved route edges");
    }
    route_nodes.push_back(saved_node_by_id(graph, route_edge.node_b));
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
        if (attachment_it != runtime_.relation_index.attachments_by_span.end() && !attachment_it->second.empty()) {
          return fail("backbone unsupported: regenerate does not preserve user attachments on retired spans yet");
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
  for (std::size_t edge_index = 0; edge_index < route_edges.size(); ++edge_index) {
    const SavedBackboneEdge& saved_edge = *route_edges[edge_index];
    generation::backbone::link made_link{};
    made_link.id = static_cast<int>(edge_index);
    made_link.a = static_cast<int>(edge_index);
    made_link.b = static_cast<int>(edge_index + 1);
    made_link.route = saved_edge.route;
    made_link.order = saved_edge.order;
    made_link.dir = saved_edge.dir;
    made_link.saved = saved_edge.edge_id;
    made_link.is_new = true;
    made_graph.links.push_back(std::move(made_link));
  }

  BackboneSpec spec{};
  spec.pole_type_id = kInvalidPoleTypeId;
  spec.constraints.lateral_offset_m = edge->lateral_offset_m;
  std::vector<std::size_t> active_bundle_indices{};
  for (const SavedBackboneEdgeBundle& scoped_edge_bundle : graph.edge_bundles) {
    if (scoped_edge_bundle.edge_id != route_edges.front()->edge_id) {
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
    bundle_spec.layer = template_it->second.default_layer;
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
  generation::backbone::pipeline trial_pipeline(trial, spec);
  EditResult<GenerateBundleFromPathResult> replay = trial_pipeline.build(
      trial_pipeline.build_input_from_saved_scope(std::move(made_graph), std::move(active_bundle_indices)));
  if (!replay.ok) {
    return fail(replay.error);
  }

  Bundle* edited_bundle = trial.authoritative_.edit_state.bundles.find(target.bundle_id);
  if (edited_bundle != nullptr) {
    edited_bundle->conductor_count = next_template.fixed_count;
    edited_bundle->phase_spacing_m = next_template.default_spacing_m;
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
  trial.cache_visual_curve_parts(generation::backbone::make_visual_curve_parts(trial, {}));

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
    result.error = "backbone regenerate: span is not bound to an edge bundle";
    return result;
  }
  const SavedBackboneEdgeBundle* edge_bundle = view().backbone_edge_bundle(backbone_it->second);
  const Bundle* bundle = edge_bundle == nullptr ? nullptr : view().bundles().find(edge_bundle->bundle_id);
  if (edge_bundle == nullptr || bundle == nullptr) {
    result.error = "backbone regenerate: span override scope is incomplete";
    return result;
  }
  const auto bundle_template_it = authoritative_.bundle_templates.find(bundle->bundle_template_id);
  if (bundle_template_it == authoritative_.bundle_templates.end()) {
    result.error = "bundle template not found";
    return result;
  }
  const std::vector<ObjectId> scope{edge_bundle->edge_bundle_id};
  return regenerate_backbone_edge_bundles(bundle->bundle_template_id, bundle_template_it->second,
                                          bundle_template_it->second, change_set, nullptr, &scope, nullptr,
                                          BackboneRegenerateCause::kSpanOverride);
}

} // namespace wire::core
