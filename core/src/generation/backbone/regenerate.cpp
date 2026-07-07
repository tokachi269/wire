#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

namespace wire::core {
namespace {

template <typename TValue> void append_unique(std::vector<TValue>& dst, const std::vector<TValue>& src) {
  for (const TValue& value : src) {
    if (std::find(dst.begin(), dst.end(), value) == dst.end()) {
      dst.push_back(value);
    }
  }
}

bool contains_id(const std::vector<ObjectId>& ids, ObjectId id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

void erase_ids(std::vector<ObjectId>& ids, const std::vector<ObjectId>& removed) {
  ids.erase(std::remove_if(ids.begin(), ids.end(),
                           [&](ObjectId id) { return contains_id(removed, id); }),
            ids.end());
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
                          ObjectId edge_bundle_id,
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
    if (binding.edge_bundle_id != edge_bundle_id || binding.lane_index < next_count) {
      return false;
    }
  }
  return true;
}

} // namespace

EditResult<bool> CoreState::regenerate_backbone_edge_bundles(BundleKind bundle_template_id,
                                                             const BundleTemplate& previous_template,
                                                             const BundleTemplate& next_template,
                                                             ChangeSet* change_set) {
  EditResult<bool> result{};
  auto fail = [&](std::string message) {
    result.error = std::move(message);
    return result;
  };

  if (previous_template.count_rule != BundleCountRuleKind::kFixed ||
      next_template.count_rule != BundleCountRuleKind::kFixed || previous_template.fixed_count <= 0 ||
      next_template.fixed_count <= 0 || next_template.fixed_count >= previous_template.fixed_count) {
    return fail("backbone unsupported: regenerate supports fixed count decreases only");
  }
  if (next_template.default_layer == SpanLayer::kUnknown) {
    return fail("backbone unsupported: regenerate requires a known layer");
  }
  if (!std::isfinite(next_template.default_spacing_m) || next_template.default_spacing_m <= 0.0) {
    return fail("backbone unsupported: regenerate requires positive finite spacing");
  }

  std::vector<ObjectId> affected_edge_bundle_ids{};
  for (const SavedBackboneEdgeBundle& edge_bundle : view().backbone().edge_bundles) {
    const Bundle* bundle = view().bundles().find(edge_bundle.bundle_id);
    if (bundle != nullptr && bundle->bundle_template_id == bundle_template_id) {
      affected_edge_bundle_ids.push_back(edge_bundle.edge_bundle_id);
    }
  }
  if (affected_edge_bundle_ids.empty()) {
    result.ok = true;
    result.value = true;
    return result;
  }
  if (affected_edge_bundle_ids.size() != 1) {
    return fail("backbone unsupported: regenerate supports one edge bundle only");
  }

  const SavedBackboneGraph& graph = view().backbone();
  RegenerateTarget target{};
  target.edge_bundle_id = affected_edge_bundle_ids.front();
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
  target.edge_id = edge->edge_id;
  target.edge = edge;
  target.node_a = saved_node_by_id(graph, edge->node_a);
  target.node_b = saved_node_by_id(graph, edge->node_b);
  if (target.node_a == nullptr || target.node_b == nullptr ||
      target.node_a->pole_id == kInvalidObjectId || target.node_b->pole_id == kInvalidObjectId) {
    return fail("backbone unsupported: regenerate supports pole-owned endpoints only");
  }

  const auto span_binding_it = runtime_.backbone_index.edge_bundle_span_bindings.find(target.edge_bundle_id);
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
    }
    const auto attachment_it = runtime_.relation_index.attachments_by_span.find(binding.span_id);
    if (attachment_it != runtime_.relation_index.attachments_by_span.end() && !attachment_it->second.empty()) {
      return fail("backbone unsupported: regenerate does not preserve user attachments yet");
    }
  }
  if (std::find(seen_lanes.begin(), seen_lanes.end(), false) != seen_lanes.end()) {
    return fail("backbone regenerate: previous bundle lanes are incomplete");
  }

  std::vector<SavedBackboneRowKey> row_keys{};
  const auto port_binding_it = runtime_.backbone_index.edge_bundle_ports.find(target.edge_bundle_id);
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
    if (port->position_mode == PortPositionMode::kManual || port->user_edited_position) {
      return fail("backbone unsupported: regenerate does not move manual ports");
    }
    if (!binding.row_key.source_is_open || binding.row_key.source_edge_a != edge_bundle->edge_id ||
        binding.row_key.source_edge_b != kInvalidObjectId) {
      return fail("backbone unsupported: regenerate supports simple open rows only");
    }
    if (std::find(row_keys.begin(), row_keys.end(), binding.row_key) == row_keys.end()) {
      row_keys.push_back(binding.row_key);
    }
    if (binding.lane_index >= static_cast<std::size_t>(next_template.fixed_count)) {
      target.retired_ports.push_back(binding.port_id);
    }
  }
  if (row_keys.size() != 2) {
    return fail("backbone unsupported: regenerate requires two endpoint rows");
  }
  for (ObjectId port_id : target.retired_ports) {
    if (!port_is_retired_only(*this, port_id, target.retired_spans, target.edge_bundle_id,
                              static_cast<std::size_t>(next_template.fixed_count))) {
      return fail("backbone unsupported: regenerate requires retired ports to be lane-local");
    }
  }

  generation::backbone::graph made_graph{};
  generation::backbone::node a{};
  a.id = 0;
  a.pos = target.node_a->position;
  a.support = target.node_a->support_kind;
  a.pole = target.node_a->pole_id;
  a.saved = target.node_a->node_id;
  a.is_new = false;
  a.on_route = true;
  a.bundle_modes = target.node_a->bundle_modes;
  generation::backbone::node b{};
  b.id = 1;
  b.pos = target.node_b->position;
  b.support = target.node_b->support_kind;
  b.pole = target.node_b->pole_id;
  b.saved = target.node_b->node_id;
  b.is_new = false;
  b.on_route = true;
  b.bundle_modes = target.node_b->bundle_modes;
  generation::backbone::link link{};
  link.id = 0;
  link.a = 0;
  link.b = 1;
  link.route = edge->route;
  link.order = edge->order;
  link.dir = edge->dir;
  link.saved = edge->edge_id;
  link.is_new = true;
  made_graph.nodes = {a, b};
  made_graph.links = {link};

  BackboneSpec spec{};
  spec.pole_type_id = kInvalidPoleTypeId;
  spec.constraints.lateral_offset_m = edge->lateral_offset_m;
  BackboneBundleSpec bundle_spec{};
  bundle_spec.bundle_template_id = bundle_template_id;
  bundle_spec.layer = next_template.default_layer;
  spec.bundles.push_back(bundle_spec);

  std::vector<BundleTemplate> template_overrides{next_template};
  CoreState trial = *this;
  generation::backbone::pipeline trial_pipeline(trial, spec);
  EditResult<GenerateBundleFromPathResult> replay =
      trial_pipeline.build_prepared_migration(made_graph, {0}, template_overrides);
  if (!replay.ok) {
    return fail(replay.error);
  }

  trial.authoritative_.bundle_templates[bundle_template_id] = next_template;
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
  auto rebuild_backbone_index = [&]() {
    BackboneIndex rebuilt{};
    auto& auth = trial.authoritative_;
    const SavedBackboneGraph& saved = auth.backbone;
    for (const SavedBackboneNode& node : saved.nodes) {
      if (node.pole_id != kInvalidObjectId) {
        rebuilt.pole_node[node.pole_id] = node.node_id;
      }
    }
    for (const SavedBackboneEdge& item : saved.edges) {
      index_add(rebuilt.node_edges, item.node_a, item.edge_id);
      index_add(rebuilt.node_edges, item.node_b, item.edge_id);
      const BackboneEdgeKey key{std::min(item.node_a, item.node_b), std::max(item.node_a, item.node_b)};
      rebuilt.edge_by_nodes[key] = item.edge_id;
    }
    for (const SavedBackboneEdgeBundle& item : saved.edge_bundles) {
      index_add(rebuilt.edge_bundles, item.edge_id, item.edge_bundle_id);
      index_add(rebuilt.bundle_edge, item.bundle_id, item.edge_id);
      for (ObjectId span_id : item.span_ids) {
        index_add(rebuilt.edge_bundle_spans, item.edge_bundle_id, span_id);
        rebuilt.span_edge_bundle[span_id] = item.edge_bundle_id;
      }
    }
    for (std::size_t i = 0; i < saved.span_bindings.size(); ++i) {
      const SavedBackboneSpanBinding& binding = saved.span_bindings[i];
      rebuilt.edge_bundle_span_bindings[binding.edge_bundle_id].push_back(i);
      rebuilt.span_bindings_by_span[binding.span_id].push_back(i);
    }
    for (std::size_t i = 0; i < saved.port_bindings.size(); ++i) {
      const SavedBackbonePortBinding& binding = saved.port_bindings[i];
      rebuilt.edge_bundle_ports[binding.edge_bundle_id].push_back(i);
      rebuilt.port_bindings_by_port[binding.port_id].push_back(i);
    }
    trial.runtime_.backbone_index = std::move(rebuilt);
  };
  auto retire_from_trial = [&]() {
    for (ObjectId span_id : target.retired_spans) {
      const Span* span = trial.authoritative_.edit_state.spans.find(span_id);
      if (span == nullptr) {
        continue;
      }
      const Span copy = *span;
      trial.remove_span_from_indexes(copy);
      trial.authoritative_.edit_state.spans.remove(span_id);
      trial.runtime_.span_runtime_states.erase(span_id);
      trial.remove_span_from_caches(span_id);
      trial.runtime_.relation_index.attachments_by_span.erase(span_id);
      if (change_set != nullptr) {
        add_unique_id(change_set->deleted_ids, span_id);
      }
    }
    for (ObjectId port_id : target.retired_ports) {
      const Port* port = trial.authoritative_.edit_state.ports.find(port_id);
      if (port == nullptr) {
        continue;
      }
      index_remove(trial.runtime_.relation_index.ports_by_pole, port->owner_pole_id, port_id);
      trial.runtime_.connection_index.spans_by_port.erase(port_id);
      trial.authoritative_.edit_state.ports.remove(port_id);
      if (change_set != nullptr) {
        add_unique_id(change_set->deleted_ids, port_id);
      }
    }
    auto& auth = trial.authoritative_;
    for (SavedBackboneEdgeBundle& item : auth.backbone.edge_bundles) {
      if (item.edge_bundle_id == target.edge_bundle_id) {
        erase_ids(item.span_ids, target.retired_spans);
      }
    }
    auth.backbone.span_bindings.erase(
        std::remove_if(auth.backbone.span_bindings.begin(),
                       auth.backbone.span_bindings.end(),
                       [&](const SavedBackboneSpanBinding& binding) {
                         return contains_id(target.retired_spans, binding.span_id);
                       }),
        auth.backbone.span_bindings.end());
    auth.backbone.port_bindings.erase(
        std::remove_if(auth.backbone.port_bindings.begin(),
                       auth.backbone.port_bindings.end(),
                       [&](const SavedBackbonePortBinding& binding) {
                         return contains_id(target.retired_ports, binding.port_id);
                       }),
        auth.backbone.port_bindings.end());
    rebuild_backbone_index();
  };
  retire_from_trial();

  identity_ = trial.identity_;
  authoritative_ = trial.authoritative_;
  runtime_ = trial.runtime_;
  debug_ = trial.debug_;

  result.ok = true;
  result.value = true;
  return result;
}

} // namespace wire::core
