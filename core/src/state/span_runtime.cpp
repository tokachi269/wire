#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include <algorithm>
#include <chrono>
#include <unordered_set>
#include <vector>

namespace wire::core {

const CurveCacheEntry* CoreState::find_curve_cache(ObjectId span_id) const {
  auto it = runtime_.cache_state.curve_cache.by_span.find(span_id);
  if (it == runtime_.cache_state.curve_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

const BoundsCacheEntry* CoreState::find_bounds_cache(ObjectId span_id) const {
  auto it = runtime_.cache_state.bounds_cache.by_span.find(span_id);
  if (it == runtime_.cache_state.bounds_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

SpanLayoutView CoreState::span_layout(ObjectId span_id) const {
  return runtime_.cache_state.span_layout_cache.layout_view(span_id);
}

SpanLayoutState CoreState::span_layout_state(ObjectId span_id) const {
  return runtime_.cache_state.span_layout_cache.layout_state(span_id);
}

SpanLayoutRulesView CoreState::span_layout_rules(ObjectId span_id) const {
  return runtime_.cache_state.span_layout_cache.rules_view(span_id);
}

const SpanVisualCacheEntry* CoreState::find_span_visual_cache(ObjectId span_id) const {
  auto it = runtime_.cache_state.visual_cache.by_span.find(span_id);
  if (it == runtime_.cache_state.visual_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

const SpanRenderCacheEntry* CoreState::find_span_render_cache(ObjectId span_id) const {
  auto it = runtime_.cache_state.render_cache.by_span.find(span_id);
  if (it == runtime_.cache_state.render_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

const SpanRuntimeState* CoreState::find_span_runtime_state(ObjectId span_id) const {
  auto it = runtime_.span_runtime_states.find(span_id);
  if (it == runtime_.span_runtime_states.end()) {
    return nullptr;
  }
  return &it->second;
}

void CoreState::remove_span_from_indexes(const Span& span) {
  index_remove(runtime_.connection_index.spans_by_port, span.port_a_id, span.id);
  index_remove(runtime_.connection_index.spans_by_port, span.port_b_id, span.id);
  if (span.anchor_a_id != kInvalidObjectId) {
    index_remove(runtime_.connection_index.spans_by_anchor, span.anchor_a_id, span.id);
  }
  if (span.anchor_b_id != kInvalidObjectId) {
    index_remove(runtime_.connection_index.spans_by_anchor, span.anchor_b_id, span.id);
  }
  if (span.bundle_id != kInvalidObjectId) {
    index_remove(runtime_.relation_index.spans_by_bundle, span.bundle_id, span.id);
  }
}

void CoreState::add_span_to_index(const Span& span) {
  index_add(runtime_.connection_index.spans_by_port, span.port_a_id, span.id);
  index_add(runtime_.connection_index.spans_by_port, span.port_b_id, span.id);
  if (span.anchor_a_id != kInvalidObjectId) {
    index_add(runtime_.connection_index.spans_by_anchor, span.anchor_a_id, span.id);
  }
  if (span.anchor_b_id != kInvalidObjectId) {
    index_add(runtime_.connection_index.spans_by_anchor, span.anchor_b_id, span.id);
  }
  if (span.bundle_id != kInvalidObjectId) {
    index_add(runtime_.relation_index.spans_by_bundle, span.bundle_id, span.id);
  }
}

void CoreState::initialize_span_runtime_state(ObjectId span_id) {
  SpanRuntimeState runtime{};
  runtime.span_id = span_id;
  runtime.data_version = 0;
  runtime.geometry_version = 0;
  runtime.bounds_version = 0;
  runtime.render_version = 0;
  runtime.raycast_version = 0;
  runtime.variation_flow_key = 0;
  runtime.dirty_bits = DirtyBits::kNone;
  runtime_.span_runtime_states[span_id] = runtime;
}

void CoreState::mark_span_dirty(ObjectId span_id, DirtyBits dirty_bits, bool bump_data_version) {
  if (authoritative_.edit_state.spans.find(span_id) == nullptr) {
    return;
  }
  SpanRuntimeState& runtime = runtime_.span_runtime_states[span_id];
  if (runtime.span_id == kInvalidObjectId) {
    runtime.span_id = span_id;
  }
  if (bump_data_version || runtime.data_version == 0) {
    runtime.data_version = identity_.next_data_version++;
  }
  runtime.dirty_bits |= dirty_bits;
}

void CoreState::mark_connected_spans_dirty_from_port(ObjectId port_id, DirtyBits dirty_bits, ChangeSet* change_set) {
  auto it = runtime_.connection_index.spans_by_port.find(port_id);
  if (it == runtime_.connection_index.spans_by_port.end()) {
    return;
  }
  for (ObjectId span_id : it->second) {
    mark_span_dirty(span_id, dirty_bits, true);
    if (change_set != nullptr) {
      add_unique_id(change_set->dirty_span_ids, span_id);
      add_unique_id(change_set->updated_ids, span_id);
    }
  }
}

void CoreState::mark_connected_spans_dirty_from_anchor(ObjectId anchor_id, DirtyBits dirty_bits,
                                                       ChangeSet* change_set) {
  auto it = runtime_.connection_index.spans_by_anchor.find(anchor_id);
  if (it == runtime_.connection_index.spans_by_anchor.end()) {
    return;
  }
  for (ObjectId span_id : it->second) {
    mark_span_dirty(span_id, dirty_bits, true);
    if (change_set != nullptr) {
      add_unique_id(change_set->dirty_span_ids, span_id);
      add_unique_id(change_set->updated_ids, span_id);
    }
  }
}

std::vector<ObjectId> CoreState::collect_topology_related_spans_for_ports(const std::vector<ObjectId>& port_ids,
                                                                          ObjectId exclude_span_id) const {
  std::unordered_set<ObjectId> related_span_ids{};
  for (ObjectId port_id : port_ids) {
    if (port_id == kInvalidObjectId) {
      continue;
    }
    const Port* port = authoritative_.edit_state.ports.find(port_id);
    if (port == nullptr) {
      continue;
    }
    const auto spans_it = runtime_.connection_index.spans_by_port.find(port_id);
    if (spans_it != runtime_.connection_index.spans_by_port.end()) {
      for (ObjectId span_id : spans_it->second) {
        if (span_id != exclude_span_id) {
          related_span_ids.insert(span_id);
        }
      }
    }
    if (port->owner_pole_id == kInvalidObjectId) {
      continue;
    }
    const auto pole_ports_it = runtime_.relation_index.ports_by_pole.find(port->owner_pole_id);
    if (pole_ports_it == runtime_.relation_index.ports_by_pole.end()) {
      continue;
    }
    for (ObjectId peer_port_id : pole_ports_it->second) {
      const auto peer_spans_it = runtime_.connection_index.spans_by_port.find(peer_port_id);
      if (peer_spans_it == runtime_.connection_index.spans_by_port.end()) {
        continue;
      }
      for (ObjectId span_id : peer_spans_it->second) {
        if (span_id != exclude_span_id) {
          related_span_ids.insert(span_id);
        }
      }
    }
  }
  return std::vector<ObjectId>(related_span_ids.begin(), related_span_ids.end());
}

void CoreState::mark_topology_related_spans_for_ports_dirty(const std::vector<ObjectId>& port_ids, ObjectId exclude_span_id,
                                                            DirtyBits dirty_bits, ChangeSet* change_set) {
  const std::vector<ObjectId> related_span_ids = collect_topology_related_spans_for_ports(port_ids, exclude_span_id);
  for (ObjectId related_span_id : related_span_ids) {
    mark_span_dirty(related_span_id, dirty_bits, true);
    if (change_set != nullptr) {
      add_unique_id(change_set->dirty_span_ids, related_span_id);
      add_unique_id(change_set->updated_ids, related_span_id);
    }
  }
}

EditResult<UpdatePlan> CoreState::make_update_plan(UpdateRequest request) const {
  const auto started = std::chrono::steady_clock::now();
  EditResult<UpdatePlan> out{};
  UpdatePlan plan{};
  plan.kind = request.kind;

  auto add_unique = [](std::vector<ObjectId>* ids, ObjectId id) {
    if (ids == nullptr || id == kInvalidObjectId) {
      return;
    }
    if (std::find(ids->begin(), ids->end(), id) == ids->end()) {
      ids->push_back(id);
    }
  };

  auto add_spans_for_edge_bundle = [&](ObjectId edge_bundle_id) {
    if (edge_bundle_id == kInvalidObjectId) {
      return;
    }
    const auto binding_it = runtime_.backbone_index.edge_bundle_span_bindings.find(edge_bundle_id);
    if (binding_it == runtime_.backbone_index.edge_bundle_span_bindings.end()) {
      return;
    }
    for (std::size_t binding_index : binding_it->second) {
      if (binding_index >= authoritative_.backbone.span_bindings.size()) {
        continue;
      }
      add_unique(&plan.affected.spans, authoritative_.backbone.span_bindings[binding_index].span_id);
    }
  };

  auto add_edge_bundle = [&](ObjectId edge_bundle_id) {
    if (edge_bundle_id != kInvalidObjectId) {
      add_spans_for_edge_bundle(edge_bundle_id);
    }
  };

  auto add_edge = [&](ObjectId edge_id) {
    if (edge_id == kInvalidObjectId) {
      return;
    }
    add_unique(&plan.affected.edges, edge_id);
    const auto edge_bundle_it = runtime_.backbone_index.edge_bundles.find(edge_id);
    if (edge_bundle_it == runtime_.backbone_index.edge_bundles.end()) {
      return;
    }
    for (ObjectId edge_bundle_id : edge_bundle_it->second) {
      add_edge_bundle(edge_bundle_id);
    }
  };

  auto add_node = [&](ObjectId node_id) {
    if (node_id == kInvalidObjectId) {
      return;
    }
    const auto edge_it = runtime_.backbone_index.node_edges.find(node_id);
    if (edge_it == runtime_.backbone_index.node_edges.end()) {
      return;
    }
    for (ObjectId edge_id : edge_it->second) {
      add_edge(edge_id);
    }
  };

  switch (request.target_kind) {
  case UpdateTargetKind::kPole: {
    add_unique(&plan.affected.poles, request.target_id);
    const auto ports_it = runtime_.relation_index.ports_by_pole.find(request.target_id);
    if (ports_it != runtime_.relation_index.ports_by_pole.end()) {
      for (ObjectId port_id : ports_it->second) {
        add_unique(&plan.affected.ports, port_id);
      }
    }
    const auto node_it = runtime_.backbone_index.pole_node.find(request.target_id);
    if (node_it != runtime_.backbone_index.pole_node.end()) {
      add_node(node_it->second);
    }
    break;
  }
  case UpdateTargetKind::kPort: {
    add_unique(&plan.affected.ports, request.target_id);
    const auto port_binding_it = runtime_.backbone_index.port_bindings_by_port.find(request.target_id);
    if (port_binding_it != runtime_.backbone_index.port_bindings_by_port.end()) {
      for (std::size_t binding_index : port_binding_it->second) {
        if (binding_index >= authoritative_.backbone.port_bindings.size()) {
          continue;
        }
        const SavedBackbonePortBinding& port_binding = authoritative_.backbone.port_bindings[binding_index];
        const auto span_binding_it = runtime_.backbone_index.edge_bundle_span_bindings.find(port_binding.edge_bundle_id);
        if (span_binding_it == runtime_.backbone_index.edge_bundle_span_bindings.end()) {
          continue;
        }
        for (std::size_t span_binding_index : span_binding_it->second) {
          if (span_binding_index >= authoritative_.backbone.span_bindings.size()) {
            continue;
          }
          const SavedBackboneSpanBinding& span_binding = authoritative_.backbone.span_bindings[span_binding_index];
          if (span_binding.lane_index == port_binding.lane_index) {
            add_unique(&plan.affected.spans, span_binding.span_id);
          }
        }
      }
    }
    break;
  }
  case UpdateTargetKind::kSpan:
    add_unique(&plan.affected.spans, request.target_id);
    if (const auto edge_bundle_it = runtime_.backbone_index.span_edge_bundle.find(request.target_id);
        edge_bundle_it != runtime_.backbone_index.span_edge_bundle.end()) {
      const ObjectId edge_bundle_id = edge_bundle_it->second;
      const auto bundle_it =
          std::find_if(authoritative_.backbone.edge_bundles.begin(), authoritative_.backbone.edge_bundles.end(),
                       [&](const SavedBackboneEdgeBundle& bundle) { return bundle.edge_bundle_id == edge_bundle_id; });
      if (bundle_it != authoritative_.backbone.edge_bundles.end()) {
        add_unique(&plan.affected.edges, bundle_it->edge_id);
      }
    }
    break;
  case UpdateTargetKind::kAllSpans:
    for (const SavedBackboneSpanBinding& binding : authoritative_.backbone.span_bindings) {
      add_unique(&plan.affected.spans, binding.span_id);
    }
    for (const SavedBackboneEdge& edge : authoritative_.backbone.edges) {
      add_unique(&plan.affected.edges, edge.edge_id);
    }
    break;
  case UpdateTargetKind::kUnknown:
    out.error = "bb2 update: unknown target";
    return out;
  }

  std::sort(plan.affected.poles.begin(), plan.affected.poles.end());
  std::sort(plan.affected.ports.begin(), plan.affected.ports.end());
  std::sort(plan.affected.spans.begin(), plan.affected.spans.end());
  std::sort(plan.affected.edges.begin(), plan.affected.edges.end());
  for (ObjectId span_id : plan.affected.spans) {
    if (authoritative_.edit_state.spans.find(span_id) == nullptr) {
      out.error = "bb2 update: affected span not found";
      return out;
    }
    if (plan.kind == UpdateKind::kReposition &&
        !runtime_.cache_state.span_layout_cache.rules_view(span_id).has_rule()) {
      out.error = "bb2 update: affected span rules not found";
      return out;
    }
    if ((plan.kind == UpdateKind::kReshape || plan.kind == UpdateKind::kRedraw) &&
        !runtime_.cache_state.span_layout_cache.layout_view(span_id).has_layout()) {
      out.error = "bb2 update: affected span layout not found";
      return out;
    }
    if (plan.kind == UpdateKind::kRedraw && find_curve_cache(span_id) == nullptr) {
      out.error = "bb2 update: affected span curve not found";
      return out;
    }
  }
  plan.plan_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  out.value = std::move(plan);
  out.ok = true;
  return out;
}

void CoreState::cache_span_layout(SpanLayoutEntry layout) {
  runtime_.cache_state.span_layout_cache.store_layout(std::move(layout));
}

void CoreState::cache_span_curve(ObjectId span_id, DetailCurve detail) {
  if (span_id == kInvalidObjectId || detail.sample_points.size() < 2) {
    return;
  }
  CurveCacheEntry entry{};
  entry.detail = std::move(detail);
  entry.points = entry.detail.sample_points;
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  entry.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  runtime_.cache_state.curve_cache.by_span[span_id] = std::move(entry);
}

void CoreState::cache_span_bounds(ObjectId span_id, BoundsCacheEntry bounds) {
  if (span_id == kInvalidObjectId) {
    return;
  }
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  bounds.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  runtime_.cache_state.bounds_cache.by_span[span_id] = std::move(bounds);
}

void CoreState::cache_span_visual(ObjectId span_id, SpanVisualCacheEntry visual) {
  if (span_id == kInvalidObjectId) {
    return;
  }
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  visual.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  runtime_.cache_state.visual_cache.by_span[span_id] = std::move(visual);
}

void CoreState::cache_span_render(ObjectId span_id, SpanRenderCacheEntry render) {
  if (span_id == kInvalidObjectId) {
    return;
  }
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  render.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  runtime_.cache_state.render_cache.by_span[span_id] = std::move(render);
}

void CoreState::cache_span_rules(const SpanLayoutRules& rules) {
  runtime_.cache_state.span_layout_cache.store_rules(rules);
}

void CoreState::remove_span_from_caches(ObjectId span_id) {
  runtime_.cache_state.curve_cache.by_span.erase(span_id);
  runtime_.cache_state.bounds_cache.by_span.erase(span_id);
  runtime_.cache_state.span_layout_cache.clear_layout(span_id);
  runtime_.cache_state.visual_cache.by_span.erase(span_id);
  runtime_.cache_state.render_cache.by_span.erase(span_id);
}

} // namespace wire::core
