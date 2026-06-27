#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"
#include "detail_curve_input_resolution.hpp"
#include "detail_curve_postprocess.hpp"
#include <algorithm>
#include <cmath>
#include <string>
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
  add_dirty_queue(span_id, dirty_bits);
}

void CoreState::add_dirty_queue(ObjectId span_id, DirtyBits dirty_bits) {
  if (any(dirty_bits, DirtyBits::kTopology))
    runtime_.dirty_queue.topology_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kDecision))
    runtime_.dirty_queue.decision_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kGeometryRefresh))
    runtime_.dirty_queue.geometry_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kBounds))
    runtime_.dirty_queue.bounds_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kRenderRefresh))
    runtime_.dirty_queue.render_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kRaycast))
    runtime_.dirty_queue.raycast_dirty_span_ids.push_back(span_id);
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
