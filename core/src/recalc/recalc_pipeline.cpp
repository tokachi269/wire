#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"
#include "detail_curve_input_resolution.hpp"
#include "detail_curve_postprocess.hpp"
#include "support_layout_materialization.hpp"
#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

namespace wire::core {

namespace {

bool set_missing_seed_error(std::string* error_message) {
  if (error_message != nullptr && error_message->empty()) {
    *error_message = "support layout decision seed is missing";
  }
  return false;
}

bool set_missing_span_error(std::string* error_message) {
  if (error_message != nullptr) {
    *error_message = "span not found";
  }
  return false;
}

bool set_invalid_curve_error(std::string* error_message) {
  if (error_message != nullptr && error_message->empty()) {
    *error_message = "generated points are invalid";
  }
  return false;
}

bool set_missing_support_layout_error(std::string* error_message) {
  if (error_message != nullptr && error_message->empty()) {
    *error_message = "support layout cache missing";
  }
  return false;
}

DetailCurve make_straight_detail_curve(const Vec3d& a, const Vec3d& b) {
  DetailCurve out{};
  CurveConstraint start{};
  start.point = a;
  CurveConstraint end{};
  end.point = b;
  out.start_constraint = start;
  out.end_constraint = end;
  DetailCurveSegment segment{};
  segment.control_points = {a, a, b, b};
  segment.u_start = 0.0;
  segment.u_end = 1.0;
  out.segments.push_back(segment);
  out.sample_points = {a, b};
  const double length = std::sqrt(LengthSquared(b - a));
  out.arc_length_table = {{0.0, 0.0}, {1.0, length}};
  out.visible_intervals = {{0.0, length}};
  out.total_length_m = length;
  out.distance_attributes.arc_length_m = {0.0, static_cast<float>(length)};
  out.distance_attributes.arc_length_normalized = {0.0f, 1.0f};
  out.distance_attributes.segment_length_m = {static_cast<float>(length)};
  return out;
}

void append_unique_support_group_key(std::vector<LoweredSupportGroupKey>* keys, const LoweredSupportGroupKey& key) {
  if (keys == nullptr) {
    return;
  }
  if (std::find(keys->begin(), keys->end(), key) == keys->end()) {
    keys->push_back(key);
  }
}

void append_support_group_keys(std::vector<LoweredSupportGroupKey>* keys,
                               const std::vector<LoweredSupportGroupKey>& additional_keys) {
  if (keys == nullptr) {
    return;
  }
  for (const LoweredSupportGroupKey& key : additional_keys) {
    append_unique_support_group_key(keys, key);
  }
}

std::vector<LoweredSupportGroupKey> collect_cached_support_group_keys(const CacheState& cache_state, ObjectId span_id) {
  std::vector<LoweredSupportGroupKey> keys{};
  const SpanSupportLayoutAuthorityView authority = cache_state.span_layout_cache.authority_view(span_id);
  if (authority.has_authority()) {
    append_support_group_keys(&keys, collect_support_group_keys_for_seed(*authority.seed));
  }
  return keys;
}

void invalidate_topology_dependent_caches(CoreState& state, const EditState& edit_state, CacheState* cache_state,
                                          ObjectId span_id) {
  if (cache_state == nullptr) {
    return;
  }
  const std::vector<LoweredSupportGroupKey> affected_group_keys = collect_cached_support_group_keys(*cache_state, span_id);
  cache_state->span_layout_cache.clear_layout(span_id);
  rebuild_lowered_support_groups_for_keys(state, edit_state, cache_state, affected_group_keys);
  cache_state->curve_cache.by_span.erase(span_id);
  cache_state->bounds_cache.by_span.erase(span_id);
  cache_state->visual_cache.by_span.erase(span_id);
  cache_state->render_cache.by_span.erase(span_id);
}

bool endpoint_from_rule(const EditState& edit_state, const EndpointLayoutRule& rule, LayoutEndpoint* target,
                        std::string* error_message) {
  const Port* port = edit_state.ports.find(rule.port_id);
  if (port == nullptr || target == nullptr) {
    if (error_message != nullptr && error_message->empty()) {
      *error_message = "span endpoint port is missing";
    }
    return false;
  }
  static_cast<LayoutSemantic&>(*target) = rule.semantic;
  target->endpoint_node_id = rule.endpoint_node_id;
  target->port_id = rule.port_id;
  target->flow_kind = rule.flow_kind;
  target->origin = rule.origin;
  target->endpoint_source = rule.endpoint_source;
  target->port_source = rule.port_source;
  target->side = rule.side;
  target->endpoint_mode = rule.endpoint_mode;
  target->automatic_branch_down_offset_m = rule.automatic_branch_down_offset_m;
  target->branch_down_offset_m = rule.branch_down_offset_m;
  target->default_lower_required = rule.default_lower_required;
  target->same_level_feasible = rule.same_level_feasible;
  target->unresolved_same_level_conflict = rule.unresolved_same_level_conflict;
  target->same_level_reason = rule.same_level_reason;
  target->projected_spacing_topview_m = rule.projected_spacing_topview_m;
  target->required_clearance_m = rule.required_clearance_m;
  target->solver_used_same_level_constraint = rule.solver_used_same_level_constraint;
  target->used_special_case_ports = rule.used_special_case_ports;
  target->order_decision_policy = rule.order_decision_policy;
  target->order_decision_choice = rule.order_decision_choice;
  target->order_decision_choice_reason = rule.order_decision_choice_reason;
  target->chosen_side = rule.chosen_side;
  target->used_junction_pair_side_assignment = rule.used_junction_pair_side_assignment;
  target->down_offset_variation = rule.down_offset_variation;
  target->support_world = port->world_position;
  target->endpoint_world = port->world_position;
  if (rule.default_lower_required || rule.semantic.lower_required) {
    const double lower_offset =
        rule.branch_down_offset_m > 0.0 ? rule.branch_down_offset_m : rule.automatic_branch_down_offset_m;
    target->endpoint_world.z -= lower_offset;
    target->branch_down_offset_m = lower_offset;
    target->automatic_branch_down_offset_m = lower_offset;
  }
  target->departure_dir = WorldForward();
  return true;
}

}  // namespace

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

SpanSupportLayoutProjectionView CoreState::support_layout_projection(ObjectId span_id) const {
  return runtime_.cache_state.span_layout_cache.projection_view(span_id);
}

SpanSupportLayoutContractView CoreState::support_layout_contract(ObjectId span_id) const {
  return runtime_.cache_state.span_layout_cache.contract_view(span_id);
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

RecalcStats CoreState::ProcessDirtyQueues() {
  RecalcStats stats{};

  std::unordered_set<ObjectId> processed_topology;
  std::unordered_set<ObjectId> processed_decision;
  std::unordered_set<ObjectId> processed_geometry;
  std::unordered_set<ObjectId> processed_bounds;
  std::unordered_set<ObjectId> processed_render;
  std::unordered_set<ObjectId> processed_raycast;
  std::vector<ObjectId> topology_promoted_decision_ids{};

  for (ObjectId span_id : runtime_.dirty_queue.topology_dirty_span_ids) {
    if (!processed_topology.insert(span_id).second) {
      continue;
    }
    auto it = runtime_.span_runtime_states.find(span_id);
    if (it == runtime_.span_runtime_states.end() || !any(it->second.dirty_bits, DirtyBits::kTopology)) {
      continue;
    }
    invalidate_topology_dependent_caches(*this, authoritative_.edit_state, &runtime_.cache_state, span_id);
    if (!any(it->second.dirty_bits, DirtyBits::kDecision)) {
      it->second.dirty_bits |= DirtyBits::kDecision;
      topology_promoted_decision_ids.push_back(span_id);
    }
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kTopology;
    ++stats.topology_processed;
  }

  std::vector<ObjectId> decision_queue = runtime_.dirty_queue.decision_dirty_span_ids;
  decision_queue.insert(decision_queue.end(), topology_promoted_decision_ids.begin(), topology_promoted_decision_ids.end());
  for (ObjectId span_id : decision_queue) {
    if (!processed_decision.insert(span_id).second) {
      continue;
    }
    auto it = runtime_.span_runtime_states.find(span_id);
    if (it == runtime_.span_runtime_states.end() || !any(it->second.dirty_bits, DirtyBits::kDecision)) {
      continue;
    }

    // Decision dirty is the only rebuild path that is allowed to require a
    // decision-bearing seed. Geometry/render refresh must not silently upgrade
    // themselves into owner-topic recomputation.
    std::string error_message;
    if (!rebuild_span_decision_path(span_id, &error_message)) {
      continue;
    }
    it->second.geometry_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kDecision;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kGeometryRefresh;
    ++stats.decision_processed;
    mark_span_dirty(span_id, DirtyBits::kBounds | DirtyBits::kRenderRefresh, false);
  }

  for (ObjectId span_id : runtime_.dirty_queue.geometry_dirty_span_ids) {
    if (!processed_geometry.insert(span_id).second) {
      continue;
    }
    auto it = runtime_.span_runtime_states.find(span_id);
    if (it == runtime_.span_runtime_states.end() || !any(it->second.dirty_bits, DirtyBits::kGeometryRefresh)) {
      continue;
    }

    // Geometry refresh consumes the existing seed/materialized authority and
    // only refreshes layout/curve-derived caches.
    std::string error_message;
    if (!rebuild_span_geometry_from_seed(span_id, &error_message)) {
      continue;
    }
    it->second.geometry_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kGeometryRefresh;
    ++stats.geometry_processed;
    mark_span_dirty(span_id, DirtyBits::kBounds | DirtyBits::kRenderRefresh, false);
  }

  std::vector<ObjectId> bounds_queue = runtime_.dirty_queue.bounds_dirty_span_ids;
  for (ObjectId span_id : bounds_queue) {
    if (!processed_bounds.insert(span_id).second) {
      continue;
    }
    auto it = runtime_.span_runtime_states.find(span_id);
    if (it == runtime_.span_runtime_states.end() || !any(it->second.dirty_bits, DirtyBits::kBounds)) {
      continue;
    }

    std::string error_message;
    if (!rebuild_span_bounds(span_id, &error_message)) {
      continue;
    }
    it->second.bounds_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kBounds;
    ++stats.bounds_processed;
  }

  std::vector<ObjectId> render_queue = runtime_.dirty_queue.render_dirty_span_ids;
  for (ObjectId span_id : render_queue) {
    if (!processed_render.insert(span_id).second) {
      continue;
    }
    auto it = runtime_.span_runtime_states.find(span_id);
    if (it == runtime_.span_runtime_states.end() || !any(it->second.dirty_bits, DirtyBits::kRenderRefresh)) {
      continue;
    }
    // Render refresh is limited to visual/render cache rebuilds.
    std::string error_message;
    if (!rebuild_span_visual(span_id, &error_message)) {
      continue;
    }
    it->second.render_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kRenderRefresh;
    ++stats.render_processed;
  }

  for (ObjectId span_id : runtime_.dirty_queue.raycast_dirty_span_ids) {
    if (!processed_raycast.insert(span_id).second) {
      continue;
    }
    auto it = runtime_.span_runtime_states.find(span_id);
    if (it == runtime_.span_runtime_states.end() || !any(it->second.dirty_bits, DirtyBits::kRaycast)) {
      continue;
    }
    it->second.raycast_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kRaycast;
    ++stats.raycast_processed;
  }

  runtime_.dirty_queue = DirtyQueue{};
  runtime_.last_recalc_stats = stats;
  return stats;
}

CommitResult CoreState::Commit() {
  return Commit(CommitOptions{});
}

CommitResult CoreState::Commit(const CommitOptions& options) {
  CommitResult out{};
  if (options.run_recalc) {
    out.recalc_stats = ProcessDirtyQueues();
  }
  if (options.run_validate_fast) {
    out.validation = ValidateFast();
  }
  if (options.run_validate) {
    out.validation = Validate();
  }
  return out;
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

bool CoreState::cache_rebuilt_span_geometry(ObjectId span_id, SpanSupportLayoutEntry support_layout, DetailCurve detail,
                                            std::string* error_message) {
  if (detail.sample_points.size() < 2) {
    return set_invalid_curve_error(error_message);
  }

  CurveCacheEntry entry{};
  entry.detail = std::move(detail);
  entry.points = entry.detail.sample_points;
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  entry.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  support_layout.source_version = entry.source_version;
  cache_span_support_layout(std::move(support_layout));
  runtime_.cache_state.curve_cache.by_span[span_id] = std::move(entry);
  return true;
}

bool CoreState::rebuild_span_geometry_with_cached_contract(ObjectId span_id, std::string* error_message) {
  const SpanLayoutRulesView rules = runtime_.cache_state.span_layout_cache.rules_view(span_id);
  const SpanSupportLayoutContractView contract = runtime_.cache_state.span_layout_cache.contract_view(span_id);
  if (rules.has_rule() && !contract.has_authority()) {
    return rebuild_span_geometry_from_saved_rule(span_id, *rules.rule, error_message);
  }
  if (!rules.has_rule()) {
    if (contract.requires_authority() && !contract.has_authority()) {
      return set_missing_seed_error(error_message);
    }
  }
  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    return set_missing_span_error(error_message);
  }

  SpanSupportLayoutEntry support_layout = generate_span_support_layout(*span, error_message);
  if (support_layout.span_id == kInvalidObjectId) {
    return false;
  }
  DetailCurve detail = generate_span_curve(*span, support_layout, error_message);
  return cache_rebuilt_span_geometry(span_id, std::move(support_layout), std::move(detail), error_message);
}

bool CoreState::rebuild_span_decision_path(ObjectId span_id, std::string* error_message) {
  // Decision dirty is the only path that is allowed to (re)assert the
  // decision-seed contract on the cached support layout.
  runtime_.cache_state.span_layout_cache.require_authority(span_id);
  return rebuild_span_geometry_with_cached_contract(span_id, error_message);
}

bool CoreState::rebuild_span_geometry_from_seed(ObjectId span_id, std::string* error_message) {
  // Geometry refresh consumes the existing cached seed contract. It does not
  // reinterpret authority ownership and it does not downgrade/upgrade whether a
  // decision seed is required for this span.
  return rebuild_span_geometry_with_cached_contract(span_id, error_message);
}

bool CoreState::rebuild_span_geometry_from_saved_rule(ObjectId span_id, const SpanLayoutRule& rule,
                                                      std::string* error_message) {
  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    return set_missing_span_error(error_message);
  }
  SpanLayoutEntry layout{};
  layout.span_id = rule.span_id;
  layout.flow_kind = rule.flow_kind;
  layout.pass_mode = rule.pass_mode;
  layout.variation_flow_key = rule.variation_flow_key;
  layout.lowering_kind = rule.lowering_kind;
  if (!endpoint_from_rule(authoritative_.edit_state, rule.start, &layout.start, error_message) ||
      !endpoint_from_rule(authoritative_.edit_state, rule.end, &layout.end, error_message)) {
    return false;
  }
  auto append_group_key = [&](const LayoutEndpoint& endpoint) {
    if (!UsesAuthoritativeGroupedLoweredSupport(endpoint)) {
      return;
    }
    const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint);
    if (std::find(layout.lowered_support_group_keys.begin(), layout.lowered_support_group_keys.end(), key) ==
        layout.lowered_support_group_keys.end()) {
      layout.lowered_support_group_keys.push_back(key);
    }
  };
  append_group_key(layout.start);
  append_group_key(layout.end);
  const Vec3d chord = layout.end.endpoint_world - layout.start.endpoint_world;
  layout.basis_length_m = std::sqrt(LengthSquared(chord));
  layout.effective_sag_ratio = 0.0;
  layout.continuity_preference = CableContinuityPolicyHint::kAuto;
  layout.bend_stiffness_hint = 1.0;
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  layout.source_version = (runtime == nullptr) ? 0 : runtime->data_version;

  DetailCurve detail = make_straight_detail_curve(layout.start.endpoint_world, layout.end.endpoint_world);
  BoundsCacheEntry bounds{};
  bounds.whole = build_aabb_from_points(detail.sample_points);
  if (detail.sample_points.size() >= 2) {
    bounds.segments.reserve(detail.sample_points.size() - 1);
    for (std::size_t i = 0; i + 1 < detail.sample_points.size(); ++i) {
      bounds.segments.push_back(build_aabb_from_two_points(detail.sample_points[i], detail.sample_points[i + 1]));
    }
  }
  bounds.source_version = layout.source_version;

  cache_span_layout(std::move(layout));
  cache_span_curve(span_id, std::move(detail));
  cache_span_bounds(span_id, std::move(bounds));
  (void)rebuild_span_visual(span_id, error_message);
  return true;
}

void CoreState::cache_span_support_layout(SpanSupportLayoutEntry layout) {
  const ObjectId span_id = layout.span_id;
  std::vector<LoweredSupportGroupKey> affected_group_keys =
      collect_cached_support_group_keys(runtime_.cache_state, span_id);
  runtime_.cache_state.span_layout_cache.store_layout(std::move(layout));
  rebuild_lowered_support_groups_for_keys(*this, authoritative_.edit_state, &runtime_.cache_state, affected_group_keys);
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

void CoreState::cache_span_support_layout_seed(SpanSupportLayoutDecisionSeed seed) {
  if (seed.span_id == kInvalidObjectId) {
    return;
  }
  const ObjectId span_id = seed.span_id;
  std::vector<LoweredSupportGroupKey> affected_group_keys =
      collect_cached_support_group_keys(runtime_.cache_state, span_id);
  append_support_group_keys(&affected_group_keys, collect_support_group_keys_for_seed(seed));
  runtime_.cache_state.span_layout_cache.store_seed(std::move(seed));
  runtime_.cache_state.span_layout_cache.clear_layout(span_id);
  rebuild_lowered_support_groups_for_keys(*this, authoritative_.edit_state, &runtime_.cache_state, affected_group_keys);

  std::string error_message;
  if (rebuild_span_geometry_with_cached_contract(span_id, &error_message)) {
    (void)rebuild_span_bounds(span_id, &error_message);
    (void)rebuild_span_visual(span_id, &error_message);
    auto runtime_it = runtime_.span_runtime_states.find(span_id);
    if (runtime_it != runtime_.span_runtime_states.end()) {
      runtime_it->second.geometry_version = runtime_it->second.data_version;
      runtime_it->second.bounds_version = runtime_it->second.data_version;
      runtime_it->second.render_version = runtime_it->second.data_version;
      runtime_it->second.dirty_bits =
          runtime_it->second.dirty_bits & ~DirtyBits::kDecision & ~DirtyBits::kGeometryRefresh & ~DirtyBits::kBounds &
          ~DirtyBits::kRenderRefresh;
    }
  }
}

void CoreState::cache_span_layout_rules(const SpanLayoutRules& rules) {
  runtime_.cache_state.span_layout_cache.store_rules(rules);
  for (const SpanLayoutRule& rule : rules.spans) {
    SpanSupportLayoutDecisionSeed seed{};
    seed.span_id = rule.span_id;
    seed.flow_kind = rule.flow_kind;
    seed.pass_mode = rule.pass_mode;
    seed.variation_flow_key = rule.variation_flow_key;
    seed.lowering_kind = rule.lowering_kind;
    auto copy_endpoint = [](const EndpointLayoutRule& source) {
      SupportLayoutDecisionSeedEndpoint out{};
      static_cast<SupportLayoutSemanticDecision&>(out) = source.semantic;
      out.endpoint_node_id = source.endpoint_node_id;
      out.port_id = source.port_id;
      out.support_authority = source.support_authority;
      out.attachment_request = source.attachment_request;
      out.resolved_socket_id = source.resolved_socket_id;
      out.flow_kind = source.flow_kind;
      out.origin = source.origin;
      out.endpoint_source = source.endpoint_source;
      out.port_source = source.port_source;
      out.side = source.side;
      out.endpoint_mode = source.endpoint_mode;
      out.automatic_branch_down_offset_m = source.automatic_branch_down_offset_m;
      out.branch_down_offset_m = source.branch_down_offset_m;
      out.default_lower_required = source.default_lower_required;
      out.same_level_feasible = source.same_level_feasible;
      out.unresolved_same_level_conflict = source.unresolved_same_level_conflict;
      out.same_level_reason = source.same_level_reason;
      out.projected_spacing_topview_m = source.projected_spacing_topview_m;
      out.required_clearance_m = source.required_clearance_m;
      out.solver_used_same_level_constraint = source.solver_used_same_level_constraint;
      out.used_special_case_ports = source.used_special_case_ports;
      out.order_decision_policy = source.order_decision_policy;
      out.order_decision_choice = source.order_decision_choice;
      out.order_decision_choice_reason = source.order_decision_choice_reason;
      out.chosen_side = source.chosen_side;
      out.used_junction_pair_side_assignment = source.used_junction_pair_side_assignment;
      out.down_offset_variation = source.down_offset_variation;
      return out;
    };
    seed.start = copy_endpoint(rule.start);
    seed.end = copy_endpoint(rule.end);
    seed.support_group_decisions = rule.support_group_rules;
    cache_span_support_layout_seed(std::move(seed));
  }
}

void CoreState::cache_span_rules(const SpanLayoutRules& rules) {
  runtime_.cache_state.span_layout_cache.store_rules(rules);
}

void CoreState::erase_cached_span_support_layout_seed(ObjectId span_id) {
  const std::vector<LoweredSupportGroupKey> affected_group_keys =
      collect_cached_support_group_keys(runtime_.cache_state, span_id);
  runtime_.cache_state.span_layout_cache.clear_seed(span_id);
  rebuild_lowered_support_groups_for_keys(*this, authoritative_.edit_state, &runtime_.cache_state, affected_group_keys);
}

void CoreState::erase_cached_span_support_layout(ObjectId span_id) {
  const std::vector<LoweredSupportGroupKey> affected_group_keys =
      collect_cached_support_group_keys(runtime_.cache_state, span_id);
  runtime_.cache_state.span_layout_cache.clear_layout(span_id);
  rebuild_lowered_support_groups_for_keys(*this, authoritative_.edit_state, &runtime_.cache_state, affected_group_keys);
}

void CoreState::rebuild_lowered_support_groups_for_span(ObjectId span_id) {
  if (authoritative_.edit_state.spans.find(span_id) == nullptr) {
    return;
  }
  const std::vector<LoweredSupportGroupKey> affected_group_keys =
      collect_cached_support_group_keys(runtime_.cache_state, span_id);
  rebuild_lowered_support_groups_for_keys(*this, authoritative_.edit_state, &runtime_.cache_state, affected_group_keys);
}

bool CoreState::rebuild_span_bounds(ObjectId span_id, std::string* error_message) {
  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span not found";
    }
    return false;
  }
  auto curve_it = runtime_.cache_state.curve_cache.by_span.find(span_id);
  if (curve_it == runtime_.cache_state.curve_cache.by_span.end()) {
    if (error_message != nullptr) {
      *error_message = "curve cache missing";
    }
    return false;
  }
  const std::vector<Vec3d>& points = curve_it->second.points;
  if (points.size() < 2) {
    if (error_message != nullptr) {
      *error_message = "curve cache has too few points";
    }
    return false;
  }

  BoundsCacheEntry bounds{};
  bounds.whole = build_aabb_from_points(points);
  bounds.segments.reserve(points.size() - 1);
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    bounds.segments.push_back(build_aabb_from_two_points(points[i], points[i + 1]));
  }
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  bounds.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  runtime_.cache_state.bounds_cache.by_span[span_id] = std::move(bounds);
  return true;
}

bool CoreState::rebuild_span_visual(ObjectId span_id, std::string* error_message) {
  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span not found";
    }
    return false;
  }
  const Port* a = authoritative_.edit_state.ports.find(span->port_a_id);
  const Port* b = authoritative_.edit_state.ports.find(span->port_b_id);
  if (a == nullptr || b == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span endpoint port is missing";
    }
    return false;
  }

  const Bundle* bundle = authoritative_.edit_state.bundles.find(span->bundle_id);
  const BundleTemplate* bundle_template =
      (bundle == nullptr) ? nullptr : find_bundle_template(bundle->bundle_template_id);
  const CableTemplate* cable_template =
      (bundle_template == nullptr) ? nullptr : find_cable_template(bundle_template->cable_template_id);
  const bool requires_insulator = (cable_template != nullptr) ? cable_template->requires_insulator : false;
  const double insulator_attachment_height_m =
      (cable_template != nullptr && cable_template->requires_insulator && cable_template->insulator_attachment_height_m > 1e-9)
          ? cable_template->insulator_attachment_height_m
          : runtime_.cache_state.visual_settings.insulator_length_m;
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  const SpanSupportLayoutContractView contract = runtime_.cache_state.span_layout_cache.contract_view(span_id);
  if (!contract.has_projection()) {
    return set_missing_support_layout_error(error_message);
  }
  const SpanSupportLayoutEntry* support_layout = contract.projection.layout;
  const auto curve_it = runtime_.cache_state.curve_cache.by_span.find(span_id);
  if (curve_it == runtime_.cache_state.curve_cache.by_span.end()) {
    if (error_message != nullptr && error_message->empty()) {
      *error_message = "curve cache missing";
    }
    return false;
  }

  auto endpoint_prefers_span_local_lowered_visual = [](const SupportLayoutEndpoint* endpoint) {
    return endpoint != nullptr && endpoint_uses_grouped_lowered_support(endpoint) &&
           endpoint->relation_kind == JunctionRelationKind::kThroughMain &&
           endpoint->continuity_class == ContinuityCategoryClass::kBundleLike;
  };
  auto append_parts_for_port = [&](const Port& port, const SupportLayoutEndpoint* layout_endpoint) {
    if (layout_endpoint == nullptr) {
      return;
    }
    ObjectId support_pole_id = layout_endpoint->owner_pole_id;
    const Pole* pole = authoritative_.edit_state.poles.find(support_pole_id);
    if (pole == nullptr) {
      return;
    }
    const bool uses_grouped_lowered_support = layout_endpoint != nullptr && endpoint_uses_grouped_lowered_support(layout_endpoint);
    const bool prefers_span_local_lowered_visual = endpoint_prefers_span_local_lowered_visual(layout_endpoint);
    SpanVisualCacheEntry& entry = runtime_.cache_state.visual_cache.by_span[span_id];
    if (uses_grouped_lowered_support && !prefers_span_local_lowered_visual) {
      return;
    }
    // Render refresh consumes materialized world-space endpoints. It does not
    // reinterpret support authority into a new local/global axis here.
    const Vec3d attachment_world = layout_endpoint->endpoint_world;
    const Vec3d support_tip_world = layout_endpoint->support_world;
    const bool has_materialized_visual_arm = layout_endpoint->has_visual_arm_geometry;
    const Vec3d support_mount_world = has_materialized_visual_arm
                                          ? layout_endpoint->visual_arm_mount_world
                                          : Vec3d{pole->world_transform.position.x,
                                                  pole->world_transform.position.y,
                                                  support_tip_world.z};
    const Vec3d materialized_tip_world =
        has_materialized_visual_arm ? layout_endpoint->visual_arm_tip_world : support_tip_world;
    const double support_planar =
        std::sqrt((materialized_tip_world.x - support_mount_world.x) * (materialized_tip_world.x - support_mount_world.x) +
                  (materialized_tip_world.y - support_mount_world.y) * (materialized_tip_world.y - support_mount_world.y));
    if (runtime_.cache_state.visual_settings.enable_support_structures &&
        support_planar > runtime_.cache_state.visual_settings.support_center_threshold_m + 1e-9) {
      VisualPart arm{};
      arm.kind = VisualPartKind::kSupportArm;
      arm.a = support_mount_world;
      arm.b = materialized_tip_world;
      arm.radius_m = 0.03;
      entry.parts.push_back(arm);
    }

    if (runtime_.cache_state.visual_settings.enable_insulators && requires_insulator) {
      VisualPart ins{};
      ins.kind = VisualPartKind::kInsulator;
      ins.a = has_materialized_visual_arm ? layout_endpoint->visual_insulator_base_world : support_tip_world;
      ins.b = attachment_world;
      ins.radius_m = runtime_.cache_state.visual_settings.insulator_radius_m;
      entry.parts.push_back(ins);
    }
  };

  auto append_grouped_lowered_support_parts = [&](const LoweredSupportGroupPlacement& group,
                                                  const std::vector<Vec3d>& span_attachment_worlds) {
    if (span_attachment_worlds.empty()) {
      return;
    }
    SpanVisualCacheEntry& entry = runtime_.cache_state.visual_cache.by_span[span_id];
    if (runtime_.cache_state.visual_settings.enable_support_structures) {
      VisualPart arm{};
      arm.kind = VisualPartKind::kSupportArm;
      arm.a = group.mount_world;
      arm.b = group.tip_world;
      arm.radius_m = 0.03;
      entry.parts.push_back(arm);
      for (const Vec3d& attachment_world : span_attachment_worlds) {
        Vec3d hanger_target = attachment_world;
        if (requires_insulator && insulator_attachment_height_m > 1e-9) {
          hanger_target.z -= insulator_attachment_height_m;
        }
        VisualPart hanger{};
        hanger.kind = VisualPartKind::kFitting;
        hanger.a = group.tip_world;
        hanger.b = hanger_target;
        hanger.radius_m = 0.02;
        entry.parts.push_back(hanger);
      }
    }
    if (runtime_.cache_state.visual_settings.enable_insulators && requires_insulator) {
      for (const Vec3d& attachment_world : span_attachment_worlds) {
        Vec3d insulator_base = attachment_world;
        insulator_base.z -= insulator_attachment_height_m;
        VisualPart ins{};
        ins.kind = VisualPartKind::kInsulator;
        ins.a = insulator_base;
        ins.b = attachment_world;
        ins.radius_m = runtime_.cache_state.visual_settings.insulator_radius_m;
        entry.parts.push_back(ins);
      }
    }
  };

  SpanVisualCacheEntry entry{};
  entry.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  runtime_.cache_state.visual_cache.by_span[span_id] = std::move(entry);
  const SupportLayoutEndpoint* start_layout =
      (support_layout->start.port_id == a->id) ? &support_layout->start : nullptr;
  const SupportLayoutEndpoint* end_layout =
      (support_layout->end.port_id == b->id) ? &support_layout->end : nullptr;
  append_parts_for_port(*a, start_layout);
  append_parts_for_port(*b, end_layout);
  for (const LoweredSupportGroupKey& key : support_layout->lowered_support_group_keys) {
    auto it = runtime_.cache_state.span_layout_cache.support_groups.placement.by_key.find(key);
    if (it != runtime_.cache_state.span_layout_cache.support_groups.placement.by_key.end()) {
      std::vector<Vec3d> span_attachment_worlds{};
      bool span_has_local_owner_visual = false;
      auto append_span_attachment = [&](const SupportLayoutEndpoint* endpoint) {
        if (!endpoint_uses_grouped_lowered_support(endpoint) ||
            endpoint_prefers_span_local_lowered_visual(endpoint)) {
          if (endpoint != nullptr && endpoint_prefers_span_local_lowered_visual(endpoint) &&
              LoweredSupportGroupKeyFromDecision(*endpoint) == key) {
            span_has_local_owner_visual = true;
          }
          return;
        }
        if (LoweredSupportGroupKeyFromDecision(*endpoint) != key) {
          return;
        }
        span_attachment_worlds.push_back(endpoint->endpoint_world);
      };
      append_span_attachment(start_layout);
      append_span_attachment(end_layout);
      if (span_has_local_owner_visual) {
        continue;
      }
      append_grouped_lowered_support_parts(it->second, span_attachment_worlds);
    }
  }

  SpanRenderCacheEntry render{};
  render.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  if (cable_template != nullptr) {
    const ResolvedStyleContext style = resolve_style_context_for_span(*this, *span, StyleObjectKind::kSpan, 0, false);
    render.wire_radius_m = std::max(0.0005, cable_template->outer_diameter_m * 0.5);
    render.color_rgba = cable_template->color_rgba;
    render.material_style = resolve_effective_cable_material_style(cable_template, style);
  }
  render.arc_length_m_by_point = curve_it->second.detail.distance_attributes.arc_length_m;
  render.arc_length_normalized_by_point = curve_it->second.detail.distance_attributes.arc_length_normalized;
  render.segment_length_m = curve_it->second.detail.distance_attributes.segment_length_m;
  runtime_.cache_state.render_cache.by_span[span_id] = render;
  return true;
}

DetailCurve CoreState::generate_span_curve(const Span& span, const SpanSupportLayoutEntry& support_layout,
                                           std::string* error_message) const {
  const Port* port_a = authoritative_.edit_state.ports.find(span.port_a_id);
  const Port* port_b = authoritative_.edit_state.ports.find(span.port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span endpoint port is missing";
    }
    return {};
  }

  const int samples = std::max(2, runtime_.cache_state.geometry_settings.curve_samples);
  const Pole* pole_a = authoritative_.edit_state.poles.find(port_a->owner_pole_id);
  const Pole* pole_b = authoritative_.edit_state.poles.find(port_b->owner_pole_id);

  CurveConstraint start = make_curve_constraint_from_support_layout(
      support_layout.start, pole_a, support_layout.basis_length_m, support_layout.effective_sag_ratio,
      support_layout.bend_stiffness_hint, support_layout.min_bend_radius_hint_m,
      support_layout.continuity_preference, support_layout.pass_mode, support_layout.detail_curve_profile_hint,
      span.placement_context);
  CurveConstraint end = make_curve_constraint_from_support_layout(
      support_layout.end, pole_b, support_layout.basis_length_m, support_layout.effective_sag_ratio,
      support_layout.bend_stiffness_hint, support_layout.min_bend_radius_hint_m,
      support_layout.continuity_preference, support_layout.pass_mode, support_layout.detail_curve_profile_hint,
      span.placement_context);
  DetailCurve curve = BuildDetailCurve(start, end, samples);
  curve.quality.sag_variation = support_layout.sag_variation;
  apply_attachment_line_effects_to_curve(*this, span.id, &curve);
  return curve;
}

AABBd CoreState::build_aabb_from_points(const std::vector<Vec3d>& points) {
  AABBd box{};
  if (points.empty()) {
    return box;
  }
  box.min = points.front();
  box.max = points.front();
  double min_height = HeightAlongWorldUp(points.front());
  double max_height = min_height;
  for (const Vec3d& p : points) {
    box.min.x = std::min(box.min.x, p.x);
    box.min.y = std::min(box.min.y, p.y);
    box.max.x = std::max(box.max.x, p.x);
    box.max.y = std::max(box.max.y, p.y);
    min_height = std::min(min_height, HeightAlongWorldUp(p));
    max_height = std::max(max_height, HeightAlongWorldUp(p));
  }
  SetHeightAlongWorldUp(&box.min, min_height);
  SetHeightAlongWorldUp(&box.max, max_height);
  return box;
}

AABBd CoreState::build_aabb_from_two_points(const Vec3d& a, const Vec3d& b) {
  AABBd box{};
  box.min = {
      std::min(a.x, b.x),
      std::min(a.y, b.y),
      std::min(a.z, b.z),
  };
  box.max = {
      std::max(a.x, b.x),
      std::max(a.y, b.y),
      0.0,
  };
  SetHeightAlongWorldUp(&box.min, std::min(HeightAlongWorldUp(a), HeightAlongWorldUp(b)));
  SetHeightAlongWorldUp(&box.max, std::max(HeightAlongWorldUp(a), HeightAlongWorldUp(b)));
  return box;
}

void CoreState::remove_span_from_caches(ObjectId span_id) {
  runtime_.cache_state.curve_cache.by_span.erase(span_id);
  runtime_.cache_state.bounds_cache.by_span.erase(span_id);
  erase_cached_span_support_layout_seed(span_id);
  erase_cached_span_support_layout(span_id);
  runtime_.cache_state.visual_cache.by_span.erase(span_id);
  runtime_.cache_state.render_cache.by_span.erase(span_id);
}

} // namespace wire::core
