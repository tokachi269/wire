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

const SpanSupportLayoutEntry* CoreState::find_span_support_layout(ObjectId span_id) const {
  auto it = runtime_.cache_state.support_layout_cache.by_span.find(span_id);
  if (it == runtime_.cache_state.support_layout_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

const SpanSupportLayoutDecisionSeed* CoreState::find_span_support_layout_seed(ObjectId span_id) const {
  auto it = runtime_.cache_state.support_layout_cache.decision_seeds_by_span.find(span_id);
  if (it == runtime_.cache_state.support_layout_cache.decision_seeds_by_span.end()) {
    return nullptr;
  }
  return &it->second;
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
  std::unordered_set<ObjectId> processed_geometry;
  std::unordered_set<ObjectId> processed_bounds;
  std::unordered_set<ObjectId> processed_render;
  std::unordered_set<ObjectId> processed_raycast;

  for (ObjectId span_id : runtime_.dirty_queue.topology_dirty_span_ids) {
    if (!processed_topology.insert(span_id).second) {
      continue;
    }
    auto it = runtime_.span_runtime_states.find(span_id);
    if (it == runtime_.span_runtime_states.end() || !any(it->second.dirty_bits, DirtyBits::kTopology)) {
      continue;
    }
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kTopology;
    ++stats.topology_processed;
  }

  for (ObjectId span_id : runtime_.dirty_queue.geometry_dirty_span_ids) {
    if (!processed_geometry.insert(span_id).second) {
      continue;
    }
    auto it = runtime_.span_runtime_states.find(span_id);
    if (it == runtime_.span_runtime_states.end() || !any(it->second.dirty_bits, DirtyBits::kGeometry)) {
      continue;
    }

    std::string error_message;
    if (!rebuild_span_curve(span_id, &error_message)) {
      continue;
    }
    it->second.geometry_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kGeometry;
    ++stats.geometry_processed;
    mark_span_dirty(span_id, DirtyBits::kBounds | DirtyBits::kRender, false);
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
    if (it == runtime_.span_runtime_states.end() || !any(it->second.dirty_bits, DirtyBits::kRender)) {
      continue;
    }
    std::string error_message;
    if (!rebuild_span_visual(span_id, &error_message)) {
      continue;
    }
    it->second.render_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kRender;
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
  if (any(dirty_bits, DirtyBits::kGeometry))
    runtime_.dirty_queue.geometry_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kBounds))
    runtime_.dirty_queue.bounds_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kRender))
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

bool CoreState::rebuild_span_curve(ObjectId span_id, std::string* error_message) {
  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span not found";
    }
    return false;
  }

  SpanSupportLayoutEntry support_layout = generate_span_support_layout(*span, error_message);
  if (support_layout.span_id == kInvalidObjectId) {
    return false;
  }

  DetailCurve detail = generate_span_curve(*span, support_layout, error_message);
  if (detail.sample_points.size() < 2) {
    if (error_message != nullptr && error_message->empty()) {
      *error_message = "generated points are invalid";
    }
    return false;
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

void CoreState::cache_span_support_layout(SpanSupportLayoutEntry layout) {
  const ObjectId span_id = layout.span_id;
  runtime_.cache_state.support_layout_cache.by_span[span_id] = std::move(layout);
  rebuild_lowered_support_groups_for_span(span_id);
}

void CoreState::cache_span_support_layout_seed(SpanSupportLayoutDecisionSeed seed) {
  if (seed.span_id == kInvalidObjectId) {
    return;
  }
  const ObjectId span_id = seed.span_id;
  runtime_.cache_state.support_layout_cache.decision_seeds_by_span[span_id] = std::move(seed);
  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    erase_cached_span_support_layout(span_id);
    return;
  }

  std::string error_message;
  SpanSupportLayoutEntry layout = generate_span_support_layout(*span, &error_message);
  if (layout.span_id == kInvalidObjectId) {
    erase_cached_span_support_layout(span_id);
    return;
  }
  cache_span_support_layout(std::move(layout));
}

void CoreState::erase_cached_span_support_layout_seed(ObjectId span_id) {
  runtime_.cache_state.support_layout_cache.decision_seeds_by_span.erase(span_id);
}

void CoreState::erase_cached_span_support_layout(ObjectId span_id) {
  runtime_.cache_state.support_layout_cache.by_span.erase(span_id);
  rebuild_all_lowered_support_groups(*this, authoritative_.edit_state, &runtime_.cache_state);
}

void CoreState::rebuild_lowered_support_groups_for_span(ObjectId span_id) {
  if (authoritative_.edit_state.spans.find(span_id) == nullptr) {
    return;
  }
  rebuild_all_lowered_support_groups(*this, authoritative_.edit_state, &runtime_.cache_state);
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
  const SpanSupportLayoutEntry* support_layout = find_span_support_layout(span_id);

  auto append_parts_for_port = [&](const Port& port, const SupportLayoutEndpoint* layout_endpoint) {
    const EndpointContinuityDecision* endpoint_decision =
        (layout_endpoint == nullptr) ? nullptr : &layout_endpoint->decision;
    ObjectId support_pole_id = port.owner_pole_id;
    if (support_pole_id == kInvalidObjectId && layout_endpoint != nullptr) {
      support_pole_id = layout_endpoint->owner_pole_id;
    }
    const Pole* pole = authoritative_.edit_state.poles.find(support_pole_id);
    if (pole == nullptr) {
      return;
    }
    const double dx = port.world_position.x - pole->world_transform.position.x;
    const double dy = port.world_position.y - pole->world_transform.position.y;
    const double planar = std::sqrt(dx * dx + dy * dy);
    Vec3d radial{
        (planar <= 1e-9) ? 1.0 : (dx / planar),
        (planar <= 1e-9) ? 0.0 : (dy / planar),
        0.0,
    };
    const bool uses_grouped_lowered_support = layout_endpoint != nullptr && endpoint_uses_grouped_lowered_support(layout_endpoint);

    SpanVisualCacheEntry& entry = runtime_.cache_state.visual_cache.by_span[span_id];
    if (uses_grouped_lowered_support) {
      return;
    }
    if (runtime_.cache_state.visual_settings.enable_support_structures &&
        port.template_side != SlotSide::kCenter &&
        planar > runtime_.cache_state.visual_settings.support_center_threshold_m + 1e-9) {
      VisualPart arm{};
      arm.kind = VisualPartKind::kSupportArm;
      arm.a = {pole->world_transform.position.x, pole->world_transform.position.y, port.world_position.z};
      arm.b = {
          port.world_position.x + radial.x * runtime_.cache_state.visual_settings.support_arm_extra_m,
          port.world_position.y + radial.y * runtime_.cache_state.visual_settings.support_arm_extra_m,
          port.world_position.z,
      };
      arm.radius_m = 0.03;
      entry.parts.push_back(arm);
    }

    if (runtime_.cache_state.visual_settings.enable_insulators && requires_insulator) {
      VisualPart ins{};
      ins.kind = VisualPartKind::kInsulator;
      ins.a = port.world_position;
      ins.b = {
          port.world_position.x,
          port.world_position.y,
          port.world_position.z + insulator_attachment_height_m,
      };
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
      (support_layout != nullptr && support_layout->start.port_id == a->id) ? &support_layout->start : nullptr;
  const SupportLayoutEndpoint* end_layout =
      (support_layout != nullptr && support_layout->end.port_id == b->id) ? &support_layout->end : nullptr;
  append_parts_for_port(*a, start_layout);
  append_parts_for_port(*b, end_layout);
  if (support_layout != nullptr) {
    for (const LoweredSupportGroupKey& key : support_layout->lowered_support_group_keys) {
      auto it = runtime_.cache_state.support_layout_cache.lowered_support_groups.find(key);
      if (it != runtime_.cache_state.support_layout_cache.lowered_support_groups.end()) {
        std::vector<Vec3d> span_attachment_worlds{};
        auto append_span_attachment = [&](const SupportLayoutEndpoint* endpoint) {
          if (!endpoint_uses_grouped_lowered_support(endpoint)) {
            return;
          }
          if (LoweredSupportGroupKeyFromDecision(endpoint->decision) != key) {
            return;
          }
          span_attachment_worlds.push_back(endpoint->endpoint_world);
        };
        append_span_attachment(start_layout);
        append_span_attachment(end_layout);
        append_grouped_lowered_support_parts(it->second, span_attachment_worlds);
      }
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
  const auto curve_it = runtime_.cache_state.curve_cache.by_span.find(span_id);
  if (curve_it != runtime_.cache_state.curve_cache.by_span.end()) {
    render.arc_length_m_by_point = curve_it->second.detail.distance_attributes.arc_length_m;
    render.arc_length_normalized_by_point = curve_it->second.detail.distance_attributes.arc_length_normalized;
    render.segment_length_m = curve_it->second.detail.distance_attributes.segment_length_m;
  }
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

