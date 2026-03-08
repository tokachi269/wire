#include "wire/core/core_state.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_set>
#include <vector>

namespace wire::core {

namespace {
constexpr double kZeroLengthEps = 1e-9;
}

const CurveCacheEntry* CoreState::find_curve_cache(ObjectId span_id) const {
  auto it = cache_state_.curve_cache.by_span.find(span_id);
  if (it == cache_state_.curve_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

const BoundsCacheEntry* CoreState::find_bounds_cache(ObjectId span_id) const {
  auto it = cache_state_.bounds_cache.by_span.find(span_id);
  if (it == cache_state_.bounds_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

const SpanVisualCacheEntry* CoreState::find_span_visual_cache(ObjectId span_id) const {
  auto it = cache_state_.visual_cache.by_span.find(span_id);
  if (it == cache_state_.visual_cache.by_span.end()) {
    return nullptr;
  }
  return &it->second;
}

const SpanRenderCacheEntry* CoreState::find_span_render_cache(ObjectId span_id) const {
  auto it = cache_state_.render_cache.by_span.find(span_id);
  if (it == cache_state_.render_cache.by_span.end()) {
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

  for (ObjectId span_id : dirty_queue_.topology_dirty_span_ids) {
    if (!processed_topology.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kTopology)) {
      continue;
    }
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kTopology;
    ++stats.topology_processed;
  }

  for (ObjectId span_id : dirty_queue_.geometry_dirty_span_ids) {
    if (!processed_geometry.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kGeometry)) {
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

  std::vector<ObjectId> bounds_queue = dirty_queue_.bounds_dirty_span_ids;
  for (ObjectId span_id : bounds_queue) {
    if (!processed_bounds.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kBounds)) {
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

  std::vector<ObjectId> render_queue = dirty_queue_.render_dirty_span_ids;
  for (ObjectId span_id : render_queue) {
    if (!processed_render.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kRender)) {
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

  for (ObjectId span_id : dirty_queue_.raycast_dirty_span_ids) {
    if (!processed_raycast.insert(span_id).second) {
      continue;
    }
    auto it = span_runtime_states_.find(span_id);
    if (it == span_runtime_states_.end() || !any(it->second.dirty_bits, DirtyBits::kRaycast)) {
      continue;
    }
    it->second.raycast_version = it->second.data_version;
    it->second.dirty_bits = it->second.dirty_bits & ~DirtyBits::kRaycast;
    ++stats.raycast_processed;
  }

  dirty_queue_ = DirtyQueue{};
  last_recalc_stats_ = stats;
  return stats;
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
  auto it = span_runtime_states_.find(span_id);
  if (it == span_runtime_states_.end()) {
    return nullptr;
  }
  return &it->second;
}

void CoreState::remove_span_from_indexes(const Span& span) {
  index_remove(connection_index_.spans_by_port, span.port_a_id, span.id);
  index_remove(connection_index_.spans_by_port, span.port_b_id, span.id);
  if (span.anchor_a_id != kInvalidObjectId) {
    index_remove(connection_index_.spans_by_anchor, span.anchor_a_id, span.id);
  }
  if (span.anchor_b_id != kInvalidObjectId) {
    index_remove(connection_index_.spans_by_anchor, span.anchor_b_id, span.id);
  }
  if (span.bundle_id != kInvalidObjectId) {
    index_remove(relation_index_.spans_by_bundle, span.bundle_id, span.id);
  }
}

void CoreState::add_span_to_index(const Span& span) {
  index_add(connection_index_.spans_by_port, span.port_a_id, span.id);
  index_add(connection_index_.spans_by_port, span.port_b_id, span.id);
  if (span.anchor_a_id != kInvalidObjectId) {
    index_add(connection_index_.spans_by_anchor, span.anchor_a_id, span.id);
  }
  if (span.anchor_b_id != kInvalidObjectId) {
    index_add(connection_index_.spans_by_anchor, span.anchor_b_id, span.id);
  }
  if (span.bundle_id != kInvalidObjectId) {
    index_add(relation_index_.spans_by_bundle, span.bundle_id, span.id);
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
  runtime.dirty_bits = DirtyBits::kNone;
  span_runtime_states_[span_id] = runtime;
}

void CoreState::mark_span_dirty(ObjectId span_id, DirtyBits dirty_bits, bool bump_data_version) {
  if (edit_state_.spans.find(span_id) == nullptr) {
    return;
  }
  SpanRuntimeState& runtime = span_runtime_states_[span_id];
  if (runtime.span_id == kInvalidObjectId) {
    runtime.span_id = span_id;
  }
  if (bump_data_version || runtime.data_version == 0) {
    runtime.data_version = next_data_version_++;
  }
  runtime.dirty_bits |= dirty_bits;
  add_dirty_queue(span_id, dirty_bits);
}

void CoreState::add_dirty_queue(ObjectId span_id, DirtyBits dirty_bits) {
  if (any(dirty_bits, DirtyBits::kTopology))
    dirty_queue_.topology_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kGeometry))
    dirty_queue_.geometry_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kBounds))
    dirty_queue_.bounds_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kRender))
    dirty_queue_.render_dirty_span_ids.push_back(span_id);
  if (any(dirty_bits, DirtyBits::kRaycast))
    dirty_queue_.raycast_dirty_span_ids.push_back(span_id);
}

void CoreState::mark_connected_spans_dirty_from_port(ObjectId port_id, DirtyBits dirty_bits, ChangeSet* change_set) {
  auto it = connection_index_.spans_by_port.find(port_id);
  if (it == connection_index_.spans_by_port.end()) {
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
  auto it = connection_index_.spans_by_anchor.find(anchor_id);
  if (it == connection_index_.spans_by_anchor.end()) {
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
  const Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span not found";
    }
    return false;
  }

  std::vector<Vec3d> points = generate_span_points(*span, error_message);
  if (points.size() < 2) {
    if (error_message != nullptr && error_message->empty()) {
      *error_message = "generated points are invalid";
    }
    return false;
  }

  CurveCacheEntry entry{};
  entry.points = std::move(points);
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  entry.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  cache_state_.curve_cache.by_span[span_id] = std::move(entry);
  return true;
}

bool CoreState::rebuild_span_bounds(ObjectId span_id, std::string* error_message) {
  const Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span not found";
    }
    return false;
  }
  auto curve_it = cache_state_.curve_cache.by_span.find(span_id);
  if (curve_it == cache_state_.curve_cache.by_span.end()) {
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
  cache_state_.bounds_cache.by_span[span_id] = std::move(bounds);
  return true;
}

bool CoreState::rebuild_span_visual(ObjectId span_id, std::string* error_message) {
  const Span* span = edit_state_.spans.find(span_id);
  if (span == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span not found";
    }
    return false;
  }
  const Port* a = edit_state_.ports.find(span->port_a_id);
  const Port* b = edit_state_.ports.find(span->port_b_id);
  if (a == nullptr || b == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span endpoint port is missing";
    }
    return false;
  }

  const Bundle* bundle = edit_state_.bundles.find(span->bundle_id);
  const BundleTemplate* bundle_template =
      (bundle == nullptr) ? nullptr : find_bundle_template(bundle->bundle_template_id);
  const CableTemplate* cable_template =
      (bundle_template == nullptr) ? nullptr : find_cable_template(bundle_template->cable_template_id);
  const bool requires_insulator = (cable_template != nullptr) ? cable_template->requires_insulator : false;

  auto append_parts_for_port = [&](const Port& port) {
    const Pole* pole = edit_state_.poles.find(port.owner_pole_id);
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

    SpanVisualCacheEntry& entry = cache_state_.visual_cache.by_span[span_id];
    if (cache_state_.visual_settings.enable_support_structures &&
        port.template_side != SlotSide::kCenter &&
        planar > cache_state_.visual_settings.support_center_threshold_m + 1e-9) {
      VisualPart arm{};
      arm.kind = VisualPartKind::kSupportArm;
      arm.a = {pole->world_transform.position.x, pole->world_transform.position.y, port.world_position.z};
      arm.b = {
          port.world_position.x + radial.x * cache_state_.visual_settings.support_arm_extra_m,
          port.world_position.y + radial.y * cache_state_.visual_settings.support_arm_extra_m,
          port.world_position.z,
      };
      arm.radius_m = 0.03;
      entry.parts.push_back(arm);
    }

    if (cache_state_.visual_settings.enable_insulators && requires_insulator) {
      VisualPart ins{};
      ins.kind = VisualPartKind::kInsulator;
      ins.a = port.world_position;
      ins.b = {
          port.world_position.x + radial.x * cache_state_.visual_settings.insulator_length_m,
          port.world_position.y + radial.y * cache_state_.visual_settings.insulator_length_m,
          port.world_position.z,
      };
      ins.radius_m = cache_state_.visual_settings.insulator_radius_m;
      entry.parts.push_back(ins);
    }
  };

  SpanVisualCacheEntry entry{};
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  entry.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  cache_state_.visual_cache.by_span[span_id] = std::move(entry);
  append_parts_for_port(*a);
  append_parts_for_port(*b);

  SpanRenderCacheEntry render{};
  render.source_version = (runtime == nullptr) ? 0 : runtime->data_version;
  if (cable_template != nullptr) {
    render.wire_radius_m = std::max(0.0005, cable_template->outer_diameter_m * 0.5);
    render.color_rgba = cable_template->color_rgba;
    render.material_style = cable_template->material_style;
  }
  cache_state_.render_cache.by_span[span_id] = render;
  return true;
}

std::vector<Vec3d> CoreState::generate_span_points(const Span& span, std::string* error_message) const {
  const Port* port_a = edit_state_.ports.find(span.port_a_id);
  const Port* port_b = edit_state_.ports.find(span.port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span endpoint port is missing";
    }
    return {};
  }

  const int samples = std::max(2, cache_state_.geometry_settings.curve_samples);
  std::vector<Vec3d> points;
  points.reserve(static_cast<std::size_t>(samples));

  const Vec3d a = port_a->world_position;
  const Vec3d b = port_b->world_position;
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dz = b.z - a.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
  const Bundle* bundle = edit_state_.bundles.find(span.bundle_id);
  const BundleTemplate* bundle_template =
      (bundle == nullptr) ? nullptr : find_bundle_template(bundle->bundle_template_id);
  const CableTemplate* cable_template =
      (bundle_template == nullptr) ? nullptr : find_cable_template(bundle_template->cable_template_id);
  const double sag_ratio = (cable_template == nullptr) ? cache_state_.geometry_settings.sag_factor
                                                       : (cable_template->sag_factor + cable_template->slack_factor);
  const bool use_reference_length = true;
  const double basis_length =
      (use_reference_length && span.reference_length_m > kZeroLengthEps) ? span.reference_length_m : distance;
  const bool use_sag = cache_state_.geometry_settings.sag_enabled && basis_length > kZeroLengthEps;
  const double sag_amount = sag_ratio * basis_length;

  for (int i = 0; i < samples; ++i) {
    const double t = (samples <= 1) ? 0.0 : (static_cast<double>(i) / static_cast<double>(samples - 1));
    Vec3d p{
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
        a.z + (b.z - a.z) * t,
    };
    if (use_sag) {
      const double sag_shape = 4.0 * t * (1.0 - t);
      OffsetAlongWorldUp(&p, -sag_amount * sag_shape);
    }
    points.push_back(p);
  }
  return points;
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
  cache_state_.curve_cache.by_span.erase(span_id);
  cache_state_.bounds_cache.by_span.erase(span_id);
  cache_state_.visual_cache.by_span.erase(span_id);
  cache_state_.render_cache.by_span.erase(span_id);
}

} // namespace wire::core
