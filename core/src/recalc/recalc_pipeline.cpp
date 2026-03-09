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

CurvePassMode curve_pass_mode_from_context(ConnectionContext context) {
  switch (context) {
  case ConnectionContext::kBranchAdd:
    return CurvePassMode::kBranch;
  case ConnectionContext::kDropAdd:
    return CurvePassMode::kTerminate;
  default:
    return CurvePassMode::kPassThrough;
  }
}

CurveEndpointMode curve_endpoint_mode_for_template(const CableTemplate* cable_template,
                                                   const BundleTemplate* bundle_template) {
  if (cable_template == nullptr) {
    return CurveEndpointMode::kDirectThrough;
  }
  switch (cable_template->attachment_style) {
  case CableAttachmentStyleHint::kDirectThrough:
    return CurveEndpointMode::kDirectThrough;
  case CableAttachmentStyleHint::kViaAttachment:
    return CurveEndpointMode::kOffsetEndpoint;
  case CableAttachmentStyleHint::kAuto:
  default:
    break;
  }
  if (bundle_template != nullptr && bundle_template->category == ConnectionCategory::kHighVoltage) {
    return CurveEndpointMode::kOffsetEndpoint;
  }
  return CurveEndpointMode::kDirectThrough;
}

Vec3d span_tangent_from_port(const Port& port, const Vec3d& chord_dir) {
  Vec3d tangent = port.direction.forward;
  if (!Normalize(&tangent)) {
    return chord_dir;
  }
  if (Dot(tangent, chord_dir) < 0.0) {
    tangent = ScaleVec(tangent, -1.0);
  }
  Vec3d blended = tangent + chord_dir;
  if (!Normalize(&blended)) {
    return chord_dir;
  }
  return blended;
}

CurveConstraint make_curve_constraint(const Port& port, const Pole* owner_pole, const Vec3d& chord_dir, double basis_length,
                                      double endpoint_offset_m, double effective_sag_ratio,
                                      double bend_stiffness_hint, double min_bend_radius_hint_m,
                                      CableContinuityPolicyHint continuity_preference, CurvePassMode pass_mode,
                                      ConnectionContext connection_context,
                                      CurveEndpointMode endpoint_mode, bool reverse_endpoint_offset) {
  CurveConstraint constraint{};
  constraint.point = port.world_position;
  constraint.tangent_dir = span_tangent_from_port(port, chord_dir);
  constraint.tangent_length_hint_m = basis_length * 0.30;
  constraint.bend_stiffness_hint = bend_stiffness_hint;
  constraint.min_bend_radius_hint_m = min_bend_radius_hint_m;
  const double signed_offset_m = reverse_endpoint_offset ? -endpoint_offset_m : endpoint_offset_m;
  constraint.endpoint_offset = ScaleVec(constraint.tangent_dir, signed_offset_m);
  constraint.sag_hint = effective_sag_ratio * 0.5;
  constraint.slack_hint = 0.0;
  constraint.continuity_preference = continuity_preference;
  constraint.pass_mode = pass_mode;
  constraint.endpoint_mode = endpoint_mode;
  constraint.corner_pass =
      (connection_context == ConnectionContext::kCornerPass) && owner_pole != nullptr &&
      owner_pole->context.kind == PoleContextKind::kCorner && owner_pole->context.corner_angle_deg > 1e-6;
  constraint.corner_angle_deg = (owner_pole == nullptr) ? 0.0 : owner_pole->context.corner_angle_deg;
  return constraint;
}
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

  DetailCurve detail = generate_span_curve(*span, error_message);
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

    if (port.placement_source == PortPlacementSourceKind::kBranchSupport) {
      BranchSupportPlacement placement{};
      placement.owner_pole_id = pole->id;
      placement.peer_node_id =
          (port.id == span->port_a_id) ? span->endpoint_node_b_id : span->endpoint_node_a_id;
      placement.side = port.template_side;
      placement.mount_world = {pole->world_transform.position.x, pole->world_transform.position.y, port.world_position.z};
      placement.tip_world = {
          port.world_position.x + radial.x * cache_state_.visual_settings.support_arm_extra_m,
          port.world_position.y + radial.y * cache_state_.visual_settings.support_arm_extra_m,
          port.world_position.z,
      };
      placement.attachment_world = port.world_position;
      double main_support_base_z_m = pole->height_m * 0.8;
      if (const PoleTypeDefinition* pole_type = find_pole_type(pole->pole_type_id); pole_type != nullptr) {
        double best_slot_z = -std::numeric_limits<double>::infinity();
        for (const PortSlotTemplate& slot : pole_type->port_slots) {
          if (!slot.enabled || slot.category != port.category) {
            continue;
          }
          best_slot_z = std::max(best_slot_z, slot.local_position.z);
        }
        if (std::isfinite(best_slot_z)) {
          main_support_base_z_m = best_slot_z;
        }
      }
      const Vec3d local = to_local_on_pole(*pole, port.world_position);
      placement.down_offset_m = std::max(0.0, main_support_base_z_m - local.z);
      entry.branch_supports.push_back(placement);

      if (cache_state_.visual_settings.enable_support_structures) {
        VisualPart hanger{};
        hanger.kind = VisualPartKind::kFitting;
        hanger.a = placement.tip_world;
        hanger.b = placement.attachment_world;
        hanger.radius_m = 0.02;
        entry.parts.push_back(hanger);
      }
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
  const auto curve_it = cache_state_.curve_cache.by_span.find(span_id);
  if (curve_it != cache_state_.curve_cache.by_span.end()) {
    render.arc_length_m_by_point = curve_it->second.detail.distance_attributes.arc_length_m;
    render.arc_length_normalized_by_point = curve_it->second.detail.distance_attributes.arc_length_normalized;
    render.segment_length_m = curve_it->second.detail.distance_attributes.segment_length_m;
  }
  cache_state_.render_cache.by_span[span_id] = render;
  return true;
}

DetailCurve CoreState::generate_span_curve(const Span& span, std::string* error_message) const {
  const Port* port_a = edit_state_.ports.find(span.port_a_id);
  const Port* port_b = edit_state_.ports.find(span.port_b_id);
  if (port_a == nullptr || port_b == nullptr) {
    if (error_message != nullptr) {
      *error_message = "span endpoint port is missing";
    }
    return {};
  }

  const int samples = std::max(2, cache_state_.geometry_settings.curve_samples);

  const Vec3d a = port_a->world_position;
  const Vec3d b = port_b->world_position;
  const Pole* pole_a = edit_state_.poles.find(port_a->owner_pole_id);
  const Pole* pole_b = edit_state_.poles.find(port_b->owner_pole_id);
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
  const double effective_sag_ratio =
      (cache_state_.geometry_settings.sag_enabled && basis_length > kZeroLengthEps) ? sag_ratio : 0.0;

  Vec3d chord_dir = b - a;
  if (!Normalize(&chord_dir)) {
    chord_dir = WorldForward();
  }

  const CurveEndpointMode endpoint_mode = curve_endpoint_mode_for_template(cable_template, bundle_template);
  const double endpoint_offset_m = std::min(std::max(0.02, basis_length * 0.03), 0.35);
  const CableContinuityPolicyHint continuity_preference =
      (cable_template == nullptr) ? CableContinuityPolicyHint::kAuto : cable_template->continuity_policy;
  const CurvePassMode pass_mode = curve_pass_mode_from_context(span.placement_context);
  const double bend_stiffness_hint = (cable_template == nullptr) ? 1.0 : cable_template->bend_stiffness;
  const double min_bend_radius_hint_m = (cable_template == nullptr) ? 0.0 : cable_template->min_bend_radius_m;

  const CurveConstraint start =
      make_curve_constraint(*port_a, pole_a, chord_dir, basis_length, endpoint_offset_m, effective_sag_ratio,
                            bend_stiffness_hint, min_bend_radius_hint_m,
                            continuity_preference, pass_mode, span.placement_context, endpoint_mode, false);
  const CurveConstraint end =
      make_curve_constraint(*port_b, pole_b, chord_dir, basis_length, endpoint_offset_m, effective_sag_ratio,
                            bend_stiffness_hint, min_bend_radius_hint_m,
                            continuity_preference, pass_mode, span.placement_context, endpoint_mode, true);

  return BuildDetailCurve(start, end, samples);
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
