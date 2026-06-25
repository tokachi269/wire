#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace wire::core {
namespace {

Vec3d mul(Vec3d v, double k) {
  return Vec3d{v.x * k, v.y * k, v.z * k};
}

double length(Vec3d v) {
  return std::sqrt(LengthSquared(v));
}

DetailCurve line_curve(const Vec3d& a, const Vec3d& b) {
  const Vec3d d = b - a;
  const double l = length(d);
  DetailCurve out{};
  out.start_constraint.point = a;
  out.end_constraint.point = b;
  out.control_points = {a, a + mul(d, 1.0 / 3.0), a + mul(d, 2.0 / 3.0), b};
  DetailCurveSegment segment{};
  segment.control_points = out.control_points;
  segment.u_start = 0.0;
  segment.u_end = 1.0;
  out.segments.push_back(segment);
  out.sample_points = {a, b};
  out.arc_length_table = {{0.0, 0.0}, {1.0, l}};
  out.visible_intervals = {{0.0, l}};
  out.total_length_m = l;
  return out;
}

AABBd box(const Vec3d& a, const Vec3d& b) {
  AABBd out{};
  out.min = {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
  out.max = {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
  return out;
}

AABBd box(const std::vector<Vec3d>& pts) {
  AABBd out{};
  if (pts.empty()) {
    return out;
  }
  out.min = pts.front();
  out.max = pts.front();
  for (const Vec3d& p : pts) {
    out.min.x = std::min(out.min.x, p.x);
    out.min.y = std::min(out.min.y, p.y);
    out.min.z = std::min(out.min.z, p.z);
    out.max.x = std::max(out.max.x, p.x);
    out.max.y = std::max(out.max.y, p.y);
    out.max.z = std::max(out.max.z, p.z);
  }
  return out;
}

bool apply_endpoint(const EditState& edit_state, const EndpointLayoutRule& rule, LayoutEndpoint* target,
                    std::string* error) {
  const Port* port = edit_state.ports.find(rule.port_id);
  if (port == nullptr || target == nullptr) {
    if (error != nullptr) {
      *error = "bb2 derive: endpoint port not found";
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

SpanRenderCacheEntry render_entry(const CoreState& state, ObjectId span_id, const DetailCurve& detail) {
  SpanRenderCacheEntry out{};
  if (const Span* span = state.view().spans().find(span_id); span != nullptr) {
    if (const Bundle* bundle = state.view().bundles().find(span->bundle_id); bundle != nullptr) {
      const auto bundle_template_it = state.view().bundle_templates().find(bundle->bundle_template_id);
      if (bundle_template_it != state.view().bundle_templates().end()) {
        const auto cable_it = state.view().cable_templates().find(bundle_template_it->second.cable_template_id);
        if (cable_it != state.view().cable_templates().end()) {
          out.wire_radius_m = std::max(0.0005, cable_it->second.outer_diameter_m * 0.5);
          out.color_rgba = cable_it->second.color_rgba;
          out.material_style = cable_it->second.material_style;
        }
      }
    }
  }
  out.arc_length_m_by_point.reserve(detail.sample_points.size());
  out.arc_length_normalized_by_point.reserve(detail.sample_points.size());
  out.segment_length_m.reserve(detail.sample_points.size() > 0 ? detail.sample_points.size() - 1 : 0);
  double total = 0.0;
  for (std::size_t i = 0; i < detail.sample_points.size(); ++i) {
    if (i > 0) {
      const double segment = length(detail.sample_points[i] - detail.sample_points[i - 1]);
      total += segment;
      out.segment_length_m.push_back(static_cast<float>(segment));
    }
    out.arc_length_m_by_point.push_back(static_cast<float>(total));
  }
  const double denom = total > 1e-9 ? total : 1.0;
  for (float value : out.arc_length_m_by_point) {
    out.arc_length_normalized_by_point.push_back(static_cast<float>(static_cast<double>(value) / denom));
  }
  return out;
}

SpanVisualCacheEntry visual_entry(const VisualSettings& settings, const SpanLayoutEntry& layout) {
  SpanVisualCacheEntry out{};
  auto add_part = [&](const LayoutEndpoint& endpoint) {
    if (!settings.enable_support_structures) {
      return;
    }
    if (!endpoint.default_lower_required && !endpoint.lower_required) {
      return;
    }
    const Vec3d support = endpoint.support_world;
    if (length(endpoint.endpoint_world - support) <= 1e-9) {
      return;
    }
    VisualPart part{};
    part.kind = VisualPartKind::kSupportArm;
    part.a = support;
    part.b = endpoint.endpoint_world;
    part.radius_m = settings.support_arm_radius_m;
    out.parts.push_back(part);
  };
  add_part(layout.start);
  add_part(layout.end);
  return out;
}

} // namespace

EditResult<bool> CoreState::DeriveGeneratedSpanOutputs(ObjectId span_id) {
  EditResult<bool> out{};
  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    out.error = "bb2 derive: span not found";
    return out;
  }
  const SpanLayoutRulesView rule_view = runtime_.cache_state.span_layout_cache.rules_view(span_id);
  if (!rule_view.has_rule()) {
    out.error = "bb2 derive: span layout rule not found";
    return out;
  }
  const SpanLayoutRule& rule = *rule_view.rule;
  SpanLayoutEntry layout{};
  layout.span_id = rule.span_id;
  layout.flow_kind = rule.flow_kind;
  layout.pass_mode = rule.pass_mode;
  layout.variation_flow_key = rule.variation_flow_key;
  layout.lowering_kind = rule.lowering_kind;
  if (!apply_endpoint(authoritative_.edit_state, rule.start, &layout.start, &out.error) ||
      !apply_endpoint(authoritative_.edit_state, rule.end, &layout.end, &out.error)) {
    return out;
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
  layout.basis_length_m = length(chord);
  layout.effective_sag_ratio = 0.0;
  layout.continuity_preference = CableContinuityPolicyHint::kAuto;
  layout.bend_stiffness_hint = 1.0;
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  layout.source_version = (runtime == nullptr) ? 0 : runtime->data_version;

  DetailCurve curve = line_curve(layout.start.endpoint_world, layout.end.endpoint_world);
  BoundsCacheEntry bounds{};
  bounds.whole = box(curve.sample_points);
  if (curve.sample_points.size() >= 2) {
    bounds.segments.reserve(curve.sample_points.size() - 1);
    for (std::size_t i = 0; i + 1 < curve.sample_points.size(); ++i) {
      bounds.segments.push_back(box(curve.sample_points[i], curve.sample_points[i + 1]));
    }
  }
  bounds.source_version = layout.source_version;
  SpanVisualCacheEntry visual = visual_entry(runtime_.cache_state.visual_settings, layout);
  SpanRenderCacheEntry render = render_entry(*this, span_id, curve);

  cache_span_layout(std::move(layout));
  cache_span_curve(span_id, std::move(curve));
  cache_span_bounds(span_id, std::move(bounds));
  cache_span_visual(span_id, std::move(visual));
  cache_span_render(span_id, std::move(render));
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> CoreState::derive_generated_span_outputs_for_dirty_spans(const std::vector<ObjectId>& span_ids) {
  EditResult<bool> out{};
  auto erase_id = [](std::vector<ObjectId>& ids, ObjectId span_id) {
    ids.erase(std::remove(ids.begin(), ids.end(), span_id), ids.end());
  };
  auto clear_direct_derived_dirty = [&](ObjectId span_id) {
    const auto runtime_it = runtime_.span_runtime_states.find(span_id);
    if (runtime_it != runtime_.span_runtime_states.end()) {
      runtime_it->second.dirty_bits = runtime_it->second.dirty_bits & ~DirtyBits::kGeometryRefresh &
                                      ~DirtyBits::kBounds & ~DirtyBits::kRenderRefresh;
    }
    erase_id(runtime_.dirty_queue.geometry_dirty_span_ids, span_id);
    erase_id(runtime_.dirty_queue.bounds_dirty_span_ids, span_id);
    erase_id(runtime_.dirty_queue.render_dirty_span_ids, span_id);
  };
  for (ObjectId span_id : span_ids) {
    if (span_id == kInvalidObjectId) {
      continue;
    }
    if (runtime_.backbone_index.span_edge_bundle.find(span_id) == runtime_.backbone_index.span_edge_bundle.end()) {
      continue;
    }
    if (!runtime_.cache_state.span_layout_cache.rules_view(span_id).has_rule()) {
      out.error = "bb2 derive: saved span rule not found";
      return out;
    }
    const auto derived = DeriveGeneratedSpanOutputs(span_id);
    if (!derived.ok) {
      out.error = derived.error;
      return out;
    }
    clear_direct_derived_dirty(span_id);
  }
  out.value = true;
  out.ok = true;
  return out;
}

} // namespace wire::core
