#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "out.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace wire::core {
namespace {

double length(Vec3d v) {
  return std::sqrt(LengthSquared(v));
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

  DetailCurve curve = generation::bb2::make_curve(*this, span_id, layout);
  BoundsCacheEntry bounds = generation::bb2::bounds(curve, layout.source_version);
  SpanVisualCacheEntry visual = generation::bb2::visual(runtime_.cache_state.visual_settings, layout);
  SpanRenderCacheEntry render = generation::bb2::render(*this, span_id, curve);

  cache_span_layout(std::move(layout));
  cache_span_curve(span_id, std::move(curve));
  cache_span_bounds(span_id, std::move(bounds));
  cache_span_visual(span_id, std::move(visual));
  cache_span_render(span_id, std::move(render));
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> CoreState::derive_generated_span_shape_outputs(ObjectId span_id) {
  EditResult<bool> out{};
  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    out.error = "bb2 update: span not found";
    return out;
  }
  const SpanLayoutView layout_view = runtime_.cache_state.span_layout_cache.layout_view(span_id);
  if (!layout_view.has_layout()) {
    out.error = "bb2 update: span layout not found";
    return out;
  }
  const SpanLayoutEntry& layout = *layout_view.entry;
  DetailCurve curve = generation::bb2::make_curve(*this, span_id, layout);
  BoundsCacheEntry bounds = generation::bb2::bounds(curve, layout.source_version);
  SpanVisualCacheEntry visual = generation::bb2::visual(runtime_.cache_state.visual_settings, layout);
  SpanRenderCacheEntry render = generation::bb2::render(*this, span_id, curve);
  cache_span_curve(span_id, std::move(curve));
  cache_span_bounds(span_id, std::move(bounds));
  cache_span_visual(span_id, std::move(visual));
  cache_span_render(span_id, std::move(render));
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> CoreState::derive_generated_span_draw_outputs(ObjectId span_id) {
  EditResult<bool> out{};
  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    out.error = "bb2 update: span not found";
    return out;
  }
  const SpanLayoutView layout_view = runtime_.cache_state.span_layout_cache.layout_view(span_id);
  if (!layout_view.has_layout()) {
    out.error = "bb2 update: span layout not found";
    return out;
  }
  const CurveCacheEntry* curve = find_curve_cache(span_id);
  if (curve == nullptr) {
    out.error = "bb2 update: span curve not found";
    return out;
  }
  SpanVisualCacheEntry visual = generation::bb2::visual(runtime_.cache_state.visual_settings, *layout_view.entry);
  SpanRenderCacheEntry render = generation::bb2::render(*this, span_id, curve->detail);
  cache_span_visual(span_id, std::move(visual));
  cache_span_render(span_id, std::move(render));
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> CoreState::execute_update_plan(const UpdatePlan& plan) {
  EditResult<bool> out{};
  if (plan.kind == UpdateKind::kRegenerate) {
    out.error = "bb2 update: regenerate is not implemented for local updates";
    return out;
  }
  for (ObjectId span_id : plan.affected.spans) {
    if (span_id == kInvalidObjectId) {
      continue;
    }
    EditResult<bool> derived{};
    switch (plan.kind) {
    case UpdateKind::kReposition:
      derived = DeriveGeneratedSpanOutputs(span_id);
      break;
    case UpdateKind::kReshape:
      derived = derive_generated_span_shape_outputs(span_id);
      break;
    case UpdateKind::kRedraw:
      derived = derive_generated_span_draw_outputs(span_id);
      break;
    case UpdateKind::kRegenerate:
      break;
    }
    if (!derived.ok) {
      out.error = derived.error;
      return out;
    }
  }
  out.value = true;
  out.ok = true;
  return out;
}

} // namespace wire::core
