#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "curve_parts.hpp"
#include "out.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>
#include <unordered_set>

namespace wire::core {
namespace {

bool apply_endpoint(const EditState& edit_state, const EndpointLayoutRule& rule, LayoutEndpoint* target,
                    const CoreState& state, std::string* error) {
  const Port* port = edit_state.ports.find(rule.port_id);
  if (port == nullptr || target == nullptr) {
    if (error != nullptr) {
      *error = "backbone derive: endpoint port not found";
    }
    return false;
  }
  Vec3d endpoint_world = port->world_position;
  if (rule.source_projection.valid()) {
    const std::optional<Vec3d> projection = generation::backbone::source_edge_projection_world(state, rule.source_projection);
    if (!projection.has_value()) {
      if (error != nullptr) {
        *error = "backbone derive: source edge projection missing";
      }
      return false;
    }
    endpoint_world = *projection;
  }
  ApplyEndpointLayoutRule(*target, rule, endpoint_world);
  return true;
}

} // namespace

EditResult<bool> CoreState::DeriveGeneratedSpanOutputs(ObjectId span_id) {
  EditResult<bool> out{};
  const Span* span = authoritative_.edit_state.spans.find(span_id);
  if (span == nullptr) {
    out.error = "backbone derive: span not found";
    return out;
  }
  const SpanLayoutRulesView rule_view = runtime_.cache_state.span_layout_cache.rules_view(span_id);
  if (!rule_view.has_rule()) {
    out.error = "backbone derive: span layout rule not found";
    return out;
  }
  const SpanLayoutRule& rule = *rule_view.rule;
  SpanLayoutEntry layout{};
  layout.span_id = rule.span_id;
  layout.flow_kind = rule.flow_kind;
  layout.pass_mode = rule.pass_mode;
  layout.variation_flow_key = rule.variation_flow_key;
  layout.lowering_kind = rule.lowering_kind;
  if (!apply_endpoint(authoritative_.edit_state, rule.start, &layout.start, *this, &out.error) ||
      !apply_endpoint(authoritative_.edit_state, rule.end, &layout.end, *this, &out.error)) {
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
  layout.basis_length_m = Length(chord);
  layout.effective_sag_ratio = 0.0;
  layout.continuity_preference = CableContinuityPolicyHint::kAuto;
  layout.bend_stiffness_hint = 1.0;
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  layout.source_version = (runtime == nullptr) ? 0 : runtime->data_version;

  EditResult<DetailCurve> curve = generation::backbone::make_curve(*this, span_id, layout);
  if (!curve.ok) {
    out.error = curve.error;
    return out;
  }
  BoundsCacheEntry bounds = generation::backbone::bounds(curve.value, layout.source_version);
  SpanVisualCacheEntry visual = generation::backbone::visual(runtime_.cache_state.visual_settings, layout);
  SpanRenderCacheEntry render = generation::backbone::render(*this, span_id, curve.value);

  cache_span_layout(std::move(layout));
  cache_span_curve(span_id, std::move(curve.value));
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
    out.error = "backbone update: span not found";
    return out;
  }
  const SpanLayoutView layout_view = runtime_.cache_state.span_layout_cache.layout_view(span_id);
  if (!layout_view.has_layout()) {
    out.error = "backbone update: span layout not found";
    return out;
  }
  const SpanLayoutEntry& layout = *layout_view.entry;
  EditResult<DetailCurve> curve = generation::backbone::make_curve(*this, span_id, layout);
  if (!curve.ok) {
    out.error = curve.error;
    return out;
  }
  BoundsCacheEntry bounds = generation::backbone::bounds(curve.value, layout.source_version);
  SpanVisualCacheEntry visual = generation::backbone::visual(runtime_.cache_state.visual_settings, layout);
  SpanRenderCacheEntry render = generation::backbone::render(*this, span_id, curve.value);
  cache_span_curve(span_id, std::move(curve.value));
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
    out.error = "backbone update: span not found";
    return out;
  }
  const SpanLayoutView layout_view = runtime_.cache_state.span_layout_cache.layout_view(span_id);
  if (!layout_view.has_layout()) {
    out.error = "backbone update: span layout not found";
    return out;
  }
  const CurveCacheEntry* curve = find_curve_cache(span_id);
  if (curve == nullptr) {
    out.error = "backbone update: span curve not found";
    return out;
  }
  SpanVisualCacheEntry visual = generation::backbone::visual(runtime_.cache_state.visual_settings, *layout_view.entry);
  SpanRenderCacheEntry render = generation::backbone::render(*this, span_id, curve->detail);
  cache_span_visual(span_id, std::move(visual));
  cache_span_render(span_id, std::move(render));
  out.value = true;
  out.ok = true;
  return out;
}

EditResult<bool> CoreState::execute_update_plan(const UpdatePlan& plan) {
  const auto started = std::chrono::steady_clock::now();
  UpdateTiming timing{};
  timing.kind = plan.kind;
  timing.affected_span_count = plan.affected.spans.size();
  timing.plan_ms = plan.plan_ms;
  EditResult<bool> out{};
  if (plan.kind == UpdateKind::kRegenerate) {
    timing.total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    debug_.last_update_timing = timing;
    out.error = "backbone unsupported: route-local regenerate is not implemented";
    return out;
  }
  for (ObjectId span_id : plan.affected.spans) {
    if (span_id == kInvalidObjectId) {
      continue;
    }
    EditResult<bool> derived{};
    const auto derive_started = std::chrono::steady_clock::now();
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
    timing.derive_ms +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - derive_started).count();
    if (!derived.ok) {
      timing.total_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
      debug_.last_update_timing = timing;
      out.error = derived.error;
      return out;
    }
  }
  if (plan.kind == UpdateKind::kReposition || plan.kind == UpdateKind::kReshape) {
    cache_visual_curve_parts(generation::backbone::make_visual_curve_parts(*this, {}));
  }
  timing.total_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  debug_.last_update_timing = timing;
  out.value = true;
  out.ok = true;
  return out;
}

} // namespace wire::core
