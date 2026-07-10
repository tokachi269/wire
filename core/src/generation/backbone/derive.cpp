#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "curve_parts.hpp"
#include "derive_span_layout.hpp"
#include "out.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace wire::core {

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
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  const auto endpoint_resolver = [&](const EndpointLayoutRule& endpoint) {
    return generation::backbone::resolve_span_layout_endpoint(*this, authoritative_.edit_state, endpoint);
  };
  EditResult<SpanLayoutEntry> layout = generation::backbone::derive_span_layout(
      rule, endpoint_resolver, (runtime == nullptr) ? 0 : runtime->data_version);
  if (!layout.ok) {
    out.error = layout.error;
    return out;
  }

  EditResult<DetailCurve> curve = generation::backbone::make_curve(*this, span_id, layout.value);
  if (!curve.ok) {
    out.error = curve.error;
    return out;
  }
  BoundsCacheEntry bounds = generation::backbone::bounds(curve.value, layout.value.source_version);
  SpanVisualCacheEntry visual = generation::backbone::visual(runtime_.cache_state.visual_settings, layout.value);
  SpanRenderCacheEntry render = generation::backbone::render(*this, span_id, curve.value);

  cache_span_layout(std::move(layout.value));
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
    cache_visual_curve_parts(generation::backbone::make_visual_curve_parts(*this, {}, plan.affected.spans));
  }
  timing.total_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  debug_.last_update_timing = timing;
  out.value = true;
  out.ok = true;
  return out;
}

} // namespace wire::core
