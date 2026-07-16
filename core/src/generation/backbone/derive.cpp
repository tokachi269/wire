#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "curve_parts.hpp"
#include "derive_span_layout.hpp"
#include "model_assembly.hpp"
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
  SpanLayoutRule rule = *rule_view.rule;
  const SpanLayoutView current_layout = runtime_.cache_state.span_layout_cache.layout_view(span_id);
  auto hydrate_from_layout_endpoint = [](EndpointLayoutRule* endpoint, const LayoutEndpoint& source) {
    if (endpoint == nullptr || !UsesAuthoritativeGroupedLoweredSupport(source)) {
      return false;
    }
    CopyLayoutSemantic(endpoint->semantic, source);
    endpoint->side = source.side;
    endpoint->origin = source.origin;
    endpoint->order_decision_policy = source.order_decision_policy;
    endpoint->order_decision_choice = source.order_decision_choice;
    endpoint->order_decision_choice_reason = source.order_decision_choice_reason;
    endpoint->chosen_side = source.chosen_side;
    endpoint->used_junction_pair_side_assignment = source.used_junction_pair_side_assignment;
    return true;
  };
  auto hydrate_grouped_endpoint = [&](EndpointLayoutRule* endpoint, const LayoutEndpoint* current_endpoint) {
    if (endpoint == nullptr || !UsesAuthoritativeGroupedLoweredSupport(endpoint->semantic)) {
      return;
    }
    if (current_endpoint != nullptr && hydrate_from_layout_endpoint(endpoint, *current_endpoint)) {
      return;
    }
    const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint->semantic);
    const auto group_it = runtime_.cache_state.span_layout_cache.support_groups.decision.by_key.find(key);
    if (group_it == runtime_.cache_state.span_layout_cache.support_groups.decision.by_key.end()) {
      return;
    }
    const SupportGroupDecision& group = group_it->second;
    CopyLayoutSemantic(endpoint->semantic, group);
    endpoint->side = group.side;
    endpoint->origin = group.origin;
    endpoint->order_decision_policy = group.order_decision_policy;
    endpoint->order_decision_choice = group.order_decision_choice;
    endpoint->order_decision_choice_reason = group.order_decision_choice_reason;
    endpoint->chosen_side = group.chosen_side;
    endpoint->used_junction_pair_side_assignment = group.used_junction_pair_side_assignment;
  };
  hydrate_grouped_endpoint(&rule.start, current_layout.has_layout() ? &current_layout.entry->start : nullptr);
  hydrate_grouped_endpoint(&rule.end, current_layout.has_layout() ? &current_layout.entry->end : nullptr);
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
  SpanVisualCacheEntry visual = generation::backbone::visual(authoritative_.visual_settings, layout.value);
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
  SpanVisualCacheEntry visual = generation::backbone::visual(authoritative_.visual_settings, layout);
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
  SpanVisualCacheEntry visual = generation::backbone::visual(authoritative_.visual_settings, *layout_view.entry);
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
  if (plan.kind == UpdateKind::kReposition) {
    SupportGroupCache rebuilt_groups{};
    auto group_for = [&](const LoweredSupportGroupKey& key) -> std::pair<SupportGroupDecision*, LoweredSupportGroupPlacement*> {
      return {&rebuilt_groups.decision.by_key[key], &rebuilt_groups.placement.by_key[key]};
    };
    auto collect_group = [&](const LayoutEndpoint& endpoint) {
      if (!UsesAuthoritativeGroupedLoweredSupport(endpoint)) {
        return;
      }
      const LoweredSupportGroupKey key = LoweredSupportGroupKeyFromDecision(endpoint);
      auto [decision, placement] = group_for(key);
      CopyLayoutSemantic(*decision, endpoint);
      decision->side = endpoint.side;
      decision->origin = endpoint.origin;
      decision->order_decision_policy = endpoint.order_decision_policy;
      decision->order_decision_choice = endpoint.order_decision_choice;
      decision->order_decision_choice_reason = endpoint.order_decision_choice_reason;
      decision->chosen_side = endpoint.chosen_side;
      decision->used_junction_pair_side_assignment = endpoint.used_junction_pair_side_assignment;
      if (placement->attachment_worlds.empty()) {
        placement->grouped_port_count = 0;
      }
      placement->grouped_port_count += 1;
      placement->down_offset_m = std::max(placement->down_offset_m, endpoint.branch_down_offset_m);
      if (placement->attachment_worlds.empty()) {
        placement->mount_world = endpoint.support_world;
        placement->tip_world = endpoint.endpoint_world;
      }
      placement->attachment_worlds.push_back(endpoint.endpoint_world);
      const Port* endpoint_port = authoritative_.edit_state.ports.find(endpoint.port_id);
      const Pole* endpoint_pole =
          endpoint_port == nullptr ? nullptr : authoritative_.edit_state.poles.find(endpoint_port->owner_pole_id);
      if (endpoint_port != nullptr && endpoint_pole != nullptr && placement->down_offset_m > 1e-9) {
        const double support_z = view().port_category_base_z_for_pole(*endpoint_pole, endpoint_port->category) -
                                 placement->down_offset_m;
        placement->mount_world.z = support_z;
        placement->tip_world.z = support_z;
      }
    };
    runtime_.cache_state.span_layout_cache.for_each_layout_record(
        [&](ObjectId, const SpanLayoutCacheRecord&, const SpanLayoutEntry& layout) {
          collect_group(layout.start);
          collect_group(layout.end);
        });
    runtime_.cache_state.span_layout_cache.support_groups = std::move(rebuilt_groups);
  }
  if (plan.kind == UpdateKind::kReposition || plan.kind == UpdateKind::kReshape) {
    cache_visual_curve_parts(generation::backbone::make_visual_curve_parts(*this, {}, plan.affected.spans));
  }
  EditResult<VisualModelInstanceCache> model_instances =
      generation::backbone::materialize_model_assemblies(*this);
  if (!model_instances.ok) {
    timing.total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    debug_.last_update_timing = timing;
    out.error = model_instances.error;
    return out;
  }
  cache_visual_model_instances(std::move(model_instances.value));
  timing.total_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  debug_.last_update_timing = timing;
  out.value = true;
  out.ok = true;
  return out;
}

} // namespace wire::core
