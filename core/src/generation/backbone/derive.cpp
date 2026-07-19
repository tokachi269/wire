#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"

#include "curve_parts.hpp"
#include "derive_span_layout.hpp"
#include "model_assembly.hpp"
#include "out.hpp"
#include "../../support/instrumentation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

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
  auto hydrate_grouped_endpoint = [&](EndpointLayoutRule* endpoint) {
    if (endpoint == nullptr || !UsesAuthoritativeGroupedLoweredSupport(endpoint->semantic)) {
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
  hydrate_grouped_endpoint(&rule.start);
  hydrate_grouped_endpoint(&rule.end);
  const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
  std::vector<SpanLayoutRule> plan_rules{};
  plan_rules.reserve(runtime_.cache_state.span_layout_cache.records_by_span.size());
  bool found_plan_rule = false;
  std::vector<ObjectId> cached_span_ids{};
  cached_span_ids.reserve(runtime_.cache_state.span_layout_cache.records_by_span.size());
  for (const auto& [cached_span_id, record] : runtime_.cache_state.span_layout_cache.records_by_span) {
    const SpanLayoutRule* cached_rule = record.span_layout_rule();
    if (cached_rule == nullptr) {
      continue;
    }
    cached_span_ids.push_back(cached_span_id);
  }
  std::sort(cached_span_ids.begin(), cached_span_ids.end());
  for (ObjectId cached_span_id : cached_span_ids) {
    const auto record_it = runtime_.cache_state.span_layout_cache.records_by_span.find(cached_span_id);
    if (record_it == runtime_.cache_state.span_layout_cache.records_by_span.end()) {
      continue;
    }
    const SpanLayoutRule* cached_rule = record_it->second.span_layout_rule();
    if (cached_rule == nullptr) {
      continue;
    }
    SpanLayoutRule plan_rule = cached_span_id == span_id ? rule : *cached_rule;
    hydrate_grouped_endpoint(&plan_rule.start);
    hydrate_grouped_endpoint(&plan_rule.end);
    found_plan_rule = found_plan_rule || cached_span_id == span_id;
    plan_rules.push_back(std::move(plan_rule));
  }
  if (!found_plan_rule) {
    plan_rules.push_back(rule);
  }
  EditResult<generation::backbone::FixturePlacementPlanByPort> fixture_plan =
      generation::backbone::fixture_placement_plan_from_rules(*this, plan_rules);
  if (!fixture_plan.ok) {
    out.error = fixture_plan.error;
    return out;
  }
  const auto endpoint_resolver = [&](const EndpointLayoutRule& endpoint) {
    return generation::backbone::resolve_span_layout_endpoint(
        *this, authoritative_.edit_state, endpoint, &fixture_plan.value);
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

  std::vector<SpanLayoutRule> reposition_rules{};
  generation::backbone::FixturePlacementPlanByPort reposition_fixture_plan{};
  const auto hydrate_endpoint_from_decision = [&](EndpointLayoutRule* endpoint) {
    if (endpoint == nullptr || !UsesAuthoritativeGroupedLoweredSupport(endpoint->semantic)) {
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
  const auto build_reposition_context = [&]() -> EditResult<bool> {
    EditResult<bool> built{};
    reposition_rules.reserve(runtime_.cache_state.span_layout_cache.records_by_span.size());
    std::vector<ObjectId> cached_span_ids{};
    cached_span_ids.reserve(runtime_.cache_state.span_layout_cache.records_by_span.size());
    for (const auto& [span_id, record] : runtime_.cache_state.span_layout_cache.records_by_span) {
      const SpanLayoutRule* cached_rule = record.span_layout_rule();
      if (cached_rule == nullptr) {
        continue;
      }
      cached_span_ids.push_back(span_id);
    }
    std::sort(cached_span_ids.begin(), cached_span_ids.end());
    for (ObjectId cached_span_id : cached_span_ids) {
      const auto record_it = runtime_.cache_state.span_layout_cache.records_by_span.find(cached_span_id);
      if (record_it == runtime_.cache_state.span_layout_cache.records_by_span.end()) {
        continue;
      }
      const SpanLayoutRule* cached_rule = record_it->second.span_layout_rule();
      if (cached_rule == nullptr) {
        continue;
      }
      SpanLayoutRule rule = *cached_rule;
      hydrate_endpoint_from_decision(&rule.start);
      hydrate_endpoint_from_decision(&rule.end);
      reposition_rules.push_back(std::move(rule));
    }
    EditResult<generation::backbone::FixturePlacementPlanByPort> fixture_plan =
        generation::backbone::fixture_placement_plan_from_rules(*this, reposition_rules);
    if (!fixture_plan.ok) {
      built.error = fixture_plan.error;
      return built;
    }
    reposition_fixture_plan = std::move(fixture_plan.value);
    built.value = true;
    built.ok = true;
    return built;
  };
  std::unordered_map<ObjectId, const SpanLayoutRule*> reposition_rules_by_span{};
  if (plan.kind == UpdateKind::kReposition) {
    EditResult<bool> context = build_reposition_context();
    if (!context.ok) {
      timing.total_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
      debug_.last_update_timing = timing;
      out.error = context.error;
      return out;
    }
    reposition_rules_by_span.reserve(reposition_rules.size());
    for (const SpanLayoutRule& rule : reposition_rules) {
      reposition_rules_by_span.emplace(rule.span_id, &rule);
    }
  }

  const auto derive_reposition_span = [&](ObjectId span_id) -> EditResult<bool> {
    EditResult<bool> derived{};
    const Span* span = authoritative_.edit_state.spans.find(span_id);
    if (span == nullptr) {
      derived.error = "backbone derive: span not found";
      return derived;
    }
    const auto rule_it = reposition_rules_by_span.find(span_id);
    if (rule_it == reposition_rules_by_span.end() || rule_it->second == nullptr) {
      derived.error = "backbone derive: span layout rule not found";
      return derived;
    }
    const SpanLayoutRule* rule = rule_it->second;
    const SpanRuntimeState* runtime = find_span_runtime_state(span_id);
    const auto endpoint_resolver = [&](const EndpointLayoutRule& endpoint) {
      return generation::backbone::resolve_span_layout_endpoint(
          *this, authoritative_.edit_state, endpoint, &reposition_fixture_plan);
    };
    EditResult<SpanLayoutEntry> layout = generation::backbone::derive_span_layout(
        *rule, endpoint_resolver, (runtime == nullptr) ? 0 : runtime->data_version);
    if (!layout.ok) {
      derived.error = layout.error;
      return derived;
    }
    EditResult<DetailCurve> curve = generation::backbone::make_curve(*this, span_id, layout.value);
    if (!curve.ok) {
      derived.error = curve.error;
      return derived;
    }
    BoundsCacheEntry bounds = generation::backbone::bounds(curve.value, layout.value.source_version);
    SpanVisualCacheEntry visual = generation::backbone::visual(authoritative_.visual_settings, layout.value);
    SpanRenderCacheEntry render = generation::backbone::render(*this, span_id, curve.value);
    cache_span_layout(std::move(layout.value));
    cache_span_curve(span_id, std::move(curve.value));
    cache_span_bounds(span_id, std::move(bounds));
    cache_span_visual(span_id, std::move(visual));
    cache_span_render(span_id, std::move(render));
    derived.value = true;
    derived.ok = true;
    return derived;
  };

  for (ObjectId span_id : plan.affected.spans) {
    if (span_id == kInvalidObjectId) {
      continue;
    }
    EditResult<bool> derived{};
    instrumentation::count_affected_span_derive();
    const auto derive_started = std::chrono::steady_clock::now();
    switch (plan.kind) {
    case UpdateKind::kReposition:
      derived = derive_reposition_span(span_id);
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
    instrumentation::count_group_cache_refresh();
    auto rebuild_groups_from_rules = [&]() -> EditResult<SupportGroupCache> {
      EditResult<SupportGroupCache> rebuilt{};
      std::vector<std::pair<LoweredSupportGroupKey, ObjectId>> representative_port_by_key{};
      auto group_for = [&](const LoweredSupportGroupKey& key) ->
          std::pair<SupportGroupDecision*, LoweredSupportGroupPlacement*> {
        return {&rebuilt.value.decision.by_key[key], &rebuilt.value.placement.by_key[key]};
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
        auto representative_it =
            std::find_if(representative_port_by_key.begin(), representative_port_by_key.end(),
                         [&](const auto& item) { return item.first == key; });
        if (representative_it == representative_port_by_key.end()) {
          representative_port_by_key.push_back({key, endpoint.port_id});
          representative_it = representative_port_by_key.end() - 1;
        }
        if (endpoint.port_id <= representative_it->second) {
          representative_it->second = endpoint.port_id;
          placement->mount_world = endpoint.support_world;
          placement->tip_world = endpoint.endpoint_world;
        }
        placement->attachment_worlds.push_back(endpoint.endpoint_world);
      };
      const auto endpoint_resolver = [&](const EndpointLayoutRule& endpoint) {
        return generation::backbone::resolve_span_layout_endpoint(
            *this, authoritative_.edit_state, endpoint, &reposition_fixture_plan);
      };
      for (const SpanLayoutRule& rule : reposition_rules) {
        const SpanRuntimeState* runtime = find_span_runtime_state(rule.span_id);
        EditResult<SpanLayoutEntry> layout = generation::backbone::derive_span_layout(
            rule, endpoint_resolver, (runtime == nullptr) ? 0 : runtime->data_version);
        if (!layout.ok) {
          rebuilt.error = layout.error;
          return rebuilt;
        }
        collect_group(layout.value.start);
        collect_group(layout.value.end);
      }
      rebuilt.ok = true;
      return rebuilt;
    };
    EditResult<SupportGroupCache> rebuilt_groups = rebuild_groups_from_rules();
    if (!rebuilt_groups.ok) {
      timing.total_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
      debug_.last_update_timing = timing;
      out.error = rebuilt_groups.error;
      return out;
    }
    runtime_.cache_state.span_layout_cache.support_groups = std::move(rebuilt_groups.value);
  }
  if (plan.kind == UpdateKind::kReposition || plan.kind == UpdateKind::kReshape) {
    EditResult<VisualCurvePartCache> visual_curves =
        generation::backbone::make_visual_curve_parts(*this, {}, plan.affected.spans);
    if (!visual_curves.ok) {
      timing.total_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
      debug_.last_update_timing = timing;
      out.error = visual_curves.error;
      return out;
    }
    cache_visual_curve_parts(std::move(visual_curves.value));
  }
  generation::backbone::FixturePlacementPlanByPort model_fixture_plan{};
  if (plan.kind == UpdateKind::kReposition) {
    model_fixture_plan = std::move(reposition_fixture_plan);
  } else {
    EditResult<generation::backbone::FixturePlacementPlanByPort> fixture_plan =
        generation::backbone::fixture_placement_plan_from_cache(*this);
    if (!fixture_plan.ok) {
      timing.total_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
      debug_.last_update_timing = timing;
      out.error = fixture_plan.error;
      return out;
    }
    model_fixture_plan = std::move(fixture_plan.value);
  }
  EditResult<VisualModelInstanceCache> model_instances =
      generation::backbone::materialize_model_assemblies(*this, model_fixture_plan);
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
