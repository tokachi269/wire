#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace helpers;

namespace backbone_tests {

bool C611_bb2_direct_derive_restores_saved_span_outputs() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId span_id = out.value.generated_span_ids.front();
  if (!state.span_layout_rules(span_id).has_rule()) {
    return false;
  }
  wire::core::CacheState& cache = wire::core::CoreStateTestHook::cache_state(state);
  cache.span_layout_cache.clear_layout(span_id);
  cache.curve_cache.by_span.erase(span_id);
  cache.bounds_cache.by_span.erase(span_id);
  cache.visual_cache.by_span.erase(span_id);
  cache.render_cache.by_span.erase(span_id);
  if (state.span_layout(span_id).has_layout() || state.find_curve_cache(span_id) != nullptr ||
      state.find_bounds_cache(span_id) != nullptr || state.find_span_visual_cache(span_id) != nullptr ||
      state.find_span_render_cache(span_id) != nullptr) {
    return false;
  }
  const auto derived = state.DeriveGeneratedSpanOutputs(span_id);
  return derived.ok && state.span_layout(span_id).has_layout() && state.find_curve_cache(span_id) != nullptr &&
         state.find_bounds_cache(span_id) != nullptr && state.find_span_visual_cache(span_id) != nullptr &&
         state.find_span_render_cache(span_id) != nullptr;
}

bool C613_bb2_port_edit_rederives_generated_span_without_recalc() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId span_id = out.value.generated_span_ids.front();
  const wire::core::Span* span = state.view().spans().find(span_id);
  if (span == nullptr) {
    return false;
  }
  const wire::core::Port* port = state.view().ports().find(span->port_a_id);
  if (port == nullptr) {
    return false;
  }
  const wire::core::Vec3d moved{port->world_position.x, port->world_position.y + 1.25, port->world_position.z};
  const auto edit = state.SetPortWorldPositionManual(port->id, moved);
  if (!edit.ok) {
    return false;
  }
  const wire::core::SpanLayoutView layout = state.span_layout(span_id);
  const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  const wire::core::BoundsCacheEntry* bounds = state.find_bounds_cache(span_id);
  const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
  const wire::core::SpanRenderCacheEntry* render = state.find_span_render_cache(span_id);
  if (!layout.has_layout() || curve == nullptr || bounds == nullptr || visual == nullptr || render == nullptr ||
      curve->detail.sample_points.empty()) {
    return false;
  }
  const bool layout_moved = almost_equal(layout.entry->start.support_world.x, moved.x, 1e-9) &&
                            almost_equal(layout.entry->start.support_world.y, moved.y, 1e-9) &&
                            almost_equal(layout.entry->start.support_world.z, moved.z, 1e-9);
  const bool curve_moved = almost_equal(curve->detail.sample_points.front().x, layout.entry->start.endpoint_world.x, 1e-9) &&
                           almost_equal(curve->detail.sample_points.front().y, layout.entry->start.endpoint_world.y, 1e-9) &&
                           almost_equal(curve->detail.sample_points.front().z, layout.entry->start.endpoint_world.z, 1e-9);
  return layout_moved && curve_moved;
}

bool C614_bb2_update_plan_uses_coarse_kinds() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_pole_ids.empty() || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId span_id = out.value.generated_span_ids.front();
  const wire::core::Span* span = state.view().spans().find(span_id);
  if (span == nullptr) {
    return false;
  }
  const auto pole_plan = wire::core::CoreStateTestHook::make_update_plan(
      state, {wire::core::UpdateKind::kReposition, wire::core::UpdateTargetKind::kPole,
              out.value.generated_pole_ids.front()});
  const auto port_plan = wire::core::CoreStateTestHook::make_update_plan(
      state, {wire::core::UpdateKind::kReposition, wire::core::UpdateTargetKind::kPort, span->port_a_id});
  const auto reshape_plan = wire::core::CoreStateTestHook::make_update_plan(
      state, {wire::core::UpdateKind::kReshape, wire::core::UpdateTargetKind::kAllSpans,
              wire::core::kInvalidObjectId});
  const auto redraw_plan = wire::core::CoreStateTestHook::make_update_plan(
      state, {wire::core::UpdateKind::kRedraw, wire::core::UpdateTargetKind::kAllSpans,
              wire::core::kInvalidObjectId});
  const auto regen_plan = wire::core::CoreStateTestHook::make_update_plan(
      state, {wire::core::UpdateKind::kRegenerate, wire::core::UpdateTargetKind::kSpan, span_id});
  return pole_plan.ok && pole_plan.value.kind == wire::core::UpdateKind::kReposition &&
         contains_id(pole_plan.value.affected.spans, span_id) && port_plan.ok &&
         port_plan.value.kind == wire::core::UpdateKind::kReposition &&
         contains_id(port_plan.value.affected.spans, span_id) && reshape_plan.ok &&
         reshape_plan.value.kind == wire::core::UpdateKind::kReshape &&
         contains_id(reshape_plan.value.affected.spans, span_id) && redraw_plan.ok &&
         redraw_plan.value.kind == wire::core::UpdateKind::kRedraw &&
         contains_id(redraw_plan.value.affected.spans, span_id) && regen_plan.ok &&
         regen_plan.value.kind == wire::core::UpdateKind::kRegenerate;
}

bool C615_bb2_regenerate_plan_is_not_local_fallback() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const auto plan = wire::core::CoreStateTestHook::make_update_plan(
      state, {wire::core::UpdateKind::kRegenerate, wire::core::UpdateTargetKind::kSpan,
              out.value.generated_span_ids.front()});
  if (!plan.ok) {
    return false;
  }
  const auto executed = wire::core::CoreStateTestHook::execute_update_plan(state, plan.value);
  return !executed.ok;
}

bool C616_bb2_reposition_keeps_saved_graph_identity() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  std::vector<wire::core::ObjectId> before_nodes{};
  std::vector<wire::core::ObjectId> before_edges{};
  std::vector<wire::core::ObjectId> before_edge_bundles{};
  std::vector<wire::core::ObjectId> before_span_bindings{};
  for (const wire::core::SavedBackboneNode& node : state.view().backbone().nodes) {
    before_nodes.push_back(node.node_id);
  }
  for (const wire::core::SavedBackboneEdge& edge : state.view().backbone().edges) {
    before_edges.push_back(edge.edge_id);
  }
  for (const wire::core::SavedBackboneEdgeBundle& edge_bundle : state.view().backbone().edge_bundles) {
    before_edge_bundles.push_back(edge_bundle.edge_bundle_id);
  }
  for (const wire::core::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    before_span_bindings.push_back(binding.span_id);
  }
  const wire::core::ObjectId span_id = out.value.generated_span_ids.front();
  const wire::core::Span* span = state.view().spans().find(span_id);
  if (span == nullptr) {
    return false;
  }
  const wire::core::Port* port = state.view().ports().find(span->port_a_id);
  if (port == nullptr) {
    return false;
  }
  const auto edited =
      state.SetPortWorldPositionManual(port->id, {port->world_position.x, port->world_position.y + 0.75,
                                                  port->world_position.z});
  std::vector<wire::core::ObjectId> after_nodes{};
  std::vector<wire::core::ObjectId> after_edges{};
  std::vector<wire::core::ObjectId> after_edge_bundles{};
  std::vector<wire::core::ObjectId> after_span_bindings{};
  for (const wire::core::SavedBackboneNode& node : state.view().backbone().nodes) {
    after_nodes.push_back(node.node_id);
  }
  for (const wire::core::SavedBackboneEdge& edge : state.view().backbone().edges) {
    after_edges.push_back(edge.edge_id);
  }
  for (const wire::core::SavedBackboneEdgeBundle& edge_bundle : state.view().backbone().edge_bundles) {
    after_edge_bundles.push_back(edge_bundle.edge_bundle_id);
  }
  for (const wire::core::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    after_span_bindings.push_back(binding.span_id);
  }
  return edited.ok && before_nodes == after_nodes && before_edges == after_edges &&
         before_edge_bundles == after_edge_bundles && before_span_bindings == after_span_bindings;
}

bool C619_bb2_reposition_updates_only_affected_spans() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok || out.value.generated_span_ids.size() < 2) {
    return false;
  }
  const wire::core::ObjectId first_span_id = out.value.generated_span_ids.front();
  const wire::core::ObjectId second_span_id = out.value.generated_span_ids.back();
  const wire::core::Span* first_span = state.view().spans().find(first_span_id);
  if (first_span == nullptr) {
    return false;
  }
  const wire::core::Port* first_port = state.view().ports().find(first_span->port_a_id);
  const wire::core::SpanLayoutView second_before_view = state.span_layout(second_span_id);
  if (first_port == nullptr || !second_before_view.has_layout()) {
    return false;
  }
  const double old_first_y = first_port->world_position.y;
  const wire::core::SpanLayoutEntry second_before = *second_before_view.entry;
  const auto edited =
      state.SetPortWorldPositionManual(first_port->id, {first_port->world_position.x, first_port->world_position.y + 1.0,
                                                        first_port->world_position.z});
  const wire::core::SpanLayoutView first_after = state.span_layout(first_span_id);
  const wire::core::SpanLayoutView second_after = state.span_layout(second_span_id);
  return edited.ok && first_after.has_layout() && second_after.has_layout() &&
         almost_equal(first_after.entry->start.support_world.y, old_first_y + 1.0, 1e-9) &&
         second_before.source_version == second_after.entry->source_version;
}

bool C621_bb2_sag_reshape_updates_geom_only() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId span_id = out.value.generated_span_ids.front();
  const wire::core::SpanLayoutView before_layout_view = state.span_layout(span_id);
  if (!before_layout_view.has_layout()) {
    return false;
  }
  const wire::core::SpanLayoutEntry before_layout = *before_layout_view.entry;
  std::vector<wire::core::ObjectId> node_ids{};
  std::vector<wire::core::ObjectId> edge_ids{};
  std::vector<wire::core::ObjectId> binding_spans{};
  for (const wire::core::SavedBackboneNode& node : state.view().backbone().nodes) {
    node_ids.push_back(node.node_id);
  }
  for (const wire::core::SavedBackboneEdge& edge : state.view().backbone().edges) {
    edge_ids.push_back(edge.edge_id);
  }
  for (const wire::core::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    binding_spans.push_back(binding.span_id);
  }

  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  settings.sag_factor = std::max(0.03, settings.sag_factor);
  settings.curve_samples = std::max(9, settings.curve_samples);
  const auto updated = state.UpdateGeometrySettings(settings);
  const wire::core::SpanLayoutView after_layout = state.span_layout(span_id);
  const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  const wire::core::BoundsCacheEntry* bounds = state.find_bounds_cache(span_id);
  if (!updated.ok || !after_layout.has_layout() || curve == nullptr || bounds == nullptr ||
      curve->detail.sag_amplitude_m <= 0.0 || curve->detail.sample_points.size() < 3) {
    return false;
  }
  const wire::core::Vec3d start = curve->detail.EvaluatePosition(0.0);
  const wire::core::Vec3d end = curve->detail.EvaluatePosition(1.0);
  const double endpoint_min_z =
      std::min(before_layout.start.endpoint_world.z, before_layout.end.endpoint_world.z);
  if (!almost_equal(start, before_layout.start.endpoint_world, 1e-9) ||
      !almost_equal(end, before_layout.end.endpoint_world, 1e-9) ||
      !(bounds->whole.min.z < endpoint_min_z)) {
    return false;
  }

  std::vector<wire::core::ObjectId> after_node_ids{};
  std::vector<wire::core::ObjectId> after_edge_ids{};
  std::vector<wire::core::ObjectId> after_binding_spans{};
  for (const wire::core::SavedBackboneNode& node : state.view().backbone().nodes) {
    after_node_ids.push_back(node.node_id);
  }
  for (const wire::core::SavedBackboneEdge& edge : state.view().backbone().edges) {
    after_edge_ids.push_back(edge.edge_id);
  }
  for (const wire::core::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    after_binding_spans.push_back(binding.span_id);
  }
  return almost_equal(after_layout.entry->start.endpoint_world, before_layout.start.endpoint_world, 1e-9) &&
         almost_equal(after_layout.entry->end.endpoint_world, before_layout.end.endpoint_world, 1e-9) &&
         after_layout.entry->source_version == before_layout.source_version && node_ids == after_node_ids &&
         edge_ids == after_edge_ids && binding_spans == after_binding_spans;
}

bool C623_bb2_layout_settings_reject_before_mutation() {
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok) return false;
  const wire::core::LayoutSettings before = state.view().layout_settings();
  wire::core::LayoutSettings edited = before;
  edited.corner_threshold_deg = std::max(1.0, before.corner_threshold_deg - 1.0);
  const auto updated = state.UpdateLayoutSettings(edited);
  return !updated.ok && contains_text(updated.error, "unsupported") &&
         almost_equal(state.view().layout_settings().corner_threshold_deg, before.corner_threshold_deg, 1e-12);
}

bool C624_bb2_variation_settings_reject_before_mutation() {
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok) return false;
  const wire::core::VariationSettings before = state.view().variation_settings();
  wire::core::VariationSettings edited = before;
  edited.enabled = !before.enabled;
  const auto updated = state.UpdateVariationSettings(edited);
  return !updated.ok && contains_text(updated.error, "unsupported") &&
         state.view().variation_settings().enabled == before.enabled;
}

bool C625_bb2_context_profile_reject_before_mutation() {
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok) return false;
  const wire::core::ContextProfile before = state.view().context_profile();
  wire::core::ContextProfile edited = before;
  edited.age = std::clamp(before.age + 0.1, 0.0, 1.0);
  if (almost_equal(edited.age, before.age, 1e-12)) edited.age = std::max(0.0, before.age - 0.1);
  const auto updated = state.UpdateContextProfile(edited);
  return !updated.ok && contains_text(updated.error, "unsupported") &&
         almost_equal(state.view().context_profile().age, before.age, 1e-12);
}

bool C626_bb2_cable_template_updates_derive_outputs() {
  wire::core::CoreState state;
  wire::core::GeometrySettings geometry = state.view().geometry_settings();
  geometry.sag_enabled = true;
  geometry.curve_samples = std::max(9, geometry.curve_samples);
  if (!state.UpdateGeometrySettings(geometry).ok) return false;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.empty()) return false;
  const wire::core::ObjectId span_id = generated.value.generated_span_ids.front();
  const wire::core::Span* span = state.view().spans().find(span_id);
  const wire::core::Bundle* bundle =
      span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
  if (bundle == nullptr) return false;
  const auto bundle_template = state.view().bundle_templates().find(bundle->bundle_template_id);
  if (bundle_template == state.view().bundle_templates().end()) return false;
  const auto cable_it = state.view().cable_templates().find(bundle_template->second.cable_template_id);
  const wire::core::SpanLayoutView before_layout = state.span_layout(span_id);
  const wire::core::CurveCacheEntry* before_curve = state.find_curve_cache(span_id);
  const wire::core::SpanRenderCacheEntry* before_render = state.find_span_render_cache(span_id);
  if (cable_it == state.view().cable_templates().end() || !before_layout.has_layout() ||
      before_curve == nullptr || before_render == nullptr) return false;
  const wire::core::SpanLayoutEntry layout_copy = *before_layout.entry;
  const double sag_before = before_curve->detail.sag_amplitude_m;

  wire::core::CableTemplate reshape = cable_it->second;
  reshape.sag_factor += 0.02;
  const auto reshaped = state.UpdateCableTemplate(reshape);
  const wire::core::CurveCacheEntry* reshaped_curve = state.find_curve_cache(span_id);
  if (!reshaped.ok || reshaped_curve == nullptr ||
      !(reshaped_curve->detail.sag_amplitude_m > sag_before) ||
      !almost_equal(state.span_layout(span_id).entry->start.endpoint_world, layout_copy.start.endpoint_world, 1e-9)) {
    return false;
  }

  wire::core::CableTemplate redraw = state.view().cable_templates().at(reshape.id);
  redraw.color_rgba ^= 0x000000FFu;
  const std::vector<wire::core::Vec3d> samples_after_reshape = reshaped_curve->detail.sample_points;
  const auto redrawn = state.UpdateCableTemplate(redraw);
  const wire::core::CurveCacheEntry* final_curve = state.find_curve_cache(span_id);
  const wire::core::SpanRenderCacheEntry* final_render = state.find_span_render_cache(span_id);
  const auto same_points = [](const std::vector<wire::core::Vec3d>& a,
                              const std::vector<wire::core::Vec3d>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
      if (!almost_equal(a[i], b[i], 1e-9)) return false;
    }
    return true;
  };
  return redrawn.ok && final_curve != nullptr && final_render != nullptr &&
         same_points(final_curve->detail.sample_points, samples_after_reshape) &&
         final_render->color_rgba == redraw.color_rgba;
}

bool C627_bb2_legacy_topology_apis_reject_before_mutation() {
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_pole_ids.size() != 2 ||
      generated.value.generated_span_ids.empty()) {
    return false;
  }
  const auto counts = [&]() {
    return std::array<std::size_t, 8>{
        state.view().poles().size(),
        state.view().ports().size(),
        state.view().bundles().size(),
        state.view().spans().size(),
        state.view().backbone().nodes.size(),
        state.view().backbone().edges.size(),
        state.view().backbone().edge_bundles.size(),
        state.view().backbone().span_bindings.size(),
    };
  };
  const auto before = counts();
  const auto connection =
      state.AddConnectionByPole(generated.value.generated_pole_ids[0],
                                generated.value.generated_pole_ids[1],
                                wire::core::ConnectionCategory::kLowVoltage);
  const auto pole_drop =
      state.AddDropFromPole(generated.value.generated_pole_ids[0], {5.0, 4.0, 3.0});
  const auto span_drop =
      state.AddDropFromSpan(generated.value.generated_span_ids.front(), 0.5, {5.0, 4.0, 3.0});
  const auto split = state.SplitSpan(generated.value.generated_span_ids.front(), 0.5);
  return !connection.ok && !pole_drop.ok && !span_drop.ok && !split.ok &&
         contains_text(connection.error, "unsupported") && contains_text(pole_drop.error, "unsupported") &&
         contains_text(span_drop.error, "unsupported") && contains_text(split.error, "unsupported") &&
         counts() == before;
}

bool C622_bb2_stage_timing_is_diagnostic_only() {
  wire::core::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::GenerationTiming& timing = out.value.timing;
  const std::vector<double> stages = {
      timing.prepare_ms,       timing.check_ms,       timing.pairs_ms, timing.preflight_ms,
      timing.intent_ms,        timing.support_groups_ms, timing.emit_ms, timing.save_graph_ms,
      timing.rules_ms,         timing.layout_ms,      timing.geom_ms,  timing.draw_ms,
  };
  if (timing.total_ms <= 0.0 ||
      std::find_if(stages.begin(), stages.end(), [](double value) { return value < 0.0; }) != stages.end()) {
    return false;
  }

  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  const auto updated = state.UpdateGeometrySettings(settings);
  const wire::core::UpdateTiming& update = state.view().last_update_timing();
  return updated.ok && update.kind == wire::core::UpdateKind::kReshape &&
         update.affected_span_count == out.value.generated_span_ids.size() && update.plan_ms >= 0.0 &&
         update.derive_ms >= 0.0 && update.total_ms >= update.derive_ms;
}

} // namespace backbone_tests
