#include "fixtures.hpp"
#include "cases.hpp"

#include "../registry.hpp"

#include "wire/core/core_test_hook.hpp"
#include "wire/core/core_view.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace helpers;

namespace backbone_tests {

namespace {

bool same_vec3(const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
  return almost_equal(a.x, b.x, 1e-12) && almost_equal(a.y, b.y, 1e-12) &&
         almost_equal(a.z, b.z, 1e-12);
}

bool same_frame(const wire::core::Frame3d& a, const wire::core::Frame3d& b) {
  return same_vec3(a.origin, b.origin) && same_vec3(a.forward, b.forward) &&
         same_vec3(a.right, b.right) && same_vec3(a.up, b.up);
}

bool same_band(const wire::core::PortPlacementBand& a, const wire::core::PortPlacementBand& b) {
  return a.band_id == b.band_id && a.category == b.category && same_frame(a.local_direction, b.local_direction) &&
         a.layer == b.layer && a.side == b.side && a.role == b.role &&
         almost_equal(a.lateral_center_m, b.lateral_center_m, 1e-12) &&
         almost_equal(a.lateral_min_m, b.lateral_min_m, 1e-12) &&
         almost_equal(a.lateral_max_m, b.lateral_max_m, 1e-12) &&
         almost_equal(a.height_center_m, b.height_center_m, 1e-12) &&
         almost_equal(a.height_min_m, b.height_min_m, 1e-12) &&
         almost_equal(a.height_max_m, b.height_max_m, 1e-12) && a.priority == b.priority &&
         almost_equal(a.min_spacing_m, b.min_spacing_m, 1e-12) && a.allow_multiple == b.allow_multiple &&
         a.overflow_policy == b.overflow_policy && a.enabled == b.enabled;
}

bool same_pole_type(const wire::core::PoleTypeDefinition& a, const wire::core::PoleTypeDefinition& b) {
  if (a.id != b.id || a.name != b.name || a.description != b.description ||
      !almost_equal(a.default_height_m, b.default_height_m, 1e-12) ||
      a.port_bands.size() != b.port_bands.size() || a.anchor_slots.size() != b.anchor_slots.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.port_bands.size(); ++i) {
    if (!same_band(a.port_bands[i], b.port_bands[i])) {
      return false;
    }
  }
  return std::equal(a.anchor_slots.begin(), a.anchor_slots.end(), b.anchor_slots.begin(),
                    [](const wire::core::AnchorSlotTemplate& lhs,
                       const wire::core::AnchorSlotTemplate& rhs) {
                      return lhs.slot_id == rhs.slot_id && lhs.usage == rhs.usage &&
                             same_vec3(lhs.local_position, rhs.local_position) &&
                             lhs.priority == rhs.priority && lhs.enabled == rhs.enabled;
                    });
}

std::vector<wire::core::Vec3d> bound_port_positions(const wire::core::CoreState& state) {
  std::vector<wire::core::Vec3d> out{};
  if (state.view().backbone().edge_bundles.empty()) {
    return out;
  }
  const wire::core::ObjectId edge_bundle_id = state.view().backbone().edge_bundles.front().edge_bundle_id;
  for (const wire::core::SavedBackbonePortBinding* binding :
       state.view().backbone_port_bindings_for_edge_bundle(edge_bundle_id)) {
    if (binding == nullptr) {
      continue;
    }
    const wire::core::Port* port = state.view().ports().find(binding->port_id);
    if (port != nullptr) {
      out.push_back(port->world_position);
    }
  }
  std::sort(out.begin(), out.end(), [](const wire::core::Vec3d& lhs, const wire::core::Vec3d& rhs) {
    if (!almost_equal(lhs.x, rhs.x, 1e-12)) {
      return lhs.x < rhs.x;
    }
    if (!almost_equal(lhs.y, rhs.y, 1e-12)) {
      return lhs.y < rhs.y;
    }
    return lhs.z < rhs.z;
  });
  return out;
}

struct SpanCurveSignature {
  std::size_t lane = 0;
  wire::core::Vec3d port_a{};
  wire::core::Vec3d port_b{};
  std::vector<wire::core::Vec3d> samples{};
};

std::vector<SpanCurveSignature> span_curve_signatures(const wire::core::CoreState& state) {
  std::vector<SpanCurveSignature> out{};
  if (state.view().backbone().edge_bundles.empty()) {
    return out;
  }
  const wire::core::ObjectId edge_bundle_id = state.view().backbone().edge_bundles.front().edge_bundle_id;
  for (const wire::core::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    if (binding.edge_bundle_id != edge_bundle_id) {
      continue;
    }
    const wire::core::Span* span = state.view().spans().find(binding.span_id);
    const auto* curve = state.find_curve_cache(binding.span_id);
    if (span == nullptr || curve == nullptr) {
      return {};
    }
    const wire::core::Port* port_a = state.view().ports().find(span->port_a_id);
    const wire::core::Port* port_b = state.view().ports().find(span->port_b_id);
    if (port_a == nullptr || port_b == nullptr) {
      return {};
    }
    SpanCurveSignature item{};
    item.lane = binding.lane_index;
    item.port_a = port_a->world_position;
    item.port_b = port_b->world_position;
    item.samples = curve->detail.sample_points;
    out.push_back(std::move(item));
  }
  std::sort(out.begin(), out.end(), [](const SpanCurveSignature& lhs, const SpanCurveSignature& rhs) {
    return lhs.lane < rhs.lane;
  });
  return out;
}

bool same_span_curve_signatures(const std::vector<SpanCurveSignature>& lhs,
                                const std::vector<SpanCurveSignature>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (lhs[i].lane != rhs[i].lane || !same_vec3(lhs[i].port_a, rhs[i].port_a) ||
        !same_vec3(lhs[i].port_b, rhs[i].port_b) || lhs[i].samples.size() != rhs[i].samples.size()) {
      return false;
    }
    for (std::size_t j = 0; j < lhs[i].samples.size(); ++j) {
      if (!same_vec3(lhs[i].samples[j], rhs[i].samples[j])) {
        return false;
      }
    }
  }
  return true;
}

bool same_saved_nodes(const std::vector<wire::core::SavedBackboneNode>& lhs,
                      const std::vector<wire::core::SavedBackboneNode>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    const auto& a = lhs[i];
    const auto& b = rhs[i];
    const bool same_modes =
        a.bundle_modes.size() == b.bundle_modes.size() &&
        std::equal(a.bundle_modes.begin(), a.bundle_modes.end(), b.bundle_modes.begin(),
                   [](const wire::core::SupportNodeBundleMode& x,
                      const wire::core::SupportNodeBundleMode& y) {
                     return x.bundle_template_id == y.bundle_template_id && x.mode == y.mode;
                   });
    if (a.node_id != b.node_id || a.pole_id != b.pole_id || a.support_kind != b.support_kind ||
        !same_vec3(a.position, b.position) || a.has_source_edge != b.has_source_edge ||
        a.source_edge_node_a != b.source_edge_node_a || a.source_edge_node_b != b.source_edge_node_b ||
        !almost_equal(a.source_edge_t, b.source_edge_t, 1e-12) ||
        a.path_point_index != b.path_point_index || !same_modes) {
      return false;
    }
  }
  return true;
}

bool same_saved_edges(const std::vector<wire::core::SavedBackboneEdge>& lhs,
                      const std::vector<wire::core::SavedBackboneEdge>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    const auto& a = lhs[i];
    const auto& b = rhs[i];
    if (a.edge_id != b.edge_id || a.node_a != b.node_a || a.node_b != b.node_b ||
        a.route != b.route || a.order != b.order || !same_vec3(a.dir, b.dir) ||
        !almost_equal(a.lateral_offset_m, b.lateral_offset_m, 1e-12)) {
      return false;
    }
  }
  return true;
}

struct CountSnapshot {
  std::size_t poles = 0;
  std::size_t ports = 0;
  std::size_t bundles = 0;
  std::size_t spans = 0;
  std::size_t nodes = 0;
  std::size_t edges = 0;
  std::size_t edge_bundles = 0;
  std::size_t port_bindings = 0;
  std::size_t span_bindings = 0;
  int fixed_count = 0;
  std::uint64_t template_version = 0;
};

CountSnapshot count_snapshot(const wire::core::CoreState& state) {
  CountSnapshot out{};
  out.poles = state.view().poles().size();
  out.ports = state.view().ports().size();
  out.bundles = state.view().bundles().size();
  out.spans = state.view().spans().size();
  out.nodes = state.view().backbone().nodes.size();
  out.edges = state.view().backbone().edges.size();
  out.edge_bundles = state.view().backbone().edge_bundles.size();
  out.port_bindings = state.view().backbone().port_bindings.size();
  out.span_bindings = state.view().backbone().span_bindings.size();
  const auto it = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  out.fixed_count = it == state.view().bundle_templates().end() ? 0 : it->second.fixed_count;
  out.template_version = it == state.view().bundle_templates().end() ? 0 : it->second.version;
  return out;
}

bool same_counts(const CountSnapshot& a, const CountSnapshot& b) {
  return a.poles == b.poles && a.ports == b.ports && a.bundles == b.bundles && a.spans == b.spans &&
         a.nodes == b.nodes && a.edges == b.edges && a.edge_bundles == b.edge_bundles &&
         a.port_bindings == b.port_bindings && a.span_bindings == b.span_bindings &&
         a.fixed_count == b.fixed_count && a.template_version == b.template_version;
}

bool update_low_voltage_count_to_two(wire::core::CoreState& state, std::string* error = nullptr) {
  const auto template_it = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  if (template_it == state.view().bundle_templates().end()) {
    return false;
  }
  wire::core::BundleTemplate edited = template_it->second;
  edited.fixed_count = 2;
  const auto updated = state.UpdateBundleTemplate(edited);
  if (error != nullptr) {
    *error = updated.error;
  }
  return updated.ok && updated.value;
}

bool update_low_voltage_count_to_one(wire::core::CoreState& state, std::string* error = nullptr) {
  const auto template_it = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  if (template_it == state.view().bundle_templates().end()) {
    return false;
  }
  wire::core::BundleTemplate edited = template_it->second;
  edited.fixed_count = 1;
  const auto updated = state.UpdateBundleTemplate(edited);
  if (error != nullptr) {
    *error = updated.error;
  }
  return updated.ok && updated.value;
}

bool set_low_voltage_count_before_generation(wire::core::CoreState& state, int fixed_count) {
  const auto template_it = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  if (template_it == state.view().bundle_templates().end()) {
    return false;
  }
  wire::core::BundleTemplate edited = template_it->second;
  edited.fixed_count = fixed_count;
  const auto updated = state.UpdateBundleTemplate(edited);
  return updated.ok && updated.value;
}

std::vector<wire::core::ObjectId> span_ids_for_lane(const wire::core::CoreState& state, std::size_t lane) {
  std::vector<wire::core::ObjectId> out{};
  for (const wire::core::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    if (binding.lane_index == lane) {
      out.push_back(binding.span_id);
    }
  }
  return out;
}

std::vector<wire::core::ObjectId> port_ids_for_lane(const wire::core::CoreState& state, std::size_t lane) {
  std::vector<wire::core::ObjectId> out{};
  for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (binding.lane_index == lane) {
      out.push_back(binding.port_id);
    }
  }
  return out;
}

bool no_binding_references(const wire::core::CoreState& state,
                           const std::vector<wire::core::ObjectId>& retired_spans,
                           const std::vector<wire::core::ObjectId>& retired_ports) {
  for (const wire::core::SavedBackboneSpanBinding& binding : state.view().backbone().span_bindings) {
    if (std::find(retired_spans.begin(), retired_spans.end(), binding.span_id) != retired_spans.end()) {
      return false;
    }
  }
  for (const wire::core::SavedBackbonePortBinding& binding : state.view().backbone().port_bindings) {
    if (std::find(retired_ports.begin(), retired_ports.end(), binding.port_id) != retired_ports.end()) {
      return false;
    }
  }
  for (const wire::core::SavedBackboneEdgeBundle& edge_bundle : state.view().backbone().edge_bundles) {
    for (wire::core::ObjectId span_id : edge_bundle.span_ids) {
      if (std::find(retired_spans.begin(), retired_spans.end(), span_id) != retired_spans.end()) {
        return false;
      }
    }
  }
  return true;
}

} // namespace

bool C611_backbone_direct_derive_restores_saved_span_outputs() {
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

bool C613_backbone_port_edit_rederives_generated_span_without_recalc() {
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

bool C614_backbone_update_plan_uses_coarse_kinds() {
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

bool C615_backbone_regenerate_plan_is_not_local_fallback() {
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

bool C616_backbone_reposition_keeps_saved_graph_identity() {
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

bool C619_backbone_reposition_updates_only_affected_spans() {
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

bool C621_backbone_sag_reshape_updates_geom_only() {
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

bool C623_backbone_layout_settings_reject_before_mutation() {
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

bool C624_backbone_variation_settings_reject_before_mutation() {
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

bool C625_backbone_context_profile_reject_before_mutation() {
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

bool C626_backbone_cable_template_updates_derive_outputs() {
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

bool C627_backbone_legacy_topology_apis_are_removed() {
  std::string state_header{};
  std::string api_types{};
  std::string cmake{};
  if (!file_text(repo_root() / "core" / "include" / "wire" / "core" / "core_state.hpp", &state_header) ||
      !file_text(repo_root() / "core" / "include" / "wire" / "core" / "core_state_api_types.hpp", &api_types) ||
      !file_text(repo_root() / "core" / "CMakeLists.txt", &cmake)) {
    return false;
  }
  const std::vector<std::string> retired = {
      "AddConnectionByPole", "AddDropFromPole", "AddDropFromSpan", "SplitSpan",
      "AddConnectionByPoleOptions", "AddConnectionByPoleResult", "AddDropResult", "SplitSpanResult",
  };
  for (const std::string& symbol : retired) {
    if (contains_text(state_header, symbol) || contains_text(api_types, symbol)) {
      return false;
    }
  }
  return !contains_text(cmake, "state/legacy/topology.cpp");
}

bool C628_backbone_active_pole_type_update_repositions_or_rejects_structure() {
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_pole_ids.empty() ||
      generated.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::ObjectId pole_id = generated.value.generated_pole_ids.front();
  const wire::core::ObjectId span_id = generated.value.generated_span_ids.front();
  const wire::core::Pole* pole = state.view().poles().find(pole_id);
  const wire::core::Span* span = state.view().spans().find(span_id);
  if (pole == nullptr || span == nullptr) {
    return false;
  }
  const auto type_it = state.view().pole_types().find(pole->pole_type_id);
  const wire::core::Port* port_a = state.view().ports().find(span->port_a_id);
  const wire::core::Port* port_b = state.view().ports().find(span->port_b_id);
  const wire::core::SpanLayoutView layout = state.span_layout(span_id);
  const wire::core::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  const wire::core::SpanVisualCacheEntry* visual = state.find_span_visual_cache(span_id);
  const wire::core::SpanRenderCacheEntry* render = state.find_span_render_cache(span_id);
  if (type_it == state.view().pole_types().end() || port_a == nullptr || port_b == nullptr ||
      !layout.has_layout() || curve == nullptr || visual == nullptr || render == nullptr) {
    return false;
  }

  const wire::core::PoleTypeDefinition type_before = type_it->second;
  const double pole_height_before = pole->height_m;
  const wire::core::Vec3d port_a_before = port_a->world_position;
  const wire::core::Vec3d port_b_before = port_b->world_position;
  const wire::core::SpanLayoutEntry layout_before = *layout.entry;
  const std::vector<wire::core::Vec3d> curve_before = curve->detail.sample_points;
  const std::uint64_t visual_version_before = visual->source_version;
  const std::uint64_t render_version_before = render->source_version;
  const std::size_t node_count_before = state.view().backbone().nodes.size();
  const std::size_t edge_count_before = state.view().backbone().edges.size();
  const std::size_t binding_count_before = state.view().backbone().span_bindings.size();

  wire::core::PoleTypeDefinition edited = type_before;
  edited.default_height_m += 0.35;
  for (wire::core::PortPlacementBand& band : edited.port_bands) {
    if (!band.enabled) {
      continue;
    }
    band.height_center_m += 0.75;
    band.height_min_m += 0.75;
    band.height_max_m += 0.75;
  }
  const auto updated = state.UpdatePoleTypeDefinition(edited);
  const wire::core::Pole* pole_after = state.view().poles().find(pole_id);
  const wire::core::Port* port_a_after = state.view().ports().find(span->port_a_id);
  const wire::core::Port* port_b_after = state.view().ports().find(span->port_b_id);
  const wire::core::SpanLayoutView layout_after = state.span_layout(span_id);
  const wire::core::CurveCacheEntry* curve_after = state.find_curve_cache(span_id);
  const wire::core::SpanVisualCacheEntry* visual_after = state.find_span_visual_cache(span_id);
  const wire::core::SpanRenderCacheEntry* render_after = state.find_span_render_cache(span_id);
  const auto type_after = state.view().pole_types().find(type_before.id);
  const bool curve_changed =
      curve_after != nullptr && curve_after->detail.sample_points.size() == curve_before.size() &&
      !std::equal(curve_before.begin(), curve_before.end(), curve_after->detail.sample_points.begin(),
                  [](const wire::core::Vec3d& a, const wire::core::Vec3d& b) {
                    return almost_equal(a, b, 1e-12);
                  });
  if (!updated.ok || type_after == state.view().pole_types().end() ||
      !almost_equal(type_after->second.default_height_m, edited.default_height_m, 1e-12) ||
      pole_after == nullptr || !almost_equal((pole_after->height_m - pole_height_before), 0.35, 1e-12) ||
      port_a_after == nullptr || port_b_after == nullptr ||
      almost_equal(port_a_after->world_position, port_a_before, 1e-12) ||
      almost_equal(port_b_after->world_position, port_b_before, 1e-12) ||
      !layout_after.has_layout() ||
      almost_equal(layout_after.entry->start.endpoint_world, layout_before.start.endpoint_world, 1e-12) ||
      almost_equal(layout_after.entry->end.endpoint_world, layout_before.end.endpoint_world, 1e-12) ||
      !curve_changed ||
      visual_after == nullptr || visual_after->source_version == visual_version_before ||
      render_after == nullptr || render_after->source_version == render_version_before ||
      state.view().backbone().nodes.size() != node_count_before ||
      state.view().backbone().edges.size() != edge_count_before ||
      state.view().backbone().span_bindings.size() != binding_count_before) {
    return false;
  }

  const wire::core::PoleTypeDefinition before_structural_reject = type_after->second;
  const wire::core::Vec3d port_a_before_reject = port_a_after->world_position;
  wire::core::PoleTypeDefinition structural = before_structural_reject;
  if (structural.port_bands.empty()) {
    return false;
  }
  structural.port_bands.front().enabled = !structural.port_bands.front().enabled;
  const auto rejected = state.UpdatePoleTypeDefinition(structural);
  const auto type_after_reject = state.view().pole_types().find(type_before.id);
  const wire::core::Port* port_a_after_reject = state.view().ports().find(span->port_a_id);
  return !rejected.ok && contains_text(rejected.error, "unsupported") &&
         type_after_reject != state.view().pole_types().end() &&
         same_pole_type(type_after_reject->second, before_structural_reject) &&
         port_a_after_reject != nullptr &&
         almost_equal(port_a_after_reject->world_position, port_a_before_reject, 1e-12);
}

bool C697_backbone_edge_saves_lateral_offset_echo() {
  wire::core::CoreState offset_state;
  wire::core::BackboneSpec offset_req = line_req(offset_state);
  offset_req.constraints.lateral_offset_m = 1.0;
  const auto offset_generated = offset_state.GenerateFromBackboneSpec(offset_req);
  if (!offset_generated.ok || offset_state.view().backbone().edges.size() != 1) {
    return false;
  }
  if (!almost_equal(offset_state.view().backbone().edges.front().lateral_offset_m, 1.0, 1e-12)) {
    return false;
  }

  wire::core::CoreState default_state;
  const auto default_generated = default_state.GenerateFromBackboneSpec(line_req(default_state));
  if (!default_generated.ok || default_state.view().backbone().edges.size() != 1) {
    return false;
  }
  if (!almost_equal(default_state.view().backbone().edges.front().lateral_offset_m, 0.0, 1e-12)) {
    return false;
  }

  wire::core::CoreState repeat_state;
  wire::core::BackboneSpec repeat_req = line_req(repeat_state);
  repeat_req.constraints.lateral_offset_m = 1.0;
  const auto repeat_generated = repeat_state.GenerateFromBackboneSpec(repeat_req);
  return repeat_generated.ok && repeat_state.view().backbone().edges.size() == 1 &&
         almost_equal(repeat_state.view().backbone().edges.front().lateral_offset_m,
                      offset_state.view().backbone().edges.front().lateral_offset_m, 1e-12);
}
bool C660_backbone_bundle_fixed_count_migration_updates_downstream_only() {
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.size() != 1) {
    return false;
  }
  const auto template_it = state.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  if (template_it == state.view().bundle_templates().end() ||
      template_it->second.count_rule != wire::core::BundleCountRuleKind::kFixed ||
      template_it->second.fixed_count != 1) {
    return false;
  }

  const std::vector<wire::core::SavedBackboneNode> nodes_before = state.view().backbone().nodes;
  const std::vector<wire::core::SavedBackboneEdge> edges_before = state.view().backbone().edges;
  const std::vector<wire::core::SavedBackboneEdgeBundle> edge_bundles_before =
      state.view().backbone().edge_bundles;
  const std::size_t span_count_before = state.view().spans().size();

  wire::core::BundleTemplate edited = template_it->second;
  edited.fixed_count = 2;
  const auto updated = state.UpdateBundleTemplate(edited);
  if (!updated.ok || !updated.value) {
    return false;
  }
  if (!same_saved_nodes(state.view().backbone().nodes, nodes_before) ||
      !same_saved_edges(state.view().backbone().edges, edges_before) ||
      state.view().backbone().edge_bundles.size() != edge_bundles_before.size() ||
      state.view().spans().size() != span_count_before + 1) {
    return false;
  }
  for (std::size_t i = 0; i < edge_bundles_before.size(); ++i) {
    const auto& after = state.view().backbone().edge_bundles[i];
    const auto& before = edge_bundles_before[i];
    if (after.edge_bundle_id != before.edge_bundle_id || after.edge_id != before.edge_id ||
        after.bundle_id != before.bundle_id || after.edge_forward != before.edge_forward ||
        after.route != before.route || after.order != before.order || !same_vec3(after.dir, before.dir)) {
      return false;
    }
  }
  const wire::core::SavedBackboneEdgeBundle& bundle = state.view().backbone().edge_bundles.front();
  if (bundle.span_ids.size() != 2) {
    return false;
  }
  for (wire::core::ObjectId span_id : bundle.span_ids) {
    if (!state.span_layout(span_id).has_layout() ||
        state.find_curve_cache(span_id) == nullptr ||
        state.find_span_visual_cache(span_id) == nullptr ||
        state.find_span_render_cache(span_id) == nullptr) {
      return false;
    }
  }

  wire::core::CoreState fresh;
  const auto fresh_template_it = fresh.view().bundle_templates().find(wire::core::BundleKind::kLowVoltage);
  if (fresh_template_it == fresh.view().bundle_templates().end()) {
    return false;
  }
  wire::core::BundleTemplate fresh_template = fresh_template_it->second;
  fresh_template.fixed_count = 2;
  const auto fresh_updated = fresh.UpdateBundleTemplate(fresh_template);
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(line_req(fresh));
  if (!fresh_updated.ok || !fresh_generated.ok || fresh_generated.value.generated_span_ids.size() != 2 ||
      fresh.view().backbone().edge_bundles.empty() ||
      fresh.view().backbone().edge_bundles.front().span_ids.size() != bundle.span_ids.size()) {
    return false;
  }
  return same_span_curve_signatures(span_curve_signatures(state), span_curve_signatures(fresh));
}

bool C668_backbone_bundle_count_migration_rejects_unreconstructable_lateral_offset() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  req.constraints.lateral_offset_m = 0.35;
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const CountSnapshot before = count_snapshot(state);
  std::string error{};
  const bool updated = update_low_voltage_count_to_two(state, &error);
  return !updated && contains_text(error, "cannot reconstruct existing port placement") &&
         same_counts(before, count_snapshot(state));
}

bool C669_backbone_bundle_count_migration_rejects_multi_bundle_group_offset() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state);
  add_backbone_bundle(req, wire::core::BundleKind::kCommunication);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() < 2) {
    return false;
  }
  const CountSnapshot before = count_snapshot(state);
  std::string error{};
  const bool updated = update_low_voltage_count_to_two(state, &error);
  return !updated && contains_text(error, "cannot reconstruct existing port placement") &&
         same_counts(before, count_snapshot(state));
}

bool C670_backbone_bundle_count_migration_rejects_pair_rows() {
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || generated.value.generated_span_ids.size() < 2) {
    return false;
  }
  const CountSnapshot before = count_snapshot(state);
  std::string error{};
  const bool updated = update_low_voltage_count_to_two(state, &error);
  return !updated && contains_text(error, "simple open rows only") && same_counts(before, count_snapshot(state));
}


bool C671_backbone_bundle_count_migration_reuses_pipeline_stages() {
  std::string source{};
  if (!file_text(repo_root() / "core/src/generation/backbone/bundle_count_migration.cpp", &source)) {
    return false;
  }
  const std::size_t transaction_boundary = source.find("Transaction boundary");
  if (transaction_boundary == std::string::npos ||
      source.find("fail(", transaction_boundary) != std::string::npos) {
    return false;
  }
  return source.find("build_prepared_migration") != std::string::npos &&
         source.find("regenerate_backbone") == std::string::npos &&
         source.find("AddPort(") == std::string::npos && source.find("AddSpan(") == std::string::npos &&
         source.find("SpanLayoutRule") == std::string::npos && source.find("save_backbone_node") == std::string::npos &&
         source.find("save_backbone_edge") == std::string::npos;
}

bool C672_backbone_bundle_count_migration_rejects_manual_ports() {
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const wire::core::Span* span = state.view().spans().find(generated.value.generated_span_ids.front());
  if (span == nullptr) {
    return false;
  }
  const wire::core::Port* port = state.view().ports().find(span->port_a_id);
  if (port == nullptr) {
    return false;
  }
  const auto moved = state.MovePort(port->id, {port->world_position.x + 0.1, port->world_position.y,
                                               port->world_position.z});
  if (!moved.ok) {
    return false;
  }
  const CountSnapshot before = count_snapshot(state);
  std::string error{};
  const bool updated = update_low_voltage_count_to_two(state, &error);
  return !updated && contains_text(error, "manual ports") && same_counts(before, count_snapshot(state));
}

bool C673_backbone_bundle_count_migration_rejects_user_attachments() {
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const auto attachment = state.AddAttachment(generated.value.generated_span_ids.front(), 0.5);
  if (!attachment.ok) {
    return false;
  }
  const CountSnapshot before = count_snapshot(state);
  std::string error{};
  const bool updated = update_low_voltage_count_to_two(state, &error);
  return !updated && contains_text(error, "attachments") && same_counts(before, count_snapshot(state));
}

bool C698_backbone_regenerate_fixed_count_decrease_retires_lanes() {
  wire::core::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  if (!generated.ok || generated.value.generated_span_ids.size() != 2 ||
      state.view().backbone().edge_bundles.empty()) {
    return false;
  }
  const std::vector<wire::core::SavedBackboneNode> nodes_before = state.view().backbone().nodes;
  const std::vector<wire::core::SavedBackboneEdge> edges_before = state.view().backbone().edges;
  const std::vector<wire::core::SavedBackboneEdgeBundle> edge_bundles_before =
      state.view().backbone().edge_bundles;
  const std::vector<wire::core::ObjectId> retired_spans = span_ids_for_lane(state, 1);
  const std::vector<wire::core::ObjectId> retired_ports = port_ids_for_lane(state, 1);
  const std::size_t span_count_before = state.view().spans().size();
  if (retired_spans.size() != 1 || retired_ports.size() != 2) {
    return false;
  }

  const bool updated = update_low_voltage_count_to_one(state);
  if (!updated || state.view().spans().size() != span_count_before - 1 ||
      !same_saved_nodes(state.view().backbone().nodes, nodes_before) ||
      !same_saved_edges(state.view().backbone().edges, edges_before) ||
      state.view().backbone().edge_bundles.size() != edge_bundles_before.size()) {
    return false;
  }
  for (std::size_t i = 0; i < edge_bundles_before.size(); ++i) {
    const auto& after = state.view().backbone().edge_bundles[i];
    const auto& before = edge_bundles_before[i];
    if (after.edge_bundle_id != before.edge_bundle_id || after.edge_id != before.edge_id ||
        after.bundle_id != before.bundle_id || after.edge_forward != before.edge_forward ||
        after.route != before.route || after.order != before.order || !same_vec3(after.dir, before.dir)) {
      return false;
    }
  }
  for (wire::core::ObjectId span_id : retired_spans) {
    if (state.view().spans().find(span_id) != nullptr || state.find_curve_cache(span_id) != nullptr ||
        state.find_span_visual_cache(span_id) != nullptr || state.find_span_render_cache(span_id) != nullptr) {
      return false;
    }
  }
  for (wire::core::ObjectId port_id : retired_ports) {
    if (state.view().ports().find(port_id) != nullptr) {
      return false;
    }
  }
  if (!no_binding_references(state, retired_spans, retired_ports)) {
    return false;
  }

  wire::core::CoreState fresh;
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(line_req(fresh));
  return fresh_generated.ok && fresh_generated.value.generated_span_ids.size() == 1 &&
         same_span_curve_signatures(span_curve_signatures(state), span_curve_signatures(fresh));
}

bool C699_backbone_regenerate_fixed_count_decrease_preserves_lateral_offset() {
  wire::core::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  wire::core::BackboneSpec req = line_req(state);
  req.constraints.lateral_offset_m = 1.0;
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() != 2) {
    return false;
  }
  if (!update_low_voltage_count_to_one(state)) {
    return false;
  }

  wire::core::CoreState fresh;
  wire::core::BackboneSpec fresh_req = line_req(fresh);
  fresh_req.constraints.lateral_offset_m = 1.0;
  const auto fresh_generated = fresh.GenerateFromBackboneSpec(fresh_req);
  wire::core::CoreState fresh_default;
  const auto fresh_default_generated = fresh_default.GenerateFromBackboneSpec(line_req(fresh_default));
  return fresh_generated.ok && fresh_generated.value.generated_span_ids.size() == 1 &&
         fresh_default_generated.ok && fresh_default_generated.value.generated_span_ids.size() == 1 &&
         state.view().backbone().edges.size() == 1 &&
         almost_equal(state.view().backbone().edges.front().lateral_offset_m, 1.0, 1e-12) &&
         !same_span_curve_signatures(span_curve_signatures(state), span_curve_signatures(fresh_default)) &&
         same_span_curve_signatures(span_curve_signatures(state), span_curve_signatures(fresh));
}

bool C700_backbone_regenerate_fixed_count_decrease_rejects_retired_attachment() {
  wire::core::CoreState state;
  if (!set_low_voltage_count_before_generation(state, 2)) {
    return false;
  }
  const auto generated = state.GenerateFromBackboneSpec(line_req(state));
  const std::vector<wire::core::ObjectId> retired_spans = span_ids_for_lane(state, 1);
  if (!generated.ok || retired_spans.size() != 1) {
    return false;
  }
  const auto attachment = state.AddAttachment(retired_spans.front(), 0.5);
  if (!attachment.ok) {
    return false;
  }
  const CountSnapshot before = count_snapshot(state);
  std::string error{};
  const bool updated = update_low_voltage_count_to_one(state, &error);
  return !updated && contains_text(error, "attachments") && same_counts(before, count_snapshot(state));
}

bool C701_backbone_regenerate_source_does_not_infer_topology_from_outputs() {
  std::string source{};
  if (!file_text(repo_root() / "core/src/generation/backbone/regenerate.cpp", &source)) {
    return false;
  }
  return source.find("build_prepared_migration") != std::string::npos &&
         source.find("span_layout(") == std::string::npos &&
         source.find("find_curve_cache") == std::string::npos &&
         source.find("find_span_visual_cache") == std::string::npos &&
         source.find("find_span_render_cache") == std::string::npos &&
         source.find("world_position") == std::string::npos;
}

bool C676_backbone_noop_move_preserves_port_positions_exactly() {
  wire::core::CoreState state;
  const auto generated = state.GenerateFromBackboneSpec(poly3_req(state));
  if (!generated.ok || generated.value.generated_pole_ids.empty()) {
    return false;
  }
  std::vector<std::pair<wire::core::ObjectId, wire::core::Vec3d>> before{};
  for (const wire::core::Port& port : state.view().ports().items()) {
    if (state.view().backbone_port_binding_for_port(port.id) != nullptr) {
      before.push_back({port.id, port.world_position});
    }
  }
  if (before.empty()) {
    return false;
  }
  for (wire::core::ObjectId pole_id : generated.value.generated_pole_ids) {
    const wire::core::Pole* pole = state.view().poles().find(pole_id);
    if (pole == nullptr) {
      return false;
    }
    const auto moved = state.MovePole(pole_id, pole->world_transform);
    if (!moved.ok) {
      return false;
    }
  }
  for (const auto& [port_id, position] : before) {
    const wire::core::Port* port = state.view().ports().find(port_id);
    if (port == nullptr || port->world_position.x != position.x ||
        port->world_position.y != position.y || port->world_position.z != position.z) {
      return false;
    }
  }
  return true;
}

bool C622_backbone_stage_timing_is_diagnostic_only() {
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
