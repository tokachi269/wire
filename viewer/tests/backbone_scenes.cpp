#include "registry.hpp"

#include "city/wire/core_state.hpp"
#include "city/wire/core_view.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

bool almost_equal(double a, double b, double eps = 1e-9) {
  return std::abs(a - b) <= eps;
}

bool almost_equal(const city::wire::Vec3d& a, const city::wire::Vec3d& b, double eps = 1e-9) {
  return almost_equal(a.x, b.x, eps) && almost_equal(a.y, b.y, eps) && almost_equal(a.z, b.z, eps);
}

double dist2(const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
  const city::wire::Vec3d d = a - b;
  return d.x * d.x + d.y * d.y + d.z * d.z;
}

bool finite(const city::wire::Vec3d& v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

city::wire::PoleTypeId first_pole_type_id(const city::wire::CoreState& state) {
  city::wire::PoleTypeId best = city::wire::kInvalidPoleTypeId;
  for (const auto& [id, _] : state.view().pole_types()) {
    if (best == city::wire::kInvalidPoleTypeId || id < best) {
      best = id;
    }
  }
  return best;
}

void add_bundle(city::wire::BackboneSpec& req, city::wire::BundleKind kind) {
  city::wire::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = kind;
  req.bundles.push_back(bundle);
}

city::wire::BackboneSpec line_req(city::wire::CoreState& state, city::wire::BundleKind kind) {
  city::wire::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.pole_type_id = first_pole_type_id(state);
  add_bundle(req, kind);
  return req;
}

city::wire::BackboneSpec poly3_req(city::wire::CoreState& state, city::wire::BundleKind kind) {
  city::wire::BackboneSpec req = line_req(state, kind);
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {12.0, 8.0, 0.0}};
  return req;
}

city::wire::BackboneInputSpec::NodeSpec pole_spec(std::size_t point_index, city::wire::ObjectId pole_id) {
  city::wire::BackboneInputSpec::NodeSpec spec{};
  spec.point_index = point_index;
  spec.support_kind = city::wire::SupportKind::kPole;
  spec.node_id = pole_id;
  return spec;
}

bool bounds_valid(const city::wire::BoundsCacheEntry& bounds) {
  return bounds.whole.min.x <= bounds.whole.max.x && bounds.whole.min.y <= bounds.whole.max.y &&
         bounds.whole.min.z <= bounds.whole.max.z;
}

bool bounds_contains(const city::wire::BoundsCacheEntry& bounds, const city::wire::Vec3d& p) {
  constexpr double eps = 1e-9;
  return p.x >= bounds.whole.min.x - eps && p.y >= bounds.whole.min.y - eps && p.z >= bounds.whole.min.z - eps &&
         p.x <= bounds.whole.max.x + eps && p.y <= bounds.whole.max.y + eps && p.z <= bounds.whole.max.z + eps;
}

bool viewer_outputs_exist(const city::wire::CoreState& state, const std::vector<city::wire::ObjectId>& span_ids) {
  const city::wire::CoreView& view = state.view();
  if (view.saved_backbone_result().nodes.empty() || view.saved_backbone_result().edges.empty()) {
    return false;
  }
  for (city::wire::ObjectId span_id : span_ids) {
    const city::wire::SpanLayoutView layout = view.span_layout(span_id);
    if (!layout.has_layout()) {
      return false;
    }
    const city::wire::CurveCacheEntry* curve = view.find_curve_cache(span_id);
    const city::wire::BoundsCacheEntry* bounds = view.find_bounds_cache(span_id);
    const city::wire::SpanRenderCacheEntry* render = view.find_span_render_cache(span_id);
    if (curve == nullptr || curve->detail.sample_points.size() < 2 || curve->detail.total_length_m <= 0.0 ||
        bounds == nullptr || !bounds_valid(*bounds) || render == nullptr || render->wire_radius_m <= 0.0 ||
        render->arc_length_m_by_point.size() != curve->detail.sample_points.size() ||
        render->arc_length_normalized_by_point.size() != curve->detail.sample_points.size() ||
        (curve->detail.sample_points.size() >= 2 &&
         render->segment_length_m.size() + 1 != curve->detail.sample_points.size())) {
      return false;
    }
    if (!almost_equal(curve->detail.start_constraint.point, layout.entry->start.endpoint_world) ||
        !almost_equal(curve->detail.end_constraint.point, layout.entry->end.endpoint_world)) {
      return false;
    }
    for (const city::wire::Vec3d& p : curve->detail.sample_points) {
      if (!finite(p) || !bounds_contains(*bounds, p)) {
        return false;
      }
    }
    const city::wire::SpanVisualCacheEntry* visual = view.find_span_visual_cache(span_id);
    if (visual == nullptr) {
      return false;
    }
    for (const city::wire::VisualPart& part : visual->parts) {
      if (!finite(part.a) || !finite(part.b) || part.radius_m < 0.0) {
        return false;
      }
    }
  }
  return true;
}

bool endpoints_are_visually_separated(const city::wire::CoreState& state,
                                      const std::vector<city::wire::ObjectId>& span_ids,
                                      bool start_endpoint) {
  std::vector<city::wire::Vec3d> endpoints{};
  for (city::wire::ObjectId span_id : span_ids) {
    const city::wire::SpanLayoutView layout = state.view().span_layout(span_id);
    if (!layout.has_layout()) {
      return false;
    }
    endpoints.push_back(start_endpoint ? layout.entry->start.endpoint_world : layout.entry->end.endpoint_world);
  }
  if (endpoints.size() < 2) {
    return false;
  }
  for (std::size_t i = 0; i < endpoints.size(); ++i) {
    for (std::size_t j = i + 1; j < endpoints.size(); ++j) {
      if (dist2(endpoints[i], endpoints[j]) <= 1e-4) {
        return false;
      }
    }
  }
  return true;
}

bool test_backbone_viewer_simple_all_templates_have_display_outputs() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = line_req(state, city::wire::BundleKind::kLowVoltage);
  add_bundle(req, city::wire::BundleKind::kHighVoltage);
  add_bundle(req, city::wire::BundleKind::kCommunication);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty() || out.value.bundle_ids.size() != 3) {
    return false;
  }
  const city::wire::BackboneResult backbone = state.view().saved_backbone_result();
  return backbone.nodes.size() == 2 && backbone.edges.size() == 1 &&
         viewer_outputs_exist(state, out.value.generated_span_ids) &&
         endpoints_are_visually_separated(state, out.value.generated_span_ids, true) &&
         endpoints_are_visually_separated(state, out.value.generated_span_ids, false);
}

bool test_backbone_viewer_existing_branch_uses_context_without_regenerating_it() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state, city::wire::BundleKind::kLowVoltage));
  if (!first.ok || first.value.generated_pole_ids.size() != 3 ||
      !viewer_outputs_exist(state, first.value.generated_span_ids)) {
    return false;
  }
  const std::size_t span_count_before = state.view().spans().size();
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const city::wire::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  city::wire::BackboneSpec branch = line_req(state, city::wire::BundleKind::kLowVoltage);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok || second.value.generated_span_ids.empty() ||
      state.view().spans().size() != span_count_before + second.value.generated_span_ids.size()) {
    return false;
  }
  const city::wire::BackboneResult backbone = state.view().saved_backbone_result();
  const city::wire::BackboneFrontier frontier = state.view().pole_frontier(b);
  return backbone.edges.size() == 3 && frontier.edge_ids.size() == 3 &&
         viewer_outputs_exist(state, second.value.generated_span_ids);
}

bool test_backbone_viewer_conflict_lowering_is_visible_from_layout_draw() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state, city::wire::BundleKind::kHighVoltage));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const city::wire::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  city::wire::BackboneSpec branch = line_req(state, city::wire::BundleKind::kHighVoltage);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok || second.value.generated_span_ids.empty() ||
      !viewer_outputs_exist(state, second.value.generated_span_ids)) {
    return false;
  }
  const auto template_it = state.view().bundle_templates().find(city::wire::BundleKind::kHighVoltage);
  if (template_it == state.view().bundle_templates().end() ||
      !template_it->second.enable_branch_down_offset ||
      template_it->second.branch_endpoint_offset_m >= 0.0) {
    return false;
  }
  for (city::wire::ObjectId span_id : second.value.generated_span_ids) {
    const city::wire::SpanLayoutView layout = state.view().span_layout(span_id);
    const city::wire::SpanVisualCacheEntry* visual = state.view().find_span_visual_cache(span_id);
    if (!layout.has_layout() || visual == nullptr) {
      return false;
    }
    const auto endpoint_has_lowered_visual = [&](const city::wire::LayoutEndpoint& endpoint) {
      if (!endpoint.lower_required && !endpoint.default_lower_required) {
        return false;
      }
      if (!almost_equal(endpoint.endpoint_offset_z_m, template_it->second.branch_endpoint_offset_m) ||
          !almost_equal(endpoint.endpoint_world.z,
                        endpoint.support_world.z + template_it->second.branch_endpoint_offset_m)) {
        return false;
      }
      for (const city::wire::VisualPart& part : visual->parts) {
        if (almost_equal(part.a, endpoint.support_world) && almost_equal(part.b, endpoint.endpoint_world)) {
          return true;
        }
      }
      return false;
    };
    if (endpoint_has_lowered_visual(layout.entry->start) || endpoint_has_lowered_visual(layout.entry->end)) {
      return true;
    }
  }
  return false;
}

bool test_backbone_viewer_acute_hv_corner_does_not_lower() {
  city::wire::CoreState state;
  city::wire::BackboneSpec req = poly3_req(state, city::wire::BundleKind::kHighVoltage);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty() || !viewer_outputs_exist(state, out.value.generated_span_ids)) {
    return false;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    const city::wire::SpanLayoutView layout = state.view().span_layout(span_id);
    if (span == nullptr || !layout.has_layout()) {
      return false;
    }
    const city::wire::Port* port_a = state.view().ports().find(span->port_a_id);
    const city::wire::Port* port_b = state.view().ports().find(span->port_b_id);
    if (port_a == nullptr || port_b == nullptr) {
      return false;
    }
    const auto endpoint_unchanged = [](const city::wire::LayoutEndpoint& endpoint, const city::wire::Port& port) {
      return !endpoint.default_lower_required && !endpoint.lower_required &&
             almost_equal(endpoint.support_world, port.world_position) &&
             almost_equal(endpoint.endpoint_world, port.world_position);
    };
    if (!endpoint_unchanged(layout.entry->start, *port_a) ||
        !endpoint_unchanged(layout.entry->end, *port_b)) {
      return false;
    }
  }
  return true;
}

bool test_backbone_viewer_cross_pair_pair_has_display_outputs() {
  city::wire::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state, city::wire::BundleKind::kLowVoltage));
  if (!first.ok || first.value.generated_pole_ids.size() != 3 ||
      !viewer_outputs_exist(state, first.value.generated_span_ids)) {
    return false;
  }
  const std::size_t span_count_before = state.view().spans().size();
  const city::wire::ObjectId b = first.value.generated_pole_ids[1];
  const city::wire::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  city::wire::BackboneSpec cross = line_req(state, city::wire::BundleKind::kLowVoltage);
  cross.path.polyline = {{12.0, -8.0, 0.0}, pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  cross.path.node_specs = {pole_spec(1, b)};
  const auto second = state.GenerateFromBackboneSpec(cross);
  if (!second.ok || second.value.generated_span_ids.empty() ||
      state.view().spans().size() != span_count_before + second.value.generated_span_ids.size()) {
    return false;
  }
  const city::wire::BackboneResult backbone = state.view().saved_backbone_result();
  const city::wire::BackboneFrontier frontier = state.view().pole_frontier(b);
  return backbone.edges.size() == 4 && frontier.edge_ids.size() == 4 &&
         viewer_outputs_exist(state, second.value.generated_span_ids);
}

bool test_backbone_viewer_segment_pick_midair_branch_has_display_outputs() {
  city::wire::CoreState state;
  const auto base = state.GenerateFromBackboneSpec(line_req(state, city::wire::BundleKind::kLowVoltage));
  if (!base.ok || base.value.generated_span_ids.empty() || !viewer_outputs_exist(state, base.value.generated_span_ids)) {
    return false;
  }
  const city::wire::Span* source_span = state.view().spans().find(base.value.generated_span_ids.front());
  if (source_span == nullptr) {
    return false;
  }
  const city::wire::Port* source_a = state.view().ports().find(source_span->port_a_id);
  const city::wire::Port* source_b = state.view().ports().find(source_span->port_b_id);
  if (source_a == nullptr || source_b == nullptr) {
    return false;
  }
  const double expected_z = 0.5 * (source_a->world_position.z + source_b->world_position.z);

  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kSegment;
  pick.hit_id = source_span->id;
  pick.hit_pos_world = {6.0, 0.0, 0.0};
  pick.has_segment_endpoints = false;
  city::wire::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {city::wire::BundleKind::kLowVoltage};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.support_kind != city::wire::SupportKind::kMidair ||
      resolved.value.resolved_node_id == city::wire::kInvalidObjectId ||
      !almost_equal(resolved.value.position.z, expected_z)) {
    return false;
  }

  city::wire::BackboneSpec branch = line_req(state, city::wire::BundleKind::kLowVoltage);
  branch.path.polyline = {resolved.value.position, {6.0, 8.0, 0.0}};
  city::wire::BackboneInputSpec::NodeSpec node{};
  node.point_index = 0;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  branch.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(branch);
  if (!out.ok || out.value.generated_span_ids.empty() || !viewer_outputs_exist(state, out.value.generated_span_ids)) {
    return false;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    if (span == nullptr) {
      return false;
    }
    const city::wire::Port* port_a = state.view().ports().find(span->port_a_id);
    const city::wire::Port* port_b = state.view().ports().find(span->port_b_id);
    if (port_a == nullptr || port_b == nullptr) {
      return false;
    }
    if ((port_a->owner_pole_id == city::wire::kInvalidObjectId &&
         almost_equal(port_a->world_position.z, expected_z)) ||
        (port_b->owner_pole_id == city::wire::kInvalidObjectId &&
         almost_equal(port_b->world_position.z, expected_z))) {
      return true;
    }
  }
  return false;
}

bool test_backbone_viewer_selected_building_pick_generates_selected_bundle_only() {
  city::wire::CoreState state;
  city::wire::PickResult pick{};
  pick.hit_kind = city::wire::PickHitKind::kExternal;
  pick.hit_id = city::wire::kInvalidObjectId;
  pick.hit_pos_world = {12.0, 0.0, 6.0};
  city::wire::ResolveBranchPickOptions resolve{};
  resolve.selected_bundle_template_ids = {city::wire::BundleKind::kCommunication};
  const auto resolved = state.ResolveBranchPick(pick, resolve);
  if (!resolved.ok || resolved.value.support_kind != city::wire::SupportKind::kExternal) {
    return false;
  }

  city::wire::BackboneSpec req = poly3_req(state, city::wire::BundleKind::kLowVoltage);
  add_bundle(req, city::wire::BundleKind::kCommunication);
  req.path.polyline = {{0.0, 0.0, 0.0}, resolved.value.position, {24.0, 0.0, 0.0}};
  city::wire::BackboneInputSpec::NodeSpec node{};
  node.point_index = 1;
  node.support_kind = resolved.value.support_kind;
  node.node_id = resolved.value.resolved_node_id;
  req.path.node_specs = {node};
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.bundle_ids.size() != 1 || out.value.generated_span_ids.empty() ||
      !viewer_outputs_exist(state, out.value.generated_span_ids)) {
    return false;
  }
  const city::wire::Bundle* bundle = state.view().bundles().find(out.value.bundle_ids.front());
  if (bundle == nullptr || bundle->bundle_template_id != city::wire::BundleKind::kCommunication) {
    return false;
  }
  const bool has_building_node =
      std::find_if(state.view().backbone().nodes.begin(), state.view().backbone().nodes.end(),
                   [](const city::wire::SavedBackboneNode& node) {
                     return node.pole_id == city::wire::kInvalidObjectId &&
                            node.support_kind == city::wire::SupportKind::kExternal;
                   }) != state.view().backbone().nodes.end();
  if (!has_building_node) {
    return false;
  }
  for (city::wire::ObjectId span_id : out.value.generated_span_ids) {
    const city::wire::Span* span = state.view().spans().find(span_id);
    const city::wire::Bundle* span_bundle =
        span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
    if (span_bundle == nullptr || span_bundle->bundle_template_id != city::wire::BundleKind::kCommunication) {
      return false;
    }
  }
  return true;
}

bool test_backbone_viewer_sag_uses_curved_geom_output() {
  city::wire::CoreState state;
  const auto out = state.GenerateFromBackboneSpec(line_req(state, city::wire::BundleKind::kLowVoltage));
  if (!out.ok || out.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId span_id = out.value.generated_span_ids.front();
  const city::wire::SpanLayoutView layout = state.span_layout(span_id);
  if (!layout.has_layout()) {
    return false;
  }
  city::wire::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = true;
  settings.curve_samples = std::max(9, settings.curve_samples);
  const auto updated = state.UpdateGeometrySettings(settings);
  const city::wire::CurveCacheEntry* curve = state.find_curve_cache(span_id);
  const city::wire::SpanRenderCacheEntry* render = state.find_span_render_cache(span_id);
  if (!updated.ok || curve == nullptr || render == nullptr || curve->detail.sag_amplitude_m <= 0.0 ||
      curve->detail.sample_points.size() < 3 ||
      render->arc_length_m_by_point.size() != curve->detail.sample_points.size()) {
    return false;
  }
  const double endpoint_min_z =
      std::min(layout.entry->start.endpoint_world.z, layout.entry->end.endpoint_world.z);
  return almost_equal(curve->detail.EvaluatePosition(0.0), layout.entry->start.endpoint_world) &&
         almost_equal(curve->detail.EvaluatePosition(1.0), layout.entry->end.endpoint_world) &&
         std::find_if(curve->detail.sample_points.begin(), curve->detail.sample_points.end(),
                      [&](const city::wire::Vec3d& point) { return point.z < endpoint_min_z; }) !=
             curve->detail.sample_points.end();
}

bool test_span_layout_debug_panel_reads_neutral_outputs() {
  std::ifstream file("viewer/src/panels.cpp");
  if (!file.is_open()) {
    return false;
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  const std::string source = buffer.str();
  return source.find("inspect_support_layout(") == std::string::npos &&
         source.find("Neutral Span Output Debug") != std::string::npos &&
         source.find("span_layout_state(") != std::string::npos &&
         source.find("span_layout_rules(") != std::string::npos &&
         source.find("span_layout(") != std::string::npos &&
         source.find("span_frontier(") != std::string::npos;
}

bool test_population_rule_produces_viewer_curve_parts() {
  city::wire::CoreState state;
  auto bundle_template = state.view().bundle_templates().at(city::wire::BundleKind::kCommunication);
  city::wire::CablePopulationRule rule{};
  rule.rule_id = 1;
  rule.explicit_seed = 42;
  rule.min_extra_count = 3;
  rule.max_extra_count = 3;
  rule.min_spacing_m = 0.04;
  rule.lateral_min_m = -2.0;
  rule.lateral_max_m = 2.0;
  rule.height_min_m = 0.0;
  rule.height_max_m = 20.0;
  rule.randomness = 0.5;
  bundle_template.population_rules.push_back(rule);
  const auto configured = state.UpdateBundleTemplate(bundle_template);
  const auto generated = state.GenerateFromBackboneSpec(line_req(state, city::wire::BundleKind::kCommunication));
  if (!configured.ok || !generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  const city::wire::ObjectId span_id = generated.value.generated_span_ids.front();
  const city::wire::SpanRenderCacheEntry* render = state.find_span_render_cache(span_id);
  if (render == nullptr) {
    return false;
  }
  std::size_t physical_count = 0;
  bool has_original_edge_body = false;
  city::wire::Vec3d original_endpoint{};
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind == city::wire::VisualCurvePartKind::kEdgeBody && part.source_span_id == span_id) {
      has_original_edge_body = true;
      original_endpoint = part.boundary_a;
      break;
    }
  }
  for (const city::wire::VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.kind != city::wire::VisualCurvePartKind::kEdgeBody ||
        !part.has_cable_instance_key || part.cable_instance_key.is_base()) {
      continue;
    }
    if (part.source_span_id != span_id || part.samples.size() < 2 ||
        !almost_equal(part.wire_radius_m, render->wire_radius_m) || part.color_rgba != render->color_rgba) {
      return false;
    }
    if (almost_equal(part.boundary_a.x, original_endpoint.x) &&
        almost_equal(part.boundary_a.y, original_endpoint.y) &&
        almost_equal(part.boundary_a.z, original_endpoint.z)) {
      return false;
    }
    ++physical_count;
  }
  return has_original_edge_body && physical_count == 3;
}

void register_backbone_scene_tests(viewer_test_registry::TestRegistry& tests) {
  viewer_test_registry::AddTest(tests, "V16", "backbone viewer scene: simple LV/HV/Communication line has display outputs",
                                test_backbone_viewer_simple_all_templates_have_display_outputs);
  viewer_test_registry::AddTest(tests, "V17", "backbone viewer scene: existing branch uses saved context only",
                                test_backbone_viewer_existing_branch_uses_context_without_regenerating_it);
  viewer_test_registry::AddTest(tests, "V18", "backbone viewer scene: conflict lowering is visible from layout/draw",
                                test_backbone_viewer_conflict_lowering_is_visible_from_layout_draw);
  viewer_test_registry::AddTest(tests, "V19", "backbone viewer scene: acute HV corner does not lower",
                                test_backbone_viewer_acute_hv_corner_does_not_lower);
  viewer_test_registry::AddTest(tests, "V20", "backbone viewer scene: cross pair+pair has display outputs",
                                test_backbone_viewer_cross_pair_pair_has_display_outputs);
  viewer_test_registry::AddTest(tests, "V21", "backbone viewer scene: segment-pick midair branch has display outputs",
                                test_backbone_viewer_segment_pick_midair_branch_has_display_outputs);
  viewer_test_registry::AddTest(tests, "V22", "backbone viewer scene: selected building pick filters bundles",
                                test_backbone_viewer_selected_building_pick_generates_selected_bundle_only);
  viewer_test_registry::AddTest(tests, "V23", "SpanLayout debug panel reads neutral span outputs",
                                test_span_layout_debug_panel_reads_neutral_outputs);
  viewer_test_registry::AddTest(tests, "V24", "backbone viewer scene: sag uses curved geom and render output",
                                test_backbone_viewer_sag_uses_curved_geom_output);
  viewer_test_registry::AddTest(tests, "V25", "backbone viewer scene: population rule adds physical curve parts",
                                test_population_rule_produces_viewer_curve_parts);
}

WIRE_REGISTER_VIEWER_TEST_SUITE(register_backbone_scene_tests);

} // namespace
