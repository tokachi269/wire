#include "registry.hpp"

#include "wire/core/core_state.hpp"
#include "wire/core/core_view.hpp"

#include <cmath>
#include <vector>

namespace {

bool almost_equal(double a, double b, double eps = 1e-9) {
  return std::abs(a - b) <= eps;
}

bool almost_equal(const wire::core::Vec3d& a, const wire::core::Vec3d& b, double eps = 1e-9) {
  return almost_equal(a.x, b.x, eps) && almost_equal(a.y, b.y, eps) && almost_equal(a.z, b.z, eps);
}

wire::core::PoleTypeId first_pole_type_id(const wire::core::CoreState& state) {
  wire::core::PoleTypeId best = wire::core::kInvalidPoleTypeId;
  for (const auto& [id, _] : state.view().pole_types()) {
    if (best == wire::core::kInvalidPoleTypeId || id < best) {
      best = id;
    }
  }
  return best;
}

void add_bundle(wire::core::BackboneSpec& req, wire::core::BundleKind kind) {
  wire::core::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = kind;
  req.bundles.push_back(bundle);
}

wire::core::BackboneSpec line_req(wire::core::CoreState& state, wire::core::BundleKind kind) {
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req.pole_type_id = first_pole_type_id(state);
  add_bundle(req, kind);
  return req;
}

wire::core::BackboneSpec poly3_req(wire::core::CoreState& state, wire::core::BundleKind kind) {
  wire::core::BackboneSpec req = line_req(state, kind);
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {12.0, 8.0, 0.0}};
  return req;
}

wire::core::BackboneInputSpec::NodeSpec pole_spec(std::size_t point_index, wire::core::ObjectId pole_id) {
  wire::core::BackboneInputSpec::NodeSpec spec{};
  spec.point_index = point_index;
  spec.support_kind = wire::core::SupportKind::kPole;
  spec.node_id = pole_id;
  return spec;
}

wire::core::BackboneSpec pass_branch_req(wire::core::CoreState& state, wire::core::ObjectId pole_id,
                                         const wire::core::Vec3d& pole_pos) {
  wire::core::BackboneSpec req = line_req(state, wire::core::BundleKind::kLowVoltage);
  req.path.polyline = {pole_pos, {20.0, 0.0, 0.0}};
  req.path.node_specs = {pole_spec(0, pole_id)};
  wire::core::BackboneSpec::NodeBundleModeSpec mode{};
  mode.point_index = 0;
  mode.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  mode.mode = wire::core::BundleNodeMode::kPassThrough;
  req.node_bundle_modes = {mode};
  return req;
}

bool bounds_valid(const wire::core::BoundsCacheEntry& bounds) {
  return bounds.whole.min.x <= bounds.whole.max.x && bounds.whole.min.y <= bounds.whole.max.y &&
         bounds.whole.min.z <= bounds.whole.max.z;
}

bool viewer_outputs_exist(const wire::core::CoreState& state, const std::vector<wire::core::ObjectId>& span_ids) {
  const wire::core::CoreView& view = state.view();
  if (view.saved_backbone_result().nodes.empty() || view.saved_backbone_result().edges.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : span_ids) {
    if (!view.span_layout(span_id).has_layout()) {
      return false;
    }
    const wire::core::CurveCacheEntry* curve = view.find_curve_cache(span_id);
    const wire::core::BoundsCacheEntry* bounds = view.find_bounds_cache(span_id);
    if (curve == nullptr || curve->detail.sample_points.size() < 2 || curve->detail.total_length_m <= 0.0 ||
        bounds == nullptr || !bounds_valid(*bounds)) {
      return false;
    }
    if (view.find_span_visual_cache(span_id) == nullptr || view.find_span_render_cache(span_id) == nullptr) {
      return false;
    }
  }
  return true;
}

bool test_bb2_viewer_simple_all_templates_have_display_outputs() {
  wire::core::CoreState state;
  wire::core::BackboneSpec req = line_req(state, wire::core::BundleKind::kLowVoltage);
  add_bundle(req, wire::core::BundleKind::kHighVoltage);
  add_bundle(req, wire::core::BundleKind::kCommunication);
  const auto out = state.GenerateFromBackboneSpec(req);
  if (!out.ok || out.value.generated_span_ids.empty() || out.value.bundle_ids.size() != 3) {
    return false;
  }
  const wire::core::BackboneResult backbone = state.view().saved_backbone_result();
  return backbone.nodes.size() == 2 && backbone.edges.size() == 1 &&
         viewer_outputs_exist(state, out.value.generated_span_ids);
}

bool test_bb2_viewer_existing_branch_uses_context_without_regenerating_it() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state, wire::core::BundleKind::kLowVoltage));
  if (!first.ok || first.value.generated_pole_ids.size() != 3 ||
      !viewer_outputs_exist(state, first.value.generated_span_ids)) {
    return false;
  }
  const std::size_t span_count_before = state.view().spans().size();
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const wire::core::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  wire::core::BackboneSpec branch = line_req(state, wire::core::BundleKind::kLowVoltage);
  branch.path.polyline = {pole_b->world_transform.position, {20.0, 0.0, 0.0}};
  branch.path.node_specs = {pole_spec(0, b)};
  const auto second = state.GenerateFromBackboneSpec(branch);
  if (!second.ok || second.value.generated_span_ids.empty() ||
      state.view().spans().size() != span_count_before + second.value.generated_span_ids.size()) {
    return false;
  }
  const wire::core::BackboneResult backbone = state.view().saved_backbone_result();
  const wire::core::BackboneFrontier frontier = state.view().pole_frontier(b);
  return backbone.edges.size() == 3 && frontier.edge_ids.size() == 3 &&
         viewer_outputs_exist(state, second.value.generated_span_ids);
}

bool test_bb2_viewer_pass_through_lowering_is_visible_from_layout_draw() {
  wire::core::CoreState state;
  const auto first = state.GenerateFromBackboneSpec(poly3_req(state, wire::core::BundleKind::kLowVoltage));
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId b = first.value.generated_pole_ids[1];
  const wire::core::Pole* pole_b = state.view().poles().find(b);
  if (pole_b == nullptr) {
    return false;
  }
  const auto second = state.GenerateFromBackboneSpec(pass_branch_req(state, b, pole_b->world_transform.position));
  if (!second.ok || second.value.generated_span_ids.empty() ||
      !viewer_outputs_exist(state, second.value.generated_span_ids)) {
    return false;
  }
  for (wire::core::ObjectId span_id : second.value.generated_span_ids) {
    const wire::core::SpanLayoutView layout = state.view().span_layout(span_id);
    const wire::core::SpanVisualCacheEntry* visual = state.view().find_span_visual_cache(span_id);
    if (!layout.has_layout() || visual == nullptr) {
      return false;
    }
    const auto endpoint_has_lowered_visual = [&](const wire::core::LayoutEndpoint& endpoint) {
      if (!endpoint.lower_required && !endpoint.default_lower_required) {
        return false;
      }
      const double lower_offset =
          endpoint.branch_down_offset_m > 0.0 ? endpoint.branch_down_offset_m : endpoint.automatic_branch_down_offset_m;
      if (lower_offset <= 0.0 || !almost_equal(endpoint.endpoint_world.z, endpoint.support_world.z - lower_offset)) {
        return false;
      }
      for (const wire::core::VisualPart& part : visual->parts) {
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

void register_bb2_scene_tests(viewer_test_registry::TestRegistry& tests) {
  viewer_test_registry::AddTest(tests, "V16", "bb2 viewer scene: simple LV/HV/Communication line has display outputs",
                                test_bb2_viewer_simple_all_templates_have_display_outputs);
  viewer_test_registry::AddTest(tests, "V17", "bb2 viewer scene: existing branch uses saved context only",
                                test_bb2_viewer_existing_branch_uses_context_without_regenerating_it);
  viewer_test_registry::AddTest(tests, "V18", "bb2 viewer scene: pass-through lowering is visible from layout/draw",
                                test_bb2_viewer_pass_through_lowering_is_visible_from_layout_draw);
}

WIRE_REGISTER_VIEWER_TEST_SUITE(register_bb2_scene_tests);

} // namespace
