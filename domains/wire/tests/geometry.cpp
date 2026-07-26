#include "registry.hpp"
#include "helpers.hpp"
#include "city/wire/core_test_hook.hpp"
#include "city/wire/coord_utils.hpp"

#include "../src/geometry/curve/curve.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace helpers;
using city::wire::PortKind;
using city::wire::PortLayer;
using city::wire::SpanKind;
using city::wire::SpanLayer;

namespace {
std::optional<city::wire::PoleTypeDefinition> find_pole_type_by_name(const CoreState& state, const std::string& name) {
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == name) {
      city::wire::PoleTypeDefinition copy = pole_type;
      copy.id = id;
      return copy;
    }
  }
  return std::nullopt;
}

double polyline_length(const std::vector<city::wire::Vec3d>& points) {
  double length = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const city::wire::Vec3d delta = points[i + 1] - points[i];
    length += std::sqrt(city::wire::LengthSquared(delta));
  }
  return length;
}

city::wire::EditResult<city::wire::GenerateBundleFromPathResult>
generate_path(CoreState& state, const city::wire::BackboneInputSpec& path, double interval,
              city::wire::PoleTypeId pole_type_id) {
  city::wire::BackboneSpec spec{};
  spec.path = path;
  spec.interval_m = interval;
  spec.pole_type_id = pole_type_id;
  city::wire::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = city::wire::DefaultBundleTemplateId(city::wire::BundleKind::kLowVoltage);
  spec.bundles.push_back(bundle);
  return state.GenerateFromBackboneSpec(spec);
}

bool test_backbone_interval_generates_pole_line_basic() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  city::wire::BackboneInputSpec path{};
  path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  city::wire::BackboneSpec spec{};
  spec.path = path;
  spec.interval_m = 5.0;
  spec.pole_type_id = type_ids.front();
  city::wire::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = bundle_template_for_category_test(city::wire::ConnectionCategory::kLowVoltage);
  spec.bundles.push_back(bundle);

  const auto result = state.GenerateFromBackboneSpec(spec);
  if (!result.ok) {
    return false;
  }
  if (result.value.generated_pole_ids.size() != 5) {
    return false;
  }

  for (std::size_t i = 0; i < result.value.generated_pole_ids.size(); ++i) {
    const auto* pole = state.view().edit_state().poles.find(result.value.generated_pole_ids[i]);
    if (pole == nullptr) {
      return false;
    }
    if (pole->pole_type_id != type_ids.front()) {
      return false;
    }
    if (!almost_equal(pole->world_transform.position.y, 0.0) || !almost_equal(pole->world_transform.position.z, 0.0)) {
      return false;
    }
  }
  if (state.view().backbone().nodes.size() != result.value.generated_pole_ids.size()) {
    return false;
  }
  return true;
}

bool test_acute_corner_auto_widens_lane_spacing() {
  auto row_count_for_interior = [](double interior_deg, bool expect_open) -> std::size_t {
    CoreState state;
    const city::wire::BundleTemplateId bundle_kind =
        bundle_template_for_category_test(city::wire::ConnectionCategory::kLowVoltage);
    city::wire::LayoutSettings layout = state.view().layout_settings();
    layout.max_side_scale = 6.0;
    if (!state.UpdateLayoutSettings(layout).ok ||
        state.view().layout_settings().max_side_scale > city::wire::kMaxCornerSideScale + 1e-9) {
      return 0;
    }
    const auto bundle_it = state.view().bundle_templates().find(bundle_kind);
    const auto type_ids = sorted_pole_type_ids(state);
    if (bundle_it == state.view().bundle_templates().end() || type_ids.empty()) {
      return 0;
    }
    city::wire::BundleTemplate bundle = bundle_it->second;
    bundle.count_rule = city::wire::BundleCountRuleKind::kFixed;
    bundle.fixed_count = 2;
    bundle.default_count = 2;
    bundle.min_count = std::min(bundle.min_count, 2);
    bundle.max_count = std::max(bundle.max_count, 2);
    if (!state.UpdateBundleTemplate(bundle).ok) {
      return 0;
    }

    constexpr double kPi = 3.14159265358979323846;
    const double rad = interior_deg * (kPi / 180.0);
    const double out_heading = kPi - rad;
    city::wire::BackboneInputSpec path{};
    path.polyline = {
        {0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0},
        {10.0 + 10.0 * std::cos(out_heading), 10.0 * std::sin(out_heading), 0.0},
    };
    const auto generated = generate_path(state, path, polyline_length(path.polyline) + 1.0, type_ids.front());
    if (!generated.ok || generated.value.generated_pole_ids.size() != 3) {
      return 0;
    }
    const city::wire::ObjectId corner_pole = generated.value.generated_pole_ids[1];
    const city::wire::SavedBackboneNode* node = state.view().backbone_node_for_pole(corner_pole);
    if (node == nullptr) {
      return 0;
    }

    std::vector<city::wire::Vec3d> row_points{};
    for (const city::wire::Port& port : state.view().ports().items()) {
      if (port.owner_pole_id != corner_pole) {
        continue;
      }
      const city::wire::SavedBackbonePortBinding* binding = state.view().backbone_port_binding_for_port(port.id);
      if (binding == nullptr || binding->row_key.node_id != node->node_id ||
          std::abs(port.side_scale_applied - 1.0) > 1e-9) {
        continue;
      }
      if (std::none_of(row_points.begin(), row_points.end(),
                       [&](const city::wire::Vec3d& value) {
                         return city::wire::Length(value - port.world_position) <=
                                1e-9;
                       })) {
        row_points.push_back(port.world_position);
      }
    }
    const std::size_t expected_lane_count = 2;
    if (row_points.size() % expected_lane_count != 0) {
      return 0;
    }
    const std::size_t row_count = row_points.size() / expected_lane_count;
    return row_count == (expect_open ? 2u : 1u) ? row_count : 0;
  };

  return row_count_for_interior(45.0, true) == 2 && row_count_for_interior(120.0, false) == 1;
}

bool test_band_selection_context_bias() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  city::wire::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  city::wire::Transformd b{};
  b.position = {10.0, 0.0, 0.0};
  city::wire::Transformd c{};
  c.position = {10.0, 8.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  const ObjectId pole_c = state.AddPole(c, 10.0, "C").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok ||
      !state.ApplyPoleType(pole_c, type_ids.front()).ok) {
    return false;
  }

  FixtureConnectionOptions trunk_options{};
  trunk_options.connection_context = city::wire::ConnectionContext::kTrunkContinue;
  const auto trunk =
      add_connection_by_category(state, pole_a, pole_b, city::wire::ConnectionCategory::kLowVoltage, trunk_options);
  if (!trunk.ok) {
    return false;
  }

  FixtureConnectionOptions branch_options{};
  branch_options.connection_context = city::wire::ConnectionContext::kBranchAdd;
  const auto branch =
      add_connection_by_category(state, pole_b, pole_c, city::wire::ConnectionCategory::kLowVoltage, branch_options);
  if (!branch.ok) {
    return false;
  }
  return branch.value.port_a_id != trunk.value.port_b_id;
}

bool test_band_selection_deterministic_and_debug_integrity() {
  auto run_once = []() -> std::pair<ObjectId, city::wire::PortResolutionDebugRecord> {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return {city::wire::kInvalidObjectId, {}};
    }
    const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
    city::wire::Transformd b{};
    b.position = {12.0, 0.0, 0.0};
    const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
    (void)state.ApplyPoleType(pole_a, type_ids.front());
    (void)state.ApplyPoleType(pole_b, type_ids.front());

  FixtureConnectionOptions options{};
    options.connection_context = city::wire::ConnectionContext::kCornerPass;
    options.branch_index = 3;
    const auto result = add_connection_by_category(state, pole_a, pole_b, city::wire::ConnectionCategory::kLowVoltage, options);
    if (!result.ok || state.view().port_resolution_debug_records().empty()) {
      return {city::wire::kInvalidObjectId, {}};
    }
    return {result.value.port_a_id, state.view().port_resolution_debug_records().back()};
  };

  const auto first = run_once();
  const auto second = run_once();
  if (first.first == city::wire::kInvalidObjectId || second.first == city::wire::kInvalidObjectId ||
      first.first != second.first) {
    return false;
  }
  if (first.second.candidates.empty()) {
    return false;
  }
  return has_selected_port_in_candidates(first.second);
}

bool test_generate_from_guide_reverse_mode_position_symmetry() {
  auto run_mode = [](city::wire::PathDirectionMode mode) {
    CoreState state;
    struct ModeResult {
      bool ok = false;
      std::vector<city::wire::Vec3d> poles{};
      std::vector<city::wire::Vec3d> guide{};
    };
    ModeResult out{};
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return out;
    }
    city::wire::BackboneSpec req{};
    req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 6.0, 0.0}};
    req.interval_m = 6.0;
    req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, city::wire::BundleKind::kLowVoltage);
    req.direction_mode = mode;
    const auto result = state.GenerateFromBackboneSpec(req);
    if (!result.ok) {
      return out;
    }
    out.ok = true;
    out.guide = req.path.polyline;
    out.poles.reserve(result.value.generated_pole_ids.size());
    for (ObjectId pole_id : result.value.generated_pole_ids) {
      const auto* pole = state.view().edit_state().poles.find(pole_id);
      if (pole == nullptr) {
        out.ok = false;
        return out;
      }
      const auto& p = pole->world_transform.position;
      out.poles.push_back(p);
    }
    return out;
  };

  auto point_to_segment_distance = [](const city::wire::Vec3d& p, const city::wire::Vec3d& a,
                                      const city::wire::Vec3d& b) -> double {
    const city::wire::Vec3d ab = b - a;
    const city::wire::Vec3d ap = p - a;
    const double ab2 = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
    if (ab2 <= 1e-12) {
      const city::wire::Vec3d d = p - a;
      return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    }
    const double t = std::clamp((ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / ab2, 0.0, 1.0);
    const city::wire::Vec3d q{a.x + ab.x * t, a.y + ab.y * t, a.z + ab.z * t};
    const city::wire::Vec3d d = p - q;
    return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
  };

  auto is_on_polyline = [&](const city::wire::Vec3d& p, const std::vector<city::wire::Vec3d>& guide) -> bool {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < guide.size(); ++i) {
      best = std::min(best, point_to_segment_distance(p, guide[i], guide[i + 1]));
    }
    return best <= 1e-6;
  };

  const auto forward = run_mode(city::wire::PathDirectionMode::kForward);
  const auto reverse = run_mode(city::wire::PathDirectionMode::kReverse);
  if (!forward.ok || !reverse.ok) {
    return false;
  }
  if (forward.poles.size() != reverse.poles.size() || forward.poles.size() < 2) {
    return false;
  }
  for (const auto& p : forward.poles) {
    if (!is_on_polyline(p, forward.guide)) {
      return false;
    }
  }
  for (const auto& p : reverse.poles) {
    if (!is_on_polyline(p, reverse.guide)) {
      return false;
    }
  }
  const city::wire::Vec3d start = forward.guide.front();
  const city::wire::Vec3d end = forward.guide.back();
  bool has_start = false;
  bool has_end = false;
  for (const auto& p : forward.poles) {
    has_start = has_start || almost_equal(p, start, 1e-6);
    has_end = has_end || almost_equal(p, end, 1e-6);
  }
  return has_start && has_end;
}

bool test_generate_from_guide_respects_avoid_constraints() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  city::wire::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, city::wire::BundleKind::kLowVoltage);
  req.constraints.avoid_points = {{10.0, 0.0, 0.0}};
  req.constraints.avoid_radius_m = 1.0;
  const auto result = state.GenerateFromBackboneSpec(req);
  if (!result.ok || result.value.generated_pole_ids.empty()) {
    return false;
  }
  const city::wire::Vec3d avoid = req.constraints.avoid_points.front();
  const double avoid_r2 = req.constraints.avoid_radius_m * req.constraints.avoid_radius_m;
  for (ObjectId pole_id : result.value.generated_pole_ids) {
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr) {
      return false;
    }
    const auto& p = pole->world_transform.position;
    const city::wire::Vec3d d{p.x - avoid.x, p.y - avoid.y, p.z - avoid.z};
    const double d2 = d.x * d.x + d.y * d.y + d.z * d.z;
    if (d2 <= avoid_r2 + 1e-9) {
      return false;
    }
  }
  return true;
}

bool test_preferred_side_uses_geometry() {
  auto pick_side = [](double peer_y) -> city::wire::SlotSide {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return city::wire::SlotSide::kCenter;
    }
    const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
    city::wire::Transformd b{};
    b.position = {10.0, peer_y, 0.0};
    const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
    (void)state.ApplyPoleType(pole_a, type_ids.front());
    (void)state.ApplyPoleType(pole_b, type_ids.front());
    state.clear_port_resolution_debug_records();

  FixtureConnectionOptions options{};
    options.connection_context = city::wire::ConnectionContext::kBranchAdd;
    options.branch_index = 7; // deliberately fixed; geometry should dominate.
    const auto add = add_connection_by_category(state, pole_a, pole_b, city::wire::ConnectionCategory::kLowVoltage, options);
    if (!add.ok) {
      return city::wire::SlotSide::kCenter;
    }
    for (const auto& debug : state.view().port_resolution_debug_records()) {
      if (debug.pole_id != pole_a || debug.selected_port_id == city::wire::kInvalidObjectId) {
        continue;
      }
      const auto* selected_port = state.view().edit_state().ports.find(debug.selected_port_id);
      if (selected_port == nullptr) {
        return city::wire::SlotSide::kCenter;
      }
      return selected_port->template_side;
    }
    return city::wire::SlotSide::kCenter;
  };

  const city::wire::SlotSide right = pick_side(+8.0);
  const city::wire::SlotSide left = pick_side(-8.0);
  return right == city::wire::SlotSide::kRight && left == city::wire::SlotSide::kLeft;
}

bool test_world_up_and_lateral_axis_are_consistent() {
  const city::wire::Vec3d forward{1.0, 2.0, 0.0};
  const city::wire::Vec3d lateral = city::wire::ComputeLateralAxis(forward);
  return almost_equal(city::wire::Dot(lateral, city::wire::WorldUp()), 0.0, 1e-9) &&
         almost_equal(city::wire::Dot(lateral, forward), 0.0, 1e-9) &&
         city::wire::LengthSquared(lateral) > 0.99;
}

bool test_build_pole_frame_roundtrips_local_point_under_tilt() {
  city::wire::Transformd tf{};
  tf.position = {3.0, -4.0, 1.5};
  tf.rotation_euler_deg = {7.0, -5.0, 15.0};
  const city::wire::PoleFrame frame = city::wire::BuildPoleFrame(tf, 33.0);
  const city::wire::PoleFrame other_layout = city::wire::BuildPoleFrame(tf, -72.0);
  const city::wire::Vec3d local{0.2, 0.8, 6.0};
  const city::wire::Vec3d world = city::wire::LocalPointToWorld(frame, local);
  const city::wire::Vec3d roundtrip = city::wire::WorldPointToLocal(frame, world);
  return almost_equal(local, roundtrip, 1e-9) &&
         almost_equal(frame.up, other_layout.up, 1e-12);
}

bool test_detail_curve_builds_with_endpoint_position_and_tangent_constraints() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.tangent_length_hint_m = 3.0;

  city::wire::CurveConstraint end{};
  end.point = {12.0, 0.0, 5.0};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.tangent_length_hint_m = 3.0;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 17);
  const city::wire::Vec3d tangent0 = curve.EvaluateTangent(0.0);
  const city::wire::Vec3d tangent1 = curve.EvaluateTangent(1.0);
  return almost_equal(curve.EvaluatePosition(0.0), start.point, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), end.point, 1e-9) &&
         city::wire::Dot(tangent0, city::wire::Vec3d{1.0, 0.0, 0.0}) > 0.98 &&
         city::wire::Dot(tangent1, city::wire::Vec3d{1.0, 0.0, 0.0}) > 0.98;
}

bool test_detail_curve_sag_preserves_endpoints_and_supports_length_queries() {
  city::wire::CurveConstraint line_start{};
  line_start.point = {0.0, 0.0, 6.0};
  line_start.tangent_dir = {1.0, 0.0, 0.0};
  city::wire::CurveConstraint line_end{};
  line_end.point = {16.0, 0.0, 6.0};
  line_end.tangent_dir = {1.0, 0.0, 0.0};
  const city::wire::DetailCurve line = city::wire::BuildDetailCurve(line_start, line_end, 33);

  city::wire::CurveConstraint sag_start = line_start;
  city::wire::CurveConstraint sag_end = line_end;
  sag_start.sag_hint = 0.03;
  sag_end.sag_hint = 0.03;
  const city::wire::DetailCurve sag = city::wire::BuildDetailCurve(sag_start, sag_end, 33);
  if (!almost_equal(sag.EvaluatePosition(0.0), line.EvaluatePosition(0.0), 1e-9) ||
      !almost_equal(sag.EvaluatePosition(1.0), line.EvaluatePosition(1.0), 1e-9)) {
    return false;
  }
  if (!is_monotonic([&]() {
        std::vector<double> lengths{};
        lengths.reserve(sag.arc_length_table.size());
        for (const auto& sample : sag.arc_length_table) {
          lengths.push_back(sample.arc_length_m);
        }
        return lengths;
      }())) {
    return false;
  }
  const city::wire::Vec3d line_mid = line.PositionAtLength(line.Length() * 0.5);
  const city::wire::Vec3d sag_mid = sag.PositionAtLength(sag.Length() * 0.5);
  return sag.Length() > 0.0 &&
         city::wire::HeightAlongWorldUp(sag_mid) < city::wire::HeightAlongWorldUp(line_mid) &&
         almost_equal(sag.PositionAtLength(0.0), sag.EvaluatePosition(0.0), 1e-9) &&
         almost_equal(sag.PositionAtLength(sag.Length()), sag.EvaluatePosition(1.0), 1e-9);
}

bool test_detail_curve_sag_uses_catenary_like_support_slope() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 6.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.sag_hint = 0.04;

  city::wire::CurveConstraint end{};
  end.point = {18.0, 0.0, 6.0};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.sag_hint = 0.04;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 33);
  const city::wire::Vec3d tangent0 = curve.EvaluateTangent(0.0);
  const city::wire::Vec3d tangent1 = curve.EvaluateTangent(1.0);
  return curve.sag_amplitude_m > 0.0 && tangent0.z < -0.02 && tangent1.z > 0.02;
}

bool test_detail_curve_near_straight_tangent_hints_do_not_wobble_sideways() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {1.0, 0.18, 0.0};
  start.tangent_length_hint_m = 6.0;

  city::wire::CurveConstraint end{};
  end.point = {20.0, 0.0, 5.0};
  end.tangent_dir = {1.0, -0.18, 0.0};
  end.tangent_length_hint_m = 6.0;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 25);
  double max_abs_y = 0.0;
  for (const city::wire::Vec3d& sample : curve.sample_points) {
    max_abs_y = std::max(max_abs_y, std::abs(sample.y));
  }
  return curve.quality.shape_policy == city::wire::CurveShapePolicyKind::kNearStraight && max_abs_y <= 0.03;
}

bool test_detail_curve_smooth_pass_suppresses_lateral_bend() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {0.90, 0.43, 0.0};
  start.tangent_length_hint_m = 8.0;
  start.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;

  city::wire::CurveConstraint end{};
  end.point = {18.0, 0.0, 5.0};
  end.tangent_dir = {0.90, 0.43, 0.0};
  end.tangent_length_hint_m = 8.0;
  end.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 33);
  double max_abs_y = 0.0;
  for (const city::wire::Vec3d& sample : curve.sample_points) {
    max_abs_y = std::max(max_abs_y, std::abs(sample.y));
  }
  return curve.quality.shape_policy == city::wire::CurveShapePolicyKind::kSmoothPass &&
         curve.quality.adopted_continuity == city::wire::DetailCurveContinuityMode::kG2 &&
         curve.quality.lateral_suppression >= 0.99 && curve.quality.start_lateral_ratio_limit <= 1e-9 &&
         curve.quality.end_lateral_ratio_limit <= 1e-9 && max_abs_y <= 0.03;
}

bool test_detail_curve_sharp_corner_stays_compact() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {0.55, 0.84, 0.0};
  start.tangent_length_hint_m = 8.0;
  start.corner_pass = true;
  start.corner_angle_deg = 60.0;
  start.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;

  city::wire::CurveConstraint end{};
  end.point = {18.0, 0.0, 5.0};
  end.tangent_dir = {0.55, -0.84, 0.0};
  end.tangent_length_hint_m = 8.0;
  end.corner_pass = true;
  end.corner_angle_deg = 60.0;
  end.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 33);
  double max_abs_y = 0.0;
  for (const city::wire::Vec3d& sample : curve.sample_points) {
    max_abs_y = std::max(max_abs_y, std::abs(sample.y));
  }
  return curve.quality.shape_policy == city::wire::CurveShapePolicyKind::kSharpCorner &&
         curve.quality.adopted_continuity == city::wire::DetailCurveContinuityMode::kG1 &&
         curve.quality.continuity_reason == city::wire::DetailCurveContinuityReason::kCornerPass &&
         max_abs_y <= 0.08;
}

bool test_detail_curve_offset_endpoint_uses_offset_endpoints() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 4.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.endpoint_offset = {0.5, 0.0, 0.0};
  start.endpoint_mode = city::wire::CurveEndpointMode::kOffsetEndpoint;

  city::wire::CurveConstraint end{};
  end.point = {10.0, 0.0, 4.0};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.endpoint_offset = {-0.5, 0.0, 0.0};
  end.endpoint_mode = city::wire::CurveEndpointMode::kOffsetEndpoint;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 17);
  return almost_equal(curve.EvaluatePosition(0.0), city::wire::Vec3d{0.5, 0.0, 4.0}, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), city::wire::Vec3d{9.5, 0.0, 4.0}, 1e-9) &&
         !almost_equal(curve.EvaluatePosition(0.0), start.point, 1e-9) &&
         !almost_equal(curve.EvaluatePosition(1.0), end.point, 1e-9);
}

bool test_detail_curve_acute_case_applies_quality_fallback() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {-1.0, 0.0, 0.0};
  start.tangent_length_hint_m = 9.0;
  start.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;

  city::wire::CurveConstraint end{};
  end.point = {8.0, 1.0, 5.0};
  end.tangent_dir = {0.0, -1.0, 0.0};
  end.tangent_length_hint_m = 9.0;
  end.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 21);
  city::wire::Vec3d chord = end.point - start.point;
  const double chord_length = std::sqrt(city::wire::LengthSquared(chord));
  if (chord_length <= 1e-9) {
    return false;
  }
  chord = city::wire::ScaleVec(chord, 1.0 / chord_length);
  double max_deviation = 0.0;
  const city::wire::Vec3d expected_start = start.point;
  const city::wire::Vec3d expected_end = end.point;
  for (const city::wire::Vec3d& sample : curve.sample_points) {
    if (!std::isfinite(sample.x) || !std::isfinite(sample.y) || !std::isfinite(sample.z)) {
      return false;
    }
    const city::wire::Vec3d delta = sample - start.point;
    const double progress = city::wire::Dot(delta, chord);
    const city::wire::Vec3d along = city::wire::ScaleVec(chord, progress);
    max_deviation = std::max(max_deviation, std::sqrt(std::max(0.0, city::wire::LengthSquared(delta - along))));
  }
  return curve.quality.adopted_continuity == city::wire::DetailCurveContinuityMode::kG1 &&
         curve.quality.continuity_reason == city::wire::DetailCurveContinuityReason::kConflictingTangents &&
         curve.quality.degraded_to_g1 &&
         almost_equal(curve.EvaluatePosition(0.0), expected_start, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), expected_end, 1e-9) &&
         max_deviation <= chord_length * 0.80 + 1e-6;
}

bool test_detail_curve_long_pass_through_prefers_g2() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 6.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.tangent_length_hint_m = 6.0;
  start.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;

  city::wire::CurveConstraint end{};
  end.point = {20.0, 0.0, 6.0};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.tangent_length_hint_m = 6.0;
  end.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 33);
  return curve.quality.requested_policy == city::wire::CableContinuityPolicyHint::kPreferG2 &&
         curve.quality.adopted_continuity == city::wire::DetailCurveContinuityMode::kG2 &&
         curve.quality.continuity_reason == city::wire::DetailCurveContinuityReason::kSmoothPassThrough &&
         curve.quality.attempted_g2 && !curve.quality.degraded_to_g1 &&
         curve.quality.handle_length_start_m > 0.0 && curve.quality.handle_length_end_m > 0.0;
}

bool test_detail_curve_short_span_falls_back_to_g1() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.tangent_length_hint_m = 1.5;
  start.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;

  city::wire::CurveConstraint end{};
  end.point = {2.0, 0.0, 5.0};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.tangent_length_hint_m = 1.5;
  end.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 17);
  return curve.quality.adopted_continuity == city::wire::DetailCurveContinuityMode::kG1 &&
         curve.quality.continuity_reason == city::wire::DetailCurveContinuityReason::kShortSpan &&
         curve.quality.degraded_to_g1 &&
         almost_equal(curve.EvaluatePosition(0.0), start.point, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), end.point, 1e-9);
}

bool test_detail_curve_branch_pass_uses_g1_and_preserves_endpoint_constraints() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.tangent_length_hint_m = 4.0;
  start.endpoint_mode = city::wire::CurveEndpointMode::kOffsetEndpoint;
  start.endpoint_offset = {0.4, 0.0, 0.0};
  start.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;
  start.pass_mode = city::wire::CurvePassMode::kBranch;

  city::wire::CurveConstraint end{};
  end.point = {12.0, 2.0, 5.0};
  const city::wire::Vec3d expected_end_tangent{0.6, 0.8, 0.0};
  end.tangent_dir = expected_end_tangent;
  end.tangent_length_hint_m = 4.0;
  end.endpoint_mode = city::wire::CurveEndpointMode::kOffsetEndpoint;
  end.endpoint_offset = {0.0, 0.35, 0.0};
  end.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;
  end.pass_mode = city::wire::CurvePassMode::kBranch;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 25);
  const city::wire::Vec3d tangent0 = curve.EvaluateTangent(0.0);
  const city::wire::Vec3d tangent1 = curve.EvaluateTangent(1.0);
  return curve.quality.shape_policy == city::wire::CurveShapePolicyKind::kBranchPass &&
         curve.quality.adopted_continuity == city::wire::DetailCurveContinuityMode::kG1 &&
         curve.quality.continuity_reason == city::wire::DetailCurveContinuityReason::kBranchPass &&
         curve.quality.degraded_to_g1 &&
         almost_equal(curve.EvaluatePosition(0.0), start.point + start.endpoint_offset, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), end.point + end.endpoint_offset, 1e-9) &&
         city::wire::Dot(tangent0, city::wire::Vec3d{1.0, 0.0, 0.0}) > 0.92 &&
         city::wire::Dot(tangent1, expected_end_tangent) > 0.74;
}

bool test_detail_curve_branch_long_span_suppresses_sideways_runout() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.5};
  start.tangent_dir = {0.20, 0.98, 0.0};
  start.tangent_length_hint_m = 8.0;
  start.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;
  start.pass_mode = city::wire::CurvePassMode::kBranch;

  city::wire::CurveConstraint end{};
  end.point = {24.0, 0.0, 5.5};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.tangent_length_hint_m = 8.0;
  end.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;
  end.pass_mode = city::wire::CurvePassMode::kBranch;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 41);
  double max_abs_y = 0.0;
  for (std::size_t i = 0; i < curve.sample_points.size(); ++i) {
    const double abs_y = std::abs(curve.sample_points[i].y);
    max_abs_y = std::max(max_abs_y, abs_y);
  }
  return curve.quality.shape_policy == city::wire::CurveShapePolicyKind::kBranchPass &&
         curve.quality.start_tangent_rule == city::wire::DetailCurveEndpointTangentRule::kBranchChordPriority &&
         curve.quality.handle_length_start_m <= curve.quality.start_departure_length_m + 1e-6 &&
         curve.quality.start_departure_length_m <= 1.10 + 1e-6 &&
         curve.quality.start_support_weight < curve.quality.start_chord_weight && max_abs_y <= 0.40;
}

bool test_detail_curve_branch_short_span_keeps_more_local_departure_than_long_span() {
  auto build_branch_curve = [](double length_m) {
    city::wire::CurveConstraint start{};
    start.point = {0.0, 0.0, 5.5};
    start.tangent_dir = {0.20, 0.98, 0.0};
    start.tangent_length_hint_m = 6.0;
    start.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;
    start.pass_mode = city::wire::CurvePassMode::kBranch;

    city::wire::CurveConstraint end{};
    end.point = {length_m, 0.0, 5.5};
    end.tangent_dir = {1.0, 0.0, 0.0};
    end.tangent_length_hint_m = 6.0;
    end.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;
    end.pass_mode = city::wire::CurvePassMode::kBranch;
    return city::wire::BuildDetailCurve(start, end, 33);
  };

  const city::wire::DetailCurve short_curve = build_branch_curve(6.0);
  const city::wire::DetailCurve long_curve = build_branch_curve(24.0);
  return short_curve.quality.start_support_weight > long_curve.quality.start_support_weight &&
         short_curve.quality.start_lateral_ratio_limit > long_curve.quality.start_lateral_ratio_limit &&
         short_curve.quality.start_tangent_rule == city::wire::DetailCurveEndpointTangentRule::kBranchChordPriority &&
         long_curve.quality.start_tangent_rule == city::wire::DetailCurveEndpointTangentRule::kBranchChordPriority;
}

bool test_detail_curve_main_sag_reads_stronger_than_branch() {
  auto midpoint_drop = [](city::wire::CurvePassMode pass_mode) {
    city::wire::CurveConstraint start{};
    start.point = {0.0, 0.0, 6.0};
    start.tangent_dir = {1.0, 0.0, 0.0};
    start.tangent_length_hint_m = 6.0;
    start.sag_hint = 0.03;
    start.pass_mode = pass_mode;

    city::wire::CurveConstraint end{};
    end.point = {20.0, 0.0, 6.0};
    end.tangent_dir = {1.0, 0.0, 0.0};
    end.tangent_length_hint_m = 6.0;
    end.sag_hint = 0.03;
    end.pass_mode = pass_mode;

    const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 33);
    const double midpoint_z = city::wire::HeightAlongWorldUp(curve.EvaluatePosition(0.5));
    return std::pair<double, city::wire::DetailCurve>{6.0 - midpoint_z, curve};
  };

  const auto main_result = midpoint_drop(city::wire::CurvePassMode::kPassThrough);
  const auto branch_result = midpoint_drop(city::wire::CurvePassMode::kBranch);
  return main_result.second.quality.sag_pass_scale > branch_result.second.quality.sag_pass_scale &&
         main_result.first > branch_result.first + 0.08 && branch_result.first > 0.0;
}

bool test_detail_curve_large_height_difference_uses_composite_segments() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 11.0};
  start.tangent_dir = {0.96, 0.0, -0.28};
  start.tangent_length_hint_m = 5.0;
  start.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;
  start.profile_hint = city::wire::CurveProfileHint::kCompositeHeightTransition;

  city::wire::CurveConstraint end{};
  end.point = {20.0, 0.0, 4.0};
  end.tangent_dir = {0.98, 0.0, -0.16};
  end.tangent_length_hint_m = 5.0;
  end.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;
  end.profile_hint = city::wire::CurveProfileHint::kCompositeHeightTransition;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 41);
  if ((curve.segments.empty() ? 1u : curve.segments.size()) <= 1 || curve.sample_points.size() < 5 ||
      curve.arc_length_table.empty()) {
    return false;
  }
  if (!almost_equal(curve.EvaluatePosition(0.0), start.point, 1e-9) ||
      !almost_equal(curve.EvaluatePosition(1.0), end.point, 1e-9)) {
    return false;
  }
  if (!is_monotonic([&]() {
        std::vector<double> lengths{};
        lengths.reserve(curve.arc_length_table.size());
        for (const auto& sample : curve.arc_length_table) {
          lengths.push_back(sample.arc_length_m);
        }
        return lengths;
      }())) {
    return false;
  }
  const double early_z = city::wire::HeightAlongWorldUp(curve.EvaluatePosition(0.18));
  const double late_z = city::wire::HeightAlongWorldUp(curve.EvaluatePosition(0.82));
  return curve.quality.shape_policy == city::wire::CurveShapePolicyKind::kSmoothPass &&
         curve.quality.adopted_continuity == city::wire::DetailCurveContinuityMode::kG2 &&
         early_z > late_z && curve.Length() > 0.0;
}

bool test_detail_curve_prefer_g1_policy_is_explicit_not_degraded() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.tangent_length_hint_m = 5.0;
  start.sag_hint = 0.03;
  start.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG1;

  city::wire::CurveConstraint end{};
  end.point = {16.0, 0.0, 5.0};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.tangent_length_hint_m = 5.0;
  end.sag_hint = 0.03;
  end.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG1;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 33);
  return curve.quality.requested_policy == city::wire::CableContinuityPolicyHint::kPreferG1 &&
         curve.quality.adopted_continuity == city::wire::DetailCurveContinuityMode::kG1 &&
         curve.quality.continuity_reason == city::wire::DetailCurveContinuityReason::kPolicyPreferG1 &&
         !curve.quality.attempted_g2 && !curve.quality.degraded_to_g1 && curve.sag_amplitude_m > 0.0 &&
         almost_equal(curve.EvaluatePosition(0.0), start.point, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), end.point, 1e-9) &&
         curve.Length() > 0.0 && !curve.arc_length_table.empty();
}

bool test_detail_curve_via_attachment_policy_uses_offset_endpoint_and_g1() {
  city::wire::CurveConstraint start{};
  start.point = {0.0, 0.0, 4.5};
  start.tangent_dir = {0.96, 0.24, 0.0};
  start.tangent_length_hint_m = 4.0;
  start.endpoint_mode = city::wire::CurveEndpointMode::kOffsetEndpoint;
  start.endpoint_offset = {0.45, 0.20, 0.0};
  start.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;

  city::wire::CurveConstraint end{};
  end.point = {12.0, 0.0, 4.5};
  end.tangent_dir = {0.96, -0.20, 0.0};
  end.tangent_length_hint_m = 4.0;
  end.endpoint_mode = city::wire::CurveEndpointMode::kOffsetEndpoint;
  end.endpoint_offset = {-0.45, 0.15, 0.0};
  end.continuity_preference = city::wire::CableContinuityPolicyHint::kPreferG2;

  const city::wire::DetailCurve curve = city::wire::BuildDetailCurve(start, end, 25);
  return curve.quality.shape_policy == city::wire::CurveShapePolicyKind::kViaAttachment &&
         curve.quality.adopted_continuity == city::wire::DetailCurveContinuityMode::kG1 &&
         curve.quality.continuity_reason == city::wire::DetailCurveContinuityReason::kEndpointConstraintPriority &&
         almost_equal(curve.EvaluatePosition(0.0), start.point + start.endpoint_offset, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), end.point + end.endpoint_offset, 1e-9);
}

bool test_cable_curve_parabolic_sag_has_exact_semantics() {
  namespace cable_curve = city::wire::geometry::curve;
  cable_curve::CableCurveInput input{};
  input.start = {0.0, 2.0, 8.0};
  input.end = {20.0, 2.0, 10.0};
  input.canonical_dir = {1.0, 0.0, 0.0};
  input.sag_m = 1.25;
  const auto built = cable_curve::BuildCableCurve(input);
  if (!built.ok || built.value.samples.size() < 3) {
    return false;
  }
  const auto& midpoint = built.value.samples[built.value.samples.size() / 2];
  const city::wire::Vec3d linear_midpoint = city::wire::ScaleVec(input.start + input.end, 0.5);
  const city::wire::DetailCurve detail = cable_curve::ToDetailCurve(input, built.value);
  const bool finite_bounds = std::isfinite(built.value.bounds.min.x) && std::isfinite(built.value.bounds.min.y) &&
                             std::isfinite(built.value.bounds.min.z) && std::isfinite(built.value.bounds.max.x) &&
                             std::isfinite(built.value.bounds.max.y) && std::isfinite(built.value.bounds.max.z) &&
                             built.value.bounds.min.x <= built.value.bounds.max.x &&
                             built.value.bounds.min.y <= built.value.bounds.max.y &&
                             built.value.bounds.min.z <= built.value.bounds.max.z;
  return almost_equal(built.value.samples.front().position, input.start) &&
         almost_equal(built.value.samples.back().position, input.end) &&
         almost_equal(midpoint.position, linear_midpoint + city::wire::Vec3d{0.0, 0.0, -input.sag_m}, 1e-9) &&
         almost_equal(midpoint.position.y, 2.0, 1e-9) &&
         almost_equal(detail.EvaluatePosition(0.5), midpoint.position, 1e-9) &&
         almost_equal(detail.sag_amplitude_m, input.sag_m, 1e-9) && finite_bounds &&
         detail.arc_length_table.size() == built.value.samples.size() &&
         detail.distance_attributes.arc_length_m.size() == built.value.samples.size();
}

bool test_cable_curve_samples_have_stable_orthonormal_frames() {
  namespace cable_curve = city::wire::geometry::curve;
  cable_curve::CableCurveInput input{};
  input.start = {-3.0, 4.0, 9.0};
  input.end = {17.0, 4.0, 11.0};
  input.canonical_dir = {1.0, 0.0, 0.0};
  input.sag_m = 0.8;
  const auto built = cable_curve::BuildCableCurve(input);
  if (!built.ok) {
    return false;
  }
  for (const cable_curve::CableCurveSample& sample : built.value.samples) {
    const auto finite = [](const city::wire::Vec3d& value) {
      return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    };
    if (!finite(sample.tangent) || !finite(sample.normal) || !finite(sample.binormal) ||
        !almost_equal(city::wire::Length(sample.tangent), 1.0, 1e-9) ||
        !almost_equal(city::wire::Length(sample.normal), 1.0, 1e-9) ||
        !almost_equal(city::wire::Length(sample.binormal), 1.0, 1e-9) ||
        std::abs(city::wire::Dot(sample.tangent, sample.normal)) > 1e-9 ||
        std::abs(city::wire::Dot(sample.tangent, sample.binormal)) > 1e-9 ||
        std::abs(city::wire::Dot(sample.normal, sample.binormal)) > 1e-9) {
      return false;
    }
  }
  return true;
}

bool test_cable_curve_reverse_keeps_canonical_lateral_frame() {
  namespace cable_curve = city::wire::geometry::curve;
  cable_curve::CableCurveInput forward{};
  forward.start = {0.0, 0.0, 7.0};
  forward.end = {16.0, 0.0, 7.0};
  forward.canonical_dir = {1.0, 0.0, 0.0};
  forward.sag_m = 0.5;
  cable_curve::CableCurveInput reverse = forward;
  std::swap(reverse.start, reverse.end);
  const auto a = cable_curve::BuildCableCurve(forward);
  const auto b = cable_curve::BuildCableCurve(reverse);
  if (!a.ok || !b.ok || a.value.samples.size() != b.value.samples.size()) {
    return false;
  }
  for (std::size_t index = 0; index < a.value.samples.size(); ++index) {
    const std::size_t reverse_index = b.value.samples.size() - 1 - index;
    if (!almost_equal(a.value.samples[index].position, b.value.samples[reverse_index].position, 1e-9) ||
        !almost_equal(a.value.samples[index].binormal, b.value.samples[reverse_index].binormal, 1e-9)) {
      return false;
    }
  }
  return true;
}

bool test_cable_curve_tessellation_grows_with_length_and_sag() {
  namespace cable_curve = city::wire::geometry::curve;
  cable_curve::CableCurveInput short_flat{};
  short_flat.end = {4.0, 0.0, 0.0};
  cable_curve::CableCurveInput long_flat = short_flat;
  long_flat.end = {40.0, 0.0, 0.0};
  cable_curve::CableCurveInput short_sag = short_flat;
  short_sag.sag_m = 2.0;
  const std::size_t short_count = cable_curve::ResolveSegmentCount(short_flat);
  return cable_curve::ResolveSegmentCount(long_flat) > short_count &&
         cable_curve::ResolveSegmentCount(short_sag) > short_count;
}

bool test_cable_curve_degenerate_and_vertical_inputs_are_deterministic() {
  namespace cable_curve = city::wire::geometry::curve;
  cable_curve::CableCurveInput point{};
  point.start = {2.0, 3.0, 4.0};
  point.end = point.start;
  point.canonical_dir = {0.0, 0.0, 1.0};
  const auto point_curve = cable_curve::BuildCableCurve(point);
  cable_curve::CableCurveInput vertical = point;
  vertical.end = {2.0, 3.0, 14.0};
  const auto first = cable_curve::BuildCableCurve(vertical);
  const auto second = cable_curve::BuildCableCurve(vertical);
  cable_curve::CableCurveInput unsupported = vertical;
  unsupported.method = static_cast<cable_curve::CurveMethod>(255);
  const auto rejected = cable_curve::BuildCableCurve(unsupported);
  return point_curve.ok && first.ok && second.ok && !rejected.ok &&
         point_curve.value.samples.size() >= 2 && almost_equal(point_curve.value.length_m, 0.0) &&
         first.value.samples.size() == second.value.samples.size() &&
         almost_equal(first.value.samples.front().normal, second.value.samples.front().normal) &&
         almost_equal(first.value.samples.front().binormal, second.value.samples.front().binormal);
}

bool test_cable_curve_uses_endpoint_tangent_hints_when_provided() {
  namespace cable_curve = city::wire::geometry::curve;
  cable_curve::CableCurveInput input{};
  input.start = {0.0, 0.0, 6.0};
  input.end = {8.0, 4.0, 5.0};
  input.canonical_dir = {1.0, 0.0, 0.0};
  input.start_tangent_hint = {0.0, 1.0, -0.2};
  input.end_tangent_hint = {1.0, 0.0, 0.1};
  input.has_start_tangent_hint = true;
  input.has_end_tangent_hint = true;
  input.sag_m = 0.8;
  const auto built = cable_curve::BuildCableCurve(input);
  if (!built.ok || built.value.samples.size() < 4) {
    return false;
  }
  city::wire::Vec3d expected_start = input.start_tangent_hint;
  city::wire::Vec3d expected_end = input.end_tangent_hint;
  (void)city::wire::Normalize(&expected_start);
  (void)city::wire::Normalize(&expected_end);
  const city::wire::Vec3d start_tangent = built.value.samples.front().tangent;
  const city::wire::Vec3d end_tangent = built.value.samples.back().tangent;
  return city::wire::Dot(start_tangent, expected_start) > 0.999 &&
         city::wire::Dot(end_tangent, expected_end) > 0.999;
}

bool test_cable_curve_one_sided_hint_keeps_natural_far_end() {
  namespace cable_curve = city::wire::geometry::curve;
  cable_curve::CableCurveInput reference{};
  reference.start = {0.0, 0.0, 6.0};
  reference.end = {32.0, 0.0, 6.0};
  reference.canonical_dir = {1.0, 0.0, 0.0};
  reference.sag_m = 0.96;
  const auto plain = cable_curve::BuildCableCurve(reference);
  if (!plain.ok || plain.value.samples.size() < 5) {
    return false;
  }
  const city::wire::Vec3d natural_start = plain.value.samples.front().tangent;
  const city::wire::Vec3d natural_end = plain.value.samples.back().tangent;

  cable_curve::CableCurveInput hinted = reference;
  hinted.start_tangent_hint = {0.9, 0.3, -0.12};
  hinted.has_start_tangent_hint = true;
  const auto built = cable_curve::BuildCableCurve(hinted);
  if (!built.ok || built.value.samples.size() < 5) {
    return false;
  }
  const city::wire::Vec3d far_end = built.value.samples.back().tangent;
  if (city::wire::Dot(far_end, natural_end) <= 0.9999) {
    return false;
  }
  if (far_end.z <= 0.05) {
    return false;
  }

  cable_curve::CableCurveInput natural_hinted = reference;
  natural_hinted.start_tangent_hint = natural_start;
  natural_hinted.has_start_tangent_hint = true;
  const auto equivalent = cable_curve::BuildCableCurve(natural_hinted);
  if (!equivalent.ok || equivalent.value.samples.size() != plain.value.samples.size()) {
    return false;
  }
  for (std::size_t i = 0; i < plain.value.samples.size(); ++i) {
    const city::wire::Vec3d delta =
        equivalent.value.samples[i].position - plain.value.samples[i].position;
    if (city::wire::Length(delta) > reference.sag_m * 0.05) {
      return false;
    }
  }
  return true;
}

bool test_hierarchical_variation_worldspace_is_continuous() {
  city::wire::VariationSettings settings{};
  settings.enabled = true;
  settings.global_seed = 77;
  settings.world_cell_size_m = 30.0;
  settings.world_bias_scale = 1.0;
  settings.flow_bias_scale = 0.0;
  settings.pole_delta_scale = 0.0;
  settings.local_jitter_scale = 0.0;

  city::wire::VariationContext near_a{};
  near_a.world_position = {10.0, 10.0, 0.0};
  city::wire::VariationContext near_b = near_a;
  near_b.world_position = {11.5, 10.5, 0.0};
  city::wire::VariationContext far = near_a;
  far.world_position = {75.0, -40.0, 0.0};

  const auto sample_a = city::wire::EvaluateHierarchicalVariation(settings, near_a);
  const auto sample_b = city::wire::EvaluateHierarchicalVariation(settings, near_b);
  const auto sample_far = city::wire::EvaluateHierarchicalVariation(settings, far);
  const double near_diff = std::abs(sample_a.world_bias - sample_b.world_bias);
  const double far_diff = std::abs(sample_a.world_bias - sample_far.world_bias);
  return near_diff < 0.10 && far_diff > near_diff;
}

void register_geometry_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C19_backbone_interval_generates_pole_line_basic",
                         "backbone interval generation creates a pole line with pole type", "Invariant", false,
                         test_backbone_interval_generates_pole_line_basic);
  test_registry::AddTest(tests, "C61_Phase48h_AcuteCorner_AutoWidenSpacing",
                         "Acute corners use per-edge rows without category-specific scaling", "Invariant", false,
                         test_acute_corner_auto_widens_lane_spacing);
  test_registry::AddTest(tests, "C58_Phase48h_Guide_ReverseSymmetry", "Guide reverse mode preserves generated pole position set", "Invariant", false, test_generate_from_guide_reverse_mode_position_symmetry);
  test_registry::AddTest(tests, "C59_Phase48h_Guide_AvoidConstraint", "Guide generation avoids forbidden radius around avoid_points", "Invariant", false, test_generate_from_guide_respects_avoid_constraints);
  test_registry::AddTest(tests, "C126_Coord_WorldUpLateralConsistency",
                         "WorldUp and ComputeLateralAxis stay perpendicular and normalized by property",
                         "Invariant", false, test_world_up_and_lateral_axis_are_consistent);
  test_registry::AddTest(tests, "C127_Coord_PoleFrameRoundtrip",
                         "BuildPoleFrame keeps local/world roundtrip stable under tilt",
                         "Invariant", false, test_build_pole_frame_roundtrips_local_point_under_tilt);
  test_registry::AddTest(tests, "C128_DetailCurve_UApi_EndpointConstraints",
                         "DetailCurve builds from endpoint position and tangent constraints with u-based evaluation",
                         "Invariant", false, test_detail_curve_builds_with_endpoint_position_and_tangent_constraints);
  test_registry::AddTest(tests, "C129_DetailCurve_SApi_SagAndLengthPlacement",
                         "DetailCurve sag preserves endpoints and supports length-based placement via arc table",
                         "Invariant", false, test_detail_curve_sag_preserves_endpoints_and_supports_length_queries);
  test_registry::AddTest(
      tests, "C141_DetailCurve_CatenarySupportSlope",
      "DetailCurve sag uses a catenary-like support slope instead of staying flat at the endpoints", "Invariant",
      false, test_detail_curve_sag_uses_catenary_like_support_slope);
  test_registry::AddTest(tests, "C142_DetailCurve_NearStraightNoSideWobble",
                         "Near-straight endpoint tangents do not introduce visible sideways wobble",
                         "Invariant", false, test_detail_curve_near_straight_tangent_hints_do_not_wobble_sideways);
  test_registry::AddTest(tests, "C143_DetailCurve_SmoothPass_SuppressesLateralBend",
                         "Smooth-pass policy keeps G2 continuity but suppresses top-view lateral wobble",
                         "Invariant", false, test_detail_curve_smooth_pass_suppresses_lateral_bend);
  test_registry::AddTest(tests, "C130_DetailCurve_OffsetEndpoint_UsesOffsetEndpoints",
                         "Derived endpoint-offset mode moves detail-curve endpoints without changing source support points",
                         "Invariant", false, test_detail_curve_offset_endpoint_uses_offset_endpoints);
  test_registry::AddTest(tests, "C131_DetailCurve_QualityFallback_AcuteCase",
                         "Acute/conflicting tangents trigger tangent fallback instead of excessive bulge",
                         "Invariant", false, test_detail_curve_acute_case_applies_quality_fallback);
  test_registry::AddTest(tests, "C148_DetailCurve_SharpCorner_StaysCompact",
                         "Sharp-corner policy shortens handles and degrades to G1 instead of producing a wide lateral peak",
                         "Invariant", false, test_detail_curve_sharp_corner_stays_compact);
  test_registry::AddTest(tests, "C144_DetailCurve_LongPassThrough_PreferG2",
                         "Long pass-through spans with smooth endpoint tangents adopt G2-preferred control-point strategy",
                         "Invariant", false, test_detail_curve_long_pass_through_prefers_g2);
  test_registry::AddTest(tests, "C145_DetailCurve_ShortSpan_FallsBackToG1",
                         "Short spans degrade to G1 instead of forcing a G2-style control-point layout",
                         "Invariant", false, test_detail_curve_short_span_falls_back_to_g1);
  test_registry::AddTest(tests, "C146_DetailCurve_BranchPass_PreservesEndpointConstraints",
                         "Branch-pass spans prefer G1 and keep offset endpoints plus endpoint tangents intact",
                         "Invariant", false, test_detail_curve_branch_pass_uses_g1_and_preserves_endpoint_constraints);
  test_registry::AddTest(tests, "C147_DetailCurve_PreferG1_IsExplicit",
                         "PreferG1 policy is an explicit continuity choice, not a failed G2 attempt, and sag still applies",
                         "Invariant", false, test_detail_curve_prefer_g1_policy_is_explicit_not_degraded);
  test_registry::AddTest(tests, "C149_DetailCurve_ViaAttachment_UsesAttachmentPolicy",
                         "ViaAttachment policy uses offset endpoints and falls back to endpoint-priority G1 instead of forcing smooth pass",
                         "Invariant", false, test_detail_curve_via_attachment_policy_uses_offset_endpoint_and_g1);
  test_registry::AddTest(tests, "C277_DetailCurve_HeightDifference_UsesCompositeSegments",
                         "Large height-difference smooth spans switch to composite segments while preserving endpoint and s-based evaluation",
                         "Invariant", false, test_detail_curve_large_height_difference_uses_composite_segments);
  test_registry::AddTest(tests, "C151_Variation_WorldspaceContinuous",
                         "Worldspace variation changes continuously across nearby positions",
                         "Invariant", false, test_hierarchical_variation_worldspace_is_continuous);
  test_registry::AddTest(tests, "C161_DetailCurve_BranchLongSpan_SuppressesSidewaysRunout",
                         "Long branch spans keep support departure local and suppress large sideways runout",
                         "Invariant", false, test_detail_curve_branch_long_span_suppresses_sideways_runout);
  test_registry::AddTest(tests, "C162_DetailCurve_BranchShortVsLong_LocalDepartureScaling",
                         "Branch local departure stays stronger on short spans and decays on long spans",
                         "Invariant", false, test_detail_curve_branch_short_span_keeps_more_local_departure_than_long_span);
  test_registry::AddTest(tests, "C163_DetailCurve_MainSag_StrongerThanBranch",
                         "Main spans keep a stronger sag read than branch spans without breaking endpoint constraints",
                         "Invariant", false, test_detail_curve_main_sag_reads_stronger_than_branch);
  test_registry::AddTest(tests, "C629_CableCurve_ParabolicSagSemantics",
                         "CableCurve preserves endpoints, applies exact midpoint sag, and has no top-view drift",
                         "Invariant", false, test_cable_curve_parabolic_sag_has_exact_semantics);
  test_registry::AddTest(tests, "C630_CableCurve_StableOrthonormalFrames",
                         "CableCurve samples expose finite normalized low-twist frames", "Invariant", false,
                         test_cable_curve_samples_have_stable_orthonormal_frames);
  test_registry::AddTest(tests, "C631_CableCurve_ReverseCanonicalFrame",
                         "CableCurve reverse traversal keeps canonical lateral frame orientation", "Invariant", false,
                         test_cable_curve_reverse_keeps_canonical_lateral_frame);
  test_registry::AddTest(tests, "C632_CableCurve_AdaptiveTessellation",
                         "CableCurve tessellation grows with curve length or sag", "Invariant", false,
                         test_cable_curve_tessellation_grows_with_length_and_sag);
  test_registry::AddTest(tests, "C633_CableCurve_DegenerateVerticalDeterministic",
                          "CableCurve handles zero-length and vertical spans deterministically and rejects unknown methods",
                          "Invariant", false, test_cable_curve_degenerate_and_vertical_inputs_are_deterministic);
  test_registry::AddTest(tests, "C658_CableCurve_EndpointTangentHintsAffectSamples",
                         "CableCurve consumes endpoint tangent hints in the generated samples", "Boundary", false,
                         test_cable_curve_uses_endpoint_tangent_hints_when_provided);
  test_registry::AddTest(tests, "C688_CableCurve_OneSidedHintKeepsNaturalFarEnd",
                         "CableCurve keeps the natural parabolic support tangent on the un-hinted end and matches the "
                         "plain sag path when the hint equals the natural tangent",
                         "Invariant", false, test_cable_curve_one_sided_hint_keeps_natural_far_end);
}

WIRE_REGISTER_TEST_SUITE(register_geometry_tests);

} // namespace
