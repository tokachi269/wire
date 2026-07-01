#include "registry.hpp"
#include "helpers.hpp"
#include "wire/core/core_test_hook.hpp"
#include "wire/core/coord_utils.hpp"

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
using wire::core::PortKind;
using wire::core::PortLayer;
using wire::core::SpanKind;
using wire::core::SpanLayer;

namespace {
std::optional<wire::core::PoleTypeDefinition> find_pole_type_by_name(const CoreState& state, const std::string& name) {
  for (const auto& [id, pole_type] : state.view().pole_types()) {
    if (pole_type.name == name) {
      wire::core::PoleTypeDefinition copy = pole_type;
      copy.id = id;
      return copy;
    }
  }
  return std::nullopt;
}

double polyline_length(const std::vector<wire::core::Vec3d>& points) {
  double length = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const wire::core::Vec3d delta = points[i + 1] - points[i];
    length += std::sqrt(wire::core::LengthSquared(delta));
  }
  return length;
}

wire::core::EditResult<wire::core::GenerateBundleFromPathResult>
generate_path(CoreState& state, const wire::core::BackboneInputSpec& path, double interval,
              wire::core::PoleTypeId pole_type_id) {
  wire::core::BackboneSpec spec{};
  spec.path = path;
  spec.interval_m = interval;
  spec.pole_type_id = pole_type_id;
  wire::core::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  spec.bundles.push_back(bundle);
  return state.GenerateFromBackboneSpec(spec);
}

bool test_backbone_interval_generates_pole_line_basic() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneInputSpec path{};
  path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  wire::core::BackboneSpec spec{};
  spec.path = path;
  spec.interval_m = 5.0;
  spec.pole_type_id = type_ids.front();
  wire::core::BackboneBundleSpec bundle{};
  bundle.bundle_template_id = bundle_template_for_category_test(wire::core::ConnectionCategory::kLowVoltage);
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
  auto lane_width_for_interior = [](double interior_deg) -> double {
    CoreState state;
    wire::core::LayoutSettings layout{};
    layout.angle_correction_enabled = true;
    layout.corner_threshold_deg = 5.0;
    layout.min_side_scale = 1.0;
    layout.max_side_scale = 1.8;
    if (!state.UpdateLayoutSettings(layout).ok) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    constexpr double kPi = 3.14159265358979323846;
    const double rad = interior_deg * (kPi / 180.0);
    const double out_heading = kPi - rad;
    wire::core::BackboneInputSpec path{};
    path.polyline = {
        {0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0},
        {10.0 + 10.0 * std::cos(out_heading), 10.0 * std::sin(out_heading), 0.0},
    };
    const auto gen = generate_path(state, path, polyline_length(path.polyline) + 1.0, type_ids.front());
    if (!gen.ok || gen.value.generated_pole_ids.size() != 3) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const auto* pole = state.view().edit_state().poles.find(gen.value.generated_pole_ids[1]);
    if (pole == nullptr) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const wire::core::Port* p_left = nullptr;
    const wire::core::Port* p_right = nullptr;
    for (const auto& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id != pole->id) {
        continue;
      }
      if (port.template_side == wire::core::SlotSide::kLeft) {
        p_left = &port;
      } else if (port.template_side == wire::core::SlotSide::kRight) {
        p_right = &port;
      }
    }
    if (p_left == nullptr || p_right == nullptr) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    double layout_yaw_deg = effective_pole_yaw_deg_test(*pole);
    if (const auto pole_view = state.view().inspect_pole(pole->id); pole_view.has_value() && pole_view->has_layout_yaw) {
      layout_yaw_deg = pole_view->layout_yaw_deg;
    }
    const wire::core::PoleFrame frame = wire::core::BuildPoleFrame(pole->world_transform, layout_yaw_deg);
    const wire::core::Vec3d local_left = wire::core::WorldPointToLocal(frame, p_left->world_position);
    const wire::core::Vec3d local_right = wire::core::WorldPointToLocal(frame, p_right->world_position);
    return std::abs(local_right.y - local_left.y);
  };

  const double width_acute = lane_width_for_interior(45.0);
  const double width_obtuse = lane_width_for_interior(120.0);
  if (!std::isfinite(width_acute) || !std::isfinite(width_obtuse)) {
    return false;
  }
  // Base template width is 0.8m between band centers -0.4 and +0.4.
  return width_acute > width_obtuse && width_acute > 0.8 + 1e-6;
}

bool test_band_selection_context_bias() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {10.0, 0.0, 0.0};
  wire::core::Transformd c{};
  c.position = {10.0, 8.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  const ObjectId pole_c = state.AddPole(c, 10.0, "C").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok ||
      !state.ApplyPoleType(pole_c, type_ids.front()).ok) {
    return false;
  }

  FixtureConnectionOptions trunk_options{};
  trunk_options.connection_context = wire::core::ConnectionContext::kTrunkContinue;
  const auto trunk =
      add_connection_by_category(state, pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, trunk_options);
  if (!trunk.ok) {
    return false;
  }

  FixtureConnectionOptions branch_options{};
  branch_options.connection_context = wire::core::ConnectionContext::kBranchAdd;
  const auto branch =
      add_connection_by_category(state, pole_b, pole_c, wire::core::ConnectionCategory::kLowVoltage, branch_options);
  if (!branch.ok) {
    return false;
  }
  return branch.value.port_a_id != trunk.value.port_b_id;
}

bool test_band_selection_deterministic_and_debug_integrity() {
  auto run_once = []() -> std::pair<ObjectId, wire::core::PortResolutionDebugRecord> {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return {wire::core::kInvalidObjectId, {}};
    }
    const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
    wire::core::Transformd b{};
    b.position = {12.0, 0.0, 0.0};
    const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
    (void)state.ApplyPoleType(pole_a, type_ids.front());
    (void)state.ApplyPoleType(pole_b, type_ids.front());

  FixtureConnectionOptions options{};
    options.connection_context = wire::core::ConnectionContext::kCornerPass;
    options.branch_index = 3;
    const auto result = add_connection_by_category(state, pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, options);
    if (!result.ok || state.view().port_resolution_debug_records().empty()) {
      return {wire::core::kInvalidObjectId, {}};
    }
    return {result.value.port_a_id, state.view().port_resolution_debug_records().back()};
  };

  const auto first = run_once();
  const auto second = run_once();
  if (first.first == wire::core::kInvalidObjectId || second.first == wire::core::kInvalidObjectId ||
      first.first != second.first) {
    return false;
  }
  if (first.second.candidates.empty()) {
    return false;
  }
  return has_selected_port_in_candidates(first.second);
}

bool test_generate_from_guide_reverse_mode_position_symmetry() {
  auto run_mode = [](wire::core::PathDirectionMode mode) {
    CoreState state;
    struct ModeResult {
      bool ok = false;
      std::vector<wire::core::Vec3d> poles{};
      std::vector<wire::core::Vec3d> guide{};
    };
    ModeResult out{};
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return out;
    }
    wire::core::BackboneSpec req{};
    req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 6.0, 0.0}};
    req.interval_m = 6.0;
    req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
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

  auto point_to_segment_distance = [](const wire::core::Vec3d& p, const wire::core::Vec3d& a,
                                      const wire::core::Vec3d& b) -> double {
    const wire::core::Vec3d ab = b - a;
    const wire::core::Vec3d ap = p - a;
    const double ab2 = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
    if (ab2 <= 1e-12) {
      const wire::core::Vec3d d = p - a;
      return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    }
    const double t = std::clamp((ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / ab2, 0.0, 1.0);
    const wire::core::Vec3d q{a.x + ab.x * t, a.y + ab.y * t, a.z + ab.z * t};
    const wire::core::Vec3d d = p - q;
    return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
  };

  auto is_on_polyline = [&](const wire::core::Vec3d& p, const std::vector<wire::core::Vec3d>& guide) -> bool {
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < guide.size(); ++i) {
      best = std::min(best, point_to_segment_distance(p, guide[i], guide[i + 1]));
    }
    return best <= 1e-6;
  };

  const auto forward = run_mode(wire::core::PathDirectionMode::kForward);
  const auto reverse = run_mode(wire::core::PathDirectionMode::kReverse);
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
  const wire::core::Vec3d start = forward.guide.front();
  const wire::core::Vec3d end = forward.guide.back();
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
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  req.constraints.avoid_points = {{10.0, 0.0, 0.0}};
  req.constraints.avoid_radius_m = 1.0;
  const auto result = state.GenerateFromBackboneSpec(req);
  if (!result.ok || result.value.generated_pole_ids.empty()) {
    return false;
  }
  const wire::core::Vec3d avoid = req.constraints.avoid_points.front();
  const double avoid_r2 = req.constraints.avoid_radius_m * req.constraints.avoid_radius_m;
  for (ObjectId pole_id : result.value.generated_pole_ids) {
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr) {
      return false;
    }
    const auto& p = pole->world_transform.position;
    const wire::core::Vec3d d{p.x - avoid.x, p.y - avoid.y, p.z - avoid.z};
    const double d2 = d.x * d.x + d.y * d.y + d.z * d.z;
    if (d2 <= avoid_r2 + 1e-9) {
      return false;
    }
  }
  return true;
}

bool test_preferred_side_uses_geometry() {
  auto pick_side = [](double peer_y) -> wire::core::SlotSide {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return wire::core::SlotSide::kCenter;
    }
    const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
    wire::core::Transformd b{};
    b.position = {10.0, peer_y, 0.0};
    const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
    (void)state.ApplyPoleType(pole_a, type_ids.front());
    (void)state.ApplyPoleType(pole_b, type_ids.front());
    state.clear_port_resolution_debug_records();

  FixtureConnectionOptions options{};
    options.connection_context = wire::core::ConnectionContext::kBranchAdd;
    options.branch_index = 7; // deliberately fixed; geometry should dominate.
    const auto add = add_connection_by_category(state, pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, options);
    if (!add.ok) {
      return wire::core::SlotSide::kCenter;
    }
    for (const auto& debug : state.view().port_resolution_debug_records()) {
      if (debug.pole_id != pole_a || debug.selected_port_id == wire::core::kInvalidObjectId) {
        continue;
      }
      const auto* selected_port = state.view().edit_state().ports.find(debug.selected_port_id);
      if (selected_port == nullptr) {
        return wire::core::SlotSide::kCenter;
      }
      return selected_port->template_side;
    }
    return wire::core::SlotSide::kCenter;
  };

  const wire::core::SlotSide right = pick_side(+8.0);
  const wire::core::SlotSide left = pick_side(-8.0);
  return right == wire::core::SlotSide::kRight && left == wire::core::SlotSide::kLeft;
}

bool test_world_up_and_lateral_axis_are_consistent() {
  const wire::core::Vec3d forward{1.0, 2.0, 0.0};
  const wire::core::Vec3d lateral = wire::core::ComputeLateralAxis(forward);
  return almost_equal(wire::core::Dot(lateral, wire::core::WorldUp()), 0.0, 1e-9) &&
         almost_equal(wire::core::Dot(lateral, forward), 0.0, 1e-9) &&
         wire::core::LengthSquared(lateral) > 0.99;
}

bool test_build_pole_frame_roundtrips_local_point_under_tilt() {
  wire::core::Transformd tf{};
  tf.position = {3.0, -4.0, 1.5};
  tf.rotation_euler_deg = {7.0, -5.0, 15.0};
  const wire::core::PoleFrame frame = wire::core::BuildPoleFrame(tf, 33.0);
  const wire::core::Vec3d local{0.2, 0.8, 6.0};
  const wire::core::Vec3d world = wire::core::LocalPointToWorld(frame, local);
  const wire::core::Vec3d roundtrip = wire::core::WorldPointToLocal(frame, world);
  return almost_equal(local, roundtrip, 1e-9);
}

bool test_detail_curve_builds_with_endpoint_position_and_tangent_constraints() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.tangent_length_hint_m = 3.0;

  wire::core::CurveConstraint end{};
  end.point = {12.0, 0.0, 5.0};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.tangent_length_hint_m = 3.0;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 17);
  const wire::core::Vec3d tangent0 = curve.EvaluateTangent(0.0);
  const wire::core::Vec3d tangent1 = curve.EvaluateTangent(1.0);
  return almost_equal(curve.EvaluatePosition(0.0), start.point, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), end.point, 1e-9) &&
         wire::core::Dot(tangent0, wire::core::Vec3d{1.0, 0.0, 0.0}) > 0.98 &&
         wire::core::Dot(tangent1, wire::core::Vec3d{1.0, 0.0, 0.0}) > 0.98;
}

bool test_detail_curve_sag_preserves_endpoints_and_supports_length_queries() {
  wire::core::CurveConstraint line_start{};
  line_start.point = {0.0, 0.0, 6.0};
  line_start.tangent_dir = {1.0, 0.0, 0.0};
  wire::core::CurveConstraint line_end{};
  line_end.point = {16.0, 0.0, 6.0};
  line_end.tangent_dir = {1.0, 0.0, 0.0};
  const wire::core::DetailCurve line = wire::core::BuildDetailCurve(line_start, line_end, 33);

  wire::core::CurveConstraint sag_start = line_start;
  wire::core::CurveConstraint sag_end = line_end;
  sag_start.sag_hint = 0.03;
  sag_end.sag_hint = 0.03;
  const wire::core::DetailCurve sag = wire::core::BuildDetailCurve(sag_start, sag_end, 33);
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
  const wire::core::Vec3d line_mid = line.PositionAtLength(line.Length() * 0.5);
  const wire::core::Vec3d sag_mid = sag.PositionAtLength(sag.Length() * 0.5);
  return sag.Length() > 0.0 &&
         wire::core::HeightAlongWorldUp(sag_mid) < wire::core::HeightAlongWorldUp(line_mid) &&
         almost_equal(sag.PositionAtLength(0.0), sag.EvaluatePosition(0.0), 1e-9) &&
         almost_equal(sag.PositionAtLength(sag.Length()), sag.EvaluatePosition(1.0), 1e-9);
}

bool test_detail_curve_sag_uses_catenary_like_support_slope() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 6.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.sag_hint = 0.04;

  wire::core::CurveConstraint end{};
  end.point = {18.0, 0.0, 6.0};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.sag_hint = 0.04;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 33);
  const wire::core::Vec3d tangent0 = curve.EvaluateTangent(0.0);
  const wire::core::Vec3d tangent1 = curve.EvaluateTangent(1.0);
  return curve.sag_amplitude_m > 0.0 && tangent0.z < -0.02 && tangent1.z > 0.02;
}

bool test_detail_curve_near_straight_tangent_hints_do_not_wobble_sideways() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {1.0, 0.18, 0.0};
  start.tangent_length_hint_m = 6.0;

  wire::core::CurveConstraint end{};
  end.point = {20.0, 0.0, 5.0};
  end.tangent_dir = {1.0, -0.18, 0.0};
  end.tangent_length_hint_m = 6.0;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 25);
  double max_abs_y = 0.0;
  for (const wire::core::Vec3d& sample : curve.sample_points) {
    max_abs_y = std::max(max_abs_y, std::abs(sample.y));
  }
  return curve.quality.shape_policy == wire::core::CurveShapePolicyKind::kNearStraight && max_abs_y <= 0.03;
}

bool test_detail_curve_smooth_pass_suppresses_lateral_bend() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {0.90, 0.43, 0.0};
  start.tangent_length_hint_m = 8.0;
  start.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;

  wire::core::CurveConstraint end{};
  end.point = {18.0, 0.0, 5.0};
  end.tangent_dir = {0.90, 0.43, 0.0};
  end.tangent_length_hint_m = 8.0;
  end.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 33);
  double max_abs_y = 0.0;
  for (const wire::core::Vec3d& sample : curve.sample_points) {
    max_abs_y = std::max(max_abs_y, std::abs(sample.y));
  }
  return curve.quality.shape_policy == wire::core::CurveShapePolicyKind::kSmoothPass &&
         curve.quality.adopted_continuity == wire::core::DetailCurveContinuityMode::kG2 &&
         curve.quality.lateral_suppression >= 0.99 && curve.quality.start_lateral_ratio_limit <= 1e-9 &&
         curve.quality.end_lateral_ratio_limit <= 1e-9 && max_abs_y <= 0.03;
}

bool test_detail_curve_sharp_corner_stays_compact() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {0.55, 0.84, 0.0};
  start.tangent_length_hint_m = 8.0;
  start.corner_pass = true;
  start.corner_angle_deg = 60.0;
  start.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;

  wire::core::CurveConstraint end{};
  end.point = {18.0, 0.0, 5.0};
  end.tangent_dir = {0.55, -0.84, 0.0};
  end.tangent_length_hint_m = 8.0;
  end.corner_pass = true;
  end.corner_angle_deg = 60.0;
  end.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 33);
  double max_abs_y = 0.0;
  for (const wire::core::Vec3d& sample : curve.sample_points) {
    max_abs_y = std::max(max_abs_y, std::abs(sample.y));
  }
  return curve.quality.shape_policy == wire::core::CurveShapePolicyKind::kSharpCorner &&
         curve.quality.adopted_continuity == wire::core::DetailCurveContinuityMode::kG1 &&
         curve.quality.continuity_reason == wire::core::DetailCurveContinuityReason::kCornerPass &&
         max_abs_y <= 0.08;
}

bool test_detail_curve_offset_endpoint_uses_offset_endpoints() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 4.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.endpoint_offset = {0.5, 0.0, 0.0};
  start.endpoint_mode = wire::core::CurveEndpointMode::kOffsetEndpoint;

  wire::core::CurveConstraint end{};
  end.point = {10.0, 0.0, 4.0};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.endpoint_offset = {-0.5, 0.0, 0.0};
  end.endpoint_mode = wire::core::CurveEndpointMode::kOffsetEndpoint;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 17);
  return almost_equal(curve.EvaluatePosition(0.0), wire::core::Vec3d{0.5, 0.0, 4.0}, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), wire::core::Vec3d{9.5, 0.0, 4.0}, 1e-9) &&
         !almost_equal(curve.EvaluatePosition(0.0), start.point, 1e-9) &&
         !almost_equal(curve.EvaluatePosition(1.0), end.point, 1e-9);
}

bool test_detail_curve_acute_case_applies_quality_fallback() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {-1.0, 0.0, 0.0};
  start.tangent_length_hint_m = 9.0;
  start.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;

  wire::core::CurveConstraint end{};
  end.point = {8.0, 1.0, 5.0};
  end.tangent_dir = {0.0, -1.0, 0.0};
  end.tangent_length_hint_m = 9.0;
  end.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 21);
  wire::core::Vec3d chord = end.point - start.point;
  const double chord_length = std::sqrt(wire::core::LengthSquared(chord));
  if (chord_length <= 1e-9) {
    return false;
  }
  chord = wire::core::ScaleVec(chord, 1.0 / chord_length);
  double max_deviation = 0.0;
  const wire::core::Vec3d expected_start = start.point;
  const wire::core::Vec3d expected_end = end.point;
  for (const wire::core::Vec3d& sample : curve.sample_points) {
    if (!std::isfinite(sample.x) || !std::isfinite(sample.y) || !std::isfinite(sample.z)) {
      return false;
    }
    const wire::core::Vec3d delta = sample - start.point;
    const double progress = wire::core::Dot(delta, chord);
    const wire::core::Vec3d along = wire::core::ScaleVec(chord, progress);
    max_deviation = std::max(max_deviation, std::sqrt(std::max(0.0, wire::core::LengthSquared(delta - along))));
  }
  return curve.quality.adopted_continuity == wire::core::DetailCurveContinuityMode::kG1 &&
         curve.quality.continuity_reason == wire::core::DetailCurveContinuityReason::kConflictingTangents &&
         curve.quality.degraded_to_g1 &&
         almost_equal(curve.EvaluatePosition(0.0), expected_start, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), expected_end, 1e-9) &&
         max_deviation <= chord_length * 0.80 + 1e-6;
}

bool test_detail_curve_long_pass_through_prefers_g2() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 6.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.tangent_length_hint_m = 6.0;
  start.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;

  wire::core::CurveConstraint end{};
  end.point = {20.0, 0.0, 6.0};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.tangent_length_hint_m = 6.0;
  end.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 33);
  return curve.quality.requested_policy == wire::core::CableContinuityPolicyHint::kPreferG2 &&
         curve.quality.adopted_continuity == wire::core::DetailCurveContinuityMode::kG2 &&
         curve.quality.continuity_reason == wire::core::DetailCurveContinuityReason::kSmoothPassThrough &&
         curve.quality.attempted_g2 && !curve.quality.degraded_to_g1 &&
         curve.quality.handle_length_start_m > 0.0 && curve.quality.handle_length_end_m > 0.0;
}

bool test_detail_curve_short_span_falls_back_to_g1() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.tangent_length_hint_m = 1.5;
  start.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;

  wire::core::CurveConstraint end{};
  end.point = {2.0, 0.0, 5.0};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.tangent_length_hint_m = 1.5;
  end.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 17);
  return curve.quality.adopted_continuity == wire::core::DetailCurveContinuityMode::kG1 &&
         curve.quality.continuity_reason == wire::core::DetailCurveContinuityReason::kShortSpan &&
         curve.quality.degraded_to_g1 &&
         almost_equal(curve.EvaluatePosition(0.0), start.point, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), end.point, 1e-9);
}

bool test_detail_curve_branch_pass_uses_g1_and_preserves_endpoint_constraints() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.tangent_length_hint_m = 4.0;
  start.endpoint_mode = wire::core::CurveEndpointMode::kOffsetEndpoint;
  start.endpoint_offset = {0.4, 0.0, 0.0};
  start.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;
  start.pass_mode = wire::core::CurvePassMode::kBranch;

  wire::core::CurveConstraint end{};
  end.point = {12.0, 2.0, 5.0};
  const wire::core::Vec3d expected_end_tangent{0.6, 0.8, 0.0};
  end.tangent_dir = expected_end_tangent;
  end.tangent_length_hint_m = 4.0;
  end.endpoint_mode = wire::core::CurveEndpointMode::kOffsetEndpoint;
  end.endpoint_offset = {0.0, 0.35, 0.0};
  end.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;
  end.pass_mode = wire::core::CurvePassMode::kBranch;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 25);
  const wire::core::Vec3d tangent0 = curve.EvaluateTangent(0.0);
  const wire::core::Vec3d tangent1 = curve.EvaluateTangent(1.0);
  return curve.quality.shape_policy == wire::core::CurveShapePolicyKind::kBranchPass &&
         curve.quality.adopted_continuity == wire::core::DetailCurveContinuityMode::kG1 &&
         curve.quality.continuity_reason == wire::core::DetailCurveContinuityReason::kBranchPass &&
         curve.quality.degraded_to_g1 &&
         almost_equal(curve.EvaluatePosition(0.0), start.point + start.endpoint_offset, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), end.point + end.endpoint_offset, 1e-9) &&
         wire::core::Dot(tangent0, wire::core::Vec3d{1.0, 0.0, 0.0}) > 0.92 &&
         wire::core::Dot(tangent1, expected_end_tangent) > 0.74;
}

bool test_detail_curve_branch_long_span_suppresses_sideways_runout() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.5};
  start.tangent_dir = {0.20, 0.98, 0.0};
  start.tangent_length_hint_m = 8.0;
  start.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;
  start.pass_mode = wire::core::CurvePassMode::kBranch;

  wire::core::CurveConstraint end{};
  end.point = {24.0, 0.0, 5.5};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.tangent_length_hint_m = 8.0;
  end.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;
  end.pass_mode = wire::core::CurvePassMode::kBranch;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 41);
  double max_abs_y = 0.0;
  for (std::size_t i = 0; i < curve.sample_points.size(); ++i) {
    const double abs_y = std::abs(curve.sample_points[i].y);
    max_abs_y = std::max(max_abs_y, abs_y);
  }
  return curve.quality.shape_policy == wire::core::CurveShapePolicyKind::kBranchPass &&
         curve.quality.start_tangent_rule == wire::core::DetailCurveEndpointTangentRule::kBranchChordPriority &&
         curve.quality.handle_length_start_m <= curve.quality.start_departure_length_m + 1e-6 &&
         curve.quality.start_departure_length_m <= 1.10 + 1e-6 &&
         curve.quality.start_support_weight < curve.quality.start_chord_weight && max_abs_y <= 0.40;
}

bool test_detail_curve_branch_short_span_keeps_more_local_departure_than_long_span() {
  auto build_branch_curve = [](double length_m) {
    wire::core::CurveConstraint start{};
    start.point = {0.0, 0.0, 5.5};
    start.tangent_dir = {0.20, 0.98, 0.0};
    start.tangent_length_hint_m = 6.0;
    start.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;
    start.pass_mode = wire::core::CurvePassMode::kBranch;

    wire::core::CurveConstraint end{};
    end.point = {length_m, 0.0, 5.5};
    end.tangent_dir = {1.0, 0.0, 0.0};
    end.tangent_length_hint_m = 6.0;
    end.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;
    end.pass_mode = wire::core::CurvePassMode::kBranch;
    return wire::core::BuildDetailCurve(start, end, 33);
  };

  const wire::core::DetailCurve short_curve = build_branch_curve(6.0);
  const wire::core::DetailCurve long_curve = build_branch_curve(24.0);
  return short_curve.quality.start_support_weight > long_curve.quality.start_support_weight &&
         short_curve.quality.start_lateral_ratio_limit > long_curve.quality.start_lateral_ratio_limit &&
         short_curve.quality.start_tangent_rule == wire::core::DetailCurveEndpointTangentRule::kBranchChordPriority &&
         long_curve.quality.start_tangent_rule == wire::core::DetailCurveEndpointTangentRule::kBranchChordPriority;
}

bool test_detail_curve_main_sag_reads_stronger_than_branch() {
  auto midpoint_drop = [](wire::core::CurvePassMode pass_mode) {
    wire::core::CurveConstraint start{};
    start.point = {0.0, 0.0, 6.0};
    start.tangent_dir = {1.0, 0.0, 0.0};
    start.tangent_length_hint_m = 6.0;
    start.sag_hint = 0.03;
    start.pass_mode = pass_mode;

    wire::core::CurveConstraint end{};
    end.point = {20.0, 0.0, 6.0};
    end.tangent_dir = {1.0, 0.0, 0.0};
    end.tangent_length_hint_m = 6.0;
    end.sag_hint = 0.03;
    end.pass_mode = pass_mode;

    const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 33);
    const double midpoint_z = wire::core::HeightAlongWorldUp(curve.EvaluatePosition(0.5));
    return std::pair<double, wire::core::DetailCurve>{6.0 - midpoint_z, curve};
  };

  const auto main_result = midpoint_drop(wire::core::CurvePassMode::kPassThrough);
  const auto branch_result = midpoint_drop(wire::core::CurvePassMode::kBranch);
  return main_result.second.quality.sag_pass_scale > branch_result.second.quality.sag_pass_scale &&
         main_result.first > branch_result.first + 0.08 && branch_result.first > 0.0;
}

bool test_detail_curve_large_height_difference_uses_composite_segments() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 11.0};
  start.tangent_dir = {0.96, 0.0, -0.28};
  start.tangent_length_hint_m = 5.0;
  start.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;
  start.profile_hint = wire::core::CurveProfileHint::kCompositeHeightTransition;

  wire::core::CurveConstraint end{};
  end.point = {20.0, 0.0, 4.0};
  end.tangent_dir = {0.98, 0.0, -0.16};
  end.tangent_length_hint_m = 5.0;
  end.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;
  end.profile_hint = wire::core::CurveProfileHint::kCompositeHeightTransition;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 41);
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
  const double early_z = wire::core::HeightAlongWorldUp(curve.EvaluatePosition(0.18));
  const double late_z = wire::core::HeightAlongWorldUp(curve.EvaluatePosition(0.82));
  return curve.quality.shape_policy == wire::core::CurveShapePolicyKind::kSmoothPass &&
         curve.quality.adopted_continuity == wire::core::DetailCurveContinuityMode::kG2 &&
         early_z > late_z && curve.Length() > 0.0;
}

bool test_detail_curve_prefer_g1_policy_is_explicit_not_degraded() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 5.0};
  start.tangent_dir = {1.0, 0.0, 0.0};
  start.tangent_length_hint_m = 5.0;
  start.sag_hint = 0.03;
  start.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG1;

  wire::core::CurveConstraint end{};
  end.point = {16.0, 0.0, 5.0};
  end.tangent_dir = {1.0, 0.0, 0.0};
  end.tangent_length_hint_m = 5.0;
  end.sag_hint = 0.03;
  end.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG1;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 33);
  return curve.quality.requested_policy == wire::core::CableContinuityPolicyHint::kPreferG1 &&
         curve.quality.adopted_continuity == wire::core::DetailCurveContinuityMode::kG1 &&
         curve.quality.continuity_reason == wire::core::DetailCurveContinuityReason::kPolicyPreferG1 &&
         !curve.quality.attempted_g2 && !curve.quality.degraded_to_g1 && curve.sag_amplitude_m > 0.0 &&
         almost_equal(curve.EvaluatePosition(0.0), start.point, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), end.point, 1e-9) &&
         curve.Length() > 0.0 && !curve.arc_length_table.empty();
}

bool test_detail_curve_via_attachment_policy_uses_offset_endpoint_and_g1() {
  wire::core::CurveConstraint start{};
  start.point = {0.0, 0.0, 4.5};
  start.tangent_dir = {0.96, 0.24, 0.0};
  start.tangent_length_hint_m = 4.0;
  start.endpoint_mode = wire::core::CurveEndpointMode::kOffsetEndpoint;
  start.endpoint_offset = {0.45, 0.20, 0.0};
  start.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;

  wire::core::CurveConstraint end{};
  end.point = {12.0, 0.0, 4.5};
  end.tangent_dir = {0.96, -0.20, 0.0};
  end.tangent_length_hint_m = 4.0;
  end.endpoint_mode = wire::core::CurveEndpointMode::kOffsetEndpoint;
  end.endpoint_offset = {-0.45, 0.15, 0.0};
  end.continuity_preference = wire::core::CableContinuityPolicyHint::kPreferG2;

  const wire::core::DetailCurve curve = wire::core::BuildDetailCurve(start, end, 25);
  return curve.quality.shape_policy == wire::core::CurveShapePolicyKind::kViaAttachment &&
         curve.quality.adopted_continuity == wire::core::DetailCurveContinuityMode::kG1 &&
         curve.quality.continuity_reason == wire::core::DetailCurveContinuityReason::kEndpointConstraintPriority &&
         almost_equal(curve.EvaluatePosition(0.0), start.point + start.endpoint_offset, 1e-9) &&
         almost_equal(curve.EvaluatePosition(1.0), end.point + end.endpoint_offset, 1e-9);
}

bool test_cable_curve_parabolic_sag_has_exact_semantics() {
  namespace cable_curve = wire::core::geometry::curve;
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
  const wire::core::Vec3d linear_midpoint = wire::core::ScaleVec(input.start + input.end, 0.5);
  const wire::core::DetailCurve detail = cable_curve::ToDetailCurve(input, built.value);
  const bool finite_bounds = std::isfinite(built.value.bounds.min.x) && std::isfinite(built.value.bounds.min.y) &&
                             std::isfinite(built.value.bounds.min.z) && std::isfinite(built.value.bounds.max.x) &&
                             std::isfinite(built.value.bounds.max.y) && std::isfinite(built.value.bounds.max.z) &&
                             built.value.bounds.min.x <= built.value.bounds.max.x &&
                             built.value.bounds.min.y <= built.value.bounds.max.y &&
                             built.value.bounds.min.z <= built.value.bounds.max.z;
  return almost_equal(built.value.samples.front().position, input.start) &&
         almost_equal(built.value.samples.back().position, input.end) &&
         almost_equal(midpoint.position, linear_midpoint + wire::core::Vec3d{0.0, 0.0, -input.sag_m}, 1e-9) &&
         almost_equal(midpoint.position.y, 2.0, 1e-9) &&
         almost_equal(detail.EvaluatePosition(0.5), midpoint.position, 1e-9) &&
         almost_equal(detail.sag_amplitude_m, input.sag_m, 1e-9) && finite_bounds &&
         detail.arc_length_table.size() == built.value.samples.size() &&
         detail.distance_attributes.arc_length_m.size() == built.value.samples.size();
}

bool test_cable_curve_samples_have_stable_orthonormal_frames() {
  namespace cable_curve = wire::core::geometry::curve;
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
    const auto finite = [](const wire::core::Vec3d& value) {
      return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    };
    if (!finite(sample.tangent) || !finite(sample.normal) || !finite(sample.binormal) ||
        !almost_equal(wire::core::Length(sample.tangent), 1.0, 1e-9) ||
        !almost_equal(wire::core::Length(sample.normal), 1.0, 1e-9) ||
        !almost_equal(wire::core::Length(sample.binormal), 1.0, 1e-9) ||
        std::abs(wire::core::Dot(sample.tangent, sample.normal)) > 1e-9 ||
        std::abs(wire::core::Dot(sample.tangent, sample.binormal)) > 1e-9 ||
        std::abs(wire::core::Dot(sample.normal, sample.binormal)) > 1e-9) {
      return false;
    }
  }
  return true;
}

bool test_cable_curve_reverse_keeps_canonical_lateral_frame() {
  namespace cable_curve = wire::core::geometry::curve;
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
  namespace cable_curve = wire::core::geometry::curve;
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
  namespace cable_curve = wire::core::geometry::curve;
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

bool test_cable_curve_hermite_sag_preserves_endpoint_tangents() {
  namespace cable_curve = wire::core::geometry::curve;
  cable_curve::CableCurveInput input{};
  input.start = {0.0, 0.0, 8.0};
  input.end = {18.0, 2.0, 9.0};
  input.start_tangent_hint = {1.0, 0.35, 0.1};
  input.end_tangent_hint = {1.0, -0.20, 0.05};
  input.canonical_dir = {1.0, 0.0, 0.0};
  input.sag_m = 1.1;
  input.method = cable_curve::CurveMethod::kCubicHermiteSag;
  const auto built = cable_curve::BuildCableCurve(input);
  if (!built.ok || built.value.samples.size() < 3) {
    return false;
  }
  wire::core::Vec3d expected_start = input.start_tangent_hint;
  wire::core::Vec3d expected_end = input.end_tangent_hint;
  wire::core::Normalize(&expected_start);
  wire::core::Normalize(&expected_end);
  const wire::core::DetailCurve detail = cable_curve::ToDetailCurve(input, built.value);
  return almost_equal(built.value.samples.front().position, input.start) &&
         almost_equal(built.value.samples.back().position, input.end) &&
         almost_equal(built.value.samples.front().tangent, expected_start, 1e-9) &&
         almost_equal(built.value.samples.back().tangent, expected_end, 1e-9) &&
         almost_equal(detail.EvaluateTangent(0.0), expected_start, 1e-9) &&
         almost_equal(detail.EvaluateTangent(1.0), expected_end, 1e-9);
}

bool test_backbone_continuous_run_is_g1_at_internal_node() {
  wire::core::CoreState state;
  wire::core::GeometrySettings geometry = state.view().geometry_settings();
  geometry.sag_enabled = true;
  geometry.sag_factor = 0.04;
  if (!state.UpdateGeometrySettings(geometry).ok) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {18.0, 8.0, 0.0}};
  const std::vector<wire::core::PoleTypeId> pole_types = sorted_pole_type_ids(state);
  if (pole_types.empty()) {
    return false;
  }
  req.pole_type_id = pole_types.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_span_ids.size() != 2) {
    return false;
  }
  const auto* first = state.find_curve_cache(generated.value.generated_span_ids[0]);
  const auto* second = state.find_curve_cache(generated.value.generated_span_ids[1]);
  const auto first_layout = state.span_layout(generated.value.generated_span_ids[0]);
  const auto second_layout = state.span_layout(generated.value.generated_span_ids[1]);
  if (first == nullptr || second == nullptr || !first_layout.has_layout() || !second_layout.has_layout()) {
    return false;
  }
  const wire::core::Vec3d incoming = first->detail.EvaluateTangent(1.0);
  const wire::core::Vec3d outgoing = second->detail.EvaluateTangent(0.0);
  return wire::core::Dot(incoming, outgoing) > 1.0 - 1e-9 &&
         almost_equal(first_layout.entry->end.departure_dir,
                      second_layout.entry->start.departure_dir, 1e-9);
}

bool test_backbone_fixture_boundary_does_not_join_context_run_tangent() {
  wire::core::CoreState state;
  wire::core::BackboneSpec base{};
  base.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {12.0, 8.0, 0.0}};
  const std::vector<wire::core::PoleTypeId> pole_types = sorted_pole_type_ids(state);
  if (pole_types.empty()) {
    return false;
  }
  base.pole_type_id = pole_types.front();
  add_backbone_bundle(base, wire::core::BundleKind::kHighVoltage);
  const auto first = state.GenerateFromBackboneSpec(base);
  if (!first.ok || first.value.generated_pole_ids.size() != 3) {
    return false;
  }
  const wire::core::ObjectId junction = first.value.generated_pole_ids[1];
  const wire::core::Pole* pole = state.view().poles().find(junction);
  if (pole == nullptr) {
    return false;
  }
  wire::core::BackboneSpec branch{};
  branch.path.polyline = {pole->world_transform.position, {20.0, 0.0, 0.0}};
  branch.pole_type_id = base.pole_type_id;
  wire::core::BackboneInputSpec::NodeSpec existing{};
  existing.point_index = 0;
  existing.support_kind = wire::core::SupportKind::kPole;
  existing.node_id = junction;
  branch.path.node_specs = {existing};
  add_backbone_bundle(branch, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(branch);
  if (!generated.ok || generated.value.generated_span_ids.empty()) {
    return false;
  }
  for (wire::core::ObjectId span_id : generated.value.generated_span_ids) {
    const auto layout = state.span_layout(span_id);
    if (!layout.has_layout() || !layout.entry->start.lower_required) {
      return false;
    }
    wire::core::Vec3d branch_chord =
        layout.entry->end.support_world - layout.entry->start.support_world;
    if (!wire::core::Normalize(&branch_chord) ||
        !almost_equal(layout.entry->start.departure_dir, branch_chord, 1e-9)) {
      return false;
    }
  }
  return true;
}

double sample_curvature(const wire::core::geometry::curve::CableCurveSample& a,
                        const wire::core::geometry::curve::CableCurveSample& b) {
  const double ds = b.arc_length_m - a.arc_length_m;
  if (ds <= 1e-9) {
    return 0.0;
  }
  return std::acos(std::clamp(wire::core::Dot(a.tangent, b.tangent), -1.0, 1.0)) / ds;
}

bool test_cable_curve_piecewise_curvature_is_localized() {
  namespace cable_curve = wire::core::geometry::curve;
  cable_curve::CableCurveInput input{};
  input.start = {0.0, 0.0, 9.0};
  input.end = {24.0, 0.0, 9.0};
  input.start_tangent_hint = {0.65, 0.76, 0.0};
  input.end_tangent_hint = {1.0, 0.0, 0.0};
  input.canonical_dir = {1.0, 0.0, 0.0};
  input.sag_m = 0.8;
  input.method = cable_curve::CurveMethod::kCubicHermiteSag;
  input.profile.attachment_blend_length_m = 1.4;
  input.profile.attachment_min_bend_radius_m = 0.30;
  input.profile.plan_view_blend_length_m = 1.0;
  input.profile.plan_view_max_deviation_m = 0.25;
  const auto built = cable_curve::BuildCableCurve(input);
  if (!built.ok || built.value.attachment_regions.empty()) {
    return false;
  }
  double max_attachment_curvature = 0.0;
  double max_main_curvature = 0.0;
  double max_plan_deviation = 0.0;
  for (std::size_t index = 1; index < built.value.samples.size(); ++index) {
    const auto& a = built.value.samples[index - 1];
    const auto& b = built.value.samples[index];
    const double curvature = sample_curvature(a, b);
    if (a.region == cable_curve::CurveRegionKind::kMainSpan &&
        b.region == cable_curve::CurveRegionKind::kMainSpan) {
      max_main_curvature = std::max(max_main_curvature, curvature);
    } else {
      max_attachment_curvature = std::max(max_attachment_curvature, curvature);
    }
    max_plan_deviation = std::max(max_plan_deviation, std::abs(b.position.y));
    if (b.region == cable_curve::CurveRegionKind::kMainSpan && std::abs(b.position.y) > 1e-9) {
      return false;
    }
  }
  const cable_curve::AttachmentRegion& start_region = built.value.attachment_regions.front();
  return max_attachment_curvature > max_main_curvature * 2.0 &&
         start_region.end_arc_length_m <= input.profile.attachment_blend_length_m * 2.0 &&
         max_plan_deviation <= input.profile.plan_view_max_deviation_m + 1e-9;
}

bool test_cable_curve_short_span_stays_finite_and_monotonic() {
  namespace cable_curve = wire::core::geometry::curve;
  cable_curve::CableCurveInput input{};
  input.start = {0.0, 0.0, 3.0};
  input.end = {0.45, 0.0, 3.0};
  input.start_tangent_hint = {0.2, 1.0, 0.0};
  input.end_tangent_hint = {0.2, -1.0, 0.0};
  input.canonical_dir = {1.0, 0.0, 0.0};
  input.sag_m = 0.08;
  input.method = cable_curve::CurveMethod::kCubicHermiteSag;
  input.profile.attachment_blend_length_m = 1.0;
  input.profile.plan_view_blend_length_m = 1.0;
  const auto built = cable_curve::BuildCableCurve(input);
  if (!built.ok || built.value.samples.size() < 2) {
    return false;
  }
  double previous_projection = -1.0;
  double previous_arc = -1.0;
  for (const auto& sample : built.value.samples) {
    const bool finite = std::isfinite(sample.position.x) && std::isfinite(sample.position.y) &&
                        std::isfinite(sample.position.z) && std::isfinite(sample.arc_length_m);
    const double projection = sample.position.x - input.start.x;
    if (!finite || projection + 1e-9 < previous_projection ||
        sample.arc_length_m + 1e-9 < previous_arc) {
      return false;
    }
    previous_projection = projection;
    previous_arc = sample.arc_length_m;
  }
  return almost_equal(built.value.samples.front().position, input.start) &&
         almost_equal(built.value.samples.back().position, input.end);
}

bool test_cable_members_share_run_parameterization() {
  namespace cable_curve = wire::core::geometry::curve;
  cable_curve::CableCurveInput input{};
  input.start = {0.0, 0.0, 7.0};
  input.end = {15.0, 0.0, 7.0};
  input.start_tangent_hint = {1.0, 0.25, 0.0};
  input.end_tangent_hint = {1.0, 0.0, 0.0};
  input.canonical_dir = {1.0, 0.0, 0.0};
  input.sag_m = 0.5;
  input.method = cable_curve::CurveMethod::kCubicHermiteSag;
  cable_curve::CableCurveInput second = input;
  second.start = input.end;
  second.end = {28.0, 0.0, 7.0};
  second.start_tangent_hint = input.end_tangent_hint;
  const auto run = cable_curve::BuildCableRun({input, second});
  cable_curve::CableCurveInput disconnected = second;
  disconnected.start.y += 1.0;
  const auto rejected = cable_curve::BuildCableRun({input, disconnected});
  if (!run.ok || run.value.sections.size() != 2 ||
      run.value.sections[0].last_sample != run.value.sections[1].first_sample || rejected.ok) {
    return false;
  }
  std::vector<wire::core::Vec3d> base_positions{};
  for (const auto& sample : run.value.samples) {
    base_positions.push_back(sample.position);
  }
  cable_curve::CableMemberProfile messenger{};
  messenger.frame_offset_normal_m = 0.12;
  cable_curve::CableMemberProfile conductor{};
  conductor.frame_offset_binormal_m = -0.08;
  const auto a = cable_curve::ExpandCableMember(run.value, messenger);
  const auto b = cable_curve::ExpandCableMember(run.value, conductor);
  if (!a.ok || !b.ok || a.value.positions.size() != run.value.samples.size() ||
      b.value.positions.size() != run.value.samples.size() ||
      a.value.arc_length_m != b.value.arc_length_m) {
    return false;
  }
  for (std::size_t index = 0; index < run.value.samples.size(); ++index) {
    if (!almost_equal(run.value.samples[index].position, base_positions[index]) ||
        !almost_equal(a.value.arc_length_m[index], run.value.samples[index].arc_length_m) ||
        almost_equal(a.value.positions[index], b.value.positions[index])) {
      return false;
    }
  }
  return true;
}

bool test_hierarchical_variation_worldspace_is_continuous() {
  wire::core::VariationSettings settings{};
  settings.enabled = true;
  settings.global_seed = 77;
  settings.world_cell_size_m = 30.0;
  settings.world_bias_scale = 1.0;
  settings.flow_bias_scale = 0.0;
  settings.pole_delta_scale = 0.0;
  settings.local_jitter_scale = 0.0;

  wire::core::VariationContext near_a{};
  near_a.world_position = {10.0, 10.0, 0.0};
  wire::core::VariationContext near_b = near_a;
  near_b.world_position = {11.5, 10.5, 0.0};
  wire::core::VariationContext far = near_a;
  far.world_position = {75.0, -40.0, 0.0};

  const auto sample_a = wire::core::EvaluateHierarchicalVariation(settings, near_a);
  const auto sample_b = wire::core::EvaluateHierarchicalVariation(settings, near_b);
  const auto sample_far = wire::core::EvaluateHierarchicalVariation(settings, far);
  const double near_diff = std::abs(sample_a.world_bias - sample_b.world_bias);
  const double far_diff = std::abs(sample_a.world_bias - sample_far.world_bias);
  return near_diff < 0.10 && far_diff > near_diff;
}

void register_geometry_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C19_backbone_interval_generates_pole_line_basic",
                         "backbone interval generation creates a pole line with pole type", "Invariant", false,
                         test_backbone_interval_generates_pole_line_basic);
  test_registry::AddTest(tests, "C61_Phase48h_AcuteCorner_AutoWidenSpacing", "Acute corners auto-widen lane spacing without category-specific branching", "Invariant", false, test_acute_corner_auto_widens_lane_spacing);
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
  test_registry::AddTest(tests, "C634_CableCurve_HermiteSagPreservesEndpointTangents",
                         "Hermite sag keeps configured endpoint tangent directions", "Invariant", false,
                         test_cable_curve_hermite_sag_preserves_endpoint_tangents);
  test_registry::AddTest(tests, "C635_Backbone_ContinuousRunIsG1",
                         "Continuous bundle lane route spans share the internal-node tangent", "Invariant", false,
                         test_backbone_continuous_run_is_g1_at_internal_node);
  test_registry::AddTest(tests, "C636_Backbone_FixtureBoundaryStaysG0",
                         "Lowered fixture boundaries do not borrow context-run tangents", "Invariant", false,
                         test_backbone_fixture_boundary_does_not_join_context_run_tangent);
  test_registry::AddTest(tests, "C637_CableCurve_PiecewiseCurvatureLocalized",
                         "Attachment curvature is local and stronger than main-span curvature", "Invariant", false,
                         test_cable_curve_piecewise_curvature_is_localized);
  test_registry::AddTest(tests, "C638_CableCurve_ShortSpanFiniteMonotonic",
                         "Short piecewise spans stay finite and do not loop", "Invariant", false,
                         test_cable_curve_short_span_stays_finite_and_monotonic);
  test_registry::AddTest(tests, "C639_CableMember_SharedRunParameterization",
                         "Cable members share run samples and arc-length basis without mutating the run", "Invariant",
                         false, test_cable_members_share_run_parameterization);
}

WIRE_REGISTER_TEST_SUITE(register_geometry_tests);

} // namespace
