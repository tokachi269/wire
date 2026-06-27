#include "registry.hpp"
#include "helpers.hpp"
#include "wire/core/core_test_hook.hpp"
#include "wire/core/coord_utils.hpp"

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

bool test_generate_poles_along_road_basic() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 10;
  road.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  const auto result = state.GeneratePolesAlongRoad(road, 5.0, type_ids.front());
  if (!result.ok) {
    return false;
  }
  if (result.value.size() != 5) {
    return false;
  }

  for (std::size_t i = 0; i < result.value.size(); ++i) {
    const auto* pole = state.view().edit_state().poles.find(result.value[i]);
    if (pole == nullptr) {
      return false;
    }
    if (pole->pole_type_id != type_ids.front()) {
      return false;
    }
    if (!pole->generation.generated || pole->generation.source != wire::core::GenerationSource::kRoadAuto) {
      return false;
    }
    if (!almost_equal(pole->world_transform.position.y, 0.0) || !almost_equal(pole->world_transform.position.z, 0.0)) {
      return false;
    }
  }
  return true;
}

bool test_pole_context_classification_basic() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment straight{};
  straight.id = 901;
  straight.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  const auto straight_result = state.GeneratePolesAlongRoad(straight, 5.0, type_ids.front());
  if (!straight_result.ok || straight_result.value.size() < 3) {
    return false;
  }

  const auto* first = state.view().edit_state().poles.find(straight_result.value.front());
  const auto* middle = state.view().edit_state().poles.find(straight_result.value[1]);
  const auto* last = state.view().edit_state().poles.find(straight_result.value.back());
  if (first == nullptr || middle == nullptr || last == nullptr) {
    return false;
  }
  if (first->context.kind != wire::core::PoleContextKind::kTerminal ||
      last->context.kind != wire::core::PoleContextKind::kTerminal ||
      middle->context.kind != wire::core::PoleContextKind::kStraight) {
    return false;
  }

  CoreState corner_state;
  const auto corner_type_ids = sorted_pole_type_ids(corner_state);
  if (corner_type_ids.empty()) {
    return false;
  }
  wire::core::RoadSegment corner{};
  corner.id = 902;
  corner.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 10.0, 0.0}};
  const auto corner_result = corner_state.GeneratePolesAlongRoad(corner, 5.0, corner_type_ids.front());
  if (!corner_result.ok) {
    return false;
  }

  bool found_corner = false;
  for (ObjectId pole_id : corner_result.value) {
    const auto* pole = corner_state.view().edit_state().poles.find(pole_id);
    if (pole != nullptr && pole->context.kind == wire::core::PoleContextKind::kCorner) {
      if (pole->context.corner_angle_deg <= 0.0 || pole->context.side_scale < 1.0) {
        return false;
      }
      found_corner = true;
      break;
    }
  }
  return found_corner;
}

bool test_angle_correction_bounds_and_finite() {
  CoreState state;
  wire::core::LayoutSettings layout{};
  layout.angle_correction_enabled = true;
  layout.corner_threshold_deg = 5.0;
  layout.min_side_scale = 1.0;
  layout.max_side_scale = 1.6;
  if (!state.UpdateLayoutSettings(layout).ok) {
    return false;
  }

  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::RoadSegment road{};
  road.id = 903;
  road.polyline = {{0.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, {8.0, 8.0, 0.0}};
  const auto result = state.GeneratePolesAlongRoad(road, 4.0, type_ids.front());
  if (!result.ok) {
    return false;
  }

  bool found_corrected_port = false;
  for (ObjectId pole_id : result.value) {
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr || pole->context.kind != wire::core::PoleContextKind::kCorner) {
      continue;
    }
    if (pole->context.side_scale < layout.min_side_scale || pole->context.side_scale > layout.max_side_scale) {
      return false;
    }
    for (const auto& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id != pole_id) {
        continue;
      }
      if (!std::isfinite(port.world_position.x) || !std::isfinite(port.world_position.y) ||
          !std::isfinite(port.world_position.z) || !std::isfinite(port.side_scale_applied)) {
        return false;
      }
      if (port.angle_correction_applied) {
        found_corrected_port = true;
      }
    }
  }
  return found_corrected_port;
}

bool test_corner_turn_sign_biases_outer_side() {
  auto check_turn = [](const std::vector<wire::core::Vec3d>& polyline, bool expect_left_turn_outer_right) -> bool {
    CoreState state;
    wire::core::LayoutSettings layout{};
    layout.angle_correction_enabled = true;
    layout.corner_threshold_deg = 5.0;
    layout.min_side_scale = 1.0;
    layout.max_side_scale = 1.8;
    if (!state.UpdateLayoutSettings(layout).ok) {
      return false;
    }

    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return false;
    }

    wire::core::RoadSegment road{};
    road.id = expect_left_turn_outer_right ? 905 : 906;
    road.polyline = polyline;
    const auto result = state.GeneratePolesAlongRoad(road, 4.0, type_ids.front());
    if (!result.ok) {
      return false;
    }

    const wire::core::Pole* corner_pole = nullptr;
    for (ObjectId pole_id : result.value) {
      const auto* pole = state.view().edit_state().poles.find(pole_id);
      if (pole != nullptr && pole->context.kind == wire::core::PoleContextKind::kCorner) {
        corner_pole = pole;
        break;
      }
    }
    if (corner_pole == nullptr) {
      return false;
    }

    const wire::core::Port* left_band_port = nullptr;
    const wire::core::Port* right_band_port = nullptr;
    for (const auto& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id != corner_pole->id) {
        continue;
      }
      if (port.template_side == wire::core::SlotSide::kLeft) {
        left_band_port = &port;
      } else if (port.template_side == wire::core::SlotSide::kRight) {
        right_band_port = &port;
      }
    }
    if (left_band_port == nullptr || right_band_port == nullptr) {
      return false;
    }

    const double left_offset = std::abs(left_band_port->world_position.y - corner_pole->world_transform.position.y);
    const double right_offset = std::abs(right_band_port->world_position.y - corner_pole->world_transform.position.y);
    if (expect_left_turn_outer_right) {
      return corner_pole->context.corner_turn_sign > 0.0 && right_offset > left_offset &&
             right_band_port->side_scale_applied > left_band_port->side_scale_applied;
    }
    return corner_pole->context.corner_turn_sign < 0.0 && left_offset > right_offset &&
           left_band_port->side_scale_applied > right_band_port->side_scale_applied;
  };

  const bool left_ok = check_turn({{0.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, {8.0, 8.0, 0.0}}, true);
  const bool right_ok = check_turn({{0.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, {8.0, -8.0, 0.0}}, false);
  return left_ok && right_ok;
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
    wire::core::RoadSegment road{};
    road.id = static_cast<std::uint64_t>(990 + static_cast<int>(interior_deg));
    road.polyline = {
        {0.0, 0.0, 0.0},
        {10.0, 0.0, 0.0},
        {10.0 + 10.0 * std::cos(out_heading), 10.0 * std::sin(out_heading), 0.0},
    };
    const auto gen =
        state.GenerateSimpleLineFromPoints(road, type_ids.front(), wire::core::ConnectionCategory::kLowVoltage);
    if (!gen.ok || gen.value.pole_ids.size() != 3) {
      return std::numeric_limits<double>::quiet_NaN();
    }

    const auto* pole = state.view().edit_state().poles.find(gen.value.pole_ids[1]);
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

  wire::core::AddConnectionByPoleOptions trunk_options{};
  trunk_options.connection_context = wire::core::ConnectionContext::kTrunkContinue;
  const auto trunk =
      add_connection_by_category(state, pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, trunk_options);
  if (!trunk.ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions branch_options{};
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

  wire::core::AddConnectionByPoleOptions options{};
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

bool test_generate_simple_line_corner_context_integration() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 904;
  road.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {12.0, 12.0, 0.0}};
  const auto result =
      state.GenerateSimpleLine(road, 4.0, type_ids.front(), wire::core::ConnectionCategory::kLowVoltage);
  if (!result.ok || result.value.span_ids.empty() || result.value.pole_ids.empty()) {
    return false;
  }

  const ObjectId corner_pole_id = find_pole_id_by_position(state, road.polyline[1]);
  if (corner_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto* corner_pole = state.view().edit_state().poles.find(corner_pole_id);
  if (corner_pole == nullptr) {
    return false;
  }
  return corner_pole->context.kind == wire::core::PoleContextKind::kCorner && validate_now(state).ok();
}

bool test_generate_simple_line_from_points_exact_poles_and_orientation() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 907;
  road.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {10.0, 10.0, 0.0}};
  const auto result =
      state.GenerateSimpleLineFromPoints(road, type_ids.front(), wire::core::ConnectionCategory::kLowVoltage);
  if (!result.ok) {
    return false;
  }
  if (result.value.pole_ids.size() != road.polyline.size()) {
    return false;
  }
  if (result.value.span_ids.size() != road.polyline.size() - 1) {
    return false;
  }

  for (std::size_t i = 0; i < road.polyline.size(); ++i) {
    const auto* pole = state.view().edit_state().poles.find(result.value.pole_ids[i]);
    if (pole == nullptr) {
      return false;
    }
    if (!almost_equal(pole->world_transform.position, road.polyline[i])) {
      return false;
    }
  }

  const auto* first = state.view().edit_state().poles.find(result.value.pole_ids[0]);
  const auto* middle = state.view().edit_state().poles.find(result.value.pole_ids[1]);
  const auto* last = state.view().edit_state().poles.find(result.value.pole_ids[2]);
  if (first == nullptr || middle == nullptr || last == nullptr) {
    return false;
  }
  if (!almost_equal(first->world_transform.rotation_euler_deg.z, 0.0, 1e-6)) {
    return false;
  }
  if (!almost_equal(last->world_transform.rotation_euler_deg.z, 90.0, 1e-6)) {
    return false;
  }
  return almost_equal(middle->world_transform.rotation_euler_deg.z, 45.0, 1e-6);
}

bool test_generate_simple_line_from_points_sharp_corner_perpendicular_orientation() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 931;
  road.polyline = {
      {0.0, 0.0, 0.0},
      {10.0, 0.0, 0.0},
      {5.0, 8.660254037844386, 0.0}, // interior ~= 60 deg at middle
  };
  const auto result = state.GenerateSimpleLineFromPoints(road, pole_type_ids.front(), ConnectionCategory::kLowVoltage);
  if (!result.ok || result.value.pole_ids.size() != 3) {
    return false;
  }

  const auto* middle = state.view().edit_state().poles.find(result.value.pole_ids[1]);
  if (middle == nullptr) {
    return false;
  }
  const wire::core::Vec3d u0 = normalize_xy_safe(road.polyline[0] - road.polyline[1]); // corner->prev
  const wire::core::Vec3d u1 = normalize_xy_safe(road.polyline[2] - road.polyline[1]); // corner->next
  const wire::core::Vec3d bisector = normalize_xy_safe({u0.x + u1.x, u0.y + u1.y, 0.0});
  if (std::abs(bisector.x) < 1e-9 && std::abs(bisector.y) < 1e-9) {
    return false;
  }

  const wire::core::Vec3d side_from_yaw = local_side_axis_from_yaw(middle->world_transform.rotation_euler_deg.z);
  const wire::core::Vec3d side_debug = normalize_xy_safe(middle->context.sharp_side_dir);
  const wire::core::Vec3d bisector_debug = normalize_xy_safe(middle->context.sharp_bisector_dir);

  const wire::core::Vec3d t_in = normalize_xy_safe(road.polyline[1] - road.polyline[0]);
  const wire::core::Vec3d t_out = normalize_xy_safe(road.polyline[2] - road.polyline[1]);
  const double turn = t_in.x * t_out.y - t_in.y * t_out.x;
  wire::core::Vec3d inward{};
  if (turn > 1e-9) {
    inward = normalize_xy_safe({-t_in.y, t_in.x, 0.0});
  } else if (turn < -1e-9) {
    inward = normalize_xy_safe({t_in.y, -t_in.x, 0.0});
  }

  const bool ok_sharp = middle->context.sharp_orientation_applied;
  const bool ok_perp = std::abs(dot_xy(side_from_yaw, bisector)) <= 1e-6;
  const bool ok_side = std::abs(dot_xy(side_debug, side_from_yaw) - 1.0) <= 1e-6;
  const bool ok_b = std::abs(dot_xy(bisector_debug, bisector) - 1.0) <= 1e-6;
  const bool ok_inward =
      (std::abs(inward.x) < 1e-9 && std::abs(inward.y) < 1e-9) ? true : (dot_xy(side_from_yaw, inward) <= 1e-9);
  return ok_sharp && ok_perp && ok_side && ok_b && ok_inward;
}

bool test_sharp_corner_threshold_boundary_orientation() {
  auto generate_middle = [](double corner_interior_deg) -> std::pair<double, bool> {
    CoreState state;
    const auto pole_type_ids = sorted_pole_type_ids(state);
    if (pole_type_ids.empty()) {
      return {std::numeric_limits<double>::quiet_NaN(), false};
    }
    constexpr double kPi = 3.14159265358979323846;
    const double interior_rad = corner_interior_deg * (kPi / 180.0);
    const double out_heading_rad = kPi - interior_rad;
    wire::core::RoadSegment road{};
    road.id = static_cast<std::uint64_t>(950 + static_cast<int>(corner_interior_deg * 10.0));
    // Unequal segment lengths so "bisector" and "chord(prev->next)" are different.
    road.polyline = {
        {-5.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {20.0 * std::cos(out_heading_rad), 20.0 * std::sin(out_heading_rad), 0.0},
    };
    const auto result =
        state.GenerateSimpleLineFromPoints(road, pole_type_ids.front(), wire::core::ConnectionCategory::kLowVoltage);
    if (!result.ok || result.value.pole_ids.size() != 3) {
      return {std::numeric_limits<double>::quiet_NaN(), false};
    }
    const auto* middle = state.view().edit_state().poles.find(result.value.pole_ids[1]);
    if (middle == nullptr) {
      return {std::numeric_limits<double>::quiet_NaN(), false};
    }
    return {middle->world_transform.rotation_euler_deg.z, middle->context.sharp_orientation_applied};
  };

  const auto [yaw74, sharp74] = generate_middle(74.0);
  const auto [yaw75, sharp75] = generate_middle(75.0);
  const auto [yaw76, sharp76] = generate_middle(76.0);
  if (!std::isfinite(yaw74) || !std::isfinite(yaw75) || !std::isfinite(yaw76)) {
    return false;
  }

  return sharp74 && !sharp75 && !sharp76 &&
         angle_diff_abs_deg(yaw74, yaw76) > 1e-3 &&
         angle_diff_abs_deg(yaw74, yaw75) > 1e-3;
}

bool test_generate_from_guide_reused_tilted_vertex_keeps_sharp_corner_perpendicular() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {2.9289321881345245, 7.0710678118654755, 0.0}};
  req.interval_m = 100.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto first = state.GenerateFromBackboneSpec(req);
  if (!first.ok) {
    return false;
  }

  ObjectId vertex_id = wire::core::kInvalidObjectId;
  for (const auto& pole : state.view().edit_state().poles.items()) {
    if (almost_equal(pole.world_transform.position, req.path.polyline[1], 1e-6)) {
      vertex_id = pole.id;
      break;
    }
  }
  if (vertex_id == wire::core::kInvalidObjectId) {
    return false;
  }

  if (!state.SetPoleTilt(vertex_id, 12.0).ok) {
    return false;
  }

  wire::core::BackboneInputSpec::NodeSpec reused{};
  reused.point_index = 1;
  reused.support_kind = wire::core::SupportKind::kPole;
  reused.node_id = vertex_id;
  req.path.node_specs.clear();
  req.path.node_specs.push_back(reused);
  const auto second = state.GenerateFromBackboneSpec(req);
  if (!second.ok) {
    return false;
  }

  const auto* vertex = state.view().edit_state().poles.find(vertex_id);
  const auto vertex_view = state.view().inspect_pole(vertex_id);
  if (vertex == nullptr || !vertex_view.has_value()) {
    return false;
  }

  const wire::core::PoleFrame frame =
      wire::core::BuildPoleFrame(vertex->world_transform, vertex_view->final_yaw_deg);
  const auto normalize_on_support_plane = [&](wire::core::Vec3d value) {
    value = value - wire::core::ScaleVec(frame.up, wire::core::Dot(value, frame.up));
    if (!wire::core::Normalize(&value)) {
      return wire::core::Vec3d{};
    }
    return value;
  };

  const wire::core::Vec3d projected_bisector = normalize_on_support_plane(vertex->context.sharp_bisector_dir);
  const wire::core::Vec3d projected_side = normalize_on_support_plane(vertex->context.sharp_side_dir);
  if (wire::core::LengthSquared(projected_bisector) <= 1e-12 || wire::core::LengthSquared(projected_side) <= 1e-12) {
    return false;
  }

  const bool ok = vertex->context.sharp_orientation_applied &&
                  std::abs(wire::core::Dot(projected_side, projected_bisector)) <= 0.03 &&
                  std::abs(wire::core::Dot(projected_side, frame.up)) <= 1e-6 &&
                  std::abs(wire::core::Dot(projected_bisector, frame.up)) <= 1e-6;
  if (!ok) {
    std::cerr << "[DBG] C294 sharp=" << (vertex->context.sharp_orientation_applied ? 1 : 0)
              << " side=(" << vertex->context.sharp_side_dir.x << "," << vertex->context.sharp_side_dir.y << ","
              << vertex->context.sharp_side_dir.z << ") bisector=(" << vertex->context.sharp_bisector_dir.x << ","
              << vertex->context.sharp_bisector_dir.y << "," << vertex->context.sharp_bisector_dir.z << ")"
              << " projSide=(" << projected_side.x << "," << projected_side.y << "," << projected_side.z << ")"
              << " projBisector=(" << projected_bisector.x << "," << projected_bisector.y << ","
              << projected_bisector.z << ") up=(" << frame.up.x << "," << frame.up.y << "," << frame.up.z
              << ") dot=" << wire::core::Dot(projected_side, projected_bisector) << "\n";
  }
  return ok;
}

bool test_generate_from_guide_reused_vertex_reorients_to_corner_rule() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req_obtuse{};
  req_obtuse.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {15.0, 8.660254037844386, 0.0}}; // interior ~=120
  req_obtuse.interval_m = 100.0;                                                                     // only vertices
  req_obtuse.pole_type_id = type_ids.front();
  add_backbone_bundle(req_obtuse, wire::core::BundleKind::kLowVoltage);
  const auto first = state.GenerateFromBackboneSpec(req_obtuse);
  if (!first.ok) {
    return false;
  }

  ObjectId vertex_id = wire::core::kInvalidObjectId;
  for (const auto& pole : state.view().edit_state().poles.items()) {
    if (almost_equal(pole.world_transform.position, req_obtuse.path.polyline[1], 1e-6)) {
      vertex_id = pole.id;
      break;
    }
  }
  if (vertex_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec req_acute = req_obtuse;
  req_acute.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {2.9289321881345245, 7.0710678118654755, 0.0}}; // ~45
  const auto second = state.GenerateFromBackboneSpec(req_acute);
  if (!second.ok) {
    return false;
  }
  const auto* vertex = state.view().edit_state().poles.find(vertex_id);
  if (vertex == nullptr) {
    return false;
  }

  const wire::core::Vec3d u0 = normalize_xy_safe(req_acute.path.polyline[0] - req_acute.path.polyline[1]); // corner->prev
  const wire::core::Vec3d u1 = normalize_xy_safe(req_acute.path.polyline[2] - req_acute.path.polyline[1]); // corner->next
  const wire::core::Vec3d bisector = normalize_xy_safe({u0.x + u1.x, u0.y + u1.y, 0.0});
  const wire::core::Vec3d side_from_yaw = local_side_axis_from_yaw(vertex->world_transform.rotation_euler_deg.z);
  return vertex->context.sharp_orientation_applied && std::abs(dot_xy(side_from_yaw, bisector)) <= 1e-6;
}

bool test_sharp_corner_orientation_consistent_across_entry_paths() {
  const std::vector<wire::core::Vec3d> polyline = {
      {0.0, 0.0, 0.0},
      {10.0, 0.0, 0.0},
      {5.0, 8.660254037844386, 0.0},
  };

  CoreState simple_state;
  const auto simple_type_ids = sorted_pole_type_ids(simple_state);
  if (simple_type_ids.empty()) {
    return false;
  }
  wire::core::RoadSegment road{};
  road.id = 960;
  road.polyline = polyline;
  const auto simple = simple_state.GenerateSimpleLineFromPoints(road, simple_type_ids.front(),
                                                                wire::core::ConnectionCategory::kLowVoltage);
  if (!simple.ok) {
    return false;
  }
  const ObjectId simple_middle_id = find_pole_id_by_position(simple_state, polyline[1]);
  if (simple_middle_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto* simple_middle = simple_state.view().edit_state().poles.find(simple_middle_id);
  if (simple_middle == nullptr) {
    return false;
  }

  CoreState backbone_state;
  const auto backbone_type_ids = sorted_pole_type_ids(backbone_state);
  if (backbone_type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = polyline;
  req.interval_m = 1000.0;
  req.pole_type_id = backbone_type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto backbone = backbone_state.GenerateFromBackboneSpec(req);
  if (!backbone.ok) {
    return false;
  }
  const ObjectId backbone_middle_id = find_pole_id_by_position(backbone_state, polyline[1]);
  if (backbone_middle_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto* backbone_middle = backbone_state.view().edit_state().poles.find(backbone_middle_id);
  if (backbone_middle == nullptr) {
    return false;
  }

  const wire::core::Vec3d simple_side = normalize_xy_safe(simple_middle->context.sharp_side_dir);
  const wire::core::Vec3d backbone_side = normalize_xy_safe(backbone_middle->context.sharp_side_dir);
  const wire::core::Vec3d simple_bisector = normalize_xy_safe(simple_middle->context.sharp_bisector_dir);
  const wire::core::Vec3d backbone_bisector = normalize_xy_safe(backbone_middle->context.sharp_bisector_dir);

  return simple_middle->context.sharp_orientation_applied &&
         backbone_middle->context.sharp_orientation_applied &&
         angle_diff_abs_deg(simple_middle->world_transform.rotation_euler_deg.z,
                            backbone_middle->world_transform.rotation_euler_deg.z) <= 1e-6 &&
         std::abs(dot_xy(simple_side, backbone_side) - 1.0) <= 1e-6 &&
         std::abs(dot_xy(simple_bisector, backbone_bisector) - 1.0) <= 1e-6;
}

bool test_generate_from_guide_reused_pole_reprojects_owned_ports() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req_obtuse{};
  req_obtuse.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {15.0, 8.660254037844386, 0.0}}; // interior ~=120
  req_obtuse.interval_m = 100.0; // vertices only
  req_obtuse.pole_type_id = type_ids.front();
  add_backbone_bundle(req_obtuse, wire::core::BundleKind::kLowVoltage);
  const auto first = state.GenerateFromBackboneSpec(req_obtuse);
  if (!first.ok) {
    return false;
  }

  ObjectId vertex_id = wire::core::kInvalidObjectId;
  for (const auto& pole : state.view().edit_state().poles.items()) {
    if (almost_equal(pole.world_transform.position, req_obtuse.path.polyline[1], 1e-6)) {
      vertex_id = pole.id;
      break;
    }
  }
  if (vertex_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const wire::core::Pole* before_pole = state.view().edit_state().poles.find(vertex_id);
  if (before_pole == nullptr) {
    return false;
  }
  double yaw_before = before_pole->world_transform.rotation_euler_deg.z;

  ObjectId left_port_id = wire::core::kInvalidObjectId;
  ObjectId right_port_id = wire::core::kInvalidObjectId;
  wire::core::Vec3d left_port_before{};
  wire::core::Vec3d right_port_before{};
  for (const auto& port : state.view().edit_state().ports.items()) {
    if (port.owner_pole_id != vertex_id) {
      continue;
    }
    if (port.template_side == wire::core::SlotSide::kLeft) {
      left_port_id = port.id;
      left_port_before = port.world_position;
    } else if (port.template_side == wire::core::SlotSide::kRight) {
      right_port_id = port.id;
      right_port_before = port.world_position;
    }
  }
  if (left_port_id == wire::core::kInvalidObjectId || right_port_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec req_acute = req_obtuse;
  req_acute.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {2.9289321881345245, 7.0710678118654755, 0.0}}; // ~45
  const auto second = state.GenerateFromBackboneSpec(req_acute);
  if (!second.ok) {
    return false;
  }

  const wire::core::Pole* after_pole = state.view().edit_state().poles.find(vertex_id);
  const wire::core::Port* left_port_after = state.view().edit_state().ports.find(left_port_id);
  const wire::core::Port* right_port_after = state.view().edit_state().ports.find(right_port_id);
  if (after_pole == nullptr || left_port_after == nullptr || right_port_after == nullptr) {
    return false;
  }

  const double yaw_after = after_pole->world_transform.rotation_euler_deg.z;
  const bool yaw_changed = angle_diff_abs_deg(yaw_before, yaw_after) > 1e-3;
  const bool moved_left = !almost_equal(left_port_after->world_position, left_port_before, 1e-6);
  const bool moved_right = !almost_equal(right_port_after->world_position, right_port_before, 1e-6);
  return yaw_changed && (moved_left || moved_right);
}

bool test_generate_from_guide_with_duplicate_points_is_robust() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {
      {0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0}, // duplicate
      {20.0, 0.0, 0.0},
      {20.0, 0.0, 0.0}, // duplicate
      {40.0, 0.0, 0.0},
  };
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto result = state.GenerateFromBackboneSpec(req);
  if (!result.ok || result.value.generated_pole_ids.size() < 2) {
    return false;
  }
  for (ObjectId pole_id : result.value.generated_pole_ids) {
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr) {
      return false;
    }
    const auto& p = pole->world_transform.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      return false;
    }
    if (!almost_equal(p.z, 0.0, 1e-9)) {
      return false;
    }
  }
  return validate_now(state).ok();
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

  wire::core::AddConnectionByPoleOptions options{};
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

bool test_inspection_backbone_uses_rebuilt_result_instead_of_last_snapshot() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req_a{};
  req_a.path.polyline = {{-8.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {8.0, 0.0, 0.0}};
  req_a.interval_m = 8.0;
  req_a.pole_type_id = type_ids.front();
  helpers::add_backbone_bundle(req_a, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(req_a).ok) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  wire::core::BackboneSpec req_b{};
  req_b.path.polyline = {{0.0, -8.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 8.0, 0.0}};
  req_b.interval_m = 8.0;
  req_b.pole_type_id = type_ids.front();
  wire::core::BackboneInputSpec::NodeSpec shared_center{};
  shared_center.point_index = 1;
  shared_center.support_kind = wire::core::SupportKind::kPole;
  shared_center.node_id = center_id;
  req_b.path.node_specs.push_back(shared_center);
  helpers::add_backbone_bundle(req_b, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(req_b).ok) {
    return false;
  }

  const auto rebuilt_before = state.BuildBackboneResult();
  const auto* rebuilt_junction_before = find_junction(rebuilt_before, center_id);
  const auto rebuilt_support_node_before =
      std::find_if(rebuilt_before.nodes.begin(), rebuilt_before.nodes.end(),
                   [&](const wire::core::SupportNode& node) { return node.node_id == center_id; });
  if (rebuilt_junction_before == nullptr || rebuilt_support_node_before == rebuilt_before.nodes.end()) {
    return false;
  }

  const auto junction_view_before = state.view().inspect_junction(center_id);
  const auto support_node_view_before =
      state.view().describe_entity({wire::core::EntityKind::kSupportNode, center_id});
  if (!junction_view_before.has_value() || !support_node_view_before.has_value() ||
      junction_view_before->incidents.size() != rebuilt_junction_before->incidents.size()) {
    return false;
  }

  wire::core::BackboneSpec req_c{};
  req_c.path.polyline = {{40.0, 0.0, 0.0}, {48.0, 0.0, 0.0}, {56.0, 0.0, 0.0}};
  req_c.interval_m = 8.0;
  req_c.pole_type_id = type_ids.front();
  helpers::add_backbone_bundle(req_c, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(req_c).ok) {
    return false;
  }

  const auto rebuilt_after = state.BuildBackboneResult();
  const auto rebuilt_support_node_it =
      std::find_if(rebuilt_after.nodes.begin(), rebuilt_after.nodes.end(),
                   [&](const wire::core::SupportNode& node) { return node.node_id == center_id; });
  if (rebuilt_support_node_it == rebuilt_after.nodes.end()) {
    return false;
  }

  const auto support_node_view_after =
      state.view().describe_entity({wire::core::EntityKind::kSupportNode, center_id});
  return support_node_view_after.has_value();
}

bool test_inspection_junction_prefers_relation_surface_when_present() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req_a{};
  req_a.path.polyline = {{-8.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {8.0, 0.0, 0.0}};
  req_a.interval_m = 8.0;
  req_a.pole_type_id = type_ids.front();
  helpers::add_backbone_bundle(req_a, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(req_a).ok) {
    return false;
  }

  wire::core::BackboneSpec req_b{};
  req_b.path.polyline = {{0.0, -8.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 8.0, 0.0}};
  req_b.interval_m = 8.0;
  req_b.pole_type_id = type_ids.front();
  helpers::add_backbone_bundle(req_b, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(req_b).ok) {
    return false;
  }

  const auto rebuilt = state.BuildBackboneResult();
  if (rebuilt.junctions.empty()) {
    return false;
  }
  const ObjectId junction_id = rebuilt.junctions.front().node_id;
  auto& relations = wire::core::CoreStateTestHook::last_generation_junction_relations(state);
  auto it = relations.find(junction_id);
  if (it == relations.end() || it->second.incidents.size() < 3) {
    return false;
  }

  it->second.through_pair.accepted = true;
  it->second.through_pair.neighbor_a_id = it->second.incidents[0].neighbor_node_id;
  it->second.through_pair.neighbor_b_id = it->second.incidents[1].neighbor_node_id;
  it->second.incidents.resize(2);

  const auto junction_view = state.view().inspect_junction(junction_id);
  return junction_view.has_value() && junction_view->incidents.size() == 2 && junction_view->has_local_relation &&
         junction_view->has_primary && junction_view->through_pair_accepted;
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
  test_registry::AddTest(tests, "C19_Phase45_GeneratePolesAlongRoad_Basic", "Road interval generates pole line with pole type", "Invariant", false, test_generate_poles_along_road_basic);
  test_registry::AddTest(tests, "C30_Phase47_PoleContext_Classification", "Pole context classification marks terminal/straight/corner", "Invariant", false, test_pole_context_classification_basic);
  test_registry::AddTest(tests, "C31_Phase47_AngleCorrection_Bounds", "Angle correction side scale stays finite and bounded", "Invariant", false, test_angle_correction_bounds_and_finite);
  test_registry::AddTest(tests, "C35_Phase47_CornerTurnSign_OuterBias", "Corner turn sign expands outer side more than inner side", "Invariant", false, test_corner_turn_sign_biases_outer_side);
  test_registry::AddTest(tests, "C61_Phase48h_AcuteCorner_AutoWidenSpacing", "Acute corners auto-widen lane spacing without category-specific branching", "Invariant", false, test_acute_corner_auto_widens_lane_spacing);
  test_registry::AddTest(tests, "C32_Phase47_SlotSelection_ContextBias", "Branch context biases band choice away from trunk-only", "Invariant", false, test_band_selection_context_bias);
  test_registry::AddTest(tests, "C33_Phase47_SlotSelection_DeterministicDebug", "Band tie-break is deterministic and debug record is coherent", "Exact", false, test_band_selection_deterministic_and_debug_integrity);
  test_registry::AddTest(tests, "C34_Phase47_GenerateSimpleLine_CornerContext", "Corner path generation keeps the guide vertex pole classified as corner under canonical path generation", "Invariant", false, test_generate_simple_line_corner_context_integration);
  test_registry::AddTest(tests, "C36_Phase47_DrawPath_ClickPointsExact", "DrawPath generation uses clicked points directly and sets pole yaw", "Exact", false, test_generate_simple_line_from_points_exact_poles_and_orientation);
  test_registry::AddTest(tests, "C43_Phase4x_SharpCorner_SideAxisPerpendicular", "Sharp-corner pole side axis is perpendicular to bisector and points away from inward side", "Invariant", false, test_generate_simple_line_from_points_sharp_corner_perpendicular_orientation);
  test_registry::AddTest(tests, "C56_Phase48h_SharpCorner_ThresholdBoundary", "Sharp-corner orientation applies at <=74deg and disables above threshold", "Invariant", false, test_sharp_corner_threshold_boundary_orientation);
  test_registry::AddTest(tests, "C60_Phase48h_Guide_ReusedVertexReorient", "Reused guide vertex pole is reoriented by sharp-corner rule when not manually overridden", "Invariant", false, test_generate_from_guide_reused_vertex_reorients_to_corner_rule);
  test_registry::AddTest(tests, "C294_Phase48h_Guide_ReusedTiltedVertexSharpCorner", "Reused tilted guide vertex keeps sharp-corner side axis perpendicular on the tilted support plane", "Invariant", false, test_generate_from_guide_reused_tilted_vertex_keeps_sharp_corner_perpendicular);
  test_registry::AddTest(tests, "C108_Phase56_SharpCorner_EntryConsistency", "Sharp-corner pole orientation stays identical between simple-line and backbone entry paths", "Invariant", false, test_sharp_corner_orientation_consistent_across_entry_paths);
  test_registry::AddTest(tests, "C70_Phase48h_Guide_ReusedVertexPortReproject", "Reused guide vertex reprojections move template-owned ports when corner orientation changes", "Invariant", false, test_generate_from_guide_reused_pole_reprojects_owned_ports);
  test_registry::AddTest(tests, "C57_Phase48h_Guide_DuplicatePointsRobust", "Guide generation with duplicate points stays finite and keeps path height along world up", "Invariant", false, test_generate_from_guide_with_duplicate_points_is_robust);
  test_registry::AddTest(tests, "C58_Phase48h_Guide_ReverseSymmetry", "Guide reverse mode preserves generated pole position set", "Invariant", false, test_generate_from_guide_reverse_mode_position_symmetry);
  test_registry::AddTest(tests, "C59_Phase48h_Guide_AvoidConstraint", "Guide generation avoids forbidden radius around avoid_points", "Invariant", false, test_generate_from_guide_respects_avoid_constraints);
  test_registry::AddTest(tests, "C37_Phase48_PreferredSide_Geometry", "Preferred side is decided by peer geometry", "Invariant", false, test_preferred_side_uses_geometry);
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
  test_registry::AddTest(tests, "C258_Inspection_Backbone_UsesRebuiltResult",
                         "Inspection resolves junction/support-node entities from rebuilt backbone instead of stale last_generation_backbone snapshot",
                         "Invariant", false, test_inspection_backbone_uses_rebuilt_result_instead_of_last_snapshot);
  test_registry::AddTest(tests, "C259_Inspection_Junction_PrefersRelationSurface",
                         "Inspection prefers last_generation_junction_relations when relation data exists instead of mixing rebuilt junction incidents into the same surface",
                         "Invariant", false, test_inspection_junction_prefers_relation_surface_when_present);
  test_registry::AddTest(tests, "C161_DetailCurve_BranchLongSpan_SuppressesSidewaysRunout",
                         "Long branch spans keep support departure local and suppress large sideways runout",
                         "Invariant", false, test_detail_curve_branch_long_span_suppresses_sideways_runout);
  test_registry::AddTest(tests, "C162_DetailCurve_BranchShortVsLong_LocalDepartureScaling",
                         "Branch local departure stays stronger on short spans and decays on long spans",
                         "Invariant", false, test_detail_curve_branch_short_span_keeps_more_local_departure_than_long_span);
  test_registry::AddTest(tests, "C163_DetailCurve_MainSag_StrongerThanBranch",
                         "Main spans keep a stronger sag read than branch spans without breaking endpoint constraints",
                         "Invariant", false, test_detail_curve_main_sag_reads_stronger_than_branch);
}

WIRE_REGISTER_TEST_SUITE(register_geometry_tests);

} // namespace
