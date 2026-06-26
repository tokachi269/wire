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

wire::core::AttachmentTemplateId find_attachment_template_by_mode(
    const CoreState& state, wire::core::AttachmentLineInteractionMode mode) {
  for (const auto& [id, attachment_template] : state.view().attachment_templates()) {
    if (attachment_template.line_interaction_mode == mode) {
      return id;
    }
  }
  return wire::core::kInvalidAttachmentTemplateId;
}

wire::core::AttachmentTemplateId find_attachment_template_by_profile(
    const CoreState& state, wire::core::AttachmentInternalPathTemplate::ProfileKind profile_kind) {
  for (const auto& [id, attachment_template] : state.view().attachment_templates()) {
    if (attachment_template.internal_paths.empty()) {
      continue;
    }
    if (attachment_template.internal_paths.front().profile_kind == profile_kind) {
      return id;
    }
  }
  return wire::core::kInvalidAttachmentTemplateId;
}

double polyline_length(const std::vector<wire::core::Vec3d>& points) {
  double length = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const wire::core::Vec3d delta = points[i + 1] - points[i];
    length += std::sqrt(wire::core::LengthSquared(delta));
  }
  return length;
}

bool build_attachment_test_span(CoreState& state, ObjectId* out_span) {
  if (out_span == nullptr) {
    return false;
  }
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 4.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {12.0, 0.0, 4.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  *out_span = span;
  return state.Commit().validation.ok();
}

bool has_entity_link(const std::vector<wire::core::RelatedEntityLink>& links, wire::core::EntityKind kind,
                     std::uint64_t stable_id) {
  return std::any_of(links.begin(), links.end(), [kind, stable_id](const wire::core::RelatedEntityLink& link) {
    return link.ref.kind == kind && link.ref.stable_id == stable_id;
  });
}

bool test_curve_cache_line_mode_is_deterministic() {
  CoreState state;
  wire::core::GeometrySettings settings{};
  settings.curve_samples = 9;
  settings.sag_enabled = false;
  settings.sag_factor = 0.0;
  (void)state.UpdateGeometrySettings(settings, false);

  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 5.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {10.0, 0.0, 5.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.Commit().recalc_stats;

  const auto* curve1 = state.find_curve_cache(span);
  if (curve1 == nullptr || curve1->points.size() != 9) {
    return false;
  }

  const auto move_result = state.MovePort(a, {0.0, 0.0, 5.0});
  if (!move_result.ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto* curve2 = state.find_curve_cache(span);
  if (curve2 == nullptr || curve2->points.size() != curve1->points.size()) {
    return false;
  }

  if (!almost_equal(curve2->points.front(), {0.0, 0.0, 5.0}) ||
      !almost_equal(curve2->points.back(), {10.0, 0.0, 5.0})) {
    return false;
  }
  for (std::size_t i = 0; i < curve1->points.size(); ++i) {
    if (!almost_equal(curve1->points[i], curve2->points[i])) {
      return false;
    }
  }
  return true;
}

bool test_sag_mode_changes_midpoint_and_keeps_endpoints() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 4.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {12.0, 0.0, 4.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;

  wire::core::GeometrySettings line{};
  line.curve_samples = 11;
  line.sag_enabled = false;
  line.sag_factor = 0.05;
  (void)state.UpdateGeometrySettings(line, true);
  (void)state.Commit().recalc_stats;
  const auto* line_curve = state.find_curve_cache(span);
  if (line_curve == nullptr || line_curve->points.size() != 11) {
    return false;
  }
  const std::vector<wire::core::Vec3d> line_points = line_curve->points;

  wire::core::GeometrySettings sag = line;
  sag.sag_enabled = true;
  sag.sag_factor = 0.10;
  (void)state.UpdateGeometrySettings(sag, true);
  (void)state.Commit().recalc_stats;
  const auto* sag_curve = state.find_curve_cache(span);
  if (sag_curve == nullptr || sag_curve->points.size() != 11) {
    return false;
  }

  const std::size_t mid = sag_curve->points.size() / 2;
  return almost_equal(sag_curve->points.front(), line_points.front()) &&
         almost_equal(sag_curve->points.back(), line_points.back()) &&
         wire::core::HeightAlongWorldUp(sag_curve->points[mid]) < wire::core::HeightAlongWorldUp(line_points[mid]);
}

bool test_geometry_bounds_version_follow_and_locality() {
  CoreState state;
  wire::core::GeometrySettings settings{};
  settings.curve_samples = 8;
  settings.sag_enabled = false;
  settings.sag_factor = 0.0;
  (void)state.UpdateGeometrySettings(settings, false);

  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId p1 = state.AddPort(pole, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId p2 = state.AddPort(pole, {5.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId p3 = state.AddPort(pole, {10.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId p4 = state.AddPort(pole, {15.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span_a = state.AddSpan(p1, p2, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  const ObjectId span_b = state.AddSpan(p3, p4, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.Commit().recalc_stats;

  const auto* before_b = state.view().find_span_runtime_state(span_b);
  if (before_b == nullptr) {
    return false;
  }
  const std::uint64_t span_b_geometry_before = before_b->geometry_version;

  const auto move_result = state.MovePort(p1, {0.0, 1.0, 1.0});
  if (!move_result.ok) {
    return false;
  }
  const auto* dirty_a = state.view().find_span_runtime_state(span_a);
  const auto* dirty_b = state.view().find_span_runtime_state(span_b);
  if (dirty_a == nullptr || dirty_b == nullptr) {
    return false;
  }
  if (!has_dirty(dirty_a, DirtyBits::kGeometry) || has_dirty(dirty_b, DirtyBits::kGeometry)) {
    return false;
  }

  (void)state.Commit().recalc_stats;
  const auto* after_a = state.view().find_span_runtime_state(span_a);
  const auto* after_b = state.view().find_span_runtime_state(span_b);
  if (after_a == nullptr || after_b == nullptr) {
    return false;
  }

  return after_a->geometry_version == after_a->data_version && after_a->bounds_version == after_a->data_version &&
         after_a->render_version == after_a->data_version && after_b->geometry_version == span_b_geometry_before;
}

bool test_bounds_cache_generated_and_valid() {
  CoreState state;
  wire::core::GeometrySettings settings{};
  settings.curve_samples = 7;
  settings.sag_enabled = true;
  settings.sag_factor = 0.08;
  (void)state.UpdateGeometrySettings(settings, false);

  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 5.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {10.0, 2.0, 5.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.Commit().recalc_stats;

  const auto* curve = state.find_curve_cache(span);
  const auto* bounds = state.find_bounds_cache(span);
  if (curve == nullptr || bounds == nullptr) {
    return false;
  }
  if (curve->points.size() < 2 || bounds->segments.size() != curve->points.size() - 1) {
    return false;
  }
  if (!aabb_valid(bounds->whole)) {
    return false;
  }
  for (const auto& segment : bounds->segments) {
    if (!aabb_valid(segment)) {
      return false;
    }
  }
  return true;
}

bool test_bounds_follow_geometry_change() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 6.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {12.0, 0.0, 6.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;

  wire::core::GeometrySettings line{};
  line.curve_samples = 9;
  line.sag_enabled = false;
  line.sag_factor = 0.1;
  (void)state.UpdateGeometrySettings(line, true);
  (void)state.Commit().recalc_stats;
  const auto* bounds_line = state.find_bounds_cache(span);
  if (bounds_line == nullptr) {
    return false;
  }
  const double line_min_height = wire::core::HeightAlongWorldUp(bounds_line->whole.min);

  wire::core::GeometrySettings sag = line;
  sag.sag_enabled = true;
  (void)state.UpdateGeometrySettings(sag, true);
  (void)state.Commit().recalc_stats;
  const auto* bounds_sag = state.find_bounds_cache(span);
  if (bounds_sag == nullptr) {
    return false;
  }

  return wire::core::HeightAlongWorldUp(bounds_sag->whole.min) < line_min_height;
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
  (void)state.Commit().recalc_stats;

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

bool test_attachment_display_offset_does_not_change_detail_curve_endpoints() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 4.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {12.0, 0.0, 4.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.Commit().recalc_stats;

  const auto* before = state.find_curve_cache(span);
  if (before == nullptr) {
    return false;
  }
  const wire::core::Vec3d start_before = before->detail.EvaluatePosition(0.0);
  const wire::core::Vec3d end_before = before->detail.EvaluatePosition(1.0);

  const auto add_attachment = state.AddAttachment(span, 0.5, wire::core::AttachmentKind::kGeneric, 1.25);
  if (!add_attachment.ok) {
    return false;
  }
  const auto* attachment = state.view().attachments().find(add_attachment.value);
  if (attachment == nullptr || !almost_equal(attachment->display_offset_m, 1.25, 1e-9)) {
    return false;
  }

  (void)state.Commit().recalc_stats;
  const auto* after = state.find_curve_cache(span);
  if (after == nullptr) {
    return false;
  }
  return almost_equal(after->detail.EvaluatePosition(0.0), start_before, 1e-9) &&
         almost_equal(after->detail.EvaluatePosition(1.0), end_before, 1e-9);
}

bool test_attachment_pass_through_keeps_outer_curve_visible() {
  CoreState state;
  ObjectId span = wire::core::kInvalidObjectId;
  if (!build_attachment_test_span(state, &span)) {
    return false;
  }
  const auto template_id =
      find_attachment_template_by_mode(state, wire::core::AttachmentLineInteractionMode::kPassThrough);
  if (template_id == wire::core::kInvalidAttachmentTemplateId) {
    return false;
  }
  if (!state.AddAttachment(span, 0.5, wire::core::AttachmentKind::kGeneric, 0.0, template_id).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto* curve = state.find_curve_cache(span);
  return curve != nullptr && curve->detail.hidden_intervals.empty() && curve->detail.replacement_paths.empty() &&
         curve->detail.visible_intervals.size() == 1 &&
         almost_equal(curve->detail.visible_intervals.front().start_m, 0.0, 1e-9) &&
         almost_equal(curve->detail.visible_intervals.front().end_m, curve->detail.Length(), 1e-9);
}

bool test_attachment_hide_segment_masks_curve_interval() {
  CoreState state;
  ObjectId span = wire::core::kInvalidObjectId;
  if (!build_attachment_test_span(state, &span)) {
    return false;
  }
  const auto template_id =
      find_attachment_template_by_mode(state, wire::core::AttachmentLineInteractionMode::kHideSegment);
  if (template_id == wire::core::kInvalidAttachmentTemplateId) {
    return false;
  }
  if (!state.AddAttachment(span, 0.5, wire::core::AttachmentKind::kMarker, 0.0, template_id).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto* curve = state.find_curve_cache(span);
  return curve != nullptr && !curve->detail.hidden_intervals.empty() && curve->detail.replacement_paths.empty() &&
         curve->detail.visible_intervals.size() == 2;
}

bool test_attachment_replace_with_internal_path_replaces_interval() {
  CoreState state;
  ObjectId span = wire::core::kInvalidObjectId;
  if (!build_attachment_test_span(state, &span)) {
    return false;
  }
  const auto template_id = find_attachment_template_by_profile(
      state, wire::core::AttachmentInternalPathTemplate::ProfileKind::kExplicitPolyline);
  if (template_id == wire::core::kInvalidAttachmentTemplateId) {
    return false;
  }
  if (!state.AddAttachment(span, 0.5, wire::core::AttachmentKind::kSpacer, 0.0, template_id).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto* curve = state.find_curve_cache(span);
  return curve != nullptr && !curve->detail.hidden_intervals.empty() && !curve->detail.replacement_paths.empty() &&
         curve->detail.replacement_paths.front().points.size() >= 2;
}

bool test_curve_offset_straight_cable_supplemental_uses_deterministic_wobble() {
  CoreState state;
  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = false;
  settings.sag_factor = 0.0;
  if (!state.UpdateGeometrySettings(settings, false).ok) {
    return false;
  }

  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::Transformd tf_a{};
  tf_a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd tf_b{};
  tf_b.position = {24.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(tf_a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(tf_b, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = true;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  const auto add = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, options);
  if (!add.ok) {
    return false;
  }

  const auto* span = state.view().edit_state().spans.find(add.value.span_id);
  if (span == nullptr) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
  if (bundle == nullptr) {
    return false;
  }
  const auto bundle_it = state.view().bundle_templates().find(bundle->bundle_template_id);
  if (bundle_it == state.view().bundle_templates().end()) {
    return false;
  }
  const auto cable_it = state.view().cable_templates().find(bundle_it->second.cable_template_id);
  if (cable_it == state.view().cable_templates().end()) {
    return false;
  }

  wire::core::CableTemplate cable = cable_it->second;
  cable.supplemental_paths.clear();
  wire::core::CableSupplementalPathTemplate supplemental{};
  supplemental.anchor_mode = wire::core::CableSupplementalPathTemplate::AnchorMode::kCurveOffset;
  supplemental.profile_kind = wire::core::CableSupplementalPathTemplate::ProfileKind::kStraightCable;
  supplemental.lateral_offset_m = 0.28;
  supplemental.wobble_amplitude_m = 0.09;
  supplemental.wobble_wavelength_m = 6.0;
  supplemental.endpoint_envelope_ratio = 0.2;
  cable.supplemental_paths.push_back(supplemental);
  if (!state.UpdateCableTemplate(cable).ok) {
    return false;
  }

  (void)state.Commit().recalc_stats;
  const auto* curve_before = state.find_curve_cache(add.value.span_id);
  if (curve_before == nullptr || curve_before->detail.supplemental_paths.size() != 1) {
    return false;
  }
  const std::vector<wire::core::Vec3d> points_before = curve_before->detail.supplemental_paths.front().points;
  if (points_before.size() <= 8) {
    return false;
  }

  const wire::core::Vec3d chord = points_before.back() - points_before.front();
  const double chord_len_sq = wire::core::LengthSquared(chord);
  if (chord_len_sq <= 1e-9) {
    return false;
  }
  double max_deviation_m = 0.0;
  for (std::size_t i = 1; i + 1 < points_before.size(); ++i) {
    const wire::core::Vec3d delta = points_before[i] - points_before.front();
    const double t = wire::core::Dot(delta, chord) / chord_len_sq;
    const wire::core::Vec3d projected = points_before.front() + wire::core::ScaleVec(chord, t);
    max_deviation_m = std::max(max_deviation_m,
                               std::sqrt(wire::core::LengthSquared(points_before[i] - projected)));
  }
  if (max_deviation_m <= 0.015) {
    return false;
  }

  const wire::core::GeometrySettings rebuild_settings = state.view().geometry_settings();
  if (!state.UpdateGeometrySettings(rebuild_settings, true).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;

  const auto* curve_after = state.find_curve_cache(add.value.span_id);
  if (curve_after == nullptr || curve_after->detail.supplemental_paths.size() != 1) {
    return false;
  }
  const auto& points_after = curve_after->detail.supplemental_paths.front().points;
  if (points_before.size() != points_after.size()) {
    return false;
  }
  for (std::size_t i = 0; i < points_before.size(); ++i) {
    if (!almost_equal(points_before[i], points_after[i], 1e-9)) {
      return false;
    }
  }
  return validate_now(state).ok();
}

bool test_context_profile_influences_detail_wobble_without_breaking_determinism() {
  CoreState state;
  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.sag_enabled = false;
  settings.sag_factor = 0.0;
  if (!state.UpdateGeometrySettings(settings, false).ok) {
    return false;
  }

  wire::core::ContextProfile calm{};
  calm.age = 0.3;
  calm.clutter = 0.2;
  calm.regularity = 0.85;
  calm.service_mix = 0.4;
  calm.style_seed = 9;
  if (!state.UpdateContextProfile(calm).ok) {
    return false;
  }

  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::Transformd tf_a{};
  tf_a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd tf_b{};
  tf_b.position = {24.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(tf_a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(tf_b, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }

  wire::core::AddConnectionByPoleOptions options{};
  options.auto_create_bundle = true;
  options.use_bundle_template = true;
  options.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  const auto add = state.AddConnectionByPole(pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage, options);
  if (!add.ok) {
    return false;
  }

  const auto* span = state.view().edit_state().spans.find(add.value.span_id);
  if (span == nullptr) {
    return false;
  }
  const auto* bundle = state.view().edit_state().bundles.find(span->bundle_id);
  if (bundle == nullptr) {
    return false;
  }
  const auto bundle_it = state.view().bundle_templates().find(bundle->bundle_template_id);
  if (bundle_it == state.view().bundle_templates().end()) {
    return false;
  }
  const auto cable_it = state.view().cable_templates().find(bundle_it->second.cable_template_id);
  if (cable_it == state.view().cable_templates().end()) {
    return false;
  }

  wire::core::CableTemplate cable = cable_it->second;
  cable.supplemental_paths.clear();
  wire::core::CableSupplementalPathTemplate supplemental{};
  supplemental.anchor_mode = wire::core::CableSupplementalPathTemplate::AnchorMode::kCurveOffset;
  supplemental.profile_kind = wire::core::CableSupplementalPathTemplate::ProfileKind::kStraightCable;
  supplemental.lateral_offset_m = 0.28;
  supplemental.wobble_amplitude_m = 0.09;
  supplemental.wobble_wavelength_m = 6.0;
  supplemental.endpoint_envelope_ratio = 0.2;
  cable.supplemental_paths.push_back(supplemental);
  if (!state.UpdateCableTemplate(cable).ok) {
    return false;
  }

  (void)state.Commit().recalc_stats;
  const auto* curve_calm = state.find_curve_cache(add.value.span_id);
  if (curve_calm == nullptr || curve_calm->detail.supplemental_paths.size() != 1) {
    return false;
  }
  const std::vector<wire::core::Vec3d> points_calm = curve_calm->detail.supplemental_paths.front().points;
  if (points_calm.size() <= 8) {
    return false;
  }

  wire::core::ContextProfile busy = calm;
  busy.clutter = 0.9;
  busy.regularity = 0.15;
  if (!state.UpdateContextProfile(busy).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;

  const auto* curve_busy = state.find_curve_cache(add.value.span_id);
  if (curve_busy == nullptr || curve_busy->detail.supplemental_paths.size() != 1) {
    return false;
  }
  const std::vector<wire::core::Vec3d> points_busy = curve_busy->detail.supplemental_paths.front().points;
  if (points_calm.size() != points_busy.size()) {
    return false;
  }

  double max_delta = 0.0;
  for (std::size_t i = 0; i < points_calm.size(); ++i) {
    max_delta = std::max(max_delta, std::sqrt(wire::core::LengthSquared(points_calm[i] - points_busy[i])));
  }
  if (max_delta <= 0.005) {
    return false;
  }

  if (!state.UpdateContextProfile(busy).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto* curve_busy_repeat = state.find_curve_cache(add.value.span_id);
  if (curve_busy_repeat == nullptr || curve_busy_repeat->detail.supplemental_paths.size() != 1) {
    return false;
  }
  const auto& points_busy_repeat = curve_busy_repeat->detail.supplemental_paths.front().points;
  if (points_busy.size() != points_busy_repeat.size()) {
    return false;
  }
  for (std::size_t i = 0; i < points_busy.size(); ++i) {
    if (!almost_equal(points_busy[i], points_busy_repeat[i], 1e-9)) {
      return false;
    }
  }
  return validate_now(state).ok();
}

bool test_attachment_socket_endpoint_can_override_curve_endpoint() {
  CoreState state;
  ObjectId span = wire::core::kInvalidObjectId;
  if (!build_attachment_test_span(state, &span)) {
    return false;
  }
  const auto template_id =
      find_attachment_template_by_mode(state, wire::core::AttachmentLineInteractionMode::kPassThrough);
  if (template_id == wire::core::kInvalidAttachmentTemplateId) {
    return false;
  }
  const auto add_attachment = state.AddAttachment(span, 0.0, wire::core::AttachmentKind::kGeneric, 0.2, template_id);
  if (!add_attachment.ok) {
    return false;
  }
  wire::core::Span* span_edit = wire::core::CoreStateTestHook::edit_state(state).spans.find(span);
  if (span_edit == nullptr) {
    return false;
  }
  span_edit->endpoint_attachment_a_id = add_attachment.value;
  if (!state.SetSpanEndpointSocketOverride(span, true, 1).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto* curve = state.find_curve_cache(span);
  if (curve == nullptr) {
    return false;
  }
  return almost_equal(curve->detail.EvaluatePosition(0.0), wire::core::Vec3d{0.12, 0.0, 4.2}, 1e-6);
}

bool test_attachment_behavior_is_not_name_driven() {
  CoreState state;
  const auto template_id =
      find_attachment_template_by_mode(state, wire::core::AttachmentLineInteractionMode::kHideSegment);
  if (template_id == wire::core::kInvalidAttachmentTemplateId) {
    return false;
  }
  const auto it = state.view().attachment_templates().find(template_id);
  if (it == state.view().attachment_templates().end()) {
    return false;
  }
  wire::core::AttachmentTemplate attachment_template = it->second;
  attachment_template.name = "totally_unrelated_object_name";
  if (!state.UpdateAttachmentTemplate(attachment_template, false).ok) {
    return false;
  }

  ObjectId span = wire::core::kInvalidObjectId;
  if (!build_attachment_test_span(state, &span)) {
    return false;
  }
  if (!state.AddAttachment(span, 0.5, wire::core::AttachmentKind::kMarker, 0.0, template_id).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto* curve = state.find_curve_cache(span);
  return curve != nullptr && !curve->detail.hidden_intervals.empty();
}

bool test_inspection_pole_template_override_and_junction_surfaces_are_visible() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req_a{};
  req_a.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}};
  req_a.interval_m = 8.0;
  req_a.pole_type_id = type_ids.front();
  helpers::add_backbone_bundle(req_a, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(req_a).ok) {
    return false;
  }
  wire::core::BackboneSpec req_b{};
  req_b.path.polyline = {{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}};
  req_b.interval_m = 8.0;
  req_b.pole_type_id = type_ids.front();
  helpers::add_backbone_bundle(req_b, wire::core::BundleKind::kLowVoltage);
  if (!state.GenerateFromBackboneSpec(req_b).ok) {
    return false;
  }
  const ObjectId center = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center == wire::core::kInvalidObjectId) {
    return false;
  }
  wire::core::Pole* center_edit = wire::core::CoreStateTestHook::edit_state(state).poles.find(center);
  if (center_edit == nullptr) {
    return false;
  }
  const wire::core::PoleTypeId pole_type_id = type_ids.front();
  center_edit->pole_type_id = pole_type_id;
  if (!state.SetPoleManualYawOverride(center, 17.5).ok || !state.SetPoleFlip180(center, true).ok) {
    return false;
  }

  (void)state.Commit().recalc_stats;
  const auto view = state.view();
  const auto pole_view = view.inspect_pole(center);
  const auto template_view = view.inspect_pole_template(pole_type_id);
  const auto override_view = view.inspect_overrides({wire::core::EntityKind::kPole, center});
  const auto backbone = state.BuildBackboneResult();
  if (!pole_view.has_value() || !template_view.has_value() || !override_view.has_value() || backbone.junctions.empty()) {
    return false;
  }
  if (!pole_view->meta.editable || !pole_view->meta.overrideable || !pole_view->manual_yaw_override_deg.has_value() ||
      !pole_view->orientation_override) {
    return false;
  }
  if (template_view->template_kind != wire::core::TemplateKind::kPoleType || template_view->properties.empty()) {
    return false;
  }
  bool has_manual_yaw = false;
  bool has_flip = false;
  for (const auto& entry : override_view->entries) {
    has_manual_yaw = has_manual_yaw || (entry.name == "manualYaw" && entry.active);
    has_flip = has_flip || (entry.name == "flip180" && entry.active);
  }
  if (!has_manual_yaw || !has_flip) {
    return false;
  }
  const auto junction_view = view.inspect_junction(backbone.junctions.front().node_id);
  if (!junction_view.has_value() || junction_view->meta.role != wire::core::EntityRoleKind::kDerived ||
      junction_view->incidents.size() < 3) {
    return false;
  }
  return has_entity_link(pole_view->links, wire::core::EntityKind::kTemplate, pole_type_id) &&
         has_entity_link(pole_view->links, wire::core::EntityKind::kOverride, center);
}

bool test_pole_orientation_override_roundtrip_returns_to_auto() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {8.0, 0.0, 0.0}, {16.0, 0.0, 0.0}};
  req.interval_m = 8.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto gen = state.GenerateFromBackboneSpec(req);
  if (!gen.ok || gen.value.generated_pole_ids.size() < 2) {
    return false;
  }
  const ObjectId pole_id = gen.value.generated_pole_ids[1];
  const auto before = state.view().inspect_pole(pole_id);
  if (!before.has_value() || !before->has_final_yaw) {
    return false;
  }
  const double auto_yaw_deg = before->automatic_yaw_deg;
  if (!state.SetPoleManualYawOverride(pole_id, 23.0).ok || !state.SetPoleFlip180(pole_id, true).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto during = state.view().inspect_pole(pole_id);
  const auto override_view = state.view().inspect_overrides({wire::core::EntityKind::kPole, pole_id});
  const auto during_trace = state.view().collect_decision_trace({wire::core::EntityKind::kPole, pole_id});
  if (!during.has_value() || !override_view.has_value() || !during->orientation_override ||
      !during->manual_yaw_override_deg.has_value() || !during->flip_180_override.value_or(false)) {
    return false;
  }
  bool has_override_trace = false;
  for (const auto& entry : during_trace) {
    has_override_trace =
        has_override_trace || (entry.topic == wire::core::DecisionTraceTopic::kOverrideResolution);
  }
  if (!state.ClearPoleOrientationOverride(pole_id).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto after = state.view().inspect_pole(pole_id);
  return after.has_value() && !after->orientation_override && !after->manual_yaw_override_deg.has_value() &&
         !after->flip_180_override.has_value() &&
         almost_equal(after->final_yaw_deg, auto_yaw_deg, 1e-6) && has_override_trace;
}

bool test_span_socket_override_roundtrip_returns_to_auto() {
  CoreState state;
  ObjectId span = wire::core::kInvalidObjectId;
  if (!build_attachment_test_span(state, &span)) {
    return false;
  }
  const auto template_id =
      find_attachment_template_by_mode(state, wire::core::AttachmentLineInteractionMode::kPassThrough);
  if (template_id == wire::core::kInvalidAttachmentTemplateId) {
    return false;
  }
  const auto add_attachment = state.AddAttachment(span, 0.0, wire::core::AttachmentKind::kGeneric, 0.2, template_id);
  if (!add_attachment.ok) {
    return false;
  }
  wire::core::Span* span_edit = wire::core::CoreStateTestHook::edit_state(state).spans.find(span);
  if (span_edit == nullptr) {
    return false;
  }
  span_edit->endpoint_attachment_a_id = add_attachment.value;
  (void)state.Commit().recalc_stats;
  const auto* auto_layout = state.view().support_layout_projection(span).layout;
  if (auto_layout == nullptr || auto_layout->start.resolved_socket_id != 1) {
    return false;
  }
  const wire::core::Vec3d auto_endpoint = auto_layout->start.endpoint_world;
  if (!state.SetSpanEndpointSocketOverride(span, true, 0).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto* overridden_layout = state.view().support_layout_projection(span).layout;
  if (overridden_layout == nullptr || overridden_layout->start.resolved_socket_id != 0 ||
      almost_equal(overridden_layout->start.endpoint_world, auto_endpoint, 1e-6)) {
    return false;
  }
  if (!state.ClearSpanEndpointSocketOverride(span, true).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto* restored_layout = state.view().support_layout_projection(span).layout;
  return restored_layout != nullptr && restored_layout->start.resolved_socket_id == 1 &&
         almost_equal(restored_layout->start.endpoint_world, auto_endpoint, 1e-6);
}

bool test_branch_down_offset_override_roundtrip_returns_to_policy_value() {
  CoreState state;
  const ObjectId pole_a =
      state.AddPole(wire::core::Transformd{{0.0, 0.0, 0.0}}, 10.0, "A").value;
  const ObjectId pole_b =
      state.AddPole(wire::core::Transformd{{12.0, 0.0, 0.0}}, 10.0, "B").value;
  const ObjectId port_a = state.AddPort(pole_a, {0.0, 0.85, 4.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId port_b = state.AddPort(pole_b, {12.0, 0.0, 4.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(port_a, port_b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;

  wire::core::Port* edit_port_a = wire::core::CoreStateTestHook::edit_state(state).ports.find(port_a);
  wire::core::Port* edit_port_b = wire::core::CoreStateTestHook::edit_state(state).ports.find(port_b);
  wire::core::Span* edit_span = wire::core::CoreStateTestHook::edit_state(state).spans.find(span);
  if (edit_port_a == nullptr || edit_port_b == nullptr || edit_span == nullptr) {
    return false;
  }
  edit_port_a->category = wire::core::ConnectionCategory::kCommunication;
  edit_port_a->generated_by_rule = true;
  edit_port_a->placement_source = wire::core::PortPlacementSourceKind::kBranchSupport;
  edit_port_a->template_side = wire::core::SlotSide::kRight;
  edit_port_a->placement_context = wire::core::ConnectionContext::kBranchAdd;
  edit_port_b->category = wire::core::ConnectionCategory::kCommunication;
  edit_port_b->generated_by_rule = true;
  edit_port_b->placement_source = wire::core::PortPlacementSourceKind::kGenerated;
  edit_span->placement_context = wire::core::ConnectionContext::kBranchAdd;

  (void)state.Commit().recalc_stats;
  const auto* auto_layout = state.view().support_layout_projection(span).layout;
  if (auto_layout == nullptr || auto_layout->start.branch_down_offset_m <= 0.0) {
    return false;
  }
  const double auto_down = auto_layout->start.branch_down_offset_m;
  if (!state.SetSpanBranchDownOffsetOverride(span, auto_down + 0.75).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto* overridden_layout = state.view().support_layout_projection(span).layout;
  const auto override_view = state.view().inspect_overrides({wire::core::EntityKind::kSpan, span});
  if (overridden_layout == nullptr || !override_view.has_value() ||
      !almost_equal(overridden_layout->start.branch_down_offset_m, auto_down + 0.75, 1e-9) ||
      !almost_equal(overridden_layout->start.automatic_branch_down_offset_m, auto_down, 1e-9)) {
    return false;
  }
  if (!state.ClearSpanBranchDownOffsetOverride(span).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
  const auto* restored_layout = state.view().support_layout_projection(span).layout;
  if (restored_layout == nullptr || !almost_equal(restored_layout->start.branch_down_offset_m, auto_down, 1e-9)) {
    return false;
  }
  return true;
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

bool test_span_curve_support_layout_height_difference_uses_composite_segments() {
  CoreState state;
  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.curve_samples = 41;
  settings.sag_enabled = true;
  settings.sag_factor = 0.03;
  (void)state.UpdateGeometrySettings(settings, true);

  const ObjectId pole_a =
      state.AddPole(wire::core::Transformd{{0.0, 0.0, 0.0}}, 12.0, "A").value;
  const ObjectId pole_b =
      state.AddPole(wire::core::Transformd{{20.0, 0.0, 0.0}}, 12.0, "B").value;
  const ObjectId port_a = state.AddPort(pole_a, {0.0, 0.0, 11.0}, PortKind::kCommunication,
                                        PortLayer::kCommunication)
                              .value;
  const ObjectId port_b = state.AddPort(pole_b, {20.0, 0.0, 4.0}, PortKind::kCommunication,
                                        PortLayer::kCommunication)
                              .value;
  const ObjectId span = state.AddSpan(port_a, port_b, SpanKind::kDistribution,
                                      SpanLayer::kCommunication)
                            .value;

  const auto commit = state.Commit();
  if (!commit.validation.ok()) {
    return false;
  }
  const auto* support_layout = state.view().support_layout_projection(span).layout;
  const auto* curve = state.find_curve_cache(span);
  if (support_layout == nullptr || curve == nullptr || curve->detail.sample_points.size() < 5 ||
      curve->detail.arc_length_table.empty()) {
    return false;
  }
  if (support_layout->detail_curve_profile_hint != wire::core::CurveProfileHint::kCompositeHeightTransition) {
    return false;
  }
  if (curve->detail.SegmentCount() <= 1) {
    return false;
  }
  const double early_z = wire::core::HeightAlongWorldUp(curve->detail.EvaluatePosition(0.18));
  const double late_z = wire::core::HeightAlongWorldUp(curve->detail.EvaluatePosition(0.82));
  return curve->detail.quality.shape_policy == wire::core::CurveShapePolicyKind::kSmoothPass &&
         curve->detail.quality.adopted_continuity == wire::core::DetailCurveContinuityMode::kG2 &&
         almost_equal(curve->detail.EvaluatePosition(0.0), support_layout->start.endpoint_world, 1e-9) &&
         almost_equal(curve->detail.EvaluatePosition(1.0), support_layout->end.endpoint_world, 1e-9) &&
         early_z > late_z;
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

bool test_render_cache_bakes_arc_length_attributes() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 4.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {14.0, 0.0, 4.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;

  wire::core::GeometrySettings settings{};
  settings.curve_samples = 19;
  settings.sag_enabled = true;
  settings.sag_factor = 0.05;
  (void)state.UpdateGeometrySettings(settings, true);
  (void)state.Commit().recalc_stats;

  const auto* curve = state.find_curve_cache(span);
  const auto* render = state.view().find_span_render_cache(span);
  return curve != nullptr && render != nullptr &&
         render->arc_length_m_by_point.size() == curve->points.size() &&
         render->arc_length_normalized_by_point.size() == curve->points.size() &&
         render->segment_length_m.size() + 1 == curve->points.size() &&
         !render->arc_length_m_by_point.empty() &&
         almost_equal(render->arc_length_m_by_point.front(), 0.0, 1e-9) &&
         almost_equal(render->arc_length_normalized_by_point.front(), 0.0, 1e-9) &&
         almost_equal(render->arc_length_normalized_by_point.back(), 1.0, 1e-6);
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

bool test_hierarchical_variation_same_flow_keeps_shared_bias_but_not_identical() {
  CoreState state;
  wire::core::GeometrySettings geometry = state.view().geometry_settings();
  geometry.sag_enabled = true;
  geometry.sag_factor = 0.06;
  (void)state.UpdateGeometrySettings(geometry, true);

  wire::core::VariationSettings variation = state.view().variation_settings();
  variation.enabled = true;
  variation.global_seed = 42;
  variation.sag_variation_scale = 0.35;
  variation.branch_down_offset_variation_scale = 0.08;
  const auto update = state.UpdateVariationSettings(variation, true);
  if (!update.ok) {
    return false;
  }

  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  BackbonePathGenerateOptions options{};
  options.road.polyline = {{0.0, 0.0, 0.0}, {14.0, 0.5, 0.0}, {28.0, 0.8, 0.0}, {42.0, 1.2, 0.0}};
  options.interval = 1000.0;
  options.pole_type_id = type_ids.front();
  options.bundle_template_id = wire::core::BundleKind::kLowVoltage;
  const auto generated = generate_from_backbone_options(state, options);
  if (!generated.ok || generated.value.span_ids.size() < 2) {
    return false;
  }
  (void)state.Commit();

  const auto* first_curve = state.find_curve_cache(generated.value.span_ids[0]);
  const auto* second_curve = state.find_curve_cache(generated.value.span_ids[1]);
  if (first_curve == nullptr || second_curve == nullptr) {
    return false;
  }
  const auto& first = first_curve->detail.quality.sag_variation;
  const auto& second = second_curve->detail.quality.sag_variation;
  return first.flow_key != 0 && first.flow_key == second.flow_key && almost_equal(first.flow_bias, second.flow_bias, 1e-12) &&
         std::abs(first.final_value - second.final_value) > 1e-6;
}

bool test_hierarchical_variation_fixed_seed_is_reproducible() {
  auto build_state = []() {
    CoreState state;
    wire::core::GeometrySettings geometry = state.view().geometry_settings();
    geometry.sag_enabled = true;
    geometry.sag_factor = 0.05;
    (void)state.UpdateGeometrySettings(geometry, true);
    wire::core::VariationSettings variation = state.view().variation_settings();
    variation.enabled = true;
    variation.global_seed = 99;
    variation.sag_variation_scale = 0.28;
    (void)state.UpdateVariationSettings(variation, true);
    return state;
  };

  auto sample_value = [&](CoreState& state) -> double {
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    BackbonePathGenerateOptions options{};
    options.road.polyline = {{0.0, 0.0, 0.0}, {16.0, 0.0, 0.0}, {32.0, 0.0, 0.0}};
    options.interval = 1000.0;
    options.pole_type_id = type_ids.front();
    const auto generated = generate_from_backbone_options(state, options);
    if (!generated.ok || generated.value.span_ids.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    (void)state.Commit();
    const auto* curve = state.find_curve_cache(generated.value.span_ids.front());
    return (curve == nullptr) ? std::numeric_limits<double>::quiet_NaN() : curve->detail.quality.sag_variation.final_value;
  };

  CoreState state_a = build_state();
  CoreState state_b = build_state();
  const double a = sample_value(state_a);
  const double b = sample_value(state_b);
  return std::isfinite(a) && std::isfinite(b) && almost_equal(a, b, 1e-12);
}

bool test_hierarchical_variation_changes_with_seed() {
  auto build_state = [](std::uint64_t seed) {
    CoreState state;
    wire::core::GeometrySettings geometry = state.view().geometry_settings();
    geometry.sag_enabled = true;
    geometry.sag_factor = 0.05;
    (void)state.UpdateGeometrySettings(geometry, true);
    wire::core::VariationSettings variation = state.view().variation_settings();
    variation.enabled = true;
    variation.global_seed = seed;
    variation.sag_variation_scale = 0.28;
    (void)state.UpdateVariationSettings(variation, true);
    return state;
  };

  auto sample_value = [&](CoreState& state) -> double {
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    BackbonePathGenerateOptions options{};
    options.road.polyline = {{0.0, 0.0, 0.0}, {16.0, 0.0, 0.0}, {32.0, 0.0, 0.0}};
    options.interval = 1000.0;
    options.pole_type_id = type_ids.front();
    const auto generated = generate_from_backbone_options(state, options);
    if (!generated.ok || generated.value.span_ids.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    (void)state.Commit();
    const auto* curve = state.find_curve_cache(generated.value.span_ids.front());
    return (curve == nullptr) ? std::numeric_limits<double>::quiet_NaN() : curve->detail.quality.sag_variation.final_value;
  };

  CoreState state_a = build_state(11);
  CoreState state_b = build_state(12);
  const double a = sample_value(state_a);
  const double b = sample_value(state_b);
  return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) > 1e-6;
}

void register_geometry_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C13_Phase4_Curve_LineDeterministic", "Line mode curve cache is deterministic", "Exact", false, test_curve_cache_line_mode_is_deterministic);
  test_registry::AddTest(tests, "C14_Phase4_Curve_SagBasic", "Sag mode lowers midpoint along world up and keeps endpoints", "Invariant", false, test_sag_mode_changes_midpoint_and_keeps_endpoints);
  test_registry::AddTest(tests, "C15_Phase4_DirtyVersion_LocalGeometryBounds", "Geometry dirty propagates to bounds/render locally", "Exact", false, test_geometry_bounds_version_follow_and_locality);
  test_registry::AddTest(tests, "C16_Phase4_Bounds_Generated", "Bounds cache is generated and valid", "Invariant", false, test_bounds_cache_generated_and_valid);
  test_registry::AddTest(tests, "C17_Phase4_Bounds_FollowGeometry", "Bounds follow geometry setting change along world up", "Invariant", false, test_bounds_follow_geometry_change);
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
  test_registry::AddTest(tests, "C278_SpanCurve_SupportLayoutHeightDifference_UsesCompositeSegments",
                         "Recalc consumes authoritative support-layout curve inputs so large height-difference spans materialize as composite curves",
                         "Invariant", false, test_span_curve_support_layout_height_difference_uses_composite_segments);
  test_registry::AddTest(tests, "C132_RenderCurve_DistanceAttributesBaked",
                         "Render cache bakes arc-length attributes for GPU-side length-driven effects",
                         "Invariant", false, test_render_cache_bakes_arc_length_attributes);
  test_registry::AddTest(tests, "C151_Variation_WorldspaceContinuous",
                         "Worldspace variation changes continuously across nearby positions",
                         "Invariant", false, test_hierarchical_variation_worldspace_is_continuous);
  test_registry::AddTest(tests, "C152_Variation_SameFlowSharedBiasWithLocalDifference",
                         "Same flow shares a common bias while adjacent spans keep pole/local differences",
                         "Invariant", false, test_hierarchical_variation_same_flow_keeps_shared_bias_but_not_identical);
  test_registry::AddTest(tests, "C153_Variation_GlobalSeedReproducible",
                         "Fixed global seed reproduces the same derived variation values",
                         "Invariant", false, test_hierarchical_variation_fixed_seed_is_reproducible);
  test_registry::AddTest(tests, "C154_Variation_SeedChangesOutput",
                         "Changing the global seed changes derived variation output without changing rules",
                         "Invariant", false, test_hierarchical_variation_changes_with_seed);
  test_registry::AddTest(tests, "C137_Attachment_DisplayOffset_IsEntityOnly",
                         "Attachment display offset stays in entity/display state and does not perturb detail-curve endpoints",
                         "Invariant", false, test_attachment_display_offset_does_not_change_detail_curve_endpoints);
  test_registry::AddTest(tests, "C156_Attachment_PassThrough_KeepsOuterCurve",
                         "PassThrough attachment keeps the outer curve visible and does not inject replacement geometry",
                         "Invariant", false, test_attachment_pass_through_keeps_outer_curve_visible);
  test_registry::AddTest(tests, "C157_Attachment_HideSegment_MasksOuterCurve",
                         "HideSegment attachment masks the covered outer-curve interval without replacement geometry",
                         "Invariant", false, test_attachment_hide_segment_masks_curve_interval);
  test_registry::AddTest(tests, "C158_Attachment_ReplaceWithInternalPath_ReplacesInterval",
                         "ReplaceWithInternalPath hides the covered outer interval and emits replacement path geometry",
                         "Invariant", false, test_attachment_replace_with_internal_path_replaces_interval);
  test_registry::AddTest(tests, "C306_DetailCurve_CurveOffsetStraightSupplemental_Wobble",
                         "Curve-offset straight supplemental paths can add deterministic gentle wobble without becoming structural support wires",
                         "Invariant", false, test_curve_offset_straight_cable_supplemental_uses_deterministic_wobble);
  test_registry::AddTest(tests, "C307_ContextProfile_InfluencesDetailWobble",
                         "Context profile can change detail wobble appearance while preserving deterministic regeneration",
                         "Invariant", false, test_context_profile_influences_detail_wobble_without_breaking_determinism);
  test_registry::AddTest(tests, "C159_Attachment_SocketEndpoint_OverridesCurveEndpoint",
                         "Attachment socket endpoint can replace a span endpoint so the curve meets the socket without a gap",
                         "Invariant", false, test_attachment_socket_endpoint_can_override_curve_endpoint);
  test_registry::AddTest(tests, "C160_Attachment_Runtime_IgnoresNames",
                         "Attachment runtime behavior is driven by explicit template mode, not template/object naming",
                         "Invariant", false, test_attachment_behavior_is_not_name_driven);
  test_registry::AddTest(tests, "C167_Inspection_PoleTemplateOverrideJunction_Surface",
                         "Inspection surface exposes pole/template/override/junction views through concept-level readonly access",
                         "Invariant", false, test_inspection_pole_template_override_and_junction_surfaces_are_visible);
  test_registry::AddTest(tests, "C168_Override_PoleOrientation_Roundtrip",
                         "Pole orientation override can be applied and cleared back to automatic resolution",
                         "Invariant", false, test_pole_orientation_override_roundtrip_returns_to_auto);
  test_registry::AddTest(tests, "C169_Override_SpanSocket_Roundtrip",
                         "Span endpoint socket override can be applied and cleared back to automatic support-layout endpoint selection",
                         "Invariant", false, test_span_socket_override_roundtrip_returns_to_auto);
  test_registry::AddTest(tests, "C170_Override_BranchDownOffset_Roundtrip",
                         "Branch down offset override can be applied and cleared back to policy-derived support layout",
                         "Invariant", false, test_branch_down_offset_override_roundtrip_returns_to_policy_value);
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
