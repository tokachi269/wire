#include "registry.hpp"
#include "helpers.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <regex>
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
template <typename T, typename = void> struct has_allow_midair_branch_member : std::false_type {};
template <typename T>
struct has_allow_midair_branch_member<T, std::void_t<decltype(std::declval<T>().allow_midair_branch)>> : std::true_type {};

template <typename T, typename = void> struct has_bezier_control_points_member : std::false_type {};
template <typename T>
struct has_bezier_control_points_member<T, std::void_t<decltype(std::declval<T>().bezier_control_points)>> : std::true_type {};

template <typename T, typename = void> struct has_arc_length_table_member : std::false_type {};
template <typename T>
struct has_arc_length_table_member<T, std::void_t<decltype(std::declval<T>().arc_length_table)>> : std::true_type {};

bool test_bundle_query_spans_by_bundle() {
  CoreState state;
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b{};
  b.position = {8.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  const ObjectId port_a = state.AddPort(pole_a, {0.0, 0.0, 7.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId port_b = state.AddPort(pole_b, {8.0, 0.0, 7.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const auto add_bundle = state.AddBundle(2, 0.2, wire::core::BundleKind::kLowVoltage);
  if (!add_bundle.ok) {
    return false;
  }
  const auto add_span = state.AddSpan(port_a, port_b, SpanKind::kDistribution, SpanLayer::kLowVoltage, add_bundle.value);
  if (!add_span.ok) {
    return false;
  }

  const auto grouped_spans = state.GetSpansByBundle(add_bundle.value);
  if (grouped_spans.size() != 1 || grouped_spans[0] != add_span.value) {
    return false;
  }
  const auto& relation_index = state.view().relation_index();
  const auto bundle_it = relation_index.spans_by_bundle.find(add_bundle.value);
  if (bundle_it == relation_index.spans_by_bundle.end() ||
      std::find(bundle_it->second.begin(), bundle_it->second.end(), add_span.value) == bundle_it->second.end()) {
    return false;
  }
  const auto pole_port_it = relation_index.ports_by_pole.find(pole_a);
  if (pole_port_it == relation_index.ports_by_pole.end() ||
      std::find(pole_port_it->second.begin(), pole_port_it->second.end(), port_a) == pole_port_it->second.end()) {
    return false;
  }
  return validate_now(state).ok();
}

bool test_add_span_invalid_bundle_fails() {
  CoreState state;
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b{};
  b.position = {6.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  const ObjectId port_a = state.AddPort(pole_a, {0.0, 0.0, 7.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId port_b = state.AddPort(pole_b, {6.0, 0.0, 7.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const auto bad = state.AddSpan(port_a, port_b, SpanKind::kDistribution, SpanLayer::kLowVoltage, 999999);
  return !bad.ok && regex_contains(bad.error, "bundle does not exist");
}

bool test_center_band_ports_are_offset_from_pole_centerline() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  if (!state.ApplyPoleType(pole, type_ids.front()).ok) {
    return false;
  }
  const auto detail = state.GetPoleDetail(pole);
  if (detail.pole == nullptr) {
    return false;
  }
  bool checked_center = false;
  for (const auto* port : detail.owned_ports) {
    if (port == nullptr || port->template_side != wire::core::SlotSide::kCenter) {
      continue;
    }
    checked_center = true;
    const wire::core::Vec3d local = to_local_on_pole_test(*detail.pole, port->world_position);
    if (std::abs(local.y) <= 1e-6) {
      return false;
    }
  }
  return checked_center;
}

bool test_template_type_ownership_is_separated() {
  constexpr bool bundle_has_flag = has_allow_midair_branch_member<wire::core::BundleTemplate>::value;
  constexpr bool cable_has_flag = has_allow_midair_branch_member<wire::core::CableTemplate>::value;
  constexpr bool span_has_bezier = has_bezier_control_points_member<wire::core::Span>::value;
  constexpr bool bundle_has_bezier = has_bezier_control_points_member<wire::core::Bundle>::value;
  constexpr bool span_has_arc_length = has_arc_length_table_member<wire::core::Span>::value;
  constexpr bool bundle_has_arc_length = has_arc_length_table_member<wire::core::Bundle>::value;
  return bundle_has_flag && !cable_has_flag && !span_has_bezier && !bundle_has_bezier &&
         !span_has_arc_length && !bundle_has_arc_length;
}

bool test_cable_template_edit_preserves_pole_tilt_instance_value() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  if (!state.ApplyPoleType(pole, type_ids.front()).ok) {
    return false;
  }
  if (!state.SetPoleTilt(pole, 7.0).ok) {
    return false;
  }
  const auto* tilted_pole = state.view().edit_state().poles.find(pole);
  if (tilted_pole == nullptr) {
    return false;
  }
  const double tilt_before = tilted_pole->tilt_magnitude_deg;
  const double rot_x_before = tilted_pole->world_transform.rotation_euler_deg.x;
  const double rot_y_before = tilted_pole->world_transform.rotation_euler_deg.y;
  const auto& cables = state.view().cable_templates();
  if (cables.empty()) {
    return false;
  }
  wire::core::CableTemplate cable = cables.begin()->second;
  cable.outer_diameter_m += 0.005;
  if (!state.UpdateCableTemplate(cable).ok) {
    return false;
  }
  const auto* updated_pole = state.view().edit_state().poles.find(pole);
  return updated_pole != nullptr &&
         almost_equal(updated_pole->tilt_magnitude_deg, tilt_before, 1e-9) &&
         almost_equal(updated_pole->world_transform.rotation_euler_deg.x, rot_x_before, 1e-9) &&
         almost_equal(updated_pole->world_transform.rotation_euler_deg.y, rot_y_before, 1e-9);
}

bool test_apply_pole_tilt_updates_explicit_selection_only() {
  CoreState state;
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd moved{};
  moved.position = {5.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(moved, 10.0, "B").value;
  const auto apply = state.ApplyPoleTilt({pole_a}, 9.0);
  if (!apply.ok) {
    return false;
  }
  const auto* updated_a = state.view().edit_state().poles.find(pole_a);
  const auto* updated_b = state.view().edit_state().poles.find(pole_b);
  return updated_a != nullptr && updated_b != nullptr &&
         updated_a->tilt_magnitude_deg > 0.0 &&
         almost_equal(updated_b->tilt_magnitude_deg, 0.0, 1e-9) &&
         (std::abs(updated_a->world_transform.rotation_euler_deg.x) > 1e-9 ||
          std::abs(updated_a->world_transform.rotation_euler_deg.y) > 1e-9) &&
         almost_equal(updated_b->world_transform.rotation_euler_deg.x, 0.0, 1e-9) &&
         almost_equal(updated_b->world_transform.rotation_euler_deg.y, 0.0, 1e-9);
}

bool test_pole_tilt_biases_toward_incident_span_direction() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b_tf{};
  b_tf.position = {12.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }
  const auto add = add_connection_by_category(state, pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage);
  if (!add.ok) {
    return false;
  }
  if (!state.SetPoleTilt(pole_a, 12.0).ok) {
    return false;
  }
  const auto* pole = state.view().edit_state().poles.find(pole_a);
  const auto* remote_port = state.view().edit_state().ports.find(add.value.port_b_id);
  const auto pole_view = state.view().inspect_pole(pole_a);
  if (pole == nullptr || remote_port == nullptr || !pole_view.has_value() || !pole_view->has_layout_yaw) {
    return false;
  }
  wire::core::Vec3d top_world = wire::core::LocalPointToWorld(
      wire::core::BuildPoleFrame(pole->world_transform, pole_view->layout_yaw_deg), {0.0, 0.0, pole->height_m});
  wire::core::Vec3d tilt_dir = top_world - pole->world_transform.position;
  tilt_dir.z = 0.0;
  wire::core::Vec3d span_dir = remote_port->world_position - pole->world_transform.position;
  span_dir.z = 0.0;
  if (!wire::core::NormalizeXY(&tilt_dir) || !wire::core::NormalizeXY(&span_dir)) {
    return false;
  }
  return wire::core::Dot(tilt_dir, span_dir) > 0.5;
}

bool test_pole_tilt_magnitude_increases_with_pull_imbalance() {
  auto measure_center_tilt = [](const wire::core::Vec3d& left, const wire::core::Vec3d& right) -> double {
    CoreState state;
    const auto type_ids = sorted_pole_type_ids(state);
    if (type_ids.empty()) {
      return -1.0;
    }
    wire::core::Transformd a_tf{};
    a_tf.position = left;
    wire::core::Transformd b_tf{};
    b_tf.position = {0.0, 0.0, 0.0};
    wire::core::Transformd c_tf{};
    c_tf.position = right;
    const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A").value;
    const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B").value;
    const ObjectId pole_c = state.AddPole(c_tf, 10.0, "C").value;
    if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok ||
        !state.ApplyPoleType(pole_c, type_ids.front()).ok) {
      return -1.0;
    }
    if (!add_connection_by_category(state, pole_b, pole_a, wire::core::ConnectionCategory::kLowVoltage).ok ||
        !add_connection_by_category(state, pole_b, pole_c, wire::core::ConnectionCategory::kLowVoltage).ok) {
      return -1.0;
    }
    if (!state.SetPoleTilt(pole_b, 12.0).ok) {
      return -1.0;
    }
    const auto* pole = state.view().edit_state().poles.find(pole_b);
    return (pole == nullptr) ? -1.0 : pole->tilt_magnitude_deg;
  };

  const double straight_tilt = measure_center_tilt({-10.0, 0.0, 0.0}, {10.0, 0.0, 0.0});
  const double corner_tilt = measure_center_tilt({-10.0, 0.0, 0.0}, {0.0, 10.0, 0.0});
  return straight_tilt >= 0.0 && corner_tilt >= 0.0 && corner_tilt > straight_tilt + 1e-6;
}

void register_bundle_visual_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C44_Phase48a_Bundle_AssignQuery", "Bundle assignment and query API works on spans", "Invariant", false, test_bundle_query_spans_by_bundle);
  test_registry::AddTest(tests, "C45_Phase48a_Bundle_InvalidReject", "Invalid bundle references are rejected", "Exact", true, test_add_span_invalid_bundle_fails);
  test_registry::AddTest(tests, "C80_Phase50_CenterOffset_NoPoleOverlap", "Center bands are offset from pole centerline by radius+clearance", "Invariant", false, test_center_band_ports_are_offset_from_pole_centerline);
  test_registry::AddTest(tests, "C121_Template_OwnershipSeparated",
                         "BundleTemplate owns branch policy while CableTemplate and entities do not own bezier inputs",
                         "Exact", true, test_template_type_ownership_is_separated);
  test_registry::AddTest(tests, "C122_CableTemplate_EditPreservesPoleTilt",
                         "Cable template edits do not overwrite pole tilt instance values", "Invariant", false,
                         test_cable_template_edit_preserves_pole_tilt_instance_value);
  test_registry::AddTest(tests, "C125_ApplyPoleTilt_SelectionOwned",
                         "ApplyPoleTilt updates only explicit pole instances and keeps tilt instance-owned",
                         "Invariant", false, test_apply_pole_tilt_updates_explicit_selection_only);
  test_registry::AddTest(tests, "C202_Tilt_BiasesTowardIncidentSpanDirection",
                         "Pole tilt direction is random but biased toward the incident span pull direction",
                         "Invariant", false, test_pole_tilt_biases_toward_incident_span_direction);
  test_registry::AddTest(tests, "C203_Tilt_MagnitudeFollowsPullImbalance",
                         "Pole tilt magnitude is reduced for balanced spans and increases when pull is imbalanced",
                         "Invariant", false, test_pole_tilt_magnitude_increases_with_pull_imbalance);
}

WIRE_REGISTER_TEST_SUITE(register_bundle_visual_tests);

} // namespace
