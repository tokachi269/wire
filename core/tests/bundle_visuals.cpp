#include "registry.hpp"
#include "helpers.hpp"

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

bool test_pole_tilt_reprojects_auto_ports_and_updates_visual_cache() {
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
  const auto* before = state.view().edit_state().ports.find(add.value.port_a_id);
  if (before == nullptr || before->position_mode != wire::core::PortPositionMode::kAuto) {
    return false;
  }
  const wire::core::Vec3d before_pos = before->world_position;

  const auto tilt = state.SetPoleTilt(pole_a, 10.0, 0.0);
  if (!tilt.ok) {
    return false;
  }
  const auto* after = state.view().edit_state().ports.find(add.value.port_a_id);
  if (after == nullptr || after->position_mode != wire::core::PortPositionMode::kAuto) {
    return false;
  }
  const double moved = std::sqrt(std::pow(after->world_position.x - before_pos.x, 2.0) +
                                 std::pow(after->world_position.y - before_pos.y, 2.0) +
                                 std::pow(after->world_position.z - before_pos.z, 2.0));
  if (moved <= 1e-5) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  options.run_validate = false;
  (void)state.Commit(options);
  const auto* visual = state.find_span_visual_cache(add.value.span_id);
  if (visual == nullptr) {
    return false;
  }
  for (const auto& part : visual->parts) {
    if (part.kind == wire::core::VisualPartKind::kInsulator) {
      return true;
    }
  }
  return false;
}

bool test_span_reference_length_keeps_sag_depth_stable_across_tilt() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::GeometrySettings gs = state.view().geometry_settings();
  gs.sag_enabled = true;
  gs.sag_factor = 0.08;
  if (!state.UpdateGeometrySettings(gs, true).ok) {
    return false;
  }

  wire::core::Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b_tf{};
  b_tf.position = {18.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 11.0, "A").value;
  const ObjectId pole_b = state.AddPole(b_tf, 11.0, "B").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }
  const auto add = add_connection_by_category(state, pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage);
  if (!add.ok) {
    return false;
  }

  wire::core::CommitOptions options{};
  options.run_recalc = true;
  options.run_validate = false;
  (void)state.Commit(options);

  auto sag_depth = [&](ObjectId span_id) -> double {
    const auto* span = state.view().edit_state().spans.find(span_id);
    const auto* pa = (span == nullptr) ? nullptr : state.view().edit_state().ports.find(span->port_a_id);
    const auto* pb = (span == nullptr) ? nullptr : state.view().edit_state().ports.find(span->port_b_id);
    const auto* curve = state.find_curve_cache(span_id);
    if (span == nullptr || pa == nullptr || pb == nullptr || curve == nullptr || curve->points.size() < 3) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const wire::core::Vec3d line_mid{
        (pa->world_position.x + pb->world_position.x) * 0.5,
        (pa->world_position.y + pb->world_position.y) * 0.5,
        (pa->world_position.z + pb->world_position.z) * 0.5,
    };
    const wire::core::Vec3d curve_mid = curve->points[curve->points.size() / 2];
    return line_mid.z - curve_mid.z;
  };

  const double depth_before = sag_depth(add.value.span_id);
  if (!std::isfinite(depth_before)) {
    return false;
  }
  if (!state.SetPoleTilt(pole_a, 14.0, 4.0).ok) {
    return false;
  }
  (void)state.Commit(options);
  const double depth_after = sag_depth(add.value.span_id);
  if (!std::isfinite(depth_after)) {
    return false;
  }
  return std::abs(depth_before - depth_after) <= 0.15;
}

bool test_center_slot_ports_are_offset_from_pole_centerline() {
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

bool test_visual_insulators_only_for_electric_lines() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd b_tf{};
  b_tf.position = {10.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B").value;
  wire::core::Transformd c_tf{};
  c_tf.position = {0.0, 10.0, 0.0};
  const ObjectId pole_c = state.AddPole(c_tf, 10.0, "C").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok ||
      !state.ApplyPoleType(pole_c, type_ids.front()).ok) {
    return false;
  }

  const auto lv = add_connection_by_category(state, pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage);
  const auto comm = add_connection_by_category(state, pole_a, pole_c, wire::core::ConnectionCategory::kCommunication);
  if (!lv.ok || !comm.ok) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  options.run_validate = false;
  (void)state.Commit(options);

  auto has_insulator = [&](ObjectId span_id) -> bool {
    const auto* visual = state.find_span_visual_cache(span_id);
    if (visual == nullptr) {
      return false;
    }
    for (const auto& part : visual->parts) {
      if (part.kind == wire::core::VisualPartKind::kInsulator) {
        return true;
      }
    }
    return false;
  };

  return has_insulator(lv.value.span_id) && !has_insulator(comm.value.span_id);
}

bool test_cable_template_outer_diameter_updates_render_radius() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::Transformd a_tf{};
  wire::core::Transformd b_tf{};
  b_tf.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }
  const auto add = add_connection_by_category(state, pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage);
  if (!add.ok) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  options.run_validate = false;
  (void)state.Commit(options);

  const auto* before_render = state.view().find_span_render_cache(add.value.span_id);
  const auto* span = state.view().edit_state().spans.find(add.value.span_id);
  const auto* bundle = (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
  if (before_render == nullptr || bundle == nullptr) {
    return false;
  }
  const double before_radius = before_render->wire_radius_m;
  const auto tpl_it = state.view().bundle_templates().find(bundle->bundle_template_id);
  if (tpl_it == state.view().bundle_templates().end()) {
    return false;
  }
  const auto cable_it = state.view().cable_templates().find(tpl_it->second.cable_template_id);
  if (cable_it == state.view().cable_templates().end()) {
    return false;
  }
  wire::core::CableTemplate cable = cable_it->second;
  cable.outer_diameter_m *= 1.5;
  if (!state.UpdateCableTemplate(cable).ok) {
    return false;
  }
  (void)state.Commit(options);
  const auto* after_render = state.view().find_span_render_cache(add.value.span_id);
  return after_render != nullptr && after_render->wire_radius_m > before_radius;
}

bool test_cable_template_requires_insulator_toggles_visual_parts() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::Transformd a_tf{};
  wire::core::Transformd b_tf{};
  b_tf.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok || !state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }
  const auto add = add_connection_by_category(state, pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage);
  if (!add.ok) {
    return false;
  }
  wire::core::CommitOptions options{};
  options.run_recalc = true;
  options.run_validate = false;
  (void)state.Commit(options);

  const auto* span = state.view().edit_state().spans.find(add.value.span_id);
  const auto* bundle = (span == nullptr) ? nullptr : state.view().edit_state().bundles.find(span->bundle_id);
  if (bundle == nullptr) {
    return false;
  }
  const auto tpl_it = state.view().bundle_templates().find(bundle->bundle_template_id);
  if (tpl_it == state.view().bundle_templates().end()) {
    return false;
  }
  const auto cable_it = state.view().cable_templates().find(tpl_it->second.cable_template_id);
  if (cable_it == state.view().cable_templates().end()) {
    return false;
  }
  auto has_insulator = [&](ObjectId span_id) -> bool {
    const auto* visual = state.find_span_visual_cache(span_id);
    if (visual == nullptr) {
      return false;
    }
    for (const auto& part : visual->parts) {
      if (part.kind == wire::core::VisualPartKind::kInsulator) {
        return true;
      }
    }
    return false;
  };
  if (!has_insulator(add.value.span_id)) {
    return false;
  }
  wire::core::CableTemplate cable = cable_it->second;
  cable.requires_insulator = false;
  if (!state.UpdateCableTemplate(cable).ok) {
    return false;
  }
  (void)state.Commit(options);
  return !has_insulator(add.value.span_id);
}

bool test_template_type_ownership_is_separated() {
  constexpr bool bundle_has_flag = has_allow_midair_branch_member<wire::core::BundleTemplate>::value;
  constexpr bool cable_has_flag = has_allow_midair_branch_member<wire::core::CableTemplate>::value;
  constexpr bool span_has_bezier = has_bezier_control_points_member<wire::core::Span>::value;
  constexpr bool bundle_has_bezier = has_bezier_control_points_member<wire::core::Bundle>::value;
  return bundle_has_flag && !cable_has_flag && !span_has_bezier && !bundle_has_bezier;
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
  if (!state.SetPoleTilt(pole, 7.0, 3.0).ok) {
    return false;
  }
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
         almost_equal(updated_pole->world_transform.rotation_euler_deg.x, 7.0, 1e-9) &&
         almost_equal(updated_pole->world_transform.rotation_euler_deg.y, 3.0, 1e-9);
}

bool test_apply_pole_tilt_updates_explicit_selection_only() {
  CoreState state;
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  wire::core::Transformd moved{};
  moved.position = {5.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(moved, 10.0, "B").value;
  const auto apply = state.ApplyPoleTilt({pole_a}, 9.0, 4.0);
  if (!apply.ok) {
    return false;
  }
  const auto* updated_a = state.view().edit_state().poles.find(pole_a);
  const auto* updated_b = state.view().edit_state().poles.find(pole_b);
  return updated_a != nullptr && updated_b != nullptr &&
         almost_equal(updated_a->world_transform.rotation_euler_deg.x, 9.0, 1e-9) &&
         almost_equal(updated_a->world_transform.rotation_euler_deg.y, 4.0, 1e-9) &&
         almost_equal(updated_b->world_transform.rotation_euler_deg.x, 0.0, 1e-9) &&
         almost_equal(updated_b->world_transform.rotation_euler_deg.y, 0.0, 1e-9);
}

void register_bundle_visual_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C44_Phase48a_Bundle_AssignQuery", "Bundle assignment and query API works on spans", "Invariant", false, test_bundle_query_spans_by_bundle);
  test_registry::AddTest(tests, "C45_Phase48a_Bundle_InvalidReject", "Invalid bundle references are rejected", "Exact", true, test_add_span_invalid_bundle_fails);
  test_registry::AddTest(tests, "C78_Phase50_Tilt_ReprojectAndVisualFollow", "Pole tilt reprojects Auto ports and updates span visual cache through one path", "Invariant", false, test_pole_tilt_reprojects_auto_ports_and_updates_visual_cache);
  test_registry::AddTest(tests, "C79_Phase50_ReferenceLength_StableSag", "Reference length keeps sag depth visually stable across tilt", "Invariant", false, test_span_reference_length_keeps_sag_depth_stable_across_tilt);
  test_registry::AddTest(tests, "C80_Phase50_CenterOffset_NoPoleOverlap", "Center slots are offset from pole centerline by radius+clearance", "Invariant", false, test_center_slot_ports_are_offset_from_pole_centerline);
  test_registry::AddTest(tests, "C81_Phase50_Insulator_ElectricOnly", "Visual insulators are generated only for electric lines", "Invariant", false, test_visual_insulators_only_for_electric_lines);
  test_registry::AddTest(tests, "C119_CableTemplate_DiameterAffectsRender",
                         "CableTemplate diameter changes update dependent wire render radius", "Invariant", false,
                         test_cable_template_outer_diameter_updates_render_radius);
  test_registry::AddTest(tests, "C120_CableTemplate_InsulatorToggle",
                         "CableTemplate requires_insulator toggles dependent insulator visuals", "Invariant", false,
                         test_cable_template_requires_insulator_toggles_visual_parts);
  test_registry::AddTest(tests, "C121_Template_OwnershipSeparated",
                         "BundleTemplate owns branch policy while CableTemplate and entities do not own bezier inputs",
                         "Exact", true, test_template_type_ownership_is_separated);
  test_registry::AddTest(tests, "C122_CableTemplate_EditPreservesPoleTilt",
                         "Cable template edits do not overwrite pole tilt instance values", "Invariant", false,
                         test_cable_template_edit_preserves_pole_tilt_instance_value);
  test_registry::AddTest(tests, "C125_ApplyPoleTilt_SelectionOwned",
                         "ApplyPoleTilt updates only explicit pole instances and keeps tilt instance-owned",
                         "Invariant", false, test_apply_pole_tilt_updates_explicit_selection_only);
}

WIRE_REGISTER_TEST_SUITE(register_bundle_visual_tests);

} // namespace
