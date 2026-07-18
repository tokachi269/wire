#include "registry.hpp"
#include "helpers.hpp"

#include <algorithm>
#include <utility>
#include <vector>

using namespace helpers;
using wire::core::PortKind;
using wire::core::PortLayer;

namespace {
bool test_port_position_mode_defaults_auto() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const auto add = state.AddPort(pole, {0.0, 0.0, 6.0}, PortKind::kPower, PortLayer::kLowVoltage);
  if (!add.ok) {
    return false;
  }
  const auto* port = state.view().edit_state().ports.find(add.value);
  if (port == nullptr) {
    return false;
  }
  return port->position_mode == wire::core::PortPositionMode::kAuto && !port->user_edited_position;
}

bool test_port_manual_set_and_reset_to_auto() {
  CoreState state;
  const auto fixture = make_backbone_fixture(state, {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}});
  if (!fixture.ok || fixture.value.spans.empty()) return false;
  const auto* span = state.view().spans().find(fixture.value.spans.front());
  if (span == nullptr) return false;
  const ObjectId port_id = span->port_a_id;
  const wire::core::Vec3d manual_pos{123.0, 45.0, 9.0};
  const auto manual = state.SetPortWorldPositionManual(port_id, manual_pos);
  if (!manual.ok) {
    return false;
  }
  const auto* manual_port = state.view().edit_state().ports.find(port_id);
  if (manual_port == nullptr) {
    return false;
  }
  if (manual_port->position_mode != wire::core::PortPositionMode::kManual ||
      manual_port->placement_source != wire::core::PortPlacementSourceKind::kManualEdit ||
      !manual_port->user_edited_position || !almost_equal(manual_port->world_position, manual_pos)) {
    return false;
  }

  const auto reset = state.ResetPortPositionToAuto(port_id);
  if (!reset.ok) {
    return false;
  }
  const auto* reset_port = state.view().edit_state().ports.find(port_id);
  if (reset_port == nullptr) {
    return false;
  }
  return reset_port->position_mode == wire::core::PortPositionMode::kAuto && !reset_port->user_edited_position &&
         reset_port->placement_source != wire::core::PortPlacementSourceKind::kManualEdit &&
         !almost_equal(reset_port->world_position, manual_pos);
}

bool test_manual_port_not_overwritten_by_auto_relayout() {
  CoreState state;
  const auto fixture = make_backbone_fixture(state, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}});
  if (!fixture.ok || fixture.value.spans.empty() || fixture.value.poles.empty()) return false;
  const ObjectId pole_a = fixture.value.poles.front();
  const auto* span = state.view().spans().find(fixture.value.spans.front());
  if (span == nullptr) return false;
  const ObjectId manual_port_id = span->port_a_id;
  const wire::core::Vec3d manual_pos{77.0, -12.0, 8.0};
  if (!state.SetPortWorldPositionManual(manual_port_id, manual_pos).ok) {
    return false;
  }
  if (!state.SetPoleFlip180(pole_a, true).ok) {
    return false;
  }
  const auto* port_after = state.view().edit_state().ports.find(manual_port_id);
  if (port_after == nullptr) {
    return false;
  }
  return port_after->position_mode == wire::core::PortPositionMode::kManual &&
         almost_equal(port_after->world_position, manual_pos);
}

bool test_move_pole_reprojects_auto_ports_and_preserves_manual_ports() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::Transformd tf{};
  tf.position = {0.0, 0.0, 0.0};
  const ObjectId pole_id = state.AddPole(tf, 10.0, "P").value;
  if (!state.ApplyPoleType(pole_id, type_ids.front()).ok) {
    return false;
  }

  ObjectId auto_port_id = wire::core::kInvalidObjectId;
  ObjectId manual_port_id = wire::core::kInvalidObjectId;
  wire::core::Vec3d auto_before{};
  wire::core::Vec3d manual_before{};
  for (const auto& port : state.view().edit_state().ports.items()) {
    if (port.owner_pole_id != pole_id) {
      continue;
    }
    if (port.template_side == wire::core::SlotSide::kLeft) {
      auto_port_id = port.id;
      auto_before = port.world_position;
    } else if (port.template_side == wire::core::SlotSide::kRight) {
      manual_port_id = port.id;
      manual_before = port.world_position;
    }
  }
  if (auto_port_id == wire::core::kInvalidObjectId || manual_port_id == wire::core::kInvalidObjectId) {
    return false;
  }
  if (!state.SetPortWorldPositionManual(manual_port_id, manual_before).ok) {
    return false;
  }

  wire::core::Transformd moved{};
  moved.position = {3.0, 4.0, 0.0};
  moved.rotation_euler_deg.z = 90.0;
  if (!state.MovePole(pole_id, moved).ok) {
    return false;
  }

  const auto* auto_after = state.view().edit_state().ports.find(auto_port_id);
  const auto* manual_after = state.view().edit_state().ports.find(manual_port_id);
  if (auto_after == nullptr || manual_after == nullptr) {
    return false;
  }
  const bool auto_moved = !almost_equal(auto_after->world_position, auto_before, 1e-6);
  const bool manual_kept = almost_equal(manual_after->world_position, manual_before, 1e-6);
  return auto_moved && manual_kept && manual_after->position_mode == wire::core::PortPositionMode::kManual;
}

bool test_generate_from_guide_keeps_manual_boundaries_stable() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto first = state.GenerateFromBackboneSpec(req);
  if (!first.ok || first.value.generated_pole_ids.size() < 2) {
    return false;
  }

  auto find_pole_at = [&](const wire::core::Vec3d& pos) -> const wire::core::Pole* {
    const wire::core::Pole* found = nullptr;
    for (const auto& pole : state.view().edit_state().poles.items()) {
      if (almost_equal(pole.world_transform.position, pos)) {
        if (found == nullptr || pole.placement_mode == wire::core::PlacementMode::kManual) {
          found = &pole;
        }
      }
    }
    return found;
  };

  const auto* start_before = find_pole_at({0.0, 0.0, 0.0});
  const auto* mid_before = find_pole_at({20.0, 0.0, 0.0});
  if (start_before == nullptr || mid_before == nullptr) {
    return false;
  }
  const ObjectId start_id = start_before->id;
  const ObjectId mid_id = mid_before->id;
  const auto start_pos_before = start_before->world_transform.position;
  const auto mid_pos_before = mid_before->world_transform.position;
  if (!state.SetPolePlacementMode(start_id, wire::core::PlacementMode::kManual).ok ||
      !state.SetPolePlacementMode(mid_id, wire::core::PlacementMode::kManual).ok) {
    return false;
  }
  start_before = state.view().edit_state().poles.find(start_id);
  mid_before = state.view().edit_state().poles.find(mid_id);
  if (start_before == nullptr || mid_before == nullptr ||
      start_before->placement_mode != wire::core::PlacementMode::kManual ||
      mid_before->placement_mode != wire::core::PlacementMode::kManual) {
    return false;
  }

  req.path.polyline = {{20.0, 0.0, 0.0}, {40.0, 0.0, 0.0}};
  wire::core::BackboneInputSpec::NodeSpec mid_spec{};
  mid_spec.point_index = 0;
  mid_spec.support_kind = wire::core::SupportKind::kPole;
  mid_spec.node_id = mid_id;
  req.path.node_specs = {mid_spec};
  const auto second = state.GenerateFromBackboneSpec(req);
  if (!second.ok) {
    return false;
  }
  const auto* start_after = state.view().edit_state().poles.find(start_id);
  const auto* mid_after = state.view().edit_state().poles.find(mid_id);
  if (start_after == nullptr || mid_after == nullptr) {
    return false;
  }
  return almost_equal(start_after->world_transform.position, start_pos_before) &&
         almost_equal(mid_after->world_transform.position, mid_pos_before) &&
         start_after->placement_mode == wire::core::PlacementMode::kManual &&
         mid_after->placement_mode == wire::core::PlacementMode::kManual;
}

bool test_generate_from_guide_vertices_are_not_forced_manual_by_default() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  auto find_pole_at = [&](const wire::core::Vec3d& pos) -> const wire::core::Pole* {
    for (const auto& pole : state.view().edit_state().poles.items()) {
      if (almost_equal(pole.world_transform.position, pos)) {
        return &pole;
      }
    }
    return nullptr;
  };

  const auto* start = find_pole_at({0.0, 0.0, 0.0});
  const auto* mid = find_pole_at({10.0, 0.0, 0.0});
  const auto* end = find_pole_at({20.0, 0.0, 0.0});
  if (start == nullptr || mid == nullptr || end == nullptr) {
    return false;
  }
  return start->placement_mode == wire::core::PlacementMode::kAuto &&
         end->placement_mode == wire::core::PlacementMode::kAuto &&
         mid->placement_mode == wire::core::PlacementMode::kAuto;
}

bool test_generate_from_guide_pin_vertices_option() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  req.interval_m = 10.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
  req.pole_placement.pin_vertices = true;
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok) {
    return false;
  }

  for (const auto& pole : state.view().edit_state().poles.items()) {
    if (almost_equal(pole.world_transform.position, wire::core::Vec3d{10.0, 0.0, 0.0})) {
      return pole.placement_mode == wire::core::PlacementMode::kManual;
    }
  }
  return false;
}

bool test_set_pole_placement_mode_auto_manual_roundtrip() {
  CoreState state;
  const ObjectId pole_id = state.AddPole({}, 10.0, "P").value;
  const auto* pole = state.view().edit_state().poles.find(pole_id);
  if (pole == nullptr || pole->placement_mode != wire::core::PlacementMode::kAuto) {
    return false;
  }
  const auto to_manual = state.SetPolePlacementMode(pole_id, wire::core::PlacementMode::kManual);
  if (!to_manual.ok) {
    return false;
  }
  pole = state.view().edit_state().poles.find(pole_id);
  if (pole == nullptr || pole->placement_mode != wire::core::PlacementMode::kManual || !pole->user_edited) {
    return false;
  }
  const auto to_auto = state.SetPolePlacementMode(pole_id, wire::core::PlacementMode::kAuto);
  if (!to_auto.ok) {
    return false;
  }
  pole = state.view().edit_state().poles.find(pole_id);
  return pole != nullptr && pole->placement_mode == wire::core::PlacementMode::kAuto && !pole->user_edited;
}

bool test_backbone_edges_and_route_search() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {24.0, 0.0, 0.0}};
  req.interval_m = 6.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
  const auto generated = state.GenerateFromBackboneSpec(req);
  if (!generated.ok || generated.value.generated_pole_ids.size() < 2) {
    return false;
  }

  const auto edges = state.SavedBackboneEdges();
  if (edges.empty()) {
    return false;
  }
  bool has_grouped_edge = false;
  for (const auto& edge : edges) {
    if (!edge.bundles.empty()) {
      has_grouped_edge = true;
      break;
    }
  }
  if (!has_grouped_edge) {
    return false;
  }

  const ObjectId start = generated.value.generated_pole_ids.front();
  const ObjectId end = generated.value.generated_pole_ids.back();
  const auto route = state.FindSavedBackboneRoute(start, end);
  return !route.empty() && route.front() == start && route.back() == end;
}

bool test_set_pole_flip180_updates_ports() {
  CoreState state;
  const auto fixture = make_backbone_fixture(state, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}});
  if (!fixture.ok || fixture.value.spans.empty() || fixture.value.poles.empty()) return false;
  const ObjectId pole_a = fixture.value.poles.front();
  const auto* span = state.view().spans().find(fixture.value.spans.front());
  if (span == nullptr) return false;
  const auto* port_before = state.view().edit_state().ports.find(span->port_a_id);
  if (port_before == nullptr) {
    return false;
  }
  const wire::core::Vec3d before_pos = port_before->world_position;
  const auto flip = state.SetPoleFlip180(pole_a, true);
  if (!flip.ok) {
    return false;
  }

  const auto* port_after = state.view().edit_state().ports.find(span->port_a_id);
  if (port_after == nullptr) {
    return false;
  }
  if (almost_equal(before_pos, port_after->world_position)) {
    return false;
  }
  const auto pole_view = state.view().inspect_pole(pole_a);
  return pole_view.has_value() && pole_view->flip_180_override.value_or(false);
}

void register_regeneration_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C50_Phase48c_PortMode_DefaultAuto", "New ports default to Auto mode", "Exact", false, test_port_position_mode_defaults_auto);
  test_registry::AddTest(tests, "C52_Phase48c_PortMode_ProtectFromRelayout", "Manual ports are not overwritten by auto pole relayout", "Invariant", false, test_manual_port_not_overwritten_by_auto_relayout);
  test_registry::AddTest(tests, "C71_Phase48k_MovePole_ReprojectAutoPreserveManual", "MovePole reprojects owned Auto ports and preserves Manual ports", "Invariant", false, test_move_pole_reprojects_auto_ports_and_preserves_manual_ports);
  test_registry::AddTest(tests, "C53_Phase48h_Guide_ManualBoundaryStable", "Guide extension keeps explicitly pinned poles fixed", "Invariant", false, test_generate_from_guide_keeps_manual_boundaries_stable);
  test_registry::AddTest(tests, "C64_Phase48i_Guide_NoForcedManualVertices", "Guide generation does not force manual poles by default (including endpoints)", "Exact", false, test_generate_from_guide_vertices_are_not_forced_manual_by_default);
  test_registry::AddTest(tests, "C65_Phase48i_Guide_PinVerticesOption", "Guide pin_vertices option explicitly pins intermediate vertices", "Exact", false, test_generate_from_guide_pin_vertices_option);
  test_registry::AddTest(tests, "C66_Phase48i_PolePlacementMode_Roundtrip", "Pole placement mode can round-trip Auto<->Manual by API", "Exact", false, test_set_pole_placement_mode_auto_manual_roundtrip);
  test_registry::AddTest(tests, "C55_Phase48h_Backbone_Route", "Backbone edges are built from grouped spans and route search works", "Invariant", false, test_backbone_edges_and_route_search);
}

WIRE_REGISTER_TEST_SUITE(register_regeneration_tests);

} // namespace
