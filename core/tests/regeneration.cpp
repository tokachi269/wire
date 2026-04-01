#include "registry.hpp"
#include "helpers.hpp"

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <unordered_set>
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
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());
  const auto add = add_connection_by_category(state, pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!add.ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;

  const ObjectId port_id = add.value.port_a_id;
  const wire::core::Vec3d manual_pos{123.0, 45.0, 9.0};
  const auto manual = state.SetPortWorldPositionManual(port_id, manual_pos);
  if (!manual.ok) {
    return false;
  }
  const auto* runtime = state.view().find_span_runtime_state(add.value.span_id);
  const auto* manual_port = state.view().edit_state().ports.find(port_id);
  if (runtime == nullptr || manual_port == nullptr) {
    return false;
  }
  if (!has_dirty(runtime, DirtyBits::kGeometry)) {
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
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {12.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());
  const auto add = add_connection_by_category(state, pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!add.ok) {
    return false;
  }
  const ObjectId manual_port_id = add.value.port_a_id;
  const wire::core::Vec3d manual_pos{77.0, -12.0, 8.0};
  if (!state.SetPortWorldPositionManual(manual_port_id, manual_pos).ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;
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

  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {40.0, 0.0, 0.0}};
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

bool test_generate_from_guide_local_update_no_duplicate_unchanged_segments() {
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
  if (!first.ok || first.value.generated_span_ids.empty()) {
    return false;
  }
  const std::size_t spans_after_first = state.view().edit_state().spans.size();

  const auto second = state.GenerateFromBackboneSpec(req);
  if (!second.ok) {
    return false;
  }
  if (!second.value.generated_span_ids.empty()) {
    return false;
  }
  if (state.view().edit_state().spans.size() != spans_after_first) {
    return false;
  }

  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
  const auto third = state.GenerateFromBackboneSpec(req);
  if (!third.ok || third.value.generated_span_ids.empty()) {
    return false;
  }
  return state.view().edit_state().spans.size() > spans_after_first;
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

bool test_regenerate_session_auto_parts_keeps_manual_pole() {
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
  if (!first.ok || first.value.generated_pole_ids.empty()) {
    return false;
  }
  const auto* any_generated = state.view().edit_state().poles.find(first.value.generated_pole_ids.front());
  if (any_generated == nullptr || any_generated->generation.generation_session_id == 0) {
    return false;
  }
  const std::uint64_t session_id = any_generated->generation.generation_session_id;

  ObjectId middle_id = wire::core::kInvalidObjectId;
  for (const auto& pole : state.view().edit_state().poles.items()) {
    if (pole.generation.generation_session_id == session_id &&
        almost_equal(pole.world_transform.position, wire::core::Vec3d{10.0, 0.0, 0.0})) {
      middle_id = pole.id;
      break;
    }
  }
  if (middle_id == wire::core::kInvalidObjectId) {
    return false;
  }

  if (!state.SetPolePlacementMode(middle_id, wire::core::PlacementMode::kManual).ok) {
    return false;
  }
  wire::core::Transformd moved{};
  moved.position = {11.0, 1.0, 0.0};
  if (!state.MovePole(middle_id, moved).ok) {
    return false;
  }

  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
  const auto regen = state.RegenerateSessionAutoParts(session_id, req);
  if (!regen.ok) {
    return false;
  }

  const auto* middle_after = state.view().edit_state().poles.find(middle_id);
  if (middle_after == nullptr) {
    return false;
  }
  return middle_after->placement_mode == wire::core::PlacementMode::kManual &&
         almost_equal(middle_after->world_transform.position, moved.position);
}

bool test_regenerate_session_auto_parts_keeps_manual_port() {
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
  if (!first.ok || first.value.generated_span_ids.empty()) {
    return false;
  }

  const auto* any_generated = state.view().edit_state().poles.find(first.value.generated_pole_ids.front());
  if (any_generated == nullptr || any_generated->generation.generation_session_id == 0) {
    return false;
  }
  const std::uint64_t session_id = any_generated->generation.generation_session_id;

  const auto* span = state.view().edit_state().spans.find(first.value.generated_span_ids.front());
  if (span == nullptr) {
    return false;
  }
  const auto* port = state.view().edit_state().ports.find(span->port_a_id);
  if (port == nullptr || port->owner_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const ObjectId manual_port_id = port->id;
  const ObjectId manual_port_owner = port->owner_pole_id;
  if (!state.SetPolePlacementMode(manual_port_owner, wire::core::PlacementMode::kManual).ok) {
    return false;
  }
  const wire::core::Vec3d manual_pos{77.0, -14.0, 8.0};
  if (!state.SetPortWorldPositionManual(manual_port_id, manual_pos).ok) {
    return false;
  }

  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {35.0, 0.0, 0.0}};
  const auto regen = state.RegenerateSessionAutoParts(session_id, req);
  if (!regen.ok) {
    return false;
  }

  const auto* port_after = state.view().edit_state().ports.find(manual_port_id);
  return port_after != nullptr && port_after->position_mode == wire::core::PortPositionMode::kManual &&
         almost_equal(port_after->world_position, manual_pos);
}

bool test_regenerate_session_auto_parts_keeps_manual_port_on_auto_pole() {
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
  if (!first.ok || first.value.generated_span_ids.empty() || first.value.generated_pole_ids.empty()) {
    return false;
  }

  const auto* any_generated = state.view().edit_state().poles.find(first.value.generated_pole_ids.front());
  if (any_generated == nullptr || any_generated->generation.generation_session_id == 0) {
    return false;
  }
  const std::uint64_t session_id = any_generated->generation.generation_session_id;

  const auto* span = state.view().edit_state().spans.find(first.value.generated_span_ids.front());
  if (span == nullptr) {
    return false;
  }
  const auto* port = state.view().edit_state().ports.find(span->port_a_id);
  if (port == nullptr || port->owner_pole_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const ObjectId manual_port_id = port->id;
  const ObjectId owner_pole_id = port->owner_pole_id;
  const auto* owner_before = state.view().edit_state().poles.find(owner_pole_id);
  if (owner_before == nullptr || owner_before->placement_mode != wire::core::PlacementMode::kAuto) {
    return false;
  }

  const wire::core::Vec3d manual_pos{61.0, -9.0, 7.0};
  if (!state.SetPortWorldPositionManual(manual_port_id, manual_pos).ok) {
    return false;
  }

  req.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {35.0, 0.0, 0.0}};
  const auto regen = state.RegenerateSessionAutoParts(session_id, req);
  if (!regen.ok) {
    return false;
  }

  const auto* owner_after = state.view().edit_state().poles.find(owner_pole_id);
  const auto* port_after = state.view().edit_state().ports.find(manual_port_id);
  if (owner_after == nullptr || port_after == nullptr) {
    return false;
  }
  return owner_after->placement_mode == wire::core::PlacementMode::kAuto &&
         port_after->position_mode == wire::core::PortPositionMode::kManual &&
         almost_equal(port_after->world_position, manual_pos);
}

bool test_regenerate_session_auto_parts_isolation_across_sessions() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req_a{};
  req_a.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}};
  req_a.interval_m = 10.0;
  req_a.pole_type_id = type_ids.front();
  add_backbone_bundle(req_a, wire::core::BundleKind::kLowVoltage);
  const auto gen_a = state.GenerateFromBackboneSpec(req_a);
  if (!gen_a.ok || gen_a.value.generated_pole_ids.empty()) {
    return false;
  }
  const auto* pole_a0 = state.view().edit_state().poles.find(gen_a.value.generated_pole_ids.front());
  if (pole_a0 == nullptr || pole_a0->generation.generation_session_id == 0) {
    return false;
  }
  const std::uint64_t session_a = pole_a0->generation.generation_session_id;

  wire::core::BackboneSpec req_b{};
  req_b.path.polyline = {{0.0, 30.0, 0.0}, {20.0, 30.0, 0.0}};
  req_b.interval_m = 10.0;
  req_b.pole_type_id = type_ids.front();
  add_backbone_bundle(req_b, wire::core::BundleKind::kLowVoltage);
  const auto gen_b = state.GenerateFromBackboneSpec(req_b);
  if (!gen_b.ok || gen_b.value.generated_pole_ids.empty()) {
    return false;
  }
  const auto* pole_b0 = state.view().edit_state().poles.find(gen_b.value.generated_pole_ids.front());
  if (pole_b0 == nullptr || pole_b0->generation.generation_session_id == 0) {
    return false;
  }
  const std::uint64_t session_b = pole_b0->generation.generation_session_id;

  std::vector<std::pair<ObjectId, wire::core::Vec3d>> session_b_poles{};
  std::vector<ObjectId> session_b_spans{};
  for (const auto& pole : state.view().edit_state().poles.items()) {
    if (pole.generation.generation_session_id == session_b) {
      session_b_poles.push_back({pole.id, pole.world_transform.position});
    }
  }
  for (const auto& span : state.view().edit_state().spans.items()) {
    if (span.generation.generation_session_id == session_b) {
      session_b_spans.push_back(span.id);
    }
  }
  if (session_b_poles.empty()) {
    return false;
  }

  req_a.path.polyline = {{0.0, 0.0, 0.0}, {20.0, 0.0, 0.0}, {30.0, 0.0, 0.0}};
  const auto regen = state.RegenerateSessionAutoParts(session_a, req_a);
  if (!regen.ok) {
    return false;
  }

  for (const auto& [pole_id, pos] : session_b_poles) {
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr || !almost_equal(pole->world_transform.position, pos)) {
      return false;
    }
  }
  for (ObjectId span_id : session_b_spans) {
    if (state.view().edit_state().spans.find(span_id) == nullptr) {
      return false;
    }
  }
  return true;
}

bool test_regenerate_session_acute_corner_keeps_lowering_without_stale_generated_ports() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::BackboneSpec req{};
  req.path.polyline = {{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {-10.0, 2.0, 0.0}};
  req.interval_m = 1000.0;
  req.pole_type_id = type_ids.front();
  add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);

  const auto first = state.GenerateFromBackboneSpec(req);
  if (!first.ok || first.value.generated_pole_ids.empty()) {
    return false;
  }

  const auto* any_generated = state.view().edit_state().poles.find(first.value.generated_pole_ids.front());
  if (any_generated == nullptr || any_generated->generation.generation_session_id == 0) {
    return false;
  }
  const std::uint64_t session_id = any_generated->generation.generation_session_id;

  const auto regen = state.RegenerateSessionAutoParts(session_id, req);
  if (!regen.ok) {
    return false;
  }

  const ObjectId prev_id = find_pole_id_by_position(state, {-12.0, 0.0, 0.0});
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  const ObjectId next_id = find_pole_id_by_position(state, {-10.0, 2.0, 0.0});
  if (prev_id == wire::core::kInvalidObjectId || center_id == wire::core::kInvalidObjectId ||
      next_id == wire::core::kInvalidObjectId) {
    return false;
  }

  std::unordered_set<ObjectId> regenerated_port_ids{};
  for (ObjectId span_id : regen.value.generated_span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr) {
      continue;
    }
    regenerated_port_ids.insert(span->port_a_id);
    regenerated_port_ids.insert(span->port_b_id);
  }

  auto collect_regenerated_hv_ports = [&](ObjectId pole_id) {
    std::vector<const wire::core::Port*> ports{};
    for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
      if (port.owner_pole_id != pole_id || port.layer != wire::core::PortLayer::kHighVoltage ||
          !regenerated_port_ids.contains(port.id)) {
        continue;
      }
      ports.push_back(&port);
    }
    return ports;
  };
  const auto prev_ports = collect_regenerated_hv_ports(prev_id);
  const auto center_ports = collect_regenerated_hv_ports(center_id);
  const auto next_ports = collect_regenerated_hv_ports(next_id);
  if (prev_ports.size() != 3 || center_ports.size() != 3 || next_ports.size() != 3) {
    return false;
  }

  auto min_z = [](const std::vector<const wire::core::Port*>& ports) {
    double z = std::numeric_limits<double>::infinity();
    for (const auto* port : ports) {
      z = std::min(z, port->world_position.z);
    }
    return z;
  };
  auto max_z = [](const std::vector<const wire::core::Port*>& ports) {
    double z = -std::numeric_limits<double>::infinity();
    for (const auto* port : ports) {
      z = std::max(z, port->world_position.z);
    }
    return z;
  };

  const double prev_min_z = min_z(prev_ports);
  const double center_min_z = min_z(center_ports);
  const double center_max_z = max_z(center_ports);
  const double next_min_z = min_z(next_ports);
  const bool center_uniform_height = (center_max_z - center_min_z) <= 1e-6;
  const bool center_lowered = center_max_z + 1e-6 < std::min(prev_min_z, next_min_z);

  int stale_generated_ports = 0;
  for (const wire::core::Port& port : state.view().edit_state().ports.items()) {
    if ((port.owner_pole_id != prev_id && port.owner_pole_id != center_id && port.owner_pole_id != next_id) ||
        port.layer != wire::core::PortLayer::kHighVoltage || !port.generated_by_rule || port.generated_from_template ||
        port.position_mode == wire::core::PortPositionMode::kManual) {
      continue;
    }
    const auto it_spans = state.view().connection_index().spans_by_port.find(port.id);
    if (it_spans == state.view().connection_index().spans_by_port.end() || it_spans->second.empty()) {
      ++stale_generated_ports;
    }
  }

  return center_uniform_height && center_lowered && stale_generated_ports == 0;
}

bool test_regenerate_session_interval_extension_preserves_hv_lane_order() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  auto make_request = [&](const std::vector<wire::core::Vec3d>& path) {
    wire::core::BackboneSpec req{};
    req.path.polyline = path;
    req.interval_m = 8.0;
    req.pole_type_id = type_ids.front();
    add_backbone_bundle(req, wire::core::BundleKind::kLowVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kHighVoltage);
    add_backbone_bundle(req, wire::core::BundleKind::kOptical);
    return req;
  };

  const std::vector<wire::core::Vec3d> base_path = {
      {19.6087, -16.6408, 0.0},
      {8.22759, -11.9276, 0.0},
      {16.8051, -20.9148, 0.0},
      {8.62249, -16.7209, 0.0},
      {12.2073, -24.74, 0.0},
  };
  const std::vector<wire::core::Vec3d> extended_path = {
      {19.6087, -16.6408, 0.0},
      {8.22759, -11.9276, 0.0},
      {16.8051, -20.9148, 0.0},
      {8.62249, -16.7209, 0.0},
      {12.2073, -24.74, 0.0},
      {4.69953, -21.4095, 0.0},
  };

  const auto first = state.GenerateFromBackboneSpec(make_request(base_path));
  if (!first.ok || first.value.generated_span_ids.empty()) {
    return false;
  }

  const auto* any_generated_span = state.view().edit_state().spans.find(first.value.generated_span_ids.front());
  if (any_generated_span == nullptr || any_generated_span->generation.generation_session_id == 0) {
    return false;
  }
  const std::uint64_t session_id = any_generated_span->generation.generation_session_id;

  const auto regen = state.RegenerateSessionAutoParts(session_id, make_request(extended_path));
  if (!regen.ok) {
    return false;
  }

  std::vector<wire::core::SegmentLaneAssignment> hv_assignments{};
  for (const auto& assignment : state.view().last_lane_assignments()) {
    const auto* bundle = state.view().edit_state().bundles.find(assignment.bundle_id);
    if (bundle == nullptr || bundle->bundle_template_id != wire::core::BundleKind::kHighVoltage) {
      continue;
    }
    hv_assignments.push_back(assignment);
  }
  if (hv_assignments.empty()) {
    return false;
  }

  const LaneOrderMetrics metrics = compute_lane_order_metrics(state, hv_assignments);
  const int adjacent_discontinuities = count_bundle_lane_adjacent_order_discontinuities(state, hv_assignments);
  const int polyline_intersections = count_bundle_lane_polyline_xy_intersections(state, hv_assignments);
  if (metrics.y_inversions != 0 || adjacent_discontinuities != 0 || polyline_intersections != 0) {
    dump_lane_assignment_debug(state, hv_assignments, "C207_regen_interval_extension");
    return false;
  }
  return true;
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

  const auto edges = state.BuildBackboneEdges();
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
  const auto route = state.FindBackboneRoute(start, end);
  return !route.empty() && route.front() == start && route.back() == end;
}

bool test_set_pole_flip180_updates_ports_and_dirty() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {12.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, type_ids.front());
  (void)state.ApplyPoleType(pole_b, type_ids.front());
  const auto add = add_connection_by_category(state, pole_a, pole_b, wire::core::ConnectionCategory::kLowVoltage);
  if (!add.ok) {
    return false;
  }
  (void)state.Commit().recalc_stats;

  const auto* port_before = state.view().edit_state().ports.find(add.value.port_a_id);
  if (port_before == nullptr) {
    return false;
  }
  const wire::core::Vec3d before_pos = port_before->world_position;
  const auto flip = state.SetPoleFlip180(pole_a, true);
  if (!flip.ok) {
    return false;
  }

  const auto* port_after = state.view().edit_state().ports.find(add.value.port_a_id);
  const auto* runtime = state.view().find_span_runtime_state(add.value.span_id);
  if (port_after == nullptr || runtime == nullptr) {
    return false;
  }
  if (almost_equal(before_pos, port_after->world_position)) {
    return false;
  }
  if (!has_dirty(runtime, DirtyBits::kGeometry)) {
    return false;
  }
  const auto pole_view = state.view().inspect_pole(pole_a);
  return pole_view.has_value() && pole_view->flip_180_override.value_or(false);
}

void register_regeneration_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C50_Phase48c_PortMode_DefaultAuto", "New ports default to Auto mode", "Exact", false, test_port_position_mode_defaults_auto);
  test_registry::AddTest(tests, "C51_Phase48c_PortMode_ManualReset", "Manual set/reset toggles port mode and keeps dirty local", "Invariant", false, test_port_manual_set_and_reset_to_auto);
  test_registry::AddTest(tests, "C52_Phase48c_PortMode_ProtectFromRelayout", "Manual ports are not overwritten by auto pole relayout", "Invariant", false, test_manual_port_not_overwritten_by_auto_relayout);
  test_registry::AddTest(tests, "C71_Phase48k_MovePole_ReprojectAutoPreserveManual", "MovePole reprojects owned Auto ports and preserves Manual ports", "Invariant", false, test_move_pole_reprojects_auto_ports_and_preserves_manual_ports);
  test_registry::AddTest(tests, "C53_Phase48h_Guide_ManualBoundaryStable", "Guide extension keeps explicitly pinned poles fixed", "Invariant", false, test_generate_from_guide_keeps_manual_boundaries_stable);
  test_registry::AddTest(tests, "C54_Phase48h_Guide_LocalUpdate", "Guide regeneration adds only missing segments without duplicating unchanged spans", "Invariant", false, test_generate_from_guide_local_update_no_duplicate_unchanged_segments);
  test_registry::AddTest(tests, "C64_Phase48i_Guide_NoForcedManualVertices", "Guide generation does not force manual poles by default (including endpoints)", "Exact", false, test_generate_from_guide_vertices_are_not_forced_manual_by_default);
  test_registry::AddTest(tests, "C65_Phase48i_Guide_PinVerticesOption", "Guide pin_vertices option explicitly pins intermediate vertices", "Exact", false, test_generate_from_guide_pin_vertices_option);
  test_registry::AddTest(tests, "C66_Phase48i_PolePlacementMode_Roundtrip", "Pole placement mode can round-trip Auto<->Manual by API", "Exact", false, test_set_pole_placement_mode_auto_manual_roundtrip);
  test_registry::AddTest(tests, "C67_Phase48i_Regenerate_KeepManualPole", "Session-local regeneration preserves manually pinned pole position", "Invariant", false, test_regenerate_session_auto_parts_keeps_manual_pole);
  test_registry::AddTest(tests, "C68_Phase48i_Regenerate_KeepManualPort", "Session-local regeneration preserves manual port position edits", "Invariant", false, test_regenerate_session_auto_parts_keeps_manual_port);
  test_registry::AddTest(tests, "C72_Phase48i_Regenerate_KeepManualPortOnAutoPole", "Session-local regeneration preserves manual port edits even when owner pole remains Auto", "Invariant", false, test_regenerate_session_auto_parts_keeps_manual_port_on_auto_pole);
  test_registry::AddTest(tests, "C69_Phase48i_Regenerate_SessionIsolation", "Session-local regeneration does not mutate other generation sessions", "Invariant", false, test_regenerate_session_auto_parts_isolation_across_sessions);
  test_registry::AddTest(tests, "C206_Phase48i_Regenerate_AcuteLoweringNoStaleGeneratedPorts", "Session regeneration keeps acute-corner lowering and does not accumulate stale generated ports on reused poles", "Invariant", false, test_regenerate_session_acute_corner_keeps_lowering_without_stale_generated_ports);
  test_registry::AddTest(tests, "C207_Phase48i_Regenerate_IntervalExtensionPreservesHvLaneOrder", "Session-local regeneration keeps HV lane order continuous when extending an interval-driven DrawPath route", "Invariant", false, test_regenerate_session_interval_extension_preserves_hv_lane_order);
  test_registry::AddTest(tests, "C55_Phase48h_Backbone_Route", "Backbone edges are built from grouped spans and route search works", "Invariant", false, test_backbone_edges_and_route_search);
  test_registry::AddTest(tests, "C40_Phase48_PoleFlip180_Dirty", "Pole flip180 updates owned ports and dirties connected spans", "Invariant", false, test_set_pole_flip180_updates_ports_and_dirty);
}

WIRE_REGISTER_TEST_SUITE(register_regeneration_tests);

} // namespace
