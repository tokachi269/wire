#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include "wire/core/core_state.hpp"

namespace {

using wire::core::ConnectionCategory;
using wire::core::CoreState;
using wire::core::DirtyBits;
using wire::core::ObjectId;
using wire::core::PoleTypeId;
using wire::core::PortKind;
using wire::core::PortLayer;
using wire::core::SpanKind;
using wire::core::SpanLayer;

struct TestCase {
  const char* name;
  const char* intent;
  std::function<bool(void)> run;
};

bool has_dirty(const wire::core::SpanRuntimeState* state, DirtyBits bits) {
  return state != nullptr && wire::core::any(state->dirty_bits, bits);
}

bool contains_id(const std::vector<ObjectId>& ids, ObjectId id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

bool has_issue_code(const wire::core::ValidationResult& validation, const std::string& code) {
  for (const auto& issue : validation.issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}

bool almost_equal(double a, double b, double eps = 1e-9) {
  return std::abs(a - b) <= eps;
}

bool almost_equal(const wire::core::Vec3d& a, const wire::core::Vec3d& b, double eps = 1e-9) {
  return almost_equal(a.x, b.x, eps) && almost_equal(a.y, b.y, eps) && almost_equal(a.z, b.z, eps);
}

bool aabb_valid(const wire::core::AABBd& aabb) {
  return aabb.min.x <= aabb.max.x &&
         aabb.min.y <= aabb.max.y &&
         aabb.min.z <= aabb.max.z;
}

bool starts_with(const std::string& value, const std::string& prefix) {
  return value.rfind(prefix, 0) == 0;
}

std::vector<PoleTypeId> sorted_pole_type_ids(const CoreState& state) {
  std::vector<PoleTypeId> ids;
  ids.reserve(state.pole_types().size());
  for (const auto& [id, _] : state.pole_types()) {
    ids.push_back(id);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

// Intent: AddSpan should create runtime state and initial dirty bits.
bool test_span_runtime_initialized_on_add() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 9.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {4.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const auto span_result = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage);
  if (!span_result.ok) {
    return false;
  }

  const auto* runtime = state.find_span_runtime_state(span_result.value);
  return runtime != nullptr &&
         runtime->span_id == span_result.value &&
         runtime->data_version > 0 &&
         has_dirty(runtime, DirtyBits::kTopology | DirtyBits::kGeometry);
}

// Intent: MovePole should dirty only related spans.
bool test_move_pole_dirties_only_related_span() {
  CoreState state;
  const ObjectId pole_a = state.AddPole({}, 9.0, "A").value;
  const ObjectId pole_b = state.AddPole({}, 9.0, "B").value;

  const ObjectId a1 = state.AddPort(pole_a, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId a2 = state.AddPort(pole_a, {3.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b1 = state.AddPort(pole_b, {10.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b2 = state.AddPort(pole_b, {13.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;

  const ObjectId related = state.AddSpan(a1, a2, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  const ObjectId unrelated = state.AddSpan(b1, b2, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.ProcessDirtyQueues();

  wire::core::Transformd moved{};
  moved.position = {2.0, 0.0, 0.0};
  const auto move_result = state.MovePole(pole_a, moved);
  if (!move_result.ok) {
    return false;
  }

  return has_dirty(state.find_span_runtime_state(related), DirtyBits::kGeometry) &&
         !has_dirty(state.find_span_runtime_state(unrelated), DirtyBits::kGeometry);
}

// Intent: SplitSpan should replace old span with two new spans and create split port.
bool test_split_span_creates_two_spans_and_port() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 9.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {10.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span_id = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.ProcessDirtyQueues();

  const auto split_result = state.SplitSpan(span_id, 0.5);
  if (!split_result.ok) {
    return false;
  }

  if (state.edit_state().spans.find(span_id) != nullptr) {
    return false;
  }
  if (state.edit_state().ports.find(split_result.value.new_port_id) == nullptr) {
    return false;
  }
  if (state.edit_state().spans.find(split_result.value.new_span_a_id) == nullptr ||
      state.edit_state().spans.find(split_result.value.new_span_b_id) == nullptr) {
    return false;
  }
  return state.Validate().ok();
}

// Intent: ApplyPoleType should create template ports owned by the pole.
bool test_apply_pole_type_generates_template_ports() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.size() < 2) {
    return false;
  }

  const ObjectId pole_id = state.AddPole({}, 10.0, "P").value;
  const auto apply_result = state.ApplyPoleType(pole_id, pole_type_ids[0]);
  if (!apply_result.ok) {
    return false;
  }

  const auto detail = state.GetPoleDetail(pole_id);
  if (detail.pole == nullptr || detail.pole_type == nullptr || detail.owned_ports.empty()) {
    return false;
  }
  for (const auto* port : detail.owned_ports) {
    if (port->owner_pole_id != pole_id) {
      return false;
    }
  }
  return true;
}

// Intent: Different PoleType should produce different port slot composition.
bool test_different_pole_types_produce_different_port_layouts() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.size() < 2) {
    return false;
  }

  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole({}, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, pole_type_ids[0]).ok) {
    return false;
  }
  if (!state.ApplyPoleType(pole_b, pole_type_ids[1]).ok) {
    return false;
  }

  const auto detail_a = state.GetPoleDetail(pole_a);
  const auto detail_b = state.GetPoleDetail(pole_b);
  if (detail_a.owned_ports.empty() || detail_b.owned_ports.empty()) {
    return false;
  }
  if (detail_a.owned_ports.size() != detail_b.owned_ports.size()) {
    return true;
  }

  std::vector<int> slots_a;
  std::vector<int> slots_b;
  for (const auto* port : detail_a.owned_ports) slots_a.push_back(port->source_slot_id);
  for (const auto* port : detail_b.owned_ports) slots_b.push_back(port->source_slot_id);
  std::sort(slots_a.begin(), slots_a.end());
  std::sort(slots_b.begin(), slots_b.end());
  return slots_a != slots_b;
}

// Intent: Auto allocation should consume unused slot first for repeated same-category connections.
bool test_auto_port_allocation_prefers_unused_slots() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {20.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_a, pole_type_ids[0]).ok || !state.ApplyPoleType(pole_b, pole_type_ids[0]).ok) {
    return false;
  }

  const auto first = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kHighVoltage);
  const auto second = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kHighVoltage);
  if (!first.ok || !second.ok) {
    return false;
  }
  return first.value.slot_a_id >= 0 &&
         second.value.slot_a_id >= 0 &&
         first.value.slot_a_id != second.value.slot_a_id;
}

// Intent: Pole->Pole connection should create span and keep dirty/index/change-set consistent.
bool test_add_connection_by_pole_updates_dirty_version_and_indices() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {16.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, pole_type_ids[0]);
  (void)state.ApplyPoleType(pole_b, pole_type_ids[0]);

  const auto connection = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!connection.ok) {
    return false;
  }

  const auto* span = state.edit_state().spans.find(connection.value.span_id);
  const auto* port_a = state.edit_state().ports.find(connection.value.port_a_id);
  const auto* port_b = state.edit_state().ports.find(connection.value.port_b_id);
  const auto* runtime = state.find_span_runtime_state(connection.value.span_id);
  if (span == nullptr || port_a == nullptr || port_b == nullptr || runtime == nullptr) {
    return false;
  }
  if (port_a->owner_pole_id != pole_a || port_b->owner_pole_id != pole_b) {
    return false;
  }

  auto it_a = state.connection_index().spans_by_port.find(port_a->id);
  auto it_b = state.connection_index().spans_by_port.find(port_b->id);
  if (it_a == state.connection_index().spans_by_port.end() || it_b == state.connection_index().spans_by_port.end()) {
    return false;
  }

  return contains_id(it_a->second, span->id) &&
         contains_id(it_b->second, span->id) &&
         has_dirty(runtime, DirtyBits::kGeometry) &&
         contains_id(connection.change_set.created_ids, span->id) &&
         contains_id(connection.change_set.dirty_span_ids, span->id);
}

// Intent: AddDropFromPole should create service span from source pole to free endpoint.
bool test_add_drop_from_pole_creates_service_connection() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  (void)state.ApplyPoleType(pole, pole_type_ids[0]);
  const auto drop = state.AddDropFromPole(pole, {5.0, 3.0, 2.0});
  if (!drop.ok) {
    return false;
  }

  const auto* span = state.edit_state().spans.find(drop.value.span_id);
  const auto* source_port = state.edit_state().ports.find(drop.value.source_port_id);
  const auto* target_port = state.edit_state().ports.find(drop.value.target_port_id);
  if (span == nullptr || source_port == nullptr || target_port == nullptr) {
    return false;
  }
  return source_port->owner_pole_id == pole &&
         target_port->owner_pole_id == wire::core::kInvalidObjectId &&
         state.Validate().ok();
}

// Intent: AddDropFromSpan should split source span then add drop span from split port.
bool test_add_drop_from_span_splits_and_connects_drop() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::Transformd a{};
  a.position = {0.0, 0.0, 0.0};
  wire::core::Transformd b{};
  b.position = {20.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a, 10.0, "A").value;
  const ObjectId pole_b = state.AddPole(b, 10.0, "B").value;
  (void)state.ApplyPoleType(pole_a, pole_type_ids[0]);
  (void)state.ApplyPoleType(pole_b, pole_type_ids[0]);

  const auto base = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!base.ok) {
    return false;
  }

  const auto drop = state.AddDropFromSpan(base.value.span_id, 0.5, {10.0, 4.0, 2.0});
  if (!drop.ok) {
    return false;
  }

  if (state.edit_state().spans.find(base.value.span_id) != nullptr) {
    return false;
  }
  const auto* split_port = state.edit_state().ports.find(drop.value.split_port_id);
  const auto* drop_span = state.edit_state().spans.find(drop.value.span_id);
  if (split_port == nullptr || drop_span == nullptr) {
    return false;
  }
  auto split_it = state.connection_index().spans_by_port.find(drop.value.split_port_id);
  if (split_it == state.connection_index().spans_by_port.end()) {
    return false;
  }
  return split_it->second.size() >= 3 && state.Validate().ok();
}

// Intent: Validation should detect template-slot inconsistency.
bool test_validation_detects_template_slot_mismatch() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  if (!state.ApplyPoleType(pole, pole_type_ids[0]).ok) {
    return false;
  }
  auto detail = state.GetPoleDetail(pole);
  if (detail.owned_ports.empty()) {
    return false;
  }
  wire::core::Port* broken_port = state.edit_state().ports.find(detail.owned_ports.front()->id);
  if (broken_port == nullptr) {
    return false;
  }
  broken_port->source_slot_id = 999999;
  broken_port->generated_from_template = true;

  const auto validation = state.Validate();
  return has_issue_code(validation, "PortSlotMissing");
}

// Intent: Line mode curve cache should be generated deterministically for the same input.
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
  (void)state.ProcessDirtyQueues();

  const auto* curve1 = state.find_curve_cache(span);
  if (curve1 == nullptr || curve1->points.size() != 9) {
    return false;
  }

  const auto move_result = state.MovePort(a, {0.0, 0.0, 5.0});
  if (!move_result.ok) {
    return false;
  }
  (void)state.ProcessDirtyQueues();
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

// Intent: Sag mode should change middle points while keeping endpoints fixed.
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
  (void)state.ProcessDirtyQueues();
  const auto* line_curve = state.find_curve_cache(span);
  if (line_curve == nullptr || line_curve->points.size() != 11) {
    return false;
  }
  const std::vector<wire::core::Vec3d> line_points = line_curve->points;

  wire::core::GeometrySettings sag = line;
  sag.sag_enabled = true;
  sag.sag_factor = 0.10;
  (void)state.UpdateGeometrySettings(sag, true);
  (void)state.ProcessDirtyQueues();
  const auto* sag_curve = state.find_curve_cache(span);
  if (sag_curve == nullptr || sag_curve->points.size() != 11) {
    return false;
  }

  const std::size_t mid = sag_curve->points.size() / 2;
  return almost_equal(sag_curve->points.front(), line_points.front()) &&
         almost_equal(sag_curve->points.back(), line_points.back()) &&
         sag_curve->points[mid].z < line_points[mid].z;
}

// Intent: Geometry dirty should rebuild only related span and propagate bounds/render versions.
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
  (void)state.ProcessDirtyQueues();

  const auto* before_b = state.find_span_runtime_state(span_b);
  if (before_b == nullptr) {
    return false;
  }
  const std::uint64_t span_b_geometry_before = before_b->geometry_version;

  const auto move_result = state.MovePort(p1, {0.0, 1.0, 1.0});
  if (!move_result.ok) {
    return false;
  }
  const auto* dirty_a = state.find_span_runtime_state(span_a);
  const auto* dirty_b = state.find_span_runtime_state(span_b);
  if (dirty_a == nullptr || dirty_b == nullptr) {
    return false;
  }
  if (!has_dirty(dirty_a, DirtyBits::kGeometry) || has_dirty(dirty_b, DirtyBits::kGeometry)) {
    return false;
  }

  (void)state.ProcessDirtyQueues();
  const auto* after_a = state.find_span_runtime_state(span_a);
  const auto* after_b = state.find_span_runtime_state(span_b);
  if (after_a == nullptr || after_b == nullptr) {
    return false;
  }

  return after_a->geometry_version == after_a->data_version &&
         after_a->bounds_version == after_a->data_version &&
         after_a->render_version == after_a->data_version &&
         after_b->geometry_version == span_b_geometry_before;
}

// Intent: Bounds cache should be generated and valid from curve cache.
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
  (void)state.ProcessDirtyQueues();

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

// Intent: Bounds should follow geometry change when sag setting changes.
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
  (void)state.ProcessDirtyQueues();
  const auto* bounds_line = state.find_bounds_cache(span);
  if (bounds_line == nullptr) {
    return false;
  }
  const double line_min_z = bounds_line->whole.min.z;

  wire::core::GeometrySettings sag = line;
  sag.sag_enabled = true;
  (void)state.UpdateGeometrySettings(sag, true);
  (void)state.ProcessDirtyQueues();
  const auto* bounds_sag = state.find_bounds_cache(span);
  if (bounds_sag == nullptr) {
    return false;
  }

  return bounds_sag->whole.min.z < line_min_z;
}

// Intent: Validation should detect invalid bounds data in cache.
bool test_validation_detects_invalid_bounds() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 10.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {5.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.ProcessDirtyQueues();

  auto bounds_it = state.cache_state().bounds_cache.by_span.find(span);
  if (bounds_it == state.cache_state().bounds_cache.by_span.end()) {
    return false;
  }
  bounds_it->second.whole.min.x = 10.0;
  bounds_it->second.whole.max.x = -10.0;

  const auto validation = state.Validate();
  return has_issue_code(validation, "BoundsInvalid");
}

// Intent: GeneratePolesAlongRoad places poles at interval and applies pole type.
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
    const auto* pole = state.edit_state().poles.find(result.value[i]);
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

// Intent: GenerateSimpleLine should fail when polyline has less than 2 points.
bool test_generate_simple_line_fails_with_short_polyline() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::RoadSegment road{};
  road.id = 80;
  road.polyline = {{0.0, 0.0, 0.0}};
  const auto result = state.GenerateSimpleLine(road, 5.0, type_ids.front(), ConnectionCategory::kLowVoltage);
  return !result.ok;
}

// Intent: GenerateSimpleLine should fail when interval is non-positive.
bool test_generate_simple_line_fails_with_invalid_interval() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  wire::core::RoadSegment road{};
  road.id = 81;
  road.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  const auto result = state.GenerateSimpleLine(road, 0.0, type_ids.front(), ConnectionCategory::kLowVoltage);
  return !result.ok;
}

// Intent: GenerateSpansBetweenPoles connects adjacent poles and keeps index/runtime consistent.
bool test_generate_spans_between_poles_basic() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  std::vector<ObjectId> poles;
  for (int i = 0; i < 4; ++i) {
    wire::core::Transformd tf{};
    tf.position = {static_cast<double>(i) * 10.0, 0.0, 0.0};
    const ObjectId pole_id = state.AddPole(tf, 10.0, "P", wire::core::PoleKind::kConcrete).value;
    if (!state.ApplyPoleType(pole_id, type_ids.front()).ok) {
      return false;
    }
    poles.push_back(pole_id);
  }

  const auto result = state.GenerateSpansBetweenPoles(poles, ConnectionCategory::kLowVoltage);
  if (!result.ok) {
    return false;
  }
  if (result.value.size() != poles.size() - 1) {
    return false;
  }
  for (ObjectId span_id : result.value) {
    const auto* span = state.edit_state().spans.find(span_id);
    const auto* runtime = state.find_span_runtime_state(span_id);
    if (span == nullptr || runtime == nullptr) {
      return false;
    }
    if (!span->generation.generated || span->generation.source != wire::core::GenerationSource::kRoadAuto) {
      return false;
    }
    if (!has_dirty(runtime, DirtyBits::kGeometry)) {
      return false;
    }
  }
  return state.Validate().ok();
}

// Intent: GenerateSimpleLine should create poles+spans and drive geometry/bounds recalc.
bool test_generate_simple_line_integration() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 44;
  road.polyline = {{0.0, 0.0, 0.0}, {15.0, 5.0, 0.0}, {30.0, 0.0, 0.0}};
  const auto result = state.GenerateSimpleLine(road, 6.0, type_ids.front(), ConnectionCategory::kLowVoltage);
  if (!result.ok) {
    return false;
  }
  if (result.value.pole_ids.size() < 3) {
    return false;
  }
  if (result.value.span_ids.size() != result.value.pole_ids.size() - 1) {
    return false;
  }
  if (result.value.generation_session_id == 0) {
    return false;
  }

  for (ObjectId pole_id : result.value.pole_ids) {
    const auto* pole = state.edit_state().poles.find(pole_id);
    if (pole == nullptr || pole->generation.generation_session_id != result.value.generation_session_id) {
      return false;
    }
  }
  for (ObjectId span_id : result.value.span_ids) {
    const auto* span = state.edit_state().spans.find(span_id);
    if (span == nullptr || span->generation.generation_session_id != result.value.generation_session_id) {
      return false;
    }
  }

  if (state.dirty_queue().geometry_dirty_span_ids.size() < result.value.span_ids.size()) {
    return false;
  }

  (void)state.ProcessDirtyQueues();
  for (ObjectId span_id : result.value.span_ids) {
    const auto* runtime = state.find_span_runtime_state(span_id);
    const auto* curve = state.find_curve_cache(span_id);
    const auto* bounds = state.find_bounds_cache(span_id);
    if (runtime == nullptr || curve == nullptr || bounds == nullptr) {
      return false;
    }
    if (runtime->geometry_version != runtime->data_version ||
        runtime->bounds_version != runtime->data_version ||
        runtime->render_version != runtime->data_version) {
      return false;
    }
  }
  return state.Validate().ok();
}

// Intent: Auto-generated line should reuse the same intermediate pole port for through continuity.
bool test_generate_simple_line_reuses_intermediate_ports() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 55;
  road.polyline = {{0.0, 0.0, 0.0}, {40.0, 0.0, 0.0}};
  const auto result = state.GenerateSimpleLine(road, 10.0, type_ids.front(), ConnectionCategory::kLowVoltage);
  if (!result.ok || result.value.pole_ids.size() < 4) {
    return false;
  }

  for (std::size_t i = 1; i + 1 < result.value.pole_ids.size(); ++i) {
    const ObjectId pole_id = result.value.pole_ids[i];
    std::vector<ObjectId> used_ports;
    for (ObjectId span_id : result.value.span_ids) {
      const auto* span = state.edit_state().spans.find(span_id);
      if (span == nullptr) {
        return false;
      }
      const auto* port_a = state.edit_state().ports.find(span->port_a_id);
      const auto* port_b = state.edit_state().ports.find(span->port_b_id);
      if (port_a == nullptr || port_b == nullptr) {
        return false;
      }
      if (port_a->owner_pole_id == pole_id) {
        used_ports.push_back(port_a->id);
      }
      if (port_b->owner_pole_id == pole_id) {
        used_ports.push_back(port_b->id);
      }
    }
    if (used_ports.size() != 2) {
      return false;
    }
    if (used_ports[0] != used_ports[1]) {
      return false;
    }
  }
  return true;
}

// Intent: Display IDs should increment independently per prefix while ObjectId remains global.
bool test_display_id_is_per_prefix_sequence() {
  CoreState state;
  const ObjectId pole1 = state.AddPole({}, 10.0, "P1").value;
  const ObjectId pole2 = state.AddPole({}, 10.0, "P2").value;
  const ObjectId port1 = state.AddPort(pole1, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId port2 = state.AddPort(pole2, {1.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span1 = state.AddSpan(port1, port2, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;

  const auto* p1 = state.edit_state().poles.find(pole1);
  const auto* p2 = state.edit_state().poles.find(pole2);
  const auto* pt1 = state.edit_state().ports.find(port1);
  const auto* pt2 = state.edit_state().ports.find(port2);
  const auto* sp1 = state.edit_state().spans.find(span1);
  if (p1 == nullptr || p2 == nullptr || pt1 == nullptr || pt2 == nullptr || sp1 == nullptr) {
    return false;
  }

  return p1->display_id == "P-000001" &&
         p2->display_id == "P-000002" &&
         pt1->display_id == "PT-000001" &&
         pt2->display_id == "PT-000002" &&
         starts_with(sp1->display_id, "SP-000001");
}

// Intent: Viewer default demo should have enough spans to visually inspect behavior.
bool test_demo_state_has_dense_spans() {
  CoreState state = wire::core::make_demo_state();
  return state.edit_state().poles.size() >= 3 &&
         state.edit_state().spans.size() >= 10 &&
         state.edit_state().ports.size() >= 20;
}

}  // namespace

int main() {
  const std::vector<TestCase> tests = {
      {"Phase3_SpanRuntime_Initialized", "AddSpan initializes runtime+dirty", test_span_runtime_initialized_on_add},
      {"Phase3_MovePole_LocalDirty", "MovePole dirties only related span", test_move_pole_dirties_only_related_span},
      {"Phase3_SplitSpan_Basic", "SplitSpan replaces structure and keeps integrity", test_split_span_creates_two_spans_and_port},
      {"Phase35_ApplyPoleType_GeneratesPorts", "ApplyPoleType generates owned template ports", test_apply_pole_type_generates_template_ports},
      {"Phase35_PoleType_DifferentLayouts", "Different pole types produce different slot layout", test_different_pole_types_produce_different_port_layouts},
      {"Phase35_AutoAlloc_UnusedPriority", "Auto allocation prefers unused slots", test_auto_port_allocation_prefers_unused_slots},
      {"Phase35_AddConnectionByPole_Basic", "Pole->Pole connection updates dirty/index/changeset", test_add_connection_by_pole_updates_dirty_version_and_indices},
      {"Phase35_AddDropFromPole_Basic", "Drop from pole creates service span", test_add_drop_from_pole_creates_service_connection},
      {"Phase35_AddDropFromSpan_Basic", "Drop from span splits and connects", test_add_drop_from_span_splits_and_connects_drop},
      {"Phase35_Validation_TemplateMismatch", "Validation detects slot mismatch", test_validation_detects_template_slot_mismatch},
      {"Phase4_Curve_LineDeterministic", "Line mode curve cache is deterministic", test_curve_cache_line_mode_is_deterministic},
      {"Phase4_Curve_SagBasic", "Sag mode changes midpoint and keeps endpoints", test_sag_mode_changes_midpoint_and_keeps_endpoints},
      {"Phase4_DirtyVersion_LocalGeometryBounds", "Geometry dirty propagates to bounds/render locally", test_geometry_bounds_version_follow_and_locality},
      {"Phase4_Bounds_Generated", "Bounds cache is generated and valid", test_bounds_cache_generated_and_valid},
      {"Phase4_Bounds_FollowGeometry", "Bounds follows geometry setting change", test_bounds_follow_geometry_change},
      {"Phase4_Validation_BoundsInvalid", "Validation detects invalid bounds", test_validation_detects_invalid_bounds},
      {"Phase4_DemoState_Dense", "Demo state has dense enough spans for initial viewer", test_demo_state_has_dense_spans},
      {"Phase45_GeneratePolesAlongRoad_Basic", "Road interval generates pole line with pole type", test_generate_poles_along_road_basic},
      {"Phase46_GenerateSimpleLine_FailShortPolyline", "Simple line fails for short polyline", test_generate_simple_line_fails_with_short_polyline},
      {"Phase46_GenerateSimpleLine_FailInvalidInterval", "Simple line fails for invalid interval", test_generate_simple_line_fails_with_invalid_interval},
      {"Phase45_GenerateSpansBetweenPoles_Basic", "Adjacent poles are auto connected", test_generate_spans_between_poles_basic},
      {"Phase45_GenerateSimpleLine_Integration", "Simple line generation integrates dirty/recalc/caches", test_generate_simple_line_integration},
      {"Phase45_GenerateSimpleLine_Continuity", "Intermediate poles reuse same through-port", test_generate_simple_line_reuses_intermediate_ports},
      {"Phase45_DisplayId_PerPrefix", "Display IDs increment per prefix", test_display_id_is_per_prefix_sequence},
  };

  bool all_passed = true;
  for (const TestCase& test : tests) {
    const bool passed = test.run();
    std::cout << (passed ? "[PASS] " : "[FAIL] ") << test.name << " - " << test.intent << "\n";
    all_passed = all_passed && passed;
  }

  if (!all_passed) {
    std::cerr << "core tests failed\n";
    return 1;
  }

  std::cout << "core tests passed (" << tests.size() << " cases)\n";
  return 0;
}
