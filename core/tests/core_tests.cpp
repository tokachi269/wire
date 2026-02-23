#include <algorithm>
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
         has_dirty(runtime, DirtyBits::kTopology | DirtyBits::kGeometry | DirtyBits::kBounds | DirtyBits::kRender);
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

  return has_dirty(state.find_span_runtime_state(related), DirtyBits::kGeometry | DirtyBits::kBounds | DirtyBits::kRender) &&
         !has_dirty(state.find_span_runtime_state(unrelated), DirtyBits::kGeometry | DirtyBits::kBounds | DirtyBits::kRender);
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
         has_dirty(runtime, DirtyBits::kGeometry | DirtyBits::kBounds | DirtyBits::kRender) &&
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
