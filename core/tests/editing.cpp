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
bool test_id_generator_monotonic_and_reset() {
  wire::core::IdGenerator gen(1);
  if (gen.next() != 1 || gen.next() != 2 || gen.next() != 3) {
    return false;
  }
  if (gen.peek() != 4) {
    return false;
  }
  gen.reset(42);
  return gen.peek() == 42 && gen.next() == 42 && gen.next() == 43;
}

bool test_object_store_integrity() {
  struct Dummy {
    ObjectId id = 0;
    int payload = 0;
  };

  wire::core::ObjectStore<Dummy> store;
  store.insert(Dummy{1, 10});
  store.insert(Dummy{2, 20});
  store.insert(Dummy{3, 30});
  if (store.size() != 3 || !store.contains(2)) {
    return false;
  }
  if (store.find(3) == nullptr || store.find(3)->payload != 30) {
    return false;
  }
  if (!store.remove(2) || store.contains(2) || store.size() != 2) {
    return false;
  }
  if (store.find(1) == nullptr || store.find(3) == nullptr) {
    return false;
  }

  // Upsert must replace value without changing count.
  store.insert(Dummy{3, 99});
  return store.size() == 2 && store.find(3) != nullptr && store.find(3)->payload == 99;
}

bool test_span_runtime_initialized_on_add() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 9.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {4.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const auto span_result = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage);
  if (!span_result.ok) {
    return false;
  }

  const auto* runtime = state.view().find_span_runtime_state(span_result.value);
  return runtime != nullptr && runtime->span_id == span_result.value && runtime->data_version > 0 &&
         has_dirty(runtime, DirtyBits::kTopology | DirtyBits::kGeometry);
}

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
  (void)state.Commit().recalc_stats;

  wire::core::Transformd moved{};
  moved.position = {2.0, 0.0, 0.0};
  const auto move_result = state.MovePole(pole_a, moved);
  if (!move_result.ok) {
    return false;
  }

  return has_dirty(state.view().find_span_runtime_state(related), DirtyBits::kGeometry) &&
         !has_dirty(state.view().find_span_runtime_state(unrelated), DirtyBits::kGeometry);
}

bool test_split_span_creates_two_spans_and_port() {
  CoreState state;
  const ObjectId pole = state.AddPole({}, 9.0, "P").value;
  const ObjectId a = state.AddPort(pole, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole, {10.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span_id = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;
  (void)state.Commit().recalc_stats;

  const auto split_result = state.SplitSpan(span_id, 0.5);
  if (!split_result.ok) {
    return false;
  }

  if (state.view().edit_state().spans.find(span_id) != nullptr) {
    return false;
  }
  if (state.view().edit_state().ports.find(split_result.value.new_port_id) == nullptr) {
    return false;
  }
  if (state.view().edit_state().spans.find(split_result.value.new_span_a_id) == nullptr ||
      state.view().edit_state().spans.find(split_result.value.new_span_b_id) == nullptr) {
    return false;
  }
  return validate_now(state).ok();
}

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
  for (const auto* port : detail_a.owned_ports)
    slots_a.push_back(port->source_slot_id);
  for (const auto* port : detail_b.owned_ports)
    slots_b.push_back(port->source_slot_id);
  std::sort(slots_a.begin(), slots_a.end());
  std::sort(slots_b.begin(), slots_b.end());
  return slots_a != slots_b;
}

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

  const auto first = add_connection_by_category(state, pole_a, pole_b, ConnectionCategory::kHighVoltage);
  const auto second = add_connection_by_category(state, pole_a, pole_b, ConnectionCategory::kHighVoltage);
  if (!first.ok || !second.ok) {
    return false;
  }
  return first.value.slot_a_id >= 0 && second.value.slot_a_id >= 0 && first.value.slot_a_id != second.value.slot_a_id;
}

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

  const auto connection = add_connection_by_category(state, pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!connection.ok) {
    return false;
  }

  const auto* span = state.view().edit_state().spans.find(connection.value.span_id);
  const auto* port_a = state.view().edit_state().ports.find(connection.value.port_a_id);
  const auto* port_b = state.view().edit_state().ports.find(connection.value.port_b_id);
  const auto* runtime = state.view().find_span_runtime_state(connection.value.span_id);
  if (span == nullptr || port_a == nullptr || port_b == nullptr || runtime == nullptr) {
    return false;
  }
  if (port_a->owner_pole_id != pole_a || port_b->owner_pole_id != pole_b) {
    return false;
  }

  auto it_a = state.view().connection_index().spans_by_port.find(port_a->id);
  auto it_b = state.view().connection_index().spans_by_port.find(port_b->id);
  if (it_a == state.view().connection_index().spans_by_port.end() || it_b == state.view().connection_index().spans_by_port.end()) {
    return false;
  }

  return contains_id(it_a->second, span->id) && contains_id(it_b->second, span->id) &&
         has_dirty(runtime, DirtyBits::kGeometry) && contains_id(connection.change_set.created_ids, span->id) &&
         contains_id(connection.change_set.dirty_span_ids, span->id);
}

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

  const auto* span = state.view().edit_state().spans.find(drop.value.span_id);
  const auto* source_port = state.view().edit_state().ports.find(drop.value.source_port_id);
  const auto* target_port = state.view().edit_state().ports.find(drop.value.target_port_id);
  if (span == nullptr || source_port == nullptr || target_port == nullptr) {
    return false;
  }
  return source_port->owner_pole_id == pole && target_port->owner_pole_id == wire::core::kInvalidObjectId &&
         validate_now(state).ok();
}

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

  const auto base = add_connection_by_category(state, pole_a, pole_b, ConnectionCategory::kLowVoltage);
  if (!base.ok) {
    return false;
  }

  const auto drop = state.AddDropFromSpan(base.value.span_id, 0.5, {10.0, 4.0, 2.0});
  if (!drop.ok) {
    return false;
  }

  if (state.view().edit_state().spans.find(base.value.span_id) != nullptr) {
    return false;
  }
  const auto* split_port = state.view().edit_state().ports.find(drop.value.split_port_id);
  const auto* drop_span = state.view().edit_state().spans.find(drop.value.span_id);
  if (split_port == nullptr || drop_span == nullptr) {
    return false;
  }
  auto split_it = state.view().connection_index().spans_by_port.find(drop.value.split_port_id);
  if (split_it == state.view().connection_index().spans_by_port.end()) {
    return false;
  }
  return split_it->second.size() >= 3 && validate_now(state).ok();
}

void register_editing_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C01_IdGenerator", "IdGenerator monotonic and reset behavior", "Exact", false, test_id_generator_monotonic_and_reset);
  test_registry::AddTest(tests, "C02_ObjectStore", "ObjectStore add/find/remove integrity", "Exact", false, test_object_store_integrity);
  test_registry::AddTest(tests, "C03_Phase3_SpanRuntime_Initialized", "AddSpan initializes runtime+dirty", "Exact", false, test_span_runtime_initialized_on_add);
  test_registry::AddTest(tests, "C04_Phase3_MovePole_LocalDirty", "MovePole dirties only related span", "Invariant", false, test_move_pole_dirties_only_related_span);
  test_registry::AddTest(tests, "C05_Phase3_SplitSpan_Basic", "SplitSpan replaces structure and keeps integrity", "Invariant", false, test_split_span_creates_two_spans_and_port);
  test_registry::AddTest(tests, "C06_Phase35_ApplyPoleType_GeneratesPorts", "ApplyPoleType generates owned template ports", "Invariant", false, test_apply_pole_type_generates_template_ports);
  test_registry::AddTest(tests, "C07_Phase35_PoleType_DifferentLayouts", "Different pole types produce different slot layout", "Invariant", false, test_different_pole_types_produce_different_port_layouts);
  test_registry::AddTest(tests, "C08_Phase35_AutoAlloc_UnusedPriority", "Auto allocation prefers unused slots", "Invariant", false, test_auto_port_allocation_prefers_unused_slots);
  test_registry::AddTest(tests, "C09_Phase35_AddConnectionByPole_Basic", "Pole->Pole connection updates dirty/index/changeset", "Invariant", false, test_add_connection_by_pole_updates_dirty_version_and_indices);
  test_registry::AddTest(tests, "C11_Phase35_AddDropFromPole_Basic", "Drop from pole creates service span", "Invariant", false, test_add_drop_from_pole_creates_service_connection);
  test_registry::AddTest(tests, "C12_Phase35_AddDropFromSpan_Basic", "Drop from span splits and connects", "Invariant", false, test_add_drop_from_span_splits_and_connects_drop);
}

WIRE_REGISTER_TEST_SUITE(register_editing_tests);

} // namespace
