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
bool test_generate_simple_line_fails_with_short_polyline() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const CoreCounts before = snapshot_counts(state);
  wire::core::RoadSegment road{};
  road.id = 80;
  road.polyline = {{0.0, 0.0, 0.0}};
  const auto result = state.GenerateSimpleLine(road, 5.0, type_ids.front(), ConnectionCategory::kLowVoltage);
  if (result.ok) {
    return false;
  }
  if (result.error != "road polyline must contain at least 2 points") {
    return false;
  }
  if (!same_counts(before, snapshot_counts(state))) {
    return false;
  }

  // Recovery: next valid input should succeed.
  road.polyline.push_back({5.0, 0.0, 0.0});
  return state.GenerateSimpleLine(road, 5.0, type_ids.front(), ConnectionCategory::kLowVoltage).ok;
}

bool test_generate_simple_line_fails_with_invalid_interval() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const CoreCounts before = snapshot_counts(state);
  wire::core::RoadSegment road{};
  road.id = 81;
  road.polyline = {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}};
  const auto result = state.GenerateSimpleLine(road, 0.0, type_ids.front(), ConnectionCategory::kLowVoltage);
  if (result.ok) {
    return false;
  }
  if (result.error != "interval must be > 0") {
    return false;
  }
  if (!same_counts(before, snapshot_counts(state))) {
    return false;
  }
  return state.GenerateSimpleLine(road, 5.0, type_ids.front(), ConnectionCategory::kLowVoltage).ok;
}

bool test_add_connection_same_pole_fails_and_recovers() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  const ObjectId pole_a = state.AddPole({}, 10.0, "A").value;
  if (!state.ApplyPoleType(pole_a, type_ids.front()).ok) {
    return false;
  }

  const CoreCounts before = snapshot_counts(state);
  const auto bad = add_connection_by_category(state, pole_a, pole_a, ConnectionCategory::kLowVoltage);
  if (bad.ok || !regex_contains(bad.error, "same pole")) {
    return false;
  }
  if (!same_counts(before, snapshot_counts(state))) {
    return false;
  }

  wire::core::Transformd pole_b_tf{};
  pole_b_tf.position = {12.0, 0.0, 0.0};
  const ObjectId pole_b = state.AddPole(pole_b_tf, 10.0, "B").value;
  if (!state.ApplyPoleType(pole_b, type_ids.front()).ok) {
    return false;
  }
  const ObjectId pa = state.AddPort(pole_a, {0.0, 2.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId pb = state.AddPort(pole_b, {12.0, 2.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  return state.AddSpan(pa, pb, SpanKind::kDistribution, SpanLayer::kLowVoltage).ok;
}

bool test_add_span_missing_ports_fails_and_recovers() {
  CoreState state;
  const CoreCounts before = snapshot_counts(state);
  const auto bad = state.AddSpan(111, 222, SpanKind::kDistribution, SpanLayer::kLowVoltage);
  if (bad.ok || bad.error != "span ports do not exist") {
    return false;
  }
  if (!same_counts(before, snapshot_counts(state))) {
    return false;
  }

  const ObjectId pole_a = state.AddPole({}, 9.0, "A").value;
  const ObjectId pole_b = state.AddPole({}, 9.0, "B").value;
  const ObjectId a = state.AddPort(pole_a, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole_b, {3.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  return state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).ok;
}

bool test_split_span_invalid_t_fails_and_recovers() {
  CoreState state;
  const ObjectId pole_a = state.AddPole({}, 9.0, "A").value;
  const ObjectId pole_b = state.AddPole({}, 9.0, "B").value;
  const ObjectId a = state.AddPort(pole_a, {0.0, 0.0, 3.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId b = state.AddPort(pole_b, {6.0, 0.0, 3.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span = state.AddSpan(a, b, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;

  const auto bad = state.SplitSpan(span, 0.0);
  if (bad.ok || bad.error != "split t must be in (0, 1)") {
    return false;
  }
  if (state.view().edit_state().spans.find(span) == nullptr) {
    return false;
  }
  return state.SplitSpan(span, 0.5).ok;
}

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
    const auto* span = state.view().edit_state().spans.find(span_id);
    const auto* runtime = state.view().find_span_runtime_state(span_id);
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
  return validate_now(state).ok();
}

bool test_generate_spans_between_poles_multiple_passes() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  std::vector<ObjectId> poles;
  for (int i = 0; i < 5; ++i) {
    wire::core::Transformd tf{};
    tf.position = {static_cast<double>(i) * 12.0, 0.0, 0.0};
    const ObjectId pole_id = state.AddPole(tf, 10.0, "P", wire::core::PoleKind::kConcrete).value;
    if (!state.ApplyPoleType(pole_id, type_ids.front()).ok) {
      return false;
    }
    poles.push_back(pole_id);
  }

  const std::size_t per_pass = poles.size() - 1;
  for (int pass = 0; pass < 6; ++pass) {
    const auto result = state.GenerateSpansBetweenPoles(poles, ConnectionCategory::kLowVoltage);
    if (!result.ok || result.value.size() != per_pass) {
      return false;
    }
    const std::size_t expected_total = per_pass * static_cast<std::size_t>(pass + 1);
    if (state.view().edit_state().spans.size() != expected_total) {
      return false;
    }
  }
  return validate_now(state).ok();
}

bool test_generate_spans_between_poles_uses_third_slot_before_reuse() {
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

  std::vector<int> slot_ids;
  for (int pass = 0; pass < 3; ++pass) {
    const auto result = state.GenerateSpansBetweenPoles(poles, ConnectionCategory::kLowVoltage);
    if (!result.ok || result.value.empty()) {
      return false;
    }
    const auto* first_span = state.view().edit_state().spans.find(result.value.front());
    if (first_span == nullptr) {
      return false;
    }
    const auto* first_port = state.view().edit_state().ports.find(first_span->port_a_id);
    if (first_port == nullptr || first_port->owner_pole_id != poles.front()) {
      return false;
    }
    if (first_port->source_slot_id < 0) {
      return false;
    }
    slot_ids.push_back(first_port->source_slot_id);
  }

  std::sort(slot_ids.begin(), slot_ids.end());
  slot_ids.erase(std::unique(slot_ids.begin(), slot_ids.end()), slot_ids.end());
  return slot_ids.size() >= 3;
}

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
  const auto& backbone = state.view().last_generation_backbone();
  if (backbone.nodes.empty() || backbone.edges.empty()) {
    return false;
  }

  for (ObjectId pole_id : result.value.pole_ids) {
    const auto* pole = state.view().edit_state().poles.find(pole_id);
    if (pole == nullptr || pole->generation.generation_session_id != result.value.generation_session_id) {
      return false;
    }
  }
  for (ObjectId span_id : result.value.span_ids) {
    const auto* span = state.view().edit_state().spans.find(span_id);
    if (span == nullptr || span->generation.generation_session_id != result.value.generation_session_id) {
      return false;
    }
  }

  if (state.view().dirty_queue().geometry_dirty_span_ids.size() < result.value.span_ids.size()) {
    return false;
  }

  (void)state.Commit().recalc_stats;
  for (ObjectId span_id : result.value.span_ids) {
    const auto* runtime = state.view().find_span_runtime_state(span_id);
    const auto* curve = state.find_curve_cache(span_id);
    const auto* bounds = state.find_bounds_cache(span_id);
    if (runtime == nullptr || curve == nullptr || bounds == nullptr) {
      return false;
    }
    if (runtime->geometry_version != runtime->data_version || runtime->bounds_version != runtime->data_version ||
        runtime->render_version != runtime->data_version) {
      return false;
    }
  }
  return validate_now(state).ok();
}

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
      const auto* span = state.view().edit_state().spans.find(span_id);
      if (span == nullptr) {
        return false;
      }
      const auto* port_a = state.view().edit_state().ports.find(span->port_a_id);
      const auto* port_b = state.view().edit_state().ports.find(span->port_b_id);
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

bool test_display_id_is_per_prefix_sequence() {
  CoreState state;
  const ObjectId pole1 = state.AddPole({}, 10.0, "P1").value;
  const ObjectId pole2 = state.AddPole({}, 10.0, "P2").value;
  const ObjectId port1 = state.AddPort(pole1, {0.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId port2 = state.AddPort(pole2, {1.0, 0.0, 1.0}, PortKind::kPower, PortLayer::kLowVoltage).value;
  const ObjectId span1 = state.AddSpan(port1, port2, SpanKind::kDistribution, SpanLayer::kLowVoltage).value;

  const auto* p1 = state.view().edit_state().poles.find(pole1);
  const auto* p2 = state.view().edit_state().poles.find(pole2);
  const auto* pt1 = state.view().edit_state().ports.find(port1);
  const auto* pt2 = state.view().edit_state().ports.find(port2);
  const auto* sp1 = state.view().edit_state().spans.find(span1);
  if (p1 == nullptr || p2 == nullptr || pt1 == nullptr || pt2 == nullptr || sp1 == nullptr) {
    return false;
  }

  return p1->display_id == "P-000001" && p2->display_id == "P-000002" && pt1->display_id == "PT-000001" &&
         pt2->display_id == "PT-000002" && starts_with(sp1->display_id, "SP-000001");
}

bool test_demo_state_has_dense_spans() {
  CoreState state = wire::core::make_demo_state();
  return state.view().edit_state().poles.size() >= 3 && state.view().edit_state().spans.size() >= 10 &&
         state.view().edit_state().ports.size() >= 20;
}

bool test_clear_debug_records_is_entity_noop() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  wire::core::Transformd pole_a_tf{};
  pole_a_tf.position = {0.0, 0.0, 0.0};
  wire::core::Transformd pole_b_tf{};
  pole_b_tf.position = {12.0, 0.0, 0.0};
  const auto add_a = state.AddPole(pole_a_tf, 10.0, "A");
  const auto add_b = state.AddPole(pole_b_tf, 10.0, "B");
  if (!add_a.ok || !add_b.ok) {
    return false;
  }
  const ObjectId pole_a = add_a.value;
  const ObjectId pole_b = add_b.value;
  if (!state.ApplyPoleType(pole_a, pole_type_ids[0]).ok || !state.ApplyPoleType(pole_b, pole_type_ids[0]).ok) {
    return false;
  }
  if (!add_connection_by_category(state, pole_a, pole_b, ConnectionCategory::kLowVoltage).ok) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 10;
  road.polyline = {{0.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, {12.0, 3.0, 0.0}};
  BackbonePathGenerateOptions options{};
  options.road = road;
  options.interval = 0.0;
  options.pole_type_id = pole_type_ids[0];
  options.bundle_template_id = wire::core::BundleKind::kCommunication;
  options.bundle_count = 2;
  const auto grouped = generate_from_backbone_options(state, options);
  if (!grouped.ok) {
    return false;
  }

  const std::size_t slot_debug_before = state.view().slot_selection_debug_records().size();
  const std::size_t path_debug_before = state.view().path_direction_debug_records().size();

  const CoreCounts before = snapshot_counts(state);
  const auto poles_before = collect_sorted_ids(state.view().edit_state().poles.items());
  const auto ports_before = collect_sorted_ids(state.view().edit_state().ports.items());
  const auto spans_before = collect_sorted_ids(state.view().edit_state().spans.items());
  const bool validate_before = validate_now(state).ok();

  state.clear_slot_selection_debug_records();
  state.clear_path_direction_debug_records();

  const CoreCounts after = snapshot_counts(state);
  const auto poles_after = collect_sorted_ids(state.view().edit_state().poles.items());
  const auto ports_after = collect_sorted_ids(state.view().edit_state().ports.items());
  const auto spans_after = collect_sorted_ids(state.view().edit_state().spans.items());

  return slot_debug_before >= state.view().slot_selection_debug_records().size() &&
         path_debug_before >= state.view().path_direction_debug_records().size() &&
         state.view().slot_selection_debug_records().empty() && state.view().path_direction_debug_records().empty() &&
         same_counts(before, after) && poles_before == poles_after && ports_before == ports_after &&
         spans_before == spans_after && validate_now(state).ok() == validate_before;
}

bool test_recalc_cache_pipeline_is_entity_noop() {
  CoreState state = wire::core::make_demo_state();
  if (state.view().edit_state().spans.empty()) {
    return false;
  }

  const CoreCounts before = snapshot_counts(state);
  const auto poles_before = collect_sorted_ids(state.view().edit_state().poles.items());
  const auto ports_before = collect_sorted_ids(state.view().edit_state().ports.items());
  const auto spans_before = collect_sorted_ids(state.view().edit_state().spans.items());

  (void)state.Commit().recalc_stats;
  wire::core::GeometrySettings settings = state.view().geometry_settings();
  settings.curve_samples = std::max(2, settings.curve_samples + 2);
  const auto update = state.UpdateGeometrySettings(settings, true);
  if (!update.ok) {
    return false;
  }
  const auto recalc = state.Commit().recalc_stats;

  const CoreCounts after = snapshot_counts(state);
  const auto poles_after = collect_sorted_ids(state.view().edit_state().poles.items());
  const auto ports_after = collect_sorted_ids(state.view().edit_state().ports.items());
  const auto spans_after = collect_sorted_ids(state.view().edit_state().spans.items());

  return recalc.geometry_processed > 0 && same_counts(before, after) && poles_before == poles_after &&
         ports_before == ports_after && spans_before == spans_after && validate_now(state).ok();
}

void register_workflow_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C20_Phase46_GenerateSimpleLine_FailShortPolyline", "Simple line short polyline fails with state unchanged", "Exact", true, test_generate_simple_line_fails_with_short_polyline);
  test_registry::AddTest(tests, "C21_Phase46_GenerateSimpleLine_FailInvalidInterval", "Simple line invalid interval fails with state unchanged", "Exact", true, test_generate_simple_line_fails_with_invalid_interval);
  test_registry::AddTest(tests, "C10_AddConnectionByPole_FailSamePole", "Same-pole connect fails with diagnostics and recovers", "Exact", true, test_add_connection_same_pole_fails_and_recovers);
  test_registry::AddTest(tests, "C22_AddSpan_FailMissingPorts", "AddSpan invalid port failure leaves state recoverable", "Exact", true, test_add_span_missing_ports_fails_and_recovers);
  test_registry::AddTest(tests, "C23_SplitSpan_FailInvalidT", "SplitSpan invalid t failure leaves state recoverable", "Exact", true, test_split_span_invalid_t_fails_and_recovers);
  test_registry::AddTest(tests, "C24_Phase45_GenerateSpansBetweenPoles_Basic", "Adjacent poles are auto connected", "Exact", false, test_generate_spans_between_poles_basic);
  test_registry::AddTest(tests, "C25_Phase45_GenerateSpansBetweenPoles_MultiPass", "Repeated auto-connect adds more spans", "Exact", false, test_generate_spans_between_poles_multiple_passes);
  test_registry::AddTest(tests, "C26_Phase47_AutoConnect_UsesThirdSlot", "Auto-connect uses at least third low-voltage slot before reuse", "Invariant", false, test_generate_spans_between_poles_uses_third_slot_before_reuse);
  test_registry::AddTest(tests, "C27_Phase45_GenerateSimpleLine_Integration", "Simple line generation integrates dirty/recalc/caches", "Invariant", false, test_generate_simple_line_integration);
  test_registry::AddTest(tests, "C28_Phase45_GenerateSimpleLine_Continuity", "Intermediate poles reuse same through-port", "Invariant", false, test_generate_simple_line_reuses_intermediate_ports);
  test_registry::AddTest(tests, "C29_Phase45_DisplayId_PerPrefix", "Display IDs increment per prefix", "Exact", false, test_display_id_is_per_prefix_sequence);
  test_registry::AddTest(tests, "C18_Phase4_DemoState_Dense", "Demo state has dense enough spans for initial viewer", "Invariant", false, test_demo_state_has_dense_spans);
  test_registry::AddTest(tests, "C41_Phase4x_ClearDebug_NoEntityMutation", "Clearing session debug records does not mutate core entities", "Exact", false, test_clear_debug_records_is_entity_noop);
  test_registry::AddTest(tests, "C42_Phase4x_RecalcCache_NoEntityMutation", "Derived cache rebuild does not mutate entity identity/counts", "Invariant", false, test_recalc_cache_pipeline_is_entity_noop);
}

WIRE_REGISTER_TEST_SUITE(register_workflow_tests);

} // namespace
