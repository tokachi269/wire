#include "registry.hpp"
#include "helpers.hpp"
#include "wire/core/style_context.hpp"

#include <algorithm>
#include <string>
#include <tuple>
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

bool test_generate_spans_between_poles_uses_third_candidate_before_reuse() {
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

  std::vector<std::tuple<int, int, int>> template_keys;
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
    template_keys.emplace_back(first_port->template_layer, static_cast<int>(first_port->template_side),
                               static_cast<int>(first_port->template_role));
  }

  std::sort(template_keys.begin(), template_keys.end());
  template_keys.erase(std::unique(template_keys.begin(), template_keys.end()), template_keys.end());
  return template_keys.size() >= 3;
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
  const auto backbone = state.BuildBackboneResult();
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

bool test_style_context_resolver_is_deterministic_and_route_correlated() {
  wire::core::ContextProfile profile{};
  profile.age = 0.7;
  profile.clutter = 0.35;
  profile.regularity = 0.8;
  profile.service_mix = 0.6;
  profile.style_seed = 42;

  wire::core::StyleRouteKey route_key{};
  route_key.family_id = 1001;
  route_key.bundle_template_id = wire::core::BundleKind::kCommunication;
  route_key.category = wire::core::ConnectionCategory::kCommunication;
  route_key.flow_kind = wire::core::BackboneFlowKind::kMain;

  wire::core::StyleObjectKey object_a{};
  object_a.route = route_key;
  object_a.segment_index = 3;
  object_a.lane_index = 0;
  object_a.kind = wire::core::StyleObjectKind::kSpan;
  object_a.ordinal = 0;

  wire::core::StyleObjectKey object_b = object_a;
  object_b.ordinal = 1;

  const wire::core::ResolvedStyleContext resolved_a =
      wire::core::ResolveStyleContext(profile, route_key, object_a);
  const wire::core::ResolvedStyleContext resolved_a_repeat =
      wire::core::ResolveStyleContext(profile, route_key, object_a);
  const wire::core::ResolvedStyleContext resolved_b =
      wire::core::ResolveStyleContext(profile, route_key, object_b);

  return resolved_a.scope.district_seed == resolved_a_repeat.scope.district_seed &&
         resolved_a.scope.route_seed == resolved_a_repeat.scope.route_seed &&
         resolved_a.scope.object_seed == resolved_a_repeat.scope.object_seed &&
         almost_equal(resolved_a.route.age_bias, resolved_a_repeat.route.age_bias, 1e-12) &&
         almost_equal(resolved_a.object.local_offset_m.x, resolved_a_repeat.object.local_offset_m.x, 1e-12) &&
         almost_equal(resolved_a.object.local_offset_m.y, resolved_a_repeat.object.local_offset_m.y, 1e-12) &&
         resolved_a.route.key.family_id == resolved_b.route.key.family_id &&
         resolved_a.scope.route_seed == resolved_b.scope.route_seed &&
         resolved_a.scope.object_seed != resolved_b.scope.object_seed &&
         (std::abs(resolved_a.object.local_offset_m.x - resolved_b.object.local_offset_m.x) > 1e-9 ||
          std::abs(resolved_a.object.local_offset_m.y - resolved_b.object.local_offset_m.y) > 1e-9 ||
          std::abs(resolved_a.object.choice_bias - resolved_b.object.choice_bias) > 1e-9);
}

bool test_inspection_exposes_resolved_style_context_on_span_and_detail_curve() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }

  wire::core::RoadSegment road{};
  road.id = 91;
  road.polyline = {{0.0, 0.0, 0.0}, {18.0, 0.0, 0.0}, {36.0, 8.0, 0.0}};
  const auto generated = state.GenerateSimpleLine(road, 18.0, type_ids.front(), ConnectionCategory::kCommunication);
  if (!generated.ok || generated.value.span_ids.empty()) {
    return false;
  }

  (void)state.Commit().recalc_stats;

  const ObjectId span_id = generated.value.span_ids.front();
  const auto span_view = state.view().inspect_span(span_id);
  const auto curve_view = state.view().inspect_detail_curve(span_id);
  if (!span_view.has_value() || !curve_view.has_value()) {
    return false;
  }

  const auto& span_style = span_view->style;
  const auto& curve_style = curve_view->style;
  return span_style.has_context && curve_style.has_context && span_style.route_key.family_id != 0 &&
         span_style.route_key.family_id == curve_style.route_key.family_id &&
         span_style.route_key.bundle_template_id == curve_style.route_key.bundle_template_id &&
         span_style.route_key.category == curve_style.route_key.category &&
         span_style.route_key.flow_kind == curve_style.route_key.flow_kind &&
         span_style.object_key.route.family_id == span_style.route_key.family_id &&
         curve_style.object_key.route.family_id == curve_style.route_key.family_id &&
         span_style.resolved.scope.district_seed == curve_style.resolved.scope.district_seed &&
         span_style.resolved.scope.route_seed == curve_style.resolved.scope.route_seed &&
         span_style.resolved.scope.object_seed == curve_style.resolved.scope.object_seed;
}

void register_workflow_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C20_Phase46_GenerateSimpleLine_FailShortPolyline", "Simple line short polyline fails with state unchanged", "Exact", true, test_generate_simple_line_fails_with_short_polyline);
  test_registry::AddTest(tests, "C21_Phase46_GenerateSimpleLine_FailInvalidInterval", "Simple line invalid interval fails with state unchanged", "Exact", true, test_generate_simple_line_fails_with_invalid_interval);
  test_registry::AddTest(tests, "C10_AddConnectionByPole_FailSamePole", "Same-pole connect fails with diagnostics and recovers", "Exact", true, test_add_connection_same_pole_fails_and_recovers);
  test_registry::AddTest(tests, "C22_AddSpan_FailMissingPorts", "AddSpan invalid port failure leaves state recoverable", "Exact", true, test_add_span_missing_ports_fails_and_recovers);
  test_registry::AddTest(tests, "C23_SplitSpan_FailInvalidT", "SplitSpan invalid t failure leaves state recoverable", "Exact", true, test_split_span_invalid_t_fails_and_recovers);
  test_registry::AddTest(tests, "C24_Phase45_GenerateSpansBetweenPoles_Basic", "Adjacent poles are auto connected", "Exact", false, test_generate_spans_between_poles_basic);
  test_registry::AddTest(tests, "C25_Phase45_GenerateSpansBetweenPoles_MultiPass", "Repeated auto-connect adds more spans", "Exact", false, test_generate_spans_between_poles_multiple_passes);
  test_registry::AddTest(tests, "C26_Phase47_AutoConnect_UsesThirdSlot", "Auto-connect uses at least third low-voltage candidate before reuse", "Invariant", false, test_generate_spans_between_poles_uses_third_candidate_before_reuse);
  test_registry::AddTest(tests, "C27_Phase45_GenerateSimpleLine_Integration", "Simple line generation integrates dirty/recalc/caches", "Invariant", false, test_generate_simple_line_integration);
  test_registry::AddTest(tests, "C28_Phase45_GenerateSimpleLine_Continuity", "Intermediate poles reuse same through-port", "Invariant", false, test_generate_simple_line_reuses_intermediate_ports);
  test_registry::AddTest(tests, "C29_Phase45_DisplayId_PerPrefix", "Display IDs increment per prefix", "Exact", false, test_display_id_is_per_prefix_sequence);
  test_registry::AddTest(tests, "C304_StyleContext_DeterministicRouteCorrelated",
                         "Style context resolver stays deterministic per semantic key and shares route-level style across sibling objects",
                         "Invariant", false, test_style_context_resolver_is_deterministic_and_route_correlated);
  test_registry::AddTest(tests, "C305_Inspection_ResolvedStyleContext",
                         "Span and detail inspections expose the same resolved style context for a generated span",
                         "Invariant", false, test_inspection_exposes_resolved_style_context_on_span_and_detail_curve);
}

WIRE_REGISTER_TEST_SUITE(register_workflow_tests);

} // namespace
