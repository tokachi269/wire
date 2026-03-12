#include "registry.hpp"

#include "../src/state/internal_services.hpp"
#include "helpers.hpp"

namespace {

using helpers::contains_id;
using helpers::sorted_pole_type_ids;
using wire::core::AddConnectionByPoleOptions;
using wire::core::BundleKind;
using wire::core::ChangeSet;
using wire::core::ConnectionCategory;
using wire::core::CoreState;
using wire::core::CoreStateTestHook;
using wire::core::ObjectId;
using wire::core::PlacementMode;
using wire::core::PoleKind;
using wire::core::Transformd;
using wire::core::Vec3d;

bool test_endpoint_refresh_service_collects_owned_endpoints_from_relation_index() {
  CoreState state;

  Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  Transformd b_tf{};
  b_tf.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;

  const ObjectId port_a = state.AddPort(pole_a, {0.0, 0.4, 6.0}).value;
  const ObjectId port_b = state.AddPort(pole_b, {10.0, 0.4, 6.0}).value;
  const ObjectId anchor_a = state.AddAnchor(pole_a, {0.0, -0.6, 1.0}).value;
  const ObjectId anchor_b = state.AddAnchor(pole_b, {10.0, -0.6, 1.0}).value;

  const auto owned = wire::core::state_internal::EndpointRefreshService::CollectOwnedEndpointIds(state, pole_a);
  const auto& relation_index = CoreStateTestHook::relation_index(state);
  const auto ports_it = relation_index.ports_by_pole.find(pole_a);
  const auto anchors_it = relation_index.anchors_by_pole.find(pole_a);

  return ports_it != relation_index.ports_by_pole.end() && anchors_it != relation_index.anchors_by_pole.end() &&
         owned.port_ids == ports_it->second && owned.anchor_ids == anchors_it->second &&
         contains_id(owned.port_ids, port_a) && !contains_id(owned.port_ids, port_b) &&
         contains_id(owned.anchor_ids, anchor_a) && !contains_id(owned.anchor_ids, anchor_b);
}

bool test_endpoint_refresh_service_refreshes_only_target_pole_owned_endpoints() {
  CoreState state;

  Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  Transformd b_tf{};
  b_tf.position = {15.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;

  const ObjectId port_a = state.AddPort(pole_a, {0.2, 0.5, 5.5}).value;
  const ObjectId anchor_a = state.AddAnchor(pole_a, {0.0, -0.5, 1.2}).value;
  const ObjectId port_b = state.AddPort(pole_b, {15.2, 0.5, 5.5}).value;
  const ObjectId anchor_b = state.AddAnchor(pole_b, {15.0, -0.5, 1.2}).value;

  const auto* old_port_a = state.view().ports().find(port_a);
  const auto* old_anchor_a = state.view().anchors().find(anchor_a);
  const auto* old_port_b = state.view().ports().find(port_b);
  const auto* old_anchor_b = state.view().anchors().find(anchor_b);
  if (old_port_a == nullptr || old_anchor_a == nullptr || old_port_b == nullptr || old_anchor_b == nullptr) {
    return false;
  }
  const Vec3d old_port_a_pos = old_port_a->world_position;
  const Vec3d old_anchor_a_pos = old_anchor_a->world_position;
  const Vec3d old_port_b_pos = old_port_b->world_position;
  const Vec3d old_anchor_b_pos = old_anchor_b->world_position;

  wire::core::Pole* pole_a_edit = CoreStateTestHook::edit_state(state).poles.find(pole_a);
  if (pole_a_edit == nullptr) {
    return false;
  }
  const wire::core::Pole previous_pole = *pole_a_edit;
  pole_a_edit->world_transform.position.x += 3.0;
  pole_a_edit->world_transform.position.y += 1.0;

  ChangeSet change_set{};
  wire::core::state_internal::EndpointRefreshService::RefreshOwnedEndpointsFromPole(state, pole_a, &change_set,
                                                                                    &previous_pole);

  const auto* new_port_a = state.view().ports().find(port_a);
  const auto* new_anchor_a = state.view().anchors().find(anchor_a);
  const auto* new_port_b = state.view().ports().find(port_b);
  const auto* new_anchor_b = state.view().anchors().find(anchor_b);
  if (new_port_a == nullptr || new_anchor_a == nullptr || new_port_b == nullptr || new_anchor_b == nullptr) {
    return false;
  }

  const bool a_moved = !helpers::almost_equal(old_port_a_pos, new_port_a->world_position) &&
                       !helpers::almost_equal(old_anchor_a_pos, new_anchor_a->world_position);
  const bool b_unchanged = helpers::almost_equal(old_port_b_pos, new_port_b->world_position) &&
                           helpers::almost_equal(old_anchor_b_pos, new_anchor_b->world_position);

  return a_moved && b_unchanged && contains_id(change_set.updated_ids, port_a) &&
         contains_id(change_set.updated_ids, anchor_a) && !contains_id(change_set.updated_ids, port_b) &&
         !contains_id(change_set.updated_ids, anchor_b);
}

bool test_override_resolution_service_matches_formal_override_state() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  Transformd b_tf{};
  b_tf.position = {8.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;
  (void)state.ApplyPoleType(pole_a, pole_type_ids.front());
  (void)state.ApplyPoleType(pole_b, pole_type_ids.front());

  AddConnectionByPoleOptions options{};
  options.use_bundle_template = true;
  options.bundle_template_id = BundleKind::kLowVoltage;
  const auto connect = state.AddConnectionByPole(pole_a, pole_b, ConnectionCategory::kLowVoltage, options);
  if (!connect.ok) {
    return false;
  }
  const ObjectId span_id = connect.value.span_id;

  if (!state.SetPoleManualYawOverride(pole_a, 37.0).ok || !state.SetPoleFlip180(pole_a, true).ok ||
      !state.SetSpanEndpointSocketOverride(span_id, true, 3).ok ||
      !state.SetSpanBranchDownOffsetOverride(span_id, 0.42).ok) {
    return false;
  }

  const wire::core::Pole* pole = state.view().poles().find(pole_a);
  const wire::core::Span* span = state.view().spans().find(span_id);
  if (pole == nullptr || span == nullptr) {
    return false;
  }

  const auto manual_yaw =
      wire::core::state_internal::OverrideResolutionService::ResolvePoleManualYawOverride(state, *pole);
  const auto flip =
      wire::core::state_internal::OverrideResolutionService::ResolvePoleFlip180Override(state, *pole);
  const int socket = wire::core::state_internal::OverrideResolutionService::ResolveSpanEndpointSocketId(state, *span, true);
  const double down = wire::core::state_internal::OverrideResolutionService::ResolveSpanBranchDownOffsetM(state, *span, 0.1);

  return wire::core::state_internal::OverrideResolutionService::HasPoleOrientationOverride(state, pole_a) &&
         wire::core::state_internal::OverrideResolutionService::HasSpanEndpointSocketOverride(state, span_id, true) &&
         wire::core::state_internal::OverrideResolutionService::HasSpanBranchDownOffsetOverride(state, span_id) &&
         manual_yaw.has_value() && helpers::almost_equal(*manual_yaw, 37.0) && flip.value_or(false) && socket == 3 &&
         helpers::almost_equal(down, 0.42);
}

bool test_apply_pole_type_reuses_only_relation_index_owned_endpoints() {
  CoreState state;
  const auto pole_type_ids = sorted_pole_type_ids(state);
  if (pole_type_ids.empty()) {
    return false;
  }

  Transformd a_tf{};
  a_tf.position = {0.0, 0.0, 0.0};
  Transformd b_tf{};
  b_tf.position = {10.0, 0.0, 0.0};
  const ObjectId pole_a = state.AddPole(a_tf, 10.0, "A", PoleKind::kGeneric, PlacementMode::kAuto).value;
  const ObjectId pole_b = state.AddPole(b_tf, 10.0, "B", PoleKind::kGeneric, PlacementMode::kAuto).value;

  if (!state.ApplyPoleType(pole_a, pole_type_ids.front()).ok || !state.ApplyPoleType(pole_b, pole_type_ids.front()).ok) {
    return false;
  }

  const auto& before_index = CoreStateTestHook::relation_index(state);
  const auto ports_a_before = before_index.ports_by_pole.find(pole_a);
  const auto ports_b_before = before_index.ports_by_pole.find(pole_b);
  const auto anchors_a_before = before_index.anchors_by_pole.find(pole_a);
  const auto anchors_b_before = before_index.anchors_by_pole.find(pole_b);
  if (ports_a_before == before_index.ports_by_pole.end() || ports_b_before == before_index.ports_by_pole.end() ||
      anchors_a_before == before_index.anchors_by_pole.end() || anchors_b_before == before_index.anchors_by_pole.end()) {
    return false;
  }

  const std::vector<ObjectId> ports_a_ids = ports_a_before->second;
  const std::vector<ObjectId> ports_b_ids = ports_b_before->second;
  const std::vector<ObjectId> anchors_a_ids = anchors_a_before->second;
  const std::vector<ObjectId> anchors_b_ids = anchors_b_before->second;
  const std::size_t port_count_before = state.view().ports().items().size();
  const std::size_t anchor_count_before = state.view().anchors().items().size();

  if (!state.ApplyPoleType(pole_a, pole_type_ids.front()).ok) {
    return false;
  }

  const auto& after_index = CoreStateTestHook::relation_index(state);
  const auto ports_a_after = after_index.ports_by_pole.find(pole_a);
  const auto ports_b_after = after_index.ports_by_pole.find(pole_b);
  const auto anchors_a_after = after_index.anchors_by_pole.find(pole_a);
  const auto anchors_b_after = after_index.anchors_by_pole.find(pole_b);
  if (ports_a_after == after_index.ports_by_pole.end() || ports_b_after == after_index.ports_by_pole.end() ||
      anchors_a_after == after_index.anchors_by_pole.end() || anchors_b_after == after_index.anchors_by_pole.end()) {
    return false;
  }

  return ports_a_after->second == ports_a_ids && ports_b_after->second == ports_b_ids &&
         anchors_a_after->second == anchors_a_ids && anchors_b_after->second == anchors_b_ids &&
         state.view().ports().items().size() == port_count_before &&
         state.view().anchors().items().size() == anchor_count_before && state.ValidateFast().ok();
}

void RegisterStateServiceTests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C181_CoreStateService_CollectOwnedEndpoints",
                         "endpoint refresh service targets owned endpoints via relation index",
                         "Invariant", false, &test_endpoint_refresh_service_collects_owned_endpoints_from_relation_index);
  test_registry::AddTest(tests, "C182_CoreStateService_RefreshOwnedEndpoints",
                         "endpoint refresh service updates only the target pole owned endpoints",
                         "Invariant", false, &test_endpoint_refresh_service_refreshes_only_target_pole_owned_endpoints);
  test_registry::AddTest(tests, "C183_CoreStateService_OverrideResolution",
                         "override resolution service keeps formal override precedence coherent",
                         "Invariant", false, &test_override_resolution_service_matches_formal_override_state);
  test_registry::AddTest(tests, "C184_CoreStateService_ApplyPoleType_ReusesOwnedEndpoints",
                         "template application reuses relation-index-owned ports and anchors without touching other poles",
                         "Invariant", false, &test_apply_pole_type_reuses_only_relation_index_owned_endpoints);
}

WIRE_REGISTER_TEST_SUITE(RegisterStateServiceTests);

} // namespace
