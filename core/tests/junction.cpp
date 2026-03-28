#include <vector>

#include "registry.hpp"
#include "helpers.hpp"

using namespace helpers;

std::optional<wire::core::JunctionInspectionView> inspect_center_junction(const CoreState& state,
                                                                          const wire::core::Vec3d& center) {
  const ObjectId center_id = find_pole_id_by_position(state, center);
  if (center_id == wire::core::kInvalidObjectId) {
    return std::nullopt;
  }
  return state.view().inspect_junction(center_id);
}

struct JunctionIncidentSnapshot {
  ObjectId neighbor = wire::core::kInvalidObjectId;
  int order = -1;
  bool primary = false;
  std::uint64_t session = 0;

  bool operator==(const JunctionIncidentSnapshot& other) const {
    return neighbor == other.neighbor && order == other.order && primary == other.primary && session == other.session;
  }
};

std::vector<JunctionIncidentSnapshot> snapshot_incidents(const wire::core::JunctionInspectionView& junction) {
  std::vector<JunctionIncidentSnapshot> result;
  result.reserve(junction.incidents.size());
  for (const auto& incident : junction.incidents) {
    result.push_back({incident.neighbor_node_id, incident.order, incident.primary, incident.source_session_id});
  }
  return result;
}

std::uint64_t generation_session_id_for_result(const CoreState& state,
                                               const wire::core::GenerateBundleFromPathResult& generated) {
  if (!generated.generated_span_ids.empty()) {
    if (const auto* span = state.view().edit_state().spans.find(generated.generated_span_ids.front()); span != nullptr) {
      return span->generation.generation_session_id;
    }
  }
  if (!generated.generated_pole_ids.empty()) {
    if (const auto* pole = state.view().edit_state().poles.find(generated.generated_pole_ids.front()); pole != nullptr) {
      return pole->generation.generation_session_id;
    }
  }
  return 0;
}

bool test_junction_t_shape_preserves_first_session_primary_order() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  auto run_path = [&](const std::vector<wire::core::Vec3d>& path)
      -> wire::core::EditResult<wire::core::GenerateBundleFromPathResult> {
    wire::core::BackboneSpec req{};
    req.path.polyline = path;
    req.interval_m = 8.0;
    req.pole_type_id = type_ids.front();
    wire::core::BackboneBundleSpec b{};
    b.bundle_template_id = wire::core::BundleKind::kLowVoltage;
    req.bundles.push_back(b);
    return state.GenerateFromBackboneSpec(req);
  };

  const auto first_path = run_path({{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}});
  const std::uint64_t first_primary = first_path.ok ? generation_session_id_for_result(state, first_path.value) : 0;
  if (!first_path.ok || first_primary == 0) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto second_path = run_path({{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}});
  const std::uint64_t second_session = second_path.ok ? generation_session_id_for_result(state, second_path.value) : 0;
  if (!second_path.ok || second_session == 0) {
    return false;
  }
  const auto second_backbone = state.BuildBackboneResult();
  const auto* second_junction = find_junction(second_backbone, center_id);
  if (second_junction == nullptr || second_junction->incidents.size() < 3) {
    return false;
  }
  const auto& primary_incident = second_junction->incidents.front();
  bool later_session_present = false;
  for (const auto& incident : second_junction->incidents) {
    later_session_present = later_session_present || (incident.source_session_id == second_session);
  }
  return second_junction->prioritized_session_id == first_primary && primary_incident.primary &&
         primary_incident.order == 0 && primary_incident.source_session_id == first_primary && later_session_present;
}

// Intent: Junction order should be stable across repeated evaluations (no alternating jitter).
bool test_junction_cross_order_stable_across_rebuilds() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  auto run_path = [&](const std::vector<wire::core::Vec3d>& path) -> bool {
    wire::core::BackboneSpec req{};
    req.path.polyline = path;
    req.interval_m = 8.0;
    req.pole_type_id = type_ids.front();
    wire::core::BackboneBundleSpec b{};
    b.bundle_template_id = wire::core::BundleKind::kLowVoltage;
    req.bundles.push_back(b);
    return state.GenerateFromBackboneSpec(req).ok;
  };
  if (!run_path({{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}}) ||
      !run_path({{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}})) {
    return false;
  }
  const auto a = inspect_center_junction(state, {0.0, 0.0, 0.0});
  const auto b = inspect_center_junction(state, {0.0, 0.0, 0.0});
  if (!a.has_value() || !b.has_value()) {
    return false;
  }
  const auto incidents_a = snapshot_incidents(*a);
  const auto incidents_b = snapshot_incidents(*b);
  if (incidents_a != incidents_b || a->through_pair_accepted != b->through_pair_accepted ||
      a->through_pair_neighbor_a_id != b->through_pair_neighbor_a_id ||
      a->through_pair_neighbor_b_id != b->through_pair_neighbor_b_id ||
      a->through_pair_straightness_score != b->through_pair_straightness_score ||
      a->local_relations.size() != b->local_relations.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a->local_relations.size(); ++i) {
    const auto& ra = a->local_relations[i];
    const auto& rb = b->local_relations[i];
    if (ra.neighbor_node_id != rb.neighbor_node_id || ra.kind != rb.kind || ra.in_through_pair != rb.in_through_pair ||
        ra.straightness_score != rb.straightness_score || ra.used_semantic_tiebreak != rb.used_semantic_tiebreak) {
      return false;
    }
  }
  return true;
}

// Intent: Later DrawPath should not overwrite prioritized session at junction.
bool test_junction_first_session_priority_not_overwritten_by_later_paths() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  auto run_path = [&](const std::vector<wire::core::Vec3d>& path)
      -> wire::core::EditResult<wire::core::GenerateBundleFromPathResult> {
    wire::core::BackboneSpec req{};
    req.path.polyline = path;
    req.interval_m = 8.0;
    req.pole_type_id = type_ids.front();
    wire::core::BackboneBundleSpec b{};
    b.bundle_template_id = wire::core::BundleKind::kLowVoltage;
    req.bundles.push_back(b);
    return state.GenerateFromBackboneSpec(req);
  };
  const auto first_path = run_path({{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}});
  const std::uint64_t first_primary = first_path.ok ? generation_session_id_for_result(state, first_path.value) : 0;
  if (!first_path.ok || first_primary == 0) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto second_path = run_path({{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}});
  const auto third_path = run_path({{0.0, 0.0, 0.0}, {0.0, -10.0, 0.0}});
  if (!second_path.ok || !third_path.ok) {
    return false;
  }
  const auto after_backbone = state.BuildBackboneResult();
  const auto* after_junction = find_junction(after_backbone, center_id);
  if (after_junction == nullptr || after_junction->incidents.size() < 3) {
    return false;
  }
  const auto& primary_incident = after_junction->incidents.front();
  return after_junction->prioritized_session_id == first_primary && primary_incident.primary &&
         primary_incident.order == 0 && primary_incident.source_session_id == first_primary;
}

namespace {

void register_junction_tests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C82_Junction_FirstSessionPrimary",
                         "T-junction keeps the first session on primary order", "Invariant", false,
                         test_junction_t_shape_preserves_first_session_primary_order);
  test_registry::AddTest(tests, "C83_Junction_OrderStableAcrossRebuilds",
                         "Cross junction order stays stable across rebuilds", "Invariant", false,
                         test_junction_cross_order_stable_across_rebuilds);
  test_registry::AddTest(tests, "C84_Junction_NoOverwriteByLaterPath",
                         "Later path does not overwrite the first prioritized session", "Invariant", false,
                         test_junction_first_session_priority_not_overwritten_by_later_paths);
}

WIRE_REGISTER_TEST_SUITE(register_junction_tests);

} // namespace




