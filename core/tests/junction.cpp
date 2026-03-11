#include <vector>

#include "registry.hpp"
#include "helpers.hpp"

using namespace helpers;

bool test_junction_t_shape_preserves_first_session_primary_order() {
  CoreState state;
  const auto type_ids = sorted_pole_type_ids(state);
  if (type_ids.empty()) {
    return false;
  }
  auto run_path = [&](const std::vector<wire::core::Vec3d>& path) -> std::pair<bool, std::uint64_t> {
    wire::core::BackboneSpec req{};
    req.path.polyline = path;
    req.interval_m = 8.0;
    req.pole_type_id = type_ids.front();
    wire::core::BackboneBundleSpec b{};
    b.bundle_template_id = wire::core::BundleKind::kLowVoltage;
    req.bundles.push_back(b);
    const auto r = state.GenerateFromBackboneSpec(req);
    if (!r.ok || r.value.generated_span_ids.empty()) {
      return {false, 0};
    }
    const auto* span = state.view().edit_state().spans.find(r.value.generated_span_ids.front());
    return {span != nullptr, (span == nullptr) ? 0 : span->generation.generation_session_id};
  };

  const auto first = run_path({{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}});
  const auto second = run_path({{0.0, -12.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 12.0, 0.0}});
  if (!first.first || !second.first || first.second == 0 || second.second == 0 || first.second == second.second) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const wire::core::BackboneResult backbone = state.BuildBackboneResult();
  const auto* junction = find_junction(backbone, center_id);
  if (junction == nullptr) {
    return false;
  }
  bool first_is_primary = false;
  bool second_not_primary = true;
  for (const auto& inc : junction->incidents) {
    if (inc.source_session_id == first.second && inc.order == 0) {
      first_is_primary = true;
    }
    if (inc.source_session_id == second.second && inc.order == 0) {
      second_not_primary = false;
    }
  }
  return first_is_primary && second_not_primary;
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
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }

  const wire::core::BackboneResult a = state.BuildBackboneResult();
  const wire::core::BackboneResult b = state.BuildBackboneResult();
  const auto* ja = find_junction(a, center_id);
  const auto* jb = find_junction(b, center_id);
  if (ja == nullptr || jb == nullptr || ja->incidents.size() != jb->incidents.size()) {
    return false;
  }
  for (std::size_t i = 0; i < ja->incidents.size(); ++i) {
    if (ja->incidents[i].neighbor_node_id != jb->incidents[i].neighbor_node_id ||
        ja->incidents[i].order != jb->incidents[i].order ||
        ja->incidents[i].source_session_id != jb->incidents[i].source_session_id) {
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
  auto run_path = [&](const std::vector<wire::core::Vec3d>& path) -> std::uint64_t {
    wire::core::BackboneSpec req{};
    req.path.polyline = path;
    req.interval_m = 8.0;
    req.pole_type_id = type_ids.front();
    wire::core::BackboneBundleSpec b{};
    b.bundle_template_id = wire::core::BundleKind::kLowVoltage;
    req.bundles.push_back(b);
    const auto r = state.GenerateFromBackboneSpec(req);
    if (!r.ok || r.value.generated_span_ids.empty()) {
      return 0;
    }
    const auto* span = state.view().edit_state().spans.find(r.value.generated_span_ids.front());
    return (span == nullptr) ? 0 : span->generation.generation_session_id;
  };
  const std::uint64_t first_session = run_path({{-12.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}});
  (void)run_path({{0.0, 0.0, 0.0}, {0.0, 10.0, 0.0}});
  (void)run_path({{0.0, 0.0, 0.0}, {0.0, -10.0, 0.0}});
  if (first_session == 0) {
    return false;
  }
  const ObjectId center_id = find_pole_id_by_position(state, {0.0, 0.0, 0.0});
  if (center_id == wire::core::kInvalidObjectId) {
    return false;
  }
  const auto backbone = state.BuildBackboneResult();
  const auto* junction = find_junction(backbone, center_id);
  return junction != nullptr && junction->prioritized_session_id == first_session;
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




