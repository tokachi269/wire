#include "registry.hpp"

#include "wire/core/core_state.hpp"
#include "wire/core/inspection.hpp"
#include "wire/core/variation.hpp"
#include "wire/core/workflow_types.hpp"

#include <type_traits>

namespace {

bool test_public_headers_offer_stable_smoke_surface() {
  using namespace wire::core;

  static_assert(std::is_trivially_copyable_v<EntityRef>);
  static_assert(noexcept(EntityRef{}.valid()));
  static_assert(std::is_default_constructible_v<AddConnectionByPoleOptions>);
  static_assert(std::is_default_constructible_v<ResolveBranchPickOptions>);
  static_assert(std::is_default_constructible_v<VariationSettings>);
  static_assert(std::is_copy_constructible_v<SupportLayoutEndpointView>);

  VariationSettings settings{};
  VariationContext context{};
  context.world_position = {12.0, -4.0, 0.0};
  context.flow_key = 77;
  context.pole_id = 10;
  context.secondary_pole_id = 11;
  context.local_key = 3;
  const HierarchicalVariationSample sample = EvaluateHierarchicalVariation(settings, context);
  if (sample.flow_key != 77) {
    return false;
  }

  SupportLayoutEndpointView endpoint{};
  endpoint.down_offset_variation.final_value = 1.25;
  if (endpoint.down_offset_variation.final_value != 1.25) {
    return false;
  }

  CoreState state;
  const CoreView& view = state.view();
  const EntityRef pole_ref{EntityKind::kPole, 1};
  if (view.describe_entity(pole_ref).has_value()) {
    return false;
  }
  if (!view.collect_decision_trace(pole_ref).empty()) {
    return false;
  }
  if (view.inspect_span(1).has_value()) {
    return false;
  }
  PickResult pick{};
  if (state.ResolveBranchPick(pick).ok) {
    return false;
  }

  BackboneSpec spec{};
  spec.interval_m = 20.0;
  BackboneBundleSpec bundle{};
  bundle.bundle_template_id = BundleKind::kLowVoltage;
  spec.bundles.push_back(bundle);
  return spec.interval_m == 20.0 && spec.bundles.size() == 1;
}

void RegisterApiSurfaceTests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C171", "public headers compile as a minimal consumer surface",
                         "Invariant: public headers stay self-sufficient and concept-level views remain usable without "
                         "reaching into internal storage",
                         false, &test_public_headers_offer_stable_smoke_surface);
}

WIRE_REGISTER_TEST_SUITE(RegisterApiSurfaceTests);

} // namespace
