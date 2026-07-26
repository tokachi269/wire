#include "registry.hpp"

#include "city/wire/core_state.hpp"
#include "city/wire/core_state_api_types.hpp"
#include "city/wire/core_view.hpp"
#include "city/wire/inspection.hpp"
#include "city/wire/model_descriptor.hpp"
#include "city/wire/style_context.hpp"
#include "city/wire/variation.hpp"
#include "city/wire/workflow_types.hpp"

#include <type_traits>

namespace {

bool test_public_headers_offer_stable_smoke_surface() {
  using namespace city::wire;

  static_assert(std::is_trivially_copyable_v<EntityRef>);
  static_assert(noexcept(EntityRef{}.valid()));
  static_assert(std::is_default_constructible_v<ResolveBranchPickOptions>);
  static_assert(std::is_default_constructible_v<VariationSettings>);
  static_assert(std::is_default_constructible_v<ContextProfile>);
  static_assert(std::is_default_constructible_v<ResolvedStyleContext>);
  static_assert(std::is_copy_constructible_v<SpanLayoutView>);
  static_assert(std::is_copy_constructible_v<SpanLayoutState>);

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

  SpanLayoutState layout_state{};

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
  bundle.bundle_template_id = kDefaultLowVoltageBundleTemplateId;
  spec.bundles.push_back(bundle);

  ModelMeasurement measurement{};
  measurement.name = "consumer-smoke";
  measurement.sockets.push_back(ModelSocket{"line_in", ModelSocketRole::kLineIn, {}, {1.0, 0.0, 0.0}});
  measurement.sockets.push_back(ModelSocket{"line_out", ModelSocketRole::kLineOut, {}, {1.0, 0.0, 0.0}});
  const ModelMergeResult merged = merge(measurement, ModelOverride{});
  const ModelAttachmentTemplateBuildResult attachment = build_attachment_template(merged.descriptor, 1);
  if (!merged.report.conflicts.empty() || attachment.attachment_template.sockets.size() != 2) {
    return false;
  }

  ContextProfile profile{};
  profile.style_seed = 7;
  StyleRouteKey route_key{};
  route_key.family_id = 11;
  StyleObjectKey object_key{};
  object_key.route = route_key;
  object_key.segment_index = 2;
  const ResolvedStyleContext resolved = ResolveStyleContext(profile, route_key, object_key);
  ResolvedStyleContext style{};
  style.profile = profile;
  style.route.key.family_id = 11;
  style.object.key.segment_index = 2;
  return spec.interval_m == 20.0 && spec.bundles.size() == 1 && style.profile.style_seed == 7 &&
         style.route.key.family_id == 11 && style.object.key.segment_index == 2 &&
         resolved.profile.style_seed == 7 && resolved.route.key.family_id == 11 &&
         resolved.object.key.segment_index == 2;
}

void RegisterApiSurfaceTests(test_registry::TestRegistry& tests) {
  test_registry::AddTest(tests, "C171", "public headers compile as a minimal consumer surface",
                         "Invariant: public headers stay self-sufficient and concept-level views remain usable without "
                         "reaching into internal storage",
                         false, &test_public_headers_offer_stable_smoke_surface);
}

WIRE_REGISTER_TEST_SUITE(RegisterApiSurfaceTests);

} // namespace
