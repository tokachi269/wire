#include "fixtures.hpp"
#include "cases.hpp"
#include "../registry.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

namespace backbone_tests {
namespace {

std::vector<city::wire::RandomBackboneBundleRule> visual_rules() {
  using namespace city::wire;
  return {
      {kDefaultHighVoltageBundleTemplateId, 1, 1, 3, 9.2, 9.2, 0.2, 0.2, 0.25},
      {kDefaultLowVoltageBundleTemplateId, 2, 4, 1, 7.0, 7.7, 0.14, 0.52, 0.20},
      {kDefaultCommunicationBundleTemplateId, 1, 4, 1, 5.0, 5.8, 0.14, 0.50, 0.16},
      {kDefaultOpticalBundleTemplateId, 0, 2, 1, 4.9, 5.7, 0.14, 0.50, 0.16},
  };
}

city::wire::RouteBundleVariationInput variation_input(std::uint64_t seed) {
  city::wire::RouteBundleVariationInput input{};
  input.route_seed = seed;
  input.preferred_side_sign = -1;
  input.pole_type_id = 2;
  input.rules = visual_rules();
  return input;
}

bool same_specs(const std::vector<city::wire::BackboneBundleSpec>& a,
                const std::vector<city::wire::BackboneBundleSpec>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t index = 0; index < a.size(); ++index) {
    if (a[index].bundle_template_id != b[index].bundle_template_id ||
        a[index].placement_key != b[index].placement_key ||
        a[index].count != b[index].count ||
        a[index].height_m != b[index].height_m ||
        a[index].lateral_m != b[index].lateral_m ||
        a[index].spacing_m != b[index].spacing_m) return false;
  }
  return true;
}

city::wire::BackboneSpec request_with(
    city::wire::CoreState& state,
    const std::vector<city::wire::BackboneBundleSpec>& bundles,
    std::vector<city::wire::Vec3d> points = {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}}) {
  city::wire::BackboneSpec request = line_req(state);
  request.pole_type_id = 2;
  request.path.polyline = std::move(points);
  request.bundles = bundles;
  return request;
}

const city::wire::RandomBackboneBundleRule* rule_for(
    const std::vector<city::wire::RandomBackboneBundleRule>& rules,
    city::wire::BundleTemplateId id) {
  const auto found = std::ranges::find_if(rules, [id](const auto& rule) {
    return rule.bundle_template_id == id;
  });
  return found == rules.end() ? nullptr : &*found;
}

} // namespace

bool C856_route_bundle_variation_is_seed_stable_and_seed_sensitive() {
  city::wire::CoreState state;
  const auto first = state.ResolveRouteBundleVariation(variation_input(0x10203040));
  const auto repeat = state.ResolveRouteBundleVariation(variation_input(0x10203040));
  const auto other = state.ResolveRouteBundleVariation(variation_input(0x50607080));
  return first.ok && repeat.ok && other.ok && same_specs(first.value, repeat.value) &&
         !same_specs(first.value, other.value);
}

bool C857_route_bundle_variation_obeys_side_envelopes_and_spacing() {
  city::wire::CoreState state;
  const auto input = variation_input(0x12345678);
  const auto resolved = state.ResolveRouteBundleVariation(input);
  if (!resolved.ok || resolved.value.size() < 4) return false;
  bool horizontal_variation = false;
  for (std::size_t i = 0; i < resolved.value.size(); ++i) {
    const auto& spec = resolved.value[i];
    const auto* rule = rule_for(input.rules, spec.bundle_template_id);
    if (rule == nullptr || spec.lateral_m >= 0.0 ||
        -spec.lateral_m < rule->lateral_abs_min_m - 1e-12 ||
        -spec.lateral_m > rule->lateral_abs_max_m + 1e-12 ||
        spec.height_m < rule->height_min_m - 1e-12 ||
        spec.height_m > rule->height_max_m + 1e-12) return false;
    for (std::size_t j = 0; j < i; ++j) {
      const auto* other_rule = rule_for(input.rules, resolved.value[j].bundle_template_id);
      if (other_rule == nullptr) return false;
      const double required = std::max(rule->min_spacing_m, other_rule->min_spacing_m);
      const double dy = spec.lateral_m - resolved.value[j].lateral_m;
      const double dz = spec.height_m - resolved.value[j].height_m;
      if (dy * dy + dz * dz + 1e-12 < required * required) return false;
      horizontal_variation = horizontal_variation || std::abs(dy) > 1e-6;
    }
  }
  return horizontal_variation;
}

bool C858_route_bundle_variation_creates_only_requested_authoritative_topology() {
  using namespace city::wire;
  CoreState state;
  RouteBundleVariationInput input{};
  input.route_seed = 77;
  input.preferred_side_sign = -1;
  input.pole_type_id = 2;
  input.rules = {
      {kDefaultCommunicationBundleTemplateId, 2, 3, 1, 5.0, 5.8, 0.14, 0.50, 0.16},
      {kDefaultOpticalBundleTemplateId, 1, 2, 1, 4.9, 5.7, 0.14, 0.50, 0.16},
  };
  const auto resolved = state.ResolveRouteBundleVariation(input);
  if (!resolved.ok || resolved.value.empty()) return false;
  const auto generated = state.GenerateFromBackboneSpec(request_with(state, resolved.value));
  if (!generated.ok || state.view().bundles().size() != resolved.value.size() ||
      state.view().backbone().edge_bundles.size() != resolved.value.size() ||
      state.view().backbone().span_bindings.empty() || state.view().backbone().port_bindings.empty()) return false;
  return std::ranges::all_of(state.view().bundles().items(), [](const Bundle& bundle) {
    const BundleTemplateId id = bundle.bundle_template_id;
    return id == kDefaultCommunicationBundleTemplateId || id == kDefaultOpticalBundleTemplateId;
  });
}

bool C859_route_bundle_variation_failure_is_atomic() {
  city::wire::CoreState state;
  std::string before{};
  if (!state.SerializeAuthoritative(&before).ok) return false;
  auto input = variation_input(99);
  input.rules = {{city::wire::kDefaultLowVoltageBundleTemplateId,
                  2, 2, 1, 7.0, 7.0, 0.2, 0.2, 0.5}};
  const auto resolved = state.ResolveRouteBundleVariation(input);
  std::string after{};
  return !resolved.ok && resolved.error.find("cannot satisfy minimum spacing") != std::string::npos &&
         state.SerializeAuthoritative(&after).ok && before == after;
}

bool C860_route_bundle_variation_survives_save_load_and_acute_corner() {
  city::wire::CoreState state;
  const auto resolved = state.ResolveRouteBundleVariation(variation_input(0x31415926));
  if (!resolved.ok) return false;
  const auto generated = state.GenerateFromBackboneSpec(request_with(
      state, resolved.value, {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0}, {5.0, 1.5, 0.0}}));
  if (!generated.ok) return false;
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state);
  std::unordered_map<std::uint64_t, std::pair<double, double>> placements{};
  for (const auto& bundle : state.view().bundles().items()) {
    placements[bundle.placement_key] = {bundle.height_m, bundle.lateral_m};
  }
  std::string saved{};
  if (!state.SerializeAuthoritative(&saved).ok) return false;
  city::wire::CoreState loaded;
  if (!loaded.DeserializeAuthoritative(saved).ok || loaded.view().bundles().size() != placements.size()) return false;
  for (const auto& bundle : loaded.view().bundles().items()) {
    const auto found = placements.find(bundle.placement_key);
    if (found == placements.end() || found->second.first != bundle.height_m ||
        found->second.second != bundle.lateral_m) return false;
  }
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(loaded);
  return true;
}

bool C861_route_bundle_variation_incremental_extension_preserves_existing_output() {
  city::wire::CoreState state;
  const auto resolved = state.ResolveRouteBundleVariation(variation_input(0x27182818));
  if (!resolved.ok) return false;
  const auto first = state.GenerateFromBackboneSpec(request_with(state, resolved.value));
  if (!first.ok || first.value.generated_pole_ids.size() != 2 ||
      first.value.bundle_ids.size() != resolved.value.size()) return false;
  const std::vector<city::wire::ObjectId> existing_spans = first.value.generated_span_ids;
  const city::wire::ObjectId endpoint_id = first.value.generated_pole_ids.back();
  const city::wire::Pole* endpoint = state.view().poles().find(endpoint_id);
  if (endpoint == nullptr) return false;
  auto continued_bundles = resolved.value;
  for (std::size_t index = 0; index < continued_bundles.size(); ++index) {
    continued_bundles[index].source_bundle_id = first.value.bundle_ids[index];
  }
  auto extension = request_with(state, continued_bundles,
                                {endpoint->world_transform.position, {22.0, 4.0, 0.0}});
  extension.path.node_specs = {pole_spec(0, endpoint_id)};
  const auto second = state.GenerateFromBackboneSpec(extension);
  if (!second.ok || second.value.generated_span_ids.empty()) return false;
  if (!std::ranges::all_of(existing_spans, [&](city::wire::ObjectId span_id) {
        return state.view().spans().find(span_id) != nullptr;
      })) return false;
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state);
  return true;
}

} // namespace backbone_tests
