#include "fixtures.hpp"
#include "cases.hpp"
#include "../registry.hpp"
#include "city/wire/coord_utils.hpp"
#include "city/wire/core_test_hook.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace backbone_tests {
namespace {

std::vector<city::wire::RandomBackboneBundleRule> visual_rules() {
  using namespace city::wire;
  return {
      {kDefaultHighVoltageBundleTemplateId, 1, 1, 3, 9.2, 9.2, 0.2, 0.2},
      {kDefaultLowVoltageBundleTemplateId, 2, 4, 1, 7.0, 7.7, 0.12, 0.52},
      {kDefaultCommunicationBundleTemplateId, 1, 4, 1, 5.0, 5.8, 0.12, 0.50},
      {kDefaultOpticalBundleTemplateId, 1, 2, 1, 4.9, 5.7, 0.12, 0.50},
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

bool configure_non_hv_model_fixtures(city::wire::CoreState* state) {
  using namespace city::wire;
  constexpr ModelAssemblyTemplateId kRowAssembly = 9891;
  constexpr ModelAssemblyTemplateId kEndpointAssembly = 9892;
  ModelAssemblyTemplate row{};
  row.id = kRowAssembly;
  ModelAssemblyPart row_part{};
  row_part.part_id = 1;
  row_part.model_key = "test_non_hv_belt";
  row_part.descriptor_version = 1;
  row_part.fit_mode = ModelFitMode::kPoleRadial;
  row.parts.push_back(row_part);
  ModelAssemblyTemplate endpoint{};
  endpoint.id = kEndpointAssembly;
  ModelAssemblyPart endpoint_part{};
  endpoint_part.part_id = 1;
  endpoint_part.model_key = "test_non_hv_clamp";
  endpoint_part.descriptor_version = 1;
  endpoint_part.fit_mode = ModelFitMode::kPoleSurface;
  endpoint_part.sockets.push_back({"wire", {0.0, 0.18, 0.0}, {1.0, 0.0, 0.0}});
  endpoint.parts.push_back(endpoint_part);
  endpoint.wire_socket = AssemblySocketRef{1, "wire"};
  if (!state->RegisterModelAssemblyTemplate(row).ok ||
      !state->RegisterModelAssemblyTemplate(endpoint).ok) return false;
  for (BundleTemplateId id : {kDefaultLowVoltageBundleTemplateId,
                              kDefaultCommunicationBundleTemplateId,
                              kDefaultOpticalBundleTemplateId}) {
    BundleTemplate edited = state->view().bundle_templates().at(id);
    edited.row_fixture_assembly_id = kRowAssembly;
    edited.endpoint_fixture_assembly_id = kEndpointAssembly;
    if (!state->UpdateBundleTemplate(edited).ok) return false;
  }
  return true;
}

std::vector<city::wire::BackboneBundleSpec> support_fixture_specs() {
  using namespace city::wire;
  return {
      {kDefaultCommunicationBundleTemplateId, 1001, SpanLayer::kCommunication, 1, true, 5.10, -0.30, 0.20},
      {kDefaultCommunicationBundleTemplateId, 1002, SpanLayer::kCommunication, 1, true, 5.25, -0.40, 0.20},
      {kDefaultOpticalBundleTemplateId, 1003, SpanLayer::kOptical, 1, true, 5.35, -0.50, 0.20},
  };
}

std::vector<const city::wire::VisualCurvePart*> edge_bodies(
    const city::wire::CoreState& state, city::wire::BundleTemplateId template_id) {
  std::vector<const city::wire::VisualCurvePart*> out{};
  for (const auto& part : state.view().visual_curve_parts().parts) {
    if (part.kind == city::wire::VisualCurvePartKind::kEdgeBody &&
        part.bundle_template_id == template_id) out.push_back(&part);
  }
  std::sort(out.begin(), out.end(), [](const auto* a, const auto* b) {
    return std::tie(a->lane_index, a->source_span_id, a->section_key.instance_index) <
           std::tie(b->lane_index, b->source_span_id, b->section_key.instance_index);
  });
  return out;
}

bool same_samples(const std::vector<city::wire::Vec3d>& a,
                  const std::vector<city::wire::Vec3d>& b) {
  if (a.size() != b.size()) return false;
  for (std::size_t index = 0; index < a.size(); ++index) {
    if (a[index].x != b[index].x || a[index].y != b[index].y ||
        a[index].z != b[index].z) return false;
  }
  return true;
}

bool same_point(const city::wire::Vec3d& a, const city::wire::Vec3d& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}

city::wire::Vec3d normalized(city::wire::Vec3d value) {
  city::wire::Normalize(&value);
  return value;
}

double polyline_length(const std::vector<city::wire::Vec3d>& samples) {
  double length = 0.0;
  for (std::size_t index = 1; index < samples.size(); ++index) {
    length += city::wire::Length(samples[index] - samples[index - 1]);
  }
  return length;
}

std::pair<city::wire::Vec3d, city::wire::Vec3d> sample_polyline(
    const std::vector<city::wire::Vec3d>& samples, double distance) {
  if (samples.size() < 2) return {};
  distance = std::clamp(distance, 0.0, polyline_length(samples));
  double accumulated = 0.0;
  for (std::size_t index = 1; index < samples.size(); ++index) {
    const city::wire::Vec3d segment = samples[index] - samples[index - 1];
    const double length = city::wire::Length(segment);
    if (distance <= accumulated + length || index + 1 == samples.size()) {
      const double fraction = length <= 1e-12 ? 0.0 :
          (distance - accumulated) / length;
      return {samples[index - 1] + city::wire::ScaleVec(
                  segment, std::clamp(fraction, 0.0, 1.0)),
              normalized(segment)};
    }
    accumulated += length;
  }
  return {samples.back(), normalized(samples.back() - samples[samples.size() - 2])};
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

bool C857_route_bundle_variation_obeys_side_envelopes() {
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
      horizontal_variation = horizontal_variation ||
          std::abs(spec.lateral_m - resolved.value[j].lateral_m) > 1e-6;
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
      {kDefaultCommunicationBundleTemplateId, 2, 3, 1, 5.0, 5.8, 0.12, 0.50},
      {kDefaultOpticalBundleTemplateId, 1, 2, 1, 4.9, 5.7, 0.12, 0.50},
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

bool C882_backbone_variation_descriptor_is_atomically_associated_and_persisted() {
  using namespace city::wire;
  CoreState state;
  BackboneSpec request = request_with(state, {});
  const RouteBundleVariationInput descriptor = variation_input(0x8820102);
  const auto generated =
      state.GenerateBackboneBundleVariation(request, descriptor);
  const SavedBackboneBundleVariation* saved_descriptor =
      generated.ok
          ? state.view().backbone_bundle_variation(
                generated.value.variation_id)
          : nullptr;
  WIRE_TEST_EXPECT_ANCHOR(
      generated.ok && saved_descriptor != nullptr &&
          saved_descriptor->descriptor.route_seed == descriptor.route_seed &&
          saved_descriptor->descriptor.rules.size() == descriptor.rules.size() &&
          saved_descriptor->instances.size() ==
              generated.value.generation.bundle_ids.size(),
      generated.error.empty()
          ? "variation descriptor was not associated with exact generated scope"
          : generated.error);

  std::map<std::uint64_t, ObjectId> identities{};
  for (const auto& instance : saved_descriptor->instances) {
    const Bundle* bundle = state.view().bundles().find(instance.bundle_id);
    WIRE_TEST_EXPECT_PRESENCE(
        bundle != nullptr && bundle->placement_key == instance.placement_key,
        "variation descriptor instance does not reference exact concrete Bundle");
    identities.emplace(instance.placement_key, instance.bundle_id);
  }
  std::string serialized{};
  WIRE_TEST_EXPECT_PRESENCE(state.SerializeAuthoritative(&serialized).ok,
                            "variation descriptor save failed");
  CoreState loaded;
  WIRE_TEST_EXPECT_PRESENCE(loaded.DeserializeAuthoritative(serialized).ok,
                            "variation descriptor load failed");
  const SavedBackboneBundleVariation* loaded_descriptor =
      loaded.view().backbone_bundle_variation(generated.value.variation_id);
  std::string resaved{};
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      loaded_descriptor != nullptr &&
          loaded_descriptor->descriptor.route_seed == descriptor.route_seed &&
          loaded_descriptor->instances.size() == identities.size() &&
          loaded.SerializeAuthoritative(&resaved).ok && serialized == resaved,
      "variation descriptor did not survive save-load-save without reroll");
  for (const auto& instance : loaded_descriptor->instances) {
    const auto expected = identities.find(instance.placement_key);
    WIRE_TEST_EXPECT_ANCHOR(
        expected != identities.end() && expected->second == instance.bundle_id,
        "load changed a persisted variation placement identity");
  }

  CoreState malformed = loaded;
  auto& malformed_variations =
      CoreStateTestHook::backbone_bundle_variations(malformed);
  if (malformed_variations.empty()) return false;
  malformed_variations.front().descriptor.rules.clear();
  const ValidationResult malformed_validation =
      CoreStateTestHook::validate(malformed);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      !malformed_validation.ok() &&
          std::ranges::any_of(
              malformed_validation.issues, [](const ValidationIssue& issue) {
                return issue.code ==
                       "BackboneBundleVariationDescriptorInvalid";
              }),
      "Validate accepted a malformed persisted variation descriptor");

  CoreState changed_resolver_input = loaded;
  auto& changed_variations =
      CoreStateTestHook::backbone_bundle_variations(changed_resolver_input);
  changed_variations.front().descriptor.route_seed ^= 0x5a5a5a5aULL;
  const ValidationResult changed_seed_validation =
      CoreStateTestHook::validate(changed_resolver_input);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      changed_seed_validation.ok(),
      "persistent validation reran the current resolver against saved concrete instances");

  std::string malformed_archive = serialized;
  malformed_archive.replace(0, std::string("wire_state_v5").size(),
                            "wire_state_v4");
  CoreState load_target = loaded;
  std::string load_before{};
  std::string load_after{};
  WIRE_TEST_EXPECT_PRESENCE(
      load_target.SerializeAuthoritative(&load_before).ok,
      "malformed variation load fixture save failed");
  const auto malformed_load =
      load_target.DeserializeAuthoritative(malformed_archive);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      !malformed_load.ok &&
          load_target.SerializeAuthoritative(&load_after).ok &&
          load_before == load_after,
      "Deserialize accepted an obsolete schema or mutated state");

  CoreState failed;
  std::string before{};
  WIRE_TEST_EXPECT_PRESENCE(failed.SerializeAuthoritative(&before).ok,
                            "variation failure fixture save failed");
  RouteBundleVariationInput impossible = descriptor;
  impossible.rules = {{kDefaultLowVoltageBundleTemplateId, 2, 1, 1,
                       7.0, 7.0, 0.2, 0.2}};
  const auto rejected = failed.GenerateBackboneBundleVariation(
      request_with(failed, {}), impossible);
  std::string after{};
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      !rejected.ok && failed.SerializeAuthoritative(&after).ok &&
          before == after,
      "failed variation generation partially persisted descriptor or topology");
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(loaded);
  return true;
}

bool C883_backbone_variation_apply_reconciles_concrete_scope_atomically() {
  using namespace city::wire;
  CoreState state;
  RouteBundleVariationInput initial{};
  initial.route_seed = 0x8830102;
  initial.preferred_side_sign = -1;
  initial.pole_type_id = 2;
  initial.rules = {{kDefaultCommunicationBundleTemplateId, 1, 1, 1,
                    5.1, 5.1, 0.20, 0.20}};
  const auto generated = state.GenerateBackboneBundleVariation(
      request_with(state, {}), initial);
  if (!generated.ok) return false;
  const ObjectId variation_id = generated.value.variation_id;
  const SavedBackboneBundleVariation* before =
      state.view().backbone_bundle_variation(variation_id);
  if (before == nullptr || before->instances.size() != 1) return false;
  const std::uint64_t surviving_key = before->instances.front().placement_key;
  const ObjectId surviving_id = before->instances.front().bundle_id;

  std::string neutral_before{};
  std::string neutral_after{};
  WIRE_TEST_EXPECT_PRESENCE(
      state.SerializeAuthoritative(&neutral_before).ok,
      "neutral variation Apply fixture save failed");
  const auto neutral =
      state.ApplyBackboneBundleVariation(variation_id, initial);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      neutral.ok && state.SerializeAuthoritative(&neutral_after).ok &&
          neutral_before == neutral_after,
      neutral.error.empty()
          ? "neutral variation Apply changed authoritative state"
          : neutral.error);

  RouteBundleVariationInput denser = initial;
  denser.rules.front().min_instances = 2;
  denser.rules.front().max_instances = 2;
  denser.rules.front().height_min_m = 5.25;
  denser.rules.front().height_max_m = 5.25;
  denser.rules.front().lateral_abs_min_m = 0.30;
  denser.rules.front().lateral_abs_max_m = 0.60;
  const auto applied = state.ApplyBackboneBundleVariation(variation_id, denser);
  const SavedBackboneBundleVariation* after =
      state.view().backbone_bundle_variation(variation_id);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      applied.ok && applied.value && after != nullptr &&
          after->instances.size() == 2,
      applied.error.empty() ? "variation density Apply did not add one instance"
                            : applied.error);
  const auto survivor = std::ranges::find_if(
      after->instances, [surviving_key](const auto& instance) {
        return instance.placement_key == surviving_key;
      });
  const Bundle* surviving_bundle =
      survivor == after->instances.end()
          ? nullptr
          : state.view().bundles().find(survivor->bundle_id);
  WIRE_TEST_EXPECT_ANCHOR(
      survivor != after->instances.end() && survivor->bundle_id == surviving_id &&
          surviving_bundle != nullptr && surviving_bundle->height_m == 5.25 &&
          surviving_bundle->lateral_m <= -0.30,
      "variation Apply did not preserve survivor identity while updating placement");

  std::string manual_before{};
  WIRE_TEST_EXPECT_PRESENCE(state.SerializeAuthoritative(&manual_before).ok,
                            "manual-edit boundary snapshot save failed");
  const auto manual = state.UpdateBackboneBundlePlacement(
      surviving_id, true, 5.4, -0.4, 0.2);
  std::string manual_after{};
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      !manual.ok && state.SerializeAuthoritative(&manual_after).ok &&
          manual_before == manual_after,
      "recipe-backed Bundle accepted an individual placement edit");

  std::string stable_before{};
  WIRE_TEST_EXPECT_PRESENCE(state.SerializeAuthoritative(&stable_before).ok,
                            "variation Apply failure snapshot save failed");
  RouteBundleVariationInput impossible = denser;
  impossible.rules.front().min_instances = 3;
  impossible.rules.front().max_instances = 3;
  impossible.rules.front().height_min_m = 5.0;
  impossible.rules.front().height_max_m = 5.0;
  impossible.rules.front().lateral_abs_min_m = 0.2;
  impossible.rules.front().lateral_abs_max_m = 0.2;
  impossible.rules.front().max_instances = 2;
  const auto rejected =
      state.ApplyBackboneBundleVariation(variation_id, impossible);
  std::string stable_after{};
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      !rejected.ok && state.SerializeAuthoritative(&stable_after).ok &&
          stable_before == stable_after,
      "failed variation Apply changed descriptor or physical authority");

  CoreState fixed_state;
  RouteBundleVariationInput fixed_descriptor{};
  fixed_descriptor.route_seed = 0x8830201;
  fixed_descriptor.preferred_side_sign = -1;
  fixed_descriptor.pole_type_id = 1;
  fixed_descriptor.rules = {{kDefaultLowVoltageBundleTemplateId, 1, 1, 1,
                             7.1, 7.1, 0.25, 0.25}};
  BackboneSpec fixed_request = line_req(fixed_state);
  fixed_request.pole_type_id = 1;
  fixed_request.bundles.clear();
  const auto fixed_generated = fixed_state.GenerateBackboneBundleVariation(
      fixed_request, fixed_descriptor);
  if (!fixed_generated.ok) return false;
  fixed_descriptor.rules.front().height_min_m = 7.5;
  fixed_descriptor.rules.front().height_max_m = 7.5;
  const auto fixed_applied = fixed_state.ApplyBackboneBundleVariation(
      fixed_generated.value.variation_id, fixed_descriptor);
  WIRE_TEST_EXPECT_ANCHOR(
      fixed_applied.ok,
      fixed_applied.error.empty()
          ? "fixed-count variation Apply did not use the template count contract"
          : fixed_applied.error);
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state);
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(fixed_state);
  return true;
}

bool C884_backbone_variation_zero_count_retires_and_readd_is_unsupported() {
  using namespace city::wire;
  CoreState state;
  RouteBundleVariationInput one{};
  one.route_seed = 0x8840102;
  one.preferred_side_sign = -1;
  one.pole_type_id = 2;
  one.rules = {{kDefaultCommunicationBundleTemplateId, 1, 1, 1,
                5.2, 5.2, 0.25, 0.25}};
  const auto generated = state.GenerateBackboneBundleVariation(
      request_with(state, {},
                   {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0},
                    {18.0, 3.0, 0.0}}),
      one);
  if (!generated.ok || generated.value.generation.bundle_ids.size() != 1) {
    return false;
  }
  const ObjectId variation_id = generated.value.variation_id;
  const ObjectId retired_bundle_id =
      generated.value.generation.bundle_ids.front();

  RouteBundleVariationInput zero = one;
  zero.rules.front().min_instances = 0;
  zero.rules.front().max_instances = 0;
  const auto removed = state.ApplyBackboneBundleVariation(variation_id, zero);
  const SavedBackboneBundleVariation* empty =
      state.view().backbone_bundle_variation(variation_id);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      removed.ok && removed.value && empty != nullptr &&
          empty->instances.empty() &&
          state.view().bundles().find(retired_bundle_id) == nullptr &&
          state.view().backbone().edge_bundles.empty() &&
          state.view().backbone().edges.empty() &&
          state.view().backbone().row_continuities.empty(),
      removed.error.empty()
          ? "zero-count Apply retained live or dormant Bundle topology"
          : removed.error);

  std::string zero_archive{};
  CoreState loaded;
  WIRE_TEST_EXPECT_ANCHOR(
      state.SerializeAuthoritative(&zero_archive).ok &&
          loaded.DeserializeAuthoritative(zero_archive).ok &&
          loaded.view().backbone_bundle_variation(variation_id) != nullptr &&
          loaded.view().backbone_bundle_variation(variation_id)
              ->instances.empty() &&
          loaded.view().backbone().edges.empty(),
      "current zero-count variation did not roundtrip without replay topology");

  std::string before_readd{};
  std::string after_readd{};
  if (!loaded.SerializeAuthoritative(&before_readd).ok) return false;
  const auto rejected =
      loaded.ApplyBackboneBundleVariation(variation_id, one);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      !rejected.ok &&
          rejected.error.find("zero-instance variation has no live exact membership source") !=
              std::string::npos &&
          loaded.SerializeAuthoritative(&after_readd).ok &&
          before_readd == after_readd,
      "zero-to-one Apply inferred deleted membership or changed authority");
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(loaded);
  return true;
}

bool C885_backbone_variation_apply_preserves_branch_cross_sharp_membership() {
  using namespace city::wire;
  CoreState state;
  RouteBundleVariationInput one{};
  one.route_seed = 0x8850102;
  one.preferred_side_sign = -1;
  one.pole_type_id = 2;
  one.rules = {{kDefaultCommunicationBundleTemplateId, 1, 1, 1,
                5.2, 5.2, 0.25, 0.25}};
  const auto generated = state.GenerateBackboneBundleVariation(
      request_with(state, {},
                   {{0.0, 0.0, 0.0}, {10.0, 0.0, 0.0},
                    {5.0, 8.660254037844386, 0.0}}),
      one);
  if (!generated.ok || generated.value.generation.generated_pole_ids.size() != 3 ||
      generated.value.generation.bundle_ids.size() != 1) return false;
  const ObjectId variation_id = generated.value.variation_id;
  const ObjectId bundle_id = generated.value.generation.bundle_ids.front();
  const Bundle* bundle = state.view().bundles().find(bundle_id);
  const Pole* junction = state.view().poles().find(
      generated.value.generation.generated_pole_ids[1]);
  if (bundle == nullptr || junction == nullptr) return false;
  BackboneBundleSpec concrete{};
  concrete.bundle_template_id = bundle->bundle_template_id;
  concrete.placement_key = bundle->placement_key;
  concrete.layer = SpanLayer::kCommunication;
  concrete.count = bundle->conductor_count;
  concrete.placement_explicit = true;
  concrete.height_m = bundle->height_m;
  concrete.lateral_m = bundle->lateral_m;
  concrete.spacing_m = bundle->spacing_override_m;
  concrete.existing_bundle_id = bundle_id;
  BackboneSpec crossing = request_with(
      state, {concrete},
      {{10.0, -8.0, 0.0}, junction->world_transform.position,
       {18.0, 2.0, 0.0}});
  crossing.path.node_specs = {
      pole_spec(1, generated.value.generation.generated_pole_ids[1])};
  std::string partial_before{};
  std::string partial_after{};
  WIRE_TEST_EXPECT_PRESENCE(
      state.SerializeAuthoritative(&partial_before).ok,
      "partial variation Extend fixture save failed");
  const auto partial =
      state.ExtendBackboneBundleVariation(variation_id, crossing);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      !partial.ok && state.SerializeAuthoritative(&partial_after).ok &&
          partial_before == partial_after,
      "variation Extend accepted caller-owned partial Bundle membership");
  crossing.bundles.clear();
  const auto extended =
      state.ExtendBackboneBundleVariation(variation_id, crossing);
  const SavedBackboneBundleVariation* extended_descriptor =
      state.view().backbone_bundle_variation(variation_id);
  if (!extended.ok || extended_descriptor == nullptr ||
      extended_descriptor->instances.size() != 1) return false;
  const ObjectId anchor_bundle_id =
      extended_descriptor->instances.front().bundle_id;
  std::set<ObjectId> anchor_edge_bundle_ids{};
  for (const SavedBackboneEdgeBundle& edge_bundle :
       state.view().backbone().edge_bundles) {
    if (edge_bundle.bundle_id == anchor_bundle_id) {
      anchor_edge_bundle_ids.insert(edge_bundle.edge_bundle_id);
    }
  }
  const std::size_t expected_edges = anchor_edge_bundle_ids.size();
  const std::size_t expected_continuities = static_cast<std::size_t>(
      std::count_if(state.view().backbone().row_continuities.begin(),
                    state.view().backbone().row_continuities.end(),
                    [&](const SavedBackboneRowContinuity& continuity) {
                      return anchor_edge_bundle_ids.contains(
                                 continuity.a.edge_bundle_id) &&
                             anchor_edge_bundle_ids.contains(
                                 continuity.b.edge_bundle_id);
                    }));
  if (expected_edges < 4 || expected_continuities == 0) return false;

  RouteBundleVariationInput two = one;
  two.rules.front().min_instances = 2;
  two.rules.front().max_instances = 2;
  two.rules.front().lateral_abs_max_m = 0.60;
  const auto applied = state.ApplyBackboneBundleVariation(variation_id, two);
  const SavedBackboneBundleVariation* final =
      state.view().backbone_bundle_variation(variation_id);
  WIRE_TEST_EXPECT_ANCHOR(
      applied.ok && final != nullptr && final->instances.size() == 2,
      applied.error.empty() ? "branch/cross/sharp variation Apply failed"
                            : applied.error);
  for (const SavedBackboneBundleVariationInstance& instance : final->instances) {
    std::set<ObjectId> edge_bundle_ids{};
    for (const SavedBackboneEdgeBundle& edge_bundle :
         state.view().backbone().edge_bundles) {
      if (edge_bundle.bundle_id == instance.bundle_id) {
        edge_bundle_ids.insert(edge_bundle.edge_bundle_id);
      }
    }
    const std::size_t continuity_count = static_cast<std::size_t>(
        std::count_if(state.view().backbone().row_continuities.begin(),
                      state.view().backbone().row_continuities.end(),
                      [&](const SavedBackboneRowContinuity& continuity) {
                        return edge_bundle_ids.contains(
                                   continuity.a.edge_bundle_id) &&
                               edge_bundle_ids.contains(
                                   continuity.b.edge_bundle_id);
                      }));
    WIRE_TEST_EXPECT_DIFFERENTIAL(
        edge_bundle_ids.size() == expected_edges &&
            continuity_count == expected_continuities,
        "branch/cross/sharp exact membership was not cloned");
  }
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state);
  return true;
}

bool C886_backbone_variation_placement_identity_is_scope_local() {
  using namespace city::wire;
  CoreState state;
  RouteBundleVariationInput descriptor{};
  descriptor.route_seed = 0x8860102;
  descriptor.preferred_side_sign = -1;
  descriptor.pole_type_id = 2;
  descriptor.rules = {{kDefaultCommunicationBundleTemplateId, 1, 1, 1,
                       5.2, 5.2, 0.25, 0.25}};
  const auto first = state.GenerateBackboneBundleVariation(
      request_with(state, {}, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}}),
      descriptor);
  const auto second = state.GenerateBackboneBundleVariation(
      request_with(state, {}, {{0.0, 20.0, 0.0}, {12.0, 20.0, 0.0}}),
      descriptor);
  if (!first.ok || !second.ok) return false;
  const SavedBackboneBundleVariation* first_saved =
      state.view().backbone_bundle_variation(first.value.variation_id);
  const SavedBackboneBundleVariation* second_saved =
      state.view().backbone_bundle_variation(second.value.variation_id);
  if (first_saved == nullptr || second_saved == nullptr ||
      first_saved->instances.size() != 1 || second_saved->instances.size() != 1) {
    return false;
  }
  const auto first_instance = first_saved->instances.front();
  const auto second_instance = second_saved->instances.front();
  WIRE_TEST_EXPECT_ANCHOR(
      first_instance.placement_key == second_instance.placement_key &&
          first_instance.bundle_id != second_instance.bundle_id,
      "same-seed variations did not keep placement correlation scope-local");

  RouteBundleVariationInput changed = descriptor;
  changed.rules.front().height_min_m = 5.6;
  changed.rules.front().height_max_m = 5.6;
  const auto applied =
      state.ApplyBackboneBundleVariation(first.value.variation_id, changed);
  const Bundle* first_bundle =
      state.view().bundles().find(first_instance.bundle_id);
  const Bundle* second_bundle =
      state.view().bundles().find(second_instance.bundle_id);
  const SavedBackboneBundleVariation* second_after =
      state.view().backbone_bundle_variation(second.value.variation_id);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      applied.ok && first_bundle != nullptr && second_bundle != nullptr &&
          first_bundle->height_m == 5.6 && second_bundle->height_m == 5.2 &&
          second_after != nullptr &&
          second_after->descriptor.route_seed == descriptor.route_seed &&
          second_after->instances.front().bundle_id == second_instance.bundle_id,
      applied.error.empty()
          ? "Apply crossed variation scope through a shared placement_key"
          : applied.error);
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state);
  return true;
}

bool C862_non_hv_route_variation_covers_each_category_and_keeps_hv_fixed() {
  using namespace city::wire;
  CoreState state;
  std::map<BundleTemplateId, std::set<std::string>> signatures{};
  for (std::uint64_t seed = 1; seed <= 32; ++seed) {
    const auto resolved = state.ResolveRouteBundleVariation(variation_input(seed));
    if (!resolved.ok) return false;
    for (const BackboneBundleSpec& spec : resolved.value) {
      if (spec.bundle_template_id == kDefaultHighVoltageBundleTemplateId) {
        if (spec.count != 3 || spec.height_m != 9.2 || spec.lateral_m != -0.2 ||
            spec.spacing_m != 0.45) return false;
        continue;
      }
      std::ostringstream signature;
      signature << spec.count << ':' << spec.height_m << ':' << spec.lateral_m;
      signatures[spec.bundle_template_id].insert(signature.str());
      if (spec.bundle_template_id == kDefaultCommunicationBundleTemplateId) {
        if (spec.count != 1) return false;
      }
    }
  }
  return signatures[kDefaultLowVoltageBundleTemplateId].size() > 1 &&
         signatures[kDefaultCommunicationBundleTemplateId].size() > 1 &&
         signatures[kDefaultOpticalBundleTemplateId].size() > 1;
}

bool C863_non_hv_support_rows_are_shared_and_direct_attachment_stays_direct() {
  using namespace city::wire;
  CoreState supported;
  if (!configure_non_hv_model_fixtures(&supported)) return false;
  const auto generated = supported.GenerateFromBackboneSpec(
      request_with(supported, support_fixture_specs()));
  if (!generated.ok) return false;
  std::vector<const VisualModelInstance*> rows{};
  std::size_t belt_count = 0;
  std::size_t supported_clamp_count = 0;
  for (const auto& model : supported.view().visual_model_instances().instances) {
    if (model.model_key == "non_hv_support_row_proxy") rows.push_back(&model);
    if (model.model_key == "test_non_hv_belt") ++belt_count;
    if (model.model_key == "test_non_hv_clamp") ++supported_clamp_count;
  }
  if (rows.size() != generated.value.generated_pole_ids.size() ||
      belt_count != 0 ||
      supported_clamp_count != generated.value.generated_pole_ids.size() * support_fixture_specs().size() ||
      !std::ranges::all_of(rows, [](const auto* row) { return row->world_transform.scale.y >= 0.34; })) {
    return false;
  }

  CoreState direct;
  if (!configure_non_hv_model_fixtures(&direct)) return false;
  BackboneBundleSpec direct_spec{kDefaultCommunicationBundleTemplateId, 2001,
      SpanLayer::kCommunication, 1, true, 5.2, -0.14, 0.20};
  const auto direct_generated = direct.GenerateFromBackboneSpec(
      request_with(direct, {direct_spec}));
  if (!direct_generated.ok) return false;
  std::size_t clamp_count = 0;
  for (const VisualModelInstance& model : direct.view().visual_model_instances().instances) {
    if (model.model_key != "test_non_hv_clamp") continue;
    ++clamp_count;
    bool near_pole_surface = false;
    for (ObjectId pole_id : direct_generated.value.generated_pole_ids) {
      const Pole* pole = direct.view().poles().find(pole_id);
      if (pole == nullptr) return false;
      const PoleFrame frame = BuildPoleFrame(pole->world_transform, 0.0);
      const Vec3d local = WorldPointToLocal(frame, model.world_transform.position);
      const double radius = direct.view().pole_radius_at_height_m(*pole, local.z);
      if (std::hypot(local.x, local.y) <= radius + 0.18 + 1e-9) {
        near_pole_surface = true;
        break;
      }
    }
    if (!near_pole_surface) return false;
  }
  if (clamp_count != direct_generated.value.generated_pole_ids.size() ||
      std::ranges::any_of(direct.view().visual_model_instances().instances,
                          [](const VisualModelInstance& model) {
        return model.model_key == "non_hv_support_row_proxy";
      })) return false;

  CoreState single_member;
  CoreState four_members;
  if (!configure_non_hv_model_fixtures(&single_member) ||
      !configure_non_hv_model_fixtures(&four_members)) return false;
  BundleTemplate single_template =
      single_member.view().bundle_templates().at(kDefaultCommunicationBundleTemplateId);
  single_template.span_visual_assembly.visual_member_count = 1;
  BundleTemplate four_template =
      four_members.view().bundle_templates().at(kDefaultCommunicationBundleTemplateId);
  four_template.span_visual_assembly.visual_member_count = 4;
  if (!single_member.UpdateBundleTemplate(single_template).ok ||
      !four_members.UpdateBundleTemplate(four_template).ok ||
      !single_member.GenerateFromBackboneSpec(
          request_with(single_member, support_fixture_specs())).ok ||
      !four_members.GenerateFromBackboneSpec(
          request_with(four_members, support_fixture_specs())).ok) return false;
  WIRE_TEST_EXPECT(single_member.view().spans().size() == four_members.view().spans().size() &&
                       single_member.view().ports().size() == four_members.view().ports().size(),
                   "visual member count multiplied authoritative attachment topology");
  WIRE_TEST_EXPECT(single_member.view().visual_model_instances().instances ==
                       four_members.view().visual_model_instances().instances,
                   "visual member count multiplied endpoint or support fixtures");
  return edge_bodies(four_members, kDefaultCommunicationBundleTemplateId).size() ==
         edge_bodies(single_member, kDefaultCommunicationBundleTemplateId).size() * 4;
}

bool C864_non_hv_span_visual_variation_preserves_attachment_contracts() {
  using namespace city::wire;
  auto generate = [](CoreState* state, BundleTemplateId template_id, int count,
                     std::uint64_t placement_key) {
    BackboneBundleSpec spec{template_id, placement_key,
        template_id == kDefaultHighVoltageBundleTemplateId ? SpanLayer::kHighVoltage
                                                            : SpanLayer::kCommunication,
        count, true, template_id == kDefaultHighVoltageBundleTemplateId ? 9.2 : 5.3,
        template_id == kDefaultHighVoltageBundleTemplateId ? -0.2 : -0.30,
        template_id == kDefaultHighVoltageBundleTemplateId ? 0.45 : 0.05};
    return state->GenerateFromBackboneSpec(request_with(*state, {spec}));
  };

  const std::vector<BackboneBundleSpec> independent_specs = {
      {kDefaultCommunicationBundleTemplateId, 3001, SpanLayer::kCommunication, 1, true, 5.1, -0.30, 0.20},
      {kDefaultCommunicationBundleTemplateId, 3002, SpanLayer::kCommunication, 1, true, 5.4, -0.40, 0.20},
      {kDefaultCommunicationBundleTemplateId, 3003, SpanLayer::kCommunication, 1, true, 5.7, -0.50, 0.20},
  };
  CoreState independent;
  BundleTemplate independent_comm = independent.view().bundle_templates().at(
      kDefaultCommunicationBundleTemplateId);
  independent_comm.span_visual_assembly.visual_member_count = 1;
  if (!independent.UpdateBundleTemplate(independent_comm).ok ||
      !independent.GenerateFromBackboneSpec(
          request_with(independent, independent_specs)).ok) return false;
  const auto independent_parts = edge_bodies(
      independent, kDefaultCommunicationBundleTemplateId);
  WIRE_TEST_EXPECT(independent_parts.size() == independent_specs.size(),
                   "independent communication cable count mismatch");
  std::set<double> sags{};
  for (const VisualCurvePart* part : independent_parts) {
    const double unvaried_sag = 0.03 * Length(part->boundary_b - part->boundary_a);
    WIRE_TEST_EXPECT(part->sag_m >= unvaried_sag * 0.88 - 1e-12 &&
                         part->sag_m <= unvaried_sag * 1.12 + 1e-12,
                     "non-HV sag exceeded the configured variation range");
    sags.insert(part->sag_m);
  }
  WIRE_TEST_EXPECT(sags.size() >= 2, "non-HV sag did not vary by independent cable");

  CoreState bundled;
  BundleTemplate bundled_comm = bundled.view().bundle_templates().at(kDefaultCommunicationBundleTemplateId);
  bundled_comm.span_visual_assembly.visual_member_count = 3;
  if (!bundled.UpdateBundleTemplate(bundled_comm).ok ||
      !generate(&bundled, kDefaultCommunicationBundleTemplateId, 1, 3010).ok) return false;
  const auto bundled_parts = edge_bodies(bundled, kDefaultCommunicationBundleTemplateId);
  WIRE_TEST_EXPECT(bundled.view().spans().size() == 1 && bundled_parts.size() == 3,
                   "visual members multiplied authoritative topology");
  const Span* bundled_span = bundled.view().spans().find(bundled_parts.front()->source_span_id);
  const Port* bundled_port_a = bundled_span == nullptr ? nullptr :
      bundled.view().ports().find(bundled_span->port_a_id);
  const Port* bundled_port_b = bundled_span == nullptr ? nullptr :
      bundled.view().ports().find(bundled_span->port_b_id);
  WIRE_TEST_EXPECT(bundled_port_a != nullptr && bundled_port_b != nullptr,
                   "visual bundle lost its authoritative endpoint ports");
  const std::size_t sample_count = bundled_parts.front()->samples.size();
  WIRE_TEST_EXPECT(sample_count >= 5 &&
                       std::ranges::all_of(bundled_parts, [sample_count](const auto* member) {
                         return member->samples.size() == sample_count;
                       }),
                   "visual bundle members do not share a stable sampling frame");
  const auto centroid_at = [&bundled_parts](std::size_t index) {
    Vec3d centroid{};
    for (const VisualCurvePart* member : bundled_parts) centroid = centroid + member->samples[index];
    return ScaleVec(centroid, 1.0 / static_cast<double>(bundled_parts.size()));
  };
  WIRE_TEST_EXPECT(Length(centroid_at(0) - bundled_port_a->world_position) <= 1e-9 &&
                       Length(centroid_at(sample_count - 1) - bundled_port_b->world_position) <= 1e-9,
                   "visual bundle endpoint centroid moved away from the authoritative Port");

  const double cable_diameter = bundled.view().cable_templates().at(
      bundled_comm.cable_template_id).outer_diameter_m;
  const double member_spacing = bundled_comm.span_visual_assembly.visual_member_spacing_m;
  WIRE_TEST_EXPECT(member_spacing >= cable_diameter &&
                       member_spacing - cable_diameter <= 0.0025,
                   "default communication bundle is not diameter-constrained");
  const auto pair_distances_at = [&bundled_parts](std::size_t index) {
    std::vector<double> distances{};
    for (std::size_t a = 0; a < bundled_parts.size(); ++a) {
      for (std::size_t b = a + 1; b < bundled_parts.size(); ++b) {
        distances.push_back(Length(bundled_parts[a]->samples[index] -
                                   bundled_parts[b]->samples[index]));
      }
    }
    std::sort(distances.begin(), distances.end());
    return distances;
  };
  const auto triangle_area_at = [&bundled_parts](std::size_t index) {
    return Length(Cross(bundled_parts[1]->samples[index] - bundled_parts[0]->samples[index],
                        bundled_parts[2]->samples[index] - bundled_parts[0]->samples[index])) * 0.5;
  };
  for (std::size_t index = 0; index < sample_count; ++index) {
    const Vec3d centroid = centroid_at(index);
    const std::vector<double> pair_distances = pair_distances_at(index);
    WIRE_TEST_EXPECT(pair_distances.front() + 1e-9 >= cable_diameter &&
                         pair_distances.back() <= member_spacing + 1e-9,
                     "visual bundle members overlap or separate beyond the compact cross-section");
    WIRE_TEST_EXPECT(triangle_area_at(index) >= member_spacing * member_spacing * 0.25,
                     "three visual members collapsed to a collinear cross-section");
    for (const VisualCurvePart* member : bundled_parts) {
      WIRE_TEST_EXPECT(Length(member->samples[index] - centroid) <= member_spacing * 0.85,
                       "visual bundle member escaped the containment radius");
    }
  }
  CoreState connected;
  BundleTemplate connected_comm =
      connected.view().bundle_templates().at(kDefaultCommunicationBundleTemplateId);
  connected_comm.span_visual_assembly.visual_member_count = 3;
  if (!connected.UpdateBundleTemplate(connected_comm).ok) return false;
  BackboneBundleSpec connected_spec{kDefaultCommunicationBundleTemplateId, 3201,
      SpanLayer::kCommunication, 1, true, 5.3, -0.30, 0.20};
  const auto connected_result = connected.GenerateFromBackboneSpec(request_with(
      connected, {connected_spec}, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0},
                                    {18.0, 6.0, 0.0}}));
  if (!connected_result.ok) return false;
  std::vector<const VisualCurvePart*> connection_members{};
  for (const VisualCurvePart& part : connected.view().visual_curve_parts().parts) {
    if (part.kind == VisualCurvePartKind::kNodePatch &&
        part.bundle_template_id == kDefaultCommunicationBundleTemplateId) {
      connection_members.push_back(&part);
    }
  }
  WIRE_TEST_EXPECT(connection_members.size() == 3,
                   "visual bundle connection collapsed to one center curve");
  const std::size_t connection_middle = connection_members.front()->samples.size() / 2;
  WIRE_TEST_EXPECT(connection_members.front()->samples.size() >= 3 &&
                       std::ranges::all_of(connection_members,
                           [connection_middle](const VisualCurvePart* member) {
                             return member->samples.size() > connection_middle;
                           }),
                   "visual bundle connection members do not share a sampling frame");
  WIRE_TEST_EXPECT(Length(Cross(
                       connection_members[1]->samples[connection_middle] -
                           connection_members[0]->samples[connection_middle],
                       connection_members[2]->samples[connection_middle] -
                           connection_members[0]->samples[connection_middle])) > 1e-6,
                   "visual bundle connection lost its two-dimensional cross-section");
  const auto connected_bodies = edge_bodies(connected, kDefaultCommunicationBundleTemplateId);
  WIRE_TEST_EXPECT(connected_bodies.size() == 6,
                   "visual bundle main spans did not keep three members per edge");
  for (const VisualCurvePart* connection : connection_members) {
    for (const Vec3d endpoint : {connection->samples.front(), connection->samples.back()}) {
      double minimum_endpoint_distance = std::numeric_limits<double>::max();
      for (const VisualCurvePart* body : connected_bodies) {
        minimum_endpoint_distance = std::min({minimum_endpoint_distance,
            Length(body->samples.front() - endpoint),
            Length(body->samples.back() - endpoint)});
      }
      WIRE_TEST_EXPECT(minimum_endpoint_distance <= 0.0025,
                       "visual bundle connection member did not meet a main-span member: " +
                           std::to_string(minimum_endpoint_distance));
    }
  }

  CoreState hv_default;
  CoreState hv_no_variation;
  VariationSettings disabled = hv_no_variation.view().variation_settings();
  disabled.sag_variation_scale = 0.0;
  if (!hv_no_variation.UpdateVariationSettings(disabled).ok ||
      !generate(&hv_default, kDefaultHighVoltageBundleTemplateId, 3, 3002).ok ||
      !generate(&hv_no_variation, kDefaultHighVoltageBundleTemplateId, 3, 3002).ok) return false;
  const auto hv_a = edge_bodies(hv_default, kDefaultHighVoltageBundleTemplateId);
  const auto hv_b = edge_bodies(hv_no_variation, kDefaultHighVoltageBundleTemplateId);
  WIRE_TEST_EXPECT(hv_a.size() == hv_b.size(), "HV member count mismatch");
  for (std::size_t index = 0; index < hv_a.size(); ++index) {
    WIRE_TEST_EXPECT(hv_a[index]->sag_m == hv_b[index]->sag_m &&
                     same_samples(hv_a[index]->samples, hv_b[index]->samples),
                     "HV changed with sag variation setting");
  }
  return true;
}

bool C865_non_hv_support_and_member_shape_survive_save_load() {
  using namespace city::wire;
  CoreState state;
  if (!configure_non_hv_model_fixtures(&state)) return false;
  BundleTemplate communication_template =
      state.view().bundle_templates().at(kDefaultCommunicationBundleTemplateId);
  communication_template.span_visual_assembly.visual_member_count = 3;
  if (!state.UpdateBundleTemplate(communication_template).ok) return false;
  BackboneBundleSpec communication{kDefaultCommunicationBundleTemplateId, 4001,
      SpanLayer::kCommunication, 1, true, 5.2, -0.40, 0.20};
  if (!state.GenerateFromBackboneSpec(request_with(state, {communication})).ok) return false;
  const auto models_before = state.view().visual_model_instances().instances;
  const auto bodies_before = edge_bodies(state, kDefaultCommunicationBundleTemplateId);
  std::vector<std::vector<Vec3d>> samples_before{};
  for (const auto* body : bodies_before) samples_before.push_back(body->samples);
  const ObjectId communication_span_id = bodies_before.empty() ? kInvalidObjectId :
      bodies_before.front()->section_key.logical_span_id;
  if (communication_span_id == kInvalidObjectId ||
      !state.DeriveGeneratedSpanOutputs(communication_span_id).ok) return false;
  const auto bodies_rederived = edge_bodies(state, kDefaultCommunicationBundleTemplateId);
  WIRE_TEST_EXPECT(bodies_rederived.size() == samples_before.size(),
                   "derived rebuild changed visual member count");
  for (std::size_t index = 0; index < bodies_rederived.size(); ++index) {
    WIRE_TEST_EXPECT(same_samples(bodies_rederived[index]->samples, samples_before[index]),
                     "derived rebuild rerolled the visual bundle cross-section");
  }
  std::string saved{};
  if (!state.SerializeAuthoritative(&saved).ok) return false;
  CoreState loaded;
  if (!loaded.DeserializeAuthoritative(saved).ok ||
      loaded.view().visual_model_instances().instances != models_before) return false;
  const auto bodies_after = edge_bodies(loaded, kDefaultCommunicationBundleTemplateId);
  if (bodies_after.size() != samples_before.size()) return false;
  for (std::size_t index = 0; index < bodies_after.size(); ++index) {
    if (!same_samples(bodies_after[index]->samples, samples_before[index])) return false;
  }

  CoreState optical;
  BundleTemplate optical_template =
      optical.view().bundle_templates().at(kDefaultOpticalBundleTemplateId);
  optical_template.span_visual_assembly.visual_member_count = 3;
  if (!optical.UpdateBundleTemplate(optical_template).ok) return false;
  BackboneBundleSpec optical_spec{kDefaultOpticalBundleTemplateId, 4101,
      SpanLayer::kOptical, 1, true, 5.1, -0.38, 0.20};
  if (!optical.GenerateFromBackboneSpec(request_with(optical, {optical_spec})).ok) return false;
  const auto optical_members = edge_bodies(optical, kDefaultOpticalBundleTemplateId);
  const VisualCurvePart* support = nullptr;
  const VisualCurvePart* helix = nullptr;
  for (const VisualCurvePart& part : optical.view().visual_curve_parts().parts) {
    if (part.bundle_template_id != kDefaultOpticalBundleTemplateId) continue;
    if (part.supplemental_kind == VisualSupplementalKind::kSupportPath) support = &part;
    if (part.supplemental_kind == VisualSupplementalKind::kHelix) helix = &part;
  }
  WIRE_TEST_EXPECT(optical.view().spans().size() == 1 && optical_members.size() == 3 &&
                       support != nullptr && helix != nullptr,
                   "optical did not use one span visual assembly pipeline");
  const ObjectId logical_span_id = optical_members.front()->section_key.logical_span_id;
  WIRE_TEST_EXPECT(optical_members.back()->section_key.logical_span_id == logical_span_id &&
                       support->source_span_id == logical_span_id && helix->source_span_id == logical_span_id,
                   "optical assembly lost source logical span identity");
  const double support_length = polyline_length(support->samples);
  const double trim = optical_template.span_visual_assembly.endpoint_trim_m;
  const auto [helix_start_anchor, helix_start_tangent] =
      sample_polyline(support->samples, trim);
  const Vec3d helix_start_lateral = normalized(Cross({0.0, 0.0, 1.0}, helix_start_tangent));
  const double helix_radius = std::abs(Dot(
      helix->samples.front() - helix_start_anchor, helix_start_lateral));
  const double inset = support->wire_radius_m * 2.0 +
      optical_template.span_visual_assembly.helix_clearance_m;
  const double optical_diameter = optical.view().cable_templates().at(
      optical_template.cable_template_id).outer_diameter_m;
  WIRE_TEST_EXPECT(helix_radius > inset,
                   "optical helix radius could not be recovered from its support frame");
  for (std::size_t index = 0; index < optical_members.front()->samples.size(); ++index) {
    for (std::size_t a = 0; a < optical_members.size(); ++a) {
      for (std::size_t b = a + 1; b < optical_members.size(); ++b) {
        WIRE_TEST_EXPECT(Length(optical_members[a]->samples[index] -
                                    optical_members[b]->samples[index]) + 1e-6 >= optical_diameter,
                         "optical visual members overlap inside the helix");
      }
    }
    const double t = optical_members.front()->samples.size() < 2 ? 0.0 :
        static_cast<double>(index) /
            static_cast<double>(optical_members.front()->samples.size() - 1);
    if (t < 0.15 || t > 0.85) continue;
    const auto [anchor, tangent] = sample_polyline(support->samples, support_length * t);
    const Vec3d lateral = normalized(Cross({0.0, 0.0, 1.0}, tangent));
    const Vec3d up = normalized(Cross(tangent, lateral));
    const Vec3d axis = anchor - ScaleVec(up, helix_radius - inset);
    for (const VisualCurvePart* member : optical_members) {
      const double occupied_radius = Length(member->samples[index] - axis) +
          support->wire_radius_m + member->wire_radius_m +
          optical_template.span_visual_assembly.helix_clearance_m;
      WIRE_TEST_EXPECT(occupied_radius <= helix_radius + 1e-8,
                       "optical visual member escaped the shared helix containment radius");
    }
  }
  return true;
}

bool C894_non_hv_sharp_jumper_keeps_two_port_g1_contract() {
  using namespace city::wire;
  CoreState state;
  BundleTemplate communication = state.view().bundle_templates().at(
      kDefaultCommunicationBundleTemplateId);
  communication.span_visual_assembly.visual_member_count = 1;
  if (!state.UpdateBundleTemplate(communication).ok) return false;

  BackboneBundleSpec spec{kDefaultCommunicationBundleTemplateId, 89401,
      SpanLayer::kCommunication, 1, true, 5.2, -0.30, 0.20};
  const auto generated = state.GenerateFromBackboneSpec(request_with(
      state, {spec}, {{0.0, 0.0, 0.0}, {12.0, 0.0, 0.0}, {6.0, 3.0, 0.0}}));
  WIRE_TEST_EXPECT_ANCHOR(generated.ok && generated.value.generated_span_ids.size() == 2,
                          generated.error.empty() ?
                              "sharp communication fixture generation failed" : generated.error);

  std::vector<const VisualCurvePart*> bodies{};
  std::vector<const VisualCurvePart*> jumpers{};
  for (const VisualCurvePart& part : state.view().visual_curve_parts().parts) {
    if (part.bundle_template_id != kDefaultCommunicationBundleTemplateId) continue;
    if (part.kind == VisualCurvePartKind::kEdgeBody) bodies.push_back(&part);
    if (part.kind == VisualCurvePartKind::kJumper) jumpers.push_back(&part);
  }
  WIRE_TEST_EXPECT_ANCHOR(bodies.size() == 2 && jumpers.size() == 1,
                          "sharp communication did not keep two bodies and one Jumper");
  const VisualCurvePart& jumper = *jumpers.front();
  WIRE_TEST_EXPECT_PRESENCE(jumper.samples.size() >= 3,
                            "sharp communication Jumper has insufficient final samples");

  const auto body_away_from = [&](const Vec3d& endpoint) -> std::optional<Vec3d> {
    for (const VisualCurvePart* body : bodies) {
      if (body->samples.size() < 2) continue;
      if (Length(body->samples.front() - endpoint) <= 1e-9) {
        return normalized(body->samples[1] - body->samples.front());
      }
      if (Length(body->samples.back() - endpoint) <= 1e-9) {
        return normalized(body->samples[body->samples.size() - 2] - body->samples.back());
      }
    }
    return std::nullopt;
  };
  const std::array<std::pair<Vec3d, Vec3d>, 2> jumper_endpoints{{
      {jumper.samples.front(), normalized(jumper.samples[1] - jumper.samples.front())},
      {jumper.samples.back(), normalized(
          jumper.samples[jumper.samples.size() - 2] - jumper.samples.back())},
  }};
  for (const auto& [endpoint, jumper_away] : jumper_endpoints) {
    const std::optional<Vec3d> body_away = body_away_from(endpoint);
    WIRE_TEST_EXPECT_PRESENCE(body_away.has_value(),
                              "sharp Jumper endpoint did not meet a main body endpoint");
    const double endpoint_dot = Dot(*body_away, jumper_away);
    WIRE_TEST_EXPECT_ORACLE(endpoint_dot <= -0.995,
                            "sharp Jumper final samples are not G1 with the main body: dot=" +
                                std::to_string(endpoint_dot));
  }
  return true;
}

bool C888_variation_branch_pick_uses_exact_live_scope() {
  using namespace city::wire;
  CoreState state{};
  RouteBundleVariationInput descriptor{};
  descriptor.route_seed = 0x8880102;
  descriptor.preferred_side_sign = -1;
  descriptor.pole_type_id = 2;
  descriptor.rules = {
      {kDefaultCommunicationBundleTemplateId, 1, 1, 1,
       5.2, 5.2, 0.25, 0.25},
      {kDefaultHighVoltageBundleTemplateId, 0, 0, 3,
       9.2, 9.2, 0.2, 0.2}};
  const auto generated = state.GenerateBackboneBundleVariation(
      request_with(state, {}), descriptor);
  WIRE_TEST_EXPECT_PRESENCE(
      generated.ok && generated.value.generation.generated_span_ids.size() == 1,
      generated.error.empty()
          ? "variation branch-pick fixture did not create one live Bundle"
          : generated.error);
  const ObjectId span_id =
      generated.value.generation.generated_span_ids.front();
  const Span* span = state.view().spans().find(span_id);
  const Port* port_a = span == nullptr
                           ? nullptr
                           : state.view().ports().find(span->port_a_id);
  const Port* port_b = span == nullptr
                           ? nullptr
                           : state.view().ports().find(span->port_b_id);
  WIRE_TEST_EXPECT_PRESENCE(port_a != nullptr && port_b != nullptr,
                            "variation branch-pick Span endpoints are missing");
  PickResult pick{};
  pick.hit_kind = PickHitKind::kSegment;
  pick.hit_id = span_id;
  pick.hit_pos_world = ScaleVec(
      port_a->world_position + port_b->world_position, 0.5);
  const auto resolved = state.ResolveBackboneBundleVariationBranchPick(
      generated.value.variation_id, pick);
  const auto& pending = CoreStateTestHook::pending_support_nodes(state);
  WIRE_TEST_EXPECT_ANCHOR(
      resolved.ok && pending.size() == 1 &&
          pending.front().bundle_modes.size() == 1 &&
          pending.front().bundle_modes.front().bundle_template_id ==
              kDefaultCommunicationBundleTemplateId,
      resolved.error.empty()
          ? "zero-instance descriptor rule entered the physical branch scope"
          : resolved.error);
  return true;
}

bool C895_route_variation_controls_resolve_and_apply_full_range() {
  using namespace city::wire;
  auto controls = [](double density, double height_spread,
                     double lateral_spread) {
    auto rules = visual_rules();
    for (RandomBackboneBundleRule& rule : rules) {
      if (rule.bundle_template_id == kDefaultHighVoltageBundleTemplateId) {
        continue;
      }
      const double height_center =
          (rule.height_min_m + rule.height_max_m) * 0.5;
      const double height_half =
          (rule.height_max_m - rule.height_min_m) * 0.5 * height_spread;
      const double lateral_center =
          (rule.lateral_abs_min_m + rule.lateral_abs_max_m) * 0.5;
      const double lateral_half =
          (rule.lateral_abs_max_m - rule.lateral_abs_min_m) * 0.5 *
          lateral_spread;
      rule.min_instances = std::max(
          1, static_cast<int>(std::lround(rule.min_instances * density)));
      rule.max_instances = std::max(
          rule.min_instances,
          static_cast<int>(std::lround(rule.max_instances * density)));
      rule.height_min_m = height_center - height_half;
      rule.height_max_m = height_center + height_half;
      rule.lateral_abs_min_m = std::max(0.0, lateral_center - lateral_half);
      rule.lateral_abs_max_m = lateral_center + lateral_half;
    }
    return rules;
  };
  auto input = [&](double density, double height_spread,
                   double lateral_spread) {
    RouteBundleVariationInput descriptor{};
    descriptor.route_seed = 0x8950102;
    descriptor.preferred_side_sign = -1;
    descriptor.pole_type_id = 2;
    descriptor.rules = controls(density, height_spread, lateral_spread);
    return descriptor;
  };
  auto placement_by_key = [](const CoreState& state, ObjectId variation_id) {
    std::map<std::uint64_t, std::pair<double, double>> placements{};
    const SavedBackboneBundleVariation* variation =
        state.view().backbone_bundle_variation(variation_id);
    if (variation == nullptr) return placements;
    for (const SavedBackboneBundleVariationInstance& instance :
         variation->instances) {
      const Bundle* bundle = state.view().bundles().find(instance.bundle_id);
      if (bundle != nullptr) {
        placements.emplace(instance.placement_key,
                           std::pair{bundle->height_m, bundle->lateral_m});
      }
    }
    return placements;
  };

  CoreState state{};
  const RouteBundleVariationInput baseline = input(1.0, 1.0, 1.0);
  const RouteBundleVariationInput dense = input(5.0, 1.0, 1.0);
  const auto baseline_resolved = state.ResolveRouteBundleVariation(baseline);
  const auto dense_resolved = state.ResolveRouteBundleVariation(dense);
  WIRE_TEST_EXPECT_PRESENCE(
      baseline_resolved.ok && dense_resolved.ok &&
          dense_resolved.value.size() > baseline_resolved.value.size(),
      dense_resolved.error.empty()
          ? "Density 5x did not resolve a denser concrete Bundle set"
          : dense_resolved.error);

  const auto generated = state.GenerateBackboneBundleVariation(
      request_with(state, {}), baseline);
  WIRE_TEST_EXPECT_PRESENCE(generated.ok, generated.error);
  const ObjectId variation_id = generated.value.variation_id;
  const auto dense_applied =
      state.ApplyBackboneBundleVariation(variation_id, dense);
  const auto dense_placements = placement_by_key(state, variation_id);
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      dense_applied.ok && dense_applied.value &&
          dense_placements.size() == dense_resolved.value.size(),
      dense_applied.error.empty()
          ? "Density 5x did not apply its full concrete Bundle set"
          : dense_applied.error);

  const RouteBundleVariationInput taller = input(5.0, 1.75, 1.0);
  const auto taller_applied =
      state.ApplyBackboneBundleVariation(variation_id, taller);
  const auto taller_placements = placement_by_key(state, variation_id);
  const bool height_changed = std::ranges::any_of(
      taller_placements, [&](const auto& placement) {
        const auto previous = dense_placements.find(placement.first);
        return previous != dense_placements.end() &&
               previous->second.first != placement.second.first;
      });
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      taller_applied.ok && taller_applied.value && height_changed,
      taller_applied.error.empty()
          ? "Height spread changed the descriptor without changing concrete placement"
          : taller_applied.error);

  const RouteBundleVariationInput wider = input(5.0, 1.75, 1.75);
  const auto wider_applied =
      state.ApplyBackboneBundleVariation(variation_id, wider);
  const auto wider_placements = placement_by_key(state, variation_id);
  const bool lateral_changed = std::ranges::any_of(
      wider_placements, [&](const auto& placement) {
        const auto previous = taller_placements.find(placement.first);
        return previous != taller_placements.end() &&
               previous->second.second != placement.second.second;
      });
  WIRE_TEST_EXPECT_DIFFERENTIAL(
      wider_applied.ok && wider_applied.value && lateral_changed,
      wider_applied.error.empty()
          ? "Lateral spread changed the descriptor without changing concrete placement"
          : wider_applied.error);

  RouteBundleVariationInput tiny = wider;
  tiny.rules[1].height_min_m += 1e-8;
  tiny.rules[1].height_max_m += 1e-8;
  const auto tiny_resolved = state.ResolveRouteBundleVariation(tiny);
  const auto tiny_applied =
      state.ApplyBackboneBundleVariation(variation_id, tiny);
  const SavedBackboneBundleVariation* tiny_variation =
      state.view().backbone_bundle_variation(variation_id);
  bool tiny_exact = tiny_resolved.ok && tiny_variation != nullptr;
  for (const BackboneBundleSpec& desired : tiny_resolved.value) {
    const auto instance = std::ranges::find_if(
        tiny_variation->instances,
        [&](const SavedBackboneBundleVariationInstance& value) {
          return value.placement_key == desired.placement_key;
        });
    const Bundle* bundle =
        instance == tiny_variation->instances.end()
            ? nullptr
            : state.view().bundles().find(instance->bundle_id);
    tiny_exact = tiny_exact && bundle != nullptr &&
                 bundle->height_m == desired.height_m &&
                 bundle->lateral_m == desired.lateral_m;
  }
  WIRE_TEST_EXPECT_ANCHOR(
      tiny_applied.ok && tiny_applied.value && tiny_exact,
      tiny_applied.error.empty()
          ? "sub-tolerance control change was not stored as the exact desired placement"
          : tiny_applied.error);
  WIRE_TEST_EXPECT_BACKBONE_INVARIANTS(state);
  return true;
}

} // namespace backbone_tests
