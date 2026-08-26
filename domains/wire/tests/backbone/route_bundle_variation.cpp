#include "fixtures.hpp"
#include "cases.hpp"
#include "../registry.hpp"
#include "city/wire/coord_utils.hpp"

#include <algorithm>
#include <cmath>
#include <map>
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
      {kDefaultHighVoltageBundleTemplateId, 1, 1, 3, 3, 0.45, 0.45, 9.2, 9.2, 0.2, 0.2, 0.25},
      {kDefaultLowVoltageBundleTemplateId, 2, 4, 1, 1, 0.20, 0.20, 7.0, 7.7, 0.12, 0.52, 0.20},
      {kDefaultCommunicationBundleTemplateId, 1, 4, 1, 3, 0.035, 0.060, 5.0, 5.8, 0.12, 0.50, 0.16},
      {kDefaultOpticalBundleTemplateId, 0, 2, 1, 1, 0.20, 0.20, 4.9, 5.7, 0.12, 0.50, 0.16},
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
    return std::tie(a->lane_index, a->source_span_id) <
           std::tie(b->lane_index, b->source_span_id);
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
      {kDefaultCommunicationBundleTemplateId, 2, 3, 1, 3, 0.035, 0.060, 5.0, 5.8, 0.12, 0.50, 0.16},
      {kDefaultOpticalBundleTemplateId, 1, 2, 1, 1, 0.20, 0.20, 4.9, 5.7, 0.12, 0.50, 0.16},
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
                  2, 2, 1, 1, 0.20, 0.20, 7.0, 7.0, 0.2, 0.2, 0.5}};
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

bool C862_non_hv_route_variation_covers_each_category_and_keeps_hv_fixed() {
  using namespace city::wire;
  CoreState state;
  std::map<BundleTemplateId, std::set<std::string>> signatures{};
  std::set<int> communication_counts{};
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
        communication_counts.insert(spec.count);
        if (spec.count > 1 && (spec.spacing_m < 0.035 || spec.spacing_m > 0.060)) return false;
      }
    }
  }
  return signatures[kDefaultLowVoltageBundleTemplateId].size() > 1 &&
         signatures[kDefaultCommunicationBundleTemplateId].size() > 1 &&
         signatures[kDefaultOpticalBundleTemplateId].size() > 1 &&
         communication_counts.contains(1) && communication_counts.contains(2) &&
         communication_counts.contains(3);
}

bool C863_non_hv_support_rows_are_shared_and_direct_attachment_stays_direct() {
  using namespace city::wire;
  CoreState supported;
  if (!configure_non_hv_model_fixtures(&supported)) return false;
  const auto generated = supported.GenerateFromBackboneSpec(
      request_with(supported, support_fixture_specs()));
  if (!generated.ok) return false;
  std::vector<const VisualModelInstance*> rows{};
  for (const auto& model : supported.view().visual_model_instances().instances) {
    if (model.model_key == "non_hv_support_row_proxy") rows.push_back(&model);
  }
  if (rows.size() != generated.value.generated_pole_ids.size() ||
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
  return clamp_count == direct_generated.value.generated_pole_ids.size() &&
         std::ranges::none_of(direct.view().visual_model_instances().instances,
                              [](const VisualModelInstance& model) {
           return model.model_key == "non_hv_support_row_proxy";
         });
}

bool C864_non_hv_bundle_wander_preserves_endpoints_and_hv_sag() {
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

  CoreState plain;
  BundleTemplate plain_comm = plain.view().bundle_templates().at(kDefaultCommunicationBundleTemplateId);
  plain_comm.span_visual_assembly.member_wander_ratio = 0.0;
  if (!plain.UpdateBundleTemplate(plain_comm).ok ||
      !generate(&plain, kDefaultCommunicationBundleTemplateId, 3, 3001).ok) return false;
  CoreState wandered;
  if (!generate(&wandered, kDefaultCommunicationBundleTemplateId, 3, 3001).ok) return false;
  const auto plain_parts = edge_bodies(plain, kDefaultCommunicationBundleTemplateId);
  const auto wandered_parts = edge_bodies(wandered, kDefaultCommunicationBundleTemplateId);
  WIRE_TEST_EXPECT(plain_parts.size() == 3 && wandered_parts.size() == 3,
                   "communication member count mismatch");
  bool middle_changed = false;
  std::set<double> sags{};
  for (std::size_t lane = 0; lane < plain_parts.size(); ++lane) {
    const auto& before = *plain_parts[lane];
    const auto& after = *wandered_parts[lane];
    if (before.samples.size() != after.samples.size() || before.samples.empty() ||
        before.samples.front().x != after.samples.front().x ||
        before.samples.front().y != after.samples.front().y ||
        before.samples.front().z != after.samples.front().z ||
        before.samples.back().x != after.samples.back().x ||
        before.samples.back().y != after.samples.back().y ||
        before.samples.back().z != after.samples.back().z) {
      WIRE_TEST_EXPECT(false, "wander changed an endpoint");
    }
    for (std::size_t index = 1; index + 1 < before.samples.size(); ++index) {
      middle_changed = middle_changed || city::wire::Length(before.samples[index] - after.samples[index]) > 1e-6;
      WIRE_TEST_EXPECT(city::wire::Length(before.samples[index] - after.samples[index]) <= 0.016,
                       "wander exceeded member spacing envelope");
    }
    const double unvaried_sag = 0.03 * Length(after.boundary_b - after.boundary_a);
    WIRE_TEST_EXPECT(after.sag_m >= unvaried_sag * 0.95 - 1e-12 &&
                     after.sag_m <= unvaried_sag * 1.05 + 1e-12,
                     "non-HV sag exceeded the configured variation range");
    sags.insert(after.sag_m);
  }
  WIRE_TEST_EXPECT(middle_changed, "wander did not change a middle sample");
  WIRE_TEST_EXPECT(sags.size() >= 2, "non-HV sag did not vary by member");

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
  BackboneBundleSpec communication{kDefaultCommunicationBundleTemplateId, 4001,
      SpanLayer::kCommunication, 3, true, 5.2, -0.40, 0.05};
  if (!state.GenerateFromBackboneSpec(request_with(state, {communication})).ok) return false;
  const auto models_before = state.view().visual_model_instances().instances;
  const auto bodies_before = edge_bodies(state, kDefaultCommunicationBundleTemplateId);
  std::vector<std::vector<Vec3d>> samples_before{};
  for (const auto* body : bodies_before) samples_before.push_back(body->samples);
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
  return true;
}

} // namespace backbone_tests
