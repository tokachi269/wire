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
      {kDefaultHighVoltageBundleTemplateId, 1, 1, 3, 9.2, 9.2, 0.2, 0.2, 0.25},
      {kDefaultLowVoltageBundleTemplateId, 2, 4, 1, 7.0, 7.7, 0.12, 0.52, 0.20},
      {kDefaultCommunicationBundleTemplateId, 1, 4, 1, 5.0, 5.8, 0.12, 0.50, 0.16},
      {kDefaultOpticalBundleTemplateId, 0, 2, 1, 4.9, 5.7, 0.12, 0.50, 0.16},
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
      {kDefaultCommunicationBundleTemplateId, 2, 3, 1, 5.0, 5.8, 0.12, 0.50, 0.16},
      {kDefaultOpticalBundleTemplateId, 1, 2, 1, 4.9, 5.7, 0.12, 0.50, 0.16},
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
  single_template.span_visual_assembly.visual_member_count_min = 1;
  single_template.span_visual_assembly.visual_member_count_max = 1;
  BundleTemplate four_template =
      four_members.view().bundle_templates().at(kDefaultCommunicationBundleTemplateId);
  four_template.span_visual_assembly.visual_member_count_min = 4;
  four_template.span_visual_assembly.visual_member_count_max = 4;
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
  CoreState plain;
  BundleTemplate plain_comm = plain.view().bundle_templates().at(kDefaultCommunicationBundleTemplateId);
  plain_comm.span_visual_assembly.visual_member_count_min = 1;
  plain_comm.span_visual_assembly.visual_member_count_max = 1;
  plain_comm.span_visual_assembly.center_wander_amplitude_m = 0.0;
  plain_comm.span_visual_assembly.member_wander_ratio = 0.0;
  if (!plain.UpdateBundleTemplate(plain_comm).ok ||
      !plain.GenerateFromBackboneSpec(request_with(plain, independent_specs)).ok) return false;
  CoreState wandered;
  BundleTemplate wandered_comm = wandered.view().bundle_templates().at(kDefaultCommunicationBundleTemplateId);
  wandered_comm.span_visual_assembly.visual_member_count_min = 1;
  wandered_comm.span_visual_assembly.visual_member_count_max = 1;
  wandered_comm.span_visual_assembly.member_wander_ratio = 0.0;
  if (!wandered.UpdateBundleTemplate(wandered_comm).ok ||
      !wandered.GenerateFromBackboneSpec(request_with(wandered, independent_specs)).ok) return false;
  const auto plain_parts = edge_bodies(plain, kDefaultCommunicationBundleTemplateId);
  const auto wandered_parts = edge_bodies(wandered, kDefaultCommunicationBundleTemplateId);
  WIRE_TEST_EXPECT(plain_parts.size() == independent_specs.size() &&
                       wandered_parts.size() == plain_parts.size(),
                   "independent communication cable count mismatch");
  std::set<double> sags{};
  std::vector<Vec3d> middle_offsets{};
  std::vector<double> maximum_offsets{};
  for (std::size_t cable = 0; cable < plain_parts.size(); ++cable) {
    const auto& before = *plain_parts[cable];
    const auto& after = *wandered_parts[cable];
    WIRE_TEST_EXPECT(before.samples.size() == after.samples.size() && before.samples.size() >= 4,
                     "wander changed sampling");
    WIRE_TEST_EXPECT(same_point(before.samples.front(), after.samples.front()) &&
                         same_point(before.samples.back(), after.samples.back()),
                     "center wander changed an attachment endpoint");
    const Vec3d before_start_tangent = normalized(before.samples[1] - before.samples[0]);
    const Vec3d after_start_tangent = normalized(after.samples[1] - after.samples[0]);
    const Vec3d before_end_tangent = normalized(before.samples.back() - before.samples[before.samples.size() - 2]);
    const Vec3d after_end_tangent = normalized(after.samples.back() - after.samples[after.samples.size() - 2]);
    WIRE_TEST_EXPECT(Dot(before_start_tangent, after_start_tangent) >= 0.9998 &&
                         Dot(before_end_tangent, after_end_tangent) >= 0.9998,
                     "center wander changed an endpoint tangent too much");
    double maximum_offset = 0.0;
    for (std::size_t index = 1; index + 1 < before.samples.size(); ++index) {
      maximum_offset = std::max(maximum_offset, Length(before.samples[index] - after.samples[index]));
    }
    maximum_offsets.push_back(maximum_offset);
    WIRE_TEST_EXPECT(Length(before.samples[1] - after.samples[1]) > 1e-8 &&
                         Length(before.samples[before.samples.size() - 2] -
                                after.samples[after.samples.size() - 2]) > 1e-8,
                     "center wander has a fully straight endpoint interval");
    middle_offsets.push_back(
        after.samples[after.samples.size() / 2] - before.samples[before.samples.size() / 2]);
    const double unvaried_sag = 0.03 * Length(after.boundary_b - after.boundary_a);
    WIRE_TEST_EXPECT(after.sag_m >= unvaried_sag * 0.88 - 1e-12 &&
                         after.sag_m <= unvaried_sag * 1.12 + 1e-12,
                     "non-HV sag exceeded the configured variation range");
    sags.insert(after.sag_m);
  }
  WIRE_TEST_EXPECT(Length(middle_offsets[0] - middle_offsets[1]) > 0.005 ||
                       Length(middle_offsets[1] - middle_offsets[2]) > 0.005,
                   "independent non-HV cables received the same center path variation");
  const auto [minimum_offset, maximum_offset] = std::ranges::minmax(maximum_offsets);
  WIRE_TEST_EXPECT(minimum_offset >= 0.03 && maximum_offset <= 0.08,
                   "center wander is not within the visible tuning envelope: min=" +
                       std::to_string(minimum_offset) + " max=" +
                       std::to_string(maximum_offset));
  WIRE_TEST_EXPECT(sags.size() >= 2, "non-HV sag did not vary by independent cable");

  CoreState bundled;
  BundleTemplate bundled_comm = bundled.view().bundle_templates().at(kDefaultCommunicationBundleTemplateId);
  bundled_comm.span_visual_assembly.visual_member_count_min = 3;
  bundled_comm.span_visual_assembly.visual_member_count_max = 3;
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
    WIRE_TEST_EXPECT(pair_distances.front() > cable_diameter &&
                         pair_distances.back() <= member_spacing * 1.35,
                     "visual bundle members overlap or separate beyond the compact cross-section");
    WIRE_TEST_EXPECT(triangle_area_at(index) >= member_spacing * member_spacing * 0.25,
                     "three visual members collapsed to a collinear cross-section");
    for (const VisualCurvePart* member : bundled_parts) {
      WIRE_TEST_EXPECT(Length(member->samples[index] - centroid) <= member_spacing * 0.85,
                       "visual bundle member escaped the containment radius");
    }
  }
  const std::size_t quarter = bundled_parts.front()->samples.size() / 4;
  const std::size_t middle = bundled_parts.front()->samples.size() / 2;
  const std::size_t three_quarters = bundled_parts.front()->samples.size() * 3 / 4;
  const std::vector<double> quarter_distances = pair_distances_at(quarter);
  const std::vector<double> middle_distances = pair_distances_at(middle);
  const std::vector<double> three_quarter_distances = pair_distances_at(three_quarters);
  double cross_section_change = 0.0;
  for (std::size_t index = 0; index < quarter_distances.size(); ++index) {
    cross_section_change = std::max({cross_section_change,
        std::abs(quarter_distances[index] - middle_distances[index]),
        std::abs(middle_distances[index] - three_quarter_distances[index])});
  }
  WIRE_TEST_EXPECT(cross_section_change > 0.0001,
                   "visual bundle cross-section remained completely fixed along the span");

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
  communication_template.span_visual_assembly.visual_member_count_min = 3;
  communication_template.span_visual_assembly.visual_member_count_max = 3;
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
  optical_template.span_visual_assembly.visual_member_count_min = 3;
  optical_template.span_visual_assembly.visual_member_count_max = 3;
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
  const Vec3d support_middle = support->samples[support->samples.size() / 2];
  Vec3d member_middle{};
  for (const VisualCurvePart* member : optical_members) {
    member_middle = member_middle + member->samples[member->samples.size() / 2];
  }
  member_middle = ScaleVec(member_middle, 1.0 / static_cast<double>(optical_members.size()));
  WIRE_TEST_EXPECT(Length(member_middle - support_middle) <= 0.10,
                   "optical support path separated from its visual members");

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
                                    optical_members[b]->samples[index]) > optical_diameter,
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

} // namespace backbone_tests
