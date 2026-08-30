#include "city/wire/core_state.hpp"
#include "city/wire/core_view.hpp"

#include "../generation/backbone/emit_shared.hpp"
#include "../generation/backbone/route_support.hpp"
#include "../support/hash_mix.hpp"
#include "route_bundle_variation_validation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <unordered_set>
#include <vector>

namespace city::wire {
namespace {

constexpr std::uint64_t kJavascriptExactIntegerMask = (std::uint64_t{1} << 53) - 1;

double unit_value(std::uint64_t bits) {
  return static_cast<double>(bits >> 11) * (1.0 / 9007199254740992.0);
}

std::uint64_t rule_seed(std::uint64_t route_seed, std::size_t rule_ordinal,
                        BundleTemplateId template_id, int instance_ordinal) {
  std::uint64_t seed = support::hash_combine(route_seed, static_cast<std::uint64_t>(rule_ordinal));
  seed = support::hash_combine(seed, static_cast<std::uint64_t>(template_id));
  return support::hash_combine(seed, static_cast<std::uint64_t>(instance_ordinal));
}

double sample_range(std::uint64_t seed, std::uint64_t salt, double minimum, double maximum) {
  if (minimum == maximum) {
    return minimum;
  }
  return minimum + (maximum - minimum) * unit_value(support::hash_combine(seed, salt));
}

int sample_count(std::uint64_t seed, int minimum, int maximum) {
  const int range = maximum - minimum + 1;
  return minimum + static_cast<int>(support::splitmix64(seed) % static_cast<std::uint64_t>(range));
}

double sample_lateral_magnitude(std::uint64_t seed, double minimum, double maximum,
                                ConnectionCategory category) {
  const double raw = sample_range(seed, 0x4c41544552414cull, minimum, maximum);
  if (generation::backbone::route_support::is_high_voltage(category) ||
      raw <= generation::backbone::route_support::kDirectReachM) {
    return raw;
  }
  const double slot_offset =
      (raw - generation::backbone::route_support::kSupportedSlotStartM) /
      generation::backbone::route_support::kSupportedSlotSpacingM;
  const double slot = generation::backbone::route_support::kSupportedSlotStartM +
      std::round(slot_offset) * generation::backbone::route_support::kSupportedSlotSpacingM;
  const double jitter = sample_range(seed, 0x534c4f544a495454ull,
                                     -generation::backbone::route_support::kSupportedSlotJitterM,
                                     generation::backbone::route_support::kSupportedSlotJitterM);
  return std::clamp(slot + jitter, minimum, maximum);
}

bool same_membership_topology(
    const SavedBackboneBundleVariationMembership& a,
    const SavedBackboneBundleVariationMembership& b) {
  if (a.edge_ids != b.edge_ids ||
      a.row_continuities.size() != b.row_continuities.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.row_continuities.size(); ++i) {
    const auto& lhs = a.row_continuities[i];
    const auto& rhs = b.row_continuities[i];
    if (std::tie(lhs.node_id, lhs.edge_a, lhs.lane_a, lhs.edge_b,
                 lhs.lane_b) !=
        std::tie(rhs.node_id, rhs.edge_a, rhs.lane_a, rhs.edge_b,
                 rhs.lane_b)) {
      return false;
    }
  }
  return true;
}

} // namespace

EditResult<bool> detail::validate_route_bundle_variation_descriptor(
    const CoreState& state, const RouteBundleVariationInput& input) {
  EditResult<bool> result{};
  if (state.view().pole_types().find(input.pole_type_id) ==
      state.view().pole_types().end()) {
    result.error =
        "core invalid input: route bundle variation pole template is missing";
    return result;
  }
  if (input.preferred_side_sign < -1 || input.preferred_side_sign > 1) {
    result.error =
        "core invalid input: preferred side sign must be -1, 0, or +1";
    return result;
  }
  if (input.rules.empty()) {
    result.error = "core invalid input: route bundle variation rules are empty";
    return result;
  }
  std::set<std::pair<BundleTemplateId, int>> effective_groups{};
  for (const RandomBackboneBundleRule& rule : input.rules) {
    const auto template_it =
        state.view().bundle_templates().find(rule.bundle_template_id);
    if (template_it == state.view().bundle_templates().end()) {
      result.error =
          "core invalid input: route bundle variation bundle template is missing";
      return result;
    }
    if (rule.min_instances < 0 || rule.max_instances < rule.min_instances ||
        !std::isfinite(rule.height_min_m) ||
        !std::isfinite(rule.height_max_m) ||
        rule.height_max_m < rule.height_min_m ||
        !std::isfinite(rule.lateral_abs_min_m) ||
        !std::isfinite(rule.lateral_abs_max_m) ||
        rule.lateral_abs_min_m < 0.0 ||
        rule.lateral_abs_max_m < rule.lateral_abs_min_m ||
        rule.conductor_count < 0) {
      result.error =
          "core invalid input: route bundle variation rule is invalid";
      return result;
    }
    const BundleTemplate& bundle_template = template_it->second;
    const int conductor_count = rule.conductor_count > 0
                                    ? rule.conductor_count
                                    : bundle_template.default_count;
    if (conductor_count <= 0 ||
        (bundle_template.count_rule == BundleCountRuleKind::kFixed &&
         conductor_count != bundle_template.fixed_count) ||
        (bundle_template.count_rule == BundleCountRuleKind::kRange &&
         (conductor_count < bundle_template.min_count ||
          conductor_count > bundle_template.max_count))) {
      result.error =
          "core invalid input: route bundle variation conductor count is invalid";
      return result;
    }
    if (!effective_groups
             .emplace(rule.bundle_template_id, conductor_count)
             .second) {
      result.error =
          "core invalid input: route bundle variation effective rule group is duplicated";
      return result;
    }
  }
  result.ok = true;
  result.value = true;
  return result;
}

const SavedBackboneBundleVariation*
CoreState::backbone_bundle_variation_for_bundle(ObjectId bundle_id) const {
  for (const SavedBackboneBundleVariation& variation :
       authoritative_.backbone_bundle_variations) {
    const auto found = std::ranges::find_if(
        variation.instances,
        [&](const SavedBackboneBundleVariationInstance& instance) {
          return instance.bundle_id == bundle_id;
        });
    if (found != variation.instances.end()) return &variation;
  }
  return nullptr;
}

EditResult<ResolveBranchPickResult>
CoreState::ResolveBackboneBundleVariationBranchPick(
    ObjectId variation_id, const PickResult& pick) {
  EditResult<ResolveBranchPickResult> result{};
  const auto variation = std::ranges::find_if(
      authoritative_.backbone_bundle_variations,
      [variation_id](const SavedBackboneBundleVariation& candidate) {
        return candidate.variation_id == variation_id;
      });
  if (variation == authoritative_.backbone_bundle_variations.end()) {
    result.error = "core invalid input: backbone Bundle variation not found";
    return result;
  }
  ResolveBranchPickOptions options{};
  options.create_midair_node = true;
  options.create_midair_node_set = true;
  options.selected_bundle_template_ids.reserve(variation->instances.size());
  for (const SavedBackboneBundleVariationInstance& instance :
       variation->instances) {
    const Bundle* bundle = view().bundles().find(instance.bundle_id);
    if (bundle == nullptr || bundle->placement_key != instance.placement_key) {
      result.error =
          "backbone invalid input: variation branch scope has stale Bundle identity";
      return result;
    }
    options.selected_bundle_template_ids.push_back(bundle->bundle_template_id);
  }
  if (options.selected_bundle_template_ids.empty()) {
    result.error =
        "backbone unsupported: variation branch scope has no live Bundle instances";
    return result;
  }
  return ResolveBranchPick(pick, options);
}

EditResult<SavedBackboneBundleVariationMembership>
CoreState::capture_backbone_bundle_variation_membership(
    ObjectId bundle_id) const {
  EditResult<SavedBackboneBundleVariationMembership> result{};
  const Bundle* bundle = view().bundles().find(bundle_id);
  if (bundle == nullptr || bundle->conductor_count <= 0) {
    result.error = "backbone invalid input: variation membership Bundle is missing or invalid";
    return result;
  }
  SavedBackboneBundleVariationMembership membership{};
  membership.bundle_template_id = bundle->bundle_template_id;
  membership.conductor_count = bundle->conductor_count;
  std::unordered_set<ObjectId> edge_bundle_ids{};
  std::unordered_set<ObjectId> edge_ids{};
  const SavedBackboneGraph& graph = view().backbone();
  for (const SavedBackboneEdgeBundle& edge_bundle : graph.edge_bundles) {
    if (edge_bundle.bundle_id != bundle_id) continue;
    const auto edge = std::ranges::find_if(
        graph.edges, [&](const SavedBackboneEdge& value) {
          return value.edge_id == edge_bundle.edge_id;
        });
    if (edge_bundle.edge_bundle_id == kInvalidObjectId ||
        edge == graph.edges.end() ||
        !edge_bundle_ids.insert(edge_bundle.edge_bundle_id).second ||
        !edge_ids.insert(edge_bundle.edge_id).second) {
      result.error = "backbone invalid input: variation membership edge relation is invalid";
      return result;
    }
    membership.edge_ids.push_back(edge->edge_id);
  }
  if (membership.edge_ids.empty()) {
    result.error = "backbone unsupported: variation membership has no saved physical edges";
    return result;
  }
  for (const SavedBackboneRowContinuity& continuity : graph.row_continuities) {
    const bool a_owned = edge_bundle_ids.contains(continuity.a.edge_bundle_id);
    const bool b_owned = edge_bundle_ids.contains(continuity.b.edge_bundle_id);
    if (a_owned != b_owned) {
      result.error = "backbone invalid input: variation membership continuity crosses Bundle identity";
      return result;
    }
    if (!a_owned) continue;
    const auto edge_bundle_a = std::ranges::find_if(
        graph.edge_bundles, [&](const SavedBackboneEdgeBundle& value) {
          return value.edge_bundle_id == continuity.a.edge_bundle_id;
        });
    const auto edge_bundle_b = std::ranges::find_if(
        graph.edge_bundles, [&](const SavedBackboneEdgeBundle& value) {
          return value.edge_bundle_id == continuity.b.edge_bundle_id;
        });
    if (edge_bundle_a == graph.edge_bundles.end() ||
        edge_bundle_b == graph.edge_bundles.end() ||
        continuity.a.lane_index >=
            static_cast<std::size_t>(bundle->conductor_count) ||
        continuity.b.lane_index >=
            static_cast<std::size_t>(bundle->conductor_count)) {
      result.error = "backbone invalid input: variation membership continuity is invalid";
      return result;
    }
    ObjectId edge_a = edge_bundle_a->edge_id;
    ObjectId edge_b = edge_bundle_b->edge_id;
    std::size_t lane_a = continuity.a.lane_index;
    std::size_t lane_b = continuity.b.lane_index;
    if (std::pair{edge_b, lane_b} < std::pair{edge_a, lane_a}) {
      std::swap(edge_a, edge_b);
      std::swap(lane_a, lane_b);
    }
    membership.row_continuities.push_back(
        {continuity.node_id, edge_a, lane_a, edge_b, lane_b});
  }
  std::sort(membership.edge_ids.begin(), membership.edge_ids.end());
  std::sort(membership.row_continuities.begin(),
            membership.row_continuities.end(), [](const auto& a, const auto& b) {
              return std::tie(a.node_id, a.edge_a, a.lane_a, a.edge_b,
                              a.lane_b) <
                     std::tie(b.node_id, b.edge_a, b.lane_a, b.edge_b,
                              b.lane_b);
            });
  result.ok = true;
  result.value = std::move(membership);
  return result;
}

EditResult<std::vector<BackboneBundleSpec>>
CoreState::ResolveRouteBundleVariation(const RouteBundleVariationInput& input) const {
  EditResult<std::vector<BackboneBundleSpec>> result{};
  const EditResult<bool> descriptor =
      detail::validate_route_bundle_variation_descriptor(*this, input);
  if (!descriptor.ok) {
    result.error = descriptor.error;
    return result;
  }
  const PoleTypeDefinition* pole_type = find_pole_type(input.pole_type_id);

  int side_sign = input.preferred_side_sign;
  if (side_sign == 0) {
    for (const RandomBackboneBundleRule& rule : input.rules) {
      const BundleTemplate* bundle_template = find_bundle_template(rule.bundle_template_id);
      if (bundle_template == nullptr) {
        continue;
      }
      const int lane_count = bundle_template->count_rule == BundleCountRuleKind::kFixed
          ? bundle_template->fixed_count
          : (rule.conductor_count > 0 ? rule.conductor_count : bundle_template->default_count);
      const auto bands = generation::backbone::SelectPortPlacementBands(
          *pole_type, bundle_template->category, bundle_template->default_layer, lane_count);
      if (!bands.ok) {
        continue;
      }
      const auto non_center = std::ranges::find_if(bands.value, [](const PortPlacementBand& band) {
        return std::abs(band.lateral_center_m) > kLengthToleranceM;
      });
      if (non_center != bands.value.end()) {
        side_sign = non_center->lateral_center_m < 0.0 ? -1 : 1;
        break;
      }
    }
    if (side_sign == 0) {
      side_sign = -1;
    }
  }

  std::vector<BackboneBundleSpec> resolved{};
  std::unordered_set<std::uint64_t> placement_keys{};
  for (std::size_t rule_ordinal = 0; rule_ordinal < input.rules.size(); ++rule_ordinal) {
    const RandomBackboneBundleRule& rule = input.rules[rule_ordinal];
    const BundleTemplate* bundle_template = find_bundle_template(rule.bundle_template_id);
    if (bundle_template == nullptr) {
      result.error =
          "core internal: structurally valid variation lost Bundle template";
      return result;
    }
    const int conductor_count = rule.conductor_count > 0
        ? rule.conductor_count : bundle_template->default_count;

    const std::uint64_t count_seed = rule_seed(input.route_seed, rule_ordinal,
                                               rule.bundle_template_id, -1);
    const int instance_count = sample_count(count_seed, rule.min_instances, rule.max_instances);
    for (int instance_ordinal = 0; instance_ordinal < instance_count; ++instance_ordinal) {
      const std::uint64_t base_seed = rule_seed(input.route_seed, rule_ordinal,
                                                rule.bundle_template_id, instance_ordinal);
      const auto bands = generation::backbone::SelectPortPlacementBands(
          *pole_type, bundle_template->category, bundle_template->default_layer, conductor_count);
      if (!bands.ok) {
        result.error = bands.error;
        return result;
      }
      // Each concrete Bundle owns an independent point in the authored
      // envelope. The resolver does not invent a cross-Bundle clearance
      // policy or reject a density selected by its caller.
      const std::uint64_t placement_seed = support::hash_combine(base_seed, 0);
      const double height_m = sample_range(
          placement_seed, 0x484549474854ull,
          rule.height_min_m, rule.height_max_m);
      const double magnitude = sample_lateral_magnitude(
          placement_seed, rule.lateral_abs_min_m, rule.lateral_abs_max_m,
          bundle_template->category);
      const double lateral_m = static_cast<double>(side_sign) * magnitude;

      std::uint64_t placement_key = support::hash_combine(base_seed, 0x504c4143454d454eull) &
                                    kJavascriptExactIntegerMask;
      if (placement_key == 0) {
        placement_key = 1;
      }
      if (!placement_keys.insert(placement_key).second) {
        result.error = "core internal: route bundle variation placement key collision";
        return result;
      }
      BackboneBundleSpec spec{};
      spec.bundle_template_id = rule.bundle_template_id;
      spec.placement_key = placement_key;
      spec.layer = bundle_template->default_layer;
      spec.count = conductor_count;
      spec.placement_explicit = true;
      spec.height_m = height_m;
      spec.lateral_m = lateral_m;
      spec.spacing_m = bundle_template->default_spacing_m;
      resolved.push_back(spec);
    }
  }

  result.value = std::move(resolved);
  result.ok = true;
  return result;
}

EditResult<GenerateBackboneBundleVariationResult>
CoreState::GenerateBackboneBundleVariation(
    const BackboneSpec& spec,
    const RouteBundleVariationInput& descriptor) {
  EditResult<GenerateBackboneBundleVariationResult> result{};
  auto reject = [&](std::string error) {
    result.error = std::move(error);
    result.classify_error();
    return result;
  };
  if (!spec.bundles.empty()) {
    return reject("backbone invalid input: variation generation owns concrete Bundle specs");
  }
  if (spec.pole_type_id != kInvalidPoleTypeId &&
      spec.pole_type_id != descriptor.pole_type_id) {
    return reject("backbone invalid input: variation generation pole type mismatch");
  }

  CoreState trial = *this;
  EditResult<std::vector<BackboneBundleSpec>> resolved =
      trial.ResolveRouteBundleVariation(descriptor);
  if (!resolved.ok) return reject(resolved.error);
  if (resolved.value.empty()) {
    return reject("backbone unsupported: variation generation resolved no concrete Bundles");
  }
  BackboneSpec concrete = spec;
  concrete.pole_type_id = descriptor.pole_type_id;
  concrete.bundles = resolved.value;
  EditResult<GenerateBundleFromPathResult> generated =
      trial.GenerateFromBackboneSpec(concrete);
  if (!generated.ok) return reject(generated.error);
  if (generated.value.bundle_ids.size() != resolved.value.size()) {
    return reject("backbone internal: variation generation Bundle result is incomplete");
  }

  SavedBackboneBundleVariation variation{};
  variation.variation_id = trial.identity_.id_generator.next();
  variation.descriptor = descriptor;
  for (std::size_t i = 0; i < resolved.value.size(); ++i) {
    const ObjectId bundle_id = generated.value.bundle_ids[i];
    const Bundle* bundle = trial.view().bundles().find(bundle_id);
    if (bundle == nullptr ||
        bundle->placement_key != resolved.value[i].placement_key) {
      return reject("backbone internal: variation generation placement identity mismatch");
    }
    variation.instances.push_back({bundle->placement_key, bundle_id});
    const auto existing_membership = std::ranges::find_if(
        variation.memberships,
        [&](const SavedBackboneBundleVariationMembership& membership) {
          return membership.bundle_template_id == bundle->bundle_template_id &&
                 membership.conductor_count == bundle->conductor_count;
        });
    if (existing_membership == variation.memberships.end()) {
      auto captured =
          trial.capture_backbone_bundle_variation_membership(bundle_id);
      if (!captured.ok) return reject(captured.error);
      variation.memberships.push_back(std::move(captured.value));
    }
  }
  // A rule may resolve to zero instances initially and still become nonzero
  // through an explicit density Apply. Capture its exact membership while the
  // initial route path is known; later Apply must not infer a route or pairing.
  std::set<std::pair<BundleTemplateId, int>> captured_groups{};
  for (const SavedBackboneBundleVariationMembership& membership :
       variation.memberships) {
    captured_groups.emplace(membership.bundle_template_id,
                            membership.conductor_count);
  }
  for (std::size_t rule_index = 0; rule_index < descriptor.rules.size();
       ++rule_index) {
    const RandomBackboneBundleRule& rule = descriptor.rules[rule_index];
    // A rule authored as permanently empty is not a latent topology group.
    // Only a group that this descriptor can actually materialize needs an
    // exact membership source when its initial seeded sample happens to be 0.
    if (rule.max_instances == 0) continue;
    const BundleTemplate* bundle_template =
        trial.find_bundle_template(rule.bundle_template_id);
    if (bundle_template == nullptr) {
      return reject(
          "backbone internal: zero-instance membership template is missing");
    }
    const int conductor_count = rule.conductor_count > 0
                                    ? rule.conductor_count
                                    : bundle_template->default_count;
    if (captured_groups.contains({rule.bundle_template_id, conductor_count})) {
      continue;
    }

    RouteBundleVariationInput forced = descriptor;
    for (RandomBackboneBundleRule& candidate : forced.rules) {
      candidate.min_instances = 0;
      candidate.max_instances = 0;
    }
    forced.rules[rule_index].min_instances = 1;
    forced.rules[rule_index].max_instances = 1;
    EditResult<std::vector<BackboneBundleSpec>> temporary_resolved =
        trial.ResolveRouteBundleVariation(forced);
    if (!temporary_resolved.ok || temporary_resolved.value.size() != 1) {
      return reject(temporary_resolved.ok
                        ? "backbone internal: zero-instance membership did not resolve one temporary Bundle"
                        : temporary_resolved.error);
    }

    BackboneSpec temporary = spec;
    temporary.interval_m = 0.0;
    temporary.pole_type_id = descriptor.pole_type_id;
    temporary.bundles = temporary_resolved.value;
    temporary.node_bundle_modes.clear();
    temporary.path.polyline.clear();
    temporary.path.node_specs.clear();
    for (std::size_t node_index = 0;
         node_index < generated.value.generated_node_ids.size(); ++node_index) {
      const ObjectId node_id = generated.value.generated_node_ids[node_index];
      const SavedBackboneNode* node = trial.view().backbone_node(node_id);
      if (node == nullptr || node->has_source_edge) {
        return reject(
            "backbone unsupported: zero-instance variation membership requires a source-edge-free exact path");
      }
      BackboneInputSpec::NodeSpec node_spec{};
      node_spec.point_index = node_index;
      node_spec.support_kind = node->support_kind;
      node_spec.node_id = node_id;
      temporary.path.polyline.push_back(node->position);
      temporary.path.node_specs.push_back(node_spec);
    }
    EditResult<GenerateBundleFromPathResult> materialized =
        trial.GenerateFromBackboneSpec(temporary);
    if (!materialized.ok || materialized.value.bundle_ids.size() != 1) {
      return reject(materialized.ok
                        ? "backbone internal: zero-instance membership temporary Bundle is incomplete"
                        : materialized.error);
    }
    const ObjectId temporary_bundle_id = materialized.value.bundle_ids.front();
    EditResult<SavedBackboneBundleVariationMembership> captured =
        trial.capture_backbone_bundle_variation_membership(temporary_bundle_id);
    if (!captured.ok) return reject(captured.error);
    EditResult<bool> retired = trial.RetireBackboneBundle(temporary_bundle_id);
    if (!retired.ok) return reject(retired.error);
    variation.memberships.push_back(std::move(captured.value));
    captured_groups.emplace(rule.bundle_template_id, conductor_count);
  }
  std::sort(variation.instances.begin(), variation.instances.end(),
            [](const auto& a, const auto& b) {
              return a.placement_key < b.placement_key;
            });
  std::sort(variation.memberships.begin(), variation.memberships.end(),
            [](const auto& a, const auto& b) {
              return std::tie(a.bundle_template_id, a.conductor_count) <
                     std::tie(b.bundle_template_id, b.conductor_count);
            });
  trial.authoritative_.backbone_bundle_variations.push_back(variation);
  const ValidationResult validation = trial.Validate();
  for (const ValidationIssue& issue : validation.issues) {
    if (issue.severity == ValidationSeverity::kError) {
      return reject("backbone invalid input: variation generation validation failed: " +
                    issue.code + ": " + issue.message);
    }
  }

  *this = std::move(trial);
  result.ok = true;
  result.value.variation_id = variation.variation_id;
  result.value.generation = std::move(generated.value);
  result.change_set = std::move(generated.change_set);
  return result;
}

EditResult<GenerateBundleFromPathResult>
CoreState::ExtendBackboneBundleVariation(
    ObjectId variation_id, const BackboneSpec& spec) {
  EditResult<GenerateBundleFromPathResult> result{};
  auto reject = [&](std::string error) {
    result.error = std::move(error);
    result.classify_error();
    return result;
  };
  CoreState trial = *this;
  auto variation_it = std::ranges::find_if(
      trial.authoritative_.backbone_bundle_variations,
      [variation_id](const SavedBackboneBundleVariation& value) {
        return value.variation_id == variation_id;
      });
  if (variation_it ==
      trial.authoritative_.backbone_bundle_variations.end()) {
    return reject("core invalid input: backbone Bundle variation not found");
  }
  SavedBackboneBundleVariation updated = *variation_it;
  if (!spec.bundles.empty()) {
    return reject(
        "backbone invalid input: variation extension membership is Core-owned");
  }
  if (updated.instances.empty()) {
    return reject(
        "backbone unsupported: variation extension requires a live exact scope");
  }

  BackboneSpec concrete = spec;
  if (concrete.pole_type_id == kInvalidPoleTypeId) {
    concrete.pole_type_id = updated.descriptor.pole_type_id;
  } else if (concrete.pole_type_id != updated.descriptor.pole_type_id) {
    return reject(
        "backbone invalid input: variation extension pole type mismatch");
  }
  std::set<std::pair<BundleTemplateId, int>> live_groups{};
  for (const SavedBackboneBundleVariationInstance& instance :
       updated.instances) {
    const Bundle* bundle = trial.view().bundles().find(instance.bundle_id);
    if (bundle == nullptr || bundle->placement_key != instance.placement_key) {
      return reject(
          "backbone invalid input: variation extension exact scope is stale");
    }
    const BundleTemplate* bundle_template =
        trial.find_bundle_template(bundle->bundle_template_id);
    if (bundle_template == nullptr) {
      return reject(
          "backbone invalid input: variation extension Bundle template is missing");
    }
    BackboneBundleSpec bundle_spec{};
    bundle_spec.bundle_template_id = bundle->bundle_template_id;
    bundle_spec.placement_key = bundle->placement_key;
    bundle_spec.layer = bundle_template->default_layer;
    bundle_spec.count = bundle->conductor_count;
    bundle_spec.placement_explicit = bundle->placement_explicit;
    bundle_spec.height_m = bundle->height_m;
    bundle_spec.lateral_m = bundle->lateral_m;
    bundle_spec.spacing_m = bundle->spacing_override_m;
    bundle_spec.source_bundle_id = bundle->id;
    bundle_spec.existing_bundle_id = bundle->id;
    concrete.bundles.push_back(bundle_spec);
    live_groups.insert(
        {bundle->bundle_template_id, bundle->conductor_count});
  }
  variation_it->instances.clear();
  EditResult<GenerateBundleFromPathResult> generated =
      trial.GenerateFromBackboneSpec(concrete);
  if (!generated.ok) return reject(generated.error);

  std::vector<SavedBackboneBundleVariationMembership> refreshed{};
  for (const SavedBackboneBundleVariationInstance& instance :
       updated.instances) {
    const Bundle* bundle = trial.view().bundles().find(instance.bundle_id);
    if (bundle == nullptr || bundle->placement_key != instance.placement_key) {
      return reject(
          "backbone internal: variation extension changed exact Bundle identity");
    }
    const auto existing = std::ranges::find_if(
        refreshed,
        [&](const SavedBackboneBundleVariationMembership& membership) {
          return membership.bundle_template_id == bundle->bundle_template_id &&
                 membership.conductor_count == bundle->conductor_count;
        });
    auto captured =
        trial.capture_backbone_bundle_variation_membership(bundle->id);
    if (!captured.ok) return reject(captured.error);
    if (existing == refreshed.end()) {
      refreshed.push_back(std::move(captured.value));
    } else if (!same_membership_topology(*existing, captured.value)) {
      return reject(
          "backbone invalid input: variation extension live membership groups diverged");
    }
  }

  if (refreshed.size() < updated.memberships.size()) {
    if (generated.value.generated_node_ids.size() !=
        concrete.path.polyline.size()) {
      return reject(
          "backbone unsupported: variation extension cannot bind zero-instance membership to exact path nodes");
    }
    BackboneSpec exact_extension = spec;
    exact_extension.pole_type_id = concrete.pole_type_id;
    exact_extension.path.node_specs.clear();
    for (std::size_t index = 0;
         index < generated.value.generated_node_ids.size(); ++index) {
      const ObjectId node_id = generated.value.generated_node_ids[index];
      const auto node = std::ranges::find_if(
          trial.view().backbone().nodes,
          [node_id](const SavedBackboneNode& value) {
            return value.node_id == node_id;
          });
      if (node == trial.view().backbone().nodes.end()) {
        return reject(
            "backbone internal: variation extension exact path node is missing");
      }
      BackboneInputSpec::NodeSpec node_spec{};
      node_spec.point_index = index;
      node_spec.support_kind = node->support_kind;
      node_spec.node_id = node->node_id;
      exact_extension.path.node_specs.push_back(node_spec);
    }

    for (const SavedBackboneBundleVariationMembership& saved :
         updated.memberships) {
      if (live_groups.contains(
              {saved.bundle_template_id, saved.conductor_count})) {
        continue;
      }
      CoreState replay = trial;
      std::uint64_t temporary_key =
          support::hash_combine(
              support::hash_combine(variation_id,
                                    static_cast<std::uint64_t>(
                                        saved.bundle_template_id)),
              static_cast<std::uint64_t>(saved.conductor_count)) &
          kJavascriptExactIntegerMask;
      if (temporary_key == 0) temporary_key = 1;
      const BundleTemplate* bundle_template =
          replay.find_bundle_template(saved.bundle_template_id);
      if (bundle_template == nullptr) {
        return reject(
            "backbone invalid input: variation membership template is missing");
      }
      BackboneBundleSpec bundle_spec{};
      bundle_spec.bundle_template_id = saved.bundle_template_id;
      bundle_spec.placement_key = temporary_key;
      bundle_spec.layer = bundle_template->default_layer;
      bundle_spec.count = saved.conductor_count;
      bundle_spec.placement_explicit = false;
      EditResult<ObjectId> materialized =
          replay.add_backbone_bundle_instance_from_variation_membership(
              saved, bundle_spec);
      if (!materialized.ok) return reject(materialized.error);
      bundle_spec.source_bundle_id = materialized.value;
      bundle_spec.existing_bundle_id = materialized.value;
      BackboneSpec replay_extension = exact_extension;
      replay_extension.bundles.push_back(bundle_spec);
      EditResult<GenerateBundleFromPathResult> replayed =
          replay.GenerateFromBackboneSpec(replay_extension);
      if (!replayed.ok) return reject(replayed.error);
      auto captured = replay.capture_backbone_bundle_variation_membership(
          materialized.value);
      if (!captured.ok) return reject(captured.error);
      refreshed.push_back(std::move(captured.value));
    }
  }
  updated.memberships = std::move(refreshed);
  std::sort(updated.memberships.begin(), updated.memberships.end(),
            [](const auto& a, const auto& b) {
              return std::tie(a.bundle_template_id, a.conductor_count) <
                     std::tie(b.bundle_template_id, b.conductor_count);
            });
  auto placeholder = std::ranges::find_if(
      trial.authoritative_.backbone_bundle_variations,
      [variation_id](const SavedBackboneBundleVariation& value) {
        return value.variation_id == variation_id;
      });
  if (placeholder ==
      trial.authoritative_.backbone_bundle_variations.end()) {
    return reject(
        "backbone internal: variation extension ownership scope is missing");
  }
  *placeholder = std::move(updated);
  const ValidationResult validation = trial.Validate();
  for (const ValidationIssue& issue : validation.issues) {
    if (issue.severity == ValidationSeverity::kError) {
      return reject(
          "backbone invalid input: variation extension validation failed: " +
          issue.code + ": " + issue.message);
    }
  }
  *this = std::move(trial);
  return generated;
}

EditResult<bool> CoreState::ApplyBackboneBundleVariation(
    ObjectId variation_id,
    const RouteBundleVariationInput& descriptor) {
  EditResult<bool> result{};
  auto reject = [&](std::string error) {
    result.error = std::move(error);
    result.classify_error();
    return result;
  };
  if (view().backbone_bundle_variation(variation_id) == nullptr) {
    return reject("core invalid input: backbone Bundle variation not found");
  }

  CoreState trial = *this;
  auto variation_it = std::ranges::find_if(
      trial.authoritative_.backbone_bundle_variations,
      [variation_id](const SavedBackboneBundleVariation& value) {
        return value.variation_id == variation_id;
      });
  if (variation_it ==
      trial.authoritative_.backbone_bundle_variations.end()) {
    return reject("core internal: backbone Bundle variation scope is missing");
  }
  SavedBackboneBundleVariation updated = *variation_it;
  variation_it->instances.clear();
  EditResult<std::vector<BackboneBundleSpec>> resolved =
      trial.ResolveRouteBundleVariation(descriptor);
  if (!resolved.ok) return reject(resolved.error);

  BackboneBundleReconcileInput reconcile{};
  std::map<std::uint64_t, ObjectId> final_bundle_id_by_key{};
  ChangeSet replay_changes{};
  auto merge_replay_changes = [&](const ChangeSet& source) {
    auto append = [](std::vector<ObjectId>* target,
                     const std::vector<ObjectId>& values) {
      for (ObjectId id : values) {
        if (std::find(target->begin(), target->end(), id) == target->end()) {
          target->push_back(id);
        }
      }
    };
    append(&replay_changes.created_ids, source.created_ids);
    append(&replay_changes.updated_ids, source.updated_ids);
    append(&replay_changes.deleted_ids, source.deleted_ids);
  };
  for (const SavedBackboneBundleVariationInstance& instance :
       updated.instances) {
    const Bundle* bundle = trial.view().bundles().find(instance.bundle_id);
    if (bundle == nullptr || bundle->placement_key != instance.placement_key) {
      return reject(
          "backbone invalid input: variation current exact Bundle scope is stale");
    }
    reconcile.current_bundle_ids.push_back(instance.bundle_id);
    final_bundle_id_by_key.emplace(instance.placement_key, instance.bundle_id);
  }

  for (const BackboneBundleSpec& desired : resolved.value) {
    BackboneBundleReconcileEntry entry{};
    entry.desired = desired;
    const BundleTemplate* desired_template =
        trial.find_bundle_template(desired.bundle_template_id);
    if (desired_template == nullptr) {
      return reject(
          "backbone invalid input: variation desired Bundle template is missing");
    }
    // Reconcile uses count=0 to mean "use the fixed template count". The
    // resolver emits the concrete conductor count, so normalize only this API
    // representation before delegating to the generic reconcile operation.
    if (desired_template->count_rule == BundleCountRuleKind::kFixed) {
      entry.desired.count = 0;
    }
    const auto survivor = std::ranges::find_if(
        updated.instances,
        [&](const SavedBackboneBundleVariationInstance& instance) {
          return instance.placement_key == desired.placement_key;
        });
    if (survivor == updated.instances.end()) {
      const int desired_count =
          desired_template->count_rule == BundleCountRuleKind::kFixed
              ? desired_template->fixed_count
              : desired.count;
      for (const SavedBackboneBundleVariationInstance& candidate :
           updated.instances) {
        const Bundle* bundle = trial.view().bundles().find(candidate.bundle_id);
        if (bundle != nullptr &&
            bundle->bundle_template_id == desired.bundle_template_id &&
            bundle->conductor_count == desired_count) {
          entry.anchor_bundle_id = bundle->id;
          break;
        }
      }
      if (entry.anchor_bundle_id == kInvalidObjectId) {
        const auto membership = std::ranges::find_if(
            updated.memberships,
            [&](const SavedBackboneBundleVariationMembership& value) {
              return value.bundle_template_id == desired.bundle_template_id &&
                     value.conductor_count == desired_count;
            });
        if (membership == updated.memberships.end()) {
          return reject(
              "backbone unsupported: initial zero-instance variation has no exact membership source");
        }
        EditResult<ObjectId> replayed =
            trial.add_backbone_bundle_instance_from_variation_membership(
                *membership, desired);
        if (!replayed.ok) return reject(replayed.error);
        reconcile.current_bundle_ids.push_back(replayed.value);
        final_bundle_id_by_key.emplace(desired.placement_key, replayed.value);
        merge_replay_changes(replayed.change_set);
      }
    }
    reconcile.desired_bundles.push_back(entry);
  }

  std::unordered_set<ObjectId> bundles_before_reconcile{};
  for (const Bundle& bundle : trial.view().bundles().items()) {
    bundles_before_reconcile.insert(bundle.id);
  }
  EditResult<bool> reconciled =
      trial.ReconcileBackboneBundleInstances(reconcile);
  if (!reconciled.ok) return reject(reconciled.error);

  updated.descriptor = descriptor;
  updated.instances.clear();
  std::set<std::pair<BundleTemplateId, int>> descriptor_groups{};
  for (const RandomBackboneBundleRule& rule : descriptor.rules) {
    const BundleTemplate* bundle_template =
        trial.find_bundle_template(rule.bundle_template_id);
    if (bundle_template == nullptr) {
      return reject(
          "backbone invalid input: variation descriptor template is missing");
    }
    descriptor_groups.emplace(
        rule.bundle_template_id,
        rule.conductor_count > 0 ? rule.conductor_count
                                 : bundle_template->default_count);
  }
  std::vector<SavedBackboneBundleVariationMembership> live_memberships{};
  for (const BackboneBundleSpec& desired : resolved.value) {
    auto final_id = final_bundle_id_by_key.find(desired.placement_key);
    if (final_id == final_bundle_id_by_key.end()) {
      for (const Bundle& bundle : trial.view().bundles().items()) {
        if (bundles_before_reconcile.contains(bundle.id) ||
            bundle.placement_key != desired.placement_key ||
            bundle.bundle_template_id != desired.bundle_template_id) {
          continue;
        }
        if (!final_bundle_id_by_key
                 .emplace(desired.placement_key, bundle.id)
                 .second) {
          return reject(
              "backbone internal: variation added placement identity is ambiguous within reconcile scope");
        }
      }
      final_id = final_bundle_id_by_key.find(desired.placement_key);
    }
    const Bundle* matched =
        final_id == final_bundle_id_by_key.end()
            ? nullptr
            : trial.view().bundles().find(final_id->second);
    if (matched == nullptr) {
      return reject(
          "backbone internal: variation final concrete Bundle is missing");
    }
    updated.instances.push_back({matched->placement_key, matched->id});
    auto captured =
        trial.capture_backbone_bundle_variation_membership(matched->id);
    if (!captured.ok) return reject(captured.error);
    const auto membership = std::ranges::find_if(
        live_memberships,
        [&](const SavedBackboneBundleVariationMembership& value) {
          return value.bundle_template_id == matched->bundle_template_id &&
                 value.conductor_count == matched->conductor_count;
        });
    if (membership == live_memberships.end()) {
      live_memberships.push_back(std::move(captured.value));
    } else if (!same_membership_topology(*membership, captured.value)) {
      return reject(
          "backbone unsupported: variation instances in one rule group do not share exact saved membership");
    }
  }
  for (const SavedBackboneBundleVariationMembership& membership :
       updated.memberships) {
    if (!descriptor_groups.contains(
            {membership.bundle_template_id, membership.conductor_count})) {
      continue;
    }
    const auto live = std::ranges::find_if(
        live_memberships,
        [&](const SavedBackboneBundleVariationMembership& value) {
          return value.bundle_template_id == membership.bundle_template_id &&
                 value.conductor_count == membership.conductor_count;
        });
    if (live == live_memberships.end()) {
      live_memberships.push_back(membership);
    }
  }
  updated.memberships = std::move(live_memberships);
  std::sort(updated.instances.begin(), updated.instances.end(),
            [](const auto& a, const auto& b) {
              return a.placement_key < b.placement_key;
            });
  std::sort(updated.memberships.begin(), updated.memberships.end(),
            [](const auto& a, const auto& b) {
              return std::tie(a.bundle_template_id, a.conductor_count) <
                     std::tie(b.bundle_template_id, b.conductor_count);
            });
  auto placeholder = std::ranges::find_if(
      trial.authoritative_.backbone_bundle_variations,
      [variation_id](const SavedBackboneBundleVariation& value) {
        return value.variation_id == variation_id;
      });
  if (placeholder ==
      trial.authoritative_.backbone_bundle_variations.end()) {
    return reject(
        "backbone internal: variation Apply ownership scope is missing");
  }
  *placeholder = std::move(updated);
  trial.cleanup_orphan_backbone_skeleton(&reconciled.change_set);

  const ValidationResult validation = trial.Validate();
  for (const ValidationIssue& issue : validation.issues) {
    if (issue.severity == ValidationSeverity::kError) {
      return reject(
          "backbone invalid input: variation Apply validation failed: " +
          issue.code + ": " + issue.message);
    }
  }
  *this = std::move(trial);
  result.ok = true;
  result.value = reconciled.value ||
                 !reconciled.change_set.deleted_ids.empty() ||
                 !replay_changes.created_ids.empty() ||
                 !replay_changes.updated_ids.empty() ||
                 !replay_changes.deleted_ids.empty();
  result.change_set = std::move(reconciled.change_set);
  for (ObjectId id : replay_changes.created_ids) {
    if (std::find(result.change_set.created_ids.begin(),
                  result.change_set.created_ids.end(), id) ==
        result.change_set.created_ids.end()) {
      result.change_set.created_ids.push_back(id);
    }
  }
  for (ObjectId id : replay_changes.updated_ids) {
    if (std::find(result.change_set.updated_ids.begin(),
                  result.change_set.updated_ids.end(), id) ==
        result.change_set.updated_ids.end()) {
      result.change_set.updated_ids.push_back(id);
    }
  }
  for (ObjectId id : replay_changes.deleted_ids) {
    if (std::find(result.change_set.deleted_ids.begin(),
                  result.change_set.deleted_ids.end(), id) ==
        result.change_set.deleted_ids.end()) {
      result.change_set.deleted_ids.push_back(id);
    }
  }
  return result;
}

} // namespace city::wire
