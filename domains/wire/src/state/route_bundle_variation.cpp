#include "city/wire/core_state.hpp"
#include "city/wire/core_view.hpp"

#include "../generation/backbone/emit_shared.hpp"
#include "../generation/backbone/route_support.hpp"
#include "../support/hash_mix.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace city::wire {
namespace {

constexpr int kPlacementAttempts = 64;
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

struct PlacedPoint {
  double lateral_m = 0.0;
  double height_m = 0.0;
  double min_spacing_m = 0.0;
};

bool separated(const PlacedPoint& candidate, const std::vector<PlacedPoint>& placed) {
  return std::ranges::all_of(placed, [&](const PlacedPoint& existing) {
    const double required = std::max(candidate.min_spacing_m, existing.min_spacing_m);
    const double dy = candidate.lateral_m - existing.lateral_m;
    const double dz = candidate.height_m - existing.height_m;
    return dy * dy + dz * dz + kLengthSquaredToleranceM2 >= required * required;
  });
}

} // namespace

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
    membership.edges.push_back({edge->edge_id, edge->node_a, edge->node_b,
                                edge->dir, edge->lateral_offset_m});
  }
  if (membership.edges.empty()) {
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
  std::sort(membership.edges.begin(), membership.edges.end(),
            [](const auto& a, const auto& b) { return a.edge_id < b.edge_id; });
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
  const PoleTypeDefinition* pole_type = find_pole_type(input.pole_type_id);
  if (pole_type == nullptr) {
    result.error = "core invalid input: route bundle variation pole template is missing";
    return result;
  }
  if (input.preferred_side_sign < -1 || input.preferred_side_sign > 1) {
    result.error = "core invalid input: preferred side sign must be -1, 0, or +1";
    return result;
  }
  if (input.rules.empty()) {
    result.error = "core invalid input: route bundle variation rules are empty";
    return result;
  }

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
  std::vector<PlacedPoint> placed{};
  std::unordered_set<std::uint64_t> placement_keys{};
  for (std::size_t rule_ordinal = 0; rule_ordinal < input.rules.size(); ++rule_ordinal) {
    const RandomBackboneBundleRule& rule = input.rules[rule_ordinal];
    const BundleTemplate* bundle_template = find_bundle_template(rule.bundle_template_id);
    if (bundle_template == nullptr) {
      result.error = "core invalid input: route bundle variation bundle template is missing";
      return result;
    }
    if (rule.min_instances < 0 || rule.max_instances < rule.min_instances ||
        rule.max_instances <= 0 || !std::isfinite(rule.height_min_m) ||
        !std::isfinite(rule.height_max_m) || rule.height_max_m < rule.height_min_m ||
        !std::isfinite(rule.lateral_abs_min_m) || !std::isfinite(rule.lateral_abs_max_m) ||
        rule.lateral_abs_min_m < 0.0 || rule.lateral_abs_max_m < rule.lateral_abs_min_m ||
        !std::isfinite(rule.min_spacing_m) || rule.min_spacing_m < 0.0 ||
        rule.conductor_count < 0) {
      result.error = "core invalid input: route bundle variation rule is invalid";
      return result;
    }
    const int conductor_count = rule.conductor_count > 0
        ? rule.conductor_count : bundle_template->default_count;
    if (conductor_count <= 0 ||
        (bundle_template->count_rule == BundleCountRuleKind::kFixed &&
         conductor_count != bundle_template->fixed_count) ||
        (bundle_template->count_rule == BundleCountRuleKind::kRange &&
         (conductor_count < bundle_template->min_count ||
          conductor_count > bundle_template->max_count))) {
      result.error = "core invalid input: route bundle variation conductor count is invalid";
      return result;
    }

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
      bool accepted = false;
      PlacedPoint candidate{};
      for (int attempt = 0; attempt < kPlacementAttempts; ++attempt) {
        const std::uint64_t attempt_seed = support::hash_combine(base_seed, static_cast<std::uint64_t>(attempt));
        candidate.height_m = sample_range(attempt_seed, 0x484549474854ull,
                                          rule.height_min_m, rule.height_max_m);
        const double magnitude = sample_lateral_magnitude(
            attempt_seed, rule.lateral_abs_min_m, rule.lateral_abs_max_m,
            bundle_template->category);
        candidate.lateral_m = static_cast<double>(side_sign) * magnitude;
        candidate.min_spacing_m = rule.min_spacing_m;
        if (separated(candidate, placed)) {
          accepted = true;
          break;
        }
      }
      if (!accepted) {
        result.error = "core unsupported: route bundle variation cannot satisfy minimum spacing";
        return result;
      }

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
      spec.height_m = candidate.height_m;
      spec.lateral_m = candidate.lateral_m;
      spec.spacing_m = bundle_template->default_spacing_m;
      resolved.push_back(spec);
      placed.push_back(candidate);
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

} // namespace city::wire
