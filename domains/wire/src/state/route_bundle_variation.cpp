#include "city/wire/core_state.hpp"

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

} // namespace city::wire
