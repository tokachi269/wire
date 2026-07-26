#include "city/wire/style_context.hpp"

#include <algorithm>
#include <cmath>

namespace city::wire {

namespace {

constexpr std::uint64_t kStyleSeedSalt = 0x9E3779B97F4A7C15ull;
constexpr std::uint64_t kDistrictSalt = 0xD1B54A32D192ED03ull;
constexpr std::uint64_t kRouteSalt = 0x94D049BB133111EBull;
constexpr std::uint64_t kClusterSalt = 0xF1357AEA2E62A9C5ull;
constexpr std::uint64_t kObjectSalt = 0xC2B2AE3D27D4EB4Full;

std::uint64_t mix_u64(std::uint64_t value) {
  value += kStyleSeedSalt;
  value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ull;
  value = (value ^ (value >> 27)) * 0x94D049BB133111EBull;
  value ^= (value >> 31);
  return value;
}

std::uint64_t combine_u64(std::uint64_t seed, std::uint64_t value) {
  return mix_u64(seed ^ mix_u64(value + kStyleSeedSalt));
}

double unit_from_seed(std::uint64_t seed) {
  constexpr double kInv = 1.0 / static_cast<double>(1ull << 53);
  return static_cast<double>((seed >> 11) & ((1ull << 53) - 1ull)) * kInv;
}

double centered_from_seed(std::uint64_t seed) { return unit_from_seed(seed) * 2.0 - 1.0; }

double clamp01(double value) { return std::clamp(value, 0.0, 1.0); }

CableMaterialStyleKind cable_family_for_route(const ContextProfile& profile, const StyleRouteKey& route_key,
                                              double route_service_mix) {
  if (route_key.category == ConnectionCategory::kHighVoltage) {
    return CableMaterialStyleKind::kBareConductor;
  }
  if (route_key.category == ConnectionCategory::kOptical) {
    return CableMaterialStyleKind::kOptical;
  }
  if (route_key.category == ConnectionCategory::kCommunication) {
    return (profile.age >= 0.45 || route_service_mix >= 0.55) ? CableMaterialStyleKind::kInsulated
                                                               : CableMaterialStyleKind::kGeneric;
  }
  if (profile.age >= 0.45) {
    return CableMaterialStyleKind::kInsulated;
  }
  return CableMaterialStyleKind::kGeneric;
}

CableAttachmentStyleHint attachment_family_for_route(const StyleRouteKey& route_key, double route_clutter,
                                                     double route_regularity) {
  if (route_key.category == ConnectionCategory::kHighVoltage) {
    return CableAttachmentStyleHint::kViaAttachment;
  }
  if (route_key.category == ConnectionCategory::kOptical) {
    return CableAttachmentStyleHint::kViaAttachment;
  }
  return (route_clutter > 0.55 || route_regularity < 0.35) ? CableAttachmentStyleHint::kViaAttachment
                                                            : CableAttachmentStyleHint::kDirectThrough;
}

VariationScope resolve_variation_scope(const ContextProfile& profile, const StyleRouteKey& route_key,
                                       const StyleObjectKey& object_key) {
  VariationScope scope{};
  scope.district_seed = combine_u64(profile.style_seed, kDistrictSalt);
  std::uint64_t route_seed = combine_u64(scope.district_seed, kRouteSalt);
  route_seed = combine_u64(route_seed, route_key.family_id);
  route_seed = combine_u64(route_seed, static_cast<std::uint64_t>(route_key.bundle_template_id));
  route_seed = combine_u64(route_seed, static_cast<std::uint64_t>(route_key.category));
  route_seed = combine_u64(route_seed, static_cast<std::uint64_t>(route_key.flow_kind));
  scope.route_seed = route_seed;

  const std::uint32_t cluster_index = object_key.segment_index / 4u;
  std::uint64_t cluster_seed = combine_u64(route_seed, kClusterSalt);
  cluster_seed = combine_u64(cluster_seed, cluster_index);
  scope.cluster_seed = cluster_seed;

  std::uint64_t object_seed = combine_u64(route_seed, kObjectSalt);
  object_seed = combine_u64(object_seed, object_key.segment_index);
  object_seed = combine_u64(object_seed, object_key.lane_index);
  object_seed = combine_u64(object_seed, static_cast<std::uint64_t>(object_key.kind));
  object_seed = combine_u64(object_seed, object_key.ordinal);
  object_seed = combine_u64(object_seed, object_key.is_start_endpoint ? 1ull : 0ull);
  scope.object_seed = object_seed;
  return scope;
}

DistrictStyle resolve_district_style(const ContextProfile& profile, std::uint64_t district_seed) {
  DistrictStyle style{};
  style.age = clamp01(profile.age + centered_from_seed(combine_u64(district_seed, 1)) * 0.08);
  style.clutter = clamp01(profile.clutter + centered_from_seed(combine_u64(district_seed, 2)) * 0.08);
  style.regularity = clamp01(profile.regularity + centered_from_seed(combine_u64(district_seed, 3)) * 0.08);
  style.service_mix = clamp01(profile.service_mix + centered_from_seed(combine_u64(district_seed, 4)) * 0.08);
  const StyleRouteKey seed_route{0, kInvalidBundleTemplateId, ConnectionCategory::kLowVoltage, BackboneFlowKind::kMain};
  style.cable_family = cable_family_for_route(profile, seed_route, style.service_mix);
  style.attachment_family =
      attachment_family_for_route(seed_route, style.clutter, style.regularity);
  return style;
}

RouteStyle resolve_route_style(const ContextProfile& profile, const DistrictStyle& district, const StyleRouteKey& route_key,
                               std::uint64_t route_seed) {
  RouteStyle style{};
  style.key = route_key;
  style.age_bias = centered_from_seed(combine_u64(route_seed, 10)) * 0.15;
  style.clutter_bias = centered_from_seed(combine_u64(route_seed, 11)) * 0.20;
  style.regularity_bias = centered_from_seed(combine_u64(route_seed, 12)) * 0.18;
  style.service_mix_bias = centered_from_seed(combine_u64(route_seed, 13)) * 0.20;
  style.sag_bias = centered_from_seed(combine_u64(route_seed, 14)) * 0.12;
  const double route_service_mix = clamp01(district.service_mix + style.service_mix_bias);
  const double route_clutter = clamp01(district.clutter + style.clutter_bias);
  const double route_regularity = clamp01(district.regularity + style.regularity_bias);
  style.cable_family = cable_family_for_route(profile, route_key, route_service_mix);
  style.attachment_family = attachment_family_for_route(route_key, route_clutter, route_regularity);
  return style;
}

ClusterStyle resolve_cluster_style(const StyleRouteKey& route_key, std::uint32_t cluster_index, std::uint64_t cluster_seed) {
  ClusterStyle style{};
  style.key.route = route_key;
  style.key.cluster_index = cluster_index;
  style.clutter_bias = centered_from_seed(combine_u64(cluster_seed, 20)) * 0.10;
  style.service_mix_bias = centered_from_seed(combine_u64(cluster_seed, 21)) * 0.10;
  style.family_mix = clamp01(unit_from_seed(combine_u64(cluster_seed, 22)));
  return style;
}

ObjectVariation resolve_object_variation(const StyleObjectKey& object_key, std::uint64_t object_seed,
                                         double regularity, double clutter) {
  ObjectVariation variation{};
  variation.key = object_key;
  const double offset_scale = 0.02 + clutter * 0.04 + (1.0 - regularity) * 0.02;
  variation.local_offset_m = {
      centered_from_seed(combine_u64(object_seed, 30)) * offset_scale,
      centered_from_seed(combine_u64(object_seed, 31)) * offset_scale,
      centered_from_seed(combine_u64(object_seed, 32)) * (offset_scale * 0.5),
  };
  variation.sag_delta_m = centered_from_seed(combine_u64(object_seed, 33)) * 0.08;
  variation.attachment_offset_m = centered_from_seed(combine_u64(object_seed, 34)) * 0.03;
  variation.choice_bias = centered_from_seed(combine_u64(object_seed, 35)) * 0.5;
  return variation;
}

} // namespace

ResolvedStyleContext ResolveStyleContext(const ContextProfile& profile, const StyleRouteKey& route_key,
                                         const StyleObjectKey& object_key) {
  ResolvedStyleContext resolved{};
  resolved.profile = profile;
  resolved.scope = resolve_variation_scope(profile, route_key, object_key);
  resolved.district = resolve_district_style(profile, resolved.scope.district_seed);
  resolved.route = resolve_route_style(profile, resolved.district, route_key, resolved.scope.route_seed);
  const std::uint32_t cluster_index = object_key.segment_index / 4u;
  resolved.cluster = resolve_cluster_style(route_key, cluster_index, resolved.scope.cluster_seed);

  const double regularity =
      clamp01(resolved.district.regularity + resolved.route.regularity_bias + resolved.cluster.clutter_bias * -0.25);
  const double clutter =
      clamp01(resolved.district.clutter + resolved.route.clutter_bias + resolved.cluster.clutter_bias);
  resolved.object = resolve_object_variation(object_key, resolved.scope.object_seed, regularity, clutter);
  return resolved;
}

} // namespace city::wire
