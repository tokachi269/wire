#include "span_visual_assembly.hpp"

#include "city/wire/core_view.hpp"
#include "city/wire/support/numeric_tolerances.hpp"
#include "city/wire/coord_utils.hpp"
#include "../../geometry/detail_curve_postprocess.hpp"
#include "out.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <unordered_map>

namespace city::wire::generation::backbone {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

struct path_sample {
  Vec3d point{};
  Vec3d tangent{1.0, 0.0, 0.0};
  double distance = 0.0;
};

std::uint64_t mix_seed(std::uint64_t value) {
  value += 0x9E3779B97F4A7C15ull;
  value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ull;
  value = (value ^ (value >> 27)) * 0x94D049BB133111EBull;
  return value ^ (value >> 31);
}

Vec3d unit_or(const Vec3d& value, const Vec3d& fallback) {
  // R5 fallback: legitimate degenerate visual sampling. Support/helix visuals keep finite frames for clamped samples.
  const double length = Length(value);
  return length > kLengthToleranceM ? ScaleVec(value, 1.0 / length) : fallback;
}

AABBd bounds_for(const std::vector<Vec3d>& points) {
  AABBd bounds{};
  if (points.empty()) return bounds;
  bounds.min = points.front();
  bounds.max = points.front();
  for (const Vec3d& point : points) {
    bounds.min.x = std::min(bounds.min.x, point.x); bounds.min.y = std::min(bounds.min.y, point.y); bounds.min.z = std::min(bounds.min.z, point.z);
    bounds.max.x = std::max(bounds.max.x, point.x); bounds.max.y = std::max(bounds.max.y, point.y); bounds.max.z = std::max(bounds.max.z, point.z);
  }
  return bounds;
}

double path_length(const std::vector<Vec3d>& samples) {
  double length = 0.0;
  for (std::size_t i = 1; i < samples.size(); ++i) length += Length(samples[i] - samples[i - 1]);
  return length;
}

path_sample sample_at_distance(const std::vector<Vec3d>& samples, double target) {
  if (samples.empty()) return {};
  if (samples.size() == 1) return {samples.front(), {}, 0.0};
  const double total = path_length(samples);
  target = std::clamp(target, 0.0, total);
  double accumulated = 0.0;
  for (std::size_t i = 1; i < samples.size(); ++i) {
    const Vec3d segment = samples[i] - samples[i - 1];
    const double length = Length(segment);
    if (target <= accumulated + length || i + 1 == samples.size()) {
      const double fraction = length <= kLengthToleranceM ? 0.0 : (target - accumulated) / length;
      return {samples[i - 1] + ScaleVec(segment, std::clamp(fraction, 0.0, 1.0)),
              unit_or(segment, {1.0, 0.0, 0.0}), target};
    }
    accumulated += length;
  }
  return {samples.back(), unit_or(samples.back() - samples[samples.size() - 2], {1.0, 0.0, 0.0}), total};
}

path_sample sample_normalized(const std::vector<Vec3d>& samples, double t) {
  return sample_at_distance(samples, path_length(samples) * std::clamp(t, 0.0, 1.0));
}

void frame_for(const Vec3d& tangent, Vec3d* lateral, Vec3d* up) {
  *lateral = unit_or(Cross({0.0, 0.0, 1.0}, tangent), {0.0, 1.0, 0.0});
  *up = unit_or(Cross(tangent, *lateral), {0.0, 0.0, 1.0});
}

Vec3d projected_offset(const Vec3d& point, const path_sample& support) {
  const Vec3d raw = point - support.point;
  return raw - ScaleVec(support.tangent, Dot(raw, support.tangent));
}

double top_inset(const VisualCurvePart& support, double clearance) {
  return support.wire_radius_m * 2.0 + clearance;
}

double required_member_radius(const VisualCurvePart& support, const VisualCurvePart& member,
                              double clearance) {
  return support.wire_radius_m + member.wire_radius_m + clearance;
}

Vec3d place_below_support(const Vec3d& offset, const Vec3d& up, double minimum_drop) {
  return offset - ScaleVec(up, std::max(0.0, Dot(offset, up) + minimum_drop));
}

bool members_fit_radius(const VisualCurvePart& support, const std::vector<VisualCurvePart*>& members,
                        double clearance, double radius) {
  const double inset = top_inset(support, clearance);
  if (radius + kLengthToleranceM < inset) return false;
  for (const VisualCurvePart* member : members) {
    if (member->samples.empty()) continue;
    for (std::size_t index = 0; index < member->samples.size(); ++index) {
      const double t = member->samples.size() < 2 ? 0.0 :
          static_cast<double>(index) / static_cast<double>(member->samples.size() - 1);
      const path_sample anchor = sample_normalized(support.samples, t);
      Vec3d lateral{};
      Vec3d up{};
      frame_for(anchor.tangent, &lateral, &up);
      Vec3d offset = projected_offset(member->samples[index], anchor);
      offset = place_below_support(offset, up, required_member_radius(support, *member, clearance));
      const Vec3d axis = anchor.point - ScaleVec(up, radius - inset);
      if (Length(anchor.point + offset - axis) + required_member_radius(support, *member, clearance) > radius + kLengthToleranceM) {
        return false;
      }
    }
  }
  return true;
}

double auto_fit_radius(const VisualCurvePart& support, const std::vector<VisualCurvePart*>& members,
                       double clearance) {
  double low = std::max(0.01, top_inset(support, clearance));
  double high = low;
  while (!members_fit_radius(support, members, clearance, high)) high *= 2.0;
  for (int iteration = 0; iteration < 32; ++iteration) {
    const double middle = (low + high) * 0.5;
    if (members_fit_radius(support, members, clearance, middle)) high = middle;
    else low = middle;
  }
  return high;
}

double smoothstep(double value) {
  value = std::clamp(value, 0.0, 1.0);
  return value * value * (3.0 - 2.0 * value);
}

double endpoint_envelope(double distance, double total, double trim) {
  if (trim <= kLengthToleranceM) return 1.0;
  return smoothstep(std::min(distance / trim, (total - distance) / trim));
}

double span_envelope(double distance, double total);

void apply_member_twist(const SpanVisualAssemblyTemplate& settings, std::vector<VisualCurvePart*> members) {
  if (settings.member_twist_turns_per_meter == 0.0 || members.size() < 2) return;
  std::size_t sample_count = members.front()->samples.size();
  for (const VisualCurvePart* member : members) sample_count = std::min(sample_count, member->samples.size());
  if (sample_count < 2) return;

  std::vector<std::vector<Vec3d>> original{};
  for (const VisualCurvePart* member : members) original.push_back(member->samples);
  double distance = 0.0;
  const double total = path_length(original.front());
  for (std::size_t index = 0; index < sample_count; ++index) {
    if (index > 0) distance += Length(original.front()[index] - original.front()[index - 1]);
    Vec3d centroid{};
    for (const VisualCurvePart* member : members) centroid = centroid + member->samples[index];
    centroid = ScaleVec(centroid, 1.0 / static_cast<double>(members.size()));
    const Vec3d tangent = index + 1 < sample_count
        ? unit_or(original.front()[index + 1] - original.front()[index], {1.0, 0.0, 0.0})
        : unit_or(original.front()[index] - original.front()[index - 1], {1.0, 0.0, 0.0});
    Vec3d lateral{};
    Vec3d up{};
    frame_for(tangent, &lateral, &up);
    const double angle = settings.member_twist_phase + kTwoPi * settings.member_twist_turns_per_meter * distance;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const double envelope = span_envelope(distance, total);
    for (std::size_t member_index = 0; member_index < members.size(); ++member_index) {
      const Vec3d offset = original[member_index][index] - centroid;
      const double x = Dot(offset, lateral);
      const double y = Dot(offset, up);
      const Vec3d rotated = centroid + ScaleVec(tangent, Dot(offset, tangent)) +
          ScaleVec(lateral, x * cosine - y * sine) + ScaleVec(up, x * sine + y * cosine);
      members[member_index]->samples[index] = original[member_index][index] +
          ScaleVec(rotated - original[member_index][index], envelope);
    }
  }
  for (VisualCurvePart* member : members) {
    member->boundary_a = member->samples.front();
    member->boundary_b = member->samples.back();
    member->bounds = bounds_for(member->samples);
  }
}

double span_envelope(double distance, double total) {
  if (total <= kLengthToleranceM) return 0.0;
  const double t = std::clamp(distance / total, 0.0, 1.0);
  const double rise_and_fall = 4.0 * t * (1.0 - t);
  return rise_and_fall * rise_and_fall;
}

double unit_value(std::uint64_t seed) {
  return static_cast<double>(seed >> 11) / static_cast<double>(1ull << 53);
}

std::uint64_t assembly_seed(const Bundle& bundle, const BundleTemplate& bundle_template,
                            std::size_t lane_index) {
  std::uint64_t seed = bundle.placement_key == 0
      ? static_cast<std::uint64_t>(bundle_template.id)
      : bundle.placement_key;
  seed = mix_seed(seed ^ std::bit_cast<std::uint64_t>(bundle.height_m));
  seed = mix_seed(seed ^ std::bit_cast<std::uint64_t>(bundle.lateral_m));
  seed = mix_seed(seed ^ (static_cast<std::uint64_t>(lane_index) << 32));
  return seed;
}

void apply_center_wander(const SpanVisualAssemblyTemplate& settings, std::uint64_t seed,
                         VisualCurvePart* center) {
  if (center == nullptr || center->samples.size() < 3 ||
      settings.center_wander_amplitude_m <= 0.0 ||
      settings.center_wander_wavelength_m <= kLengthToleranceM) {
    return;
  }
  const std::vector<Vec3d> original = center->samples;
  const double total = path_length(original);
  const double phase = settings.member_wander_phase_bias + kTwoPi * unit_value(seed);
  double distance = 0.0;
  for (std::size_t index = 0; index < original.size(); ++index) {
    if (index > 0) distance += Length(original[index] - original[index - 1]);
    const Vec3d tangent = index + 1 < original.size()
        ? unit_or(original[index + 1] - original[index], {1.0, 0.0, 0.0})
        : unit_or(original[index] - original[index - 1], {1.0, 0.0, 0.0});
    Vec3d lateral{};
    Vec3d up{};
    frame_for(tangent, &lateral, &up);
    const double primary = phase + kTwoPi * distance / settings.center_wander_wavelength_m;
    const double secondary = phase * 0.73 + kTwoPi * distance /
        (settings.center_wander_wavelength_m * 1.71);
    const double envelope = span_envelope(distance, total);
    const double lateral_offset = settings.center_wander_amplitude_m * envelope *
        (0.68 * std::sin(primary) + 0.32 * std::sin(secondary));
    const double vertical_offset = settings.center_wander_amplitude_m * 0.35 * envelope *
        (0.70 * std::cos(primary * 0.83) + 0.30 * std::sin(secondary * 1.19));
    center->samples[index] = original[index] + ScaleVec(lateral, lateral_offset) +
        ScaleVec(up, vertical_offset);
  }
  center->samples.front() = original.front();
  center->samples.back() = original.back();
  center->boundary_a = original.front();
  center->boundary_b = original.back();
  center->bounds = bounds_for(center->samples);
}

int visual_member_count(const SpanVisualAssemblyTemplate& settings, std::uint64_t seed) {
  const int range = settings.visual_member_count_max - settings.visual_member_count_min + 1;
  return settings.visual_member_count_min +
      static_cast<int>(mix_seed(seed ^ 0x56495355414c4d43ull) % static_cast<std::uint64_t>(range));
}

std::vector<VisualCurvePart> make_visual_members(const SpanVisualAssemblyTemplate& settings,
                                                 std::uint64_t seed,
                                                 const VisualCurvePart& center) {
  const int count = visual_member_count(settings, seed);
  std::vector<VisualCurvePart> members(static_cast<std::size_t>(count), center);
  const double total = path_length(center.samples);
  for (int member_index = 0; member_index < count; ++member_index) {
    VisualCurvePart& member = members[static_cast<std::size_t>(member_index)];
    member.section_count = static_cast<std::size_t>(count);
    member.section_key.instance_index = static_cast<std::size_t>(member_index);
    const double baseline =
        (static_cast<double>(member_index) - static_cast<double>(count - 1) * 0.5) *
        settings.visual_member_spacing_m;
    const std::uint64_t member_seed = mix_seed(seed ^ static_cast<std::uint64_t>(member_index + 1));
    const double phase = settings.member_wander_phase_bias + kTwoPi * unit_value(member_seed);
    const double relative_amplitude = settings.visual_member_spacing_m * settings.member_wander_ratio;
    double distance = 0.0;
    for (std::size_t index = 0; index < center.samples.size(); ++index) {
      if (index > 0) distance += Length(center.samples[index] - center.samples[index - 1]);
      const Vec3d tangent = index + 1 < center.samples.size()
          ? unit_or(center.samples[index + 1] - center.samples[index], {1.0, 0.0, 0.0})
          : unit_or(center.samples[index] - center.samples[index - 1], {1.0, 0.0, 0.0});
      Vec3d lateral{};
      Vec3d up{};
      frame_for(tangent, &lateral, &up);
      const double envelope = span_envelope(distance, total);
      double lateral_offset = baseline;
      double vertical_offset = 0.0;
      if (relative_amplitude > 0.0 &&
          settings.member_wander_wavelength_m > kLengthToleranceM) {
        const double primary = phase + kTwoPi * distance / settings.member_wander_wavelength_m;
        const double secondary = phase * 1.31 + kTwoPi * distance /
            (settings.member_wander_wavelength_m * 1.83);
        lateral_offset += relative_amplitude *
            (0.72 * std::sin(primary) + 0.28 * std::sin(secondary));
        vertical_offset = relative_amplitude * 0.45 *
            (0.70 * std::cos(primary * 0.89) + 0.30 * std::sin(secondary));
      }
      member.samples[index] = center.samples[index] +
          ScaleVec(lateral, lateral_offset * envelope) +
          ScaleVec(up, vertical_offset * envelope);
    }
    member.samples.front() = center.samples.front();
    member.samples.back() = center.samples.back();
    member.boundary_a = center.boundary_a;
    member.boundary_b = center.boundary_b;
    member.bounds = bounds_for(member.samples);
  }
  return members;
}

void contain_members(const VisualCurvePart& support, const SpanVisualAssemblyTemplate& settings,
                     const std::vector<VisualCurvePart*>& members, double radius) {
  const double inset = top_inset(support, settings.helix_clearance_m);
  for (VisualCurvePart* member : members) {
    const std::vector<Vec3d> original = member->samples;
    const double member_length = path_length(original);
    for (std::size_t index = 0; index < original.size(); ++index) {
      const double t = original.size() < 2 ? 0.0 : static_cast<double>(index) / static_cast<double>(original.size() - 1);
      const double member_distance = member_length * t;
      const path_sample anchor = sample_normalized(support.samples, t);
      Vec3d lateral{};
      Vec3d up{};
      frame_for(anchor.tangent, &lateral, &up);
      const Vec3d axis = anchor.point - ScaleVec(up, radius - inset);
      Vec3d baseline = projected_offset(original[index], anchor);
      baseline = place_below_support(
          baseline, up, required_member_radius(support, *member, settings.helix_clearance_m));
      Vec3d radial = anchor.point + baseline - axis;
      const double limit = std::max(0.0, radius - required_member_radius(support, *member, settings.helix_clearance_m));
      const double radial_length = Length(radial);
      if (radial_length > limit && radial_length > kLengthToleranceM) radial = ScaleVec(radial, limit / radial_length);
      const double envelope = span_envelope(member_distance, member_length);
      const Vec3d contained = axis + radial;
      member->samples[index] = original[index] + ScaleVec(contained - original[index], envelope);
    }
    member->samples.front() = original.front();
    member->samples.back() = original.back();
    member->boundary_a = member->samples.front();
    member->boundary_b = member->samples.back();
    member->bounds = bounds_for(member->samples);
  }
}

VisualCurvePart make_helix_part(const VisualCurvePart& support, const SpanVisualAssemblyTemplate& settings,
                                double radius) {
  VisualCurvePart helix = support;
  helix.supplemental_kind = VisualSupplementalKind::kHelix;
  helix.samples.clear();
  const double support_length = path_length(support.samples);
  const double trim = std::min(settings.endpoint_trim_m, support_length * 0.5);
  const double visible = support_length - trim * 2.0;
  const int samples = std::max(4, static_cast<int>(std::ceil(visible * settings.helix_turns_per_meter * settings.helix_samples_per_turn)));
  if (visible <= kLengthToleranceM || samples < 2) return helix;
  const double inset = top_inset(support, settings.helix_clearance_m);
  for (int i = 0; i <= samples; ++i) {
    const double distance = trim + visible * static_cast<double>(i) / static_cast<double>(samples);
    const path_sample anchor = sample_at_distance(support.samples, distance);
    Vec3d lateral{};
    Vec3d up{};
    frame_for(anchor.tangent, &lateral, &up);
    const Vec3d axis = anchor.point - ScaleVec(up, radius - inset);
    const double phase = kTwoPi * settings.helix_turns_per_meter * (distance - trim);
    helix.samples.push_back(axis + ScaleVec(lateral, std::cos(phase) * radius) + ScaleVec(up, std::sin(phase) * radius));
  }
  helix.boundary_a = helix.samples.front();
  helix.boundary_b = helix.samples.back();
  helix.bounds = bounds_for(helix.samples);
  return helix;
}

void separate_support_from_member(VisualCurvePart* support,
                                  const SpanVisualAssemblyTemplate& settings) {
  if (support == nullptr || support->samples.size() < 2) return;
  const std::vector<Vec3d> source = support->samples;
  const double total = path_length(source);
  double distance = 0.0;
  const double separation = support->wire_radius_m * 2.0;
  for (std::size_t index = 0; index < source.size(); ++index) {
    if (index > 0) distance += Length(source[index] - source[index - 1]);
    const Vec3d tangent = index + 1 < source.size()
        ? unit_or(source[index + 1] - source[index], {1.0, 0.0, 0.0})
        : unit_or(source[index] - source[index - 1], {1.0, 0.0, 0.0});
    Vec3d lateral{};
    Vec3d up{};
    frame_for(tangent, &lateral, &up);
    support->samples[index] = source[index] + ScaleVec(
        up, separation * endpoint_envelope(distance, total, settings.endpoint_trim_m));
  }
}

std::optional<std::pair<Vec3d, Vec3d>> support_endpoints(
    const CoreState& state, const Span& span, int pole_band_id,
    const SpanVisualAssemblyEndpointMap& member_endpoints) {
  if (pole_band_id > 0) {
    return resolve_pole_band_chord_endpoints(state, span, pole_band_id);
  }
  const auto endpoints = member_endpoints.find(span.id);
  if (endpoints == member_endpoints.end()) return std::nullopt;
  return std::pair<Vec3d, Vec3d>{endpoints->second.start, endpoints->second.end};
}

std::optional<VisualCurvePart> make_support_path(
    const CoreState& state, const Span& span, const BundleTemplate& bundle_template,
    const SpanVisualAssemblyTemplate& settings, std::uint64_t seed,
    const VisualCurvePart& member,
    const SpanVisualAssemblyEndpointMap& member_endpoints, VisualCurvePartCache* cache) {
  const std::optional<std::pair<Vec3d, Vec3d>> endpoints =
      support_endpoints(state, span, bundle_template.support_wire_pole_band_id, member_endpoints);
  if (!endpoints.has_value()) return std::nullopt;
  const EditResult<DetailCurve> support_curve =
      make_primary_curve_between(state, span.id, endpoints->first, endpoints->second);
  if (cache != nullptr) ++cache->stats.curve_builds;
  if (!support_curve.ok || support_curve.value.sample_points.size() < 2) return std::nullopt;

  VisualCurvePart support{};
  support.kind = VisualCurvePartKind::kSupplemental;
  support.supplemental_kind = VisualSupplementalKind::kSupportPath;
  support.source_span_id = span.id;
  support.source_bundle_id = span.bundle_id;
  support.bundle_template_id = bundle_template.id;
  support.lane_index = member.lane_index;
  support.samples = support_curve.value.sample_points;
  const auto support_cable_it = state.view().cable_templates().find(kDefaultSupportWireCableTemplateId);
  if (support_cable_it != state.view().cable_templates().end()) {
    const CableTemplate& support_cable = support_cable_it->second;
    support.wire_radius_m = support_cable.outer_diameter_m * 0.5;
    support.color_rgba = support_cable.color_rgba;
    support.material_style = support_cable.material_style;
    support.source_version = std::max(member.source_version, support_cable.version);
  } else {
    support.wire_radius_m = member.wire_radius_m;
    support.color_rgba = member.color_rgba;
    support.material_style = member.material_style;
    support.source_version = member.source_version;
  }
  apply_center_wander(settings, seed, &support);
  if (bundle_template.support_wire_pole_band_id == 0) {
    separate_support_from_member(&support, settings);
  }
  support.boundary_a = support.samples.front();
  support.boundary_b = support.samples.back();
  support.tangent_a = unit_or(support.samples[1] - support.samples[0], member.tangent_a);
  support.tangent_b = unit_or(support.samples[support.samples.size() - 2] - support.samples.back(),
                              member.tangent_b);
  support.bounds = bounds_for(support.samples);
  return support;
}

} // namespace

void apply_span_visual_assemblies(const CoreState& state,
                                  const SpanVisualAssemblyEndpointMap& member_endpoints,
                                  VisualCurvePartCache* cache) {
  if (cache == nullptr) return;
  std::unordered_map<ObjectId, std::vector<VisualCurvePart*>> members_by_span{};
  for (VisualCurvePart& part : cache->parts) {
    if (part.kind == VisualCurvePartKind::kEdgeBody && part.has_section_key) {
      members_by_span[part.section_key.logical_span_id].push_back(&part);
    }
  }
  std::vector<VisualCurvePart> supplemental{};
  for (auto& [logical_span_id, members] : members_by_span) {
    const Span* span = state.view().spans().find(logical_span_id);
    const Bundle* bundle = span == nullptr ? nullptr : state.view().bundles().find(span->bundle_id);
    const auto template_it = bundle == nullptr ? state.view().bundle_templates().end() :
        state.view().bundle_templates().find(bundle->bundle_template_id);
    if (template_it == state.view().bundle_templates().end()) continue;
    const SpanVisualAssemblyTemplate& settings = template_it->second.span_visual_assembly;
    const auto base = std::find_if(members.begin(), members.end(), [](const VisualCurvePart* member) {
      return member->section_key.is_base();
    });
    if (base == members.end()) continue;

    const std::uint64_t seed = assembly_seed(*bundle, template_it->second, (*base)->lane_index);
    VisualCurvePart center = **base;
    if (template_it->second.category != ConnectionCategory::kHighVoltage) {
      apply_center_wander(settings, seed, &center);
    }
    std::vector<VisualCurvePart> visual_members = make_visual_members(settings, seed, center);
    std::vector<VisualCurvePart*> visual_member_ptrs{};
    visual_member_ptrs.reserve(visual_members.size());
    for (VisualCurvePart& member : visual_members) visual_member_ptrs.push_back(&member);
    apply_member_twist(settings, visual_member_ptrs);

    if (!settings.support_path_enabled) {
      **base = std::move(visual_members.front());
      for (std::size_t index = 1; index < visual_members.size(); ++index) {
        supplemental.push_back(std::move(visual_members[index]));
      }
      continue;
    }
    std::optional<VisualCurvePart> support_result = make_support_path(
        state, *span, template_it->second, settings, seed, center, member_endpoints, cache);
    if (!support_result.has_value()) continue;
    VisualCurvePart support = std::move(*support_result);
    std::optional<VisualCurvePart> helix_part{};
    if (settings.helix_enabled) {
      const double fitted_radius = auto_fit_radius(support, visual_member_ptrs,
                                                   settings.helix_clearance_m);
      const double radius = settings.helix_radius_m > 0.0 ? settings.helix_radius_m : fitted_radius;
      if (settings.helix_radius_m > 0.0 &&
          settings.helix_radius_m + kLengthToleranceM < fitted_radius) {
        cache->diagnostics.push_back({kInvalidObjectId, logical_span_id, template_it->second.id,
                                      visual_members.front().lane_index,
                                      "span visual assembly radius clamped members"});
      }
      contain_members(support, settings, visual_member_ptrs, radius);
      VisualCurvePart helix = make_helix_part(support, settings, radius);
      if (helix.samples.size() >= 2) helix_part = std::move(helix);
    }
    **base = std::move(visual_members.front());
    for (std::size_t index = 1; index < visual_members.size(); ++index) {
      supplemental.push_back(std::move(visual_members[index]));
    }
    supplemental.push_back(std::move(support));
    if (helix_part.has_value()) supplemental.push_back(std::move(*helix_part));
  }
  cache->parts.insert(cache->parts.end(), std::make_move_iterator(supplemental.begin()),
                      std::make_move_iterator(supplemental.end()));
}

} // namespace city::wire::generation::backbone
