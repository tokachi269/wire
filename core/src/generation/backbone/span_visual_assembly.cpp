#include "span_visual_assembly.hpp"

#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"
#include "../../geometry/detail_curve_input_resolution.hpp"
#include "../../geometry/detail_curve_postprocess.hpp"
#include "out.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace wire::core::generation::backbone {
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
  const double length = Length(value);
  return length > 1e-9 ? ScaleVec(value, 1.0 / length) : fallback;
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
      const double fraction = length <= 1e-9 ? 0.0 : (target - accumulated) / length;
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

double smoothstep(double value) {
  value = std::clamp(value, 0.0, 1.0);
  return value * value * (3.0 - 2.0 * value);
}

double endpoint_envelope(double distance, double total, double trim) {
  if (trim <= 1e-9) return 1.0;
  return smoothstep(std::min(distance / trim, (total - distance) / trim));
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
  if (radius + 1e-9 < inset) return false;
  for (const VisualCurvePart* member : members) {
    if (member->samples.empty()) continue;
    for (const double t : {0.0, 1.0}) {
      const path_sample anchor = sample_normalized(support.samples, t);
      Vec3d lateral{};
      Vec3d up{};
      frame_for(anchor.tangent, &lateral, &up);
      Vec3d offset = projected_offset(member->samples[t == 0.0 ? 0 : member->samples.size() - 1], anchor);
      offset = place_below_support(offset, up, required_member_radius(support, *member, clearance));
      const Vec3d axis = anchor.point - ScaleVec(up, radius - inset);
      if (Length(anchor.point + offset - axis) + required_member_radius(support, *member, clearance) > radius + 1e-9) {
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
    const double envelope = endpoint_envelope(distance, total, settings.endpoint_trim_m);
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

void contain_members(const CoreState& state, const VisualCurvePart& support, const SpanVisualAssemblyTemplate& settings,
                     const std::vector<VisualCurvePart*>& members, double radius) {
  const double support_length = path_length(support.samples);
  const double inset = top_inset(support, settings.helix_clearance_m);
  const path_sample start_anchor = sample_normalized(support.samples, 0.0);
  const path_sample end_anchor = sample_normalized(support.samples, 1.0);
  std::unordered_map<const VisualCurvePart*, std::pair<Vec3d, Vec3d>> endpoint_offsets{};
  for (const VisualCurvePart* member : members) {
    endpoint_offsets.emplace(member, std::make_pair(projected_offset(member->samples.front(), start_anchor),
                                                     projected_offset(member->samples.back(), end_anchor)));
  }
  const auto baseline_offset = [&](const VisualCurvePart& member, double t, const path_sample& anchor) {
    const auto endpoint = endpoint_offsets.find(&member);
    const Vec3d start = endpoint->second.first;
    const Vec3d end = endpoint->second.second;
    Vec3d offset = start + ScaleVec(end - start, t);
    return offset - ScaleVec(anchor.tangent, Dot(offset, anchor.tangent));
  };
  for (VisualCurvePart* member : members) {
    const std::vector<Vec3d> original = member->samples;
    const double member_length = path_length(original);
    const Span* source_span = state.view().spans().find(member->section_key.logical_span_id);
    const std::uint64_t flow_key = source_span == nullptr ? 0 :
        variation_flow_key_for_span(state.view().find_span_runtime_state(source_span->id), *source_span);
    const std::uint64_t seed = mix_seed(flow_key ^ static_cast<std::uint64_t>(member->section_key.logical_span_id) ^
        member->section_key.rule_owner_id ^ static_cast<std::uint64_t>(member->section_key.rule_id) ^
        static_cast<std::uint64_t>(member->section_key.instance_index));
    for (std::size_t index = 0; index < original.size(); ++index) {
      const double t = original.size() < 2 ? 0.0 : static_cast<double>(index) / static_cast<double>(original.size() - 1);
      const double member_distance = member_length * t;
      const path_sample anchor = sample_normalized(support.samples, t);
      Vec3d lateral{};
      Vec3d up{};
      frame_for(anchor.tangent, &lateral, &up);
      const Vec3d axis = anchor.point - ScaleVec(up, radius - inset);
      Vec3d baseline = baseline_offset(*member, t, anchor);
      if (settings.member_twist_turns_per_meter != 0.0 && members.size() > 1) {
        Vec3d centroid{};
        for (const VisualCurvePart* candidate : members) centroid = centroid + baseline_offset(*candidate, t, anchor);
        centroid = ScaleVec(centroid, 1.0 / static_cast<double>(members.size()));
        const Vec3d relative = baseline - centroid;
        const double angle = settings.member_twist_phase +
            kTwoPi * settings.member_twist_turns_per_meter * support_length * t;
        const double lateral_value = Dot(relative, lateral);
        const double up_value = Dot(relative, up);
        baseline = centroid + ScaleVec(lateral, lateral_value * std::cos(angle) - up_value * std::sin(angle)) +
            ScaleVec(up, lateral_value * std::sin(angle) + up_value * std::cos(angle));
      }
      baseline = place_below_support(
          baseline, up, required_member_radius(support, *member, settings.helix_clearance_m));
      Vec3d radial = anchor.point + baseline - axis;
      const double limit = std::max(0.0, radius - required_member_radius(support, *member, settings.helix_clearance_m));
      const double radial_length = Length(radial);
      if (radial_length > limit && radial_length > 1e-9) radial = ScaleVec(radial, limit / radial_length);
      const double margin = std::max(0.0, limit - Length(radial));
      const double envelope = endpoint_envelope(member_distance, member_length, settings.endpoint_trim_m);
      if (settings.member_wander_ratio > 0.0 && margin > 0.0 && envelope > 0.0) {
        const double phase = settings.member_wander_phase_bias +
            kTwoPi * (static_cast<double>(seed >> 11) / static_cast<double>(1ull << 53)) +
            kTwoPi * member_distance / settings.member_wander_wavelength_m;
        radial = radial + ScaleVec(lateral, std::cos(phase) * margin * settings.member_wander_ratio * envelope) +
            ScaleVec(up, std::sin(phase * 1.37) * margin * settings.member_wander_ratio * envelope);
        const double wandered = Length(radial);
        if (wandered > limit && wandered > 1e-9) radial = ScaleVec(radial, limit / wandered);
      }
      const Vec3d contained = axis + radial;
      member->samples[index] = original[index] + ScaleVec(contained - original[index], envelope);
    }
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
  if (visible <= 1e-9 || samples < 2) return helix;
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

} // namespace

void apply_span_visual_assemblies(const CoreState& state, VisualCurvePartCache* cache) {
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
    if (!settings.support_path_enabled) {
      apply_member_twist(settings, members);
      continue;
    }
    const std::optional<std::pair<Vec3d, Vec3d>> endpoints =
        resolve_pole_band_chord_endpoints(state, *span, template_it->second.support_wire_pole_band_id);
    if (!endpoints.has_value()) continue;
    const EditResult<DetailCurve> support_curve =
        make_primary_curve_between(state, logical_span_id, endpoints->first, endpoints->second);
    ++cache->stats.curve_builds;
    if (!support_curve.ok || support_curve.value.sample_points.size() < 2) continue;
    VisualCurvePart support{};
    support.kind = VisualCurvePartKind::kSupplemental;
    support.supplemental_kind = VisualSupplementalKind::kSupportPath;
    support.source_span_id = logical_span_id;
    support.source_bundle_id = span->bundle_id;
    support.bundle_template_id = template_it->second.id;
    support.lane_index = members.front()->lane_index;
    support.samples = support_curve.value.sample_points;
    support.boundary_a = support.samples.front();
    support.boundary_b = support.samples.back();
    support.tangent_a = support_curve.value.start_constraint.tangent_dir;
    support.tangent_b = ScaleVec(support_curve.value.end_constraint.tangent_dir, -1.0);
    support.wire_radius_m = members.front()->wire_radius_m;
    support.color_rgba = members.front()->color_rgba;
    support.material_style = members.front()->material_style;
    support.bounds = bounds_for(support.samples);
    support.source_version = members.front()->source_version;
    if (!settings.helix_enabled) {
      apply_member_twist(settings, members);
      supplemental.push_back(std::move(support));
      continue;
    }
    const double fitted_radius = auto_fit_radius(support, members, settings.helix_clearance_m);
    const double radius = settings.helix_radius_m > 0.0 ? settings.helix_radius_m : fitted_radius;
    if (settings.helix_radius_m > 0.0 && settings.helix_radius_m + 1e-9 < fitted_radius) {
      cache->diagnostics.push_back({kInvalidObjectId, logical_span_id, template_it->second.id,
                                    members.front()->lane_index, "span visual assembly radius clamped members"});
    }
    contain_members(state, support, settings, members, radius);
    VisualCurvePart helix = make_helix_part(support, settings, radius);
    supplemental.push_back(std::move(support));
    if (helix.samples.size() >= 2) supplemental.push_back(std::move(helix));
  }
  cache->parts.insert(cache->parts.end(), std::make_move_iterator(supplemental.begin()),
                      std::make_move_iterator(supplemental.end()));
}

} // namespace wire::core::generation::backbone
