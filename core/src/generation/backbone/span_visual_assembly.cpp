#include "span_visual_assembly.hpp"

#include "wire/core/core_view.hpp"
#include "wire/core/coord_utils.hpp"
#include "../../geometry/detail_curve_postprocess.hpp"
#include "out.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace wire::core::generation::backbone {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

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

double auto_fit_radius(const std::vector<VisualCurvePart*>& members, double clearance, double helix_wire_radius) {
  double radius = helix_wire_radius + clearance;
  for (const VisualCurvePart* member : members) {
    if (member->samples.empty()) continue;
    radius = std::max(radius, member->wire_radius_m + helix_wire_radius + clearance);
  }
  return std::max(0.01, radius);
}

Vec3d sample_normalized(const std::vector<Vec3d>& samples, double t) {
  if (samples.size() < 2) return samples.empty() ? Vec3d{} : samples.front();
  const double scaled = std::clamp(t, 0.0, 1.0) * static_cast<double>(samples.size() - 1);
  const std::size_t index = static_cast<std::size_t>(scaled);
  const std::size_t next = std::min(index + 1, samples.size() - 1);
  return samples[index] + ScaleVec(samples[next] - samples[index], scaled - static_cast<double>(index));
}

void contain_members(const VisualCurvePart& support, const SpanVisualAssemblyTemplate& settings,
                     const std::vector<VisualCurvePart*>& members, double radius) {
  const double allowed = std::max(0.0, radius - support.wire_radius_m - settings.helix_clearance_m);
  for (VisualCurvePart* member : members) {
    for (std::size_t index = 0; index < member->samples.size(); ++index) {
      const double t = member->samples.size() < 2 ? 0.0 : static_cast<double>(index) / static_cast<double>(member->samples.size() - 1);
      const Vec3d center = sample_normalized(support.samples, t);
      const Vec3d tangent = unit_or(sample_normalized(support.samples, std::min(1.0, t + 0.01)) -
                                    sample_normalized(support.samples, std::max(0.0, t - 0.01)), {1.0, 0.0, 0.0});
      Vec3d offset = member->samples[index] - center;
      offset = offset - ScaleVec(tangent, Dot(offset, tangent));
      const double limit = std::max(0.0, allowed - member->wire_radius_m);
      const double distance = Length(offset);
      if (distance > limit && distance > 1e-9) offset = ScaleVec(offset, limit / distance);
      const double margin = std::max(0.0, limit - Length(offset));
      if (settings.member_wander_ratio > 0.0 && margin > 0.0) {
        const std::uint64_t seed = mix_seed(static_cast<std::uint64_t>(member->section_key.logical_span_id) ^
            member->section_key.rule_owner_id ^ static_cast<std::uint64_t>(member->section_key.rule_id) ^
            static_cast<std::uint64_t>(member->section_key.instance_index));
        const double phase = settings.member_wander_phase_bias +
            kTwoPi * (static_cast<double>(seed >> 11) / static_cast<double>(1ull << 53)) +
            kTwoPi * t * std::max(0.0, settings.member_wander_wavelength_m);
        const Vec3d lateral = unit_or(Cross({0.0, 0.0, 1.0}, tangent), {0.0, 1.0, 0.0});
        const Vec3d up = unit_or(Cross(tangent, lateral), {0.0, 0.0, 1.0});
        offset = offset + ScaleVec(lateral, std::cos(phase) * margin * settings.member_wander_ratio) +
                 ScaleVec(up, std::sin(phase * 1.37) * margin * settings.member_wander_ratio);
        const double wandered = Length(offset);
        if (wandered > limit && wandered > 1e-9) offset = ScaleVec(offset, limit / wandered);
      }
      const double endpoint_blend = std::min(1.0, settings.endpoint_trim_m <= 1e-9 ? 1.0 :
          std::min(t, 1.0 - t) * static_cast<double>(member->samples.size() - 1) / std::max(1.0, settings.endpoint_trim_m));
      member->samples[index] = member->samples[index] + ScaleVec(center + offset - member->samples[index], endpoint_blend);
    }
    member->bounds = bounds_for(member->samples);
  }
}

VisualCurvePart make_helix_part(const VisualCurvePart& support, const SpanVisualAssemblyTemplate& settings,
                                const std::vector<VisualCurvePart*>& members) {
  VisualCurvePart helix = support;
  helix.supplemental_kind = VisualSupplementalKind::kHelix;
  helix.samples.clear();
  const double support_length = [&]() { double length = 0.0; for (std::size_t i = 1; i < support.samples.size(); ++i) length += Length(support.samples[i] - support.samples[i - 1]); return length; }();
  const double trim = std::min(settings.endpoint_trim_m, support_length * 0.5);
  const double visible = support_length - trim * 2.0;
  const double radius = settings.helix_radius_m > 0.0 ? settings.helix_radius_m :
      auto_fit_radius(members, settings.helix_clearance_m, support.wire_radius_m);
  const int samples = std::max(4, static_cast<int>(std::ceil(std::max(visible, 0.0) * settings.helix_turns_per_meter * settings.helix_samples_per_turn)));
  if (visible <= 1e-9 || samples < 2) return helix;
  for (int i = 0; i <= samples; ++i) {
    const double target = trim + visible * static_cast<double>(i) / static_cast<double>(samples);
    double accumulated = 0.0; std::size_t segment = 1;
    while (segment < support.samples.size() && accumulated + Length(support.samples[segment] - support.samples[segment - 1]) < target) { accumulated += Length(support.samples[segment] - support.samples[segment - 1]); ++segment; }
    if (segment >= support.samples.size()) segment = support.samples.size() - 1;
    const Vec3d a = support.samples[segment - 1], b = support.samples[segment];
    const double length = std::max(1e-9, Length(b - a)); const double t = std::clamp((target - accumulated) / length, 0.0, 1.0);
    const Vec3d center = a + ScaleVec(b - a, t); const Vec3d tangent = unit_or(b - a, {1.0, 0.0, 0.0});
    const Vec3d lateral = unit_or(Cross({0.0, 0.0, 1.0}, tangent), {0.0, 1.0, 0.0});
    const Vec3d up = unit_or(Cross(tangent, lateral), {0.0, 0.0, 1.0});
    const double phase = kTwoPi * settings.helix_turns_per_meter * (target - trim);
    helix.samples.push_back(center + ScaleVec(lateral, std::cos(phase) * radius) + ScaleVec(up, std::sin(phase) * radius));
  }
  helix.boundary_a = helix.samples.front(); helix.boundary_b = helix.samples.back(); helix.bounds = bounds_for(helix.samples);
  return helix;
}

void apply_member_twist(const SpanVisualAssemblyTemplate& settings, std::vector<VisualCurvePart*> members) {
  if (settings.member_twist_turns_per_meter == 0.0 || members.size() < 2) {
    return;
  }
  std::size_t sample_count = members.front()->samples.size();
  for (const VisualCurvePart* member : members) {
    sample_count = std::min(sample_count, member->samples.size());
  }
  if (sample_count < 2) {
    return;
  }
  double distance = 0.0;
  for (std::size_t index = 0; index < sample_count; ++index) {
    if (index > 0) {
      distance += Length(members.front()->samples[index] - members.front()->samples[index - 1]);
    }
    Vec3d centroid{};
    for (const VisualCurvePart* member : members) {
      centroid = centroid + member->samples[index];
    }
    centroid = ScaleVec(centroid, 1.0 / static_cast<double>(members.size()));
    const Vec3d tangent = index + 1 < sample_count
        ? unit_or(members.front()->samples[index + 1] - members.front()->samples[index], {1.0, 0.0, 0.0})
        : unit_or(members.front()->samples[index] - members.front()->samples[index - 1], {1.0, 0.0, 0.0});
    Vec3d lateral = Cross({0.0, 0.0, 1.0}, tangent);
    lateral = unit_or(lateral, {0.0, 1.0, 0.0});
    const Vec3d up = unit_or(Cross(tangent, lateral), {0.0, 0.0, 1.0});
    const double angle = settings.member_twist_phase + kTwoPi * settings.member_twist_turns_per_meter * distance;
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    for (VisualCurvePart* member : members) {
      const Vec3d offset = member->samples[index] - centroid;
      const double x = Dot(offset, lateral);
      const double y = Dot(offset, up);
      const Vec3d axial = ScaleVec(tangent, Dot(offset, tangent));
      member->samples[index] = centroid + axial + ScaleVec(lateral, x * cosine - y * sine) +
                               ScaleVec(up, x * sine + y * cosine);
    }
  }
  for (VisualCurvePart* member : members) {
    member->bounds = bounds_for(member->samples);
  }
}

} // namespace

void apply_span_visual_assemblies(const CoreState& state, VisualCurvePartCache* cache) {
  if (cache == nullptr) {
    return;
  }
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
    if (template_it == state.view().bundle_templates().end()) {
      continue;
    }
    const SpanVisualAssemblyTemplate& settings = template_it->second.span_visual_assembly;
    apply_member_twist(settings, members);
    if (!settings.helix_enabled) continue;
    const std::optional<std::pair<Vec3d, Vec3d>> endpoints =
        resolve_pole_band_chord_endpoints(state, *span, template_it->second.support_wire_pole_band_id);
    if (!endpoints.has_value()) {
      continue;
    }
    const EditResult<DetailCurve> support_curve =
        make_primary_curve_between(state, logical_span_id, endpoints->first, endpoints->second);
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
    const double radius = settings.helix_radius_m > 0.0 ? settings.helix_radius_m :
        auto_fit_radius(members, settings.helix_clearance_m, support.wire_radius_m);
    contain_members(support, settings, members, radius);
    VisualCurvePart helix = make_helix_part(support, settings, members);
    supplemental.push_back(std::move(support));
    if (helix.samples.size() >= 2) supplemental.push_back(std::move(helix));
  }
  cache->parts.insert(cache->parts.end(), std::make_move_iterator(supplemental.begin()), std::make_move_iterator(supplemental.end()));
}

} // namespace wire::core::generation::backbone
