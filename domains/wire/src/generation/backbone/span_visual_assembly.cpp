#include "span_visual_assembly.hpp"

#include "city/wire/core_view.hpp"
#include "city/wire/support/numeric_tolerances.hpp"
#include "city/wire/coord_utils.hpp"
#include "../../geometry/detail_curve_postprocess.hpp"
#include "out.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace city::wire::generation::backbone {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

struct path_sample {
  Vec3d point{};
  Vec3d tangent{1.0, 0.0, 0.0};
  double distance = 0.0;
};

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

std::vector<Vec3d> place_bundle_below_support(
    const VisualCurvePart& support, const std::vector<VisualCurvePart*>& members,
    std::size_t sample_index, const path_sample& anchor, double clearance) {
  std::vector<Vec3d> offsets{};
  offsets.reserve(members.size());
  Vec3d centroid{};
  for (const VisualCurvePart* member : members) {
    Vec3d offset = projected_offset(member->samples[sample_index], anchor);
    offsets.push_back(offset);
    centroid = centroid + offset;
  }
  centroid = ScaleVec(centroid, 1.0 / static_cast<double>(members.size()));
  double separation_scale = 1.0;
  for (std::size_t a = 0; a < offsets.size(); ++a) {
    for (std::size_t b = a + 1; b < offsets.size(); ++b) {
      const double separation = Length(offsets[a] - offsets[b]);
      const double required = members[a]->wire_radius_m + members[b]->wire_radius_m;
      if (separation > kLengthToleranceM) {
        separation_scale = std::max(separation_scale, required / separation);
      }
    }
  }
  if (separation_scale > 1.0) {
    for (Vec3d& offset : offsets) {
      offset = centroid + ScaleVec(offset - centroid, separation_scale);
    }
  }
  Vec3d lateral{};
  Vec3d up{};
  frame_for(anchor.tangent, &lateral, &up);
  double required_drop = 0.0;
  for (std::size_t index = 0; index < members.size(); ++index) {
    required_drop = std::max(required_drop,
        Dot(offsets[index] - centroid, up) +
        required_member_radius(support, *members[index], clearance));
  }
  const double shift = std::max(0.0, Dot(centroid, up) + required_drop);
  for (Vec3d& offset : offsets) offset = offset - ScaleVec(up, shift);
  return offsets;
}

bool members_fit_radius(const VisualCurvePart& support, const std::vector<VisualCurvePart*>& members,
                        double clearance, double radius) {
  const double inset = top_inset(support, clearance);
  if (radius + kLengthToleranceM < inset || members.empty()) return false;
  std::size_t sample_count = members.front()->samples.size();
  for (const VisualCurvePart* member : members) {
    sample_count = std::min(sample_count, member->samples.size());
  }
  for (std::size_t index = 0; index < sample_count; ++index) {
    const double t = sample_count < 2 ? 0.0 :
        static_cast<double>(index) / static_cast<double>(sample_count - 1);
    const path_sample anchor = sample_normalized(support.samples, t);
    Vec3d lateral{};
    Vec3d up{};
    frame_for(anchor.tangent, &lateral, &up);
    const std::vector<Vec3d> offsets = place_bundle_below_support(
        support, members, index, anchor, clearance);
    const Vec3d axis = anchor.point - ScaleVec(up, radius - inset);
    for (std::size_t member_index = 0; member_index < members.size(); ++member_index) {
      if (Length(anchor.point + offsets[member_index] - axis) +
              required_member_radius(support, *members[member_index], clearance) >
          radius + kLengthToleranceM) {
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
  return high + kLengthToleranceM;
}

double smoothstep(double value) {
  value = std::clamp(value, 0.0, 1.0);
  return value * value * (3.0 - 2.0 * value);
}

double endpoint_envelope(double distance, double total, double trim) {
  if (trim <= kLengthToleranceM) return 1.0;
  return smoothstep(std::min(distance / trim, (total - distance) / trim));
}

struct cross_section_offset {
  double lateral = 0.0;
  double up = 0.0;
};

std::vector<cross_section_offset> compact_cross_section(int count, double spacing) {
  std::vector<cross_section_offset> offsets{};
  offsets.reserve(static_cast<std::size_t>(count));
  if (count <= 1) {
    offsets.push_back({});
    return offsets;
  }
  if (count == 2) {
    offsets.push_back({-spacing * 0.5, 0.0});
    offsets.push_back({spacing * 0.5, 0.0});
    return offsets;
  }
  if (count == 3) {
    const double radius = spacing / std::sqrt(3.0);
    for (int index = 0; index < count; ++index) {
      const double angle = -kTwoPi * 0.25 + kTwoPi * static_cast<double>(index) / 3.0;
      offsets.push_back({std::cos(angle) * radius, std::sin(angle) * radius});
    }
    return offsets;
  }
  if (count == 4) {
    for (const double up : {-spacing * 0.5, spacing * 0.5}) {
      for (const double lateral : {-spacing * 0.5, spacing * 0.5}) {
        offsets.push_back({lateral, up});
      }
    }
    return offsets;
  }

  // Five or more members use one compact deterministic ring. This is intentionally not a packing solver.
  const double radius = spacing / (2.0 * std::sin(3.14159265358979323846 /
                                                  static_cast<double>(count)));
  for (int index = 0; index < count; ++index) {
    const double angle = -kTwoPi * 0.25 + kTwoPi * static_cast<double>(index) /
        static_cast<double>(count);
    offsets.push_back({std::cos(angle) * radius, std::sin(angle) * radius});
  }
  return offsets;
}

std::vector<VisualCurvePart> make_visual_members(
    const SpanVisualAssemblyTemplate& settings,
    const VisualCurvePart& center) {
  const int count = settings.visual_member_count;
  const double cable_diameter = center.wire_radius_m * 2.0;
  const double spacing = count < 2 ? 0.0 :
      std::max(cable_diameter + kLengthToleranceM,
               settings.visual_member_spacing_m);
  std::vector<VisualCurvePart> members(
      static_cast<std::size_t>(count), center);
  const std::vector<cross_section_offset> offsets =
      compact_cross_section(count, spacing);
  for (int member_index = 0; member_index < count; ++member_index) {
    VisualCurvePart& member = members[static_cast<std::size_t>(member_index)];
    member.section_count = static_cast<std::size_t>(count);
    member.section_key.instance_index = static_cast<std::size_t>(member_index);
  }

  for (std::size_t sample_index = 0; sample_index < center.samples.size();
       ++sample_index) {
    const Vec3d tangent = sample_index + 1 < center.samples.size()
        ? unit_or(center.samples[sample_index + 1] -
                      center.samples[sample_index], {1.0, 0.0, 0.0})
        : unit_or(center.samples[sample_index] -
                      center.samples[sample_index - 1], {1.0, 0.0, 0.0});
    Vec3d lateral{};
    Vec3d up{};
    frame_for(tangent, &lateral, &up);
    for (int member_index = 0; member_index < count; ++member_index) {
      const cross_section_offset& offset =
          offsets[static_cast<std::size_t>(member_index)];
      VisualCurvePart& member = members[static_cast<std::size_t>(member_index)];
      member.samples[sample_index] = center.samples[sample_index] +
          ScaleVec(lateral, offset.lateral) + ScaleVec(up, offset.up);
    }
  }
  for (VisualCurvePart& member : members) {
    member.boundary_a = member.samples.front();
    member.boundary_b = member.samples.back();
    member.bounds = bounds_for(member.samples);
  }
  return members;
}

void contain_members(const VisualCurvePart& support, const SpanVisualAssemblyTemplate& settings,
                     const std::vector<VisualCurvePart*>& members, double radius) {
  if (members.empty()) return;
  const double inset = top_inset(support, settings.helix_clearance_m);
  std::vector<std::vector<Vec3d>> original{};
  original.reserve(members.size());
  std::size_t sample_count = members.front()->samples.size();
  for (const VisualCurvePart* member : members) {
    original.push_back(member->samples);
    sample_count = std::min(sample_count, member->samples.size());
  }
  const double member_length = path_length(original.front());
  for (std::size_t index = 0; index < sample_count; ++index) {
    const double t = sample_count < 2 ? 0.0 :
        static_cast<double>(index) / static_cast<double>(sample_count - 1);
    const double member_distance = member_length * t;
    const path_sample anchor = sample_normalized(support.samples, t);
    Vec3d lateral{};
    Vec3d up{};
    frame_for(anchor.tangent, &lateral, &up);
    const Vec3d axis = anchor.point - ScaleVec(up, radius - inset);
    const std::vector<Vec3d> offsets = place_bundle_below_support(
        support, members, index, anchor, settings.helix_clearance_m);
    const double envelope = endpoint_envelope(
        member_distance, member_length, settings.endpoint_trim_m);
    for (std::size_t member_index = 0; member_index < members.size(); ++member_index) {
      VisualCurvePart* member = members[member_index];
      Vec3d radial = anchor.point + offsets[member_index] - axis;
      const double limit = std::max(0.0, radius - required_member_radius(
          support, *member, settings.helix_clearance_m));
      const double radial_length = Length(radial);
      if (radial_length > limit && radial_length > kLengthToleranceM) {
        radial = ScaleVec(radial, limit / radial_length);
      }
      const Vec3d contained = axis + radial;
      member->samples[index] = original[member_index][index] +
          ScaleVec(contained - original[member_index][index], envelope);
    }
  }
  for (VisualCurvePart* member : members) {
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
    const SpanVisualAssemblyTemplate& settings, const VisualCurvePart& member,
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

    VisualCurvePart center = **base;
    std::vector<VisualCurvePart> visual_members = make_visual_members(
        settings, center);
    std::vector<VisualCurvePart*> visual_member_ptrs{};
    visual_member_ptrs.reserve(visual_members.size());
    for (VisualCurvePart& member : visual_members) visual_member_ptrs.push_back(&member);
    if (!settings.support_path_enabled) {
      **base = std::move(visual_members.front());
      for (std::size_t index = 1; index < visual_members.size(); ++index) {
        supplemental.push_back(std::move(visual_members[index]));
      }
      continue;
    }
    std::optional<VisualCurvePart> support_result = make_support_path(
        state, *span, template_it->second, settings, center, member_endpoints, cache);
    if (!support_result.has_value()) continue;
    VisualCurvePart support = std::move(*support_result);
    std::optional<VisualCurvePart> helix_part{};
    if (settings.helix_enabled) {
      const double fitted_radius = auto_fit_radius(support, visual_member_ptrs,
                                                   settings.helix_clearance_m);
      const double radius = std::max(settings.helix_radius_m, fitted_radius);
      if (settings.helix_radius_m > 0.0 &&
          settings.helix_radius_m + kLengthToleranceM < fitted_radius) {
        cache->diagnostics.push_back({kInvalidObjectId, logical_span_id, template_it->second.id,
                                      visual_members.front().lane_index,
                                      "span visual assembly radius expanded to contain members"});
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

  std::vector<VisualCurvePart> connection_members{};
  const std::size_t part_count = cache->parts.size();
  for (std::size_t part_index = 0; part_index < part_count; ++part_index) {
    VisualCurvePart& part = cache->parts[part_index];
    if (part.kind != VisualCurvePartKind::kNodePatch &&
        part.kind != VisualCurvePartKind::kLead &&
        part.kind != VisualCurvePartKind::kJumper) {
      continue;
    }
    const Bundle* bundle = state.view().bundles().find(part.source_bundle_id);
    const auto template_it = bundle == nullptr ? state.view().bundle_templates().end() :
        state.view().bundle_templates().find(bundle->bundle_template_id);
    if (template_it == state.view().bundle_templates().end() ||
        template_it->second.category == ConnectionCategory::kHighVoltage) {
      continue;
    }
    const SpanVisualAssemblyTemplate& settings = template_it->second.span_visual_assembly;
    std::vector<VisualCurvePart> visual_members = make_visual_members(
        settings, part);
    part = std::move(visual_members.front());
    for (std::size_t member_index = 1; member_index < visual_members.size(); ++member_index) {
      connection_members.push_back(std::move(visual_members[member_index]));
    }
  }
  cache->parts.insert(cache->parts.end(), std::make_move_iterator(connection_members.begin()),
                      std::make_move_iterator(connection_members.end()));
}

} // namespace city::wire::generation::backbone
