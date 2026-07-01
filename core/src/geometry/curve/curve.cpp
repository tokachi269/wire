#include "curve.hpp"

#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace wire::core::geometry::curve {
namespace {

constexpr double kEpsilon = 1e-9;

bool finite(const Vec3d& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

Vec3d normalized_or(const Vec3d& value, const Vec3d& fallback) {
  Vec3d result = value;
  if (!Normalize(&result)) {
    result = fallback;
    if (!Normalize(&result)) {
      return {1.0, 0.0, 0.0};
    }
  }
  return result;
}

Vec3d least_aligned_axis(const Vec3d& direction) {
  const std::array<Vec3d, 3> axes{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};
  return *std::min_element(axes.begin(), axes.end(), [&](const Vec3d& a, const Vec3d& b) {
    return std::abs(Dot(a, direction)) < std::abs(Dot(b, direction));
  });
}

Vec3d projected_normal(const Vec3d& seed, const Vec3d& tangent) {
  Vec3d result = seed - ScaleVec(tangent, Dot(seed, tangent));
  if (!Normalize(&result)) {
    const Vec3d fallback = least_aligned_axis(tangent);
    result = fallback - ScaleVec(tangent, Dot(fallback, tangent));
    Normalize(&result);
  }
  return result;
}

Vec3d canonical_lateral(const Vec3d& canonical, const Vec3d& up) {
  Vec3d lateral = Cross(canonical, up);
  if (!Normalize(&lateral)) {
    lateral = projected_normal(least_aligned_axis(canonical), canonical);
  }
  return lateral;
}

void align_frame(const Vec3d& preferred_lateral, CableCurveSample* sample) {
  if (sample == nullptr || Dot(sample->binormal, preferred_lateral) >= 0.0) {
    return;
  }
  sample->normal = ScaleVec(sample->normal, -1.0);
  sample->binormal = ScaleVec(sample->binormal, -1.0);
}

void rebuild_frames(std::vector<CableCurveSample>* samples, const Vec3d& canonical_dir,
                    const Vec3d& gravity_dir) {
  if (samples == nullptr || samples->empty()) {
    return;
  }
  const Vec3d gravity = normalized_or(gravity_dir, {0.0, 0.0, -1.0});
  const Vec3d up = ScaleVec(gravity, -1.0);
  const Vec3d canonical = normalized_or(canonical_dir, samples->front().tangent);
  const Vec3d preferred_lateral = canonical_lateral(canonical, up);
  for (std::size_t index = 0; index < samples->size(); ++index) {
    CableCurveSample& sample = (*samples)[index];
    sample.normal =
        index == 0 ? projected_normal(up, sample.tangent)
                   : projected_normal((*samples)[index - 1].normal, sample.tangent);
    sample.binormal = normalized_or(Cross(sample.tangent, sample.normal), preferred_lateral);
    sample.normal = normalized_or(Cross(sample.binormal, sample.tangent), sample.normal);
    align_frame(preferred_lateral, &sample);
  }
}

double smooth_sag_weight(double t) {
  return 16.0 * t * t * (1.0 - t) * (1.0 - t);
}

double smooth_sag_derivative(double t) {
  return 32.0 * t * (1.0 - t) * (1.0 - 2.0 * t);
}

Vec3d hermite_position(const Vec3d& p0, const Vec3d& p1, const Vec3d& m0, const Vec3d& m1, double t) {
  const double t2 = t * t;
  const double t3 = t2 * t;
  return ScaleVec(p0, 2.0 * t3 - 3.0 * t2 + 1.0) +
         ScaleVec(m0, t3 - 2.0 * t2 + t) +
         ScaleVec(p1, -2.0 * t3 + 3.0 * t2) +
         ScaleVec(m1, t3 - t2);
}

Vec3d hermite_derivative(const Vec3d& p0, const Vec3d& p1, const Vec3d& m0, const Vec3d& m1, double t) {
  const double t2 = t * t;
  return ScaleVec(p0, 6.0 * t2 - 6.0 * t) +
         ScaleVec(m0, 3.0 * t2 - 4.0 * t + 1.0) +
         ScaleVec(p1, -6.0 * t2 + 6.0 * t) +
         ScaleVec(m1, 3.0 * t2 - 2.0 * t);
}

double angle_between(const Vec3d& a, const Vec3d& b) {
  return std::acos(std::clamp(Dot(a, b), -1.0, 1.0));
}

Vec3d constrained_attachment_tangent(const Vec3d& hint, const Vec3d& chord_dir, double blend_length,
                                     const CableCurveProfile& profile) {
  Vec3d tangent = normalized_or(hint, chord_dir);
  if (Dot(tangent, chord_dir) <= 0.0) {
    tangent = chord_dir;
  }
  const double radius = std::max(0.0, profile.attachment_min_bend_radius_m);
  const double max_angle = radius > kEpsilon ? std::min(1.5707963267948966, blend_length / radius)
                                             : 1.5707963267948966;
  const double angle = angle_between(tangent, chord_dir);
  if (angle > max_angle && angle > kEpsilon) {
    tangent = normalized_or(ScaleVec(chord_dir, 1.0 - max_angle / angle) +
                                ScaleVec(tangent, max_angle / angle),
                            chord_dir);
  }
  const Vec3d horizontal_chord = normalized_or({chord_dir.x, chord_dir.y, 0.0}, chord_dir);
  Vec3d horizontal_tangent = normalized_or({tangent.x, tangent.y, 0.0}, horizontal_chord);
  const Vec3d lateral = horizontal_tangent - ScaleVec(horizontal_chord, Dot(horizontal_tangent, horizontal_chord));
  const double lateral_length = Length(lateral);
  const double max_deviation = std::max(0.0, profile.plan_view_max_deviation_m);
  if (lateral_length > kEpsilon && blend_length * lateral_length / 3.0 > max_deviation) {
    const double scale = max_deviation * 3.0 / (blend_length * lateral_length);
    horizontal_tangent =
        normalized_or(horizontal_chord + ScaleVec(lateral, std::clamp(scale, 0.0, 1.0)), horizontal_chord);
    tangent = normalized_or({horizontal_tangent.x, horizontal_tangent.y, tangent.z}, chord_dir);
  }
  return tangent;
}

struct PiecewisePolicy {
  double start_fraction = 0.0;
  double end_fraction = 1.0;
  double start_blend_m = 0.0;
  double end_blend_m = 0.0;
  Vec3d chord_dir{1.0, 0.0, 0.0};
  Vec3d start_tangent{1.0, 0.0, 0.0};
  Vec3d end_tangent{1.0, 0.0, 0.0};
  Vec3d main_start{};
  Vec3d main_end{};
};

double boundary_blend_length(CableEndpointBoundary boundary, const Vec3d& raw_tangent,
                             const Vec3d& chord_dir, double span_length,
                             const CableCurveProfile& profile) {
  if (boundary != CableEndpointBoundary::kContinuous || span_length <= kEpsilon) {
    return 0.0;
  }
  const double angle = angle_between(raw_tangent, chord_dir);
  if (angle <= std::max(0.0, profile.nearly_straight_angle_rad)) {
    return 0.0;
  }
  const double radius = std::max(0.0, profile.attachment_min_bend_radius_m);
  const double angle_length = radius > kEpsilon ? angle * radius : 0.0;
  const double profile_length = std::max(0.0, profile.plan_view_blend_length_m);
  const double side_limit = std::max(0.0, profile.max_blend_fraction_per_side) * span_length;
  return std::min({std::max(angle_length, profile_length), side_limit, span_length * 0.45});
}

PiecewisePolicy resolve_piecewise_policy(const CableCurveInput& input) {
  PiecewisePolicy policy{};
  const Vec3d chord = input.end - input.start;
  const double length = Length(chord);
  policy.chord_dir = normalized_or(chord, input.canonical_dir);
  if (length <= kEpsilon) {
    policy.main_start = input.start;
    policy.main_end = input.end;
    policy.start_tangent = policy.chord_dir;
    policy.end_tangent = policy.chord_dir;
    return policy;
  }
  const Vec3d raw_start = normalized_or(input.start_tangent_hint, policy.chord_dir);
  const Vec3d raw_end = normalized_or(input.end_tangent_hint, policy.chord_dir);
  policy.start_blend_m = boundary_blend_length(input.start_boundary, raw_start, policy.chord_dir, length, input.profile);
  policy.end_blend_m = boundary_blend_length(input.end_boundary, raw_end, policy.chord_dir, length, input.profile);
  const double max_blend_total = length * 0.60;
  const double blend_total = policy.start_blend_m + policy.end_blend_m;
  if (blend_total > max_blend_total && blend_total > kEpsilon) {
    const double scale = max_blend_total / blend_total;
    policy.start_blend_m *= scale;
    policy.end_blend_m *= scale;
  }
  policy.start_fraction = policy.start_blend_m / length;
  policy.end_fraction = 1.0 - policy.end_blend_m / length;
  policy.start_tangent = policy.start_blend_m > kEpsilon
                             ? constrained_attachment_tangent(raw_start, policy.chord_dir, policy.start_blend_m,
                                                             input.profile)
                             : policy.chord_dir;
  policy.end_tangent = policy.end_blend_m > kEpsilon
                           ? constrained_attachment_tangent(raw_end, policy.chord_dir, policy.end_blend_m,
                                                           input.profile)
                           : policy.chord_dir;
  policy.main_start = input.start + ScaleVec(policy.chord_dir, policy.start_blend_m);
  policy.main_end = input.end - ScaleVec(policy.chord_dir, policy.end_blend_m);
  return policy;
}

struct EvaluatedPoint {
  Vec3d position{};
  Vec3d derivative{};
  CurveRegionKind region = CurveRegionKind::kMainSpan;
};

EvaluatedPoint evaluate_piecewise(const CableCurveInput& input, const PiecewisePolicy& policy,
                                  const Vec3d& gravity, double t) {
  const double sag = std::max(0.0, input.sag_m);
  if (policy.start_fraction > kEpsilon && t < policy.start_fraction) {
    const double local = t / policy.start_fraction;
    const Vec3d m0 = ScaleVec(policy.start_tangent, policy.start_blend_m);
    const Vec3d m1 = ScaleVec(policy.chord_dir, policy.start_blend_m);
    return {hermite_position(input.start, policy.main_start, m0, m1, local),
            ScaleVec(hermite_derivative(input.start, policy.main_start, m0, m1, local),
                     1.0 / policy.start_fraction),
            CurveRegionKind::kStartAttachment};
  }
  if (policy.end_fraction < 1.0 - kEpsilon && t > policy.end_fraction) {
    const double fraction = 1.0 - policy.end_fraction;
    const double local = (t - policy.end_fraction) / fraction;
    const Vec3d m0 = ScaleVec(policy.chord_dir, policy.end_blend_m);
    const Vec3d m1 = ScaleVec(policy.end_tangent, policy.end_blend_m);
    return {hermite_position(policy.main_end, input.end, m0, m1, local),
            ScaleVec(hermite_derivative(policy.main_end, input.end, m0, m1, local), 1.0 / fraction),
            CurveRegionKind::kEndAttachment};
  }
  const double main_fraction = std::max(kEpsilon, policy.end_fraction - policy.start_fraction);
  const double local = std::clamp((t - policy.start_fraction) / main_fraction, 0.0, 1.0);
  const Vec3d main_chord = policy.main_end - policy.main_start;
  return {policy.main_start + ScaleVec(main_chord, local) +
              ScaleVec(gravity, sag * smooth_sag_weight(local)),
          ScaleVec(main_chord + ScaleVec(gravity, sag * smooth_sag_derivative(local)),
                   1.0 / main_fraction),
          CurveRegionKind::kMainSpan};
}

EvaluatedPoint evaluate_at(const CableCurveInput& input, const PiecewisePolicy& policy,
                           const Vec3d& gravity, double t) {
  const double sag = std::max(0.0, input.sag_m);
  if (input.method == CurveMethod::kCubicHermiteSag) {
    return evaluate_piecewise(input, policy, gravity, t);
  }
  const Vec3d chord = input.end - input.start;
  return {input.start + ScaleVec(chord, t) + ScaleVec(gravity, sag * 4.0 * t * (1.0 - t)),
          chord + ScaleVec(gravity, sag * 4.0 * (1.0 - 2.0 * t)),
          CurveRegionKind::kMainSpan};
}

void expand_bounds(const Vec3d& point, double radius, AABBd* bounds) {
  bounds->min.x = std::min(bounds->min.x, point.x - radius);
  bounds->min.y = std::min(bounds->min.y, point.y - radius);
  bounds->min.z = std::min(bounds->min.z, point.z - radius);
  bounds->max.x = std::max(bounds->max.x, point.x + radius);
  bounds->max.y = std::max(bounds->max.y, point.y + radius);
  bounds->max.z = std::max(bounds->max.z, point.z + radius);
}

} // namespace

std::size_t ResolveSegmentCount(const CableCurveInput& input) {
  const TessellationPolicy& policy = input.tessellation;
  const std::size_t minimum = std::max<std::size_t>(1, policy.min_segments);
  const std::size_t maximum = std::max(minimum, policy.max_segments);
  const double length = Length(input.end - input.start);
  const std::size_t length_segments =
      policy.meters_per_segment > kEpsilon
          ? static_cast<std::size_t>(std::ceil(length / policy.meters_per_segment))
          : minimum;
  const std::size_t sag_segments =
      policy.sag_m_per_segment > kEpsilon
          ? static_cast<std::size_t>(std::ceil(std::max(0.0, input.sag_m) / policy.sag_m_per_segment)) * 2
          : minimum;
  return std::clamp(std::max({minimum, length_segments, sag_segments}), minimum, maximum);
}

EditResult<CableCurveOutput> BuildCableCurve(const CableCurveInput& input) {
  EditResult<CableCurveOutput> result{};
  if (input.method != CurveMethod::kParabolicSag && input.method != CurveMethod::kCubicHermiteSag) {
    result.error = "cable curve method is unsupported";
    return result;
  }
  if (input.family != CurveFamily::kMainSpan) {
    result.error = "cable curve family is unsupported";
    return result;
  }
  if (input.profile.attachment_method != AttachmentCurveMethod::kCubicHermite) {
    result.error = "attachment curve method is unsupported";
    return result;
  }
  if (!finite(input.start) || !finite(input.end) || !finite(input.gravity_dir) ||
      !finite(input.canonical_dir) || !std::isfinite(input.sag_m) || input.sag_m < 0.0 ||
      !std::isfinite(input.radius_m) || input.radius_m < 0.0 ||
      !std::isfinite(input.profile.attachment_blend_length_m) ||
      input.profile.attachment_blend_length_m < 0.0 ||
      !std::isfinite(input.profile.attachment_min_bend_radius_m) ||
      input.profile.attachment_min_bend_radius_m < 0.0 ||
      !std::isfinite(input.profile.plan_view_blend_length_m) ||
      input.profile.plan_view_blend_length_m < 0.0 ||
      !std::isfinite(input.profile.plan_view_max_deviation_m) ||
      input.profile.plan_view_max_deviation_m < 0.0 ||
      !std::isfinite(input.profile.max_blend_fraction_per_side) ||
      input.profile.max_blend_fraction_per_side < 0.0 ||
      !std::isfinite(input.profile.nearly_straight_angle_rad) ||
      input.profile.nearly_straight_angle_rad < 0.0) {
    result.error = "cable curve input is invalid";
    return result;
  }

  const Vec3d chord = input.end - input.start;
  const Vec3d chord_dir = normalized_or(chord, input.canonical_dir);
  const Vec3d gravity = normalized_or(input.gravity_dir, {0.0, 0.0, -1.0});
  const Vec3d up = ScaleVec(gravity, -1.0);
  const Vec3d canonical = normalized_or(input.canonical_dir, chord_dir);
  const Vec3d preferred_lateral = canonical_lateral(canonical, up);
  const std::size_t segments = ResolveSegmentCount(input);
  const PiecewisePolicy piecewise = resolve_piecewise_policy(input);
  std::vector<double> parameters{};
  parameters.reserve(segments + 12);
  auto push_parameter = [&](double value) {
    if (value >= -kEpsilon && value <= 1.0 + kEpsilon) {
      parameters.push_back(std::clamp(value, 0.0, 1.0));
    }
  };
  for (std::size_t index = 0; index <= segments; ++index) {
    push_parameter(static_cast<double>(index) / static_cast<double>(segments));
  }
  if (input.method == CurveMethod::kCubicHermiteSag) {
    push_parameter(piecewise.start_fraction);
    push_parameter(piecewise.end_fraction);
    if (piecewise.start_fraction > kEpsilon) {
      push_parameter(piecewise.start_fraction * 0.25);
      push_parameter(piecewise.start_fraction * 0.50);
      push_parameter(piecewise.start_fraction * 0.75);
    }
    if (piecewise.end_fraction < 1.0 - kEpsilon) {
      const double end_len = 1.0 - piecewise.end_fraction;
      push_parameter(piecewise.end_fraction + end_len * 0.25);
      push_parameter(piecewise.end_fraction + end_len * 0.50);
      push_parameter(piecewise.end_fraction + end_len * 0.75);
    }
  }
  std::sort(parameters.begin(), parameters.end());
  parameters.erase(std::unique(parameters.begin(), parameters.end(), [](double a, double b) {
                     return std::abs(a - b) <= kEpsilon;
                   }),
                   parameters.end());

  CableCurveOutput& output = result.value;
  output.samples.reserve(parameters.size());
  for (double t : parameters) {
    CableCurveSample sample{};
    sample.parameter = t;
    const EvaluatedPoint evaluated = evaluate_at(input, piecewise, gravity, t);
    sample.position = evaluated.position;
    sample.speed_m_per_u = Length(evaluated.derivative);
    sample.tangent = normalized_or(evaluated.derivative, input.end - input.start);
    sample.region = evaluated.region;
    if (output.samples.empty()) {
      sample.normal = projected_normal(up, sample.tangent);
    } else {
      sample.normal = projected_normal(output.samples.back().normal, sample.tangent);
    }
    sample.binormal = normalized_or(Cross(sample.tangent, sample.normal), preferred_lateral);
    sample.normal = normalized_or(Cross(sample.binormal, sample.tangent), sample.normal);
    align_frame(preferred_lateral, &sample);
    if (!output.samples.empty()) {
      sample.arc_length_m =
          output.samples.back().arc_length_m + Length(sample.position - output.samples.back().position);
    }
    output.samples.push_back(sample);
  }

  output.length_m = output.samples.back().arc_length_m;
  output.start_blend_length_m = piecewise.start_blend_m;
  output.end_blend_length_m = piecewise.end_blend_m;
  output.bounds.min = output.samples.front().position - Vec3d{input.radius_m, input.radius_m, input.radius_m};
  output.bounds.max = output.samples.front().position + Vec3d{input.radius_m, input.radius_m, input.radius_m};
  for (const CableCurveSample& sample : output.samples) {
    if (!finite(sample.position) || !finite(sample.tangent) || !finite(sample.normal) ||
        !finite(sample.binormal) || !std::isfinite(sample.speed_m_per_u) ||
        !std::isfinite(sample.arc_length_m)) {
      result.error = "cable curve output is invalid";
      result.value = {};
      return result;
    }
    expand_bounds(sample.position, input.radius_m, &output.bounds);
  }
  auto append_region = [&](CurveRegionKind kind) {
    const auto first = std::find_if(output.samples.begin(), output.samples.end(),
                                    [&](const CableCurveSample& sample) { return sample.region == kind; });
    if (first == output.samples.end()) {
      return;
    }
    const auto last = std::find_if(output.samples.rbegin(), output.samples.rend(),
                                   [&](const CableCurveSample& sample) { return sample.region == kind; });
    const std::size_t first_index = static_cast<std::size_t>(std::distance(output.samples.begin(), first));
    const std::size_t last_index =
        output.samples.size() - 1 - static_cast<std::size_t>(std::distance(output.samples.rbegin(), last));
    output.attachment_regions.push_back(
        {kind, first_index, last_index, first->arc_length_m, output.samples[last_index].arc_length_m});
  };
  append_region(CurveRegionKind::kStartAttachment);
  append_region(CurveRegionKind::kEndAttachment);
  output.sections.push_back({0, output.samples.size() - 1, 0.0, output.length_m});
  result.ok = true;
  return result;
}

EditResult<CableRunShape> BuildCableRun(const std::vector<CableCurveInput>& sections) {
  EditResult<CableRunShape> result{};
  if (sections.empty()) {
    result.error = "cable run has no sections";
    return result;
  }
  for (const CableCurveInput& input : sections) {
    const EditResult<CableCurveOutput> built = BuildCableCurve(input);
    if (!built.ok) {
      result.error = built.error;
      return result;
    }
    if (result.value.sections.empty()) {
      result.value.bounds = built.value.bounds;
    } else {
      expand_bounds(built.value.bounds.min, 0.0, &result.value.bounds);
      expand_bounds(built.value.bounds.max, 0.0, &result.value.bounds);
    }
    std::size_t source_begin = 0;
    if (!result.value.samples.empty()) {
      if (Length(result.value.samples.back().position - built.value.samples.front().position) > kEpsilon) {
        result.error = "cable run sections are disconnected";
        return result;
      }
      if (Dot(result.value.samples.back().tangent, built.value.samples.front().tangent) < 1.0 - 1e-6) {
        result.error = "cable run sections are not G1 continuous";
        return result;
      }
      source_begin = 1;
    }
    const std::size_t first_sample = result.value.samples.empty() ? 0 : result.value.samples.size() - 1;
    const double arc_offset = result.value.samples.empty() ? 0.0 : result.value.samples.back().arc_length_m;
    for (std::size_t index = source_begin; index < built.value.samples.size(); ++index) {
      CableCurveSample sample = built.value.samples[index];
      sample.arc_length_m += arc_offset;
      result.value.samples.push_back(sample);
    }
    const std::size_t last_sample = result.value.samples.size() - 1;
    result.value.sections.push_back(
        {first_sample, last_sample, arc_offset, result.value.samples.back().arc_length_m});
    for (const AttachmentRegion& source : built.value.attachment_regions) {
      if (source.last_sample < source_begin) {
        continue;
      }
      const std::size_t index_offset = first_sample;
      const std::size_t adjusted_first =
          source.first_sample < source_begin ? first_sample : index_offset + source.first_sample - source_begin;
      const std::size_t adjusted_last = index_offset + source.last_sample - source_begin;
      result.value.attachment_regions.push_back(
          {source.kind, adjusted_first, adjusted_last, source.start_arc_length_m + arc_offset,
           source.end_arc_length_m + arc_offset});
    }
  }
  rebuild_frames(&result.value.samples, sections.front().canonical_dir, sections.front().gravity_dir);
  result.value.length_m = result.value.samples.back().arc_length_m;
  result.ok = true;
  return result;
}

EditResult<CableMemberShape> ExpandCableMember(const CableRunShape& run, const CableMemberProfile& profile) {
  EditResult<CableMemberShape> result{};
  if (!std::isfinite(profile.frame_offset_normal_m) ||
      !std::isfinite(profile.frame_offset_binormal_m) || run.samples.empty()) {
    result.error = "cable member profile or run is invalid";
    return result;
  }
  result.value.positions.reserve(run.samples.size());
  result.value.arc_length_m.reserve(run.samples.size());
  for (const CableCurveSample& sample : run.samples) {
    result.value.positions.push_back(
        sample.position + ScaleVec(sample.normal, profile.frame_offset_normal_m) +
        ScaleVec(sample.binormal, profile.frame_offset_binormal_m));
    result.value.arc_length_m.push_back(sample.arc_length_m);
  }
  result.ok = true;
  return result;
}

DetailCurve ToDetailCurve(const CableCurveInput& input, const CableCurveOutput& output) {
  DetailCurve detail{};
  if (output.samples.size() < 2) {
    return detail;
  }
  detail.start_constraint.point = input.start;
  detail.start_constraint.tangent_dir = output.samples.front().tangent;
  detail.end_constraint.point = input.end;
  detail.end_constraint.tangent_dir = output.samples.back().tangent;
  detail.method_debug = input.method == CurveMethod::kCubicHermiteSag ? CableCurveMethodDebug::kCubicHermiteSag
                                                                     : CableCurveMethodDebug::kParabolicSag;
  detail.start_boundary = input.start_boundary;
  detail.end_boundary = input.end_boundary;
  detail.start_blend_length_m = output.start_blend_length_m;
  detail.end_blend_length_m = output.end_blend_length_m;
  detail.segments.reserve(output.samples.size() - 1);
  for (std::size_t index = 0; index + 1 < output.samples.size(); ++index) {
    const CableCurveSample& a = output.samples[index];
    const CableCurveSample& b = output.samples[index + 1];
    const double u_step = std::max(kEpsilon, b.parameter - a.parameter);
    DetailCurveSegment segment{};
    segment.control_points = {
        a.position,
        a.position + ScaleVec(a.tangent, a.speed_m_per_u * u_step / 3.0),
        b.position - ScaleVec(b.tangent, b.speed_m_per_u * u_step / 3.0),
        b.position,
    };
    segment.u_start = a.parameter;
    segment.u_end = b.parameter;
    detail.segments.push_back(segment);
  }
  detail.control_points = detail.segments.front().control_points;
  detail.sag_amplitude_m = std::max(0.0, input.sag_m);
  detail.sag_application = DetailCurveSagApplication::kBakedIntoControlCurve;
  detail.sample_points.reserve(output.samples.size());
  detail.arc_length_table.reserve(output.samples.size());
  detail.distance_attributes.arc_length_m.reserve(output.samples.size());
  detail.distance_attributes.arc_length_normalized.reserve(output.samples.size());
  detail.distance_attributes.segment_length_m.reserve(output.samples.size() - 1);
  for (std::size_t index = 0; index < output.samples.size(); ++index) {
    const CableCurveSample& sample = output.samples[index];
    const double u = sample.parameter;
    detail.sample_points.push_back(sample.position);
    detail.sample_regions.push_back(static_cast<CableCurveSampleRegion>(sample.region));
    detail.arc_length_table.push_back({u, sample.arc_length_m});
    detail.distance_attributes.arc_length_m.push_back(static_cast<float>(sample.arc_length_m));
    detail.distance_attributes.arc_length_normalized.push_back(
        static_cast<float>(output.length_m > kEpsilon ? sample.arc_length_m / output.length_m : 0.0));
    if (index > 0) {
      detail.distance_attributes.segment_length_m.push_back(
          static_cast<float>(sample.arc_length_m - output.samples[index - 1].arc_length_m));
    }
  }
  detail.total_length_m = output.length_m;
  detail.visible_intervals.push_back({0.0, output.length_m});
  return detail;
}

} // namespace wire::core::geometry::curve
