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

Vec3d hermite(const Vec3d& p0, const Vec3d& m0, const Vec3d& p1, const Vec3d& m1, double t) {
  const double t2 = t * t;
  const double t3 = t2 * t;
  const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
  const double h10 = t3 - 2.0 * t2 + t;
  const double h01 = -2.0 * t3 + 3.0 * t2;
  const double h11 = t3 - t2;
  return {
      p0.x * h00 + m0.x * h10 + p1.x * h01 + m1.x * h11,
      p0.y * h00 + m0.y * h10 + p1.y * h01 + m1.y * h11,
      p0.z * h00 + m0.z * h10 + p1.z * h01 + m1.z * h11,
  };
}

Vec3d hermite_derivative(const Vec3d& p0, const Vec3d& m0, const Vec3d& p1, const Vec3d& m1, double t) {
  const double t2 = t * t;
  const double h00 = 6.0 * t2 - 6.0 * t;
  const double h10 = 3.0 * t2 - 4.0 * t + 1.0;
  const double h01 = -6.0 * t2 + 6.0 * t;
  const double h11 = 3.0 * t2 - 2.0 * t;
  return {
      p0.x * h00 + m0.x * h10 + p1.x * h01 + m1.x * h11,
      p0.y * h00 + m0.y * h10 + p1.y * h01 + m1.y * h11,
      p0.z * h00 + m0.z * h10 + p1.z * h01 + m1.z * h11,
  };
}

bool uses_endpoint_tangents(const CableCurveInput& input) {
  return input.has_start_tangent_hint || input.has_end_tangent_hint;
}

Vec3d tangent_or_chord(const Vec3d& hint, bool has_hint, const Vec3d& chord_dir) {
  return has_hint ? normalized_or(hint, chord_dir) : chord_dir;
}

Vec3d smooth_sag_offset(const Vec3d& gravity, double sag_m, double t) {
  const double one_minus_t = 1.0 - t;
  const double weight = 16.0 * t * t * one_minus_t * one_minus_t;
  return ScaleVec(gravity, std::max(0.0, sag_m) * weight);
}

Vec3d smooth_sag_derivative(const Vec3d& gravity, double sag_m, double t) {
  const double one_minus_t = 1.0 - t;
  const double weight_derivative = 32.0 * t * one_minus_t * (1.0 - 2.0 * t);
  return ScaleVec(gravity, std::max(0.0, sag_m) * weight_derivative);
}

Vec3d position_at(const CableCurveInput& input, const Vec3d& gravity, double t) {
  const Vec3d chord = input.end - input.start;
  if (uses_endpoint_tangents(input)) {
    const double chord_length = Length(chord);
    const Vec3d chord_dir = normalized_or(chord, input.canonical_dir);
    const Vec3d start_tangent =
        ScaleVec(tangent_or_chord(input.start_tangent_hint, input.has_start_tangent_hint, chord_dir), chord_length);
    const Vec3d end_tangent =
        ScaleVec(tangent_or_chord(input.end_tangent_hint, input.has_end_tangent_hint, chord_dir), chord_length);
    return hermite(input.start, start_tangent, input.end, end_tangent, t) +
           smooth_sag_offset(gravity, input.sag_m, t);
  }
  return input.start + ScaleVec(chord, t) +
         ScaleVec(gravity, std::max(0.0, input.sag_m) * 4.0 * t * (1.0 - t));
}

Vec3d tangent_at(const CableCurveInput& input, const Vec3d& gravity, double t) {
  const Vec3d chord = input.end - input.start;
  if (uses_endpoint_tangents(input)) {
    const double chord_length = Length(chord);
    const Vec3d chord_dir = normalized_or(chord, input.canonical_dir);
    const Vec3d start_tangent =
        ScaleVec(tangent_or_chord(input.start_tangent_hint, input.has_start_tangent_hint, chord_dir), chord_length);
    const Vec3d end_tangent =
        ScaleVec(tangent_or_chord(input.end_tangent_hint, input.has_end_tangent_hint, chord_dir), chord_length);
    const Vec3d derivative =
        hermite_derivative(input.start, start_tangent, input.end, end_tangent, t) +
        smooth_sag_derivative(gravity, input.sag_m, t);
    return normalized_or(derivative, chord);
  }
  const Vec3d derivative =
      chord + ScaleVec(gravity, std::max(0.0, input.sag_m) * 4.0 * (1.0 - 2.0 * t));
  return normalized_or(derivative, chord);
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
  if (input.method != CurveMethod::kParabolicSag) {
    result.error = "cable curve method is unsupported";
    return result;
  }
  if (input.family != CurveFamily::kMainSpan) {
    result.error = "cable curve family is unsupported";
    return result;
  }
  if (!finite(input.start) || !finite(input.end) || !finite(input.gravity_dir) ||
      !finite(input.canonical_dir) || !std::isfinite(input.sag_m) || input.sag_m < 0.0 ||
      !std::isfinite(input.radius_m) || input.radius_m < 0.0) {
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

  CableCurveOutput& output = result.value;
  output.samples.reserve(segments + 1);
  for (std::size_t index = 0; index <= segments; ++index) {
    const double t = static_cast<double>(index) / static_cast<double>(segments);
    CableCurveSample sample{};
    sample.position = position_at(input, gravity, t);
    sample.tangent = tangent_at(input, gravity, t);
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
  output.bounds.min = output.samples.front().position - Vec3d{input.radius_m, input.radius_m, input.radius_m};
  output.bounds.max = output.samples.front().position + Vec3d{input.radius_m, input.radius_m, input.radius_m};
  for (const CableCurveSample& sample : output.samples) {
    if (!finite(sample.position) || !finite(sample.tangent) || !finite(sample.normal) ||
        !finite(sample.binormal) || !std::isfinite(sample.arc_length_m)) {
      result.error = "cable curve output is invalid";
      result.value = {};
      return result;
    }
    expand_bounds(sample.position, input.radius_m, &output.bounds);
  }
  result.ok = true;
  return result;
}

DetailCurve ToDetailCurve(const CableCurveInput& input, const CableCurveOutput& output) {
  DetailCurve detail{};
  if (output.samples.size() < 2) {
    return detail;
  }
  const Vec3d gravity = normalized_or(input.gravity_dir, {0.0, 0.0, -1.0});
  const Vec3d chord = input.end - input.start;
  const bool tangent_aware = uses_endpoint_tangents(input);
  const Vec3d chord_dir = normalized_or(chord, input.canonical_dir);
  const double chord_length = Length(chord);
  const Vec3d start_tangent =
      tangent_or_chord(input.start_tangent_hint, input.has_start_tangent_hint, chord_dir);
  const Vec3d end_tangent = tangent_or_chord(input.end_tangent_hint, input.has_end_tangent_hint, chord_dir);
  const Vec3d sag_control_offset = ScaleVec(gravity, std::max(0.0, input.sag_m) * (4.0 / 3.0));
  detail.start_constraint.point = input.start;
  detail.start_constraint.tangent_dir = output.samples.front().tangent;
  detail.end_constraint.point = input.end;
  detail.end_constraint.tangent_dir = output.samples.back().tangent;
  if (tangent_aware) {
    detail.control_points = {input.start, input.start + ScaleVec(start_tangent, chord_length / 3.0),
                             input.end - ScaleVec(end_tangent, chord_length / 3.0), input.end};
  } else {
    detail.control_points = {input.start, input.start + ScaleVec(chord, 1.0 / 3.0) + sag_control_offset,
                             input.start + ScaleVec(chord, 2.0 / 3.0) + sag_control_offset, input.end};
  }
  detail.segments.push_back({detail.control_points, 0.0, 1.0});
  detail.sag_amplitude_m = std::max(0.0, input.sag_m);
  detail.sag_application = DetailCurveSagApplication::kBakedIntoControlCurve;
  detail.sample_points.reserve(output.samples.size());
  detail.arc_length_table.reserve(output.samples.size());
  detail.distance_attributes.arc_length_m.reserve(output.samples.size());
  detail.distance_attributes.arc_length_normalized.reserve(output.samples.size());
  detail.distance_attributes.segment_length_m.reserve(output.samples.size() - 1);
  for (std::size_t index = 0; index < output.samples.size(); ++index) {
    const CableCurveSample& sample = output.samples[index];
    const double u = static_cast<double>(index) / static_cast<double>(output.samples.size() - 1);
    detail.sample_points.push_back(sample.position);
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
