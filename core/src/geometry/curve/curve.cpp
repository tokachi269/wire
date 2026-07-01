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

double smooth_sag_weight(double t) {
  return 16.0 * t * t * (1.0 - t) * (1.0 - t);
}

double smooth_sag_derivative(double t) {
  return 32.0 * t * (1.0 - t) * (1.0 - 2.0 * t);
}

Vec3d hermite_position(const CableCurveInput& input, double t) {
  const Vec3d chord = input.end - input.start;
  const double length = Length(chord);
  const Vec3d chord_dir = normalized_or(chord, input.canonical_dir);
  const Vec3d start_tangent = normalized_or(input.start_tangent_hint, chord_dir);
  const Vec3d end_tangent = normalized_or(input.end_tangent_hint, chord_dir);
  const Vec3d m0 = ScaleVec(start_tangent, length);
  const Vec3d m1 = ScaleVec(end_tangent, length);
  const double t2 = t * t;
  const double t3 = t2 * t;
  return ScaleVec(input.start, 2.0 * t3 - 3.0 * t2 + 1.0) +
         ScaleVec(m0, t3 - 2.0 * t2 + t) +
         ScaleVec(input.end, -2.0 * t3 + 3.0 * t2) +
         ScaleVec(m1, t3 - t2);
}

Vec3d hermite_derivative(const CableCurveInput& input, double t) {
  const Vec3d chord = input.end - input.start;
  const double length = Length(chord);
  const Vec3d chord_dir = normalized_or(chord, input.canonical_dir);
  const Vec3d m0 = ScaleVec(normalized_or(input.start_tangent_hint, chord_dir), length);
  const Vec3d m1 = ScaleVec(normalized_or(input.end_tangent_hint, chord_dir), length);
  const double t2 = t * t;
  return ScaleVec(input.start, 6.0 * t2 - 6.0 * t) +
         ScaleVec(m0, 3.0 * t2 - 4.0 * t + 1.0) +
         ScaleVec(input.end, -6.0 * t2 + 6.0 * t) +
         ScaleVec(m1, 3.0 * t2 - 2.0 * t);
}

Vec3d position_at(const CableCurveInput& input, const Vec3d& gravity, double t) {
  const double sag = std::max(0.0, input.sag_m);
  if (input.method == CurveMethod::kCubicHermiteSag) {
    return hermite_position(input, t) + ScaleVec(gravity, sag * smooth_sag_weight(t));
  }
  const Vec3d chord = input.end - input.start;
  return input.start + ScaleVec(chord, t) + ScaleVec(gravity, sag * 4.0 * t * (1.0 - t));
}

Vec3d derivative_at(const CableCurveInput& input, const Vec3d& gravity, double t) {
  const double sag = std::max(0.0, input.sag_m);
  if (input.method == CurveMethod::kCubicHermiteSag) {
    return hermite_derivative(input, t) + ScaleVec(gravity, sag * smooth_sag_derivative(t));
  }
  return (input.end - input.start) + ScaleVec(gravity, sag * 4.0 * (1.0 - 2.0 * t));
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
    const Vec3d derivative = derivative_at(input, gravity, t);
    sample.speed_m_per_u = Length(derivative);
    sample.tangent = normalized_or(derivative, input.end - input.start);
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
        !finite(sample.binormal) || !std::isfinite(sample.speed_m_per_u) ||
        !std::isfinite(sample.arc_length_m)) {
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
  detail.start_constraint.point = input.start;
  detail.start_constraint.tangent_dir = output.samples.front().tangent;
  detail.end_constraint.point = input.end;
  detail.end_constraint.tangent_dir = output.samples.back().tangent;
  const double u_step = 1.0 / static_cast<double>(output.samples.size() - 1);
  detail.segments.reserve(output.samples.size() - 1);
  for (std::size_t index = 0; index + 1 < output.samples.size(); ++index) {
    const CableCurveSample& a = output.samples[index];
    const CableCurveSample& b = output.samples[index + 1];
    DetailCurveSegment segment{};
    segment.control_points = {
        a.position,
        a.position + ScaleVec(a.tangent, a.speed_m_per_u * u_step / 3.0),
        b.position - ScaleVec(b.tangent, b.speed_m_per_u * u_step / 3.0),
        b.position,
    };
    segment.u_start = static_cast<double>(index) * u_step;
    segment.u_end = static_cast<double>(index + 1) * u_step;
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
