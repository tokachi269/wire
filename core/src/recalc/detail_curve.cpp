#include "wire/core/detail_curve.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core {

namespace {

constexpr double kZeroLengthEps = 1e-9;

double Clamp01(double v) {
  return std::max(0.0, std::min(1.0, v));
}

Vec3d ClampToChordDirection(const Vec3d& chord_dir, const Vec3d& hint_dir) {
  Vec3d aligned = hint_dir;
  if (!Normalize(&aligned)) {
    return chord_dir;
  }
  if (Dot(aligned, chord_dir) < 0.0) {
    aligned = ScaleVec(aligned, -1.0);
  }
  Vec3d blended = chord_dir + aligned;
  if (!Normalize(&blended)) {
    return chord_dir;
  }
  return blended;
}

Vec3d ResolveEndpointPoint(const CurveConstraint& constraint) {
  if (constraint.endpoint_mode == CurveEndpointMode::kViaAttachment) {
    return constraint.point + constraint.attach_offset;
  }
  return constraint.point;
}

double SagShape(double u) {
  const double x = Clamp01(u);
  const double one_minus = 1.0 - x;
  return 16.0 * x * x * one_minus * one_minus;
}

double SagShapeDerivative(double u) {
  const double x = Clamp01(u);
  return 32.0 * x * (1.0 - x) * (1.0 - 2.0 * x);
}

Vec3d EvaluateBezier(const std::array<Vec3d, 4>& cp, double u) {
  const double t = Clamp01(u);
  const double omt = 1.0 - t;
  const double b0 = omt * omt * omt;
  const double b1 = 3.0 * omt * omt * t;
  const double b2 = 3.0 * omt * t * t;
  const double b3 = t * t * t;
  return {
      cp[0].x * b0 + cp[1].x * b1 + cp[2].x * b2 + cp[3].x * b3,
      cp[0].y * b0 + cp[1].y * b1 + cp[2].y * b2 + cp[3].y * b3,
      cp[0].z * b0 + cp[1].z * b1 + cp[2].z * b2 + cp[3].z * b3,
  };
}

Vec3d EvaluateBezierDerivative(const std::array<Vec3d, 4>& cp, double u) {
  const double t = Clamp01(u);
  const double omt = 1.0 - t;
  const Vec3d a = cp[1] - cp[0];
  const Vec3d b = cp[2] - cp[1];
  const Vec3d c = cp[3] - cp[2];
  return ScaleVec(a, 3.0 * omt * omt) + ScaleVec(b, 6.0 * omt * t) + ScaleVec(c, 3.0 * t * t);
}

double ChordProjectionProgress(const Vec3d& start, const Vec3d& chord_dir, const Vec3d& point) {
  return Dot(point - start, chord_dir);
}

double MaxPerpendicularDeviation(const Vec3d& start, const Vec3d& chord_dir, const Vec3d& point) {
  const Vec3d delta = point - start;
  const Vec3d along = ScaleVec(chord_dir, Dot(delta, chord_dir));
  return std::sqrt(std::max(0.0, LengthSquared(delta - along)));
}

double ContinuityScale(CableContinuityPolicyHint hint) {
  switch (hint) {
  case CableContinuityPolicyHint::kPreferG2:
    return 1.0;
  case CableContinuityPolicyHint::kPreferG1:
    return 0.75;
  default:
    return 0.9;
  }
}

double PassModeScale(CurvePassMode mode) {
  switch (mode) {
  case CurvePassMode::kPassThrough:
    return 1.0;
  case CurvePassMode::kBranch:
    return 0.8;
  case CurvePassMode::kTerminate:
    return 0.65;
  default:
    return 1.0;
  }
}

double ComputeTangentLength(double chord_length, const Vec3d& chord_dir, const CurveConstraint& constraint) {
  if (chord_length <= kZeroLengthEps) {
    return 0.0;
  }
  const double hinted = (constraint.tangent_length_hint_m > 0.0) ? constraint.tangent_length_hint_m : chord_length * 0.32;
  Vec3d tangent = constraint.tangent_dir;
  if (!Normalize(&tangent)) {
    tangent = chord_dir;
  }
  const double alignment = std::max(-1.0, std::min(1.0, Dot(tangent, chord_dir)));
  const double angle_scale = std::max(0.25, 1.0 - (std::acos(alignment) / 3.14159265358979323846) * 0.7);
  const double continuity_scale = ContinuityScale(constraint.continuity_preference);
  const double pass_scale = PassModeScale(constraint.pass_mode);
  const double capped = std::min(chord_length * 0.45, hinted * angle_scale * continuity_scale * pass_scale);
  return std::max(chord_length * 0.06, capped);
}

bool HasConflictingTangentHint(const Vec3d& chord_dir, const CurveConstraint& constraint) {
  Vec3d tangent = constraint.tangent_dir;
  if (!Normalize(&tangent)) {
    return false;
  }
  return Dot(tangent, chord_dir) < 0.25;
}

bool CurveHasPoorQuality(const std::array<Vec3d, 4>& cp, double sag_amplitude_m) {
  const Vec3d start = cp[0];
  Vec3d chord_dir = cp[3] - cp[0];
  const double chord_length = std::sqrt(LengthSquared(chord_dir));
  if (chord_length <= kZeroLengthEps) {
    return false;
  }
  chord_dir = ScaleVec(chord_dir, 1.0 / chord_length);

  double last_progress = 0.0;
  int regressions = 0;
  double max_deviation = 0.0;
  for (int i = 0; i <= 12; ++i) {
    const double u = static_cast<double>(i) / 12.0;
    Vec3d p = EvaluateBezier(cp, u);
    OffsetAlongWorldUp(&p, -sag_amplitude_m * SagShape(u));
    const double progress = ChordProjectionProgress(start, chord_dir, p);
    if (i > 0 && progress + 1e-6 < last_progress) {
      ++regressions;
    }
    last_progress = progress;
    max_deviation = std::max(max_deviation, MaxPerpendicularDeviation(start, chord_dir, p));
  }
  return regressions > 0 || max_deviation > chord_length * 0.60;
}

void PopulateLengthData(DetailCurve* curve) {
  curve->arc_length_table.clear();
  curve->distance_attributes = CurveDistanceAttributes{};
  curve->total_length_m = 0.0;
  if (curve->sample_points.empty()) {
    return;
  }
  curve->arc_length_table.reserve(curve->sample_points.size());
  curve->distance_attributes.arc_length_m.reserve(curve->sample_points.size());
  curve->distance_attributes.arc_length_normalized.reserve(curve->sample_points.size());
  if (curve->sample_points.size() >= 2) {
    curve->distance_attributes.segment_length_m.reserve(curve->sample_points.size() - 1);
  }
  curve->arc_length_table.push_back({0.0, 0.0});
  curve->distance_attributes.arc_length_m.push_back(0.0f);
  for (std::size_t i = 1; i < curve->sample_points.size(); ++i) {
    const double segment_length = std::sqrt(LengthSquared(curve->sample_points[i] - curve->sample_points[i - 1]));
    curve->total_length_m += segment_length;
    const double u = static_cast<double>(i) / static_cast<double>(curve->sample_points.size() - 1);
    curve->arc_length_table.push_back({u, curve->total_length_m});
    curve->distance_attributes.arc_length_m.push_back(static_cast<float>(curve->total_length_m));
    curve->distance_attributes.segment_length_m.push_back(static_cast<float>(segment_length));
  }
  const double total = std::max(curve->total_length_m, kZeroLengthEps);
  for (float arc_length : curve->distance_attributes.arc_length_m) {
    curve->distance_attributes.arc_length_normalized.push_back(static_cast<float>(arc_length / total));
  }
  curve->visible_intervals.clear();
  curve->hidden_intervals.clear();
  curve->replacement_intervals.clear();
  curve->visible_intervals.push_back({0.0, curve->total_length_m});
}

} // namespace

Vec3d DetailCurve::EvaluatePosition(double u) const {
  Vec3d point = EvaluateBezier(control_points, u);
  if (std::abs(sag_amplitude_m) > kZeroLengthEps) {
    OffsetAlongWorldUp(&point, -sag_amplitude_m * SagShape(u));
  }
  return point;
}

Vec3d DetailCurve::EvaluateTangent(double u) const {
  Vec3d tangent = EvaluateBezierDerivative(control_points, u);
  if (std::abs(sag_amplitude_m) > kZeroLengthEps) {
    tangent = tangent + ScaleVec(WorldUp(), -sag_amplitude_m * SagShapeDerivative(u));
  }
  Normalize(&tangent);
  return tangent;
}

double DetailCurve::LengthToU(double s) const {
  if (arc_length_table.empty()) {
    return 0.0;
  }
  const double clamped = std::max(0.0, std::min(total_length_m, s));
  const auto it = std::lower_bound(arc_length_table.begin(), arc_length_table.end(), clamped,
                                   [](const CurveLengthSample& sample, double target) {
                                     return sample.arc_length_m < target;
                                   });
  if (it == arc_length_table.begin()) {
    return it->u;
  }
  if (it == arc_length_table.end()) {
    return arc_length_table.back().u;
  }
  const CurveLengthSample& b = *it;
  const CurveLengthSample& a = *(it - 1);
  const double denom = std::max(kZeroLengthEps, b.arc_length_m - a.arc_length_m);
  const double t = (clamped - a.arc_length_m) / denom;
  return a.u + (b.u - a.u) * t;
}

double DetailCurve::NormalizedLengthToU(double x) const {
  return LengthToU(std::max(0.0, std::min(1.0, x)) * total_length_m);
}

Vec3d DetailCurve::PositionAtLength(double s) const {
  return EvaluatePosition(LengthToU(s));
}

DetailCurve BuildDetailCurve(const CurveConstraint& start_constraint, const CurveConstraint& end_constraint,
                             int sample_count) {
  DetailCurve curve{};
  curve.start_constraint = start_constraint;
  curve.end_constraint = end_constraint;

  const Vec3d start_point = ResolveEndpointPoint(start_constraint);
  const Vec3d end_point = ResolveEndpointPoint(end_constraint);
  Vec3d chord = end_point - start_point;
  const double chord_length = std::sqrt(LengthSquared(chord));
  Vec3d chord_dir = chord;
  if (!Normalize(&chord_dir)) {
    chord_dir = WorldForward();
  }

  Vec3d start_tangent = ClampToChordDirection(chord_dir, start_constraint.tangent_dir);
  Vec3d end_tangent = ClampToChordDirection(chord_dir, end_constraint.tangent_dir);

  const double base_sag = std::max(0.0, start_constraint.sag_hint + end_constraint.sag_hint +
                                            start_constraint.slack_hint + end_constraint.slack_hint);

  double tangent_scale = 1.0;
  std::array<Vec3d, 4> cp{};
  int fallback_iterations = 0;
  bool degraded = false;
  if (HasConflictingTangentHint(chord_dir, start_constraint) || HasConflictingTangentHint(chord_dir, end_constraint)) {
    tangent_scale *= 0.65;
    degraded = true;
  }
  for (; fallback_iterations < 3; ++fallback_iterations) {
    const double h0 = ComputeTangentLength(chord_length, chord_dir, start_constraint) * tangent_scale;
    const double h1 = ComputeTangentLength(chord_length, chord_dir, end_constraint) * tangent_scale;
    cp[0] = start_point;
    cp[1] = start_point + ScaleVec(start_tangent, h0);
    cp[2] = end_point - ScaleVec(end_tangent, h1);
    cp[3] = end_point;
    if (!CurveHasPoorQuality(cp, chord_length * base_sag)) {
      break;
    }
    tangent_scale *= 0.65;
    degraded = true;
  }

  curve.control_points = cp;
  curve.sag_amplitude_m = chord_length * base_sag;
  curve.quality.tangent_scale = tangent_scale;
  curve.quality.fallback_iterations = fallback_iterations;
  curve.quality.degraded_to_g1 = degraded;

  const int samples = std::max(2, sample_count);
  curve.sample_points.reserve(static_cast<std::size_t>(samples));
  for (int i = 0; i < samples; ++i) {
    const double u = (samples <= 1) ? 0.0 : (static_cast<double>(i) / static_cast<double>(samples - 1));
    curve.sample_points.push_back(curve.EvaluatePosition(u));
  }
  PopulateLengthData(&curve);
  return curve;
}

} // namespace wire::core
