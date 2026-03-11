#include "wire/core/detail_curve.hpp"
#include "wire/core/coord_utils.hpp"

#include <algorithm>
#include <cmath>

namespace wire::core {

namespace {

constexpr double kZeroLengthEps = 1e-9;
constexpr double kNormalizedCatenarySteepness = 2.6;
constexpr double kG2MinChordLengthM = 4.0;
constexpr double kG2EndpointOffsetRatioLimit = 0.08;
constexpr double kG2EndpointOffsetMetersLimit = 0.45;
constexpr double kFallbackTangentScale = 0.65;

struct ContinuityDecision {
  DetailCurveContinuityMode mode = DetailCurveContinuityMode::kG1;
  DetailCurveContinuityReason reason = DetailCurveContinuityReason::kAutoBalanced;
  bool attempted_g2 = false;
  double handle_scale = 0.78;
};

struct CurveShapePolicyDecision {
  CurveShapePolicyKind kind = CurveShapePolicyKind::kNearStraight;
  double lateral_suppression = 1.0;
  double handle_scale = 0.82;
  double quality_lateral_limit_ratio = 0.06;
};

struct TopViewMetrics {
  double max_lateral_overshoot_m = 0.0;
  int lateral_sign_changes = 0;
};

struct SideViewMetrics {
  double max_drop_m = 0.0;
  double endpoint_slope_delta = 0.0;
};

struct PerspectiveMetrics {
  int progress_regressions = 0;
  double max_spatial_deviation_m = 0.0;
};

struct CurveQualityMetrics {
  TopViewMetrics top{};
  SideViewMetrics side{};
  PerspectiveMetrics perspective{};
};

struct HandleLengthComponents {
  double base_scale = 1.0;
  double angle_scale = 1.0;
  double policy_scale = 1.0;
  double length_m = 0.0;
};

struct EndpointTangentDecision {
  Vec3d tangent{};
  DetailCurveEndpointTangentRule rule = DetailCurveEndpointTangentRule::kFallbackChord;
  double support_weight = 0.0;
  double chord_weight = 1.0;
  double departure_length_m = 0.0;
  double lateral_ratio_limit = 0.0;
};

double Clamp01(double v) {
  return std::max(0.0, std::min(1.0, v));
}

double SmoothStep(double edge0, double edge1, double x) {
  if (edge1 <= edge0) {
    return (x >= edge1) ? 1.0 : 0.0;
  }
  const double t = Clamp01((x - edge0) / (edge1 - edge0));
  return t * t * (3.0 - 2.0 * t);
}

double CornerStrengthForAngle(double angle_deg) {
  if (angle_deg <= 1e-6) {
    return 0.0;
  }
  return 1.0 - Clamp01((angle_deg - 35.0) / 120.0);
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

Vec3d BlendTowardChord(const Vec3d& chord_dir, const Vec3d& tangent, double weight) {
  Vec3d blended = ScaleVec(tangent, 1.0 - weight) + ScaleVec(chord_dir, weight);
  if (!Normalize(&blended)) {
    return chord_dir;
  }
  return blended;
}

Vec3d BlendDirections(const Vec3d& a, const Vec3d& b, double weight, const Vec3d& fallback) {
  Vec3d blended = ScaleVec(a, 1.0 - weight) + ScaleVec(b, weight);
  if (!Normalize(&blended)) {
    return fallback;
  }
  return blended;
}

Vec3d SuppressPlanarLateralComponent(const Vec3d& chord_dir, const Vec3d& tangent) {
  const Vec3d lateral_axis = ComputeLateralAxis(chord_dir);
  Vec3d adjusted = tangent - ScaleVec(lateral_axis, Dot(tangent, lateral_axis));
  if (!Normalize(&adjusted)) {
    return chord_dir;
  }
  if (Dot(adjusted, chord_dir) < 0.0) {
    adjusted = ScaleVec(adjusted, -1.0);
  }
  return adjusted;
}

Vec3d ApplyLateralSuppressionPolicy(const Vec3d& chord_dir, const Vec3d& tangent, double suppression) {
  if (suppression <= 1e-6) {
    return tangent;
  }
  const Vec3d suppressed = SuppressPlanarLateralComponent(chord_dir, tangent);
  return BlendDirections(tangent, suppressed, suppression, chord_dir);
}

Vec3d ClampPlanarLateralRatio(const Vec3d& chord_dir, const Vec3d& tangent, double lateral_ratio_limit) {
  if (lateral_ratio_limit <= 1e-6) {
    return tangent;
  }
  const Vec3d lateral_axis = ComputeLateralAxis(chord_dir);
  const Vec3d vertical_axis = WorldUp();
  const double along = Dot(tangent, chord_dir);
  const double lateral = Dot(tangent, lateral_axis);
  const double vertical = Dot(tangent, vertical_axis);
  const double max_lateral = std::max(0.0, lateral_ratio_limit) * std::max(0.22, std::abs(along));
  Vec3d clamped = ScaleVec(chord_dir, along) + ScaleVec(lateral_axis, std::clamp(lateral, -max_lateral, max_lateral)) +
                  ScaleVec(vertical_axis, vertical);
  if (!Normalize(&clamped)) {
    return chord_dir;
  }
  if (Dot(clamped, chord_dir) < 0.0) {
    clamped = ScaleVec(clamped, -1.0);
  }
  return clamped;
}

double SignedLateralComponent(const Vec3d& chord_dir, const Vec3d& tangent) {
  const Vec3d lateral_axis = ComputeLateralAxis(chord_dir);
  return Dot(tangent, lateral_axis);
}

void DampNearStraightTangents(const Vec3d& chord_dir, Vec3d* start_tangent, Vec3d* end_tangent) {
  if (start_tangent == nullptr || end_tangent == nullptr) {
    return;
  }
  const double start_alignment = std::clamp(Dot(*start_tangent, chord_dir), -1.0, 1.0);
  const double end_alignment = std::clamp(Dot(*end_tangent, chord_dir), -1.0, 1.0);
  double weight = SmoothStep(0.94, 0.995, std::min(start_alignment, end_alignment));
  const double start_lateral = SignedLateralComponent(chord_dir, *start_tangent);
  const double end_lateral = SignedLateralComponent(chord_dir, *end_tangent);
  if (start_lateral * end_lateral < -1e-6) {
    weight = std::max(weight, 0.82);
  }
  if (weight <= 1e-6) {
    return;
  }
  *start_tangent = BlendTowardChord(chord_dir, *start_tangent, weight);
  *end_tangent = BlendTowardChord(chord_dir, *end_tangent, weight);
}

void ApplyCornerPassTangentPolicy(const Vec3d& chord_dir, const CurveConstraint& constraint, Vec3d* tangent) {
  if (tangent == nullptr || !constraint.corner_pass) {
    return;
  }
  const double corner_strength = CornerStrengthForAngle(constraint.corner_angle_deg);
  if (corner_strength <= 1e-6) {
    return;
  }
  *tangent = chord_dir;
}

double CornerPassTangentLengthScale(const CurveConstraint& constraint) {
  if (!constraint.corner_pass) {
    return 1.0;
  }
  const double corner_strength = CornerStrengthForAngle(constraint.corner_angle_deg);
  return 1.0 - 0.65 * corner_strength;
}

Vec3d ResolveEndpointPoint(const CurveConstraint& constraint) {
  if (constraint.endpoint_mode == CurveEndpointMode::kOffsetEndpoint) {
    return constraint.point + constraint.endpoint_offset;
  }
  return constraint.point;
}

double SagShape(double u) {
  const double x = Clamp01(u) - 0.5;
  const double edge = std::cosh(kNormalizedCatenarySteepness * 0.5);
  const double denom = std::max(kZeroLengthEps, edge - 1.0);
  return std::max(0.0, (edge - std::cosh(kNormalizedCatenarySteepness * x)) / denom);
}

double SagShapeDerivative(double u) {
  const double x = Clamp01(u) - 0.5;
  const double edge = std::cosh(kNormalizedCatenarySteepness * 0.5);
  const double denom = std::max(kZeroLengthEps, edge - 1.0);
  return (-kNormalizedCatenarySteepness * std::sinh(kNormalizedCatenarySteepness * x)) / denom;
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

double AngleDegreesBetween(const Vec3d& a, const Vec3d& b) {
  Vec3d na = a;
  Vec3d nb = b;
  if (!Normalize(&na) || !Normalize(&nb)) {
    return 0.0;
  }
  const double dot = std::clamp(Dot(na, nb), -1.0, 1.0);
  return std::acos(dot) * (180.0 / 3.14159265358979323846);
}

bool HasValidTangentHint(const CurveConstraint& constraint) {
  Vec3d tangent = constraint.tangent_dir;
  return Normalize(&tangent);
}

double AlignmentToChord(const CurveConstraint& constraint, const Vec3d& chord_dir) {
  Vec3d tangent = constraint.tangent_dir;
  if (!Normalize(&tangent)) {
    return 1.0;
  }
  return std::clamp(Dot(tangent, chord_dir), -1.0, 1.0);
}

double EndpointOffsetMagnitude(const CurveConstraint& constraint) {
  if (constraint.endpoint_mode != CurveEndpointMode::kOffsetEndpoint) {
    return 0.0;
  }
  return std::sqrt(LengthSquared(constraint.endpoint_offset));
}

double BranchSupportWeightForChordLength(double chord_length) {
  return std::clamp(0.42 - 0.30 * SmoothStep(3.0, 22.0, chord_length), 0.10, 0.42);
}

double BranchLateralRatioLimitForChordLength(double chord_length) {
  return std::clamp(0.24 - 0.18 * SmoothStep(3.0, 24.0, chord_length), 0.04, 0.24);
}

double BranchDepartureLengthForConstraint(const CurveConstraint& constraint, double chord_length) {
  const double hinted = (constraint.support_departure_length_m > 0.0)
                            ? constraint.support_departure_length_m
                            : std::max(0.18, chord_length * 0.10);
  return std::clamp(hinted, 0.18, std::max(0.18, chord_length * 0.22));
}

double MainDepartureLengthForConstraint(const CurveConstraint& constraint, double chord_length) {
  const double hinted = (constraint.support_departure_length_m > 0.0)
                            ? constraint.support_departure_length_m
                            : std::max(0.22, chord_length * 0.16);
  return std::clamp(hinted, 0.22, std::max(0.22, chord_length * 0.30));
}

EndpointTangentDecision ResolveEndpointTangentDecision(const CurveConstraint& constraint,
                                                       const CurveShapePolicyDecision& shape_policy,
                                                       const Vec3d& chord_dir, double chord_length) {
  EndpointTangentDecision decision{};
  decision.tangent = chord_dir;
  decision.departure_length_m = MainDepartureLengthForConstraint(constraint, chord_length);

  Vec3d support_dir = constraint.tangent_dir;
  if (!Normalize(&support_dir)) {
    return decision;
  }
  if (Dot(support_dir, chord_dir) < 0.0) {
    support_dir = ScaleVec(support_dir, -1.0);
  }

  if (constraint.pass_mode == CurvePassMode::kBranch || shape_policy.kind == CurveShapePolicyKind::kBranchPass) {
    decision.rule = DetailCurveEndpointTangentRule::kBranchChordPriority;
    decision.support_weight = BranchSupportWeightForChordLength(chord_length) * std::clamp(constraint.tangent_strength, 0.3, 1.2);
    decision.chord_weight = std::max(0.0, 1.0 - decision.support_weight);
    decision.departure_length_m = BranchDepartureLengthForConstraint(constraint, chord_length);
    decision.lateral_ratio_limit = BranchLateralRatioLimitForChordLength(chord_length);
    decision.tangent = BlendDirections(chord_dir, support_dir, decision.support_weight, chord_dir);
    decision.tangent = ClampPlanarLateralRatio(chord_dir, decision.tangent, decision.lateral_ratio_limit);
    return decision;
  }

  if (constraint.pass_mode == CurvePassMode::kTerminate || shape_policy.kind == CurveShapePolicyKind::kTerminate) {
    decision.rule = DetailCurveEndpointTangentRule::kTerminateEndpointPriority;
    decision.support_weight = 0.28 * std::clamp(constraint.tangent_strength, 0.3, 1.2);
    decision.chord_weight = std::max(0.0, 1.0 - decision.support_weight);
    decision.departure_length_m = std::clamp(constraint.support_departure_length_m, 0.16, std::max(0.16, chord_length * 0.18));
    decision.lateral_ratio_limit = 0.10;
    decision.tangent = BlendDirections(chord_dir, support_dir, decision.support_weight, chord_dir);
    decision.tangent = ClampPlanarLateralRatio(chord_dir, decision.tangent, decision.lateral_ratio_limit);
    return decision;
  }

  if (constraint.endpoint_mode == CurveEndpointMode::kOffsetEndpoint || shape_policy.kind == CurveShapePolicyKind::kViaAttachment) {
    decision.rule = DetailCurveEndpointTangentRule::kAttachmentEndpointPriority;
    decision.support_weight = 0.36 * std::clamp(constraint.tangent_strength, 0.3, 1.2);
    decision.chord_weight = std::max(0.0, 1.0 - decision.support_weight);
    decision.departure_length_m = std::clamp(
        std::max(constraint.support_departure_length_m, std::sqrt(LengthSquared(constraint.endpoint_offset)) * 1.75), 0.18,
        std::max(0.18, chord_length * 0.24));
    decision.lateral_ratio_limit = 0.12;
    decision.tangent = BlendDirections(chord_dir, support_dir, decision.support_weight, chord_dir);
    decision.tangent = ClampPlanarLateralRatio(chord_dir, decision.tangent, decision.lateral_ratio_limit);
    return decision;
  }

  if (shape_policy.kind == CurveShapePolicyKind::kSmoothPass) {
    decision.rule = DetailCurveEndpointTangentRule::kMainFlowBlend;
    decision.support_weight = 0.58 * std::clamp(constraint.tangent_strength, 0.3, 1.2);
    decision.chord_weight = std::max(0.0, 1.0 - decision.support_weight);
    decision.departure_length_m = std::clamp(std::max(constraint.support_departure_length_m, chord_length * 0.20), 0.26,
                                             std::max(0.26, chord_length * 0.34));
    decision.lateral_ratio_limit = 0.68;
    decision.tangent = BlendDirections(chord_dir, support_dir, decision.support_weight, chord_dir);
    return decision;
  }

  decision.rule = DetailCurveEndpointTangentRule::kMainFlowBlend;
  decision.support_weight = 0.48 * std::clamp(constraint.tangent_strength, 0.3, 1.2);
  decision.chord_weight = std::max(0.0, 1.0 - decision.support_weight);
  decision.departure_length_m = MainDepartureLengthForConstraint(constraint, chord_length);
  decision.lateral_ratio_limit = (shape_policy.kind == CurveShapePolicyKind::kSmoothPass) ? 0.46 : 0.18;
  decision.tangent = BlendDirections(chord_dir, support_dir, decision.support_weight, chord_dir);
  decision.tangent = ClampPlanarLateralRatio(chord_dir, decision.tangent, decision.lateral_ratio_limit);
  return decision;
}

CurveShapePolicyDecision ChooseShapePolicy(const CurveConstraint& start_constraint, const CurveConstraint& end_constraint,
                                           double chord_length, const Vec3d& chord_dir) {
  CurveShapePolicyDecision decision{};
  const double max_endpoint_offset_m =
      std::max(EndpointOffsetMagnitude(start_constraint), EndpointOffsetMagnitude(end_constraint));
  const double min_alignment =
      std::min(AlignmentToChord(start_constraint, chord_dir), AlignmentToChord(end_constraint, chord_dir));
  const double tangent_turn_deg = AngleDegreesBetween(start_constraint.tangent_dir, end_constraint.tangent_dir);

  if (start_constraint.pass_mode == CurvePassMode::kTerminate || end_constraint.pass_mode == CurvePassMode::kTerminate) {
    decision.kind = CurveShapePolicyKind::kTerminate;
    decision.lateral_suppression = 0.95;
    decision.handle_scale = 0.58;
    decision.quality_lateral_limit_ratio = 0.04;
    return decision;
  }

  if (start_constraint.pass_mode == CurvePassMode::kBranch || end_constraint.pass_mode == CurvePassMode::kBranch) {
    decision.kind = CurveShapePolicyKind::kBranchPass;
    decision.lateral_suppression = 0.65;
    decision.handle_scale = 0.72;
    decision.quality_lateral_limit_ratio = 0.08;
    return decision;
  }

  if ((start_constraint.endpoint_mode == CurveEndpointMode::kOffsetEndpoint ||
       end_constraint.endpoint_mode == CurveEndpointMode::kOffsetEndpoint) &&
      max_endpoint_offset_m > 0.02) {
    decision.kind = CurveShapePolicyKind::kViaAttachment;
    decision.lateral_suppression = 0.68;
    decision.handle_scale = 0.76;
    decision.quality_lateral_limit_ratio = 0.08;
    return decision;
  }

  if (start_constraint.corner_pass || end_constraint.corner_pass || tangent_turn_deg >= 55.0 || min_alignment <= 0.65) {
    decision.kind = CurveShapePolicyKind::kSharpCorner;
    decision.lateral_suppression = 0.82;
    decision.handle_scale = 0.50;
    decision.quality_lateral_limit_ratio = 0.08;
    return decision;
  }

  const bool near_straight = chord_length <= 8.0 || (min_alignment >= 0.965 && tangent_turn_deg <= 24.0);
  if (near_straight) {
    decision.kind = CurveShapePolicyKind::kNearStraight;
    decision.lateral_suppression = 1.0;
    decision.handle_scale = 0.72;
    decision.quality_lateral_limit_ratio = 0.03;
    return decision;
  }

  decision.kind = CurveShapePolicyKind::kSmoothPass;
  decision.lateral_suppression = 0.20;
  decision.handle_scale = 1.12;
  decision.quality_lateral_limit_ratio = 0.22;
  return decision;
}

double RigidityScale(const CurveConstraint& constraint) {
  const double stiffness = std::clamp(constraint.bend_stiffness_hint, 0.25, 3.0);
  return std::clamp(0.85 + 0.15 * stiffness, 0.75, 1.30);
}

HandleLengthComponents ComputeHandleLength(double chord_length, const Vec3d& chord_dir, const CurveConstraint& constraint,
                                           const CurveShapePolicyDecision& policy, double continuity_scale) {
  HandleLengthComponents out{};
  if (chord_length <= kZeroLengthEps) {
    return out;
  }
  switch (policy.kind) {
  case CurveShapePolicyKind::kNearStraight:
    out.base_scale = 0.24;
    break;
  case CurveShapePolicyKind::kSmoothPass:
    out.base_scale = 0.40;
    break;
  case CurveShapePolicyKind::kSharpCorner:
    out.base_scale = 0.18;
    break;
  case CurveShapePolicyKind::kBranchPass:
    out.base_scale = 0.20;
    break;
  case CurveShapePolicyKind::kTerminate:
    out.base_scale = 0.16;
    break;
  case CurveShapePolicyKind::kViaAttachment:
    out.base_scale = 0.22;
    break;
  default:
    out.base_scale = 0.24;
    break;
  }
  const double hinted = (constraint.tangent_length_hint_m > 0.0) ? constraint.tangent_length_hint_m
                                                                 : chord_length * out.base_scale;
  Vec3d tangent = constraint.tangent_dir;
  if (!Normalize(&tangent)) {
    tangent = chord_dir;
  }
  const double alignment = std::max(-1.0, std::min(1.0, Dot(tangent, chord_dir)));
  out.angle_scale = std::max(0.22, 1.0 - (std::acos(alignment) / 3.14159265358979323846) * 0.7);
  const double min_from_radius = std::max(0.0, constraint.min_bend_radius_hint_m * 0.6);
  out.policy_scale = std::max(0.1, policy.handle_scale * constraint.tangent_strength);
  const double target =
      std::max(min_from_radius, hinted * out.angle_scale * out.policy_scale * RigidityScale(constraint) * continuity_scale);
  double capped = std::min(chord_length * 0.45, target);
  if (constraint.support_departure_length_m > 0.0) {
    if (policy.kind == CurveShapePolicyKind::kBranchPass) {
      capped = std::min(capped, constraint.support_departure_length_m);
    } else if (policy.kind == CurveShapePolicyKind::kTerminate) {
      capped = std::min(capped, constraint.support_departure_length_m * 1.05);
    } else if (policy.kind == CurveShapePolicyKind::kViaAttachment) {
      capped = std::min(capped, constraint.support_departure_length_m * 1.15);
    }
  }
  double min_length_scale = 0.06;
  if (policy.kind == CurveShapePolicyKind::kBranchPass) {
    min_length_scale = 0.03;
  } else if (policy.kind == CurveShapePolicyKind::kTerminate) {
    min_length_scale = 0.04;
  }
  out.length_m = std::max(chord_length * min_length_scale, capped);
  return out;
}

CableContinuityPolicyHint ResolveContinuityPreference(const CurveConstraint& start_constraint,
                                                      const CurveConstraint& end_constraint) {
  if (start_constraint.continuity_preference != CableContinuityPolicyHint::kAuto) {
    return start_constraint.continuity_preference;
  }
  return end_constraint.continuity_preference;
}

bool HasConflictingTangentHint(const Vec3d& chord_dir, const CurveConstraint& constraint) {
  Vec3d tangent = constraint.tangent_dir;
  if (!Normalize(&tangent)) {
    return false;
  }
  return Dot(tangent, chord_dir) < 0.25;
}

CurveQualityMetrics EvaluateCurveQuality(const std::array<Vec3d, 4>& cp, double sag_amplitude_m) {
  CurveQualityMetrics metrics{};
  const Vec3d start = cp[0];
  Vec3d chord_dir = cp[3] - cp[0];
  const double chord_length = std::sqrt(LengthSquared(chord_dir));
  if (chord_length <= kZeroLengthEps) {
    return metrics;
  }
  chord_dir = ScaleVec(chord_dir, 1.0 / chord_length);
  const Vec3d lateral_axis = ComputeLateralAxis(chord_dir);
  double last_lateral = 0.0;
  bool has_last_lateral = false;

  double last_progress = 0.0;
  for (int i = 0; i <= 12; ++i) {
    const double u = static_cast<double>(i) / 12.0;
    Vec3d p = EvaluateBezier(cp, u);
    OffsetAlongWorldUp(&p, -sag_amplitude_m * SagShape(u));
    const double progress = ChordProjectionProgress(start, chord_dir, p);
    if (i > 0 && progress + 1e-6 < last_progress) {
      ++metrics.perspective.progress_regressions;
    }
    last_progress = progress;
    metrics.perspective.max_spatial_deviation_m =
        std::max(metrics.perspective.max_spatial_deviation_m, MaxPerpendicularDeviation(start, chord_dir, p));
    const double lateral = Dot(p - start, lateral_axis);
    metrics.top.max_lateral_overshoot_m = std::max(metrics.top.max_lateral_overshoot_m, std::abs(lateral));
    if (has_last_lateral && std::abs(lateral) > 1e-6 && std::abs(last_lateral) > 1e-6 &&
        ((lateral > 0.0) != (last_lateral > 0.0))) {
      ++metrics.top.lateral_sign_changes;
    }
    has_last_lateral = true;
    last_lateral = lateral;
    metrics.side.max_drop_m = std::max(metrics.side.max_drop_m, HeightAlongWorldUp(start) - HeightAlongWorldUp(p));
  }
  const Vec3d tangent0 = EvaluateBezierDerivative(cp, 0.0) + ScaleVec(WorldUp(), -sag_amplitude_m * SagShapeDerivative(0.0));
  const Vec3d tangent1 = EvaluateBezierDerivative(cp, 1.0) + ScaleVec(WorldUp(), -sag_amplitude_m * SagShapeDerivative(1.0));
  metrics.side.endpoint_slope_delta = AngleDegreesBetween(tangent0, tangent1);
  return metrics;
}

bool CurveHasPoorQuality(const CurveQualityMetrics& metrics, const CurveShapePolicyDecision& policy, double chord_length) {
  if (metrics.perspective.progress_regressions > 0) {
    return true;
  }
  if (metrics.perspective.max_spatial_deviation_m > chord_length * 0.60) {
    return true;
  }
  if (metrics.top.max_lateral_overshoot_m > chord_length * policy.quality_lateral_limit_ratio) {
    return true;
  }
  if (policy.kind != CurveShapePolicyKind::kSmoothPass && metrics.top.lateral_sign_changes > 0) {
    return true;
  }
  return false;
}

ContinuityDecision DecideContinuity(const CurveConstraint& start_constraint, const CurveConstraint& end_constraint,
                                    const CurveShapePolicyDecision& shape_policy, double chord_length,
                                    const Vec3d& chord_dir) {
  ContinuityDecision decision{};
  decision.handle_scale = shape_policy.handle_scale;

  const CableContinuityPolicyHint policy = start_constraint.continuity_preference;
  if (policy == CableContinuityPolicyHint::kPreferG1) {
    decision.reason = DetailCurveContinuityReason::kPolicyPreferG1;
    return decision;
  }

  if (chord_length <= kZeroLengthEps) {
    decision.reason = DetailCurveContinuityReason::kContextInsufficient;
    return decision;
  }

  if (!HasValidTangentHint(start_constraint) || !HasValidTangentHint(end_constraint)) {
    decision.reason = DetailCurveContinuityReason::kContextInsufficient;
    return decision;
  }

  if (shape_policy.kind == CurveShapePolicyKind::kBranchPass) {
    decision.reason = DetailCurveContinuityReason::kBranchPass;
    return decision;
  }

  if (HasConflictingTangentHint(chord_dir, start_constraint) || HasConflictingTangentHint(chord_dir, end_constraint)) {
    decision.reason = DetailCurveContinuityReason::kConflictingTangents;
    return decision;
  }

  if (shape_policy.kind == CurveShapePolicyKind::kSharpCorner || shape_policy.kind == CurveShapePolicyKind::kTerminate) {
    decision.reason = DetailCurveContinuityReason::kCornerPass;
    return decision;
  }

  const double max_endpoint_offset_m =
      std::max(EndpointOffsetMagnitude(start_constraint), EndpointOffsetMagnitude(end_constraint));
  if (shape_policy.kind == CurveShapePolicyKind::kViaAttachment ||
      max_endpoint_offset_m > kG2EndpointOffsetMetersLimit ||
      max_endpoint_offset_m > chord_length * kG2EndpointOffsetRatioLimit) {
    decision.reason = DetailCurveContinuityReason::kEndpointConstraintPriority;
    return decision;
  }

  const double min_g2_chord_m =
      std::max(kG2MinChordLengthM,
               6.0 * std::max(start_constraint.min_bend_radius_hint_m, end_constraint.min_bend_radius_hint_m));
  if (chord_length < min_g2_chord_m) {
    decision.reason = DetailCurveContinuityReason::kShortSpan;
    return decision;
  }

  decision.mode = DetailCurveContinuityMode::kG2;
  decision.attempted_g2 = true;
  decision.handle_scale = (policy == CableContinuityPolicyHint::kPreferG2) ? 1.0 : 0.92;
  decision.reason = (policy == CableContinuityPolicyHint::kPreferG2) ? DetailCurveContinuityReason::kSmoothPassThrough
                                                                     : DetailCurveContinuityReason::kAutoBalanced;
  return decision;
}

double ComputeSagAmplitudeM(const CurveConstraint& start_constraint, const CurveConstraint& end_constraint, double chord_length,
                            const CurveShapePolicyDecision& shape_policy, double* out_base_ratio,
                            double* out_length_scale, double* out_pass_scale, double* out_rigidity_scale) {
  const double base_ratio = std::max(0.0, start_constraint.sag_hint + end_constraint.sag_hint + start_constraint.slack_hint +
                                              end_constraint.slack_hint);
  const double length_scale = 0.82 + 0.40 * SmoothStep(6.0, 32.0, chord_length);
  double pass_scale = 1.0;
  switch (shape_policy.kind) {
  case CurveShapePolicyKind::kBranchPass:
    pass_scale = 0.72;
    break;
  case CurveShapePolicyKind::kTerminate:
    pass_scale = 0.78;
    break;
  case CurveShapePolicyKind::kSmoothPass:
    pass_scale = 1.14;
    break;
  case CurveShapePolicyKind::kNearStraight:
    pass_scale = 1.05;
    break;
  case CurveShapePolicyKind::kViaAttachment:
    pass_scale = 0.92;
    break;
  case CurveShapePolicyKind::kSharpCorner:
    pass_scale = 0.86;
    break;
  default:
    pass_scale = 1.0;
    break;
  }
  const double avg_stiffness =
      0.5 * (std::clamp(start_constraint.bend_stiffness_hint, 0.25, 3.0) + std::clamp(end_constraint.bend_stiffness_hint, 0.25, 3.0));
  const double rigidity_scale = std::clamp(1.16 - 0.14 * avg_stiffness, 0.72, 1.12);

  if (out_base_ratio != nullptr) {
    *out_base_ratio = base_ratio;
  }
  if (out_length_scale != nullptr) {
    *out_length_scale = length_scale;
  }
  if (out_pass_scale != nullptr) {
    *out_pass_scale = pass_scale;
  }
  if (out_rigidity_scale != nullptr) {
    *out_rigidity_scale = rigidity_scale;
  }
  return chord_length * base_ratio * length_scale * pass_scale * rigidity_scale;
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
  const CurveShapePolicyDecision shape_policy = ChooseShapePolicy(start_constraint, end_constraint, chord_length, chord_dir);
  const CableContinuityPolicyHint requested_policy = ResolveContinuityPreference(start_constraint, end_constraint);

  const EndpointTangentDecision start_tangent_decision =
      ResolveEndpointTangentDecision(start_constraint, shape_policy, chord_dir, chord_length);
  const EndpointTangentDecision end_tangent_decision =
      ResolveEndpointTangentDecision(end_constraint, shape_policy, chord_dir, chord_length);
  Vec3d start_tangent = start_tangent_decision.tangent;
  Vec3d end_tangent = end_tangent_decision.tangent;
  ApplyCornerPassTangentPolicy(chord_dir, start_constraint, &start_tangent);
  ApplyCornerPassTangentPolicy(chord_dir, end_constraint, &end_tangent);
  start_tangent = ApplyLateralSuppressionPolicy(chord_dir, start_tangent, shape_policy.lateral_suppression);
  end_tangent = ApplyLateralSuppressionPolicy(chord_dir, end_tangent, shape_policy.lateral_suppression);
  if (shape_policy.kind == CurveShapePolicyKind::kNearStraight) {
    DampNearStraightTangents(chord_dir, &start_tangent, &end_tangent);
  }

  CurveConstraint normalized_start = start_constraint;
  CurveConstraint normalized_end = end_constraint;
  normalized_start.continuity_preference = requested_policy;
  normalized_end.continuity_preference = requested_policy;
  ContinuityDecision continuity =
      DecideContinuity(normalized_start, normalized_end, shape_policy, chord_length, chord_dir);
  const bool prefer_g1_policy = requested_policy == CableContinuityPolicyHint::kPreferG1;
  const bool started_as_g2 = continuity.mode == DetailCurveContinuityMode::kG2;

  double sag_base_ratio = 0.0;
  double sag_length_scale = 1.0;
  double sag_pass_scale = 1.0;
  double sag_rigidity_scale = 1.0;
  const double sag_amplitude_m =
      ComputeSagAmplitudeM(start_constraint, end_constraint, chord_length, shape_policy, &sag_base_ratio,
                           &sag_length_scale, &sag_pass_scale, &sag_rigidity_scale);

  double tangent_scale = continuity.handle_scale;
  std::array<Vec3d, 4> cp{};
  int fallback_iterations = 0;
  bool degraded = (continuity.mode == DetailCurveContinuityMode::kG1) && !prefer_g1_policy;
  double h0 = 0.0;
  double h1 = 0.0;
  HandleLengthComponents start_handle{};
  HandleLengthComponents end_handle{};
  for (; fallback_iterations < 3; ++fallback_iterations) {
    start_handle = ComputeHandleLength(chord_length, chord_dir, start_constraint, shape_policy, tangent_scale);
    end_handle = ComputeHandleLength(chord_length, chord_dir, end_constraint, shape_policy, tangent_scale);
    h0 = start_handle.length_m * CornerPassTangentLengthScale(start_constraint);
    h1 = end_handle.length_m * CornerPassTangentLengthScale(end_constraint);
    h0 = std::min(h0, std::max(chord_length * 0.06, start_tangent_decision.departure_length_m));
    h1 = std::min(h1, std::max(chord_length * 0.06, end_tangent_decision.departure_length_m));
    cp[0] = start_point;
    cp[1] = start_point + ScaleVec(start_tangent, h0);
    cp[2] = end_point - ScaleVec(end_tangent, h1);
    cp[3] = end_point;
    const CurveQualityMetrics quality = EvaluateCurveQuality(cp, sag_amplitude_m);
    if (!CurveHasPoorQuality(quality, shape_policy, chord_length)) {
      break;
    }
    if (fallback_iterations == 0) {
      tangent_scale *= 0.78;
      continue;
    }
    if (fallback_iterations == 1 && continuity.mode == DetailCurveContinuityMode::kG2) {
      continuity.mode = DetailCurveContinuityMode::kG1;
      continuity.reason = DetailCurveContinuityReason::kPoorQualityFallback;
      degraded = true;
      tangent_scale *= 0.88;
      continue;
    }
    tangent_scale *= kFallbackTangentScale;
    degraded = true;
    if (started_as_g2) {
      continuity.mode = DetailCurveContinuityMode::kG1;
      continuity.reason = DetailCurveContinuityReason::kPoorQualityFallback;
    }
  }

  curve.control_points = cp;
  curve.sag_amplitude_m = sag_amplitude_m;
  curve.quality.requested_policy = requested_policy;
  curve.quality.shape_policy = shape_policy.kind;
  curve.quality.adopted_continuity = continuity.mode;
  curve.quality.continuity_reason = continuity.reason;
  curve.quality.tangent_scale = tangent_scale;
  curve.quality.fallback_iterations = fallback_iterations;
  curve.quality.base_handle_scale = 0.5 * (start_handle.base_scale + end_handle.base_scale);
  curve.quality.policy_handle_scale = 0.5 * (start_handle.policy_scale + end_handle.policy_scale);
  curve.quality.start_angle_scale = start_handle.angle_scale;
  curve.quality.end_angle_scale = end_handle.angle_scale;
  curve.quality.handle_length_start_m = h0;
  curve.quality.handle_length_end_m = h1;
  curve.quality.start_tangent_rule = start_tangent_decision.rule;
  curve.quality.end_tangent_rule = end_tangent_decision.rule;
  curve.quality.start_support_weight = start_tangent_decision.support_weight;
  curve.quality.end_support_weight = end_tangent_decision.support_weight;
  curve.quality.start_chord_weight = start_tangent_decision.chord_weight;
  curve.quality.end_chord_weight = end_tangent_decision.chord_weight;
  curve.quality.start_departure_length_m = start_tangent_decision.departure_length_m;
  curve.quality.end_departure_length_m = end_tangent_decision.departure_length_m;
  curve.quality.start_lateral_ratio_limit = start_tangent_decision.lateral_ratio_limit;
  curve.quality.end_lateral_ratio_limit = end_tangent_decision.lateral_ratio_limit;
  curve.quality.sag_base_ratio = sag_base_ratio;
  curve.quality.sag_length_scale = sag_length_scale;
  curve.quality.sag_pass_scale = sag_pass_scale;
  curve.quality.sag_rigidity_scale = sag_rigidity_scale;
  curve.quality.attempted_g2 = continuity.attempted_g2;
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
