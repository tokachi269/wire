#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "wire/core/entities.hpp"

namespace wire::core {

enum class CurveEndpointMode : std::uint8_t {
  kDirectThrough = 0,
  kOffsetEndpoint = 1,
};

enum class CurvePassMode : std::uint8_t {
  kPassThrough = 0,
  kBranch = 1,
  kTerminate = 2,
};

enum class DetailCurveContinuityMode : std::uint8_t {
  kG1 = 0,
  kG2 = 1,
};

enum class DetailCurveContinuityReason : std::uint8_t {
  kAutoBalanced = 0,
  kSmoothPassThrough = 1,
  kPolicyPreferG1 = 2,
  kShortSpan = 3,
  kBranchPass = 4,
  kCornerPass = 5,
  kEndpointConstraintPriority = 6,
  kConflictingTangents = 7,
  kContextInsufficient = 8,
  kPoorQualityFallback = 9,
};

struct CurveConstraint {
  Vec3d point{};
  Vec3d tangent_dir{};
  double tangent_length_hint_m = 0.0;
  double bend_stiffness_hint = 1.0;
  double min_bend_radius_hint_m = 0.0;
  Vec3d endpoint_offset{};
  double sag_hint = 0.0;
  double slack_hint = 0.0;
  CableContinuityPolicyHint continuity_preference = CableContinuityPolicyHint::kAuto;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  CurveEndpointMode endpoint_mode = CurveEndpointMode::kDirectThrough;
  bool corner_pass = false;
  double corner_angle_deg = 0.0;
};

struct CurveLengthSample {
  double u = 0.0;
  double arc_length_m = 0.0;
};

struct CurveLengthInterval {
  double start_m = 0.0;
  double end_m = 0.0;
};

struct CurveDistanceAttributes {
  std::vector<float> arc_length_m{};
  std::vector<float> arc_length_normalized{};
  std::vector<float> segment_length_m{};
};

struct DetailCurveQualityInfo {
  CableContinuityPolicyHint requested_policy = CableContinuityPolicyHint::kAuto;
  DetailCurveContinuityMode adopted_continuity = DetailCurveContinuityMode::kG1;
  DetailCurveContinuityReason continuity_reason = DetailCurveContinuityReason::kAutoBalanced;
  double tangent_scale = 1.0;
  int fallback_iterations = 0;
  double handle_length_start_m = 0.0;
  double handle_length_end_m = 0.0;
  bool attempted_g2 = false;
  bool degraded_to_g1 = false;
};

struct DetailCurve {
  CurveConstraint start_constraint{};
  CurveConstraint end_constraint{};
  std::array<Vec3d, 4> control_points{};
  double sag_amplitude_m = 0.0;
  std::vector<Vec3d> sample_points{};
  std::vector<CurveLengthSample> arc_length_table{};
  std::vector<CurveLengthInterval> visible_intervals{};
  std::vector<CurveLengthInterval> hidden_intervals{};
  std::vector<CurveLengthInterval> replacement_intervals{};
  CurveDistanceAttributes distance_attributes{};
  DetailCurveQualityInfo quality{};
  double total_length_m = 0.0;

  [[nodiscard]] Vec3d EvaluatePosition(double u) const;
  [[nodiscard]] Vec3d EvaluateTangent(double u) const;
  [[nodiscard]] double Length() const { return total_length_m; }
  [[nodiscard]] double LengthToU(double s) const;
  [[nodiscard]] double NormalizedLengthToU(double x) const;
  [[nodiscard]] Vec3d PositionAtLength(double s) const;
};

[[nodiscard]] DetailCurve BuildDetailCurve(const CurveConstraint& start_constraint,
                                           const CurveConstraint& end_constraint, int sample_count);

} // namespace wire::core
