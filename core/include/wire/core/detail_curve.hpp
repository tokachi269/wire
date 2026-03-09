#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "wire/core/entities.hpp"

namespace wire::core {

enum class CurveEndpointMode : std::uint8_t {
  kDirectThrough = 0,
  kViaAttachment = 1,
};

enum class CurvePassMode : std::uint8_t {
  kPassThrough = 0,
  kBranch = 1,
  kTerminate = 2,
};

struct CurveConstraint {
  Vec3d point{};
  Vec3d tangent_dir{};
  double tangent_length_hint_m = 0.0;
  Vec3d attach_offset{};
  double sag_hint = 0.0;
  double slack_hint = 0.0;
  CableContinuityPolicyHint continuity_preference = CableContinuityPolicyHint::kAuto;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  CurveEndpointMode endpoint_mode = CurveEndpointMode::kDirectThrough;
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
  double tangent_scale = 1.0;
  int fallback_iterations = 0;
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
