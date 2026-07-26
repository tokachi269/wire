#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "city/wire/entities.hpp"
#include "city/wire/variation.hpp"

namespace city::wire {

enum class CurveEndpointMode : std::uint8_t {
  kDirectThrough = 0,
  kOffsetEndpoint = 1,
};

enum class CurvePassMode : std::uint8_t {
  kPassThrough = 0,
  kBranch = 1,
  kTerminate = 2,
};

enum class CurveShapePolicyKind : std::uint8_t {
  kNearStraight = 0,
  kSmoothPass = 1,
  kSharpCorner = 2,
  kBranchPass = 3,
  kTerminate = 4,
  kViaAttachment = 5,
};

enum class CurveProfileHint : std::uint8_t {
  kAuto = 0,
  kCompositeHeightTransition = 1,
  kGroupedLoweredSupport = 2,
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

enum class DetailCurveEndpointTangentRule : std::uint8_t {
  kFallbackChord = 0,
  kMainFlowBlend = 1,
  kBranchChordPriority = 2,
  kTerminateEndpointPriority = 3,
  kAttachmentEndpointPriority = 4,
};

enum class DetailCurveSagApplication : std::uint8_t {
  kSeparateProfile = 0,
  kBakedIntoControlCurve = 1,
};

struct CurveConstraint {
  Vec3d point{};
  Vec3d tangent_dir{};
  double tangent_strength = 1.0;
  double tangent_length_hint_m = 0.0;
  double support_departure_length_m = 0.0;
  double bend_stiffness_hint = 1.0;
  double min_bend_radius_hint_m = 0.0;
  Vec3d endpoint_offset{};
  double sag_hint = 0.0;
  double slack_hint = 0.0;
  CableContinuityPolicyHint continuity_preference = CableContinuityPolicyHint::kAuto;
  CurvePassMode pass_mode = CurvePassMode::kPassThrough;
  CurveEndpointMode endpoint_mode = CurveEndpointMode::kDirectThrough;
  CurveProfileHint profile_hint = CurveProfileHint::kAuto;
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

struct DetailReplacementPath {
  ObjectId attachment_id = kInvalidObjectId;
  AttachmentTemplateId attachment_template_id = kInvalidAttachmentTemplateId;
  AttachmentLineInteractionMode interaction_mode = AttachmentLineInteractionMode::kPassThrough;
  CurveLengthInterval replaced_interval{};
  std::vector<Vec3d> points{};
};

struct DetailSupplementalPath {
  ObjectId attachment_id = kInvalidObjectId;
  AttachmentTemplateId attachment_template_id = kInvalidAttachmentTemplateId;
  AttachmentLineInteractionMode interaction_mode = AttachmentLineInteractionMode::kPassThrough;
  std::vector<Vec3d> points{};
};

struct CurveDistanceAttributes {
  std::vector<float> arc_length_m{};
  std::vector<float> arc_length_normalized{};
  std::vector<float> segment_length_m{};
};

struct DetailCurveQualityInfo {
  CableContinuityPolicyHint requested_policy = CableContinuityPolicyHint::kAuto;
  CurveShapePolicyKind shape_policy = CurveShapePolicyKind::kNearStraight;
  DetailCurveContinuityMode adopted_continuity = DetailCurveContinuityMode::kG1;
  DetailCurveContinuityReason continuity_reason = DetailCurveContinuityReason::kAutoBalanced;
  double tangent_scale = 1.0;
  int fallback_iterations = 0;
  double base_handle_scale = 1.0;
  double policy_handle_scale = 1.0;
  double start_angle_scale = 1.0;
  double end_angle_scale = 1.0;
  double handle_length_start_m = 0.0;
  double handle_length_end_m = 0.0;
  DetailCurveEndpointTangentRule start_tangent_rule = DetailCurveEndpointTangentRule::kFallbackChord;
  DetailCurveEndpointTangentRule end_tangent_rule = DetailCurveEndpointTangentRule::kFallbackChord;
  double start_support_weight = 0.0;
  double end_support_weight = 0.0;
  double start_chord_weight = 1.0;
  double end_chord_weight = 1.0;
  double start_departure_length_m = 0.0;
  double end_departure_length_m = 0.0;
  double start_lateral_ratio_limit = 0.0;
  double end_lateral_ratio_limit = 0.0;
  double lateral_suppression = 0.0;
  double sag_base_ratio = 0.0;
  double sag_length_scale = 1.0;
  double sag_pass_scale = 1.0;
  double sag_rigidity_scale = 1.0;
  bool attempted_g2 = false;
  bool degraded_to_g1 = false;
  HierarchicalVariationSample sag_variation{};
};

struct DetailCurveSegment {
  std::array<Vec3d, 4> control_points{};
  double u_start = 0.0;
  double u_end = 1.0;
};

struct DetailCurve {
  CurveConstraint start_constraint{};
  CurveConstraint end_constraint{};
  std::array<Vec3d, 4> control_points{};
  std::vector<DetailCurveSegment> segments{};
  double sag_amplitude_m = 0.0;
  DetailCurveSagApplication sag_application = DetailCurveSagApplication::kSeparateProfile;
  std::vector<Vec3d> sample_points{};
  std::vector<CurveLengthSample> arc_length_table{};
  std::vector<CurveLengthInterval> visible_intervals{};
  std::vector<CurveLengthInterval> hidden_intervals{};
  std::vector<CurveLengthInterval> replacement_intervals{};
  std::vector<DetailReplacementPath> replacement_paths{};
  std::vector<DetailSupplementalPath> supplemental_paths{};
  CurveDistanceAttributes distance_attributes{};
  DetailCurveQualityInfo quality{};
  double total_length_m = 0.0;

  [[nodiscard]] Vec3d EvaluatePosition(double u) const;
  [[nodiscard]] Vec3d EvaluateTangent(double u) const;
  [[nodiscard]] double Length() const { return total_length_m; }
  [[nodiscard]] std::size_t SegmentCount() const { return segments.empty() ? 1u : segments.size(); }
  [[nodiscard]] double LengthToU(double s) const;
  [[nodiscard]] double NormalizedLengthToU(double x) const;
  [[nodiscard]] Vec3d PositionAtLength(double s) const;
};

[[nodiscard]] DetailCurve BuildDetailCurve(const CurveConstraint& start_constraint,
                                           const CurveConstraint& end_constraint, int sample_count);

} // namespace city::wire
