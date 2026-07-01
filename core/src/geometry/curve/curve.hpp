#pragma once

#include "wire/core/core_edit_types.hpp"
#include "wire/core/detail_curve.hpp"
#include "wire/core/types.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace wire::core::geometry::curve {

enum class CurveFamily : std::uint8_t {
  kMainSpan = 0,
  kSupportLead = 1,
  kJumper = 2,
  kVisualDetail = 3,
};

enum class CurveMethod : std::uint8_t {
  kParabolicSag = 0,
  kCubicHermiteSag = 1,
};

enum class AttachmentCurveMethod : std::uint8_t {
  kCubicHermite = 0,
};

enum class CurveRegionKind : std::uint8_t {
  kStartAttachment = 0,
  kMainSpan = 1,
  kEndAttachment = 2,
};

enum class StableFramePolicy : std::uint8_t {
  kParallelTransportCanonical = 0,
};

struct CableCurveProfile {
  double attachment_blend_length_m = 1.0;
  double attachment_min_bend_radius_m = 0.25;
  double plan_view_blend_length_m = 0.75;
  double plan_view_max_deviation_m = 0.35;
  AttachmentCurveMethod attachment_method = AttachmentCurveMethod::kCubicHermite;
};

struct TessellationPolicy {
  std::size_t min_segments = 2;
  std::size_t max_segments = 128;
  double meters_per_segment = 2.0;
  double sag_m_per_segment = 0.20;
};

struct CableCurveInput {
  Vec3d start{};
  Vec3d end{};
  Vec3d start_tangent_hint{};
  Vec3d end_tangent_hint{};
  Vec3d gravity_dir{0.0, 0.0, -1.0};
  Vec3d canonical_dir{1.0, 0.0, 0.0};
  double sag_m = 0.0;
  double tension_hint = 0.0;
  double radius_m = 0.0;
  CurveFamily family = CurveFamily::kMainSpan;
  CurveMethod method = CurveMethod::kParabolicSag;
  CableCurveProfile profile{};
  TessellationPolicy tessellation{};
};

struct CableCurveSample {
  double parameter = 0.0;
  Vec3d position{};
  Vec3d tangent{};
  Vec3d normal{};
  Vec3d binormal{};
  double speed_m_per_u = 0.0;
  double arc_length_m = 0.0;
  CurveRegionKind region = CurveRegionKind::kMainSpan;
};

struct AttachmentRegion {
  CurveRegionKind kind = CurveRegionKind::kStartAttachment;
  std::size_t first_sample = 0;
  std::size_t last_sample = 0;
  double start_arc_length_m = 0.0;
  double end_arc_length_m = 0.0;
};

struct CableRunSection {
  std::size_t first_sample = 0;
  std::size_t last_sample = 0;
  double start_arc_length_m = 0.0;
  double end_arc_length_m = 0.0;
};

struct CableRunShape {
  std::vector<CableCurveSample> samples{};
  double length_m = 0.0;
  AABBd bounds{};
  std::vector<AttachmentRegion> attachment_regions{};
  std::vector<CableRunSection> sections{};
  StableFramePolicy frame_policy = StableFramePolicy::kParallelTransportCanonical;
};

using CableCurveOutput = CableRunShape;

struct CableMemberProfile {
  double frame_offset_normal_m = 0.0;
  double frame_offset_binormal_m = 0.0;
};

struct CableMemberShape {
  std::vector<Vec3d> positions{};
  std::vector<double> arc_length_m{};
};

[[nodiscard]] std::size_t ResolveSegmentCount(const CableCurveInput& input);
[[nodiscard]] EditResult<CableCurveOutput> BuildCableCurve(const CableCurveInput& input);
[[nodiscard]] EditResult<CableRunShape> BuildCableRun(const std::vector<CableCurveInput>& sections);
[[nodiscard]] EditResult<CableMemberShape> ExpandCableMember(const CableRunShape& run,
                                                             const CableMemberProfile& profile);
[[nodiscard]] DetailCurve ToDetailCurve(const CableCurveInput& input, const CableCurveOutput& output);

} // namespace wire::core::geometry::curve
