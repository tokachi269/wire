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
  TessellationPolicy tessellation{};
};

struct CableCurveSample {
  Vec3d position{};
  Vec3d tangent{};
  Vec3d normal{};
  Vec3d binormal{};
  double arc_length_m = 0.0;
};

struct CableCurveOutput {
  std::vector<CableCurveSample> samples{};
  double length_m = 0.0;
  AABBd bounds{};
};

[[nodiscard]] std::size_t ResolveSegmentCount(const CableCurveInput& input);
[[nodiscard]] EditResult<CableCurveOutput> BuildCableCurve(const CableCurveInput& input);
[[nodiscard]] DetailCurve ToDetailCurve(const CableCurveInput& input, const CableCurveOutput& output);

} // namespace wire::core::geometry::curve
