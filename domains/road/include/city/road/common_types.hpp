#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace city::road {

struct Vec2d { double x = 0.0; double y = 0.0; };
struct Vec3d { double x = 0.0; double y = 0.0; double z = 0.0; };

enum class ErrorKind { kNone, kValidation, kUnsupported, kInternal };

template <typename T>
struct Result {
  bool ok = false;
  T value{};
  ErrorKind error_kind = ErrorKind::kNone;
  std::string error{};

  [[nodiscard]] static Result<T> Ok(T value) {
    Result<T> result{};
    result.ok = true;
    result.value = std::move(value);
    return result;
  }
  [[nodiscard]] static Result<T> Fail(ErrorKind kind, std::string error) {
    Result<T> result{};
    result.error_kind = kind;
    result.error = std::move(error);
    return result;
  }
};

struct BezierSpan { Vec2d p0{}; Vec2d p1{}; Vec2d p2{}; Vec2d p3{}; };
struct Path { std::vector<BezierSpan> spans{}; };

using RoadNodeId = std::uint64_t;
using RoadSegmentId = std::uint64_t;
using CrossSectionTemplateId = std::uint64_t;
using SectionTransitionId = std::uint64_t;
using ManualMarkingId = std::uint64_t;
using NodeConnectionPolicyOverrideId = std::uint64_t;

struct SurfaceStyleId {
  std::uint64_t value = 0;
  bool operator==(const SurfaceStyleId&) const = default;
  bool operator<(const SurfaceStyleId& other) const { return value < other.value; }
};

struct MarkingStyleId {
  std::uint64_t value = 0;
  bool operator==(const MarkingStyleId&) const = default;
  bool operator<(const MarkingStyleId& other) const { return value < other.value; }
};

namespace builtin_surface_styles {
inline constexpr SurfaceStyleId kAsphalt{1};
inline constexpr SurfaceStyleId kSidewalk{2};
inline constexpr SurfaceStyleId kCurb{3};
inline constexpr SurfaceStyleId kMedian{4};
} // namespace builtin_surface_styles

namespace builtin_marking_styles {
inline constexpr MarkingStyleId kWhiteSolid{1};
inline constexpr MarkingStyleId kCenterLine{2};
inline constexpr MarkingStyleId kStopLine{3};
inline constexpr MarkingStyleId kCrosswalk{4};
} // namespace builtin_marking_styles

[[nodiscard]] inline bool IsKnownSurfaceStyle(SurfaceStyleId id) {
  return id == builtin_surface_styles::kAsphalt ||
         id == builtin_surface_styles::kSidewalk ||
         id == builtin_surface_styles::kCurb ||
         id == builtin_surface_styles::kMedian;
}

[[nodiscard]] inline bool IsKnownMarkingStyle(MarkingStyleId id) {
  return id == builtin_marking_styles::kWhiteSolid ||
         id == builtin_marking_styles::kCenterLine ||
         id == builtin_marking_styles::kStopLine ||
         id == builtin_marking_styles::kCrosswalk;
}

enum class SurfaceRole { kCarriageway, kSidewalk, kMedian };
enum class BoundaryRole { kOuterEdge, kCurb, kLaneDivider, kMedianEdge };
enum class MarkingRule { kNone, kCenterLine, kOuterLine };
enum class StationRefKind { kFromStart, kFromEnd, kRatio };
enum class TransitionAction { kContinue, kChangeWidthHeightOffset, kTaperOut, kTaperIn, kEndCap, kUnsupported };
enum class TransitionAnchor { kCenter, kLeftEdge, kRightEdge };

struct SurfaceBand {
  std::uint64_t element_id = 0;
  SurfaceRole role = SurfaceRole::kCarriageway;
  double width_m = 0.0;
  double cross_slope = 0.0;
  SurfaceStyleId style_id{};
};
struct BoundaryProfile {
  std::uint64_t boundary_id = 0;
  BoundaryRole role = BoundaryRole::kCurb;
  double width_m = 0.0;
  double height_m = 0.0;
  MarkingRule marking_rule = MarkingRule::kNone;
};
struct CrossSectionTemplate {
  CrossSectionTemplateId id = 0;
  std::vector<SurfaceBand> bands{};
  std::vector<BoundaryProfile> boundaries{};
};
struct StationRef { StationRefKind kind = StationRefKind::kFromStart; double value = 0.0; };
struct SectionTransitionRule {
  std::uint64_t element_id = 0;
  TransitionAction action = TransitionAction::kContinue;
};

} // namespace city::road
