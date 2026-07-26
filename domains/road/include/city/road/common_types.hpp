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
  std::string style{};
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
