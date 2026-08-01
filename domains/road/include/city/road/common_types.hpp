#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <tuple>
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
using RoadCorridorId = std::uint64_t;
using SectionStripId = std::uint64_t;
using LaneId = std::uint64_t;
using BoundaryId = std::uint64_t;
using LaneConnectionId = std::uint64_t;
using BoundaryContinuationId = std::uint64_t;
using CrossSectionTemplateId = std::uint64_t;
using SectionTransitionId = std::uint64_t;
using ManualMarkingId = std::uint64_t;
using NodeConnectionPolicyOverrideId = std::uint64_t;
using JunctionMarkingOverrideId = std::uint64_t;

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
inline constexpr MarkingStyleId kWhiteDashed{5};
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
         id == builtin_marking_styles::kCrosswalk ||
         id == builtin_marking_styles::kWhiteDashed;
}

enum class StripFunction { kCarriageway, kShoulder, kSidewalk, kMedian };
enum class BoundaryRole { kOuterEdge, kCurb, kLaneDivider, kMedianEdge };
enum class MarkingRole {
  kLaneSeparator,
  kCenterLine,
  kCarriagewayEdge,
  kStopLine,
  kCrosswalk,
  kFree,
};
struct AutoMarkingPolicy {
  bool enabled = false;
  MarkingRole role = MarkingRole::kLaneSeparator;
  MarkingStyleId style_id{};

  bool operator==(const AutoMarkingPolicy&) const = default;
};
// A lane requests a line on its own side. The request resolves to the
// BoundaryProfile adjacent in template element order; the boundary owns the line.
struct LaneSideMarkingPolicy {
  AutoMarkingPolicy left{};
  AutoMarkingPolicy right{};

  bool operator==(const LaneSideMarkingPolicy&) const = default;
};
enum class DistanceRefKind { kFromStart, kFromEnd, kRatio };
enum class TransitionAction { kContinue, kChangeWidthHeightOffset, kTaperOut, kTaperIn, kEndCap, kUnsupported };
enum class TransitionAnchor { kCenter, kLeftEdge, kRightEdge };
enum class EndpointRole {
  kStart,
  kEnd,
};
enum class LaneTravelDirection {
  kAlongSegment,
  kAgainstSegment,
};
enum class LaneConnectionKind {
  kContinuation,
  kTransition,
  kMerge,
  kSplit,
  kJunctionMovement,
};

struct LaneEndpointKey {
  RoadSegmentId segment_id = 0;
  LaneId lane_id = 0;
  EndpointRole endpoint_role = EndpointRole::kStart;

  bool operator==(const LaneEndpointKey&) const = default;
  bool operator<(const LaneEndpointKey& other) const {
    return std::tie(segment_id, lane_id, endpoint_role) <
           std::tie(other.segment_id, other.lane_id, other.endpoint_role);
  }
};

struct BoundaryEndpointKey {
  RoadSegmentId segment_id = 0;
  BoundaryId boundary_id = 0;
  EndpointRole endpoint_role = EndpointRole::kStart;

  bool operator==(const BoundaryEndpointKey&) const = default;
  bool operator<(const BoundaryEndpointKey& other) const {
    return std::tie(segment_id, boundary_id, endpoint_role) <
           std::tie(other.segment_id, other.boundary_id, other.endpoint_role);
  }
};

struct DirectedSegmentRef {
  RoadSegmentId segment_id = 0;
  bool reversed = false;

  bool operator==(const DirectedSegmentRef&) const = default;
};

struct CorridorDistanceRef {
  RoadCorridorId corridor_id = 0;
  double corridor_distance_m = 0.0;
};

struct ResolvedSegmentDistance {
  RoadSegmentId segment_id = 0;
  double segment_distance_m = 0.0;
  bool reversed = false;
};

struct RepeatingPlacementPolicy {
  double spacing_m = 0.0;
  double phase_m = 0.0;
};

enum class RoadSide {
  kLeft,
  kRight,
};

struct RoadSideRef {
  RoadSegmentId segment_id = 0;
  RoadSide side = RoadSide::kLeft;
  double segment_distance_m = 0.0;
  double lateral_offset_m = 0.0;
};

struct RoadSideIntervalRef {
  RoadSegmentId segment_id = 0;
  RoadSide side = RoadSide::kLeft;
  double start_segment_distance_m = 0.0;
  double end_segment_distance_m = 0.0;
};

struct CorridorSideRef {
  RoadCorridorId corridor_id = 0;
  RoadSide side = RoadSide::kLeft;
  double corridor_distance_m = 0.0;
  double lateral_offset_m = 0.0;
};

struct ApproachKey {
  RoadNodeId node_id = 0;
  RoadSegmentId segment_id = 0;
  EndpointRole endpoint_role = EndpointRole::kStart;

  bool operator==(const ApproachKey&) const = default;
  bool operator<(const ApproachKey& other) const {
    return std::tie(node_id, segment_id, endpoint_role) <
           std::tie(other.node_id, other.segment_id, other.endpoint_role);
  }
};
struct MarkingOwner {
  enum class Kind {
    kRoadSegment,
    kJunction,
    kManual,
  };
  Kind kind = Kind::kRoadSegment;
  RoadSegmentId segment_id = 0;
  RoadNodeId node_id = 0;
  ManualMarkingId manual_id = 0;

  bool operator==(const MarkingOwner&) const = default;
  bool operator<(const MarkingOwner& other) const {
    return std::tie(kind, segment_id, node_id, manual_id) <
           std::tie(other.kind, other.segment_id, other.node_id, other.manual_id);
  }
};
struct MarkingTrackKey {
  RoadSegmentId segment_id = 0;
  BoundaryId boundary_id = 0;
  MarkingRole role = MarkingRole::kLaneSeparator;

  bool operator==(const MarkingTrackKey&) const = default;
  bool operator<(const MarkingTrackKey& other) const {
    return std::tie(segment_id, boundary_id, role) <
           std::tie(other.segment_id, other.boundary_id, other.role);
  }
};
struct AutoMarkingKey {
  MarkingOwner owner{};
  MarkingRole role = MarkingRole::kLaneSeparator;
  std::optional<MarkingTrackKey> track{};
  std::optional<ApproachKey> approach{};

  bool operator==(const AutoMarkingKey&) const = default;
  bool operator<(const AutoMarkingKey& other) const {
    return std::tie(owner, role, track, approach) <
           std::tie(other.owner, other.role, other.track, other.approach);
  }
};

struct SectionStrip {
  SectionStripId id = 0;
  StripFunction function = StripFunction::kCarriageway;
  double width_m = 0.0;
  double cross_slope = 0.0;
  SurfaceStyleId style_id{};
  LaneSideMarkingPolicy side_marking{};
};
struct LaneBand {
  LaneId id = 0;
  SectionStripId surface_strip_id = 0;
  double lateral_start_m = 0.0;
  double lateral_end_m = 0.0;
  LaneTravelDirection direction = LaneTravelDirection::kAlongSegment;
};
struct BoundaryProfile {
  BoundaryId boundary_id = 0;
  BoundaryRole role = BoundaryRole::kCurb;
  double width_m = 0.0;
  double height_m = 0.0;
  AutoMarkingPolicy marking{};
};
struct CrossSectionTemplate {
  CrossSectionTemplateId id = 0;
  std::vector<SectionStrip> strips{};
  std::vector<LaneBand> lane_bands{};
  std::vector<BoundaryProfile> boundaries{};
};
struct DistanceRef { DistanceRefKind kind = DistanceRefKind::kFromStart; double value = 0.0; };
struct SectionTransitionRule {
  SectionStripId strip_id = 0;
  TransitionAction action = TransitionAction::kContinue;
};

} // namespace city::road
