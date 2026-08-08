#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace city::road {

inline constexpr double kDefaultRoadCornerRadiusM = 4.0;

struct Vec2d { double x = 0.0; double y = 0.0; };
struct Vec3d { double x = 0.0; double y = 0.0; double z = 0.0; };

enum class CommitFailureCategory : std::uint8_t {
  kNone = 0,
  kRequirementConstraint = 1,
  kInvalidInput = 2,
  kNotImplemented = 3,
  kStateConflict = 4,
  kInternalError = 5,
};

[[nodiscard]] inline const char* DefaultReasonCode(CommitFailureCategory category) {
  switch (category) {
    case CommitFailureCategory::kNone: return "";
    case CommitFailureCategory::kRequirementConstraint: return "requirement_constraint";
    case CommitFailureCategory::kInvalidInput: return "invalid_input";
    case CommitFailureCategory::kNotImplemented: return "not_implemented";
    case CommitFailureCategory::kStateConflict: return "state_conflict";
    case CommitFailureCategory::kInternalError: return "internal_error";
  }
  return "internal_error";
}

template <typename T>
struct Result {
  bool ok = false;
  T value{};
  CommitFailureCategory failure_category = CommitFailureCategory::kNone;
  std::string error{};
  std::string reason_code{};

  [[nodiscard]] static Result<T> Ok(T value) {
    Result<T> result{};
    result.ok = true;
    result.value = std::move(value);
    return result;
  }
  [[nodiscard]] static Result<T> Fail(CommitFailureCategory kind,
                                      std::string error,
                                      std::string reason_code = {}) {
    Result<T> result{};
    result.failure_category = kind;
    result.error = std::move(error);
    result.reason_code = reason_code.empty() ? DefaultReasonCode(kind) : std::move(reason_code);
    return result;
  }
};

struct BezierSpan { Vec2d p0{}; Vec2d p1{}; Vec2d p2{}; Vec2d p3{}; };
struct Path { std::vector<BezierSpan> spans{}; };

using RoadNodeId = std::uint64_t;
using RoadSegmentId = std::uint64_t;
using RoadCorridorId = std::uint64_t;
using RoadLayoutStripId = std::uint64_t;
using LaneId = std::uint64_t;
using BoundaryId = std::uint64_t;
using LaneConnectionId = std::uint64_t;
using BoundaryContinuationId = std::uint64_t;
using RoadLayoutTemplateId = std::uint64_t;
using RoadLayoutTransitionId = std::uint64_t;
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
// Inside and outside are toward and away from the carriageway. kUnspecified is
// rejected rather than defaulted.
enum class MarkingPlacement {
  kUnspecified,
  kCenter,
  kInside,
  kOutside,
};
struct AutoMarkingPolicy {
  bool enabled = false;
  MarkingRole role = MarkingRole::kLaneSeparator;
  MarkingStyleId style_id{};
  MarkingPlacement placement = MarkingPlacement::kUnspecified;

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
enum class TransitionAnchor { kCenter, kLeftEdge, kRightEdge, kBoundary };
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
enum class BoundaryContinuationKind {
  kContinuation,
  kMerge,
  kSplit,
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

struct SegmentPosition {
  RoadSegmentId segment_id = 0;
  double t = 0.0;
};

enum class RoadSide {
  kLeft,
  kRight,
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

struct RoadLayoutStrip {
  RoadLayoutStripId id = 0;
  StripFunction function = StripFunction::kCarriageway;
  double width_m = 0.0;
  double cross_slope = 0.0;
  SurfaceStyleId style_id{};
  LaneSideMarkingPolicy side_marking{};
};
struct LaneBand {
  LaneId id = 0;
  RoadLayoutStripId surface_strip_id = 0;
  double lateral_start_m = 0.0;
  double lateral_end_m = 0.0;
  LaneTravelDirection direction = LaneTravelDirection::kAlongSegment;
};
// Measured from the edge datum, the road-facing vertical face of whatever sits
// there. Lateral runs the way the section does, left to right.
struct ProfilePoint {
  double lateral_m = 0.0;
  double height_m = 0.0;
};
// A boundary occupies no layout width: its contour reaches into the strips
// beside it, which is what lets a gutter's top face come out of the walkway's
// declared width instead of being added to it.
struct BoundaryProfile {
  BoundaryId boundary_id = 0;
  BoundaryRole role = BoundaryRole::kCurb;
  std::vector<ProfilePoint> contour{};
  std::vector<SurfaceStyleId> segment_styles{};
  AutoMarkingPolicy marking{};
};
struct RoadLayoutTemplate {
  RoadLayoutTemplateId id = 0;
  std::vector<RoadLayoutStrip> strips{};
  std::vector<LaneBand> lane_bands{};
  std::vector<BoundaryProfile> boundaries{};
  // Measured from the layout's left outer end, looking along the segment. NaN
  // so a layout that never set it is rejected rather than silently centred.
  double alignment_offset_from_left_m =
      std::numeric_limits<double>::quiet_NaN();
};
struct DistanceRef { DistanceRefKind kind = DistanceRefKind::kFromStart; double value = 0.0; };
struct RoadLayoutTransitionRule {
  RoadLayoutStripId strip_id = 0;
  TransitionAction action = TransitionAction::kContinue;
};

} // namespace city::road
