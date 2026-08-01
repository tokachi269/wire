#pragma once

#include "city/road/common_types.hpp"

#include <optional>
#include <tuple>

namespace city::road {

struct SectionTransition {
  SectionTransitionId id = 0;
  CrossSectionTemplateId from_template = 0;
  CrossSectionTemplateId to_template = 0;
  DistanceRef start{};
  DistanceRef end{};
  TransitionAnchor anchor = TransitionAnchor::kCenter;
  BoundaryId anchor_boundary_id = 0;
  std::vector<SectionTransitionRule> rules{};
};
struct RoadNode { RoadNodeId id = 0; Vec2d position{}; };
struct SegmentKnot { Vec2d position{}; Vec2d handle_in{}; Vec2d handle_out{}; };
enum class SegmentShapeIntent {
  kCurve,
  kStraight,
};
struct SegmentShape {
  Vec2d start_handle{};
  std::vector<SegmentKnot> internal_knots{};
  Vec2d end_handle{};
  SegmentShapeIntent intent = SegmentShapeIntent::kCurve;
};
struct RoadSegment {
  RoadSegmentId id = 0;
  RoadNodeId node_a = 0;
  RoadNodeId node_b = 0;
  SegmentShape shape{};
  CrossSectionTemplateId section_template = 0;
  std::optional<SectionTransitionId> transition{};
};
struct RoadCorridor {
  RoadCorridorId id = 0;
  CrossSectionTemplateId section_template_id = 0;
  std::vector<DirectedSegmentRef> segments{};
};
enum class NodeConnectionPolicy { kForcePassThrough, kForceCorner, kForceJunction };
struct NodeConnectionPolicyOverride {
  NodeConnectionPolicyOverrideId id = 0;
  RoadNodeId node_id = 0;
  NodeConnectionPolicy policy = NodeConnectionPolicy::kForceJunction;
};
struct ManualDoubleOverride {
  bool has_value = false;
  double value = 0.0;
};
struct ApproachGeometryOverride {
  ApproachKey key{};
  ManualDoubleOverride setback_m{};
  ManualDoubleOverride lateral_shift_m{};
};
struct ManualLineMarking {
  ManualMarkingId id = 0;
  RoadSegmentId owner_segment_id = 0;
  Path path{};
  MarkingStyleId style_id{};
};
struct ManualAreaMarking {
  ManualMarkingId id = 0;
  RoadSegmentId owner_segment_id = 0;
  Vec2d frame_origin{};
  double rotation_rad = 0.0;
  double width_m = 0.0;
  double length_m = 0.0;
  MarkingStyleId style_id{};
};
struct AutoMarkingOverride {
  AutoMarkingKey key{};
  bool suppressed = false;
};
struct LaneConnection {
  LaneConnectionId id = 0;
  LaneEndpointKey source{};
  LaneEndpointKey target{};
  LaneConnectionKind kind = LaneConnectionKind::kContinuation;
};
struct BoundaryContinuation {
  BoundaryContinuationId id = 0;
  BoundaryEndpointKey source{};
  BoundaryEndpointKey target{};
};
struct JunctionMarkingEndpoint {
  ApproachKey approach{};
  BoundaryId boundary_id = 0;
  MarkingRole role = MarkingRole::kCenterLine;

  bool operator==(const JunctionMarkingEndpoint&) const = default;
};
enum class JunctionMarkingAction {
  kTerminateAtGate,
  kConnectToApproach,
  kSuppress,
};
struct JunctionMarkingOverride {
  JunctionMarkingOverrideId id = 0;
  RoadNodeId node_id = 0;
  JunctionMarkingEndpoint source{};
  JunctionMarkingAction action = JunctionMarkingAction::kTerminateAtGate;
  std::optional<JunctionMarkingEndpoint> target{};
};
struct SavedRoadGraph {
  std::vector<RoadNode> nodes{};
  std::vector<RoadSegment> segments{};
  std::vector<RoadCorridor> corridors{};
  std::vector<CrossSectionTemplate> section_templates{};
  std::vector<SectionTransition> transitions{};
  std::vector<LaneConnection> lane_connections{};
  std::vector<BoundaryContinuation> boundary_continuations{};
  std::vector<NodeConnectionPolicyOverride> connection_policy_overrides{};
  std::vector<ApproachGeometryOverride> approach_geometry_overrides{};
  std::vector<JunctionMarkingOverride> junction_marking_overrides{};
  std::vector<AutoMarkingOverride> auto_marking_overrides{};
  std::vector<ManualLineMarking> manual_lines{};
  std::vector<ManualAreaMarking> manual_areas{};
};

} // namespace city::road
