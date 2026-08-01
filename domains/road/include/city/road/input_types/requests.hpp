#pragma once

#include "city/road/authoritative_types/road_graph.hpp"

#include <optional>

namespace city::road {

struct AddSegmentRequest { Path alignment{}; CrossSectionTemplateId section_template = 0; };
struct AddSegmentConnectedToRequest {
  Path alignment{};
  CrossSectionTemplateId section_template = 0;
  RoadNodeId start_node = 0;
};
struct AddSegmentConnectedToSegmentRequest {
  Path alignment{};
  CrossSectionTemplateId section_template = 0;
  RoadSegmentId start_segment = 0;
  double segment_distance_m = 0.0;
};
struct ExtendCorridorFromEndRequest {
  RoadCorridorId corridor_id = 0;
  RoadNodeId endpoint_node_id = 0;
  Path extension{};
  CrossSectionTemplateId section_template = 0;
};
struct SplitSegmentAtDistanceRequest {
  RoadSegmentId segment_id = 0;
  double segment_distance_m = 0.0;
};
struct DeleteSegmentRangeRequest {
  RoadSegmentId segment_id = 0;
  double start_segment_distance_m = 0.0;
  double end_segment_distance_m = 0.0;
};
struct EditSegmentShapeRequest { RoadSegmentId segment_id = 0; SegmentShape shape{}; };
struct MoveNodeRequest { RoadNodeId node_id = 0; Vec2d position{}; };
struct DeleteSegmentRequest { RoadSegmentId segment_id = 0; };
struct SetApproachSetbackOverrideRequest {
  ApproachKey key{};
  double setback_m = 0.0;
};
struct SetApproachLateralShiftOverrideRequest {
  ApproachKey key{};
  double lateral_shift_m = 0.0;
};
enum class ApproachOverrideField {
  kSetback,
  kLateralShift,
};
struct ResetApproachOverrideFieldRequest {
  ApproachKey key{};
  ApproachOverrideField field = ApproachOverrideField::kSetback;
};
struct ResetAllApproachOverridesRequest {
  ApproachKey key{};
};
struct AddSectionTemplateRequest { CrossSectionTemplate section_template{}; };
struct EditSectionTemplateRequest { CrossSectionTemplate section_template{}; };
struct SetBoundaryMarkingPolicyRequest {
  CrossSectionTemplateId section_template_id = 0;
  std::uint64_t boundary_id = 0;
  AutoMarkingPolicy policy{};
};
struct ResetBoundaryMarkingPolicyRequest {
  CrossSectionTemplateId section_template_id = 0;
  std::uint64_t boundary_id = 0;
};
enum class LaneSide {
  kLeft,
  kRight,
};
struct SetLaneSideMarkingPolicyRequest {
  CrossSectionTemplateId section_template_id = 0;
  SectionStripId strip_id = 0;
  LaneSide side = LaneSide::kLeft;
  AutoMarkingPolicy policy{};
};
struct ResetLaneSideMarkingPolicyRequest {
  CrossSectionTemplateId section_template_id = 0;
  SectionStripId strip_id = 0;
  LaneSide side = LaneSide::kLeft;
};
struct SectionTransitionRequest {
  CrossSectionTemplateId from_template = 0;
  CrossSectionTemplateId to_template = 0;
  DistanceRef start{};
  DistanceRef end{};
  TransitionAnchor anchor = TransitionAnchor::kCenter;
  BoundaryId anchor_boundary_id = 0;
  std::vector<SectionTransitionRule> rules{};
};
struct AddLaneTransitionRequest {
  RoadCorridorId corridor_id = 0;
  LaneTravelDirection direction = LaneTravelDirection::kAlongSegment;
  RoadSide side = RoadSide::kRight;
  double start_corridor_distance_m = 0.0;
  double full_width_corridor_distance_m = 0.0;
  double lane_width_m = 0.0;
  BoundaryId anchor_boundary_id = 0;
};
struct AddTransitionToSegmentRequest {
  RoadSegmentId segment_id = 0;
  SectionTransitionRequest transition{};
};
struct AttachSectionTransitionRequest {
  RoadSegmentId segment_id = 0;
  SectionTransitionId transition_id = 0;
};
struct ManualLineRequest {
  RoadSegmentId owner_segment_id = 0;
  Path path{};
  MarkingStyleId style_id = builtin_marking_styles::kWhiteSolid;
};
struct ManualAreaRequest {
  RoadSegmentId owner_segment_id = 0;
  Vec2d frame_origin{};
  double rotation_rad = 0.0;
  double width_m = 0.0;
  double length_m = 0.0;
  MarkingStyleId style_id = builtin_marking_styles::kWhiteSolid;
};
struct SuppressAutoMarkingRequest {
  AutoMarkingKey key{};
};
struct ResetAutoMarkingSuppressionRequest {
  AutoMarkingKey key{};
};
struct SetJunctionMarkingOverrideRequest {
  JunctionMarkingOverride override{};
};
struct DeleteJunctionMarkingOverrideRequest {
  JunctionMarkingOverrideId id = 0;
};
struct RoadToolDraft {
  Path preview_path{};
  bool has_live_preview = false;
  bool supports_bezier_handles = false;
};

} // namespace city::road
