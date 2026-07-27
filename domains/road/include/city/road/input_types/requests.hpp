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
  double station_m = 0.0;
};
struct ExtendSegmentRequest {
  RoadSegmentId segment_id = 0;
  RoadNodeId endpoint_node_id = 0;
  Path extension{};
  CrossSectionTemplateId section_template = 0;
};
struct EditSegmentShapeRequest { RoadSegmentId segment_id = 0; SegmentShape shape{}; };
struct MoveNodeRequest { RoadNodeId node_id = 0; Vec2d position{}; };
struct DeleteSegmentRequest { RoadSegmentId segment_id = 0; };
struct AddSectionTemplateRequest { CrossSectionTemplate section_template{}; };
struct EditSectionTemplateRequest { CrossSectionTemplate section_template{}; };
struct SectionTransitionRequest {
  CrossSectionTemplateId from_template = 0;
  CrossSectionTemplateId to_template = 0;
  StationRef start{};
  StationRef end{};
  TransitionAnchor anchor = TransitionAnchor::kCenter;
  std::vector<SectionTransitionRule> rules{};
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
  double width_m = 0.0;
  double length_m = 0.0;
  MarkingStyleId style_id = builtin_marking_styles::kWhiteSolid;
};
struct RoadToolDraft {
  Path preview_path{};
  bool has_live_preview = false;
  bool supports_bezier_handles = false;
};

} // namespace city::road
