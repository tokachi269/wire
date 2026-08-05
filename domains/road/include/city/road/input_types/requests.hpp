#pragma once

#include "city/road/authoritative_types/road_graph.hpp"

#include <optional>

namespace city::road {

struct AddSegmentRequest {
  Path alignment{};
  CrossSectionTemplateId section_template = 0;
  // The drawing mode the user chose. Left empty the intent is read back from
  // the control points, which cannot tell a drawn curve from a drawn line.
  std::optional<SegmentShapeIntent> intent{};
};
struct AddSegmentConnectedToRequest {
  Path alignment{};
  CrossSectionTemplateId section_template = 0;
  RoadNodeId start_node = 0;
  EndpointRole connected_endpoint = EndpointRole::kStart;
};
struct AddSegmentConnectedToSegmentRequest {
  Path alignment{};
  CrossSectionTemplateId section_template = 0;
  RoadSegmentId start_segment = 0;
  double segment_distance_m = 0.0;
  EndpointRole connected_endpoint = EndpointRole::kStart;
};
struct ExtendCorridorFromEndRequest {
  RoadCorridorId corridor_id = 0;
  RoadNodeId endpoint_node_id = 0;
  Path extension{};
  CrossSectionTemplateId section_template = 0;
  std::optional<SegmentShapeIntent> intent{};
};
struct SplitSegmentAtDistanceRequest {
  RoadSegmentId segment_id = 0;
  double segment_distance_m = 0.0;
};
struct EditSegmentShapeRequest { RoadSegmentId segment_id = 0; SegmentShape shape{}; };
struct MoveNodeRequest { RoadNodeId node_id = 0; Vec2d position{}; };
struct DeleteSegmentRequest { RoadSegmentId segment_id = 0; };
struct AddSectionTemplateRequest { CrossSectionTemplate section_template{}; };
struct EditSectionTemplateRequest { CrossSectionTemplate section_template{}; };
struct AddLaneRequest {
  RoadCorridorId corridor_id = 0;
  LaneTravelDirection direction = LaneTravelDirection::kAlongSegment;
  RoadSide side = RoadSide::kRight;
  SegmentPosition transition_start{};
  SegmentPosition transition_complete{};
  RoadNodeId continuation_end_node_id = 0;
  double lane_width_m = 0.0;
};
} // namespace city::road
