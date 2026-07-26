#pragma once

#include "city/road/common_types.hpp"

#include <optional>

namespace city::road {

struct SectionTransition {
  SectionTransitionId id = 0;
  CrossSectionTemplateId from_template = 0;
  CrossSectionTemplateId to_template = 0;
  StationRef start{};
  StationRef end{};
  TransitionAnchor anchor = TransitionAnchor::kCenter;
  std::vector<SectionTransitionRule> rules{};
};
struct RoadNode { RoadNodeId id = 0; Vec2d position{}; };
struct SegmentKnot { Vec2d position{}; Vec2d handle_in{}; Vec2d handle_out{}; };
struct SegmentShape {
  Vec2d start_handle{};
  std::vector<SegmentKnot> internal_knots{};
  Vec2d end_handle{};
};
struct RoadSegment {
  RoadSegmentId id = 0;
  RoadNodeId node_a = 0;
  RoadNodeId node_b = 0;
  SegmentShape shape{};
  CrossSectionTemplateId section_template = 0;
  std::optional<SectionTransitionId> transition{};
};
enum class NodeConnectionPolicy { kForcePassThrough, kForceCorner, kForceJunction };
struct NodeConnectionPolicyOverride {
  NodeConnectionPolicyOverrideId id = 0;
  RoadNodeId node_id = 0;
  NodeConnectionPolicy policy = NodeConnectionPolicy::kForceJunction;
};
struct ManualLineMarking {
  ManualMarkingId id = 0;
  RoadSegmentId owner_segment_id = 0;
  Path path{};
  std::string style{};
};
struct ManualAreaMarking {
  ManualMarkingId id = 0;
  RoadSegmentId owner_segment_id = 0;
  Vec2d frame_origin{};
  double width_m = 0.0;
  double length_m = 0.0;
  std::string style{};
};
struct SavedRoadGraph {
  std::vector<RoadNode> nodes{};
  std::vector<RoadSegment> segments{};
  std::vector<CrossSectionTemplate> section_templates{};
  std::vector<SectionTransition> transitions{};
  std::vector<NodeConnectionPolicyOverride> connection_policy_overrides{};
  std::vector<ManualLineMarking> manual_lines{};
  std::vector<ManualAreaMarking> manual_areas{};
};

} // namespace city::road
