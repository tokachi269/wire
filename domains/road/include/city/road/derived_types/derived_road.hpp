#pragma once

#include "city/road/common_types.hpp"

#include <array>

namespace city::road {

struct SectionBoundarySample {
  std::uint64_t boundary_id = 0;
  BoundaryRole role = BoundaryRole::kCurb;
  double lateral_m = 0.0;
  double height_m = 0.0;
  MarkingRule marking_rule = MarkingRule::kNone;
};
struct SectionEvaluation {
  RoadSegmentId segment_id = 0;
  double station_m = 0.0;
  std::vector<SectionBoundarySample> boundaries{};
  std::vector<std::string> surface_materials{};
};
struct Mesh {
  RoadSegmentId owner_segment_id = 0;
  std::string material{};
  std::vector<Vec3d> vertices{};
  std::vector<std::uint32_t> indices{};
};
struct TerrainMaskPolygon { RoadSegmentId segment_id = 0; std::vector<Vec2d> points{}; };
struct ConnectionGate {
  RoadSegmentId segment_id = 0;
  RoadNodeId node_id = 0;
  Vec3d position{};
  Vec3d tangent{};
  std::vector<SectionBoundarySample> boundaries{};
};
struct JunctionArea {
  NodeConnectionPolicyOverrideId policy_override_id = 0;
  RoadNodeId node_id = 0;
  std::vector<ConnectionGate> gates{};
};
struct ConnectionArea { RoadNodeId node_id = 0; std::vector<ConnectionGate> gates{}; };
struct CanonicalAlignment { RoadSegmentId segment_id = 0; Path path{}; };
struct DerivedRoad {
  std::array<std::size_t, 9> build_stage_runs{};
  std::vector<CanonicalAlignment> canonical_alignments{};
  std::vector<SectionEvaluation> section_evaluations{};
  std::vector<Mesh> segment_meshes{};
  std::vector<Mesh> marking_meshes{};
  std::vector<TerrainMaskPolygon> terrain_masks{};
  std::vector<ConnectionGate> connection_gates{};
  std::vector<ConnectionArea> connection_areas{};
  std::vector<Mesh> connection_meshes{};
  std::vector<JunctionArea> junction_areas{};
  std::vector<Mesh> junction_meshes{};
  std::vector<Mesh> junction_marking_meshes{};
  std::vector<Mesh> manual_marking_meshes{};
};

} // namespace city::road
