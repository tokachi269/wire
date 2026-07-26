#pragma once

#include "city/road/road.hpp"

#include <string>
#include <vector>

namespace city::road::materialization {

struct SegmentSample {
  Vec2d center{};
  Vec2d tangent{};
  std::vector<SectionBoundarySample> boundaries{};
  std::vector<std::string> surface_materials{};
};

struct SegmentInput {
  RoadSegmentId segment_id = 0;
  std::vector<SegmentSample> samples{};
};

struct SegmentOutput {
  std::vector<Mesh> surface_meshes{};
  Mesh marking_mesh{};
  TerrainMaskPolygon terrain_mask{};
};

struct ManualLineInput {
  ManualMarkingId marking_id = 0;
  std::vector<Vec3d> left{};
  std::vector<Vec3d> right{};
};

struct ManualAreaInput {
  ManualMarkingId marking_id = 0;
  std::array<Vec3d, 4> corners{};
};

struct ConnectionInput {
  ConnectionArea area{};
  Vec2d node_position{};
  std::vector<std::string> surface_materials{};
};

struct JunctionInput {
  JunctionArea area{};
  Vec2d node_position{};
};

struct JunctionOutput {
  std::vector<Mesh> surface_meshes{};
  std::vector<Mesh> marking_meshes{};
};

[[nodiscard]] Result<SegmentOutput> MaterializeSegment(const SegmentInput& input);
[[nodiscard]] Result<Mesh> MaterializeManualLine(const ManualLineInput& input);
[[nodiscard]] Result<Mesh> MaterializeManualArea(const ManualAreaInput& input);
[[nodiscard]] Result<std::vector<Mesh>> MaterializeConnection(const ConnectionInput& input);
[[nodiscard]] Result<JunctionOutput> MaterializeJunction(const JunctionInput& input);

} // namespace city::road::materialization
