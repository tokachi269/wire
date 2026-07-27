#pragma once

#include "city/road/derived_types/derived_road.hpp"

#include <vector>

namespace city::road::materialization {

struct SegmentSample {
  Vec2d center{};
  Vec2d tangent{};
  std::vector<SectionBoundarySample> boundaries{};
  std::vector<RenderStyleRef> surface_styles{};
};

struct SegmentInput {
  RoadSegmentId segment_id = 0;
  std::vector<SegmentSample> samples{};
};

struct SegmentOutput {
  std::vector<Mesh> surface_meshes{};
  std::vector<Mesh> marking_meshes{};
  TerrainMaskPolygon terrain_mask{};
};

struct ManualLineInput {
  ManualMarkingId marking_id = 0;
  RenderStyleRef style = RenderStyleFromMarking(builtin_marking_styles::kWhiteSolid);
  std::vector<Vec3d> left{};
  std::vector<Vec3d> right{};
};

struct ManualAreaInput {
  ManualMarkingId marking_id = 0;
  RenderStyleRef style = RenderStyleFromMarking(builtin_marking_styles::kWhiteSolid);
  std::array<Vec3d, 4> corners{};
};

struct JunctionOutput {
  std::vector<Mesh> surface_meshes{};
  std::vector<Mesh> marking_meshes{};
};

[[nodiscard]] Result<SegmentOutput> MaterializeSegment(const SegmentInput& input);
[[nodiscard]] Result<Mesh> MaterializeManualLine(const ManualLineInput& input);
[[nodiscard]] Result<Mesh> MaterializeManualArea(const ManualAreaInput& input);
[[nodiscard]] Result<std::vector<Mesh>> MaterializeConnection(const ConnectionGeometry& input);
[[nodiscard]] Result<JunctionOutput> MaterializeJunction(const JunctionGeometry& input);

} // namespace city::road::materialization
