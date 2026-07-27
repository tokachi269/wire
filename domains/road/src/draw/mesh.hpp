#pragma once

#include "city/road/derived_types/derived_road.hpp"

#include <vector>

namespace city::road::draw {

struct segment_sample {
  Vec2d center{};
  Vec2d tangent{};
  std::vector<SectionBoundarySample> boundaries{};
  std::vector<RenderStyleRef> surface_styles{};
};

struct segment_input {
  RoadSegmentId segment_id = 0;
  std::vector<segment_sample> samples{};
};

struct segment_output {
  std::vector<Mesh> surface_meshes{};
  TerrainMaskPolygon terrain_mask{};
};

struct junction_output {
  std::vector<Mesh> surface_meshes{};
};

[[nodiscard]] Result<segment_output> make_segment(const segment_input &input);
[[nodiscard]] Result<std::vector<Mesh>>
make_connection(const ConnectionGeometry &input);
[[nodiscard]] Result<junction_output>
make_junction(const JunctionGeometry &input);
[[nodiscard]] Result<std::vector<Mesh>>
make_markings(const ResolvedMarkingGraph &input);

} // namespace city::road::draw