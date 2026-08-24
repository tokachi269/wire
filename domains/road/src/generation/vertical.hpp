#pragma once

#include "city/road/derived_types/derived_road.hpp"

#include "../geometry/geometry.hpp"

namespace city::road::generation {

struct RoadFrame {
  Vec3d position{};
  Vec3d tangent{};
  Vec3d lateral{};
  Vec3d normal{0.0, 0.0, 1.0};
};

[[nodiscard]] double elevation_at(const DerivedSegment& segment,
                                  double segment_distance_m);

[[nodiscard]] Result<RoadFrame> road_frame_at(const DerivedSegment& segment,
                                              double segment_distance_m);

[[nodiscard]] Vec3d place_on_frame(const RoadFrame& frame, double lateral_m,
                                   double local_height_m);

} // namespace city::road::generation
