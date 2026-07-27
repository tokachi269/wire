#pragma once

#include "pipeline.hpp"

namespace city::road::build {

inline constexpr double station_epsilon = 1e-9;
inline constexpr double surface_sample_step_m = 2.0;

[[nodiscard]] bool is_finite(double value);
[[nodiscard]] bool is_finite(Vec2d value);
[[nodiscard]] Vec2d add(Vec2d a, Vec2d b);
[[nodiscard]] Vec2d subtract(Vec2d a, Vec2d b);
[[nodiscard]] Vec2d scale(Vec2d value, double factor);
[[nodiscard]] double dot(Vec2d a, Vec2d b);
[[nodiscard]] double cross(Vec2d a, Vec2d b);
[[nodiscard]] double magnitude(Vec2d value);
[[nodiscard]] Vec2d normalize(Vec2d value);
[[nodiscard]] Vec3d to3(Vec2d value, double z = 0.0);
[[nodiscard]] ApproachKey make_approach_key(const RoadSegment &segment,
                                            RoadNodeId node_id);
[[nodiscard]] Result<Vec2d> inward_tangent(const RoadSegment &segment,
                                           const Path &alignment,
                                           const ApproachKey &key);
[[nodiscard]] Result<Vec2d> tangent_at(const Path &alignment, double station_m);
[[nodiscard]] double endpoint_station(const ApproachKey &key, double length_m);
void sort_unique_stations(std::vector<double> &stations);

} // namespace city::road::build