#pragma once

#include "city/wire/entities.hpp"

constexpr double kBackboneDisplayPlaneZ = 0.0;

city::wire::Vec3d ProjectBackbonePointToDisplayPlane(const city::wire::Vec3d& world);
