#pragma once

#include "wire/core/entities.hpp"

constexpr double kBackboneDisplayPlaneZ = 0.0;

wire::core::Vec3d ProjectBackbonePointToDisplayPlane(const wire::core::Vec3d& world);
