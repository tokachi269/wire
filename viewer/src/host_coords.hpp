#pragma once

#include "raylib.h"
#include "city/wire/types.hpp"

city::wire::Vec3d HostWorldToInternal(const Vector3& host_world);
Vector3 InternalToHostWorld(const city::wire::Vec3d& internal_world);
