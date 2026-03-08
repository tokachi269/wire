#pragma once

#include "raylib.h"
#include "wire/core/types.hpp"

wire::core::Vec3d HostWorldToInternal(const Vector3& host_world);
Vector3 InternalToHostWorld(const wire::core::Vec3d& internal_world);
