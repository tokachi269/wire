#include "host_coords.hpp"

city::wire::Vec3d HostWorldToInternal(const Vector3& host_world) {
  return {
      static_cast<double>(host_world.x),
      static_cast<double>(host_world.z),
      static_cast<double>(host_world.y),
  };
}

Vector3 InternalToHostWorld(const city::wire::Vec3d& internal_world) {
  return {
      static_cast<float>(internal_world.x),
      static_cast<float>(internal_world.z),
      static_cast<float>(internal_world.y),
  };
}
