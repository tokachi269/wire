#include "backbone_plane.hpp"

#include "city/wire/coord_utils.hpp"

city::wire::Vec3d ProjectBackbonePointToDisplayPlane(const city::wire::Vec3d& world) {
  city::wire::Vec3d flattened = world;
  city::wire::SetHeightAlongWorldUp(&flattened, kBackboneDisplayPlaneZ);
  return flattened;
}
