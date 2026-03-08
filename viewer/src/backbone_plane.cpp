#include "backbone_plane.hpp"

#include "wire/core/coord_utils.hpp"

wire::core::Vec3d ProjectBackbonePointToDisplayPlane(const wire::core::Vec3d& world) {
  wire::core::Vec3d flattened = world;
  wire::core::SetHeightAlongWorldUp(&flattened, kBackboneDisplayPlaneZ);
  return flattened;
}
