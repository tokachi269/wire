#include "backbone_plane.hpp"

wire::core::Vec3d ProjectBackbonePointToDisplayPlane(const wire::core::Vec3d& world) {
  wire::core::Vec3d flattened = world;
  flattened.z = kBackboneDisplayPlaneZ;
  return flattened;
}
