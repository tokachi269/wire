#include "backbone_plane.hpp"
#include "registry.hpp"
#include "city/wire/coord_utils.hpp"

namespace {

bool test_backbone_points_are_projected_to_zero_plane() {
  const city::wire::Vec3d projected = ProjectBackbonePointToDisplayPlane({12.5, -3.0, 7.25});
  return projected.x == 12.5 && projected.y == -3.0 &&
         city::wire::HeightAlongWorldUp(projected) == kBackboneDisplayPlaneZ;
}

void register_backbone_plane_tests(viewer_test_registry::TestRegistry& tests) {
  viewer_test_registry::AddTest(tests, "V05", "Backbone overlay/pick points stay on the shared 0m plane",
                                test_backbone_points_are_projected_to_zero_plane);
}

WIRE_REGISTER_VIEWER_TEST_SUITE(register_backbone_plane_tests);

} // namespace
