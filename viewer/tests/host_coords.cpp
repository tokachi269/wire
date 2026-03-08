#include "host_coords.hpp"
#include "registry.hpp"

namespace {

bool test_host_internal_coord_roundtrip_is_stable() {
  const Vector3 host{12.5f, -7.0f, 3.25f};
  const wire::core::Vec3d internal = HostWorldToInternal(host);
  const Vector3 roundtrip = InternalToHostWorld(internal);
  return roundtrip.x == host.x && roundtrip.y == host.y && roundtrip.z == host.z;
}

void register_host_coord_tests(viewer_test_registry::TestRegistry& tests) {
  viewer_test_registry::AddTest(tests, "V06", "Host/internal world coordinate boundary round-trips cleanly",
                                test_host_internal_coord_roundtrip_is_stable);
}

WIRE_REGISTER_VIEWER_TEST_SUITE(register_host_coord_tests);

} // namespace
