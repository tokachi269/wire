#pragma once

#include "wire/core/core_state.hpp"

namespace wire::core::generation::backbone {

struct DerivedBackboneRowKey {
  ObjectId node_id = kInvalidObjectId;
  ObjectId edge_a = kInvalidObjectId;
  ObjectId edge_b = kInvalidObjectId;

  bool operator==(const DerivedBackboneRowKey& other) const {
    return node_id == other.node_id && edge_a == other.edge_a &&
           edge_b == other.edge_b;
  }
};

struct EndpointRowRepresentation {
  DerivedBackboneRowKey row_key{};
  Vec3d row_axis{};
  double layout_yaw_deg = 0.0;
  bool connected = false;
  bool sharp = false;
  ObjectId peer_edge_bundle_id = kInvalidObjectId;
  std::size_t peer_lane_index = 0;
};

bool IsSharpBackboneInteriorAngle(double interior_angle_deg);

EditResult<EndpointRowRepresentation> DeriveEndpointRowRepresentation(
    const CoreState& state, const SavedBackbonePortBinding& binding);

} // namespace wire::core::generation::backbone
